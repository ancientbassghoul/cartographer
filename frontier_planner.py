"""frontier_planner.py — Map-mode goal selection + done verification (pure numpy; offline-testable).

Chooses the next frontier the drone should fly to, from `ground_grid.GroundGrid.frontiers()`, with:

  * UTILITY selection — prefer BIG, AHEAD, NEAR frontiers:
        util = size * max(behind_floor, cos(turn)) / (1 + dist_weight * dist)
    so it does NOT flip to a tiny frontier directly behind the drone (a frontier with cos(turn) < 0 is
    floored to `behind_floor`, i.e. chosen only when nothing better exists).
  * STRONG COMMITMENT — keep the committed goal (re-associating it to the nearest live frontier as the
    cluster centroid drifts while the map grows) until it is reached / gone, UNLESS another frontier's
    utility clearly beats the committed one (by `switch_factor`). Stops the goal thrash where the planner
    abandoned a good far goal every replan.
  * DONE VERIFICATION — when no frontiers remain, fly ONCE to the farthest known free corner (computed by
    the caller exactly once on the transition, cached here as a STATIC target — never re-evaluated while
    verifying, so it can't oscillate between equidistant corners) to look for uncharted territory; only
    declare `done` if, after reaching that corner, there are STILL no frontiers.
  * UNREACHABLE-GOAL BLACKLIST (progress-stall) — there is NO path planner: the drone flies a STRAIGHT
    LINE toward the goal bearing, so a goal behind glass / a wall / a corner can never be reached and its
    frontier is never consumed (the loop the operator hit at the glass window). A per-committed-goal
    progress watchdog measures the BEST (closest) distance achieved; while the drone is roughly AIMED at
    the goal yet that best distance fails to improve by `progress_eps` for `stall_s`, the goal is declared
    unreachable, its region is BLACKLISTED, and the planner reselects. The blacklist is POSITION-
    CONDITIONED — a region is excluded only while the drone is within `vantage_radius` of the spot it gave
    up from, so a different angle / a newly-opened route can still retry it (promoted to permanent after
    `blacklist_permanent_after` re-blacklists from the same vantage, to stop two dead goals ping-ponging).
    This is NOT a repeat-counter: healthy far goals (re-committed every replan, approached over many legs /
    parallax-scout steps) keep improving best-distance and are never blacklisted.

Transport-agnostic (mirrors ground_grid.py / map_store.py): plain values in, plain values out. No ZMQ,
no torch, no SLAM. HARD RULE (CLAUDE.md): every knob is a GENERAL planner param and every blacklist POINT
is computed LIVE from the drone's own failure to progress — the map/frontiers are built LIVE from SLAM;
nothing here encodes this room's answer.
"""

import argparse
import math

import numpy as np

from ground_grid import explore_cfg


def _wrap180(a):
    """Wrap an angle (deg) to (-180, 180]."""
    return (a + 180.0) % 360.0 - 180.0


class FrontierPlanner:
    """Stateful frontier goal chooser. `select(frontiers, pos, heading_deg, farthest_free)` is called
    once per replan with the LIVE frontier list; it owns the committed-goal + verification state."""

    def __init__(self, cfg: dict | None = None, **overrides):
        e = explore_cfg(cfg)
        g = lambda k, d: overrides.get(k, e.get(k, d))
        self.dist_weight = float(g("goal_dist_weight", 0.5))   # distance penalty in the utility
        self.behind_floor = float(g("goal_behind_floor", 0.15))  # utility floor for a frontier behind us
        self.switch_factor = float(g("goal_switch_factor", 1.5))  # abandon commitment only if beaten by this
        self.assoc_dist = float(g("goal_assoc_dist", 1.0))     # associate committed goal w/ a live frontier
        self.goal_reach_dist = float(g("goal_reach_dist", 0.4))  # verify-corner "reached" test
        self.verify_done = bool(g("verify_done", True))
        self.verify_min_dist = float(g("verify_min_dist", 0.6))  # skip verify if the far corner is already here
        # --- unreachable-goal progress-stall blacklist (general params + LIVE-computed points) ---
        # A goal is blacklisted only when GENUINELY WEDGED: aimed at it AND blocked ahead AND SLAM alive AND
        # armed (airborne) AND not progressing. Those gates keep the watchdog from firing on the ground
        # prelude or during SLAM-settle/HOLD pauses (where the drone is stationary but NOT failing to reach).
        self.stall_s = float(g("goal_stall_s", 6.0))          # wedged-but-no-progress seconds before blacklist
        self.progress_eps = float(g("goal_progress_eps", 0.2))  # min closing distance that counts as progress
        self.stall_aim_deg = float(g("goal_stall_aim_deg", 45.0))  # only accrue stall while ~aimed at the goal
        self.stall_clearance = float(g("goal_stall_clearance", 0.8))  # forward clearance <= this = "blocked ahead"
        self.stall_arm_dist = float(g("goal_stall_arm_dist", 0.5))  # move this far from start before the watchdog arms
        self.slam_slow_ms = float(g("slam_slow_ms", 1000.0))  # SLAM build time >= this = choking (a cooldown; don't accrue)
        self.blacklist_radius = float(g("goal_blacklist_radius", self.assoc_dist))  # "same region" as a dead goal
        self.vantage_radius = float(g("goal_vantage_radius", 1.0))  # give-up spot influence (position-conditioned)
        self.permanent_after = int(g("goal_blacklist_permanent_after", 3))  # re-blacklists -> promote to permanent
        self.committed_goal = None     # [x, z] currently committed, or None
        self.verifying = False         # True while flying to the cached far corner to confirm "done"
        self.verify_target = None      # frozen [x, z] of the corner during verification (NEVER recomputed)
        self._best_dist = None         # closest distance achieved toward the current committed goal
        self._stall_accum = 0.0        # accumulated WEDGED seconds (all gates true) — the stall clock
        self._last_now = None          # previous watchdog timestamp (for dt); None until first tick
        self._start_pos = None         # first pose the planner ever saw (drone start) — for the arm gate
        self._blacklist = []           # [{goal:[x,z], from_pos:[x,z]|None, count:int}] — LIVE dead-goal regions
        self.last_blacklist = None     # [x,z] blacklisted on THIS select() call (caller logs it once), else None

    @staticmethod
    def _d(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    # ---------------------------------------------------------------- blacklist
    def _blacklisted(self, center, pos):
        """True if `center` falls in a dead-goal region that is STILL active from `pos` (position-
        conditioned: a give-up region only excludes while the drone is near where it gave up; a
        promoted entry has from_pos=None and excludes everywhere)."""
        for e in self._blacklist:
            if self._d(center, e["goal"]) <= self.blacklist_radius and (
                    e["from_pos"] is None or self._d(pos, e["from_pos"]) <= self.vantage_radius):
                return True
        return False

    def _blacklist_goal(self, goal, pos):
        """Record `goal` as unreachable FROM near `pos`. Re-blacklisting the same region from ~the same
        vantage bumps a counter and, past `permanent_after`, promotes the entry to permanent (from_pos
        None) so two mutually-unreachable goals can't ping-pong the drone forever."""
        g = [float(goal[0]), float(goal[1])]
        p = [float(pos[0]), float(pos[1])]
        for e in self._blacklist:
            same_region = self._d(e["goal"], g) <= self.blacklist_radius
            same_vantage = e["from_pos"] is not None and self._d(e["from_pos"], p) <= self.vantage_radius
            if same_region and (e["from_pos"] is None or same_vantage):
                e["count"] += 1
                e["goal"] = g
                if e["from_pos"] is not None:
                    e["from_pos"] = (None if (self.permanent_after > 0 and e["count"] >= self.permanent_after)
                                     else p)
                self.last_blacklist = g
                return
        self._blacklist.append({"goal": g, "from_pos": p, "count": 1})
        self.last_blacklist = g

    def blacklist_points(self):
        """Public snapshot for telemetry/visualizer: [[x, z], ...] of every dead-goal region."""
        return [[e["goal"][0], e["goal"][1]] for e in self._blacklist]

    # ------------------------------------------------------------ progress stall
    def _reset_progress(self):
        self._best_dist = None
        self._stall_accum = 0.0

    def _watchdog(self, pos, heading_deg, now, forward_clearance, slam_ms):
        """Per-committed-goal WEDGED watchdog. Accrues stall time ONLY while the drone is genuinely stuck:
        armed (has left the start), aimed at the goal, blocked ahead (a wall within the stand-off), SLAM
        alive (not a cooldown), and not getting closer. The accumulator is PAUSED when any gate is false
        (so ground-prelude time and SLAM-settle/HOLD pauses don't count) and RESET on real progress. When
        the accumulated wedged time reaches `stall_s`, blacklist the goal FROM HERE and drop the commitment
        so the caller reselects."""
        if self._start_pos is None:                # remember the drone's start pose for the arm gate
            self._start_pos = [float(pos[0]), float(pos[1])]
        # dt since the last tick (clamped so a publish gap / first tick can't dump a huge chunk of time).
        dt = 0.0 if self._last_now is None else max(0.0, min(now - self._last_now, 1.0)) if now is not None else 0.0
        self._last_now = now
        if self.committed_goal is None or now is None:
            return
        d = self._d(pos, self.committed_goal)
        if self._best_dist is None or d <= self._best_dist - self.progress_eps:
            self._best_dist = d                    # real progress toward the goal -> reset the stall clock
            self._stall_accum = 0.0
            return
        # --- not progressing: accrue stall ONLY if ALL "genuinely wedged" gates hold ---
        armed = self._d(pos, self._start_pos) >= self.stall_arm_dist
        aimed = True
        if heading_deg is not None:
            dx, dz = self.committed_goal[0] - pos[0], self.committed_goal[1] - pos[1]
            if abs(dx) > 1e-9 or abs(dz) > 1e-9:
                bearing = math.degrees(math.atan2(dx, dz))
                aimed = abs(_wrap180(bearing - heading_deg)) <= self.stall_aim_deg
        blocked = forward_clearance is not None and forward_clearance <= self.stall_clearance
        slam_alive = slam_ms is None or slam_ms < self.slam_slow_ms
        if not (armed and aimed and blocked and slam_alive):
            return                                 # a gate failed -> PAUSE the clock (don't reset, don't fire)
        self._stall_accum += dt
        if self._stall_accum >= self.stall_s:
            self._blacklist_goal(self.committed_goal, pos)
            self.committed_goal = None
            self._reset_progress()

    def _commit(self, goal):
        """Commit to `goal`, restarting progress tracking only when it is a genuinely DIFFERENT region
        (a jump beyond `assoc_dist`) — small centroid drift under association keeps the same stall clock."""
        g = [float(goal[0]), float(goal[1])]
        if self.committed_goal is None or self._d(self.committed_goal, g) > self.assoc_dist:
            self._reset_progress()
        self.committed_goal = g

    def _utility(self, center, size, pos, heading_deg):
        dx, dz = center[0] - pos[0], center[1] - pos[1]
        dist = math.hypot(dx, dz)
        if heading_deg is None or (abs(dx) < 1e-9 and abs(dz) < 1e-9):
            turn_factor = 1.0
        else:
            bearing = math.degrees(math.atan2(dx, dz))     # 0 = +Z, +90 = +X (matches heading_from_pose)
            turn = abs(_wrap180(bearing - heading_deg))
            turn_factor = max(self.behind_floor, math.cos(math.radians(turn)))
        return size * turn_factor / (1.0 + self.dist_weight * dist)

    def _choose(self, frontiers, pos, heading_deg):
        centers = [(float(f["center"][0]), float(f["center"][1])) for f in frontiers]
        sizes = [float(f["size"]) for f in frontiers]
        utils = [self._utility(centers[i], sizes[i], pos, heading_deg) for i in range(len(frontiers))]
        best_i = int(np.argmax(utils))
        # COMMITMENT: if the committed goal still maps to a live frontier (within assoc_dist), keep it
        # unless a candidate's utility beats the committed one's by switch_factor.
        if self.committed_goal is not None:
            dlist = [self._d(self.committed_goal, c) for c in centers]
            j = int(np.argmin(dlist))
            if dlist[j] <= self.assoc_dist and utils[best_i] <= utils[j] * self.switch_factor:
                return centers[j]      # keep commitment, snapped to the live centroid
        return centers[best_i]

    def any_reachable(self, frontiers, pos):
        """True if at least one live frontier is NOT blacklisted-from-here. The caller uses this to decide
        whether to compute `farthest_free` (the reposition target when every frontier is dead-from-here)."""
        pos = (float(pos[0]), float(pos[1]))
        return any(not self._blacklisted((float(f["center"][0]), float(f["center"][1])), pos)
                   for f in frontiers)

    def select(self, frontiers, pos, heading_deg=None, farthest_free=None, now=None,
               forward_clearance=None, slam_ms=None):
        """Returns (goal [x,z] | None, n_frontiers, done). `now` (monotonic), `forward_clearance` and
        `slam_ms` drive the WEDGED watchdog (which blacklists only when armed+aimed+blocked+SLAM-alive+
        not-progressing; inert when `now` is None). `farthest_free` is consulted ONLY on the transition into
        verification (and the caller only computes it when no frontier is reachable-from-here)."""
        pos = (float(pos[0]), float(pos[1]))
        self.last_blacklist = None
        self._watchdog(pos, heading_deg, now, forward_clearance, slam_ms)  # may blacklist + clear the commitment

        # Reachable = live frontiers not blacklisted from the current vantage.
        reachable = [f for f in frontiers
                     if not self._blacklisted((float(f["center"][0]), float(f["center"][1])), pos)]
        if reachable:
            self.verifying = False                     # something to chase -> not done, not verifying
            self.verify_target = None
            goal = self._choose(reachable, pos, heading_deg)
            self._commit(goal)
            return self.committed_goal, len(frontiers), False

        # --- nothing reachable-from-here (no frontiers exist, OR all are blacklisted from this vantage) ---
        # Same path as "done verification": fly ONCE to the farthest free corner. That doubles as a
        # REPOSITION — moving away from the give-up vantage re-enables the position-conditioned blacklist,
        # so on arrival those frontiers become reachable again and exploration resumes.
        self.committed_goal = None
        self._reset_progress()
        if not self.verify_done:
            return None, len(frontiers), True
        if self.verifying:
            if self.verify_target is None or self._d(pos, self.verify_target) <= self.goal_reach_dist:
                self.verifying = False
                self.verify_target = None
                return None, len(frontiers), True
            return list(self.verify_target), len(frontiers), False
        # Transition INTO verifying: cache the far corner EXACTLY ONCE (caller computed it just now).
        if farthest_free is not None and self._d(pos, farthest_free) > self.verify_min_dist:
            self.verifying = True
            self.verify_target = [float(farthest_free[0]), float(farthest_free[1])]
            return list(self.verify_target), len(frontiers), False
        return None, len(frontiers), True              # nothing worth verifying -> truly done


# ==============================================================================
# Self-test: synthetic frontier lists (no hardware) — utility, commitment, static-target verification.
# ==============================================================================
def run_self_test():
    ok = True

    def check(name, cond):
        nonlocal ok
        ok = ok and bool(cond)
        print(f"[frontier_planner][self-test] {'PASS' if cond else 'FAIL'}  {name}")

    def close(a, b):
        return a is not None and b is not None and abs(a[0] - b[0]) < 1e-6 and abs(a[1] - b[1]) < 1e-6

    A = {"center": [0.0, 3.0], "size": 10}      # far ahead (+Z), modest size
    behind = {"center": [0.0, -1.0], "size": 5}  # near but directly behind

    # (a) prefer the ahead frontier over a nearer one that's behind us (no behind-flip).
    p = FrontierPlanner(None)
    goal, n, done = p.select([A, behind], [0.0, 0.0], heading_deg=0.0)
    check("(a) picks ahead frontier over a nearer behind one", close(goal, [0.0, 3.0]) and n == 2 and not done)

    # (b) commitment: once committed to A, a slightly-better near frontier does NOT steal the goal,
    #     but a dramatically-better one does.
    p = FrontierPlanner(None)
    p.select([A], [0.0, 0.0], heading_deg=0.0)                     # commit to A (util 10/2.5 = 4.0)
    c_slight = {"center": [0.0, 0.4], "size": 6}                   # util 6/1.2 = 5.0  (< 4.0*1.5=6.0)
    g2, _, _ = p.select([A, c_slight], [0.0, 0.0], heading_deg=0.0)
    check("(b) commitment holds vs a slightly-better frontier", close(g2, [0.0, 3.0]))
    c_big = {"center": [0.0, 0.4], "size": 12}                     # util 12/1.2 = 10.0 (> 6.0) -> switch
    g3, _, _ = p.select([A, c_big], [0.0, 0.0], heading_deg=0.0)
    check("(b) commitment yields to a much-better frontier", close(g3, [0.0, 0.4]))

    # (c) done verification: empty frontiers + a distant far corner -> go there (done=False); reaching
    #     it with still-empty frontiers -> done=True.
    p = FrontierPlanner(None)
    g, n, done = p.select([], [0.0, 0.0], farthest_free=[5.0, 0.0])
    check("(c) empty -> verify far corner, not done", close(g, [5.0, 0.0]) and not done and p.verifying)
    g, n, done = p.select([], [5.0, 0.0])                          # reached the corner, still empty
    check("(c) reached corner + empty -> done", g is None and done and not p.verifying)

    # (c2) STATIC target: while verifying, a moving drone (and a different farthest_free passed in) must
    #      keep returning the SAME cached corner — no oscillation.
    p = FrontierPlanner(None)
    p.select([], [0.0, 0.0], farthest_free=[5.0, 0.0])            # cache [5,0]
    g_a, _, _ = p.select([], [1.0, 0.0], farthest_free=[9.0, 9.0])
    g_b, _, _ = p.select([], [2.0, 2.0], farthest_free=[-9.0, -9.0])
    check("(c2) verify target stays frozen as the drone moves", close(g_a, [5.0, 0.0]) and close(g_b, [5.0, 0.0]))

    # (c3) frontiers reappearing mid-verify -> resume selection, no premature done.
    p = FrontierPlanner(None)
    p.select([], [0.0, 0.0], farthest_free=[5.0, 0.0])
    g, n, done = p.select([A], [1.0, 0.0], heading_deg=0.0)
    check("(c3) frontier reappears mid-verify -> resume, not done", g is not None and not done and not p.verifying)

    # (c4) nothing worth verifying -> done immediately; verify_done=False -> done immediately.
    p = FrontierPlanner(None)
    _, _, d_none = p.select([], [0.0, 0.0], farthest_free=None)
    _, _, d_near = FrontierPlanner(None).select([], [0.0, 0.0], farthest_free=[0.3, 0.0])  # within verify_min_dist
    _, _, d_off = FrontierPlanner(None, verify_done=False).select([], [0.0, 0.0], farthest_free=[5.0, 0.0])
    check("(c4) no corner / too-near / disabled -> done", d_none and d_near and d_off)

    # ---- unreachable-goal WEDGED blacklist (the glass-window loop) ----
    # A goal is blacklisted only when GENUINELY WEDGED: armed (moved from start) + aimed + blocked ahead
    # (small forward_clearance) + SLAM alive (slam_ms < slam_slow_ms) + not progressing. `wedged(...)` is a
    # select at the stand-off with all env gates open; ticks are ~1 s apart (dt clamps to 1 s).
    W = {"center": [0.0, 3.0], "size": 8}          # a goal we can never reach (behind glass/a wall)

    def wedged(planner, at, t, clr=0.5, slam=200.0, head=0.0, goal=W):
        return planner.select([goal], at, heading_deg=head, now=float(t), forward_clearance=clr, slam_ms=slam)

    # (d) a HEALTHY far goal — even with the env gates OPEN (blocked + SLAM alive + armed) — is NEVER
    #     blacklisted, because best-distance keeps IMPROVING (progress resets the accumulator every tick).
    p = FrontierPlanner(None)
    far = {"center": [0.0, 20.0], "size": 10}
    p.select([far], [0.0, 0.0], heading_deg=0.0, now=0.0)            # start=[0,0], commit
    for t in range(1, 15):                                           # 14 s > stall_s, but closing 1 unit each step
        gg, _, _ = wedged(p, [0.0, float(t)], t, goal=far)
    check("(d) healthy far goal (closing) never blacklists", len(p._blacklist) == 0 and close(gg, [0.0, 20.0]))

    # (e) a WEDGED goal is blacklisted once the accumulated wedged time reaches stall_s — but not before.
    pe = FrontierPlanner(None)                                      # kept for (g) — position-conditioned retry
    pe.select([W], [0.0, 0.0], heading_deg=0.0, now=0.0)            # start=[0,0], commit W
    wedged(pe, [0.0, 2.4], 0.5)                                     # approach to the stand-off (progress) -> best=0.6
    for t in (1.5, 2.5, 3.5, 4.5, 5.5):                            # 5 wedged ticks -> accum ~5 s (< stall_s)
        wedged(pe, [0.0, 2.4], t)
    before = len(pe._blacklist) == 0 and pe.committed_goal is not None
    g6, n6, _ = wedged(pe, [0.0, 2.4], 6.6)                        # accum crosses stall_s (6) -> blacklist
    check("(e) wedged goal blacklists at stall_s (not before)",
          before and len(pe._blacklist) == 1 and close(pe.last_blacklist, [0.0, 3.0]) and n6 == 1)

    # (f) AIM gate: armed + blocked + SLAM alive, but bearing err > aim window (still turning to FACE the
    #     goal — legit parallax scout) -> no accrual, never blacklisted.
    p = FrontierPlanner(None)
    side = {"center": [10.0, 0.0], "size": 8}
    p.select([side], [0.0, 0.0], heading_deg=0.0, now=0.0)          # start
    for t in range(1, 12):
        wedged(p, [3.0, 0.0], t, head=0.0, goal=side)              # at [3,0] bearing to [10,0]=90 deg > 45 -> not aimed
    check("(f) not-aimed (still scouting) never stalls", len(p._blacklist) == 0 and p.committed_goal is not None)

    # (g) POSITION-CONDITIONED: the goal blacklisted in (e) from ~[0,2.4] is reachable again from a far
    #     vantage, but still excluded from near the give-up spot. (reuses pe, which holds that blacklist.)
    g_far, _, _ = pe.select([W], [0.0, -10.0], heading_deg=0.0, now=8.0)
    g_near, _, _ = pe.select([W], [0.0, 2.5], heading_deg=0.0, now=9.0)
    check("(g) blacklist is position-conditioned (far=retry, near=excluded)",
          close(g_far, [0.0, 3.0]) and g_near is None)

    # (d2) GATE SUPPRESSION — each missing gate alone prevents a blacklist even over a long window:
    #      not-armed (never moved from start), not-blocked (clearance large), SLAM slow (a cooldown).
    p_arm = FrontierPlanner(None)                                   # start == wedge pos -> never armed
    for t in range(0, 12):
        wedged(p_arm, [0.0, 2.4], t)                               # start=[0,2.4]; dist-from-start = 0 < arm_dist
    p_blk = FrontierPlanner(None)
    p_blk.select([W], [0.0, 0.0], heading_deg=0.0, now=0.0)
    for t in range(1, 12):
        wedged(p_blk, [0.0, 2.4], t, clr=5.0)                     # armed + aimed but NOT blocked (clearance large)
    p_slam = FrontierPlanner(None)
    p_slam.select([W], [0.0, 0.0], heading_deg=0.0, now=0.0)
    for t in range(1, 12):
        wedged(p_slam, [0.0, 2.4], t, slam=1500.0)                # armed + aimed + blocked but SLAM choking (cooldown)
    check("(d2) not-armed / not-blocked / SLAM-slow each SUPPRESS the blacklist",
          len(p_arm._blacklist) == 0 and len(p_blk._blacklist) == 0 and len(p_slam._blacklist) == 0)

    # (h) re-blacklisting the same region from ~the same vantage promotes it to PERMANENT (from_pos None
    #     -> excluded everywhere) so two dead goals can't ping-pong the drone forever.
    p = FrontierPlanner(None)                        # permanent_after = 3
    for _ in range(3):
        p._blacklist_goal([1.0, 1.0], [0.0, 0.0])
    check("(h) promoted to permanent after N re-blacklists",
          p._blacklist[0]["from_pos"] is None and p._blacklisted([1.0, 1.0], [99.0, 99.0]))

    # (i) when EVERY live frontier is dead-from-here, route to the far-corner reposition (not a crash, not
    #     a premature done); the true frontier count is still reported.
    p = FrontierPlanner(None)
    p._blacklist_goal([0.0, 3.0], [0.0, 2.4])
    g_v, n_v, done_v = p.select([W], [0.0, 2.4], heading_deg=0.0, farthest_free=[5.0, 5.0], now=0.0)
    check("(i) all-dead-from-here -> reposition via far corner, frontiers still reported, not done",
          close(g_v, [5.0, 5.0]) and n_v == 1 and not done_v and p.verifying)

    print(f"\n[frontier_planner][self-test] {'ALL PASS' if ok else 'FAILURES PRESENT'}")
    return ok


def main():
    ap = argparse.ArgumentParser(description="Frontier goal planner (utility + commitment + done-verify + blacklist)")
    ap.add_argument("--self-test", action="store_true", help="run the synthetic self-test (no hardware)")
    args = ap.parse_args()
    if args.self_test:
        raise SystemExit(0 if run_self_test() else 1)
    ap.print_help()


if __name__ == "__main__":
    main()
