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
  * UNREACHABLE-GOAL BLACKLIST (progress-stall, PERMANENT round-based) — there is NO path planner: the
    drone flies a STRAIGHT LINE toward the goal bearing, so a goal behind glass / a wall / a corner can
    never be reached and its frontier is never consumed (the loop the operator hit at the glass window). A
    per-committed-goal progress watchdog measures the BEST (closest) distance achieved; while the drone is
    roughly AIMED at the goal yet that best distance fails to improve by `progress_eps` for `stall_s`, the
    goal is declared unreachable, its region is BLACKLISTED, and the planner reselects. The blacklist is
    POSITION-UNCONDITIONED: once blacklisted a goal STAYS excluded — it is not silently re-enabled by the
    drone merely moving away (that position-conditioning was the ping-pong: goal A blacklisted, drone
    starts toward B, moving off the give-up spot re-whitelisted A, drone turns back, wedges, repeat). A
    blacklisted goal is whitelisted only when we have "been over all other goals" — every live frontier is
    excluded — at which point the planner REPOSITIONS to the farthest free corner and, on arrival, clears
    the round's soft blacklist so the goals get one retry from a genuinely new vantage. CONVERGENCE: a goal
    blacklisted AGAIN in a later round WITHOUT ever having gotten closer (best-distance never improved past
    its prior best) is promoted to PERMANENT and never whitelisted again — so each truly-dead goal gets at
    most ~2 real attempts, then drops out for good (no endless A->B->A cycle). This is NOT a repeat-counter:
    healthy far goals (re-committed every replan, approached over many legs / parallax-scout steps) keep
    improving best-distance and are never blacklisted.

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
        # --- unreachable-goal DISTANCE-STAGNATION watchdog (general params + LIVE-computed points) ---
        # A committed goal is blacklisted when the best-ever Euclidean distance toward it fails to improve by
        # progress_eps for `stagnation_s` of SLAM-HEALTHY time. This single stagnation timer catches BOTH an
        # invisible GLASS collider (SLAM stays healthy + the path reads clear, but the drone can't get closer —
        # an invisible treadmill) AND an opaque wall the drone rams then re-commits (the pos plateaus at the
        # wall across ram/recover cycles). No aim/clear/advancing gates: nothing about clearance or thrust can
        # hide a goal that simply never gets closer. Ticks only on VALID (TRACKING) frames (perception skips
        # select() otherwise), so a full SLAM loss naturally pauses it; a slam_ms CHOKE also pauses (unstable
        # pose). The window is generous so slow flight + noisy scout maneuvers don't clip a reachable goal.
        self.stagnation_s = float(g("goal_stagnation_s", 60.0))   # SLAM-healthy seconds of no progress -> blacklist
        self.progress_eps = float(g("goal_progress_eps", 0.2))    # min closing distance that counts as progress
        self.slam_slow_ms = float(g("slam_slow_ms", 1000.0))      # SLAM build time >= this = choking -> PAUSE (don't accrue)
        self.blacklist_radius = float(g("goal_blacklist_radius", self.assoc_dist))  # "same region" as a dead goal
        self.committed_goal = None     # [x, z] currently committed, or None
        self.verifying = False         # True while flying to the cached far corner to confirm "done"
        self.verify_target = None      # frozen [x, z] of the corner during verification (NEVER recomputed)
        self._best_dist = None         # closest distance achieved toward the current committed goal
        self._stagnation_accum = 0.0   # accumulated SLAM-healthy seconds with no progress toward the goal
        self._last_now = None          # previous watchdog timestamp (for dt); None until first tick
        # Position-UNCONDITIONED dead-goal regions. Each: {goal:[x,z], best_ever:float, permanent:bool,
        # active:bool}. `active` = excluded THIS round (cleared by _whitelist_round on the reposition
        # arrival); `permanent` = excluded forever (a goal re-blacklisted with no cross-round progress).
        # `best_ever` = the closest distance ever achieved toward the region (persists across rounds).
        self._blacklist = []
        self.last_blacklist = None     # [x,z] blacklisted on THIS select() call (caller logs it once), else None

    @staticmethod
    def _d(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    # ---------------------------------------------------------------- blacklist
    def _excluded(self, center):
        """True if `center` falls in a dead-goal region that is currently excluded — PERMANENT (forever)
        or ACTIVE (this round). Position-UNCONDITIONED: a blacklisted goal is not re-enabled by the drone
        moving away (that was the ping-pong); a soft entry clears only via _whitelist_round on reposition."""
        for e in self._blacklist:
            if self._d(center, e["goal"]) <= self.blacklist_radius and (e["permanent"] or e["active"]):
                return True
        return False

    def _blacklist_goal(self, goal, permanent=False):
        """Record `goal` as unreachable, using the best (closest) distance reached this commit
        (`self._best_dist`). `permanent=True` (the stagnation watchdog) marks it dead FOR GOOD immediately —
        the drone spent the full stagnation window unable to get closer, so it is truly unreachable. Otherwise
        (re-blacklisting the same region): if we NEVER got closer than a prior round (no cross-round progress)
        -> promote to PERMANENT (two dead goals can't cycle the drone forever); if we DID get closer (a new
        route opened) -> keep it soft/retryable. A first-ever soft blacklist stays soft."""
        g = [float(goal[0]), float(goal[1])]
        best_now = self._best_dist if self._best_dist is not None else float("inf")
        for e in self._blacklist:
            if self._d(e["goal"], g) <= self.blacklist_radius:
                improved = best_now < e["best_ever"] - self.progress_eps
                e["best_ever"] = min(e["best_ever"], best_now)
                e["goal"] = g
                e["active"] = True
                if permanent or not improved:
                    e["permanent"] = True          # forced, or re-dead with no cross-round progress -> for good
                self.last_blacklist = g
                return
        self._blacklist.append({"goal": g, "best_ever": best_now, "permanent": bool(permanent), "active": True})
        self.last_blacklist = g

    def _whitelist_round(self):
        """Clear the round's SOFT exclusions (set active=False on every non-permanent entry) so the
        surviving goals get one retry from the new vantage. Permanent entries and best_ever memory stay."""
        for e in self._blacklist:
            if not e["permanent"]:
                e["active"] = False

    def blacklist_points(self):
        """Public snapshot for telemetry/visualizer: [[x, z], ...] of every dead-goal region (soft+permanent)."""
        return [[e["goal"][0], e["goal"][1]] for e in self._blacklist]

    def blacklist_permanent(self):
        """Parallel [bool, ...] flag list (matches blacklist_points order) so the visualizer can mark a
        'dead for good' region distinctly from a 'dead this round' one."""
        return [bool(e["permanent"]) for e in self._blacklist]

    # --------------------------------------------------- distance-stagnation watchdog
    def _reset_progress(self):
        self._best_dist = None
        self._stagnation_accum = 0.0

    def _watchdog(self, pos, now, slam_ms):
        """Per-committed-goal DISTANCE-STAGNATION watchdog. Tracks the best-ever distance toward the goal and
        accrues SLAM-healthy time in which that best distance fails to improve by `progress_eps`; at
        `stagnation_s` the goal is declared UNREACHABLE and PERMANENTLY blacklisted (the drone spent the whole
        window unable to get closer -> a glass collider or a rammed wall behind the goal). Only ticks on VALID
        frames (perception skips select() when not TRACKING), so a full SLAM loss pauses it; a slam_ms CHOKE
        (unstable pose) also PAUSES accrual. Real progress RESETS the clock. No aim/clear/thrust gates — a
        goal that simply never gets closer is unreachable however the sensors read."""
        # dt since the last tick (clamped so a publish gap / first tick can't dump a huge chunk of time).
        dt = 0.0 if self._last_now is None else max(0.0, min(now - self._last_now, 1.0)) if now is not None else 0.0
        self._last_now = now
        if self.committed_goal is None or now is None:
            return
        d = self._d(pos, self.committed_goal)
        if self._best_dist is None or d <= self._best_dist - self.progress_eps:
            self._best_dist = d                    # real progress toward the goal -> reset the stagnation clock
            self._stagnation_accum = 0.0
            return
        # No progress. Accrue wall-clock time only while SLAM is HEALTHY (a choke gives an unstable pose we
        # must not trust); a full loss doesn't tick here at all. No other gates.
        if slam_ms is None or slam_ms < self.slam_slow_ms:
            self._stagnation_accum += dt
        if self._stagnation_accum >= self.stagnation_s:
            self._blacklist_goal(self.committed_goal, permanent=True)
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

    def any_reachable(self, frontiers):
        """True if at least one live frontier is NOT excluded (soft or permanent). The caller uses this to
        decide whether to compute `farthest_free` (the reposition target when every frontier is dead)."""
        return any(not self._excluded((float(f["center"][0]), float(f["center"][1]))) for f in frontiers)

    def _select_reachable(self, frontiers, pos, heading_deg):
        """Choose + commit among the currently non-excluded frontiers, clearing any verify state."""
        reachable = [f for f in frontiers
                     if not self._excluded((float(f["center"][0]), float(f["center"][1])))]
        if not reachable:
            return None
        self.verifying = False                         # something to chase -> not done, not verifying
        self.verify_target = None
        goal = self._choose(reachable, pos, heading_deg)
        self._commit(goal)
        return self.committed_goal

    def select(self, frontiers, pos, heading_deg=None, farthest_free=None, now=None, slam_ms=None):
        """Returns (goal [x,z] | None, n_frontiers, done). `now` (monotonic) + `slam_ms` drive the
        distance-stagnation watchdog (blacklists a committed goal that never gets closer for `stagnation_s` of
        SLAM-healthy time; inert when `now` is None). `farthest_free` is consulted ONLY on the transition into
        verification/reposition (and the caller only computes it when no frontier is reachable)."""
        pos = (float(pos[0]), float(pos[1]))
        self.last_blacklist = None
        self._watchdog(pos, now, slam_ms)          # may blacklist + clear the commitment

        goal = self._select_reachable(frontiers, pos, heading_deg)
        if goal is not None:
            return goal, len(frontiers), False

        # --- nothing reachable: no frontiers exist, OR every live frontier is excluded ("been over all
        # goals"). Fly ONCE to the farthest free corner (a fresh vantage); this doubles as done-verification
        # (empty frontiers) AND a REPOSITION-then-retry (all excluded) — on arrival the round's soft
        # blacklist is cleared and the surviving goals get one retry from there.
        self.committed_goal = None
        self._reset_progress()
        if not self.verify_done:
            return None, len(frontiers), True
        if self.verifying:
            if self.verify_target is None or self._d(pos, self.verify_target) <= self.goal_reach_dist:
                self.verifying = False
                self.verify_target = None
                if frontiers:                          # they were all excluded -> whitelist the round + retry
                    self._whitelist_round()
                    g = self._select_reachable(frontiers, pos, heading_deg)
                    if g is not None:
                        return g, len(frontiers), False
                return None, len(frontiers), True      # no live frontiers -> verification complete = done
            return list(self.verify_target), len(frontiers), False
        # Transition INTO reposition/verify: cache the (inset) far corner EXACTLY ONCE (caller computed it).
        if farthest_free is not None and self._d(pos, farthest_free) > self.verify_min_dist:
            self.verifying = True
            self.verify_target = [float(farthest_free[0]), float(farthest_free[1])]
            return list(self.verify_target), len(frontiers), False
        # No corner worth repositioning to (no free space beyond verify_min_dist). If frontiers exist (all
        # excluded) whitelist + retry IN PLACE — but NOT on the same tick we just blacklisted one: let the
        # exclusion stand a tick so we don't instantly re-commit the goal we just killed (cross-round
        # no-progress then promotes it to permanent). If frontiers exist but we can't retry yet, idle (not
        # done); only a truly empty frontier list is "done".
        if frontiers and self.last_blacklist is None:
            self._whitelist_round()
            g = self._select_reachable(frontiers, pos, heading_deg)
            if g is not None:
                return g, len(frontiers), False
        if frontiers:
            return None, len(frontiers), False
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

    # ---- unreachable-goal DISTANCE-STAGNATION blacklist (glass collider OR rammed opaque wall) ----
    # A committed goal is PERMANENTLY blacklisted when its best-ever distance fails to improve by progress_eps
    # for stagnation_s of SLAM-HEALTHY time. No aim/clear/thrust gates. `stag(...)` is a select at a fixed
    # stand-off; ticks are ~1 s apart (dt clamps to 1 s). SLAM-slow ticks PAUSE (don't accrue).
    W = {"center": [0.0, 3.0], "size": 8}          # a goal we can never reach (behind glass / a wall)

    def stag(planner, at, t, slam=200.0, head=0.0, goal=W):
        return planner.select([goal], at, heading_deg=head, now=float(t), slam_ms=slam)

    # (d) a HEALTHY far goal that keeps CLOSING is never blacklisted (progress resets the clock every tick).
    p = FrontierPlanner(None)
    far = {"center": [0.0, 80.0], "size": 10}
    p.select([far], [0.0, 0.0], heading_deg=0.0, now=0.0)           # commit
    for t in range(1, 70):                                          # 69 s > stagnation_s, but closing 1 unit/step
        gg, _, _ = stag(p, [0.0, float(t)], t, goal=far)
    check("(d) healthy far goal (closing) never blacklists", len(p._blacklist) == 0 and close(gg, [0.0, 80.0]))

    # (e) a STAGNATING goal is PERMANENTLY blacklisted once the accumulated no-progress time reaches
    #     stagnation_s — but not before. (Held at a fixed stand-off; best distance never improves.)
    pe = FrontierPlanner(None)                                     # kept for (g) — position-UNconditioned check
    pe.select([W], [0.0, 0.0], heading_deg=0.0, now=0.0)           # commit W
    stag(pe, [0.0, 2.4], 0.5)                                      # approach to the stand-off -> best=0.6
    S = int(pe.stagnation_s)
    for t in range(1, S - 3):                                      # well under stagnation_s (dt clamps to 1/tick)
        stag(pe, [0.0, 2.4], float(t))
    before = len(pe._blacklist) == 0 and pe.committed_goal is not None
    for t in range(S - 3, S + 4):                                  # cross stagnation_s
        stag(pe, [0.0, 2.4], float(t))
    check("(e) stagnating goal PERMANENT-blacklists at stagnation_s (not before)",
          before and len(pe._blacklist) == 1 and pe._blacklist[0]["permanent"] is True
          and close(pe._blacklist[0]["goal"], [0.0, 3.0]))

    # (f) SLAM-CHOKE PAUSE: a stagnating goal with slam_ms >= slam_slow_ms EVERY tick never accrues -> never
    #     blacklisted, even over a long window (unstable pose must not be trusted).
    p = FrontierPlanner(None)
    p.select([W], [0.0, 0.0], heading_deg=0.0, now=0.0)
    for t in range(1, int(p.stagnation_s) + 10):
        stag(p, [0.0, 2.4], float(t), slam=1500.0)                # SLAM choking every tick -> PAUSE
    check("(f) SLAM-choke every tick PAUSES the clock (never blacklists)",
          len(p._blacklist) == 0 and p.committed_goal is not None)

    # (g) POSITION-UNCONDITIONED (the ping-pong fix): the goal blacklisted in (e) stays excluded no matter
    #     where the drone moves — moving away does NOT silently re-whitelist it. (reuses pe.)
    check("(g) blacklist stays excluded regardless of vantage (no re-whitelist by moving)",
          pe._excluded([0.0, 3.0]) and not pe.any_reachable([W]))

    # (h) OPAQUE-WALL RAM loop: the drone rams, SLAM dies (loss = no tick), recovers TRACKING at the same
    #     wall stand-off, rams again — best distance plateaus. Accrual happens on the healthy stretches
    #     (loss gaps just don't tick), so it still reaches stagnation_s and blacklists.
    pr = FrontierPlanner(None)
    pr.select([W], [0.0, 0.0], heading_deg=0.0, now=0.0)
    stag(pr, [0.0, 2.4], 0.5)                                      # first approach -> best=0.6
    t = 1.0
    for _cycle in range(int(pr.stagnation_s) + 5):                # healthy ticks at the wall; big time gaps between
        stag(pr, [0.0, 2.4], t)                                   # (loss gap) then a fresh healthy tick — dt clamps to 1
        t += 3.0                                                  # 3 s of wall-clock but only 1 s credited (clamp)
    check("(h) opaque-wall ram/recover (plateau on healthy stretches) still blacklists",
          len(pr._blacklist) == 1 and pr._blacklist[0]["permanent"] is True)

    # (k) CROSS-ROUND CONVERGENCE (soft re-blacklist path): a SOFT goal re-blacklisted in a later round
    #     WITHOUT ever having gotten closer is promoted to PERMANENT (survives a whitelist); but if a new
    #     vantage DID close the distance, it stays soft (retryable). (permanent=False default path.)
    p = FrontierPlanner(None)
    p._best_dist = 2.0
    p._blacklist_goal([1.0, 1.0])                     # round 1: soft, best_ever=2.0
    soft_ok = p._blacklist[0]["permanent"] is False and p._blacklist[0]["active"] is True
    p._whitelist_round()                              # "been over all goals" -> retry round
    inactive_ok = p._blacklist[0]["active"] is False and p._excluded([1.0, 1.0]) is False
    p._best_dist = 2.0                                # round 2: no closer than before (no progress)
    p._blacklist_goal([1.0, 1.0])
    check("(k) re-dead with no cross-round progress -> PERMANENT (survives whitelist)",
          soft_ok and inactive_ok and p._blacklist[0]["permanent"] is True
          and (p._whitelist_round() or p._excluded([1.0, 1.0])))
    p2 = FrontierPlanner(None)                        # WITH progress -> stays soft/retryable
    p2._best_dist = 2.0
    p2._blacklist_goal([1.0, 1.0])
    p2._whitelist_round()
    p2._best_dist = 1.5                               # got 0.5 closer (> progress_eps) from the new vantage
    p2._blacklist_goal([1.0, 1.0])
    check("(k) re-dead but progressed -> stays soft (best_ever updated, retryable)",
          p2._blacklist[0]["permanent"] is False and abs(p2._blacklist[0]["best_ever"] - 1.5) < 1e-9)

    # (i) when EVERY live frontier is excluded, route to the (inset) far-corner reposition (not a crash,
    #     not a premature done); on ARRIVAL the round's soft blacklist clears and the frontier is retried.
    p = FrontierPlanner(None)
    p._best_dist = 0.6
    p._blacklist_goal([0.0, 3.0])                     # W soft-excluded
    g_v, n_v, done_v = p.select([W], [0.0, 2.4], heading_deg=0.0, farthest_free=[5.0, 5.0], now=0.0)
    check("(i) all-excluded -> reposition via far corner, frontiers still reported, not done",
          close(g_v, [5.0, 5.0]) and n_v == 1 and not done_v and p.verifying)
    g_r, n_r, done_r = p.select([W], [5.0, 5.0], heading_deg=0.0, now=1.0)   # reached the corner
    check("(i) reached corner -> whitelist the round + retry the frontier (not done)",
          close(g_r, [0.0, 3.0]) and n_r == 1 and not done_r and not p.verifying)

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
