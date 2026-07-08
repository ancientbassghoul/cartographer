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
  * UNREACHABLE-GOAL BLACKLIST (EVENT-DRIVEN 2-BUMP, PERMANENT) — there is NO path planner: the drone
    flies a STRAIGHT LINE toward the goal bearing, so a goal behind glass / a wall / a corner can never be
    reached and its frontier is never consumed (the loop the operator hit at the glass window). The autopilot
    reports each DISCRETE advance-blocked stop (optical-flow WALL contact, ram-guard, or clearance stand-off)
    as a "bump" pulse via `note_wall_hit(goal)`; TWO bumps on the SAME goal region (within `assoc_dist`) ⇒ the
    goal is unreachable ⇒ PERMANENTLY blacklisted, the commitment dropped, and the planner reselects. A bump
    on a DIFFERENT goal resets the counter (so only consecutive same-region bumps accumulate). This is
    event-driven ON PURPOSE: a prior time-accumulation watchdog gated its clock on SLAM-healthy frames and so
    went BLIND exactly in the heavy glass/wall pockets (SLAM runs hot while the drone still flies on valid
    poses), never firing. Counting hard physical stops sidesteps SLAM-clock health entirely. A kinematic latch
    in the autopilot (displacement-or-retreat re-arm) guarantees one continuous contact counts as ONE bump.
    The blacklist store is POSITION-UNCONDITIONED and PERMANENT: once blacklisted a goal STAYS excluded (the
    round-based whitelist/reposition machinery below remains for the all-frontiers-excluded reposition, but
    2-bump entries are permanent and never whitelisted).

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
        # --- unreachable-goal EVENT-DRIVEN 2-BUMP blacklist (general params + LIVE-computed points) ---
        # There is NO path planner: the drone flies a STRAIGHT line to the goal bearing, so a goal behind an
        # invisible GLASS collider (the drone rides an invisible treadmill) or behind an opaque wall is never
        # reachable. We do NOT time-accumulate stagnation (a prior watchdog gated accrual on SLAM-healthy time,
        # so it went BLIND exactly in the heavy glass/wall pockets where SLAM runs hot yet the drone still flies
        # on valid poses). Instead the autopilot reports each DISCRETE advance-blocked stop (flow WALL / ram
        # guard / clearance stand-off) as a "bump" pulse; TWO bumps on the SAME goal region ⇒ the goal is
        # unreachable ⇒ PERMANENTLY blacklisted. Event-driven, so it is immune to the SLAM-clock health that
        # defeated the timer. A kinematic latch in the autopilot ensures one continuous contact = one bump.
        self.progress_eps = float(g("goal_progress_eps", 0.2))    # min closing distance counted as progress (round-blacklist promotion)
        self.blacklist_radius = float(g("goal_blacklist_radius", self.assoc_dist))  # "same region" as a dead goal
        self.committed_goal = None     # [x, z] currently committed, or None
        self.verifying = False         # True while flying to the cached far corner to confirm "done"
        self.verify_target = None      # frozen [x, z] of the corner during verification (NEVER recomputed)
        self._best_dist = None         # closest distance achieved toward the current committed goal (round-blacklist memory)
        self._last_wall_hit_goal = None  # [x, z] of the goal the last counted bump was against, or None
        self._wall_hit_count = 0         # consecutive advance-blocked bumps on _last_wall_hit_goal (>=2 -> blacklist)
        self.last_bump = None            # outcome dict of the most recent note_wall_hit (caller logs it once), else None
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

    # --------------------------------------------------- event-driven 2-bump blacklist
    def _reset_progress(self):
        self._best_dist = None

    @property
    def wall_hit_count(self):
        """Live bump count on the currently-tracked goal region (0 when idle, 1 after a first bump)."""
        return self._wall_hit_count

    @property
    def wall_hit_goal(self):
        """[x,z] the current bump counter is tracking (the last counted bump's goal), or None."""
        return list(self._last_wall_hit_goal) if self._last_wall_hit_goal is not None else None

    def note_wall_hit(self, goal):
        """Register ONE advance-blocked "bump" (flow WALL / ram-guard / stand-off) against `goal`, reported by
        the autopilot. Consecutive bumps on the SAME goal region (within `assoc_dist`) accumulate; a bump on a
        DIFFERENT goal resets the counter (so only genuinely-repeated same-goal contacts count). The SECOND bump
        declares the goal UNREACHABLE and PERMANENTLY blacklists it, drops the commitment, and resets the
        counter (the next select() reselects around the dead region). Event-driven — no timer, no SLAM-health
        gate. The autopilot's kinematic latch guarantees a single continuous contact is only one bump, so this
        never fires on state-machine flicker. Sets `last_blacklist` (the [x,z] just blacklisted, else None) and
        returns/stashes `last_bump`, an outcome dict {goal, count, threshold, action, prev_goal}, so the caller
        can log EVERY bump (not just blacklists) — making the goal-change counter resets visible."""
        self.last_blacklist = None
        g = [float(goal[0]), float(goal[1])]
        prev_goal = list(self._last_wall_hit_goal) if self._last_wall_hit_goal is not None else None
        if self._last_wall_hit_goal is not None and self._d(g, self._last_wall_hit_goal) <= self.assoc_dist:
            self._wall_hit_count += 1
            action = "increment"
        else:
            self._wall_hit_count = 1
            self._last_wall_hit_goal = g
            # "reset" only when a DIFFERENT prior goal was displaced; a first-ever bump just arms the counter.
            action = "reset" if prev_goal is not None else "arm"
        count_at_hit = self._wall_hit_count      # count reached BY this bump (before any blacklist zeroing)
        if self._wall_hit_count >= 2:
            self._blacklist_goal(g, permanent=True)
            if self.committed_goal is not None and self._d(self.committed_goal, g) <= self.blacklist_radius:
                self.committed_goal = None         # drop the dead commitment -> next select() reselects
                self._reset_progress()
            self._wall_hit_count = 0
            self._last_wall_hit_goal = None
            action = "blacklist"
        self.last_bump = {"goal": g, "count": count_at_hit, "threshold": 2,
                          "action": action, "prev_goal": prev_goal}
        return self.last_bump

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

    def select(self, frontiers, pos, heading_deg=None, farthest_free=None):
        """Returns (goal [x,z] | None, n_frontiers, done). Unreachable-goal blacklisting is event-driven now
        (`note_wall_hit`, fed by the autopilot's advance-blocked stops) — NOT a per-select timer. `farthest_free`
        is consulted ONLY on the transition into verification/reposition (and the caller only computes it when
        no frontier is reachable)."""
        pos = (float(pos[0]), float(pos[1]))
        self.last_blacklist = None

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

    # ---- unreachable-goal EVENT-DRIVEN 2-BUMP blacklist (glass collider / rammed opaque wall) ----
    # The autopilot reports each advance-blocked stop via note_wall_hit(goal); TWO bumps on the SAME goal
    # region PERMANENTLY blacklist it. A bump on a DIFFERENT goal resets the counter. No timer, no SLAM gate.
    W = {"center": [0.0, 3.0], "size": 8}          # a goal we can never reach (behind glass / a wall)

    # (d) a SINGLE bump never blacklists (a lone contact could be transient); the counter just arms at 1.
    p = FrontierPlanner(None)
    p.select([W], [0.0, 0.0], heading_deg=0.0)     # commit W
    b_d = p.note_wall_hit([0.0, 3.0])
    check("(d) one bump does NOT blacklist (armed at 1)",
          len(p._blacklist) == 0 and p._wall_hit_count == 1 and p.committed_goal is not None
          and p.wall_hit_count == 1 and close(p.wall_hit_goal, [0.0, 3.0])
          and b_d == p.last_bump and b_d["action"] == "arm" and b_d["count"] == 1
          and b_d["prev_goal"] is None)

    # (e) the SECOND bump on the same goal region (within assoc_dist, so centroid drift still counts)
    #     PERMANENTLY blacklists it, drops the commitment, and resets the counter.
    pe = FrontierPlanner(None)                                     # reused by (g)
    pe.select([W], [0.0, 0.0], heading_deg=0.0)                    # commit W
    pe.note_wall_hit([0.0, 3.0])
    b_e = pe.note_wall_hit([0.0, 2.95])                            # same region (< assoc_dist) -> 2nd bump
    check("(e) two same-region bumps PERMANENT-blacklist + clear commitment + reset count",
          len(pe._blacklist) == 1 and pe._blacklist[0]["permanent"] is True
          and close(pe._blacklist[0]["goal"], [0.0, 2.95]) and pe.committed_goal is None
          and pe._wall_hit_count == 0 and pe.wall_hit_goal is None
          and b_e["action"] == "blacklist" and b_e["count"] == 2)

    # (f) GOAL-CHANGE RESET: a bump on a goal > assoc_dist away resets the counter, so alternating bumps on
    #     two far-apart goals never reach 2 (only consecutive same-region contacts accumulate).
    p = FrontierPlanner(None)
    p.note_wall_hit([0.0, 3.0])                                    # goal A -> count 1
    p.note_wall_hit([9.0, 9.0])                                    # goal B (far) -> reset, count 1 on B
    b_f = p.note_wall_hit([0.0, 3.0])                              # back to A -> reset, count 1 on A
    check("(f) a different-goal bump RESETS the counter (no blacklist)",
          len(p._blacklist) == 0 and p._wall_hit_count == 1 and close(p._last_wall_hit_goal, [0.0, 3.0])
          and b_f["action"] == "reset" and b_f["count"] == 1 and close(b_f["prev_goal"], [9.0, 9.0]))

    # (g) POSITION-UNCONDITIONED + PERMANENT: the goal blacklisted in (e) stays excluded from any vantage.
    check("(g) blacklist stays excluded regardless of vantage",
          pe._excluded([0.0, 3.0]) and not pe.any_reachable([W]))

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
    g_v, n_v, done_v = p.select([W], [0.0, 2.4], heading_deg=0.0, farthest_free=[5.0, 5.0])
    check("(i) all-excluded -> reposition via far corner, frontiers still reported, not done",
          close(g_v, [5.0, 5.0]) and n_v == 1 and not done_v and p.verifying)
    g_r, n_r, done_r = p.select([W], [5.0, 5.0], heading_deg=0.0)   # reached the corner
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
