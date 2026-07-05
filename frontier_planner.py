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

Transport-agnostic (mirrors ground_grid.py / map_store.py): plain values in, plain values out. No ZMQ,
no torch, no SLAM. HARD RULE (CLAUDE.md): every knob is a GENERAL planner param — the map/frontiers are
built LIVE from SLAM; nothing here encodes this room's answer.
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
        self.committed_goal = None     # [x, z] currently committed, or None
        self.verifying = False         # True while flying to the cached far corner to confirm "done"
        self.verify_target = None      # frozen [x, z] of the corner during verification (NEVER recomputed)

    @staticmethod
    def _d(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

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

    def select(self, frontiers, pos, heading_deg=None, farthest_free=None):
        """Returns (goal [x,z] | None, n_frontiers, done). `farthest_free` is consulted ONLY on the
        transition into verification (and the caller only computes it then) — see module docstring."""
        pos = (float(pos[0]), float(pos[1]))
        if frontiers:
            self.verifying = False                     # live frontiers exist -> not done, not verifying
            self.verify_target = None
            goal = self._choose(frontiers, pos, heading_deg)
            self.committed_goal = [float(goal[0]), float(goal[1])]
            return self.committed_goal, len(frontiers), False

        # --- no frontiers ---
        self.committed_goal = None
        if not self.verify_done:
            return None, 0, True
        if self.verifying:
            # Heading to the cached far corner; declare done only once we've actually reached it (and
            # frontiers are still empty — checked above). NEVER recompute the target here.
            if self.verify_target is None or self._d(pos, self.verify_target) <= self.goal_reach_dist:
                self.verifying = False
                self.verify_target = None
                return None, 0, True
            return list(self.verify_target), 0, False
        # Transition INTO verifying: cache the far corner EXACTLY ONCE (caller computed it just now).
        if farthest_free is not None and self._d(pos, farthest_free) > self.verify_min_dist:
            self.verifying = True
            self.verify_target = [float(farthest_free[0]), float(farthest_free[1])]
            return list(self.verify_target), 0, False
        return None, 0, True                            # nothing worth verifying -> truly done


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

    print(f"\n[frontier_planner][self-test] {'ALL PASS' if ok else 'FAILURES PRESENT'}")
    return ok


def main():
    ap = argparse.ArgumentParser(description="Frontier goal planner (utility + commitment + done-verify)")
    ap.add_argument("--self-test", action="store_true", help="run the synthetic self-test (no hardware)")
    args = ap.parse_args()
    if args.self_test:
        raise SystemExit(0 if run_self_test() else 1)
    ap.print_help()


if __name__ == "__main__":
    main()
