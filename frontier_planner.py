"""frontier_planner.py — Map-mode goal selection + diagonal-sweep done verification (pure numpy; offline-testable).

Chooses the next frontier the drone should fly to, from `ground_grid.GroundGrid.frontiers()`, with:

  * UTILITY selection — prefer BIG, AHEAD, NEAR frontiers:
        util = size * max(behind_floor, cos(turn)) / (1 + dist_weight * dist)
    so it does NOT flip to a tiny frontier directly behind the drone (a frontier with cos(turn) < 0 is
    floored to `behind_floor`, i.e. chosen only when nothing better exists).
  * STRONG COMMITMENT — keep the committed goal (re-associating it to the nearest live frontier as the
    cluster centroid drifts while the map grows) until it is reached / gone, UNLESS another frontier's
    utility clearly beats the committed one (by `switch_factor`). Stops the goal thrash where the planner
    abandoned a good far goal every replan.
  * DONE VERIFICATION via an ALL-CORNERS TOUR — when no frontier is reachable, TOUR the inset corners of
    the known bounding box (`ground_grid.bbox_corners`, computed by the caller). Corners are visited
    farthest-first (opposite corner first, then the farthest-unvisited, then the last), each cached here as
    a STATIC target while flying to it (never re-evaluated en route, so it can't oscillate) — a
    deterministic full-room traverse that thickens the off-path corners. If new frontiers appear en route,
    resume selection; only declare `done` once every corner has been reached/retired with STILL no
    reachable frontier. Corner targets IGNORE the frontier blacklist (a walled-off corner is retired by a
    fresh 2-bump, not `_excluded`). Replaces the fragile farthest-free / "too near" gate that could
    silently return no goal and dead-stall the controller.
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
    """Stateful frontier goal chooser. `select(frontiers, pos, heading_deg, sweep_corners)` is called
    once per replan with the LIVE frontier list; it owns the committed-goal + all-corners-tour state."""

    def __init__(self, cfg: dict | None = None, **overrides):
        e = explore_cfg(cfg)
        g = lambda k, d: overrides.get(k, e.get(k, d))
        self.dist_weight = float(g("goal_dist_weight", 0.5))   # distance penalty in the utility
        self.behind_floor = float(g("goal_behind_floor", 0.15))  # utility floor for a frontier behind us
        self.switch_factor = float(g("goal_switch_factor", 1.5))  # abandon commitment only if beaten by this
        self.assoc_dist = float(g("goal_assoc_dist", 1.0))     # associate committed goal w/ a live frontier
        self.goal_reach_dist = float(g("goal_reach_dist", 0.4))  # sweep-corner "reached" test
        self.verify_done = bool(g("verify_done", True))          # do the diagonal-sweep done-confirmation
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
        self.sweeping = False          # True while flying toward a cached corner in the all-corners tour
        self.sweep_target = None       # frozen [x, z] of the current corner (cached ONCE, never recomputed mid-leg)
        self._swept_corners = []       # [x,z] of corners already reached/retired this flight (the tour's memory;
        #                                persists for the flight, self-corrects if the bbox grows past assoc_dist)
        # Session 24: True once ANY corner was retired via force_retire_corner (the autopilot gave up on it --
        # corner_giveup_limit far-corner strikes, never once close enough for a real 2-bump) rather than
        # genuinely reached or 2-bump-confirmed unreachable. Distinguishes a truly-exhausted mission (every
        # corner reached/confirmed) from a stuck one (at least one corner was simply abandoned) for the caller.
        self._gave_up_corner = False
        self._ever_had_frontiers = False  # True once any select() saw a non-empty frontier list (distinguishes
        #                                   a still-forming startup map from a genuinely-exhausted one)
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
        # Optional map-validated clearance inset applied to a chosen frontier goal BEFORE commitment
        # (set via set_clearance_fn). fn(goal, pos) -> adjusted FREE [x,z] | None. `clearance_ok` is a
        # visible telemetry flag: False when the inset could not find a FREE buffered cell (NO silent
        # fallback — we commit the raw goal but flag it).
        self._clearance_fn = None
        self.clearance_ok = True
        # --- goals DATABASE: circling-LOOP + STALL (strike) blacklists (session 20b) ----------------------
        # Treat every PICKED goal as a DISC (center = the picked point, radius = goal_area_radius). The DB is fed
        # by the AUTOPILOT, once per leg (a REPLAN commit = one hop), NOT by the ~2 Hz select() — so a repeatedly
        # re-picked goal actually accumulates. Two independent guards, both writing the SAME permanent _blacklist
        # store _excluded reads; NEITHER blocks flight, both just add evidence:
        #   • PICKS + drone_locs -> CIRCLING/ping-pong: a disc picked MORE than goal_loop_min_picks times with ALL
        #     pick-time drone locations inside one goal_loop_pos_dist-wide cluster (max pairwise spread <= it) —
        #     the drone keeps re-picking the same goal from ~one spot. "ALL clustered" (not "any pair close") so a
        #     legit MARCHING approach over short <1u hops (adjacent picks close, trail >1u) does NOT false-fire.
        #   • STRIKES -> STALL: the autopilot measures each hop's progress toward the goal; a hop that did not get
        #     >= hop_progress_eps closer is a STRIKE (register_hop_outcome), reset to 0 on a hop that DID progress;
        #     at goal_strike_limit strikes the goal is blacklisted. A FAR corner is strike-EXEMPT (caller passes
        #     strike_eligible=False) — a corner is a reposition target flown from afar, unlike a nearby frontier.
        # The DB PERSISTS the WHOLE flight — NEVER reset on a goal switch, hop, or recovery. General SLAM-unit
        # params (HARD RULE), never a room answer.
        self.goal_area_radius = float(g("goal_area_radius", 0.5))      # pick-association disc radius (SLAM units)
        self.goal_loop_min_picks = int(g("goal_loop_min_picks", 2))   # a disc must be picked MORE than this (>2 => >=3)
        self.goal_loop_pos_dist = float(g("goal_loop_pos_dist", 1.0))  # "same drone spot" across picks -> circling
        self.goal_strike_limit = int(g("goal_strike_limit", 2))       # consecutive no-progress hops -> blacklist
        self.goal_db_maxlocs = int(g("goal_db_maxlocs", 12))          # cap the per-disc drone-location history
        self._goal_db = []            # [{center:[x,z], picks, drone_locs:[[x,z]...], strikes}] — PERSISTS the flight
        self.last_loop_event = None    # transient {goal,picks,reason} of a DB-blacklist for the caller to log once

    @staticmethod
    def _d(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def set_clearance_fn(self, fn):
        """Inject a map-validated clearance inset `fn(goal, pos) -> adjusted [x,z] | None`, applied to a
        chosen frontier goal before commitment so the committed (and published) goal keeps a clearance
        buffer off obstacles/corners. `None` means no buffered FREE cell was found."""
        self._clearance_fn = fn

    def is_excluded(self, center):
        """Public wrapper over `_excluded` so callers (e.g. the perception worker computing a blacklist-aware
        `farthest_free`) can query the dead-goal regions without reaching into a private method."""
        return self._excluded(center)

    # ---------------------------------------------------------------- blacklist
    def _excluded(self, center):
        """True if `center` falls in a dead-goal region that is currently excluded — PERMANENT (forever)
        or ACTIVE (this round). Position-UNCONDITIONED: a blacklisted goal is not re-enabled by the drone
        moving away (that was the ping-pong); a soft entry clears only via _whitelist_round on reposition."""
        for e in self._blacklist:
            if self._d(center, e["goal"]) <= self.blacklist_radius and (e["permanent"] or e["active"]):
                return True
        return False

    def _blacklist_goal(self, goal, permanent=False, reason=None, evidence=None):
        """Record `goal` as unreachable, using the best (closest) distance reached this commit
        (`self._best_dist`). `permanent=True` (the stagnation watchdog) marks it dead FOR GOOD immediately —
        the drone spent the full stagnation window unable to get closer, so it is truly unreachable. Otherwise
        (re-blacklisting the same region): if we NEVER got closer than a prior round (no cross-round progress)
        -> promote to PERMANENT (two dead goals can't cycle the drone forever); if we DID get closer (a new
        route opened) -> keep it soft/retryable. A first-ever soft blacklist stays soft.

        `reason` (`"2bump"|"stall"|"loop"`, corner give-ups never call this — see force_retire_corner) and
        `evidence` (a small dict of whatever was at hand at the call site — position, strikes/picks/spread,
        SLAM state) are recorded on the entry so the debugger's Goals DB panel can show WHY a goal died, not
        just THAT it died. All numeric evidence values must already be float-cast by the caller (goals-DB
        schema split — every coordinate/distance/drift written here is a plain float/list-of-float, never a
        raw tuple or numpy scalar, so this never trips json.dumps in the timeline logger)."""
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
                if reason is not None:
                    e["reason"] = reason
                    e["evidence"] = dict(evidence) if evidence else {}
                self.last_blacklist = g
                return
        self._blacklist.append({"goal": g, "best_ever": best_now, "permanent": bool(permanent), "active": True,
                                "reason": reason, "evidence": (dict(evidence) if evidence else {})})
        self.last_blacklist = g

    # --------------------------------------------------- goals database (persistent) + loop/stall guards
    def _db_entry(self, goal, create=True):
        """The goals-DB disc whose center is within goal_area_radius of `goal` (first match), creating a fresh
        one when none exists (unless create=False). Discs persist for the whole flight.

        Schema split (operator ask): ALL FOUR blacklist mechanisms' bookkeeping lives on this ONE per-disc
        record now — `picks`/`drone_locs` (loop guard), `strikes` (stall guard), `bumps` (2-bump, previously
        only the separate active `_wall_hit_count` streak), and `corner_giveups` (far-corner give-up,
        previously only the separate `_corner_giveup_counts` list) — plus `is_corner`, since a corner disc
        can legitimately pass through BOTH the far-exempt give-up phase and the near-bump/strike phase in one
        flight (the far-corner guard in autopilot.py only exempts a corner while it's still far away; once
        the drone is close, a corner is bumped/struck exactly like any frontier — see the corner audit in
        note_wall_hit's docstring)."""
        g = [float(goal[0]), float(goal[1])]
        for e in self._goal_db:
            if self._d(e["center"], g) <= self.goal_area_radius:
                return e
        if not create:
            return None
        e = {"center": g, "picks": 0, "drone_locs": [], "strikes": 0, "bumps": 0,
             "corner_giveups": 0, "is_corner": False}
        self._goal_db.append(e)
        return e

    def _db_blacklist(self, center, reason, extra):
        """Permanently blacklist a goals-DB disc via the SAME _blacklist store _excluded reads, dropping the
        commitment if it is this region. Idempotent (skips an already-dead region so it logs once). Sets
        last_loop_event {goal, reason, **extra} for the caller to surface, and records the SAME reason +
        evidence (`extra`) on the `_blacklist` entry (goals-DB schema split). Returns True if it newly
        blacklisted."""
        c = [float(center[0]), float(center[1])]
        if self._excluded(c):
            return False
        self._blacklist_goal(c, permanent=True, reason=reason, evidence=extra)
        self.last_loop_event = dict(extra, goal=c, reason=reason)
        if self.committed_goal is not None and self._d(self.committed_goal, c) <= self.blacklist_radius:
            self.committed_goal = None          # drop the dead commitment -> the caller re-selects around it
            self._reset_progress()
        return True

    def register_goal_pick(self, goal, pos, slam_ms=None):
        """Fed by the AUTOPILOT once per leg (a REPLAN commit). Increment the goal's disc pick count + record the
        pick-time drone `pos` (bounded), then run the CIRCLING/ping-pong test: picked MORE than goal_loop_min_picks
        times with any two pick-time drone locations within goal_loop_pos_dist -> permanent blacklist.
        `slam_ms` (optional) is evidence-only — the SLAM solve time at pick time, folded into the loop-
        blacklist evidence dict below; it never affects the loop decision itself."""
        e = self._db_entry(goal)
        e["picks"] += 1
        if pos is not None:
            e["drone_locs"].append([float(pos[0]), float(pos[1])])
            if len(e["drone_locs"]) > self.goal_db_maxlocs:
                e["drone_locs"] = e["drone_locs"][-self.goal_db_maxlocs:]
        if e["picks"] > self.goal_loop_min_picks:
            locs = e["drone_locs"]
            # CIRCLING = ALL the pick-time drone locations sit inside one goal_loop_pos_dist-wide cluster (max
            # pairwise spread <= the threshold). NOT "any pair within" — that false-fired on a legit MARCHING
            # approach to a far goal over several short (<1u) hops (adjacent picks are close, but the trail
            # spans >1u). A drone hammering one goal from ~one spot keeps every pick in the small ball; the
            # STALL/strike guard separately covers "approached then got stuck" (net no progress).
            spread = max((self._d(locs[i], locs[j])
                          for i in range(len(locs)) for j in range(i + 1, len(locs))), default=0.0)
            if len(locs) >= 2 and spread <= self.goal_loop_pos_dist:
                evidence = {"picks": int(e["picks"]), "spread": round(float(spread), 3)}
                if pos is not None:
                    evidence["pos"] = [round(float(pos[0]), 3), round(float(pos[1]), 3)]
                if slam_ms is not None:
                    evidence["slam_ms"] = round(float(slam_ms), 1)
                self._db_blacklist(e["center"], "loop", evidence)

    def register_hop_outcome(self, goal, progressed, strike_eligible=True, pos=None, slam_ms=None, is_corner=False):
        """Fed by the AUTOPILOT once per hop (at the REPLAN that ends it). STALL guard via STRIKES: a hop that got
        meaningfully closer (`progressed=True`) RESETS the goal's strikes; one that did not adds a STRIKE; at
        goal_strike_limit strikes -> permanent blacklist. `strike_eligible=False` (a FAR corner) is a no-op —
        neither strike nor reset (a far corner must never be retired for being stalled far from it). `pos`/
        `slam_ms` (optional) are evidence-only — the drone position and SLAM solve time at the judging REPLAN
        — folded into the stall-blacklist evidence dict; neither affects the strike decision itself.
        `is_corner` (independent of `strike_eligible` — a NEAR corner IS strike-eligible but still a corner)
        just flags the disc for the debugger; set unconditionally, even on a strike-exempt far-corner no-op."""
        e = self._db_entry(goal)
        if is_corner:
            e["is_corner"] = True
        if progressed:
            e["strikes"] = 0
            return
        if not strike_eligible:
            return
        e["strikes"] += 1
        if e["strikes"] >= self.goal_strike_limit:
            evidence = {"strikes": int(e["strikes"])}
            if pos is not None:
                evidence["pos"] = [round(float(pos[0]), 3), round(float(pos[1]), 3)]
            if slam_ms is not None:
                evidence["slam_ms"] = round(float(slam_ms), 1)
            self._db_blacklist(e["center"], "stall", evidence)

    def goal_db_snapshot(self):
        """Per-disc view for telemetry / the replay debugger: center, picks, strikes, bumps, corner_giveups,
        is_corner, the DRONE LOCATIONS at each pick (what the <goal_loop_pos_dist clustering test runs on —
        so the operator can see whether a loop blacklist was legit), whether the disc is currently
        blacklisted (dead in the _blacklist store), and — when it is — the mechanism that killed it
        (`blacklist_reason`: "2bump"|"stall"|"loop"|None) plus its recorded `blacklist_evidence` (goals-DB
        schema split — one place that answers "why is this goal dead", instead of three disconnected
        counters). A corner force-retired via the far-corner give-up cap is NOT in `_blacklist` (see
        force_retire_corner) so `blacklisted`/`blacklist_reason` reflect only 2bump/stall/loop; its
        `corner_giveups` count is the record of that separate retirement."""
        out = []
        for e in self._goal_db:
            dead = self._excluded(e["center"])
            reason, evidence = None, {}
            if dead:
                for bl in self._blacklist:
                    if self._d(bl["goal"], e["center"]) <= self.blacklist_radius:
                        reason = bl.get("reason")
                        evidence = bl.get("evidence") or {}
                        break
            out.append({
                "center": [round(e["center"][0], 3), round(e["center"][1], 3)],
                "picks": e["picks"], "strikes": e["strikes"], "bumps": e["bumps"],
                "corner_giveups": e["corner_giveups"], "is_corner": bool(e["is_corner"]),
                "drone_locs": [[round(p[0], 3), round(p[1], 3)] for p in e["drone_locs"]],
                "blacklisted": bool(dead), "blacklist_reason": reason, "blacklist_evidence": evidence,
            })
        return out

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

    def note_wall_hit(self, goal, pos=None, is_corner=False):
        """Register ONE advance-blocked "bump" (flow WALL / ram-guard / stand-off) against `goal`, reported by
        the autopilot. Consecutive bumps on the SAME goal region (within `assoc_dist`) accumulate; a bump on a
        DIFFERENT goal resets the counter (so only genuinely-repeated same-goal contacts count). The SECOND bump
        declares the goal UNREACHABLE and PERMANENTLY blacklists it, drops the commitment, and resets the
        counter (the next select() reselects around the dead region). Event-driven — no timer, no SLAM-health
        gate. The autopilot's kinematic latch guarantees a single continuous contact is only one bump, so this
        never fires on state-machine flicker. Sets `last_blacklist` (the [x,z] just blacklisted, else None) and
        returns/stashes `last_bump`, an outcome dict {goal, count, threshold, action, prev_goal}, so the caller
        can log EVERY bump (not just blacklists) — making the goal-change counter resets visible.

        CORNER AUDIT (operator ask): a corner is NOT unconditionally exempt here — the autopilot's
        `_register_bump` only calls this at all once the corner is within its (live, room-scaled)
        far-corner exemption distance; farther than that, the autopilot diverts to the SEPARATE
        `force_retire_corner` give-up path instead and this never fires. So a near corner is bumped/
        blacklisted exactly like any frontier goal — `is_corner` here is passed through purely for the
        goals-DB disc's `is_corner` flag/evidence, not to change this decision.

        `pos` (optional) is evidence-only — the drone position at bump time (the autopilot's existing
        `_last_bump_anchor`) — folded into the goals-DB disc's persistent `bumps` tally (distinct from
        this method's own transient `_wall_hit_count` streak) and the 2-bump blacklist evidence."""
        self.last_blacklist = None
        g = [float(goal[0]), float(goal[1])]
        e = self._db_entry(g)
        e["bumps"] += 1
        if is_corner:
            e["is_corner"] = True
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
            evidence = {"bumps": int(e["bumps"])}
            if pos is not None:
                evidence["pos"] = [round(float(pos[0]), 3), round(float(pos[1]), 3)]
            self._blacklist_goal(g, permanent=True, reason="2bump", evidence=evidence)
            if self.committed_goal is not None and self._d(self.committed_goal, g) <= self.blacklist_radius:
                self.committed_goal = None         # drop the dead commitment -> next select() reselects
                self._reset_progress()
            # Unreachable-CORNER retirement WITHOUT `_excluded` filtering: if this bump killed the corner
            # we were touring toward, mark that corner VISITED (so _pick_sweep_corner skips it) and clear
            # the sweep. A corner the drone actively fails to reach THIS tour is retired via the fresh
            # 2-bump — never via a stale blacklist filter — preserving termination while corners still
            # ignore `_excluded`.
            if (self.sweeping and self.sweep_target is not None
                    and self._d(self.sweep_target, g) <= self.assoc_dist):
                self._mark_corner_visited(self.sweep_target)
                self.sweeping = False
                self.sweep_target = None
            self._wall_hit_count = 0
            self._last_wall_hit_goal = None
            action = "blacklist"
        self.last_bump = {"goal": g, "count": count_at_hit, "threshold": 2,
                          "action": action, "prev_goal": prev_goal}
        return self.last_bump

    def _commit(self, goal):
        """Commit to `goal`, restarting progress tracking only when it is a genuinely DIFFERENT region
        (a jump beyond `assoc_dist`) — small centroid drift under association keeps the same stall clock. The
        goals-DB pick/strike counting is driven by the AUTOPILOT per leg (register_goal_pick / register_hop_
        outcome), NOT here — the ~2 Hz select() must not inflate a held goal's counts."""
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
        decide whether to compute `bbox_corners` (the corner tour target when every frontier is dead)."""
        return any(not self._excluded((float(f["center"][0]), float(f["center"][1]))) for f in frontiers)

    # --------------------------------------------------- all-corners verification tour
    def _corner_visited(self, c):
        """True if corner `c` is within `assoc_dist` of a corner already reached/retired this flight."""
        return any(self._d(c, v) <= self.assoc_dist for v in self._swept_corners)

    def _mark_corner_visited(self, c):
        """Record corner `c` as reached/retired (deduped by `assoc_dist`), so the tour advances past it."""
        cc = [float(c[0]), float(c[1])]
        if not self._corner_visited(cc):
            self._swept_corners.append(cc)

    def force_retire_corner(self, goal):
        """A corner the autopilot GAVE UP reaching (session 24: `corner_giveup_limit` far-corner strikes,
        never once close enough for a real 2-bump) is retired anyway: mark it visited (the tour skips it,
        moving on to the next unvisited corner exactly like a real 2-bump retirement) and remember we
        ABANDONED it rather than reached/2-bump-confirmed it unreachable (`_gave_up_corner`), so the caller
        can tell a genuinely-exhausted mission (every corner reached/confirmed) from a stuck one (at least
        one corner was simply abandoned). Mirrors `note_wall_hit`'s corner-retirement branch, but without
        requiring the drone to have ever gotten close.

        Goals-DB schema split: records the give-up on the SAME per-disc entry `note_wall_hit`/
        `register_hop_outcome` use (`corner_giveups` count + `is_corner` flag), so the debugger can see a
        corner's full history in one place. Deliberately does NOT call `_blacklist_goal` — a force-retired
        corner is "given up on for this tour", not declared permanently unreachable the way a 2-bump/stall/
        loop is; `_excluded()` stays exactly as it was (corners already ignore it by design, per
        `_pick_sweep_corner`'s docstring), so this is bookkeeping only, no behavior change."""
        g = [float(goal[0]), float(goal[1])]
        e = self._db_entry(g)
        e["corner_giveups"] += 1
        e["is_corner"] = True
        self._mark_corner_visited(g)
        self._gave_up_corner = True
        if self.sweeping and self.sweep_target is not None and self._d(self.sweep_target, g) <= self.assoc_dist:
            self.sweeping, self.sweep_target = False, None
        if self.committed_goal is not None and self._d(self.committed_goal, g) <= self.blacklist_radius:
            self.committed_goal = None
            self._reset_progress()

    def _pick_sweep_corner(self, corners, pos):
        """The FARTHEST-from-`pos` corner that is not yet visited — and NOTHING ELSE. It MUST NOT consult
        `_excluded`: corner targets are NEVER suppressed by old frontier blacklists (operator's explicit
        requirement — a walled-off corner is retired by a fresh 2-bump in note_wall_hit, not a stale
        filter). Farthest-first ⇒ opposite corner first, then the far one of the rest, then the last."""
        cand = [c for c in (corners or []) if not self._corner_visited(c)]
        if not cand:
            return None
        return max(cand, key=lambda c: self._d(c, pos))

    def _select_reachable(self, frontiers, pos, heading_deg):
        """Choose + commit among the currently non-excluded frontiers, clearing any verify state."""
        reachable = [f for f in frontiers
                     if not self._excluded((float(f["center"][0]), float(f["center"][1])))]
        if not reachable:
            return None
        self.sweeping = False                          # something to chase -> not done, not sweeping
        self.sweep_target = None
        goal = self._choose(reachable, pos, heading_deg)
        # Map-validated clearance inset: pull a goal that hugs an obstacle/corner back along the drone->goal
        # axis to a FREE buffered cell before we commit (so committed == published; bump association holds).
        if self._clearance_fn is not None:
            adjusted = self._clearance_fn(goal, pos)
            if adjusted is not None:
                goal = (float(adjusted[0]), float(adjusted[1]))
                self.clearance_ok = True
            else:
                # NO SILENT FALLBACK: no FREE buffered cell on the segment -> commit the raw goal but flag it.
                self.clearance_ok = False
                print(f"[planner] clearance inset found no FREE buffered cell for goal {goal} "
                      f"(pos {pos}) -> committing raw goal, clearance_ok=False", flush=True)
        self._commit(goal)
        return self.committed_goal

    def select(self, frontiers, pos, heading_deg=None, sweep_corners=None):
        """Returns (goal [x,z] | None, n_frontiers, done). Unreachable-goal blacklisting is event-driven
        (`note_wall_hit`, fed by the autopilot's advance-blocked stops) — NOT a per-select timer.
        `sweep_corners` is the LIST of inset bbox corners (`ground_grid.bbox_corners`), consulted only
        when no frontier is reachable (the caller computes it then): the planner TOURS them (farthest-first
        ⇒ opposite, then farthest-unvisited, then last) so every room corner reconstructs densely. Each
        corner target is cached STATICALLY while flying to it. This NEVER returns a `goal=None, done=False`
        resting state — the only such case is a momentary startup tick before the first frontiers form
        (bounded by the autopilot's idle backstop)."""
        pos = (float(pos[0]), float(pos[1]))
        self.last_blacklist = None
        if frontiers:
            self._ever_had_frontiers = True

        goal = self._select_reachable(frontiers, pos, heading_deg)
        if goal is not None:
            return goal, len(frontiers), False

        # --- nothing reachable: no frontiers exist, OR every live frontier is excluded ("been over all
        # goals"). Run the ALL-CORNERS TOUR: fly to each inset bbox corner in turn (farthest-first) — a
        # deterministic full-room traverse that thickens the off-path corners AND doubles as
        # done-verification / reposition-retry (on arrival the round's soft blacklist clears + frontiers
        # retry from the fresh vantage). Corner targets IGNORE the frontier blacklist (operator ask); a
        # walled-off corner is retired by a fresh 2-bump in note_wall_hit, not `_excluded`.
        self.committed_goal = None
        self._reset_progress()
        if not self.verify_done:
            return None, len(frontiers), True

        corners = sweep_corners or []
        # Auto-mark any corner we are already sitting on (the start corner + any passed en route).
        for c in corners:
            if self._d(pos, c) <= self.goal_reach_dist:
                self._mark_corner_visited(c)

        if self.sweeping and self.sweep_target is not None:
            if self._d(pos, self.sweep_target) > self.goal_reach_dist:
                return list(self.sweep_target), len(frontiers), False   # still en route -> keep the static target
            # Reached the current corner: mark it visited, drop the target, and retry frontiers from this
            # fresh vantage before touring on.
            self._mark_corner_visited(self.sweep_target)
            self.sweeping = False
            self.sweep_target = None
            if frontiers:
                self._whitelist_round()
                g = self._select_reachable(frontiers, pos, heading_deg)
                if g is not None:
                    return g, len(frontiers), False
            # else fall through to pick the NEXT corner.

        # Pick the next unvisited corner (farthest-first; corners ignore the blacklist) + cache it STATICALLY.
        nxt = self._pick_sweep_corner(corners, pos)
        if nxt is not None:
            self.sweeping = True
            self.sweep_target = [float(nxt[0]), float(nxt[1])]
            return list(self.sweep_target), len(frontiers), False

        # No corner left to tour. One in-place whitelist retry — but NOT on the same tick we just
        # blacklisted a goal (let the exclusion stand a tick so we don't instantly re-commit it).
        if frontiers and self.last_blacklist is None:
            self._whitelist_round()
            g = self._select_reachable(frontiers, pos, heading_deg)
            if g is not None:
                return g, len(frontiers), False
        if self._ever_had_frontiers or self._swept_corners:
            return None, len(frontiers), True          # explored + toured, nothing left -> done
        return None, len(frontiers), False             # startup: map still forming -> transient idle


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

    # (c) done verification via the ALL-CORNERS TOUR: empty frontiers + a corner LIST -> tour them
    #     farthest-first (opposite corner first), each cached EXACTLY as given (already inset by
    #     ground_grid.bbox_corners); reaching the LAST corner with still-empty frontiers -> done=True.
    p = FrontierPlanner(None)
    tour = [[0.0, 0.0], [5.0, 0.0], [0.0, 5.0], [5.0, 5.0]]
    g, n, done = p.select([], [0.0, 0.0], sweep_corners=tour)      # at [0,0] -> farthest is [5,5]
    check("(c) empty -> tour to farthest (opposite) corner, not done",
          close(g, [5.0, 5.0]) and not done and p.sweeping)
    g, _, done = p.select([], [5.0, 5.0], sweep_corners=tour)      # reached [5,5] -> next farthest-of-rest
    check("(c) reached a corner -> advance to the next unvisited, not done",
          g is not None and not close(g, [5.0, 5.0]) and not done)
    g, _, done = p.select([], list(g), sweep_corners=tour)         # reach it -> the last unvisited corner
    check("(c) tour advances to the last unvisited corner, not done", g is not None and not done)
    _, _, done = p.select([], list(g), sweep_corners=tour)         # reached the last (all 4 visited) -> done
    check("(c) all corners toured + empty -> done", done and not p.sweeping and len(p._swept_corners) == 4)

    # (c2) STATIC target: while flying to a cached corner, a moving drone (and a different corner list
    #      passed in) must keep returning the SAME cached corner — no oscillation.
    p = FrontierPlanner(None)
    p.select([], [0.0, 0.0], sweep_corners=[[0.0, 0.0], [9.0, 9.0]])   # cache [9,9]
    g_a, _, _ = p.select([], [1.0, 0.0], sweep_corners=[[0.0, 0.0], [3.0, 3.0]])
    g_b, _, _ = p.select([], [2.0, 2.0], sweep_corners=[[-9.0, -9.0]])
    check("(c2) corner target stays frozen as the drone moves", close(g_a, [9.0, 9.0]) and close(g_b, [9.0, 9.0]))

    # (c3) frontiers reappearing mid-tour -> resume selection, no premature done.
    p = FrontierPlanner(None)
    p.select([], [0.0, 0.0], sweep_corners=[[0.0, 0.0], [5.0, 0.0]])
    g, n, done = p.select([A], [1.0, 0.0], heading_deg=0.0)
    check("(c3) frontier reappears mid-tour -> resume, not done", g is not None and not done and not p.sweeping)

    # (c4) verify_done=False -> done immediately; no corners AFTER exploring -> done; no corners at
    #      STARTUP (never had a frontier) -> a transient idle (goal=None, done=False), NOT a premature done.
    _, _, d_off = FrontierPlanner(None, verify_done=False).select([], [0.0, 0.0], sweep_corners=[[5.0, 0.0]])
    p_end = FrontierPlanner(None); p_end._ever_had_frontiers = True
    _, _, d_end = p_end.select([], [0.0, 0.0], sweep_corners=None)
    _, _, d_startup = FrontierPlanner(None).select([], [0.0, 0.0], sweep_corners=None)
    check("(c4) disabled -> done; exhausted+no-corners -> done; startup+no-corners -> idle (not done)",
          d_off and d_end and not d_startup)

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

    # (i) when EVERY live frontier is excluded, route to the corner tour (not a crash, not a premature
    #     done); on ARRIVAL the round's soft blacklist clears and the frontier is retried.
    p = FrontierPlanner(None)
    p._best_dist = 0.6
    p._blacklist_goal([0.0, 3.0])                     # W soft-excluded
    g_v, n_v, done_v = p.select([W], [0.0, 2.4], heading_deg=0.0, sweep_corners=[[5.0, 5.0], [0.0, 0.0]])
    check("(i) all-excluded -> tour to farthest corner (as given), frontiers still reported, not done",
          close(g_v, [5.0, 5.0]) and n_v == 1 and not done_v and p.sweeping)
    g_r, n_r, done_r = p.select([W], [5.0, 5.0], heading_deg=0.0, sweep_corners=[[5.0, 5.0], [0.0, 0.0]])
    check("(i) reached corner -> whitelist the round + retry the frontier (not done)",
          close(g_r, [0.0, 3.0]) and n_r == 1 and not done_r and not p.sweeping)

    # (A3) NO extra inset: the cached corner target equals the passed corner exactly (ground_grid.bbox_corners
    #      already applies the stand-off inset; the planner must not double-inset it).
    p = FrontierPlanner(None)
    g_pull, _, _ = p.select([], [1.0, 1.0], sweep_corners=[[9.0, 5.0], [0.0, 0.0]])
    check("(A3) corner target is the passed corner exactly (no extra pull)", close(g_pull, [9.0, 5.0]))

    # (A1) corners IGNORE the frontier blacklist: a corner sitting in a blacklisted region is STILL toured
    #      (operator requirement) — corner retirement is via a fresh 2-bump, not `_excluded` filtering.
    p = FrontierPlanner(None); p._ever_had_frontiers = True
    p._blacklist_goal([5.0, 0.0], permanent=True)                  # dead region at a corner
    g_x, _, done_x = p.select([], [0.0, 0.0], sweep_corners=[[0.0, 0.0], [5.0, 0.0]])
    check("(A1) corner in a blacklisted region is STILL toured (corners ignore _excluded)",
          close(g_x, [5.0, 0.0]) and not done_x and p.sweeping)

    # (A2) ESCAPE via corner retirement: while touring toward a corner, two bumps on that target retire it
    #      (mark it visited + clear the sweep); the next select ADVANCES to a different unvisited corner.
    p = FrontierPlanner(None)
    tour_esc = [[0.0, 0.0], [5.0, 0.0], [-6.0, 0.0]]
    g0, _, _ = p.select([], [0.0, 0.0], sweep_corners=tour_esc)    # at [0,0] -> farthest is [-6,0]
    check("(A2) tour picks the farthest corner first", close(g0, [-6.0, 0.0]))
    p.note_wall_hit(g0); p.note_wall_hit(g0)                       # 2 bumps on the current corner -> retire it
    g_esc, _, done_esc = p.select([], [0.0, 0.0], sweep_corners=tour_esc)
    check("(A2) escape: retired corner marked visited -> advance to the next unvisited corner",
          p.sweeping and not done_esc and close(g_esc, [5.0, 0.0]) and p._corner_visited([-6.0, 0.0]))
    # …and if NO unvisited corner remains, the escape declares done instead of looping forever (explored).
    p2 = FrontierPlanner(None); p2._ever_had_frontiers = True
    tour2 = [[0.0, 0.0], [5.0, 0.0]]
    g1, _, _ = p2.select([], [0.0, 0.0], sweep_corners=tour2)      # at [0,0] (auto-marked) -> [5,0]
    p2.note_wall_hit(g1); p2.note_wall_hit(g1)                     # retire [5,0]
    _, _, done_none = p2.select([], [0.0, 0.0], sweep_corners=tour2)   # no unvisited corner left
    check("(A2) escape with no corner left -> done", done_none and not p2.sweeping)

    # (A4, session 24) force_retire_corner: the autopilot's far-corner give-up escalation retires a corner
    #      it never got close to (unlike note_wall_hit's 2-bump, no proximity required) -- marks it visited,
    #      drops a matching sweep/commitment, and flags _gave_up_corner; the tour advances to the next corner.
    p = FrontierPlanner(None)
    tour_giveup = [[0.0, 0.0], [5.0, 0.0], [-6.0, 0.0]]
    g0, _, _ = p.select([], [0.0, 0.0], sweep_corners=tour_giveup)   # at [0,0] -> farthest is [-6,0]
    force_ok_before = (not p._corner_visited([-6.0, 0.0]) and not p._gave_up_corner)
    p.force_retire_corner(g0)                                        # give up WITHOUT ever bumping it
    g_next, _, done_gu = p.select([], [0.0, 0.0], sweep_corners=tour_giveup)
    force_retire_ok = (force_ok_before and p._corner_visited([-6.0, 0.0]) and p._gave_up_corner
                       and p.sweeping and not done_gu and close(g_next, [5.0, 0.0]))
    # once EVERY corner is retired (mix of give-up + a genuine reach), the tour still correctly declares done
    # -- and _gave_up_corner (a flight-level flag, set once) stays True, distinguishing this from a clean finish.
    p2gu = FrontierPlanner(None); p2gu._ever_had_frontiers = True
    tour_gu2 = [[0.0, 0.0], [5.0, 0.0]]
    g1gu, _, _ = p2gu.select([], [0.0, 0.0], sweep_corners=tour_gu2)   # -> [5,0]
    p2gu.force_retire_corner(g1gu)                                     # give up on it (no bumps at all)
    _, _, done_gu2 = p2gu.select([], [0.0, 0.0], sweep_corners=tour_gu2)
    force_retire_exhausts_ok = (done_gu2 and not p2gu.sweeping and p2gu._gave_up_corner)
    check("(A4) force_retire_corner: give-up retires without proximity, advances tour, "
          "exhaustion still -> done, _gave_up_corner flags it",
          force_retire_ok and force_retire_exhausts_ok)

    # (opt) clearance inset: a chosen frontier goal is run through the injected clearance_fn before commit,
    #       so committed==published; a None return commits the RAW goal and flags clearance_ok=False.
    p = FrontierPlanner(None)
    p.set_clearance_fn(lambda goal, pos: [0.0, 2.0])              # pretend the map pulls it back to [0,2]
    g_c, _, _ = p.select([A], [0.0, 0.0], heading_deg=0.0)        # A center [0,3]
    check("(opt) clearance inset moves the committed goal + clearance_ok stays True",
          close(g_c, [0.0, 2.0]) and close(p.committed_goal, [0.0, 2.0]) and p.clearance_ok is True)
    p = FrontierPlanner(None)
    p.set_clearance_fn(lambda goal, pos: None)                   # no FREE buffered cell on the segment
    g_c2, _, _ = p.select([A], [0.0, 0.0], heading_deg=0.0)
    check("(opt) clearance inset None -> commit raw goal + clearance_ok=False (visible, no silent fallback)",
          close(g_c2, [0.0, 3.0]) and p.clearance_ok is False)

    # ---- (session 20) GOALS DATABASE + circling-LOOP blacklist ----
    # Each PICKED goal is a DISC (radius goal_area_radius=0.5). A pick registers only on a genuine goal-SWITCH
    # (a different disc than the last pick), so holding one goal across the ~2 Hz selects counts once, while
    # ping-pong counts each switch. A disc picked > goal_loop_min_picks (>2) with any two pick-time drone
    # locations within goal_loop_pos_dist (<1u) == circling -> PERMANENT blacklist. The DB PERSISTS the flight.
    A = [5.0, 0.0]

    # (db1) CIRCLING loop: register_goal_pick counts REPEATED picks of the SAME goal (autopilot-driven, per leg);
    #       3 picks from ~one spot (drone-locs clustered <1u) -> permanent LOOP blacklist. This is the regression
    #       the session-20 flight exposed (the old disc-cursor suppressed repeated same-goal picks).
    p = FrontierPlanner(None)
    for _ in range(3):
        p.register_goal_pick(A, [0.1, 0.1])
    check("(db1) 3 repeated picks of one goal from ~one spot -> LOOP blacklist",
          p._excluded(A) and p.last_loop_event and p.last_loop_event["reason"] == "loop"
          and p.last_loop_event["picks"] == 3)

    # (db2) picks from SPREAD-OUT spots -> a legit revisit, NOT a loop.
    p = FrontierPlanner(None)
    for ps in ([0.0, 0.0], [3.0, 0.0], [6.0, 0.0]):
        p.register_goal_pick(A, ps)
    check("(db2) 3 picks from spread-out spots -> NOT a loop", not p._excluded(A) and p.last_loop_event is None)

    # (db2b) MARCHING approach: adjacent picks < 1u apart but the trail spans > 1u -> NOT a loop (the tightening;
    #        the old "any pair < 1u" rule would have false-fired here on a legit approach to a far goal).
    p = FrontierPlanner(None)
    for ps in ([0.0, 0.0], [0.6, 0.0], [1.2, 0.0]):    # adjacent gaps 0.6u (<1), total spread 1.2u (>1)
        p.register_goal_pick(A, ps)
    check("(db2b) marching approach (adjacent <1u, spread >1u) -> NOT a loop",
          not p._excluded(A) and p.last_loop_event is None)

    # (db3) STALL via strikes: two consecutive no-progress hops -> blacklist; a progressing hop RESETS strikes.
    p = FrontierPlanner(None)
    p.register_hop_outcome(A, progressed=False)
    armed = (not p._excluded(A)) and p._goal_db[0]["strikes"] == 1
    p.register_hop_outcome(A, progressed=False)
    check("(db3) two no-progress hops -> STALL blacklist (1 strike doesn't)",
          armed and p._excluded(A) and p.last_loop_event["reason"] == "stall")
    p2 = FrontierPlanner(None)
    p2.register_hop_outcome(A, progressed=False)
    p2.register_hop_outcome(A, progressed=True)      # meaningful advancement resets the strike
    p2.register_hop_outcome(A, progressed=False)
    check("(db3) a progressing hop resets strikes (no premature blacklist)",
          not p2._excluded(A) and p2._goal_db[0]["strikes"] == 1)

    # (db4) a FAR corner (strike_eligible=False) is NEVER struck/blacklisted, however many stalled hops.
    p = FrontierPlanner(None)
    for _ in range(5):
        p.register_hop_outcome([9.0, 0.0], progressed=False, strike_eligible=False)
    check("(db4) far corner is strike-exempt -> never blacklisted",
          not p._excluded([9.0, 0.0]) and p._goal_db[0]["strikes"] == 0)

    # (db5) DB PERSISTS across goal switches + snapshot shape (picks/strikes/blacklisted).
    p = FrontierPlanner(None)
    p.register_goal_pick(A, [0.0, 0.0]); p.register_goal_pick([0.0, 5.0], [0.0, 0.0])
    p.register_hop_outcome(A, progressed=False); p.register_goal_pick(A, [0.0, 0.0])
    snap = p.goal_db_snapshot()
    a_row = next((r for r in snap if abs(r["center"][0] - 5.0) < 1e-6), None)
    check("(db5) DB persists across switches + snapshot has picks/strikes/drone_locs",
          len(snap) == 2 and a_row and a_row["picks"] == 2 and a_row["strikes"] == 1
          and a_row["drone_locs"] == [[0.0, 0.0], [0.0, 0.0]]
          and set(a_row) == {"center", "picks", "strikes", "bumps", "corner_giveups", "is_corner",
                             "drone_locs", "blacklisted", "blacklist_reason", "blacklist_evidence"})

    # (db6) goals-DB schema split (operator ask): each blacklist mechanism records its OWN reason +
    # evidence on the disc, and a corner disc can carry BOTH a bump tally and a give-up tally.
    p = FrontierPlanner(None)
    p.note_wall_hit([1.0, 1.0], pos=[0.5, 0.5])
    p.note_wall_hit([1.0, 1.0], pos=[0.6, 0.6])            # 2nd same-region bump -> 2bump blacklist
    snap = p.goal_db_snapshot()
    row = next(r for r in snap if abs(r["center"][0] - 1.0) < 1e-6)
    check("(db6) 2bump: disc bumps tally + blacklist reason/evidence recorded",
          row["bumps"] == 2 and row["blacklisted"] and row["blacklist_reason"] == "2bump"
          and row["blacklist_evidence"].get("pos") == [0.6, 0.6])

    p2 = FrontierPlanner(None)
    p2.register_hop_outcome(A, progressed=False, pos=[2.0, 3.0], slam_ms=850.0)
    p2.register_hop_outcome(A, progressed=False, pos=[2.1, 3.1], slam_ms=900.0)   # 2nd strike -> stall blacklist
    row2 = next(r for r in p2.goal_db_snapshot() if abs(r["center"][0] - A[0]) < 1e-6)
    check("(db6) stall: blacklist reason/evidence (pos+slam_ms) recorded",
          row2["blacklisted"] and row2["blacklist_reason"] == "stall"
          and row2["blacklist_evidence"].get("pos") == [2.1, 3.1]
          and row2["blacklist_evidence"].get("slam_ms") == 900.0)

    p3 = FrontierPlanner(None)
    corner = [9.0, 0.0]
    p3.force_retire_corner(corner)
    p3.note_wall_hit(corner, pos=[8.0, 0.0], is_corner=True)      # same corner, now bumped once it's near
    row3 = next(r for r in p3.goal_db_snapshot() if abs(r["center"][0] - corner[0]) < 1e-6)
    check("(db6) a corner disc carries BOTH give-up + bump history, is_corner flagged, giveup not blacklisted",
          row3["corner_giveups"] == 1 and row3["bumps"] == 1 and row3["is_corner"] is True
          and not row3["blacklisted"])   # one bump doesn't blacklist; force_retire_corner never does either

    print(f"\n[frontier_planner][self-test] {'ALL PASS' if ok else 'FAILURES PRESENT'}")
    return ok


def main():
    ap = argparse.ArgumentParser(description="Frontier goal planner (utility + commitment + diagonal-sweep done + blacklist)")
    ap.add_argument("--self-test", action="store_true", help="run the synthetic self-test (no hardware)")
    args = ap.parse_args()
    if args.self_test:
        raise SystemExit(0 if run_self_test() else 1)
    ap.print_help()


if __name__ == "__main__":
    main()
