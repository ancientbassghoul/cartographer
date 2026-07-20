# Session 24 — settle-gate rewrite, pick-pulse dedup, and bounded/scaled far-corner exemption

Four more issues found dissecting the `20260717_102403` flight, independent of the session-23 BACKWALL work.

## 1 — Double SETTLE wait -> rolling-window two-gate design

**Bug**: `10:30:22.941 SLAM_HOLD "wait for SLAM to settle"` → `10:30:27.226 SETTLE "SLAM settled after 4.3s
(6 fast frames) -> resume SETTLE"` → `10:30:32.694 REPLAN` (~5.5s later) — two back-to-back waits for the
same "is SLAM producing fresh healthy frames" signal. `SLAM_HOLD`'s exit (`_slam_stable`, 3 fast frames)
already proved the solve healthy WHILE STATIONARY; entering `SETTLE` then reset the fresh-frame counter and
demanded 6 brand-new frames for no added safety.

**Root cause of why a local patch (crediting the streak only at the SLAM_HOLD->SETTLE sites) wasn't right**:
`_slam_fast_streak` conflated two different questions — "is SLAM's SOLVE currently healthy" (a data-quality
signal, reusable instantly once true) vs "has the airframe rested long enough since it stopped moving" (a
wall-clock question whose answer depends on WHERE the wait started). A full survey of every settle call site
found three buckets: **A** (follows active motion — genuinely needs a fresh dwell), **B** (resumes to
`SETTLE` from an already-stationary `SLAM_HOLD` — the double-wait bug), **C** (resumes directly to
`ADVANCE`/`PARALLAX_PUSH` from `SLAM_HOLD` with a weaker, no-capture-time-verified, no-minimum-dwell check).

**Fix — a rolling `(slam_ms, cap_ts)` window (`_slam_hist`, `deque(maxlen=settle_fresh_frames)`) fed on every
fresh frame, decoupled into two independent gates**:
- `_slam_window_ready(since=None)`: FRESHNESS — window full, every entry under `slam_slow_ms`, every entry has
  a KNOWN capture time (a frame we can't timestamp never counts as verified-fresh, even "prequalified" — this
  bit a first draft: a stale/cap_ts-less stream could otherwise look permanently "already clean"); if `since`
  given, every entry captured at/after it.
- `_settle_gate_begin(now)` / `_settle_gate_poll(now, require_fresh=True)`: opens a gate window at the TRUE
  stationary-start instant (motion-end for Category A; `SLAM_HOLD` ENTRY for B/C — not exit, so elapsed time
  naturally covers however long the hold lasted, no separate "credit" bookkeeping); polls both FRESHNESS and
  a `settle_gate_s` minimum physical dwell.

`SLAM_HOLD`'s exit now uses this gate for ALL resume targets (SETTLE/ADVANCE/PARALLAX_PUSH — Category C gets
a real gate for free). `_enter`'s SETTLE branch opens a fresh gate UNLESS the prior state was `SLAM_HOLD`
(carries the already-open window forward instead of restamping it — this one line is the actual fix for the
double-wait). Scoped deliberately: `CALIB_LOST_HOLD`/`CALIB_ESCAPE`/`POSTLUDE_LOST_HOLD` keep using
`_slam_fast_streak` unchanged (separate, already-validated mechanism, out of scope per the operator).
Removed: `slam_settle_frames`, `_slam_stable`. New config: `settle_gate_s` (default 1.0, was `rest_between_s`).

## 2 & 3 — LOOP-blacklist fires on re-orient sub-steps of the SAME goal

**Bug**: `register_goal_pick` (the goals-DB circling guard) was fed, unconditionally, on EVERY `REPLAN` that
resolved to a goal — including a multi-step turn's own repeated sub-commits (ORIENT partial-turn ->
PARALLAX_PUSH -> SETTLE -> REPLAN, re-reading the SAME still-uncommitted goal each time). Three such sub-steps
trip `goal_loop_min_picks` and permanently blacklist a perfectly good, merely-far goal. Confirmed as the exact
mechanism behind `goal=[4.65, 8.25]` (one of the 4 sweep corners) getting `LOOP-BLACKLIST` at `10:27:22.049`
while the drone kept flying toward it — corners deliberately ignore `_excluded()` (by design), so the
blacklist entry was real but inert for corner selection, while the log line misleadingly implied a reselect
that never actually happens for a corner.

**Fix** (the operator's own proposed rule, fixing both bugs at once for ALL goal types): in the REPLAN
goal-commit branch, suppress the PICK half of the pulse (`pick_goal=None`) when the newly committed goal is
the SAME as the last one (reusing the already-computed `goal_moved` boolean — the identical question the
recalibration trigger already asks, no new constant). The hop-outcome half (strike/progress) still judges
every hop as before.

## 4 — Smarter, bounded far-corner exemption

**4a**: `corner_no_blacklist_dist` (a flat 1.0u) now gets overridden LIVE by `corner_span_half` — half the
largest pairwise distance among the known `bbox_corners()` (the true diagonal in the normal 4-corner case),
published by `perception_worker.py`, consumed by `autopilot._corner_no_blacklist_dist(plan)`. Falls back to
the config default before corners are known or with fewer than 2 corners (no meaningful diagonal).

**4b**: the exemption is not infinite. A NEW persistent, proximity-keyed give-up counter
(`_corner_giveup_counts`, a list of `{goal, count}` — NOT a single reset-on-switch slot, since a reviewer
caught that oscillating between two unreachable corners would defeat a single-slot counter by resetting it on
every switch) tracks every far-corner "would-have-bumped" decision. At `corner_giveup_limit` (10) give-ups
against the SAME corner, `FrontierPlanner.force_retire_corner(goal)` retires it anyway (marks visited, drops a
matching sweep/commitment, sets `_gave_up_corner`) — a per-corner retirement, same as a real 2-bump, that does
NOT end the mission by itself.

**4c**: the mission only ends once every corner is exhausted this way (or genuinely reached) AND no frontier
is reachable (the existing `done` condition, unchanged). `perception_worker.py` publishes
`corner_giveup_stuck = bool(planner._gave_up_corner)`. REPLAN's `done` branch routes to a HARD **STUCK** hold
(not the graceful `RETURN_TO_ORIGIN` dock) when `corner_giveup_stuck` is set — per the operator: "if all
corners are exhausted, we are probably really stuck, no point trying to return to origin." Caught a real bug
while testing this: the generic step()-top "recovery -> status OK -> settle -> replan" convergence (which
`STUCK` is normally subject to, so a SLAM-fallback STUCK can auto-resume) would otherwise immediately bounce
this NEW terminal STUCK back out, since `plan.get("done")` stays permanently True. Both the convergence check
AND STUCK's own resume check now gate on a new `_corner_giveup_stuck` flag so this specific STUCK never exits.

## Files touched
- `autopilot.py` — `_slam_hist`/`_settle_gate_begin`/`_settle_gate_poll`/`_slam_window_ready` (replacing
  `_slam_stable`/`slam_settle_frames` and the SETTLE state's old fresh-frame gate); REPLAN pick-pulse dedup;
  `_corner_no_blacklist_dist`/`_corner_giveup_tick`/`_corner_giveup_counts`/`take_corner_giveup_pulse`; REPLAN
  `done` branch + STUCK exit + step()-top convergence guards for `_corner_giveup_stuck`.
- `frontier_planner.py` — `force_retire_corner` + `_gave_up_corner`.
- `perception_worker.py` — publish `corner_span_half` + `corner_giveup_stuck`; drain the new giveup pulse.
- `config.yaml` — `settle_gate_s: 1.0` (replaces `slam_settle_frames`); `corner_giveup_limit: 10`.

All 6 module self-tests green (`python autopilot.py --self-test`, `python frontier_planner.py --self-test`,
`python flow_contact_detector.py --self-test`), including new dedicated cases for every fix above.

**NEXT = LIVE-FLY** alongside the still-pending session 20b/21/22/23 checklist. Watch for: a SLAM-loss
recovery reaching REPLAN noticeably faster (no second full fresh-frame wait); a multi-step turn toward one
far goal NOT tripping LOOP-BLACKLIST from its own re-orient sub-steps (goals-DB `picks` stays at 1 across an
ORIENT/PARALLAX_PUSH/SETTLE/REPLAN cycle on the same goal); a distant, unreachable corner logging increasing
give-up counts and retiring (not spamming MISSED-BUMP forever); if every corner ends up retired this way, the
flight ending in a stationary STUCK hold (logging paused) rather than a homing/dock sequence.
