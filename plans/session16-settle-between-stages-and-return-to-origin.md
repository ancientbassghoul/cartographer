# Plan — Session 16: a SETTLE between every action + full return-to-origin ending  [BUILT; all module self-tests green; live-fly PENDING]

## Context
A test flight finished its last corner, tried to return to origin, and fell apart: it "turned like a maniac,"
fired the reverse-list back-to-back with no settles, exhausted itself, fell back to spinning (also no settles),
declared STUCK, then retried. Root cause is one pattern in three places — **commanded actions fire back-to-back
with no still window for monocular SLAM to re-lock**, so the pose stays frozen/stale and the failure compounds.
Session 15 fixed exactly this for the per-leg SETTLE (wait for N fresh SLAM frames CAPTURED after the settle
began), but that gate only guards settles ending in REPLAN/REVERSE_PROBE. The operator asked to fold a
settle-between-stages into the return-to-origin plan AND extend it to the two recovery mechanisms, in one
"fix the ending" session (also builds the four diagnosed return-to-origin fixes).

## Part 0 — Shared settle gate (reusable primitive)
Generalized the session-15 SETTLE tracker into `_settle_begin(now)` + `_settle_poll(now, plan, *, require_fast,
min_frames, max_hold_s) -> (done, capped)` on `ExploreController`. Counts each FRESH frame (dedup on frame_id)
CAPTURED after the window began (`cap_ts >= _settle_t0`); `require_fast` also demands `slam_ms < slam_slow_ms`.
`done` at `>= rest_between_s` elapsed AND `>= min_frames`; `capped=True` (still ends) at `max_hold_s` with too
few fresh frames (VISIBLE — no silent fallback). Two flavors: **healthy** (`require_fast=True, max_hold_s=None`
— status==OK stays structurally enforced by the step()-top guard) and **lost-SLAM** (`require_fast=False` +
finite `max_hold_s` — SLAM is STALE by definition, so gate on fresh CAPTURE only, bounded so a re-exposure
maneuver still follows). Refactored the SETTLE state to call the helper — behavior-identical (session-15 SETTLE
tests stay green).

## Part 1 — Recovery settles (REWIND + spin FALLBACK)
Config `recovery_settle_frames` (4) + `recovery_settle_max_s` (2.5); flag `_rec_settling` (cleared on any real
`_enter` and in `reset_leg`). In `_step_stale`: when a REWIND inverse finishes, HOLD in a lost-SLAM settle
before popping the next; when a FALLBACK attempt finishes, HOLD before the next attempt / STUCK. Removed the
fallback recipe's trailing rest (the settle owns the post-attempt pause). The step()-top guard still runs each
tick during the hold, so a genuine OK re-lock exits recovery naturally. This resolves the session-15 parked
"reverse-without-settling" item.

**FALLBACK step order flipped (operator ask): turn → rest → push (was push → rest → turn).** Each attempt is now
`turn 15deg (yaw + 'c' reset) → rest → ring-picked push (backward/strafe, never forward)` so the LAST motion
before the inter-attempt SETTLE is the parallax translation, not a bare rotation (the SLAM-killer) — SLAM
re-locks on the rescued view, and it matches the "reset attitude with 'c' BEFORE a push" playbook recipe. Push
direction is still picked from the pre-turn `_last_ring` (15deg is small, push is short/throttled/never-forward,
so ram risk stays low). Self-test asserts `turn-before-push`.

## Part 2 — Full return-to-origin ending  (folds `plans/return-to-origin-and-graceful-dock.md`)
1. **`home_reach_dist` 1.0 -> 0.5** (config) so the drone flies home at altitude before docking.
2. **`_takeoff_heading`** captured once airborne + healthy (first healthy `heading_deg`); new **`ORIENT_HOME`**
   state after homing: `<=turn_step_deg` turns toward it with a SETTLE between, then -> DOCK_FLOOR (docks when
   within half a turn step; skips if no heading was ever captured).
3. **DOCK survives a SLAM loss:** `POSTLUDE_STATES` set + `_step_postlude_lost` (mirror of `_step_calib_lost`)
   + a top-guard exception — a loss in any postlude state diverts to **`POSTLUDE_LOST_HOLD`** (owns every status,
   OK-gated exit) and RESUMES the interrupted stage, never the generic HOLD_LOST/FALLBACK. On resume the
   homing/orient turn phase re-plans (never resumes a mid-turn recipe on a cleared player).
4. **No floor re-inflation:** DOCK_FLOOR clears `target_altitude_y`; the step-top lock caching is gated off in
   `_POSTLUDE_NOLOCK` states so it can't re-cache and shove a floor-level drone back up.
5. **Homing settles:** `RETURN_TO_ORIGIN` now runs `PLAN -> TURN -> SETTLE -> ADVANCE -> SETTLE -> PLAN` (the
   ADVANCE->SETTLE->PLAN settle is the "turning like a maniac" fix — never re-aim on a just-moved stale pose).
   Postlude settles use the lost-SLAM-tolerant `require_fast=False` (fresh capture, no cap) so a slow-but-alive
   pose still makes progress and a true loss diverts to POSTLUDE_LOST_HOLD.
   (DOCK/LOW_STANDOFF/DONE keep their existing PULSE/REST + timers — nothing flies a stale-pose turn after them.)

Sequence: `done -> RETURN_TO_ORIGIN (home at altitude, settles between turns/advances) -> ORIENT_HOME (settle)
-> DOCK_FLOOR (survives loss, no re-inflate) -> LOW_STANDOFF -> DONE`.

## Self-tests (all green; extended `autopilot.py --self-test`)
- Shared gate via the SETTLE refactor (session-15 SETTLE tests unchanged-green).
- **RECOVERY inter-action settles:** REWIND holds between pops; FALLBACK holds between attempts + STUCK reached;
  bounded-escape ends the settle when the pipeline is dead.
- **POSTLUDE:** happy path now `RETURN_TO_ORIGIN->ORIENT_HOME->DOCK_FLOOR->LOW_STANDOFF->DONE`; homing
  turn+SETTLE+advance; home_max_s / dock_max_s caps.
- **POSTLUDE loss-survival + orient:** ORIENT_HOME takes the SHORT wrap-way turn then docks (bearing-wrap math);
  DOCK survives an injected PLAN-STALE -> POSTLUDE_LOST_HOLD -> resume; no floor re-inflate (`target_altitude_y`
  stays None through the descent).
All six modules green: `autopilot / flow_contact_detector / frontier_planner / ground_grid / flight_replay /
perception`.

## Verification / live-fly (PENDING)
`python fly.py`, press `m`, let a mission complete. Watch: (a) it homes AT altitude, faces the take-off heading,
descends gently, bumps up, stands by — no descend-in-place, no jump-up; (b) a deliberate SLAM loss during the
dock -> HOLD + resume, never recovery; (c) whenever recovery fires, a NEUTRAL settle sits between every rewind
step and every spin attempt (no back-to-back); (d) the homing loop no longer re-aims on a stale pose.
