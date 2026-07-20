# Session 21 — Restore periodic height re-calibration + gradual height TRIM + the height debugger panel

## Context

The drone does NOT hold its altitude — it sags in flight, wrecking mapping runs. Session 17 (`a737aa4`) deleted
the periodic height re-calibration trigger, all of the gradual height TRIM, and the ceiling/desired/delta
debugger rows, on the belief that the sag was self-inflicted (thrust never engaged because `triggerDown` was
unset) and would vanish once thrust worked. Live flights proved the sag is real, so the machinery is restored
from `44b4fa6` (sessions 14–16) onto `leg-hops-and-goal-commit-fix`, adapted to the session-18 median and the
session-20b hop/pick-pulse REPLAN. Operator decisions: restore BOTH mechanisms; the 60 s cooldown stays a
**configurable** knob (`calib_cooldown_s`); every height reference stays LIVE / self-calibrating (HARD RULE — no
baked room values). The session-20b work was committed first (`592b0c0`).

## What was restored (and what was already there)

Already on the branch, wired: CALIBRATING_HEIGHT / DESCEND / CALIB_VERIFY / CALIB_LOST_HOLD / ASCEND_ESCAPE /
CALIB_TRANSLATE / CALIB_ESCAPE, the escape/STUCK escalation, and the session-18 rolling flying-height median.
Restored:

1. **Periodic re-calibration trigger** (`autopilot.py` REPLAN): on a GENUINE goal change (moved >
   `calib_goal_change_dist` = 1.0u from `_leg_goal_prev`) past the `calib_cooldown_s` (60 s) cooldown →
   `CALIBRATING_HEIGHT` (re-tap the ceiling) → DESCEND → REPLAN re-enters with the SAME goal → normal orient.
   Config: `calibrate_on_goal_change` / `calib_cooldown_s` / `calib_goal_change_dist`.
   - **Session-20b integration:** the recalib REPLAN emits a **hop-outcome-only pulse** (`pick_goal=None`,
     carrying the finished hop's strike/progress judgment); the leg's PICK registers post-calib when the normal
     branch re-commits the same goal — one pick per leg, still. `perception_worker`'s drain now handles the
     hop-outcome and pick parts of a pulse independently.
   - **review-A:** `cooldown_ok = _last_calib_t is None or elapsed >= cooldown` — a drone that NEVER calibrated
     (`--no-takeoff` / failed prelude) is allowed to calibrate, not locked out; a normal takeoff's ASCEND sets
     `_last_calib_t`, so the first post-prelude goal is still cooldown-gated.
   - **review-D:** the post-calib resume is smooth by design — ASCEND/DESCEND is vertical-only (heading kept), so
     bearing-err ≈ 0 → `_turn_steps(0)` = the `'c'`-only attitude reset. Locked by self-test.
2. **Live height references**: `_ceiling_y` captured in ASCEND (clean flow latch / latch-hold stall / ascend cap
   — the climb peak), `_desired_y` = the settled post-descend pose at a CALIB_VERIFY PASS, `_trim_delta` =
   desired − ceiling, re-measured every calibration; the PASS logs `HEIGHT-CALIB values: ceiling/desired/delta`
   LOUD (terminal + HTML). All LIVE, flight-level (persist across reset_leg), never baked.
3. **Gradual TRIM** (`_TRIM_TRIGGER_STATES`={SETTLE, ADVANCE}; trigger before the state dispatch; `TRIM` state;
   `_trim_exit`): pos_y sank past `ceiling_y + trim_sag_ratio*delta` on a fresh healthy frame → ring-gate
   (fwd-open → climb; else reverse/strafe to open forward room; else abort visible) → AIM (pitch up) → FWD
   (forward push with pitch up — flow/ram guards stay ACTIVE; `trigger_down` derives centrally in `_full_vector`,
   so the climb push engages real thrust) → RESET ('c') → WAIT (a healthy frame CAPTURED ≥ `_trim_cmd_t0 +
   trim_settle_s` — review-C: phase-relative, stale pre-TRIM frames can't exit early) → `_trim_exit` re-aims
   ORIENT at the goal snapshotted on entry (Trap B — never re-picks, can't pollute the goals-DB).
   - **review-B:** the trigger None-guards every reference — refs are None until the first CALIB_VERIFY PASS, so
     TRIM cannot fire (or crash) pre-calibration.
   - **Session-20b integration:** TRIM entry clears the pending per-hop progress eval (`_hop_start_goal`) — a
     trim-interrupted hop is NOT judged (no false strike), same rule as a plan-loss.
4. **Debugger** (`flight_replay.py` HEIGHT CALIBRATION panel): live `pos_y` (reddens past the sag threshold),
   `ceiling` / `desired` / `delta`, `trim-at` (= ceiling + 1.2·delta), `median`, and an `active` TRIM/CALIB flag.
   Carried via `alt_*`/`trim_on`/`calib_on` step-record fields (old logs degrade to —).

## Self-tests (all 6 module suites green)

- `explore HEIGHT-TRIM`: sag→TRIM + goal snapshot + hop-eval-clear; no-sag no-fire; **None-refs no-op
  (review-B)**; calib/state suppression; reverse/strafe reposition; ring-blocked abort; climb emits
  pitch → forward **with triggerDown** → 'c' then re-aims the preserved goal; cap_ts=None holds (review-C).
- `PERIODIC-RECALIB`: goal-change+cooldown → CALIBRATING_HEIGHT with the hop-outcome-only pulse; same-goal /
  in-cooldown gates; **never-calibrated allowed (review-A)**; post-calib resume θ≈0 'c'-only (review-D).
- Harness note: `run_self_test` disables `calibrate_on_goal_change` globally (the restored trigger would divert
  every unrelated leg test into CALIBRATING_HEIGHT); the PERIODIC-RECALIB tests re-enable it explicitly.
- `flight_replay`: the `alt_ceiling/desired/delta` fields survive load + the panel/sag-threshold code renders.

## Live-fly verification (pending)

`python fly.py`, `m`: after the first calibration the HEIGHT CALIBRATION panel shows live
ceiling/desired/delta/median; a genuine goal-change past the 60 s cooldown re-taps the ceiling (watch
`HEIGHT-CALIB values:` in the log); a sag past `desired + 0.2*delta` fires TRIM (pitch-up + forward climb,
`trim-at` threshold + red pos_y in the panel) and re-aims the SAME goal; altitude visibly recovers and holds.
Watch the interaction with hops: TRIM fires from SETTLE/ADVANCE between hops; a trimmed hop takes no strike.
