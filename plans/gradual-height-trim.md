# Plan — Gradual height TRIM (session 14) — BUILT + all module self-tests green, LIVE-FLY PENDING

## Context
On flight `20260713_223231` the drone gradually LOST height. Calibration only re-taps the ceiling on a goal
change (60 s cooldown) — a last-minute save — and ~half the flight sat in SLAM_HOLD/HOLD_LOST, where the
discrete `joy_vertical` altitude-lock never corrects, so the drone sagged. `joy_vertical` is a DISCRETE
full-thrust ±1 axis (the reason the two-phase pulsed ascent exists), so we had no *gradual, dose-able*
vertical primitive.

Operator's idea (built): use the sim's PITCH aim. Pitch the aim UP and push forward → the drone flies toward
the raised aim = a GRADUAL climb (rate set by the push duration). Bonus: the forward part is translation =
SLAM parallax (a pure `joy_vertical` pulse stretches vertical features + chokes SLAM). `pitch` is already a
plumbed control field (`io_bridge.py:102/199/397`); `c`/attitude-reset is available. No new plumbing.

## What was built (autopilot.py + config.yaml)

### Three references captured at every calibration (flight-level, persist across `reset_leg`)
- `_ceiling_y` — the climb peak, captured at the ASCEND ceiling-latch (and the two fallback ASCEND exits).
- `_desired_y` / `_trim_delta` — captured ONLY at the `CALIB_VERIFY` height-OK pass (settled + `status==OK`,
  so never the raw tap / post-bump wobble — Trap D). `delta = desired_y - ceiling_y` (+Y down ⇒ delta > 0).
- Logged LOUD on the PASS: `HEIGHT-CALIB values: ceiling_y=… desired_y=… delta=…` (terminal + replay HTML).

### Sag trigger (`_TRIM_TRIGGER_STATES = {SETTLE, ADVANCE}`)
On a fresh healthy frame, not `_calib_active`, if `pos_y > _ceiling_y + trim_sag_ratio*_trim_delta`
(== `desired_y + 0.2*delta`), snapshot the committed `leg_goal` (Trap B) and enter `TRIM`.

### `TRIM` state machine (autopilot.py, coded sub-phases)
`ring-gate` → `REPOS?` → `AIM` → `FWD` → `RESET` → `WAIT`:
- **ring-gate:** fwd-open → climb; else back-open → reverse to open forward room; else a side open → strafe to
  it; else **abort** (VISIBLE "skip trim (pray)"), resume. Uses `plan.clearance_ring` + `forward_clearance_dist`
  with the None-is-open convention.
- **AIM:** hold `pitch=trim_pitch_up` for `trim_aim_s`; then record `_trim_cmd_t0 = now`.
- **FWD:** forward push (`trigger=trim_throttle`) WITH pitch still up for `trim_fwd_s` — the climb. A wall/ram
  contact aborts the push to RESET; the flow/ram guards stay active (Trap A — not suppressed).
- **RESET:** pulse `c` (`btnCdown`) for `trim_reset_s` to reset the aim.
- **WAIT:** hold neutral until a healthy frame with `cap_ts >= _trim_cmd_t0 + trim_settle_s` (None-guarded;
  same monotonic clock as the CALIB_VERIFY settlement gate — Trap C), then LOG the post-trim height.
- **exit (`_trim_exit`):** restore the snapshotted goal and re-aim **ORIENT** at it (never re-pick — Trap B);
  SETTLE→REPLAN only if no goal was committed / pose unavailable. `_trimming` is telemetry, defensively
  cleared whenever `st != "TRIM"` so a trim abandoned by a mid-trim SLAM loss can't leave it stuck True.

### Config (config.yaml `autonomy.explore`)
`trim_enable`(true), `trim_sag_ratio`(1.2), `trim_aim_s`(1.0), `trim_fwd_s`(0.5), `trim_settle_s`(1.0),
`trim_reposition_s`(0.5), `trim_pitch_up`(+1.0 — **confirm sign live**), `trim_throttle`(0.4).

### Logging (§C — simplified: no replay-HTML panel)
Two loud `event` lines (terminal + HTML event panel, no `flight_replay.py` change): the `HEIGHT-CALIB values`
line at each calibration, and `TRIM enter:` / `TRIM done:` (sag ratio, ring decision, before/after `pos_y`).

## Self-tests (all green)
`autopilot.py --self-test` gained a **HEIGHT-TRIM** test: sag→TRIM + goal snapshot; no-sag at desired; suppress
while `_calib_active` / in a non-whitelisted state; reverse-repos; strafe-repos; ring-blocked→abort→ORIENT
(goal kept); full climb emits pitch→forward→`c` then re-aims ORIENT at the preserved goal; `cap_ts=None` holds
WAIT. Also green: `flow_contact_detector / frontier_planner / ground_grid / flight_replay / perception`.

## Verification / live-fly (PENDING)
`python fly.py`, press `m`. Watch a whitelisted-state sag fire a TRIM with the loud logs; **confirm
`trim_pitch_up` sign on the first trim** (if it descends, flip it — the before/after `pos_y` makes it obvious);
confirm the trim re-aims at the SAME goal.

## Diagnosed-but-not-built this session (each its own plan for a fresh context)
- `plans/return-to-origin-and-graceful-dock.md` — the post-mission ending.
- `plans/blacklist-region-and-counter.md` — the 2-minute glass-wall bounce.
