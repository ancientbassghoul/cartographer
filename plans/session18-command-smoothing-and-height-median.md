# Session 18 — Manual-style command smoothing for autonomy + a real height-median

_BUILT 2026-07-15. io_bridge + autopilot + flight_replay self-tests green. LIVE-FLY PENDING._

## Context / why

On the `20260715_103644` flight (post session-17 triggerDown fix) the operator observed autonomous flight is
height-erratic — a hard brake + pitch-up + altitude jump on every stop and every plan-loss — while his MANUAL
flight is "very very controlled." Two independent problems, fixed together:

1. **Autonomy never got the manual stick smoothing.** Diffing the `20260715_001039` manual command CSV showed
   manual flight is smooth because a keypress only toggles the `trigger_down`/`reverse_down` (and arrow) GATES,
   and io_bridge's 60 Hz loop RAMPS the analog toward them — trigger/reverse `+0.05`/tick attack, `−0.1`/tick
   decay; yaw/pitch `±0.05`/tick (a persisting AIM, snapped to 0 by `c`). The autopilot BYPASSED it:
   `_apply_autonomy_overlay` hard-wrote the analog *after* the ramp, and `_neutralize_autonomy` snapped to 0.
2. **The debugger's drone-height median was nonsense** — appended every ~50 Hz tick with no frame dedup (~25
   re-appends of one stale pose) and seeded with ~0 ground samples pre-takeoff. Confirmed: timeline idx 972→973
   both carry `frame_id=2486` yet the median marches −0.0085 → −0.896 → −1.783.

Both comply with NO-MANUAL-DATA-LEAKAGE: the ramp constants are the sim's own platform control-dynamics (copied
from the manual input model), not a room-specific answer; the median is a general live measurement.

## What was built

### Fix 1 — smoothing in `io_bridge.py` (THROTTLE only: trigger, reverse)
- `_ramp(cur, target, up, down)` module helper (rise by `up`, fall by `down`, no overshoot).
- Two ramp targets in `__init__`: `_auto_trigger_target` / `_auto_reverse_target`.
- `_update_controls` → thin loop over a new **`_step_controls()`** (one tick, testable). In AUTONOMY it chases
  the trigger/reverse targets `(0.05, 0.1)`; **yaw/pitch are applied directly by the overlay, NOT ramped**
  (`btnCdown` still snaps aim to 0). MANUAL runs the original key-gated ramp **verbatim** (byte-identical feel).
- `_apply_autonomy_overlay` fresh-path sets trigger/reverse as TARGETS; everything else (yaw, pitch, boolean
  gates, buttons, joysticks) applied directly. `trigger_down`/`reverse_down` still gate Unity thrust, held True
  while the analog ramps up = identical to manual.
- `_neutralize_autonomy`: throttle bleeds down via targets=0 (smooth release — the operator's ask); aim axes +
  gates snap to neutral immediately (a lost link must not coast a spin/thrust).
- **Gas-gate timing fix (2nd live pass):** `trigger_down`/`reverse_down` are now derived in `_step_controls`
  from the **RAMPED** analog (`gate = ramped_analog > 0`), NOT the autopilot's commanded gate. Previously the
  gate dropped to False the instant a stop was commanded, while the analog kept decaying for ~4 ticks — and
  since Unity gates real thrust on the boolean (session 17), that hard-cut the thrust and defeated the smooth
  release (the suspected plan-lost pitch-up). Now the gate stays True until the analog reaches 0, so Unity's
  thrust follows the smooth decay. **Hypothesis to confirm live** (Unity's model is opaque; harmless if Unity
  instead follows the analog). The autopilot's `_full_vector` derivation stays (harmless, now overridden by io_bridge).

**Why NOT yaw/pitch (live finding, flight `20260715_130330`):** the sim's yaw is not a magnitude axis — the
drone rotates at a fixed rate ONLY once the aim SATURATES at ±1, and the turn amount is set by how long it's
HELD there. Unity also eases the aim itself. An io_bridge yaw ramp therefore (a) double-smoothed on top of Unity
and (b) delayed the aim reaching ±1 by ~0.33 s, leaving only ~0.17 s of a ~0.5 s hold at full deflection → turns
collapsed 30°→~5°. The `flight_playbook.json` turn recipe (`turn_left/right`: `yaw ±1, 1.625 s`) was calibrated
for "snap yaw to 1.0, no ramp" (its DERIVATION note). Passing yaw/pitch straight through restores that. Also
confirmed: the plan-lost **pitch-up is Unity's momentum brake** (0 non-zero pitch rows in the outgoing log);
trigger DOES decay smoothly on neutral.

### Fix 2 — `--log-commands` (`io_bridge.py`, `fly.py`)
Re-added the reverted session-17 outgoing-packet logger: one row per 60 Hz send of the ACTUAL post-ramp packet
(`mono_ts, source(MANUAL|AUTO|AUTO(STALE)), pitch, yaw, trigger, triggerDown, ...`) to
`OUTPUT/diag/<ts>_commands.csv`. `--log-commands` argparse flag; wired always-on into `fly.py`'s io_bridge launch.

### Fix 3 — height-median (`autopilot.py`)
- `_height_calibrated` + `_last_alt_frame_id` in `__init__`.
- Ingest gate (was `autopilot.py:1661`): `_height_calibrated and not _calib_active and healthy` **and one append
  per FRESH `frame_id`** (dedup). `MAPPING_ALT_STATES` state-gate retired (measure in any state).
- Latch `_height_calibrated = True` at the CALIB_VERIFY **PASS** and the **abandon-after-retries** branch (the
  first calibration itself needs no baseline — `:1907` already PASSes "insufficient baseline → cannot judge").
- `_alt_median` + per-step logging unchanged; the median now simply steps once per SLAM frame.

### Tests
- `io_bridge.py --self-test`: trigger ramps `0→0.2` at 0.05, holds, decays at 0.1; yaw ramps 0.05 + `c` snap;
  manual unchanged. **PASS.**
- Rewrote the autopilot ingest-gate unit test (not-calibrated→0, calibrated+2-fresh-frames→2 with repeat
  deduped, frozen-during-calib→0). Full autopilot suite + flight_replay **PASS.**

## Verification (live — pending)
1. `python fly.py`, press `m`. Open `OUTPUT/diag/<ts>_commands.csv`: AUTO rows must show `trigger` ramping
   (0.05 up / 0.1 down) and yaw ramping 0.05/tick — matching the MANUAL curve, not hard steps.
2. Forward legs + turns EASE in/out; a plan-loss brake is markedly gentler (thrust bleeds instead of snapping).
3. Replay HTML: drone-height median steps smoothly one update/SLAM-frame, converging to live `pos_y` — no
   −0.008→−1.8 jump, no frozen lag.

## Turn-strength knobs (if needed after the yaw-ramp removal)
With yaw un-ramped the calibrated turns are restored, so a config change may not be needed. If the drone still
under/over-rotates, tune EMPIRICALLY:
- `flight_playbook.json` → `turn_right`/`turn_left` `duration_s` (currently `1.625` — the yaw HOLD time at full
  deflection; the direct physical knob).
- `config.yaml` → `turn_recipe_deg` (currently `90.0` — the calibration "1.625 s ≈ 90°"; LOWER it to rotate MORE
  per requested degree, since `hold = turn_hold_dur * |theta| / turn_recipe_deg`).

## CAVEAT — throttle re-tune
Throttle (trigger/reverse) is still ramped, so short THROTTLE pulses are gentler/attenuated; expect the
already-pending session-17 "lower the speed knobs" pass. Turn durations are UNAFFECTED (yaw no longer ramped).
