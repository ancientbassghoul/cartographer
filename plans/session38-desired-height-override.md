# Config override for desired flying height (skip/replace live calibration)

## Context

The operator wants a config knob to force a fixed "desired height" instead of letting the
autopilot's live height calibration decide it — useful for testing (e.g. flying at a known,
repeatable height while exercising other features like the visual-recovery probe, without
depending on the ceiling-tap dance each flight). Request: a single float param; `0` (default) =
today's behavior (live self-calibration); any other value = use that value directly as the
desired height. World frame is +Y DOWN (camera convention), so a flying height is a NEGATIVE
value (e.g. `-1.9`, matching the self-test fixtures).

**Where "desired height" lives today** (`autopilot.py`, `ExploreController`): two attributes,
normally set from ONE live measurement:
- `self.target_altitude_y` — the altitude-lock hold target `ADVANCE`'s "push up if sunk"
  correction reads (also what the session-37 visualizer telemetry panel shows as "desired").
- `self._desired_y` / `self._trim_delta` — the bidirectional TRIM's sag/high reference band
  (`_trim_delta = _desired_y - _ceiling_y`).

Both are set in `CALIB_VERIFY` PASS from `settled_y` (wherever DESCEND happened to stop), plus a
fallback latch (only reached if `CALIB_VERIFY` never ran at all, e.g. `--no-takeoff`) that caches
whatever `pos_y` is first seen post-prelude.

**Design: override the VALUE, not the calibration PROCESS.** The ceiling-tap
(ASCEND/DESCEND/`CALIB_VERIFY`) still runs physically and `_ceiling_y` is still measured live —
a legitimate self-calibrating platform signal, not a room-specific answer, and TRIM's sag
detection needs it regardless of the chosen desired height. Only the *desired_y* half of the
measurement gets replaced by the override when non-zero; `_trim_delta` is still computed as
`override − ceiling_y` (one live + one operator-set input), so TRIM's band still tracks the real
ceiling. The fallback latch gets the identical override for consistency.

**Autonomy-standard note:** flagged, and proceeding on the operator's explicit request — a fixed
height is exactly the kind of thing CLAUDE.md's "NO MANUAL-FLIGHT DATA LEAKAGE" section is wary
of. The distinction: explicit, visible, opt-in operator override, default `0` = full live
self-calibration (matches this codebase's existing pattern: `use_rewind_on_stale`,
`use_visual_recovery_on_stale`, `calibrate_on_goal_change`), for testing/debugging — never left
non-zero for a real autonomous survey.

## Changes

- **`config.yaml`**: new `autonomy.explore.desired_height_override_y` (default `0.0`), next to
  the other height-calibration knobs.
- **`autopilot.py`**:
  - `ExploreController.__init__`: `self.desired_height_override_y = float(e.get(...))`, right
    after `self.target_altitude_y = None`.
  - `CALIB_VERIFY` PASS: `self._desired_y = override if override else settled_y` in place of the
    bare `settled_y` assignment; `_trim_delta`/`target_altitude_y` derive from that as before.
    `calib_log` gets an explicit `DESIRED-HEIGHT OVERRIDE: ...` line whenever active — never a
    silent substitution.
  - Fallback latch: same substitution (`self.desired_height_override_y if ... else
    float(plan["pos_y"])`).
- **Self-tests**: extended the existing SESSION-22 `CALIB_VERIFY` PASS block with an override
  case (`desired_height_override_y=-1.5` against a measured `settled_y=-1.9`) confirming
  `target_altitude_y`/`_desired_y` land on the override, `_trim_delta` still derives from the
  live `_ceiling_y`, and the log names the override explicitly.

## A real regression found + fixed along the way (not part of this feature)

Running the full self-test suite after this change surfaced 8 unrelated FAILs (RECOVERY
control-space, SESSION-12, RECOVERY inter-action settles, FALLBACK sweep, SLAM step-back,
SLAM-STEPBACK PERSISTENCE, PROACTIVE CLEARANCE, no-spin startup). Root cause: unrelated to this
feature — the operator had separately flipped `use_visual_recovery_on_stale: false -> true` in
`config.yaml` (to test session 36's visual-recovery path live), and `run_self_test(cfg)` passes
that SAME shared `cfg` into every legacy test, none of which explicitly pin the flag OFF —
exactly the "config-drift" class of gap session 30 already hit once with
`calibrate_on_goal_change` (fixed there by force-pinning it OFF on the self-test's own
`copy.deepcopy(cfg)`, since the dedicated tests for that feature use their own separate deep
copy). Applied the identical fix one line below the session-30 one: force
`use_visual_recovery_on_stale = False` on the self-test's local `cfg` copy — the VISUAL RECOVERY
tests already build their own `cfg_vr = copy.deepcopy(cfg)` and force it back to `True` there, so
nothing about that coverage changes. `config.yaml`'s real, intentional `true` flip is untouched.

## Verification

- `python autopilot.py --self-test`: ALL PASS (confirmed both the new override case and the
  full pre-existing suite, including the fix above).
- `python flight_replay.py --self-test`: ALL PASS (unaffected, spot-checked for safety).
- Manual: set `desired_height_override_y` to a plausible value (e.g. `-1.9`) and confirm on a
  flight (or the timeline JSONL / visualizer telemetry panel) that `target_altitude_y` reads the
  override immediately after `CALIB_VERIFY` PASS instead of wherever DESCEND settled, and that
  TRIM still reacts to sag/high relative to it. Not yet flown (no hardware in this dev
  environment).
