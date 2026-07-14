# Plan — Return-to-origin + graceful dock (session 14 diagnosis)

> **BUILT in session 16** (`plans/session16-settle-between-stages-and-return-to-origin.md`), together with a
> SETTLE between every homing/orient action (and the recovery mechanisms). All four fixes below were
> implemented; a new `ORIENT_HOME` state + `POSTLUDE_LOST_HOLD` loss-survival + `_POSTLUDE_NOLOCK` anti-
> re-inflate landed. All module self-tests green; **live-fly PENDING.** The design below is retained as the
> diagnosis of record.

## Context
On flight `20260713_223231` the post-mission ending was wrong. Desired: home to origin AT NORMAL FLYING
HEIGHT, orient to the take-off heading, descend GRADUALLY, then a slight up-bump — like a controlled reverse
of take-off. Instead the drone descended in place, lost SLAM, dropped into recovery, then looped
land/crawl/jump. Root causes (all confirmed from the timeline):

1. **Homing skipped.** `home_reach_dist = 1.0` (config.yaml:212, defaults to `goal_reach_dist=1.0`). At
   mission-complete the drone was at `pos=[-0.39,-0.76]`, **0.86 u from origin** → `RETURN_TO_ORIGIN`
   immediately logged *"reached origin (d=0.86)"* and jumped to `DOCK_FLOOR` — it never homed. It began
   descending from flying height (`pos_y≈-1.66`) at the wrong spot.
2. **Dock can't survive a SLAM loss.** The pulsed descent (which itself works — `-1.66 → -1.0` gently) lost
   the plan at `pos_y≈-1.0` → the global guard forced `HOLD_LOST → FALLBACK` (generic recovery hijacked the
   dock), same class of bug as the calibration one `CALIB_LOST_HOLD` already fixed.
3. **Altitude-lock re-inflates at the floor.** After a floor-level recovery, re-entering `RETURN_TO_ORIGIN`
   still holds the flying-height `target_altitude_y (≈-1.9)`; the lock sees the floor-level drone
   (`pos_y≈-0.03 > -1.9 + drift`) and injects UP → the drone jumps back toward flying height → docks again →
   loses SLAM → repeats. This is the land/crawl/jump loop.
4. **No orient-to-take-off-heading step** exists anywhere in the postlude.

## Desired sequence
`done → RETURN_TO_ORIGIN (home at flying height, altitude held) → ORIENT_HOME (turn to the recorded take-off
heading) → DOCK_FLOOR (gradual pulsed descent, SURVIVES a SLAM loss) → LOW_STANDOFF (slight up-bump) → DONE`

## Fixes (autopilot.py + config.yaml)
1. **Tighten `home_reach_dist`** to ~0.5 (general tolerance, NOT a room answer) so the drone actually flies
   home at altitude before docking. Keep `home_max_s` as the visible cap → "dock HERE" if it truly can't
   reach (no silent fallback).
2. **Record `_takeoff_heading`** at ARM/TAKEOFF (the SLAM `heading_deg` once the prelude is airborne + SLAM
   OK — settle-gated so it isn't a wobble reading). Add an **`ORIENT_HOME`** state after homing: open-loop
   ≤`turn_step_deg` turns toward `_takeoff_heading`, then → `DOCK_FLOOR`. (Reuse `_quantize_turn`/`_build_turn`
   and the `_wrap180`/bearing math already in `RETURN_TO_ORIGIN`.)
3. **Give `DOCK_FLOOR` a loss-survival hold**, mirroring `CALIB_LOST_HOLD` (autopilot.py ~`_step_calib_lost`):
   on a plan loss during the dock, HOLD and watch the SLAM `slam_ms` pulse; resume the pulsed descent once
   SLAM solves fast AND plan is OK. Never hand the dock to the generic `HOLD_LOST/FALLBACK` recovery. Add a
   `_dock_interrupted` flag paralleling `_calib_interrupted`, and route a loss while `st in POSTLUDE_STATES`
   into the dedicated hold instead of the global guard (the global guard at step() top currently forces
   `HOLD_LOST` for LOST/NO-PLAN — add a postlude exception like the calib one at autopilot.py:~1424).
4. **Disable the flying-height altitude-lock in the postlude.** Clear `target_altitude_y = None` on
   `DOCK_FLOOR` entry (or gate the lock off whenever `st in POSTLUDE_STATES`) so it can't fight the descent or
   re-inflate a floor-level drone. Re-homing after a recovery must not jump up.

## Self-tests (extend autopilot.py `--self-test`, near the postlude tests ~line 2900)
- **Home-from-away:** start away from origin with a healthy pose → `RETURN_TO_ORIGIN` runs a turn+advance at
  altitude (altitude-lock injects UP while sunk) → reaches within the tightened `home_reach_dist` →
  `ORIENT_HOME` turns toward `_takeoff_heading` → `DOCK_FLOOR`. Assert the ORIENT_HOME turn math (bearing
  wrap) — drive the branch that computes the angle (the session-10 lesson).
- **Dock survives a loss:** inject a PLAN-LOST mid-`DOCK_FLOOR` → the dedicated hold (NOT `FALLBACK`), then a
  fast-frame + OK recovery resumes the pulsed descent; `pos_y` keeps sinking toward the floor.
- **No up-inject once docking:** with `target_altitude_y` cleared, a floor-level pose in the postlude does NOT
  emit `joy_vertical=-1` (no re-inflation).
- **Happy-path subsequence:** `done → RETURN_TO_ORIGIN → ORIENT_HOME → DOCK_FLOOR → LOW_STANDOFF → DONE`.

## Verification
Offline self-tests green, then a live re-fly: at mission end the drone homes at altitude, faces the take-off
heading, descends gently, bumps up slightly, and stands by — no descend-in-place, no recovery loop, no
jump-up. Watch a deliberate SLAM loss during the dock: it holds + resumes, never falls into recovery.
