# Plan — Session 15 fixes — BUILT + all module self-tests green, LIVE-FLY PENDING

## Context
Session-14 TRIM flew (`OUTPUT/diag/20260714_113312_timeline.jsonl`). Six fixes came out of it, all built +
self-tested this session. (`plans/return-to-origin-and-graceful-dock.md` and
`plans/blacklist-region-and-counter.md` remain separate future sessions.)

## Fix 1 — TRIM pitch axis was reversed (drone descended)
`trim_pitch_up = +1.0` aimed the crosshair DOWN. Flipped to **-1.0** (config.yaml + the `e.get(..., -1.0)`
default). Aim up + forward now CLIMBS.

## Fix 2 — Calibration endless-loop → escape push, then STUCK  (autopilot.py)
SLAM getting badly confused during a re-calibration looped forever (finish/interrupt → lose plan →
`CALIB_LOST_HOLD` redo → …). Bounded it:
- New flight-level `_calib_fail_streak` / `_calib_escaped` (+ config `calib_escape_after` 3,
  `calib_escape_ok_frames` 12, `calib_escape_push_s` 1.0).
- Shared helper **`_calib_fail_escalate(now, why)`**: a failed attempt bumps the streak → REDO while < N;
  **`CALIB_ESCAPE`** at N (first time); **`STUCK`** at N after an escape already ran. Used by both
  `_step_calib_lost` (loss-interrupted) and CALIB_VERIFY (Fix 3b).
- New state **`CALIB_ESCAPE`** (`_step_calib_escape`): one ring-picked parallax push to a fresh vantage
  (backward if pushable, else strafe, else hold), then HOLD indefinitely until `_slam_fast_streak >= 12` AND
  `status == OK`, then RETRY the calibration. It is checked at the step() top **before** the calib-lost divert
  so a loss during the escape doesn't bounce it back into `CALIB_LOST_HOLD`. `_calib_active` stays True through
  it (baseline ingest stays frozen).
- Streak/escaped reset on a real `CALIB_VERIFY` PASS and in `reset_leg`. STUCK reuses the existing state (holds,
  per-step logging paused via the session-12 D4 block); `_calib_escaped` persists so it can't loop.

## Fix 3 — SETTLE waits for 6 FRESH SLAM frames (no fly on a stale pose)  (autopilot.py)
A leg SETTLE proceeded on a 1 s timer with SLAM ~2 s stale → shaky ORIENT → plan loss. Now: a settle whose
`nxt` flies toward a goal (REPLAN / REVERSE_PROBE / …) proceeds only after **`settle_fresh_frames` (6) SLAM
"done" frames CAPTURED after settle entry (`cap_ts >= _settle_t0`) AND `slam_ms < slam_slow_ms`**, plus the
`rest_between_s` floor. Frames captured before the settle never count. No timeout on a gated settle — if SLAM
stops, the plan status goes STALE/LOST and the step() top diverts to recovery. The **vertical prelude/calib
routine is EXEMPT** (`_SETTLE_EXEMPT_NXT = {TAKEOFF, ASCEND, DESCEND, BASELINE_NUDGE}`) and keeps its timed
settle — verified from the flight that SLAM tracks from the first prelude tick, so this is by state-role, not a
deadlock concern. Per-settle counters reset in `_enter` on SETTLE entry. **Test harness:** `_drive` now injects
an advancing `frame_id` + `cap_ts` + fast `slam_ms` (mirrors real flight) so the gate is exercised; the
manual-loop REVERSE-PROBE test injects the same.

## Fix 3b — CALIB_VERIFY must not fly on a stale pose  (autopilot.py)
The two `calib_verify_max_s` timeout branches that PASSED-and-flew to REPLAN with no settled healthy pose now
resolve to **`TIMEOUT_FAIL` → `_calib_fail_escalate`** (a failed attempt → escape/STUCK guard), never a
goal-flying leg on an unsettled pose. The normal settled+healthy PASS/FAIL judge is unchanged. Other timers
(`ascend_max_s`, `baseline_nudge_max_s`, `dock_max_s`) stay as logged safety caps (operator Q2).

## Fix 4 — Debugger live height numbers  (autopilot.py + flight_replay.py)
Removed the unhelpful `Δpos / Δgoal` group. The timeline now logs `alt_ceiling / alt_desired / alt_delta`
(the calibration references, change between successful calibrations) + `alt_median` (the all-flight rolling
`_mapping_altitude_history` median — the baseline CALIB_VERIFY judges against, updates every frame, via the new
`_alt_median` property). `flight_replay` renders them as a **HEIGHT CALIBRATION** number group (None-safe on
old logs).

## Fix 5 — fly.py console flood
Restored `creationflags=NEW_CONSOLE` on the visualizer + io_bridge Popen calls (io_bridge printed its per-command
`[AUTO]` snapshot into the shared launcher console).

## Self-tests (all green)
`autopilot.py --self-test` gained: **SETTLE fresh-frame gate** (stale-holds / 6-fresh→REPLAN /
exempt-vertical-timed) and **CALIB_ESCAPE/STUCK** (3-fails→escape / push+12-hold→retry / escape+3-fails→STUCK);
the CALIB_VERIFY `cap_ts=None` test now asserts escalate-not-fly. All modules green:
`autopilot / flow_contact_detector / frontier_planner / ground_grid / flight_replay / perception`.

## Verification / live-fly (PENDING)
`python fly.py`, press `m`: (a) a TRIM now CLIMBS; (b) the launcher console is quiet (io_bridge in its own
window); (c) a leg SETTLE visibly waits for 6 fresh <1000 ms frames before ORIENT; (d) a looping re-calibration
escapes after 3 fails (push + hold for SLAM), retries, and after 3 more reports STUCK + stops logging; (e) the
replay's HEIGHT CALIBRATION panel shows live ceiling/desired/delta + a constantly-updating median.

## Parked (diagnose AFTER this flight's log)
Reverse commands fired back-to-back without settling — check WHICH reverse path on the next log; the Fix-3
SETTLE gate may already have tamed the ones that end `→ SETTLE → REPLAN`.
