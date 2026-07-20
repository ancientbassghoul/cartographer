# Session 31 — kill REWIND (config-gated), replace FALLBACK with a simple 4-phase sweep

## Origin

The operator reported REWIND (replaying the inverse of recent commands on a PLAN-STALE, to re-expose the
camera to keyframes it already recorded) has never once visibly helped recover a stale plan, across many
real flights. Separately, session 29's FALLBACK rebuild — a shuffled-direction-queue with per-direction
tries and an opposite-phase retry — tested badly live: locking a push direction across several tries
produced bad, unpredictable results ("It's unpredictable, so I rather leave it to chance").

## Design

**REWIND**: gated behind a new `use_rewind_on_stale` config boolean, default `false` — the operator's own
suggested approach, code kept (not deleted) so it's one config edit to bring back. `_step_stale()` now
checks `_ever_tracked` (the pre-existing startup WARMUP guard) FIRST, then the REWIND flag — order matters:
REWIND being off must not bypass "don't blind-sweep before SLAM has ever tracked" at startup.

**FALLBACK**: replaced the direction-cycling search with the operator's own exact algorithm — a simple
4-phase cycle: wait 20s → turn 15° → push a FRESH random direction (fwd/bkwd/lft/rt, re-rolled every single
cycle, no per-direction budget or opposite-phase retry) → wait 10s → repeat until cumulative commanded
rotation reaches 720° → STUCK. The live wall/backwall-contact early-exit stays (operator: "leave it for the
slim chance we WILL sense the wall") — confirmed via `flow_contact_detector.py` that it needs ~1.2s of
sustained motion to latch (`arm_blank_s` 0.4s + `contact_seconds` 0.8s), which the new 2.0s forward/backward
push duration now comfortably clears (the old 0.5s never could). A genuine recovery (`status == OK`) exits
the sweep immediately via the existing generic recovery convergence — no new code needed for "stop waiting
the moment it's OK."

**Push throttle/duration**, operator-specified from a live manual-flight comparison: forward/backward full
throttle (1.0) held 2.0s including ramp-up; left/right full magnitude (±1.0) held 0.5s (`joy_horizontal`
isn't ramped at all, unlike `trigger`/`reverse`). All four bypass the throttled knobs (`reverse_throttle`,
`_strafe_mag`) — those stay unchanged for every other site that uses them (`PARALLAX_PUSH`, etc.).

**Flicker-safety**: PLAN-STALE↔PLAN-LOST commonly flickers mid-episode, bouncing the drone through
`HOLD_LOST` (a separate top-level branch) and back. `_fallback_phase`/`_fallback_cum_deg`/`_fallback_cycle`
PERSIST across that bounce — a fresh episode only initializes when `_fallback_phase is None`; a resume just
re-enters `"FALLBACK"` and continues wherever it left off. Each phase rebuilds `self._player` if `None`
(build-if-None, matching `REVERSE_PROBE`'s existing pattern) rather than assuming a player survived the
interruption.

## Built

**`config.yaml`**: removed `fallback_retreat_s`, `fallback_dir_tries`, `fallback_max_attempts`. Added
`use_rewind_on_stale` (false), `fallback_initial_wait_s` (20.0), `fallback_post_push_wait_s` (10.0),
`fallback_max_rotation_deg` (720.0), `fallback_push_fwd_back_s` (2.0), `fallback_push_strafe_s` (0.5).

**`autopilot.py`**: rewrote `_step_stale()`'s fresh-entry logic (REWIND flag gate, reordered ahead of the
WARMUP guard fix below); removed `_FALLBACK_OPPOSITE`, `_reset_fallback_dir_search()`, `_begin_fallback()`;
added `_reset_fallback_sweep()`, `_enter_fallback_sweep()`, `_step_fallback_sweep()` (the new phase-timer
dispatch: `INITIAL_WAIT → TURN → PUSH → WAIT_POST → (loop or STUCK)`). Updated the 3 reset call sites
(`reset_leg()`, confirmed-recovery in ADVANCE, fresh-episode arm) to use `_reset_fallback_sweep()`.

**Bug found + fixed while wiring the REWIND gate**: the first draft checked `use_rewind_on_stale` before
`_ever_tracked`, so with REWIND off (the new default) a PLAN-STALE at startup — before SLAM had ever
tracked — went straight into the blind FALLBACK sweep instead of holding in WARMUP. Reordered so the
startup guard always fires first, regardless of the REWIND flag.

## Self-tests

Replaced the session-29 "FALLBACK direction-cycling search" block with a new "FALLBACK sweep (session 31)"
block: initial wait holds for `fallback_initial_wait_s` then starts turning; a full cycle visits
TURN→PUSH→WAIT_POST and accumulates `_fallback_cum_deg`; live contact matching the in-flight push direction
ends it early (a mismatched contact does not); 720° with no recovery → STUCK; a flicker through `HOLD_LOST`
mid-phase does not reset `_fallback_cum_deg`/`_fallback_cycle`. All REWIND-touching pre-existing tests
(`RECOVERY control-space`, `RECOVERY inter-action settles`, `SESSION-12`'s consuming-drain + ghost-path-guard
cases) got an explicit `ctrl.use_rewind_on_stale = True` override to keep exercising that code path despite
it now being default-off; several also needed `ctrl._ever_tracked = True` after the WARMUP-ordering fix,
since a fresh test controller otherwise reads as "startup" and diverts to WARMUP instead of the path under
test.

`python autopilot.py --self-test` and `python io_bridge.py --self-test`: **ALL PASS**.

## Verification

No hardware/GPU in this environment — self-tests only. Live-fly checklist for next flight:
- Does a PLAN-STALE episode now go straight to the turn/push/wait cycle without a REWIND detour.
- Does the push direction genuinely look randomized attempt-to-attempt (not stuck repeating one direction).
- Does a forward/backward push now visibly carry enough authority to matter (full throttle, 2s) vs. the old
  throttled/short one.
- Does recovery still cut the sweep short immediately once status reads OK (no waiting out
  `fallback_post_push_wait_s`).
- Total time-to-STUCK for a real stuck episode should roughly match 48 cycles × (turn ~0.5s + push 0.5–2.0s
  + 10s wait) + 20s ≈ 9–10 minutes worst case — these durations came from the operator's own judgment call,
  not a live measurement; flag if that feels too long/short in practice.
