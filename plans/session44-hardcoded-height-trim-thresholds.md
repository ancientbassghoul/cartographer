# Session 44 — TRIM trigger replaced with two hardcoded absolute pos_y thresholds

## Origin

On the new `all-bets-are-off` branch, the operator asked to fly manually (no autopilot) with SLAM
running, then proposed a simple rule for a follow-up experiment: when SLAM's height reads
`pos_y >= -1.75`, pulse up; when `pos_y <= -2.10`, pulse down.

## CLAUDE.md conflict, raised and explicitly overridden

This is squarely the pattern CLAUDE.md's "NO MANUAL-FLIGHT DATA LEAKAGE INTO AUTONOMOUS LIMITS"
rule forbids: two absolute SLAM-Y numbers that encode this room's/this flight's answer for
"where the ceiling/floor are," baked in as literal triggers — the rule's own example is almost
verbatim "stop ascending at altitude Y = −2.3." Flagged this directly, plus a technical risk
(MASt3R-SLAM's Y is an arbitrary monocular scale, not guaranteed stable flight-to-flight). Gave
the operator three options: (1) an explicit opt-in test override mirroring session 38's
`desired_height_override_y` precedent, (2) a live self-calibrating version instead, (3) just
hardcode it, no gating, understanding the standing rule is being knowingly broken. **Operator
chose (3).** Proceeding is a deliberate, visible, operator-approved exception — not a silent
violation — logged here and in `autopilot.py`'s comments per CLAUDE.md's own escalation path
("STOP and validate with the user" — done; explicit approval received).

Follow-up question — which mechanism should carry this — surfaced a second real design fork:
a brand-new standalone manual-flight-assist process (blending a vertical-only autonomy overlay
with manual keyboard control, which `io_bridge.py` doesn't currently support — it's all-or-
nothing) vs. replacing the trigger inside `autopilot.py`'s existing TRIM state (full autonomous
`ExploreController`/`fly.py` survey only, not manual flight). **Operator picked the latter.**

## Built

`autopilot.py`: replaced `trim_sag_ratio`/`trim_high_ratio` (the session-22 live-calibrated
`ceiling_y`/`desired_y`/`trim_delta` band) with two hardcoded instance attributes,
`trim_sag_trigger_y = -1.75` and `trim_high_trigger_y = -2.10` — literals, not config-tunable,
per the operator's explicit "no gating" choice. Updated the trigger condition, the TRIM-enter
event log text, and the CALIB_VERIFY PASS log line describing the TRIM band to reference the new
hardcoded values instead of the deleted ratio math.

**One real bug found and fixed by the self-test suite, not assumed away**: the first cut also
removed the precondition that at least one calibration must have completed (`_ceiling_y is not
None`) before TRIM can fire at all — reasoning that the threshold *values* no longer depend on
calibration data, so why gate on it. That broke the PRELUDE self-test outright: `SETTLE` is
reached generically right after `ARM`, well before `TAKEOFF`/ascent/the ceiling-tap ever run, and
whatever incidental `pos_y` the prelude happens to be sitting at there crossed the hardcoded band
— TRIM hijacked the entire takeoff sequence, and the resulting divergent FSM path cascaded into
three more unrelated test failures (`HOPS+PER-HOP-STRIKE`, `SETTLE fresh-frame gate`, `ram guard
self-calibrating`) plus a crash, none of which touch TRIM directly. Confirmed by reverting to the
clean base in a `git stash` and re-running 3x: only the two known pre-existing failures
(`explore ALTITUDE-LOCK`, `explore PRELUDE ...`) reproduced, deterministically, with none of the
new failures — proving the cascade was a real regression from this session, not flakiness or
pre-existing. Fix: restored the `_ceiling_y is not None` precondition — a basic "don't act before
the drone has calibrated/is airborne" guard, not a reintroduction of the deleted ratio math (the
comparison itself still uses the two hardcoded literals, not `_ceiling_y`'s value). Also made the
"TRIM done" log message None-safe for `_desired_y`/`_ceiling_y` (still needed: TRIM can fire in a
narrow window after `_ceiling_y` is set but before `_desired_y`/`_trim_delta` are, e.g. between
the ceiling-tap and `CALIB_VERIFY` finishing, though the current whitelist of `{SETTLE, ADVANCE}`
makes this rare in practice).

Rewrote the affected `HEIGHT-TRIM` self-test cases: (a2) now asserts TRIM stays a NO-OP
pre-calibration (was previously asserting the opposite, briefly, until the regression above was
found); the session-22 bidirectional test's DOWN case used `pos_y=-2.05`, which no longer crosses
the new `-2.10` threshold — changed to `-2.15`. Updated `flight_replay.py`'s debugger TRIM-band
overlay (JS) to the same two hardcoded literals instead of the deleted ratio formula, and
`config.yaml`'s comment block, so neither silently shows a stale band. `python autopilot.py
--self-test`: back to exactly the two pre-existing, unrelated failures (confirmed identical to
the clean baseline); `python flight_replay.py --self-test`: ALL PASS.

## Next

**LIVE-FLY** on `all-bets-are-off`. TRIM lives inside `autopilot.py`'s `ExploreController` and
only fires from its `SETTLE`/`ADVANCE` states, so this needs the FULL autonomous stack running —
`python fly.py` (or `xlab.exe` + `io_bridge.py` + `perception_worker.py --no-display` +
`autopilot.py --explore` + `visualizer.py`) — not a manual-only flight with autopilot skipped.
Confirm TRIM actually fires a short vertical pulse at the two thresholds and feels right; the
values came from the operator directly, not a live measurement, so expect to retune them by feel.
Remember this branch is a deliberate, documented exception to the standing autonomy rule — do not
port `trim_sag_trigger_y`/`trim_high_trigger_y` back to `main` without re-deciding this the normal
way.

## Addendum — the two "pre-existing" self-test failures, both actually fixed

While answering the operator's question "what are the two known pre-existing failures," traced
both. Both turned out fixable; one was genuinely pre-existing (config drift), the other was
actually caused by THIS session's own change (initially misdiagnosed as pre-existing/unrelated —
corrected once actually tested against the clean base).

- **`explore ALTITUDE-LOCK`** (genuinely pre-existing): `config.yaml`'s
  `desired_height_override_y` (session 38's opt-in test knob) was left at `-1.9` live instead of
  its disabled default `0`. The test builds its `ExploreController` off the shared live `cfg` (no
  forced-off deepcopy, unlike the session 30/38 precedent for other knobs), so the override leaked
  in and latched `target_altitude_y` to `-1.9` instead of the test's expected `0.0` — corrupting
  every "sunk"/"at target" comparison after it. **Fixed** by resetting `desired_height_override_y`
  back to `0` in `config.yaml` (operator's choice, over patching the test itself).
- **`explore PRELUDE arm+takeoff+...`** (a real session-44 regression, NOT pre-existing —
  confirmed by reverting to the clean `main`/session-43 base with the override also reset to `0`:
  it passes there). The test's synthetic post-ascend `pos_y` is pinned at a flat `0.0` stand-in,
  never a realistic cruise altitude — harmless under the OLD live-calibrated band (self-consistent
  by construction, computed from this same test's own numbers) but read as permanently "sagged"
  by the new HARDCODED `trim_sag_trigger_y` (`0.0 >= -1.75` is always true). TRIM's trigger check
  runs before `SETTLE` gets to decide its own next state, so every arrival at `SETTLE` was
  hijacked into `TRIM` again, forever, before `REPLAN`/`ORIENT` could ever be reached — a real,
  concrete instance of the exact risk flagged when this exception was approved: the hardcoded
  band only makes sense for the coordinate range it was picked for; anything outside it (this
  test's `0.0`, or potentially a real flight with a different SLAM scale/origin) makes TRIM fire
  every tick with no resolution. **Fixed**, per the operator's explicit "kill tests relevant to
  before the hack, I need this working" instruction: this test is about the arm/takeoff/ascend/
  descend/baseline-nudge sequence, not TRIM (which has its own dedicated `HEIGHT-TRIM` test,
  already green) — disabled TRIM for this one test (`cp.trim_enable = False`) rather than contort
  its synthetic altitude to fit the new band. `python autopilot.py --self-test`: **ALL PASS**, 0
  failures, confirmed on two consecutive runs (deterministic).

## Live-flight bug: TRIM_RESUME_WAIT could hang forever on slow SLAM (real bug, fixed)

First live-fly attempt (flight `20260901_103028`) got permanently stuck in `TRIM_RESUME_WAIT`
right after takeoff — operator report: "stuck on TRIM_RESUME_WAIT forever." Traced from the
flight's own log, not guessed:

- Calibration was fine: `ceiling_y=-2.296 desired_y=-2.035 delta=0.261` — `desired_y` sits
  comfortably inside the new `[-2.10, -1.75]` band, just close to its `-2.10` edge.
- `10:32:46.470` — `pos_y=-2.101` crossed `trim_high_trigger_y` by 0.001 (plausible drift/noise
  right at the edge of the band) → `TRIM enter (DOWN)`, correct behavior.
- `10:32:50.718` — `TRIM done (DOWN): post pos_y=-1.878` — the pulse actually worked and
  corrected the drone back to `-1.878`, safely inside the band. Hands off to `TRIM_RESUME_WAIT`.
- From `10:32:50.718` to `10:33:25.329` (~35s) — SLAM's solve times ran consistently 900-1500ms
  (`slam_slow_ms` is 1000ms; `settle_fresh_frames` needs 6 CONSECUTIVE sub-1000ms solves), so the
  settle-gate's freshness half never cleared. `status` stayed `OK` the entire time (no PLAN-LOST
  logged until the very end) — this is a pure perception-throughput patch, not evidence of a bad
  pose. The flight sat frozen in `TRIM_RESUME_WAIT` until it finally degraded to `PLAN-LOST`.

Root cause: `TRIM_RESUME_WAIT` (introduced session 28) never had a timeout for sustained SLAM
slowness. `SLAM_HOLD` got exactly this rescue in sessions 35/43 (`slam_slow_hop_after_s`, on the
identical reasoning — a slow settle-gate with plan OK is a throughput signal, not a bad-pose
signal) but it was never extended to `TRIM_RESUME_WAIT`. This is NOT caused by the hardcoded
thresholds themselves — the calibration and the pulse both worked correctly — it's a pre-existing
gap in a different, older mechanism that this session's live TRIM activity happened to expose for
the first time.

**Fixed**: `TRIM_RESUME_WAIT`'s handler now reuses the identical `slam_slow_hop_after_s` rescue —
if the settle-gate hasn't cleared after that long, force-resolve anyway via
`_trim_resolve_resume` (which already degrades gracefully to `SETTLE`→`REPLAN` if the plan's pose
is unavailable), gated by the same `has_any_capture` guard SLAM_HOLD uses (never force off a
total capture blackout — only genuine slowness). New self-test case (i) in the `HEIGHT-TRIM`
block reproduces the exact bug (60 ticks of `slam_ms=1500` with valid `cap_ts`, `slam_slow_hop_after_s`
shrunk to 0.5s for the test) and confirms it force-resolves to `ORIENT`; a companion case confirms
a total capture blackout (`cap_ts` always `None`) does NOT force-resolve. Verified the new test
actually catches the bug: reverted the fix on a scratch copy and confirmed it flips to FAIL
(`force-resolve on slow SLAM=False`), then confirmed it passes again restored. `python
autopilot.py --self-test`: **ALL PASS**, 0 failures.

**NEXT = LIVE-FLY AGAIN** — this specific hang is fixed, but it was found on the FIRST live
attempt; there may be more surprises. Watch for: TRIM still pulsing correctly at the two
hardcoded edges; `TRIM_RESUME_WAIT` resolving normally within ~1s when SLAM is healthy; and if
SLAM is slow again, a log line ending in "(forced after N.Ns of slow settle-gate)" instead of an
indefinite hang.
