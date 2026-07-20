# Session 30 — BACKOFF: hard gate cut + full-magnitude 2-second reverse

## Origin

Using the session-29 Clearance debug tab, the operator flagged flight `20260720_210809` at 21:16:56.457: a
BACKOFF fires mid-ADVANCE and (per several previously-crashed flights) suspected it "never executes."

## Diagnosis

Traced the raw jsonl: the autopilot's own state machine and `cmd` output were fine — ADVANCE→BACKOFF
transitions cleanly and `reverse: 0.2` is emitted for the whole 0.3s `back_off` recipe immediately after
`trigger: 1.0`. Nothing is blocked or skipped at the autopilot layer.

The real lag lives one layer down, in `io_bridge.py`'s throttle smoothing (session 18, shared by manual AND
autonomous flight — not an autonomy-only bug). `trigger`/`reverse` are ramp TARGETS a 60Hz loop chases
(`_ramp()`: attack +0.05/tick, decay −0.1/tick). Going from `trigger=1.0` to the old throttled
`reverse=0.2` target takes trigger ~10 ticks (~167ms) to decay to 0 while reverse only takes ~4 ticks
(~67ms) to reach 0.2 — and the boolean thrust gate Unity actually gates on (`triggerDown`/`reverseDown`)
was *derived from the ramped analog* (`> 0.0`), not the freshly-commanded boolean, so both gates could read
`True` simultaneously during that window. Against BACKOFF's old 0.3s total recipe, a meaningful fraction of
the already-short reaction window was spent under residual forward thrust.

The operator then ran a manual experiment: full throttle → release trigger → immediately hold reverse —
found it takes **~2 seconds** of held reverse to get the right backoff effect. io_bridge's own ramp math
only explains ~167ms of that; the rest is very likely Unity's own physics/momentum once thrust reaches the
sim (a black box from this side of the socket, can't be read or measured from Python). The operator then
specified the exact desired BACKOFF behavior, built entirely around the platform's own already-characterized
ramp rates (10 ticks trigger-down / 20 ticks reverse-up — unchanged) restructured around the 2-second
finding: hard-cut the gate immediately on entry, hold full reverse for 2s (ramp-up included in that window),
then release with the same hard-immediate gate cut, waiting out the decay tail before declaring done.

## Built

**`io_bridge.py`** — a new, strictly opt-in gate override:
- `"gate_override"` added to `AUTONOMY_FIELDS`, `control_state`'s initial dict, and `_neutralize_autonomy()`
  (defensively cleared there so a dropped link mid-BACKOFF can never leave it stuck for a later state).
- `_step_controls`'s autonomy branch: computes the ramped analog exactly as before, but only re-derives
  `trigger_down`/`reverse_down` from it **when `gate_override` is not set**. When it IS set, the booleans
  `_apply_autonomy_overlay` already wrote from the commanded values that same tick are trusted as-is —
  every other emit site never sets this flag, so today's smooth-release derivation (and its own documented
  rationale) is completely unaffected everywhere except BACKOFF.

**`autopilot.py`** — BACKOFF is now a phase-timer, not a `flight_playbook.json` recipe:
- `_NEUTRAL` gained `"gate_override": False` (same "always declared, never stuck from a previous tick"
  guarantee the dict already gives `trigger_down`/`reverse_down`).
- New knobs (`autonomy.explore`): `backoff_hold_s` (2.0, clock starts at entry, ramp-up included),
  `backoff_release_s` (0.2, open-loop wait for the release-phase decay to finish — the autopilot has no
  visibility into io_bridge's internal ramped values, same as every other timed sub-phase in this file),
  `backoff_reverse_mag` (1.0, BACKOFF's own reverse target — independent of the shared `reverse_throttle`
  (0.2) every other reverse-emitting site uses).
- The three entry sites (clearance stand-off, wall-contact-without-reverse-probe, leg-timeout) now stash
  `self._backoff_t0 = now` instead of building a `RecipePlayer`.
- The `BACKOFF` state handler is a pure elapsed-time phase-timer: HOLD phase (`trigger=0, reverse=
  backoff_reverse_mag, gate_override=True`) until `backoff_hold_s`, then RELEASE phase (`trigger=0,
  reverse=0, gate_override=True`) until `backoff_hold_s + backoff_release_s`, then → SETTLE.
- Explicitly OUT of scope: the other two `"back_off"`-recipe call sites — `RETURN_TO_ORIGIN`'s homing
  `_home_phase == "BACKOFF"` sub-phase and the flow-reactive `BLIND_BACKOFF` state — are different
  states/sub-phases and keep their current recipe-based behavior untouched.

## Self-tests

- `io_bridge.py`: new `gate_override` block — a command with `gate_override=True` flips the gates
  immediately despite a large still-decaying residual analog; the SAME switch without the flag reproduces
  today's exact derived-from-analog behavior (regression guard); `_neutralize_autonomy()` clears a stuck
  flag.
- `autopilot.py`: new dedicated `BACKOFF phase-timer` block covering both phases + the SETTLE handoff +
  confirming the default magnitude is full (1.0), not the throttled `reverse_throttle`. Two PRE-EXISTING
  tests (`explore leg ORIENT->ADVANCE->WALL->BACKOFF->SETTLE->...`, `explore CLEARANCE-STOP`) had their
  drive windows sized for the old fixed 0.3s recipe — widened to cover the new ~2.2s duration.
- Also fixed, unrelated to BACKOFF: found `config.yaml`'s `recovery_settle_max_s` had been live-tuned
  (2.5 → 10.0) since the last full self-test run, which silently broke two OTHER pre-existing tests
  (`RECOVERY control-space`'s FALLBACK case, `SESSION-12`'s consuming-REWIND-drain case) whose fixed tick
  budgets assumed the old default. Fixed by giving both tests their own local `recovery_settle_max_s`
  override (matching the pattern several other tests in this suite already use), making them robust to
  future global tuning of that knob instead of silently depending on its current value.

`python autopilot.py --self-test` and `python io_bridge.py --self-test`: **ALL PASS**.

## Verification

No hardware/GPU in this environment — self-tests only. Live-fly checklist for next flight:
- Does the drone stop/reverse noticeably faster and more decisively at a clearance stand-off (or wall
  contact / leg-timeout) than before.
- Does the 2-second full-reverse hold feel right in person — this exact duration came from ONE manual test;
  expect to retune `backoff_hold_s` after watching it live.
- Confirm no regression in any OTHER reverse-emitting maneuver (parallax push, fallback, reverse-probe,
  homing backoff) — `gate_override` is strictly opt-in and defaults off everywhere except BACKOFF.
