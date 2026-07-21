# Session 35 — simplify `_recovering` trust restoration + a config switch between step-back and forced-hop

## Origin

The operator asked why the "SLAM slow → REWIND step-back" mechanism had produced zero events in the last
11 flights (last seen `20260720_123903`). Investigating turned up a real structural bug: `_recovering`
(armed on the first `PLAN-STALE` of a loss) was only ever cleared by a confirmed `>= 1.0u` displacement
measured from `_recovery_adv_start`, inside the `ADVANCE` handler — but `_enter()` wiped that anchor back to
`None` on every transition into a state other than `"ADVANCE"`/`"SLAM_HOLD"`, which includes
`SETTLE`/`REPLAN`/`ORIENT` — i.e. every ordinary hop boundary (`hop_duration_s`=2.0s). So the confirm
distance could only ever be measured *within a single hop*, never accumulated across several. Verified
directly against `20260721_134052`: 80 separate `ADVANCE` runs after the first `PLAN-STALE`, and in every
one the logged `pos` was identical at the start and end — confirmation was structurally impossible for the
rest of that flight. Since the step-back escape is gated on `not self._recovering`, it stayed jammed for the
same reason.

## Diagnosis → decision

Discussed the fix with the operator. Rather than patch the hop-boundary reset bug, **simplified**: clear
`_recovering` as soon as a loss recovers to a genuinely *settled* `OK` (the existing settle-gate — several
consecutive fast, fresh frames — already runs first regardless), instead of requiring a further
confirmed-motion step on top of it. Examined what else `_recovering` gates (freezing `command_history`
appends; the `_history_broken` ghost-path guard, protecting a *later* REWIND/step-back from popping a
maneuver anchored to a possibly-still-wrong post-reloc position) and judged the simplification acceptable:
`use_rewind_on_stale` already defaults `false` (session 31), so REWIND-on-stale isn't consuming that history
by default anyway; step-back's worst case off a slightly-early clear is one small corrective nudge; and the
confirm-distance check wasn't delivering this protection today regardless (the bug above).

Separately, the operator wanted the "SLAM slow + plan OK for 30s → force one hop toward the current goal"
idea added as a genuine alternative to step-back, selected by a config switch — same pattern as
`use_rewind_on_stale`: keep the old mechanism, gate it behind a flag, default to the new one. ("I want to
eventually throw out that REWIND bullshit... but knowing this project — we might also want to bring it
back.")

## Built

**`autopilot.py` (+ `config.yaml`):**
- Removed the dead confirm-distance block from `ADVANCE`, the `_recovery_adv_start` field (all sites), and
  the `recovery_confirm_dist` config knob.
- `SLAM_HOLD`'s settle-gate-clear branch, gated strictly on `nxt == "SETTLE"` (the recovery-resume path —
  every other resume target is untouched): if `_recovering`, clear it (+ `_history_broken`, reset the
  fallback-sweep counter, clear `command_history`) right there, *before* session 34's Idea-A clearance check
  in the same branch.
- New `slam_slow_hop_after_s`/`slam_slow_hop_grace_s` forced-hop escape: after 30s of continuous slow-but-OK
  holding (`_slam_resume == "ADVANCE"`, `not self._recovering`, `_slam_slow`), force a transition into
  `REPLAN` (re-reads the live goal, drives `ORIENT` → `ADVANCE` like an ordinary hop, just entered early). A
  self-expiring `_slam_slow_hop_deadline` lets `ORIENT`/`ADVANCE`'s own slow-gates bypass re-entering
  `SLAM_HOLD` for exactly that window (`_slam_slow_hop_active(now)`), cleared automatically by `_enter()` on
  any state besides `"ADVANCE"`/`"ORIENT"` — closing a real leak found during implementation: a physical
  guard (`BACKOFF`) cutting the forced hop short used to leave the deadline armed, letting an unrelated LATER
  leg inherit the bypass.
- New `use_slam_stepback_on_slow` switch (bool, default `False`, mirrors `use_rewind_on_stale`'s exact
  pattern): `SLAM_HOLD`'s fallthrough is now a clean either/or between the new forced-hop escape (default)
  and the classic step-back (kept fully intact, `True` brings it straight back).

**Bug found + fixed during implementation**: the forced-hop trigger originally set
`_slam_slow_hop_deadline` *before* calling `self._enter("REPLAN", now)` — but `"REPLAN"` isn't in `_enter()`'s
`("ADVANCE", "ORIENT")` exemption, so that same call immediately wiped the deadline it had just set,
defeating the whole bypass. Fixed by setting the deadline *after* `_enter()`.

## Self-tests

Rewrote "CONFIRMING ADVANCE (D5)" to assert the new settle-gate-based clear instead of the distance-based
one. New "SLAM-slow strategy switch" block: default routes a slow-but-OK hold through the forced hop and
never step-back; `use_slam_stepback_on_slow=true` routes through step-back and never the forced hop; the
escape never fires for a recovery-settle hold (`_slam_resume == "SETTLE"`) even past 30s; the grace-window
leak fix (a standoff cutting the forced hop into `BACKOFF` clears the deadline immediately; a later
unrelated `ADVANCE` does not inherit the bypass). Existing step-back tests updated with an explicit
`use_slam_stepback_on_slow = True` override to keep exercising that path now that it's opt-in.

`python autopilot.py --self-test`: **ALL PASS**.

## Verification

No hardware/GPU here — self-tests only. Live-fly checklist: does `_recovering` now visibly clear right at
"SLAM settled" instead of staying stuck for the rest of the flight; does a genuinely slow-but-OK patch force
a hop after ~30s with the default switch; flip `use_slam_stepback_on_slow: true` on one test flight to
confirm the classic step-back path still works end-to-end now that `_recovering` clears faster (it should
fire far more often than the last 11 flights showed); watch for a step-back or forced hop firing off a
still-slightly-wrong post-reloc position (the one accepted risk from the `_recovering` simplification).
