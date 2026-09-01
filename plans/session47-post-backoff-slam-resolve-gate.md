# Session 47 — the back-off loop: SLAM never got to look at where the back-off put us

## Origin

The operator flagged flight `20260901_142738` — session 46's first live test — in blunt terms: two or
three back-offs in a row, an uncontrolled mess, and then the drone put its back to a wall and the
answer it chose was *another back-off*. He then named the fix himself, and it was the right one:
**SLAM had no opportunity to recover after the back-off, which is the whole point of backing off.**

## What session 46 was trying to do, and what actually happened

Session 46's goal: after 2 failed back-offs against the same obstacle, stop backing off and run the
`FALLBACK` sweep (turn + push a fresh random direction, bounded to 720° → `STUCK`).

| Session 46 chunk | Outcome in flight `20260901_142738` |
|---|---|
| 1. `_step_backoff` extracted + status-ownership | **Worked.** `state=BACKOFF fields={"gate_override": true, "reverse": 1.0}` — real reverse commanded. Previous flight: `fields={}` 6/6. |
| 2. `FALLBACK` dispatchable under `PLAN-LOST` | **Dead code.** 0 `FALLBACK` entries. |
| 3. `_blind_contact_reacts` escalation past 2 reflexes | **Never fired.** 0 `WEDGED`, 0 `BLIND_BACKOFF`. |

## Diagnosis (from the flight's own data, before touching code)

**Bug A — the escalation counter was on the wrong door.** All 7 back-offs came from
`_maybe_loss_snapshot_backoff` (the loss-instant path: *"loss detected with a visual match against
F_LKG"*). That path never touched `_blind_contact_reacts`. Session 46 wired the counter exclusively
into `_blind_contact_backoff`. Counter stayed at 0 all flight.

**Bug B — worse, the counter's trigger is structurally unobtainable where it is polled.**
`_blind_contact_backoff` is polled from `HOLD_LOST` and `SLAM_HOLD`, but `wall_contact` /
`backwall_contact` are only computed while a *directional command* is held (`_detector_command({})`
returns `None`), and those two states hold nothing. The log proves it: **12 `HOLD_LOST` entries, 14
`SLAM_HOLD` entries, zero detector verdicts from either.** All 55 verdicts came from `BACKOFF`, 16
from `ADVANCE`. Whole-flight latched contacts: **one**, a `CEILING` during `ASCEND`. Zero `WALL`,
zero `BACKWALL`. A perfectly wedged drone could never have tripped session 46's escalation.

**Bug C — the root cause of the loop.** After a `BACKOFF` completes it goes to `SETTLE`, whose exit
to `REPLAN` needs **6 consecutive SLAM frames under `slam_slow_ms=1000`**. SLAM was solving at
**3407 / 3547 / 3725 ms**. The gate was mathematically impossible to open, and `SETTLE` deliberately
has no timeout (*"if SLAM stops delivering, the plan status goes STALE/LOST and the step() top
diverts to recovery"*). So the only thing that ever broke the deadlock was the next status flip —
and the recovery path's first act is another back-off:

```
14:29:49.218  SETTLE: backed off -> settle          <- back-off #2 ends
14:29:50.199  plan status: PLAN-LOST                <- fresh loss edge, 0.98s later
14:29:50.265  BACKOFF: loss detected with a visual match against F_LKG (479 inliers)
```

Every flip re-arms `_loss_snapshot_checked`, and the trigger asks only two things — am I newly lost,
does F_LKG match as too-close — **neither of which changes when a back-off fails**. The drone had
barely moved, so the evidence was literally identical each time: 538 → 577 → 479 → 704 → 444 → 452 →
412 inliers, `contained=True` on all seven. With SLAM at ~3.5 s/frame the status flipped
`OK`↔`PLAN-LOST` **19 times**; 7 of those became back-offs, one only **1.05 s** after the previous
one ended.

**Bug D — `BACKOFF` was deaf to the wall behind it.** `_step_backoff(now, lost)` never received
`backwall_contact` though both call sites had it. The detector streamed **31 `BACKWALL-WATCH`
verdicts** from inside `BACKOFF` — `signal` pinned at ±0.02, `ratio=0.00`, while commanding *full*
reverse — and the phase timer ground the full 2.0 s into it anyway, seven times. This is the
operator's "back to the wall, and the solution it tries is a back-off", literally.

Contributing: all 7 re-pulsed `BUMP` at the identical goal `[2.6659, 2.0256]`, but the 2-bump
blacklist kept resetting (`count=1/2 (RESET from prev goal [4.308, 7.9] -> counter defeated)`) — the
planner jittered between goals, so the goal-level guard never accumulated either.

## Built

1. **Post-backoff SLAM re-solve gate.** `_step_backoff` stamps `_backoff_resolve_since` on completion;
   `_maybe_loss_snapshot_backoff` refuses to fire another back-off until a **fresh SLAM solve of a
   frame CAPTURED at/after that instant** has arrived (the one opening site is `_update_slam`). It
   spends the one-shot and returns `None`, so the caller's hard hover-hold runs — which is exactly
   "hold still and let SLAM look at where the back-off put you". Deliberately **not** gated on the
   solve being *fast*: at 3400-3700 ms/frame, requiring `< slam_slow_ms` would never open the gate
   and would just relocate the stall. Bounded by `backoff_resolve_budget_s` (12 s) for a capture
   stream carrying no `cap_ts`, and that timeout is **LOUD** (`_pending_notice`), never silent.
2. **The loss-instant path now feeds the SAME wedge counter.** New shared tail `_arm_loss_backoff`
   used by both loss-instant triggers: increments `_blind_contact_reacts` and escalates to the
   `FALLBACK` sweep past `blind_contact_escalate_after`, mirroring `_blind_contact_backoff`'s
   semantics exactly (fresh sweep skips the initial wait; an in-progress sweep keeps its phase,
   timer and 720° budget). This is what finally makes session 46's chunks 2+3 reachable — from the
   door the drone actually uses.
3. **`BACKOFF` reacts to the wall behind it.** `_step_backoff` takes `backwall_contact` and skips
   straight to `RELEASE` (keeping the full ramp-down window) instead of grinding the rest of the hold.

## Verified

`python autopilot.py --self-test`: **ALL PASS, 0 failures**, including the new
`SESSION-47 post-backoff re-solve gate` block. `frontier_planner.py`, `map_store.py`,
`ground_grid.py`, `flight_replay.py`: all green. `perception_worker.py` / `io_bridge.py` /
`visualizer.py`: `py_compile` clean (no `torch`/`NDIlib` in this environment).

Every assertion proven to catch its own defect by reverting the fix on a scratch copy, confirming
FAIL, then discarding the copy:

| Revert | Caught by |
|---|---|
| gate CHECK removed from `_maybe_loss_snapshot_backoff` | `SUPPRESSED=False`, `re-enables=False`, both budget assertions `False` |
| gate ARMING removed from `_step_backoff` | `gate armed on completion=False` (+ 4 downstream) |
| escalation removed from `_arm_loss_backoff` | `escalates to FALLBACK=False`, `1st fires+counts=False` |
| `backwall_contact` abort removed | `BACKWALL cuts the reverse push=False` |
| **the trap**: gate opens on ANY fresh frame, ignoring `cap_ts` | initially **PASSED** — the suite did not catch it. Added a dedicated assertion (a fresh solve of a frame captured *before* the back-off must NOT open the gate — at 3.5 s latency those are exactly what arrive first), which then failed as it should. |

One test-harness bug found and fixed during the exercise, not a code bug: the first draft parked the
drone in `HOLD_LOST` before the second loss edge, but `_maybe_loss_snapshot_backoff` only runs when
`st != "HOLD_LOST"` — so nothing was under test. Corrected to `SETTLE`, which is what the flight
actually showed (`14:29:49.218 SETTLE` → `14:29:50.265 BACKOFF`).

## Next

**LIVE-FLY** (`python fly.py`, full stack). Watch for:

- `LOSS-INSTANT BACK-OFF SUPPRESSED: waiting for SLAM to solve a frame captured after the last
  back-off (Xs / 12s budget)` — the back-off cadence should drop to at most one per genuine SLAM
  cycle, instead of the 1-second re-fires this flight showed.
- `WEDGED: N back-offs with no confirmed recovery between them -> escalate to FALLBACK sweep` after
  the third one, then the sweep visibly turning and pushing a *different* direction — the thing
  session 46 built and never once reached.
- `BACKOFF: flow BACKWALL contact while reversing -> stop pushing into the wall behind us` instead of
  a full 2.0 s push into it.
- **Watch closely:** `POST-BACKOFF RE-SOLVE GATE TIMED OUT` would mean the capture stream is not
  carrying `cap_ts` at all — a real problem to chase, not something to raise the budget over.
- **Still open, deliberately not touched this session:** the flow detector's `contact_seconds=0.8`
  cannot latch inside a 2.0 s back-off (longest continuous `BACKWALL` run all flight: **0.58 s**, and
  `arm_blank_s=0.4` eats the start), so fix 3 will fire rarely until that is retuned. Retuning it
  changes contact behaviour in *every* state, so it wants its own session and its own evidence.
- Sessions 44/45/46's own fixes are still awaiting live confirmation; this flight can cover them all.
