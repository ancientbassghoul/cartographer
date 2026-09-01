# Session 48 — the back-off was reacting to a 3-second blip, not to being stuck

## Origin

Session 47 stopped the back-off *loop*. The operator then asked the better question: is a back-off
~70 ms after the plan goes `PLAN-LOST` too harsh? He proposed holding for a TBD number of seconds
first — suggesting 12 — to give SLAM a chance to re-lock, and asked for statistics from previous
flights before committing to the number.

## The measurement

First, the latency itself. Every back-off across both recent flights fired one tick after the status
flip — **66-78 ms, 8 for 8** (the ~70 ms is the SIFT match against `F_LKG`). That is by design:
`_maybe_loss_snapshot_backoff` is a one-shot fired at the loss instant, justified in its own
docstring by "before that boundary nothing has moved, so the cached snapshot / F_LKG are
trustworthy".

But `PLAN-LOST` is not an event — it is a timeout: `_plan_status` returns it when
`plan_age > plan_timeout_s` (3.0 s). So the reaction is immediate relative to a status flip that is
itself already ≥3 s stale.

Then the population, parsed from **all 128 autopilot logs — 2,066 loss episodes**, measuring each
`non-OK → OK` interval and classifying by whether any maneuver was commanded during it:

| | held still (n=2041) | maneuvered during it (n=25) |
|---|---|---|
| median | **2.4 s** | 29.1 s |
| p90 | 7.6 s | 130.2 s |
| p95 | **9.5 s** | 143.6 s |
| max | 128.3 s | 183.6 s |

Cumulative for the held-still population: 59.6% recover within 3 s, 83.4% within 6 s, 94.0% within
9 s, **96.9% within 12 s**.

The maneuvered column is 25 samples and at least partly reverse-causal (a maneuver fires *because*
the episode is already bad), so it is suggestive, not proof — but the direction is unambiguous and
the mechanism is already documented in this codebase (moving while blind destroys the visual
continuity SLAM needs to re-lock).

**The finding that settled it** — per-episode, for the two most recent flights:

```
14:29:15.705 -> OK after  0.11s   backoff fired
14:29:46.999 -> OK after  0.20s   backoff fired
14:29:50.199 -> OK after  0.55s   backoff fired
14:30:53.236 -> OK after  0.67s   backoff fired
14:31:24.734 -> OK after  0.77s   backoff fired
14:31:28.500 -> OK after  0.87s   backoff fired
14:31:01.020 -> OK after 17.04s   backoff fired      <- the only genuine one
15:23:58.538 -> OK after  1.09s   backoff fired      <- the new flight's single back-off
```

**Seven of the eight back-offs fired into losses that self-healed in under 1.1 s** — the plan was
green again before the 2 s reverse push had even finished. Including the one session 47 had called
legitimate; it wasn't, it was premature. The drone was not reacting to being stuck. It was reacting
to a 3-second-timeout blip.

## Built

**`loss_backoff_grace_s: 12.0`** — a loss must OUTLIVE the window (on top of the 3.0 s
`plan_timeout_s` that declares `PLAN-LOST` at all) before it earns a physical reaction. 12 s sits
just past the measured p95 of 9.5 s with margin; beyond it you are in the tail where holding
demonstrably is not working, and that is `FALLBACK`'s regime, not the back-off's.

- `_loss_episode_t0` is stamped at the fresh-loss edge and cleared by a genuine `OK`. Both, not
  either: a slow-but-alive SLAM solving every ~3.5 s is **not** lost (`SLAM_HOLD`'s forced hop is
  that remedy), so the 19-flip `OK`/`PLAN-LOST` oscillation of flight `20260901_142738` can never
  accumulate across flips into a spurious reaction.
- While deferring, the one-shot stays **ARMED, deliberately not spent**, and the `HOLD_LOST` tick
  re-runs the deferred check every tick. Without that re-call the back-off would be dead code rather
  than delayed — it is only ever dispatched from the non-hold fresh-entry branch.
- The drone holds still throughout the window, which is what preserves the "nothing has moved since
  the snapshot was taken" invariant the whole check is built on.
- The cached-clearance decision now **reports the pose's age** (`stale pose, 15.3s old`). See the
  open item below — this is visibility, not a fix.

## Verified

`python autopilot.py --self-test`: **ALL PASS, 0 failures** (82 blocks), including the new
`SESSION-48 loss-recovery grace`. `frontier_planner.py`, `map_store.py`, `ground_grid.py`,
`flight_replay.py`: all green. `perception_worker.py` / `io_bridge.py` / `visualizer.py`:
`py_compile` clean.

The sessions 34/35/47 blocks assert the *decision tree*, not the timing; each of their controllers
now sets `loss_backoff_grace_s = 0.0` so they keep testing exactly what they were written to test,
and session 48's block owns the timing. Every assertion proven against its own defect by reverting
on a scratch copy:

| Revert | Caught by |
|---|---|
| no grace gate (fire at the loss instant) | 6 of 7 assertions fail |
| no `HOLD_LOST` re-call | `fires once the loss outlives the grace=False` — the back-off becomes dead code, not delayed |
| one-shot SPENT while deferring | `holds+says why=False`, never fires again |
| window not cleared on a genuine `OK` | `recovery inside the grace never reacts=False`, `next loss re-stamps=False` |
| window re-stamped every tick (a timer that never expires) | `fires once the loss outlives the grace=False` |
| stamp once per flight instead of per episode | **initially PASSED** — the OK-clear neutralises it on its own |
| **both window guards removed together** | `OK/LOST oscillation can never accumulate=False` |

That last pair is worth recording: the per-episode re-stamp and the OK-clear are **redundant by
design** — either alone defeats the oscillation — so no single-guard revert can fail the oscillation
assertion. Breaking both together does, which is what proves that assertion has teeth.

## Next

**LIVE-FLY.** Watch for:

- `LOSS-RECOVERY GRACE: too-close evidence at the loss instant, but holding still for 12s to let
  SLAM re-lock before reacting` — and then, in the overwhelming majority of cases, **nothing**: the
  loss resolves and the flight continues. On the last two flights this alone would have removed 7 of
  8 back-offs.
- A back-off that *does* fire is now a real event: the loss outlived 15 s total. Session 47's
  re-solve gate then still governs any second one.
- **Watch the hold itself.** This lengthens exactly the window in which the 20260718 flight drifted
  into a wall while parked. The intended guard is `_blind_contact_backoff`, which session 47 showed
  is structurally inert while holding (a hold commands nothing, so the flow detector produces no
  verdict). During these 12 s the drone is genuinely passive — safe in that it is not moving, but
  **not protected**. If a drift-into-wall shows up, that is the thing to fix, and it is a real gap.

## Open items (deliberately not done here)

1. **The geometric path now decides on a ≥15 s-old pose.** Past the grace, `_last_good_clearance` is
   at least `loss_backoff_grace_s + plan_timeout_s` old. The live `F_LKG` visual match is genuinely
   current and fine; the cached *pose* half is not. This session only makes the staleness **visible**
   in the event text rather than changing the tree, because reordering the tree (visual first) breaks
   the established `clearance-wins-first` contract and that deserves its own decision. Worth
   revisiting: after a 12 s hold, should stale geometry be allowed to trigger a physical maneuver at
   all?
2. **`contact_seconds=0.8` still cannot latch inside a 2.0 s back-off** (session 47's open item,
   unchanged): longest continuous `BACKWALL` run observed 0.58 s, with `arm_blank_s=0.4` eating the
   start. The BACKWALL cut will fire rarely until that is retuned, and retuning it changes contact
   behaviour in every state.
