# Session 34 — proactive clearance checks that close the "no one is watching while blind" gap

## Origin

Diagnosed flight `20260721_014631`: the drone sat as close as 0.25-0.5 units from a wall (well inside
`stop_clearance_dist: 1.0`) for minutes at a time, and the clearance stand-off / BACKOFF never re-fired.
Root cause: the clearance-stand-off check lives entirely inside the `ADVANCE` state handler. That flight
spent almost no time in `ADVANCE` (336 ticks) and nearly all of it cycling `SLAM_HOLD`/`HOLD_LOST`/`FALLBACK`
(12893/3624/2787 ticks) — being that close to a wall degrades SLAM, which keeps the drone bouncing through
holds instead of ever completing a clean ADVANCE where the stand-off could re-check. Nothing else watches
wall proximity while holding — the optical-flow contact detector needs ~1.2s of *sustained motion* to latch,
so a stationary, hovering drone never trips it either.

The operator proposed two ideas closing this gap at the two moments recovery naturally passes through, and
asked to combine both into one plan.

## Design

Both ideas reuse the EXACT existing clearance-stand-off action ADVANCE already uses
(`_register_bump(...)` then `BACKOFF`/`SETTLE`, gated on the existing `stop_on_clearance`/`stop_clearance_dist`/
`backoff_on_standoff` knobs) — no new stopping mechanism, just two new trigger points feeding it.

**Idea B — one-shot check at the instant a loss begins.** `step()` now caches `_last_good_pos`/
`_last_good_clearance` every tick `plan_valid` is true (right next to the existing similar caches —
`_ever_tracked`, `target_altitude_y`, `_takeoff_heading`). A new `_was_lost`/`_loss_snapshot_checked` pair
tracks loss-episode boundaries independent of `_recovering` (which only the PLAN-STALE path sets — a loss
that starts as PLAN-LOST needs the same one-shot chance). The new helper `_maybe_loss_snapshot_backoff` is
called from BOTH existing "fresh entry into a loss" branches (PLAN-LOST's `HOLD_LOST` entry,
`_step_stale`'s fresh-entry) — marks the one-shot spent immediately (so a LOST↔STALE flicker within one
episode can't double-fire), and if the cached clearance reads too close, backs off using the CACHED position
(the live `plan['pos']` is `None` during a loss). Deliberately scoped to ONE attempt at the very first tick
of the episode — before that boundary nothing has moved yet by construction, sidestepping the need to
enumerate every possible motion-causing state (`_history_broken` only covers ORIENT/PARALLAX_PUSH/ADVANCE and
would miss `BLIND_BACKOFF`/`SLAM_STEPBACK`-caused motion).

This satisfies CLAUDE.md's no-silent-fallback rule: `perception_worker.py`'s own invariant ("if SLAM is not
TRACKING... the plan is published with `plan_valid=false`... never a coast on the last good goal") is
untouched — perception still honestly reports nothing new. The caching + the decision to act on a labeled,
explicitly-stale snapshot (the event string says "(stale pose @ loss)") is a distinct, visibly-logged
AUTOPILOT-side judgment call.

**Idea A — check the moment the recovery settle gate clears.** In `SLAM_HOLD`'s settle-gate-clear branch,
scoped specifically to the RECOVERY resume target (`nxt == "SETTLE"`, i.e. the `st in _RECOVERY_STATES`
path) — NOT the plain mid-leg SLAM-slow holds that resume straight into `ADVANCE`/`PARALLAX_PUSH`, where
ADVANCE's own per-tick check would catch it moments later anyway. Right before resuming, reads the now-LIVE
`forward_clearance_dist` (trustworthy here — the settle gate just confirmed several consecutive fresh, fast
frames) and backs off instead of resuming to `SETTLE`/`REPLAN` if too close.

## Built

**`autopilot.py` only.** `_last_good_pos`/`_last_good_clearance`/`_was_lost`/`_loss_snapshot_checked` added
(declared in `reset_leg()`, refreshed in `step()`); new `_maybe_loss_snapshot_backoff()` helper (next to
`_register_bump`); wired into the PLAN-LOST fresh-entry branch and `_step_stale`'s fresh-entry (Idea B); the
`SLAM_HOLD` settle-gate-clear branch gained the `nxt == "SETTLE"`-scoped check (Idea A).

## Self-tests

New "PROACTIVE CLEARANCE while blind (session 34)" block: Idea B fires immediate `BACKOFF` on a fresh
`PLAN-LOST` and on a fresh `PLAN-STALE` first-entry (using the cached pos, not the live `None` one); a
`LOST->STALE` flicker within one episode fires only once (one bump pulse, not two); a clearance that isn't
close falls through to the normal `HOLD_LOST` entry unchanged; no cached pose yet (startup) is a no-op.
Idea A: a too-close live clearance at the recovery settle-gate-clear (`_slam_resume == "SETTLE"`) fires
`BACKOFF` instead of resuming; the same close reading during a plain mid-leg hold (`_slam_resume ==
"ADVANCE"`) is unaffected (regression); a clear reading at the recovery settle-gate-clear still resumes to
`SETTLE` as before (regression).

`python autopilot.py --self-test`: **ALL PASS**.

## Verification

No hardware/GPU here — self-tests only. Live-fly checklist: does a loss that happens while already close to
a wall now back off immediately instead of sitting through the whole blind period; does a recovery that
re-locks close to a wall back off before ever reaching REPLAN/ORIENT; confirm no regression in the ordinary
ADVANCE-triggered stand-off or in the plain mid-leg SLAM-slow hold-and-resume path; watch for any new BACKOFF
firing off a stale cached pose that turns out to be wrong (the one accepted risk from Idea B) and whether it
happens often enough in practice to warrant tightening further.
