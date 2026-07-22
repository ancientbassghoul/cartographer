# Session 39 — remove RETURN_TO_ORIGIN's BACKOFF sub-phase + fix DONE resurrecting the mission on a loss

## Origin

Walked the operator through what `RETURN_TO_ORIGIN`/`ORIENT_HOME`/`HOME_REFINE`/`DOCK_FLOOR`/
`LOW_STANDOFF`/`DONE` actually do, step by step, after he flagged the postlude ending "acting up."
Two concrete asks came out of the walkthrough.

## Fix 1 — remove homing's `BACKOFF` sub-phase (operator's explicit call)

`RETURN_TO_ORIGIN`'s `ADVANCE` had a clearance-stand-off `BACKOFF` reaction (session 26), mirroring
explore's own `ADVANCE->BACKOFF`. The operator's reasoning: homing always turns to face the true
origin before advancing, so a properly-oriented leg isn't expected to run into a wall the way
frontier exploration (headings not origin-directed) can — the extra phase is unneeded complexity
here specifically. Removed the `blocked`/`clr` check and the `"BACKOFF"` sub-phase branch from
`RETURN_TO_ORIGIN`'s `ADVANCE`; explore's own `ADVANCE->BACKOFF` is untouched.

## Fix 2 — a real bug: DONE didn't survive a plan loss

Root-caused directly against last night's flight (`OUTPUT/diag/20260721_233244`, both the
`_timeline.jsonl` and `_autopilot.log`): `DONE` was reached cleanly at `23:53:58.784`
("EXPLORE COMPLETE -> STANDBY AT LOW HEIGHT" logged once, per its one-shot `_done_logged` flag).
Four seconds later (`23:54:02.747`) SLAM reported `PLAN-LOST` — plausible near the floor at low
altitude. `RETURN_TO_ORIGIN`/`ORIENT_HOME`/`HOME_REFINE`/`DOCK_FLOOR`/`LOW_STANDOFF` are all
protected against exactly this (`POSTLUDE_STATES`, `autopilot.py`, diverts to the dedicated
`POSTLUDE_LOST_HOLD` and resumes the same stage) — **but `DONE` was missing from that set.** The
loss instead fell through to the *generic* explore recovery path (gated only on
`self._explore_started`, set once after the prelude and never cleared): `HOLD_LOST`, then once
`status` read `OK` again, the generic recovery convergence unconditionally forced a transition
into `REPLAN`. `REPLAN` re-committed a corner goal (`[-1.6, 8.1]`) and resumed the *entire* explore
FSM — confirmed directly in the log from `23:54:13` onward: repeated `BUMP`/clearance-stand-off
`BACKOFF` cycles (hard 2s full-reverse bursts — the reported "flying backwards like a maniac"),
`BLACKLIST PERMANENT` churn, and by `23:55:03` a `TRIM enter (DOWN)` firing because `pos_y=-2.290`
was already near ceiling territory (the reported "jumped to the ceiling"). The mission effectively
un-retired itself after already declaring itself complete.

**Fix:** added `"DONE"` to `POSTLUDE_STATES`. Minimal change, because the existing
`_step_postlude_lost` machinery already does the right thing generically — on a loss it records
`self._postlude_resume = self.state` (now `"DONE"`), holds neutral, and on recovery calls
`self._enter(resume, now)`, i.e. re-enters `DONE` itself. `_done_logged` is already `True` by
then, so the one-shot completion line does not repeat. No new resume-phase branch was needed
(that machinery is only required for stages with sub-phases, like `ORIENT_HOME`/`HOME_REFINE`).
Mirrors the precedent already in the code for `STUCK`'s corner-giveup terminal hold (session 24's
`_corner_giveup_stuck` guard on the same generic convergence) — a genuinely terminal state must
own its own recovery, not be swept back into the live mission.

## Verification

Added two self-tests alongside the existing POSTLUDE tests in `autopilot.py`:
- **DONE survives a plan loss**: drives to `DONE`, injects `PLAN-LOST`, confirms the divert to
  `POSTLUDE_LOST_HOLD` with `_postlude_resume == "DONE"`, then confirms recovery resumes `DONE`
  directly without ever touching `REPLAN` or any `_RECOVERY_STATES` member.
- **RETURN_TO_ORIGIN ADVANCE no longer backs off**: drives with a forward clearance inside
  `stop_clearance_dist` and confirms `_home_phase` never becomes `"BACKOFF"`.

Cross-checked both by reverting the two edits in a scratch copy and re-running the self-test
suite: both new tests correctly flip to FAIL on the reverted code (confirming they actually
exercise the fix, not a tautology), and two *pre-existing, unrelated* failures (`explore
ALTITUDE-LOCK`, `explore PRELUDE arm+takeoff+...`) reproduce identically on the reverted copy —
confirming they predate this session's changes and aren't a regression introduced here.

`python autopilot.py --self-test`: new tests + all touched postlude tests PASS. The two
pre-existing failures above are untouched by this session and still need their own diagnosis.

**NEXT = LIVE-FLY** — watch for: no `BACKOFF` phase logged during homing; a loss while parked in
`DONE` now logs "plan loss DURING DONE" and quietly resumes `DONE`, never re-entering
`REPLAN`/`BUMP`/`BACKOFF`/`TRIM`. Separately, the two pre-existing self-test failures
(`ALTITUDE-LOCK`, `PRELUDE`) are still open and unrelated to this session's fixes.
