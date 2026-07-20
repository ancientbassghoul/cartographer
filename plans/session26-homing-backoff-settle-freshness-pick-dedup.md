# Session 26 — homing back-off, settle-gate stale-frame fix, postlude recovery budget, pick-dedup fix

Two flights, four bugs, all diagnosed line-by-line off the `_timeline.jsonl` diag stream (not the
`.log`/console text — the jsonl is the canonical source; a couple of early wrong conclusions in this
session came from trusting the `.log` file or under-checking a hypothesis against the raw data, and
got corrected once verified against the jsonl directly).

## 1-3 — Flight `20260719_233845`: wall-bounce loops (`autopilot.py`)

**Bug A — homing never backs off from a wall.** `RETURN_TO_ORIGIN`'s own homing `ADVANCE` sub-phase
detects a clearance stop (`blocked`) but only stopped and re-aimed from the same wall-pinned pose —
unlike the main explore `ADVANCE` handler, which plays a `back_off` reverse maneuver before settling.
Confirmed in the log: `status` flipped `PLAN-LOST`↔`OK` 7 times in ~1m45s while `fwd_clear`/
`ring_clear` sat pinned at 0.25 (physical contact, not just stand-off) and `pos` jittered in a tiny
box. Fix: split the `blocked` case out into a new `_home_phase == "BACKOFF"` sub-phase that plays the
same `back_off` recipe as explore mode, then falls into the existing `SETTLE -> PLAN` re-aim.
Deliberately NOT wired into `_register_bump`/goals-DB — `self.leg_goal` sits on a stale exploration
goal throughout homing (never repointed at the origin), so a bump here would misattribute to an
unrelated disc; this is a pure kinematic reaction.

**Bug B — SETTLE can finish without ever seeing a frame from after the maneuver it's judging.**
`_settle_gate_begin` snapshots `_settle_gate_prequalified = _slam_window_ready(since=None)` — if the
rolling 6-frame health window is ALREADY clean the instant the gate opens, freshness is satisfied
purely by pre-existing history, and `SETTLE` completes on the 1.0s dwell timer alone, zero new frames
required. Confirmed in the mid-flight corner-goal bounce (`23:46:43`-`23:46:49`, same flight): the
only frame seen during a `BACKOFF->SETTLE` window was captured BEFORE the collision it was meant to
be judging, so `REPLAN`/`ORIENT`/`ADVANCE` recomputed the identical turn and drove into the same spot
3 times running. Fix (per the operator's own proposed bar): keep the prequalified shortcut for the
BULK of the window (no need for 6 brand-new frames when SLAM was already healthy), but add
`_slam_window_ready(..., latest_since=...)` — a WEAKER check that only the single MOST RECENT frame
must postdate the gate-open instant. `_settle_gate_poll` now always passes `latest_since=
self._settle_gate_t0`, prequalified or not.

**Bug C — the `home_max_s` (30s) safety cap couldn't fire while stuck recovering.** The cap check
lives inside `RETURN_TO_ORIGIN`'s own handler, but `_step_postlude_lost` only hands control back once
`status=="OK"` AND a full `calib_lost_recover_frames`-frame streak holds SIMULTANEOUSLY — jammed
against the wall (bug A), SLAM kept re-losing before it could sustain that streak, so `status` flipped
OK 7 times without the cap ever getting evaluated: 145.3s elapsed against a configured 30s budget
(verified via `t_mono`). Rejected the first fix idea (force a transition out of the hold once a
wall-clock budget is blown) after review: `POSTLUDE_LOST_HOLD` deliberately "owns every status once
entered" so its OK-gated exit beats status flicker — forcing a transition while still `PLAN-LOST`
would get intercepted right back into the same hold by the step()-top `POSTLUDE_STATES` router
(a livelock), and would violate the project's own no-blind-recovery/no-silent-fallback rule. Fix
instead relaxes the RECOVERY GATE itself once `postlude_recover_budget_s` (new, default 30.0) is
exhausted — the streak requirement drops to 1 frame, but the resume still only ever fires on a tick
where `status=="OK"`. Never acts blind; no new livelock.

Self-test note: `(g2)` in "settle-gate two-gate design" exercised exactly the closed loophole (a
prequalified window, zero post-open frames, dwell timer alone) as its expected-pass case — updated it
to add one frame captured at the gate-open instant, correctly isolating the dwell gate as its own
comment already said it should.

## 4 — Flight `20260720_024455`: 40+-cycle "goal reached" loop never blacklisted (`autopilot.py`)

The drone spent ~90s reaching the same frontier from ~0.5-0.7u away, settling, replanning, and
re-picking the (practically identical) goal — 40+ full hops — before losing SLAM lock. Ruled out the
session-26 settle fix above: every REPLAN in this loop fired at the expected ~1.0-1.3s cadence, so the
two-gate timing was working correctly; the bug is about WHICH goal gets re-picked, not WHEN.

`frontier_planner.py` already has an explicit CIRCLING guard for exactly this
(`register_goal_pick`: >`goal_loop_min_picks` picks with all pick-time drone positions clustered
within `goal_loop_pos_dist` -> permanent blacklist) — confirmed it never got fed more than 1 pick for
this disc (`goal_db`: `picks:1, strikes:0, bumps:0, blacklisted:False`, frozen the whole loop). Cause:
`REPLAN`'s pick-dedup (session 24, meant to stop a single leg's own multi-step-turn sub-commits from
over-counting) only checks whether the new goal's POSITION is close to the last commit
(`goal_area_radius`, 0.5u) — with no way to tell that apart from "a brand-new leg, following a
genuinely completed hop, that happens to land close to the last one." Every one of the 40+ cycles WAS
a fully completed hop (real `ADVANCE`, reached, settled, replanned), yet all got swallowed as
same-leg sub-steps because the target only jittered ~0.001-0.002u each time. Reaching a goal is also
unconditional progress (never a strike) by design, so the strike guard structurally can't break this
kind of loop either — the picks-based circling guard was the ONLY mechanism that could have, and it
was starved.

Fix: the discriminator was already computed one line earlier in the same function --
`prev_goal = self._hop_start_goal`, non-`None` only when a real `ADVANCE` tick actually ran and judged
a hop this cycle.
```python
same_goal_as_last_pick = (not pick_moved) and (prev_goal is None)
```
No regression on the `CALIBRATING_HEIGHT` re-entry path (verified: `pick_moved` was already `False`
there regardless, for unrelated reasons).

Found the self-test suite's OWN "PICK DEDUP" test encoded this exact bug as expected: its
`dup_suppressed_ok` case manually set `_hop_start_goal` again before the "duplicate" REPLAN
(simulating a genuinely judged hop, confirmed by its own `prev_progressed is True` assertion) while
asserting the pick should still be suppressed — despite its comment claiming to represent an
in-progress same-leg turn sub-step (which would never have `_hop_start_goal` set). Split into two
correct sub-cases: an unjudged same-leg sub-step (still suppresses) and a genuinely judged repeat hop
(now correctly registers a fresh pick).

Also investigated (not a bug): why the `20260720_024455` jsonl/log stopped abruptly with no
"MISSION COMPLETE"/STUCK entry. `STUCK` was never reached in that flight, so the log-pause mechanism
that suppresses per-tick spam while parked doesn't apply — the raw jsonl ticks at the normal ~30ms
cadence right up to its last real line, then ends on one truncated/empty record. Just the process
being killed mid-run while stuck in `HOLD_LOST`, not a deliberate size-management pause.

## Status

All fixes built + self-tested (`./venv/Scripts/python.exe autopilot.py --self-test` — ALL PASS,
including the two updated test cases above). Not yet live-fly-confirmed.
