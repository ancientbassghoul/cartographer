# Session 46 — wedged in a corner: recognised it, reacted the only way it knew, couldn't

## Origin

The operator flagged the end of flight `20260901_124211`: brief flashes of `BACKOFF` state near the
end, but the drone stayed stuck. My first read (that fixing `BACKOFF`'s status-wipe bug alone would
explain it) was wrong, and the operator said so directly, naming the actual missing piece: *"tried
to backoff, can't, on hold too long, let's try a different movement."*

## Diagnosis (validated from the flight's own data before touching any code)

**The wedge is real**, confirmed by two independent signals:

- **Reverse authority collapsed monotonically.** Displacement per ~2s commanded reverse push,
  measured from push start to 4s after it ends (guaranteeing a fresh SLAM solve): `0.335u → 0.444u
  → 0.345u → 0.360u → 0.269u → 0.329u → 0.271u → 0.211u → 0.240u → 0.139u → 0.046u → 0.016u →
  0.060u → 0.144u → 0.133u → 0.094u → **0.000u**`. The final strafe attempt also moved **0.000u**.
- **The flow detector independently latched `BACKWALL`**, entirely without SLAM: at
  `mono_ts=1761064.937`, backward flow ratio went *negative* (−0.11 → −0.40), `contact_held=0.891`.
  36 WALL/BACKWALL detections total in the flight — the camera confirming contact behind.

**Why nothing escalated.** Status oscillated **`OK 3.0s / PLAN-LOST 0.6s`** for the last minute, so
every mechanism keyed on continuous time-in-one-state reset before it could fire:

| escape | why it never fired |
|---|---|
| `FALLBACK` sweep (turn + push a random direction) | dispatched **only** from `_step_stale`; **0 PLAN-STALE events** all flight |
| `VISUAL_RECOVERY` probe | PLAN-STALE only (session 42) |
| `SLAM_HOLD` forced hop (15s) | `_enter_slam_hold` restamps `_slam_hold_start`; OK windows only 3.0s |
| `HOLD_LOST` escalation | **does not exist** — no timer at all |
| `BLIND_BACKOFF` (flow reflex) | fired but just replayed the same reflex forever, no escalation |
| `BACKOFF` (what it chose) | destroyed by the PLAN-LOST router one tick later; 6/6 entries emitted `fields={}` |

The machinery for "try a different movement" **already existed and was proven** — the `FALLBACK`
sweep (wait → turn 15° → push a *freshly randomised* direction → wait → repeat → 720° → `STUCK`).
It was simply unreachable under `PLAN-LOST`.

## Review (external, before implementation)

An external review of the initial plan confirmed the structural diagnosis and caught two real
implementation traps in the draft, both accepted and folded into the final design:

- **`_fallback_phase` pre-setting side effect**: forcing the sweep's phase to `TURN` unconditionally
  on escalation would stomp a sweep already in progress (e.g. mid `WAIT_POST` at `cum 45°`),
  restarting its timer. Fixed by gating the phase-force on `self._fallback_phase is None` (a
  genuinely fresh episode only). The review's suggested companion fix — also resetting
  `_fallback_cum_deg` defensively — was checked against the code and found **unreachable and
  unnecessary**: `_reset_fallback_sweep()` is the sole writer of both `_fallback_phase` and
  `_fallback_cum_deg` and always zeroes them together, so `_fallback_phase is None` already implies
  a zeroed budget.
- **Counter reset scope**: also reset `_blind_contact_reacts` on a REPLAN commit to a **materially
  new** goal, so a drone that frees itself and moves on doesn't inherit a stale count. The
  "materially new" qualifier (reusing the existing `pick_moved` / `goal_area_radius` test) is
  load-bearing: an unqualified reset would recreate the exact starvation class session 45 already
  fixed once — flight `20260901_112227` re-committed a jittered goal every ~25s inside one
  `goal_area_radius` disc, which would clear the counter before it could ever reach the threshold.

Two further review points (threading live contact signals into the PLAN-LOST `FALLBACK` dispatch;
routing `BACKOFF`'s completion to `HOLD_LOST` not `SETTLE` while still blind) were already correct
in the plan; the review's reasoning for *why* they matter was made explicit in code comments.

## Built

Implemented as a 4-chunk, contract-first specification (exact signatures, a load-bearing
reset-site table, explicit "do NOT do X" guardrails) so each piece could be built and verified in
isolation:

1. **`_step_backoff(self, now, lost)`** — extracted the session-30 `BACKOFF` phase-timer body so it
   can own every status while it runs, matching `BLIND_BACKOFF`/`CALIB_ESCAPE`. Completion routes to
   `HOLD_LOST` (still blind) or `SETTLE` (OK) — routing to `SETTLE` while `PLAN-LOST` would be
   intercepted by the very router this fix escapes, recreating the bug one layer down. A loud
   impossible-state guard for `_backoff_t0 is None` (CLAUDE.md: never silent). Dispatched early
   (next to `BLIND_BACKOFF`'s own status-ownership branch) for every status, and reduced to a
   pass-through in the main OK-only dispatch chain.
2. **`FALLBACK` survives `PLAN-LOST`** — a dispatch inside the `PLAN-LOST`/`NO-PLAN` branch,
   mirroring `CALIB_ESCAPE`/`BLIND_BACKOFF`'s existing pattern, threading the live flow contacts
   through unchanged. `_step_stale`'s own PLAN-STALE dispatch is untouched.
3. **`_blind_contact_reacts` escalation counter** — `_blind_contact_backoff` now counts consecutive
   reflexes; past `blind_contact_escalate_after` (2, general robustness count) with no confirmed
   recovery between, it escalates into the `FALLBACK` sweep instead of replaying `BLIND_BACKOFF`.
   Resets at exactly 4 sites: `reset_leg`, the SLAM-settle trust boundary, the forced-hop trust
   boundary, and REPLAN on a materially new goal — never on a bare status flip or same-disc
   re-commit.

## Verified

Each chunk's self-test proven to catch its own defect by reverting the fix on a scratch copy,
confirming FAIL, then restoring (sessions 39/44/45 practice):

| Chunk | Regression proven |
|---|---|
| 1 (counter/reset wiring) | removing `reset_leg` wiring → `reset_leg clears=False`; removing the REPLAN reset → `materially-new REPLAN resets=False`; simulating an **unqualified** REPLAN reset (the exact mistake warned against) → `same-disc REPLAN holds=False` |
| 2 (`_step_backoff`) | removing the early dispatch → `commands real reverse=False` (and broke the guard test too); forcing completion to always `SETTLE` → `lost completion -> HOLD_LOST=False`; removing the impossible-state guard → an actual `TypeError` crash |
| 3 (`FALLBACK` under `PLAN-LOST`) | removing the dispatch → `phase advances=False` + `live contact threaded through=False`; simulating an accidental break of `_step_stale`'s own dispatch → correctly flagged by both the new AND the pre-existing session-31 test |
| 4 (escalation) | removing escalation entirely → no escalation possible; dropping the `is None` gate → stomps an in-progress sweep's phase/budget/timer; off-by-one (`>=` vs `>`) → escalates one reaction early |

One implementation bug was caught by a test's own loop logic, not the revert exercise: an early
draft of Chunk 2's regression test drove `BACKOFF` for a fixed tick count that ran past the
phase-timer's own natural completion, then misread the resulting `HOLD_LOST` transition as the bug
recurring. Fixed by tightening the loop to stay strictly within the hold phase; the actual
implementation was correct throughout.

`python autopilot.py --self-test`: **ALL PASS, 0 failures** (82 blocks). `frontier_planner.py`,
`flight_replay.py`, `map_store.py`, `ground_grid.py`: unaffected, all green.
`perception_worker.py`/`io_bridge.py`: cannot self-test in this environment (no `torch`/`NDIlib`);
both `py_compile`-checked clean.

## Next

**LIVE-FLY** (`python fly.py`, full stack). Watch for:
- After ~2 failed blind back-offs against the same obstacle, a `WEDGED: escalate to FALLBACK sweep`
  line **while status is `PLAN-LOST`** — previously structurally impossible.
- The `FALLBACK` sweep visibly turning and pushing a *different* direction each cycle instead of
  reversing into the same wall again.
- A real `reverse` payload (`gate_override: True`, full magnitude) in `state=BACKOFF` commands,
  instead of the `fields={}` this flight showed on 6/6 entries.
- If it still cannot free itself: the sweep reaches its 720° budget and declares `STUCK` — a
  **bounded, visible** ending rather than an indefinite hover.
- **New accepted risk, watch closely:** this is the first mechanism that flies a blind *randomised*
  push under `PLAN-LOST` — a deliberate, bounded softening of session 42's "never move blind on a
  loss" policy, gated behind two confirmed physical contacts. Watch for a push driving the drone
  somewhere worse rather than freeing it.
- Session 45's fixes are also still awaiting their own first live confirmation; this flight can
  cover both.
