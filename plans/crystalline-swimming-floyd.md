# Height-calibration must survive a plan loss (redo on recovery; one bump if stuck)

## Context

Flight `20260713_163055`, at `16:34:18.274` (frame ~11087): a per-goal `CALIBRATING_HEIGHT`
fired (`goal changed >1.0u + 60s cooldown`) → `ASCEND`. During ASCEND Phase-2 (continuous UP
hold, flush at the ceiling), SLAM ground on the frozen glued-to-ceiling image: **frame 11121
(368 ms) → a 3.3 s gap with no fresh frame → frame 11154 solved in 2845 ms.** That gap crossed
`plan_timeout_s = 3.0`, so at `16:34:21.759` a **PLAN-LOST** fired. The global top-of-`step()`
guard (`autopilot.py:1355-1364`) unconditionally forced `HOLD_LOST`, and when the plan returned
(~0.28 s later) the normal recovery funnelled `HOLD_LOST → SLAM_HOLD → SETTLE → REPLAN` — the
**normal mission leg loop, with zero memory of the calibration.** The DESCEND + CALIB_VERIFY of
the re-tap never ran, so the drone **stayed glued near the ceiling (`pos_y ≈ -2.2…-2.28`) for the
entire rest of the flight** and mapped at that too-high altitude. (Secondary: `_calib_active` was
never cleared, so the flying-height baseline froze for the rest of the flight too.)

The frame pulse shows **SLAM recovered on its own** (11154=2845, 11352=1127, 11437=1870 choked,
then 11556+ healthy <400 ms) even while the drone stayed glued — so the actual damage was the
*skipped descend*, not SLAM death. The un-glue bump is a contingency for the worse case.

**Desired behaviour (operator, confirmed).** Losing the plan must NOT erase the calibration
memory. On any plan loss *during a calibration*, latch a "calibration interrupted" flag, release
all controls, and hold in a dedicated visible state, watching the SLAM frame "pulse" (fresh
`frame_id` + `slam_ms`, maintained by `_update_slam`; `slam_slow_ms` = 1000). The decision tree:

1. **RECOVER → redo:** once **≥ 6 consecutive fresh frames under `slam_slow_ms`** (SLAM's solve is
   healthy) **AND** the planner has caught up (`status == "OK"`) → **redo the Height Calibration**
   (its own descend re-establishes the mapping height; this alone fixes the flight).
2. **STUCK → one bump (max), then hold:** while holding, if the drone looks stuck by *either*
   signal, give **one** DOWN bump (`descend` recipe) to try to unglue, then keep holding
   **indefinitely** until `status == "OK"` (then redo). Two stuck causes, one bump total:
   - **Cause A — SLAM's solve is grinding:** ≥ 6 consecutive fresh frames *at/over* `slam_slow_ms`
     (choked on the frozen ceiling image) → bump to try to **wake SLAM**.
   - **Cause B — planner won't lock:** ≥ 6 fast frames but `status != "OK"` (SLAM solves fine, the
     planner still can't produce a valid path) → bump to **unglue / reach mapped space**.
3. **One bump per hold, total** (either cause, whichever fires first). Rationale: a second downward
   nudge won't help SLAM and risks driving the drone into walls — bump once, then wait and see.
4. No time cap — the SLAM frame stream is the liveness signal.
5. Cover **PLAN-LOST / NO-PLAN / PLAN-STALE** (the same context-loss bug hits all three).
6. When the redone calibration completes smoothly (CALIB_VERIFY PASS), reset the interrupted flag.

## Design

Reuse existing machinery:
- `_update_slam(plan)` already runs every tick (`autopilot.py:1343`, *before* the status guard)
  and maintains `self._slam_fast_streak` / `self._slam_slow_streak` = consecutive fresh frames
  under / at-or-over `slam_slow_ms` (=1000, `config.yaml:264`). These ARE the operator's "6
  consecutive fast/choked frames" gates — no new frame-dedup code. (Fast and slow streaks are
  mutually exclusive at any tick: a fresh frame is one or the other and resets the counterpart.)
- The `descend` recipe (`flight_playbook.json:68` = `{"joy_vertical": 1}` for 0.1 s), played via
  `self.pb.player("descend")`, is the reusable DOWN bump (same code path as the `DESCEND` state).
- `_calib_active` is already True for the whole calibration excursion (set at `CALIBRATING_HEIGHT`
  entry `:1453`; cleared only when CALIB_VERIFY resolves). In the explore phase (`_explore_started`
  True, prelude already past) `_calib_active` True ⟺ a per-goal re-calibration is in progress — the
  discriminator for "loss during calibration."
- Altitude lock is injected only inside the spatial states (ADVANCE `:1949`, PARALLAX `:2133`),
  NOT globally — so a neutral-hold state returning `{}` is genuinely neutral and won't fight the bump.

### New dedicated state `CALIB_LOST_HOLD` + handler `_step_calib_lost(now, status)`

A telemetry-visible state (per the explicit-state / no-silent-fallback standard) owning the whole
interrupted-calibration lifecycle. New method on `ExploreController`, modelled on `_step_stale`.
**Two traps addressed inline:** the redo exit is gated on `status == "OK"` (Trap 1), and the bump
emits its first frame on the trigger tick (Trap 2).

```
def _step_calib_lost(self, now, status):
    # ENTRY (first loss during a calibration): latch, release controls, count the pulse FRESH.
    if self.state != "CALIB_LOST_HOLD":
        self._calib_interrupted = True
        self._calib_lost_bumped = False
        self._player = None
        self._slam_fast_streak = 0          # ignore the pre-loss streak; require fresh confirmation
        self._slam_slow_streak = 0
        self._enter("CALIB_LOST_HOLD", now)
        return {}, "CALIB_LOST_HOLD", ("plan loss DURING height-calib -> release controls, HOLD; "
                                       "redo calibration once SLAM solves fast AND plan is OK")
    # A descend bump in flight -> play it out, then neutral hold.
    if self._player is not None:
        active, done = self._player.fields(now)
        if done: self._player = None; return {}, "CALIB_LOST_HOLD", None
        return active, "CALIB_LOST_HOLD", None
    slam_fast = self._slam_fast_streak >= self.calib_lost_recover_frames
    # RECOVER (Trap-1 fix): SLAM fast AND the level-triggered planner status has ALSO caught up.
    if slam_fast and status == "OK":
        self._recalibrating = True          # DESCEND PASS -> REPLAN (per-goal path), never prelude
        self._calib_retries = 0             # fresh redo gets its full retry budget
        self._enter("CALIBRATING_HEIGHT", now)   # re-sets _calib_active, clears _player/_ascend_phase
        return {}, "CALIBRATING_HEIGHT", (f"SLAM healthy ({self._slam_fast_streak} fresh frames "
                                          f"<{self.slam_slow_ms:.0f}ms) + plan OK -> REDO height calibration")
    # STUCK -> ONE bump total per hold (either cause), first frame emitted NOW (Trap-2), then hold.
    stuck_slam = self._slam_slow_streak >= self.calib_lost_bump_slow_frames   # cause A: wake SLAM
    stuck_plan = slam_fast and status != "OK"                                 # cause B: unglue planner
    if not self._calib_lost_bumped and (stuck_slam or stuck_plan):
        self._calib_lost_bumped = True
        self._player = self.pb.player("descend")
        active, done = self._player.fields(now)
        if done: self._player = None
        why = "SLAM solve choking" if stuck_slam else f"SLAM fast but plan {status}"
        return active, "CALIB_LOST_HOLD", (f"{why} -> bump DOWN once (max) to unglue, then hold for plan OK")
    return {}, "CALIB_LOST_HOLD", None                # holding; wait for the pulse / plan OK
```

**Why the `status == OK` gate is essential (Trap 1).** `status` is level-triggered by the async
perception/planner pipeline and lags a healthy SLAM by a compute cycle. If the redo exited on the
frame streak alone, it would enter `CALIBRATING_HEIGHT` while `status` was still lost; the next
tick the guard would see `lost and _calib_active` (CALIBRATING_HEIGHT re-sets `_calib_active`),
re-divert into `_step_calib_lost`, the entry block would wipe the streaks, and it would oscillate
`CALIBRATING_HEIGHT ↔ CALIB_LOST_HOLD` at 1-tick cadence forever. Gating the exit on `status ==
OK` means we only leave the hold when the planner has genuinely caught up. (The 6-fast-frame gate
is itself a strong anti-flicker filter, so a stable OK won't immediately relapse; a *later* genuine
re-choke is a real new interruption and correctly re-enters the hold.)

### Guard change — divert into it (and stay in it) — `autopilot.py:1354-1376`

At the top of the `if self._explore_started:` block, *before* the existing LOST/STALE/OK branches,
intercept when a calibration is (or was) in progress. `self.state == "CALIB_LOST_HOLD"` routes
every status (including OK) into the handler so it owns the recovery exit:

```
if self._explore_started:
    lost = status in ("PLAN-LOST", "NO-PLAN", "PLAN-STALE")
    if self.state == "CALIB_LOST_HOLD" or (lost and self._calib_active):
        return self._step_calib_lost(now, status)
    # ... existing PLAN-LOST/NO-PLAN -> HOLD_LOST, PLAN-STALE -> _step_stale, OK -> recovery ...
```

`CALIB_LOST_HOLD` is deliberately kept OUT of `_RECOVERY_STATES` (`:2234`) and the
`_recovering`/reverse-list machinery — it is a calibration hold, not a spatial-recovery, and must
not touch the session-12 ghost-path logic.

### Reset the interrupted flag when the redo resolves — `CALIB_VERIFY`, `autopilot.py:1594-1616`

Add `self._calib_interrupted = False` at both terminal CALIB_VERIFY outcomes: PASS (`:1595`, "went
smoothly") and FAIL-retries-exhausted (`:1610`, resolved even if poorly). Mirrors the
`_calib_active`/`_recalibrating` lifecycle. (A FAIL that still has retries keeps the flag, since
the calibration is still ongoing.)

### New fields + config

`autopilot.py __init__` (near `:784`, after `_descend_issue_t`):
```
self.calib_lost_recover_frames  = int(e.get("calib_lost_recover_frames", 6))
self.calib_lost_bump_slow_frames = int(e.get("calib_lost_bump_slow_frames", 6))
self._calib_interrupted = False      # a calibration was cut short by a plan loss -> redo owed
self._calib_lost_bumped = False      # the one-shot (max-1) un-glue DOWN bump has fired this hold
```
`reset_leg()` (near `:969`): reset `self._calib_interrupted = False` and
`self._calib_lost_bumped = False` (a manual takeover abandons the owed redo).

`config.yaml` (calib section, near `:197`) — general robustness params (counts of the platform's
SLAM pulse, NOT room answers):
```
calib_lost_recover_frames: 6      # consecutive fresh frames < slam_slow_ms in the interrupted-calib
                                  #   hold that confirm SLAM's SOLVE recovered; with plan OK -> redo
calib_lost_bump_slow_frames: 6    # consecutive fresh frames >= slam_slow_ms (SLAM grinding) -> the
                                  #   one-time DOWN bump to try to wake SLAM (max 1 bump per hold)
```

## Files to modify
- `autopilot.py` — new `_step_calib_lost` handler; guard divert at `:1354-1376`; `_calib_interrupted`
  reset in CALIB_VERIFY (`:1595`, `:1610`); new `__init__`/`reset_leg` fields.
- `config.yaml` — two new calib knobs.
- (No `flight_playbook.json` change — the `descend` recipe is reused as-is.)
- `flight_replay.py` — the new state renders by string; confirm no enum change is needed.

## Self-test (offline, before live)
Extend the autopilot self-tests (near the SLAM-streak test `autopilot.py:~3330` and the recovery
tests) driving `ExploreController.step` with `_calib_active=True`, `_explore_started=True`:
1. From ASCEND, feed `status="PLAN-LOST"` → assert state `CALIB_LOST_HOLD`, `_calib_interrupted`
   True, active `== {}`, both streaks reset to 0.
2. **Cause A + Trap-2:** feed 6 fresh *choked* frames (`slam_ms=2000`) → assert exactly one `descend`
   bump (`joy_vertical==1`) fires **on the 6th choked frame's tick**, `_calib_lost_bumped` True; a
   7th choked frame → no second bump (neutral `{}`).
3. **Cause B + Trap-1:** fresh hold; feed 6 fast frames (`slam_ms=300`) but keep `status="PLAN-LOST"`
   → assert it does NOT exit to `CALIBRATING_HEIGHT`; instead one bump fires; more fast frames still
   `PLAN-LOST` → no second bump.
4. **RECOVER:** with 6 fast frames maintained, feed `status="OK"` → assert transition to
   `CALIBRATING_HEIGHT` with `_recalibrating` True, `_calib_retries` 0.
5. Drive a full redo to CALIB_VERIFY PASS → assert `_calib_interrupted` cleared.
6. Repeat entry with `status="PLAN-STALE"` → confirm it diverts to `_step_calib_lost` (not `_step_stale`).
Run all six module self-tests (`autopilot`, `flow_contact_detector`, `frontier_planner`,
`ground_grid`, `flight_replay`, `perception`).

## Verification (live)
`python fly.py`; press `m` to hand over. Fly until a per-goal `CALIBRATING_HEIGHT` fires and SLAM
chokes during the re-tap (glued-ceiling). Confirm in the replay HTML / timeline:
- On the loss the state goes `... ASCEND → CALIB_LOST_HOLD` (not `HOLD_LOST`).
- No 1-tick `CALIBRATING_HEIGHT ↔ CALIB_LOST_HOLD` oscillation while `status` lags (Trap 1).
- When 6 fast frames land and `status==OK`, it re-enters `CALIBRATING_HEIGHT` and completes
  `ASCEND → DESCEND → CALIB_VERIFY`; altitude DROPS off the ceiling (`pos_y` increases toward the
  flying-height median) — the failure symptom (`pos_y` pinned at ~-2.2) is gone.
- If SLAM stays choked (or solves fast but the plan won't lock), exactly ONE DOWN bump appears,
  then an indefinite hold.

## Closing steps (per CLAUDE.md — every plan ends with these)
- **Update `PROGRESS.md`:** fold this into the session narrative ("calibration lost its memory on a
  brief PLAN-LOST during the ceiling re-tap → glued to the ceiling for the rest of the flight; fixed
  with a dedicated `CALIB_LOST_HOLD` that survives the loss, redoes the calibration once SLAM solves
  6 fast frames AND the plan is OK, bumps down once — max — if SLAM stays choked OR the plan won't
  lock, and gates the redo exit on `status==OK` to beat the level-triggered flicker"), refresh the
  **Next** resume pointer, and reference this plan file.
- **Ready for a context clear:** leave the tree + PROGRESS.md self-describing (self-tests noted,
  live-fly pending flagged) so the next session can resume cold from PROGRESS.md alone.
