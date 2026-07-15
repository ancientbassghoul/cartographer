# Plan — Session 17: triggerDown/reverseDown fix + height-system simplification  [PLANNED, NOT BUILT]

> **Self-contained build spec** (survives a context clear — resume cold from here).
> Git baseline: commit **44b4fa6** (sessions 14-16, committed). Only `io_bridge.py` is dirty (diagnostic
> scaffolding to be reverted — Step 0). Line refs are from base 44b4fa6; confirm via the named anchors since
> edits shift lines. Class is `ExploreController` (autopilot.py:629).

## Context — the discovery (why this whole change)
While diagnosing the broken height TRIM we built temporary io_bridge diagnostics (a `t` macro that fires one
synthetic trim, a `y` replay of a recorded manual trim, and a `--log-commands` CSV of the full outgoing control
vector). Comparing a hand-flown trim against the macro proved the root cause of MONTHS of pain:

**The Unity sim gates real thrust on the `triggerDown` / `reverseDown` BOOLEAN flags — NOT the analog
`trigger` / `reverse` values.** When you hold `w`, io_bridge sends `triggerDown=True`; the drone thrusts. The
autopilot has **never** set that boolean: `AUTONOMY_FIELDS` (io_bridge.py:73-76) — the whitelist the autonomy
overlay applies — lists only `trigger`/`reverse` (analog), and the autopilot never references `trigger_down`.
So **every autonomous forward/reverse the drone ever made had `triggerDown=False`** = the "terrible movement,
no real thrust" condition. io_bridge's smoothing even DECAYS the analog back toward 0 unless the boolean is held
(io_bridge.py:267-275). This almost certainly explains the legendary "crawl" (~0.02-0.04 u/s) and much of the
mapping difficulty.

Two consequences the operator confirmed in MANUAL mode:
1. With `triggerDown` held, the `t` macro finally "plays beautiful" — the drone rises properly.
2. **Horizontal flight holds altitude to the millimetre.** The drone does NOT inherently sag — so the whole
   periodic height-calibration + TRIM apparatus was fighting a self-inflicted problem. The ONE thing that
   moves height without command: **flying FORWARD or STRAFING into a wall → the drone climbs uncontrollably
   while forward is still pressed** (reverse into a wall does NOT). So height calibration is still wanted
   AFTER a wall hit — which is why we keep the flight-height median to judge a calibration's result.

Intended outcome: the drone flies at a proper speed (thrust engaged), holds height on its own, and a large
chunk of height machinery (periodic re-calibration + TRIM) is deleted. Operator expects to re-tune config
(e.g. `forward_throttle`, speeds) afterward since the drone will be much faster.

---

## Step 0 — Revert the diagnostic scaffolding
`git restore io_bridge.py` (removes the `t` macro, the `y` replay, `--log-commands`, the full-packet command
logger, `_REPLAY_TRIM`, all `_trim_macro_*`/`_replay_*` state — everything is in io_bridge.py and uncommitted).
Optionally `rm OUTPUT/diag/*_commands.csv` (diagnostic outputs). After this, `git status` should be clean at
44b4fa6. **The real changes below start from this clean base.**

## Step 1 — Document the finding in PROGRESS.md
Fold a session-17 narrative into the Documentation section (concise, per CLAUDE.md doc style): the
triggerDown/reverseDown discovery, how we found it (full-packet command-log diff: manual `triggerDown=True` vs
macro `False`), the crawl explanation, and the height insight (drone is altitude-stable except a forward/strafe
wall-hit climb). (PROGRESS.md's Next pointer already references this plan.)

## Step 2 — Wire `triggerDown` / `reverseDown` into the REAL control path
**2a. io_bridge whitelist.** Add `"trigger_down"` and `"reverse_down"` to `AUTONOMY_FIELDS` (io_bridge.py:73-76)
so the overlay (`for k in AUTONOMY_FIELDS: cs[k]=cmd[k]`, ~io_bridge.py:321) will apply them.

**2b. Central derivation — the single choke point is `_full_vector()` (autopilot.py:83-87).** Do NOT edit every
emit site (confirmed: all forward/reverse — `forward_preset` 2388, parallax `{"trigger":…}` 2123/2162,
`back_off` 2126/2165/2460, `reverse_probe` 2408, calib-escape/`_begin_fallback` reverse 1352/1434, `_invert_one`
rewind steps 1133/1135, postlude homing 2678 — flow through `RecipePlayer`/dicts into ONE `active`, published
via the `publish()` closure at autopilot.py:2945-2952 → `_full_vector(active, seq, now, state)`). `_full_vector`
merges a `_NEUTRAL` base (77-80) with `active` and is the single place every command (incl. the empty-command
HOLD publishes at 3184 and the mission-runner sends 508/581/595) is finalized. **In `_full_vector`, after the
merge, ALWAYS set both** (True *or* False — never conditionally, so the overlay can't leave a boolean stuck on):
```python
vec["trigger_down"] = float(vec.get("trigger", 0.0) or 0.0) > 0.0
vec["reverse_down"] = float(vec.get("reverse", 0.0) or 0.0) > 0.0
```
(Add `trigger_down`/`reverse_down` to `_NEUTRAL` at 77-80 too, so they're always present = `False`.) This covers
every branch — including recipe-player reverses (rewind/fallback/reverse-probe) — in one edit.

**2c. Fail-safe.** Confirm io_bridge `_neutralize_autonomy` (stale-command zeroing) also sets
`trigger_down=False`/`reverse_down=False` so a dropped link releases the gas; add if missing.

## Step 3 — Simplify the height system
**KEEP (untouched):**
- **Prelude / first calibration**: `ARM`(1827) → `TAKEOFF`(1837-1848, sets `airborne_done`=True 1843,
  `_calib_active`=True 1844) → `ASCEND`(1863-1946) → `DESCEND`(1948-1968) → `CALIB_VERIFY`(1970-2059) →
  `BASELINE_NUDGE`(2141-2179). Prelude ENDS at `REPLAN`(2181) which sets `_explore_started=True`(2182). Gated by
  `ascend_to_ceiling`(669)/`airborne_done`(950)/`_calib_active`(819).
- **Flight-height median**: `_mapping_altitude_history`(818, cfg `mapping_alt_history_len` 792), `_alt_median`
  property(1490-1498), ingest at step() top (1714-1720, `MAPPING_ALT_STATES` 2836). `CALIB_VERIFY` judges the
  settled `pos_y` vs this median (1984, 1996-2013; cfg `calib_min_baseline_samples` 793, `calib_low_height_margin`
  795). This is how a future wall-hit calibration gets judged "very different than the median".
- **Calibration recovery — KEEP ALL** (keyed on `_calib_active`, so they protect the FIRST calib, not just
  periodic): `CALIB_LOST_HOLD`/`_step_calib_lost`(1264-1313, entered 1734), `CALIB_ESCAPE`/`_step_calib_escape`
  (1340-1385, entered 1732), `_calib_fail_escalate`(1315-1338), `ASCEND_ESCAPE`(2061-2101),
  `CALIB_TRANSLATE`(2103-2139).

**DELETE:**
- **Periodic per-goal re-calibration TRIGGER** — the block **inside the REPLAN handler at autopilot.py:2204-2213**
  (`if calibrate_on_goal_change and ascend_to_ceiling and goal_moved and cooldown_ok and not _recalibrating:
  … _enter("CALIBRATING_HEIGHT")`). Remove that `if` block; keep the normal orient path. The `CALIBRATING_HEIGHT`
  state(1850-1861) + all ASCEND/DESCEND/CALIB_VERIFY machinery **stays reusable** (a future wall-hit trigger just
  does `_recalibrating=True; _enter("CALIBRATING_HEIGHT")`). Now-dead once the trigger's gone: cfg
  `calibrate_on_goal_change`/`calib_cooldown_s`/`calib_goal_change_dist`(787-789), vars `_last_calib_t`/
  `_leg_goal_prev`(813-814) + their writes (ASCEND 1882/1894/1918, REPLAN 2208/2216) — remove.
- **TRIM, entirely**: `TRIM` handler(2536-2611); the inline sag trigger at step() top(**1807-1825**,
  `_TRIM_TRIGGER_STATES` 2840); `_trim_exit`(1072-1097); the reset_leg trim runtime(1001-1008) + defensive clear
  (1694-1697); all `_trim_*`/`_trimming` vars; cfg keys(836-849): `trim_enable`,`trim_sag_ratio`,`trim_aim_s`,
  `trim_fwd_s`,`trim_settle_s`,`trim_reposition_s`,`trim_pitch_up`,`trim_throttle`,`trim_reset_s`.
- **The 3 TRIM refs** `_ceiling_y`/`_desired_y`/`_trim_delta`(853-855): CONFIRMED NOT read by any kept control
  logic (`CALIB_VERIFY` judges vs the MEDIAN, not `delta`). Delete the refs — but they have **two non-TRIM
  readers that MUST be updated in lockstep or the build breaks**, so do these together:
  1. **`CALIB_VERIFY` PASS log string** (autopilot.py:**2030-2035**) — builds `calib_log` from
     `_ceiling_y/_desired_y/_trim_delta`. Trim it to median-only (or drop it).
  2. **Debugger telemetry — the WRITER (MANDATORY, else AttributeError on every log write).** The replay-record
     caller at autopilot.py:**3161-3165** passes `alt={"ceiling": ctrl._ceiling_y, "desired": ctrl._desired_y,
     "delta": ctrl._trim_delta, "median": ctrl._alt_median}` — this reads the deleted attrs. **Change it to
     `alt={"median": ctrl._alt_median}`.** Then drop the now-dead fields in `_timeline_step_record`
     (autopilot.py:**178-180** `alt_ceiling/alt_desired/alt_delta`; KEEP **181** `alt_median`; update the
     comment 174-177). Check the `_timeline_step_record` self-tests (autopilot.py:**3246-3251**) don't pass a
     ceiling/desired/delta `alt`.
  3. **Debugger telemetry — the READER (flight_replay.py HTML panel).** At **394-398**: remove the three spans
     **395-397** (`ceiling`/`desired`/`delta`; the `<br>` moves to line 394's group or the median line), KEEP
     the group header **394** (`HEIGHT CALIBRATION (+Y DOWN)`) + the median span **398**. The flight_replay
     self-test at **565** asserts that header string is present — leave the header text so it stays green.

**FUTURE (NOT this session):** wall-hit-triggered re-calibration (forward/strafe wall contact → uncontrolled
climb → re-calibrate, judged against the retained median). The kept `CALIBRATING_HEIGHT` machinery + median
exist for exactly this. PROGRESS.md backlog note only.

## Step 4 — Self-tests  (autopilot.py --self-test)
- **DELETE** `explore HEIGHT-TRIM` (3690-3756; note 3693 hardcodes the 3 refs).
- **UPDATE** `explore HEIGHT RE-CALIB state-gated` (3824-3932): its subcases (1)(2) test the periodic *trigger*
  (delete those); subcases (3)(4)(5) + the median `ingest_gate` (3908-3926) exercise CALIB_VERIFY FAIL/escape/
  timeout + ingest — KEEP those but re-drive them WITHOUT the periodic entry path (they currently enter via
  `_last_calib_t`/`_leg_goal_prev` at 3878/3886/3892/3897/3905; enter `CALIBRATING_HEIGHT` directly instead).
- **KEEP** `explore PRELUDE …`(3775-3822), `CALIB_LOST_HOLD …`(3934-4001), `CALIB_ESCAPE/STUCK …`(~4003-4070).
- Then run all six green: `autopilot / flow_contact_detector / frontier_planner / ground_grid / flight_replay /
  perception` via `./venv/Scripts/python.exe <module>.py --self-test` (`perception` = `perception_worker.py`).

## Critical files
- `io_bridge.py` — `AUTONOMY_FIELDS` (+trigger_down/reverse_down), `_neutralize_autonomy`. (Also the Step-0
  `git restore` target.)
- `autopilot.py` — `_full_vector`/`_NEUTRAL` derivation; delete periodic re-calib trigger + all TRIM; keep
  prelude/median/CALIB_VERIFY/recovery; self-tests.
- `config.yaml` — remove all `trim_*` keys (under `autonomy.explore`); leave calibration + `forward_throttle`/
  speed knobs (operator will re-tune). autopilot reads `cfg["autonomy"]["explore"]`.
- `flight_replay.py` — HEIGHT CALIBRATION HTML panel → median-only (drop ceiling/desired/delta spans 395-397;
  keep header 394 + median 398; self-test 565 stays green). NB the WRITER side is in `autopilot.py` (the
  `alt={...}` caller at 3161-3165 + `_timeline_step_record` 178-181) — MANDATORY to update or the logger crashes
  on the deleted `_ceiling_y/_desired_y/_trim_delta`.
- `PROGRESS.md` — Step 1 documentation + Next pointer + wall-hit-calibration backlog note.

## Verification
- **Offline:** all six module self-tests green.
- **Live (`python fly.py`, press `m`):**
  1. Drone moves at a **proper speed** (thrust engaged — crawl gone). Expect to LOWER speed knobs
     (`forward_throttle`, `parallax_push_throttle`, reverse/strafe throttles) — it will be much faster.
  2. **Height HOLDS on its own** during horizontal flight — no sag; TRIM/periodic-recalib gone and not missed.
  3. **First calibration still runs** at takeoff (ceiling tap → back off → CALIB_VERIFY vs median → OK).
  4. Fly forward/strafe into a wall → CONFIRM the uncontrolled-climb (motivates the future wall-hit
     re-calibration; not fixed this session).

## Closing (per CLAUDE.md — mandatory last steps)
1. Fold this session into PROGRESS.md (concise narrative + refreshed Next pointer + the wall-hit backlog item).
2. Leave the tree + PROGRESS.md self-describing for a cold resume; note self-test status.
