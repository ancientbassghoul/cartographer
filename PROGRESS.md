# Cartographer — Progress & Resume Handoff

_Last updated **2026-07-15** (session 18 **BUILT — io_bridge + autopilot + flight_replay self-tests green;
LIVE-FLY PENDING**). Resume from THIS file. **NEXT = LIVE-FLY** (`python fly.py`, press `m`) to confirm session
18 AND the still-pending sessions 17/16/15/14/11-13 in one go; then **RE-TUNE the throttle knobs** (session-17
"lower the speed knobs"; turn durations are unaffected — yaw is no longer ramped). Plan of record:
**`plans/session18-command-smoothing-and-height-median.md`**._

_**Session 18 — manual-style command SMOOTHING for autonomy + a real height-median (BUILT):** the operator
noticed autonomous flight is height-erratic (hard brake + pitch-up + altitude jump on every stop / plan-loss)
while his manual flight is "very very controlled." Root cause (found by diffing the `20260715_001039` manual
command CSV): manual keys only toggle the `trigger_down`/`reverse_down` (and arrow) GATES, and io_bridge's 60 Hz
loop RAMPS the analog toward them (`+0.05`/tick attack, `−0.1`/tick decay; yaw/pitch `±0.05` aim). The autopilot
BYPASSED all of it — `_apply_autonomy_overlay` hard-wrote the analog after the ramp, and `_neutralize_autonomy`
snapped to 0. Fix: the autopilot's **`trigger`/`reverse`** are now RAMP TARGETS the existing loop
chases (new `_ramp` + `_auto_*_target`; `_update_controls`→testable `_step_controls`), so thrust eases in/out
like a hand-flown stick while KEEPING the throttle magnitudes; release decays smoothly (aim axes + gates
still snap for safety). **yaw/pitch are NOT ramped** — live-flight showed the turn is duration-not-magnitude (the
sim eases the aim itself and the drone only rotates once the aim REACHES ±1), so ramping stole ~0.33 s from every
turn (30°→~5°); they pass straight through, restoring the calibrated turn recipe. Also **re-added `--log-commands`** (the reverted session-17 outgoing-packet CSV) — now
permanent + always-on via `fly.py` — so MANUAL vs AUTO smoothing is diffable. Second, independent fix: the
debugger's **drone-height median** was appended every ~50 Hz tick with no frame dedup (re-appending one stale
pose ~25×) and seeded with ~0 ground samples pre-takeoff — hence the −0.008→−1.8 jump-with-no-new-frame and the
lag. Now it ingests ONE reading per FRESH SLAM frame (`frame_id` dedup), only after the first calibration
(`_height_calibrated`), frozen during any calibration; `MAPPING_ALT_STATES` retired. New io_bridge
`--self-test` (ramp) + rewritten autopilot ingest-gate test; all green. **LIVE-FLY PENDING. CAVEAT: smoothing
attenuates short pulses (a 1–2-frame reverse tap / brief turn reaches less than commanded before the next
command) — expect to RE-TUNE throttle knobs AND maneuver durations / back-off counts on the first flight.**_

_**Session 17 — THE BIG ONE (BUILT):** while diagnosing the broken height TRIM we built temporary
io_bridge diagnostics (a `t` trim macro, a `y` replay, a `--log-commands` full-packet CSV) and, by diffing a
hand-flown trim against the macro, found the root cause of MONTHS of pain: **the Unity sim gates real thrust on
the `triggerDown`/`reverseDown` BOOLEAN, NOT the analog `trigger`/`reverse`.** The autopilot had NEVER set it
(`AUTONOMY_FIELDS` omitted it; io_bridge even decays the analog to 0 unless the boolean is held), so every
autonomous forward/reverse ran with the gas button UNPRESSED — the near-certain explanation for the legendary
~0.02-0.04 u/s "crawl". The operator also confirmed the drone HOLDS ALTITUDE on its own during horizontal
flight; it only climbs uncontrollably when flying FORWARD or STRAFING into a wall (reverse doesn't) — so the
periodic height-calibration + TRIM were fighting a self-inflicted sag. Built: (a) `trigger_down`/`reverse_down`
added to `AUTONOMY_FIELDS` (io_bridge) + `_neutralize_autonomy`, and DERIVED CENTRALLY in `autopilot._full_vector`
(the single choke point) from the analog value — so EVERY forward/reverse emit site engages thrust; (b) KEPT the
first calibration + flight-height median + all calib recovery; (c) DELETED the periodic re-calibration trigger
and ALL of TRIM (state/trigger/exit/vars/config/self-test). All six module self-tests green. LIVE-FLY pending;
expect to re-tune speed knobs afterward. A wall-hit-triggered re-calibration is the next FUTURE item (the kept
`CALIBRATING_HEIGHT` machinery + median exist for exactly it). The Step-0 diagnostic scaffolding was reverted
(`git restore io_bridge.py`)._

_**Session 16** (`plans/session16-settle-between-stages-and-return-to-origin.md`, **BUILT + committed 44b4fa6,
live-fly PENDING** — will be confirmed on the same flight as session 17): a test flight's
return-to-origin fell apart — it "turned like a maniac," fired the reverse-list back-to-back with no settles,
then spun (no settles), declared STUCK, retried. One pattern in three places: commanded actions fire
back-to-back with no still window for monocular SLAM to re-lock. Built a **shared settle gate** (`_settle_begin`
/ `_settle_poll`, healthy + lost-SLAM flavors; the SETTLE state now calls it) and put a settle between EVERY
action: (1) **REWIND** inverse maneuvers, (2) **spin FALLBACK** attempts (both lost-SLAM flavor, bounded by
`recovery_settle_max_s` so a dead pipeline still re-exposes) — this resolves the session-15 parked
"reverse-without-settling". Also **flipped the FALLBACK order to turn→push** (was push→turn) so the parallax
translation is the LAST motion before the settle (rescues the rotation for RELOC; matches the 'c'-reset-then-push
recipe). And built the **full return-to-origin ending**: `home_reach_dist` 1.0→0.5, a new
**ORIENT_HOME** state facing the recorded `_takeoff_heading`, a **POSTLUDE_LOST_HOLD** so the dock survives a
SLAM loss (mirror of CALIB_LOST_HOLD), `_POSTLUDE_NOLOCK` to stop floor re-inflation, and homing settles
(`PLAN→TURN→SETTLE→ADVANCE→SETTLE→PLAN`). All module self-tests green; **live-fly PENDING.**_

_**Session 15** (`plans/session15-trim-and-settle-fixes.md`): six fixes off the session-14 TRIM
flight (`20260714_113312`). (1) TRIM **pitch was reversed** → `trim_pitch_up=-1.0` (now climbs). (2) A
calibration **endless loop** (finish→lose-plan→retry) is now bounded: after 3 consecutive failed attempts a
new **`CALIB_ESCAPE`** state does a ring-picked push to a fresh vantage + holds for SLAM (12 frames + OK) then
retries; 3 more fails → **STUCK** (logging paused). (3) **SETTLE** no longer flies on a stale pose — a
goal-flying settle waits for **6 SLAM frames captured AFTER the settle began** (`cap_ts ≥ entry`) and
<1000 ms; the vertical prelude routine is exempt. (3b) **CALIB_VERIFY**'s timeout no longer PASSes-and-flies on
a stale pose — it feeds the same escape/STUCK guard. (4) Debugger shows **live height numbers** (ceiling /
desired / delta / all-flight median), dropped Δpos/Δgoal. (5) **fly.py** console flood fixed (restored
`NEW_CONSOLE`). All module self-tests green; **live-fly PENDING**. Parked: reverse-without-settling — diagnose
on the next log (the SETTLE gate may already fix it). Sessions 11/12/13/14 items fold into the same re-fly._

_**Session 14** (`plans/gradual-height-trim.md`): flight `20260713_223231` flew great (height-calib + parallax
fixes worked; **`CALIB_LOST_HOLD` fired 3× live and recovered cleanly — session 13 LIVE-PROVEN**). Built the
operator's **gradual height TRIM**:
a fine, dose-able altitude correction BETWEEN calibrations that uses the sim's PITCH aim (pitch the aim UP +
push forward → fly toward the raised aim = a gradual climb; the forward part feeds SLAM parallax, unlike a
discrete full-thrust `joy_vertical` pulse that chokes SLAM). At each calibration we now record ceiling_y /
desired_y / delta; on a fresh healthy frame in SETTLE or ADVANCE, if `pos_y` sank past
`ceiling_y + 1.2*delta`, a ring-gated TRIM climbs back (fwd-open → climb; else reverse/strafe to open forward
room; else abort+pray), preserving the committed goal (re-aims ORIENT, never re-picks). Also diagnosed two
things and wrote them up for a fresh session: the **return-to-origin ending** (`plans/return-to-origin-and-
graceful-dock.md`) and the **2-minute glass-wall bounce** (`plans/blacklist-region-and-counter.md`). All
module self-tests green; **live-fly PENDING**. Sessions 11/12/13 items also fold into the same re-fly.
Flight `20260713_101220` flew well then **"lost its shit"** after a parallax strafe; we diagnosed it
fully, wrote **`plans/strafe-throttle-and-recovery-loop.md`** (D1–D5), and **BUILT all five** (49/49 autopilot
self-tests pass; flow/frontier/ground_grid/perception green). **NEXT = live re-fly** of the same far-corner
scenario to confirm the fixes (watch the D1 caveat: does 0.2 actually slow the strafe? and the D2 reposition
displacement). The diagnosis: a full-magnitude strafe (`joy_horizontal −1.0` — strafe was the one axis never throttled
to 0.2) into an UNMAPPED side, while the drone was yawed, **scraped the wall → spun the drone to face it →
monocular SLAM died** (the spin is invisible in the log: the pose froze while the real airframe rotated). Then
a **frantic HOLD_LOST↔REWIND loop for 100+ s that could never die**, because a flickering SLAM status
(PLAN-LOST↔PLAN-STALE) RESET the recovery FSM every ~3 s (`_fallback_attempts=0` + a fresh non-consuming
`_invert_history()`), making `STUCK` mathematically unreachable. Raycast never fired: the forward ray is blind
to a lateral strafe, and the side ring read `None` (unmapped ⇒ treated as open). Session-11 height-calib +
session-10 tour/floor-dock still await their own clean live confirmation._

**Status:** Phase-1 (manual map + target localization) done & hardware-verified. Phase-2 autonomous
**Map-mode explorer** (`autopilot.py --explore`) flies live — clean session-8 flight
(`20260708_195009`). Session 8 confirmed **turns work** (the earlier "no-op" was a stale-heading logging
artifact), made the flight log **trustworthy** (logs the controller's committed goal + data staleness),
and added **`[SLAM_TRACKER]`** telemetry so the async ~2 Hz SLAM ticks are visible in the terminal. Next:
item 2 (REPLAN dead-stall) then item 1 (height calibration).

This file is three-fold: **Next** (resume-after-clear pointer), **Future** (the concise backlog → plan
files), and **Documentation** (the terse "we tried X, it failed because Y" narrative + the reference
blocks). Keep the Documentation half narrative — detailed designs live in `plans/*.md`.

---

## Next (resume after a context clear)

_**NEXT = LIVE-FLY SESSION 18** (BUILT — `plans/session18-command-smoothing-and-height-median.md`; io_bridge +
autopilot + flight_replay self-tests green). Run `python fly.py`, press `m` to hand over, and watch:_
- _**Smoothed flight** — forward legs + turns EASE in/out; a plan-loss brake is markedly GENTLER (thrust bleeds,
  no hard pitch-up/altitude jump). Open `OUTPUT/diag/<ts>_commands.csv` (now always-on): AUTO rows show `trigger`
  ramping 0.05 up / 0.1 down and yaw 0.05/tick — the SAME curve as MANUAL rows._
- _**Height median is sane** in the replay HTML — steps once per SLAM frame toward the live `pos_y`; no
  −0.008→−1.8 jump-with-no-new-frame, no frozen lag._
- _**RE-TUNE** afterward: throttle knobs (session-17 "lower the speed knobs") AND maneuver durations / back-off
  counts — smoothing attenuates short pulses (`flight_playbook.json`, turn durations, `strafe_reposition_fwd_s`)._
- _Session-17 items still hold: proper speed (thrust engaged), height HOLDS during horizontal flight, first
  calibration runs at takeoff; fly forward/strafe INTO a wall → CONFIRM the uncontrolled-climb (motivates the
  future wall-hit re-calibration)._
- _This one flight also confirms the still-pending sessions 17/16/15/14/11-13._

_**Session 16 — settle between every action + full return-to-origin ending**
(`plans/session16-settle-between-stages-and-return-to-origin.md`, BUILT + all module self-tests green). Watch:_
- _At mission end: **homes AT altitude → ORIENT_HOME faces the take-off heading → gentle dock → up-bump → DONE**
  — no descend-in-place, no jump-up, no "maniac" turning (homing settles between every turn/advance)._
- _A deliberate **SLAM loss during the dock** → `POSTLUDE_LOST_HOLD` (NOT HOLD_LOST/FALLBACK) → resumes the dock
  once SLAM+plan recover. `target_altitude_y` stays None through the descent (no floor re-inflation)._
- _Whenever recovery fires: a **neutral settle between every REWIND step and every spin FALLBACK attempt** (no
  back-to-back). A dead pipeline caps out at `recovery_settle_max_s` and still proceeds (logged)._
- _Knobs: `recovery_settle_frames` (4), `recovery_settle_max_s` (2.5), `home_reach_dist` (0.5)._

_**Session 15 — six fixes** (`plans/session15-trim-and-settle-fixes.md`, BUILT + all module self-tests green).
Watch:_
- _A **TRIM now CLIMBS** (`trim_pitch_up=-1.0` — the +1.0 was inverted). The before/after `pos_y` confirms it._
- _A **leg SETTLE waits for 6 fresh <1000 ms SLAM frames captured after the settle** before ORIENT (no more
  ORIENT one second after settle with a 2 s-stale pose). The vertical prelude routine stays timed._
- _A **looping re-calibration escapes**: after 3 fails → `CALIB_ESCAPE` (ring push + hold for SLAM) → retry;
  3 more → `STUCK` (logging paused). `CALIB_VERIFY` no longer flies to a goal on a stale/None pose._
- _The replay's **HEIGHT CALIBRATION** panel shows live ceiling/desired/delta + a constantly-updating median
  (Δpos/Δgoal removed). The launcher console is quiet (io_bridge back in its own window)._
- _**Parked:** reverse fired back-to-back without settling — diagnose on THIS flight's log (the SETTLE gate may
  already have fixed the `→SETTLE→REPLAN` reverses)._

_**Session 14 — gradual height TRIM** (`plans/gradual-height-trim.md`, BUILT). A whitelisted-state sag
(`pos_y > ceiling_y + 1.2*delta`, in SETTLE/ADVANCE) fires a ring-gated `TRIM` (pitch-up → forward → `c` →
frame-dated WAIT) that re-aims at the SAME committed goal (never re-picks). Two diagnosed-not-built items are
queued as their own plans: `plans/return-to-origin-and-graceful-dock.md` (the ending) and
`plans/blacklist-region-and-counter.md` (the glass-wall bounce)._

_**Session 13 — calibration survives a plan loss** (`plans/crystalline-swimming-floyd.md`, BUILT + self-test
green). Watch a per-goal `CALIBRATING_HEIGHT` where SLAM chokes during the re-tap:_
- _On the loss the state must go `... ASCEND → CALIB_LOST_HOLD` (NOT `HOLD_LOST`), with NO 1-tick
  `CALIBRATING_HEIGHT↔CALIB_LOST_HOLD` oscillation while `status` lags._
- _On recovery (≥6 fresh frames <1000 ms AND `status==OK`) it re-enters `CALIBRATING_HEIGHT` and completes
  `ASCEND→DESCEND→CALIB_VERIFY`; **altitude must DROP off the ceiling** (`pos_y` back toward the flying-height
  median — the `pos_y≈-2.2` glued symptom gone)._
- _If SLAM stays choked (or solves fast but the plan won't lock) exactly ONE DOWN bump appears, then a hold._

_**Session 12 — strafe throttle + un-killable recovery loop** (`plans/strafe-throttle-and-recovery-loop.md`,
BUILT + self-test-green). The five decisions, all built (watch a far-corner strafe + a SLAM loss):_
- _**D1 — Strafe throttle → 0.2.** Add config `strafe_throttle` (default 0.2) → `self._strafe_mag`; strafe was
  the one axis left at full 1.0. CAVEAT: `joy_horizontal` MIGHT be a discrete full-thrust axis like
  `joy_vertical` (documented identically "(-1 to 1)") — verify live that 0.2 actually slows it._
- _**D2 — Forward-reposition before a "scraping-danger" strafe.** When a parallax push resolves to STRAFE AND
  back-ring is very close (`strafe_backwall_danger_dist` ~0.4) AND the forward raycast is clearly open → a
  ~2.0s forward push @0.2 (`strafe_reposition_fwd_s`) to leave the tight/yawed corner, then strafe (coasts into
  a safe fwd-left diagonal). Else skip → throttled strafe._
- _**D3 — Recovery FALLBACK sweep uses the REAL ring-picked parallax push** (backward-first, else strafe to the
  roomier MAPPED side) at a **15°** step (`recovery_turn_step_deg`), not the blind fwd/back retreat._
- _**D4 — Kill the frantic loop / graceful death / bounded log.** Make STUCK reachable (D5); on terminal STUCK
  latch stuck-interval `[start,end]` + PAUSE the log spam; if a valid plan returns, resume mission + logging; at
  normal mission-complete the session-10 floor-dock postlude homes to origin, logs a mission-end summary
  INCLUDING the stuck ranges, then turns logging OFF (so the operator can walk away without a 200GB log)._
- _**D5 — Reverse-list lifecycle (core).** `_recovering` + `_history_broken` flags that PERSIST across
  PLAN-LOST/PLAN-STALE flickers (this is the loop fix). On first PLAN-STALE: freeze `command_history` appends,
  enter a CONSUMING pop-based REWIND (drain to empty → FALLBACK → STUCK; remove the counter resets at
  `autopilot.py:1299`/`:1744`). OK-return is NOT trusted: re-aim (ORIENT/parallax/ADVANCE) is unlogged + counter
  unchanged, and entering any spatial state sets `_history_broken`. A secondary drop: if `_history_broken` is
  False (still the initial rewind) continue popping; if True (drone already moved unconfirmed) CLEAR the stale
  history + BYPASS REWIND straight to the D3 FALLBACK sweep (no ghost path). Only a post-recovery ADVANCE that
  travels **≥1 SLAM unit** (`recovery_confirm_dist`) confirms: drop both flags, reset counter, clear the list,
  resume logging fresh._

_Build order suggestion: D1 (+ playbook) → D5 recovery FSM (the meat, has the most self-test surface) → D3 →
D2 → D4. Self-test after each (extend the recovery tests near `autopilot.py:3005-3055`), then a live re-fly of
the same far-corner scenario. Session-11 height-calib + session-10 tour/floor-dock still await clean live
confirmation and can fold into the same re-fly._

_Running the stack is now one command: **`python fly.py`** (spawns perception `--no-display` + autopilot
`--explore --log --stop-file` + visualizer + io_bridge in separate windows, then `Xlab.exe`; press `m` on
io_bridge to hand over; press ENTER in the launcher to stop — it drops the stop-file so the autopilot exits
CLEANLY, keeping the replay MAP backdrop, then auto-compiles + opens the report). The manual sequence still
works (`Xlab.exe` → io_bridge → perception → visualizer → `autopilot.py --explore --log`, press `m`)._

_**Session-11 build (flew `20260712`; all six module self-tests green):**_

1. _**State-gated height-calibration fix — BUILT, flew, UNDER SCRUTINY.** A continuous rolling baseline
   `_mapping_altitude_history` (ingested only in `MAPPING_ALT_STATES` at healthy SLAM, **frozen whenever
   `_calib_active`**) is judged AFTER the routine by the new `CALIB_VERIFY` (holds neutral, settlement gate
   on the plumbed `cap_ts`, None-guarded): settled `pos_y` significantly below the frozen median ⇒ FAIL ⇒
   `ASCEND_ESCAPE` (climb) → `CALIB_TRANSLATE` (slide 1u) → re-`CALIBRATING_HEIGHT` (bounded by
   `calib_max_retries`); PASS ⇒ "height OK" (unfreezes ingest). Retired the ceiling-tap median /
   `_is_low_object_tap` / `CALIB_NUDGE`. **Not yet proven to fully solve the low-drone occupancy poisoning —
   the operator is re-examining the flight.**_
2. _**Paired SLAM logging → REPLAY HTML (terminals stay clean) — BUILT + timestamp-fixed live.** Two
   records per fresh `frame_id`: `slam_start`(orange) positioned + labeled at the frame CAPTURE wall-time
   (from `cap_ts` via the loop-top monotonic→wall offset) and `slam_finish`(green) positioned + labeled at
   the log/`now` wall-time, stating the capture time + `Latency:` (= `slam_ms`) inline — so neither reads
   ahead of its playback slot (the first-flight "from the future" bug). NB: the green↔orange span is the
   FULL capture→controller latency; `Latency:` is only the SLAM solve, so the span is legitimately larger
   than the number (the gap = transport + perception post-work + the 0.5s plan timer + controller cadence)._
3. _**Timeline 1 ms skew — BUILT.** `now`/`now_wall` captured together at the loop top and used for both
   the SLAM rows and the step row (benign single-frame poll effect; replay still sorts by `t_mono`)._

_**Session-10 build — still needs its OWN clean live confirmation** (fold into a later flight): the
all-corners TOUR (frontiers exhaust → visit opposite → farthest-unvisited → last corner) + the floor-dock
postlude (home to origin → gentle pulsed descent, watch the NEW FLOOR latch, `dock_max_s` is the fail-safe
→ `STANDBY AT LOW HEIGHT`). Then the two Deferred ideas in `plans/all-corners-sweep-and-slam-parallax.md`:
(1) plan-lost-too-often investigation (SLAM choking?), (2) a parallax-strafe alongside each turn._

_Session-10 items BELOW were BUILT + all offline self-tests green (ground_grid / frontier_planner /
flow_contact_detector / autopilot / flight_replay / perception), live-fly pending:_

- **Part A — all-corners verification TOUR — BUILT (session 10).** Generalized the single opposite-corner
  sweep into a room-corner tour so every corner reconstructs densely (motivated by
  `DEBUG_IMAGES/mission_complete__mapping_so_so.png`). `ground_grid.sweep_corner` → **`bbox_corners(inset)`**
  (up to 4 inset corners, SW/SE/NW/NE, midpoint-collapse on narrow axes, deduped). `frontier_planner.select`
  now takes a corner LIST and TOURS them farthest-first (opposite → farthest-unvisited → last) via
  `_swept_corners` + `_pick_sweep_corner`; **corners IGNORE the frontier blacklist** (operator ask) and a
  walled-off corner is retired by a fresh 2-bump in `note_wall_hit` (not `_excluded`). Perception passes
  `bbox_corners` as `sweep_corners`.
- **Part B — post-mission floor-dock postlude — BUILT (session 10).** When the tour is exhausted
  (`done=True`) the drone no longer hovers at mapping height: **`RETURN_TO_ORIGIN → DOCK_FLOOR → LOW_STANDOFF
  → DONE`**. Homing is a self-contained turn→advance mini-loop to SLAM-frame `[0,0]` (clearance stand-off +
  altitude lock; `home_max_s` caps it → "dock here"). DOCK_FLOOR is a gentle **two-phase PULSED descent**
  mirroring the ascent (DOWN micro-pulses metered by the SLAM descent gain, then a continuous latch hold) —
  a continuous hold-down is forbidden (chokes SLAM). New **flow FLOOR detector** (`CMD_DOWN`, `|dy_med|`
  collapse, mirror of CEILING); `dock_max_s` is the fail-safe since FLOOR is new/unvalidated. LOW_STANDOFF is
  a short UP nudge; DONE logs `EXPLORE COMPLETE -> STANDBY AT LOW HEIGHT`.

_Session-9 items below BUILT + flew OK (`20260709_091706`, recoveries fine):_

- **Item 2 — REPLAN dead-stall → diagonal sweep — BUILT (session 9), flew OK.**
  Plan: **`plans/replan-deadstall-sweep-and-slam-tracker.md`**. Diagnosed on `20260708_195009`: the
  planner returned `goal=None && !done` and the controller idled forever — the done-verification stage
  silently never fired (the `farthest_free`/`verify_min_dist` "too near" gate failed). Fix (built):
  deterministic **bounding-box diagonal sweep** — `ground_grid.sweep_corner` (opposite corner, inset per
  axis with midpoint-clamp on narrow axes), `frontier_planner.select` reworked to sweep semantics
  (`sweeping`/`sweep_target`; never a `goal=None/!done` resting state), perception passes the sweep
  corner, and the autopilot gained a fail-visible bounded-idle backstop + a one-shot **EXPLORE COMPLETE**
  DONE log. Also **moved `[SLAM_TRACKER]` from the terminal into the replay HTML** (teal `ev_kind:"slam"`
  records). Operator note: room is only *mildly* mapped — deep interior coverage is the Part-3 next-phase
  idea below, not this fix.
- **Item 1 — per-replan height recalibration (`CALIBRATING_HEIGHT`) — BUILT (session 9), flew OK after two
  live fixes.** Fires on a genuine goal change (moved > `calib_goal_change_dist`) gated by a 60 s cooldown
  (also skips the first post-prelude goal); re-runs the two-phase ascend→descend, then orients to the same
  goal. Keeps a LIVE running median of ceiling taps and rejects a low-object tap (`pos_y` well below the
  median, +Y DOWN) → `CALIB_NUDGE` forward + re-ascend (bounded). **Two bugs found + fixed in live test:**
  (1) a spent `_player` from the interrupted leg leaked into DESCEND (guard `if _player is None` skipped
  the down-push) → `CALIBRATING_HEIGHT` now clears `_player` on entry, like the prelude's TAKEOFF; (2)
  re-latching `target_altitude_y` right after the re-tap pegged the hold target AT the ceiling (descend
  momentum hadn't dropped the drone yet) so the altitude lock fought it back UP → "glued to ceiling" — the
  re-latch was REMOVED (the re-tap resets the physical altitude; the prelude target stays valid). See
  `plans/glass-corner-blacklist-and-height-calib.md`.

---

## Future (backlog)
- **Session-18 command smoothing + height-median — BUILT, self-tests green, LIVE-FLY PENDING**
  (`plans/session18-command-smoothing-and-height-median.md`): autopilot trigger/reverse/yaw/pitch are now RAMP
  TARGETS io_bridge's 60 Hz loop chases (manual constants) → smoothed flight + smooth release; `--log-commands`
  re-added (always-on via fly.py); height-median ingests one reading per FRESH SLAM frame after the first calib
  (frame dedup, `MAPPING_ALT_STATES` retired). **NEXT = live-fly + re-tune throttle knobs AND maneuver durations
  (smoothing attenuates short pulses).**
- **Session-17 triggerDown fix + height simplification — BUILT, all six module self-tests green, LIVE-FLY PENDING**
  (`plans/session17-triggerdown-and-height-simplification.md`): Unity gates thrust on the
  `triggerDown`/`reverseDown` BOOLEAN (autopilot never set it → the "crawl"). Added them to `AUTONOMY_FIELDS` +
  `_neutralize_autonomy` (io_bridge) and DERIVED centrally in `autopilot._full_vector` from the analog value;
  deleted the periodic re-calibration trigger + all of TRIM (state/trigger/exit/vars/config/self-test); kept the
  first calibration + flight-height median + calib recovery. **NEXT = live-fly + re-tune speed knobs.**
- **Wall-hit-triggered re-calibration — FUTURE (the next thing to build after session 17 flies).** The drone
  holds altitude on its own EXCEPT it climbs uncontrollably when flying forward/strafe INTO a wall (reverse
  doesn't). Session 17 kept the `CALIBRATING_HEIGHT` machinery + flight-height median (both unwired now)
  specifically so a wall-contact event can trigger a re-calibration judged against the median. To wire: on a
  forward/strafe wall-contact event, `self._recalibrating = True; self._enter("CALIBRATING_HEIGHT")`.
- **Session-16 settle-between-stages + return-to-origin — BUILT (committed 44b4fa6), LIVE-FLY PENDING**
  (`plans/session16-settle-between-stages-and-return-to-origin.md`): confirmed on the same flight as session 17.
- **Session-15 six fixes — BUILT, all module self-tests green, LIVE-FLY PENDING**
  (`plans/session15-trim-and-settle-fixes.md`): TRIM pitch sign (`trim_pitch_up=-1.0`); calibration
  escape/STUCK guard (`CALIB_ESCAPE` + `_calib_fail_escalate`, config `calib_escape_*`); SETTLE fresh-frame
  gate (`settle_fresh_frames`, `_SETTLE_EXEMPT_NXT`); CALIB_VERIFY no-fly-on-stale (`TIMEOUT_FAIL`→escalate);
  debugger live height numbers (`alt_*` + `_alt_median`); fly.py `NEW_CONSOLE`.
- **Settle between every recovery action + full return-to-origin ending — BUILT (session 16), LIVE-FLY PENDING**
  (`plans/session16-settle-between-stages-and-return-to-origin.md`): shared `_settle_begin`/`_settle_poll` gate;
  a settle between REWIND inverse maneuvers and between spin FALLBACK attempts (resolves the parked
  "reverse-without-settling"); `home_reach_dist` 0.5, `ORIENT_HOME`, `POSTLUDE_LOST_HOLD` (dock survives a SLAM
  loss), `_POSTLUDE_NOLOCK` (no floor re-inflation), homing `TURN→SETTLE→ADVANCE→SETTLE`. Knobs
  `recovery_settle_frames`/`recovery_settle_max_s`.
- **Gradual height TRIM — BUILT (session 14), pitch sign fixed session 15, LIVE-FLY PENDING**
  (`plans/gradual-height-trim.md`): PITCH-aim + forward climb between calibrations; 3-value capture
  (ceiling_y/desired_y/delta) at CALIB_VERIFY; `pos_y > ceiling_y + 1.2*delta` trigger in SETTLE/ADVANCE;
  ring-gated (reverse/strafe reposition, else abort); goal-preserving exit. Config knobs `trim_*`.
- **Return-to-origin + graceful dock — BUILT (session 16), LIVE-FLY PENDING**
  (`plans/return-to-origin-and-graceful-dock.md` = diagnosis of record; built per
  `plans/session16-settle-between-stages-and-return-to-origin.md`): home at altitude (`home_reach_dist` 0.5),
  `ORIENT_HOME` to the recorded take-off heading, `POSTLUDE_LOST_HOLD` (DOCK survives a SLAM loss),
  `_POSTLUDE_NOLOCK` kills the floor-level altitude-lock re-inflation.
- **Glass-wall bounce (blacklist region + counter) — DIAGNOSED (session 14), plan written, NOT BUILT**
  (`plans/blacklist-region-and-counter.md`): widen the blacklist region past the frontier spacing + per-region
  bump tallies (stop the `counter defeated` thrash).
- **Calibration survives a plan loss — BUILT (session 13), LIVE-PROVEN (session 14 flight, 3× recover)**
  (`plans/crystalline-swimming-floyd.md`): `CALIB_LOST_HOLD` + `_calib_interrupted`; redo on a 6-fast-frame +
  `status==OK` SLAM-pulse recovery, one DOWN bump if stuck, `status==OK`-gated exit (anti-flicker).
- **Height calibration — BUILT + FLEW (session 11), NOT confirmed good** (`plans/height-calib-state-gate-and-slam-debug.md`):
  state-gated `CALIB_VERIFY`/`ASCEND_ESCAPE`/`CALIB_TRANSLATE`. The operator is dissecting the `20260712`
  flight log; expect follow-up questions on whether the low-drone occupancy poisoning is actually solved.
- **Paired SLAM logging + timestamp fix — DONE (session 11)**; `fly.py` one-command launcher — DONE.
- **REPLAN dead-stall (item 2)** — no infinite idle when the planner returns no goal. Designed:
  `plans/replan-deadstall-sweep-and-slam-tracker.md` (bbox diagonal sweep + SLAM_TRACKER → replay HTML).
- **Per-goal height calibration (item 1)** — BUILT session 9, live-fly pending
  (`plans/glass-corner-blacklist-and-height-calib.md`).
- **Glass-corner blacklist escape (Bug A+B)** — built session 7, still needs a clean live confirm.
- **Phase-2b — dense low-altitude interior mapping, then detection.** Operator idea: map the inner
  room near ground level so the target can be found there later. Recommendation (see item-2 plan
  Part 3): a low-altitude interior traverse is worth it for denser geometry, but a *blind SLAM-off*
  flight drifts (no pose feedback). For detection, prefer **(a) offline cascade on the recorded
  map-mode footage** (reuses map-mode poses; no GPU contention — start here) or **(b) a temporally
  interleaved Scan mode** (SLAM navigate → pause → detect → resume), NOT a pure SLAM-off pass.
- Deferred: Scan mode (360° cascade with SLAM/GPU temporal separation); a glass-window altitude
  descend-probe; Phase-3 report polish + GUI.

---

## Documentation (what we tried)

### Session 18 (2026-07-15) — gave autonomy the manual stick-smoothing; fixed the nonsensical height-median  [BUILT; io_bridge + autopilot + flight_replay self-tests green; live-fly pending]
Post-session-17 the drone finally thrusts, but autonomous flight was height-erratic — a hard brake + pitch-up +
altitude jump on every stop and plan-loss — while the operator's MANUAL flight is "very very controlled." He
suspected the missing piece was the smoothing he feels manually, and he was right. Diffing his `20260715_001039`
manual command CSV against how the autopilot drives showed it exactly: manual keys only toggle the
`trigger_down`/`reverse_down` (and arrow) GATES, and io_bridge's 60 Hz loop RAMPS the analog toward them
(`+0.05`/tick attack, `−0.1`/tick decay; yaw/pitch `±0.05` aim). The autopilot BYPASSED all of it — the overlay
hard-wrote the analog *after* the ramp and `_neutralize_autonomy` snapped to 0 — so every scripted thrust was a
hard step and every release a hard zero (the jolt). We made the autopilot's THROTTLE (trigger/reverse) RAMP
TARGETS the existing loop chases (reusing the manual constants), so thrust now eases in/out like a hand-flown
stick while KEEPING its magnitudes; release decays smoothly (aim + gates still snap for safety). A first live-fly
then taught us to LEAVE yaw/pitch UN-ramped: the turn is duration-not-magnitude (the sim eases the aim itself,
and the drone only rotates once the aim REACHES ±1), so a yaw ramp merely delayed reaching ±1 and shrank every
turn (30°→~5° — visible in the command log: yaw took 0.33 s to reach 1.0, leaving ~0.17 s of a ~0.5 s hold at
full deflection) — and it was double-smoothing on top of Unity anyway. So ONLY throttle is ramped; yaw/pitch pass
straight through (one tick), restoring the calibrated `turn_left/right` recipe (`turn_recipe_deg=90`, hold
1.625 s). The same flight also showed the **plan-lost pitch-up is a Unity braking response, not a pitch we send** — the
outgoing command log has ZERO non-zero pitch rows all flight, and trigger DOES decay smoothly on neutral
(0.4→0 at 0.1/tick). BUT a follow-up code read found a GAS-GATE TIMING miss that likely CAUSES that brake: the
`trigger_down`/`reverse_down` boolean (which Unity gates thrust on) was set from the COMMANDED analog, so it
dropped to False the instant a stop was commanded while the analog was still decaying → Unity hard-cut the thrust
and the smooth decay never reached it. Fixed: io_bridge now derives the gate from its own RAMPED analog
(`gate = analog > 0`), holding it True until the throttle reaches 0 (hypothesis — confirm on the next flight that
the pitch-up softens; harmless if Unity actually follows the analog). We also
re-added the `--log-commands` outgoing-packet CSV (regretted reverting it in session 17) — now permanent and
always-on via fly.py — so MANUAL vs AUTO smoothing is directly diffable (it was the tool that proved both the
yaw-delay and the zero-pitch findings above). Separately, the operator couldn't make
sense of the debugger's drone-height median (it jumped −0.008→−1.8 with no new SLAM frame, then wouldn't reach
the current height). Root cause: it appended every ~50 Hz control tick with no frame dedup — re-adding one stale
pose ~25× — and was seeded with ~0 ground samples during the pre-takeoff SETTLE. Now it ingests ONE reading per
FRESH `frame_id`, only after the first calibration reports height-OK, frozen during any calibration; the old
`MAPPING_ALT_STATES` state-gate is retired (measure in any state). New io_bridge `--self-test` (ramp) + a
rewritten autopilot ingest-gate test; all green. **Lesson: to make a scripted actuator behave like a human's,
replicate the platform's OWN input-conditioning (its ramp/gate model) rather than writing raw setpoints — and a
rolling statistic must ingest once per real SAMPLE (dedup by frame id), not once per consumer tick. And know your
actuator's model before you smooth it — the sim's YAW isn't a magnitude axis (it rotates at a fixed rate once the
aim saturates), so "smoothing" it only stole turn time; smooth THROTTLE, pass AIM through. CAVEAT: throttle knobs
still want the session-17 "lower the speed knobs" pass; turn durations are UNAFFECTED (yaw no longer ramped).**

### Session 17 (2026-07-15) — the triggerDown discovery: autonomous thrust was never engaged; simplified the height system  [BUILT; all six module self-tests green; live-fly pending]
For MONTHS the autonomous drone "crawled" (~0.02-0.04 u/s) and the height sagged, and we blamed SLAM/geometry.
While diagnosing the broken height TRIM we finally instrumented the FULL outgoing control vector (a temporary
io_bridge `--log-commands` CSV + a `t` trim macro + a `y` replay of a hand-flown trim) and diffed a MANUAL trim
against the autopilot's macro. The manual packets carried `triggerDown=True`; the macro's carried `False`. That
was it: **Unity gates REAL thrust on the `triggerDown`/`reverseDown` BOOLEAN, not the analog `trigger`/`reverse`
we'd been driving.** The autopilot never set the boolean (`AUTONOMY_FIELDS` omitted it, and io_bridge's smoothing
DECAYS the analog to 0 unless the boolean is held), so every autonomous forward/reverse ever flown ran with the
gas button UNPRESSED — the whole "crawl." The operator confirmed two things in manual: with the boolean held the
`t` macro "plays beautiful," and the drone HOLDS ALTITUDE on its own in horizontal flight — it only climbs
uncontrollably when flying forward/strafe INTO a wall (reverse doesn't). So the periodic height re-calibration +
the gradual TRIM had been fighting a SELF-INFLICTED sag that only existed because thrust was never on. We fixed
the root cause once, centrally: added `trigger_down`/`reverse_down` to io_bridge's `AUTONOMY_FIELDS` +
`_neutralize_autonomy`, and DERIVED them in `autopilot._full_vector` — the single choke point every command
flows through — from the analog value (`trigger>0 → trigger_down=True`), so all emit sites (presets, parallax
pushes, back_off, rewind/fallback reverses, homing) engage thrust with one edit. Then we DELETED the now-pointless
machinery: the periodic per-goal re-calibration trigger and ALL of TRIM (state, sag trigger, `_trim_exit`, vars,
the 3 ceiling/desired/delta references, config, self-test). We KEPT the first-takeoff calibration, the
flight-height median, and all calibration-recovery states — retained (unwired) for a FUTURE wall-hit-triggered
re-calibration, which the wall-climb behaviour now motivates. All six module self-tests green; live-fly pending,
and the speed knobs will need lowering now that the drone actually thrusts. **Lesson: when a whole platform
"just moves badly," LOG THE LITERAL BYTES LEAVING YOUR PROCESS and diff them against a known-good manual action
before building elaborate compensation — months of height machinery were treating a symptom of one unset boolean.**

### Session 16 (2026-07-14) — a SETTLE between every action (recovery + postlude) + the full return-to-origin ending  [BUILT; all module self-tests green; live-fly pending]
A test flight finished its last corner, tried to return to origin, and fell apart — it "turned like a maniac,"
fired the reverse-list back-to-back with NO settles, exhausted itself, fell back to spinning (also no settles),
declared STUCK, then retried. We recognized ONE pattern in three places: commanded actions fire back-to-back
with no still window for monocular SLAM to re-lock, so the pose stays frozen/stale and the failure compounds —
the same thing session 15 fixed for the per-leg SETTLE, just never applied to the recovery mechanisms or the
postlude. So we generalized the session-15 SETTLE tracker into a **shared gate** (`_settle_begin` /
`_settle_poll`, two flavors — HEALTHY `require_fast=True` and a bounded LOST-SLAM `require_fast=False` that gates
on fresh CAPTURE only, since SLAM is STALE by definition during recovery) and refactored the SETTLE state onto
it (behavior-identical). Then we put a settle **between every REWIND inverse maneuver and every spin FALLBACK
attempt** (bounded by `recovery_settle_max_s` so a dead pipeline still re-exposes) — resolving the session-15
parked "reverse-without-settling." The operator then caught a related ordering bug: each spin FALLBACK attempt
was `push → turn`, leaving a BARE ROTATION as the last motion before the settle — exactly the SLAM-killer, right
when we ask it to re-lock. Flipped to **`turn → push`** so the parallax translation is last (rescues the
rotation for RELOC; also matches the established "reset attitude with 'c' BEFORE a push" recipe, since a turn is
`yaw + 'c'`). The operator also chose to build the **full return-to-origin ending** in the
same session (it's where the mess showed up): `home_reach_dist` 1.0→0.5 (so it homes at altitude instead of
docking 0.86u out), a new **ORIENT_HOME** state facing the recorded `_takeoff_heading`, a **POSTLUDE_LOST_HOLD**
(mirror of CALIB_LOST_HOLD) so a SLAM loss during the dock HOLDs + resumes instead of thrashing into recovery,
**`_POSTLUDE_NOLOCK`** to stop the flying-height altitude lock from re-inflating a floor-level drone, and homing
settles (`PLAN→TURN→SETTLE→ADVANCE→SETTLE→PLAN` — the direct "maniac turning" fix). All module self-tests green
(new inter-action-settle + ORIENT_HOME-bearing-wrap + DOCK-survives-loss + no-re-inflate tests). **Lesson: the
"settle so SLAM can re-lock" discipline isn't just for the mapping loop — every place that emits a maneuver
(recovery, homing, orient) must give the monocular solver a still window, or it thrashes; a bounded settle
(fresh-capture-verified, time-capped) is the general primitive.**

### Session 15 (2026-07-14) — TRIM pitch fix + calib escape/STUCK + SETTLE fresh-frame gate + debugger numbers  [BUILT; all module self-tests green; live-fly pending]
The session-14 TRIM flight (`20260714_113312`) surfaced six things. (1) The TRIM **pitch axis was inverted** —
`+1.0` aimed DOWN so the drone descended; flipped to `-1.0`. (2) When SLAM got badly confused a re-calibration
**looped forever** (finish/interrupt → lose plan → redo → …); we bounded it with a shared `_calib_fail_escalate`
counter and a new `CALIB_ESCAPE` state — after 3 consecutive failed attempts, push once to a fresh vantage +
hold for SLAM (12 fresh frames + OK) then retry; 3 more → `STUCK` (logging paused). (3) The operator caught a
`SETTLE` that fired `ORIENT` ~1 s later with the last SLAM solve ~2 s stale → a shaky pose → plan loss; a
**settle must SETTLE**. Fixed: a goal-flying settle now waits for **6 SLAM "done" frames CAPTURED after the
settle began** (`cap_ts ≥ entry`) and under 1000 ms — the running streak was stale-high (frames had stopped
arriving), so we count frames by their capture time, not a pre-existing streak. The operator challenged an
early claim that the prelude runs before SLAM tracks — the data proved him right (SLAM is solving from the
first ARM tick, `frame_id=670`), so the vertical prelude routine is exempt by role, not by track status. (3b)
`CALIB_VERIFY`'s 5 s timeout used to PASS-and-fly to a goal on a stale/absent pose — now it counts a failed
attempt and feeds the same escape/STUCK guard. (4) The debugger's useless `Δpos/Δgoal` was replaced with a
**HEIGHT CALIBRATION** number group (last ceiling/desired/delta + the all-flight rolling median CALIB_VERIFY
judges against). (5) The launcher console flooded because two services had lost their `NEW_CONSOLE`. All module
self-tests green (new SETTLE-gate + CALIB_ESCAPE tests; `_drive` now injects a live frame stream so the gate is
exercisable). **Lesson: a "settle" that trusts a running health streak can proceed on a frozen-but-recently-
healthy track — gate on frames whose CAPTURE time is after the settle started; and always CHECK THE DATA before
asserting what the prelude does.** Parked: reverse fired without settling — diagnose on the next log.

### Session 14 (2026-07-14) — gradual height TRIM (pitch-aim climb); diagnosed the ending + glass-wall bounce  [BUILT; all module self-tests green; live-fly pending]
Flight `20260713_223231` flew great and **live-proved `CALIB_LOST_HOLD`** (it fired 3× and recovered every
time — session 13 confirmed). But the drone still gradually LOST height: calibration only re-taps on a goal
change, and ~half the flight sat in SLAM_HOLD/HOLD_LOST where the discrete `joy_vertical` altitude-lock never
corrects. We wanted a FINE, dose-able vertical primitive. The operator's idea: use the sim's PITCH aim — pitch
the aim UP and push forward, and the drone flies toward the raised aim = a GRADUAL climb (rate = push
duration), the forward part feeding SLAM parallax (a pure `joy_vertical` pulse stretches vertical features and
chokes SLAM — exactly what bit DOCK_FLOOR this flight). Built it: at each calibration's `CALIB_VERIFY` pass we
record `ceiling_y` (climb peak), `desired_y` (settled), `delta`; on a fresh healthy frame in SETTLE/ADVANCE, if
`pos_y > ceiling_y + 1.2*delta` (== sunk >20% of the ceiling gap below desired) a **`TRIM`** state runs: a ring
gate picks a safe way to climb-forward (fwd-open → climb; else reverse to open forward room; else strafe to an
open side; else abort+"pray", all visible), then pitch-up (`trim_aim_s`) → forward push with pitch still up
(`trim_fwd_s`) → `c` reset → WAIT for a healthy frame CAPTURED ≥ `trim_cmd_t0 + trim_settle_s` (the async-SLAM
guard, same monotonic clock as CALIB_VERIFY) → LOG the post-trim height. It **preserves the committed goal**
(snapshots `leg_goal`, re-aims ORIENT at it on exit — never re-picks, so a trim can't pollute goal
commitment). Four review "traps" folded in: forward-push stays interruptible by the live flow/ram guards
(A); goal snapshot+restore (B); the cap_ts↔now monotonic baseline is the project's proven CALIB_VERIFY gate
(C); the 3 values are captured only at a settled CALIB_VERIFY pass, never mid-wobble (D). All module
self-tests green (incl. a new HEIGHT-TRIM test). **Also diagnosed but NOT built** (each its own plan for a
fresh session): the **return-to-origin ending** — `home_reach_dist=1.0` made it "reach" origin 0.86u out and
dock in place from flying height; the dock then lost SLAM → recovery loop; the flying-height altitude-lock
re-inflated the floor-level drone → land/crawl/jump (`plans/return-to-origin-and-graceful-dock.md`); and the
**2-minute glass-wall bounce** — the blacklist region is smaller than the frontier spacing (whack-a-mole,
every blacklist `1 total`) AND the 2-bump counter reset when the planner alternated to distant goals
(`counter defeated` ×209) (`plans/blacklist-region-and-counter.md`). **Lesson: `joy_vertical` being a discrete
full-thrust axis is WHY we had no gentle altitude trim; the pitch-aim + forward "fly toward your aim" trick is
a gradual, SLAM-friendly vertical primitive — and any brief interrupt maneuver must snapshot + restore the
committed goal so it doesn't pollute the mission's goal commitment.**

### Session 13 (2026-07-13) — a plan loss during a ceiling re-tap erased the calibration; built CALIB_LOST_HOLD  [BUILT; all self-tests green; live-fly pending]
Flight `20260713_163055`: a per-goal `CALIBRATING_HEIGHT` fired, and mid-ASCEND (flush at the ceiling) SLAM
ground on the frozen image for 2.8 s — long enough that `plan_age` crossed `plan_timeout_s` and a **brief
PLAN-LOST** fired. The global recovery guard forced `HOLD_LOST`, and when the plan returned ~0.28 s later the
normal path funnelled `SLAM_HOLD→SETTLE→REPLAN` — the mission leg loop, **with zero memory of the
calibration**. The DESCEND never ran, so the drone **stayed glued to the ceiling (`pos_y≈-2.2`) for the whole
rest of the flight**. We wanted the calibration to SURVIVE a loss. Built a dedicated, telemetry-visible
`CALIB_LOST_HOLD` state (`plans/crystalline-swimming-floyd.md`): on any loss (LOST/NO-PLAN/STALE) while
`_calib_active`, latch `_calib_interrupted`, release controls, and hold watching the SLAM frame pulse
(`slam_ms`, the true liveness signal — even when the coarse plan status lags). **Redo** the whole calibration
once ≥6 fresh frames solve <1000 ms **AND** `status==OK`; **bump DOWN once (max)** if either the SLAM solve
stays choked (≥6 slow frames → wake SLAM) OR it solves fast but the planner still won't lock a path.
**Two traps caught in review:** (1) the redo exit MUST be gated on `status==OK`, not the frame streak alone —
`status` is level-triggered and lags a healthy SLAM, so exiting on the streak would re-enter the guard on the
next (still-lost) tick, wipe the streaks, and **1-tick-oscillate `CALIBRATING_HEIGHT↔CALIB_LOST_HOLD` forever**;
(2) emit the descend bump's first frame on the trigger tick, not a wasted neutral tick. **Lesson: a maneuver
interrupted by a transient loss must remember it was mid-maneuver — dropping into generic recovery silently
abandons the sub-mission; and any exit gated on a fast signal (SLAM frames) must ALSO wait for the slow
level-triggered signal (plan status) to catch up, or the two race into an oscillation.**

### Session 12 (2026-07-13) — diagnosed the strafe scrape-spin + the un-killable recovery loop; built the fix  [BUILT; all self-tests green; live-fly pending]
Flight `20260713_101220` flew well, then died after a parallax strafe. We wanted to know why, and found two
distinct failures. **The death:** at a far, tightly-boxed corner the planner correctly chose `strafe_left` (back
+ right were too close to push into), but the strafe fired at FULL magnitude (`joy_horizontal −1.0`) — strafe
turned out to be the ONE control axis we never throttled to 0.2 like advance/reverse — into a side the map read
as `None` (unmapped ⇒ `_pushable` treats it as open room). Because the drone was yawed relative to that wall, a
full-tilt lateral shove **scraped the wall, torqued the airframe into a spin, and swung the camera to face the
wall → monocular SLAM died.** The spin never shows in the log because the last good pose freezes while the real
drone keeps rotating — a lesson in itself. Raycast couldn't have saved us: the forward clearance ray is blind to
a sideways strafe, and the side ring was `None`. **The frantic loop after:** the drone thrashed `HOLD_LOST↔REWIND`
for 100+ s and could never give up, because a flickering SLAM status (PLAN-LOST↔PLAN-STALE, ~every 3 s) RESET the
whole recovery FSM each cycle — `_fallback_attempts=0` + a fresh, non-consuming `_invert_history()` — so `STUCK`
was mathematically unreachable and the reverse-list never emptied (exactly the operator's intuition). We built
`plans/strafe-throttle-and-recovery-loop.md` (all self-tests green): throttle the strafe (`strafe_throttle` 0.2);
a gated forward-reposition out of a scrape-danger corner before strafing; a recovery that CONSUMES its
reverse-list and PERSISTS across the flicker so it marches REWIND→FALLBACK→STUCK; a "don't trust the re-lock
until we've flown ≥1u" rule (`_recovering`/`_history_broken` flags, confirming ADVANCE) with a ghost-path guard
(a secondary drop after the drone has moved unconfirmed clears the now-spatially-stale history and jumps straight
to the ring-picked fallback sweep at a gentle 15° step); and a graceful STUCK that latches the stuck interval,
pauses the per-step log spam, and reports+closes it at the normal mission-end home/dock. Live re-fly pending.
**Lesson: a wall CONTACT that
induces a SPIN is invisible to a pose-based log (the pose freezes) — and a recovery FSM whose progress + give-up
counter can be reset by the very status flicker a real loss produces can never terminate.**

### Session 11 (2026-07-12) — the height-calib bug was JUDGING TOO EARLY; state-gated it + paired SLAM spans  [built; self-test-green; FLEW 20260712, calib not yet confirmed]
Session-10 flew, but a per-goal re-calibration on `20260709_122349` left the drone ~0.5u LOW: it re-tapped
the ceiling, did its brief descend, but async SLAM only caught up mid-move and it sank to `pos_y=-1.768` in
`PARALLAX_PUSH` (which doesn't hold altitude) — and because occupancy is built from a slab relative to the
LIVE camera Y, a low drone clipped standoffs and blacklisted valid frontiers. The old defence (reject a
ceiling tap below the running median of TAPS) was wrong twice over: too few taps to know "normal", and it
judged AT the ceiling before the drone had settled. **So we stopped judging the tap and judged the RESULT
after the routine ends.** A continuous rolling baseline of NORMAL flying altitude
(`_mapping_altitude_history`, ingested only in steady mapping states at healthy SLAM, FROZEN during any
calibration) is the reference; a new `CALIB_VERIFY` holds neutral after the descend, waits a settlement gate
on the plumbed camera-capture timestamp (`cap_ts`, None-guarded so a dropped frame can't crash), then
compares the SETTLED `pos_y` to the frozen median — significantly lower ⇒ the calibration sank the drone ⇒
climb to clean airspace (`ASCEND_ESCAPE`) BEFORE sliding 1u sideways (`CALIB_TRANSLATE`, never translate
while sunk) ⇒ retry; else "height OK" unfreezes ingest. Separately, to see WHY SLAM spikes, the autopilot now
emits PAIRED `slam_start`(orange)/`slam_finish`(green) replay records keyed on `frame_id` — in the browser,
terminals stay clean. The first live flight (`20260712_123815`) exposed a timestamp bug: each record sat at
the frame's own `t_mono` but was LABELED with the ~0.6s-later processing wall-time, so the orange START read
"from the future" during playback. Fixed to a dead-simple convention: START is positioned + labeled at the
frame CAPTURE wall-time (derived from `cap_ts` via the loop-top monotonic→wall offset); FINISH is positioned
+ labeled at the log/`now` wall-time and states the capture time inline (`"… finished working on the frame
#N from: [capture] … Latency: Nms."`) — so nothing reads ahead of its playback slot. (Follow-up Q from the
operator: the green↔orange span (~2.4s) is much bigger than the `Latency:` number (~1.8s). Correct + by
design — the span is the FULL capture→controller latency; `Latency:` is only the SLAM solve (`slam_ms`
wraps just `slam.process`); the ~0.6s difference is transport + perception post-work + the 0.5s plan timer +
the controller's loop cadence. The frame bus is conflated so there's no giant FIFO backlog.) We also added a
one-command **`fly.py`** launcher that stops the autopilot GRACEFULLY via a stop-file sentinel (a parent
can't Ctrl+C a separate-console child on Windows), so the report keeps its shutdown-emitted occupancy-map
backdrop, then auto-compiles the replay. All six module self-tests green. **Height calibration flew but is
NOT yet confirmed — the operator is still dissecting the flight. Lesson: a settling maneuver must be judged
AFTER it settles, against a general "normal" baseline — not at the peak, and not against a handful of
samples that can't define normal. And a replay record's shown time must be the time of WHERE it sits, not
when it was written.**

### Session 10 (2026-07-09) — all-corners verification tour + a post-mission floor-dock postlude  [built; self-test-green; live-fly pending]
Session-9 flew fine but reconstruction was UNEVEN: the drone flew one main diagonal, so occupancy was
dense on that line and thin at the two off-path corners (`DEBUG_IMAGES/mission_complete__mapping_so_so.png`).
We wanted every corner mapped, and a graceful ending instead of a hover at ceiling height. So (A) we
generalized the single opposite-corner "sweep" into an **all-corners TOUR** — `ground_grid.bbox_corners`
returns the inset bbox corners and `frontier_planner.select` visits them farthest-first (opposite, then the
far one of the rest, then the last), each cached statically while flying to it. Per the operator, **corner
targets ignore the frontier blacklist** — a genuinely walled-off corner is retired by the SAME event-driven
2-bump that retires unreachable frontiers (marked "visited" in `note_wall_hit`), which keeps termination
without a stale filter suppressing a corner we simply haven't reached yet. And (B) a **floor-dock postlude**:
on `done`, fly home to the take-off origin, then descend GENTLY to the floor and stand by low. The descent
MIRRORS the two-phase ceiling ascent (DOWN micro-pulses metered by the live SLAM descent gain, then a
continuous latch hold) — a continuous plunge would stretch the vertical features and choke SLAM right at the
finish. This needed a NEW flow **FLOOR** detector (the exact mirror of CEILING: descending `|dy_med|`
collapses to ~0 on floor contact); since it's unvalidated (unlike CEILING/WALL) a `dock_max_s` cap is the
fail-safe. All six module self-tests green. **Lesson (caught in review): a homing branch that computes a
fresh bearing needs its own angle-wrap — the self-test only exercised the at-origin path, so a missing
`_wrap180` hid until we added a turn+advance homing test; always drive the branch that does the math.**

### Session 9 (2026-07-09) — killed the REPLAN dead-stall with a bbox diagonal sweep; SLAM_TRACKER → replay HTML  [built; self-test-green; live-fly pending]
The clean session-8 flight still ended "doing nothing in a loop": the planner returned
`goal=None, done=False` and the controller idled in REPLAN until SLAM drifted → HOLD_LOST. Root cause —
the done-**verification** stage EXISTED but was silently bypassed: it only started when a fragile gate
passed (`farthest_free` non-None, not excluded, **> verify_min_dist**), and when that gate failed
`select()` fell through to a silent `return None, False` idle. So we replaced the whole fragile path with
the operator's idea: a deterministic **diagonal sweep** — take the known bbox, fly to the corner
OPPOSITE the one nearest the drone, inset ~1 u so it's reachable; if the traverse surfaces new frontiers
resume exploring, else declare a visible **DONE**. Built as `ground_grid.sweep_corner` (per-axis inset
with a **midpoint clamp** on axes narrower than 2·inset, so a corridor never overshoots its short axis
out of bounds), a reworked `frontier_planner.select` (`sweeping`/`sweep_target`; guarantees it never
rests on `goal=None/!done`), perception passing the corner, and an autopilot fail-visible **bounded-idle
backstop** (`no_goal_idle_s`) + one-shot EXPLORE-COMPLETE log. Separately, per the operator's ask, the
`[SLAM_TRACKER]` per-pose stream was **moved out of the terminal into the replay HTML** (teal
`ev_kind:"slam"` records, interleaved by time). Also built **item 1 — per-goal height re-calibration**
(`CALIBRATING_HEIGHT`): on a genuine goal change (past a 60 s cooldown) the drone re-taps the ceiling
(reusing the two-phase ascend→descend), re-latches `target_altitude_y`, then orients to the goal; a tap
well below the LIVE running median of taps is a low object → nudge forward + re-ascend (bounded). All
offline self-tests green (planner/ground_grid/autopilot/flight_replay). **Lesson: a "verify then done"
stage guarded by a fragile distance gate can silently choose to do NOTHING — make the terminal branch
deterministic (a goal or a flagged done), never a bare no-op.**

### Session 8 (2026-07-08) — "turns are broken" was a logging lie; made the flight log trustworthy
First flight (`20260708_135719`): the heading changed ~0° during every ORIENT turn, and travel bearing
matched reported heading on every leg, so we *concluded the body wasn't rotating*. We instrumented the
turn (log-bomb "TRYING TO TURN") and re-flew (`20260708_154431`). **The operator watched the drone
physically TURN — the conclusion was wrong.** Root cause: `heading` is the SLAM pose heading, published
~2 Hz and barely resolvable during pure rotation, so a whole ~1 s turn completes inside one perception
interval — the log repeats the same heading, then jumps ~45° one update later (heading sweeps the full
±180° over the flight). The **real bug was the LOGGING:** the timeline logged perception's async plan
(goal/heading/pos), not the controller's acted-on state — so a "goal reached (d=0.55)" printed next to a
shown goal 3.65 u away (the shown goal was perception's newer pick; the drone reached its committed
`leg_goal`), and a goal "changed" mid-advance simply because a fresh plan replaced the held snapshot.
**Fixes:** (1) the timeline now logs the committed `leg_goal` as `goal` (+ `dist_to_goal`), keeps
perception's pick as `plan_goal`, and exposes staleness (`plan_age_s`, `frame_id`); `flight_replay`
renders the committed goal and greys held-stale pose. (2) a synchronous **`[SLAM_TRACKER]`** line prints
every fresh pose the autopilot accepts (`dx/dy/dYaw [mode] - SLAM Latency`) so the ~2 Hz SLAM ticks are no
longer dark between state logs. (3) small eases: SLAM-settle 3→6, reach 0.4→1.0, clearance 0.6→1.0,
plan-lost grey goal marker. A follow-up flight (`20260708_195009`) flew cleanly with the corrected,
readable telemetry. **Lesson: a held-stale ~2 Hz pose logged every ~33 Hz loop tick makes a fast maneuver
look motionless — log what the controller ACTS ON, and always expose data age.**

Also **diagnosed but NOT fixed** (queued as item 2): a "blacklist with nothing blocking" that ends in a
dead stall — the forward-clearance stand-off (fwd_clear≈0.5 < 0.6) counts as a blacklist *bump*, two in
~2 s retire a reachable goal, and once every reachable goal is blacklisted the planner returns
`goal=None, done=False` and the drone idles in REPLAN forever (`autopilot.py:1378`).

### Session 7 (2026-07-08) — glass-corner blacklist escape (Bug A+B) + frontier clearance buffer  [built; flew in the session-8 flights, glass-corner escape not yet specifically re-confirmed]
A glass corner still trapped the drone forever: it fired standoff stops "like crazy" yet never retired
the goal. Two coupled bugs. **Bug A** — when no frontier was reachable the planner flew to `farthest_free`
as a fixed verify target that NEVER consulted the blacklist, and `farthest_free` is a plain geometric
argmax, so it re-picked the SAME dead corner; the 2-bump blacklist fired but was a no-op. Fix: made
`farthest_free` blacklist-aware (an `exclude` predicate skips dead regions), and `select()` now abandons
a verify target the moment its region gets blacklisted, re-caching a fresh corner or declaring done — and
caches that corner pulled 25 % back toward the drone for a vantage off the wall. **Bug B** — once SLAM
mapped the wall, the clearance stand-off stopped ADVANCE and went straight to SETTLE, so the drone never
reversed/displaced and the bump latch never re-armed (counter stuck at 1). Fix: a small `back_off` on the
standoff stop (gated `backoff_on_standoff`) whose reverse re-arms the latch (and seeds SLAM parallax), so
a second standoff counts and the corner reaches 2 bumps. **Also** added a general goal-stalling guard: a
committed frontier goal is pulled back along the drone→goal axis to a map-validated FREE cell with a
clearance buffer (`inset_to_clearance`), publishing a visible `goal_clearance_ok` flag (no silent
fallback). All module self-tests green; **live re-fly still pending.**

### Session 6 (2026-07-08) — blacklist/telemetry observability + self-calibrating ram guard
We couldn't tell WHY goals were being blacklisted. We added per-bump logging (PLANNER / MISSED-BUMP +
a live 2-bump counter in the replay timeline) and a per-frame raw-telemetry panel to `flight_replay`
(SLAM x/y/z, yaw, the literal command dict sent to the sim, Δpos, dist-to-goal, plan status). The logs
proved the blacklists were FALSE: the ram guard demanded the drone close ~0.05 u/s toward the goal, but
the drone crawls at ~0.02–0.04 u/s, so in OPEN space (clear ahead, healthy SLAM) it kept firing
"invisible collider" and two such false stops retired a reachable goal. **Fix — self-calibrating ram
guard:** measure the drone's OWN nominal free-flight speed live (1 s into the first ADVANCE, sampled
≤5 s or until a SLAM event), then fire only when the live windowed speed drops below 33 % of nominal.
Re-flew: no ram-guard false positives. Deferred: the glass-corner blacklist bugs + Part 3 height
calibration (see the plan file).

### Session 5 (2026-07-07) — dropped depth-map height logic; two-phase gentle ceiling ascent
Because the sim can't physically crash, we removed all depth-based height keeping and freed the GPU
for SLAM.
- **Removed the depth-map height patches** (the "low inner wall" bump-up / BUMP state) from the
  autopilot and **disabled DA-V2 depth inference entirely** in perception (it only fed the removed
  bump-up + the dashboard). SLAM now owns the GPU alone — peak VRAM ~9.7 → 6.75 GB — and the wall
  stand-off already used the SLAM raycast, not depth. The visualizer shows an explicit
  "DEPTH DISABLED" panel (no silent hang). The SLAM-pose **altitude lock** stays.
- **Two-Phase Hybrid Ascent** replaces the old continuous full-thrust climb that built momentum and
  smashed the ceiling (hurting SLAM). `joy_vertical` is a DISCRETE ±1 axis (can't throttle), so:
  - **Phase 1** — short UP micro-pulses; after each pulse read the live SLAM altitude gain and keep
    pulsing while still rising, so the drone approaches the ceiling with near-zero momentum.
  - **Phase 2** — once the gain flattens (flush at the ceiling), hold UP continuously so the existing
    flow CEILING detector latches a clean, low-velocity contact. (A single continuous hold is needed
    because the detector only latches within one uninterrupted pulse.)
- **Baseline nudge** — after the ceiling tap + descend, a short horizontal translation seeds a SLAM
  translational baseline before the first turn (pure rotation is the known SLAM-killer).
- **Deferred — Part 3** (per-goal `CALIBRATING_HEIGHT`) — now item 1 in Next/Future.
- **Tests:** autopilot / flow / frontier / ground_grid / perception self-tests PASS.

### Session 4 (2026-07-06) — event-driven 2-bump blacklist (replaced a broken time-watchdog)
Symptom: at a glass wall the drone sat ~9 min never blacklisting the unreachable beyond-glass goals.
- **Root cause:** the unreachable-goal watchdog was a *time accumulator gated on SLAM health*. In the
  glass pocket SLAM ran hot but the drone kept flying on valid poses, so the accrual clock stayed
  frozen and never fired. **Lesson: time-accumulation proxies gated on SLAM health go blind exactly
  in the heavy glass/wall pockets.**
- **Fix — event-driven 2-bump rule:** the autopilot reports each discrete advance-blocked stop as a
  "bump"; TWO bumps on the same goal region permanently blacklist it (a bump elsewhere resets the
  count). Immune to SLAM-clock health; a kinematic latch makes one continuous contact = one bump.
- Also added reverse **BACKWALL** contact detection (detection-only; logs a reverse-into-wall).

### Session 3 (2026-07-06) — flight-replay debug tool
Built `flight_replay.py`: the autopilot writes a structured per-step `*_timeline.jsonl` on `--log`,
and the tool renders a self-contained animated HTML (top-down scene + scrubber + event log + SLAM-ms
sparkline) so a flight can be debugged without reading 2000-line text logs. Self-test-verified.

### Session 2 (2026-07-06) — corrected glass model + flight fixes
A live flight showed the earlier "glass-stuck" watchdog was built on a WRONG glass model.
- **Correction:** the monocular camera looks THROUGH clear glass and tracks features on the far side,
  so **SLAM stays healthy and the clearance ray reads clear** — the drone hits the invisible collider,
  bounces, pushes again (an "invisible treadmill"). A watchdog that required SLAM to choke + the path
  blocked was exactly backwards.
- Other fixes: a no-spin startup that holds for SLAM instead of a blind 360° sweep; and a pos-space
  **ram guard** that stops a slow ram into an opaque wall before the frozen image kills SLAM.

### Earlier (2026-06-27 → 07-05) — Phase-2 explorer build & the goal saga
- **Ceiling detector v1** (SLAM-pose rate/plateau) **failed twice live** — monocular pose is only
  ~1 Hz, so the rate window never armed. **Lesson: validate detectors on REAL captured data, not
  synthetic streams.** → pivoted to `flow_contact_detector.py` (CPU optical-flow, self-calibrating):
  CEILING = vertical flow collapses while ascending; WALL = radial looming collapses while moving
  forward. Validated on real flights.
- **Turns vs SLAM:** closed-loop-on-heading thrashed (heading goes stale mid-spin); a "pulsed" yaw was
  wrong (yaw latches). Settled on **open-loop quantized turns clamped to ≤45°** (a small turn doesn't
  kill SLAM; the per-leg replan is the outer correction). **[Session 8: verified these turns DO rotate
  the body — a live re-fly showed the drone turning; the earlier "no-op" reading was the SLAM heading
  lagging in the log (~2 Hz, pure rotation), not the drone.]**
- **Ramming a wall kills monocular SLAM** (no parallax freezes the image); reversing a dead track
  can't revive it. → the **forward-clearance stand-off** (SLAM raycast) is the primary wall stop; the
  flow WALL detector is the fallback.
- **Frontier planner** (`frontier_planner.py`): utility selection + strong commitment + done-
  verification (fly to the farthest free corner, then declare done) — fixed goal thrash and false
  "mission complete".
- **Control-space SLAM-loss recovery** (pose is invalid during a loss): PLAN-LOST → hard hover-hold;
  PLAN-STALE → replay the inverse of recent maneuvers to re-expose keyframes; history empty → a
  bounded ≤45° fallback sweep → STUCK.
- **The unreachable-goal saga:** a goal behind glass / a wall is never consumed, so the planner
  re-hands it forever. The handling went through several dead ends — a position-conditioned watchdog
  (an A→B→A **ping-pong**), a round-based permanent blacklist, then a distance-stagnation timer — each
  failing because it inferred "unreachable" from a proxy that went blind in the glass pocket. Session
  4's **event-driven 2-bump** rule finally holds.

### Open issues
- **Return-to-origin ending + inter-action settles (session 16) — BUILT + all self-tests green, LIVE-FLY
  PENDING.** New states `ORIENT_HOME` / `POSTLUDE_LOST_HOLD` and the recovery/postlude settles have never flown.
  Watch live: the ending homes AT altitude → faces the take-off heading → gentle dock → up-bump (no
  descend-in-place / no jump-up / no maniac turning); a SLAM loss mid-dock → `POSTLUDE_LOST_HOLD` → resume (not
  recovery); a neutral settle between every REWIND step + spin attempt. If `recovery_settle_max_s` (2.5) is too
  short/long or `home_reach_dist` (0.5) too tight, adjust. `_takeoff_heading` is captured from the first healthy
  post-prelude `heading_deg` — confirm it reads a stable heading, not a wobble.
- **`CALIB_LOST_HOLD` (session 13) — BUILT + self-test green, LIVE-FLY PENDING.** A plan loss during a
  ceiling re-tap no longer forgets the calibration. Watch live: on the loss → `CALIB_LOST_HOLD` (not
  `HOLD_LOST`); NO `CALIBRATING_HEIGHT↔CALIB_LOST_HOLD` oscillation while `status` lags; on recovery the
  altitude drops off the ceiling (no more `pos_y≈-2.2` glue). Knobs: `calib_lost_recover_frames`,
  `calib_lost_bump_slow_frames` (both 6). The one-bump-max is deliberate (a 2nd nudge risks hitting walls).
- **`CALIB_VERIFY`/`ASCEND_ESCAPE`/`CALIB_TRANSLATE` (session 11) — FLEW `20260712`, NOT confirmed good.**
  The operator is dissecting this flight's log; whether the low-drone occupancy poisoning is actually solved
  is still an open question (expect follow-up questions here). Watch the real per-goal re-calibration: the
  drone should never settle low; a bad result should climb + slide 1u + retry, and occupancy should stay
  clean. The settlement gate leans on the plumbed `cap_ts` (None-guarded).
- **FLOOR detector is NEW + UNVALIDATED (session 10) — watch the first live dock closely.** Unlike
  CEILING/WALL (flight-validated), the floor collapse (`CMD_DOWN`, descending `|dy_med|`→0) has never fired
  on real footage. `dock_max_s` is the fail-safe (log + proceed to LOW_STANDOFF). If it never latches, the
  de-risk fallback is a fixed pulsed-descent count instead of flow detection (`plans/all-corners-...md`).
- **Corner tour termination relies on the fresh 2-bump (session 10), not `_excluded`** — corners ignore the
  frontier blacklist by design, so a genuinely walled-off corner still ends the tour only via two bumps on
  it. Confirm live that a truly unreachable corner retires (doesn't loop).
- **REPLAN dead-stall (item 2) — FIXED in code (session 9), live-fly pending.** Was: `goal=None && !done`
  idled REPLAN forever. Now the corner tour always yields a goal or a visible DONE, with a fail-visible
  bounded-idle backstop. (Turns are fine — session 8.) The older "heading decided only at REPLAN, no
  mid-leg re-aim" is a separate, milder concern.
- **Deferred (session-10 plan):** plan-lost-too-often investigation (SLAM choking?); a parallax-strafe
  alongside each turn. **Earlier deferred:** Scan mode; a glass-window altitude descend-probe; Phase-3
  report polish + GUI.

---

## What this project is
Assessment task: from the black-box **XLAB** Unity sim's single monocular drone feed, autonomously
map the room and report the 3D location of a target object (+ uncertainty). Phases: 1 Human Recon →
2 Autonomous Survey → 3 Localize & Report → GUI. Grading = internal consistency (metric scale and
compute efficiency NOT graded). Local on an RTX 3080 Laptop (16 GB).

## Architecture (processes over a ZMQ bus)
- **P1 `io_bridge.py`** — NDI capture + 60 Hz TCP control to Unity + keyboard. Publishes 512×288
  transport frames (:5601) + hi-res 720p (:5605). Applies the autopilot's control ONLY while autonomy
  is ON (toggle `m`; any manual key aborts).
- **P2 `perception_worker.py`** — MASt3R-SLAM every frame → `MapStore` voxel map + `GroundGrid` 2D
  free/unknown/occupied. Publishes TOPIC_POSE/MAP/PLAN/TARGET (:5603); lifts detections into the map.
  (DA-V2 depth removed in session 5.)
- **P3 `visualizer.py`** — read-only dashboard (input | top-down map + path + frontiers/goal + target).
- **P4 `object_worker.py`** — 3-stage cascade detector; publishes TOPIC_DETECTION (:5604).
- **P5 `autopilot.py`** — CPU-only flight controller (optical-flow CEILING/WALL detector + playbook
  recipes). Modes: `--dry-run`, `--mission`, `--explore` (Map mode).
- **GPU note:** SLAM and the detection cascade **cannot share the GPU** (compute contention → SLAM
  RELOC spiral). Phase-2 separates them in time (Map mode = SLAM only; a future Scan mode pauses SLAM
  to run the cascade).

## What's built
**Phase 1 (done, hardware-verified):** io_bridge + bus + dashboard; SLAM + voxel map; **target
detector** = 3-stage cascade (GroundingDINO+OWLv2 propose → DINOv2 verify → SIFT/LightGlue geom gate)
— solved a small-object, mural-cluttered task that **every single-shot/VLM engine failed** (Qwen2.5-VL,
OWLv2, dense DINOv2/SIFT/LightGlue); **3D lift + consensus** (`target_estimator`) → confident
multi-target estimate. Offline E2E confirmed.

**Phase 2 — Map-mode explorer (`autopilot.py --explore`), flies live:**
- `ground_grid.py` — 2D grid + frontier extraction from SLAM points.
- `perception` publishes **TOPIC_PLAN** (pose/heading/goal/bearing/done + forward clearance + ring);
  goal = frontier planner pick; `plan_valid=false` when SLAM not TRACKING.
- `ExploreController`: **ARM → TAKEOFF → ASCEND (two-phase) → DESCEND → CALIB_VERIFY → BASELINE_NUDGE →**
  leg loop **REPLAN → ORIENT (open-loop ≤45° turn) → ADVANCE (forward until the clearance stand-off / flow
  WALL / self-calibrating ram guard) → SETTLE**; a per-goal **CALIBRATING_HEIGHT** re-tap routes ASCEND →
  DESCEND → **CALIB_VERIFY** (state-gated judge vs the frozen flying-height baseline; a sunk result →
  **ASCEND_ESCAPE → CALIB_TRANSLATE →** re-tap, session 11); a plan loss DURING any re-tap diverts to
  **CALIB_LOST_HOLD** (survive the loss → redo the calibration on a 6-fast-frame + `status==OK` SLAM-pulse
  recovery, one DOWN bump if stuck; session 13), and 3 consecutive failed calibrations divert to
  **CALIB_ESCAPE** (ring push + hold for SLAM → retry; 3 more → STUCK; session 15); a gradual-height **TRIM**
  (session 14) fires from SETTLE/ADVANCE when `pos_y` sinks past `ceiling_y + 1.2*delta` — ring-gated PITCH-aim
  + forward climb (`trim_pitch_up=-1.0`), goal-preserving; a leg **SETTLE waits for 6 fresh post-settle SLAM
  frames** before flying (session 15); on `done` the **floor-dock postlude
  RETURN_TO_ORIGIN → DOCK_FLOOR (two-phase pulsed descent) → LOW_STANDOFF → DONE** (session 10);
  control-space **recovery** on SLAM loss; **STUCK** hold; event-driven 2-bump blacklist for unreachable
  goals. **Ram guard is self-calibrating**; the clearance stand-off is the primary wall stop. (The ORIENT
  open-loop turn works — session-8 re-fly.)
- `flight_playbook.json` + `RecipePlayer` — control recipes as data (the tunable durations).
- `fly.py` — one-command stack launcher (perception + autopilot + visualizer + io_bridge + Xlab in separate
  windows), a graceful stop-file shutdown so the autopilot flushes its replay map backdrop, then auto-compiles
  + opens the flight report. The autopilot honours `--stop-file <path>` (polled `_FileStopEvent` → clean exit).

---

## Reference — don't re-derive

### Drone control mechanic
Yaw is a **"fly toward your aim"** scheme: yaw moves an aim crosshair, forward thrust flies toward it;
a **SUSTAINED yaw hold then `c` (reset)** rotates the body (turn ANGLE = hold duration) — **confirmed
live in session 8** (the drone visibly turns). NB: the SLAM *heading* in the log lags the turn badly
(pose is ~2 Hz and monocular SLAM barely resolves pure rotation), so a real turn looks motionless in the
timeline until the drone translates — do NOT read that as "the drone didn't turn". io_bridge applies
autopilot values directly (no ramp); yaw latches until `c`. `joy_vertical` is a **DISCRETE −1/0/+1 axis**
(up/down = full thrust, can't be throttled); trigger & reverse ARE continuous 0–1. The only Unity
telemetry back is `time` — everything else is vision. Calibration: ~90° at yaw 1.0 for ~1.625 s.

### Environment & build
- Tree: `D:\EXTEND\C2_SIM\XLAB\` → `XLAB\` (read-only sim: Xlab.exe, Sample_Drone_Interface.py,
  OUTPUT\*.mp4) + `cartographer\` (our repo). One venv `cartographer\venv` (py 3.11.9,
  torch 2.5.1+cu121) — run everything from it.
- **lietorch is a PATCHED LOCAL build** (`third_party/lietorch`) — NEVER pip-install upstream.
- MASt3R-SLAM rebuild: `build_mast3r_slam.bat` then `build_mast3r_slam_step23.bat`.
- SLAM quirks (`slam_engine.py`): `os.chdir` into the SLAM repo before loading; recover the 4×4 pose
  via **Act3 on origin+unit axes, NOT `T_WC.matrix()`** (matrix() corrupts the pose under patched
  lietorch).

### Key technical facts
- **Sim protocol** (`Sample_Drone_Interface.py`): Python is the TCP **server** (127.0.0.1:65432);
  Unity connects in. 60 Hz `control_state` JSON (trigger/reverse, joy_horizontal strafe, joy_vertical
  altitude [−1 up/+1 down], yaw, pitch). Video = NDI 1280×720@30. Keys: 1=arm, w/s, a/d strafe,
  e/f up/down, arrows yaw/pitch, b=land, c=reset attitude, space=full-res capture, g=detect.
- **Resolution:** transport 512×288 (16:9, never squash); the cascade runs on the hi-res (:5605) stream.
- **Ray lift:** world ray = `pose[:3,:3] @ ray_cam`; center ray ≈ [0,0,1]; raycast skip 0.25 u.
- **World frame is +Y DOWN** (camera convention) — a sinking drone has an INCREASING `pos_y`.
- **Recording is ~58 fps, not 30** — durations must come from keystroke `mono_ts`, never frame counts.

### Run procedure
1. Designate target once: `venv\Scripts\python.exe make_target.py` → `target.yaml`.
2. **One command: `python fly.py`** — spawns perception `--no-display` + autopilot
   `--explore --log --stop-file` + visualizer + io_bridge (separate windows) + `Xlab.exe`; press `m` on
   io_bridge to hand over; press ENTER in the launcher to stop CLEANLY (drops the stop-file so the autopilot
   flushes its replay map backdrop) → auto-compiles + opens the report. Manual equivalent: `Xlab.exe` →
   `python io_bridge.py` → `python perception_worker.py --no-display` → `python visualizer.py` →
   `python autopilot.py --explore --log`; press `m` to hand over.
3. Offline self-tests: `autopilot.py --self-test`, `flow_contact_detector.py --self-test`,
   `frontier_planner.py --self-test`, `ground_grid.py --self-test`, `perception_worker.py --self-test`.
   Offline SLAM+map E2E: `perception_worker.py --video OUTPUT\flight_<ts>.mp4 --no-display`.
4. Diagnostics: `--log` → `OUTPUT/diag/<ts>_autopilot.{log,csv}` + `<ts>_timeline.{jsonl,html}`
   (open the HTML in a browser).

---

## Standing rules (every change)
- **NO SILENT FALLBACKS:** fail-fast OR set a visible/logged/HUD flag; any fallback approved first.
- **NO manual-flight data leakage:** every autonomous limit is a LIVE self-calibrating signal;
  platform/signal characteristics (flow signatures, control magnitudes, turn calibration, the ~1 s
  healthy-SLAM compute time) are legitimate — this room's geometry is not.
- Image integrity (no undisclosed downscaling); start multi-step work with a TaskCreate list;
  **never commit unless asked**; self-test offline before live.

## Milestones
Phase 1: models on GPU ✅ · io_bridge + bus ✅ · SLAM + map + dashboard ✅ · target cascade + 3D
localize ✅. Phase 2: mission runner flew live ✅ · Map-mode explorer flies (SLAM-safe turns,
clearance stand-off, control-space recovery, event-driven 2-bump blacklist, two-phase ascent,
self-calibrating ram guard) ✅ · rich flight-replay debugger ✅ · glass-corner blacklist escape (Bug
A+B) + frontier clearance buffer 🛠️ built, flew in the session-8 flights. **Session 8: confirmed turns
work (the "no-op" was a stale-heading logging artifact) + made the flight log trustworthy (committed goal
+ data staleness) + `[SLAM_TRACKER]` telemetry + reach/clearance/SLAM-settle eases + a plan-lost grey
marker; a clean flight (`20260708_195009`) followed.** **Session 9: killed the REPLAN dead-stall with a
bbox diagonal sweep (`ground_grid.sweep_corner` + reworked `frontier_planner.select` + autopilot
bounded-idle backstop + visible EXPLORE-COMPLETE DONE), moved `[SLAM_TRACKER]` into the replay HTML
(teal), and built per-goal height re-calibration (item 1 — `CALIBRATING_HEIGHT` on a goal change, low-
object-tap reject) 🛠️ all built, self-test-green, live-fly pending.** **Session 10: generalized the single
sweep into an ALL-CORNERS TOUR (`ground_grid.bbox_corners` + multi-corner `frontier_planner.select`, corners
ignore the blacklist / retired by a fresh 2-bump) + a post-mission floor-dock postlude (RETURN_TO_ORIGIN →
DOCK_FLOOR two-phase pulsed descent → LOW_STANDOFF → DONE) with a new flow FLOOR detector 🛠️ all built,
self-test-green, live-fly pending.** **Session 11: BUILT + FLEW (`20260712`) the three test-flight asks —
a state-gated height-calibration fix (frozen-during-calib `_mapping_altitude_history` baseline + post-descend
`CALIB_VERIFY` settlement gate on the plumbed `cap_ts` → `ASCEND_ESCAPE`/`CALIB_TRANSLATE` retry, retiring
`CALIB_NUDGE`), paired `slam_start`/`slam_finish` SLAM logging in the replay HTML (capture-wall START / log-
wall FINISH, timestamp bug found + fixed on this flight), and a `t_wall`/`t_mono` unify; plus a one-command
`fly.py` launcher with a graceful stop-file shutdown 🛠️ all built + all six module self-tests green + flew;
**height calibration NOT yet confirmed — operator dissecting the flight log.** Plan
`plans/height-calib-state-gate-and-slam-debug.md`.** **Session 13: diagnosed `20260713_163055` — a brief
PLAN-LOST during a per-goal ceiling re-tap made the drone forget it was calibrating, skip the DESCEND, and stay
glued to the ceiling (`pos_y≈-2.2`) for the whole flight. Built a dedicated `CALIB_LOST_HOLD` state (+
`_calib_interrupted` flag) that survives the loss, redoes the calibration on a 6-fast-frame + `status==OK`
SLAM-pulse recovery, bumps DOWN once if stuck, and gates the redo exit on `status==OK` to beat the
level-triggered flicker (avoids a 1-tick `CALIBRATING_HEIGHT↔CALIB_LOST_HOLD` oscillation) 🛠️ all built + all
six module self-tests green, live-fly pending. Plan `plans/crystalline-swimming-floyd.md`.**
