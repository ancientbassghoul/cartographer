# Cartographer — Progress & Resume Handoff

_Last updated **2026-07-13** (session 13 **BUILT + all self-tests green; live-fly PENDING**). Resume from THIS
file. **Session 13** (`plans/crystalline-swimming-floyd.md`): on flight `20260713_163055` a per-goal ceiling
re-tap started, then a brief PLAN-LOST hit mid-ASCEND — the drone **forgot it was calibrating**, dropped into
normal recovery (`HOLD_LOST→SLAM_HOLD→SETTLE→REPLAN`), skipped the DESCEND, and **stayed glued to the ceiling
(`pos_y≈-2.2`) for the rest of the flight**. Fixed with a dedicated `CALIB_LOST_HOLD` state that survives the
loss: hold, watch the SLAM frame pulse (`slam_ms`), **redo** the calibration once ≥6 fresh frames <1000 ms AND
`status==OK` (the OK-gate beats the level-triggered status flicker that would otherwise 1-tick-oscillate), and
**bump DOWN once (max)** if SLAM stays choked (6 slow frames → wake SLAM) OR solves fast but the plan won't lock.
All self-tests green; **live-fly PENDING**. Session-12 (below) also still awaits its live re-fly.
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

_**NEXT = LIVE RE-FLY** — one flight now confirms BOTH session 13 and session 12 (both BUILT + all self-tests
green). Run `python fly.py`, press `m` to hand over._

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
- **Calibration survives a plan loss — BUILT (session 13), self-test green, LIVE-FLY PENDING**
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
  recovery, one DOWN bump if stuck; session 13); on `done` the **floor-dock postlude
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
