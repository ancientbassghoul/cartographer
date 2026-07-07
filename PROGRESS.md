# Cartographer — Progress & Resume Handoff

_Last updated 2026-07-06 (session 4: REPLACED the time-based unreachable-goal watchdog with an EVENT-DRIVEN
2-BUMP blacklist + added reverse (BACKWALL) contact DETECTION-ONLY. All self-tests + an E2E ZMQ pulse test
PASS; NOT yet re-flown. NEXT UP = re-fly to confirm the glass goals go red at their 2nd bump, THEN build the
reverse-recovery reaction fresh — see the ⏭ block below.) Resume from THIS file._

**✅ SESSION 4 (2026-07-06) — EVENT-DRIVEN 2-BUMP BLACKLIST (replaces the broken time watchdog) + reverse
BACKWALL detection-only (self-test + E2E-verified, NOT re-flown).** Debugged flight `20260706_165548`: the
drone sat 9 min at the glass wall never blacklisting the unreachable beyond-glass goals, then froze in a
reverse-into-wall loop.
- **ROOT CAUSE (corrected):** the old `frontier_planner._watchdog` was a time-accumulator GATED on
  `slam_ms < slam_slow_ms`. In the pocket SLAM ran hot (1800–4200 ms) yet the drone kept flying on valid
  poses, so the accrual clock stayed frozen and never fired (goal was stationary at `[-2.1,-4.8]` for 143 s
  with zero accrual). **Lesson: time-accumulation proxies gated on SLAM health go BLIND exactly in the heavy
  glass/wall pockets.** Earlier "goal-hopping" / "confinement-watchdog" theories were both wrong (verified
  against the timeline).
- **FIX — event-driven 2-bump rule.** The autopilot reports each DISCRETE advance-blocked stop as a "bump"
  pulse (`TOPIC_AUTOPILOT_EVENT`, new); TWO bumps on the SAME goal region (within `goal_assoc_dist`) ⇒
  `FrontierPlanner.note_wall_hit` PERMANENTLY blacklists it + drops the commitment. A bump on a different
  goal resets the counter. Immune to SLAM-clock health (counts hard physical stops).
  - **Bump source = ALL THREE stops** (flow-WALL + ram-guard + standoff): the flow-WALL detector fired only
    3× in the whole flight (ram-guard/standoff were ~40×), so flow-only would blacklist just 1 of 4 dead
    goals. With all three, each dead goal dies at its 2nd encounter (would've been ~17:00:33 / 17:02:09 /
    17:04:13 / 17:05:28 instead of never).
  - **Kinematic bump-latch** (autopilot `_bump_armed`/`_last_bump_anchor`): one continuous contact = one
    bump. Disarms on a bump; RE-ARMS on a published reverse command OR displacement > `goal_reach_dist`
    (0.4) from the anchor (Option 2). Verified: the 4 legit pairs travelled 0.58–1.06 u (re-arm) while the
    standoff STUTTERS sat at 0 displacement (stay disarmed). SLAM-freeze-safe (frozen pose → no re-arm).
  - **Wiring:** `frame_bus.TOPIC_AUTOPILOT_EVENT`; autopilot publishes `{bump_goal,seq}` on the control port
    (5606) after `step()`; perception SUBs it (`apevent_sub`), dedups by seq, feeds `note_wall_hit`
    UNCONDITIONALLY (never SLAM-gated). Removed `_watchdog` + `goal_stagnation_s` (net −1 knob; threshold=2
    hardcoded). Removed the now/slam_ms watchdog plumbing from `select()`/`_plan_payload`.
- **Reverse BACKWALL detection (DETECTION-ONLY this session).** `flow_contact_detector` gained `CMD_BACK`
  (`signal = -expansion`, the contraction-collapse mirror of WALL; `_ref_back` persists like `_ref_up`).
  The autopilot now derives the detector command from the ACTUAL published control vector (`_detector_command`:
  reverse→BACK / trigger→FWD / joy_vertical<0→UP), replacing the static `_EXPLORE_STATE_CMD` map, so BACKWALL
  arms across every reverse state (REWIND/REVERSE_PROBE/BACKOFF/backward-parallax). On a BACKWALL contact it
  LOGS `BACKWALL contact` (one line per onset) — **no control reaction yet.**
- **Tests:** `frontier_planner`/`flow_contact_detector`/`autopilot`/`ground_grid --self-test` ALL PASS (new:
  2-bump same-region→permanent + goal-change reset; BACKWALL collapse; `_detector_command`; the latch
  one-pulse/stutter/re-arm). Plus an in-process **E2E ZMQ pulse test** (bump#1→arm, dup-seq→ignored,
  bump#2→permanent-blacklist+drop-commitment→reselect drops the goal). Config: −`goal_stagnation_s`.
- **⏭ NEXT — (1) RE-FLY** `autopilot.py --explore --log`: confirm each beyond-glass goal goes RED (permanent)
  at its **2nd** advance-blocked stop (~2 encounters, not 9 min), the planner repositions OUT of the pocket,
  the log shows `BLACKLIST PERMANENT (2 advance-blocked bumps)`, and a reverse into a wall LOGS `BACKWALL
  contact`. **(2) THEN build the reverse-bump REACTION fresh** (deferred, user's call): on a BACKWALL contact
  → give up → clear history → push **FORWARD** (camera view; confirmed there is NO left/right strafe-recovery
  code — recovery only uses `trigger`/`reverse`) → recovery spin; AND fix the REWIND-restart bug so the spin
  is reachable + STUCK terminates the reverse loop (`_step_stale` rebuilds a fresh rewind on every
  HOLD_LOST↔PLAN-STALE flap, resetting `_fallback_attempts`, so the fallback sweep is never reached). Plan
  file: `read-cartographer-progress-md-...` (Part 3, DEFERRED).

**✅ SESSION 3 (2026-07-06) — F8 FLIGHT-REPLAY DEBUG TOOL BUILT (self-test-verified; awaiting the first real
flight to feed it).** Two audiences, ONE data source, exactly as speced.
- **Part A — structured JSONL timeline.** `AutopilotLog` (`autopilot.py`) now also opens
  `OUTPUT/diag/<ts>_timeline.jsonl` on `--log` and has a no-op-when-disabled `timeline(record)` sink.
  `run_explore` emits ONE compact record per explore step (`_timeline_step_record`: t_wall/t_mono/rec_frame,
  state/event/status, pos/heading/pos_y/slam_ms/fwd_clear/top_clear/goal/bearing_err + `goals` tagged
  active/blacklist_soft/blacklist_permanent by zipping `blacklist`×`blacklist_permanent`). **Map = a single
  static backdrop (user's call):** we do NOT replay the map growing — only the newest GroundGrid summary is
  kept and emitted ONCE at shutdown (`_downsample_map`, flat int `cls` grid ≤2500 cells ~5 KB, `t_mono=0`),
  so the drone + goal states animate over the final room outline. All `--log`-gated (`if log:`); zero
  flight-behavior change.
- **Part B — `flight_replay.py` (new, stdlib only).** `python flight_replay.py <ts>_timeline.jsonl [-o …]
  [--open] [--slam-slow-ms 1000]` → a SELF-CONTAINED animated HTML (Canvas + `<input range>` scrubber +
  play/pause/speed; side panel = event log auto-scrolled to the cursor + slam_ms readout & sparkline
  green<1000/red≥). Top-down scene: occupancy outline / fading path trail / drone dot+heading arrow / goals
  gold(active, ringed)-orange(soft)-red-✕(permanent), all updating as you scrub. **Direct O(1) array indexing**
  (slider index → `STEPS[i]`, no binary search) and an **explicit Y-flip normalization** helper (`fit()`+`P()`,
  equal-aspect) — both per the user's critique.
- **Tests:** `flight_replay.py --self-test` (synth timeline → HTML; asserts record count preserved, step/map
  split, goal state timeline active→soft→permanent, slam spike + scene code embedded) — ALL PASS.
  `autopilot.py --self-test` — ALL PASS incl. a new **F8 timeline** case (disabled sink is a no-op; the record/
  map builders produce the right shape). Sample to eyeball: `OUTPUT/diag/SAMPLE_timeline.{jsonl,html}` (open
  the HTML in a browser).
- **⏭ NEXT — re-fly F4–F7 with `--log`, then open `<ts>_timeline.html` (and `Read` the JSONL) to debug.**

**✅ SESSION 2 (2026-07-06) — CORRECTED GLASS MODEL + 3 flight fixes (self-tested, NOT yet re-flown).**
A live flight showed the session-1 "glass-stuck" watchdog was built on a WRONG glass model. **User's
correction:** the monocular camera looks THROUGH clear glass and tracks features on the far side, so **SLAM
stays healthy + FAST and the depth/forward-clearance ray reads clear**; the drone hits the invisible
collider, bounces, and pushes again — an **invisible treadmill**. So a watchdog that required SLAM to CHOKE +
the path BLOCKED was exactly backwards. Reworked into ONE robust detector, plus fixes for takeoff-startup
spins, the bump-up ceiling smash, and opaque-wall ramming.
1. **F4 rewrite — DISTANCE-STAGNATION goal watchdog** (`frontier_planner._watchdog`, replaces the
   slam_alive/blocked stuck-clock). A committed goal whose best-ever Euclidean distance fails to improve by
   `goal_progress_eps` for **`goal_stagnation_s` (60 s) of SLAM-HEALTHY time** is **PERMANENTLY blacklisted**
   (+ reposition). No aim/clearance/thrust gates. It ticks only on VALID (TRACKING) frames (perception skips
   `select()` otherwise) so a full SLAM loss pauses it; a `slam_ms` choke also pauses (unstable pose). ONE
   timer catches BOTH the glass treadmill (pos plateaus at the collider, SLAM healthy) AND the opaque-wall
   ram-and-recommit loop (pos plateaus at the wall, accruing on the healthy stretches). `_blacklist_goal`
   gained a `permanent=` arg. `select()` trimmed to `(…, now, slam_ms)` — the old `forward_clearance`
   arg + the considered `controls.trigger` plumbing are gone.
2. **F5 — GENTLE + ceiling-safe bump-up** (`autopilot.py`, fixes the session-1 F3 ceiling smash). The old
   bump injected `joy_vertical=-1` (full up) EVERY tick → sustained full-thrust climb into the ceiling. Now a
   new **`BUMP` state** plays a short bounded UP-**pulse** (`bump_pulse_s` 0.2 s), raises `target_altitude_y`
   by `bump_step`, then rests (SETTLE) and re-checks the stand-off; capped by `bump_max_per_leg` (lowered
   0.6→0.3). `BUMP` maps to `CMD_UP` so the flow **CEILING** detector is armed and aborts the climb on
   contact; `top_clear` dropping near the ceiling also ends it.
3. **F6 — NO-SPIN startup** (`autopilot.py`). The prelude finishes on the flow ceiling detector while SLAM is
   still initializing → the first `PLAN-STALE` with an empty history launched the full 16-attempt 360° sweep
   (~50 s spinning). Added `_ever_tracked` (set on the first valid plan); an empty-history `PLAN-STALE` while
   `not _ever_tracked` now enters a **`WARMUP`** hold (waits for SLAM) instead of the blind sweep. A real
   mid-flight loss (history wiped by a wall hit, already tracked) still goes to the fallback.
4. **F7 — RAM GUARD** (`autopilot.py`). Opaque-wall loop: the forward-clearance ray never stopped the drone
   (it's `None` when SLAM flickers, and rises above the wall's voxels as the drone climbs it), the flow WALL
   needs a looming COLLAPSE that never comes on a slow ram, so the drone rammed → SLAM died → recovered →
   re-rammed for ~2 min. New pos-space guard in `ADVANCE`: if forward-commanded but the SLAM pos hasn't
   advanced toward the goal by `ram_progress_eps` for `ram_stall_s` (3 s), STOP the leg (→SETTLE→REPLAN).
   Cuts each ram; the F4 60 s stagnation blacklist then breaks the re-commit loop.
- **Diagnoses (logs):** `20260706_124600` — startup `PLAN-STALE`→16-spin sweep (F6). `20260706_124858` —
  opaque-wall ram loop on goal `[4.1569,-4.1532]` (F7 + F4). Takeoff violence: **no fix possible** —
  `joy_vertical` is a DISCRETE ±1 axis (full thrust), takeoff is a measured 3.25 s hold (3.1 s floor), so it
  can't be throttled; only the hold duration is tunable and it's near-minimal.
- **Config (`autonomy.explore`):** REMOVED `goal_stall_s`/`goal_stuck_s`/`goal_stall_aim_deg`/
  `goal_stall_clearance`/`goal_stall_arm_dist`; ADDED `goal_stagnation_s` (60), `bump_pulse_s` (0.2),
  `ram_stall_s` (3.0), `ram_progress_eps` (0.15); `bump_step` 0.05→0.1, `bump_max_per_leg` 0.6→0.3.
  (Session-1 knobs `slam_stepback_*`, `depth_bump_up`, `top_clear_thresh` stay.) All general params.
- **Tests:** `frontier_planner.py --self-test` (rewrote the watchdog cases: stagnation-blacklists-at-60s,
  SLAM-choke-pauses, ram/recover-plateau-still-blacklists), `autopilot.py --self-test` (rewrote bump-up for
  BUMP, added no-spin-startup + ram-guard), `ground_grid.py --self-test` — **ALL PASS**; perception compiles.
- **⏭ NEXT (deferred, user's call) — F8 FLIGHT-REPLAY DEBUG TOOL.** Build in a FRESH session, THEN re-fly
  these fixes and debug with it. (1) a structured `<ts>_timeline.jsonl` written by `autopilot.py run_explore`
  on `--log` (per-step pose/heading/slam_ms/clearance/top_clear/goal/bearing + goals-with-states +
  event/state + periodic bbox) — the log *I* read instead of 2 000-line text; (2) `flight_replay.py` → a
  **self-contained HTML** (Canvas + timeline scrubber + play/pause; side panel with the event log +
  SLAM-ms sparkline) rendering room bbox / path / goals-by-state / drone. See the plan file
  `read-cartographer-progress-md-...` F8 section.
- **TO DO — live re-fly (after F8):** startup holds for SLAM (no spin); a low wall gets a gentle stepped bump
  (no ceiling smash); a goal behind a solid wall stops ramming within ~3 s and goes **RED (permanent)** after
  ~60 s, then repositions; a glass goal likewise goes red. Tune `goal_stagnation_s`/`ram_stall_s`/`bump_*` live.

**✅ DONE — goal-selection PING-PONG fixed (self-tested, NOT yet re-flown).** The drone oscillated between
two unreachable goals: A blacklisted → start toward B → **moving away silently re-whitelisted A** → turn back
→ wedge → repeat. Root cause = the blacklist was **position-conditioned** (`_blacklisted(center, pos)`: a
dead goal was only excluded while the drone stayed within `goal_vantage_radius` of the give-up spot), and the
`goal_blacklist_permanent_after` guard never fired because the give-up vantage drifted each time.
**Fix (all in `frontier_planner.py`, wired through perception):** the blacklist is now
**position-UNconditioned + round-based**:
- Each entry `{goal, best_ever, permanent, active}`. `_excluded(center)` (no pos) excludes a goal that is
  `permanent` OR `active` — a blacklisted goal STAYS excluded; moving no longer re-whitelists it.
- **Whitelist only when "been over all goals"** (every live frontier excluded) → the planner REPOSITIONS to
  the farthest free corner (existing `farthest_free`/verify path) and, **on arrival**, `_whitelist_round()`
  clears the round's soft exclusions so the goals get ONE retry from a fresh vantage.
- **Convergence:** a goal re-blacklisted in a later round WITHOUT ever getting closer (`best_ever` never
  improved by `goal_progress_eps`) is promoted to **PERMANENT** (never whitelisted) → each dead goal gets
  ≤2 real attempts then drops out for good. (User pick: "re-dead with no progress → permanent".)
- **Reachable corner ("almost in the corner"):** `ground_grid.farthest_free(pos, margin=reposition_inset)`
  pulls the target inward so the drone can actually reach it (the raw farthest cell sits against the wall,
  inside the stand-off shell — the "goal stuck in the corner, never reached" the user saw). `reposition_inset`
  (0.8) is clamped to the reachable band `[stop_clearance_dist, stop_clearance_dist+goal_reach_dist]` with a
  visible warning in perception (NO SILENT FALLBACK).
- Telemetry: TOPIC_PLAN now also carries `blacklist_permanent` (bool list); the visualizer rings PERMANENT
  dead goals with a diamond. Retired knobs `goal_vantage_radius`/`goal_blacklist_permanent_after`; added
  `reposition_inset`. Tests: `frontier_planner.py --self-test` ALL PASS (17, incl. no-re-whitelist-by-moving,
  reposition-then-whitelist-on-arrival, cross-round permanent promotion, cross-round-progress-stays-soft);
  `ground_grid.py --self-test` + `autopilot.py --self-test` ALL PASS.
- **DEFERRED (user pick):** the glass-window altitude descend-probe (Part 2 — descend in small steps to hunt
  a lower opening; clear-but-can't-advance ⇒ permanent-blacklist goal + XZ). It is control-space and needs a
  new autopilot→perception channel; revisit later.
- **TO DO — live re-fly:** confirm once a goal goes red it STAYS red (no turn-back); all-dead → the drone
  flies to a corner it can REACH, whitelists, retries; a re-wedged no-progress goal goes permanent (diamond)
  and drops out; the log no longer shows the A→B→A oscillation.

**⚠️⚠️ OPEN — DRONE FLIES STRAIGHT PAST THE GOAL, WON'T TURN (diagnosed, NOT fixed).** Flight
`OUTPUT/diag/20260705_174456_autopilot.log` + `DEBUG_IMAGES/when will you turn to the goal.png`: the drone
takes off, flies its takeoff heading, and sails past an off-axis goal — the path-vs-goal angle reaches ~45°
and STEEPENS as it passes, yet it never turns. Root cause (CONFIRMED in code, NOT a quantizer dead-zone red
herring):
- **The heading is re-decided ONLY at REPLAN, and REPLAN only runs at LEG-END. There is ZERO re-aiming
  mid-leg.** `bearing_err` → `_quantize_turn` → turn is computed once, at REPLAN (`autopilot.py:921`).
  `ADVANCE` (`autopilot.py:974-1016`) flies straight OPEN-LOOP and never re-reads the bearing; it ends only
  on one of four leg-end events: forward-clearance stand-off (wall mapped ahead), flow wall-contact,
  goal-reached (< `goal_reach_dist` 0.4), or the `leg_max_s` **20 s** timeout.
- **In open space** (no wall ahead, never within 0.4 of the goal) NONE of the first three fire, so the leg
  runs the full **20 s straight** — the drone drifts 45°+ off the goal with no replan, no turn. It only
  re-aims when the 20 s expires (or it finally hits a wall).
- **A SLAM choke does NOT cause a replan.** The settle gate (`autopilot.py:978`) enters `SLAM_HOLD` and
  RESUMES the SAME leg (same `leg_goal`/heading); STALE/LOST → HOLD_LOST/rewind recovery. Neither re-aims.
- (Secondary: even at leg-end, `_quantize_turn = round(be/45)*45` ignores |be| < 22.5°, so small errors
  never turn either — but the PRIMARY problem here is the no-mid-leg-reaiming above.)
- **NOT yet fixed / no approach chosen.** The fix direction (mid-leg re-aim? shorter leg cap? close-loop
  heading correction / strafe?) was NOT agreed — do not assume. Get the user's decision before building.

**⚠️ GLASS-WINDOW blacklist — v1 flew, TWO follow-up bugs FIXED (self-tested, awaiting a live re-fly).** The
unreachable-goal blacklist (`## DONE — unreachable-goal blacklist`) flew and revealed two problems, now fixed:
1. **Blacklist fired FAR too early** — red X's before the drone even flew, and goals turned red mid-approach
   (the user's read: when the drone stops to let SLAM cool down). Cause: the watchdog accrued **wall-clock**
   time on merely *aimed + not-closing*, counting the ground prelude and SLAM-settle/HOLD pauses. **Fix:**
   accrue stall ONLY while GENUINELY WEDGED — armed (moved from start) + aimed + **blocked ahead** (small
   `forward_clearance_dist`) + **SLAM alive** (`slam_ms < slam_slow_ms`) + not-closing; an accumulator that
   pauses when any gate fails and resets on progress. All gate signals are already on TOPIC_PLAN (no new
   channel). See `## DONE — unreachable-goal blacklist`.
2. **Parallax push too weak, esp. FORWARD.** The forward push reused the 0.1 ADVANCE crawl (`forward_throttle`)
   → never covered `parallax_push_dist` before the time cap → ~no parallax. **Fix:** a dedicated brisk
   `parallax_push_throttle` (0.4) for the forward push, decoupled from the cautious ADVANCE throttle; backward
   kept on `reverse_throttle`; `parallax_push_s` 1.2→2.0 so DISTANCE ends the push.

**Test flight 2026-07-05 (`OUTPUT/diag/20260705_094830_autopilot.log`) — much better; two critical bugs +
one minor, addressed below and then re-flown 07-05 with GOOD PROGRESS (the rewind fix + SLAM settle gate
behaved as intended):**
- **Rewind spun in place (bug 1, FIXED).** Every `REWIND` command was a turn — translations were missing
  from `command_history`. Root cause was the `duration > 0.1` drop in `_log_move` (micro-short ADVANCE legs
  of a loss-spiral never logged). **Dropped the guard** (translations always log now); kept the wall-contact
  history-wipe. Added a rewind-composition diagnostic (`[history: N turns, M translations / S s]`).
- **Heading drift + constant PLAN-LOST after turns (bug 2, MITIGATED via a SLAM frame-timing settle gate).**
  The drone flew on shaky poses computed while SLAM was choking right after a turn (the ~45° Visualizer gap).
  New rule: SLAM is "stable" only after **>2 consecutive FRESH frames each built <1000 ms**; while
  translating / just after a turn / on recovering, **HOLD (`SLAM_HOLD`) until settled, then fly**. `slam_ms`
  now rides on TOPIC_PLAN. Did NOT chase the indexing/order bug yet (deferred, per the user). Knobs:
  `slam_slow_ms` (1000, a COMPUTE characteristic — not room geometry), `slam_settle_frames` (3).
- **Minor depth "too-low" cue — PARKED** (see "## Second-priority / future fixes").

**Earlier this session (07-04/05, self-test-verified, NOT yet all flown together):** forward & reverse
throttle knobs, SLAM altitude lock, ray-guided parallax scouting, a new frontier goal planner +
done-verification (`frontier_planner.py`), a **control-space SLAM-loss recovery** (hold-on-LOST →
command-rewind on STALE → parallax+≤45 fallback), and the **fallback-sweep tweak** (unidirectional +45°
sweep, fwd/back-alternating retreat, `fallback_max_attempts` 16). See "## BUILT THIS SESSION".
**NEXT = live re-fly to validate the unreachable-goal blacklist** (the glass-window loop; see
`## DONE — unreachable-goal blacklist`). The rewind fix + SLAM settle gate flew with good progress.

**Where we are:** Phase-1 (manual map + target localize) built & verified. Phase-2 autonomous
**Map-mode explorer** (`autopilot.py --explore`) flies live and **stops before ramming walls** via the
raycast forward-clearance stand-off (`stop_clearance_dist: 0.6`, flown 06-30 — saved itself repeatedly).
**This session (07-04/05) built + self-test-verified a big batch, NOT yet all flown together:** forward &
reverse throttle knobs, SLAM altitude lock, ray-guided parallax scouting, a new frontier goal
planner + done-verification (`frontier_planner.py`), and a **control-space SLAM-loss recovery**
(hold-on-LOST → command-rewind on STALE → parallax+≤45 fallback). See "## BUILT THIS SESSION".
**DONE 07-05: the fallback-sweep tweak** — the FALLBACK turn is now a UNIDIRECTIONAL +45° sweep (drop the
±45 wiggle; 16 attempts = a full >360° RELOC re-expose) and the RETREAT alternates fwd/back (seeded on
attempt 0 by the roomier body axis from `_last_ring`); `fallback_max_attempts` 4 → 16. `autopilot.py
--self-test` ALL PASS (case-d now asserts both a fwd `trigger>0` and a back `reverse>0` retreat, turn always
`yaw>0` never `<0`, STUCK reached). **NEXT = the full live flight (checklist under "## BUILT THIS SESSION").**

## DONE — unreachable-goal blacklist (progress-stall) — the glass-window loop
**⚠️ PARTLY SUPERSEDED (2026-07-05):** the WEDGED watchdog + gates below still stand, but the blacklist is no
longer POSITION-CONDITIONED (that caused the ping-pong — see the "goal-selection PING-PONG fixed" block at the
top). It is now permanent + round-based with a reposition-then-whitelist retry and cross-round no-progress
promotion. Ignore the "position-conditioned / vantage / permanent-after" details in this section; keep the
watchdog-gate description.

**Problem (latest flight):** a goal behind a GLASS WINDOW is physically unreachable (no path planner — the
drone flies a STRAIGHT line to the goal bearing). Its frontier is never consumed (SLAM never sees behind the
glass → the FREE↔UNKNOWN seam persists), and `FrontierPlanner`'s commitment re-hands the same goal every
replan → an endless `REPLAN→ORIENT→ADVANCE→standoff→SETTLE→REPLAN` loop. Generalizes to any goal behind a
wall / around a corner.

**Why not the first idea (a REPLAN repeat-counter):** REPLAN fires many times while HEALTHILY approaching a
far goal — once per ≤45° parallax-scout step (`autopilot.py:932-935`, drone ~stationary, same goal) and once
per `leg_max_s` timeout (`autopilot.py:1012`). A bare repeat-count would abandon good far goals. The fix is
the SAME counter gated on PROGRESS.

**Fix (all in `frontier_planner.py`, wired through perception):** a per-committed-goal WEDGED watchdog.
It tracks the BEST (closest) distance achieved toward the goal and accrues a stall accumulator ONLY while
the drone is GENUINELY WEDGED — **armed** (moved ≥ `goal_stall_arm_dist` from its start, so the ground
prelude is skipped) + **aimed** (`|bearing_err| ≤ goal_stall_aim_deg`, so a multi-step turn isn't mistaken
for wedged) + **blocked ahead** (`forward_clearance_dist ≤ goal_stall_clearance`) + **SLAM alive**
(`slam_ms < slam_slow_ms`, so a SLAM-settle/HOLD cooldown doesn't count) + **not closing**. The accumulator
PAUSES when any gate fails and RESETS on real progress; at `goal_stall_s` of wedged time the goal is declared
UNREACHABLE and BLACKLISTED and the planner reselects. Healthy far goals keep closing → never blacklisted.
(v1 used a wall-clock timer on aimed+not-closing → fired on the ground prelude and during SLAM cooldowns;
the five gates fixed that. All gate signals were already on TOPIC_PLAN — no new channel.)
- **Position-conditioned** (per the user): a blacklist entry stores the give-up VANTAGE; the region is
  excluded only while the drone is within `goal_vantage_radius` of that spot, so a different angle / a newly
  opened route can retry it. Re-blacklisting one region from ~the same vantage `goal_blacklist_permanent_after`
  times promotes it to permanent (stops two dead goals ping-ponging the drone).
- **All-frontiers-dead-from-here** routes into the existing done-verification path (fly to
  `farthest_free`) — which doubles as a REPOSITION: moving off the give-up spot re-enables the blacklist so
  those frontiers become reachable again on arrival. Perception now computes `farthest_free` on
  `not planner.any_reachable(...)` (was `not frontiers`).
- **NO SILENT FALLBACK:** perception logs `planner: goal […] UNREACHABLE … -> BLACKLIST from vantage […]`;
  TOPIC_PLAN carries `n_blacklisted` + `blacklist` points; the visualizer draws blacklisted goals as a red
  tilted-cross and shows `blacklist=N` on the HUD.
- **No-leakage-safe:** blacklist POINTS are computed LIVE from the drone's own failure to progress; the
  thresholds are general map-scale params (`goal_blacklist_radius` defaults to `goal_assoc_dist`;
  `goal_progress_eps` ≈ ½·`goal_reach_dist`) + a time window — none encodes this room's geometry.
- **Config** (`config.yaml autonomy.explore`): `goal_stall_s` (6.0), `goal_progress_eps` (0.2),
  `goal_stall_aim_deg` (45), `goal_stall_clearance` (0.8), `goal_stall_arm_dist` (0.5), `goal_blacklist_radius`
  (1.0), `goal_vantage_radius` (1.0), `goal_blacklist_permanent_after` (3); reuses `slam_slow_ms`. Signature:
  `FrontierPlanner.select(..., now=None, forward_clearance=None, slam_ms=None)` — perception passes
  `payload["forward_clearance_dist"]` + `slam_ms`.
- **Tests:** `frontier_planner.py --self-test` ALL PASS (15) — healthy far goal (closing, gates open) never
  blacklists; wedged goal blacklists at `stall_s`; aim-gate ignores turning; **each of not-armed / not-blocked
  / SLAM-slow alone SUPPRESSES the blacklist**; position-conditioned retry-from-far / excluded-from-near;
  permanent promotion; all-dead-from-here → reposition.
- **Parallax push (paired fix):** the FORWARD push now uses `parallax_push_throttle` (0.4, `autopilot.py`
  PARALLAX_PUSH), decoupled from the 0.1 ADVANCE crawl; backward stays on `reverse_throttle`; `parallax_push_s`
  2.0 so DISTANCE ends the push. `autopilot.py --self-test` PARALLAX-SCOUT asserts the forward push commands
  `parallax_push_throttle`.
- **TO DO — live re-fly:** confirm NO red X's before takeoff or during SLAM-cooldown pauses; a healthy goal
  being approached stays golden; the glass loop STILL blacklists (aimed + standoff + SLAM alive) then picks a
  DIFFERENT goal; the forward parallax push visibly moves the drone (log `parallax forward push done (dist)`,
  not `(timer)`). Tune `parallax_push_throttle`/`parallax_push_dist` + `goal_stall_clearance`/`goal_stall_s` live.

## What this project is
Assessment task: from the black-box **XLAB** Unity sim's single monocular drone feed, autonomously
**map the room and report the 3D location of a target object** (+ uncertainty). Phases: 1 Human Recon →
2 Autonomous Survey → 3 Localize & Report → GUI. Grading = internal consistency (metric scale NOT
required; compute efficiency NOT graded). All local on an RTX 3080 Laptop (16 GB).

## Architecture (processes over a ZMQ bus)
- **P1 `io_bridge.py`** — NDI capture + 60 Hz TCP control to Unity + keyboard. Publishes 512×288
  transport frames (:5601) + hi-res 720p (:5605). Applies the autopilot's `TOPIC_CONTROL` ONLY while
  autonomy is ON (toggle `m`; any manual flight key aborts; a stale command is zeroed).
- **P2 `perception_worker.py`** — MASt3R-SLAM (every frame) + DA-V2 depth (throttled) in ONE CUDA
  context → `MapStore` (voxel map) + `GroundGrid` (2D free/unknown/occupied). Publishes
  TOPIC_POSE/DEPTH/MAP/PLAN/TARGET (:5603); lifts detections into the map.
- **P3 `visualizer.py`** — read-only dashboard (input | depth | top-down map + path + frontiers/goal + target).
- **P4 `object_worker.py`** — 3-stage cascade detector; publishes TOPIC_DETECTION (:5604).
- **P5 `autopilot.py`** — CPU-only flight controller (optical-flow CEILING/WALL detector + playbook
  recipes). Modes: `--dry-run`, `--mission` (run_mission), `--explore` (Map mode).
- **GPU note:** SLAM and the cascade **cannot share the GPU** (compute contention → SLAM RELOC spiral).
  Phase-2 separates them in time (Map mode = SLAM only; a future Scan mode pauses SLAM to run the cascade).

## What's built
**Phase 1 (done, hardware-verified):** io_bridge + bus + dashboard; SLAM + voxel map; DA-V2 depth;
**target detector** = 3-stage cascade (GroundingDINO+OWLv2 propose → DINOv2 verify → SIFT/LightGlue geom
gate); **3D lift + consensus** (`target_estimator`, mode-seeking + uncertainty) → confident multi-target
estimate. Offline E2E confirmed.

**Phase 2 — Map-mode explorer (`autopilot.py --explore`), built + flies live:**
- `ground_grid.py` — 2D free/unknown/occupied grid + frontier extraction, built per keyframe from SLAM points.
- `perception_worker` publishes **`TOPIC_PLAN`** (pos/heading/goal/bearing_err/done/plan_valid + ground
  raster); goal = nearest frontier (sticky hysteresis); `plan_valid=false` when SLAM not TRACKING.
- `ExploreController`: **ARM → TAKEOFF → ASCEND-to-ceiling → DESCEND →** leg loop **REPLAN → ORIENT
  (open-loop quantized turn from the calibrated playbook recipe) → ADVANCE (forward until flow WALL) →
  BACKOFF → SETTLE**; **RECOVER** when SLAM lost (back off→settle→rotate→settle, turns grow, give up by
  accumulated ~360°) and **STUCK** (HOLD, auto-resumes if a plan returns).
- `flight_playbook.json` + `RecipePlayer` — all control recipes + presets + `rules.rest_between_s`; the
  durations are the tunable knobs.
- `visualizer.overlay_plan` draws free/frontier/goal/heading aligned to the occupancy map.

## Solved-this-round: SLAM dies RAMMING the wall (not turning)
Live log `OUTPUT/diag/20260630_003008_autopilot.log`: the ≤45° turn clamp held (plan `OK` through BOTH
turns), then the drone flew forward into a flat wall — looming went `+1.78 → +0.006` in ONE frame, ~0.9s
of frozen image, `PLAN-STALE` appeared DURING the post-wall settle **before** the reverse command. So
**ramming a wall until the image freezes kills monocular SLAM (no parallax); reversing a dead track can't
revive it** (reverse-probe FAILED — kept for the glass/unmapped fallback only). The win to keep: **a
small (≤45°) turn does NOT kill SLAM**.

## DONE + FLOWN — forward-clearance STAND-OFF (primary forward stop)
`map_store.MapStore.clearance` (ground-plane ray FAN, nearest hit) → perception publishes
`forward_clearance_dist` on TOPIC_PLAN → `autopilot.ADVANCE` stops with margin (`SETTLE → REPLAN`, SLAM
alive) when `forward_clearance_dist <= stop_clearance_dist`. Flow `wall_contact` = glass/unmapped fallback.
≤45° turn clamp decoupled to its own `clamp_leg_turn` flag. Visualizer draws the red clearance ray +
`clear=Xu`. **Live 2026-06-30 09:17: `stop_clearance_dist: 0.6` is RIGHT** (1.5 was too conservative — the
drone couldn't move; 0.6 saved it repeatedly). Knobs: `config.yaml autonomy.explore` (`stop_on_clearance`,
`clearance_fan_deg/n`, `clearance_skip`, `clearance_min_count`, `clearance_max_range`, `stop_clearance_dist`).

## BUILT THIS SESSION (self-test-verified; awaiting a full live flight)
Two fixes for the 09:17 flight's remaining issues (vertical drift into
inner walls; 45° batch turns head-butting partitions). Implemented + self-test-verified 2026-06-30:
- **Altitude lock.** `perception._plan_payload` publishes `pos_y` (camera Y; **+Y is DOWN**). `autopilot`
  caches `target_altitude_y` LIVE from the first valid post-prelude plan (persists across `reset_leg`); in
  ADVANCE, if `pos_y > target + alt_drift_floor` (sunk past the deadband) it injects UP (`joy_vertical:-1`)
  with the forward push, clearing at target. One-sided (counters sinking). Knobs: `altitude_lock`,
  `alt_drift_floor`.
- **Parallax scouting (turn↔push↔turn, matches the user's script).** `perception` publishes a
  `clearance_ring` (clearance at 8 headings via `MapStore.clearance`). A goal needing MORE than one
  `turn_step` (`|bearing_err| > turn_step_deg`) is reached as: **turn one step → short `PARALLAX_PUSH`
  translation (forward/back, whichever the post-turn ring says is roomier) → SETTLE → REPLAN → turn again →
  … → once aimed within one step, ADVANCE to the goal.** The translation between rotations gives SLAM the
  parallax to survive the multi-step turn and keeps the drone roughly in place instead of advancing
  off-goal into inner walls. Push is **distance-quantized** — translate `parallax_push_dist` SLAM units
  (measured live from the pose), with `parallax_push_s` as a SAFETY time cap and the live clearance as a
  guard; if boxed in both axes it skips the push and just turns. `parallax_max_pushes` caps it. Knobs:
  `parallax_scout`, `parallax_push_dist`, `parallax_pad`, `parallax_push_s`, `parallax_max_pushes`.
- **Forward throttle (new, after the 10:50 flight raced into a brick wall before SLAM mapped it).** The
  clearance stop never fired — looming exploded `2→6` in a few frames (fast approach), wall unmapped, SLAM
  died on impact. Fix = slow the approach. `config.yaml autonomy.explore.forward_throttle` (set **0.1** for
  the first validation run) overrides `forward_preset["trigger"]` (was 0.55) for BOTH the ADVANCE leg and
  the forward parallax push. Verified the value reaches Unity unramped (`io_bridge` overlay runs last,
  overwriting the manual trigger-decay). Self-test `forward_throttle override` PASS.
- **Reverse throttle (backward ram fix).** `config.yaml autonomy.explore.reverse_throttle` rewrites the
  reverse magnitude (was 0.7) in ALL reverse maneuvers (back_off, reverse_probe, recovery back-off, backward
  parallax push) so a fast backward ram can't throw the drone to SLAM-killing angles. Reverse is a continuous
  0-1 throttle like forward. (Up/down NOT scaled: `joy_vertical` is a discrete -1/+1 axis, and weakening it
  would risk breaking the calibrated takeoff/ascend — left at full thrust.)
- **Frontier goal selection + done verification (`frontier_planner.py`, new pure-numpy module).** Fixes
  goal thrash (the planner abandoned a good far goal and flipped to a tiny frontier BEHIND, err +160°) and
  false "mission complete" (declared done with the lab half-built). (1) **Utility selection** —
  `size · max(behind_floor, cos(turn)) / (1 + dist_weight·dist)` → prefer BIG/AHEAD/NEAR, behind frontiers
  floored (last resort). (2) **Strong commitment** — keep the goal (re-associated to the nearest live
  frontier within `goal_assoc_dist` as the centroid drifts) until reached/gone or beaten by `goal_switch_factor`.
  (3) **Done verification** — on empty frontiers, fly ONCE to `ground_grid.farthest_free(pos)` (cached as a
  STATIC target — computed exactly once on the transition, never re-evaluated while verifying → no
  oscillation, per review) and re-scan; declare done only if still no frontiers after reaching it. Wired into
  `perception_worker._plan_payload` (replaced `_select_goal`; `farthest_free` computed only on the verify
  transition); logs `planner: … VERIFYING via far corner`. Config: `goal_dist_weight`, `goal_behind_floor`,
  `goal_switch_factor`, `goal_assoc_dist`, `verify_done`, `verify_min_dist` (removed unused `goal_switch_margin`).
- **Control-space SLAM-loss recovery (`autopilot.py`, replaces the old escalating-turn recovery).** Forensics
  of the 12:43 flight: it wasn't a wall — legs hit the 20 s timeout (throttle 0.2 = slow); PLAN-LOST (×18) =
  perception SILENT >3 s (slow frames), PLAN-STALE (×10) = SLAM not TRACKING; and the old `PLAN-LOST + ADVANCE
  → reset_leg` guard **livelocked** with REPLAN (relaunch stale goal → `c`-reset → ADVANCE aborted → repeat,
  zero motion). And the old recovery did **escalating 90/135/180° turns** — the exact SLAM killer. New design
  (with the user; pose is invalid during a tracking loss, so recovery is CONTROL-space not state-space): the
  controller is now `status`-aware —
  - **PLAN-LOST/NO-PLAN → HARD HOVER-HOLD (`HOLD_LOST`), indefinitely** — zero velocity, no clock-based
    recovery while perception is silent; kills the livelock. On the next packet: valid → OK/resume; invalid →
    PLAN-STALE.
  - **PLAN-STALE → `RECOVERY_REWIND`** — replay the INVERSE of the last `command_history_s` of flown maneuvers
    (`command_history` deque; `_invert_history`: forward↔reverse, turn θ→−θ, reversed order) to re-expose the
    camera to recorded keyframes; watch for OK → brake (SETTLE) → REPLAN.
  - **history empty/exhausted (e.g. a WALL hit cleared it) → parallax + ≤45° `FALLBACK`** (roomier axis from
    the last ring, single ≤45° turn, alternating), bounded by `fallback_max_attempts` → `STUCK`.
  - A `wall_contact` COLLISION clears `command_history` (post-impact orientation unknown → inverse replay
    invalid). Config: `command_history_s`, `fallback_retreat_s`, `fallback_max_attempts` (retired
    `recover_after_s`/`recover_turn_deg`/`recover_turn_step_deg`/`recover_max_total_deg` + the REC_BACKOFF/
    REC_TURN states).
- Tests: `autopilot --self-test` ALL PASS (`ALTITUDE-LOCK`, `PARALLAX-SCOUT`, `_ring_get`, `forward_throttle`,
  `reverse_throttle`, and the new `RECOVERY control-space`: invert / LOST→hold / STALE→rewind / OK→snapback /
  empty→fallback≤45 / wall-clears-history); `frontier_planner --self-test` ALL PASS; `ground_grid --self-test`
  ALL PASS (+ `farthest_free`).
- Offline-validated on `flight_20260628_092640.mp4`: SLAM TRACKING throughout; `pos_y` + `clearance_ring`
  publish; per-frame log now shows `y`, `ring f/b`. (That flight stayed ~level so no big ascent to sign-check;
  `pos_y` drifted slightly +, consistent with +Y down. Final sign confirmation = the live HUD `y` readout.)
- **TO DO — live run, RECORD VIDEO.** Keep `stop_clearance_dist: 0.6`. First validate `forward_throttle: 0.1`
  DRASTICALLY slows/stops forward motion (watch `[io_bridge] AUTO … trig=0.10`), then raise toward a value
  that moves but lets SLAM map walls + the clearance stop fire before impact (~0.25–0.4). Also watch: altitude
  holds through long legs; cramped far-goal turns log `parallax forward/backward` then progress. ⚠️ FIRST-RUN
  SAFETY: if the altitude sign were wrong the drone would DIVE on the first correction — watch the first
  `joy_vertical` inject; any manual key aborts. Tune `forward_throttle`, `alt_drift_floor`, `parallax_*` live.
  Also watch the NEW planner: it should COMMIT to the far goal (no flip to behind-me targets), and on
  "done" log `planner: … VERIFYING via far corner` + fly there before truly stopping. Tune `goal_dist_weight`
  / `goal_switch_factor` / `verify_min_dist` live.

## Second-priority / future fixes
- **Depth "too-low" bump-up (PARKED — approved design, not yet built).** When a stripe of hard-yellow
  (very-near) appears in the LOWER part of the depth frame, nudge the drone UP a bit — guards both "too
  close to the FLOOR" and "about to hit a LOW wall instead of flying over it"
  (`DEBUG_IMAGES/almost_too_low_02.png`). Note `obstacle_bar` deliberately EXCLUDES the bottom 30%
  (`BAND_BOTTOM=0.70`, "floor always near"), so this needs its own lower-band read. Design: perception
  computes a self-calibrating `low_obstacle` from the per-frame **normalized** proximity (lower band,
  fraction of columns above a "hard-yellow" threshold forming a stripe), publishes it on TOPIC_PLAN;
  autopilot injects UP (`joy_vertical`) during ADVANCE/PARALLAX_PUSH with a visible `LOW-OBSTACLE -> bump up`
  event; visualizer flags it on the depth panel. Complements (does NOT replace) the SLAM-`pos_y` altitude
  lock. No-leakage-safe (relative signal, no baked altitude).
- **Heading indexing / order-of-operations bug (DEFERRED, per user).** A ~45° gap between the aimed heading
  and the actual heading was visible in Visualizer during the PLAN-LOST-spiral stretch. The SLAM settle-gate
  (wait after a turn until the solve settles) is the first-pass mitigation; if the gap persists once SLAM is
  stable, hunt the actual pose/heading indexing. No automated "angle-vs-Visualizer" verifier exists today.

## Standing rules (every change)
- **NO SILENT FALLBACKS:** fail-fast OR set a visible/logged/HUD state flag; any fallback approved first.
- **HARD RULE — no manual-flight data leakage:** every autonomous limit is a LIVE self-calibrating
  signal; platform/signal characteristics (flow signatures, control magnitudes, turn calibration, the
  ~1 s healthy-SLAM compute time) are legitimate, this room's geometry is not.
- Image integrity (no undisclosed downscaling); start multi-step work with a TaskCreate list; **never
  commit unless asked**; self-test offline before live.

## Drone control mechanic (non-obvious — don't re-derive)
Yaw is a **"fly toward your aim"** scheme: yaw moves an aim crosshair, forward thrust flies toward it; a
**SUSTAINED yaw hold then `c` (reset)** rotates the body — turn ANGLE = hold duration (a brief pulse does
nothing useful). Calibrated turn ≈ 90° at yaw 1.0 for ~1.625 s (a true 90° is ~1.85–2.0 s, so turns
slightly under-rotate). io_bridge applies autopilot values directly (no ramp); yaw latches until `c`.
The only Unity telemetry back is `time` — everything else must come from vision.

## Environment & build (don't re-derive)
- Tree: `D:\EXTEND\C2_SIM\XLAB\` → `XLAB\` (read-only sim: Xlab.exe, Sample_Drone_Interface.py,
  OUTPUT\*.mp4) + `cartographer\` (our repo). One venv `cartographer\venv` (py 3.11.9, torch
  2.5.1+cu121) — run everything from it.
- **lietorch is a PATCHED LOCAL build** (`third_party/lietorch`; `build_lietorch.bat` +
  `lietorch_windows_const_fix.patch`) — NEVER pip-install upstream. Validate `lietorch_probe.py`.
- MASt3R-SLAM rebuild: `build_mast3r_slam.bat` then `build_mast3r_slam_step23.bat`.
- SLAM quirks (`slam_engine.py` = our streaming wrapper): `os.chdir` into the SLAM repo before loading;
  import only the needed modules (NOT `mast3r_slam.visualization`); recover the 4×4 pose via **Act3 on
  origin+unit axes, NOT `T_WC.matrix()`** (matrix() corrupts the pose under patched lietorch).
- `.gitignore` excludes `venv/`, `third_party/`, `test_assets/`, `OUTPUT/`, weights.

## Key technical facts (don't re-derive)
- **Sim protocol** (`Sample_Drone_Interface.py`): Python is the TCP **server** (127.0.0.1:65432); Unity
  connects in. 60 Hz `control_state` JSON (trigger/reverse, joy_horizontal strafe, joy_vertical altitude
  [−1 up/+1 down], yaw, pitch). Video = NDI 1280×720@30. Keys: 1=arm, w/s, a/d strafe, e/f up/down,
  arrows yaw/pitch, b=land, c=reset attitude, space=full-res capture, g=detect event.
- **Resolution:** transport 512×288 (16:9, never anamorphically squash); cascade runs on the hi-res
  (:5605) stream, box scaled back to 512×288 for the lift.
- **Ray lift:** per-pixel rays = normalized `X_canon` (cached `SlamEngine.ray_field`); world ray =
  `pose[:3,:3] @ ray_cam`; center ray ≈[0,0,1]; raycast skip 0.25u.
- **Recording is ~58 fps, not 30** — durations must come from keystroke `mono_ts`, never frame counts.

## Run procedure
1. Designate target once: `venv\Scripts\python.exe make_target.py` → `target.yaml`.
2. `Xlab.exe` → `python io_bridge.py` → `python perception_worker.py --no-display` →
   `python visualizer.py` → `python autopilot.py --explore --log`; press `m` to hand over.
3. Offline self-tests (no hardware): `autopilot.py --self-test`, `flow_contact_detector.py --self-test`,
   `ground_grid.py --self-test`. Offline SLAM E2E: `perception_worker.py --video OUTPUT\flight_<ts>.mp4 --no-display`.
4. Diagnostics: `--log` → `OUTPUT/diag/<ts>_autopilot.{log,csv}` (state transitions + flow verdicts +
   published commands); annotated shots in `DEBUG_IMAGES/`.

---

## History (compressed changelog — preserves the decisions & lessons)

### Phase 1 — manual map + target localize (2026-06-21 → -27)
- **M1–M4 ✅:** env + all models on GPU; io_bridge + frame_bus; DA-V2 depth overlay (finding: depth reads
  the glass window as open air → glass needs a SLAM-stall signal, not depth); SLAM + voxel `MapStore` +
  live dashboard ("fly a loop" signed off).
- **Target detector ✅ (the hard problem):** every single-shot/VLM engine failed this small-object,
  mural-cluttered, low-texture-3D task — Qwen2.5-VL-3B (non-deterministic, boxes murals), OWLv2 (scores
  any framed rectangle alike), dense DINOv2 / SIFT / LightGlue (5-engine bake-off: planar poster solvable,
  **3D rifle unsolved by any single engine**). Solved by the **3-stage cascade** (propose GD+OWLv2 →
  verify DINOv2 cosine vs a reference crop → geom gate: SIFT+RANSAC HARD for `2D_PLANAR`, LightGlue SOFT
  for `3D_GEOMETRY`); generalized by `AssetClass` (no hardcoded names). Result: rifle 0.77 good / **0 FP**
  across all negatives. Classifier (`target_classifier`, Qwen) is designation-only; flight path carries no VLM.
- **3D lift + consensus ✅:** back-project each detection center into the voxel map (`ingest_detection`
  raycast) → `target_estimator.estimate_all()` (iterative peel-off mode-seeking + uncertainty).
- **Two live runs → fixes:** (B) flight path froze because the trajectory was recorded only on keyframes
  → now per-frame + TOPIC_MAP on a 0.5 s timer. (C) "misplaced" target was actually **TWO real rifles** →
  reverted a wrong 1/distance weighting, added multi-target.
- **GPU choke (decisive):** with `--log`, SLAM + cascade on one GPU → perception ~0.29 fps + a RELOC
  spiral, stale poses, scattered hits. Not patchable; the fix is structural (never run them together) →
  motivates the Phase-2 **Map/Scan temporal separation**. (VRAM was fine ~9.7/16; it's compute contention.)

### Phase 2 — autonomy (2026-06-27 → -29)
- **Ceiling detector v1 = SLAM-pose rate/plateau → FAILED twice live.** Monocular pose is only ~1 Hz and
  drops to ~0.27 Hz at a near surface → the rate window never had 2 samples → never armed. **Lesson:
  validate detectors on REAL captured data, not synthetic streams** (the dense self-test hid it). → pivot.
- **`flow_contact_detector.py` (the working detector):** CPU Farneback, self-calibrating, scale-free.
  CEILING = vertical-flow `|dy_med|` collapses while ascending; WALL = looming `expansion` collapses
  while moving forward (unifies textureless-freeze and textured-slow-climb). Airborne latch + persistent
  per-command running-max ref (fixes "re-press UP while already parked at the ceiling"). Validated on real flights.
- **`flight_playbook.json` + `RecipePlayer`:** control recipes as DATA (platform dynamics). **Frame-rate
  bug:** recording is ~58 fps not 30 → first derivation inflated durations ~1.92×; a later "fix" merely
  divided by 1.92 (fabricated) — both discarded. Honestly **re-measured from keys `mono_ts`**: takeoff
  3.25 s, turn ~2.0 s/90°, back_off 0.3 s, arm = a real double-press.
- **Mission runner** (`run_mission` + editable `mission_demo.json`; `expand_mission` auto-inserts rests
  between steps): the full demo (arm→takeoff→ascend_until_ceiling→turn→forward_until_wall) **flew live
  2026-06-29**. Autonomy gate: the runner holds until `m` is pressed (else arm/takeoff elapse before handover).
- **HARD RULE codified** (CLAUDE.md + memory): room-specific answers must be detected LIVE; platform/
  signal characteristics (flow signatures, control magnitudes, the `c`-before-forward rule) are legitimate.
- **Map mode (`--explore`) — build + fix journey (all live-log driven, 2026-06-29):**
  - `--explore` didn't arm → added the arm→takeoff **prelude** (reuses the mission recipes; `airborne_done`
    guard never re-arms; `--no-takeoff` to skip).
  - Turns: first did closed-loop-on-SLAM-heading → thrashed (heading goes stale mid-spin → overshoot,
    spin↔backoff). A "pulsed" attempt was WRONG (yaw latches; a pulse only nudges the aim). The user
    explained the **"fly toward your aim" yaw mechanic** → settled on **open-loop quantized turns** built
    from the calibrated playbook recipe (per-leg re-plan is the outer correction).
  - Added **ASCEND-to-ceiling + a descend nudge** to the prelude so mapping runs at a consistent height
    near the ceiling; the descent is a tunable playbook recipe.
  - **Recovery** (SLAM lost): back off → settle → rotate → settle → replan; turns GROW each attempt;
    give up by **accumulated ~360°** (not a fixed count). Made **rest-separated** after an early version
    glued reverse+yaw with no settle. `rest_between_s` lives in the playbook (one tunable source).
  - Per-leg **forward-ref reset** so each ADVANCE re-calibrates its own free-forward looming.
  - **Current blocker → "## Current problem":** big turns break SLAM (RELOC freezes the pose) → path/map/
    plan freeze → stuck. → "## NEXT" reverse-probe experiment.

### Milestones
M1 models on GPU ✅ · M2 io_bridge+bus ✅ · M3 depth ✅ · M4 SLAM+map+dashboard ✅ · target cascade +
3D localize ✅ · Phase-2 mission runner flew live ✅ · Map-mode explorer built, **live SLAM-on-turns
blocker open** (reverse-probe experiment next). Deferred: Scan mode (360° cascade, GPU separation);
glass/opening nav detectors; Phase-3 report polish + GUI.
