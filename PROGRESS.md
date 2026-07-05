# Cartographer — Progress & Resume Handoff

_Last updated 2026-07-05 (post test-flight fixes, then re-flown). Resume from THIS file._

**⚠️ A NEW ISSUE was observed on the latest flight ("good progress" — the fixes below worked — but a new
problem appeared). It has NOT been described/recorded yet: get the details before acting.**

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
**NEXT = triage the NEW ISSUE from the latest flight** (details pending from the user — see the ⚠️ note at
the top). The rewind fix + SLAM settle gate flew with good progress.

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
