# Cartographer — Progress & Resume Handoff

_Last updated **2026-07-07** (session 5). Resume from THIS file._

**Status:** Phase-1 (manual map + target localization) done & hardware-verified. Phase-2 autonomous
**Map-mode explorer** (`autopilot.py --explore`) flies live and stops before walls via the SLAM
forward-clearance stand-off. Session 5's two-phase gentle ceiling ascent is in and behaving (gradual
takeoff done).

**NEXT (in order):**
1. **Fix the goal-blacklist failure** — a flight shows the event-driven 2-bump blacklist failing
   badly (unreachable goals not being retired). Debug from that flight next (fresh context).
2. **Then Part 3 — per-goal height recalibration** (`CALIBRATING_HEIGHT`), designed below.

---

## Next up — planned, NOT built

### 1. Fix the goal-blacklist mechanism (do this FIRST)
The session-4 event-driven **2-bump blacklist** is not reliably retiring unreachable goals — a flight
shows it failing spectacularly (to be analyzed in a fresh context). **This blocks Part 3:** the height
recalibration is *triggered by goal changes*, and a broken blacklist means goals don't change
correctly, so the blacklist must be fixed before building on top of it. Debug inputs: that flight's
`OUTPUT/diag/<ts>_autopilot.{log,csv}` + `<ts>_timeline.html` (the session-3 replay tool).

### 2. Part 3 — per-goal height recalibration (`CALIBRATING_HEIGHT`), designed
- **Trigger:** ONLY when the committed frontier goal genuinely changes — at REPLAN, when
  `dist(new goal, previous leg goal) > goal_assoc_dist`. The FIRST goal after the prelude does NOT
  recalibrate (the prelude already tapped the ceiling). **NOT** tied to PLAN-LOST / PLAN-STALE.
- **Action:** re-run the Part-2 two-phase ascend → descend to re-tap the ceiling, then **re-null
  `target_altitude_y`** so the altitude lock re-latches from the fresh post-descend pose (undoing the
  per-leg downward drift), then hand to ORIENT for the new goal.
- **Reuse:** give ASCEND/DESCEND a `_post_ascend` return field so the same two states serve both the
  prelude (→ BASELINE_NUDGE → REPLAN) and CALIBRATING_HEIGHT (→ ORIENT). Gate with a
  `calibrate_on_goal_change` config flag. (Full design also in `plans/we-are-doing-a-hazy-quokka.md`.)

---

## Session log (newest first)

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
- **Deferred — Part 3** (per-goal `CALIBRATING_HEIGHT`) — see the **Next up** section near the top.
- **Tests:** autopilot / flow / frontier / ground_grid / perception self-tests PASS (new two-phase-
  ascent + baseline-nudge cases; prelude is now ARM→TAKEOFF→ASCEND→DESCEND→BASELINE_NUDGE→REPLAN). An
  offline SLAM+map run maps cleanly with depth gone.
- **⏭ NEXT — RE-FLY** `autopilot.py --explore --log`: the ascent should climb in gentle steps and
  latch the ceiling without a smash; a baseline translation should precede the first turn; the
  dashboard should read "DEPTH DISABLED". Tune the ascent pulse/rest/gain + baseline distance live.

### Session 4 (2026-07-06) — event-driven 2-bump blacklist (replaced a broken time-watchdog)
Symptom: at a glass wall the drone sat ~9 min never blacklisting the unreachable beyond-glass goals.
- **Root cause:** the unreachable-goal watchdog was a *time accumulator gated on SLAM health*. In the
  glass pocket SLAM ran hot but the drone kept flying on valid poses, so the accrual clock stayed
  frozen and never fired. **Lesson: time-accumulation proxies gated on SLAM health go blind exactly
  in the heavy glass/wall pockets.**
- **Fix — event-driven 2-bump rule:** the autopilot reports each discrete advance-blocked stop as a
  "bump"; TWO bumps on the same goal region permanently blacklist it (a bump elsewhere resets the
  count). Immune to SLAM-clock health; a kinematic latch makes one continuous contact = one bump.
- Also added reverse **BACKWALL** contact detection (detection-only; logs a reverse-into-wall). Tests
  + an in-process ZMQ pulse test PASS.

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
  kill SLAM; the per-leg replan is the outer correction).
- **Ramming a wall kills monocular SLAM** (no parallax freezes the image); reversing a dead track
  can't revive it (a reverse-probe experiment failed — kept only as a glass/unmapped fallback). →
  the **forward-clearance stand-off** (SLAM raycast, `stop_clearance_dist: 0.6`, flown) is the primary
  wall stop; the flow WALL detector is the fallback.
- **Frontier planner** (`frontier_planner.py`): utility selection + strong commitment + done-
  verification (fly to the farthest free corner, then declare done) — fixed goal thrash and false
  "mission complete".
- **Control-space SLAM-loss recovery** (pose is invalid during a loss): PLAN-LOST → hard hover-hold;
  PLAN-STALE → replay the inverse of recent maneuvers to re-expose keyframes; history empty → a
  bounded ≤45° fallback sweep → STUCK.
- **The unreachable-goal saga:** a goal behind glass / a wall is never consumed, so the planner
  re-hands it forever. The handling went through several dead ends — a position-conditioned watchdog
  (caused an A→B→A **ping-pong** because moving re-whitelisted a dead goal), a round-based permanent
  blacklist, then a distance-stagnation timer — each failing because it inferred "unreachable" from a
  proxy that went blind in the glass pocket. Session 4's **event-driven 2-bump** rule finally holds.

---

## Open issues
- **⚠️ Drone flies straight past an off-axis goal, won't turn (diagnosed, NOT fixed).** The heading is
  decided only at REPLAN, and REPLAN only runs at leg-end; ADVANCE flies open-loop with **no mid-leg
  re-aim**, so in open space it runs the full leg (`leg_max_s`) drifting off the goal before it
  re-plans. Fix direction (mid-leg re-aim vs shorter leg cap vs closed-loop heading) **not chosen —
  get the user's decision before building.**
- **Deferred:** Scan mode (360° cascade with SLAM/GPU temporal separation); a glass-window altitude
  descend-probe; Phase-3 report polish + GUI. (Part 3 height recalibration + the blacklist fix are in
  the **Next up** section near the top.)

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
- `ExploreController`: **ARM → TAKEOFF → ASCEND (two-phase) → DESCEND → BASELINE_NUDGE →** leg loop
  **REPLAN → ORIENT (open-loop ≤45° turn) → ADVANCE (forward until the clearance stand-off / flow
  WALL) → SETTLE**; control-space **recovery** on SLAM loss; **STUCK** hold; event-driven 2-bump
  blacklist for unreachable goals.
- `flight_playbook.json` + `RecipePlayer` — control recipes as data (the tunable durations).

---

## Reference — don't re-derive

### Drone control mechanic
Yaw is a **"fly toward your aim"** scheme: yaw moves an aim crosshair, forward thrust flies toward it;
a **SUSTAINED yaw hold then `c` (reset)** rotates the body — turn ANGLE = hold duration (a brief pulse
does nothing). Calibrated turn ≈ 90° at yaw 1.0 for ~1.625 s (a true 90° is ~1.85–2.0 s, so turns
slightly under-rotate). io_bridge applies autopilot values directly (no ramp); yaw latches until `c`.
`joy_vertical` is a **DISCRETE −1/0/+1 axis** (up/down = full thrust, can't be throttled); trigger &
reverse ARE continuous 0–1. The only Unity telemetry back is `time` — everything else is vision.

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
2. `Xlab.exe` → `python io_bridge.py` → `python perception_worker.py --no-display` →
   `python visualizer.py` → `python autopilot.py --explore --log`; press `m` to hand over.
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
clearance stand-off, control-space recovery, event-driven 2-bump blacklist) ✅ · depth removed +
two-phase ascent built, awaiting re-fly. Deferred: per-goal height calibration, Scan mode, GUI.
