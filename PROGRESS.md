# Cartographer — Progress & Resume Handoff

_Last updated **2026-07-09** (session 9 — planning). Resume from THIS file. Next build:
`plans/replan-deadstall-sweep-and-slam-tracker.md` (item 2)._

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

_Session-8 work is DONE + flight-confirmed (`20260708_195009`) — see the Documentation session-8 entry.
Two items remain, both designed, neither built:_

- **Item 2 — kill the REPLAN dead-stall** — DESIGNED (session 9), NOT built. Full plan:
  **`plans/replan-deadstall-sweep-and-slam-tracker.md`**. Diagnosed on `20260708_195009`: the planner
  returned `goal=None && !done` and the controller idled forever (`autopilot.py:1401-1403`) — the
  done-verification stage silently never fired (the `farthest_free`/`verify_min_dist` "too near" gate
  failed). Fix: replace the fragile verify with a deterministic **bounding-box diagonal sweep** (fly to
  the opposite corner, inset 1 u/axis with per-axis midpoint-clamp on narrow axes; new
  `ground_grid.sweep_corner`) that either surfaces new frontiers en route or declares a visible **DONE**
  (just print it — no Phase-3 handoff yet). Same plan also **moves `[SLAM_TRACKER]` out of the terminal
  into the replay HTML** (teal, distinct color). Operator note: room is only *mildly* mapped — deep
  interior coverage is the Part-3 next-phase idea below, not this fix.
- **Item 1 — per-replan height recalibration (`CALIBRATING_HEIGHT`).** 60 s cooldown from the last
  calibration; keep running altitude statistics; reject a calibration that taps a low ceiling object
  (new `pos_y` well below the live median) → nudge forward and retry. Design in
  `plans/glass-corner-blacklist-and-height-calib.md` (extended with the session-8 asks).

---

## Future (backlog)
- **REPLAN dead-stall (item 2)** — no infinite idle when the planner returns no goal. Designed:
  `plans/replan-deadstall-sweep-and-slam-tracker.md` (bbox diagonal sweep + SLAM_TRACKER → replay HTML).
- **Per-goal height calibration (item 1)** — `plans/glass-corner-blacklist-and-height-calib.md`.
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
- **REPLAN dead-stall (item 2, NOT fixed — DESIGNED session 9).** When the planner returns
  `goal=None && !done`, the controller idles in REPLAN forever (`autopilot.py:1401-1403`) — re-seen on
  the clean `20260708_195009` flight (planner ran out of reachable frontiers; the done-verification
  gate silently never fired). Fix designed in `plans/replan-deadstall-sweep-and-slam-tracker.md`:
  bounding-box diagonal sweep → visible DONE, plus a fail-visible bounded-idle backstop. (Turns
  themselves are fine — see session 8.) The older "heading decided only at REPLAN, no mid-leg re-aim"
  is a separate, milder concern.
- **Deferred:** Scan mode; a glass-window altitude descend-probe; Phase-3 report polish + GUI.

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
  WALL / self-calibrating ram guard) → SETTLE**; control-space **recovery** on SLAM loss; **STUCK**
  hold; event-driven 2-bump blacklist for unreachable goals. **Ram guard is self-calibrating**; the
  clearance stand-off is the primary wall stop. (The ORIENT open-loop turn works — session-8 re-fly.)
- `flight_playbook.json` + `RecipePlayer` — control recipes as data (the tunable durations).

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
clearance stand-off, control-space recovery, event-driven 2-bump blacklist, two-phase ascent,
self-calibrating ram guard) ✅ · rich flight-replay debugger ✅ · glass-corner blacklist escape (Bug
A+B) + frontier clearance buffer 🛠️ built, flew in the session-8 flights. **Session 8: confirmed turns
work (the "no-op" was a stale-heading logging artifact) + made the flight log trustworthy (committed goal
+ data staleness) + `[SLAM_TRACKER]` telemetry + reach/clearance/SLAM-settle eases + a plan-lost grey
marker; a clean flight (`20260708_195009`) followed. REPLAN stall fix (item 2) and per-goal height
calibration (item 1) queued.**
