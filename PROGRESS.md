# Cartographer — Progress & Resume Handoff

_Last updated: 2026-06-26. Read this first when resuming with fresh context. The design plan is at
`C:\Users\owner\.claude\plans\hey-read-this-file-breezy-otter.md`. **The single actionable resume
pointer is the "## NEXT" section at the bottom.**_

## What this project is (1 paragraph)
Assessment task: from the black-box **XLAB** Unity sim's single monocular drone feed, autonomously
**map the room and report the 3D location of a target object** (+ uncertainty). Three phases: Phase 1
Human Recon → Phase 2 Autonomous Survey → Phase 3 Localize & Report; then a GUI ("make it look like
an app"). Grading = internal consistency (metric scale NOT required); compute efficiency NOT graded.
All local on an RTX 3080 Laptop (16 GB).

## Where we are (2026-06-26)
**Phase 1** (the user's framing) = fly manually + SLAM-map the lab + record the flight path + detect
the target + report its 3D location. Every core piece is built and verified on hardware:
- **Sensing/transport** (io_bridge + frame bus), **SLAM + voxel map + live dashboard**, **depth** —
  all done and signed off live (see Milestones).
- **Target detector** — the hard problem; SOLVED by a **3-stage cascade** after every single-shot
  engine failed. Built, generalized, committed.
- **3D lift + consensus** — back-project each detection into the voxel map → robust 3D estimate.
  Built and verified offline; detector-agnostic.

The cascade is now **wired into the live app** (object_worker P4, `object_mode=CASCADE`), target
auto-classified at designation, verified offline E2E (`confident` 3D estimate, peak VRAM 9.68 GB).

**First live run done (2026-06-26)** — flight smooth, detection worked, but surfaced 3 issues; see
"## Live-flight fixes" for the diagnosis + what's fixed vs pending.

## Live-flight fixes (2026-06-26)
First live 4-process run revealed three problems (annotated shots in `DEBUG_IMAGES/`):
- **A. Detection slower than 2 s** — *pending*. Likely the cascade (~1.5-2 s) contends with SLAM on the
  one GPU. Added per-detection cadence logging; needs the user's re-fly CSVs to quantify (logging-only
  this pass).
- **B. Flight path froze / didn't persist** — **FIXED**. Root cause: `perception_worker` recorded the
  trajectory **only on keyframes** (~20/flight, with a ~1.5 s keyframe-creation stall every ~52
  frames). Now records the camera pose **every frame** + publishes `TOPIC_MAP` on a 0.5 s timer (not
  only per keyframe), trajectory capped to 1500 pts for the bus. Verified: `traj_poses` = every
  processed frame (was 1/keyframe). Residual ~1.9 fps base rate is SLAM throughput (tied to A).
- **C. Target marker misplaced — actually TWO RIFLES → MULTI-TARGET (FIXED).** The lift CSV showed two
  near-tied clusters: `[0.03,0.03,3.38]` (55 hits, frames→source ~1287) and `[0.68,0.03,5.03]` (51
  hits, source ~2419). First read them as accurate-vs-inaccurate (far ray hitting the wall) and added a
  1/distance weighting "fix". **The user then revealed the lab has TWO rifles** (source frames ~1100 &
  ~2300) — the two clusters are the **two real rifles**, not one good + one bad. So the weighting was
  misguided (it just switched which rifle was reported). **Reverted it; replaced with multi-target:**
  `TargetEstimator.estimate_all()` returns EVERY well-supported instance via iterative peel-off
  mode-seeking (`CLUSTER_RADIUS` 0.30 u extent; `MIN_INSTANCE` 8 raw hits to report — filters
  incidental 4-5-hit blips; `MIN_CLUSTER` 3 only gates the `confident` flag). `TOPIC_TARGET` now carries
  a `{"targets":[...]}` list; the visualizer marks each (T0/T1…), and the offline export writes all
  instances + a `.ply` (cloud + green flight path + magenta target points). Verified: self-test (2
  instances + a blip + scatter → exactly 2) and replaying the real hits → **both rifles**
  `[0.025,0.025,3.425]` and `[0.675,0.025,5.025]`, each confident.

**Diagnostic logging:** `--log` on object_worker + perception_worker writes CSVs to `OUTPUT/diag/<ts>/`
(`object` = detection cadence/timing; `perception` = per-frame SLAM/loop timing; `lift` = per-hit
geometry + estimate). Used to diagnose B/C offline; the user re-flies with `--log` to nail A.

## The detector: a 3-stage CASCADE (the solution)
Single-shot engines all conflated *"where is a candidate"* with *"is this THE target"* and over-fired
(see "## Detector history"). The cascade separates propose from verify. It is **generalized** by an
`AssetClass` — no hardcoded target names:

- **Stage 1 — propose:** GroundingDINO (text phrase) + OWLv2 image-guided (reference crop), pooled,
  low thresholds, per-source NMS. (Stage-1 recall ceiling = 1.00 on both test targets.)
- **Stage 2 — verify:** DINOv2 ViT-S/14 global crop-embedding cosine vs the reference; ref AND each
  candidate crop **letterboxed** (aspect-preserving, never squashed) to 224². Gate = the asset
  class's DINOv2 threshold.
- **Stage 3 — geometric gate (per asset class):** `2D_PLANAR` → SIFT+RANSAC homography **HARD gate**;
  `3D_GEOMETRY` → LightGlue inliers **SOFT bonus** (never vetoes). Survivors ranked by **DINOv2
  cosine first**, geometry as tie-break.

`AssetClass` params live in `cascade_detector.ASSET_CLASS_PARAMS` (the only per-class config):
`2D_PLANAR` → DINOv2 ≥0.33 + SIFT hard; `3D_GEOMETRY` → DINOv2 ≥0.40 + LightGlue soft.

**Results** (9 poster + 13 rifle positives, 15 negatives — `OUTPUT/cascade/`): the **3D rifle, which
every prior engine failed** (best 0.23 good / 0.93 FP), reaches **0.77 good / 0.00 FP**; poster 0.33
good / 0.00 FP. **0 false positives across all negatives** — the asymmetric gate proved its worth:
even when the looser planar DINOv2 0.33 leaked negatives into Stage 2, the SIFT hard gate held final
FP at 0. **Warm per-frame ~1.55 s** (GD 0.44 + OWLv2 0.84 + DINOv2 0.08 + geom 0.18; all resident).

**Diagnostic scripts (kept, reproducible):** `cascade_detector.py` (CLI single-target or
`--targets cascade_targets.yaml` batch) + `cascade_report.py` (embedded HTML funnel). The earlier
`benchmark_detectors.py`/`benchmark_report.py` (5-engine bake-off) remain as the evidence base.

## Live integration (in progress this session)
Goal: cascade running live in `object_worker` (P4) firing every ~2 s, feeding the existing 3D lift +
consensus; target auto-classified at designation with user override. **The 3D side is unchanged** —
the lift only consumes `found/frame_id/center/target_label` from `TOPIC_DETECTION`, which the payload
still provides. Built:
- `cascade_detector.LiveCascade` — all cascade models resident; `set_target(ref,text,asset_class)` +
  `detect(frame)`. Explicit `torch.cuda.empty_cache()` after each init stage + per-frame (first live
  VRAM-coexistence test).
- `object_worker.py` — the three dead detectors (Qwen/OWLv2/old-cascade) **removed**; new
  `CascadeDetector` wraps `LiveCascade`; `object_mode` is **`CASCADE`** only; loads the designated
  target from `target.yaml`; payload + overlay carry `asset_class`.
- `target_classifier.py` (new) — one-time Qwen2.5-VL pass on the crop → suggested text phrase +
  PLANAR/3D class. **Designation-only**; the flight path carries no VLM.
- `make_target.py` — crops at **native resolution** (fidelity), runs the classifier, lets the user
  confirm/override label + class, writes `target.yaml` `{reference_crop, text, asset_class}`.

**Verification status (2026-06-26):** ✅ classifier sanity (rifle→3D_GEOMETRY, poster→2D_PLANAR,
labels correct — needed a "judge the medium not the depicted content" prompt or it called the poster
SOLID). ✅ **offline E2E** on the rifle flight (`flight_20260622_183816.mp4`, single-process SLAM +
cascade + lift + consensus): rifle found 119/188, 114 lifted to map hits, estimator
**`confident=True` @ [0.025, 0.025, 3.425], radial_rms 0.14u, spread_p90 0.22u**, **peak VRAM 9.68 GB
/ 16** (coexistence headroom confirmed), ~2 s/detection. ⏳ Live 4-process run pending (user flies).
Note: two hit clusters appeared (estimate settled on the denser earlier one at z≈3.4; near-approach
hits sat at ~[0.7, 0, 5.0]) — mode-seeking is internally consistent, but worth an eye on which is the
true rifle. Not chased (per "don't over-tune the honest read").

## ⚠️ Binding rules (from `cartographer/CLAUDE.md`)
- **NO SILENT FALLBACKS.** No auto-failover / hidden try-except downgrades. Fail-fast OR set a
  visible, logged, UI-surfaced state flag (`tracking_mode`, `object_mode`, `asset_class`). Any
  fallback must be approved before coding.
- **Image integrity:** no undisclosed downscaling; maximize source fidelity into each model and log
  every resize (letterbox→224², OWLv2→960², etc.).
- **Always start work with a TaskCreate list.** Never commit unless the user explicitly asks.
- **Checkpoint at milestone boundaries;** the user reviews each step.

## Architecture & data flow
4 processes over a ZMQ bus (frame bus = CONFLATE newest-wins; state bus = multipart `[topic][json]`):
- **P1 `io_bridge.py`** — NDI capture + 60 Hz TCP control + keyboard. Publishes the 512×288 transport
  stream (:5601), a **hi-res 720p stream (:5605)** for detection, and status/`space`-capture events.
- **P2 `perception_worker.py`** — SLAM (MASt3R) every frame + DA-V2 depth (throttled) in ONE CUDA
  context → voxel `MapStore`; publishes TOPIC_POSE/DEPTH/MAP (:5603). SUBs TOPIC_DETECTION, **lifts**
  each detection (`ingest_detection` ray-casts the center pixel into the map), feeds
  `target_estimator`, publishes TOPIC_TARGET.
- **P3 `visualizer.py`** — read-only dashboard: [status | input | depth+bar | top-down map + live
  camera track + magenta TARGET marker].
- **P4 `object_worker.py`** — the cascade detector; SUBs the hi-res stream, publishes TOPIC_DETECTION
  (:5604) with bbox/center scaled to 512×288 transport.

**KEEP — detector-agnostic, verified, reused by any detector (don't break):** the **3D lift** in
`perception_worker.ingest_detection` (pose ring + `MapStore.raycast`; geometry confirmed, center ray
≈[0,0,1]), **`target_estimator.py`** (mode-seeking cluster consensus + uncertainty), the hi-res
`:5605` stream, the `TOPIC_DETECTION`/`TOPIC_TARGET` bus contract, the visualizer target marker, and
`--debug-lift`.

## Environment & build (don't re-derive)
```
D:\EXTEND\C2_SIM\XLAB\
├── XLAB\          ← black-box sim (READ-ONLY). Xlab.exe, Sample_Drone_Interface.py, OUTPUT\*.mp4
└── cartographer\  ← our repo (this dir). Sim referenced as ../XLAB/
```
- **One venv:** `cartographer\venv` (Python 3.11.9, torch 2.5.1+cu121). All processes run from it.
- Re-validate models: `venv\Scripts\python.exe smoke_test_models.py` (DA-V2 + Qwen) and
  `smoke_test_slam.py` (MASt3R two-view).
- **lietorch is a PATCHED LOCAL build** at `third_party/lietorch` — upstream pip/git segfaults on CUDA
  group ops (`const scalar_t*` kernels miscompiled by nvcc 12.1 + MSVC 14.36). Rebuild via
  `build_lietorch.bat`; the tracked fix is `lietorch_windows_const_fix.patch`. NEVER `pip install`
  upstream lietorch. Validate: `lietorch_probe.py` ("ALL LIETORCH CASES PASSED").
- **MASt3R-SLAM** rebuild: `build_mast3r_slam.bat` then `build_mast3r_slam_step23.bat` (the int64_t
  kernel patch must already be present in `third_party/MASt3R-SLAM/.../backend/src/*.cu`).
- `.gitignore` excludes `venv/`, `third_party/`, `test_assets/`, `OUTPUT/`, weights.

## Key technical facts (don't re-derive)
- **Sim protocol** (`../XLAB/Sample_Drone_Interface.py`): Python is the TCP **SERVER**
  (127.0.0.1:65432); Unity connects as client. 60 Hz length-prefixed `control_state` JSON
  (trigger/reverse fwd-back, joy_horizontal strafe, joy_vertical altitude, yaw, pitch). Only
  telemetry back = `time`. Video = **NDI** 1280×720@30 BGRA. Keys: 1=arm, w/s, a/d strafe, e/f up/down,
  arrows yaw/pitch, b=land, c=reset cam; `space`=full-res capture, `g`=object-detect event. ('o','f' taken.)
- **MASt3R-SLAM API** (import ONLY these — never `mast3r_slam.visualization`, needs absent pyimgui):
  from `mast3r_slam.config` → `load_config`, `config`; `mast3r_utils` → `load_mast3r`,
  `mast3r_inference_mono`, `mast3r_symmetric_inference`; `frame` → `create_frame`. RGB = float32 [0,1]
  HxWx3. `os.chdir(REPO)` before loading (repo uses relative paths). Reference loop: `third_party/
  MASt3R-SLAM/main.py`; our streaming wrapper: `slam_engine.py` (INIT→TRACKING→backend+retrieval).
- **Driving the loop single-process:** mirror main.py but skip viz and use an `InProcessManager` shim
  (real `mp.Manager()` deadlocks on Windows). World pts = `kf.T_WC.act(kf.X_canon)`, conf-filter
  `kf.get_average_conf()`. Recover the 4×4 pose via **Act3 on origin+unit axes — NOT `T_WC.matrix()`**
  (matrix() routes through Act4 and under patched lietorch corrupts the frame pose, killing keyframes).
- **Ray geometry (lift):** camera per-pixel rays = normalized `X_canon` (intrinsics fixed → cached on
  `SlamEngine.ray_field`); world ray = `pose[:3,:3] @ ray_cam`; raycast skip 0.25u. Verified center
  ray ≈[0,0,1].
- **Resolution:** transport 512×288 (16:9). MASt3R's own resize makes 512×288 from 1280×720 — do NOT
  anamorphically squash. The cascade runs on the **hi-res** (720p/native) stream; its box is scaled to
  512×288 by `object_worker.Pipeline._to_transport` for the lift.

## Files in repo
- `config.yaml` — all settings (paths via ../XLAB/, ports, resolution, model ids, thresholds,
  `runtime.object_mode: CASCADE`).
- **io/transport:** `frame_bus.py`, `io_bridge.py`, `test_frame_subscriber.py`.
- **perception/SLAM:** `perception_worker.py`, `slam_engine.py`, `slam_offline.py`, `map_store.py`,
  `target_estimator.py`, `visualizer.py`; diagnostics `lietorch_probe.py`, `slam_match_probe.py`.
- **detection:** `object_worker.py` (live, CASCADE), `cascade_detector.py` (+`LiveCascade`),
  `cascade_report.py`, `cascade_targets.yaml`, `target_classifier.py`, `make_target.py`,
  `benchmark_detectors.py`, `benchmark_report.py`, `annotate_targets.py`. `target.yaml` = the
  designated target (written by make_target).
- **build/env:** `build_lietorch.bat`, `lietorch_windows_const_fix.patch`, `build_mast3r_slam*.bat`,
  `smoke_test_models.py`, `smoke_test_slam.py`, `third_party/` (gitignored).

## Live-run launch procedure (4 processes)
0. Designate the target once: `venv\Scripts\python.exe make_target.py [--video <recon.mp4>]` → crop +
   confirm class/label → writes `target.yaml`. Validate: `object_worker.py --self-test`.
1. Kill stray `perception_worker`/`object_worker`/`visualizer` (a stray PUB on :5603/:5604 makes a
   worker fail-fast on bind).
2. Xlab.exe → T1 `python io_bridge.py` (arm with 1; Admin if the keyboard hook is dead).
3. T2 `python perception_worker.py --no-display` (SLAM+depth+map+3D lift; SUB :5604, PUB :5603).
4. T3 `python object_worker.py` (cascade ~0.5 Hz; PUB TOPIC_DETECTION :5604; `--no-display` for headless).
5. T4 `python visualizer.py` (dashboard + magenta TARGET marker + uncertainty).
- VRAM budget: perception ~7.6 GB + cascade ~1.5 GB ≈ 9.1 GB / 16 (Qwen classifier is
  designation-only, not concurrent).
- **Offline E2E (no hardware):** `perception_worker.py --video <flight.mp4> --detect --debug-lift
  --no-display` runs SLAM + cascade + lift + consensus in one process. Rifle flight with the target
  in view: `OUTPUT/flight_20260622_183816.mp4`. Poster flight: `flight_20260621_120829.mp4`.

## Milestones
- **M1 env + all models on GPU** ✅
- **M2 io_bridge + frame_bus** ✅ (hardware-verified)
- **M3 depth (DA-V2) overlay** ✅ (live wall/glass fly-through signed off). Finding: DA-V2 reads glass
  as open air → an M5 glass detector must make SLAM-stall authoritative.
- **M4 SLAM + voxel map + map_store + live perception_worker + live dashboard** ✅ (fly-a-loop signed off).
- **Target detection (cascade) + 3D localize** — detector ✅; live integration ✅; first live run ✅;
  flight-path (B) + target-placement (C) bugs ✅ fixed offline; **detection cadence (A) pending the
  re-fly logs**. ← Phase-1 capstone, nearly closed.
- **DEFERRED to Phase 2:** glass + opening detectors (navigation safety, only autonomy needs them);
  live point-cloud save / 3D flight replay; then autonomy (planner, explore) + Phase-3 report polish + GUI.

## Detector history (compressed — don't re-try these)
Every learned/VLM single-shot detector was inadequate for this small-object, mural/clutter-competing,
low-texture-3D task: **Qwen2.5-VL-3B 4-bit** (non-deterministic, degenerates to `!!!` at 720p, boxes
murals not the poster); **OWLv2 image-guided** as a gate (scores any framed rectangle ~the same);
**Qwen→OWLv2 cascade** (recall too low); and a 5-engine benchmark (SIFT/LightGlue/DINOv2-dense/OWLv2/
Qwen) confirming the planar poster is solvable but the **3D rifle is unsolved by any single engine**.
That benchmark is the evidence base that motivated the verify-cascade above. Full numbers in
`OUTPUT/benchmark/` and git history.

## NEXT (exactly one)
**Re-fly the live 4-process run WITH `--log`** (first live run done; B + C now fixed, A pending).
Launch per the procedure above but add `--log` to `object_worker.py` and `perception_worker.py`. Fly
manually, then post `OUTPUT/diag/<ts>/{object,perception,lift}.csv`. Confirm: (B) the flight path is
now dense + persists, (C) the TARGET marker lands on the actual rifle, and (A — the open one) read the
detection cadence + SLAM-stall correlation from the CSVs to decide the contention fix. That closes
Phase 1; then Phase 2 (autonomous flying).
