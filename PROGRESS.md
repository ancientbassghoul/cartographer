# Cartographer — Progress & Resume Handoff

_Last updated: 2026-06-22. Read this first when resuming with fresh context, alongside the
design plan at `C:\Users\owner\.claude\plans\hey-read-this-file-breezy-otter.md`. **The single
actionable resume pointer is the "## NEXT" section near the bottom.**_

## What this project is (1 paragraph)
Assessment task: from the black-box **XLAB** Unity sim's single monocular drone feed, autonomously
**map the room and report the 3D location of a target object**. 3 phases: Human Recon → Autonomous
Survey → Localize & Report (+uncertainty). Later: a GUI. Grading = internal consistency (metric scale
NOT required); compute efficiency NOT graded. All local on an RTX 3080 Laptop (16 GB).

## ⚠️ Binding rules (from `cartographer/CLAUDE.md`)
- **NO SILENT FALLBACKS.** No auto-failover / hidden try-except downgrades. Fail-fast OR set a visible
  state flag (`tracking_mode`, `object_mode`) that is logged + shown in the UI. Any fallback (e.g.
  Qwen→DINOv2, MASt3R→FeatureVO) must be **approved before coding**.
- **Always start work with a TaskCreate list.** Never commit unless the user explicitly asks.
- Work style with this user: **checkpoint at milestone boundaries**; they review each step.

## Layout & environment
```
D:\EXTEND\C2_SIM\XLAB\
├── XLAB\          ← black-box sim (READ-ONLY). Xlab.exe, Sample_Drone_Interface.py, OUTPUT\*.mp4
└── cartographer\  ← our repo (this dir). Referenced sim path = ../XLAB/
```
- **One unified venv:** `cartographer\venv` — Python 3.11.9, torch 2.5.1+cu121. Activate:
  `cartographer\venv\Scripts\python.exe`. All 3 processes run from it.
- Re-validate the env anytime: `venv\Scripts\python.exe smoke_test_models.py` (DA-V2 + Qwen) and
  `venv\Scripts\python.exe smoke_test_slam.py` (MASt3R two-view).
- To rebuild MASt3R-SLAM from scratch: `build_mast3r_slam.bat` then `build_mast3r_slam_step23.bat`
  (the int64_t kernel patch in `third_party/MASt3R-SLAM/mast3r_slam/backend/src/*.cu` must already be
  present in the source before building — it is a manual edit baked into the third_party checkout, NOT
  re-applied by the .bats; on a fresh clone you must re-apply it by hand).
- **lietorch is a PATCHED LOCAL build** at `third_party/lietorch` (NOT the pip/git version, which
  segfaults — see M4 below). To rebuild it: `build_lietorch.bat`. NEVER `pip install` upstream lietorch.
  Re-validate group ops: `venv\Scripts\python.exe lietorch_probe.py` (expects "ALL LIETORCH CASES PASSED").

## Status: Milestone 4 DONE ✅ — SLAM + offline map + map_store + live perception_worker + live
## dashboard (visualizer.py) + ON-HARDWARE FLY-A-LOOP SIGNED OFF 2026-06-22
_M4 fully complete. `visualizer.py` (Task 3) built + verified; the live dashboard was watched during a
real fly-a-loop and the live map was globally consistent (user sign-off 2026-06-22). Next milestone =
M5 (glass + opening detector). See "## NEXT" for the M5 resume pointer._

### PARKED (raised 2026-06-22, defer to autonomy): live point-cloud save + 3D flight replay
Live flights currently do NOT auto-save the map — only the offline `--video` path exports
(`*_livemap.npz` = voxel centers+colors+**trajectory**; `slam_offline.py` dumps a dense `.ply`). The
user wants the *live* run to also persist the cloud so the flight can be replayed/recreated in 3D.
Small change: add a save-on-exit (and/or a snapshot hotkey) to `run_live` calling
`MapStore.save_npz` + `render_topdown`, optionally streaming the dense per-keyframe pointmaps. This is
flagged as important for **Phase-2 autonomy**: the planner needs the voxel occupancy + pointmaps to
reason about free space and gap-vs-drone-clearance ("which holes can I fly through"). Do when we start
autonomy, not before.
- Built `slam_offline.py` to drive the FULL MASt3R-SLAM loop (tracker + FactorGraph + retrieval)
  over a recorded flight (`../XLAB/OUTPUT/flight_20260621_120829.mp4`), single-process, no viz.
  Diagnostics: `slam_match_probe.py`, `lietorch_probe.py`. Map export → `.ply` + `.npz` + top-down PNG.
- **Fixed: `mp.Manager()` deadlock** on Windows (spawn re-imports this module) → replaced with an
  in-process `InProcessManager` shim (we run tracker + backend in ONE process, no separate viz).
- **Fixed: lietorch CUDA group-op segfault (the real blocker).** A bare
  `lietorch.Sim3.Identity(1,device='cuda').inv()*...` access-violated (0xC0000005); the two-view smoke
  test missed it because it never called a group op. **Root cause:** in
  `third_party/lietorch/lietorch/src/lietorch_gpu.cu`, the `__global__` kernels declared input pointers
  `const scalar_t*`; with Eigen, `const` vs non-const selects different `Eigen::Map` template
  instantiations, and the const path is **miscompiled by nvcc 12.1 + MSVC 14.36** → illegal access.
  **Fix (user-authorized quarantine bypass, community-vetted):** changed input-pointer params from
  `const scalar_t*` → `scalar_t*` in ALL forward kernels (exp/log/inv/mul/adj/adjT/act/act4/as_matrix/
  orthogonal_projector/jleft). Rebuilt from a LOCAL clone via `build_lietorch.bat` (MSVC 14.36 + CUDA
  12.1, `TORCH_CUDA_ARCH_LIST=8.6`). `lietorch_probe.py` now passes inv/mul/act/retr (fresh +
  shared-memory). `slam_offline.py` runs the full tracker+backend end-to-end (~3 fps, peak 6.75 GB).
  **NOTE:** lietorch is now a LOCAL source build at `third_party/lietorch` — to rebuild, run
  `build_lietorch.bat` (do NOT `pip install` the upstream git version, which reintroduces the crash).
- **Full-video run VERIFIED:** 587 frames (stride 3) → **23 keyframes**, tracking never lost (0 reloc),
  retrieval backend found loop-closure candidates, 2.08M world points, ~2 fps, peak 7.2 GB. Map artifacts
  in `OUTPUT/`: `*_cloud.ply`, `*_map.npz`, `*_topdown.png`. Top-down shows a globally-consistent
  room/corridor with coherent walls + a clean forward trajectory. **Offline SLAM milestone done.**
- **Live integration DONE + offline-verified 2026-06-22.** `slam_engine.py` wraps the proven loop as a
  streaming `SlamEngine.process(rgb)->SlamResult` (lazy/side-effect-free import; chdir's into the SLAM
  repo only on init). `perception_worker.py` now runs **SLAM every frame + DA-V2 throttled in ONE CUDA
  context**, fuses each new keyframe's pointmap into an in-process `MapStore`, and publishes TOPIC_POSE
  (+TOPIC_DEPTH) — never raw pointmaps. New offline mode `perception_worker.py --video <mp4>` drives the
  whole pipeline from a recording and exports the map. **Verified vs the slam_offline reference:** full
  587-frame pass → **22 keyframes, 0 reloc** (tracking never lost), 2.37M pts → 58K voxels, **peak 7.57 GB**
  (DA-V2+SLAM together), ~2 fps; `*_livemap_topdown.png` is structurally identical to the offline map.
- **NEXT (rest of M4):** Task 3 = live top-down view subscribing to the bus (OpenCV first; Rerun optional),
  then the on-hardware fly-a-loop sign-off (globally consistent LIVE map). Note: a `perception_worker`
  from the M3 test was still holding state port :5603 — kill stray workers before a live run.

## Status: Milestone 3 DONE ✅ (depth overlay) — LIVE wall-vs-glass SIGNED OFF 2026-06-22
_(Live hardware fly-through complete. Opaque surfaces read near/red, clearance collapses on approach
and recovers on retreat (`raw med` tracks distance correctly); ~2.5 Hz depth @ ~64 ms infer, no
crash; 60 Hz manual flight had NO lag while perception ran.)_
- **GLASS BEHAVIOR CONFIRMED (key M3→M5 finding):** when approaching the glass *pane* with a gap, DA-V2
  reads it as **far / open air** — the forward obstacle bars stay GREEN even though a barrier is there
  (`DEBUG_IMAGES/one_more.png`, `looking at glass window from afar.png`). i.e. **depth cannot see the
  glass, so depth alone would fly you straight into it** — this validates the plan's premise that the
  M5 glass detector must make the **SLAM-stall authoritative** and treat depth-open as only
  *corroborating*. Caveat: pressed right against the glass, the opaque **window frame/mullions** fill
  the forward band and read near (`fwd_clear`→0.09 in `bumping into glass window.png`) — frame ≠ pane.
- `perception_worker.py` written (P2, first GPU worker). Subscribes to the frame bus, runs **Depth
  Anything V2** (`depth-anything/Depth-Anything-V2-Base-hf`) capped at `perception.depth_cadence_hz`
  (3 Hz), derives a **forward-obstacle bar** (16 columns) + `forward_clearance` scalar + a coarse
  18×32 proximity grid, publishes them as JSON on the state bus topic `depth`, and renders a live
  `[ input | depth-colormap + obstacle-bar ]` window with telemetry.
- **Offline self-test PASSED** (`perception_worker.py --self-test` on `test_assets/frame_a.png`):
  DA-V2 loads 1.6 s / **0.39 GB VRAM**, infer ~340–470 ms, depth (288,512). On the recon frame the
  wall reads mid-distance → obstacle bar ~0.20 (green/clear), `fwd_clearance` 0.80. Overlay saved to
  `test_assets/perception_selftest.png`.
- **Design choices made:** new **`perception_state_port: 5603`** in config — each PUB binds its own
  port (frame_bus convention; io_bridge keeps 5602 for status/detect, perception owns 5603 for depth),
  subscribers connect to both. **Depth semantics:** DA-V2 `-hf` relative model emits inverse depth
  (larger = nearer); we robustly normalize (2nd/98th pctl) to `proximity ∈ [0,1]` (1=nearest, bright
  in the INFERNO colormap). Glass reads as open air → low proximity = the corroborating signal M5
  wants. Raw stats published too. **Obstacle band = 0.25–0.70 of frame height** (forward view; excludes
  the floor directly beneath, which always reads "near" but is not a forward hazard) — a TUNABLE to
  revisit during live flight. Uses `torch.autocast` fp16; no CPU fallback (fail-fast if no CUDA).
- **Live M3 verification: DONE 2026-06-22** (see the M3 status block above for the wall/glass result).

## Status: Milestone 2 DONE ✅ (io_bridge + frame_bus) — verified on hardware 2026-06-21
- `frame_bus.py`, `io_bridge.py`, `test_frame_subscriber.py` written and verified.
- **Hardware verification PASSED** (user ran Xlab.exe + io_bridge + test_frame_subscriber): live frames
  flowing to a 2nd process with no control lag. Earlier synthetic loopback confirmed ~10 fps 512×288 BGR,
  per-frame control/sim_time metadata, ~0 ms localhost latency, 'g' detect event on the state bus.
- Design choices made: frame bus uses ZMQ **CONFLATE** (newest-wins, true drop-old) with a single
  length-prefixed `[hdr_len][hdr_json][raw bytes]` blob (CONFLATE forbids multipart). State bus is
  non-conflated multipart `[topic][json]`. Per-frame meta carries `frame_id`, `mono_ts`, `sim_time`,
  and a `controls` snapshot (trigger/reverse/joy/yaw/pitch) so the glass detector later knows
  "forward-commanded". Dropped the sample's YOLO 'o'-autopilot + `detect_target` try-except (GPU work
  + silent fallback — both forbidden); manual flight mapping is otherwise byte-for-byte unchanged.

## Status: Milestone 1 DONE ✅ (env + all models verified on GPU)
- Depth Anything V2 (`depth-anything/Depth-Anything-V2-Base-hf`): 0.49 s/frame, 0.40 GB.
- Qwen2.5-VL-3B 4-bit (`Qwen/Qwen2.5-VL-3B-Instruct`): 2.6 GB, reads scenes correctly. Helper:
  `qwen_vl_utils.process_vision_info`, class `Qwen2_5_VLForConditionalGeneration`.
- MASt3R-SLAM native Windows build works: two-view inference 0.7 s/pair, peak 3.2 GB,
  pointmaps (4,288,512,3) at 512×288.

## Files in repo now
- `config.yaml` — all settings (paths use ../XLAB/, ports, resolution, model ids, thresholds).
- `requirements-ai.txt` — installed deps (torch via cu121 index, NOT listed there).
- `smoke_test_models.py`, `smoke_test_slam.py` — env validators (keep working).
- `build_mast3r_slam.bat` (steps 0-1), `build_mast3r_slam_step23.bat` (steps 2-4) — Windows build.
- `test_assets/` — sample_frame.png, frame_a/b.png (512-ready XLAB frames), smoke_depth.png.
- `third_party/MASt3R-SLAM/` — built editable + checkpoints/ (2.6 GB metric + retrieval).
- `frame_bus.py` — DropOldRing + ZMQ Frame/State Pub/Sub + encode/decode. Self-test: `python frame_bus.py`.
- `io_bridge.py` — P1: NDI capture + 60 Hz TCP control server + keyboard, fail-fast init, publishes
  downscaled frames + status/detect events. Flags: `--debug-keys`, `--no-display`, `--config`.
- `test_frame_subscriber.py` — M2 verification stand-in for perception_worker (prints fps/shape/latency).
- `perception_worker.py` — P2 GPU worker. M3: DA-V2 depth → obstacle bar/clearance/grid. M4: + SLAM
  every frame (via `slam_engine`) fused into an in-process `MapStore`; publishes TOPIC_POSE + TOPIC_DEPTH
  on :5603; depth/map windows. `Pipeline.step()` is the shared per-frame body. Flags: `--self-test`,
  `--video <mp4>` (offline: full pipeline from a recording + map export), `--stride`, `--max-frames`,
  `--conf-thresh`, `--out`, `--no-display`, `--config`.
- `slam_engine.py` — M4: `SlamEngine` streaming wrapper around the MASt3R-SLAM loop (INIT/TRACKING/RELOC
  + backend + retrieval) extracted from `slam_offline.py`; `process(rgb)->SlamResult` (mode, pose,
  new-keyframe world points+colors). Lazy import + chdir-on-init so importing it is side-effect-free.
- `slam_offline.py` — M4 de-risk: drives the FULL MASt3R-SLAM loop over a recorded mp4 (single-process,
  no viz; in-process `InProcessManager` shim), exports `.ply`/`.npz`/top-down PNG. Flags: `--video`,
  `--stride`, `--max-frames`, `--conf-thresh`, `--out`. Run unbuffered (`$env:PYTHONUNBUFFERED=1`).
- `build_lietorch.bat` — rebuilds the patched local lietorch (`third_party/lietorch`).
- `lietorch_windows_const_fix.patch` — the tracked const→non-const kernel fix (third_party is
  gitignored, so this patch is the version-controlled copy; `git apply` it onto a fresh lietorch clone).
- `lietorch_probe.py`, `slam_match_probe.py` — M4 diagnostics (lietorch group ops; matching kernels).
- `third_party/lietorch/` — patched local clone (`lietorch_gpu.cu` const→non-const kernel fix).
- `map_store.py` — M4: `MapStore` fuses per-keyframe world pointmaps+poses into a sparse voxel/occupancy
  grid (`map.voxel_size`=0.05) + trajectory; top-down render + `.npz` export. Transport-agnostic (pure
  numpy, no ZMQ/torch) so it runs in-process inside perception_worker AND is offline-testable. Offline
  build vs the 2.08M-pt npz: 52.6K voxels (39.5x), coherent top-down. Flags: `--npz`, `--voxel-size`,
  `--min-count`, `--chunks`, `--out`.
- `visualizer.py` — M4 Task 3: P3 live dashboard. Read-only SUB on the perception state bus
  (:5603) for TOPIC_POSE+TOPIC_DEPTH+**TOPIC_MAP** (+ optional frame-bus :5601 for the input
  panel). Composes [status strip | input | depth+bar | top-down map+traj]. Caches the map
  render (redraws only on a new keyframe snapshot). Surfaces tracking_mode/reloc visibly
  (NO SILENT FALLBACKS). Flags: `--no-frame`, `--config`. No GPU/SLAM — pure display.
- NOT yet created: object_worker.py, report.py, run.py.

## Key technical facts already learned (don't re-derive)
- **Sim protocol** (`../XLAB/Sample_Drone_Interface.py`): Python is the TCP **SERVER** (127.0.0.1:65432);
  Unity connects as client. Python sends `control_state` JSON at 60 Hz (length-prefixed: 4-byte big-
  endian len + payload). Controls: trigger/reverse (fwd/back 0..1), joy_horizontal (strafe -1..1),
  joy_vertical (altitude -1..1), yaw, pitch (camera). Only telemetry back = `time`. Video = **NDI**
  (1280×720@30, BGRA). `static_boxes` is an overlay-draw channel TO Unity. Keys: 1=arm, w/s, a/d
  strafe, e/f up/down, arrows yaw/pitch, b=land, c=reset cam. 'o' and 'f' are TAKEN — use 'g' for
  object-detect hotkey.
- **MASt3R-SLAM API** (import only these — NEVER `mast3r_slam.visualization`, it needs the absent
  pyimgui): from `mast3r_slam.config` → `load_config("config/base.yaml")`, `config`; from
  `mast3r_slam.mast3r_utils` → `load_mast3r(device="cuda")`, `mast3r_inference_mono(model, frame)`,
  `mast3r_symmetric_inference(model, fi, fj)`; from `mast3r_slam.frame` → `create_frame(i, rgb, T_WC,
  img_size=512, device="cuda")`. `T_WC = lietorch.Sim3.Identity(1, device="cuda")`. RGB input =
  float32 [0,1], HxWx3 (cv2 BGR→RGB /255). The repo uses RELATIVE paths, so `os.chdir(REPO)` before
  loading. Full driving loop reference: `third_party/MASt3R-SLAM/main.py`.
- **Resolution:** transport 512×288 (16:9). MASt3R's own resize already produces 512×288 from
  1280×720 — do NOT anamorphically squash; letterbox if a model needs square.
- **lietorch CUDA group ops (inv/mul/act/retr) crash on the stock Windows build** — bug is `const
  scalar_t*` kernel params in `lietorch_gpu.cu` (Eigen picks a miscompiled `Eigen::Map` instantiation
  under nvcc 12.1 + MSVC 14.36). FIXED in the local patched build (`third_party/lietorch`, const→non-
  const on all forward kernels). Don't re-debug; just ensure `lietorch_probe.py` passes after any rebuild.
- **Driving the SLAM loop single-process:** mirror `main.py` (INIT mono → TRACKING `tracker.track` →
  `run_backend`), but skip the viz process and feed `SharedKeyframes`/`SharedStates` an `InProcessManager`
  shim (real `mp.Manager()` deadlocks on Windows). World points = `kf.T_WC.act(kf.X_canon)`, conf-filter
  `kf.get_average_conf() > thresh`, color from `kf.uimg`. See `slam_offline.py`.

## NEXT: M5 — glass detector + opening detector (the resume point)
**M4 is DONE** (live dashboard + on-hardware fly-a-loop signed off 2026-06-22). Start M5:
- **Glass detector:** when the drone is **forward-commanded** (controls.trigger>0) but SLAM camera
  translation ≈ 0 over `map.glass_stall_seconds` (1.5 s), declare a **virtual wall** there. This is
  AUTHORITATIVE; depth reading "open air" is only corroborating (M3 confirmed DA-V2 can't see glass —
  it'd fly you into it). Surface as a visible state flag (no silent fallbacks) + draw it on the map.
- **Opening detector:** fit wall planes (RANSAC) to the voxel map, find gaps in the walls, compare gap
  width to `map.drone_clearance_m` (0.30) → passable vs not. Feeds the Phase-2 planner.
- The frame bus already carries `controls` per frame (trigger/yaw/etc.) — the glass detector needs
  "forward-commanded", which is in `meta["controls"]`. Camera translation = delta of `res.camera_center`.

### Live-run launch procedure (reference — used for the M4 fly-a-loop, reuse for M5):
1. Kill any stray `perception_worker`/`visualizer` first (a stray PUB on :5603 makes the worker fail-fast on bind).
2. Xlab.exe → Terminal 1 `venv\Scripts\python.exe io_bridge.py` (arm with 1; Admin if keyboard hook dead).
3. Terminal 2 `venv\Scripts\python.exe perception_worker.py --no-display` (SLAM+depth; PUBs TOPIC_MAP on :5603 every keyframe).
4. Terminal 3 `venv\Scripts\python.exe visualizer.py` → live dashboard (input+depth+top-down map).
- Offline re-verify anytime (no hardware, drives the dashboard too with `--publish`):
  `perception_worker.py --video ../XLAB/OUTPUT/flight_20260621_120829.mp4 [--publish --no-display]`
  → exports `OUTPUT/*_livemap_topdown.png` (known-good baseline: 22 kf, 0 reloc, peak ~7.6 GB, 58044 voxels).
  Pair with `visualizer.py --no-frame` to watch it grow.
- Depth obstacle-bar tunables if needed: `BAND_TOP/BAND_BOTTOM` (0.25/0.70) + `COL_NEAR_PCTL` in `perception_worker.py`.

## Remaining milestones (see plan file for detail)
- M3: Depth overlay (DA-V2) ✅ offline AND ✅ live wall/glass fly-through signed off on hardware 2026-06-22.
- M4: SLAM engine ✅ Windows + ✅ offline map + ✅ `map_store.py` voxel/trajectory + ✅ live
  perception_worker integration + ✅ live dashboard (`visualizer.py`, Task 3) + ✅ on-hardware
  fly-a-loop signed off 2026-06-22. **DONE.**
- M5: glass detector (SLAM translation≈0 while forward-commanded = authoritative) + opening detector
  (RANSAC wall planes, gaps vs drone clearance).
- M6: object_worker (Qwen multi-image: reference crop + live frame, hotkey 'g'); DINOv2 fallback is
  approval-gated only.
- Later: Phase-2 autonomy (planner, bump-and-recover, frontier explore) + Phase-3 report + GUI.
