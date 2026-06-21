# Cartographer — Progress & Resume Handoff

_Last updated: 2026-06-21. Read this first when resuming with fresh context, alongside the
auto-loaded memory files (`cartographer-project-overview`, `mast3r-slam-windows-build`) and the
design plan at `C:\Users\owner\.claude\plans\hey-read-this-file-breezy-otter.md`._

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
  (the int64_t kernel patch in `third_party/MASt3R-SLAM/mast3r_slam/backend/src/*.cu` must be present
  — see `mast3r-slam-windows-build` memory).

## Status: Milestone 3 DONE ✅ (depth overlay) — offline-verified + user-approved 2026-06-21
_(Live wall-vs-glass flight is the M3 sign-off ritual; confirm it opportunistically during the M4
flights — see the checklist at the bottom. User reviewed the offline overlay output and approved.)_
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
- **NEXT (do FIRST on hardware):** live M3 verification — see checklist at the bottom.

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
- `perception_worker.py` — P2 GPU worker. M3: DA-V2 depth → obstacle bar + clearance + coarse grid on
  state bus :5603 + live overlay window. Flags: `--self-test`, `--no-display`, `--config`.
- NOT yet created: object_worker.py, map_store.py, visualizer.py, report.py, run.py.

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

## NEXT: live M3 verification on hardware (do this FIRST), then M4 — SLAM
M3 live verification checklist (with Xlab.exe running):
  1. Start Xlab.exe (Unity).
  2. Terminal 1: `cd cartographer && venv\Scripts\python.exe io_bridge.py` → "=== READY ===".
     Arm (1) and fly. (As Administrator if the keyboard hook is dead.)
  3. Terminal 2: `venv\Scripts\python.exe perception_worker.py` → DA-V2 loads, "=== READY ===",
     then a depth window appears `[ input | depth-colormap + obstacle-bar ]`. Expect ~3 Hz depth.
  4. **Fly toward an opaque wall** → the wall region of the colormap brightens (proximity↑), the
     central obstacle bars turn red, `fwd_clearance` drops toward 0. **Fly at the glass window** →
     depth stays dark/"far" there, obstacle bars stay green even though you're driving forward (this
     is exactly the corroborating signal the M5 glass detector relies on — SLAM stall = authoritative,
     depth-open = corroboration).
  5. Confirm io_bridge's 60 Hz manual flight has NO lag while perception runs. Then M3 is signed off.
  - If the obstacle bar feels floor-dominated or mis-aimed, tune `BAND_TOP/BAND_BOTTOM` (currently
    0.25/0.70) and `COL_NEAR_PCTL` at the top of `perception_worker.py`.

Then **Milestone 4 — SLAM + map**: add MASt3R-SLAM to `perception_worker` (same CUDA context, ~5–10 Hz
tracking loop, depth stays the slower cadence), publish poses + pointmaps, and build `map_store.py`
(voxel/occupancy + trajectory) with a Rerun / top-down view. *Verify:* fly a loop → globally
consistent map. (See the plan file for M4–M6 detail.)

## Remaining milestones (see plan file for detail)
- M3: Depth overlay (DA-V2) + forward-obstacle bar; verify depth collapses at walls, stays "far" at glass.
- M4: MASt3R-SLAM in perception_worker → map_store voxel/trajectory, Rerun view.
- M5: glass detector (SLAM translation≈0 while forward-commanded = authoritative) + opening detector
  (RANSAC wall planes, gaps vs drone clearance).
- M6: object_worker (Qwen multi-image: reference crop + live frame, hotkey 'g'); DINOv2 fallback is
  approval-gated only.
- Later: Phase-2 autonomy (planner, bump-and-recover, frontier explore) + Phase-3 report + GUI.
