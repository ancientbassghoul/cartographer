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

## Status: Post-live-run fixes (detection sensitivity + 3D accuracy) — IN PROGRESS 2026-06-23
_First live 4-process run flew smoothly but: (1) Qwen detection was knife-edge sensitive (1px flips
found/not-found), (2) the 3D target marker was wrong, (3) the visualizer lagged (updated only at
keyframe rate). Plan: `~/.claude/plans/golden-orbiting-cherny.md`. Root cause: ~60px poster in the
512×288 transport frame → too few Qwen patches → jittery boxes → rays fan through the partition-wall
room → scattered 3D. **Geometry confirmed CORRECT** (center-pixel ray = [0,0,1] via `--debug-lift`),
so the lift math is fine; the fix is better detection + outlier-robust aggregation. Detector stays
Qwen (user-approved "full-res first"); OWLv2/Grounding-DINO swap is the gated escalation if needed._
- **Phase A (done): viz responsiveness + diagnostics.** `visualizer.py` now draws the **live camera
  track + position every frame** from `TOPIC_POSE.camera_center` (deque, projected via map bounds) —
  decoupled from keyframe-rate map redraws (`overlay_live_camera`, generalized `_world_to_px`).
  `perception_worker.py --debug-lift` logs per-detection {pixel, cam, ray, hit, dist} + a one-time
  center-pixel ray sanity check (verified ≈[0,0,1]).
- **Phase B (done): "sniper" full-res detection.** New `network.frame_bus_hires_port: 5605` +
  `perception.object_frame_height: 720`. `io_bridge` publishes a 2nd **hi-res** frame stream (native
  720p, same `meta`/`frame_id` as the 512×288 stream). `object_worker` SUBs the hi-res port, runs
  Qwen on full pixels, and **scales the box/center back to 512×288** (`_to_transport`) before
  publishing TOPIC_DETECTION (the lift's `ray_field` is 512×288). Offline `_video_frames` now yields
  `(small, hires, meta)` and `--detect` feeds the hi-res frame to Qwen, mirroring live. Confirmed
  Qwen now sees `src 1280x720`; inference ~1.85 s (≈6× tokens; fine at 0.5 Hz / detect_every=5).
- **Phase C (done): spatial-consensus 3D.** `target_estimator.py` replaced median with
  **mode-seeking** — densest cluster of hits within `CLUSTER_RADIUS=0.30u` wins, refined once around
  its centroid; outliers (wrong-wall rays) discarded. Robust past a >50% outlier majority (self-test:
  8 good + 7 scattered → locks the cluster, 20 mm err, confident). Adds `cluster_frac`. Raycast `skip`
  raised to **0.25u** (was 0.1) so a downward ray can't grab a near-camera floor voxel.
- **Phase D (NOT built — gated escalation):** if full-res Qwen still flickers, swap detection to
  **OWLv2 (image-conditioned, query=reference crop)** or **Grounding DINO (text-query)** behind a new
  visible `object_mode` flag (approval-gated per NO-SILENT-FALLBACKS). Try full-res Qwen first.
- **Verify (DONE — mixed result, points to Phase D):** offline `--detect --debug-lift` over the full
  flight. **Geometry CONFIRMED correct** (center ray ≈[0,0,1]); hi-res plumbing works (Qwen `src
  1280x720`); consensus + honest uncertainty work (final est `not confident`, cluster_frac 0.25).
  BUT hi-res did **not** fix accuracy: 5 finds (vs 2) but **scattered across the WRONG objects** —
  the auto-label **"Man with beard" matches the graffiti murals + a framed photo by the WELCOME
  sign, not the Nasrallah poster** (verified by rendering the detection frames). Also Qwen-4bit at
  720p is **slow (~5–6 s per *found* detection)** and **numerically fragile** (single-image full-res
  produced degenerate `!!!` output). Direct label test on a poster+murals frame: no label
  (generic or specific) reliably boxed the poster at 720p. **Conclusion: Qwen-3B-4bit is unreliable
  for this small-object, mural-competing grounding task at any resolution.** → escalate to **Phase D**
  (OWLv2 image-query / Grounding DINO), which the user anticipated.
  The infra fixes (hi-res stream, consensus, viz responsiveness, `--debug-lift`) are all kept.

### ⏭️ RESUME POINTER (after context clear): swap Qwen → OWLv2 image-guided detection
**Decision (user-approved 2026-06-23):** replace the Qwen detector with **OWLv2 image-guided
(one-shot) detection** — query = the reference crop, find visually-matching regions in each frame.
Rationale: Qwen-3B-4bit proved unreliable here (5 finds but all on murals / a framed photo, not the
poster; slow ~5–6 s/found; degenerate `!!!` at full-res). OWLv2 keys off the actual poster *image*,
so it should discriminate the printed poster from the painted murals where a text label cannot.

**SCOPE: ONLY the detector swaps.** Everything else stays exactly as built + verified: the hi-res
`:5605` stream, `object_worker._to_transport` (box→512×288), the `TOPIC_DETECTION` schema, the
perception lift + `MapStore.raycast` (skip 0.25u), `target_estimator` cluster-consensus, the
visualizer live overlay, `make_target.py`, `--debug-lift`. Do NOT touch SLAM/map/lift geometry
(geometry is CONFIRMED correct: center ray ≈[0,0,1]).

**STEP 0 — de-risk spike FIRST (before integrating).** Confirm OWLv2 discriminates poster vs murals
on the both-visible frame (src **1290** of `../XLAB/OUTPUT/flight_20260621_120829.mp4`; poster RIGHT
~x>950, murals LEFT ~x<350 of the 1280×720 frame):
```
from transformers import Owlv2Processor, Owlv2ForObjectDetection
import torch, cv2
proc  = Owlv2Processor.from_pretrained("google/owlv2-base-patch16-ensemble")
model = Owlv2ForObjectDetection.from_pretrained("google/owlv2-base-patch16-ensemble").to("cuda").eval()
crop  = cv2.cvtColor(cv2.imread("test_assets/target_dev.png"), cv2.COLOR_BGR2RGB)   # query image
frame = cv2.cvtColor(<src1290 bgr>, cv2.COLOR_BGR2RGB)                              # search image
inp = proc(images=frame, query_images=crop, return_tensors="pt").to("cuda")
with torch.no_grad(): out = model.image_guided_detection(**inp)
res = proc.post_process_image_guided_detection(
        out, target_sizes=torch.tensor([frame.shape[:2]]), threshold=0.9, nms_threshold=0.3)[0]
# take the highest-score box; its center should land on the RIGHT (poster), not LEFT (murals)
```
If it boxes the poster → integrate. If not → tune threshold, then try Grounding DINO, then
Qwen2.5-VL-7B fp16 (all approval-gated, all visible NO-SILENT-FALLBACK flags).

**STEP 1 — integrate in `object_worker.py`.** Add `Owlv2Detector` with the SAME interface the
pipeline already calls: `detect(ref_rgb, frame_rgb, label) -> {found, bbox, center, raw}` (label is
unused for OWLv2 — keep the arg for signature compat; skip the startup label-derivation in OWLv2
mode). Return box/center in the frame's (hi-res) pixels — existing `_to_transport` scales to
512×288. Take the single highest-score detection above `score_thresh`. Select the detector by
`runtime.object_mode`: `"QWEN"`→`QwenDetector` (keep intact), `"OWLV2"`→`Owlv2Detector`; set module
`OBJECT_MODE` from it so the visible flag + every payload reflect the active path.

**CONFIG to add:** `models.owlv2: {hf_id: "google/owlv2-base-patch16-ensemble", score_thresh: 0.9,
nms_thresh: 0.3}` and flip `runtime.object_mode: "OWLV2"`. Reference crop stays
`models.qwen_vl.reference_crop` (`test_assets/target_dev.png`) — OWLv2 needs only the crop, no label.

**NOTES / gotchas:**
- VRAM: OWLv2-base ≈0.6 GB (vs Qwen 2.6 GB) → more headroom; inference ~50–100 ms → could raise
  `object_cadence_hz` later (keep 0.5 first).
- Resolution: OWLv2-base resizes internally to 960²; the 720p stream is fine. Worth testing whether
  plain 512×288 already suffices — if so, point object_worker back at `:5601` and retire `:5605`
  (but keep `:5605` until proven unneeded).
- `image_guided_detection` returns MANY boxes → threshold + take top score. OWLv2 image-guided scores
  are NOT 0–1 calibrated; 0.9 is a starting guess — tune on the spike.
- Verify `from transformers import Owlv2ForObjectDetection` works in the venv. If missing,
  `pip install -U transformers` then re-run `object_worker.py --self-test` (a transformers bump can
  perturb the Qwen path).
- **Verify chain:** spike (poster vs murals) → offline `perception_worker.py --video
  ../XLAB/OUTPUT/flight_20260621_120829.mp4 --detect --debug-lift --no-display` (expect poster finds
  cluster, marker on poster wall, `confident`) → live 4-process run.

## Status: 3D lift (Task 2 of the gradable core) — WIRED + proven E2E offline 2026-06-22
_The full object chain runs end-to-end offline: Qwen detection → ray-cast into the voxel map →
3D hit → aggregated estimate, with a dashboard target marker. **Quality is not yet good (only 2
noisy hits on the small dev poster; see below) — that tuning is Task 3 / waits for the real
target.** The wiring is done and correct._
- **Lift design (user-approved): ray-cast into the voxel map.** `MapStore.raycast(origin, dir,
  max_range, min_count, skip)` (pure numpy, O(1) per step via the occupancy hash) marches the
  detection pixel's ray and returns the first occupied voxel — grounds the target on the exact
  geometry we report. Runs in **`perception_worker`** (owns SLAM poses + map): it SUBs
  TOPIC_DETECTION (:5604), keeps a ring of recent `frame_id→pose`, looks up the detection frame's
  pose (nearest if dropped), builds the world ray from the camera ray field + pose, casts, and
  feeds `TargetEstimator`. Publishes **TOPIC_TARGET** on :5603 (position + uncertainty).
- **Ray geometry:** camera per-pixel rays = normalized `X_canon` from the latest keyframe
  (intrinsics are fixed → view-independent, cached on `SlamEngine.ray_field`). Pose recovered via
  **Act3 on origin + unit axes** to build the 4×4 — NOT `T_WC.matrix()` (matrix() routes through
  Act4 on a view of the pose data and under the patched lietorch corrupts/freezes the frame pose,
  which silently kills keyframe creation; Act3 is proven safe). **Don't reintroduce matrix().**
- **`target_estimator.py`** (new, pure numpy): accumulates per-detection world hits → robust
  **median** position + uncertainty (radial_rms, per-axis std, spread_p90, inlier/hit/found/miss
  counts, coarse `confident` bool). MAD-trims outliers only when ≥4 hits; never lets inliers go
  empty (fixed a NaN-on-2-disagreeing-hits bug). Self-test passes (rejects a gross outlier, 10 mm).
- **E2E offline run** (`perception_worker --video <flight> --detect --no-display`, single process):
  587 frames, **22 kf / 58044 voxels (== M4 baseline) / peak 10.17 GB** (SLAM+DA-V2+Qwen together),
  1.4 fps. Qwen found the dev poster on **2 frames**; both lifted to map hits. Exports
  `OUTPUT/<stem>_target.json` + marks the target on `_livemap_topdown.png`. New flags `--detect`,
  `--detect-every`. **NOTE: single-process offline does NOT test the live 4-process VRAM/timing.**
- **KNOWN QUALITY GAP (→ Task 3):** only 2 hits, ~2u apart, one suspiciously near the camera
  (0.48u — likely a floor/near-voxel the downward ray hit before the poster wall). Fix levers:
  denser detection while the target is in view (lower `--detect-every` / live cadence) so the
  estimator's MAD trim rejects the floor outliers, + possibly a larger raycast `skip`/min-range.
  Deferred: tuning the **dev** poster isn't worthwhile — do it on the user's real Phase-1 target.
- **`visualizer.py`** now consumes TOPIC_TARGET: magenta target marker on the top-down map +
  a target line in the status strip (position, ±radial_rms, hits, confident/tentative).

## Status: Object detection (Task 1 of the gradable core) — `object_worker.py` BUILT + offline-verified 2026-06-22
_The detection leg of the object chain is done and self-test-passing; **live VRAM coexistence
(Qwen + SLAM + depth all running) is NOT yet tested** — that needs the 4-process live run. Next
= the 3D lift (Task 2). Decisions below settled with the user 2026-06-22._
- **Triggering (user-approved):** detection runs **continuously, throttled** to
  `perception.object_cadence_hz` (=0.5, start conservative), NOT hotkey-gated. Rationale: this is
  the eventual Phase-2 autonomy mode (no human to press a key); surface VRAM/latency cost now.
- **Separate process P4** (`object_worker.py`), its OWN CUDA context — Qwen-VL generation is
  autoregressive + slow (cold ~3.5 s, warm ~0.4–0.5 s/detection on the 3B 4-bit); folding it into
  the SLAM loop would stall tracking. New state-bus port **`object_state_port: 5604`**; publishes
  **TOPIC_DETECTION** `{object_mode, target_label, frame_id, sim_time, found, bbox[x1y1x2y2],
  center[cx,cy], infer_ms, raw}` — bbox/center in 512×288 frame pixels (null when not seen).
- **Reference crop = PROVIDED ASSET** (`models.qwen_vl.reference_crop`), loaded at startup. KEY
  FINDING: Qwen-VL **image-to-image matching of a small reference returns `[]`** (unreliable), but
  **label-driven text grounding lands the box**. So we derive a short text **label from the crop
  once at startup** (overridable via `models.qwen_vl.target_label`) and ground THAT per frame (the
  crop still rides along as the FIRST image, visual aid). This is the visible primary path, not a
  silent fallback — `target_label` is in every payload + logged. Prompt must stay **short/direct**;
  a verbose prompt that double-emphasizes the empty case makes the 3B model collapse to `[]`.
- **Coord mapping:** Qwen returns bbox in its smart-resized pixel space; resized side =
  `image_grid_thw[patch]*14` (smart_resize rounds each side to a multiple of 28). 512×288 →
  Qwen sees **504×280**; rescale by orig/resized. Verified correct on the self-test.
- **DEV TARGET = a framed portrait poster** ("Man with beard") cropped from the flight recording
  (`test_assets/target_dev.png`; scene `target_scene.png`, negative `no_target_scene.png`). This is
  a **stand-in to validate the pipeline** — the user must designate/provide their real Phase-1 target.
- Self-test: `object_worker.py --self-test` → loads 2.58 GB, derives label, POSITIVE frame boxed on
  the poster (`object_selftest.png`), NEGATIVE correctly empty, PASS. Modes: `--self-test`,
  `--video <mp4> [--publish]` (offline scan, saves overlays where found), live (default; SUB :5601,
  PUB :5604). NO SILENT FALLBACKS: CUDA + Qwen load fail-fast; no CPU/DINOv2 path.

## Status: Milestone 4 DONE ✅ — SLAM + offline map + map_store + live perception_worker + live dashboard
_M4 fully complete: `slam_engine` + `perception_worker` SLAM/depth fusion + `map_store` voxel map +
`visualizer.py` (Task 3) live dashboard, with the **on-hardware fly-a-loop SIGNED OFF 2026-06-22**
(live map globally consistent during a real flight). Next = **object detection (Qwen) + 3D
localize/report** — M5 (glass + opening) was deferred to Phase-2 on 2026-06-22; see "## NEXT". M4
history is below; a PARKED follow-up sits at the end of this section._
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
- **Task 3 (live dashboard) DONE 2026-06-22.** Added `frame_bus.TOPIC_MAP` + `MapStore.topdown_summary`
  (compact sparse occupancy snapshot: occupied cells + count-weighted colors + trajectory, already in
  pixel coords, ~220 KB, one per keyframe — each a full self-contained snapshot). `perception_worker`
  PUBs it per keyframe (and `--publish` replays pose/depth/map from a recording to drive the dashboard
  offline); `visualizer.py` composes [status | input | depth+bar | top-down map+traj]. Verified offline
  (render path + live bus on real GPU: pose/depth/map all flow) AND on hardware (fly-a-loop, live map
  globally consistent). Shipped in commit `ed92e1a`. Note: kill stray `perception_worker` (holds :5603)
  before the next live run.

### PARKED (raised 2026-06-22, defer to Phase-2 autonomy): live point-cloud save + 3D flight replay
Live flights do NOT auto-save the map yet — only the offline `--video` path exports (`*_livemap.npz` =
voxel centers+colors+**trajectory**; `slam_offline.py` dumps a dense `.ply`). The user wants the *live*
run to also persist the cloud so a flight can be replayed/recreated in 3D. Small change: save-on-exit
(and/or a snapshot hotkey) in `run_live` calling `MapStore.save_npz` + `render_topdown`, optionally
streaming the dense per-keyframe pointmaps too. Flagged as important for autonomy: the planner needs the
voxel occupancy + pointmaps to reason about free space and gap-vs-drone-clearance ("which holes can I
fly through"). Do when we start autonomy, not before.

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

## NEXT (2026-06-23): ➡️ see the "⏭️ RESUME POINTER: swap Qwen → OWLv2" block in the TOP status
section ("Post-live-run fixes"). The object chain is built + verified E2E but Qwen is unreliable for
this target; the immediate next task is the OWLv2 image-guided detector swap (detector ONLY; all
infra/lift/consensus/viz stays). The history below is the prior resume context (now superseded).

## (history) object detection (Qwen) + 3D localize/report — prior resume point
**M4 is DONE.** **Re-prioritization 2026-06-22 (user-approved): M5 (glass + opening detectors) is
DEFERRED to Phase-2 autonomy.** Rationale: both are navigation-safety features only an *autonomous*
drone needs — a human pilot already avoids glass / picks gaps during recon, and neither feeds the
grader. The gradable deliverable is **map the room + report the target object's 3D location (with
uncertainty)**, so jump straight to the object chain. It depends on NOTHING in M5 — it reuses the SLAM
pose + per-keyframe pointmaps we already produce, and the hotkey-`g` trigger fits the current
human-flying recon mode.

Build a thin END-TO-END vertical (human flies recon → reported 3D target):
1. ✅ **Detect — `object_worker.py`** DONE + offline-verified 2026-06-22 (see the status block at the
   top). Separate process P4, Qwen2.5-VL-3B 4-bit, **continuous-throttled** (not hotkey), derives a
   text label from the provided reference crop and grounds it (image-only matching was unreliable),
   publishes TOPIC_DETECTION on :5604. `object_mode="QWEN"` visible; DINOv2 fallback approval-gated
   only. **Live VRAM coexistence with SLAM+DA-V2 still untested** (needs the 4-process live run).
2. ✅ **Lift to 3D:** DONE + proven E2E (see the status block at the top). Ray-cast into the voxel
   map in `perception_worker`; `target_estimator.py` aggregates hits → position + uncertainty.
3. **← RESUME HERE. Report (Phase-3 core) + quality:** the wiring exists (TOPIC_TARGET + `_target.json`
   + dashboard marker). Remaining: (a) get a *confident* estimate — denser detection while the target
   is in view so the estimator trims outliers, + tune raycast skip/min-range to drop near-camera floor
   hits; (b) decide the final report artifact (the JSON + marked top-down may already suffice). **Best
   done on the user's REAL Phase-1 target, not the dev poster.** Also still pending: the live 4-process
   run (Xlab + io_bridge + perception + object_worker + visualizer) to test real VRAM/timing coexistence.

**Settle with the user BEFORE coding (checkpoint culture):** where does the target **reference crop**
come from — a provided asset, or picked from a recon frame? That choice shapes the Qwen prompt and the
worker's inputs. Also confirm: run object detection only on the `g` hotkey (recommended, VRAM) vs.
continuously.

### Designate the real Phase-1 target (do this BEFORE the live object run):
- Fly a recon with `io_bridge.py` (it records to `../XLAB/OUTPUT/flight_*.mp4`), looking at your target.
- `venv\Scripts\python.exe make_target.py [--video <that mp4>]` → browse frames (n/p ±1, N/P ±15,
  ENTER pick), drag a box around the target. Saves the crop → `models.qwen_vl.reference_crop`
  (`test_assets/target_dev.png`) AND the full frame → `test_assets/target_scene.png`.
- Validate: `venv\Scripts\python.exe object_worker.py --self-test` (must box your target; label is
  auto-derived from the crop — pin `models.qwen_vl.target_label` in config if you want a specific one).

### Live-run launch procedure (4 processes — object detection + 3D target):
1. Kill any stray `perception_worker`/`object_worker`/`visualizer` (stray PUB on :5603/:5604 makes a worker fail-fast on bind).
2. Xlab.exe → Terminal 1 `venv\Scripts\python.exe io_bridge.py` (arm with 1; Admin if keyboard hook dead).
3. Terminal 2 `venv\Scripts\python.exe perception_worker.py --no-display` (SLAM+depth+map+3D lift; SUBs detections :5604, PUBs POSE/DEPTH/MAP/TARGET :5603).
4. Terminal 3 `venv\Scripts\python.exe object_worker.py` (Qwen detection ~0.5 Hz; PUBs TOPIC_DETECTION :5604; shows the live bbox overlay — add `--no-display` for headless).
5. Terminal 4 `venv\Scripts\python.exe visualizer.py` → dashboard (input+depth+top-down map + magenta TARGET marker + uncertainty).
- VRAM budget: perception ~7.6 GB + Qwen ~2.6 GB ≈ 10.2 GB of 16 (single-process combined peaked 10.17 GB — fits; live 2-process timing still to confirm).
- (M4 3-process map-only run = same minus Terminal 3.)
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
- **NEXT — object detection + 3D localize/report (the gradable core):** `object_worker.py` (Qwen
  multi-image: reference crop + live frame, hotkey 'g') → back-project the detection through the SLAM
  pose + pointmap into a 3D world position → report position + uncertainty. DINOv2 fallback approval-
  gated only. _(Formerly "M6"; promoted ahead of M5 on 2026-06-22.)_
- **DEFERRED to Phase-2 — glass + opening detectors** _(formerly M5)_: glass = SLAM translation≈0 while
  forward-commanded (authoritative; depth can't see glass); opening = RANSAC wall planes, gaps vs
  `map.drone_clearance_m`. Navigation safety only an autonomous drone needs — NOT on the grading path.
  Build alongside autonomy. (Config keys `map.glass_stall_seconds`=1.5, `map.drone_clearance_m`=0.30
  already exist; M3 confirmed DA-V2 reads glass as open air, so SLAM-stall must be authoritative.)
- Later: Phase-2 autonomy (planner, bump-and-recover, frontier explore, + the deferred glass/opening
  detectors) + Phase-3 report polish + GUI.
