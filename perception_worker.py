"""perception_worker.py — Process P2: GPU perception. M3 = depth overlay; M4 = + SLAM.

Subscribes to the io_bridge frame bus (downscaled 512x288 BGR frames) and, in ONE CUDA
context, runs:
  * **MASt3R-SLAM** (via `slam_engine.SlamEngine`) every frame — camera trajectory + dense
    per-keyframe pointmaps, fused in-process into a `map_store.MapStore` voxel/occupancy map;
  * **Depth Anything V2** (relative model) at a capped, *slower* cadence
    (`perception.depth_cadence_hz`) so its passes never stall SLAM tracking — yielding a
    forward-obstacle bar + a coarse depth grid.

It publishes two compact JSON payloads on its state bus (`perception_state_port`):
TOPIC_POSE (pose / mode / keyframe + voxel counts) and TOPIC_DEPTH (obstacle bar / grid) —
never raw pointmaps (those stay in-process; they are ~440 K floats/keyframe). A live window
shows the depth colormap + obstacle bar; in display mode a second window previews the
growing top-down map.

Offline mode (`--video`) drives the entire pipeline straight from a recorded mp4 (no
io_bridge/NDI), then exports the fused map — this is the M4 offline verification.

Depth semantics: Depth Anything V2's `-hf` relative model emits affine-invariant
**inverse depth** — larger value = *nearer*. We robustly normalize it per frame to a
`proximity` field in [0,1] (1 = nearest). "Obstacle near" = high proximity; the glass
window (which the model reads as open air) stays *low* proximity, which is exactly the
corroborating signal M5's glass detector wants. Raw (un-normalized) stats are published
too so absolute movement is visible, not just the per-frame normalization.

NO SILENT FALLBACKS (per CLAUDE.md): CUDA availability and the model load are asserted up
front; any failure raises. There is no CPU fallback and no try-except that downgrades to a
no-depth mode. The active path is published as a visible `depth_mode` flag in every payload.
"""

import argparse
import os
import time

import cv2
import numpy as np
import torch
import yaml

import frame_bus
import slam_engine
from map_store import MapStore

REPO = os.path.dirname(os.path.abspath(__file__))

# Depth Anything V2 relative model: predicted_depth is inverse-depth (larger = nearer).
DEPTH_MODE = "DAv2-relative"

# Forward-obstacle bar geometry.
N_BARS = 16                 # columns across the frame width
BAND_TOP = 0.25             # forward-view band (fraction of height): focus on what's *ahead*,
BAND_BOTTOM = 0.70          # excluding the floor directly beneath (always "near", not a fwd hazard)
COL_NEAR_PCTL = 75          # per-column near-ness = this percentile of proximity (emphasize near)
GRID_ROWS, GRID_COLS = 18, 32   # coarse proximity grid shipped on the bus for the map/UI


def load_config(path=None):
    path = path or os.path.join(REPO, "config.yaml")
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


# ==============================================================================
# Depth Anything V2
# ==============================================================================
class DepthEstimator:
    """Wraps DA-V2 relative depth. Fail-fast load; returns raw predicted depth.

    `infer(bgr)` takes an HxWx3 uint8 BGR frame and returns a float32 depth map at
    the same HxW (raw inverse-depth, larger = nearer).
    """

    def __init__(self, hf_id: str, device: str = "cuda"):
        assert torch.cuda.is_available(), (
            "CUDA not available — perception_worker requires the GPU. "
            "No CPU fallback (NO SILENT FALLBACKS)."
        )
        from transformers import AutoImageProcessor, AutoModelForDepthEstimation

        self.device = device
        self.hf_id = hf_id
        t0 = time.time()
        self.processor = AutoImageProcessor.from_pretrained(hf_id)
        self.model = AutoModelForDepthEstimation.from_pretrained(hf_id).to(device).eval()
        torch.cuda.synchronize()
        print(f"[perception] DA-V2 '{hf_id}' loaded in {time.time() - t0:.1f}s "
              f"| VRAM {torch.cuda.memory_allocated() / 1e9:.2f} GB")

    def infer(self, bgr: np.ndarray) -> np.ndarray:
        h, w = bgr.shape[:2]
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        inputs = self.processor(images=rgb, return_tensors="pt").to(self.device)
        with torch.inference_mode(), torch.autocast("cuda", dtype=torch.float16):
            out = self.model(pixel_values=inputs.pixel_values)
        # predicted_depth: (1, H', W') at the model grid -> resize back to the frame.
        pred = out.predicted_depth[:, None]  # (1,1,H',W')
        pred = torch.nn.functional.interpolate(
            pred, size=(h, w), mode="bicubic", align_corners=False
        )
        return pred[0, 0].float().cpu().numpy()


# ==============================================================================
# Depth -> obstacle features
# ==============================================================================
def robust_proximity(depth: np.ndarray):
    """Normalize raw inverse-depth to proximity in [0,1] (1 = nearest).

    Uses the 2nd/98th percentiles so a few hot pixels don't crush the scale.
    Returns (proximity, raw_stats_dict).
    """
    lo, hi = np.percentile(depth, 2), np.percentile(depth, 98)
    span = max(hi - lo, 1e-6)
    proximity = np.clip((depth - lo) / span, 0.0, 1.0).astype(np.float32)
    stats = {
        "min": float(depth.min()), "max": float(depth.max()),
        "mean": float(depth.mean()), "median": float(np.median(depth)),
    }
    return proximity, stats


def obstacle_bar(proximity: np.ndarray, n_bars=N_BARS):
    """Per-column near-ness across the central band. Returns a list of n_bars floats."""
    h, w = proximity.shape
    band = proximity[int(h * BAND_TOP):int(h * BAND_BOTTOM), :]
    edges = np.linspace(0, w, n_bars + 1, dtype=int)
    bars = []
    for i in range(n_bars):
        col = band[:, edges[i]:edges[i + 1]]
        bars.append(float(np.percentile(col, COL_NEAR_PCTL)) if col.size else 0.0)
    return bars


def forward_clearance(bars):
    """1 - (near-ness of the central third) => higher means more open straight ahead."""
    n = len(bars)
    central = bars[n // 3: 2 * n // 3] or bars
    return float(1.0 - max(central))


# ==============================================================================
# Visualization
# ==============================================================================
def render(frame_bgr, proximity, bars, telemetry):
    """Compose [ input | depth-colormap ] with an obstacle bar + telemetry overlay."""
    h, w = proximity.shape
    depth_u8 = (proximity * 255.0).astype(np.uint8)
    depth_color = cv2.applyColorMap(depth_u8, cv2.COLORMAP_INFERNO)  # bright = near

    # Obstacle bar drawn across the bottom of the depth panel.
    n = len(bars)
    bar_h = max(28, h // 5)
    y0 = h - bar_h
    edges = np.linspace(0, w, n + 1, dtype=int)
    cv2.rectangle(depth_color, (0, y0), (w, h), (0, 0, 0), -1)
    for i, nearness in enumerate(bars):
        x0, x1 = edges[i], edges[i + 1]
        bh = int(nearness * (bar_h - 4))
        # green (far/clear) -> red (near/obstacle)
        color = (0, int(255 * (1 - nearness)), int(255 * nearness))
        cv2.rectangle(depth_color, (x0 + 1, h - 2 - bh), (x1 - 1, h - 2), color, -1)
    cv2.putText(depth_color, "OBSTACLE  near=red", (6, y0 + 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    panel = np.hstack([frame_bgr, depth_color])
    cv2.putText(panel, "input", (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    cv2.putText(panel, f"depth ({DEPTH_MODE})  bright=near", (w + 6, 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    for i, line in enumerate(telemetry):
        cv2.putText(panel, line, (6, h - 10 - 16 * (len(telemetry) - 1 - i)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
    return panel


def build_payload(meta, depth, infer_ms, cadence_hz):
    proximity, stats = robust_proximity(depth)
    bars = obstacle_bar(proximity)
    grid = cv2.resize(proximity, (GRID_COLS, GRID_ROWS), interpolation=cv2.INTER_AREA)
    payload = {
        "depth_mode": DEPTH_MODE,
        "frame_id": meta.get("frame_id"),
        "mono_ts": meta.get("mono_ts"),
        "sim_time": meta.get("sim_time"),
        "controls": meta.get("controls"),
        "infer_ms": round(infer_ms, 1),
        "cadence_hz": cadence_hz,
        "obstacle_bar": [round(b, 3) for b in bars],
        "forward_clearance": round(forward_clearance(bars), 3),
        "depth_stats": {k: round(v, 3) for k, v in stats.items()},
        "grid_rows": GRID_ROWS, "grid_cols": GRID_COLS,
        "depth_grid": np.round(grid, 3).tolist(),
    }
    return payload, proximity, bars


# ==============================================================================
# Pipeline: SLAM (every frame) + DA-V2 depth (throttled), fused into the map.
# ==============================================================================
DEPTH_WINDOW = "Cartographer — perception (depth + obstacles)"
MAP_WINDOW = "Cartographer — top-down map"


class Pipeline:
    """Holds the GPU workers + the map, and processes one frame at a time.

    `step()` runs SLAM on every frame and DA-V2 at the depth cadence, integrates each new
    keyframe's pointmap into the voxel map, publishes TOPIC_POSE (every frame) + TOPIC_DEPTH
    (when depth ran), and returns the render panel (or None on a depth-skipped frame).
    """

    def __init__(self, cfg, conf_thresh=1.5):
        self.cadence_hz = float(cfg["perception"]["depth_cadence_hz"])
        self.min_interval = 1.0 / self.cadence_hz
        self.voxel_size = float(cfg["map"]["voxel_size"])

        # DA-V2 first (no cwd dependency), then SLAM (chdir's into its repo last).
        self.depth = DepthEstimator(cfg["models"]["depth_anything"]["hf_id"])
        self.slam = slam_engine.SlamEngine(conf_thresh=conf_thresh)
        self.mapstore = MapStore(self.voxel_size, tracking_mode=self.slam.tracking_mode)

        self.last_infer_mono = 0.0
        self.n_depth = 0
        self.last_report = time.monotonic()

    def step(self, frame_bgr, meta, state_pub=None, show=True):
        # --- SLAM every frame ---
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        t_slam = time.time()
        res = self.slam.process(rgb)
        slam_ms = (time.time() - t_slam) * 1000.0

        map_updated = False
        if res.new_keyframe and res.kf_points is not None and len(res.kf_points):
            self.mapstore.integrate(res.kf_points, res.kf_colors)
            self.mapstore.add_pose(res.camera_center)
            map_updated = True

        if state_pub is not None:
            cc = res.camera_center
            state_pub.publish(frame_bus.TOPIC_POSE, {
                "tracking_mode": res.tracking_mode, "mode": res.mode,
                "n_keyframes": res.n_keyframes, "n_voxels": len(self.mapstore),
                "frame_id": meta.get("frame_id"), "sim_time": meta.get("sim_time"),
                "camera_center": [round(float(x), 4) for x in cc] if cc is not None else None,
                "new_keyframe": res.new_keyframe, "reloc_event": res.reloc_event,
                "slam_ms": round(slam_ms, 1),
            })

        # --- DA-V2 depth, throttled to the slower cadence ---
        now = time.monotonic()
        payload = proximity = bars = None
        infer_ms = None
        if now - self.last_infer_mono >= self.min_interval:
            self.last_infer_mono = now
            t0 = time.time()
            depth_map = self.depth.infer(frame_bgr)
            infer_ms = (time.time() - t0) * 1000.0
            self.n_depth += 1
            payload, proximity, bars = build_payload(meta, depth_map, infer_ms, self.cadence_hz)
            if state_pub is not None:
                state_pub.publish(frame_bus.TOPIC_DEPTH, payload)

        if now - self.last_report >= 1.0:
            c = meta.get("controls", {}) or {}
            depth_hz = self.n_depth / (now - self.last_report)
            fc = f"{payload['forward_clearance']:.2f}" if payload else " -- "
            print(f"[perception] SLAM {res.mode:<8} kf {res.n_keyframes:3d} | "
                  f"vox {len(self.mapstore):6d} | slam {slam_ms:5.1f} ms | "
                  f"depth {depth_hz:3.1f} Hz | fwd_clear {fc} | "
                  f"trigger {c.get('trigger')} yaw {c.get('yaw')}")
            self.n_depth = 0
            self.last_report = now

        panel = None
        if show and payload is not None:
            telem = [
                f"SLAM {res.mode} kf={res.n_keyframes} vox={len(self.mapstore)} "
                f"slam={slam_ms:.0f}ms{'  RELOC!' if res.reloc_event else ''}",
                f"depth_mode={DEPTH_MODE} infer={infer_ms:.0f}ms  "
                f"fwd_clearance={payload['forward_clearance']:.2f}  "
                f"raw[min/med/max]={payload['depth_stats']['min']:.1f}/"
                f"{payload['depth_stats']['median']:.1f}/{payload['depth_stats']['max']:.1f}",
            ]
            panel = render(frame_bgr, proximity, bars, telem)
        return res, payload, panel, map_updated


# ==============================================================================
# Live loop (frame bus) and offline loop (recorded mp4)
# ==============================================================================
def _show_and_quit(panel, pipe, map_updated, show):
    """Render windows and return True if the user pressed 'q'."""
    if not show:
        return False
    if panel is not None:
        cv2.imshow(DEPTH_WINDOW, panel)
    if map_updated:
        cv2.imshow(MAP_WINDOW, pipe.mapstore.render_topdown(size=600, point_px=2, min_count=1))
    return (cv2.waitKey(1) & 0xFF) == ord("q")


def run_live(cfg, show=True, conf_thresh=1.5):
    frame_port = cfg["network"]["frame_bus_port"]
    pstate_port = cfg["network"]["perception_state_port"]
    pipe = Pipeline(cfg, conf_thresh=conf_thresh)
    frame_sub = frame_bus.FrameSubscriber(frame_port)
    state_pub = frame_bus.StatePublisher(pstate_port)  # binds; fail-fast if taken
    print(f"[perception] frame bus SUB :{frame_port} | state PUB :{pstate_port} "
          f"(TOPIC_POSE + TOPIC_DEPTH)")
    print(f"[perception] SLAM every frame ({pipe.slam.tracking_mode}); DA-V2 cap ~{pipe.cadence_hz:g} Hz")
    print("[perception] === READY === waiting for frames from io_bridge "
          "(focus a window, 'q' to quit).\n")
    try:
        while True:
            got = frame_sub.recv(timeout_ms=500)
            if got is None:
                if show and (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
                continue
            frame, meta = got
            _, _, panel, map_updated = pipe.step(frame, meta, state_pub, show)
            if _show_and_quit(panel, pipe, map_updated, show):
                break
    except KeyboardInterrupt:
        pass
    finally:
        print("[perception] shutting down ...")
        frame_sub.close()
        state_pub.close()
        if show:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass


def _video_frames(path, stride, max_frames, proc_w, proc_h):
    """Yield (bgr_512x288, meta) from an mp4, sub-sampled — mirrors io_bridge's output."""
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise RuntimeError(f"could not open recording: {path}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    src_idx = yielded = 0
    while True:
        ret, bgr = cap.read()
        if not ret:
            break
        if src_idx % stride == 0:
            bgr = cv2.resize(bgr, (proc_w, proc_h), interpolation=cv2.INTER_AREA)
            meta = {"frame_id": yielded, "mono_ts": time.monotonic(),
                    "sim_time": round(src_idx / fps, 3), "controls": {}}
            yield bgr, meta
            yielded += 1
            if max_frames and yielded >= max_frames:
                break
        src_idx += 1
    cap.release()


def run_offline_video(cfg, video, show=False, stride=3, max_frames=0,
                      out_dir=None, conf_thresh=1.5):
    """M4 offline verification: drive the full pipeline from a recorded mp4, export the map."""
    from pathlib import Path
    # Resolve to absolute BEFORE Pipeline()/SlamEngine chdir's into the SLAM repo.
    video = Path(video).resolve()
    assert video.exists(), f"recording not found: {video}"
    out_dir = Path(out_dir or os.path.join(REPO, "OUTPUT")).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    proc_w = cfg["perception"]["processing_width"]
    proc_h = cfg["perception"]["processing_height"]

    pipe = Pipeline(cfg, conf_thresh=conf_thresh)
    # Offline mode is self-contained: it builds + exports the map and does NOT touch the
    # state bus (no subscribers, and avoids colliding with a live worker on the same port).
    state_pub = None
    print(f"[perception] OFFLINE video={video.name} stride={stride} "
          f"max_frames={max_frames or 'all'} | exporting to {out_dir}")
    print("[perception] === READY === processing recording (SLAM + depth + map).\n")

    n = 0
    t0 = time.time()
    try:
        for frame, meta in _video_frames(video, stride, max_frames, proc_w, proc_h):
            _, _, panel, map_updated = pipe.step(frame, meta, state_pub, show)
            n += 1
            if _show_and_quit(panel, pipe, map_updated, show):
                print("[perception] interrupted by user")
                break
    except KeyboardInterrupt:
        print("[perception] interrupted — exporting what we have ...")

    dt = time.time() - t0
    print(f"\n[perception] DONE: {n} frames in {dt:.1f}s ({n/max(dt,1e-6):.1f} fps) | "
          f"{pipe.slam.n_keyframes} keyframes | reloc {pipe.slam.n_reloc} | "
          f"peak VRAM {torch.cuda.max_memory_allocated()/1e9:.2f} GB")
    print(f"[perception] map: {pipe.mapstore.stats(min_count=2)}")

    stem = video.stem
    png = out_dir / f"{stem}_livemap_topdown.png"
    pipe.mapstore.render_topdown(png, min_count=2)
    pipe.mapstore.save_npz(out_dir / f"{stem}_livemap.npz", min_count=2)
    print(f"[perception] top-down -> {png}")
    print(f"[perception] voxel map -> {out_dir / f'{stem}_livemap.npz'}")
    if state_pub is not None:
        state_pub.close()
    if show:
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass
    print("[perception] OK")


# ==============================================================================
# Offline self-test (no bus / no sim) — proves the model + overlay work on hardware.
# ==============================================================================
def run_self_test(cfg):
    hf_id = cfg["models"]["depth_anything"]["hf_id"]
    cadence_hz = float(cfg["perception"]["depth_cadence_hz"])
    src = os.path.join(REPO, "test_assets", "frame_a.png")
    assert os.path.exists(src), f"self-test asset missing: {src}"

    bgr = cv2.imread(src, cv2.IMREAD_COLOR)
    proc_w = cfg["perception"]["processing_width"]
    proc_h = cfg["perception"]["processing_height"]
    bgr = cv2.resize(bgr, (proc_w, proc_h), interpolation=cv2.INTER_AREA)

    depth = DepthEstimator(hf_id)
    t0 = time.time()
    depth_map = depth.infer(bgr)
    infer_ms = (time.time() - t0) * 1000.0

    meta = {"frame_id": 0, "mono_ts": time.monotonic(), "sim_time": 0.0, "controls": {}}
    payload, proximity, bars = build_payload(meta, depth_map, infer_ms, cadence_hz)
    print(f"[perception][self-test] {src}")
    print(f"[perception][self-test] infer {infer_ms:.1f} ms | depth {depth_map.shape} "
          f"| raw {payload['depth_stats']} | fwd_clear {payload['forward_clearance']}")
    print(f"[perception][self-test] obstacle_bar {payload['obstacle_bar']}")

    out = os.path.join(REPO, "test_assets", "perception_selftest.png")
    cv2.imwrite(out, render(bgr, proximity, bars,
                            [f"SELF-TEST infer={infer_ms:.0f}ms",
                             f"fwd_clear={payload['forward_clearance']:.2f}"]))
    print(f"[perception][self-test] overlay -> {out}")
    print("[perception][self-test] PASS")


def main():
    parser = argparse.ArgumentParser(description="Cartographer perception_worker (P2): SLAM + DA-V2 depth")
    parser.add_argument("--config", default=None)
    parser.add_argument("--no-display", action="store_true", help="headless: skip the OpenCV windows")
    parser.add_argument("--self-test", action="store_true",
                        help="run depth once on a test asset, save an overlay, exit (no bus/sim/SLAM)")
    parser.add_argument("--video", default=None,
                        help="OFFLINE: drive the full SLAM+depth+map pipeline from this mp4, export the map")
    parser.add_argument("--stride", type=int, default=3, help="offline: process every Nth source frame")
    parser.add_argument("--max-frames", type=int, default=0, help="offline: cap processed frames (0=all)")
    parser.add_argument("--conf-thresh", type=float, default=1.5,
                        help="per-point confidence cutoff for pointmaps fed into the map")
    parser.add_argument("--out", default=None, help="offline: output dir (default: OUTPUT/)")
    args = parser.parse_args()

    cfg = load_config(args.config)
    if args.self_test:
        run_self_test(cfg)
    elif args.video:
        run_offline_video(cfg, args.video, show=not args.no_display, stride=args.stride,
                          max_frames=args.max_frames, out_dir=args.out, conf_thresh=args.conf_thresh)
    else:
        run_live(cfg, show=not args.no_display, conf_thresh=args.conf_thresh)


if __name__ == "__main__":
    main()
