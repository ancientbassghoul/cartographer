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
from target_estimator import TargetEstimator
from diag_log import DiagLog, NullLog

REPO = os.path.dirname(os.path.abspath(__file__))

# Depth Anything V2 relative model: predicted_depth is inverse-depth (larger = nearer).
DEPTH_MODE = "DAv2-relative"

# Forward-obstacle bar geometry.
N_BARS = 16                 # columns across the frame width
BAND_TOP = 0.25             # forward-view band (fraction of height): focus on what's *ahead*,
BAND_BOTTOM = 0.70          # excluding the floor directly beneath (always "near", not a fwd hazard)
COL_NEAR_PCTL = 75          # per-column near-ness = this percentile of proximity (emphasize near)
GRID_ROWS, GRID_COLS = 18, 32   # coarse proximity grid shipped on the bus for the map/UI

MAP_GRID = 200              # resolution of the compact top-down occupancy summary on TOPIC_MAP


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

    def __init__(self, cfg, conf_thresh=1.5, debug_lift=False):
        self.debug_lift = debug_lift
        self._geom_logged = False
        self.cadence_hz = float(cfg["perception"]["depth_cadence_hz"])
        self.min_interval = 1.0 / self.cadence_hz
        self.voxel_size = float(cfg["map"]["voxel_size"])
        self.proc_w = int(cfg["perception"]["processing_width"])
        self.proc_h = int(cfg["perception"]["processing_height"])

        # DA-V2 first (no cwd dependency), then SLAM (chdir's into its repo last).
        self.depth = DepthEstimator(cfg["models"]["depth_anything"]["hf_id"])
        self.slam = slam_engine.SlamEngine(conf_thresh=conf_thresh)
        self.mapstore = MapStore(self.voxel_size, tracking_mode=self.slam.tracking_mode)

        self.last_infer_mono = 0.0
        self.n_depth = 0
        self.last_report = time.monotonic()
        self.last_map_pub = 0.0           # timer for TOPIC_MAP (dense trajectory) publishing
        self.MAP_PUB_INTERVAL = 0.5       # publish the map at >= 2 Hz even between keyframes

        # --- target lift (M-object Task 2): back-project detections into the voxel map ---
        # Recent per-frame poses so a detection (which lags its frame by the Qwen latency) can be
        # matched back to the camera pose of the frame it fired on. SLAM tracks every frame.
        self._pose_hist: dict[int, np.ndarray] = {}
        self._pose_keys: list[int] = []
        self.POSE_HIST_MAX = 600
        self.TARGET_MIN_COUNT = 2     # require a voxel seen >= this for a ray hit (denoise)
        self.TARGET_SKIP = 0.25       # skip the first 0.25u of each ray so a downward ray can't
                                      # grab a near-camera floor voxel before the target surface
        self.estimator = TargetEstimator()
        self.n_det_seen = 0
        self.last_target_pub = 0.0

        # --- diagnostic CSV logging (off unless enable_diag is called) ---
        self.diag_perf = NullLog()    # per-frame SLAM/loop timing
        self.diag_lift = NullLog()    # per-detection lift geometry + estimate evolution
        self._last_step_ts = None

    def enable_diag(self, ts=None, out_dir=None):
        """Open CSV diagnostic logs (per-frame timing + per-lift hit geometry)."""
        self.diag_perf = DiagLog("perception", [
            "wall_ts", "frame_id", "loop_dt", "slam_ms", "mode", "new_keyframe",
            "n_keyframes", "n_voxels", "reloc"], out_dir=out_dir, ts=ts)
        self.diag_lift = DiagLog("lift", [
            "wall_ts", "frame_id", "found", "bbox_area", "center_x", "center_y",
            "pose_found", "cam_x", "cam_y", "cam_z", "ray_x", "ray_y", "ray_z",
            "hit", "hit_x", "hit_y", "hit_z", "march_dist",
            "n_hits", "n_inliers", "cluster_frac", "est_x", "est_y", "est_z", "confident",
        ], out_dir=out_dir, ts=ts)

    def close_diag(self):
        self.diag_perf.close()
        self.diag_lift.close()

    def step(self, frame_bgr, meta, state_pub=None, show=True):
        # --- SLAM every frame ---
        t_step = time.time()
        loop_dt = (t_step - self._last_step_ts) if self._last_step_ts else 0.0
        self._last_step_ts = t_step
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        t_slam = time.time()
        res = self.slam.process(rgb)
        slam_ms = (time.time() - t_slam) * 1000.0

        fid = meta.get("frame_id")
        if res.pose is not None and fid is not None:
            self._remember_pose(int(fid), res.pose)

        self.diag_perf.row(
            wall_ts=round(t_step, 4), frame_id=fid, loop_dt=round(loop_dt, 4),
            slam_ms=round(slam_ms, 1), mode=res.mode, new_keyframe=int(bool(res.new_keyframe)),
            n_keyframes=res.n_keyframes, n_voxels=len(self.mapstore),
            reloc=int(bool(res.reloc_event)))

        # One-time geometry sanity: the center-pixel ray (camera frame) should point forward.
        if self.debug_lift and not self._geom_logged and self.slam.ray_field is not None:
            h, w = self.slam.ray_hw
            fwd = self.slam.ray_field[h // 2, w // 2]
            print(f"[perception][debug-lift] center-pixel ray (camera frame) = "
                  f"{np.round(fwd, 4).tolist()} (expect ~[0,0,1] forward) | ray_hw={self.slam.ray_hw}",
                  flush=True)
            self._geom_logged = True

        # Trajectory: record the camera center EVERY frame so the persisted/displayed flight path is
        # dense and "remembers" the whole flight (previously add_pose was keyframe-gated → ~1 pt/kf,
        # a sparse path that froze between keyframes). Voxel integration still happens per keyframe.
        map_updated = False
        if res.camera_center is not None:
            self.mapstore.add_pose(res.camera_center)
        if res.new_keyframe and res.kf_points is not None and len(res.kf_points):
            self.mapstore.integrate(res.kf_points, res.kf_colors)
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
            # Top-down occupancy snapshot: on a new keyframe (cells changed) OR on a timer, so the
            # DENSE trajectory reaches the visualizer without waiting for the next (sparse) keyframe.
            # Each message is a full self-contained snapshot, so a late joiner catches up immediately.
            now_mono = time.monotonic()
            if map_updated or (now_mono - self.last_map_pub) >= self.MAP_PUB_INTERVAL:
                state_pub.publish(frame_bus.TOPIC_MAP, self._map_payload(res, meta))
                self.last_map_pub = now_mono

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

    def _map_payload(self, res, meta):
        """Serialize MapStore.topdown_summary() to a JSON-able TOPIC_MAP payload.

        Colors are packed to one 0xRRGGBB int per cell to keep the snapshot compact.
        """
        s = self.mapstore.topdown_summary(grid=MAP_GRID)
        rgb = s["cells_rgb"].astype(np.int32)
        packed = (rgb[:, 0] << 16) | (rgb[:, 1] << 8) | rgb[:, 2]
        return {
            "tracking_mode": s["tracking_mode"], "grid": s["grid"],
            "bounds": s["bounds"], "span_world": round(s["span_world"], 3),
            "n_voxels": s["n_voxels_kept"], "n_keyframes": res.n_keyframes,
            "cells_u": s["cells_u"].tolist(), "cells_v": s["cells_v"].tolist(),
            "cells_rgb": packed.tolist(),
            "traj_u": s["traj_u"].tolist(), "traj_v": s["traj_v"].tolist(),
            "frame_id": meta.get("frame_id"), "sim_time": meta.get("sim_time"),
        }

    # ------------------------------------------------------------- target lift
    def _remember_pose(self, fid: int, pose: np.ndarray):
        if fid not in self._pose_hist:
            self._pose_keys.append(fid)
        self._pose_hist[fid] = pose
        while len(self._pose_keys) > self.POSE_HIST_MAX:
            self._pose_hist.pop(self._pose_keys.pop(0), None)

    def _pose_for(self, fid):
        """Pose of frame `fid`, or the nearest remembered frame (detections lag their frame)."""
        if fid is None or not self._pose_keys:
            return None
        if fid in self._pose_hist:
            return self._pose_hist[fid]
        nearest = min(self._pose_keys, key=lambda k: abs(k - fid))
        return self._pose_hist[nearest]

    def ingest_detection(self, det: dict):
        """Back-project a TOPIC_DETECTION center pixel into the voxel map → a target hit.

        Returns (hit_world (3,), distance) when the ray hits a map voxel, else None. Feeds the
        running TargetEstimator either way (a 'found but no map hit' is recorded as a miss).
        """
        if not det or not det.get("found"):
            return None
        self.n_det_seen += 1
        if not self.estimator.label and det.get("target_label"):
            self.estimator.label = det["target_label"]
        fid, center = det.get("frame_id"), det.get("center")
        bbox = det.get("bbox")
        bbox_area = (round((bbox[2] - bbox[0]) * (bbox[3] - bbox[1]), 1)
                     if isinstance(bbox, (list, tuple)) and len(bbox) == 4 else "")

        def _log(pose_found, hit_flag, cam=None, ray=None, hw=None, dist=None):
            """One lift.csv row per detection — logs cam+hit so true cam->hit distance + the
            bbox size (detection reliability proxy) can be correlated against the cluster offline."""
            e = self.estimator.estimate() or {}
            pos = e.get("position") or [None, None, None]
            g = lambda a, i: (round(float(a[i]), 4) if a is not None else "")
            self.diag_lift.row(
                wall_ts=round(time.time(), 4), frame_id=fid, found=1, bbox_area=bbox_area,
                center_x=(round(center[0], 1) if center else ""),
                center_y=(round(center[1], 1) if center else ""),
                pose_found=int(pose_found),
                cam_x=g(cam, 0), cam_y=g(cam, 1), cam_z=g(cam, 2),
                ray_x=g(ray, 0), ray_y=g(ray, 1), ray_z=g(ray, 2), hit=int(hit_flag),
                hit_x=g(hw, 0), hit_y=g(hw, 1), hit_z=g(hw, 2),
                march_dist=(round(float(dist), 4) if dist is not None else ""),
                n_hits=self.estimator.n_hits, n_inliers=e.get("n_inliers", ""),
                cluster_frac=e.get("cluster_frac", ""),
                est_x=pos[0], est_y=pos[1], est_z=pos[2],
                confident=(int(bool(e.get("confident"))) if e else ""))

        pose = self._pose_for(fid)
        if pose is None or center is None or self.slam.ray_field is None:
            self.estimator.add_found_no_hit(fid)
            _log(pose_found=False, hit_flag=False)
            return None

        # Detection center is in transport pixels (proc_w x proc_h); map it onto the ray field.
        h, w = self.slam.ray_hw
        u = int(np.clip(round(center[0] * (w - 1) / max(self.proc_w - 1, 1)), 0, w - 1))
        v = int(np.clip(round(center[1] * (h - 1) / max(self.proc_h - 1, 1)), 0, h - 1))
        ray_cam = self.slam.ray_field[v, u].astype(np.float64)
        ray_world = pose[:3, :3].astype(np.float64) @ ray_cam     # Sim3 scale cancels on normalize
        rd = ray_world / (np.linalg.norm(ray_world) + 1e-9)
        cam = pose[:3, 3]
        hit = self.mapstore.raycast(
            cam, ray_world, min_count=self.TARGET_MIN_COUNT, skip=self.TARGET_SKIP)
        if hit is None:
            if self.debug_lift:
                print(f"[perception][debug-lift] frame {fid} px=({u},{v}) MISS "
                      f"(ray hit no voxel; cam={np.round(cam,2).tolist()})", flush=True)
            self.estimator.add_found_no_hit(fid)
            _log(pose_found=True, hit_flag=False, cam=cam, ray=rd)
            return None
        center_world, dist = hit
        self.estimator.add(center_world, fid)
        if self.debug_lift:
            print(f"[perception][debug-lift] frame {fid} px=({u},{v}) cam={np.round(cam,2).tolist()} "
                  f"ray={np.round(rd,3).tolist()} -> hit={np.round(center_world,3).tolist()} @ {dist:.2f}u "
                  f"| n_hits={self.estimator.n_hits}", flush=True)
        _log(pose_found=True, hit_flag=True, cam=cam, ray=rd, hw=center_world, dist=dist)
        return center_world, dist

    def target_payload(self):
        """TOPIC_TARGET payload — a LIST of target instances (the object can appear more than once),
        sorted by support; or None if nothing is localized yet. Each instance carries its own
        position + uncertainty + counts + `confident` flag."""
        ests = self.estimator.estimate_all()
        if not ests:
            return None
        for e in ests:
            e["tracking_mode"] = self.slam.tracking_mode
            e["voxel_size"] = self.voxel_size
            e["min_count"] = self.TARGET_MIN_COUNT
        return {
            "targets": ests,
            "n_targets": len(ests),
            "label": ests[0].get("label"),
            "tracking_mode": self.slam.tracking_mode,
        }


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


def run_live(cfg, show=True, conf_thresh=1.5, debug_lift=False, log=False):
    frame_port = cfg["network"]["frame_bus_port"]
    pstate_port = cfg["network"]["perception_state_port"]
    obj_port = cfg["network"]["object_state_port"]
    pipe = Pipeline(cfg, conf_thresh=conf_thresh, debug_lift=debug_lift)
    if log:
        pipe.enable_diag()
    frame_sub = frame_bus.FrameSubscriber(frame_port)
    state_pub = frame_bus.StatePublisher(pstate_port)  # binds; fail-fast if taken
    # SUB to object_worker's detections (lazy connect — fine whether or not it's running yet).
    det_sub = frame_bus.StateSubscriber(obj_port, topics=[frame_bus.TOPIC_DETECTION])
    print(f"[perception] frame bus SUB :{frame_port} | state PUB :{pstate_port} "
          f"(TOPIC_POSE/DEPTH/MAP/TARGET) | detection SUB :{obj_port}")
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

            # Drain any target detections and lift them into the map.
            d = det_sub.recv(timeout_ms=0)
            while d is not None:
                hit = pipe.ingest_detection(d[1])
                if hit is not None:
                    e = pipe.estimator.estimate()
                    epos = e["position"] if e else "(<min instance)"
                    print(f"[perception] target hit {np.round(hit[0], 3).tolist()} "
                          f"@ {hit[1]:.2f}u | best {epos} | n_hits={pipe.estimator.n_hits}", flush=True)
                d = det_sub.recv(timeout_ms=0)
            now = time.monotonic()
            tp = pipe.target_payload()
            if tp is not None and (now - pipe.last_target_pub) >= 0.5:
                state_pub.publish(frame_bus.TOPIC_TARGET, tp)
                pipe.last_target_pub = now

            if _show_and_quit(panel, pipe, map_updated, show):
                break
    except KeyboardInterrupt:
        pass
    finally:
        print("[perception] shutting down ...")
        pipe.close_diag()
        frame_sub.close()
        state_pub.close()
        det_sub.close()
        if show:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass


def _video_frames(path, stride, max_frames, proc_w, proc_h, object_frame_h=720):
    """Yield (small_512x288, hires, meta) from an mp4, sub-sampled — mirrors io_bridge's two
    streams. `hires` is the native frame downscaled to `object_frame_h` (no upscale), for the
    object detector; perception uses the 512x288 `small`."""
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
            small = cv2.resize(bgr, (proc_w, proc_h), interpolation=cv2.INTER_AREA)
            sh, sw = bgr.shape[:2]
            if sh > object_frame_h:
                ow = int(round(sw * object_frame_h / sh))
                hires = cv2.resize(bgr, (ow, object_frame_h), interpolation=cv2.INTER_AREA)
            else:
                hires = bgr
            meta = {"frame_id": yielded, "mono_ts": time.monotonic(),
                    "sim_time": round(src_idx / fps, 3), "controls": {}}
            yield small, hires, meta
            yielded += 1
            if max_frames and yielded >= max_frames:
                break
        src_idx += 1
    cap.release()


def run_offline_video(cfg, video, show=False, stride=3, max_frames=0,
                      out_dir=None, conf_thresh=1.5, publish=False,
                      detect=False, detect_every=5, debug_lift=False, log=False):
    """M4 offline verification: drive the full pipeline from a recorded mp4, export the map.

    With `publish=True` it ALSO publishes TOPIC_POSE/DEPTH/MAP on the perception state bus,
    so `visualizer.py` can be exercised against a recording with no hardware/NDI. Default
    off so a plain export run stays self-contained and never collides with a live worker.

    With `detect=True` it ALSO loads Qwen (object_worker) and runs the full object chain in
    THIS process — detection every `detect_every` frames, back-projected into the map and
    aggregated — so the 3D-lift end-to-end can be verified offline (frame_ids align because a
    single loop owns both). Note: single-process, so this does NOT test live VRAM coexistence.
    """
    import json
    from pathlib import Path
    # Resolve to absolute BEFORE Pipeline()/SlamEngine chdir's into the SLAM repo.
    video = Path(video).resolve()
    assert video.exists(), f"recording not found: {video}"
    out_dir = Path(out_dir or os.path.join(REPO, "OUTPUT")).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    proc_w = cfg["perception"]["processing_width"]
    proc_h = cfg["perception"]["processing_height"]
    object_frame_h = int(cfg["perception"].get("object_frame_height", 720))

    pipe = Pipeline(cfg, conf_thresh=conf_thresh, debug_lift=debug_lift)
    if log:
        pipe.enable_diag()
    # Optional in-process object detector (offline E2E lift test).
    obj_pipe = None
    if detect:
        import object_worker
        obj_pipe = object_worker.Pipeline(cfg)
        obj_pipe.min_interval = 0.0   # cadence is governed by detect_every here, not wall-clock
        print(f"[perception] OFFLINE --detect: {obj_pipe.detector.object_mode} target "
              f"'{obj_pipe.label}' [{obj_pipe.asset_class}] every {detect_every} frames -> 3D lift")
    # Offline mode is self-contained by default: it builds + exports the map and does NOT
    # touch the state bus. --publish opts into the live bus to drive the visualizer offline.
    state_pub = None
    if publish:
        state_pub = frame_bus.StatePublisher(cfg["network"]["perception_state_port"])
        print(f"[perception] OFFLINE --publish: state bus PUB "
              f":{state_pub.port} (TOPIC_POSE+DEPTH+MAP+TARGET) for visualizer.py")
    print(f"[perception] OFFLINE video={video.name} stride={stride} "
          f"max_frames={max_frames or 'all'} | exporting to {out_dir}")
    print("[perception] === READY === processing recording (SLAM + depth + map).\n")

    n = 0
    t0 = time.time()
    try:
        for frame, hires, meta in _video_frames(video, stride, max_frames, proc_w, proc_h,
                                                object_frame_h):
            _, _, panel, map_updated = pipe.step(frame, meta, state_pub, show)
            n += 1
            if obj_pipe is not None and (n % detect_every == 0):
                det_payload, _ = obj_pipe.step(hires, meta, None, show=False)
                if det_payload is not None:
                    hit = pipe.ingest_detection(det_payload)
                    if hit is not None:
                        est = pipe.estimator.estimate()
                        epos = est["position"] if est else "(<min instance)"
                        print(f"[perception]   target hit {np.round(hit[0],3).tolist()} "
                              f"@ {hit[1]:.2f}u -> best {epos} n_hits={pipe.estimator.n_hits}",
                              flush=True)
                    if state_pub is not None:
                        tp = pipe.target_payload()
                        if tp is not None:
                            state_pub.publish(frame_bus.TOPIC_TARGET, tp)
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
    targets = None
    if obj_pipe is not None:
        ests = pipe.estimator.estimate_all()
        if ests:
            targets = [e["position"] for e in ests]
            report = {"label": ests[0].get("label"), "n_targets": len(ests), "targets": ests}
            with open(out_dir / f"{stem}_target.json", "w", encoding="utf-8") as f:
                json.dump(report, f, indent=2)
            print(f"[perception] {len(ests)} TARGET instance(s) of '{ests[0].get('label')}':")
            for k, e in enumerate(ests):
                print(f"[perception]   #{k} @ {e['position']} | inliers {e['n_inliers']}/{e['n_hits']}"
                      f" | radial_rms {e['radial_rms']}u spread_p90 {e['spread_p90']}u "
                      f"confident={e['confident']}")
            print(f"[perception] target report -> {out_dir / f'{stem}_target.json'}")
        else:
            print("[perception] TARGET: no map hits (target never lifted)")

    png = out_dir / f"{stem}_livemap_topdown.png"
    pipe.mapstore.render_topdown(png, min_count=2, targets=targets)
    pipe.mapstore.save_npz(out_dir / f"{stem}_livemap.npz", min_count=2)
    ply = out_dir / f"{stem}_livemap.ply"
    pipe.mapstore.save_ply(ply, min_count=2, trajectory=True, targets=targets)
    print(f"[perception] top-down (flight path + target marks) -> {png}")
    print(f"[perception] voxel map -> {out_dir / f'{stem}_livemap.npz'}")
    print(f"[perception] point cloud + flight path + targets (.ply) -> {ply}")
    pipe.close_diag()
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
    parser.add_argument("--publish", action="store_true",
                        help="offline: also publish TOPIC_POSE/DEPTH/MAP/TARGET on the state bus "
                             "(drives visualizer.py from a recording, no hardware)")
    parser.add_argument("--detect", action="store_true",
                        help="offline: also run Qwen target detection + 3D lift in-process "
                             "(E2E object-chain test; exports <stem>_target.json + marks the map)")
    parser.add_argument("--detect-every", type=int, default=5,
                        help="offline --detect: run a detection every Nth processed frame")
    parser.add_argument("--debug-lift", action="store_true",
                        help="log per-detection lift geometry (pixel, cam, ray, hit) + a one-time "
                             "center-pixel ray sanity check")
    parser.add_argument("--log", action="store_true",
                        help="write diagnostic CSVs to OUTPUT/diag/ (per-frame SLAM/loop timing + "
                             "per-lift hit geometry) for live-flight debugging")
    args = parser.parse_args()

    cfg = load_config(args.config)
    if args.self_test:
        run_self_test(cfg)
    elif args.video:
        run_offline_video(cfg, args.video, show=not args.no_display, stride=args.stride,
                          max_frames=args.max_frames, out_dir=args.out,
                          conf_thresh=args.conf_thresh, publish=args.publish,
                          detect=args.detect, detect_every=args.detect_every,
                          debug_lift=args.debug_lift, log=args.log)
    else:
        run_live(cfg, show=not args.no_display, conf_thresh=args.conf_thresh,
                 debug_lift=args.debug_lift, log=args.log)


if __name__ == "__main__":
    main()
