"""perception_worker.py — Process P2: GPU perception. Milestone 3 = depth overlay.

Subscribes to the io_bridge frame bus (downscaled 512x288 BGR frames), runs
**Depth Anything V2** (relative model) at a capped cadence (`perception.depth_cadence_hz`),
and from each depth map derives a **forward-obstacle bar** + a coarse depth grid. It
then:
  * publishes a compact JSON depth payload on its own state bus (`perception_state_port`)
    for map_store / the visualizer to consume (TOPIC_DEPTH),
  * renders a live window: the input frame, the depth colormap, the per-column
    obstacle bar, and telemetry text.

This is the first GPU worker. MASt3R-SLAM joins this same CUDA context in M4; depth is
deliberately the *slower* cadence so it never stalls SLAM tracking.

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
# Live loop
# ==============================================================================
def run_live(cfg, show=True):
    frame_port = cfg["network"]["frame_bus_port"]
    pstate_port = cfg["network"]["perception_state_port"]
    hf_id = cfg["models"]["depth_anything"]["hf_id"]
    cadence_hz = float(cfg["perception"]["depth_cadence_hz"])
    min_interval = 1.0 / cadence_hz

    depth = DepthEstimator(hf_id)
    frame_sub = frame_bus.FrameSubscriber(frame_port)
    state_pub = frame_bus.StatePublisher(pstate_port)  # binds; fail-fast if taken
    print(f"[perception] frame bus SUB :{frame_port} | depth state PUB :{pstate_port}")
    print(f"[perception] DA-V2 cadence cap ~{cadence_hz:g} Hz. depth_mode={DEPTH_MODE}")
    print("[perception] === READY === waiting for frames from io_bridge "
          "(focus the depth window, 'q' to quit).\n")

    last_infer_mono = 0.0
    n_depth = 0
    last_report = time.monotonic()
    window = "Cartographer — perception (depth + obstacles)"

    try:
        while True:
            got = frame_sub.recv(timeout_ms=500)
            if got is None:
                if show and (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
                continue
            frame, meta = got
            now = time.monotonic()
            if now - last_infer_mono < min_interval:
                # Throttle the GPU: newest frame already conflated, just keep UI live.
                if show and (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
                continue
            last_infer_mono = now

            t0 = time.time()
            depth_map = depth.infer(frame)
            infer_ms = (time.time() - t0) * 1000.0
            n_depth += 1

            payload, proximity, bars = build_payload(meta, depth_map, infer_ms, cadence_hz)
            state_pub.publish(frame_bus.TOPIC_DEPTH, payload)

            if now - last_report >= 1.0:
                c = meta.get("controls", {})
                print(f"[perception] depth {n_depth / (now - last_report):4.1f} Hz | "
                      f"infer {infer_ms:5.1f} ms | fwd_clear {payload['forward_clearance']:.2f} | "
                      f"raw med {payload['depth_stats']['median']:.2f} | "
                      f"trigger {c.get('trigger')} yaw {c.get('yaw')}")
                n_depth = 0
                last_report = now

            if show:
                telem = [
                    f"depth_mode={DEPTH_MODE}  infer={infer_ms:.0f}ms  cap~{cadence_hz:g}Hz",
                    f"fwd_clearance={payload['forward_clearance']:.2f}  "
                    f"raw[min/med/max]={payload['depth_stats']['min']:.1f}/"
                    f"{payload['depth_stats']['median']:.1f}/{payload['depth_stats']['max']:.1f}",
                ]
                cv2.imshow(window, render(frame, proximity, bars, telem))
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
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
    parser = argparse.ArgumentParser(description="Cartographer perception_worker (P2): DA-V2 depth")
    parser.add_argument("--config", default=None)
    parser.add_argument("--no-display", action="store_true", help="headless: skip the OpenCV window")
    parser.add_argument("--self-test", action="store_true",
                        help="run depth once on a test asset, save an overlay, exit (no bus/sim)")
    args = parser.parse_args()

    cfg = load_config(args.config)
    if args.self_test:
        run_self_test(cfg)
    else:
        run_live(cfg, show=not args.no_display)


if __name__ == "__main__":
    main()
