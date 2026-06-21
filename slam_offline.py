"""slam_offline.py — drive the FULL MASt3R-SLAM loop over a recorded flight mp4.

Milestone 4, step 1 (de-risk). The smoke test (`smoke_test_slam.py`) only proved
two-view inference. This script exercises the *complete* SLAM pipeline on Windows —
`FrameTracker` + `FactorGraph` global optimization + retrieval-based loop closure /
relocalization — exactly as `third_party/MASt3R-SLAM/main.py` does, but:

  * single process, **no visualization process** (the upstream viz needs pyimgui),
  * reads frames straight from an mp4 with OpenCV (avoids the dataloader's
    pyrealsense2 / torchcodec imports, which are not installed here),
  * on finish, exports the map for review: a confidence-filtered, RGB-colored world
    **point cloud (.ply + .npz)**, the **camera trajectory**, and a cv2-rendered
    **top-down (X-Z) map PNG**.

This validates the engine that M4 then wires into the live `perception_worker`. It is
*offline* on purpose: prove the map is globally consistent on a known flight first.

NO SILENT FALLBACKS (per CLAUDE.md): CUDA + every model/checkpoint load fails fast.
Tracking/reloc state (INIT/TRACKING/RELOC) is the engine's own explicit mode — logged
here, not silently absorbed. Runs uncalibrated (`use_calib: False`), reporting positions
in MASt3R's own self-consistent units (metric scale is NOT required by the brief).
"""

import argparse
import os
import sys
import time
from pathlib import Path

import threading

import cv2
import numpy as np
import torch
import lietorch

CARTO = Path(__file__).resolve().parent
SLAM_REPO = CARTO / "third_party" / "MASt3R-SLAM"

# The repo resolves checkpoints/ and config/ relatively, so run from its root.
os.chdir(SLAM_REPO)
sys.path.insert(0, str(SLAM_REPO))

from mast3r_slam.config import load_config, config  # noqa: E402
from mast3r_slam.frame import Mode, SharedKeyframes, SharedStates, create_frame  # noqa: E402
from mast3r_slam.mast3r_utils import (  # noqa: E402
    load_mast3r,
    load_retriever,
    mast3r_inference_mono,
)
from mast3r_slam.tracker import FrameTracker  # noqa: E402
from mast3r_slam.global_opt import FactorGraph  # noqa: E402

try:
    from plyfile import PlyData, PlyElement
    HAS_PLYFILE = True
except ImportError:
    HAS_PLYFILE = False


# ==============================================================================
# In-process stand-in for mp.Manager.
#
# SharedKeyframes / SharedStates were written for the upstream multi-process app
# (main loop + a separate viz process), so they ask a `manager` for RLock / Value /
# list. We run tracker + backend in ONE process (no viz), so real multiprocessing is
# unnecessary — and on Windows `mp.Manager()` spawns a child that re-imports this
# module (with its module-level os.chdir + heavy imports) and deadlocks. This shim
# satisfies the same tiny API in-process. The `.share_memory_()` CUDA tensors inside
# those classes are harmless no-ops when nothing else attaches.
# ==============================================================================
class _Value:
    def __init__(self, value):
        self.value = value


class InProcessManager:
    def RLock(self):
        return threading.RLock()

    def Value(self, typecode, value):
        return _Value(value)

    def list(self):
        return []


# ==============================================================================
# Backend (adapted from main.py — globals there become explicit args here)
# ==============================================================================
def relocalization(frame, keyframes, factor_graph, retrieval_database):
    with keyframes.lock:
        retrieval_inds = retrieval_database.update(
            frame, add_after_query=False,
            k=config["retrieval"]["k"], min_thresh=config["retrieval"]["min_thresh"],
        )
        kf_idx = list(retrieval_inds)
        successful_loop_closure = False
        if kf_idx:
            keyframes.append(frame)
            n_kf = len(keyframes)
            frame_idx = [n_kf - 1] * len(kf_idx)
            print(f"  [reloc] against kf {n_kf - 1} and {kf_idx}")
            if factor_graph.add_factors(
                frame_idx, kf_idx,
                config["reloc"]["min_match_frac"], is_reloc=config["reloc"]["strict"],
            ):
                retrieval_database.update(
                    frame, add_after_query=True,
                    k=config["retrieval"]["k"], min_thresh=config["retrieval"]["min_thresh"],
                )
                print("  [reloc] success")
                successful_loop_closure = True
                keyframes.T_WC[n_kf - 1] = keyframes.T_WC[kf_idx[0]].clone()
            else:
                keyframes.pop_last()
                print("  [reloc] failed")
        if successful_loop_closure:
            if config["use_calib"]:
                factor_graph.solve_GN_calib()
            else:
                factor_graph.solve_GN_rays()
        return successful_loop_closure


def run_backend(states, keyframes, factor_graph, retrieval_database):
    mode = states.get_mode()
    if mode == Mode.INIT or states.is_paused():
        return
    if mode == Mode.RELOC:
        frame = states.get_frame()
        success = relocalization(frame, keyframes, factor_graph, retrieval_database)
        if success:
            states.set_mode(Mode.TRACKING)
        states.dequeue_reloc()
        return

    idx = -1
    with states.lock:
        if len(states.global_optimizer_tasks) > 0:
            idx = states.global_optimizer_tasks[0]
    if idx == -1:
        return

    # Graph construction: previous consecutive keyframe + retrieval (loop closure).
    kf_idx = []
    n_consec = 1
    for j in range(min(n_consec, idx)):
        kf_idx.append(idx - 1 - j)
    frame = keyframes[idx]
    retrieval_inds = retrieval_database.update(
        frame, add_after_query=True,
        k=config["retrieval"]["k"], min_thresh=config["retrieval"]["min_thresh"],
    )
    kf_idx += retrieval_inds

    lc_inds = set(retrieval_inds)
    lc_inds.discard(idx - 1)
    if lc_inds:
        print(f"  [backend] db retrieval {idx}: loop-closure candidates {sorted(lc_inds)}")

    kf_idx = set(kf_idx)
    kf_idx.discard(idx)
    kf_idx = list(kf_idx)
    frame_idx = [idx] * len(kf_idx)
    if kf_idx:
        factor_graph.add_factors(kf_idx, frame_idx, config["local_opt"]["min_match_frac"])

    with states.lock:
        states.edges_ii[:] = factor_graph.ii.cpu().tolist()
        states.edges_jj[:] = factor_graph.jj.cpu().tolist()

    if config["use_calib"]:
        factor_graph.solve_GN_calib()
    else:
        factor_graph.solve_GN_rays()

    with states.lock:
        if len(states.global_optimizer_tasks) > 0:
            states.global_optimizer_tasks.pop(0)


# ==============================================================================
# Frame source
# ==============================================================================
def iter_frames(mp4_path, stride, max_frames):
    """Yield (timestamp_s, rgb_float01) from an mp4, sub-sampled by `stride`."""
    cap = cv2.VideoCapture(str(mp4_path))
    if not cap.isOpened():
        raise RuntimeError(f"Could not open recording: {mp4_path}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    src_idx, yielded = 0, 0
    while True:
        ret, bgr = cap.read()
        if not ret:
            break
        if src_idx % stride == 0:
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
            yield src_idx / fps, rgb
            yielded += 1
            if max_frames and yielded >= max_frames:
                break
        src_idx += 1
    cap.release()


# ==============================================================================
# Map export
# ==============================================================================
def extract_map(keyframes, conf_thresh, device):
    """Return (points Nx3, colors Nx3 uint8, trajectory Kx3) in world coords."""
    pts, cols, traj = [], [], []
    origin = torch.zeros(1, 3, device=device)
    for i in range(len(keyframes)):
        kf = keyframes[i]
        pW = kf.T_WC.act(kf.X_canon).cpu().numpy().reshape(-1, 3)
        color = (kf.uimg.cpu().numpy() * 255).astype(np.uint8).reshape(-1, 3)
        conf = kf.get_average_conf().cpu().numpy().astype(np.float32).reshape(-1)
        valid = conf > conf_thresh
        pts.append(pW[valid])
        cols.append(color[valid])
        traj.append(kf.T_WC.act(origin).cpu().numpy().reshape(3))
    points = np.concatenate(pts, axis=0) if pts else np.zeros((0, 3), np.float32)
    colors = np.concatenate(cols, axis=0) if cols else np.zeros((0, 3), np.uint8)
    trajectory = np.asarray(traj, dtype=np.float32) if traj else np.zeros((0, 3), np.float32)
    return points, colors, trajectory


def save_ply(filename, points, colors):
    if not HAS_PLYFILE:
        print("  [export] plyfile not installed — skipping .ply (npz still written)")
        return
    colors = colors.astype(np.uint8)
    pcd = np.empty(len(points), dtype=[
        ("x", "f4"), ("y", "f4"), ("z", "f4"),
        ("red", "u1"), ("green", "u1"), ("blue", "u1")])
    pcd["x"], pcd["y"], pcd["z"] = points.T
    pcd["red"], pcd["green"], pcd["blue"] = colors.T
    PlyData([PlyElement.describe(pcd, "vertex")], text=False).write(str(filename))
    print(f"  [export] point cloud -> {filename}  ({len(points)} pts)")


def render_topdown(points, colors, trajectory, out_path, size=900, pad=0.06):
    """Render an X-Z (ground-plane) top-down map with the camera trajectory overlaid.

    Camera convention: X right, Y down, Z forward => X-Z is the horizontal plane.
    Robust 1st/99th-percentile bounds keep a few outlier points from squashing it.
    """
    if len(points) == 0:
        print("  [export] no points to render top-down")
        return
    X, Z = points[:, 0], points[:, 2]
    xlo, xhi = np.percentile(X, 1), np.percentile(X, 99)
    zlo, zhi = np.percentile(Z, 1), np.percentile(Z, 99)
    span = max(xhi - xlo, zhi - zlo, 1e-6)
    cx, cz = (xlo + xhi) / 2, (zlo + zhi) / 2
    half = span * (0.5 + pad)

    def to_px(x, z):
        u = (x - (cx - half)) / (2 * half) * (size - 1)
        v = (z - (cz - half)) / (2 * half) * (size - 1)
        return np.clip(u, 0, size - 1).astype(int), np.clip(v, 0, size - 1).astype(int)

    img = np.full((size, size, 3), 18, np.uint8)
    u, v = to_px(X, Z)
    # color points by their RGB (BGR for cv2); v flipped so +Z reads "up"
    img[size - 1 - v, u] = colors[:, ::-1]

    if len(trajectory) > 1:
        tu, tv = to_px(trajectory[:, 0], trajectory[:, 2])
        path = np.stack([tu, size - 1 - tv], axis=1).astype(np.int32)
        cv2.polylines(img, [path], False, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.circle(img, tuple(path[0]), 6, (0, 255, 0), -1)   # start
        cv2.circle(img, tuple(path[-1]), 6, (0, 255, 255), -1)  # end

    cv2.putText(img, f"top-down X-Z  {len(points)} pts  {len(trajectory)} kf  "
                f"~{2 * half:.2f}u across", (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(img, "traj: green=start yellow=end (red path)", (10, size - 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 255), 1)
    cv2.imwrite(str(out_path), img)
    print(f"  [export] top-down map -> {out_path}")


# ==============================================================================
# Main
# ==============================================================================
def main():
    parser = argparse.ArgumentParser(description="Offline full MASt3R-SLAM over a flight mp4")
    parser.add_argument("--video", default=str(CARTO.parent / "XLAB" / "OUTPUT" /
                                               "flight_20260621_120829.mp4"))
    parser.add_argument("--config", default="config/base.yaml")
    parser.add_argument("--stride", type=int, default=3, help="process every Nth source frame")
    parser.add_argument("--max-frames", type=int, default=0, help="cap processed frames (0=all)")
    parser.add_argument("--conf-thresh", type=float, default=1.5,
                        help="per-point average-confidence cutoff for the exported cloud")
    parser.add_argument("--out", default=str(CARTO / "OUTPUT"))
    args = parser.parse_args()

    torch.backends.cuda.matmul.allow_tf32 = True
    torch.set_grad_enabled(False)
    device = "cuda:0"
    assert torch.cuda.is_available(), "CUDA required (no CPU fallback)."

    video = Path(args.video)
    assert video.exists(), f"recording not found: {video}"
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = video.stem

    load_config(args.config)
    config["use_calib"] = False
    print(f"[slam] video={video.name} stride={args.stride} "
          f"max_frames={args.max_frames or 'all'} device={device}")

    t0 = time.time()
    model = load_mast3r(device=device)
    print(f"[slam] MASt3R loaded {time.time() - t0:.1f}s | VRAM {torch.cuda.memory_allocated()/1e9:.2f} GB")
    t0 = time.time()
    retrieval_database = load_retriever(model)
    print(f"[slam] retriever loaded {time.time() - t0:.1f}s")

    # Determine the resized frame shape MASt3R works at, to size the shared buffers.
    frame_iter = iter_frames(video, args.stride, args.max_frames)
    first_ts, first_rgb = next(frame_iter)
    probe = create_frame(0, first_rgb, lietorch.Sim3.Identity(1, device=device),
                         img_size=512, device=device)
    h, w = int(probe.img_true_shape.flatten()[0]), int(probe.img_true_shape.flatten()[1])
    print(f"[slam] working resolution {w}x{h}")

    manager = InProcessManager()
    keyframes = SharedKeyframes(manager, h, w)
    states = SharedStates(manager, h, w)
    tracker = FrameTracker(model, keyframes, device)
    factor_graph = FactorGraph(model, keyframes, None, device)
    print(f"[slam] SLAM state initialized | VRAM {torch.cuda.memory_allocated()/1e9:.2f} GB", flush=True)

    timestamps = []
    n_proc = n_kf = n_reloc = 0
    loop_t0 = time.time()

    def step(i, ts, rgb):
        nonlocal n_kf, n_reloc
        timestamps.append(ts)
        mode = states.get_mode()
        T_WC = (lietorch.Sim3.Identity(1, device=device)
                if i == 0 else states.get_frame().T_WC)
        frame = create_frame(i, rgb, T_WC, img_size=512, device=device)

        if mode == Mode.INIT:
            X, C = mast3r_inference_mono(model, frame)
            frame.update_pointmap(X, C)
            keyframes.append(frame)
            states.queue_global_optimization(len(keyframes) - 1)
            states.set_mode(Mode.TRACKING)
            states.set_frame(frame)
            n_kf += 1
            return
        if mode == Mode.TRACKING:
            add_new_kf, _, try_reloc = tracker.track(frame)
            if try_reloc:
                states.set_mode(Mode.RELOC)
                n_reloc += 1
            states.set_frame(frame)
        elif mode == Mode.RELOC:
            X, C = mast3r_inference_mono(model, frame)
            frame.update_pointmap(X, C)
            states.set_frame(frame)
            states.queue_reloc()
            add_new_kf = False
        else:
            raise RuntimeError(f"invalid mode {mode}")

        if add_new_kf:
            keyframes.append(frame)
            states.queue_global_optimization(len(keyframes) - 1)
            n_kf += 1
        run_backend(states, keyframes, factor_graph, retrieval_database)

    try:
        # frame 0 was already pulled off the iterator for the shape probe
        step(0, first_ts, first_rgb)
        n_proc = 1
        for ts, rgb in frame_iter:
            step(n_proc, ts, rgb)
            n_proc += 1
            if n_proc % 30 == 0:
                fps = n_proc / (time.time() - loop_t0)
                print(f"[slam] {n_proc} frames | {len(keyframes)} kf | mode={Mode(states.get_mode()).name} "
                      f"| {fps:.1f} fps")
    except KeyboardInterrupt:
        print("[slam] interrupted — exporting what we have so far ...")

    dt = time.time() - loop_t0
    print(f"\n[slam] DONE: {n_proc} frames in {dt:.1f}s ({n_proc/max(dt,1e-6):.1f} fps) "
          f"| {len(keyframes)} keyframes | reloc events {n_reloc}")
    print(f"[slam] peak VRAM {torch.cuda.max_memory_allocated()/1e9:.2f} GB")

    print("[slam] extracting map ...")
    points, colors, trajectory = extract_map(keyframes, args.conf_thresh, device)
    print(f"[slam] {len(points)} world points (conf>{args.conf_thresh}) | "
          f"trajectory {len(trajectory)} keyframes")

    np.savez(out_dir / f"{stem}_map.npz",
             points=points, colors=colors, trajectory=trajectory)
    print(f"  [export] arrays -> {out_dir / f'{stem}_map.npz'}")
    save_ply(out_dir / f"{stem}_cloud.ply", points, colors)
    render_topdown(points, colors, trajectory, out_dir / f"{stem}_topdown.png")
    print("\n[slam] OK")


if __name__ == "__main__":
    main()
