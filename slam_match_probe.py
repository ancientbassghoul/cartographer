"""slam_match_probe.py — isolate which matching CUDA kernel segfaults on Windows.

track() crashes (access violation) in mast3r_match_asymmetric -> matching.match. That
path runs two custom kernels: mast3r_slam_backends.iter_proj and (if radius>0)
refine_matches. This probe reproduces the exact match on two XLAB frames, bracketing
each kernel, so we learn which one faults (and whether radius=0 dodges it).
"""
import os
import sys
import time
from pathlib import Path

CARTO = Path(__file__).resolve().parent
SLAM_REPO = CARTO / "third_party" / "MASt3R-SLAM"
ASSETS = CARTO / "test_assets"
os.chdir(SLAM_REPO)
sys.path.insert(0, str(SLAM_REPO))

import cv2
import numpy as np
import torch
import torch.nn.functional as F
import lietorch

from mast3r_slam.config import load_config, config
from mast3r_slam.frame import create_frame
from mast3r_slam.mast3r_utils import load_mast3r, mast3r_asymmetric_inference
import mast3r_slam.image as img_utils
from mast3r_slam.matching import prep_for_iter_proj, lin_to_pixel, pixel_to_lin
import mast3r_slam_backends


def load_rgb(path):
    bgr = cv2.imread(str(path))
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0


def main():
    p = print
    load_config("config/base.yaml")
    config["use_calib"] = False
    model = load_mast3r(device="cuda")
    model.eval()
    p(f"[probe] model loaded | matching cfg = {config['matching']}", flush=True)

    T = lietorch.Sim3.Identity(1, device="cuda")
    fa = create_frame(0, load_rgb(ASSETS / "frame_a.png"), T, img_size=512, device="cuda")
    fb = create_frame(1, load_rgb(ASSETS / "frame_b.png"), T, img_size=512, device="cuda")

    p("[probe] asymmetric inference ...", flush=True)
    X, C, D, Q = mast3r_asymmetric_inference(model, fa, fb)
    b, h, w = X.shape[:-1]
    b = b // 2
    Xii, Xji = X[:b], X[b:]
    Dii, Dji = D[:b], D[b:]
    p(f"[probe] inference ok | X {tuple(X.shape)} D {tuple(D.shape)} | b={b} h={h} w={w}", flush=True)

    cfg = config["matching"]
    rays_with_grad_img, pts3d_norm, p_init = prep_for_iter_proj(Xii, Xji, None)
    p(f"[probe] prep ok | rays {tuple(rays_with_grad_img.shape)} pts {tuple(pts3d_norm.shape)} "
      f"p_init {tuple(p_init.shape)} {p_init.dtype}", flush=True)

    torch.cuda.synchronize()
    p("[probe] CALLING iter_proj ...", flush=True)
    t0 = time.time()
    p1, valid_proj2 = mast3r_slam_backends.iter_proj(
        rays_with_grad_img, pts3d_norm, p_init,
        cfg["max_iter"], cfg["lambda_init"], cfg["convergence_thresh"],
    )
    torch.cuda.synchronize()
    p(f"[probe] iter_proj OK in {time.time()-t0:.3f}s | p1 {tuple(p1.shape)} {p1.dtype}", flush=True)
    p1 = p1.long()

    if cfg["radius"] > 0:
        torch.cuda.synchronize()
        p(f"[probe] CALLING refine_matches (radius={cfg['radius']}, dilation={cfg['dilation_max']}) ...",
          flush=True)
        t0 = time.time()
        (p1r,) = mast3r_slam_backends.refine_matches(
            Dii.half(), Dji.view(b, h * w, -1).half(), p1, cfg["radius"], cfg["dilation_max"],
        )
        torch.cuda.synchronize()
        p(f"[probe] refine_matches OK in {time.time()-t0:.3f}s | p1r {tuple(p1r.shape)}", flush=True)

    p("[probe] ALL MATCHING KERNELS PASSED", flush=True)


if __name__ == "__main__":
    main()
