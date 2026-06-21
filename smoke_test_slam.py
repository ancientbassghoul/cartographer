"""SLAM smoke test: load MASt3R model on GPU and run a real two-view inference on
two XLAB frames, exercising the freshly compiled mast3r_slam_backends + lietorch.

Validates the Windows build end-to-end and that the core SLAM stack runs against
numpy 2.4.4 (rather than the pinned 1.26.4). Per NO-SILENT-FALLBACKS: fails fast.

Run from cartographer/:
    venv\\Scripts\\python.exe smoke_test_slam.py
"""
import os
import sys
import time
from pathlib import Path

CARTO = Path(__file__).resolve().parent
REPO = CARTO / "third_party" / "MASt3R-SLAM"
ASSETS = CARTO / "test_assets"

# The repo uses relative paths for checkpoints/ and config/, so run from its root.
os.chdir(REPO)
sys.path.insert(0, str(REPO))

import cv2
import numpy as np
import torch
import lietorch

from mast3r_slam.config import load_config, config
from mast3r_slam.mast3r_utils import load_mast3r, mast3r_symmetric_inference
from mast3r_slam.frame import create_frame


def load_rgb(path: Path) -> np.ndarray:
    bgr = cv2.imread(str(path))
    assert bgr is not None, f"could not read {path}"
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0


def main() -> int:
    print(f"torch {torch.__version__} | numpy {np.__version__} | cuda {torch.cuda.is_available()}",
          flush=True)
    assert torch.cuda.is_available(), "CUDA required"

    load_config("config/base.yaml")
    print("config loaded", flush=True)

    t0 = time.time()
    model = load_mast3r(device="cuda")
    model.eval()
    print(f"MASt3R checkpoint loaded in {time.time() - t0:.1f}s | "
          f"VRAM {torch.cuda.memory_allocated() / 1e9:.2f} GB", flush=True)

    img_a = load_rgb(ASSETS / "frame_a.png")
    img_b = load_rgb(ASSETS / "frame_b.png")
    T_WC = lietorch.Sim3.Identity(1, device="cuda")
    frame_a = create_frame(0, img_a, T_WC, img_size=512, device="cuda")
    frame_b = create_frame(1, img_b, T_WC, img_size=512, device="cuda")
    print(f"frames built | true_shape {frame_a.img_true_shape.tolist()}", flush=True)

    t0 = time.time()
    X, C, D, Q = mast3r_symmetric_inference(model, frame_a, frame_b)
    torch.cuda.synchronize()
    dt = time.time() - t0
    print(f"two-view inference {dt:.2f}s ({1/dt:.1f} pair/s)", flush=True)
    print(f"pointmap X {tuple(X.shape)} | conf C {tuple(C.shape)} "
          f"min {C.min().item():.2f} max {C.max().item():.2f}", flush=True)
    print(f"X range x[{X[...,0].min():.2f},{X[...,0].max():.2f}] "
          f"z[{X[...,2].min():.2f},{X[...,2].max():.2f}]", flush=True)
    print(f"peak VRAM {torch.cuda.max_memory_allocated() / 1e9:.2f} GB", flush=True)
    print("\n====================  SLAM SMOKE TEST: PASS  ====================", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
