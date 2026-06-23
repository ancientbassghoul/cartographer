"""make_target.py — designate the Phase-1 target by cropping it from a recon recording.

The object chain (`object_worker.py`) grounds a **provided reference crop**. This helper lets
you pick that crop interactively from a flight recording (the recon you just flew):

  1. Browse frames (the drone feed at transport resolution, 512x288 — exactly what the workers
     see) with the keyboard.
  2. On the frame that best shows your target, drag a box around it.
  3. It saves the crop to `models.qwen_vl.reference_crop` (the asset object_worker loads) AND
     saves that full frame as `test_assets/target_scene.png` so `object_worker --self-test`
     can immediately validate detection on your real target.

It does NOT touch the GPU / models. Browsing keys are printed at startup.

Example:
    venv\\Scripts\\python.exe make_target.py --video ../XLAB/OUTPUT/flight_XXXX.mp4
    venv\\Scripts\\python.exe make_target.py            # uses the newest mp4 in recordings_dir
"""

import argparse
import os
from pathlib import Path

import cv2
import yaml

REPO = Path(__file__).resolve().parent
BROWSE_WINDOW = "make_target — browse (n/p step, N/P jump, ENTER pick, q quit)"


def load_config(path=None):
    with open(path or REPO / "config.yaml", "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def newest_recording(cfg):
    rec_dir = (REPO / cfg["simulator"]["recordings_dir"]).resolve()
    mp4s = sorted(rec_dir.glob("*.mp4"), key=lambda p: p.stat().st_mtime)
    if not mp4s:
        raise SystemExit(f"no .mp4 recordings in {rec_dir} — fly a recon with io_bridge first.")
    return mp4s[-1]


def main():
    ap = argparse.ArgumentParser(description="Crop the Phase-1 target from a recon recording")
    ap.add_argument("--video", default=None, help="recording mp4 (default: newest in recordings_dir)")
    ap.add_argument("--frame", type=int, default=None, help="start at this frame index")
    ap.add_argument("--config", default=None)
    args = ap.parse_args()

    cfg = load_config(args.config)
    proc_w = int(cfg["perception"]["processing_width"])
    proc_h = int(cfg["perception"]["processing_height"])
    ref_rel = cfg["models"]["qwen_vl"]["reference_crop"]
    ref_path = ref_rel if os.path.isabs(ref_rel) else str(REPO / ref_rel)
    scene_path = str(REPO / "test_assets" / "target_scene.png")

    video = Path(args.video).resolve() if args.video else newest_recording(cfg)
    assert video.exists(), f"recording not found: {video}"
    cap = cv2.VideoCapture(str(video))
    if not cap.isOpened():
        raise SystemExit(f"could not open {video}")
    n_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f"[make_target] {video.name}: {n_frames} frames @ {proc_w}x{proc_h}")
    print("[make_target] browse: n/p = +/-1, N/P = +/-15, ENTER = pick this frame, q = quit")

    idx = args.frame if args.frame is not None else n_frames // 2
    idx = max(0, min(idx, n_frames - 1))

    def read(i):
        cap.set(cv2.CAP_PROP_POS_FRAMES, i)
        ok, bgr = cap.read()
        return cv2.resize(bgr, (proc_w, proc_h)) if ok else None

    picked = None
    while True:
        frame = read(idx)
        if frame is None:
            idx = max(0, idx - 1); continue
        disp = frame.copy()
        cv2.putText(disp, f"frame {idx}/{n_frames-1}", (6, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.imshow(BROWSE_WINDOW, disp)
        k = cv2.waitKey(0) & 0xFF
        if k in (ord("q"), 27):
            break
        elif k == ord("n"):
            idx = min(n_frames - 1, idx + 1)
        elif k == ord("p"):
            idx = max(0, idx - 1)
        elif k == ord("N"):
            idx = min(n_frames - 1, idx + 15)
        elif k == ord("P"):
            idx = max(0, idx - 15)
        elif k in (13, 10):  # ENTER
            picked = frame
            break

    cv2.destroyWindow(BROWSE_WINDOW)
    if picked is None:
        print("[make_target] no frame picked — nothing saved.")
        cap.release()
        return

    roi = cv2.selectROI("make_target — drag a box around the TARGET, then ENTER",
                        picked, showCrosshair=True, fromCenter=False)
    cv2.destroyAllWindows()
    x, y, w, h = [int(v) for v in roi]
    if w < 4 or h < 4:
        print("[make_target] box too small / cancelled — nothing saved.")
        cap.release()
        return

    crop = picked[y:y + h, x:x + w]
    os.makedirs(os.path.dirname(ref_path), exist_ok=True)
    cv2.imwrite(ref_path, crop)
    cv2.imwrite(scene_path, picked)
    cap.release()

    print(f"\n[make_target] target crop  ({w}x{h})  -> {ref_path}")
    print(f"[make_target] target scene (full frame) -> {scene_path}")
    print("[make_target] NOTE: models.qwen_vl.target_label is auto-derived from this crop at "
          "startup; set it explicitly in config.yaml if you want a specific label.")
    print("\nNext:")
    print("  1) Validate detection on your target:")
    print("       venv\\Scripts\\python.exe object_worker.py --self-test")
    print("  2) Then do the live 4-process run (see PROGRESS.md launch procedure).")


if __name__ == "__main__":
    main()
