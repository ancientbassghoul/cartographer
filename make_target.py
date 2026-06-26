"""make_target.py — designate the Phase-1 target: crop it, auto-classify it, write target.yaml.

The cascade detector (`object_worker.py`) grounds a designated target described by THREE things:
a reference crop, a GroundingDINO text phrase, and an AssetClass (2D_PLANAR vs 3D_GEOMETRY). This
tool produces all three from a recon recording (the flight you just flew):

  1. Browse frames (drone feed at transport resolution, 512x288 — what the workers see).
  2. On the frame that best shows your target, drag a box around it. The crop is taken from the
     FULL-RESOLUTION frame (maximum fidelity for OWLv2/DINOv2), not the downscaled preview.
  3. A one-time Qwen2.5-VL pass auto-suggests the text phrase + asset class; you confirm or edit
     each in the terminal (you can always override the classification).
  4. It writes `target.yaml` {reference_crop, text, asset_class} (loaded by object_worker) and saves
     the full scene frame as `test_assets/target_scene.png` for `object_worker --self-test`.

Qwen loads here for the ONE classification pass only (designation), never during a flight.

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
TARGET_FILE = REPO / "target.yaml"
BROWSE_WINDOW = "make_target — browse (n/p step, N/P jump, ENTER pick, q quit)"
VALID_CLASSES = ("2D_PLANAR", "3D_GEOMETRY")


def load_config(path=None):
    with open(path or REPO / "config.yaml", "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def newest_recording(cfg):
    rec_dir = (REPO / cfg["simulator"]["recordings_dir"]).resolve()
    mp4s = sorted(rec_dir.glob("*.mp4"), key=lambda p: p.stat().st_mtime)
    if not mp4s:
        raise SystemExit(f"no .mp4 recordings in {rec_dir} — fly a recon with io_bridge first.")
    return mp4s[-1]


def confirm_or_edit(field, suggestion, validate=None):
    """Print a suggestion and let the user accept (Enter) or type a replacement. Re-prompts until a
    validated value is given. Returns the chosen string."""
    while True:
        ans = input(f"  {field}: suggested = {suggestion!r}  [Enter=accept / type new]: ").strip()
        value = suggestion if ans == "" else ans
        if validate is None or validate(value):
            return value
        print(f"    invalid — expected one of {VALID_CLASSES}.")


def main():
    ap = argparse.ArgumentParser(description="Designate + classify the Phase-1 target from a recon recording")
    ap.add_argument("--video", default=None, help="recording mp4 (default: newest in recordings_dir)")
    ap.add_argument("--frame", type=int, default=None, help="start at this frame index")
    ap.add_argument("--config", default=None)
    args = ap.parse_args()

    cfg = load_config(args.config)
    proc_w = int(cfg["perception"]["processing_width"])
    proc_h = int(cfg["perception"]["processing_height"])
    # The crop is written here (object_worker's target.yaml references it).
    ref_rel = cfg["models"]["qwen_vl"]["reference_crop"]
    ref_path = ref_rel if os.path.isabs(ref_rel) else str(REPO / ref_rel)
    scene_path = str(REPO / "test_assets" / "target_scene.png")

    video = Path(args.video).resolve() if args.video else newest_recording(cfg)
    assert video.exists(), f"recording not found: {video}"
    cap = cv2.VideoCapture(str(video))
    if not cap.isOpened():
        raise SystemExit(f"could not open {video}")
    n_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    native_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    native_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[make_target] {video.name}: {n_frames} frames | native {native_w}x{native_h} "
          f"| preview {proc_w}x{proc_h}")
    print("[make_target] browse: n/p = +/-1, N/P = +/-15, ENTER = pick this frame, q = quit")

    idx = args.frame if args.frame is not None else n_frames // 2
    idx = max(0, min(idx, n_frames - 1))

    def read_native(i):
        cap.set(cv2.CAP_PROP_POS_FRAMES, i)
        ok, bgr = cap.read()
        return bgr if ok else None

    picked_native = None
    while True:
        native = read_native(idx)
        if native is None:
            idx = max(0, idx - 1); continue
        preview = cv2.resize(native, (proc_w, proc_h))
        disp = preview.copy()
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
            picked_native = native
            break

    cv2.destroyWindow(BROWSE_WINDOW)
    if picked_native is None:
        print("[make_target] no frame picked — nothing saved.")
        cap.release()
        return
    cap.release()

    # Drag the box on the transport-resolution preview, then map it up to NATIVE pixels and crop the
    # full-res frame (image-integrity: maximize source fidelity before the crop enters the models).
    preview = cv2.resize(picked_native, (proc_w, proc_h))
    roi = cv2.selectROI("make_target — drag a box around the TARGET, then ENTER",
                        preview, showCrosshair=True, fromCenter=False)
    cv2.destroyAllWindows()
    x, y, w, h = [int(v) for v in roi]
    if w < 4 or h < 4:
        print("[make_target] box too small / cancelled — nothing saved.")
        return
    ph, pw = picked_native.shape[:2]
    sx, sy = pw / proc_w, ph / proc_h
    nx1, ny1 = int(round(x * sx)), int(round(y * sy))
    nx2, ny2 = int(round((x + w) * sx)), int(round((y + h) * sy))
    nx1, nx2 = max(0, nx1), min(pw, nx2)
    ny1, ny2 = max(0, ny1), min(ph, ny2)
    crop = picked_native[ny1:ny2, nx1:nx2]

    os.makedirs(os.path.dirname(ref_path), exist_ok=True)
    cv2.imwrite(ref_path, crop)
    cv2.imwrite(scene_path, picked_native)       # native-res scene for object_worker --self-test
    print(f"\n[make_target] target crop  ({crop.shape[1]}x{crop.shape[0]}, native)  -> {ref_path}")
    print(f"[make_target] target scene (full frame {pw}x{ph}) -> {scene_path}")

    # --- auto-classify (Qwen-VL, one-time) -> suggest label + asset class, user confirms/overrides
    print("\n[make_target] classifying target (Qwen-VL, one-time) ...", flush=True)
    from target_classifier import TargetClassifier
    q = cfg["models"]["qwen_vl"]
    clf = TargetClassifier(hf_id=q["hf_id"], quantization=q.get("quantization", "4bit"))
    res = clf.classify(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))
    clf.close()
    print(f"\n[make_target] Qwen suggests: label={res['label']!r}  class={res['asset_class']}")
    print(f"[make_target]   (raw: {res['raw']!r})")
    print("[make_target] Confirm or override (Enter accepts the suggestion):")
    text = confirm_or_edit("text  ", res["label"])
    asset_class = confirm_or_edit("class ", res["asset_class"],
                                  validate=lambda v: v.strip().upper() in VALID_CLASSES)
    asset_class = asset_class.strip().upper()

    spec = {"reference_crop": ref_rel, "text": text, "asset_class": asset_class}
    with open(TARGET_FILE, "w", encoding="utf-8") as f:
        yaml.safe_dump(spec, f, sort_keys=False, default_flow_style=False)
    print(f"\n[make_target] wrote {TARGET_FILE}:")
    print(f"    reference_crop: {ref_rel}")
    print(f"    text:           {text!r}")
    print(f"    asset_class:    {asset_class}")
    print("\nNext:")
    print("  1) Validate detection on your target:")
    print("       venv\\Scripts\\python.exe object_worker.py --self-test")
    print("  2) Then do the live 4-process run (see PROGRESS.md launch procedure).")


if __name__ == "__main__":
    main()
