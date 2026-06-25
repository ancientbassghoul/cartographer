"""benchmark_detectors.py — pick the detection engine(s) for the object chain.

Per PROGRESS.md ("⛔ ALL learned-detector attempts FAILED → benchmarking matching tools"), every
prior detector (Qwen-4bit, OWLv2-as-gate, the Qwen→OWLv2 cascade) failed on the SAME problem:
DISCRIMINATION — firing on the wrong object (murals, a framed photo, the poster-through-glass).
This script benchmarks four visual-matching strategies on TWO target types so we can choose the
engine(s) on evidence, then rewrite ONLY object_worker.py's detector around the winner:

  * sift       — cv2 SIFT + ratio-test matches + RANSAC homography (the deterministic baseline;
                 PROGRESS.md: 40-67 inliers on clear poster views, 0 on the window/sign).
  * lightglue  — SuperPoint local features + LightGlue matcher + RANSAC ("SIFT but better").
  * dinov2     — semantic dense correspondence (ViT-S/14 patch tokens; best for 3D / distant /
                 through-glass where local features die).
  * owlv2      — OWLv2 image-guided one-shot detection (baseline; reuses object_worker's path).

It scores each engine on each target's POSITIVE frames (must find + land on the GT box) and on the
shared NONE negatives (must NOT fire). The decisive output is the pos-vs-neg score separation per
engine — that is what "discrimination" means here.

GPU-only (no CPU fallback, per CLAUDE.md NO SILENT FALLBACKS). Engines are loaded ONE AT A TIME and
freed, so peak VRAM stays small. First run downloads weights (DINOv2 via torch.hub, OWLv2 via HF,
SuperPoint/LightGlue via their package) — needs internet once; cached thereafter.

Layout it expects (already built by the user with io_bridge 'space' capture + annotate_targets.py):
  test_assets/Nasrallah_ref.png      test_assets/Nasrallah/*.png + labels.json
  test_assets/Rifle_ref.png          test_assets/Rifle/*.png     + labels.json
  test_assets/None/*.png             (negatives — target-absent; no labels)

Examples:
  venv\\Scripts\\python.exe benchmark_detectors.py                       # all engines, both targets
  venv\\Scripts\\python.exe benchmark_detectors.py --engines sift,dinov2 # subset
  venv\\Scripts\\python.exe benchmark_detectors.py --no-overlays
"""

import argparse
import json
import os
import time
from pathlib import Path

import cv2
import numpy as np
import torch
import yaml

REPO = Path(__file__).resolve().parent
IMG_EXTS = {".png", ".jpg", ".jpeg", ".bmp"}

# --- target definitions (ref crop + positive folder); negatives are shared ---
TARGETS = {
    "Nasrallah": {"ref": "test_assets/Nasrallah_ref.png", "dir": "test_assets/Nasrallah"},
    "Rifle":     {"ref": "test_assets/Rifle_ref.png",     "dir": "test_assets/Rifle"},
}
NEG_DIR = "test_assets/None"
LABELS_NAME = "labels.json"

# Language hints for --mask-refs (Grounded-SAM template onboarding). Keyed by target name.
REF_TEXT = {
    "Nasrallah": "a printed portrait poster",
    "Rifle": "a rifle",
}

# --- decision criterion -------------------------------------------------------
CENTER_IN_BOX = True   # a "correct" detection = predicted center falls inside the GT box
IOU_REPORT = 0.5       # also report how many founds reach this IoU (informational)

# --- per-engine thresholds (TUNABLE; the summary prints pos/neg score spreads so you can retune) ---
SIFT_MIN_INLIERS = 12      # RANSAC inliers to call it a find (clear poster ~40-67, distractor ~0)
SIFT_RATIO = 0.75          # Lowe ratio test
LG_MIN_INLIERS = 12        # RANSAC inliers on LightGlue correspondences
DINO_SIM_THRESH = 0.55     # max dense cosine similarity (frame patch -> nearest ref patch)
OWLV2_LOGIT_THRESH = 5.0   # OWLv2 found-gate: ABS image-guided logit out.logits[0,:,0].max() (default
                           #   from config models.owlv2.verify_thresh; dev poster ~7, murals/sign ~3.7).
                           #   The post-processed score self-normalizes to ~1.0 on every frame, so it
                           #   CANNOT gate (forces FP=1.0 on empty rooms) — kept only as norm_top_score.


SHAPE_DEBUG = False        # set by --debug-shapes: print tensor shapes at the forward pass


# ==============================================================================
# Dataset
# ==============================================================================
def list_images(folder: Path):
    return sorted(p for p in folder.iterdir() if p.is_file() and p.suffix.lower() in IMG_EXTS)


def load_rgb(path: Path):
    bgr = cv2.imread(str(path))
    if bgr is None:
        raise SystemExit(f"could not read image: {path}")
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def load_dataset():
    """Return refs{name->rgb}, positives{name->[(fname,rgb,gtbox)]}, negatives[(fname,rgb)]."""
    refs, positives = {}, {}
    for name, spec in TARGETS.items():
        ref_p = REPO / spec["ref"]
        if not ref_p.exists():
            raise SystemExit(f"missing reference crop for {name}: {ref_p}")
        refs[name] = load_rgb(ref_p)

        fdir = REPO / spec["dir"]
        labels = {}
        lp = fdir / LABELS_NAME
        if lp.exists():
            labels = json.loads(lp.read_text(encoding="utf-8"))
        items = []
        for f in list_images(fdir):
            gt = labels.get(f.name)
            gt = [int(v) for v in gt] if isinstance(gt, list) and len(gt) == 4 else None
            items.append((f.name, load_rgb(f), gt))
        n_gt = sum(1 for _, _, g in items if g is not None)
        if n_gt < len(items):
            print(f"[bench] WARNING: {name}: {len(items)-n_gt}/{len(items)} frames have no GT box "
                  f"(localization can't be scored on those).")
        positives[name] = items

    negs = [(f.name, load_rgb(f)) for f in list_images(REPO / NEG_DIR)]
    return refs, positives, negs


# ==============================================================================
# Geometry helpers
# ==============================================================================
def center_in_box(center, box):
    if center is None or box is None:
        return False
    cx, cy = center
    return box[0] <= cx <= box[2] and box[1] <= cy <= box[3]


def iou(a, b):
    if a is None or b is None:
        return 0.0
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    if inter == 0:
        return 0.0
    area_a = max(0, a[2] - a[0]) * max(0, a[3] - a[1])
    area_b = max(0, b[2] - b[0]) * max(0, b[3] - b[1])
    return inter / float(area_a + area_b - inter)


def bbox_of_points(pts):
    """Axis-aligned bbox + median center of a set of frame-pixel keypoints (robust for 3D)."""
    if pts is None or len(pts) == 0:
        return None, None
    xs, ys = pts[:, 0], pts[:, 1]
    box = [float(xs.min()), float(ys.min()), float(xs.max()), float(ys.max())]
    center = [float(np.median(xs)), float(np.median(ys))]
    return box, center


# ==============================================================================
# Engine: SIFT (cv2) — ratio-test matches + RANSAC homography, inliers = confidence
# ==============================================================================
class SiftDetector:
    name = "sift"
    uses_gpu = False

    def __init__(self, **_):
        self.sift = cv2.SIFT_create()
        self.matcher = cv2.BFMatcher(cv2.NORM_L2)
        print("[bench] SIFT ready (cv2).", flush=True)

    def _kp(self, rgb):
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        return self.sift.detectAndCompute(gray, None)

    def detect(self, ref_rgb, frame_rgb):
        t0 = time.time()
        kp0, des0 = self._kp(ref_rgb)
        kp1, des1 = self._kp(frame_rgb)
        out = {"found": False, "score": 0.0, "bbox": None, "center": None}
        if des0 is None or des1 is None or len(kp0) < 2 or len(kp1) < 2:
            out["infer_ms"] = (time.time() - t0) * 1e3
            return out
        good = []
        for m_n in self.matcher.knnMatch(des0, des1, k=2):
            if len(m_n) == 2 and m_n[0].distance < SIFT_RATIO * m_n[1].distance:
                good.append(m_n[0])
        inliers = 0
        if len(good) >= 4:
            src = np.float32([kp0[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
            dst = np.float32([kp1[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
            H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)
            if mask is not None:
                mask = mask.ravel().astype(bool)
                inliers = int(mask.sum())
                box, center = bbox_of_points(dst.reshape(-1, 2)[mask])
                out["bbox"], out["center"] = box, center
        out["score"] = float(inliers)
        out["found"] = inliers >= SIFT_MIN_INLIERS
        out["infer_ms"] = (time.time() - t0) * 1e3
        return out


# ==============================================================================
# Engine: SuperPoint + LightGlue — learned local features + matcher, RANSAC inliers
# ==============================================================================
class LightGlueDetector:
    name = "lightglue"
    uses_gpu = True

    def __init__(self, device="cuda", **_):
        from lightglue import LightGlue, SuperPoint
        self.device = device
        t0 = time.time()
        self.extractor = SuperPoint(max_num_keypoints=2048).eval().to(device)
        self.matcher = LightGlue(features="superpoint").eval().to(device)
        print(f"[bench] SuperPoint+LightGlue ready in {time.time()-t0:.1f}s "
              f"| VRAM {torch.cuda.memory_allocated()/1e9:.2f} GB", flush=True)

    def _to_t(self, rgb):
        t = torch.from_numpy(rgb).float().permute(2, 0, 1) / 255.0
        return t.to(self.device)

    def detect(self, ref_rgb, frame_rgb):
        from lightglue.utils import rbd
        t0 = time.time()
        out = {"found": False, "score": 0.0, "bbox": None, "center": None}
        with torch.inference_mode():
            f0 = self.extractor.extract(self._to_t(ref_rgb))
            f1 = self.extractor.extract(self._to_t(frame_rgb))
            m = self.matcher({"image0": f0, "image1": f1})
        f0, f1, m = rbd(f0), rbd(f1), rbd(m)
        matches = m["matches"].cpu().numpy()
        inliers = 0
        if len(matches) >= 4:
            k0 = f0["keypoints"].cpu().numpy()[matches[:, 0]]
            k1 = f1["keypoints"].cpu().numpy()[matches[:, 1]]
            H, mask = cv2.findHomography(k0.reshape(-1, 1, 2), k1.reshape(-1, 1, 2), cv2.RANSAC, 5.0)
            if mask is not None:
                mask = mask.ravel().astype(bool)
                inliers = int(mask.sum())
                box, center = bbox_of_points(k1[mask])
                out["bbox"], out["center"] = box, center
        out["score"] = float(inliers)
        out["found"] = inliers >= LG_MIN_INLIERS
        out["infer_ms"] = (time.time() - t0) * 1e3
        return out


# ==============================================================================
# Engine: DINOv2 — semantic dense correspondence (frame patch -> nearest ref patch cosine)
# ==============================================================================
class DinoV2Detector:
    name = "dinov2"
    uses_gpu = True
    PATCH = 14
    FRAME_W = 1288     # 92*14 — near-native 720p width (was a starved 700; see --debug-shapes)
    FRAME_H = 728      # 52*14 — near-native 720p height -> dense 92x52 = 4784-patch grid
    MEAN = (0.485, 0.456, 0.406)
    STD = (0.229, 0.224, 0.225)

    def __init__(self, device="cuda", **_):
        self.device = device
        t0 = time.time()
        self.model = torch.hub.load("facebookresearch/dinov2", "dinov2_vits14",
                                    trust_repo=True).to(device).eval()
        self._mean = torch.tensor(self.MEAN, device=device).view(1, 3, 1, 1)
        self._std = torch.tensor(self.STD, device=device).view(1, 3, 1, 1)
        self._coord_logged = False        # one-time coordinate-reconstruction diagnostic
        print(f"[bench] DINOv2 (vits14) ready in {time.time()-t0:.1f}s "
              f"| VRAM {torch.cuda.memory_allocated()/1e9:.2f} GB", flush=True)
        # IMAGE INTEGRITY disclosure (CLAUDE.md): state the input resolution transform explicitly.
        print(f"[bench]   DINOv2 input transform: frame -> resize {self.FRAME_W}x{self.FRAME_H} "
              f"(~native 720p, x14-aligned); ref -> NATIVE pixels + zero-pad to x14 (aspect preserved).",
              flush=True)

    def _ceil14(self, v):
        return max(self.PATCH, ((int(v) + self.PATCH - 1) // self.PATCH) * self.PATCH)

    def _prep_ref(self, rgb):
        # Aspect-EXACT: keep the template's NATIVE pixels (no stretch/square), zero-pad bottom/right
        # to the next x14 multiple so visual tokens retain true geometric proportions.
        h, w = rgb.shape[:2]
        nh, nw = self._ceil14(h), self._ceil14(w)
        padded = np.zeros((nh, nw, 3), dtype=rgb.dtype)
        padded[:h, :w] = rgb
        if SHAPE_DEBUG:
            print(f"[shapes][dinov2] ref   native(HxWx3)={tuple(rgb.shape)} -> zero-padded "
                  f"tensor=(1, 3, {nh}, {nw}) [aspect preserved]  patch grid={nw//self.PATCH}x{nh//self.PATCH}",
                  flush=True)
        return padded

    def _prep_frame(self, rgb):
        small = cv2.resize(rgb, (self.FRAME_W, self.FRAME_H), interpolation=cv2.INTER_AREA)
        if SHAPE_DEBUG:
            print(f"[shapes][dinov2] frame input(HxWx3)={tuple(rgb.shape)} -> resized "
                  f"tensor=(1, 3, {self.FRAME_H}, {self.FRAME_W})  patch grid={self.FRAME_W//self.PATCH}x{self.FRAME_H//self.PATCH}",
                  flush=True)
        return small

    def _tokens(self, img):
        nh, nw = img.shape[:2]
        t = torch.from_numpy(img).float().permute(2, 0, 1).unsqueeze(0).to(self.device) / 255.0
        t = (t - self._mean) / self._std
        with torch.inference_mode():
            feats = self.model.forward_features(t)["x_norm_patchtokens"][0]  # (N, D)
        feats = torch.nn.functional.normalize(feats, dim=1)
        return feats, nw // self.PATCH, nh // self.PATCH, nw, nh

    def detect(self, ref_rgb, frame_rgb):
        t0 = time.time()
        out = {"found": False, "score": 0.0, "bbox": None, "center": None}
        ref_f, _, _, _, _ = self._tokens(self._prep_ref(ref_rgb))
        frm_f, gw, gh, nw, nh = self._tokens(self._prep_frame(frame_rgb))
        # For each frame patch, cosine sim to its NEAREST ref patch (part-based; robust to 3D pose).
        sim = (frm_f @ ref_f.t()).max(dim=1).values        # (N,)
        sim_map = sim.view(gh, gw)
        max_sim = float(sim_map.max())
        out["score"] = max_sim
        if max_sim >= DINO_SIM_THRESH:
            out["found"] = True
            sx, sy = frame_rgb.shape[1] / nw, frame_rgb.shape[0] / nh
            # Draw ONLY the single winning patch cell; its midpoint IS the center dot (so they agree).
            r, c = np.unravel_index(int(sim_map.argmax().cpu()), (gh, gw))
            x1, y1 = c * self.PATCH * sx, r * self.PATCH * sy
            x2, y2 = (c + 1) * self.PATCH * sx, (r + 1) * self.PATCH * sy
            out["bbox"] = [float(x1), float(y1), float(x2), float(y2)]
            out["center"] = [float((x1 + x2) / 2), float((y1 + y2) / 2)]
            if not self._coord_logged:        # one-time coordinate-reconstruction check
                self._coord_logged = True
                print(f"[coord-check][dinov2] sim_map grid (gh,gw)=({gh},{gw}); peak matrix index "
                      f"[row={r}, col={c}]; scale (sx,sy)=({sx:.4f},{sy:.4f}) [frameW/nw,frameH/nh]; "
                      f"-> mapped center (X,Y)=({out['center'][0]:.1f},{out['center'][1]:.1f}) px in the "
                      f"{frame_rgb.shape[1]}x{frame_rgb.shape[0]} frame.", flush=True)
        out["infer_ms"] = (time.time() - t0) * 1e3
        return out


# ==============================================================================
# Engine: OWLv2 image-guided — reuses object_worker.Owlv2Detector's exact API
# ==============================================================================
class Owlv2BenchDetector:
    name = "owlv2"
    uses_gpu = True

    def __init__(self, device="cuda", hf_id="google/owlv2-base-patch16-ensemble",
                 logit_thresh=OWLV2_LOGIT_THRESH, **_):
        from transformers import Owlv2Processor, Owlv2ForObjectDetection
        self.device = device
        self.logit_thresh = float(logit_thresh)
        t0 = time.time()
        self.processor = Owlv2Processor.from_pretrained(hf_id)
        self.model = Owlv2ForObjectDetection.from_pretrained(hf_id).to(device).eval()
        print(f"[bench] OWLv2 '{hf_id}' ready in {time.time()-t0:.1f}s "
              f"| abs-logit gate >= {self.logit_thresh} | VRAM {torch.cuda.memory_allocated()/1e9:.2f} GB",
              flush=True)
        # IMAGE INTEGRITY disclosure (CLAUDE.md): the OWLv2 processor resizes+pads BOTH the query crop
        # and the frame to a model-required 960x960 square internally. We hand it the raw asset with no
        # intermediate downscale/crop, so it upscales from maximum source fidelity.
        print("[bench]   OWLv2 input transform: processor upscales/pads query + frame to 960x960 "
              "(model-required); no intermediate resize applied by us.", flush=True)

    def detect(self, ref_rgb, frame_rgb):
        t0 = time.time()
        H, W = frame_rgb.shape[:2]
        inp = self.processor(images=frame_rgb, query_images=ref_rgb, return_tensors="pt").to(self.device)
        if SHAPE_DEBUG:
            print(f"[shapes][owlv2] ref  input(HxWx3)={tuple(ref_rgb.shape)} "
                  f"-> query_pixel_values={tuple(inp['query_pixel_values'].shape)}", flush=True)
            print(f"[shapes][owlv2] frame input(HxWx3)={tuple(frame_rgb.shape)} "
                  f"-> pixel_values      ={tuple(inp['pixel_values'].shape)}", flush=True)
        with torch.inference_mode():
            o = self.model.image_guided_detection(**inp)

        # Found-gate = the RAW absolute image-guided logit (slot 0 = the single query image), the
        # discriminative signal object_worker.Owlv2Detector.verify_logit uses. NOT the post-processed
        # score, which self-normalizes to ~1.0 on every frame and so can't separate target from wall.
        logits = o.logits[0, :, 0]
        idx = int(torch.argmax(logits))
        abs_logit = float(logits[idx])
        found = abs_logit >= self.logit_thresh

        # Box for the top-logit anchor. OWLv2 pads the image to a SQUARE of side max(H,W) and
        # normalizes target_pred_boxes (cx,cy,w,h) to that square — _scale_boxes multiplies all four
        # coords by max(H,W), NOT W,H separately (verified in image_processing_owlv2._scale_boxes).
        cx, cy, bw, bh = (float(v) for v in o.target_pred_boxes[0, idx])
        s = float(max(H, W))
        x1 = float(np.clip((cx - bw / 2) * s, 0, W - 1)); x2 = float(np.clip((cx + bw / 2) * s, 0, W - 1))
        y1 = float(np.clip((cy - bh / 2) * s, 0, H - 1)); y2 = float(np.clip((cy + bh / 2) * s, 0, H - 1))

        # Post-processed top-box score kept ONLY for transparency (the strawman before/after gate).
        res = self.processor.post_process_image_guided_detection(
            o, target_sizes=torch.tensor([[H, W]], device=self.device),
            threshold=0.0, nms_threshold=0.3)[0]
        ns = res["scores"].detach().cpu().numpy()
        norm_top = float(ns.max()) if len(ns) else 0.0

        return {
            "found": found,
            "score": abs_logit,          # reported score for owlv2 = the abs logit (gated value)
            "abs_logit": abs_logit,
            "norm_top_score": norm_top,
            "bbox": [x1, y1, x2, y2] if found else None,
            "center": [(x1 + x2) / 2, (y1 + y2) / 2] if found else None,
            "infer_ms": (time.time() - t0) * 1e3,
        }


# ==============================================================================
# Engine: Qwen2.5-VL-3B (4-bit) — label-driven grounding on the 512x288 frame
# ==============================================================================
class QwenBenchDetector:
    """Standalone Qwen engine. Reuses the tested object_worker.QwenDetector (4-bit nf4, label
    derived from the reference crop, multi-image grounding, smart-resize rescale). Runs on the
    512x288 DOWNSCALED frame — the proven low-latency path (~2.5 s/frame; 720p degenerates to
    `!!!`). Qwen is binary (found / not) so score is 1.0/0.0; recall/good/FP/IoU are the signal."""

    name = "qwen"
    uses_gpu = True
    QWEN_W, QWEN_H = 512, 288   # downscaled transport frame (16:9 of 1280x720, exactly x2.5)

    def __init__(self, device="cuda", hf_id="Qwen/Qwen2.5-VL-3B-Instruct",
                 quantization="4bit", max_new_tokens=256, **_):
        from object_worker import QwenDetector
        self.core = QwenDetector(hf_id, quantization=quantization,
                                 max_new_tokens=max_new_tokens, device=device)
        self._labels = {}   # derived label cached per reference array (id)
        # IMAGE INTEGRITY disclosure (CLAUDE.md): frame is downscaled to 512x288 for the 4-bit @512
        # path; Qwen's processor then smart-resizes internally to ~504x280 (multiple of 28).
        print(f"[bench]   Qwen input transform: frame -> {self.QWEN_W}x{self.QWEN_H} downscale "
              f"(4-bit @512 path); processor smart-resizes to ~504x280 internally.", flush=True)

    def _label_for(self, ref_rgb):
        k = id(ref_rgb)
        if k not in self._labels:
            self._labels[k] = self.core.derive_label(ref_rgb)
            print(f"[bench]   Qwen derived label: '{self._labels[k]}'", flush=True)
        return self._labels[k]

    def detect(self, ref_rgb, frame_rgb):
        t0 = time.time()
        H, W = frame_rgb.shape[:2]
        small = cv2.resize(frame_rgb, (self.QWEN_W, self.QWEN_H), interpolation=cv2.INTER_AREA)
        label = self._label_for(ref_rgb)
        r = self.core.detect(ref_rgb, small, label)   # bbox/center in 512x288 px
        out = {"found": bool(r["found"]), "score": 1.0 if r["found"] else 0.0,
               "bbox": None, "center": None, "raw": r.get("raw"),
               "infer_ms": (time.time() - t0) * 1e3}
        if r["found"] and r.get("bbox"):
            sx, sy = W / self.QWEN_W, H / self.QWEN_H   # 512->1280, 288->720 (x2.5)
            b = r["bbox"]
            out["bbox"] = [b[0] * sx, b[1] * sy, b[2] * sx, b[3] * sy]
            out["center"] = [(out["bbox"][0] + out["bbox"][2]) / 2,
                             (out["bbox"][1] + out["bbox"][3]) / 2]
        return out


ENGINES = {
    "sift": SiftDetector,
    "lightglue": LightGlueDetector,
    "dinov2": DinoV2Detector,
    "owlv2": Owlv2BenchDetector,
    "qwen": QwenBenchDetector,
}


# ==============================================================================
# Grounded-SAM: language-guided foreground masking of reference templates (offline)
# ==============================================================================
class RefSegmenter:
    """One-time template onboarding: GroundingDINO (text -> box) -> SAM (box -> silhouette mask).
    Erases the template background to absolute black so patch/feature engines aren't swamped by
    clutter tokens. GPU-only; weights download on first use. NO SILENT FALLBACK: fail-fast if the
    text isn't grounded."""

    def __init__(self, device="cuda", gd_id="IDEA-Research/grounding-dino-tiny",
                 sam_id="facebook/sam-vit-base"):
        from transformers import (AutoProcessor, GroundingDinoForObjectDetection,
                                  SamModel, SamProcessor)
        self.device = device
        t0 = time.time()
        self.gd_proc = AutoProcessor.from_pretrained(gd_id)
        self.gd = GroundingDinoForObjectDetection.from_pretrained(gd_id).to(device).eval()
        self.sam_proc = SamProcessor.from_pretrained(sam_id)
        self.sam = SamModel.from_pretrained(sam_id).to(device).eval()
        print(f"[bench] Grounded-SAM ready in {time.time()-t0:.1f}s (GroundingDINO '{gd_id}' + SAM '{sam_id}') "
              f"| VRAM {torch.cuda.memory_allocated()/1e9:.2f} GB", flush=True)

    def mask(self, rgb, text):
        """Return (masked_rgb, gd_box, fg_fraction). Background pixels forced to (0,0,0)."""
        H, W = rgb.shape[:2]
        prompt = text.strip().lower()
        if not prompt.endswith("."):
            prompt += "."
        gin = self.gd_proc(images=rgb, text=prompt, return_tensors="pt").to(self.device)
        with torch.inference_mode():
            gout = self.gd(**gin)
        det = self.gd_proc.post_process_grounded_object_detection(
            gout, gin["input_ids"], threshold=0.25, text_threshold=0.25, target_sizes=[(H, W)])[0]
        boxes = det["boxes"].detach().cpu().numpy()
        scores = det["scores"].detach().cpu().numpy()
        if len(boxes) == 0:
            raise SystemExit(f"GroundingDINO did not ground '{text}' in the reference crop — adjust the "
                             f"REF_TEXT hint or lower the threshold (NO SILENT FALLBACK).")
        box = boxes[int(np.argmax(scores))].tolist()  # x0,y0,x1,y1

        sin = self.sam_proc(rgb, input_boxes=[[box]], return_tensors="pt").to(self.device)
        with torch.inference_mode():
            sout = self.sam(**sin)
        masks = self.sam_proc.image_processor.post_process_masks(
            sout.pred_masks.cpu(), sin["original_sizes"].cpu(), sin["reshaped_input_sizes"].cpu())[0]
        mt = masks[0] if masks.ndim == 4 else masks            # (num_masks, H, W) for the single box
        iou = sout.iou_scores[0].reshape(-1).detach().cpu().numpy()
        mask = np.asarray(mt[int(np.argmax(iou))]).astype(bool)
        masked = rgb.copy()
        masked[~mask] = 0
        return masked, box, float(mask.mean())

    def close(self):
        del self.gd, self.sam, self.gd_proc, self.sam_proc
        if torch.cuda.is_available():
            torch.cuda.empty_cache()


def build_masked_refs(refs):
    """Mask every target's reference template once. Saves test_assets/{target}_ref_masked.png and prints
    the non-black (foreground) fraction for a visual sanity check. Returns {target: masked_rgb}."""
    seg = RefSegmenter()
    masked = {}
    for name, rgb in refs.items():
        text = REF_TEXT.get(name)
        if text is None:
            raise SystemExit(f"no REF_TEXT hint for target '{name}' — add one before --mask-refs.")
        m, box, frac = seg.mask(rgb, text)
        masked[name] = m
        sp = REPO / "test_assets" / f"{name}_ref_masked.png"
        cv2.imwrite(str(sp), cv2.cvtColor(m, cv2.COLOR_RGB2BGR))
        print(f"[bench] masked ref [{name}] '{text}': foreground {frac*100:.1f}% of {m.shape[1]}x{m.shape[0]} "
              f"(GD box {[round(v) for v in box]}) -> {sp.name}", flush=True)
    seg.close()
    return masked


# ==============================================================================
# Overlays
# ==============================================================================
def save_overlay(out_dir: Path, engine, target, fname, rgb, gt, pred):
    out_dir.mkdir(parents=True, exist_ok=True)
    img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR).copy()
    if gt is not None:
        cv2.rectangle(img, (gt[0], gt[1]), (gt[2], gt[3]), (0, 255, 0), 2)        # GT = green
    pb = pred.get("bbox") if pred.get("found") else None                          # only a real detection
    if pb is not None:
        p = [int(round(v)) for v in pb]
        ok = center_in_box(pred.get("center"), gt) if gt is not None else None
        col = (0, 255, 255) if ok else (0, 0, 255)                                # pred = cyan ok / red wrong
        cv2.rectangle(img, (p[0], p[1]), (p[2], p[3]), col, 2)
        c = [int(round(v)) for v in pred["center"]]
        cv2.circle(img, (c[0], c[1]), 5, col, -1)
    tag = f"{engine}/{target} {fname}  found={pred['found']} score={pred['score']:.2f}"
    cv2.putText(img, tag, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.imwrite(str(out_dir / fname), img)


# ==============================================================================
# Run + metrics
# ==============================================================================
def summarize(rows):
    """rows = list of per-frame dicts. Aggregate per (engine, target)."""
    agg = {}
    for r in rows:
        key = (r["engine"], r["target"])
        a = agg.setdefault(key, {"pos": [], "neg": []})
        a["pos" if r["is_pos"] else "neg"].append(r)
    summary = {}
    for (engine, target), a in sorted(agg.items()):
        pos, neg = a["pos"], a["neg"]
        found_pos = [r for r in pos if r["found"]]
        loc_ok = [r for r in found_pos if r["center_in_gt"]]
        iou_hits = [r for r in found_pos if r["iou"] >= IOU_REPORT]
        pos_scores = [r["score"] for r in pos]
        neg_scores = [r["score"] for r in neg]
        fp = [r for r in neg if r["found"]]
        summary[f"{engine}|{target}"] = {
            "engine": engine, "target": target,
            "n_pos": len(pos), "n_neg": len(neg),
            "recall_found": round(len(found_pos) / max(1, len(pos)), 3),
            "good_detect": round(len(loc_ok) / max(1, len(pos)), 3),   # found AND center in GT box
            "loc_ok_of_found": round(len(loc_ok) / max(1, len(found_pos)), 3) if found_pos else 0.0,
            "iou>=0.5_of_found": round(len(iou_hits) / max(1, len(found_pos)), 3) if found_pos else 0.0,
            "fp_rate_neg": round(len(fp) / max(1, len(neg)), 3),
            "pos_score_med": round(float(np.median(pos_scores)), 3) if pos_scores else 0.0,
            "neg_score_med": round(float(np.median(neg_scores)), 3) if neg_scores else 0.0,
            "median_ms": round(float(np.median([r["infer_ms"] for r in pos + neg])), 1),
        }
    return summary


def print_table(summary):
    cols = [("engine", 10), ("target", 10), ("n_pos", 6), ("recall", 7), ("good", 6),
            ("loc/fnd", 8), ("FP_neg", 7), ("pos_med", 8), ("neg_med", 8), ("ms", 7)]
    print("\n" + "=" * 92)
    print("  BENCHMARK SUMMARY   (good = found AND center in GT box;  FP_neg = fires on a negative)")
    print("  pos_med/neg_med = median engine score on positives vs negatives -- the separation IS")
    print("  the discrimination signal. Thresholds are tunable constants at the top of the file.")
    print("=" * 92)
    print("  " + "".join(h.ljust(w) for h, w in cols))
    print("  " + "-" * 88)
    for v in summary.values():
        row = [v["engine"], v["target"], str(v["n_pos"]),
               f"{v['recall_found']:.2f}", f"{v['good_detect']:.2f}",
               f"{v['loc_ok_of_found']:.2f}", f"{v['fp_rate_neg']:.2f}",
               f"{v['pos_score_med']:.2f}", f"{v['neg_score_med']:.2f}", f"{v['median_ms']:.0f}"]
        print("  " + "".join(c.ljust(w) for c, (_, w) in zip(row, cols)))
    print("=" * 92 + "\n")


def main():
    ap = argparse.ArgumentParser(description="Benchmark detection engines for the object chain.")
    ap.add_argument("--engines", default="sift,lightglue,dinov2,owlv2,qwen",
                    help="comma list subset of: " + ",".join(ENGINES))
    ap.add_argument("--out", default="OUTPUT/benchmark", help="output dir (under repo)")
    ap.add_argument("--no-overlays", action="store_true", help="skip writing annotated overlay images")
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--config", default=None)
    ap.add_argument("--debug-shapes", action="store_true",
                    help="print input tensor shapes at each model forward pass")
    ap.add_argument("--max-frames", type=int, default=None,
                    help="run only N positives per target and skip negatives (quick shape/smoke check)")
    ap.add_argument("--mask-refs", action="store_true",
                    help="Grounded-SAM: language-mask each reference template to a background-free "
                         "silhouette before benchmarking (applies to ALL engines)")
    args = ap.parse_args()

    global SHAPE_DEBUG
    SHAPE_DEBUG = args.debug_shapes

    engines = [e.strip() for e in args.engines.split(",") if e.strip()]
    bad = [e for e in engines if e not in ENGINES]
    if bad:
        raise SystemExit(f"unknown engine(s): {bad}. choose from {list(ENGINES)}")

    cfg = {}
    cfg_path = Path(args.config) if args.config else REPO / "config.yaml"
    if cfg_path.exists():
        cfg = yaml.safe_load(cfg_path.read_text(encoding="utf-8")) or {}
    owlv2_cfg = (cfg.get("models", {}).get("owlv2", {}) or {})
    owlv2_id = owlv2_cfg.get("hf_id", "google/owlv2-base-patch16-ensemble")
    owlv2_logit_thresh = float(owlv2_cfg.get("verify_thresh", OWLV2_LOGIT_THRESH))
    qwen_cfg = (cfg.get("models", {}).get("qwen_vl", {}) or {})

    def engine_kwargs(ename):
        kw = {"device": args.device}
        if ename == "owlv2":
            kw.update(hf_id=owlv2_id, logit_thresh=owlv2_logit_thresh)
        elif ename == "qwen":
            kw.update(hf_id=qwen_cfg.get("hf_id", "Qwen/Qwen2.5-VL-3B-Instruct"),
                      quantization=qwen_cfg.get("quantization", "4bit"),
                      max_new_tokens=int(qwen_cfg.get("max_new_tokens", 256)))
        return kw

    needs_gpu = any(ENGINES[e].uses_gpu for e in engines)
    if needs_gpu and not torch.cuda.is_available():
        raise SystemExit("CUDA not available — GPU engines require it (NO SILENT FALLBACKS).")

    print("[bench] loading dataset ...", flush=True)
    refs, positives, negatives = load_dataset()
    for name, items in positives.items():
        print(f"[bench]   {name}: {len(items)} positives | ref {refs[name].shape[1]}x{refs[name].shape[0]}")
    print(f"[bench]   None: {len(negatives)} negatives")
    if args.mask_refs:
        # One-time offline template onboarding; ALL engines then use the background-free silhouettes.
        refs = build_masked_refs(refs)

    out_dir = REPO / args.out
    out_dir.mkdir(parents=True, exist_ok=True)
    rows = []

    for ename in engines:
        print(f"\n[bench] === engine: {ename} ===", flush=True)
        det = ENGINES[ename](**engine_kwargs(ename))
        for target, items in positives.items():
            ref = refs[target]   # all engines use the canonical user-provided template
            # positives
            sel = items[:args.max_frames] if args.max_frames else items
            for fname, rgb, gt in sel:
                r = det.detect(ref, rgb)
                rec = _record(ename, target, fname, True, gt, r)
                rows.append(rec)
                if not args.no_overlays:
                    save_overlay(out_dir / "overlays" / ename / target, ename, target, fname, rgb, gt, r)
            if args.max_frames:
                continue  # quick shape/smoke check: skip negatives
            # negatives (run per ref — a None frame may match each target's ref differently)
            for fname, rgb in negatives:
                r = det.detect(ref, rgb)
                rec = _record(ename, target, fname, False, None, r)
                rows.append(rec)
                if not args.no_overlays:
                    save_overlay(out_dir / "overlays" / ename / f"None_vs_{target}", ename, target, fname, rgb, None, r)
        del det
        if torch.cuda.is_available():
            torch.cuda.empty_cache()

    summary = summarize(rows)
    print_table(summary)

    (out_dir / "per_frame.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"[bench] wrote {out_dir/'summary.json'} + per_frame.json"
          + ("" if args.no_overlays else f" + overlays/ under {out_dir}"))


def _record(engine, target, fname, is_pos, gt, r):
    # pred_* are tied to a real detection: a silent/empty detection (found=False) => pred_bbox null,
    # giving a clean machine-readable layer to verify the box math (e.g. OWLv2's padded-square scaling).
    found = bool(r["found"])
    pred_bbox = r.get("bbox") if found else None
    pred_center = r.get("center") if found else None
    return {
        "engine": engine, "target": target, "frame": fname, "is_pos": is_pos,
        "found": found, "score": round(float(r["score"]), 4),
        "abs_logit": round(float(r["abs_logit"]), 4) if "abs_logit" in r else None,
        "norm_top_score": round(float(r["norm_top_score"]), 4) if "norm_top_score" in r else None,
        "gt_bbox": [int(v) for v in gt] if gt else None,
        "pred_bbox": [round(v, 1) for v in pred_bbox] if pred_bbox else None,
        "pred_center": [round(v, 1) for v in pred_center] if pred_center else None,
        "center_in_gt": center_in_box(pred_center, gt) if (is_pos and gt) else False,
        "iou": round(iou(pred_bbox, gt), 3) if (is_pos and gt) else 0.0,
        "infer_ms": round(float(r["infer_ms"]), 1),
        "raw": r.get("raw"),   # Qwen's literal text reply (None for other engines)
    }


if __name__ == "__main__":
    main()
