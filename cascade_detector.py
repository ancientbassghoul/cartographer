"""cascade_detector.py — generalized verified cascade detector (any target, any asset class).

Per PROGRESS.md, every single-shot engine (SIFT/LightGlue/DINOv2-dense/OWLv2/Qwen) over-fired
because it conflated "where is a candidate" with "is this THE target". This keeps the cascade
paradigm (propose wide -> verify ruthlessly) and is driven entirely by an AssetClass, NOT by any
hardcoded target name:

  STAGE 1  Propose (recall ceiling)
      GroundingDINO (text-guided) + OWLv2 image-guided, POOLED, low thresholds. The cascade can
      never beat this stage's recall, so we report each proposer's standalone recall first.

  STAGE 2  Verify (DINOv2 crop embedding)
      Letterbox (NEVER squash) each candidate crop AND the reference to a square 224x224, embed with
      DINOv2 ViT-S/14, cosine-compare (CLS + mean-pool both reported). Survivor gate = the
      asset class's `dino_thresh`.

  STAGE 3  Geometric (asset-class-driven gate)
      The asset class chooses the engine + mode:
        2D_PLANAR    -> SIFT + RANSAC homography, HARD gate (>= GEOM_HARD_MIN_INLIERS inliers).
        3D_GEOMETRY  -> LightGlue inliers, SOFT bonus (reported; NEVER vetoes a Stage-2 survivor).

  DECISION  Among accepted survivors, rank by the Stage-2 DINOv2 cosine FIRST (primary), geometric
      inliers only as a tie-break. (Geometry-first previously mis-ranked planar targets — a loose
      off-target box with marginally more inliers beat the near-exact box with a much higher DINOv2
      score; appearance-primary fixes that.)

You supply each target at runtime: TEXT + REFERENCE IMAGE + ASSET CLASS (+ frames, optional labels
and negatives). Either one target via CLI flags, or many via a --targets YAML.

GPU-only, NO SILENT FALLBACKS (CLAUDE.md): CUDA + each model load + invalid asset class / missing
ref all fail-fast; image resizes are logged (image-integrity rule). Models load ONE STAGE AT A TIME
and are freed, so peak VRAM stays small.

  # single target
  venv\\Scripts\\python.exe cascade_detector.py --text "a rifle" --ref test_assets/Rifle_ref.png \\
        --asset-class 3D_GEOMETRY --frames test_assets/Rifle --labels test_assets/Rifle/labels.json \\
        --negatives test_assets/None
  # batch (combined summary table)
  venv\\Scripts\\python.exe cascade_detector.py --targets cascade_targets.yaml
  # quick smoke (N positives/target, negatives skipped)
  venv\\Scripts\\python.exe cascade_detector.py --targets cascade_targets.yaml --max-frames 2
"""

import argparse
import json
import time
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import torch
import yaml

# Reuse only the GENERIC, pure helpers + geometric engines from the benchmark harness (its main() is
# __main__-guarded, so importing is side-effect-free). No hardcoded TARGETS/REF_TEXT/load_dataset.
from benchmark_detectors import (
    LightGlueDetector,
    SiftDetector,
    center_in_box,
    iou,
    list_images,
    load_rgb,
)

REPO = Path(__file__).resolve().parent


# ==============================================================================
# Asset classes — the ONLY place per-target behavior is defined (no target names)
# ==============================================================================
class AssetClass(Enum):
    """A target's geometric nature. Drives the DINOv2 threshold and the Stage-3 gate."""
    PLANAR_2D = "2D_PLANAR"        # flat/printed (poster, sign): texture-rich -> SIFT can hard-gate
    GEOMETRY_3D = "3D_GEOMETRY"    # 3D object (rifle): low-texture/self-similar -> geometry is soft


# Per-class parameters. dino_thresh = Stage-2 CLS-cosine survivor gate; gate_engine/gate_mode =
# Stage-3 geometric verifier + whether it can VETO (hard) or only annotate/rank (soft). TUNABLE.
ASSET_CLASS_PARAMS = {
    AssetClass.PLANAR_2D:   {"dino_thresh": 0.33, "gate_engine": "sift",      "gate_mode": "hard"},
    AssetClass.GEOMETRY_3D: {"dino_thresh": 0.40, "gate_engine": "lightglue", "gate_mode": "soft"},
}


def resolve_asset_class(value) -> AssetClass:
    """Map a CLI/YAML string ('2D_PLANAR' / '3D_GEOMETRY') to an AssetClass; fail-fast on unknown."""
    if isinstance(value, AssetClass):
        return value
    try:
        return AssetClass(str(value).strip())
    except ValueError:
        raise SystemExit(f"unknown asset class '{value}' — choose from "
                         f"{[a.value for a in AssetClass]} (NO SILENT FALLBACK).")


# --- Stage-1 proposer tunables (recall-first; the summary prints the recall ceiling) ----------
GD_BOX_THRESH = 0.25      # GroundingDINO box confidence -- intentionally low to cast a wide net
GD_TEXT_THRESH = 0.25     # GroundingDINO text-token confidence
GD_TOPK = 8               # keep at most this many GD boxes per frame
OWLV2_TOPK = 5            # keep this many OWLv2 image-guided anchors per frame (by abs logit)
OWLV2_MIN_LOGIT = 2.0     # floor on the OWLv2 abs image-guided logit to be a candidate
NMS_IOU = 0.7             # per-source dedup of overlapping proposals

# --- Stage-2 DINOv2 verifier tunables (threshold is per asset class, above) --------------------
DINO_SIZE = 224           # square side fed to DINOv2 (16x16 patch grid at /14)
DINO_PAD = 114            # neutral letterbox fill (mid-gray), applied to ref AND candidate alike

# --- Stage-3 geometric tunables ----------------------------------------------------------------
GEOM_HARD_MIN_INLIERS = 12   # HARD gate: RANSAC inliers to accept (clear planar target ~40-67)
GEOM_MIN_CROP = 24           # crops smaller than this skip geometric matching (explicit, logged)

PROPOSAL_HIT_IOU = 1e-6   # a proposal "covers" GT if IoU > 0 (recall-ceiling definition)


# ==============================================================================
# Target specification (supplied at runtime)
# ==============================================================================
@dataclass
class TargetSpec:
    name: str
    text: str                       # GroundingDINO grounding phrase, e.g. "a rifle"
    ref_path: str                   # reference crop (OWLv2 query + DINOv2 template)
    frames_dir: str                 # folder of frames to search (positives)
    asset_class: AssetClass
    labels_path: Optional[str] = None     # GT boxes; default <frames_dir>/labels.json if present
    negatives_dir: Optional[str] = None   # optional target-absent frames for the FP metric


def _resolve(p: str) -> Path:
    p = Path(p)
    return p if p.is_absolute() else REPO / p


def load_spec(spec: TargetSpec, max_frames=None):
    """Load (ref_rgb, positives[(fname,rgb,gt)], negatives[(fname,rgb)]) for one target spec."""
    ref_p = _resolve(spec.ref_path)
    if not ref_p.exists():
        raise SystemExit(f"missing reference image for '{spec.name}': {ref_p}")
    ref = load_rgb(ref_p)

    fdir = _resolve(spec.frames_dir)
    if not fdir.is_dir():
        raise SystemExit(f"frames dir for '{spec.name}' not found: {fdir}")
    lp = _resolve(spec.labels_path) if spec.labels_path else fdir / "labels.json"
    labels = json.loads(lp.read_text(encoding="utf-8")) if lp.exists() else {}

    positives = []
    for f in list_images(fdir):
        gt = labels.get(f.name)
        gt = [int(v) for v in gt] if isinstance(gt, list) and len(gt) == 4 else None
        positives.append((f.name, load_rgb(f), gt))
    if max_frames:
        positives = positives[:max_frames]

    negatives = []
    if spec.negatives_dir and not max_frames:        # smoke check (max_frames) skips negatives
        ndir = _resolve(spec.negatives_dir)
        if not ndir.is_dir():
            raise SystemExit(f"negatives dir for '{spec.name}' not found: {ndir}")
        negatives = [(f.name, load_rgb(f)) for f in list_images(ndir)]
    return ref, positives, negatives


# ==============================================================================
# Image helpers
# ==============================================================================
def letterbox(rgb, size=DINO_SIZE, pad=DINO_PAD):
    """Aspect-PRESERVING resize into a square `size` canvas with neutral padding.

    NEVER squashes: an extreme-aspect ref (e.g. a 168x447 rifle) force-resized to a square would
    morphologically distort it (thin barrel -> stubby) and tank the cosine. Scale-to-fit + pad.
    """
    h, w = rgb.shape[:2]
    s = min(size / w, size / h)
    nw, nh = max(1, int(round(w * s))), max(1, int(round(h * s)))
    interp = cv2.INTER_AREA if s < 1 else cv2.INTER_CUBIC
    resized = cv2.resize(rgb, (nw, nh), interpolation=interp)
    canvas = np.full((size, size, 3), pad, dtype=rgb.dtype)
    y0, x0 = (size - nh) // 2, (size - nw) // 2
    canvas[y0:y0 + nh, x0:x0 + nw] = resized
    return canvas


def clip_box(box, W, H):
    """Round + clamp a float box to integer pixels inside the frame; None if degenerate."""
    x1, y1, x2, y2 = box
    x1 = int(max(0, min(W - 1, round(x1)))); x2 = int(max(0, min(W, round(x2))))
    y1 = int(max(0, min(H - 1, round(y1)))); y2 = int(max(0, min(H, round(y2))))
    if x2 <= x1 or y2 <= y1:
        return None
    return [x1, y1, x2, y2]


def crop_of(rgb, box):
    return rgb[box[1]:box[3], box[0]:box[2]]


def box_center(box):
    return [(box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0]


def nms(cands, iou_thresh=NMS_IOU):
    """Greedy per-source dedup: keep the highest-scoring box, drop boxes overlapping it."""
    keep = []
    for c in sorted(cands, key=lambda c: -c["score"]):
        if all(iou(c["box"], k["box"]) < iou_thresh for k in keep):
            keep.append(c)
    return keep


# ==============================================================================
# Stage 1a: GroundingDINO text-guided proposer
# ==============================================================================
class GroundingDinoProposer:
    def __init__(self, device="cuda", gd_id="IDEA-Research/grounding-dino-tiny"):
        from transformers import AutoProcessor, GroundingDinoForObjectDetection
        self.device = device
        t0 = time.time()
        self.proc = AutoProcessor.from_pretrained(gd_id)
        self.model = GroundingDinoForObjectDetection.from_pretrained(gd_id).to(device).eval()
        print(f"[cascade] GroundingDINO '{gd_id}' ready in {time.time()-t0:.1f}s | "
              f"box>={GD_BOX_THRESH} text>={GD_TEXT_THRESH} topk={GD_TOPK} | "
              f"VRAM {torch.cuda.memory_allocated()/1e9:.2f} GB", flush=True)

    def propose(self, frame_rgb, text):
        H, W = frame_rgb.shape[:2]
        prompt = text.strip().lower()
        if not prompt.endswith("."):
            prompt += "."
        gin = self.proc(images=frame_rgb, text=prompt, return_tensors="pt").to(self.device)
        with torch.inference_mode():
            gout = self.model(**gin)
        det = self.proc.post_process_grounded_object_detection(
            gout, gin["input_ids"], threshold=GD_BOX_THRESH, text_threshold=GD_TEXT_THRESH,
            target_sizes=[(H, W)])[0]
        boxes = det["boxes"].detach().cpu().numpy()
        scores = det["scores"].detach().cpu().numpy()
        out = []
        for i in np.argsort(-scores)[:GD_TOPK]:
            b = clip_box(boxes[i].tolist(), W, H)        # GD returns x0,y0,x1,y1 in frame pixels
            if b:
                out.append({"source": "gd", "score": float(scores[i]), "box": b})
        return out


# ==============================================================================
# Stage 1b: OWLv2 image-guided proposer (top-K anchors, not just top-1)
# ==============================================================================
class Owlv2Proposer:
    def __init__(self, device="cuda", hf_id="google/owlv2-base-patch16-ensemble"):
        from transformers import Owlv2ForObjectDetection, Owlv2Processor
        self.device = device
        t0 = time.time()
        self.processor = Owlv2Processor.from_pretrained(hf_id)
        self.model = Owlv2ForObjectDetection.from_pretrained(hf_id).to(device).eval()
        print(f"[cascade] OWLv2 '{hf_id}' ready in {time.time()-t0:.1f}s | top{OWLV2_TOPK} "
              f"abs-logit>={OWLV2_MIN_LOGIT} | VRAM {torch.cuda.memory_allocated()/1e9:.2f} GB",
              flush=True)
        # IMAGE INTEGRITY (CLAUDE.md): the OWLv2 processor upscales/pads query crop + frame to a
        # model-required 960x960 square. We hand it raw assets (no intermediate downscale).
        print("[cascade]   OWLv2 input transform: processor pads/upscales query+frame to 960x960 "
              "(model-required); no resize applied by us.", flush=True)

    def propose(self, frame_rgb, ref_rgb):
        H, W = frame_rgb.shape[:2]
        inp = self.processor(images=frame_rgb, query_images=ref_rgb,
                             return_tensors="pt").to(self.device)
        with torch.inference_mode():
            o = self.model.image_guided_detection(**inp)
        logits = o.logits[0, :, 0]                       # slot 0 = the single query image
        k = min(OWLV2_TOPK, int(logits.shape[0]))
        top = torch.topk(logits, k)
        # OWLv2 normalizes target_pred_boxes (cx,cy,w,h) to a SQUARE of side max(H,W) -- multiply all
        # four coords by max(H,W), NOT W,H separately (image_processing_owlv2._scale_boxes).
        s = float(max(H, W))
        out = []
        for val, idx in zip(top.values.tolist(), top.indices.tolist()):
            if val < OWLV2_MIN_LOGIT:
                continue
            cx, cy, bw, bh = (float(v) for v in o.target_pred_boxes[0, idx])
            b = clip_box([(cx - bw / 2) * s, (cy - bh / 2) * s,
                          (cx + bw / 2) * s, (cy + bh / 2) * s], W, H)
            if b:
                out.append({"source": "owlv2", "score": float(val), "box": b})
        return out


# ==============================================================================
# Stage 2: DINOv2 crop-embedding verifier (letterboxed, global embedding)
# ==============================================================================
class DinoV2CropEmbedder:
    MEAN = (0.485, 0.456, 0.406)
    STD = (0.229, 0.224, 0.225)

    def __init__(self, device="cuda"):
        self.device = device
        t0 = time.time()
        self.model = torch.hub.load("facebookresearch/dinov2", "dinov2_vits14",
                                    trust_repo=True).to(device).eval()
        self._mean = torch.tensor(self.MEAN, device=device).view(1, 3, 1, 1)
        self._std = torch.tensor(self.STD, device=device).view(1, 3, 1, 1)
        print(f"[cascade] DINOv2 (vits14) ready in {time.time()-t0:.1f}s | "
              f"VRAM {torch.cuda.memory_allocated()/1e9:.2f} GB", flush=True)
        # IMAGE INTEGRITY (CLAUDE.md): disclose the exact transform.
        print(f"[cascade]   DINOv2 input transform: ref + each candidate crop -> aspect-preserving "
              f"LETTERBOX to {DINO_SIZE}x{DINO_SIZE} (pad={DINO_PAD} gray, NO squash); /14 -> "
              f"{DINO_SIZE//14}x{DINO_SIZE//14} patch grid.", flush=True)

    def embed(self, rgb):
        """Return (cls_unit, meanpool_unit) L2-normalized embeddings of a letterboxed crop."""
        img = letterbox(rgb)
        t = torch.from_numpy(img).float().permute(2, 0, 1).unsqueeze(0).to(self.device) / 255.0
        t = (t - self._mean) / self._std
        with torch.inference_mode():
            f = self.model.forward_features(t)
        cls = torch.nn.functional.normalize(f["x_norm_clstoken"][0], dim=0)
        mean = torch.nn.functional.normalize(f["x_norm_patchtokens"][0].mean(0), dim=0)
        return cls, mean

    @staticmethod
    def cosine(a, b):
        return float((a * b).sum())


# ==============================================================================
# Pipeline
# ==============================================================================
def build_samples(loaded):
    """loaded = [(spec, ref_rgb, positives, negatives)]. Returns (samples, refs{name->rgb}).

    Each sample carries its target's resolved asset-class params (dino_thresh / gate_engine /
    gate_mode / text), so NO downstream stage ever looks up a target by name."""
    samples, refs = [], {}
    for spec, ref, positives, negatives in loaded:
        refs[spec.name] = ref
        p = ASSET_CLASS_PARAMS[spec.asset_class]
        meta = {"target": spec.name, "text": spec.text, "asset_class": spec.asset_class.value,
                "dino_thresh": p["dino_thresh"], "gate_engine": p["gate_engine"],
                "gate_mode": p["gate_mode"]}
        for fname, rgb, gt in positives:
            samples.append({**meta, "frame": fname, "rgb": rgb, "gt": gt, "is_pos": True, "cands": []})
        for fname, rgb in negatives:
            samples.append({**meta, "frame": fname, "rgb": rgb, "gt": None, "is_pos": False, "cands": []})
    return samples, refs


def stage1_propose(samples, refs, device, owlv2_id):
    print("\n[cascade] === STAGE 1: propose (GroundingDINO + OWLv2) ===", flush=True)
    gd = GroundingDinoProposer(device=device)
    for s in samples:
        s["cands"] += gd.propose(s["rgb"], s["text"])     # text comes from the target spec
    del gd
    torch.cuda.empty_cache()

    owl = Owlv2Proposer(device=device, hf_id=owlv2_id)
    for s in samples:
        s["cands"] += owl.propose(s["rgb"], refs[s["target"]])
    del owl
    torch.cuda.empty_cache()

    # Per-source NMS (don't merge across sources -- GD scores are 0..1, OWLv2 logits ~2..10), then
    # tag each candidate with whether it covers GT (for the proposal-level recall ceiling).
    for s in samples:
        gd_c = nms([c for c in s["cands"] if c["source"] == "gd"])
        ow_c = nms([c for c in s["cands"] if c["source"] == "owlv2"])
        s["cands"] = gd_c + ow_c
        for c in s["cands"]:
            c["covers_gt"] = bool(s["gt"]) and iou(c["box"], s["gt"]) > PROPOSAL_HIT_IOU


def stage2_verify(samples, refs, device):
    print("\n[cascade] === STAGE 2: verify (DINOv2 crop embedding, letterboxed) ===", flush=True)
    dino = DinoV2CropEmbedder(device=device)
    ref_emb = {t: dino.embed(rgb) for t, rgb in refs.items()}
    for s in samples:
        rc, rm = ref_emb[s["target"]]
        for c in s["cands"]:
            crop = crop_of(s["rgb"], c["box"])
            if crop.size == 0:
                c["dino_cls"], c["dino_mean"], c["stage2"] = 0.0, 0.0, False
                continue
            cls, mean = dino.embed(crop)
            c["dino_cls"] = dino.cosine(cls, rc)
            c["dino_mean"] = dino.cosine(mean, rm)
            c["stage2"] = c["dino_cls"] >= s["dino_thresh"]   # per asset-class threshold
    del dino
    torch.cuda.empty_cache()


def stage3_geometric(samples, refs, device):
    print("\n[cascade] === STAGE 3: geometric (asset-class gate: hard SIFT / soft LightGlue) ===",
          flush=True)
    sift = SiftDetector()
    lg = LightGlueDetector(device=device)
    engines = {"sift": sift, "lightglue": lg}
    for s in samples:
        hard = s["gate_mode"] == "hard"
        engine = engines[s["gate_engine"]]
        for c in s["cands"]:
            c["geom_inliers"] = 0
            c["geom_engine"] = s["gate_engine"]
            c["geom_note"] = None
            if not c.get("stage2"):
                continue                                  # only verified survivors reach geometry
            crop = crop_of(s["rgb"], c["box"])
            if min(crop.shape[:2]) < GEOM_MIN_CROP:
                c["geom_note"] = "crop_too_small"        # explicit, visible -- NOT a silent skip
                c["stage3_hard"] = not hard               # soft gate unaffected; hard gate fails
                continue
            r = engine.detect(refs[s["target"]], crop)
            c["geom_inliers"] = int(r["score"])
            # hard gate can VETO; soft gate only annotates/ranks (never vetoes a Stage-2 survivor).
            c["stage3_hard"] = (c["geom_inliers"] >= GEOM_HARD_MIN_INLIERS) if hard else True
    del sift, lg
    torch.cuda.empty_cache()


def decide(samples):
    """Final per-frame decision + ranking among accepted candidates."""
    for s in samples:
        hard = s["gate_mode"] == "hard"
        accepted = [c for c in s["cands"]
                    if c.get("stage2") and (c.get("stage3_hard") if hard else True)]
        for c in s["cands"]:
            c["accepted"] = c in accepted
            c["reject_reason"] = _reject_reason(c, s)
        if accepted:
            # Rank survivors by the Stage-2 DINOv2 cosine FIRST (appearance is the primary metric),
            # geometric inliers ONLY as a tie-break -- for every asset class. Geometry-first
            # mis-ranked planar targets (a loose off-target box with marginally more inliers beat the
            # near-exact box that had a much higher DINOv2 score). A hard gate (planar) still filters
            # above in stage3_geometric; here geometry never overrides a better appearance match.
            best = max(accepted, key=lambda c: (c["dino_cls"], c["geom_inliers"]))
            s["final"] = best
        else:
            s["final"] = None


def _reject_reason(c, s):
    if c.get("accepted"):
        return None
    if not c.get("stage2"):
        return f"stage2_dino<{s['dino_thresh']}"
    if s["gate_mode"] == "hard" and not c.get("stage3_hard"):
        note = c.get("geom_note")
        return f"stage3_{s['gate_engine']}<{GEOM_HARD_MIN_INLIERS}" + (f"({note})" if note else "")
    return "not_best"


# ==============================================================================
# Overlays
# ==============================================================================
def neg_subdir(target):
    return f"neg_vs_{target}"


def save_overlay(out_dir: Path, s):
    out_dir.mkdir(parents=True, exist_ok=True)
    img = cv2.cvtColor(s["rgb"], cv2.COLOR_RGB2BGR).copy()
    gt = s["gt"]
    if gt:
        cv2.rectangle(img, (gt[0], gt[1]), (gt[2], gt[3]), (0, 255, 0), 2)            # GT = green
    # all proposals = thin gray; Stage-2 survivors = yellow
    for c in s["cands"]:
        b = c["box"]
        if c.get("stage2"):
            cv2.rectangle(img, (b[0], b[1]), (b[2], b[3]), (0, 255, 255), 1)          # yellow
        else:
            cv2.rectangle(img, (b[0], b[1]), (b[2], b[3]), (140, 140, 140), 1)        # gray
        cv2.putText(img, f"{c['source']}:{c['score']:.2f} d{c.get('dino_cls',0):.2f}",
                    (b[0], max(12, b[1] - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    (200, 200, 200), 1)
    # final accepted box = cyan (on target) / red (wrong or false positive)
    f = s.get("final")
    if f:
        b, ctr = f["box"], box_center(f["box"])
        ok = center_in_box(ctr, gt) if gt else False
        col = (255, 255, 0) if ok else (0, 0, 255)        # cyan correct / red wrong (BGR)
        cv2.rectangle(img, (b[0], b[1]), (b[2], b[3]), col, 3)
        cv2.circle(img, (int(ctr[0]), int(ctr[1])), 5, col, -1)
    tag = (f"{s['target']} [{s['asset_class']}] {s['frame']} | cands={len(s['cands'])} "
           f"surv={sum(1 for c in s['cands'] if c.get('stage2'))} "
           f"final={'Y' if f else 'N'}")
    cv2.putText(img, tag, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.imwrite(str(out_dir / s["frame"]), img)


# ==============================================================================
# Records + metrics
# ==============================================================================
def record(s):
    f = s.get("final")
    final_box = f["box"] if f else None
    final_center = box_center(final_box) if final_box else None
    cands = [{
        "source": c["source"], "stage1_score": round(c["score"], 4), "box": c["box"],
        "covers_gt": c.get("covers_gt", False),
        "dino_cls": round(c.get("dino_cls", 0.0), 4), "dino_mean": round(c.get("dino_mean", 0.0), 4),
        "stage2": c.get("stage2", False),
        "geom_engine": c.get("geom_engine"), "geom_inliers": c.get("geom_inliers", 0),
        "geom_note": c.get("geom_note"),
        "accepted": c.get("accepted", False), "reject_reason": c.get("reject_reason"),
    } for c in s["cands"]]
    return {
        "target": s["target"], "frame": s["frame"], "is_pos": s["is_pos"],
        "asset_class": s["asset_class"], "dino_thresh": s["dino_thresh"],
        "gate_engine": s["gate_engine"], "gate_mode": s["gate_mode"],
        "gt_bbox": s["gt"],
        "stage1_covers_gt": any(c.get("covers_gt") for c in s["cands"]) if s["is_pos"] else None,
        "stage1_covers_gt_gd": any(c.get("covers_gt") for c in s["cands"] if c["source"] == "gd")
                               if s["is_pos"] else None,
        "stage1_covers_gt_owlv2": any(c.get("covers_gt") for c in s["cands"] if c["source"] == "owlv2")
                                  if s["is_pos"] else None,
        "n_cands": len(s["cands"]),
        "n_stage2_survivors": sum(1 for c in s["cands"] if c.get("stage2")),
        "found": bool(f),
        "final_box": final_box, "final_center": [round(v, 1) for v in final_center] if final_center else None,
        "final_source": f["source"] if f else None,
        "final_dino_cls": round(f["dino_cls"], 4) if f else None,
        "final_geom_inliers": f.get("geom_inliers", 0) if f else None,
        "center_in_gt": center_in_box(final_center, s["gt"]) if (s["is_pos"] and s["gt"]) else False,
        "iou": round(iou(final_box, s["gt"]), 3) if (s["is_pos"] and s["gt"]) else 0.0,
        "candidates": cands,
    }


def summarize(records):
    summary = {}
    for target in sorted({r["target"] for r in records}):
        rs = [r for r in records if r["target"] == target]
        pos = [r for r in rs if r["is_pos"]]
        neg = [r for r in rs if not r["is_pos"]]
        found_pos = [r for r in pos if r["found"]]
        good = [r for r in found_pos if r["center_in_gt"]]
        fp = [r for r in neg if r["found"]]
        # post-Stage-2 (before geometry): any survivor whose center lands in GT / any survivor on a neg
        s2_good = [r for r in pos if any(_cand_center_in(c, r["gt_bbox"]) for c in r["candidates"]
                                         if c["stage2"])]
        s2_fp = [r for r in neg if r["n_stage2_survivors"] > 0]
        # DINOv2 separability: strongest candidate cls per frame, pos vs neg medians
        pos_best = [max([c["dino_cls"] for c in r["candidates"]], default=0.0) for r in pos]
        neg_best = [max([c["dino_cls"] for c in r["candidates"]], default=0.0) for r in neg]
        summary[target] = {
            "target": target, "asset_class": rs[0]["asset_class"], "dino_thresh": rs[0]["dino_thresh"],
            "gate": f"{rs[0]['gate_engine']}/{rs[0]['gate_mode']}",
            "n_pos": len(pos), "n_neg": len(neg),
            "stage1_ceiling": _frac([r["stage1_covers_gt"] for r in pos]),
            "stage1_ceiling_gd": _frac([r["stage1_covers_gt_gd"] for r in pos]),
            "stage1_ceiling_owlv2": _frac([r["stage1_covers_gt_owlv2"] for r in pos]),
            "stage2_good": round(len(s2_good) / max(1, len(pos)), 3),
            "stage2_fp": round(len(s2_fp) / max(1, len(neg)), 3),
            "final_recall": round(len(found_pos) / max(1, len(pos)), 3),
            "final_good": round(len(good) / max(1, len(pos)), 3),
            "final_fp": round(len(fp) / max(1, len(neg)), 3),
            "dino_pos_med": round(float(np.median(pos_best)), 3) if pos_best else 0.0,
            "dino_neg_med": round(float(np.median(neg_best)), 3) if neg_best else 0.0,
        }
    return summary


def _cand_center_in(cand, gt):
    return gt is not None and center_in_box(box_center(cand["box"]), gt)


def _frac(bools):
    bools = [b for b in bools if b is not None]
    return round(sum(1 for b in bools if b) / max(1, len(bools)), 3)


def print_table(summary):
    cols = [("target", 11), ("asset", 12), ("dThr", 6), ("n_pos", 6), ("S1ceil", 8), ("S1_gd", 7),
            ("S1_owl", 8), ("S2good", 8), ("S2fp", 7), ("good", 6), ("FP", 6), ("dPOS", 6), ("dNEG", 6)]
    print("\n" + "=" * 108)
    print("  CASCADE SUMMARY   S1ceil = max recall (a proposal covers GT). good = final on-target.")
    print("  S2good/S2fp = after DINOv2, before geometry. dPOS/dNEG = best-candidate DINO cosine")
    print("  median on positives vs negatives -- the gap IS the verifier's discrimination signal.")
    print("=" * 108)
    print("  " + "".join(h.ljust(w) for h, w in cols))
    print("  " + "-" * 104)
    for v in summary.values():
        row = [v["target"], v["asset_class"], f"{v['dino_thresh']:.2f}", str(v["n_pos"]),
               f"{v['stage1_ceiling']:.2f}", f"{v['stage1_ceiling_gd']:.2f}",
               f"{v['stage1_ceiling_owlv2']:.2f}", f"{v['stage2_good']:.2f}", f"{v['stage2_fp']:.2f}",
               f"{v['final_good']:.2f}", f"{v['final_fp']:.2f}",
               f"{v['dino_pos_med']:.2f}", f"{v['dino_neg_med']:.2f}"]
        print("  " + "".join(c.ljust(w) for c, (_, w) in zip(row, cols)))
    print("=" * 108 + "\n")


# ==============================================================================
# CLI -> target specs
# ==============================================================================
def parse_targets(args):
    if args.targets:
        tp = _resolve(args.targets)
        if not tp.exists():
            raise SystemExit(f"--targets file not found: {tp}")
        doc = yaml.safe_load(tp.read_text(encoding="utf-8")) or {}
        shared_neg = doc.get("negatives")
        specs = []
        for t in doc.get("targets", []):
            for req in ("text", "ref", "frames", "asset_class"):
                if req not in t:
                    raise SystemExit(f"--targets entry missing '{req}': {t}")
            specs.append(TargetSpec(
                name=t.get("name") or Path(t["frames"]).name,
                text=t["text"], ref_path=t["ref"], frames_dir=t["frames"],
                asset_class=resolve_asset_class(t["asset_class"]),
                labels_path=t.get("labels"), negatives_dir=t.get("negatives", shared_neg)))
        if not specs:
            raise SystemExit(f"{tp} has no 'targets:' entries.")
        return specs

    if not (args.text and args.ref and args.asset_class and args.frames):
        raise SystemExit("provide a single target (--text --ref --asset-class --frames) "
                         "OR a batch file (--targets <yaml>).")
    return [TargetSpec(
        name=args.name or Path(args.frames).name,
        text=args.text, ref_path=args.ref, frames_dir=args.frames,
        asset_class=resolve_asset_class(args.asset_class),
        labels_path=args.labels, negatives_dir=args.negatives)]


# ==============================================================================
# Main
# ==============================================================================
def main():
    ap = argparse.ArgumentParser(
        description="Generalized cascade detector — any target via text + ref + asset class.")
    # single-target
    ap.add_argument("--text", help="GroundingDINO grounding phrase, e.g. \"a rifle\"")
    ap.add_argument("--ref", help="reference crop image (OWLv2 query + DINOv2 template)")
    ap.add_argument("--asset-class", help=f"one of {[a.value for a in AssetClass]}")
    ap.add_argument("--frames", help="folder of frames to search")
    ap.add_argument("--labels", default=None, help="GT labels.json (default <frames>/labels.json)")
    ap.add_argument("--negatives", default=None, help="optional target-absent frames (FP metric)")
    ap.add_argument("--name", default=None, help="target name (default = frames-dir basename)")
    # batch
    ap.add_argument("--targets", default=None, help="YAML listing multiple target specs")
    # shared
    ap.add_argument("--out", default="OUTPUT/cascade", help="output dir (under repo)")
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--config", default=None)
    ap.add_argument("--no-overlays", action="store_true")
    ap.add_argument("--max-frames", type=int, default=None,
                    help="run only N positives per target and skip negatives (quick smoke check)")
    args = ap.parse_args()

    if not torch.cuda.is_available():
        raise SystemExit("CUDA not available — cascade is GPU-only (NO SILENT FALLBACKS).")

    cfg = {}
    cfg_path = Path(args.config) if args.config else REPO / "config.yaml"
    if cfg_path.exists():
        cfg = yaml.safe_load(cfg_path.read_text(encoding="utf-8")) or {}
    owlv2_id = (cfg.get("models", {}).get("owlv2", {}) or {}).get(
        "hf_id", "google/owlv2-base-patch16-ensemble")

    specs = parse_targets(args)
    print(f"[cascade] loading {len(specs)} target(s) ...", flush=True)
    loaded = []
    for spec in specs:
        ref, positives, negatives = load_spec(spec, max_frames=args.max_frames)
        p = ASSET_CLASS_PARAMS[spec.asset_class]
        print(f"[cascade]   {spec.name}: {len(positives)} pos / {len(negatives)} neg | "
              f"ref {ref.shape[1]}x{ref.shape[0]} | text='{spec.text}' | "
              f"class={spec.asset_class.value} dino>={p['dino_thresh']} "
              f"gate={p['gate_engine']}/{p['gate_mode']}", flush=True)
        loaded.append((spec, ref, positives, negatives))
    if args.max_frames:
        print(f"[cascade] SMOKE: max {args.max_frames} positives/target, negatives skipped.", flush=True)

    samples, refs = build_samples(loaded)
    t0 = time.time()
    stage1_propose(samples, refs, args.device, owlv2_id)
    stage2_verify(samples, refs, args.device)
    stage3_geometric(samples, refs, args.device)
    decide(samples)
    print(f"\n[cascade] pipeline done in {time.time()-t0:.1f}s over {len(samples)} samples.", flush=True)

    out_dir = REPO / args.out
    out_dir.mkdir(parents=True, exist_ok=True)
    records = [record(s) for s in samples]
    if not args.no_overlays:
        for s in samples:
            sub = s["target"] if s["is_pos"] else neg_subdir(s["target"])
            save_overlay(out_dir / "overlays" / sub, s)

    summary = summarize(records)
    print_table(summary)
    (out_dir / "per_frame.json").write_text(json.dumps(records, indent=2), encoding="utf-8")
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"[cascade] wrote {out_dir/'summary.json'} + per_frame.json"
          + ("" if args.no_overlays else f" + overlays/ under {out_dir}"))
    print("[cascade] build the visual report: venv\\Scripts\\python.exe cascade_report.py")


if __name__ == "__main__":
    main()
