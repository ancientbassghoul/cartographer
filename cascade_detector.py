"""cascade_detector.py — verified two/three-stage cascade to find the poster + rifle.

Per PROGRESS.md, every single-shot engine (SIFT/LightGlue/DINOv2-dense/OWLv2/Qwen) over-fired
because it conflated "where is a candidate" with "is this THE target". This script keeps the
cascade paradigm (propose wide -> verify ruthlessly) but is driven by our own benchmark numbers:

  STAGE 1  Propose (recall ceiling)
      GroundingDINO (text-guided) + OWLv2 image-guided (image-guided), POOLED, low thresholds.
      The cascade can never beat this stage's recall, so we report each proposer's standalone
      recall first. OWLv2 already hit rifle recall 1.0 (FP 0.93) in the bench; the verifiers exist
      to kill that FP.

  STAGE 2  Verify (DINOv2 crop embedding)
      Letterbox (NEVER squash -- the rifle ref is 168x447) each candidate crop AND the reference to
      a square 224x224, embed with DINOv2 ViT-S/14, cosine-compare (CLS + mean-pool both reported).
      Survivor gate = cosine threshold chosen from the printed pos/neg separation, not hardcoded.

  STAGE 3  Geometric (per-target asymmetric -- user decision)
      Planar poster  = HARD gate: SIFT + RANSAC homography inliers >= SIFT_MIN_INLIERS (bench: 0 FP).
      3D rifle       = SOFT bonus: LightGlue inliers reported as confidence; NEVER vetoes (bench:
                       LightGlue rifle recall 0.23 -> a hard gate would destroy recall).

Output: OUTPUT/cascade/{per_frame.json, summary.json} + per-stage composite overlays/. Build the
shareable HTML with cascade_report.py afterwards.

GPU-only, NO SILENT FALLBACKS (CLAUDE.md): CUDA + each model load fail-fast; image resizes are
logged (image-integrity rule). Models load ONE STAGE AT A TIME and are freed, so peak VRAM is small.

  venv\\Scripts\\python.exe cascade_detector.py                 # full run, both targets
  venv\\Scripts\\python.exe cascade_detector.py --max-frames 2  # quick smoke (skips negatives)
"""

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import torch
import yaml

# Reuse the proven, pure helpers + geometric engines from the benchmark harness (its main() is
# __main__-guarded, so importing is side-effect-free).
from benchmark_detectors import (
    REF_TEXT,
    LightGlueDetector,
    SiftDetector,
    center_in_box,
    iou,
    load_dataset,
)

REPO = Path(__file__).resolve().parent

# --- targets whose Stage-3 geometric check is a HARD gate (planar) vs SOFT bonus (3D) ----------
PLANAR_TARGETS = {"Nasrallah"}      # poster: SIFT homography hard-gates; everything else is soft

# --- Stage-1 proposer tunables (recall-first; the summary prints the recall ceiling) ----------
GD_BOX_THRESH = 0.25      # GroundingDINO box confidence -- intentionally low to cast a wide net
GD_TEXT_THRESH = 0.25     # GroundingDINO text-token confidence
GD_TOPK = 8               # keep at most this many GD boxes per frame
OWLV2_TOPK = 5            # keep this many OWLv2 image-guided anchors per frame (by abs logit)
OWLV2_MIN_LOGIT = 2.0     # floor on the OWLv2 abs image-guided logit to be a candidate
NMS_IOU = 0.7             # per-source dedup of overlapping proposals

# --- Stage-2 DINOv2 verifier tunables ----------------------------------------------------------
DINO_SIZE = 224           # square side fed to DINOv2 (16x16 patch grid at /14)
DINO_PAD = 114            # neutral letterbox fill (mid-gray), applied to ref AND candidate alike
DINO_THRESH = 0.50        # CLS-cosine survivor gate (TUNE from the printed pos/neg separation)

# --- Stage-3 geometric tunables ----------------------------------------------------------------
SIFT_MIN_INLIERS = 12     # poster HARD gate: RANSAC inliers to accept (bench: clear poster 40-67)
GEOM_MIN_CROP = 24        # crops smaller than this skip geometric matching (explicit, logged)

PROPOSAL_HIT_IOU = 1e-6   # a proposal "covers" GT if IoU > 0 (recall-ceiling definition)


# ==============================================================================
# Image helpers
# ==============================================================================
def letterbox(rgb, size=DINO_SIZE, pad=DINO_PAD):
    """Aspect-PRESERVING resize into a square `size` canvas with neutral padding.

    NEVER squashes: the rifle ref is 168x447, so a force-resize to a square would morphologically
    distort it (thin barrel -> stubby) and tank the cosine. We scale-to-fit and pad the remainder.
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
def build_samples(positives, negatives):
    """One sample per (target, frame). Negatives are evaluated against BOTH targets (the GD text
    and OWLv2 ref differ per target), mirroring benchmark_detectors' per-ref negative pass."""
    samples = []
    for target, items in positives.items():
        for fname, rgb, gt in items:
            samples.append({"target": target, "frame": fname, "rgb": rgb,
                            "gt": gt, "is_pos": True, "cands": []})
        for fname, rgb in negatives:
            samples.append({"target": target, "frame": fname, "rgb": rgb,
                            "gt": None, "is_pos": False, "cands": []})
    return samples


def stage1_propose(samples, refs, device, owlv2_id, max_frames):
    print("\n[cascade] === STAGE 1: propose (GroundingDINO + OWLv2) ===", flush=True)
    gd = GroundingDinoProposer(device=device)
    for s in samples:
        s["cands"] += gd.propose(s["rgb"], REF_TEXT[s["target"]])
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
            c["stage2"] = c["dino_cls"] >= DINO_THRESH
    del dino
    torch.cuda.empty_cache()


def stage3_geometric(samples, refs, device):
    print("\n[cascade] === STAGE 3: geometric (SIFT hard=poster / LightGlue soft=rifle) ===",
          flush=True)
    sift = SiftDetector()
    lg = LightGlueDetector(device=device)
    for s in samples:
        planar = s["target"] in PLANAR_TARGETS
        for c in s["cands"]:
            c["geom_inliers"] = 0
            c["geom_engine"] = "sift" if planar else "lightglue"
            c["geom_note"] = None
            if not c.get("stage2"):
                continue                                  # only verified survivors reach geometry
            crop = crop_of(s["rgb"], c["box"])
            if min(crop.shape[:2]) < GEOM_MIN_CROP:
                c["geom_note"] = "crop_too_small"        # explicit, visible -- NOT a silent skip
                c["stage3_hard"] = not planar             # soft target unaffected; planar fails gate
                continue
            engine = sift if planar else lg
            r = engine.detect(refs[s["target"]], crop)
            c["geom_inliers"] = int(r["score"])
            # Planar = HARD gate; 3D = SOFT (never vetoes a Stage-2 survivor).
            c["stage3_hard"] = (c["geom_inliers"] >= SIFT_MIN_INLIERS) if planar else True
    del sift, lg
    torch.cuda.empty_cache()


def decide(samples):
    """Final per-frame decision + ranking among accepted candidates."""
    for s in samples:
        planar = s["target"] in PLANAR_TARGETS
        accepted = [c for c in s["cands"]
                    if c.get("stage2") and (c.get("stage3_hard") if planar else True)]
        for c in s["cands"]:
            c["accepted"] = c in accepted
            c["reject_reason"] = _reject_reason(c, planar)
        if accepted:
            # Rank survivors by appearance (DINOv2 cosine) FIRST, geometric inliers as tie-break,
            # for BOTH target types. Geometry-first mis-ranked the poster: a loose off-target box
            # with marginally more SIFT inliers (e.g. 28 vs 24) beat the near-exact poster box that
            # had a much higher DINOv2 score (0.82 vs 0.55). For the planar poster SIFT remains the
            # hard GATE above (in stage3_geometric); here it only breaks ties among verified survivors.
            best = max(accepted, key=lambda c: (c["dino_cls"], c["geom_inliers"]))
            s["final"] = best
        else:
            s["final"] = None


def _reject_reason(c, planar):
    if c.get("accepted"):
        return None
    if not c.get("stage2"):
        return f"stage2_dino<{DINO_THRESH}"
    if planar and not c.get("stage3_hard"):
        note = c.get("geom_note")
        return f"stage3_sift<{SIFT_MIN_INLIERS}" + (f"({note})" if note else "")
    return "not_best"


# ==============================================================================
# Overlays
# ==============================================================================
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
    tag = (f"{s['target']} {s['frame']} | cands={len(s['cands'])} "
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
        pos = [r for r in records if r["target"] == target and r["is_pos"]]
        neg = [r for r in records if r["target"] == target and not r["is_pos"]]
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
            "target": target, "n_pos": len(pos), "n_neg": len(neg),
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
    cols = [("target", 11), ("n_pos", 6), ("S1ceil", 8), ("S1_gd", 7), ("S1_owl", 8),
            ("S2good", 8), ("S2fp", 7), ("good", 6), ("FP", 6), ("dPOS", 6), ("dNEG", 6)]
    print("\n" + "=" * 92)
    print("  CASCADE SUMMARY   S1ceil = max recall (a proposal covers GT). good = final on-target.")
    print("  S2good/S2fp = after DINOv2, before geometry. dPOS/dNEG = best-candidate DINO cosine")
    print("  median on positives vs negatives -- the gap IS the verifier's discrimination signal.")
    print("=" * 92)
    print("  " + "".join(h.ljust(w) for h, w in cols))
    print("  " + "-" * 88)
    for v in summary.values():
        row = [v["target"], str(v["n_pos"]),
               f"{v['stage1_ceiling']:.2f}", f"{v['stage1_ceiling_gd']:.2f}",
               f"{v['stage1_ceiling_owlv2']:.2f}", f"{v['stage2_good']:.2f}", f"{v['stage2_fp']:.2f}",
               f"{v['final_good']:.2f}", f"{v['final_fp']:.2f}",
               f"{v['dino_pos_med']:.2f}", f"{v['dino_neg_med']:.2f}"]
        print("  " + "".join(c.ljust(w) for c, (_, w) in zip(row, cols)))
    print("=" * 92 + "\n")


# ==============================================================================
# Main
# ==============================================================================
def main():
    ap = argparse.ArgumentParser(description="Verified cascade detector for the poster + rifle.")
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

    print("[cascade] loading dataset ...", flush=True)
    refs, positives, negatives = load_dataset()
    for name, items in positives.items():
        print(f"[cascade]   {name}: {len(items)} positives | ref "
              f"{refs[name].shape[1]}x{refs[name].shape[0]} | text='{REF_TEXT.get(name)}'")
    print(f"[cascade]   None: {len(negatives)} negatives")

    if args.max_frames:
        positives = {t: items[:args.max_frames] for t, items in positives.items()}
        negatives = []                                   # smoke check: skip negatives
        print(f"[cascade] SMOKE: {args.max_frames} positives/target, negatives skipped.", flush=True)

    samples = build_samples(positives, negatives)
    t0 = time.time()
    stage1_propose(samples, refs, args.device, owlv2_id, args.max_frames)
    stage2_verify(samples, refs, args.device)
    stage3_geometric(samples, refs, args.device)
    decide(samples)
    print(f"\n[cascade] pipeline done in {time.time()-t0:.1f}s over {len(samples)} samples.", flush=True)

    out_dir = REPO / args.out
    out_dir.mkdir(parents=True, exist_ok=True)
    records = [record(s) for s in samples]
    if not args.no_overlays:
        for s in samples:
            sub = s["target"] if s["is_pos"] else f"None_vs_{s['target']}"
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
