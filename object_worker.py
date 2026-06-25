"""object_worker.py — Process P4: target detection (OWLv2 image-guided | Qwen2.5-VL).

The gradable-core detector. Subscribes to the io_bridge frame bus (CONFLATE newest-wins)
and, in its **own process / CUDA context**, grounds a designated target object in the live
frame. Two visible, mutually-exclusive detector paths selected by `runtime.object_mode`:

  * **OWLV2** (default): **OWLv2 image-guided (one-shot) detection** — query = the reference
    crop, find visually-matching regions in each frame, take the single highest-score box.
    Keys off the actual target *image*, so it discriminates a printed poster from painted
    murals where a text label cannot. Replaced Qwen after Qwen-3B-4bit proved unreliable for
    this small-object, mural-competing grounding task (5 finds but all on murals / a framed
    photo, slow ~5-6 s/found, degenerate `!!!` at full-res).
  * **QWEN**: **Qwen2.5-VL-3B (4-bit)** label-driven grounding (kept intact, not removed).

NO SILENT FALLBACKS: the active mode is the visible `object_mode` flag in every payload + log;
switching detectors is a config edit (`runtime.object_mode`), never an automatic runtime swap.

Why a separate process (not folded into perception_worker): Qwen-VL generation is
autoregressive and slow (≈1-3 s/detection). Running it inside the SLAM loop would stall
tracking and drop keyframes. Here it gets its own GPU scheduling; VRAM is the only shared
budget (Qwen 4-bit ≈2.6 GB on top of SLAM+depth ≈7.6 GB of 16). Detection runs *continuously*
but throttled to `perception.object_cadence_hz` (start conservative) — this is the eventual
Phase-2 autonomy mode (no human to press a hotkey), so we surface VRAM/latency cost now.

Input: a **provided reference crop** of the target (`models.qwen_vl.reference_crop`), loaded
at startup. The prompt is multi-image — [reference crop, live frame] — asking Qwen to locate
that same object in the live frame and emit a bounding box as JSON. Coordinates come back in
Qwen's smart-resized pixel space for the *live* image; we rescale them to the 512x288 frame
using `image_grid_thw` (resized dims = grid_patches * 14).

Output: publishes TOPIC_DETECTION on its own state bus (`object_state_port`, default :5604):
  {object_mode, frame_id, sim_time, found, bbox[x1,y1,x2,y2], center[cx,cy], infer_ms, raw}
bbox/center are in 512x288 frame pixels (or null when the target is not seen). These feed the
3D lift (back-project the center through the SLAM pose + pointmap) downstream.

NO SILENT FALLBACKS (per CLAUDE.md): CUDA + the Qwen load are asserted up front; any failure
raises. There is no CPU path and no auto-swap to a different detector. `object_mode="QWEN"` is
the visible state flag in every payload; a DINOv2 fallback would be approval-gated and would
flip this flag (it is NOT implemented here).
"""

import argparse
import json
import os
import re
import time

import cv2
import numpy as np
import torch
import yaml

import frame_bus

REPO = os.path.dirname(os.path.abspath(__file__))

# Visible NO-FALLBACK state flag (mirrors runtime.object_mode in config.yaml). Set once at
# startup from config by set_object_mode(); every payload + log line reflects the active path.
# {"QWEN_OWLV2" (default cascade), "OWLV2", "QWEN"}.
OBJECT_MODE = "QWEN_OWLV2"
VALID_OBJECT_MODES = ("QWEN_OWLV2", "OWLV2", "QWEN")


def set_object_mode(cfg) -> str:
    """Read the visible detector flag from config (runtime.object_mode) and pin the module
    global so build_payload/render/logs all report the active path. Fail-fast on an unknown
    mode (NO SILENT FALLBACKS — a typo must crash, not silently pick a default)."""
    global OBJECT_MODE
    mode = str(cfg.get("runtime", {}).get("object_mode", "QWEN_OWLV2")).strip().upper()
    if mode not in VALID_OBJECT_MODES:
        raise ValueError(f"unknown runtime.object_mode {mode!r} (expected one of "
                         f"{VALID_OBJECT_MODES}; NO SILENT FALLBACKS — fix config).")
    OBJECT_MODE = mode
    return mode

# Qwen2.5-VL vision patch size: smart_resize rounds each side to a multiple of patch*merge
# (14*2=28); grounding coords are in the resized-image pixel space = grid_patches * PATCH_PX.
PATCH_PX = 14

# Asked once at startup to turn the provided reference crop into a short text label. Qwen
# grounds a *described* object far more reliably than it matches a raw reference image (verified:
# image-only matching of a small target returns []; label-driven grounding lands the box), so the
# crop is the source of truth and the label is derived from it.
LABEL_PROMPT = (
    "This image is a reference crop of a single target object. In 3-6 words, name the target "
    "object as a short noun phrase suitable for an object detector. Output only the phrase."
)


def build_prompt(label: str) -> str:
    """Grounding prompt for one live frame. Strict JSON bbox so parsing is unambiguous; an
    empty list is an explicit, first-class 'not visible' answer (not a forced hallucinated box).

    The reference crop is passed as the FIRST image (a visual aid) and the live frame SECOND;
    the derived `label` is what actually drives the grounding.
    """
    # NOTE: keep this short and direct. A verbose prompt that double-emphasizes the empty case
    # makes the 3B model collapse to [] (verified — the wordy variant returned [] on a frame the
    # simple variant boxed correctly). The reference crop rides along as the FIRST image.
    return (
        f"Locate the {label} in the SECOND image. "
        f'Output JSON [{{"bbox_2d": [x1, y1, x2, y2]}}] in pixel coordinates of the SECOND image, '
        f"or [] if it is not visible."
    )


def load_config(path=None):
    path = path or os.path.join(REPO, "config.yaml")
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


# ==============================================================================
# Qwen2.5-VL detector
# ==============================================================================
class QwenDetector:
    """Wraps Qwen2.5-VL-3B (4-bit) grounding. Fail-fast load; returns a bbox in frame pixels.

    `detect(ref_rgb, frame_rgb)` takes two HxWx3 uint8 RGB arrays (reference crop + live
    frame) and returns a dict {found, bbox, center, raw}, bbox/center in `frame_rgb` pixels.
    """

    def __init__(self, hf_id: str, quantization: str = "4bit",
                 max_new_tokens: int = 256, device: str = "cuda"):
        assert torch.cuda.is_available(), (
            "CUDA not available — object_worker requires the GPU. "
            "No CPU fallback (NO SILENT FALLBACKS)."
        )
        from transformers import (
            Qwen2_5_VLForConditionalGeneration, AutoProcessor, BitsAndBytesConfig,
        )

        self.device = device
        self.hf_id = hf_id
        self.max_new_tokens = max_new_tokens
        self.object_mode = OBJECT_MODE

        quant_cfg = None
        if quantization == "4bit":
            quant_cfg = BitsAndBytesConfig(
                load_in_4bit=True, bnb_4bit_quant_type="nf4",
                bnb_4bit_compute_dtype=torch.float16,
            )
        elif quantization not in (None, "none", "fp16"):
            raise ValueError(f"unsupported qwen quantization '{quantization}' "
                             "(NO SILENT FALLBACKS — fix config).")

        t0 = time.time()
        self.model = Qwen2_5_VLForConditionalGeneration.from_pretrained(
            hf_id, quantization_config=quant_cfg, device_map="auto",
            torch_dtype=torch.float16,
        ).eval()
        self.processor = AutoProcessor.from_pretrained(hf_id)
        torch.cuda.synchronize()
        print(f"[object] Qwen2.5-VL '{hf_id}' ({quantization}) loaded in {time.time()-t0:.1f}s "
              f"| VRAM {torch.cuda.memory_allocated()/1e9:.2f} GB", flush=True)

    def _generate(self, content, max_new_tokens):
        """Run one chat-completion over a list of message-content dicts. Returns (text, inputs)."""
        from qwen_vl_utils import process_vision_info

        messages = [{"role": "user", "content": content}]
        text = self.processor.apply_chat_template(
            messages, tokenize=False, add_generation_prompt=True)
        image_inputs, video_inputs = process_vision_info(messages)
        inputs = self.processor(text=[text], images=image_inputs, videos=video_inputs,
                                padding=True, return_tensors="pt").to(self.model.device)
        with torch.inference_mode():
            gen = self.model.generate(**inputs, max_new_tokens=max_new_tokens, do_sample=False)
        trimmed = [out[len(inp):] for inp, out in zip(inputs.input_ids, gen)]
        out = self.processor.batch_decode(
            trimmed, skip_special_tokens=True, clean_up_tokenization_spaces=False)[0].strip()
        return out, inputs

    def derive_label(self, ref_rgb: np.ndarray) -> str:
        """Turn the provided reference crop into a short text label (one-time, at startup)."""
        from PIL import Image
        content = [{"type": "image", "image": Image.fromarray(ref_rgb)},
                   {"type": "text", "text": LABEL_PROMPT}]
        label, _ = self._generate(content, max_new_tokens=32)
        # Keep it a clean one-liner; strip quotes/trailing punctuation.
        label = label.splitlines()[0].strip().strip('".\'').strip()
        return label

    def detect(self, ref_rgb: np.ndarray, frame_rgb: np.ndarray, label: str) -> dict:
        from PIL import Image

        content = [
            {"type": "image", "image": Image.fromarray(ref_rgb)},
            {"type": "image", "image": Image.fromarray(frame_rgb)},
            {"type": "text", "text": build_prompt(label)},
        ]
        raw, inputs = self._generate(content, max_new_tokens=self.max_new_tokens)

        # Resized pixel dims Qwen actually saw for the SECOND image (the live frame). grid_thw
        # is (n_images, 3) = [t, h_patches, w_patches]; resized side = patches * PATCH_PX.
        grid = inputs["image_grid_thw"]
        rh = int(grid[1][1].item()) * PATCH_PX
        rw = int(grid[1][2].item()) * PATCH_PX
        H, W = frame_rgb.shape[:2]

        bbox = self._parse_bbox(raw)
        if bbox is None:
            return {"found": False, "bbox": None, "center": None, "raw": raw}

        sx, sy = W / rw, H / rh
        x1, y1, x2, y2 = bbox
        x1, x2 = sorted((x1 * sx, x2 * sx))
        y1, y2 = sorted((y1 * sy, y2 * sy))
        x1 = float(np.clip(x1, 0, W - 1)); x2 = float(np.clip(x2, 0, W - 1))
        y1 = float(np.clip(y1, 0, H - 1)); y2 = float(np.clip(y2, 0, H - 1))
        center = [round((x1 + x2) / 2, 1), round((y1 + y2) / 2, 1)]
        return {
            "found": True,
            "bbox": [round(x1, 1), round(y1, 1), round(x2, 1), round(y2, 1)],
            "center": center, "raw": raw,
        }

    @staticmethod
    def _parse_bbox(raw: str):
        """Pull the first [x1,y1,x2,y2] out of Qwen's reply. Returns 4 floats or None.

        Tries strict JSON first (the requested format), then falls back to the first run of
        four numbers — a *parsing* tolerance for format drift, NOT a model/behavior fallback.
        """
        # Strict JSON object with a bbox_2d field.
        for m in re.finditer(r'"bbox_2d"\s*:\s*\[([^\]]+)\]', raw):
            nums = re.findall(r'-?\d+\.?\d*', m.group(1))
            if len(nums) >= 4:
                return [float(n) for n in nums[:4]]
        # Any 4-number bracketed list.
        m = re.search(r'\[\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*,\s*'
                      r'(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)', raw)
        if m:
            return [float(g) for g in m.groups()]
        return None


# ==============================================================================
# OWLv2 image-guided detector (default path)
# ==============================================================================
class Owlv2Detector:
    """Wraps OWLv2 image-guided (one-shot) detection. Fail-fast load; box in frame pixels.

    `detect(ref_rgb, frame_rgb, label=None)` queries with the reference crop and returns the
    single highest-score box in `frame_rgb` pixels. `label` is accepted for interface parity
    with QwenDetector but unused (OWLv2 keys off the crop image, not a text label).

    NOTE: image-guided scores are NOT 0-1 calibrated and can cluster near 1.0 across many
    boxes; we keep only the top-scoring one above `score_thresh` (de-risk spike confirmed the
    top box lands on the poster, not the murals). Tune `score_thresh` if it false-positives.
    """

    def __init__(self, hf_id: str, score_thresh: float = 0.9, nms_thresh: float = 0.3,
                 device: str = "cuda"):
        assert torch.cuda.is_available(), (
            "CUDA not available — object_worker requires the GPU. "
            "No CPU fallback (NO SILENT FALLBACKS)."
        )
        from transformers import Owlv2Processor, Owlv2ForObjectDetection

        self.device = device
        self.hf_id = hf_id
        self.score_thresh = float(score_thresh)
        self.nms_thresh = float(nms_thresh)
        self.object_mode = OBJECT_MODE

        t0 = time.time()
        self.processor = Owlv2Processor.from_pretrained(hf_id)
        self.model = Owlv2ForObjectDetection.from_pretrained(hf_id).to(device).eval()
        torch.cuda.synchronize()
        print(f"[object] OWLv2 '{hf_id}' loaded in {time.time()-t0:.1f}s "
              f"| score_thresh {self.score_thresh} nms {self.nms_thresh} "
              f"| VRAM {torch.cuda.memory_allocated()/1e9:.2f} GB", flush=True)

    def derive_label(self, ref_rgb: np.ndarray) -> str:
        """OWLv2 needs no text label (image-guided). Returns a fixed tag for display/logging."""
        return "(owlv2 image-guided)"

    def detect(self, ref_rgb: np.ndarray, frame_rgb: np.ndarray, label=None) -> dict:
        H, W = frame_rgb.shape[:2]
        inp = self.processor(images=frame_rgb, query_images=ref_rgb,
                             return_tensors="pt").to(self.device)
        with torch.inference_mode():
            out = self.model.image_guided_detection(**inp)
        res = self.processor.post_process_image_guided_detection(
            out, target_sizes=torch.tensor([[H, W]], device=self.device),
            threshold=self.score_thresh, nms_threshold=self.nms_thresh)[0]

        boxes = res["boxes"].detach().cpu().numpy()
        scores = res["scores"].detach().cpu().numpy()
        if len(boxes) == 0:
            return {"found": False, "bbox": None, "center": None,
                    "raw": f"owlv2: no box >= {self.score_thresh}"}

        i = int(np.argmax(scores))
        x1, y1, x2, y2 = boxes[i]
        x1, x2 = sorted((float(x1), float(x2)))
        y1, y2 = sorted((float(y1), float(y2)))
        x1 = float(np.clip(x1, 0, W - 1)); x2 = float(np.clip(x2, 0, W - 1))
        y1 = float(np.clip(y1, 0, H - 1)); y2 = float(np.clip(y2, 0, H - 1))
        center = [round((x1 + x2) / 2, 1), round((y1 + y2) / 2, 1)]
        return {
            "found": True,
            "bbox": [round(x1, 1), round(y1, 1), round(x2, 1), round(y2, 1)],
            "center": center,
            "raw": f"owlv2 top score={scores[i]:.3f} of {len(boxes)} boxes",
        }

    def verify_logit(self, crop_rgb: np.ndarray, ref_rgb: np.ndarray) -> float:
        """Image-guided similarity of a candidate crop to the reference, as the **absolute**
        max logit (NOT the post-processed score, which self-normalizes to ~1.0 on every image
        and so can't separate a true match from a distractor). On the dev poster: a real-poster
        crop scores ≈7, a mural/sign crop ≈3.7 — so a fixed threshold (~5.0) is the precision
        gate in the cascade. Returns -inf for an empty crop (cannot verify → reject)."""
        if crop_rgb is None or crop_rgb.size == 0:
            return float("-inf")
        inp = self.processor(images=crop_rgb, query_images=ref_rgb,
                             return_tensors="pt").to(self.device)
        with torch.inference_mode():
            out = self.model.image_guided_detection(**inp)
        return float(out.logits[0, :, 0].max())


# ==============================================================================
# Cascade detector (default path): Qwen proposes @512, OWLv2 verifies the crop
# ==============================================================================
class CascadeDetector:
    """Two-stage "both-positive" detector — the user-approved fix for the false-positive problem.

    Stage 1 (recall): **4-bit Qwen @ 512** grounds the target label and proposes a box. Qwen is
    only numerically stable at 512 (it degenerates to `!!!` at 720p, *worst on the target frame*),
    so we always run it at 512 regardless of the resolution we are fed.
    Stage 2 (precision): **OWLv2** verifies the proposed crop against the reference image via the
    absolute image-guided logit; a candidate is accepted only if logit >= `verify_thresh`. This
    rejects Qwen's mural / framed-photo false positives, which a text label can't distinguish.

    A detection is forwarded only when BOTH agree. This is an ensemble (both models always run,
    both required) — NOT a fallback. `detect(ref, frame, label)` returns the box in the **input
    frame's** pixel space, so the pipeline's `_to_transport` scales it correctly whether fed a
    512 or a hi-res frame.
    """

    def __init__(self, qwen: "QwenDetector", owlv2: "Owlv2Detector", verify_thresh: float = 5.0):
        self.qwen = qwen
        self.owlv2 = owlv2
        self.verify_thresh = float(verify_thresh)
        self.object_mode = OBJECT_MODE

    def derive_label(self, ref_rgb: np.ndarray) -> str:
        """Cascade still grounds via Qwen, so the label comes from the Qwen proposer."""
        return self.qwen.derive_label(ref_rgb)

    def detect(self, ref_rgb: np.ndarray, frame_rgb: np.ndarray, label: str) -> dict:
        H, W = frame_rgb.shape[:2]
        # Stage 1 — Qwen proposes at its stable resolution (512x288).
        if (W, H) != (512, 288):
            small = cv2.resize(frame_rgb, (512, 288), interpolation=cv2.INTER_AREA)
        else:
            small = frame_rgb
        qdet = self.qwen.detect(ref_rgb, small, label)  # box in 512 px
        if not qdet["found"] or qdet["bbox"] is None:
            return {"found": False, "bbox": None, "center": None,
                    "raw": f"cascade: qwen no proposal ({qdet['raw'][:60]})"}

        # Map the 512-space box up to the input frame's native pixels, crop at full fidelity.
        sx, sy = W / 512.0, H / 288.0
        x1, y1, x2, y2 = qdet["bbox"]
        bx1 = int(np.clip(round(x1 * sx), 0, W - 1)); bx2 = int(np.clip(round(x2 * sx), 0, W))
        by1 = int(np.clip(round(y1 * sy), 0, H - 1)); by2 = int(np.clip(round(y2 * sy), 0, H))
        crop = frame_rgb[by1:by2, bx1:bx2]

        # Stage 2 — OWLv2 verifies the crop against the reference image.
        logit = self.owlv2.verify_logit(crop, ref_rgb)
        if logit < self.verify_thresh:
            return {"found": False, "bbox": None, "center": None,
                    "raw": f"cascade: owlv2 reject logit={logit:.2f} < {self.verify_thresh} "
                           f"(qwen box {qdet['bbox']})"}

        bbox = [round(float(bx1), 1), round(float(by1), 1), round(float(bx2), 1), round(float(by2), 1)]
        center = [round((bbox[0] + bbox[2]) / 2, 1), round((bbox[1] + bbox[3]) / 2, 1)]
        return {"found": True, "bbox": bbox, "center": center,
                "raw": f"cascade qwen+owlv2 logit={logit:.2f} >= {self.verify_thresh}"}


# ==============================================================================
# Reference crop + payload + render
# ==============================================================================
def load_reference(cfg) -> np.ndarray:
    """Load the provided target reference crop (RGB). Fail-fast if missing."""
    rel = cfg["models"]["qwen_vl"]["reference_crop"]
    path = rel if os.path.isabs(rel) else os.path.join(REPO, rel)
    assert os.path.exists(path), (
        f"reference crop not found: {path} — set models.qwen_vl.reference_crop "
        "to the provided target asset (NO SILENT FALLBACKS).")
    bgr = cv2.imread(path, cv2.IMREAD_COLOR)
    assert bgr is not None, f"could not read reference crop: {path}"
    print(f"[object] reference crop: {path} {bgr.shape[1]}x{bgr.shape[0]}", flush=True)
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def build_payload(det, meta, infer_ms, cadence_hz, label):
    return {
        "object_mode": OBJECT_MODE,
        "target_label": label,
        "frame_id": meta.get("frame_id"),
        "mono_ts": meta.get("mono_ts"),
        "sim_time": meta.get("sim_time"),
        "controls": meta.get("controls"),
        "infer_ms": round(infer_ms, 1),
        "cadence_hz": cadence_hz,
        "found": det["found"],
        "bbox": det["bbox"],
        "center": det["center"],
        "raw": det["raw"][:200],
    }


DET_WINDOW = "Cartographer — object detection (Qwen)"


def render(frame_bgr, ref_rgb, det, infer_ms, label=""):
    """Compose [ reference crop | live frame + bbox ] with telemetry."""
    h, w = frame_bgr.shape[:2]
    panel = frame_bgr.copy()
    if det["found"] and det["bbox"]:
        x1, y1, x2, y2 = [int(round(v)) for v in det["bbox"]]
        cv2.rectangle(panel, (x1, y1), (x2, y2), (0, 0, 255), 2)
        cx, cy = [int(round(v)) for v in det["center"]]
        cv2.drawMarker(panel, (cx, cy), (0, 255, 255), cv2.MARKER_CROSS, 16, 2)
    tag = "TARGET" if det["found"] else "no target"
    status = f"{OBJECT_MODE} [{label}]  {tag}  infer={infer_ms:.0f}ms"
    col = (0, 255, 0) if det["found"] else (0, 165, 255)
    cv2.putText(panel, status, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)

    # Reference crop inset (left), matched to frame height.
    ref_bgr = cv2.cvtColor(ref_rgb, cv2.COLOR_RGB2BGR)
    ref_h = h
    ref_w = max(1, int(ref_bgr.shape[1] * ref_h / ref_bgr.shape[0]))
    ref_panel = cv2.resize(ref_bgr, (ref_w, ref_h), interpolation=cv2.INTER_AREA)
    cv2.putText(ref_panel, "reference", (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
    return np.hstack([ref_panel, panel])


# ==============================================================================
# Detector construction (shared by the cascade + the standalone diagnostic modes)
# ==============================================================================
def _build_qwen(cfg, q) -> "QwenDetector":
    return QwenDetector(
        q["hf_id"], quantization=q.get("quantization", "4bit"),
        max_new_tokens=int(q.get("max_new_tokens", 256)))


def _build_owlv2(cfg) -> "Owlv2Detector":
    o = cfg["models"]["owlv2"]
    return Owlv2Detector(
        o["hf_id"], score_thresh=float(o.get("score_thresh", 0.9)),
        nms_thresh=float(o.get("nms_thresh", 0.3)))


def _qwen_label(detector, q, ref_rgb) -> str:
    """The provided crop is the source of truth; the grounding label is taken from config if set,
    else derived from the crop once (Qwen grounds a described object far more reliably than it
    matches a raw reference image). Logged for transparency."""
    cfg_label = q.get("target_label")
    if cfg_label:
        label = str(cfg_label).strip()
        print(f"[object] target label (from config): {label!r}", flush=True)
    else:
        label = detector.derive_label(ref_rgb)
        print(f"[object] target label (derived from crop): {label!r}", flush=True)
    return label


# ==============================================================================
# Pipeline
# ==============================================================================
class Pipeline:
    def __init__(self, cfg):
        mode = set_object_mode(cfg)  # pins the module OBJECT_MODE flag from config
        q = cfg["models"]["qwen_vl"]
        self.cadence_hz = float(cfg["perception"]["object_cadence_hz"])
        self.min_interval = 1.0 / self.cadence_hz if self.cadence_hz > 0 else 0.0
        # The lift (perception_worker) works in the 512x288 transport space, so detections are
        # scaled back to it regardless of what (higher) resolution the detector grounded on.
        self.proc_w = int(cfg["perception"]["processing_width"])
        self.proc_h = int(cfg["perception"]["processing_height"])
        self.ref_rgb = load_reference(cfg)
        print(f"[object] === object_mode = {mode} ===", flush=True)

        if mode == "QWEN_OWLV2":
            # The cascade: Qwen (4-bit @512) proposes, OWLv2 verifies the crop. Both load.
            qwen = _build_qwen(cfg, q)
            owlv2 = _build_owlv2(cfg)
            self.detector = CascadeDetector(
                qwen, owlv2, verify_thresh=float(cfg["models"]["owlv2"].get("verify_thresh", 5.0)))
            self.label = _qwen_label(qwen, q, self.ref_rgb)  # Qwen still grounds, so it needs a label
            print(f"[object] cascade ready: Qwen proposer + OWLv2 verify "
                  f"(thresh {self.detector.verify_thresh})", flush=True)
        elif mode == "OWLV2":
            self.detector = _build_owlv2(cfg)
            # OWLv2 is image-guided (the crop IS the query) — no text label to derive.
            self.label = self.detector.derive_label(self.ref_rgb)
            print(f"[object] OWLv2 image-guided (no text label needed)", flush=True)
        else:  # QWEN
            self.detector = _build_qwen(cfg, q)
            self.label = _qwen_label(self.detector, q, self.ref_rgb)
        self.last_infer_mono = 0.0
        self.n_det = 0
        self.n_found = 0

    def _to_transport(self, det, src_w, src_h):
        """Rescale a detection (in the detection frame's pixels) to the 512x288 transport space
        the lift expects. Identity when the frame is already transport-sized."""
        if not det["found"] or det["bbox"] is None:
            return det
        sx, sy = self.proc_w / src_w, self.proc_h / src_h
        x1, y1, x2, y2 = det["bbox"]
        bbox = [round(x1 * sx, 1), round(y1 * sy, 1), round(x2 * sx, 1), round(y2 * sy, 1)]
        center = [round((bbox[0] + bbox[2]) / 2, 1), round((bbox[1] + bbox[3]) / 2, 1)]
        return {**det, "bbox": bbox, "center": center}

    def step(self, frame_bgr, meta, state_pub=None, show=True):
        """Run detection if the cadence is due. Returns (payload|None, panel|None)."""
        now = time.monotonic()
        if self.min_interval and (now - self.last_infer_mono) < self.min_interval:
            return None, None
        self.last_infer_mono = now

        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        h, w = frame_bgr.shape[:2]
        t0 = time.time()
        det = self.detector.detect(self.ref_rgb, frame_rgb, self.label)  # box in frame (hi-res) px
        infer_ms = (time.time() - t0) * 1000.0
        self.n_det += 1
        self.n_found += int(det["found"])

        # Publish the detection in transport (512x288) pixels for the lift; render on the hi-res frame.
        det_tx = self._to_transport(det, w, h)
        payload = build_payload(det_tx, meta, infer_ms, self.cadence_hz, self.label)
        if state_pub is not None:
            state_pub.publish(frame_bus.TOPIC_DETECTION, payload)

        c = (meta.get("controls") or {})
        print(f"[object] {OBJECT_MODE} frame {meta.get('frame_id')} | "
              f"{'TARGET '+str(det_tx['center']) if det['found'] else 'no target':<28} | "
              f"infer {infer_ms:6.0f} ms | found {self.n_found}/{self.n_det} | "
              f"src {w}x{h} | trigger {c.get('trigger')}", flush=True)

        panel = render(frame_bgr, self.ref_rgb, det, infer_ms, self.label) if show else None
        return payload, panel


# ==============================================================================
# Live loop / offline video / self-test
# ==============================================================================
def run_live(cfg, show=True):
    # Prefer the hi-res object stream (full pixel fidelity stabilizes grounding); fall back to the
    # 512x288 perception stream only if no hi-res port is configured.
    frame_port = cfg["network"].get("frame_bus_hires_port") or cfg["network"]["frame_bus_port"]
    obj_port = cfg["network"]["object_state_port"]
    pipe = Pipeline(cfg)
    frame_sub = frame_bus.FrameSubscriber(frame_port)
    state_pub = frame_bus.StatePublisher(obj_port)  # binds; fail-fast if taken
    print(f"[object] frame bus SUB :{frame_port} (hi-res) | detection PUB :{obj_port} (TOPIC_DETECTION)")
    print(f"[object] {OBJECT_MODE} continuous @ ~{pipe.cadence_hz:g} Hz (throttled)")
    print("[object] === READY === waiting for frames from io_bridge "
          "(focus a window, 'q' to quit).\n", flush=True)
    try:
        while True:
            got = frame_sub.recv(timeout_ms=500)
            if got is None:
                if show and (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
                continue
            frame, meta = got
            _, panel = pipe.step(frame, meta, state_pub, show)
            if show and panel is not None:
                cv2.imshow(DET_WINDOW, panel)
            if show and (cv2.waitKey(1) & 0xFF) == ord("q"):
                break
    except KeyboardInterrupt:
        pass
    finally:
        print("[object] shutting down ...")
        frame_sub.close()
        state_pub.close()
        if show:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass


def _video_frames(path, stride, max_frames, proc_w, proc_h):
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
            bgr = cv2.resize(bgr, (proc_w, proc_h), interpolation=cv2.INTER_AREA)
            meta = {"frame_id": yielded, "mono_ts": time.monotonic(),
                    "sim_time": round(src_idx / fps, 3), "controls": {}}
            yield bgr, meta
            yielded += 1
            if max_frames and yielded >= max_frames:
                break
        src_idx += 1
    cap.release()


def run_offline_video(cfg, video, show=False, stride=15, max_frames=0,
                      out_dir=None, publish=False):
    """Offline verification: run detection over a recording at the configured cadence.

    Saves an overlay PNG for every frame where the target is found, so you can eyeball the
    grounding without hardware. `--publish` also emits TOPIC_DETECTION on the state bus.
    """
    from pathlib import Path
    video = Path(video).resolve()
    assert video.exists(), f"recording not found: {video}"
    out_dir = Path(out_dir or os.path.join(REPO, "OUTPUT")).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    proc_w = cfg["perception"]["processing_width"]
    proc_h = cfg["perception"]["processing_height"]

    pipe = Pipeline(cfg)
    pipe.min_interval = 0.0  # offline: detect on every sampled frame (stride controls rate)
    state_pub = None
    if publish:
        state_pub = frame_bus.StatePublisher(cfg["network"]["object_state_port"])
        print(f"[object] OFFLINE --publish: detection PUB :{state_pub.port}")
    print(f"[object] OFFLINE video={video.name} stride={stride} "
          f"max_frames={max_frames or 'all'} | overlays -> {out_dir}")
    print("[object] === READY === scanning recording for the target.\n", flush=True)

    n = n_found = 0
    t0 = time.time()
    try:
        for frame, meta in _video_frames(video, stride, max_frames, proc_w, proc_h):
            payload, panel = pipe.step(frame, meta, state_pub, show=True)
            n += 1
            if payload and payload["found"]:
                n_found += 1
                cv2.imwrite(str(out_dir / f"{video.stem}_det_{meta['frame_id']:05d}.png"), panel)
            if show:
                cv2.imshow(DET_WINDOW, panel)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
    except KeyboardInterrupt:
        print("[object] interrupted")

    dt = time.time() - t0
    print(f"\n[object] DONE: {n} frames in {dt:.1f}s | target found in {n_found} | "
          f"peak VRAM {torch.cuda.max_memory_allocated()/1e9:.2f} GB")
    if state_pub is not None:
        state_pub.close()
    if show:
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass
    print("[object] OK")


def run_self_test(cfg):
    """Run detection once on a frame known to contain the target, save an overlay.

    Also runs a negative frame (no target) and warns (does not fail) if it false-positives —
    grounding is not perfect, but the positive case must produce a plausible box.
    """
    proc_w = cfg["perception"]["processing_width"]
    proc_h = cfg["perception"]["processing_height"]
    pos = os.path.join(REPO, "test_assets", "target_scene.png")
    neg = os.path.join(REPO, "test_assets", "no_target_scene.png")
    assert os.path.exists(pos), f"self-test asset missing: {pos}"

    pipe = Pipeline(cfg)
    pipe.min_interval = 0.0

    bgr = cv2.resize(cv2.imread(pos, cv2.IMREAD_COLOR), (proc_w, proc_h))
    meta = {"frame_id": 0, "mono_ts": time.monotonic(), "sim_time": 0.0, "controls": {}}
    payload, panel = pipe.step(bgr, meta, None, show=True)
    out = os.path.join(REPO, "test_assets", "object_selftest.png")
    cv2.imwrite(out, panel)
    print(f"[object][self-test] POSITIVE: found={payload['found']} bbox={payload['bbox']} "
          f"center={payload['center']}")
    print(f"[object][self-test] raw: {payload['raw']!r}")
    print(f"[object][self-test] overlay -> {out}")

    if os.path.exists(neg):
        bgr_n = cv2.resize(cv2.imread(neg, cv2.IMREAD_COLOR), (proc_w, proc_h))
        meta_n = {"frame_id": 1, "mono_ts": time.monotonic(), "sim_time": 0.0, "controls": {}}
        pn, _ = pipe.step(bgr_n, meta_n, None, show=False)
        print(f"[object][self-test] NEGATIVE (no target in frame): found={pn['found']} "
              f"{'(false positive — note for tuning)' if pn['found'] else '(correctly empty)'}")

    assert payload["found"], (
        "self-test FAILED: target not found in the positive frame — check the reference crop "
        "and prompt before trusting live runs.")
    print("[object][self-test] PASS")


def main():
    ap = argparse.ArgumentParser(description="Cartographer object_worker (P4): Qwen2.5-VL target detection")
    ap.add_argument("--config", default=None)
    ap.add_argument("--no-display", action="store_true", help="headless: skip the OpenCV window")
    ap.add_argument("--self-test", action="store_true",
                    help="detect once on a known target frame, save an overlay, exit (no bus/sim)")
    ap.add_argument("--video", default=None,
                    help="OFFLINE: scan this mp4 for the target, save overlays where found")
    ap.add_argument("--stride", type=int, default=15, help="offline: process every Nth source frame")
    ap.add_argument("--max-frames", type=int, default=0, help="offline: cap processed frames (0=all)")
    ap.add_argument("--out", default=None, help="offline: output dir (default: OUTPUT/)")
    ap.add_argument("--publish", action="store_true",
                    help="offline: also publish TOPIC_DETECTION on the state bus")
    args = ap.parse_args()

    cfg = load_config(args.config)
    if args.self_test:
        run_self_test(cfg)
    elif args.video:
        run_offline_video(cfg, args.video, show=not args.no_display, stride=args.stride,
                          max_frames=args.max_frames, out_dir=args.out, publish=args.publish)
    else:
        run_live(cfg, show=not args.no_display)


if __name__ == "__main__":
    main()
