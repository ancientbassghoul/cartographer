"""target_classifier.py — one-time Qwen-VL classification of a designated target crop.

At designation (`make_target.py`) the cascade needs two things the user shouldn't have to type:
  * a short **GroundingDINO phrase** (e.g. "a rifle"), and
  * the **AssetClass** — `2D_PLANAR` (flat printed/painted surface: poster, sign, screen) vs
    `3D_GEOMETRY` (a solid 3D object) — which selects the DINOv2 threshold + geometric gate.

This runs Qwen2.5-VL-3B ONCE over the crop and proposes both, for the user to confirm or override.
It is **designation-only**: the live flight path (`object_worker` + `LiveCascade`) carries NO VLM —
Qwen is loaded here and freed, never during a flight. (A clean single-crop yes/no question is a
robust use of Qwen, unlike the per-frame grounding that failed in earlier milestones.)

GPU-only, NO SILENT FALLBACKS: CUDA + the Qwen load are asserted; failures raise.
"""

import time

import numpy as np
import torch

from cascade_detector import AssetClass

# Two-line answer keeps parsing unambiguous. Line 1 = detector phrase; line 2 = PLANAR/SOLID.
# Critical: judge the PHYSICAL MEDIUM, not what the picture depicts — Qwen otherwise calls a portrait
# poster "SOLID" because it shows a person. The examples + the explicit "even if it shows a person"
# clause fix that.
CLASSIFY_PROMPT = (
    "Categorize the PHYSICAL FORM of this target object for a 3D mapping system. Judge the MEDIUM, "
    "NOT what the picture depicts. Answer in EXACTLY two lines, nothing else.\n"
    "Line 1: a short noun phrase (2-5 words) naming the target, suitable as an object-detector prompt "
    "(e.g. 'a rifle', 'a printed portrait poster').\n"
    "Line 2: one word, PLANAR or SOLID.\n"
    "  PLANAR = a flat, thin 2D surface mounted on a wall: poster, sign, painting, photo, screen, "
    "banner, sticker. Answer PLANAR even if it SHOWS a person or object — the medium is a flat sheet.\n"
    "  SOLID = a real three-dimensional physical object with depth you could walk around: a rifle, a "
    "chair, a tool, a box."
)


class TargetClassifier:
    def __init__(self, hf_id="Qwen/Qwen2.5-VL-3B-Instruct", quantization="4bit", device="cuda"):
        assert torch.cuda.is_available(), (
            "CUDA not available — target classification requires the GPU (NO SILENT FALLBACKS).")
        from transformers import (
            Qwen2_5_VLForConditionalGeneration, AutoProcessor, BitsAndBytesConfig)

        quant_cfg = None
        if quantization == "4bit":
            quant_cfg = BitsAndBytesConfig(
                load_in_4bit=True, bnb_4bit_quant_type="nf4", bnb_4bit_compute_dtype=torch.float16)
        elif quantization not in (None, "none", "fp16"):
            raise ValueError(f"unsupported qwen quantization '{quantization}' "
                             "(NO SILENT FALLBACKS — fix config).")
        t0 = time.time()
        self.model = Qwen2_5_VLForConditionalGeneration.from_pretrained(
            hf_id, quantization_config=quant_cfg, device_map="auto",
            torch_dtype=torch.float16).eval()
        self.processor = AutoProcessor.from_pretrained(hf_id)
        torch.cuda.synchronize()
        print(f"[classify] Qwen2.5-VL '{hf_id}' ({quantization}) loaded in {time.time()-t0:.1f}s "
              f"| VRAM {torch.cuda.memory_allocated()/1e9:.2f} GB", flush=True)

    def classify(self, crop_rgb: np.ndarray) -> dict:
        """Return {label, asset_class (AssetClass value str), raw} for one HxWx3 RGB crop."""
        from PIL import Image
        from qwen_vl_utils import process_vision_info

        messages = [{"role": "user", "content": [
            {"type": "image", "image": Image.fromarray(crop_rgb)},
            {"type": "text", "text": CLASSIFY_PROMPT}]}]
        text = self.processor.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)
        image_inputs, video_inputs = process_vision_info(messages)
        inputs = self.processor(text=[text], images=image_inputs, videos=video_inputs,
                                padding=True, return_tensors="pt").to(self.model.device)
        with torch.inference_mode():
            gen = self.model.generate(**inputs, max_new_tokens=48, do_sample=False)
        trimmed = [out[len(inp):] for inp, out in zip(inputs.input_ids, gen)]
        raw = self.processor.batch_decode(
            trimmed, skip_special_tokens=True, clean_up_tokenization_spaces=False)[0].strip()

        lines = [ln.strip() for ln in raw.splitlines() if ln.strip()]
        label = lines[0].strip('".\'').strip() if lines else ""
        verdict = " ".join(lines[1:]).upper() if len(lines) > 1 else raw.upper()
        # PLANAR wins only if explicitly stated; default to 3D_GEOMETRY (the safer soft gate — a
        # mis-tagged planar would be SIFT-hard-gated and starve detections).
        asset = AssetClass.PLANAR_2D if "PLANAR" in verdict and "SOLID" not in verdict.split() \
            else AssetClass.GEOMETRY_3D
        return {"label": label, "asset_class": asset.value, "raw": raw}

    def close(self):
        del self.model, self.processor
        torch.cuda.empty_cache()


if __name__ == "__main__":
    # Quick sanity: classify the two benchmark reference crops.
    import argparse
    from pathlib import Path
    import cv2

    ap = argparse.ArgumentParser(description="Sanity-check Qwen target classification on crops.")
    ap.add_argument("crops", nargs="*",
                    default=["test_assets/Rifle_ref.png", "test_assets/Nasrallah_ref.png"])
    args = ap.parse_args()
    clf = TargetClassifier()
    for c in args.crops:
        p = Path(c)
        bgr = cv2.imread(str(p))
        if bgr is None:
            print(f"  [skip] could not read {p}")
            continue
        res = clf.classify(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        print(f"  {p.name}: label={res['label']!r} class={res['asset_class']} | raw={res['raw']!r}")
