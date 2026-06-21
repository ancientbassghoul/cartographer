"""Stream-A smoke test: verify Depth Anything V2 + Qwen2.5-VL-3B (4-bit) load and run on the GPU.

Per cartographer/CLAUDE.md NO-SILENT-FALLBACKS: this script fails fast. It does not wrap model
loads in try/except to downgrade; any failure raises and is visible. Run:

    venv\\Scripts\\python.exe smoke_test_models.py
"""
import sys
import time
from pathlib import Path

import torch
from PIL import Image

HERE = Path(__file__).resolve().parent
FRAME = HERE / "test_assets" / "sample_frame.png"
OUT = HERE / "test_assets"

DEPTH_MODEL = "depth-anything/Depth-Anything-V2-Base-hf"
QWEN_MODEL = "Qwen/Qwen2.5-VL-3B-Instruct"


def banner(msg: str) -> None:
    print(f"\n{'=' * 70}\n{msg}\n{'=' * 70}", flush=True)


def check_cuda() -> None:
    banner("CUDA")
    assert torch.cuda.is_available(), "CUDA not available — fix the env before proceeding."
    print(f"torch {torch.__version__} | device {torch.cuda.get_device_name(0)}", flush=True)
    print(f"VRAM total {torch.cuda.get_device_properties(0).total_memory / 1e9:.1f} GB", flush=True)


def test_depth(image: Image.Image) -> None:
    banner(f"DEPTH ANYTHING V2  ({DEPTH_MODEL})")
    from transformers import pipeline

    t0 = time.time()
    pipe = pipeline(task="depth-estimation", model=DEPTH_MODEL, device=0)
    print(f"loaded in {time.time() - t0:.1f}s", flush=True)

    t0 = time.time()
    result = pipe(image)
    depth = result["depth"]  # PIL Image, single channel
    import numpy as np

    arr = np.asarray(depth, dtype="float32")
    print(f"inference {time.time() - t0:.2f}s | depth shape {arr.shape} "
          f"| min {arr.min():.1f} max {arr.max():.1f} mean {arr.mean():.1f}", flush=True)
    depth.save(OUT / "smoke_depth.png")
    print(f"saved colorless depth -> {OUT / 'smoke_depth.png'}", flush=True)
    print(f"VRAM allocated {torch.cuda.memory_allocated() / 1e9:.2f} GB", flush=True)
    del pipe
    torch.cuda.empty_cache()


def test_qwen(image: Image.Image) -> None:
    banner(f"QWEN2.5-VL-3B 4-bit  ({QWEN_MODEL})")
    from transformers import (
        Qwen2_5_VLForConditionalGeneration,
        AutoProcessor,
        BitsAndBytesConfig,
    )
    from qwen_vl_utils import process_vision_info

    bnb = BitsAndBytesConfig(
        load_in_4bit=True,
        bnb_4bit_quant_type="nf4",
        bnb_4bit_compute_dtype=torch.float16,
    )
    t0 = time.time()
    model = Qwen2_5_VLForConditionalGeneration.from_pretrained(
        QWEN_MODEL, quantization_config=bnb, device_map="auto"
    )
    processor = AutoProcessor.from_pretrained(QWEN_MODEL)
    print(f"loaded in {time.time() - t0:.1f}s | VRAM {torch.cuda.memory_allocated() / 1e9:.2f} GB",
          flush=True)

    messages = [{
        "role": "user",
        "content": [
            {"type": "image", "image": image},
            {"type": "text", "text": "Describe this scene in one sentence, then list any distinct "
                                     "objects you could designate as a search target."},
        ],
    }]
    text = processor.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)
    image_inputs, video_inputs = process_vision_info(messages)
    inputs = processor(text=[text], images=image_inputs, videos=video_inputs,
                       padding=True, return_tensors="pt").to(model.device)

    t0 = time.time()
    gen = model.generate(**inputs, max_new_tokens=128)
    trimmed = [out[len(inp):] for inp, out in zip(inputs.input_ids, gen)]
    answer = processor.batch_decode(trimmed, skip_special_tokens=True)[0]
    print(f"inference {time.time() - t0:.2f}s", flush=True)
    print(f"\nQWEN SAYS:\n{answer}", flush=True)


def main() -> int:
    assert FRAME.exists(), f"sample frame missing: {FRAME}"
    image = Image.open(FRAME).convert("RGB")
    print(f"frame: {FRAME.name} {image.size}", flush=True)
    check_cuda()
    test_depth(image)
    test_qwen(image)
    banner("STREAM-A SMOKE TEST: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
