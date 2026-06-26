"""object_worker.py — Process P4: target detection via the verified CASCADE.

The gradable-core detector. Subscribes to the io_bridge **hi-res** frame bus (:5605, CONFLATE
newest-wins) and, in its **own process / CUDA context**, runs the verified cascade detector on each
frame to ground the designated target:

  Stage 1  propose : GroundingDINO (text phrase) + OWLv2 image-guided (reference crop), pooled.
  Stage 2  verify  : DINOv2 ViT-S/14 crop-embedding cosine vs the reference (letterboxed).
  Stage 3  gate    : per the target's AssetClass — 2D_PLANAR = SIFT homography HARD gate;
                     3D_GEOMETRY = LightGlue SOFT bonus (never vetoes). DINOv2-cosine-primary rank.

The full pipeline lives in `cascade_detector.LiveCascade` (all models resident). This worker is the
live wrapper: throttle to `perception.object_cadence_hz` (~0.5 Hz = every 2 s), scale the box to the
512x288 transport space the 3D lift expects, and publish TOPIC_DETECTION.

The designated target (reference crop + GroundingDINO phrase + asset class) is produced by
`make_target.py` and stored in `target.yaml`; run that first.

Output: publishes TOPIC_DETECTION on its own state bus (`object_state_port`, default :5604):
  {object_mode, target_label, asset_class, frame_id, sim_time, found, bbox[x1,y1,x2,y2],
   center[cx,cy], infer_ms, raw}
bbox/center are in 512x288 frame pixels (or null when not seen). These feed the 3D lift
(back-project the center through the SLAM pose + pointmap) in perception_worker — unchanged.

NO SILENT FALLBACKS (per CLAUDE.md): CUDA + every model load are asserted up front; any failure
raises. There is no CPU path and no auto-swap. `object_mode="CASCADE"` is the visible state flag in
every payload + log; the active `asset_class` is logged, shown in the overlay, and carried in the
payload.
"""

import argparse
import os
import time

import cv2
import numpy as np
import torch
import yaml

import frame_bus
from diag_log import DiagLog, NullLog

REPO = os.path.dirname(os.path.abspath(__file__))
TARGET_FILE = os.path.join(REPO, "target.yaml")

# Visible NO-FALLBACK state flag. The cascade is the only detector; a typo in config must crash.
OBJECT_MODE = "CASCADE"
VALID_OBJECT_MODES = ("CASCADE",)


def set_object_mode(cfg) -> str:
    """Read the visible detector flag from config (runtime.object_mode) and pin the module global.
    Fail-fast on an unknown mode (NO SILENT FALLBACKS)."""
    global OBJECT_MODE
    mode = str(cfg.get("runtime", {}).get("object_mode", "CASCADE")).strip().upper()
    if mode not in VALID_OBJECT_MODES:
        raise ValueError(f"unknown runtime.object_mode {mode!r} (expected one of "
                         f"{VALID_OBJECT_MODES}; NO SILENT FALLBACKS — fix config).")
    OBJECT_MODE = mode
    return mode


def load_config(path=None):
    path = path or os.path.join(REPO, "config.yaml")
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def load_target(cfg):
    """Load the designated target (reference crop RGB, GroundingDINO text, asset class) from
    target.yaml. Fail-fast with guidance if it (or the crop) is missing — designation comes from
    make_target.py (NO SILENT FALLBACKS)."""
    if not os.path.exists(TARGET_FILE):
        raise SystemExit(f"no target designated: {TARGET_FILE} not found — run "
                         f"`python make_target.py` to pick + classify your target first.")
    with open(TARGET_FILE, "r", encoding="utf-8") as f:
        spec = yaml.safe_load(f) or {}
    for k in ("reference_crop", "text", "asset_class"):
        if not spec.get(k):
            raise SystemExit(f"target.yaml missing '{k}' — re-run make_target.py (NO SILENT FALLBACKS).")
    rel = spec["reference_crop"]
    path = rel if os.path.isabs(rel) else os.path.join(REPO, rel)
    bgr = cv2.imread(path, cv2.IMREAD_COLOR)
    if bgr is None:
        raise SystemExit(f"reference crop not found/readable: {path} (from target.yaml).")
    ref_rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    print(f"[object] target.yaml: crop {path} {ref_rgb.shape[1]}x{ref_rgb.shape[0]} | "
          f"text={spec['text']!r} | class={spec['asset_class']}", flush=True)
    return ref_rgb, str(spec["text"]).strip(), str(spec["asset_class"]).strip()


# ==============================================================================
# Cascade detector — thin live wrapper around cascade_detector.LiveCascade
# ==============================================================================
class CascadeDetector:
    """Live cascade detector. Loads all cascade models resident (via LiveCascade) and onboards the
    one designated target. `detect(ref, frame, label)` keeps the Pipeline's existing interface;
    `ref`/`label` are ignored (the target is held inside LiveCascade)."""

    def __init__(self, ref_rgb, text, asset_class, owlv2_id, device="cuda"):
        assert torch.cuda.is_available(), (
            "CUDA not available — object_worker requires the GPU. No CPU fallback (NO SILENT FALLBACKS).")
        from cascade_detector import LiveCascade
        t0 = time.time()
        self.core = LiveCascade(device=device, owlv2_id=owlv2_id)
        self.core.set_target(ref_rgb, text, asset_class)
        self.object_mode = OBJECT_MODE
        self.text = text
        self.asset_class = self.core.asset_class.value
        print(f"[object] cascade ready in {time.time()-t0:.1f}s | target '{text}' "
              f"[{self.asset_class}] | VRAM {torch.cuda.memory_allocated()/1e9:.2f} GB", flush=True)

    def detect(self, ref_rgb, frame_rgb, label=None) -> dict:
        r = self.core.detect(frame_rgb)
        return {"found": r["found"], "bbox": r["bbox"], "center": r["center"], "raw": r["raw"]}


# ==============================================================================
# Payload + render
# ==============================================================================
def build_payload(det, meta, infer_ms, cadence_hz, label, asset_class):
    return {
        "object_mode": OBJECT_MODE,
        "target_label": label,
        "asset_class": asset_class,
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


DET_WINDOW = "Cartographer — object detection (cascade)"


def render(frame_bgr, ref_rgb, det, infer_ms, label="", asset_class=""):
    """Compose [ reference crop | live frame + bbox ] with telemetry."""
    h, w = frame_bgr.shape[:2]
    panel = frame_bgr.copy()
    if det["found"] and det["bbox"]:
        x1, y1, x2, y2 = [int(round(v)) for v in det["bbox"]]
        cv2.rectangle(panel, (x1, y1), (x2, y2), (0, 0, 255), 2)
        cx, cy = [int(round(v)) for v in det["center"]]
        cv2.drawMarker(panel, (cx, cy), (0, 255, 255), cv2.MARKER_CROSS, 16, 2)
    tag = "TARGET" if det["found"] else "no target"
    status = f"{OBJECT_MODE} [{asset_class}:{label}]  {tag}  infer={infer_ms:.0f}ms"
    col = (0, 255, 0) if det["found"] else (0, 165, 255)
    cv2.putText(panel, status, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)

    ref_bgr = cv2.cvtColor(ref_rgb, cv2.COLOR_RGB2BGR)
    ref_h = h
    ref_w = max(1, int(ref_bgr.shape[1] * ref_h / ref_bgr.shape[0]))
    ref_panel = cv2.resize(ref_bgr, (ref_w, ref_h), interpolation=cv2.INTER_AREA)
    cv2.putText(ref_panel, "reference", (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
    return np.hstack([ref_panel, panel])


# ==============================================================================
# Pipeline
# ==============================================================================
class Pipeline:
    def __init__(self, cfg):
        mode = set_object_mode(cfg)  # pins the module OBJECT_MODE flag from config (CASCADE)
        self.cadence_hz = float(cfg["perception"]["object_cadence_hz"])
        self.min_interval = 1.0 / self.cadence_hz if self.cadence_hz > 0 else 0.0
        # The lift (perception_worker) works in the 512x288 transport space, so detections are
        # scaled back to it regardless of what (higher) resolution the detector grounded on.
        self.proc_w = int(cfg["perception"]["processing_width"])
        self.proc_h = int(cfg["perception"]["processing_height"])
        self.ref_rgb, self.label, self.asset_class = load_target(cfg)
        owlv2_id = cfg["models"]["owlv2"]["hf_id"]
        print(f"[object] === object_mode = {mode} | target '{self.label}' [{self.asset_class}] ===",
              flush=True)
        self.detector = CascadeDetector(self.ref_rgb, self.label, self.asset_class, owlv2_id=owlv2_id)
        self.last_infer_mono = 0.0
        self.n_det = 0
        self.n_found = 0
        self.diag = NullLog()          # detection cadence/timing CSV (off unless enable_diag)
        self._last_det_ts = None

    def enable_diag(self, ts=None, out_dir=None):
        self.diag = DiagLog("object", [
            "wall_ts", "frame_id", "dt_since_last", "infer_ms", "found",
            "center_x", "center_y", "bbox_area", "raw"], out_dir=out_dir, ts=ts)

    def close_diag(self):
        self.diag.close()

    def _to_transport(self, det, src_w, src_h):
        """Rescale a detection (in the detection frame's pixels) to the 512x288 transport space the
        lift expects. Identity when the frame is already transport-sized."""
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

        # dt_since_last = wall-clock gap between detection runs = the EFFECTIVE cadence (issue A).
        dt_since_last = (now - self._last_det_ts) if self._last_det_ts else 0.0
        self._last_det_ts = now
        b = det.get("bbox")
        self.diag.row(
            wall_ts=round(time.time(), 4), frame_id=meta.get("frame_id"),
            dt_since_last=round(dt_since_last, 3), infer_ms=round(infer_ms, 1),
            found=int(det["found"]),
            center_x=(det["center"][0] if det.get("center") else ""),
            center_y=(det["center"][1] if det.get("center") else ""),
            bbox_area=(round((b[2] - b[0]) * (b[3] - b[1]), 1)
                       if isinstance(b, (list, tuple)) and len(b) == 4 else ""),
            raw=(det.get("raw") or "")[:120])

        # Publish the detection in transport (512x288) pixels for the lift; render on the hi-res frame.
        det_tx = self._to_transport(det, w, h)
        payload = build_payload(det_tx, meta, infer_ms, self.cadence_hz, self.label, self.asset_class)
        if state_pub is not None:
            state_pub.publish(frame_bus.TOPIC_DETECTION, payload)

        c = (meta.get("controls") or {})
        print(f"[object] {OBJECT_MODE}[{self.asset_class}] frame {meta.get('frame_id')} | "
              f"{'TARGET '+str(det_tx['center']) if det['found'] else 'no target':<28} | "
              f"infer {infer_ms:6.0f} ms | found {self.n_found}/{self.n_det} | "
              f"src {w}x{h} | trigger {c.get('trigger')}", flush=True)

        panel = render(frame_bgr, self.ref_rgb, det, infer_ms, self.label, self.asset_class) if show else None
        return payload, panel


# ==============================================================================
# Live loop / offline video / self-test
# ==============================================================================
def run_live(cfg, show=True, log=False):
    # Prefer the hi-res object stream (full pixel fidelity stabilizes grounding); fall back to the
    # 512x288 perception stream only if no hi-res port is configured.
    frame_port = cfg["network"].get("frame_bus_hires_port") or cfg["network"]["frame_bus_port"]
    obj_port = cfg["network"]["object_state_port"]
    pipe = Pipeline(cfg)
    if log:
        pipe.enable_diag()
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
        pipe.close_diag()
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
            # Feed the cascade NATIVE pixels (it was tuned on 1280x720); the box is scaled to the
            # 512x288 transport by _to_transport using the frame's own size. (No downscale here.)
            meta = {"frame_id": yielded, "mono_ts": time.monotonic(),
                    "sim_time": round(src_idx / fps, 3), "controls": {}}
            yield bgr, meta
            yielded += 1
            if max_frames and yielded >= max_frames:
                break
        src_idx += 1
    cap.release()


def run_offline_video(cfg, video, show=False, stride=15, max_frames=0, out_dir=None, publish=False):
    """Offline verification: run the cascade over a recording at the configured cadence. Saves an
    overlay PNG for every frame where the target is found."""
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
    """Run the cascade once on a frame known to contain the designated target, save an overlay.

    Also runs a negative frame (no target) and warns (does not fail) if it false-positives. The
    target is whatever target.yaml designates; target_scene.png must contain it."""
    pos = os.path.join(REPO, "test_assets", "target_scene.png")
    neg = os.path.join(REPO, "test_assets", "no_target_scene.png")
    assert os.path.exists(pos), f"self-test asset missing: {pos}"

    pipe = Pipeline(cfg)
    pipe.min_interval = 0.0

    # Feed the scene at NATIVE resolution (the cascade was tuned on full-res frames; _to_transport
    # scales the resulting box down). No downscale here.
    bgr = cv2.imread(pos, cv2.IMREAD_COLOR)
    meta = {"frame_id": 0, "mono_ts": time.monotonic(), "sim_time": 0.0, "controls": {}}
    payload, panel = pipe.step(bgr, meta, None, show=True)
    out = os.path.join(REPO, "test_assets", "object_selftest.png")
    cv2.imwrite(out, panel)
    print(f"[object][self-test] POSITIVE: found={payload['found']} bbox={payload['bbox']} "
          f"center={payload['center']}")
    print(f"[object][self-test] raw: {payload['raw']!r}")
    print(f"[object][self-test] overlay -> {out}")

    if os.path.exists(neg):
        bgr_n = cv2.imread(neg, cv2.IMREAD_COLOR)
        meta_n = {"frame_id": 1, "mono_ts": time.monotonic(), "sim_time": 0.0, "controls": {}}
        pn, _ = pipe.step(bgr_n, meta_n, None, show=False)
        print(f"[object][self-test] NEGATIVE (no target in frame): found={pn['found']} "
              f"{'(false positive — note for tuning)' if pn['found'] else '(correctly empty)'}")

    assert payload["found"], (
        "self-test FAILED: target not found in the positive frame — check target.yaml (crop, text, "
        "asset class) before trusting live runs.")
    print("[object][self-test] PASS")


def main():
    ap = argparse.ArgumentParser(description="Cartographer object_worker (P4): cascade target detection")
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
    ap.add_argument("--log", action="store_true",
                    help="live: write detection cadence/timing CSV to OUTPUT/diag/ (issue A)")
    args = ap.parse_args()

    cfg = load_config(args.config)
    if args.self_test:
        run_self_test(cfg)
    elif args.video:
        run_offline_video(cfg, args.video, show=not args.no_display, stride=args.stride,
                          max_frames=args.max_frames, out_dir=args.out, publish=args.publish)
    else:
        run_live(cfg, show=not args.no_display, log=args.log)


if __name__ == "__main__":
    main()
