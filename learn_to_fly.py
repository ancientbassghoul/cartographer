"""learn_to_fly.py — OFFLINE flight characterizer (CPU-only; OpenCV + numpy, NO torch/GPU).

Purpose: build LEARNING MATERIAL for an optical-flow-based ceiling/wall detector. The ceiling
detector twice failed leaning on the ~1 Hz, irregular SLAM pose; the right "am I still moving?"
signal is in the camera IMAGE at 30 fps. This tool takes a manually-flown, recorded flight (the
`flight_<ts>.mp4` written by io_bridge) plus its frame-synced keystroke log (`<ts>_keys.csv` from
`io_bridge --log-keys`, REQUIRED — the analyzer correlates flow with the commands that produced it)
and characterizes, offline:

  * what the dense optical flow does each frame (robust scalars: dy_med / dx_med / mag_mean /
    expansion + a cheap frame-diff energy);
  * which command was held at each frame (from the key log, replayed into a per-frame timeline);
  * how those line up — candidate states (ceiling_contact, wall_contact, free_ascent/forward,
    hover) and maneuvers (arm pattern, back-off runs) extracted as labeled, self-calibrating
    heuristics.

Outputs: a per-frame timeline CSV and a characterization JSON under OUTPUT/learn/. These are the
human/AI-readable artifact we read TOGETHER to design the real, SELF-CALIBRATING live detector.

HARD-RULE note (CLAUDE.md "CRITICAL AUTONOMY STANDARD"): this is a VALIDATION/learning step only.
Nothing here is fed back as a constant into the live autonomy. The candidate-state heuristics are
deliberately RELATIVE/self-calibrating (e.g. "flow collapsed below ~15% of the ascent flow just
measured in THIS climb") so what we learn is the platform's flow SIGNATURE, never this room's
answer (which altitude / which frame the ceiling is at). The frame index == the video's frame ==
io_bridge's `rec_frame`, so the key log lines up 1:1 with the footage.
"""

import argparse
import csv
import json
import os
from collections import OrderedDict

import cv2
import numpy as np

REPO = os.path.dirname(os.path.abspath(__file__))

# Raw key -> semantic command (mirrors io_bridge._on_key_event mapping exactly).
KEY_TO_CMD = {
    "1": "arm", "2": "btnA", "b": "land", "c": "reset_cam",
    "w": "forward", "s": "back", "e": "up", "f": "down",
    "a": "strafe_left", "d": "strafe_right",
    "up": "pitch_fwd", "down": "pitch_back", "left": "yaw_left", "right": "yaw_right",
    "k": "joy_click", "p": "thumb", "m": "autonomy_toggle", "g": "detect", "space": "capture",
    "r": "record", "q": "quit",
}
# The motion commands that actually move the airframe (used to segment the flight by intent).
MOTION_CMDS = {"forward", "back", "up", "down", "strafe_left", "strafe_right",
               "pitch_fwd", "pitch_back", "yaw_left", "yaw_right"}


# ==============================================================================
# Optical flow
# ==============================================================================
# Farneback dense-flow primitive. Shared by this offline analyzer AND the live causal detector
# (flow_contact_detector.py) so both reduce frames to the SAME scalars with the SAME params.
FARNEBACK = dict(pyr_scale=0.5, levels=3, winsize=15, iterations=3, poly_n=5, poly_sigma=1.2, flags=0)


def make_radial_field(fw, fh):
    """Outward-pointing unit vectors from the image center, for the expansion / looming proxy."""
    ys, xs = np.mgrid[0:fh, 0:fw].astype(np.float32)
    cx, cy = (fw - 1) / 2.0, (fh - 1) / 2.0
    rx, ry = xs - cx, ys - cy
    rnorm = np.sqrt(rx * rx + ry * ry) + 1e-6
    return rx / rnorm, ry / rnorm


def farneback_scalars(prev_gray, gray, rux, ruy):
    """Dense Farneback flow between two equal-size grayscale frames, reduced to robust scalars:
    dy_med (vertical — the ascent signal), dx_med (horizontal), mag_mean (overall motion),
    expansion (mean outward radial flow — looming / forward progress). `rux,ruy` from make_radial_field
    must match the frame size. NO downscaling here — callers normalize resolution before calling."""
    flow = cv2.calcOpticalFlowFarneback(prev_gray, gray, None, **FARNEBACK)
    fx, fy = flow[..., 0], flow[..., 1]
    return {
        "dy_med": float(np.median(fy)),
        "dx_med": float(np.median(fx)),
        "mag_mean": float(np.mean(np.sqrt(fx * fx + fy * fy))),
        "expansion": float(np.mean(fx * rux + fy * ruy)),
    }


def compute_flow_timeline(video_path, flow_long_side=320, max_frames=None, progress_every=200):
    """Per-frame dense Farneback flow reduced to robust scalars. Frame index == video frame index
    (== io_bridge rec_frame). Returns (rows, meta). Each row keys: frame, dy_med, dx_med, mag_mean,
    expansion, framediff. Flow is computed on a grayscale frame downscaled so its long side is
    `flow_long_side` (disclosed; for SPEED only — this is an offline analyzer, not a model input)."""
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open video: {video_path}")
    src_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    src_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    n_frames_total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    # flow-resolution scale (long side -> flow_long_side, never upscale)
    scale = min(1.0, flow_long_side / float(max(src_w, src_h)))
    fw, fh = max(1, int(round(src_w * scale))), max(1, int(round(src_h * scale)))

    # radial unit field (from image center) for the expansion / looming proxy, built once.
    rux, ruy = make_radial_field(fw, fh)

    rows = []
    prev_gray = None
    prev_full_gray = None
    idx = -1
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        idx += 1
        if max_frames is not None and idx >= max_frames:
            break
        gray_full = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.resize(gray_full, (fw, fh), interpolation=cv2.INTER_AREA) if scale < 1.0 else gray_full
        if prev_gray is None:
            # first frame: no flow yet (define as zero-motion baseline)
            rows.append({"frame": idx, "dy_med": 0.0, "dx_med": 0.0, "mag_mean": 0.0,
                         "expansion": 0.0, "framediff": 0.0})
        else:
            sc = farneback_scalars(prev_gray, gray, rux, ruy)
            sc["frame"] = idx
            sc["framediff"] = float(np.mean(np.abs(gray.astype(np.int16) - prev_full_gray.astype(np.int16))))
            rows.append(sc)
        prev_gray = gray
        prev_full_gray = gray
        if progress_every and idx % progress_every == 0:
            print(f"[learn] flow frame {idx}/{n_frames_total}", flush=True)
    cap.release()

    meta = {"video": os.path.abspath(video_path), "fps": float(fps),
            "resolution": [src_w, src_h], "n_frames": len(rows),
            "flow_params": {"method": "farneback", "flow_resolution": [fw, fh],
                            "flow_long_side": flow_long_side, "scale": round(scale, 4),
                            "note": "downscale is for analyzer speed only; not a model input"}}
    return rows, meta


# ==============================================================================
# Keystroke timeline
# ==============================================================================
def load_key_edges(keys_csv):
    """Read <ts>_keys.csv (rec_frame, mono_ts, key, action). Keep only rows with an integer
    rec_frame (i.e. edges that occurred while recording, so they map onto the video). Returns a
    list sorted by (rec_frame, mono_ts)."""
    edges = []
    with open(keys_csv, "r", newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            rf = r.get("rec_frame", "")
            if rf is None or str(rf).strip() == "" or str(rf).strip().lower() == "none":
                continue   # edge happened before recording started -> not on the video timeline
            try:
                frame = int(float(rf))
            except ValueError:
                continue
            edges.append({"frame": frame, "mono_ts": float(r.get("mono_ts") or 0.0),
                          "key": r.get("key", ""), "action": r.get("action", "")})
    edges.sort(key=lambda e: (e["frame"], e["mono_ts"]))
    return edges


def build_active_timeline(edges, n_frames):
    """Replay key edges into a per-frame set of SEMANTIC commands held at that frame. A 'down' at
    frame f makes the command active from f onward; 'up' at f deactivates from f onward."""
    # bucket edges by frame
    by_frame = {}
    for e in edges:
        by_frame.setdefault(e["frame"], []).append(e)
    active = []
    held = set()             # semantic commands currently held
    for f in range(n_frames):
        for e in by_frame.get(f, []):
            cmd = KEY_TO_CMD.get(e["key"], e["key"])
            if e["action"] == "down":
                held.add(cmd)
            elif e["action"] == "up":
                held.discard(cmd)
        active.append(frozenset(held))
    return active


def extract_maneuvers(edges):
    """Group each key's press->release into runs, in chronological order. Each run carries its REAL
    duration `duration_s` from mono_ts (wall-clock seconds) — this is what flight recipes must use.
    Do NOT derive durations from frame counts: the recording is ~58 fps (not 30), so frames/30 is ~1.92x
    too long. `start`/`end`/`n_frames` (rec_frame) are kept only for scrubbing the video.
    These are PLATFORM control dynamics (legitimate to learn per the refined HARD RULE)."""
    open_press = {}          # key -> (down rec_frame, down mono_ts)
    runs = []
    for e in edges:
        if e["action"] == "down":
            open_press[e["key"]] = (e["frame"], e["mono_ts"])
        elif e["action"] == "up" and e["key"] in open_press:
            f0, t0 = open_press.pop(e["key"])
            runs.append({"key": e["key"], "command": KEY_TO_CMD.get(e["key"], e["key"]),
                         "start": f0, "end": e["frame"], "n_frames": e["frame"] - f0,
                         "duration_s": round(e["mono_ts"] - t0, 3)})
    runs.sort(key=lambda r: r["start"])
    return runs


# ==============================================================================
# Segmentation + candidate states
# ==============================================================================
def _flow_summary(flow_rows, s, e):
    """Robust summary of the flow scalars over frames [s, e)."""
    sub = flow_rows[s:e]
    if not sub:
        return {}
    def col(k):
        return np.array([r[k] for r in sub], dtype=np.float64)
    out = {}
    for k in ("dy_med", "dx_med", "mag_mean", "expansion", "framediff"):
        v = col(k)
        out[k] = {"median": float(np.median(v)), "mean": float(np.mean(v)),
                  "p90_abs": float(np.percentile(np.abs(v), 90))}
    return out


def segment_by_command(flow_rows, active, n_frames):
    """Split the flight into maximal runs where the held MOTION-command set is constant; summarize
    flow per segment. (Non-motion commands like arm/capture don't split segments.)"""
    def motion_key(f):
        return frozenset(c for c in active[f] if c in MOTION_CMDS) if f < len(active) else frozenset()
    segments = []
    s = 0
    cur = motion_key(0)
    for f in range(1, n_frames):
        k = motion_key(f)
        if k != cur:
            segments.append({"keys": sorted(cur), "start": s, "end": f, "n_frames": f - s,
                             "flow": _flow_summary(flow_rows, s, f)})
            s, cur = f, k
    segments.append({"keys": sorted(cur), "start": s, "end": n_frames, "n_frames": n_frames - s,
                     "flow": _flow_summary(flow_rows, s, n_frames)})
    return segments


def detect_ceiling_candidates(flow_rows, active, n_frames,
                              calib_win=45, stall_frac=0.15, min_hold=20):
    """SELF-CALIBRATING ceiling heuristic (labeled; learning only). For each frame, compare the
    current |dy_med| against the climb's OWN recently-measured ascent level (robust high percentile
    of |dy_med| over the preceding `calib_win` frames). A 'ceiling contact' candidate = a sustained
    (>= min_hold frames) collapse of |dy_med| below `stall_frac` of that live ascent reference while
    UP is commanded.

    NOTE: this mirrors the eventual LIVE detector's scale-free logic on purpose, but it is RELATIVE
    (no absolute altitude/frame baked in) — it surfaces WHERE the signature occurs so we can read it.
    """
    dy = np.array([abs(r["dy_med"]) for r in flow_rows], dtype=np.float64)
    candidates = []
    run_start = None
    run_ref = 0.0
    for f in range(n_frames):
        lo = max(0, f - calib_win)
        ref = float(np.percentile(dy[lo:f], 90)) if f - lo >= 5 else 0.0
        commanded_up = "up" in active[f]
        # "was ascending recently" gate so we don't flag a hover as a ceiling
        ascending_recently = ref > max(0.2, np.median(dy[lo:f]) if f - lo >= 5 else 0.0)
        collapsed = ref > 0 and dy[f] < stall_frac * ref
        in_stall = commanded_up and ascending_recently and collapsed
        if in_stall and run_start is None:
            run_start, run_ref = f, ref
        elif not in_stall and run_start is not None:
            if f - run_start >= min_hold:
                candidates.append({"frame_range": [run_start, f], "n_frames": f - run_start,
                                   "evidence": {"ascent_ref_dy": round(run_ref, 3),
                                                "stall_frac": stall_frac,
                                                "mean_dy_in_stall": round(float(np.mean(dy[run_start:f])), 3)}})
            run_start = None
    if run_start is not None and n_frames - run_start >= min_hold:
        candidates.append({"frame_range": [run_start, n_frames], "n_frames": n_frames - run_start,
                           "evidence": {"ascent_ref_dy": round(run_ref, 3), "stall_frac": stall_frac,
                                        "mean_dy_in_stall": round(float(np.mean(dy[run_start:n_frames])), 3)}})
    return candidates


def detect_wall_candidates(flow_rows, active, n_frames, min_hold=15):
    """Wall-contact heuristic (labeled; learning only): while FORWARD is commanded, the looming
    EXPANSION collapses from its recent free-forward level to ~0 — forward progress has stopped.
    This single signal unifies BOTH wall flavors: a textureless wall freezes the image (mag→0) and a
    textured wall shows a slow vertical climb; either way the radial looming dies. (Earlier versions
    also required mag>0.2, which wrongly excluded the freeze case — removed.)"""
    exp = np.array([r["expansion"] for r in flow_rows], dtype=np.float64)
    mag = np.array([r["mag_mean"] for r in flow_rows], dtype=np.float64)
    candidates = []
    run_start = None
    for f in range(n_frames):
        lo = max(0, f - 45)
        exp_ref = float(np.percentile(exp[lo:f], 90)) if f - lo >= 5 else 0.0
        commanded_fwd = "forward" in active[f]
        stalled = exp_ref > 0.05 and exp[f] < 0.15 * exp_ref     # looming collapsed vs its live ref
        in_wall = commanded_fwd and stalled
        if in_wall and run_start is None:
            run_start = f
        elif not in_wall and run_start is not None:
            if f - run_start >= min_hold:
                candidates.append({"frame_range": [run_start, f], "n_frames": f - run_start,
                                   "evidence": {"mean_expansion": round(float(np.mean(exp[run_start:f])), 3),
                                                "mean_mag": round(float(np.mean(mag[run_start:f])), 3)}})
            run_start = None
    if run_start is not None and n_frames - run_start >= min_hold:
        candidates.append({"frame_range": [run_start, n_frames], "n_frames": n_frames - run_start,
                           "evidence": {"mean_expansion": round(float(np.mean(exp[run_start:n_frames])), 3),
                                        "mean_mag": round(float(np.mean(mag[run_start:n_frames])), 3)}})
    return candidates


# ==============================================================================
# Main
# ==============================================================================
def main():
    ap = argparse.ArgumentParser(description="Offline flight characterizer (optical flow + keys)")
    ap.add_argument("--video", required=True, help="recorded flight_<ts>.mp4")
    ap.add_argument("--keys", required=True,
                    help="<ts>_keys.csv from io_bridge --log-keys (required — the analyzer correlates "
                         "flow with the commands that produced it)")
    ap.add_argument("--out-dir", default=os.path.join(REPO, "OUTPUT", "learn"))
    ap.add_argument("--flow-long-side", type=int, default=320,
                    help="downscale long side for flow compute speed (analyzer only)")
    ap.add_argument("--max-frames", type=int, default=None, help="cap frames (debug)")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    stem = os.path.splitext(os.path.basename(args.video))[0]

    print(f"[learn] analyzing {args.video}")
    flow_rows, meta = compute_flow_timeline(args.video, flow_long_side=args.flow_long_side,
                                            max_frames=args.max_frames)
    n_frames = len(flow_rows)
    print(f"[learn] {n_frames} frames @ {meta['fps']:.1f} fps, {meta['resolution']}")

    # keystrokes (required — the analyzer's job is to correlate flow with the commands that drove it)
    if not os.path.exists(args.keys):
        raise RuntimeError(f"--keys file not found: {args.keys}")
    edges = load_key_edges(args.keys)
    active = build_active_timeline(edges, n_frames)
    maneuvers = extract_maneuvers(edges)
    print(f"[learn] {len(edges)} key edges on the recording timeline, {len(maneuvers)} press-runs")

    # per-frame timeline CSV (flow + active commands)
    per_frame_csv = os.path.join(args.out_dir, f"{stem}_flow.csv")
    with open(per_frame_csv, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["frame", "dy_med", "dx_med", "mag_mean", "expansion", "framediff", "active_cmds"])
        for i, r in enumerate(flow_rows):
            cmds = "|".join(sorted(c for c in active[i] if c in MOTION_CMDS))
            w.writerow([r["frame"], f"{r['dy_med']:.4f}", f"{r['dx_med']:.4f}",
                        f"{r['mag_mean']:.4f}", f"{r['expansion']:.4f}", f"{r['framediff']:.3f}", cmds])
    print(f"[learn] per-frame timeline -> {per_frame_csv}")

    # segmentation + candidate states + maneuvers
    segments = segment_by_command(flow_rows, active, n_frames)
    ceiling = detect_ceiling_candidates(flow_rows, active, n_frames)
    wall = detect_wall_candidates(flow_rows, active, n_frames)

    out = OrderedDict()
    out["video"] = meta["video"]
    out["fps"] = meta["fps"]
    out["resolution"] = meta["resolution"]
    out["n_frames"] = meta["n_frames"]
    out["flow_params"] = meta["flow_params"]
    out["keys_csv"] = os.path.abspath(args.keys)
    out["per_frame_csv"] = os.path.abspath(per_frame_csv)
    out["heuristics_note"] = ("RELATIVE/self-calibrating, labeled — learning material only; no value "
                              "here is a baked autonomy constant (HARD RULE).")
    out["command_segments"] = segments
    out["candidate_states"] = {"ceiling_contact": ceiling, "wall_contact": wall}
    out["maneuvers"] = maneuvers

    json_path = os.path.join(args.out_dir, f"{stem}.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=2)
    print(f"[learn] characterization JSON -> {json_path}")

    # quick console read
    print(f"[learn] ceiling_contact candidates: {len(ceiling)}")
    for c in ceiling:
        print(f"        frames {c['frame_range']} ({c['n_frames']}f) "
              f"ascent_ref_dy={c['evidence']['ascent_ref_dy']} "
              f"mean_dy_in_stall={c['evidence']['mean_dy_in_stall']}")
    print(f"[learn] wall_contact candidates: {len(wall)}")
    for c in wall:
        print(f"        frames {c['frame_range']} ({c['n_frames']}f) "
              f"mean_expansion={c['evidence']['mean_expansion']}")


if __name__ == "__main__":
    main()
