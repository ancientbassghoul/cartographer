"""annotate_targets.py — draw a ground-truth target box on each frame in a folder.

A standalone, GPU-free OpenCV utility for the detection benchmark. Point it at a folder of
full-res frames (e.g. test_assets/Nasrallah, test_assets/Rifle) and drag one box around the
target on each frame; it writes a `labels.json` next to the frames:

    { "frame_01.png": [x1, y1, x2, y2], ... }

Coordinates are integer pixels in the image's NATIVE full resolution, TOP-LEFT origin
(x -> right, y -> down) — cv2's own mouse convention, so no axis flipping.

Loop: a frame shows up -> left-drag a rectangle (drawing another REPLACES it) -> ENTER saves it
and advances. If the folder already has a labels.json, each frame opens with its saved box drawn.
labels.json is rewritten after every ENTER/clear, so quitting mid-session never loses work; frames
left without a box are simply omitted.

Keys:  ENTER = save box + next   |   c / Backspace = clear this frame's box
       p / Left = previous frame  |   q / Esc = quit (progress saved)

It does NOT import torch or any worker module. Example:
    venv\\Scripts\\python.exe annotate_targets.py test_assets\\Rifle
"""

import argparse
import json
from pathlib import Path

import cv2

IMG_EXTS = {".png", ".jpg", ".jpeg", ".bmp"}
MIN_BOX_PX = 4          # drags smaller than this (in original px) are treated as stray clicks
MAX_DISP_W = 1600       # fit the display window within this box; boxes stay in native coords
MAX_DISP_H = 900
WINDOW = "annotate_targets — drag box | ENTER next | c clear | p back | q quit"


def list_frames(folder: Path):
    return sorted(p for p in folder.iterdir()
                  if p.is_file() and p.suffix.lower() in IMG_EXTS)


def load_labels(labels_path: Path) -> dict:
    if labels_path.exists():
        with open(labels_path, "r", encoding="utf-8") as f:
            return json.load(f)
    return {}


def save_labels(labels_path: Path, labels: dict):
    with open(labels_path, "w", encoding="utf-8") as f:
        json.dump(labels, f, indent=2)


class BoxState:
    """Mutable state shared with the mouse callback for the current frame."""

    def __init__(self, scale: float, w: int, h: int):
        self.scale = scale          # display_px = original_px * scale
        self.w = w                  # original (native) image width
        self.h = h                  # original image height
        self.box = None             # committed box [x1,y1,x2,y2] in ORIGINAL pixels (or None)
        self.dragging = False
        self._start = None          # drag start in original px
        self._cur = None            # drag current point in original px

    def _to_orig(self, x, y):
        ox = int(round(x / self.scale))
        oy = int(round(y / self.scale))
        return (max(0, min(ox, self.w - 1)), max(0, min(oy, self.h - 1)))

    def live_box(self):
        """The box to draw right now: the in-progress drag if any, else the committed box."""
        if self.dragging and self._start and self._cur:
            return _normalize(self._start, self._cur)
        return self.box


def _normalize(p0, p1):
    x1, x2 = sorted((p0[0], p1[0]))
    y1, y2 = sorted((p0[1], p1[1]))
    return [x1, y1, x2, y2]


def on_mouse(event, x, y, flags, state: BoxState):
    if event == cv2.EVENT_LBUTTONDOWN:
        state.dragging = True
        state._start = state._to_orig(x, y)
        state._cur = state._start
    elif event == cv2.EVENT_MOUSEMOVE and state.dragging:
        state._cur = state._to_orig(x, y)
    elif event == cv2.EVENT_LBUTTONUP and state.dragging:
        state.dragging = False
        state._cur = state._to_orig(x, y)
        box = _normalize(state._start, state._cur)
        # Ignore an accidental click / tiny drag so it can't wipe a good box.
        if (box[2] - box[0]) >= MIN_BOX_PX and (box[3] - box[1]) >= MIN_BOX_PX:
            state.box = box


def render(frame, state: BoxState, name, idx, total):
    disp = cv2.resize(frame, (0, 0), fx=state.scale, fy=state.scale) if state.scale != 1.0 else frame.copy()
    box = state.live_box()
    if box is not None:
        s = state.scale
        p1 = (int(round(box[0] * s)), int(round(box[1] * s)))
        p2 = (int(round(box[2] * s)), int(round(box[3] * s)))
        cv2.rectangle(disp, p1, p2, (0, 255, 255), 2)
    tag = "box" if box is not None else "NO BOX"
    cv2.putText(disp, f"[{idx + 1}/{total}] {name}  ({tag})", (8, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(disp, "drag=box  ENTER=next  c=clear  p=back  q=quit", (8, disp.shape[0] - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    cv2.imshow(WINDOW, disp)


def fit_scale(w, h):
    return min(1.0, MAX_DISP_W / w, MAX_DISP_H / h)


def main():
    ap = argparse.ArgumentParser(description="Draw a GT target box on each frame in a folder.")
    ap.add_argument("folder", help="folder of frames to annotate (e.g. test_assets/Rifle)")
    ap.add_argument("--labels", default="labels.json", help="labels filename (written in <folder>)")
    args = ap.parse_args()

    folder = Path(args.folder).resolve()
    if not folder.is_dir():
        raise SystemExit(f"not a folder: {folder}")
    frames = list_frames(folder)
    if not frames:
        raise SystemExit(f"no images ({'/'.join(sorted(IMG_EXTS))}) in {folder}")

    labels_path = folder / args.labels
    labels = load_labels(labels_path)

    print(f"[annotate] {folder}: {len(frames)} frames  | labels -> {labels_path}")
    print(f"[annotate] {sum(1 for f in frames if f.name in labels)} already have a box.")
    print("[annotate] drag=box (redraw replaces)  ENTER=save+next  c/Backspace=clear  p/Left=back  q/Esc=quit")

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)

    idx = 0
    cur_idx = -1            # forces a (re)load of state when the frame changes
    state = None
    frame = None
    while 0 <= idx < len(frames):
        if idx != cur_idx:
            name = frames[idx].name
            frame = cv2.imread(str(frames[idx]))
            if frame is None:
                print(f"[annotate] WARNING: could not read {name} — skipping.")
                idx += 1
                cur_idx = -1
                continue
            h, w = frame.shape[:2]
            state = BoxState(fit_scale(w, h), w, h)
            seed = labels.get(name)
            if isinstance(seed, list) and len(seed) == 4:
                state.box = [int(v) for v in seed]
            cv2.setMouseCallback(WINDOW, on_mouse, state)
            cur_idx = idx

        render(frame, state, frames[idx].name, idx, len(frames))
        k = cv2.waitKey(20) & 0xFF
        if k == 255:
            continue  # no key this tick; keep redrawing the live drag

        name = frames[idx].name
        if k in (ord("q"), 27):                 # quit
            break
        elif k in (13, 10):                     # ENTER -> commit + next
            if state.box is not None:
                labels[name] = [int(v) for v in state.box]
            else:
                labels.pop(name, None)
            save_labels(labels_path, labels)
            idx += 1
        elif k in (ord("c"), 8):                # clear (c / Backspace)
            state.box = None
            labels.pop(name, None)
            save_labels(labels_path, labels)
        elif k in (ord("p"), 81):               # back (p / Left=81 on many builds)
            idx = max(0, idx - 1)
            cur_idx = -1

    cv2.destroyAllWindows()
    n_boxed = sum(1 for f in frames if f.name in labels)
    print(f"\n[annotate] done. {n_boxed}/{len(frames)} frames boxed -> {labels_path}")


if __name__ == "__main__":
    main()
