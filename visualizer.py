"""visualizer.py — Process P3: the live dashboard (M4 Task 3).

A read-only consumer that subscribes to the perception state bus (`perception_state_port`,
default :5603) and composes a single OpenCV window from four topics published by
`perception_worker`:

  * TOPIC_MAP   -> the growing top-down (X-Z) occupancy map + camera trajectory. The worker
                   keeps the dense per-keyframe pointmaps in-process (far too big for the JSON
                   bus) and ships only a compact, downsampled occupancy *summary* — a sparse
                   list of occupied grid cells + colors + the trajectory, already in pixel
                   coords (see MapStore.topdown_summary). Each message is a full snapshot, so
                   joining late just means catching up on the next keyframe.
  * TOPIC_POSE  -> SLAM mode, tracking_mode, keyframe/voxel counts, reloc events.
  * TOPIC_PLAN  -> the frontier goal + SLAM-raycast forward clearance (drawn on the map), plus
                   the live pos_y (drone height) and plan-status fields shown in the telemetry
                   panel below.
  * TOPIC_TARGET-> the lifted 3D target position + uncertainty (drawn as a marker on the map
                   and summarized in the status strip).

It ALSO subscribes to `autopilot.py`'s own control bus (`autonomy_control_port`, default :5606,
TOPIC_CONTROL) — the same PUB `io_bridge.py` already reads to actually drive Unity — purely to
read the live FSM `state` (ADVANCE/TRIM/SETTLE/...) and `target_altitude_y` (the autopilot's
self-calibrated desired-height hold target) for display. An extra ZeroMQ SUB on an existing PUB
is free and steals nothing (same reasoning as the frame-bus subscription below).

(DA-V2 depth / TOPIC_DEPTH was removed 2026-07-07; that panel slot now shows autopilot
telemetry instead of a depth placeholder — see `render_telemetry_panel`.)

It also (optionally) subscribes to the frame bus (`frame_bus_port`, default :5601) to show the
live input frame — the frame bus is conflated PUB/SUB, so an extra subscriber is free and never
steals frames from the perception worker.

This process owns no GPU and no SLAM; it is pure display. NO SILENT FALLBACKS (per CLAUDE.md):
`tracking_mode` and reloc events are surfaced prominently in the status strip — a degraded or
non-default SLAM state is always visible, never hidden. If nothing has been received yet the
panels say so rather than faking content.

Layout:  [ status strip                         ]
         [ input frame    ] [                     ]
         [ telemetry panel] [   top-down map + traj  ]
"""

import argparse
import os
import time
from collections import deque
from datetime import datetime

import cv2
import numpy as np
import yaml

import frame_bus

REPO = os.path.dirname(os.path.abspath(__file__))
WINDOW = "Cartographer — live dashboard"

PANEL_W, PANEL_H = 416, 234   # the two 16:9 left-column panels (input + telemetry)
GAP = 12
MAP_SIZE = PANEL_H * 2 + GAP  # square map, same height as the stacked left column
STATUS_H = 48                 # two lines: SLAM state + target estimate
RELOC_FLASH_S = 2.0           # keep the RELOC banner up this long after the event


def load_config(path=None):
    path = path or os.path.join(REPO, "config.yaml")
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


# ---------------------------------------------------------------------------
# Panel renderers
# ---------------------------------------------------------------------------
def _placeholder(w, h, text):
    p = np.full((h, w, 3), 30, np.uint8)
    cv2.putText(p, text, (10, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
    return p


def render_frame_panel(frame, w=PANEL_W, h=PANEL_H):
    if frame is None:
        return _placeholder(w, h, "input: no frame bus")
    p = cv2.resize(frame, (w, h), interpolation=cv2.INTER_AREA)
    cv2.putText(p, "input", (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    return p


def render_telemetry_panel(control, plan, w=PANEL_W, h=PANEL_H):
    """Live autopilot telemetry — replaces the DA-V2 depth panel (removed 2026-07-07). Shows the
    FSM state, current vs. desired (autopilot-locked) height, and plan status, so the operator
    always has these visible instead of only on the map's transient overlay text. `control` is
    the latest TOPIC_CONTROL payload (autopilot -> io_bridge, state + target_altitude_y); `plan`
    is the latest TOPIC_PLAN payload (perception_worker, pos_y + plan-status fields). NO SILENT
    FALLBACK: an unavailable reading prints as `--`, never a stale or guessed number, and the
    whole panel says so explicitly if autopilot.py isn't running at all."""
    if control is None:
        return _placeholder(w, h, "waiting for autopilot on the control bus ...")
    panel = np.full((h, w, 3), 30, np.uint8)
    cv2.putText(panel, "telemetry", (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    state = control.get("state")
    cv2.putText(panel, f"STATE: {state}", (8, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    plan = plan or {}
    pos_y = plan.get("pos_y")
    desired_y = control.get("target_altitude_y")
    hy = f"{pos_y:+.3f}u" if pos_y is not None else "--"
    hd = f"{desired_y:+.3f}u" if desired_y is not None else "--"
    delta = (f"{pos_y - desired_y:+.3f}u" if pos_y is not None and desired_y is not None else "--")
    cv2.putText(panel, f"HEIGHT   pos_y={hy}  desired={hd}  delta={delta}", (8, 78),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

    if not plan.get("plan_valid"):
        cv2.putText(panel, f"PLAN     STALE (SLAM {plan.get('mode')})", (8, 102),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 165, 255), 1)
    else:
        clr = plan.get("forward_clearance_dist")
        be = plan.get("bearing_err")
        cv2.putText(panel,
                    f"PLAN     valid  done={plan.get('done')}  frontiers={plan.get('n_frontiers')}",
                    (8, 102), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
        cv2.putText(panel,
                    f"         blacklisted={plan.get('n_blacklisted') or 0}  "
                    f"bearing_err={be if be is not None else '--'}  "
                    f"clear={f'{clr:.2f}u' if clr is not None else '--'}",
                    (8, 124), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
    return panel


def _world_to_px(x, z, m, size):
    """Project a world (X,Z) point into map-panel pixels using the map summary's bounds.

    Mirrors MapStore.topdown_summary's to_cell (u from X, v flipped from Z) then scales the
    grid cell to panel pixels. Returns (px, py) or None if bounds are unavailable.
    """
    if not m or not m.get("bounds"):
        return None
    x0, x1, z0, _z1 = m["bounds"]
    grid = int(m["grid"])
    span = max(x1 - x0, 1e-6)
    sc = (grid - 1) / span
    u = (x - x0) * sc
    v = (grid - 1) - (z - z0) * sc
    return int(np.clip(u * size / grid, 0, size - 1)), int(np.clip(v * size / grid, 0, size - 1))


def _target_list(target):
    """TOPIC_TARGET carries a LIST of instances ({"targets":[...]}); return it (or [])."""
    return (target or {}).get("targets") or []


def overlay_live_camera(img, m, cam_track, size):
    """Draw the live camera track (recent poses) + current position on a map copy.

    Updated every render tick from the per-frame TOPIC_POSE camera_center, so the drone's
    position/path feel live instead of lagging at keyframe rate (the cached voxel/keyframe
    trajectory only refreshes per keyframe). Yellow = live track + 'now' dot.
    """
    if not m or not cam_track:
        return
    pts = [p for p in (_world_to_px(c[0], c[2], m, size) for c in cam_track) if p is not None]
    if len(pts) >= 2:
        cv2.polylines(img, [np.asarray(pts, np.int32)], False, (0, 255, 255), 1, cv2.LINE_AA)
    if pts:
        cv2.circle(img, pts[-1], 5, (0, 255, 255), -1)
        cv2.circle(img, pts[-1], 7, (0, 0, 0), 1)


# Class ids in the plan's ground raster (must match ground_grid.CLS_*).
_CLS_FREE, _CLS_FRONTIER = 1, 3


def overlay_plan(img, plan, m, size):
    """Overlay the Map-mode plan on the (world-aligned) map panel: explored-FREE cells (dim),
    FRONTIER cells (cyan), the current goal (yellow star), and a heading arrow at the drone — all
    projected through the SAME TOPIC_MAP bounds as the occupancy map so they line up. Also surfaces a
    degraded plan (PLAN-STALE) rather than hiding it (NO SILENT FALLBACKS)."""
    if not plan or not m or not m.get("bounds"):
        return
    x0, x1, z0, _z1 = m["bounds"]
    grid = int(m["grid"])
    span = max(x1 - x0, 1e-6)
    sc = (grid - 1) / span

    def to_px_vec(X, Z):
        u = (X - x0) * sc
        v = (grid - 1) - (Z - z0) * sc
        return (np.clip(u * size / grid, 0, size - 1).astype(int),
                np.clip(v * size / grid, 0, size - 1).astype(int))

    g = plan.get("ground")
    if g and g.get("bounds") and g.get("cls"):
        gx0, gx1, gz0, gz1 = g["bounds"]
        rows, cols = int(g["rows"]), int(g["cols"])
        cls = np.asarray(g["cls"], np.int16)
        if rows > 0 and cols > 0 and cls.size == rows * cols:
            idx = np.arange(cls.size)
            r, c = idx // cols, idx % cols
            # Cell centers -> world (row 0 is +Z up, matching ground_grid.summary's flip).
            X = gx0 + (c + 0.5) / cols * (gx1 - gx0)
            Z = gz1 - (r + 0.5) / rows * (gz1 - gz0)
            px, py = to_px_vec(X, Z)
            free = cls == _CLS_FREE
            front = cls == _CLS_FRONTIER
            img[py[free], px[free]] = (70, 70, 70)            # explored free = dim gray
            for dv in (-1, 0, 1):                              # thicken frontiers so they read
                for du in (-1, 0, 1):
                    img[np.clip(py[front] + dv, 0, size - 1),
                        np.clip(px[front] + du, 0, size - 1)] = (255, 255, 0)  # frontier = cyan

    pos = plan.get("pos")
    goal = plan.get("goal")
    # Blacklisted (unreachable) goals: red X — visible so the operator sees WHY the drone gave up on a
    # frontier behind glass/a wall instead of silently looping (NO SILENT FALLBACK). A PERMANENT entry
    # (dead for good, no cross-round progress) gets a second diamond ring to distinguish it from a soft
    # (this-round) exclusion that will be retried after a reposition.
    bl = plan.get("blacklist") or []
    perm = plan.get("blacklist_permanent") or []
    for i, (bx, bz) in enumerate(bl):
        bu, bv = to_px_vec(np.array([bx]), np.array([bz]))
        px, py = int(bu[0]), int(bv[0])
        cv2.drawMarker(img, (px, py), (0, 0, 255), cv2.MARKER_TILTED_CROSS, 14, 2)
        if i < len(perm) and perm[i]:
            cv2.drawMarker(img, (px, py), (0, 0, 255), cv2.MARKER_DIAMOND, 20, 1)
    if goal is not None:
        gu, gv = to_px_vec(np.array([goal[0]]), np.array([goal[1]]))
        cv2.drawMarker(img, (int(gu[0]), int(gv[0])), (0, 255, 255), cv2.MARKER_STAR, 18, 2)
    clr = plan.get("forward_clearance_dist")
    if pos is not None and plan.get("heading_deg") is not None:
        h = np.radians(plan["heading_deg"])     # 0 = +Z, +90 = +X
        L = 0.08 * span
        pu, pv = to_px_vec(np.array([pos[0]]), np.array([pos[1]]))
        hu, hv = to_px_vec(np.array([pos[0] + L * np.sin(h)]), np.array([pos[1] + L * np.cos(h)]))
        cv2.arrowedLine(img, (int(pu[0]), int(pv[0])), (int(hu[0]), int(hv[0])),
                        (0, 255, 0), 2, tipLength=0.3)
        # Forward-clearance ray (red): drone -> nearest mapped wall ahead, in WORLD units, so the
        # operator sees the geometric stand-off the autopilot stops on (NO SILENT FALLBACK = visible).
        if clr is not None:
            cu, cv = to_px_vec(np.array([pos[0] + clr * np.sin(h)]),
                               np.array([pos[1] + clr * np.cos(h)]))
            cv2.line(img, (int(pu[0]), int(pv[0])), (int(cu[0]), int(cv[0])), (0, 0, 255), 1)
            cv2.circle(img, (int(cu[0]), int(cv[0])), 3, (0, 0, 255), -1)

    if not plan.get("plan_valid"):
        cv2.putText(img, f"PLAN-STALE (SLAM {plan.get('mode')})", (8, 36),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)
    else:
        be = plan.get("bearing_err")
        nbl = plan.get("n_blacklisted") or 0
        cv2.putText(img, f"explore: frontiers={plan.get('n_frontiers')} "
                    f"bearing_err={be if be is not None else '--'} "
                    f"clear={f'{clr:.2f}u' if clr is not None else '--'} "
                    f"{f'blacklist={nbl} ' if nbl else ''}"
                    f"{'DONE' if plan.get('done') else ''}", (8, 36),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)


def render_map_panel(m, size=MAP_SIZE, target=None):
    img = np.full((size, size, 3), 18, np.uint8)
    if not m or not m.get("cells_u"):
        cv2.putText(img, "map: waiting for keyframes...", (12, size // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 1)
        return img

    grid = int(m["grid"])
    scale = size / grid
    u = np.asarray(m["cells_u"], np.int64)
    v = np.asarray(m["cells_v"], np.int64)
    packed = np.asarray(m["cells_rgb"], np.int64)
    bgr = np.stack([packed & 255, (packed >> 8) & 255, (packed >> 16) & 255], axis=1).astype(np.uint8)
    uu = np.clip((u * scale).astype(int), 0, size - 1)
    vv = np.clip((v * scale).astype(int), 0, size - 1)
    ps = max(1, int(round(scale)))  # thicken each cell so the occupancy reads clearly
    if ps <= 1:
        img[vv, uu] = bgr
    else:
        for du in range(ps):
            for dv in range(ps):
                img[np.clip(vv + dv, 0, size - 1), np.clip(uu + du, 0, size - 1)] = bgr

    tu = np.asarray(m.get("traj_u") or [], np.int64)
    tv = np.asarray(m.get("traj_v") or [], np.int64)
    if len(tu) > 1:
        pts = np.stack([np.clip((tu * scale).astype(int), 0, size - 1),
                        np.clip((tv * scale).astype(int), 0, size - 1)], axis=1).astype(np.int32)
        cv2.polylines(img, [pts], False, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.circle(img, tuple(pts[0]), 6, (0, 255, 0), -1)    # start
        cv2.circle(img, tuple(pts[-1]), 6, (0, 255, 255), -1)  # end (drone now)

    # Estimated target marker(s) (magenta), one per detected instance, projected into the map frame.
    tgts = _target_list(target)
    for i, t in enumerate(tgts):
        pos = t.get("position")
        tpx = _world_to_px(pos[0], pos[2], m, size) if pos else None
        if tpx is None:
            continue
        col = (255, 0, 255) if t.get("confident") else (200, 120, 255)
        cv2.drawMarker(img, tpx, col, cv2.MARKER_TILTED_CROSS, 20, 2)
        cv2.circle(img, tpx, 10, col, 2)
        lbl = f"T{i}" if len(tgts) > 1 else "TARGET"
        cv2.putText(img, lbl, (tpx[0] + 12, tpx[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 2)

    cv2.putText(img, f"top-down X-Z  {m.get('n_voxels')} vox  {m.get('n_keyframes')} kf  "
                f"~{m.get('span_world')}u  mode={m.get('tracking_mode')}",
                (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    cv2.putText(img, "traj: green=start  yellow=now  (red path)", (8, size - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 200, 255), 1)
    return img


def render_status(pose, width, reloc_active, target=None):
    # Two-line strip: SLAM state on top, the target estimate below.
    strip = np.full((STATUS_H, width, 3), 45, np.uint8)
    if pose is None:
        cv2.putText(strip, "waiting for perception_worker on the state bus ...", (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
        return strip
    tm = pose.get("tracking_mode")
    txt = (f"tracking={tm}  SLAM={pose.get('mode')}  kf={pose.get('n_keyframes')}  "
           f"vox={pose.get('n_voxels')}  slam={pose.get('slam_ms')}ms")
    # Default tracking mode = green; anything else = orange (a fallback must never be silent).
    col = (0, 255, 0) if tm == "MASt3R" else (0, 165, 255)
    cv2.putText(strip, txt, (8, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
    if reloc_active:
        cv2.putText(strip, "RELOC!", (width - 95, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)

    tgts = _target_list(target)
    if tgts:
        head = f"{len(tgts)} TARGETS" if len(tgts) > 1 else "TARGET"
        lbl = (target.get("label") or tgts[0].get("label") or "?")
        parts = []
        for t in tgts:
            p = t.get("position", [0, 0, 0])
            parts.append(f"({p[0]:.2f},{p[1]:.2f},{p[2]:.2f})"
                         f"{'' if t.get('confident') else '?'}")
        n_conf = sum(1 for t in tgts if t.get("confident"))
        ttxt = f"{head} [{lbl}]  " + "  ".join(parts) + f"  conf={n_conf}/{len(tgts)}"
        tcol = (255, 0, 255) if n_conf else (200, 120, 255)
        cv2.putText(strip, ttxt, (8, STATUS_H - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.42, tcol, 1)
    else:
        cv2.putText(strip, "TARGET: not yet localized", (8, STATUS_H - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (120, 120, 120), 1)
    return strip


# ---------------------------------------------------------------------------
# Dashboard state
# ---------------------------------------------------------------------------
class Dashboard:
    """Holds the latest payload per topic + the latest frame, and composes the window.

    The map panel is cached and only re-rendered when a new TOPIC_MAP snapshot arrives
    (~once per keyframe); the cheap frame + telemetry panels redraw every tick.
    """

    def __init__(self):
        self.frame = None
        self.pose = None
        self.map = None
        self.target = None
        self.plan = None
        self.control = None    # latest TOPIC_CONTROL payload (autopilot's own control bus)
        self.cam_track = deque(maxlen=600)   # recent world camera centers (live, per-frame)
        self._map_img = None
        self._map_sig = None
        self._last_reloc = 0.0

    def update(self, topic, payload):
        if topic == "pose":
            self.pose = payload
            cc = payload.get("camera_center")
            if cc is not None:
                self.cam_track.append(cc)
            if payload.get("reloc_event"):
                self._last_reloc = time.monotonic()
        elif topic == "map":
            self.map = payload
            self._map_img = None  # invalidate cache
        elif topic == "target":
            self.target = payload
            self._map_img = None  # redraw map with the updated target marker
        elif topic == "plan":
            self.plan = payload   # drawn live on the map copy each tick (no cache invalidation)
        elif topic == "control":
            self.control = payload   # autopilot FSM state + target_altitude_y (telemetry panel)

    def _map_image(self):
        tpos = tuple(tuple(t.get("position") or ()) for t in _target_list(self.target))
        sig = None if self.map is None else (self.map.get("n_keyframes"), self.map.get("frame_id"), tpos)
        if self._map_img is None or sig != self._map_sig:
            self._map_img = render_map_panel(self.map, target=self.target)
            self._map_sig = sig
        return self._map_img

    def render(self):
        frame_p = render_frame_panel(self.frame)
        tel_p = render_telemetry_panel(self.control, self.plan)
        # Cached voxel/keyframe base + live camera overlay (per-frame, so position feels live).
        map_p = self._map_image().copy()
        overlay_live_camera(map_p, self.map, self.cam_track, map_p.shape[0])
        overlay_plan(map_p, self.plan, self.map, map_p.shape[0])

        col_gap = np.zeros((GAP, PANEL_W, 3), np.uint8)
        left = np.vstack([frame_p, col_gap, tel_p])            # (MAP_SIZE, PANEL_W)
        row_gap = np.zeros((left.shape[0], GAP, 3), np.uint8)
        body = np.hstack([left, row_gap, map_p])               # (MAP_SIZE, width)

        reloc_active = (time.monotonic() - self._last_reloc) < RELOC_FLASH_S
        status = render_status(self.pose, body.shape[1], reloc_active, self.target)
        return np.vstack([status, body])


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
def _open_video_writer(fps):
    """Open a VideoWriter sized to the dashboard's fixed composed dimensions (no need to wait for a
    sample frame — the layout constants determine it exactly), timestamped like every other module's
    OUTPUT/diag/<ts>_<role> artifact. Pure CPU/software encode (mp4v via OpenCV's bundled FFmpeg) —
    this process touches no GPU today and this doesn't change that."""
    out_dir = os.path.join(REPO, "OUTPUT", "diag")
    os.makedirs(out_dir, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(out_dir, f"{ts}_visualizer.mp4")
    width = PANEL_W + GAP + MAP_SIZE
    height = STATUS_H + MAP_SIZE
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(path, fourcc, fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError(f"could not open video writer for {path} (mp4v codec unavailable?)")
    print(f"[visualizer] recording -> {path} ({width}x{height} @ {fps:g}fps target)")
    return writer


def run(cfg, show_frame=True, record=False, record_fps=15.0, stop_file=None):
    pstate_port = cfg["network"]["perception_state_port"]
    frame_port = cfg["network"]["frame_bus_port"]
    ctrl_port = cfg["network"]["autonomy_control_port"]
    state_sub = frame_bus.StateSubscriber(pstate_port)  # all topics (pose/map/plan/target)
    frame_sub = frame_bus.FrameSubscriber(frame_port) if show_frame else None
    # A second, independent SUB on autopilot.py's own control bus (io_bridge already reads this
    # to drive Unity) -- purely to read `state`/`target_altitude_y` for the telemetry panel.
    # Lazy-connect: fine whether or not autopilot.py is running yet.
    ctrl_sub = frame_bus.StateSubscriber(ctrl_port, topics=[frame_bus.TOPIC_CONTROL])

    print(f"[visualizer] state bus SUB :{pstate_port} (pose+map+plan+target)"
          + (f" | frame bus SUB :{frame_port}" if frame_sub else " | input frame OFF")
          + f" | control bus SUB :{ctrl_port} (autopilot state+target_altitude_y)")
    print("[visualizer] === READY === waiting for perception_worker ('q' to quit).\n")

    writer = _open_video_writer(record_fps) if record else None
    write_interval = 1.0 / record_fps
    last_write_t = 0.0

    dash = Dashboard()
    try:
        while True:
            # Graceful-stop sentinel (mirrors autopilot.py's _FileStopEvent / perception_worker.py's
            # own poll): a launcher that hard-terminates a CREATE_NEW_CONSOLE child on Windows skips
            # `finally` entirely, which would skip `writer.release()` below and leave --record's MP4
            # without its moov atom (unplayable, even though frame data was written) -- confirmed by
            # reproducing exactly that corruption via a hard TerminateProcess. Checked every
            # iteration so a stop request is noticed even while idle (no new state arriving).
            if stop_file is not None and os.path.exists(stop_file):
                print("[visualizer] stop-file seen -> shutting down cleanly")
                break
            # Drain the (non-conflated) state bus so we always render the freshest of each topic.
            got = state_sub.recv(timeout_ms=30)
            while got is not None:
                dash.update(*got)
                got = state_sub.recv(timeout_ms=0)
            # Drain the control bus the same way (autopilot publishes at 20 Hz; we only want the
            # freshest state/target_altitude_y reading each render tick).
            got = ctrl_sub.recv(timeout_ms=0)
            while got is not None:
                dash.update(*got)
                got = ctrl_sub.recv(timeout_ms=0)
            if frame_sub is not None:
                fr = frame_sub.recv(timeout_ms=0)
                if fr is not None:
                    dash.frame = fr[0]
            img = dash.render()
            cv2.imshow(WINDOW, img)
            if writer is not None:
                # Wall-clock throttled (the render loop's own tick rate is NOT fixed) so the output
                # plays back at roughly real elapsed time instead of assuming a fixed tick rate.
                now = time.monotonic()
                if now - last_write_t >= write_interval:
                    writer.write(img)
                    last_write_t = now
            if (cv2.waitKey(15) & 0xFF) == ord("q"):
                break
    except KeyboardInterrupt:
        pass
    finally:
        print("[visualizer] shutting down ...")
        state_sub.close()
        ctrl_sub.close()
        if frame_sub is not None:
            frame_sub.close()
        if writer is not None:
            writer.release()
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass


def main():
    ap = argparse.ArgumentParser(description="Cartographer visualizer (P3): live map + telemetry dashboard")
    ap.add_argument("--config", default=None)
    ap.add_argument("--no-frame", action="store_true",
                    help="don't subscribe to the frame bus (skip the live input panel)")
    ap.add_argument("--record", action="store_true",
                    help="also write the composed dashboard to OUTPUT/diag/<ts>_visualizer.mp4 "
                         "(CPU-side encode; this process owns no GPU)")
    ap.add_argument("--record-fps", type=float, default=15.0,
                    help="target playback fps for --record (wall-clock throttled)")
    ap.add_argument("--stop-file", default=None,
                    help="path to a sentinel file; when it appears, exit the loop CLEANLY (releases "
                         "the --record video properly) instead of being hard-terminated by a "
                         "launcher. Mirrors autopilot.py's --stop-file.")
    args = ap.parse_args()
    # A stale sentinel from a crashed prior run would stop us instantly -- clear it before we start.
    if args.stop_file and os.path.exists(args.stop_file):
        try:
            os.remove(args.stop_file)
        except OSError:
            pass
    run(load_config(args.config), show_frame=not args.no_frame,
        record=args.record, record_fps=args.record_fps, stop_file=args.stop_file)


if __name__ == "__main__":
    main()
