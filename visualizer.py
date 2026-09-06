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

It also subscribes to `visrec_canvas_port` (session 60, C9) — the composed F_LKG/LIVE debug canvas
`autopilot.py` used to show in its own OS window (imshow), now published here instead and rendered
as the dashboard's leftmost column, so it lands in the `--record` MP4 too.

This process owns no GPU and no SLAM; it is pure display. NO SILENT FALLBACKS (per CLAUDE.md):
`tracking_mode` and reloc events are surfaced prominently in the status strip — a degraded or
non-default SLAM state is always visible, never hidden. If nothing has been received yet the
panels say so rather than faking content.

Layout:  [ status strip                                     ]
         [ LKG canvas ] [ input frame    ] [                     ]
         [            ] [ telemetry panel] [   top-down map + traj  ]
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
# Session 62: the composed dashboard's exact pixel size, defined ONCE. This expression used to be
# written out longhand in _open_video_writer, in two self-tests AND in salvage_flight.repair_mp4 --
# and when session 60 added the leftmost LKG column (+PANEL_W+GAP, 908->1336 wide) only the first
# three were updated. salvage_flight then "repaired" the crashed flight 20260905_113346 by
# prepending a 908-wide VOL header to a 1336-wide elementary stream: every row wrapped 428px early
# and the file decoded into garbage. One definition, so a layout change can never again leave a
# stale copy behind.
CANVAS_W = PANEL_W + GAP + PANEL_W + GAP + MAP_SIZE   # LKG | input over telemetry | top-down map
CANVAS_H = STATUS_H + MAP_SIZE
RELOC_FLASH_S = 2.0           # keep the RELOC banner up this long after the event

LKG_CANVAS_STALE_S = 300.0  # session 61, revised same day: the operator found the grey swap-out
                            #   more annoying than useful in normal flight, so this is pushed to 5
                            #   minutes -- effectively never during an ordinary session -- rather
                            #   than removing the guard outright. Still catches a genuinely dead
                            #   publisher (a crashed/hung autopilot.py) without flapping the panel.
LKG_TEXT_SCALE = 0.42       # drawn at PANEL resolution, so this is the size actually seen
LKG_TEXT_LINE_H = 15
LKG_TEXT_PAD = 8
LKG_TEXT_MAX_LINES = 6


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


def _wrap_text(text, max_chars):
    """Greedy word-wrap `text` into lines of at most `max_chars` (session 52). A single word
    longer than `max_chars` is hard-split across as many lines as it needs rather than
    overflowing the panel. Returns [] for None/empty input; never raises."""
    if not text:
        return []
    lines = []
    cur = ""
    for word in text.split():
        while len(word) > max_chars:
            if cur:
                lines.append(cur)
                cur = ""
            lines.append(word[:max_chars])
            word = word[max_chars:]
        candidate = f"{cur} {word}".strip()
        if len(candidate) <= max_chars:
            cur = candidate
        else:
            if cur:
                lines.append(cur)
            cur = word
    if cur:
        lines.append(cur)
    return lines


def _wrap_text_segments(segments, max_px, font, scale):
    """Greedily pack whole `segments` (list[str]) into lines whose rendered width, per
    cv2.getTextSize(line, font, scale, 1)[0][0], is <= max_px. Segments joined by two spaces.

    Session 61: segments are NEVER split mid-token, so no field can render half-visible (Finding D
    lost scale/size/closer/src/age off the right edge of a 512px canvas). NO SILENT FALLBACK: a lone
    segment wider than max_px gets its own hard-split line (like the existing char-based
    `_wrap_text`) — never a truncation. Returns [] for a falsy `segments`.
    """
    if not segments:
        return []
    lines = []
    cur = ""
    for seg in segments:
        candidate = f"{cur}  {seg}" if cur else seg
        if cv2.getTextSize(candidate, font, scale, 1)[0][0] <= max_px:
            cur = candidate
            continue
        if cur:
            lines.append(cur)
            cur = ""
        if cv2.getTextSize(seg, font, scale, 1)[0][0] <= max_px:
            cur = seg
            continue
        # A single segment alone is still too wide -- hard-split it character by character rather
        # than truncate (NO SILENT FALLBACK: every character must land somewhere on screen).
        piece = ""
        for ch in seg:
            candidate_piece = piece + ch
            if cv2.getTextSize(candidate_piece, font, scale, 1)[0][0] <= max_px:
                piece = candidate_piece
            else:
                if piece:
                    lines.append(piece)
                piece = ch
        cur = piece
    if cur:
        lines.append(cur)
    return lines


def render_frame_panel(frame, w=PANEL_W, h=PANEL_H):
    if frame is None:
        return _placeholder(w, h, "input: no frame bus")
    p = cv2.resize(frame, (w, h), interpolation=cv2.INTER_AREA)
    cv2.putText(p, "input", (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    return p


def push_lines(push):
    """Session 62: one line describing what the last PARALLAX_PUSH achieved, for the telemetry panel.

    Watch-only -- nothing in the autopilot branches on this verdict yet; it is here so the operator can
    see whether the stuck/moved call would have been right before it is wired to anything. NO SILENT
    FALLBACK: `unknown` (SLAM gave too few distinct poses to judge) renders as its own word, never as
    `stuck` -- the two mean completely different things and only one of them justifies a reaction."""
    if not push:
        return []
    v = push.get("verdict") or "--"
    t = push.get("traveled")
    d = push.get("drift")
    t_txt = "--" if t is None else f"{t:.2f}u"
    d_txt = "--" if d is None else f"{d:.2f}u"
    run = push.get("stuck_run") or 0
    tail = f"  x{run}" if v == "stuck" and run > 1 else ""
    return [f"PUSH {push.get('dir') or '--'} {push.get('why') or '--'}: moved {t_txt} "
            f"drift {d_txt} ({push.get('poses') or 0}p) -> {v}{tail}"]


def recovery_lines(recovery):
    """Session 62: turn `control["recovery"]` (autopilot's `ExploreController.recovery_status`) into at
    most two operator-readable lines describing WHERE we are in a loss episode.

    The panel's bottom two rows used to carry the session-52 notice/planner-event block, which the
    operator judged uninformative in the situation that matters most -- a plan going stale, with three
    FALLBACK episodes flown and no way to tell the phases apart on screen. So during a loss episode
    these lines take that space, and the notice block returns the moment the episode ends (it is not
    deleted, only outranked -- it is still the only surfacing of a timeout/forced-escape).

    NO SILENT FALLBACK: a phase with no clock deadline is NOT drawn with an invented one (`limit_s` is
    None and the elapsed time is shown bare), and an unknown/absent phase says so rather than guessing.
    Returns [] for None, so the caller falls through to the notice block."""
    if not recovery:
        return []
    phase = recovery.get("phase")
    el, lim = recovery.get("elapsed_s"), recovery.get("limit_s")
    if el is None:
        clock = ""
    elif lim is None:
        clock = f" {el:.1f}s"
    else:
        clock = f" {el:.1f}/{lim:.0f}s"
    cyc = recovery.get("cycle") or 0

    if phase is None:
        # In a loss episode but the sweep has not been entered yet -- this is the grace hold.
        grace = recovery.get("grace_s")
        lost = recovery.get("loss_elapsed_s")
        head = "LOSS GRACE" + (f" {lost:.1f}/{grace:.0f}s" if lost is not None and grace else "")
        return [f"{head}  holding still before FALLBACK"]
    if phase == "INITIAL_WAIT":
        body = f"INITIAL_WAIT{clock}  letting a transient patch clear"
    elif phase == "BACKOFF":
        body = "BACKOFF  backing off to widen the view"
    elif phase == "BACKOFF_WAIT":
        body = f"BACKOFF_WAIT{clock}  looking for F_LKG from here"
    elif phase == "TURN":
        body = f"TURN  cycle {cyc}  cum {recovery.get('cum_deg') or 0:.0f}deg"
    elif phase == "PUSH":
        body = f"PUSH {recovery.get('push_dirn') or '--'}  cycle {cyc}"
    elif phase == "WAIT_POST":
        body = f"WAIT_POST{clock}  cycle {cyc}  settling after the push"
    elif phase == "SERVO":
        lost_s = recovery.get("servo_lost_s")
        if lost_s is not None:
            g = recovery.get("servo_lost_grace_s")
            body = f"SERVO  match lost {lost_s:.1f}/{g:.1f}s  holding" if g else f"SERVO  match lost {lost_s:.1f}s"
        else:
            v = recovery.get("servo_verdict") or "--"
            if v == "EQUAL":
                body = (f"SERVO  EQUAL  held {recovery.get('servo_frames') or 0}"
                        f"/{recovery.get('servo_hold_frames') or 0} solved frames")
            elif v == "LIVE":
                body = "SERVO  LIVE (too close) -> backing off"
            elif v == "LKG":
                body = "SERVO  LKG (too far) -> nudging forward"
            else:
                body = f"SERVO  {v}"
    else:
        body = f"{phase}{clock}"

    lost = recovery.get("loss_elapsed_s")
    lines = [f"FALLBACK  {body}"]
    if lost is not None:
        lines.append(f"          blind for {lost:.0f}s")
    return lines[:2]


def render_telemetry_panel(control, plan, w=PANEL_W, h=PANEL_H):
    """Live autopilot telemetry — replaces the DA-V2 depth panel (removed 2026-07-07). Shows the
    FSM state (with time-in-state), current vs. desired (autopilot-locked) height, plan status,
    (while the plan is valid) live straight-line distance to the current goal, the SLAM_HOLD
    forced-hop countdown (session 52), the F_LKG source + age (session 60: F_LKG now arrives
    pre-resolved off its own bus, so AGE-OUT is structurally impossible and no longer rendered here;
    session 61 (C9) adds back a plain elapsed-time AGE reading, a different thing -- how old the
    current reference is, not a staleness verdict), and a 1-2
    line notice block surfacing the latest timeout (`control["notice"]`) and/or the latest planner
    event (`plan["planner_event"]`) so the operator always has these visible instead of only on the
    map's transient overlay text or the console log. `control` is the latest TOPIC_CONTROL payload
    (autopilot -> io_bridge, state + target_altitude_y + state_since_s + slam_hold + notice +
    visrec_lkg); `plan` is the latest TOPIC_PLAN payload (perception_worker, pos_y + pos/goal +
    plan-status fields). NO SILENT FALLBACK: an unavailable reading prints as `--` (never a stale
    or guessed number), the SLAMHOLD row prints as `SLAMHOLD  --` when no slam_hold payload is
    present, the LKG segment prints as `LKG=--` when no visrec_lkg payload is present, the notice
    block renders nothing when neither source has content, and the whole panel says so explicitly
    if autopilot.py isn't running at all."""
    if control is None:
        return _placeholder(w, h, "waiting for autopilot on the control bus ...")
    panel = np.full((h, w, 3), 30, np.uint8)
    cv2.putText(panel, "telemetry", (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    state = control.get("state")
    state_since = control.get("state_since_s")
    t_txt = f"{state_since:.1f}s" if state_since is not None else "--"
    cv2.putText(panel, f"STATE: {state}   t={t_txt}", (8, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

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
        pos, goal = plan.get("pos"), plan.get("goal")
        dg = f"{np.hypot(pos[0] - goal[0], pos[1] - goal[1]):.2f}u" if pos is not None and goal is not None else "--"
        cv2.putText(panel, f"GOAL     dist={dg}", (8, 146),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

    # SLAM solve time (session 44 ask): rides on EVERY plan, valid or not, so it's shown unconditionally --
    # this is the number that explains a stuck settle-gate (TRIM_RESUME_WAIT/SLAM_HOLD need several
    # CONSECUTIVE frames under slam_slow_ms, 1000ms by default) long before the FSM state itself looks wrong.
    ms = plan.get("slam_ms")
    ms_txt = f"{ms:.0f}ms" if ms is not None else "--"
    ms_color = (0, 0, 255) if (ms is not None and ms >= 1000.0) else (255, 255, 255)  # red once >= slow threshold
    cv2.putText(panel, f"SLAM     ms={ms_txt}", (8, 168),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, ms_color, 1)

    # F_LKG source (session 60), appended to the SLAM row above (no new row -- panel is full). F_LKG now
    # arrives pre-resolved off its own bus (run_explore's lkg_sub), so age-out can no longer occur -- the
    # red STALE indicator this used to carry is retired along with it. NO SILENT FALLBACK: absent payload
    # still prints "--", never a guessed source.
    visrec_lkg = control.get("visrec_lkg")
    if visrec_lkg is None:
        lkg_txt = "LKG=--"
    else:
        _lkg_age = visrec_lkg.get("age_s")
        lkg_txt = f"LKG={visrec_lkg.get('src')} age={'n/a' if _lkg_age is None else f'{_lkg_age}s'}"
    cv2.putText(panel, lkg_txt, (190, 168), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

    # SLAM_HOLD forced-hop countdown (session 52): mirrors the SLAM ms= red-past-threshold
    # treatment above so an operator sees a stuck hold approaching its forced-hop deadline
    # before the FSM state itself looks wrong. NO SILENT FALLBACK: absent payload -> "--".
    slam_hold = control.get("slam_hold")
    if slam_hold is None:
        cv2.putText(panel, "SLAMHOLD  --", (8, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    else:
        now_s = slam_hold.get("now_s")
        deadline_s = slam_hold.get("deadline_s")
        now_txt = f"{now_s:.1f}s" if now_s is not None else "--"
        deadline_txt = f"{deadline_s:.1f}s" if deadline_s is not None else "--"
        total_txt = f"{slam_hold.get('total_s'):.1f}s" if slam_hold.get("total_s") is not None else "--"
        hold_color = (0, 0, 255) if (now_s is not None and deadline_s is not None and now_s >= deadline_s) \
            else (255, 255, 255)
        cv2.putText(panel,
                    f"SLAMHOLD now={now_txt}/{deadline_txt}  n={slam_hold.get('entries')}  total={total_txt}",
                    (8, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.45, hold_color, 1)

    # Notice block (session 52): latest timeout/forced-escape (`control["notice"]`) then the
    # latest planner event (`plan["planner_event"]`, a list -> last entry), so the operator sees
    # WHY the FSM did something unusual without having to scroll the console log. Capped at 2
    # rendered lines total; a truncated tail gets a trailing "..." (NO SILENT FALLBACK would be
    # dropping the line with no indication more text existed).
    # Session 62: during a LOSS EPISODE these two rows carry the recovery FSM's own position instead
    # -- which phase of the FALLBACK ladder we are in, and how far through it. The operator flew three
    # FALLBACK episodes unable to tell the phases apart on screen and judged the notice block useless in
    # exactly that situation. The notice block is NOT removed, only outranked: it comes straight back the
    # moment the episode ends, and it remains the only place a timeout/forced-escape is surfaced.
    # Drawn CYAN so it reads as live state, distinct from the orange after-the-fact notice.
    # Session 62 priority for these two rows, highest first: an active loss episode (where we are in the
    # recovery ladder) > the last parallax push's measurement (watch-only) > the session-52 notice block.
    # Each outranks the next only while it has something to say, so nothing is permanently hidden.
    rec_lines = recovery_lines(control.get("recovery"))
    if rec_lines:
        for i, line in enumerate(rec_lines[:2]):
            cv2.putText(panel, line, (8, 210 + i * 16), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 0), 1)
        return panel

    psh_lines = push_lines(control.get("push"))
    if psh_lines:
        # Amber for `stuck`, plain white otherwise -- a verdict worth noticing should look different from
        # a routine one, but `unknown` must NOT borrow the alarm colour (it is an absence of evidence).
        _stuck = (control.get("push") or {}).get("verdict") == "stuck"
        for i, line in enumerate(psh_lines[:2]):
            cv2.putText(panel, line, (8, 210 + i * 16), cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                        (0, 200, 255) if _stuck else (200, 200, 200), 1)
        return panel

    notice_lines = []
    notice = control.get("notice")
    if notice is not None:
        notice_lines = _wrap_text(
            f"! {notice.get('kind')} ({notice.get('age_s'):.0f}s ago): {notice.get('text')}", 68)
    events = plan.get("planner_event")
    event_lines = _wrap_text(events[-1], 68) if events else []
    all_lines = (notice_lines + event_lines)[:2]
    if len(notice_lines) + len(event_lines) > 2 and all_lines:
        tail = all_lines[-1][:65].rstrip()
        all_lines[-1] = tail + "..."
    for i, line in enumerate(all_lines):
        cv2.putText(panel, line, (8, 210 + i * 16), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 165, 255), 1)
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
    FRONTIER cells (cyan), the current goal (blue star if a bbox-corner-tour goal, else yellow star),
    and a heading arrow at the drone — all projected through the SAME TOPIC_MAP bounds as the
    occupancy map so they line up. Also surfaces a degraded plan (PLAN-STALE) rather than hiding it
    (NO SILENT FALLBACKS)."""
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
        # Session 57: a bbox-corner-tour goal (planner.sweeping) is drawn BLUE instead of the usual
        # frontier YELLOW, so the operator can tell at a glance which behaviour picked this goal.
        goal_bgr = (255, 0, 0) if plan.get("goal_is_corner") else (0, 255, 255)   # corner = BLUE, frontier = yellow
        cv2.drawMarker(img, (int(gu[0]), int(gv[0])), goal_bgr, cv2.MARKER_STAR, 18, 2)
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


def render_lkg_panel(canvas, info=None, age_s=None, w=PANEL_W, h=MAP_SIZE):
    """Session 60 (C9): the retired OS debug window's replacement. `autopilot.py` composes the
    F_LKG/LIVE canvas (`visual_recovery.VisualRecoveryProbe.compose_stacked_live`) on a cadence and
    publishes it on `visrec_canvas_port` instead of showing it with imshow; this renders that canvas
    as the dashboard's new leftmost column so it lands in the recording too.

    IMAGE INTEGRITY (CLAUDE.md) disclosure — the ONE display-only scale in this session: the canvas
    arrives at its native (512px-wide transport-frame-derived) resolution; this column is a fixed
    `PANEL_W` wide, so it is fit-scaled here preserving aspect and letterboxed, never cropped. This
    touches no model input — the full-resolution canvas still reaches the PNG evidence trail under
    OUTPUT/diag/<ts>_visrec/ completely unchanged (see `_visrec_debug_sink`).

    GREY placeholder (not black, matching every other "waiting" panel in this file) until the first
    canvas arrives — idle must read as idle, never as signal-lost.

    Session 61: `info` is the F_LKG/LIVE canvas's field segments (C5), drawn HERE at panel
    resolution instead of baked into the 512px canvas and then downscaled — crisp and complete.
    `age_s` is how long ago the canvas arrived; past LKG_CANVAS_STALE_S the panel greys out rather
    than showing a frozen image that reads as current (the 22:40:29 failure). Revised same day:
    LKG_CANVAS_STALE_S is now 5 minutes (operator found the swap-out more annoying than useful in
    normal flight), and the yellow "F_LKG (reference)"/"LIVE" labels session 60's canvas used to
    bake in are drawn back HERE (compose_stacked_live composes no text at all).
    """
    if age_s is not None and age_s > LKG_CANVAS_STALE_S:
        return _placeholder(w, h, f"LKG canvas stale ({age_s:.1f}s)")
    if canvas is None:
        return _placeholder(w, h, "LKG: waiting for autopilot canvas ...")
    lines = _wrap_text_segments(info, w - 12, cv2.FONT_HERSHEY_SIMPLEX, LKG_TEXT_SCALE)
    lines = lines[:LKG_TEXT_MAX_LINES]
    text_h = (len(lines) * LKG_TEXT_LINE_H + LKG_TEXT_PAD) if lines else 0
    avail_h = h - text_h
    ch, cw = canvas.shape[:2]
    scale = min(w / cw, avail_h / ch)
    new_w, new_h = max(1, int(round(cw * scale))), max(1, int(round(ch * scale)))
    resized = cv2.resize(canvas, (new_w, new_h), interpolation=cv2.INTER_AREA)
    panel = np.full((h, w, 3), 30, np.uint8)
    x0 = (w - new_w) // 2
    y0 = text_h + (avail_h - new_h) // 2
    panel[y0:y0 + new_h, x0:x0 + new_w] = resized
    # Session 61, revised same day: the yellow "F_LKG (reference)" / "LIVE" labels session 60's
    # side-by-side canvas used to bake in are back, drawn here instead since compose_stacked_live
    # composes no text at all. Both halves of the source canvas are equal height (both come from the
    # same 512x288 transport frame), so the split sits at the image's own vertical midpoint.
    half_h = new_h // 2
    cv2.putText(panel, "F_LKG (reference)", (x0 + 6, y0 + half_h - 8), cv2.FONT_HERSHEY_SIMPLEX,
               0.45, (0, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(panel, "LIVE", (x0 + 6, y0 + new_h - 8), cv2.FONT_HERSHEY_SIMPLEX,
               0.45, (0, 255, 255), 1, cv2.LINE_AA)
    for i, line in enumerate(lines):
        cv2.putText(panel, line, (6, LKG_TEXT_LINE_H * (i + 1)), cv2.FONT_HERSHEY_SIMPLEX,
                   LKG_TEXT_SCALE, (220, 220, 220), 1)
    return panel


def backend_status_text(map_payload):
    """Session 64: compact backend-thread status for the top strip -- "" when the payload predates
    Session 64 (no "backend_mode" key), so an older flight/replay renders exactly as it always did.
    FAILED is the operator-visible signal that global optimization has stopped (CLAUDE.md rule 3);
    a nonzero clobber count is reported but is NOT an alarm (see render_status: expected + self-
    correcting, colouring it red would train the operator to ignore red).

    Session 65: appends the bounded global-optimisation window's state after the clobber suffix.
    OFF (the default, and any payload predating session 65) appends nothing, so this renders
    byte-identical to session 64's output until the operator opts in. ON shows the configured
    window size and how many keyframes the last solve actually touched (`w{W}k{solve_kf}`), plus
    `a{anchors}` only when a loop edge dragged an out-of-window keyframe in. SHADOW shows only `W`,
    marked with a `?` -- it is priced but NOT applied, so a solve_kf number here would describe a
    cut that never happened to the graph this frame."""
    if not map_payload or "backend_mode" not in map_payload:
        return ""
    mode = map_payload.get("backend_mode")
    if mode == "FAILED":
        return "bk=FAILED"
    txt = f"bk={mode} q{map_payload.get('backend_queue_depth', 0)}"
    clobbers = map_payload.get("backend_pose_clobbers", 0)
    if clobbers:
        txt += f" clob{clobbers}"
    win_mode = map_payload.get("backend_window_mode", "OFF")
    if win_mode == "ON":
        txt += f" w{map_payload.get('backend_window_kf', 0)}k{map_payload.get('backend_solve_kf', 0)}"
        anchors = map_payload.get("backend_anchors", 0)
        if anchors:
            txt += f"a{anchors}"
    elif win_mode == "SHADOW":
        txt += f" w?{map_payload.get('backend_window_kf', 0)}"
    return txt


def render_status(pose, width, reloc_active, target=None, map_payload=None):
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
    bk_txt = backend_status_text(map_payload)
    if bk_txt:
        (txt_w, _), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        bk_col = (0, 0, 255) if bk_txt == "bk=FAILED" else col
        cv2.putText(strip, bk_txt, (8 + txt_w + 16, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, bk_col, 1)
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
        self.lkg_canvas = None  # Session 60 (C9): latest F_LKG/LIVE canvas off visrec_canvas_port
        self.lkg_canvas_info = None   # Session 61 (C5): the canvas's field segments (banner_fields)
        self.lkg_canvas_t = None      # Session 61: time.monotonic() when the last canvas ARRIVED --
                                       # drives the stale-placeholder guard (LKG_CANVAS_STALE_S)
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
        # Session 61: age_s drives the stale-placeholder guard -- None (never a canvas yet) is
        # distinct from a real elapsed time, so it must not be coerced to 0.0 here.
        age_s = None if self.lkg_canvas_t is None else time.monotonic() - self.lkg_canvas_t
        lkg_p = render_lkg_panel(self.lkg_canvas, self.lkg_canvas_info, age_s)  # (MAP_SIZE, PANEL_W)
        frame_p = render_frame_panel(self.frame)
        tel_p = render_telemetry_panel(self.control, self.plan)
        # Cached voxel/keyframe base + live camera overlay (per-frame, so position feels live).
        map_p = self._map_image().copy()
        overlay_live_camera(map_p, self.map, self.cam_track, map_p.shape[0])
        overlay_plan(map_p, self.plan, self.map, map_p.shape[0])

        col_gap = np.zeros((GAP, PANEL_W, 3), np.uint8)
        left = np.vstack([frame_p, col_gap, tel_p])            # (MAP_SIZE, PANEL_W)
        row_gap = np.zeros((left.shape[0], GAP, 3), np.uint8)
        body = np.hstack([lkg_p, row_gap, left, row_gap, map_p])  # (MAP_SIZE, width)

        reloc_active = (time.monotonic() - self._last_reloc) < RELOC_FLASH_S
        status = render_status(self.pose, body.shape[1], reloc_active, self.target, self.map)
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
    width, height = CANVAS_W, CANVAS_H   # session 60 (C9) added the leftmost LKG column; see CANVAS_W
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
    canvas_port = cfg["network"]["visrec_canvas_port"]
    state_sub = frame_bus.StateSubscriber(pstate_port)  # all topics (pose/map/plan/target)
    frame_sub = frame_bus.FrameSubscriber(frame_port) if show_frame else None
    # A second, independent SUB on autopilot.py's own control bus (io_bridge already reads this
    # to drive Unity) -- purely to read `state`/`target_altitude_y` for the telemetry panel.
    # Lazy-connect: fine whether or not autopilot.py is running yet.
    ctrl_sub = frame_bus.StateSubscriber(ctrl_port, topics=[frame_bus.TOPIC_CONTROL])
    # Session 60 (C9): the retired OS debug window's replacement -- autopilot.py publishes the composed
    # F_LKG/LIVE canvas here instead of showing it with imshow. Lazy-connect, same as ctrl_sub.
    canvas_sub = frame_bus.FrameSubscriber(canvas_port)

    print(f"[visualizer] state bus SUB :{pstate_port} (pose+map+plan+target)"
          + (f" | frame bus SUB :{frame_port}" if frame_sub else " | input frame OFF")
          + f" | control bus SUB :{ctrl_port} (autopilot state+target_altitude_y)"
          + f" | LKG canvas SUB :{canvas_port}")
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
            cv_frame = canvas_sub.recv(timeout_ms=0)
            if cv_frame is not None:
                dash.lkg_canvas = cv_frame[0]
                # Session 61 (C11): meta carries banner_fields()'s 12 segments as {"info": [...]}.
                dash.lkg_canvas_info = (cv_frame[1] or {}).get("info") or []
                dash.lkg_canvas_t = time.monotonic()
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
        canvas_sub.close()
        if frame_sub is not None:
            frame_sub.close()
        if writer is not None:
            writer.release()
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass


# ==============================================================================
# Self-test: deterministic synthetic overlay_plan calls (no window, no socket, no hardware).
# ==============================================================================
def run_self_test():
    ok = True

    def case(name, good):
        nonlocal ok
        ok = ok and good
        print(f"[self-test] {'PASS' if good else 'FAIL'}  {name}")

    def has_bgr(img, bgr):
        return bool(np.any(np.all(img == np.array(bgr, dtype=np.uint8), axis=-1)))

    size = 200
    m = {"bounds": (0.0, 10.0, 0.0, 10.0), "grid": 50}

    # SESSION-57 CORNER GOAL COLOUR: a bbox-corner-tour goal (goal_is_corner=True) must draw a pure
    # BLUE star and no yellow; a frontier goal (False, or the key absent) must draw yellow and no blue.
    img_corner = np.zeros((size, size, 3), dtype=np.uint8)
    overlay_plan(img_corner, {"goal": (5.0, 5.0), "goal_is_corner": True}, m, size)
    case(f"(57-1) corner_goal_is_blue (blue={has_bgr(img_corner, (255, 0, 0))} "
         f"yellow={has_bgr(img_corner, (0, 255, 255))})",
         has_bgr(img_corner, (255, 0, 0)) and not has_bgr(img_corner, (0, 255, 255)))

    img_frontier = np.zeros((size, size, 3), dtype=np.uint8)
    overlay_plan(img_frontier, {"goal": (5.0, 5.0), "goal_is_corner": False}, m, size)
    case(f"(57-2) frontier_goal_is_yellow (yellow={has_bgr(img_frontier, (0, 255, 255))} "
         f"blue={has_bgr(img_frontier, (255, 0, 0))})",
         has_bgr(img_frontier, (0, 255, 255)) and not has_bgr(img_frontier, (255, 0, 0)))

    img_missing = np.zeros((size, size, 3), dtype=np.uint8)
    overlay_plan(img_missing, {"goal": (5.0, 5.0)}, m, size)
    case(f"(57-3) missing_flag_defaults_yellow (yellow={has_bgr(img_missing, (0, 255, 255))} "
         f"blue={has_bgr(img_missing, (255, 0, 0))})",
         has_bgr(img_missing, (0, 255, 255)) and not has_bgr(img_missing, (255, 0, 0)))

    # ---- SESSION-60 LKG PANEL: new leftmost column -- grey placeholder until a canvas has arrived,
    # then a fit-scaled (aspect-preserved, letterboxed, never cropped) render of whatever autopilot.py
    # published; the composed width always follows the new 4-panel formula in EITHER state. -----------
    dash60 = Dashboard()
    img_no_canvas = dash60.render()
    expected_w = CANVAS_W
    case(f"(60-1) no canvas yet -> composes without raising, width == PANEL_W+GAP+PANEL_W+GAP+MAP_SIZE "
         f"(got {img_no_canvas.shape[1]}, want {expected_w})",
         img_no_canvas.shape[1] == expected_w)

    dash60.lkg_canvas = np.full((288, 512, 3), 200, np.uint8)   # a stacked-looking canvas, unrelated size
    img_with_canvas = dash60.render()
    case(f"(60-2) canvas present -> composes without raising, SAME width as the no-canvas case "
         f"(got {img_with_canvas.shape[1]})",
         img_with_canvas.shape[1] == expected_w == img_no_canvas.shape[1])

    lkg_panel_placeholder = render_lkg_panel(None)
    lkg_panel_filled = render_lkg_panel(np.full((288, 512, 3), 200, np.uint8))
    case(f"(60-3) render_lkg_panel: both states return (MAP_SIZE, PANEL_W) regardless of input "
         f"(placeholder={lkg_panel_placeholder.shape} filled={lkg_panel_filled.shape})",
         lkg_panel_placeholder.shape == (MAP_SIZE, PANEL_W, 3) == lkg_panel_filled.shape)

    # ---- SESSION-61 LKG PANEL TEXT: the info block wraps between whole segments (never mid-field,
    # Finding D) drawn at PANEL resolution, and the panel greys out ONLY when the publisher truly
    # went silent (never when the plan is merely healthy -- operator's requirement). ------------------
    font61 = cv2.FONT_HERSHEY_SIMPLEX
    max_px61 = PANEL_W - 12

    segs61 = ["HOLD_LOST / PLAN-LOST", "src=slam:19385", "lkg_age=12.3s", "live=#20741",
             "matched=T", "inliers=42", "cont=F", "planar=T", "scale=1.23", "size=0.98",
             "closer=LIVE", "lines=17"]
    wrapped61 = _wrap_text_segments(segs61, max_px61, font61, LKG_TEXT_SCALE)
    all_fit61 = all(cv2.getTextSize(l, font61, LKG_TEXT_SCALE, 1)[0][0] <= max_px61 for l in wrapped61)
    joined61 = "  ".join(wrapped61)
    all_present61 = all(seg in joined61 for seg in segs61)
    empty_none61 = _wrap_text_segments(None, max_px61, font61, LKG_TEXT_SCALE) == []
    empty_list61 = _wrap_text_segments([], max_px61, font61, LKG_TEXT_SCALE) == []
    wrapped_long61 = _wrap_text_segments(["x" * 400], max_px61, font61, LKG_TEXT_SCALE)
    long_fits61 = (len(wrapped_long61) > 1
                  and all(cv2.getTextSize(l, font61, LKG_TEXT_SCALE, 1)[0][0] <= max_px61
                         for l in wrapped_long61))
    case(f"(61-1) _wrap_text_segments: every line fits max_px, no segment dropped, []/None safe, "
         f"a 400-char segment hard-splits (all_fit={all_fit61}, all_present={all_present61}, "
         f"empty_none={empty_none61}, empty_list={empty_list61}, long_fits={long_fits61})",
         all_fit61 and all_present61 and empty_none61 and empty_list61 and long_fits61)

    case(f"(61-2) realistic 12-segment banner_fields list wraps to <= LKG_TEXT_MAX_LINES lines "
         f"(n_lines={len(wrapped61)}, max={LKG_TEXT_MAX_LINES})",
         len(wrapped61) <= LKG_TEXT_MAX_LINES)

    canvas61 = np.full((576, 512, 3), 200, np.uint8)
    panel61 = render_lkg_panel(canvas61, info=segs61, age_s=0.1)
    has_image_px61 = bool(np.any(np.all(panel61 == 200, axis=-1)))
    case(f"(61-3a) render_lkg_panel(512x576 canvas, 12 segs, age_s=0.1) -> shape=(MAP_SIZE,PANEL_W,3), "
         f"image region non-empty (shape={panel61.shape}, has_image_pixels={has_image_px61})",
         panel61.shape == (MAP_SIZE, PANEL_W, 3) and has_image_px61)

    segs61_20 = segs61 + [f"extra{i}=value{i}" for i in range(8)]
    panel61b = render_lkg_panel(canvas61, info=segs61_20, age_s=0.1)
    has_image_px61b = bool(np.any(np.all(panel61b == 200, axis=-1)))
    case(f"(61-3b) 20-segment worst case -> shape=(MAP_SIZE,PANEL_W,3), image region non-empty "
         f"(shape={panel61b.shape}, has_image_pixels={has_image_px61b})",
         panel61b.shape == (MAP_SIZE, PANEL_W, 3) and has_image_px61b)

    panel_stale61 = render_lkg_panel(canvas61, info=segs61, age_s=LKG_CANVAS_STALE_S + 0.1)
    panel_fresh61 = render_lkg_panel(canvas61, info=segs61, age_s=0.1)
    differs61 = not np.array_equal(panel_stale61, panel_fresh61)
    stale_has_no_image61 = not bool(np.any(np.all(panel_stale61 == 200, axis=-1)))
    case(f"(61-4) age_s > LKG_CANVAS_STALE_S -> grey stale placeholder, differs from age_s=0.1 "
         f"(differs={differs61}, stale_has_no_image={stale_has_no_image61})",
         differs61 and stale_has_no_image61)

    panel_none61 = render_lkg_panel(None)
    case(f"(61-5) render_lkg_panel(None) -> waiting placeholder, shape=(MAP_SIZE,PANEL_W,3) "
         f"(shape={panel_none61.shape})",
         panel_none61.shape == (MAP_SIZE, PANEL_W, 3))

    expected_w61 = CANVAS_W
    expected_h61 = STATUS_H + MAP_SIZE
    dash61 = Dashboard()
    img_no_canvas61 = dash61.render()
    dash61.lkg_canvas = canvas61
    dash61.lkg_canvas_info = segs61
    dash61.lkg_canvas_t = time.monotonic()
    img_fresh61 = dash61.render()
    dash61.lkg_canvas_t = time.monotonic() - (LKG_CANVAS_STALE_S + 1.0)
    img_stale61 = dash61.render()
    expected_shape61 = (expected_h61, expected_w61, 3)
    case(f"(61-6) Dashboard.render() composes to the fixed size in all three states: "
         f"no-canvas/fresh/stale (shapes={img_no_canvas61.shape}, {img_fresh61.shape}, "
         f"{img_stale61.shape}, expected={expected_shape61})",
         img_no_canvas61.shape == expected_shape61 and img_fresh61.shape == expected_shape61
         and img_stale61.shape == expected_shape61)

    # (61-7) telemetry row (C9): spy cv2.putText to read back the EXACT string drawn, since pixels
    # alone aren't OCR-able -- same technique visual_recovery.py's (52-lkg-3) uses for its own banner.
    _put_text_calls61 = []
    _real_put_text61 = cv2.putText

    def _spy_put_text61(img, text, *rest, **kw):
        _put_text_calls61.append(text)
        return _real_put_text61(img, text, *rest, **kw)

    cv2.putText = _spy_put_text61
    try:
        render_telemetry_panel({"state": "SLAM_HOLD", "visrec_lkg": {"src": "slam:42", "age_s": 1.4}}, {})
        lkg_float_txt61 = next((t for t in _put_text_calls61 if t.startswith("LKG=")), None)
        _put_text_calls61.clear()
        render_telemetry_panel({"state": "SLAM_HOLD", "visrec_lkg": {"src": "none", "age_s": None}}, {})
        lkg_none_txt61 = next((t for t in _put_text_calls61 if t.startswith("LKG=")), None)
        _put_text_calls61.clear()
        render_telemetry_panel({"state": "SLAM_HOLD"}, {})
        lkg_missing_txt61 = next((t for t in _put_text_calls61 if t.startswith("LKG=")), None)
    finally:
        cv2.putText = _real_put_text61
    case(f"(61-7) telemetry row: 'age=<float>s' for a float, 'age=n/a' for None, 'LKG=--' for a "
         f"missing payload (float={lkg_float_txt61!r}, none={lkg_none_txt61!r}, "
         f"missing={lkg_missing_txt61!r})",
         lkg_float_txt61 == "LKG=slam:42 age=1.4s" and lkg_none_txt61 == "LKG=none age=n/a"
         and lkg_missing_txt61 == "LKG=--")

    # ---- SESSION-62 RECOVERY PANEL LINES -------------------------------------------------------
    # The panel's bottom two rows carried the session-52 notice/planner-event block, which the operator
    # judged useless in the one situation that matters most -- three FALLBACK episodes flown with no way
    # to tell the ladder's phases apart on screen. During a loss episode those rows now carry the
    # recovery FSM's own position. These cases pin that every phase renders something specific, that a
    # phase with no clock deadline is not drawn with an invented one, and (load-bearing) that the notice
    # block is only OUTRANKED, never removed.
    _rl = recovery_lines
    case("(62-1) recovery_lines(None) -> [] so the notice block still renders",
         _rl(None) == [] and _rl({}) == [])
    _phases = {
        "INITIAL_WAIT": {"elapsed_s": 12.4, "limit_s": 20.0},
        "BACKOFF": {"elapsed_s": 0.4, "limit_s": None},
        "BACKOFF_WAIT": {"elapsed_s": 3.2, "limit_s": 5.0},
        "TURN": {"cycle": 3, "cum_deg": 68},
        "PUSH": {"push_dirn": "left", "cycle": 3},
        "WAIT_POST": {"elapsed_s": 4.1, "limit_s": 10.0, "cycle": 3},
        "SERVO": {"servo_verdict": "EQUAL", "servo_frames": 2, "servo_hold_frames": 3},
    }
    _all_named = True
    for _ph, _extra in _phases.items():
        _out = _rl(dict({"phase": _ph, "loss_elapsed_s": 30.0}, **_extra))
        if not _out or _ph not in _out[0] and _ph not in ("SERVO",):
            _all_named = False
    case(f"(62-2) every FALLBACK phase renders a line naming itself ({len(_phases)} phases)", _all_named)
    case("(62-3) a phase with NO clock deadline shows elapsed bare, never an invented limit "
         "(BACKOFF: limit_s=None)",
         "/" not in _rl({"phase": "BACKOFF", "elapsed_s": 0.4, "limit_s": None})[0])
    case("(62-4) a phase WITH a deadline shows elapsed/limit",
         "3.2/5s" in _rl({"phase": "BACKOFF_WAIT", "elapsed_s": 3.2, "limit_s": 5.0})[0])
    case("(62-5) phase=None inside an episode is the pre-FALLBACK grace hold, not a blank",
         "LOSS GRACE" in _rl({"phase": None, "loss_elapsed_s": 7.2, "grace_s": 12.0})[0])
    _servo = {"phase": "SERVO", "servo_verdict": "EQUAL", "servo_frames": 2, "servo_hold_frames": 3}
    case("(62-6) SERVO distinguishes a live verdict from a lost match",
         "held 2/3" in _rl(_servo)[0]
         and "match lost" in _rl(dict(_servo, servo_lost_s=0.8, servo_lost_grace_s=1.5))[0])
    case("(62-7) never more than the 2 rows the panel has",
         all(len(_rl(dict({"phase": _ph, "loss_elapsed_s": 99.0}, **_ex))) <= 2
             for _ph, _ex in _phases.items()))
    _ctrl_rec = {"state": "FALLBACK", "recovery": {"phase": "TURN", "cycle": 1, "cum_deg": 22,
                                                   "loss_elapsed_s": 40.0},
                 "notice": {"kind": "TIMEOUT", "age_s": 3.0, "text": "something"}}
    _ctrl_no_rec = {"state": "ADVANCE", "recovery": None,
                    "notice": {"kind": "TIMEOUT", "age_s": 3.0, "text": "something"}}
    # Crop to the two NOTICE rows (y>=200) before colour-testing: the panel's own `PLAN STALE` row at
    # y=102 is drawn in the same orange, so a whole-panel test would be measuring that instead.
    _p_rec = render_telemetry_panel(_ctrl_rec, {})[200:, :]
    _p_notice = render_telemetry_panel(_ctrl_no_rec, {})[200:, :]
    case("(62-8) recovery OUTRANKS the notice block (cyan present, orange gone) and the notice "
         "block returns when the episode ends (orange back)",
         has_bgr(_p_rec, (255, 255, 0)) and not has_bgr(_p_rec, (0, 165, 255))
         and has_bgr(_p_notice, (0, 165, 255)))

    # ---- SESSION-62 PUSH MEASUREMENT ROW -------------------------------------------------------
    _pl = push_lines
    case("(62-9) push_lines(None) -> [] so the notice block still renders",
         _pl(None) == [] and _pl({}) == [])
    _pmoved = {"dir": "backward", "why": "timer", "traveled": 0.106, "drift": 0.400,
               "poses": 3, "verdict": "moved", "stuck_run": 0}
    _pstuck = {"dir": "backward", "why": "timer", "traveled": 0.034, "drift": 0.048,
               "poses": 4, "verdict": "stuck", "stuck_run": 3}
    _punk = {"dir": "backward", "why": "timer", "traveled": None, "drift": None,
             "poses": 1, "verdict": "unknown", "stuck_run": 0}
    case("(62-10) each verdict renders its own word, and unknown shows `--` rather than a fake 0.00",
         "moved" in _pl(_pmoved)[0] and "stuck" in _pl(_pstuck)[0]
         and "unknown" in _pl(_punk)[0] and "--" in _pl(_punk)[0])
    case("(62-11) a repeated stuck verdict shows its run count",
         "x3" in _pl(_pstuck)[0] and "x" not in _pl(_pmoved)[0].split("->")[-1])
    _c_push = {"state": "SETTLE", "recovery": None, "push": _pstuck,
               "notice": {"kind": "TIMEOUT", "age_s": 3.0, "text": "something"}}
    _c_rec = {"state": "FALLBACK", "push": _pstuck,
              "recovery": {"phase": "TURN", "cycle": 1, "cum_deg": 22, "loss_elapsed_s": 40.0}}
    _rows_push = render_telemetry_panel(_c_push, {})[200:, :]
    _rows_rec = render_telemetry_panel(_c_rec, {})[200:, :]
    case("(62-12) push row outranks the notice block, and an active loss episode outranks the push row",
         has_bgr(_rows_push, (0, 200, 255)) and not has_bgr(_rows_push, (0, 165, 255))
         and has_bgr(_rows_rec, (255, 255, 0)) and not has_bgr(_rows_rec, (0, 200, 255)))
    case("(62-13) `unknown` does NOT borrow the stuck alarm colour (absence of evidence is not alarm)",
         not has_bgr(render_telemetry_panel({"state": "SETTLE", "recovery": None, "push": _punk},
                                            {})[200:, :], (0, 200, 255)))

    # ---- SESSION-64 BACKEND STATUS: "" for pre-session-64 payloads (older flight/replay renders
    # unchanged); ASYNC/FAILED surfaced in the top strip; FAILED reads red, a nonzero clobber count
    # (expected + self-correcting, see C4) does NOT -- colouring it red would train the operator to
    # ignore red. ------------------------------------------------------------------------------------
    case("(64-1) backend_status_text({}) == \"\" and backend_status_text(None) == \"\" (older flight)",
         backend_status_text({}) == "" and backend_status_text(None) == "")

    _bk_async = backend_status_text({"backend_mode": "ASYNC", "backend_queue_depth": 2,
                                      "backend_pose_clobbers": 0})
    case(f"(64-2) ASYNC q2, no clobbers -> contains ASYNC and q2, no clob (got {_bk_async!r})",
         "ASYNC" in _bk_async and "q2" in _bk_async and "clob" not in _bk_async)

    _bk_clob = backend_status_text({"backend_mode": "ASYNC", "backend_queue_depth": 2,
                                     "backend_pose_clobbers": 7})
    case(f"(64-3) nonzero clobbers -> adds clob (got {_bk_clob!r})", "clob" in _bk_clob)

    _bk_failed = backend_status_text({"backend_mode": "FAILED", "backend_queue_depth": 5,
                                       "backend_pose_clobbers": 0})
    case(f"(64-4) FAILED -> contains FAILED (got {_bk_failed!r})", "FAILED" in _bk_failed)

    _pose64 = {"tracking_mode": "MASt3R", "mode": "TRACKING", "n_keyframes": 5, "n_voxels": 100,
               "slam_ms": 50.0}
    dash64 = Dashboard()
    dash64.pose = _pose64
    dash64.map = {"backend_mode": "FAILED", "backend_queue_depth": 5, "backend_pose_clobbers": 0}
    _strip_failed = dash64.render()[:STATUS_H, :]
    case(f"(64-5) FAILED status strip is red (has_red={has_bgr(_strip_failed, (0, 0, 255))})",
         has_bgr(_strip_failed, (0, 0, 255)))

    dash64.map = {"backend_mode": "ASYNC", "backend_queue_depth": 2, "backend_pose_clobbers": 7}
    _strip_async_clob = dash64.render()[:STATUS_H, :]
    case(f"(64-6) ASYNC+clobbers status strip has NO red (has_red="
         f"{has_bgr(_strip_async_clob, (0, 0, 255))})",
         not has_bgr(_strip_async_clob, (0, 0, 255)))

    # ---- SESSION-65 WINDOW STATUS: appended after the clob suffix; absent/"OFF" renders BYTE-
    # IDENTICAL to session 64's output (older flights + the still-default OFF mode); ON shows
    # W/solve_kf + anchors-if-any; SHADOW marks its W with "?" (measured, not applied); FAILED still
    # short-circuits first even when window keys are present. -------------------------------------
    case(f"(65-1) no backend_window_mode -> byte-identical to session 64 output (got {_bk_async!r})",
         _bk_async == "bk=ASYNC q2")
    _bk_off = backend_status_text({"backend_mode": "ASYNC", "backend_queue_depth": 2,
                                    "backend_pose_clobbers": 0, "backend_window_mode": "OFF"})
    case(f"(65-2) backend_window_mode=OFF -> byte-identical to session 64 output (got {_bk_off!r})",
         _bk_off == "bk=ASYNC q2")

    _bk_on = backend_status_text({"backend_mode": "ASYNC", "backend_queue_depth": 2,
                                   "backend_pose_clobbers": 0, "backend_window_mode": "ON",
                                   "backend_window_kf": 10, "backend_solve_kf": 12,
                                   "backend_anchors": 0})
    case(f"(65-3) ON W=10 solve_kf=12 anchors=0 -> ends with w10k12, no anchor suffix "
         f"(got {_bk_on!r})", _bk_on.endswith("w10k12"))

    _bk_on_anchors = backend_status_text({"backend_mode": "ASYNC", "backend_queue_depth": 2,
                                           "backend_pose_clobbers": 0, "backend_window_mode": "ON",
                                           "backend_window_kf": 10, "backend_solve_kf": 12,
                                           "backend_anchors": 2})
    case(f"(65-4) ON anchors=2 -> contains a2 (got {_bk_on_anchors!r})", "a2" in _bk_on_anchors)

    _bk_shadow = backend_status_text({"backend_mode": "ASYNC", "backend_queue_depth": 0,
                                       "backend_pose_clobbers": 0, "backend_window_mode": "SHADOW",
                                       "backend_window_kf": 10, "backend_solve_kf": 12})
    case(f"(65-5) SHADOW W=10 -> contains w?10, not w10k (got {_bk_shadow!r})",
         "w?10" in _bk_shadow and "w10k" not in _bk_shadow)

    _bk_failed_win = backend_status_text({"backend_mode": "FAILED", "backend_queue_depth": 5,
                                           "backend_pose_clobbers": 0, "backend_window_mode": "ON",
                                           "backend_window_kf": 10, "backend_solve_kf": 12,
                                           "backend_anchors": 3})
    case(f"(65-6) FAILED short-circuits even with window keys present (got {_bk_failed_win!r})",
         _bk_failed_win == "bk=FAILED")

    dash65 = Dashboard()
    dash65.pose = _pose64
    dash65.map = {"backend_mode": "ASYNC", "backend_queue_depth": 2, "backend_pose_clobbers": 0,
                  "backend_window_mode": "ON", "backend_window_kf": 10, "backend_solve_kf": 12,
                  "backend_anchors": 2}
    _strip_on = dash65.render()[:STATUS_H, :]
    case(f"(65-7) ON status strip has NO red (has_red={has_bgr(_strip_on, (0, 0, 255))})",
         not has_bgr(_strip_on, (0, 0, 255)))

    print(f"\n[self-test] {'ALL PASS' if ok else 'FAILURES PRESENT'}")
    return ok


def main():
    ap = argparse.ArgumentParser(description="Cartographer visualizer (P3): live map + telemetry dashboard")
    ap.add_argument("--self-test", action="store_true",
                    help="synthetic overlay_plan colour validation, no window/socket/hardware")
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
    if args.self_test:
        raise SystemExit(0 if run_self_test() else 1)
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
