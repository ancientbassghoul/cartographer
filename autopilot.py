"""autopilot.py — Process P5: autonomous flight controller (Phase-2 foundation).

Closes a minimal autonomy loop on OPTICAL-FLOW perception (no SLAM pose dependency): arm -> ascend
until the CEILING -> level the attitude -> push FORWARD until a WALL -> back off -> hold. It proves
programmatic control + live vision feedback and is the skeleton for the later Map (frontier-explore) /
Scan (stop, 360deg, fire the cascade) state machine.

Why flow, not SLAM pose: the previous SLAM-pose ceiling detector failed twice — monocular poses arrive
at ~1 Hz and slow further at a near surface, so a rate/plateau primitive never armed. Detection now
lives in `flow_contact_detector.FlowContactDetector` (self-calibrating optical-flow collapse, validated
on real footage). Control recipes live in `flight_playbook.json` (platform dynamics, as data).

Modes:
  --self-test : exercise the detection LOGIC (synthetic signal streams) + the playbook player. No hw.
  --dry-run   : SUB the frame bus, derive the held command from the frame meta `controls`, run the
                detector and LOG its verdict while the USER flies. Sends NO controls (validation only).
  (default)   : closed loop — PUB TOPIC_CONTROL to drive ARM -> ASCEND -> FORWARD -> BACK_OFF -> HOLD,
                using the flow detector to decide when to stop. Enable on io_bridge with 'm'; any
                manual flight key aborts. Needs only io_bridge running (frame bus + control apply);
                perception/SLAM is NOT run concurrently.

================================ HARD RULE ================================
NO MANUAL-FLIGHT DATA LEAKAGE (cartographer/CLAUDE.md "CRITICAL AUTONOMY STANDARD"). Every condition is
detected LIVE by the self-calibrating flow detector (relative ratios; see flow_contact_detector.py).
Playbook magnitudes are PLATFORM control dynamics (how the airframe responds), not this room's answer.
No constant here encodes a ceiling altitude / distance-to-wall / frame index.
==========================================================================

--log writes a rec_frame-prefixed text log + CSV to OUTPUT/diag/. rec_frame is io_bridge's
recording-relative video frame index, so a log line ties to the exact frame in OUTPUT/flight_<ts>.mp4.
"""

import argparse
import collections
import copy
import json
import math
import os
import time
from datetime import datetime, timedelta

import yaml

import frame_bus
from diag_log import DiagLog, NullLog
from flow_contact_detector import (FlowContactDetector, detector_from_cfg, FlowVerdict,
                                    CMD_UP, CMD_FWD, CMD_BACK, CMD_DOWN)
from flight_playbook import FlightPlaybook, RecipePlayer

REPO = os.path.dirname(os.path.abspath(__file__))


def load_config(path=None):
    path = path or os.path.join(REPO, "config.yaml")
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


# ==============================================================================
# Command inference (dry-run): which event are we testing for, from the pilot's controls?
# ==============================================================================
def _command_from_controls(controls: dict, ascend_cmd: int):
    """Map io_bridge's forwarded control snapshot to the detector command (UP precedence). None if the
    pilot is neither ascending nor pushing forward (detector idles). Returns the sentinel False if
    `controls` is absent so the caller can warn (NO SILENT FALLBACK)."""
    if not controls:
        return False
    if controls.get("joy_vertical") == ascend_cmd:
        return CMD_UP
    if float(controls.get("trigger", 0.0)) > 0.1:
        return CMD_FWD
    return None


# ==============================================================================
# Full control vector (sent EVERY tick; a dropped command simply goes stale and io_bridge zeroes it)
# ==============================================================================
_NEUTRAL = {
    "btnARMdown": False, "btnCdown": False, "trigger": 0.0, "reverse": 0.0,
    "trigger_down": False, "reverse_down": False,
    "joy_vertical": 0, "joy_horizontal": 0, "yaw": 0.0, "pitch": 0.0,
}


def _full_vector(active: dict, seq: int, now: float, state: str) -> dict:
    v = {"seq": seq, "mono_ts": now, "state": state}
    v.update(_NEUTRAL)
    v.update(active or {})
    # Unity gates REAL thrust on the triggerDown/reverseDown BOOLEANS, not the analog trigger/reverse
    # (session 17 discovery). Derive them here, at the single choke point every command flows through,
    # so EVERY forward/reverse emit site (presets, parallax pushes, back_off, rewind/fallback reverses,
    # postlude homing) engages thrust. ALWAYS set both (True or False) so the io_bridge overlay can't
    # leave a boolean stuck on from a previous tick.
    v["trigger_down"] = float(v.get("trigger", 0.0) or 0.0) > 0.0
    v["reverse_down"] = float(v.get("reverse", 0.0) or 0.0) > 0.0
    return v


# ==============================================================================
# Logging
# ==============================================================================
def _rec_prefix(rec_frame) -> str:
    return f"{int(rec_frame):07d}" if rec_frame is not None else "-------"


def _verdict_line(tag: str, v: FlowVerdict) -> str:
    sig = f"{v.signal:+.4f}" if v.signal is not None else "   -   "
    ratio = f"{v.ratio:.2f}" if v.ratio is not None else " -  "
    return (f"{tag} cmd={str(v.command):7s} signal={sig} ref={v.ref:6.3f} ratio={ratio} "
            f"airborne={int(v.airborne)} blank={int(v.blanking)} held={v.contact_held:.2f}s -> {v.label()}")


def _timeline_goals(plan: dict, leg_goal=None) -> list:
    """Goal markers for a replay step: the goal the CONTROLLER is committed to (`leg_goal`, tagged
    `active` — this is what "goal reached" is measured against) + perception's live frontier pick tagged
    `plan_pick` when it differs (perception re-picks ~2 Hz while the controller strong-commits) + each
    blacklisted point tagged `blacklist_soft`/`blacklist_permanent`. Zips plan['blacklist'] with
    plan['blacklist_permanent'] (the same arrays the visualizer rings) so the viewer can flip a goal
    gold->orange->red. The `active` marker is the committed leg_goal, NOT perception's async goal, so the
    marker no longer jumps to a goal the drone isn't flying to."""
    goals = []
    if leg_goal is not None:
        goals.append({"xz": [round(float(leg_goal[0]), 4), round(float(leg_goal[1]), 4)], "state": "active"})
    plan_goal = plan.get("goal")
    if plan_goal is not None and (leg_goal is None
                                  or abs(plan_goal[0] - leg_goal[0]) > 1e-6
                                  or abs(plan_goal[1] - leg_goal[1]) > 1e-6):
        goals.append({"xz": [round(float(plan_goal[0]), 4), round(float(plan_goal[1]), 4)],
                      "state": "plan_pick"})
    bl = plan.get("blacklist") or []
    perm = plan.get("blacklist_permanent") or []
    for i, pt in enumerate(bl):
        if pt is None:
            continue
        is_perm = bool(perm[i]) if i < len(perm) else False
        goals.append({"xz": [round(float(pt[0]), 4), round(float(pt[1]), 4)],
                      "state": "blacklist_permanent" if is_perm else "blacklist_soft"})
    return goals


def _timeline_step_record(t_wall, t_mono, rec_frame, state, event, status, plan: dict, cmd=None,
                          leg_goal=None, plan_age_s=None, alt=None) -> dict:
    """One structured replay record per explore step. Pose/heading/slam come straight off the plan payload
    (perception_worker._plan_payload, published ~2 Hz on a SLAM-paced pose), but the GOAL fields reflect
    what the CONTROLLER is actually doing: `goal` is the committed `leg_goal` (what "goal reached" is
    measured against), `plan_goal` is perception's async frontier pick, and `dist_to_goal` makes reach
    self-evident. `plan_age_s` + `frame_id` expose staleness — held-stale pose/heading (age grows,
    frame_id repeats) is why a real turn can look motionless in the log. `cmd` is the literal control dict
    sent to the sim this frame ({} = hover/neutral)."""
    g = plan.get
    pos = g("pos")
    dist_to_goal = None
    if pos is not None and leg_goal is not None:
        dist_to_goal = round(math.hypot(pos[0] - leg_goal[0], pos[1] - leg_goal[1]), 4)
    return {
        "t_wall": t_wall, "t_mono": round(float(t_mono), 3),
        "rec_frame": (int(rec_frame) if rec_frame is not None else None),
        "state": state, "event": event, "status": status,
        "pos": pos, "heading": g("heading_deg"), "pos_y": g("pos_y"),
        "slam_ms": g("slam_ms"), "fwd_clear": g("forward_clearance_dist"),
        # The four push-relevant clearance-ring reads (fwd/back/left/right, SLAM units or null=open near-field)
        # the parallax push actually saw — so a "no room" skip can be debugged directly instead of guessed.
        "ring_clear": ((lambda r: {"fwd": ExploreController._ring_get(r, 0.0),
                                   "back": ExploreController._ring_get(r, 180.0),
                                   "left": ExploreController._ring_get(r, -90.0),
                                   "right": ExploreController._ring_get(r, 90.0)})(g("clearance_ring"))
                       if g("clearance_ring") else None),
        # GOAL = the controller's committed leg_goal (acted-on); plan_goal = perception's async pick.
        "goal": ([round(float(leg_goal[0]), 4), round(float(leg_goal[1]), 4)] if leg_goal is not None else None),
        "plan_goal": g("goal"), "dist_to_goal": dist_to_goal, "plan_bearing_err": g("bearing_err"),
        # Staleness: age of the plan snapshot these pose/heading/slam values came from + its SLAM frame id.
        "plan_age_s": (round(float(plan_age_s), 2) if plan_age_s is not None else None),
        "frame_id": g("frame_id"), "cap_ts": g("cap_ts"),
        "goals": _timeline_goals(plan, leg_goal),
        # 2-bump blacklist observability: the live counter + the planner's transient bump-outcome event
        # (goal-change reset / blacklist), so the replay shows the mechanism the flight log used to hide.
        "wall_hit_count": g("wall_hit_count"), "wall_hit_goal": g("wall_hit_goal"),
        "planner_event": g("planner_event"),
        # Persistent goals DB snapshot (per-disc picks/strikes/blacklisted) -> the replay's floating table.
        "goal_db": g("goal_db"),
        # Raw command actually sent to the sim this frame (the joystick-bridge output) — pristine
        # per-frame telemetry so a crawl (forward trigger set but pose barely moving) is self-evident.
        # {} is preserved (hover/neutral); None only when no command was supplied (old logs omit the key).
        "cmd": (dict(cmd) if cmd is not None else None),
        # Debugger live HEIGHT group (session 21): the all-flight rolling drone-height MEDIAN (the baseline
        # CALIB_VERIFY judges against; updates every frame) + the three live calibration references
        # (ceiling/desired/delta, re-measured at every CALIB_VERIFY pass) + the TRIM/CALIB activity flags —
        # so a sag and its trigger threshold are self-evident while scrubbing the replay. `alt` is a dict
        # passed by run_explore; None on old logs -> the replay degrades cleanly.
        "alt_median": (alt or {}).get("median"),
        "alt_ceiling": (alt or {}).get("ceiling"),
        "alt_desired": (alt or {}).get("desired"),
        "alt_delta": (alt or {}).get("delta"),
        "trim_on": (alt or {}).get("trim_on"),
        "calib_on": (alt or {}).get("calib_on"),
    }


def _downsample_map(ground: dict, max_cells: int = 2500):
    """A compact copy of the GroundGrid summary for the replay JSONL: same WORLD bounds, but the flat
    row-major `cls` grid subsampled by an integer stride so rows*cols <= max_cells (keeps the JSONL small;
    the viewer only draws the newest map under the cursor). Returns None for an empty/degenerate grid."""
    if not ground or not ground.get("bounds"):
        return None
    rows, cols = int(ground.get("rows", 0)), int(ground.get("cols", 0))
    cls = ground.get("cls") or []
    if rows <= 0 or cols <= 0 or len(cls) < rows * cols:
        return None
    stride = 1
    while (math.ceil(rows / stride) * math.ceil(cols / stride)) > max_cells:
        stride += 1
    if stride == 1:
        return {"bounds": ground["bounds"], "rows": rows, "cols": cols, "cls": list(cls)}
    out = []
    for r in range(0, rows, stride):
        base = r * cols
        for c in range(0, cols, stride):
            out.append(cls[base + c])
    out_rows = len(range(0, rows, stride))
    out_cols = len(range(0, cols, stride))
    return {"bounds": ground["bounds"], "rows": out_rows, "cols": out_cols, "cls": out}


class AutopilotLog:
    """Optional `--log` sink: tees verdict lines to OUTPUT/diag/<ts>_autopilot.log, writes a structured
    verdict CSV (<ts>_autopilot.csv) AND a COMMAND CSV (<ts>_autopilot_cmd.csv) of every control vector
    the autopilot PUBLISHES (so arm/takeoff/turn are visible even though they emit no flow verdict).
    Disabled = no-op."""
    FIELDS = ["rec_frame", "frame_id", "mono_ts", "command", "kind", "signal", "ref", "ratio",
              "airborne", "blanking", "contact_held", "verdict"]
    CMD_FIELDS = ["rec_frame", "mono_ts", "seq", "step", "source", "fields"]

    def __init__(self, enabled: bool):
        self.enabled = enabled
        self._txt = None
        self._jsonl = None
        self.csv = NullLog()
        self.cmd_csv = NullLog()
        if enabled:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            d = os.path.join(REPO, "OUTPUT", "diag")
            os.makedirs(d, exist_ok=True)
            self.csv = DiagLog("autopilot", self.FIELDS, ts=ts)
            self.cmd_csv = DiagLog("autopilot_cmd", self.CMD_FIELDS, ts=ts)
            txt_path = os.path.join(d, f"{ts}_autopilot.log")
            self._txt = open(txt_path, "w", encoding="utf-8")
            # Structured replay timeline (F8): one JSON record per explore step (pose/goal/state/slam)
            # + a periodic map record — the machine-readable log I read instead of the giant text log,
            # and the data source for flight_replay.py's animated HTML.
            jsonl_path = os.path.join(d, f"{ts}_timeline.jsonl")
            self._jsonl = open(jsonl_path, "w", encoding="utf-8")
            print(f"[diag] autopilot text log -> {txt_path}", flush=True)
            print(f"[diag] flight replay timeline -> {jsonl_path}", flush=True)

    def line(self, text: str):
        if self._txt is not None:
            # Wall-clock stamp on every line so a flight log can be read back in real time
            # ("when did the drone wait for SLAM, and for how long").
            stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            self._txt.write(f"{stamp} {text}\n")
            self._txt.flush()

    def timeline(self, record: dict):
        """Write one JSON record (a replay timeline step) + flush. No-op when logging is disabled."""
        if self._jsonl is not None:
            self._jsonl.write(json.dumps(record) + "\n")
            self._jsonl.flush()

    def cmd(self, rec_frame, seq, step, source, fields):
        self.cmd_csv.row(rec_frame=("" if rec_frame is None else int(rec_frame)),
                         mono_ts=round(time.monotonic(), 4), seq=seq, step=step, source=source,
                         fields=json.dumps(fields, sort_keys=True))

    def row(self, rec_frame, meta: dict, v: FlowVerdict):
        self.csv.row(
            rec_frame=("" if rec_frame is None else int(rec_frame)),
            frame_id=meta.get("frame_id"), mono_ts=round(v.t, 4),
            command=v.command, kind=v.kind,
            signal=("" if v.signal is None else round(v.signal, 5)),
            ref=round(v.ref, 5), ratio=("" if v.ratio is None else round(v.ratio, 4)),
            airborne=int(v.airborne), blanking=int(v.blanking),
            contact_held=round(v.contact_held, 3), verdict=v.label(),
        )

    def close(self):
        if self._txt is not None:
            self._txt.close()
        if self._jsonl is not None:
            self._jsonl.close()
        self.csv.close()
        self.cmd_csv.close()


# ==============================================================================
# Dry-run: observe the frame bus + the pilot's commands; log the contact verdict (send NO controls)
# ==============================================================================
def run_dry(cfg, log=False, stop_event=None):
    ascend_cmd = int(cfg["autonomy"]["ascend_cmd"])
    detector = detector_from_cfg(cfg)
    frame_port = cfg["network"]["frame_bus_port"]
    sub = frame_bus.FrameSubscriber(frame_port)
    diag = AutopilotLog(log)
    print(f"[autopilot][dry-run] SUB frame bus :{frame_port}. Sending NO controls.")
    print("[autopilot][dry-run] Fly manually: hold UP to test CEILING, hold FORWARD to test WALL. "
          "A verdict only arms while that command is held. Press 'r' in io_bridge to record (log lines "
          "then carry the video frame index).\n")

    last_label = None
    last_log = 0.0
    warned = False
    try:
        while stop_event is None or not stop_event.is_set():
            msg = sub.recv(timeout_ms=1000)
            if msg is None:
                continue
            frame, meta = msg
            command = _command_from_controls(meta.get("controls"), ascend_cmd)
            if command is False:
                if not warned:
                    print("[autopilot][dry-run] WARNING: frame meta carries no 'controls' — cannot gate "
                          "on the pilot's command (restart io_bridge with the current code). Held quiet.",
                          flush=True)
                    warned = True
                command = None
            now = time.monotonic()
            v = detector.update(now, frame, command)
            rec_frame = meta.get("rec_frame")
            diag.row(rec_frame, meta, v)
            label = v.label()
            if label != last_label or (now - last_log) >= 0.5:
                line = f"{_rec_prefix(rec_frame)} {_verdict_line('[autopilot][dry-run]', v)}"
                print(line, flush=True)
                diag.line(line)
                last_label, last_log = label, now
    except KeyboardInterrupt:
        print("\n[autopilot][dry-run] stopped.")
    finally:
        diag.close()
        sub.close()


# ==============================================================================
# Mission: an editable JSON script of steps the autopilot flies in order.
# ==============================================================================
DEFAULT_MISSION = os.path.join(REPO, "mission_demo.json")
UNTIL_STEPS = {"ascend_until_ceiling", "forward_until_wall"}


def load_mission(path=None) -> dict:
    path = path or DEFAULT_MISSION
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _normalize_step(s, recipe_names):
    """Raw mission step -> typed dict. Fail-fast on anything unrecognized (NO silent skip)."""
    if isinstance(s, dict):
        if "rest" in s:
            return {"type": "rest", "seconds": float(s["rest"])}
        if "duration_s" in s:
            fields = {k: v for k, v in s.items() if k != "duration_s"}
            return {"type": "inline", "fields": fields, "seconds": float(s["duration_s"])}
        raise ValueError(f"bad mission step {s!r} (dict must have 'rest' or 'duration_s')")
    if s in UNTIL_STEPS:
        return {"type": "until", "name": s}
    if s in recipe_names:
        return {"type": "recipe", "name": s}
    raise ValueError(f"unknown mission step '{s}' — must be a playbook recipe {sorted(recipe_names)}, "
                     f"an until-keyword {sorted(UNTIL_STEPS)}, {{'rest': N}}, or {{'<field>':v, 'duration_s':N}}")


def expand_mission(mission: dict, pb: FlightPlaybook) -> list:
    """Normalize + validate steps, and auto-insert a `rest_between_s` settle between consecutive
    NON-rest steps (explicit rests are left as-is, no double-rest)."""
    recipe_names = set(pb.recipes.keys())
    norm = [_normalize_step(s, recipe_names) for s in mission.get("steps", [])]
    rest_between = float(mission.get("rest_between_s", 1.0))
    out = []
    for s in norm:
        if out and s["type"] != "rest" and out[-1]["type"] != "rest":
            out.append({"type": "rest", "seconds": rest_between})
        out.append(s)
    return out


def _command_from_vector(active: dict, ascend_cmd: int):
    """Which contact the detector should test for, derived from the control vector being PUBLISHED
    (so the airborne latch + refs build during takeoff/ascend/forward alike). UP precedence."""
    if active.get("joy_vertical", 0) == ascend_cmd:
        return CMD_UP
    if float(active.get("trigger", 0.0)) > 0.1:
        return CMD_FWD
    return None


def _step_source(step, phase):
    """Human-readable origin of the current command vector (user's scheme: recipe:<name> /
    preset:<name>). `step` is None while gated/waiting."""
    if step is None:
        return "wait"
    t = step["type"]
    if t == "rest":
        return "preset:hold"
    if t == "inline":
        return "inline"
    if t == "recipe":
        return f"recipe:{step['name']}"
    if phase == "reset":                      # forward_until_wall's attitude-reset prefix
        return "recipe:reset_attitude"
    return "preset:ascend" if step["name"] == "ascend_until_ceiling" else "preset:forward"


def run_mission(cfg, mission_path=None, max_contact_s=None, stop_event=None, log=False):
    ascend_cmd = int(cfg["autonomy"]["ascend_cmd"])
    detector = detector_from_cfg(cfg)
    pb = FlightPlaybook.load()
    reset_before_fwd = bool(pb.rule("reset_attitude_before_forward", True))
    presets = {k: pb.preset(k) for k in ("ascend", "forward", "hold")}

    mission = load_mission(mission_path)
    steps = expand_mission(mission, pb)
    mcs = float(max_contact_s if max_contact_s is not None else mission.get("max_contact_s", 0.0))

    frame_port = cfg["network"]["frame_bus_port"]
    ctrl_port = cfg["network"]["autonomy_control_port"]
    pub_dt = 0.05   # 20 Hz — inside io_bridge cmd_timeout so commands stay "fresh"

    pub = frame_bus.StatePublisher(ctrl_port)
    sub = frame_bus.FrameSubscriber(frame_port)
    diag = AutopilotLog(log)

    print(f"[autopilot] MISSION '{os.path.basename(mission_path or DEFAULT_MISSION)}' "
          f"({len(steps)} steps incl. auto-rests). PUB TOPIC_CONTROL :{ctrl_port} | SUB frame bus :{frame_port}")
    print("[autopilot] On io_bridge press 'm' to hand control over; any flight key aborts.")
    for i, s in enumerate(steps):
        extra = f" {s['seconds']}s" if s["type"] in ("rest", "inline") else ""
        print(f"   {i+1:>2}. {s.get('name', s['type'])}{extra}")
    if mcs > 0:
        print(f"[autopilot] SAFETY: an until-contact step aborts the MISSION to HOLD after {mcs}s "
              f"(reported as a non-detection, never a contact).")

    seq = 0
    idx = 0
    entered = False
    player = None
    reset_player = None
    phase = None              # for 'until' steps: 'reset' (attitude) then 'contact'
    phase_t0 = 0.0
    last_pub = last_log = 0.0
    last_label = None
    aborted = False
    # Autonomy gate: the mission must NOT start until the operator has enabled autonomy on io_bridge
    # ('m'), else the early steps (arm/takeoff) elapse before io_bridge applies anything and the drone
    # never arms. io_bridge stamps its status into every frame meta (controls.autonomy); we hold at the
    # current step while it's MANUAL and run only while it's AUTO / AUTO(STALE).
    enabled = False
    was_enabled = False
    announced_wait = False
    warned_no_auto = False
    last_cmd_key = None
    last_rec_frame = None

    def log_cmd(active: dict, source: str, step_name: str):
        """Log every PUBLISHED command vector on change (source + fields) — so arm/takeoff/turn are
        visible even though they emit no flow verdict (the user's request)."""
        nonlocal last_cmd_key
        key = (source, json.dumps(active, sort_keys=True))
        if key == last_cmd_key:
            return
        last_cmd_key = key
        line = (f"{_rec_prefix(last_rec_frame)} [autopilot][CMD] step={step_name} src={source} "
                f"fields={json.dumps(active, sort_keys=True)}")
        print(line, flush=True)
        diag.line(line)
        diag.cmd(last_rec_frame, seq, step_name, source, active)

    def emit(v: FlowVerdict, rec_frame, meta, tag):
        nonlocal last_log, last_label
        diag.row(rec_frame, meta, v)
        n = time.monotonic()
        if v.label() != last_label or (n - last_log) >= 0.5:
            line = f"{_rec_prefix(rec_frame)} {_verdict_line(tag, v)}"
            print(line, flush=True)
            diag.line(line)
            last_log, last_label = n, v.label()

    try:
        while stop_event is None or not stop_event.is_set():
            now = time.monotonic()
            msg = sub.recv(timeout_ms=20)
            frame = meta = None
            if msg is not None:
                frame, meta = msg
                if meta.get("rec_frame") is not None:
                    last_rec_frame = meta.get("rec_frame")

            # ---- autonomy gate: only run the mission while io_bridge reports autonomy ON ----
            if meta is not None:
                st = (meta.get("controls") or {}).get("autonomy")
                if st is None:
                    if not warned_no_auto:
                        print("[autopilot] WARNING: frame meta has no controls.autonomy — cannot tell if "
                              "autonomy is enabled; HOLDING. Restart io_bridge with the current code.", flush=True)
                        warned_no_auto = True
                    enabled = False
                else:
                    enabled = (st != "MANUAL")   # AUTO or AUTO(STALE) both mean the operator handed over
            if not enabled:
                # Wait (or pause) — hold neutral so io_bridge has a fresh command to apply when 'm' is
                # pressed, and restart the current step cleanly on resume. Keep prev_gray fresh.
                if was_enabled:
                    print("[autopilot] autonomy OFF -> mission PAUSED (press 'm' to resume).", flush=True)
                elif not announced_wait:
                    print("[autopilot] waiting for autonomy enable ('m' on the io_bridge window) ...", flush=True)
                    announced_wait = True
                was_enabled = False
                entered = False
                if frame is not None:
                    detector.update(now, frame, None)
                if (now - last_pub) >= pub_dt:
                    log_cmd({}, "wait", "(wait)")
                    pub.publish(frame_bus.TOPIC_CONTROL, _full_vector({}, seq, now, "WAIT"))
                    seq += 1
                    last_pub = now
                continue
            if not was_enabled:
                print(f"[autopilot] autonomy LIVE -> running mission from step {idx+1}/{len(steps)}.", flush=True)
                was_enabled = True

            advance = False
            # ---- determine the active control vector for the current step ----
            if idx >= len(steps):
                active = presets["hold"]
                cur_name = "DONE"
            else:
                step = steps[idx]
                cur_name = step.get("name", step["type"])
                if not entered:
                    entered, phase_t0 = True, now
                    if step["type"] == "recipe":
                        player = pb.player(step["name"])
                    elif step["type"] == "until":
                        phase = "reset" if (step["name"] == "forward_until_wall" and reset_before_fwd) else "contact"
                        reset_player = pb.player("reset_attitude") if phase == "reset" else None
                    extra = f" ({step['seconds']}s)" if step["type"] in ("rest", "inline") else ""
                    print(f"[autopilot] step {idx+1}/{len(steps)}: {cur_name}{extra}", flush=True)

                if step["type"] == "rest":
                    active = presets["hold"]
                    if now - phase_t0 >= step["seconds"]:
                        advance = True
                elif step["type"] == "inline":
                    active = step["fields"]
                    if now - phase_t0 >= step["seconds"]:
                        advance = True
                elif step["type"] == "recipe":
                    active, done = player.fields(now)
                    if done:
                        advance = True
                else:  # until
                    if phase == "reset":
                        active, rdone = reset_player.fields(now)
                        if rdone:
                            phase, phase_t0 = "contact", now
                    else:
                        active = presets["ascend"] if step["name"] == "ascend_until_ceiling" else presets["forward"]

            # ---- feed the detector EVERY frame (command from the published vector) ----
            if frame is not None:
                command = _command_from_vector(active, ascend_cmd)
                v = detector.update(now, frame, command)
                if command in (CMD_UP, CMD_FWD):
                    emit(v, meta.get("rec_frame"), meta, f"[autopilot][{cur_name}]")
                    if idx < len(steps) and steps[idx]["type"] == "until" and phase == "contact":
                        expected = "CEILING" if steps[idx]["name"] == "ascend_until_ceiling" else "WALL"
                        if v.contact and v.kind == expected:
                            print(f"[autopilot] *** {expected} contact -> step done ***", flush=True)
                            advance = True

            # ---- until-contact SAFETY timeout -> abort the whole mission to HOLD ----
            if (not advance and mcs > 0 and idx < len(steps) and steps[idx]["type"] == "until"
                    and phase == "contact" and (now - phase_t0) >= mcs):
                exp = "CEILING" if steps[idx]["name"] == "ascend_until_ceiling" else "WALL"
                print(f"[autopilot] !! SAFETY: no {exp} within {mcs}s -> ABORT mission to HOLD "
                      f"(non-detection, NOT a contact).", flush=True)
                aborted, idx, entered = True, len(steps), False

            # ---- publish the full vector (logging every command on change: src=recipe/preset) ----
            if (now - last_pub) >= pub_dt:
                if idx >= len(steps):
                    source, step_name = "preset:hold", "DONE"
                else:
                    source, step_name = _step_source(steps[idx], phase), cur_name
                log_cmd(active, source, step_name)
                pub.publish(frame_bus.TOPIC_CONTROL,
                            _full_vector(active, seq, now, ("DONE" if idx >= len(steps) else cur_name)))
                seq += 1
                last_pub = now

            # ---- advance to the next step ----
            if advance:
                idx += 1
                entered, player, reset_player, phase = False, None, None, None
                if idx >= len(steps) and not aborted:
                    print("[autopilot] mission complete -> HOLD.", flush=True)
    except KeyboardInterrupt:
        print("\n[autopilot] interrupted — sending a final HOLD (neutral).")
    finally:
        pub.publish(frame_bus.TOPIC_CONTROL, _full_vector({}, seq, time.monotonic(), "HOLD"))
        time.sleep(0.05)
        diag.close()
        pub.close()
        sub.close()


# ==============================================================================
# Map mode (--explore): execute the frontier plan published by perception_worker on TOPIC_PLAN.
#
# The planner (perception_worker) owns the map + frontier selection; the autopilot is the pure
# EXECUTOR. Per leg: ORIENT (gentle closed-loop bus yaw to the goal bearing) -> RESET attitude ->
# ADVANCE (forward until the flow WALL detector fires, the goal is reached, or a leg timeout) ->
# BACK_OFF -> SETTLE -> REPLAN. Done when the plan reports no frontiers remain.
#
# The decision LOGIC lives in `ExploreController` (pure, no I/O) so it is unit-testable with synthetic
# plan/flow streams; `run_explore` is the thin bus wrapper (frames + TOPIC_PLAN in, TOPIC_CONTROL out).
# ==============================================================================
def _plan_status(last_plan, plan_age, plan_timeout_s):
    """Classify the freshest plan (caller acts on it). NO SILENT FALLBACK — a missing/old/invalid plan
    is an explicit non-OK state that holds the drone, never a coast on the last good goal.
      NO-PLAN     : nothing received yet.
      PLAN-LOST   : no plan within plan_timeout_s (perception likely dead) — the reviewer's case.
      PLAN-STALE  : plan present but SLAM not TRACKING (plan_valid=false).
      OK          : a fresh, valid plan."""
    if last_plan is None:
        return "NO-PLAN"
    if plan_age > plan_timeout_s:
        return "PLAN-LOST"
    if not last_plan.get("plan_valid"):
        return "PLAN-STALE"
    return "OK"


class ExploreController:
    """Pure per-leg state machine for frontier exploration. `step(now, plan, wall_contact)` is called
    only with a fresh, valid plan; it returns (active_fields, state, event). The caller handles the
    autonomy gate + degraded plan states and calls `reset_leg()` whenever it interrupts the machine.

    A one-time PRELUDE (ARM -> TAKEOFF) runs first — flying the SAME `arm`/`takeoff` playbook recipes
    the mission uses — so `--explore` is fully autonomous from a grounded, disarmed drone. `no_takeoff`
    skips it (manual handover / already airborne)."""

    def __init__(self, cfg, no_takeoff=False):
        e = (cfg["autonomy"].get("explore") or {})
        self.leg_max_s = float(e.get("leg_max_s", 20.0))
        self.goal_reach_dist = float(e.get("goal_reach_dist", 0.4))
        self.pb = FlightPlaybook.load()
        # The neutral settle inserted BETWEEN composed maneuvers (recovery: back off -> settle -> rotate
        # -> settle -> decide) and between explore legs — tunable in flight_playbook.json (rules.rest_between_s).
        self.rest_between_s = float(self.pb.rule("rest_between_s", 1.0))
        self.forward_preset = self.pb.preset("forward")
        # Forward throttle override (config): slow the approach so SLAM maps a wall before the drone reaches
        # it and the clearance stop can fire (a fast push raced into a wall before it was mapped -> SLAM died).
        # Applies to BOTH the ADVANCE leg and the forward parallax push (both use forward_preset).
        ft = e.get("forward_throttle", None)
        if ft is not None:
            self.forward_preset = dict(self.forward_preset, trigger=float(ft))
        # HOP cadence, NO goal commitment (session 20 rev): ADVANCE flies hop_duration_s SECONDS, SETTLEs (a
        # fresh-frame SLAM breather), then REPLANs — re-reading SLAM's CURRENT goal. If SLAM re-picked a
        # different goal while the drone advanced, the drone adopts the NEW goal (re-orient WITH the parallax
        # scout -> hop), instead of resuming the old, unreached leg_goal. So the drone never hardens its life by
        # committing to one distant goal; SLAM stays free to re-pick, and the goals-DB guards (frontier_planner)
        # retire ping-pong + stalls. 0 = disabled (cruise straight to the goal).
        # TIME-based, not a tick count (20260719 investigation): the controller loop's own tick rate is an
        # emergent property of I/O timing (frame arrival, detector load), not a config-locked rate -- measured
        # 30-35Hz across real flights, never the 20Hz `pub_dt` (a different, unrelated publish-rate throttle)
        # the operator expected. A raw tick count (the old `hop_ticks`) let the real hop duration drift with
        # whatever the loop's rate happened to be; hop_duration_s fixes the REAL elapsed time instead.
        self.hop_duration_s = float(e.get("hop_duration_s", 0.0))
        self._hop_tick = 0                   # diagnostic-only tick counter (reset on each ADVANCE entry) --
        #                                      no longer gates anything, just reports how many loop iterations
        #                                      a hop actually took (useful given the rate isn't fixed)
        # Per-HOP progress (session 20b): a stall is a MEASURED CONSEQUENCE of a hop that failed to get closer,
        # never a precondition that blocks ADVANCE. On ADVANCE entry we snapshot the distance to the goal; when
        # the hop ends and we REPLAN we compare — closed >= hop_progress_eps == progress (reset that goal's
        # strikes), else a STRIKE toward that goal in the planner's goals-DB (2 strikes -> blacklist). A mid-hop
        # plan-loss INVALIDATES the pending eval (_hop_start_goal cleared) so the interrupted hop isn't a strike.
        self._hop_start_dist = None          # dist(pos, leg_goal) snapshotted at the current hop's ADVANCE entry
        self._hop_start_goal = None          # the goal that snapshot was against (None = no pending hop to judge)
        self.hop_progress_eps = float(e.get("hop_progress_eps", e.get("goal_progress_eps", 0.2)))  # min closing
        #                                    distance (SLAM units) over a hop that counts as MEANINGFUL advancement
        # Mirrors frontier_planner.py's own disc-matching radius (same config key, same default) -- used ONLY to
        # decide "is this leg-goal the SAME goals-DB disc as last REPLAN" for the pick-dedup below. Bug fix: this
        # decision previously reused `calib_goal_change_dist` (a height-recalibration knob, deliberately coarser),
        # so a goal picked 0.5-1.0u from the last one (a genuinely different disc, per goal_area_radius) was
        # wrongly treated as "not a new pick" -- register_goal_pick never ran for it (picks stuck at 0 forever)
        # even though its hops still accrued real strikes via register_hop_outcome (confirmed on 20260719_005402:
        # goal [3.27, 6.58], 0.69u from the prior goal, showed picks=0/strikes=1 the first time it appeared).
        self.goal_area_radius = float(e.get("goal_area_radius", 0.5))
        self._leg_is_corner = False          # True while the committed leg_goal is a sweep-tour CORNER (from the
        #                                    plan's goal_is_corner) — gates the far-corner bump/strike suppression
        self.corner_no_blacklist_dist = float(e.get("corner_no_blacklist_dist", 1.0))  # a sweep CORNER goal
        #                                    farther than this from the drone CANNOT be bumped/struck/blacklisted:
        #                                    a mildly-stuck drone must never retire a far corner it hasn't reached
        # Session 24: the far-corner exemption above is not infinite -- track how many times we've been about
        # to give up on EACH corner (a would-have-bumped decision suppressed by the guard above). PERSISTS the
        # whole flight, keyed by proximity (like the goals-DB), so oscillating between two unreachable corners
        # can't defeat the cap by resetting a single tracked slot. At corner_giveup_limit, force-retire that
        # corner (never blacklist/end the mission by itself -- see force_retire_corner + the REPLAN done branch).
        self.corner_giveup_limit = int(e.get("corner_giveup_limit", 10))
        self._corner_giveup_counts = []       # [{"goal":[x,z], "count":int}] -- ALL tracked corners, never reset
        self._corner_giveup_pulse = None      # stashed [x,z] corner for run_explore to publish (mirrors _bump_pulse)
        self._corner_giveup_stuck = False     # True once REPLAN routes a give-up-exhausted mission into STUCK
        #                                    (gates STUCK's own resume check -- this hold must NOT auto-exit)
        self._pick_pulse = None              # stashed {pick_goal,pick_pos,prev_goal,prev_progressed,
        #                                    prev_strike_eligible} for run_explore to publish to perception (the
        #                                    goals-DB pick + previous-hop strike/progress outcome), mirror of _bump_pulse
        # Reverse throttle override (config): gentler BACKWARD speed for every reverse maneuver (back_off,
        # reverse_probe, recovery back-off, backward parallax push), so a fast backward ram into a wall can't
        # throw the drone to SLAM-killing angles. Reverse is a continuous 0-1 throttle like forward; we rewrite
        # the reverse magnitude in the loaded playbook recipes (durations unchanged), and the backward parallax
        # push reads back_off so it inherits it too.
        rt = e.get("reverse_throttle", None)
        if rt is not None:
            for steps in self.pb.recipes.values():
                for step in steps:
                    if "reverse" in step:
                        step["reverse"] = float(rt)
        self.reverse_throttle = float(rt) if rt is not None else 0.7   # magnitude for the fallback retreat
        self.ascend_preset = self.pb.preset("ascend")     # {"joy_vertical": -1}
        self.reset_before_fwd = bool(self.pb.rule("reset_attitude_before_forward", True))
        # Prelude ceiling phase: after takeoff, ascend until the flow CEILING fires, then drop a bit, so
        # mapping happens at a consistent height near the ceiling (the user's requested behavior).
        self.ascend_to_ceiling = bool(e.get("ascend_to_ceiling", True))
        self.ascend_max_s = float(e.get("ascend_max_s", 15.0))
        # The post-ceiling descent is a PLAYBOOK recipe ("descend") so its key-press duration is tunable
        # in flight_playbook.json (the user's request), not a config constant.
        # --- SLAM-loss recovery (CONTROL-SPACE, not state-space: pose is invalid during a tracking loss) ---
        # PLAN-LOST (perception silent) -> HARD HOVER-HOLD indefinitely (no blind recovery on a clock).
        # PLAN-STALE (perception publishing, SLAM not TRACKING) -> RECOVERY_REWIND: replay the INVERSE of the
        # recently-flown maneuvers to re-expose the camera to keyframes it already recorded, watching for OK.
        # If the history is empty/exhausted (e.g. a wall hit cleared it) -> parallax + <=45deg fallback.
        self.command_history = collections.deque(maxlen=100)   # maneuvers flown during normal exploration
        self.command_history_s = float(e.get("command_history_s", 12.0))  # rewind horizon (seconds of motion)
        self.fallback_retreat_s = float(e.get("fallback_retreat_s", 0.5))  # retreat duration per fallback attempt
        self.fallback_max_attempts = int(e.get("fallback_max_attempts", 16))
        self._fallback_attempts = 0
        # --- SESSION 12 recovery redesign (see plans/strafe-throttle-and-recovery-loop.md D5) ---
        # A flickering SLAM status (PLAN-LOST<->PLAN-STALE) used to RESET recovery every ~3s, so STUCK was
        # unreachable and the rewind never emptied (flight 20260713 frantic loop). Fix: `_recovering` PERSISTS
        # across the flicker; the rewind CONSUMES command_history one maneuver at a time; the give-up counter is
        # reset ONLY by a confirming ADVANCE (>= recovery_confirm_dist of real forward progress), never by a bare
        # OK. And a re-locked drone is NOT trusted until it flies that ADVANCE: while `_recovering`, appends to
        # command_history are frozen and moving post-relock sets `_history_broken` so the now spatially-stale
        # leftover history is cleared + bypassed straight to FALLBACK (no displaced "ghost path" replay).
        self.recovery_confirm_dist = float(e.get("recovery_confirm_dist", 1.0))  # >=this ADVANCE progress = trust restored
        self.recovery_turn_step_deg = float(e.get("recovery_turn_step_deg", 15.0))  # gentler sweep step in recovery
        # A settle BETWEEN every recovery action (REWIND inverse maneuvers + spin FALLBACK attempts): back-to-back
        # commands never gave monocular SLAM a still moment to re-lock (the "firing/spinning with no settles"
        # operator report). Lost-SLAM flavor of the shared settle gate — fresh CAPTURE verified, but bounded by
        # recovery_settle_max_s so a dead pipeline still proceeds to the next re-exposure maneuver.
        self.recovery_settle_frames = int(e.get("recovery_settle_frames", 4))   # fresh post-hold frames to end a recovery settle
        self.recovery_settle_max_s = float(e.get("recovery_settle_max_s", 2.5))  # bounded escape if the pipeline is dead
        self._rec_settling = False        # in a between-action settle hold inside REWIND/FALLBACK
        # The fallback turn is ALWAYS +turn_step_deg (a UNIDIRECTIONAL +45deg sweep: N attempts systematically
        # re-expose every past heading for RELOC, vs the old +/- wiggle that just oscillated in place). The
        # RETREAT direction is what alternates fwd/back (seeded on attempt 0 by the roomier body axis).
        self._fallback_retreat_forward = None   # seeded on the first fallback attempt from the last-known ring
        self._last_ring = None           # last non-None clearance ring (for fallback direction choice while STALE)
        self._leg_theta = 0.0            # theta of the current ORIENT turn (logged into command_history when flown)
        self._explore_started = bool(no_takeoff)   # recovery only after the prelude (True immediately if no_takeoff)
        self._ever_tracked = False        # SLAM has produced >=1 valid TRACKING plan in explore (gates the startup no-spin)
        # --- SLAM frame-timing settle gate ---
        # A healthy MASt3R-SLAM solve on this GPU builds a frame in well under a second; a choke (esp. right
        # after a turn) spikes it and the pose it emits is unreliable -> the drone flew on a bad heading. So:
        # while translating (or right after a turn / on recovering) HOLD until SLAM is "stable" = >N consecutive
        # FRESH frames each built in < slam_slow_ms. The threshold is a COMPUTE characteristic (tunable),
        # NOT this room's geometry. slam_ms + frame_id ride on TOPIC_PLAN.
        self.slam_slow_ms = float(e.get("slam_slow_ms", 1000.0))
        # SETTLE fresh-frame gate (session 15): a goal-flying settle (nxt REPLAN/REVERSE_PROBE) must wait for
        # this many SLAM "done" frames CAPTURED AFTER the settle started (cap_ts >= entry) AND under slam_slow_ms
        # -> no flying command on a stale pose. The vertical prelude/calib routine is exempt (kept timed).
        self.settle_fresh_frames = int(e.get("settle_fresh_frames", 6))
        self._settle_t0 = None            # SETTLE entry time (monotonic); frames CAPTURED >= this count toward the gate
        self._settle_ok = 0               # fresh fast post-entry frames counted this SETTLE
        self._settle_last_fid = None      # last frame_id evaluated this SETTLE (dedup on the republish timer)
        # Session 24: SLAM_HOLD -> SETTLE two-gate primitive (replaces the old slam_settle_frames/_slam_stable
        # single-counter check for THIS pathway only -- calibration-recovery holds below keep _slam_fast_streak).
        # A rolling (slam_ms, cap_ts) window fed on EVERY fresh frame decouples two questions that a single
        # integer streak conflated: is SLAM's SOLVE currently healthy (reusable instantly if already true) vs
        # has the airframe had enough REAL time to stop drifting since it last moved (depends on WHERE the wait
        # started). See _slam_window_ready / _settle_gate_begin / _settle_gate_poll.
        self._slam_hist = collections.deque(maxlen=self.settle_fresh_frames)   # rolling (slam_ms, cap_ts)
        self.settle_gate_s = float(e.get("settle_gate_s", self.rest_between_s))  # min PHYSICAL dwell post-motion
        self._settle_gate_t0 = None            # wall time the current gate window's clock started
        self._settle_gate_prequalified = False  # was the rolling window ALREADY clean the instant it opened?
        self._slam_fast_streak = 0        # consecutive FRESH frames under the slow threshold (calibration-recovery holds only, post-session-24)
        self._slam_slow_streak = 0        # consecutive FRESH frames AT/OVER it (arms a rewind step-back)
        self._slam_ms_latest = None       # last FRESH frame's build time (ms)
        self._slam_frame_id = None        # frame_id of that last-counted frame (dedup; plan republishes on a timer)
        self._slam_resume = None          # state SLAM_HOLD re-enters once SLAM settles
        # SLAM-settle REWIND step-back: while SLAM stays slow in a HOLD and the plan is still OK (NOT
        # lost/stale — those keep their own recovery), stepping one entry back through the rewind queue
        # re-exposes known-good geometry to help the solve re-lock (the user's "back up until it settles"
        # heuristic). Re-arm needs another full run of slow frames; capped per hold. Platform params.
        self.slam_stepback_after_frames = int(e.get("slam_stepback_after_frames", 10))
        self.slam_stepback_max_steps = int(e.get("slam_stepback_max_steps", 3))
        # PERSISTS across a PLAN-LOST/HOLD_LOST bounce within one bad SLAM patch (mirrors the
        # `_recovering`/`_fallback_attempts` persistence rule) -- a solve slow enough to trip this almost
        # always exceeds `plan_timeout_s` before it finishes, so the FSM bounces HOLD_LOST -> OK -> a FRESH
        # SLAM_HOLD every time; resetting this on every fresh hold entry (the old behavior) meant the
        # escalation could never reach its cap in exactly the scenario it exists to bound (confirmed on the
        # 20260718 flight: #1/3 fired 3x running, `plan_valid` bounced between, never reaching #2 or #3).
        # Reset ONLY on a genuinely trusted recovery (REPLAN / confirming ADVANCE) or a materially NEW leg
        # goal (both in the REPLAN handler) -- NOT on every `_enter_slam_hold`.
        self._slam_stepback_count = 0
        self._slam_hold_start = None      # 'now' when the current SLAM_HOLD began (total-wait logging)
        # Reactive wall/backwall response while BLIND (HOLD_LOST / waiting in SLAM_HOLD): the flow contact
        # detector doesn't need SLAM, but nothing read it in those states before this fix (the drone could
        # drift into a wall for 30-40s of a bad SLAM patch with no reaction). Edge-triggered (armed=True
        # means "ready to react to a fresh contact"; disarmed once reacted, re-arms only once contact
        # clears) so a sustained pin doesn't replay back_off every tick.
        self._blind_contact_armed = True
        self._blind_backoff_resume = None  # the hold state ("HOLD_LOST"/"SLAM_HOLD") to resume after it plays
        # Yaw is "fly toward your aim": a SUSTAINED hold (then 'c' reset) rotates the body; the turn ANGLE
        # is set by the hold DURATION, not a steerable rate (pulses do nothing; SLAM under-tracks rotation
        # so no in-turn closed loop). Turn OPEN-LOOP in quantized steps using the user's calibrated turn
        # recipe, scaling its yaw-hold; the per-leg re-plan after each ADVANCE is the outer correction.
        self.turn_step_deg = float(e.get("turn_step_deg", 45.0))      # quantize each aim change to this
        self.turn_recipe_deg = float(e.get("turn_recipe_deg", 90.0))  # angle the playbook turn recipe produces
        # EXPERIMENT (reverse-probe): big open-loop turns break MASt3R-SLAM (RELOC freezes the pose) while
        # straight translation should keep it TRACKING. When ON: clamp each leg's turn to ONE step (SLAM
        # still alive at the wall) and, on a WALL hit, fly straight BACKWARD (camera still facing the wall)
        # instead of the tiny back-off, to test whether reverse keeps SLAM alive. The reverse DURATION is the
        # "reverse_probe" recipe knob in flight_playbook.json. See config.yaml autonomy.explore.
        self.reverse_probe_on_wall = bool(e.get("reverse_probe_on_wall", False))
        # The ≤45° leg-turn clamp proved live to keep SLAM TRACKING through turns; keep it under its OWN
        # flag (not coupled to the reverse experiment) so disabling reverse_probe never silently drops it.
        self.clamp_leg_turn = bool(e.get("clamp_leg_turn", True))
        # Forward stand-off: stop the ADVANCE leg when the raycast clearance ahead (TOPIC_PLAN
        # forward_clearance_dist, SLAM units) drops below this, BEFORE ramming a wall freezes the image
        # and kills SLAM. Primary forward stop; the flow wall_contact stays as the glass/unmapped fallback.
        self.stop_on_clearance = bool(e.get("stop_on_clearance", True))
        self.stop_clearance_dist = float(e.get("stop_clearance_dist", 0.6))
        # On a clearance stand-off stop, play the small back_off recipe before settling. Its reverse pulse
        # re-arms the 2-bump latch (rearm_bump_if_disengaged fires on a backward command), so a wall the
        # drone gets pinned against by the stand-off can still accrue a SECOND bump and be blacklisted —
        # otherwise the tight REPLAN->ORIENT(0)->ADVANCE->standoff loop never reverses/displaces and the
        # counter is stuck at 1 (Bug B). Also seeds SLAM parallax. Set False to restore the direct settle.
        self.backoff_on_standoff = bool(e.get("backoff_on_standoff", True))
        # Altitude lock: hold the LIVE-cached mapping height during long ADVANCE pushes (forward pitch sinks
        # the drone into inner walls). target_altitude_y is cached live from the first valid post-prelude
        # pose (self-calibrating, NOT a baked value); world frame is +Y DOWN so a sink = LARGER y.
        self.altitude_lock = bool(e.get("altitude_lock", True))
        self.alt_drift_floor = float(e.get("alt_drift_floor", 0.3))
        self.target_altitude_y = None        # cached lazily; PERSISTS across reset_leg (flight-level hold target)
        # Two-Phase Hybrid Ascent (Part 2): approach the ceiling with short SLAM-metered UP micro-pulses
        # (near-zero momentum), then a single continuous hold to cleanly latch the flow CEILING detector.
        # joy_vertical is a DISCRETE -1/0/+1 axis (io_bridge) so the "gradual" climb is keystroke pulses,
        # not a throttle ramp. All general platform params (durations + a per-step gain floor) -> leakage-safe.
        self.ascend_micro_pulse_s = float(e.get("ascend_micro_pulse_s", 0.3))  # Phase-1 UP pulse length
        self.ascend_rest_s = float(e.get("ascend_rest_s", 0.5))                # Phase-1 rest between pulses (momentum bleed + pose read)
        self.ascend_gain_eps = float(e.get("ascend_gain_eps", 0.05))           # per-cycle altitude-gain noise floor (SLAM units)
        self.ascend_stall_cycles = int(e.get("ascend_stall_cycles", 2))        # consecutive flat cycles that confirm the ceiling
        self.ascend_latch_hold_s = float(e.get("ascend_latch_hold_s", 2.0))    # Phase-2 continuous hold (> detector arm_blank + contact window)
        # Height re-calibration (CALIBRATING_HEIGHT): re-run the two-phase ascend->descend to re-tap the ceiling.
        # SESSION-21: the PERIODIC per-goal-change TRIGGER is RESTORED (session 17 deleted it believing the sag
        # was self-inflicted via the unset triggerDown; live flights proved the drone still does NOT hold
        # altitude). On a GENUINE goal change (moved > calib_goal_change_dist) past the calib_cooldown_s
        # cooldown, re-tap the ceiling to re-latch the mapping height for the new leg. SESSION-11 STATE-GATED
        # VERIFY: judge the calibration's RESULT after it ends (CALIB_VERIFY) against a continuous rolling
        # baseline of NORMAL flying altitude (_mapping_altitude_history), frozen during any calibration. A
        # settled height significantly BELOW the baseline median (+Y DOWN => a LARGER pos_y) => the calibration
        # SANK the drone (poisoning the live-camera-Y occupancy slab) => climb to clean airspace (ASCEND_ESCAPE)
        # -> slide 1u (CALIB_TRANSLATE) -> retry. All GENERAL params / LIVE-relative thresholds -> no room leak.
        self.calibrate_on_goal_change = bool(e.get("calibrate_on_goal_change", False))  # session 22: default OFF
        self.calib_cooldown_s = float(e.get("calib_cooldown_s", 60.0))         # min seconds between ceiling taps (configurable)
        self.calib_goal_change_dist = float(e.get("calib_goal_change_dist", 1.0))  # goal must move > this to re-calibrate
        # SLAM-COMFORT gate (session 22): a calibration launch/redo/retry additionally requires the rolling
        # average of HEALTHY-frame SLAM latencies to clear this bar (once the window is full) — comfortable,
        # not merely alive. Platform SLAM-behavior params (latency/frame counts), room-independent.
        self.calib_slam_avg_ms = float(e.get("calib_slam_avg_ms", 666.0))
        self.calib_slam_avg_window = int(e.get("calib_slam_avg_window", 10))
        self.calib_gate_max_s = float(e.get("calib_gate_max_s", 30.0))   # gated redo wait bound -> one failed attempt
        self._slam_ms_win = collections.deque(maxlen=self.calib_slam_avg_window)  # healthy-frame latency window
        self._calib_gate_since = None        # 'now' the comfort gate started blocking a CALIB_LOST_HOLD redo
        self._pending_notice = None          # one-shot operator notice (run_explore prints; e.g. height-drift warn)
        # Diagnostic session: position-state monitoring at the two hop-judgment-relevant instants. One-shot
        # stashes, mirroring `_pending_notice` -- run_explore pops + prints + diag-logs each tick.
        self._hop_baseline_msg = None        # set when the hop-start pose/cap_ts is bound (ADVANCE)
        self._hop_judge_msg = None           # set when the hop outcome is evaluated (REPLAN)
        self.calib_max_retries = int(e.get("calib_max_retries", 2))            # climb+translate+re-run attempts per calibration
        self._last_calib_t = None            # 'now' of the last ceiling tap (cooldown gauge); FLIGHT-level (persists).
        #                                      review-A: None does NOT lock calibration out — cooldown_ok treats
        #                                      "never calibrated" (--no-takeoff / failed prelude) as allowed.
        self._leg_goal_prev = None           # last goal committed for ORIENT/calibration (goal-change gauge; persists)
        # --- session-11 state-gated verification (CALIB_VERIFY / ASCEND_ESCAPE / CALIB_TRANSLATE) ---
        self.mapping_alt_history_len = int(e.get("mapping_alt_history_len", 200))   # rolling baseline length
        self.calib_min_baseline_samples = int(e.get("calib_min_baseline_samples", 10))  # samples before VERIFY can judge
        self.calib_settle_gate_s = float(e.get("calib_settle_gate_s", 1.0))    # hold until a frame CAPTURED >= this after DESCEND
        self.calib_low_height_margin = float(e.get("calib_low_height_margin", 0.3))  # settled y > med + this => SANK => FAIL
        self.calib_verify_max_s = float(e.get("calib_verify_max_s", 5.0))      # SAFETY cap on settle-and-judge (then PASS, logged)
        self.calib_retry_translate_dist = float(e.get("calib_retry_translate_dist", 1.0))  # CALIB_TRANSLATE slide distance
        # --- calibration INTERRUPTED by a plan loss (CALIB_LOST_HOLD): survive the loss, redo the re-tap ---
        # A plan loss DURING a calibration must NOT drop the drone into the normal recovery (which forgets the
        # calibration and leaves it glued near the ceiling). Instead hold, watch the SLAM frame "pulse", and
        # REDO the calibration once SLAM solves fast AND the plan is OK. Frame counts of the platform's SLAM
        # pulse (general robustness params, NOT a room answer).
        self.calib_lost_recover_frames = int(e.get("calib_lost_recover_frames", 6))    # fresh frames < slam_slow_ms => solve OK
        self.calib_lost_bump_slow_frames = int(e.get("calib_lost_bump_slow_frames", 6))  # fresh frames >= slam_slow_ms => wake-SLAM bump
        # --- calibration ESCAPE (session 15): bound the finish->lose-plan->retry loop. After N consecutive
        # failed calibrations, push to a fresh vantage + hold for SLAM, retry; N more -> STUCK. General counts.
        self.calib_escape_after = int(e.get("calib_escape_after", 3))            # consecutive fails -> escape / then STUCK
        self.calib_escape_ok_frames = int(e.get("calib_escape_ok_frames", 12))  # fresh fast frames + OK to recover post-escape
        self.calib_escape_push_s = float(e.get("calib_escape_push_s", 1.0))     # ring-picked push to a new vantage
        self._calib_fail_streak = 0          # consecutive failed calibration attempts (reset on a clean CALIB_VERIFY PASS)
        self._calib_escaped = False          # a CALIB_ESCAPE has already run this streak -> the next N fails -> STUCK
        self._calib_escape_phase = None      # None | "PUSH" | "HOLD" within CALIB_ESCAPE
        # Continuous rolling baseline of NORMAL flying altitude (pos_y). Session 18: ingest ONE reading per
        # FRESH SLAM frame (deduped by frame_id — NOT once per ~50 Hz control tick, which used to re-append
        # the same stale pose ~25x and make the median lurch/lag), starting ONLY after the first calibration
        # reports height-OK (`_height_calibrated`) and FROZEN during any calibration (_calib_active). +Y DOWN.
        # FLIGHT-level: persists across reset_leg (like target_altitude_y).
        self._mapping_altitude_history = collections.deque(maxlen=self.mapping_alt_history_len)
        self._height_calibrated = False      # latched True once the first CALIB_VERIFY resolves -> start measuring
        self._last_alt_frame_id = None       # last SLAM frame_id ingested into the baseline (per-frame dedup)
        self._calib_active = False           # True from a calibration start (TAKEOFF->ASCEND / CALIBRATING_HEIGHT)
        #                                      until CALIB_VERIFY resolves — freezes the baseline ingest.
        self._descend_issue_t = None         # 'now' the DESCEND recipe was created (CALIB_VERIFY settlement-gate origin)
        self._calib_interrupted = False      # a calibration was cut short by a plan loss -> a redo is owed (redo on recovery)
        self._calib_lost_bumped = False      # the one-shot (max-1) un-glue DOWN bump has fired this CALIB_LOST_HOLD episode
        # --- POST-MISSION FLOOR-DOCK POSTLUDE (RETURN_TO_ORIGIN -> DOCK_FLOOR -> LOW_STANDOFF -> DONE) ---
        # When the corner tour is fully exhausted (planner done=True), instead of a static hover at mapping
        # height the drone flies home to the take-off origin, descends GENTLY (pulsed, mirroring the two-phase
        # ascent) to the floor, nudges up to a low stand-off, then stands by. All GENERAL platform/robustness
        # params (durations / stand-off scale) — NO room answer (origin is the SLAM-frame [0,0]; the floor is
        # detected LIVE by the flow FLOOR collapse). A continuous hold-down is FORBIDDEN (see DOCK_FLOOR).
        self.home_reach_dist = float(e.get("home_reach_dist", self.goal_reach_dist))  # "reached origin" test
        self.home_max_s = float(e.get("home_max_s", 30.0))          # SAFETY cap on homing (then dock here; logged)
        self.postlude_recover_budget_s = float(e.get("postlude_recover_budget_s", 30.0))  # SAFETY: total wall-clock
        #        budget for POSTLUDE_LOST_HOLD to demand a full clean streak before relaxing the recovery gate (still
        #        only ever resumes on a status=="OK" tick -- never blind; see _step_postlude_lost)
        self.dock_pulse_s = float(e.get("dock_pulse_s", self.ascend_micro_pulse_s))   # Phase-1 DOWN micro-pulse length
        self.dock_rest_s = float(e.get("dock_rest_s", self.ascend_rest_s))            # Phase-1 rest (momentum bleed + pose read)
        self.dock_max_s = float(e.get("dock_max_s", 20.0))          # SAFETY cap on the descent (then proceed; logged)
        self.floor_standoff_nudge = float(e.get("floor_standoff_nudge", 0.5))  # LOW_STANDOFF up-nudge duration (s)
        # --- GRADUAL HEIGHT TRIM (session 14, RESTORED session 21): a fine PITCH-aim + forward climb BETWEEN
        # calibrations. (Session 17 deleted it as "a self-inflicted sag"; live flights proved the sag is real.)
        # joy_vertical is a DISCRETE full-thrust axis; TRIM instead pitches the aim UP and pushes forward so
        # the drone flies toward the raised aim = a gradual climb (rate = push duration), the forward part
        # feeding SLAM parallax. Trigger: pos_y sank past ceiling_y + trim_sag_ratio*delta. All GENERAL params;
        # the 3 references (_ceiling_y/_desired_y/_trim_delta) are re-measured LIVE at every calibration.
        self.trim_enable = bool(e.get("trim_enable", True))
        self.trim_sag_ratio = float(e.get("trim_sag_ratio", 1.2))
        self.trim_high_ratio = float(e.get("trim_high_ratio", 0.2))  # too-HIGH mirror band (session 22): TRIM
        #                                    DOWN when pos_y < desired_y - this*delta (glued near the ceiling)
        self.trim_aim_s = 0.5        # AUTOMATIC (session 22): io_bridge ramps the aim ±0.05/tick @60Hz -> ±1.0
        #                              saturates in ~0.33s; 0.5s is a platform constant with margin (not a knob).
        #                              The aim is then HELD at ±1 through the entire FWD push (re-emitted per tick).
        self.trim_fwd_s = float(e.get("trim_fwd_s", 0.5))
        self.trim_settle_s = float(e.get("trim_settle_s", 1.0))
        self.trim_reposition_s = float(e.get("trim_reposition_s", 0.5))
        self.trim_pitch_up = float(e.get("trim_pitch_up", -1.0))   # -1 aims UP = climb (confirmed live; +1 aimed DOWN)
        self.trim_throttle = float(e.get("trim_throttle", 0.4))   # brisk forward push, like the parallax scoot
        self.trim_reset_s = 0.15     # brief 'c' aim-reset pulse after the climb (platform action, like a turn's 'c')
        # The three calibration references (FLIGHT-level, persist across reset_leg like target_altitude_y).
        # Captured ONLY at a settled CALIB_VERIFY pass (Trap D: never the raw tap / post-bump wobble). +Y DOWN,
        # so _desired_y > _ceiling_y and _trim_delta > 0.
        self._ceiling_y = None       # pos_y while glued to the ceiling (this calibration's climb peak)
        self._desired_y = None       # settled pos_y after the bump-down (THE flight's height reference, sess 22)
        self._trim_delta = None      # _desired_y - _ceiling_y (how far below the ceiling we fly)
        self._first_ceiling_y = None  # the FIRST calibration's ceiling (Y-DRIFT audit baseline; never overwritten)
        self._height_drift_warned = False  # once-per-flight LOUD warning when |median - desired_y| > delta
        # Ram guard: "pushing forward but the SLAM pos isn't advancing toward the goal" = riding an unmapped
        # (invisible) collider. The forward-clearance ray can't see it (None when SLAM flickers; it also rises
        # with the drone as it climbs the wall) and the flow WALL needs a looming COLLAPSE that never comes on a
        # slow ram. So detect it in POS space: accrue forward-advancing time without progress; stop the leg
        # before the ram kills SLAM. Repeated re-commits then hit the F4 60 s stagnation blacklist.
        # The ram decision is SELF-CALIBRATING (no baked absolute, per the no-leakage rule): measure the
        # drone's OWN free-flight world speed live (1s into the first ADVANCE, sampled up to sample_s or
        # until a SLAM event), then fire only when the live windowed speed drops below `ram_speed_frac` of
        # that nominal. This distinguishes a legitimately SLOW crawl in open space (speed ~= nominal) from a
        # drone physically pinned on an invisible collider (speed -> 0), which the old absolute goal-closing
        # threshold (0.15 u / 3 s ~= 0.05 u/s) could not — it false-fired on the platform's normal crawl.
        self.ram_stall_s = float(e.get("ram_stall_s", 3.0))            # below-nominal seconds -> stop the leg
        self.ram_speed_frac = float(e.get("ram_speed_frac", 0.33))     # fire when speed < frac * nominal
        self.ram_speed_window_s = float(e.get("ram_speed_window_s", 1.0))   # rolling window for the live speed
        self.ram_calib_skip_s = float(e.get("ram_calib_skip_s", 1.0))       # skip the first Ns of the first ADVANCE
        self.ram_calib_sample_s = float(e.get("ram_calib_sample_s", 5.0))   # then sample nominal for up to Ns
        self.ram_calib_min_sample_s = float(e.get("ram_calib_min_sample_s", 1.0))  # min clean span to accept a nominal
        self.ram_calib_min_speed = float(e.get("ram_calib_min_speed", 1e-3))       # reject a degenerate ~0 nominal (stuck calib window)
        self._nominal_speed = None           # LIVE-calibrated free-flight speed (FLIGHT-level; persists across legs)
        self._ram_speed_win = collections.deque()   # rolling (t, x, z) for the timestamp-based windowed speed
        self._ram_speed = None               # last computed live windowed speed (for logging / telemetry)
        self._calib_start_t = None           # first-ADVANCE entry time (calibration clock origin)
        self._calib_samples = []             # collected windowed-speed samples during calibration
        self._ram_accum = 0.0                # accrued below-nominal (stalled) time
        self._ram_last_t = None              # last stalled-tick time (for a clamped dt)
        # Parallax scouting: a goal needing MORE than one turn_step is reached as turn -> short translate
        # (forward/back per the rays, for SLAM parallax) -> settle -> turn again -> ... -> aim -> advance.
        self.parallax_scout = bool(e.get("parallax_scout", True))
        self.parallax_push_dist = float(e.get("parallax_push_dist", 0.5))  # BACKWARD push: translate this far (SLAM units)
        # PUSH GATE (self-scaling to the push, NOT a room answer): a ring direction is "pushable" when its
        # clearance is None (nothing near within the ring's near-field range -> room; worst case we bump a wall
        # and the flow WALL detector + 2-bump blacklist recover) OR >= parallax_min_clear = push_dist + buffer.
        # Replaces the old stop_clearance_dist + parallax_pad gate (1.4u), which refused a 0.5u scoot into a
        # 1.25u space (the 20260712 "no room fwd/back" skip-loop).
        self.parallax_clear_buffer = float(e.get("parallax_clear_buffer", 0.2))
        self.parallax_min_clear = self.parallax_push_dist + self.parallax_clear_buffer
        self.parallax_push_s = float(e.get("parallax_push_s", 2.0))        # SAFETY time cap on a push
        # (legacy forward push magnitude; forward push retired -> backward/strafe only. Kept for compat.)
        self.parallax_push_throttle = float(e.get("parallax_push_throttle", 0.4))
        self.parallax_max_pushes = int(e.get("parallax_max_pushes", 8))
        # STRAFE recipe (platform control dynamic, manually calibrated by the operator): joy_horizontal is a
        # strafe axis (+1 right / -1 left); strafe is the most RESPONSIVE axis (near-zero warm-up) so a short
        # TIMED hold gives a reliable slight scoot (SLAM barely resolves 0.5u of a brief lateral move, so a
        # distance-quantized loop would just ride the time cap). Magnitude + hold read from the recipe.
        _strafe = self.pb.recipe("strafe")
        _sh = next(s for s in _strafe if "joy_horizontal" in s)
        # Strafe throttle override (config): the strafe (joy_horizontal) was the ONE control axis left at full
        # magnitude while advance/reverse were throttled to 0.2 -> a full-tilt lateral scoot into an unmapped,
        # yawed corner SCRAPED the wall and spun the drone, killing SLAM (flight 20260713). Throttle it like the
        # others. CAVEAT: joy_horizontal is documented "(-1 to 1)" but so is joy_vertical, which is empirically a
        # DISCRETE full-thrust axis -> verify live that 0.2 actually slows the strafe (else shorten strafe_hold_s).
        st_thr = e.get("strafe_throttle", None)
        self._strafe_mag = float(st_thr) if st_thr is not None else abs(float(_sh["joy_horizontal"]))
        self.strafe_hold_s = float(_sh.get("duration_s", 0.25))
        # D2 SCRAPE GUARD: a parallax strafe while pinned VERY close behind (and possibly yawed) can drive the
        # drone's flank into the wall -> scrape -> spin -> SLAM death. When that danger is present AND forward is
        # clearly open (the forward raycast IS reliable forward), reposition forward out of the tight corner first,
        # then strafe from safer space. All GENERAL margins/durations (no room answer): a close-behind danger
        # distance, a forward-open threshold (~the stand-off + reach scale), and a forward-push duration (scaled
        # for the gentle 0.2 throttle + slow acceleration-from-rest so the push actually translates).
        self.strafe_backwall_danger_dist = float(e.get("strafe_backwall_danger_dist", 0.4))
        self.strafe_reposition_min_fwd = float(e.get("strafe_reposition_min_fwd", 2.0))  # forward "clearly open" gate
        self.strafe_reposition_fwd_s = float(e.get("strafe_reposition_fwd_s", 2.0))      # forward reposition duration
        # Baseline nudge (Part 2): a one-shot horizontal translation after the ceiling tap + descend, to
        # seed a SLAM translational baseline (parallax) BEFORE the first exploration yaw (pure rotation is
        # the SLAM-killer). Reuses the parallax ring-pick + distance-quantized translate. General params.
        self.baseline_nudge_dist = float(e.get("baseline_nudge_dist", 0.4))    # translate this far (SLAM units)
        self.baseline_nudge_max_s = float(e.get("baseline_nudge_max_s", 2.0))  # SAFETY time cap on the nudge
        # PERSISTS across reset_leg (like airborne_done): seed the baseline exactly once. True when there is
        # no prelude (no_takeoff = a manual handover, SLAM already has a flown baseline).
        self._baseline_seeded = bool(no_takeoff)
        self._push_count = 0                 # consecutive scout pushes this leg (anti-deadlock cap)
        self._push_dir = None                # active push axis: "forward"|"backward" (prelude nudge/calib-translate)
                                             #   or "backward"|"strafe_left"|"strafe_right" (PARALLAX_PUSH; never forward)
        self._push_start_pos = None          # SLAM pos at the start of the current push (distance gauge)
        # A full give-up (backward AND both sides blocked) latches here so the NEXT direction pick (this leg's
        # re-ORIENT or a fresh PARALLAX_PUSH) doesn't immediately retry the same doomed backward push just
        # because the ring still (falsely) reads it as open. Cleared once the drone has moved
        # parallax_min_clear away from the anchor -- SLAM-freeze-safe (see _pick_ring_direction).
        self._parallax_back_blocked = False
        self._parallax_back_blocked_anchor = None
        self._after_orient = "ADVANCE"       # where ORIENT routes after the turn: ADVANCE (aimed) | PARALLAX_PUSH
        # REPLAN idle backstop: with the diagonal-sweep planner a goal=None/!done plan is only a momentary
        # startup tick before the first frontiers form. If it ever PERSISTS past this window, raise a
        # visible log + telemetry flag (`no_goal_stall`) instead of idling dark forever (NO SILENT FALLBACK).
        # A general robustness timeout (long enough to cover normal frontier formation), NOT a room answer.
        self.no_goal_idle_s = float(e.get("no_goal_idle_s", 12.0))
        self.no_goal_stall = False           # telemetry: True once the backstop fired (visible degraded flag)
        self._no_goal_since = None           # 'now' when the current goal=None/!done idle began, else None
        self._no_goal_warned = False         # one-shot guard for the backstop warning
        self._done_logged = False            # one-shot guard for the EXPLORE COMPLETE annunciation
        _tr = self.pb.recipe("turn_right")
        _hold = next(s for s in _tr if "yaw" in s)                    # the yaw-hold step = the actual turn
        _creset = next((s for s in _tr if s.get("btnCdown")), None)   # the 'c' aim-reset step
        self._turn_hold_dur = float(_hold["duration_s"])
        self._turn_yaw_mag = abs(float(_hold["yaw"]))
        self._turn_c_dur = float(_creset["duration_s"]) if _creset else 0.16
        # airborne_done gates the one-time arm/takeoff prelude; it PERSISTS across reset_leg so an
        # autonomy-off / PLAN-STALE / PLAN-LOST interruption never re-arms a flying drone.
        self.no_takeoff = bool(no_takeoff)
        self.airborne_done = bool(no_takeoff)
        self.reset_leg()

    def reset_leg(self):
        """Return to a clean state on entry / whenever the caller interrupts (autonomy off, plan
        lost/stale). Resume exploring (REPLAN) once airborne — NEVER re-running the prelude mid-flight;
        restart the prelude (ARM) only if takeoff never completed."""
        self.state = "REPLAN" if self.airborne_done else "ARM"
        self.t_state = 0.0
        self.leg_goal = None
        self._player = None
        self._settle_to = None     # where SETTLE routes next (prelude chaining); None => REPLAN
        self._push_count = 0       # scout cap resets on interruption (target_altitude_y persists, like airborne_done)
        self._push_dir = None
        self._push_after_reposition = None   # D2: the strafe dir queued behind a forward-reposition (scrape guard)
        self._push_start_pos = None
        self._after_orient = "ADVANCE"
        self._fallback_attempts = 0
        self._fallback_retreat_forward = None
        # Session-12 recovery flags. A manual takeover (the only caller of reset_leg) invalidates any in-flight
        # recovery, so clear them here; DURING a flight they persist across the PLAN-LOST/PLAN-STALE flicker.
        self._recovering = False       # True from the first PLAN-STALE of a loss until a confirming ADVANCE (>=1u)
        self._history_broken = False   # True once a re-locked-but-unconfirmed drone moves -> leftover history is stale
        self._rec_settling = False     # not mid an inter-action recovery settle
        self._recovery_adv_start = None  # pos at the start of a post-recovery ADVANCE leg (the >=1u progress gauge)
        self._slam_resume = None    # SLAM streak/latest persist (health is flight-level); only the pending resume clears
        self._slam_stepback_count = 0   # per-hold step-back counter + timer clear on interruption
        self._slam_hold_start = None
        # Two-Phase Hybrid Ascent runtime (lazy-init in the ASCEND handler when _ascend_phase is None).
        self._ascend_phase = None       # "PULSE" | "REST" | "LATCH" within ASCEND (None => (re)initialize)
        self._ascend_phase_t0 = None    # entry time of the current ascend sub-phase
        self._ascend_prev_y = None      # last valid pos_y sample (for the per-cycle altitude gain dZ)
        self._ascend_stall_count = 0    # consecutive flat-gain cycles (confirms the ceiling)
        self._ascend_start_t = None     # ASCEND entry time (ascend_max_s safety cap)
        # Postlude runtime (lazy-init in the handlers when the phase is None): homing + orient + pulsed floor-dock.
        self._home_phase = None         # None | "PLAN" | "TURN" | "SETTLE" | "ADVANCE" within RETURN_TO_ORIGIN
        self._home_t0 = None            # RETURN_TO_ORIGIN entry time (home_max_s cap)
        self._home_adv_t0 = None        # current homing ADVANCE sub-leg start (per-leg time cap)
        self._home_adv_start_pos = None # pose at the sub-leg start (re-aim after a bounded advance)
        self._home_settle_to = None     # which homing phase the SETTLE routes back to ("ADVANCE" after a turn, "PLAN" after an advance)
        self._takeoff_heading = None    # SLAM heading_deg captured once airborne+healthy = the take-off heading (ORIENT_HOME target)
        self._orient_home_phase = None  # None | "PLAN" | "TURN" | "SETTLE" within ORIENT_HOME
        self._dock_phase = None         # None | "PULSE" | "REST" | "LATCH" within DOCK_FLOOR (mirrors ASCEND)
        self._dock_phase_t0 = None      # entry time of the current dock sub-phase
        self._dock_prev_y = None        # last valid pos_y sample (per-cycle descent gain dZ)
        self._dock_stall_count = 0      # consecutive flat-gain cycles (confirms the floor)
        self._dock_start_t = None       # DOCK_FLOOR entry time (dock_max_s cap)
        # Postlude loss-survival (mirror of CALIB_LOST_HOLD): a plan loss during homing/orient/dock must NOT drop
        # into the generic HOLD_LOST/FALLBACK recovery (which abandons the postlude); HOLD + resume when SLAM+plan OK.
        self._dock_interrupted = False  # telemetry: a postlude stage was interrupted by a plan loss
        self._postlude_resume = None    # which postlude state to resume after a POSTLUDE_LOST_HOLD
        self._postlude_t0 = None        # wall-clock start of the whole postlude ending (postlude_recover_budget_s cap)
        # Gradual height TRIM runtime (per-episode; the 3 references are flight-level and set in __init__).
        self._trim_dir = "UP"           # "UP" (sagged low -> climb) | "DOWN" (glued high -> descend); session 22
        self._trim_phase = None         # None | "REPOS" | "AIM" | "FWD" | "RESET" | "WAIT" within TRIM
        self._trim_phase_t0 = None      # entry time of the current TRIM sub-phase
        self._trim_cmd_t0 = None        # 'now' the climb command issued (WAIT settle-gate origin; same clock as cap_ts)
        self._trim_resume_goal = None   # the committed leg_goal snapshotted on TRIM entry (Trap B: re-aim at it, don't re-pick)
        self._trim_repos_move = None    # the reposition control dict (reverse/strafe) chosen by the ring gate
        self._trim_sag_y = None         # pos_y that tripped the sag trigger (for the entry log)
        self._trimming = False          # telemetry: True while a TRIM is running
        # Calibration escape runtime (a manual takeover invalidates a stuck-calibration episode).
        self._calib_fail_streak = 0
        self._calib_escaped = False
        self._calib_escape_phase = None
        self._ram_accum = 0.0       # ram-guard stall accumulator is per-leg
        self._ram_last_t = None
        self._ram_speed_win.clear() # a time gap across an interruption must not read as a false slowdown
        self._ram_speed = None
        self._hop_tick = 0          # session 20: hop cadence is per-leg
        self._hop_start_dist = None  # a hard interruption abandons the pending per-hop progress eval
        self._hop_start_goal = None
        # A finalized nominal free-flight speed PERSISTS (flight-level, like target_altitude_y); only an
        # in-progress calibration is discarded on a hard interruption -> it restarts on the next clean ADVANCE.
        if self._nominal_speed is None:
            self._calib_samples = []
            self._calib_start_t = None
        # 2-bump blacklist latch (kinematic): an advance-blocked stop (flow WALL / ram-guard / stand-off)
        # emits ONE bump pulse to the planner, then DISARMS until the drone physically disengages (run_explore
        # re-arms on a published reverse command OR displacement > goal_reach_dist from the anchor). This
        # guarantees a single continuous contact counts as exactly one bump, immune to state-machine flicker.
        self._bump_armed = True
        self._last_bump_anchor = None   # [x,z] where the last counted bump fired (displacement re-arm gauge)
        self._bump_pulse = None         # pending bump goal for run_explore to publish, then clear
        self._bump_reason = None        # why the pending bump fired (standoff / wall-contact / ram-guard), for the log
        self._bump_is_corner = None     # was the bumped goal a NEAR sweep-tour corner? (goals-DB evidence)
        self._missed_bump = None        # a real advance-blocked contact that did NOT emit a pulse (latch disarmed /
        #                                 parallax-blocked path) -> run_explore logs a MISSED-BUMP marker
        # An interruption (autonomy off = a manual takeover) invalidates the command history: the drone may
        # have been moved by hand, so the recorded maneuvers no longer map to the trajectory. Drop it.
        self.command_history.clear()
        self.done = False
        self.no_goal_stall = False
        self._no_goal_since = None
        self._no_goal_warned = False
        self._done_logged = False
        # Height re-calibration is per-attempt: a manual interruption abandons an in-progress re-tap (the
        # flight-level cooldown / prev-goal / rolling altitude baseline PERSIST — they live in __init__, not
        # here). Clear the freeze flag too: reset_leg only fires on a MANUAL takeover (autonomy off), where an
        # interrupted calibration is genuinely abandoned and the baseline ingest should resume on the next
        # clean flight — a SLAM blip DURING a calibration does NOT call reset_leg, so it keeps the freeze.
        self._recalibrating = False
        self._calib_retries = 0
        self._calib_active = False
        self._descend_issue_t = None
        self._calib_interrupted = False      # a manual takeover abandons any owed calibration redo
        self._calib_lost_bumped = False

    def _quantize_turn(self, be):
        """Quantize a bearing error (deg) to the nearest whole `turn_step_deg` aim change (signed)."""
        if be is None:
            return 0.0
        return round(be / self.turn_step_deg) * self.turn_step_deg

    def _turn_steps(self, theta):
        """Recipe steps for an OPEN-LOOP ~`theta` deg turn (sustained yaw hold scaled from the calibrated
        recipe, then the 'c' aim reset). theta≈0 -> just the attitude reset. Shared by _build_turn, the
        command-history rewind (inverse turn = _turn_steps(-theta)), and the fallback."""
        if abs(theta) < 1e-6:
            return list(self.pb.recipe("reset_attitude"))
        hold = self._turn_hold_dur * abs(theta) / self.turn_recipe_deg
        return [{"yaw": math.copysign(self._turn_yaw_mag, theta), "duration_s": hold},
                {"btnCdown": True, "duration_s": self._turn_c_dur}]

    def _build_turn(self, theta):
        """A RecipePlayer that turns ~`theta` deg open-loop then resets the aim with 'c'."""
        return RecipePlayer(self._turn_steps(theta), name=f"turn{theta:+.0f}")

    def _trim_exit(self, now, plan, msg):
        """Leave a gradual-height TRIM (session 14, restored session 21). Trap B: RESTORE the committed goal
        snapshotted on entry and re-aim (ORIENT) at it — never a fresh planner pick, so the trim can't pollute
        goal commitment or the goals-DB. Falls back to SETTLE->REPLAN only if no goal was committed or the pose
        is untrustworthy. Sets the next state via _enter and RETURNS the event string (the TRIM handler falls
        through to the common return)."""
        self._trimming = False
        self._trim_phase = None
        self._player = None
        g = self._trim_resume_goal
        self._trim_resume_goal = None
        pos, hd = plan.get("pos"), plan.get("heading_deg")
        if g is not None and pos is not None and hd is not None:
            self.leg_goal = list(g)
            bearing = math.degrees(math.atan2(g[0] - pos[0], g[1] - pos[1]))   # 0=+Z, +90=+X (matches homing)
            be = ((bearing - float(hd) + 180.0) % 360.0) - 180.0
            theta = self._quantize_turn(be)
            if self.clamp_leg_turn:
                theta = max(-self.turn_step_deg, min(self.turn_step_deg, theta))
            self._leg_theta = theta
            self._after_orient = "ADVANCE"
            self._player = self._build_turn(theta)
            self._enter("ORIENT", now)
            return f"{msg} -> re-aim ORIENT at preserved goal {self.leg_goal} (turn {theta:+.0f})"
        self._settle_to = "REPLAN"
        self._enter("SETTLE", now)
        return f"{msg} -> settle -> replan (no committed goal / pose unavailable)"

    # ------------------------------------------------- command history (control-space rewind)
    # While `_recovering` (a re-lock we don't yet trust), appends are FROZEN: the re-aim maneuvers are flown on a
    # shaky fresh pose, so logging them would poison the rewind chain (D5). Logging resumes on a confirming ADVANCE.
    def _log_turn(self, theta):
        if self._recovering:
            return
        if abs(theta) > 1e-6:
            self.command_history.append({"kind": "turn", "theta": float(theta)})

    def _log_move(self, kind, value, duration):
        """Record a flown translation (kind='forward'|'reverse'|'strafe') for a later inverse replay. EVERY
        flown translation is logged — no minimum-duration guard: the SLAM-loss spiral is made of micro-short
        ADVANCE legs, and dropping them left the rewind with turns only (it just spun in place). For 'strafe'
        the value is the SIGNED joy_horizontal (+right / -left). FROZEN while `_recovering` (untrusted re-lock)."""
        if self._recovering:
            return
        self.command_history.append({"kind": kind, "value": float(value), "duration_s": float(max(0.0, duration))})

    def _log_move_push(self, dirn, duration):
        """Log a completed PARALLAX_PUSH translation into the command history (backward -> reverse; strafe ->
        signed joy_horizontal). Shared by the SLAM-slow bail and the normal push-done exit."""
        if dirn == "backward":
            self._log_move("reverse", self.reverse_throttle, duration)
        elif dirn == "strafe_right":
            self._log_move("strafe", self._strafe_mag, duration)
        elif dirn == "strafe_left":
            self._log_move("strafe", -self._strafe_mag, duration)

    def _invert_one(self, m):
        """Inverse recipe steps for ONE recorded maneuver (forward<->reverse, strafe sign-flip, turn theta ->
        -theta). Shared by the full-history rewind and the single SLAM-settle step-back."""
        if m["kind"] == "turn":
            return list(self._turn_steps(-m["theta"]))
        if m["kind"] == "forward":
            return [{"reverse": m["value"], "duration_s": m["duration_s"]}]
        if m["kind"] == "reverse":
            return [{"trigger": m["value"], "duration_s": m["duration_s"]}]
        if m["kind"] == "strafe":
            return [{"joy_horizontal": -m["value"], "duration_s": m["duration_s"]}]   # left<->right
        return []

    def _invert_history(self):
        """Flatten the recent command history into inverse recipe steps: reverse chronological order and
        invert each maneuver (forward<->reverse; turn theta -> -theta), bounded to the last
        `command_history_s` seconds of motion. Playing these open-loop approximately RETRACES the path,
        re-exposing the camera to keyframes it already recorded so RELOC can re-match."""
        steps, acc = [], 0.0
        for m in reversed(self.command_history):
            steps.extend(self._invert_one(m))
            if m["kind"] == "turn":
                acc += self._turn_hold_dur * abs(m["theta"]) / self.turn_recipe_deg
            else:
                acc += m["duration_s"]
            if acc >= self.command_history_s:
                break
        return steps

    def _pop_stepback(self):
        """Pop the MOST-RECENT recorded maneuver off the rewind queue and return its inverse recipe steps
        (ONE step back through the queue). Progresses backward through the history on each call. Returns
        None when nothing poppable remains."""
        while self.command_history:
            m = self.command_history.pop()
            steps = self._invert_one(m)
            if steps:
                return steps
        return None

    def _step_stale(self, now, plan, wall_contact):
        """PLAN-STALE (SLAM not TRACKING, perception publishing): a CONSUMING control-space rewind — pop the
        inverse of the recently-flown maneuvers ONE at a time (watching for OK at the step() top), draining the
        history to empty, then the ring-picked <=45deg fallback -> STUCK. `_recovering` + the give-up counter
        PERSIST across any PLAN-LOST/HOLD_LOST flicker, so the rewind never restarts and STUCK stays reachable
        (fixes the flight-20260713 frantic loop). If the drone already MOVED on an unconfirmed re-lock
        (`_history_broken`), the leftover history is spatially stale -> clear it and go straight to FALLBACK
        (no displaced ghost-path replay)."""
        ring = plan.get("clearance_ring")
        if ring:
            self._last_ring = ring          # remember the last good ring for the fallback direction choice
        st = self.state
        if st in ("STUCK", "WARMUP"):
            return {}, st, None              # hold until OK returns (handled at the step() top)
        if st == "REWIND":
            if self._rec_settling:
                # Inter-action settle: hold neutral so SLAM gets a still window to re-lock BEFORE the next inverse
                # (lost-SLAM flavor: fresh CAPTURE verified, not fast/OK — a genuine re-lock exits at the step()
                # top). Bounded so a dead pipeline still proceeds to the next re-exposure maneuver.
                sdone, capped = self._settle_poll(now, plan, require_fast=False,
                                                  min_frames=self.recovery_settle_frames,
                                                  max_hold_s=self.recovery_settle_max_s)
                if not sdone:
                    return {}, "REWIND", None
                self._rec_settling = False
                cap = " (settle timed out, no fresh frames)" if capped else ""
                # DRAIN the queue (consuming rewind): pop + play the next inverse, else the ring-picked fallback.
                steps = self._pop_stepback()
                if steps is not None:
                    self._player = RecipePlayer(steps, name="rewind")
                    return {}, "REWIND", (f"settled between rewind steps{cap} -> next inverse "
                                          f"[{len(self.command_history)} left]")
                self._player = None
                return self._begin_fallback(now, f"rewind drained (history empty){cap} -> ring-picked parallax fallback")
            active, done = self._player.fields(now)
            if not done:
                return active, "REWIND", None
            # This inverse maneuver finished -> SETTLE (let SLAM re-lock) BEFORE popping the next one.
            self._rec_settling = True
            self._settle_begin(now)
            return {}, "REWIND", "rewind step done -> settle (let SLAM breathe / re-lock) before the next inverse"
        if st == "FALLBACK":
            if self._rec_settling:
                # Inter-attempt settle: no more back-to-back spinning — hold neutral between sweep attempts so
                # SLAM can re-lock (bounded lost-SLAM flavor).
                sdone, capped = self._settle_poll(now, plan, require_fast=False,
                                                  min_frames=self.recovery_settle_frames,
                                                  max_hold_s=self.recovery_settle_max_s)
                if not sdone:
                    return {}, "FALLBACK", None
                self._rec_settling = False
                cap = " (settle timed out, no fresh frames)" if capped else ""
                if self._fallback_attempts >= self.fallback_max_attempts:
                    self._enter("STUCK", now)
                    return {}, "STUCK", (f"fallback exhausted ({self._fallback_attempts} attempts){cap} -> STUCK "
                                         "(HOLD; awaiting perception)")
                return self._begin_fallback(now, None)
            active, done = self._player.fields(now)
            if not done:
                return active, "FALLBACK", None
            self._player = None
            # This sweep attempt finished -> SETTLE before the next attempt / STUCK check.
            self._rec_settling = True
            self._settle_begin(now)
            return {}, "FALLBACK", "fallback attempt done -> settle (let SLAM breathe / re-lock) before the next sweep"
        # ---- fresh entry (first PLAN-STALE of this loss) OR re-entry after a HOLD_LOST flicker ----
        if not self._recovering:
            # The FIRST PLAN-STALE of this loss episode arms recovery. The flags + counter then PERSIST until a
            # confirming ADVANCE — never reset by a bare OK or by a LOST/STALE flicker (that was the loop bug).
            self._recovering = True
            self._history_broken = False
            self._fallback_attempts = 0
        # Ghost-path guard: a re-lock that already MOVED (unconfirmed) decoupled the leftover history from the
        # true pose -> clear it and BYPASS REWIND straight to the safe ring-picked fallback sweep.
        if self._history_broken:
            if self.command_history:
                self.command_history.clear()
                return self._begin_fallback(now, "secondary loss after an unconfirmed re-aim -> stale history "
                                                 "cleared -> ring-picked parallax fallback (no ghost path)")
            return self._begin_fallback(now, "secondary loss after an unconfirmed re-aim (history already "
                                             "drained) -> ring-picked parallax fallback")
        # CONSUMING rewind: pop the newest maneuver's inverse and play it; the step() top watches for OK.
        steps = self._pop_stepback()
        if steps is not None:
            self._player = RecipePlayer(steps, name="rewind")
            self._enter("REWIND", now)
            return {}, "REWIND", ("PLAN-STALE -> RECOVERY_REWIND (consuming): retracing recent maneuvers one "
                                  f"at a time to re-expose keyframes [{len(self.command_history)} left after this pop]")
        if not self._ever_tracked:
            # STARTUP: SLAM has never TRACKED yet (the prelude finishes on the FLOW ceiling detector, not on
            # SLAM). Don't spin a blind fallback into an unmapped room — HOLD and wait for SLAM to initialize.
            # The step() top snaps WARMUP -> SLAM_HOLD -> SETTLE -> REPLAN when OK returns.
            self._enter("WARMUP", now)
            return {}, "WARMUP", "PLAN-STALE at startup (SLAM still initializing) -> HOLD (no blind sweep)"
        return self._begin_fallback(now, "PLAN-STALE + EMPTY command history (post-collision?) -> ring-picked "
                                         "parallax fallback")

    def _step_calib_lost(self, now, status):
        """A plan loss (LOST/NO-PLAN/STALE) interrupted a height calibration. Release all controls and HOLD;
        watch the SLAM frame "pulse" (fresh frame_id + slam_ms, maintained by _update_slam every tick).
          RECOVER: >= calib_lost_recover_frames consecutive FRESH frames under slam_slow_ms AND the (level-
            triggered) planner status has ALSO caught up (status == OK) -> REDO the interrupted calibration
            (its own descend re-establishes the mapping height). The status == OK gate is what stops a 1-tick
            CALIBRATING_HEIGHT<->CALIB_LOST_HOLD oscillation when the status lags a healthy SLAM.
          STUCK: ONE DOWN bump (max, per hold) to try to unglue, then keep holding indefinitely for plan OK.
            Two causes, one bump total: (A) SLAM's SOLVE grinding (>= calib_lost_bump_slow_frames choked fresh
            frames) -> wake SLAM; (B) SLAM fast but the planner still can't lock a path -> unglue. A second
            nudge won't help SLAM and risks hitting walls, so it is capped at one.
        No time cap — the SLAM frame stream is the liveness signal (operator ask)."""
        # ENTRY (first loss during a calibration): latch, release controls, count the pulse FRESH from here
        # (ignore the pre-loss streak, which would let a stale "healthy" reading exit immediately).
        if self.state != "CALIB_LOST_HOLD":
            self._calib_interrupted = True
            self._calib_lost_bumped = False
            self._player = None
            self._slam_fast_streak = 0
            self._slam_slow_streak = 0
            self._enter("CALIB_LOST_HOLD", now)
            return {}, "CALIB_LOST_HOLD", ("plan loss DURING height-calib -> release controls, HOLD; redo "
                                           "calibration once SLAM solves fast AND plan is OK (calib interrupted)")
        # A descend bump in flight -> play it out, then back to neutral hold.
        if self._player is not None:
            active, done = self._player.fields(now)
            if done:
                self._player = None
                return {}, "CALIB_LOST_HOLD", None
            return active, "CALIB_LOST_HOLD", None
        slam_fast = self._slam_fast_streak >= self.calib_lost_recover_frames
        # RECOVER: SLAM's solve is healthy AND the planner has caught up -> this interrupted attempt is over and
        # COUNTS as a failure. Escalate before blindly redoing in place (session 15): redo < N; CALIB_ESCAPE at
        # N (first); STUCK at N after an escape (shared with CALIB_VERIFY via _calib_fail_escalate).
        # SESSION-22 COMFORT GATE: alive is not enough — the 20260717 redos fired on 6 alive-but-marginal
        # (616-797ms) frames and died in every ASCEND. Require the healthy-frame latency AVERAGE to clear
        # calib_slam_avg_ms too; while it doesn't, KEEP HOLDING (logged), and if it stays over the bar for
        # calib_gate_max_s count ONE failed attempt (allow_redo=False -> hold on; the escalation still reaches
        # CALIB_ESCAPE, which relocates away from the chronically uncomfortable spot).
        if slam_fast and status == "OK":
            if self._calib_slam_comfortable():
                self._calib_gate_since = None
                ev = self._calib_fail_escalate(now, f"SLAM healthy ({self._slam_fast_streak} fresh frames "
                                                    f"<{self.slam_slow_ms:.0f}ms) + plan OK")
                return {}, self.state, ev
            if self._calib_gate_since is None:
                self._calib_gate_since = now
                return {}, "CALIB_LOST_HOLD", (f"SLAM alive but NOT comfortable (avg {self._slam_ms_avg:.0f}ms "
                                               f">= {self.calib_slam_avg_ms:.0f}) -> HOLD the redo until the "
                                               f"average clears (max {self.calib_gate_max_s:.0f}s)")
            if (now - self._calib_gate_since) >= self.calib_gate_max_s:
                self._calib_gate_since = None      # next gate episode restarts its own clock
                ev = self._calib_fail_escalate(now, f"comfort gate timeout ({self.calib_gate_max_s:.0f}s with "
                                                    f"avg {self._slam_ms_avg:.0f}ms >= "
                                                    f"{self.calib_slam_avg_ms:.0f})", allow_redo=False)
                return {}, self.state, ev
            return {}, "CALIB_LOST_HOLD", None     # gated; holding for the average to clear
        # STUCK -> ONE bump total per hold (either cause), first frame emitted NOW, then hold for plan OK.
        stuck_slam = self._slam_slow_streak >= self.calib_lost_bump_slow_frames   # cause A: wake a grinding SLAM
        stuck_plan = slam_fast and status != "OK"                                 # cause B: unglue a stuck planner
        if not self._calib_lost_bumped and (stuck_slam or stuck_plan):
            self._calib_lost_bumped = True
            self._player = self.pb.player("descend")
            active, done = self._player.fields(now)   # emit the first bump frame THIS tick (no wasted neutral tick)
            if done:
                self._player = None
            why = "SLAM solve choking" if stuck_slam else f"SLAM fast but plan {status}"
            return active, "CALIB_LOST_HOLD", (f"{why} -> bump DOWN once (max) to unglue, then hold for plan OK")
        return {}, "CALIB_LOST_HOLD", None          # holding; wait for the SLAM pulse / plan OK

    def _calib_fail_escalate(self, now, base_why, allow_redo=True):
        """A calibration attempt FAILED (loss-interrupted, a CALIB_VERIFY timeout with no settled healthy
        pose, or a comfort-gate timeout). Bump the consecutive-fail streak and pick the next state (shared by
        _step_calib_lost and CALIB_VERIFY): REDO (CALIBRATING_HEIGHT) while < calib_escape_after; CALIB_ESCAPE
        at the threshold (first time); STUCK at the threshold after an escape already ran. Sets the state via
        _enter and RETURNS the event string. `allow_redo=False` (session-22 comfort-gate timeout): never launch
        a fresh ASCEND into uncomfortable SLAM — below the threshold just count the fail and KEEP HOLDING (the
        escalation to CALIB_ESCAPE/STUCK still fires at the threshold, relocating away from the bad spot)."""
        self._calib_fail_streak += 1
        if self._calib_fail_streak >= self.calib_escape_after:
            if not self._calib_escaped:
                self._calib_escaped = True
                self._calib_fail_streak = 0
                self._calib_escape_phase = None
                self._player = None
                self._enter("CALIB_ESCAPE", now)
                return (f"{base_why} -> {self.calib_escape_after} consecutive failed calibrations -> CALIB_ESCAPE "
                        "(ring-picked push to a fresh vantage, then hold for SLAM)")
            self._calib_active = False           # give up calibrating; stop freezing the baseline
            self._enter("STUCK", now)
            return (f"{base_why} -> {self.calib_escape_after} more failed calibrations after an escape -> "
                    "STUCK (HOLD in place; per-step logging paused)")
        if not allow_redo:
            return (f"{base_why} -> failed attempt [{self._calib_fail_streak}/{self.calib_escape_after}]; "
                    "KEEP HOLDING (no redo into uncomfortable SLAM)")
        self._recalibrating = True               # DESCEND PASS -> REPLAN (per-goal path), never the prelude path
        self._calib_retries = 0                  # a fresh redo gets its full retry budget
        self._enter("CALIBRATING_HEIGHT", now)   # re-sets _calib_active, clears _player/_ascend_phase
        return f"{base_why} -> REDO height calibration [fail {self._calib_fail_streak}/{self.calib_escape_after}]"

    def _step_calib_escape(self, now, status):
        """Escape a STUCK calibration (session 15): after calib_escape_after consecutive failed attempts, move
        ONCE to a fresh vantage (ring-picked parallax push — backward if pushable, else strafe to the roomier
        side, never forward) then HOLD indefinitely until SLAM+plan are healthy for calib_escape_ok_frames
        fresh frames, then RETRY the calibration. Owns EVERY status (routed at the step() top before the generic
        recovery divert) so a loss during the escape doesn't bounce it back into CALIB_LOST_HOLD. _calib_active
        stays True through the escape (the baseline ingest stays frozen)."""
        if self._calib_escape_phase is None:              # ENTRY: pick the push direction from the live ring
            self._calib_escape_phase = "PUSH"
            ring = self._last_ring
            move, tag = None, None
            if self._pushable(self._ring_get(ring, 180.0)):
                move, tag = {"reverse": self.reverse_throttle}, "backward"
            else:
                sides = [(-90.0, self._ring_get(ring, -90.0)), (90.0, self._ring_get(ring, 90.0))]
                pushable = [(rel, c) for rel, c in sides if self._pushable(c)]
                if pushable:
                    rel, _ = max(pushable, key=lambda kv: (float("inf") if kv[1] is None else kv[1]))
                    sign = 1.0 if rel == 90.0 else -1.0
                    move = {"joy_horizontal": sign * self._strafe_mag}
                    tag = "strafe_right" if rel == 90.0 else "strafe_left"
            if move is None:                              # ring boxed all sides -> just hold for SLAM
                self._calib_escape_phase = "HOLD"
                self._slam_fast_streak = 0
                self._player = None
                return {}, "CALIB_ESCAPE", "escape: ring boxed all sides -> HOLD for SLAM (no push)"
            self._player = RecipePlayer([dict(move, duration_s=self.calib_escape_push_s)], name="calib-escape-push")
            return {}, "CALIB_ESCAPE", f"escape push {tag} to a fresh vantage, then HOLD for SLAM+plan OK"
        if self._calib_escape_phase == "PUSH":
            active, done = self._player.fields(now)
            if done:
                self._player = None
                self._calib_escape_phase = "HOLD"
                self._slam_fast_streak = 0                # count the recovery streak FRESH from the hold
                return {}, "CALIB_ESCAPE", (f"escape push done -> HOLD for SLAM+plan OK "
                                            f"({self.calib_escape_ok_frames} fresh fast frames)")
            return active, "CALIB_ESCAPE", None
        # HOLD: wait indefinitely until SLAM's solve is healthy AND the planner is OK — AND (session 22) the
        # healthy-frame latency AVERAGE is comfortable (the escape hold is already the "wait for good SLAM"
        # state, so the stricter bar just extends the same wait; no extra bound needed here).
        if (self._slam_fast_streak >= self.calib_escape_ok_frames and status == "OK"
                and self._calib_slam_comfortable()):
            self._recalibrating = True
            self._calib_retries = 0
            self._calib_escape_phase = None
            self._enter("CALIBRATING_HEIGHT", now)
            return {}, "CALIBRATING_HEIGHT", (f"escape recovered ({self._slam_fast_streak} fresh frames + plan OK"
                                              f" + avg {self._slam_ms_avg:.0f}ms comfortable) "
                                              "-> RETRY height calibration")
        return {}, "CALIB_ESCAPE", None

    def _step_postlude_lost(self, now, plan, status, floor_contact):
        """A plan loss (LOST/NO-PLAN/STALE) during the post-mission ending (RETURN_TO_ORIGIN / ORIENT_HOME /
        DOCK_FLOOR / LOW_STANDOFF). Mirror of _step_calib_lost: release controls and HOLD, watching the SLAM
        pulse; resume the interrupted stage once SLAM solves fast (>= calib_lost_recover_frames fresh frames
        under slam_slow_ms) AND the planner status has caught up (status == OK). A still hold is the safest
        thing to do near the ground, so this NEVER acts on a status other than OK — but a fragile re-lock can
        flicker OK/LOST indefinitely without ever sustaining the full clean streak (the 20260719 ending: 7
        OK/LOST flips in ~1m45s, never once hitting calib_lost_recover_frames). `postlude_recover_budget_s`
        bounds that: once the WHOLE postlude ending has been trying to recover longer than the budget, the
        streak requirement relaxes to 1 fresh frame — but the resume STILL only ever fires on a status=="OK"
        tick, same as always (never blind; forcing a state change while still LOST would just get intercepted
        right back into this same hold by the step()-top POSTLUDE_STATES router). On resume, re-plan the turn
        phase (homing/orient) rather than replay a mid-turn recipe on a cleared player."""
        # ENTRY (first loss during the postlude): remember which stage to resume, release controls, count the
        # pulse FRESH from here (ignore the pre-loss streak so a stale "healthy" reading can't exit immediately).
        if self.state != "POSTLUDE_LOST_HOLD":
            self._postlude_resume = self.state
            self._dock_interrupted = True
            self._player = None
            self._slam_fast_streak = 0
            self._slam_slow_streak = 0
            if self._postlude_t0 is None:      # stamped ONCE, on the very first loss of the whole ending —
                self._postlude_t0 = now        # a later loss/recover cycle does not restart the budget
            self._enter("POSTLUDE_LOST_HOLD", now)
            return {}, "POSTLUDE_LOST_HOLD", (f"plan loss DURING {self._postlude_resume} -> release controls, HOLD; "
                                              "resume the ending once SLAM solves fast AND plan is OK")
        # RECOVER: SLAM healthy AND the planner caught up -> resume the interrupted stage. Reset the turn phase so
        # homing/orient re-aims cleanly (never resume a mid-turn recipe with a cleared _player). The full streak
        # is required normally; past the recovery budget, ANY fresh frame on an OK tick is enough (still never
        # blind — status must be OK either way).
        budget_exhausted = (self._postlude_t0 is not None
                             and (now - self._postlude_t0) >= self.postlude_recover_budget_s)
        required_streak = 1 if budget_exhausted else self.calib_lost_recover_frames
        if self._slam_fast_streak >= required_streak and status == "OK":
            resume = self._postlude_resume or "RETURN_TO_ORIGIN"
            if resume == "RETURN_TO_ORIGIN":
                self._home_phase = "PLAN"
            elif resume == "ORIENT_HOME":
                self._orient_home_phase = "PLAN"
            self._postlude_resume = None
            self._postlude_t0 = None
            self._enter(resume, now)
            tag = " (RECOVERY BUDGET EXHAUSTED — relaxed streak requirement, VISIBLE)" if budget_exhausted else ""
            return {}, resume, (f"postlude recovered ({self._slam_fast_streak} fresh frames + plan OK){tag} "
                                 f"-> resume {resume}")
        return {}, "POSTLUDE_LOST_HOLD", None          # holding; wait for the SLAM pulse + plan OK

    def _begin_fallback(self, now, event):
        """One fallback attempt: a SINGLE recovery-step turn to re-expose a new heading, THEN a short RING-PICKED
        parallax push (the SAME direction pick as normal scouting — backward if pushable, else strafe toward the
        roomier pushable side; NEVER forward, so it can't ram). The turn uses `recovery_turn_step_deg` (default
        15deg — gentler than the normal step so a fragile re-lock survives; still a UNIDIRECTIONAL sweep, so N
        attempts re-expose every heading for RELOC). Push LAST is deliberate: the motion right before the
        inter-attempt SETTLE is then a TRANSLATION (parallax), not a bare rotation (the SLAM-killer), so SLAM
        re-locks on the rescued view — and it matches the "reset attitude with 'c' BEFORE a push" playbook recipe
        (`_turn_steps` = yaw + 'c'). If no direction is pushable, just turn (the rotation alone re-exposes geometry)."""
        self._fallback_attempts += 1
        ring = self._last_ring
        # Direction pick mirrors PARALLAX_PUSH: backward-first (pure translation, camera still on scene), else the
        # roomier pushable side (None = open near-field ranks as most room). Strafe magnitude is now throttled
        # (strafe_throttle) so a recovery scoot is gentle. `_pushable` gates each candidate on the live clearance.
        move, tag = None, "no-push"
        if self._pushable(self._ring_get(ring, 180.0)):
            move, tag = {"reverse": self.reverse_throttle}, "backward"
        else:
            sides = [(-90.0, self._ring_get(ring, -90.0)), (90.0, self._ring_get(ring, 90.0))]
            pushable = [(rel, c) for rel, c in sides if self._pushable(c)]
            if pushable:
                rel, _ = max(pushable, key=lambda kv: (float("inf") if kv[1] is None else kv[1]))
                sign = 1.0 if rel == 90.0 else -1.0
                move = {"joy_horizontal": sign * self._strafe_mag}
                tag = "strafe_right" if rel == 90.0 else "strafe_left"
        theta = self.recovery_turn_step_deg              # gentle unidirectional RELOC sweep step
        # ORDER: turn (yaw + 'c' attitude reset) FIRST, then a rest, then the ring-picked push LAST — so the last
        # motion before the inter-attempt SETTLE is the parallax translation that rescues the rotation for RELOC.
        # (The push direction was picked from the PRE-turn ring; 15deg is small + the push is short/throttled/
        # never-forward, so the ram risk stays low.) The inter-attempt SETTLE (fresh-frame gated) owns the pause
        # after the push — no trailing rest here.
        steps = [*self._turn_steps(theta)]
        if move is not None:
            steps += [{"duration_s": self.rest_between_s}, dict(move, duration_s=self.fallback_retreat_s)]
        self._player = RecipePlayer(steps, name=f"fallback#{self._fallback_attempts}")
        self._enter("FALLBACK", now)
        ev = event or (f"FALLBACK #{self._fallback_attempts}: turn {theta:+.0f} then ring-picked {tag} push "
                       "(recovery sweep — parallax last), then settle")
        return {}, "FALLBACK", ev

    @staticmethod
    def _fmt(be):
        return f"{be:+.1f}" if be is not None else "n/a"

    # ------------------------------------------------- SLAM frame-timing settle gate
    def _update_slam(self, plan):
        """Track SLAM health from the plan's per-frame build time. Count consecutive FRESH frames (dedup on
        frame_id, since the plan republishes on a timer) that came in under slam_slow_ms; a slow frame resets
        the streak. 'Stable' = the streak has exceeded the settle count."""
        ms = plan.get("slam_ms")
        fid = plan.get("frame_id")
        if ms is None or fid == self._slam_frame_id:
            return
        self._slam_frame_id = fid
        self._slam_ms_latest = float(ms)
        self._slam_hist.append((float(ms), plan.get("cap_ts")))   # rolling window for the settle-gate (session 24)
        if ms < self.slam_slow_ms:
            self._slam_fast_streak += 1
            self._slam_slow_streak = 0
            # SLAM-COMFORT barometer (session 22): rolling window of HEALTHY-frame latencies. A calibration
            # only launches/redoes when the average clears calib_slam_avg_ms — "comfortable", not merely alive
            # (the 20260717 redos fired on 6 alive-but-marginal 616-797ms frames and died in every ASCEND).
            self._slam_ms_win.append(float(ms))
        else:
            self._slam_fast_streak = 0
            self._slam_slow_streak += 1

    @property
    def _slam_slow(self):
        """The most recent FRESH frame took too long to build (SLAM choking; its pose is untrustworthy)."""
        return self._slam_ms_latest is not None and self._slam_ms_latest >= self.slam_slow_ms

    @property
    def _slam_ms_avg(self):
        """Rolling average of the last calib_slam_avg_window HEALTHY-frame SLAM latencies (None if empty)."""
        return (sum(self._slam_ms_win) / len(self._slam_ms_win)) if self._slam_ms_win else None

    def _calib_slam_comfortable(self):
        """True when SLAM is COMFORTABLE enough to survive a vertical calibration excursion: the healthy-frame
        latency window is not yet FULL (don't deadlock the early flight — the prelude has barely any history),
        or its average clears calib_slam_avg_ms. Gate for calibration launch/redo/retry (session 22)."""
        if len(self._slam_ms_win) < self.calib_slam_avg_window:
            return True
        return self._slam_ms_avg < self.calib_slam_avg_ms

    # ------------------------------------------------- session 24: settle-gate (SLAM_HOLD -> SETTLE unification)
    def _slam_window_ready(self, since=None, latest_since=None):
        """SLAM FRESHNESS gate: the rolling window (`_slam_hist`, last `settle_fresh_frames` FRESH frames) is
        FULL, every entry built under `slam_slow_ms`, and every entry has a KNOWN capture time (a frame we
        can't timestamp can never count as verified-fresh, prequalified or not — this is NOT merely the
        `since` check below, it applies unconditionally so a cap_ts-less stream can never look "already
        clean"). If `since` is given, every entry must ALSO be captured at/after it (demands brand-new
        post-transition evidence rather than trusting a stale window). `latest_since` is a WEAKER variant:
        only the single MOST RECENT entry must be captured at/after it — lets a settle keep leaning on an
        already-healthy older window while still proving at least one frame arrived after the instant this
        settle is meant to be judging (closes the "prequalified on frames from before the maneuver" gap:
        `since` demands a brand-new 6-frame window, which can add real latency for no benefit when SLAM was
        already healthy; `latest_since` only demands proof of ONE fresh look at the world). A pure
        SLAM-solve-health question, decoupled from how long the airframe has been resting."""
        if len(self._slam_hist) < self._slam_hist.maxlen:
            return False
        if any(ms >= self.slam_slow_ms for ms, _ in self._slam_hist):
            return False
        if any(cap_ts is None for _, cap_ts in self._slam_hist):
            return False
        if since is not None and any(cap_ts < since for _, cap_ts in self._slam_hist):
            return False
        if latest_since is not None and max(cap_ts for _, cap_ts in self._slam_hist) < latest_since:
            return False
        return True

    def _settle_gate_begin(self, now):
        """Open a settle-gate window AT THE TRUE MOMENT the airframe stops moving. A just-finished ADVANCE/
        PARALLAX_PUSH/etc. (Category A) calls this exactly when motion ends. `_enter_slam_hold` (Category
        B/C) calls this at SLAM_HOLD ENTRY -- the real stationary-start instant, NOT at exit -- so elapsed
        time naturally includes however long the hold lasted; no separate 'credit' bookkeeping is needed.
        `_settle_gate_prequalified` snapshots whether the window was ALREADY clean the instant the gate
        opened, so an already-healthy pipeline can pass the freshness gate without demanding brand-new
        frames on top of frames it already has."""
        self._settle_gate_t0 = now
        self._settle_gate_prequalified = self._slam_window_ready(since=None)

    def _settle_gate_poll(self, now, *, require_fresh=True):
        """True once BOTH gates clear: (1) FRESHNESS -- skipped if `require_fresh=False` (the vertical-prelude
        settle-to targets, which keep a plain timer); else prequalified, or the window is clean with every
        entry captured at/after the gate opened. Even when prequalified, the MOST RECENT entry must still be
        captured at/after the gate opened (`latest_since`) — a settle can lean on an already-healthy window,
        but it must never complete having seen literally ZERO frames since the maneuver it's judging finished
        (the 20260719 corner-bounce bug: a `SETTLE` reused a frame captured BEFORE the collision it was
        supposed to be judging, so REPLAN re-aimed off a pose that never updated post-impact). (2) PHYSICAL
        MOTION -- elapsed real time since the gate opened >= settle_gate_s. For a resume from a stationary
        SLAM_HOLD the gate opened at hold ENTRY, so elapsed already covers the whole (typically multi-second)
        hold -- gate 2 clears near-instantly while gate 1 still independently proves current health. For a
        fresh post-motion settle both gates run their full course."""
        fresh_ok = (not require_fresh) or self._slam_window_ready(
            since=None if self._settle_gate_prequalified else self._settle_gate_t0,
            latest_since=self._settle_gate_t0)
        elapsed = 0.0 if self._settle_gate_t0 is None else (now - self._settle_gate_t0)
        return fresh_ok and elapsed >= self.settle_gate_s

    @property
    def _alt_median(self):
        """Median of the rolling flying-height baseline (_mapping_altitude_history) — the reference CALIB_VERIFY
        judges a calibration against; None until the baseline has any samples. For the replay's live numbers."""
        h = self._mapping_altitude_history
        if not h:
            return None
        s = sorted(h); n = len(s)
        return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])

    def _settle_begin(self, now):
        """Start a settle window HERE: only SLAM frames CAPTURED after this instant (cap_ts >= now) count toward
        the gate. The reusable primitive behind the SETTLE state (session 15), the recovery inter-action settle,
        and the postlude stage settles. Stamps the origin + zeroes the post-entry fresh-frame count. (The SETTLE
        state also gets this stamp via `_enter`; recovery/postlude sub-phases call this directly.)"""
        self._settle_t0 = now
        self._settle_ok = 0
        self._settle_last_fid = None

    def _settle_poll(self, now, plan, *, require_fast, min_frames, max_hold_s):
        """Poll a settle window opened by `_settle_begin`. Count each FRESH frame (dedup on frame_id) whose
        CAPTURE time is after the window began (cap_ts >= _settle_t0); when `require_fast`, also demand the solve
        was fast (slam_ms < slam_slow_ms). Returns (done, capped):
          done   -> settled: >= `rest_between_s` elapsed AND >= `min_frames` post-entry frames counted; OR the
                    bounded escape fired (see capped).
          capped -> the window hit `max_hold_s` WITHOUT enough fresh frames (a dead/choked pipeline). Returned as
                    (True, True) so the settle still ENDS, but the caller MUST annunciate it (NO SILENT FALLBACK).
        `require_fast=True, max_hold_s=None` is the HEALTHY-SLAM flavor (SETTLE state / postlude) — status==OK
        stays structurally enforced by the step()-top recovery guard. `require_fast=False` + a finite
        `max_hold_s` is the LOST-SLAM recovery flavor: SLAM is STALE/LOST by definition, so we can't demand
        fast/OK (a genuine re-lock exits recovery at the step() top on the next tick) — we just give it a still
        window verified by fresh capture, bounded so a re-exposure maneuver still follows if the pipeline is dead."""
        fid = plan.get("frame_id")
        if fid is not None and fid != self._settle_last_fid:
            self._settle_last_fid = fid
            cap_ts, ms = plan.get("cap_ts"), plan.get("slam_ms")
            if (self._settle_t0 is not None and cap_ts is not None and cap_ts >= self._settle_t0
                    and (not require_fast or (ms is not None and ms < self.slam_slow_ms))):
                self._settle_ok += 1
        elapsed = None if self._settle_t0 is None else (now - self._settle_t0)
        rest_done = elapsed is not None and elapsed >= self.rest_between_s
        if rest_done and self._settle_ok >= min_frames:
            return True, False
        if max_hold_s is not None and elapsed is not None and elapsed >= max_hold_s:
            return True, True
        return False, False

    def _enter_slam_hold(self, resume, now, why):
        """Hover-hold (zero velocity) until SLAM settles, then re-enter `resume`. Returned by a gate site.
        Stamps the hold start. Does NOT reset `_slam_stepback_count` (a step-back re-enters SLAM_HOLD via
        `_enter` directly, so those persist across step-backs within one hold ANYWAY) — a bad SLAM patch
        typically bounces through PLAN-LOST/HOLD_LOST before the next `_enter_slam_hold`, and resetting
        the counter on every fresh entry (the old behavior) meant the #1/3->#2/3->#3/3 escalation could
        never advance past #1 in exactly that scenario. It resets only in the REPLAN handler, on a
        genuinely trusted recovery or a materially new leg goal — see `_hop_start_goal` nearby there."""
        self._slam_resume = resume
        self._player = None
        self._slam_hold_start = now
        self._settle_gate_begin(now)      # session 24: open the shared gate HERE (the true stationary-start
                                           # instant), so a later resume's motion-gate elapsed time already
                                           # covers the whole hold -- no separate credit bookkeeping needed
        self._enter("SLAM_HOLD", now)
        return {}, "SLAM_HOLD", why

    def _blind_contact_backoff(self, now, wall_contact, backwall_contact, resume_state):
        """Reactive, bounded safety response to a flow-detected wall/backwall contact while BLIND
        (HOLD_LOST, or waiting in SLAM_HOLD before its settle gate clears) — states where the ADVANCE/
        PARALLAX_PUSH clearance/contact checks never run, even though `wall_contact`/`backwall_contact`
        are computed every tick independently of SLAM health (flow_contact_detector.py doesn't need a
        plan or a pose). Confirmed on the 20260718 flight: a drone parked in this exact hold-bounce for
        30-40s drifted into a wall with nothing reacting until the FSM happened to reach ADVANCE again.

        Edge-triggered via `_blind_contact_armed` (disarms on a reaction, re-arms once contact clears) so
        a sustained pin against the wall doesn't replay `back_off` every tick. Plays the SAME `back_off`
        recipe/BACKOFF machinery ADVANCE already uses, then resumes the SAME hold state it interrupted —
        NOT settle/replan, the plan is still untrustworthy. Returns the (active, state, event) tuple to
        return immediately if it reacted, else None (caller continues its normal hold logic).

        Deliberately NOT wired into `_register_bump`/the goals-DB — this is a pure safety reflex during a
        blind hold, independent of which goal (if any) is committed; the strike/bump/loop accounting
        stays exactly the mechanism it is today, judged only from ADVANCE."""
        if not (wall_contact or backwall_contact):
            self._blind_contact_armed = True     # re-armed once clear of the wall
            return None
        if not self._blind_contact_armed:
            return None                          # already reacted to this same, still-ongoing contact
        self._blind_contact_armed = False
        self._blind_backoff_resume = resume_state
        self._player = self.pb.player("back_off")
        self._enter("BLIND_BACKOFF", now)
        kind = "flow BACKWALL" if backwall_contact and not wall_contact else "flow WALL"
        return {}, "BLIND_BACKOFF", (f"{kind} contact while blind in {resume_state} -> back off, "
                                     f"then resume {resume_state}")

    def _enter(self, state, now):
        # Ghost-path guard (D5): the moment a re-locked-but-unconfirmed drone enters a SPATIAL state it physically
        # moves (turn/translate) with logging frozen, so the leftover pre-loss command_history no longer maps to
        # the drone's true pose. Mark it broken -> a secondary SLAM drop clears it and jumps to the ring-picked
        # FALLBACK sweep instead of replaying a displaced ghost path.
        if self._recovering and state in ("ORIENT", "PARALLAX_PUSH", "ADVANCE"):
            self._history_broken = True
        # The post-recovery ADVANCE progress gauge is captured lazily in the ADVANCE handler; drop it when leaving
        # an advance (so the NEXT advance re-measures from its own start), but NOT across a mid-leg SLAM_HOLD.
        if state not in ("ADVANCE", "SLAM_HOLD"):
            self._recovery_adv_start = None
        # Session 20: every ADVANCE entry (a fresh leg OR a resume after a hop-SETTLE / SLAM_HOLD) starts a fresh
        # hop tick count + a FRESH per-hop progress snapshot: clear _hop_start_goal here so the ADVANCE handler
        # re-captures the start distance on its first posed tick (each hop is judged from its OWN start).
        if state == "ADVANCE":
            self._hop_tick = 0
            self._hop_start_goal = None
        # A plan-loss / SLAM-choke hold ABANDONS the pending per-hop eval (an interrupted hop is not a strike —
        # per the rule "wait for OK, then settle, then it's a NEW advance command"). Recovery chains from these.
        if state in ("HOLD_LOST", "SLAM_HOLD"):
            self._hop_start_goal = None
        # SETTLE fresh-frame gate (session 15, legacy fields still used by the LOST-SLAM settle flavor
        # elsewhere): start the post-entry frame count from THIS instant.
        if state == "SETTLE":
            self._settle_t0 = now
            self._settle_ok = 0
            self._settle_last_fid = None
            # Session 24 two-gate settle: a fresh Category-A settle (arriving from active motion) opens a NEW
            # gate window HERE. Arriving from SLAM_HOLD (Category B) is the ONE exception -- that gate was
            # already opened at the hold's TRUE stationary-start instant (_enter_slam_hold), so it must NOT be
            # restamped here (restamping it is exactly the double-wait bug this session fixes).
            if self.state != "SLAM_HOLD":
                self._settle_gate_begin(now)
        # A recovery inter-action settle (REWIND/FALLBACK) is a sub-phase that never spans a real state transition
        # (the hold ticks return the same state without calling _enter), so any actual _enter clears it.
        self._rec_settling = False
        self.state = state
        self.t_state = now

    # ------------------------------------------------- ram guard: self-calibrated speed
    def _finalize_or_discard_calib(self):
        """At a sampling break (SLAM event / leg change) or a full clean sample: accept the mean of the
        collected windowed-speed samples as the nominal free-flight speed IF they span >=
        `ram_calib_min_sample_s`, else discard so calibration restarts on the next continuous ADVANCE run.
        Idempotent once a nominal is set."""
        s = self._calib_samples
        if self._nominal_speed is None and s and (s[-1][0] - s[0][0]) >= self.ram_calib_min_sample_s:
            mean = sum(v for _, v in s) / len(s)
            if mean >= self.ram_calib_min_speed:      # reject a degenerate ~0 nominal (drone was stuck the whole window)
                self._nominal_speed = mean
        self._calib_samples = []
        self._calib_start_t = None

    def _advance_speed(self, now, pos):
        """Rolling live world speed (u/s) for the ram guard + the one-time nominal calibration.
        REAL timestamps (span = now - oldest_t; never a fixed frame rate) so a SLAM frame-rate spike can't
        corrupt the velocity, and the window is PRUNED to `ram_speed_window_s` BEFORE the speed is computed
        so a stale sample left by an interrupted leg can't inflate the denominator into a false slowdown.
        Returns the live speed (or None until the window refills)."""
        win = self._ram_speed_win
        if pos is None:
            self._ram_speed = None
            return None
        # a time gap since the last sample (SLAM hold / leg change) breaks the continuous run
        if win and (now - win[-1][0]) > self.ram_speed_window_s:
            self._finalize_or_discard_calib()      # accept the partial nominal if clean enough, else discard
            win.clear()
            self._ram_accum = 0.0
            self._ram_last_t = None
        win.append((now, float(pos[0]), float(pos[1])))
        cutoff = now - self.ram_speed_window_s
        while len(win) > 1 and win[0][0] < cutoff:   # PRUNE before compute
            win.popleft()
        spd = None
        span = now - win[0][0]
        if len(win) >= 2 and span >= 0.5 * self.ram_speed_window_s and span > 0.0:
            spd = math.hypot(pos[0] - win[0][1], pos[1] - win[0][2]) / span
        self._ram_speed = spd
        # one-time nominal calibration: 1s into the first continuous ADVANCE, sample up to sample_s
        if self._nominal_speed is None:
            if self._calib_start_t is None:
                self._calib_start_t = now
            elapsed = now - self._calib_start_t
            if spd is not None and elapsed >= self.ram_calib_skip_s:
                self._calib_samples.append((now, spd))
                if elapsed >= self.ram_calib_skip_s + self.ram_calib_sample_s:
                    self._finalize_or_discard_calib()   # a full clean sample -> accept the nominal
        return spd

    def _corner_no_blacklist_dist(self, plan):
        """The far-corner exemption distance (session 24): prefer the LIVE room-scaled value published as
        `corner_span_half` (half the known bbox's largest corner-to-corner diagonal — computed by
        perception_worker.py from ground_grid.bbox_corners) over the static config default. A fixed 1.0u
        exemption is a guess at one particular room's scale; half the room's OWN diagonal scales with it
        automatically. Falls back to the config default before any corners are known, or when perception
        published a degenerate value (fewer than 2 corners -> no meaningful diagonal, guarded upstream by
        never publishing a non-positive corner_span_half)."""
        span_half = plan.get("corner_span_half")
        return float(span_half) if span_half is not None else self.corner_no_blacklist_dist

    def _corner_giveup_tick(self, goal):
        """Persistent per-corner give-up counter (session 24): tracks EVERY corner the far-corner guard has
        ever suppressed a bump against, keyed by proximity (not a single reset-on-switch slot) so the planner
        oscillating between two unreachable corners can't defeat the cap by resetting a shared counter back to
        zero on every switch. Returns the running count for `goal`'s corner after this tick."""
        g = [float(goal[0]), float(goal[1])]
        for e in self._corner_giveup_counts:
            if self._dist(e["goal"], g) <= self.calib_goal_change_dist:   # "materially the same point"
                e["count"] += 1
                return e["count"]
        self._corner_giveup_counts.append({"goal": g, "count": 1})
        return 1

    # ------------------------------------------------- 2-bump blacklist latch (kinematic)
    def _register_bump(self, plan, reason="advance-blocked"):
        """Latch-gated bump for the event-driven 2-bump blacklist: on an advance-blocked stop (flow WALL /
        ram-guard / clearance stand-off) toward the committed goal, stash ONE bump pulse for run_explore to
        publish, then DISARM + record the stop position as the re-arm anchor. Suppressed while already
        disarmed (a stuttering state machine can't multiply-count one continuous contact) — a suppressed real
        contact stashes a MISSED-BUMP marker (`self._missed_bump`) so run_explore can log the un-counted hit.

        FAR-CORNER GUARD (session 20): a sweep-tour CORNER goal that is still farther than
        `corner_no_blacklist_dist` from the drone is NEVER bumped — so a mildly-stuck-then-freed drone can't
        blacklist a distant corner it simply hasn't reached yet (the pulse never reaches note_wall_hit, which
        blocks BOTH the region blacklist and the corner retirement). Frontiers and near corners bump normally."""
        if self.leg_goal is None:
            return
        if self._leg_is_corner:
            d = self._dist(plan.get("pos"), self.leg_goal)
            no_bl_dist = self._corner_no_blacklist_dist(plan)
            if d is not None and d > no_bl_dist:
                count = self._corner_giveup_tick(self.leg_goal)
                if count >= self.corner_giveup_limit:
                    # Bounded escalation (operator ask): the exemption is not infinite. corner_giveup_limit
                    # give-ups against the SAME corner (still never once close enough for a real 2-bump) means
                    # the drone is almost certainly physically stuck near it -- force-retire the corner (mark
                    # visited, tour moves on to the next unvisited one) instead of exempting it forever.
                    self._corner_giveup_pulse = list(self.leg_goal)
                    self._missed_bump = (f"{reason} (FAR-CORNER guard EXPIRED — corner {self.leg_goal} still "
                                         f"{d:.2f}u away after {count} give-ups >= corner_giveup_limit "
                                         f"{self.corner_giveup_limit} -> force-retiring it)")
                    return
                self._missed_bump = (f"{reason} (FAR-CORNER guard — corner {self.leg_goal} is {d:.2f}u away "
                                     f"> {no_bl_dist:.2f}u; {count}/{self.corner_giveup_limit}; "
                                     "not blacklisting a far corner)")
                return
        if not self._bump_armed:
            self._missed_bump = f"{reason} (latch disarmed — drone hasn't disengaged since the last bump)"
            return
        self._bump_pulse = list(self.leg_goal)
        self._bump_reason = reason
        self._bump_is_corner = bool(self._leg_is_corner)   # reaching here means a NEAR corner (far ones
        self._bump_armed = False                           # returned above) -- evidence for the goals-DB
        pos = plan.get("pos")
        self._last_bump_anchor = list(pos) if pos is not None else None

    def take_missed_bump(self):
        """Pop the pending MISSED-BUMP marker (a real contact that emitted no pulse), or None."""
        m, self._missed_bump = self._missed_bump, None
        return m

    def take_notice(self):
        """Pop the pending one-shot operator NOTICE (e.g. the session-22 height-reference disagreement
        warning), or None. run_explore prints + diag-logs it (VISIBLE telemetry, no silent state)."""
        n, self._pending_notice = self._pending_notice, None
        return n

    def take_hop_baseline_msg(self):
        """Pop the pending [HOP_BASELINE] diagnostic (the pose/cap_ts a hop's start was bound against),
        or None. run_explore prints + diag-logs it."""
        m, self._hop_baseline_msg = self._hop_baseline_msg, None
        return m

    def take_hop_judge_msg(self):
        """Pop the pending [HOP_JUDGE] diagnostic (the pose/cap_ts + verdict a hop was judged with),
        or None. run_explore prints + diag-logs it."""
        m, self._hop_judge_msg = self._hop_judge_msg, None
        return m

    def rearm_bump_if_disengaged(self, active, plan):
        """Re-arm the bump latch once the drone has DISENGAGED from the last bump anchor — EITHER a backward
        control vector is actively published (retreat) OR it has moved > goal_reach_dist from the anchor.
        SLAM-freeze-safe: a frozen pose stalls displacement at 0, so a jammed drone never falsely re-arms."""
        if self._bump_armed or self._last_bump_anchor is None:
            return
        moved = self._dist(plan.get("pos"), self._last_bump_anchor)
        backward = float((active or {}).get("reverse", 0.0) or 0.0) > 0.0
        if backward or (moved is not None and moved > self.goal_reach_dist):
            self._bump_armed = True

    def take_bump_pulse(self):
        """Pop the pending (bump goal, reason, pos, is_corner) or (None, None, None, None). run_explore
        publishes it on TOPIC_AUTOPILOT_EVENT and logs the reason (which advance-blocked stop fired the
        bump). `pos` (the bump-time drone position, `_last_bump_anchor`) and `is_corner` ride along as
        goals-DB evidence — NOT cleared here, since `_last_bump_anchor` is still needed for the re-arm
        gauge (`rearm_bump_if_disengaged`)."""
        g, r, ic = self._bump_pulse, self._bump_reason, self._bump_is_corner
        pos = list(self._last_bump_anchor) if self._last_bump_anchor is not None else None
        self._bump_pulse = self._bump_reason = self._bump_is_corner = None
        return g, r, pos, ic

    def take_corner_giveup_pulse(self):
        """Pop the pending corner [x,z] that just hit `corner_giveup_limit` far-corner give-ups (or None).
        run_explore publishes it on TOPIC_AUTOPILOT_EVENT; perception feeds it to
        planner.force_retire_corner (mark visited, tour moves on -- never blacklists/ends the mission by
        itself; see the REPLAN `done` branch for the all-corners-exhausted ending)."""
        g, self._corner_giveup_pulse = self._corner_giveup_pulse, None
        return g

    def take_pick_pulse(self):
        """Pop the pending goals-DB pick+hop-outcome dict (or None) stashed at a REPLAN leg-commit. run_explore
        publishes it on TOPIC_AUTOPILOT_EVENT; perception feeds it to planner.register_hop_outcome (previous
        hop's STRIKE/progress) + planner.register_goal_pick (this leg's pick -> ping-pong loop guard)."""
        p, self._pick_pulse = self._pick_pulse, None
        return p

    @staticmethod
    def _dist(a, b):
        if a is None or b is None:
            return None
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def _ring_get(ring, rel_deg):
        """Clearance from the published ring (list of [rel_deg, dist]) at the heading offset nearest
        `rel_deg` (wrapped). Returns the distance (SLAM units) or None if the ring is empty/that dir
        unmapped. Forward = 0, backward = 180."""
        if not ring:
            return None
        best, best_diff = None, 1e9
        for r, d in ring:
            diff = abs(((rel_deg - r + 180.0) % 360.0) - 180.0)
            if diff < best_diff:
                best_diff, best = diff, d
        return best

    def _pushable(self, c):
        """Is a ring clearance `c` (SLAM units, or None) roomy enough to translate a short parallax scoot?
        None = nothing mapped within the ring's NEAR-FIELD range -> treat as OPEN (room). A finite hit is
        pushable only if it clears the push itself (parallax_min_clear = push_dist + buffer). Miss-is-room is
        an explicit operator decision (worst case we bump; the flow WALL detector + 2-bump blacklist recover)."""
        return c is None or c >= self.parallax_min_clear

    def _pick_ring_direction(self, ring, plan, force_no_backward=False):
        """PARALLAX_PUSH direction pick: backward-first (ideal parallax), else the roomier pushable side
        (D2 scrape guard may reposition forward first), else give up. Shared by the entry-tick pick AND the
        mid-push retry after a backward push proves blocked (`force_no_backward=True` there — this episode
        already showed backward is bad regardless of what the ring says).

        Also consults/sets/clears the cross-episode `_parallax_back_blocked` give-up latch: a full give-up
        (backward excluded/blocked AND both sides blocked) latches the drone's position so the NEXT pick
        (next leg's re-ORIENT, or a fresh PARALLAX_PUSH) doesn't immediately retry backward at the same spot
        just because the ring's "open" reading hasn't changed (SLAM still hasn't mapped what's behind).
        Cleared once the drone has moved `parallax_min_clear` away from the anchor — SLAM-freeze-safe
        (`_dist` returns None on a missing/frozen pose, which keeps the latch set), mirroring the
        `rearm_bump_if_disengaged` anchor-distance pattern.

        Returns (push_dir, after_reposition, event): push_dir is "backward" | "strafe_left" | "strafe_right"
        | "reposition_fwd" | None (give up — caller settles/replans); after_reposition is the queued strafe
        direction when push_dir == "reposition_fwd", else None; event is an informational string or None.
        """
        allow_backward = not force_no_backward
        if allow_backward and self._parallax_back_blocked:
            moved = self._dist(plan.get("pos"), self._parallax_back_blocked_anchor)
            if moved is not None and moved > self.parallax_min_clear:
                self._parallax_back_blocked = False
                self._parallax_back_blocked_anchor = None
            else:
                allow_backward = False
        if allow_backward and self._pushable(self._ring_get(ring, 180.0)):
            return "backward", None, None
        sides = [(-90.0, self._ring_get(ring, -90.0)), (90.0, self._ring_get(ring, 90.0))]
        pushable = [(rel, c) for rel, c in sides if self._pushable(c)]
        if pushable:                 # None (open near-field) ranks as most room
            rel, _ = max(pushable, key=lambda kv: (float("inf") if kv[1] is None else kv[1]))
            strafe_dir = "strafe_right" if rel == 90.0 else "strafe_left"
            # D2 SCRAPE GUARD: strafing while pinned VERY close behind (possibly yawed) can drive the flank
            # into the wall -> scrape -> spin -> SLAM death (flight 20260713). If forward is CLEARLY open
            # (forward raycast, reliable forward), reposition forward out of the corner FIRST, then strafe
            # from safer space. Otherwise strafe as before (throttled by D1).
            back_c = self._ring_get(ring, 180.0)
            fwd_clr = plan.get("forward_clearance_dist")
            if (back_c is not None and back_c < self.strafe_backwall_danger_dist
                    and fwd_clr is not None and fwd_clr > self.strafe_reposition_min_fwd):
                event = (f"parallax {strafe_dir} but pinned behind (back {back_c:.2f} < "
                         f"{self.strafe_backwall_danger_dist:g}) & fwd {fwd_clr:.2f} open -> reposition "
                         f"forward {self.strafe_reposition_fwd_s:g}s first, then strafe")
                return "reposition_fwd", strafe_dir, event
            return strafe_dir, None, None
        # give up: backward excluded/blocked and both sides blocked too
        self._parallax_back_blocked = True
        self._parallax_back_blocked_anchor = plan.get("pos")
        return None, None, None

    def step(self, now, plan, wall_contact, ceiling_contact=False, floor_contact=False,
             backwall_contact=False, status="OK"):
        event = None
        active = {}
        st = self.state
        # `_trimming` is telemetry only (true while a TRIM runs). Defensively clear it whenever we are not in
        # TRIM so a trim abandoned by a mid-trim SLAM-loss recovery can't leave the flag stuck True.
        if st != "TRIM" and self._trimming:
            self._trimming = False
        # Altitude lock: cache the hold target once, from the first valid pose after the prelude (lazy, so a
        # stale pose at the transition just defers it). Persists across reset_leg (flight-level reference).
        # NB: never (re-)cache in the descent postlude — DOCK_FLOOR clears the target on purpose, and a re-cache
        # here would re-inflate a floor-level drone straight back toward flying height (the land/crawl/jump loop).
        if (self.altitude_lock and self.airborne_done and self.target_altitude_y is None
                and st not in _POSTLUDE_NOLOCK
                and plan.get("plan_valid") and plan.get("pos_y") is not None):
            self.target_altitude_y = float(plan["pos_y"])
        # Record the TAKE-OFF heading once: the first healthy SLAM heading after the prelude completes. General
        # (whatever heading the drone armed at — not a room answer); ORIENT_HOME faces it before the final dock.
        if (self.airborne_done and self._takeoff_heading is None
                and plan.get("plan_valid") and plan.get("heading_deg") is not None and not self._slam_slow):
            self._takeoff_heading = float(plan["heading_deg"])
        self._update_slam(plan)   # track SLAM frame-build time for the settle gate (below + at the gate sites)
        if plan.get("plan_valid"):
            self._ever_tracked = True   # SLAM has tracked at least once -> a later empty-history STALE is a real loss, not warmup
        # Continuous rolling baseline of NORMAL flying altitude (the median CALIB_VERIFY judges against + the
        # debugger's live drone-height number). Session 18: measure only AFTER the first calibration reports
        # height-OK (`_height_calibrated`), NEVER during a calibration (_calib_active freeze), at healthy SLAM,
        # and append exactly ONE reading per FRESH SLAM frame (dedup by frame_id) — so the median tracks real
        # poses instead of ~25 per-tick re-appends of one stale pose. +Y DOWN (a lower drone = a larger pos_y).
        if (self._height_calibrated and not self._calib_active
                and plan.get("plan_valid") and plan.get("pos_y") is not None and not self._slam_slow):
            _alt_fid = plan.get("frame_id")
            if _alt_fid is not None and _alt_fid != self._last_alt_frame_id:
                self._last_alt_frame_id = _alt_fid
                self._mapping_altitude_history.append(float(plan["pos_y"]))
                # Session-22 reference-disagreement backstop (VISIBLE, display-only — the median is demoted to
                # telemetry now that desired_y is THE fixed reference): if the rolling median wanders more than
                # one delta from desired_y, either the drone spent long off-height (TRIM should be correcting)
                # or — if TRIM reports being ON-height — SLAM's Y actually drifted. Warn LOUDLY once per flight.
                if (not self._height_drift_warned and self._desired_y is not None
                        and self._trim_delta):
                    _med = self._alt_median
                    if _med is not None and abs(_med - self._desired_y) > abs(self._trim_delta):
                        self._height_drift_warned = True
                        self._pending_notice = (
                            f"HEIGHT-REFERENCE DISAGREEMENT: flying-height median {_med:+.3f} is "
                            f"{abs(_med - self._desired_y):.3f}u (> delta {abs(self._trim_delta):.3f}) from "
                            f"desired_y {self._desired_y:+.3f} — long off-height flight, or SLAM Y drift "
                            f"(check the Y-DRIFT audit / HEIGHT panel)")

        # --- status-gated SLAM-loss recovery (CONTROL-SPACE); active only in the explore phase ---
        if self._explore_started:
            lost = status in ("PLAN-LOST", "NO-PLAN", "PLAN-STALE")
            # A plan loss DURING a height calibration must NOT drop us into the normal recovery (which forgets
            # the calibration and leaves the drone glued near the ceiling). Latch "interrupted", release
            # controls, and hold in a DEDICATED state; on recovery REDO the calibration. Covers LOST/NO-PLAN/
            # STALE. `st == CALIB_LOST_HOLD` routes EVERY status (incl. OK) into the handler so it owns its own
            # recovery exit — and that exit is gated on status == OK to beat the level-triggered status flicker.
            # CALIB_ESCAPE owns EVERY status too (it deliberately holds through a loss while re-localizing) —
            # check it FIRST, before the calib-lost divert, so `(lost and _calib_active)` can't hijack it.
            if st == "CALIB_ESCAPE":
                return self._step_calib_escape(now, status)
            # BLIND_BACKOFF (the reactive wall/backwall response fired from HOLD_LOST/SLAM_HOLD, see
            # _blind_contact_backoff) must likewise own EVERY status while it plays: it typically starts
            # WHILE status is still LOST/STALE (that's the whole point — reacting despite being blind), so
            # without this it would be swept straight back into a fresh HOLD_LOST after a single tick,
            # abandoning the back_off recipe before it ever moved. Its own completion returns state to the
            # SAME hold it interrupted, where the normal status handling below resumes as usual.
            if st == "BLIND_BACKOFF":
                active, bdone = self._player.fields(now)
                if not bdone:
                    return active, "BLIND_BACKOFF", None
                self._player = None
                resume = self._blind_backoff_resume or "HOLD_LOST"
                self._blind_backoff_resume = None
                if resume == "SLAM_HOLD":
                    return self._enter_slam_hold(self._slam_resume, now,
                                                 "blind back-off done -> resume waiting for SLAM to settle")
                self._enter(resume, now)
                return {}, resume, f"blind back-off done -> resume {resume}"
            if st == "CALIB_LOST_HOLD" or (lost and self._calib_active):
                return self._step_calib_lost(now, status)
            # A plan loss DURING the post-mission ending must NOT drop into the generic HOLD_LOST/FALLBACK recovery
            # (which abandons the homing/dock and thrashes — the flight-20260713 ending). Divert to a dedicated
            # HOLD that resumes the interrupted postlude stage once SLAM+plan recover. Owns EVERY status once
            # entered (like CALIB_LOST_HOLD), so its OK-gated exit beats the status flicker.
            if st == "POSTLUDE_LOST_HOLD" or (lost and st in POSTLUDE_STATES):
                return self._step_postlude_lost(now, plan, status, floor_contact)
            if status in ("PLAN-LOST", "NO-PLAN"):
                # Perception is SILENT. HARD HOVER-HOLD indefinitely — never move on a clock while we're
                # blind. Wait for perception to speak; the branch below (OK/STALE) then decides.
                if st != "HOLD_LOST":
                    self._player = None
                    self._enter("HOLD_LOST", now)
                    return {}, "HOLD_LOST", ("PLAN-LOST -> HARD HOVER-HOLD (indefinite; waiting for "
                                             "perception, no blind recovery)")
                # "Hard hover" is not "ignore a wall we're touching" — the flow contact detector runs
                # independently of SLAM, so react to it even while blind (see _blind_contact_backoff).
                reaction = self._blind_contact_backoff(now, wall_contact, backwall_contact, "HOLD_LOST")
                if reaction is not None:
                    return reaction
                return {}, "HOLD_LOST", None
            if status == "PLAN-STALE":
                # Perception is publishing but SLAM is not TRACKING -> retrace to re-expose keyframes.
                return self._step_stale(now, plan, wall_contact)
            # status OK: if we were recovering, perception is TRACKING again -> DON'T fly on the first frame
            # back (a fresh RELOC pose is shaky). Hold until SLAM settles (>N fast frames), THEN brake + REPLAN.
            # NOTE: `_recovering` + the give-up counter are NOT cleared here — a bare OK is not yet trusted; only a
            # confirming ADVANCE (>= recovery_confirm_dist, in the ADVANCE handler) restores trust (D5).
            # Session 24: a corner-giveup-terminal STUCK (see the REPLAN `done` branch) must NOT be swept into
            # this generic "recovery -> OK -> settle -> replan" convergence -- it has nothing to recover INTO
            # (every corner is exhausted); the drone is meant to hold here for good, not bounce back out the
            # instant status reads OK (which it does continuously while just sitting still).
            if st in _RECOVERY_STATES and not (st == "STUCK" and self._corner_giveup_stuck):
                self._settle_to = "REPLAN"
                return self._enter_slam_hold("SETTLE", now,
                                             "plan OK -> wait for SLAM to settle -> brake -> replan "
                                             "(re-locked; NOT trusted until a >=1u ADVANCE confirms)")

        # SLAM settle gate (session 24): hover until BOTH the freshness + physical-motion gates clear (see
        # _settle_gate_poll), then resume the deferred state. Covers every resume target uniformly -- "SETTLE"
        # (the gate stays open across the hop, so SETTLE's own poll below almost always passes instantly),
        # "ADVANCE"/"PARALLAX_PUSH" (previously resumed with NO gate at all; now get the same two-gate check).
        if st == "SLAM_HOLD":
            if self._settle_gate_poll(now):
                nxt = self._slam_resume or "REPLAN"
                self._slam_resume = None
                waited = now - (self._slam_hold_start if self._slam_hold_start is not None else self.t_state)
                self._enter(nxt, now)
                return {}, nxt, (f"SLAM settled after {waited:.1f}s ({self._slam_fast_streak} fast frames, "
                                 f"last {self._slam_ms_latest:.0f}ms) -> resume {nxt}")
            # Still waiting on the settle gate — blind, same as HOLD_LOST. React to a live wall/backwall
            # contact the same way (see _blind_contact_backoff) before falling through to the step-back logic.
            reaction = self._blind_contact_backoff(now, wall_contact, backwall_contact, "SLAM_HOLD")
            if reaction is not None:
                return reaction
            # SLAM is still choking. If it has been slow for a sustained run (and the plan is OK — LOST/STALE
            # are handled at the step() top), step one entry back through the rewind queue to re-expose
            # known-good geometry so the solve can re-lock. Re-arm needs another full run of slow frames.
            # SKIP while `_recovering`: the history is frozen/possibly spatially stale during an untrusted re-lock,
            # so popping it for a step-back could fly a ghost path — just keep holding until a confirming ADVANCE.
            if self._slam_slow_streak >= self.slam_stepback_after_frames and not self._recovering:
                waited = now - (self._slam_hold_start if self._slam_hold_start is not None else self.t_state)
                if self._slam_stepback_count >= self.slam_stepback_max_steps:
                    self._slam_slow_streak = 0            # stop re-checking every frame; keep holding (visible)
                    return {}, "SLAM_HOLD", (f"SLAM still slow after {waited:.1f}s and "
                                             f"{self.slam_stepback_max_steps} step-backs -> keep holding")
                steps = self._pop_stepback()
                if steps is None:
                    self._slam_slow_streak = 0
                    return {}, "SLAM_HOLD", (f"SLAM slow {waited:.1f}s but rewind queue empty -> keep holding")
                self._slam_stepback_count += 1
                self._slam_slow_streak = 0
                self._player = RecipePlayer(steps, name=f"slam-stepback#{self._slam_stepback_count}")
                self._enter("SLAM_STEPBACK", now)
                return {}, "SLAM_STEPBACK", (
                    f"SLAM still slow {waited:.1f}s ({self._slam_ms_latest:.0f}ms) -> REWIND step-back "
                    f"#{self._slam_stepback_count}/{self.slam_stepback_max_steps} to re-expose geometry")
            return {}, "SLAM_HOLD", None

        # One rewind step-back: play the single inverse maneuver, then return to SLAM_HOLD to keep waiting
        # for the solve to settle (the step-back count + hold timer persist across this).
        if st == "SLAM_STEPBACK":
            active, done = self._player.fields(now)
            if done:
                self._player = None
                self._enter("SLAM_HOLD", now)
                return {}, "SLAM_HOLD", "step-back done -> hold for SLAM to settle"
            return active, "SLAM_STEPBACK", None

        # --- GRADUAL HEIGHT TRIM trigger (session 14; BIDIRECTIONAL session 22) ---
        # On a FRESH HEALTHY frame, in a whitelisted settled/travel state, hold the FIRST calibration's
        # desired_y (SLAM height is stable within a flight — confirmed on 20260717_004418) with a gradual
        # PITCH-aim + forward TRIM in EITHER direction:
        #   too LOW  (sagged):  pos_y > ceiling_y + trim_sag_ratio*delta  -> TRIM UP (pitch aim up + push)
        #   too HIGH (glued near the ceiling — an interrupted calibration / wall-climb leaves the drone there
        #            and NOTHING else can bring it down): pos_y < desired_y - trim_high_ratio*delta -> TRIM DOWN
        # Whitelist = {SETTLE, ADVANCE}. Suppressed during any calibration and while already trimming.
        # review-B: every reference is None-guarded — the refs stay None until the first CALIB_VERIFY PASS,
        # so TRIM cannot fire (or crash on a float>None) pre-calibration.
        if (self.trim_enable and st in _TRIM_TRIGGER_STATES and not self._calib_active
                and self._trim_delta is not None and self._ceiling_y is not None
                and self._desired_y is not None
                and plan.get("plan_valid") and plan.get("pos_y") is not None and not self._slam_slow):
            _y = float(plan["pos_y"])
            trim_dir = None
            if _y > self._ceiling_y + self.trim_sag_ratio * self._trim_delta:
                trim_dir = "UP"                          # sagged low -> climb back
            elif _y < self._desired_y - self.trim_high_ratio * self._trim_delta:
                trim_dir = "DOWN"                        # glued high (near the ceiling) -> descend back
            if trim_dir is not None:
                # Snapshot the committed goal (Trap B) so TRIM re-aims at the SAME goal on exit — the trim
                # must not pollute goal commitment. Clear any leftover maneuver player from the interrupted
                # state, AND the pending per-hop progress eval (session 20b): a trim-interrupted hop moves the
                # drone off its measured line, so judging it could strike a goal falsely — it is NOT judged.
                self._trim_dir = trim_dir
                self._trim_resume_goal = list(self.leg_goal) if self.leg_goal is not None else None
                self._trim_phase = None
                self._trim_sag_y = _y
                self._player = None
                self._hop_start_dist = None
                self._hop_start_goal = None
                self._enter("TRIM", now)
                st = "TRIM"          # route into the TRIM handler below this tick

        if st == "ARM":
            if self._player is None:
                self._player = self.pb.player("arm")
            active, adone = self._player.fields(now)
            if adone:
                self._player = None
                self._settle_to = "TAKEOFF"          # rest_between settle, then take off
                self._enter("SETTLE", now)
                event = "armed -> settle -> takeoff"

        elif st == "TAKEOFF":
            if self._player is None:
                self._player = self.pb.player("takeoff")
            active, tdone = self._player.fields(now)
            if tdone:
                self._player = None
                self.airborne_done = True            # prelude past takeoff: never re-arm on a later reset
                if self.ascend_to_ceiling:
                    self._calib_active = True         # FREEZE the mapping-altitude baseline through the prelude ascend->verify
                self._settle_to = "ASCEND" if self.ascend_to_ceiling else "REPLAN"
                self._enter("SETTLE", now)
                event = "airborne -> settle -> " + ("ascend to ceiling" if self.ascend_to_ceiling else "explore")

        elif st == "CALIBRATING_HEIGHT":
            # Per-goal height re-calibration marker (item 1): re-run the SAME two-phase ASCEND->DESCEND to
            # re-tap the ceiling + re-latch target_altitude_y; DESCEND then routes back to REPLAN (which
            # orients to the already-committed goal). A distinct state so the re-tap is visible in the timeline.
            # CLEAR the maneuver player (as the prelude's TAKEOFF does before ASCEND) — otherwise a spent
            # player from the interrupted leg leaks into DESCEND, whose `if _player is None` guard then skips
            # loading the descend recipe and the drone never pushes back down off the ceiling.
            self._player = None
            self._ascend_phase = None
            self._calib_active = True             # FREEZE the mapping-altitude baseline through this re-tap -> CALIB_VERIFY
            self._enter("ASCEND", now)
            event = "CALIBRATING_HEIGHT -> re-tap ceiling (two-phase ascent)"

        elif st == "ASCEND":
            # TWO-PHASE HYBRID ASCENT (gentle, SLAM-metered) — a long continuous climb builds too much
            # vertical momentum before the ceiling and smashes SLAM. Instead:
            #   Phase 1 (micro-pulse approach): short UP pulses separated by rests. After each rest read
            #     the live SLAM altitude gain dZ = prev_y - cur_y (+Y is DOWN so a RISING drone's pos_y
            #     DECREASES). Keep pulsing while still climbing (dZ > eps). These 0.3s taps are too short
            #     to ever latch the flow detector (its episode resets each command change) — by design.
            #   Phase 2 (flow latch): once the gain flattens (dZ <= eps for `ascend_stall_cycles`), the
            #     drone is flush at the ceiling with near-zero momentum -> a single CONTINUOUS UP hold,
            #     long enough (> arm_blank_s + contact_seconds) to latch a CLEAN, low-velocity CEILING.
            if self._ascend_phase is None:                 # lazy init on entry
                self._ascend_phase, self._ascend_phase_t0 = "PULSE", now
                self._ascend_prev_y, self._ascend_stall_count = None, 0
                self._ascend_start_t = now
            if (now - self._ascend_start_t) > self.ascend_max_s:
                # Safety cap: never found a ceiling latch. NO SILENT FALLBACK — log + go descend anyway.
                if self._ascend_prev_y is not None:   # best ceiling estimate = the climb peak (session-14 TRIM ref)
                    self._ceiling_y = float(self._ascend_prev_y)
                self._ascend_phase = None
                self._last_calib_t = now          # reset the re-calibration cooldown even without a clean tap
                self._settle_to = "DESCEND"
                self._enter("SETTLE", now)
                event = f"ascend cap ({self.ascend_max_s}s, no ceiling latch) -> settle -> descend a bit"
            elif self._ascend_phase == "LATCH":
                active = dict(self.ascend_preset)          # continuous UP; flow CEILING detector is authoritative
                y = plan.get("pos_y") if plan.get("plan_valid") else None
                if ceiling_contact:
                    # A clean ceiling latch. The session-11 fix judges the RESULT of the whole re-tap AFTER the
                    # descend (CALIB_VERIFY) against the flying-height baseline — no ascend-time low-object
                    # reject here anymore (too few taps to know "normal ceiling"; a low tap that sinks the drone
                    # is caught by CALIB_VERIFY -> ASCEND_ESCAPE -> CALIB_TRANSLATE -> re-run).
                    self._last_calib_t = now
                    # Record the glued-to-ceiling height for the TRIM references (session 14). At a clean latch
                    # the drone is flush at the ceiling = the climb peak; fall back to the last sampled y.
                    if y is not None:
                        self._ceiling_y = float(y)
                    elif self._ascend_prev_y is not None:
                        self._ceiling_y = float(self._ascend_prev_y)
                    self._ascend_phase = None
                    self._settle_to = "DESCEND"
                    self._enter("SETTLE", now)
                    event = "CEILING latched (flush, low-velocity) -> settle -> descend a bit"
                elif (y is not None and self._ascend_prev_y is not None
                      and (self._ascend_prev_y - y) > self.ascend_gain_eps):
                    # Still climbing during the hold -> the Phase-1 stall was spurious -> resume micro-pulses.
                    self._ascend_phase, self._ascend_phase_t0 = "PULSE", now
                    self._ascend_stall_count, self._ascend_prev_y = 0, y
                    event = "ascend LATCH but still climbing (spurious stall) -> back to micro-pulses"
                elif (now - self._ascend_phase_t0) >= self.ascend_latch_hold_s:
                    # Hold elapsed with no flow latch and no renewed climb -> demonstrably stalled at the top.
                    if y is not None:                 # ceiling estimate for the TRIM references (session 14)
                        self._ceiling_y = float(y)
                    elif self._ascend_prev_y is not None:
                        self._ceiling_y = float(self._ascend_prev_y)
                    self._ascend_phase = None
                    self._last_calib_t = now          # reset the re-calibration cooldown even without a clean tap
                    self._settle_to = "DESCEND"
                    self._enter("SETTLE", now)
                    event = "ascend LATCH hold elapsed, no flow latch (stalled at top) -> settle -> descend"
            elif self._ascend_phase == "PULSE":
                active = dict(self.ascend_preset)          # a short UP micro-pulse (near-zero momentum)
                if (now - self._ascend_phase_t0) >= self.ascend_micro_pulse_s:
                    self._ascend_phase, self._ascend_phase_t0 = "REST", now
            else:   # REST: neutral (momentum bleeds); at the end, sample the SLAM altitude gain this cycle
                if (now - self._ascend_phase_t0) >= self.ascend_rest_s:
                    valid = plan.get("plan_valid") and plan.get("pos_y") is not None and not self._slam_slow
                    if not valid:
                        # No trustworthy pose -> PAUSE (hold, don't guess); ascend_max_s is the backstop.
                        self._ascend_phase_t0 = now
                        event = "ascend: pose invalid/slow -> pause (hold) until SLAM recovers"
                    else:
                        y = float(plan["pos_y"])
                        dz = None if self._ascend_prev_y is None else (self._ascend_prev_y - y)
                        self._ascend_prev_y = y
                        if dz is not None and dz <= self.ascend_gain_eps:
                            self._ascend_stall_count += 1
                        else:
                            self._ascend_stall_count = 0
                        if self._ascend_stall_count >= self.ascend_stall_cycles:
                            self._ascend_phase, self._ascend_phase_t0 = "LATCH", now
                            event = (f"ascend: height gain flattened (dZ<={self.ascend_gain_eps}) "
                                     f"x{self._ascend_stall_count} -> Phase 2 continuous latch hold")
                        else:
                            self._ascend_phase, self._ascend_phase_t0 = "PULSE", now

        elif st == "DESCEND":
            # Brief DOWN nudge (playbook "descend" recipe — tune its duration in flight_playbook.json)
            # so we sit a little below the ceiling while mapping.
            if self._player is None:
                self._player = self.pb.player("descend")
                self._descend_issue_t = now       # settlement-gate origin for CALIB_VERIFY (frame CAPTURED >= this + gate_s)
            active, ddone = self._player.fields(now)
            if ddone:
                self._player = None
                # Session-11: ALWAYS route through CALIB_VERIFY to JUDGE the calibration's settled result
                # against the frozen flying-height baseline before resuming. Carry where a PASS goes: per-goal
                # re-calib -> REPLAN (orients to the committed goal); prelude -> BASELINE_NUDGE (seed the SLAM
                # baseline) unless already seeded. _recalibrating / _calib_active stay set until CALIB_VERIFY
                # resolves. (We still never re-latch target_altitude_y — the descend already reset the physical
                # altitude; re-latching at the ceiling would glue the altitude lock UP into it.)
                if self._recalibrating:
                    self._settle_to = "REPLAN"
                else:
                    self._settle_to = "REPLAN" if self._baseline_seeded else "BASELINE_NUDGE"
                self._enter("CALIB_VERIFY", now)
                event = "dropped a bit -> CALIB_VERIFY (judge the settled height vs the flying-height baseline)"

        elif st == "CALIB_VERIFY":
            # THE session-11 core fix. Post-descend, HOLD NEUTRAL (no vertical command) so the TRUE settled
            # altitude is observable, wait a settlement gate on the plumbed camera-capture timestamp (dynamics
            # settled + latency backlog cleared), then compare the settled pos_y to the FROZEN rolling median of
            # normal flying altitude. Significantly lower (+Y DOWN => a LARGER pos_y) => the calibration SANK the
            # drone => FAIL -> ASCEND_ESCAPE (climb) -> CALIB_TRANSLATE (slide 1u) -> re-run. PASS => explicit
            # height-OK: unfreeze the baseline ingest and resume. NO SILENT FALLBACK (every branch logs).
            active = {}                                # neutral hold -> the settled altitude is unbiased
            cap_ts = plan.get("cap_ts")
            healthy = plan.get("plan_valid") and plan.get("pos_y") is not None and not self._slam_slow
            # None-guard (Trap B): a dropped-frame / missing cap_ts must not crash a `None >= float` compare.
            settled = (self._descend_issue_t is not None and cap_ts is not None
                       and cap_ts >= self._descend_issue_t + self.calib_settle_gate_s)
            verify_timeout = (now - self.t_state) >= self.calib_verify_max_s
            hist = self._mapping_altitude_history
            # Session-14 TRIM: the settled, healthy post-descend pos_y is exactly `desired_y` — capture it (with
            # `ceiling_y` from the ASCEND) ONLY on a PASS below (Trap D: never mid-wobble). None if not settled.
            settled_y = float(plan["pos_y"]) if (settled and healthy) else None
            # result: None = keep holding, "PASS", "FAIL" (sank), "TIMEOUT_FAIL" (no settled healthy pose in cap)
            result, why = None, ""
            if verify_timeout and not settled:
                # Session 15: timed out with NO settled post-descend frame -> DON'T fly to a goal on a stale
                # pose. Count it as a failed attempt (escape/STUCK guard), never a silent PASS.
                result, why = "TIMEOUT_FAIL", (f"settle gate not met within {self.calib_verify_max_s:.0f}s "
                                               f"(no populated post-descend frame)")
            elif settled and healthy:
                if len(hist) < self.calib_min_baseline_samples:
                    result, why = "PASS", (f"insufficient baseline ({len(hist)}<"
                                           f"{self.calib_min_baseline_samples}) -> cannot judge -> PASS")
                else:
                    s = sorted(hist); n = len(s)
                    med = s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])
                    y = float(plan["pos_y"])
                    if y < med:                       # spatially HIGHER than normal -> not a sink -> keep waiting
                        if verify_timeout:
                            result, why = "PASS", (f"settled y={y:+.3f} above median {med:+.3f} at "
                                                   f"verify timeout -> PASS")
                    elif y > med + self.calib_low_height_margin:
                        result, why = "FAIL", (f"settled y={y:+.3f} is {y - med:+.3f} BELOW the flying-height "
                                               f"median {med:+.3f} (> {self.calib_low_height_margin:.2f}) -> "
                                               f"calibration SANK the drone")
                    else:
                        result, why = "PASS", (f"settled y={y:+.3f} within {self.calib_low_height_margin:.2f} "
                                               f"of median {med:+.3f} -> height OK")
            elif verify_timeout:                     # gate met (or n/a) but no healthy pose within the cap
                # Session 15: timed out without a HEALTHY settled pose -> failed attempt, not a stale-pose PASS.
                result, why = "TIMEOUT_FAIL", (f"verify timed out ({self.calib_verify_max_s:.0f}s) with no "
                                               f"healthy settled pose")
            # (else: settled but pose momentarily unhealthy, or not yet settled -> keep holding neutral)
            if result == "PASS":
                self._calib_active = False           # UNFREEZE the baseline ingest — height confirmed OK
                self._height_calibrated = True        # session 18: first PASS -> start measuring drone height
                self._recalibrating = False
                self._calib_interrupted = False      # the (possibly interrupted) calibration completed smoothly
                self._calib_fail_streak = 0          # a completed calibration breaks the failure streak (session 15)
                self._calib_escaped = False
                # Session-14 (RESTORED session 21): record the three TRIM references from THIS settled
                # calibration (desired_y is the settled height; delta = how far below the ceiling we fly). Only
                # when we actually settled with a healthy pose AND have a ceiling from the ASCEND — a
                # timeout-PASS (no settled_y) keeps the last good references. Logged LOUD (terminal + HTML).
                calib_log = ""
                if settled_y is not None and self._ceiling_y is not None:
                    self._desired_y = settled_y
                    self._trim_delta = self._desired_y - self._ceiling_y
                    # Session 22: the altitude lock holds the SAME verified height TRIM defends (previously it
                    # lazily cached whatever pos_y came first post-prelude).
                    self.target_altitude_y = settled_y
                    calib_log = (f" | HEIGHT-CALIB values: ceiling_y={self._ceiling_y:+.3f} "
                                 f"desired_y={self._desired_y:+.3f} delta={self._trim_delta:.3f} "
                                 f"(TRIM band: {self._desired_y - self.trim_high_ratio * self._trim_delta:+.3f} "
                                 f"(high) .. {self._ceiling_y + self.trim_sag_ratio * self._trim_delta:+.3f} (low))")
                    # Y-DRIFT AUDIT (session 22): any non-first tap measures how far the ceiling reading moved
                    # since the FIRST calibration — with SLAM height stable this should be ~0; a real drift is
                    # VISIBLE here (the whole point of a rare re-enabled re-tap).
                    if self._first_ceiling_y is None:
                        self._first_ceiling_y = float(self._ceiling_y)
                    else:
                        calib_log += (f" | Y-DRIFT check: ceiling_y moved "
                                      f"{self._ceiling_y - self._first_ceiling_y:+.3f}u since the first calibration")
                elif settled_y is not None:
                    med = self._alt_median
                    calib_log = (f" | HEIGHT-CALIB: settled pos_y={settled_y:+.3f} (no ceiling ref)"
                                 + (f" (flight-median {med:+.3f})" if med is not None else ""))
                nxt = self._settle_to or "REPLAN"
                self._settle_to = None
                self._enter(nxt, now)
                event = f"height OK -> {nxt} ({why}){calib_log}"
            elif result == "FAIL":
                if self._calib_retries < self.calib_max_retries:
                    self._calib_retries += 1
                    self._ascend_phase = None
                    self._ascend_start_t = None
                    self._enter("ASCEND_ESCAPE", now)   # _calib_active STAYS True through the retry
                    event = (f"height FAIL -> ASCEND_ESCAPE (climb to clean airspace before sliding sideways) "
                             f"[retry {self._calib_retries}/{self.calib_max_retries}] ({why})")
                else:
                    self._calib_active = False
                    self._height_calibrated = True     # session 18: calibration resolved (even if abandoned) -> measure
                    self._recalibrating = False
                    self._calib_interrupted = False   # calibration resolved (abandoned after retries) -> no redo owed
                    nxt = self._settle_to or "REPLAN"
                    self._settle_to = None
                    self._enter(nxt, now)
                    event = (f"height FAIL but retries exhausted ({self.calib_max_retries}) -> abandon calib -> "
                             f"{nxt} (VISIBLE WARN: mapping may be degraded) ({why})")
            elif result == "TIMEOUT_FAIL":
                # Never fly to a goal on a stale/absent pose: route through the escape/STUCK guard (session 15).
                event = self._calib_fail_escalate(now, f"CALIB_VERIFY {why}")

        elif st == "ASCEND_ESCAPE":
            # Height-calib retry, step 1 (vertical-THEN-horizontal — never slide while sunk at a corrupted low
            # height, risking clipping low furniture/walls). A bounded pulsed climb into clean airspace, reusing
            # the two-phase UP-pulse approach, but recording NO ceiling tap and NO altitude latch (purely to
            # gain altitude). Ends on a ceiling contact / gain flatten / ascend_max_s cap -> CALIB_TRANSLATE.
            if self._ascend_phase is None:
                self._ascend_phase, self._ascend_phase_t0 = "PULSE", now
                self._ascend_prev_y, self._ascend_stall_count = None, 0
                self._ascend_start_t = now
            done_climb, why = False, ""
            if (now - self._ascend_start_t) > self.ascend_max_s:
                done_climb, why = True, f"cap {self.ascend_max_s:.0f}s"
            elif ceiling_contact:
                done_climb, why = True, "ceiling contact"
            elif self._ascend_phase == "PULSE":
                active = dict(self.ascend_preset)          # a short UP micro-pulse (near-zero momentum)
                if (now - self._ascend_phase_t0) >= self.ascend_micro_pulse_s:
                    self._ascend_phase, self._ascend_phase_t0 = "REST", now
            else:   # REST: neutral (momentum bleeds); sample the SLAM altitude gain this cycle
                if (now - self._ascend_phase_t0) >= self.ascend_rest_s:
                    valid = plan.get("plan_valid") and plan.get("pos_y") is not None and not self._slam_slow
                    if not valid:
                        self._ascend_phase_t0 = now        # pose invalid/slow -> pause (hold); the cap is the backstop
                    else:
                        y = float(plan["pos_y"])
                        dz = None if self._ascend_prev_y is None else (self._ascend_prev_y - y)
                        self._ascend_prev_y = y
                        if dz is not None and dz <= self.ascend_gain_eps:
                            self._ascend_stall_count += 1
                        else:
                            self._ascend_stall_count = 0
                        if self._ascend_stall_count >= self.ascend_stall_cycles:
                            done_climb, why = True, "height gain flattened (at ceiling)"
                        else:
                            self._ascend_phase, self._ascend_phase_t0 = "PULSE", now
            if done_climb:
                self._ascend_phase = None
                self._push_dir = None
                self._push_start_pos = None
                self._enter("CALIB_TRANSLATE", now)
                event = f"ascend-escape done ({why}) -> translate to clean airspace before re-calibrating"

        elif st == "CALIB_TRANSLATE":
            # Height-calib retry, step 2: a CLEAN horizontal translation (calib_retry_translate_dist, ~1u) off
            # the CURRENT pose in the now-high airspace, before re-running the calibration. Mirrors
            # BASELINE_NUDGE: pick the roomier fwd/back axis from the clearance ring, distance-quantized off the
            # live pose, clearance-guarded + a time cap; boxed on both axes -> re-calibrate anyway (logged).
            # Done -> CALIBRATING_HEIGHT (-> ASCEND -> DESCEND -> CALIB_VERIFY). _calib_active stays True.
            ring = plan.get("clearance_ring")
            if self._push_dir is None:            # first tick: choose the roomier PUSHABLE fwd/back axis
                cands = [(0.0, self._ring_get(ring, 0.0)), (180.0, self._ring_get(ring, 180.0))]
                pushable = [(rel, c) for rel, c in cands if self._pushable(c)]
                if not pushable:
                    self._ascend_phase = None
                    self._enter("CALIBRATING_HEIGHT", now)
                    event = "calib-translate: no room fwd/back -> re-calibrate anyway (VISIBLE)"
                else:                             # None (open near-field) ranks as most room
                    rel, _ = max(pushable, key=lambda kv: (float("inf") if kv[1] is None else kv[1]))
                    self._push_dir = "forward" if rel == 0.0 else "backward"
                    self._push_start_pos = plan.get("pos")
            if self.state == "CALIB_TRANSLATE":   # still translating (didn't bail above)
                if self._push_dir == "forward":
                    active = {"trigger": self.parallax_push_throttle}   # brisk, decoupled from the ADVANCE crawl
                    guard = self._ring_get(ring, 0.0)
                else:
                    active = dict(self.pb.recipe("back_off")[0])        # reverse magnitude, held continuously
                    active.pop("duration_s", None)
                    guard = self._ring_get(ring, 180.0)
                traveled = self._dist(plan.get("pos"), self._push_start_pos)
                far = traveled is not None and traveled >= self.calib_retry_translate_dist
                blocked = guard is not None and guard <= self.parallax_min_clear
                timeout = (now - self.t_state) >= self.baseline_nudge_max_s
                if far or blocked or timeout:
                    why = "dist" if far else "blocked" if blocked else "timer"
                    dirn = self._push_dir
                    self._push_dir = None
                    self._ascend_phase = None
                    self._enter("CALIBRATING_HEIGHT", now)
                    event = f"calib-translate {dirn} done ({why}) -> re-calibrate height"

        elif st == "BASELINE_NUDGE":
            # One-shot open-loop horizontal translation after the ceiling tap, to give monocular SLAM the
            # translational parallax it needs BEFORE the first exploration yaw (pure rotation is the known
            # SLAM-killer here). Reuse the parallax machinery: pick the roomier body axis from the clearance
            # ring, translate a bounded distance (distance-quantized off the live pose), guarded by clearance
            # + a time cap. Boxed in both axes -> skip (logged). The time cap bounds it if the pose is stale.
            ring = plan.get("clearance_ring")
            if self._push_dir is None:            # first tick: choose the roomier PUSHABLE fwd/back axis
                cands = [(0.0, self._ring_get(ring, 0.0)), (180.0, self._ring_get(ring, 180.0))]
                pushable = [(rel, c) for rel, c in cands if self._pushable(c)]
                if not pushable:
                    self._baseline_seeded = True
                    self._settle_to = "REPLAN"
                    self._enter("SETTLE", now)
                    event = "baseline nudge: no room fwd/back -> skip -> settle -> replan"
                else:                             # None (open near-field) ranks as most room
                    rel, _ = max(pushable, key=lambda kv: (float("inf") if kv[1] is None else kv[1]))
                    self._push_dir = "forward" if rel == 0.0 else "backward"
                    self._push_start_pos = plan.get("pos")
            if self.state == "BASELINE_NUDGE":    # still nudging (didn't skip above)
                if self._push_dir == "forward":
                    active = {"trigger": self.parallax_push_throttle}   # brisk, decoupled from the ADVANCE crawl
                    guard = self._ring_get(ring, 0.0)
                else:
                    active = dict(self.pb.recipe("back_off")[0])        # reverse magnitude, held continuously
                    active.pop("duration_s", None)
                    guard = self._ring_get(ring, 180.0)
                traveled = self._dist(plan.get("pos"), self._push_start_pos)
                far = traveled is not None and traveled >= self.baseline_nudge_dist
                blocked = guard is not None and guard <= self.parallax_min_clear
                timeout = (now - self.t_state) >= self.baseline_nudge_max_s
                if far or blocked or timeout:
                    why = "dist" if far else "blocked" if blocked else "timer"
                    dirn = self._push_dir
                    self._push_dir = None
                    self._baseline_seeded = True
                    self._settle_to = "REPLAN"
                    self._enter("SETTLE", now)
                    event = f"baseline {dirn} nudge done ({why}) -> settle -> replan"

        elif st == "REPLAN":
            self._explore_started = True          # past the prelude -> status-gated recovery is now armed
            # Reaching REPLAN at all is a genuinely trusted recovery point (it only happens once the
            # two-gate settle gate — freshness + physical dwell — has cleared), and per-leg it's also
            # where a materially NEW goal gets committed (`goal_moved`, below) -- either way, the
            # SLAM_STEPBACK escalation counter's "this physical location is giving SLAM trouble" penalty
            # no longer applies. Reset unconditionally here rather than on every `_enter_slam_hold` (which
            # let a bad patch's PLAN-LOST/HOLD_LOST bounce wipe it before it could ever escalate).
            self._slam_stepback_count = 0
            self._slam_slow_streak = 0
            if plan.get("done") or plan.get("goal") is not None:
                self._no_goal_since = None         # a live goal / done clears the idle backstop tracker
                self._no_goal_warned = False
                self.no_goal_stall = False
            if plan.get("done"):
                self.done = True
                if plan.get("corner_giveup_stuck"):
                    # Session 24: the corner tour only finished because at least one corner was ABANDONED
                    # (corner_giveup_limit far-corner strikes, never once close enough for a real 2-bump) --
                    # not because every corner was genuinely reached/2-bump-confirmed. The drone is almost
                    # certainly physically stuck; per the operator, don't attempt a graceful homing/dock —
                    # just hold in place (mirrors the SLAM-fallback-exhaustion use of STUCK).
                    self._corner_giveup_stuck = True   # gates STUCK's own resume check below
                    self._enter("STUCK", now)
                    event = ("mission ABANDONED: at least one corner was never reached (far-corner give-up "
                             "cap hit) -> drone likely physically stuck -> STUCK (hold in place; logging paused)")
                else:
                    self._home_phase = None               # lazy-init the homing sub-loop on entry
                    self._enter("RETURN_TO_ORIGIN", now)
                    event = ("mission complete — no reachable frontier remains -> RETURN_TO_ORIGIN "
                             "(floor-dock postlude)")
            elif plan.get("goal") is not None:
                pos = plan.get("pos")
                # --- Session 20b: judge the HOP that just finished (per-hop progress -> STRIKE/reset), THEN
                #     register the new PICK. Both ride one pulse to the planner's goals-DB (run_explore publishes
                #     it). prev_strike_eligible uses the OLD leg's corner-ness + our distance to it: a FAR corner
                #     (> corner_no_blacklist_dist) is never struck (a corner is a far reposition target, unlike a
                #     nearby frontier). A mid-hop plan-loss already cleared _hop_start_goal, so it won't be judged.
                prev_goal = self._hop_start_goal
                prev_progressed, prev_strike_eligible = None, True
                prev_is_corner = bool(self._leg_is_corner)    # the JUDGED (old) leg's corner-ness, for evidence
                if prev_goal is not None and self._hop_start_dist is not None:
                    end_d = self._dist(pos, prev_goal)
                    if end_d is not None:
                        # progress = closed >= eps, OR the goal is now REACHED (reaching is the ultimate progress,
                        # never a strike — a hop that ends by arriving must reset, not accrue, strikes).
                        prev_progressed = ((self._hop_start_dist - end_d) >= self.hop_progress_eps
                                           or end_d <= self.goal_reach_dist)
                        if self._leg_is_corner and end_d > self._corner_no_blacklist_dist(plan):
                            prev_strike_eligible = False   # far corner -> no strike
                        self._hop_judge_msg = (
                            f"[HOP_JUDGE] pos={pos} cap_ts={plan.get('cap_ts')} "
                            f"frame_id={plan.get('frame_id')} prev_goal={prev_goal} "
                            f"start_dist={self._hop_start_dist:.3f} end_dist={end_d:.3f} "
                            f"closed={self._hop_start_dist - end_d:.3f} progressed={prev_progressed}")
                self._hop_start_dist = None               # judged (or unjudgeable) -> clear the pending eval
                self._hop_start_goal = None
                self.leg_goal = list(plan["goal"])
                self._leg_is_corner = bool(plan.get("goal_is_corner"))  # the new published goal is a sweep-tour corner?
                # Per-goal height re-calibration (RESTORED session 21): on a GENUINE goal change (moved >
                # calib_goal_change_dist) past the cooldown, re-tap the ceiling first to re-latch the mapping
                # altitude for the new leg. It routes ASCEND->DESCEND->REPLAN, which re-enters here with the
                # SAME goal (goal_moved False, _leg_goal_prev already set) -> the normal branch below then emits
                # the leg's PICK pulse exactly once and orients (theta≈0 -> a 'c'-only reset, review-D: the
                # vertical excursion preserved the heading, so no attitude thrash).
                goal_moved = (self._leg_goal_prev is None
                              or self._dist(self.leg_goal, self._leg_goal_prev) > self.calib_goal_change_dist)
                # review-A: `_last_calib_t is None` (never calibrated — --no-takeoff / failed prelude) ALLOWS a
                # calibration instead of locking it out forever; a normal takeoff's ASCEND sets it, so the first
                # post-prelude goal is still cooldown-gated.
                cooldown_ok = (self._last_calib_t is None
                               or (now - self._last_calib_t) >= self.calib_cooldown_s)
                # SESSION-22 comfort gate: a (re-enabled) periodic tap launches only when SLAM is COMFORTABLE
                # (healthy-frame latency avg under the bar). A deferred tap is NOTED on the leg event (visible,
                # not silent) and simply re-tests on the next replan — the cooldown is untouched.
                calib_wanted = (self.calibrate_on_goal_change and self.ascend_to_ceiling
                                and goal_moved and cooldown_ok and not self._recalibrating)
                calib_deferred = calib_wanted and not self._calib_slam_comfortable()
                if calib_wanted and not calib_deferred:
                    self._recalibrating = True
                    self._calib_retries = 0
                    self._leg_goal_prev = list(self.leg_goal)   # post-calib REPLAN sees the SAME goal -> no re-trigger
                    self._ascend_phase = None
                    # Hop-outcome-ONLY pulse (pick_goal=None): the finished hop is still judged (strike/progress)
                    # but the PICK registers post-calib when the normal branch re-commits this same goal.
                    if prev_goal is not None:
                        self._pick_pulse = {"pick_goal": None, "pick_pos": None,
                                            "prev_goal": list(prev_goal),
                                            "prev_progressed": prev_progressed,
                                            "prev_strike_eligible": prev_strike_eligible,
                                            "prev_is_corner": prev_is_corner,
                                            "judge_pos": (list(pos) if pos is not None else None),
                                            "judge_slam_ms": plan.get("slam_ms")}
                    self._enter("CALIBRATING_HEIGHT", now)
                    event = (f"goal changed (> {self.calib_goal_change_dist:.1f}u) + "
                             f"{self.calib_cooldown_s:.0f}s cooldown elapsed -> CALIBRATING_HEIGHT "
                             f"(re-tap ceiling) for goal {self.leg_goal}")
                else:
                    # De-dupe the PICK against the goals-DB loop guard (operator diagnosis): a multi-step turn
                    # (ORIENT partial-turn -> PARALLAX_PUSH -> SETTLE -> REPLAN, repeated per turn-step) re-reads
                    # and re-commits the SAME still-uncommitted goal on every one of those sub-steps -- that is
                    # NOT a genuinely new pick, just this leg's own re-orientation in progress. Bug fix (found on
                    # 20260719_005402): this used to reuse `goal_moved` (computed against `calib_goal_change_dist`,
                    # a height-recalibration knob, 1.0u) for this decision too -- but the goals-DB's OWN "same
                    # goal" radius is `goal_area_radius` (0.5u, what frontier_planner.py's _db_entry actually
                    # matches discs on), so a goal picked 0.5-1.0u from the last one was wrongly deduped as
                    # "not a new pick" (register_goal_pick never ran; picks stuck at 0) while its hops still
                    # earned real strikes via register_hop_outcome. `pick_moved` is the SAME comparison as
                    # `goal_moved` but against the goals-DB's own radius, used ONLY for this pick-dedup decision
                    # -- `goal_moved` itself (and the calibration trigger it drives, above) is untouched.
                    # The hop-outcome half (strike/progress) still judges normally either way -- only the PICK
                    # half (the goals-DB circling counter) is suppressed on a same-goal re-commit.
                    pick_moved = (self._leg_goal_prev is None
                                  or self._dist(self.leg_goal, self._leg_goal_prev) > self.goal_area_radius)
                    # A genuinely completed hop (prev_goal is not None -- a real ADVANCE was judged THIS replan,
                    # not a same-leg turn/scout sub-step still in progress) always counts as a real pick, even
                    # if it lands close to the last commit -- that IS the circling behaviour register_goal_pick's
                    # loop guard exists to catch (20260720 bug: a frontier repeatedly "reached" from ~the same
                    # spot never accrued a strike -- reaching is unconditional progress -- so only the picks-based
                    # loop guard could ever retire it, and it was starved by this same-position dedup treating
                    # every completed hop as if it were a same-leg sub-step). Only a same-tick sub-step of the
                    # SAME still-uncommitted leg (no hop judged yet) gets deduped.
                    same_goal_as_last_pick = (not pick_moved) and (prev_goal is None)
                    self._leg_goal_prev = list(self.leg_goal)   # track for the next goal-change test
                    self._pick_pulse = {"pick_goal": (None if same_goal_as_last_pick else list(self.leg_goal)),
                                        "pick_pos": (None if same_goal_as_last_pick
                                                     else (list(pos) if pos is not None else None)),
                                        "prev_goal": (list(prev_goal) if prev_goal is not None else None),
                                        "prev_progressed": prev_progressed,
                                        "prev_strike_eligible": prev_strike_eligible,
                                        "prev_is_corner": prev_is_corner,
                                        "judge_pos": (list(pos) if pos is not None else None),
                                        "judge_slam_ms": plan.get("slam_ms")}
                    self._recalibrating = False       # orienting to a goal (self-heals if a SLAM blip cut a re-tap short)
                    self._ram_accum = 0.0             # fresh ram-guard stall tracking for this leg
                    self._ram_last_t = None
                    self._ram_speed_win.clear()      # a new leg breaks the speed run; window must not span the gap
                    self._ram_speed = None
                    self._finalize_or_discard_calib()  # a leg boundary ends a sampling run: accept if clean, else restart
                    be = plan.get("bearing_err")
                    theta = self._quantize_turn(be)
                    if self.clamp_leg_turn:
                        # Cap to ONE turn_step (<=45 deg): SLAM survives a small open-loop turn (proven live);
                        # the per-leg replan after each ADVANCE is the outer correction toward the goal.
                        theta = max(-self.turn_step_deg, min(self.turn_step_deg, theta))
                    self._leg_theta = theta           # logged into command_history when the ORIENT turn is flown
                    # PARALLAX SCOUT: if the goal needs MORE than one turn_step, we won't be aimed after this
                    # turn — so after turning, do a short translation (for SLAM parallax) BEFORE turning again,
                    # instead of advancing toward an intermediate (off-goal) heading. Aimed within one step ->
                    # ADVANCE straight to the goal. ORIENT routes to whichever we pick here.
                    need_more = be is not None and abs(be) > self.turn_step_deg + 1e-6
                    if (self.parallax_scout and need_more and plan.get("clearance_ring")
                            and self._push_count < self.parallax_max_pushes):
                        self._after_orient = "PARALLAX_PUSH"
                    else:
                        self._after_orient = "ADVANCE"   # aimed within one step, no ring, or cap hit -> straight advance
                    self._player = self._build_turn(theta)       # open-loop turn (or just 'c' if theta≈0)
                    self._enter("ORIENT", now)
                    event = (f"leg -> turn {theta:+.0f} deg (err {self._fmt(be)}) then "
                             f"{'parallax push' if self._after_orient == 'PARALLAX_PUSH' else 'advance'} "
                             f"toward goal {self.leg_goal}")
                    if calib_deferred:              # visible, not silent: the tap waits for a comfortable SLAM
                        event += (f" (calib DEFERRED: SLAM avg {self._slam_ms_avg:.0f}ms >= "
                                  f"{self.calib_slam_avg_ms:.0f} — retry next replan)")
            else:
                # No goal AND not done with a HEALTHY plan. With the diagonal-sweep planner this is only a
                # momentary startup tick before the first frontiers form (the planner now returns a sweep
                # goal or done=True once the map exists). FAIL-VISIBLE BACKSTOP: never idle dark forever —
                # if it persists past no_goal_idle_s, log once + raise a telemetry flag so the operator sees
                # the degraded state (NO SILENT FALLBACK). SLAM-loss recovery stays status-driven (PLAN-
                # STALE/LOST), not triggered from here.
                if self._no_goal_since is None:
                    self._no_goal_since = now
                elif (now - self._no_goal_since) > self.no_goal_idle_s and not self._no_goal_warned:
                    self._no_goal_warned = True
                    self.no_goal_stall = True
                    event = (f"REPLAN IDLE: planner returned no goal (and not done) for "
                             f">{self.no_goal_idle_s:.0f}s — holding, VISIBLE-FLAGGED (no silent idle; "
                             f"map may be too small to sweep / SLAM still forming)")

        elif st == "STUCK":
            # HOLD (neutral) after the fallback gave up. A valid goal (SLAM re-acquired + planning) resumes.
            # The give-up counter is NOT reset here (D5): `_recovering` stays set and only a confirming >=1u
            # ADVANCE restores trust — so a re-lock that can't fly a real leg falls back to STUCK, not a loop.
            # EXCEPT (session 24) a corner-giveup-exhausted mission: `plan.get("done")` stays permanently True
            # here (nothing left to explore), so the generic "done -> resume" check below would immediately
            # bounce this STUCK back out on the very next tick — `_corner_giveup_stuck` makes THIS hold a true
            # terminal one instead (the drone is almost certainly stuck; no resume to attempt).
            if not self._corner_giveup_stuck and (plan.get("goal") is not None or plan.get("done")):
                self._enter("REPLAN", now)
                event = "plan recovered -> resume exploring (re-locked; NOT trusted until a >=1u ADVANCE)"

        elif st == "ORIENT":
            # OPEN-LOOP: play the (scaled) turn recipe to completion — sustained yaw hold then 'c' — then
            # fly. No in-turn feedback; the next leg's re-plan corrects any residual heading error.
            active, tdone = self._player.fields(now)
            if tdone:
                self._log_turn(self._leg_theta)   # record the flown rotation for a later inverse rewind
                nxt = self._after_orient or "ADVANCE"
                if nxt == "ADVANCE":
                    self._push_count = 0      # aimed -> real ADVANCE leg = progress; reset the scout cap
                else:
                    self._push_dir = None     # choose the push axis fresh from the post-turn ring
                # Turns are the hardest thing for monocular SLAM; if the solve is still choking, HOLD before
                # flying on a shaky post-turn pose (the ~45deg heading gap). Settle first, then proceed to nxt.
                if self._slam_slow:
                    return self._enter_slam_hold(nxt, now,
                                                 f"turn complete, SLAM slow ({self._slam_ms_latest:.0f}ms "
                                                 f">= {self.slam_slow_ms:.0f}) -> hold to settle before {nxt}")
                self._enter(nxt, now)
                event = f"turn complete -> {nxt}"

        elif st == "ADVANCE":
            reached = self._dist(plan.get("pos"), self.leg_goal)
            clr = plan.get("forward_clearance_dist")
            fwd_dur, fwd_val = (now - self.t_state), float(self.forward_preset.get("trigger", 0.0))
            # Session-20b PER-HOP progress snapshot: on the first ADVANCE tick with a pose, record the distance to
            # the goal so the NEXT REPLAN can judge whether this hop got meaningfully closer (progress) or not (a
            # STRIKE toward this goal). Captured here (not _enter) because _enter has no pose. Cleared at REPLAN.
            if reached is not None and self._hop_start_goal is None:
                self._hop_start_dist = reached
                self._hop_start_goal = list(self.leg_goal)
                self._hop_baseline_msg = (
                    f"[HOP_BASELINE] pos={plan.get('pos')} cap_ts={plan.get('cap_ts')} "
                    f"frame_id={plan.get('frame_id')} bound against goal={self.leg_goal} dist={reached:.3f}")
            # CONFIRMING ADVANCE (D5): a re-locked drone is trusted again ONLY after it flies a genuine leg. Gauge
            # progress from this leg's start; once it has advanced >= recovery_confirm_dist, RESTORE trust — drop
            # `_recovering`/`_history_broken`, reset the give-up counter, and CLEAR the (now-stale) reverse-list so
            # logging resumes from a fresh, coherent chain. Progress-gated, NOT "the ADVANCE state ran": a re-lock
            # that instantly bumps a wall at ~0 distance must NOT count.
            if self._recovering:
                if self._recovery_adv_start is None:
                    self._recovery_adv_start = plan.get("pos")
                else:
                    moved = self._dist(plan.get("pos"), self._recovery_adv_start)
                    if moved is not None and moved >= self.recovery_confirm_dist:
                        self._recovering = False
                        self._history_broken = False
                        self._fallback_attempts = 0
                        self.command_history.clear()
                        self._recovery_adv_start = None
                        event = (f"recovery CONFIRMED: advanced {moved:.2f}u >= {self.recovery_confirm_dist:g}u "
                                 "-> trust restored (counter reset, reverse-list cleared, logging resumed)")
            if self._slam_slow:
                # SLAM started choking mid-leg -> STOP moving and let it settle before it loses the track.
                # Log the clean sub-leg flown so far (also keeps translations in the rewind history), then hold
                # and resume ADVANCE (leg_goal persists) once stable. (No speed sample on a choking frame.)
                self._log_move("forward", fwd_val, fwd_dur)
                return self._enter_slam_hold("ADVANCE", now,
                                             f"ADVANCE: SLAM slow ({self._slam_ms_latest:.0f}ms) -> "
                                             "hold to settle, then resume")
            # Live self-calibrated world speed for the ram guard (updates the window + one-time nominal).
            had_nominal = self._nominal_speed is not None
            spd = self._advance_speed(now, plan.get("pos"))
            calib_event = (None if had_nominal or self._nominal_speed is None else
                           f"ram-calib: nominal free-flight speed = {self._nominal_speed:.3f} u/s "
                           f"(guard now armed at {self.ram_speed_frac:.0%} of it)")
            if self.stop_on_clearance and clr is not None and clr <= self.stop_clearance_dist:
                # PRIMARY forward stop: SLAM has mapped a wall ahead within the stand-off margin. Stop
                # gently with the image still rich (SLAM ALIVE) BEFORE ramming -> REPLAN picks the next
                # frontier. A small back_off follows (default) so the reverse re-arms the 2-bump latch (a
                # stand-off pin can then blacklist an unreachable wall) and seeds SLAM parallax; back_off
                # itself routes to SETTLE. `backoff_on_standoff=False` restores the direct settle.
                self._log_move("forward", fwd_val, fwd_dur)   # record the clean forward leg for a later rewind
                self._register_bump(plan, "clearance stand-off")  # advance-blocked stop -> bump toward committed goal
                if self.backoff_on_standoff:
                    self._player = self.pb.player("back_off")
                    self._enter("BACKOFF", now)
                    event = (f"clearance {clr:.2f} <= {self.stop_clearance_dist:.2f} -> standoff stop -> "
                             "back off (re-arm bump latch) -> settle")
                else:
                    self._enter("SETTLE", now)
                    event = f"clearance {clr:.2f} <= {self.stop_clearance_dist:.2f} -> standoff stop -> settle"
            elif wall_contact:
                # A COLLISION invalidates the command history (unknown post-impact orientation) -> drop it.
                self.command_history.clear()
                self._register_bump(plan, "flow WALL contact")   # advance-blocked stop -> bump toward committed goal
                if self.reverse_probe_on_wall:
                    # EXPERIMENT: instead of a tiny back-off, settle then fly straight BACKWARD (camera
                    # still facing the wall, seeing familiar features) to test whether reverse keeps SLAM
                    # TRACKING. Rest-separated (like recovery) so forward momentum dies before the reverse.
                    self._player = None              # clear the spent ORIENT turn player so REVERSE_PROBE builds fresh
                    self._settle_to = "REVERSE_PROBE"
                    self._enter("SETTLE", now)
                    event = "WALL contact -> clear history -> settle -> reverse probe (experiment)"
                else:
                    self._player = self.pb.player("back_off")
                    self._enter("BACKOFF", now)
                    event = "WALL contact -> clear history -> back off"
            elif reached is not None and reached <= self.goal_reach_dist:
                self._log_move("forward", fwd_val, fwd_dur)
                self._enter("SETTLE", now)
                event = f"goal reached (d={reached:.2f}) -> settle"
            elif (now - self.t_state) > self.leg_max_s:
                self._log_move("forward", fwd_val, fwd_dur)
                self._player = self.pb.player("back_off")
                self._enter("BACKOFF", now)
                event = f"LEG-TIMEOUT (>{self.leg_max_s}s) -> back off"
            elif self.hop_duration_s > 0 and fwd_dur >= self.hop_duration_s:
                # Session-20 HOP (NO commitment): advanced hop_duration_s SECONDS -> SETTLE (a fresh-frame SLAM
                # breather) -> REPLAN — re-read SLAM's CURRENT goal. If SLAM re-picked a different goal while the
                # drone hopped, REPLAN commits the NEW one and re-orients (WITH the parallax scout for an off-axis
                # goal) before the next hop; a same-goal re-pick just re-orients ('c') and hops on. The drone NEVER
                # resumes an old, unreached leg_goal — SLAM stays free to steer, the goals-DB retires any ping-pong
                # loop. Time-based (not a tick count) since the 20260719 investigation -- see __init__.
                self._log_move("forward", fwd_val, fwd_dur)
                self._settle_to = "REPLAN"
                self._enter("SETTLE", now)
                event = ((f"hop {fwd_dur:.2f}s ({self._hop_tick} ticks) -> settle -> REPLAN (re-pick SLAM's "
                          f"current goal; was {self.leg_goal}, d={reached:.2f})") if reached is not None else
                         f"hop {fwd_dur:.2f}s ({self._hop_tick} ticks) -> settle -> REPLAN (re-pick SLAM's current goal)")
            else:
                # Ram guard (SELF-CALIBRATING): accrue time while the live world speed is BELOW
                # `ram_speed_frac` of the drone's own calibrated nominal free-flight speed; reset the clock
                # whenever it recovers. Stop the leg after `ram_stall_s` continuously below-nominal — the drone
                # is physically pinned (riding an invisible collider). A legitimately SLOW open-space crawl runs
                # AT ~nominal and never trips this (the bug the old absolute goal-closing threshold caused).
                # Inactive until the nominal is calibrated (fail-safe: never fire on an unknown baseline).
                dt = 0.0 if self._ram_last_t is None else min(max(now - self._ram_last_t, 0.0), 0.5)
                self._ram_last_t = now
                stalled = (self._nominal_speed is not None and spd is not None
                           and spd < self.ram_speed_frac * self._nominal_speed)
                self._ram_accum = self._ram_accum + dt if stalled else 0.0
                if (self.ram_stall_s > 0 and self._nominal_speed is not None
                        and self._ram_accum >= self.ram_stall_s):
                    self._log_move("forward", fwd_val, fwd_dur)
                    self._register_bump(plan, "ram guard")   # advance-blocked stop -> bump toward committed goal
                    self._enter("SETTLE", now)
                    event = (f"ram guard: speed {spd:.3f} < {self.ram_speed_frac:.0%} of nominal "
                             f"{self._nominal_speed:.3f} u/s for {self._ram_accum:.1f}s "
                             f"(d={reached:.2f}) -> stop leg -> settle -> replan")
                else:
                    if calib_event is not None:
                        event = calib_event      # surface the one-time nominal calibration
                    self._hop_tick += 1          # session 20: count advancing ticks toward the hop cap
                    active = dict(self.forward_preset)
                    # Altitude lock: counter the forward-push sink. World frame +Y is DOWN, so a drone that has
                    # sunk reads a LARGER pos_y than the cached target -> inject UP until it climbs back (deadband).
                    y = plan.get("pos_y")
                    if (self.altitude_lock and self.target_altitude_y is not None and y is not None
                            and y > self.target_altitude_y + self.alt_drift_floor):
                        active["joy_vertical"] = self.ascend_preset["joy_vertical"]   # -1 = up (camera Y down)

        elif st == "BACKOFF":
            active, bdone = self._player.fields(now)
            if bdone:
                self._enter("SETTLE", now)
                event = "backed off -> settle"

        elif st == "REVERSE_PROBE":
            # EXPERIMENT: sustained straight reverse (playbook "reverse_probe" recipe — tune its duration
            # there). The BACKWALL detector arms here (the command is derived from the reverse control
            # vector): a live contact ends the probe EARLY (we've backed into something — no point grinding
            # the rest of the recipe's fixed duration into it) instead of only the natural recipe timeout.
            if self._player is None:
                self._player = self.pb.player("reverse_probe")
            active, rdone = self._player.fields(now)
            if rdone or backwall_contact:
                self._player = None
                self._settle_to = "REPLAN"
                self._enter("SETTLE", now)
                event = ("reverse probe: BACKWALL contact -> stop early -> settle -> replan" if backwall_contact
                         else "reverse probe done -> settle -> replan")

        elif st == "PARALLAX_PUSH":
            # Short open-loop translation BETWEEN rotation steps, to give SLAM the parallax it needs to survive
            # a multi-step turn (and to stay roughly in place rather than advance off-goal). NEVER forward: a
            # forward push advances off an intermediate heading, sinks the drone (forward pitch), risks ramming
            # a wall (image freeze = SLAM death), and has the longest warm-up (barely translates). Priority:
            # (1) BACKWARD if pushable (ideal parallax: pure translation, camera still on the scene; the
            # validated reverse path) -> distance-quantized ~parallax_push_dist off the live pose; else
            # (2) STRAFE toward the roomier pushable side (left/right ring) -> a short TIMED hold (strafe is the
            # most responsive axis, near-zero warm-up); else (3) skip -> turn again. "Pushable" = ring clearance
            # None (open near-field) OR >= parallax_min_clear. Guarded by the live side/back clearance + time cap.
            ring = plan.get("clearance_ring")
            if self._push_dir is None:        # first tick: backward-first, then strafe, never forward
                push_dir, after_repo, pick_event = self._pick_ring_direction(ring, plan)
                if push_dir is None:
                    self._settle_to = "REPLAN"   # boxed all ways -> can't push safely -> turn again next REPLAN
                    self._enter("SETTLE", now)
                    event = "parallax: no room back/left/right -> skip push -> settle -> replan"
                else:
                    self._push_dir = push_dir
                    self._push_after_reposition = after_repo
                    event = pick_event
                if self._push_dir is not None:
                    self._push_count += 1
                    self._push_start_pos = plan.get("pos")
            if self.state == "PARALLAX_PUSH":    # still pushing (didn't bail to SETTLE above)
                if self._push_dir == "backward":
                    active = dict(self.pb.recipe("back_off")[0])   # reverse magnitude, held continuously
                    active.pop("duration_s", None)
                    guard = self._ring_get(ring, 180.0)
                elif self._push_dir == "reposition_fwd":           # D2: forward escape out of a scrape-danger corner
                    active = dict(self.forward_preset)             # forward @ forward_throttle
                    guard = None                                   # forward is guarded by the raycast test below
                else:                                              # strafe_left / strafe_right
                    sign = 1.0 if self._push_dir == "strafe_right" else -1.0
                    active = {"joy_horizontal": sign * self._strafe_mag}
                    guard = self._ring_get(ring, 90.0 if self._push_dir == "strafe_right" else -90.0)
                if self._slam_slow:
                    # SLAM choking mid-push -> log what we translated, stop, and settle before re-planning.
                    self._log_move_push(self._push_dir, now - self.t_state)
                    self._push_dir = None
                    self._settle_to = "REPLAN"
                    return self._enter_slam_hold("SETTLE", now,
                                                 f"parallax push: SLAM slow ({self._slam_ms_latest:.0f}ms) -> "
                                                 "hold to settle -> replan")
                if self._push_dir == "reposition_fwd":
                    # D2: run the forward escape for strafe_reposition_fwd_s (or until the forward raycast says a
                    # wall got close), then HAND OFF to the queued strafe from the roomier position (no settle).
                    fwd_clr = plan.get("forward_clearance_dist")
                    rep_done = (now - self.t_state) >= self.strafe_reposition_fwd_s
                    rep_blocked = fwd_clr is not None and fwd_clr <= self.stop_clearance_dist
                    if rep_done or rep_blocked:
                        self._log_move("forward", float(self.forward_preset.get("trigger", 0.0)), now - self.t_state)
                        self._push_dir = self._push_after_reposition
                        self._push_after_reposition = None
                        self._push_start_pos = plan.get("pos")
                        self._enter("PARALLAX_PUSH", now)          # reset the phase timer for the strafe hold
                        return {}, "PARALLAX_PUSH", (f"reposition forward done "
                                                     f"({'wall-close' if rep_blocked else 'timer'}) -> "
                                                     f"strafe {self._push_dir}")
                    return active, "PARALLAX_PUSH", event
                if self._push_dir == "backward" and (
                        (guard is not None and guard <= self.parallax_min_clear) or backwall_contact):
                    # Backward proved blocked THIS episode — either the ring caught up (guard) or the flow
                    # BACKWALL detector fired (we're commanding reverse and the image shows we stopped moving,
                    # i.e. SLAM hadn't mapped the wall behind us when the ring said "open"). Don't just give up:
                    # retry the SAME direction pick with backward excluded, exactly mirroring the entry-time
                    # logic (try a side, else give up for real).
                    why_blocked = "flow BACKWALL contact" if backwall_contact else "ring clearance"
                    self._log_move_push("backward", now - self.t_state)
                    new_dir, new_after_repo, pick_event = self._pick_ring_direction(ring, plan, force_no_backward=True)
                    if new_dir is None:
                        if self.leg_goal is not None:
                            self._missed_bump = (f"no room for backwards parallax push ({why_blocked}; "
                                                  "back+sides all blocked) (this path emits no bump)")
                        self._push_dir = None
                        self._settle_to = "REPLAN"
                        self._enter("SETTLE", now)
                        return {}, "SETTLE", (f"parallax backward blocked ({why_blocked}) -> no room "
                                              "back/left/right either -> settle -> replan")
                    self._push_dir = new_dir
                    self._push_after_reposition = new_after_repo
                    self._push_start_pos = plan.get("pos")
                    self._enter("PARALLAX_PUSH", now)          # reset the phase timer for the new direction
                    ev = pick_event or f"strafe {new_dir}"
                    return {}, "PARALLAX_PUSH", f"parallax backward blocked ({why_blocked}) -> {ev}"
                traveled = self._dist(plan.get("pos"), self._push_start_pos)
                if self._push_dir == "backward":
                    far, far_why = (traveled is not None and traveled >= self.parallax_push_dist), "dist"
                else:                                              # strafe: short TIMED hold, not distance
                    far, far_why = ((now - self.t_state) >= self.strafe_hold_s), "hold"
                blocked = guard is not None and guard <= self.parallax_min_clear
                timeout = (now - self.t_state) >= self.parallax_push_s
                if far or blocked or timeout:
                    why = far_why if far else "blocked" if blocked else "timer"
                    dirn = self._push_dir
                    self._log_move_push(dirn, now - self.t_state)
                    # A push STOPPED BY AN OBSTACLE is a real advance-blocked contact, but this path does NOT
                    # register a bump (behavior unchanged) -> mark it MISSED so the un-counted glass contacts
                    # are visible. (Whether to actually emit a bump here is a deferred behavior decision.)
                    if blocked and self.leg_goal is not None:
                        self._missed_bump = f"parallax {dirn} push blocked by obstacle (this path emits no bump)"
                    self._push_dir = None
                    self._settle_to = "REPLAN"
                    self._enter("SETTLE", now)
                    event = f"parallax {dirn} push done ({why}) -> settle -> replan"

        elif st == "SETTLE":
            # Two-gate settle (session 24): a settle that will fly TOWARD A GOAL (nxt REPLAN/REVERSE_PROBE/…)
            # must not proceed on a stale pose — both a clean rolling SLAM-freshness window AND a minimum
            # physical dwell (settle_gate_s) since the gate opened. The vertical prelude/calib routine
            # (TAKEOFF/ASCEND/DESCEND/BASELINE_NUDGE) is known-good and skips the freshness half (plain timer).
            # No timeout on a gated settle: if SLAM stops delivering, the plan status goes STALE/LOST and the
            # step() top diverts to recovery. `_enter("SETTLE")` opened the gate window (fresh, or carried over
            # from an antecedent SLAM_HOLD -- see `_enter`), so a resume from a stationary hold typically
            # passes on this very first tick.
            nxt = self._settle_to or "REPLAN"
            gated = nxt not in _SETTLE_EXEMPT_NXT
            if self._settle_gate_poll(now, require_fresh=gated):
                self._settle_to = None
                self._enter(nxt, now)
                if gated:
                    event = (f"settled: SLAM window clean ({self.settle_fresh_frames} frames <"
                             f"{self.slam_slow_ms:.0f}ms) + {self.settle_gate_s:.1f}s dwell -> {nxt}")

        elif st == "TRIM":
            # GRADUAL HEIGHT TRIM (session 14, RESTORED session 21). Pitch the aim UP + push forward -> the drone
            # flies toward the raised aim = a GRADUAL climb (joy_vertical is a discrete full-thrust axis; this is
            # the fine, dose-able vertical primitive). The forward part is translation = SLAM parallax. Sub-phases:
            #   (init/ring-gate) pick a safe direction to climb-FORWARD into: fwd open -> climb; else reposition
            #     (reverse to open fwd room; else strafe to an open side) -> climb; else ring blocked -> abort.
            #   REPOS -> short reverse/strafe to open forward room.  AIM -> hold pitch up (aim to highest).
            #   FWD   -> forward push WITH pitch up (the climb); a wall/ram contact aborts it (Trap A: guards stay
            #            active).  RESET -> pulse 'c' to reset the aim.  WAIT -> hold until a HEALTHY frame
            #            CAPTURED >= _trim_cmd_t0 + trim_settle_s (review-C async-SLAM guard: cap_ts is compared
            #            to the CLIMB-COMMAND instant on the same monotonic clock, so a stale pre-TRIM frame that
            #            arrives out of order can never satisfy the gate), LOG the post-trim height, then re-aim
            #            (ORIENT) at the PRESERVED goal.
            ring = plan.get("clearance_ring")
            clr = plan.get("forward_clearance_dist")
            if self._trim_phase is None:                        # ring-gate on entry
                def _open(c):    # None (unmapped near-field) == open room (the _pushable convention)
                    return c is None or c > self.stop_clearance_dist
                self._trimming = True
                left_c, right_c = self._ring_get(ring, -90.0), self._ring_get(ring, 90.0)
                sag_ratio = ((self._trim_sag_y - self._ceiling_y) / self._trim_delta
                             if self._trim_delta else float("nan"))
                if _open(clr):
                    self._trim_repos_move, ring_txt = None, "fwd-open"
                elif _open(self._ring_get(ring, 180.0)):
                    self._trim_repos_move, ring_txt = {"reverse": self.reverse_throttle}, "reverse-to-open-fwd"
                elif _open(left_c) or _open(right_c):
                    # strafe toward the OPEN (roomier) side; joy_horizontal -1 = left, +1 = right.
                    go_left = _open(left_c) and (not _open(right_c) or left_c is None
                                                 or (right_c is not None and left_c >= right_c))
                    sign = -1.0 if go_left else 1.0
                    self._trim_repos_move = {"joy_horizontal": sign * self._strafe_mag}
                    ring_txt = "strafe-left-to-open" if go_left else "strafe-right-to-open"
                else:
                    # Ring blocked all sides -> can't safely climb-forward. Abort (VISIBLE), resume the leg.
                    event = (f"TRIM abort: sag pos_y={self._trim_sag_y:+.3f} (ratio {sag_ratio:.2f}) but ring "
                             "blocked fwd+back+sides -> skip trim (pray); resume")
                    event = self._trim_exit(now, plan, event)
                    return active, self.state, event
                self._trim_phase, self._trim_phase_t0 = ("REPOS" if self._trim_repos_move else "AIM"), now
                if self._trim_dir == "UP":
                    event = (f"TRIM enter (UP): sag pos_y={self._trim_sag_y:+.3f} > desired "
                             f"{self._desired_y:+.3f} + {self.trim_sag_ratio - 1.0:.1f}*delta "
                             f"(delta={self._trim_delta:.3f}, ratio {sag_ratio:.2f}) -> climb via {ring_txt}")
                else:
                    event = (f"TRIM enter (DOWN): high pos_y={self._trim_sag_y:+.3f} < desired "
                             f"{self._desired_y:+.3f} - {self.trim_high_ratio:.1f}*delta "
                             f"(delta={self._trim_delta:.3f}, ratio {sag_ratio:.2f}) -> descend via {ring_txt}")
            elif self._trim_phase == "REPOS":
                active = dict(self._trim_repos_move)
                if (now - self._trim_phase_t0) >= self.trim_reposition_s:
                    self._trim_phase, self._trim_phase_t0 = "AIM", now
            elif self._trim_phase == "AIM":
                # raise the aim to its highest (UP) or drop it to its lowest (DOWN — session 22 mirror)
                active = {"pitch": (self.trim_pitch_up if self._trim_dir == "UP" else -self.trim_pitch_up)}
                if (now - self._trim_phase_t0) >= self.trim_aim_s:
                    self._trim_cmd_t0 = now                     # climb command issued (settle-gate origin, monotonic)
                    self._trim_phase, self._trim_phase_t0 = "FWD", now
            elif self._trim_phase == "FWD":
                # forward push WITH pitch still aimed = fly toward the raised/lowered aim = gradual climb or
                # descent. Trap A: a wall/ram contact aborts the push straight to RESET (guards stay ACTIVE).
                active = dict(self.forward_preset)
                active["trigger"] = self.trim_throttle
                active["pitch"] = (self.trim_pitch_up if self._trim_dir == "UP" else -self.trim_pitch_up)
                contact = wall_contact or (clr is not None and clr <= self.stop_clearance_dist)
                if contact or (now - self._trim_phase_t0) >= self.trim_fwd_s:
                    self._trim_phase, self._trim_phase_t0 = "RESET", now
                    if contact:
                        event = f"TRIM: contact during {self._trim_dir} push -> abort push -> reset aim"
            elif self._trim_phase == "RESET":
                active = {"btnCdown": True}                     # pulse 'c' to reset the aim/attitude
                if (now - self._trim_phase_t0) >= self.trim_reset_s:
                    self._trim_phase, self._trim_phase_t0 = "WAIT", now
            else:   # WAIT: hold neutral until a fresh HEALTHY post-trim frame, then log + exit
                cap_ts = plan.get("cap_ts")
                healthy = plan.get("plan_valid") and plan.get("pos_y") is not None and not self._slam_slow
                ready = (self._trim_cmd_t0 is not None and cap_ts is not None
                         and cap_ts >= self._trim_cmd_t0 + self.trim_settle_s and healthy)
                if ready:
                    y_after = float(plan["pos_y"])
                    ratio = ((y_after - self._ceiling_y) / self._trim_delta) if self._trim_delta else float("nan")
                    msg = (f"TRIM done ({self._trim_dir}): post pos_y={y_after:+.3f} (desired "
                           f"{self._desired_y:+.3f}, delta_to_desired={y_after - self._desired_y:+.3f}, "
                           f"ratio {ratio:.2f})")
                    event = self._trim_exit(now, plan, msg)
                    return active, self.state, event

        elif st == "RETURN_TO_ORIGIN":
            # Postlude leg 1: home to the take-off origin [0,0] (SLAM frame) at the current mapping height, via a
            # turn -> SETTLE -> advance -> SETTLE -> re-aim mini-loop. The SETTLEs (fresh-frame gated) are the fix
            # for the "turning like a maniac" ending: never re-aim or advance on a just-turned/just-moved stale
            # pose. Bounded by home_max_s -> proceed HERE (NO SILENT FALLBACK: logged). Reaching within
            # home_reach_dist -> ORIENT_HOME. A plan loss diverts to POSTLUDE_LOST_HOLD (step() top).
            pos = plan.get("pos")
            if self._home_phase is None:                   # lazy init on entry
                self._home_phase, self._home_t0 = "PLAN", now
                self._home_adv_start_pos = None
            reached = self._dist(pos, [0.0, 0.0])
            if reached is not None and reached <= self.home_reach_dist:
                self._player = None
                self._orient_home_phase = None
                self._enter("ORIENT_HOME", now)
                event = f"reached origin (d={reached:.2f}) -> ORIENT_HOME (face the take-off heading)"
            elif (now - self._home_t0) >= self.home_max_s:
                self._player = None
                self._orient_home_phase = None
                self._enter("ORIENT_HOME", now)
                event = (f"RETURN_TO_ORIGIN home_max_s ({self.home_max_s:.0f}s) cap — couldn't reach origin, "
                         "proceeding HERE (VISIBLE; no silent fallback) -> ORIENT_HOME")
            elif self._home_phase == "PLAN":
                # Aim at the origin from the LIVE pose+heading; hold this tick if the pose isn't trustworthy.
                if plan.get("plan_valid") and pos is not None and plan.get("heading_deg") is not None:
                    bearing = math.degrees(math.atan2(0.0 - pos[0], 0.0 - pos[1]))   # 0=+Z, +90=+X
                    be = ((bearing - float(plan["heading_deg"]) + 180.0) % 360.0) - 180.0   # wrap to (-180,180]
                    theta = self._quantize_turn(be)
                    if self.clamp_leg_turn:
                        theta = max(-self.turn_step_deg, min(self.turn_step_deg, theta))
                    self._player = self._build_turn(theta)
                    self._home_phase = "TURN"
                    event = f"homing: aim at origin, turn {theta:+.0f} deg (err {self._fmt(be)})"
                else:
                    event = "homing: pose invalid -> hold (wait for SLAM)"
            elif self._home_phase == "TURN":
                active, tdone = self._player.fields(now)
                if tdone:
                    self._player = None
                    self._home_phase, self._home_settle_to = "SETTLE", "ADVANCE"   # settle before advancing
                    self._settle_begin(now)
            elif self._home_phase == "SETTLE":
                # Let SLAM re-lock after the turn / advance before the next action (postlude flavor: wait for fresh
                # CAPTURE, not fast — a genuine loss diverts to POSTLUDE_LOST_HOLD at the step() top).
                sdone, _cap = self._settle_poll(now, plan, require_fast=False,
                                                min_frames=self.settle_fresh_frames, max_hold_s=None)
                if sdone:
                    if self._home_settle_to == "ADVANCE":
                        self._home_phase, self._home_adv_t0 = "ADVANCE", now
                        self._home_adv_start_pos = pos
                    else:
                        self._home_phase = "PLAN"
            elif self._home_phase == "BACKOFF":
                # A clearance stop while homing gets the SAME physical reaction as the main explore ADVANCE ->
                # BACKOFF path (session 20260720): reverse off the wall before settling/re-aiming, instead of
                # sitting flush against it and re-aiming from the same pinned pose (the "hopeless bounce" bug —
                # a wall-jammed drone can't hold a clean SLAM lock long enough to ever re-plan a way around it).
                active, bdone = self._player.fields(now)
                if bdone:
                    self._player = None
                    self._home_phase, self._home_settle_to = "SETTLE", "PLAN"
                    self._settle_begin(now)
            else:   # ADVANCE: push forward toward the aim for a bounded sub-leg, then SETTLE -> re-aim (PLAN)
                clr = plan.get("forward_clearance_dist")
                blocked = self.stop_on_clearance and clr is not None and clr <= self.stop_clearance_dist
                moved = self._dist(pos, self._home_adv_start_pos)
                reaim = moved is not None and moved >= self.goal_reach_dist
                adv_timeout = (now - self._home_adv_t0) >= self.leg_max_s
                if blocked:
                    self._home_adv_start_pos = None
                    self._player = self.pb.player("back_off")
                    self._home_phase = "BACKOFF"
                    event = f"homing: wall ahead (clr {clr:.2f}) -> back off -> settle -> re-aim toward origin"
                elif reaim or adv_timeout or self._slam_slow:
                    self._home_adv_start_pos = None
                    self._home_phase, self._home_settle_to = "SETTLE", "PLAN"   # settle, then re-aim
                    self._settle_begin(now)
                else:
                    active = dict(self.forward_preset)
                    y = plan.get("pos_y")
                    if (self.altitude_lock and self.target_altitude_y is not None and y is not None
                            and y > self.target_altitude_y + self.alt_drift_floor):
                        active["joy_vertical"] = self.ascend_preset["joy_vertical"]   # -1 = up (camera Y down)

        elif st == "ORIENT_HOME":
            # Postlude leg 1b: face the recorded take-off heading before the final dock — a controlled reverse of
            # take-off. Open-loop <=turn_step_deg turns with a SETTLE between (no spin on a stale pose), then ->
            # DOCK_FLOOR. Clears the flying-height altitude lock on the handoff so the descent can't be fought /
            # re-inflated. If no take-off heading was ever captured, skip straight to the dock (VISIBLE).
            if self._orient_home_phase is None:
                self._orient_home_phase = "PLAN"
            if self._takeoff_heading is None:
                self._dock_phase = None
                self.target_altitude_y = None
                self._enter("DOCK_FLOOR", now)
                event = "ORIENT_HOME: no take-off heading recorded -> DOCK_FLOOR"
            elif self._orient_home_phase == "PLAN":
                if plan.get("plan_valid") and plan.get("heading_deg") is not None:
                    be = ((self._takeoff_heading - float(plan["heading_deg"]) + 180.0) % 360.0) - 180.0
                    theta = self._quantize_turn(be)
                    if self.clamp_leg_turn:
                        theta = max(-self.turn_step_deg, min(self.turn_step_deg, theta))
                    if abs(theta) < 1e-6:                   # within half a turn step of the take-off heading -> dock
                        self._dock_phase = None
                        self.target_altitude_y = None       # drop the flying-height lock before descending
                        self._enter("DOCK_FLOOR", now)
                        event = f"ORIENT_HOME: facing take-off heading (err {self._fmt(be)}) -> DOCK_FLOOR"
                    else:
                        self._player = self._build_turn(theta)
                        self._orient_home_phase = "TURN"
                        event = f"ORIENT_HOME: turn {theta:+.0f} deg toward take-off heading (err {self._fmt(be)})"
                else:
                    event = "ORIENT_HOME: pose invalid -> hold (wait for SLAM)"
            elif self._orient_home_phase == "TURN":
                active, tdone = self._player.fields(now)
                if tdone:
                    self._player = None
                    self._orient_home_phase = "SETTLE"
                    self._settle_begin(now)
            else:   # SETTLE -> re-check the heading error (turn again or dock)
                sdone, _cap = self._settle_poll(now, plan, require_fast=False,
                                                min_frames=self.settle_fresh_frames, max_hold_s=None)
                if sdone:
                    self._orient_home_phase = "PLAN"

        elif st == "DOCK_FLOOR":
            # Postlude leg 2: a gentle PULSED (two-phase) descent to the floor — the MIRROR of the two-phase
            # ascent. A continuous hold-down is FORBIDDEN: rapid downward acceleration stretches vertical
            # visual features and chokes SLAM right at mission end. Phase 1 (micro-pulse approach): short DOWN
            # pulses separated by rests; after each rest read the live SLAM descent gain dZ = cur_y - prev_y
            # (+Y is DOWN so a SINKING drone's pos_y INCREASES) and keep pulsing while still sinking. Phase 2
            # (flow latch): once the gain flattens (flush on the floor, near-zero momentum), a single
            # CONTINUOUS DOWN hold long enough to latch a CLEAN, low-velocity FLOOR. dock_max_s is the
            # fail-safe (FLOOR is NEW/unvalidated) -> log + proceed. Reuses the ascend gain/stall/latch knobs.
            if self._dock_phase is None:                   # lazy init on entry
                self._dock_phase, self._dock_phase_t0 = "PULSE", now
                self._dock_prev_y, self._dock_stall_count = None, 0
                self._dock_start_t = now
                self.target_altitude_y = None              # drop the flying-height lock (can't re-inflate the descent; NOLOCK-gated)
            if (now - self._dock_start_t) > self.dock_max_s:
                self._dock_phase = None
                self._enter("LOW_STANDOFF", now)
                event = (f"dock cap ({self.dock_max_s:.0f}s, no FLOOR latch) -> LOW_STANDOFF "
                         "(VISIBLE WARN; FLOOR detection is new/unvalidated)")
            elif self._dock_phase == "LATCH":
                active = {"joy_vertical": 1}               # continuous DOWN; the flow FLOOR detector is authoritative
                y = plan.get("pos_y") if plan.get("plan_valid") else None
                if floor_contact:
                    self._dock_phase = None
                    self._enter("LOW_STANDOFF", now)
                    event = "FLOOR latched (flush, low-velocity) -> LOW_STANDOFF"
                elif (y is not None and self._dock_prev_y is not None
                      and (y - self._dock_prev_y) > self.ascend_gain_eps):
                    # Still sinking during the hold -> the Phase-1 stall was spurious -> resume micro-pulses.
                    self._dock_phase, self._dock_phase_t0 = "PULSE", now
                    self._dock_stall_count, self._dock_prev_y = 0, y
                    event = "dock LATCH but still sinking (spurious stall) -> back to micro-pulses"
                elif (now - self._dock_phase_t0) >= self.ascend_latch_hold_s:
                    self._dock_phase = None
                    self._enter("LOW_STANDOFF", now)
                    event = "dock LATCH hold elapsed, no flow latch (rested on floor) -> LOW_STANDOFF"
            elif self._dock_phase == "PULSE":
                active = {"joy_vertical": 1}               # a short DOWN micro-pulse (near-zero momentum)
                if (now - self._dock_phase_t0) >= self.dock_pulse_s:
                    self._dock_phase, self._dock_phase_t0 = "REST", now
            else:   # REST: neutral (momentum bleeds); at the end sample the SLAM descent gain this cycle
                if (now - self._dock_phase_t0) >= self.dock_rest_s:
                    valid = plan.get("plan_valid") and plan.get("pos_y") is not None and not self._slam_slow
                    if not valid:
                        self._dock_phase_t0 = now         # no trustworthy pose -> PAUSE (hold); dock_max_s backstops
                        event = "dock: pose invalid/slow -> pause (hold) until SLAM recovers"
                    else:
                        y = float(plan["pos_y"])
                        dz = None if self._dock_prev_y is None else (y - self._dock_prev_y)   # +Y down: sinking => +dz
                        self._dock_prev_y = y
                        if dz is not None and dz <= self.ascend_gain_eps:
                            self._dock_stall_count += 1
                        else:
                            self._dock_stall_count = 0
                        if self._dock_stall_count >= self.ascend_stall_cycles:
                            self._dock_phase, self._dock_phase_t0 = "LATCH", now
                            event = (f"dock: descent gain flattened (dZ<={self.ascend_gain_eps}) "
                                     f"x{self._dock_stall_count} -> Phase 2 continuous latch hold")
                        else:
                            self._dock_phase, self._dock_phase_t0 = "PULSE", now

        elif st == "LOW_STANDOFF":
            # Postlude leg 3: a short UP nudge to clear the ground safely, then stand by low. joy_vertical
            # -1 = UP (camera Y is DOWN). floor_standoff_nudge is a general platform behavior duration.
            active = dict(self.ascend_preset)              # {"joy_vertical": -1} = UP
            if (now - self.t_state) >= self.floor_standoff_nudge:
                self._enter("DONE", now)
                event = "low stand-off up-nudge done -> DONE (standby at low height)"

        elif st == "DONE":
            # Postlude complete: hold neutral (hover) at the low stand-off. One-shot VISIBLE annunciation.
            if not self._done_logged:
                self._done_logged = True
                event = "EXPLORE COMPLETE -> STANDBY AT LOW HEIGHT"

        return active, self.state, event


# Which flow event the detector tests for is derived from the ACTUALLY-commanded control vector (the command
# held during the just-elapsed frame interval), NOT a static state map — so it arms the right detector for
# UP (CEILING), FORWARD (WALL), or BACKWARD (BACKWALL) across every state, including a bidirectional REWIND
# whose direction a state map couldn't know. Priority reverse > forward > up: an ADVANCE with an altitude-lock
# up-inject still reads as FORWARD (its primary motion). Yaw-only / neutral / DOWN -> None (idle).
def _detector_command(active):
    if not active:
        return None
    if float(active.get("reverse", 0.0) or 0.0) > 0.0:
        return CMD_BACK
    if float(active.get("trigger", 0.0) or 0.0) > 0.0:
        return CMD_FWD
    if float(active.get("joy_vertical", 0.0) or 0.0) < 0.0:   # -1 = up (camera Y down) -> CEILING
        return CMD_UP
    if float(active.get("joy_vertical", 0.0) or 0.0) > 0.0:   # +1 = down (camera Y down) -> FLOOR (postlude dock)
        return CMD_DOWN
    return None


# Recovery states (SLAM-loss). The step() top snaps out of these to a brake+REPLAN when the plan returns OK.
_RECOVERY_STATES = {"HOLD_LOST", "REWIND", "FALLBACK", "STUCK", "WARMUP"}

# Post-mission ending states. A plan loss WHILE in one of these diverts to the dedicated POSTLUDE_LOST_HOLD
# (mirror of CALIB_LOST_HOLD) instead of the generic recovery, so the ending survives a SLAM loss and resumes.
POSTLUDE_STATES = {"RETURN_TO_ORIGIN", "ORIENT_HOME", "DOCK_FLOOR", "LOW_STANDOFF"}
# Postlude states where the flying-height altitude lock must be OFF (the descent + standby) — clearing
# target_altitude_y on DOCK_FLOOR entry could otherwise be re-cached next tick and re-inflate a floor-level drone.
# RETURN_TO_ORIGIN / ORIENT_HOME are NOT here: they home + orient AT altitude, lock on.
_POSTLUDE_NOLOCK = {"DOCK_FLOOR", "LOW_STANDOFF", "DONE", "POSTLUDE_LOST_HOLD"}

# SESSION 18: the old MAPPING_ALT_STATES state-gate for the altitude baseline is retired. The baseline now
# measures the live pos_y once per FRESH SLAM frame in ANY state, gated only by `_height_calibrated` (first
# calibration done) and NOT `_calib_active` (frozen during a calibration) — see the ingest at ExploreController.step.

# States from which a gradual-height TRIM may fire (session 14, RESTORED session 21). SETTLE is the pre-REPLAN /
# settled-SLAM_HOLD rest point; ADVANCE is the operator's explicit request. TRIM itself is absent (no re-entry).
_TRIM_TRIGGER_STATES = {"SETTLE", "ADVANCE"}

# SETTLE targets EXEMPT from the session-15 fresh-frame gate: the vertical prelude/calibration routine, which is
# known-good and left on the plain timed settle. Every other target (REPLAN/REVERSE_PROBE/…) flies toward a
# goal, so its settle must wait for fresh SLAM frames first.
_SETTLE_EXEMPT_NXT = {"TAKEOFF", "ASCEND", "DESCEND", "BASELINE_NUDGE"}


class _FileStopEvent:
    """A stop_event whose `is_set()` reports the presence of a sentinel FILE, so a separate launcher
    process can request a GRACEFUL shutdown of a child in its own console. On Windows a parent cannot
    deliver a console Ctrl+C/Ctrl+Break to a child created with CREATE_NEW_CONSOLE (separate console), so
    signal-based teardown would hard-kill the loop and skip the `finally` (losing the shutdown-emitted map
    backdrop). Polling a sentinel path lets run_explore exit its loop NORMALLY -> `finally` runs -> the map
    is written + diag is closed. Mirrors the threading.Event `.is_set()` the loop already checks."""
    def __init__(self, path):
        self._path = path

    def is_set(self):
        return self._path is not None and os.path.exists(self._path)


def _stuck_summary(intervals):
    """One-line mission-end summary of every STUCK episode (D4): wall-time ranges + durations, so an unattended
    flight's log records WHEN the drone gave up + recovered without a per-tick 'stuck' spam."""
    if not intervals:
        return "no STUCK episodes."
    parts = [f"{a.strftime('%H:%M:%S.%f')[:-3]}-{b.strftime('%H:%M:%S.%f')[:-3]} ({(b - a).total_seconds():.1f}s)"
             for a, b in intervals]
    return f"was STUCK {len(intervals)}x: " + "; ".join(parts)


def run_explore(cfg, stop_event=None, log=False, no_takeoff=False):
    """Bus wrapper around ExploreController: SUB frames (:frame_bus_port, flow WALL detector) + the
    explore plan (TOPIC_PLAN on :perception_state_port); PUB TOPIC_CONTROL. Enable on io_bridge with
    'm' (any manual flight key aborts to manual). Arms + takes off automatically (the prelude) unless
    `no_takeoff`. Needs io_bridge + perception_worker running."""
    ascend_cmd = int(cfg["autonomy"]["ascend_cmd"])
    e = (cfg["autonomy"].get("explore") or {})
    plan_timeout_s = float(e.get("plan_timeout_s", 2.0))
    detector = detector_from_cfg(cfg)
    ctrl = ExploreController(cfg, no_takeoff=no_takeoff)

    frame_port = cfg["network"]["frame_bus_port"]
    pstate_port = cfg["network"]["perception_state_port"]
    ctrl_port = cfg["network"]["autonomy_control_port"]
    pub_dt = 0.05   # 20 Hz — within io_bridge cmd_timeout

    pub = frame_bus.StatePublisher(ctrl_port)
    sub = frame_bus.FrameSubscriber(frame_port)
    plan_sub = frame_bus.StateSubscriber(pstate_port, topics=[frame_bus.TOPIC_PLAN])
    diag = AutopilotLog(log)

    print(f"[autopilot][explore] MAP MODE. PUB TOPIC_CONTROL :{ctrl_port} | SUB frames :{frame_port} "
          f"+ TOPIC_PLAN :{pstate_port}")
    print("[autopilot][explore] " + ("--no-takeoff: assuming the drone is ALREADY airborne; no arm/takeoff."
          if no_takeoff else "Will ARM + TAKE OFF automatically (same recipes as the mission), then explore."))
    print("[autopilot][explore] On io_bridge press 'm' to hand control over; any flight key aborts. "
          "REQUIRES perception_worker running (it publishes the frontier plan).")

    seq = 0
    bump_seq = 0          # dedup id for TOPIC_AUTOPILOT_EVENT bump pulses (perception drops repeats)
    pick_seq = 0          # dedup id for TOPIC_AUTOPILOT_EVENT pick+hop-outcome pulses (goals-DB)
    giveup_seq = 0        # dedup id for TOPIC_AUTOPILOT_EVENT corner-giveup pulses (force_retire_corner)
    last_pub = last_log = 0.0
    last_plan = None
    last_plan_t = time.monotonic()
    last_rec_frame = None
    enabled = False
    was_enabled = False
    announced_wait = warned_no_auto = False
    last_status = None
    last_cmd_key = None
    last_label = None
    # [TRIGGER] tracking (diagnostic session): the exact wall-time the forward-push command engages/
    # releases, tracked purely from the published command vector -- decoupled from FSM state or SLAM.
    # `_trig_release_t` derives a "hop-end" marker at release + 1.0s ease-down (a fixed diagnostic bound
    # to compare against the pose used to judge the hop, NOT io_bridge's actual ramp/decay physics, which
    # settles much faster).
    _trig_on = False
    _trig_release_t = None
    prev_ctrl_state = None
    prev_active = {}      # last published control vector -> derives the detector command for THIS frame
    backwall_active = False   # BACKWALL contact edge tracker (log once per onset)
    # SLAM_TRACKER: last pose we surfaced (frame_id/pos/heading/wall-time) so each fresh perception pose can
    # be recorded to the replay timeline with its dx/dy/dYaw + staleness gap. See the drain below.
    _slam_fid = None
    _slam_pos = None
    _slam_hd = None
    _slam_t = None
    # Diagnostic session: two INDEPENDENT strictly-consecutive counters. `_slam_seq_last` mirrors
    # perception_worker.py's Pipeline.step-owned `slam_seq` (one increment per actual SLAM invocation) --
    # a gap here means THIS process (autopilot) dropped a published plan in its own "drain to freshest"
    # loop below. `_slam_last_cap` is the previous frame's cap_ts, used only to report the NDI
    # camera-frame gap's time span (context, not a bug by itself -- CONFLATE dropping camera frames while
    # SLAM is busy is expected).
    _slam_seq_last = None
    _slam_last_cap = None
    last_ground = None    # newest GroundGrid summary; the final room outline is emitted ONCE at shutdown as
                          # a static backdrop (we don't replay the map growing — only the pose + goals matter)
    # D4 (session 12): graceful STUCK + bounded log. A drone parked in STUCK (or standing by in DONE) would emit
    # a per-step timeline/line record every tick forever -> a 200GB log of "stuck / SLAM alive" for an unattended
    # flight. Track each STUCK interval's wall-time [start,end], PAUSE the per-step spam while parked in STUCK
    # (one entry record, then quiet; resume on recovery), and at mission-end DONE log a summary INCLUDING the
    # stuck ranges, then turn per-step logging OFF (the shutdown map backdrop still emits in `finally`).
    prev_state_d4 = None      # state on the previous iteration (STUCK-enter/leave + DONE edge detection)
    stuck_intervals = []      # list of (start_wall, end_wall) datetimes for every STUCK episode
    stuck_start_wall = None   # open STUCK interval start (None = not currently stuck)
    logging_off = False       # set True once at mission-end DONE -> suppress further per-step records

    def diag_event(ev_kind, msg):
        """Diagnostic-session shared emitter: prints + diag-logs a line (terminal/autopilot.log visibility,
        matching every other event in this loop) AND writes a matching `diag.timeline()` record so the
        SAME event shows up in the replay HTML (flight_replay.py's ALL_EVENTS) -- not just the plain-text
        log, which is where the previous pass of this diagnostic left it (a real gap, per the operator)."""
        line = f"{_rec_prefix(last_rec_frame)} [autopilot][explore] {msg}"
        print(line, flush=True)
        diag.line(line)
        diag.timeline({"t_wall": now_wall.strftime("%H:%M:%S.%f")[:-3], "t_mono": round(now, 3),
                       "ev_kind": ev_kind, "msg": msg})

    def log_cmd(active, source):
        nonlocal last_cmd_key, _trig_on, _trig_release_t
        key = (source, json.dumps(active, sort_keys=True))
        if key == last_cmd_key:
            return
        last_cmd_key = key
        line = (f"{_rec_prefix(last_rec_frame)} [autopilot][explore][CMD] state={source} "
                f"fields={json.dumps(active, sort_keys=True)}")
        print(line, flush=True)
        diag.line(line)
        diag.cmd(last_rec_frame, seq, source, source, active)
        # [TRIGGER] engage/release edge tracking (diagnostic session) -- purely off the published command
        # vector, independent of which FSM state produced it.
        on_now = bool(active.get("trigger_down")) or float(active.get("trigger", 0.0) or 0.0) > 0.0
        if not _trig_on and on_now:
            _trig_on = True
            diag_event("trigger_event", "[TRIGGER] engaged (trigger_down/trigger>0)")
        elif _trig_on and not on_now:
            _trig_on = False
            _trig_release_t = time.monotonic()
            diag_event("trigger_event", "[TRIGGER] released")

    def publish(active, state):
        nonlocal seq, last_pub
        now = time.monotonic()
        if (now - last_pub) >= pub_dt:
            log_cmd(active, state)
            pub.publish(frame_bus.TOPIC_CONTROL, _full_vector(active, seq, now, state))
            seq += 1
            last_pub = now

    try:
        while stop_event is None or not stop_event.is_set():
            # Capture the monotonic clock AND the wall clock at ONE instant at the loop top, and use both for
            # every timeline row emitted this iteration (the SLAM records + the step row). The earlier ~1 ms
            # skew was a benign single-frame poll effect (t_mono snapshotted here, t_wall written later), NOT a
            # compounding tracking offset — replay still sorts by t_mono; this just unifies the capture instant.
            now = time.monotonic()
            now_wall = datetime.now()
            # [TRIGGER] derived "hop-end" marker (diagnostic session): fires exactly once, 1.0s after the
            # forward-push command was released -- a fixed diagnostic ease-down bound to compare the pose
            # used for hop judgment against, independent of state/SLAM.
            if _trig_release_t is not None and now - _trig_release_t >= 1.0:
                _trig_release_t = None
                diag_event("trigger_event", "[TRIGGER] hop-end (release + 1.0s ease-down)")
            msg = sub.recv(timeout_ms=20)
            frame = meta = None
            if msg is not None:
                frame, meta = msg
                if meta.get("rec_frame") is not None:
                    last_rec_frame = meta.get("rec_frame")
            # drain the plan bus to the freshest message. `planner_event` is TRANSIENT (perception clears it
            # after one plan), so capture it DURING the drain — otherwise draining to the freshest could skip
            # the event-carrying plan and lose the blacklist/reset marker.
            pending_planner_event = None
            p = plan_sub.recv(timeout_ms=0)
            while p is not None:
                last_plan = p[1]
                last_plan_t = now
                pe = last_plan.get("planner_event")
                if pe:
                    pending_planner_event = pe
                p = plan_sub.recv(timeout_ms=0)

            # ---- PAIRED SLAM logging: for every FRESH pose the controller accepts, synthesize TWO timeline
            # records (into the REPLAY HTML event log — NOT the live terminal), keyed on the perception
            # frame_id. Each record's DISPLAYED wall-time matches WHERE it sits in the timeline so nothing reads
            # ahead of its playback position (the earlier bug: labeling both with `now_wall`, the ~0.6s-later
            # processing instant). START sits at the frame's CAPTURE instant (cap_ts -> cap_wall); FINISH sits
            # at NOW (the log/completion instant -> now_wall) and references the capture time inline. The literal
            # string carries its own bracketed time, so the record's `t_wall` is "" (renderer prepends nothing).
            # dx/dy are horizontal FLOOR motion (world X and Z; vertical is Y), off the prev pose. ----
            if last_plan is not None:
                _fid = last_plan.get("frame_id")
                if _fid is not None and _fid != _slam_fid:
                    _pos = last_plan.get("pos")
                    _hd = last_plan.get("heading_deg")
                    _sms = last_plan.get("slam_ms")
                    _cap = last_plan.get("cap_ts")
                    _seq = last_plan.get("slam_seq")
                    _lat = f"{float(_sms):.0f}" if isinstance(_sms, (int, float)) else "—"
                    if _pos is not None and _slam_pos is not None:
                        # NOT clamped: after a SLAM loss+recover the true massive jump is useful drift data.
                        _dtxt = f"dx: {_pos[0] - _slam_pos[0]:+.2f} dy: {_pos[1] - _slam_pos[1]:+.2f}"
                    else:
                        _dtxt = "dx: — dy: —"          # first frame / TRACKING just back online (seed the tracker)
                    # Capture wall-time from cap_ts via the loop-top monotonic->wall offset (cap_ts None on a
                    # dropped frame -> fall back to `now` so t_mono is never None; the span just collapses).
                    _cap_t = _cap if _cap is not None else now
                    _cap_wall = (now_wall - timedelta(seconds=(now - _cap_t))).strftime("%H:%M:%S.%f")[:-3]
                    _now_wall = now_wall.strftime("%H:%M:%S.%f")[:-3]
                    diag.timeline({
                        "t_wall": "", "t_mono": round(_cap_t, 3), "ev_kind": "slam_start",
                        "slam": f"[{_cap_wall}] SLAM had currently began working on frame #{_seq}. (NDI: #{_fid})",
                        "frame_id": _fid, "slam_ms": _sms, "slam_seq": _seq,
                    })
                    diag.timeline({
                        "t_wall": "", "t_mono": round(now, 3), "ev_kind": "slam_finish",
                        "slam": (f"[{_now_wall}]. SLAM had just finished working on the frame #{_seq} "
                                 f"(NDI: #{_fid}) from: [{_cap_wall}]. The deltas are: ({_dtxt}) "
                                 f"Latency: {_lat}ms."),
                        "frame_id": _fid, "slam_ms": _sms, "slam_seq": _seq,
                    })
                    # [SLAM_TRACKER]/[SLAM_GAP] (diagnostic session): two INDEPENDENT strictly-consecutive
                    # counters, printed to the terminal + autopilot.log (not just the replay HTML) so a
                    # skip of either kind is impossible to miss. `slam_seq` gaps prove THIS process dropped
                    # a published plan in the drain loop above; `frame_id` (NDI) gaps just report how many
                    # raw camera frames CONFLATE discarded while SLAM was busy (expected, not a bug).
                    _ndi_gap = ""
                    if _slam_fid is not None and _fid != _slam_fid + 1 and _slam_last_cap is not None:
                        _ndi_gap = (f" (NDI camera-frame gap: +{_fid - _slam_fid}, "
                                    f"{(_cap_t - _slam_last_cap) * 1000:.0f}ms span)")
                    _trkline = (f"{_rec_prefix(last_rec_frame)} [autopilot][explore] [SLAM_TRACKER] "
                                f"frame #{_seq} processed (NDI: #{_fid}, cap_ts={_cap_t:.3f}, "
                                f"slam_ms={_sms}, prev seq=#{_slam_seq_last}, prev NDI=#{_slam_fid})"
                                f"{_ndi_gap}")
                    print(_trkline, flush=True)
                    diag.line(_trkline)
                    if _seq is not None and _slam_seq_last is not None and _seq != _slam_seq_last + 1:
                        diag_event("slam_gap",
                                   f"[SLAM_GAP] autopilot DROPPED a published plan. Expected slam_seq "
                                   f"#{_slam_seq_last + 1}, received #{_seq} "
                                   f"(missed {_seq - _slam_seq_last - 1} plan publish(es)).")
                    _slam_seq_last, _slam_last_cap = _seq, _cap_t
                    _slam_fid, _slam_t = _fid, now
                    if _pos is not None:
                        _slam_pos = _pos
                    if _hd is not None:
                        _slam_hd = _hd

            # ---- autonomy gate (mirror run_mission: only fly while io_bridge reports AUTO) ----
            if meta is not None:
                stt = (meta.get("controls") or {}).get("autonomy")
                if stt is None:
                    if not warned_no_auto:
                        print("[autopilot][explore] WARNING: frame meta has no controls.autonomy — "
                              "HOLDING. Restart io_bridge with the current code.", flush=True)
                        warned_no_auto = True
                    enabled = False
                else:
                    enabled = (stt != "MANUAL")
            if not enabled:
                if was_enabled:
                    print("[autopilot][explore] autonomy OFF -> PAUSED (press 'm' to resume).", flush=True)
                elif not announced_wait:
                    print("[autopilot][explore] waiting for autonomy enable ('m' on io_bridge) ...", flush=True)
                    announced_wait = True
                was_enabled = False
                ctrl.reset_leg()
                prev_active, backwall_active = {}, False   # no command held while paused
                if frame is not None:
                    detector.update(now, frame, None)   # keep prev_gray fresh
                publish({}, "WAIT")
                continue
            if not was_enabled:
                print("[autopilot][explore] autonomy LIVE -> executing the frontier plan.", flush=True)
                was_enabled = True

            # ---- plan health (visibility; NO SILENT FALLBACK) ----
            # The status is passed into the controller, which owns the CONTROL-SPACE SLAM-loss recovery:
            #   PLAN-LOST/NO-PLAN (perception silent) -> HARD HOVER-HOLD indefinitely (no blind recovery);
            #   PLAN-STALE (SLAM not TRACKING) -> RECOVERY_REWIND (retrace) -> parallax+<=45 fallback;
            #   OK -> normal flight (and snap out of recovery). The prelude is exempt (it needs no plan).
            status = _plan_status(last_plan, now - last_plan_t, plan_timeout_s)
            if status != last_status:
                print(f"[autopilot][explore] plan status: {status} (plan_age={now - last_plan_t:.2f}s)", flush=True)
                diag.line(f"{_rec_prefix(last_rec_frame)} [autopilot][explore] plan status: {status}")
                last_status = status
            plan_for_step = last_plan if last_plan is not None else {}

            # ---- flow contact detection (command derived from the ACTUAL last-published control vector) ----
            wall_contact = ceiling_contact = floor_contact = backwall_contact = False
            if frame is not None:
                command = _detector_command(prev_active)   # UP (CEILING) / FWD (WALL) / BACK (BACKWALL) / DOWN (FLOOR) / None
                v = detector.update(now, frame, command)
                if command in (CMD_FWD, CMD_UP, CMD_BACK, CMD_DOWN):
                    if v.label() != last_label or (now - last_log) >= 0.5:
                        line = f"{_rec_prefix(last_rec_frame)} {_verdict_line(f'[autopilot][explore][{ctrl.state}]', v)}"
                        print(line, flush=True)
                        diag.line(line)
                        diag.row(last_rec_frame, meta, v)
                        last_label, last_log = v.label(), now
                    if v.contact and v.kind == "WALL" and command == CMD_FWD:
                        wall_contact = True
                    if v.contact and v.kind == "CEILING" and command == CMD_UP:
                        ceiling_contact = True
                    if v.contact and v.kind == "FLOOR" and command == CMD_DOWN:
                        floor_contact = True
                    # BACKWALL: reverse commanded, but the image shows we've stopped moving -> a wall behind us
                    # that SLAM's clearance ring may not have mapped yet. Fed into ctrl.step() so PARALLAX_PUSH /
                    # REVERSE_PROBE can react (retry a side / stop early) instead of grinding a blind timer.
                    now_backwall = bool(v.contact and v.kind == "BACKWALL" and command == CMD_BACK)
                    if now_backwall:
                        backwall_contact = True
                    if now_backwall and not backwall_active:
                        line = (f"{_rec_prefix(last_rec_frame)} [autopilot][explore][{ctrl.state}] BACKWALL "
                                f"contact (reverse into a wall)")
                        print(line, flush=True)
                        diag.line(line)
                    backwall_active = now_backwall
                else:
                    backwall_active = False

            # ---- step the controller + publish ----
            active, state, event = ctrl.step(now, plan_for_step, wall_contact, ceiling_contact,
                                             floor_contact=floor_contact, backwall_contact=backwall_contact,
                                             status=status)
            if state == "ADVANCE" and prev_ctrl_state != "ADVANCE":
                detector.reset_forward_ref()   # each leg recalibrates its own free-forward looming
            prev_ctrl_state = state
            prev_active = active               # command for the NEXT frame's detector + the bump re-arm test
            # 2-bump latch: re-arm once the drone has disengaged (backward cmd OR moved > goal_reach_dist),
            # then publish any pending bump pulse for the planner's event-driven blacklist.
            ctrl.rearm_bump_if_disengaged(active, plan_for_step)
            bump_goal, bump_reason, bump_pos, bump_is_corner = ctrl.take_bump_pulse()
            if bump_goal is not None:
                pub.publish(frame_bus.TOPIC_AUTOPILOT_EVENT, {"bump_goal": bump_goal, "seq": bump_seq,
                                                              "bump_pos": bump_pos,
                                                              "bump_is_corner": bool(bump_is_corner)})
                bline = (f"{_rec_prefix(last_rec_frame)} [autopilot][explore] BUMP pulse #{bump_seq} "
                         f"goal={bump_goal} ({bump_reason} -> planner)")
                print(bline, flush=True)
                diag.line(bline)
                bump_seq += 1
            # Session 24: a far-corner give-up escalation (corner_giveup_limit strikes, still never close
            # enough for a real 2-bump) -> force-retire that corner via the planner (mark visited, tour moves
            # on). Deduped like the bump pulse.
            giveup_goal = ctrl.take_corner_giveup_pulse()
            if giveup_goal is not None:
                pub.publish(frame_bus.TOPIC_AUTOPILOT_EVENT,
                            {"corner_giveup_goal": giveup_goal, "giveup_seq": giveup_seq})
                gline = (f"{_rec_prefix(last_rec_frame)} [autopilot][explore] CORNER-GIVEUP pulse #{giveup_seq} "
                         f"goal={giveup_goal} (corner_giveup_limit hit -> planner force-retires it)")
                print(gline, flush=True)
                diag.line(gline)
                giveup_seq += 1
            # Goals-DB pick + previous-hop STRIKE/progress outcome, stashed at the REPLAN leg-commit. One pulse
            # per leg; perception drains it into the planner's goals-DB (loop + stall guards). Deduped by seq.
            pick = ctrl.take_pick_pulse()
            if pick is not None:
                pub.publish(frame_bus.TOPIC_AUTOPILOT_EVENT, dict(pick, pick_seq=pick_seq))
                pick_seq += 1
            # A real advance-blocked contact that emitted NO pulse (latch disarmed / parallax-blocked path) —
            # these are the un-counted glass contacts that let the 2-bump blacklist under-count. Surface them.
            missed = ctrl.take_missed_bump()
            if missed is not None:
                mline = f"{_rec_prefix(last_rec_frame)} [autopilot][explore] MISSED-BUMP: {missed}"
                print(mline, flush=True)
                diag.line(mline)
            # One-shot operator notices (session 22: e.g. the height-reference disagreement warning) — LOUD.
            notice = ctrl.take_notice()
            if notice is not None:
                nline = f"{_rec_prefix(last_rec_frame)} [autopilot][explore] *** {notice} ***"
                print(nline, flush=True)
                diag.line(nline)
            # Diagnostic session: [HOP_BASELINE]/[HOP_JUDGE] position-state monitoring.
            hop_baseline = ctrl.take_hop_baseline_msg()
            if hop_baseline is not None:
                diag_event("hop_baseline", hop_baseline)
            hop_judge = ctrl.take_hop_judge_msg()
            if hop_judge is not None:
                diag_event("hop_judge", hop_judge)
            # The planner's bump outcome (count climb / goal-change RESET / BLACKLIST), computed in the
            # perception process, mirrored into the flight diag so the mechanism is no longer invisible.
            if pending_planner_event is not None:
                eline = f"{_rec_prefix(last_rec_frame)} [autopilot][explore] PLANNER: {pending_planner_event}"
                print(eline, flush=True)
                diag.line(eline)
            if event:
                line = f"{_rec_prefix(last_rec_frame)} [autopilot][explore] {state}: {event}"
                print(line, flush=True)
                diag.line(line)
            publish(active, state)

            # ---- D4 (session 12): STUCK-interval memory + log-spam pause + mission-end summary/logging-off ----
            stuck_entry = (state == "STUCK" and prev_state_d4 != "STUCK")
            if stuck_entry:
                stuck_start_wall = now_wall
                sline = (f"{_rec_prefix(last_rec_frame)} [autopilot][explore] STUCK: recovery exhausted -> "
                         "STANDBY (per-step logging PAUSED; resumes on recovery, summarized at mission end)")
                print(sline, flush=True); diag.line(sline)
            elif state != "STUCK" and prev_state_d4 == "STUCK" and stuck_start_wall is not None:
                stuck_intervals.append((stuck_start_wall, now_wall))    # recovered -> close the interval, resume log
                rline = (f"{_rec_prefix(last_rec_frame)} [autopilot][explore] recovered from STUCK "
                         f"(~{(now_wall - stuck_start_wall).total_seconds():.1f}s) -> logging resumed")
                print(rline, flush=True); diag.line(rline)
                stuck_start_wall = None
            if state == "DONE" and prev_state_d4 != "DONE" and not logging_off:
                if stuck_start_wall is not None:                        # defensively close an open interval
                    stuck_intervals.append((stuck_start_wall, now_wall)); stuck_start_wall = None
                mline = (f"{_rec_prefix(last_rec_frame)} [autopilot][explore] MISSION COMPLETE. "
                         f"{_stuck_summary(stuck_intervals)} -> per-step logging OFF (map backdrop still emitted at exit)")
                print(mline, flush=True); diag.line(mline)
                logging_off = True
            # Suppress the per-step timeline record while PARKED in STUCK (after its entry record) or after the
            # mission-end DONE — the two states that otherwise emit an identical record every tick forever.
            suppress_step = logging_off or (state == "STUCK" and not stuck_entry)
            prev_state_d4 = state

            # ---- F8 replay timeline (purely additive; --log-gated via the no-op sink) ----
            # ONE record per step (pose + goal states); the room outline is NOT streamed — we keep only the
            # newest ground summary and emit it once at shutdown as a static backdrop for the whole replay.
            if log and not suppress_step:
                t_wall = now_wall.strftime("%H:%M:%S.%f")[:-3]   # same instant as `now` (unified at the loop top)
                rec = _timeline_step_record(t_wall, now, last_rec_frame, state, event,
                                            status, plan_for_step, cmd=active,
                                            leg_goal=ctrl.leg_goal, plan_age_s=(now - last_plan_t),
                                            alt={"median": ctrl._alt_median,
                                                 "ceiling": ctrl._ceiling_y, "desired": ctrl._desired_y,
                                                 "delta": ctrl._trim_delta,
                                                 "trim_on": ctrl._trimming, "calib_on": ctrl._calib_active})
                # The transient planner_event was captured during the drain (the freshest plan may have
                # already cleared it) and the un-counted contact from the controller — stitch both onto THIS
                # step's record so the replay marks the exact frame of each.
                if pending_planner_event is not None:
                    rec["planner_event"] = pending_planner_event
                if missed is not None:
                    rec["missed_bump"] = missed
                # Live self-calibrated ram-guard speed telemetry (u/s) + the flight's calibrated nominal, so
                # the replay panel shows exactly why the ram guard did or didn't fire (crawl vs true stall).
                rec["speed"] = (round(ctrl._ram_speed, 4) if ctrl._ram_speed is not None else None)
                rec["nominal_speed"] = (round(ctrl._nominal_speed, 4) if ctrl._nominal_speed is not None else None)
                diag.timeline(rec)
                g = plan_for_step.get("ground")
                if g and g.get("bounds"):
                    last_ground = g
    except KeyboardInterrupt:
        print("\n[autopilot][explore] interrupted — sending a final HOLD (neutral).")
    finally:
        pub.publish(frame_bus.TOPIC_CONTROL, _full_vector({}, seq, time.monotonic(), "HOLD"))
        time.sleep(0.05)
        # Emit the final room outline ONCE, at t_mono=0 so it's the static backdrop under every step (the
        # viewer draws the newest map at/under the cursor). The drone + goal states animate over it.
        if log and last_ground is not None:
            m = _downsample_map(last_ground)
            if m is not None:
                diag.timeline({"t_mono": 0.0, "map": m})
        diag.close()
        pub.close()
        sub.close()
        plan_sub.close()


# ------------------------------------------------------------------ explore self-test helpers
def _drive(ctrl, plan, wall, seconds, t0, dt=0.05, ceiling=False, floor=False, status="OK"):
    """Step ExploreController over `seconds` at dt with a fixed plan/wall/ceiling/floor/status. Returns
    (t_end, last_active, last_state, states_visited). Injects an ADVANCING SLAM frame each tick (fresh
    frame_id + capture time + a fast default latency) so the session-15 SETTLE fresh-frame gate and
    _update_slam see a live healthy stream, exactly as in real flight."""
    states, active, state = [], {}, ctrl.state
    t = t0
    fid0 = int(plan.get("frame_id") or 1000)
    for i in range(max(1, int(seconds / dt))):
        p = dict(plan)
        p["frame_id"] = fid0 + i          # always advance the frame so the gate/streak see a live stream
        p.setdefault("cap_ts", t)         # respect an explicit cap_ts (incl. None, to test the hold path)
        p.setdefault("slam_ms", 200.0)
        active, state, _ev = ctrl.step(t, p, wall, ceiling, floor_contact=floor, status=status)
        if not states or states[-1] != state:
            states.append(state)
        t += dt
    return t, active, state, states


def _is_subsequence(needle, hay):
    """True if `needle` appears in order (not necessarily contiguous) within `hay`."""
    it = iter(hay)
    return all(x in it for x in needle)


# ==============================================================================
# Self-test: delegate the detection logic to flow_contact_detector + sanity-check the playbook player.
# ==============================================================================
def run_self_test(cfg):
    import flow_contact_detector
    ok = flow_contact_detector.run_self_test()

    # Session 21: the periodic goal-change re-calibration is a REAL trigger again — with review-A a fresh
    # controller (`_last_calib_t is None`, e.g. no-takeoff) may calibrate on its FIRST goal, which would divert
    # every unrelated leg/recovery test into CALIBRATING_HEIGHT. Isolate it harness-wide (like
    # `hop_duration_s = 0` in the ram tests); the dedicated PERIODIC-RECALIB tests below re-enable it explicitly.
    cfg = copy.deepcopy(cfg)
    cfg.setdefault("autonomy", {}).setdefault("explore", {})["calibrate_on_goal_change"] = False

    # F8 replay timeline: the JSONL sink is --log-gated, so a disabled AutopilotLog must swallow
    # .timeline()/.line() as no-ops (no file, no crash) — the path taken when self-test/dry constructs run.
    dl = AutopilotLog(False)
    try:
        dl.timeline({"state": "ADVANCE", "goals": []})
        dl.line("noop")
        tl_noop = (dl._jsonl is None and dl._txt is None)
    finally:
        dl.close()
    # And the pure record builders produce the expected shape. GOAL fields reflect the CONTROLLER's
    # committed leg_goal (not perception's async plan goal); staleness fields are exposed.
    plan = {"pos": [0.1, 0.2], "heading_deg": 45.0, "goal": [1.0, 2.0], "bearing_err": 3.0,
            "frame_id": 42, "blacklist": [[9.0, 9.0]], "blacklist_permanent": [True]}
    # (a) committed leg_goal == perception's goal: single `active` marker, no `plan_pick`.
    rec = _timeline_step_record("00:00:01.000", 1.234, 7, "ADVANCE", "leg", "OK", plan,
                                cmd={"trigger": 0.2}, leg_goal=[1.0, 2.0], plan_age_s=0.3)
    rec_hover = _timeline_step_record("00:00:01.000", 1.234, 7, "SETTLE", None, "OK", plan, cmd={},
                                      leg_goal=[1.0, 2.0], plan_age_s=0.3)
    # (b) committed leg_goal DIFFERS from perception's pick: `active`=leg_goal + faint `plan_pick`.
    rec_split = _timeline_step_record("00:00:01.000", 1.234, 7, "ADVANCE", None, "OK", plan,
                                      cmd={"trigger": 0.2}, leg_goal=[5.0, 5.0], plan_age_s=1.9)
    ds = _downsample_map({"bounds": [0, 4, 0, 4], "rows": 2, "cols": 2, "cls": [0, 1, 2, 3]})
    import math as _m
    goal_fields_ok = (rec["goal"] == [1.0, 2.0] and rec["plan_goal"] == [1.0, 2.0]
                      and rec["plan_bearing_err"] == 3.0 and rec["frame_id"] == 42
                      and rec["plan_age_s"] == 0.3
                      and abs(rec["dist_to_goal"] - _m.hypot(0.9, 1.8)) < 1e-3)
    markers_ok = (len(rec["goals"]) == 2 and rec["goals"][0]["state"] == "active"
                  and rec["goals"][0]["xz"] == [1.0, 2.0]
                  and rec["goals"][1]["state"] == "blacklist_permanent"
                  # committed != plan pick -> active(leg_goal) + plan_pick + blacklist = 3 markers
                  and len(rec_split["goals"]) == 3
                  and rec_split["goals"][0]["state"] == "active" and rec_split["goals"][0]["xz"] == [5.0, 5.0]
                  and rec_split["goals"][1]["state"] == "plan_pick" and rec_split["goals"][1]["xz"] == [1.0, 2.0])
    tl_rec = (rec["state"] == "ADVANCE" and rec["pos"] == [0.1, 0.2]
              and rec["cmd"] == {"trigger": 0.2} and rec_hover["cmd"] == {}   # {} hover preserved
              and goal_fields_ok and markers_ok
              and ds["rows"] == 2 and ds["cls"] == [0, 1, 2, 3])
    good = tl_noop and tl_rec
    ok = ok and good
    print(f"[self-test] {'PASS' if good else 'FAIL'}  F8 timeline (disabled sink no-op={tl_noop}, "
          f"record/map builders={tl_rec}, goal=leg_goal={goal_fields_ok}, markers={markers_ok})")

    # Playbook RecipePlayer sanity: step the (multi-step) arm recipe forward in time and confirm it
    # drives btnARMdown at some point and then completes. (One fields() call advances at most one step,
    # so a single far-future call wouldn't reach 'done' on a multi-step recipe — must step over time.)
    pb = FlightPlaybook.load()
    player = pb.player("arm")
    total = sum(float(s.get("duration_s", 0.0)) for s in pb.recipe("arm"))
    saw_arm, done = False, False
    t = 0.0
    while t <= total + 0.5:
        fields, done = player.fields(t)
        if fields.get("btnARMdown") is True:
            saw_arm = True
        t += 0.05
    good = saw_arm and done
    ok = ok and good
    print(f"[self-test] {'PASS' if good else 'FAIL'}  playbook arm recipe (multi-step) plays then completes")

    # Mission load + expansion sanity: default mission expands, auto-rests interleave (no two adjacent
    # non-rest steps), every step resolves, and an unknown step fails loudly.
    steps = expand_mission(load_mission(), pb)
    no_adjacent = all(not (steps[i]["type"] != "rest" and steps[i + 1]["type"] != "rest")
                      for i in range(len(steps) - 1))
    try:
        expand_mission({"steps": ["fly_to_the_moon"]}, pb)
        rejected = False
    except ValueError:
        rejected = True
    inline_steps = expand_mission({"steps": [{"joy_vertical": 1, "duration_s": 0.17}]}, pb)
    inline_ok = (len(inline_steps) == 1 and inline_steps[0]["type"] == "inline"
                 and inline_steps[0]["fields"] == {"joy_vertical": 1}
                 and inline_steps[0]["seconds"] == 0.17)
    good = len(steps) > 0 and no_adjacent and rejected and inline_ok
    ok = ok and good
    print(f"[self-test] {'PASS' if good else 'FAIL'}  mission expands ({len(steps)} steps), auto-rests "
          f"interleaved, unknown step rejected, inline step parses")

    # ---- Forward-clearance raycast (MapStore.clearance): synthetic wall; the fan catches an off-center
    # wall a single center ray would thread past. Pure numpy (no SLAM/GPU). ----
    import numpy as np
    from map_store import MapStore

    def _wall(xr, z, nx=61, ny=21):
        xs, ys = np.linspace(xr[0], xr[1], nx), np.linspace(-0.4, 0.4, ny)  # ny odd -> includes Y=0
        X, Y = np.meshgrid(xs, ys)
        return np.column_stack([X.ravel(), Y.ravel(), np.full(X.size, z)])

    ms = MapStore(0.05)
    ms.integrate(_wall((-1.0, 1.0), 2.0)); ms.integrate(_wall((-1.0, 1.0), 2.0))  # 2 obs -> count >= min_count
    d_center = ms.clearance([0.0, 0.0, 0.0], 0.0)                 # heading 0 = +Z -> wall ~2.0u ahead
    center_ok = d_center is not None and abs(d_center - 2.0) < 0.12
    ms2 = MapStore(0.05)
    ms2.integrate(_wall((0.3, 1.0), 2.0)); ms2.integrate(_wall((0.3, 1.0), 2.0))  # wall ONLY off to +X
    d_single = ms2.clearance([0.0, 0.0, 0.0], 0.0, fan_n=1)       # center ray misses
    d_fan = ms2.clearance([0.0, 0.0, 0.0], 0.0, fan_n=3, fan_deg=15.0)  # +15deg ray catches it ~2.07u
    fan_ok = (d_single is None) and (d_fan is not None) and abs(d_fan - 2.0 / np.cos(np.radians(15))) < 0.2
    empty_ok = MapStore(0.05).clearance([0.0, 0.0, 0.0], 0.0) is None
    ray_ok = center_ok and fan_ok and empty_ok
    ok = ok and ray_ok
    print(f"[self-test] {'PASS' if ray_ok else 'FAIL'}  MapStore.clearance (center={d_center}, "
          f"off-center single={d_single}/fan={d_fan}, empty=None)")

    # ---- Map mode: plan-health classifier (degraded plan => HOLD, never coast) ----
    ps_ok = (_plan_status(None, 0.0, 2.0) == "NO-PLAN"
             and _plan_status({"plan_valid": True}, 5.0, 2.0) == "PLAN-LOST"
             and _plan_status({"plan_valid": False}, 0.1, 2.0) == "PLAN-STALE"
             and _plan_status({"plan_valid": True}, 0.1, 2.0) == "OK")
    ok = ok and ps_ok
    print(f"[self-test] {'PASS' if ps_ok else 'FAIL'}  plan-health classifier "
          f"(NO-PLAN / PLAN-LOST / PLAN-STALE / OK)")

    # ---- Map mode: ExploreController full leg (ORIENT[open-loop turn]->ADVANCE->BACKOFF->SETTLE->REPLAN->DONE) ----
    ctrl = ExploreController(cfg, no_takeoff=True)   # skip the prelude; this test covers the frontier loop
    ctrl.reverse_probe_on_wall = False               # this test covers the default back_off wall path
    goal = [3.0, 0.0]                                 # beyond goal_reach_dist so the WALL path (not goal-reached) runs
    order = []
    rec = lambda sts: [order.append(s) for s in sts if not order or order[-1] != s]
    t = 100.0
    plan_turn = {"done": False, "goal": goal, "pos": [0.0, 0.0], "bearing_err": 90.0}
    # REPLAN snapshots err=+90 -> quantized +90 open-loop turn; during it yaw must be POSITIVE (toward +X).
    t, a, s, st = _drive(ctrl, plan_turn, False, 0.3, t)
    rec(st)
    yaw_pos = (s == "ORIENT" and a.get("yaw", 0.0) > 0)
    # The open-loop turn plays to completion then -> ADVANCE (forward preset has a trigger).
    t, a, s, st = _drive(ctrl, plan_turn, False, 2.4, t)
    rec(st)
    advancing = (s == "ADVANCE" and float(a.get("trigger", 0)) > 0)
    # WALL contact -> BACKOFF -> SETTLE.
    t, a, s, st = _drive(ctrl, plan_turn, True, 0.05, t)
    rec(st)
    t, a, s, st = _drive(ctrl, plan_turn, False, 0.6, t)
    rec(st)
    # Frontiers exhausted during the settle window: a DONE plan must carry SETTLE -> REPLAN -> the postlude
    # (RETURN_TO_ORIGIN; pos=[0,0] so it reaches the origin immediately -> DOCK_FLOOR). Postlude coverage is
    # its own test below; here we only confirm the done-branch enters the postlude (not a static DONE hover).
    t, a, s, st = _drive(ctrl, {"done": True, "goal": None, "pos": [0.0, 0.0], "bearing_err": None}, False, ctrl.rest_between_s + 0.4, t)
    rec(st)
    leg_ok = (yaw_pos and advancing and ctrl.done
              and _is_subsequence(["ORIENT", "ADVANCE", "BACKOFF", "SETTLE", "REPLAN", "RETURN_TO_ORIGIN"], order))
    ok = ok and leg_ok
    print(f"[self-test] {'PASS' if leg_ok else 'FAIL'}  explore leg ORIENT(turn+)->ADVANCE->WALL->"
          f"BACKOFF->SETTLE->REPLAN->RETURN_TO_ORIGIN  (visited {order})")

    # ---- Map mode: POST-MISSION FLOOR-DOCK POSTLUDE (done -> RETURN_TO_ORIGIN -> DOCK_FLOOR(pulsed) ->
    #      LOW_STANDOFF(up-nudge) -> DONE), plus the home_max_s + dock_max_s safety caps. ----
    # Happy path: pos already at the origin so RETURN_TO_ORIGIN reaches immediately; DOCK_FLOOR descends in
    # gentle DOWN micro-pulses (joy_vertical=+1) until the descent gain flattens -> a LATCH hold where
    # floor_contact latches -> LOW_STANDOFF nudges UP (joy_vertical=-1) -> DONE.
    cpost = ExploreController(cfg, no_takeoff=True)
    plan_done = {"plan_valid": True, "done": True, "goal": None, "pos": [0.0, 0.0], "heading_deg": 0.0,
                 "bearing_err": None, "pos_y": 0.0, "forward_clearance_dist": 5.0}
    porder, prev_p = [], None
    saw_down_pulse = saw_up_nudge = False
    tp, dtp = 0.0, 0.05
    for _ in range(int(40.0 / dtp)):
        a, s, _ = cpost.step(tp, plan_done, False, floor_contact=True)   # floor_contact only acts in LATCH
        if s != prev_p:
            porder.append(s); prev_p = s
        if s == "DOCK_FLOOR" and a.get("joy_vertical") == 1:
            saw_down_pulse = True
        if s == "LOW_STANDOFF" and a.get("joy_vertical") == -1:
            saw_up_nudge = True
        if s == "DONE":
            break
        tp += dtp
    post_ok = (saw_down_pulse and saw_up_nudge and cpost.state == "DONE"
               and _is_subsequence(["RETURN_TO_ORIGIN", "DOCK_FLOOR", "LOW_STANDOFF", "DONE"], porder))
    # home_max_s cap: the drone is NOT at the origin and can't get there -> dock HERE (no infinite homing).
    chome = ExploreController(cfg, no_takeoff=True)
    chome.home_max_s = 0.0
    far_done = dict(plan_done, pos=[9.0, 9.0])
    _, _, _, sthome = _drive(chome, far_done, False, 0.3, 0.0)
    home_cap_ok = _is_subsequence(["RETURN_TO_ORIGIN", "DOCK_FLOOR"], sthome)
    # dock_max_s cap: the floor never latches (floor_contact False) -> still proceed to LOW_STANDOFF.
    cdock = ExploreController(cfg, no_takeoff=True)
    cdock.dock_max_s = 0.0
    _, _, _, stdock = _drive(cdock, plan_done, False, 0.5, 0.0, floor=False)
    dock_cap_ok = _is_subsequence(["RETURN_TO_ORIGIN", "DOCK_FLOOR", "LOW_STANDOFF"], stdock)
    # HOMING loop: the drone starts AWAY from the origin -> RETURN_TO_ORIGIN must aim (PLAN, bearing-wrap),
    # turn, SETTLE (fresh-frame gated — no re-aim/advance on a stale pose), ADVANCE (forward trigger), SETTLE,
    # re-aim. Simulate the pose closing on the origin whenever a forward push is commanded; confirm it reaches
    # DOCK_FLOOR, emitted a forward push, AND visibly SETTLED between homing actions. Inject a live frame stream
    # (like _drive) so the settles resolve.
    chomeloop = ExploreController(cfg, no_takeoff=True)
    chomeloop.rest_between_s = 0.1; chomeloop.settle_fresh_frames = 2
    hx, saw_home_push, reached_dock, saw_home_settle = 3.0, False, False, False
    th, fidh = 0.0, 5000
    hplan = {"plan_valid": True, "done": True, "goal": None, "heading_deg": 0.0, "bearing_err": None,
             "pos_y": 0.0, "forward_clearance_dist": 5.0}
    for _ in range(int(90.0 / 0.05)):
        fidh += 1
        a, s, _ = chomeloop.step(th, dict(hplan, pos=[hx, 0.0], frame_id=fidh, cap_ts=th, slam_ms=200.0), False)
        if s == "RETURN_TO_ORIGIN" and float(a.get("trigger", 0.0) or 0.0) > 0.0:
            saw_home_push = True
            hx = max(0.0, hx - 0.05)          # the forward push closes on the origin
        if s == "RETURN_TO_ORIGIN" and chomeloop._home_phase == "SETTLE":
            saw_home_settle = True
        if s == "DOCK_FLOOR":
            reached_dock = True
            break
        th += 0.05
    home_loop_ok = saw_home_push and reached_dock and saw_home_settle
    postlude_ok = post_ok and home_cap_ok and dock_cap_ok and home_loop_ok
    ok = ok and postlude_ok
    print(f"[self-test] {'PASS' if postlude_ok else 'FAIL'}  POSTLUDE (done->RETURN_TO_ORIGIN->ORIENT_HOME->"
          f"DOCK_FLOOR(down-pulse)->LOW_STANDOFF(up-nudge)->DONE={post_ok}, home_max_s cap={home_cap_ok}, "
          f"dock_max_s cap={dock_cap_ok}, homing turn+SETTLE+advance={home_loop_ok})  visited {porder}")

    # ---- Postlude session-16 additions: ORIENT_HOME bearing-wrap, DOCK survives a SLAM loss, no re-inflate ----
    # (a) ORIENT_HOME: at the origin with a take-off heading OFFSET from the current heading -> it must TURN
    #     toward the take-off heading (driving the bearing-wrap math), then dock. heading_deg=170, takeoff=-170:
    #     the short way is +20 (wrap), NOT -340.
    corient = ExploreController(cfg, no_takeoff=True)
    corient.rest_between_s = 0.1; corient.settle_fresh_frames = 2
    corient._takeoff_heading = -170.0
    saw_orient_turn = reached_dock2 = False
    to, fido, oh, last_turn, first_delta = 0.0, 6000, 170.0, None, None
    for _ in range(int(30.0 / 0.05)):
        fido += 1
        a, s, _ = corient.step(to, {"plan_valid": True, "done": True, "goal": None, "pos": [0.0, 0.0],
                                    "heading_deg": oh, "bearing_err": None, "pos_y": 0.0,
                                    "forward_clearance_dist": 5.0, "frame_id": fido, "cap_ts": to,
                                    "slam_ms": 200.0}, False)
        if s == "ORIENT_HOME" and corient._orient_home_phase == "TURN" and corient._player is not None:
            saw_orient_turn = True
            nm = corient._player.name                      # e.g. "turn+30" -> simulate the body rotating by that
            if nm != last_turn and nm.startswith("turn"):
                last_turn = nm
                delta = float(nm[4:])
                if first_delta is None:
                    first_delta = delta                    # short-way check: the FIRST turn is +20-ish (wrap), not -330
                oh = ((oh + delta + 180.0) % 360.0) - 180.0
        if s == "DOCK_FLOOR":
            reached_dock2 = True
            break
        to += 0.05
    orient_short_way = first_delta is not None and first_delta > 0
    orient_ok = saw_orient_turn and orient_short_way and reached_dock2

    # (b) DOCK survives a SLAM loss: reach DOCK_FLOOR healthy, then inject PLAN-LOST -> the DEDICATED
    #     POSTLUDE_LOST_HOLD (NOT HOLD_LOST / FALLBACK); then recover (OK + fast frames) -> resume DOCK_FLOOR.
    cdl = ExploreController(cfg, no_takeoff=True)
    dplan = {"plan_valid": True, "done": True, "goal": None, "pos": [0.0, 0.0], "heading_deg": 0.0,
             "bearing_err": None, "pos_y": 0.0, "forward_clearance_dist": 5.0}
    _drive(cdl, dplan, False, 0.6, 0.0, floor=False)             # settle into DOCK_FLOOR
    in_dock = cdl.state == "DOCK_FLOOR"
    # inject a plan loss (STALE): _update_slam needs fresh frames; status drives the divert
    tl, fidl = 5.0, 7000
    for _ in range(6):
        fidl += 1
        cdl.step(tl, dict(dplan, plan_valid=False, frame_id=fidl, cap_ts=tl, slam_ms=200.0), False, status="PLAN-STALE")
        tl += 0.05
    dock_diverts = cdl.state == "POSTLUDE_LOST_HOLD" and cdl._dock_interrupted
    # recover: status OK + >= calib_lost_recover_frames fresh fast frames -> resume DOCK_FLOOR
    for _ in range(cdl.calib_lost_recover_frames + 2):
        fidl += 1
        cdl.step(tl, dict(dplan, frame_id=fidl, cap_ts=tl, slam_ms=200.0), False, status="OK")
        tl += 0.05
    dock_resumes = cdl.state == "DOCK_FLOOR"
    dock_loss_ok = in_dock and dock_diverts and dock_resumes

    # (c) No re-inflate: once DOCK_FLOOR clears target_altitude_y, the step-top lock caching must NOT re-cache it
    #     (a floor-level drone would otherwise be shoved back up). Drive DOCK with a floor-level pose; assert the
    #     lock target stays None and no UP (joy_vertical=-1) is ever emitted in the descent.
    cri = ExploreController(cfg, no_takeoff=True)
    _drive(cri, dict(dplan, pos_y=0.02), False, 0.6, 0.0, floor=False)
    no_reinflate = cri.target_altitude_y is None
    tri, fidri = 5.0, 8000
    for _ in range(40):
        fidri += 1
        a, s, _ = cri.step(tri, dict(dplan, pos_y=0.02, frame_id=fidri, cap_ts=tri, slam_ms=200.0), False, floor_contact=False)
        if s in ("DOCK_FLOOR", "LOW_STANDOFF") and a.get("joy_vertical") == -1 and s == "DOCK_FLOOR":
            no_reinflate = False        # DOCK must never inject UP; LOW_STANDOFF's deliberate up-nudge is fine
        if cri.target_altitude_y is not None:
            no_reinflate = False
        tri += 0.05
    postlude2_ok = orient_ok and dock_loss_ok and no_reinflate
    ok = ok and postlude2_ok
    print(f"[self-test] {'PASS' if postlude2_ok else 'FAIL'}  POSTLUDE loss-survival + orient "
          f"(ORIENT_HOME short-way turn+dock={orient_ok}, DOCK survives loss->hold->resume={dock_loss_ok}, "
          f"no floor re-inflate={no_reinflate})")

    # ---- Map mode: reverse-probe EXPERIMENT (flag on) — clamp leg turn to ONE step; WALL -> reverse probe ----
    # With reverse_probe_on_wall: a big bearing err is clamped to ONE turn_step (SLAM stays alive at the
    # wall), and a WALL hit goes ADVANCE -> SETTLE -> REVERSE_PROBE (sustained reverse) -> SETTLE -> REPLAN
    # (NOT back_off). The BACKWALL detector arms in REVERSE_PROBE and can end the probe early on a live contact
    # (untested here — this case never fires it; see the dedicated REVERSE-PROBE-BACKWALL self-test below).
    cre = ExploreController(cfg, no_takeoff=True)
    cre.reverse_probe_on_wall = True
    plan_e = {"done": False, "goal": [1.0, 0.0], "pos": [0.0, 0.0], "bearing_err": 135.0}  # would be +135 (3 steps) unclamped
    eorder, prev_e = [], None
    saw_reverse, saw_backoff, turn_name = False, False, None
    te, dt, wall = 200.0, 0.05, False
    for _i in range(int(14.0 / dt)):
        if cre.state == "ADVANCE":
            wall = True                       # trip the wall the moment we start advancing
        p_e = dict(plan_e, frame_id=1000 + _i, cap_ts=te, slam_ms=200.0)   # live stream for the SETTLE gate
        a, s, _ = cre.step(te, p_e, wall, False)
        if s != prev_e:
            eorder.append(s)
            prev_e = s
        if s == "ORIENT" and turn_name is None:
            turn_name = cre._player.name      # clamped open-loop turn -> "turn+30", not "turn+135"
        if s == "REVERSE_PROBE" and float(a.get("reverse", 0.0)) > 0:
            saw_reverse = True
        if s == "BACKOFF":
            saw_backoff = True
        te += dt
    clamp_ok = (turn_name == "turn+30")       # +135 bearing clamped to one +30 turn_step
    rev_path_ok = _is_subsequence(["ORIENT", "ADVANCE", "SETTLE", "REVERSE_PROBE", "SETTLE", "REPLAN"], eorder)
    rev_ok = (clamp_ok and saw_reverse and rev_path_ok and not saw_backoff)
    ok = ok and rev_ok
    print(f"[self-test] {'PASS' if rev_ok else 'FAIL'}  explore REVERSE-PROBE (clamp +135->{turn_name}, "
          f"WALL->SETTLE->REVERSE_PROBE(reverse>0)->SETTLE->REPLAN, no back_off)  visited {eorder}")

    # (REVERSE-PROBE-BACKWALL) a live flow BACKWALL contact ends the probe EARLY, well before the recipe's
    # fixed 4.0s duration -- instead of only the natural timeout.
    cre2 = ExploreController(cfg, no_takeoff=True)
    cre2.reverse_probe_on_wall = True
    cre2._enter("REVERSE_PROBE", 0.0)
    plan_rp = {"plan_valid": True, "done": False, "goal": [1.0, 0.0], "pos": [0.0, 0.0], "bearing_err": 0.0}
    a0, s0, _ = cre2.step(0.0, plan_rp, False)                          # starts the reverse_probe recipe
    still_probing = (s0 == "REVERSE_PROBE" and float(a0.get("reverse", 0.0)) > 0)
    a1, s1, ev1 = cre2.step(0.05, plan_rp, False, backwall_contact=True)   # BACKWALL fires well before 4.0s
    rp_backwall_ok = (still_probing and s1 == "SETTLE" and "BACKWALL" in (ev1 or ""))
    ok = ok and rp_backwall_ok
    print(f"[self-test] {'PASS' if rp_backwall_ok else 'FAIL'}  explore REVERSE-PROBE-BACKWALL "
          f"(live contact ends probe early -> settle -> replan)")

    # ---- Map mode: forward-clearance STAND-OFF (primary forward stop; SLAM-preserving) ----
    # A mapped wall ahead within stop_clearance_dist stops the ADVANCE leg WITHOUT a wall_contact. With the
    # default backoff_on_standoff=True it routes ADVANCE -> BACKOFF (a small reverse that re-arms the 2-bump
    # latch so a stand-off pin can still blacklist an unreachable wall — Bug B) -> SETTLE; with the flag OFF
    # it settles directly. A large or None clearance keeps advancing. NB the clearance check is FIRST in
    # ADVANCE, so it acts before the image ever freezes.
    cs = ExploreController(cfg, no_takeoff=True)
    cs_on = cs.stop_on_clearance                          # config default true
    big = {"done": False, "goal": [3.0, 0.0], "pos": [0.0, 0.0], "bearing_err": 0.0, "forward_clearance_dist": 5.0}
    t, a, s, _ = _drive(cs, big, False, 0.6, 100.0)       # far clearance -> still advancing
    adv_big = (s == "ADVANCE" and float(a.get("trigger", 0)) > 0)
    near = dict(big, forward_clearance_dist=cs.stop_clearance_dist - 0.05)
    # default (backoff_on_standoff=True): standoff -> BACKOFF (reverse>0) -> SETTLE, no REVERSE_PROBE.
    bo_states, saw_rev_bo, tt = [], False, t
    for _ in range(40):                                   # ~2s: BACKOFF(0.3s) -> SETTLE
        a2, s2, _ev = cs.step(tt, near, False, False, status="OK")
        if not bo_states or bo_states[-1] != s2:
            bo_states.append(s2)
        if float((a2 or {}).get("reverse", 0.0) or 0.0) > 0.0:
            saw_rev_bo = True
        tt += 0.05
    backoff_path = (cs.backoff_on_standoff and s == "ADVANCE" and saw_rev_bo
                    and ("REVERSE_PROBE" not in bo_states)
                    and _is_subsequence(["BACKOFF", "SETTLE"], bo_states))
    # backoff_on_standoff=False: standoff settles directly (old behavior), no BACKOFF / reverse.
    cfg_off = {**cfg, "autonomy": {**cfg["autonomy"],
                                   "explore": {**(cfg["autonomy"].get("explore") or {}), "backoff_on_standoff": False}}}
    cs_off = ExploreController(cfg_off, no_takeoff=True)
    toff, _, _, _ = _drive(cs_off, big, False, 0.6, 100.0)
    _, _, s_off, st_off = _drive(cs_off, near, False, 0.2, toff)
    direct_settle = (not cs_off.backoff_on_standoff and s_off == "SETTLE"
                     and "BACKOFF" not in st_off and "REVERSE_PROBE" not in st_off)
    cn = ExploreController(cfg, no_takeoff=True)
    _, an, sn, _ = _drive(cn, dict(big, forward_clearance_dist=None), False, 0.6, 0.0)  # None -> keep advancing
    adv_none = (sn == "ADVANCE" and float(an.get("trigger", 0)) > 0)
    clr_ok = (cs_on and adv_big and backoff_path and direct_settle and adv_none)
    ok = ok and clr_ok
    print(f"[self-test] {'PASS' if clr_ok else 'FAIL'}  explore CLEARANCE-STOP (far->advance, "
          f"<{cs.stop_clearance_dist:g}->BACKOFF(reverse re-arm)->settle | flag-off->direct settle, None->advance)")

    # ---- forward_throttle override: the config knob sets the ADVANCE/parallax forward trigger ----
    ft_cfg = (cfg["autonomy"].get("explore") or {}).get("forward_throttle", None)
    cft = ExploreController(cfg, no_takeoff=True)
    preset_ok = (ft_cfg is None) or abs(float(cft.forward_preset.get("trigger", -1)) - float(ft_cfg)) < 1e-9
    pf = {"plan_valid": True, "done": False, "goal": [3.0, 0.0], "pos": [0.0, 0.0], "bearing_err": 0.0,
          "forward_clearance_dist": 5.0}
    _, af, sf, _ = _drive(cft, pf, False, 0.6, 0.0)       # the override value rides the live ADVANCE command
    drive_ok = (sf == "ADVANCE") and (ft_cfg is None or abs(float(af.get("trigger", -1)) - float(ft_cfg)) < 1e-9)
    ft_ok = preset_ok and drive_ok
    ok = ok and ft_ok
    print(f"[self-test] {'PASS' if ft_ok else 'FAIL'}  forward_throttle override (preset + ADVANCE trigger = {ft_cfg})")

    # ---- reverse_throttle override: the config knob rewrites the reverse magnitude in all reverse recipes ----
    rt_cfg = (cfg["autonomy"].get("explore") or {}).get("reverse_throttle", None)
    crt = ExploreController(cfg, no_takeoff=True)
    rev_ok = (rt_cfg is None) or (
        abs(float(crt.pb.recipe("back_off")[0]["reverse"]) - float(rt_cfg)) < 1e-9
        and abs(float(crt.pb.recipe("reverse_probe")[0]["reverse"]) - float(rt_cfg)) < 1e-9)
    ok = ok and rev_ok
    print(f"[self-test] {'PASS' if rev_ok else 'FAIL'}  reverse_throttle override (back_off + reverse_probe reverse = {rt_cfg})")

    # ---- _ring_get nearest-offset lookup (wrap-aware) ----
    _rg = ExploreController._ring_get
    _ring = [[0.0, 1.0], [45.0, 2.0], [180.0, 3.0], [-45.0, 4.0]]
    ringget_ok = (_rg(_ring, 0.0) == 1.0 and _rg(_ring, 44.0) == 2.0 and _rg(_ring, 179.0) == 3.0
                  and _rg(_ring, -44.0) == 4.0 and _rg(None, 0.0) is None and _rg([], 0.0) is None)
    ok = ok and ringget_ok
    print(f"[self-test] {'PASS' if ringget_ok else 'FAIL'}  _ring_get nearest-offset lookup (wrap-aware)")

    # ---- Map mode: ALTITUDE LOCK (hold the live-cached mapping height; +Y is DOWN so a sink = larger y) ----
    ca = ExploreController(cfg, no_takeoff=True)
    pA = {"plan_valid": True, "done": False, "goal": [3.0, 0.0], "pos": [0.0, 0.0], "bearing_err": 0.0,
          "pos_y": 0.0, "forward_clearance_dist": 5.0}            # at target -> no correction
    t, a, s, _ = _drive(ca, pA, False, 0.6, 0.0)
    cached = (ca.target_altitude_y == 0.0)                        # cached from first valid plan
    adv_noinj = (s == "ADVANCE" and "joy_vertical" not in a)
    pB = dict(pA, pos_y=ca.alt_drift_floor + 0.1)                 # sunk past the deadband -> inject UP
    _, aB, sB, _ = _drive(ca, pB, False, 0.2, t)
    adv_inj = (sB == "ADVANCE" and aB.get("joy_vertical") == -1 and float(aB.get("trigger", 0)) > 0)
    _, aC, sC, _ = _drive(ca, dict(pA, pos_y=0.0), False, 0.2, t)  # back at target -> override clears
    adv_clear = (sC == "ADVANCE" and "joy_vertical" not in aC)
    alt_ok = (cached and adv_noinj and adv_inj and adv_clear)
    ok = ok and alt_ok
    print(f"[self-test] {'PASS' if alt_ok else 'FAIL'}  explore ALTITUDE-LOCK (cache target, inject UP when "
          f"sunk > {ca.alt_drift_floor:g}, clear at target)")

    # ---- Map mode: PARALLAX SCOUT (multi-step turn -> turn, then translate for parallax, then turn again) ----
    open_ring = [[r, 5.0] for r in (0.0, 45.0, 90.0, 135.0, 180.0, -135.0, -90.0, -45.0)]

    def _plan_be(be, pos=(0.0, 0.0), ring=open_ring, fcd=5.0):
        return {"plan_valid": True, "done": False, "goal": [0.0, 1.0], "pos": list(pos),
                "bearing_err": be, "pos_y": 0.0, "forward_clearance_dist": fcd, "clearance_ring": ring}
    # (a) goal needs MORE than one step (135 deg) -> turn THEN parallax push (not straight to ADVANCE).
    ca1 = ExploreController(cfg, no_takeoff=True)
    _, _, _, st1 = _drive(ca1, _plan_be(135.0), False, 2.0, 0.0)
    multi_push = _is_subsequence(["ORIENT", "PARALLAX_PUSH"], st1) and ("ADVANCE" not in st1)
    # (b) goal within one step (30 deg) -> turn THEN advance, no push.
    ca2 = ExploreController(cfg, no_takeoff=True)
    _, _, _, st2 = _drive(ca2, _plan_be(30.0), False, 1.5, 0.0)   # >1 turn duration so ORIENT completes -> ADVANCE
    aim_adv = ("ADVANCE" in st2) and ("PARALLAX_PUSH" not in st2)
    # (c) with an open ring the push picks BACKWARD (never forward) and is distance-quantized: it ends by
    #     'dist' once translated parallax_push_dist (before the time cap), commanding reverse_throttle.
    cd = ExploreController(cfg, no_takeoff=True)
    cd._enter("PARALLAX_PUSH", 0.0)
    cd._push_dir = None
    tt, moved, ended, push_rev = 0.0, 0.0, None, None
    for _ in range(400):
        a, s, _ = cd.step(tt, _plan_be(90.0, pos=(0.0, moved)), False)
        if s != "PARALLAX_PUSH":
            ended = (moved, tt)
            break
        if a.get("reverse") is not None:      # backward push magnitude actually commanded
            push_rev = a["reverse"]
        moved += 0.05                         # drone translates 0.05u/tick -> reaches 0.5u well before the cap
        tt += 0.05
    dist_stop = (ended is not None and ended[0] >= cd.parallax_push_dist - 1e-6 and ended[1] < cd.parallax_push_s
                 and push_rev is not None and abs(push_rev - cd.reverse_throttle) < 1e-9)
    # (d) boxed in (back+sides all a tight FINITE < min_clear) -> skip the push (enter PARALLAX_PUSH but bail).
    cb = ExploreController(cfg, no_takeoff=True)
    tight = [[r, 0.5] for r in (0.0, 45.0, 90.0, 135.0, 180.0, -135.0, -90.0, -45.0)]
    _, _, _, stb = _drive(cb, _plan_be(135.0, ring=tight), False, 2.0, 0.0)
    boxed_skip = ("PARALLAX_PUSH" in stb) and (cb._push_count == 0)
    # (e) backward blocked (rel 180 tight) but a SIDE open -> STRAFE toward the open side (+joy_horizontal, never forward).
    ce = ExploreController(cfg, no_takeoff=True)
    ce._enter("PARALLAX_PUSH", 0.0)
    ce._push_dir = None
    ring_side = [[0.0, 0.5], [90.0, 5.0], [180.0, 0.5], [-90.0, 0.5]]   # right (rel +90) open
    a_e, _, _ = ce.step(0.0, _plan_be(90.0, ring=ring_side), False)
    strafe_ok = (ce._push_dir == "strafe_right" and a_e.get("joy_horizontal", 0.0) > 0
                 and "trigger" not in a_e)                              # never forward
    # (f) MISS is room: a direction reading None is pushable (not skip). back=None -> backward push.
    cf2 = ExploreController(cfg, no_takeoff=True)
    cf2._enter("PARALLAX_PUSH", 0.0)
    cf2._push_dir = None
    ring_none = [[0.0, 0.5], [90.0, 0.5], [180.0, None], [-90.0, 0.5]]  # back unmapped -> open near-field
    cf2.step(0.0, _plan_be(90.0, ring=ring_none), False)
    miss_room_ok = (cf2._push_dir == "backward")
    scout_ok = (multi_push and aim_adv and dist_stop and boxed_skip and strafe_ok and miss_room_ok)
    ok = ok and scout_ok
    print(f"[self-test] {'PASS' if scout_ok else 'FAIL'}  explore PARALLAX-SCOUT (multi-step->turn+push, "
          f"aim->advance, back dist-stop@{ended[0] if ended else '?'}, boxed->skip, strafe-on-side, miss=room)")

    # ---- Map mode: PARALLAX_PUSH backward-blocked mid-push retry (BACKWALL flow contact + ring catch-up) ----
    # (g) ring never maps the back (stays None/"open" all along -- the ring guard alone would NEVER stop this
    #     push), but the live flow BACKWALL detector fires mid-push: hand off to the roomier side IN PLACE
    #     (same episode, no settle/replan/re-orient), never re-try backward this episode.
    ring_g1 = [[0.0, 0.5], [90.0, 5.0], [180.0, None], [-90.0, 0.5]]   # back unmapped, right open, left tight
    cg1 = ExploreController(cfg, no_takeoff=True)
    cg1._enter("PARALLAX_PUSH", 0.0); cg1._push_dir = None
    cg1.step(0.0, _plan_be(90.0, ring=ring_g1), False)                       # entry: backward (miss = room)
    picked_backward = (cg1._push_dir == "backward")
    a1, s1, _ = cg1.step(0.05, _plan_be(90.0, ring=ring_g1), False, backwall_contact=True)  # flow fires
    handoff_ok = (s1 == "PARALLAX_PUSH" and cg1._push_dir == "strafe_right")
    a2, s2, _ = cg1.step(0.10, _plan_be(90.0, ring=ring_g1), False)          # next tick: actually strafing
    strafe_cmd_ok = (s2 == "PARALLAX_PUSH" and a2.get("joy_horizontal", 0.0) > 0
                     and "reverse" not in a2 and "trigger" not in a2)
    g1_ok = picked_backward and handoff_ok and strafe_cmd_ok
    # (h) same, but BOTH sides are also tight -> give up for real (settle -> replan), missed-bump mentions why.
    ring_g2 = [[0.0, 0.5], [90.0, 0.5], [180.0, None], [-90.0, 0.5]]   # back unmapped, BOTH sides tight
    cg2 = ExploreController(cfg, no_takeoff=True)
    cg2._enter("PARALLAX_PUSH", 0.0); cg2._push_dir = None
    cg2.leg_goal = [0.0, 1.0]                                          # so a missed-bump gets stashed
    cg2.step(0.0, _plan_be(90.0, ring=ring_g2), False)
    a1h, s1h, _ = cg2.step(0.05, _plan_be(90.0, ring=ring_g2), False, backwall_contact=True)
    missed = cg2.take_missed_bump()
    g2_ok = (s1h == "SETTLE" and cg2._push_dir is None
             and missed is not None and "no room" in missed and "back+sides" in missed)
    # (i) the EXISTING ring-based mid-push block (no flow contact at all) now ALSO retries a side instead of
    #     bailing straight to settle/replan: back reads open at entry, then the ring "catches up" to tight.
    ring_open_back = [[0.0, 0.5], [90.0, 5.0], [180.0, 5.0], [-90.0, 0.5]]
    # 0.5: blocked (<= parallax_min_clear 0.7) but NOT inside the D2 scrape-danger zone (< 0.4) -- isolates the
    # plain ring-catch-up retry from the D2 reposition-forward branch (covered separately by g1/g2 above).
    ring_now_tight = [[0.0, 0.5], [90.0, 5.0], [180.0, 0.5], [-90.0, 0.5]]
    cg3 = ExploreController(cfg, no_takeoff=True)
    cg3._enter("PARALLAX_PUSH", 0.0); cg3._push_dir = None
    cg3.step(0.0, _plan_be(90.0, ring=ring_open_back), False)
    a1i, s1i, _ = cg3.step(0.05, _plan_be(90.0, ring=ring_now_tight), False)   # ring-only block, no flow
    ring_retry_ok = (s1i == "PARALLAX_PUSH" and cg3._push_dir == "strafe_right")
    retry_ok = (g1_ok and g2_ok and ring_retry_ok)
    ok = ok and retry_ok
    print(f"[self-test] {'PASS' if retry_ok else 'FAIL'}  explore PARALLAX-PUSH backward-blocked retry "
          f"(flow contact->strafe in-place={g1_ok}, both sides also tight->give up={g2_ok}, "
          f"ring-only block also retries a side={ring_retry_ok})")

    # (j) give-up MEMORY: a full give-up latches the position; the NEXT pick at ~the same spot must NOT retry
    #     backward even if the ring (falsely) reads it as open; moving parallax_min_clear away clears it.
    cg4 = ExploreController(cfg, no_takeoff=True)
    ring_boxed = [[0.0, 0.5], [90.0, 0.5], [180.0, 0.5], [-90.0, 0.5]]        # everywhere tight -> give up
    d0, _, _ = cg4._pick_ring_direction(ring_boxed, _plan_be(0.0, pos=(0.0, 0.0), ring=ring_boxed))
    latched_ok = (d0 is None and cg4._parallax_back_blocked
                  and cg4._parallax_back_blocked_anchor == [0.0, 0.0])
    ring_back_open = [[0.0, 0.5], [90.0, 5.0], [180.0, 5.0], [-90.0, 0.5]]    # back (falsely) reads open now
    d1, _, _ = cg4._pick_ring_direction(ring_back_open, _plan_be(0.0, pos=(0.05, 0.0), ring=ring_back_open))
    suppressed_ok = (d1 == "strafe_right" and cg4._parallax_back_blocked)    # near the anchor -> backward denied
    d2, _, _ = cg4._pick_ring_direction(ring_back_open, _plan_be(0.0, pos=(0.0, 1.0), ring=ring_back_open))
    cleared_ok = (d2 == "backward" and not cg4._parallax_back_blocked)      # far from the anchor -> latch clears
    memory_ok = (latched_ok and suppressed_ok and cleared_ok)
    ok = ok and memory_ok
    print(f"[self-test] {'PASS' if memory_ok else 'FAIL'}  explore PARALLAX-PUSH give-up MEMORY "
          f"(latches={latched_ok}, suppresses nearby={suppressed_ok}, clears once moved away={cleared_ok})")

    # (SESSION-17: the GRADUAL HEIGHT TRIM self-test was DELETED along with the TRIM feature.)

    # ---- Map mode: GRADUAL HEIGHT TRIM (session 14, restored 21) — sag trigger, ring-gate, climb, WAIT, goal keep ----
    def _mk_trim():
        c = ExploreController(cfg, no_takeoff=True)
        c._ceiling_y, c._desired_y, c._trim_delta = -2.3, -1.9, 0.4   # threshold = -2.3 + 1.2*0.4 = -1.82
        c.trim_aim_s = c.trim_fwd_s = 0.1
        c.trim_reset_s, c.trim_reposition_s, c.trim_settle_s = 0.05, 0.1, 0.2
        return c

    def _tplan(posy, pos=(0.0, 0.0), fcd=5.0, ring=None, cap=0.0):
        return {"plan_valid": True, "done": False, "goal": [3.0, 0.0], "pos": list(pos),
                "bearing_err": 0.0, "heading_deg": 0.0, "pos_y": posy, "slam_ms": 200.0, "frame_id": 1,
                "cap_ts": cap, "forward_clearance_dist": fcd,
                "clearance_ring": ring if ring is not None else [[0.0, 5.0], [180.0, 5.0], [90.0, 5.0], [-90.0, 5.0]]}
    # (a) sag in ADVANCE fires TRIM, snapshots the committed goal (Trap B) AND clears the pending per-hop eval
    #     (session 20b: a trim-interrupted hop must not be judged); at desired height it does NOT fire.
    ta = _mk_trim(); ta._enter("ADVANCE", 0.0); ta.leg_goal = [3.0, 0.0]
    ta._hop_start_goal = [3.0, 0.0]; ta._hop_start_dist = 3.0
    _, sA, _ = ta.step(0.0, _tplan(-1.7), False)                     # sunk below -1.82 -> TRIM
    trig_adv = (sA == "TRIM" and ta._trim_resume_goal == [3.0, 0.0] and ta._hop_start_goal is None)
    tb = _mk_trim(); tb._enter("ADVANCE", 0.0); tb.leg_goal = [3.0, 0.0]
    _, sB, _ = tb.step(0.0, _tplan(-1.95), False)                    # above desired -> no sag
    no_trig = (sB == "ADVANCE")
    # (a2) review-B: refs still None (pre-calibration) -> the trigger is a NO-OP (no fire, no float>None crash).
    tn = ExploreController(cfg, no_takeoff=True); tn._enter("ADVANCE", 0.0); tn.leg_goal = [3.0, 0.0]
    _, sN, _ = tn.step(0.0, _tplan(-1.7), False)
    none_guard = (sN == "ADVANCE" and tn._ceiling_y is None and tn._trim_delta is None)
    # (b) suppressed while calibrating and in a non-whitelisted state (ORIENT).
    tc = _mk_trim(); tc._enter("ADVANCE", 0.0); tc.leg_goal = [3.0, 0.0]; tc._calib_active = True
    _, sC, _ = tc.step(0.0, _tplan(-1.7), False)
    td = _mk_trim(); td._enter("ORIENT", 0.0); td.leg_goal = [3.0, 0.0]; td._player = td._build_turn(0.0)
    _, sD, _ = td.step(0.0, _tplan(-1.7), False)
    suppress_ok = (sC != "TRIM" and sD != "TRIM")
    # (c) ring-gate: fwd blocked + back open -> REPOS reverse (active emitted the tick after the gate).
    tr = _mk_trim(); tr._enter("ADVANCE", 0.0); tr.leg_goal = [3.0, 0.0]
    rp = _tplan(-1.7, fcd=0.3, ring=[[0.0, 0.3], [180.0, 5.0], [90.0, 0.3], [-90.0, 0.3]])
    tr.step(0.0, rp, False); ar, _, _ = tr.step(0.02, rp, False)
    repos_rev = (tr._trim_phase in ("REPOS", "AIM") and float(ar.get("reverse", 0.0)) > 0.0)
    # (d) fwd+back blocked but a SIDE open -> strafe reposition.
    ts = _mk_trim(); ts._enter("ADVANCE", 0.0); ts.leg_goal = [3.0, 0.0]
    ts.step(0.0, _tplan(-1.7, fcd=0.3, ring=[[0.0, 0.3], [180.0, 0.3], [90.0, 5.0], [-90.0, 0.3]]), False)
    strafe_repos = (ts._trim_repos_move is not None and "joy_horizontal" in ts._trim_repos_move)
    # (e) ring blocked all sides -> abort (VISIBLE), exit re-aiming ORIENT at the PRESERVED goal (not TRIM).
    tz = _mk_trim(); tz._enter("ADVANCE", 0.0); tz.leg_goal = [3.0, 0.0]
    _, sZ, evZ = tz.step(0.0, _tplan(-1.7, fcd=0.3, ring=[[0.0, 0.3], [180.0, 0.3], [90.0, 0.3], [-90.0, 0.3]]), False)
    abort_ok = (sZ != "TRIM" and tz.leg_goal == [3.0, 0.0] and "abort" in (evZ or ""))
    # (f) full climb: emits pitch-up -> forward push (WITH trigger_down derived at the choke point — Unity gates
    #     real thrust on the boolean) -> 'c' reset; WAIT holds until cap_ts >= t0+settle (review-C: the gate is
    #     phase-relative, so a stale pre-TRIM frame can't exit early); then re-aims ORIENT at the preserved goal.
    te = _mk_trim(); te._enter("ADVANCE", 0.0); te.leg_goal = [3.0, 0.0]
    saw_pitch = saw_fwd = saw_c = fwd_gated = False
    tt, cap, final = 0.0, 0.0, None
    for _ in range(300):
        aE, sE, _ = te.step(tt, _tplan(-1.7, cap=cap), False)
        if float(aE.get("pitch", 0.0)) != 0.0:
            saw_pitch = True
        if float(aE.get("trigger", 0.0) or 0.0) > 0.0:
            saw_fwd = True
            # operator ask: the TRIM climb push must engage triggerDown (derived centrally in _full_vector).
            fwd_gated = _full_vector(aE, 0, tt, sE)["trigger_down"] is True
        if aE.get("btnCdown"):
            saw_c = True
        if sE != "TRIM":
            final = sE; break
        tt += 0.02; cap += 0.02
    climb_ok = (saw_pitch and saw_fwd and fwd_gated and saw_c and final == "ORIENT" and te.leg_goal == [3.0, 0.0])
    tw = _mk_trim(); tw._enter("TRIM", 0.0); tw._trim_phase = "WAIT"; tw._trim_cmd_t0 = 0.0
    tw._trim_resume_goal = [3.0, 0.0]; tw._trimming = True
    _, sW, _ = tw.step(1.0, _tplan(-1.7, cap=None), False)           # cap_ts None -> not ready -> hold
    wait_hold = (sW == "TRIM")
    trim_ok = (trig_adv and no_trig and none_guard and suppress_ok and repos_rev and strafe_repos and abort_ok
               and climb_ok and wait_hold)
    ok = ok and trim_ok
    print(f"[self-test] {'PASS' if trim_ok else 'FAIL'}  explore HEIGHT-TRIM (sag->TRIM+goal-snapshot+hop-eval-clear="
          f"{trig_adv}, no-sag={no_trig}, None-refs-no-op={none_guard}, calib/state-suppress={suppress_ok}, "
          f"reverse-repos={repos_rev}, strafe-repos={strafe_repos}, ring-blocked-abort={abort_ok}, "
          f"climb pitch/fwd+triggerDown/c+re-aim={climb_ok}, cap-None-holds={wait_hold})")

    # ---- (session 21) PERIODIC HEIGHT RE-CALIBRATION on goal change (cooldown-gated) ----
    cfg_cal = copy.deepcopy(cfg)
    cfg_cal["autonomy"]["explore"]["calibrate_on_goal_change"] = True   # re-enable (harness-wide default False)
    def _mk_cal(last_t, prev):
        c = ExploreController(cfg_cal, no_takeoff=True)
        c._last_calib_t = last_t
        c._leg_goal_prev = prev
        c._enter("REPLAN", 100.0)
        return c
    def _cplan(g, be=0.0):
        return {"plan_valid": True, "done": False, "goal": list(g), "pos": [0.0, 0.0], "bearing_err": be,
                "forward_clearance_dist": 9.0, "pos_y": 0.0, "frame_id": 1, "cap_ts": 100.0, "slam_ms": 200.0,
                "clearance_ring": [[0.0, None]]}
    # (a) genuine goal change (>1u) + cooldown elapsed -> CALIBRATING_HEIGHT; the pulse is hop-outcome-ONLY
    #     (pick_goal None — the pick registers post-calib) and still judges the finished hop.
    ca = _mk_cal(last_t=0.0, prev=[0.0, 0.0])                        # 100s since tap > 60s cooldown
    ca._hop_start_goal = [0.0, 0.0]; ca._hop_start_dist = 5.0        # a finished hop to judge (no progress)
    _, sCa, _ = ca.step(100.0, _cplan([5.0, 0.0]), False)
    pu = ca.take_pick_pulse()
    recal_fires = (sCa == "CALIBRATING_HEIGHT" and ca._recalibrating
                   and pu is not None and pu["pick_goal"] is None and pu["prev_goal"] == [0.0, 0.0])
    # (b) SAME goal region (<1u moved) -> no re-tap -> normal ORIENT; (c) within cooldown -> no re-tap.
    cb = _mk_cal(last_t=0.0, prev=[4.8, 0.0])
    _, sCb, _ = cb.step(100.0, _cplan([5.0, 0.0]), False)
    cc = _mk_cal(last_t=90.0, prev=[0.0, 0.0])                       # only 10s since tap < 60s cooldown
    _, sCc, _ = cc.step(100.0, _cplan([5.0, 0.0]), False)
    recal_gates = (sCb == "ORIENT" and sCc == "ORIENT")
    # (d) review-A: NEVER calibrated (_last_calib_t None — --no-takeoff / failed prelude) -> ALLOWED, not
    #     locked out forever.
    cd2 = _mk_cal(last_t=None, prev=[0.0, 0.0])
    _, sCd, _ = cd2.step(100.0, _cplan([5.0, 0.0]), False)
    recal_none_ok = (sCd == "CALIBRATING_HEIGHT")
    # (e) review-D: the post-calib REPLAN resumes the SAME goal with an unchanged heading -> theta≈0 -> the
    #     ORIENT player is the 'c'-only attitude reset (no yaw thrash). Simulate the resume directly.
    ce = _mk_cal(last_t=100.0, prev=[5.0, 0.0])                      # just tapped; same goal -> normal branch
    _, sCe, _ = ce.step(101.0, _cplan([5.0, 0.0], be=0.0), False)
    aCe, _, _ = ce.step(101.02, _cplan([5.0, 0.0], be=0.0), False)   # first ORIENT tick emits the player
    resume_smooth = (sCe == "ORIENT" and ce._leg_theta == 0
                     and float(aCe.get("yaw", 0.0) or 0.0) == 0.0)   # 'c'-only: no yaw command
    recal_ok = recal_fires and recal_gates and recal_none_ok and resume_smooth
    ok = ok and recal_ok
    print(f"[self-test] {'PASS' if recal_ok else 'FAIL'}  PERIODIC-RECALIB (goal-change+cooldown->CALIBRATING_HEIGHT"
          f"+hop-outcome-only pulse={recal_fires}, same-goal/cooldown gates={recal_gates}, "
          f"never-calibrated allowed={recal_none_ok}, post-calib resume theta~0 'c'-only={resume_smooth})")

    # ---- (session 22) BIDIRECTIONAL TRIM + SLAM-COMFORT GATE + fixed height reference ----
    # (a) TRIM DOWN: glued near the ceiling (pos_y < desired - 0.2*delta) -> TRIM with a POSITIVE pitch (aim
    #     DOWN, since trim_pitch_up=-1) through AIM/FWD; the preserved goal is re-aimed on exit. An IN-BAND
    #     pos_y fires NEITHER direction.
    tdn = _mk_trim(); tdn._enter("ADVANCE", 0.0); tdn.leg_goal = [3.0, 0.0]
    _, sDn, evDn = tdn.step(0.0, _tplan(-2.05), False)     # high thr = -1.9 - 0.2*0.4 = -1.98; -2.05 < -1.98 -> DOWN
    down_fired = (sDn == "TRIM" and tdn._trim_dir == "DOWN" and "(DOWN)" in (evDn or ""))
    saw_down_pitch, tt, cap, final_dn = False, 0.02, 0.02, None
    for _ in range(300):
        aD, sD2, _ = tdn.step(tt, _tplan(-2.05, cap=cap), False)
        if float(aD.get("pitch", 0.0)) > 0.0:              # +1.0 = aim DOWN
            saw_down_pitch = True
        if sD2 != "TRIM":
            final_dn = sD2; break
        tt += 0.02; cap += 0.02
    down_ok = (down_fired and saw_down_pitch and final_dn == "ORIENT" and tdn.leg_goal == [3.0, 0.0])
    tin = _mk_trim(); tin._enter("ADVANCE", 0.0); tin.leg_goal = [3.0, 0.0]
    _, sIn, _ = tin.step(0.0, _tplan(-1.9), False)          # exactly desired -> inside the band
    band_ok = (sIn == "ADVANCE")
    # (b) SLAM-comfort gate in CALIB_LOST_HOLD: alive-but-marginal (full window avg 800ms >= 666) -> the redo
    #     HOLDS (logged); staying gated past calib_gate_max_s counts ONE failed attempt (still holding, no redo
    #     into uncomfortable SLAM); the average dropping under the bar RELEASES the redo.
    def _lplan(fid, ms):
        return {"plan_valid": True, "done": False, "goal": None, "pos": [0.0, 0.0], "pos_y": -1.9,
                "slam_ms": ms, "frame_id": fid, "cap_ts": 0.0}
    cgt = ExploreController(cfg, no_takeoff=True)
    cgt._explore_started = True; cgt._calib_active = True; cgt.calib_gate_max_s = 0.5
    cgt._slam_ms_win.extend([800.0] * cgt.calib_slam_avg_window)     # FULL window, uncomfortable average
    cgt._enter("CALIB_LOST_HOLD", 0.0)
    t, fid, gated_ev, timeout_ev = 0.0, 100, None, None
    for _ in range(40):                                    # marginal 800ms frames: gate holds, then times out
        _a, sG, evG = cgt.step(t, _lplan(fid, 800.0), False, status="OK")
        if evG and "NOT comfortable" in evG:
            gated_ev = evG
        if evG and "comfort gate timeout" in evG:
            timeout_ev = evG; break
        t += 0.05; fid += 1
    gate_holds = (gated_ev is not None and timeout_ev is not None
                  and cgt.state == "CALIB_LOST_HOLD" and cgt._calib_fail_streak == 1)
    released = False
    for _ in range(20):                                    # fast 300ms frames pull the average under the bar
        _a, sG, _evG = cgt.step(t, _lplan(fid, 300.0), False, status="OK")
        if sG == "CALIBRATING_HEIGHT":
            released = True; break
        t += 0.05; fid += 1
    gate_ok = gate_holds and released
    # (c) shipped default: the periodic re-tap is OFF -> a >1u goal change past the cooldown ORIENTs normally.
    coff = ExploreController(cfg, no_takeoff=True)         # harness cfg has calibrate_on_goal_change=False
    coff._last_calib_t = 0.0; coff._leg_goal_prev = [0.0, 0.0]; coff._enter("REPLAN", 100.0)
    _, sOff, _ = coff.step(100.0, _cplan([5.0, 0.0]), False)
    default_off = (coff.calibrate_on_goal_change is False and sOff == "ORIENT")
    # (d) CALIB_VERIFY PASS latches target_altitude_y = the settled desired height + captures the Y-DRIFT
    #     baseline on the FIRST pass; a LATER pass logs the ceiling movement (the rare-tap drift audit).
    def _vpass(ceiling, first):
        c = ExploreController(cfg, no_takeoff=True)
        c._ceiling_y, c._first_ceiling_y = ceiling, first
        c._calib_active = True; c._descend_issue_t = 0.0
        c._enter("CALIB_VERIFY", 0.0)
        _a, sV, evV = c.step(0.2, {"plan_valid": True, "done": False, "goal": None, "pos": [0.0, 0.0],
                                   "pos_y": -1.9, "slam_ms": 200.0, "frame_id": 1,
                                   "cap_ts": c.calib_settle_gate_s + 0.1}, False, status="OK")
        return c, (evV or "")
    cv1, ev1 = _vpass(-2.3, None)
    latch_ok = (cv1.target_altitude_y == -1.9 and cv1._desired_y == -1.9
                and cv1._first_ceiling_y == -2.3 and "Y-DRIFT" not in ev1)
    cv2, ev2 = _vpass(-2.25, -2.3)
    ydrift_ok = ("Y-DRIFT check" in ev2 and "+0.050" in ev2)
    # (e) reference-disagreement warning: the rolling median wandering > delta from desired_y raises ONE loud
    #     notice (take_notice pops it once; display-only, no behavior change).
    cw = ExploreController(cfg, no_takeoff=True)
    cw._height_calibrated = True; cw._desired_y = -1.9; cw._trim_delta = 0.4; cw._ceiling_y = -2.3
    for i in range(12):
        cw.step(i * 0.05, {"plan_valid": True, "done": False, "goal": None, "pos": [0.0, 0.0],
                           "pos_y": -1.3, "slam_ms": 200.0, "frame_id": 500 + i, "cap_ts": i * 0.05}, False)
    n1 = cw.take_notice()
    warn_ok = (n1 is not None and "DISAGREEMENT" in n1 and cw.take_notice() is None)
    s22_ok = down_ok and band_ok and gate_ok and default_off and latch_ok and ydrift_ok and warn_ok
    ok = ok and s22_ok
    print(f"[self-test] {'PASS' if s22_ok else 'FAIL'}  SESSION-22 (TRIM DOWN fires+pitch+re-aim={down_ok}, "
          f"in-band no-fire={band_ok}, comfort gate hold/timeout/release={gate_ok}, re-tap default OFF="
          f"{default_off}, PASS latches target+drift baseline={latch_ok}, Y-DRIFT audit line={ydrift_ok}, "
          f"median-disagreement notice={warn_ok})")

    # Negative bearing error -> open-loop turn yaw NEGATIVE (turn left).
    c2 = ExploreController(cfg, no_takeoff=True)
    _, a2, s2, _ = _drive(c2, {"done": False, "goal": [-1.0, 0.0], "pos": [0.0, 0.0], "bearing_err": -90.0}, False, 0.3, 0.0)
    yaw_neg = (s2 == "ORIENT" and a2.get("yaw", 0.0) < 0)
    # Quantization: nearest whole turn_step_deg (now 30) aim change.
    q = c2._quantize_turn
    quant_ok = (q(70) == 60 and q(50) == 60 and q(10) == 0 and q(-70) == -60 and q(None) == 0)
    # theta≈0 (small err) -> no turn, just the 'c' reset -> ADVANCE; then goal reached with NO wall -> SETTLE.
    c4 = ExploreController(cfg, no_takeoff=True)
    t4, _, s4a, st4a = _drive(c4, {"done": False, "goal": [1.0, 0.0], "pos": [0.0, 0.0], "bearing_err": 5.0}, False, 0.6, 0.0)
    _, _, _, st4 = _drive(c4, {"done": False, "goal": [1.0, 0.0], "pos": [0.9, 0.0], "bearing_err": 5.0}, False, 0.2, t4)
    reached_ok = ("ADVANCE" in st4a) and ("SETTLE" in st4)
    edges_ok = yaw_neg and quant_ok and reached_ok
    ok = ok and edges_ok
    print(f"[self-test] {'PASS' if edges_ok else 'FAIL'}  explore edges (turn- left, quantize 70->60/50->60/"
          f"10->0, theta~0->reset->ADVANCE, goal-reached settle)")

    # ---- (session 20b) HOPS + per-hop PROGRESS pulse (strike/reset) + far-corner exemption + far-corner bump ----
    # (1a) hop cadence RE-PLANS: ADVANCE hops toward leg_goal; at hop_duration_s -> SETTLE routed to REPLAN (NOT a
    #      resume of the old goal). Capture _settle_to at the hop->settle transition.
    chop = ExploreController(cfg, no_takeoff=True)
    chop.hop_duration_s = 0.1; chop.settle_fresh_frames = 2
    chop.leg_goal = [15.0, 0.0]; chop._enter("ADVANCE", 0.0)
    settle_route, t, x, fr, prev = None, 0.0, 0.0, 5000, "ADVANCE"
    for _ in range(12):
        x = round(x + 0.25, 3)                          # steady clear progress toward a FAR goal (never reached here)
        _a, s, _ev = chop.step(t, {"plan_valid": True, "done": False, "goal": [15.0, 0.0], "pos": [x, 0.0],
                                   "bearing_err": 0.0, "forward_clearance_dist": 15.0, "pos_y": 0.0,
                                   "frame_id": fr, "cap_ts": t, "slam_ms": 200.0}, False)
        if s == "SETTLE" and prev == "ADVANCE":
            settle_route = chop._settle_to             # captured at the hop->settle transition (before SETTLE runs)
        prev = s; t += 0.05; fr += 1
    hop_route_ok = (settle_route == "REPLAN")
    # (1b) REPLAN ADOPTS a re-picked, off-axis goal and re-orients WITH the parallax scout (not the old goal).
    crp = ExploreController(cfg, no_takeoff=True); crp.hop_duration_s = 0.1
    crp.leg_goal = [15.0, 0.0]; crp._enter("REPLAN", 0.0)
    crp.step(0.0, {"plan_valid": True, "done": False, "goal": [0.0, 15.0], "pos": [1.0, 0.0],
                   "bearing_err": 90.0, "forward_clearance_dist": 15.0, "pos_y": 0.0,
                   "clearance_ring": [[0.0, None]], "frame_id": 1, "cap_ts": 0.0, "slam_ms": 200.0}, False)
    replan_adopt_ok = (crp.leg_goal == [0.0, 15.0] and crp.state == "ORIENT"
                       and crp._after_orient == "PARALLAX_PUSH")
    # (1c) PER-HOP progress pulse: a REPLAN judges the finished hop (from _hop_start_dist) and emits the combined
    #      pick+outcome pulse. A hop that CLOSED >= hop_progress_eps -> prev_progressed True; one that didn't ->
    #      False (a STRIKE). A FAR corner (old leg corner + still > corner_no_blacklist_dist) -> not strike-eligible.
    def _hop_pulse(start_dist, end_pos, is_corner, goal=[9.0, 0.0]):
        c = ExploreController(cfg, no_takeoff=True); c.hop_duration_s = 0.1
        c.leg_goal = list(goal); c._hop_start_goal = list(goal); c._hop_start_dist = start_dist
        c._leg_is_corner = is_corner; c._enter("REPLAN", 0.0)
        c.step(0.0, {"plan_valid": True, "done": False, "goal": list(goal), "pos": list(end_pos),
                     "bearing_err": 0.0, "forward_clearance_dist": 9.0, "pos_y": 0.0,
                     "frame_id": 1, "cap_ts": 0.0, "slam_ms": 200.0}, False)
        return c.take_pick_pulse()
    pu_prog = _hop_pulse(5.0, [8.0, 0.0], False)                    # 5.0 -> 1.0 closed 4.0 -> progress
    pu_stall = _hop_pulse(5.0, [4.05, 0.0], False)                  # 5.0 -> 4.95 closed 0.05 -> STALL
    pu_corner = _hop_pulse(8.0, [2.0, 0.0], True, goal=[10.0, 0.0])  # no progress but 8u FAR corner -> exempt
    hop_pulse_ok = (pu_prog and pu_prog["prev_progressed"] is True and pu_prog["prev_strike_eligible"] is True
                    and pu_prog["prev_goal"] == [9.0, 0.0] and pu_prog["pick_goal"] == [9.0, 0.0]
                    and pu_stall and pu_stall["prev_progressed"] is False and pu_stall["prev_strike_eligible"] is True
                    and pu_corner and pu_corner["prev_progressed"] is False
                    and pu_corner["prev_strike_eligible"] is False)
    # (2) a plan-loss / SLAM-choke hold ABANDONS the pending per-hop eval (an interrupted hop is not a strike).
    cabort = ExploreController(cfg, no_takeoff=True)
    cabort._hop_start_goal = [9.0, 0.0]; cabort._hop_start_dist = 5.0
    cabort._enter("HOLD_LOST", 0.0)
    abort_ok = cabort._hop_start_goal is None
    # (3) FAR-CORNER bump guard: a corner goal >corner_no_blacklist_dist away is NOT bumped; a near one IS.
    def _corner_bump(dist_away, span_half=None):
        c = ExploreController(cfg, no_takeoff=True)
        c.leg_goal = [10.0, 0.0]; c._leg_is_corner = True; c._bump_armed = True
        pl = {"pos": [10.0 - dist_away, 0.0]}
        if span_half is not None:
            pl["corner_span_half"] = span_half
        c._register_bump(pl, "flow WALL contact")
        return c._bump_pulse
    far_corner_ok = (_corner_bump(3.0) is None                       # far corner -> suppressed (no pulse)
                     and _corner_bump(0.5) == [10.0, 0.0])           # near corner -> bumps normally
    # (3b) session 24: a live corner_span_half OVERRIDES the static config default when present.
    span_override_ok = (_corner_bump(3.0, span_half=5.0) == [10.0, 0.0]   # room is big -> 3u no longer "far"
                         and _corner_bump(0.5, span_half=0.3) is None)    # room is tiny -> 0.5u now IS "far"
    hops_ok = hop_route_ok and replan_adopt_ok and hop_pulse_ok and abort_ok and far_corner_ok and span_override_ok
    ok = ok and hops_ok
    print(f"[self-test] {'PASS' if hops_ok else 'FAIL'}  HOPS+PER-HOP-STRIKE "
          f"(hop->REPLAN={hop_route_ok}; adopts new goal+parallax={replan_adopt_ok}; "
          f"progress/stall/far-corner pulse={hop_pulse_ok}; hold abandons eval={abort_ok}; "
          f"far-corner bump={far_corner_ok}; live corner_span_half overrides default={span_override_ok})")

    # ---- PICK DEDUP (operator diagnosis, session 24; corrected 20260720): a REPLAN re-committing the SAME
    #      goal as the last registered pick, with NO hop judged since (a multi-step ORIENT -> PARALLAX_PUSH ->
    #      SETTLE -> REPLAN sub-step still on the SAME uncommitted leg -- _hop_start_goal never got set) must
    #      NOT register a fresh pick. But a REPLAN that DID judge a genuinely completed hop (_hop_start_goal
    #      was set -- a real ADVANCE ran) always gets a fresh pick, even landing close to the last commit --
    #      that repeated-completed-hop case is exactly the circling behaviour register_goal_pick's loop guard
    #      exists to catch (the 20260720 stuck-flight bug: a frontier "reached" 40+ times from ~the same spot
    #      never accrued a strike -- reaching is unconditional progress -- so only this picks-based loop guard
    #      could ever retire it, and it was starved because every completed hop was wrongly treated as a
    #      same-leg sub-step). A genuinely different goal still gets a full pick either way.
    cdup = ExploreController(cfg, no_takeoff=True); cdup.hop_duration_s = 0.1
    goalA, goalB = [9.0, 0.0], [9.0, 5.0]
    cdup.leg_goal = list(goalA); cdup._hop_start_goal = list(goalA); cdup._hop_start_dist = 5.0
    cdup._enter("REPLAN", 0.0)
    cdup.step(0.0, {"plan_valid": True, "done": False, "goal": list(goalA), "pos": [1.0, 0.0],
                    "bearing_err": 0.0, "forward_clearance_dist": 9.0, "pos_y": 0.0,
                    "frame_id": 1, "cap_ts": 0.0, "slam_ms": 200.0}, False)
    pu_first = cdup.take_pick_pulse()
    first_pick_ok = (pu_first is not None and pu_first["pick_goal"] == goalA)
    # re-commit the SAME goal with NO hop judged (a same-leg turn/scout sub-step, _hop_start_goal left unset
    # since the prior REPLAN cleared it) -> no fresh pick, hop-outcome is unjudgeable (prev_goal is None)
    cdup._enter("REPLAN", 1.0)
    cdup.step(1.0, {"plan_valid": True, "done": False, "goal": list(goalA), "pos": [1.0, 0.0],
                    "bearing_err": 0.0, "forward_clearance_dist": 9.0, "pos_y": 0.0,
                    "frame_id": 2, "cap_ts": 1.0, "slam_ms": 200.0}, False)
    pu_substep = cdup.take_pick_pulse()
    substep_suppressed_ok = (pu_substep is not None and pu_substep["pick_goal"] is None
                              and pu_substep["pick_pos"] is None and pu_substep["prev_goal"] is None
                              and pu_substep["prev_progressed"] is None)
    # re-commit the SAME goal AFTER a genuinely judged hop (_hop_start_goal set -> a real ADVANCE ran and
    # reached it) -> a FRESH pick registers despite landing close to the last commit (the fixed bug)
    cdup._hop_start_goal = list(goalA); cdup._hop_start_dist = 8.0
    cdup._enter("REPLAN", 2.0)
    cdup.step(2.0, {"plan_valid": True, "done": False, "goal": list(goalA), "pos": [3.0, 0.0],
                    "bearing_err": 0.0, "forward_clearance_dist": 9.0, "pos_y": 0.0,
                    "frame_id": 3, "cap_ts": 2.0, "slam_ms": 200.0}, False)
    pu_repick = cdup.take_pick_pulse()
    judged_repick_ok = (pu_repick is not None and pu_repick["pick_goal"] == goalA
                         and pu_repick["pick_pos"] is not None
                         and pu_repick["prev_goal"] == goalA and pu_repick["prev_progressed"] is True)
    # now commit a GENUINELY different goal (> calib_goal_change_dist away) -> full pick again
    cdup._hop_start_goal = list(goalA); cdup._hop_start_dist = 3.0
    cdup._enter("REPLAN", 3.0)
    cdup.step(3.0, {"plan_valid": True, "done": False, "goal": list(goalB), "pos": [2.0, 0.0],
                    "bearing_err": 0.0, "forward_clearance_dist": 9.0, "pos_y": 0.0,
                    "frame_id": 4, "cap_ts": 3.0, "slam_ms": 200.0}, False)
    pu_new = cdup.take_pick_pulse()
    new_goal_pick_ok = (pu_new is not None and pu_new["pick_goal"] == goalB)
    pick_dedup_ok = (first_pick_ok and substep_suppressed_ok and judged_repick_ok and new_goal_pick_ok)
    ok = ok and pick_dedup_ok
    print(f"[self-test] {'PASS' if pick_dedup_ok else 'FAIL'}  PICK DEDUP "
          f"(first commit picks={first_pick_ok}, unjudged same-leg sub-step suppresses pick="
          f"{substep_suppressed_ok}, judged repeated hop re-picks the same goal={judged_repick_ok}, "
          f"genuinely-new goal picks again={new_goal_pick_ok})")

    # ---- PICK DEDUP regression (bug found on 20260719_005402): a goal 0.5-1.0u from the last one is a
    #      genuinely DIFFERENT goals-DB disc (> goal_area_radius, default 0.5) even though it's inside
    #      calib_goal_change_dist (default 1.0) -- it must still register as a fresh pick, not get
    #      swallowed by the same-goal dedup (which used to wrongly compare against the calibration
    #      constant instead of goal_area_radius).
    cdup2 = ExploreController(cfg, no_takeoff=True); cdup2.hop_duration_s = 0.1
    goalC, goalD = [0.0, 0.0], [0.69, 0.0]   # 0.69u apart: > goal_area_radius (0.5), < calib_goal_change_dist (1.0)
    cdup2.leg_goal = list(goalC); cdup2._hop_start_goal = list(goalC); cdup2._hop_start_dist = 5.0
    cdup2._enter("REPLAN", 0.0)
    cdup2.step(0.0, {"plan_valid": True, "done": False, "goal": list(goalC), "pos": [1.0, 0.0],
                     "bearing_err": 0.0, "forward_clearance_dist": 9.0, "pos_y": 0.0,
                     "frame_id": 1, "cap_ts": 0.0, "slam_ms": 200.0}, False)
    cdup2.take_pick_pulse()
    cdup2._hop_start_goal = list(goalC); cdup2._hop_start_dist = 3.0
    cdup2._enter("REPLAN", 1.0)
    cdup2.step(1.0, {"plan_valid": True, "done": False, "goal": list(goalD), "pos": [2.0, 0.0],
                     "bearing_err": 0.0, "forward_clearance_dist": 9.0, "pos_y": 0.0,
                     "frame_id": 2, "cap_ts": 1.0, "slam_ms": 200.0}, False)
    pu_close = cdup2.take_pick_pulse()
    close_new_pick_ok = (pu_close is not None and pu_close["pick_goal"] == goalD and pu_close["pick_pos"] is not None)
    print(f"[self-test] {'PASS' if close_new_pick_ok else 'FAIL'}  PICK DEDUP regression "
          f"(goal 0.69u away -> genuinely new pick registered, not swallowed by calib_goal_change_dist={close_new_pick_ok})")
    ok = ok and close_new_pick_ok

    # ---- session 24: persistent corner give-up escalation + STUCK-vs-RETURN_TO_ORIGIN ending ----
    def _far_bump(c, corner, dist_away=3.0):
        c.leg_goal = list(corner); c._leg_is_corner = True; c._bump_armed = True
        c._register_bump({"pos": [corner[0] - dist_away, corner[1]]}, "flow WALL contact")
        return c.take_corner_giveup_pulse()
    # (h1) the give-up count PERSISTS per corner regardless of oscillating between two far corners -- a
    #      single reset-on-switch slot (mirroring note_wall_hit's) would let oscillation defeat the cap.
    cgu = ExploreController(cfg, no_takeoff=True); cgu.corner_giveup_limit = 5
    cornerA, cornerB = [10.0, 0.0], [-10.0, 0.0]
    for _ in range(2):
        _far_bump(cgu, cornerA)
    for _ in range(2):
        _far_bump(cgu, cornerB)
    countA = next(e["count"] for e in cgu._corner_giveup_counts if e["goal"] == cornerA)
    countB = next(e["count"] for e in cgu._corner_giveup_counts if e["goal"] == cornerB)
    oscillation_ok = (countA == 2 and countB == 2)   # neither switch reset the other's count
    # (h2) below corner_giveup_limit: plain missed-bump, no pulse. AT the limit: a giveup pulse fires + the
    #      missed-bump message says EXPIRED (force-retiring).
    clim = ExploreController(cfg, no_takeoff=True); clim.corner_giveup_limit = 3
    cornerC = [5.0, 5.0]
    pulses, msgs = [], []
    for _ in range(3):
        pulses.append(_far_bump(clim, cornerC))
        msgs.append(clim.take_missed_bump())
    below_limit_quiet = (pulses[0] is None and pulses[1] is None
                         and all(m is not None and "EXPIRED" not in m for m in msgs[:2]))
    at_limit_pulse = (pulses[2] == cornerC and msgs[2] is not None and "EXPIRED" in msgs[2])
    giveup_escalation_ok = oscillation_ok and below_limit_quiet and at_limit_pulse
    # (h3) REPLAN's done branch: corner_giveup_stuck=True -> STUCK (not RETURN_TO_ORIGIN); that STUCK must NOT
    #      auto-resume even though plan.get("done") stays permanently True (unlike the ordinary SLAM-fallback
    #      use of STUCK, which still auto-resumes once a goal/done returns).
    cstuck = ExploreController(cfg, no_takeoff=True); cstuck.settle_gate_s = 0.01
    cstuck._enter("REPLAN", 0.0)
    _a, s_stuck, _ = cstuck.step(0.0, {"done": True, "corner_giveup_stuck": True, "goal": None,
                                       "pos": [0.0, 0.0]}, False)
    stuck_entered_ok = (s_stuck == "STUCK" and cstuck._corner_giveup_stuck is True)
    # drive PLENTY of ticks with a perfectly healthy SLAM stream -- even so, this STUCK must never resume
    # (unlike the generic recovery convergence, which would normally seize on exactly this healthy stream).
    t, still_stuck = 0.0, True
    for i in range(40):
        t += 0.05
        _a, s_chk, _ = cstuck.step(t, {"done": True, "corner_giveup_stuck": True, "goal": None,
                                       "pos": [0.0, 0.0], "plan_valid": True, "frame_id": i,
                                       "cap_ts": t, "slam_ms": 200.0}, False)
        if s_chk != "STUCK":
            still_stuck = False
            break
    stuck_stays_ok = still_stuck
    cend = ExploreController(cfg, no_takeoff=True); cend._enter("REPLAN", 0.0)
    _a, s_end, _ = cend.step(0.0, {"done": True, "goal": None, "pos": [0.0, 0.0]}, False)
    graceful_end_ok = (s_end == "RETURN_TO_ORIGIN" and cend._corner_giveup_stuck is False)
    # the ORDINARY (non-giveup) use of STUCK (e.g. FALLBACK exhaustion) still auto-resumes once SLAM/planning
    # are healthy again -- via the generic recovery convergence (STUCK -> SLAM_HOLD -> SETTLE -> REPLAN).
    cfb = ExploreController(cfg, no_takeoff=True); cfb.settle_gate_s = 0.01
    cfb._enter("STUCK", 0.0)
    t, s_fb = 0.0, "STUCK"
    for i in range(40):
        t += 0.05
        _a, s_fb, _ = cfb.step(t, {"goal": [1.0, 0.0], "pos": [0.0, 0.0], "plan_valid": True,
                                   "bearing_err": 0.0, "frame_id": i, "cap_ts": t, "slam_ms": 200.0}, False)
        if s_fb == "REPLAN":
            break
    fallback_stuck_resumes_ok = (s_fb == "REPLAN")
    stuck_ending_ok = stuck_entered_ok and stuck_stays_ok and graceful_end_ok and fallback_stuck_resumes_ok
    corner_giveup_ok = giveup_escalation_ok and stuck_ending_ok
    ok = ok and corner_giveup_ok
    print(f"[self-test] {'PASS' if corner_giveup_ok else 'FAIL'}  CORNER GIVE-UP escalation "
          f"(persists across oscillation={oscillation_ok}, below-limit quiet+at-limit pulse={below_limit_quiet and at_limit_pulse}, "
          f"done+giveup->STUCK no-resume={stuck_entered_ok and stuck_stays_ok}, "
          f"ordinary done->RETURN_TO_ORIGIN={graceful_end_ok}, ordinary STUCK still auto-resumes={fallback_stuck_resumes_ok})")

    # ---- Map mode: PRELUDE arm + takeoff + TWO-PHASE ascent + descend + baseline nudge (airborne + to height) ----
    ascend = int(cfg["autonomy"]["ascend_cmd"])
    plan_goal = {"done": False, "goal": [1.0, 0.0], "pos": [0.0, 0.0], "bearing_err": 90.0}
    cp = ExploreController(cfg)                      # default: full prelude
    cp.rest_between_s = 0.2                           # speed the settles up for the test
    cp.ascend_micro_pulse_s, cp.ascend_rest_s = 0.1, 0.1
    cp.ascend_stall_cycles, cp.ascend_latch_hold_s = 2, 0.3
    cp.baseline_nudge_max_s = 0.3                     # end the baseline nudge by its time cap (pos held at 0)
    cp.calib_settle_gate_s = 0.1                      # let the prelude CALIB_VERIFY settle quickly (empty baseline -> PASS)
    porder, saw_arm, saw_to_up, saw_asc_up, saw_desc = [], False, False, False, False
    asc_seen = 0
    t, fid = 0.0, 0
    for _ in range(int(18.0 / 0.05)):
        cur = cp.state
        # Feed a valid pose that RISES (pos_y decreases) then flattens so Phase 1 hands to Phase 2; fire the
        # flow CEILING only once we're in the Phase-2 LATCH hold (flush at the ceiling).
        posy = -0.05 * min(asc_seen, 10) if cur == "ASCEND" else 0.0
        fire_ceiling = (cur == "ASCEND" and cp._ascend_phase == "LATCH")
        plan = dict(plan_goal, plan_valid=True, pos_y=posy, slam_ms=200.0, frame_id=fid, cap_ts=t,
                    forward_clearance_dist=9.0, clearance_ring=[[0.0, 5.0], [180.0, 0.3]])
        active, _state, _ev = cp.step(t, plan, False, ceiling_contact=fire_ceiling)
        if not porder or porder[-1] != cur:
            porder.append(cur)
        if cur == "ARM" and active.get("btnARMdown") is True:
            saw_arm = True
        if cur == "TAKEOFF" and active.get("joy_vertical") == ascend:
            saw_to_up = True
        if cur == "ASCEND":
            asc_seen += 1
            if active.get("joy_vertical") == ascend:
                saw_asc_up = True
        if cur == "DESCEND" and active.get("joy_vertical") == -ascend:
            saw_desc = True
        t += 0.05; fid += 1
    prelude_ok = (saw_arm and saw_to_up and saw_asc_up and saw_desc and cp.airborne_done and cp._baseline_seeded
                  and _is_subsequence(["ARM", "TAKEOFF", "ASCEND", "DESCEND", "BASELINE_NUDGE", "REPLAN", "ORIENT"],
                                      porder))
    # reset_leg AFTER airborne must NOT re-run the prelude (-> REPLAN); a grounded controller restarts at ARM.
    cp.reset_leg()
    no_rearm = (cp.state == "REPLAN")
    cg = ExploreController(cfg)
    cg.step(0.0, plan_goal, False)                  # enters ARM (not yet airborne)
    cg.reset_leg()
    rearm_if_grounded = (cg.state == "ARM")
    prelude_ok = prelude_ok and no_rearm and rearm_if_grounded
    ok = ok and prelude_ok
    print(f"[self-test] {'PASS' if prelude_ok else 'FAIL'}  explore PRELUDE arm+takeoff+two-phase-ascent+descend+baseline "
          f"(ascend joy={ascend}, descend joy={-ascend}, seeded={cp._baseline_seeded}, no re-run once airborne)  visited {porder}")

    # ---- Map mode: per-goal HEIGHT RE-CALIBRATION (item 1 + session-11 STATE-GATED CALIB_VERIFY fix) ----
    def _run_calib(cc, goal, t0, secs=40.0, baseline=None, verify_posy=-1.0, feed_cap=True):
        """Drive an airborne controller through the CALIBRATING_HEIGHT re-tap machinery (session-17: the periodic
        per-goal TRIGGER is gone, so we ENTER the state DIRECTLY — mirroring the future wall-hit trigger). Feed a
        rising-then-flat pose during ASCEND (fire the flow CEILING in the Phase-2 LATCH); in CALIB_VERIFY (and
        ASCEND_ESCAPE) feed `verify_posy` as the SETTLED height, plus a cap_ts (unless feed_cap=False) so the
        settlement gate can pass. `baseline` primes the rolling flying-height history.
        Returns (visited_states, saw_up, saw_down)."""
        cc.rest_between_s = 0.1
        cc.settle_gate_s = 0.1    # session 24: independent of rest_between_s -- must be set explicitly too
        cc.ascend_micro_pulse_s, cc.ascend_rest_s = 0.1, 0.1
        cc.ascend_stall_cycles, cc.ascend_latch_hold_s = 2, 0.3
        cc.baseline_nudge_dist, cc.baseline_nudge_max_s = 0.3, 0.3
        cc.calib_retry_translate_dist = 0.3
        cc.calib_settle_gate_s, cc.calib_verify_max_s = 0.1, 1.0
        if baseline is not None:
            cc._mapping_altitude_history = collections.deque(baseline, maxlen=cc.mapping_alt_history_len)
        cc.leg_goal = list(goal["goal"])          # the committed goal a PASS re-aims to (ORIENT)
        cc._recalibrating = True                  # per-goal DESCEND routing (-> REPLAN -> ORIENT) + a clean retry budget
        cc._calib_retries = 0
        cc._ascend_phase = None
        cc._enter("CALIBRATING_HEIGHT", t0)       # enter the re-tap DIRECTLY (no periodic trigger anymore)
        order, asc_seen, saw_up, saw_down = [], 0, False, False
        t, fid = t0, 0
        for _ in range(int(secs / 0.05)):
            cur = cc.state
            if cur == "ASCEND":
                posy = -0.05 * min(asc_seen, 10)
                asc_seen += 1
            elif cur in ("CALIB_VERIFY", "ASCEND_ESCAPE"):
                posy = verify_posy                       # the settled (possibly sunk) height under test
            else:
                posy = -1.0
            fire = (cur == "ASCEND" and cc._ascend_phase == "LATCH")
            pl = dict(goal, plan_valid=True, pos_y=posy, slam_ms=200.0, frame_id=fid,
                      forward_clearance_dist=9.0, clearance_ring=[[0.0, 5.0], [180.0, 0.3]])
            if feed_cap:
                pl["cap_ts"] = t                         # camera-capture ts (same monotonic domain as `now`)
            _active, state, _ev = cc.step(t, pl, False, ceiling_contact=fire)
            if not order or order[-1] != cur:
                order.append(cur)
            if cur == "ASCEND" and _active.get("joy_vertical") == ascend:
                saw_up = True
            if cur == "DESCEND" and _active.get("joy_vertical") == -ascend:
                saw_down = True                          # the re-tap MUST push back down off the ceiling
            if state == "ORIENT":
                if not order or order[-1] != "ORIENT":
                    order.append("ORIENT")
                break
            t += 0.05; fid += 1
        return order, saw_up, saw_down

    goal_far = {"done": False, "goal": [5.0, 5.0], "pos": [0.0, 0.0], "bearing_err": 0.0}
    flat_baseline = [-1.0] * 15      # a populated flying-height baseline, median ~-1.0

    # (SESSION-17: the former subcases (1) PASS-at-height and (2) cooldown-gate tested the PERIODIC per-goal
    #  TRIGGER, now deleted. The retained CALIB_VERIFY machinery below is entered DIRECTLY by _run_calib,
    #  mirroring the future wall-hit trigger.)
    # (a) HAPPY PATH: enter the re-tap -> ASCEND (up) -> DESCEND (down: the re-tap MUST push back off the ceiling,
    #     saw_down) -> CALIB_VERIFY settles AT the flying-height median (verify_posy == median) -> PASS -> REPLAN ->
    #     ORIENT. PASS clears the freeze (_calib_active / _recalibrating False).
    ca = ExploreController(cfg, no_takeoff=True)
    oA, upA, downA = _run_calib(ca, goal_far, 100.0, baseline=flat_baseline, verify_posy=-1.0)
    happy_ok = (_is_subsequence(["CALIBRATING_HEIGHT", "ASCEND", "DESCEND", "CALIB_VERIFY", "REPLAN", "ORIENT"], oA)
                and upA and downA and not ca._recalibrating and not ca._calib_active)
    # (b) CALIB_VERIFY settles SIGNIFICANTLY LOWER than the flying-height median (+Y DOWN => larger pos_y) ->
    #     FAIL -> climb (ASCEND_ESCAPE) -> slide (CALIB_TRANSLATE) -> re-calibrate; retries bound the loop.
    c3 = ExploreController(cfg, no_takeoff=True)
    o3, _, _ = _run_calib(c3, goal_far, 100.0, baseline=flat_baseline, verify_posy=-0.2)  # -0.2 >> -1.0 => sunk
    fail_retry = _is_subsequence(["CALIB_VERIFY", "ASCEND_ESCAPE", "CALIB_TRANSLATE", "CALIBRATING_HEIGHT"], o3)
    # (c) EMPTY baseline (prelude case) -> cannot judge -> PASS immediately even from a low settle.
    c4 = ExploreController(cfg, no_takeoff=True)
    o4, _, _ = _run_calib(c4, goal_far, 100.0, baseline=[], verify_posy=-0.2)   # sunk, but no baseline to judge
    empty_pass = ("ASCEND_ESCAPE" not in o4 and "ORIENT" in o4 and not c4._calib_active)
    # (d) cap_ts None (dropped frame) -> the settlement gate HOLDS in CALIB_VERIFY (no crash on None >= float);
    #     session-15 Fix-3b: on the verify_max_s cap with NO settled healthy pose it must NOT fly to a goal on a
    #     stale pose -> it counts the attempt as failed and escalates (redo, then CALIB_ESCAPE after N), NEVER
    #     reaching ORIENT (and never a silent PASS). ASCEND_ESCAPE (the sink-retry) is a separate path.
    c5 = ExploreController(cfg, no_takeoff=True)
    o5, _, _ = _run_calib(c5, goal_far, 100.0, baseline=flat_baseline, verify_posy=-1.0, feed_cap=False)
    capnone_ok = ("CALIB_VERIFY" in o5 and "ORIENT" not in o5 and "CALIB_ESCAPE" in o5)
    # (unit, session 18) baseline ingest: measures ONLY after the first calibration (_height_calibrated),
    # NEVER while calibrating (_calib_active), at healthy SLAM, and exactly ONE reading per FRESH frame_id.
    def _alt_step(c, fid):
        c._slam_ms_latest = 100.0   # healthy SLAM (< slow threshold) so ingest is allowed
        c.step(0.0, {"plan_valid": True, "pos_y": -1.2, "frame_id": fid, "goal": None, "done": False}, False)
    c_nocal = ExploreController(cfg, no_takeoff=True)     # not yet calibrated -> no ingest even on a fresh frame
    c_nocal._height_calibrated = False
    c_nocal._mapping_altitude_history.clear()
    _alt_step(c_nocal, 1)
    c_cal = ExploreController(cfg, no_takeoff=True)       # calibrated + healthy + not calib -> ingest one per FRESH frame
    c_cal._height_calibrated = True
    c_cal._mapping_altitude_history.clear()
    _alt_step(c_cal, 1)                                   # fresh frame -> append (1)
    _alt_step(c_cal, 1)                                   # SAME frame_id -> deduped (still 1)
    _alt_step(c_cal, 2)                                   # fresh frame -> append (2)
    c_frozen = ExploreController(cfg, no_takeoff=True)    # frozen during a calibration -> no ingest
    c_frozen._height_calibrated, c_frozen._calib_active = True, True
    c_frozen._mapping_altitude_history.clear()
    _alt_step(c_frozen, 5)
    ingest_gate = (len(c_nocal._mapping_altitude_history) == 0        # not calibrated -> no measurement
                   and len(c_cal._mapping_altitude_history) == 2      # 2 fresh frames -> 2 (repeat deduped)
                   and len(c_frozen._mapping_altitude_history) == 0)  # frozen during calibration
    calib_ok = (happy_ok and fail_retry and empty_pass and capnone_ok and ingest_gate)
    ok = ok and calib_ok
    print(f"[self-test] {'PASS' if calib_ok else 'FAIL'}  explore HEIGHT RE-CALIB state-gated "
          f"(happy re-tap up+down->PASS->orient={happy_ok}, "
          f"low-settle->escape/translate/retry={fail_retry}, empty-baseline->PASS={empty_pass}, "
          f"cap_ts-None-holds={capnone_ok}, baseline-ingest-gated={ingest_gate})  visited {oA}")

    # ---- Calibration INTERRUPTED by a plan loss: CALIB_LOST_HOLD (survive the loss, redo the re-tap) ----
    def _calib_lost_ctrl():
        c = ExploreController(cfg, no_takeoff=True)     # no_takeoff => _explore_started True
        c.calib_lost_recover_frames, c.calib_lost_bump_slow_frames = 6, 6
        c._calib_active, c.state = True, "ASCEND"        # mid re-tap when the loss lands
        return c
    ANY_LOST = "PLAN-LOST"
    # (a) ENTRY: a loss DURING a calibration diverts to CALIB_LOST_HOLD (NOT HOLD_LOST), latches the flag,
    #     releases controls, and resets the pulse streaks so we count FRESH from the loss.
    cl = _calib_lost_ctrl()
    cl._slam_fast_streak = 9                              # a stale pre-loss streak that must NOT leak in
    a_ent, s_ent, _ = cl.step(0.0, {"frame_id": 500, "slam_ms": 200.0}, False, status=ANY_LOST)
    entry_ok = (s_ent == "CALIB_LOST_HOLD" and cl._calib_interrupted and a_ent == {}
                and cl._slam_fast_streak == 0 and cl._slam_slow_streak == 0)
    # (b) CAUSE A (wake SLAM) + immediate-bump: 6 fresh CHOKED frames -> exactly one DOWN bump on the 6th
    #     frame's tick (joy_vertical == +1 = down), then a 7th choked frame yields no second bump.
    t, fid = 0.05, 600
    bump_tick, bumps = None, 0
    for k in range(6):
        a, s, _ = cl.step(t, {"frame_id": fid, "slam_ms": 2000.0}, False, status=ANY_LOST)
        if a.get("joy_vertical") == 1:
            bumps += 1; bump_tick = k
        t += 0.05; fid += 1
    # drain the in-flight descend player, then a further choked frame -> no new bump
    for _ in range(6):
        a, s, _ = cl.step(t, {"frame_id": fid, "slam_ms": 2000.0}, False, status=ANY_LOST)
        if a.get("joy_vertical") == 1 and cl._player is None:
            bumps += 1
        t += 0.05; fid += 1
    causeA_ok = (bumps == 1 and bump_tick == 5 and cl._calib_lost_bumped and cl.state == "CALIB_LOST_HOLD")
    # (c) CAUSE B + TRAP-1: 6 fresh FAST frames but status still lost -> MUST NOT exit to CALIBRATING_HEIGHT;
    #     it bumps once (to unglue the stuck planner) and keeps holding.
    cb = _calib_lost_ctrl()
    cb.step(0.0, {"frame_id": 700, "slam_ms": 200.0}, False, status=ANY_LOST)   # enter the hold
    t, fid, saw_calib, saw_downB = 0.05, 701, False, False
    for _ in range(8):
        a, s, _ = cb.step(t, {"frame_id": fid, "slam_ms": 200.0}, False, status=ANY_LOST)
        saw_calib = saw_calib or (s == "CALIBRATING_HEIGHT")
        saw_downB = saw_downB or (a.get("joy_vertical") == 1)
        t += 0.05; fid += 1
    # `_calib_lost_bumped` is one-shot, so it guarantees "exactly one bump" without counting playout ticks.
    causeB_trap1_ok = (not saw_calib and cb.state == "CALIB_LOST_HOLD" and saw_downB and cb._calib_lost_bumped)
    # (d) RECOVER: 6 fresh FAST frames AND status OK -> redo the calibration (CALIBRATING_HEIGHT), with a
    #     fresh retry budget and the per-goal DESCEND routing (_recalibrating True).
    cr = _calib_lost_ctrl()
    cr._calib_retries = 2                                 # a spent budget that the redo must reset
    cr.step(0.0, {"frame_id": 800, "slam_ms": 200.0}, False, status="PLAN-LOST")  # enter the hold
    t, fid, reached = 0.05, 801, False
    for _ in range(8):
        a, s, _ = cr.step(t, {"frame_id": fid, "slam_ms": 200.0}, False, status="OK")
        if s == "CALIBRATING_HEIGHT":
            reached = True; break
        t += 0.05; fid += 1
    recover_ok = (reached and cr._recalibrating and cr._calib_retries == 0)
    # (e) a FULL redo that reaches CALIB_VERIFY PASS clears the interrupted flag (redo went smoothly).
    cp = ExploreController(cfg, no_takeoff=True)
    cp._calib_interrupted = True                          # pretend a prior loss owed a redo
    _run_calib(cp, goal_far, 100.0, baseline=flat_baseline, verify_posy=-1.0)
    verify_clears_ok = (not cp._calib_interrupted and not cp._calib_active)
    # (f) STALE (not just LOST) during a calibration also diverts to _step_calib_lost (not _step_stale).
    cs2 = _calib_lost_ctrl()
    _a, s_stale, _ = cs2.step(0.0, {"frame_id": 900, "slam_ms": 200.0}, False, status="PLAN-STALE")
    stale_divert_ok = (s_stale == "CALIB_LOST_HOLD" and cs2._calib_interrupted)
    calib_lost_ok = (entry_ok and causeA_ok and causeB_trap1_ok and recover_ok and verify_clears_ok
                     and stale_divert_ok)
    ok = ok and calib_lost_ok
    print(f"[self-test] {'PASS' if calib_lost_ok else 'FAIL'}  CALIB_LOST_HOLD (interrupted re-tap survives loss) "
          f"(entry+reset={entry_ok}, causeA-1bump-immediate={causeA_ok}, causeB+trap1-no-exit={causeB_trap1_ok}, "
          f"recover=redo={recover_ok}, verify-clears-flag={verify_clears_ok}, stale-diverts={stale_divert_ok})")

    # ---- Map mode: SETTLE fresh-frame gate (session 15) — a goal-flying settle waits for N SLAM frames
    #      CAPTURED after the settle began (cap_ts >= entry) AND fast; the vertical routine is exempt. ----
    def _splan(cap, fid, posy=-1.0):
        return {"plan_valid": True, "pos_y": posy, "goal": [1.0, 0.0], "bearing_err": 0.0,
                "frame_id": fid, "cap_ts": cap, "slam_ms": 200.0, "forward_clearance_dist": 5.0}
    # (a) gated (nxt REPLAN): frames CAPTURED BEFORE entry (cap_ts < t0) never count -> HOLD.
    cg1 = ExploreController(cfg, no_takeoff=True); cg1.settle_gate_s = 0.1; cg1.settle_fresh_frames = 6
    cg1._settle_to = None; cg1._enter("SETTLE", 1.0)          # _settle_t0 = 1.0
    held = True
    for i in range(20):
        _a, s_g, _ = cg1.step(1.5 + i * 0.05, _splan(cap=0.0, fid=100 + i), False)   # cap_ts=0.0 < 1.0
        if s_g != "SETTLE":
            held = False; break
    settle_stale_holds = held and cg1.state == "SETTLE"
    # (b) gated: 6 fresh fast frames CAPTURED after entry -> proceed to REPLAN.
    cg2 = ExploreController(cfg, no_takeoff=True); cg2.settle_gate_s = 0.1; cg2.settle_fresh_frames = 6
    cg2._settle_to = None; cg2._enter("SETTLE", 0.0)
    proceeded, t = None, 0.5
    for i in range(20):
        _a, s_g, _ = cg2.step(t, _splan(cap=t, fid=500 + i), False)                  # cap_ts=t >= 0.0, fast
        if s_g != "SETTLE":
            proceeded = s_g; break
        t += 0.05
    settle_fresh_proceeds = (proceeded == "REPLAN")
    # (c) EXEMPT (nxt ASCEND): proceeds on the timer with NO fresh frames.
    ce1 = ExploreController(cfg, no_takeoff=True); ce1.settle_gate_s = 0.1
    ce1._settle_to = "ASCEND"; ce1._enter("SETTLE", 1.0)
    exempt_next = None
    for i in range(10):
        _a, s_g, _ = ce1.step(1.5 + i * 0.05, _splan(cap=0.0, fid=700 + i), False)   # stale frames, still advances
        if s_g != "SETTLE":
            exempt_next = s_g; break
    settle_exempt_proceeds = (exempt_next == "ASCEND")
    settle_ok = settle_stale_holds and settle_fresh_proceeds and settle_exempt_proceeds
    ok = ok and settle_ok
    print(f"[self-test] {'PASS' if settle_ok else 'FAIL'}  SETTLE fresh-frame gate (stale-holds="
          f"{settle_stale_holds}, 6-fresh->REPLAN={settle_fresh_proceeds}, exempt-vertical-timed="
          f"{settle_exempt_proceeds})")

    # ---- Map mode: CALIB_ESCAPE / STUCK guard (session 15) — bound the finish->lose->retry loop. ----
    cesc = ExploreController(cfg, no_takeoff=True)
    cesc.calib_escape_after, cesc.calib_escape_ok_frames, cesc.calib_escape_push_s = 3, 12, 0.2
    cesc._calib_active = True
    cesc._last_ring = [[180.0, 5.0]]                          # backward is pushable
    cesc._calib_fail_escalate(0.0, "t"); redo1 = cesc.state   # fail 1 -> REDO
    cesc._calib_fail_escalate(0.1, "t")                       # fail 2 -> REDO
    cesc._calib_fail_escalate(0.2, "t")                       # fail 3 -> CALIB_ESCAPE
    esc_entered = (redo1 == "CALIBRATING_HEIGHT" and cesc.state == "CALIB_ESCAPE"
                   and cesc._calib_escaped and cesc._calib_fail_streak == 0)
    # escape: ring push (reverse) then HOLD for 12 fast frames + OK -> RETRY CALIBRATING_HEIGHT.
    saw_push, retried, t = False, False, 1.0
    for i in range(80):
        a_e, s_e, _ = cesc.step(t, _splan(cap=t, fid=900 + i, posy=-2.0), False, status="OK")
        if float(a_e.get("reverse", 0.0)) > 0:
            saw_push = True
        if s_e == "CALIBRATING_HEIGHT":
            retried = True; break
        t += 0.05
    escape_retry_ok = saw_push and retried
    # after the escape, calib_escape_after MORE fails -> STUCK (and stop freezing the baseline).
    cesc._calib_active = True
    cesc._calib_fail_escalate(5.0, "t"); cesc._calib_fail_escalate(5.1, "t"); cesc._calib_fail_escalate(5.2, "t")
    stuck_after_escape = (cesc.state == "STUCK" and not cesc._calib_active)
    escape_ok = esc_entered and escape_retry_ok and stuck_after_escape
    ok = ok and escape_ok
    print(f"[self-test] {'PASS' if escape_ok else 'FAIL'}  CALIB_ESCAPE/STUCK (3-fails->escape={esc_entered}, "
          f"push+12hold->retry={escape_retry_ok}, escape+3-fails->STUCK={stuck_after_escape})")

    # ---- Map mode: CONTROL-SPACE SLAM-loss recovery (hold-on-LOST + rewind-on-STALE + parallax fallback) ----
    # (a) invert_history: reverse order, invert each maneuver (forward<->reverse, turn theta->-theta).
    ci = ExploreController(cfg, no_takeoff=True)
    ci.command_history.append({"kind": "forward", "value": 0.2, "duration_s": 3.0})   # flown 1st
    ci.command_history.append({"kind": "turn", "theta": 45.0})                        # flown 2nd
    ci.command_history.append({"kind": "reverse", "value": 0.2, "duration_s": 1.0})   # flown 3rd (newest)
    inv = ci._invert_history()
    invert_ok = (inv[0].get("trigger") == 0.2                      # newest (reverse) inverted first -> forward
                 and any(s.get("yaw", 0.0) < 0 for s in inv)       # +45 turn inverted -> yaw the other way
                 and inv[-1].get("reverse") == 0.2)                # oldest (forward) inverted last -> reverse
    # (b) PLAN-LOST -> HARD HOVER-HOLD, indefinitely (neutral; never moves).
    ch = ExploreController(cfg, no_takeoff=True)
    _, ah, sh, _ = _drive(ch, {"plan_valid": False, "goal": None, "pos": [0.0, 0.0]}, False, 3.0, 0.0, status="PLAN-LOST")
    hold_ok = (sh == "HOLD_LOST" and ah == {})
    # (c) PLAN-STALE (with history) -> RECOVERY_REWIND; then OK -> wait for SLAM to settle -> brake -> resume.
    cw = ExploreController(cfg, no_takeoff=True)
    cw.settle_gate_s = 0.05            # small physical dwell so this test's tick budget stays short
    cw.command_history.append({"kind": "forward", "value": 0.2, "duration_s": 2.0})
    cw.command_history.append({"kind": "turn", "theta": 45.0})
    stale = {"plan_valid": False, "goal": None, "pos": [0.0, 0.0], "clearance_ring": None}
    t, _, _, st_st = _drive(cw, stale, False, 1.0, 0.0, status="PLAN-STALE")
    rewind_ok = ("REWIND" in st_st)
    _, _, so, st_ok = _drive(cw, {"plan_valid": True, "goal": [3.0, 0.0], "pos": [0.0, 0.0], "bearing_err": 0.0,
                                  "slam_ms": 120.0, "frame_id": 1},
                             False, cw.settle_gate_s + 1.0, t, status="OK")
    # recovery-exit now HOLDs for SLAM to settle before braking (strengthen the solve) -> SETTLE -> replan.
    snap_ok = ("SLAM_HOLD" in st_ok) and ("SETTLE" in st_ok) and (so in ("REPLAN", "ORIENT", "ADVANCE"))
    # (d) PLAN-STALE + EMPTY history -> RING-PICKED FALLBACK -> STUCK after cap. The sweep is UNIDIRECTIONAL
    #     (turn always +, never <0); the push is backward-if-pushable / else strafe (NEVER forward -> no ram).
    cf = ExploreController(cfg, no_takeoff=True)
    cf.fallback_max_attempts = 3            # small cap so STUCK is reached within the drive window
    cf._ever_tracked = True                 # a MID-FLIGHT loss (history wiped by a wall hit), not startup warmup
    cf.command_history.clear()
    cf._last_ring = [[0.0, 5.0], [45.0, 5.0], [90.0, 5.0], [135.0, 1.0],
                     [180.0, 1.0], [-135.0, 1.0], [-90.0, 5.0], [-45.0, 5.0]]   # back pushable (1.0 >= 0.7)
    seen, saw_fwd, saw_back, saw_turn_pos, saw_turn_neg, t = set(), False, False, False, False, 0.0
    for _ in range(int(30.0 / 0.05)):
        a, s, _ = cf.step(t, stale, False, status="PLAN-STALE")
        seen.add(s)
        if s == "FALLBACK":
            if float(a.get("trigger", 0.0)) > 0:
                saw_fwd = True                        # forward push must NEVER happen (no-ram principle)
            if float(a.get("reverse", 0.0)) > 0:
                saw_back = True                       # ring-picked backward push (back is pushable here)
            y = float(a.get("yaw", 0.0))
            if y > 0:
                saw_turn_pos = True
            if y < 0:
                saw_turn_neg = True                   # must NEVER happen (unidirectional sweep)
        t += 0.05
    fallback_ok = ("FALLBACK" in seen and "STUCK" in seen and saw_back and not saw_fwd
                   and saw_turn_pos and not saw_turn_neg)
    # the fallback turn is a SINGLE gentle recovery step (recovery_turn_step_deg=15), never a 90/135/180 escalation.
    fallback_le45 = (cf.recovery_turn_step_deg <= 45.0)
    # (e) a WALL collision clears the command history (post-impact orientation is unknown).
    ce = ExploreController(cfg, no_takeoff=True)
    ce.reverse_probe_on_wall = False
    padv = {"plan_valid": True, "done": False, "goal": [3.0, 0.0], "pos": [0.0, 0.0],
            "bearing_err": 0.0, "forward_clearance_dist": 5.0}
    t, _, _, _ = _drive(ce, padv, False, 0.6, 0.0)                 # reach ADVANCE
    ce.command_history.append({"kind": "forward", "value": 0.2, "duration_s": 1.0})
    _drive(ce, padv, True, 0.05, t)                                # wall_contact -> clears history
    wall_clear_ok = (len(ce.command_history) == 0)
    rec_ok = (invert_ok and hold_ok and rewind_ok and snap_ok and fallback_ok and fallback_le45 and wall_clear_ok)
    ok = ok and rec_ok
    print(f"[self-test] {'PASS' if rec_ok else 'FAIL'}  RECOVERY control-space (invert={invert_ok}, "
          f"LOST->hold={hold_ok}, STALE->rewind={rewind_ok}, OK->snapback={snap_ok}, "
          f"empty->fallback<=45={fallback_ok and fallback_le45}, wall-clears-history={wall_clear_ok})")

    # ---- SESSION-12 recovery redesign (persist across flicker; consuming rewind; ghost-path guard; confirm) ----
    stale_nr = {"plan_valid": False, "goal": None, "pos": [0.0, 0.0], "clearance_ring": None}
    lost_nr = {"plan_valid": False, "goal": None, "pos": [0.0, 0.0]}
    # (f) FLICKER PERSISTENCE: a PLAN-LOST<->PLAN-STALE flicker must NOT reset recovery -> STUCK stays reachable
    #     (the flight-20260713 frantic loop). Arm recovery on STALE (fallback counter climbs), flick to LOST
    #     (HOLD_LOST), back to STALE -> the counter PERSISTS and STUCK is reached.
    cflk = ExploreController(cfg, no_takeoff=True)
    cflk.fallback_max_attempts = 3
    cflk._ever_tracked = True
    tt = 0.0
    tt, _, _, _ = _drive(cflk, stale_nr, False, 3.0, tt, status="PLAN-STALE")   # arm + a couple fallback attempts
    att_mid, rec_mid = cflk._fallback_attempts, cflk._recovering
    tt, _, _, s_lost = _drive(cflk, lost_nr, False, 1.0, tt, status="PLAN-LOST")  # flicker -> HOLD_LOST
    att_after, rec_after = cflk._fallback_attempts, cflk._recovering              # must NOT reset
    seen_f = set()
    for _ in range(int(30.0 / 0.05)):
        _a, s2, _ = cflk.step(tt, stale_nr, False, status="PLAN-STALE"); seen_f.add(s2); tt += 0.05
    flicker_ok = (rec_mid and att_mid >= 1 and rec_after and att_after >= att_mid
                  and "HOLD_LOST" in s_lost and "STUCK" in seen_f)
    # (g) CONSUMING REWIND: each REWIND cycle pops ONE maneuver; the history DRAINS to empty then -> FALLBACK.
    crw = ExploreController(cfg, no_takeoff=True)
    crw._ever_tracked = True
    for _ in range(4):
        crw.command_history.append({"kind": "forward", "value": 0.2, "duration_s": 0.3})
    seen_g = set(); tt = 0.0
    for _ in range(int(20.0 / 0.05)):
        _a, s3, _ = crw.step(tt, stale_nr, False, status="PLAN-STALE"); seen_g.add(s3); tt += 0.05
    drain_ok = (len(crw.command_history) == 0 and "REWIND" in seen_g and "FALLBACK" in seen_g)
    # (h) GHOST-PATH GUARD: a re-lock that already MOVED (_history_broken) -> a secondary PLAN-STALE CLEARS the
    #     now-stale leftover history and jumps straight to FALLBACK (no displaced ghost-path REWIND replay).
    cgp = ExploreController(cfg, no_takeoff=True)
    cgp._ever_tracked = True
    cgp._recovering = True
    cgp._history_broken = True
    cgp.command_history.append({"kind": "forward", "value": 0.2, "duration_s": 0.3})
    cgp.command_history.append({"kind": "turn", "theta": 30.0})
    cgp.state = "HOLD_LOST"
    _a, s_gp, _ = cgp.step(0.0, stale_nr, False, status="PLAN-STALE")
    ghost_ok = (s_gp == "FALLBACK" and len(cgp.command_history) == 0)
    # (i) _history_broken is SET by entering a spatial state while recovering, and NOT by a non-spatial state.
    chb = ExploreController(cfg, no_takeoff=True); chb._recovering = True; chb._enter("ORIENT", 0.0)
    chb2 = ExploreController(cfg, no_takeoff=True); chb2._recovering = True; chb2._enter("SETTLE", 0.0)
    hb_ok = (chb._history_broken is True and chb2._history_broken is False)
    # (j) CONFIRMING ADVANCE: >= recovery_confirm_dist of progress restores trust (drop flags, reset counter,
    #     clear history); a sub-threshold bump does NOT confirm.
    padv_c = {"plan_valid": True, "done": False, "goal": [10.0, 0.0], "pos": [0.0, 0.0], "bearing_err": 0.0,
              "forward_clearance_dist": 8.0, "pos_y": 0.0, "slam_ms": 120.0, "frame_id": 1}
    cca = ExploreController(cfg, no_takeoff=True)
    cca._recovering = True; cca._history_broken = True; cca._fallback_attempts = 5
    cca.command_history.append({"kind": "turn", "theta": 15.0})
    cca.leg_goal = [10.0, 0.0]; cca.target_altitude_y = 0.0; cca.state = "ADVANCE"
    cca.step(0.0, padv_c, False, status="OK")                                   # capture start = [0,0]
    cca.step(0.1, dict(padv_c, pos=[1.2, 0.0], frame_id=2), False, status="OK")  # moved 1.2 >= 1.0 -> confirm
    confirm_ok = (not cca._recovering and not cca._history_broken and cca._fallback_attempts == 0
                  and len(cca.command_history) == 0)
    ccb = ExploreController(cfg, no_takeoff=True)
    ccb._recovering = True; ccb.leg_goal = [10.0, 0.0]; ccb.target_altitude_y = 0.0; ccb.state = "ADVANCE"
    ccb.step(0.0, padv_c, False, status="OK")
    ccb.step(0.1, dict(padv_c, pos=[0.5, 0.0], frame_id=2), False, status="OK")  # only 0.5u -> NOT confirmed
    noconfirm_ok = (ccb._recovering is True)
    # (k) D2 SCRAPE GUARD: a strafe pick while pinned close behind + forward clearly open -> reposition_fwd
    #     (drives FORWARD) then hands off to the queued strafe.
    crp = ExploreController(cfg, no_takeoff=True)
    crp.state = "PARALLAX_PUSH"; crp._push_dir = None; crp._push_count = 0
    crp.leg_goal = [0.0, 5.0]; crp.target_altitude_y = 0.0
    ring_pin = [[0.0, 8.0], [90.0, 3.0], [-90.0, None], [180.0, 0.3]]            # back 0.3 close, left open(None)
    ppush = {"plan_valid": True, "done": False, "goal": [0.0, 5.0], "pos": [0.0, 0.0], "bearing_err": 0.0,
             "clearance_ring": ring_pin, "forward_clearance_dist": 8.0, "pos_y": 0.0, "slam_ms": 120.0, "frame_id": 1}
    _a1, _s1, _ = crp.step(0.0, ppush, False, status="OK")
    repos_chosen = (crp._push_dir == "reposition_fwd"
                    and crp._push_after_reposition in ("strafe_left", "strafe_right"))
    drove_fwd = float(_a1.get("trigger", 0.0)) > 0
    tt, handed = 0.0, False
    for _ in range(int((crp.strafe_reposition_fwd_s + 1.0) / 0.05)):
        crp.step(tt, ppush, False, status="OK")
        if crp._push_dir in ("strafe_left", "strafe_right"):
            handed = True; break
        tt += 0.05
    d2_ok = (repos_chosen and drove_fwd and handed)
    # (l) D4 mission-end STUCK summary formatting (empty + a populated interval).
    _a0, _b0 = datetime(2026, 7, 13, 10, 18, 48, 771000), datetime(2026, 7, 13, 10, 19, 30, 171000)
    d4_ok = (_stuck_summary([]) == "no STUCK episodes."
             and "STUCK 1x" in _stuck_summary([(_a0, _b0)]) and "41.4s" in _stuck_summary([(_a0, _b0)]))
    s12_ok = (flicker_ok and drain_ok and ghost_ok and hb_ok and confirm_ok and noconfirm_ok and d2_ok and d4_ok)
    ok = ok and s12_ok
    print(f"[self-test] {'PASS' if s12_ok else 'FAIL'}  SESSION-12 recovery redesign "
          f"(flicker-persist={flicker_ok}, consuming-drain={drain_ok}, ghost-path-guard={ghost_ok}, "
          f"history-broken-set={hb_ok}, confirm>=1u={confirm_ok}, sub-1u-no-confirm={noconfirm_ok}, "
          f"D2-reposition={d2_ok}, D4-summary={d4_ok})")

    # ---- SESSION-16: a SETTLE between EVERY recovery action (no back-to-back reverse/spin: let SLAM re-lock) ----
    rec_stale = {"plan_valid": False, "goal": None, "pos": [0.0, 0.0], "clearance_ring": None}
    # (a) REWIND: with 2 flown maneuvers, a PLAN-STALE recovery must HOLD (_rec_settling) BETWEEN popping each
    #     inverse — not fire them back-to-back. Inject a live frame stream so the bounded lost-SLAM settle resolves.
    crs = ExploreController(cfg, no_takeoff=True); crs._ever_tracked = True
    crs.rest_between_s = 0.1; crs.recovery_settle_frames = 2
    crs.command_history.append({"kind": "forward", "value": 0.2, "duration_s": 0.2})
    crs.command_history.append({"kind": "forward", "value": 0.2, "duration_s": 0.2})
    rewind_settled, seen_rw, trs, fidrs = False, set(), 0.0, 200
    for _ in range(int(15.0 / 0.05)):
        fidrs += 1
        _a, s, _ = crs.step(trs, dict(rec_stale, frame_id=fidrs, cap_ts=trs, slam_ms=200.0), False, status="PLAN-STALE")
        seen_rw.add(s)
        if s == "REWIND" and crs._rec_settling:
            rewind_settled = True
        trs += 0.05
    rewind_settle_ok = rewind_settled and "REWIND" in seen_rw and "FALLBACK" in seen_rw
    # (b) FALLBACK: empty history -> ring-picked sweep; consecutive attempts must be SEPARATED by a settle, and
    #     STUCK is still reached at the cap.
    cfs = ExploreController(cfg, no_takeoff=True); cfs._ever_tracked = True
    cfs.rest_between_s = 0.1; cfs.recovery_settle_frames = 2; cfs.fallback_max_attempts = 3
    fb_settled, seen_fb, tfs, fidfs = False, set(), 0.0, 300
    for _ in range(int(25.0 / 0.05)):
        fidfs += 1
        _a, s, _ = cfs.step(tfs, dict(rec_stale, frame_id=fidfs, cap_ts=tfs, slam_ms=200.0), False, status="PLAN-STALE")
        seen_fb.add(s)
        if s == "FALLBACK" and cfs._rec_settling:
            fb_settled = True
        tfs += 0.05
    fb_settle_ok = fb_settled and "STUCK" in seen_fb
    # (c) BOUNDED escape: with NO fresh frames (frame_id absent) the recovery settle must still END at
    #     recovery_settle_max_s (dead pipeline) so a re-exposure maneuver follows — never hang.
    cbnd = ExploreController(cfg, no_takeoff=True)
    cbnd._settle_begin(0.0)
    cbnd.recovery_settle_max_s = 0.5
    d_early, _ = cbnd._settle_poll(0.2, {"pos": [0, 0]}, require_fast=False, min_frames=4, max_hold_s=0.5)
    d_cap, capped = cbnd._settle_poll(0.6, {"pos": [0, 0]}, require_fast=False, min_frames=4, max_hold_s=0.5)
    bounded_ok = (not d_early) and d_cap and capped
    # (d) FALLBACK step ORDER: turn (yaw) FIRST, ring-picked push (reverse/strafe) LAST -> the motion right before
    #     the settle is a parallax translation, not a bare rotation. Back is pushable (ring None), so expect a
    #     reverse push, and its index must be AFTER the yaw's.
    cord = ExploreController(cfg, no_takeoff=True); cord._last_ring = None
    cord._begin_fallback(0.0, None)
    fsteps = cord._player.steps
    yaw_i = next((k for k, st in enumerate(fsteps) if "yaw" in st), None)
    push_i = next((k for k, st in enumerate(fsteps) if "reverse" in st or "joy_horizontal" in st), None)
    order_ok = yaw_i is not None and push_i is not None and yaw_i < push_i
    rec_settle_ok = rewind_settle_ok and fb_settle_ok and bounded_ok and order_ok
    ok = ok and rec_settle_ok
    print(f"[self-test] {'PASS' if rec_settle_ok else 'FAIL'}  RECOVERY inter-action settles "
          f"(REWIND holds between pops={rewind_settle_ok}, FALLBACK holds between attempts+STUCK={fb_settle_ok}, "
          f"bounded-escape-when-dead={bounded_ok}, turn-before-push={order_ok})")

    # ---- SLAM frame-timing settle gate (stop moving while SLAM chokes; resume once it settles) ----
    # (a) _update_slam: counts consecutive FRESH fast frames (deduped on frame_id) + feeds the rolling
    #     _slam_hist window (session 24); a slow frame resets the streak AND breaks window health.
    cs = ExploreController(cfg, no_takeoff=True)
    cs.slam_slow_ms = 1000.0
    cs._update_slam({"slam_ms": 200, "frame_id": 1, "cap_ts": 0.0})
    cs._update_slam({"slam_ms": 200, "frame_id": 1, "cap_ts": 0.0})   # same frame_id -> counted once
    streak1 = (cs._slam_fast_streak == 1 and len(cs._slam_hist) == 1)
    for fid in range(2, 1 + cs.settle_fresh_frames):     # fill the window to settle_fresh_frames total entries
        cs._update_slam({"slam_ms": 200, "frame_id": fid, "cap_ts": float(fid)})
    window_ready_ok = cs._slam_window_ready() and not cs._slam_slow
    cs._update_slam({"slam_ms": 1500, "frame_id": 999, "cap_ts": 999.0})   # a slow fresh frame breaks the window
    slow_ok = cs._slam_slow and (cs._slam_fast_streak == 0) and (not cs._slam_window_ready())
    track_ok = streak1 and window_ready_ok and slow_ok

    # (b) ADVANCE + a slow frame -> SLAM_HOLD (logs the sub-leg), then fresh healthy frames settle -> resume
    #     ADVANCE (session 24: needs both the rolling window full+clean AND settle_gate_s elapsed since hold
    #     entry -- explicit cap_ts per tick, small settle_gate_s to keep this test's tick budget short).
    cadv = ExploreController(cfg, no_takeoff=True)
    cadv.settle_gate_s = 0.05
    padv2 = {"plan_valid": True, "done": False, "goal": [5.0, 0.0], "pos": [0.0, 0.0], "bearing_err": 0.0,
             "forward_clearance_dist": 5.0}
    t = 0.0
    for i in range(30):
        _a, s, _ = cadv.step(t, dict(padv2, frame_id=i, slam_ms=200.0, cap_ts=t), False, status="OK"); t += 0.05
        if s == "ADVANCE":
            break
    reached_adv = (cadv.state == "ADVANCE")
    _a, s_hold, _ = cadv.step(t, dict(padv2, frame_id=100, slam_ms=1500.0, cap_ts=t), False, status="OK"); t += 0.05
    adv_held = (s_hold == "SLAM_HOLD")
    logged_fwd = any(m["kind"] == "forward" for m in cadv.command_history)
    for i in range(101, 101 + cadv.settle_fresh_frames + 2):
        _a, _s, _ = cadv.step(t, dict(padv2, frame_id=i, slam_ms=200.0, cap_ts=t), False, status="OK"); t += 0.05
    adv_resumed = (cadv.state == "ADVANCE")
    adv_gate_ok = reached_adv and adv_held and logged_fwd and adv_resumed

    # (c2) a slow frame AT turn completion -> hold before flying the shaky post-turn pose (the ~45deg gap).
    cpt = ExploreController(cfg, no_takeoff=True)
    pturn = {"plan_valid": True, "done": False, "goal": [0.0, 5.0], "pos": [0.0, 0.0], "bearing_err": 45.0,
             "forward_clearance_dist": 5.0,
             "clearance_ring": [[r, 5.0] for r in (0.0, 45.0, 90.0, 135.0, 180.0, -135.0, -90.0, -45.0)]}
    t, saw_orient = 0.0, False
    for i in range(80):
        _a, s, _ = cpt.step(t, dict(pturn, frame_id=i, slam_ms=1500.0), False, status="OK"); t += 0.05
        saw_orient = saw_orient or (s == "ORIENT")
        if s == "SLAM_HOLD":
            break
    postturn_ok = saw_orient and (cpt.state == "SLAM_HOLD") and (cpt._slam_resume in ("ADVANCE", "PARALLAX_PUSH"))

    # (d2) bug-1: a sub-0.1s translation is now LOGGED (no duration guard) and inverts into the rewind.
    csh = ExploreController(cfg, no_takeoff=True)
    csh._log_move("forward", 0.2, 0.02)
    short_logged = (len(csh.command_history) == 1 and csh.command_history[0]["kind"] == "forward")
    short_inv_ok = any("reverse" in step for step in csh._invert_history())
    bug1_ok = short_logged and short_inv_ok

    slam_ok = track_ok and adv_gate_ok and postturn_ok and bug1_ok
    ok = ok and slam_ok
    print(f"[self-test] {'PASS' if slam_ok else 'FAIL'}  SLAM settle-gate (track={track_ok}, "
          f"ADVANCE-slow->hold->resume={adv_gate_ok}, turn-slow->hold={postturn_ok}, "
          f"bug1 short-move-logged={bug1_ok})")

    # ---- session 24: settle-gate two-gate design (FRESHNESS + PHYSICAL MOTION, decoupled) ----
    # (g1) FRESHNESS alone: full+healthy+timestamped window -> ready; one slow entry, a MISSING cap_ts (a
    #      frame we can't timestamp must never look "already clean"), or an incomplete window all fail it.
    cg_fresh = ExploreController(cfg, no_takeoff=True)
    for fid in range(cg_fresh.settle_fresh_frames):
        cg_fresh._update_slam({"slam_ms": 200.0, "frame_id": fid, "cap_ts": float(fid)})
    fresh_full_ok = cg_fresh._slam_window_ready()
    cg_slow = ExploreController(cfg, no_takeoff=True)
    for fid in range(cg_slow.settle_fresh_frames):
        ms = 1500.0 if fid == 0 else 200.0
        cg_slow._update_slam({"slam_ms": ms, "frame_id": fid, "cap_ts": float(fid)})
    fresh_slow_fails = not cg_slow._slam_window_ready()
    cg_none = ExploreController(cfg, no_takeoff=True)
    for fid in range(cg_none.settle_fresh_frames):
        cg_none._update_slam({"slam_ms": 200.0, "frame_id": fid})   # no cap_ts key at all -> None
    fresh_none_fails = not cg_none._slam_window_ready()
    cg_partial = ExploreController(cfg, no_takeoff=True)
    for fid in range(cg_partial.settle_fresh_frames - 1):           # one short of a full window
        cg_partial._update_slam({"slam_ms": 200.0, "frame_id": fid, "cap_ts": float(fid)})
    fresh_partial_fails = not cg_partial._slam_window_ready()
    freshness_ok = fresh_full_ok and fresh_slow_fails and fresh_none_fails and fresh_partial_fails

    # (g2) MOTION gate alone: even a fully-clean window must still wait out settle_gate_s from when the gate
    #      opened -- gate 1 (freshness) alone can't shortcut gate 2 (physical dwell). One frame captured AT the
    #      gate-open instant keeps freshness (incl. `latest_since`, closing the stale-prequalified-window bug)
    #      satisfied throughout, so only the dwell timer is under test here.
    cg_motion = ExploreController(cfg, no_takeoff=True)
    cg_motion.settle_gate_s = 0.5
    for fid in range(cg_motion.settle_fresh_frames):
        cg_motion._update_slam({"slam_ms": 200.0, "frame_id": fid, "cap_ts": float(fid)})
    cg_motion._settle_gate_begin(10.0)
    cg_motion._update_slam({"slam_ms": 200.0, "frame_id": 900, "cap_ts": 10.0})   # fresh AT gate-open
    motion_too_soon = not cg_motion._settle_gate_poll(10.1)     # only 0.1s elapsed < 0.5s
    motion_ok_later = cg_motion._settle_gate_poll(10.5)         # 0.5s elapsed -> both gates clear
    motion_gate_ok = motion_too_soon and motion_ok_later

    # (g3) CATEGORY A: a SETTLE opened at active-motion-end pays the FULL settle_gate_s even if the window is
    #      already clean at the moment motion ends (no free pass on the motion gate from stale pre-motion health).
    ca24 = ExploreController(cfg, no_takeoff=True)
    ca24.settle_gate_s = 0.3
    ca24._enter("ADVANCE", 0.0)          # a plain, non-SLAM_HOLD prior state
    for fid in range(ca24.settle_fresh_frames):
        ca24._update_slam({"slam_ms": 200.0, "frame_id": fid, "cap_ts": float(fid)})
    ca24._settle_to = "REPLAN"
    ca24._enter("SETTLE", 5.0)            # Category A: fresh gate opens HERE, window already clean beforehand
    catA_too_soon = not ca24._settle_gate_poll(5.05)      # 0.05s < 0.3s -> not yet
    _a, s24, _ = ca24.step(5.35, {"plan_valid": True, "goal": [1.0, 0.0], "pos": [0.0, 0.0], "bearing_err": 0.0,
                                  "frame_id": 900, "cap_ts": 5.35, "slam_ms": 200.0}, False)
    catA_proceeds = (s24 == "REPLAN")
    category_a_ok = catA_too_soon and catA_proceeds

    # (g4) CATEGORY B regression (the ORIGINAL double-wait bug): a SLAM_HOLD open for LONGER than settle_gate_s
    #      that just became freshness-clean resumes to SETTLE, which must pass on its VERY NEXT tick -- no
    #      second full wait stacked on top of the one the hold already paid.
    cb24 = ExploreController(cfg, no_takeoff=True)
    cb24.settle_gate_s = 0.2
    cb24._enter_slam_hold("SETTLE", 0.0, "test")      # gate opens at t=0.0
    cb24._settle_to = "REPLAN"
    t = 0.0
    for fid in range(cb24.settle_fresh_frames):       # fills well past settle_gate_s (0.2s) elapsed
        t += 0.05
        cb24._update_slam({"slam_ms": 200.0, "frame_id": fid, "cap_ts": t})
    _a, s_b1, _ = cb24.step(t, {"plan_valid": True, "goal": [1.0, 0.0], "pos": [0.0, 0.0], "bearing_err": 0.0,
                                "frame_id": 100 + cb24.settle_fresh_frames, "cap_ts": t, "slam_ms": 200.0}, False)
    resumed_to_settle = (s_b1 == "SETTLE")
    _a, s_b2, _ = cb24.step(t + 0.01, {"plan_valid": True, "goal": [1.0, 0.0], "pos": [0.0, 0.0], "bearing_err": 0.0,
                                       "frame_id": 200, "cap_ts": t + 0.01, "slam_ms": 200.0}, False)
    settle_passes_first_tick = (s_b2 == "REPLAN")
    category_b_ok = resumed_to_settle and settle_passes_first_tick

    # (g5) CATEGORY C: SLAM_HOLD resuming to ADVANCE (previously a weaker 3-frame, no-cap_ts, no-dwell check)
    #      now shares the SAME two-gate check: stays held while SLAM is slow, resumes once both gates clear.
    cc24 = ExploreController(cfg, no_takeoff=True)
    cc24.settle_gate_s = 0.05
    cc24._enter_slam_hold("ADVANCE", 0.0, "test")
    t = 0.0
    for i in range(20):
        t += 0.05
        cc24.step(t, {"plan_valid": True, "goal": [1.0, 0.0], "pos": [0.0, 0.0], "bearing_err": 0.0,
                     "forward_clearance_dist": 5.0, "frame_id": i, "cap_ts": t, "slam_ms": 1500.0}, False)
    catC_holds_while_slow = (cc24.state == "SLAM_HOLD")
    s_c = cc24.state
    for i in range(20, 20 + cc24.settle_fresh_frames + 1):
        t += 0.05
        _a, s_c, _ = cc24.step(t, {"plan_valid": True, "goal": [1.0, 0.0], "pos": [0.0, 0.0], "bearing_err": 0.0,
                                   "forward_clearance_dist": 5.0, "frame_id": i, "cap_ts": t, "slam_ms": 200.0}, False)
    catC_resumes = (s_c == "ADVANCE")
    category_c_ok = catC_holds_while_slow and catC_resumes

    gate24_ok = freshness_ok and motion_gate_ok and category_a_ok and category_b_ok and category_c_ok
    ok = ok and gate24_ok
    print(f"[self-test] {'PASS' if gate24_ok else 'FAIL'}  settle-gate two-gate design "
          f"(freshness={freshness_ok}, motion={motion_gate_ok}, category-A-full-dwell={category_a_ok}, "
          f"category-B-no-double-wait={category_b_ok}, category-C-now-gated={category_c_ok})")

    # ---- SLAM-settle REWIND step-back (sustained slow in a HOLD -> step back through the rewind queue) ----
    padv3 = {"plan_valid": True, "done": False, "goal": [9.0, 0.0], "pos": [0.0, 0.0], "bearing_err": 0.0,
             "forward_clearance_dist": 9.0}
    # (a) the slow-streak counter mirrors the fast-streak: increments on slow FRESH frames, resets on a fast one.
    csb = ExploreController(cfg, no_takeoff=True)
    for i in range(4):
        csb._update_slam({"slam_ms": 1500.0, "frame_id": i})
    slow_streak_ok = (csb._slam_slow_streak == 4 and csb._slam_fast_streak == 0)
    csb._update_slam({"slam_ms": 200.0, "frame_id": 99})
    slow_reset_ok = (csb._slam_slow_streak == 0)

    # (b) ADVANCE -> a slow frame -> SLAM_HOLD; sustained slow -> SLAM_STEPBACK pops the forward move and
    #     plays its inverse (a reverse), then returns to SLAM_HOLD to keep waiting.
    csb2 = ExploreController(cfg, no_takeoff=True)
    csb2.slam_stepback_after_frames, csb2.slam_stepback_max_steps = 4, 2
    tb, fb = 0.0, 0
    reached = False
    for _ in range(40):
        _a, s, _ = csb2.step(tb, dict(padv3, frame_id=fb, slam_ms=200.0), False, status="OK"); tb += 0.05; fb += 1
        if s == "ADVANCE":
            reached = True; break
    _a, s, _ = csb2.step(tb, dict(padv3, frame_id=fb, slam_ms=1500.0), False, status="OK"); tb += 0.05; fb += 1
    held_ok = reached and (s == "SLAM_HOLD") and any(m["kind"] == "forward" for m in csb2.command_history)
    hist_before = len(csb2.command_history)
    saw_stepback = False
    for _ in range(12):
        _a, s, _ = csb2.step(tb, dict(padv3, frame_id=fb, slam_ms=1500.0), False, status="OK"); tb += 0.05; fb += 1
        if s == "SLAM_STEPBACK":
            saw_stepback = True; break
    popped_ok = saw_stepback and (len(csb2.command_history) == hist_before - 1) and (csb2._slam_stepback_count == 1)
    saw_reverse, back_hold = False, False
    for _ in range(60):
        a, s, _ = csb2.step(tb, dict(padv3, frame_id=fb, slam_ms=1500.0), False, status="OK"); tb += 0.05; fb += 1
        if a.get("reverse"):
            saw_reverse = True
        if s == "SLAM_HOLD":
            back_hold = True; break
    stepback_ok = held_ok and popped_ok and saw_reverse and back_hold

    # (c) cap: a longer pre-seeded history + sustained slow -> at most slam_stepback_max_steps step-backs.
    csb3 = ExploreController(cfg, no_takeoff=True)
    csb3.slam_stepback_after_frames, csb3.slam_stepback_max_steps = 3, 2
    for _ in range(4):
        csb3._log_move("forward", 0.2, 0.05)
    hist0 = len(csb3.command_history)
    csb3._enter_slam_hold("ADVANCE", 0.0, "test")
    tb, fb = 0.05, 0
    for _ in range(200):
        csb3.step(tb, dict(padv3, frame_id=fb, slam_ms=1500.0), False, status="OK"); tb += 0.05; fb += 1
    cap_ok = (csb3._slam_stepback_count == 2) and (len(csb3.command_history) == hist0 - 2)

    # (d) empty rewind queue -> never enters SLAM_STEPBACK, just keeps holding (no silent fallback / crash).
    csb4 = ExploreController(cfg, no_takeoff=True)
    csb4.slam_stepback_after_frames = 3
    csb4._enter_slam_hold("ADVANCE", 0.0, "test")     # command_history is empty
    tb, fb, empty_ok = 0.05, 0, True
    for _ in range(20):
        _a, s, _ = csb4.step(tb, dict(padv3, frame_id=fb, slam_ms=1500.0), False, status="OK"); tb += 0.05; fb += 1
        if s == "SLAM_STEPBACK":
            empty_ok = False; break
    empty_ok = empty_ok and (csb4.state == "SLAM_HOLD")

    # (e) PLAN-LOST while holding -> HOLD_LOST (the step-back is OK-only; recovery owns the loss path).
    csb5 = ExploreController(cfg, no_takeoff=True)
    for _ in range(3):
        csb5._log_move("forward", 0.2, 0.05)
    csb5._enter_slam_hold("ADVANCE", 0.0, "test")
    for i in range(10):
        csb5._update_slam({"slam_ms": 1500.0, "frame_id": i})
    _a, s_lost, _ = csb5.step(0.5, dict(padv3, frame_id=50, slam_ms=1500.0), False, status="PLAN-LOST")
    lost_ok = (s_lost == "HOLD_LOST")

    stepback_selftest_ok = (slow_streak_ok and slow_reset_ok and stepback_ok and cap_ok and empty_ok and lost_ok)
    ok = ok and stepback_selftest_ok
    print(f"[self-test] {'PASS' if stepback_selftest_ok else 'FAIL'}  SLAM step-back "
          f"(streak={slow_streak_ok and slow_reset_ok}, ADVANCE-slow->stepback={stepback_ok}, "
          f"cap={cap_ok}, empty->hold={empty_ok}, LOST-suppresses={lost_ok})")

    # ---- SLAM_STEPBACK counter PERSISTENCE across a PLAN-LOST bounce + goal-change reset (bug diagnosed
    #      off the 20260718 flight: #1/3 fired repeatedly, never escalating, because a bad SLAM patch always
    #      bounces PLAN-LOST -> HOLD_LOST -> OK before the next solve, and the old code reset the counter on
    #      EVERY fresh _enter_slam_hold, wiping it before it could ever reach #2 or #3). ----
    padv6 = {"plan_valid": True, "done": False, "goal": [9.0, 0.0], "pos": [0.0, 0.0], "bearing_err": 0.0,
             "forward_clearance_dist": 9.0}
    csp = ExploreController(cfg, no_takeoff=True)
    csp.slam_stepback_after_frames, csp.slam_stepback_max_steps = 3, 3
    for _ in range(6):
        csp._log_move("forward", 0.2, 0.05)
    csp._enter_slam_hold("ADVANCE", 0.0, "test")
    tp, fidp, saw_sb1 = 0.05, 0, False
    for _ in range(60):
        _a, s, _ = csp.step(tp, dict(padv6, frame_id=fidp, slam_ms=1500.0), False, status="OK")
        tp += 0.05; fidp += 1
        if s == "SLAM_STEPBACK":
            saw_sb1 = True
            break
    sb1_ok = saw_sb1 and csp._slam_stepback_count == 1
    for _ in range(20):    # drain the step-back player back to SLAM_HOLD
        _a, s, _ = csp.step(tp, dict(padv6, frame_id=fidp, slam_ms=1500.0), False, status="OK")
        tp += 0.05; fidp += 1
        if s == "SLAM_HOLD":
            break
    # (a) a PLAN-LOST bounce must PRESERVE the count (not reset it).
    _a, s_bounce, _ = csp.step(tp, dict(padv6, frame_id=fidp, slam_ms=1500.0), False, status="PLAN-LOST")
    tp += 0.05; fidp += 1
    bounce_ok = (s_bounce == "HOLD_LOST" and csp._slam_stepback_count == 1)
    # (b) OK returning re-enters a FRESH SLAM_HOLD (the generic recovery convergence) — must still NOT reset.
    _a, s_fresh, _ = csp.step(tp, dict(padv6, frame_id=fidp, slam_ms=200.0), False, status="OK")
    tp += 0.05; fidp += 1
    fresh_hold_ok = (s_fresh == "SLAM_HOLD" and csp._slam_stepback_count == 1)
    # (c) continuing to go slow in that fresh hold must escalate to #2 (proves the cap is reachable again).
    saw_sb2 = False
    for _ in range(60):
        _a, s, _ = csp.step(tp, dict(padv6, frame_id=fidp, slam_ms=1500.0), False, status="OK")
        tp += 0.05; fidp += 1
        if s == "SLAM_STEPBACK":
            saw_sb2 = True
            break
    escalate_ok = saw_sb2 and csp._slam_stepback_count == 2
    # (d) a genuinely NEW committed goal (REPLAN) resets the count unconditionally, even outside a recovery.
    csg = ExploreController(cfg, no_takeoff=True)
    csg._slam_stepback_count = 2
    csg._enter("REPLAN", 0.0)
    _a, s_goal, _ = csg.step(0.0, dict(padv6, goal=[3.0, 4.0]), False, status="OK")
    goal_reset_ok = (csg._slam_stepback_count == 0)
    stepback_persist_ok = sb1_ok and bounce_ok and fresh_hold_ok and escalate_ok and goal_reset_ok
    ok = ok and stepback_persist_ok
    print(f"[self-test] {'PASS' if stepback_persist_ok else 'FAIL'}  SLAM-STEPBACK counter PERSISTENCE "
          f"(first={sb1_ok}, bounce-preserves={bounce_ok}, fresh-hold-preserves={fresh_hold_ok}, "
          f"escalates-to-2={escalate_ok}, goal-change-resets={goal_reset_ok})")

    # ---- Reactive blind-hold back_off on a flow wall/backwall contact (HOLD_LOST / waiting SLAM_HOLD) ----
    # (a) HOLD_LOST + a live wall contact -> BLIND_BACKOFF plays back_off, then resumes HOLD_LOST; a
    #     CONTINUOUSLY true contact must not re-trigger every tick (edge-triggered); clearing + refiring re-arms.
    cbb = ExploreController(cfg, no_takeoff=True)
    cbb._enter("HOLD_LOST", 0.0)
    plost = {"plan_valid": False, "done": False, "goal": None, "pos": [0.0, 0.0]}
    _a, s1, _ = cbb.step(0.0, plost, True, status="PLAN-LOST")
    entry_ok = (s1 == "BLIND_BACKOFF" and cbb._blind_backoff_resume == "HOLD_LOST")
    tb, saw_rev = 0.05, False
    for _ in range(20):
        a, s, _ = cbb.step(tb, plost, True, status="PLAN-LOST")
        tb += 0.05
        if a.get("reverse"):
            saw_rev = True
        if s != "BLIND_BACKOFF":
            break
    resume_ok = (s == "HOLD_LOST") and saw_rev
    _a, s_norefire, _ = cbb.step(tb, plost, True, status="PLAN-LOST")
    tb += 0.05
    norefire_ok = (s_norefire == "HOLD_LOST")   # still touching the same wall -> no second reaction
    cbb.step(tb, plost, False, status="PLAN-LOST"); tb += 0.05             # contact clears -> re-arm
    _a, s_rearm, _ = cbb.step(tb, plost, True, status="PLAN-LOST")         # fires again on a fresh contact
    rearm_ok = (s_rearm == "BLIND_BACKOFF")
    blind_backoff_lost_ok = entry_ok and resume_ok and norefire_ok and rearm_ok
    print(f"[self-test] {'PASS' if blind_backoff_lost_ok else 'FAIL'}  BLIND-HOLD back_off in HOLD_LOST "
          f"(entry={entry_ok}, resumes-hold={resume_ok}, edge-triggered={norefire_ok}, rearms={rearm_ok})")

    # (b) waiting in SLAM_HOLD (settle gate not yet clear) + a live backwall contact -> BLIND_BACKOFF, then
    #     resumes SLAM_HOLD (NOT settle/replan — the plan is still untrusted while it was waiting).
    cbw = ExploreController(cfg, no_takeoff=True)
    cbw._enter_slam_hold("ADVANCE", 0.0, "test")
    pwait = dict(padv6, plan_valid=True, slam_ms=1500.0, frame_id=0)
    _a, sw1, _ = cbw.step(0.05, pwait, False, backwall_contact=True, status="OK")
    bw_entry_ok = (sw1 == "BLIND_BACKOFF" and cbw._blind_backoff_resume == "SLAM_HOLD")
    tw = 0.1
    for _ in range(20):
        _a, sw, _ = cbw.step(tw, pwait, False, backwall_contact=True, status="OK")
        tw += 0.05
        if sw != "BLIND_BACKOFF":
            break
    bw_resume_ok = (sw == "SLAM_HOLD")
    blind_backoff_slam_hold_ok = bw_entry_ok and bw_resume_ok
    ok = ok and blind_backoff_lost_ok and blind_backoff_slam_hold_ok
    print(f"[self-test] {'PASS' if blind_backoff_slam_hold_ok else 'FAIL'}  BLIND-HOLD back_off while waiting "
          f"in SLAM_HOLD (entry={bw_entry_ok}, resumes-SLAM_HOLD-not-settle={bw_resume_ok})")

    # ---- F5 TWO-PHASE HYBRID ASCENT: Phase-1 SLAM-metered UP micro-pulses (dZ gate) -> Phase-2 continuous
    #      latch hold; ceiling_contact -> DESCEND; renewed climb during the hold reverts to Phase 1; an
    #      invalid pose pauses and the ascend_max_s cap is the backstop; + BASELINE_NUDGE seeds the SLAM baseline.
    UPV = cfg["autonomy"].get("explore", {}).get("ascend_cmd", -1)
    def _ascend_plan(i, posy):
        return {"plan_valid": True, "pos_y": posy, "slam_ms": 200.0, "frame_id": i,
                "goal": [9.0, 0.0], "pos": [0.0, 0.0], "bearing_err": 0.0}
    # (a) rise (pos_y DECREASES) then flatten -> Phase 1 pulses/rests, then Phase 2 latch, then CEILING->DESCEND
    ca = ExploreController(cfg, no_takeoff=True)
    ca.ascend_micro_pulse_s, ca.ascend_rest_s = 0.1, 0.1
    ca.ascend_gain_eps, ca.ascend_stall_cycles, ca.ascend_latch_hold_s = 0.05, 2, 0.5
    ca.ascend_max_s = 100.0
    ca._enter("ASCEND", 0.0); ca._ascend_phase = None
    saw_pulse_up = saw_rest = reached_latch = descended = False
    for i in range(400):
        t = i * 0.05
        posy = -0.3 * min(t, 2.0)                     # climbing until t=2.0s, then flat at -0.6
        ceil = (ca._ascend_phase == "LATCH")          # once flush + latching, the flow CEILING fires
        a, s, _ = ca.step(t, _ascend_plan(i, posy), False, ceiling_contact=ceil, status="OK")
        if s == "ASCEND" and ca._ascend_phase == "PULSE" and float(a.get("joy_vertical", 0) or 0) < 0:
            saw_pulse_up = True
        if s == "ASCEND" and ca._ascend_phase == "REST" and not a:
            saw_rest = True
        if ca._ascend_phase == "LATCH":
            reached_latch = True
        if s == "SETTLE" and ca._settle_to == "DESCEND":
            descended = True; break
    ascent_ok = saw_pulse_up and saw_rest and reached_latch and descended
    # (b) Phase-2 revert: in LATCH but the pose shows renewed climb (dZ > eps) -> back to micro-pulses
    cr = ExploreController(cfg, no_takeoff=True)
    cr.ascend_gain_eps, cr.ascend_latch_hold_s, cr.ascend_max_s = 0.05, 1.0, 100.0
    cr._enter("ASCEND", 0.0)
    cr._ascend_phase, cr._ascend_phase_t0, cr._ascend_prev_y, cr._ascend_start_t = "LATCH", 0.0, 0.0, 0.0
    cr.step(0.1, _ascend_plan(1, -0.2), False, ceiling_contact=False, status="OK")   # dropped 0.2 (>eps) -> rising
    revert_ok = (cr._ascend_phase == "PULSE")
    # (c) invalid pose pauses (no dZ) -> never latches -> the ascend_max_s cap sends it to DESCEND
    cp = ExploreController(cfg, no_takeoff=True)
    cp.ascend_micro_pulse_s, cp.ascend_rest_s, cp.ascend_max_s = 0.1, 0.1, 0.5
    cp._enter("ASCEND", 0.0); cp._ascend_phase = None
    cap_descended = never_latched = True
    for i in range(40):
        t = i * 0.05
        a, s, _ = cp.step(t, {"plan_valid": False, "pos_y": None, "slam_ms": 200.0, "frame_id": i,
                              "goal": [9.0, 0.0], "pos": [0.0, 0.0], "bearing_err": 0.0}, False, status="OK")
        if cp._ascend_phase == "LATCH":
            never_latched = False
        if s == "SETTLE" and cp._settle_to == "DESCEND":
            break
    else:
        cap_descended = False
    pause_ok = cap_descended and never_latched
    # (d) BASELINE_NUDGE: pick the roomier axis (forward) from the ring, translate baseline_nudge_dist -> REPLAN
    cbn = ExploreController(cfg, no_takeoff=True)
    cbn.baseline_nudge_dist, cbn.baseline_nudge_max_s, cbn._baseline_seeded = 0.4, 5.0, False
    cbn._enter("BASELINE_NUDGE", 0.0); cbn._push_dir = None
    ring = [[0.0, 5.0], [90.0, 5.0], [180.0, 0.3], [-90.0, 5.0]]      # forward roomy, back blocked
    saw_translate = nudge_replan = False
    t, f = 0.0, 0
    for i in range(300):
        px = min(0.5, 0.01 * i)                                       # creep forward so `traveled` grows
        pl = {"plan_valid": True, "pos": [px, 0.0], "clearance_ring": ring, "slam_ms": 200.0,
              "frame_id": f, "goal": [9.0, 0.0], "bearing_err": 0.0, "forward_clearance_dist": 5.0, "pos_y": 0.0}
        a, s, _ = cbn.step(t, pl, False, status="OK")
        if s == "BASELINE_NUDGE" and float(a.get("trigger", 0) or 0) > 0:
            saw_translate = True
        if s == "SETTLE" and cbn._settle_to == "REPLAN" and cbn._baseline_seeded:
            nudge_replan = True; break
        t += 0.05; f += 1
    nudge_ok = saw_translate and nudge_replan
    # (e) boxed in both axes -> skip the nudge (logged), still seed + go REPLAN
    cbs = ExploreController(cfg, no_takeoff=True)
    cbs._baseline_seeded = False
    cbs._enter("BASELINE_NUDGE", 0.0); cbs._push_dir = None
    _a, s_skip, _ = cbs.step(0.0, {"plan_valid": True, "pos": [0.0, 0.0], "clearance_ring": [[0.0, 0.2], [180.0, 0.2]],
                                   "slam_ms": 200.0, "frame_id": 0, "goal": [9.0, 0.0], "bearing_err": 0.0,
                                   "forward_clearance_dist": 0.2, "pos_y": 0.0}, False, status="OK")
    skip_ok = (s_skip == "SETTLE" and cbs._settle_to == "REPLAN" and cbs._baseline_seeded)
    ascent_all_ok = ascent_ok and revert_ok and pause_ok and nudge_ok and skip_ok
    ok = ok and ascent_all_ok
    print(f"[self-test] {'PASS' if ascent_all_ok else 'FAIL'}  two-phase ascent + baseline nudge "
          f"(phase1->phase2->ceiling={ascent_ok}, revert-on-climb={revert_ok}, invalid-pause->cap={pause_ok}, "
          f"baseline-translate={nudge_ok}, boxed->skip={skip_ok})")

    # ---- F6 no-spin startup: empty history + SLAM never tracked -> WARMUP hold (not the fallback sweep) ----
    cw = ExploreController(cfg, no_takeoff=True)          # _explore_started True (no_takeoff)
    _a, s_warm, _ = cw.step(0.0, {"plan_valid": False}, False, status="PLAN-STALE")
    warmup_ok = (s_warm == "WARMUP") and not cw._ever_tracked
    ct = ExploreController(cfg, no_takeoff=True)          # once SLAM tracks, empty-history STALE -> fallback
    ct.step(0.0, {"plan_valid": True, "done": False, "goal": [1.0, 0.0], "pos": [0.0, 0.0],
                  "bearing_err": 0.0, "frame_id": 0, "slam_ms": 200.0}, False, status="OK")
    tracked_ok = ct._ever_tracked
    _a, s_fb, _ = ct.step(0.1, {"plan_valid": False}, False, status="PLAN-STALE")
    fallback_ok = s_fb in ("REWIND", "FALLBACK")
    startup_ok = warmup_ok and tracked_ok and fallback_ok
    ok = ok and startup_ok
    print(f"[self-test] {'PASS' if startup_ok else 'FAIL'}  no-spin startup "
          f"(warmup-hold={warmup_ok}, marks-tracked={tracked_ok}, later-stale->fallback={fallback_ok})")

    # ---- F7 ram guard: SELF-CALIBRATING (fire on speed < 33% of the drone's own nominal free-flight speed).
    # Small calib params so it calibrates fast; then (A) nominal is learned, (B) a STEADY CRAWL at nominal
    # does NOT false-fire (the exact bug the old absolute goal-closing threshold caused), (C) a true STALL
    # (frozen pos) DOES fire, (D) before calibration a frozen pose does not fire (guard inactive).
    cr = ExploreController(cfg, no_takeoff=True)
    cr.ram_stall_s, cr.ram_speed_window_s = 0.5, 0.2
    cr.ram_calib_skip_s, cr.ram_calib_sample_s, cr.ram_calib_min_sample_s = 0.2, 0.5, 0.2
    cr.leg_max_s = 100.0
    cr.hop_duration_s = 0       # session 20: isolate the SPEED ram guard (cruise mode; no hop preemption)
    gram = {"plan_valid": True, "done": False, "goal": [9.0, 0.0], "bearing_err": 0.0,
            "forward_clearance_dist": 9.0, "pos_y": 0.0}
    tr, fr, x, frozen, crawl_fired, ram_fired = 0.0, 0, 0.0, False, False, False
    for _ in range(600):
        if not frozen and x < 1.2:
            x = round(x + 0.02, 5)          # steady ~0.4 u/s crawl (advances until x=1.2, then FREEZE = true stall)
        else:
            frozen = True
        _a, s, ev = cr.step(tr, dict(gram, pos=[x, 0.0], frame_id=fr, slam_ms=200.0), False, status="OK")
        tr += 0.05; fr += 1
        if ev and "ram guard" in ev and "stop leg" in ev:      # the FIRE event (not the calib note)
            if frozen:
                ram_fired = True; break
            else:
                crawl_fired = True; break     # a steady crawl tripped the guard -> the OLD bug
    ramA = cr._nominal_speed is not None and cr._nominal_speed > 0.1       # (A) sane nominal calibrated
    ramB = not crawl_fired                                                 # (B) the fix: crawl does NOT fire
    ramC = ram_fired and cr.state == "SETTLE" and cr._bump_pulse == [9.0, 0.0]   # (C) true stall fires + bumps
    # (D) frozen pos from the very start -> degenerate calib discarded -> nominal stays None -> guard never fires
    crd = ExploreController(cfg, no_takeoff=True)
    crd.ram_stall_s, crd.ram_speed_window_s = 0.5, 0.2
    crd.ram_calib_skip_s, crd.ram_calib_sample_s, crd.ram_calib_min_sample_s = 0.2, 0.5, 0.2
    crd.leg_max_s = 100.0
    crd.hop_duration_s = 0     # session 20: isolate the SPEED ram guard (cruise mode)
    tr, fr, precalib_fired = 0.0, 0, False
    for _ in range(80):
        _a, s, ev = crd.step(tr, dict(gram, pos=[0.0, 0.0], frame_id=fr, slam_ms=200.0), False, status="OK")
        tr += 0.05; fr += 1
        if ev and "ram guard" in ev:
            precalib_fired = True; break
    ramD = (not precalib_fired) and crd._nominal_speed is None
    ram_ok = ramA and ramB and ramC and ramD
    ok = ok and ram_ok
    print(f"[self-test] {'PASS' if ram_ok else 'FAIL'}  ram guard self-calibrating "
          f"(nominal={ramA}, crawl-no-fire={ramB}, stall-fires={ramC}, pre-calib-no-fire={ramD})")

    # ---- 2-bump blacklist plumbing: _detector_command + the kinematic bump latch ----
    dc_ok = (_detector_command({"reverse": 0.4}) == CMD_BACK
             and _detector_command({"trigger": 0.1}) == CMD_FWD
             and _detector_command({"trigger": 0.1, "joy_vertical": -1}) == CMD_FWD   # altlock ADVANCE -> FWD
             and _detector_command({"joy_vertical": -1}) == CMD_UP
             and _detector_command({"joy_vertical": 1}) == CMD_DOWN                   # DESCEND (down) -> FLOOR
             and _detector_command({"yaw": 1.0}) is None and _detector_command({}) is None)
    ok = ok and dc_ok
    print(f"[self-test] {'PASS' if dc_ok else 'FAIL'}  _detector_command maps reverse->BACK / fwd->FWD / up->UP / down->DOWN")

    # The ram-guard stop above (cr) fired exactly ONE bump pulse toward the leg goal and disarmed the latch,
    # anchored at the stop position (wherever the drone was when it stalled).
    anchor = list(cr._last_bump_anchor)
    latch_armed_once = (cr._bump_pulse == [9.0, 0.0] and cr._bump_armed is False
                        and cr._last_bump_anchor is not None)
    _, first_reason, _, _ = cr.take_bump_pulse()             # publish consumes it (carries the trigger reason)
    reason_ok = first_reason == "ram guard"
    cr._register_bump({"pos": anchor}, "flow WALL contact")  # a stutter while still disarmed -> NO new pulse
    stutter_ok = cr._bump_pulse is None and cr.take_missed_bump() is not None   # but it IS marked MISSED-BUMP
    cr.rearm_bump_if_disengaged({}, {"pos": anchor})        # same spot, no reverse -> stays disarmed
    still_disarmed = cr._bump_armed is False
    cr.rearm_bump_if_disengaged({}, {"pos": [anchor[0] + cr.goal_reach_dist + 0.2, anchor[1]]})   # moved -> re-arm
    rearmed_by_move = cr._bump_armed is True
    cr._register_bump({"pos": [9.0, 0.0]})                   # a genuine 2nd encounter -> a fresh pulse
    second_pulse_ok = cr._bump_pulse == [9.0, 0.0]
    crb = ExploreController(cfg, no_takeoff=True); crb.leg_goal = [5.0, 0.0]
    crb._register_bump({"pos": [0.0, 0.0]}); popped, popped_reason, _, _ = crb.take_bump_pulse()
    crb.rearm_bump_if_disengaged({"reverse": 0.3}, {"pos": [0.0, 0.0]})   # reverse cmd re-arms at 0 displacement
    rearmed_by_reverse = crb._bump_armed is True and popped == [5.0, 0.0]
    # STANDOFF coupling (Bug B): a stand-off bump then the back_off reverse re-arms the latch, so a SECOND
    # stand-off contact at ~the same pinned pose emits a FRESH pulse — this is what lets the planner's
    # 2-bump rule reach 2 at a clearance stand-off (where the drone never reverses/displaces on its own).
    cso = ExploreController(cfg, no_takeoff=True); cso.leg_goal = [4.0, 0.0]
    cso._register_bump({"pos": [3.4, 0.0]}, "clearance stand-off"); p1_so, _, _, _ = cso.take_bump_pulse()
    cso.rearm_bump_if_disengaged({"reverse": 0.7}, {"pos": [3.4, 0.0]})   # the back_off maneuver's reverse re-arms
    cso._register_bump({"pos": [3.42, 0.0]}, "clearance stand-off")       # 2nd standoff pin -> fresh pulse, not missed
    standoff_latch_ok = (p1_so == [4.0, 0.0] and cso._bump_pulse == [4.0, 0.0] and cso._bump_armed is False)
    latch_ok = (latch_armed_once and reason_ok and stutter_ok and still_disarmed and rearmed_by_move
                and second_pulse_ok and rearmed_by_reverse and standoff_latch_ok)
    ok = ok and latch_ok
    print(f"[self-test] {'PASS' if latch_ok else 'FAIL'}  2-bump latch "
          f"(one pulse/contact, stutter-suppressed, re-arm on move|reverse, standoff back-off re-arm)")

    print(f"\n[autopilot][self-test] {'ALL PASS' if ok else 'FAILURES PRESENT'}")
    return ok


def main():
    parser = argparse.ArgumentParser(description="Cartographer autopilot (P5)")
    parser.add_argument("--config", default=None)
    parser.add_argument("--dry-run", action="store_true",
                        help="observe the frame bus + pilot commands and LOG the contact verdict; send NO controls")
    parser.add_argument("--self-test", action="store_true",
                        help="validate the detection LOGIC (synthetic) + playbook + mission expansion (no hardware)")
    parser.add_argument("--explore", action="store_true",
                        help="MAP MODE: execute the frontier plan published by perception_worker on "
                             "TOPIC_PLAN (autonomous exploration), instead of a fixed mission script. "
                             "Arms + takes off automatically, then explores.")
    parser.add_argument("--no-takeoff", action="store_true",
                        help="--explore: skip the arm+takeoff prelude (drone is already airborne)")
    parser.add_argument("--mission", default=None,
                        help=f"mission JSON script (default {os.path.basename(DEFAULT_MISSION)})")
    parser.add_argument("--max-contact-s", type=float, default=None,
                        help="override the mission's SAFETY timeout for until-contact steps (seconds)")
    parser.add_argument("--log", action="store_true",
                        help="write the verdict log (rec_frame-prefixed) + a CSV to OUTPUT/diag/")
    parser.add_argument("--stop-file", default=None,
                        help="--explore: path to a sentinel file; when it appears, exit the loop CLEANLY "
                             "(runs the shutdown that emits the replay map backdrop + closes diag). Lets a "
                             "launcher request a graceful stop of this separate-console process.")
    args = parser.parse_args()
    cfg = load_config(args.config)

    if args.self_test:
        raise SystemExit(0 if run_self_test(cfg) else 1)
    if args.dry_run:
        run_dry(cfg, log=args.log)
    elif args.explore:
        # A stale sentinel from a crashed prior run would stop us instantly — clear it before we start.
        if args.stop_file and os.path.exists(args.stop_file):
            try:
                os.remove(args.stop_file)
            except OSError:
                pass
        stop_event = _FileStopEvent(args.stop_file) if args.stop_file else None
        run_explore(cfg, stop_event=stop_event, log=args.log, no_takeoff=args.no_takeoff)
    else:
        run_mission(cfg, mission_path=args.mission, max_contact_s=args.max_contact_s, log=args.log)


if __name__ == "__main__":
    main()
