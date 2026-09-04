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
import random
import time
from datetime import datetime, timedelta

import cv2
import yaml

import frame_bus
from diag_log import DiagLog, NullLog
from flow_contact_detector import (FlowContactDetector, detector_from_cfg, FlowVerdict,
                                    CMD_UP, CMD_FWD, CMD_BACK, CMD_DOWN)
from flight_playbook import FlightPlaybook, RecipePlayer
from visual_recovery import VisualRecoveryProbe, VisualMatch

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
    "gate_override": False,   # session 30: ALWAYS declared so io_bridge can't leave it stuck from a
                              # previous tick (same guarantee this dict already gives trigger_down/reverse_down)
}


def _full_vector(active: dict, seq: int, now: float, state: str,
                 target_altitude_y=None,
                 state_since_s: float | None = None,
                 slam_hold: dict | None = None,
                 notice: dict | None = None,
                 visrec_lkg: dict | None = None) -> dict:
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
    # ExploreController's live-calibrated altitude-lock hold target (None before the first
    # calibration latches it) -- rides TOPIC_CONTROL so the visualizer can show it next to the
    # live pos_y (which already rides TOPIC_PLAN) without a third bus/port.
    v["target_altitude_y"] = (round(float(target_altitude_y), 4) if target_altitude_y is not None else None)
    # Session 52 (chunk 6): panel telemetry -- how long the controller has been in its CURRENT state,
    # SLAM_HOLD counters/deadline, and the latched last-timeout notice. Always present (never omitted)
    # so the visualizer can render a fixed-shape panel without a KeyError guard per field.
    v["state_since_s"] = round(float(state_since_s), 2) if state_since_s is not None else None
    v["slam_hold"] = slam_hold
    v["notice"] = notice
    # Session 56: F_LKG age-out state -- always present (never omitted), same fixed-panel-shape
    # convention as slam_hold/notice above.
    v["visrec_lkg"] = visrec_lkg
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
                          leg_goal=None, plan_age_s=None, alt=None, visrec=None) -> dict:
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
        # Session 29: the raw ray-hit picture behind the fwd/back/left/right clearance judgment (hits,
        # total rays, fraction, closest/farthest, the min_hit_fraction vote outcome) -> the replay's
        # Clearance tab, so a "ring blocked" call is auditable instead of re-derived by hand.
        "clearance_detail": g("clearance_detail"),
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
        # Session 35 ALT: the visual-recovery probe's phase + last match verdict against F_LKG (None when the
        # feature is off / no probe built) -> the replay's Visual Recovery floating panel.
        "visual_recovery_detail": visrec,
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
        self.ts = None          # flight stamp (YYYYmmdd_HHMMSS) when enabled, else None
        self.diag_dir = None    # OUTPUT/diag when enabled, else None
        self._fsync_failed = False   # session 55: latched after the first fsync() OSError (see fsync())
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
            self.ts, self.diag_dir = ts, d

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

    def fsync(self):
        """Session 55: periodic (NOT per-write) os.fsync of the raw text/timeline handles + the CSV
        sinks, so a hard machine reboot (a GPU-driver TDR bugcheck gave zero shutdown path on flight
        20260902_165340) loses at most one fsync period instead of whatever the OS page cache hadn't
        written back yet. `line()`/`timeline()`/`row()` already flush() every call -- that's sufficient
        against a process kill, not against a reboot. Call from the owning loop on a timer; a failure
        is surfaced once per handle (see DiagLog.fsync) rather than silently dropped."""
        if not self._fsync_failed:
            for f in (self._txt, self._jsonl):
                if f is not None:
                    try:
                        os.fsync(f.fileno())
                    except OSError as exc:
                        self._fsync_failed = True
                        print(f"*** CRITICAL: fsync failed for {f.name} ({exc}) -> periodic fsync "
                              f"DISABLED for the rest of this flight; flush()-only durability "
                              f"continues ***", flush=True)
                        break
        self.csv.fsync()
        self.cmd_csv.fsync()

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
        # Session 45: upper bound on how long the same-goal PICK dedup above may keep suppressing picks. Past
        # this much continuous same-goal re-committing with no hop ever judged, the leg is circling rather than
        # re-orienting, and the pick registers for real -- otherwise a flight where no hop can complete (slow
        # SLAM) starves the goals-DB loop guard forever (flight 20260901_112227: picks stuck at 1 vs. the 3
        # needed). A general robustness duration, NOT a room answer.
        self.goal_dedup_max_hold_s = float(e.get("goal_dedup_max_hold_s", 20.0))
        self._dedup_run_t0 = None            # 'now' the current continuous same-goal dedup run began (None = idle)
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
        # PLAN-STALE (perception publishing, SLAM not TRACKING) -> RECOVERY_REWIND (config-gated, default OFF
        # as of session 31 -- see use_rewind_on_stale below): replay the INVERSE of the recently-flown
        # maneuvers to re-expose the camera to keyframes it already recorded, watching for OK. Otherwise (or
        # once the history is empty/exhausted) -> the FALLBACK sweep.
        self.command_history = collections.deque(maxlen=100)   # maneuvers flown during normal exploration
        self.command_history_s = float(e.get("command_history_s", 12.0))  # rewind horizon (seconds of motion)
        # Session 31 (operator ask): REWIND never once visibly helped recover a stale plan across many real
        # flights -- config-gated off rather than deleted, so it's one edit to bring back if that changes.
        self.use_rewind_on_stale = bool(e.get("use_rewind_on_stale", False))
        # --- VISUAL RECOVERY (session 35 ALT, operator vision: "the live NDI image tells us why tracking
        # dropped and what to do about it"). Two integration points, both reusing EXISTING machinery: (1)
        # the loss-instant snapshot check (`_maybe_loss_snapshot_backoff`, session 34 Idea B) gains a visual
        # clause alongside its existing cached-clearance one — catches the case geometry can't (a wall SLAM
        # never integrated reads "clear" by ray-cast clearance alone, but the live image shows we're nose-to
        # -it); (2) if visual is ALSO inconclusive, an explicit 15° rotational turn-probe (VISUAL_RECOVERY
        # state, `_step_visual_recovery`) runs BEFORE the blind FALLBACK sweep, re-matching the image after
        # each turn step. Config-gated (mirrors `use_rewind_on_stale`'s exact pattern), default OFF —
        # live-fly-untested, per this project's standing convention for a brand-new stale-recovery path.
        # Session 42: integration point (2), the turn-probe hand-off, is PLAN-STALE-only (perception alive,
        # SLAM confused) — a PLAN-LOST/NO-PLAN loss (perception itself silent, e.g. a slow SLAM solve
        # backlog per session 28) never hands off into the probe, only ever the plain hard-hover-hold;
        # integration point (1)'s two BACKOFF reactions are unaffected and still fire for any status.
        self.use_visual_recovery_on_stale = bool(e.get("use_visual_recovery_on_stale", False))
        self.visrec_min_inliers = int(e.get("visrec_min_inliers", 12))          # reuse the project's already-validated SIFT/RANSAC inlier threshold
        self.visrec_planar_inlier_ratio = float(e.get("visrec_planar_inlier_ratio", 0.85))  # inlier fraction to call a match "planar-like" (Step 2b)
        self.visrec_contain_margin_frac = float(e.get("visrec_contain_margin_frac", 0.02))   # slack for the corner-containment test (Step 2a)
        # Session 57: inlier-SPREAD size ratio (live/LKG) -- the direction estimator for PLAN-LOST/NO-PLAN
        # (see `_step_lost_recovery`). Configurable, not yet consumed by any decision in this chunk.
        self.visrec_size_ratio_hi = float(e.get("visrec_size_ratio_hi", 1.25))
        self.visrec_size_ratio_lo = float(e.get("visrec_size_ratio_lo", 0.80))
        self.visrec_size_min_inliers = int(e.get("visrec_size_min_inliers", 20))
        self.visrec_close_scale = float(e.get("visrec_close_scale", 1.15))      # homography linear scale >= this => zoomed-in => closer (Step 2c BACKOFF)
        self.visrec_turn_step_deg = float(e.get("visrec_turn_step_deg", 15.0))  # the probe's discrete rotation step (operator's exact value; independent of FALLBACK's own recovery_turn_step_deg)
        self.visrec_max_rotation_deg = float(e.get("visrec_max_rotation_deg", 720.0))  # cumulative probe budget before "exhausted" -> FALLBACK
        self.visrec_wait_recover_s = float(e.get("visrec_wait_recover_s", 30.0))       # bounded wait for a SLAM re-anchor after a farther re-match
        self._visrec_phase = None           # None | "TURN" | "MATCH" | "WAIT_RECOVER"
        self._visrec_phase_t0 = None
        self._visrec_cum_deg = 0.0          # cumulative commanded turn this probe episode (exhaustion criterion)
        self._visrec_wait_t0 = None         # WAIT_RECOVER entry time (visrec_wait_recover_s cap)
        # Session 49: the operator-visible LKG debug window (F_LKG | LIVE + drawn inliers) -- what the CV
        # probe is matching against, live. Default OFF, mirrors use_visual_recovery_on_stale's convention.
        self.visrec_match_min_interval_s = float(e.get("visrec_match_min_interval_s", 0.5))
        self.visrec_debug_window = bool(e.get("visrec_debug_window", False))
        self.visrec_save_max = int(e.get("visrec_save_max", 200))
        # Session 52: F_LKG-by-frame-identity ring length (frame COUNT, not a duration) -- see the
        # config.yaml comment for the latency/memory rationale. 0 = legacy live-frame behaviour.
        self.visrec_lkg_ring_len = int(e.get("visrec_lkg_ring_len", 160))
        # Session 56: rate-limit for the F_LKG age-out warning below (was a one-shot `_lkg_ring_warned`
        # flag, so a defect that ran for 31 of 34 minutes logged exactly once).
        self.visrec_lkg_ageout_log_interval_s = float(e.get("visrec_lkg_ageout_log_interval_s", 5.0))
        # NO SILENT FALLBACK (CLAUDE.md rules 2+3): two INDEPENDENT degradation states, each logged CRITICAL
        # once and each carried into the timeline, so the replay shows WHICH half died. A debug window must
        # never kill a flight, but it must never fail quietly either.
        self.visrec_window_failed = False    # imshow/waitKey raised (no display) -> window disabled, saving continues
        self.visrec_save_failed = False      # imwrite/makedirs raised -> saving disabled, window continues
        self.visrec_window_open = False      # Session 58: an LKG canvas is currently SHOWN in the OS window
        # Session 56: F_LKG age-out counter/flag -- mirrors visrec_window_failed/visrec_save_failed exactly.
        # Set when the plan's frame_id has fallen out of the ring (SLAM solve latency exceeded the ring's
        # depth); the PREVIOUS reference is kept rather than substituting the live frame (see run_explore).
        self.visrec_lkg_ageouts = 0          # count of ticks the plan's frame_id was absent from the ring
        self.visrec_lkg_degraded = False     # sticky: True on the first age-out, never cleared
        self.visrec_cap_logged = False       # so the "evidence cap reached" line prints exactly ONCE per flight
        # --- FALLBACK sweep (session 31, replaces session 29's shuffled-direction-queue-with-per-direction-
        # tries-and-opposite-phase search -- operator ask, after live flights showed LOCKING a push direction
        # across several tries produced bad, unpredictable results). Deliberately simple, 4-phase cycle:
        # wait -> turn -> push (a FRESH random direction every single cycle, no per-direction budget or
        # opposite-phase retry) -> wait -> repeat, until the cumulative commanded turn reaches
        # fallback_max_rotation_deg. Forward is a candidate here (unlike normal scouting, which never pushes
        # forward) -- while blind there's no live signal saying the back is any safer than the front.
        self.fallback_initial_wait_s = float(e.get("fallback_initial_wait_s", 20.0))    # step 0: let a transient stale patch clear on its own first
        self.fallback_post_push_wait_s = float(e.get("fallback_post_push_wait_s", 10.0))  # step 3: settle after each push
        self.fallback_max_rotation_deg = float(e.get("fallback_max_rotation_deg", 720.0))  # exhaustion cap (replaces fallback_max_attempts)
        self.fallback_push_fwd_back_s = float(e.get("fallback_push_fwd_back_s", 2.0))   # forward/backward push hold, FULL throttle, includes ramp-up
        self.fallback_push_strafe_s = float(e.get("fallback_push_strafe_s", 0.5))       # left/right push hold, FULL magnitude (joy_horizontal isn't ramped)
        # Session 46 (flight 20260901_124211): after this many blind back-off reflexes against the same
        # obstacle with no confirmed recovery between them, stop repeating a reflex that is demonstrably not
        # working and escalate into this FALLBACK sweep instead (see _blind_contact_backoff). A general
        # robustness COUNT, not a room answer.
        self.blind_contact_escalate_after = int(e.get("blind_contact_escalate_after", 2))
        self._fallback_phase = None         # None | "INITIAL_WAIT" | "TURN" | "PUSH" | "WAIT_POST"
        self._fallback_phase_t0 = None      # 'now' the current phase began
        self._fallback_cum_deg = 0.0        # cumulative commanded turn this episode (exhaustion criterion)
        self._fallback_cycle = 0            # diagnostic: completed turn+push+wait cycles this episode
        self._fallback_push_dirn = None     # the CURRENT push's direction (for the live-contact early-exit)
        # --- SESSION 12 recovery redesign (see plans/strafe-throttle-and-recovery-loop.md D5) ---
        # A flickering SLAM status (PLAN-LOST<->PLAN-STALE) used to RESET recovery every ~3s, so STUCK was
        # unreachable and the rewind never emptied (flight 20260713 frantic loop). Fix: `_recovering` PERSISTS
        # across the flicker; the rewind CONSUMES command_history one maneuver at a time; the give-up counter
        # is reset only on a genuinely trusted recovery. While `_recovering`, appends to command_history are
        # frozen and moving post-relock sets `_history_broken` so the now spatially-stale leftover history is
        # cleared + bypassed straight to FALLBACK (no displaced "ghost path" replay).
        #
        # Session 35 (operator ask, diagnosed off 11 flights with zero step-back events): trust restoration
        # used to require a CONFIRMED >=1u ADVANCE (`recovery_confirm_dist`, now removed) measured from
        # `_recovery_adv_start` -- but that anchor was wiped on every hop boundary (SETTLE/REPLAN/ORIENT are
        # not `_enter()`'s ("ADVANCE","SLAM_HOLD") exemption), so confirmation could only ever be measured
        # WITHIN one `hop_duration_s` (2.0s) -- structurally impossible once SLAM couldn't even deliver two
        # fresh poses that fast. Simplified: `_recovering`/`_history_broken` now clear as soon as a loss
        # recovers to a genuinely SETTLED `OK` (`SLAM_HOLD`'s existing settle-gate -- several consecutive
        # fast, fresh frames -- already runs first regardless of this change), not a further confirmed-motion
        # step on top of it. See the settle-gate-clear branch below for where this actually happens.
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
        self._slam_gate_since = None   # session 56: monotonic floor for the gate's CURRENCY proof.
                                       # None == a solve of a frame captured at/after the floor has landed.
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
        # Session 35 (operator ask): TWO mutually-exclusive strategies for "SLAM is slow but the plan is
        # still OK" -- the classic REWIND step-back above, or a simpler forward escape: after
        # `slam_slow_hop_after_s` of continuous slow-but-OK holding, stop waiting and force one hop toward
        # the CURRENT goal anyway (re-reads it via REPLAN), instead of holding indefinitely. Diagnosed off
        # 11 consecutive flights with zero step-back events (last seen 20260720_123903): step-back's own
        # `not self._recovering` gate was almost always false because of a SEPARATE bug (see the
        # `_recovering` simplification below) -- kept step-back fully intact behind this switch (operator:
        # "I want to eventually throw out that REWIND bullshit... but we might also want to bring it back"),
        # default OFF in favor of the new forward-hop strategy.
        self.use_slam_stepback_on_slow = bool(e.get("use_slam_stepback_on_slow", False))
        self.slam_slow_hop_after_s = float(e.get("slam_slow_hop_after_s", 30.0))
        self.slam_slow_hop_grace_s = float(e.get("slam_slow_hop_grace_s", 8.0))  # one turn + one hop_duration_s, with margin
        self._slam_slow_hop_deadline = None  # 'now' past which a forced hop's SLAM-slow bypass no longer applies
        # PERSISTS across a PLAN-LOST/HOLD_LOST bounce within one bad SLAM patch (mirrors the
        # `_recovering`/FALLBACK-sweep persistence rule) -- a solve slow enough to trip this almost
        # always exceeds `plan_timeout_s` before it finishes, so the FSM bounces HOLD_LOST -> OK -> a FRESH
        # SLAM_HOLD every time; resetting this on every fresh hold entry (the old behavior) meant the
        # escalation could never reach its cap in exactly the scenario it exists to bound (confirmed on the
        # 20260718 flight: #1/3 fired 3x running, `plan_valid` bounced between, never reaching #2 or #3).
        # Reset ONLY on a genuinely trusted recovery (REPLAN / confirming ADVANCE) or a materially NEW leg
        # goal (both in the REPLAN handler) -- NOT on every `_enter_slam_hold`.
        self._slam_stepback_count = 0
        self._slam_hold_start = None      # 'now' when the current SLAM_HOLD began (total-wait logging)
        # Session 53 (flight 20260902_143207): the session-43 forced-hop rescue below reads `_slam_hold_start`,
        # which is re-stamped on EVERY `_enter_slam_hold` -- exactly the hazard the `_slam_stepback_count`
        # comment above already documents ("resetting this on every fresh hold entry meant the escalation
        # could never reach its cap in exactly the scenario it exists to bound"), just never applied to this
        # field. That flight's SLAM ran ~3.1s/frame: too slow to hold a plan valid past `plan_timeout_s`
        # (3.0s), so status flip-flopped OK<->PLAN-LOST every ~3.3s and `_slam_hold_start` was wiped every
        # cycle -- `waited` never got past ~3.0s against the 15.0s bar. 43 bounces, 0 forced hops. This field
        # is the EPISODE clock: stamped ONLY on the first `_enter_slam_hold` of a bad patch (see there), left
        # untouched by every re-entry the bounce causes, so the rescue's `waited` finally accumulates across
        # the oscillation instead of being repeatedly zeroed by it. `_slam_hold_start` itself is UNCHANGED and
        # keeps feeding the "this hold" wait already printed in the settle/rescue log lines -- widening ITS
        # meaning would silently change numbers the operator reads on every flight. Reset at the same
        # boundaries `_slam_stepback_count` uses: the REPLAN handler (trusted recovery / new goal),
        # reset_leg() (autonomy pause/interruption), and SLAM_HOLD's own settle-gate-clear branch (the hold
        # genuinely settled, the episode is over).
        self._slam_hold_episode_t0 = None
        # Reactive wall/backwall response while BLIND (HOLD_LOST / waiting in SLAM_HOLD): the flow contact
        # detector doesn't need SLAM, but nothing read it in those states before this fix (the drone could
        # drift into a wall for 30-40s of a bad SLAM patch with no reaction). Edge-triggered (armed=True
        # means "ready to react to a fresh contact"; disarmed once reacted, re-arms only once contact
        # clears) so a sustained pin doesn't replay back_off every tick.
        self._blind_contact_armed = True
        # Session 46 (flight 20260901_124211): consecutive blind-contact reflexes with NO confirmed recovery
        # between them -- lets the caller notice "this reflex is not working" instead of replaying it forever.
        # Resets ONLY at genuine recovery/reset boundaries (see reset_leg, the SLAM-settle and forced-hop
        # trust-restoration sites, and REPLAN on a materially new goal) -- NEVER on a bare status flip, so the
        # 3s OK/PLAN-LOST oscillation that flight showed cannot silently clear it.
        self._blind_contact_reacts = 0
        self._blind_backoff_resume = None  # the hold state ("HOLD_LOST"/"SLAM_HOLD") to resume after it plays
        # Session 47 (flight 20260901_142738): the POST-BACKOFF SLAM RE-SOLVE GATE. A back-off is only
        # meaningful if SLAM gets to look at where the back-off PUT us; that flight never allowed it. SLAM was
        # solving at 3400-3700ms/frame, so `status` oscillated OK<->PLAN-LOST 19 times, and EVERY flip is a
        # fresh loss edge that re-arms `_loss_snapshot_checked` -> `_maybe_loss_snapshot_backoff` re-fired off
        # the SAME unchanged F_LKG evidence (the drone had barely moved, so the match was identical: 538 ->
        # 577 -> 479 -> 704 -> 444 -> 452 -> 412 inliers, `contained=True` every time). 7 back-offs, two of
        # them 3s apart, none of them ever judged. `_backoff_resolve_since` holds the cap_ts FLOOR a fresh
        # SLAM solve must clear before the loss-instant trigger may fire again (cleared in `_update_slam`);
        # `_backoff_resolve_t0` is the same instant on the wall clock, bounding the wait by
        # `backoff_resolve_budget_s` so a capture stream that never carries a cap_ts cannot hang us forever.
        self._backoff_resolve_since = None
        self._backoff_resolve_t0 = None
        # Session 48: when the CURRENT loss episode began (stamped at the fresh-loss edge, cleared by a
        # genuine OK), and whether its "deferring" notice has already been printed. A slow-but-alive SLAM
        # solving every ~3.5s is NOT lost -- each genuine OK ends the episode and the next loss re-stamps the
        # window -- so this can never accumulate across the OK/PLAN-LOST oscillation into a spurious reaction.
        self._loss_episode_t0 = None
        self._loss_grace_noticed = False
        self._lost_hold_noticed = False    # session 57: one-shot for the "LKG is closer -> holding" notice
        # Session 52 (chunk 3): mirrors `_loss_grace_noticed` but for the POST-BACKOFF RE-SOLVE GATE
        # (`_backoff_resolve_since`) instead of the loss-recovery grace -- stops the "SUPPRESSED" notice
        # repeating every tick while the gate holds the one-shot deferred (armed, not spent).
        self._backoff_gate_noticed = False
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
        # BACKOFF phase-timer (session 30, replaces the old fixed 0.3s/0.2-throttled recipe): a manual
        # experiment (full throttle -> release trigger -> immediately hold reverse) found it takes ~2s of
        # held reverse to get the right backoff effect -- io_bridge's own ramp math only accounts for a
        # fraction of that (the rest is very likely Unity's own physics/momentum once thrust reaches the
        # sim, invisible from this side of the socket). General platform CONTROL-DYNAMICS characteristics
        # (learned response time + magnitude), not a room-specific value.
        self.backoff_hold_s = float(e.get("backoff_hold_s", 2.0))          # hold full reverse this long (clock starts at BACKOFF entry, includes the ramp-up)
        self.backoff_release_s = float(e.get("backoff_release_s", 0.2))    # then wait this long (open-loop) for the reverse ramp-down to finish before SETTLE
        self.backoff_reverse_mag = float(e.get("backoff_reverse_mag", 1.0))  # BACKOFF's own reverse target -- independent of reverse_throttle (every OTHER reverse site)
        # Session 47: upper bound on the post-backoff SLAM re-solve wait (see `_backoff_resolve_since`). A
        # general robustness duration, NOT a room answer: it only decides how long we hold still waiting for
        # a capture-timestamped frame before giving up on confirming and saying so LOUDLY.
        self.backoff_resolve_budget_s = float(e.get("backoff_resolve_budget_s", 12.0))
        # Session 48: how long a loss must OUTLIVE before it earns a physical reaction (see
        # `_maybe_loss_snapshot_backoff`'s grace gate). A general robustness duration measured from the fleet's
        # own recovery statistics, NOT a room answer -- it encodes how long THIS PLATFORM's SLAM typically
        # takes to re-lock while the drone holds still, which holds in any room.
        self.loss_backoff_grace_s = float(e.get("loss_backoff_grace_s", 12.0))
        self._backoff_t0 = None      # 'now' BACKOFF was entered (phase-timer origin)
        # Altitude lock: hold the LIVE-cached mapping height during long ADVANCE pushes (forward pitch sinks
        # the drone into inner walls). target_altitude_y is cached live from the first valid post-prelude
        # pose (self-calibrating, NOT a baked value); world frame is +Y DOWN so a sink = LARGER y.
        self.altitude_lock = bool(e.get("altitude_lock", True))
        self.alt_drift_floor = float(e.get("alt_drift_floor", 0.3))
        self.target_altitude_y = None        # cached lazily; PERSISTS across reset_leg (flight-level hold target)
        # Operator testing override (0 = disabled): forces the desired flying height to a fixed value instead
        # of the live CALIB_VERIFY measurement, so a test flight can hold a known/repeatable height regardless
        # of where DESCEND happens to settle. World frame is +Y DOWN, so a flying height is a NEGATIVE value
        # (e.g. -1.9). Explicit, visible, opt-in (default 0 = today's full self-calibration; NOT autonomous
        # decision-making) -- the ceiling itself (_ceiling_y, below) is still measured LIVE regardless, so
        # TRIM's sag/high band still tracks the real room.
        self.desired_height_override_y = float(e.get("desired_height_override_y", 0.0))
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
        # Session 52 (chunk 6): FLIGHT-level SLAM_HOLD observability counters + the latched last-timeout notice
        # for the visualizer panel. Deliberately NOT reset in reset_leg (a manual takeover must not zero the
        # flight-so-far picture) -- see Contract 1.2 in PROGRESS.md / the session-52 spec.
        self._slam_hold_entries = 0          # SLAM_HOLD entries this flight (re-entries within one hold don't recount)
        self._slam_hold_total_s = 0.0        # cumulative seconds spent in SLAM_HOLD this flight
        self.last_timeout = None             # {"kind","text","t"} of the most recent timeout/forced-escape, or None
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
        # ORIENT_HOME: converge on the take-off heading to within a real tolerance (NOT "quantized turn rounds
        # to 0", which knife-edges into a side-to-side ping-pong whenever the residual sits near half a turn
        # step — see the 20260720 diagnosis) using the ACTUAL bearing error each turn (still clamped to one
        # turn_step_deg per turn for the same SLAM-survives-a-small-turn reason as everywhere else).
        self.orient_home_tol_deg = float(e.get("orient_home_tol_deg", 5.0))    # "facing the take-off heading" tolerance
        self.orient_home_max_s = float(e.get("orient_home_max_s", 60.0))  # SAFETY cap (then refine/dock HERE; logged)
        # HOME_REFINE: after ORIENT_HOME, tighten the resting POSITION against the true origin using fixed,
        # full-throttle push pulses (not a continuous ADVANCE) picked by body-frame quadrant each cycle, settled
        # between pushes. General platform/robustness params, NOT a room answer.
        self.home_fine_reach_dist = float(e.get("home_fine_reach_dist", 0.15))   # "good enough" origin radius
        self.home_refine_fwd_s = float(e.get("home_refine_fwd_s", 0.32))    # forward/backward pulse hold (ramps as usual)
        self.home_refine_strafe_s = float(e.get("home_refine_strafe_s", 0.16))  # left/right pulse hold (never ramped)
        self.home_refine_max_s = float(e.get("home_refine_max_s", 45.0))    # SAFETY cap (then dock HERE; logged)
        self.postlude_recover_budget_s = float(e.get("postlude_recover_budget_s", 30.0))  # SAFETY: total wall-clock
        #        budget for POSTLUDE_LOST_HOLD to demand a full clean streak before relaxing the recovery gate (still
        #        only ever resumes on a status=="OK" tick -- never blind; see _step_postlude_lost)
        self.dock_pulse_s = float(e.get("dock_pulse_s", self.ascend_micro_pulse_s))   # Phase-1 DOWN micro-pulse length
        # Phase-1 rest after each DOWN pulse is a REAL settle-gate (settle_fresh_frames), not a fixed timer —
        # the old dock_rest_s timer read whatever pose happened to be in `plan` when it expired, the same
        # stale-frame gap session 24's settle-gate rewrite fixed everywhere else.
        self.dock_max_s = float(e.get("dock_max_s", 20.0))          # SAFETY cap on the descent (then proceed; logged)
        self.floor_standoff_nudge = float(e.get("floor_standoff_nudge", 0.5))  # LOW_STANDOFF up-nudge duration (s)
        # --- GRADUAL HEIGHT TRIM (session 14, RESTORED session 21): a fine PITCH-aim + forward climb BETWEEN
        # calibrations. (Session 17 deleted it as "a self-inflicted sag"; live flights proved the sag is real.)
        # Session 40: a single short joy_vertical pulse (mirrors DOCK_FLOOR's already-proven pulse+settle-gate
        # primitive) straight into the existing WAIT phase -- no forward/lateral room needed at all, unlike the
        # old pitch-aim+forward-push+ring-gate mechanism it replaced.
        # Session 44 (operator's explicit instruction): the trigger is now TWO HARDCODED absolute SLAM pos_y
        # thresholds, not the session-22 live-calibrated ceiling/desired/delta band. THIS IS A DELIBERATE,
        # OPERATOR-APPROVED EXCEPTION to CLAUDE.md's "NO MANUAL-FLIGHT DATA LEAKAGE" standing rule -- these two
        # numbers ARE this room's/this flight's measured answer, baked in on purpose for a fast manual-flight
        # test on the `all-bets-are-off` branch, not derived from any live signal. They will NOT generalize to
        # another room, and may not even hold across a different flight in the SAME room if SLAM's arbitrary
        # monocular scale drifts. Not config-tunable by design (the operator asked for a hardcode, not a knob)
        # -- see plans/session44-hardcoded-height-trim-thresholds.md.
        self.trim_enable = bool(e.get("trim_enable", True))
        self.trim_sag_trigger_y = -1.75   # pos_y >= this -> too LOW (sagged) -> TRIM UP. HARDCODED, see above.
        self.trim_high_trigger_y = -2.10  # pos_y <= this -> too HIGH (near ceiling) -> TRIM DOWN. HARDCODED.
        self.trim_pulse_s = float(e.get("trim_pulse_s", 0.16))   # full-magnitude joy_vertical pulse (unramped)
        self.trim_settle_s = float(e.get("trim_settle_s", 1.0))
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
        self._backoff_t0 = None
        self._fallback_retreat_forward = None
        self._reset_fallback_sweep()
        self._blind_contact_reacts = 0    # session 46: a manual takeover clears the wedge-reflex count too
        self._backoff_resolve_since = None   # session 47: and clears any pending post-backoff re-solve gate
        self._backoff_resolve_t0 = None
        self._loss_episode_t0 = None         # session 48: and any in-flight loss-recovery grace window
        self._loss_grace_noticed = False
        self._lost_hold_noticed = False      # session 57: and any in-flight LKG-closer hold notice
        self._backoff_gate_noticed = False   # session 52: and any in-flight post-backoff gate suppression
        self._reset_visual_recovery()
        # Session-12 recovery flags. A manual takeover (the only caller of reset_leg) invalidates any in-flight
        # recovery, so clear them here; DURING a flight they persist across the PLAN-LOST/PLAN-STALE flicker.
        self._recovering = False       # True from the first PLAN-STALE of a loss until a confirming ADVANCE (>=1u)
        self._history_broken = False   # True once a re-locked-but-unconfirmed drone moves -> leftover history is stale
        # Session 34: proactive clearance checks that don't wait for ADVANCE to re-check. `_last_good_pos`/
        # `_last_good_clearance` cache the most recent VALID plan's position/forward-clearance (refreshed every
        # tick in step()); `_was_lost`/`_loss_snapshot_checked` gate the one-shot check fired at the instant a
        # loss episode begins (Idea B) -- see `_maybe_loss_snapshot_backoff`.
        self._last_good_pos = None
        self._last_good_clearance = None
        self._last_good_t = None
        self._was_lost = False
        self._loss_snapshot_checked = False
        # Session 52 (chunk 4): arms the 15° probe's LATE/in-flight entry (`_maybe_enter_visual_probe`,
        # polled from `_step_stale`). Set True at the fresh-loss edge below; spent by `_enter_visual_recovery`
        # (either entry path) and by `_arm_loss_backoff` (a physical back-off, not a turn-probe, already
        # answered this episode); cleared by `_reset_visual_recovery` so the next fresh loss re-arms.
        self._visrec_probe_armed = False
        self._rec_settling = False     # not mid an inter-action recovery settle
        self._slam_resume = None    # SLAM streak/latest persist (health is flight-level); only the pending resume clears
        self._slam_stepback_count = 0   # per-hold step-back counter + timer clear on interruption
        self._slam_hold_start = None
        self._slam_hold_episode_t0 = None   # session 53: autonomy pause/leg interruption ends the episode too
        # Session 56: safe because None only reads as "satisfied" while _settle_gate_t0 is also None (the
        # dwell already fails then), and every real gate use calls _settle_gate_begin first.
        self._slam_gate_since = None
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
        self._orient_home_t0 = None     # ORIENT_HOME entry time (orient_home_max_s cap)
        self._home_refine_phase = None  # None | "PLAN" | "PUSH" | "SETTLE" within HOME_REFINE
        self._home_refine_t0 = None     # HOME_REFINE entry time (home_refine_max_s cap)
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
        self._trim_phase = None         # None | "PULSE" | "WAIT" within TRIM
        self._trim_phase_t0 = None      # entry time of the current TRIM sub-phase
        self._trim_cmd_t0 = None        # 'now' the pulse command issued (WAIT settle-gate origin; same clock as cap_ts)
        self._trim_resume_goal = None   # the committed leg_goal snapshotted on TRIM entry (Trap B: re-aim at it, don't re-pick)
        self._trim_exit_msg = None      # the TRIM-end reason string, stashed across TRIM_RESUME_WAIT for the final event line
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
        """Leave a gradual-height TRIM (session 14, restored session 21, GATED session 28). Does NOT resolve
        immediately: stashes the exit reason and opens a settle-gate wait (`TRIM_RESUME_WAIT`) so the eventual
        re-aim (or fallback) is computed off a PROVABLY FRESH post-TRIM frame, never whatever pose happened to
        be sitting in `plan` the instant TRIM ended (the 20260720 bug: an at-entry ring-blocked abort re-aimed
        INSTANTLY at a goal that had, moments earlier, been permanently 2-bump-blacklisted, and rode that stale
        commitment for the rest of the flight because no REPLAN ever got another chance — see
        `_trim_resolve_resume` for the other half of the fix, the blacklist re-check). Sets the next state via
        `_enter` and RETURNS the event string (the TRIM handler falls through to the common return)."""
        self._trimming = False
        self._trim_phase = None
        self._player = None
        self._trim_exit_msg = msg
        self._settle_gate_begin(now)
        self._enter("TRIM_RESUME_WAIT", now)
        return f"{msg} -> wait for a fresh post-trim frame before resuming"

    def _trim_resolve_resume(self, now, plan):
        """Resolve a `TRIM_RESUME_WAIT` once its settle-gate clears (see `_trim_exit`). Trap B: RESTORE the
        committed goal snapshotted on TRIM entry and re-aim (ORIENT) at it — never a fresh planner pick, so a
        routine TRIM can't pollute goal commitment or the goals-DB. BUT first re-validate the preserved goal
        against the NOW-current blacklist (`plan.get("blacklist")`/`blacklist_permanent`, the same live data
        `_timeline_goals` already reads) — a goal that died (2-bump/stall/loop) while TRIM was interrupting the
        leg must NOT be blindly restored. Falls back to SETTLE->REPLAN (the SAME convergence a genuinely new
        leg uses — its own pick-dedup already suppresses a redundant pick when the goal turns out unchanged,
        so this preserves Trap B's intent for the common case) when the preserved goal is dead, unset, or the
        pose is unavailable.

        Session 56: takes priority OVER Trap B -- if `_slam_resume` is still set, this TRIM interrupted an
        as-yet-UNRESOLVED SLAM_HOLD episode (see the trigger block's comment at the SLAM_HOLD branch of
        `_TRIM_TRIGGER_STATES`). A height trim is not evidence SLAM is healthy again, so re-aiming ORIENT
        at the preserved goal here would be exactly the silent trust restoration sessions 35/43 deliberately
        gate behind SLAM_HOLD's OWN settle-gate/trust check (autopilot.py ~3175). Instead this re-enters
        SLAM_HOLD honouring `_slam_resume` as the target IT will resume once genuinely settled -- the
        preserved Trap-B goal (`g` below) is simply discarded for this cycle (`self.leg_goal` itself was
        never touched by TRIM, so it is unaffected). Only when `_slam_resume is None` (the plain
        SETTLE/ADVANCE trigger case) does Trap B run at all. Returns the event string."""
        msg = self._trim_exit_msg or "TRIM done"
        self._trim_exit_msg = None
        g = self._trim_resume_goal
        self._trim_resume_goal = None
        if self._slam_resume is not None:
            resume = self._slam_resume
            _, _, why = self._enter_slam_hold(
                resume, now, f"{msg} -> resume SLAM_HOLD ({resume} pending, episode still open)")
            return why
        pos, hd = plan.get("pos"), plan.get("heading_deg")
        dead = self._goal_is_blacklisted(plan, g)   # session 58: shared predicate (was inline here)
        if g is not None and not dead and pos is not None and hd is not None:
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
        reason = "preserved goal was blacklisted while trimming" if dead else "no committed goal / pose unavailable"
        return f"{msg} -> settle -> replan ({reason})"

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

    def _step_stale(self, now, plan, wall_contact, backwall_contact=False, visual_match=None):
        """PLAN-STALE (SLAM not TRACKING, perception publishing). If `use_rewind_on_stale` is True: a
        CONSUMING control-space rewind — pop the inverse of the recently-flown maneuvers ONE at a time
        (watching for OK at the step() top), draining the history to empty, then the FALLBACK sweep (session
        31 — see `_enter_fallback_sweep`/`_step_fallback_sweep`) -> STUCK. Default (session 31, operator ask
        after REWIND never once visibly helped recover a stale plan across many real flights): skip straight
        to the FALLBACK sweep. `_recovering` + the give-up state PERSIST across any PLAN-LOST/HOLD_LOST
        flicker (fixes the flight-20260713 frantic loop). If the drone already MOVED on an unconfirmed
        re-lock (`_history_broken`), the leftover history is spatially stale -> clear it and go straight to
        the sweep (no displaced ghost-path replay) regardless of the REWIND flag. `wall_contact`/
        `backwall_contact` are the SLAM-independent flow detectors, threaded through so a FALLBACK push can
        be cut short on a live contact (see `_step_fallback_sweep`)."""
        ring = plan.get("clearance_ring")
        if ring:
            self._last_ring = ring          # still used by CALIB_ESCAPE's own ring-picked push
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
                # DRAIN the queue (consuming rewind): pop + play the next inverse, else the fallback sweep.
                steps = self._pop_stepback()
                if steps is not None:
                    self._player = RecipePlayer(steps, name="rewind")
                    return {}, "REWIND", (f"settled between rewind steps{cap} -> next inverse "
                                          f"[{len(self.command_history)} left]")
                self._player = None
                return self._enter_fallback_sweep(now, f"rewind drained (history empty){cap} -> FALLBACK sweep")
            active, done = self._player.fields(now)
            if not done:
                return active, "REWIND", None
            # This inverse maneuver finished -> SETTLE (let SLAM re-lock) BEFORE popping the next one.
            self._rec_settling = True
            self._settle_begin(now)
            return {}, "REWIND", "rewind step done -> settle (let SLAM breathe / re-lock) before the next inverse"
        if st == "FALLBACK":
            return self._step_fallback_sweep(now, wall_contact, backwall_contact)
        if st == "VISUAL_RECOVERY":
            return self._step_visual_recovery(now, plan, visual_match)
        # ---- fresh entry (first PLAN-STALE of this loss) OR re-entry after a HOLD_LOST flicker ----
        if not self._recovering:
            # The FIRST PLAN-STALE of this loss episode arms recovery. The flags PERSIST until a confirming
            # ADVANCE — never reset by a bare OK or by a LOST/STALE flicker (that was the loop bug).
            self._recovering = True
            self._history_broken = False
        if not self._loss_snapshot_checked:
            snap = self._maybe_loss_snapshot_backoff(plan, now, visual_match, status="PLAN-STALE")
            if snap is not None:
                return snap
        if not self._ever_tracked:
            # STARTUP: SLAM has never TRACKED yet (the prelude finishes on the FLOW ceiling detector, not on
            # SLAM). Don't spin a blind fallback into an unmapped room — HOLD and wait for SLAM to initialize.
            # The step() top snaps WARMUP -> SLAM_HOLD -> SETTLE -> REPLAN when OK returns. This guard applies
            # regardless of use_rewind_on_stale -- REWIND being off doesn't mean "sweep blindly at startup".
            self._enter("WARMUP", now)
            return {}, "WARMUP", "PLAN-STALE at startup (SLAM still initializing) -> HOLD (no blind sweep)"
        if not self.use_rewind_on_stale:
            # Session 52 (chunk 4): the loss-instant one-shot is very often already spent by the time
            # PLAN-STALE arrives (see `_maybe_enter_visual_probe`'s docstring) -- give the probe a second,
            # LATE shot before falling through to the blind sweep.
            probe = self._maybe_enter_visual_probe(now)
            if probe is not None:
                return probe
            return self._enter_fallback_sweep(now, "PLAN-STALE -> FALLBACK sweep (REWIND disabled)")
        # Ghost-path guard: a re-lock that already MOVED (unconfirmed) decoupled the leftover history from the
        # true pose -> clear it and BYPASS REWIND straight to the safe FALLBACK sweep.
        if self._history_broken:
            if self.command_history:
                self.command_history.clear()
                return self._enter_fallback_sweep(now, "secondary loss after an unconfirmed re-aim -> stale "
                                                       "history cleared -> FALLBACK sweep (no ghost path)")
            return self._enter_fallback_sweep(now, "secondary loss after an unconfirmed re-aim (history "
                                                   "already drained) -> FALLBACK sweep")
        # CONSUMING rewind: pop the newest maneuver's inverse and play it; the step() top watches for OK.
        steps = self._pop_stepback()
        if steps is not None:
            self._player = RecipePlayer(steps, name="rewind")
            self._enter("REWIND", now)
            return {}, "REWIND", ("PLAN-STALE -> RECOVERY_REWIND (consuming): retracing recent maneuvers one "
                                  f"at a time to re-expose keyframes [{len(self.command_history)} left after this pop]")
        return self._enter_fallback_sweep(now, "PLAN-STALE + EMPTY command history (post-collision?) -> "
                                               "FALLBACK sweep")

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
                self.note_timeout("CALIB_GATE", ev, now, loud=False)
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

    def _step_backoff(self, now, lost, backwall_contact=False):
        """Per-tick BACKOFF phase-timer (session 30 body, EXTRACTED session 46 so a BACKOFF in flight can own
        EVERY status, matching BLIND_BACKOFF/CALIB_ESCAPE above). Flight 20260901_124211: all 6 loss-instant
        BACKOFF entries emitted `fields={}` and were wiped by the PLAN-LOST router one tick later, before the
        phase-timer's own thrust (below) ever got to run on a later tick -- zero reverse was ever commanded.
        `lost` selects the completion target: routing to SETTLE while still PLAN-LOST would be intercepted by
        the very router this fix escapes and wiped again, re-creating the bug one layer down.
        HOLD: trigger cut immediately, full-magnitude reverse held for backoff_hold_s (the underlying analog
        ramps stay untouched -- trigger decays over its normal ~10 ticks, reverse climbs over its normal ~20
        ticks to backoff_reverse_mag, both while the gates already read the new state via gate_override).
        RELEASE: reverse released immediately; wait backoff_release_s (open-loop) for the reverse analog to
        finish decaying before declaring BACKOFF done."""
        if self._backoff_t0 is None:      # impossible-state guard -- LOUD, never silent (CLAUDE.md)
            nxt = "HOLD_LOST" if lost else "SETTLE"
            self._enter(nxt, now)
            return {}, nxt, f"BACKOFF entered with no _backoff_t0 (bug) -> {nxt}"
        elapsed = now - self._backoff_t0
        if elapsed < self.backoff_hold_s:
            # Session 47: a wall BEHIND us ends the push. Flight 20260901_142738 streamed 31 BACKWALL-WATCH
            # verdicts from inside BACKOFF -- `signal` pinned at +-0.02, `ratio=0.00`, while commanding FULL
            # reverse: the camera saying "you are not moving, there is something behind you" -- and this timer
            # ground the full backoff_hold_s into it anyway, 7 times, because `_step_backoff` was never handed
            # the flag its own callers already had. Skip straight to RELEASE (keeping its full ramp-down
            # window) rather than aborting outright, so the reverse analog still decays before SETTLE.
            if backwall_contact:
                self._backoff_t0 = now - self.backoff_hold_s
                return ({"trigger": 0.0, "reverse": 0.0, "gate_override": True}, "BACKOFF",
                        "BACKOFF: flow BACKWALL contact while reversing -> stop pushing into the wall behind "
                        "us, release early")
            return {"trigger": 0.0, "reverse": self.backoff_reverse_mag, "gate_override": True}, "BACKOFF", None
        if elapsed < self.backoff_hold_s + self.backoff_release_s:
            return {"trigger": 0.0, "reverse": 0.0, "gate_override": True}, "BACKOFF", None
        self._backoff_t0 = None
        # Session 47: ARM the post-backoff SLAM re-solve gate. Until SLAM solves a frame CAPTURED at/after
        # this instant, the loss-instant trigger must not fire another back-off -- that is the whole point of
        # backing off, and flight 20260901_142738 never once granted it (the next PLAN-LOST flip, ~1s later,
        # re-fired off pre-backoff evidence). Both stamps are the same monotonic clock cap_ts uses.
        self._backoff_resolve_since = now
        self._backoff_resolve_t0 = now
        nxt = "HOLD_LOST" if lost else "SETTLE"
        self._enter(nxt, now)
        return {}, nxt, ("backed off -> hold (still blind); waiting for SLAM to solve a frame captured after "
                         "the back-off before any further loss-instant back-off" if lost else
                         "backed off -> settle; waiting for SLAM to solve a frame captured after the back-off "
                         "before any further loss-instant back-off")

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
            elif resume == "HOME_REFINE":
                self._home_refine_phase = "PLAN"
            self._postlude_resume = None
            self._postlude_t0 = None
            self._enter(resume, now)
            tag = " (RECOVERY BUDGET EXHAUSTED — relaxed streak requirement, VISIBLE)" if budget_exhausted else ""
            return {}, resume, (f"postlude recovered ({self._slam_fast_streak} fresh frames + plan OK){tag} "
                                 f"-> resume {resume}")
        return {}, "POSTLUDE_LOST_HOLD", None          # holding; wait for the SLAM pulse + plan OK

    def _reset_fallback_sweep(self):
        """Clear the FALLBACK sweep's phase-timer state (session 31). Called everywhere a fresh loss
        episode is armed, a recovery is confirmed, or a manual takeover resets the leg — so a NEW blind
        episode always starts a fresh 4-phase cycle rather than resuming mid-sweep from a stale prior
        episode."""
        self._fallback_phase = None
        self._fallback_phase_t0 = None
        self._fallback_cum_deg = 0.0
        self._fallback_cycle = 0
        self._fallback_push_dirn = None

    def _reset_visual_recovery(self):
        """Clear VISUAL_RECOVERY's phase-timer state (session 35 ALT; mirrors `_reset_fallback_sweep`).
        Called wherever a fresh loss episode is armed, a recovery is confirmed, or a manual takeover resets
        the leg — so a NEW blind episode always starts a fresh 15° probe rather than resuming mid-probe
        from a stale prior episode."""
        self._visrec_phase = None
        self._visrec_phase_t0 = None
        self._visrec_cum_deg = 0.0
        self._visrec_wait_t0 = None
        self._visrec_probe_armed = False   # session 52 (chunk 4): a confirmed recovery ends the episode

    def wants_visual_match(self, now=None, status=None):
        """Session 51: True only when THIS tick can actually CONSUME a visual match. There are exactly
        three consumers, each precisely guarded:
          • `_maybe_loss_snapshot_backoff` -- the loss-instant one-shot, dispatched from BOTH of its call
            sites under `if not self._loss_snapshot_checked:` (the PLAN-LOST branch in `step()` and the
            fresh-entry branch in `_step_stale`). Once the one-shot is SPENT nothing reads a match again
            for the rest of the episode.
          • `_step_visual_recovery`'s MATCH phase -- one match per turn cycle, and it must be the FRESH
            post-turn view.
          • Session 57/58: `_step_lost_recovery`, once a PLAN-LOST/NO-PLAN episode has outlived
            `loss_backoff_grace_s`. Session 57 added this clause but placed it BEHIND the ticket check
            above, so on the PLAN-LOST/NO-PLAN path the ticket clause (re-armed False at every loss edge,
            see `self._loss_snapshot_checked = False`) always fired first and returned True immediately --
            the grace clause below it was unreachable dead code. Measured in flight
            `20260903_223345_autopilot.log` (MISSION CONTEXT finding 1 in plans/session58-spec.md):
            233 of 253 matches (92%) ran INSIDE a grace window no consumer could read, a full
            SIFT + BFMatcher + RANSAC every 0.5s on the CPU while SLAM fights to relocalize. Session 58
            fixes this by checking `status` FIRST on this path and never consulting the ticket there at
            all -- PLAN-LOST/NO-PLAN now always evaluates the grace directly, so it looks exactly once the
            episode has matured, never before.
        Everything else that touches `visual_match` (the timeline record, the debug canvas) is
        observability and already handles None. `run_explore` previously matched on EVERY tick of a loss:
        a full SIFT + brute-force BFMatcher + RANSAC at ~32Hz, i.e. ~380 complete matches across session
        48's 12s grace to make ONE decision, all but one discarded.

        CALLER CONTRACT (load-bearing): this is an AND-narrowing of run_explore's existing status gate,
        NEVER a replacement for it. `_loss_snapshot_checked` starts False at construction and is only
        re-armed at a fresh loss edge, so it reads False throughout healthy flight before the first loss --
        using this predicate alone would start running SIFT on every healthy tracking frame, the exact
        opposite of the point. The session 57/58 PLAN-LOST/NO-PLAN clause is an ADDITIONAL narrow
        permission on top of that -- `now`/`status` default to None so every pre-existing caller (and
        self-test) is unaffected."""
        if self._visrec_phase == "MATCH":
            return True
        if status in ("PLAN-LOST", "NO-PLAN"):
            return (now is not None and self._loss_episode_t0 is not None
                    and (now - self._loss_episode_t0) >= self.loss_backoff_grace_s)
        return not self._loss_snapshot_checked

    def _enter_visual_recovery(self, now, event):
        """Route into the 15° visual recovery probe (session 35 ALT; mirrors `_enter_fallback_sweep`). A
        TRUE fresh episode (`_visrec_phase is None`) starts at TURN; a RESUME after a PLAN-LOST/PLAN-STALE
        flicker (which bounces the drone through HOLD_LOST, a separate top-level branch, abandoning
        whatever sub-phase was in progress, then flickers back to PLAN-STALE) just re-enters
        VISUAL_RECOVERY and continues from wherever `_visrec_phase` already was — the exact
        flicker-persistence rule `_fallback_phase`/`_recovering` already use."""
        # Session 52 (chunk 4): spend the latch HERE, in the one place both entry paths (the existing tail
        # hand-off in `_maybe_loss_snapshot_backoff` and the new late entry in `_maybe_enter_visual_probe`)
        # funnel through -- so neither path can double-spend it.
        self._visrec_probe_armed = False
        if self._visrec_phase is None:
            self._visrec_phase = "TURN"
            self._visrec_phase_t0 = now
            self._player = None    # a TRUE fresh episode must not inherit whatever maneuver was mid-flight
                                    # when the loss hit (e.g. an in-progress ORIENT turn) -- the TURN phase
                                    # below builds its OWN 15° turn player from scratch (build-if-None).
        self._enter("VISUAL_RECOVERY", now)
        return {}, "VISUAL_RECOVERY", event

    def _step_visual_recovery(self, now, plan, visual_match):
        """Per-tick VISUAL_RECOVERY dispatch (session 35 ALT), on `self._visrec_phase`:
          TURN -> (visrec_turn_step_deg open-loop turn, then a brief lost-SLAM settle so the match frame is
            clean) -> MATCH -> (consume the freshest `visual_match`: no match -> back to TURN for the next
            15° step; matched + scale >= visrec_close_scale (closer) -> BACKOFF; matched + scale < close
            (farther/same) -> WAIT_RECOVER) -> hover, bounded by visrec_wait_recover_s (the generic
            OK-convergence in `step()`'s _RECOVERY_STATES check breaks this out for free the instant SLAM
            re-anchors, exactly like HOLD_LOST/REWIND/FALLBACK).
        TURN rebuilds `self._player`/settle if `None` (build-if-None, the same pattern REWIND/FALLBACK
        already use) so a resume after a PLAN-LOST/HOLD_LOST flicker cleanly restarts just the CURRENT
        sub-step, not the whole probe; `_visrec_cum_deg` persists across the flicker like `_fallback_cum_deg`.
        """
        phase = self._visrec_phase
        if phase == "TURN":
            if self._rec_settling:
                sdone, capped = self._settle_poll(now, plan, require_fast=False,
                                                  min_frames=self.recovery_settle_frames,
                                                  max_hold_s=self.recovery_settle_max_s)
                if not sdone:
                    return {}, "VISUAL_RECOVERY", None
                self._rec_settling = False
                self._visrec_phase = "MATCH"
                cap = " (settle timed out, no fresh frames)" if capped else ""
                return {}, "VISUAL_RECOVERY", (f"visual probe: turned {self.visrec_turn_step_deg:+.0f}° "
                                               f"(cum {self._visrec_cum_deg:.0f}/{self.visrec_max_rotation_deg:.0f}), "
                                               f"settled{cap} -> matching against F_LKG")
            if self._player is None:
                self._player = self._build_turn(self.visrec_turn_step_deg)
            active, done = self._player.fields(now)
            if not done:
                return active, "VISUAL_RECOVERY", None
            self._player = None
            self._visrec_cum_deg += self.visrec_turn_step_deg
            if self._visrec_cum_deg >= self.visrec_max_rotation_deg:
                return self._enter_fallback_sweep(now, (f"visual turn search exhausted "
                                                        f"({self._visrec_cum_deg:.0f}° with no F_LKG "
                                                        "re-acquire) -> FALLBACK sweep"))
            self._rec_settling = True
            self._settle_begin(now)
            return {}, "VISUAL_RECOVERY", "visual probe: turn done -> settle before matching"
        if phase == "MATCH":
            vm = visual_match
            if vm is None or not vm.matched:
                self._visrec_phase = "TURN"
                return {}, "VISUAL_RECOVERY", "visual probe: no match against F_LKG -> next 15° turn step"
            if vm.scale is not None and vm.scale >= self.visrec_close_scale:
                self._register_bump(dict(plan, pos=self._last_good_pos),
                                    f"visual probe closer @ +{self._visrec_cum_deg:.0f}°")
                if self.backoff_on_standoff:
                    self._player = None
                    self._backoff_t0 = now
                    self._enter("BACKOFF", now)
                    return {}, "BACKOFF", (f"visual probe re-matched F_LKG at +{self._visrec_cum_deg:.0f}° "
                                           f"with scale {vm.scale:.2f} >= {self.visrec_close_scale:.2f} "
                                           "(closer) -> standoff back off (re-arm bump latch) -> settle")
                self._enter("SETTLE", now)
                return {}, "SETTLE", (f"visual probe re-matched F_LKG at +{self._visrec_cum_deg:.0f}° "
                                      f"with scale {vm.scale:.2f} >= {self.visrec_close_scale:.2f} "
                                      "(closer) -> standoff settle")
            self._visrec_phase = "WAIT_RECOVER"
            self._visrec_wait_t0 = now
            scale_txt = f"{vm.scale:.2f}" if vm.scale is not None else "n/a"
            return {}, "VISUAL_RECOVERY", (f"visual probe re-matched F_LKG at +{self._visrec_cum_deg:.0f}° "
                                           f"with scale {scale_txt} < {self.visrec_close_scale:.2f} "
                                           f"(farther/same) -> wait up to {self.visrec_wait_recover_s:.0f}s "
                                           "for SLAM to re-anchor")
        # WAIT_RECOVER: hover; the generic OK-convergence (step()'s _RECOVERY_STATES check) breaks this out
        # the instant status reads OK, same as every other recovery state -- this branch only ever handles
        # the bounded give-up.
        if (now - self._visrec_wait_t0) >= self.visrec_wait_recover_s:
            self._enter("STUCK", now)
            _msg = (f"visual probe re-acquired F_LKG farther at +{self._visrec_cum_deg:.0f}°, "
                    f"waited {self.visrec_wait_recover_s:.0f}s, no SLAM re-anchor -> STUCK")
            self.note_timeout("VISREC_WAIT_TIMEOUT", _msg, now, loud=False)
            return {}, "STUCK", _msg
        return {}, "VISUAL_RECOVERY", None

    def _maybe_enter_visual_probe(self, now):
        """Session 52. Second, LATE entry into the 15° probe, polled from `_step_stale` just before the
        blind FALLBACK sweep. Returns the usual (fields, state, event) triple, or None to let the
        caller fall through to the sweep.

        Exists because the probe's original entry (the tail of `_maybe_loss_snapshot_backoff`) is
        guarded by the back-off one-shot, which 57 of 58 loss episodes on flight 20260901_222552 spent
        on their opening PLAN-LOST tick -- closing the door before the PLAN-STALE that the probe is
        scoped to ever arrived.

        The probe TURNS, so it waits out `loss_backoff_grace_s` exactly like a back-off does (session
        48: 96.9% of held-still losses resolve inside that window); while waiting it HOLDS and returns
        a HOLD_LOST triple, and it is re-polled on every subsequent stale tick.
        """
        if not self.use_visual_recovery_on_stale:
            return None
        if not self._visrec_probe_armed:
            return None
        if self._loss_episode_t0 is not None and now - self._loss_episode_t0 < self.loss_backoff_grace_s:
            waited = now - self._loss_episode_t0
            self._enter("HOLD_LOST", now)
            return {}, "HOLD_LOST", (f"loss-recovery grace {waited:.1f}s/{self.loss_backoff_grace_s:.0f}s -> "
                                     f"holding still before the {self.visrec_turn_step_deg:.0f}deg visual "
                                     "probe (a probe TURNS)")
        return self._enter_visual_recovery(now, "PLAN-STALE with the loss-instant one-shot already spent -> "
                                                 "late entry into the 15deg visual recovery probe")

    def _enter_fallback_sweep(self, now, event):
        """Route into the FALLBACK sweep (session 31: wait -> turn -> push a FRESH random direction -> wait
        -> repeat until `fallback_max_rotation_deg` is reached -> STUCK; see `_step_fallback_sweep` for the
        per-tick phase logic). A TRUE fresh episode (`_fallback_phase is None`, via `_reset_fallback_sweep`)
        starts at INITIAL_WAIT; a RESUME after a PLAN-LOST/PLAN-STALE flicker (which bounces the drone
        through HOLD_LOST — a totally separate top-level branch — abandoning whatever phase was in
        progress, then flickers back) just re-enters FALLBACK and continues from wherever `_fallback_phase`
        already was, so the 720° budget and elapsed phase timer both persist across the flicker exactly
        like `_recovering` already does (the flight-20260713 flicker-persist fix)."""
        if self._fallback_phase is None:
            self._fallback_phase = "INITIAL_WAIT"
            self._fallback_phase_t0 = now
        self._enter("FALLBACK", now)
        return {}, "FALLBACK", event

    def _step_fallback_sweep(self, now, wall_contact, backwall_contact):
        """Per-tick FALLBACK sweep dispatch (session 31), on `self._fallback_phase`:
          INITIAL_WAIT -> (fallback_initial_wait_s) -> TURN -> (recovery_turn_step_deg, unidirectional) ->
          PUSH -> (a FRESH random direction every cycle, full throttle/magnitude, fallback_push_fwd_back_s
          or fallback_push_strafe_s) -> WAIT_POST -> (fallback_post_push_wait_s) -> TURN again, until
          `_fallback_cum_deg >= fallback_max_rotation_deg` -> STUCK.
        Each phase rebuilds `self._player` if it's `None` (build-if-None, the same pattern `REVERSE_PROBE`
        already uses) rather than assuming a player survived a PLAN-LOST/HOLD_LOST interruption intact — a
        resume after a flicker (see `_enter_fallback_sweep`) cleanly restarts just the CURRENT TURN/PUSH
        sub-step, not the whole sweep. Forward IS a candidate (unlike normal scouting, which never pushes
        forward) — while blind there's no live signal saying the back is any safer than the front; a live
        wall/backwall contact matching the in-flight push direction still ends it early (real information,
        just faster) — no equivalent live signal exists for left/right."""
        phase = self._fallback_phase
        elapsed = now - self._fallback_phase_t0
        if phase == "INITIAL_WAIT":
            if elapsed < self.fallback_initial_wait_s:
                return {}, "FALLBACK", None
            self._fallback_phase, self._fallback_phase_t0 = "TURN", now
            return {}, "FALLBACK", (f"FALLBACK: initial {self.fallback_initial_wait_s:.0f}s wait done -> turn")
        if phase == "TURN":
            if self._player is None:
                self._player = self._build_turn(self.recovery_turn_step_deg)
            active, done = self._player.fields(now)
            if not done:
                return active, "FALLBACK", None
            self._player = None
            self._fallback_cum_deg += self.recovery_turn_step_deg
            self._fallback_cycle += 1
            self._fallback_push_dirn = random.choice(["forward", "backward", "left", "right"])
            self._fallback_phase, self._fallback_phase_t0 = "PUSH", now
            return {}, "FALLBACK", (f"FALLBACK cycle {self._fallback_cycle}: turned "
                                    f"{self.recovery_turn_step_deg:+.0f} (cum {self._fallback_cum_deg:.0f}/"
                                    f"{self.fallback_max_rotation_deg:.0f}) -> push {self._fallback_push_dirn}")
        if phase == "PUSH":
            if self._player is None:
                dirn = self._fallback_push_dirn
                move = {
                    "forward": {"trigger": 1.0}, "backward": {"reverse": 1.0},
                    "left": {"joy_horizontal": -1.0}, "right": {"joy_horizontal": 1.0},
                }[dirn]
                dur = (self.fallback_push_fwd_back_s if dirn in ("forward", "backward")
                       else self.fallback_push_strafe_s)
                self._player = RecipePlayer([dict(move, duration_s=dur)], name=f"fallback-push-{dirn}")
            if ((self._fallback_push_dirn == "forward" and wall_contact)
                    or (self._fallback_push_dirn == "backward" and backwall_contact)):
                self._player.i = len(self._player.steps)   # abort the push -- RecipePlayer.done reads True next
            active, done = self._player.fields(now)
            if not done:
                return active, "FALLBACK", None
            self._player = None
            self._fallback_phase, self._fallback_phase_t0 = "WAIT_POST", now
            return {}, "FALLBACK", f"FALLBACK: push {self._fallback_push_dirn} done -> settle"
        # WAIT_POST
        if elapsed < self.fallback_post_push_wait_s:
            return {}, "FALLBACK", None
        if self._fallback_cum_deg >= self.fallback_max_rotation_deg:
            self._enter("STUCK", now)
            _msg = (f"FALLBACK sweep exhausted ({self._fallback_cum_deg:.0f}° over "
                    f"{self._fallback_cycle} cycles) -> STUCK (HOLD; awaiting perception)")
            self.note_timeout("FALLBACK_EXHAUSTED", _msg, now, loud=False)
            return {}, "STUCK", _msg
        self._fallback_phase, self._fallback_phase_t0 = "TURN", now
        return {}, "FALLBACK", None

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
        # Session 47: the ONLY site that opens the post-backoff SLAM re-solve gate. A FRESH solve (this
        # function already deduped on frame_id) whose CAPTURE instant is at/after the back-off's end is
        # positive evidence that SLAM has now looked at where the back-off put us. Deliberately NOT gated on
        # the solve being FAST -- at the 3400-3700ms/frame that flight 20260901_142738 ran at, requiring
        # `< slam_slow_ms` would never open the gate and would just move the stall somewhere new.
        cap = plan.get("cap_ts")
        if self._backoff_resolve_since is not None and cap is not None and cap >= self._backoff_resolve_since:
            self._backoff_resolve_since = None
            self._backoff_resolve_t0 = None
        # Session 56: the ONLY site that clears the settle gate's currency floor. This function has
        # already deduped on frame_id, so this is a genuinely FRESH solve; if its CAPTURE instant is
        # at/after the gate opened, SLAM has now looked at where we actually are. Deliberately NOT
        # gated on the solve being FAST -- at this flight's 1699ms median (p25 1251ms) a `< slam_slow_ms`
        # requirement is the arithmetically-unreachable gate this change exists to remove.
        if self._slam_gate_since is not None and cap is not None and cap >= self._slam_gate_since:
            self._slam_gate_since = None
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

    def _slam_slow_hop_active(self, now):
        """Session 35: True while a forced SLAM-slow-hop's bypass window is still open -- `ORIENT`'s and
        `ADVANCE`'s own `if self._slam_slow:` gates skip re-entering `SLAM_HOLD` for exactly this bounded
        window, so the one forced hop can actually complete despite SLAM staying slow throughout. Self-
        expiring by wall-clock time; ALSO cleared immediately on entering any state besides "ADVANCE"/
        "ORIENT" (see `_enter()`), so it can never leak into an unrelated later leg."""
        return self._slam_slow_hop_deadline is not None and now < self._slam_slow_hop_deadline

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

    def _settle_gate_begin(self, now, *, new_floor=True):
        """Open a settle-gate window AT THE TRUE MOMENT the airframe stops moving. A just-finished ADVANCE/
        PARALLAX_PUSH/etc. (Category A) calls this exactly when motion ends. `_enter_slam_hold` (Category
        B/C) calls this at SLAM_HOLD ENTRY -- the real stationary-start instant, NOT at exit -- so elapsed
        time naturally includes however long the hold lasted; no separate 'credit' bookkeeping is needed.

        `new_floor` (session 56): True (default, every Category-A caller) stamps a FRESH currency floor
        (`_slam_gate_since = now`), demanding a solve of a frame captured after THIS moment. False leaves
        `_slam_gate_since` exactly as it is; used only by `_enter_slam_hold` when re-entering an
        ALREADY-OPEN bad-SLAM episode, so a PLAN-LOST/HOLD_LOST bounce cannot zero the floor it is supposed
        to survive (session 53's `_slam_hold_episode_t0` hazard, one field over). `_settle_gate_t0` is
        stamped unconditionally in both cases -- the physical dwell genuinely restarts on every entry."""
        self._settle_gate_t0 = now
        if new_floor:
            self._slam_gate_since = now

    def _settle_gate_poll(self, now, *, require_fresh=True):
        """True once BOTH gates clear: (1) CURRENCY -- skipped if `require_fresh=False` (the vertical-prelude
        settle-to targets, which keep a plain timer); else `self._slam_gate_since is None`, i.e. SLAM has
        published a solve of a frame CAPTURED at/after the gate opened (session 56 -- replaces the old
        6-consecutive-fast-frames THROUGHPUT proof, which was arithmetically unreachable at this flight's
        1699ms median solve time; a slow solve is a throughput signal, not evidence the pose is wrong, so
        this gate deliberately does NOT require the solve to be fast). This is still exactly the guard that
        closes the 20260719 corner-bounce bug: a `SETTLE` completing on a frame captured BEFORE the
        collision it was supposed to be judging (so REPLAN re-aimed off a pose that never updated
        post-impact) -- the `cap_ts >= floor` rule in `_update_slam` still prevents that, it just no longer
        also demands the solve be fast. (2) PHYSICAL MOTION -- elapsed real time since the gate opened >=
        settle_gate_s. For a resume from a stationary SLAM_HOLD the gate opened at hold ENTRY, so elapsed
        already covers the whole (typically multi-second) hold -- gate 2 clears near-instantly while gate 1
        still independently proves current health. For a fresh post-motion settle both gates run their full
        course."""
        fresh_ok = (not require_fresh) or (self._slam_gate_since is None)
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
        genuinely trusted recovery or a materially new leg goal — see `_hop_start_goal` nearby there.

        Session 53: `_slam_hold_episode_t0` gets the SAME treatment, for the SAME reason -- stamped only
        if it is currently None, so a PLAN-LOST/HOLD_LOST bounce within one bad SLAM patch does not zero
        it on every re-entry (see the field's own comment). `_slam_hold_start` is stamped unconditionally
        as before -- it still means "this particular hold began", feeding the per-hold wait already in the
        settle/rescue log text; `_slam_hold_episode_t0` means "this bad-SLAM-patch episode began" and feeds
        only the forced-hop rescue's bounded wait."""
        _fresh_episode = self._slam_hold_episode_t0 is None      # BEFORE the stamp below
        self._slam_resume = resume
        self._player = None
        self._slam_hold_start = now
        if self._slam_hold_episode_t0 is None:
            self._slam_hold_episode_t0 = now
        # Session 56: a re-entry within the SAME bad-SLAM episode must NOT re-stamp the currency floor --
        # the drone has not moved since the episode opened, so a frame captured after the FIRST entry is
        # still a valid current look. Re-stamping on every entry is exactly the session-53 hazard
        # (`_slam_hold_episode_t0` being zeroed by the very PLAN-LOST/HOLD_LOST bounce it must survive),
        # one field over.
        self._settle_gate_begin(now, new_floor=_fresh_episode)   # session 24: open the shared gate HERE (the
                                           # true stationary-start instant), so a later resume's motion-gate
                                           # elapsed time already covers the whole hold -- no separate credit
                                           # bookkeeping needed
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

        Session 46: counts consecutive reactions in `_blind_contact_reacts` (reset only at genuine
        recovery/reset boundaries — see the reset-site table at each writer — never by this method itself).
        Past `blind_contact_escalate_after` reactions with no confirmed recovery between them, this stops
        repeating a reflex that is demonstrably not working (flight 20260901_124211: reverse authority had
        collapsed to 0.000u) and escalates into the FALLBACK sweep instead — same return shape, just a
        different state/event.

        Deliberately NOT wired into `_register_bump`/the goals-DB — this is a pure safety reflex during a
        blind hold, independent of which goal (if any) is committed; the strike/bump/loop accounting
        stays exactly the mechanism it is today, judged only from ADVANCE."""
        if not (wall_contact or backwall_contact):
            self._blind_contact_armed = True     # re-armed once clear of the wall
            return None
        if not self._blind_contact_armed:
            return None                          # already reacted to this same, still-ongoing contact
        self._blind_contact_armed = False
        self._blind_contact_reacts += 1
        kind = "flow BACKWALL" if backwall_contact and not wall_contact else "flow WALL"
        # Session 46 (flight 20260901_124211): the drone was WEDGED -- commanded reverse produced 0.000u
        # and the flow detector latched BACKWALL -- yet the only response was to replay this same reflex
        # forever. After blind_contact_escalate_after failed reflexes with no confirmed recovery between,
        # stop repeating what demonstrably is not working and escalate to the FALLBACK sweep, which tries
        # a DIFFERENT direction each cycle and is bounded (720 deg -> STUCK).
        if self._blind_contact_reacts > self.blind_contact_escalate_after:
            if self._fallback_phase is None:      # FRESH sweep -> skip the 20s transient-wait; we have
                self._fallback_phase = "TURN"     #   already been stuck far longer, with confirmed contact
                self._fallback_phase_t0 = now
            self._player = None
            return self._enter_fallback_sweep(
                now, f"{kind} contact while blind in {resume_state}, {self._blind_contact_reacts} failed "
                     f"back-offs -> WEDGED: escalate to FALLBACK sweep")
        self._blind_backoff_resume = resume_state
        self._player = self.pb.player("back_off")
        self._enter("BLIND_BACKOFF", now)
        return {}, "BLIND_BACKOFF", (f"{kind} contact while blind in {resume_state} -> back off, "
                                     f"then resume {resume_state}")

    def _enter(self, state, now):
        # Session 52 (chunk 6): FLIGHT-level SLAM_HOLD counters, latched here (before self.state/t_state are
        # overwritten below) so a re-entry into SLAM_HOLD from SLAM_HOLD itself never double-counts, and the
        # dwell time is always measured from the true entry instant (t_state).
        if state == "SLAM_HOLD" and self.state != "SLAM_HOLD":
            self._slam_hold_entries += 1
        elif self.state == "SLAM_HOLD" and state != "SLAM_HOLD":
            self._slam_hold_total_s += max(0.0, now - self.t_state)
        # Ghost-path guard (D5): the moment a re-locked-but-unconfirmed drone enters a SPATIAL state it physically
        # moves (turn/translate) with logging frozen, so the leftover pre-loss command_history no longer maps to
        # the drone's true pose. Mark it broken -> a secondary SLAM drop clears it and jumps to the
        # direction-search FALLBACK sweep instead of replaying a displaced ghost path.
        if self._recovering and state in ("ORIENT", "PARALLAX_PUSH", "ADVANCE"):
            self._history_broken = True
        # Session 35: a forced SLAM-slow-hop's bypass window (see `_slam_slow_hop_deadline` /
        # `_slam_slow_hop_active`) is only ever meant to cover ONE turn + ONE hop. "ADVANCE"/"ORIENT"/
        # "PARALLAX_PUSH" are the only states that legitimately spend it; entering ANYTHING else -- a physical
        # guard cutting the hop short into BACKOFF, a clean hop-done into SETTLE, a fresh SLAM_HOLD, HOLD_LOST,
        # whatever -- ends the grace immediately rather than letting it linger (time-based) into an unrelated
        # LATER leg.
        # Session 45 (root cause of flight 20260901_112227's 221s stall): "PARALLAX_PUSH" was MISSING here, so
        # a forced hop whose leg needed a parallax scout (any off-axis goal) had its own grace window wiped the
        # instant it entered the push -- and the push's slow-check then bounced it straight back to SLAM_HOLD
        # ~30ms later. Under chronically slow SLAM that made the whole session-35/43 rescue a no-op: the drone
        # turned +30 deg every ~25s and NEVER translated. A parallax push IS the hop for an off-axis leg, so it
        # spends the window exactly like ADVANCE does.
        if state not in ("ADVANCE", "ORIENT", "PARALLAX_PUSH"):
            self._slam_slow_hop_deadline = None
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

    def _goal_is_blacklisted(self, plan, goal):
        """True when `goal` sits inside a PERMANENTLY blacklisted region of the live plan.

        Session 58: lifted verbatim out of `_trim_resolve_resume`'s inline `bl`/`perm`/`dead` block so
        `_register_bump` and the SLAM_HOLD settle-resume path can share the SAME dead-goal predicate --
        the 22:40:25.165 trace showed a bump pulse fired and a standoff back-off ran against a goal the
        planner had ALREADY retired PERMANENT 1.0s earlier, because neither path re-checked the live
        blacklist the way `_trim_resolve_resume` already did for its own resume goal."""
        if goal is None:
            return False
        bl = plan.get("blacklist") or []
        perm = plan.get("blacklist_permanent") or []
        return any(
            (bool(perm[i]) if i < len(perm) else False) and self._dist(goal, pt) <= self.goal_area_radius
            for i, pt in enumerate(bl))

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
        if self._goal_is_blacklisted(plan, self.leg_goal):
            # Session 58: 22:40:25.165 -- a bump pulse fired 1.0s AFTER the planner already retired this
            # SAME goal PERMANENT (22:40:24.170), because nothing here re-checked the live blacklist. Stash
            # a MISSED-BUMP (never silent) instead of pulsing a dead region.
            self._missed_bump = (f"{reason} (goal {self.leg_goal} is already PERMANENTLY blacklisted "
                                 f"— no pulse; the planner retired it before this contact)")
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

    def _arm_loss_backoff(self, now, why):
        """Session 47: the shared tail for BOTH loss-instant back-off triggers (the geometric clearance one
        and the visual F_LKG one). Two reasons it exists:

        (1) It counts into `_blind_contact_reacts` -- the SAME wedge counter `_blind_contact_backoff` uses.
            Session 46 built the "stop repeating a reflex that isn't working -> run the FALLBACK sweep
            instead" escalation but wired it ONLY into `_blind_contact_backoff`, which is polled from
            HOLD_LOST/SLAM_HOLD -- states that hold NO directional command, so `_detector_command` returns
            None and `wall_contact`/`backwall_contact` are structurally ALWAYS False there. Flight
            20260901_142738 proves it: 12 HOLD_LOST entries, 14 SLAM_HOLD entries, ZERO detector verdicts
            from either, one latched contact all flight (a CEILING during ASCEND). The escalation could not
            fire even in principle, while the door the drone actually used -- this one, 7 times -- counted
            nothing. A back-off is a back-off whichever trigger fired it; both now feed one count.
        (2) It keeps the escalation decision in ONE place, so the two call sites cannot drift apart.

        Returns the (active, state, event) tuple for the caller to hand straight back."""
        # Session 52 (chunk 4): an episode that earned a physical back-off must not ALSO turn-probe on top
        # of it -- clear the probe's late-entry latch here, the shared tail both triggers funnel through.
        self._visrec_probe_armed = False
        self._blind_contact_reacts += 1
        if self._blind_contact_reacts > self.blind_contact_escalate_after:
            # Same shape as `_blind_contact_backoff`'s escalation: a FRESH sweep skips the initial
            # transient-wait (we have been stuck far longer than that already), but an IN-PROGRESS sweep
            # keeps its phase, timer and 720-degree budget untouched.
            if self._fallback_phase is None:
                self._fallback_phase = "TURN"
                self._fallback_phase_t0 = now
            self._player = None
            return self._enter_fallback_sweep(
                now, f"{why} -> WEDGED: {self._blind_contact_reacts} back-offs with no confirmed recovery "
                     f"between them -> escalate to FALLBACK sweep (a DIFFERENT maneuver)")
        self._player = None
        self._backoff_t0 = now
        self._enter("BACKOFF", now)
        return {}, "BACKOFF", f"{why} -> immediate standoff back off (re-arm bump latch) -> settle"

    def _step_lost_recovery(self, plan, now, visual_match, status):
        """Session 57: the WHOLE PLAN-LOST/NO-PLAN loss-instant decision, replacing the one-shot
        ticket path for these statuses. PLAN-STALE still uses `_maybe_loss_snapshot_backoff`.

        Returns the usual (fields: dict, state: str, event: str | None) triple to hand straight
        back to the caller, or None to fall through to the caller's plain hard hover-hold.

        Diagnosed off flight 20260903_083329 (see plans/session57-spec.md MISSION CONTEXT):
        `_maybe_loss_snapshot_backoff`'s one-shot ticket was spent on the very first tick of a loss
        whenever the cached clearance already read clear -- before `visual_match` could ever be
        anything but `None` (the match block runs BEFORE `ctrl.step()` in `run_explore`) -- so the
        camera was never consulted again for the rest of the episode. 56 of 86 loss episodes in that
        flight ran zero matches. The rule here: always wait `loss_backoff_grace_s` unconditionally
        (no more "too-close evidence" framing -- the wait is now unconditional); past the grace,
        ALWAYS run LKG matching for as long as the episode lasts (see `wants_visual_match`'s session
        57 clause); the remembered clearance decides only whether a back-off is PENDING, never
        whether we look; and the inlier-spread `closer` verdict (session 57 chunks 1-3) adjudicates a
        pending back-off three ways -- LIVE/EQUAL/UNKNOWN backs off, LKG holds indefinitely. Firing a
        back-off restamps `_loss_episode_t0`, which is what stops it repeating -- see the restamp
        below, step 6."""
        # step 1: no episode stamp -> nothing to time.
        if self._loss_episode_t0 is None:
            return None
        # step 2: unconditional wait, regardless of any evidence -- see MISSION CONTEXT finding 1.
        waited = now - self._loss_episode_t0
        if waited < self.loss_backoff_grace_s:
            if not self._loss_grace_noticed:
                self._loss_grace_noticed = True
                self.note_timeout("LOSS_GRACE", (
                    f"LOSS-RECOVERY GRACE: holding still for {self.loss_backoff_grace_s:.0f}s before any "
                    f"reaction (96.9% of held-still losses resolve inside that window). Looking at the "
                    f"camera after that."), now)
            return None
        # step 3: the cached clearance decides whether a back-off is PENDING -- never whether we look.
        pending = (self._last_good_clearance is not None
                  and self._last_good_clearance <= self.stop_clearance_dist)
        if not pending:
            return None
        # step 4/5: the three-way size-ratio verdict adjudicates the pending back-off. "LKG" means the
        # camera is confident we are already FARTHER than F_LKG was -- hold, do not act on stale geometry.
        verdict = visual_match.closer if visual_match is not None else "UNKNOWN"
        if verdict == "LKG":
            if not self._lost_hold_noticed:
                self._lost_hold_noticed = True
                self.note_timeout("LOST_VISUAL_HOLD", (
                    f"LOST-RECOVERY HOLD: F_LKG reads closer than the live view (size_ratio="
                    f"{visual_match.size_ratio:.2f}, {visual_match.inliers} inliers) -- we are already "
                    f"farther from the surface than the cached too-close evidence describes. Holding, "
                    f"no back-off. This hold is indefinite by design: it exits only on SLAM returning OK, "
                    f"or on a fresh plan routing to PLAN-STALE and its own probe path."), now)
            return None
        # step 6: LIVE / EQUAL / UNKNOWN all proceed -- weak CV (UNKNOWN) must never veto a back-off.
        clr = self._last_good_clearance
        ratio_txt = (f"{visual_match.size_ratio:.2f}"
                    if (visual_match is not None and visual_match.size_ratio is not None) else "n/a")
        why = (f"loss detected with cached clearance {clr:.2f} <= {self.stop_clearance_dist:.2f}, "
              f"visual verdict closer={verdict} (size_ratio={ratio_txt})")
        self._register_bump(dict(plan, pos=self._last_good_pos),
                            "clearance stand-off (stale pose @ loss)")
        # Session 57: taking a PHYSICAL action restarts the wait. This is what stops a back-off
        # repeating -- it replaces the one-shot ticket's re-fire protection. After the push the drone
        # is farther from the surface, so when the next window matures the match reads closer="LKG"
        # and step 5 holds. Restamping also re-arms both notices, since this is a NEW wait window.
        self._loss_episode_t0 = now
        self._loss_grace_noticed = False
        self._lost_hold_noticed = False
        if self.backoff_on_standoff:
            return self._arm_loss_backoff(now, why)
        self._enter("SETTLE", now)
        return {}, "SETTLE", f"{why} -> immediate standoff settle"

    def _maybe_loss_snapshot_backoff(self, plan, now, visual_match=None, status=None):
        """Session 34, Idea B (Step 1) + session 35 ALT (Step 2/2c): the ONE-SHOT check at the instant a
        loss episode begins. Marks the one-shot spent immediately (so a LOST<->STALE flicker within the
        same episode can't fire twice), then runs a short decision tree and returns the (active, state,
        event) tuple to return immediately, or `None` if the caller should fall through to its normal
        loss-entry behavior (REWIND/FALLBACK/plain hard-hover):
        Session 57: reached ONLY for `status == "PLAN-STALE"` now -- the PLAN-LOST/NO-PLAN path this
        function used to also serve is entirely handled by `_step_lost_recovery` above (see MISSION
        CONTEXT in plans/session57-spec.md: PLAN-LOST/NO-PLAN needed an unconditional wait-then-always-
        look rule that this one-shot-ticket shape could not express). The docstring below is otherwise
        unchanged.
          Step 1 (geometric, UNCHANGED): a last-known-good clearance was cached (see `step()`'s per-tick
            cache) and it reads too close -> back off using that CACHED position (the live `plan['pos']` is
            unavailable during a loss), exactly like ADVANCE's own clearance stand-off. Fires for ANY status.
          Step 2 (visual, session 35 ALT): Step 1 was inconclusive (clearance clear or never cached — the
            case geometry can't cover, e.g. SLAM never integrated the wall we flew into) -- a CONTAINED
            (zoomed-in crop) or PLANAR-LIKE (flat-surface) match of the live frame against F_LKG is the same
            "too close" verdict, reusing the identical bump+BACKOFF action. Also fires for ANY status --
            both Step 1 and Step 2 are one-shot DEFENSIVE reactions to an already-known reading, not an
            active search, so they stay available regardless of why perception went quiet.
          Step 2c (session 35 ALT; session 42 scoped it to PLAN-STALE only): both loss-instant checks were
            inconclusive -- if `use_visual_recovery_on_stale` AND `status == "PLAN-STALE"`, hand off to the
            15° rotational visual probe (VISUAL_RECOVERY); otherwise return None (falls through to the
            caller's plain hard-hover-hold). Diagnosed off two real flights (`20260721_233244`,
            `20260722_124351`): every VISUAL_RECOVERY entry in both (62 total) was actually triggered by
            PLAN-LOST, never PLAN-STALE, and every one reverted to HOLD_LOST one tick later (a separate,
            now-fixed dispatch gap -- `_step_visual_recovery` was only ever reachable from PLAN-STALE's
            `_step_stale`). PLAN-LOST/NO-PLAN means perception itself stopped publishing -- a throughput/
            backlog problem (session 28 diagnosed exactly this: a synchronous SLAM solve blocking the loop
            for 9-10s), not a "this viewpoint is confusing" problem -- so an active turn-search isn't a
            coherent remedy and the operator's call is to always just hold still and wait for perception to
            speak again. PLAN-STALE (perception alive, SLAM explicitly reports not-tracking) is the case
            where a different viewpoint is a coherent remedy, and stays the only entry into the probe.
        Deliberately scoped to ONE attempt at the very first tick of the episode -- before that boundary
        nothing has moved yet, so the cached snapshot / F_LKG are trustworthy; ANY later tick could include
        motion from a reactive maneuver (BLIND_BACKOFF, etc.), which is why this is never re-tried
        mid-episode (the probe itself, once entered, runs its own multi-tick loop separately)."""
        # Session 48 -- LOSS-RECOVERY GRACE. Firing at the loss INSTANT was reacting to a 3s-timeout blip,
        # not to being stuck. Measured over ALL 128 flight logs (2066 loss episodes): with the drone HOLDING
        # STILL a loss resolves on its own in a median of 2.4s, p95 9.5s, and 96.9% within 12s of the
        # PLAN-LOST declaration. The two most recent flights make it concrete -- SEVEN of their EIGHT
        # back-offs fired into episodes that recovered by themselves in under 1.1s (0.11 / 0.20 / 0.55 / 0.67
        # / 0.77 / 0.87 / 1.09s): the plan was green again before the 2s reverse push had even finished. So a
        # loss only earns a physical reaction once it OUTLIVES `loss_backoff_grace_s`. While deferring we
        # leave the one-shot ARMED (deliberately NOT spent) -- the HOLD_LOST tick and _step_stale both re-call
        # us every tick, and the drone is holding still throughout, which preserves the "nothing has moved
        # since the snapshot was taken" invariant this whole check is built on.
        # Session 57: `planar_like` means "flat surface", not "closer" -- see the identical note at
        # the action site above (08:51:44.313 evidence). This clause must stay in lockstep with that
        # site's conjunct, or the grace/gate can arm for a reaction the action then declines to take,
        # silently stalling the episode.
        _would_react = ((self._last_good_clearance is not None
                         and self._last_good_clearance <= self.stop_clearance_dist)
                        or (self.use_visual_recovery_on_stale and visual_match is not None
                            and visual_match.matched
                            and (visual_match.contained or visual_match.planar_like)
                            and visual_match.closer == "LIVE"))
        if _would_react and self._loss_episode_t0 is not None:
            waited_loss = now - self._loss_episode_t0
            if waited_loss < self.loss_backoff_grace_s:
                if not self._loss_grace_noticed:
                    self._loss_grace_noticed = True
                    self.note_timeout("LOSS_GRACE", (
                        f"LOSS-RECOVERY GRACE: too-close evidence at the loss instant, but holding still for "
                        f"{self.loss_backoff_grace_s:.0f}s to let SLAM re-lock before reacting "
                        f"(96.9% of held-still losses resolve inside that window). No back-off yet."), now)
                return None
        # Session 47 -- POST-BACKOFF SLAM RE-SOLVE GATE (see `_backoff_resolve_since`). A back-off that SLAM
        # never got to look at cannot have been judged, so re-firing off the SAME pre-backoff evidence is not
        # a decision, it is a loop: flight 20260901_142738 did exactly that 7 times (once only 1.05s after the
        # previous back-off ENDED), because SETTLE's freshness gate needs 6 frames under slam_slow_ms=1000 and
        # SLAM was solving at 3400-3700ms, so SETTLE could never complete and the next OK->PLAN-LOST flip was
        # always the thing that broke the deadlock. HOLD instead -- the caller falls through to its hard
        # hover-hold, which is precisely "give SLAM the opportunity to recover".
        # Session 52 (chunk 3, flight 20260901_222552): this gate used to sit ABOVE the evidence clauses and
        # unconditionally spent the one-shot + printed the suppression notice, even when `_would_react` is
        # False -- i.e. even when there was never any too-close evidence to re-fire off in the first place
        # (both occurrences this flight, 22:46:34 and 22:49:51, were exactly that: the drone had already
        # backed off to a clear reading, so the gate fired on nothing). Gate it behind `_would_react` so an
        # evidence-free loss falls straight through to the visual-recovery hand-off below instead of being
        # silently swallowed for the whole `backoff_resolve_budget_s` window, and leave the one-shot ARMED
        # (do NOT set `_loss_snapshot_checked` yet) while genuinely suppressing so the check gets a real shot
        # once the gate clears.
        if _would_react and self._backoff_resolve_since is not None:
            waited = now - (self._backoff_resolve_t0 if self._backoff_resolve_t0 is not None else now)
            if waited < self.backoff_resolve_budget_s:
                if not self._backoff_gate_noticed:
                    self._backoff_gate_noticed = True
                    self.note_timeout("BACKOFF_SUPPRESSED", (
                        f"LOSS-INSTANT BACK-OFF SUPPRESSED: waiting for SLAM to solve a frame captured after the "
                        f"last back-off ({waited:.1f}s / {self.backoff_resolve_budget_s:.0f}s budget) — holding "
                        f"still rather than re-firing off pre-backoff evidence "
                        f"[{self._blind_contact_reacts} back-off(s) this episode]"), now)
                return None
            # Budget exhausted -- the capture stream never produced a timestamped frame we could compare
            # against. Open the gate, but say so LOUDLY (CLAUDE.md: a degraded path is never silent).
            self.note_timeout("BACKOFF_GATE_TIMEOUT", (
                f"POST-BACKOFF RE-SOLVE GATE TIMED OUT after {waited:.1f}s (budget "
                f"{self.backoff_resolve_budget_s:.0f}s) — no SLAM frame captured after the last back-off ever "
                f"solved. Proceeding UNCONFIRMED; the loss-instant back-off is re-enabled."), now)
            self._backoff_resolve_since = None
            self._backoff_resolve_t0 = None
        self._loss_snapshot_checked = True
        if self._last_good_clearance is not None and self._last_good_clearance <= self.stop_clearance_dist:
            self._register_bump(dict(plan, pos=self._last_good_pos),
                                "clearance stand-off (stale pose @ loss)")
            clr = self._last_good_clearance
            # Session 48: REPORT the snapshot's age. Past the grace this pose is >= loss_backoff_grace_s +
            # plan_timeout_s old (>=15s at the defaults) -- still the best geometric evidence available while
            # blind, but the operator must be able to see how stale the thing we acted on actually was.
            _age = (f"stale pose, {now - self._last_good_t:.1f}s old"
                    if self._last_good_t is not None else "stale pose, age unknown")
            if self.backoff_on_standoff:
                return self._arm_loss_backoff(
                    now, f"loss detected with cached clearance {clr:.2f} <= "
                         f"{self.stop_clearance_dist:.2f} ({_age})")
            self._enter("SETTLE", now)
            return {}, "SETTLE", (f"loss detected with cached clearance {clr:.2f} <= "
                                  f"{self.stop_clearance_dist:.2f} (stale pose) -> immediate standoff settle")
        # Everything past this point is the session-35-ALT visual path, gated as ONE unit behind
        # `use_visual_recovery_on_stale` -- flag False reproduces today's (session 34) behavior
        # byte-for-byte: `visual_match` is only ever non-None here because `run_explore` only builds the
        # probe / computes a match when this same flag is on, but gating it here too keeps that an
        # explicit invariant of this function rather than an implicit contract with its one caller.
        if not self.use_visual_recovery_on_stale:
            return None
        # Session 57: `planar_like` means "flat surface", not "closer" -- it is a pure inlier-ratio
        # test, direction-blind. Real flight evidence (08:51:44.313): matched=True inliers=45
        # contained=False planar_like=True scale=0.32 -- scale<1 means the live view is a SHRUNK
        # view of F_LKG, i.e. farther away, yet this branch backed off anyway. Require the
        # inlier-spread verdict to agree that the live frame is actually the closer one.
        if (visual_match is not None and visual_match.matched
                and (visual_match.contained or visual_match.planar_like)
                and visual_match.closer == "LIVE"):
            self._register_bump(dict(plan, pos=self._last_good_pos), "visual too-close @ loss")
            kind = "contained crop" if visual_match.contained else "planar/flat surface"
            if self.backoff_on_standoff:
                return self._arm_loss_backoff(
                    now, f"loss detected with a visual match against F_LKG ({kind}, "
                         f"{visual_match.inliers} inliers)")
            self._enter("SETTLE", now)
            return {}, "SETTLE", (f"loss detected with a visual match against F_LKG ({kind}, "
                                  f"{visual_match.inliers} inliers) -> immediate standoff settle")
        # Session 42: the active turn-search hand-off is PLAN-STALE-only (see the docstring's Step 2c) --
        # a PLAN-LOST/NO-PLAN loss with both checks inconclusive falls through to the caller's plain
        # hard-hover-hold instead, never spinning open-loop while perception itself is silent.
        if status != "PLAN-STALE":
            return None
        # Session 57: a probe TURNS -- it must wait out the same loss-recovery grace a back-off does
        # (session 48: 2066 loss episodes, 96.9% of held-still losses resolve inside the window) before
        # ANY physical reaction, including a turn. Before chunk 5 this hand-off had no grace check of its
        # own and relied on the one-shot ticket being unspent to keep it from firing early -- chunk 4's
        # late-entry `_maybe_enter_visual_probe` path bypasses that ticket, so this hand-off could turn
        # the drone inside the grace. Reuses `_maybe_enter_visual_probe`'s condition verbatim rather than
        # writing a second variant of the same rule.
        if self._loss_episode_t0 is not None and now - self._loss_episode_t0 < self.loss_backoff_grace_s:
            return None
        return self._enter_visual_recovery(now, "loss detected, clearance + visual loss-instant checks "
                                                 "both inconclusive -> 15° visual recovery probe")

    def take_missed_bump(self):
        """Pop the pending MISSED-BUMP marker (a real contact that emitted no pulse), or None."""
        m, self._missed_bump = self._missed_bump, None
        return m

    def take_notice(self):
        """Pop the pending one-shot operator NOTICE (e.g. the session-22 height-reference disagreement
        warning), or None. run_explore prints + diag-logs it (VISIBLE telemetry, no silent state)."""
        n, self._pending_notice = self._pending_notice, None
        return n

    def note_timeout(self, kind: str, text: str, now: float, loud: bool = True) -> None:
        """Latch a timeout/forced-escape for the visualizer's notice line (session 52).

        ALWAYS overwrites `self.last_timeout` (newest wins; it is never consumed, unlike
        `_pending_notice`). When `loud`, ALSO sets `_pending_notice` so run_explore prints the
        existing `*** ... ***` console/diag line. Pass loud=False at sites that already log the same
        sentence as their `event` return value, so nothing double-logs.
        """
        self.last_timeout = {"kind": kind, "text": text, "t": float(now)}
        if loud:
            self._pending_notice = text

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
             backwall_contact=False, status="OK", visual_match=None):
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
            # Same operator override as CALIB_VERIFY's latch (only reached here if CALIB_VERIFY never ran
            # at all, e.g. --no-takeoff) -- keeps the override consistent regardless of prelude path.
            self.target_altitude_y = (self.desired_height_override_y if self.desired_height_override_y
                                      else float(plan["pos_y"]))
        # Record the TAKE-OFF heading once: the first healthy SLAM heading after the prelude completes. General
        # (whatever heading the drone armed at — not a room answer); ORIENT_HOME faces it before the final dock.
        if (self.airborne_done and self._takeoff_heading is None
                and plan.get("plan_valid") and plan.get("heading_deg") is not None and not self._slam_slow):
            self._takeoff_heading = float(plan["heading_deg"])
        self._update_slam(plan)   # track SLAM frame-build time for the settle gate (below + at the gate sites)
        if plan.get("plan_valid"):
            self._ever_tracked = True   # SLAM has tracked at least once -> a later empty-history STALE is a real loss, not warmup
            # Session 34: cache the last-known-good position + forward clearance every valid tick. The map
            # itself is frozen (no new integration) the instant tracking drops, so this snapshot stays a
            # reasonable proxy for "what's near us right now" for as long as the drone hasn't actually moved
            # since it was taken -- see the one-shot loss-instant check below, which is the only consumer and
            # enforces that "hasn't moved" boundary itself.
            if plan.get("pos") is not None:
                self._last_good_pos = list(plan["pos"])
            if plan.get("forward_clearance_dist") is not None:
                self._last_good_clearance = float(plan["forward_clearance_dist"])
                self._last_good_t = now      # session 48: so a decision made off it can REPORT its age
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
                        self.note_timeout("HEIGHT_DISAGREEMENT", (
                            f"HEIGHT-REFERENCE DISAGREEMENT: flying-height median {_med:+.3f} is "
                            f"{abs(_med - self._desired_y):.3f}u (> delta {abs(self._trim_delta):.3f}) from "
                            f"desired_y {self._desired_y:+.3f} — long off-height flight, or SLAM Y drift "
                            f"(check the Y-DRIFT audit / HEIGHT panel)"), now)

        # --- status-gated SLAM-loss recovery (CONTROL-SPACE); active only in the explore phase ---
        if self._explore_started:
            lost = status in ("PLAN-LOST", "NO-PLAN", "PLAN-STALE")
            # Session 34: a FRESH loss (this tick lost, last tick wasn't) arms the one-shot loss-instant
            # clearance check (see the PLAN-LOST/PLAN-STALE fresh-entry branches below) exactly once per
            # episode -- reset here, independent of `_recovering` (which only the PLAN-STALE path sets), so a
            # loss that starts as PLAN-LOST gets the same one-shot chance a PLAN-STALE start would.
            if lost and not self._was_lost:
                self._loss_snapshot_checked = False
                self._loss_episode_t0 = now          # session 48: the loss-recovery grace window starts HERE
                self._loss_grace_noticed = False
                self._backoff_gate_noticed = False   # session 52: fresh episode, gate suppression may re-notice
                self._lost_hold_noticed = False       # session 57: fresh episode, hold notice may re-notice
                self._visrec_probe_armed = True       # session 52 (chunk 4): re-arm the probe's late entry
            if not lost:
                self._loss_episode_t0 = None         # a genuine OK ended the episode -- next loss re-stamps
                self._loss_grace_noticed = False
            self._was_lost = lost
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
            # Session 46: BACKOFF must own EVERY status while its phase-timer runs, exactly like
            # BLIND_BACKOFF/CALIB_ESCAPE above. Flight 20260901_124211: all 6 loss-instant BACKOFFs emitted
            # fields={} and were wiped by the PLAN-LOST router one tick later -- zero reverse ever commanded.
            if st == "BACKOFF":
                # Session 47: thread the live rear-contact flag through -- BACKOFF is the ONE state that
                # actually commands reverse, so it is the one state whose detector can see a wall behind us.
                return self._step_backoff(now, lost=lost, backwall_contact=backwall_contact)
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
                    # Session 46: a FALLBACK sweep in progress must keep running while blind. Its ONLY other
                    # dispatch is _step_stale (PLAN-STALE), so under PLAN-LOST it was entered and then wiped
                    # by the HOLD_LOST forcing below one tick later -- the same structural bug as BACKOFF
                    # (flight 20260901_124211, which never saw a single PLAN-STALE event). The live flow
                    # contacts MUST be threaded through: while blind they are the only signal that can cut a
                    # randomized push short on an obstacle.
                    if st == "FALLBACK":
                        return self._step_fallback_sweep(now, wall_contact, backwall_contact)
                    # Session 52 (chunk 4): the same session-46 fix, applied to the one remaining state that
                    # needed it. An in-flight VISUAL_RECOVERY probe (now reachable via the late entry in
                    # `_step_stale`) was structurally killed by a PLAN-LOST flicker one tick after entry --
                    # this flight's only VISUAL_RECOVERY (22:59:04) was entered and then wiped by a PLAN-LOST
                    # flip 1.5s later (22:59:06.265), before it ever reached its MATCH phase. Without this
                    # branch a probe can never survive long enough to complete a single match on a real flight.
                    if st == "VISUAL_RECOVERY":
                        return self._step_visual_recovery(now, plan, visual_match)
                    # Session 57: unconditional -- the ticket is no longer consulted or spent on this
                    # path (see `_step_lost_recovery`'s docstring for why the one-shot shape couldn't
                    # express "always wait, then always look").
                    snap = self._step_lost_recovery(plan, now, visual_match, status)
                    if snap is not None:
                        return snap
                    self._player = None
                    self._enter("HOLD_LOST", now)
                    return {}, "HOLD_LOST", ("PLAN-LOST -> HARD HOVER-HOLD (indefinite; waiting for "
                                             "perception, no blind recovery)")
                # "Hard hover" is not "ignore a wall we're touching" — the flow contact detector runs
                # independently of SLAM, so react to it even while blind (see _blind_contact_backoff).
                reaction = self._blind_contact_backoff(now, wall_contact, backwall_contact, "HOLD_LOST")
                if reaction is not None:
                    return reaction
                # Session 48: the loss-instant check is not decided at the loss INSTANT -- it defers
                # while the loss-recovery grace runs, so re-run it here every tick until it actually
                # reaches a decision; without this re-call the deferred check would simply never be
                # revisited and the back-off would be dead code rather than delayed.
                # Session 57: `_step_lost_recovery` now owns the WHOLE PLAN-LOST/NO-PLAN decision --
                # unconditional every tick, no ticket to check or spend -- see the fresh-entry site above.
                deferred = self._step_lost_recovery(plan, now, visual_match, status)
                if deferred is not None:
                    return deferred
                return {}, "HOLD_LOST", None
            if status == "PLAN-STALE":
                # Perception is publishing but SLAM is not TRACKING -> retrace to re-expose keyframes.
                return self._step_stale(now, plan, wall_contact, backwall_contact, visual_match)
            # status OK: if we were recovering, perception is TRACKING again -> DON'T fly on the first frame
            # back (a fresh RELOC pose is shaky). Hold until SLAM settles (>N fast frames), THEN brake + REPLAN.
            # NOTE: `_recovering` + the give-up counter are NOT cleared here — a bare, un-settled OK is not yet
            # trusted; trust restores in SLAM_HOLD's settle-gate-clear branch below, once genuinely settled
            # (session 35 D5 simplification).
            # Session 24: a corner-giveup-terminal STUCK (see the REPLAN `done` branch) must NOT be swept into
            # this generic "recovery -> OK -> settle -> replan" convergence -- it has nothing to recover INTO
            # (every corner is exhausted); the drone is meant to hold here for good, not bounce back out the
            # instant status reads OK (which it does continuously while just sitting still).
            if st in _RECOVERY_STATES and not (st == "STUCK" and self._corner_giveup_stuck):
                self._settle_to = "REPLAN"
                return self._enter_slam_hold("SETTLE", now,
                                             "plan OK -> wait for SLAM to settle -> brake -> replan "
                                             "(re-locked; NOT trusted until a >=1u ADVANCE confirms)")

        # --- STATE-INDEPENDENT "goal already reached" check (session 45) ---
        # ADVANCE has always had this test, but ONLY inside its own handler (see the `goal reached (d=...)`
        # branch there), so it can only fire while actively flying a forward hop. Flight 20260901_112227 sat
        # 0.31u from its committed goal for 221 SECONDS -- goal_reach_dist is 1.0, so it was three times deeper
        # inside "reached" than the threshold -- cycling SLAM_HOLD -> REPLAN -> ORIENT -> PARALLAX_PUSH ->
        # SLAM_HOLD and never once entering ADVANCE, so the test never ran. Retire the leg from WHEREVER it is
        # parked instead: a goal we are already standing on cannot teach us anything more from here.
        #
        # EXCLUSIONS (each load-bearing, do NOT drop):
        #   • plan_valid + a live pos: never retire a goal off a stale/blind pose (session 42's PLAN-LOST
        #     reasoning -- during a loss the map and pose are frozen, so "reached" would be a guess).
        #   • POSTLUDE_STATES / _RECOVERY_STATES / _calib_active: those own their own convergence.
        #   • "SETTLE"/"REPLAN": _enter("SETTLE") RESETS _settle_t0, _settle_ok=0, _settle_last_fid and (unless
        #     arriving from SLAM_HOLD) restamps the settle gate. The drone does NOT move during SETTLE, so the
        #     goal stays inside goal_reach_dist and this would re-fire EVERY TICK, zeroing the fresh-frame
        #     counter before it could ever reach settle_fresh_frames -- an infinite SETTLE hover, strictly
        #     worse than the stall being fixed. REPLAN is a one-tick pass-through that re-reads the goal anyway.
        #   • "TRIM"/"TRIM_RESUME_WAIT": never interrupt a physical pulse mid-maneuver; _trim_resolve_resume
        #     already re-validates its preserved goal on exit, so this simply fires on the next ORIENT/ADVANCE.
        # Terminates: fire -> SETTLE (excluded, dwells normally) -> REPLAN -> the planner's own session-45
        # too-close rejection no longer hands back a goal we are standing on.
        if (st not in _REACHED_EXCLUDED_STATES and not self._calib_active
                and self.leg_goal is not None
                and plan.get("plan_valid") and plan.get("pos") is not None):
            _reached_d = self._dist(plan["pos"], self.leg_goal)
            if _reached_d is not None and _reached_d <= self.goal_reach_dist:
                self._player = None
                self._slam_resume = None      # this leg is over; no deferred resume target survives it
                self._settle_to = "REPLAN"
                self._enter("SETTLE", now)
                return {}, "SETTLE", (f"goal already reached (d={_reached_d:.2f} <= "
                                      f"{self.goal_reach_dist:.2f}) while parked in {st} -> settle -> replan")

        # --- GRADUAL HEIGHT TRIM trigger (session 14; BIDIRECTIONAL session 22; HARDCODED session 44) ---
        # On a FRESH HEALTHY frame, in a whitelisted settled/travel/hold state, fire a short vertical pulse in
        # EITHER direction against two fixed absolute SLAM pos_y thresholds (session 44 -- see the operator-
        # approved-exception note on trim_sag_trigger_y/trim_high_trigger_y above; this replaced the session-22
        # live-calibrated ceiling/desired/delta band):
        #   too LOW  (sagged):  pos_y >= trim_sag_trigger_y   -> TRIM UP (vertical pulse up)
        #   too HIGH (glued near the ceiling): pos_y <= trim_high_trigger_y -> TRIM DOWN (vertical pulse down)
        # Whitelist = {SETTLE, ADVANCE, SLAM_HOLD}. Suppressed during any calibration and while already trimming.
        # Still requires at least one completed calibration (_ceiling_y is not None) before it can arm at all --
        # NOT because the threshold VALUES depend on it anymore (they're the hardcoded literals above), but
        # because SETTLE is reached generically during PRELUDE/ARM too, before the drone has even calibrated
        # or taken off; without this precondition the hardcoded band can (and, caught by the self-test suite,
        # DID) misfire against whatever incidental pos_y the prelude happens to be sitting at and hijack the
        # whole takeoff sequence. This is a basic "don't act before airborne+calibrated" precondition, not a
        # reintroduction of the deleted ratio math.
        # Session 52: dropped the "and not self._slam_slow" gate that used to guard this trigger.
        # Flight 20260901_222552 logged 2749 consecutive SETTLE/ADVANCE ticks with slam_ms >= 1000
        # (zero ticks under 1000ms) while pos_y drifted -2.03 -> -2.22, past trim_high_trigger_y
        # (-2.10) for the entirety of the last minute -- yet TRIM never fired again after 22:37:59.
        # Per the doctrine already established at sessions 28/42/43, a slow SLAM solve is a
        # perception THROUGHPUT signal, not evidence the last published pose is wrong -- and pos_y
        # is the slowest-varying quantity SLAM publishes, so a stale-but-slow reading is still a
        # trustworthy height reading. The other six _slam_slow gates in this file guard MOTION/
        # HEADING decisions and are untouched -- CORRECTION (session 54): that count was wrong. The
        # TRIM WAIT sub-phase's `healthy` check (below, in the "elif st == TRIM" handler) is ALSO a
        # height-reading gate on the identical pos_y, and session 52 left it in place -- so TRIM became
        # enterable under slow SLAM without becoming exitable. Flight 20260902_155916 hovered in TRIM
        # for 73.9s as a direct result. Session 54 removed that gate too and added a bounded forced
        # exit (slam_slow_hop_after_s) as a backstop; see that handler for the full trace.
        # Session 56: MOVED here from just below the SLAM_HOLD hold-handler (its old position, directly
        # above the old "if st == 'ARM':" chain start). That old spot is UNREACHABLE whenever st ==
        # "SLAM_HOLD": the "if st == 'SLAM_HOLD':" block right below EXHAUSTIVELY returns on every one of
        # its branches (settle-gate-clear resume, blind-contact reaction, forced hop, step-back, or the
        # plain "keep holding" fall-through) -- so simply adding "SLAM_HOLD" to _TRIM_TRIGGER_STATES without
        # relocating the check would have been a silent no-op. Moving the check to fire BEFORE that handler
        # makes SLAM_HOLD's height-trim trigger reachable exactly like SETTLE/ADVANCE already are (neither
        # has an earlier exhaustive-return handler in between). No behavior change for SETTLE/ADVANCE: this
        # is strictly before their own handlers too, both before and after the move.
        if (self.trim_enable and st in _TRIM_TRIGGER_STATES and not self._calib_active
                and self._ceiling_y is not None
                and plan.get("plan_valid") and plan.get("pos_y") is not None):
            _y = float(plan["pos_y"])
            trim_dir = None
            if _y >= self.trim_sag_trigger_y:
                trim_dir = "UP"                          # sagged low -> climb back
            elif _y <= self.trim_high_trigger_y:
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
                # Session 56: when st == "SLAM_HOLD" this is a trim FROM a still-open bad-SLAM episode, not
                # a trusted resolution of it. Deliberately left untouched here (and _enter("TRIM", now) below
                # touches none of them either -- see _enter's field-by-field list): _slam_hold_episode_t0 (the
                # episode isn't over because we trimmed), _recovering/_history_broken (a trim is not a trust
                # restoration -- only SLAM_HOLD's own settle-gate/session-35 trust check may clear those), and
                # _slam_resume (the deferred resume target SLAM_HOLD was already holding for -- preserved so
                # _trim_resolve_resume can re-enter SLAM_HOLD honouring it; see that method).
                self._enter("TRIM", now)
                st = "TRIM"          # route into the TRIM handler below this tick

        # SLAM settle gate (session 24): hover until BOTH the freshness + physical-motion gates clear (see
        # _settle_gate_poll), then resume the deferred state. Covers every resume target uniformly -- "SETTLE"
        # (the gate stays open across the hop, so SETTLE's own poll below almost always passes instantly),
        # "ADVANCE"/"PARALLAX_PUSH" (previously resumed with NO gate at all; now get the same two-gate check).
        if st == "SLAM_HOLD":
            if self._settle_gate_poll(now):
                nxt = self._slam_resume or "REPLAN"
                self._slam_resume = None
                waited = now - (self._slam_hold_start if self._slam_hold_start is not None else self.t_state)
                # Session 53: the hold genuinely settled -- the bad-SLAM-patch episode this clock was
                # bounding is over. Reset here (mirrors _slam_stepback_count's REPLAN-handler reset) so the
                # NEXT bad patch starts its own fresh episode rather than inheriting this one's elapsed time.
                self._slam_hold_episode_t0 = None
                trust_note = ""
                # Session 35: settle-gate-based trust restoration. Gated STRICTLY on nxt == "SETTLE" (the
                # recovery-resume path) -- every other resume target ("ADVANCE"/"PARALLAX_PUSH", a plain
                # mid-leg/post-turn slow hold) is untouched, so an un-settled or non-recovery transition can
                # never clear these flags. MUST run before the Idea-A clearance check right below (not
                # because Idea-A reads `_recovering` -- it doesn't, it reads the live clearance directly --
                # but so trust is established first, then a decision is made using the now-trusted pose,
                # unambiguous on inspection).
                if nxt == "SETTLE" and self._recovering:
                    self._recovering = False
                    self._history_broken = False
                    self._reset_fallback_sweep()
                    self._blind_contact_reacts = 0    # session 46: confirmed recovery clears the wedge count
                    self._reset_visual_recovery()
                    self.command_history.clear()
                    trust_note = " -> recovery trust restored (SLAM settled)"
                # Session 34, Idea A: at this recovery trust boundary (only for the RECOVERY resume target,
                # "SETTLE" -- a plain mid-leg SLAM-slow hold resuming straight into ADVANCE/PARALLAX_PUSH gets
                # its own per-tick clearance check moments later anyway, so re-checking here would just be
                # redundant), check the now-LIVE clearance before resuming -- don't wait out SETTLE -> REPLAN
                # -> ORIENT -> ADVANCE to notice we re-locked right on top of a wall.
                if nxt == "SETTLE":
                    # Session 58: 22:40:24.170 BLACKLIST PERMANENT retired goal=[3.9,-3.6]; 1.0s later, at
                    # 22:40:25.165, bump pulse #2 fired against that SAME dead goal and backed off, because
                    # this clearance check never re-checked the live blacklist. `_register_bump`'s own guard
                    # (below) now stops the pulse, but leg_goal must also stop being DEFENDED here -- this is
                    # the one path that produced the observed event. Route straight to SETTLE->REPLAN, the
                    # same convergence `_trim_resolve_resume` already uses for a goal that died mid-TRIM.
                    if self.leg_goal is not None and self._goal_is_blacklisted(plan, self.leg_goal):
                        dead_goal = self.leg_goal
                        self.leg_goal = None
                        self._settle_to = "REPLAN"
                        self._enter("SETTLE", now)
                        return {}, "SETTLE", (f"SLAM settled after {waited:.1f}s but leg_goal {dead_goal} "
                                              f"was already PERMANENTLY blacklisted{trust_note} -> settle -> "
                                              "replan (dead goal, no standoff back-off)")
                    clr = plan.get("forward_clearance_dist")
                    if self.stop_on_clearance and clr is not None and clr <= self.stop_clearance_dist:
                        self._register_bump(plan, "clearance stand-off (post-recovery settle)")
                        if self.backoff_on_standoff:
                            self._player = None
                            self._backoff_t0 = now
                            self._enter("BACKOFF", now)
                            return {}, "BACKOFF", (f"SLAM settled after {waited:.1f}s but clearance {clr:.2f} "
                                                   f"<= {self.stop_clearance_dist:.2f}{trust_note} -> standoff "
                                                   "back off (re-arm bump latch) -> settle, before resuming REPLAN")
                        self._enter("SETTLE", now)
                        return {}, "SETTLE", (f"SLAM settled after {waited:.1f}s but clearance {clr:.2f} <= "
                                              f"{self.stop_clearance_dist:.2f}{trust_note} -> standoff settle "
                                              "before REPLAN")
                self._enter(nxt, now)
                # Session 56 ORDERING TRAP (identical to the forced hop's, autopilot.py:3212 / :4223-4228):
                # _enter() WIPES _slam_slow_hop_deadline for any state outside ("ADVANCE", "ORIENT",
                # "PARALLAX_PUSH"), and nxt is usually SETTLE/REPLAN -- so the grace MUST be stamped AFTER
                # _enter, never before. Without it the gate's new ~3.5s release would re-divert into SLAM_HOLD
                # on the very next slow frame, trading a 15s stall for a 3.5s limit cycle.
                self._slam_slow_hop_deadline = now + self.slam_slow_hop_grace_s
                return {}, nxt, (f"SLAM settled after {waited:.1f}s ({self._slam_fast_streak} fast frames, "
                                 f"last {self._slam_ms_latest:.0f}ms){trust_note} -> resume {nxt}")
            # Still waiting on the settle gate — blind, same as HOLD_LOST. React to a live wall/backwall
            # contact the same way (see _blind_contact_backoff) before falling through to the slow-SLAM
            # escape below.
            reaction = self._blind_contact_backoff(now, wall_contact, backwall_contact, "SLAM_HOLD")
            if reaction is not None:
                return reaction
            waited = now - (self._slam_hold_start if self._slam_hold_start is not None else self.t_state)
            # Session 53: the EPISODE clock -- persists across the PLAN-LOST/HOLD_LOST bounce that
            # `_slam_hold_start` does not (see `_slam_hold_episode_t0`'s own comment). Only the forced-hop
            # rescue below reads it; the legacy step-back branch further down keeps using the per-hold
            # `waited` above, unchanged.
            episode_waited = now - (self._slam_hold_episode_t0 if self._slam_hold_episode_t0 is not None
                                     else self.t_state)
            if not self.use_slam_stepback_on_slow:
                # Session 35 default, simplified session 43 (operator ask, off flight 20260723_000631: a
                # recovery-settle SLAM_HOLD sat 31.5s with plan OK the whole time and never forced a hop,
                # because the old condition below required `_slam_resume == "ADVANCE" and not self._recovering`
                # -- scoping this rescue to a plain mid-leg slow hold only, never a post-loss recovery-settle
                # hold). Operator's call: plan OK + still sitting in SLAM_HOLD past slam_slow_hop_after_s is
                # the ONLY condition that matters -- PLAN-LOST/a slow settle-gate is a perception THROUGHPUT
                # signal (session 28/42), not evidence the pose itself is wrong, so there's no principled
                # reason to let a recovery-settle hold wait indefinitely while a plain slow hold gets rescued.
                # Firing this now doubles as the trust-restoration boundary (mirrors exactly what the
                # settle-gate-clear branch above does) so a forced hop never leaves `_recovering`/history state
                # stuck as if still untrusted.
                # ONE guard kept (caught by the HEIGHT RE-CALIB self-test, off a STUCK->SLAM_HOLD convergence
                # with cap_ts NEVER fed): the timer alone can't tell "SLAM is slow but alive" from "SLAM has
                # never produced a single genuinely-captured frame" -- a total capture blackout is a different,
                # worse failure (perception saying NOTHING, not just something slow) and must never be papered
                # over by a wall clock -- that's flying on zero live data. Require at least one entry in the
                # current window to carry a real cap_ts (SLAM has told us SOMETHING, even if slow).
                # Session 56: now a BACKSTOP, not the normal path -- the settle-gate-clear branch above
                # stamps its own grace on every normal exit (see the ordering-trap comment there), so this
                # 15s bar should only ever be hit by the pathological case where perception republishes a
                # plan on its 0.5s timer (status stays OK) while cap_ts itself never advances.
                has_any_capture = any(cap_ts is not None for _, cap_ts in self._slam_hist)
                if episode_waited >= self.slam_slow_hop_after_s and has_any_capture:
                    if self._recovering:
                        self._recovering = False
                        self._history_broken = False
                        self._reset_fallback_sweep()
                        self._blind_contact_reacts = 0    # session 46: confirmed recovery clears the wedge count
                        self._reset_visual_recovery()
                        self.command_history.clear()
                    self._slam_resume = None
                    # NOTE: set the deadline AFTER _enter() -- "REPLAN" is not in _enter()'s ("ADVANCE",
                    # "ORIENT") exemption (it's a one-tick pass-through, not a state that itself spends the
                    # grace window), so setting it BEFORE would have it wiped by that same call.
                    self._enter("REPLAN", now)
                    self._slam_slow_hop_deadline = now + self.slam_slow_hop_grace_s
                    _msg = (f"SLAM_HOLD still waiting (this hold {waited:.1f}s / episode {episode_waited:.1f}s) "
                            f"but plan OK -> forcing one hop toward the current goal "
                            f"(grace {self.slam_slow_hop_grace_s:.0f}s)")
                    self.note_timeout("SLAM_HOLD_FORCED_HOP", _msg, now, loud=False)
                    return {}, "REPLAN", _msg
                return {}, "SLAM_HOLD", None
            # Legacy (use_slam_stepback_on_slow=true): step one entry back through the rewind queue to
            # re-expose known-good geometry so the solve can re-lock. Re-arm needs another full run of slow
            # frames. SKIP while `_recovering`: the history is frozen/possibly spatially stale during an
            # untrusted re-lock, so popping it for a step-back could fly a ghost path.
            if self._slam_slow_streak >= self.slam_stepback_after_frames and not self._recovering:
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
                self.note_timeout("ASCEND_CAP", event, now, loud=False)
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
                    # Operator override (desired_height_override_y, 0 = disabled): use the fixed value in
                    # place of the MEASURED settle instead -- _ceiling_y stays live either way, so
                    # _trim_delta (and TRIM's band) still tracks the real room.
                    override = self.desired_height_override_y
                    self._desired_y = override if override else settled_y
                    self._trim_delta = self._desired_y - self._ceiling_y
                    # Session 22: the altitude lock holds the SAME verified height TRIM defends (previously it
                    # lazily cached whatever pos_y came first post-prelude).
                    self.target_altitude_y = self._desired_y
                    calib_log = (f" | HEIGHT-CALIB values: ceiling_y={self._ceiling_y:+.3f} "
                                 f"desired_y={self._desired_y:+.3f} delta={self._trim_delta:.3f} "
                                 f"(TRIM band is HARDCODED, session 44, independent of this calibration: "
                                 f"{self.trim_high_trigger_y:+.3f} (high) .. {self.trim_sag_trigger_y:+.3f} (low))")
                    if override:
                        calib_log += (f" | DESIRED-HEIGHT OVERRIDE: {override:+.3f} (config, "
                                      f"NOT measured; settled_y was {settled_y:+.3f})")
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
            self._slam_hold_episode_t0 = None   # session 53: same trusted-recovery boundary, same reason
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
                    # Session 46: a MATERIALLY new goal (reusing the exact pick_moved test above, not a fresh
                    # distance check) clears the wedge-reflex counter -- a new leg starts with a clean count.
                    # Deliberately NOT reset on every same-disc re-commit: flight 20260901_112227 re-committed
                    # a jittered goal inside one goal_area_radius disc every ~25s via the forced hop, which
                    # would starve this counter (session 29/45's exact starvation class) before it could ever
                    # reach blind_contact_escalate_after.
                    if pick_moved:
                        self._blind_contact_reacts = 0
                    # A genuinely completed hop (prev_goal is not None -- a real ADVANCE was judged THIS replan,
                    # not a same-leg turn/scout sub-step still in progress) always counts as a real pick, even
                    # if it lands close to the last commit -- that IS the circling behaviour register_goal_pick's
                    # loop guard exists to catch (20260720 bug: a frontier repeatedly "reached" from ~the same
                    # spot never accrued a strike -- reaching is unconditional progress -- so only the picks-based
                    # loop guard could ever retire it, and it was starved by this same-position dedup treating
                    # every completed hop as if it were a same-leg sub-step). Only a same-tick sub-step of the
                    # SAME still-uncommitted leg (no hop judged yet) gets deduped.
                    same_goal_as_last_pick = (not pick_moved) and (prev_goal is None)
                    # Session 45 -- BOUND the dedup (flight 20260901_112227). The rule above suppresses a pick
                    # whenever no hop was JUDGED, which is right for a leg's own multi-step re-orientation
                    # (seconds). But under chronically slow SLAM no hop EVER completes, so `prev_goal` is
                    # permanently None; with the goal jittering less than goal_area_radius, EVERY pick was
                    # suppressed for the whole flight. The goals-DB proves it: the hammered disc ended at
                    # picks=1 after ~7 commits, while the loop guard needs picks > goal_loop_min_picks (2) --
                    # so the ONE mechanism that could have retired that goal was starved by the exact
                    # condition it exists to catch. (This is the SAME class of starvation the note above
                    # records fixing once already for a different path, 20260720.)
                    # Discriminator: a real re-orientation makes angular progress and resolves quickly; a leg
                    # re-committing the same disc for tens of seconds with no hop judged is CIRCLING. So the
                    # dedup is allowed to run only for goal_dedup_max_hold_s of continuous same-goal
                    # re-commits; past that the pick registers for real and a fresh window opens. General
                    # robustness duration (like hop_duration_s / leg_max_s), NOT a room answer.
                    if same_goal_as_last_pick:
                        if self._dedup_run_t0 is None:
                            self._dedup_run_t0 = now
                        elif (now - self._dedup_run_t0) >= self.goal_dedup_max_hold_s:
                            same_goal_as_last_pick = False     # circling, not orienting -> count this pick
                            self._dedup_run_t0 = now           # open a fresh window
                    else:
                        self._dedup_run_t0 = None              # genuine new goal / judged hop -> run resets
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
                    self.note_timeout("NO_GOAL_IDLE", event, now, loud=False)

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
                # Session 35: a forced SLAM-slow-hop in progress bypasses this for its bounded grace window.
                if self._slam_slow and not self._slam_slow_hop_active(now):
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
            if self._slam_slow and not self._slam_slow_hop_active(now):
                # SLAM started choking mid-leg -> STOP moving and let it settle before it loses the track.
                # Log the clean sub-leg flown so far (also keeps translations in the rewind history), then hold
                # and resume ADVANCE (leg_goal persists) once stable. (No speed sample on a choking frame.)
                # Session 35: a forced SLAM-slow-hop in progress bypasses this for its bounded grace window.
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
                    self._player = None
                    self._backoff_t0 = now
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
                    self._player = None
                    self._backoff_t0 = now
                    self._enter("BACKOFF", now)
                    event = "WALL contact -> clear history -> back off"
            elif reached is not None and reached <= self.goal_reach_dist:
                self._log_move("forward", fwd_val, fwd_dur)
                self._enter("SETTLE", now)
                event = f"goal reached (d={reached:.2f}) -> settle"
            elif (now - self.t_state) > self.leg_max_s:
                self._log_move("forward", fwd_val, fwd_dur)
                self._player = None
                self._backoff_t0 = now
                self._enter("BACKOFF", now)
                event = f"LEG-TIMEOUT (>{self.leg_max_s}s) -> back off"
                self.note_timeout("LEG_TIMEOUT", event, now, loud=False)
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
            # Session 46: BACKOFF's phase-timer body now lives in _step_backoff (extracted so a BACKOFF in
            # flight can own every OTHER status too, via the early dispatch near BLIND_BACKOFF above). This
            # branch is only ever reached with status == OK (every other status is caught by that early
            # return first) -- lost=False accordingly.
            active, _, event = self._step_backoff(now, lost=False, backwall_contact=backwall_contact)

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
                if self._slam_slow and not self._slam_slow_hop_active(now):
                    # SLAM choking mid-push -> log what we translated, stop, and settle before re-planning.
                    # Session 45: a forced SLAM-slow-hop in progress bypasses this for its bounded grace window,
                    # exactly as ADVANCE already did -- without this guard the forced hop could never survive its
                    # own push (see the _enter() note above; flight 20260901_112227).
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
                # Session 56 ORDERING TRAP (identical to the forced hop's, autopilot.py:3212 / :4223-4228):
                # _enter() WIPES _slam_slow_hop_deadline for any state outside ("ADVANCE", "ORIENT",
                # "PARALLAX_PUSH"), and nxt is usually SETTLE/REPLAN -- so the grace MUST be stamped AFTER
                # _enter, never before. Without it the gate's new ~3.5s release would re-divert into SLAM_HOLD
                # on the very next slow frame, trading a 15s stall for a 3.5s limit cycle.
                self._slam_slow_hop_deadline = now + self.slam_slow_hop_grace_s
                if gated:
                    event = (f"settled: SLAM window clean ({self.settle_fresh_frames} frames <"
                             f"{self.slam_slow_ms:.0f}ms) + {self.settle_gate_s:.1f}s dwell -> {nxt}")
            # SESSION 50 — the slow-SLAM DEAD BAND escape. Diagnosed off flight 20260901_172217: SETTLE sat
            # for 91.6s (3531 ticks) with plan status OK the whole time, while SLAM delivered steadily at
            # ~1995ms/frame (42 frames, min 1834 / max 2093, ZERO under slam_slow_ms). The freshness gate
            # needs settle_fresh_frames CONSECUTIVE frames under slam_slow_ms (1000ms), so at 2x the
            # threshold it was ARITHMETICALLY unreachable -- while frames arriving every ~2.2s stayed
            # comfortably inside plan_timeout_s (3.0s), so the status never went LOST either. The comment
            # above ("if SLAM stops delivering, the plan status goes STALE/LOST and the step() top diverts
            # to recovery") assumed the only failure mode was SLAM going SILENT; a pipeline that is alive,
            # tracking, and merely SLOW falls between the two thresholds and nothing catches it. Unlike
            # ORIENT/ADVANCE (which both divert to _enter_slam_hold on _slam_slow) and SLAM_HOLD/
            # TRIM_RESUME_WAIT (which both have this exact forced-resolve rescue), SETTLE had no upper
            # bound at all. It escaped only by accident, when one 3.006s inter-frame gap finally tripped
            # PLAN-LOST. Same rule as session 43's SLAM_HOLD hop, same knob: plan OK + stuck past
            # slam_slow_hop_after_s -> proceed anyway, LOUDLY.
            #
            # Reaching this handler already implies status OK (the step() top diverts every non-OK status
            # for a non-exempt state -- that is exactly how this flight left SETTLE), so no status re-check.
            # The ONE guard kept, mirroring SLAM_HOLD's: a wall clock cannot tell "slow but alive" from "a
            # total capture blackout" (perception producing NOTHING). Require at least one entry in the
            # current window to carry a real cap_ts, so a forced proceed never flies on zero live data.
            # Session 56: now a BACKSTOP, not the normal path -- the gated release just above stamps its own
            # grace on every normal exit (see the ordering-trap comment there), so this 15s bar should only
            # ever be hit by the pathological case where perception republishes a plan on its 0.5s timer
            # (status stays OK) while cap_ts itself never advances.
            elif gated and (now - self.t_state) >= self.slam_slow_hop_after_s:
                if any(cap_ts is not None for _, cap_ts in self._slam_hist):
                    waited = now - self.t_state
                    ms_txt = ("n/a" if self._slam_ms_latest is None else f"{self._slam_ms_latest:.0f}ms")
                    self._settle_to = None
                    self._enter(nxt, now)
                    # Ordering trap (identical to SLAM_HOLD's forced hop): _enter() WIPES
                    # _slam_slow_hop_deadline for any state outside ("ADVANCE", "ORIENT", "PARALLAX_PUSH"),
                    # and nxt is normally REPLAN -- so the grace MUST be stamped after _enter, not before.
                    # Without it the very next ORIENT/ADVANCE would re-divert into SLAM_HOLD on the same
                    # slow frames and the forced proceed would buy nothing.
                    self._slam_slow_hop_deadline = now + self.slam_slow_hop_grace_s
                    event = (f"SETTLE gate blocked {waited:.1f}s by slow-but-ALIVE SLAM (last {ms_txt}, gate "
                             f"needs {self.settle_fresh_frames} frames <{self.slam_slow_ms:.0f}ms) with plan "
                             f"OK -> forcing {nxt} (grace {self.slam_slow_hop_grace_s:.0f}s)")
                    self.note_timeout("SETTLE_DEADBAND", event, now, loud=False)

        elif st == "TRIM":
            # GRADUAL HEIGHT TRIM (session 14, RESTORED session 21, VERTICAL PULSE session 40). A single short,
            # full-magnitude joy_vertical pulse -- mirrors DOCK_FLOOR's already-proven pulse+settle-gate
            # primitive (session 14's "vertical thrust chokes SLAM" finding was about a CONTINUOUS/un-gated
            # push; a brief pulse followed by a real settle-gate, exactly what DOCK_FLOOR already does, is
            # SLAM-safe). No forward/lateral room needed at all -- the old ring-gate/reposition machinery (and
            # its "ring blocked -> skip trim (pray)" indefinite-retry abort, diagnosed off flight
            # 20260722_124351) is gone entirely, not patched. Sub-phases: PULSE -> full joy_vertical for
            # trim_pulse_s. WAIT -> hold until a HEALTHY frame CAPTURED >= _trim_cmd_t0 + trim_settle_s
            # (review-C async-SLAM guard: cap_ts is compared to the PULSE-COMMAND instant on the same monotonic
            # clock, so a stale pre-TRIM frame that arrives out of order can never satisfy the gate), LOG the
            # post-trim height, then re-aim (ORIENT) at the PRESERVED goal.
            if self._trim_phase is None:                        # lazy init on entry
                self._trimming = True
                self._trim_phase, self._trim_phase_t0 = "PULSE", now
                if self._trim_dir == "UP":
                    event = (f"TRIM enter (UP): sag pos_y={self._trim_sag_y:+.3f} >= HARDCODED "
                             f"trim_sag_trigger_y={self.trim_sag_trigger_y:+.3f} -> pulse up")
                else:
                    event = (f"TRIM enter (DOWN): high pos_y={self._trim_sag_y:+.3f} <= HARDCODED "
                             f"trim_high_trigger_y={self.trim_high_trigger_y:+.3f} -> pulse down")
            if self._trim_phase == "PULSE":
                active = {"joy_vertical": -1 if self._trim_dir == "UP" else 1}   # -1 = up (camera Y down)
                if (now - self._trim_phase_t0) >= self.trim_pulse_s:
                    self._trim_cmd_t0 = now                     # pulse command issued (settle-gate origin, monotonic)
                    self._trim_phase, self._trim_phase_t0 = "WAIT", now
            else:   # WAIT: hold neutral until a fresh post-trim frame, then log + exit
                # Session 54 (live-flight bug, flight 20260902_155916): dropped the `not self._slam_slow`
                # conjunct that used to gate `healthy` here. Session 52 removed the same conjunct from
                # TRIM's *trigger* above, arguing pos_y is the slowest-varying quantity SLAM publishes so a
                # stale-but-slow reading is still trustworthy -- and its comment claimed only the trigger
                # was a height-reading gate, the rest guarded MOTION/HEADING. That claim missed this one:
                # this IS a height-reading gate too (its only consumer is the "post pos_y=" log line below).
                # With SLAM solving at a flat ~2700ms this flight, `_slam_slow` was true on EVERY frame, so
                # `ready` could never fire -- and TRIM has no other exit, so it hovered 73.9s until the
                # operator killed the run. The real freshness guarantee is `cap_ts >= _trim_cmd_t0 +
                # trim_settle_s` below: it proves the frame was CAPTURED after the pulse settled, on the
                # same monotonic clock -- `_slam_slow` says only how long the solve took, not whether pos_y
                # is right.
                cap_ts = plan.get("cap_ts")
                healthy = plan.get("plan_valid") and plan.get("pos_y") is not None
                ready = (self._trim_cmd_t0 is not None and cap_ts is not None
                         and cap_ts >= self._trim_cmd_t0 + self.trim_settle_s and healthy)
                if ready:
                    y_after = float(plan["pos_y"])
                    # _desired_y is only set post-CALIB_VERIFY (session 44's hardcoded trigger can fire before
                    # that) -- NO SILENT FALLBACK: say so explicitly rather than crash formatting None.
                    desired_str = (f"{self._desired_y:+.3f}, delta_to_desired={y_after - self._desired_y:+.3f}"
                                   if self._desired_y is not None else "n/a (never calibrated)")
                    msg = (f"TRIM done ({self._trim_dir}): post pos_y={y_after:+.3f} (desired {desired_str})")
                    event = self._trim_exit(now, plan, msg)
                    return active, self.state, event
                # Session 54: bounded forced exit, belt-and-suspenders alongside the fix above (a DIFFERENT
                # unsatisfiable condition -- e.g. plan_valid staying False, or cap_ts never advancing --
                # could still park WAIT forever otherwise). Mirrors TRIM_RESUME_WAIT's session-44 rescue
                # verbatim: same knob (no new one for a fourth name of the same idea), same
                # has_any_capture blackout guard (a wall clock must never paper over perception producing
                # NOTHING, only genuine slowness).
                elif self._trim_cmd_t0 is not None:
                    waited = now - self._trim_cmd_t0
                    has_any_capture = any(c is not None for _, c in self._slam_hist)
                    if waited >= self.slam_slow_hop_after_s and has_any_capture:
                        y_txt = (f"{plan['pos_y']:+.3f}" if plan.get("pos_y") is not None else "unavailable")
                        msg = (f"TRIM done ({self._trim_dir}): FORCED after {waited:.1f}s waiting for a "
                               f"post-trim frame (last pos_y={y_txt})")
                        event = self._trim_exit(now, plan, msg)
                        self.note_timeout("TRIM_WAIT_FORCED", event, now, loud=False)
                        return active, self.state, event

        elif st == "TRIM_RESUME_WAIT":
            # Session 28: hold neutral until the settle-gate proves a fresh post-TRIM frame exists, then hand
            # off to _trim_resolve_resume (re-aim at the preserved goal, or fall back to SETTLE->REPLAN if it
            # died — e.g. got permanently blacklisted — while TRIM was interrupting the leg). See _trim_exit
            # for why this wait exists instead of resolving instantly off a possibly-stale pose.
            # Session 44 (live-flight bug, flight 20260901_103028): this settle-gate had NO escape for
            # sustained SLAM slowness -- it sat here ~35s while SLAM solves ran 900-1500ms (never accumulating
            # settle_fresh_frames consecutive sub-slam_slow_ms frames), plan status staying OK the whole time,
            # until it finally degraded to PLAN-LOST. SLAM_HOLD already has exactly this rescue
            # (slam_slow_hop_after_s, sessions 35/43) on the same reasoning -- a slow settle-gate is a
            # perception THROUGHPUT signal, not evidence the pose is wrong -- but it was never extended to
            # TRIM_RESUME_WAIT. Give it the identical bounded give-up, reusing the same knob and the same
            # has_any_capture guard (never force off a total capture blackout, only genuine slowness).
            # _trim_resolve_resume already degrades gracefully to SETTLE->REPLAN if the plan's pose is
            # unavailable, so forcing it here on a merely-slow (not dead) plan is safe.
            if self._settle_gate_poll(now):
                event = self._trim_resolve_resume(now, plan)
                # Session 56 ORDERING TRAP (identical to the forced hop's, autopilot.py:3212 / :4223-4228):
                # _enter() WIPES _slam_slow_hop_deadline for any state outside ("ADVANCE", "ORIENT",
                # "PARALLAX_PUSH"), and _trim_resolve_resume has already entered ORIENT/SETTLE by the time it
                # returns here -- so the grace MUST be stamped AFTER the call, never before. Without it the
                # gate's new ~3.5s release would re-divert into SLAM_HOLD on the very next slow frame, trading
                # a 15s stall for a 3.5s limit cycle.
                self._slam_slow_hop_deadline = now + self.slam_slow_hop_grace_s
            else:
                waited = now - (self._settle_gate_t0 if self._settle_gate_t0 is not None else self.t_state)
                has_any_capture = any(cap_ts is not None for _, cap_ts in self._slam_hist)
                # Session 56: now a BACKSTOP, not the normal path -- the gated release just above stamps its
                # own grace on every normal exit (see the ordering-trap comment there), so this 15s bar should
                # only ever be hit by the pathological case where perception republishes a plan on its 0.5s
                # timer (status stays OK) while cap_ts itself never advances.
                if waited >= self.slam_slow_hop_after_s and has_any_capture:
                    event = self._trim_resolve_resume(now, plan)
                    if event:
                        event = f"{event} (forced after {waited:.1f}s of slow settle-gate)"
                        self.note_timeout("TRIM_RESUME_FORCED", event, now, loud=False)

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
                self.note_timeout("HOME_CAP", event, now, loud=False)
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
            else:   # ADVANCE: push forward toward the aim for a bounded sub-leg, then SETTLE -> re-aim (PLAN)
                # No clearance stand-off/BACKOFF here (operator's call, session 39): homing always turns to
                # face the true origin before advancing, so a properly-oriented leg isn't expected to run into
                # a wall the way frontier-exploration's ADVANCE can — that stage keeps its own BACKOFF.
                moved = self._dist(pos, self._home_adv_start_pos)
                reaim = moved is not None and moved >= self.goal_reach_dist
                adv_timeout = (now - self._home_adv_t0) >= self.leg_max_s
                if reaim or adv_timeout or self._slam_slow:
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
            # take-off. Open-loop turns (clamped to <=turn_step_deg per turn, but NOT quantized to it — the
            # open-loop recipe already scales continuously; forcing a fixed step here made the residual error
            # overshoot side-to-side forever whenever it landed near half a step, see the 20260720 ping-pong)
            # with a SETTLE between (no spin on a stale pose), converging to within orient_home_tol_deg, then ->
            # HOME_REFINE. Bounded by orient_home_max_s -> proceed anyway (VISIBLE; no silent fallback). Clears
            # the flying-height altitude lock on the handoff so the descent can't be fought / re-inflated. If no
            # take-off heading was ever captured, skip straight to HOME_REFINE (VISIBLE).
            if self._orient_home_phase is None:
                self._orient_home_phase, self._orient_home_t0 = "PLAN", now
            if self._takeoff_heading is None:
                self._orient_home_phase, self._orient_home_t0 = None, None
                self._home_refine_phase = None
                self._enter("HOME_REFINE", now)
                event = "ORIENT_HOME: no take-off heading recorded -> HOME_REFINE"
            elif (now - self._orient_home_t0) >= self.orient_home_max_s:
                self._orient_home_phase, self._orient_home_t0 = None, None
                self._home_refine_phase = None
                self._enter("HOME_REFINE", now)
                event = (f"ORIENT_HOME orient_home_max_s ({self.orient_home_max_s:.0f}s) cap — couldn't converge "
                         "on take-off heading, proceeding HERE (VISIBLE; no silent fallback) -> HOME_REFINE")
                self.note_timeout("ORIENT_HOME_CAP", event, now, loud=False)
            elif self._orient_home_phase == "PLAN":
                if plan.get("plan_valid") and plan.get("heading_deg") is not None:
                    be = ((self._takeoff_heading - float(plan["heading_deg"]) + 180.0) % 360.0) - 180.0
                    if abs(be) <= self.orient_home_tol_deg:     # within tolerance of the take-off heading -> refine position
                        self._orient_home_phase, self._orient_home_t0 = None, None
                        self._home_refine_phase = None
                        self._enter("HOME_REFINE", now)
                        event = f"ORIENT_HOME: facing take-off heading (err {self._fmt(be)}) -> HOME_REFINE"
                    else:
                        theta = be
                        if self.clamp_leg_turn:
                            theta = max(-self.turn_step_deg, min(self.turn_step_deg, theta))
                        self._player = self._build_turn(theta)
                        self._orient_home_phase = "TURN"
                        event = f"ORIENT_HOME: turn {theta:+.1f} deg toward take-off heading (err {self._fmt(be)})"
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

        elif st == "HOME_REFINE":
            # Postlude leg 1c: now that the drone FACES the take-off heading (ORIENT_HOME), nudge the residual
            # POSITION closer to the true origin using short, FIXED-magnitude push pulses — not a continuous
            # ADVANCE — so the final resting spot is tighter than home_reach_dist without re-opening the
            # turning machinery. Each cycle: read the live bearing-to-origin in the CURRENT body frame and pick
            # ONE of forward/backward/strafe-left/strafe-right (full throttle; forward/back RAMP as usual,
            # strafe is never ramped — same as every other strafe in this codebase), play it, then a real
            # SETTLE (6 fresh frames) before re-measuring — never chains pushes blind. Bounded by
            # home_refine_max_s -> proceed to dock anyway (VISIBLE; no silent fallback), same idiom as
            # home_max_s/orient_home_max_s.
            if self._home_refine_phase is None:
                self._home_refine_phase, self._home_refine_t0 = "PLAN", now
            pos = plan.get("pos")
            reached = self._dist(pos, [0.0, 0.0])
            if reached is not None and reached <= self.home_fine_reach_dist:
                self._home_refine_phase, self._home_refine_t0 = None, None
                self._dock_phase = None
                self.target_altitude_y = None       # drop the flying-height lock before descending
                self._enter("DOCK_FLOOR", now)
                event = f"HOME_REFINE: within {self.home_fine_reach_dist:.2f}u of origin (d={reached:.2f}) -> DOCK_FLOOR"
            elif (now - self._home_refine_t0) >= self.home_refine_max_s:
                self._home_refine_phase, self._home_refine_t0 = None, None
                self._dock_phase = None
                self.target_altitude_y = None
                self._enter("DOCK_FLOOR", now)
                event = (f"HOME_REFINE home_refine_max_s ({self.home_refine_max_s:.0f}s) cap — couldn't tighten "
                         f"within {self.home_fine_reach_dist:.2f}u (d={self._fmt(reached)}), proceeding HERE "
                         "(VISIBLE; no silent fallback) -> DOCK_FLOOR")
                self.note_timeout("HOME_REFINE_CAP", event, now, loud=False)
            elif self._home_refine_phase == "PLAN":
                if plan.get("plan_valid") and pos is not None and plan.get("heading_deg") is not None:
                    bearing = math.degrees(math.atan2(0.0 - pos[0], 0.0 - pos[1]))   # 0=+Z, +90=+X (matches homing)
                    be = ((bearing - float(plan["heading_deg"]) + 180.0) % 360.0) - 180.0   # wrap to (-180,180]
                    if abs(be) <= 45.0:
                        push, dur, dirn = {"trigger": 1.0}, self.home_refine_fwd_s, "forward"
                    elif abs(be) >= 135.0:
                        push, dur, dirn = {"reverse": 1.0}, self.home_refine_fwd_s, "backward"
                    elif be > 0.0:
                        push, dur, dirn = {"joy_horizontal": 1.0}, self.home_refine_strafe_s, "strafe_right"
                    else:
                        push, dur, dirn = {"joy_horizontal": -1.0}, self.home_refine_strafe_s, "strafe_left"
                    self._player = RecipePlayer([dict(push, duration_s=dur)], name=f"refine_{dirn}")
                    self._home_refine_phase = "PUSH"
                    event = f"HOME_REFINE: push {dirn} {dur:.2f}s (d={reached:.2f}, err {self._fmt(be)}) toward origin"
                else:
                    event = "HOME_REFINE: pose invalid -> hold (wait for SLAM)"
            elif self._home_refine_phase == "PUSH":
                active, pdone = self._player.fields(now)
                if pdone:
                    self._player = None
                    self._home_refine_phase = "SETTLE"
                    self._settle_begin(now)
            else:   # SETTLE -> re-measure distance/bearing to origin (push again or dock)
                sdone, _cap = self._settle_poll(now, plan, require_fast=False,
                                                min_frames=self.settle_fresh_frames, max_hold_s=None)
                if sdone:
                    self._home_refine_phase = "PLAN"

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
                self.note_timeout("DOCK_CAP", event, now, loud=False)
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
                    self._settle_begin(now)
            else:   # REST: neutral (momentum bleeds) while a REAL settle-gate (6 fresh frames, the same
                    # primitive every other maneuver-loop in this file uses) proves the pose we're about to
                    # read is a genuine POST-pulse frame — not whatever `plan` happened to hold when a fixed
                    # timer expired (the same class of stale-frame gap session 24's settle-gate rewrite fixed
                    # everywhere else; DOCK_FLOOR, added later mirroring ASCEND, never got it).
                sdone, _cap = self._settle_poll(now, plan, require_fast=False,
                                                min_frames=self.settle_fresh_frames, max_hold_s=None)
                if sdone:
                    valid = plan.get("plan_valid") and plan.get("pos_y") is not None and not self._slam_slow
                    if not valid:
                        self._settle_begin(now)           # not trustworthy yet -> re-open the gate, keep waiting (dock_max_s backstops)
                        event = "dock: pose invalid/slow after settle -> re-wait for SLAM"
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
_RECOVERY_STATES = {"HOLD_LOST", "REWIND", "FALLBACK", "STUCK", "WARMUP", "VISUAL_RECOVERY"}

# Post-mission ending states. A plan loss WHILE in one of these diverts to the dedicated POSTLUDE_LOST_HOLD
# (mirror of CALIB_LOST_HOLD) instead of the generic recovery, so the ending survives a SLAM loss and resumes.
# DONE included (session 39 fix): without it, a loss after mission-complete fell through to the ordinary
# explore recovery path, which on recovering forces a REPLAN -> resurrects the whole explore FSM (BUMP/
# BACKOFF/BLACKLIST/TRIM chasing a stale goal) instead of quietly resuming DONE (diagnosed off flight
# 20260721_233244: DONE at 23:53:58.784, PLAN-LOST at 23:54:02.747, HOLD_LOST -> SLAM_HOLD -> REPLAN ->
# BUMP/BACKOFF against corner [-1.6, 8.1] from 23:54:13 on, TRIM-DOWN at 23:55:03 with pos_y already near
# ceiling territory). DONE has no sub-phase, so _step_postlude_lost's generic "_enter(resume, now)" resume
# path (no special-case needed) just re-enters DONE — _done_logged is already True, so nothing re-announces.
POSTLUDE_STATES = {"RETURN_TO_ORIGIN", "ORIENT_HOME", "HOME_REFINE", "DOCK_FLOOR", "LOW_STANDOFF", "DONE"}
# Postlude states where the flying-height altitude lock must be OFF (the descent + standby) — clearing
# target_altitude_y on DOCK_FLOOR entry could otherwise be re-cached next tick and re-inflate a floor-level drone.
# RETURN_TO_ORIGIN / ORIENT_HOME / HOME_REFINE are NOT here: they home + orient + reposition AT altitude, lock on.
_POSTLUDE_NOLOCK = {"DOCK_FLOOR", "LOW_STANDOFF", "DONE", "POSTLUDE_LOST_HOLD"}

# SESSION 18: the old MAPPING_ALT_STATES state-gate for the altitude baseline is retired. The baseline now
# measures the live pos_y once per FRESH SLAM frame in ANY state, gated only by `_height_calibrated` (first
# calibration done) and NOT `_calib_active` (frozen during a calibration) — see the ingest at ExploreController.step.

# States from which a gradual-height TRIM may fire (session 14, RESTORED session 21; SLAM_HOLD ADDED
# session 56). SETTLE is the pre-REPLAN / settled-SLAM_HOLD rest point; ADVANCE is the operator's explicit
# request. TRIM itself is absent (no re-entry).
# Session 56: flight 20260902_165340 spent 17:16:36 -> 17:20:51 entirely in HOLD_LOST/SLAM_HOLD, and TRIM
# fired 3ms after ADVANCE was finally reached -- the WHITELIST, not the trim threshold, was starving TRIM.
# pos_y is the slowest-varying quantity SLAM publishes (session 52's reasoning for dropping the
# not-_slam_slow gate applies identically here), so a drone parked in SLAM_HOLD can and should still be
# height-corrected. Explicitly NOT added:
#   - HOLD_LOST: no fresh pos_y is available there at all (it's the blind twin of SLAM_HOLD), and TRIM's
#     own WAIT phase needs a post-pulse cap_ts to ever exit -- one never arrives during a loss. Whitelisting
#     it would reproduce session 54's 73.9s TRIM hang exactly, just from the other end.
#   - every mid-maneuver state (ORIENT/ADVANCE-hop/PARALLAX_PUSH/BACKOFF/...): a trim must never interrupt
#     a maneuver already in flight; SLAM_HOLD is a rest point, those are not.
_TRIM_TRIGGER_STATES = {"SETTLE", "ADVANCE", "SLAM_HOLD"}

# Session 45: states where the STATE-INDEPENDENT "goal already reached" check must NOT fire (see the long
# note at its site in step()). "SETTLE" is the critical one -- _enter("SETTLE") resets _settle_ok/_settle_t0,
# and since the drone does not move during a settle the reached condition stays true, so firing there would
# re-open the settle every tick and hang forever. "REPLAN" is a one-tick pass-through that re-reads the goal
# anyway; the TRIM pair must never be interrupted mid-pulse; the recovery/postlude sets own their own
# convergence and must never act on a frozen (blind) pose.
_REACHED_EXCLUDED_STATES = ({"SETTLE", "REPLAN", "TRIM", "TRIM_RESUME_WAIT", "WAIT", "ARM", "TAKEOFF",
                             "ASCEND", "DESCEND", "CALIB_VERIFY", "BASELINE_NUDGE", "CALIBRATING_HEIGHT",
                             "CALIB_LOST_HOLD", "CALIB_ESCAPE", "SLAM_STEPBACK", "POSTLUDE_LOST_HOLD"}
                            | _RECOVERY_STATES | POSTLUDE_STATES)

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


VISREC_WINDOW = "F_LKG | LIVE - visual recovery"


def _visrec_should_cache_reference(status: str, plan: dict) -> bool:
    """Session 56: may F_LKG be (re)stored on this tick?

    The reference must be pinned to a CURRENT plan, not a merely once-valid one. `plan_for_step`
    is `last_plan` of ANY age (run_explore :5106), and PLAN-LOST is a pure age verdict
    (_plan_status, :678-691) -- the stale plan still carries plan_valid=True. So the old
    `plan_valid`-only gate fired on EVERY tick of a PLAN-LOST episode, re-storing the reference
    the whole time. Flight 20260902_165340: combined with the ring age-out, F_LKG became the
    CURRENT LIVE FRAME, re-stored ~35x/s. Fingerprint at 17:00:32.140 -- `inliers=732
    contained=True planar_like=True scale=1.00`, a frame matched against itself, which is exactly
    the "too close -> BACKOFF" evidence at :2658-2662.
    """
    return status == "OK" and bool(plan.get("plan_valid"))


def _visrec_should_match(ctrl, *, needs_match, has_frame, loss_edge, moved_since_match,
                         memo, memo_age_s, now=None, status=None):
    """Session 51: does THIS tick have to actually COMPUTE a visual match, or can it reuse `memo`?

    Pure decision function (no I/O, no state) so it is unit-testable -- `run_explore` itself is never
    entered by the self-test suite. Returns True to compute a fresh match, False to reuse `memo` (which
    may be None, meaning simply no match this tick).

    GATE A -- exact, zero staleness. `needs_match` is run_explore's ORIGINAL status condition and is kept
    as-is; `ctrl.wants_visual_match(now=now, status=status)` narrows it to ticks where a consumer can
    actually read the result -- session 57 adds a third such consumer (`_step_lost_recovery` on a matured
    PLAN-LOST/NO-PLAN episode), which is why `now`/`status` thread through here too. The AND is
    load-bearing in both directions: see `wants_visual_match`'s caller contract for why the predicate must
    never replace the status condition.

    GATE B -- bounded staleness. Even with a live consumer, the answer cannot change while the drone holds
    still against a frozen F_LKG (session 48's own grace invariant), so an unchanged value is reused until
    it ages past `visrec_match_min_interval_s`. Four conditions FORCE a fresh compute regardless:
      • `loss_edge`      -- loss-instant evidence must come from THIS episode, never the previous one.
      • probe in MATCH   -- it re-matches AFTER a turn; a pre-turn result is a different view entirely.
      • `moved_since_match` -- any commanded motion (e.g. a mid-loss BLIND_BACKOFF reverse) changes the view.
      • `memo is None`   -- nothing to reuse (a new F_LKG clears it at the call site).
    """
    if not (needs_match and has_frame and ctrl.wants_visual_match(now=now, status=status)):
        return False
    if loss_edge or ctrl._visrec_phase == "MATCH" or moved_since_match or memo is None:
        return True
    return memo_age_s >= ctrl.visrec_match_min_interval_s


def _visrec_debug_sink(ctrl, diag, canvas, save, stamp, saved_count):
    """Show `canvas` in the LKG debug window and, when `save` is True, write it under
    OUTPUT/diag/<flight_ts>_visrec/<stamp>.png.

    Args:
        ctrl (ExploreController): owns the two degradation flags + the save cap.
        diag (AutopilotLog): supplies `ts` / `diag_dir`; a disabled log (no --log) means no saving.
        canvas (np.ndarray): BGR image from VisualMatch.debug_image.
        save (bool): True only at a decision instant (see run_explore's probe block).
        stamp (str): "HH-MM-SS_mmm", the filename stem.
        saved_count (int): canvases written so far this flight (cap gauge).

    Returns:
        str | None: the timeline-facing RELATIVE path ("<ts>_visrec/<stamp>.png") when a file was
        written this call, else None. The path is relative to OUTPUT/diag so flight_replay.py's HTML,
        which lives in that same directory, can reference it directly.

    The window half and the save half fail INDEPENDENTLY (CLAUDE.md NO-SILENT-FALLBACK): a display
    failure must not stop the file evidence (which is the half that survives the flight), and a disk
    failure must not close the window. Each sets its own flag, logs one CRITICAL line, and is not
    retried."""
    if ctrl.visrec_debug_window and not ctrl.visrec_window_failed:
        try:
            cv2.imshow(VISREC_WINDOW, canvas)
            cv2.waitKey(1)
            ctrl.visrec_window_open = True   # Session 58: window is loss-scoped -- track that it is up
        except Exception as exc:
            ctrl.visrec_window_failed = True
            line = (f"*** CRITICAL: LKG debug window unavailable ({exc}) -> window DISABLED for this "
                     f"flight; PNG evidence continues ***")
            print(line, flush=True)
            diag.line(line)

    rel = None
    if save and not ctrl.visrec_save_failed and diag.diag_dir and saved_count < ctrl.visrec_save_max:
        try:
            sub_dir = f"{diag.ts}_visrec"
            out_dir = os.path.join(diag.diag_dir, sub_dir)
            os.makedirs(out_dir, exist_ok=True)
            fname = f"{stamp}.png"
            ok = cv2.imwrite(os.path.join(out_dir, fname), canvas)
            if not ok:
                raise IOError(f"cv2.imwrite returned False for {fname}")
            rel = f"{sub_dir}/{fname}"
        except Exception as exc:
            ctrl.visrec_save_failed = True
            line = (f"*** CRITICAL: LKG evidence save failed ({exc}) -> saving DISABLED for this "
                     f"flight; debug window continues ***")
            print(line, flush=True)
            diag.line(line)
    elif save and not ctrl.visrec_save_failed and diag.diag_dir and saved_count >= ctrl.visrec_save_max:
        if not ctrl.visrec_cap_logged:
            ctrl.visrec_cap_logged = True
            line = f"*** LKG evidence cap reached (visrec_save_max={ctrl.visrec_save_max}) -> no further canvases saved ***"
            print(line, flush=True)
            diag.line(line)
    return rel


def _visrec_close_window(ctrl, diag):
    """Close the LKG debug window at the end of a loss episode. No-op unless one is open.

    Session 58: the window is now loss-scoped (see MISSION CONTEXT finding 2 -- the old idle
    refresh kept it open for the whole flight). Closing it on the loss->recovered edge makes its
    on-screen appearance itself a SIGNAL that a loss outlived the grace, instead of a fixture that
    is always there and says nothing."""
    if not ctrl.visrec_window_open:
        return
    try:
        cv2.destroyWindow(VISREC_WINDOW)
        ctrl.visrec_window_open = False
    except Exception as exc:
        ctrl.visrec_window_failed = True
        ctrl.visrec_window_open = False
        line = (f"*** CRITICAL: LKG debug window close failed ({exc}) -> window DISABLED for this "
                 f"flight; PNG evidence continues ***")
        print(line, flush=True)
        diag.line(line)


def run_explore(cfg, stop_event=None, log=False, no_takeoff=False):
    """Bus wrapper around ExploreController: SUB frames (:frame_bus_port, flow WALL detector) + the
    explore plan (TOPIC_PLAN on :perception_state_port); PUB TOPIC_CONTROL. Enable on io_bridge with
    'm' (any manual flight key aborts to manual). Arms + takes off automatically (the prelude) unless
    `no_takeoff`. Needs io_bridge + perception_worker running."""
    ascend_cmd = int(cfg["autonomy"]["ascend_cmd"])
    e = (cfg["autonomy"].get("explore") or {})
    plan_timeout_s = float(e.get("plan_timeout_s", 2.0))
    # Session 55 (crash survivability): a hard machine bugcheck gives no shutdown path, so both of
    # these must happen PERIODICALLY during the flight, not only in `finally` (see diag.yaml).
    _d = (cfg.get("diag") or {})
    timeline_map_period_s = float(_d.get("timeline_map_period_s", 10.0))
    log_fsync_period_s = float(_d.get("log_fsync_period_s", 2.0))
    detector = detector_from_cfg(cfg)
    ctrl = ExploreController(cfg, no_takeoff=no_takeoff)
    # Session 35 ALT: only build the visual-recovery probe (SIFT model load + per-tick reference cache) when
    # the config flag is actually on -- an idle unused probe still shouldn't pay SIFT's init cost.
    visrec_probe = VisualRecoveryProbe(
        min_inliers=ctrl.visrec_min_inliers, planar_inlier_ratio=ctrl.visrec_planar_inlier_ratio,
        contain_margin_frac=ctrl.visrec_contain_margin_frac,
        # Session 57: inlier-SPREAD direction verdict thresholds (C4 in _step_lost_recovery, chunk 3+).
        size_ratio_hi=ctrl.visrec_size_ratio_hi, size_ratio_lo=ctrl.visrec_size_ratio_lo,
        size_min_inliers=ctrl.visrec_size_min_inliers) if ctrl.use_visual_recovery_on_stale else None

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
    last_map_emit_t = 0.0    # session 55: t_mono of the last periodic map-backdrop timeline record
    last_fsync_t = 0.0       # session 55: t_mono of the last periodic diag fsync
    last_plan = None
    last_plan_t = time.monotonic()
    last_rec_frame = None
    enabled = False
    was_enabled = False
    announced_wait = warned_no_auto = False
    _auto_off_t = None   # session 52 (chunk 9): timestamp of the ON->OFF edge, for the pause-duration log line
    last_status = None
    last_cmd_key = None
    last_label = None
    last_visrec_label = None    # dedup/throttle for [VISREC] match-verdict log lines (session 35 ALT)
    last_visrec_log = 0.0
    visrec_saved = 0            # canvases written this flight (visrec_save_max gauge) (session 49)
    visrec_prev_status = None   # previous tick's status, for the loss-EDGE decision instant (session 49)
    visrec_episode_saved = False # Session 58: has THIS loss episode already saved a PNG? (see C5 --
                                 # chunk 1 makes loss_edge ticks match-free, so `decision` can no longer
                                 # key off loss_edge alone)
    # Session 51 GATE B: memoised VisualMatch + the invalidation trackers. The answer cannot change while
    # the drone holds still against a frozen F_LKG, so an unchanged value is reused rather than recomputed
    # ~32x/second. `visrec_moved_since_match` is the motion guard -- set on ANY non-empty published command
    # vector, cleared when a match is actually computed.
    visrec_memo = None
    visrec_memo_t = 0.0
    visrec_moved_since_match = False
    visrec_matches = 0          # total real matches computed this flight (waste-reduction gauge)
    # Session 52: F_LKG must be the EXACT frame SLAM's plan was computed from, not just whatever frame was
    # live when plan_valid ticked true -- this flight measured up to 14.9s SLAM solve latency, so those can
    # differ by many frames. Ring of trailing (frame_id, frame) pairs, newest last, looked up by the plan's
    # own frame_id at the update_reference call site below. 0-length ring = NO SILENT FALLBACK: stated LOUD
    # once, not silently degraded.
    _lkg_ring = (collections.deque(maxlen=ctrl.visrec_lkg_ring_len)
                 if ctrl.visrec_lkg_ring_len > 0 else None)   # (frame_id, frame) newest last
    _lkg_ageout_last_log = 0.0      # session 56: rate-limit for the LOUD age-out notice below
    if _lkg_ring is None:
        _line = ("*** F_LKG running in LEGACY LIVE-FRAME mode (visrec_lkg_ring_len<=0) -- the visual "
                 "recovery reference is the tick's live frame, not the frame SLAM's plan actually describes; "
                 "this degrades under slow SLAM solves ***")
        print(_line, flush=True)
        diag.line(_line)
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
            # Session 52 (chunk 6): panel telemetry, built fresh each publish from the controller's live
            # flight-level counters (Contracts 1.6/1.7) -- last_timeout is LATCHED, never consumed, so
            # age_s is recomputed every tick from its stamped 't' rather than popped once.
            _lt = ctrl.last_timeout
            notice = (None if _lt is None else
                     {"kind": _lt["kind"], "text": _lt["text"], "age_s": now - _lt["t"]})
            slam_hold = {
                "now_s": (now - ctrl.t_state) if ctrl.state == "SLAM_HOLD" else None,
                "deadline_s": ctrl.slam_slow_hop_after_s,
                "entries": ctrl._slam_hold_entries,
                "total_s": ctrl._slam_hold_total_s,
            }
            # Session 56: F_LKG age-out telemetry -- mirrors slam_hold's shape/placement exactly.
            visrec_lkg = {
                "src": visrec_probe._lkg_src if visrec_probe is not None else "none",
                "ageouts": ctrl.visrec_lkg_ageouts,
                "degraded": ctrl.visrec_lkg_degraded,
            }
            pub.publish(frame_bus.TOPIC_CONTROL,
                        _full_vector(active, seq, now, state, ctrl.target_altitude_y,
                                    state_since_s=now - ctrl.t_state,
                                    slam_hold=slam_hold, notice=notice, visrec_lkg=visrec_lkg))
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
            # Session 55: periodic fsync (NOT per-record — see AutopilotLog.fsync) so a hard reboot loses
            # at most log_fsync_period_s of the already-flush()'d logs, not the whole flight.
            if log and log_fsync_period_s > 0 and (now - last_fsync_t) >= log_fsync_period_s:
                diag.fsync()
                last_fsync_t = now
            visrec_last_saved_rel = None   # session 49: only the tick that ACTUALLY writes a PNG carries the path
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
                # Session 52: feed the F_LKG-by-identity ring. Dedup on frame_id -- the frame bus can repeat
                # an id across ticks (same source the SLAM_TRACKER drain elsewhere dedups on).
                if _lkg_ring is not None and frame is not None and meta.get("frame_id") is not None:
                    _fid_new = meta.get("frame_id")
                    if not _lkg_ring or _lkg_ring[-1][0] != _fid_new:
                        _lkg_ring.append((_fid_new, frame))
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
                    _auto_off_t = now
                    # Session 52 (chunk 9): reset_leg() below silently clears a pile of in-flight recovery
                    # state; without this line the log shows the FALLBACK sweep restarting at cycle 1 (or
                    # similar) with no explanation of why.
                    diag.line(f"{_rec_prefix(last_rec_frame)} [autopilot][explore] autonomy OFF -> PAUSED: "
                              f"reset_leg() clears the FALLBACK sweep budget/cycle, _recovering, "
                              f"_blind_contact_reacts, the post-back-off re-solve gate, _loss_episode_t0 "
                              f"and _visrec_probe_armed.")
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
                _pause_note = f"paused {now - _auto_off_t:.1f}s" if _auto_off_t is not None else "pause duration unknown"
                diag.line(f"{_rec_prefix(last_rec_frame)} [autopilot][explore] autonomy LIVE -> resuming ({_pause_note}): "
                          f"recovery state (FALLBACK sweep, _recovering, wedge/back-off gates, loss/probe flags) "
                          f"was reset by reset_leg() on pause.")
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

            # ---- visual recovery (session 35 ALT): cache F_LKG every tick (cheap -- a copy); only run the
            # SIFT match when it could matter (the loss-instant check, or while the 15° probe is actively
            # re-testing after a turn) -- no reason to pay match cost on every healthy tracking frame. ----
            visual_match = None
            if visrec_probe is not None:
                if frame is not None:
                    # Session 52: resolve the TRUE reference frame by identity, not by tick freshness -- see
                    # the ring comment near this loop's top. `tracked` still gates WHETHER to store (session
                    # 34/35 plan_valid boundary, unchanged); only WHICH frame gets stored changes here.
                    tracked = _visrec_should_cache_reference(status, plan_for_step)
                    ref_frame, ref_src = frame, "live"
                    if tracked and _lkg_ring is not None:
                        _pfid = plan_for_step.get("frame_id")
                        if _pfid is not None:
                            _hit = next((f for fid, f in reversed(_lkg_ring) if fid == _pfid), None)
                            if _hit is not None:
                                ref_frame, ref_src = _hit, f"slam:{_pfid}"
                            else:
                                # Session 56: NO SILENT FALLBACK, and no FAKE reference either. Substituting
                                # the LIVE frame here made F_LKG self-match (17:00:32.140: inliers=732
                                # scale=1.00 contained=True), manufacturing the exact "too close" evidence
                                # that drives BACKOFF. An OLD true reference beats a fake current one -- so
                                # skip the store entirely and keep whatever F_LKG is already held. Loud,
                                # counted, rate-limited, and exposed.
                                tracked = False
                                ctrl.visrec_lkg_ageouts += 1
                                ctrl.visrec_lkg_degraded = True
                                if (now - _lkg_ageout_last_log) >= ctrl.visrec_lkg_ageout_log_interval_s:
                                    _lkg_ageout_last_log = now
                                    _oldest = _lkg_ring[0][0] if _lkg_ring else "--"
                                    _newest = _lkg_ring[-1][0] if _lkg_ring else "--"
                                    _line = (f"*** F_LKG AGE-OUT #{ctrl.visrec_lkg_ageouts}: plan "
                                             f"frame_id={_pfid} is outside the ring (holding ids "
                                             f"{_oldest}..{_newest}, len={len(_lkg_ring)}/"
                                             f"{ctrl.visrec_lkg_ring_len}) -> KEEPING the previous reference "
                                             f"(src={visrec_probe._lkg_src}); the live frame is NOT a valid "
                                             f"F_LKG. SLAM solve latency exceeds the ring depth. ***")
                                    print(_line, flush=True)
                                    diag.line(_line)
                    if visrec_probe.update_reference(ref_frame, tracked, src=ref_src):
                        visrec_memo, visrec_memo_t = None, 0.0   # new F_LKG -> the memo describes nothing
                # Session 49's decision-instant edge, hoisted here: session 51's GATE B needs `loss_edge`
                # as a force condition, so it must be computed BEFORE the match decision, not inside it.
                loss_now = status in ("PLAN-LOST", "NO-PLAN", "PLAN-STALE")
                loss_edge = loss_now and visrec_prev_status not in ("PLAN-LOST", "NO-PLAN", "PLAN-STALE")
                # Session 58: the mirror image of loss_edge -- a loss episode just ENDED. The window is
                # now loss-scoped (MISSION CONTEXT finding 2 -- the old idle refresh kept it open for the
                # whole flight), so this is where it closes; its reappearance next episode is a SIGNAL a
                # loss outlived the grace, not a fixture that is always on screen and says nothing.
                recover_edge = (not loss_now) and visrec_prev_status in ("PLAN-LOST", "NO-PLAN", "PLAN-STALE")
                if recover_edge and ctrl.visrec_debug_window:
                    _visrec_close_window(ctrl, diag)
                if loss_edge:
                    visrec_episode_saved = False   # Session 58 (C5): new episode, no PNG written for it yet
                needs_match = (loss_now or ctrl.state == "VISUAL_RECOVERY")
                # ---- SESSION 51 gates A + B (see _visrec_should_match for the full rationale) ----
                do_match = _visrec_should_match(
                    ctrl, needs_match=needs_match, has_frame=(frame is not None), loss_edge=loss_edge,
                    moved_since_match=visrec_moved_since_match, memo=visrec_memo,
                    memo_age_s=(now - visrec_memo_t), now=now, status=status)
                if not do_match and needs_match and visrec_memo is not None and ctrl.wants_visual_match(
                        now=now, status=status):
                    visual_match = visrec_memo          # unchanged answer -- reuse, do not recompute
                if do_match:
                    banner = f"{ctrl.state} / {status}"
                    visual_match = visrec_probe.match(frame, debug=ctrl.visrec_debug_window, banner=banner)
                    visrec_memo, visrec_memo_t = visual_match, now
                    visrec_moved_since_match = False
                    visrec_matches += 1
                    vlabel = (visual_match.has_lkg, visual_match.matched, visual_match.contained,
                             visual_match.planar_like, round(visual_match.scale, 2) if visual_match.scale else None,
                             visual_match.closer)
                    if vlabel != last_visrec_label or (now - last_visrec_log) >= 0.5:
                        scale_txt = f"{visual_match.scale:.2f}" if visual_match.scale is not None else "n/a"
                        # Session 57: the inlier-SPREAD size ratio, logged beside `scale` so the next flight
                        # compares both direction estimators on real data (see MISSION CONTEXT finding 3).
                        size_txt = f"{visual_match.size_ratio:.2f}" if visual_match.size_ratio is not None else "n/a"
                        vline = (f"{_rec_prefix(last_rec_frame)} [autopilot][explore][{ctrl.state}] [VISREC] "
                                 f"has_lkg={visual_match.has_lkg} matched={visual_match.matched} "
                                 f"inliers={visual_match.inliers} contained={visual_match.contained} "
                                 f"planar_like={visual_match.planar_like} scale={scale_txt}"
                                 f" size={size_txt} closer={visual_match.closer}")
                        print(vline, flush=True)
                        diag.line(vline)
                        last_visrec_label, last_visrec_log = vlabel, now
                    # Session 49: a DECISION INSTANT is the first tick of a loss episode (the tick the
                    # loss-instant checks actually act on) or a VISUAL_RECOVERY re-match after a turn step.
                    # Those are the frames worth keeping; every other matched tick is shown live but not
                    # written.
                    # Session 58 (C5): chunk 1 stops `wants_visual_match` from returning True on the
                    # loss_edge tick itself (the ticket it used to key off is no longer consulted on
                    # PLAN-LOST/NO-PLAN), so a loss_edge tick is now usually match-free and `decision`
                    # can no longer key off `loss_edge`. It saves the FIRST match of a loss episode
                    # instead, tracked via `visrec_episode_saved`, or a VISUAL_RECOVERY probe re-match.
                    decision = (loss_now and not visrec_episode_saved) or ctrl._visrec_phase == "MATCH"
                    if ctrl.visrec_debug_window and visual_match.debug_image is not None:
                        rel = _visrec_debug_sink(ctrl, diag, visual_match.debug_image, decision,
                                                 now_wall.strftime("%H-%M-%S_%f")[:-3], visrec_saved)
                        if rel is not None:
                            visrec_saved += 1
                            visrec_episode_saved = True
                            visrec_last_saved_rel = rel
                visrec_prev_status = status

            # ---- step the controller + publish ----
            active, state, event = ctrl.step(now, plan_for_step, wall_contact, ceiling_contact,
                                             floor_contact=floor_contact, backwall_contact=backwall_contact,
                                             visual_match=visual_match,
                                             status=status)
            if state == "ADVANCE" and prev_ctrl_state != "ADVANCE":
                detector.reset_forward_ref()   # each leg recalibrates its own free-forward looming
            prev_ctrl_state = state
            prev_active = active               # command for the NEXT frame's detector + the bump re-arm test
            # Session 51 GATE B motion guard: ANY non-empty command vector means the camera is about to see
            # something different, so the memoised match no longer describes the live view. A hover-hold
            # publishes {} and therefore preserves the memo -- which is exactly session 48's grace window.
            if active:
                visrec_moved_since_match = True
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
                visrec_detail = None
                if visrec_probe is not None:
                    visrec_detail = {
                        "phase": ctrl._visrec_phase,
                        "cum_deg": round(ctrl._visrec_cum_deg, 1),
                        "has_lkg": (visual_match.has_lkg if visual_match is not None
                                   else visrec_probe._lkg is not None),
                        "matched": (visual_match.matched if visual_match is not None else None),
                        "inliers": (visual_match.inliers if visual_match is not None else None),
                        "contained": (visual_match.contained if visual_match is not None else None),
                        "planar_like": (visual_match.planar_like if visual_match is not None else None),
                        "scale": (round(visual_match.scale, 3)
                                 if visual_match is not None and visual_match.scale is not None else None),
                        "debug_image": visrec_last_saved_rel,          # relative path, or None on a tick that saved nothing
                        "window_failed": bool(ctrl.visrec_window_failed),
                        "save_failed": bool(ctrl.visrec_save_failed),
                        "lkg_src": visrec_probe._lkg_src,
                        "lkg_ageouts": ctrl.visrec_lkg_ageouts,
                    }
                rec = _timeline_step_record(t_wall, now, last_rec_frame, state, event,
                                            status, plan_for_step, cmd=active,
                                            leg_goal=ctrl.leg_goal, plan_age_s=(now - last_plan_t),
                                            alt={"median": ctrl._alt_median,
                                                 "ceiling": ctrl._ceiling_y, "desired": ctrl._desired_y,
                                                 "delta": ctrl._trim_delta,
                                                 "trim_on": ctrl._trimming, "calib_on": ctrl._calib_active},
                                            visrec=visrec_detail)
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
                # Session 55: periodic map-backdrop record, IN ADDITION to the final one emitted in
                # `finally` below — a hard crash before that block runs previously left the replay with
                # ZERO map records (flight 20260902_165340). Same _downsample_map call, same "map" key
                # flight_replay.py already reads (MAPS = RECORDS.filter(r => r.map !== undefined), newest
                # at/before the cursor) — periodic records only make the replay show the map EVOLVE.
                if log and timeline_map_period_s > 0 and last_ground is not None \
                        and (now - last_map_emit_t) >= timeline_map_period_s:
                    m = _downsample_map(last_ground)
                    if m is not None:
                        diag.timeline({"t_mono": round(now, 3), "map": m})
                    last_map_emit_t = now
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
        if ctrl.visrec_debug_window and not ctrl.visrec_window_failed:
            try:
                cv2.destroyWindow(VISREC_WINDOW)
            except cv2.error as exc:                     # window already gone / never opened
                print(f"[autopilot][explore] LKG debug window close: {exc}", flush=True)
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
    # Session 38: same isolation for `use_visual_recovery_on_stale` -- a live config.yaml retune (operator
    # testing the new path) would otherwise divert every unrelated loss/recovery test below into
    # VISUAL_RECOVERY instead of REWIND/FALLBACK. The dedicated VISUAL RECOVERY tests re-enable it
    # explicitly on their OWN deepcopy (`cfg_vr`, below) — this harness-wide default stays OFF regardless.
    cfg["autonomy"]["explore"]["use_visual_recovery_on_stale"] = False
    # Session 52: and the same for `visrec_debug_window` -- armed ON in config.yaml so the operator can
    # watch the probe live, which made the "(a) DEFAULT OFF" case below fail on the shipped value rather
    # than on any real regression. Third flag of the same family (session 21/30 calibrate_on_goal_change,
    # session 38 use_visual_recovery_on_stale): the harness must never read an operator's live retune.
    cfg["autonomy"]["explore"]["visrec_debug_window"] = False

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
    # WALL contact -> BACKOFF -> SETTLE. BACKOFF is now a phase-timer (session 30): backoff_hold_s (1.0) +
    # backoff_release_s (0.2) before it hands off, so the drive window must comfortably clear that.
    t, a, s, st = _drive(ctrl, plan_turn, True, 0.05, t)
    rec(st)
    t, a, s, st = _drive(ctrl, plan_turn, False, ctrl.backoff_hold_s + ctrl.backoff_release_s + 0.3, t)
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
    # home_refine_max_s=0 too: HOME_REFINE has its own bounded give-up (same idiom) and would otherwise keep
    # pushing forever at this same far-from-origin pose (no takeoff heading -> ORIENT_HOME is skipped).
    chome = ExploreController(cfg, no_takeoff=True)
    chome.home_max_s = 0.0
    chome.home_refine_max_s = 0.0
    far_done = dict(plan_done, pos=[9.0, 9.0])
    _, _, _, sthome = _drive(chome, far_done, False, 0.3, 0.0)
    home_cap_ok = _is_subsequence(["RETURN_TO_ORIGIN", "HOME_REFINE", "DOCK_FLOOR"], sthome)
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
    chomeloop.home_refine_max_s = 0.1  # this test only simulates position closing DURING RETURN_TO_ORIGIN's
                                        # own forward push, not HOME_REFINE's -- give it up fast so the test
                                        # still measures what it's actually testing (the homing loop itself)
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
          f"HOME_REFINE->DOCK_FLOOR(down-pulse)->LOW_STANDOFF(up-nudge)->DONE={post_ok}, home_max_s cap="
          f"{home_cap_ok}, dock_max_s cap={dock_cap_ok}, homing turn+SETTLE+advance={home_loop_ok})  visited {porder}")

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

    # ---- Session 39: DONE must survive a plan loss too (not just RETURN_TO_ORIGIN/ORIENT_HOME/HOME_REFINE/
    #      DOCK_FLOOR/LOW_STANDOFF) -- diagnosed off flight 20260721_233244: a loss after mission-complete fell
    #      through to the generic explore recovery path, which on recovering forced a REPLAN and resurrected
    #      the whole explore FSM (BUMP/BACKOFF/BLACKLIST/TRIM chasing a stale goal) instead of quietly resuming
    #      DONE. Fix: DONE added to POSTLUDE_STATES -> a loss diverts to the existing POSTLUDE_LOST_HOLD and
    #      resumes DONE directly (no new resume-phase branch needed; DONE has no sub-phase).
    cdone = ExploreController(cfg, no_takeoff=True)
    dplan2 = {"plan_valid": True, "done": True, "goal": None, "pos": [0.0, 0.0], "heading_deg": 0.0,
              "bearing_err": None, "pos_y": 0.0, "forward_clearance_dist": 5.0}
    _drive(cdone, dplan2, False, 40.0, 0.0, floor=True)          # happy path all the way to DONE
    reached_done = cdone.state == "DONE"
    tld, fidld = 45.0, 9000
    for _ in range(6):                                          # inject a loss while parked in DONE
        fidld += 1
        cdone.step(tld, dict(dplan2, plan_valid=False, frame_id=fidld, cap_ts=tld, slam_ms=200.0),
                   False, status="PLAN-STALE")
        tld += 0.05
    done_diverts = cdone.state == "POSTLUDE_LOST_HOLD" and cdone._postlude_resume == "DONE"
    touched_recovery = False                                    # recover: OK + fresh fast frames -> resume DONE,
    for _ in range(cdone.calib_lost_recover_frames + 2):        # NEVER touch REPLAN or any _RECOVERY_STATES member
        fidld += 1
        _, s, _ = cdone.step(tld, dict(dplan2, frame_id=fidld, cap_ts=tld, slam_ms=200.0), False, status="OK")
        if s in _RECOVERY_STATES or s == "REPLAN":
            touched_recovery = True
        tld += 0.05
    done_resumes = cdone.state == "DONE" and not touched_recovery
    done_survives_ok = reached_done and done_diverts and done_resumes
    ok = ok and done_survives_ok
    print(f"[self-test] {'PASS' if done_survives_ok else 'FAIL'}  DONE survives a plan loss (reached DONE="
          f"{reached_done}, diverts to POSTLUDE_LOST_HOLD={done_diverts}, resumes DONE without touching "
          f"REPLAN/recovery states={done_resumes})")

    # ---- Session 39: RETURN_TO_ORIGIN's ADVANCE no longer reacts to a blocked forward clearance with BACKOFF
    #      (operator's call: a properly-oriented homing leg isn't expected to hit a wall the way frontier
    #      exploration's own ADVANCE can; that stage keeps its own BACKOFF untouched). _home_phase must never
    #      become "BACKOFF" during RETURN_TO_ORIGIN, no matter how tight the reported forward clearance is.
    cnb = ExploreController(cfg, no_takeoff=True)
    cnb.leg_max_s = 1000.0    # keep ADVANCE running the whole test window (isolates the clearance reaction)
    blocked_plan = {"plan_valid": True, "done": True, "goal": None, "pos": [5.0, 5.0], "heading_deg": 0.0,
                    "bearing_err": None, "pos_y": 0.0, "forward_clearance_dist": 0.1}   # << inside stop_clearance_dist
    saw_backoff = False
    tnb, fidnb = 0.0, 10000
    for _ in range(int(10.0 / 0.05)):
        fidnb += 1
        a, s, _ = cnb.step(tnb, dict(blocked_plan, frame_id=fidnb, cap_ts=tnb, slam_ms=200.0), False)
        if s == "BACKOFF" or cnb._home_phase == "BACKOFF":
            saw_backoff = True
        tnb += 0.05
    no_home_backoff_ok = (not saw_backoff
                          and cnb.state in ("RETURN_TO_ORIGIN", "ORIENT_HOME", "HOME_REFINE", "DOCK_FLOOR"))
    ok = ok and no_home_backoff_ok
    print(f"[self-test] {'PASS' if no_home_backoff_ok else 'FAIL'}  RETURN_TO_ORIGIN ADVANCE no longer backs "
          f"off a blocked clearance (saw_backoff={saw_backoff}, ended state={cnb.state})")

    # ---- ORIENT_HOME real-angle convergence (the 20260720 ping-pong) + orient_home_max_s cap ----
    # (a) Reproduce the diagnosed bug's shape: a bearing error starting just past HALF a turn_step_deg (the
    #     exact knife-edge that made the OLD quantized-to-a-fixed-step logic overshoot side-to-side forever),
    #     with a REALISTIC noisy open-loop turn (actual rotation = 1.2x the commanded angle, never exact).
    #     The fix (turn by the real, clamped-not-quantized angle) must still converge within a bounded number
    #     of turn+settle cycles instead of oscillating indefinitely.
    coh = ExploreController(cfg, no_takeoff=True)
    coh.rest_between_s = 0.1; coh.settle_fresh_frames = 2
    coh._takeoff_heading = 0.0
    OVERSHOOT = 1.4
    saw_oh_turn = converged_oh = False
    toh, fidoh, ohh, last_turn_oh, turns_taken = 0.0, 9500, coh.turn_step_deg / 2.0 + 1.0, None, 0
    for _ in range(int(20.0 / 0.05)):
        fidoh += 1
        a, s, _ = coh.step(toh, {"plan_valid": True, "done": True, "goal": None, "pos": [0.0, 0.0],
                                  "heading_deg": ohh, "bearing_err": None, "pos_y": 0.0,
                                  "forward_clearance_dist": 5.0, "frame_id": fidoh, "cap_ts": toh,
                                  "slam_ms": 200.0}, False)
        if s == "ORIENT_HOME" and coh._orient_home_phase == "TURN" and coh._player is not None:
            saw_oh_turn = True
            nm = coh._player.name
            if nm != last_turn_oh and nm.startswith("turn"):
                last_turn_oh = nm
                turns_taken += 1
                delta = float(nm[4:]) * OVERSHOOT     # the ACTUAL rotation overshoots the commanded angle
                ohh = ((ohh + delta + 180.0) % 360.0) - 180.0
        if s == "HOME_REFINE":
            converged_oh = True
            break
        toh += 0.05
    orient_converge_ok = saw_oh_turn and converged_oh and turns_taken <= 6
    # (b) orient_home_max_s cap: pose stays invalid forever (never converges) -> proceed anyway (VISIBLE).
    coc = ExploreController(cfg, no_takeoff=True)
    coc._takeoff_heading = 90.0
    coc.orient_home_max_s = 0.0
    coc.home_refine_max_s = 0.0
    _, _, _, stoc = _drive(coc, dict(dplan, plan_valid=False), False, 0.3, 0.0, floor=False)
    orient_cap_ok = _is_subsequence(["RETURN_TO_ORIGIN", "ORIENT_HOME", "DOCK_FLOOR"], stoc)
    orient_fix_ok = orient_converge_ok and orient_cap_ok
    ok = ok and orient_fix_ok
    print(f"[self-test] {'PASS' if orient_fix_ok else 'FAIL'}  ORIENT_HOME real-angle convergence "
          f"(converges from a half-step residual under noisy overshoot in {turns_taken} turn(s)="
          f"{orient_converge_ok}, orient_home_max_s cap={orient_cap_ok})")

    # ---- HOME_REFINE: quadrant push-pick + convergence + home_refine_max_s cap ----
    def _refine_pick(pos):
        c = ExploreController(cfg, no_takeoff=True)
        c.rest_between_s = 0.1; c.settle_fresh_frames = 2
        c.state = "HOME_REFINE"
        p = {"plan_valid": True, "done": True, "goal": None, "pos": pos, "heading_deg": 0.0,
             "bearing_err": None, "pos_y": 0.0, "forward_clearance_dist": 5.0,
             "frame_id": 1, "cap_ts": 0.0, "slam_ms": 200.0}
        c.step(0.0, p, False)                                          # PLAN -> picks + builds the push player
        name0 = c._player.name if c._player is not None else None
        a, _s, _e = c.step(0.01, dict(p, frame_id=2, cap_ts=0.01), False)   # PUSH -> emits the actual push fields
        return name0, a
    nfwd, afwd = _refine_pick([0.0, -5.0])      # origin dead ahead (heading 0 = +Z)
    nback, aback = _refine_pick([0.0, 5.0])     # origin dead behind
    nright, aright = _refine_pick([-5.0, 0.0])  # origin to the body-right
    nleft, aleft = _refine_pick([5.0, 0.0])     # origin to the body-left
    quadrant_ok = (nfwd == "refine_forward" and float(afwd.get("trigger", 0.0) or 0.0) == 1.0
                   and nback == "refine_backward" and float(aback.get("reverse", 0.0) or 0.0) == 1.0
                   and nright == "refine_strafe_right" and float(aright.get("joy_horizontal", 0.0) or 0.0) > 0
                   and nleft == "refine_strafe_left" and float(aleft.get("joy_horizontal", 0.0) or 0.0) < 0)
    # Convergence: position genuinely closes on each forward push -> reaches DOCK_FLOOR.
    crf = ExploreController(cfg, no_takeoff=True)
    crf.rest_between_s = 0.1; crf.settle_fresh_frames = 2
    crf.state = "HOME_REFINE"
    trf, fidrf, posrf, reached_dockrf = 0.0, 9800, [0.0, -1.0], False
    for _ in range(int(20.0 / 0.05)):
        fidrf += 1
        a, s, _ = crf.step(trf, {"plan_valid": True, "done": True, "goal": None, "pos": list(posrf),
                                  "heading_deg": 0.0, "bearing_err": None, "pos_y": 0.0,
                                  "forward_clearance_dist": 5.0, "frame_id": fidrf, "cap_ts": trf,
                                  "slam_ms": 200.0}, False)
        if s == "HOME_REFINE" and crf._home_refine_phase == "PUSH" and float(a.get("trigger", 0.0) or 0.0) > 0:
            posrf[1] = min(0.0, posrf[1] + 0.3)   # simulate the push closing distance toward the origin
        if s == "DOCK_FLOOR":
            reached_dockrf = True
            break
        trf += 0.05
    refine_converge_ok = reached_dockrf
    # home_refine_max_s cap: position never improves -> proceed anyway (VISIBLE).
    crc = ExploreController(cfg, no_takeoff=True)
    crc.home_refine_max_s = 0.0
    crc.state = "HOME_REFINE"
    _, _, _, strc = _drive(crc, {"plan_valid": True, "done": True, "goal": None, "pos": [0.0, -5.0],
                                  "heading_deg": 0.0, "bearing_err": None, "pos_y": 0.0,
                                  "forward_clearance_dist": 5.0}, False, 0.3, 0.0, floor=False)
    refine_cap_ok = "DOCK_FLOOR" in strc
    refine_ok = quadrant_ok and refine_converge_ok and refine_cap_ok
    ok = ok and refine_ok
    print(f"[self-test] {'PASS' if refine_ok else 'FAIL'}  HOME_REFINE (quadrant push-pick fwd/back/left/right="
          f"{quadrant_ok}, converges as position closes={refine_converge_ok}, home_refine_max_s cap="
          f"{refine_cap_ok})")

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
    # BACKOFF is now a phase-timer (session 30): backoff_hold_s (1.0) + backoff_release_s (0.2) before it
    # hands off to SETTLE, so the drive window must comfortably clear that (was a flat 40 ticks / ~2s, sized
    # for the old fixed 0.3s recipe).
    bo_ticks = int((cs.backoff_hold_s + cs.backoff_release_s + 0.3) / 0.05)
    bo_states, saw_rev_bo, saw_full_rev, tt = [], False, False, t
    for _ in range(bo_ticks):
        a2, s2, _ev = cs.step(tt, near, False, False, status="OK")
        if not bo_states or bo_states[-1] != s2:
            bo_states.append(s2)
        rev = float((a2 or {}).get("reverse", 0.0) or 0.0)
        if rev > 0.0:
            saw_rev_bo = True
        if rev >= cs.backoff_reverse_mag - 1e-6:
            saw_full_rev = True
        tt += 0.05
    backoff_path = (cs.backoff_on_standoff and s == "ADVANCE" and saw_rev_bo and saw_full_rev
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

    # ---- BACKOFF phase-timer (session 30): hard gate cut + full-magnitude 2s reverse ----
    cbo = ExploreController(cfg, no_takeoff=True)
    cbo.backoff_hold_s = 0.3          # shrink the timings for a fast test; the LOGIC under test is the
    cbo.backoff_release_s = 0.1       # phase transitions themselves, not the specific durations
    cbo._backoff_t0 = 0.0
    cbo._enter("BACKOFF", 0.0)
    plan_bo = {"plan_valid": True, "done": False, "goal": [3.0, 0.0], "pos": [0.0, 0.0], "bearing_err": 0.0}
    a0, s0, _ = cbo.step(0.0, plan_bo, False)
    hold_ok = (s0 == "BACKOFF" and a0.get("gate_override") is True
              and float(a0.get("trigger", -1)) == 0.0
              and abs(float(a0.get("reverse", -1)) - cbo.backoff_reverse_mag) < 1e-9)
    a1, s1, _ = cbo.step(0.2, plan_bo, False)                    # inside hold_s (0.3) still
    still_holding = (s1 == "BACKOFF" and abs(float(a1.get("reverse", -1)) - cbo.backoff_reverse_mag) < 1e-9)
    a2, s2, _ = cbo.step(0.31, plan_bo, False)                   # past hold_s -> RELEASE phase
    release_ok = (s2 == "BACKOFF" and a2.get("gate_override") is True
                 and float(a2.get("reverse", -1)) == 0.0 and float(a2.get("trigger", -1)) == 0.0)
    a3, s3, ev3 = cbo.step(0.45, plan_bo, False)                 # past hold_s + release_s (0.4) -> done
    done_ok = (s3 == "SETTLE" and "backed off" in (ev3 or ""))
    # default backoff_reverse_mag is FULL magnitude (1.0), independent of the throttled reverse_throttle
    # (0.2 by config) used by every other reverse-emitting site.
    full_mag_ok = abs(cbo.backoff_reverse_mag - 1.0) < 1e-9
    backoff_timer_ok = hold_ok and still_holding and release_ok and done_ok and full_mag_ok
    ok = ok and backoff_timer_ok
    print(f"[self-test] {'PASS' if backoff_timer_ok else 'FAIL'}  BACKOFF phase-timer "
          f"(hold: gate_override+trigger=0+full reverse={hold_ok}, holds through hold_s={still_holding}, "
          f"release: reverse=0+gate_override still True={release_ok}, done->settle={done_ok}, "
          f"default mag is FULL (1.0, not reverse_throttle)={full_mag_ok})")

    # ---- SESSION 46 Chunk 2: BACKOFF must own EVERY status while its phase-timer runs (flight
    #      20260901_124211: all 6 loss-instant BACKOFFs emitted fields={} and were wiped by the PLAN-LOST
    #      router one tick later, before the phase-timer's own thrust ever ran on a later tick -- ZERO
    #      reverse was ever commanded). ----
    cbl = ExploreController(cfg, no_takeoff=True)
    cbl.backoff_hold_s = 0.3
    cbl.backoff_release_s = 0.1
    cbl._backoff_t0 = 0.0
    cbl._enter("BACKOFF", 0.0)
    plan_lost_bo = {"plan_valid": False, "done": False, "goal": None, "pos": None, "bearing_err": None}
    # (a) THE REGRESSION: step repeatedly with status=PLAN-LOST, staying WITHIN the hold phase (< hold_s)
    #     -- must stay in BACKOFF and actually command reverse (this fails on the pre-session-46 code: the
    #     top-level router wipes it to HOLD_LOST on the very next tick, emitting {} forever).
    saw_reverse_lost, stayed_backoff_lost = False, True
    tt = 0.0
    for _ in range(3):                    # 0.0, 0.1, 0.2 -- all still < backoff_hold_s (0.3)
        a, s, _ = cbl.step(tt, plan_lost_bo, False, status="PLAN-LOST")
        if s != "BACKOFF":
            stayed_backoff_lost = False
            break
        if a.get("gate_override") is True and abs(float(a.get("reverse", -1)) - cbl.backoff_reverse_mag) < 1e-9:
            saw_reverse_lost = True
        tt += 0.1
    regression_fixed_ok = (stayed_backoff_lost and saw_reverse_lost)
    # (b) completion routes to HOLD_LOST when still lost (never SETTLE, which the router would re-wipe).
    a_done_lost, s_done_lost, ev_done_lost = cbl.step(tt + 0.15, plan_lost_bo, False, status="PLAN-LOST")
    lost_completion_ok = (s_done_lost == "HOLD_LOST" and "backed off" in (ev_done_lost or ""))
    # (c) the SAME phase-timer, entered fresh, completes to SETTLE when status is OK (unchanged behavior).
    cbl2 = ExploreController(cfg, no_takeoff=True)
    cbl2.backoff_hold_s = 0.3
    cbl2.backoff_release_s = 0.1
    cbl2._backoff_t0 = 0.0
    cbl2._enter("BACKOFF", 0.0)
    plan_ok_bo = {"plan_valid": True, "done": False, "goal": [3.0, 0.0], "pos": [0.0, 0.0], "bearing_err": 0.0}
    cbl2.step(0.0, plan_ok_bo, False, status="OK")
    _, s_ok_mid, _ = cbl2.step(0.2, plan_ok_bo, False, status="OK")
    _, s_ok_done, ev_ok_done = cbl2.step(0.45, plan_ok_bo, False, status="OK")
    ok_completion_ok = (s_ok_mid == "BACKOFF" and s_ok_done == "SETTLE" and "backed off" in (ev_ok_done or ""))
    # (d) the impossible-state guard: BACKOFF entered with _backoff_t0 never set must fail LOUD (a visible
    #     recovery to a real state + an explicit event string), never silently do nothing (CLAUDE.md).
    cbl3 = ExploreController(cfg, no_takeoff=True)
    cbl3._backoff_t0 = None
    cbl3._enter("BACKOFF", 0.0)
    a_guard, s_guard, ev_guard = cbl3.step(0.0, plan_lost_bo, False, status="PLAN-LOST")
    guard_ok = (s_guard == "HOLD_LOST" and "_backoff_t0" in (ev_guard or "") and a_guard == {})
    s46c2_ok = regression_fixed_ok and lost_completion_ok and ok_completion_ok and guard_ok
    ok = ok and s46c2_ok
    print(f"[self-test] {'PASS' if s46c2_ok else 'FAIL'}  SESSION-46 wedge escalation Chunk2 "
          f"(BACKOFF survives PLAN-LOST + commands real reverse={regression_fixed_ok}, "
          f"lost completion -> HOLD_LOST={lost_completion_ok}, OK completion -> SETTLE unchanged={ok_completion_ok}, "
          f"no-_backoff_t0 guard is LOUD={guard_ok})")

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
        # Session 45: the goal sits at 3.0u, not the original 1.0u. 1.0 is EXACTLY config's goal_reach_dist,
        # so the new state-independent "goal already reached" check legitimately retired the leg before these
        # parallax/ring assertions could run. These tests are about scout/push DIRECTION logic, not about goal
        # distance, so the fixture just needs a goal the drone is genuinely still travelling toward.
        return {"plan_valid": True, "done": False, "goal": [0.0, 3.0], "pos": list(pos),
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
    cg2.leg_goal = [0.0, 3.0]     # so a missed-bump gets stashed (session 45: 3.0u, matching _plan_be -- at the
                                  # old 1.0u the drone is AT goal_reach_dist, so the state-independent reached
                                  # check would retire the leg before this give-up path could run)
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

    # ---- Map mode: GRADUAL HEIGHT TRIM (session 14, restored 21, VERTICAL PULSE session 40) — sag trigger,
    #      pulse, WAIT, goal keep ----
    def _mk_trim():
        c = ExploreController(cfg, no_takeoff=True)
        # session 44: TRIM's trigger is now the HARDCODED trim_sag_trigger_y/trim_high_trigger_y (-1.75/-2.10),
        # independent of these three -- kept populated here only so tests can also exercise the (still-live)
        # informational "desired_y" reporting in the TRIM-done log message.
        c._ceiling_y, c._desired_y, c._trim_delta = -2.3, -1.9, 0.4
        c.trim_pulse_s, c.trim_settle_s = 0.05, 0.2
        c.settle_gate_s = 0.05          # session 28: TRIM_RESUME_WAIT's gate -- independent of rest_between_s
        c.settle_fresh_frames = 3       # shorten the post-pulse wait loops below
        return c

    def _tplan(posy, pos=(0.0, 0.0), fcd=5.0, ring=None, cap=0.0, fid=1, goal=(3.0, 0.0),
              blacklist=None, blacklist_permanent=None):
        return {"plan_valid": True, "done": False, "goal": list(goal), "pos": list(pos),
                "bearing_err": 0.0, "heading_deg": 0.0, "pos_y": posy, "slam_ms": 200.0, "frame_id": fid,
                "cap_ts": cap, "forward_clearance_dist": fcd,
                "clearance_ring": ring if ring is not None else [[0.0, 5.0], [180.0, 5.0], [90.0, 5.0], [-90.0, 5.0]],
                "blacklist": blacklist or [], "blacklist_permanent": blacklist_permanent or []}

    def _run_trim_resume_wait(c, t0, cap0, fid0, max_ticks=20, **plan_kw):
        """Step `c` (currently in TRIM_RESUME_WAIT) forward, feeding a fresh healthy frame each tick
        (new frame_id + cap_ts >= t0) until the settle-gate clears and it resolves away, or max_ticks is
        exhausted. Returns (active, state, event) of the LAST tick."""
        tt, cap, fid = t0, cap0, fid0
        active, s, ev = {}, c.state, None
        for _ in range(max_ticks):
            active, s, ev = c.step(tt, _tplan(-1.7, cap=cap, fid=fid, **plan_kw), False)
            if s != "TRIM_RESUME_WAIT":
                break
            tt += 0.02; cap += 0.02; fid += 1
        return active, s, ev
    # (a) sag in ADVANCE fires TRIM, snapshots the committed goal (Trap B) AND clears the pending per-hop eval
    #     (session 20b: a trim-interrupted hop must not be judged); at desired height it does NOT fire.
    ta = _mk_trim(); ta._enter("ADVANCE", 0.0); ta.leg_goal = [3.0, 0.0]
    ta._hop_start_goal = [3.0, 0.0]; ta._hop_start_dist = 3.0
    _, sA, _ = ta.step(0.0, _tplan(-1.7), False)                     # -1.7 >= trim_sag_trigger_y(-1.75) -> TRIM
    trig_adv = (sA == "TRIM" and ta._trim_resume_goal == [3.0, 0.0] and ta._hop_start_goal is None)
    tb = _mk_trim(); tb._enter("ADVANCE", 0.0); tb.leg_goal = [3.0, 0.0]
    _, sB, _ = tb.step(0.0, _tplan(-1.95), False)                    # inside [-2.10, -1.75] -> no trigger
    no_trig = (sB == "ADVANCE")
    # (a2) session 44: the threshold VALUES are hardcoded (no longer ratio math off _ceiling_y/_desired_y/
    # _trim_delta), but firing still requires at least one completed calibration (_ceiling_y is not None) --
    # otherwise SETTLE reached generically during PRELUDE/ARM (well before takeoff/calibration) would misfire
    # against this pos_y, which is exactly what broke the PRELUDE self-test before this guard was restored.
    tn = ExploreController(cfg, no_takeoff=True); tn._enter("ADVANCE", 0.0); tn.leg_goal = [3.0, 0.0]
    _, sN, _ = tn.step(0.0, _tplan(-1.7), False)
    precal_no_fire = (sN == "ADVANCE" and tn._ceiling_y is None and tn._trim_delta is None)
    # (b) suppressed while calibrating and in a non-whitelisted state (ORIENT).
    tc = _mk_trim(); tc._enter("ADVANCE", 0.0); tc.leg_goal = [3.0, 0.0]; tc._calib_active = True
    _, sC, _ = tc.step(0.0, _tplan(-1.7), False)
    td = _mk_trim(); td._enter("ORIENT", 0.0); td.leg_goal = [3.0, 0.0]; td._player = td._build_turn(0.0)
    _, sD, _ = td.step(0.0, _tplan(-1.7), False)
    suppress_ok = (sC != "TRIM" and sD != "TRIM")
    # (c) full pulse: emits a full-magnitude joy_vertical pulse (UP -> -1) for trim_pulse_s -- no ring/clearance
    #     check at all, so a ring blocked on EVERY side (the exact 20260722_124351 scenario) must still pulse
    #     normally instead of aborting. WAIT holds until cap_ts >= t0+settle (review-C: the gate is
    #     phase-relative, so a stale pre-TRIM frame can't exit early); TRIM's own exit then hands off to
    #     TRIM_RESUME_WAIT (session 28), which re-aims ORIENT at the preserved goal once ITS OWN settle-gate
    #     (a fresh frame captured after the exit) clears.
    te = _mk_trim(); te._enter("ADVANCE", 0.0); te.leg_goal = [3.0, 0.0]
    blocked_ring = [[0.0, 0.3], [180.0, 0.3], [90.0, 0.3], [-90.0, 0.3]]   # blocked on all 4 sides
    saw_up_pulse, no_abort = False, True
    tt, cap, fid, mid = 0.0, 0.0, 1, None
    for _ in range(300):
        aE, sE, evE = te.step(tt, _tplan(-1.7, fcd=0.3, ring=blocked_ring, cap=cap, fid=fid), False)
        if int(aE.get("joy_vertical", 0)) == -1:            # -1 = up (camera Y down)
            saw_up_pulse = True
        if evE and "abort" in evE:
            no_abort = False
        if sE != "TRIM":
            mid = sE; break
        tt += 0.02; cap += 0.02; fid += 1
    climb_wait_gated = (mid == "TRIM_RESUME_WAIT")
    _, final, _ = _run_trim_resume_wait(te, tt + 0.02, cap + 0.02, fid + 1)
    climb_ok = (saw_up_pulse and no_abort and climb_wait_gated
                and final == "ORIENT" and te.leg_goal == [3.0, 0.0])
    tw = _mk_trim(); tw._enter("TRIM", 0.0); tw._trim_phase = "WAIT"; tw._trim_cmd_t0 = 0.0
    tw._trim_resume_goal = [3.0, 0.0]; tw._trimming = True
    _, sW, _ = tw.step(1.0, _tplan(-1.7, cap=None), False)           # cap_ts None -> not ready -> hold
    wait_hold = (sW == "TRIM")
    def _run_trim_to_exit(c, t0, cap0, fid0, max_ticks=30):
        """Step `c` (freshly entered TRIM) forward until its PULSE+WAIT cycle completes and it hands off to
        TRIM_RESUME_WAIT. Returns (t, cap, fid, state) at that point (or after max_ticks)."""
        tt, cap, fid, s = t0, cap0, fid0, c.state
        for _ in range(max_ticks):
            _, s, _ = c.step(tt, _tplan(-1.7, cap=cap, fid=fid), False)
            if s == "TRIM_RESUME_WAIT":
                break
            tt += 0.02; cap += 0.02; fid += 1
        return tt, cap, fid, s
    # (g) session 28: the preserved goal died (permanently blacklisted) WHILE TRIM was interrupting the leg
    #     -> TRIM_RESUME_WAIT must NOT blindly restore it -> falls through to SETTLE->REPLAN instead.
    tg = _mk_trim(); tg._enter("ADVANCE", 0.0); tg.leg_goal = [3.0, 0.0]
    tg.step(0.0, _tplan(-1.7, cap=0.0, fid=1), False)   # fire TRIM
    ttg, capg, fidg, sG0 = _run_trim_to_exit(tg, 0.02, 0.02, 2)
    gated_g = (sG0 == "TRIM_RESUME_WAIT")
    _, sG, evG = _run_trim_resume_wait(tg, ttg + 0.02, capg + 0.02, fidg + 1,
                                       blacklist=[[3.0, 0.0]], blacklist_permanent=[True])
    blacklist_reroute_ok = (gated_g and sG == "SETTLE" and tg._settle_to == "REPLAN"
                            and tg.leg_goal == [3.0, 0.0]     # leg_goal untouched (never re-restored)
                            and "blacklisted" in (evG or ""))
    # (h) same setup, but the goal is only SOFT-blacklisted (not permanent) -> still restored normally
    #     (only a PERMANENT kill should stop the restore -- a soft/round exclusion may clear next round).
    th = _mk_trim(); th._enter("ADVANCE", 0.0); th.leg_goal = [3.0, 0.0]
    th.step(0.0, _tplan(-1.7, cap=0.0, fid=1), False)   # fire TRIM
    tth, caph, fidh, _ = _run_trim_to_exit(th, 0.02, 0.02, 2)
    _, sH, _ = _run_trim_resume_wait(th, tth + 0.02, caph + 0.02, fidh + 1,
                                     blacklist=[[3.0, 0.0]], blacklist_permanent=[False])
    soft_blacklist_still_restores = (sH == "ORIENT" and th.leg_goal == [3.0, 0.0])
    # (i) session 44 LIVE-FLIGHT BUG (20260901_103028): sustained SLAM slowness (plan OK, every solve
    #     >= slam_slow_ms) inside TRIM_RESUME_WAIT must not hang forever -- force-resolve after
    #     slam_slow_hop_after_s, the SAME rescue SLAM_HOLD already had (sessions 35/43) but TRIM_RESUME_WAIT
    #     never got. A total capture blackout (cap_ts NEVER fed) must NOT be force-resolved.
    #     SESSION 56 REWRITE, same reasoning as the SESSION-50 block: the gate no longer demands FAST
    #     frames, only a CURRENT one, so a slow-but-alive TRIM_RESUME_WAIT now leaves via the NORMAL
    #     gate and never reaches the backstop. Both halves are asserted below on the conditions that
    #     actually drive them now; the session-44 hang this test exists for remains fully covered.
    #     (i1) slow but CURRENT -> normal gate, promptly, and NOT via the forced path.
    ti = _mk_trim(); ti._enter("ADVANCE", 0.0); ti.leg_goal = [3.0, 0.0]
    ti.step(0.0, _tplan(-1.7, cap=0.0, fid=1), False)   # fire TRIM
    tti, capi, fidi, sI0 = _run_trim_to_exit(ti, 0.02, 0.02, 2)
    gated_i = (sI0 == "TRIM_RESUME_WAIT")
    ti.slam_slow_hop_after_s = 5.0                      # far away: must NOT be what resolves this
    tt, fid, sI, evI = tti, fidi, "TRIM_RESUME_WAIT", None
    for _ in range(200):
        tt += 0.02; fid += 1
        p = _tplan(-1.7, cap=tt, fid=fid)               # CURRENT captures, persistently SLOW solves
        p["slam_ms"] = 1500.0
        _, sI, evI = ti.step(tt, p, False)
        if sI != "TRIM_RESUME_WAIT":
            break
    slow_current_resume_ok = (gated_i and sI == "ORIENT" and ti.leg_goal == [3.0, 0.0]
                              and "forced after" not in (evI or "")
                              and (tt - tti) < ti.slam_slow_hop_after_s)
    #     (i2) the backstop is still REACHABLE and still says so: real cap_ts on every frame (not a
    #     blackout), but every capture predates the gate, so only the wall clock can resolve it.
    tk = _mk_trim(); tk._enter("ADVANCE", 0.0); tk.leg_goal = [3.0, 0.0]
    tk.step(0.0, _tplan(-1.7, cap=0.0, fid=1), False)   # fire TRIM
    ttk, capk, fidk, sK0 = _run_trim_to_exit(tk, 0.02, 0.02, 2)
    tk.slam_slow_hop_after_s = 0.5                      # shrink for the test
    tt3, fid3, sK, evK = ttk, fidk, "TRIM_RESUME_WAIT", None
    for _ in range(60):
        tt3 += 0.02; fid3 += 1
        p = _tplan(-1.7, cap=-1.0, fid=fid3)            # STALE captures: real, but before the gate opened
        p["slam_ms"] = 1500.0
        _, sK, evK = tk.step(tt3, p, False)
        if sK != "TRIM_RESUME_WAIT":
            break
    force_resolve_ok = (sK0 == "TRIM_RESUME_WAIT" and sK == "ORIENT" and tk.leg_goal == [3.0, 0.0]
                        and "forced after" in (evK or ""))
    tj = _mk_trim(); tj._enter("ADVANCE", 0.0); tj.leg_goal = [3.0, 0.0]
    tj.step(0.0, _tplan(-1.7, cap=0.0, fid=1), False)
    ttj, capj, fidj, sJ0 = _run_trim_to_exit(tj, 0.02, 0.02, 2)
    tj.slam_slow_hop_after_s = 0.5
    tt2, fid2, sJ = ttj, fidj, "TRIM_RESUME_WAIT"
    for _ in range(60):
        tt2 += 0.02; fid2 += 1
        p = _tplan(-1.7, cap=None, fid=fid2)
        p["slam_ms"] = 1500.0
        _, sJ, _ = tj.step(tt2, p, False)
        if sJ != "TRIM_RESUME_WAIT":
            break
    blackout_no_force = (sJ0 == "TRIM_RESUME_WAIT" and sJ == "TRIM_RESUME_WAIT")
    trim_ok = (trig_adv and no_trig and precal_no_fire and suppress_ok
               and climb_ok and wait_hold and blacklist_reroute_ok and soft_blacklist_still_restores
               and slow_current_resume_ok and force_resolve_ok and blackout_no_force)
    ok = ok and trim_ok
    print(f"[self-test] {'PASS' if trim_ok else 'FAIL'}  explore HEIGHT-TRIM (sag->TRIM+goal-snapshot+hop-eval-clear="
          f"{trig_adv}, no-sag={no_trig}, precalib-no-fire={precal_no_fire}, calib/state-suppress={suppress_ok}, "
          f"vertical pulse even with ring blocked all sides+re-aim(gated)={climb_ok}, cap-None-holds={wait_hold}, "
          f"perm-blacklist->settle-replan={blacklist_reroute_ok}, "
          f"soft-blacklist-still-restores={soft_blacklist_still_restores}, "
          f"TRIM_RESUME_WAIT slow-but-CURRENT resumes via the normal gate={slow_current_resume_ok}, "
          f"TRIM_RESUME_WAIT force-resolve on a stale-capture wedge={force_resolve_ok}, "
          f"blackout does NOT force={blackout_no_force})")

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
    # (a) TRIM DOWN: glued near the ceiling (pos_y <= trim_high_trigger_y, HARDCODED session 44) -> TRIM pulses
    #     joy_vertical=+1 (DOWN, session 40); the preserved goal is re-aimed on exit. An IN-BAND pos_y fires
    #     NEITHER direction.
    tdn = _mk_trim(); tdn._enter("ADVANCE", 0.0); tdn.leg_goal = [3.0, 0.0]
    _, sDn, evDn = tdn.step(0.0, _tplan(-2.15), False)     # -2.15 <= trim_high_trigger_y(-2.10) -> DOWN
    down_fired = (sDn == "TRIM" and tdn._trim_dir == "DOWN" and "(DOWN)" in (evDn or ""))
    saw_down_pulse, tt, cap, fid, mid_dn = False, 0.02, 0.02, 1, None
    for _ in range(300):
        aD, sD2, _ = tdn.step(tt, _tplan(-2.15, cap=cap, fid=fid), False)
        if int(aD.get("joy_vertical", 0)) == 1:            # +1 = down (camera Y down)
            saw_down_pulse = True
        if sD2 != "TRIM":
            mid_dn = sD2; break
        tt += 0.02; cap += 0.02; fid += 1
    # session 28: TRIM's exit hands off to TRIM_RESUME_WAIT (gated re-aim); resolve it here too.
    _, final_dn, _ = _run_trim_resume_wait(tdn, tt + 0.02, cap + 0.02, fid + 1)
    down_ok = (down_fired and saw_down_pulse and mid_dn == "TRIM_RESUME_WAIT"
               and final_dn == "ORIENT" and tdn.leg_goal == [3.0, 0.0])
    tin = _mk_trim(); tin._enter("ADVANCE", 0.0); tin.leg_goal = [3.0, 0.0]
    _, sIn, _ = tin.step(0.0, _tplan(-1.9), False)          # -1.9 is inside [-2.10, -1.75] -> no trigger
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
    def _vpass(ceiling, first, override=0.0):
        c = ExploreController(cfg, no_takeoff=True)
        c._ceiling_y, c._first_ceiling_y = ceiling, first
        c.desired_height_override_y = override
        c._calib_active = True; c._descend_issue_t = 0.0
        c._enter("CALIB_VERIFY", 0.0)
        _a, sV, evV = c.step(0.2, {"plan_valid": True, "done": False, "goal": None, "pos": [0.0, 0.0],
                                   "pos_y": -1.9, "slam_ms": 200.0, "frame_id": 1,
                                   "cap_ts": c.calib_settle_gate_s + 0.1}, False, status="OK")
        return c, (evV or "")
    cv1, ev1 = _vpass(-2.3, None)
    latch_ok = (cv1.target_altitude_y == -1.9 and cv1._desired_y == -1.9
                and cv1._first_ceiling_y == -2.3 and "Y-DRIFT" not in ev1
                and "OVERRIDE" not in ev1)
    cv2, ev2 = _vpass(-2.25, -2.3)
    ydrift_ok = ("Y-DRIFT check" in ev2 and "+0.050" in ev2)
    # (d2) desired_height_override_y session 38: a non-zero override replaces the MEASURED settle (-1.9) with
    # the fixed value, while _ceiling_y stays LIVE so _trim_delta still tracks the real room, and the log
    # names the override explicitly (never a silent substitution).
    cv3, ev3 = _vpass(-2.3, None, override=-1.5)
    override_ok = (cv3.target_altitude_y == -1.5 and cv3._desired_y == -1.5
                   and abs(cv3._trim_delta - (-1.5 - -2.3)) < 1e-9
                   and "DESIRED-HEIGHT OVERRIDE: -1.500" in ev3 and "settled_y was -1.900" in ev3)
    # (e) reference-disagreement warning: the rolling median wandering > delta from desired_y raises ONE loud
    #     notice (take_notice pops it once; display-only, no behavior change).
    cw = ExploreController(cfg, no_takeoff=True)
    cw._height_calibrated = True; cw._desired_y = -1.9; cw._trim_delta = 0.4; cw._ceiling_y = -2.3
    for i in range(12):
        cw.step(i * 0.05, {"plan_valid": True, "done": False, "goal": None, "pos": [0.0, 0.0],
                           "pos_y": -1.3, "slam_ms": 200.0, "frame_id": 500 + i, "cap_ts": i * 0.05}, False)
    n1 = cw.take_notice()
    warn_ok = (n1 is not None and "DISAGREEMENT" in n1 and cw.take_notice() is None)
    s22_ok = down_ok and band_ok and gate_ok and default_off and latch_ok and ydrift_ok and warn_ok and override_ok
    ok = ok and s22_ok
    print(f"[self-test] {'PASS' if s22_ok else 'FAIL'}  SESSION-22 (TRIM DOWN fires+pulse+re-aim={down_ok}, "
          f"in-band no-fire={band_ok}, comfort gate hold/timeout/release={gate_ok}, re-tap default OFF="
          f"{default_off}, PASS latches target+drift baseline={latch_ok}, Y-DRIFT audit line={ydrift_ok}, "
          f"median-disagreement notice={warn_ok}, desired-height override (session 38)={override_ok})")

    # ---- (session 52) TRIM must fire while SLAM is slow: flight 20260901_222552 logged 2749 consecutive
    #      SETTLE/ADVANCE ticks, ALL with slam_ms >= 1000 (zero fast frames), while pos_y drifted past
    #      trim_high_trigger_y for the entire last minute and TRIM never fired again -- the "not self._slam_slow"
    #      conjunct on the trigger made it unreachable. It has since been removed; these prove it fires under
    #      slow SLAM, still fires under fast SLAM (no regression), and that the OTHER guards (band, ceiling-
    #      calibrated, not-mid-calibration) still gate it regardless of SLAM speed.
    def _mk_trim52():
        c = ExploreController(cfg, no_takeoff=True)
        c._ceiling_y = -2.3           # "plausible negative float" -- calibrated
        c._enter("SETTLE", 0.0)
        return c

    def _fill_slam(c, ms, start_fid=1):
        # settle_fresh_frames FRESH frames at `ms`, real (monotonically increasing) cap_ts, per spec.
        fid = start_fid
        for _ in range(c.settle_fresh_frames):
            c._update_slam({"slam_ms": ms, "frame_id": fid, "cap_ts": float(fid) * 0.1})
            fid += 1
        return fid

    # (52-trim-1) TRIM fires under slow SLAM: the triggering tick's OWN plan is slow (matching the flight,
    # where every tick -- including the ones that should have fired TRIM -- was slow), not just past history.
    t52a = _mk_trim52()
    nfid = _fill_slam(t52a, 2000.0)
    p52a = _tplan(-2.20, cap=float(nfid) * 0.1, fid=nfid); p52a["slam_ms"] = 2000.0
    _, s52a, _ = t52a.step(float(nfid) * 0.1, p52a, False)
    trim_under_slow_ok = (t52a._slam_slow and s52a == "TRIM" and t52a._trim_dir == "DOWN")

    # (52-trim-2) TRIM still fires under FAST SLAM -- no regression from removing the gate.
    t52b = _mk_trim52()
    nfid = _fill_slam(t52b, 300.0)
    p52b = _tplan(-2.20, cap=float(nfid) * 0.1, fid=nfid); p52b["slam_ms"] = 300.0
    _, s52b, _ = t52b.step(float(nfid) * 0.1, p52b, False)
    trim_under_fast_ok = (not t52b._slam_slow and s52b == "TRIM" and t52b._trim_dir == "DOWN")

    # (52-trim-3) the OTHER guards still block, even while SLAM is slow: in-band pos_y, no completed
    # calibration (_ceiling_y is None), and mid-calibration (_calib_active) must all still suppress TRIM.
    t52c = _mk_trim52()
    nfid = _fill_slam(t52c, 2000.0)
    p52c = _tplan(-1.90, cap=float(nfid) * 0.1, fid=nfid); p52c["slam_ms"] = 2000.0    # inside the band
    _, s52c, _ = t52c.step(float(nfid) * 0.1, p52c, False)
    band_still_gates = (s52c != "TRIM")

    t52d = _mk_trim52(); t52d._ceiling_y = None
    nfid = _fill_slam(t52d, 2000.0)
    p52d = _tplan(-2.20, cap=float(nfid) * 0.1, fid=nfid); p52d["slam_ms"] = 2000.0
    _, s52d, _ = t52d.step(float(nfid) * 0.1, p52d, False)
    no_ceiling_still_gates = (s52d != "TRIM")

    t52e = _mk_trim52(); t52e._calib_active = True
    nfid = _fill_slam(t52e, 2000.0)
    p52e = _tplan(-2.20, cap=float(nfid) * 0.1, fid=nfid); p52e["slam_ms"] = 2000.0
    _, s52e, _ = t52e.step(float(nfid) * 0.1, p52e, False)
    calib_active_still_gates = (s52e != "TRIM")

    trim52_ok = (trim_under_slow_ok and trim_under_fast_ok and band_still_gates
                 and no_ceiling_still_gates and calib_active_still_gates)
    ok = ok and trim52_ok
    print(f"[self-test] {'PASS' if trim52_ok else 'FAIL'}  SESSION-52 TRIM fires under slow SLAM "
          f"(slow-SLAM fires DOWN={trim_under_slow_ok}, fast-SLAM still fires (no regression)="
          f"{trim_under_fast_ok}, in-band guard={band_still_gates}, no-ceiling guard="
          f"{no_ceiling_still_gates}, mid-calibration guard={calib_active_still_gates})")

    # ---- (session 54) TRIM's WAIT sub-phase must not hang under sustained slow SLAM ----
    # Flight 20260902_155916: TRIM fired correctly (session 52 confirmed above), then hovered in TRIM's
    # WAIT sub-phase for 73.9s -- `healthy` (the exit gate) still required `not self._slam_slow`, and
    # SLAM was solving at a flat ~2700ms the entire time, so it was arithmetically unreachable. Fixed by
    # (1) dropping that conjunct -- the real freshness proof is `cap_ts >= _trim_cmd_t0 + trim_settle_s`,
    # which session 52's own argument already covers -- and (2) a bounded forced exit
    # (slam_slow_hop_after_s, has_any_capture-guarded) as a backstop against any OTHER unsatisfiable
    # condition. These four cases mirror the session-44 TRIM_RESUME_WAIT block above one-for-one.
    def _mk_trim_wait(hop_after=None):
        c = _mk_trim()
        c._enter("TRIM", 0.0)
        c._trim_phase, c._trim_cmd_t0, c._trim_dir = "WAIT", 0.0, "UP"
        c._trim_resume_goal, c._trimming = [3.0, 0.0], True
        if hop_after is not None:
            c.slam_slow_hop_after_s = hop_after
        return c

    def _drive_trim_wait(c, slam_ms, cap_ts_none=False, plan_valid=True, max_ticks=60, dt=0.05):
        tt, fid, s, ev = 0.0, 1, "TRIM", None
        for _ in range(max_ticks):
            tt += dt; fid += 1
            p = _tplan(-1.7, cap=(None if cap_ts_none else tt), fid=fid)
            p["slam_ms"], p["plan_valid"] = slam_ms, plan_valid
            _, s, ev = c.step(tt, p, False)
            if s != "TRIM":
                break
        return tt, fid, s, ev

    # (54-trim-1) THE REGRESSION CASE: sustained slow SLAM (2700ms/frame, matching the flight) with an
    # otherwise-healthy plan must resolve in a couple of ticks, NOT the 15s backstop -- the fix, not the
    # safety net, is what should be firing here.
    t54a = _mk_trim_wait()
    tt54a, _, s54a, ev54a = _drive_trim_wait(t54a, slam_ms=2700.0)
    fixed_fast_exit_ok = (s54a == "TRIM_RESUME_WAIT" and tt54a < t54a.slam_slow_hop_after_s
                          and "FORCED" not in (ev54a or ""))

    # (54-trim-2) no regression under fast SLAM -- unaffected by the fix, and not misreported as forced.
    t54b = _mk_trim_wait()
    _, _, s54b, _ = _drive_trim_wait(t54b, slam_ms=200.0)
    fast_no_regression_ok = (s54b == "TRIM_RESUME_WAIT"
                             and (t54b.last_timeout is None
                                  or t54b.last_timeout.get("kind") != "TRIM_WAIT_FORCED"))

    # (54-trim-3) belt-and-suspenders: a DIFFERENT unsatisfiable condition (plan_valid pinned False, real
    # cap_ts still advancing) must still force-resolve after slam_slow_hop_after_s, and the preserved goal
    # (Trap B) must survive the forced hand-off through to ORIENT, exactly like TRIM_RESUME_WAIT's own
    # force-resolve test above.
    t54c = _mk_trim_wait(hop_after=0.5)
    tt54c, fid54c, s54c, ev54c = _drive_trim_wait(t54c, slam_ms=200.0, plan_valid=False, dt=0.02)
    forced_exit_ok = (s54c == "TRIM_RESUME_WAIT" and "FORCED after" in (ev54c or "")
                      and t54c.last_timeout is not None
                      and t54c.last_timeout.get("kind") == "TRIM_WAIT_FORCED")
    _, s54c2, _ = _run_trim_resume_wait(t54c, tt54c + 0.02, tt54c + 0.02, fid54c + 1)
    forced_goal_preserved_ok = (s54c2 == "ORIENT" and t54c.leg_goal == [3.0, 0.0])

    # (54-trim-4) blackout guard: cap_ts NEVER arrives (perception producing nothing) -- the forced exit
    # must NOT fire even past slam_slow_hop_after_s; a wall clock must never paper over a total blackout.
    t54d = _mk_trim_wait(hop_after=0.5)
    _, _, s54d, _ = _drive_trim_wait(t54d, slam_ms=200.0, cap_ts_none=True, dt=0.02)
    blackout_no_force_ok = (s54d == "TRIM")

    trim54_ok = (fixed_fast_exit_ok and fast_no_regression_ok and forced_exit_ok
                and forced_goal_preserved_ok and blackout_no_force_ok)
    ok = ok and trim54_ok
    print(f"[self-test] {'PASS' if trim54_ok else 'FAIL'}  SESSION-54 TRIM WAIT no-hang "
          f"(sustained-slow-SLAM exits fast, not forced={fixed_fast_exit_ok}, fast-SLAM no regression="
          f"{fast_no_regression_ok}, other-unsatisfiable-condition force-resolves={forced_exit_ok}, "
          f"forced hand-off preserves goal (Trap B)={forced_goal_preserved_ok}, "
          f"total blackout never forces={blackout_no_force_ok})")

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

    # ---- SESSION 45: the "stuck next to its own goal" stall (flight 20260901_112227 -- the drone hovered
    #      0.31u from its committed goal for 221s, goal_reach_dist being 1.0, and nothing retired it). ----
    def _s45plan(goal, pos, slam_ms=200.0, fid=1, cap=0.0):
        return {"plan_valid": True, "done": False, "goal": list(goal), "pos": list(pos),
                "bearing_err": 0.0, "heading_deg": 0.0, "pos_y": -1.9, "slam_ms": slam_ms,
                "frame_id": fid, "cap_ts": cap, "forward_clearance_dist": 9.0,
                "clearance_ring": [[0.0, 5.0], [180.0, 5.0], [90.0, 5.0], [-90.0, 5.0]]}
    # (a) a leg PARKED anywhere that is not an excluded state, with the goal already inside goal_reach_dist,
    #     is retired -> SETTLE(->REPLAN). Previously the reached test lived ONLY in the ADVANCE handler, so a
    #     leg parked in SLAM_HOLD/ORIENT (which is where that flight sat) never evaluated it at all.
    reached_from = {}
    for parked in ("SLAM_HOLD", "ORIENT", "PARALLAX_PUSH"):
        c45 = ExploreController(cfg, no_takeoff=True)
        c45.leg_goal = [0.3, 0.0]                       # 0.3u away -> well inside goal_reach_dist (1.0)
        c45._enter(parked, 0.0)
        if parked == "ORIENT":
            c45._player = c45._build_turn(0.0)          # ORIENT's handler needs a live player to fall through to
        _, s45, ev45 = c45.step(0.1, _s45plan([0.3, 0.0], [0.0, 0.0]), False)
        reached_from[parked] = (s45 == "SETTLE" and c45._settle_to == "REPLAN"
                                and "already reached" in (ev45 or ""))
    reached_anystate_ok = all(reached_from.values())
    # (a2) a goal still genuinely FAR away is untouched (no spurious retirement).
    cfar = ExploreController(cfg, no_takeoff=True)
    cfar.leg_goal = [5.0, 0.0]
    cfar._enter("ORIENT", 0.0)
    cfar._player = cfar._build_turn(0.0)
    _, s_far, _ = cfar.step(0.1, _s45plan([5.0, 0.0], [0.0, 0.0]), False)
    far_untouched_ok = (s_far == "ORIENT")
    # (a3) a BLIND tick (plan invalid) must never retire a goal off a frozen pose, however close it reads.
    cblind = ExploreController(cfg, no_takeoff=True)
    cblind.leg_goal = [0.3, 0.0]
    cblind._enter("ORIENT", 0.0)
    cblind._player = cblind._build_turn(0.0)
    blind_plan = dict(_s45plan([0.3, 0.0], [0.0, 0.0]), plan_valid=False)
    _, s_blind, _ = cblind.step(0.1, blind_plan, False, status="PLAN-STALE")
    blind_no_retire_ok = (s_blind != "SETTLE" or cblind._settle_to != "REPLAN")
    # (b) THE SETTLE TRAP REGRESSION. _enter("SETTLE") resets _settle_ok/_settle_t0, and the drone does not
    #     move during a settle, so if the reached check were allowed to fire in SETTLE it would re-open the
    #     settle EVERY TICK and hang forever -- strictly worse than the stall being fixed. Drive a SETTLE with
    #     the reached condition permanently true and assert it still completes its gate and reaches REPLAN
    #     WITHOUT its gate ever being restamped (a re-fire would call _enter("SETTLE") again, moving
    #     _settle_gate_t0 forward and resetting _settle_ok -- the signature of the trap).
    cset = ExploreController(cfg, no_takeoff=True)
    cset.settle_gate_s = 0.05; cset.settle_fresh_frames = 3
    cset.leg_goal = [0.3, 0.0]                          # permanently "reached"
    cset._settle_to = "REPLAN"
    cset._enter("SETTLE", 0.0)
    gate_t0_at_entry = cset._settle_gate_t0
    t45, cap45, fid45, s_set, gate_restamped, ticks_in_settle = 0.0, 0.0, 10, "SETTLE", False, 0
    for _ in range(200):
        t45 += 0.02; cap45 += 0.02; fid45 += 1
        _, s_set, _ = cset.step(t45, _s45plan([0.3, 0.0], [0.0, 0.0], fid=fid45, cap=cap45), False)
        if s_set != "SETTLE":
            break
        ticks_in_settle += 1
        if cset._settle_gate_t0 != gate_t0_at_entry:
            gate_restamped = True                       # the gate moved -> the check re-fired inside SETTLE
    settle_trap_ok = (s_set == "REPLAN" and not gate_restamped and ticks_in_settle < 190)
    # (c) a forced SLAM-slow hop routed into PARALLAX_PUSH must SURVIVE its own push for the grace window --
    #     the root cause: PARALLAX_PUSH was missing from _enter()'s exemption AND from the _slam_slow_hop_active
    #     bypass, so every forced hop died ~30ms in and the drone turned without ever translating.
    cpp = ExploreController(cfg, no_takeoff=True)
    cpp._enter("PARALLAX_PUSH", 0.0)
    cpp._push_dir = None
    cpp._slam_slow_hop_deadline = 100.0                 # a forced hop's grace window is open
    _, s_pp, _ = cpp.step(0.1, _s45plan([0.0, 3.0], [0.0, 0.0], slam_ms=1500.0, fid=2, cap=0.1), False)
    push_survives_ok = (s_pp == "PARALLAX_PUSH")
    # …and with NO grace window open, a slow push still stops and settles exactly as before (regression).
    cpp2 = ExploreController(cfg, no_takeoff=True)
    cpp2._enter("PARALLAX_PUSH", 0.0)
    cpp2._push_dir = None
    _, s_pp2, _ = cpp2.step(0.1, _s45plan([0.0, 3.0], [0.0, 0.0], slam_ms=1500.0, fid=2, cap=0.1), False)
    push_still_holds_ok = (s_pp2 == "SLAM_HOLD")
    # (d) DEDUP UN-STARVATION: same goal re-committed with NO hop ever judged. Inside goal_dedup_max_hold_s the
    #     pick stays suppressed (a real re-orientation); past it the pick registers for real, so the goals-DB
    #     loop guard can finally arm instead of being starved for the whole flight (picks stuck at 1).
    cded = ExploreController(cfg, no_takeoff=True); cded.hop_duration_s = 0.1
    cded.goal_dedup_max_hold_s = 5.0
    goalE = [9.0, 0.0]
    cded.leg_goal = list(goalE); cded._hop_start_goal = list(goalE); cded._hop_start_dist = 8.0
    cded._enter("REPLAN", 0.0)
    cded.step(0.0, _s45plan(goalE, [1.0, 0.0], fid=1, cap=0.0), False)
    cded.take_pick_pulse()                              # the first, genuine pick
    cded._enter("REPLAN", 1.0)
    cded.step(1.0, _s45plan(goalE, [1.0, 0.0], fid=2, cap=1.0), False)
    within = cded.take_pick_pulse()
    dedup_holds_ok = (within is not None and within["pick_goal"] is None)     # 1s in -> still deduped
    cded._enter("REPLAN", 9.0)
    cded.step(9.0, _s45plan(goalE, [1.0, 0.0], fid=3, cap=9.0), False)        # 9s > goal_dedup_max_hold_s
    past = cded.take_pick_pulse()
    dedup_bounded_ok = (past is not None and past["pick_goal"] == goalE and past["pick_pos"] is not None)
    s45_ok = (reached_anystate_ok and far_untouched_ok and blind_no_retire_ok and settle_trap_ok
              and push_survives_ok and push_still_holds_ok and dedup_holds_ok and dedup_bounded_ok)
    ok = ok and s45_ok
    print(f"[self-test] {'PASS' if s45_ok else 'FAIL'}  SESSION-45 stuck-at-goal "
          f"(reached retires from any parked state={reached_anystate_ok}, far goal untouched={far_untouched_ok}, "
          f"blind never retires={blind_no_retire_ok}, SETTLE-trap avoided={settle_trap_ok}, "
          f"forced hop survives PARALLAX_PUSH={push_survives_ok}, slow push still holds w/o grace={push_still_holds_ok}, "
          f"dedup holds within window={dedup_holds_ok}, dedup bounded past it={dedup_bounded_ok})")

    # ---- SESSION 46 Chunk 1: blind-contact escalation counter -- config knob + reset wiring ONLY, no
    #      behavior change yet (nothing reads the counter here). Flight 20260901_124211: the drone was
    #      wedged (reverse -> 0.000u, flow detector latched BACKWALL) and BLIND_BACKOFF just replayed the
    #      same failing reflex forever with no escalation. This block proves the counter exists with the
    #      right default and resets ONLY at genuine recovery boundaries, never on a same-disc re-commit. ----
    c46 = ExploreController(cfg, no_takeoff=True)
    knob_default_ok = (c46.blind_contact_escalate_after == 2 and c46._blind_contact_reacts == 0)
    # (b) reset_leg() (manual takeover / leg reset) clears it.
    c46._blind_contact_reacts = 2
    c46.reset_leg()
    resetleg_ok = (c46._blind_contact_reacts == 0)
    # (c) REPLAN: a same-disc re-commit (within goal_area_radius, no hop judged -- the exact circling
    #     pattern flight 20260901_112227 showed every ~25s) must NOT clear it; a MATERIALLY new goal must.
    c46b = ExploreController(cfg, no_takeoff=True); c46b.hop_duration_s = 0.1
    goalX = [9.0, 0.0]
    c46b._enter("REPLAN", 0.0)
    c46b.step(0.0, _s45plan(goalX, [1.0, 0.0], fid=1, cap=0.0), False)   # first commit -> seeds _leg_goal_prev
    c46b._blind_contact_reacts = 2
    goalX_close = [9.3, 0.0]                      # 0.3u away -> inside goal_area_radius (0.5) -> NOT new
    c46b._enter("REPLAN", 1.0)
    c46b.step(1.0, _s45plan(goalX_close, [1.0, 0.0], fid=2, cap=1.0), False)
    replan_close_holds_ok = (c46b._blind_contact_reacts == 2)
    c46b._blind_contact_reacts = 2
    goalY = [20.0, 0.0]                           # far away -> materially new
    c46b._enter("REPLAN", 2.0)
    c46b.step(2.0, _s45plan(goalY, [1.0, 0.0], fid=3, cap=2.0), False)
    replan_far_resets_ok = (c46b._blind_contact_reacts == 0)
    s46c1_ok = (knob_default_ok and resetleg_ok and replan_close_holds_ok and replan_far_resets_ok)
    ok = ok and s46c1_ok
    print(f"[self-test] {'PASS' if s46c1_ok else 'FAIL'}  SESSION-46 wedge escalation Chunk1 "
          f"(config+field defaults={knob_default_ok}, reset_leg clears={resetleg_ok}, "
          f"same-disc REPLAN holds={replan_close_holds_ok}, materially-new REPLAN resets={replan_far_resets_ok})")

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
    # session 44: this test's synthetic post-ascend pos_y is pinned at 0.0 (a flat stand-in, not a
    # realistic cruise altitude), which the OLD live-calibrated TRIM band tolerated by construction
    # (it was computed from this same test's own numbers) but the new HARDCODED trim_sag_trigger_y
    # reads as permanently "sagged" (0.0 >= -1.75), firing TRIM every tick forever and never letting
    # SETTLE reach REPLAN/ORIENT. This test is about the arm/takeoff/ascend/descend/baseline sequence,
    # not TRIM (which has its own dedicated HEIGHT-TRIM self-test) -- disable it here rather than
    # contort this test's altitude to fit the new band.
    cp.trim_enable = False
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
    # (c) PLAN-STALE (with history) + use_rewind_on_stale=True -> RECOVERY_REWIND; then OK -> wait for SLAM
    #     to settle -> brake -> resume. REWIND is default-OFF as of session 31 (operator ask -- never once
    #     visibly helped on a real flight), so this test opts back in explicitly to keep the (still-present,
    #     just default-disabled) code path covered.
    cw = ExploreController(cfg, no_takeoff=True)
    cw.use_rewind_on_stale = True
    cw._ever_tracked = True            # a MID-FLIGHT loss, not startup warmup (that's its own WARMUP guard)
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
    # (d) PLAN-STALE + EMPTY history, default use_rewind_on_stale=False -> straight to the FALLBACK sweep ->
    #     STUCK once fallback_max_rotation_deg is reached (session 31). The turn sweep is UNIDIRECTIONAL
    #     (turn always +, never <0); forward pushes ARE allowed (unlike normal scouting -- while blind
    #     there's no live signal saying the back is any safer than the front), so this only checks that SOME
    #     push happens and the turn direction never flips. Shrink every timing knob for a fast test.
    cf = ExploreController(cfg, no_takeoff=True)
    cf.fallback_initial_wait_s = 0.02
    cf.fallback_post_push_wait_s = 0.02
    cf.fallback_push_fwd_back_s = 0.02
    cf.fallback_push_strafe_s = 0.02
    cf.fallback_max_rotation_deg = 2 * cf.recovery_turn_step_deg   # 2 cycles -> STUCK within the drive window
    cf._ever_tracked = True                 # a MID-FLIGHT loss (history wiped by a wall hit), not startup warmup
    cf.command_history.clear()
    seen, saw_push, saw_turn_pos, saw_turn_neg, t = set(), False, False, False, 0.0
    for _ in range(int(5.0 / 0.02)):
        a, s, _ = cf.step(t, stale, False, status="PLAN-STALE")
        seen.add(s)
        if s == "FALLBACK":
            if any(abs(float(a.get(k, 0.0) or 0.0)) > 0 for k in ("trigger", "reverse", "joy_horizontal")):
                saw_push = True
            y = float(a.get("yaw", 0.0))
            if y > 0:
                saw_turn_pos = True
            if y < 0:
                saw_turn_neg = True                   # must NEVER happen (unidirectional sweep)
        t += 0.02
    fallback_ok = ("FALLBACK" in seen and "STUCK" in seen and saw_push
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
    #     (the flight-20260713 frantic loop). Arm recovery on STALE (the sweep's cycle count climbs), flick to
    #     LOST (HOLD_LOST), back to STALE -> the cycle count + cumulative rotation PERSIST and STUCK is reached.
    cflk = ExploreController(cfg, no_takeoff=True)
    cflk.fallback_initial_wait_s = 0.02
    cflk.fallback_post_push_wait_s = 0.02
    cflk.fallback_push_fwd_back_s = 0.02
    cflk.fallback_push_strafe_s = 0.02
    cflk.fallback_max_rotation_deg = 2 * cflk.recovery_turn_step_deg   # 2 cycles -> STUCK
    cflk._ever_tracked = True
    tt = 0.0
    tt, _, _, _ = _drive(cflk, stale_nr, False, 1.0, tt, status="PLAN-STALE")   # arm + a fallback cycle or two
    cyc_mid, rec_mid = cflk._fallback_cycle, cflk._recovering
    tt, _, _, s_lost = _drive(cflk, lost_nr, False, 1.0, tt, status="PLAN-LOST")  # flicker -> HOLD_LOST
    cyc_after, rec_after = cflk._fallback_cycle, cflk._recovering                # must NOT reset
    seen_f = set()
    for _ in range(int(5.0 / 0.02)):
        _a, s2, _ = cflk.step(tt, stale_nr, False, status="PLAN-STALE"); seen_f.add(s2); tt += 0.02
    flicker_ok = (rec_mid and rec_after and cyc_after >= cyc_mid
                  and "HOLD_LOST" in s_lost and "STUCK" in seen_f)
    # (g) CONSUMING REWIND (use_rewind_on_stale=True -- default off as of session 31, opted back in here to
    #     keep the code path covered): each REWIND cycle pops ONE maneuver; the history DRAINS to empty then
    #     -> FALLBACK.
    crw = ExploreController(cfg, no_takeoff=True)
    crw.use_rewind_on_stale = True
    crw._ever_tracked = True
    crw.recovery_settle_max_s = 0.1         # keep the inter-step settle fast regardless of the live config
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
    cgp.use_rewind_on_stale = True   # exercise the REWIND-side ghost-path guard (default-off since session 31)
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
    # (j) CONFIRMING RECOVERY (session 35): trust restores the moment a loss recovers to a genuinely SETTLED
    #     OK (SLAM_HOLD's settle-gate-clear, resume target "SETTLE") -- drop flags, reset the fallback-sweep
    #     counter, clear the (now-stale) history. An UN-settled OK tick (still mid-gate) does NOT confirm.
    padv_c = {"plan_valid": True, "done": False, "goal": [10.0, 0.0], "pos": [0.0, 0.0], "bearing_err": 0.0,
              "forward_clearance_dist": 8.0, "pos_y": 0.0, "slam_ms": 120.0, "frame_id": 1}
    cca = ExploreController(cfg, no_takeoff=True)
    cca.settle_gate_s = 0.02
    cca._recovering = True; cca._history_broken = True; cca._fallback_cycle = 5
    cca.command_history.append({"kind": "turn", "theta": 15.0})
    cca.leg_goal = [10.0, 0.0]; cca.target_altitude_y = 0.0
    cca._enter("HOLD_LOST", 0.0)
    t = 0.0
    _a, s, _ = cca.step(t, dict(padv_c, frame_id=1), False, status="OK"); t += 0.05
    entered_hold_ok = (s == "SLAM_HOLD" and cca._slam_resume == "SETTLE")
    s_final = s
    for i in range(2, 2 + cca.settle_fresh_frames + 4):
        _a, s_final, _ = cca.step(t, dict(padv_c, frame_id=i, cap_ts=t), False, status="OK"); t += 0.05
        if s_final != "SLAM_HOLD":
            break
    confirm_ok = (entered_hold_ok and s_final == "SETTLE" and not cca._recovering and not cca._history_broken
                  and cca._fallback_cycle == 0 and len(cca.command_history) == 0)
    ccb = ExploreController(cfg, no_takeoff=True)
    ccb._recovering = True; ccb.leg_goal = [10.0, 0.0]; ccb.target_altitude_y = 0.0
    ccb._enter("HOLD_LOST", 0.0)
    _a, s_b, _ = ccb.step(0.0, dict(padv_c, frame_id=1), False, status="OK")   # one tick -> not yet settled
    noconfirm_ok = (ccb._recovering is True and s_b == "SLAM_HOLD")
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
    crs.use_rewind_on_stale = True   # exercise REWIND specifically (default-off since session 31)
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
    # (b) FALLBACK: empty history -> the 4-phase sweep (session 31); consecutive pushes must be SEPARATED by
    #     the fixed WAIT_POST phase (not back-to-back), and STUCK is still reached at the rotation cap.
    cfs = ExploreController(cfg, no_takeoff=True); cfs._ever_tracked = True
    cfs.fallback_initial_wait_s = 0.02; cfs.fallback_post_push_wait_s = 0.05
    cfs.fallback_push_fwd_back_s = 0.02; cfs.fallback_push_strafe_s = 0.02
    cfs.fallback_max_rotation_deg = 2 * cfs.recovery_turn_step_deg
    cfs.command_history.clear()
    fb_settled, seen_fb, tfs, fidfs = False, set(), 0.0, 300
    for _ in range(int(5.0 / 0.02)):
        fidfs += 1
        _a, s, _ = cfs.step(tfs, dict(rec_stale, frame_id=fidfs, cap_ts=tfs, slam_ms=200.0), False, status="PLAN-STALE")
        seen_fb.add(s)
        if s == "FALLBACK" and cfs._fallback_phase == "WAIT_POST":
            fb_settled = True
        tfs += 0.02
    fb_settle_ok = fb_settled and "STUCK" in seen_fb
    # (c) BOUNDED escape: with NO fresh frames (frame_id absent) the recovery settle must still END at
    #     recovery_settle_max_s (dead pipeline) so a re-exposure maneuver follows — never hang.
    cbnd = ExploreController(cfg, no_takeoff=True)
    cbnd._settle_begin(0.0)
    cbnd.recovery_settle_max_s = 0.5
    d_early, _ = cbnd._settle_poll(0.2, {"pos": [0, 0]}, require_fast=False, min_frames=4, max_hold_s=0.5)
    d_cap, capped = cbnd._settle_poll(0.6, {"pos": [0, 0]}, require_fast=False, min_frames=4, max_hold_s=0.5)
    bounded_ok = (not d_early) and d_cap and capped
    # (d) FALLBACK phase ORDER: TURN (yaw) completes FIRST, PUSH (reverse/strafe/trigger) LAST -> the motion
    #     right before the settle is a parallax translation, not a bare rotation.
    cord = ExploreController(cfg, no_takeoff=True)
    cord._ever_tracked = True
    cord.fallback_initial_wait_s = 0.02
    cord.command_history.clear()
    saw_turn_yaw, cum_before_push, t = False, None, 0.0
    for _ in range(int(3.0 / 0.02)):
        a, s, _ = cord.step(t, stale, False, status="PLAN-STALE")
        if cord._fallback_phase == "TURN" and abs(float(a.get("yaw", 0.0) or 0.0)) > 0:
            saw_turn_yaw = True
        if cord._fallback_phase == "PUSH":
            cum_before_push = cord._fallback_cum_deg
            break
        t += 0.02
    order_ok = saw_turn_yaw and cum_before_push == cord.recovery_turn_step_deg
    rec_settle_ok = rewind_settle_ok and fb_settle_ok and bounded_ok and order_ok
    ok = ok and rec_settle_ok
    print(f"[self-test] {'PASS' if rec_settle_ok else 'FAIL'}  RECOVERY inter-action settles "
          f"(REWIND holds between pops={rewind_settle_ok}, FALLBACK holds between attempts+STUCK={fb_settle_ok}, "
          f"bounded-escape-when-dead={bounded_ok}, turn-before-push={order_ok})")

    # ---- FALLBACK sweep (session 31, replaces the session-29 direction-cycling search) ----
    # (a) INITIAL_WAIT holds neutral for fallback_initial_wait_s before the first TURN begins.
    caw = ExploreController(cfg, no_takeoff=True)
    caw._ever_tracked = True
    caw.fallback_initial_wait_s = 0.1
    caw.command_history.clear()
    seen_aw, t = set(), 0.0
    for _ in range(int(0.3 / 0.02)):
        _a, s, _ = caw.step(t, stale, False, status="PLAN-STALE")
        seen_aw.add((s, caw._fallback_phase))
        t += 0.02
    initial_wait_ok = (("FALLBACK", "INITIAL_WAIT") in seen_aw and ("FALLBACK", "TURN") in seen_aw)
    # (b) a full cycle visits TURN -> PUSH -> WAIT_POST, accumulates _fallback_cum_deg by recovery_turn_step_deg
    #     per cycle, and PUSH commands a FULL-magnitude move (no throttled knobs).
    ccy = ExploreController(cfg, no_takeoff=True)
    ccy._ever_tracked = True
    ccy.fallback_initial_wait_s = 0.02; ccy.fallback_post_push_wait_s = 0.02
    ccy.fallback_push_fwd_back_s = 0.02; ccy.fallback_push_strafe_s = 0.02
    ccy.command_history.clear()
    seen_cy, saw_push_mag, t = set(), False, 0.0
    for _ in range(int(2.0 / 0.02)):
        a, s, _ = ccy.step(t, stale, False, status="PLAN-STALE")
        seen_cy.add(ccy._fallback_phase)
        if ccy._fallback_phase == "PUSH":
            mags = [abs(float(a.get(k, 0.0) or 0.0)) for k in ("trigger", "reverse", "joy_horizontal")]
            if mags and max(mags) >= 0.999:
                saw_push_mag = True
        t += 0.02
        if ccy._fallback_phase == "WAIT_POST":
            break
    cycle_ok = ({"INITIAL_WAIT", "TURN", "PUSH"} <= seen_cy and saw_push_mag
                and ccy._fallback_cum_deg == ccy.recovery_turn_step_deg)
    # (c) a live contact MATCHING the in-flight push direction ends that push early; a MISMATCHED contact
    #     (e.g. backwall while pushing forward) does not.
    ccc = ExploreController(cfg, no_takeoff=True)
    ccc._ever_tracked = True
    ccc.fallback_initial_wait_s = 0.0
    ccc.fallback_push_fwd_back_s = 1.0
    ccc._enter_fallback_sweep(0.0, None)
    ccc._fallback_phase, ccc._fallback_phase_t0 = "PUSH", 0.0
    ccc._fallback_push_dirn = "forward"
    ccc.step(0.0, stale, False, status="PLAN-STALE")             # builds the push player
    ccc.step(0.01, stale, True, status="PLAN-STALE")             # wall_contact=True mid-forward-push
    push_aborted_ok = ccc._fallback_phase == "WAIT_POST"
    ccm = ExploreController(cfg, no_takeoff=True)
    ccm._ever_tracked = True
    ccm.fallback_initial_wait_s = 0.0
    ccm.fallback_push_fwd_back_s = 1.0
    ccm._enter_fallback_sweep(0.0, None)
    ccm._fallback_phase, ccm._fallback_phase_t0 = "PUSH", 0.0
    ccm._fallback_push_dirn = "forward"
    ccm.step(0.0, stale, False, status="PLAN-STALE")
    ccm.step(0.01, stale, False, backwall_contact=True, status="PLAN-STALE")   # mismatched -> ignored
    push_mismatch_ignored = ccm._fallback_phase == "PUSH"
    contact_early_exit_ok = push_aborted_ok and push_mismatch_ignored
    # (d) exhaustion: cumulative commanded rotation reaching fallback_max_rotation_deg -> STUCK.
    cex = ExploreController(cfg, no_takeoff=True)
    cex._ever_tracked = True
    cex.fallback_initial_wait_s = 0.02; cex.fallback_post_push_wait_s = 0.02
    cex.fallback_push_fwd_back_s = 0.02; cex.fallback_push_strafe_s = 0.02
    cex.fallback_max_rotation_deg = 2 * cex.recovery_turn_step_deg
    cex.command_history.clear()
    s, t = None, 0.0
    for _ in range(int(5.0 / 0.02)):
        _a, s, _ = cex.step(t, stale, False, status="PLAN-STALE")
        t += 0.02
        if s == "STUCK":
            break
    exhaust_ok = (s == "STUCK" and cex._fallback_cum_deg >= cex.fallback_max_rotation_deg)
    # (e) flicker persistence: a bounce through HOLD_LOST mid-phase and back does NOT reset
    #     _fallback_cum_deg/_fallback_cycle (mirrors the existing _recovering flicker-persist rule).
    cfl29 = ExploreController(cfg, no_takeoff=True)
    cfl29._ever_tracked = True
    cfl29.fallback_initial_wait_s = 0.02
    cfl29._enter_fallback_sweep(0.0, None)
    cfl29._fallback_cum_deg = 45.0
    cfl29._fallback_cycle = 3
    cfl29._fallback_phase = "TURN"
    cfl29.state = "HOLD_LOST"                       # simulate the PLAN-LOST flicker bounce
    cfl29.step(0.0, stale, False, status="PLAN-LOST")
    cum_after_flicker, cyc_after_flicker = cfl29._fallback_cum_deg, cfl29._fallback_cycle
    cfl29.state = "FALLBACK"                        # flicker back
    cfl29.step(0.02, stale, False, status="PLAN-STALE")
    flicker_persist_ok = (cum_after_flicker == 45.0 and cyc_after_flicker == 3
                           and cfl29._fallback_cum_deg == 45.0 and cfl29._fallback_cycle == 3)
    fallback_sweep_ok = (initial_wait_ok and cycle_ok and contact_early_exit_ok and exhaust_ok
                          and flicker_persist_ok)
    ok = ok and fallback_sweep_ok
    print(f"[self-test] {'PASS' if fallback_sweep_ok else 'FAIL'}  FALLBACK sweep (session 31) "
          f"(initial-wait={initial_wait_ok}, cycle-turn-push-wait={cycle_ok}, "
          f"live-contact-early-exit={contact_early_exit_ok}, 720deg-exhausted->STUCK={exhaust_ok}, "
          f"flicker-persist={flicker_persist_ok})")

    # ---- SESSION 46 Chunk 3: FALLBACK must survive and keep running under PLAN-LOST, not just
    #      PLAN-STALE (flight 20260901_124211 never saw a single PLAN-STALE event -- FALLBACK's only OTHER
    #      dispatch, in _step_stale, was structurally unreachable all flight). Without this, a sweep entered
    #      via the wedge-escalation path (Chunk 4) would be wiped back to HOLD_LOST one tick later, exactly
    #      like BACKOFF was in Chunk 2. ----
    lost_plan = {"plan_valid": False, "goal": None, "pos": [0.0, 0.0], "clearance_ring": None}
    # (a) THE REGRESSION: a controller already in FALLBACK/TURN, stepped with status=PLAN-LOST, must stay
    #     in FALLBACK and the phase must be free to advance -- fails today (returns HOLD_LOST).
    cpl = ExploreController(cfg, no_takeoff=True)
    cpl._ever_tracked = True
    cpl._enter_fallback_sweep(0.0, None)
    cpl._fallback_phase, cpl._fallback_phase_t0 = "TURN", 0.0
    survived_lost = True
    t, s = 0.0, "FALLBACK"
    for _ in range(int(2.0 / 0.02)):
        _a, s, _ = cpl.step(t, lost_plan, False, status="PLAN-LOST")
        if s != "FALLBACK":
            survived_lost = False
            break
        t += 0.02
        if cpl._fallback_phase != "TURN":     # phase advanced past TURN -> proves it isn't frozen/wiped
            break
    phase_advanced_ok = cpl._fallback_phase != "TURN" or cpl._fallback_cum_deg > 0.0
    regression3_fixed_ok = survived_lost and phase_advanced_ok
    # (b) the live flow contacts are actually THREADED through under PLAN-LOST, not dropped: a matching
    #     wall_contact mid-forward-push still cuts the push short, exactly like under PLAN-STALE.
    cplc = ExploreController(cfg, no_takeoff=True)
    cplc._ever_tracked = True
    cplc.fallback_initial_wait_s = 0.0
    cplc.fallback_push_fwd_back_s = 1.0
    cplc._enter_fallback_sweep(0.0, None)
    cplc._fallback_phase, cplc._fallback_phase_t0 = "PUSH", 0.0
    cplc._fallback_push_dirn = "forward"
    cplc.step(0.0, lost_plan, False, status="PLAN-LOST")               # builds the push player
    cplc.step(0.01, lost_plan, True, status="PLAN-LOST")               # wall_contact=True mid-push
    contact_threaded_ok = cplc._fallback_phase == "WAIT_POST"
    # (c) scope constraint: _step_stale's OWN "if st == FALLBACK" dispatch (PLAN-STALE) is untouched --
    #     the existing session-31/29 tests above (fallback_sweep_ok) already assert this stays green;
    #     restate it here so a Chunk-3 regression is visible in THIS block's own pass/fail too.
    stale_path_untouched_ok = fallback_sweep_ok
    s46c3_ok = regression3_fixed_ok and contact_threaded_ok and stale_path_untouched_ok
    ok = ok and s46c3_ok
    print(f"[self-test] {'PASS' if s46c3_ok else 'FAIL'}  SESSION-46 wedge escalation Chunk3 "
          f"(FALLBACK survives PLAN-LOST + phase advances={regression3_fixed_ok}, "
          f"live contact still threaded through={contact_threaded_ok}, "
          f"PLAN-STALE path unchanged={stale_path_untouched_ok})")

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

    # (g5) CATEGORY C: SLAM_HOLD resuming to ADVANCE shares the SAME two-gate check.
    #      SESSION 56 REWRITE. Gate 1 used to be "a full window of FAST frames"; it is now CURRENCY --
    #      a solve of a frame CAPTURED at/after the gate opened, however long that solve took. So a
    #      slow-but-CURRENT stream no longer holds, it resumes, and the old "stays held while SLAM is
    #      slow" assertion asserts the dead band session 56 deleted on purpose. Worse, it was only
    #      still reading PASS by accident: under sustained-slow SLAM the drone was oscillating
    #      ADVANCE<->SLAM_HOLD every tick and the 20-tick snapshot happened to land on a SLAM_HOLD
    #      tick -- which is precisely the limit cycle session 56's release grace exists to remove.
    #      Both gates are still asserted here, each on the condition that actually gates it now.
    def _cc_plan(fid, cap, ms):
        return {"plan_valid": True, "goal": [1.0, 0.0], "pos": [0.0, 0.0], "bearing_err": 0.0,
                "forward_clearance_dist": 5.0, "frame_id": fid, "cap_ts": cap, "slam_ms": ms}
    # C-1: real cap_ts on every frame, but every capture PREDATES the gate -> currency unmet -> HOLD.
    #      (Deliberately not a cap_ts=None blackout: this proves the gate reads the capture INSTANT,
    #      not merely the presence of a timestamp.) Runs 1.0s, far short of slam_slow_hop_after_s,
    #      so the hold under test is the GATE holding, never the backstop.
    cc24 = ExploreController(cfg, no_takeoff=True)
    cc24.settle_gate_s = 0.05
    cc24._enter_slam_hold("ADVANCE", 0.0, "test")
    t = 0.0
    for i in range(20):
        t += 0.05
        cc24.step(t, _cc_plan(i, -1.0, 1500.0), False)
    catC_holds_while_stale = (cc24.state == "SLAM_HOLD")
    # C-2: CURRENT captures, still SLOW solves -> both gates clear -> resumes to ADVANCE.
    cc24b = ExploreController(cfg, no_takeoff=True)
    cc24b.settle_gate_s = 0.05
    cc24b._enter_slam_hold("ADVANCE", 0.0, "test")
    t, s_c = 0.0, "SLAM_HOLD"
    for i in range(20):
        t += 0.05
        _a, s_c, _ = cc24b.step(t, _cc_plan(i, t, 1500.0), False)
        if s_c != "SLAM_HOLD":
            break
    catC_resumes = (s_c == "ADVANCE" and t < cc24b.slam_slow_hop_after_s)
    category_c_ok = catC_holds_while_stale and catC_resumes

    gate24_ok = freshness_ok and motion_gate_ok and category_a_ok and category_b_ok and category_c_ok
    ok = ok and gate24_ok
    print(f"[self-test] {'PASS' if gate24_ok else 'FAIL'}  settle-gate two-gate design "
          f"(freshness={freshness_ok}, motion={motion_gate_ok}, category-A-full-dwell={category_a_ok}, "
          f"category-B-no-double-wait={category_b_ok}, category-C-currency-gated={category_c_ok})")

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
    csb2.use_slam_stepback_on_slow = True   # exercise the legacy step-back path (default-off since session 35)
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
    csb3.use_slam_stepback_on_slow = True
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
    csb4.use_slam_stepback_on_slow = True
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
    csb5.use_slam_stepback_on_slow = True
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
    csp.use_slam_stepback_on_slow = True
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

    # ---- Session 53: SLAM_HOLD forced-hop rescue must accumulate ACROSS a PLAN-LOST/HOLD_LOST bounce
    #      (bug diagnosed off flight 20260902_143207: SLAM ran ~3.1s/frame -- too slow to hold a plan
    #      valid past plan_timeout_s (3.0s) but fast enough to keep re-locking, so status flip-flopped
    #      OK<->PLAN-LOST every ~3.3s for 2m21s. `_slam_hold_start` is re-stamped on EVERY `_enter_slam_
    #      hold`, so the session-43 rescue's `waited` never got past ~3.0s against the 15.0s bar: 43
    #      bounces, 0 forced hops. `_slam_hold_episode_t0` must survive the bounce the same way
    #      `_slam_stepback_count` already does, proven above.) ----
    padv53 = {"plan_valid": True, "done": False, "goal": [9.0, 0.0], "pos": [0.0, 0.0], "bearing_err": 0.0,
              "forward_clearance_dist": 9.0}

    def _tick53(ctrl, t, fid, status, ms=3155.0):
        p = dict(padv53, frame_id=fid, slam_ms=ms, cap_ts=t)
        _a, s, ev = ctrl.step(t, p, False, status=status)
        return s, ev, t + 0.05, fid + 1

    cs53 = ExploreController(cfg, no_takeoff=True)
    cs53.slam_slow_hop_after_s = 0.5     # shrink the real 15.0s bar so the bounce test runs fast
    cs53._enter("HOLD_LOST", 0.0)        # mirrors the flight: recovering from a fresh loss
    t53, fid53 = 0.0, 0

    # First OK tick: HOLD_LOST (a _RECOVERY_STATE) + status OK -> _enter_slam_hold, OPENS the episode.
    s53, _, t53, fid53 = _tick53(cs53, t53, fid53, "OK")
    ep0_53 = cs53._slam_hold_episode_t0
    first_entry_ok = (s53 == "SLAM_HOLD" and ep0_53 is not None)

    # Bounce: a run of slow-but-OK ticks (never fast enough to clear the settle gate) <-> ONE PLAN-LOST
    # tick <-> ONE OK tick that re-enters a FRESH SLAM_HOLD, repeated until the forced hop fires. Every
    # tick in between must PRESERVE the episode clock (never reset it to a later value).
    bounce_preserved, saw_hop_53, hop_event_53 = True, False, None
    for _cycle in range(20):
        for _ in range(4):
            s53, ev53, t53, fid53 = _tick53(cs53, t53, fid53, "OK")
            if s53 == "REPLAN":
                saw_hop_53, hop_event_53 = True, ev53
                break
            if not (s53 == "SLAM_HOLD" and cs53._slam_hold_episode_t0 == ep0_53):
                bounce_preserved = False
        if saw_hop_53:
            break
        s_lost_53, _, t53, fid53 = _tick53(cs53, t53, fid53, "PLAN-LOST")
        if not (s_lost_53 == "HOLD_LOST" and cs53._slam_hold_episode_t0 == ep0_53):
            bounce_preserved = False
        s53, ev53, t53, fid53 = _tick53(cs53, t53, fid53, "OK")
        if s53 == "REPLAN":
            saw_hop_53, hop_event_53 = True, ev53
            break
        if not (s53 == "SLAM_HOLD" and cs53._slam_hold_episode_t0 == ep0_53):
            bounce_preserved = False

    hop_fires_ok = (saw_hop_53 and hop_event_53 is not None
                     and "forcing" in hop_event_53 and "hop" in hop_event_53)
    # The forced hop only calls _enter("REPLAN", now) and returns; the episode-clock reset lives in the
    # REPLAN handler itself, which runs on the NEXT step() call once state == "REPLAN" -- one more tick.
    _s_replan_53, _, t53, fid53 = _tick53(cs53, t53, fid53, "OK")
    episode_reset_ok = (cs53._slam_hold_episode_t0 is None)

    slamhold_episode_ok = first_entry_ok and bounce_preserved and hop_fires_ok and episode_reset_ok
    ok = ok and slamhold_episode_ok
    print(f"[self-test] {'PASS' if slamhold_episode_ok else 'FAIL'}  SLAM_HOLD EPISODE clock PERSISTENCE "
          f"(first-entry={first_entry_ok}, bounce-preserves={bounce_preserved}, forced-hop-fires={hop_fires_ok}, "
          f"REPLAN-resets={episode_reset_ok})")

    # ---- Session 35: forced-hop escape (default) vs legacy step-back, selected by use_slam_stepback_on_slow ----
    padv35 = {"plan_valid": True, "done": False, "goal": [9.0, 0.0], "pos": [0.0, 0.0], "bearing_err": 0.0,
              "forward_clearance_dist": 9.0}

    def _drive_to_advance(ctrl, tb=0.0, fb=0):
        reached = False
        for _ in range(40):
            _a, s, _ = ctrl.step(tb, dict(padv35, frame_id=fb, slam_ms=200.0), False, status="OK")
            tb += 0.05; fb += 1
            if s == "ADVANCE":
                reached = True
                break
        return reached, tb, fb

    # (a) default (use_slam_stepback_on_slow=False): a sustained slow-but-OK hold forces a hop (REPLAN), and
    #     NEVER falls into SLAM_STEPBACK even well past slam_stepback_after_frames.
    c35a = ExploreController(cfg, no_takeoff=True)
    c35a.slam_slow_hop_after_s = 0.05
    c35a.slam_stepback_after_frames = 3
    reached_a, tb, fb = _drive_to_advance(c35a)
    saw_stepback_a, saw_hop_a = False, False
    for _ in range(15):
        _a, s, _ = c35a.step(tb, dict(padv35, frame_id=fb, slam_ms=1500.0, cap_ts=tb), False, status="OK")
        tb += 0.05; fb += 1
        if s == "SLAM_STEPBACK":
            saw_stepback_a = True
        if s == "REPLAN":
            saw_hop_a = True
            break
    default_hops_ok = reached_a and saw_hop_a and not saw_stepback_a

    # (b) use_slam_stepback_on_slow=True: the SAME sustained slow-but-OK hold now goes through step-back, and
    #     NEVER forces a hop (REPLAN) even with slam_slow_hop_after_s set to fire almost instantly.
    c35b = ExploreController(cfg, no_takeoff=True)
    c35b.use_slam_stepback_on_slow = True
    c35b.slam_slow_hop_after_s = 0.05
    reached_b, tb, fb = _drive_to_advance(c35b)
    saw_replan_b, saw_stepback_b = False, False
    for _ in range(15):
        _a, s, _ = c35b.step(tb, dict(padv35, frame_id=fb, slam_ms=1500.0, cap_ts=tb), False, status="OK")
        tb += 0.05; fb += 1
        if s == "REPLAN":
            saw_replan_b = True
            break
        if s == "SLAM_STEPBACK":
            saw_stepback_b = True
            break
    legacy_never_hops_ok = reached_b and saw_stepback_b and not saw_replan_b

    # (c) session 43 (operator ask, off flight 20260723_000631): the forced hop now ALSO fires for a
    #     recovery-settle hold (`_slam_resume == "SETTLE"`, `_recovering=True`) -- not just a plain
    #     ADVANCE-resume hold -- since a slow settle-gate under plan OK is a throughput signal (session
    #     28/42), not evidence the pose itself is wrong. Firing it must ALSO clear `_recovering`/history
    #     state, mirroring the normal settle-gate-clear trust-restoration (so a forced hop never leaves
    #     the flight permanently "untrusted").
    c35c = ExploreController(cfg, no_takeoff=True)
    c35c.slam_slow_hop_after_s = 0.05
    c35c._recovering = True
    c35c._history_broken = True
    c35c.command_history.append(("fwd", 1.0))
    c35c._enter("HOLD_LOST", 0.0)
    _a, s_c, _ = c35c.step(0.0, dict(padv35, frame_id=1, slam_ms=1500.0, cap_ts=0.0), False, status="OK")
    resume_settle_ok = (s_c == "SLAM_HOLD" and c35c._slam_resume == "SETTLE")
    saw_hop_c, t = False, 0.05
    for i in range(2, 12):
        _a, s, _ = c35c.step(t, dict(padv35, frame_id=i, slam_ms=1500.0, cap_ts=t), False, status="OK"); t += 0.05
        if s == "REPLAN":
            saw_hop_c = True
            break
    recovery_now_hops_ok = (resume_settle_ok and saw_hop_c and not c35c._recovering
                            and not c35c._history_broken and len(c35c.command_history) == 0)

    # (d) grace-window leak fix: a physical guard (clearance stand-off) cutting the forced hop short into
    #     BACKOFF must clear `_slam_slow_hop_deadline` immediately -- a LATER, unrelated ADVANCE (still slow)
    #     must NOT inherit the bypass and must correctly re-enter SLAM_HOLD.
    c35d = ExploreController(cfg, no_takeoff=True)
    c35d.slam_slow_hop_after_s = 0.05
    c35d.slam_slow_hop_grace_s = 5.0
    reached_d, tb, fb = _drive_to_advance(c35d)
    for _ in range(15):     # go slow -> SLAM_HOLD(resume=ADVANCE) -> waited>=0.05s -> forces the hop (REPLAN)
        _a, s, _ = c35d.step(tb, dict(padv35, frame_id=fb, slam_ms=1500.0, cap_ts=tb), False, status="OK")
        tb += 0.05; fb += 1
        if s == "REPLAN":
            break
    for _ in range(15):     # drive REPLAN -> ORIENT -> ADVANCE (still slow, riding the grace bypass)
        _a, s, _ = c35d.step(tb, dict(padv35, frame_id=fb, slam_ms=1500.0, cap_ts=tb), False, status="OK")
        tb += 0.05; fb += 1
        if s == "ADVANCE":
            break
    deadline_active_ok = (c35d._slam_slow_hop_deadline is not None and tb < c35d._slam_slow_hop_deadline
                          and s == "ADVANCE")
    _a, s_bo, _ = c35d.step(tb, dict(padv35, frame_id=fb, slam_ms=1500.0, forward_clearance_dist=0.5),
                            False, status="OK")
    backoff_ok = (s_bo == "BACKOFF")
    deadline_cleared_ok = c35d._slam_slow_hop_deadline is None
    # a later, unrelated ADVANCE (simulating a fresh leg reached after BACKOFF/SETTLE/REPLAN resolve) must
    # NOT inherit the cleared bypass -- still slow -> correctly re-enters SLAM_HOLD.
    c35d.state = "ADVANCE"
    c35d._player = None
    c35d.t_state = tb
    _a, s_new, _ = c35d.step(tb + 0.1, dict(padv35, frame_id=fb + 1, slam_ms=1500.0), False, status="OK")
    no_leak_ok = (s_new == "SLAM_HOLD")

    grace_leak_ok = (deadline_active_ok and backoff_ok and deadline_cleared_ok and no_leak_ok)
    switch_ok = default_hops_ok and legacy_never_hops_ok and recovery_now_hops_ok and grace_leak_ok
    ok = ok and switch_ok
    print(f"[self-test] {'PASS' if switch_ok else 'FAIL'}  SLAM-slow strategy switch (session 35, "
          f"simplified session 43) (default->hop-not-stepback={default_hops_ok}, "
          f"stepback-mode->never-hops={legacy_never_hops_ok}, "
          f"recovery-settle-now-hops-and-restores-trust={recovery_now_hops_ok}, "
          f"grace-window-leak-fix={grace_leak_ok})")

    # ---- Session 34: proactive clearance checks that don't wait for ADVANCE to re-check ----
    # (a) Idea B: a fresh PLAN-LOST with a cached close last-good clearance -> immediate BACKOFF, using the
    #     CACHED position (the live plan's pos is None during a loss).
    c34a = ExploreController(cfg, no_takeoff=True)
    c34a.loss_backoff_grace_s = 0.0   # session 48 timing is tested in its own block
    c34a.leg_goal = [5.0, 5.0]
    ok_plan_close = {"plan_valid": True, "pos": [1.0, 1.0], "forward_clearance_dist": 0.5,
                      "goal": [5.0, 5.0], "done": False, "bearing_err": 0.0, "slam_ms": 100.0, "frame_id": 1}
    c34a.step(0.0, ok_plan_close, False, status="OK")             # caches last-good pos/clearance
    _a, s34a, ev34a = c34a.step(0.02, {"plan_valid": False}, False, status="PLAN-LOST")
    # Session 57: PLAN-LOST now routes through `_step_lost_recovery`, whose event text names the cached
    # clearance directly rather than the old "(stale pose, N.Ns old)" age suffix -- see C6.
    lost_immediate_ok = (s34a == "BACKOFF" and c34a._last_bump_anchor == [1.0, 1.0]
                          and ev34a is not None and "cached clearance" in ev34a)

    # (b) Idea B: same, but the episode STARTS as PLAN-STALE (perception publishing, SLAM not TRACKING).
    c34b = ExploreController(cfg, no_takeoff=True)
    c34b.loss_backoff_grace_s = 0.0   # session 48 timing is tested in its own block
    c34b.leg_goal = [5.0, 5.0]
    c34b._ever_tracked = True     # a MID-FLIGHT loss, not startup warmup
    c34b.step(0.0, ok_plan_close, False, status="OK")
    _a, s34b, _ = c34b.step(0.02, {"plan_valid": False}, False, status="PLAN-STALE")
    stale_immediate_ok = (s34b == "BACKOFF" and c34b._last_bump_anchor == [1.0, 1.0])

    # (c) a LOST->STALE flicker within the SAME episode fires only ONCE (one bump, not two). Session 57:
    #     PLAN-LOST no longer shares `_loss_snapshot_checked` with PLAN-STALE (that ticket is now
    #     PLAN-STALE-only, see `_step_lost_recovery`'s docstring) -- the shared protection against a
    #     flicker re-fire is now `_loss_episode_t0` itself, restamped by the LOST back-off and read by
    #     BOTH `_step_lost_recovery`'s own grace AND `_maybe_loss_snapshot_backoff`'s pre-existing
    #     session-48 grace clause. That only holds with a REAL (non-zero) grace window, so this case
    #     alone uses one, unlike its siblings which zero it out to isolate the instant-fire logic.
    c34c = ExploreController(cfg, no_takeoff=True)
    c34c.loss_backoff_grace_s = 0.5
    c34c.leg_goal = [5.0, 5.0]
    c34c.step(0.0, ok_plan_close, False, status="OK")
    _a, s34c0, _ = c34c.step(0.02, {"plan_valid": False}, False, status="PLAN-LOST")   # fresh edge -> defers
    inside_grace_ok = (s34c0 == "HOLD_LOST")
    _a, s34c1, _ = c34c.step(0.6, {"plan_valid": False}, False, status="PLAN-LOST")   # past grace -> BACKOFF
    g1, _r1, _p1, _ic1 = c34c.take_bump_pulse()
    first_fire_ok = (s34c1 == "BACKOFF" and g1 is not None)
    c34c._bump_armed = True     # re-arm the SEPARATE bump latch, to isolate the field under test
    c34c.state = "HOLD_LOST"    # simulate falling back to HOLD_LOST, then flickering to STALE (same episode)
    _a, s34c2, _ = c34c.step(0.65, {"plan_valid": False}, False, status="PLAN-STALE")   # restamped grace holds
    g2, _r2, _p2, _ic2 = c34c.take_bump_pulse()
    no_double_fire_ok = (inside_grace_ok and first_fire_ok and s34c2 != "BACKOFF" and g2 is None)

    # (d) a cached clearance that ISN'T close -> falls through to the normal HOLD_LOST entry unchanged.
    c34d = ExploreController(cfg, no_takeoff=True)
    c34d.loss_backoff_grace_s = 0.0   # session 48 timing is tested in its own block
    c34d.leg_goal = [5.0, 5.0]
    ok_plan_clear = dict(ok_plan_close, forward_clearance_dist=5.0)
    c34d.step(0.0, ok_plan_clear, False, status="OK")
    _a, s34d, _ = c34d.step(0.02, {"plan_valid": False}, False, status="PLAN-LOST")
    clear_no_fire_ok = (s34d == "HOLD_LOST")

    # (e) no cached pose yet (a loss before any valid plan, e.g. at startup) -> no-op, normal HOLD_LOST entry.
    c34e = ExploreController(cfg, no_takeoff=True)
    c34e.loss_backoff_grace_s = 0.0   # session 48 timing is tested in its own block
    _a, s34e, _ = c34e.step(0.0, {"plan_valid": False}, False, status="PLAN-LOST")
    no_cache_ok = (s34e == "HOLD_LOST" and c34e._last_good_clearance is None)

    ideaB_ok = (lost_immediate_ok and stale_immediate_ok and no_double_fire_ok
                and clear_no_fire_ok and no_cache_ok)

    # (f) Idea A: at the recovery settle-gate-clear (resume target "SETTLE"), a too-close LIVE clearance ->
    #     BACKOFF instead of resuming to SETTLE/REPLAN.
    c34f = ExploreController(cfg, no_takeoff=True)
    c34f.loss_backoff_grace_s = 0.0   # session 48 timing is tested in its own block
    c34f.settle_gate_s = 0.02
    c34f.leg_goal = [5.0, 5.0]
    c34f._enter("HOLD_LOST", 0.0)
    padv_rec = {"plan_valid": True, "done": False, "goal": [5.0, 5.0], "pos": [1.0, 1.0], "bearing_err": 0.0,
                "forward_clearance_dist": 0.5, "slam_ms": 100.0}
    t = 0.0
    _a, s_hold, _ = c34f.step(t, dict(padv_rec, frame_id=1), False, status="OK"); t += 0.05
    entered_slam_hold_ok = (s_hold == "SLAM_HOLD" and c34f._slam_resume == "SETTLE")
    s_final = s_hold
    for i in range(2, 2 + c34f.settle_fresh_frames + 4):
        _a, s_final, _ = c34f.step(t, dict(padv_rec, frame_id=i, cap_ts=t), False, status="OK"); t += 0.05
        if s_final != "SLAM_HOLD":
            break
    recovery_backoff_ok = (entered_slam_hold_ok and s_final == "BACKOFF")

    # (g) regression: the SAME too-close clearance during a PLAIN mid-leg SLAM-slow hold (resume target
    #     "ADVANCE", not a recovery) must NOT be affected -- resumes to ADVANCE unchanged (ADVANCE's own
    #     per-tick stand-off check, unrelated to this session, would catch it moments later anyway).
    c34g = ExploreController(cfg, no_takeoff=True)
    c34g.loss_backoff_grace_s = 0.0   # session 48 timing is tested in its own block
    c34g.settle_gate_s = 0.02
    padv2g = {"plan_valid": True, "done": False, "goal": [5.0, 0.0], "pos": [0.0, 0.0], "bearing_err": 0.0,
              "forward_clearance_dist": 5.0}
    t = 0.0
    for i in range(30):
        _a, s, _ = c34g.step(t, dict(padv2g, frame_id=i, slam_ms=200.0, cap_ts=t), False, status="OK"); t += 0.05
        if s == "ADVANCE":
            break
    _a, s_hold2, _ = c34g.step(t, dict(padv2g, frame_id=100, slam_ms=1500.0, forward_clearance_dist=0.5,
                                       cap_ts=t), False, status="OK"); t += 0.05
    resume_target_ok = (s_hold2 == "SLAM_HOLD" and c34g._slam_resume == "ADVANCE")
    s_final2 = s_hold2
    for i in range(101, 101 + c34g.settle_fresh_frames + 4):
        _a, s_final2, _ = c34g.step(t, dict(padv2g, frame_id=i, slam_ms=200.0, forward_clearance_dist=0.5,
                                            cap_ts=t), False, status="OK"); t += 0.05
        if s_final2 != "SLAM_HOLD":
            break
    plain_hold_unaffected_ok = (resume_target_ok and s_final2 == "ADVANCE")

    # (h) regression: a CLEAR live reading at the recovery settle-gate-clear -> resumes to SETTLE as before.
    c34h = ExploreController(cfg, no_takeoff=True)
    c34h.loss_backoff_grace_s = 0.0   # session 48 timing is tested in its own block
    c34h.settle_gate_s = 0.02
    c34h.leg_goal = [5.0, 5.0]
    c34h._enter("HOLD_LOST", 0.0)
    padv_clear = dict(padv_rec, forward_clearance_dist=5.0)
    t = 0.0
    _a, s_hold3, _ = c34h.step(t, dict(padv_clear, frame_id=1), False, status="OK"); t += 0.05
    s_final3 = s_hold3
    for i in range(2, 2 + c34h.settle_fresh_frames + 4):
        _a, s_final3, _ = c34h.step(t, dict(padv_clear, frame_id=i, cap_ts=t), False, status="OK"); t += 0.05
        if s_final3 != "SLAM_HOLD":
            break
    recovery_clear_regression_ok = (s_final3 == "SETTLE")

    ideaA_ok = recovery_backoff_ok and plain_hold_unaffected_ok and recovery_clear_regression_ok

    proactive_ok = ideaB_ok and ideaA_ok
    ok = ok and proactive_ok
    print(f"[self-test] {'PASS' if proactive_ok else 'FAIL'}  PROACTIVE CLEARANCE while blind (session 34) "
          f"(loss-instant PLAN-LOST={lost_immediate_ok}, loss-instant PLAN-STALE={stale_immediate_ok}, "
          f"no-double-fire-on-flicker={no_double_fire_ok}, clear-no-fire={clear_no_fire_ok}, "
          f"no-cache-startup={no_cache_ok}, recovery-settle-backoff={recovery_backoff_ok}, "
          f"plain-hold-unaffected={plain_hold_unaffected_ok}, "
          f"recovery-settle-clear-regression={recovery_clear_regression_ok})")

    # ---- Session 47: post-backoff SLAM re-solve gate + loss-instant wedge escalation ----
    # Flight 20260901_142738 fired SEVEN loss-instant back-offs in ~2 minutes (two of them 3s apart, one only
    # 1.05s after the previous one ENDED), never once letting SLAM look at where a back-off had put it, and
    # never escalating -- session 46's escalation counted only `_blind_contact_backoff`, a path that flight
    # never took (and structurally cannot: HOLD_LOST/SLAM_HOLD command nothing, so the flow detector produces
    # no verdict there at all).
    def _run_backoff(ctl, t, status="PLAN-LOST", dt=0.05):
        """Drive an in-flight BACKOFF to its natural completion; returns (time, final state)."""
        st = ctl.state
        for _ in range(200):
            _a, st, _e = ctl.step(t, {"plan_valid": False}, False, status=status)
            t += dt
            if st != "BACKOFF":
                break
        return t, st

    # (a) after a back-off completes, the NEXT fresh loss edge must NOT re-fire off pre-backoff evidence --
    #     it holds still until SLAM has solved a frame captured after the back-off ended.
    c47a = ExploreController(cfg, no_takeoff=True)
    c47a.loss_backoff_grace_s = 0.0   # session 48 timing is tested in its own block
    c47a.leg_goal = [5.0, 5.0]
    c47a.step(0.0, ok_plan_close, False, status="OK")                 # caches the too-close clearance
    _a, s47a1, _ = c47a.step(0.02, {"plan_valid": False}, False, status="PLAN-LOST")
    fire1_ok = (s47a1 == "BACKOFF" and c47a._blind_contact_reacts == 1)
    t47, s47a2 = _run_backoff(c47a, 0.07)
    gate_armed_ok = (s47a2 == "HOLD_LOST" and c47a._backoff_resolve_since is not None)
    # The flight's OK->PLAN-LOST flip. State is SETTLE, not HOLD_LOST, exactly as in flight 20260901_142738
    # (`14:29:49.218 SETTLE` -> `14:29:50.265 BACKOFF`) -- the loss-instant check only runs from a NON-hold
    # state, which is why SETTLE's un-satisfiable freshness gate kept feeding it.
    # Session 57: this gate (`_backoff_resolve_since`) lives ONLY in `_maybe_loss_snapshot_backoff` now,
    # which PLAN-LOST no longer calls (see `_step_lost_recovery`'s docstring) -- redrive through
    # PLAN-STALE, the path the gate still guards. This harness's shared `cfg` runs with
    # `use_visual_recovery_on_stale=False` (session 38 isolation, above), so a suppressed geometric
    # back-off correctly falls through to PLAN-STALE's own default recovery, the FALLBACK sweep -- not
    # re-gated by `_backoff_resolve_since` at all -- rather than a bare hold. The assertion that matters
    # is still "no second BACKOFF", not the exact resulting state.
    c47a.state, c47a._was_lost, c47a._bump_armed = "SETTLE", False, True
    _a, s47a3, _ = c47a.step(t47, {"plan_valid": False}, False, status="PLAN-STALE"); t47 += 0.05
    notice47 = c47a.take_notice()
    suppressed_ok = (s47a3 == "FALLBACK" and c47a._blind_contact_reacts == 1
                     and notice47 is not None and "SUPPRESSED" in notice47)

    # (b) ONLY a solve of a frame CAPTURED AFTER the back-off ended opens the gate. A fresh solve of an
    #     OLDER capture is pre-backoff evidence -- exactly what the flight kept re-deciding on -- and at the
    #     3400-3700ms solve latency that flight ran at, in-flight frames captured before the maneuver are
    #     precisely what arrives first. It must NOT open the gate.
    c47a.step(t47, dict(ok_plan_close, frame_id=49, cap_ts=0.0, slam_ms=200.0), False, status="OK"); t47 += 0.05
    stale_capture_holds_ok = (c47a._backoff_resolve_since is not None)
    c47a.step(t47, dict(ok_plan_close, frame_id=50, cap_ts=t47, slam_ms=200.0), False, status="OK"); t47 += 0.05
    gate_opens_ok = (stale_capture_holds_ok and c47a._backoff_resolve_since is None)
    # Session 57: PLAN-STALE again (see the suppressed_ok comment above) -- once the gate is open, Step 1's
    # cached-clearance back-off still fires the SAME as before, since it runs regardless of status.
    c47a.state, c47a._was_lost, c47a._bump_armed = "SETTLE", False, True
    _a, s47a4, _ = c47a.step(t47, {"plan_valid": False}, False, status="PLAN-STALE")
    refire_ok = (gate_opens_ok and s47a4 == "BACKOFF" and c47a._blind_contact_reacts == 2)

    # (c) the budget bounds the wait: a capture stream that never carries a cap_ts cannot hang the trigger
    #     forever -- past backoff_resolve_budget_s the gate opens, LOUDLY (never a silent downgrade).
    c47b = ExploreController(cfg, no_takeoff=True)
    c47b.loss_backoff_grace_s = 0.0   # session 48 timing is tested in its own block
    c47b.leg_goal = [5.0, 5.0]
    c47b.backoff_resolve_budget_s = 1.0
    c47b.step(0.0, ok_plan_close, False, status="OK")
    c47b.step(0.02, {"plan_valid": False}, False, status="PLAN-LOST")
    t47b, _s = _run_backoff(c47b, 0.07)
    # Session 57: PLAN-STALE again (see the suppressed_ok comment above) -- budget_holds_ok lands on the
    # PLAN-STALE default fall-through (FALLBACK) exactly like suppressed_ok; budget_timeout_ok's
    # timeout re-opens the gate and Step 1's cached-clearance back-off fires the SAME as before.
    c47b.state, c47b._was_lost, c47b._bump_armed = "SETTLE", False, True
    _a, s47b1, _ = c47b.step(t47b, {"plan_valid": False}, False, status="PLAN-STALE")   # inside the budget
    budget_holds_ok = (s47b1 == "FALLBACK" and c47b.take_notice() is not None)
    c47b.state, c47b._was_lost, c47b._bump_armed = "SETTLE", False, True
    _a, s47b2, _ = c47b.step(t47b + 2.0, {"plan_valid": False}, False, status="PLAN-STALE")   # past it
    n47b = c47b.take_notice()
    budget_timeout_ok = (s47b2 == "BACKOFF" and c47b._backoff_resolve_since is None
                         and n47b is not None and "TIMED OUT" in n47b)

    # (d) THE session-46 fix, now reachable from the door the drone actually uses: past
    #     blind_contact_escalate_after loss-instant back-offs with no confirmed recovery, escalate to the
    #     FALLBACK sweep (a DIFFERENT maneuver) instead of reversing into the same wall again.
    c47c = ExploreController(cfg, no_takeoff=True)
    c47c.loss_backoff_grace_s = 0.0   # session 48 timing is tested in its own block
    c47c.leg_goal = [5.0, 5.0]
    c47c.blind_contact_escalate_after = 2
    t47c, fid47 = 0.0, 1
    eps47 = []
    for _episode in range(3):
        # one healthy frame between episodes: re-caches the close clearance AND opens the re-solve gate
        c47c.state, c47c._was_lost, c47c._bump_armed = "SETTLE", False, True
        c47c.step(t47c, dict(ok_plan_close, frame_id=fid47, cap_ts=t47c, slam_ms=200.0), False, status="OK")
        fid47 += 1; t47c += 0.05
        c47c.state, c47c._was_lost, c47c._bump_armed = "SETTLE", False, True
        _a, s47c, ev47c = c47c.step(t47c, {"plan_valid": False}, False, status="PLAN-LOST"); t47c += 0.05
        eps47.append((s47c, ev47c))
        if s47c == "BACKOFF":
            t47c, _s = _run_backoff(c47c, t47c)
    escalates_ok = (eps47[0][0] == "BACKOFF" and eps47[1][0] == "BACKOFF" and eps47[2][0] == "FALLBACK"
                    and "WEDGED" in (eps47[2][1] or ""))

    # (e) a wall BEHIND us ends the reverse push instead of grinding the full backoff_hold_s into it.
    c47d = ExploreController(cfg, no_takeoff=True)
    c47d.loss_backoff_grace_s = 0.0   # session 48 timing is tested in its own block
    c47d.leg_goal = [5.0, 5.0]
    c47d.step(0.0, ok_plan_close, False, status="OK")
    _a, s47d0, _ = c47d.step(0.02, {"plan_valid": False}, False, status="PLAN-LOST")
    a_push, _s, _ = c47d.step(0.5, {"plan_valid": False}, False, status="PLAN-LOST")
    pushing_ok = (s47d0 == "BACKOFF" and a_push.get("reverse") == c47d.backoff_reverse_mag)
    a_cut, s_cut, ev_cut = c47d.step(0.7, {"plan_valid": False}, False,
                                     backwall_contact=True, status="PLAN-LOST")
    cut_ok = (pushing_ok and s_cut == "BACKOFF" and a_cut.get("reverse") == 0.0
              and "BACKWALL" in (ev_cut or ""))
    t47d, s_after = 0.75, s_cut
    for _ in range(20):
        _a, s_after, _ = c47d.step(t47d, {"plan_valid": False}, False, status="PLAN-LOST"); t47d += 0.05
        if s_after != "BACKOFF":
            break
    backwall_cut_ok = (cut_ok and s_after == "HOLD_LOST" and t47d < 1.2)   # released early, not at 2.2s

    # (f) session 57: PLAN-LOST's own replacement protection. This gate (`_backoff_resolve_since`) no
    #     longer guards PLAN-LOST at all -- `_step_lost_recovery` restamps `_loss_episode_t0` the instant
    #     a back-off fires (C6), and that restamp is what stops an immediate re-fire on the SAME cached
    #     evidence now. Chunk 4's SESSION-57 PLAN-LOST RECOVERY block (`restamp_blocks_immediate_refire`)
    #     already proves this across a longer window; this is the equivalent check in this gate's own
    #     test, so the file records PLAN-LOST's protection alongside PLAN-STALE's.
    c47e = ExploreController(cfg, no_takeoff=True)
    c47e.loss_backoff_grace_s = 2.0
    c47e.leg_goal = [5.0, 5.0]
    c47e.step(0.0, ok_plan_close, False, status="OK")
    c47e.step(0.02, {"plan_valid": False}, False, status="PLAN-LOST")   # fresh entry, arms the grace
    t47e, s47e = 0.02, "HOLD_LOST"
    while s47e == "HOLD_LOST":
        t47e += 0.1
        _a, s47e, _ = c47e.step(t47e, {"plan_valid": False}, False, status="PLAN-LOST")
    fired_ok = (s47e == "BACKOFF" and c47e._blind_contact_reacts == 1)
    t47e, s47e_after = _run_backoff(c47e, t47e + 0.05)
    _a, s47e_next, _ = c47e.step(t47e, {"plan_valid": False}, False, status="PLAN-LOST")
    plan_lost_restamp_protection_ok = (fired_ok and s47e_after == "HOLD_LOST"
                                       and s47e_next == "HOLD_LOST" and c47e._blind_contact_reacts == 1)

    s47_ok = (fire1_ok and gate_armed_ok and suppressed_ok and stale_capture_holds_ok and refire_ok
              and budget_holds_ok and budget_timeout_ok and escalates_ok and backwall_cut_ok
              and plan_lost_restamp_protection_ok)
    ok = ok and s47_ok
    print(f"[self-test] {'PASS' if s47_ok else 'FAIL'}  SESSION-47 post-backoff re-solve gate "
          f"(1st fires+counts={fire1_ok}, gate armed on completion={gate_armed_ok}, "
          f"next loss edge SUPPRESSED={suppressed_ok}, "
          f"a PRE-backoff capture does NOT open it={stale_capture_holds_ok}, "
          f"post-backoff solve re-enables={refire_ok}, "
          f"budget holds={budget_holds_ok}, budget timeout is LOUD={budget_timeout_ok}, "
          f"loss-instant path escalates to FALLBACK={escalates_ok}, "
          f"BACKWALL cuts the reverse push={backwall_cut_ok}, "
          f"PLAN-LOST restamp blocks immediate re-fire={plan_lost_restamp_protection_ok})")

    # ---- Session 48: the LOSS-RECOVERY GRACE (a loss must OUTLIVE it to earn a physical reaction) ----
    # Measured over all 128 flight logs (2066 episodes): holding still, a loss self-heals in a median 2.4s,
    # p95 9.5s, 96.9% within 12s. Seven of the eight back-offs in flights 20260901_142738/_152201 fired into
    # episodes that recovered by themselves in under 1.1s.
    grace_cfg = copy.deepcopy(cfg)
    grace_cfg["autonomy"]["explore"]["loss_backoff_grace_s"] = 2.0    # scaled down; the RULE is what's tested

    # (a) the loss instant itself no longer reacts -- it holds, and says why, exactly once.
    c48a = ExploreController(grace_cfg, no_takeoff=True)
    c48a.leg_goal = [5.0, 5.0]
    c48a.step(0.0, ok_plan_close, False, status="OK")                 # caches the too-close clearance
    _a, s48a1, _ = c48a.step(0.05, {"plan_valid": False}, False, status="PLAN-LOST")
    n48 = c48a.take_notice()
    holds_ok = (s48a1 == "HOLD_LOST" and c48a._blind_contact_reacts == 0
                and n48 is not None and "LOSS-RECOVERY GRACE" in n48
                and c48a._loss_snapshot_checked is False)      # still ARMED, deliberately not spent
    _a, s48a2, _ = c48a.step(0.10, {"plan_valid": False}, False, status="PLAN-LOST")
    quiet_ok = (holds_ok and s48a2 == "HOLD_LOST" and c48a.take_notice() is None)   # notice is one-shot

    # (b) once the loss OUTLIVES the grace, the HOLD_LOST tick re-runs the deferred check and it fires.
    t48 = 0.15
    s48b = s48a2
    for _ in range(200):
        _a, s48b, _ = c48a.step(t48, {"plan_valid": False}, False, status="PLAN-LOST"); t48 += 0.05
        if s48b == "BACKOFF":
            break
    fires_after_grace_ok = (s48b == "BACKOFF" and t48 >= c48a.loss_backoff_grace_s
                            and c48a._blind_contact_reacts == 1)

    # (c) THE POINT: a loss that recovers inside the grace -- the 1.1s-or-less case that was 7 of the last 8
    #     back-offs -- never reacts at all, and the window re-stamps for the NEXT loss rather than carrying
    #     over (a slow-but-alive SLAM solving every few seconds is NOT lost).
    c48b = ExploreController(grace_cfg, no_takeoff=True)
    c48b.leg_goal = [5.0, 5.0]
    c48b.step(0.0, ok_plan_close, False, status="OK")
    _a, s48c1, _ = c48b.step(0.05, {"plan_valid": False}, False, status="PLAN-LOST")
    _a, s48c2, _ = c48b.step(1.00, {"plan_valid": False}, False, status="PLAN-LOST")   # still inside grace
    c48b.step(1.10, dict(ok_plan_close, frame_id=9, cap_ts=1.10, slam_ms=200.0), False, status="OK")
    episode_cleared_ok = (s48c1 == "HOLD_LOST" and s48c2 == "HOLD_LOST"
                          and c48b._loss_episode_t0 is None and c48b._blind_contact_reacts == 0)
    #     ... and the NEXT loss starts its own fresh window rather than inheriting the spent one.
    c48b.state = "SETTLE"
    _a, s48c3, _ = c48b.step(1.15, {"plan_valid": False}, False, status="PLAN-LOST")
    restamp_ok = (episode_cleared_ok and s48c3 == "HOLD_LOST"
                  and c48b._loss_episode_t0 is not None and abs(c48b._loss_episode_t0 - 1.15) < 1e-6
                  and c48b._blind_contact_reacts == 0)

    # (d) an OK/PLAN-LOST oscillation (flight 20260901_142738: 19 flips, every one of them a "fresh loss")
    #     can NEVER accumulate into a back-off -- each genuine OK re-stamps the window.
    c48c = ExploreController(grace_cfg, no_takeoff=True)
    c48c.leg_goal = [5.0, 5.0]
    t48c, fid48 = 0.0, 1
    s48d = None
    for _ in range(8):                                   # 8 flips, far longer in total than the grace
        c48c.step(t48c, dict(ok_plan_close, frame_id=fid48, cap_ts=t48c, slam_ms=200.0), False, status="OK")
        fid48 += 1; t48c += 0.6
        c48c.state = "SETTLE"
        _a, s48d, _ = c48c.step(t48c, {"plan_valid": False}, False, status="PLAN-LOST"); t48c += 0.6
        if s48d == "BACKOFF":
            break
    oscillation_ok = (s48d == "HOLD_LOST" and c48c._blind_contact_reacts == 0)

    # (e) the decision the grace defers is still made on evidence, not blindly: clear evidence -> no back-off
    #     even long past the window.
    c48d = ExploreController(grace_cfg, no_takeoff=True)
    c48d.leg_goal = [5.0, 5.0]
    c48d.step(0.0, dict(ok_plan_close, forward_clearance_dist=5.0), False, status="OK")
    t48d, s48e = 0.05, None
    for _ in range(200):
        _a, s48e, _ = c48d.step(t48d, {"plan_valid": False}, False, status="PLAN-LOST"); t48d += 0.05
        if s48e == "BACKOFF":
            break
    clear_never_fires_ok = (s48e == "HOLD_LOST" and t48d > c48d.loss_backoff_grace_s * 2)

    s48_ok = (holds_ok and quiet_ok and fires_after_grace_ok and episode_cleared_ok and restamp_ok
              and oscillation_ok and clear_never_fires_ok)
    ok = ok and s48_ok
    print(f"[self-test] {'PASS' if s48_ok else 'FAIL'}  SESSION-48 loss-recovery grace "
          f"(loss instant HOLDS + says why={holds_ok}, notice is one-shot={quiet_ok}, "
          f"fires once the loss outlives the grace={fires_after_grace_ok}, "
          f"recovery inside the grace never reacts={episode_cleared_ok}, next loss re-stamps={restamp_ok}, "
          f"OK/LOST oscillation can never accumulate={oscillation_ok}, "
          f"clear evidence still never fires={clear_never_fires_ok})")

    # ---- SESSION-50: SETTLE's slow-SLAM DEAD-BAND escape -----------------------------------------------
    # Reproduces flight 20260901_172217 exactly: SLAM alive, TRACKING, plan status OK, publishing steadily --
    # but every frame builds in ~1995ms, i.e. ~2x slam_slow_ms (1000). The freshness gate needs
    # settle_fresh_frames CONSECUTIVE sub-1000ms frames, so it can NEVER clear; frames arriving well inside
    # plan_timeout_s mean the status never goes LOST either. That flight sat in SETTLE for 91.6s and only
    # escaped when an inter-frame gap accidentally tripped PLAN-LOST.
    SLOW_MS = 1995.0

    def _settle_wedge_ctrl():
        """A controller parked in a gated SETTLE (-> REPLAN), gate freshly opened, as if a maneuver just ended."""
        c = ExploreController(cfg, no_takeoff=True)
        c.leg_goal = [5.0, 0.0]
        c._settle_to = "REPLAN"
        c._enter("SETTLE", 0.0)      # _enter stamps t_state AND opens the settle gate (prev state != SLAM_HOLD)
        return c

    def _drive_slow_settle(c, seconds, t0, *, cap_ts=True, cap_fixed=None, dt=0.05, seen=None):
        """Drive `c` with a chronically-SLOW but perfectly ALIVE SLAM stream (a fresh frame every tick, status OK).
        `cap_ts=False` reproduces a total capture BLACKOUT (perception publishing nothing timestamped).
        `cap_fixed=<float>` (session 56) pins every frame's capture instant to that value instead of `t`:
        pass something BEFORE the gate opened to reproduce the residual dead band -- real timestamps on
        every frame, so this is NOT a blackout, but no capture is ever current enough to clear the gate.
        `seen` (optional list) collects every state visited -- REPLAN is a one-tick pass-through, so the
        ESCAPE must be asserted on the transition, not on a later snapshot."""
        t, state = t0, c.state
        fid = 5000 + int(t0 * 1000)
        for i in range(int(seconds / dt)):
            fid += 1
            cap = cap_fixed if cap_fixed is not None else (t if cap_ts else None)
            p = {"plan_valid": True, "done": False, "goal": [5.0, 0.0], "pos": [0.0, 0.0],
                 "heading_deg": 0.0, "bearing_err": 0.0, "pos_y": -1.9, "frame_id": fid,
                 "slam_ms": SLOW_MS, "cap_ts": cap}
            _a, state, _ev = c.step(t, p, False, False, status="OK")
            if seen is not None and (not seen or seen[-1] != state):
                seen.append(state)
            t += dt
        return t, state

    # (a) SESSION 56 REWRITE. This block was written when the settle gate demanded a full window of FAST
    #     frames, so a slow-but-alive SETTLE genuinely wedged and ONLY the slam_slow_hop_after_s escape
    #     could free it. The gate now demands a CURRENT frame instead (a solve of a frame CAPTURED at/after
    #     the gate opened, however slow that solve was), and `_drive_slow_settle` feeds cap_ts=t from its
    #     very first tick -- so this is no longer a wedge at all: it clears through the NORMAL gate in
    #     ~settle_gate_s. Asserting the old "still SETTLE after 13s" would be asserting the dead band
    #     session 56 deliberately deleted (the same treatment session 43 gave the session-35 test its own
    #     change contradicted). The block's PURPOSE is unchanged and still fully covered: SETTLE must never
    #     hang forever under slow-but-alive SLAM, and a capture blackout must never earn a forced proceed.
    #
    #     (a1) slow but CURRENT -> out via the NORMAL gate, an order of magnitude inside the backstop.
    cfast_gate = _settle_wedge_ctrl()
    seen_g = []
    _t_g, st_g = _drive_slow_settle(cfast_gate, cfast_gate.settle_gate_s + 0.5, 0.0, seen=seen_g)
    slow_current_exits_ok = ("REPLAN" in seen_g and st_g != "SETTLE")
    #     (a2) THE RESIDUAL DEAD BAND, which is what the backstop is now FOR: every frame carries a real
    #     cap_ts (so this is emphatically not a blackout), but every capture PREDATES the gate, so the
    #     currency floor can never clear and only the wall clock can resolve it.
    cwedge = _settle_wedge_ctrl()
    t_w, st_w = _drive_slow_settle(cwedge, cwedge.slam_slow_hop_after_s - 2.0, 0.0, cap_fixed=-1.0)
    still_settling_ok = (st_w == "SETTLE")
    seen_w = []
    t_w, st_w = _drive_slow_settle(cwedge, 3.0, t_w, cap_fixed=-1.0, seen=seen_w)
    escaped_ok = ("REPLAN" in seen_w and st_w != "SETTLE")
    # The forced proceed must arm the SAME bypass grace SLAM_HOLD's hop does, or the next ORIENT/ADVANCE
    # would immediately re-divert into SLAM_HOLD on these very same slow frames and buy nothing.
    grace_armed_ok = cwedge._slam_slow_hop_active(t_w - 0.05)

    # (b) NOT PREMATURE: a HEALTHY-but-not-yet-settled SETTLE must still clear through the normal gate, never
    #     via the escape -- so a fast pipeline reaches REPLAN long before slam_slow_hop_after_s.
    cfast = _settle_wedge_ctrl()
    t_f, state_f = 0.0, "SETTLE"
    fid_f = 9000
    while t_f < cfast.slam_slow_hop_after_s - 2.0 and state_f == "SETTLE":
        fid_f += 1
        p = {"plan_valid": True, "done": False, "goal": [5.0, 0.0], "pos": [0.0, 0.0], "heading_deg": 0.0,
             "bearing_err": 0.0, "pos_y": -1.9, "frame_id": fid_f, "slam_ms": 300.0, "cap_ts": t_f}
        _a, state_f, _ev = cfast.step(t_f, p, False, False, status="OK")
        t_f += 0.05
    normal_gate_ok = (state_f == "REPLAN" and t_f < cfast.slam_slow_hop_after_s - 1.0)

    # (c) CAPTURE BLACKOUT still holds: a wall clock cannot distinguish "slow but alive" from "perception has
    #     produced NOTHING timestamped". The latter must never earn a forced proceed (session 43's guard).
    cblind = _settle_wedge_ctrl()
    t_b, st_b = _drive_slow_settle(cblind, cblind.slam_slow_hop_after_s + 5.0, 0.0, cap_ts=False)
    blackout_holds_ok = (st_b == "SETTLE")

    s50_ok = (slow_current_exits_ok and still_settling_ok and escaped_ok and grace_armed_ok
              and normal_gate_ok and blackout_holds_ok)
    ok = ok and s50_ok
    print(f"[self-test] {'PASS' if s50_ok else 'FAIL'}  SESSION-50 SETTLE slow-SLAM dead-band escape "
          f"(slow-but-CURRENT exits via the normal gate={slow_current_exits_ok}, stale-capture wedge "
          f"holds before the timeout={still_settling_ok}, forces that {SLOW_MS:.0f}ms wedge out past it="
          f"{escaped_ok}, arms the hop grace={grace_armed_ok}, healthy SLAM still exits via the normal "
          f"gate={normal_gate_ok}, capture blackout still holds={blackout_holds_ok})")

    # ---- VISUAL RECOVERY: 15° rotational probe on PLAN-STALE (session 35 ALT) ----
    cfg_vr = copy.deepcopy(cfg)
    cfg_vr["autonomy"]["explore"]["use_visual_recovery_on_stale"] = True

    # (a) Step 2 loss-instant: cached clearance CLEAR + a CONTAINED visual match -> BACKOFF (the exact
    #     Idea-B gap this session closes: geometry alone reads "clear" when SLAM never mapped the wall).
    # Session 57: this Step-2 visual-ALONE trigger is now PLAN-STALE-only -- PLAN-LOST/NO-PLAN went to
    # `_step_lost_recovery`, whose "pending" gate is clearance-only by design (MISSION CONTEXT's new
    # rule: "the remembered clearance decides only whether a back-off is pending, never whether we
    # look"; the visual verdict ADJUDICATES a pending back-off, it no longer triggers one on its own).
    cva = ExploreController(cfg_vr, no_takeoff=True)
    cva.loss_backoff_grace_s = 0.0   # session 48 timing is tested in its own block
    cva.leg_goal = [5.0, 5.0]
    p_clear = {"plan_valid": True, "pos": [1.0, 1.0], "forward_clearance_dist": 5.0,
              "goal": [5.0, 5.0], "done": False, "bearing_err": 0.0, "slam_ms": 100.0, "frame_id": 1}
    cva.step(0.0, p_clear, False, status="OK")
    # Session 57: closer="LIVE" makes this fixture's evidence agree with the new direction gate (chunk 6) --
    # this test's INTENT (a contained crop that is genuinely too close) is unchanged, it just now has to say
    # so explicitly instead of the old direction-blind default.
    vm_contained = VisualMatch(has_lkg=True, matched=True, inliers=40, contained=True, planar_like=False,
                               scale=1.8, closer="LIVE")
    _a, s_va, ev_va = cva.step(0.02, {"plan_valid": False}, False, status="PLAN-STALE", visual_match=vm_contained)
    visual_contained_ok = (s_va == "BACKOFF" and ev_va is not None and "visual" in ev_va.lower())

    # (b) same, but PLANAR-LIKE (flat-surface) instead of a contained crop -> also BACKOFF.
    cvb = ExploreController(cfg_vr, no_takeoff=True)
    cvb.loss_backoff_grace_s = 0.0   # session 48 timing is tested in its own block
    cvb.leg_goal = [5.0, 5.0]
    cvb.step(0.0, p_clear, False, status="OK")
    # Session 57: same closer="LIVE" fix as vm_contained above -- this fixture's intent is the CLOSE case.
    vm_planar = VisualMatch(has_lkg=True, matched=True, inliers=55, contained=False, planar_like=True,
                            scale=1.0, closer="LIVE")
    _a, s_vb, _ = cvb.step(0.02, {"plan_valid": False}, False, status="PLAN-STALE", visual_match=vm_planar)
    visual_planar_ok = (s_vb == "BACKOFF")

    # (c) a close CACHED CLEARANCE still wins first — visual is never even consulted, even when it would
    #     have said something else (an inconclusive no-match here). Session 57: PLAN-LOST's event text now
    #     names the cached clearance directly (see `_step_lost_recovery`, C6) rather than the old
    #     "(stale pose, N.Ns old)" age suffix.
    p_close = {"plan_valid": True, "pos": [1.0, 1.0], "forward_clearance_dist": 0.5,
              "goal": [5.0, 5.0], "done": False, "bearing_err": 0.0, "slam_ms": 100.0, "frame_id": 1}
    cvc = ExploreController(cfg_vr, no_takeoff=True)
    cvc.loss_backoff_grace_s = 0.0   # session 48 timing is tested in its own block
    cvc.leg_goal = [5.0, 5.0]
    cvc.step(0.0, p_close, False, status="OK")
    vm_none = VisualMatch(has_lkg=True, matched=False)
    _a, s_vc, ev_vc = cvc.step(0.02, {"plan_valid": False}, False, status="PLAN-LOST", visual_match=vm_none)
    clearance_wins_first_ok = (s_vc == "BACKOFF" and ev_vc is not None and "cached clearance" in ev_vc)

    # (d) both loss-instant checks inconclusive (clear cache + no visual match) -> hands off into the 15°
    #     VISUAL_RECOVERY probe (not straight to FALLBACK) since the flag is on -- PLAN-STALE only.
    cvd = ExploreController(cfg_vr, no_takeoff=True); cvd._ever_tracked = True
    cvd.loss_backoff_grace_s = 0.0   # session 48 timing is tested in its own block
    cvd.leg_goal = [5.0, 5.0]
    cvd.step(0.0, p_clear, False, status="OK")
    _a, s_vd, ev_vd = cvd.step(0.02, {"plan_valid": False}, False, status="PLAN-STALE", visual_match=vm_none)
    probe_entered_ok = (s_vd == "VISUAL_RECOVERY" and cvd._visrec_phase == "TURN"
                        and ev_vd is not None and "probe" in ev_vd.lower())

    # (d2) session 42: the SAME both-inconclusive setup, but under PLAN-LOST (perception itself silent,
    #      e.g. a slow-solve backlog) instead of PLAN-STALE -- must NOT hand off into VISUAL_RECOVERY;
    #      falls straight through to the plain HOLD_LOST hard-hover-hold instead. Diagnosed off two real
    #      flights where every VISUAL_RECOVERY entry (62 total) came from PLAN-LOST and reverted to
    #      HOLD_LOST one tick later anyway (`_step_visual_recovery` was never reachable from PLAN-LOST) --
    #      this test locks in the fix as the honest, intentional behavior instead of an accidental one-tick
    #      log artifact.
    cvd2 = ExploreController(cfg_vr, no_takeoff=True); cvd2._ever_tracked = True
    cvd2.loss_backoff_grace_s = 0.0   # session 48 timing is tested in its own block
    cvd2.leg_goal = [5.0, 5.0]
    cvd2.step(0.0, p_clear, False, status="OK")
    _a, s_vd2, ev_vd2 = cvd2.step(0.02, {"plan_valid": False}, False, status="PLAN-LOST", visual_match=vm_none)
    plan_lost_no_probe_ok = (s_vd2 == "HOLD_LOST" and ev_vd2 is not None
                             and "HARD HOVER-HOLD" in ev_vd2)

    def _drive_visrec_turn(ctrl, t0, fid0):
        """Step an in-progress VISUAL_RECOVERY controller through its current TURN sub-phase (turn player +
        inter-action settle, feeding a live fresh-frame stream throughout) until it reaches MATCH awaiting a
        verdict, or falls out of VISUAL_RECOVERY entirely. Returns (t, fid, state)."""
        t, fid = t0, fid0
        for _ in range(4000):
            fid += 1
            p = {"plan_valid": False, "goal": None, "pos": None, "frame_id": fid, "cap_ts": t, "slam_ms": 200.0}
            _a, s, _ = ctrl.step(t, p, False, status="PLAN-STALE", visual_match=vm_none)
            t += 0.02
            if s != "VISUAL_RECOVERY" or ctrl._visrec_phase == "MATCH":
                return t, fid, s
        raise RuntimeError("visrec TURN never reached MATCH")

    # (e) TURN actually commands a real turn (yaw observed), accumulates visrec_turn_step_deg, settles, then
    #     reaches MATCH awaiting a verdict.
    cve = ExploreController(cfg_vr, no_takeoff=True); cve._ever_tracked = True
    cve.loss_backoff_grace_s = 0.0   # session 48 timing is tested in its own block
    cve.leg_goal = [5.0, 5.0]; cve.recovery_settle_frames = 2
    cve.step(0.0, p_clear, False, status="OK")
    cve.step(0.02, {"plan_valid": False}, False, status="PLAN-STALE", visual_match=vm_none)   # enters TURN
    saw_yaw, t, fid = False, 0.04, 100
    s_e = "VISUAL_RECOVERY"
    for _ in range(300):
        fid += 1
        p = {"plan_valid": False, "goal": None, "pos": None, "frame_id": fid, "cap_ts": t, "slam_ms": 200.0}
        a, s_e, _ = cve.step(t, p, False, status="PLAN-STALE", visual_match=vm_none)
        if abs(float(a.get("yaw", 0.0) or 0.0)) > 0:
            saw_yaw = True
        t += 0.02
        if s_e != "VISUAL_RECOVERY" or cve._visrec_phase == "MATCH":
            break
    turn_reached_match_ok = (saw_yaw and s_e == "VISUAL_RECOVERY" and cve._visrec_phase == "MATCH"
                             and cve._visrec_cum_deg == cve.visrec_turn_step_deg)

    # (f) MATCH: a re-match with scale >= visrec_close_scale (closer) -> BACKOFF.
    p_match_f = {"plan_valid": False, "goal": None, "pos": None, "frame_id": fid + 1, "cap_ts": t, "slam_ms": 200.0}
    vm_closer = VisualMatch(has_lkg=True, matched=True, inliers=30, contained=False, planar_like=False, scale=1.5)
    _a, s_close, ev_close = cve.step(t, p_match_f, False, status="PLAN-STALE", visual_match=vm_closer)
    match_closer_backoff_ok = (s_close == "BACKOFF" and ev_close is not None and "closer" in ev_close.lower())

    # (g) MATCH: a re-match with scale < visrec_close_scale (farther/same) -> WAIT_RECOVER, which the
    #     generic OK-convergence (step()'s _RECOVERY_STATES check) breaks out of instantly once status OK.
    cvg = ExploreController(cfg_vr, no_takeoff=True); cvg._ever_tracked = True
    cvg.loss_backoff_grace_s = 0.0   # session 48 timing is tested in its own block
    cvg.leg_goal = [5.0, 5.0]; cvg.recovery_settle_frames = 2
    cvg.step(0.0, p_clear, False, status="OK")
    cvg.step(0.02, {"plan_valid": False}, False, status="PLAN-STALE", visual_match=vm_none)
    t, fid, s_g = _drive_visrec_turn(cvg, 0.04, 100)
    vm_farther = VisualMatch(has_lkg=True, matched=True, inliers=20, contained=False, planar_like=False, scale=0.9)
    p_match_g = {"plan_valid": False, "goal": None, "pos": None, "frame_id": fid + 1, "cap_ts": t, "slam_ms": 200.0}
    _a, s_wait, _ = cvg.step(t, p_match_g, False, status="PLAN-STALE", visual_match=vm_farther)
    wait_entered_ok = (s_wait == "VISUAL_RECOVERY" and cvg._visrec_phase == "WAIT_RECOVER")
    p_ok = {"plan_valid": True, "pos": [1.0, 1.0], "goal": [5.0, 5.0], "bearing_err": 0.0,
           "forward_clearance_dist": 5.0, "frame_id": fid + 2, "cap_ts": t + 0.02, "slam_ms": 100.0}
    _a, s_ok_break, _ = cvg.step(t + 0.02, p_ok, False, status="OK")
    wait_breaks_on_ok_ok = (s_ok_break == "SLAM_HOLD" and cvg._slam_resume == "SETTLE")
    wait_recover_ok = wait_entered_ok and wait_breaks_on_ok_ok

    # (h) WAIT_RECOVER timeout with no re-anchor -> STUCK.
    cvh = ExploreController(cfg_vr, no_takeoff=True); cvh._ever_tracked = True
    cvh.loss_backoff_grace_s = 0.0   # session 48 timing is tested in its own block
    cvh.leg_goal = [5.0, 5.0]; cvh.recovery_settle_frames = 2; cvh.visrec_wait_recover_s = 0.05
    cvh.step(0.0, p_clear, False, status="OK")
    cvh.step(0.02, {"plan_valid": False}, False, status="PLAN-STALE", visual_match=vm_none)
    t, fid, s_h = _drive_visrec_turn(cvh, 0.04, 100)
    p_match_h = {"plan_valid": False, "goal": None, "pos": None, "frame_id": fid + 1, "cap_ts": t, "slam_ms": 200.0}
    _a, s_wait2, _ = cvh.step(t, p_match_h, False, status="PLAN-STALE", visual_match=vm_farther)
    t += 0.1   # exceed visrec_wait_recover_s
    _a, s_stuck, ev_stuck = cvh.step(t, {"plan_valid": False}, False, status="PLAN-STALE")
    wait_timeout_stuck_ok = (s_wait2 == "VISUAL_RECOVERY" and s_stuck == "STUCK"
                             and ev_stuck is not None and "no slam re-anchor" in ev_stuck.lower())

    # (i) turn-budget exhausted (never re-acquires F_LKG — always no-match) -> LOUD event -> FALLBACK hand-off.
    cvi = ExploreController(cfg_vr, no_takeoff=True); cvi._ever_tracked = True
    cvi.loss_backoff_grace_s = 0.0   # session 48 timing is tested in its own block
    cvi.leg_goal = [5.0, 5.0]; cvi.recovery_settle_frames = 2
    cvi.visrec_max_rotation_deg = 2 * cvi.visrec_turn_step_deg   # exhaust after 2 turn steps
    cvi.step(0.0, p_clear, False, status="OK")
    t, fid = 0.02, 200
    _a, s_i, _ = cvi.step(t, {"plan_valid": False}, False, status="PLAN-STALE", visual_match=vm_none)
    last_ev = None
    for _ in range(4000):
        fid += 1
        p = {"plan_valid": False, "goal": None, "pos": None, "frame_id": fid, "cap_ts": t, "slam_ms": 200.0}
        _a, s_i, ev_i = cvi.step(t, p, False, status="PLAN-STALE", visual_match=vm_none)
        if ev_i:
            last_ev = ev_i
        t += 0.02
        if s_i == "FALLBACK":
            break
    exhausted_fallback_ok = (s_i == "FALLBACK" and last_ev is not None and "exhausted" in last_ev.lower())

    # (j) regression: flag OFF reproduces today's (session 34) behavior byte-for-byte — a visual clause
    #     never fires and the probe never enters, even given the SAME too-close visual match as (a).
    cvj = ExploreController(cfg, no_takeoff=True)   # `cfg`, NOT `cfg_vr` -- flag stays at its False default
    cvj.leg_goal = [5.0, 5.0]
    cvj.step(0.0, p_clear, False, status="OK")
    _a, s_vj, _ = cvj.step(0.02, {"plan_valid": False}, False, status="PLAN-LOST", visual_match=vm_contained)
    regression_off_ok = (s_vj == "HOLD_LOST")

    visrec_ok = (visual_contained_ok and visual_planar_ok and clearance_wins_first_ok and probe_entered_ok
                and plan_lost_no_probe_ok and turn_reached_match_ok and match_closer_backoff_ok
                and wait_recover_ok and wait_timeout_stuck_ok and exhausted_fallback_ok and regression_off_ok)
    ok = ok and visrec_ok
    print(f"[self-test] {'PASS' if visrec_ok else 'FAIL'}  VISUAL RECOVERY 15° probe (session 35 ALT) "
          f"(loss-instant contained={visual_contained_ok}, loss-instant planar={visual_planar_ok}, "
          f"clearance-wins-first={clearance_wins_first_ok}, probe-entered={probe_entered_ok}, "
          f"plan-lost-no-probe={plan_lost_no_probe_ok}, "
          f"turn->match={turn_reached_match_ok}, match-closer->backoff={match_closer_backoff_ok}, "
          f"wait-recover-breaks-on-ok={wait_recover_ok}, wait-timeout->stuck={wait_timeout_stuck_ok}, "
          f"exhausted->fallback={exhausted_fallback_ok}, flag-off-regression={regression_off_ok})")

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

    # ---- SESSION 46 Chunk 4: escalate out of a failing blind-contact reflex (flight 20260901_124211: the
    #      drone was WEDGED -- reverse collapsed to 0.000u, BACKWALL latched -- yet BLIND_BACKOFF just
    #      replayed the same failing reflex forever). After blind_contact_escalate_after reactions with no
    #      confirmed recovery between them, escalate into the FALLBACK sweep instead. Calls
    #      _blind_contact_backoff directly (its own return-shape contract), each contact separated by a
    #      contact-clear tick to re-arm -- exactly the (a)/(b) idiom above. ----
    # (a) contacts 1 and 2 -> BLIND_BACKOFF (as before); contact 3 -> escalate to FALLBACK, event says
    #     WEDGED, and the fresh sweep skips straight to TURN (no 20s initial wait).
    c4a = ExploreController(cfg, no_takeoff=True)
    c4a.blind_contact_escalate_after = 2
    c4a._enter("HOLD_LOST", 0.0)
    r1 = c4a._blind_contact_backoff(0.0, True, False, "HOLD_LOST")
    contact1_ok = (r1 is not None and r1[1] == "BLIND_BACKOFF" and c4a._blind_contact_reacts == 1)
    c4a._blind_contact_backoff(0.1, False, False, "HOLD_LOST")     # contact clears -> re-arms
    r2 = c4a._blind_contact_backoff(0.2, True, False, "HOLD_LOST")
    contact2_ok = (r2 is not None and r2[1] == "BLIND_BACKOFF" and c4a._blind_contact_reacts == 2)
    c4a._blind_contact_backoff(0.3, False, False, "HOLD_LOST")     # re-arms again
    r3 = c4a._blind_contact_backoff(0.4, True, False, "HOLD_LOST")
    contact3_escalates_ok = (r3 is not None and r3[1] == "FALLBACK" and "WEDGED" in (r3[2] or "")
                              and c4a._fallback_phase == "TURN" and c4a.state == "FALLBACK")
    test_a_ok = contact1_ok and contact2_ok and contact3_escalates_ok
    # (b) THE FLICKER REGRESSION: a bare status flicker (OK <-> PLAN-LOST, state bouncing through HOLD_LOST,
    #     mirroring the existing session-29 flicker test's cfl29.state-bounce idiom) must NOT reset the
    #     counter -- only a genuine confirmed-recovery site (Chunk 1's reset_leg/trust-boundary/new-goal
    #     sites) may. Flight 20260901_124211's status oscillated OK 3.0s / PLAN-LOST 0.6s for the last
    #     minute; this counter must survive exactly that.
    c4b = ExploreController(cfg, no_takeoff=True)
    c4b.blind_contact_escalate_after = 5     # high enough it never escalates mid-test (that's test (a)'s job)
    c4b._enter("HOLD_LOST", 0.0)
    c4b._blind_contact_backoff(0.0, True, False, "HOLD_LOST")
    after_react1 = c4b._blind_contact_reacts
    c4b._blind_contact_backoff(0.1, False, False, "HOLD_LOST")    # re-arm
    pok4b = {"plan_valid": True, "done": False, "goal": [3.0, 0.0], "pos": [0.0, 0.0], "bearing_err": 0.0,
             "forward_clearance_dist": 9.0, "pos_y": 0.0, "frame_id": 1, "cap_ts": 0.0, "slam_ms": 200.0}
    plost4b = {"plan_valid": False, "done": False, "goal": None, "pos": [0.0, 0.0]}
    c4b.state = "HOLD_LOST"                  # simulate the flicker bounce (mirrors cfl29 above)
    c4b.step(0.2, pok4b, False, status="OK")
    after_ok_flicker = c4b._blind_contact_reacts
    c4b.state = "HOLD_LOST"                  # flicker back to LOST
    c4b.step(0.3, plost4b, False, status="PLAN-LOST")
    after_lost_again = c4b._blind_contact_reacts
    c4b._blind_contact_backoff(0.4, True, False, "HOLD_LOST")     # react again -> increments further
    after_react2 = c4b._blind_contact_reacts
    flicker_survives_ok = (after_react1 == 1 and after_ok_flicker == 1 and after_lost_again == 1
                            and after_react2 == 2)
    c4b.reset_leg()                          # only a GENUINE reset site clears it
    genuine_reset_ok = c4b._blind_contact_reacts == 0
    test_b_ok = flicker_survives_ok and genuine_reset_ok
    # (c) escalating while a sweep is ALREADY in progress must NOT stomp its phase/budget/timer -- only a
    #     FRESH sweep (_fallback_phase is None) skips the initial wait. escalate_after=0 -> the very first
    #     reaction escalates, isolating this from the multi-reaction machinery in test (a).
    c4c = ExploreController(cfg, no_takeoff=True)
    c4c.blind_contact_escalate_after = 0
    c4c._fallback_phase = "WAIT_POST"
    c4c._fallback_cum_deg = 45.0
    c4c._fallback_phase_t0 = 10.0
    c4c._enter("HOLD_LOST", 0.0)
    r_inprog = c4c._blind_contact_backoff(0.0, True, False, "HOLD_LOST")
    test_c_ok = (r_inprog is not None and r_inprog[1] == "FALLBACK"
                 and c4c._fallback_phase == "WAIT_POST" and c4c._fallback_cum_deg == 45.0
                 and c4c._fallback_phase_t0 == 10.0)
    s46c4_ok = test_a_ok and test_b_ok and test_c_ok
    ok = ok and s46c4_ok
    print(f"[self-test] {'PASS' if s46c4_ok else 'FAIL'}  SESSION-46 wedge escalation Chunk4 "
          f"(2 reflexes then escalate+WEDGED+skips-initial-wait={test_a_ok}, "
          f"flicker cannot reset (only genuine recovery can)={test_b_ok}, "
          f"escalation preserves an in-progress sweep's phase/budget/timer={test_c_ok})")

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

    # ---- SESSION-58 DEAD-GOAL BUMP GUARD -----------------------------------------------------------
    # 22:40:24.170 BLACKLIST PERMANENT retired goal=[3.9,-3.6] -> 22:40:25.165 bump pulse #2 fired against
    # that SAME dead goal and backed off, because neither _register_bump nor the SLAM_HOLD settle-resume
    # clearance check re-checked the live blacklist before defending `leg_goal`. `_goal_is_blacklisted`
    # (lifted verbatim from `_trim_resolve_resume`'s old inline dead-goal check) now gates both.
    cd1 = ExploreController(cfg, no_takeoff=True)
    cd1.leg_goal = [3.0, 0.0]
    cd1._bump_armed = True
    cd1._register_bump(_tplan(0.0, goal=(3.0, 0.0), blacklist=[[3.0, 0.0]], blacklist_permanent=[True]),
                       "clearance stand-off")
    no_pulse_goal, _, _, _ = cd1.take_bump_pulse()
    missed1 = cd1.take_missed_bump()
    perm_blacklisted_goal_emits_no_pulse = (no_pulse_goal is None and missed1 is not None
                                            and "blacklisted" in missed1)
    # (b) same setup but SOFT blacklist (not permanent) -> still bumps (soft entries are retryable).
    cd2 = ExploreController(cfg, no_takeoff=True)
    cd2.leg_goal = [3.0, 0.0]
    cd2._bump_armed = True
    cd2._register_bump(_tplan(0.0, goal=(3.0, 0.0), blacklist=[[3.0, 0.0]], blacklist_permanent=[False]),
                       "clearance stand-off")
    soft_pulse_goal, _, _, _ = cd2.take_bump_pulse()
    soft_blacklisted_goal_still_bumps = (soft_pulse_goal == [3.0, 0.0])
    # (c) regression guard: no blacklist arrays at all -> the ordinary path still bumps.
    cd3 = ExploreController(cfg, no_takeoff=True)
    cd3.leg_goal = [3.0, 0.0]
    cd3._bump_armed = True
    cd3._register_bump(_tplan(0.0, goal=(3.0, 0.0)), "clearance stand-off")
    clean_pulse_goal, _, _, _ = cd3.take_bump_pulse()
    clean_goal_still_bumps = (clean_pulse_goal == [3.0, 0.0])
    # (d) a PERMANENT blacklist point farther than goal_area_radius from leg_goal -> still bumps.
    cd4 = ExploreController(cfg, no_takeoff=True)
    cd4.leg_goal = [3.0, 0.0]
    cd4._bump_armed = True
    cd4._register_bump(_tplan(0.0, goal=(3.0, 0.0), blacklist=[[10.0, 10.0]], blacklist_permanent=[True]),
                       "clearance stand-off")
    far_pulse_goal, _, _, _ = cd4.take_bump_pulse()
    guard_respects_radius = (far_pulse_goal == [3.0, 0.0])
    # (e) SLAM_HOLD settle-resume: the clearance stand-off path must not defend a dead leg_goal -- it
    #     replans instead of backing off (recipe follows the g4 cb24 pattern above: gate opens at t=0.0,
    #     settle_fresh_frames of currency fills the settle gate, then one step resolves the resume).
    cd5 = ExploreController(cfg, no_takeoff=True)
    cd5.settle_gate_s = 0.2
    cd5.leg_goal = [3.0, 0.0]
    cd5._enter_slam_hold("SETTLE", 0.0, "test")
    t = 0.0
    for fid in range(cd5.settle_fresh_frames):
        t += 0.05
        cd5._update_slam({"slam_ms": 200.0, "frame_id": fid, "cap_ts": t})
    plan5 = _tplan(0.0, pos=(0.0, 0.0), fcd=0.1, cap=t, fid=100 + cd5.settle_fresh_frames, goal=(3.0, 0.0),
                   blacklist=[[3.0, 0.0]], blacklist_permanent=[True])
    _a5, s5, ev5 = cd5.step(t, plan5, False)
    slam_hold_dead_goal_replans_instead_of_backing_off = (
        s5 == "SETTLE" and cd5._settle_to == "REPLAN" and cd5.leg_goal is None and "BACKOFF" not in (ev5 or ""))
    # (f) identical setup, no blacklist arrays -> the live-goal clearance stand-off path is untouched.
    cd6 = ExploreController(cfg, no_takeoff=True)
    cd6.settle_gate_s = 0.2
    cd6.leg_goal = [3.0, 0.0]
    cd6._enter_slam_hold("SETTLE", 0.0, "test")
    t = 0.0
    for fid in range(cd6.settle_fresh_frames):
        t += 0.05
        cd6._update_slam({"slam_ms": 200.0, "frame_id": fid, "cap_ts": t})
    plan6 = _tplan(0.0, pos=(0.0, 0.0), fcd=0.1, cap=t, fid=100 + cd6.settle_fresh_frames, goal=(3.0, 0.0))
    _a6, s6, ev6 = cd6.step(t, plan6, False)
    slam_hold_live_goal_still_backs_off = (s6 == "BACKOFF")
    dead_goal_guard_ok = (perm_blacklisted_goal_emits_no_pulse and soft_blacklisted_goal_still_bumps
                          and clean_goal_still_bumps and guard_respects_radius
                          and slam_hold_dead_goal_replans_instead_of_backing_off
                          and slam_hold_live_goal_still_backs_off)
    ok = ok and dead_goal_guard_ok
    print(f"[self-test] {'PASS' if dead_goal_guard_ok else 'FAIL'}  SESSION-58 DEAD-GOAL BUMP GUARD "
          f"(perm-blacklisted-no-pulse={perm_blacklisted_goal_emits_no_pulse}, "
          f"soft-still-bumps={soft_blacklisted_goal_still_bumps}, "
          f"clean-still-bumps={clean_goal_still_bumps}, "
          f"respects-radius={guard_respects_radius}, "
          f"slam-hold-dead-goal-replans={slam_hold_dead_goal_replans_instead_of_backing_off}, "
          f"slam-hold-live-goal-backs-off={slam_hold_live_goal_still_backs_off})")

    # ---- SESSION 49 — LKG debug window sink (window/save fail INDEPENDENTLY, no silent fallback) -------
    import tempfile

    class _FakeDiag:
        """Minimal stand-in for AutopilotLog: _visrec_debug_sink only reads .diag_dir/.ts and calls
        .line() -- no need for a real log file / OUTPUT/diag write in a self-test."""
        def __init__(self, diag_dir, ts):
            self.diag_dir = diag_dir
            self.ts = ts

        def line(self, text):
            pass

    canvas49 = np.zeros((10, 10, 3), dtype=np.uint8)
    # `cv2` is shadowed by an unrelated local variable earlier in THIS function -- re-bind the real
    # module under an alias (same underlying module object, so mutating an attribute here is visible
    # to _visrec_debug_sink's own module-level `cv2` reference).
    import cv2 as _cv2mod

    # (a) DEFAULT OFF: with the key ABSENT the CODE default must still be off, and both degradation
    # flags start clean. Session 52: previously this constructed from the live cfg and so asserted
    # "config.yaml currently ships false" -- a different (and much weaker) property than the one the
    # case name claims, which broke the moment the operator armed the window for a real flight.
    cfg_no_window = copy.deepcopy(cfg)
    cfg_no_window["autonomy"]["explore"].pop("visrec_debug_window", None)
    ctrl_a = ExploreController(cfg_no_window, no_takeoff=True)
    default_off_ok = (ctrl_a.visrec_debug_window is False and ctrl_a.visrec_window_failed is False
                      and ctrl_a.visrec_save_failed is False)
    ok = ok and default_off_ok
    print(f"[self-test] {'PASS' if default_off_ok else 'FAIL'}  visrec debug window: default OFF, "
          f"both degradation flags start clean")

    # (b) WINDOW FAILURE IS ISOLATED AND LOUD: a dead display must not stop the PNG evidence (the half
    # that survives the flight).
    tmp_b = tempfile.mkdtemp()
    ctrl_b = ExploreController(cfg, no_takeoff=True)
    ctrl_b.visrec_debug_window = True
    diag_b = _FakeDiag(tmp_b, "20260101_000000")
    orig_imshow = _cv2mod.imshow
    _cv2mod.imshow = lambda *a, **k: (_ for _ in ()).throw(_cv2mod.error("no display"))
    try:
        rel_b = _visrec_debug_sink(ctrl_b, diag_b, canvas49, True, "00-00-00_000", 0)
    finally:
        _cv2mod.imshow = orig_imshow
    window_isolated_ok = (ctrl_b.visrec_window_failed is True and ctrl_b.visrec_save_failed is False
                          and rel_b is not None
                          and os.path.exists(os.path.join(tmp_b, "20260101_000000_visrec", "00-00-00_000.png")))
    ok = ok and window_isolated_ok
    print(f"[self-test] {'PASS' if window_isolated_ok else 'FAIL'}  visrec debug sink: window failure "
          f"isolated (window_failed=True, save_failed stays False, PNG evidence still written)")

    # (c) SAVE FAILURE IS ISOLATED: a disk failure must not touch the window flag.
    tmp_c = tempfile.mkdtemp()
    ctrl_c = ExploreController(cfg, no_takeoff=True)
    ctrl_c.visrec_debug_window = False   # keep the window path inert -- isolate the save half only
    diag_c = _FakeDiag(tmp_c, "20260101_000000")
    orig_imwrite = _cv2mod.imwrite
    _cv2mod.imwrite = lambda *a, **k: (_ for _ in ()).throw(_cv2mod.error("disk full"))
    try:
        rel_c = _visrec_debug_sink(ctrl_c, diag_c, canvas49, True, "00-00-01_000", 0)
    finally:
        _cv2mod.imwrite = orig_imwrite
    save_isolated_ok = (ctrl_c.visrec_save_failed is True and ctrl_c.visrec_window_failed is False
                        and rel_c is None)
    ok = ok and save_isolated_ok
    print(f"[self-test] {'PASS' if save_isolated_ok else 'FAIL'}  visrec debug sink: save failure "
          f"isolated (save_failed=True, window_failed stays False, returns None)")

    # (d) CAP: once saved_count reaches visrec_save_max, no further file is written.
    tmp_d = tempfile.mkdtemp()
    ctrl_d = ExploreController(cfg, no_takeoff=True)
    ctrl_d.visrec_debug_window = False
    ctrl_d.visrec_save_max = 3
    diag_d = _FakeDiag(tmp_d, "20260101_000000")
    rel_d = _visrec_debug_sink(ctrl_d, diag_d, canvas49, True, "00-00-02_000", ctrl_d.visrec_save_max)
    cap_ok = (rel_d is None
             and not os.path.exists(os.path.join(tmp_d, "20260101_000000_visrec", "00-00-02_000.png")))
    ok = ok and cap_ok
    print(f"[self-test] {'PASS' if cap_ok else 'FAIL'}  visrec debug sink: at the save cap, "
          f"no further canvas is written")

    # (e) NO-LOG NO-SAVE: a disabled AutopilotLog (no --log) means diag_dir is None -> the save half is a
    # clean no-op, never a crash.
    diag_e = AutopilotLog(False)
    ctrl_e = ExploreController(cfg, no_takeoff=True)
    ctrl_e.visrec_debug_window = False
    no_log_ok = True
    try:
        rel_e = _visrec_debug_sink(ctrl_e, diag_e, canvas49, True, "00-00-03_000", 0)
        no_log_ok = rel_e is None
    except Exception:
        no_log_ok = False
    ok = ok and no_log_ok
    print(f"[self-test] {'PASS' if no_log_ok else 'FAIL'}  visrec debug sink: a disabled AutopilotLog "
          f"(diag_dir=None) -> clean no-op, no crash")

    # ---- SESSION-58 LKG WINDOW SCOPE: the window is now loss-scoped (opens on a match, closes on the
    # loss->recovered edge) instead of staying up for the whole flight (MISSION CONTEXT finding 2). ----

    # (1) window_open_flag_set: a working imshow leaves visrec_window_open True.
    tmp_f1 = tempfile.mkdtemp()
    ctrl_f1 = ExploreController(cfg, no_takeoff=True)
    ctrl_f1.visrec_debug_window = True
    diag_f1 = _FakeDiag(tmp_f1, "20260101_000000")
    _visrec_debug_sink(ctrl_f1, diag_f1, canvas49, False, "00-00-04_000", 0)
    window_open_flag_set_ok = ctrl_f1.visrec_window_open is True
    ok = ok and window_open_flag_set_ok
    print(f"[self-test] {'PASS' if window_open_flag_set_ok else 'FAIL'}  visrec window: a successful "
          f"show sets visrec_window_open=True")

    # (2) window_open_flag_clear_on_close: _visrec_close_window clears it (no display needed -- destroyWindow
    # is patched to a no-op).
    orig_destroy_f2 = _cv2mod.destroyWindow
    _cv2mod.destroyWindow = lambda *a, **k: None
    try:
        _visrec_close_window(ctrl_f1, diag_f1)
    finally:
        _cv2mod.destroyWindow = orig_destroy_f2
    window_open_flag_clear_ok = ctrl_f1.visrec_window_open is False
    ok = ok and window_open_flag_clear_ok
    print(f"[self-test] {'PASS' if window_open_flag_clear_ok else 'FAIL'}  visrec window: "
          f"_visrec_close_window clears visrec_window_open")

    # (3) close_is_noop_when_never_opened: nothing to close -> no raise, destroyWindow not called.
    tmp_f3 = tempfile.mkdtemp()
    ctrl_f3 = ExploreController(cfg, no_takeoff=True)
    ctrl_f3.visrec_debug_window = True
    diag_f3 = _FakeDiag(tmp_f3, "20260101_000000")
    destroy_calls_f3 = []
    orig_destroy_f3 = _cv2mod.destroyWindow
    _cv2mod.destroyWindow = lambda *a, **k: destroy_calls_f3.append(1)
    try:
        _visrec_close_window(ctrl_f3, diag_f3)
    finally:
        _cv2mod.destroyWindow = orig_destroy_f3
    close_noop_ok = (ctrl_f3.visrec_window_open is False and len(destroy_calls_f3) == 0
                     and ctrl_f3.visrec_window_failed is False)
    ok = ok and close_noop_ok
    print(f"[self-test] {'PASS' if close_noop_ok else 'FAIL'}  visrec window: closing a never-opened "
          f"window is a clean no-op")

    # (4) close_failure_is_loud_and_isolated: destroyWindow raises -> window_failed=True, window_open=False,
    # nothing propagates.
    tmp_f4 = tempfile.mkdtemp()
    ctrl_f4 = ExploreController(cfg, no_takeoff=True)
    ctrl_f4.visrec_debug_window = True
    diag_f4 = _FakeDiag(tmp_f4, "20260101_000000")
    _visrec_debug_sink(ctrl_f4, diag_f4, canvas49, False, "00-00-05_000", 0)   # opens the window
    orig_destroy_f4 = _cv2mod.destroyWindow
    _cv2mod.destroyWindow = lambda *a, **k: (_ for _ in ()).throw(_cv2mod.error("no display"))
    close_failure_ok = True
    try:
        _visrec_close_window(ctrl_f4, diag_f4)
    except Exception:
        close_failure_ok = False
    finally:
        _cv2mod.destroyWindow = orig_destroy_f4
    close_failure_ok = (close_failure_ok and ctrl_f4.visrec_window_failed is True
                        and ctrl_f4.visrec_window_open is False)
    ok = ok and close_failure_ok
    print(f"[self-test] {'PASS' if close_failure_ok else 'FAIL'}  visrec window: close failure is loud "
          f"(window_failed=True) and isolated (does not propagate)")

    # (5) window_failure_still_blocks_open_flag: imshow raises -> visrec_window_open stays False, PNG
    # evidence is still written (reuses case (b)'s setup).
    tmp_f5 = tempfile.mkdtemp()
    ctrl_f5 = ExploreController(cfg, no_takeoff=True)
    ctrl_f5.visrec_debug_window = True
    diag_f5 = _FakeDiag(tmp_f5, "20260101_000000")
    orig_imshow_f5 = _cv2mod.imshow
    _cv2mod.imshow = lambda *a, **k: (_ for _ in ()).throw(_cv2mod.error("no display"))
    try:
        rel_f5 = _visrec_debug_sink(ctrl_f5, diag_f5, canvas49, True, "00-00-06_000", 0)
    finally:
        _cv2mod.imshow = orig_imshow_f5
    window_failure_blocks_open_ok = (ctrl_f5.visrec_window_open is False and rel_f5 is not None)
    ok = ok and window_failure_blocks_open_ok
    print(f"[self-test] {'PASS' if window_failure_blocks_open_ok else 'FAIL'}  visrec window: a failed "
          f"imshow leaves visrec_window_open False while the PNG is still written")

    # ---- SESSION-51: the visual match is computed only when someone can read it, and only when the answer
    # could have CHANGED. run_explore matched on EVERY tick of a loss: ~380 full SIFT+BFMatcher+RANSAC
    # passes across session 48's 12s grace to make ONE decision. -----------------------------------------
    def _mk_gate_ctrl(*, one_shot_spent, phase=None):
        c = ExploreController(cfg, no_takeoff=True)
        c._loss_snapshot_checked = bool(one_shot_spent)
        c._visrec_phase = phase
        return c

    def _gate(c, **kw):
        kw.setdefault("needs_match", True); kw.setdefault("has_frame", True)
        kw.setdefault("loss_edge", False); kw.setdefault("moved_since_match", False)
        kw.setdefault("memo", "cached"); kw.setdefault("memo_age_s", 0.0)
        return _visrec_should_match(c, **kw)

    # (a) GATE A -- the predicate itself, and the trap that makes the AND load-bearing.
    armed, spent = _mk_gate_ctrl(one_shot_spent=False), _mk_gate_ctrl(one_shot_spent=True)
    probing = _mk_gate_ctrl(one_shot_spent=True, phase="MATCH")
    predicate_ok = (armed.wants_visual_match() is True and spent.wants_visual_match() is False
                    and probing.wants_visual_match() is True)
    # THE TRAP: a fresh controller has _loss_snapshot_checked False, so the predicate reads True all
    # through healthy flight. Only the AND with needs_match keeps SIFT off every tracking frame.
    fresh = ExploreController(cfg, no_takeoff=True)
    healthy_ok = (fresh.wants_visual_match() is True                      # predicate alone says yes...
                  and _gate(fresh, needs_match=False) is False)           # ...but the gate still says no
    # Mid-loss with the one-shot SPENT -> skip. Asserted under conditions where GATE B would otherwise say
    # YES (no memo / a stale memo), so this isolates GATE A instead of being masked by the rate-limit.
    spent_skips_ok = (_gate(spent, memo=None) is False
                      and _gate(spent, memo_age_s=99.0) is False)
    gate_a_ok = predicate_ok and healthy_ok and spent_skips_ok

    # (b) GATE B -- the memo is reused only while nothing could have changed.
    memo_reused_ok = _gate(armed, memo_age_s=0.1) is False                # fresh memo, still holding -> reuse
    memo_aged_ok = _gate(armed, memo_age_s=armed.visrec_match_min_interval_s + 0.01) is True
    no_memo_ok = _gate(armed, memo=None) is True                          # nothing to reuse
    # FORCE conditions: each alone must defeat a brand-new memo.
    force_edge_ok = _gate(armed, loss_edge=True, memo_age_s=0.0) is True
    force_moved_ok = _gate(armed, moved_since_match=True, memo_age_s=0.0) is True
    force_probe_ok = _gate(probing, memo_age_s=0.0) is True               # MATCH re-matches AFTER a turn
    gate_b_ok = (memo_reused_ok and memo_aged_ok and no_memo_ok
                 and force_edge_ok and force_moved_ok and force_probe_ok)

    # (c) THE WIN, counted: simulate session 48's 12s grace at 32Hz with the drone holding still (the
    #     grace's own invariant -- no commanded motion), one-shot ARMED throughout, and count real matches.
    csim = _mk_gate_ctrl(one_shot_spent=False)
    dt, matches, memo, memo_t = 1.0 / 32.0, 0, None, 0.0
    t = 0.0
    for i in range(int(12.0 / dt)):
        if _visrec_should_match(csim, needs_match=True, has_frame=True, loss_edge=(i == 0),
                                moved_since_match=False, memo=memo, memo_age_s=(t - memo_t)):
            matches += 1
            memo, memo_t = "cached", t
        t += dt
    ticks = int(12.0 / dt)
    expected = int(12.0 / csim.visrec_match_min_interval_s) + 1          # one per interval, plus the edge
    grace_ok = matches <= expected + 1 and matches < ticks // 10
    print(f"[self-test] {'PASS' if grace_ok else 'FAIL'}  SESSION-51 grace-window cost: {matches} real "
          f"matches over {ticks} ticks of a 12s held-still loss (was {ticks}; cap {expected + 1})")

    s51_ok = gate_a_ok and gate_b_ok and grace_ok
    ok = ok and s51_ok
    print(f"[self-test] {'PASS' if s51_ok else 'FAIL'}  SESSION-51 visual-match gating "
          f"(GATE A predicate + healthy-flight trap={gate_a_ok}, GATE B memo reuse + all 3 force "
          f"conditions={gate_b_ok}, grace-window match count collapses={grace_ok})")

    # ---- SESSION 52 (chunk 3): the loss-instant gate must not speak (or spend the one-shot) before reading
    # the evidence. Diagnosed off flight 20260901_222552 -- at 22:49:51.742 the drone had ALREADY backed off
    # to a clear reading (fwd_clear 0.975 -> 1.625, past stop_clearance_dist 1.25), so `_would_react` was
    # False and no back-off was ever contemplated, yet the old ordering (gate ABOVE the evidence clauses)
    # unconditionally spent the one-shot and printed the SUPPRESSED notice anyway -- a false alarm that also
    # swallowed the whole episode's evaluation (cached-clearance check, F_LKG visual check, AND the
    # VISUAL_RECOVERY hand-off, all of which live below the gate) for the rest of `backoff_resolve_budget_s`.
    cfg_gate52 = copy.deepcopy(cfg)
    cfg_gate52["autonomy"]["explore"]["use_visual_recovery_on_stale"] = True

    def _mk_c52(clearance):
        c = ExploreController(cfg_gate52, no_takeoff=True)
        c._ever_tracked = True
        c.loss_backoff_grace_s = 0.0   # session 48 timing is tested in its own block
        c._last_good_clearance = clearance
        c._last_good_pos = [1.0, 1.0]
        c._last_good_t = 0.0
        return c

    # (52-gate-1) no evidence (clearance clear, no visual match) -> the gate is never even consulted -- falls
    #             straight through to the VISUAL_RECOVERY hand-off, with no false-alarm notice and no bump.
    c52a = _mk_c52(1.625)                    # > stop_clearance_dist (1.25) -- clearance clause reads CLEAR
    now52a = 100.0
    c52a._backoff_resolve_since = now52a - 1.5   # a gate WOULD be open here if it were consulted
    c52a._backoff_resolve_t0 = now52a - 1.5
    _a52a, s52a, ev52a = c52a._maybe_loss_snapshot_backoff({}, now52a, None, status="PLAN-STALE")
    gate1_notice_ok = c52a._pending_notice is None
    gate1_pulse_ok = c52a.take_bump_pulse() == (None, None, None, None)
    gate1_state_ok = s52a == "VISUAL_RECOVERY"
    gate1_ok = gate1_notice_ok and gate1_pulse_ok and gate1_state_ok

    # (52-gate-2) real too-close evidence, inside the budget -> still suppressed, but the one-shot is LEFT
    #             ARMED (not spent) so the check gets a real shot once the gate clears.
    c52b = _mk_c52(0.97)                     # <= stop_clearance_dist -- real too-close evidence
    now52b = 100.0
    c52b._backoff_resolve_since = now52b - 1.5
    c52b._backoff_resolve_t0 = now52b - 1.5
    r52b = c52b._maybe_loss_snapshot_backoff({}, now52b, None, status="PLAN-STALE")
    notice52b = c52b.take_notice()
    gate2_notice_ok = notice52b is not None and "SUPPRESSED" in notice52b
    gate2_armed_ok = c52b._loss_snapshot_checked is False
    gate2_no_backoff_ok = r52b is None        # no state returned at all -- certainly not "BACKOFF"
    gate2_ok = gate2_notice_ok and gate2_armed_ok and gate2_no_backoff_ok

    # (52-gate-3) the suppression notice must not repeat every tick while the gate holds the one-shot
    #             deferred -- `_backoff_gate_noticed` mirrors `_loss_grace_noticed` for exactly this.
    count52c, t52c = 0, now52b
    for _ in range(5):
        t52c += 0.05
        c52b._maybe_loss_snapshot_backoff({}, t52c, None, status="PLAN-STALE")
        if c52b.take_notice() is not None:
            count52c += 1
    gate3_ok = count52c == 0                  # the ONE notice was already popped by gate-2, above

    # (52-gate-4) budget exhausted -> the timeout sentence fires LOUDLY, the gate clears, and the deferred
    #             back-off finally proceeds on the (still valid) too-close evidence.
    c52d = _mk_c52(0.97)
    now52d = 100.0
    c52d._backoff_resolve_since = now52d - (c52d.backoff_resolve_budget_s + 1.0)
    c52d._backoff_resolve_t0 = now52d - (c52d.backoff_resolve_budget_s + 1.0)
    _a52d, s52d, ev52d = c52d._maybe_loss_snapshot_backoff({}, now52d, None, status="PLAN-STALE")
    notice52d = c52d.take_notice()
    gate4_notice_ok = notice52d is not None and "TIMED OUT" in notice52d
    gate4_cleared_ok = c52d._backoff_resolve_since is None
    gate4_state_ok = s52d == "BACKOFF"
    gate4_ok = gate4_notice_ok and gate4_cleared_ok and gate4_state_ok

    s52_ok = gate1_ok and gate2_ok and gate3_ok and gate4_ok
    ok = ok and s52_ok
    print(f"[self-test] {'PASS' if s52_ok else 'FAIL'}  SESSION-52 chunk 3 loss-instant gate ordering "
          f"(no-evidence hand-off reached, no false alarm={gate1_ok}, "
          f"real evidence still suppressed + one-shot left armed={gate2_ok}, "
          f"suppression notice does not repeat={gate3_ok}, "
          f"budget timeout is LOUD + gate clears + back-off proceeds={gate4_ok})")

    # ---- SESSION 52 (chunk 4): make the 15° visual probe reachable, and let it survive a status flicker.
    # Diagnosed off flight 20260901_222552: 57 of the flight's 58 loss episodes BEGAN as PLAN-LOST, which
    # spends the loss-instant one-shot on `_maybe_loss_snapshot_backoff`'s opening tick -- so the
    # PLAN-STALE-only tail hand-off into the probe (session 42) never gets a real shot. `_visrec_probe_armed`
    # gives the probe its OWN latch, polled LATE from `_step_stale` (`_maybe_enter_visual_probe`); the flight's
    # single VISUAL_RECOVERY entry (22:59:04) was then killed 1.5s later (22:59:06.265) by a PLAN-LOST flip
    # before it ever reached MATCH -- the new `st == "VISUAL_RECOVERY"` dispatch under PLAN-LOST fixes that.
    cfg_probe_on = copy.deepcopy(cfg)
    cfg_probe_on["autonomy"]["explore"]["use_visual_recovery_on_stale"] = True

    def _mk_c52p(clearance=3.0, grace_s=1.0):
        c = ExploreController(cfg_probe_on, no_takeoff=True)
        c._ever_tracked = True
        c.loss_backoff_grace_s = grace_s   # scaled down; the RULE (grace elapses -> probe proceeds) is tested
        c._last_good_clearance = clearance
        c._last_good_pos = [1.0, 1.0]
        c._last_good_t = 0.0
        c.leg_goal = [5.0, 5.0]
        return c

    # (52-probe-1) THE 57/58 CASE: loss opens as PLAN-LOST, held by `_step_lost_recovery`'s own grace.
    #              Session 57: PLAN-LOST no longer touches `_loss_snapshot_checked` at all (that ticket is
    #              PLAN-STALE-only now, see `_step_lost_recovery`'s docstring) -- it stays UNSPENT here,
    #              unlike the pre-57 one-shot this comment used to describe.
    c52p1 = _mk_c52p()
    t52p1 = 0.0
    _a, s52p1a, _ = c52p1.step(t52p1, {"plan_valid": False}, False, status="PLAN-LOST")
    probe1_tick1_ok = (s52p1a == "HOLD_LOST" and c52p1._loss_snapshot_checked is False
                       and c52p1._visrec_probe_armed is True)
    t52p1 += c52p1.loss_backoff_grace_s + 0.1     # past the grace
    _a, s52p1b, _ = c52p1.step(t52p1, {"plan_valid": False}, False, status="PLAN-STALE")
    probe1_tick2_ok = (s52p1b == "VISUAL_RECOVERY")
    probe1_ok = probe1_tick1_ok and probe1_tick2_ok

    # (52-probe-2) the probe waits out the grace instead of turning immediately -- a probe TURNS, and
    #              session 48's grace applies to it exactly like a back-off.
    c52p2 = _mk_c52p()
    t52p2 = 0.0
    c52p2.step(t52p2, {"plan_valid": False}, False, status="PLAN-LOST")
    t52p2 += 0.1                                   # still INSIDE the grace
    _a, s52p2a, _ = c52p2.step(t52p2, {"plan_valid": False}, False, status="PLAN-STALE")
    probe2_wait_ok = (s52p2a == "HOLD_LOST" and c52p2._visrec_probe_armed is True)
    t52p2 += c52p2.loss_backoff_grace_s + 0.1      # now past it
    _a, s52p2b, _ = c52p2.step(t52p2, {"plan_valid": False}, False, status="PLAN-STALE")
    probe2_turn_ok = (s52p2b == "VISUAL_RECOVERY")
    probe2_ok = probe2_wait_ok and probe2_turn_ok

    # (52-probe-3) an in-flight probe SURVIVES a PLAN-LOST flicker (was forced back to HOLD_LOST, abandoning
    #              the phase, before this chunk -- the exact 22:59:04 -> 22:59:06 kill).
    c52p3 = _mk_c52p()
    t52p3 = 0.0
    c52p3.step(t52p3, {"plan_valid": False}, False, status="PLAN-LOST")
    t52p3 += c52p3.loss_backoff_grace_s + 0.1
    _a, s52p3_enter, _ = c52p3.step(t52p3, {"plan_valid": False}, False, status="PLAN-STALE")
    entered_probe_ok = (s52p3_enter == "VISUAL_RECOVERY")
    t52p3 += 0.05
    _a, s52p3_flicker, _ = c52p3.step(t52p3, {"plan_valid": False}, False, status="PLAN-LOST")
    probe3_ok = (entered_probe_ok and s52p3_flicker == "VISUAL_RECOVERY"
                and c52p3._visrec_phase is not None)

    # (52-probe-4) a genuine loss-instant back-off (real too-close evidence, grace elapsed, resolve gate
    #              clear) consumes the probe latch -- an episode that already earned a physical reaction
    #              must not also turn-probe on top of it.
    c52p4 = _mk_c52p(clearance=0.3)   # <= stop_clearance_dist -- real too-close evidence
    t52p4 = 0.0
    _a, s52p4a, _ = c52p4.step(t52p4, {"plan_valid": False}, False, status="PLAN-LOST")
    armed_before_ok = (s52p4a == "HOLD_LOST" and c52p4._visrec_probe_armed is True)   # deferred by the grace
    t52p4 += c52p4.loss_backoff_grace_s + 0.1
    _a, s52p4b, _ = c52p4.step(t52p4, {"plan_valid": False}, False, status="PLAN-LOST")
    probe4_ok = (armed_before_ok and s52p4b == "BACKOFF" and c52p4._visrec_probe_armed is False)

    # (52-probe-5) flag OFF reproduces today's behavior byte-for-byte: (52-probe-1) with
    #              use_visual_recovery_on_stale=False lands on the FALLBACK sweep, never the probe.
    cfg_probe_off = copy.deepcopy(cfg)
    cfg_probe_off["autonomy"]["explore"]["use_visual_recovery_on_stale"] = False
    c52p5 = ExploreController(cfg_probe_off, no_takeoff=True)
    c52p5._ever_tracked = True
    c52p5.loss_backoff_grace_s = 1.0
    c52p5._last_good_clearance = 3.0
    c52p5._last_good_pos = [1.0, 1.0]
    c52p5._last_good_t = 0.0
    c52p5.leg_goal = [5.0, 5.0]
    t52p5 = 0.0
    c52p5.step(t52p5, {"plan_valid": False}, False, status="PLAN-LOST")
    t52p5 += c52p5.loss_backoff_grace_s + 0.1
    _a, s52p5, _ = c52p5.step(t52p5, {"plan_valid": False}, False, status="PLAN-STALE")
    probe5_ok = (s52p5 == "FALLBACK")

    s52_probe_ok = probe1_ok and probe2_ok and probe3_ok and probe4_ok and probe5_ok
    ok = ok and s52_probe_ok
    print(f"[self-test] {'PASS' if s52_probe_ok else 'FAIL'}  SESSION-52 chunk 4 visual-probe reachability "
          f"(57/58 PLAN-LOST-opened case reaches the probe={probe1_ok}, "
          f"probe waits out the grace before turning={probe2_ok}, "
          f"an in-flight probe survives a PLAN-LOST flicker={probe3_ok}, "
          f"a genuine back-off consumes the probe latch={probe4_ok}, "
          f"flag OFF is byte-identical to today={probe5_ok})")

    # ---- SESSION 57 (chunk 5): probe_inside_grace_holds -- the NEW grace check Fix A added to the Step
    #      2c hand-off in `_maybe_loss_snapshot_backoff`, exercised by a FRESH PLAN-STALE loss with no
    #      cached clearance and no visual match, so `_would_react` is False and the PRE-EXISTING
    #      session-48 grace clause (gated behind `_would_react`) never fires -- only Fix A's own check can
    #      hold this tick. Asserts the STATE (HOLD_LOST / VISUAL_RECOVERY), not just whether a turn player
    #      was built, since a probe that silently held without changing state would also look like "no turn".
    cfg_probe_grace = copy.deepcopy(cfg)
    cfg_probe_grace["autonomy"]["explore"]["use_visual_recovery_on_stale"] = True
    c57g = ExploreController(cfg_probe_grace, no_takeoff=True)
    c57g._ever_tracked = True
    c57g.loss_backoff_grace_s = 1.0
    c57g.leg_goal = [5.0, 5.0]
    t57g = 0.0
    _a, s57g_in, _ = c57g.step(t57g, {"plan_valid": False}, False, status="PLAN-STALE")
    inside_grace_ok = (s57g_in == "HOLD_LOST")
    t57g += c57g.loss_backoff_grace_s + 0.1
    _a, s57g_out, _ = c57g.step(t57g, {"plan_valid": False}, False, status="PLAN-STALE")
    past_grace_ok = (s57g_out == "VISUAL_RECOVERY")
    probe_inside_grace_holds_ok = inside_grace_ok and past_grace_ok
    ok = ok and probe_inside_grace_holds_ok
    print(f"[self-test] {'PASS' if probe_inside_grace_holds_ok else 'FAIL'}  SESSION-57 chunk 5 "
          f"probe_inside_grace_holds (inside_grace_holds_LOST={inside_grace_ok}, "
          f"past_grace_enters_VISUAL_RECOVERY={past_grace_ok})")

    # ---- SESSION 57 (chunk 5): no_second_grace_variant -- Fix A reuses `_maybe_enter_visual_probe`'s
    #      grace condition VERBATIM rather than writing a second, subtly different comparison (e.g. a
    #      `waited = now - t0; if waited < grace` variant like the pre-existing session-48/step_lost_recovery
    #      clauses use) -- so the exact literal string appears in only the two intended sites.
    _grace_condition_literal = ("self._loss_episode_t0 is not None and now - self._loss_episode_t0 < "
                                "self.loss_backoff_grace_s")
    with open(__file__, "r", encoding="utf-8") as _gf:
        _grace_src = _gf.read()
    _grace_variant_count = _grace_src.count(_grace_condition_literal)
    no_second_grace_variant_ok = (_grace_variant_count <= 2)
    ok = ok and no_second_grace_variant_ok
    print(f"[self-test] {'PASS' if no_second_grace_variant_ok else 'FAIL'}  SESSION-57 chunk 5 "
          f"no_second_grace_variant (grace condition literal appears {_grace_variant_count} time(s), <= 2)")

    # ---- SESSION 57 PLAN-LOST RECOVERY -- `_step_lost_recovery` replaces the one-shot ticket path for
    #      PLAN-LOST/NO-PLAN: always wait `loss_backoff_grace_s`, then always look, and let the
    #      inlier-spread `closer` verdict adjudicate a pending back-off. See plans/session57-spec.md
    #      MISSION CONTEXT -- 56 of 86 loss episodes in flight 20260903_083329 never ran a single match
    #      because the old ticket was spent before `visual_match` could ever be non-None. ----
    def _mk_c57(clearance=3.0, grace_s=1.0):
        c = ExploreController(cfg, no_takeoff=True)
        c._ever_tracked = True
        c.loss_backoff_grace_s = grace_s
        c._last_good_clearance = clearance
        c._last_good_pos = [1.0, 1.0]
        c._last_good_t = 0.0
        c.leg_goal = [5.0, 5.0]
        return c

    # (57-1) inside_grace_holds: clear-front PLAN-LOST inside the grace -> HOLD_LOST, no BACKOFF, and the
    #        grace notice fires exactly once across three consecutive ticks.
    c57_1 = _mk_c57(clearance=3.0, grace_s=5.0)
    t57_1 = 0.0
    _a, s57_1a, _ = c57_1.step(t57_1, {"plan_valid": False}, False, status="PLAN-LOST")
    grace_notices = 1 if c57_1.take_notice() is not None else 0
    for _ in range(2):
        t57_1 += 0.1
        _a, s57_1a, _ = c57_1.step(t57_1, {"plan_valid": False}, False, status="PLAN-LOST")
        if c57_1.take_notice() is not None:
            grace_notices += 1
    s57_1_ok = (s57_1a == "HOLD_LOST" and grace_notices == 1)

    # (57-2) clear_front_past_grace_holds: `_last_good_clearance = 3.0` past the grace -> HOLD_LOST, no
    #        BACKOFF (nothing pending), and NO hold-notice (that notice is only for the LKG verdict).
    c57_2 = _mk_c57(clearance=3.0, grace_s=1.0)
    t57_2 = 0.0
    c57_2.step(t57_2, {"plan_valid": False}, False, status="PLAN-LOST")
    c57_2.take_notice()   # drain the grace notice from the fresh-entry tick, irrelevant to this case
    t57_2 += c57_2.loss_backoff_grace_s + 0.1
    _a, s57_2, _ = c57_2.step(t57_2, {"plan_valid": False}, False, status="PLAN-LOST")
    notice57_2 = c57_2.take_notice()
    s57_2_ok = (s57_2 == "HOLD_LOST" and notice57_2 is None)

    # (57-3/4/5) verdict_{live,equal,unknown}_backs_off: `_last_good_clearance = 0.5` (pending), past the
    #            grace, with `closer` LIVE / EQUAL / UNKNOWN -> BACKOFF every time (weak CV never vetoes).
    def _mk_vm(closer, size_ratio=1.5):
        return VisualMatch(has_lkg=True, matched=True, inliers=30, contained=False, planar_like=False,
                           scale=1.2, size_ratio=size_ratio, closer=closer)

    def _run_verdict(closer):
        c = _mk_c57(clearance=0.5, grace_s=1.0)
        t = 0.0
        c.step(t, {"plan_valid": False}, False, status="PLAN-LOST")
        t += c.loss_backoff_grace_s + 0.1
        _a, s, _ = c.step(t, {"plan_valid": False}, False, status="PLAN-LOST", visual_match=_mk_vm(closer))
        return c, s

    _c57_live, s57_live = _run_verdict("LIVE")
    _c57_equal, s57_equal = _run_verdict("EQUAL")
    _c57_unknown, s57_unknown = _run_verdict("UNKNOWN")
    s57_3_ok = s57_live == "BACKOFF"
    s57_4_ok = s57_equal == "BACKOFF"
    s57_5_ok = s57_unknown == "BACKOFF"

    # (57-6) verdict_lkg_holds: same but `closer="LKG"` -> HOLD_LOST, no BACKOFF, hold notice fired once.
    c57_6 = _mk_c57(clearance=0.5, grace_s=1.0)
    t57_6 = 0.0
    c57_6.step(t57_6, {"plan_valid": False}, False, status="PLAN-LOST")
    t57_6 += c57_6.loss_backoff_grace_s + 0.1
    vm_lkg = _mk_vm("LKG", size_ratio=0.5)
    _a, s57_6a, _ = c57_6.step(t57_6, {"plan_valid": False}, False, status="PLAN-LOST", visual_match=vm_lkg)
    hold_notices = 1 if c57_6.take_notice() is not None else 0
    for _ in range(2):
        t57_6 += 0.1
        _a, s57_6a, _ = c57_6.step(t57_6, {"plan_valid": False}, False, status="PLAN-LOST",
                                   visual_match=vm_lkg)
        if c57_6.take_notice() is not None:
            hold_notices += 1
    s57_6_ok = (s57_6a == "HOLD_LOST" and hold_notices == 1)

    # (57-7) lkg_hold_is_indefinite: the `closer="LKG"` controller still holds after 60s of further ticks.
    t57_6 += 60.0
    _a, s57_7, _ = c57_6.step(t57_6, {"plan_valid": False}, False, status="PLAN-LOST", visual_match=vm_lkg)
    s57_7_ok = (s57_7 == "HOLD_LOST")

    # (57-8) backoff_restamps_the_wait: after a `closer="LIVE"` back-off, `_loss_episode_t0` equals the
    #        back-off's `now`, and both notice flags are back to False.
    s57_8_ok = (_c57_live._loss_episode_t0 is not None
               and abs(_c57_live._loss_episode_t0 - (1.0 + 0.1)) < 1e-6
               and _c57_live._loss_grace_noticed is False
               and _c57_live._lost_hold_noticed is False)

    # (57-9) restamp_blocks_immediate_refire: continuing to tick that controller with the SAME evidence
    #        produces no second BACKOFF until a further `loss_backoff_grace_s` has elapsed. BACKOFF owns
    #        every status while its own phase-timer runs (session 46, ~backoff_hold_s + backoff_release_s
    #        ~= 1.2s at defaults), so this needs a grace comfortably longer than that to see a real gap
    #        between "backoff just completed" and "grace re-elapsed" -- a dedicated controller, not
    #        `_c57_live` (grace_s=1.0, too close to the backoff's own duration to separate the two).
    def _run_c57_backoff(ctl, t, dt=0.05):
        st = ctl.state
        for _ in range(400):
            _a, st, _e = ctl.step(t, {"plan_valid": False}, False, status="PLAN-LOST",
                                  visual_match=_mk_vm("LIVE"))
            t += dt
            if st != "BACKOFF":
                break
        return t, st

    c57_9 = _mk_c57(clearance=0.5, grace_s=5.0)
    t57_9 = 0.0
    c57_9.step(t57_9, {"plan_valid": False}, False, status="PLAN-LOST")
    t57_9 += c57_9.loss_backoff_grace_s + 0.1
    _a, s57_9_fire, _ = c57_9.step(t57_9, {"plan_valid": False}, False, status="PLAN-LOST",
                                   visual_match=_mk_vm("LIVE"))
    fire_ok = (s57_9_fire == "BACKOFF")
    t57_9, s57_9_done = _run_c57_backoff(c57_9, t57_9 + 0.05)
    backoff_done_ok = (s57_9_done == "HOLD_LOST")
    _a, s57_9a, _ = c57_9.step(t57_9, {"plan_valid": False}, False, status="PLAN-LOST",
                               visual_match=_mk_vm("LIVE"))
    no_immediate_refire_ok = (s57_9a != "BACKOFF")
    t57_9 = c57_9._loss_episode_t0 + c57_9.loss_backoff_grace_s + 0.1
    _a, s57_9b, _ = c57_9.step(t57_9, {"plan_valid": False}, False, status="PLAN-LOST",
                               visual_match=_mk_vm("LIVE"))
    s57_9_ok = (fire_ok and backoff_done_ok and no_immediate_refire_ok and s57_9b == "BACKOFF")

    # (57-10) plan_stale_path_untouched: a PLAN-STALE loss still routes through
    #         `_maybe_loss_snapshot_backoff` and still spends `_loss_snapshot_checked`.
    cfg57_stale = copy.deepcopy(cfg)
    cfg57_stale["autonomy"]["explore"]["use_visual_recovery_on_stale"] = False
    c57_10 = ExploreController(cfg57_stale, no_takeoff=True)
    c57_10._ever_tracked = True
    c57_10.loss_backoff_grace_s = 0.0
    c57_10._last_good_clearance = 0.5
    c57_10._last_good_pos = [1.0, 1.0]
    c57_10._last_good_t = 0.0
    t57_10 = 0.0
    ticket_before_ok = (c57_10._loss_snapshot_checked is False)
    _a, s57_10, _ = c57_10.step(t57_10, {"plan_valid": False}, False, status="PLAN-STALE")
    s57_10_ok = (ticket_before_ok and c57_10._loss_snapshot_checked is True and s57_10 == "BACKOFF")

    # (57-11) episode_edge_resets: a genuine OK followed by a fresh loss clears `_lost_hold_noticed`.
    c57_11 = _mk_c57(clearance=0.5, grace_s=1.0)
    t57_11 = 0.0
    c57_11.step(t57_11, {"plan_valid": False}, False, status="PLAN-LOST")
    t57_11 += c57_11.loss_backoff_grace_s + 0.1
    c57_11.step(t57_11, {"plan_valid": False}, False, status="PLAN-LOST", visual_match=_mk_vm("LKG", 0.5))
    hold_latched_ok = (c57_11._lost_hold_noticed is True)
    t57_11 += 0.1
    c57_11.step(t57_11, {"plan_valid": True, "pos": [1.0, 1.0], "forward_clearance_dist": 3.0}, False,
               status="OK")
    episode_ended_ok = (c57_11._loss_episode_t0 is None)
    t57_11 += 0.1
    c57_11.step(t57_11, {"plan_valid": False}, False, status="PLAN-LOST")   # a FRESH loss -- new episode
    s57_11_ok = (hold_latched_ok and episode_ended_ok and c57_11._lost_hold_noticed is False)

    s57_ok = (s57_1_ok and s57_2_ok and s57_3_ok and s57_4_ok and s57_5_ok and s57_6_ok and s57_7_ok
             and s57_8_ok and s57_9_ok and s57_10_ok and s57_11_ok)
    ok = ok and s57_ok
    print(f"[self-test] {'PASS' if s57_ok else 'FAIL'}  SESSION-57 PLAN-LOST RECOVERY "
          f"(inside_grace_holds={s57_1_ok}, clear_front_past_grace_holds={s57_2_ok}, "
          f"verdict_live_backs_off={s57_3_ok}, verdict_equal_backs_off={s57_4_ok}, "
          f"verdict_unknown_backs_off={s57_5_ok}, verdict_lkg_holds={s57_6_ok}, "
          f"lkg_hold_is_indefinite={s57_7_ok}, backoff_restamps_the_wait={s57_8_ok}, "
          f"restamp_blocks_immediate_refire={s57_9_ok}, plan_stale_path_untouched={s57_10_ok}, "
          f"episode_edge_resets={s57_11_ok})")

    # ---- (session 52, chunk 6) controller-side telemetry: SLAM_HOLD counters, note_timeout, control-bus
    #      fields (`_full_vector`'s state_since_s/slam_hold/notice). No rendering here -- visualizer.py's
    #      panel is chunk 7's job; this proves the DATA it will consume is produced correctly. ----
    # (52-tel-1) SLAM_HOLD counters: a re-entry into SLAM_HOLD FROM SLAM_HOLD must not double-count entries;
    #            dwell time accumulates from the TRUE entry instant (t_state), across separate episodes.
    c52t1 = ExploreController(cfg, no_takeoff=True)
    c52t1._enter("SLAM_HOLD", 0.0)
    c52t1._enter("SLAM_HOLD", 1.0)     # re-entry -- must NOT double-count
    tel1a_ok = (c52t1._slam_hold_entries == 1)
    c52t1._enter("SETTLE", 10.0)
    tel1b_ok = abs(c52t1._slam_hold_total_s - 9.0) < 1e-6   # measured from t=1 (the true SLAM_HOLD entry)
    c52t1._enter("SLAM_HOLD", 12.0)
    c52t1._enter("REPLAN", 15.0)
    tel1c_ok = (c52t1._slam_hold_entries == 2 and abs(c52t1._slam_hold_total_s - 12.0) < 1e-6)
    tel1_ok = tel1a_ok and tel1b_ok and tel1c_ok
    ok = ok and tel1_ok
    print(f"[self-test] {'PASS' if tel1_ok else 'FAIL'}  SESSION-52 chunk 6 SLAM_HOLD counters "
          f"(re-entry no double-count={tel1a_ok}, dwell from t_state={tel1b_ok}, "
          f"second episode accumulates={tel1c_ok})")

    # (52-tel-2) note_timeout ALWAYS overwrites last_timeout (newest wins); loud=True ALSO sets the
    #            one-shot _pending_notice (popped via take_notice), loud=False leaves it alone.
    c52t2 = ExploreController(cfg, no_takeoff=True)
    c52t2.note_timeout("X", "hello", 5.0, loud=False)
    tel2a_ok = (c52t2.last_timeout == {"kind": "X", "text": "hello", "t": 5.0})
    tel2b_ok = (c52t2.take_notice() is None)
    c52t2.note_timeout("Y", "world", 6.0, loud=True)
    tel2c_ok = (c52t2.take_notice() == "world" and c52t2.last_timeout["kind"] == "Y")
    tel2d_ok = (c52t2.take_notice() is None and c52t2.last_timeout is not None)   # latched, never consumed
    tel2_ok = tel2a_ok and tel2b_ok and tel2c_ok and tel2d_ok
    ok = ok and tel2_ok
    print(f"[self-test] {'PASS' if tel2_ok else 'FAIL'}  SESSION-52 chunk 6 note_timeout "
          f"(latches={tel2a_ok}, quiet by default={tel2b_ok}, loud shouts={tel2c_ok}, "
          f"latch survives consumption={tel2d_ok})")

    # (52-tel-3) reset_leg is a manual-takeover interruption -- must NOT zero the flight-so-far picture
    #            (SLAM_HOLD counters, last_timeout are flight-level), but DOES still clear the two
    #            per-episode flags it always has (_visrec_probe_armed, _backoff_gate_noticed).
    c52t3 = ExploreController(cfg, no_takeoff=True)
    c52t3._slam_hold_entries = 3
    c52t3._slam_hold_total_s = 42.0
    c52t3.last_timeout = {"kind": "Z", "text": "z", "t": 1.0}
    c52t3._visrec_probe_armed = True
    c52t3._backoff_gate_noticed = True
    c52t3.reset_leg()
    tel3_ok = (c52t3._slam_hold_entries == 3 and c52t3._slam_hold_total_s == 42.0
              and c52t3.last_timeout == {"kind": "Z", "text": "z", "t": 1.0}
              and c52t3._visrec_probe_armed is False and c52t3._backoff_gate_noticed is False)
    ok = ok and tel3_ok
    print(f"[self-test] {'PASS' if tel3_ok else 'FAIL'}  SESSION-52 chunk 6 reset_leg preserves "
          f"flight-level counters, still clears per-episode flags ({tel3_ok})")

    # (52-tel-4) _full_vector: the three new keys are ALWAYS present (never omitted, even when the caller
    #            passes none of the new kwargs), and round-trip when the caller does pass them.
    v_empty = _full_vector({}, 1, 0.0, "WAIT")
    tel4a_ok = (v_empty["state_since_s"] is None and v_empty["slam_hold"] is None and v_empty["notice"] is None)
    v_full = _full_vector({}, 1, 0.0, "SLAM_HOLD", None, state_since_s=3.0,
                          slam_hold={"now_s": 3.0, "deadline_s": 15.0, "entries": 2, "total_s": 12.0},
                          notice={"kind": "X", "text": "t", "age_s": 1.0})
    tel4b_ok = (v_full["state_since_s"] == 3.0
               and v_full["slam_hold"] == {"now_s": 3.0, "deadline_s": 15.0, "entries": 2, "total_s": 12.0}
               and v_full["notice"] == {"kind": "X", "text": "t", "age_s": 1.0})
    tel4_ok = tel4a_ok and tel4b_ok
    ok = ok and tel4_ok
    print(f"[self-test] {'PASS' if tel4_ok else 'FAIL'}  SESSION-52 chunk 6 _full_vector shape "
          f"(omitted -> None defaults={tel4a_ok}, round-trips={tel4b_ok})")

    # (52-tel-5) the session-35/43 forced SLAM_HOLD hop latches its kind via note_timeout, and the event
    #            string it RETURNS is unchanged from today's wording (loud=False -- the return value
    #            already carries this exact sentence into the log, so note_timeout must not also shout it).
    c52t5 = ExploreController(cfg, no_takeoff=True)
    c52t5.slam_slow_hop_after_s = 0.05
    reached5, tb5, fb5 = _drive_to_advance(c52t5)
    saw_hop5, hop_event5 = False, None
    for _ in range(15):
        _a, s, ev = c52t5.step(tb5, dict(padv35, frame_id=fb5, slam_ms=1500.0, cap_ts=tb5), False, status="OK")
        tb5 += 0.05; fb5 += 1
        if s == "REPLAN":
            saw_hop5 = True
            hop_event5 = ev
            break
    # Session 53: wording gained a second clock ("this hold X.Xs / episode Y.Ys") so the log makes the
    # PLAN-LOST/HOLD_LOST bounce visible instead of inferred -- see _slam_hold_episode_t0.
    event5_text_ok = (hop_event5 is not None
                      and hop_event5.startswith("SLAM_HOLD still waiting (this hold")
                      and "episode" in hop_event5
                      and "forcing one hop toward the current goal" in hop_event5
                      and hop_event5.endswith(f"grace {c52t5.slam_slow_hop_grace_s:.0f}s)"))
    tel5_ok = (reached5 and saw_hop5 and event5_text_ok
              and c52t5.last_timeout is not None
              and c52t5.last_timeout["kind"] == "SLAM_HOLD_FORCED_HOP"
              and c52t5.last_timeout["text"] == hop_event5)
    ok = ok and tel5_ok
    print(f"[self-test] {'PASS' if tel5_ok else 'FAIL'}  SESSION-52 chunk 6 forced hop latches its kind "
          f"(reached ADVANCE={reached5}, hop fired={saw_hop5}, event text unchanged={event5_text_ok}, "
          f"kind=SLAM_HOLD_FORCED_HOP + text matches the returned event={tel5_ok})")

    # ---- Session 55 (H3): periodic diag fsync must actually reach disk, and a failure must LATCH
    #      (surfaced once, then degrade to flush()-only rather than raising/crashing on every
    #      subsequent tick) -- see AutopilotLog.fsync()/DiagLog.fsync(). The periodic TIMER GATING
    #      itself lives inline in run_explore()'s ZMQ loop (not a separately-callable function), so it
    #      is verified live-fly, not here (see PROGRESS.md session 55); this proves the primitive the
    #      timer calls is correct. ----
    import io as _io_mod
    import tempfile as _tempfile_mod
    import contextlib as _ctxlib
    log55 = AutopilotLog(True)   # needs REAL open file handles to prove os.fsync() actually reaches disk
    fsync_no_raise = True
    try:
        log55.fsync()          # healthy handles -- must not raise
    except Exception:
        fsync_no_raise = False
    # Force the REAL failure shape: os.fsync() itself raising OSError on a still-open, valid handle
    # (a disk error, not a Python-level "file already closed" mistake -- that raises ValueError from
    # Python's own io layer before the syscall, a different bug this isn't testing for). Monkeypatch
    # os.fsync for the duration of this check only; restored in `finally` even if an assertion below
    # fails, so no other self-test module state is left disturbed.
    _real_os_fsync = os.fsync
    def _boom_fsync(_fd):
        raise OSError("simulated disk failure (session-55 self-test)")
    _buf = _io_mod.StringIO()
    try:
        os.fsync = _boom_fsync
        with _ctxlib.redirect_stdout(_buf):
            log55.fsync()
        # AutopilotLog.fsync() drives 3 independently-latching sinks (its own _txt/_jsonl pair, then
        # self.csv and self.cmd_csv -- each a SEPARATE DiagLog with its own _fsync_failed flag) -- so
        # the first failing call prints once per sink (3), latching all three.
        latched_after_failure = (log55._fsync_failed and log55.csv._fsync_failed
                                 and log55.cmd_csv._fsync_failed)
        printed_on_first_failure = _buf.getvalue().count("CRITICAL: fsync failed") == 3
        with _ctxlib.redirect_stdout(_buf):
            log55.fsync()       # 2nd call: every sink already latched -> must be fully silent
        silent_after_latch = _buf.getvalue().count("CRITICAL: fsync failed") == 3   # unchanged
    finally:
        os.fsync = _real_os_fsync
    log55.close()
    # AutopilotLog has no output-dir override -- clean up the 4 real files it wrote under OUTPUT/diag/
    # so --self-test stays side-effect-free there (every OTHER self-test in this file uses
    # AutopilotLog(False), precisely to avoid this; this one needs real handles, so it cleans up).
    for _suffix in ("_autopilot.csv", "_autopilot_cmd.csv", "_autopilot.log", "_timeline.jsonl"):
        _p = os.path.join(log55.diag_dir, f"{log55.ts}{_suffix}")
        if os.path.exists(_p):
            os.remove(_p)

    _dl_dir = _tempfile_mod.mkdtemp(prefix="diaglog_fsync_selftest_")
    diaglog55 = DiagLog("selftest55", ["x"], out_dir=_dl_dir)
    diaglog55.row(x=1)
    diaglog55_no_raise = True
    try:
        diaglog55.fsync()
    except Exception:
        diaglog55_no_raise = False
    try:
        os.fsync = _boom_fsync
        with _ctxlib.redirect_stdout(_buf):
            diaglog55.fsync()
    finally:
        os.fsync = _real_os_fsync
    diaglog55_latched = diaglog55._fsync_failed is True
    diaglog55.close()
    import shutil as _shutil_mod
    _shutil_mod.rmtree(_dl_dir, ignore_errors=True)

    fsync55_ok = (fsync_no_raise and latched_after_failure and printed_on_first_failure
                 and silent_after_latch and diaglog55_no_raise and diaglog55_latched)
    ok = ok and fsync55_ok
    print(f"[self-test] {'PASS' if fsync55_ok else 'FAIL'}  SESSION-55 (H3) periodic fsync "
          f"(healthy fsync doesn't raise={fsync_no_raise}, a failure latches all 3 sinks="
          f"{latched_after_failure}, CRITICAL printed once per sink on first failure="
          f"{printed_on_first_failure}, post-latch calls are silent no-ops={silent_after_latch}, "
          f"DiagLog.fsync same contract={diaglog55_no_raise and diaglog55_latched})")

    # ---- SESSION-56 GATE CURRENCY: the settle gate releases on a CURRENT solve, not a FAST one ----
    # (1) slow_but_current_releases: a solve well over slam_slow_ms still clears the gate as long as its
    #     capture instant is at/after the floor -- this is the whole point of the change.
    c56a = ExploreController(cfg, no_takeoff=True)
    c56a.settle_gate_s = 1.0
    c56a._settle_gate_begin(0.0)
    c56a._update_slam({"slam_ms": 2700.0, "frame_id": 1, "cap_ts": 0.5})
    slow_but_current_releases = (c56a._settle_gate_poll(1.5) is True and c56a._slam_gate_since is None)

    # (2) pre_gate_capture_does_not_release: the session-47 trap -- a frame captured BEFORE the gate opened
    #     must not release it, however slow or fast; only a capture at/after the floor clears it.
    c56b = ExploreController(cfg, no_takeoff=True)
    c56b.settle_gate_s = 1.0
    c56b._settle_gate_begin(10.0, new_floor=True)
    c56b._update_slam({"slam_ms": 200.0, "frame_id": 1, "cap_ts": 9.2})
    pre_gate_step1 = (c56b._settle_gate_poll(11.5) is False and c56b._slam_gate_since == 10.0)
    c56b._update_slam({"slam_ms": 200.0, "frame_id": 2, "cap_ts": 10.3})
    pre_gate_step2 = (c56b._settle_gate_poll(11.5) is True)
    pre_gate_capture_does_not_release = pre_gate_step1 and pre_gate_step2

    # (3) dwell_still_applies: currency alone is not enough -- the physical settle_gate_s dwell still gates
    #     release even when a satisfying solve lands the instant the gate opens.
    c56c = ExploreController(cfg, no_takeoff=True)
    c56c.settle_gate_s = 1.0
    c56c._settle_gate_begin(0.0)
    c56c._update_slam({"slam_ms": 200.0, "frame_id": 1, "cap_ts": 0.0})
    dwell_still_applies = (c56c._settle_gate_poll(0.1) is False and c56c._settle_gate_poll(1.5) is True)

    # (4) require_fresh_false_unchanged: the vertical-prelude plain-timer path bypasses currency entirely.
    c56d = ExploreController(cfg, no_takeoff=True)
    c56d.settle_gate_s = 1.0
    c56d._settle_gate_begin(0.0, new_floor=True)
    require_fresh_false_unchanged = (
        c56d._slam_gate_since is not None
        and c56d._settle_gate_poll(1.5, require_fresh=False) is True)

    # (5) floor_survives_bounce: a re-entry into the SAME bad-SLAM episode (episode_t0 not cleared between
    #     calls, exactly what a PLAN-LOST/HOLD_LOST bounce does) must NOT re-stamp the currency floor.
    c56e = ExploreController(cfg, no_takeoff=True)
    c56e._enter_slam_hold("SETTLE", 0.0, "test")
    floor_at_entry_ok = (c56e._slam_gate_since == 0.0)
    c56e._enter_slam_hold("SETTLE", 3.3, "test2")   # bounce: _slam_hold_episode_t0 still set from above
    floor_survives_bounce = (floor_at_entry_ok and c56e._slam_gate_since == 0.0
                             and c56e._settle_gate_t0 == 3.3)

    # (6) fresh_episode_gets_a_new_floor: a genuinely NEW episode (episode_t0 was None) DOES get a new floor.
    c56f = ExploreController(cfg, no_takeoff=True)
    c56f._enter_slam_hold("SETTLE", 20.0, "test")
    fresh_episode_gets_a_new_floor = (c56f._slam_gate_since == 20.0)

    # (7) category_a_always_new_floor: a bare _settle_gate_begin (no keyword) always stamps a fresh floor.
    c56g = ExploreController(cfg, no_takeoff=True)
    c56g._settle_gate_begin(5.0)
    category_a_always_new_floor = (c56g._slam_gate_since == 5.0)

    gate56_ok = (slow_but_current_releases and pre_gate_capture_does_not_release and dwell_still_applies
                and require_fresh_false_unchanged and floor_survives_bounce
                and fresh_episode_gets_a_new_floor and category_a_always_new_floor)
    ok = ok and gate56_ok
    print(f"[self-test] {'PASS' if gate56_ok else 'FAIL'}  SESSION-56 GATE CURRENCY "
          f"(slow_but_current_releases={slow_but_current_releases}, "
          f"pre_gate_capture_does_not_release={pre_gate_capture_does_not_release}, "
          f"dwell_still_applies={dwell_still_applies}, "
          f"require_fresh_false_unchanged={require_fresh_false_unchanged}, "
          f"floor_survives_bounce={floor_survives_bounce}, "
          f"fresh_episode_gets_a_new_floor={fresh_episode_gets_a_new_floor}, "
          f"category_a_always_new_floor={category_a_always_new_floor})")

    # ---- SESSION-56 RELEASE GRACE: a fast (currency-based) release must not become a fast re-divert ----
    # (1) slam_hold_release_stamps_grace: a normal SLAM_HOLD settle-gate release (nxt="ADVANCE") stamps the
    #     grace window immediately, even though the releasing frame is itself SLOW (2700ms) -- the whole
    #     point is that currency, not speed, gates the release now (chunk 2), so the grace must cover it.
    c56r1 = ExploreController(cfg, no_takeoff=True)
    c56r1.settle_gate_s = 0.05
    c56r1._enter_slam_hold("ADVANCE", 0.0, "test")
    now_r1 = 0.5
    _, s_r1, _ = c56r1.step(now_r1, {"plan_valid": True, "goal": [100.0, 0.0], "pos": [0.0, 0.0],
                                     "bearing_err": 0.0, "heading_deg": 0.0, "forward_clearance_dist": 5.0,
                                     "frame_id": 1, "cap_ts": now_r1, "slam_ms": 2700.0}, False)
    slam_hold_release_stamps_grace = (
        s_r1 == "ADVANCE" and c56r1._slam_slow_hop_deadline is not None
        and c56r1._slam_slow_hop_deadline >= now_r1 + c56r1.slam_slow_hop_grace_s - 1e-6)

    # (2) no_instant_redivert: from that release, 20 more ticks with SLAM still slow (2700ms) must NOT bounce
    #     back into SLAM_HOLD within the grace window, and ADVANCE must actually command motion (non-empty
    #     active dict on at least one tick) -- proving the grace buys real flight, not just a state label.
    c56r1.leg_goal = [100.0, 0.0]
    c56r1.ram_stall_s = 0.0        # disable: this test holds pos constant, which would otherwise misread as a ram-stall
    c56r1.hop_duration_s = 0.0     # disable: don't let the hop timer end the leg mid-test
    saw_slam_hold_2 = False
    saw_nonempty_active_2 = False
    t_r2 = now_r1
    for _i2 in range(20):
        t_r2 += 0.05
        active_r2, s_r2, _ = c56r1.step(t_r2, {"plan_valid": True, "goal": [100.0, 0.0], "pos": [0.0, 0.0],
                                               "bearing_err": 0.0, "heading_deg": 0.0,
                                               "forward_clearance_dist": 5.0, "frame_id": 100 + _i2,
                                               "cap_ts": t_r2, "slam_ms": 2700.0}, False)
        if s_r2 == "SLAM_HOLD":
            saw_slam_hold_2 = True
        if active_r2:
            saw_nonempty_active_2 = True
    no_instant_redivert = (not saw_slam_hold_2) and saw_nonempty_active_2

    # (3) settle_release_stamps_grace: same assertion as (1), for SETTLE's normal gated release.
    c56r3 = ExploreController(cfg, no_takeoff=True)
    c56r3.settle_gate_s = 0.05
    c56r3._settle_to = "REPLAN"
    c56r3._enter("SETTLE", 0.0)
    now_r3 = 0.5
    _, s_r3, _ = c56r3.step(now_r3, {"plan_valid": True, "goal": [1.0, 0.0], "pos": [0.0, 0.0],
                                     "bearing_err": 0.0, "heading_deg": 0.0, "forward_clearance_dist": 5.0,
                                     "frame_id": 1, "cap_ts": now_r3, "slam_ms": 2700.0}, False)
    settle_release_stamps_grace = (
        s_r3 == "REPLAN" and c56r3._slam_slow_hop_deadline is not None
        and c56r3._slam_slow_hop_deadline >= now_r3 + c56r3.slam_slow_hop_grace_s - 1e-6)

    # (4) trim_resume_release_stamps_grace: same assertion as (1), for TRIM_RESUME_WAIT's normal release
    #     (via _trim_resolve_resume, which resolves internally to ORIENT here).
    c56r4 = ExploreController(cfg, no_takeoff=True)
    c56r4.settle_gate_s = 0.05
    c56r4._trim_resume_goal = [3.0, 0.0]
    c56r4._trim_exit(0.0, {}, "TRIM done")
    now_r4 = 0.5
    _, s_r4, _ = c56r4.step(now_r4, {"plan_valid": True, "goal": [3.0, 0.0], "pos": [0.0, 0.0],
                                     "bearing_err": 0.0, "heading_deg": 0.0, "forward_clearance_dist": 5.0,
                                     "frame_id": 1, "cap_ts": now_r4, "slam_ms": 2700.0,
                                     "blacklist": [], "blacklist_permanent": []}, False)
    trim_resume_release_stamps_grace = (
        s_r4 == "ORIENT" and c56r4._slam_slow_hop_deadline is not None
        and c56r4._slam_slow_hop_deadline >= now_r4 + c56r4.slam_slow_hop_grace_s - 1e-6)

    # (5) grace_expires: the grace is a WINDOW, not a disable -- once it elapses with SLAM still slow, the
    #     next ADVANCE tick must divert to SLAM_HOLD again.
    c56r5 = ExploreController(cfg, no_takeoff=True)
    c56r5.settle_gate_s = 0.05
    c56r5.slam_slow_hop_grace_s = 0.3     # shrink for the test; the mechanism, not the value, is under test
    c56r5._enter_slam_hold("ADVANCE", 0.0, "test")
    now_r5 = 0.5
    _, s_r5, _ = c56r5.step(now_r5, {"plan_valid": True, "goal": [100.0, 0.0], "pos": [0.0, 0.0],
                                     "bearing_err": 0.0, "heading_deg": 0.0, "forward_clearance_dist": 5.0,
                                     "frame_id": 1, "cap_ts": now_r5, "slam_ms": 2700.0}, False)
    released_ok_5 = (s_r5 == "ADVANCE")
    c56r5.leg_goal = [100.0, 0.0]
    c56r5.ram_stall_s = 0.0
    c56r5.hop_duration_s = 0.0
    t_r5, state_r5 = now_r5, s_r5
    for _i5 in range(30):
        t_r5 += 0.05
        _, state_r5, _ = c56r5.step(t_r5, {"plan_valid": True, "goal": [100.0, 0.0], "pos": [0.0, 0.0],
                                           "bearing_err": 0.0, "heading_deg": 0.0,
                                           "forward_clearance_dist": 5.0, "frame_id": 100 + _i5,
                                           "cap_ts": t_r5, "slam_ms": 2700.0}, False)
        if state_r5 == "SLAM_HOLD":
            break
    grace_expires = (released_ok_5 and state_r5 == "SLAM_HOLD"
                     and t_r5 > now_r5 + c56r5.slam_slow_hop_grace_s - 1e-9)

    release56_ok = (slam_hold_release_stamps_grace and no_instant_redivert and settle_release_stamps_grace
                    and trim_resume_release_stamps_grace and grace_expires)
    ok = ok and release56_ok
    print(f"[self-test] {'PASS' if release56_ok else 'FAIL'}  SESSION-56 RELEASE GRACE "
          f"(slam_hold_release_stamps_grace={slam_hold_release_stamps_grace}, "
          f"no_instant_redivert={no_instant_redivert}, "
          f"settle_release_stamps_grace={settle_release_stamps_grace}, "
          f"trim_resume_release_stamps_grace={trim_resume_release_stamps_grace}, "
          f"grace_expires={grace_expires})")

    # ---- SESSION-56 F_LKG FREEZE: the reference may only be (re)stored against a CURRENT plan (status ==
    # "OK"), never a merely once-valid one. PLAN-LOST is a pure age verdict on a plan of any age -- the stale
    # plan still reads plan_valid=True, so the old plan_valid-only gate fired on every tick of a loss.
    ok_and_valid_caches = _visrec_should_cache_reference("OK", {"plan_valid": True}) is True
    plan_lost_with_stale_valid_does_not_cache = (
        _visrec_should_cache_reference("PLAN-LOST", {"plan_valid": True}) is False)
    plan_stale_does_not_cache = _visrec_should_cache_reference("PLAN-STALE", {"plan_valid": False}) is False
    no_plan_does_not_cache = _visrec_should_cache_reference("NO-PLAN", {}) is False
    ok_but_invalid_does_not_cache = _visrec_should_cache_reference("OK", {"plan_valid": False}) is False
    s56_lkg_freeze_ok = (ok_and_valid_caches and plan_lost_with_stale_valid_does_not_cache
                         and plan_stale_does_not_cache and no_plan_does_not_cache
                         and ok_but_invalid_does_not_cache)
    ok = ok and s56_lkg_freeze_ok
    print(f"[self-test] {'PASS' if s56_lkg_freeze_ok else 'FAIL'}  SESSION-56 F_LKG FREEZE "
          f"(ok_and_valid_caches={ok_and_valid_caches}, "
          f"plan_lost_with_stale_valid_does_not_cache={plan_lost_with_stale_valid_does_not_cache}, "
          f"plan_stale_does_not_cache={plan_stale_does_not_cache}, "
          f"no_plan_does_not_cache={no_plan_does_not_cache}, "
          f"ok_but_invalid_does_not_cache={ok_but_invalid_does_not_cache})")

    # ---- SESSION-56 F_LKG AGE-OUT: an aged-out plan frame_id keeps the PREVIOUS F_LKG (loud, counted,
    # rate-limited) instead of silently substituting the live frame -- see run_explore's resolution block
    # + VisualRecoveryProbe.update_reference's tracked=False contract (visual_recovery.py --self-test
    # covers the probe side directly; these cover the controller/telemetry/panel side). ----

    # (56-ageout-1) the removed live-frame-substitution src label (see visual_recovery.py's `_lkg_src`
    # docstring history) must not survive anywhere in this repo's own top-level modules. The needle is
    # built via concatenation below so this very check can never false-positive on its own search string.
    _removed_lkg_src = "live" + "(aged-out)"
    _hits = []
    for _fn in sorted(os.listdir(REPO)):
        if _fn.endswith(".py"):
            with open(os.path.join(REPO, _fn), "r", encoding="utf-8") as _f:
                for _lineno, _line in enumerate(_f, 1):
                    if _removed_lkg_src in _line:
                        _hits.append(f"{_fn}:{_lineno}")
    lkg_src_never_says_live_aged_out = (len(_hits) == 0)
    ok = ok and lkg_src_never_says_live_aged_out
    print(f"[self-test] {'PASS' if lkg_src_never_says_live_aged_out else 'FAIL'}  SESSION-56 F_LKG "
          f"AGE-OUT lkg_src_never_says_live_aged_out (hits={_hits})")

    # (56-ageout-2) the counter/flag mirror visrec_window_failed/visrec_save_failed exactly: they start
    # at (0, False), and the sticky flag stays True across a later, otherwise-unrelated mutation (i.e. it
    # is never silently cleared by anything downstream).
    c56a = ExploreController(cfg, no_takeoff=True)
    a_before = (c56a.visrec_lkg_ageouts, c56a.visrec_lkg_degraded)
    c56a.visrec_lkg_ageouts += 1
    c56a.visrec_lkg_degraded = True
    a_after_first = (c56a.visrec_lkg_ageouts, c56a.visrec_lkg_degraded)
    c56a.visrec_lkg_ageouts += 1                 # a second, independent age-out
    a_sticky = c56a.visrec_lkg_degraded          # must still read True
    ageout_counter_and_flag = (a_before == (0, False) and a_after_first == (1, True)
                               and c56a.visrec_lkg_ageouts == 2 and a_sticky is True)
    ok = ok and ageout_counter_and_flag
    print(f"[self-test] {'PASS' if ageout_counter_and_flag else 'FAIL'}  SESSION-56 F_LKG AGE-OUT "
          f"ageout_counter_and_flag (before={a_before}, after_first={a_after_first}, "
          f"count_after_second={c56a.visrec_lkg_ageouts}, sticky={a_sticky})")

    # (56-ageout-3) _full_vector: visrec_lkg is ALWAYS present (never omitted; None when the caller
    # passes none of the new kwargs), and round-trips the exact three sub-keys when the caller does.
    v56_empty = _full_vector({}, 1, 0.0, "WAIT")
    tel56a_ok = ("visrec_lkg" in v56_empty and v56_empty["visrec_lkg"] is None)
    v56_full = _full_vector({}, 1, 0.0, "SLAM_HOLD",
                            visrec_lkg={"src": "slam:67719", "ageouts": 3, "degraded": True})
    tel56b_ok = (v56_full["visrec_lkg"] == {"src": "slam:67719", "ageouts": 3, "degraded": True})
    telemetry_payload_shape = tel56a_ok and tel56b_ok
    ok = ok and telemetry_payload_shape
    print(f"[self-test] {'PASS' if telemetry_payload_shape else 'FAIL'}  SESSION-56 F_LKG AGE-OUT "
          f"telemetry_payload_shape (present/None when omitted={tel56a_ok}, round-trips={tel56b_ok})")

    # (56-ageout-4) visualizer.render_telemetry_panel composes without raising both with NO visrec_lkg
    # payload and with a DEGRADED one -- shape-only smoke test (mirrors session 41's approach), not a
    # pixel-content check.
    import visualizer
    panel_no_payload = visualizer.render_telemetry_panel({"state": "SLAM_HOLD"}, {})
    panel_degraded = visualizer.render_telemetry_panel(
        {"state": "SLAM_HOLD", "visrec_lkg": {"src": "slam:1", "ageouts": 5, "degraded": True}}, {})
    expected_shape = (visualizer.PANEL_H, visualizer.PANEL_W, 3)
    panel_renders_without_payload = (panel_no_payload.shape == expected_shape
                                     and panel_degraded.shape == expected_shape)
    ok = ok and panel_renders_without_payload
    print(f"[self-test] {'PASS' if panel_renders_without_payload else 'FAIL'}  SESSION-56 F_LKG AGE-OUT "
          f"panel_renders_without_payload (no_payload_shape={panel_no_payload.shape}, "
          f"degraded_shape={panel_degraded.shape}, expected={expected_shape})")

    # ---- SESSION-56 TRIM FROM SLAM_HOLD: _TRIM_TRIGGER_STATES gained "SLAM_HOLD" (the whitelist, not the
    # trim threshold, was starving TRIM during flight 20260902_165340's 17:16:36->17:20:51 HOLD_LOST/
    # SLAM_HOLD stretch). The trigger check itself was RELOCATED (see the "Session 56: MOVED here" comment
    # at its new site, just above the "if st == 'SLAM_HOLD':" hold-handler) since its old position was
    # unreachable whenever parked in SLAM_HOLD -- that handler exhaustively returns on every branch. ----

    def _run_full_trim(c, t0, cap0, fid0, pos_y=-1.70, max_ticks=60, **plan_kw):
        """Step `c` forward from a freshly-TRIM-triggered tick through PULSE -> WAIT -> TRIM_RESUME_WAIT
        until it lands outside both, feeding a fresh healthy frame each tick. Returns (active, state, event)
        of the LAST tick."""
        tt, cap, fid = t0, cap0, fid0
        active, s, ev = {}, c.state, None
        for _ in range(max_ticks):
            active, s, ev = c.step(tt, _tplan(pos_y, cap=cap, fid=fid, **plan_kw), False)
            if s not in ("TRIM", "TRIM_RESUME_WAIT"):
                break
            tt += 0.02; cap += 0.02; fid += 1
        return active, s, ev

    # (1) trims_from_slam_hold: parked in SLAM_HOLD, calibrated, plan_valid, pos_y past trim_sag_trigger_y
    # (-1.75) -> the very next step enters TRIM (UP) and publishes the up pulse.
    c56t1 = _mk_trim()
    c56t1._enter_slam_hold("SETTLE", 0.0, "test setup")
    active_t1, s_t1, _ = c56t1.step(0.0, _tplan(-1.70, cap=0.0, fid=1), False)
    trims_from_slam_hold = (s_t1 == "TRIM" and c56t1._trim_dir == "UP"
                            and active_t1.get("joy_vertical") == -1)
    ok = ok and trims_from_slam_hold
    print(f"[self-test] {'PASS' if trims_from_slam_hold else 'FAIL'}  SESSION-56 TRIM FROM SLAM_HOLD "
          f"trims_from_slam_hold (state={s_t1}, trim_dir={c56t1._trim_dir}, active={active_t1})")

    # (2) episode_clock_and_trust_survive: a trim triggered mid-episode must NOT reset the bad-SLAM episode
    # clock or silently restore recovery trust -- only SLAM_HOLD's OWN settle-gate/trust check may do that.
    c56t2 = _mk_trim()
    c56t2._enter_slam_hold("SETTLE", 0.0, "test setup")
    c56t2._slam_hold_episode_t0 = 100.0
    c56t2._recovering = True
    _, s_t2, _ = c56t2.step(0.0, _tplan(-1.70, cap=0.0, fid=1), False)
    episode_clock_and_trust_survive = (s_t2 == "TRIM" and c56t2._slam_hold_episode_t0 == 100.0
                                       and c56t2._recovering is True)
    ok = ok and episode_clock_and_trust_survive
    print(f"[self-test] {'PASS' if episode_clock_and_trust_survive else 'FAIL'}  SESSION-56 TRIM FROM "
          f"SLAM_HOLD episode_clock_and_trust_survive (state={s_t2}, "
          f"episode_t0={c56t2._slam_hold_episode_t0}, recovering={c56t2._recovering})")

    # (3) resume_target_is_honoured: Step 6.2's DECISION -- a TRIM interrupting an as-yet-unresolved
    # SLAM_HOLD episode (`_slam_resume` still set) re-enters SLAM_HOLD honouring `_slam_resume` on exit,
    # rather than blindly re-aiming ORIENT at the Trap-B preserved goal (that would be exactly the silent
    # trust restoration sessions 35/43 gate behind SLAM_HOLD's own settle-gate check). See
    # `_trim_resolve_resume`'s docstring/comment for the full reasoning.
    c56t3 = _mk_trim()
    c56t3._enter_slam_hold("SETTLE", 0.0, "test setup")
    _, s_t3a, _ = c56t3.step(0.0, _tplan(-1.70, cap=0.0, fid=1), False)
    active_t3, s_t3, _ = _run_full_trim(c56t3, 0.02, 0.02, 2)
    resume_target_is_honoured = (s_t3a == "TRIM" and s_t3 == "SLAM_HOLD" and c56t3._slam_resume == "SETTLE")
    ok = ok and resume_target_is_honoured
    print(f"[self-test] {'PASS' if resume_target_is_honoured else 'FAIL'}  SESSION-56 TRIM FROM SLAM_HOLD "
          f"resume_target_is_honoured (triggered_state={s_t3a}, final_state={s_t3}, "
          f"slam_resume={c56t3._slam_resume})")

    # (4) no_reentry_within_one_solve: TRIM's WAIT phase structurally cannot exit without a post-pulse
    # cap_ts (>= _trim_cmd_t0 + trim_settle_s) -- no cooldown knob needed (Step 6.3). Feed the SAME stale
    # cap_ts/frame_id for many ticks (well under the slam_slow_hop_after_s backstop): TRIM must stay parked
    # in its own WAIT sub-phase, never re-entering (there is nothing to re-enter FROM -- it never left).
    # Asserts the IMMEDIATE post-trigger state too (mirrors test 1): without that check, a broken whitelist
    # (SLAM_HOLD missing) still reaches "TRIM" by the end of the loop below via a DIFFERENT route -- SLAM_HOLD's
    # own settle-gate resolves to SETTLE within a couple of the loop's ticks (SETTLE is untouched, still
    # whitelisted), and TRIM then fires from THAT -- silently masking the defect this test targets.
    c56t4 = _mk_trim()
    c56t4._enter_slam_hold("SETTLE", 0.0, "test setup")
    _, s_t4_trig, _ = c56t4.step(0.0, _tplan(-1.70, cap=0.0, fid=1), False)   # trigger TRIM (PULSE)
    tt4 = 0.0
    s_t4 = c56t4.state
    for _ in range(20):
        tt4 += 0.02
        _, s_t4, _ = c56t4.step(tt4, _tplan(-1.70, cap=0.0, fid=1), False)   # cap_ts/frame_id never advance
    no_reentry_within_one_solve = (s_t4_trig == "TRIM" and s_t4 == "TRIM" and c56t4._trim_phase == "WAIT")
    ok = ok and no_reentry_within_one_solve
    print(f"[self-test] {'PASS' if no_reentry_within_one_solve else 'FAIL'}  SESSION-56 TRIM FROM SLAM_HOLD "
          f"no_reentry_within_one_solve (triggered_state={s_t4_trig}, final_state={s_t4}, "
          f"trim_phase={c56t4._trim_phase})")

    # (5) hold_lost_still_excluded: HOLD_LOST is deliberately NOT in _TRIM_TRIGGER_STATES (no fresh pos_y,
    # and TRIM's WAIT phase would then hang exactly like session 54's 73.9s bug) -- TRIM must not fire.
    # `_explore_started=False` bypasses the UNRELATED status-gated recovery block (autopilot.py ~3017,
    # `if self._explore_started:`) that otherwise intercepts + returns on EVERY status while st=="HOLD_LOST"
    # (e.g. status=="OK" sweeps any _RECOVERY_STATES member, HOLD_LOST included, straight into a fresh
    # SLAM_HOLD via _enter_slam_hold -- confirmed by hand, that convergence is correct and pre-existing, but
    # it would reach state=="SLAM_HOLD" before ever reaching the trim trigger either way, silently passing
    # this test even if "HOLD_LOST" were wrongly added to the whitelist). Bypassing it isolates the ACTUAL
    # unit under test -- the whitelist membership check itself -- exactly like test 1's SLAM_HOLD setup.
    c56t5 = _mk_trim()
    c56t5._explore_started = False
    c56t5._enter("HOLD_LOST", 0.0)
    _, s_t5, _ = c56t5.step(0.0, _tplan(-1.70, cap=0.0, fid=1), False)
    hold_lost_still_excluded = (s_t5 == "HOLD_LOST")
    ok = ok and hold_lost_still_excluded
    print(f"[self-test] {'PASS' if hold_lost_still_excluded else 'FAIL'}  SESSION-56 TRIM FROM SLAM_HOLD "
          f"hold_lost_still_excluded (state={s_t5})")

    # ---- SESSION-57 CONFIG WIRING: the three visrec_size_* thresholds load from config.yaml and honour
    # overrides (C5/C11) -- no decision logic yet, this just confirms the wiring itself. ----
    # (1) defaults_present: a controller built from the repo's OWN config.yaml gets the C11 defaults.
    c57cfg_a = ExploreController(cfg, no_takeoff=True)
    defaults_present = (c57cfg_a.visrec_size_ratio_hi == 1.25 and c57cfg_a.visrec_size_ratio_lo == 0.80
                        and c57cfg_a.visrec_size_min_inliers == 20)

    # (2) overrides_honoured: a cfg dict with different explore.* values produces those values on the
    #     controller, proving the read is live (`e.get(...)`), not hardcoded.
    cfg57 = copy.deepcopy(cfg)
    cfg57["autonomy"]["explore"]["visrec_size_ratio_hi"] = 2.0
    cfg57["autonomy"]["explore"]["visrec_size_ratio_lo"] = 0.5
    cfg57["autonomy"]["explore"]["visrec_size_min_inliers"] = 7
    c57cfg_b = ExploreController(cfg57, no_takeoff=True)
    overrides_honoured = (c57cfg_b.visrec_size_ratio_hi == 2.0 and c57cfg_b.visrec_size_ratio_lo == 0.5
                          and c57cfg_b.visrec_size_min_inliers == 7)

    # (3) lo_below_hi: sanity invariant on the repo's shipped defaults -- a mis-set config would make
    #     EVERY match verdict either LIVE or LKG with no EQUAL band, silently breaking C4.
    lo_below_hi = (c57cfg_a.visrec_size_ratio_lo < c57cfg_a.visrec_size_ratio_hi)

    config57_ok = defaults_present and overrides_honoured and lo_below_hi
    ok = ok and config57_ok
    print(f"[self-test] {'PASS' if config57_ok else 'FAIL'}  SESSION-57 CONFIG WIRING "
          f"(defaults_present={defaults_present}, overrides_honoured={overrides_honoured}, "
          f"lo_below_hi={lo_below_hi})")

    # ---- SESSION-57 PLAN-LOST ALWAYS LOOKS: chunk 3 opens `wants_visual_match` for a matured
    # PLAN-LOST/NO-PLAN episode regardless of the one-shot ticket's state (MISSION CONTEXT finding 1 --
    # 56 of 86 loss episodes in flight 20260903_083329 ran zero matches because the ticket was spent on
    # the episode's very first tick, before any camera evidence existed). ----
    c57w = _mk_gate_ctrl(one_shot_spent=True)          # ticket SPENT -- the old code path never looks again
    grace57 = c57w.loss_backoff_grace_s
    t0_57 = 200.0
    c57w._loss_episode_t0 = t0_57

    # (1) spent_ticket_clear_front_still_looks -- past the grace, a matured PLAN-LOST episode looks anyway.
    spent_ticket_clear_front_still_looks = c57w.wants_visual_match(
        now=t0_57 + grace57 + 0.1, status="PLAN-LOST") is True

    # (2) inside_grace_does_not_look -- the unconditional 12s wait is still honoured; no early peek.
    inside_grace_does_not_look = c57w.wants_visual_match(
        now=t0_57 + grace57 - 0.1, status="PLAN-LOST") is False

    # (3) no_args_is_unchanged -- every pre-existing caller (no now/status) keeps today's answer.
    no_args_is_unchanged = c57w.wants_visual_match() is False

    # (4) plan_stale_unaffected -- PLAN-STALE still runs through `_maybe_loss_snapshot_backoff` only; the
    #     new clause names PLAN-LOST/NO-PLAN exclusively.
    plan_stale_unaffected = c57w.wants_visual_match(
        now=t0_57 + grace57 + 0.1, status="PLAN-STALE") is False

    # (5) no_episode_stamp -- with no loss-episode stamp there is nothing to time, regardless of `now`.
    c57w_noep = _mk_gate_ctrl(one_shot_spent=True)
    c57w_noep._loss_episode_t0 = None
    no_episode_stamp = c57w_noep.wants_visual_match(now=t0_57 + 9999.0, status="PLAN-LOST") is False

    # (6) should_match_threads_args -- `_visrec_should_match` forwards now/status into the new clause,
    #     and the status name itself still gates it (an unrelated "OK" status must not force a match).
    c57sm = _mk_gate_ctrl(one_shot_spent=True)
    c57sm._loss_episode_t0 = t0_57
    should_match_lost = _gate(c57sm, memo=None, now=t0_57 + grace57 + 0.1, status="PLAN-LOST") is True
    should_match_ok = _gate(c57sm, memo=None, now=t0_57 + grace57 + 0.1, status="OK") is False
    should_match_threads_args = should_match_lost and should_match_ok

    s57_looks_ok = (spent_ticket_clear_front_still_looks and inside_grace_does_not_look
                    and no_args_is_unchanged and plan_stale_unaffected and no_episode_stamp
                    and should_match_threads_args)
    ok = ok and s57_looks_ok
    print(f"[self-test] {'PASS' if spent_ticket_clear_front_still_looks else 'FAIL'}  SESSION-57 PLAN-LOST "
          f"ALWAYS LOOKS spent_ticket_clear_front_still_looks")
    print(f"[self-test] {'PASS' if inside_grace_does_not_look else 'FAIL'}  SESSION-57 PLAN-LOST "
          f"ALWAYS LOOKS inside_grace_does_not_look")
    print(f"[self-test] {'PASS' if no_args_is_unchanged else 'FAIL'}  SESSION-57 PLAN-LOST "
          f"ALWAYS LOOKS no_args_is_unchanged")
    print(f"[self-test] {'PASS' if plan_stale_unaffected else 'FAIL'}  SESSION-57 PLAN-LOST "
          f"ALWAYS LOOKS plan_stale_unaffected")
    print(f"[self-test] {'PASS' if no_episode_stamp else 'FAIL'}  SESSION-57 PLAN-LOST "
          f"ALWAYS LOOKS no_episode_stamp")
    print(f"[self-test] {'PASS' if should_match_threads_args else 'FAIL'}  SESSION-57 PLAN-LOST "
          f"ALWAYS LOOKS should_match_threads_args")
    print(f"[self-test] {'PASS' if s57_looks_ok else 'FAIL'}  SESSION-57 PLAN-LOST ALWAYS LOOKS overall")

    # ---- SESSION-58 GRACE BEFORE LOOKING: chunk 1 fixes the defect the SESSION-57 block above never
    # covered -- every SESSION-57 case above builds with one_shot_spent=True, which is exactly why the
    # ticket-first check masked the grace clause in real flight (the ticket is re-armed False at every
    # loss edge, so it is ALWAYS armed when a real loss episode begins). These cases build with the
    # ticket ARMED (one_shot_spent=False), the real flight condition, proving the grace is now honoured
    # even though the ticket alone would have said "yes, look" (MISSION CONTEXT finding 1: 233 of 253
    # matches ran inside an unreadable grace window in flight 20260903_223345). ----
    c58g = _mk_gate_ctrl(one_shot_spent=False)          # ticket ARMED -- the real-flight condition
    grace58 = c58g.loss_backoff_grace_s
    t0_58 = 500.0
    c58g._loss_episode_t0 = t0_58

    # (1) armed_inside_grace_does_not_look -- today this returns True; that is the defect being fixed.
    armed_inside_grace_does_not_look = c58g.wants_visual_match(
        now=t0_58 + grace58 - 0.1, status="PLAN-LOST") is False

    # (2) armed_matured_looks -- past the grace, the same armed controller looks.
    armed_matured_looks = c58g.wants_visual_match(
        now=t0_58 + grace58 + 0.1, status="PLAN-LOST") is True

    # (3) armed_probe_still_looks -- the MATCH-phase probe is never gated by the loss grace.
    c58probe = _mk_gate_ctrl(one_shot_spent=False, phase="MATCH")
    c58probe._loss_episode_t0 = t0_58
    armed_probe_still_looks = c58probe.wants_visual_match(
        now=t0_58 + grace58 - 0.1, status="PLAN-LOST") is True

    # (4) armed_stale_unaffected -- PLAN-STALE still falls through to the ticket, which is armed -> True.
    armed_stale_unaffected = c58g.wants_visual_match(
        now=t0_58 + grace58 + 0.1, status="PLAN-STALE") is True

    # (5) armed_no_plan_matches_plan_lost -- NO-PLAN takes the identical status branch as PLAN-LOST.
    armed_no_plan_inside_grace = c58g.wants_visual_match(
        now=t0_58 + grace58 - 0.1, status="NO-PLAN") is False
    armed_no_plan_matured = c58g.wants_visual_match(
        now=t0_58 + grace58 + 0.1, status="NO-PLAN") is True
    armed_no_plan_matches_plan_lost = armed_no_plan_inside_grace and armed_no_plan_matured

    # (6) armed_gate_blocks_compute -- the predicate reaches the actual compute decision, not just itself.
    gate_blocks_inside_grace = _gate(
        c58g, memo=None, now=t0_58 + grace58 - 0.1, status="PLAN-LOST") is False
    gate_allows_after_grace = _gate(
        c58g, memo=None, now=t0_58 + grace58 + 0.1, status="PLAN-LOST") is True
    armed_gate_blocks_compute = gate_blocks_inside_grace and gate_allows_after_grace

    s58_grace_ok = (armed_inside_grace_does_not_look and armed_matured_looks
                    and armed_probe_still_looks and armed_stale_unaffected
                    and armed_no_plan_matches_plan_lost and armed_gate_blocks_compute)
    ok = ok and s58_grace_ok
    print(f"[self-test] {'PASS' if armed_inside_grace_does_not_look else 'FAIL'}  SESSION-58 GRACE BEFORE "
          f"LOOKING armed_inside_grace_does_not_look")
    print(f"[self-test] {'PASS' if armed_matured_looks else 'FAIL'}  SESSION-58 GRACE BEFORE LOOKING "
          f"armed_matured_looks")
    print(f"[self-test] {'PASS' if armed_probe_still_looks else 'FAIL'}  SESSION-58 GRACE BEFORE LOOKING "
          f"armed_probe_still_looks")
    print(f"[self-test] {'PASS' if armed_stale_unaffected else 'FAIL'}  SESSION-58 GRACE BEFORE LOOKING "
          f"armed_stale_unaffected")
    print(f"[self-test] {'PASS' if armed_no_plan_matches_plan_lost else 'FAIL'}  SESSION-58 GRACE BEFORE "
          f"LOOKING armed_no_plan_matches_plan_lost")
    print(f"[self-test] {'PASS' if armed_gate_blocks_compute else 'FAIL'}  SESSION-58 GRACE BEFORE LOOKING "
          f"armed_gate_blocks_compute")
    print(f"[self-test] {'PASS' if s58_grace_ok else 'FAIL'}  SESSION-58 GRACE BEFORE LOOKING overall")

    # ---- SESSION-57 STALE DIRECTION GATE (chunk 6): Finding 2 on the PLAN-STALE path -- `planar_like`
    # is a pure inlier-ratio test ("flat surface"), not a distance verdict, and is completely
    # direction-blind. Real flight evidence (08:51:44.313): matched=True inliers=45 contained=False
    # planar_like=True scale=0.32 -- scale<1 means the live view is a SHRUNK view of F_LKG (farther
    # away), yet the pre-chunk-6 code backed off anyway. Both the action site (:2879-2891) and the
    # `_would_react` predicate that arms the grace/gate ahead of it (:2809-2814) now require
    # `closer == "LIVE"` in lockstep. ----
    cfg_dir57 = cfg_vr   # use_visual_recovery_on_stale=True; same fixture as the block above

    # (1) planar_far_no_backoff: PLAN-STALE, matched+planar_like, but closer="LKG" (live frame is the
    #     FARTHER one) with a clear cached clearance -> both loss-instant checks inconclusive, hands
    #     off into the 15deg probe instead of backing off.
    c57dir1 = ExploreController(cfg_dir57, no_takeoff=True); c57dir1._ever_tracked = True
    c57dir1.loss_backoff_grace_s = 0.0   # session 48 timing is tested in its own block
    c57dir1.leg_goal = [5.0, 5.0]
    c57dir1.step(0.0, p_clear, False, status="OK")
    vm_planar_far = VisualMatch(has_lkg=True, matched=True, inliers=50, contained=False, planar_like=True,
                                scale=1.0, closer="LKG")
    _a, s_dir1, _ = c57dir1.step(0.02, {"plan_valid": False}, False, status="PLAN-STALE",
                                 visual_match=vm_planar_far)
    planar_far_no_backoff = (s_dir1 == "VISUAL_RECOVERY")

    # (2) planar_near_backs_off: identical evidence, but closer="LIVE" (live frame is genuinely the
    #     closer one) -> BACKOFF, same as the pre-chunk-6 behavior for this evidence shape.
    c57dir2 = ExploreController(cfg_dir57, no_takeoff=True)
    c57dir2.loss_backoff_grace_s = 0.0
    c57dir2.leg_goal = [5.0, 5.0]
    c57dir2.step(0.0, p_clear, False, status="OK")
    vm_planar_near = VisualMatch(has_lkg=True, matched=True, inliers=50, contained=False, planar_like=True,
                                 scale=1.0, closer="LIVE")
    _a, s_dir2, ev_dir2 = c57dir2.step(0.02, {"plan_valid": False}, False, status="PLAN-STALE",
                                       visual_match=vm_planar_near)
    planar_near_backs_off = (s_dir2 == "BACKOFF" and ev_dir2 is not None and "visual" in ev_dir2.lower())

    # (3) contained_far_no_backoff: same shape as (1) but via the CONTAINED (zoomed-crop) clause instead
    #     of planar_like -- both disjuncts of the OR must honour the same closer=="LIVE" requirement.
    c57dir3 = ExploreController(cfg_dir57, no_takeoff=True); c57dir3._ever_tracked = True
    c57dir3.loss_backoff_grace_s = 0.0
    c57dir3.leg_goal = [5.0, 5.0]
    c57dir3.step(0.0, p_clear, False, status="OK")
    vm_contained_far = VisualMatch(has_lkg=True, matched=True, inliers=40, contained=True, planar_like=False,
                                   scale=1.8, closer="LKG")
    _a, s_dir3, _ = c57dir3.step(0.02, {"plan_valid": False}, False, status="PLAN-STALE",
                                 visual_match=vm_contained_far)
    contained_far_no_backoff = (s_dir3 == "VISUAL_RECOVERY")

    # (4) would_react_agrees: the grace/gate predicate (:2809-2814) must arm for exactly the same
    #     evidence the action then honours. Isolated via the grace-notice side effect of
    #     `_maybe_loss_snapshot_backoff` itself: a clear cached clearance removes the geometric
    #     disjunct, so with `waited < loss_backoff_grace_s` the LOSS_GRACE notice fires ONLY when the
    #     visual disjunct (closer=="LIVE") is True -- closer=="LKG" leaves `_would_react` False, so that
    #     branch is skipped entirely (falls through silently to the probe's own, separate grace gate,
    #     which also returns None but never touches the notice).
    def _mk_dir57_probe(clearance=5.0, grace=10.0, t0=100.0):
        c = ExploreController(cfg_dir57, no_takeoff=True)
        c._ever_tracked = True
        c.loss_backoff_grace_s = grace
        c._last_good_clearance = clearance
        c._last_good_pos = [1.0, 1.0]
        c._last_good_t = 0.0
        c._loss_episode_t0 = t0
        return c

    now_dir57 = 105.0   # waited = 5.0 < grace = 10.0
    vm_dir57_live = VisualMatch(has_lkg=True, matched=True, inliers=50, contained=False, planar_like=True,
                                scale=1.0, closer="LIVE")
    c_dir57_live = _mk_dir57_probe()
    r_dir57_live = c_dir57_live._maybe_loss_snapshot_backoff({}, now_dir57, vm_dir57_live, status="PLAN-STALE")
    notice_dir57_live = c_dir57_live.take_notice()
    would_react_live_ok = (r_dir57_live is None and notice_dir57_live is not None
                           and "GRACE" in notice_dir57_live.upper())

    vm_dir57_lkg = VisualMatch(has_lkg=True, matched=True, inliers=50, contained=False, planar_like=True,
                               scale=1.0, closer="LKG")
    c_dir57_lkg = _mk_dir57_probe()
    r_dir57_lkg = c_dir57_lkg._maybe_loss_snapshot_backoff({}, now_dir57, vm_dir57_lkg, status="PLAN-STALE")
    notice_dir57_lkg = c_dir57_lkg.take_notice()
    would_react_lkg_ok = (r_dir57_lkg is None and notice_dir57_lkg is None)

    would_react_agrees = would_react_live_ok and would_react_lkg_ok

    dir57_ok = (planar_far_no_backoff and planar_near_backs_off and contained_far_no_backoff
               and would_react_agrees)
    ok = ok and dir57_ok
    print(f"[self-test] {'PASS' if planar_far_no_backoff else 'FAIL'}  SESSION-57 STALE DIRECTION GATE "
          f"planar_far_no_backoff (state={s_dir1})")
    print(f"[self-test] {'PASS' if planar_near_backs_off else 'FAIL'}  SESSION-57 STALE DIRECTION GATE "
          f"planar_near_backs_off (state={s_dir2})")
    print(f"[self-test] {'PASS' if contained_far_no_backoff else 'FAIL'}  SESSION-57 STALE DIRECTION GATE "
          f"contained_far_no_backoff (state={s_dir3})")
    print(f"[self-test] {'PASS' if would_react_agrees else 'FAIL'}  SESSION-57 STALE DIRECTION GATE "
          f"would_react_agrees (live={would_react_live_ok}, lkg={would_react_lkg_ok})")
    print(f"[self-test] {'PASS' if dir57_ok else 'FAIL'}  SESSION-57 STALE DIRECTION GATE overall")

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
