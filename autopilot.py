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
import json
import math
import os
import time
from datetime import datetime

import yaml

import frame_bus
from diag_log import DiagLog, NullLog
from flow_contact_detector import FlowContactDetector, detector_from_cfg, FlowVerdict, CMD_UP, CMD_FWD, CMD_BACK
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
    "joy_vertical": 0, "joy_horizontal": 0, "yaw": 0.0, "pitch": 0.0,
}


def _full_vector(active: dict, seq: int, now: float, state: str) -> dict:
    v = {"seq": seq, "mono_ts": now, "state": state}
    v.update(_NEUTRAL)
    v.update(active or {})
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


def _timeline_goals(plan: dict) -> list:
    """Goal markers for a replay step: the live goal (tagged `active`) + each blacklisted point tagged
    `blacklist_soft`/`blacklist_permanent`. Zips plan['blacklist'] points with plan['blacklist_permanent']
    flags (the same arrays the visualizer rings), so the HTML viewer can flip a goal gold->orange->red."""
    goals = []
    goal = plan.get("goal")
    if goal is not None:
        goals.append({"xz": [round(float(goal[0]), 4), round(float(goal[1]), 4)], "state": "active"})
    bl = plan.get("blacklist") or []
    perm = plan.get("blacklist_permanent") or []
    for i, pt in enumerate(bl):
        if pt is None:
            continue
        is_perm = bool(perm[i]) if i < len(perm) else False
        goals.append({"xz": [round(float(pt[0]), 4), round(float(pt[1]), 4)],
                      "state": "blacklist_permanent" if is_perm else "blacklist_soft"})
    return goals


def _timeline_step_record(t_wall, t_mono, rec_frame, state, event, status, plan: dict) -> dict:
    """One structured replay record per explore step. Pulls the pose/goal/slam fields straight off the
    plan payload (perception_worker._plan_payload) plus the controller's own state/event/status."""
    g = plan.get
    return {
        "t_wall": t_wall, "t_mono": round(float(t_mono), 3),
        "rec_frame": (int(rec_frame) if rec_frame is not None else None),
        "state": state, "event": event, "status": status,
        "pos": g("pos"), "heading": g("heading_deg"), "pos_y": g("pos_y"),
        "slam_ms": g("slam_ms"), "fwd_clear": g("forward_clearance_dist"),
        "top_clear": g("top_clear"), "goal": g("goal"), "bearing_err": g("bearing_err"),
        "goals": _timeline_goals(plan),
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
        self.slam_settle_frames = int(e.get("slam_settle_frames", 3))   # ">2 consecutive" fresh fast frames
        self._slam_fast_streak = 0        # consecutive FRESH frames under the slow threshold
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
        self._slam_stepback_count = 0     # step-backs taken during the CURRENT SLAM_HOLD
        self._slam_hold_start = None      # 'now' when the current SLAM_HOLD began (total-wait logging)
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
        # Altitude lock: hold the LIVE-cached mapping height during long ADVANCE pushes (forward pitch sinks
        # the drone into inner walls). target_altitude_y is cached live from the first valid post-prelude
        # pose (self-calibrating, NOT a baked value); world frame is +Y DOWN so a sink = LARGER y.
        self.altitude_lock = bool(e.get("altitude_lock", True))
        self.alt_drift_floor = float(e.get("alt_drift_floor", 0.3))
        self.target_altitude_y = None        # cached lazily; PERSISTS across reset_leg (flight-level hold target)
        # Depth inner-wall bump-up: when the forward clearance ray says "wall ahead" but the TOP band of the
        # depth frame is open air (a LOW inner wall), rise a little and keep advancing to fly OVER it instead
        # of stopping. Raising target_altitude_y makes the altitude lock hold the new height. The clearance
        # ray is cast at the drone's height (map_store), so once the drone clears the low wall the ray opens.
        # top_clear is a RELATIVE, self-calibrating depth signal (no baked height) -> leakage-safe. Bounded
        # per leg so a truly tall wall / glass (which depth reads as open air) can't make it climb forever.
        self.depth_bump_up = bool(e.get("depth_bump_up", True))
        self.top_clear_thresh = float(e.get("top_clear_thresh", 0.6))  # top-band openness needed to bump up
        self.bump_step = float(e.get("bump_step", 0.1))                # target-altitude raise per gentle pulse (SLAM units)
        self.bump_pulse_s = float(e.get("bump_pulse_s", 0.2))          # duration of ONE gentle up-pulse (bounded impulse)
        self.bump_max_per_leg = float(e.get("bump_max_per_leg", 0.3))  # cumulative rise budget per ADVANCE leg (anti-smash)
        self._bump_accum = 0.0               # rise applied on the current leg (reset when a new leg is planned)
        # Ram guard: "pushing forward but the SLAM pos isn't advancing toward the goal" = riding an unmapped
        # (invisible) collider. The forward-clearance ray can't see it (None when SLAM flickers; it also rises
        # with the drone as it climbs the wall) and the flow WALL needs a looming COLLAPSE that never comes on a
        # slow ram. So detect it in POS space: accrue forward-advancing time without progress; stop the leg
        # before the ram kills SLAM. Repeated re-commits then hit the F4 60 s stagnation blacklist.
        self.ram_stall_s = float(e.get("ram_stall_s", 3.0))            # advancing-but-not-progressing seconds -> stop the leg
        self.ram_progress_eps = float(e.get("ram_progress_eps", 0.15)) # min leg-distance improvement that counts as advancing
        self._ram_accum = 0.0                # accrued forward-advancing time with no leg progress
        self._leg_best_dist = None           # best distance to leg_goal this ADVANCE leg
        self._ram_last_t = None              # last advancing-tick time (for a clamped dt)
        # Parallax scouting: a goal needing MORE than one turn_step is reached as turn -> short translate
        # (forward/back per the rays, for SLAM parallax) -> settle -> turn again -> ... -> aim -> advance.
        self.parallax_scout = bool(e.get("parallax_scout", True))
        self.parallax_push_dist = float(e.get("parallax_push_dist", 0.5))  # translate this far per push (SLAM units)
        self.parallax_pad = float(e.get("parallax_pad", 0.4))
        self.parallax_push_s = float(e.get("parallax_push_s", 2.0))        # SAFETY time cap on a push
        # FORWARD push magnitude, DECOUPLED from the deliberately-slow ADVANCE forward_throttle (0.1 crawls
        # -> no parallax). The push is a short, clearance-guarded, deliberate translation -> it can be brisk.
        # Backward push keeps reverse_throttle (already strong enough). General platform param (HARD RULE).
        self.parallax_push_throttle = float(e.get("parallax_push_throttle", 0.4))
        self.parallax_max_pushes = int(e.get("parallax_max_pushes", 8))
        self._push_count = 0                 # consecutive scout pushes this leg (anti-deadlock cap)
        self._push_dir = None                # "forward" | "backward" for the active PARALLAX_PUSH
        self._push_start_pos = None          # SLAM pos at the start of the current push (distance gauge)
        self._after_orient = "ADVANCE"       # where ORIENT routes after the turn: ADVANCE (aimed) | PARALLAX_PUSH
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
        self._push_start_pos = None
        self._after_orient = "ADVANCE"
        self._fallback_attempts = 0
        self._fallback_retreat_forward = None
        self._slam_resume = None    # SLAM streak/latest persist (health is flight-level); only the pending resume clears
        self._slam_stepback_count = 0   # per-hold step-back counter + timer clear on interruption
        self._slam_hold_start = None
        self._bump_accum = 0.0      # bump-up budget is per-leg
        self._ram_accum = 0.0       # ram-guard accumulators are per-leg
        self._leg_best_dist = None
        self._ram_last_t = None
        # 2-bump blacklist latch (kinematic): an advance-blocked stop (flow WALL / ram-guard / stand-off)
        # emits ONE bump pulse to the planner, then DISARMS until the drone physically disengages (run_explore
        # re-arms on a published reverse command OR displacement > goal_reach_dist from the anchor). This
        # guarantees a single continuous contact counts as exactly one bump, immune to state-machine flicker.
        self._bump_armed = True
        self._last_bump_anchor = None   # [x,z] where the last counted bump fired (displacement re-arm gauge)
        self._bump_pulse = None         # pending bump goal for run_explore to publish, then clear
        # An interruption (autonomy off = a manual takeover) invalidates the command history: the drone may
        # have been moved by hand, so the recorded maneuvers no longer map to the trajectory. Drop it.
        self.command_history.clear()
        self.done = False

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

    # ------------------------------------------------- command history (control-space rewind)
    def _log_turn(self, theta):
        if abs(theta) > 1e-6:
            self.command_history.append({"kind": "turn", "theta": float(theta)})

    def _log_move(self, kind, value, duration):
        """Record a flown translation (kind='forward'|'reverse') for a later inverse replay. EVERY flown
        translation is logged — no minimum-duration guard: the SLAM-loss spiral is made of micro-short
        ADVANCE legs, and dropping them left the rewind with turns only (it just spun in place)."""
        self.command_history.append({"kind": kind, "value": float(value), "duration_s": float(max(0.0, duration))})

    def _invert_one(self, m):
        """Inverse recipe steps for ONE recorded maneuver (forward<->reverse, turn theta -> -theta).
        Shared by the full-history rewind and the single SLAM-settle step-back."""
        if m["kind"] == "turn":
            return list(self._turn_steps(-m["theta"]))
        if m["kind"] == "forward":
            return [{"reverse": m["value"], "duration_s": m["duration_s"]}]
        if m["kind"] == "reverse":
            return [{"trigger": m["value"], "duration_s": m["duration_s"]}]
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
        """PLAN-STALE (SLAM not TRACKING, perception publishing): RECOVERY_REWIND (replay the inverse of the
        recently-flown maneuvers to re-expose keyframes), watching for OK at the step() top; if the history
        is empty/exhausted -> the parallax + <=45deg fallback."""
        ring = plan.get("clearance_ring")
        if ring:
            self._last_ring = ring          # remember the last good ring for the fallback direction choice
        st = self.state
        if st in ("STUCK", "WARMUP"):
            return {}, st, None              # hold until OK returns (handled at the step() top)
        if st == "REWIND":
            active, done = self._player.fields(now)
            if not done:
                return active, "REWIND", None
            self._player = None
            return self._begin_fallback(now, "rewind exhausted, still not TRACKING -> parallax fallback")
        if st == "FALLBACK":
            active, done = self._player.fields(now)
            if not done:
                return active, "FALLBACK", None
            self._player = None
            if self._fallback_attempts >= self.fallback_max_attempts:
                self._enter("STUCK", now)
                return {}, "STUCK", (f"fallback exhausted ({self._fallback_attempts} attempts) -> STUCK "
                                     "(HOLD; awaiting perception)")
            return self._begin_fallback(now, None)
        # Entering recovery fresh (from a normal state or HOLD_LOST): start the control-space rewind.
        self._fallback_attempts = 0
        steps = self._invert_history()
        if steps:
            self._player = RecipePlayer(steps, name="rewind")
            self._enter("REWIND", now)
            # DIAGNOSTIC (bug-1 watch): report what the rewind is actually made of. A turns-only rewind =
            # translations never reached command_history (the spin-in-place failure we just fixed).
            n_turn = sum(1 for m in self.command_history if m["kind"] == "turn")
            n_move = sum(1 for m in self.command_history if m["kind"] in ("forward", "reverse"))
            move_s = sum(m.get("duration_s", 0.0) for m in self.command_history if m["kind"] != "turn")
            return {}, "REWIND", ("PLAN-STALE -> RECOVERY_REWIND: retracing the last "
                                  f"{self.command_history_s:g}s of maneuvers to re-expose keyframes "
                                  f"[history: {n_turn} turns, {n_move} translations / {move_s:.1f}s]")
        if not self._ever_tracked:
            # STARTUP: SLAM has never TRACKED yet (the prelude finishes on the FLOW ceiling detector, not on
            # SLAM). Don't spin a blind 360deg fallback into an unmapped room — HOLD and wait for SLAM to
            # initialize. The step() top snaps WARMUP -> SLAM_HOLD -> SETTLE -> REPLAN when OK returns.
            self._enter("WARMUP", now)
            return {}, "WARMUP", "PLAN-STALE at startup (SLAM still initializing) -> HOLD (no blind sweep)"
        return self._begin_fallback(now, "PLAN-STALE + EMPTY command history (post-collision?) -> parallax fallback")

    def _begin_fallback(self, now, event):
        """One fallback attempt: a SINGLE +45deg turn (UNIDIRECTIONAL sweep -> N attempts re-expose every past
        heading for RELOC; never the old 90/135/180 escalation) after a short parallax retreat, rest-separated.
        The RETREAT direction alternates fwd/back each attempt, seeded on attempt 0 by the roomier body axis
        (from the last-known ring) so we do not just wander into whichever wall killed the track."""
        self._fallback_attempts += 1
        if self._fallback_retreat_forward is None:      # attempt 0: seed retreat from the roomier axis
            fwd = self._ring_get(self._last_ring, 0.0)
            back = self._ring_get(self._last_ring, 180.0)
            # forward if strictly roomier ahead; default forward when the ring is unknown either way
            self._fallback_retreat_forward = (back is None) or (fwd is not None and fwd > back)
        if self._fallback_retreat_forward:
            move, tag = {"trigger": float(self.forward_preset.get("trigger", 0.2))}, "forward"
        else:
            move, tag = {"reverse": self.reverse_throttle}, "backward"
        self._fallback_retreat_forward = not self._fallback_retreat_forward   # alternate retreat axis each attempt
        theta = self.turn_step_deg                       # always +45deg: unidirectional RELOC sweep
        steps = [dict(move, duration_s=self.fallback_retreat_s),
                 {"duration_s": self.rest_between_s},
                 *self._turn_steps(theta),
                 {"duration_s": self.rest_between_s}]
        self._player = RecipePlayer(steps, name=f"fallback#{self._fallback_attempts}")
        self._enter("FALLBACK", now)
        ev = event or (f"FALLBACK #{self._fallback_attempts}: parallax {tag} + turn {theta:+.0f} "
                       "(<=45; ring-checked), then settle")
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
        if ms < self.slam_slow_ms:
            self._slam_fast_streak += 1
            self._slam_slow_streak = 0
        else:
            self._slam_fast_streak = 0
            self._slam_slow_streak += 1

    @property
    def _slam_slow(self):
        """The most recent FRESH frame took too long to build (SLAM choking; its pose is untrustworthy)."""
        return self._slam_ms_latest is not None and self._slam_ms_latest >= self.slam_slow_ms

    @property
    def _slam_stable(self):
        """More than the settle count of consecutive fresh frames each built fast -> the solve has settled."""
        return self._slam_fast_streak >= self.slam_settle_frames

    def _enter_slam_hold(self, resume, now, why):
        """Hover-hold (zero velocity) until SLAM settles, then re-enter `resume`. Returned by a gate site.
        Stamps the hold start + resets the per-hold step-back counter (a step-back re-enters SLAM_HOLD via
        `_enter` directly, so those persist across step-backs within one hold)."""
        self._slam_resume = resume
        self._player = None
        self._slam_stepback_count = 0
        self._slam_hold_start = now
        self._enter("SLAM_HOLD", now)
        return {}, "SLAM_HOLD", why

    def _enter(self, state, now):
        self.state = state
        self.t_state = now

    # ------------------------------------------------- 2-bump blacklist latch (kinematic)
    def _register_bump(self, plan):
        """Latch-gated bump for the event-driven 2-bump blacklist: on an advance-blocked stop (flow WALL /
        ram-guard / clearance stand-off) toward the committed goal, stash ONE bump pulse for run_explore to
        publish, then DISARM + record the stop position as the re-arm anchor. Suppressed while already
        disarmed (a stuttering state machine can't multiply-count one continuous contact)."""
        if not self._bump_armed or self.leg_goal is None:
            return
        self._bump_pulse = list(self.leg_goal)
        self._bump_armed = False
        pos = plan.get("pos")
        self._last_bump_anchor = list(pos) if pos is not None else None

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
        """Pop the pending bump goal (or None). run_explore publishes it on TOPIC_AUTOPILOT_EVENT."""
        g, self._bump_pulse = self._bump_pulse, None
        return g

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

    def step(self, now, plan, wall_contact, ceiling_contact=False, status="OK"):
        event = None
        active = {}
        st = self.state
        # Altitude lock: cache the hold target once, from the first valid pose after the prelude (lazy, so a
        # stale pose at the transition just defers it). Persists across reset_leg (flight-level reference).
        if (self.altitude_lock and self.airborne_done and self.target_altitude_y is None
                and plan.get("plan_valid") and plan.get("pos_y") is not None):
            self.target_altitude_y = float(plan["pos_y"])
        self._update_slam(plan)   # track SLAM frame-build time for the settle gate (below + at the gate sites)
        if plan.get("plan_valid"):
            self._ever_tracked = True   # SLAM has tracked at least once -> a later empty-history STALE is a real loss, not warmup

        # --- status-gated SLAM-loss recovery (CONTROL-SPACE); active only in the explore phase ---
        if self._explore_started:
            if status in ("PLAN-LOST", "NO-PLAN"):
                # Perception is SILENT. HARD HOVER-HOLD indefinitely — never move on a clock while we're
                # blind. Wait for perception to speak; the branch below (OK/STALE) then decides.
                if st != "HOLD_LOST":
                    self._player = None
                    self._enter("HOLD_LOST", now)
                    return {}, "HOLD_LOST", ("PLAN-LOST -> HARD HOVER-HOLD (indefinite; waiting for "
                                             "perception, no blind recovery)")
                return {}, "HOLD_LOST", None
            if status == "PLAN-STALE":
                # Perception is publishing but SLAM is not TRACKING -> retrace to re-expose keyframes.
                return self._step_stale(now, plan, wall_contact)
            # status OK: if we were recovering, perception is TRACKING again -> DON'T fly on the first frame
            # back (a fresh RELOC pose is shaky). Hold until SLAM settles (>N fast frames), THEN brake + REPLAN.
            if st in _RECOVERY_STATES:
                self._fallback_attempts = 0
                self._settle_to = "REPLAN"
                return self._enter_slam_hold("SETTLE", now,
                                             "plan OK -> wait for SLAM to settle -> brake -> replan (recovered)")

        # SLAM settle gate: hover until the solve is stable again, then resume the deferred state.
        if st == "SLAM_HOLD":
            if self._slam_stable:
                nxt = self._slam_resume or "REPLAN"
                self._slam_resume = None
                waited = now - (self._slam_hold_start if self._slam_hold_start is not None else self.t_state)
                self._enter(nxt, now)
                return {}, nxt, (f"SLAM settled after {waited:.1f}s ({self._slam_fast_streak} fast frames, "
                                 f"last {self._slam_ms_latest:.0f}ms) -> resume {nxt}")
            # SLAM is still choking. If it has been slow for a sustained run (and the plan is OK — LOST/STALE
            # are handled at the step() top), step one entry back through the rewind queue to re-expose
            # known-good geometry so the solve can re-lock. Re-arm needs another full run of slow frames.
            if self._slam_slow_streak >= self.slam_stepback_after_frames:
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
                self._settle_to = "ASCEND" if self.ascend_to_ceiling else "REPLAN"
                self._enter("SETTLE", now)
                event = "airborne -> settle -> " + ("ascend to ceiling" if self.ascend_to_ceiling else "explore")

        elif st == "ASCEND":
            # Climb until the flow CEILING fires (or a safety cap), so mapping runs at a consistent height.
            active = dict(self.ascend_preset)
            if ceiling_contact:
                self._settle_to = "DESCEND"
                self._enter("SETTLE", now)
                event = "CEILING reached -> settle -> descend a bit"
            elif (now - self.t_state) > self.ascend_max_s:
                self._settle_to = "DESCEND"
                self._enter("SETTLE", now)
                event = f"ascend cap ({self.ascend_max_s}s, no ceiling) -> settle -> descend a bit"

        elif st == "DESCEND":
            # Brief DOWN nudge (playbook "descend" recipe — tune its duration in flight_playbook.json)
            # so we sit a little below the ceiling while mapping.
            if self._player is None:
                self._player = self.pb.player("descend")
            active, ddone = self._player.fields(now)
            if ddone:
                self._player = None
                self._settle_to = "REPLAN"
                self._enter("SETTLE", now)
                event = "dropped a bit -> settle -> explore"

        elif st == "REPLAN":
            self._explore_started = True          # past the prelude -> status-gated recovery is now armed
            if plan.get("done"):
                self.done = True
                self._enter("DONE", now)
                event = "mission complete — no frontiers remain"
            elif plan.get("goal") is not None:
                self.leg_goal = list(plan["goal"])
                self._bump_accum = 0.0            # fresh bump-up budget for this leg
                self._ram_accum = 0.0             # fresh ram-guard tracking for this leg
                self._leg_best_dist = None
                self._ram_last_t = None
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
            # else: no goal (and not done) with a HEALTHY plan -> just idle in REPLAN (frontiers forming /
            # the planner's done-verification is choosing a corner). SLAM-loss recovery is status-driven
            # now (PLAN-STALE/LOST), NOT triggered from here.

        elif st == "STUCK":
            # HOLD (neutral) after the fallback gave up. A valid goal (SLAM re-acquired + planning) resumes.
            if plan.get("goal") is not None or plan.get("done"):
                self._fallback_attempts = 0
                self._enter("REPLAN", now)
                event = "plan recovered -> resume exploring"

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
            # Ram-guard progress tracking: real closing toward the goal resets the ram timer.
            if reached is not None and (self._leg_best_dist is None
                                        or reached < self._leg_best_dist - self.ram_progress_eps):
                self._leg_best_dist = reached
                self._ram_accum = 0.0
            if self._slam_slow:
                # SLAM started choking mid-leg -> STOP moving and let it settle before it loses the track.
                # Log the clean sub-leg flown so far (also keeps translations in the rewind history), then hold
                # and resume ADVANCE (leg_goal persists) once stable.
                self._log_move("forward", fwd_val, fwd_dur)
                return self._enter_slam_hold("ADVANCE", now,
                                             f"ADVANCE: SLAM slow ({self._slam_ms_latest:.0f}ms) -> "
                                             "hold to settle, then resume")
            if self.stop_on_clearance and clr is not None and clr <= self.stop_clearance_dist:
                top_clear = plan.get("top_clear")
                if (self.depth_bump_up and top_clear is not None and top_clear >= self.top_clear_thresh
                        and self.target_altitude_y is not None and self._bump_accum < self.bump_max_per_leg):
                    # The forward ray says a wall is close, but the TOP band of the depth frame is open air
                    # -> a LOW inner wall we can fly OVER. Rise a little via a GENTLE, ceiling-safe up-PULSE
                    # (state BUMP) instead of a sustained full-thrust climb (which smashed the ceiling), then
                    # re-check the stand-off. Bounded per leg by bump_max_per_leg.
                    self._log_move("forward", fwd_val, fwd_dur)   # record the clean forward sub-leg before the bump
                    self._enter("BUMP", now)
                    event = (f"clearance {clr:.2f} <= {self.stop_clearance_dist:.2f} but top clear "
                             f"({top_clear:.2f} >= {self.top_clear_thresh:.2f}) -> gentle BUMP UP over low wall")
                else:
                    # PRIMARY forward stop: SLAM has mapped a wall ahead within the stand-off margin. Stop
                    # gently with the image still rich (SLAM ALIVE) BEFORE ramming -> settle -> REPLAN picks
                    # the next frontier. No reverse/back-off: we're already at a safe stand-off.
                    self._log_move("forward", fwd_val, fwd_dur)   # record the clean forward leg for a later rewind
                    self._register_bump(plan)     # advance-blocked stop -> a bump toward the committed goal
                    self._enter("SETTLE", now)
                    event = f"clearance {clr:.2f} <= {self.stop_clearance_dist:.2f} -> standoff stop -> settle"
            elif wall_contact:
                # A COLLISION invalidates the command history (unknown post-impact orientation) -> drop it.
                self.command_history.clear()
                self._register_bump(plan)         # advance-blocked stop -> a bump toward the committed goal
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
            else:
                # Ram guard: accrue forward-advancing time (clamped dt, so an interleaved hold doesn't dump)
                # and stop the leg once we've pushed forward `ram_stall_s` without any leg progress — the drone
                # is riding an invisible collider the sensors can't stop it on. Hand back to REPLAN; a repeated
                # re-commit then hits the F4 stagnation blacklist.
                dt = 0.0 if self._ram_last_t is None else min(max(now - self._ram_last_t, 0.0), 0.5)
                self._ram_last_t = now
                self._ram_accum += dt
                if self.ram_stall_s > 0 and reached is not None and self._ram_accum >= self.ram_stall_s:
                    self._log_move("forward", fwd_val, fwd_dur)
                    self._register_bump(plan)     # advance-blocked stop -> a bump toward the committed goal
                    self._enter("SETTLE", now)
                    event = (f"ram guard: forward-commanded but not advancing {self._ram_accum:.1f}s "
                             f"(d={reached:.2f}) -> stop leg -> settle -> replan")
                else:
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

        elif st == "BUMP":
            # GENTLE, ceiling-safe up-pulse to clear a LOW inner wall (replaces the old sustained full-thrust
            # climb that smashed the ceiling). Command UP only for `bump_pulse_s` (a bounded impulse), then
            # raise the altitude-lock target by `bump_step` and rest (SETTLE) so momentum dies before we
            # re-check the stand-off in ADVANCE. BUMP maps to CMD_UP so the flow CEILING detector is armed;
            # a ceiling contact (or `top_clear` dropping on the ADVANCE re-check) ends bumping this leg.
            if ceiling_contact:
                self._bump_accum = self.bump_max_per_leg   # stop bumping this leg -> next stand-off will SETTLE
                self._settle_to = "REPLAN"
                self._enter("SETTLE", now)
                event = "BUMP: CEILING contact -> stop bumping -> settle"
            elif (now - self.t_state) >= self.bump_pulse_s:
                self.target_altitude_y -= self.bump_step   # +Y is DOWN -> subtract to rise; lock holds the gain
                self._bump_accum += self.bump_step
                self._settle_to = "ADVANCE"                # rest, then re-check the stand-off (did we clear it?)
                self._enter("SETTLE", now)
                event = (f"BUMP: up-pulse done (+{self._bump_accum:.2f}/{self.bump_max_per_leg:.2f}) "
                         "-> settle -> re-check")
            else:
                active = {"joy_vertical": self.ascend_preset["joy_vertical"]}   # -1 = up (gentle bounded pulse)

        elif st == "REVERSE_PROBE":
            # EXPERIMENT: sustained straight reverse (playbook "reverse_probe" recipe — tune its duration
            # there). The BACKWALL detector now arms here (the command is derived from the reverse control
            # vector) but is DETECTION-ONLY this session: it logs a back-wall contact, takes no action. Then
            # settle -> replan, and watch whether the plan stayed OK + the path kept growing (PASS) vs STALE.
            if self._player is None:
                self._player = self.pb.player("reverse_probe")
            active, rdone = self._player.fields(now)
            if rdone:
                self._player = None
                self._settle_to = "REPLAN"
                self._enter("SETTLE", now)
                event = "reverse probe done -> settle -> replan"

        elif st == "PARALLAX_PUSH":
            # Short open-loop translation BETWEEN rotation steps, to give SLAM the parallax it needs to
            # survive a multi-step turn (and to stay roughly in place rather than advance off-goal). The
            # rays pick the roomier body axis (forward vs backward) from the CURRENT post-turn ring; if
            # boxed in both ways we skip and just turn again. Distance-quantized (translate ~parallax_push_dist
            # SLAM units, measured live from the pose), guarded by the live clearance, with a SAFETY time cap.
            # The detector arms per the actual push direction (WALL fwd / BACKWALL back) but open-loop control
            # here doesn't react to it; a stale pose mid-push rides the cap.
            ring = plan.get("clearance_ring")
            pad = self.stop_clearance_dist + self.parallax_pad
            if self._push_dir is None:        # first tick: choose the axis from the post-turn ring
                fwd, back = self._ring_get(ring, 0.0), self._ring_get(ring, 180.0)
                rel, room = max([(0.0, fwd), (180.0, back)],
                                key=lambda kv: kv[1] if kv[1] is not None else -1.0)
                if room is None or room <= pad:
                    self._settle_to = "REPLAN"   # boxed in -> can't push safely -> turn again next REPLAN
                    self._enter("SETTLE", now)
                    event = "parallax: no room fwd/back -> skip push -> settle -> replan"
                else:
                    self._push_dir = "forward" if rel == 0.0 else "backward"
                    self._push_count += 1
                    self._push_start_pos = plan.get("pos")
            if self.state == "PARALLAX_PUSH":    # still pushing (didn't bail to SETTLE above)
                if self._push_dir == "forward":
                    active = {"trigger": self.parallax_push_throttle}   # brisk, decoupled from the ADVANCE crawl
                    guard = self._ring_get(ring, 0.0)
                else:
                    active = dict(self.pb.recipe("back_off")[0])   # reverse magnitude, held continuously
                    active.pop("duration_s", None)
                    guard = self._ring_get(ring, 180.0)
                if self._slam_slow:
                    # SLAM choking mid-push -> log what we translated, stop, and settle before re-planning.
                    kind = "forward" if self._push_dir == "forward" else "reverse"
                    mag = self.parallax_push_throttle if kind == "forward" else self.reverse_throttle
                    self._log_move(kind, mag, now - self.t_state)
                    self._push_dir = None
                    self._settle_to = "REPLAN"
                    return self._enter_slam_hold("SETTLE", now,
                                                 f"parallax push: SLAM slow ({self._slam_ms_latest:.0f}ms) -> "
                                                 "hold to settle -> replan")
                traveled = self._dist(plan.get("pos"), self._push_start_pos)
                far = traveled is not None and traveled >= self.parallax_push_dist
                blocked = guard is not None and guard <= self.stop_clearance_dist
                timeout = (now - self.t_state) >= self.parallax_push_s
                if far or blocked or timeout:
                    why = "dist" if far else "blocked" if blocked else "timer"
                    dirn = self._push_dir
                    if dirn == "forward":
                        self._log_move("forward", self.parallax_push_throttle, now - self.t_state)
                    else:
                        self._log_move("reverse", self.reverse_throttle, now - self.t_state)
                    self._push_dir = None
                    self._settle_to = "REPLAN"
                    self._enter("SETTLE", now)
                    event = f"parallax {dirn} push done ({why}) -> settle -> replan"

        elif st == "SETTLE":
            if (now - self.t_state) >= self.rest_between_s:
                nxt = self._settle_to or "REPLAN"    # prelude chains via _settle_to; legs default REPLAN
                self._settle_to = None
                self._enter(nxt, now)

        # st == "DONE": active stays neutral (HOLD)
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
    if float(active.get("joy_vertical", 0.0) or 0.0) < 0.0:   # -1 = up (camera Y down)
        return CMD_UP
    return None


# Recovery states (SLAM-loss). The step() top snaps out of these to a brake+REPLAN when the plan returns OK.
_RECOVERY_STATES = {"HOLD_LOST", "REWIND", "FALLBACK", "STUCK", "WARMUP"}


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
    prev_ctrl_state = None
    prev_active = {}      # last published control vector -> derives the detector command for THIS frame
    backwall_active = False   # BACKWALL contact edge tracker (log once per onset)
    last_ground = None    # newest GroundGrid summary; the final room outline is emitted ONCE at shutdown as
                          # a static backdrop (we don't replay the map growing — only the pose + goals matter)

    def log_cmd(active, source):
        nonlocal last_cmd_key
        key = (source, json.dumps(active, sort_keys=True))
        if key == last_cmd_key:
            return
        last_cmd_key = key
        line = (f"{_rec_prefix(last_rec_frame)} [autopilot][explore][CMD] state={source} "
                f"fields={json.dumps(active, sort_keys=True)}")
        print(line, flush=True)
        diag.line(line)
        diag.cmd(last_rec_frame, seq, source, source, active)

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
            now = time.monotonic()
            msg = sub.recv(timeout_ms=20)
            frame = meta = None
            if msg is not None:
                frame, meta = msg
                if meta.get("rec_frame") is not None:
                    last_rec_frame = meta.get("rec_frame")
            # drain the plan bus to the freshest message
            p = plan_sub.recv(timeout_ms=0)
            while p is not None:
                last_plan = p[1]
                last_plan_t = now
                p = plan_sub.recv(timeout_ms=0)

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
            wall_contact = ceiling_contact = False
            if frame is not None:
                command = _detector_command(prev_active)   # UP (CEILING) / FWD (WALL) / BACK (BACKWALL) / None
                v = detector.update(now, frame, command)
                if command in (CMD_FWD, CMD_UP, CMD_BACK):
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
                    # BACKWALL is DETECTION-ONLY this session (no control reaction yet). Log its onset once so
                    # the next flight captures the reverse-into-wall signal (NO SILENT FALLBACK: operator sees it).
                    now_backwall = bool(v.contact and v.kind == "BACKWALL" and command == CMD_BACK)
                    if now_backwall and not backwall_active:
                        line = (f"{_rec_prefix(last_rec_frame)} [autopilot][explore][{ctrl.state}] BACKWALL "
                                f"contact (reverse into a wall; detection-only — no reaction yet)")
                        print(line, flush=True)
                        diag.line(line)
                    backwall_active = now_backwall
                else:
                    backwall_active = False

            # ---- step the controller + publish ----
            active, state, event = ctrl.step(now, plan_for_step, wall_contact, ceiling_contact, status=status)
            if state == "ADVANCE" and prev_ctrl_state != "ADVANCE":
                detector.reset_forward_ref()   # each leg recalibrates its own free-forward looming
            prev_ctrl_state = state
            prev_active = active               # command for the NEXT frame's detector + the bump re-arm test
            # 2-bump latch: re-arm once the drone has disengaged (backward cmd OR moved > goal_reach_dist),
            # then publish any pending bump pulse for the planner's event-driven blacklist.
            ctrl.rearm_bump_if_disengaged(active, plan_for_step)
            bump_goal = ctrl.take_bump_pulse()
            if bump_goal is not None:
                pub.publish(frame_bus.TOPIC_AUTOPILOT_EVENT, {"bump_goal": bump_goal, "seq": bump_seq})
                bline = f"{_rec_prefix(last_rec_frame)} [autopilot][explore] BUMP pulse #{bump_seq} goal={bump_goal} (advance-blocked -> planner)"
                print(bline, flush=True)
                diag.line(bline)
                bump_seq += 1
            if event:
                line = f"{_rec_prefix(last_rec_frame)} [autopilot][explore] {state}: {event}"
                print(line, flush=True)
                diag.line(line)
            publish(active, state)

            # ---- F8 replay timeline (purely additive; --log-gated via the no-op sink) ----
            # ONE record per step (pose + goal states); the room outline is NOT streamed — we keep only the
            # newest ground summary and emit it once at shutdown as a static backdrop for the whole replay.
            if log:
                t_wall = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                diag.timeline(_timeline_step_record(t_wall, now, last_rec_frame, state, event,
                                                    status, plan_for_step))
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
def _drive(ctrl, plan, wall, seconds, t0, dt=0.05, ceiling=False, status="OK"):
    """Step ExploreController over `seconds` at dt with a fixed plan/wall/ceiling/status. Returns
    (t_end, last_active, last_state, states_visited)."""
    states, active, state = [], {}, ctrl.state
    t = t0
    for _ in range(max(1, int(seconds / dt))):
        active, state, _ev = ctrl.step(t, plan, wall, ceiling, status=status)
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

    # F8 replay timeline: the JSONL sink is --log-gated, so a disabled AutopilotLog must swallow
    # .timeline()/.line() as no-ops (no file, no crash) — the path taken when self-test/dry constructs run.
    dl = AutopilotLog(False)
    try:
        dl.timeline({"state": "ADVANCE", "goals": []})
        dl.line("noop")
        tl_noop = (dl._jsonl is None and dl._txt is None)
    finally:
        dl.close()
    # And the pure record builders produce the expected shape (goal state tagging + map downsample).
    plan = {"pos": [0.1, 0.2], "heading_deg": 45.0, "goal": [1.0, 2.0], "bearing_err": 3.0,
            "blacklist": [[9.0, 9.0]], "blacklist_permanent": [True]}
    rec = _timeline_step_record("00:00:01.000", 1.234, 7, "ADVANCE", "leg", "OK", plan)
    ds = _downsample_map({"bounds": [0, 4, 0, 4], "rows": 2, "cols": 2, "cls": [0, 1, 2, 3]})
    tl_rec = (rec["state"] == "ADVANCE" and rec["pos"] == [0.1, 0.2] and len(rec["goals"]) == 2
              and rec["goals"][0]["state"] == "active"
              and rec["goals"][1]["state"] == "blacklist_permanent"
              and ds["rows"] == 2 and ds["cls"] == [0, 1, 2, 3])
    good = tl_noop and tl_rec
    ok = ok and good
    print(f"[self-test] {'PASS' if good else 'FAIL'}  F8 timeline (disabled sink no-op={tl_noop}, "
          f"record/map builders={tl_rec})")

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
    goal = [1.0, 0.0]
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
    # Frontiers exhausted during the settle window: a DONE plan must carry SETTLE -> REPLAN -> DONE.
    t, a, s, st = _drive(ctrl, {"done": True, "goal": None, "pos": [0.0, 0.0], "bearing_err": None}, False, ctrl.rest_between_s + 0.4, t)
    rec(st)
    leg_ok = (yaw_pos and advancing and ctrl.done
              and _is_subsequence(["ORIENT", "ADVANCE", "BACKOFF", "SETTLE", "REPLAN", "DONE"], order))
    ok = ok and leg_ok
    print(f"[self-test] {'PASS' if leg_ok else 'FAIL'}  explore leg ORIENT(turn+)->ADVANCE->WALL->"
          f"BACKOFF->SETTLE->DONE  (visited {order})")

    # ---- Map mode: reverse-probe EXPERIMENT (flag on) — clamp leg turn to ONE step; WALL -> reverse probe ----
    # With reverse_probe_on_wall: a big bearing err is clamped to ONE turn_step (SLAM stays alive at the
    # wall), and a WALL hit goes ADVANCE -> SETTLE -> REVERSE_PROBE (sustained reverse) -> SETTLE -> REPLAN
    # (NOT back_off). The BACKWALL detector arms in REVERSE_PROBE (detection-only) but takes no action here.
    cre = ExploreController(cfg, no_takeoff=True)
    cre.reverse_probe_on_wall = True
    plan_e = {"done": False, "goal": [1.0, 0.0], "pos": [0.0, 0.0], "bearing_err": 135.0}  # would be +135 (3 steps) unclamped
    eorder, prev_e = [], None
    saw_reverse, saw_backoff, turn_name = False, False, None
    te, dt, wall = 200.0, 0.05, False
    for _ in range(int(14.0 / dt)):
        if cre.state == "ADVANCE":
            wall = True                       # trip the wall the moment we start advancing
        a, s, _ = cre.step(te, plan_e, wall, False)
        if s != prev_e:
            eorder.append(s)
            prev_e = s
        if s == "ORIENT" and turn_name is None:
            turn_name = cre._player.name      # clamped open-loop turn -> "turn+45", not "turn+135"
        if s == "REVERSE_PROBE" and float(a.get("reverse", 0.0)) > 0:
            saw_reverse = True
        if s == "BACKOFF":
            saw_backoff = True
        te += dt
    clamp_ok = (turn_name == "turn+45")       # +135 bearing clamped to one +45 step
    rev_path_ok = _is_subsequence(["ORIENT", "ADVANCE", "SETTLE", "REVERSE_PROBE", "SETTLE", "REPLAN"], eorder)
    rev_ok = (clamp_ok and saw_reverse and rev_path_ok and not saw_backoff)
    ok = ok and rev_ok
    print(f"[self-test] {'PASS' if rev_ok else 'FAIL'}  explore REVERSE-PROBE (clamp +135->{turn_name}, "
          f"WALL->SETTLE->REVERSE_PROBE(reverse>0)->SETTLE->REPLAN, no back_off)  visited {eorder}")

    # ---- Map mode: forward-clearance STAND-OFF (primary forward stop; SLAM-preserving) ----
    # A mapped wall ahead within stop_clearance_dist stops the ADVANCE leg WITHOUT a wall_contact, routing
    # to SETTLE -> REPLAN (no back_off / reverse). A large or None clearance keeps advancing (lean on the
    # flow detector). NB the clearance check is FIRST in ADVANCE, so it acts before the image ever freezes.
    cs = ExploreController(cfg, no_takeoff=True)
    cs_on = cs.stop_on_clearance                          # config default true
    big = {"done": False, "goal": [3.0, 0.0], "pos": [0.0, 0.0], "bearing_err": 0.0, "forward_clearance_dist": 5.0}
    t, a, s, _ = _drive(cs, big, False, 0.6, 100.0)       # far clearance -> still advancing
    adv_big = (s == "ADVANCE" and float(a.get("trigger", 0)) > 0)
    near = dict(big, forward_clearance_dist=cs.stop_clearance_dist - 0.05)
    _, _, s2, st_stop = _drive(cs, near, False, 0.1, t)   # clearance under the margin -> stand-off stop
    stop_settle = (s2 == "SETTLE") and ("BACKOFF" not in st_stop) and ("REVERSE_PROBE" not in st_stop)
    cn = ExploreController(cfg, no_takeoff=True)
    _, an, sn, _ = _drive(cn, dict(big, forward_clearance_dist=None), False, 0.6, 0.0)  # None -> keep advancing
    adv_none = (sn == "ADVANCE" and float(an.get("trigger", 0)) > 0)
    clr_ok = (cs_on and adv_big and stop_settle and adv_none)
    ok = ok and clr_ok
    print(f"[self-test] {'PASS' if clr_ok else 'FAIL'}  explore CLEARANCE-STOP (far->advance, "
          f"<{cs.stop_clearance_dist:g}->standoff settle (no backoff/reverse), None->advance)")

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
    # (c) distance-quantized: the push ends by 'dist' once translated parallax_push_dist (before the time cap),
    #     and the FORWARD push uses the brisk parallax_push_throttle (decoupled from the 0.1 ADVANCE crawl).
    cd = ExploreController(cfg, no_takeoff=True)
    cd._enter("PARALLAX_PUSH", 0.0)
    cd._push_dir = None
    tt, moved, ended, push_trig = 0.0, 0.0, None, None
    for _ in range(400):
        a, s, _ = cd.step(tt, _plan_be(90.0, pos=(0.0, moved)), False)
        if s != "PARALLAX_PUSH":
            ended = (moved, tt)
            break
        if a.get("trigger") is not None:      # forward push magnitude actually commanded
            push_trig = a["trigger"]
        moved += 0.05                         # drone translates 0.05u/tick -> reaches 0.5u well before the cap
        tt += 0.05
    dist_stop = (ended is not None and ended[0] >= cd.parallax_push_dist - 1e-6 and ended[1] < cd.parallax_push_s
                 and push_trig is not None and abs(push_trig - cd.parallax_push_throttle) < 1e-9)
    # (d) boxed in (no room fwd/back) -> skip the push (enter PARALLAX_PUSH but bail, no push counted).
    cb = ExploreController(cfg, no_takeoff=True)
    tight = [[r, 0.5] for r in (0.0, 45.0, 90.0, 135.0, 180.0, -135.0, -90.0, -45.0)]
    _, _, _, stb = _drive(cb, _plan_be(135.0, ring=tight), False, 2.0, 0.0)
    boxed_skip = ("PARALLAX_PUSH" in stb) and (cb._push_count == 0)
    scout_ok = (multi_push and aim_adv and dist_stop and boxed_skip)
    ok = ok and scout_ok
    print(f"[self-test] {'PASS' if scout_ok else 'FAIL'}  explore PARALLAX-SCOUT (multi-step->turn+push, "
          f"aim->advance, dist-stop@{ended[0] if ended else '?'}, boxed->skip)")

    # Negative bearing error -> open-loop turn yaw NEGATIVE (turn left).
    c2 = ExploreController(cfg, no_takeoff=True)
    _, a2, s2, _ = _drive(c2, {"done": False, "goal": [-1.0, 0.0], "pos": [0.0, 0.0], "bearing_err": -90.0}, False, 0.3, 0.0)
    yaw_neg = (s2 == "ORIENT" and a2.get("yaw", 0.0) < 0)
    # Quantization: nearest whole turn_step_deg (default 45) aim change.
    q = c2._quantize_turn
    quant_ok = (q(70) == 90 and q(50) == 45 and q(10) == 0 and q(-70) == -90 and q(None) == 0)
    # theta≈0 (small err) -> no turn, just the 'c' reset -> ADVANCE; then goal reached with NO wall -> SETTLE.
    c4 = ExploreController(cfg, no_takeoff=True)
    t4, _, s4a, st4a = _drive(c4, {"done": False, "goal": [1.0, 0.0], "pos": [0.0, 0.0], "bearing_err": 5.0}, False, 0.6, 0.0)
    _, _, _, st4 = _drive(c4, {"done": False, "goal": [1.0, 0.0], "pos": [0.9, 0.0], "bearing_err": 5.0}, False, 0.2, t4)
    reached_ok = ("ADVANCE" in st4a) and ("SETTLE" in st4)
    edges_ok = yaw_neg and quant_ok and reached_ok
    ok = ok and edges_ok
    print(f"[self-test] {'PASS' if edges_ok else 'FAIL'}  explore edges (turn- left, quantize 70->90/50->45/"
          f"10->0, theta~0->reset->ADVANCE, goal-reached settle)")

    # ---- Map mode: PRELUDE arm + takeoff + ASCEND-to-ceiling + descend (explore gets airborne + to height) ----
    ascend = int(cfg["autonomy"]["ascend_cmd"])
    plan_goal = {"done": False, "goal": [1.0, 0.0], "pos": [0.0, 0.0], "bearing_err": 90.0}
    cp = ExploreController(cfg)                      # default: full prelude
    porder, saw_arm, saw_to_up, saw_asc_up, saw_desc = [], False, False, False, False
    asc_seen = 0
    t = 0.0
    for _ in range(int(13.0 / 0.05)):
        cur = cp.state
        fire_ceiling = (cur == "ASCEND" and asc_seen >= 4)   # let it ascend ~0.2s, then CEILING fires
        active, _state, _ev = cp.step(t, plan_goal, False, ceiling_contact=fire_ceiling)
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
        t += 0.05
    prelude_ok = (saw_arm and saw_to_up and saw_asc_up and saw_desc and cp.airborne_done
                  and _is_subsequence(["ARM", "TAKEOFF", "ASCEND", "DESCEND", "REPLAN", "ORIENT"], porder))
    # reset_leg AFTER airborne must NOT re-run the prelude (-> REPLAN); a grounded controller restarts at ARM.
    cp.reset_leg()
    no_rearm = (cp.state == "REPLAN")
    cg = ExploreController(cfg)
    cg.step(0.0, plan_goal, False)                  # enters ARM (not yet airborne)
    cg.reset_leg()
    rearm_if_grounded = (cg.state == "ARM")
    prelude_ok = prelude_ok and no_rearm and rearm_if_grounded
    ok = ok and prelude_ok
    print(f"[self-test] {'PASS' if prelude_ok else 'FAIL'}  explore PRELUDE arm+takeoff+ascend-to-ceiling+descend "
          f"(ascend joy={ascend}, descend joy={-ascend}, no re-run once airborne)  visited {porder}")

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
    cw.slam_settle_frames = 1          # one fresh fast frame is enough to declare settled for this test
    cw.command_history.append({"kind": "forward", "value": 0.2, "duration_s": 2.0})
    cw.command_history.append({"kind": "turn", "theta": 45.0})
    stale = {"plan_valid": False, "goal": None, "pos": [0.0, 0.0], "clearance_ring": None}
    t, _, _, st_st = _drive(cw, stale, False, 1.0, 0.0, status="PLAN-STALE")
    rewind_ok = ("REWIND" in st_st)
    _, _, so, st_ok = _drive(cw, {"plan_valid": True, "goal": [1.0, 0.0], "pos": [0.0, 0.0], "bearing_err": 0.0,
                                  "slam_ms": 120.0, "frame_id": 1},
                             False, cw.rest_between_s + 0.6, t, status="OK")
    # recovery-exit now HOLDs for SLAM to settle before braking (strengthen the solve) -> SETTLE -> replan.
    snap_ok = ("SLAM_HOLD" in st_ok) and ("SETTLE" in st_ok) and (so in ("REPLAN", "ORIENT", "ADVANCE"))
    # (d) PLAN-STALE + EMPTY history -> parallax+<=45 FALLBACK -> STUCK after cap. The sweep is UNIDIRECTIONAL
    #     (turn always +45, never <0) while the RETREAT alternates fwd/back (seeded forward by the roomier ring).
    cf = ExploreController(cfg, no_takeoff=True)
    cf.fallback_max_attempts = 3            # small cap so STUCK is reached within the drive window
    cf._ever_tracked = True                 # a MID-FLIGHT loss (history wiped by a wall hit), not startup warmup
    cf.command_history.clear()
    cf._last_ring = [[0.0, 5.0], [45.0, 5.0], [90.0, 5.0], [135.0, 1.0],
                     [180.0, 1.0], [-135.0, 1.0], [-90.0, 5.0], [-45.0, 5.0]]   # forward roomier than back
    seen, saw_fwd, saw_back, saw_turn_pos, saw_turn_neg, t = set(), False, False, False, False, 0.0
    for _ in range(int(30.0 / 0.05)):
        a, s, _ = cf.step(t, stale, False, status="PLAN-STALE")
        seen.add(s)
        if s == "FALLBACK":
            if float(a.get("trigger", 0.0)) > 0:
                saw_fwd = True                        # forward retreat occurred
            if float(a.get("reverse", 0.0)) > 0:
                saw_back = True                       # backward retreat occurred (alternation)
            y = float(a.get("yaw", 0.0))
            if y > 0:
                saw_turn_pos = True
            if y < 0:
                saw_turn_neg = True                   # must NEVER happen (unidirectional sweep)
        t += 0.05
    fallback_ok = ("FALLBACK" in seen and "STUCK" in seen and saw_fwd and saw_back
                   and saw_turn_pos and not saw_turn_neg)
    # the fallback turn is a SINGLE <=45 step (built from turn_step_deg), never the old 90/135/180 escalation.
    fallback_le45 = (cf.turn_step_deg <= 45.0)
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

    # ---- SLAM frame-timing settle gate (stop moving while SLAM chokes; resume once it settles) ----
    # (a) _update_slam: counts consecutive FRESH fast frames (deduped on frame_id); a slow frame resets.
    cs = ExploreController(cfg, no_takeoff=True)
    cs.slam_slow_ms, cs.slam_settle_frames = 1000.0, 3
    cs._update_slam({"slam_ms": 200, "frame_id": 1})
    cs._update_slam({"slam_ms": 200, "frame_id": 1})              # same frame_id -> counted once
    streak1 = (cs._slam_fast_streak == 1)
    cs._update_slam({"slam_ms": 200, "frame_id": 2})
    cs._update_slam({"slam_ms": 200, "frame_id": 3})             # now >2 fresh fast frames
    stable_ok = cs._slam_stable and not cs._slam_slow
    cs._update_slam({"slam_ms": 1500, "frame_id": 4})           # a slow fresh frame resets the streak
    slow_ok = cs._slam_slow and (cs._slam_fast_streak == 0) and (not cs._slam_stable)
    track_ok = streak1 and stable_ok and slow_ok

    # (b) ADVANCE + a slow frame -> SLAM_HOLD (logs the sub-leg), then fast frames settle -> resume ADVANCE.
    cadv = ExploreController(cfg, no_takeoff=True)
    cadv.slam_settle_frames = 2
    padv2 = {"plan_valid": True, "done": False, "goal": [5.0, 0.0], "pos": [0.0, 0.0], "bearing_err": 0.0,
             "forward_clearance_dist": 5.0}
    t = 0.0
    for i in range(30):
        _a, s, _ = cadv.step(t, dict(padv2, frame_id=i, slam_ms=200.0), False, status="OK"); t += 0.05
        if s == "ADVANCE":
            break
    reached_adv = (cadv.state == "ADVANCE")
    _a, s_hold, _ = cadv.step(t, dict(padv2, frame_id=100, slam_ms=1500.0), False, status="OK"); t += 0.05
    adv_held = (s_hold == "SLAM_HOLD")
    logged_fwd = any(m["kind"] == "forward" for m in cadv.command_history)
    for i in range(101, 106):
        _a, _s, _ = cadv.step(t, dict(padv2, frame_id=i, slam_ms=200.0), False, status="OK"); t += 0.05
    adv_resumed = (cadv.state == "ADVANCE")
    adv_gate_ok = reached_adv and adv_held and logged_fwd and adv_resumed

    # (c2) a slow frame AT turn completion -> hold before flying the shaky post-turn pose (the ~45deg gap).
    cpt = ExploreController(cfg, no_takeoff=True)
    cpt.slam_settle_frames = 2
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
    csb2.slam_settle_frames, csb2.slam_stepback_after_frames, csb2.slam_stepback_max_steps = 3, 4, 2
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
    csb3.slam_settle_frames, csb3.slam_stepback_after_frames, csb3.slam_stepback_max_steps = 3, 3, 2
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

    # ---- F5 gentle + ceiling-safe bump-up: at a stand-off with the top band clear, a bounded UP-PULSE
    #      (BUMP state) raises the altitude target and re-checks; ceiling contact aborts; capped per leg ----
    pbump = {"plan_valid": True, "done": False, "goal": [9.0, 0.0], "pos": [0.0, 0.0], "bearing_err": 0.0,
             "forward_clearance_dist": 0.3, "top_clear": 0.9, "pos_y": 0.0}
    def _reach_advance(ctrl, plan):
        t, f = 0.0, 0
        for _ in range(60):
            _a, s, _ = ctrl.step(t, dict(plan, frame_id=f, slam_ms=200.0, forward_clearance_dist=9.0),
                                 False, status="OK"); t += 0.05; f += 1
            if s == "ADVANCE":
                return t, f
        return t, f
    cbu = ExploreController(cfg, no_takeoff=True)
    cbu.rest_between_s, cbu.bump_pulse_s, cbu.ram_stall_s = 0.0, 0.1, 0.0   # speed up + isolate from ram guard
    cbu.depth_bump_up, cbu.top_clear_thresh, cbu.bump_step, cbu.bump_max_per_leg = True, 0.6, 0.1, 0.25
    cbu.target_altitude_y = 0.0
    tb, fb = _reach_advance(cbu, pbump)
    y0 = cbu.target_altitude_y
    saw_bump = saw_up = saw_replan = False
    for _ in range(300):   # BUMP pulses (up) -> settle -> re-check; budget caps -> stand-off stop -> REPLAN
        a, s, _ = cbu.step(tb, dict(pbump, frame_id=fb, slam_ms=200.0), False, status="OK"); tb += 0.05; fb += 1
        if s == "BUMP":
            saw_bump = True
            if a.get("joy_vertical") == cbu.ascend_preset["joy_vertical"]:
                saw_up = True
        if s == "REPLAN":
            saw_replan = True; break
    raised = cbu.target_altitude_y < y0            # +Y is DOWN -> rising lowers the target
    capped = cbu._bump_accum <= cbu.bump_max_per_leg + cbu.bump_step + 1e-9   # no runaway climb
    bump_ok = saw_bump and saw_up and raised and saw_replan and capped

    # ceiling contact DURING a bump aborts the climb (no smash)
    cbc = ExploreController(cfg, no_takeoff=True)
    cbc.rest_between_s, cbc.bump_pulse_s, cbc.ram_stall_s = 0.0, 0.5, 0.0
    cbc.depth_bump_up, cbc.target_altitude_y = True, 0.0
    tb, fb = _reach_advance(cbc, pbump)
    cbc.step(tb, dict(pbump, frame_id=fb, slam_ms=200.0), False, status="OK"); tb += 0.05; fb += 1   # enter BUMP
    _a, s_c, _ = cbc.step(tb, dict(pbump, frame_id=fb, slam_ms=200.0), False, ceiling_contact=True, status="OK")
    ceiling_abort_ok = (cbc._bump_accum >= cbc.bump_max_per_leg)   # ceiling -> stop bumping this leg

    # a LOW top-clear must NOT bump (a real tall wall ahead) -> normal stand-off stop
    cbu2 = ExploreController(cfg, no_takeoff=True)
    cbu2.ram_stall_s, cbu2.target_altitude_y = 0.0, 0.0
    tb, fb = _reach_advance(cbu2, dict(pbump, top_clear=0.1))
    _a, s2, _ = cbu2.step(tb, dict(pbump, frame_id=fb, slam_ms=200.0, top_clear=0.1), False, status="OK")
    no_bump_ok = (s2 == "SETTLE")
    bump_all_ok = bump_ok and ceiling_abort_ok and no_bump_ok
    ok = ok and bump_all_ok
    print(f"[self-test] {'PASS' if bump_all_ok else 'FAIL'}  gentle bump-up "
          f"(pulse-up={saw_bump and saw_up}, raised+capped={raised and capped and saw_replan}, "
          f"ceiling-aborts={ceiling_abort_ok}, low-top->no-bump={no_bump_ok})")

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

    # ---- F7 ram guard: forward-commanded but SLAM pos not advancing for ram_stall_s -> stop the leg ----
    cr = ExploreController(cfg, no_takeoff=True)
    cr.ram_stall_s, cr.ram_progress_eps = 0.5, 0.15
    pram = {"plan_valid": True, "done": False, "goal": [9.0, 0.0], "pos": [0.0, 0.0], "bearing_err": 0.0,
            "forward_clearance_dist": 9.0, "pos_y": 0.0}   # clear path, but pos never changes (collider)
    tr, fr, ram_stopped = 0.0, 0, False
    for _ in range(160):
        _a, s, ev = cr.step(tr, dict(pram, frame_id=fr, slam_ms=200.0), False, status="OK"); tr += 0.05; fr += 1
        if ev and "ram guard" in ev:
            ram_stopped = True; break
    ram_ok = ram_stopped and cr.state == "SETTLE"
    # a leg that IS advancing (pos closing) must NOT trip the ram guard
    cr2 = ExploreController(cfg, no_takeoff=True)
    cr2.ram_stall_s = 0.5
    tr, fr, tripped = 0.0, 0, False
    for i in range(40):
        pos = [min(9.0, i * 0.3), 0.0]                   # steadily advancing toward the goal
        _a, s, ev = cr2.step(tr, dict(pram, pos=pos, frame_id=fr, slam_ms=200.0), False, status="OK")
        tr += 0.05; fr += 1
        if ev and "ram guard" in ev:
            tripped = True; break
    ram_ok = ram_ok and not tripped
    ok = ok and ram_ok
    print(f"[self-test] {'PASS' if ram_ok else 'FAIL'}  ram guard "
          f"(stuck-collider->stop={ram_stopped}, advancing->no-trip={not tripped})")

    # ---- 2-bump blacklist plumbing: _detector_command + the kinematic bump latch ----
    dc_ok = (_detector_command({"reverse": 0.4}) == CMD_BACK
             and _detector_command({"trigger": 0.1}) == CMD_FWD
             and _detector_command({"trigger": 0.1, "joy_vertical": -1}) == CMD_FWD   # altlock ADVANCE -> FWD
             and _detector_command({"joy_vertical": -1}) == CMD_UP
             and _detector_command({"joy_vertical": 1}) is None                       # DESCEND (down) -> idle
             and _detector_command({"yaw": 1.0}) is None and _detector_command({}) is None)
    ok = ok and dc_ok
    print(f"[self-test] {'PASS' if dc_ok else 'FAIL'}  _detector_command maps reverse->BACK / fwd->FWD / up->UP")

    # The ram-guard stop above (cr) fired exactly ONE bump pulse toward the leg goal and disarmed the latch.
    latch_armed_once = (cr._bump_pulse == [9.0, 0.0] and cr._bump_armed is False
                        and cr._last_bump_anchor is not None)
    cr.take_bump_pulse()                                     # publish consumes it
    cr._register_bump({"pos": [0.0, 0.0]})                   # a stutter while still disarmed -> NO new pulse
    stutter_ok = cr._bump_pulse is None
    cr.rearm_bump_if_disengaged({}, {"pos": [0.0, 0.0]})     # same spot, no reverse -> stays disarmed
    still_disarmed = cr._bump_armed is False
    cr.rearm_bump_if_disengaged({}, {"pos": [cr.goal_reach_dist + 0.2, 0.0]})   # moved away -> re-arm
    rearmed_by_move = cr._bump_armed is True
    cr._register_bump({"pos": [9.0, 0.0]})                   # a genuine 2nd encounter -> a fresh pulse
    second_pulse_ok = cr._bump_pulse == [9.0, 0.0]
    crb = ExploreController(cfg, no_takeoff=True); crb.leg_goal = [5.0, 0.0]
    crb._register_bump({"pos": [0.0, 0.0]}); popped = crb.take_bump_pulse()
    crb.rearm_bump_if_disengaged({"reverse": 0.3}, {"pos": [0.0, 0.0]})   # reverse cmd re-arms at 0 displacement
    rearmed_by_reverse = crb._bump_armed is True and popped == [5.0, 0.0]
    latch_ok = (latch_armed_once and stutter_ok and still_disarmed and rearmed_by_move
                and second_pulse_ok and rearmed_by_reverse)
    ok = ok and latch_ok
    print(f"[self-test] {'PASS' if latch_ok else 'FAIL'}  2-bump latch "
          f"(one pulse/contact, stutter-suppressed, re-arm on move|reverse)")

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
    args = parser.parse_args()
    cfg = load_config(args.config)

    if args.self_test:
        raise SystemExit(0 if run_self_test(cfg) else 1)
    if args.dry_run:
        run_dry(cfg, log=args.log)
    elif args.explore:
        run_explore(cfg, log=args.log, no_takeoff=args.no_takeoff)
    else:
        run_mission(cfg, mission_path=args.mission, max_contact_s=args.max_contact_s, log=args.log)


if __name__ == "__main__":
    main()
