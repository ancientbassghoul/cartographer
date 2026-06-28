"""io_bridge.py — Process P1: sim IO + frame publishing (NO GPU).

Refactor of ../XLAB/Sample_Drone_Interface.py. Responsibilities:
  * 60 Hz TCP control server (io_bridge is the SERVER; Unity connects as client)
  * keyboard hook -> manual flight (mapping IDENTICAL to the sample; behaviour unchanged)
  * NDI video capture -> live OpenCV display
  * publish a downscaled, sub-sampled frame stream to the perception worker (P2)
    over the ZeroMQ frame bus, plus a lightweight status stream on the state bus.

What was DELIBERATELY dropped vs. the sample: the YOLO 'o'-key autopilot and its
`detect_target` try-except. That was GPU work and a silent except-and-continue —
both forbidden here. Object detection returns later in object_worker.py (P3),
triggered by the 'g' hotkey, which this bridge surfaces as a state-bus event.

NO SILENT FALLBACKS (per CLAUDE.md): NDI init, source discovery, and the TCP bind
fail-fast with explicit errors instead of degrading to a control-only / no-video
mode. The keyboard hook is installed without a swallowing try-except so a missing
privilege surfaces immediately.
"""

import argparse
import json
import os
import socket
import struct
import threading
import time
from datetime import datetime

import cv2
import numpy as np
import yaml
import NDIlib as ndi
import keyboard

import frame_bus
from diag_log import DiagLog, NullLog

REPO = os.path.dirname(os.path.abspath(__file__))


def clamp(minimum, x, maximum):
    return max(minimum, min(x, maximum))


def load_config(path=None):
    path = path or os.path.join(REPO, "config.yaml")
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


# ==============================================================================
# Control state + threads (manual flight — behaviour identical to the sample)
# ==============================================================================
class DroneControl:
    """Owns control_state and the three 60 Hz control threads + keyboard hook.

    The key mapping and ramping math are copied verbatim from the sample so manual
    flight feels exactly the same. Additions: an edge-triggered 'g' object-detect
    request, and a `debug_keys` toggle (the sample's per-key print, off by default).
    """

    # Manual flight keys: pressing ANY of these while autonomy is active is an immediate ABORT
    # back to manual control (NO SILENT FALLBACKS — the operator override is loud and instant).
    MANUAL_FLIGHT_KEYS = {
        "1", "2", "b", "c", "w", "s", "e", "f", "a", "d", "k",
        "left", "right", "up", "down", "p",
    }
    # Fields the autopilot is permitted to drive over the control bus. Anything else (the static
    # boxes, the wire-only 'autopilot' flag) stays owned by io_bridge. btnCdown is included so the
    # autopilot can pulse the attitude/camera reset ('c') before a forward push (clean yaw/pitch so a
    # wall reads as a clean expansion-collapse) — see flight_playbook.json reset_attitude_before_forward.
    AUTONOMY_FIELDS = (
        "btnARMdown", "btnCdown", "trigger", "reverse",
        "joy_vertical", "joy_horizontal", "yaw", "pitch",
    )

    def __init__(self, host, port, detect_key="g", capture_key="space", debug_keys=False,
                 autonomy_enable_key="m", cmd_timeout_s=0.5, key_log=None):
        self.host = host
        self.port = port
        self.detect_key = detect_key
        self.capture_key = capture_key
        self.debug_keys = debug_keys
        self.autonomy_enable_key = autonomy_enable_key
        self.cmd_timeout_s = float(cmd_timeout_s)

        # --- frame-synced keystroke logging (--log-keys; learning material for the optical-flow
        # detector). The main loop stamps current_rec_frame each iteration; _on_key_event writes one
        # CSV row per REAL edge (auto-repeat 'down's suppressed) so the key log aligns 1:1 with the
        # recorded flight_<ts>.mp4 (rec_frame is the video frame index). NO behaviour change — log only.
        self.key_log = key_log or NullLog()
        self.current_rec_frame = None      # set by the main loop; blank in the log before recording starts
        self._keys_down = set()            # keys currently physically held, to drop keyboard auto-repeats

        self.control_state = {
            "btnAdown": False, "btnBdown": False, "btnCdown": False,
            "btnARMdown": False,
            "trigger": 0.0, "trigger_down": False,
            "reverse": 0.0, "reverse_down": False,
            "joy_vertical": 0, "joy_horizontal": 0,
            "yaw": 0.0, "pitch": 0.0,
            "thumb_down": False, "joy_click": False,
            "joy_up": False, "joy_down": False,
            "joy_left": False, "joy_right": False,
            "arrow_left": False, "arrow_right": False,
            "arrow_up": False, "arrow_down": False,
            "autopilot": False,  # kept in the wire payload for sim compatibility; never set True here
        }
        self.time_from_unity = 0.0
        self.static_boxes = [
            {"x": -100, "y": -100, "width": 100, "height": 100, "id": "box1"},
            {"x": 100, "y": -100, "width": 100, "height": 100, "id": "box2"},
            {"x": 200, "y": -200, "width": 120, "height": 140, "id": "box3"},
        ]

        self._server_socket = None
        self._conn = None
        self._running = threading.Event()
        self._running.set()
        self._detect_held = False          # rising-edge tracker for the 'g' key
        self._detect_requests = 0          # incremented on each 'g' press; main loop drains it
        self._capture_held = False         # rising-edge tracker for the capture key (space)
        self._capture_requests = 0         # incremented on each capture press; main loop drains it

        # --- Autonomy (Phase 2): a VISIBLE flag, NO SILENT FALLBACKS. autopilot (P5) PUBs a desired
        # control vector on the control bus; io_bridge applies it into control_state ONLY while
        # autonomy_active. Any manual flight key aborts instantly; a stale command (older than
        # cmd_timeout_s) zeroes the autonomous controls so the drone can't run away on a dropped link.
        self.autonomy_active = False       # toggled by the enable key; cleared by any manual key / abort
        self._enable_held = False          # rising-edge tracker for the enable key
        self._autonomy_cmd = None          # latest control vector from the bus (dict) or None
        self._autonomy_cmd_mono = 0.0      # mono time the latest command arrived
        self._autonomy_cmd_seq = -1        # last applied command seq (for logging gaps)
        self.autonomy_status = "MANUAL"    # HUD/telemetry string: MANUAL | AUTO | AUTO(STALE)
        self._last_auto_log = None         # on-change diagnostic of the APPLIED autonomy vector

    # -- TCP server -----------------------------------------------------------
    def start(self):
        """Bind, wait for Unity, and launch the control threads. Fail-fast on bind."""
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind((self.host, self.port))
        except OSError as e:
            raise RuntimeError(
                f"Could not bind control server on {self.host}:{self.port} ({e}). "
                f"Is another io_bridge / Sample_Drone_Interface already running?"
            ) from e
        s.listen(1)
        self._server_socket = s
        print(f"[io_bridge] Control server up. Waiting for Unity on {self.host}:{self.port} ...")
        conn, addr = s.accept()
        self._conn = conn
        print(f"[io_bridge] Unity connected from {addr}")

        threading.Thread(target=self._listen_to_unity, daemon=True).start()
        threading.Thread(target=self._send_to_unity, daemon=True).start()
        threading.Thread(target=self._update_controls, daemon=True).start()

        # No swallowing try-except: if the hook can't install (e.g. needs admin),
        # we want the failure to be loud, not a silently dead control surface.
        keyboard.hook(self._on_key_event)
        print("[io_bridge] Keyboard hook active. Manual flight: WASD/EF/arrows, 1=arm, b=land, c=reset cam.")
        print(f"[io_bridge] '{self.detect_key}' = request object detection (forwarded to object_worker).")
        print(f"[io_bridge] '{self.capture_key}' = save the current FULL-RES frame to the capture dir.")

    def _listen_to_unity(self):
        conn = self._conn
        while self._running.is_set():
            raw_msglen = conn.recv(4)
            if not raw_msglen:
                break
            message_length = struct.unpack(">I", raw_msglen)[0]
            data = b""
            while len(data) < message_length:
                packet = conn.recv(message_length - len(data))
                if not packet:
                    break
                data += packet
            try:
                message = json.loads(data)
            except json.JSONDecodeError:
                continue  # a malformed packet is skipped; the stream is not torn down
            self.time_from_unity = message.get("time", 0)

    def _send_to_unity(self):
        conn = self._conn
        cs = self.control_state
        while self._running.is_set():
            response = {
                "num_of_boxes": len(self.static_boxes), "data": self.static_boxes,
                "time": self.time_from_unity,
                "btnAdown": cs["btnAdown"], "btnBdown": cs["btnBdown"],
                "btnCdown": cs["btnCdown"], "btnARMdown": cs["btnARMdown"],
                "trigger": cs["trigger"], "triggerDown": cs["trigger_down"],
                "reverse": cs["reverse"], "reverseDown": cs["reverse_down"],
                "joy_vertical": cs["joy_vertical"], "joy_horizontal": cs["joy_horizontal"],
                "yaw": cs["yaw"], "pitch": cs["pitch"],
                "thumbDown": cs["thumb_down"], "joyClick": cs["joy_click"],
            }
            body = json.dumps(response).encode("utf-8")
            try:
                conn.send(len(body).to_bytes(4, byteorder="big"))
                conn.send(body)
            except (ConnectionResetError, BrokenPipeError):
                print("[io_bridge] Unity connection lost (sender). Stopping.")
                self._running.clear()
                break
            time.sleep(1 / 60)  # 60 Hz

    def _update_controls(self):
        cs = self.control_state
        while self._running.is_set():
            # Smooth Trigger (gas)
            if cs["trigger"] > 0 and not cs["trigger_down"]:
                cs["trigger"] = clamp(0, cs["trigger"] - 0.1, 1)
            if cs["trigger"] < 1 and cs["trigger_down"]:
                cs["trigger"] = clamp(0, cs["trigger"] + 0.05, 1)
            # Smooth Reverse
            if cs["reverse"] > 0 and not cs["reverse_down"]:
                cs["reverse"] = clamp(0, cs["reverse"] - 0.1, 1)
            if cs["reverse"] < 1 and cs["reverse_down"]:
                cs["reverse"] = clamp(0, cs["reverse"] + 0.05, 1)
            # Joystick (altitude / strafe)
            if cs["joy_up"]:
                cs["joy_vertical"] = -1
            elif cs["joy_down"]:
                cs["joy_vertical"] = 1
            else:
                cs["joy_vertical"] = 0
            if cs["joy_left"]:
                cs["joy_horizontal"] = -1
            elif cs["joy_right"]:
                cs["joy_horizontal"] = 1
            else:
                cs["joy_horizontal"] = 0
            # Yaw / pitch with smoothing
            if cs["arrow_right"]:
                cs["yaw"] = clamp(-1, cs["yaw"] + 0.05, 1)
            elif cs["arrow_left"]:
                cs["yaw"] = clamp(-1, cs["yaw"] - 0.05, 1)
            if cs["arrow_up"]:
                cs["pitch"] = clamp(-1, cs["pitch"] - 0.05, 1)
            elif cs["arrow_down"]:
                cs["pitch"] = clamp(-1, cs["pitch"] + 0.05, 1)
            if cs["btnCdown"]:
                cs["pitch"] = 0
                cs["yaw"] = 0
            self._apply_autonomy_overlay()
            time.sleep(1 / 60)

    def _apply_autonomy_overlay(self, now=None):
        """While autonomy is active, the autopilot's command vector OVERWRITES the manual-derived
        fields. A command older than cmd_timeout_s is treated as lost and the autonomous controls are
        ZEROED (fail-safe — a dropped link can't run the drone away). Manual keys can't co-drive:
        pressing one aborts autonomy entirely (see _on_key_event), so there is no silent blend.
        Updates self.autonomy_status (MANUAL | AUTO | AUTO(STALE)) for the HUD + telemetry."""
        cs = self.control_state
        if not self.autonomy_active:
            self.autonomy_status = "MANUAL"
            return
        now = time.monotonic() if now is None else now
        cmd = self._autonomy_cmd
        fresh = cmd is not None and (now - self._autonomy_cmd_mono) <= self.cmd_timeout_s
        if fresh:
            for k in self.AUTONOMY_FIELDS:
                if k in cmd:
                    cs[k] = cmd[k]
            self.autonomy_status = "AUTO"
        else:
            self._neutralize_autonomy()   # no fresh command -> don't run away
            self.autonomy_status = "AUTO(STALE)"

        # Diagnostic: print the APPLIED autonomy vector whenever it changes, so we can confirm what
        # actually reaches Unity (e.g. that btnARMdown is being applied during the arm recipe).
        snap = (self.autonomy_status, cs["btnARMdown"], cs["btnCdown"], cs["joy_vertical"],
                round(float(cs["trigger"]), 2), round(float(cs["reverse"]), 2),
                round(float(cs["yaw"]), 2), (cmd or {}).get("seq"), (cmd or {}).get("state"))
        if snap != self._last_auto_log:
            self._last_auto_log = snap
            print(f"[io_bridge][AUTO] status={self.autonomy_status} state={(cmd or {}).get('state')} "
                  f"seq={(cmd or {}).get('seq')} btnARM={cs['btnARMdown']} btnC={cs['btnCdown']} "
                  f"joyV={cs['joy_vertical']} trig={cs['trigger']:.2f} rev={cs['reverse']:.2f} "
                  f"yaw={cs['yaw']:.2f}", flush=True)

    def _on_key_event(self, event):
        cs = self.control_state
        is_down = event.event_type == "down"
        key = event.name
        if self.debug_keys:
            print(f"[io_bridge][key] {key} | {event.event_type}")

        # --- frame-synced edge logging (--log-keys) — suppress keyboard auto-repeat 'down's so only
        # real press/release edges are recorded, stamped with the current recording-relative frame. ---
        if is_down and key not in self._keys_down:
            self._keys_down.add(key)
            self.key_log.row(rec_frame=self.current_rec_frame, mono_ts=time.monotonic(),
                             key=key, action="down")
        elif not is_down and key in self._keys_down:
            self._keys_down.discard(key)
            self.key_log.row(rec_frame=self.current_rec_frame, mono_ts=time.monotonic(),
                             key=key, action="up")

        # --- Autonomy enable (toggle) + manual-key ABORT (checked BEFORE manual mapping) ---
        if key == self.autonomy_enable_key:
            if is_down and not self._enable_held:
                self.autonomy_active = not self.autonomy_active
                if self.autonomy_active:
                    print("[io_bridge] AUTONOMY ENABLED — autopilot may drive the drone. "
                          "Press any flight key (or the enable key again) to abort.")
                else:
                    self._neutralize_autonomy()
                    print("[io_bridge] AUTONOMY DISABLED — back to manual.")
            self._enable_held = is_down
            return  # the enable key itself is not a flight key
        if is_down and self.autonomy_active and key in self.MANUAL_FLIGHT_KEYS:
            self.autonomy_active = False
            self._neutralize_autonomy()
            print(f"[io_bridge] AUTONOMY ABORTED by manual key '{key}' — manual control restored.")
            # fall through so this same press also takes effect as a normal manual input

        if key == "2": cs["btnAdown"] = is_down
        if key == "b": cs["btnBdown"] = is_down
        if key == "c": cs["btnCdown"] = is_down
        if key == "1": cs["btnARMdown"] = is_down
        if key == "w": cs["trigger_down"] = is_down
        if key == "s": cs["reverse_down"] = is_down
        if key == "e": cs["joy_up"] = is_down
        if key == "f": cs["joy_down"] = is_down
        if key == "a": cs["joy_left"] = is_down
        if key == "d": cs["joy_right"] = is_down
        if key == "k": cs["joy_click"] = is_down
        if key == "left": cs["arrow_left"] = is_down
        if key == "right": cs["arrow_right"] = is_down
        if key == "up": cs["arrow_up"] = is_down
        if key == "down": cs["arrow_down"] = is_down
        if key == "p": cs["thumb_down"] = is_down

        # Object-detect hotkey: rising edge only (keyboard repeats 'down' while held).
        if key == self.detect_key:
            if is_down and not self._detect_held:
                self._detect_requests += 1
                print(f"[io_bridge] object-detect requested (#{self._detect_requests})")
            self._detect_held = is_down

        # Frame-capture hotkey: rising edge only. The main loop saves the current
        # full-res NDI frame (drone camera) so the press grabs exactly what's on screen.
        if key == self.capture_key:
            if is_down and not self._capture_held:
                self._capture_requests += 1
            self._capture_held = is_down

    def set_autonomy_command(self, cmd: dict):
        """Store the latest control vector from the control bus (applied only while autonomy_active)."""
        self._autonomy_cmd = cmd
        self._autonomy_cmd_mono = time.monotonic()
        seq = cmd.get("seq")
        if seq is not None and self._autonomy_cmd_seq >= 0 and seq != self._autonomy_cmd_seq + 1:
            print(f"[io_bridge] autonomy command gap: seq {self._autonomy_cmd_seq} -> {seq}")
        if seq is not None:
            self._autonomy_cmd_seq = seq

    def _neutralize_autonomy(self):
        """Zero every field the autopilot can drive, immediately (used on abort/disable)."""
        cs = self.control_state
        cs["btnARMdown"] = False
        cs["btnCdown"] = False
        cs["trigger"] = 0.0
        cs["reverse"] = 0.0
        cs["joy_vertical"] = 0
        cs["joy_horizontal"] = 0
        cs["yaw"] = 0.0
        cs["pitch"] = 0.0

    def drain_detect_requests(self):
        """Return how many new 'g' presses occurred since the last call (and reset)."""
        n = self._detect_requests
        self._detect_requests = 0
        return n

    def drain_capture_requests(self):
        """Return how many new capture-key presses occurred since the last call (and reset)."""
        n = self._capture_requests
        self._capture_requests = 0
        return n

    def control_snapshot(self):
        """A small copy of the fields perception needs (forward-commanded etc.)."""
        cs = self.control_state
        return {
            "trigger": round(float(cs["trigger"]), 3),
            "reverse": round(float(cs["reverse"]), 3),
            "joy_vertical": cs["joy_vertical"],
            "joy_horizontal": cs["joy_horizontal"],
            "yaw": round(float(cs["yaw"]), 3),
            "pitch": round(float(cs["pitch"]), 3),
            "autonomy": self.autonomy_status,   # MANUAL | AUTO | AUTO(STALE) — surfaced on HUD + state bus
        }

    def stop(self):
        self._running.clear()
        try:
            keyboard.unhook_all()
        except Exception:
            pass
        self.key_log.close()
        if self._conn:
            try:
                self._conn.close()
            except OSError:
                pass
        if self._server_socket:
            try:
                self._server_socket.close()
            except OSError:
                pass


# ==============================================================================
# NDI capture
# ==============================================================================
def open_ndi(source_filter="Unity", discover_iters=10, wait_ms=500):
    """Initialize NDI and connect to the drone camera. Fail-fast at each step."""
    if not ndi.initialize():
        raise RuntimeError("NDI failed to initialize (ndi.initialize() returned False).")

    finder = ndi.find_create_v2()
    if finder is None:
        raise RuntimeError("NDI find_create_v2() returned None — cannot discover sources.")

    sources = []
    print("[io_bridge] Looking for NDI sources (drone camera) ...")
    for _ in range(discover_iters):
        ndi.find_wait_for_sources(finder, wait_ms)
        sources = ndi.find_get_current_sources(finder)
        if len(sources) > 0:
            break

    if not sources:
        ndi.find_destroy(finder)
        raise RuntimeError(
            "No NDI sources found. Start Xlab.exe (Unity) before io_bridge — "
            "video is required; there is no control-only fallback."
        )

    selected = sources[0]
    for s in sources:
        print(f"[io_bridge] Found NDI source: {s.ndi_name}")
        if source_filter and source_filter in s.ndi_name:
            selected = s
    print(f"[io_bridge] Connecting to video: {selected.ndi_name}")

    recv_create = ndi.RecvCreateV3()
    recv_create.color_format = ndi.RECV_COLOR_FORMAT_BGRX_BGRA
    recv = ndi.recv_create_v3(recv_create)
    if recv is None:
        ndi.find_destroy(finder)
        raise RuntimeError("NDI recv_create_v3() returned None.")
    ndi.recv_connect(recv, selected)
    ndi.find_destroy(finder)
    return recv


# ==============================================================================
# Main loop
# ==============================================================================
def main():
    parser = argparse.ArgumentParser(description="Cartographer io_bridge (P1)")
    parser.add_argument("--config", default=None, help="path to config.yaml")
    parser.add_argument("--debug-keys", action="store_true", help="print every key event")
    parser.add_argument("--no-display", action="store_true", help="headless: skip the OpenCV window")
    parser.add_argument("--log-keys", action="store_true",
                        help="write every keyboard edge (rec_frame,mono_ts,key,action) to "
                             "OUTPUT/diag/<ts>_keys.csv, frame-synced to the recorded flight video")
    args = parser.parse_args()

    cfg = load_config(args.config)
    host = cfg["network"]["unity_server_ip"]
    port = cfg["network"]["unity_control_port"]
    ndi_filter = cfg["network"].get("ndi_source_name", "Unity")
    frame_port = cfg["network"]["frame_bus_port"]
    state_port = cfg["network"]["state_bus_port"]
    hires_port = cfg["network"].get("frame_bus_hires_port")
    proc_w = cfg["perception"]["processing_width"]
    proc_h = cfg["perception"]["processing_height"]
    object_frame_h = int(cfg["perception"].get("object_frame_height", 720))
    target_fps = cfg["perception"]["target_processing_fps"]
    detect_key = cfg["models"]["qwen_vl"].get("object_trigger_key", "g")
    capture_key = cfg["perception"].get("capture_key", "space")
    capture_dir = os.path.join(REPO, cfg["perception"].get("capture_dir", "test_assets/captures"))
    publish_interval = 1.0 / float(target_fps)
    # Autonomy (Phase 2): control-bus port + general fail-safe params. NO room/flight-specific values.
    autonomy_cfg = cfg.get("autonomy", {}) or {}
    autonomy_port = cfg["network"].get("autonomy_control_port")
    autonomy_enable_key = autonomy_cfg.get("enable_key", "m")
    cmd_timeout_s = float(autonomy_cfg.get("cmd_timeout_s", 0.5))

    # --- bring up the bus (publishers bind; fail-fast if a port is taken) ---
    frame_pub = frame_bus.FramePublisher(frame_port)
    state_pub = frame_bus.StatePublisher(state_port)
    # Second, higher-res frame stream for the object worker (Qwen grounds a ~60px target far more
    # reliably with full pixel fidelity; detection runs ~0.5 Hz so loopback bandwidth is a non-issue).
    hires_pub = frame_bus.FramePublisher(hires_port) if hires_port else None
    print(f"[io_bridge] frame bus PUB on :{frame_port}  | state bus PUB on :{state_port}")
    print(f"[io_bridge] perception stream: {proc_w}x{proc_h} @ ~{target_fps} fps (mono-clock gated)")
    if hires_pub:
        print(f"[io_bridge] hi-res object stream: ~{object_frame_h}p PUB on :{hires_port}")

    # --- optional frame-synced keystroke log (learning material for the optical-flow detector) ---
    key_log = None
    if args.log_keys:
        key_log = DiagLog("keys", ["rec_frame", "mono_ts", "key", "action"])

    # --- control server + NDI (both fail-fast) ---
    control = DroneControl(host, port, detect_key=detect_key, capture_key=capture_key,
                           debug_keys=args.debug_keys, autonomy_enable_key=autonomy_enable_key,
                           cmd_timeout_s=cmd_timeout_s, key_log=key_log)
    control.start()
    recv = open_ndi(ndi_filter)

    # --- autonomy control bus: SUB the autopilot's desired control vector (applied only when enabled) ---
    autonomy_sub = None
    if autonomy_port:
        autonomy_sub = frame_bus.StateSubscriber(autonomy_port, topics=[frame_bus.TOPIC_CONTROL])

        def _autonomy_listener():
            while control._running.is_set():
                msg = autonomy_sub.recv(timeout_ms=500)
                if msg is None:
                    continue
                _topic, cmd = msg
                control.set_autonomy_command(cmd)

        threading.Thread(target=_autonomy_listener, daemon=True).start()
        print(f"[io_bridge] autonomy control bus SUB on :{autonomy_port} "
              f"(enable key '{autonomy_enable_key}', cmd_timeout {cmd_timeout_s}s). Autonomy starts OFF.")

    window_name = "Cartographer — io_bridge (NDI)"
    output_dir = os.path.join(REPO, "OUTPUT")
    os.makedirs(output_dir, exist_ok=True)
    os.makedirs(capture_dir, exist_ok=True)
    print(f"[io_bridge] full-res captures -> {capture_dir}")
    recording = False
    prev_recording = False       # detect the off->on edge to (re)start the recording frame counter
    rec_count = 0                # recording-relative frame index: 0 at each recording start, +1 per written frame
    video_writer = None
    captures_saved = 0           # session count, shown on the HUD
    last_capture_mono = 0.0      # drives a brief on-screen "SAVED" flash

    frame_id = 0
    published = 0
    last_pub_mono = 0.0          # 0 => the very first frame always publishes
    last_status_mono = time.monotonic()
    # fps bookkeeping
    cap_count = 0
    cap_window_start = time.monotonic()
    cap_fps = 0.0

    print(f"\n[io_bridge] === READY === fly manually. '{capture_key}'=capture full-res frame (global). "
          f"Video keys (window focused): r=record, q=quit.\n")
    try:
        while control._running.is_set():
            t, v, a, _ = ndi.recv_capture_v2(recv, 1000)

            if t == ndi.FRAME_TYPE_VIDEO:
                bgra = np.copy(v.data)
                ndi.recv_free_video_v2(recv, v)
                bgr = cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)

                now = time.monotonic()
                frame_id += 1
                cap_count += 1
                if now - cap_window_start >= 1.0:
                    cap_fps = cap_count / (now - cap_window_start)
                    cap_count = 0
                    cap_window_start = now

                # --- recording-relative frame counter (synced to the .mp4 for log correlation) ---
                # Resets to 0 on every record start; `rec_frame` is this NDI frame's index in the
                # recorded video (None when not recording). Stamped into meta so downstream (autopilot)
                # can prefix each log line with the exact frame to scrub to. Incremented at write time.
                if recording and not prev_recording:
                    rec_count = 0
                prev_recording = recording
                rec_frame = rec_count if recording else None
                control.current_rec_frame = rec_frame   # stamp for the frame-synced key log

                # --- perception stream: mono-clock gated sub-sample + downscale ---
                if now - last_pub_mono >= publish_interval:
                    small = cv2.resize(bgr, (proc_w, proc_h), interpolation=cv2.INTER_AREA)
                    meta = {
                        "frame_id": frame_id,
                        "mono_ts": now,
                        "sim_time": self_time(control),
                        "controls": control.control_snapshot(),
                        "rec_frame": rec_frame,
                    }
                    frame_pub.publish(small, meta)
                    # Same frame, same meta/frame_id, at higher resolution for the object worker
                    # (so its detection frame_id still matches perception's pose history). Downscale
                    # only if the source is taller than the target; never upscale.
                    if hires_pub is not None:
                        sh, sw = bgr.shape[:2]
                        if sh > object_frame_h:
                            ow = int(round(sw * object_frame_h / sh))
                            hires = cv2.resize(bgr, (ow, object_frame_h), interpolation=cv2.INTER_AREA)
                        else:
                            hires = bgr
                        hires_pub.publish(hires, meta)
                    published += 1
                    last_pub_mono = now

                # --- object-detect requests -> state bus (object_worker consumes later) ---
                n_req = control.drain_detect_requests()
                if n_req:
                    state_pub.publish(
                        frame_bus.TOPIC_DETECTION,
                        {"event": "detect_request", "count": n_req,
                         "frame_id": frame_id, "mono_ts": now},
                    )

                # --- frame-capture requests -> save the CURRENT full-res NDI frame ---
                # `bgr` is the native NDI frame (no downscale), so a press grabs full resolution.
                # One save per loop iteration even if the key was hit multiple times (same frame).
                if control.drain_capture_requests():
                    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                    fname = os.path.join(capture_dir, f"capture_{ts}_f{frame_id}.png")
                    if cv2.imwrite(fname, bgr):
                        captures_saved += 1
                        last_capture_mono = now
                        h, w = bgr.shape[:2]
                        print(f"[io_bridge] captured frame -> {fname}  ({w}x{h}, #{captures_saved})")
                    else:
                        # NO SILENT FALLBACKS: a failed write is surfaced, not swallowed.
                        print(f"[io_bridge] WARNING: cv2.imwrite failed for {fname}")

                # --- periodic status heartbeat (~2 Hz) ---
                if now - last_status_mono >= 0.5:
                    window = now - last_status_mono
                    state_pub.publish(
                        frame_bus.TOPIC_STATUS,
                        {"capture_fps": round(cap_fps, 1),
                         "publish_fps": round(published / window, 1),
                         "sim_time": self_time(control),
                         "controls": control.control_snapshot()},
                    )
                    published = 0
                    last_status_mono = now

                # --- recording ---
                if recording:
                    if video_writer is None:
                        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                        fname = os.path.join(output_dir, f"flight_{ts}.mp4")
                        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                        h, w = bgr.shape[:2]
                        # Tag the file with the MEASURED capture rate (NDI is ~58 fps, not 30) so the
                        # mp4 plays at real speed and rec_frame maps to real time. (Recipes should still
                        # use mono_ts, not frame/fps — but this keeps the artifact honest.)
                        rec_fps = round(cap_fps, 2) if cap_fps >= 1.0 else 60.0
                        video_writer = cv2.VideoWriter(fname, fourcc, rec_fps, (w, h))
                        print(f"[io_bridge] recording -> {fname} @ {rec_fps} fps (frame counter starts at 0)")
                    video_writer.write(bgr)
                    rec_count += 1   # this written frame's index was `rec_frame`; advance for the next
                elif video_writer is not None:
                    video_writer.release()
                    video_writer = None
                    print("[io_bridge] recording stopped.")

                # --- live display ---
                if not args.no_display:
                    disp = cv2.resize(bgr, (0, 0), fx=0.5, fy=0.5)
                    cv2.putText(disp, f"cap {cap_fps:4.1f}fps  sim_t {self_time(control):.1f}  shots {captures_saved}",
                                (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)
                    if recording:
                        cv2.circle(disp, (disp.shape[1] - 20, 20), 8, (0, 0, 255), -1)
                    if now - last_capture_mono < 0.6:   # brief confirmation flash
                        cv2.putText(disp, "SAVED", (disp.shape[1] // 2 - 40, disp.shape[0] // 2),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
                    # Autonomy state banner (NO SILENT FALLBACKS: the operator always sees who's flying).
                    a_status = control.autonomy_status
                    if a_status != "MANUAL":
                        a_color = (0, 0, 255) if a_status == "AUTO(STALE)" else (0, 165, 255)
                        cv2.putText(disp, f"AUTONOMY: {a_status}", (8, 44),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, a_color, 2)
                    cv2.imshow(window_name, disp)

            elif t == ndi.FRAME_TYPE_AUDIO:
                ndi.recv_free_audio_v2(recv, a)

            if not args.no_display:
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                elif key == ord("r"):
                    recording = not recording

    except KeyboardInterrupt:
        pass
    finally:
        print("[io_bridge] shutting down ...")
        if video_writer:
            video_writer.release()
        control.stop()
        frame_pub.close()
        if hires_pub is not None:
            hires_pub.close()
        if autonomy_sub is not None:
            autonomy_sub.close()
        state_pub.close()
        if recv:
            ndi.recv_destroy(recv)
        ndi.destroy()
        if not args.no_display:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass


def self_time(control):
    """The single piece of telemetry Unity returns: its clock."""
    return float(control.time_from_unity)


if __name__ == "__main__":
    main()
