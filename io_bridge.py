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
triggered by the 'h' hotkey (default; configurable), which this bridge surfaces as
a state-bus event.

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
from flight_playbook import FlightPlaybook, RecipePlayer

REPO = os.path.dirname(os.path.abspath(__file__))


def clamp(minimum, x, maximum):
    return max(minimum, min(x, maximum))


def _ramp(cur, target, up, down):
    """Move `cur` one 60 Hz step toward `target`: rise by `up`, fall by `down`, never overshooting.
    This is the sim's own manual-stick smoothing (trigger/reverse up=0.05 / down=0.1; yaw/pitch 0.05 /
    0.05) applied to AUTONOMOUS commands too (session 18), so scripted flight eases in and out exactly
    like a hand-flown stick instead of hard-stepping. `target` is always already in range, so no clamp
    is needed here."""
    if cur < target:
        return min(target, cur + up)
    if cur > target:
        return max(target, cur - down)
    return cur


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
    # 't'/'g' (the TRIM UP / TRIM DOWN macro keys) are included here too: a real flight input must
    # always win, whether that's aborting autonomy or cancelling an in-progress trim macro.
    MANUAL_FLIGHT_KEYS = {
        "1", "2", "b", "c", "w", "s", "e", "f", "a", "d", "k",
        "left", "right", "up", "down", "p", "t", "g",
    }
    TRIM_KEYS = {"t": "trim_up", "g": "trim_down"}   # key -> flight_playbook.json recipe name
    # Fields the autopilot is permitted to drive over the control bus. Anything else (the static
    # boxes, the wire-only 'autopilot' flag) stays owned by io_bridge. btnCdown is included so the
    # autopilot can pulse the attitude/camera reset ('c') before a forward push (clean yaw/pitch so a
    # wall reads as a clean expansion-collapse) — see flight_playbook.json reset_attitude_before_forward.
    # NB: trigger_down/reverse_down are the BOOLEAN gas gates — Unity gates REAL thrust on these,
    # not the analog trigger/reverse (session 17). The autopilot now derives them centrally in
    # _full_vector and drives them over this whitelist, so autonomous flight finally has real thrust.
    AUTONOMY_FIELDS = (
        "btnARMdown", "btnCdown", "trigger", "reverse", "trigger_down", "reverse_down",
        "joy_vertical", "joy_horizontal", "yaw", "pitch",
    )

    def __init__(self, host, port, detect_key="h", capture_key="space", debug_keys=False,
                 autonomy_enable_key="m", cmd_timeout_s=0.5, key_log=None, cmd_log=None):
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

        # --- outgoing-packet logging (--log-commands). One row per 60 Hz send of the ACTUAL packet that
        # reaches Unity (post-ramp, post-overlay), tagged with the live source (MANUAL | AUTO | AUTO(STALE)).
        # This is the diagnostic that found the session-17 triggerDown bug and lets us confirm the session-18
        # autonomy smoothing matches manual — diff AUTO vs MANUAL rows to see the identical ramp. Log-only.
        self.cmd_log = cmd_log or NullLog()

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
        self._detect_held = False          # rising-edge tracker for the detect key
        self._detect_requests = 0          # incremented on each detect-key press; main loop drains it
        self._capture_held = False         # rising-edge tracker for the capture key (space)
        self._capture_requests = 0         # incremented on each capture press; main loop drains it

        # --- Manual TRIM UP / TRIM DOWN macros ('t' / 'g'): a direct replay of the autonomous TRIM
        # state's AIM->FWD->RESET motion (flight_playbook.json "trim_up"/"trim_down"), with the ring-gate/
        # height-threshold DECISION logic stripped out per the operator's ask — just the motion. Manual,
        # not tied to autonomy_active; any other manual flight key cancels it (a real stick input always
        # wins, same philosophy as the autonomy abort below).
        self.pb = FlightPlaybook.load()
        self._trim_keys_down = set()       # rising-edge tracker (mirrors _keys_down but scoped to t/g)
        self._trim_player = None           # active RecipePlayer, or None
        self._trim_name = None             # "trim_up" | "trim_down" — which macro is playing (for logs)
        self._trim_phase_logged = None     # last (name, step index) printed, so we log each phase once

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
        # Session 18: the autopilot's THROTTLE (trigger/reverse) are RAMP TARGETS the 60 Hz loop chases
        # (via _ramp), not values written straight through — so thrust is smoothed like a manual stick.
        # Neutral (0) = the ramp bleeds the axis down; the throttle magnitude the autopilot asks for is
        # preserved (the ramp just chases 0.2 instead of manual's 1.0). yaw/pitch are NOT ramped (aim axes,
        # eased by the sim; turn is duration-not-magnitude) — ramping them stole time from every turn.
        self._auto_trigger_target = 0.0
        self._auto_reverse_target = 0.0

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
        print("[io_bridge] 't' = TRIM UP macro, 'g' = TRIM DOWN macro (direct replay of the autonomous "
              "TRIM motion).")
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
            # --log-commands: record the ACTUAL outgoing packet (post-ramp) with its source, so the
            # manual vs autonomous smoothing is directly diffable. NullLog when the flag is off (no cost).
            self.cmd_log.row(
                mono_ts=round(time.monotonic(), 4), source=self.autonomy_status,
                pitch=cs["pitch"], yaw=cs["yaw"],
                trigger=cs["trigger"], triggerDown=cs["trigger_down"],
                reverse=cs["reverse"], reverseDown=cs["reverse_down"],
                joy_vertical=cs["joy_vertical"], joy_horizontal=cs["joy_horizontal"],
                btnAdown=cs["btnAdown"], btnBdown=cs["btnBdown"], btnCdown=cs["btnCdown"],
                btnARMdown=cs["btnARMdown"], thumbDown=cs["thumb_down"], joyClick=cs["joy_click"])
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
        while self._running.is_set():
            self._step_controls()
            time.sleep(1 / 60)

    def _step_controls(self):
        """One 60 Hz control tick. Apply the autonomy overlay (which sets the ramp TARGETS when the
        autopilot is driving), then ramp the analog axes toward those targets. Manual and autonomous
        flight share the SAME smoothing constants (session 18) so scripted flight eases in/out like a
        hand-flown stick instead of hard-stepping. Extracted from the loop so a self-test can drive it
        tick-by-tick."""
        cs = self.control_state
        self._apply_autonomy_overlay()
        if self.autonomy_active:
            # AUTONOMY: smooth only the THROTTLE axes (trigger/reverse) — asymmetric +0.05 attack / -0.1
            # decay, floor 0 — which is what gentled the height/brake jolt. yaw/pitch are AIM axes and are
            # applied DIRECTLY by the overlay (NOT ramped): the sim itself eases the aim, and the drone only
            # rotates once the aim REACHES full deflection, with the turn amount set by how long it's HELD
            # there (duration, not magnitude). Ramping the aim here just delayed it reaching +/-1 and stole
            # ~0.33s from every turn (session-18 live finding: 30deg turns collapsed to ~5deg). 'c' still
            # snaps the aim to 0 (matches the sample) so a reset_attitude_before_forward pulse is prompt.
            cs["trigger"] = _ramp(cs["trigger"], self._auto_trigger_target, 0.05, 0.1)
            cs["reverse"] = _ramp(cs["reverse"], self._auto_reverse_target, 0.05, 0.1)
            # Hold the gas GATE for as long as thrust is actually being SENT: derive trigger_down/reverse_down
            # from the RAMPED analog, NOT the autopilot's commanded gate. Unity gates real thrust on the
            # boolean (session 17), so if the gate drops the instant a stop is commanded while the analog is
            # still decaying (0.4->0 over ~4 ticks), Unity hard-cuts the thrust and the smooth release never
            # reaches it — the suspected plan-lost brake/pitch-up. Gate True <=> analog > 0 keeps thrust
            # following the smooth decay all the way to 0 (and still engages on tick 1 of a ramp-up).
            cs["trigger_down"] = cs["trigger"] > 0.0
            cs["reverse_down"] = cs["reverse"] > 0.0
            if cs["btnCdown"]:
                cs["yaw"] = 0.0
                cs["pitch"] = 0.0
            return
        # TRIM UP / TRIM DOWN macro ('t' / 'g'): drive control_state from the RecipePlayer, overriding the
        # manual key-derived values for the macro's duration — same ramp-toward-target treatment as the
        # autonomy overlay above, so the push eases in/out instead of stepping. Independent of
        # autonomy_active; any other manual flight key cancels it (handled in _on_key_event).
        if self._trim_player is not None:
            fields, done = self._trim_player.fields(time.monotonic())
            if not done:
                phase_key = (self._trim_name, self._trim_player.i)
                if phase_key != self._trim_phase_logged:
                    self._trim_phase_logged = phase_key
                    print(f"[io_bridge] TRIM {self._trim_name} phase {self._trim_player.i + 1}/"
                          f"{len(self._trim_player.steps)}: {fields}")
                cs["trigger"] = _ramp(cs["trigger"], float(fields.get("trigger", 0.0)), 0.05, 0.1)
                cs["trigger_down"] = bool(fields.get("trigger_down", False))
                cs["pitch"] = float(fields.get("pitch", 0.0))
                cs["btnCdown"] = bool(fields.get("btnCdown", False))
                return
            print(f"[io_bridge] TRIM {self._trim_name} complete.")
            cs["btnCdown"] = False
            self._trim_player = None
            self._trim_name = None
            self._trim_phase_logged = None
            # fall through to normal manual handling this same tick — the macro just ended
        # MANUAL: behaviour IDENTICAL to the sample (key-gated ramp; persisting yaw/pitch aim). UNCHANGED.
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
            # Session 18: the THROTTLE axes (trigger/reverse) become RAMP TARGETS the 60 Hz loop chases
            # (see _step_controls) instead of being written straight through — so thrust eases in/out like
            # a manual stick (this gentled the height/brake jolt). Everything else — yaw/pitch (AIM axes,
            # which the sim eases itself and whose turn is duration-not-magnitude), the boolean gas gates
            # trigger_down/reverse_down, btnARM/btnC, and the joysticks — is applied DIRECTLY. NB the
            # derived trigger_down/reverse_down still gate Unity's real thrust, held True while the analog
            # ramps up — identical to manual.
            for k in self.AUTONOMY_FIELDS:
                if k not in cmd:
                    continue
                if k == "trigger":
                    self._auto_trigger_target = float(cmd[k] or 0.0)
                elif k == "reverse":
                    self._auto_reverse_target = float(cmd[k] or 0.0)
                else:
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
        # A real flight input always wins: any OTHER manual flight key cancels an in-progress trim macro
        # (no blending — the operator's direct stick input takes over immediately, same philosophy as
        # the autonomy abort above).
        if is_down and key in self.MANUAL_FLIGHT_KEYS and key not in self.TRIM_KEYS and self._trim_player is not None:
            print(f"[io_bridge] TRIM ({self._trim_name}) cancelled by manual key '{key}'.")
            self._trim_player = None
            self._trim_name = None
            self._trim_phase_logged = None

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

        # TRIM UP / TRIM DOWN macro hotkeys: rising edge only (starts fresh each press; re-pressing the
        # SAME key while it's already playing just restarts it — pressing the OTHER trim key cancels and
        # starts the new one, same as any manual key would above).
        if key in self.TRIM_KEYS:
            if is_down and key not in self._trim_keys_down:
                self._start_trim_macro(key)
            self._trim_keys_down.add(key) if is_down else self._trim_keys_down.discard(key)

    def _start_trim_macro(self, key):
        """Begin playing the TRIM UP ('t') / TRIM DOWN ('g') macro: a direct replay of the autonomous
        TRIM state's motion (see flight_playbook.json), independent of autonomy_active. `_step_controls`
        drives control_state from it every 60 Hz tick until it completes."""
        name = self.TRIM_KEYS[key]
        self._trim_player = self.pb.player(name)
        self._trim_name = name
        self._trim_phase_logged = None
        cs = self.control_state
        # Clean slate: a forward-pitching climb/descend shouldn't fight a lingering reverse hold.
        cs["reverse"], cs["reverse_down"] = 0.0, False
        print(f"[io_bridge] TRIM macro '{name}' started ('{key}').")

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
        """Release everything the autopilot drives (abort / disable / stale-link). Session 18: THROTTLE
        (trigger/reverse) bleeds down SMOOTHLY via the ramp targets — a hard cut is exactly the jolt we
        removed — while the AIM axes (yaw/pitch) and the boolean gas gates snap to neutral immediately, so
        a lost link can never leave the drone rotating or gated-on. The ramp (_step_controls) then decays
        cs['trigger']/['reverse'] toward these 0 targets over the next few ticks, like releasing the stick."""
        cs = self.control_state
        cs["btnARMdown"] = False
        cs["btnCdown"] = False
        cs["trigger_down"] = False   # release the BOOLEAN gas gate on abort/stale-link (session 17)
        cs["reverse_down"] = False
        cs["joy_vertical"] = 0
        cs["joy_horizontal"] = 0
        cs["yaw"] = 0.0             # aim axes snap to neutral (no auto-return in manual; must not coast a spin)
        cs["pitch"] = 0.0
        self._auto_trigger_target = 0.0   # throttle bleeds down smoothly (ramp chases 0)
        self._auto_reverse_target = 0.0

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
        self.cmd_log.close()
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
def _self_test():
    """Offline unit test for the session-18 command SMOOTHING (no sockets / NDI). Drives _step_controls
    tick-by-tick and asserts the autopilot's trigger/reverse/yaw are ramped with the manual constants,
    that release DECAYS, that 'c' snaps the aim, and that MANUAL flight is byte-identical to before."""
    APPROX = 1e-9
    dc = DroneControl("127.0.0.1", 0)
    dc.autonomy_active = True
    seq = 0

    def auto(**cmd):
        nonlocal seq
        seq += 1
        cmd["seq"] = seq
        dc.set_autonomy_command(cmd)
        dc._step_controls()

    # trigger ramps 0 -> 0.2 at +0.05/tick (throttle target 0.2, NOT full 1.0), then holds without overshoot
    for _ in range(3):
        auto(trigger=0.2, trigger_down=True)
    up_ok = abs(dc.control_state["trigger"] - 0.15) < APPROX
    for _ in range(5):
        auto(trigger=0.2, trigger_down=True)
    hold_ok = abs(dc.control_state["trigger"] - 0.2) < APPROX
    # release (fresh command, trigger 0) DECAYS at -0.1/tick down to 0 — the smooth "let go of the stick".
    # The gas GATE must stay True WHILE the analog is still >0 (derived from the RAMPED analog, not the
    # commanded gate=False), so Unity's boolean-gated thrust follows the smooth decay instead of hard-cutting.
    auto(trigger=0.0, trigger_down=False)
    dec1_ok = abs(dc.control_state["trigger"] - 0.1) < APPROX and dc.control_state["trigger_down"] is True
    auto(trigger=0.0, trigger_down=False)
    dec0_ok = abs(dc.control_state["trigger"] - 0.0) < APPROX and dc.control_state["trigger_down"] is False
    trig_ok = up_ok and hold_ok and dec1_ok and dec0_ok

    # yaw (AIM) is NOT ramped — it passes straight through in ONE tick (the sim eases it; the turn is
    # duration-not-magnitude, so ramping would only steal turn time). A 'c' reset snaps the aim to 0.
    dy = DroneControl("127.0.0.1", 0)
    dy.autonomy_active = True
    dy.set_autonomy_command({"seq": 1, "yaw": 1.0})
    dy._step_controls()
    yaw_passthru_ok = dy.control_state["yaw"] == 1.0            # full deflection immediately, no ramp
    dy.set_autonomy_command({"seq": 2, "yaw": 1.0, "btnCdown": True})
    dy._step_controls()
    yaw_snap_ok = dy.control_state["yaw"] == 0.0
    yaw_ok = yaw_passthru_ok and yaw_snap_ok

    # MANUAL flight unchanged: holding the gas gate ramps trigger toward 1.0 at +0.05/tick
    dm = DroneControl("127.0.0.1", 0)   # autonomy_active stays False
    dm.control_state["trigger_down"] = True
    for _ in range(3):
        dm._step_controls()
    manual_ok = abs(dm.control_state["trigger"] - 0.15) < APPROX

    ok = trig_ok and yaw_ok and manual_ok
    print(f"[self-test] {'PASS' if ok else 'FAIL'}  io_bridge command SMOOTHING "
          f"(auto trigger up/hold/decay={trig_ok}, auto yaw passthru+c-snap={yaw_ok}, manual unchanged={manual_ok})")
    return ok


# ==============================================================================
def main():
    parser = argparse.ArgumentParser(description="Cartographer io_bridge (P1)")
    parser.add_argument("--config", default=None, help="path to config.yaml")
    parser.add_argument("--debug-keys", action="store_true", help="print every key event")
    parser.add_argument("--no-display", action="store_true", help="headless: skip the OpenCV window")
    parser.add_argument("--log-keys", action="store_true",
                        help="write every keyboard edge (rec_frame,mono_ts,key,action) to "
                             "OUTPUT/diag/<ts>_keys.csv, frame-synced to the recorded flight video")
    parser.add_argument("--log-commands", action="store_true",
                        help="write the actual outgoing control packet (post-ramp, tagged MANUAL/AUTO) "
                             "every 60 Hz send to OUTPUT/diag/<ts>_commands.csv — for diffing manual vs "
                             "autonomous stick smoothing (session 18)")
    parser.add_argument("--self-test", action="store_true",
                        help="run the offline command-smoothing unit test (no sockets/NDI) and exit")
    args = parser.parse_args()

    if args.self_test:
        import sys
        sys.exit(0 if _self_test() else 1)

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
    detect_key = cfg["models"]["qwen_vl"].get("object_trigger_key", "h")
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

    # --- optional outgoing-packet log (--log-commands): the actual post-ramp control vector + source,
    # for diffing manual vs autonomous stick smoothing (session 18). Header matches the session-17 artifacts. ---
    cmd_log = None
    if args.log_commands:
        cmd_log = DiagLog("commands", [
            "mono_ts", "source", "pitch", "yaw", "trigger", "triggerDown", "reverse", "reverseDown",
            "joy_vertical", "joy_horizontal", "btnAdown", "btnBdown", "btnCdown", "btnARMdown",
            "thumbDown", "joyClick"])

    # --- control server + NDI (both fail-fast) ---
    control = DroneControl(host, port, detect_key=detect_key, capture_key=capture_key,
                           debug_keys=args.debug_keys, autonomy_enable_key=autonomy_enable_key,
                           cmd_timeout_s=cmd_timeout_s, key_log=key_log, cmd_log=cmd_log)
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
