"""flow_contact_detector.py — self-calibrating OPTICAL-FLOW contact detector (CPU-only).

Replaces autopilot's SLAM-pose CeilingStallDetector (which failed twice: monocular SLAM poses arrive
at only ~1 Hz and slow to ~0.27 Hz at a near surface, so a rate/plateau primitive never armed). The
right "am I still making progress?" signal is in the camera IMAGE at 30 fps. Two events, one shared
mechanism — a self-calibrating COLLAPSE of the relevant flow signal while the matching command is held:

  * CEILING (while ASCENT is commanded): vertical flow |dy_med| collapses from its live ascent level
    to ~0. (Validated on flight_20260627_214625: |dy_med| high during climbs, ~0 at the plateaus.)
  * WALL (while FORWARD is commanded): the looming radial EXPANSION collapses from its live
    free-forward level to ~0 — forward progress has stopped. This ONE signal unifies both wall flavors:
    a textureless wall freezes the image (mag→0) and a textured wall shows a slow vertical climb; either
    way the radial looming dies. (Validated: expansion ramped to 3.7/8.7 then collapsed at the walls.)

================================ HARD RULE ================================
NO MANUAL-FLIGHT DATA LEAKAGE (cartographer/CLAUDE.md "CRITICAL AUTONOMY STANDARD"). Detection is
RELATIVE/self-calibrating: each verdict compares the CURRENT flow to a reference MEASURED LIVE this
episode (the running windowed-max of the signal while the command is held), as a scale-free ratio. Flow
magnitudes are resolution-dependent, so input is normalized to a fixed working resolution and only
ratios + general durations/noise-floors are used. NO constant here encodes this room's geometry
(ceiling altitude, distance-to-wall, a frame index). Gating on the COMMAND is a structural signal.
==========================================================================

ACCELERATION DEAD-ZONE (inertia at command onset): when a push starts from rest the drone accelerates
over a fraction of a second, so the flow signal is ~0 for the first frames. Three layered guards keep
that from reading as a (false) collapse, ALL required before a collapse can count:
  1. ref must cross a baseline: `armed` only after the live reference has exceeded its noise floor
     (the drone measurably moved/loomed this episode);
  2. onset blanking: the collapse test is suppressed for `arm_blank_s` after the command's onset;
  3. sustained: the collapse must hold `contact_seconds` continuously.
Each fresh command stretch is a new EPISODE (calibration + verdict reset) so every push re-runs the
ramp-up guard and no stale verdict latches across a pause.
"""

import argparse
import os
import time
from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np
import yaml

# Reuse the EXACT flow primitive + key-timeline helpers from the offline analyzer so the live detector
# and learn_to_fly characterize frames identically.
from learn_to_fly import farneback_scalars, make_radial_field, load_key_edges, build_active_timeline

REPO = os.path.dirname(os.path.abspath(__file__))

CMD_UP = "up"            # ascent commanded  -> tests for CEILING (signal = |dy_med|)
CMD_FWD = "forward"      # forward commanded -> tests for WALL    (signal = expansion)


def load_config(path=None):
    path = path or os.path.join(REPO, "config.yaml")
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


# ==============================================================================
# Verdict
# ==============================================================================
@dataclass
class FlowVerdict:
    """One detector evaluation — everything needed to LOG the live reasoning."""
    t: float
    command: str | None
    kind: str | None = None            # CEILING | WALL — what the current command tests for
    signal: float | None = None        # current flow signal (|dy_med| or expansion); None until flow exists
    ref: float = 0.0                   # LIVE persistent per-command reference (running max of the signal)
    ratio: float | None = None         # signal / ref (scale-free); None pre-reference
    airborne: bool = False             # the drone has measurably moved at least once this flight (takeoff done)
    blanking: bool = False             # within arm_blank_s of command onset (dead-zone guard)
    collapse_cond: bool = False        # the instantaneous collapse condition holds this frame
    contact_held: float = 0.0          # seconds the collapse has held continuously
    contact: bool = False              # CONTACT reached (latched within the current command episode)

    def label(self) -> str:
        if self.command not in (CMD_UP, CMD_FWD):
            return "NOT-COMMANDED"      # nothing to test for -> idle
        if self.contact:
            return self.kind            # CEILING or WALL
        if self.signal is None:
            return "PRE-FLOW"           # commanded, but no flow computed yet (first frame of episode)
        if self.blanking:
            return "BLANK"              # onset blanking window (dead-zone guard)
        if not self.airborne:
            return "PRE-MOTION"         # TAKEOFF only: never moved yet (countdown / first climb)
        if self.collapse_cond:
            return f"{self.kind}-WATCH({self.contact_held:.2f}s)"
        return "RISING" if self.command == CMD_UP else "LOOMING"


# ==============================================================================
# Detector
# ==============================================================================
class FlowContactDetector:
    """Causal, self-calibrating optical-flow contact detector. Feed frames + the current command;
    get a FlowVerdict each step. See module docstring for the signal model + dead-zone guards."""

    def __init__(self, *, flow_long_side, stall_frac,
                 dy_noise_floor, exp_noise_floor, contact_seconds, arm_blank_s):
        self.flow_long_side = int(flow_long_side)
        self.stall_frac = float(stall_frac)
        self.dy_noise_floor = float(dy_noise_floor)
        self.exp_noise_floor = float(exp_noise_floor)
        self.contact_seconds = float(contact_seconds)
        self.arm_blank_s = float(arm_blank_s)

        # working-resolution + flow state
        self._prev_gray = None
        self._fw = self._fh = None
        self._rux = self._ruy = None
        # FLIGHT-level state (persists across episodes): airborne latch + per-command running-max ref.
        # `airborne` flips True the first time any motion exceeds the noise floor (takeoff). The refs
        # are the characteristic ascent / forward-looming flow this flight; they MUST persist so that a
        # push which starts already blocked (e.g. re-pressing UP while parked at the ceiling) still has
        # a reference to collapse against (the per-episode reset was the live PRE-MOTION-forever bug).
        self._airborne = False
        self._ref_up = 0.0
        self._ref_fwd = 0.0
        # per-PUSH (episode) state — reset on each command change
        self._cmd = None
        self._ep_t0 = None
        self._contact_since = None
        self._contact = False

    def _signal_cfg(self, command):
        """(kind, noise_floor, ref_attr) for the command, or (None, None, None) if not tested."""
        if command == CMD_UP:
            return "CEILING", self.dy_noise_floor, "_ref_up"
        if command == CMD_FWD:
            return "WALL", self.exp_noise_floor, "_ref_fwd"
        return None, None, None

    def _reset_episode(self, t, command):
        """A command change ends the push: reset ONLY the per-push state. Refs + airborne persist."""
        self._cmd = command
        self._ep_t0 = t
        self._contact_since = None
        self._contact = False

    def notify_landed(self):
        """Clear the airborne latch + learned references (the drone is grounded and must take off
        again). Matches the user's mental model; unused in normal operation since we never land."""
        self._airborne = False
        self._ref_up = 0.0
        self._ref_fwd = 0.0

    def _prep_gray(self, frame):
        """BGR/gray frame -> grayscale, downscaled so the long side == flow_long_side (resolution
        normalization: identical behavior whether fed the 512x288 live transport or a 1280x720
        recording; disclosed, never an upscale). Rebuilds the radial field if the size changes."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        h, w = gray.shape[:2]
        scale = min(1.0, self.flow_long_side / float(max(h, w)))
        if scale < 1.0:
            gray = cv2.resize(gray, (max(1, int(round(w * scale))), max(1, int(round(h * scale)))),
                              interpolation=cv2.INTER_AREA)
        fh, fw = gray.shape[:2]
        if (fw, fh) != (self._fw, self._fh):
            self._fw, self._fh = fw, fh
            self._rux, self._ruy = make_radial_field(fw, fh)
            self._prev_gray = None        # size changed -> previous frame is incomparable
        return gray

    def update(self, t: float, frame, command) -> FlowVerdict:
        """Process one frame for the given held command. Computes the flow signal, then runs the
        self-calibrating collapse logic (see _update_signal)."""
        gray = self._prep_gray(frame)
        signal = None
        if command in (CMD_UP, CMD_FWD) and self._prev_gray is not None:
            sc = farneback_scalars(self._prev_gray, gray, self._rux, self._ruy)
            signal = abs(sc["dy_med"]) if command == CMD_UP else sc["expansion"]
        self._prev_gray = gray
        return self._update_signal(t, signal, command)

    def _update_signal(self, t: float, signal, command) -> FlowVerdict:
        """The decision logic, separated from frame->flow so the self-test can drive it with synthetic
        signal streams (no fabricated frames). `signal` is |dy_med| for UP, expansion for FORWARD, or
        None when no flow exists yet."""
        # --- episode management: a new/none command starts a fresh episode (reset calibration) ---
        if command != self._cmd:
            self._reset_episode(t, command)

        kind, noise, ref_attr = self._signal_cfg(command)
        if kind is None:
            return FlowVerdict(t=t, command=command)        # not commanding up/forward -> idle

        v = FlowVerdict(t=t, command=command, kind=kind)
        if signal is None:
            return v                                        # commanded but no flow yet (PRE-FLOW)

        v.signal = signal
        # --- LIVE calibration: persistent per-command running-max reference; airborne latch ---
        ref = getattr(self, ref_attr)
        if signal > noise:                                  # real motion this command -> learn + take off
            ref = max(ref, signal)
            setattr(self, ref_attr, ref)
            self._airborne = True
        v.ref = ref
        v.airborne = self._airborne
        v.ratio = (signal / ref) if ref > 0 else None

        # --- dead-zone guard: suppress the collapse test during onset blanking (hover->move inertia) ---
        v.blanking = (t - self._ep_t0) < self.arm_blank_s

        # --- collapse (only post-takeoff): forward/vertical progress stopped while commanded. Requires
        # a LEARNED reference for this command (ref>0): the drone must have demonstrably moved this way
        # (this flight) before "no movement" can mean "blocked", else the acceleration ramp at the start
        # of a push (low signal before the drone gets going) would false-fire. The persistent ref is what
        # lets a push that STARTS blocked still fire (e.g. re-pressing UP while parked at the ceiling —
        # ref_up was learned at takeoff). `signal < stall_frac*ref` is the scale-free collapse; the
        # absolute `signal < noise` confirms it actually stopped. This captures the freeze-OR-climb OR:
        # a wall freezes the image OR makes it climb vertically — both drive expansion -> ~0.
        collapse = (
            self._airborne
            and not v.blanking
            and ref > 0.0
            and signal < self.stall_frac * ref
            and signal < noise
        )
        v.collapse_cond = collapse
        if collapse:
            if self._contact_since is None:
                self._contact_since = t
            v.contact_held = t - self._contact_since
            if v.contact_held >= self.contact_seconds:
                self._contact = True
        else:
            self._contact_since = None
        v.contact = self._contact
        return v


def detector_from_cfg(cfg):
    f = cfg["autonomy"]["flow"]
    return FlowContactDetector(
        flow_long_side=f["flow_long_side"],
        stall_frac=f["stall_frac"], dy_noise_floor=f["dy_noise_floor"],
        exp_noise_floor=f["exp_noise_floor"], contact_seconds=f["contact_seconds"],
        arm_blank_s=f["arm_blank_s"])


# ==============================================================================
# Offline validation: replay a recorded flight (video + keys) frame-by-frame.
# Validates the detection LOGIC on REAL captured data (the dry-run #2 lesson: synthetic streams hid
# the failure). Frame index == io_bridge rec_frame == the video frame, so events tie to the footage.
# ==============================================================================
def _active_command(active_set):
    """Map the per-frame held semantic commands to the single command the detector tests (UP wins
    over FORWARD if both somehow held; everything else -> None)."""
    if CMD_UP in active_set:
        return CMD_UP
    if CMD_FWD in active_set:
        return CMD_FWD
    return None


def _parse_ranges(s):
    """'a:b,c:d' -> [(a,b),(c,d)] (validation ground-truth ranges; NOT used by the live detector)."""
    out = []
    for part in (s or "").split(","):
        part = part.strip()
        if not part:
            continue
        a, b = part.split(":")
        out.append((int(a), int(b)))
    return out


def run_validate(cfg, video, keys, expect_ceiling="", expect_wall="", dead_zone_frames=12):
    det = detector_from_cfg(cfg)
    cap = cv2.VideoCapture(video)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open video: {video}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    n_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    edges = load_key_edges(keys)
    active = build_active_timeline(edges, n_frames)
    print(f"[validate] {video}  {n_frames} frames @ {fps:.1f} fps | {len(edges)} key edges")
    print(f"[validate] detector: long_side={det.flow_long_side} stall_frac={det.stall_frac} "
          f"dy_floor={det.dy_noise_floor} exp_floor={det.exp_noise_floor} "
          f"contact={det.contact_seconds}s arm_blank={det.arm_blank_s}s\n")

    events = []                 # contact onsets: (kind, frame)
    push_starts = {CMD_UP: [], CMD_FWD: []}
    prev_contact = False
    prev_cmd = None
    last_label = None
    idx = -1
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        idx += 1
        cmd = _active_command(active[idx]) if idx < len(active) else None
        if cmd in (CMD_UP, CMD_FWD) and cmd != prev_cmd:
            push_starts[cmd].append(idx)
        prev_cmd = cmd
        t = idx / fps
        v = det.update(t, frame, cmd)
        if v.contact and not prev_contact:
            events.append((v.kind, idx))
            print(f"  >>> {v.kind} CONTACT @ frame {idx}  (ref={v.ref:.3f} signal={v.signal:.4f})")
        prev_contact = v.contact
        if v.label() != last_label:
            sig = f"{v.signal:.4f}" if v.signal is not None else "  -   "
            print(f"  f{idx:>5} cmd={str(cmd):7s} signal={sig} ref={v.ref:6.3f} -> {v.label()}")
            last_label = v.label()
    cap.release()

    ceil_events = [f for (k, f) in events if k == "CEILING"]
    wall_events = [f for (k, f) in events if k == "WALL"]
    print(f"\n[validate] CEILING contacts at frames {ceil_events}")
    print(f"[validate] WALL    contacts at frames {wall_events}")

    ok_all = True

    def check(expect, got, kind):
        nonlocal ok_all
        for (a, b) in expect:
            hit = any(a - 5 <= f <= b for f in got)
            ok_all = ok_all and hit
            print(f"  {'PASS' if hit else 'FAIL'} {kind} expected in [{a},{b}]: "
                  f"{'a contact at '+str([f for f in got if a-5<=f<=b]) if hit else 'NONE'}")

    ec, ew = _parse_ranges(expect_ceiling), _parse_ranges(expect_wall)
    if ec:
        check(ec, ceil_events, "CEILING")
    if ew:
        check(ew, wall_events, "WALL")
    # Dead-zone: no contact must fire within the first `dead_zone_frames` of any push (inertial ramp).
    for kind, cmd in (("CEILING", CMD_UP), ("WALL", CMD_FWD)):
        got = ceil_events if cmd == CMD_UP else wall_events
        for ps in push_starts[cmd]:
            early = [f for f in got if ps <= f < ps + dead_zone_frames]
            if early:
                ok_all = False
                print(f"  FAIL DEAD-ZONE: {kind} fired at {early} within {dead_zone_frames}f of push @ {ps}")
    if ec or ew:
        print(f"  PASS DEAD-ZONE: no contact within {dead_zone_frames}f of any push onset"
              if ok_all else "  (see dead-zone failures above)")
        print(f"\n[validate] {'ALL PASS' if ok_all else 'FAILURES PRESENT'}")
    return ok_all


# ==============================================================================
# Self-test: SYNTHETIC signal streams (no frames, no flight data). Drives _update_signal directly to
# exercise the calibration / dead-zone / collapse logic, at varied scales (proves it is scale-free).
# ==============================================================================
def _mk(**over):
    p = dict(flow_long_side=320, stall_frac=0.15,
             dy_noise_floor=0.05, exp_noise_floor=0.05, contact_seconds=0.8, arm_blank_s=0.4)
    p.update(over)
    return FlowContactDetector(**p)


def _feed_signal(det, seq, command, dt=1 / 30.0):
    """Feed (signal_or_None) samples for a fixed command; return frame index of first CONTACT."""
    fired = None
    t = 0.0
    for i, s in enumerate(seq):
        t += dt
        v = det._update_signal(t, s, command)
        if v.contact and fired is None:
            fired = i
    return fired


def run_self_test():
    dt = 1 / 30.0
    ok = True

    def case(name, seq, command, expect_fire):
        nonlocal ok
        det = _mk()
        fired = _feed_signal(det, seq, command, dt)
        good = (fired is not None) == expect_fire
        ok = ok and good
        got = f"FIRED@f{fired}" if fired is not None else "no-fire"
        print(f"[self-test] {'PASS' if good else 'FAIL'}  {name}: {got} "
              f"(expected {'fire' if expect_fire else 'quiet'})")

    def ramp(peak, n):                         # 0 -> peak over n samples (inertial accel)
        return [peak * (i + 1) / n for i in range(n)]

    def rise_then_plateau(peak, rise_n, plat_n, plateau=0.0):
        return ramp(peak, rise_n) + [peak] * 10 + [plateau] * plat_n

    # 1-3. Commanded motion then a hard collapse -> CONTACT, across scales (scale-free); plus a
    #      below-noise-floor case that must NEVER arm (dead-zone guard #1: ref must cross the baseline).
    case("CEILING rise->plateau (scale 1.0)", rise_then_plateau(1.0, 15, 40), CMD_UP, True)
    case("CEILING rise->plateau (scale 10x)", rise_then_plateau(10.0, 15, 40), CMD_UP, True)
    case("CEILING rise->plateau (small scale 0.2)", rise_then_plateau(0.2, 15, 40), CMD_UP, True)
    case("CEILING below noise floor (never arms)", rise_then_plateau(0.03, 15, 40), CMD_UP, False)
    # 4. WALL: expansion ramps then collapses -> WALL fires (textureless freeze: plateau exactly 0).
    case("WALL loom->freeze (collapse to 0)", rise_then_plateau(3.0, 15, 40, plateau=0.0), CMD_FWD, True)
    # 5. WALL textured slow-climb: residual tiny expansion below the floor -> still fires.
    case("WALL loom->slow-climb (resid 0.01)", rise_then_plateau(3.0, 15, 40, plateau=0.01), CMD_FWD, True)
    # 6. Continuous motion, no collapse -> quiet.
    case("CEILING continuous rise (no ceiling)", ramp(1.0, 10) + [1.0] * 60, CMD_UP, False)
    # 7. DEAD-ZONE: 12 frames of ~0 (inertia) then ramp up and KEEP rising -> must NOT fire.
    case("DEAD-ZONE accel then rise (no false fire)",
         [0.0] * 12 + ramp(1.0, 10) + [1.0] * 50, CMD_UP, False)
    # 8. DEAD-ZONE then a REAL collapse later -> fires (and not during the initial zeros).
    case("DEAD-ZONE accel, rise, then real collapse",
         [0.0] * 12 + ramp(1.0, 10) + [1.0] * 10 + [0.0] * 40, CMD_UP, True)
    # 9. Not commanded (rise+plateau shape but command=None) -> idle, quiet.
    case("never-commanded rise+plateau", rise_then_plateau(1.0, 15, 40), None, False)
    # 10. NEVER AIRBORNE: commanded up but the signal is zero from the very start (e.g. sitting on the
    #     ground / textureless from frame 0, never took off) -> must stay quiet (PRE-MOTION protection).
    case("never-airborne all-zero (no takeoff)", [0.0] * 60, CMD_UP, False)

    # 11. THE LIVE BUG (frozen start AFTER takeoff): episode 1 climbs (airborne + ref learned); release;
    #     episode 2 re-presses UP while ALREADY parked at the ceiling (signal ~0 from frame 0). With the
    #     persistent ref + airborne latch this now fires CEILING (previously stuck in PRE-MOTION forever).
    det = _mk()
    _feed_signal(det, rise_then_plateau(1.0, 15, 40), CMD_UP)   # takeoff + first climb -> airborne, ref_up
    _feed_signal(det, [0.0] * 5, None)                          # release UP (episode ends; contact clears)
    fired = _feed_signal(det, [0.0] * 50, CMD_UP)               # re-press UP, already frozen at the ceiling
    good = fired is not None
    ok = ok and good
    print(f"[self-test] {'PASS' if good else 'FAIL'}  frozen-start after takeoff -> CEILING: "
          f"{'FIRED@f'+str(fired) if good else 'no-fire (BUG)'}")

    # 12. Per-push contact verdict resets on command change: UP fires, switch to FORWARD ramp (no
    #     collapse) -> the stale CEILING does not carry over as a FORWARD contact.
    det = _mk()
    _feed_signal(det, rise_then_plateau(1.0, 15, 40), CMD_UP)
    fired2 = _feed_signal(det, ramp(3.0, 15) + [3.0] * 20, CMD_FWD)
    good = fired2 is None
    ok = ok and good
    print(f"[self-test] {'PASS' if good else 'FAIL'}  contact resets on command change: "
          f"{'no carry-over' if good else 'FALSE carry-over @f'+str(fired2)}")

    print(f"\n[self-test] {'ALL PASS' if ok else 'FAILURES PRESENT'}")
    return ok


def main():
    ap = argparse.ArgumentParser(description="Optical-flow contact detector (CEILING + WALL)")
    ap.add_argument("--config", default=None)
    ap.add_argument("--self-test", action="store_true",
                    help="validate the detection LOGIC on synthetic signal streams (no hardware)")
    ap.add_argument("--validate", action="store_true",
                    help="replay a recorded flight (needs --video and --keys)")
    ap.add_argument("--video", default=None)
    ap.add_argument("--keys", default=None)
    ap.add_argument("--expect-ceiling", default="", help="ground-truth ranges 'a:b,c:d' for PASS/FAIL")
    ap.add_argument("--expect-wall", default="", help="ground-truth ranges 'a:b,c:d' for PASS/FAIL")
    args = ap.parse_args()

    if args.self_test:
        raise SystemExit(0 if run_self_test() else 1)
    if args.validate:
        if not (args.video and args.keys):
            ap.error("--validate requires --video and --keys")
        cfg = load_config(args.config)
        ok = run_validate(cfg, args.video, args.keys, args.expect_ceiling, args.expect_wall)
        raise SystemExit(0 if ok else 1)
    ap.error("nothing to do: pass --self-test or --validate")


if __name__ == "__main__":
    main()
