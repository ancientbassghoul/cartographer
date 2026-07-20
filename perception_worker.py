"""perception_worker.py — Process P2: GPU perception (MASt3R-SLAM + fused map).

Subscribes to the io_bridge frame bus (downscaled 512x288 BGR frames) and runs
**MASt3R-SLAM** (via `slam_engine.SlamEngine`) every frame — camera trajectory + dense
per-keyframe pointmaps, fused in-process into a `map_store.MapStore` voxel/occupancy map and
a `ground_grid.GroundGrid` 2D free/unknown/occupied layer for the frontier planner.

DA-V2 depth was REMOVED (2026-07-07 refactor): the sim can't crash, so the depth-map height
adjustments it fed are gone, and dropping it frees the GPU that SLAM shares (SLAM is the
sensitive consumer). The autopilot's wall stand-off uses the SLAM raycast (forward_clearance_dist
on TOPIC_PLAN), not depth. Obstacle/clearance signals are all SLAM-derived now.

It publishes compact JSON payloads on its state bus (`perception_state_port`):
TOPIC_POSE (pose / mode / keyframe + voxel counts), TOPIC_MAP (top-down snapshot), TOPIC_PLAN
(frontier goal + clearances), TOPIC_TARGET — never raw pointmaps (those stay in-process; they
are ~440 K floats/keyframe). In display mode a window previews the growing top-down map.

Offline mode (`--video`) drives the entire SLAM+map pipeline straight from a recorded mp4 (no
io_bridge/NDI), then exports the fused map — the offline verification path.

NO SILENT FALLBACKS (per CLAUDE.md): CUDA availability and the SLAM load are asserted up front;
any failure raises. There is no CPU fallback.
"""

import argparse
import math
import os
import time

import cv2
import numpy as np
import torch
import yaml

import frame_bus
import slam_engine
from map_store import MapStore
from ground_grid import GroundGrid, explore_cfg
from frontier_planner import FrontierPlanner
from target_estimator import TargetEstimator
from diag_log import DiagLog, NullLog

REPO = os.path.dirname(os.path.abspath(__file__))

MAP_GRID = 200              # resolution of the compact top-down occupancy summary on TOPIC_MAP


def load_config(path=None):
    path = path or os.path.join(REPO, "config.yaml")
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _wrap180(a):
    """Wrap an angle (deg) to (-180, 180]."""
    return (a + 180.0) % 360.0 - 180.0


def heading_from_pose(pose):
    """World heading (deg) of the camera's forward axis projected onto the X-Z ground plane.

    The camera looks along +Z in its own frame (the lift confirms the center ray ~[0,0,1]), so the
    world forward vector is `R @ [0,0,1]` = the 3rd column of the rotation. Sim3 scale cancels in
    atan2. Returns None if pose is missing / degenerate. heading 0 = +Z, +90 = +X (right)."""
    if pose is None:
        return None
    fwd = np.asarray(pose, dtype=np.float64)[:3, 2]   # R @ [0,0,1]
    if abs(fwd[0]) < 1e-9 and abs(fwd[2]) < 1e-9:
        return None
    return math.degrees(math.atan2(fwd[0], fwd[2]))


# ==============================================================================
# Pipeline: SLAM (every frame), fused into the map. (DA-V2 depth removed 2026-07-07.)
# ==============================================================================
MAP_WINDOW = "Cartographer — top-down map"


class Pipeline:
    """Holds the SLAM worker + the map, and processes one frame at a time.

    `step()` runs SLAM on every frame, integrates each new keyframe's pointmap into the voxel
    map + ground grid, and publishes TOPIC_POSE (every frame) plus TOPIC_MAP/PLAN on their
    timers. (DA-V2 depth was removed 2026-07-07; the panel return is always None now.)
    """

    def __init__(self, cfg, conf_thresh=1.5, debug_lift=False):
        self.debug_lift = debug_lift
        self._geom_logged = False
        self.voxel_size = float(cfg["map"]["voxel_size"])
        self.proc_w = int(cfg["perception"]["processing_width"])
        self.proc_h = int(cfg["perception"]["processing_height"])

        # SLAM chdir's into its repo on load; it owns the GPU alone now (depth removed).
        self.slam = slam_engine.SlamEngine(conf_thresh=conf_thresh)
        self.mapstore = MapStore(self.voxel_size, tracking_mode=self.slam.tracking_mode)

        self.last_report = time.monotonic()
        self.last_map_pub = 0.0           # timer for TOPIC_MAP (dense trajectory) publishing
        self.MAP_PUB_INTERVAL = 0.5       # publish the map at >= 2 Hz even between keyframes

        # --- target lift (M-object Task 2): back-project detections into the voxel map ---
        # Recent per-frame poses so a detection (which lags its frame by the Qwen latency) can be
        # matched back to the camera pose of the frame it fired on. SLAM tracks every frame.
        self._pose_hist: dict[int, np.ndarray] = {}
        self._pose_keys: list[int] = []
        self.POSE_HIST_MAX = 600
        self.TARGET_MIN_COUNT = 2     # require a voxel seen >= this for a ray hit (denoise)
        self.TARGET_SKIP = 0.25       # skip the first 0.25u of each ray so a downward ray can't
                                      # grab a near-camera floor voxel before the target surface
        self.estimator = TargetEstimator()
        self.n_det_seen = 0
        self.last_target_pub = 0.0

        # --- Map mode (Phase 2): 2D ground occupancy + frontier planner ---
        # GroundGrid is the free/unknown/occupied layer MapStore lacks; the planner picks the next
        # frontier and publishes it on TOPIC_PLAN for the autopilot to execute. Pure numpy → no GPU.
        self.ground = GroundGrid(cfg)
        # Goal selection + done verification (utility + strong commitment + farthest-corner verify) lives
        # in the pure-numpy FrontierPlanner; perception just feeds it the live frontiers + pose.
        self.planner = FrontierPlanner(cfg)
        e = explore_cfg(cfg)
        self.goal_reach_dist = float(e.get("goal_reach_dist", 0.4))
        # Map-validated clearance buffer: a chosen frontier goal (which sits on the free/unknown boundary)
        # can hug an obstacle/corner and stall the drone. Pull it back along the drone->goal axis to a FREE
        # cell with this much clearance before committing. A general stand-off-scale distance validated
        # against the LIVE map every replan — never a precomputed coordinate. Default tracks the obstacle
        # inflation width so the buffered goal sits at least one inflation ring off known obstacles.
        self.goal_clearance_buffer = float(e.get("goal_clearance_buffer", self.ground.obstacle_inflation * self.ground.cell))
        self.planner.set_clearance_fn(
            lambda goal, pos: self.ground.inset_to_clearance(goal, pos, self.goal_clearance_buffer))
        # Pull the reposition/verify far-corner target inward by this margin so it is REACHABLE (the raw
        # farthest free cell sits against the wall, inside the stand-off shell). General stand-off scale.
        # It must be coordinated with the autopilot's forward stand-off: the drone stops
        # stop_clearance_dist short of walls and "reaches" a goal within goal_reach_dist, so the inset
        # target is reachable only for stop_clearance_dist <= inset <= stop_clearance_dist + goal_reach_dist.
        # Clamp into that band with a VISIBLE warning (NO SILENT FALLBACK) rather than strand the drone.
        self.reposition_inset = float(e.get("reposition_inset", 0.8))
        _stop_clr = float(e.get("stop_clearance_dist", 0.6))
        _lo, _hi = _stop_clr, _stop_clr + self.goal_reach_dist
        if not (_lo <= self.reposition_inset <= _hi):
            clamped = min(max(self.reposition_inset, _lo), _hi)
            print(f"[perception] WARNING: reposition_inset {self.reposition_inset:.2f} outside the reachable "
                  f"band [{_lo:.2f}, {_hi:.2f}] (stop_clearance_dist + goal_reach_dist) -> clamped to "
                  f"{clamped:.2f} so the reposition corner stays reachable", flush=True)
            self.reposition_inset = clamped
        self.PLAN_PUB_INTERVAL = float(e.get("replan_period_s", 0.5))
        self.GROUND_RASTER = 160
        self.last_plan_pub = 0.0
        # Forward clearance: cast a ground-plane ray fan into the voxel map we built and report the
        # nearest hit distance on TOPIC_PLAN, so the autopilot stops BEFORE ramming a wall (a head-on
        # ram freezes the image and kills monocular SLAM). General stand-off, NOT a room answer (the wall
        # is mapped LIVE). Knobs in config.yaml autonomy.explore; reuses MapStore.clearance().
        self.clearance_fan_deg = float(e.get("clearance_fan_deg", 15.0))
        self.clearance_fan_n = int(e.get("clearance_fan_n", 3))
        self.clearance_skip = float(e.get("clearance_skip", 0.25))
        self.clearance_min_count = int(e.get("clearance_min_count", 2))
        self.clearance_max_range = float(e.get("clearance_max_range", 10.0))
        # Session 28: a direction only counts as BLOCKED once at least this FRACTION of the fan's rays hit
        # something within range (0.0 = prior behavior, a single ray hit is enough) — protects against an
        # isolated, spatially-noisy voxel (still passing clearance_min_count) falsely reading an entire
        # direction as blocked on a sparse/messy reconstruction. See MapStore.clearance()'s docstring for
        # the tradeoff against the reason MIN-over-fan was originally chosen (thin/off-center wall capture).
        self.clearance_min_hit_fraction = float(e.get("clearance_min_hit_fraction", 0.0))
        # Clearance RING: clearance at headings around the drone (for the autopilot's parallax scouting).
        # Sampled at multiples of turn_step_deg so it lines up with the autopilot's turn quantization.
        self.clearance_ring_step = float(e.get("turn_step_deg", 45.0))
        # The ring feeds SHORT parallax scoots (~parallax_push_dist), so it uses a NEAR-FIELD range cap: a far
        # wall is irrelevant to "can I translate a bit this way", and capping keeps the cone a tight pencil where
        # it's consumed. The forward-cruise stand-off keeps the full clearance_max_range (it wants distant walls).
        self.ring_max_range = float(e.get("ring_max_range", 1.5))
        self._last_clearance = None       # last published forward_clearance_dist (for the report line)
        self._last_pos_y = None           # last published camera Y (altitude; +Y is DOWN)
        self._last_ring_fb = (None, None) # last (forward, backward) ring clearances (report line)
        self._sweep_logged = False       # True while the planner is touring corners (one-shot per-corner log below)
        self._sweep_target_logged = None # last corner [x,z] we logged (re-logs on each new tour corner)
        self.last_planner_event = []     # transient bump-outcome summaries set by run()'s bump drain; ride ONE
                                          # plan (a LIST, not a scalar: pipe.step() only runs — and drains this
                                          # — once per SLAM solve, which can take many seconds while SLOW; a
                                          # single overwritable slot would silently drop every message but the
                                          # last one generated in that window. See _consume_planner_event.)

        # --- diagnostic CSV logging (off unless enable_diag is called) ---
        self.diag_perf = NullLog()    # per-frame SLAM/loop timing
        self.diag_lift = NullLog()    # per-detection lift geometry + estimate evolution
        self._last_step_ts = None
        # Strictly-consecutive SLAM invocation counter (diagnostic session): increments by exactly 1 on
        # EVERY step() call, independent of the NDI-side `frame_id` (io_bridge's raw-camera-frame counter,
        # which jumps whenever the CONFLATEd frame bus drops frames while SLAM was busy). A gap in THIS
        # counter, as observed downstream in autopilot.py, proves the AUTOPILOT's own "drain the plan bus
        # to the freshest message" loop dropped a published plan -- something the NDI frame_id can't show,
        # since that only reveals camera-side skips. Rides every plan payload as "slam_seq".
        self._slam_seq = 0

    def enable_diag(self, ts=None, out_dir=None):
        """Open CSV diagnostic logs (per-frame timing + per-lift hit geometry)."""
        self.diag_perf = DiagLog("perception", [
            "wall_ts", "frame_id", "loop_dt", "slam_ms", "mode", "new_keyframe",
            "n_keyframes", "n_voxels", "reloc"], out_dir=out_dir, ts=ts)
        self.diag_lift = DiagLog("lift", [
            "wall_ts", "frame_id", "found", "bbox_area", "center_x", "center_y",
            "pose_found", "cam_x", "cam_y", "cam_z", "ray_x", "ray_y", "ray_z",
            "hit", "hit_x", "hit_y", "hit_z", "march_dist",
            "n_hits", "n_inliers", "cluster_frac", "est_x", "est_y", "est_z", "confident",
        ], out_dir=out_dir, ts=ts)

    def close_diag(self):
        self.diag_perf.close()
        self.diag_lift.close()

    def step(self, frame_bgr, meta, state_pub=None, show=True):
        # --- SLAM every frame ---
        self._slam_seq += 1   # one actual SLAM invocation, unconditionally -- see __init__ for why
        t_step = time.time()
        loop_dt = (t_step - self._last_step_ts) if self._last_step_ts else 0.0
        self._last_step_ts = t_step
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        t_slam = time.time()
        res = self.slam.process(rgb)
        slam_ms = (time.time() - t_slam) * 1000.0

        fid = meta.get("frame_id")
        if res.pose is not None and fid is not None:
            self._remember_pose(int(fid), res.pose)

        self.diag_perf.row(
            wall_ts=round(t_step, 4), frame_id=fid, loop_dt=round(loop_dt, 4),
            slam_ms=round(slam_ms, 1), mode=res.mode, new_keyframe=int(bool(res.new_keyframe)),
            n_keyframes=res.n_keyframes, n_voxels=len(self.mapstore),
            reloc=int(bool(res.reloc_event)))

        # One-time geometry sanity: the center-pixel ray (camera frame) should point forward.
        if self.debug_lift and not self._geom_logged and self.slam.ray_field is not None:
            h, w = self.slam.ray_hw
            fwd = self.slam.ray_field[h // 2, w // 2]
            print(f"[perception][debug-lift] center-pixel ray (camera frame) = "
                  f"{np.round(fwd, 4).tolist()} (expect ~[0,0,1] forward) | ray_hw={self.slam.ray_hw}",
                  flush=True)
            self._geom_logged = True

        # Trajectory: record the camera center EVERY frame so the persisted/displayed flight path is
        # dense and "remembers" the whole flight (previously add_pose was keyframe-gated → ~1 pt/kf,
        # a sparse path that froze between keyframes). Voxel integration still happens per keyframe.
        map_updated = False
        if res.camera_center is not None:
            self.mapstore.add_pose(res.camera_center)
        if res.new_keyframe and res.kf_points is not None and len(res.kf_points):
            self.mapstore.integrate(res.kf_points, res.kf_colors)
            if res.camera_center is not None:
                # Same per-keyframe data feeds the 2D free/unknown/occupied ground layer.
                self.ground.integrate(res.camera_center, res.kf_points)
            map_updated = True

        heading_deg = heading_from_pose(res.pose)

        if state_pub is not None:
            cc = res.camera_center
            state_pub.publish(frame_bus.TOPIC_POSE, {
                "tracking_mode": res.tracking_mode, "mode": res.mode,
                "n_keyframes": res.n_keyframes, "n_voxels": len(self.mapstore),
                "frame_id": meta.get("frame_id"), "sim_time": meta.get("sim_time"),
                "camera_center": [round(float(x), 4) for x in cc] if cc is not None else None,
                "heading_deg": (round(heading_deg, 2) if heading_deg is not None else None),
                "new_keyframe": res.new_keyframe, "reloc_event": res.reloc_event,
                "slam_ms": round(slam_ms, 1),
                # Forwarded from io_bridge meta so the autopilot can (a) gate the ceiling-stall on
                # commanded ascent (controls.joy_vertical) and (b) tag each log line with the
                # recording-relative frame index (rec_frame) for video correlation.
                "controls": meta.get("controls"),
                "rec_frame": meta.get("rec_frame"),
            })
            # Top-down occupancy snapshot: on a new keyframe (cells changed) OR on a timer, so the
            # DENSE trajectory reaches the visualizer without waiting for the next (sparse) keyframe.
            # Each message is a full self-contained snapshot, so a late joiner catches up immediately.
            now_mono = time.monotonic()
            if map_updated or (now_mono - self.last_map_pub) >= self.MAP_PUB_INTERVAL:
                state_pub.publish(frame_bus.TOPIC_MAP, self._map_payload(res, meta))
                self.last_map_pub = now_mono
            # Map mode: republish the explore plan (goal/bearing/done + ground layer) on a timer.
            if (now_mono - self.last_plan_pub) >= self.PLAN_PUB_INTERVAL:
                state_pub.publish(frame_bus.TOPIC_PLAN,
                                  self._plan_payload(res, meta, heading_deg, slam_ms))
                self.last_plan_pub = now_mono

        now = time.monotonic()
        if now - self.last_report >= 1.0:
            c = meta.get("controls", {}) or {}
            rc = f"{self._last_clearance:.2f}u" if self._last_clearance is not None else " -- "
            py = f"{self._last_pos_y:+.2f}" if self._last_pos_y is not None else " -- "
            rf, rb = self._last_ring_fb
            rfb = (f"{rf:.2f}" if rf is not None else "--") + "/" + (f"{rb:.2f}" if rb is not None else "--")
            print(f"[perception] SLAM {res.mode:<8} kf {res.n_keyframes:3d} | "
                  f"vox {len(self.mapstore):6d} | slam {slam_ms:5.1f} ms | "
                  f"ray_clear {rc} | y {py} | ring f/b {rfb} | "
                  f"trigger {c.get('trigger')} yaw {c.get('yaw')}")
            self.last_report = now

        # DA-V2 depth removed: no depth panel/payload. Callers get panel=None (only the map window shows).
        return res, None, None, map_updated

    def _map_payload(self, res, meta):
        """Serialize MapStore.topdown_summary() to a JSON-able TOPIC_MAP payload.

        Colors are packed to one 0xRRGGBB int per cell to keep the snapshot compact.
        """
        s = self.mapstore.topdown_summary(grid=MAP_GRID)
        rgb = s["cells_rgb"].astype(np.int32)
        packed = (rgb[:, 0] << 16) | (rgb[:, 1] << 8) | rgb[:, 2]
        return {
            "tracking_mode": s["tracking_mode"], "grid": s["grid"],
            "bounds": s["bounds"], "span_world": round(s["span_world"], 3),
            "n_voxels": s["n_voxels_kept"], "n_keyframes": res.n_keyframes,
            "cells_u": s["cells_u"].tolist(), "cells_v": s["cells_v"].tolist(),
            "cells_rgb": packed.tolist(),
            "traj_u": s["traj_u"].tolist(), "traj_v": s["traj_v"].tolist(),
            "frame_id": meta.get("frame_id"), "sim_time": meta.get("sim_time"),
        }

    # ------------------------------------------------------------- map mode planner
    def _consume_planner_event(self):
        """Pop ALL transient bump-outcome summaries queued since the last plan (set by the run() bump
        drain) so they ride EXACTLY ONE plan then clear — discrete event markers for the timeline, not a
        persistent field. Joined into one string (";"-separated) rather than a single overwritable slot:
        when a SLAM solve is slow, run()'s drain loop can process several bump/pick/loop events before
        this is next called, and a scalar mailbox would silently keep only the LAST one (the exact "0
        strikes to blacklisted with nothing in between" symptom diagnosed off the 20260718 flight)."""
        evs, self.last_planner_event = self.last_planner_event, []
        return "; ".join(evs) if evs else None

    def _plan_payload(self, res, meta, heading_deg, slam_ms=None):
        """TOPIC_PLAN payload: drone pose (X-Z + heading), the chosen frontier goal + bearing, the
        done flag, and a compact ground-grid raster for the visualizer. NO SILENT FALLBACK: if SLAM
        is not TRACKING (or pose/heading missing) the plan is published with plan_valid=false and NO
        goal, so the autopilot holds instead of chasing a stale target. `slam_ms` (this frame's SLAM
        build time) rides on EVERY plan — even when invalid — so the autopilot's SLAM settle gate can
        watch it (a healthy solve is sub-second; a choke spikes it) independent of tracking state.
        Unreachable-goal blacklisting is EVENT-DRIVEN (planner.note_wall_hit, fed by the autopilot's
        advance-blocked bump pulses in run()) — NOT computed here per-frame."""
        cc = res.camera_center
        pos = [float(cc[0]), float(cc[2])] if cc is not None else None
        valid = (res.mode == "TRACKING") and pos is not None and heading_deg is not None
        payload = {
            "plan_valid": bool(valid), "mode": res.mode, "tracking_mode": res.tracking_mode,
            # Strictly-consecutive SLAM invocation counter (diagnostic session, see __init__/step) --
            # distinct from `frame_id` (the NDI raw-camera-frame counter, below): a gap in THIS one proves
            # the autopilot's own plan-bus drain dropped a published plan, not a camera-frame skip.
            "slam_seq": self._slam_seq,
            "pos": ([round(pos[0], 4), round(pos[1], 4)] if pos else None),
            "heading_deg": (round(heading_deg, 2) if heading_deg is not None else None),
            "goal": None, "bearing_deg": None, "bearing_err": None,
            "n_frontiers": 0, "done": False, "forward_clearance_dist": None,
            "n_blacklisted": len(self.planner._blacklist), "blacklist": self.planner.blacklist_points(),
            "blacklist_permanent": self.planner.blacklist_permanent(),
            # Live 2-bump counter (rides EVERY plan, valid or not, so the autopilot timeline always has it) +
            # a TRANSIENT planner_event: the last bump receipt's summary, emitted on the FIRST plan after that
            # bump then cleared, so the goal-change reset / blacklist shows as a discrete timeline event.
            "wall_hit_count": self.planner.wall_hit_count, "wall_hit_goal": self.planner.wall_hit_goal,
            "planner_event": self._consume_planner_event(),
            "pos_y": None, "clearance_ring": None,
            "slam_ms": (round(float(slam_ms), 1) if slam_ms is not None else None),
            # Camera-capture monotonic timestamp (io_bridge stamps meta["mono_ts"] = time.monotonic() at grab;
            # same clock domain the autopilot issues commands on) — rides EVERY plan, valid or not. Serves BOTH
            # the paired SLAM START/FINISH replay records (frame ingress = cap_ts, done = cap_ts + slam_ms) and
            # the height-calibration settlement gate (a frame CAPTURED >= gate_s after DESCEND went out).
            "cap_ts": meta.get("mono_ts"),
            "frame_id": meta.get("frame_id"), "sim_time": meta.get("sim_time"),
            "ground": self.ground.summary(raster=self.GROUND_RASTER),
        }
        if not valid:
            return payload
        # Distance to the nearest mapped wall straight ahead (a fan of ground-plane rays into the voxel
        # map). Only needs pose+heading (independent of whether a goal exists). None = nothing mapped
        # within range ahead -> the autopilot leans on the flow contact detector instead.
        clr = self.mapstore.clearance(cc, heading_deg, fan_deg=self.clearance_fan_deg,
                                      fan_n=self.clearance_fan_n, skip=self.clearance_skip,
                                      min_count=self.clearance_min_count, max_range=self.clearance_max_range,
                                      min_hit_fraction=self.clearance_min_hit_fraction)
        payload["forward_clearance_dist"] = (round(float(clr), 4) if clr is not None else None)
        self._last_clearance = payload["forward_clearance_dist"]
        # Camera altitude for the autopilot's altitude lock. World frame is camera-convention +Y DOWN
        # (map_store.py), so a SINKING drone has an INCREASING pos_y — the autopilot corrects on that sign.
        payload["pos_y"] = round(float(cc[1]), 4)
        self._last_pos_y = payload["pos_y"]
        # Clearance ring: nearest mapped obstacle at headings around the drone (multiples of the turn step),
        # so the autopilot can check the intended turn heading + pick a roomier axis for a parallax push.
        step = self.clearance_ring_step
        n = max(1, int(round(360.0 / step)))
        ring = []
        for i in range(n):
            relw = ((i * step + 180.0) % 360.0) - 180.0     # wrap each offset to (-180, 180]
            d = self.mapstore.clearance(cc, heading_deg + i * step, fan_deg=self.clearance_fan_deg,
                                        fan_n=self.clearance_fan_n, skip=self.clearance_skip,
                                        min_count=self.clearance_min_count, max_range=self.ring_max_range,
                                        min_hit_fraction=self.clearance_min_hit_fraction)
            ring.append([round(relw, 1), (round(float(d), 4) if d is not None else None)])
        payload["clearance_ring"] = ring
        # Session 29: the raw ray-hit picture at the 4 cardinal directions (same params the ring/TRIM/
        # PARALLAX_PUSH/FALLBACK actually consult — ring_max_range, not the longer forward-cruise range) —
        # for the replay debugger's Clearance tab, so a "ring blocked" judgment is auditable at a glance
        # instead of re-derived from raw voxel data by hand after the fact.
        cd = {}
        for tag, off in (("fwd", 0.0), ("back", 180.0), ("left", -90.0), ("right", 90.0)):
            cd[tag] = self.mapstore.clearance(cc, heading_deg + off, fan_deg=self.clearance_fan_deg,
                                              fan_n=self.clearance_fan_n, skip=self.clearance_skip,
                                              min_count=self.clearance_min_count, max_range=self.ring_max_range,
                                              min_hit_fraction=self.clearance_min_hit_fraction, detail=True)
        payload["clearance_detail"] = cd

        def _ring_fb(target):                                # nearest-offset lookup (forward=0, backward=180)
            best, bd = None, 1e9
            for r, dd in ring:
                diff = abs(((target - r + 180.0) % 360.0) - 180.0)
                if diff < bd:
                    bd, best = diff, dd
            return best
        self._last_ring_fb = (_ring_fb(0.0), _ring_fb(180.0))
        # Goal selection + ALL-CORNERS verification tour. The inset bbox corners (`bbox_corners`) are needed
        # whenever NOTHING is reachable — no frontiers at all OR every live frontier blacklisted (the
        # glass-loop escape). The planner TOURS them (opposite corner first, then farthest-unvisited, then
        # last) so every room corner reconstructs densely; on arrival at each it clears the round's soft
        # blacklist so those goals get one retry. Each corner is inset from the bbox edge by `reposition_inset`
        # so it stays reachable inside the forward stand-off shell.
        fr = self.ground.frontiers()
        # Compute on EVERY not-reachable tick (not just the transition): when the 2-bump rule retires the
        # corner we're touring toward, `select` needs the corner list in hand that same tick to advance.
        # `select` still caches each corner target ONCE, so it stays STATIC while flying to it.
        reachable = self.planner.any_reachable(fr)
        corners = (self.ground.bbox_corners(inset=self.reposition_inset) if not reachable else None)
        # Session 24: scale the autopilot's far-corner blacklist-exemption distance with the ROOM instead of
        # a flat constant -- half the largest pairwise distance among the known corners (the true diagonal in
        # the normal 4-corner case; degrades gracefully for a collapsed corridor/tiny-box case). None with
        # fewer than 2 corners (no meaningful diagonal yet) -- the autopilot falls back to its config default.
        span_half = None
        if corners and len(corners) >= 2:
            span_half = 0.5 * max(math.hypot(a[0] - b[0], a[1] - b[1])
                                   for i, a in enumerate(corners) for b in corners[i + 1:])
        payload["corner_span_half"] = span_half
        # Goal selection (blacklisting is event-driven via note_wall_hit in run(), NOT a per-select timer).
        goal, n_frontiers, done = self.planner.select(fr, pos, heading_deg, sweep_corners=corners)
        payload["n_blacklisted"] = len(self.planner._blacklist)
        payload["blacklist"] = self.planner.blacklist_points()
        payload["blacklist_permanent"] = self.planner.blacklist_permanent()
        payload["goal_clearance_ok"] = bool(self.planner.clearance_ok)   # visible flag: clearance inset succeeded
        # A published corner goal (sweep tour) is flagged so the autopilot can SUPPRESS a bump/strike against a
        # FAR corner (a mildly-stuck drone must not blacklist a corner it hasn't approached — session 20).
        payload["goal_is_corner"] = bool(self.planner.sweeping)
        # Session 24: True once ANY corner was force-retired via a give-up (never reached/2-bump-confirmed).
        # Meaningful once `done` is also True: the autopilot's REPLAN distinguishes a genuinely-exhausted
        # mission (every corner reached/confirmed -> graceful RETURN_TO_ORIGIN) from a stuck one (at least one
        # corner simply abandoned -> a hard STUCK hold instead).
        payload["corner_giveup_stuck"] = bool(self.planner._gave_up_corner)
        # Persistent goals DB (per-disc picks / strikes / blacklisted) -> the replay debugger's floating table.
        # DB-blacklist events (loop/stall) are logged in the run() drain that feeds the DB, not here.
        payload["goal_db"] = self.planner.goal_db_snapshot()
        tgt = self.planner.sweep_target
        if self.planner.sweeping and tgt is not None and tgt != self._sweep_target_logged:
            n_left = sum(1 for c in (corners or []) if not self.planner._corner_visited(c))
            print(f"[perception] planner: touring room corners -> SWEEPING to corner {tgt} "
                  f"({len(self.planner._swept_corners)} visited, {n_left} left)", flush=True)
            self._sweep_target_logged = list(tgt)
            self._sweep_logged = True
        elif not self.planner.sweeping and self._sweep_logged:
            print(f"[perception] planner: corner tour {'COMPLETE -> done' if done else 'cleared -> frontiers found'}",
                  flush=True)
            self._sweep_logged = False
            self._sweep_target_logged = None
        payload["n_frontiers"], payload["done"] = n_frontiers, done
        if goal is not None:
            payload["goal"] = [round(float(goal[0]), 4), round(float(goal[1]), 4)]
            bearing = math.degrees(math.atan2(goal[0] - pos[0], goal[1] - pos[1]))
            payload["bearing_deg"] = round(bearing, 2)
            payload["bearing_err"] = round(_wrap180(bearing - heading_deg), 2)
        return payload

    # ------------------------------------------------------------- target lift
    def _remember_pose(self, fid: int, pose: np.ndarray):
        if fid not in self._pose_hist:
            self._pose_keys.append(fid)
        self._pose_hist[fid] = pose
        while len(self._pose_keys) > self.POSE_HIST_MAX:
            self._pose_hist.pop(self._pose_keys.pop(0), None)

    def _pose_for(self, fid):
        """Pose of frame `fid`, or the nearest remembered frame (detections lag their frame)."""
        if fid is None or not self._pose_keys:
            return None
        if fid in self._pose_hist:
            return self._pose_hist[fid]
        nearest = min(self._pose_keys, key=lambda k: abs(k - fid))
        return self._pose_hist[nearest]

    def ingest_detection(self, det: dict):
        """Back-project a TOPIC_DETECTION center pixel into the voxel map → a target hit.

        Returns (hit_world (3,), distance) when the ray hits a map voxel, else None. Feeds the
        running TargetEstimator either way (a 'found but no map hit' is recorded as a miss).
        """
        if not det or not det.get("found"):
            return None
        self.n_det_seen += 1
        if not self.estimator.label and det.get("target_label"):
            self.estimator.label = det["target_label"]
        fid, center = det.get("frame_id"), det.get("center")
        bbox = det.get("bbox")
        bbox_area = (round((bbox[2] - bbox[0]) * (bbox[3] - bbox[1]), 1)
                     if isinstance(bbox, (list, tuple)) and len(bbox) == 4 else "")

        def _log(pose_found, hit_flag, cam=None, ray=None, hw=None, dist=None):
            """One lift.csv row per detection — logs cam+hit so true cam->hit distance + the
            bbox size (detection reliability proxy) can be correlated against the cluster offline."""
            e = self.estimator.estimate() or {}
            pos = e.get("position") or [None, None, None]
            g = lambda a, i: (round(float(a[i]), 4) if a is not None else "")
            self.diag_lift.row(
                wall_ts=round(time.time(), 4), frame_id=fid, found=1, bbox_area=bbox_area,
                center_x=(round(center[0], 1) if center else ""),
                center_y=(round(center[1], 1) if center else ""),
                pose_found=int(pose_found),
                cam_x=g(cam, 0), cam_y=g(cam, 1), cam_z=g(cam, 2),
                ray_x=g(ray, 0), ray_y=g(ray, 1), ray_z=g(ray, 2), hit=int(hit_flag),
                hit_x=g(hw, 0), hit_y=g(hw, 1), hit_z=g(hw, 2),
                march_dist=(round(float(dist), 4) if dist is not None else ""),
                n_hits=self.estimator.n_hits, n_inliers=e.get("n_inliers", ""),
                cluster_frac=e.get("cluster_frac", ""),
                est_x=pos[0], est_y=pos[1], est_z=pos[2],
                confident=(int(bool(e.get("confident"))) if e else ""))

        pose = self._pose_for(fid)
        if pose is None or center is None or self.slam.ray_field is None:
            self.estimator.add_found_no_hit(fid)
            _log(pose_found=False, hit_flag=False)
            return None

        # Detection center is in transport pixels (proc_w x proc_h); map it onto the ray field.
        h, w = self.slam.ray_hw
        u = int(np.clip(round(center[0] * (w - 1) / max(self.proc_w - 1, 1)), 0, w - 1))
        v = int(np.clip(round(center[1] * (h - 1) / max(self.proc_h - 1, 1)), 0, h - 1))
        ray_cam = self.slam.ray_field[v, u].astype(np.float64)
        ray_world = pose[:3, :3].astype(np.float64) @ ray_cam     # Sim3 scale cancels on normalize
        rd = ray_world / (np.linalg.norm(ray_world) + 1e-9)
        cam = pose[:3, 3]
        hit = self.mapstore.raycast(
            cam, ray_world, min_count=self.TARGET_MIN_COUNT, skip=self.TARGET_SKIP)
        if hit is None:
            if self.debug_lift:
                print(f"[perception][debug-lift] frame {fid} px=({u},{v}) MISS "
                      f"(ray hit no voxel; cam={np.round(cam,2).tolist()})", flush=True)
            self.estimator.add_found_no_hit(fid)
            _log(pose_found=True, hit_flag=False, cam=cam, ray=rd)
            return None
        center_world, dist = hit
        self.estimator.add(center_world, fid)
        if self.debug_lift:
            print(f"[perception][debug-lift] frame {fid} px=({u},{v}) cam={np.round(cam,2).tolist()} "
                  f"ray={np.round(rd,3).tolist()} -> hit={np.round(center_world,3).tolist()} @ {dist:.2f}u "
                  f"| n_hits={self.estimator.n_hits}", flush=True)
        _log(pose_found=True, hit_flag=True, cam=cam, ray=rd, hw=center_world, dist=dist)
        return center_world, dist

    def target_payload(self):
        """TOPIC_TARGET payload — a LIST of target instances (the object can appear more than once),
        sorted by support; or None if nothing is localized yet. Each instance carries its own
        position + uncertainty + counts + `confident` flag."""
        ests = self.estimator.estimate_all()
        if not ests:
            return None
        for e in ests:
            e["tracking_mode"] = self.slam.tracking_mode
            e["voxel_size"] = self.voxel_size
            e["min_count"] = self.TARGET_MIN_COUNT
        return {
            "targets": ests,
            "n_targets": len(ests),
            "label": ests[0].get("label"),
            "tracking_mode": self.slam.tracking_mode,
        }


# ==============================================================================
# Live loop (frame bus) and offline loop (recorded mp4)
# ==============================================================================
def _show_and_quit(panel, pipe, map_updated, show):
    """Render the top-down map window and return True if the user pressed 'q'. (`panel` is always
    None since the DA-V2 depth panel was removed; kept in the signature for call-site symmetry.)"""
    if not show:
        return False
    if map_updated:
        cv2.imshow(MAP_WINDOW, pipe.mapstore.render_topdown(size=600, point_px=2, min_count=1))
    return (cv2.waitKey(1) & 0xFF) == ord("q")


def run_live(cfg, show=True, conf_thresh=1.5, debug_lift=False, log=False, stop_file=None):
    from datetime import datetime
    from pathlib import Path
    frame_port = cfg["network"]["frame_bus_port"]
    pstate_port = cfg["network"]["perception_state_port"]
    obj_port = cfg["network"]["object_state_port"]
    ctrl_port = cfg["network"]["autonomy_control_port"]
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")   # shared by the diag CSVs + the shutdown map export
    out_dir = Path(REPO) / "OUTPUT" / "diag"
    out_dir.mkdir(parents=True, exist_ok=True)
    pipe = Pipeline(cfg, conf_thresh=conf_thresh, debug_lift=debug_lift)
    if log:
        pipe.enable_diag(ts=ts)
    frame_sub = frame_bus.FrameSubscriber(frame_port)
    state_pub = frame_bus.StatePublisher(pstate_port)  # binds; fail-fast if taken
    # SUB to object_worker's detections (lazy connect — fine whether or not it's running yet).
    det_sub = frame_bus.StateSubscriber(obj_port, topics=[frame_bus.TOPIC_DETECTION])
    # SUB to the autopilot's advance-blocked BUMP pulses (event-driven 2-bump blacklist). Lazy connect;
    # deduped by seq so a republished pulse is applied once. Feeds planner.note_wall_hit UNCONDITIONALLY
    # (never gated on SLAM health — the whole point of the event-driven design).
    apevent_sub = frame_bus.StateSubscriber(ctrl_port, topics=[frame_bus.TOPIC_AUTOPILOT_EVENT])
    last_bump_seq = -1
    last_pick_seq = -1
    last_giveup_seq = -1
    print(f"[perception] frame bus SUB :{frame_port} | state PUB :{pstate_port} "
          f"(TOPIC_POSE/MAP/PLAN/TARGET) | detection SUB :{obj_port}")
    print(f"[perception] SLAM every frame ({pipe.slam.tracking_mode}); depth removed (SLAM owns the GPU)")
    print("[perception] === READY === waiting for frames from io_bridge "
          "(focus a window, 'q' to quit).\n")
    try:
        while True:
            # Graceful-stop sentinel (mirrors autopilot.py's _FileStopEvent): a launcher that hard-
            # terminates a CREATE_NEW_CONSOLE child on Windows skips `finally` entirely, so a polled
            # file is the reliable way to let this loop exit NORMALLY and run the shutdown map export
            # below. Checked every iteration, independent of whether a frame arrived this tick.
            if stop_file is not None and os.path.exists(stop_file):
                print("[perception] stop-file seen -> shutting down cleanly")
                break
            got = frame_sub.recv(timeout_ms=500)
            if got is None:
                if show and (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
                continue
            frame, meta = got

            # Drain autopilot BUMP pulses -> event-driven 2-bump blacklist (before pipe.step publishes the
            # next plan, so a fresh blacklist is reflected immediately). Deduped by seq.
            ap = apevent_sub.recv(timeout_ms=0)
            while ap is not None:
                ev = ap[1]
                seq, bg = ev.get("seq"), ev.get("bump_goal")
                if bg is not None and seq is not None and seq != last_bump_seq:
                    last_bump_seq = seq
                    out = pipe.planner.note_wall_hit(bg, pos=ev.get("bump_pos"),
                                                     is_corner=bool(ev.get("bump_is_corner")))
                    # Log EVERY bump receipt (not just blacklists) so the counter's climb AND its resets are
                    # visible in perception stdout — a goal-change reset is the mechanism that defeats the
                    # blacklist, and it was previously silent. `pipe.last_planner_event` rides the next
                    # TOPIC_PLAN so the autopilot timeline captures the same transition.
                    g = [round(out["goal"][0], 3), round(out["goal"][1], 3)]
                    if out["action"] == "blacklist":
                        msg = (f"BUMP goal={g} count=2/2 -> BLACKLIST PERMANENT "
                               f"({len(pipe.planner._blacklist)} total) -> reselecting")
                    elif out["action"] == "reset":
                        pg = [round(out["prev_goal"][0], 3), round(out["prev_goal"][1], 3)]
                        msg = f"BUMP goal={g} count=1/2 (RESET from prev goal {pg} -> counter defeated)"
                    elif out["action"] == "arm":
                        msg = f"BUMP goal={g} count=1/2 (armed; one more same-goal bump blacklists)"
                    else:  # increment (reached 1 already, now higher but < threshold — unreachable in a 2-gate)
                        msg = f"BUMP goal={g} count={out['count']}/2 (increment)"
                    pipe.last_planner_event.append(msg)
                    print(f"[perception] planner: {msg}", flush=True)
                # Session 24: a far-corner give-up escalation (corner_giveup_limit strikes, never once close
                # enough for a real 2-bump) -> force-retire that corner (mark visited, tour moves on). Never
                # blacklists/ends the mission by itself -- see planner._gave_up_corner + the autopilot's REPLAN
                # `done` branch for the all-corners-exhausted STUCK ending.
                gseq, gg = ev.get("giveup_seq"), ev.get("corner_giveup_goal")
                if gg is not None and gseq is not None and gseq != last_giveup_seq:
                    last_giveup_seq = gseq
                    pipe.planner.force_retire_corner(gg)
                    ggr = [round(gg[0], 3), round(gg[1], 3)]
                    gmsg = f"CORNER-GIVEUP goal={ggr} -> force-retired (never reached; tour advances)"
                    pipe.last_planner_event.append(gmsg)
                    print(f"[perception] planner: {gmsg}", flush=True)
                # Goals-DB pick + previous-hop STRIKE/progress outcome (one pulse per leg). Feed the STALL guard
                # (register_hop_outcome) then the CIRCLING guard (register_goal_pick); log any DB-blacklist.
                # INDEPENDENT parts (session 21): a re-calibration REPLAN emits a hop-outcome-ONLY pulse
                # (pick_goal=None) — the strike/progress still registers; the pick registers post-calib.
                pseq, pg = ev.get("pick_seq"), ev.get("pick_goal")
                if pseq is not None and pseq != last_pick_seq:
                    last_pick_seq = pseq
                    prev_goal = ev.get("prev_goal")
                    if prev_goal is not None:
                        pipe.planner.register_hop_outcome(prev_goal, bool(ev.get("prev_progressed")),
                                                          bool(ev.get("prev_strike_eligible", True)),
                                                          pos=ev.get("judge_pos"),
                                                          slam_ms=ev.get("judge_slam_ms"),
                                                          is_corner=bool(ev.get("prev_is_corner")))
                    if pg is not None:
                        # Defense in depth (NOT a fallback -- just visibility): a genuinely new pick landing on
                        # an ALREADY-excluded goal would silently defeat the loop/2-bump/stall blacklist (this
                        # is exactly the bug found off flight 20260721_005658 -- a clearance-inset candidate
                        # collapsing onto a dead disc kept re-picking it 49x after it was permanently
                        # blacklisted at pick 3, since exclusion was only ever checked pre-inset). The fix lives
                        # in frontier_planner._select_reachable (re-checks _excluded AFTER the inset), so this
                        # should never fire again -- if it ever does, surface it loudly as a structured
                        # planner_event (console + the timeline/flight_replay debugger), not a silent no-op.
                        if pipe.planner.is_excluded(pg):
                            wg = [round(float(pg[0]), 3), round(float(pg[1]), 3)]
                            wmsg = f"WARNING: pick landed on an ALREADY-excluded goal={wg} -> blacklist bypassed"
                            pipe.last_planner_event.append(wmsg)
                            print(f"[perception] planner: {wmsg}", flush=True)
                        pipe.planner.register_goal_pick(pg, ev.get("pick_pos"),
                                                        slam_ms=ev.get("judge_slam_ms"))
                    lev = pipe.planner.last_loop_event
                    if lev is not None:
                        lg = [round(lev["goal"][0], 3), round(lev["goal"][1], 3)]
                        tag = "STRIKE-BLACKLIST" if lev.get("reason") == "stall" else "LOOP-BLACKLIST"
                        extra = (f"strikes={lev['strikes']}" if lev.get("reason") == "stall"
                                 else f"picks={lev['picks']}")
                        msg = f"{tag} goal={lg} {extra} ({len(pipe.planner._blacklist)} total) -> reselecting"
                        pipe.last_planner_event.append(msg)
                        print(f"[perception] planner: {msg}", flush=True)
                        pipe.planner.last_loop_event = None
                ap = apevent_sub.recv(timeout_ms=0)

            _, _, panel, map_updated = pipe.step(frame, meta, state_pub, show)

            # Drain any target detections and lift them into the map.
            d = det_sub.recv(timeout_ms=0)
            while d is not None:
                hit = pipe.ingest_detection(d[1])
                if hit is not None:
                    e = pipe.estimator.estimate()
                    epos = e["position"] if e else "(<min instance)"
                    print(f"[perception] target hit {np.round(hit[0], 3).tolist()} "
                          f"@ {hit[1]:.2f}u | best {epos} | n_hits={pipe.estimator.n_hits}", flush=True)
                d = det_sub.recv(timeout_ms=0)
            now = time.monotonic()
            tp = pipe.target_payload()
            if tp is not None and (now - pipe.last_target_pub) >= 0.5:
                state_pub.publish(frame_bus.TOPIC_TARGET, tp)
                pipe.last_target_pub = now

            if _show_and_quit(panel, pipe, map_updated, show):
                break
    except KeyboardInterrupt:
        pass
    finally:
        print("[perception] shutting down ...")
        # Export the fused SLAM map -- same three calls run_offline_video() already makes (proven
        # there), just <ts>-prefixed into OUTPUT/diag/ instead of <stem>-prefixed into OUTPUT/. Only
        # reachable if this loop exits NORMALLY (the 'q' key, Ctrl+C in this console, or the
        # stop-file above) -- a hard TerminateProcess skips this, same caveat as autopilot's own
        # shutdown-emitted report.
        ests = pipe.estimator.estimate_all()
        targets = [e["position"] for e in ests] if ests else None
        png = out_dir / f"{ts}_livemap_topdown.png"
        pipe.mapstore.render_topdown(png, min_count=2, targets=targets)
        pipe.mapstore.save_npz(out_dir / f"{ts}_livemap.npz", min_count=2)
        ply = out_dir / f"{ts}_livemap.ply"
        pipe.mapstore.save_ply(ply, min_count=2, trajectory=True, targets=targets)
        print(f"[perception] top-down (flight path + target marks) -> {png}")
        print(f"[perception] voxel map -> {out_dir / f'{ts}_livemap.npz'}")
        print(f"[perception] point cloud + flight path + targets (.ply, Blender-loadable) -> {ply}")
        pipe.close_diag()
        frame_sub.close()
        state_pub.close()
        det_sub.close()
        apevent_sub.close()
        if show:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass


def _video_frames(path, stride, max_frames, proc_w, proc_h, object_frame_h=720):
    """Yield (small_512x288, hires, meta) from an mp4, sub-sampled — mirrors io_bridge's two
    streams. `hires` is the native frame downscaled to `object_frame_h` (no upscale), for the
    object detector; perception uses the 512x288 `small`."""
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise RuntimeError(f"could not open recording: {path}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    src_idx = yielded = 0
    while True:
        ret, bgr = cap.read()
        if not ret:
            break
        if src_idx % stride == 0:
            small = cv2.resize(bgr, (proc_w, proc_h), interpolation=cv2.INTER_AREA)
            sh, sw = bgr.shape[:2]
            if sh > object_frame_h:
                ow = int(round(sw * object_frame_h / sh))
                hires = cv2.resize(bgr, (ow, object_frame_h), interpolation=cv2.INTER_AREA)
            else:
                hires = bgr
            meta = {"frame_id": yielded, "mono_ts": time.monotonic(),
                    "sim_time": round(src_idx / fps, 3), "controls": {}}
            yield small, hires, meta
            yielded += 1
            if max_frames and yielded >= max_frames:
                break
        src_idx += 1
    cap.release()


def run_offline_video(cfg, video, show=False, stride=3, max_frames=0,
                      out_dir=None, conf_thresh=1.5, publish=False,
                      detect=False, detect_every=5, debug_lift=False, log=False):
    """M4 offline verification: drive the full SLAM+map pipeline from a recorded mp4, export the map.

    With `publish=True` it ALSO publishes TOPIC_POSE/MAP/PLAN on the perception state bus,
    so `visualizer.py` can be exercised against a recording with no hardware/NDI. Default
    off so a plain export run stays self-contained and never collides with a live worker.

    With `detect=True` it ALSO loads Qwen (object_worker) and runs the full object chain in
    THIS process — detection every `detect_every` frames, back-projected into the map and
    aggregated — so the 3D-lift end-to-end can be verified offline (frame_ids align because a
    single loop owns both). Note: single-process, so this does NOT test live VRAM coexistence.
    """
    import json
    from pathlib import Path
    # Resolve to absolute BEFORE Pipeline()/SlamEngine chdir's into the SLAM repo.
    video = Path(video).resolve()
    assert video.exists(), f"recording not found: {video}"
    out_dir = Path(out_dir or os.path.join(REPO, "OUTPUT")).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    proc_w = cfg["perception"]["processing_width"]
    proc_h = cfg["perception"]["processing_height"]
    object_frame_h = int(cfg["perception"].get("object_frame_height", 720))

    pipe = Pipeline(cfg, conf_thresh=conf_thresh, debug_lift=debug_lift)
    if log:
        pipe.enable_diag()
    # Optional in-process object detector (offline E2E lift test).
    obj_pipe = None
    if detect:
        import object_worker
        obj_pipe = object_worker.Pipeline(cfg)
        obj_pipe.min_interval = 0.0   # cadence is governed by detect_every here, not wall-clock
        print(f"[perception] OFFLINE --detect: {obj_pipe.detector.object_mode} target "
              f"'{obj_pipe.label}' [{obj_pipe.asset_class}] every {detect_every} frames -> 3D lift")
    # Offline mode is self-contained by default: it builds + exports the map and does NOT
    # touch the state bus. --publish opts into the live bus to drive the visualizer offline.
    state_pub = None
    if publish:
        state_pub = frame_bus.StatePublisher(cfg["network"]["perception_state_port"])
        print(f"[perception] OFFLINE --publish: state bus PUB "
              f":{state_pub.port} (TOPIC_POSE+MAP+PLAN+TARGET) for visualizer.py")
    print(f"[perception] OFFLINE video={video.name} stride={stride} "
          f"max_frames={max_frames or 'all'} | exporting to {out_dir}")
    print("[perception] === READY === processing recording (SLAM + map).\n")

    n = 0
    t0 = time.time()
    try:
        for frame, hires, meta in _video_frames(video, stride, max_frames, proc_w, proc_h,
                                                object_frame_h):
            _, _, panel, map_updated = pipe.step(frame, meta, state_pub, show)
            n += 1
            if obj_pipe is not None and (n % detect_every == 0):
                det_payload, _ = obj_pipe.step(hires, meta, None, show=False)
                if det_payload is not None:
                    hit = pipe.ingest_detection(det_payload)
                    if hit is not None:
                        est = pipe.estimator.estimate()
                        epos = est["position"] if est else "(<min instance)"
                        print(f"[perception]   target hit {np.round(hit[0],3).tolist()} "
                              f"@ {hit[1]:.2f}u -> best {epos} n_hits={pipe.estimator.n_hits}",
                              flush=True)
                    if state_pub is not None:
                        tp = pipe.target_payload()
                        if tp is not None:
                            state_pub.publish(frame_bus.TOPIC_TARGET, tp)
            if _show_and_quit(panel, pipe, map_updated, show):
                print("[perception] interrupted by user")
                break
    except KeyboardInterrupt:
        print("[perception] interrupted — exporting what we have ...")

    dt = time.time() - t0
    print(f"\n[perception] DONE: {n} frames in {dt:.1f}s ({n/max(dt,1e-6):.1f} fps) | "
          f"{pipe.slam.n_keyframes} keyframes | reloc {pipe.slam.n_reloc} | "
          f"peak VRAM {torch.cuda.max_memory_allocated()/1e9:.2f} GB")
    print(f"[perception] map: {pipe.mapstore.stats(min_count=2)}")

    stem = video.stem
    targets = None
    if obj_pipe is not None:
        ests = pipe.estimator.estimate_all()
        if ests:
            targets = [e["position"] for e in ests]
            report = {"label": ests[0].get("label"), "n_targets": len(ests), "targets": ests}
            with open(out_dir / f"{stem}_target.json", "w", encoding="utf-8") as f:
                json.dump(report, f, indent=2)
            print(f"[perception] {len(ests)} TARGET instance(s) of '{ests[0].get('label')}':")
            for k, e in enumerate(ests):
                print(f"[perception]   #{k} @ {e['position']} | inliers {e['n_inliers']}/{e['n_hits']}"
                      f" | radial_rms {e['radial_rms']}u spread_p90 {e['spread_p90']}u "
                      f"confident={e['confident']}")
            print(f"[perception] target report -> {out_dir / f'{stem}_target.json'}")
        else:
            print("[perception] TARGET: no map hits (target never lifted)")

    png = out_dir / f"{stem}_livemap_topdown.png"
    pipe.mapstore.render_topdown(png, min_count=2, targets=targets)
    pipe.mapstore.save_npz(out_dir / f"{stem}_livemap.npz", min_count=2)
    ply = out_dir / f"{stem}_livemap.ply"
    pipe.mapstore.save_ply(ply, min_count=2, trajectory=True, targets=targets)
    print(f"[perception] top-down (flight path + target marks) -> {png}")
    print(f"[perception] voxel map -> {out_dir / f'{stem}_livemap.npz'}")
    print(f"[perception] point cloud + flight path + targets (.ply) -> {ply}")
    # Map-mode ground layer: free/unknown/occupied + frontier centroids + last committed goal.
    gpng = out_dir / f"{stem}_groundgrid.png"
    pipe.ground.render_overlay(gpng, goal=pipe.planner.committed_goal)
    fr = pipe.ground.frontiers()
    print(f"[perception] ground grid (free/unknown/occ/frontier) -> {gpng} "
          f"| {len(fr)} frontier cluster(s), {len(pipe.ground)} cells")
    pipe.close_diag()
    if state_pub is not None:
        state_pub.close()
    if show:
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass
    print("[perception] OK")


# ==============================================================================
# Offline self-test (no bus / no sim / no GPU) — proves the module builds after the depth removal.
# ==============================================================================
def run_self_test(cfg):
    """Smoke test after the DA-V2 depth removal (2026-07-07): the module imports cleanly and the
    pure-numpy map-mode pieces (GroundGrid + FrontierPlanner) construct from config with NO depth
    model and NO GPU. The full SLAM + map pipeline is exercised by the offline `--video` path."""
    g = GroundGrid(cfg)
    p = FrontierPlanner(cfg)
    assert g is not None and p is not None
    assert len(g) == 0 and p.committed_goal is None
    print("[perception][self-test] depth removed; GroundGrid + FrontierPlanner construct OK (no GPU/no depth).")
    print("[perception][self-test] full SLAM+map path -> use: perception_worker.py --video <mp4> --no-display")
    print("[perception][self-test] PASS")


def main():
    parser = argparse.ArgumentParser(description="Cartographer perception_worker (P2): MASt3R-SLAM + map")
    parser.add_argument("--config", default=None)
    parser.add_argument("--no-display", action="store_true", help="headless: skip the OpenCV windows")
    parser.add_argument("--self-test", action="store_true",
                        help="no-GPU smoke test: module + GroundGrid/FrontierPlanner build (depth removed)")
    parser.add_argument("--video", default=None,
                        help="OFFLINE: drive the full SLAM+map pipeline from this mp4, export the map")
    parser.add_argument("--stride", type=int, default=3, help="offline: process every Nth source frame")
    parser.add_argument("--max-frames", type=int, default=0, help="offline: cap processed frames (0=all)")
    parser.add_argument("--conf-thresh", type=float, default=1.5,
                        help="per-point confidence cutoff for pointmaps fed into the map")
    parser.add_argument("--out", default=None, help="offline: output dir (default: OUTPUT/)")
    parser.add_argument("--publish", action="store_true",
                        help="offline: also publish TOPIC_POSE/MAP/PLAN/TARGET on the state bus "
                             "(drives visualizer.py from a recording, no hardware)")
    parser.add_argument("--detect", action="store_true",
                        help="offline: also run Qwen target detection + 3D lift in-process "
                             "(E2E object-chain test; exports <stem>_target.json + marks the map)")
    parser.add_argument("--detect-every", type=int, default=5,
                        help="offline --detect: run a detection every Nth processed frame")
    parser.add_argument("--debug-lift", action="store_true",
                        help="log per-detection lift geometry (pixel, cam, ray, hit) + a one-time "
                             "center-pixel ray sanity check")
    parser.add_argument("--log", action="store_true",
                        help="write diagnostic CSVs to OUTPUT/diag/ (per-frame SLAM/loop timing + "
                             "per-lift hit geometry) for live-flight debugging")
    parser.add_argument("--stop-file", default=None,
                        help="live: path to a sentinel file; when it appears, exit the loop CLEANLY "
                             "(runs the shutdown map/point-cloud export) instead of being hard-"
                             "terminated by a launcher. Mirrors autopilot.py's --stop-file.")
    args = parser.parse_args()

    cfg = load_config(args.config)
    if args.self_test:
        run_self_test(cfg)
    elif args.video:
        run_offline_video(cfg, args.video, show=not args.no_display, stride=args.stride,
                          max_frames=args.max_frames, out_dir=args.out,
                          conf_thresh=args.conf_thresh, publish=args.publish,
                          detect=args.detect, detect_every=args.detect_every,
                          debug_lift=args.debug_lift, log=args.log)
    else:
        # A stale sentinel from a crashed prior run would stop us instantly -- clear it before we start.
        if args.stop_file and os.path.exists(args.stop_file):
            try:
                os.remove(args.stop_file)
            except OSError:
                pass
        run_live(cfg, show=not args.no_display, conf_thresh=args.conf_thresh,
                 debug_lift=args.debug_lift, log=args.log, stop_file=args.stop_file)


if __name__ == "__main__":
    main()
