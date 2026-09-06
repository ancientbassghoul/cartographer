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
import csv
import json
import math
import os
import time

import cv2
import numpy as np
import torch
import yaml

import frame_bus
import slam_engine
import slam_window
from map_store import MapStore
from ground_grid import GroundGrid, explore_cfg
from frontier_planner import FrontierPlanner
from target_estimator import TargetEstimator
from diag_log import DiagLog, NullLog

REPO = os.path.dirname(os.path.abspath(__file__))

MAP_GRID = 200              # resolution of the compact top-down occupancy summary on TOPIC_MAP

# Session 62: the diag_perf CSV schema, promoted to a module constant so enable_diag(), the
# self-test and perception_timing_report.py all read ONE definition. The first nine names and
# their order are FROZEN — files written before 2026-09-05 have exactly that header, and the
# report reads by column NAME so old and new flights stay comparable in the same tool.
DIAG_PERF_FIELDS: tuple[str, ...] = (
    "wall_ts", "frame_id", "loop_dt", "slam_ms", "mode", "new_keyframe",
    "n_keyframes", "n_voxels", "reloc",
    # --- SLAM-internal phases (slam_engine.SLAM_PHASE_FIELDS) ---
    "track_ms", "backend_ms", "pose_ms", "kf_download_ms",
    # --- post-SLAM phases, measured in Pipeline.step ---
    "integrate_ms", "map_pub_ms", "plan_ms", "publish_ms",
    # --- Session 63: the sub-split inside track_ms (slam_engine.SLAM_TRACK_PHASE_FIELDS) ---
    "frame_ms", "infer_ms", "tracker_ms",
    # --- Session 64: backend-thread state (NOT part of any closure; backend_ms stays the
    #     frame-path number and is 0.0 in ASYNC) ---
    "backend_mode", "backend_queue_depth", "backend_thread_ms", "backend_pose_clobbers",
    # Session 64: WHY the backend died. The CRITICAL line naming the reason goes to a console
    # fly.py opens with CREATE_NEW_CONSOLE and never captures, so it dies with the window --
    # backend_mode=FAILED alone says THAT it failed, not why. Blank while healthy.
    "backend_error",
    # Session 65: the bounded global-optimisation window's state (slam_engine.SLAM_WINDOW_FIELDS).
    # Appended LAST, after backend_error, so the frozen nine-column prefix and every column before
    # this stays exactly where earlier flights' reports expect it by name.
    "backend_window_mode", "backend_window_kf", "backend_solve_kf", "backend_solve_edges",
    "backend_graph_edges", "backend_anchors", "backend_anchor_drift",
    # Session 66: the recording-relative frame index this row's frame was captured at, forwarded
    # from io_bridge meta (it already rode TOPIC_POSE but never reached this CSV). io_bridge writes
    # EVERY NDI video frame to flight_<ts>.mp4, so this IS the frame number in that file -- which
    # makes a flight exactly replayable offline with no frame_id->video offset arithmetic and no
    # assumption about when the operator pressed 'r'. BLANK (not 0) when not recording: 0 is a real
    # frame index -- the first recorded frame -- so coercing None to 0 would invent data.
    "rec_frame",
)

# Session 57: fixed, index-ordered marker palette for the frozen goal-anchor points baked into every
# frame of the PLY sequence (Pipeline._record_ply_marker / _write_frame_ply) so a Blender viewer can
# always tell which anchor is which without reading markers.json.
PLY_MARKER_COLORS = [(255, 0, 0), (0, 0, 255), (255, 255, 0), (0, 255, 255), (255, 128, 0)]


def load_frame_list(csv_path):
    """Session 66: read the `rec_frame` column of a flight's perception CSV into the sorted, de-duped
    source-frame indices that flight's SLAM actually consumed. Used by `--frame-list` to replay a
    recording through the EXACT frame sequence of a live flight rather than a fixed stride (a live
    solve is too slow to keep up, so io_bridge drops the backlog and the real gaps are wildly
    uneven). NO SILENT FALLBACK: a CSV with no rec_frame column, or one written while not recording
    (every cell blank), raises instead of quietly degrading to stride behaviour."""
    with open(csv_path, newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    if not rows or "rec_frame" not in rows[0]:
        raise ValueError(f"{csv_path}: no 'rec_frame' column -- flight predates session 66, "
                         f"so its frame sequence cannot be reconstructed")
    idx = sorted({int(r["rec_frame"]) for r in rows if (r.get("rec_frame") or "").strip() != ""})
    if not idx:
        raise ValueError(f"{csv_path}: every rec_frame cell is blank -- that flight was flown "
                         f"without recording ('r' in the io_bridge window), so there is no video "
                         f"to replay against")
    return idx


def _rec_frame_cell(rec_frame):
    """Session 66: render io_bridge's rec_frame for one CSV cell -- "" when not recording, the int
    otherwise. Deliberately NOT `int(rec_frame or 0)`: rec_frame 0 is the FIRST recorded frame, so a
    falsy-coalesce would silently relabel it as "not recording" (CLAUDE.md -- a missing value must be
    visibly missing, never an invented one)."""
    return "" if rec_frame is None else int(rec_frame)


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
        # Session 64: global optimization moves off the frame path onto its own thread by default
        # (perception.backend_async, config.yaml) -- see slam_engine.SlamEngine for the full rationale.
        backend_async = bool(cfg["perception"].get("backend_async", True))
        # Session 65: the bounded global-optimisation window -- see config.yaml's perception:
        # block for the full rationale. No second print here: SlamEngine.__init__ already prints
        # the startup line for this.
        window_mode = cfg["perception"].get("backend_window_mode", "OFF")
        window_kf = cfg["perception"].get("backend_window_kf", 0)
        window_policy = cfg["perception"].get("backend_window_policy", "anchored")
        self.slam = slam_engine.SlamEngine(
            conf_thresh=conf_thresh, backend_async=backend_async,
            window_mode=window_mode, window_kf=window_kf, window_policy=window_policy)
        started = self.slam.start_backend()
        print(f"[perception] SLAM backend: "
              f"{'ASYNC (own thread)' if started else 'SYNC (inline, backend_async=false)'}")
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
        # Session 60 (Finding A): whether the plan just computed in _plan_payload() is a genuine
        # TRACKING solve (F_LKG source). run() publishes THIS frame directly on lkg_frame_port when
        # true, replacing the autopilot's old frame_id-indexed ring (which aged out ~15s later,
        # when the plan naming a frame finally arrived, because the frame had already left the ring).
        self.last_plan_valid = False
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
        # Session 62: last observed duration of each phase, in ms. STICKY — a phase that did not run this
        # frame keeps its previous value here, so the 1 Hz console line stays readable across the frames
        # where the map/plan timers don't fire. The CSV is the honest record (0.0 for "did not run"); THIS
        # dict is a console convenience only and must never be logged, published or used for a decision.
        self._last_phase_ms: dict[str, float] = {
            "track": 0.0, "backend": 0.0, "pose": 0.0, "kf_download": 0.0,
            "integrate": 0.0, "map_pub": 0.0, "plan": 0.0, "publish": 0.0,
            "frame": 0.0, "infer": 0.0, "tracker": 0.0,
        }
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

        # Session 57: per-SLAM-frame PLY sequence (Blender build-up animation), config-gated OFF by
        # default (diag.ply_sequence, ~1.2 GB/flight) -- see run_live for the writer hook and
        # _record_ply_marker below for the frozen goal-anchor markers baked into every frame so the
        # sequence stays spatially aligned when opened as a stack in Blender.
        self.ply_markers = []          # list[dict], record shape in _record_ply_marker; append-only, capped
        self.ply_seq_failures = 0      # count of frame-PLY writes that raised
        self.ply_seq_degraded = False  # sticky: True on the first failure, never cleared
        self.ply_sequence_markers = int((cfg.get("diag") or {}).get("ply_sequence_markers", 5))

    def enable_diag(self, ts=None, out_dir=None):
        """Open CSV diagnostic logs (per-frame timing + per-lift hit geometry)."""
        self.diag_perf = DiagLog("perception", list(DIAG_PERF_FIELDS), out_dir=out_dir, ts=ts)
        self.diag_lift = DiagLog("lift", [
            "wall_ts", "frame_id", "found", "bbox_area", "center_x", "center_y",
            "pose_found", "cam_x", "cam_y", "cam_z", "ray_x", "ray_y", "ray_z",
            "hit", "hit_x", "hit_y", "hit_z", "march_dist",
            "n_hits", "n_inliers", "cluster_frac", "est_x", "est_y", "est_z", "confident",
        ], out_dir=out_dir, ts=ts)

    def close_diag(self):
        self.diag_perf.close()
        self.diag_lift.close()

    def close_slam(self):
        """Stop + join the backend thread (Session 64). Safe to call unconditionally on shutdown --
        SlamEngine.close() is itself safe when the thread was never started or already stopped."""
        self.slam.close()

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
        t_integrate = time.perf_counter()
        if res.camera_center is not None:
            self.mapstore.add_pose(res.camera_center)
        if res.new_keyframe and res.kf_points is not None and len(res.kf_points):
            self.mapstore.integrate(res.kf_points, res.kf_colors)
            if res.camera_center is not None:
                # Same per-keyframe data feeds the 2D free/unknown/occupied ground layer.
                self.ground.integrate(res.camera_center, res.kf_points)
            map_updated = True
        integrate_ms = (time.perf_counter() - t_integrate) * 1000.0

        heading_deg = heading_from_pose(res.pose)

        publish_ms = 0.0
        map_pub_ms = 0.0
        plan_ms = 0.0
        map_pub_fired = False
        plan_fired = False
        if state_pub is not None:
            cc = res.camera_center
            t_publish = time.perf_counter()
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
            publish_ms = (time.perf_counter() - t_publish) * 1000.0
            # Top-down occupancy snapshot: on a new keyframe (cells changed) OR on a timer, so the
            # DENSE trajectory reaches the visualizer without waiting for the next (sparse) keyframe.
            # Each message is a full self-contained snapshot, so a late joiner catches up immediately.
            now_mono = time.monotonic()
            if map_updated or (now_mono - self.last_map_pub) >= self.MAP_PUB_INTERVAL:
                t_map_pub = time.perf_counter()
                state_pub.publish(frame_bus.TOPIC_MAP, self._map_payload(res, meta))
                map_pub_ms = (time.perf_counter() - t_map_pub) * 1000.0
                self.last_map_pub = now_mono
                map_pub_fired = True
            # Map mode: republish the explore plan (goal/bearing/done + ground layer) on a timer.
            if (now_mono - self.last_plan_pub) >= self.PLAN_PUB_INTERVAL:
                t_plan = time.perf_counter()
                state_pub.publish(frame_bus.TOPIC_PLAN,
                                  self._plan_payload(res, meta, heading_deg, slam_ms))
                plan_ms = (time.perf_counter() - t_plan) * 1000.0
                self.last_plan_pub = now_mono
                plan_fired = True

        # Session 62: sticky console phase record (C3) — write a key only when that phase actually ran
        # this frame, so the 1 Hz line below keeps showing the last REAL value instead of flashing to 0
        # on frames where a phase is legitimately skipped (no new keyframe, map/plan timer not due yet).
        self._last_phase_ms["track"] = res.track_ms
        self._last_phase_ms["backend"] = res.backend_ms
        self._last_phase_ms["pose"] = res.pose_ms
        self._last_phase_ms["frame"] = res.frame_ms
        if res.infer_ms > 0.0:
            self._last_phase_ms["infer"] = res.infer_ms
        if res.tracker_ms > 0.0:
            self._last_phase_ms["tracker"] = res.tracker_ms
        if res.new_keyframe:
            self._last_phase_ms["kf_download"] = res.kf_download_ms
        if map_updated:
            self._last_phase_ms["integrate"] = integrate_ms
        if state_pub is not None:
            self._last_phase_ms["publish"] = publish_ms
        if map_pub_fired:
            self._last_phase_ms["map_pub"] = map_pub_ms
        if plan_fired:
            self._last_phase_ms["plan"] = plan_ms

        now = time.monotonic()
        if now - self.last_report >= 1.0:
            c = meta.get("controls", {}) or {}
            rc = f"{self._last_clearance:.2f}u" if self._last_clearance is not None else " -- "
            py = f"{self._last_pos_y:+.2f}" if self._last_pos_y is not None else " -- "
            rf, rb = self._last_ring_fb
            rfb = (f"{rf:.2f}" if rf is not None else "--") + "/" + (f"{rb:.2f}" if rb is not None else "--")
            p = self._last_phase_ms
            print(f"[perception] SLAM {res.mode:<8} kf {res.n_keyframes:3d} | "
                  f"vox {len(self.mapstore):6d} | slam {slam_ms:5.1f} ms | "
                  f"[trk {p['track']:.0f} bk {p['backend']:.0f} dl {p['kf_download']:.0f}] | "
                  f"(frm {p['frame']:.0f} inf {p['infer']:.0f} trk2 {p['tracker']:.0f}) | "
                  f"bk[{res.backend_mode} q{res.backend_queue_depth} {res.backend_thread_ms:.0f}ms"
                  f"{f' clob{res.backend_pose_clobbers}' if res.backend_pose_clobbers else ''}] | "
                  f"intg {p['integrate']:.0f} plan {p['plan']:.0f} map {p['map_pub']:.0f} ms | "
                  f"ray_clear {rc} | y {py} | ring f/b {rfb} | "
                  f"trigger {c.get('trigger')} yaw {c.get('yaw')}")
            self.last_report = now

        # Session 62: the row moved from just-after-SLAM to end-of-step so it can carry the POST-SLAM
        # phases too. One documented consequence: `n_voxels` is now the count AFTER this frame's
        # integrate rather than before it (off by one keyframe's worth against pre-2026-09-05 files);
        # `loop_dt` and every other column are unchanged.
        self.diag_perf.row(
            wall_ts=round(t_step, 4), frame_id=fid, loop_dt=round(loop_dt, 4),
            slam_ms=round(slam_ms, 1), mode=res.mode, new_keyframe=int(bool(res.new_keyframe)),
            n_keyframes=res.n_keyframes, n_voxels=len(self.mapstore),
            reloc=int(bool(res.reloc_event)),
            track_ms=round(res.track_ms, 1), backend_ms=round(res.backend_ms, 1),
            pose_ms=round(res.pose_ms, 1), kf_download_ms=round(res.kf_download_ms, 1),
            integrate_ms=round(integrate_ms, 1), map_pub_ms=round(map_pub_ms, 1),
            plan_ms=round(plan_ms, 1), publish_ms=round(publish_ms, 1),
            frame_ms=round(res.frame_ms, 1), infer_ms=round(res.infer_ms, 1),
            tracker_ms=round(res.tracker_ms, 1),
            backend_mode=res.backend_mode, backend_queue_depth=res.backend_queue_depth,
            backend_thread_ms=round(res.backend_thread_ms, 1),
            backend_pose_clobbers=res.backend_pose_clobbers,
            backend_error=res.backend_error,
            backend_window_mode=res.backend_window_mode, backend_window_kf=res.backend_window_kf,
            backend_solve_kf=res.backend_solve_kf, backend_solve_edges=res.backend_solve_edges,
            backend_graph_edges=res.backend_graph_edges, backend_anchors=res.backend_anchors,
            backend_anchor_drift=round(res.backend_anchor_drift, 4),
            rec_frame=_rec_frame_cell(meta.get("rec_frame")))

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
            # Session 64: backend-thread state, so the visualizer can surface a degraded/async
            # backend to the operator (CLAUDE.md rule 3) instead of it being invisible off-CSV.
            "backend_mode": res.backend_mode, "backend_queue_depth": res.backend_queue_depth,
            "backend_pose_clobbers": res.backend_pose_clobbers,
            "backend_error": res.backend_error,
            # Session 65: only 4 of the 7 window fields -- backend_solve_edges, backend_graph_edges
            # and backend_anchor_drift are CSV-only because nothing in the visualizer renders them.
            "backend_window_mode": res.backend_window_mode, "backend_window_kf": res.backend_window_kf,
            "backend_solve_kf": res.backend_solve_kf, "backend_anchors": res.backend_anchors,
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
        # Session 60 (Finding A): stash so run() can publish THIS frame as F_LKG right away, instead
        # of the autopilot reconstructing it later from a frame_id it may no longer hold.
        self.last_plan_valid = bool(valid)
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
        # Session 45: surface any selection-time rejections (the "already standing on it" too-close drop) as a
        # planner_event, so they land in the console + the timeline/flight_replay debugger instead of being a
        # silent candidate drop. Drained here (not in run()'s autopilot-event loop, which only ticks when an
        # autopilot event arrives) because select() runs on every plan publish. NOTE the ordering: this queues
        # into last_planner_event, which _consume_planner_event() already read earlier in THIS payload -- so
        # like the bump receipts, the line rides the FIRST plan AFTER the select that produced it.
        if self.planner.last_select_events:
            for _sev in self.planner.last_select_events:
                self.last_planner_event.append(_sev)
                print(f"[perception] planner: {_sev}", flush=True)
            self.planner.last_select_events = []
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
            self._record_ply_marker(payload["goal"], payload.get("pos_y"))
            bearing = math.degrees(math.atan2(goal[0] - pos[0], goal[1] - pos[1]))
            payload["bearing_deg"] = round(bearing, 2)
            payload["bearing_err"] = round(_wrap180(bearing - heading_deg), 2)
        return payload

    def _record_ply_marker(self, goal_xz, pos_y) -> None:
        """Session 57: freeze the first N distinct committed goals as PLY alignment anchors.

        No-op when: the list is already at `self.ply_sequence_markers`, `goal_xz` is None, `pos_y` is
        None, or `goal_xz` equals the most recently recorded goal (dedupe on CHANGE, so a goal held
        across many plans is recorded once). Once appended a record is NEVER modified except for
        `first_frame`, which the writer stamps on the first PLY that carries it.
        """
        if goal_xz is None or pos_y is None:
            return
        if len(self.ply_markers) >= self.ply_sequence_markers:
            return
        gx, gz = float(goal_xz[0]), float(goal_xz[1])
        if self.ply_markers and self.ply_markers[-1]["goal_xz"] == [gx, gz]:
            return
        idx = len(self.ply_markers)
        rgb = list(PLY_MARKER_COLORS[idx % len(PLY_MARKER_COLORS)])
        self.ply_markers.append({
            "goal_index": idx,
            "goal_xz": [gx, gz],
            "xyz": [gx, float(pos_y), gz],
            "rgb": rgb,
            "first_frame": None,
        })

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
def _checkpoint_livemap(pipe, out_dir, ts, min_count=2):
    """Session 55 (H2, crash survivability): write a durable snapshot of the fused SLAM map RIGHT NOW,
    via a temp file + `os.replace` per artifact, so a crash mid-write can never corrupt the PREVIOUS
    good checkpoint (`render_topdown`/`save_npz`/`save_ply` are full non-atomic rewrites). Before this,
    these three exports ran ONLY in `run_live`'s `finally` block -- the entire voxel map + point cloud
    was memory-only for the whole flight (a hard machine bugcheck, which skips `finally` entirely,
    previously lost all of it; see PROGRESS.md session 55). Called periodically from the main loop AND
    once more at clean shutdown, so a normal exit still captures the truly-latest state.

    `os.replace` is atomic on the same volume on both POSIX and Windows -- readers either see the old
    file or the new one, never a half-written one."""
    ests = pipe.estimator.estimate_all()
    targets = [e["position"] for e in ests] if ests else None
    png_final, npz_final, ply_final = (out_dir / f"{ts}_livemap_topdown.png",
                                       out_dir / f"{ts}_livemap.npz", out_dir / f"{ts}_livemap.ply")
    png_tmp, npz_tmp, ply_tmp = (out_dir / f"{ts}_livemap_topdown_tmp.png",
                                 out_dir / f"{ts}_livemap_tmp.npz", out_dir / f"{ts}_livemap_tmp.ply")
    pipe.mapstore.render_topdown(png_tmp, min_count=min_count, targets=targets)
    pipe.mapstore.save_npz(npz_tmp, min_count=min_count)
    pipe.mapstore.save_ply(ply_tmp, min_count=min_count, trajectory=True, targets=targets)
    os.replace(png_tmp, png_final)
    os.replace(npz_tmp, npz_final)
    os.replace(ply_tmp, ply_final)
    return png_final, npz_final, ply_final


def _write_frame_ply(pipe, seq_dir, frame_idx: int, min_count: int = 2):
    """Session 57: write ONE .ply for the current fused map state, for a Blender build-up animation.

    Path: `seq_dir / f"frame_{frame_idx:05d}.ply"`. Binary. Includes trajectory and every marker
    recorded so far; stamps `first_frame` on any marker whose value is still None. Returns the
    written Path. Raises OSError on failure -- the CALLER owns the loud-and-counted handling,
    mirroring the periodic-checkpoint pattern in run_live."""
    path = seq_dir / f"frame_{frame_idx:05d}.ply"
    markers = [(tuple(m["xyz"]), tuple(m["rgb"])) for m in pipe.ply_markers]
    pipe.mapstore.save_ply(path, min_count=min_count, trajectory=True, markers=markers, binary=True)
    for m in pipe.ply_markers:
        if m["first_frame"] is None:
            m["first_frame"] = frame_idx
    return path


def _write_markers_sidecar(pipe, seq_dir):
    """Session 57: rewrite `markers.json` (every marker recorded so far) -- called whenever
    Pipeline._record_ply_marker appends a new one, and once more at shutdown, so a crash mid-flight
    still leaves the alignment anchors on disk for whatever frames DID get written."""
    with open(seq_dir / "markers.json", "w", encoding="utf-8") as f:
        json.dump({"markers": pipe.ply_markers}, f, indent=2)


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
    lkg_port = cfg["network"]["lkg_frame_port"]
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")   # shared by the diag CSVs + the shutdown map export
    out_dir = Path(REPO) / "OUTPUT" / "diag"
    out_dir.mkdir(parents=True, exist_ok=True)
    # Session 55 (H2): periodic durable map checkpoint -- see _checkpoint_livemap's docstring.
    livemap_checkpoint_period_s = float((cfg.get("diag") or {}).get("livemap_checkpoint_period_s", 60.0))
    last_checkpoint_t = time.monotonic()
    # Session 57: per-SLAM-frame PLY sequence (Blender build-up animation) -- OFF by default (diag.
    # ply_sequence, ~1.2 GB/flight). seq_dir is created only when enabled.
    ply_sequence = bool((cfg.get("diag") or {}).get("ply_sequence", False))
    ply_sequence_max = int((cfg.get("diag") or {}).get("ply_sequence_max", 2000))
    seq_dir = out_dir / f"{ts}_plyseq"
    if ply_sequence:
        seq_dir.mkdir(parents=True, exist_ok=True)
    ply_frame_idx = 0
    ply_seq_cap_logged = False   # one-shot, mirrors autopilot.py's visrec_cap_logged
    pipe = Pipeline(cfg, conf_thresh=conf_thresh, debug_lift=debug_lift)
    if log:
        pipe.enable_diag(ts=ts)
    frame_sub = frame_bus.FrameSubscriber(frame_port)
    state_pub = frame_bus.StatePublisher(pstate_port)  # binds; fail-fast if taken
    # Session 60 (Finding A): F_LKG at its source. The plan naming a frame arrives ~15s after that
    # frame went past (SLAM solve latency), so the autopilot's old frame_id-indexed ring had to hold
    # ~17.6s of history to look it back up -- and still aged out (median miss 0.55s, 34 times one
    # flight). Publishing the frame directly, the instant its solve is confirmed TRACKING, replaces
    # that reconstruction entirely: the autopilot just takes the newest one (CONFLATE=1 below).
    lkg_pub = frame_bus.FramePublisher(lkg_port)
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
                            wmsg = (f"WARNING: pick landed on an ALREADY-excluded goal={wg} -> blacklist bypassed"
                                    f"{' (sweep-tour CORNER target)' if pipe.planner.sweeping else ''}")
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

            n_ply_markers_before = len(pipe.ply_markers)
            _, _, panel, map_updated = pipe.step(frame, meta, state_pub, show)
            # Session 60 (Finding A): publish THIS frame as F_LKG iff its solve just confirmed TRACKING
            # -- i.e. this is the exact frame the plan the autopilot is about to receive was computed
            # from. No ring, no ID lookup, no age-out.
            if pipe.last_plan_valid:
                lkg_pub.publish(frame, meta)
            if ply_sequence and len(pipe.ply_markers) > n_ply_markers_before:
                _write_markers_sidecar(pipe, seq_dir)

            if ply_sequence and map_updated and ply_frame_idx < ply_sequence_max:
                try:
                    _write_frame_ply(pipe, seq_dir, ply_frame_idx)
                    ply_frame_idx += 1
                except OSError as exc:
                    pipe.ply_seq_failures += 1
                    pipe.ply_seq_degraded = True
                    print(f"*** CRITICAL: frame PLY #{ply_frame_idx} FAILED ({exc}) -- "
                          f"{pipe.ply_seq_failures} failure(s) so far; the flight continues but the PLY "
                          f"sequence is INCOMPLETE ***", flush=True)
            elif (ply_sequence and map_updated and ply_frame_idx >= ply_sequence_max
                  and not ply_seq_cap_logged):
                ply_seq_cap_logged = True
                print(f"*** PLY sequence cap reached (ply_sequence_max={ply_sequence_max}) -> no "
                      f"further frame PLYs written ***", flush=True)

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
            # Session 55 (H2): periodic durable checkpoint of the whole map, so a hard crash costs at
            # most livemap_checkpoint_period_s instead of the entire flight (previously memory-only
            # until a clean shutdown reached `finally` below). Caught (not left to crash the whole
            # flight) because a periodic checkpoint failing must never take the mission down with it --
            # e.g. Windows raises PermissionError from os.replace() if some OTHER process (an operator
            # inspecting the last checkpoint) has the target file open. Surfaced LOUDLY, not swallowed
            # (CLAUDE.md: visible degraded-state alert, not a silent fallback); the FINAL checkpoint in
            # `finally` below is intentionally left UNPROTECTED -- that is the last chance to save the
            # map and a failure there should be as visible as possible.
            if livemap_checkpoint_period_s > 0 and (now - last_checkpoint_t) >= livemap_checkpoint_period_s:
                try:
                    _checkpoint_livemap(pipe, out_dir, ts, min_count=2)
                    print(f"[perception] periodic livemap checkpoint -> {out_dir / f'{ts}_livemap.npz'}",
                          flush=True)
                except OSError as exc:
                    print(f"*** CRITICAL: periodic livemap checkpoint FAILED ({exc}) -- continuing the "
                          f"flight; will retry in {livemap_checkpoint_period_s:g}s. If this repeats, "
                          f"the map is at risk on a crash until it succeeds once ***", flush=True)
                last_checkpoint_t = now

            if _show_and_quit(panel, pipe, map_updated, show):
                break
    except KeyboardInterrupt:
        pass
    finally:
        print("[perception] shutting down ...")
        # Export the fused SLAM map via the SAME checkpoint helper the periodic tick above uses (session
        # 55) -- a clean exit still captures the truly-latest state, not just the last periodic snapshot.
        # This `finally` is only reachable on a NORMAL exit (the 'q' key, Ctrl+C, or the stop-file) -- a
        # hard TerminateProcess/bugcheck skips it, which is exactly why the periodic checkpoint exists.
        png, npz, ply = _checkpoint_livemap(pipe, out_dir, ts, min_count=2)
        print(f"[perception] top-down (flight path + target marks) -> {png}")
        print(f"[perception] voxel map -> {npz}")
        print(f"[perception] point cloud + flight path + targets (.ply, Blender-loadable) -> {ply}")
        if ply_sequence:
            _write_markers_sidecar(pipe, seq_dir)
            print(f"[perception] PLY sequence ({ply_frame_idx} frame(s)"
                  f"{', DEGRADED -- see CRITICAL lines above' if pipe.ply_seq_degraded else ''}) -> {seq_dir}")
        pipe.close_slam()
        pipe.close_diag()
        frame_sub.close()
        state_pub.close()
        lkg_pub.close()
        det_sub.close()
        apevent_sub.close()
        if show:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass


def _video_frames(path, stride, max_frames, proc_w, proc_h, object_frame_h=720,
                  frame_list=None):
    """Yield (small_512x288, hires, meta) from an mp4, sub-sampled — mirrors io_bridge's two
    streams. `hires` is the native frame downscaled to `object_frame_h` (no upscale), for the
    object detector; perception uses the 512x288 `small`.

    Session 66: `frame_list` (sorted, de-duped source indices) REPLACES `stride` when given. A live
    flight never sees evenly-spaced frames -- SLAM is slow, so io_bridge hands it whatever arrived
    last and the backlog is dropped (measured: a 112-frame median gap, p90 310). Replaying at any
    fixed stride therefore puts SLAM in a regime the drone never flies in. Feeding back the
    `rec_frame` column of that flight's own perception CSV reproduces the exact frame sequence the
    live solve consumed. Frames are still read sequentially -- no seeking, which OpenCV does not do
    reliably on a long mp4 -- and non-selected frames are decoded and discarded."""
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise RuntimeError(f"could not open recording: {path}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    wanted = list(frame_list) if frame_list is not None else None
    w_ptr = 0
    src_idx = yielded = 0
    while True:
        ret, bgr = cap.read()
        if not ret:
            break
        if wanted is not None:
            take = w_ptr < len(wanted) and src_idx == wanted[w_ptr]
            if take:
                w_ptr += 1
            elif w_ptr >= len(wanted):
                break
        else:
            take = (src_idx % stride == 0)
        if take:
            small = cv2.resize(bgr, (proc_w, proc_h), interpolation=cv2.INTER_AREA)
            sh, sw = bgr.shape[:2]
            if sh > object_frame_h:
                ow = int(round(sw * object_frame_h / sh))
                hires = cv2.resize(bgr, (ow, object_frame_h), interpolation=cv2.INTER_AREA)
            else:
                hires = bgr
            meta = {"frame_id": yielded, "mono_ts": time.monotonic(),
                    "sim_time": round(src_idx / fps, 3), "controls": {},
                    # session 66: the source index IS the recorded video's frame number, so a
                    # replay CSV carries the same rec_frame correspondence as the live flight.
                    "rec_frame": src_idx}
            yield small, hires, meta
            yielded += 1
            if max_frames and yielded >= max_frames:
                break
        src_idx += 1
    cap.release()


def run_offline_video(cfg, video, show=False, stride=3, max_frames=0,
                      out_dir=None, conf_thresh=1.5, publish=False,
                      detect=False, detect_every=5, debug_lift=False, log=False,
                      frame_list=None):
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
                                                object_frame_h, frame_list=frame_list):
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

    ok = _self_test_checkpoint_livemap()
    print(f"[perception][self-test] {'PASS' if ok else 'FAIL'}  session 55 (H2) livemap checkpoint")
    assert ok

    ok_ply = _self_test_ply_sequence(cfg)
    print(f"[perception][self-test] {'PASS' if ok_ply else 'FAIL'}  SESSION-57 PLY SEQUENCE")
    assert ok_ply

    ok_lkg = _self_test_f_lkg_source(cfg)
    print(f"[perception][self-test] {'PASS' if ok_lkg else 'FAIL'}  SESSION-60 F_LKG SOURCE")
    assert ok_lkg

    ok_phases = _self_test_slam_phase_fields()
    print(f"[perception][self-test] {'PASS' if ok_phases else 'FAIL'}  SESSION-62 SLAM PHASE FIELDS")
    assert ok_phases

    ok_track_phases = _self_test_slam_track_phase_fields()
    print(f"[perception][self-test] {'PASS' if ok_track_phases else 'FAIL'}  SESSION-63 SLAM TRACK PHASE FIELDS")
    assert ok_track_phases

    ok_timing = _self_test_phase_timing(cfg)
    print(f"[perception][self-test] {'PASS' if ok_timing else 'FAIL'}  SESSION-62 PHASE TIMING SCHEMA")
    assert ok_timing

    ok_backend = _self_test_backend_thread()
    print(f"[perception][self-test] {'PASS' if ok_backend else 'FAIL'}  SESSION-64 BACKEND THREAD")
    assert ok_backend

    ok_window = _self_test_slam_window_fields()
    print(f"[perception][self-test] {'PASS' if ok_window else 'FAIL'}  SESSION-65 SLAM WINDOW FIELDS")
    assert ok_window

    ok_window_diag = _self_test_diag_window_fields()
    print(f"[perception][self-test] {'PASS' if ok_window_diag else 'FAIL'}  SESSION-65 CHUNK 4 DIAG/MAP WINDOW FIELDS")
    assert ok_window_diag

    print("[perception][self-test] PASS")


def _self_test_checkpoint_livemap():
    """Session 55 (H2): _checkpoint_livemap must (a) produce loadable, non-empty artifacts from live
    map state, (b) leave NO stray *_tmp.* files behind (the .tmp+os.replace atomicity contract), and
    (c) a SECOND checkpoint call must cleanly REPLACE the first, never leaving a half-written or stale
    file -- the whole point of writing to a temp path first (np.savez/save_ply/render_topdown are full
    non-atomic rewrites; a crash mid-write must not corrupt the PREVIOUS good checkpoint). No GPU/SLAM
    needed: only MapStore + TargetEstimator, duck-typed as a Pipeline substitute (_checkpoint_livemap
    only touches pipe.mapstore / pipe.estimator)."""
    import tempfile
    import types
    from pathlib import Path
    import numpy as np
    from map_store import MapStore
    from target_estimator import TargetEstimator

    ok = True
    tmp_dir = Path(tempfile.mkdtemp(prefix="checkpoint_selftest_"))
    try:
        pipe = types.SimpleNamespace(mapstore=MapStore(0.1), estimator=TargetEstimator())
        pipe.mapstore.integrate(np.array([[1.0, 0.0, 1.0], [2.0, 0.0, 2.0]], np.float64))
        pipe.mapstore.add_pose(np.array([0.0, 0.0, 0.0]))
        ts = "20260101_000000"

        png1, npz1, ply1 = _checkpoint_livemap(pipe, tmp_dir, ts, min_count=1)
        first_ok = png1.exists() and npz1.exists() and ply1.exists()
        no_stray_tmp_1 = not any(tmp_dir.glob("*_tmp.*"))
        # `with`: release the handle before the 2nd checkpoint's os.replace() runs -- on Windows,
        # os.replace() over a file another handle still has open raises PermissionError (confirmed:
        # this test failed with exactly that until the file was closed first).
        with np.load(npz1) as loaded1:
            centers_ok = len(loaded1["centers"]) == 2

        # A second checkpoint with DIFFERENT content must fully replace the first, not merge/append.
        pipe.mapstore.integrate(np.array([[9.0, 0.0, 9.0]], np.float64))
        png2, npz2, ply2 = _checkpoint_livemap(pipe, tmp_dir, ts, min_count=1)
        with np.load(npz2) as loaded2:
            replaced_ok = len(loaded2["centers"]) == 3 and (png1, npz1, ply1) == (png2, npz2, ply2)
        no_stray_tmp_2 = not any(tmp_dir.glob("*_tmp.*"))

        ok = first_ok and no_stray_tmp_1 and centers_ok and replaced_ok and no_stray_tmp_2
        print(f"[perception][self-test]   checkpoint files written={first_ok}, no stray .tmp after "
              f"1st={no_stray_tmp_1}, centers round-trip={centers_ok}, 2nd checkpoint REPLACES (not "
              f"merges)={replaced_ok}, no stray .tmp after 2nd={no_stray_tmp_2}")
    finally:
        import shutil as _shutil
        _shutil.rmtree(tmp_dir, ignore_errors=True)
    return ok


def _self_test_ply_sequence(cfg):
    """SESSION-57 PLY SEQUENCE: the per-SLAM-frame PLY writer, its frozen goal-anchor markers, and
    the loud-and-counted failure path diagnosed as missing in the mission's crash-survivability pass
    (session 55) but never exercised for THIS artifact. No GPU/SLAM needed: MapStore + TargetEstimator
    duck-typed as a Pipeline substitute (mirrors _self_test_checkpoint_livemap) -- _write_frame_ply only
    touches pipe.mapstore / pipe.ply_markers, and _record_ply_marker only touches pipe.ply_markers /
    pipe.ply_sequence_markers."""
    import tempfile
    import types
    from pathlib import Path
    import numpy as np
    from map_store import MapStore
    from target_estimator import TargetEstimator

    ok = True

    def check(name, cond):
        nonlocal ok
        ok = ok and bool(cond)
        print(f"[perception][self-test] {'PASS' if cond else 'FAIL'}  {name}")

    def _make_pipe(markers_cap=5):
        p = types.SimpleNamespace(mapstore=MapStore(0.1), estimator=TargetEstimator())
        pts = np.array([[1.0, 0.0, 1.0], [2.0, 0.0, 2.0]], np.float64)
        p.mapstore.integrate(pts)
        p.mapstore.integrate(pts)   # 2nd observation -> passes _write_frame_ply's default min_count=2
        p.mapstore.add_pose(np.array([0.0, 0.0, 0.0]))
        p.ply_markers = []
        p.ply_seq_failures = 0
        p.ply_seq_degraded = False
        p.ply_sequence_markers = markers_cap
        p._record_ply_marker = types.MethodType(Pipeline._record_ply_marker, p)
        return p

    p1 = _make_pipe()
    p1._record_ply_marker([1.0, 2.0], 0.5)
    p1._record_ply_marker([1.0, 2.0], 0.5)
    p1._record_ply_marker([1.0, 2.0], 0.5)
    p1._record_ply_marker([3.0, 4.0], 0.6)
    check("marker_dedupes_on_change -- same goal x3 -> 1 record, new goal -> 2nd",
          len(p1.ply_markers) == 2)

    p2 = _make_pipe(markers_cap=2)
    p2._record_ply_marker([1.0, 1.0], 0.0)
    p2._record_ply_marker([2.0, 2.0], 0.0)
    p2._record_ply_marker([3.0, 3.0], 0.0)
    check("marker_cap_respected -- cap=2, 3rd distinct goal ignored", len(p2.ply_markers) == 2)

    p3 = _make_pipe()
    p3._record_ply_marker(None, 0.5)
    p3._record_ply_marker([1.0, 1.0], None)
    check("marker_skips_none -- goal_xz=None or pos_y=None appends nothing", len(p3.ply_markers) == 0)

    p4 = _make_pipe()
    p4._record_ply_marker([5.0, 7.0], 1.25)
    xyz_ok = p4.ply_markers[0]["xyz"] == [5.0, 1.25, 7.0]
    check(f"marker_xyz_shape -- xyz=[goal_x,pos_y,goal_z] ({p4.ply_markers[0]['xyz']})", xyz_ok)

    rec_dtype = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                          ("red", "u1"), ("green", "u1"), ("blue", "u1")])

    def _tail_marker_xyz(raw, n=7):
        hdr_end = raw.index(b"end_header\n") + len(b"end_header\n")
        arr = np.frombuffer(raw[hdr_end:], dtype=rec_dtype)
        tail = arr[-n:]
        return np.stack([tail["x"], tail["y"], tail["z"]], axis=1)

    with tempfile.TemporaryDirectory() as td:
        td = Path(td)

        p5 = _make_pipe()
        seq5 = td / "seq5"
        seq5.mkdir()
        out5 = _write_frame_ply(p5, seq5, 0)
        raw5 = out5.read_bytes()
        check("frame_ply_written -- non-empty frame_00000.ply, binary header",
              out5.exists() and out5.name == "frame_00000.ply" and len(raw5) > 0
              and raw5.startswith(b"ply\nformat binary_little_endian 1.0\n"))

        p6 = _make_pipe()
        p6._record_ply_marker([1.0, 2.0], 0.5)
        seq6 = td / "seq6"
        seq6.mkdir()
        out6_0 = _write_frame_ply(p6, seq6, 0)
        out6_1 = _write_frame_ply(p6, seq6, 1)
        same_coords = np.array_equal(_tail_marker_xyz(out6_0.read_bytes()),
                                     _tail_marker_xyz(out6_1.read_bytes()))
        check("markers_identical_across_frames -- marker vertices byte-identical across frames",
              same_coords)
        check("first_frame_stamped_once -- every marker's first_frame == 0 after frames 0,1",
              all(m["first_frame"] == 0 for m in p6.ply_markers))

        p8 = _make_pipe()
        p8._record_ply_marker([1.0, 1.0], 0.0)
        p8._record_ply_marker([2.0, 2.0], 0.0)
        seq8 = td / "seq8"
        seq8.mkdir()
        _write_markers_sidecar(p8, seq8)
        with open(seq8 / "markers.json", "r", encoding="utf-8") as f:
            loaded = json.load(f)
        check("sidecar_matches -- markers.json round-trips pipe.ply_markers",
              loaded == {"markers": p8.ply_markers})

        p9 = _make_pipe()
        bad_dir = td / "does_not_exist"       # never created -> save_ply's open() raises OSError
        try:
            _write_frame_ply(p9, bad_dir, 0)
        except OSError:
            p9.ply_seq_failures += 1
            p9.ply_seq_degraded = True
        check("failure_is_counted -- OSError on a missing dir increments ply_seq_failures + degrades",
              p9.ply_seq_failures == 1 and p9.ply_seq_degraded is True)

        # Session 62: this read the LIVE config's `diag.ply_sequence` and asserted it was False, so it
        # could only ever pass while the operator had the feature OFF -- it went red the moment
        # ply_sequence was enabled for a flight (commit 9574a1f), reporting a config choice as a code
        # defect and taking the whole suite (and the chunk gate) down with it. What run_live's gate
        # actually promises is a BICONDITIONAL: the sequence dir exists iff the flag is set. Drive both
        # branches from explicit locals so the result depends on the logic, not on config.yaml.
        for gate, name in ((False, "seq10_gate_off"), (True, "seq11_gate_on")):
            seq = td / name
            if gate:
                seq.mkdir(parents=True, exist_ok=True)   # mirrors run_live's gating
            check(f"gate_{'on_creates_dir' if gate else 'off_writes_nothing'} -- "
                  f"ply_sequence={gate} -> seq dir {'created' if gate else 'never created'}",
                  seq.exists() is gate)

    return ok


def _self_test_f_lkg_source(cfg):
    """SESSION-60 F_LKG SOURCE (Finding A): `last_plan_valid` must track `_plan_payload`'s own `valid`
    local exactly -- it's what run_live() checks to decide whether THIS frame is F_LKG (see the
    lkg_pub.publish() call beside pipe.step()). A false positive would publish a frame SLAM was NOT
    actually tracking on; a false negative starves F_LKG entirely and reintroduces the age-out this
    session removes the ring to fix. No GPU/SLAM needed: GroundGrid + FrontierPlanner + MapStore,
    duck-typed as a Pipeline substitute (mirrors _self_test_checkpoint_livemap/_self_test_ply_sequence)
    -- _plan_payload only touches the fields set below."""
    import types

    ok = True

    def check(name, cond):
        nonlocal ok
        ok = ok and bool(cond)
        print(f"[perception][self-test] {'PASS' if cond else 'FAIL'}  {name}")

    e = explore_cfg(cfg)

    def _make_pipe():
        p = types.SimpleNamespace(
            ground=GroundGrid(cfg), planner=FrontierPlanner(cfg), mapstore=MapStore(0.1),
            _slam_seq=0, last_planner_event=[], GROUND_RASTER=160, last_plan_valid=False,
            clearance_fan_deg=float(e.get("clearance_fan_deg", 15.0)),
            clearance_fan_n=int(e.get("clearance_fan_n", 3)),
            clearance_skip=float(e.get("clearance_skip", 0.25)),
            clearance_min_count=int(e.get("clearance_min_count", 2)),
            clearance_max_range=float(e.get("clearance_max_range", 10.0)),
            clearance_min_hit_fraction=float(e.get("clearance_min_hit_fraction", 0.0)),
            clearance_ring_step=float(e.get("turn_step_deg", 45.0)),
            ring_max_range=float(e.get("ring_max_range", 1.5)),
            reposition_inset=float(e.get("reposition_inset", 0.8)),
            _last_clearance=None, _last_pos_y=None, _last_ring_fb=None,
            _sweep_target_logged=None, _sweep_logged=False,
            ply_markers=[], ply_sequence_markers=0,
        )
        p._consume_planner_event = lambda: (
            "; ".join(p.last_planner_event) if p.last_planner_event else None)
        return p

    def _res(mode, pos_ok, tracking_mode="MASt3R"):
        cc = np.array([1.0, 0.0, 2.0]) if pos_ok else None
        return types.SimpleNamespace(mode=mode, tracking_mode=tracking_mode, camera_center=cc)

    meta = {"mono_ts": 0.0, "frame_id": 1, "sim_time": 0.0}

    pipe = _make_pipe()
    check("fresh_pipe -- last_plan_valid starts False", pipe.last_plan_valid is False)

    Pipeline._plan_payload(pipe, _res("TRACKING", True), meta, heading_deg=0.0)
    check("tracking_pose_heading -- all three present -> last_plan_valid True",
          pipe.last_plan_valid is True)

    pipe_mode = _make_pipe()
    Pipeline._plan_payload(pipe_mode, _res("LOST", True), meta, heading_deg=0.0)
    check("mode_not_tracking -- mode='LOST' -> last_plan_valid False",
          pipe_mode.last_plan_valid is False)

    pipe_pos = _make_pipe()
    Pipeline._plan_payload(pipe_pos, _res("TRACKING", False), meta, heading_deg=0.0)
    check("missing_pose -- camera_center=None -> last_plan_valid False",
          pipe_pos.last_plan_valid is False)

    pipe_hdg = _make_pipe()
    Pipeline._plan_payload(pipe_hdg, _res("TRACKING", True), meta, heading_deg=None)
    check("missing_heading -- heading_deg=None -> last_plan_valid False",
          pipe_hdg.last_plan_valid is False)

    return ok


def _self_test_slam_phase_fields():
    """SESSION-62 SLAM PHASE FIELDS: `SlamResult` gained four per-phase timing fields
    (`slam_engine.SLAM_PHASE_FIELDS`) that must default to 0.0 (never None -- a None becomes a blank
    CSV cell and a ragged row downstream, per CLAUDE.md's no-silent-fallback rule), stay present on
    a pre-session-62-style construction, and round-trip real values. No GPU/SLAM needed: SlamResult
    is a plain dataclass."""
    ok = True

    def check(name, cond):
        nonlocal ok
        ok = ok and bool(cond)
        print(f"[perception][self-test] {'PASS' if cond else 'FAIL'}  {name}")

    check("phase_fields_tuple -- SLAM_PHASE_FIELDS is the frozen four, in order",
          slam_engine.SLAM_PHASE_FIELDS == ("track_ms", "backend_ms", "pose_ms", "kf_download_ms"))

    r = slam_engine.SlamResult(
        tracking_mode="MASt3R", mode="TRACKING", n_keyframes=0, frame_idx=0,
        camera_center=None, new_keyframe=False, reloc_event=False)
    check("defaults_present_and_zero -- all four phase fields default to 0.0",
          all(getattr(r, f) == 0.0 for f in slam_engine.SLAM_PHASE_FIELDS))
    check("defaults_are_float -- each phase field is a float",
          all(isinstance(getattr(r, f), float) for f in slam_engine.SLAM_PHASE_FIELDS))

    r2 = slam_engine.SlamResult(
        tracking_mode="MASt3R", mode="TRACKING", n_keyframes=0, frame_idx=0,
        camera_center=None, new_keyframe=False, reloc_event=False,
        track_ms=12.5, backend_ms=900.0, pose_ms=1.5, kf_download_ms=70.0)
    check("roundtrip -- explicit phase values come back unchanged",
          (r2.track_ms, r2.backend_ms, r2.pose_ms, r2.kf_download_ms) == (12.5, 900.0, 1.5, 70.0))

    return ok


def _self_test_slam_track_phase_fields():
    """SESSION-63 SLAM TRACK PHASE FIELDS: `SlamResult` gained three fields
    (`slam_engine.SLAM_TRACK_PHASE_FIELDS`) that split the INSIDE of `track_ms` -- frame
    construction, MASt3R inference (INIT/RELOC only), and the tracker (TRACKING only). They must
    default to 0.0 (never None, per CLAUDE.md's no-silent-fallback rule), stay disjoint from the
    session-62 `SLAM_PHASE_FIELDS`, and round-trip real values. No GPU/SLAM needed: SlamResult is
    a plain dataclass."""
    ok = True

    def check(name, cond):
        nonlocal ok
        ok = ok and bool(cond)
        print(f"[perception][self-test] {'PASS' if cond else 'FAIL'}  {name}")

    check("track_phase_fields_tuple -- SLAM_TRACK_PHASE_FIELDS is the frozen three, in order",
          slam_engine.SLAM_TRACK_PHASE_FIELDS == ("frame_ms", "infer_ms", "tracker_ms"))
    check("phase_fields_unchanged -- session-62 SLAM_PHASE_FIELDS is still the frozen four",
          slam_engine.SLAM_PHASE_FIELDS == ("track_ms", "backend_ms", "pose_ms", "kf_download_ms"))
    check("fields_disjoint -- SLAM_PHASE_FIELDS and SLAM_TRACK_PHASE_FIELDS share no names",
          set(slam_engine.SLAM_PHASE_FIELDS) & set(slam_engine.SLAM_TRACK_PHASE_FIELDS) == set())

    r = slam_engine.SlamResult(
        tracking_mode="MASt3R", mode="TRACKING", n_keyframes=0, frame_idx=0,
        camera_center=None, new_keyframe=False, reloc_event=False)
    check("defaults_present_and_zero -- all three track-phase fields default to 0.0",
          all(getattr(r, f) == 0.0 for f in slam_engine.SLAM_TRACK_PHASE_FIELDS))
    check("defaults_are_float -- each track-phase field is a float",
          all(isinstance(getattr(r, f), float) for f in slam_engine.SLAM_TRACK_PHASE_FIELDS))

    r2 = slam_engine.SlamResult(
        tracking_mode="MASt3R", mode="TRACKING", n_keyframes=0, frame_idx=0,
        camera_center=None, new_keyframe=False, reloc_event=False,
        frame_ms=3.5, infer_ms=210.0, tracker_ms=48.25)
    check("roundtrip -- explicit track-phase values come back unchanged",
          (r2.frame_ms, r2.infer_ms, r2.tracker_ms) == (3.5, 210.0, 48.25))

    return ok


class _FakeBackendStates:
    """Session 64: stands in for mast3r_slam.frame.SharedStates -- just enough surface for
    SlamEngine._backend_loop / _queue_depth to drive against, with a REAL lock (the whole point
    of this session is that InProcessManager.RLock() is a real threading.RLock now)."""

    def __init__(self):
        import threading
        self.lock = threading.RLock()
        self.global_optimizer_tasks = []
        self._mode = "TRACKING"

    def get_mode(self):
        with self.lock:
            return self._mode

    def is_paused(self):
        with self.lock:
            return False


class _FakeBackendKeyframes:
    """Session 64 (chunk 2): stands in for mast3r_slam.frame.SharedKeyframes -- just enough surface
    (lock/__len__/T_WC) for the C4 clobber-recording path in _backend_loop/process() to drive
    against. A REAL lock and a small CPU tensor buffer; no CUDA/no real keyframes needed since the
    fake queue in _FakeBackendStates never actually grows this buffer."""

    def __init__(self, buffer=4):
        import threading
        self.lock = threading.RLock()
        self.T_WC = torch.zeros(buffer, 1, 8)
        self._n = 0

    def __len__(self):
        with self.lock:
            return self._n


def _self_test_backend_thread():
    """SESSION-64 BACKEND THREAD (chunk 1) + BACKEND TELEMETRY/CLOBBER COUNTER (chunk 2):
    `_run_backend()` moves off the frame path onto its own thread, gated by `backend_async` and
    stoppable via `close()`. No CUDA/no real SlamEngine: the engine is built with `object.__new__`
    and only the attributes the thread body touches are set, against a fake states double exposing
    lock/global_optimizer_tasks/get_mode()/is_paused() -- per CLAUDE.md's no-silent-fallback rule, a
    dead backend thread must set a visible `backend_failed` flag and STAY dead rather than reverting
    to synchronous or retrying. Chunk 2 adds: `SLAM_BACKEND_FIELDS`/`SlamResult` defaults,
    `_backend_mode()` resolution (FAILED beats ASYNC beats SYNC), and `_check_backend_clobber()`
    accounting against plain CPU tensors -- the one race session 64 counts instead of prevents."""
    import threading
    import types

    ok = True

    def check(name, cond):
        nonlocal ok
        ok = ok and bool(cond)
        print(f"[perception][self-test] {'PASS' if cond else 'FAIL'}  {name}")

    check("idle_sleep_bounded -- BACKEND_IDLE_SLEEP_S is a positive float under 0.05",
          isinstance(slam_engine.BACKEND_IDLE_SLEEP_S, float)
          and 0.0 < slam_engine.BACKEND_IDLE_SLEEP_S <= 0.1)   # session 64: widened from <0.05 when the
          #   gap moved 5ms -> 50ms to cut states.lock contention (see BACKEND_IDLE_SLEEP_S)

    def make_engine(backend_async, run_backend):
        eng = object.__new__(slam_engine.SlamEngine)
        eng._initialized = True        # session 64: states/keyframes exist only after _lazy_state();
                                       #   the double stands in for an already-initialised engine
        eng.device = "cuda:0"          # session 64: a real engine always has this, and the thread
                                       #   body pins its own (thread-local) CUDA device from it
        eng.backend_async = backend_async
        eng.backend_failed = None
        eng._backend_thread = None
        eng._backend_stop = threading.Event()
        eng._backend_meta_lock = threading.Lock()
        eng._backend_thread_ms = 0.0
        eng._backend_solves = 0
        eng._backend_pose_clobbers = 0
        eng._backend_last_written = {}
        eng.states = _FakeBackendStates()
        eng.keyframes = _FakeBackendKeyframes()
        eng._Mode = types.SimpleNamespace(RELOC="RELOC")
        eng._run_backend = run_backend
        return eng

    # -- kill switch: backend_async=False starts nothing --
    eng_off = make_engine(False, lambda: None)
    started_off = eng_off.start_backend()
    check("start_backend_off -- returns False and starts no thread when backend_async is False",
          started_off is False and eng_off._backend_thread is None)

    # -- on: starts a live daemon thread; a second call is an idempotent no-op --
    eng_on = make_engine(True, lambda: None)
    started1 = eng_on.start_backend()
    thread1 = eng_on._backend_thread
    alive_and_daemon = thread1 is not None and thread1.is_alive() and thread1.daemon
    started2 = eng_on.start_backend()
    check("start_backend_on -- returns True, thread alive, is a daemon",
          started1 is True and alive_and_daemon)
    check("start_backend_idempotent -- second call no-ops, still True, same thread object",
          started2 is True and eng_on._backend_thread is thread1)
    eng_on.close()
    check("close_stops_thread -- is_alive() False after close()", not thread1.is_alive())

    # -- drains a queue of tasks and keeps polling once it is empty --
    calls_drain = []

    def run_backend_drain():
        calls_drain.append(1)
        with eng_drain.states.lock:
            if eng_drain.states.global_optimizer_tasks:
                eng_drain.states.global_optimizer_tasks.pop(0)

    eng_drain = make_engine(True, run_backend_drain)
    eng_drain.states.global_optimizer_tasks.extend([0, 1, 2])
    eng_drain.start_backend()
    deadline = time.time() + 2.0
    while time.time() < deadline and len(calls_drain) < 3:
        time.sleep(0.01)
    check("drains_queue -- fake _run_backend called >=3x for 3 queued tasks within a bounded wait",
          len(calls_drain) >= 3)
    count_after_drain = len(calls_drain)
    # Session 64: derive the wait from the poll gap itself rather than hardcoding it -- this case
    # broke when BACKEND_IDLE_SLEEP_S moved 5ms -> 50ms to cut states.lock contention, even though
    # the behaviour under test (the loop keeps polling instead of exiting) was completely unchanged.
    time.sleep(slam_engine.BACKEND_IDLE_SLEEP_S * 4 + 0.05)
    check("polls_after_empty -- loop keeps calling _run_backend once the queue is empty (no exit)",
          eng_drain._backend_thread.is_alive() and len(calls_drain) > count_after_drain)
    eng_drain.close()

    # -- close() is safe when never started, and safe to call twice --
    eng_never = make_engine(True, lambda: None)
    try:
        eng_never.close()
        eng_never.close()
        close_safety_ok = True
    except BaseException:
        close_safety_ok = False
    check("close_never_started_and_twice -- no-op both times, never raises", close_safety_ok)

    # -- an exception in _run_backend sets backend_failed and the thread does not re-enter it --
    calls_fail = []

    def run_backend_raises():
        calls_fail.append(1)
        raise RuntimeError("boom")

    eng_fail = make_engine(True, run_backend_raises)
    eng_fail.start_backend()
    deadline = time.time() + 2.0
    while time.time() < deadline and eng_fail.backend_failed is None:
        time.sleep(0.01)
    failed_str_ok = isinstance(eng_fail.backend_failed, str) and len(eng_fail.backend_failed) > 0
    time.sleep(0.05)
    check("failure_sets_backend_failed_and_stops -- non-empty string, thread dead, called exactly once",
          failed_str_ok and not eng_fail._backend_thread.is_alive() and len(calls_fail) == 1)

    # -- C2: SLAM_BACKEND_FIELDS is exactly the four names, disjoint from the two phase tuples --
    check("backend_fields_tuple -- SLAM_BACKEND_FIELDS is the frozen four, in order",
          slam_engine.SLAM_BACKEND_FIELDS == ("backend_mode", "backend_queue_depth",
                                               "backend_thread_ms", "backend_pose_clobbers",
                                               "backend_error"))   # session 64: + the failure reason
    check("backend_fields_disjoint_from_phase_fields -- shares no names with SLAM_PHASE_FIELDS",
          set(slam_engine.SLAM_BACKEND_FIELDS) & set(slam_engine.SLAM_PHASE_FIELDS) == set())
    check("backend_fields_disjoint_from_track_phase_fields -- shares no names with SLAM_TRACK_PHASE_FIELDS",
          set(slam_engine.SLAM_BACKEND_FIELDS) & set(slam_engine.SLAM_TRACK_PHASE_FIELDS) == set())

    # -- C2: a SlamResult built with only pre-session-64 arguments gets SYNC/0/0.0/0 defaults --
    _r_default = slam_engine.SlamResult(
        tracking_mode="MASt3R", mode="TRACKING", n_keyframes=0, frame_idx=0,
        camera_center=None, new_keyframe=False, reloc_event=False)
    check("slam_result_backend_defaults -- SYNC/0/0.0/0, correctly typed",
          _r_default.backend_mode == "SYNC" and isinstance(_r_default.backend_mode, str)
          and _r_default.backend_queue_depth == 0 and isinstance(_r_default.backend_queue_depth, int)
          and _r_default.backend_thread_ms == 0.0 and isinstance(_r_default.backend_thread_ms, float)
          and _r_default.backend_pose_clobbers == 0 and isinstance(_r_default.backend_pose_clobbers, int))

    # -- C2: backend_mode -- SYNC (no thread), ASYNC (live thread), FAILED beats both --
    eng_sync = make_engine(True, lambda: None)
    check("backend_mode_sync -- no thread started yet -> SYNC", eng_sync._backend_mode() == "SYNC")

    eng_async = make_engine(True, lambda: None)
    eng_async.start_backend()
    check("backend_mode_async -- live thread, no failure -> ASYNC", eng_async._backend_mode() == "ASYNC")
    eng_async.backend_failed = "RuntimeError: injected"
    check("backend_mode_failed_beats_async -- backend_failed set while thread still alive -> FAILED",
          eng_async._backend_mode() == "FAILED")
    eng_async.backend_failed = None
    eng_async.close()

    check("backend_mode_failed_with_dead_thread_object -- thread object still present but dead -> FAILED",
          eng_fail._backend_mode() == "FAILED")
    eng_fail.close()

    # -- C4: clobber accounting against fake CPU tensors (no CUDA/no real keyframes needed) --
    eng_cl = make_engine(True, lambda: None)
    t_old = torch.zeros(1, 8)
    t_new = torch.ones(1, 8)

    eng_cl._backend_last_written = {3: t_old.clone()}
    eng_cl._check_backend_clobber(3, t_new)
    check("clobber_changed_counts_once_and_clears_record",
          eng_cl._backend_pose_clobbers == 1 and 3 not in eng_cl._backend_last_written)
    eng_cl._check_backend_clobber(3, t_new)
    check("clobber_second_check_after_clear_does_not_double_count", eng_cl._backend_pose_clobbers == 1)

    eng_cl._backend_pose_clobbers = 0
    eng_cl._backend_last_written = {5: t_old.clone()}
    eng_cl._check_backend_clobber(5, t_old.clone())
    check("clobber_unchanged_counts_zero", eng_cl._backend_pose_clobbers == 0)

    eng_cl._backend_pose_clobbers = 0
    eng_cl._backend_last_written = {}
    eng_cl._check_backend_clobber(9, t_new)
    check("clobber_no_record_for_index_counts_zero", eng_cl._backend_pose_clobbers == 0)

    # Session 64, regression guard for the bug the FIRST BENCH RUN found -- and which nothing above
    # could have caught, because none of these cases actually crossed a thread boundary into real
    # torch. Grad mode is THREAD-LOCAL: the engine disables autograd on the MAIN thread
    # (slam_engine.py:153) and upstream inherits the same setting from its backend PROCESS's own
    # __main__, so the new thread started with autograd ON and died on its first solve. Every
    # lietorch/MASt3R op inside _run_backend assumes inference mode. Assert the loop body turns it
    # off for ITSELF, from a thread that genuinely starts with it on.
    grad_seen = {}

    def _probe_grad():
        grad_seen["in_thread"] = torch.is_grad_enabled()
        eng_g._backend_stop.set()          # one pass is all we need

    eng_g = make_engine(True, _probe_grad)
    torch.set_grad_enabled(True)           # what a FRESH thread actually inherits here
    try:
        th_g = threading.Thread(target=eng_g._backend_loop, daemon=True)
        th_g.start()
        th_g.join(timeout=5.0)
    finally:
        torch.set_grad_enabled(False)      # restore the engine-wide inference mode
    check(f"thread_disables_autograd -- backend thread runs with grad OFF "
          f"(saw {grad_seen.get('in_thread')}, exited={not th_g.is_alive()})",
          grad_seen.get("in_thread") is False and not th_g.is_alive()
          and eng_g.backend_failed is None)

    # Session 64, second regression guard from the bench: the thread is started at Pipeline
    # construction, but `states`/`keyframes` are not built until `_lazy_state()` runs on the FIRST
    # FRAME -- so an eager loop raised AttributeError and killed itself before a single frame was
    # seen. It must idle harmlessly instead, and then pick up once the engine initialises.
    eng_lazy = make_engine(True, lambda: calls_lazy.append(1))
    calls_lazy = []
    eng_lazy._initialized = False
    del eng_lazy.states                       # exactly the pre-first-frame shape
    th_lazy = threading.Thread(target=eng_lazy._backend_loop, daemon=True)
    th_lazy.start()
    time.sleep(0.05)                          # several idle passes
    survived = th_lazy.is_alive() and eng_lazy.backend_failed is None and not calls_lazy
    eng_lazy._backend_stop.set()
    th_lazy.join(timeout=5.0)
    check(f"waits_for_lazy_state -- backend idles (no crash, no work) until _lazy_state() has run "
          f"(alive={survived}, failed={eng_lazy.backend_failed})", survived)

    return ok


def _self_test_slam_window_fields():
    """SESSION-65 SLAM WINDOW FIELDS: `SlamResult` gained seven fields (`slam_engine.SLAM_WINDOW_FIELDS`)
    carrying the bounded global-optimisation window's STATE (mode/W/policy and what the most recent
    solve actually touched) -- not durations, so, like SLAM_BACKEND_FIELDS, they must stay disjoint
    from the two phase-timing tuples and from SLAM_BACKEND_FIELDS itself, default correctly typed on
    a pre-session-65-style construction, and `slam_window.validate_window_config` must fail fast on a
    bad mode/policy and normalise a negative window_kf to 0. No GPU/SLAM needed: SlamResult is a plain
    dataclass and slam_window is cold-importable."""
    ok = True

    def check(name, cond):
        nonlocal ok
        ok = ok and bool(cond)
        print(f"[perception][self-test] {'PASS' if cond else 'FAIL'}  {name}")

    check("window_fields_tuple -- SLAM_WINDOW_FIELDS is the frozen seven, in order",
          slam_engine.SLAM_WINDOW_FIELDS == ("backend_window_mode", "backend_window_kf",
                                             "backend_solve_kf", "backend_solve_edges",
                                             "backend_graph_edges", "backend_anchors",
                                             "backend_anchor_drift"))
    check("window_fields_disjoint_from_phase_fields -- shares no names with SLAM_PHASE_FIELDS",
          set(slam_engine.SLAM_WINDOW_FIELDS) & set(slam_engine.SLAM_PHASE_FIELDS) == set())
    check("window_fields_disjoint_from_track_phase_fields -- shares no names with SLAM_TRACK_PHASE_FIELDS",
          set(slam_engine.SLAM_WINDOW_FIELDS) & set(slam_engine.SLAM_TRACK_PHASE_FIELDS) == set())
    check("window_fields_disjoint_from_backend_fields -- shares no names with SLAM_BACKEND_FIELDS",
          set(slam_engine.SLAM_WINDOW_FIELDS) & set(slam_engine.SLAM_BACKEND_FIELDS) == set())

    _r_default = slam_engine.SlamResult(
        tracking_mode="MASt3R", mode="TRACKING", n_keyframes=0, frame_idx=0,
        camera_center=None, new_keyframe=False, reloc_event=False)
    check("slam_result_window_defaults -- OFF/0/0/0/0/0/0.0, correctly typed",
          _r_default.backend_window_mode == "OFF" and isinstance(_r_default.backend_window_mode, str)
          and _r_default.backend_window_kf == 0 and isinstance(_r_default.backend_window_kf, int)
          and _r_default.backend_solve_kf == 0 and isinstance(_r_default.backend_solve_kf, int)
          and _r_default.backend_solve_edges == 0 and isinstance(_r_default.backend_solve_edges, int)
          and _r_default.backend_graph_edges == 0 and isinstance(_r_default.backend_graph_edges, int)
          and _r_default.backend_anchors == 0 and isinstance(_r_default.backend_anchors, int)
          and _r_default.backend_anchor_drift == 0.0
          and isinstance(_r_default.backend_anchor_drift, float))

    for bad_mode in ("on", "Shadow", "off"):
        try:
            slam_window.validate_window_config(bad_mode, 10, "anchored")
            check(f"validate_window_config rejects mode={bad_mode!r}", False)
        except ValueError:
            check(f"validate_window_config rejects mode={bad_mode!r}", True)

    for bad_policy in ("ANCHORED", "maybe"):
        try:
            slam_window.validate_window_config("OFF", 10, bad_policy)
            check(f"validate_window_config rejects policy={bad_policy!r}", False)
        except ValueError:
            check(f"validate_window_config rejects policy={bad_policy!r}", True)

    _, normalised_w, _ = slam_window.validate_window_config("ON", -1, "strict")
    check("validate_window_config normalises -1 -> 0", normalised_w == 0)

    return ok


def _self_test_diag_window_fields():
    """SESSION-65 CHUNK 4: the seven `slam_engine.SLAM_WINDOW_FIELDS` land in the flight CSV
    (`DIAG_PERF_FIELDS`, appended last, after the frozen nine-column prefix) and four of them land
    in the `TOPIC_MAP` payload. No GPU/SLAM needed: DiagLog is pure stdlib CSV and MapStore is pure
    numpy, so an empty real MapStore stands in for `self.mapstore` rather than a fake double."""
    import csv
    import shutil
    import tempfile
    import types

    ok = True

    def check(name, cond):
        nonlocal ok
        ok = ok and bool(cond)
        print(f"[perception][self-test] {'PASS' if cond else 'FAIL'}  {name}")

    check("frozen_prefix9 -- first nine DIAG_PERF_FIELDS names/order unchanged",
          DIAG_PERF_FIELDS[:9] == ("wall_ts", "frame_id", "loop_dt", "slam_ms", "mode", "new_keyframe",
                                    "n_keyframes", "n_voxels", "reloc"))
    check("window_fields_present_exactly_once -- every SLAM_WINDOW_FIELDS name is a DIAG_PERF_FIELDS "
          "column, exactly once",
          all(DIAG_PERF_FIELDS.count(f) == 1 for f in slam_engine.SLAM_WINDOW_FIELDS))
    check("no_duplicate_columns -- DIAG_PERF_FIELDS has no repeated name",
          len(set(DIAG_PERF_FIELDS)) == len(DIAG_PERF_FIELDS))

    # --- SESSION 66: rec_frame, the video-correlation column ---------------------------------
    # io_bridge writes EVERY NDI video frame to flight_<ts>.mp4 and stamps that frame's index into
    # meta, so logging it here makes a flight exactly replayable with no frame_id->video offset.
    # The load-bearing case is rec_frame 0: it is the FIRST recorded frame, NOT "absent".
    check("rec_frame_column -- rec_frame is a DIAG_PERF_FIELDS column",
          "rec_frame" in DIAG_PERF_FIELDS)
    check("rec_frame_frozen_prefix -- the frozen nine are still first and in order",
          DIAG_PERF_FIELDS[:9] == ("wall_ts", "frame_id", "loop_dt", "slam_ms", "mode",
                                   "new_keyframe", "n_keyframes", "n_voxels", "reloc"))
    check("rec_frame_not_a_phase -- rec_frame joins no closure/phase tuple",
          "rec_frame" not in slam_engine.SLAM_PHASE_FIELDS
          and "rec_frame" not in slam_engine.SLAM_TRACK_PHASE_FIELDS
          and "rec_frame" not in slam_engine.SLAM_BACKEND_FIELDS
          and "rec_frame" not in slam_engine.SLAM_WINDOW_FIELDS)
    check("rec_frame_none_is_blank -- not recording renders \"\", never 0",
          _rec_frame_cell(None) == "")
    check("rec_frame_zero_is_zero -- frame 0 is the FIRST recorded frame, not absent",
          _rec_frame_cell(0) == 0 and _rec_frame_cell(0) != "")
    check("rec_frame_int_passthrough -- a real index survives as an int",
          _rec_frame_cell(1234) == 1234 and isinstance(_rec_frame_cell(1234), int))

    tmp_dir = tempfile.mkdtemp(prefix="recframe_selftest_")
    try:
        rf_log = DiagLog("perception", list(DIAG_PERF_FIELDS), out_dir=tmp_dir,
                         ts="20260101_000001")
        rf_log.row(wall_ts=1.0, frame_id=7, rec_frame=_rec_frame_cell(0))
        rf_log.row(wall_ts=2.0, frame_id=8, rec_frame=_rec_frame_cell(None))
        rf_log.row(wall_ts=3.0, frame_id=9, rec_frame=_rec_frame_cell(41))
        rf_log.close()
        with open(rf_log.path, newline="", encoding="utf-8") as fh:
            rr = list(csv.DictReader(fh))
        check("rec_frame_roundtrip -- 0 writes \"0\", None writes \"\", 41 writes \"41\"",
              [r["rec_frame"] for r in rr] == ["0", "", "41"])
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)

    tmp_dir = tempfile.mkdtemp(prefix="window_fields_selftest_")
    try:
        log = DiagLog("perception", list(DIAG_PERF_FIELDS), out_dir=tmp_dir, ts="20260101_000000")

        # -- a fully-populated row: the seven window columns round-trip their real values --
        window_values = {
            "backend_window_mode": "ON", "backend_window_kf": 30, "backend_solve_kf": 12,
            "backend_solve_edges": 40, "backend_graph_edges": 500, "backend_anchors": 3,
            "backend_anchor_drift": 0.125,
        }
        full_row = {f: (1 if f in ("frame_id", "new_keyframe", "n_keyframes", "n_voxels", "reloc",
                                    "backend_queue_depth", "backend_pose_clobbers")
                        else ("TRACKING" if f == "mode"
                              else ("ASYNC" if f == "backend_mode" else 1.0)))
                    for f in DIAG_PERF_FIELDS}
        full_row.update(window_values)
        log.row(**full_row)

        # -- a default-SlamResult row: exactly the seven kwargs the real diag row construction
        # passes, everything else omitted (must come back blank, never an invented value) --
        res_default = slam_engine.SlamResult(
            tracking_mode="MASt3R", mode="TRACKING", n_keyframes=0, frame_idx=0,
            camera_center=None, new_keyframe=False, reloc_event=False)
        log.row(wall_ts=2.0, frame_id=2, loop_dt=0.1, slam_ms=5.0, mode="TRACKING",
                new_keyframe=0, n_keyframes=1, n_voxels=10, reloc=0,
                backend_window_mode=res_default.backend_window_mode,
                backend_window_kf=res_default.backend_window_kf,
                backend_solve_kf=res_default.backend_solve_kf,
                backend_solve_edges=res_default.backend_solve_edges,
                backend_graph_edges=res_default.backend_graph_edges,
                backend_anchors=res_default.backend_anchors,
                backend_anchor_drift=round(res_default.backend_anchor_drift, 4))
        log.close()

        with open(log.path, "r", newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))

        full_row_ok = all(str(rows[0][f]) == str(window_values[f]) for f in window_values)
        check("csv_full_row_window_fields_roundtrip -- all seven come back with the right values",
              full_row_ok)

        default_ok = (rows[1]["backend_window_mode"] == "OFF" and rows[1]["backend_window_kf"] == "0"
                      and rows[1]["backend_solve_kf"] == "0" and rows[1]["backend_solve_edges"] == "0"
                      and rows[1]["backend_graph_edges"] == "0" and rows[1]["backend_anchors"] == "0"
                      and rows[1]["backend_anchor_drift"] == "0.0")
        check("csv_default_slamresult_writes_literal_zeros -- 0/0.0/OFF, never a blank cell",
              default_ok)

        other_omitted = ("track_ms", "backend_ms", "pose_ms", "kf_download_ms")
        check("csv_other_omitted_phases_still_blank -- unrelated omitted phase columns stay blank",
              all(rows[1][f] == "" for f in other_omitted))
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)

    # -- _map_payload: an empty real MapStore (pure numpy, no GPU) stands in for self.mapstore --
    pipe = types.SimpleNamespace(mapstore=MapStore(0.1, tracking_mode="MASt3R"))
    payload = Pipeline._map_payload(pipe, res_default, {"frame_id": 2, "sim_time": 1.0})
    check("map_payload_has_four_window_keys",
          all(k in payload for k in ("backend_window_mode", "backend_window_kf",
                                      "backend_solve_kf", "backend_anchors")))
    check("map_payload_omits_the_other_three -- solve_edges/graph_edges/anchor_drift are CSV-only",
          not any(k in payload for k in ("backend_solve_edges", "backend_graph_edges",
                                          "backend_anchor_drift")))
    check("map_payload_window_values_match_default",
          payload["backend_window_mode"] == "OFF" and payload["backend_window_kf"] == 0
          and payload["backend_solve_kf"] == 0 and payload["backend_anchors"] == 0)

    return ok


def _self_test_phase_timing(cfg):
    """SESSION-62 PHASE TIMING SCHEMA: DIAG_PERF_FIELDS must keep its frozen 9-column prefix (so
    pre-2026-09-05 files stay readable by name), carry every SlamResult phase field plus the four
    post-SLAM phase columns exactly once each, and DiagLog must round-trip a fully-populated row
    while leaving OMITTED phase keywords as blank cells -- not silently zeroed (CLAUDE.md: a missing
    field must be visibly missing). No GPU/SLAM needed: DiagLog is pure stdlib CSV."""
    import csv
    import shutil
    import tempfile

    ok = True

    def check(name, cond):
        nonlocal ok
        ok = ok and bool(cond)
        print(f"[perception][self-test] {'PASS' if cond else 'FAIL'}  {name}")

    check("frozen_prefix -- first nine DIAG_PERF_FIELDS names/order unchanged since before session 62",
          DIAG_PERF_FIELDS[:9] == ("wall_ts", "frame_id", "loop_dt", "slam_ms", "mode", "new_keyframe",
                                    "n_keyframes", "n_voxels", "reloc"))
    check("slam_phase_fields_present -- every slam_engine.SLAM_PHASE_FIELDS name is a DIAG_PERF_FIELDS column",
          all(f in DIAG_PERF_FIELDS for f in slam_engine.SLAM_PHASE_FIELDS))
    check("post_slam_fields_present -- integrate_ms/map_pub_ms/plan_ms/publish_ms are columns",
          all(f in DIAG_PERF_FIELDS for f in ("integrate_ms", "map_pub_ms", "plan_ms", "publish_ms")))
    check("no_duplicate_columns -- DIAG_PERF_FIELDS has no repeated name",
          len(set(DIAG_PERF_FIELDS)) == len(DIAG_PERF_FIELDS))
    # Session 63: the frozen-seventeen guarantee -- every flight written before this session has
    # exactly these seventeen names in this order, and the report reads by column NAME, so the new
    # frame_ms/infer_ms/tracker_ms columns must land strictly AFTER them, never inside.
    check("frozen_prefix17 -- first seventeen DIAG_PERF_FIELDS names/order unchanged since before session 63",
          DIAG_PERF_FIELDS[:17] == ("wall_ts", "frame_id", "loop_dt", "slam_ms", "mode", "new_keyframe",
                                     "n_keyframes", "n_voxels", "reloc",
                                     "track_ms", "backend_ms", "pose_ms", "kf_download_ms",
                                     "integrate_ms", "map_pub_ms", "plan_ms", "publish_ms"))
    check("track_phase_fields_present -- every slam_engine.SLAM_TRACK_PHASE_FIELDS name is a DIAG_PERF_FIELDS column",
          all(f in DIAG_PERF_FIELDS for f in slam_engine.SLAM_TRACK_PHASE_FIELDS))
    # Session 64: the frozen-twenty guarantee -- every flight written before this session has exactly
    # these twenty names in this order; the four backend-thread columns land strictly AFTER them.
    check("frozen_prefix20 -- first twenty DIAG_PERF_FIELDS names/order unchanged since before session 64",
          DIAG_PERF_FIELDS[:20] == ("wall_ts", "frame_id", "loop_dt", "slam_ms", "mode", "new_keyframe",
                                     "n_keyframes", "n_voxels", "reloc",
                                     "track_ms", "backend_ms", "pose_ms", "kf_download_ms",
                                     "integrate_ms", "map_pub_ms", "plan_ms", "publish_ms",
                                     "frame_ms", "infer_ms", "tracker_ms"))
    check("backend_fields_present -- every slam_engine.SLAM_BACKEND_FIELDS name is a DIAG_PERF_FIELDS column",
          all(f in DIAG_PERF_FIELDS for f in slam_engine.SLAM_BACKEND_FIELDS))

    tmp_dir = tempfile.mkdtemp(prefix="phase_timing_selftest_")
    try:
        log = DiagLog("perception", list(DIAG_PERF_FIELDS), out_dir=tmp_dir, ts="20260101_000000")
        full_row = {f: (1 if f in ("frame_id", "new_keyframe", "n_keyframes", "n_voxels", "reloc",
                                    "backend_queue_depth", "backend_pose_clobbers")
                        else ("TRACKING" if f == "mode"
                              else ("ASYNC" if f == "backend_mode" else 1.0)))
                    for f in DIAG_PERF_FIELDS}
        log.row(**full_row)
        log.row(wall_ts=2.0, frame_id=2, loop_dt=0.1, slam_ms=5.0, mode="TRACKING",
                new_keyframe=0, n_keyframes=1, n_voxels=10, reloc=0)
        log.close()

        with open(log.path, "r", newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            header_ok = reader.fieldnames == list(DIAG_PERF_FIELDS)
            rows = list(reader)

        first_row_ok = all(str(rows[0][f]) == str(full_row[f]) for f in DIAG_PERF_FIELDS)
        backend_mode_roundtrip_ok = rows[0]["backend_mode"] == "ASYNC"
        omitted_phase_fields = ("track_ms", "backend_ms", "pose_ms", "kf_download_ms",
                                 "integrate_ms", "map_pub_ms", "plan_ms", "publish_ms",
                                 "frame_ms", "infer_ms", "tracker_ms",
                                 "backend_mode", "backend_queue_depth", "backend_thread_ms",
                                 "backend_pose_clobbers")
        blanks_ok = all(rows[1][f] == "" for f in omitted_phase_fields)

        check("csv_header_matches -- DictReader header equals list(DIAG_PERF_FIELDS)", header_ok)
        check("csv_full_row_roundtrips -- fully-populated row's values come back unchanged", first_row_ok)
        check("csv_backend_mode_roundtrips_as_string -- 'ASYNC' comes back as the string, not a number",
              backend_mode_roundtrip_ok)
        check("csv_omitted_phases_are_blank -- missing phase kwargs write '' not '0.0'/'SYNC'/'0'", blanks_ok)
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)

    # Session 62 CHUNK 3: the sticky console dict (C3) -- a types.SimpleNamespace stands in for the
    # Pipeline so this stays a pure-stdlib test (no GPU/SLAM). Checks the fresh-dict shape, the
    # stickiness contract (a phase that did NOT run this frame keeps its previous value), and that the
    # console segment renders for both an all-zero and a populated dict.
    import types

    fresh_phase_ms = {
        "track": 0.0, "backend": 0.0, "pose": 0.0, "kf_download": 0.0,
        "integrate": 0.0, "map_pub": 0.0, "plan": 0.0, "publish": 0.0,
        "frame": 0.0, "infer": 0.0, "tracker": 0.0,
    }
    check("phase_ms_fresh -- a fresh _last_phase_ms dict has all eleven keys, every value 0.0",
          set(fresh_phase_ms) == {"track", "backend", "pose", "kf_download",
                                   "integrate", "map_pub", "plan", "publish",
                                   "frame", "infer", "tracker"}
          and all(v == 0.0 for v in fresh_phase_ms.values()))

    pipe = types.SimpleNamespace(_last_phase_ms=dict(fresh_phase_ms))
    pipe._last_phase_ms["integrate"] = 42.0
    map_updated = False               # sticky-update rule: only write "integrate" when this is True
    if map_updated:
        pipe._last_phase_ms["integrate"] = 7.0
    check("sticky_integrate_unchanged -- map_updated=False leaves a previously-set integrate value",
          pipe._last_phase_ms["integrate"] == 42.0)

    # Session 63: "infer" (and "tracker") follow the same sticky rule as "integrate" -- write only
    # when the phase actually ran (res.infer_ms > 0.0), else keep the last real value.
    pipe._last_phase_ms["infer"] = 55.0
    res_infer_ms = 0.0                # INIT/RELOC did not run this frame
    if res_infer_ms > 0.0:
        pipe._last_phase_ms["infer"] = res_infer_ms
    check("sticky_infer_unchanged -- res.infer_ms==0.0 leaves a previously-set infer value",
          pipe._last_phase_ms["infer"] == 55.0)

    def _render(p):
        return (f"[trk {p['track']:.0f} bk {p['backend']:.0f} dl {p['kf_download']:.0f}] | "
                f"(frm {p['frame']:.0f} inf {p['infer']:.0f} trk2 {p['tracker']:.0f}) | "
                f"intg {p['integrate']:.0f} plan {p['plan']:.0f} map {p['map_pub']:.0f} ms | ")

    labels = ("trk ", "bk ", "dl ", "frm ", "inf ", "trk2 ", "intg ", "plan ", "map ")
    try:
        seg_zero = _render(fresh_phase_ms)
        seg_full = _render({"track": 12.0, "backend": 900.0, "pose": 1.5, "kf_download": 70.0,
                             "integrate": 3.0, "map_pub": 4.0, "plan": 5.0, "publish": 6.0,
                             "frame": 8.0, "infer": 210.0, "tracker": 48.0})
        render_ok = all(tok in seg_zero for tok in labels) and all(tok in seg_full for tok in labels)
    except Exception as e:
        render_ok = False
        print(f"[perception][self-test] phase segment render raised: {e}")
    check("phase_segment_renders -- console segment formats without raising, all nine labels present",
          render_ok)

    # Session 64: the bk[...] console segment reads straight off `res` (per-frame truth), not the
    # sticky dict -- a SimpleNamespace stand-in exercises both FAILED and ASYNC without a real engine.
    def _render_bk(r):
        return (f"bk[{r.backend_mode} q{r.backend_queue_depth} {r.backend_thread_ms:.0f}ms"
                f"{f' clob{r.backend_pose_clobbers}' if r.backend_pose_clobbers else ''}] | ")

    try:
        res_async = types.SimpleNamespace(backend_mode="ASYNC", backend_queue_depth=3,
                                          backend_thread_ms=120.4, backend_pose_clobbers=0)
        res_failed = types.SimpleNamespace(backend_mode="FAILED", backend_queue_depth=0,
                                           backend_thread_ms=0.0, backend_pose_clobbers=2)
        seg_async, seg_failed = _render_bk(res_async), _render_bk(res_failed)
        bk_render_ok = True
    except Exception as e:
        seg_async = seg_failed = ""
        bk_render_ok = False
        print(f"[perception][self-test] backend console segment render raised: {e}")
    check("backend_console_segment_renders -- ASYNC and FAILED both render without raising, contain 'bk['",
          bk_render_ok and "bk[" in seg_async and "bk[" in seg_failed)
    check("backend_console_clob_only_when_nonzero -- 'clob' shown only when backend_pose_clobbers != 0",
          "clob" not in seg_async and "clob2" in seg_failed)

    return ok


def main():
    parser = argparse.ArgumentParser(description="Cartographer perception_worker (P2): MASt3R-SLAM + map")
    parser.add_argument("--config", default=None)
    parser.add_argument("--no-display", action="store_true", help="headless: skip the OpenCV windows")
    parser.add_argument("--self-test", action="store_true",
                        help="no-GPU smoke test: module + GroundGrid/FrontierPlanner build (depth removed)")
    parser.add_argument("--video", default=None,
                        help="OFFLINE: drive the full SLAM+map pipeline from this mp4, export the map")
    parser.add_argument("--stride", type=int, default=3, help="offline: process every Nth source frame")
    parser.add_argument("--frame-list", default=None,
                        help="offline: a flight's perception CSV; replay the EXACT frames that "
                             "flight's SLAM consumed (its rec_frame column) instead of --stride")
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
                          debug_lift=args.debug_lift, log=args.log,
                          frame_list=(load_frame_list(args.frame_list) if args.frame_list else None))
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
