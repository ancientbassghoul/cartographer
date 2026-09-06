"""slam_engine.py — production wrapper around the MASt3R-SLAM driving loop.

Encapsulates what `slam_offline.py` proved (INIT/TRACKING/RELOC + FactorGraph backend +
retrieval loop-closure, single-process via an in-process manager shim) behind a small
**streaming API** so the live `perception_worker` can drive SLAM one frame at a time and
pull each new keyframe's world pointmap straight into `map_store.MapStore`.

`SlamEngine.process(rgb)` runs exactly one tracker step + backend pass and returns a
compact `SlamResult` — current camera centre, mode, keyframe count, and (only on a new
keyframe) that keyframe's confidence-filtered world points + colors. The caller decides
what to do with them (integrate into the map, publish a pose on the bus).

Lazy, side-effect-free import: every MASt3R-SLAM import and the `os.chdir` into the repo
(it resolves `checkpoints/` and `config/` relatively) happen inside `__init__`, never at
module import — so `import slam_engine` costs nothing and does not disturb cwd. State
buffers (sized to the model's working resolution) are built on the first frame.

NO SILENT FALLBACKS (per CLAUDE.md): CUDA + every checkpoint load fail fast. The SLAM mode
(INIT/TRACKING/RELOC) is the engine's own explicit state, surfaced in every `SlamResult`,
never hidden or auto-absorbed.
"""

import os
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import torch
import lietorch

CARTO = Path(__file__).resolve().parent
SLAM_REPO = CARTO / "third_party" / "MASt3R-SLAM"


# In-process stand-in for mp.Manager (Windows mp.Manager() deadlocks here — see
# slam_offline.py for the full rationale; we run tracker + backend in ONE process).
class _Value:
    def __init__(self, value):
        self.value = value


class InProcessManager:
    def RLock(self):
        return threading.RLock()

    def Value(self, typecode, value):
        return _Value(value)

    def list(self):
        return []


@dataclass
class SlamResult:
    tracking_mode: str            # always "MASt3R" here (visible NO-FALLBACK flag)
    mode: str                     # "INIT" | "TRACKING" | "RELOC" (engine's own state)
    n_keyframes: int
    frame_idx: int
    camera_center: np.ndarray | None  # (3,) world coords of the current frame's camera
    new_keyframe: bool
    reloc_event: bool             # tracking was lost this frame -> entered RELOC
    pose: np.ndarray | None = None        # (4,4) world<-camera Sim3 matrix [[sR,t],[0,1]] this frame
    kf_points: np.ndarray | None = None   # (N,3) world points of the NEW keyframe only
    kf_colors: np.ndarray | None = None   # (N,3) uint8, paired with kf_points
    # Session 62 — per-phase wall-clock split of what `slam_ms` measures, so the choke can be
    # attributed instead of guessed. Always present; 0.0 = the phase did not run this frame.
    track_ms: float = 0.0          # frame construction + the INIT/TRACKING/RELOC mode branch
    backend_ms: float = 0.0        # _run_backend(): retrieval update + add_factors + solve_GN_*
    pose_ms: float = 0.0           # pose recovery (Act3 basis -> numpy pose_mat + center)
    kf_download_ms: float = 0.0    # new-keyframe GPU->CPU pull (X_canon, pW, conf, uimg) + ray field
    # Session 63 — the split INSIDE track_ms. Once the backend leaves the frame path (phase 2),
    # this is the whole budget; measure it before optimising it, not after.
    frame_ms: float = 0.0      # self._create_frame(...) — runs on EVERY frame
    infer_ms: float = 0.0      # _mast3r_inference_mono — INIT and RELOC branches only
    tracker_ms: float = 0.0    # self.tracker.track(frame) — TRACKING branch only
    # Session 64 — the backend is no longer on the frame path, so its state must be reported
    # explicitly rather than inferred from a timing column that is now always 0.0 (CLAUDE.md 2+3).
    backend_mode: str = "SYNC"          # "SYNC" | "ASYNC" | "FAILED"
    backend_queue_depth: int = 0        # global_optimizer_tasks depth at this frame
    backend_thread_ms: float = 0.0      # the thread's most recent completed solve (0.0 in SYNC)
    backend_pose_clobbers: int = 0      # cumulative; see C4
    # Session 64 (operator ask, after two bench failures): the CRITICAL line naming WHY the thread
    # died is printed to a console that fly.py opens with CREATE_NEW_CONSOLE and never captures, so
    # it dies with the window. `backend_mode == "FAILED"` survives in the CSV but says only THAT it
    # failed; this carries the reason, so a flight stays diagnosable from artifacts alone with
    # nobody having had to be watching. Empty string whenever the backend is healthy.
    backend_error: str = ""


# Session 62: the phase names in SlamResult, in pipeline order. Single source of truth shared by
# perception_worker's CSV schema and the timing report — never re-type this list anywhere.
SLAM_PHASE_FIELDS: tuple[str, ...] = ("track_ms", "backend_ms", "pose_ms", "kf_download_ms")

# Session 63: the sub-split INSIDE track_ms, in pipeline order. Kept SEPARATE from
# SLAM_PHASE_FIELDS because those four close against slam_ms and these three close against
# track_ms — two different invariants, and merging them would break both.
SLAM_TRACK_PHASE_FIELDS: tuple[str, ...] = ("frame_ms", "infer_ms", "tracker_ms")

# Session 64: backend-thread state carried on every SlamResult. NOT phase timings -- two of the
# four are not durations at all -- so they are deliberately kept out of SLAM_PHASE_FIELDS and
# SLAM_TRACK_PHASE_FIELDS, whose members close against slam_ms and track_ms respectively.
SLAM_BACKEND_FIELDS: tuple[str, ...] = ("backend_mode", "backend_queue_depth",
                                        "backend_thread_ms", "backend_pose_clobbers",
                                        "backend_error")

# Session 64: how much of a backend exception text rides the CSV. Long enough for an exception type
# plus a useful message, short enough that a per-frame column stays readable in a spreadsheet; the
# console CRITICAL line carries the untruncated text for anyone watching live.
BACKEND_ERROR_MAX_CHARS: int = 200


def _flatten_backend_error(text):
    """Session 64: make an exception text safe for ONE CSV cell -- newlines and tabs collapsed to
    spaces, then truncated to BACKEND_ERROR_MAX_CHARS with an explicit ellipsis so a reader can tell
    truncation happened rather than guessing (NO SILENT FALLBACKS applies to diagnostics too).
    Returns "" for None/empty."""
    if not text:
        return ""
    flat = " ".join(str(text).split())
    if len(flat) > BACKEND_ERROR_MAX_CHARS:
        return flat[: BACKEND_ERROR_MAX_CHARS - 3] + "..."
    return flat

# Session 64: poll gap for the backend thread when global_optimizer_tasks is empty and the mode
# is not RELOC. This is a responsiveness/CPU-spin tradeoff, not a room- or flight-specific value --
# it bounds how quickly a newly-queued keyframe gets picked up, nothing about the XLAB itself.
# Session 64 POST-FLIGHT FIX: was 0.005. The idle pass takes states.lock TWICE (_queue_depth
# + get_mode), so a 5ms gap hammered that lock ~400x/second against a frame path that needs it
# for its own get_mode()/set_frame() -- pure contention, burned while the backend had nothing
# to do. Solves run 14s median, so even 50ms of extra start latency is ~0.3% of one solve; the
# lock pressure drops 10x. This is a poll gap, not a timeout: a queued task is still picked up
# within one gap.
BACKEND_IDLE_SLEEP_S: float = 0.05


class SlamEngine:
    def __init__(self, device="cuda:0", config_path="config/base.yaml", conf_thresh=1.5,
                 backend_async: bool = True):
        assert torch.cuda.is_available(), "CUDA required for SLAM (NO SILENT FALLBACKS)."
        self.device = device
        self.conf_thresh = conf_thresh
        self.tracking_mode = "MASt3R"

        # Session 64: global optimization moves off the frame path onto its own thread (upstream
        # runs this in a separate PROCESS; Windows mp.Manager() deadlocks here, so a thread is the
        # closest equivalent -- see InProcessManager above). backend_async=False restores the prior
        # inline call exactly, for A/B comparison and as the escape hatch NO SILENT FALLBACKS
        # requires: a dead thread sets backend_failed and STAYS dead, it never reverts to inline.
        self.backend_async: bool = bool(backend_async)
        self.backend_failed: str | None = None   # None | the exception text that killed the thread
        self._backend_thread: threading.Thread | None = None
        self._backend_stop = threading.Event()
        self._backend_meta_lock = threading.Lock()   # guards the four fields below ONLY
        self._backend_thread_ms: float = 0.0     # duration of the thread's most recent solve
        self._backend_solves: int = 0            # completed solves this flight
        self._backend_pose_clobbers: int = 0     # cumulative tracker-overwrites-backend races (chunk 2)
        self._backend_last_written: dict[int, "torch.Tensor"] = {}
        # Session 64 POST-FLIGHT FIX (flight 20260906_000915): `process()` used to call
        # `_queue_depth()` -- which takes `states.lock` -- once per frame purely to report a number
        # on the console. The backend thread holds that same lock across its solve, so the frame
        # path blocked on it: 685.3s, 11.7% of that flight, spent waiting to read an int. It landed
        # OUTSIDE every phase stamp, which is why the session-62 closure residual jumped from ~0 to
        # p90 2839ms and localised it immediately. The depth is now PUBLISHED by whoever runs the
        # backend, and the frame path only ever reads this cached copy under the private meta lock.
        self._backend_queue_depth: int = 0
        # Same treatment for the keyframe count. `len(self.keyframes)` takes keyframes.lock, and the
        # frame path called it on EVERY frame just to fill SlamResult.n_keyframes -- a REPORTING
        # field only (CSV column, console line, TOPIC_POSE/TOPIC_MAP payloads; nothing decides on
        # it). `_relocalization` holds that lock across an entire reloc solve, so a free read became
        # a multi-second stall. Published by whoever already holds the lock; the frame path reads
        # this copy. Exact on keyframe frames, at most one backend poll (5ms) stale otherwise.
        self._kf_count: int = 0   # idx -> T_WC clone (chunk 2)

        # The repo resolves checkpoints/ and config/ relatively, so run from its root.
        os.chdir(SLAM_REPO)
        if str(SLAM_REPO) not in sys.path:
            sys.path.insert(0, str(SLAM_REPO))

        from mast3r_slam.config import load_config, config
        from mast3r_slam.frame import Mode, SharedKeyframes, SharedStates, create_frame
        from mast3r_slam.mast3r_utils import (
            load_mast3r, load_retriever, mast3r_inference_mono,
        )
        from mast3r_slam.tracker import FrameTracker
        from mast3r_slam.global_opt import FactorGraph

        self._Mode = Mode
        self._create_frame = create_frame
        self._mast3r_inference_mono = mast3r_inference_mono
        self._SharedKeyframes = SharedKeyframes
        self._SharedStates = SharedStates
        self._FrameTracker = FrameTracker
        self._FactorGraph = FactorGraph

        torch.backends.cuda.matmul.allow_tf32 = True
        torch.set_grad_enabled(False)
        load_config(config_path)
        config["use_calib"] = False
        self._config = config

        self.model = load_mast3r(device=device)
        self.retrieval_database = load_retriever(self.model)

        self._initialized = False     # state buffers built on the first frame
        self._i = 0
        self.n_keyframes = 0
        self.n_reloc = 0
        self._origin = torch.zeros(1, 3, device=device)
        # Origin + 3 unit axes, acted on by T_WC to recover the full pose via Act3 only.
        # (T_WC.matrix() routes through Act4 on a view of the pose data, which corrupts the
        # frame pose under the patched lietorch and freezes keyframe motion — Act3 is safe.)
        self._pose_basis = torch.tensor(
            [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=torch.float32, device=device)
        # Per-pixel camera viewing rays (unit dirs in the camera frame), refreshed from each
        # keyframe's canonical pointmap. Intrinsics are fixed (one sim camera), so this field is
        # ~view-independent and can be reused to back-project a detection on any frame.
        self.ray_field = None         # (h, w, 3) float32, or None until the first keyframe
        self.ray_hw = None            # (h, w) of ray_field

    # ------------------------------------------------------------------ setup
    def _lazy_state(self, first_rgb):
        """Size the shared keyframe/state buffers to MASt3R's working resolution."""
        probe = self._create_frame(
            0, first_rgb, lietorch.Sim3.Identity(1, device=self.device),
            img_size=512, device=self.device)
        self.h = int(probe.img_true_shape.flatten()[0])
        self.w = int(probe.img_true_shape.flatten()[1])
        mgr = InProcessManager()
        self.keyframes = self._SharedKeyframes(mgr, self.h, self.w)
        self.states = self._SharedStates(mgr, self.h, self.w)
        self.tracker = self._FrameTracker(self.model, self.keyframes, self.device)
        self.factor_graph = self._FactorGraph(self.model, self.keyframes, None, self.device)
        self._initialized = True

    # ------------------------------------------------------------- backend
    def _relocalization(self, frame):
        cfg = self._config
        keyframes, factor_graph = self.keyframes, self.factor_graph
        with keyframes.lock:
            retrieval_inds = self.retrieval_database.update(
                frame, add_after_query=False,
                k=cfg["retrieval"]["k"], min_thresh=cfg["retrieval"]["min_thresh"])
            kf_idx = list(retrieval_inds)
            success = False
            if kf_idx:
                keyframes.append(frame)
                n_kf = len(keyframes)
                frame_idx = [n_kf - 1] * len(kf_idx)
                if factor_graph.add_factors(
                        frame_idx, kf_idx, cfg["reloc"]["min_match_frac"],
                        is_reloc=cfg["reloc"]["strict"]):
                    self.retrieval_database.update(
                        frame, add_after_query=True,
                        k=cfg["retrieval"]["k"], min_thresh=cfg["retrieval"]["min_thresh"])
                    success = True
                    keyframes.T_WC[n_kf - 1] = keyframes.T_WC[kf_idx[0]].clone()
                else:
                    keyframes.pop_last()
            if success:
                if cfg["use_calib"]:
                    factor_graph.solve_GN_calib()
                else:
                    factor_graph.solve_GN_rays()
            return success

    def _run_backend(self):
        Mode, cfg = self._Mode, self._config
        states, keyframes, factor_graph = self.states, self.keyframes, self.factor_graph
        mode = states.get_mode()
        if mode == Mode.INIT or states.is_paused():
            return
        if mode == Mode.RELOC:
            frame = states.get_frame()
            if self._relocalization(frame):
                states.set_mode(Mode.TRACKING)
            states.dequeue_reloc()
            return

        idx = -1
        with states.lock:
            if len(states.global_optimizer_tasks) > 0:
                idx = states.global_optimizer_tasks[0]
        if idx == -1:
            return

        # previous consecutive keyframe + retrieval (loop closure) candidates
        kf_idx = []
        for j in range(min(1, idx)):
            kf_idx.append(idx - 1 - j)
        frame = keyframes[idx]
        retrieval_inds = self.retrieval_database.update(
            frame, add_after_query=True,
            k=cfg["retrieval"]["k"], min_thresh=cfg["retrieval"]["min_thresh"])
        kf_idx += retrieval_inds

        kf_idx = set(kf_idx)
        kf_idx.discard(idx)
        kf_idx = list(kf_idx)
        if kf_idx:
            factor_graph.add_factors(kf_idx, [idx] * len(kf_idx),
                                     cfg["local_opt"]["min_match_frac"])
        with states.lock:
            states.edges_ii[:] = factor_graph.ii.cpu().tolist()
            states.edges_jj[:] = factor_graph.jj.cpu().tolist()
        if cfg["use_calib"]:
            factor_graph.solve_GN_calib()
        else:
            factor_graph.solve_GN_rays()
        with states.lock:
            if len(states.global_optimizer_tasks) > 0:
                states.global_optimizer_tasks.pop(0)

    # ------------------------------------------------------- backend thread
    def _queue_depth(self) -> int:
        """global_optimizer_tasks depth, read under the SAME lock _run_backend uses."""
        with self.states.lock:
            return len(self.states.global_optimizer_tasks)

    def _backend_mode(self) -> str:
        """C2: FAILED beats ASYNC beats SYNC -- a dead thread object can still be sitting in
        self._backend_thread (NO SILENT FALLBACKS: we never clear backend_failed or the thread
        reference just because the loop exited), so backend_failed must be checked first."""
        if self.backend_failed is not None:
            return "FAILED"
        if self._backend_thread is not None and self._backend_thread.is_alive():
            return "ASYNC"
        return "SYNC"

    def _check_backend_clobber(self, idx: int, current_T_WC: "torch.Tensor") -> None:
        """C4: the tracker's write-back (tracker.py:101 -> keyframes[idx] = value, which assigns
        the WHOLE row including pose) can overwrite a pose the backend just solved for the same
        keyframe. This does not repair it -- the factor graph re-optimises next solve regardless --
        it only counts the race so flight telemetry says whether it matters (session64-spec.md)."""
        with self._backend_meta_lock:
            recorded = self._backend_last_written.get(idx)
            if recorded is None:
                return
            if not torch.equal(recorded, current_T_WC):
                self._backend_pose_clobbers += 1
                del self._backend_last_written[idx]

    def _poll_backend_clobber(self) -> None:
        """Session 64 POST-FLIGHT FIX: run the C4 clobber check on the BACKEND thread instead of the
        frame path. It used to sit in `process()` right after `tracker.track()`, taking
        `keyframes.lock` twice per frame (`len()` takes it too) while the backend held it across a
        solve -- 102.3s, 1.7% of flight 20260906_000915, and it inflated `track_ms`'s own closure
        residual to p90 285ms / max 19.6s. The information is identical from here: the tracker's
        write-back happens between backend passes, so comparing at the top of each pass catches
        exactly the same overwrites.

        Lock discipline unchanged and still non-nested: meta (copy) -> release -> keyframes (read)
        -> release -> meta (via _check_backend_clobber). Never holds two at once."""
        with self._backend_meta_lock:
            recorded = dict(self._backend_last_written)
        if not recorded:
            return
        live = {}
        with self.keyframes.lock:
            n = len(self.keyframes)
            for idx in recorded:
                if 0 <= idx < n:
                    live[idx] = self.keyframes.T_WC[idx].detach().clone()
        for idx, pose in live.items():
            self._check_backend_clobber(idx, pose)

    def start_backend(self) -> bool:
        """Start the backend thread. Returns True if it started, False when backend_async is off.
        Idempotent: a second call while the thread is alive is a no-op returning True."""
        if not self.backend_async:
            return False
        if self._backend_thread is not None and self._backend_thread.is_alive():
            return True
        self._backend_stop.clear()
        self._backend_thread = threading.Thread(
            target=self._backend_loop, name="slam-backend", daemon=True)
        self._backend_thread.start()
        return True

    def close(self, timeout: float = 10.0) -> None:
        """Stop and join the backend thread. Safe to call when it was never started, and safe to
        call twice. Never raises."""
        self._backend_stop.set()
        thread = self._backend_thread
        self._backend_thread = None
        if thread is not None:
            thread.join(timeout=timeout)

    def _backend_loop(self) -> None:
        """Thread body: drain global_optimizer_tasks by calling the EXISTING _run_backend(),
        unchanged. Session 64: this is what moves the 64.4%-of-loop backend cost (see
        plans/session64-spec.md) off the frame path -- _run_backend already takes every lock it
        needs (states.lock / keyframes.lock, both real threading.RLocks via InProcessManager), so
        this loop adds none of its own around it.

        Session 64, found by the FIRST BENCH RUN: torch's grad mode and current CUDA device are
        THREAD-LOCAL. `__init__` runs `torch.set_grad_enabled(False)` on the MAIN thread
        (slam_engine.py:153), and upstream gets the same setting from its backend PROCESS's own
        `__main__` (third_party/MASt3R-SLAM/main.py:137) -- neither reaches this thread, which
        therefore started with autograd ON and died on its first solve, exactly as the console's
        `bk[FAILED]` reported. Every MASt3R-SLAM / lietorch op below assumes inference mode.
        This is the one hazard that appears ONLY once the backend leaves the frame path, which
        is why it is set here and nowhere else."""
        torch.set_grad_enabled(False)
        if torch.cuda.is_available():          # the engine asserts CUDA at __init__; this guard
            torch.cuda.set_device(self.device)  # exists only so the self-test double can run
        while not self._backend_stop.is_set():
            try:
                # Session 64, second bench finding: `states` / `keyframes` do not exist until
                # `_lazy_state()` builds them on the FIRST FRAME (slam_engine.py:178) -- but the
                # caller starts this thread at Pipeline construction, long before any frame arrives.
                # Upstream never meets this: its buffers are built before the backend process is
                # spawned. Idle until the engine is actually initialised rather than touching
                # attributes that are not there yet.
                if not self._initialized:
                    time.sleep(BACKEND_IDLE_SLEEP_S)
                    continue
                self._poll_backend_clobber()   # session 64 post-flight fix: off the frame path
                depth = self._queue_depth()
                _kf_n = len(self.keyframes)         # backend thread: it appends/pops during RELOC
                with self._backend_meta_lock:
                    self._backend_queue_depth = depth   # publish for the frame path to read cheaply
                    self._kf_count = _kf_n
                mode = self.states.get_mode()
                did_work_pass = depth > 0 or mode == self._Mode.RELOC
                _t0 = time.perf_counter()
                self._run_backend()
                if did_work_pass:
                    elapsed_ms = (time.perf_counter() - _t0) * 1000.0
                    # C4: record what the backend just wrote for the newest keyframe only (one
                    # small tensor, not the whole shared buffer) so process() can tell, after the
                    # tracker's next write-back, whether the tracker clobbered this solve. Clone
                    # under keyframes.lock, THEN take _backend_meta_lock -- never the reverse order,
                    # or a tracker thread doing the same in the other order would deadlock.
                    n = len(self.keyframes)
                    last_written = {}
                    if n > 0:
                        idx = n - 1
                        with self.keyframes.lock:
                            last_written = {idx: self.keyframes.T_WC[idx].detach().clone()}
                    with self._backend_meta_lock:
                        self._backend_thread_ms = elapsed_ms
                        self._backend_solves += 1
                        self._backend_last_written = last_written
                else:
                    time.sleep(BACKEND_IDLE_SLEEP_S)
            except BaseException as exc:
                # NO SILENT FALLBACKS (CLAUDE.md): a dead backend thread stays dead and says so
                # loudly -- it must never quietly revert to synchronous or quietly stop optimising.
                self.backend_failed = f"{type(exc).__name__}: {exc}"
                print(f"*** CRITICAL: SLAM backend thread died ({self.backend_failed}) -> global "
                      f"optimization is STOPPED; tracking continues DEGRADED. Set "
                      f"perception.backend_async=false to run it inline. ***")
                break

    # ------------------------------------------------------------- per frame
    def process(self, rgb_float01) -> SlamResult:
        """Drive one SLAM step on an HxWx3 float32 RGB frame in [0,1]."""
        Mode = self._Mode
        if not self._initialized:
            self._lazy_state(rgb_float01)

        _t0 = time.perf_counter()
        i = self._i
        mode = states_mode = self.states.get_mode()
        T_WC = (lietorch.Sim3.Identity(1, device=self.device)
                if i == 0 else self.states.get_frame().T_WC)
        _t_frame0 = time.perf_counter()
        frame = self._create_frame(i, rgb_float01, T_WC, img_size=512, device=self.device)
        _t_frame1 = time.perf_counter()
        frame_ms = (_t_frame1 - _t_frame0) * 1000.0

        new_kf = False
        reloc_event = False
        ran_init = False
        infer_ms = 0.0
        tracker_ms = 0.0
        if mode == Mode.INIT:
            _t_infer0 = time.perf_counter()
            X, C = self._mast3r_inference_mono(self.model, frame)
            infer_ms = (time.perf_counter() - _t_infer0) * 1000.0
            frame.update_pointmap(X, C)
            self.keyframes.append(frame)
            self.states.queue_global_optimization(len(self.keyframes) - 1)
            self.states.set_mode(Mode.TRACKING)
            self.states.set_frame(frame)
            new_kf = True
            ran_init = True
        elif mode == Mode.TRACKING:
            _t_tracker0 = time.perf_counter()
            add_new_kf, _, try_reloc = self.tracker.track(frame)
            tracker_ms = (time.perf_counter() - _t_tracker0) * 1000.0
            # C4 clobber accounting deliberately does NOT happen here any more -- see
            # _poll_backend_clobber(). Taking keyframes.lock on the frame path cost 1.7% of flight
            # 20260906_000915 while the backend held it, and the backend thread can observe the
            # very same overwrites for free.
            if try_reloc:
                self.states.set_mode(Mode.RELOC)
                reloc_event = True
                self.n_reloc += 1
            self.states.set_frame(frame)
            if add_new_kf:
                self.keyframes.append(frame)
                self.states.queue_global_optimization(len(self.keyframes) - 1)
                new_kf = True
        elif mode == Mode.RELOC:
            _t_infer0 = time.perf_counter()
            X, C = self._mast3r_inference_mono(self.model, frame)
            infer_ms = (time.perf_counter() - _t_infer0) * 1000.0
            frame.update_pointmap(X, C)
            self.states.set_frame(frame)
            self.states.queue_reloc()
        else:
            raise RuntimeError(f"invalid SLAM mode {mode}")

        # Backend runs for TRACKING/RELOC frames (not on the INIT-creating frame),
        # matching slam_offline.py's proven ordering.
        _t_track = time.perf_counter()
        if not ran_init and not self.backend_async:
            self._run_backend()
            # SYNC: nothing else publishes the depth, and there is no contending thread here, so
            # reading it directly is free.
            with self._backend_meta_lock:
                self._backend_queue_depth = self._queue_depth()
        _t_backend = time.perf_counter()

        # Recover the full pose using Act3 on origin + unit axes (act is safe; matrix()/Act4
        # is not — see _pose_basis). w[0]=center=t; w[i]-w[0]=sR·e_i = i-th column of sR.
        cur_frame = self.states.get_frame()
        w = cur_frame.T_WC.act(self._pose_basis).detach().cpu().numpy().reshape(4, 3)
        pose_mat = np.eye(4, dtype=np.float32)
        pose_mat[:3, :3] = (w[1:] - w[0]).T
        pose_mat[:3, 3] = w[0]
        center = w[0].astype(np.float32).copy()
        _t_pose = time.perf_counter()

        kf_points = kf_colors = None
        if new_kf:
            self.n_keyframes += 1
            _kf_n_now = len(self.keyframes)     # already taking the lock on this path anyway
            with self._backend_meta_lock:
                self._kf_count = _kf_n_now
            kf = self.keyframes[_kf_n_now - 1]
            X_canon = kf.X_canon.detach().cpu().numpy().reshape(self.h, self.w, 3)
            pW = (kf.T_WC.act(kf.X_canon).detach().cpu().numpy().reshape(-1, 3))
            conf = kf.get_average_conf().detach().cpu().numpy().reshape(-1)
            valid = conf > self.conf_thresh
            kf_points = pW[valid]
            kf_colors = (kf.uimg.detach().cpu().numpy() * 255).astype(np.uint8).reshape(-1, 3)[valid]
            # Refresh the camera ray field from this keyframe's canonical pointmap (unit dirs).
            norm = np.linalg.norm(X_canon, axis=2, keepdims=True)
            self.ray_field = (X_canon / np.clip(norm, 1e-9, None)).astype(np.float32)
            self.ray_hw = (self.h, self.w)

        _t_kf = time.perf_counter()
        self._i += 1
        cur_mode = Mode(self.states.get_mode()).name
        # Session 62: attribute slam_ms instead of guessing at it. A phase that did not run reports a
        # literal 0.0, never the sub-microsecond noise of an unentered branch — see CLAUDE.md's
        # no-silent-fallback rule applied to measurement: "did not run" must be distinguishable.
        track_ms = (_t_track - _t0) * 1000.0
        backend_ms = 0.0 if ran_init else (_t_backend - _t_track) * 1000.0
        pose_ms = (_t_pose - _t_backend) * 1000.0
        kf_download_ms = (_t_kf - _t_pose) * 1000.0 if new_kf else 0.0
        # Session 64 (C2): backend_ms is 0.0 in ASYNC mode (it never ran on this frame's path), so
        # the backend's state has to be reported explicitly instead -- FAILED beats ASYNC beats SYNC
        # regardless of whether a (dead) thread object still exists.
        backend_mode = self._backend_mode()
        # Session 64 post-flight fix: read the PUBLISHED depth, never states.lock (see
        # _backend_queue_depth). In SYNC mode the inline _run_backend() below keeps it fresh.
        with self._backend_meta_lock:
            backend_queue_depth = self._backend_queue_depth
            kf_count = self._kf_count
        backend_error = _flatten_backend_error(self.backend_failed)   # session 64: "" when healthy
        with self._backend_meta_lock:
            backend_thread_ms = self._backend_thread_ms
            backend_pose_clobbers = self._backend_pose_clobbers
        return SlamResult(
            tracking_mode=self.tracking_mode, mode=cur_mode,
            n_keyframes=kf_count, frame_idx=i, camera_center=center,
            new_keyframe=new_kf, reloc_event=reloc_event, pose=pose_mat,
            kf_points=kf_points, kf_colors=kf_colors,
            track_ms=track_ms, backend_ms=backend_ms, pose_ms=pose_ms,
            kf_download_ms=kf_download_ms,
            frame_ms=frame_ms, infer_ms=infer_ms, tracker_ms=tracker_ms,
            backend_mode=backend_mode, backend_queue_depth=backend_queue_depth,
            backend_error=backend_error,
            backend_thread_ms=backend_thread_ms, backend_pose_clobbers=backend_pose_clobbers)
