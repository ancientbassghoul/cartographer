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
    kf_points: np.ndarray | None = None   # (N,3) world points of the NEW keyframe only
    kf_colors: np.ndarray | None = None   # (N,3) uint8, paired with kf_points


class SlamEngine:
    def __init__(self, device="cuda:0", config_path="config/base.yaml", conf_thresh=1.5):
        assert torch.cuda.is_available(), "CUDA required for SLAM (NO SILENT FALLBACKS)."
        self.device = device
        self.conf_thresh = conf_thresh
        self.tracking_mode = "MASt3R"

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

    # ------------------------------------------------------------- per frame
    def process(self, rgb_float01) -> SlamResult:
        """Drive one SLAM step on an HxWx3 float32 RGB frame in [0,1]."""
        Mode = self._Mode
        if not self._initialized:
            self._lazy_state(rgb_float01)

        i = self._i
        mode = states_mode = self.states.get_mode()
        T_WC = (lietorch.Sim3.Identity(1, device=self.device)
                if i == 0 else self.states.get_frame().T_WC)
        frame = self._create_frame(i, rgb_float01, T_WC, img_size=512, device=self.device)

        new_kf = False
        reloc_event = False
        ran_init = False
        if mode == Mode.INIT:
            X, C = self._mast3r_inference_mono(self.model, frame)
            frame.update_pointmap(X, C)
            self.keyframes.append(frame)
            self.states.queue_global_optimization(len(self.keyframes) - 1)
            self.states.set_mode(Mode.TRACKING)
            self.states.set_frame(frame)
            new_kf = True
            ran_init = True
        elif mode == Mode.TRACKING:
            add_new_kf, _, try_reloc = self.tracker.track(frame)
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
            X, C = self._mast3r_inference_mono(self.model, frame)
            frame.update_pointmap(X, C)
            self.states.set_frame(frame)
            self.states.queue_reloc()
        else:
            raise RuntimeError(f"invalid SLAM mode {mode}")

        # Backend runs for TRACKING/RELOC frames (not on the INIT-creating frame),
        # matching slam_offline.py's proven ordering.
        if not ran_init:
            self._run_backend()

        center = (self.states.get_frame().T_WC.act(self._origin)
                  .detach().cpu().numpy().reshape(3).astype(np.float32))

        kf_points = kf_colors = None
        if new_kf:
            self.n_keyframes += 1
            kf = self.keyframes[len(self.keyframes) - 1]
            pW = kf.T_WC.act(kf.X_canon).detach().cpu().numpy().reshape(-1, 3)
            conf = kf.get_average_conf().detach().cpu().numpy().reshape(-1)
            valid = conf > self.conf_thresh
            kf_points = pW[valid]
            kf_colors = (kf.uimg.detach().cpu().numpy() * 255).astype(np.uint8).reshape(-1, 3)[valid]

        self._i += 1
        cur_mode = Mode(self.states.get_mode()).name
        return SlamResult(
            tracking_mode=self.tracking_mode, mode=cur_mode,
            n_keyframes=len(self.keyframes), frame_idx=i, camera_center=center,
            new_keyframe=new_kf, reloc_event=reloc_event,
            kf_points=kf_points, kf_colors=kf_colors)
