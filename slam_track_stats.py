"""Session 67: instrument `FrameTracker.track()` without touching `third_party/`.

`STATE.md`'s top item is that `tracker_ms` is an in-flight accumulator: flat at ~250-400ms on a
deterministic replay of the exact frames a flight consumed, but 2000-6000ms live on the flight those
frames came from, stepping up 7-17x within two minutes of every takeoff and never coming back down.
`track()` (`third_party/MASt3R-SLAM/mast3r_slam/tracker.py:28`) is not one measurement -- it is a fixed
ViT match forward plus pointmap update (lines 31-65), an early-exit gate (67-70), a data-dependent
Gauss-Newton solve that is the ONLY part whose cost can plausibly grow with flight time (`:75`, looping
into `opt_pose_ray_dist_sim3` at `:173-214`), and a fixed keyframe write-back (95-114). Today none of
that is visible: only a single `tracker_ms` reaches the CSV. This module splits it into `trk_pre_ms`
(fixed work) vs `trk_solve_ms` (the GN loop) and records the loop's exit reason and correspondence
count, so a flight's numbers can say whether iterations climb (the warm-start `idx_f2k` loop is the
accumulator) or iterations stay flat while `trk_pre_ms` alone rises (the accumulator is process-level
state a replay never touches, not the solver).

Because `third_party/` stays pristine (CLAUDE.md; also `slam_engine.py:234-236`'s comment on the same
rule), `track()` is WRAPPED, not copied: `make_instrumented_frame_tracker` below builds a subclass, one
level of inheritance over whatever `FrameTracker` class it is handed, exactly the way
`slam_window.make_windowed_factor_graph` builds a subclass over `FactorGraph` for the same reason. The
subclass's `track()` calls `super().track()` unchanged and only measures around it; only
`opt_pose_ray_dist_sim3` -- the one method that needs mid-loop instrumentation a wrapper cannot get from
the outside -- is overridden, and its body is the vendored loop verbatim plus counters, never rewritten.

Module-level imports are stdlib + `torch` only, so this file imports on a box with no CUDA, no model
checkpoint and no vendored MASt3R-SLAM checkout at all (`torch` itself is optional too -- see the
import guard below -- so the file at least IMPORTS even on a bare interpreter with nothing installed).
Every `mast3r_slam.*` name is imported lazily, inside `make_instrumented_frame_tracker`, so calling the
factory is the only operation that actually requires the vendored repo to be on `sys.path`.
"""

import argparse
import dataclasses
import sys
import time
from dataclasses import dataclass

try:
    import torch
except ImportError:  # pragma: no cover -- exercised only on a bare interpreter with no torch
    torch = None


@dataclass(frozen=True)
class TrackStats:
    """One `FrameTracker.track()` call, measured. Frozen and REPLACED by whole-object attribute
    rebind, never mutated -- the same lock-free-read argument slam_engine.py:623-627 makes for
    FactorGraph.last_window_stats."""

    trk_pre_ms: float = 0.0      # track() entry -> GN-loop entry: MASt3R match forward + iter_proj/
                                  # refine_matches + pointmap update + validity masks. FIXED work.
    trk_solve_ms: float = 0.0    # the GN loop itself. The ONLY data-dependent work in track().
    trk_gn_iters: int = 0        # GN steps actually executed, 1..max_iters
    trk_gn_exit: str = ""        # one of GN_EXITS; "" means track() did not run this frame
    trk_valid_opt: int = 0       # correspondences entering the solve (valid_opt.sum())
    trk_match_frac: float = 0.0  # valid_opt.sum() / valid_opt.numel() -- the min_match_frac number
    trk_seeded: int = 0          # 1 if idx_f2k was warm-started, 0 if reset by the last keyframe


TRACK_STATS_ABSENT: TrackStats = TrackStats()

GN_EXITS: tuple = (
    "",                 # track() did not run this frame (INIT / RELOC)
    "rel_error",        # check_convergence fired on the relative cost decrease
    "delta_norm",       # check_convergence fired on the step norm
    "max_iters",        # ran out the max_iters ceiling without converging
    "skipped",          # min_match_frac gate fired (tracker.py:68) -- no solve ran
    "cholesky",         # the solve raised (tracker.py:91's branch); we re-raise
    "error",            # track() raised outside the solve -- must never be silently a "skipped"
    "unclassified",     # check_convergence said True but the mirror could not attribute it: a BUG,
)                       # made loud rather than guessed at

TRACKER_PHASE_FIELDS: tuple = ("trk_pre_ms", "trk_solve_ms")
TRACKER_STATE_FIELDS: tuple = ("trk_gn_iters", "trk_gn_exit", "trk_valid_opt", "trk_match_frac",
                                "trk_seeded")


def classify_convergence(step, old_cost, new_cost, delta_norm, rel_thresh, delta_thresh) -> str:
    """Attribute a check_convergence() == True to ONE of its two predicates.

    Exact mirror of nonlinear_optimizer.py:14-19, which returns only a bool:
        rel_dec = fabs((old_cost - new_cost) / old_cost)
        converged = rel_dec < rel_error_threshold or delta_norm < delta_norm_threshold
    `or` short-circuits, so rel_error wins a tie -- reproduced here.
    On step 0 old_cost is inf, so rel_dec is nan and `nan < x` is False: step 0 can only ever
    converge via delta_norm. Division by a zero old_cost is NOT guarded -- upstream does not guard
    it either, and this only ever runs after check_convergence already survived the same division.

    Returns "rel_error" | "delta_norm" | "" (caller maps "" to GN_EXITS' "unclassified").
    """
    import math

    cost_diff = old_cost - new_cost
    rel_dec = math.fabs(cost_diff / old_cost)

    if rel_dec < rel_thresh:
        return "rel_error"
    if delta_norm < delta_thresh:
        return "delta_norm"
    return ""


def _cuda_sync() -> None:
    """Wall-clock timing is meaningless across async CUDA launches, so each of the four timing
    boundaries syncs first. Adds NO GPU work and cannot change behaviour, but it makes tracker_ms
    marginally larger than on a pre-session-67 flight -- stated, never hidden (CLAUDE.md). The
    availability guard exists ONLY so the CPU-only self-test can construct this class."""
    if torch.cuda.is_available():
        torch.cuda.synchronize()


_INSTRUMENTED_CLS: dict = {}


def make_instrumented_frame_tracker(base_cls: type) -> type:
    """Build the FrameTracker subclass that publishes a TrackStats per track() call.

    Factory-over-injected-base, cached per base class, exactly like
    slam_window.make_windowed_factor_graph -- so third_party/ stays pristine and this module
    imports and self-tests with no CUDA, no model and no vendored repo on the path.
    """
    cached = _INSTRUMENTED_CLS.get(base_cls)
    if cached is not None:
        return cached

    from mast3r_slam.geometry import act_Sim3, point_to_ray_dist
    from mast3r_slam.nonlinear_optimizer import check_convergence

    class InstrumentedFrameTracker(base_cls):
        def __init__(self, model, frames, device):
            super().__init__(model, frames, device)
            self.last_track_stats = TRACK_STATS_ABSENT
            self._trk_solve_entered = False
            self._trk_t_track0 = 0.0
            self._trk_t_solve0 = 0.0
            self._trk_solve_ms = 0.0
            self._trk_gn_iters = 0
            self._trk_gn_exit = ""
            self._trk_valid_opt = 0
            self._trk_match_frac = 0.0

        def track(self, frame):
            self._trk_solve_entered = False
            self._trk_t_track0 = 0.0
            self._trk_t_solve0 = 0.0
            self._trk_solve_ms = 0.0
            self._trk_gn_iters = 0
            self._trk_gn_exit = ""
            self._trk_valid_opt = 0
            self._trk_match_frac = 0.0
            seeded = 1 if self.idx_f2k is not None else 0

            _cuda_sync()
            self._trk_t_track0 = time.perf_counter()
            try:
                result = super().track(frame)
            except Exception:
                _cuda_sync()
                t_end = time.perf_counter()
                if not self._trk_solve_entered:
                    self._trk_gn_exit = "error"
                    pre_ms = (t_end - self._trk_t_track0) * 1000.0
                else:
                    pre_ms = (self._trk_t_solve0 - self._trk_t_track0) * 1000.0
                self.last_track_stats = TrackStats(
                    trk_pre_ms=pre_ms,
                    trk_solve_ms=self._trk_solve_ms,
                    trk_gn_iters=self._trk_gn_iters,
                    trk_gn_exit=self._trk_gn_exit,
                    trk_valid_opt=self._trk_valid_opt,
                    trk_match_frac=self._trk_match_frac,
                    trk_seeded=seeded,
                )
                raise

            _cuda_sync()
            t_end = time.perf_counter()
            if self._trk_solve_entered:
                pre_ms = (self._trk_t_solve0 - self._trk_t_track0) * 1000.0
                exit_reason = self._trk_gn_exit
            else:
                pre_ms = (t_end - self._trk_t_track0) * 1000.0
                exit_reason = "skipped"

            self.last_track_stats = TrackStats(
                trk_pre_ms=pre_ms,
                trk_solve_ms=self._trk_solve_ms,
                trk_gn_iters=self._trk_gn_iters,
                trk_gn_exit=exit_reason,
                trk_valid_opt=self._trk_valid_opt,
                trk_match_frac=self._trk_match_frac,
                trk_seeded=seeded,
            )
            return result

        def opt_pose_ray_dist_sim3(self, Xf, Xk, T_WCf, T_WCk, Qk, valid):
            self._trk_solve_entered = True
            # tracker.py:67's number, read once at solver entry -- exactly valid_opt.
            self._trk_valid_opt = int(valid.sum())
            self._trk_match_frac = float(valid.sum()) / float(valid.numel())
            _cuda_sync()
            self._trk_t_solve0 = time.perf_counter()

            n_iters = 0
            exit_reason = "max_iters"
            try:
                # -- verbatim tracker.py:173-214 below, plus the n_iters/exit_reason bookkeeping --
                last_error = 0
                sqrt_info_ray = 1 / self.cfg["sigma_ray"] * valid * torch.sqrt(Qk)
                sqrt_info_dist = 1 / self.cfg["sigma_dist"] * valid * torch.sqrt(Qk)
                sqrt_info = torch.cat((sqrt_info_ray.repeat(1, 3), sqrt_info_dist), dim=1)

                # Solving for relative pose without scale!
                T_CkCf = T_WCk.inv() * T_WCf

                # Precalculate distance and ray for obs k
                rd_k = point_to_ray_dist(Xk, jacobian=False)

                old_cost = float("inf")
                for step in range(self.cfg["max_iters"]):
                    Xf_Ck, dXf_Ck_dT_CkCf = act_Sim3(T_CkCf, Xf, jacobian=True)
                    rd_f_Ck, drd_f_Ck_dXf_Ck = point_to_ray_dist(Xf_Ck, jacobian=True)
                    # r = z-h(x)
                    r = rd_k - rd_f_Ck
                    # Jacobian
                    J = -drd_f_Ck_dXf_Ck @ dXf_Ck_dT_CkCf

                    tau_ij_sim3, new_cost = self.solve(sqrt_info, r, J)
                    T_CkCf = T_CkCf.retr(tau_ij_sim3)
                    n_iters = step + 1

                    if check_convergence(
                        step,
                        self.cfg["rel_error"],
                        self.cfg["delta_norm"],
                        old_cost,
                        new_cost,
                        tau_ij_sim3,
                    ):
                        delta_norm_val = float(torch.linalg.norm(tau_ij_sim3).item())
                        reason = classify_convergence(
                            step, old_cost, new_cost, delta_norm_val,
                            self.cfg["rel_error"], self.cfg["delta_norm"])
                        exit_reason = reason if reason else "unclassified"
                        break
                    old_cost = new_cost

                    if step == self.cfg["max_iters"] - 1:
                        print(f"max iters reached {last_error}")
                # -- end verbatim block --
            except Exception:
                self._trk_gn_iters = n_iters
                self._trk_gn_exit = "cholesky"
                _cuda_sync()
                t_solve_end = time.perf_counter()
                self._trk_solve_ms = (t_solve_end - self._trk_t_solve0) * 1000.0
                raise

            _cuda_sync()
            t_solve_end = time.perf_counter()
            self._trk_solve_ms = (t_solve_end - self._trk_t_solve0) * 1000.0
            self._trk_gn_iters = n_iters
            self._trk_gn_exit = exit_reason

            # Assign new pose based on relative pose
            T_WCf = T_WCk * T_CkCf

            return T_WCf, T_CkCf

        def opt_pose_calib_sim3(self, *args, **kwargs):
            raise NotImplementedError(
                "opt_pose_calib_sim3 has no instrumentation -- use_calib is forced False "
                "(slam_engine.py:238), so this method should never be reached while tracking is "
                "instrumented. A quiet uninstrumented second path is exactly the hidden state this "
                "session hunts, so this fails loud rather than silently tracking without stats.")

    InstrumentedFrameTracker.__name__ = "InstrumentedFrameTracker"
    InstrumentedFrameTracker.__qualname__ = "InstrumentedFrameTracker"
    _INSTRUMENTED_CLS[base_cls] = InstrumentedFrameTracker
    return InstrumentedFrameTracker


# ---------------------------------------------------------------------------------------------
# Self-test
# ---------------------------------------------------------------------------------------------


def _run_pure_self_tests(check) -> None:
    field_names = tuple(f.name for f in dataclasses.fields(TrackStats))
    check("TrackStats field order == TRACKER_PHASE_FIELDS + TRACKER_STATE_FIELDS",
          field_names == TRACKER_PHASE_FIELDS + TRACKER_STATE_FIELDS)
    check("TrackStats has no duplicate fields", len(set(field_names)) == len(field_names))
    check("TRACK_STATS_ABSENT.trk_gn_exit == ''", TRACK_STATS_ABSENT.trk_gn_exit == "")

    expected_exits = {"", "rel_error", "delta_norm", "max_iters", "skipped", "cholesky", "error",
                       "unclassified"}
    check("GN_EXITS covers exactly the values the factory can produce", set(GN_EXITS) == expected_exits)
    check("GN_EXITS has no duplicates", len(set(GN_EXITS)) == len(GN_EXITS))

    def local_check_convergence(old_cost, new_cost, delta_norm, rel_thresh, delta_thresh):
        import math
        rel_dec = math.fabs((old_cost - new_cost) / old_cost)
        return rel_dec < rel_thresh or delta_norm < delta_thresh

    grid = [
        # (old_cost, new_cost, delta_norm, rel_thresh, delta_thresh)
        (float("inf"), 10.0, 1.0, 1e-3, 1e-3),      # step 0: rel_dec is nan -- never rel_error
        (float("inf"), 10.0, 1e-6, 1e-3, 1e-3),     # step 0, delta small enough -> delta_norm
        (100.0, 99.999, 1e-6, 1e-3, 1e-3),          # both predicates true -> rel_error wins the tie
        (100.0, 50.0, 1e-6, 1e-3, 1e-3),            # only delta_norm true
        (100.0, 50.0, 1.0, 1e-3, 1e-3),             # neither true -> ""
    ]
    for old_cost, new_cost, delta_norm, rel_thresh, delta_thresh in grid:
        expect_converged = local_check_convergence(old_cost, new_cost, delta_norm, rel_thresh, delta_thresh)
        reason = classify_convergence(0, old_cost, new_cost, delta_norm, rel_thresh, delta_thresh)
        check(f"classify_convergence agrees with local bool for "
              f"(old={old_cost}, new={new_cost}, dn={delta_norm})",
              (reason != "") == expect_converged)

    check("step 0 with old_cost=inf never returns 'rel_error'",
          classify_convergence(0, float("inf"), 10.0, 1.0, 1e-3, 1e-3) != "rel_error")
    check("step 0 with old_cost=inf and tiny delta_norm returns 'delta_norm'",
          classify_convergence(0, float("inf"), 10.0, 1e-6, 1e-3, 1e-3) == "delta_norm")
    check("both predicates true -> 'rel_error' (tie-break matches Python 'or')",
          classify_convergence(0, 100.0, 99.999, 1e-6, 1e-3, 1e-3) == "rel_error")
    check("only delta_norm true -> 'delta_norm'",
          classify_convergence(0, 100.0, 50.0, 1e-6, 1e-3, 1e-3) == "delta_norm")
    check("neither true -> ''",
          classify_convergence(0, 100.0, 50.0, 1.0, 1e-3, 1e-3) == "")


class _StubBaseTracker:
    """Stand-in for `mast3r_slam.tracker.FrameTracker`: same constructor shape and the same
    `self.solve(...)` Cholesky body (tracker.py:156-171, copied verbatim in spirit -- see
    slam_window.py's `_FakeBaseFactorGraph` for the same pattern), but `track()` is a minimal
    dispatcher over a `mode` the self-test sets, so exercising "skip" / "raise before the solve"
    never needs a real ViT model or match forward."""

    def __init__(self, model, frames, device):
        self.model = model
        self.frames = frames
        self.device = device
        self.idx_f2k = None
        self.cfg = {
            "sigma_ray": 0.003, "sigma_dist": 1e1, "huber": 1.345,
            "max_iters": 50, "rel_error": 1e-3, "delta_norm": 1e-3,
        }
        self.mode = "normal"
        self.fake_data = None

    def reset_idx_f2k(self):
        self.idx_f2k = None

    def track(self, frame):
        if self.mode == "skip":
            return False, [], True
        if self.mode == "raise_before_solve":
            raise RuntimeError("stub: failure before the solve is ever entered")
        d = self.fake_data
        T_WCf, T_CkCf = self.opt_pose_ray_dist_sim3(
            d["Xf"], d["Xk"], d["T_WCf"], d["T_WCk"], d["Qk"], d["valid"])
        return True, [], False

    def solve(self, sqrt_info, r, J):
        from mast3r_slam.nonlinear_optimizer import huber
        whitened_r = sqrt_info * r
        robust_sqrt_info = sqrt_info * torch.sqrt(huber(whitened_r, k=self.cfg["huber"]))
        mdim = J.shape[-1]
        A = (robust_sqrt_info[..., None] * J).view(-1, mdim)
        b = (robust_sqrt_info * r).view(-1, 1)
        H = A.T @ A
        g = -A.T @ b
        cost = 0.5 * (b.T @ b).item()

        L = torch.linalg.cholesky(H, upper=False)
        tau_j = torch.cholesky_solve(g, L, upper=False).view(1, -1)
        return tau_j, cost


def _make_fake_track_data(perfect_match: bool, n=20):
    import lietorch
    T_WCf = lietorch.Sim3.Identity(1, device="cpu")
    T_WCk = lietorch.Sim3.Identity(1, device="cpu")
    Xk = torch.rand(n, 3) + torch.tensor([0.0, 0.0, 2.0])  # keep z > 0 for a valid ray/dist
    Xf = Xk.clone() if perfect_match else Xk + 0.05 * torch.randn(n, 3)
    Qk = torch.ones(n, 1)
    valid = torch.ones(n, 1, dtype=torch.bool)
    return {"Xf": Xf, "Xk": Xk, "T_WCf": T_WCf, "T_WCk": T_WCk, "Qk": Qk, "valid": valid}


def _run_factory_self_tests(check) -> None:
    try:
        import lietorch  # noqa: F401
        from mast3r_slam.geometry import act_Sim3, point_to_ray_dist  # noqa: F401
        from mast3r_slam.nonlinear_optimizer import check_convergence, huber  # noqa: F401
    except ImportError as e:
        print(f"[self-test] SKIP  factory/vendored-repo tests -- {e!r} not importable")
        return

    InstrumentedFrameTracker = make_instrumented_frame_tracker(_StubBaseTracker)
    check("make_instrumented_frame_tracker caches: same class object twice",
          make_instrumented_frame_tracker(_StubBaseTracker) is InstrumentedFrameTracker)

    # -- normal solve: exact-match data converges at step 0 via delta_norm (old_cost=inf there) --
    t = InstrumentedFrameTracker(None, None, "cpu")
    t.fake_data = _make_fake_track_data(perfect_match=True)
    t0 = time.perf_counter()
    new_kf, _, lost = t.track(object())
    wall_ms = (time.perf_counter() - t0) * 1000.0
    check("normal: track() returns without raising", lost is False)
    check("normal: trk_gn_iters >= 1", t.last_track_stats.trk_gn_iters >= 1)
    check("normal: trk_gn_exit in {rel_error, delta_norm, max_iters}",
          t.last_track_stats.trk_gn_exit in {"rel_error", "delta_norm", "max_iters"})
    check("normal: exact-match data converges via delta_norm at step 0",
          t.last_track_stats.trk_gn_exit == "delta_norm" and t.last_track_stats.trk_gn_iters == 1)
    check("normal: trk_valid_opt == 20, trk_match_frac == 1.0",
          t.last_track_stats.trk_valid_opt == 20 and t.last_track_stats.trk_match_frac == 1.0)
    check("normal: trk_seeded == 0 (idx_f2k was never set)", t.last_track_stats.trk_seeded == 0)
    check("timing invariant: trk_pre_ms + trk_solve_ms <= wall clock of track()",
          t.last_track_stats.trk_pre_ms + t.last_track_stats.trk_solve_ms <= wall_ms + 1e-6)

    # -- trk_seeded == 1 when idx_f2k was warm-started --
    t_seed = InstrumentedFrameTracker(None, None, "cpu")
    t_seed.idx_f2k = torch.zeros(1, 4, dtype=torch.int64)
    t_seed.fake_data = _make_fake_track_data(perfect_match=True)
    t_seed.track(object())
    check("trk_seeded == 1 when idx_f2k is not None at entry",
          t_seed.last_track_stats.trk_seeded == 1)

    # -- max_iters: impossible thresholds force every iteration to run out the ceiling --
    t_max = InstrumentedFrameTracker(None, None, "cpu")
    t_max.cfg["max_iters"] = 3
    t_max.cfg["rel_error"] = 0.0
    t_max.cfg["delta_norm"] = 0.0
    t_max.fake_data = _make_fake_track_data(perfect_match=False)
    t_max.track(object())
    check("max_iters: exit == 'max_iters', iters == max_iters",
          t_max.last_track_stats.trk_gn_exit == "max_iters" and t_max.last_track_stats.trk_gn_iters == 3)

    # -- skipped: mode short-circuits before the solve is ever entered --
    t_skip = InstrumentedFrameTracker(None, None, "cpu")
    t_skip.mode = "skip"
    new_kf, _, lost = t_skip.track(object())
    check("skipped: track() returns normally (lost=True, no exception)", lost is True)
    check("skipped: trk_gn_exit == 'skipped'", t_skip.last_track_stats.trk_gn_exit == "skipped")
    check("skipped: trk_pre_ms > 0", t_skip.last_track_stats.trk_pre_ms > 0)
    check("skipped: trk_solve_ms == 0.0", t_skip.last_track_stats.trk_solve_ms == 0.0)

    # -- cholesky: an all-invalid mask makes H singular, so self.solve()'s cholesky raises --
    t_chol = InstrumentedFrameTracker(None, None, "cpu")
    d = _make_fake_track_data(perfect_match=True)
    d["valid"] = torch.zeros_like(d["valid"])
    t_chol.fake_data = d
    raised = False
    try:
        t_chol.track(object())
    except Exception:
        raised = True
    check("cholesky: exception propagated (re-raised, not swallowed)", raised)
    check("cholesky: trk_gn_exit == 'cholesky'", t_chol.last_track_stats.trk_gn_exit == "cholesky")

    # -- error: track() raises before the solve is ever entered --
    t_err = InstrumentedFrameTracker(None, None, "cpu")
    t_err.mode = "raise_before_solve"
    raised = False
    try:
        t_err.track(object())
    except RuntimeError:
        raised = True
    check("error: exception propagated (re-raised, not swallowed)", raised)
    check("error: trk_gn_exit == 'error'", t_err.last_track_stats.trk_gn_exit == "error")

    # -- opt_pose_calib_sim3 is deliberately uninstrumented --
    t_calib = InstrumentedFrameTracker(None, None, "cpu")
    try:
        t_calib.opt_pose_calib_sim3()
        check("opt_pose_calib_sim3 raises NotImplementedError", False)
    except NotImplementedError:
        check("opt_pose_calib_sim3 raises NotImplementedError", True)


def run_self_test() -> None:
    failures = []

    def check(label, cond):
        status = "PASS" if cond else "FAIL"
        print(f"[self-test] {status}  {label}")
        if not cond:
            failures.append(label)

    _run_pure_self_tests(check)

    if torch is None:
        print("[self-test] SKIP  torch is not installed -- skipping all torch/vendored-repo tests")
    else:
        _run_factory_self_tests(check)

    print()
    if failures:
        print(f"[self-test] FAILURES PRESENT ({len(failures)}):")
        for f in failures:
            print(f"  - {f}")
        sys.exit(1)
    print("[self-test] ALL PASS")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        run_self_test()
    else:
        ap.error("nothing to do: pass --self-test")
