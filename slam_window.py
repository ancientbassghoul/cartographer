"""The bounded global-optimisation window (session 65).

`FactorGraph` (`third_party/MASt3R-SLAM/mast3r_slam/global_opt.py`) is **append-only** — `add_factors`
only ever `torch.cat`s onto its eight per-edge tensors, and `solve_GN_rays` re-optimises
`get_unique_kf_idx()`, i.e. every keyframe that has ever appeared in any edge, on every single solve.
Nothing is ever pruned, so `backend_ms` grows with total keyframe count instead of flattening — see
`plans/slam_report.html`. This module is the pure decision core that bounds that growth: given the
edge lists `ii`/`jj` of the graph, decide which edges a solve should keep.

The window is a suffix of keyframe indices, `[window_lo, n_new]`, where `n_new` is the newest keyframe
touched by any edge. An edge survives if at least one endpoint falls in that suffix (a stricter
`policy="strict"` requires both). A surviving edge whose far endpoint falls *below* `window_lo` is a
loop closure reaching into history; that far endpoint is kept as an **anchor** so the closure can still
pull the window into agreement with the map's older geometry.

This is safe to do purely by dropping edges, with no index renumbering, for two reasons found by
reading `gn_kernels.cu`:

- `gauss_newton_rays_cuda` already remaps whatever keyframe set it is given to dense row positions via
  `create_inds` (`gn_kernels.cu:166-170`), which is exactly `torch.searchsorted(unique_kf_idx, ii/jj)`.
  A **non-contiguous** keyframe set — a window plus a couple of anchors, not `arange(N)` — is handled
  correctly as long as the Python side gathers poses/points in the same sorted-unique order it hands
  to the kernel, which `get_poses_points(unique_kf_idx)` already does. `local_edge_indices` below is a
  Python mirror of that mapping, kept only so the self-test can prove agreement with the kernel's
  contract.
- The kernel pins `num_fix = 1` poses — the **lowest-indexed** keyframe in the solve
  (`gn_kernels.cu:1157`). Because a window is a suffix and anchors are, by construction, indices below
  `window_lo`, the lowest sorted index in any windowed solve is always an anchor (or the window floor
  when there are none). The oldest anchor is pinned automatically, so a windowed solve is anchored to
  the existing global estimate rather than floating free — no extra bookkeeping required here.

No CUDA, no model, no keyframe store, and no import of the vendored SLAM repo: this module must import
and run standalone so tooling can exercise the selection logic without a GPU or a build.
"""

import argparse
import sys
from dataclasses import dataclass, replace

import torch

WINDOW_MODES: tuple = ("OFF", "SHADOW", "ON")
WINDOW_POLICIES: tuple = ("anchored", "free", "strict")


@dataclass(frozen=True)
class WindowStats:
    mode: str = "OFF"           # "OFF" | "SHADOW" | "ON"
    window_kf: int = 0          # configured W; 0 = unbounded
    window_lo: int = -1         # lowest in-window keyframe index; -1 when unbounded or empty
    solve_kf: int = 0           # keyframes handed to the solver (would-be, in SHADOW)
    solve_edges: int = 0        # edges handed to the solver (would-be, in SHADOW)
    graph_edges: int = 0        # total edges in the graph
    anchors: int = 0            # out-of-window keyframes dragged in by loop edges
    anchor_drift: float = 0.0   # max ||pose_after - pose_before|| over anchors; 0.0 when anchors == 0


@dataclass(frozen=True)
class EdgeSelection:
    mask: "torch.Tensor"        # (E,) bool — True = keep this edge
    window_lo: int              # -1 when unbounded
    n_anchors: int
    anchor_idx: "torch.Tensor"  # (n_anchors,) int64, ascending — the out-of-window keyframes kept
    active_kf: "torch.Tensor"   # (K,) int64, ascending — unique keyframes of the masked edges
    # Session 66: host-side scalars, computed while the selection is already on the CPU. They exist
    # so a caller never has to ask a CUDA tensor a question (`mask.all()`, `mask.sum()`) just to
    # decide what to do next -- every such question is a device->host sync that drains the queue.
    n_active_edges: int = 0     # == int(mask.sum()), without the sync
    all_active: bool = False    # == bool(mask.all()), without the sync


def validate_window_config(mode: str, window_kf: int, policy: str) -> tuple:
    """Normalise and validate a (mode, window_kf, policy) triple. Case-sensitive on purpose: a typo'd
    "on" or "ANCHORED" must raise, not silently get accepted by an .upper()/.lower() normalisation."""
    if mode not in WINDOW_MODES:
        raise ValueError(f"backend_window_mode={mode!r} is not one of {WINDOW_MODES}")
    if policy not in WINDOW_POLICIES:
        raise ValueError(f"backend_window_policy={policy!r} is not one of {WINDOW_POLICIES}")
    if not isinstance(window_kf, int) or isinstance(window_kf, bool):
        raise ValueError(f"backend_window_kf={window_kf!r} must be an int")
    if window_kf < 0:
        window_kf = 0
    return mode, window_kf, policy


def select_active_edges(ii, jj, window_kf: int, policy: str = "anchored") -> EdgeSelection:
    """Decide which edges of the factor graph a solve should keep.

    `ii`/`jj` are the graph's (E,) int64 endpoint tensors, on any device; every tensor this returns
    lives on `ii.device`. `window_kf <= 0` means unbounded — the whole graph passes through unchanged,
    which is the disabled path and must be provably an identity operation (see self-test).
    """
    if policy not in WINDOW_POLICIES:
        raise ValueError(f"policy={policy!r} is not one of {WINDOW_POLICIES}")

    device = ii.device
    e = ii.numel()

    if e == 0:
        empty_bool = torch.zeros(0, dtype=torch.bool, device=device)
        empty_idx = torch.zeros(0, dtype=torch.int64, device=device)
        return EdgeSelection(mask=empty_bool, window_lo=-1, n_anchors=0,
                              anchor_idx=empty_idx, active_kf=empty_idx)

    if window_kf <= 0:
        mask = torch.ones(e, dtype=torch.bool, device=device)
        active_kf = torch.unique(torch.cat([ii, jj]), sorted=True)
        empty_idx = torch.zeros(0, dtype=torch.int64, device=device)
        return EdgeSelection(mask=mask, window_lo=-1, n_anchors=0,
                              anchor_idx=empty_idx, active_kf=active_kf,
                              n_active_edges=e, all_active=True)

    # Session 66 -- ONE host transfer, then every decision below is host-side arithmetic.
    # Measured (A/B on flight 20260906_165141's exact frame list): computing this selection with
    # CUDA tensors cost ~1.8 s per solve EVEN AT keyframe counts below the window, where the mask is
    # all-True and the cut is a provable no-op -- because `ii.max()`, the boolean indexing
    # `active_kf[active_kf < window_lo]` (data-dependent output shape), `.numel()` on its result and
    # the caller's `mask.all()` are four separate device->host syncs, each draining the whole CUDA
    # queue before the solve could start. That overhead made the window a NET LOSS (+10.3% wall
    # clock) despite genuinely flattening the growth curve. The edge lists are tiny -- a few hundred
    # int64, ~1.3 KB at this flight's 167-edge maximum -- so pulling them to the host once is far
    # cheaper than asking the device four questions.
    ii_c = ii.cpu()
    jj_c = jj.cpu()

    n_new = int(torch.maximum(ii_c.max(), jj_c.max()))
    window_lo = n_new - window_kf + 1
    in_i = ii_c >= window_lo
    in_j = jj_c >= window_lo
    mask_c = (in_i & in_j) if policy == "strict" else (in_i | in_j)

    active_kf_c = torch.unique(torch.cat([ii_c[mask_c], jj_c[mask_c]]), sorted=True)
    anchor_idx_c = active_kf_c[active_kf_c < window_lo]
    n_anchors = int(anchor_idx_c.numel())
    n_active_edges = int(mask_c.sum())

    return EdgeSelection(mask=mask_c.to(device), window_lo=window_lo, n_anchors=n_anchors,
                          anchor_idx=anchor_idx_c.to(device), active_kf=active_kf_c.to(device),
                          n_active_edges=n_active_edges, all_active=(n_active_edges == e))


def local_edge_indices(active_kf, ii, jj):
    """Map absolute keyframe ids in `ii`/`jj` to row positions within `active_kf`.

    This is a Python mirror of `create_inds` (`gn_kernels.cu:166-170`), which does exactly
    `torch.searchsorted(unique_kf_idx, ii/jj) - pin`. It exists only so the self-test can assert
    agreement with the contract the kernel will apply to a non-contiguous keyframe set — it is not
    used by the kernel path itself.
    """
    return torch.searchsorted(active_kf, ii), torch.searchsorted(active_kf, jj)


_WINDOWED_CLS: dict = {}


def make_windowed_factor_graph(base_cls: type, gn_rays_fn) -> type:
    """Build the `FactorGraph` subclass that applies `select_active_edges` to `solve_GN_rays`.

    This is a factory, not a plain subclass, because the thing being subclassed
    (`mast3r_slam.global_opt.FactorGraph`) and the CUDA solver it calls
    (`mast3r_slam_backends.gauss_newton_rays`) both live in packages this module must never import —
    the module docstring's "cold-importable with no GPU, no mast3r_slam" guarantee only holds if
    those names arrive as injected parameters at the one call site that actually has them in scope,
    rather than as a module-level `import`. `base_cls`/`gn_rays_fn` are closed over by the class body
    instead of stored as constructor arguments so every instance built from the same pair shares the
    same bound solver with no per-instance wiring. The class is cached by `(base_cls, gn_rays_fn)` so
    repeated calls (e.g. once per SLAM session) hand back the identical class object rather than a
    lookalike that would fail `isinstance` checks against earlier instances.
    """
    key = (base_cls, gn_rays_fn)
    cached = _WINDOWED_CLS.get(key)
    if cached is not None:
        return cached

    class WindowedFactorGraph(base_cls):
        def __init__(self, model, frames, K=None, device="cuda", *, window_mode="OFF",
                     window_kf=0, window_policy="anchored"):
            super().__init__(model, frames, K, device)
            window_mode, window_kf, window_policy = validate_window_config(
                window_mode, window_kf, window_policy)
            self.window_mode = window_mode
            self.window_kf = window_kf
            self.window_policy = window_policy
            self.last_window_stats = WindowStats(mode=self.window_mode, window_kf=self.window_kf)

        def solve_GN_rays(self, force_full: bool = False) -> None:
            graph_edges = int(self.ii.numel())

            if force_full or self.window_mode == "OFF" or self.window_kf <= 0:
                super().solve_GN_rays()
                # The one extra torch.unique here is microseconds against a multi-second solve --
                # cheap enough to always report solve_kf even on the disabled/full path.
                self.last_window_stats = WindowStats(
                    mode=self.window_mode, window_kf=self.window_kf, window_lo=-1,
                    solve_edges=graph_edges, solve_kf=int(self.get_unique_kf_idx().numel()),
                    graph_edges=graph_edges, anchors=0, anchor_drift=0.0)
                return

            sel = select_active_edges(self.ii, self.jj, self.window_kf, self.window_policy)

            if sel.all_active:
                # Graph hasn't grown past the window yet -- a cut here would be a no-op, so go
                # through upstream unchanged rather than pay for a tensor swap that changes nothing.
                super().solve_GN_rays()
                # solve_kf from the selection, NOT a second get_unique_kf_idx(): torch.unique has
                # a data-dependent output shape, so asking for its count is another sync -- and this
                # is the no-cut path, which must not cost more than the OFF path it mirrors.
                self.last_window_stats = WindowStats(
                    mode=self.window_mode, window_kf=self.window_kf, window_lo=sel.window_lo,
                    solve_edges=graph_edges, solve_kf=int(sel.active_kf.numel()),
                    graph_edges=graph_edges, anchors=0, anchor_drift=0.0)
                return

            if self.window_mode == "SHADOW":
                # Price the cut without taking it: solve the FULL graph as usual (so nothing about
                # today's trajectory changes), but record the numbers a real ON cut would have
                # produced. backend_window_mode == "SHADOW" in the CSV is what disambiguates these
                # would-be numbers from a real ON cut's actual ones.
                super().solve_GN_rays()
                self.last_window_stats = WindowStats(
                    mode="SHADOW", window_kf=self.window_kf, window_lo=sel.window_lo,
                    solve_edges=sel.n_active_edges, solve_kf=int(sel.active_kf.numel()),
                    graph_edges=graph_edges, anchors=sel.n_anchors, anchor_drift=0.0)
                return

            # ON with a real cut. Swap the eight per-edge tensors for their mask-filtered copies so
            # the inherited get_unique_kf_idx()/prep_two_way_edges()/get_poses_points() all operate
            # on the window with no duplication of upstream logic, then ALWAYS restore -- a solver
            # exception must not leave the graph permanently truncated.
            originals = (self.ii, self.jj, self.idx_ii2jj, self.idx_jj2ii,
                         self.valid_match_j, self.valid_match_i, self.Q_ii2jj, self.Q_jj2ii)
            mask = sel.mask
            try:
                self.ii = self.ii[mask]
                self.jj = self.jj[mask]
                self.idx_ii2jj = self.idx_ii2jj[mask]
                self.idx_jj2ii = self.idx_jj2ii[mask]
                self.valid_match_j = self.valid_match_j[mask]
                self.valid_match_i = self.valid_match_i[mask]
                self.Q_ii2jj = self.Q_ii2jj[mask]
                self.Q_jj2ii = self.Q_jj2ii[mask]
                stats = self._solve_GN_rays_windowed(sel)
            finally:
                (self.ii, self.jj, self.idx_ii2jj, self.idx_jj2ii,
                 self.valid_match_j, self.valid_match_i, self.Q_ii2jj, self.Q_jj2ii) = originals

            # graph_edges was captured before the swap, over the un-cut graph; _solve_GN_rays_windowed
            # only ever sees the already-cut tensors and cannot know that number, so it is filled in
            # here rather than threaded through as an extra argument.
            self.last_window_stats = replace(stats, graph_edges=graph_edges)

        def _solve_GN_rays_windowed(self, sel: EdgeSelection) -> WindowStats:
            """Mirrors upstream `solve_GN_rays` (`global_opt.py:121-158`) but runs on the swapped,
            mask-filtered tensors `solve_GN_rays` installed before calling this. Two deliberate
            changes from upstream: (1) anchor drift is measured immediately before/after the solve;
            (2) write-back skips anchors under the "anchored" policy so their stored poses are left
            untouched and the un-windowed part of the map is never corrupted by a partial solve.
            """
            cfg = self.cfg
            pin = cfg["pin"]
            solve_edges = int(self.ii.numel())
            solve_kf = int(sel.active_kf.numel())

            def make_stats(anchor_drift=0.0):
                return WindowStats(
                    mode=self.window_mode, window_kf=self.window_kf, window_lo=sel.window_lo,
                    solve_edges=solve_edges, solve_kf=solve_kf, graph_edges=0,
                    anchors=sel.n_anchors, anchor_drift=anchor_drift)

            unique_kf_idx = self.get_unique_kf_idx()
            if not torch.equal(unique_kf_idx, sel.active_kf):
                raise RuntimeError(
                    "windowed solve: get_unique_kf_idx() disagrees with sel.active_kf -- the "
                    f"edge-tensor swap did not land. get_unique_kf_idx()={unique_kf_idx.tolist()!r} "
                    f"sel.active_kf={sel.active_kf.tolist()!r}")

            if unique_kf_idx.numel() <= pin:
                return make_stats()

            Xs, T_WCs, Cs = self.get_poses_points(unique_kf_idx)
            ii, jj, idx_ii2jj, valid_match, Q_ii2jj = self.prep_two_way_edges()

            C_thresh = cfg["C_conf"]
            Q_thresh = cfg["Q_conf"]
            max_iter = cfg["max_iters"]
            sigma_ray = cfg["sigma_ray"]
            sigma_dist = cfg["sigma_dist"]
            delta_thresh = cfg["delta_norm"]

            pose_data = T_WCs.data[:, 0, :]

            n_anchors = sel.n_anchors
            # Whole pose vector, not a translation slice: a layout-agnostic "did the anchor move"
            # magnitude, not a metric distance -- this system has no metric scale to measure in anyway.
            before = pose_data[:n_anchors].clone() if n_anchors > 0 else None

            gn_rays_fn(
                pose_data, Xs, Cs, ii, jj, idx_ii2jj, valid_match, Q_ii2jj,
                sigma_ray, sigma_dist, C_thresh, Q_thresh, max_iter, delta_thresh)

            anchor_drift = 0.0
            if n_anchors > 0:
                anchor_drift = float((pose_data[:n_anchors] - before).norm(dim=1).max())

            # "anchored" keeps anchors' stored poses untouched (start clears them);
            # "free"/"strict" write back everything from pin onward, upstream-faithful.
            start = max(pin, n_anchors) if self.window_policy == "anchored" else pin
            if start >= unique_kf_idx.numel():
                raise RuntimeError(
                    f"windowed write-back: start={start} (pin={pin}, n_anchors={n_anchors}) >= "
                    f"unique_kf count={unique_kf_idx.numel()} -- the window is supposed to always "
                    "contain the newest keyframe, so this is an invariant violation, not a runtime "
                    "condition")
            self.frames.update_T_WCs(T_WCs[start:], unique_kf_idx[start:])

            return make_stats(anchor_drift=anchor_drift)

        def solve_GN_calib(self) -> None:
            if self.window_mode != "OFF":
                raise NotImplementedError(
                    "solve_GN_calib() has no windowed implementation. use_calib is forced False at "
                    "slam_engine.py:201, so this method should never be reached while a window is "
                    "active -- if it is, that assumption broke and this needs a real implementation, "
                    "not a silent fallback to the unbounded solve.")
            return super().solve_GN_calib()

    WindowedFactorGraph.__name__ = "WindowedFactorGraph"
    WindowedFactorGraph.__qualname__ = "WindowedFactorGraph"
    _WINDOWED_CLS[key] = WindowedFactorGraph
    return WindowedFactorGraph


class _FakePoses:
    """Stand-in for the `lietorch.Sim3` wrapper `get_poses_points` returns: `.data` is a (K, 1, D)
    tensor, `[idx]` slices like the real thing. No lietorch import here -- see module docstring."""

    def __init__(self, data):
        self.data = data

    def __getitem__(self, idx):
        return _FakePoses(self.data[idx])


class _FakeFrames:
    """Stand-in for `SharedKeyframes`: holds one pose vector per keyframe id and records every
    `update_T_WCs` call so the self-test can assert exactly which indices got written back."""

    def __init__(self, poses: dict):
        self.poses = poses
        self.update_calls = []

    def update_T_WCs(self, T_WCs, idx):
        self.update_calls.append((T_WCs.data.clone(), idx.clone()))
        for row, kf_idx in enumerate(idx.tolist()):
            self.poses[kf_idx] = T_WCs.data[row, 0].clone()


class _FakeBaseFactorGraph:
    """Minimal stand-in for `mast3r_slam.global_opt.FactorGraph`, with the inherited-method bodies
    upstream actually has (`get_unique_kf_idx`, `prep_two_way_edges`, `get_poses_points`,
    `solve_GN_rays`, `solve_GN_calib`) copied verbatim in spirit, so `WindowedFactorGraph`'s
    `super()` calls exercise the same code shape the real base class runs. `full_solver` is a class
    attribute, not a constructor argument, mirroring how upstream hardcodes a module-level import
    (`mast3r_slam_backends.gauss_newton_rays`) rather than taking the solver as a parameter --
    `WindowedFactorGraph.__init__` calls `super().__init__(model, frames, K, device)` with exactly
    those four positional arguments, leaving no room for an injected solver here."""

    full_solver = None

    def __init__(self, model, frames, K=None, device="cuda"):
        self.model = model
        self.frames = frames
        self.device = device
        self.cfg = {
            "pin": 1, "C_conf": 0.0, "Q_conf": 0.0, "max_iters": 10,
            "sigma_ray": 1.0, "sigma_dist": 1.0, "delta_norm": 1e-8,
        }
        self.ii = torch.zeros(0, dtype=torch.int64)
        self.jj = torch.zeros(0, dtype=torch.int64)
        self.idx_ii2jj = torch.zeros(0, dtype=torch.int64)
        self.idx_jj2ii = torch.zeros(0, dtype=torch.int64)
        self.valid_match_j = torch.zeros(0, dtype=torch.bool)
        self.valid_match_i = torch.zeros(0, dtype=torch.bool)
        self.Q_ii2jj = torch.zeros(0, dtype=torch.float32)
        self.Q_jj2ii = torch.zeros(0, dtype=torch.float32)
        self.K = K
        self.calib_calls = 0

    def get_unique_kf_idx(self):
        return torch.unique(torch.cat([self.ii, self.jj]), sorted=True)

    def prep_two_way_edges(self):
        ii = torch.cat((self.ii, self.jj), dim=0)
        jj = torch.cat((self.jj, self.ii), dim=0)
        idx_ii2jj = torch.cat((self.idx_ii2jj, self.idx_jj2ii), dim=0)
        valid_match = torch.cat((self.valid_match_j, self.valid_match_i), dim=0)
        Q_ii2jj = torch.cat((self.Q_ii2jj, self.Q_jj2ii), dim=0)
        return ii, jj, idx_ii2jj, valid_match, Q_ii2jj

    def get_poses_points(self, unique_kf_idx):
        data = torch.stack([self.frames.poses[int(i)] for i in unique_kf_idx]).unsqueeze(1)
        n = unique_kf_idx.numel()
        Xs = torch.zeros(n, 1, 1, 3)
        Cs = torch.zeros(n, 1, 1)
        return Xs, _FakePoses(data), Cs

    def solve_GN_rays(self):
        pin = self.cfg["pin"]
        unique_kf_idx = self.get_unique_kf_idx()
        if unique_kf_idx.numel() <= pin:
            return
        Xs, T_WCs, Cs = self.get_poses_points(unique_kf_idx)
        ii, jj, idx_ii2jj, valid_match, Q_ii2jj = self.prep_two_way_edges()
        pose_data = T_WCs.data[:, 0, :]
        type(self).full_solver(
            pose_data, Xs, Cs, ii, jj, idx_ii2jj, valid_match, Q_ii2jj,
            self.cfg["sigma_ray"], self.cfg["sigma_dist"], self.cfg["C_conf"], self.cfg["Q_conf"],
            self.cfg["max_iters"], self.cfg["delta_norm"])
        self.frames.update_T_WCs(T_WCs[pin:], unique_kf_idx[pin:])

    def solve_GN_calib(self):
        self.calib_calls += 1


def _make_fake_gn_rays_fn(call_log, perturb=0.0, raises=False):
    def fake_gn_rays_fn(pose_data, Xs, Cs, ii, jj, idx_ii2jj, valid_match, Q_ii2jj,
                         sigma_ray, sigma_dist, C_thresh, Q_thresh, max_iter, delta_thresh):
        call_log.append({
            "kf_count": int(pose_data.shape[0]),
            "edge_count": int(ii.numel()),
            "ii": ii.clone(),
            "jj": jj.clone(),
        })
        if raises:
            raise RuntimeError("fake solver failure (deliberate, for the restoration test)")
        if perturb:
            pose_data += perturb
    return fake_gn_rays_fn


def _make_chain_graph_frames(extra_edges, n=60, pose_dim=3):
    chain_ii = list(range(0, n - 1))
    chain_jj = list(range(1, n))
    eii = chain_ii + [a for a, b in extra_edges]
    ejj = chain_jj + [b for a, b in extra_edges]
    poses = {i: torch.full((pose_dim,), float(i)) for i in range(n)}
    frames = _FakeFrames(poses)
    return torch.tensor(eii, dtype=torch.int64), torch.tensor(ejj, dtype=torch.int64), frames


def _install_edges(g, ii, jj):
    """Install a chain/loop edge set on a fake graph, sizing all eight per-edge tensors to match --
    real `add_factors` always keeps them in lockstep, and the self-tests bypass `add_factors`
    entirely, so this is what stands in for it."""
    e = ii.numel()
    g.ii, g.jj = ii, jj
    g.idx_ii2jj = torch.zeros(e, dtype=torch.int64)
    g.idx_jj2ii = torch.zeros(e, dtype=torch.int64)
    g.valid_match_j = torch.zeros(e, dtype=torch.bool)
    g.valid_match_i = torch.zeros(e, dtype=torch.bool)
    g.Q_ii2jj = torch.zeros(e, dtype=torch.float32)
    g.Q_jj2ii = torch.zeros(e, dtype=torch.float32)


def _edge_tensors(g):
    return (g.ii, g.jj, g.idx_ii2jj, g.idx_jj2ii, g.valid_match_j, g.valid_match_i,
            g.Q_ii2jj, g.Q_jj2ii)


def _run_windowed_factor_graph_self_tests(check) -> None:
    base_cls = _FakeBaseFactorGraph

    # -- factory caching --
    fn_a = _make_fake_gn_rays_fn([])
    cls1 = make_windowed_factor_graph(base_cls, fn_a)
    cls2 = make_windowed_factor_graph(base_cls, fn_a)
    check("make_windowed_factor_graph caches: same class object twice", cls1 is cls2)

    # -- bad construction --
    WFG = cls1
    _, _, frames0 = _make_chain_graph_frames([])
    try:
        WFG(None, frames0, window_mode="on")
        check("bad window_mode raises ValueError", False)
    except ValueError:
        check("bad window_mode raises ValueError", True)
    try:
        WFG(None, frames0, window_policy="ANCHORED")
        check("bad window_policy raises ValueError", False)
    except ValueError:
        check("bad window_policy raises ValueError", True)

    # -- mode="OFF": full edge count, no window --
    call_log = []
    full_fn = _make_fake_gn_rays_fn(call_log)
    base_cls.full_solver = staticmethod(full_fn)
    ii, jj, frames = _make_chain_graph_frames([(3, 57)])
    WFG = make_windowed_factor_graph(base_cls, _make_fake_gn_rays_fn([]))
    g = WFG(None, frames, window_mode="OFF", window_kf=10)
    _install_edges(g, ii, jj)
    graph_edges = int(g.ii.numel())
    g.solve_GN_rays()
    check("OFF: full_solver saw the full (two-way) edge count",
          call_log[-1]["edge_count"] == graph_edges * 2)
    check("OFF: last_window_stats.mode == 'OFF'", g.last_window_stats.mode == "OFF")
    check("OFF: last_window_stats.solve_edges == graph_edges", g.last_window_stats.solve_edges == graph_edges)
    check("OFF: last_window_stats.anchors == 0", g.last_window_stats.anchors == 0)

    # -- force_full=True on an otherwise-cutting ON graph --
    call_log.clear()
    ii, jj, frames = _make_chain_graph_frames([(3, 57)])
    g = WFG(None, frames, window_mode="ON", window_kf=10)
    _install_edges(g, ii, jj)
    graph_edges = int(g.ii.numel())
    g.solve_GN_rays(force_full=True)
    check("force_full=True: full_solver saw the full edge count",
          call_log[-1]["edge_count"] == graph_edges * 2)
    check("force_full=True: last_window_stats.solve_edges == graph_edges",
          g.last_window_stats.solve_edges == graph_edges)

    # -- mode="SHADOW": full solver still runs on the full graph, stats report the would-be cut --
    call_log.clear()
    ii, jj, frames = _make_chain_graph_frames([(3, 57)])
    g = WFG(None, frames, window_mode="SHADOW", window_kf=10)
    _install_edges(g, ii, jj)
    graph_edges = int(g.ii.numel())
    sel_shadow = select_active_edges(ii, jj, 10, "anchored")
    g.solve_GN_rays()
    check("SHADOW: full_solver still saw the full edge count",
          call_log[-1]["edge_count"] == graph_edges * 2)
    check("SHADOW: last_window_stats.mode == 'SHADOW'", g.last_window_stats.mode == "SHADOW")
    check("SHADOW: last_window_stats.solve_edges is the reduced count",
          g.last_window_stats.solve_edges == int(sel_shadow.mask.sum()))
    check("SHADOW: last_window_stats.anchors is the would-be anchor count",
          g.last_window_stats.anchors == sel_shadow.n_anchors)
    check("SHADOW: last_window_stats.graph_edges == graph_edges",
          g.last_window_stats.graph_edges == graph_edges)

    # -- mode="ON" on chain + 1 loop edge: real cut --
    win_log = []
    win_fn = _make_fake_gn_rays_fn(win_log)
    WFG_on = make_windowed_factor_graph(base_cls, win_fn)
    ii, jj, frames = _make_chain_graph_frames([(3, 57)])
    g = WFG_on(None, frames, window_mode="ON", window_kf=10)
    _install_edges(g, ii, jj)
    originals = _edge_tensors(g)
    graph_edges = int(g.ii.numel())
    sel = select_active_edges(ii, jj, 10, "anchored")
    g.solve_GN_rays()
    check("ON: solver saw the reduced (two-way) edge count",
          win_log[-1]["edge_count"] == int(sel.mask.sum()) * 2)
    check("ON: last_window_stats.solve_kf == len(active_kf)",
          g.last_window_stats.solve_kf == sel.active_kf.numel())
    seen_kf = set(win_log[-1]["ii"].tolist()) | set(win_log[-1]["jj"].tolist())
    check("ON: recorded ii/jj handed to the solver contain only active keyframes",
          seen_kf <= set(sel.active_kf.tolist()))
    check("ON: last_window_stats.graph_edges == full graph_edges",
          g.last_window_stats.graph_edges == graph_edges)

    # -- restoration on success: all eight tensors are the original objects afterward --
    same_object = all(a is b for a, b in zip(originals, _edge_tensors(g)))
    check("ON: all eight tensors restored to original objects", same_object)
    check("ON: self.ii.numel() == graph_edges after restore", g.ii.numel() == graph_edges)

    # -- restoration on failure: solver raises, tensors still restored, exception propagates --
    raising_fn = _make_fake_gn_rays_fn([], raises=True)
    WFG_fail = make_windowed_factor_graph(base_cls, raising_fn)
    ii, jj, frames = _make_chain_graph_frames([(3, 57)])
    g = WFG_fail(None, frames, window_mode="ON", window_kf=10)
    _install_edges(g, ii, jj)
    originals = _edge_tensors(g)
    graph_edges = int(g.ii.numel())
    raised = False
    try:
        g.solve_GN_rays()
    except RuntimeError:
        raised = True
    check("restoration-on-failure: exception propagated, not swallowed", raised)
    check("restoration-on-failure: all eight tensors restored",
          all(a is b for a, b in zip(originals, _edge_tensors(g))))
    check("restoration-on-failure: self.ii.numel() == graph_edges after restore",
          g.ii.numel() == graph_edges)

    # -- write-back: 2 anchors, pin=1 --
    # A plain chain always contributes one "boundary" anchor: the edge (window_lo - 1, window_lo)
    # has exactly one endpoint in-window, so window_lo - 1 always joins active_kf as an anchor (see
    # module docstring: "the lowest sorted index in any windowed solve is always an anchor"). One
    # explicit loop edge on top of that chain therefore already yields exactly 2 anchors total.
    def build_two_anchor_graph(policy, gn_fn):
        WFG_p = make_windowed_factor_graph(base_cls, gn_fn)
        ii, jj, frames = _make_chain_graph_frames([(3, 57)])
        g = WFG_p(None, frames, window_mode="ON", window_kf=10, window_policy=policy)
        _install_edges(g, ii, jj)
        sel = select_active_edges(ii, jj, 10, policy)
        return g, sel

    g_anchored, sel2 = build_two_anchor_graph("anchored", _make_fake_gn_rays_fn([], perturb=0.0))
    check("2 anchors: sanity n_anchors == 2", sel2.n_anchors == 2)
    g_anchored.solve_GN_rays()
    written_idx = g_anchored.frames.update_calls[-1][1]
    check("anchored write-back: starts at active_kf[2]",
          written_idx.numel() > 0 and int(written_idx[0]) == int(sel2.active_kf[2]))
    check("anchored write-back: written count == len(active_kf) - 2",
          written_idx.numel() == sel2.active_kf.numel() - 2)

    g_free, sel2b = build_two_anchor_graph("free", _make_fake_gn_rays_fn([], perturb=0.0))
    g_free.solve_GN_rays()
    written_idx_free = g_free.frames.update_calls[-1][1]
    check("free write-back: starts at active_kf[1]",
          written_idx_free.numel() > 0 and int(written_idx_free[0]) == int(sel2b.active_kf[1]))
    check("free write-back: anchor active_kf[1] is written (not just active_kf[2])",
          int(sel2b.active_kf[1]) in written_idx_free.tolist())

    # -- drift: fake solver perturbs anchor rows by a known delta --
    # perturb adds a scalar to every element of pose_data in place; each pose row has pose_dim=3
    # elements, so the per-anchor displacement norm is delta * sqrt(pose_dim).
    delta = 0.25
    g_drift, sel_drift = build_two_anchor_graph("anchored", _make_fake_gn_rays_fn([], perturb=delta))
    g_drift.solve_GN_rays()
    expected_drift = abs(delta) * (3 ** 0.5)
    check("drift: anchor_drift matches known perturbation within 1e-5",
          abs(g_drift.last_window_stats.anchor_drift - expected_drift) < 1e-5)

    ii, jj, frames = _make_chain_graph_frames([(3, 57)])
    WFG_nd = make_windowed_factor_graph(base_cls, _make_fake_gn_rays_fn([], perturb=1.0))
    g_nd = WFG_nd(None, frames, window_mode="ON", window_kf=10, window_policy="strict")
    _install_edges(g_nd, ii, jj)
    sel_check = select_active_edges(ii, jj, 10, "strict")
    check("drift sanity: strict policy on this graph has 0 anchors", sel_check.n_anchors == 0)
    g_nd.solve_GN_rays()
    check("drift: zero anchors -> anchor_drift exactly 0.0", g_nd.last_window_stats.anchor_drift == 0.0)

    # -- solve_GN_calib: NotImplementedError when windowed, delegates when OFF --
    WFG_calib = make_windowed_factor_graph(base_cls, _make_fake_gn_rays_fn([]))
    ii, jj, frames = _make_chain_graph_frames([])
    g_on = WFG_calib(None, frames, window_mode="ON", window_kf=10)
    _install_edges(g_on, ii, jj)
    try:
        g_on.solve_GN_calib()
        check("solve_GN_calib raises NotImplementedError when window_mode != OFF", False)
    except NotImplementedError:
        check("solve_GN_calib raises NotImplementedError when window_mode != OFF", True)

    g_off = WFG_calib(None, frames, window_mode="OFF")
    g_off.ii, g_off.jj = ii, jj
    g_off.solve_GN_calib()
    check("solve_GN_calib delegates to super() when window_mode == OFF", g_off.calib_calls == 1)

    # -- mismatched active_kf raises RuntimeError --
    ii, jj, frames = _make_chain_graph_frames([(3, 57)])
    g_bad = WFG_calib(None, frames, window_mode="ON", window_kf=10)
    g_bad.ii, g_bad.jj = ii, jj
    real_sel = select_active_edges(ii, jj, 10, "anchored")
    bad_sel = replace(real_sel, active_kf=torch.tensor([999], dtype=torch.int64))
    try:
        g_bad._solve_GN_rays_windowed(bad_sel)
        check("mismatched active_kf raises RuntimeError", False)
    except RuntimeError:
        check("mismatched active_kf raises RuntimeError", True)


def run_self_test() -> None:
    failures = []

    def check(label, cond):
        status = "PASS" if cond else "FAIL"
        print(f"[self-test] {status}  {label}")
        if not cond:
            failures.append(label)

    def tensor_eq(a, b):
        a = torch.as_tensor(a, dtype=torch.int64)
        return a.numel() == b.numel() and bool(torch.equal(a, b.to(torch.int64)))

    # -- SESSION 66: the sync-free host-side scalars ------------------------------------------
    # These exist so a caller never asks a CUDA tensor a question just to branch. The contract is
    # that they agree EXACTLY with the tensor answers they replace -- if they ever drift, the ON
    # path silently takes the wrong branch, which is far worse than the sync they removed.
    for w, pol in ((0, "anchored"), (-5, "anchored"), (10, "anchored"),
                   (10, "strict"), (10, "free"), (1000, "anchored")):
        ii_s = torch.tensor([0, 1, 2, 3, 4, 5, 6, 7, 8, 3], dtype=torch.int64)
        jj_s = torch.tensor([1, 2, 3, 4, 5, 6, 7, 8, 9, 9], dtype=torch.int64)
        sel_s = select_active_edges(ii_s, jj_s, w, pol)
        check(f"W={w} {pol}: n_active_edges == int(mask.sum()) (no-sync scalar agrees)",
              sel_s.n_active_edges == int(sel_s.mask.sum()))
        check(f"W={w} {pol}: all_active == bool(mask.all()) (no-sync scalar agrees)",
              sel_s.all_active == bool(sel_s.mask.all()))

    # A REAL cut must report all_active False -- this is the branch that decides whether the
    # expensive tensor swap happens at all.
    ii_cut = torch.arange(0, 59, dtype=torch.int64)
    jj_cut = torch.arange(1, 60, dtype=torch.int64)
    sel_cut = select_active_edges(ii_cut, jj_cut, 10, "anchored")
    check("real cut: all_active is False and n_active_edges < E",
          sel_cut.all_active is False and sel_cut.n_active_edges < ii_cut.numel())
    check("real cut: n_active_edges == int(mask.sum())",
          sel_cut.n_active_edges == int(sel_cut.mask.sum()))
    check("no-cut: window wider than graph -> all_active True",
          select_active_edges(ii_cut, jj_cut, 1000, "anchored").all_active is True)
    check("empty graph: all_active False, n_active_edges 0 (nothing to solve, not 'everything')",
          select_active_edges(torch.zeros(0, dtype=torch.int64),
                              torch.zeros(0, dtype=torch.int64), 10, "anchored").n_active_edges == 0)
    check("host-side selection still returns tensors on the input device",
          sel_cut.mask.device == ii_cut.device and sel_cut.active_kf.device == ii_cut.device
          and sel_cut.anchor_idx.device == ii_cut.device)

    # -- disabled path is provably identity --
    for w in (0, -5):
        ii = torch.arange(0, 10, dtype=torch.int64)
        jj = torch.arange(1, 11, dtype=torch.int64)
        sel = select_active_edges(ii, jj, w, "anchored")
        check(f"window_kf={w}: mask all True", bool(sel.mask.all()) and sel.mask.numel() == ii.numel())
        check(f"window_kf={w}: n_anchors == 0", sel.n_anchors == 0)
        check(f"window_kf={w}: window_lo == -1", sel.window_lo == -1)

    # -- window larger than the whole graph --
    ii = torch.arange(0, 20, dtype=torch.int64)
    jj = torch.arange(1, 21, dtype=torch.int64)
    sel = select_active_edges(ii, jj, 1000, "anchored")
    check("window_kf > graph: mask all True", bool(sel.mask.all()))
    check("window_kf > graph: n_anchors == 0", sel.n_anchors == 0)

    # -- base chain + loop edges, W=10 --
    chain_ii = list(range(0, 59))
    chain_jj = list(range(1, 60))

    def make_graph(extra_edges):
        eii = chain_ii + [a for a, b in extra_edges]
        ejj = chain_jj + [b for a, b in extra_edges]
        return torch.tensor(eii, dtype=torch.int64), torch.tensor(ejj, dtype=torch.int64)

    # A consecutive chain's own boundary edge (window_lo - 1, window_lo) has exactly one endpoint
    # in-window, so under "anchored"'s OR semantics that edge survives and window_lo - 1 ALWAYS
    # joins active_kf as an anchor (module docstring: "the lowest sorted index in any windowed
    # solve is always an anchor") -- a plain chain with no explicit loop edge still has 1 anchor.
    ii, jj = make_graph([])
    sel = select_active_edges(ii, jj, 10, "anchored")
    kept_i, kept_j = ii[sel.mask], jj[sel.mask]
    touches_window = ((kept_i >= 50) | (kept_j >= 50))
    check("consecutive chain W=10: surviving edges all touch [50,59]", bool(touches_window.all()))
    check("consecutive chain W=10: surviving edge count == 10",
          int(sel.mask.sum()) == 10)
    check("consecutive chain W=10: active_kf contiguous incl. boundary anchor 49",
          tensor_eq(list(range(49, 60)), sel.active_kf))
    check("consecutive chain W=10: n_anchors == 1 (the boundary anchor, kf 49)", sel.n_anchors == 1)

    ii, jj = make_graph([(3, 57)])
    sel = select_active_edges(ii, jj, 10, "anchored")
    loop_edge_kept = bool(sel.mask[len(chain_ii)])
    check("chain + loop(3,57) W=10: loop edge kept", loop_edge_kept)
    check("chain + loop(3,57) W=10: n_anchors == 2 (loop kf 3 + boundary kf 49)", sel.n_anchors == 2)
    check("chain + loop(3,57) W=10: anchor_idx == [3, 49]", tensor_eq([3, 49], sel.anchor_idx))
    check("chain + loop(3,57) W=10: active_kf[0] == 3", int(sel.active_kf[0]) == 3)

    ii, jj = make_graph([(3, 57), (30, 55)])
    sel = select_active_edges(ii, jj, 10, "anchored")
    check("chain + 2 loops W=10: n_anchors == 3 (loops kf 3,30 + boundary kf 49)", sel.n_anchors == 3)
    check("chain + 2 loops W=10: anchor_idx == active_kf[:3] == [3,30,49]",
          tensor_eq([3, 30, 49], sel.anchor_idx) and tensor_eq([3, 30, 49], sel.active_kf[:3]))

    sel_strict = select_active_edges(ii, jj, 10, "strict")
    check("strict policy: both loop edges dropped", sel_strict.n_anchors == 0)
    strict_subset = bool(torch.all(~sel_strict.mask | sel.mask)) and int(sel_strict.mask.sum()) < int(sel.mask.sum())
    check("strict policy: mask is a strict subset of anchored's", strict_subset)

    sel_free = select_active_edges(ii, jj, 10, "free")
    check("free policy: same mask as anchored", tensor_eq(sel.mask.to(torch.int64), sel_free.mask.to(torch.int64)))

    # -- kernel-contract test: non-contiguous active_kf round-trips through searchsorted --
    ii, jj = make_graph([(3, 57), (30, 55)])
    sel = select_active_edges(ii, jj, 10, "anchored")
    ii_masked, jj_masked = ii[sel.mask], jj[sel.mask]
    local_ii, local_jj = local_edge_indices(sel.active_kf, ii_masked, jj_masked)
    in_range = bool(((local_ii >= 0) & (local_ii < sel.active_kf.numel())).all()) and \
        bool(((local_jj >= 0) & (local_jj < sel.active_kf.numel())).all())
    check("kernel-contract: local indices in [0, len(active_kf))", in_range)
    roundtrip = bool(torch.equal(sel.active_kf[local_ii], ii_masked)) and \
        bool(torch.equal(sel.active_kf[local_jj], jj_masked))
    check("kernel-contract: active_kf[local_idx] round-trips", roundtrip)

    # -- E == 0 --
    ii0 = torch.zeros(0, dtype=torch.int64)
    jj0 = torch.zeros(0, dtype=torch.int64)
    sel0 = select_active_edges(ii0, jj0, 10, "anchored")
    check("E==0: no crash, mask empty", sel0.mask.numel() == 0)
    check("E==0: window_lo == -1", sel0.window_lo == -1)
    check("E==0: n_anchors == 0", sel0.n_anchors == 0)
    check("E==0: active_kf empty", sel0.active_kf.numel() == 0)

    # -- validate_window_config --
    for bad_mode in ("on", "Shadow", "off"):
        try:
            validate_window_config(bad_mode, 10, "anchored")
            check(f"validate_window_config rejects mode={bad_mode!r}", False)
        except ValueError:
            check(f"validate_window_config rejects mode={bad_mode!r}", True)

    for bad_policy in ("ANCHORED", "maybe", "Strict"):
        try:
            validate_window_config("OFF", 10, bad_policy)
            check(f"validate_window_config rejects policy={bad_policy!r}", False)
        except ValueError:
            check(f"validate_window_config rejects policy={bad_policy!r}", True)

    try:
        validate_window_config("OFF", 10.5, "anchored")
        check("validate_window_config rejects float window_kf", False)
    except ValueError:
        check("validate_window_config rejects float window_kf", True)

    mode, w, policy = validate_window_config("ON", -3, "strict")
    check("validate_window_config normalises -3 -> 0", w == 0)

    for m in WINDOW_MODES:
        for p in WINDOW_POLICIES:
            mode, w, policy = validate_window_config(m, 5, p)
            check(f"validate_window_config accepts ({m!r}, 5, {p!r})",
                  mode == m and w == 5 and policy == p)

    # -- device consistency --
    ii = torch.arange(0, 20, dtype=torch.int64)
    jj = torch.arange(1, 21, dtype=torch.int64)
    sel = select_active_edges(ii, jj, 10, "anchored")
    check("returned tensors share ii.device",
          sel.mask.device == ii.device and sel.anchor_idx.device == ii.device
          and sel.active_kf.device == ii.device)

    _run_windowed_factor_graph_self_tests(check)

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
