"""Session 69: instrument relocalisation without touching `third_party/`.

Flight 20260917_175917 lost tracking at 2.5 min and then made 466 relocalisation attempts over ten
minutes, every one failing, while the autopilot's visual matcher reported known scenery on 85-90 %
of samples. The suspected reason is the acceptance rule, not the view: `_relocalization` asks
retrieval for its k=3 best keyframes and calls `FactorGraph.add_factors(..., is_reloc=True)`, which
(`third_party/MASt3R-SLAM/mast3r_slam/global_opt.py:74-77`) rejects the WHOLE attempt if ANY candidate's
mutual match fraction is below `reloc.min_match_frac` (0.3) -- and this room's consecutive-keyframe
match fractions run 0.19-0.38. That is a mechanism, not a measurement: nothing in the stack records
which keyframes were proposed or how well each matched, because those numbers are locals inside
`add_factors` and it returns a bare bool.

So, exactly as session 67 did for `FrameTracker.track()`: the vendored repo stays pristine and
`make_reloc_instrumented_factor_graph` builds a one-level subclass whose `add_factors` is the upstream
body VERBATIM plus one recorded struct (`last_add_factors_stats`) -- the per-candidate fractions, the
threshold, and the verdict. `slam_engine._relocalization` turns that into a `RelocStats` per attempt
that rides the perception CSV (`SLAM_RELOC_FIELDS`) and one console line, so a flight can SHOW whether
candidate #1 passes and #2/#3 veto, before anyone decides to loosen `strict`/`min_match_frac`/`k`.

Module-level imports are stdlib + `torch` only; `mast3r_match_symmetric` is passed INTO the factory
(the same injection `slam_window.make_windowed_factor_graph` uses for `gauss_newton_rays`) so the
self-test can run the vendored body against a stub matcher on a box with no model and no CUDA.
"""

import dataclasses
from dataclasses import dataclass

try:
    import torch
except ImportError:  # pragma: no cover -- exercised only on a bare interpreter with no torch
    torch = None


@dataclass(frozen=True)
class AddFactorsStats:
    """What ONE `add_factors` call saw, recorded by the instrumented subclass. Frozen and REPLACED by
    whole-object attribute rebind, never mutated (the lock-free-read argument of
    slam_engine.py's last_window_stats / last_track_stats)."""
    ii: tuple            # edge sources (the reloc frame's index, repeated) or graph sources
    jj: tuple            # edge targets (retrieval candidates during reloc)
    match_frac_j: tuple  # per edge: fraction of j's pixels with a valid confident match into i
    match_frac_i: tuple  # per edge: the reverse direction
    min_match_frac: float
    is_reloc: bool
    accepted: bool       # the bool add_factors returned


@dataclass(frozen=True)
class RelocStats:
    """One relocalisation ATTEMPT (one `_relocalization` call), as it rides the perception CSV.
    Defaults = "no attempt on this frame"; `reloc_attempt` is a cumulative counter so a row with
    `reloc_attempt > 0` is exactly a row on which an attempt ran (CLAUDE.md: "did not run" must be
    distinguishable from a genuine zero)."""
    reloc_attempt: int = 0      # cumulative attempts this session; 0 = none on this frame
    reloc_n_cand: int = 0       # keyframes retrieval proposed (0..k)
    reloc_cands: str = ""       # their keyframe indices, "3|17|22"
    reloc_fracs: str = ""       # per candidate min(match_frac_j, match_frac_i), "0.412|0.220|0.081"
    reloc_best_frac: float = 0.0  # max over candidates of that min -- the number a non-strict rule would judge
    reloc_vetoed: int = 0       # candidates below min_match_frac -- under strict=True, >0 fails the attempt
    reloc_ok: int = 0           # 1 = accepted (tracking recovered), 0 = rejected
    reloc_ms: float = 0.0       # retrieval query + add_factors, wall-clock


RELOC_STATS_ABSENT: RelocStats = RelocStats()

RELOC_FIELDS: tuple = tuple(f.name for f in dataclasses.fields(RelocStats))


def build_reloc_stats(attempt: int, cands, stats, min_match_frac: float, ms: float) -> RelocStats:
    """Fold one attempt into a RelocStats. `stats` is the AddFactorsStats the instrumented graph
    recorded, or None when retrieval proposed nothing (then add_factors never ran)."""
    cands = [int(c) for c in cands]
    if stats is None:
        return RelocStats(reloc_attempt=attempt, reloc_n_cand=0, reloc_cands="", reloc_fracs="",
                          reloc_best_frac=0.0, reloc_vetoed=0, reloc_ok=0, reloc_ms=round(ms, 1))
    mins = [min(a, b) for a, b in zip(stats.match_frac_j, stats.match_frac_i)]
    return RelocStats(
        reloc_attempt=attempt, reloc_n_cand=len(cands),
        reloc_cands="|".join(str(c) for c in cands),
        reloc_fracs="|".join(f"{m:.3f}" for m in mins),
        reloc_best_frac=round(max(mins), 4) if mins else 0.0,
        reloc_vetoed=sum(1 for m in mins if m < min_match_frac),
        reloc_ok=int(bool(stats.accepted)), reloc_ms=round(ms, 1))


def format_console_line(s: RelocStats, min_match_frac: float, strict: bool) -> str:
    verdict = "OK -> TRACKING" if s.reloc_ok else ("FAIL (no candidates)" if s.reloc_n_cand == 0
                                                     else f"FAIL (vetoed {s.reloc_vetoed}/{s.reloc_n_cand})")
    return (f"[slam] RELOC attempt #{s.reloc_attempt}: cands=[{s.reloc_cands}] "
            f"min_frac=[{s.reloc_fracs}] thr={min_match_frac:.2f} strict={strict} "
            f"best={s.reloc_best_frac:.3f} -> {verdict} ({s.reloc_ms:.0f} ms)")


def make_reloc_instrumented_factor_graph(base_cls: type, match_fn) -> type:
    """Subclass `base_cls` (upstream FactorGraph, or slam_window's windowed subclass of it) so that
    `add_factors` records `last_add_factors_stats`. The body below is
    `third_party/MASt3R-SLAM/mast3r_slam/global_opt.py:29-98` verbatim; the ONLY additions are the
    `AddFactorsStats` rebinds before each return. `match_fn` is `mast3r_slam.mast3r_utils.
    mast3r_match_symmetric` in production and a stub in the self-test."""

    class RelocInstrumentedFactorGraph(base_cls):
        last_add_factors_stats: "AddFactorsStats | None" = None

        def add_factors(self, ii, jj, min_match_frac, is_reloc=False):
            kf_ii = [self.frames[idx] for idx in ii]
            kf_jj = [self.frames[idx] for idx in jj]
            feat_i = torch.cat([kf_i.feat for kf_i in kf_ii])
            feat_j = torch.cat([kf_j.feat for kf_j in kf_jj])
            pos_i = torch.cat([kf_i.pos for kf_i in kf_ii])
            pos_j = torch.cat([kf_j.pos for kf_j in kf_jj])
            shape_i = [kf_i.img_true_shape for kf_i in kf_ii]
            shape_j = [kf_j.img_true_shape for kf_j in kf_jj]

            (
                idx_i2j,
                idx_j2i,
                valid_match_j,
                valid_match_i,
                Qii,
                Qjj,
                Qji,
                Qij,
            ) = match_fn(
                self.model, feat_i, pos_i, feat_j, pos_j, shape_i, shape_j
            )

            batch_inds = torch.arange(idx_i2j.shape[0], device=idx_i2j.device)[
                :, None
            ].repeat(1, idx_i2j.shape[1])
            Qj = torch.sqrt(Qii[batch_inds, idx_i2j] * Qji)
            Qi = torch.sqrt(Qjj[batch_inds, idx_j2i] * Qij)

            valid_Qj = Qj > self.cfg["Q_conf"]
            valid_Qi = Qi > self.cfg["Q_conf"]
            valid_j = valid_match_j & valid_Qj
            valid_i = valid_match_i & valid_Qi
            nj = valid_j.shape[1] * valid_j.shape[2]
            ni = valid_i.shape[1] * valid_i.shape[2]
            match_frac_j = valid_j.sum(dim=(1, 2)) / nj
            match_frac_i = valid_i.sum(dim=(1, 2)) / ni

            ii_tensor = torch.as_tensor(ii, device=self.device)
            jj_tensor = torch.as_tensor(jj, device=self.device)

            # NOTE: Saying we need both edge directions to be above thrhreshold to accept either
            invalid_edges = torch.minimum(match_frac_j, match_frac_i) < min_match_frac
            consecutive_edges = ii_tensor == (jj_tensor - 1)
            invalid_edges = (~consecutive_edges) & invalid_edges

            # Session 69 (addition): record what this call saw, whatever it decides below.
            _stats = AddFactorsStats(
                ii=tuple(int(x) for x in ii), jj=tuple(int(x) for x in jj),
                match_frac_j=tuple(float(x) for x in match_frac_j.detach().cpu().tolist()),
                match_frac_i=tuple(float(x) for x in match_frac_i.detach().cpu().tolist()),
                min_match_frac=float(min_match_frac), is_reloc=bool(is_reloc), accepted=False)

            if invalid_edges.any() and is_reloc:
                self.last_add_factors_stats = _stats                      # session 69
                return False

            valid_edges = ~invalid_edges
            ii_tensor = ii_tensor[valid_edges]
            jj_tensor = jj_tensor[valid_edges]
            idx_i2j = idx_i2j[valid_edges]
            idx_j2i = idx_j2i[valid_edges]
            valid_match_j = valid_match_j[valid_edges]
            valid_match_i = valid_match_i[valid_edges]
            Qj = Qj[valid_edges]
            Qi = Qi[valid_edges]

            self.ii = torch.cat([self.ii, ii_tensor])
            self.jj = torch.cat([self.jj, jj_tensor])
            self.idx_ii2jj = torch.cat([self.idx_ii2jj, idx_i2j])
            self.idx_jj2ii = torch.cat([self.idx_jj2ii, idx_j2i])
            self.valid_match_j = torch.cat([self.valid_match_j, valid_match_j])
            self.valid_match_i = torch.cat([self.valid_match_i, valid_match_i])
            self.Q_ii2jj = torch.cat([self.Q_ii2jj, Qj])
            self.Q_jj2ii = torch.cat([self.Q_jj2ii, Qi])

            added_new_edges = valid_edges.sum() > 0
            self.last_add_factors_stats = dataclasses.replace(                # session 69
                _stats, accepted=bool(added_new_edges))
            return added_new_edges

    RelocInstrumentedFactorGraph.__name__ = f"RelocInstrumented{base_cls.__name__}"
    RelocInstrumentedFactorGraph.__qualname__ = RelocInstrumentedFactorGraph.__name__
    return RelocInstrumentedFactorGraph


# ----------------------------------------------------------------------------- self-test
class _StubFrame:
    def __init__(self, h=2, w=3):
        self.feat = torch.zeros(1, 1, 4)
        self.pos = torch.zeros(1, 1, 2)
        self.img_true_shape = torch.tensor([[h, w]])


class _StubBaseGraph:
    """The attributes upstream add_factors touches, nothing else."""
    def __init__(self, n_frames=6, h=2, w=3):
        self.frames = [_StubFrame(h, w) for _ in range(n_frames)]
        self.model = None
        self.device = "cpu"
        self.cfg = {"Q_conf": 0.5}
        e = torch.as_tensor([], dtype=torch.long)
        self.ii = e.clone(); self.jj = e.clone()
        self.idx_ii2jj = e.clone(); self.idx_jj2ii = e.clone()
        self.valid_match_j = torch.as_tensor([], dtype=torch.bool)
        self.valid_match_i = torch.as_tensor([], dtype=torch.bool)
        self.Q_ii2jj = torch.as_tensor([], dtype=torch.float32)
        self.Q_jj2ii = torch.as_tensor([], dtype=torch.float32)


def _make_stub_matcher(fracs, h=2, w=3):
    """A `mast3r_match_symmetric` stand-in with upstream's shapes -- idx (b, h*w), valid_match
    (b, h*w, 1), Q (b, h*w, 1) (mast3r_utils.py:171-179): edge e gets exactly round(fracs[e] * h*w)
    valid, confident pixels in BOTH directions, so min(match_frac_j, match_frac_i) == fracs[e]."""
    def match_fn(model, feat_i, pos_i, feat_j, pos_j, shape_i, shape_j):
        b = len(shape_i)
        idx = torch.zeros(b, h * w, dtype=torch.long)
        valid = torch.zeros(b, h * w, 1, dtype=torch.bool)
        for e, f in enumerate(fracs):
            k = int(round(f * h * w))
            valid[e, :k, 0] = True
        Q = torch.ones(b, h * w, 1)
        return idx, idx.clone(), valid, valid.clone(), Q, Q.clone(), Q.clone(), Q.clone()
    return match_fn


def run_self_test() -> None:
    ok = True

    def check(label, cond):
        nonlocal ok
        ok = ok and bool(cond)
        print(f"[reloc-stats][self-test] {'PASS' if cond else 'FAIL'}  {label}")

    check("reloc_fields_tuple -- RELOC_FIELDS is the frozen eight, in order",
          RELOC_FIELDS == ("reloc_attempt", "reloc_n_cand", "reloc_cands", "reloc_fracs",
                           "reloc_best_frac", "reloc_vetoed", "reloc_ok", "reloc_ms"))
    check("absent_is_zero -- RELOC_STATS_ABSENT.reloc_attempt == 0 (no attempt on this frame)",
          RELOC_STATS_ABSENT.reloc_attempt == 0 and RELOC_STATS_ABSENT.reloc_ok == 0)

    # 1. strict reloc, one weak candidate vetoes: fracs 0.5 / 0.17 / 0.83 against thr 0.3.
    G = make_reloc_instrumented_factor_graph(_StubBaseGraph, _make_stub_matcher([0.5, 1 / 6, 5 / 6]))
    g = G()
    res = g.add_factors([5, 5, 5], [0, 2, 3], 0.3, is_reloc=True)
    s = g.last_add_factors_stats
    check("strict_veto_returns_false -- one candidate at 0.17 < 0.3 rejects the attempt", res is False)
    check("strict_veto_records_all_three -- fractions recorded even though rejected",
          s is not None and len(s.match_frac_j) == 3 and abs(s.match_frac_j[1] - 1 / 6) < 1e-6
          and s.accepted is False and s.is_reloc is True)
    check("strict_veto_adds_no_edges -- graph untouched on rejection", g.ii.numel() == 0)
    rs = build_reloc_stats(7, [0, 2, 3], s, 0.3, 12.34)
    check("reloc_stats_fold -- attempt 7, 3 cands, best 0.833, vetoed 1, ok 0",
          rs.reloc_attempt == 7 and rs.reloc_n_cand == 3 and rs.reloc_cands == "0|2|3"
          and rs.reloc_fracs == "0.500|0.167|0.833" and abs(rs.reloc_best_frac - 0.8333) < 1e-3
          and rs.reloc_vetoed == 1 and rs.reloc_ok == 0 and rs.reloc_ms == 12.3)
    line = format_console_line(rs, 0.3, True)
    check("console_line -- names the veto count", "vetoed 1/3" in line and "#7" in line)

    # 2. strict reloc, all candidates pass -> accepted, edges added, stats.accepted True.
    G2 = make_reloc_instrumented_factor_graph(_StubBaseGraph, _make_stub_matcher([0.5, 5 / 6]))
    g2 = G2()
    res2 = g2.add_factors([5, 5], [0, 2], 0.3, is_reloc=True)
    check("strict_pass_returns_true -- both candidates >= 0.3 accept the attempt", bool(res2) is True)
    check("strict_pass_adds_two_edges", g2.ii.numel() == 2 and g2.last_add_factors_stats.accepted is True)
    rs2 = build_reloc_stats(8, [0, 2], g2.last_add_factors_stats, 0.3, 5.0)
    check("reloc_stats_ok -- vetoed 0, ok 1", rs2.reloc_vetoed == 0 and rs2.reloc_ok == 1)
    check("console_line_ok", "OK -> TRACKING" in format_console_line(rs2, 0.3, True))

    # 3. NON-reloc add_factors (normal backend): a weak non-consecutive edge is dropped, the rest
    #    kept -- upstream behaviour preserved, stats still recorded.
    G3 = make_reloc_instrumented_factor_graph(_StubBaseGraph, _make_stub_matcher([1 / 6, 0.5]))
    g3 = G3()
    res3 = g3.add_factors([4, 1], [5, 5], 0.3, is_reloc=False)   # [4]->[5] consecutive, [1]->[5] not
    check("backend_consecutive_kept_even_if_weak -- upstream rule, 2 edges in, 2 kept",
          bool(res3) is True and g3.ii.numel() == 2)
    G4 = make_reloc_instrumented_factor_graph(_StubBaseGraph, _make_stub_matcher([0.5, 1 / 6]))
    g4 = G4()
    g4.add_factors([4, 1], [5, 5], 0.3, is_reloc=False)
    check("backend_weak_loop_edge_dropped -- non-consecutive 0.17 edge dropped, consecutive kept",
          g4.ii.numel() == 1 and int(g4.ii[0]) == 4)

    # 4. no candidates: build_reloc_stats with stats=None
    rs0 = build_reloc_stats(9, [], None, 0.3, 3.0)
    check("no_candidates -- n_cand 0, ok 0, still counted as attempt 9",
          rs0.reloc_n_cand == 0 and rs0.reloc_ok == 0 and rs0.reloc_attempt == 9)
    check("console_line_no_candidates", "no candidates" in format_console_line(rs0, 0.3, True))

    check("subclass_name", G.__name__ == "RelocInstrumented_StubBaseGraph")
    print("[reloc-stats][self-test]", "ALL PASS" if ok else "FAILURES")
    if not ok:
        raise SystemExit(1)


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args()
    if a.self_test:
        run_self_test()
    else:
        ap.print_help()
