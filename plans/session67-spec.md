# Session 67 — Instrument `tracker.track()` — Sonnet-Ready Implementation Specification

## Execution Guidelines (read first, obey exactly)

- **Implement ONLY the chunk you were asked for.** Do not start the next one, do not "while I'm here"
  anything.
- **Adhere strictly to the signatures, names, types and return shapes in this document.** They are
  contracts other chunks compile against. If a name here disagrees with your instinct, this document wins.
- **Make no architectural changes.** No new modules beyond the one named, no refactors of neighbouring
  code, no reordering or renaming of existing CSV columns, no config-key changes.
- **Never edit anything under `third_party/`.** It is gitignored and the project rule is that the
  vendored repo stays pristine (`slam_engine.py:234-236`). Every change lands in tracked repo-root files.
- **NO SILENT FALLBACKS (`CLAUDE.md`).** No `try/except` that swallows. Every `except` in this spec
  re-raises. A missing value is a visibly missing value (`""`), never a plausible-looking zero.
- **No behaviour changes.** This session only measures. Do not touch `max_iters`, `rel_error`,
  `delta_norm`, `min_match_frac`, `match_frac_thresh`, or any autopilot logic.
- Match the surrounding file's comment density and idiom. Every new constant gets a comment saying
  **why**, in the style of the session 62/63/64/65 blocks already in these files.
- Run the chunk's acceptance tests before reporting completion. If one fails, fix it; do not report done.

---

## Context (why this exists)

`STATE.md`'s top item: **the problem is an in-flight accumulator; everything else is secondary.**
`tracker_ms` is flat at ~250–400 ms on a deterministic replay of the exact frames a flight consumed, and
2 000–6 000 ms live on the flight those frames came from. Every live flight starts at ~370 ms on the
ground, steps up 7–17× within two minutes of takeoff, and never returns. The frames, the session 63–66
speed work, and plain GPU contention are ruled out.

`tracker.track()` (`third_party/MASt3R-SLAM/mast3r_slam/tracker.py:28`) is not just a solve:

| part | line | cost shape |
|---|---|---|
| `mast3r_match_asymmetric(...)` | `:31` | ViT-Large forward + `iter_proj` + `refine_matches` — **fixed work** |
| pointmap update / `get_points_poses` / validity masks | `:44`–`:65` | fixed |
| `min_match_frac` skip gate | `:67`–`:70` | early return, no solve |
| `opt_pose_ray_dist_sim3(...)` | `:75` → `:173` | **the only data-dependent work** — GN loop, 1…`max_iters: 50` |
| keyframe write-back + selection | `:95`–`:114` | fixed |

So the fixed-work/solve split is as decisive as the iteration count, and neither is visible today.
`slam_engine.py:239` sets `config["use_calib"] = False`, so `opt_pose_ray_dist_sim3` is the **only** live
GN path. `check_convergence` (`nonlinear_optimizer.py:5-25`) returns a bare bool, so the exit *reason*
must be re-derived. On iteration 0 `old_cost = inf` ⇒ `rel_dec = nan` ⇒ convergence there can only fire
via `delta_norm`.

**Verdict this measurement produces:** iterations climb live but stay flat on the matched replay → the
warm-start feedback loop (`idx_f2k`) is the accumulator. Iterations flat while `tracker_ms` rises → the
solver is eliminated and `trk_pre_ms` vs `trk_solve_ms` says which half grows, pointing at process-level
state a replay never accumulates.

### Known limits of this design, accepted deliberately

Because `third_party/` stays pristine, `track()` is **wrapped, not copied**. Consequences, all documented
in code:
- The two post-solve keyframe-selection fractions (`match_frac_k`, `unique_frac_f`, `tracker.py:105-107`)
  are **not** captured.
- `trk_pre_ms` covers everything from `track()` entry to GN-loop entry (the match forward *plus* pointmap
  update, `get_points_poses` and the validity masks), not the match call alone. It is ~95 % the ViT
  forward.
- On the `skipped` path the solver never runs, so `trk_valid_opt`/`trk_match_frac` are `0`; the
  `trk_gn_exit == "skipped"` label is what disambiguates "not captured" from "genuinely zero".

---

## Contracts (referenced by chunk number below)

### C1 — `TrackStats` and the field tuples (`slam_track_stats.py`)

```python
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

GN_EXITS: tuple[str, ...] = (
    "",                 # track() did not run this frame (INIT / RELOC)
    "rel_error",        # check_convergence fired on the relative cost decrease
    "delta_norm",       # check_convergence fired on the step norm
    "max_iters",        # ran out the max_iters ceiling without converging
    "skipped",          # min_match_frac gate fired (tracker.py:68) -- no solve ran
    "cholesky",         # the solve raised (tracker.py:91's branch); we re-raise
    "error",            # track() raised outside the solve -- must never be silently a "skipped"
    "unclassified",     # check_convergence said True but the mirror could not attribute it: a BUG,
)                       # made loud rather than guessed at

TRACKER_PHASE_FIELDS: tuple[str, ...] = ("trk_pre_ms", "trk_solve_ms")
TRACKER_STATE_FIELDS: tuple[str, ...] = ("trk_gn_iters", "trk_gn_exit", "trk_valid_opt",
                                         "trk_match_frac", "trk_seeded")
```

**Invariant:** `tuple(f.name for f in dataclasses.fields(TrackStats)) == TRACKER_PHASE_FIELDS + TRACKER_STATE_FIELDS`.

**Closure invariant:** `trk_pre_ms + trk_solve_ms <= tracker_ms`. The remainder (write-back + keyframe
selection) is **derived** by the report, never stored. This is a **third** invariant, separate from the
four fields that close against `slam_ms` and the three that close against `track_ms`.

### C2 — the convergence-reason mirror

```python
def classify_convergence(step: int, old_cost: float, new_cost: float, delta_norm: float,
                         rel_thresh: float, delta_thresh: float) -> str:
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
```

### C3 — the subclass factory

```python
def make_instrumented_frame_tracker(base_cls: type) -> type:
    """Build the FrameTracker subclass that publishes a TrackStats per track() call.

    Factory-over-injected-base, cached per base class, exactly like
    slam_window.make_windowed_factor_graph -- so third_party/ stays pristine and this module
    imports and self-tests with no CUDA, no model and no vendored repo on the path.
    """
```

Returned class, `InstrumentedFrameTracker(base_cls)`:

| member | contract |
|---|---|
| `__init__(self, model, frames, device)` | calls `super().__init__(model, frames, device)`, then sets `self.last_track_stats = TRACK_STATS_ABSENT` and the private accumulators below |
| `last_track_stats: TrackStats` | public read surface; rebound as a whole object at the end of every `track()` |
| `track(self, frame)` | wraps `super().track(frame)`. **Does not reproduce its body.** Returns `super()`'s value unchanged |
| `opt_pose_ray_dist_sim3(self, Xf, Xk, T_WCf, T_WCk, Qk, valid)` | overridden; reproduces the vendored loop (`tracker.py:173-214`) verbatim plus counters. Same return: `(T_WCf, T_CkCf)` |
| `opt_pose_calib_sim3(self, *args, **kwargs)` | raises `NotImplementedError` naming `use_calib=False` — a quiet uninstrumented second path is exactly the hidden state this session hunts |

Private accumulators, all reset at the top of every `track()`:
`_trk_solve_entered: bool`, `_trk_t_track0: float`, `_trk_t_solve0: float`, `_trk_solve_ms: float`,
`_trk_gn_iters: int`, `_trk_gn_exit: str`, `_trk_valid_opt: int`, `_trk_match_frac: float`.

`trk_pre_ms` rule: `(_trk_t_solve0 - _trk_t_track0) * 1000` when the solver was entered; otherwise the
**whole** `track()` duration (the `skipped`/`error` paths never reach a solve, and reporting the full
call is honest — the exit column says which).

`trk_gn_exit` resolution at the end of `track()`:

| condition | value |
|---|---|
| solver set one (`rel_error`/`delta_norm`/`max_iters`/`unclassified`/`cholesky`) | that value |
| solver never entered, `track()` returned normally | `"skipped"` |
| solver never entered, `track()` raised | `"error"` |

### C4 — CUDA timing

```python
def _cuda_sync() -> None:
    """Wall-clock timing is meaningless across async CUDA launches, so each of the four timing
    boundaries syncs first. Adds NO GPU work and cannot change behaviour, but it makes tracker_ms
    marginally larger than on a pre-session-67 flight -- stated, never hidden (CLAUDE.md). The
    availability guard exists ONLY so the CPU-only self-test can construct this class."""
    if torch.cuda.is_available():
        torch.cuda.synchronize()
```

Four sync points: `track()` entry, `opt_pose_ray_dist_sim3` entry, `opt_pose_ray_dist_sim3` exit,
`track()` exit. Everything else is free: `check_convergence` already forces one device→host sync per GN
iteration and `self.solve` already `.item()`s the cost, so the exit-reason attribution costs **one**
extra `.item()` per `track()` call — at the break only, never inside the loop.

### C5 — CSV schema delta

Appended to `perception_worker.DIAG_PERF_FIELDS` **after `rec_frame`**, in this order:

```
"trk_pre_ms", "trk_solve_ms", "trk_gn_iters", "trk_gn_exit",
"trk_valid_opt", "trk_match_frac", "trk_seeded"
```

Making the header 40 columns. The frozen 9/17/20/33-column prefixes are unchanged.

---

## CHUNK 5 — surface it in the visualizer

**Module Objective.** Put the solver's state in front of the operator in flight, as `CLAUDE.md` rule 3
requires for any degraded-path state — and so the accumulator can be watched, not just post-mortemed.

**Target File(s).** `visualizer.py`.

**Strict Interfaces.**

1. `tracker_status_text(map_payload) -> str` — returns `""` when the payload carries no `trk_gn_exit`
   (an older flight or older perception) **or** when `trk_gn_exit == ""` (INIT/RELOC); otherwise a
   compact `gn=12/50` using the payload's iteration count and `max_iters` ceiling.
2. Render it in the **existing** top status strip, next to the session-64 backend segment.
   **`max_iters` renders red**; every other exit renders in the strip's normal colour — hitting the
   ceiling is the degraded state, converging is not.

**Prohibited.** Do not add a panel or change the dashboard layout — `CANVAS_W` / `CANVAS_H`
must not change. Do not render anything when the field is absent.

## Outcome (2026-09-07)

Chunks 1–5 all implemented and self-tested green: `slam_track_stats.py`, `perception_worker.py`'s CSV
schema + console segment + `_map_payload` forwarding, and `visualizer.py`'s `tracker_status_text` in the
top status strip (`gn=<iters>/50`, red only on `max_iters`). `TRACKER_GN_MAX_ITERS = 50` is a
display-only mirror of `third_party/MASt3R-SLAM/config/base.yaml`'s `tracker.max_iters` — that config
lives in the SLAM process and is never published on the bus, so it could not be read live through the
one-argument `tracker_status_text(map_payload)` signature this spec fixed.

The measurement itself — bench, fly with recording, frame-exact replay, `tracker_trend` on both CSVs —
is POST-RUN, operator work, not yet done. `plans/slam_report.html` §03's "next step" prediction is
therefore still a prediction, not replaced with a real table; that is the next session's first task once
a flight exists to read.
