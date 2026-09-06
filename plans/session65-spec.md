# Session 65 — Bound the global-optimisation window (Sonnet-ready spec)

> **Step 0 (operator/Claude, before any chunk runs):** copy this document to
> `plans/session65-spec.md` in the repo, so it is archived where every other session spec lives.

---

## Execution Guidelines (read before every chunk)

1. **Implement ONLY the chunk you were given.** Do not start the next one, do not "while I'm here"
   an unrelated file.
2. **Adhere strictly to the signatures in this document.** Names, argument order, types, return
   shapes and constant values are contracts. If a signature looks wrong, **stop and say so** — do not
   silently improve it.
3. **No architectural changes.** No new processes, threads, locks, queues, caches or config keys
   beyond those specified. Do not refactor neighbouring code.
4. **NO SILENT FALLBACKS (CLAUDE.md).** Invalid config raises. An impossible state raises. A phase
   that did not run reports a literal `0` / `0.0` / `""`, never a blank and never an invented number.
   No `try/except` that swallows.
5. **Do not edit `third_party/`.** The vendored MASt3R-SLAM stays pristine; all new behaviour lives in
   project-root modules and subclasses.
6. **Every chunk ends green.** Run the chunk's own self-test plus every suite already in the gate
   before reporting done. Report the actual command output; if something fails, say so.
7. **Comment the *why*, in this repo's voice.** Every non-obvious constant gets a sentence saying what
   evidence set it. Match the density of the surrounding code.
8. Windows: the venv interpreter is `venv\Scripts\python.exe`.

---

## Context (why this exists)

`plans/slam_report.html` ranks **"Bound the optimisation window"** the highest-value remaining lever
and the only one that changes the *growth curve* rather than a constant factor:

| keyframes | median `backend_ms` |
|---|---|
| 0–9 | 1 502 |
| 20–29 | 10 032 |
| 40–49 | 12 812 |
| 50–59 | **14 597** |

Root cause, confirmed by reading the code:

- `FactorGraph` is **append-only** — `add_factors` (`global_opt.py:89-96`) only ever `torch.cat`s onto
  the eight per-edge tensors. Nothing is ever pruned.
- `solve_GN_rays` (`global_opt.py:121`) calls `get_unique_kf_idx()` — *every keyframe ever seen in any
  edge* — and re-optimises all of them on every keyframe.
- `local_opt.window_size` (`third_party/MASt3R-SLAM/config/base.yaml`, `1e+6`) is read into
  `self.window_size` at `global_opt.py:26` and **never referenced again**. Dead code.

Three costs scale with keyframe count N, not just GN FLOPs:

1. `get_poses_points` stacks all N pointmaps (`X_canon`, ~512×288×3 fp32 ≈ 1.7 MB each) + confidences
   into fresh GPU tensors **every solve**.
2. `prep_two_way_edges` (`global_opt.py:104`) `torch.cat`s every per-edge tensor with itself,
   duplicating ~2 MB/edge of `idx_ii2jj`/`valid_match`/`Q` — hundreds of MB allocated and copied per
   solve at a few hundred edges. Pure memory bandwidth, invisible in a FLOP model.
3. The kernel: `ray_align_kernel<<<num_edges>>>` × `max_iters` (10, with `delta_norm: 1e-8`, so it
   effectively never converges early), then `SparseBlock A(num_poses - 1, 7)` and `A.solve()`.

Bounding the edge set attacks all three at once.

### Two findings that de-risk this

**The CUDA already remaps keyframe indices.** `gauss_newton_rays_cuda` (`gn_kernels.cu:1140`) computes
its own `get_unique_kf_idx(ii, jj)` and maps absolute ids to row positions via `searchsorted`
(`create_inds`, `gn_kernels.cu:166-170`). A **non-contiguous** keyframe set is handled correctly
provided the Python side gathers `Xs`/`T_WCs`/`Cs` in sorted-unique order over the *same* filtered
edges — exactly what `get_poses_points(unique_kf_idx)` already does. Today this is invisible because
edges always include the consecutive pair `(idx-1, idx)`, so the unique set is always `arange(N)`.
**Filtering edges is therefore safe with no index surgery.**

**The pin lands right by construction.** `num_fix = 1` (`gn_kernels.cu:1155`) fixes the
**lowest-indexed** keyframe in the solve. A window is a suffix `[n-W+1, n]`, so any out-of-window
keyframe pulled in by a loop-closure edge sorts *below* the whole window — the oldest anchor is pinned
automatically and the window is anchored to the existing global estimate rather than floating.

### One fact that lowers the stakes

The voxel map is integrated **incrementally, once per keyframe, at that keyframe's then-current pose**
(`map_store.py` has no re-integration path; `add_pose` deliberately does not dirty the raster). Backend
corrections to old keyframes never retroactively re-fuse the map, so "windowing degrades the map
because old keyframes stop being refined" is a weaker objection here than in a re-fusing system.

### Outcome this session

Config-gated bounded window with explicit telemetry, **defaulting off**, plus an offline bench over a
recorded flight answering the report's own question: *does `backend_ms` flatten against keyframe count
while trajectory consistency holds?* **No live flight** — that is the operator's step, per session 64.

---

## Design summary

**Window.** Per solve, derived from the graph itself: `n_new = max(ii.max(), jj.max())`,
`window_lo = n_new - W + 1`, in-window iff `k >= window_lo`. An edge is **active** iff **at least one**
endpoint is in-window. Both in-window → local chain, kept. One in-window → loop closure, kept, and its
far endpoint joins as an **anchor**. Neither → old history, **dropped** — that is where the unbounded
growth lives.

**Policies** (a knob, so the bench measures rather than assumes):

- `anchored` *(default)* — keep loop edges; after the solve write back **only in-window keyframes**.
  Anchors keep their stored poses, so the old map is never corrupted; the window absorbs what share of
  the loop correction it can and the edge stays in the graph, so it converges over successive solves.
  Anchors are the lowest sorted indices, so this is a contiguous slice.
- `free` — upstream-faithful: anchors move and are written back. Prices the difference.
- `strict` — drop every edge with an out-of-window endpoint. Pure sliding window, no loop closure. The
  pessimistic bound.

`anchor_drift` is measured under every policy. ~0 means `anchored` and `free` are the same thing and
the approximation cost nothing; large is the number that justifies the deferred CUDA work.

**Mode tri-state.** `OFF` (default, today's behaviour) / `SHADOW` (compute and report the mask, solve
the **full** graph — prices any `W` before cutting a single edge) / `ON` (apply it).

**RELOC is exempt.** `_relocalization` (`slam_engine.py:239-267`) also calls `solve_GN_rays()`, and a
successful reloc matches against candidates that are usually far out of window. Windowing that would
weaken the one mechanism that recovers tracking, to save time on a rare event. `force_full=True` there.

---

## Contract-first definitions

### `slam_window.WindowStats`

```python
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
```

Frozen and replaced by whole-object attribute rebind, never mutated in place: the backend thread
writes it and the frame path reads it, and an atomic rebind means **no lock** — deliberately, after
session 64 measured 787 s of a flight lost to taking a lock just to read numbers.

### `slam_window.EdgeSelection`

```python
@dataclass(frozen=True)
class EdgeSelection:
    mask: "torch.Tensor"        # (E,) bool — True = keep this edge
    window_lo: int              # -1 when unbounded
    n_anchors: int
    anchor_idx: "torch.Tensor"  # (n_anchors,) int64, ascending — the out-of-window keyframes kept
    active_kf: "torch.Tensor"   # (K,) int64, ascending — unique keyframes of the masked edges
```

**Invariant (the whole gauge argument rests on it):** `anchor_idx` is exactly `active_kf[:n_anchors]`.

### Module constants

```python
WINDOW_MODES: tuple[str, ...] = ("OFF", "SHADOW", "ON")
WINDOW_POLICIES: tuple[str, ...] = ("anchored", "free", "strict")
```

### `slam_engine.SLAM_WINDOW_FIELDS` and the seven `SlamResult` fields

```python
SLAM_WINDOW_FIELDS: tuple[str, ...] = (
    "backend_window_mode", "backend_window_kf", "backend_solve_kf",
    "backend_solve_edges", "backend_graph_edges", "backend_anchors",
    "backend_anchor_drift")
```

| `SlamResult` field | type | default | source |
|---|---|---|---|
| `backend_window_mode` | `str` | `"OFF"` | `stats.mode` |
| `backend_window_kf` | `int` | `0` | `stats.window_kf` |
| `backend_solve_kf` | `int` | `0` | `stats.solve_kf` |
| `backend_solve_edges` | `int` | `0` | `stats.solve_edges` |
| `backend_graph_edges` | `int` | `0` | `stats.graph_edges` |
| `backend_anchors` | `int` | `0` | `stats.anchors` |
| `backend_anchor_drift` | `float` | `0.0` | `stats.anchor_drift` |

**These are state, not durations.** They must not join `SLAM_PHASE_FIELDS`, `SLAM_TRACK_PHASE_FIELDS`,
`SLAM_BACKEND_FIELDS`, `PHASE_COLUMNS`, or either closure invariant — exactly how session 64 handled
`backend_mode`.

### Config keys (`config.yaml`, `perception:` block)

| key | type | default | meaning |
|---|---|---|---|
| `backend_window_mode` | str | `"OFF"` | `OFF` / `SHADOW` / `ON` |
| `backend_window_kf` | int | `0` | `W`; `<= 0` means unbounded |
| `backend_window_policy` | str | `"anchored"` | `anchored` / `free` / `strict` |

---

## CHUNK 1 — `slam_window.py`: the selection core

**Module Objective.** The pure, testable heart of the window: decide which edges a solve keeps, and
describe the result. No CUDA, no model, no keyframe store, **no import of the vendored SLAM repo**.

**Required Context/Dependencies.** `torch` only (plus stdlib `dataclasses`, `argparse`). This module
must import successfully when `third_party/MASt3R-SLAM` is *not* on `sys.path` — the self-test and any
later tooling need it cold.

**Target File(s).** `slam_window.py` *(new, project root)*.

**Strict Interfaces.**

1. Module docstring in this repo's voice: what the window is, why the append-only graph makes it
   necessary, the `create_inds` finding that makes edge filtering safe, and the `num_fix = 1` limit.
2. `WINDOW_MODES`, `WINDOW_POLICIES`, `WindowStats`, `EdgeSelection` exactly as specified above.
3. ```python
   def validate_window_config(mode: str, window_kf: int, policy: str) -> tuple[str, int, str]
   ```
   Returns the normalised triple. Raises `ValueError` naming the offending value *and the legal set*
   when `mode not in WINDOW_MODES`, `policy not in WINDOW_POLICIES`, or `window_kf` is not an `int`.
   A negative `window_kf` normalises to `0`. **`mode` and `policy` are case-sensitive** — do not
   `.upper()` a typo into silent acceptance.
4. ```python
   def select_active_edges(ii, jj, window_kf: int, policy: str = "anchored") -> EdgeSelection
   ```
   - Validates `policy`; raises `ValueError` otherwise.
   - `ii`/`jj` are `(E,)` int64 tensors on any device. All returned tensors live on `ii.device`.
   - **`E == 0`** → empty bool mask, `window_lo=-1`, `n_anchors=0`, empty `anchor_idx`, empty
     `active_kf`.
   - **`window_kf <= 0`** → all-`True` mask, `window_lo=-1`, `n_anchors=0`, empty `anchor_idx`,
     `active_kf = torch.unique(torch.cat([ii, jj]), sorted=True)`.
   - Otherwise: `n_new = int(torch.maximum(ii.max(), jj.max()))`,
     `window_lo = n_new - window_kf + 1`, `in_i = ii >= window_lo`, `in_j = jj >= window_lo`;
     `mask = (in_i & in_j)` for `policy == "strict"`, else `mask = (in_i | in_j)`.
   - `active_kf = torch.unique(torch.cat([ii[mask], jj[mask]]), sorted=True)`;
     `anchor_idx = active_kf[active_kf < window_lo]`; `n_anchors = int(anchor_idx.numel())`.
5. ```python
   def local_edge_indices(active_kf, ii, jj) -> tuple["torch.Tensor", "torch.Tensor"]
   ```
   `torch.searchsorted(active_kf, ii)`, same for `jj`. A Python mirror of the CUDA's `create_inds`
   (`gn_kernels.cu:166-170`), existing **only** so the self-test can assert agreement with the
   contract the kernel will apply. Docstring must say so.
6. `run_self_test() -> None` and a `__main__` with `--self-test`, in this repo's `check(label, cond)`
   style: prints each check, counts failures, `sys.exit(1)` on any failure.

**Prohibited.** No `import lietorch`, no `mast3r_slam` import, no CUDA requirement, no I/O, no config
file reading. Do not mutate `ii`/`jj`.

**Acceptance Tests** (`slam_window.py --self-test`, CPU int64 tensors throughout):

- `window_kf=0` and `window_kf=-5` → mask all `True`, `n_anchors == 0`, `window_lo == -1`. **The
  disabled path is provably identity.**
- `window_kf` larger than the whole graph → mask all `True`, `n_anchors == 0`.
- Consecutive chain `ii = [0..58]`, `jj = [1..59]`, `W=10` → surviving edges are exactly those
  touching `[50, 59]`; `active_kf` is contiguous; `n_anchors == 0`.
- Same chain plus loop edge `(3, 57)`, `W=10` → that edge is kept, `n_anchors == 1`,
  `anchor_idx == [3]`, and **`active_kf[0] == 3`** (assert directly — the upstream pin depends on it).
- Same chain plus loop edges `(3, 57)` and `(30, 55)`, `W=10` → `n_anchors == 2`,
  `anchor_idx == active_kf[:2] == [3, 30]`.
- `policy="strict"` on that graph → both loop edges dropped, `n_anchors == 0`, and the mask is a
  strict subset of the `anchored` mask.
- `policy="free"` returns the **same mask** as `anchored` (the policies differ only at write-back).
- **Kernel-contract test:** for a deliberately non-contiguous `active_kf` (e.g. `[3, 30, 50..59]`),
  `local_edge_indices(active_kf, ii_masked, jj_masked)` returns values in `[0, len(active_kf))` and
  round-trips: `active_kf[local_ii] == ii_masked`.
- `E == 0` → no crash, all counters zero.
- `validate_window_config` raises `ValueError` for `"on"`, `"ANCHORED"`, `"maybe"`, and a float
  `window_kf`; normalises `-3 → 0`; accepts each legal combination.
- Every returned tensor is on the same device as `ii`.

Run: `venv\Scripts\python.exe slam_window.py --self-test`

---

## CHUNK 4 — config + `perception_worker.py`

**Module Objective.** Expose the knobs to the operator and get the seven columns into the flight CSV
and the map payload.

**Required Context/Dependencies.** CHUNK 3. Existing anchors: the `backend_async` config block
(`config.yaml:33-47`), `Pipeline.__init__` (`perception_worker.py:122-129`), `DIAG_PERF_FIELDS`
(`:52-68`), the diag row construction (`:~402`), `_map_payload` (`:~429`).

**Target File(s).** `config.yaml`, `perception_worker.py`.

**Strict Interfaces.**

1. `config.yaml`, in `perception:` directly under the `backend_async` block, a comment block in that
   block's voice explaining: the append-only graph, the measured 1 502 → 14 597 ms climb, what each
   mode does, what each policy does, and that `OFF` is the default **pending a bench and a flight**,
   exactly as `backend_async` is. Then the three keys at their tabled defaults.
2. `Pipeline.__init__` reads them with `.get()` and the tabled defaults, passes them to
   `SlamEngine(...)` as keyword arguments alongside `backend_async`. **No second print** — the engine
   already prints the startup line.
3. `DIAG_PERF_FIELDS` gains the seven, appended **last**, under a session-65 comment. The first nine
   names and their order stay frozen; the report reads by column name, so old flights stay comparable.
4. The diag row gains `backend_window_mode=res.backend_window_mode, ...` for all seven.
5. `_map_payload` gains **four** only — `backend_window_mode`, `backend_window_kf`,
   `backend_solve_kf`, `backend_anchors` — with a comment saying the other three are CSV-only because
   nothing renders them.

**Prohibited.** Do not reorder or rename existing columns. Do not add a config key beyond the three.
Do not print a second backend/window line. Do not change `MAP_GRID` or the payload's existing keys.

**Acceptance Tests.** Extend `perception_worker.py`'s `run_self_test()`:

- `DIAG_PERF_FIELDS[:9]` is unchanged and in its frozen order.
- Every name in `slam_engine.SLAM_WINDOW_FIELDS` appears in `DIAG_PERF_FIELDS`, exactly once.
- `DIAG_PERF_FIELDS` has no duplicates overall.
- A synthetic full row written with `csv.DictWriter` round-trips all seven with the right values, and
  a row built from a default `SlamResult` writes literal `0`/`0.0`/`OFF` — **never a blank**.
- `_map_payload` on a default `SlamResult` carries the four keys and does **not** carry
  `backend_anchor_drift`.

Run: `venv\Scripts\python.exe perception_worker.py --self-test`

---

## CHUNK 5 — `perception_timing_report.py`

**Module Objective.** Make a flight's window behaviour readable from the CSV alone, without
misrepresenting non-durations as phase timings.

**Required Context/Dependencies.** CHUNK 4's columns. **Import neither `slam_engine` nor
`perception_worker`** — this tool stays pure stdlib so it runs with no venv and no torch. Re-declare
names locally, as the module already does for its other three mirrors. Model the code on
`backend_summary` (`:194-236`) and `_BACKEND_STATE_COLUMNS` (`:68-70`).

**Target File(s).** `perception_timing_report.py`.

**Strict Interfaces.**

1. `_WINDOW_STATE_COLUMNS: tuple[str, ...]` = the seven names, with a comment saying why they are
   deliberately **not** in `PHASE_COLUMNS` (one is a string, five are counts, and `anchor_drift` is a
   pose magnitude — `render_table`'s "median/p90 of milliseconds" would misrepresent all seven).
2. ```python
   def window_summary(rows) -> str
   ```
   - Any of `_WINDOW_STATE_COLUMNS` absent (including empty `rows`) → **exactly**
     `"window state: unavailable (file predates session 65)"`.
   - Otherwise one line reporting: distinct `backend_window_mode` values with row counts (plus
     `blank=n` when any are blank, as `backend_summary` does); the distinct `backend_window_kf`
     values; median/max of `backend_solve_kf`, `backend_solve_edges`, `backend_graph_edges`; median
     and max of `backend_anchors`; max `backend_anchor_drift`. Reuse `phase_stats` for every numeric
     column so blanks are counted and named, never coerced.
   - **Append a second line when any row is `SHADOW`:** `"  (SHADOW rows: solve_kf/solve_edges/anchors
     are WOULD-BE counts; the full graph was solved)"`. Without it the numbers are actively
     misleading.
   - **Append a third line when `backend_anchors` exceeds 1 on any row:**
     `"  *** anchors>1 on N row(s) (max M): only the OLDEST is pinned (num_fix=1) -- see the deferred
     CUDA num_fix work in plans/session65-spec.md ***"`. This line is the trigger condition for the
     deferred work; it must be findable from a report alone.
3. `report()` prints `window_summary(rows)` immediately **under** the existing `backend_summary` line.

**Prohibited.** Pure stdlib only. Do not add any of the seven to `PHASE_COLUMNS`. Do not change
`phase_closure`, `track_closure`, `backend_summary`, `PhaseStats`, `render_table`, or any existing
signature.

**Acceptance Tests.** Extend `run_self_test()`, building synthetic CSVs with `csv.DictWriter` in a
`tempfile.mkdtemp()` cleaned up in a `finally` (the module's existing pattern):

- A modern CSV with mixed `backend_window_mode` values names each mode with its count.
- A pre-session-65 CSV yields exactly the `"unavailable (file predates session 65)"` string, **and
  `report(path)` still prints both closure lines and `backend_summary` normally** — one missing
  feature must not suppress another.
- Blank `backend_anchors` cells are counted via `n_blank`, not zeroed.
- A CSV containing a `SHADOW` row emits the would-be caveat line; an all-`ON` CSV does not.
- A CSV with `backend_anchors` of `[0, 1, 3]` emits the `anchors>1` line naming `max 3`; one with
  `[0, 1, 1]` does not.
- `PHASE_COLUMNS` contains none of `_WINDOW_STATE_COLUMNS`.
- Every session-62/63/64 test still passes unchanged.

Run: `venv\Scripts\python.exe perception_timing_report.py --self-test`
**and** `python perception_timing_report.py --self-test` (bare system interpreter, no venv, no torch).

---

## CHUNK 6 — `visualizer.py`

**Module Objective.** Put the window state in front of the operator in flight, per CLAUDE.md rule 3.

**Required Context/Dependencies.** CHUNK 4's `_map_payload` keys. `backend_status_text`
(`visualizer.py:649-664`) and `render_status` (`:667`).

**Target File(s).** `visualizer.py`.

**Strict Interfaces.**

Extend **`backend_status_text`** — do not add a second function and do not add a panel.

1. The `FAILED` short-circuit stays first and unchanged (`"bk=FAILED"`).
2. After the existing `clob` suffix, append a window suffix:
   - `backend_window_mode` absent, or `"OFF"` → **append nothing** (older flights and the default
     render exactly as they do today).
   - `"ON"` → `f" w{window_kf}k{solve_kf}"`, plus `f"a{anchors}"` when `anchors` is non-zero.
   - `"SHADOW"` → `f" w?{window_kf}"` — the `?` marks *measured, not applied*. Comment it.

**Prohibited.** `CANVAS_W`/`CANVAS_H` (`:73-74`) must not change. No new colours — this is
informational, and a non-`OFF` window is an operator-chosen state, not an alarm. No layout change.

**Acceptance Tests.** Extend `run_self_test()` in its `case(...)` style:

- A payload with no `backend_window_mode` returns the **byte-identical** string session 64 produced.
- `mode="OFF"` also returns that identical string.
- `ON`, `W=10`, `solve_kf=12`, `anchors=0` → contains `"w10k12"` and **no** `"a"` suffix.
- `ON` with `anchors=2` → contains `"a2"`.
- `SHADOW`, `W=10` → contains `"w?10"` and **not** `"w10k"`.
- `FAILED` still returns exactly `"bk=FAILED"` even when window keys are present.
- Rendering an `ON` payload puts **no** red in the status strip (crop to the strip rows, as the
  session-62 recovery-row test does).

Run: `venv\Scripts\python.exe visualizer.py --self-test`

---

## CHUNK 7 — `backend_window_bench.py`

**Module Objective.** Turn "trajectory consistency holds" into a number, by comparing two offline
`_livemap.npz` exports.

**Required Context/Dependencies.** `map_store.MapStore.save_npz` (`map_store.py:375-379`) writes
`centers`, `colors`, `trajectory`, `voxel_size`, `tracking_mode`. `run_offline_video`
(`perception_worker.py:1074`) writes `<stem>_livemap.npz` on exit. Same video + `--stride` +
`--max-frames` → same frame count and order, so trajectories are directly comparable **with no Sim3
alignment**.

**Target File(s).** `backend_window_bench.py` *(new, project root)*.

**Strict Interfaces.** NumPy + stdlib only — no torch, no project imports.

```python
def load_run(npz_path) -> dict
    # {"trajectory": (N,3) float32, "centers": (M,3) float32, "voxel_size": float, "path": str}
    # Raises FileNotFoundError, or KeyError naming the missing key and the file.

def trajectory_rmse(a, b) -> tuple[float, int]
    # (rmse over per-pose Euclidean distance, n_compared).
    # Length mismatch -> ValueError naming BOTH lengths. Never truncate silently.

def occupancy_overlap(a_centers, b_centers, voxel_size: float) -> tuple[float, int, int]
    # Voxelise both to integer keys at voxel_size, return (IoU, n_a, n_b).
    # voxel_size <= 0 -> ValueError.

def compare(baseline_npz, candidate_npz) -> str
    # Multi-line report: both paths, n poses, trajectory RMSE, max per-pose deviation,
    # voxel counts and IoU. Mismatched voxel_size between the two runs -> ValueError.

def run_self_test() -> None
```

`main()`: `argparse` with positional `baseline`, positional `candidate`, and `--self-test`.

**Prohibited.** No plotting, no torch, no project imports, no writing to `OUTPUT/`. Do not
Sim3-align — say in the docstring that it is unnecessary *and why* (identical input stream, and the
oldest keyframe is pinned in both runs).

**Acceptance Tests** (`backend_window_bench.py --self-test`, synthetic `.npz` in a `tempfile.mkdtemp()`
cleaned up in a `finally`):

- A file compared with itself → RMSE exactly `0.0`, IoU exactly `1.0`.
- A trajectory offset by a known constant → RMSE equals that constant within `1e-6`.
- Differing lengths → `ValueError` whose message contains both lengths.
- Disjoint voxel sets → IoU `0.0`; one set a strict subset of the other → the exact expected IoU.
- Mismatched `voxel_size` → `ValueError`.
- A missing `trajectory` key → `KeyError` naming the key and the file.

Run: `venv\Scripts\python.exe backend_window_bench.py --self-test`

---

## Deferred (documented, NOT built): multi-anchor pinning via `num_fix`

Recorded at the operator's explicit request so it survives a context clear. **Do not implement this
session.** Fold it into `PROGRESS.md` and `STATE.md` as a named deferred candidate.

**The gap.** `const int num_fix = 1;` (`gn_kernels.cu:1155`) fixes exactly one pose. With several
out-of-window loop-closure anchors in one solve, only the oldest is held; the rest are free to move to
satisfy their own loop edges. That is backwards — the loop constraint should correct the *recent*
trajectory, not drag old keyframes. The `anchored` policy contains the damage (their motion is
discarded at write-back) but wastes part of the constraint.

**Why the fix is unusually clean here.** Anchors are always the *lowest* indices in the sorted unique
set, so "fix the first `k`" is exactly "fix all anchors" — **no reordering, no permutation buffer, no
change to `create_inds`**, which already subtracts `pin` and sizes the system as
`num_poses - num_fix`. The work:

1. `gn_kernels.cu` — `num_fix` becomes a parameter of `gauss_newton_rays_cuda` (and, for symmetry,
   `gauss_newton_calib_cuda`) instead of a local constant. `SparseBlock A(num_poses - num_fix, ...)`
   and `pose_retr_kernel(..., num_fix)` already consume it correctly.
2. `include/gn.h` + `src/gn.cpp` — thread the argument through the binding (`gn.cpp:28`, `:50`, and
   the `m.def` at `:118`).
3. `slam_window.py` — pass `max(pin, n_anchors)`.
4. Rebuild: `build_mast3r_slam.bat` (note `build_mast3r_slam_step23.bat` and
   `lietorch_windows_const_fix.patch` — this toolchain has been fought before).

**Trigger to build it.** A bench or flight where `backend_anchors` is regularly `> 1` **and**
`backend_anchor_drift` is non-trivial against the window's own motion scale. CHUNK 5's third summary
line exists precisely to make this visible from a report alone. If anchors are almost always 0–1, this
is not worth the rebuild risk and `anchored` is sufficient.

**What this still is not.** True marginalisation. Fixing old poses is not a Schur complement with a
prior on the marginalised block — it over-constrains (treats the old estimate as infinitely certain)
rather than carrying its uncertainty forward. For a monocular system with drifting scale that is the
*conservative* error, and it is the standard cheap first step. Proper marginalisation stays open, and
is question 3 in `plans/slam_report.html` section 08.

---

## POST-RUN (operator, not Sonnet)

Harness that already exists — no new runner:
```
venv\Scripts\python.exe perception_worker.py --video <mp4> --no-display --log --max-frames 400
venv\Scripts\python.exe perception_timing_report.py
```
~400 frames of a long recording, so the run reaches the 50–59 keyframe band where the baseline reads
14 597 ms. Candidate: `../XLAB/OUTPUT/flight_20260621_120829.mp4`.

1. **Noise floor first.** Run the baseline (`OFF`) **twice**. CUDA reductions are not bit-deterministic
   — establish how far two identical runs diverge before reading any windowed delta. Skipping this
   makes every later number unreadable.
2. **`SHADOW`, one run.** Prices the cut without making it: how many edges a given `W` would drop, and
   — the number that decides the deferred CUDA work — how often `backend_anchors > 1`.
3. **`ON`, `W ∈ {30, 20, 10}`, policy `anchored`.** Then one run at the most promising `W` with `free`
   and one with `strict`, to price the anchor policy.

Accept criteria:

- **Growth flattens.** `backend_ms` by keyframe bucket stops climbing with keyframe count. That is the
  whole point; a constant-factor win is a partial result and must be reported as such.
- **Trajectory consistency holds.** `backend_window_bench.py` RMSE and occupancy IoU judged **against
  the step-1 noise floor**, never against zero.
- **Closure invariants unmoved.** `phase_closure` and `track_closure` residuals stay near 0 ms — the
  session-64 lock-contention bug was caught exactly this way, and time appearing outside every stamp
  means the mask is costing something unaccounted for.
- **Nothing regressed with the feature off.** One `OFF` run after the change matches the pre-change
  baseline within the noise floor.

Report as a table of `W` → median `backend_ms` at 50–59 keyframes → trajectory RMSE → anchors, so the
default is chosen on numbers. **Ship the default as `OFF` regardless**, per the `backend_async`
precedent: the operator flies it before it becomes the default.
