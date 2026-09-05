# SESSION 63 (Phase 1) — Sonnet-Ready Implementation Specification
# Cut the map-publish cost, and split `track_ms` so the next wall has a name

Run with: `python sonnet_runner.py --plan C:\Users\owner\.claude\plans\hey-please-read-state-md-vast-lemon.md`

On approval, archive a copy as `plans/session63-spec.md` (repo convention).

**Scope note, deliberate:** this spec covers **Phase 1 only**. Phase 2 — moving `_run_backend()` off
the frame-critical path into a thread, the 55-83 % win — is **not chunked here** and must not be
started. The operator's sequencing decision is to fly Phase 1 first so a bad flight has one suspect;
and a concurrency bug on shared CUDA structures is precisely the class of defect a self-test suite
does not catch, so Phase 2 earns its own spec written *after* Phase 1's flight says what `track_ms`
actually is. See "PHASE 2 IS NOT IN THIS SPEC" at the bottom.

---

## EXECUTION GUIDELINES (read before every chunk)

1. **Implement ONLY the current chunk.** Do not start, preview, refactor or "improve" any other
   chunk. Earlier chunks are already applied on disk — do not re-verify or re-implement them.
2. **Signatures are contracts.** Use the exact names, parameter names, parameter order, defaults,
   types and return shapes given in `SHARED CONTRACTS`. No renames, no extra parameters, no changed
   return shapes, no "while I was here" additions. If a contract looks wrong, implement it as
   written and say so in your report.
3. **No architectural changes.** Chunks 1-2 are a pure performance rewrite of storage that must
   return **identical values**; chunks 3-5 are measurement only. Do **not** introduce threads,
   queues, locks, processes, ports or new dependencies anywhere in this session. Do not change what
   is published or when, and do not touch `_run_backend`, the tracker, or any CUDA call's behaviour.
4. **Anchors are source strings, not line numbers.** `perception_worker.py` is ~1.5k lines and
   `autopilot.py` ~11k — use `Grep` to locate each anchor string, then `Edit`.
5. **NO SILENT FALLBACKS** (`CLAUDE.md`). The trap in this session is a **cache that lies**: a stale
   `topdown_summary` returned after the map changed is a silent wrong answer, which is worse than a
   slow right one. The invalidation rule in **C2** is mandatory and must be asserted, not assumed.
   Likewise a phase that did not run reports a literal `0.0`, never `None` and never a guess.
6. **IMAGE INTEGRITY** (`CLAUDE.md`). No frame is resized, cropped or re-encoded anywhere in this
   session. If a chunk seems to require it, stop and report instead.
7. **NO MANUAL-FLIGHT DATA LEAKAGE** (`CLAUDE.md`). Every constant here is a CSV column name, an
   array capacity, or a formatting width. Nothing derived from a specific flight or room, and nothing
   in this session feeds the autopilot at all.
8. **Comment in the surrounding style.** Dense "why", not "what", tagged `Session 63:` and citing the
   evidence in `MISSION CONTEXT`.
9. **Do not commit, stage, stash, or otherwise mutate git state.**
10. **The gate runs ten suites under the project venv after every chunk** — `autopilot.py`,
    `frontier_planner.py`, `visual_recovery.py`, `flight_replay.py`, `ground_grid.py`, `map_store.py`,
    `salvage_flight.py`, `perception_worker.py`, `visualizer.py`, `perception_timing_report.py`.
    Breaking any one halts the run. **All ten were verified green at HEAD (`d22d9e7`) immediately
    before this spec was written.** If you hit a red suite you did not cause, say so and stop — do not
    fix it inside a chunk.
11. **Finish by running the self-test commands the chunk names**, and report the full PASS/FAIL list
    verbatim.
12. **Every chunk must change at least one file.** An empty diff is treated as a failed chunk.

---

## MISSION CONTEXT (why this work exists)

Session 62 instrumented the perception loop and settled where `slam_ms` goes. Flight
`20260905_184034` (`OUTPUT/diag/20260905_184036_perception.csv`, 339 rows):

| flight min | median `slam_ms` | `backend_ms` | `track_ms` | `map_pub_ms` |
|---|---|---|---|---|
| 0–5   | 425 ms | 0 | 417 | 67 |
| 5–10  | 2 279 | 0 | 2 108 | 202 |
| 20–25 | **23 816** | **19 851 (83 %)** | 4 115 | 242 |
| 25–30 | 22 094 | 18 099 (82 %) | 3 568 | 242 |

A **56× degradation**, overwhelmingly `_run_backend()`. The prior flight's whole-loop attribution
agreed: `backend_ms` 55.4 % of 2 010 s, `track_ms` 29.9 %, `map_pub_ms` 4.6 %, `integrate_ms` 1.9 %.
That measurement is also what *refuted* the original proposal to thread map integration — it targets
3.3 % of the flight.

**This session takes the two items that are safe to fix now.**

**(a) `map_pub_ms` is the CPU cost that actually scales with map size** — 4 → 70 → 138 → 231 →
**1 238 ms** across voxel-count buckets up to 386 k. `Pipeline._map_payload`
(`perception_worker.py:383`) calls `MapStore.topdown_summary(grid=MAP_GRID)` (`map_store.py:247`) on
the `MAP_PUB_INTERVAL = 0.5 s` timer — **at ≥2 Hz whether or not the map changed**. Per call it does
`np.asarray(self._keys, dtype=np.int64)` (`map_store.py:272`) where `_keys` is a **Python list of
tuples** (`:53`), converting all 386 k entries every time; then two `np.percentile` passes, an
`np.unique(..., return_inverse=True)`, and three `np.bincount`. Occupied cells only change inside
`integrate()` (`:74`), which runs on **keyframes only** — roughly one frame in six. So most of that
work recomputes an identical answer.

**(b) `track_ms` is the next wall and is currently opaque.** It grew 417 → 4 115 ms on the last
flight and covers three very different things in one number: frame construction, MASt3R inference,
and the tracker. Once Phase 2 removes `backend_ms` from the frame path, `track_ms` *is* the budget —
and there is no point guessing at it twice in one project.

**Not in scope, and explicitly not the problem:** the FALLBACK/recovery logic. Session 62b's
reordering works — it recovered a 6-minute plan-stale, and on a 10.5-minute one handed SLAM good
viewpoints that SLAM simply never solved. Parallax-push work is parked at step 2 (watch-only) by
operator decision.

---

## SHARED CONTRACTS

Read before every chunk.

### C1 — `MapStore` row storage (CHUNK 1)

`_keys` changes from a Python list of tuples to a preallocated `(cap, 3) int64` array, and the live
row count moves to an explicit counter. **The class's public behaviour does not change at all.**

```python
# in MapStore.__init__, REPLACING `self._keys: list[tuple] = []`
self._keys = np.zeros((0, 3), np.int64)   # row -> (ix,iy,iz), preallocated to self._cap
self._n: int = 0                          # live rows; NOT len(self._keys), which is capacity
```

**Invariant, asserted by the tests:** `self._n == len(self._row_of) <= self._cap`, and
`self._keys.shape == (self._cap, 3)`.

Every existing `len(self._keys)` becomes `self._n`, and every
`np.asarray(self._keys, dtype=np.int64)` becomes `self._keys[:self._n]` (no conversion, no copy).
Call sites, all in `map_store.py`: `_grow` (`:64`), `integrate` (`:112`, `:114`), `__len__` (`:126`),
`occupied` (`:130`, `:133`), `stats` (`:232`), `topdown_summary` (`:269`, `:272`).

`_grow` grows `_keys` with the same amortized doubling it already applies to `_count`/`_color_sum`:

```python
k = np.zeros((new_cap, 3), np.int64)
k[: self._cap] = self._keys
self._keys = k
```

In `integrate`'s per-voxel loop, `row = len(self._keys); self._keys.append(key)` becomes
`row = self._n; self._keys[row] = uniq[i]; self._n += 1`. `self._row_of` stays a dict keyed by the
same `(int, int, int)` tuple — **do not** change the key type.

### C2 — `topdown_summary` cache (CHUNK 2)

```python
# in MapStore.__init__
self._td_cache: dict | None = None    # cached cell raster + bounds from the last recompute
self._td_key: tuple | None = None     # (grid, pad, min_count) the cache was built for
self._td_dirty: bool = True           # set True by ANY mutation of counts/colors/keys
```

**Invalidation rule (mandatory, and the thing CHUNK 2's tests must prove):** `self._td_dirty = True`
at the **end of `integrate()`**, on every path that mutated the grid. `add_pose()` must **NOT** set it
— the trajectory is recomputed on every call regardless (see below).

A call to `topdown_summary(grid, pad, min_count)` reuses the cache **only** when
`self._td_dirty is False AND self._td_key == (grid, pad, min_count)`; otherwise it recomputes in full
and refreshes both. Cached fields — everything derived from the voxel grid:

| cached | why |
|---|---|
| `cells_u`, `cells_v`, `cells_rgb` | the raster itself |
| `bounds`, `span_world`, `n_voxels_kept` | derived from the same percentile pass |
| `x0`, `z0`, `scale` | needed to re-project the trajectory against the SAME frame |

**`traj_u` / `traj_v` are recomputed on EVERY call**, cached or not, using the cached `x0/z0/scale`
and the existing `to_cell` mapping — the trajectory grows every frame while the cells do not, and a
cached trajectory would freeze the drone's path on screen.

The returned dict must be **identical in content** to today's for the same inputs, including dtypes
(`cells_u/cells_v/traj_u/traj_v` → `np.int32`, `cells_rgb` → `np.uint8`) and the `TRAJ_MAX = 1500`
stride. Return copies of the cached arrays, never the cached objects themselves, so a caller cannot
mutate the cache.

### C3 — `SlamResult` tracking sub-phases (CHUNK 3)

Three fields **appended** to the existing dataclass, exactly mirroring the session-62 pattern already
in `slam_engine.py:66-72`. All `float`, in **milliseconds**, **always present**, default `0.0`. A
phase that did not run this frame is `0.0` — never `None`.

```python
@dataclass
class SlamResult:
    # ... every existing field UNCHANGED, in its existing order, ending with kf_download_ms ...
    # Session 63 — the split INSIDE track_ms. Once the backend leaves the frame path (phase 2),
    # this is the whole budget; measure it before optimising it, not after.
    frame_ms: float = 0.0      # self._create_frame(...) — runs on EVERY frame
    infer_ms: float = 0.0      # _mast3r_inference_mono — INIT and RELOC branches only
    tracker_ms: float = 0.0    # self.tracker.track(frame) — TRACKING branch only
```

Module-level constant, immediately after the existing `SLAM_PHASE_FIELDS` line
(`slam_engine.py:78`), which is **left unchanged**:

```python
# Session 63: the sub-split INSIDE track_ms, in pipeline order. Kept SEPARATE from
# SLAM_PHASE_FIELDS because those four close against slam_ms and these three close against
# track_ms — two different invariants, and merging them would break both.
SLAM_TRACK_PHASE_FIELDS: tuple[str, ...] = ("frame_ms", "infer_ms", "tracker_ms")
```

**Closure invariant (asserted by the report in CHUNK 5):**
`frame_ms + infer_ms + tracker_ms  ≈  track_ms`, residual within a few ms.

### C4 — `perception_worker` CSV schema extension (CHUNK 4)

`DIAG_PERF_FIELDS` (`perception_worker.py:52`) is extended by **appending only**. The existing
seventeen names and their order are frozen — files written before this session have exactly that
header, and the report reads by column NAME so old and new flights stay comparable in one tool.

```python
DIAG_PERF_FIELDS: tuple[str, ...] = (
    # ... all seventeen existing names UNCHANGED and in order ...
    "integrate_ms", "map_pub_ms", "plan_ms", "publish_ms",
    # --- Session 63: the sub-split inside track_ms (slam_engine.SLAM_TRACK_PHASE_FIELDS) ---
    "frame_ms", "infer_ms", "tracker_ms",
)
```

### C5 — `perception_timing_report` extension (CHUNK 5)

```python
PHASE_COLUMNS: tuple[str, ...] = (
    "slam_ms", "track_ms", "backend_ms", "pose_ms", "kf_download_ms",
    "integrate_ms", "map_pub_ms", "plan_ms", "publish_ms",
    "frame_ms", "infer_ms", "tracker_ms")          # session 63, appended

_TRACK_PHASE_FIELDS: tuple[str, ...] = ("frame_ms", "infer_ms", "tracker_ms")

def track_closure(rows: list[dict[str, str]]) -> PhaseStats: ...
```

`track_closure` mirrors the existing `phase_closure` (`perception_timing_report.py:124`) exactly:
per-row residual `track_ms - (frame_ms + infer_ms + tracker_ms)`, returns a `PhaseStats` with
`column="track_closure_residual_ms"`, rows missing any component counted in `n_blank`, and
`KeyError` if `track_ms` or any of `_TRACK_PHASE_FIELDS` is absent from the header. `report()` prints
its line directly under the existing SLAM-closure line, and — same as that one — prints an explicit
"unavailable (missing phase columns…)" line instead when the file predates this session. The tool
stays **pure stdlib**; it must not import `slam_engine` or `perception_worker`.

---

## CHUNK 1 — `MapStore` keys as a numpy array

**Module Objective.** Remove the per-call Python-list→numpy conversion from every readout path, with
zero change to any returned value.

**Required Context/Dependencies.** None (first chunk). Contract **C1**.

**Target File(s).** `map_store.py`

**Strict Interfaces.**

1. Apply **C1** exactly: `self._keys` becomes the preallocated `(cap, 3) int64` array, `self._n` is
   added, `_grow` grows `_keys` alongside `_count`/`_color_sum`.
2. Update all nine call sites listed in **C1**. `occupied`, `stats` and `topdown_summary` read
   `self._keys[:self._n]` directly — **delete** the `np.asarray(...)` calls rather than pointing them
   at the new array.
3. `__len__` returns `self._n`.

**Prohibited.** Do not change `_row_of`'s key type, the voxelization maths, `integrate`'s return
value, or any public method signature. Do not add caching (that is CHUNK 2). Do not vectorize
`integrate`'s per-voxel loop — a correctness-neutral rewrite of that loop is a separate decision and
is **not** in this session.

**Acceptance Tests.** Extend `map_store.py`'s existing self-test in its established style:

- **Invariant:** after several `integrate()` calls, `store._n == len(store._row_of) <= store._cap`
  and `store._keys.shape == (store._cap, 3)`.
- **Growth:** integrating enough distinct voxels to cross the doubling boundary more than once leaves
  every previously-stored key byte-identical (compare `store._keys[:n]` before and after a `_grow`).
- **Equivalence, the load-bearing one:** build a store from a fixed seeded point cloud, and assert
  `occupied()`, `stats()` and `topdown_summary()` return values equal to a hard-coded expectation
  captured in the test — specifically that `len(store) == store._n`, that `occupied()[0]` is sorted
  identically to the row order, and that `stats()["n_voxels"] == store._n`.
- **Empty store:** `len(store) == 0`, `occupied()` returns the `(0,3)` pair, `topdown_summary()`
  returns the zero-filled dict, none of them raising.

Run: `venv\Scripts\python.exe map_store.py --self-test`

---

## CHUNK 2 — `topdown_summary` cache

**Module Objective.** Stop recomputing an identical raster at ≥2 Hz, while making a stale answer
structurally impossible.

**Required Context/Dependencies.** CHUNK 1's `self._keys` / `self._n` (already applied on disk).
Contract **C2**.

**Target File(s).** `map_store.py`

**Strict Interfaces.**

1. Add the three cache fields from **C2** to `__init__`.
2. Set `self._td_dirty = True` at the **end of `integrate()`**, on every path that mutated the grid.
   Add a comment stating that `add_pose` deliberately does not set it, and why.
3. In `topdown_summary`, reuse the cache only under the exact condition in **C2**; otherwise
   recompute in full and refresh `_td_cache`/`_td_key`, clearing `_td_dirty`.
4. Recompute `traj_u`/`traj_v` on **every** call from the cached `x0`/`z0`/`scale`, including the
   `TRAJ_MAX = 1500` stride.
5. Return copies of cached arrays, never the cached objects.

**Prohibited.** Do not change the returned dict's keys, dtypes, ordering or values. Do not cache the
trajectory. Do not invalidate on `add_pose`. Do not add a time-based expiry — the dirty flag is the
only correct signal, and a timer would reintroduce exactly the staleness this must prevent.

**Acceptance Tests.** Extend `map_store.py`'s self-test:

- **Identical output:** for a seeded store, the summary returned on a **cached** call equals the one
  from the preceding uncached call, key for key, including dtypes — compare with
  `np.array_equal` per array field and `==` for the scalars.
- **Invalidation is real (the no-silent-fallback case):** take a summary, `integrate()` new points
  that add voxels, take another — `n_voxels_kept` and `cells_u` MUST differ. Assert `_td_dirty` was
  `True` immediately after the integrate.
- **Trajectory is never cached:** take a summary, call `add_pose()` several times, take another —
  `traj_u` MUST grow while `cells_u` stays byte-identical, and `_td_dirty` must still be `False`.
- **Cache key covers the parameters:** `topdown_summary(grid=100)` then `topdown_summary(grid=200)`
  must return rasters of the corresponding sizes, not the first one twice.
- **Returned arrays are copies:** mutate a returned `cells_u` in place, take another cached summary,
  and assert the second is unaffected.
- **It is actually faster:** build ~200 k voxels, time one uncached call and ten cached calls; assert
  the mean cached call is at least 5× faster. Use `time.perf_counter`; keep the voxel count modest
  enough that the suite stays quick.

Run: `venv\Scripts\python.exe map_store.py --self-test`

---

## CHUNK 3 — split `track_ms` inside `SlamEngine.process`

**Module Objective.** Name the three things `track_ms` currently hides, so the budget after Phase 2
is measured rather than guessed.

**Required Context/Dependencies.** Contract **C3**. Follows the session-62 pattern already present in
`slam_engine.py` (`SlamResult` phase fields, `SLAM_PHASE_FIELDS`, the `_t0.._t_kf` stamps).

**Target File(s).** `slam_engine.py`

**Strict Interfaces.**

1. Add the three fields and `SLAM_TRACK_PHASE_FIELDS` exactly as in **C3**. Append the fields; do not
   reorder or retype any existing field, and leave `SLAM_PHASE_FIELDS` untouched.
2. In `process()`, take `time.perf_counter()` stamps around the three regions. Anchors, in order:
   - Around `frame = self._create_frame(i, rgb_float01, T_WC, img_size=512, device=self.device)` —
     stamp immediately before and immediately after; the difference is `frame_ms`.
   - In the `Mode.INIT` branch and again in the `Mode.RELOC` branch, around
     `X, C = self._mast3r_inference_mono(self.model, frame)` — the difference is `infer_ms`.
     Only one of these branches can run per frame, so a single local accumulates it.
   - In the `Mode.TRACKING` branch, around
     `add_new_kf, _, try_reloc = self.tracker.track(frame)` — the difference is `tracker_ms`.
3. Initialise all three locals to `0.0` before the mode branch so an unentered branch reports a
   literal `0.0`, matching the session-62 convention documented at the existing
   `track_ms = (_t_track - _t0) * 1000.0` computation.
4. Pass all three into the `SlamResult(...)` call as keyword arguments, after `kf_download_ms=`.

**Prohibited.** Do not change control flow, do not add a `try`/`except`, do not touch
`_run_backend`, `_relocalization`, the tracker or any CUDA call. Do not add a thread. Do not alter
the existing four phase stamps or `track_ms`'s own computation.

**Acceptance Tests.** Add `_self_test_slam_track_phase_fields()` to `perception_worker.py` (it
already imports `slam_engine` at module level, and `SlamResult` is a plain dataclass needing no
CUDA), wired into `run_self_test` in the established `ok = ...; print PASS/FAIL; assert ok` style:

- `slam_engine.SLAM_TRACK_PHASE_FIELDS == ("frame_ms", "infer_ms", "tracker_ms")`.
- `slam_engine.SLAM_PHASE_FIELDS` is **unchanged** — still exactly the session-62 four.
- The two tuples are disjoint (`set(...) & set(...) == set()`).
- A `SlamResult` built with only the pre-session-63 required arguments has all three new fields
  present, `== 0.0`, and of type `float`.
- Constructing with `frame_ms=3.5, infer_ms=210.0, tracker_ms=48.25` round-trips those exact values.

Run: `venv\Scripts\python.exe perception_worker.py --self-test`

---

## CHUNK 4 — carry the three columns into the CSV and the console

**Module Objective.** Write the new phases per frame and make them visible live, reusing the
session-62 plumbing rather than adding a parallel path.

**Required Context/Dependencies.** CHUNK 3's `SlamResult` fields and
`slam_engine.SLAM_TRACK_PHASE_FIELDS` (already applied on disk). Contracts **C3**, **C4**.

**Target File(s).** `perception_worker.py`

**Strict Interfaces.**

1. Extend `DIAG_PERF_FIELDS` exactly as in **C4** — append only.
2. Extend the existing `self.diag_perf.row(...)` call at the end of `Pipeline.step()` with three more
   keyword arguments, each `round(..., 1)`: `frame_ms=round(res.frame_ms, 1)`,
   `infer_ms=round(res.infer_ms, 1)`, `tracker_ms=round(res.tracker_ms, 1)`. Keep every existing
   keyword argument unchanged.
3. Add `"frame"`, `"infer"`, `"tracker"` to the `self._last_phase_ms` dict (the session-62 sticky
   console dict, contract C3 of `plans/session62-spec.md`), initialised `0.0`, updated stickily on
   the same rule: write a key only when its phase actually ran this frame (`frame` every frame;
   `infer` only when `res.infer_ms > 0.0`; `tracker` only when `res.tracker_ms > 0.0`).
4. Extend the 1 Hz console line by inserting one new segment immediately after the existing
   `[trk … bk … dl …]` segment:
   ```python
   f"(frm {p['frame']:.0f} inf {p['infer']:.0f} trk2 {p['tracker']:.0f}) | "
   ```
   where `p = self._last_phase_ms`. Keep every existing segment, in its existing order.

**Prohibited.** Do not reorder or rename any existing CSV column. Do not move the `diag_perf.row`
call again. Do not log `_last_phase_ms` to the CSV or publish it on any topic — it is sticky and
therefore not an honest per-frame record.

**Acceptance Tests.** Extend `_self_test_phase_timing(cfg)` (do not add a new self-test function):

- `DIAG_PERF_FIELDS[:17]` is unchanged from the session-62 seventeen, in order — the frozen-prefix
  guarantee that keeps every earlier flight's CSV readable.
- Every name in `slam_engine.SLAM_TRACK_PHASE_FIELDS` appears in `DIAG_PERF_FIELDS`.
- `len(set(DIAG_PERF_FIELDS)) == len(DIAG_PERF_FIELDS)` — no duplicated column name.
- A `DiagLog` round-trip in a `tempfile.mkdtemp()` directory (cleaned up in a `finally`) with one
  fully-populated row yields a header equal to `list(DIAG_PERF_FIELDS)` and the exact values written.
- A row written with the three new keywords **omitted** yields blank (`""`) cells for them, not
  `"0.0"` — a missing field must stay visibly missing.
- The sticky dict has the three new keys; applying the update rule with `res.infer_ms == 0.0` leaves
  a previously-set `infer` value **unchanged**, and the rendered console segment contains `"frm "`,
  `"inf "` and `"trk2 "`.

Run: `venv\Scripts\python.exe perception_worker.py --self-test`

---

## CHUNK 5 — report the sub-split and its closure

**Module Objective.** Make the new columns first-class in the analysis tool, with their own closure
check, while keeping every pre-session-63 CSV readable.

**Required Context/Dependencies.** CHUNK 4's `DIAG_PERF_FIELDS` and CHUNK 3's
`SLAM_TRACK_PHASE_FIELDS` — **import neither**. `slam_engine` pulls in `torch` and `lietorch`; this
tool must stay pure stdlib so it runs outside the venv on any machine. Re-declare the names locally
per **C5**, and extend the module docstring's existing note about hand-kept mirroring to cover them.

**Target File(s).** `perception_timing_report.py`

**Strict Interfaces.** Implement **C5** exactly:

1. Append the three columns to `PHASE_COLUMNS`.
2. Add `_TRACK_PHASE_FIELDS` beside the existing `_SLAM_PHASE_FIELDS` (`:44`), with a comment saying
   why they are separate constants: the four close against `slam_ms`, the three against `track_ms`.
3. Add `track_closure(rows)`, mirroring `phase_closure` (`:124`) in structure, guards and return
   shape, with `column="track_closure_residual_ms"`.
4. In `report()`, print the track-closure line immediately under the existing SLAM-closure line,
   guarded the same way — when any required column is missing, print an explicit
   "unavailable (missing phase columns, see banner above)" line rather than omitting it silently.

**Prohibited.** Pure stdlib only — no `numpy`, no `pandas`, no importing project modules. Do not
change `phase_closure`, `PhaseStats`, or any existing function's signature or behaviour.

**Acceptance Tests.** Extend `run_self_test()` in `perception_timing_report.py`, printing
`[timing-report][self-test] PASS/FAIL  <name>` per check and ending with a bare `PASS`. Build
synthetic CSVs with `csv.DictWriter` in a `tempfile.mkdtemp()` directory, cleaned up in a `finally`:

- **Closure maths:** rows with `track_ms=500, frame_ms=50, infer_ms=0, tracker_ms=450` give
  `track_closure(...).median == 0.0`; changing `tracker_ms` to `400` gives a median residual of
  `100.0`.
- **Missing columns are loud:** on a CSV carrying the session-62 columns but none of the three new
  ones, `track_closure(rows)` raises `KeyError`, `present_columns` names all three as missing, and
  `report(path)` returns a string containing `"MISSING COLUMNS"`, the name `tracker_ms`, and the
  track-closure "unavailable" line — **and still prints the SLAM closure line normally**, proving one
  missing invariant does not suppress the other.
- **Blank handling:** blank cells in a component column are counted in `n_blank`, never coerced to
  `0.0`.
- **Both closures coexist:** on a fully-populated modern CSV, `report(path)` contains both closure
  lines and `present_columns` reports nothing missing.
- **Backward compatibility:** the existing session-62 test cases still pass unchanged.

Run: `venv\Scripts\python.exe perception_timing_report.py --self-test`
and `python perception_timing_report.py --self-test` (proving it still runs on the bare system
interpreter with no venv, no torch).

---

## POST-RUN (operator, not Sonnet)

1. **Bench before flying.** `venv\Scripts\python.exe perception_worker.py --video <any recorded mp4>
   --no-display --log --max-frames 60`, then `venv\Scripts\python.exe perception_timing_report.py` —
   confirm `frame_ms`/`infer_ms`/`tracker_ms` populate, both closure residuals sit within a few ms of
   0, and `map_pub_ms` is small.
2. **Fly once.** Expect: `map_pub_ms` **flat and small** even at high voxel counts (it hit 1 238 ms
   at 386 k before); the `track_ms` breakdown naming whatever is growing; `backend_ms` **unchanged** —
   nothing in this session touches it.
3. **Read the verdict**, then write the Phase 2 spec against it.

## PHASE 2 IS NOT IN THIS SPEC

Moving `_run_backend()` off the frame-critical path into a thread — the 55-83 % — is deliberately
excluded and must not be started. Recorded here so the next session need not re-derive it:

- `_run_backend()` is already a **queue consumer**. `states.global_optimizer_tasks` is a FIFO,
  `queue_global_optimization(idx)` appends (`third_party/MASt3R-SLAM/mast3r_slam/frame.py:185`), and
  **every `SharedStates` accessor is already `with self.lock`** (`frame.py:156,169,199-203`).
  `keyframes.lock` is taken in `_relocalization` (`slam_engine.py:151`). The reference implementation
  to port is `third_party/MASt3R-SLAM/main.py:74 run_backend(states, keyframes)`, which upstream runs
  in its **own process**.
- A separate **process** is ruled out by `slam_engine.py:38`: *"Windows `mp.Manager()` deadlocks here
  — we run tracker + backend in ONE process."* A **thread** is the remaining option; torch/CUDA ops
  release the GIL.
- It will need, per `CLAUDE.md` rules 2-3: `SlamResult.backend_mode = "ASYNC" | "SYNC"`, a
  `backend_queue_depth`, a `slam.backend_async` config kill switch, `backend_wait_ms` so the
  `slam_ms` closure still holds, and a thread exception that raises a visible degraded flag rather
  than dying quietly.
- Two consequences to state up front: keyframe points would be downloaded before that keyframe is
  optimised (a *widening of an existing property* — `solve_GN_rays()` already revises earlier
  keyframes after their points were integrated), and RELOC's mode transition becomes asynchronous
  (upstream's own behaviour).

## Closing steps (non-negotiable, per CLAUDE.md)

1. Fold this into `PROGRESS.md`'s narrative and refresh `STATE.md`'s resume pointer; archive this
   spec as `plans/session63-spec.md`.
2. Leave the tree self-describing for a cold resume from `STATE.md` alone.
