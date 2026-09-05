# SESSION 62 — Sonnet-Ready Implementation Specification
# Perception phase timing: find out where `slam_ms` actually goes

Run with: `python sonnet_runner.py --plan C:\Users\owner\.claude\plans\hey-please-read-state-md-vast-lemon.md`

On approval, archive a copy as `plans/session62-spec.md` (repo convention).

---

## EXECUTION GUIDELINES (read before every chunk)

1. **Implement ONLY the current chunk.** Do not start, preview, refactor or "improve" any other
   chunk. Earlier chunks are already applied on disk — do not re-verify or re-implement them.
2. **Signatures are contracts.** Use the exact names, parameter names, parameter order, defaults,
   types and return shapes given in `SHARED CONTRACTS`. No renames, no extra parameters, no changed
   return shapes, no "while I was here" additions. If a contract looks wrong, implement it as
   written and say so in your report.
3. **No architectural changes.** This session is *measurement only*. Do **not** introduce threads,
   queues, locks, processes, ports or new dependencies. Do not move work between call sites, do not
   reorder the pipeline, do not change what is published or when. The only intentional behavioural
   change in the whole session is the single documented `diag_perf.row()` relocation in **CHUNK 2**.
4. **Anchors are source strings, not line numbers.** `perception_worker.py` is ~1.4k lines and
   `autopilot.py` ~11k — use `Grep` to locate each anchor string, then `Edit`.
5. **NO SILENT FALLBACKS** (`CLAUDE.md`). Never swallow an error into a default. The specific trap
   in this session is *absent or unparseable data*: a missing CSV column, a blank cell, or a phase
   that did not run must each be NAMED explicitly — an absent column is a loud banner, a blank cell
   is counted in `n_blank`, a phase that did not run is an honest `0.0` in the CSV. Never invent a
   number to make a table look complete.
6. **IMAGE INTEGRITY** (`CLAUDE.md`). No frame is resized, cropped or re-encoded anywhere in this
   session. If a chunk seems to require it, stop and report instead.
7. **NO MANUAL-FLIGHT DATA LEAKAGE** (`CLAUDE.md`). Every constant here is a CSV column name, a
   report bucket width or a formatting width. Nothing derived from a specific flight or room, and
   nothing in this session feeds the autopilot at all.
8. **Comment in the surrounding style.** Dense "why", not "what", tagged `Session 62:` and citing
   the evidence in `MISSION CONTEXT`.
9. **Do not commit, stage, stash, or otherwise mutate git state.**
10. **The gate runs nine suites under the project venv after every chunk** — `autopilot.py`,
    `frontier_planner.py`, `visual_recovery.py`, `flight_replay.py`, `ground_grid.py`,
    `map_store.py`, `salvage_flight.py`, `perception_worker.py`, `visualizer.py`. Breaking any one
    halts the run. **Correction to this spec, 2026-09-05:** an earlier draft asserted all nine were
    green at session start. They were not — `autopilot.py` and `perception_worker.py` were red before
    any chunk ran, both from self-tests that asserted on the operator's LIVE `config.yaml`
    (`use_visual_backoff_trigger`, `diag.ply_sequence`) rather than on code, so flipping an
    operator-tunable flag for a flight reported itself as a code defect. Both were repaired
    out-of-band before CHUNK 2 and all nine are green now, verified under BOTH values of each flag.
    If you hit a red suite you did not cause, say so and stop — do not fix it inside a chunk.
11. **Finish by running the self-test commands the chunk names**, and report the full PASS/FAIL list
    verbatim.
12. **Every chunk must change at least one file.** An empty diff is treated as a failed chunk.

---

## MISSION CONTEXT (why this work exists)

A proposal came in to split `Pipeline` into a tracking thread and a background mapping thread, on
the theory that CPU-side `MapStore.integrate()` / `GroundGrid.integrate()` is what progressively
slows the pipeline as the voxel map grows. **The repo's own instrumentation does not support that
theory, and this session exists to settle the question with live data before any refactor is built.**

**`slam_ms` brackets only `slam.process()`.** In `perception_worker.py`, `t_slam = time.time()` sits
immediately before `res = self.slam.process(rgb)` and the stopwatch stops immediately after. The
integrate block, the clearance raycasts, `planner.select()` and every publish happen *after* it
stops. Every figure in `STATE.md`'s choke table — the 791 ms → 12 682 ms median degradation over 20
flight-minutes, the 72.9 s worst gap — is `slam_ms`. So the proposed refactor moves work that is not
inside the measurement that is degrading.

**Archived evidence (the only perception timing data that exists).**
`OUTPUT/diag/20260626_101636_perception.csv`, 1046 frames, healthy, voxels 7 536 → 26 827:

| quarter | voxels at end | non-SLAM time (`loop_dt` − `slam_ms`), median |
|---|---|---|
| 1 | 7 536 | 84 ms |
| 2 | 11 410 | 86 ms |
| 3 | 21 932 | 74 ms |
| 4 | 26 827 | 70 ms |

Non-SLAM CPU work is **~78 ms and flat — mildly *decreasing*** while the map more than triples. Over
the same file `slam_ms` median by keyframe-count bucket is 318 / 487 / 331 / 275 / 275 ms: SLAM cost
does not grow with the keyframe graph on a healthy flight either. This matches the code —
`MapStore.integrate` is O(new unique voxels in the batch), `MapStore.raycast` is O(range/step) hash
lookups and O(1) in map size.

**Where the time appears to go.** Same file, split by `new_keyframe`:

| frame kind | n | median `slam_ms` |
|---|---|---|
| ordinary TRACKING frame | 1026 | **322 ms** |
| new-keyframe frame | 20 | **1 501 ms** |

A keyframe frame costs ~4.7×, and all of it is inside `slam.process()`: `slam_engine.py` calls
`self._run_backend()` synchronously in the per-frame path (retrieval-database update,
`factor_graph.add_factors` over loop-closure candidates, a full `solve_GN_rays()` over the graph),
then pulls `X_canon`, `pW`, `conf` and `uimg` off the GPU for the keyframe. `slam_engine.py`'s own
module comment records that this repo *deliberately* collapsed upstream MASt3R-SLAM's separate
backend process into one process — that decision is what put global optimization on the critical path.

**The mode split explains the choke's non-monotonicity.** In the unhealthy file
`OUTPUT/diag/20260626_161731_perception.csv`: TRACKING median 410 ms vs **RELOC median 3 030 ms**
(p90 9 893, max 22 842). In `RELOC`, `process()` runs a full `_mast3r_inference_mono` *and*
`_relocalization()` — retrieval query plus factor-graph solve — on **every** frame. In that same file
the keyframe and voxel counts were **frozen at 15 / 53 634** while `slam_ms` still doubled
1 560 → 2 821 ms: the map was not growing at all and SLAM still halved in speed. That is direct
evidence against the "keyframe graph grows monotonically" theory, and it explains `STATE.md`'s
puzzling partial recovery at flight-minute 25-30 (the flight left RELOC). It also fits the flown
numbers — 1 348 s `HOLD_LOST` + 471 s `FALLBACK` out of 47 minutes is exactly the window where
per-frame relocalization would dominate.

**The blocker this session removes.** `fly.py` launches `perception_worker.py` **without `--log`**, so
`pipe.enable_diag()` never fires on a real flight. **No perception timing CSV has been written since
2026-06-26.** Every number above comes from June files, on a map far smaller than today's. Until that
is fixed, no claim about the choke — including the ones above — is checkable on a current flight.

**Operator decisions on record (2026-09-05):**

- **Stage 0 only this session.** Instrument, fly session 61 once with the instrumentation in the
  tree, and let the data pick the refactor. No threads are built until the data justifies one.
- **Instrument before the session-61 flight**, so one flight serves both the session-61 watch list
  (`STATE.md`, unchanged and still current) and the timing breakdown.
- **Deliberately parked, not cancelled** — recorded so the next session need not re-derive them:
  **Stage A**, move `_run_backend()` off the frame-critical path into a backend thread inside
  `slam_engine.py` (the only change that can move `slam_ms` itself; risky — `factor_graph`,
  `keyframes` and `states` are shared CUDA-backed structures and both threads would launch on one
  stream). **Stage B**, decouple `TOPIC_PLAN` publishing from the SLAM cadence — today a plan can
  only be published from inside `step()`, so `PLAN_PUB_INTERVAL = 0.5 s` is a lie and during a 12 s
  solve the autopilot gets no plan and no forward clearance at all; a fix must carry explicit
  `pose_age_s` / `pose_frame_id`. **Stage C**, the original proposal — and if built, `MapStore._grow`
  *reallocates* `_count` / `_color_sum`, so a concurrent raycast is a **torn** read, not a stale one
  (copy-on-write snapshot swap, not a "lightweight" lock); `clearance` and `planner.select` must stay
  on one side of the fence or a single `_plan_payload` mixes two map snapshots; and
  `flight_replay.py` / `salvage_flight.py` / the timeline all assume plan↔frame correspondence.

**Out of scope, stays parked in `STATE.md`:** everything on the session-61 watch list, the SLAM-choke
*cure* itself, the staleness UI, and the goal-management rewrite decision rule.

---

## SHARED CONTRACTS

Read before every chunk.

### C1 — `slam_engine.SlamResult` phase fields (CHUNK 1)

Four new fields **appended** to the existing dataclass. All are `float`, in **milliseconds**,
**always present**, and default to `0.0`. A phase that did not run this frame is `0.0` — never
`None` (a `None` becomes a blank CSV cell and a ragged row downstream).

```python
@dataclass
class SlamResult:
    # ... every existing field UNCHANGED, in its existing order ...
    kf_colors: np.ndarray | None = None
    # Session 62 — per-phase wall-clock split of what `slam_ms` measures, so the choke can be
    # attributed instead of guessed. Always present; 0.0 = the phase did not run this frame.
    track_ms: float = 0.0          # frame construction + the INIT/TRACKING/RELOC mode branch
    backend_ms: float = 0.0        # _run_backend(): retrieval update + add_factors + solve_GN_*
    pose_ms: float = 0.0           # pose recovery (Act3 basis -> numpy pose_mat + center)
    kf_download_ms: float = 0.0    # new-keyframe GPU->CPU pull (X_canon, pW, conf, uimg) + ray field
```

Module-level constant, in `slam_engine.py`, immediately after the dataclass:

```python
# Session 62: the phase names in SlamResult, in pipeline order. Single source of truth shared by
# perception_worker's CSV schema and the timing report — never re-type this list anywhere.
SLAM_PHASE_FIELDS: tuple[str, ...] = ("track_ms", "backend_ms", "pose_ms", "kf_download_ms")
```

**Closure invariant (asserted by the report in CHUNK 4):**
`track_ms + backend_ms + pose_ms + kf_download_ms  ≈  slam_ms`, residual within a few ms.

### C2 — `perception_worker` CSV schema (CHUNK 2)

Module-level constant in `perception_worker.py`, beside `MAP_GRID`:

```python
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
)
```

Post-SLAM phase definitions — each a `float` in milliseconds, rounded to 1 decimal, **always
written**, `0.0` when that phase did not run on this frame:

| column | measures exactly |
|---|---|
| `integrate_ms` | `mapstore.add_pose` + `mapstore.integrate` + `ground.integrate` (the `map_updated` block) |
| `publish_ms` | the `state_pub.publish(frame_bus.TOPIC_POSE, {...})` call |
| `map_pub_ms` | `self._map_payload(res, meta)` **and** its publish; `0.0` when the map timer did not fire |
| `plan_ms` | `self._plan_payload(...)` **and** its publish; `0.0` when the plan timer did not fire |

### C3 — sticky console phases (CHUNK 3)

```python
# Session 62: last observed duration of each phase, in ms. STICKY — a phase that did not run this
# frame keeps its previous value here, so the 1 Hz console line stays readable across the frames
# where the map/plan timers don't fire. The CSV is the honest record (0.0 for "did not run"); THIS
# dict is a console convenience only and must never be logged, published or used for a decision.
self._last_phase_ms: dict[str, float] = {
    "track": 0.0, "backend": 0.0, "pose": 0.0, "kf_download": 0.0,
    "integrate": 0.0, "map_pub": 0.0, "plan": 0.0, "publish": 0.0,
}
```

### C4 — `perception_timing_report.py` public API (CHUNK 4)

```python
PHASE_COLUMNS: tuple[str, ...] = (
    "slam_ms", "track_ms", "backend_ms", "pose_ms", "kf_download_ms",
    "integrate_ms", "map_pub_ms", "plan_ms", "publish_ms")

@dataclass(frozen=True)
class PhaseStats:
    column: str        # the CSV column these stats describe
    n: int             # rows with a parseable float
    n_blank: int       # rows whose cell was "" or unparseable (NAMED, never silently dropped)
    median: float      # 0.0 when n == 0
    p90: float         # 0.0 when n == 0
    maximum: float     # 0.0 when n == 0

def load_rows(csv_path: str | Path) -> list[dict[str, str]]: ...
def present_columns(rows: list[dict[str, str]]) -> tuple[list[str], list[str]]: ...
def phase_stats(rows: list[dict[str, str]], column: str) -> PhaseStats: ...
def phase_closure(rows: list[dict[str, str]]) -> PhaseStats: ...
def bucket_by_minute(rows: list[dict[str, str]], minutes: float = 5.0) -> list[tuple[str, list[dict[str, str]]]]: ...
def bucket_by_mode(rows: list[dict[str, str]]) -> list[tuple[str, list[dict[str, str]]]]: ...
def bucket_by_keyframe(rows: list[dict[str, str]]) -> list[tuple[str, list[dict[str, str]]]]: ...
def render_table(title: str, buckets: list[tuple[str, list[dict[str, str]]]],
                 columns: tuple[str, ...] = PHASE_COLUMNS) -> str: ...
def report(csv_path: str | Path) -> str: ...
def run_self_test() -> None: ...
def main() -> None: ...
```

Behavioural contracts:

- `load_rows` — opens with `encoding="utf-8"`, returns `csv.DictReader` rows as a `list`. Raises
  `FileNotFoundError` if absent, `ValueError` if the file has no header row. Never returns `None`.
- `present_columns` — returns `(present, missing)`: which of `PHASE_COLUMNS` are in the header and
  which are not. Empty `rows` → `([], list(PHASE_COLUMNS))`.
- `phase_stats` — **raises `KeyError(column)`** when `column` is absent from the header. Blank or
  unparseable cells are counted in `n_blank`, never coerced to `0.0`. `p90` is the value at index
  `int(0.9 * n)` clamped to `n - 1` of the sorted values.
- `phase_closure` — per-row residual `slam_ms - (track_ms + backend_ms + pose_ms + kf_download_ms)`;
  returns a `PhaseStats` with `column="closure_residual_ms"`. Rows missing any component count as
  `n_blank`. Raises `KeyError` if `slam_ms` or any of `SLAM_PHASE_FIELDS` is absent from the header.
- `bucket_by_minute` — buckets on `wall_ts` (float seconds) relative to the **first row's** `wall_ts`;
  bucket index `int((ts - ts0) // (minutes * 60))`; label `f"{lo:g}-{hi:g} min"`. Rows with a blank
  or unparseable `wall_ts` go to a bucket labelled `"unparseable wall_ts"` — reported, not dropped.
- `bucket_by_mode` — one bucket per distinct `mode` value, sorted by label.
- `bucket_by_keyframe` — exactly two buckets, `"new_keyframe"` (`new_keyframe` cell `== "1"`) and
  `"ordinary"`, in that order, both always present even when empty.
- `render_table` — a markdown table: one row per bucket, one column group per phase, cells rendered
  `f"{median:.0f}/{p90:.0f}"` (median/p90 in ms), plus a leading `n` column. A column absent from the
  header is rendered `"--"` in every row, and the table's caption names it as missing.
- `report` — the whole multi-section text: a header naming the file and row count, a loud
  `*** MISSING COLUMNS: ... (file predates session 62) ***` banner when `present_columns` reports
  any, the SLAM-phase-closure line, then the three tables (by flight-minute, by SLAM mode, by
  keyframe). Returns a `str`; performs no printing.
- `main` — `argparse`: positional `csv_path` (optional), `--self-test`, `--minutes` (float, default
  `5.0`). With `--self-test` runs `run_self_test()` and returns. Without a `csv_path` and without
  `--self-test`, resolves the **newest** `OUTPUT/diag/*_perception.csv` by filename sort and reports
  on it, printing which file it chose; if none exists it prints an explicit message naming the
  directory it searched and exits non-zero.

---

## CHUNK 1 — SLAM-internal phase timing

**Module Objective.** Split what `slam_ms` measures into four named phases inside
`SlamEngine.process()`, and carry them out on `SlamResult`. Measurement only — no behavioural change.

**Required Context/Dependencies.** None (first chunk). Contract **C1**.

**Target File(s).** `slam_engine.py`

**Strict Interfaces.**

1. Add the four fields and `SLAM_PHASE_FIELDS` exactly as in **C1**. Append the fields — do not
   reorder or retype any existing field.
2. Add `import time` to the module imports if absent (alphabetical position among the stdlib group).
3. In `process()`, take five `time.perf_counter()` stamps. Anchors, in order:
   - Anchor `        i = self._i` — insert immediately **before** it:
     `_t0 = time.perf_counter()`
   - Anchor `        if not ran_init:\n            self._run_backend()` — insert
     `_t_track = time.perf_counter()` immediately **before** the `if`, and
     `_t_backend = time.perf_counter()` immediately **after** the `self._run_backend()` line
     (dedented back to the `if`'s level, so it runs on both branches).
   - Anchor `        center = w[0].astype(np.float32).copy()` — insert immediately **after** it:
     `_t_pose = time.perf_counter()`
   - Anchor `        self._i += 1` — insert immediately **before** it:
     `_t_kf = time.perf_counter()`
4. Immediately before the `return SlamResult(` statement, compute:
   ```python
   # Session 62: attribute slam_ms instead of guessing at it. A phase that did not run reports a
   # literal 0.0, never the sub-microsecond noise of an unentered branch — see CLAUDE.md's
   # no-silent-fallback rule applied to measurement: "did not run" must be distinguishable.
   track_ms = (_t_track - _t0) * 1000.0
   backend_ms = 0.0 if ran_init else (_t_backend - _t_track) * 1000.0
   pose_ms = (_t_pose - _t_backend) * 1000.0
   kf_download_ms = (_t_kf - _t_pose) * 1000.0 if new_kf else 0.0
   ```
5. Pass all four into the `SlamResult(...)` call as keyword arguments, after `kf_colors=kf_colors`.

**Prohibited.** Do not change control flow, do not add a `try`/`except`, do not touch
`_run_backend`, `_relocalization`, the tracker or any CUDA call. Do not add a thread.

**Acceptance Tests.** Add `_self_test_slam_phase_fields()` to `perception_worker.py` (it already
imports `slam_engine` at module level, and `SlamResult` is a plain dataclass needing no CUDA), and
call it from `run_self_test` in the established `ok = ...; print PASS/FAIL; assert ok` style:

- `slam_engine.SLAM_PHASE_FIELDS == ("track_ms", "backend_ms", "pose_ms", "kf_download_ms")`.
- A `SlamResult` built with only the pre-session-62 required arguments has all four phase fields
  present and `== 0.0`.
- Every name in `SLAM_PHASE_FIELDS` is an attribute of that instance, and each is a `float`.
- Constructing with `track_ms=12.5, backend_ms=900.0, pose_ms=1.5, kf_download_ms=70.0` round-trips
  those exact values.

Run: `venv\Scripts\python.exe perception_worker.py --self-test`

---

## CHUNK 2 — post-SLAM phase timing + CSV schema

**Module Objective.** Promote the `diag_perf` schema to a shared constant, time the four post-SLAM
phases in `Pipeline.step()`, and write all eight new columns per frame.

**Required Context/Dependencies.** CHUNK 1's `SlamResult` phase fields and
`slam_engine.SLAM_PHASE_FIELDS` (already applied on disk). Contracts **C1**, **C2**.

**Target File(s).** `perception_worker.py`

**Strict Interfaces.**

1. Add the `DIAG_PERF_FIELDS` module constant exactly as in **C2**, immediately after the `MAP_GRID`
   line.
2. `enable_diag` — replace the inline column list in the `DiagLog("perception", [...])` call with
   `list(DIAG_PERF_FIELDS)`. Leave the `diag_lift` log untouched.
3. **Relocate the `diag_perf.row(...)` call.** This is the one intentional behavioural change in the
   session and must carry a comment saying so. Anchor: the existing `        self.diag_perf.row(`
   block. Cut it from its current position (immediately after the `slam_ms` computation) and place
   it immediately **before** `        return res, None, None, map_updated` at the end of `step()`.
   Add:
   ```python
   # Session 62: the row moved from just-after-SLAM to end-of-step so it can carry the POST-SLAM
   # phases too. One documented consequence: `n_voxels` is now the count AFTER this frame's
   # integrate rather than before it (off by one keyframe's worth against pre-2026-09-05 files);
   # `loop_dt` and every other column are unchanged.
   ```
4. Time the four post-SLAM phases with `time.perf_counter()`, initialising each to `0.0` before its
   guard so a frame that skipped the phase writes an honest `0.0`:
   - `integrate_ms` — around the block anchored by `        if res.new_keyframe and res.kf_points is not None and len(res.kf_points):`, **including** the preceding `self.mapstore.add_pose(res.camera_center)` guard.
   - `publish_ms` — around the `state_pub.publish(frame_bus.TOPIC_POSE, {` call only.
   - `map_pub_ms` — around the body of the `if map_updated or (now_mono - self.last_map_pub) >= self.MAP_PUB_INTERVAL:` branch.
   - `plan_ms` — around the body of the `if (now_mono - self.last_plan_pub) >= self.PLAN_PUB_INTERVAL:` branch.
   When `state_pub is None` (the offline path) all four publish-related timings stay `0.0`;
   `integrate_ms` is still measured.
5. Extend the relocated `diag_perf.row(...)` call with the eight new keyword arguments, each
   `round(..., 1)`: `track_ms=round(res.track_ms, 1)`, `backend_ms=`, `pose_ms=`,
   `kf_download_ms=` (read from `res`), and `integrate_ms=`, `map_pub_ms=`, `plan_ms=`,
   `publish_ms=` (the locals measured here). Keep every existing keyword argument unchanged.

**Prohibited.** Do not change publish conditions, timers, payloads or ordering. Do not wrap anything
in `try`/`except`. Do not touch `_plan_payload` or `_map_payload` internals.

**Acceptance Tests.** Add `_self_test_phase_timing(cfg)` to `perception_worker.py`, wired into
`run_self_test` in the established style. No GPU, no SLAM — it exercises the schema and the
`DiagLog` round-trip in a `tempfile.mkdtemp()` directory, cleaned up in a `finally`:

- `DIAG_PERF_FIELDS[:9] == ("wall_ts", "frame_id", "loop_dt", "slam_ms", "mode", "new_keyframe",
  "n_keyframes", "n_voxels", "reloc")` — the frozen-prefix guarantee that keeps June files readable.
- Every name in `slam_engine.SLAM_PHASE_FIELDS` appears in `DIAG_PERF_FIELDS`.
- `DIAG_PERF_FIELDS` contains `"integrate_ms"`, `"map_pub_ms"`, `"plan_ms"`, `"publish_ms"`.
- `len(set(DIAG_PERF_FIELDS)) == len(DIAG_PERF_FIELDS)` — no duplicated column name.
- A `DiagLog("perception", list(DIAG_PERF_FIELDS), out_dir=tmp, ts="20260101_000000")` written with
  one fully-populated row and read back with `csv.DictReader` yields a header equal to
  `list(DIAG_PERF_FIELDS)` and the exact values written.
- A row written with the phase keywords **omitted** yields blank (`""`) cells for them, not `"0.0"` —
  proving a missing field is visibly missing rather than silently zeroed.

Run: `venv\Scripts\python.exe perception_worker.py --self-test`

---

## CHUNK 3 — live console phase breakdown

**Module Objective.** Make the split legible in the perception console at 1 Hz, not only post-hoc in
the CSV.

**Required Context/Dependencies.** CHUNK 2's phase locals in `step()`. Contract **C3**.

**Target File(s).** `perception_worker.py`

**Strict Interfaces.**

1. In `Pipeline.__init__`, beside `self._last_clearance = None`, add `self._last_phase_ms` exactly as
   in **C3**, with that comment.
2. In `step()`, after the phase locals are known and **before** the 1 Hz report block, update
   `self._last_phase_ms` **stickily**: write a key only when its phase actually ran this frame
   (`track` / `backend` / `pose` from `res` every frame; `kf_download` only when `res.new_keyframe`;
   `integrate` only when `map_updated`; `map_pub` / `plan` / `publish` only when that publish fired).
3. Extend the existing 1 Hz report `print(` (anchor: `f"vox {len(self.mapstore):6d} | slam {slam_ms:5.1f} ms | "`) with one new segment inserted immediately after the `slam ... ms` segment:
   ```python
   f"[trk {p['track']:.0f} bk {p['backend']:.0f} dl {p['kf_download']:.0f}] | "
   f"intg {p['integrate']:.0f} plan {p['plan']:.0f} map {p['map_pub']:.0f} ms | "
   ```
   where `p = self._last_phase_ms`. Keep every existing segment, in its existing order.

**Prohibited.** Do not log `_last_phase_ms` to the CSV, publish it on any topic, or read it anywhere
outside this print. It is sticky and therefore not an honest per-frame record.

**Acceptance Tests.** Extend `_self_test_phase_timing(cfg)` from CHUNK 2 (do not add a new self-test
function) with a `types.SimpleNamespace` standing in for the pipeline:

- A fresh `Pipeline`-shaped dict from **C3** has all eight keys, every value `0.0`.
- Applying the sticky-update rule with `map_updated=False` leaves a previously-set
  `integrate` value **unchanged** (not reset to `0.0`) — the stickiness contract.
- The formatted segment renders without raising for an all-zero dict and for a populated one, and
  the rendered string contains `"trk "`, `"bk "`, `"dl "`, `"intg "`, `"plan "` and `"map "`.

Run: `venv\Scripts\python.exe perception_worker.py --self-test`

---

## CHUNK 4 — the timing report

**Module Objective.** A standalone, GPU-free analysis tool that turns a flight's
`*_perception.csv` into the go/no-go table for Stages A/B/C.

**Required Context/Dependencies.** CHUNK 2's `perception_worker.DIAG_PERF_FIELDS` and CHUNK 1's
`slam_engine.SLAM_PHASE_FIELDS` — **import neither**. `slam_engine` pulls in `torch` and
`lietorch`; this tool must stay pure stdlib so it runs outside the venv on any machine. Re-declare
`PHASE_COLUMNS` locally per **C4** and note in the module docstring that it mirrors
`perception_worker.DIAG_PERF_FIELDS`, with the frozen-prefix guarantee as the reason that is safe.

**Target File(s).** `perception_timing_report.py` (new), `sonnet_runner.py` (one-line edit)

**Strict Interfaces.** Implement exactly the API and behavioural contracts in **C4**. Pure stdlib
only — `argparse`, `csv`, `statistics`, `dataclasses`, `pathlib`, `sys`. No `numpy`, no `pandas`.

Module docstring must state: what the tool answers, that it reads by column **name** so pre-session-62
files still work (with their missing columns named loudly), and the closure invariant from **C1**.

In `sonnet_runner.py`, append `"perception_timing_report.py"` to `DEFAULT_SUITES` so the gate covers
it from the next run onward. Note in your report that `DEFAULT_SUITES` is read at module load, so
this does not affect the currently-executing run.

**Acceptance Tests.** `run_self_test()` inside `perception_timing_report.py`, printing
`[timing-report][self-test] PASS/FAIL  <name>` per check and ending with a bare `PASS`, matching the
repo's suite convention (non-zero exit or the word `FAIL` is what the gate detects). Build synthetic
CSVs with `csv.DictWriter` in a `tempfile.mkdtemp()` directory, cleaned up in a `finally`:

1. **Round-trip.** A 10-row modern CSV loads to 10 dicts; `present_columns` returns all of
   `PHASE_COLUMNS` present and nothing missing.
2. **Statistics.** A column with values `1..10` gives `median == 5.5`, `p90 == 10.0`,
   `maximum == 10.0`, `n == 10`, `n_blank == 0`.
3. **Blank handling.** Two blank cells among 10 give `n == 8`, `n_blank == 2`, and a median computed
   over the 8 parseable values only — assert the exact expected median, and assert `n_blank` is
   non-zero rather than the blanks being coerced to `0.0`.
4. **Missing column is loud.** A CSV with the nine frozen columns only: `present_columns` reports the
   eight session-62 columns as missing; `phase_stats(rows, "backend_ms")` raises `KeyError`; and
   `report(path)` returns a string containing `"MISSING COLUMNS"` and the name `backend_ms`.
5. **Closure.** Rows with `slam_ms=1000, track_ms=300, backend_ms=600, pose_ms=20,
   kf_download_ms=80` give `phase_closure(...).median == 0.0`; changing `backend_ms` to `500` gives
   a median residual of `100.0`.
6. **Minute buckets.** Rows spanning 12 minutes of `wall_ts` with `minutes=5.0` produce exactly three
   buckets, labelled `"0-5 min"`, `"5-10 min"`, `"10-15 min"`, in that order.
7. **Unparseable `wall_ts`.** One row with `wall_ts=""` produces a bucket labelled
   `"unparseable wall_ts"` and that row appears in no other bucket.
8. **Mode + keyframe buckets.** A mixed TRACKING/RELOC CSV yields two mode buckets with the right
   counts; `bucket_by_keyframe` always yields exactly two buckets in the order
   `["new_keyframe", "ordinary"]`, both present even when one is empty.
9. **Render.** `render_table` on a bucket list returns a string whose first line is the title and
   which contains a `|`-delimited header row; a table built where `backend_ms` is missing renders
   `"--"` cells and names the column in its caption.

Run: `venv\Scripts\python.exe perception_timing_report.py --self-test`
and `python perception_timing_report.py --self-test` (proving it runs on the bare system
interpreter with no venv, no torch).

---

## CHUNK 5 — arm it on real flights

**Module Objective.** Make `fly.py` actually produce the CSV. Last, so the tree is only armed once
everything above is green.

**Required Context/Dependencies.** Chunks 1-4 applied.

**Target File(s).** `fly.py`

**Strict Interfaces.** Anchor:
`perception = subprocess.Popen([python_exe, "perception_worker.py", "--no-display", "--stop-file", perception_stop_file]`

Insert `"--log"` into the argv list immediately after `"--no-display"`. Add a comment above the line:

```python
# Session 62: --log turns on perception_worker's diag CSVs (per-frame SLAM/loop/phase timing).
# It was never passed here, so pipe.enable_diag() never fired on a real flight and NO perception
# timing CSV existed after 2026-06-26 -- the SLAM-choke table in STATE.md had to be reconstructed
# from autopilot-side plan payloads. Matches how autopilot.py is launched on the next line.
```

**Prohibited.** Do not change any other launch argument, the launch order, the sleep, or teardown.

**Acceptance Tests.** No self-test module owns `fly.py`; verify by inspection and report:

- `Grep` `fly.py` for `perception_worker.py` and quote the full resulting line, showing `--log`
  present, `--no-display` and `--stop-file perception_stop_file` still present and in order.
- Confirm `python -c "import ast,pathlib; ast.parse(pathlib.Path('fly.py').read_text(encoding='utf-8'))"` parses clean.
- Re-run the full nine-suite gate.

Run: `venv\Scripts\python.exe perception_worker.py --self-test`
and `venv\Scripts\python.exe perception_timing_report.py --self-test`

---

## POST-RUN (operator, not Sonnet)

1. **Bench check before flying.** `venv\Scripts\python.exe perception_worker.py --video <any recorded mp4> --no-display --log --max-frames 60`, then
   `venv\Scripts\python.exe perception_timing_report.py` — confirm the newest CSV has the eight new
   columns populated non-zero and that the closure residual median is within a few ms of `0`.
2. **Fly session 61 once.** Its watch list in `STATE.md` is unchanged and still applies;
   `diag.ply_sequence` is already on.
3. **Read the verdict.** Run the report on the flight CSV. If the diagnosis in `MISSION CONTEXT` is
   right, expect: `backend_ms` dominating on `new_keyframe` frames, RELOC frames several × TRACKING
   frames, and `integrate_ms + plan_ms` small and roughly flat across flight-minutes. That table
   decides between Stage A, Stage B and Stage C — or shows all three are aimed wrong.

## Closing steps (non-negotiable, per CLAUDE.md)

1. Fold this into `PROGRESS.md`'s narrative (the async-refactor proposal, the evidence that re-aimed
   it, what Stage 0 measured) and refresh `STATE.md`'s resume pointer; archive this spec as
   `plans/session62-spec.md` with the parked Stage A/B/C designs intact.
2. Leave the tree self-describing for a cold resume from `STATE.md` alone.
