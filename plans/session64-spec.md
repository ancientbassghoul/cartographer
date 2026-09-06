# SESSION 64 — Sonnet-Ready Implementation Specification
# Stage A: move the SLAM backend off the frame-critical path

Run with: `python sonnet_runner.py --plan C:\Users\owner\.claude\plans\hey-please-read-state-md-vast-lemon.md`

On approval, archive a copy as `plans/session64-spec.md` (repo convention).

---

## EXECUTION GUIDELINES (read before every chunk)

1. **Implement ONLY the current chunk.** Do not start, preview, refactor or "improve" any other
   chunk. Earlier chunks are already applied on disk — do not re-verify or re-implement them.
2. **Signatures are contracts.** Use the exact names, parameter names, parameter order, defaults,
   types and return shapes given in `SHARED CONTRACTS`. No renames, no extra parameters, no changed
   return shapes. If a contract looks wrong, implement it as written and say so in your report.
3. **Do not modify anything under `third_party/`.** Not one line. Every change lives in
   `slam_engine.py`, `perception_worker.py`, `perception_timing_report.py`, `visualizer.py` and
   `config.yaml`. The upstream concurrency model is being *restored*, not edited.
4. **This session introduces exactly ONE thread, in CHUNK 1, and nothing else concurrent.** No
   queues, no processes, no ports, no new dependencies, no thread pools. Do not thread map
   integration, planning, publishing or the tracker.
5. **Anchors are source strings, not line numbers.** Use `Grep` to locate each anchor, then `Edit`.
6. **NO SILENT FALLBACKS** (`CLAUDE.md`). This is the session where that rule earns its keep. A
   backend thread that dies must NOT quietly revert to synchronous or quietly stop optimising — it
   sets a visible `FAILED` state that reaches the CSV, the console and the visualizer, and logs
   CRITICAL once. A queue that grows without bound must be *visible*, not capped behind the
   operator's back. "The backend is behind" and "the backend is dead" must be distinguishable.
7. **IMAGE INTEGRITY** (`CLAUDE.md`). No frame is resized, cropped or re-encoded. If a chunk seems
   to require it, stop and report instead.
8. **NO MANUAL-FLIGHT DATA LEAKAGE** (`CLAUDE.md`). Every constant here is a poll interval, a join
   timeout, a column name or a formatting width. Nothing derived from a specific flight or room.
9. **Comment in the surrounding style.** Dense "why", not "what", tagged `Session 64:` and citing the
   evidence in `MISSION CONTEXT`.
10. **Do not commit, stage, stash, or otherwise mutate git state.**
11. **The gate runs ten suites under the project venv after every chunk** — `autopilot.py`,
    `frontier_planner.py`, `visual_recovery.py`, `flight_replay.py`, `ground_grid.py`, `map_store.py`,
    `salvage_flight.py`, `perception_worker.py`, `visualizer.py`, `perception_timing_report.py`.
    All ten were verified green at HEAD immediately before this spec was written. If you hit a red
    suite you did not cause, say so and stop — do not fix it inside a chunk.
12. **No self-test may require CUDA, a GPU, or a real `SlamEngine`.** The suites run on any machine.
    Every test here works on plain dataclasses, a fake states/keyframes double, or a `SlamEngine`
    instance created without `__init__` (`object.__new__`), never a constructed engine.
13. **Finish by running the self-test commands the chunk names**, and report the full PASS/FAIL list
    verbatim. **Every chunk must change at least one file.**

---

## MISSION CONTEXT (why this work exists)

Session 62 instrumented the perception loop; session 63 Phase 1 took the cheap half. The remaining
problem is one number. Flight `20260905_221128` (386 frames, 2536 s in the loop, **0.15 Hz**):

| phase | total | share |
|---|---|---|
| **`backend_ms`** | **1633.6 s** | **64.4 %** |
| `track_ms` | 800.4 s | 31.6 % |
| `integrate_ms` | 24.1 s | 0.9 % |
| `plan_ms` | 16.5 s | 0.7 % |
| `map_pub_ms` | 6.7 s | 0.3 % |

By flight-minute the medians reach `slam_ms` **19 129 ms** with `backend_ms` **15 583 ms** (minutes
25-30) and a p90 of **36 549 ms** (minutes 40-45). Phase 1 did exactly what it promised —
`map_pub_ms` fell from 49.3 s / 191 ms-per-publish to 6.7 s / 9.1 ms, a 21× cut — and the operator
correctly did not notice, because 43 saved seconds are invisible beside 1634.

**Session 63's `track_ms` split, now flown, also decomposed the other 30 %:**

| mode | `track_ms` | dominated by |
|---|---|---|
| RELOC | 1783 ms | `infer_ms` **1762 ms (99 %)** — `_mast3r_inference_mono` |
| TRACKING | 1109 ms | `tracker_ms` **945 ms (85 %)** — `tracker.track()` |

`frame_ms` is 5-8 ms throughout. So the frame path is model work, and **that sets the ceiling for
this session: even a perfect backend thread leaves ~2.1 s/frame, roughly 0.5 Hz.** The goal is NOT a
fast pipeline. The goal is turning 20-40 second blackouts into a steady ~2 s cadence, so the
autopilot stops going blind — which is what the FALLBACK work of sessions 62b/62c could not fix from
the outside.

**Why this is a restoration, not an invention.** `_run_backend()` is already a queue consumer copied
from upstream, and every structure it shares with the tracker is already lock-guarded:

- `states.global_optimizer_tasks` is a FIFO; `queue_global_optimization(idx)` appends
  (`third_party/MASt3R-SLAM/mast3r_slam/frame.py:185`), `_run_backend` pops one per call.
- **Every `SharedStates` accessor is `with self.lock`** (`frame.py:156,169,185,199-203`).
- **Every `SharedKeyframes` accessor is `with self.lock`** — `__getitem__`, `__setitem__`, `append`,
  `pop_last`, `last_keyframe`, `update_T_WCs` (`frame.py:250,271,295,299,303,309`), over
  `share_memory_()` CUDA tensors sized by a fixed `buffer=512`.
- The reference implementation is `third_party/MASt3R-SLAM/main.py:74 run_backend(states, keyframes)`,
  which upstream runs in its **own process**.

`slam_engine.py:38` records why this repo collapsed it: *"Windows `mp.Manager()` deadlocks here — we
run tracker + backend in ONE process."* That rules out a separate **process** and leaves a **thread**,
where `InProcessManager.RLock()` returns a real `threading.RLock` — so those guards become genuinely
effective for the first time in this repo, instead of uncontended no-ops.

**The one race, stated plainly (operator decision: ship it and count it).**
`FrameTracker.track()` reads the newest keyframe (`tracker.py:29`), spends ~1 s matching, then writes
the **whole row back** including the pose (`tracker.py:101` → `__setitem__`, which assigns
`self.T_WC[idx] = value.T_WC.data`). If the backend's `solve_GN_rays()` optimised that keyframe during
the match, the tracker's write-back overwrites it with a stale pose. Upstream has exactly this race —
same code, backend in another process over the same shared tensors — so it is upstream's validated
operating condition, not something introduced here. It is also **self-correcting**: the factor graph
persists, so the next solve re-optimises the same keyframe. This session therefore does not prevent
it; it **counts** it (`backend_pose_clobbers`), so flight one says whether it matters at all.

**Out of scope.** Stage B (decoupling `TOPIC_PLAN` from the SLAM cadence) and Stage C (the `Pipeline`
thread split, measured at 3.3 %) stay parked in `plans/session62-spec.md`. No recovery/FALLBACK
changes. No `track_ms` optimisation — that is the next question, and it needs this flight first.

---

## SHARED CONTRACTS

Read before every chunk.

### C1 — `SlamEngine` backend thread (CHUNK 1)

```python
class SlamEngine:
    def __init__(self, device="cuda:0", config_path="config/base.yaml", conf_thresh=1.5,
                 backend_async: bool = True):        # <-- appended, keyword, defaulted
        ...
        self.backend_async: bool = bool(backend_async)
        self.backend_failed: str | None = None   # None | the exception text that killed the thread
        self._backend_thread: threading.Thread | None = None
        self._backend_stop = threading.Event()
        self._backend_meta_lock = threading.Lock()   # guards the four counters below ONLY
        self._backend_thread_ms: float = 0.0     # duration of the thread's most recent solve
        self._backend_solves: int = 0            # completed solves this flight
        self._backend_pose_clobbers: int = 0     # see C4
        self._backend_last_written: dict[int, "torch.Tensor"] = {}   # idx -> T_WC clone, see C4

    def start_backend(self) -> bool:
        """Start the backend thread. Returns True if it started, False when backend_async is off.
        Idempotent: a second call while the thread is alive is a no-op returning True."""

    def close(self, timeout: float = 10.0) -> None:
        """Stop and join the backend thread. Safe to call when it was never started, and safe to
        call twice. Never raises."""

    def _backend_loop(self) -> None:
        """Thread body: drain global_optimizer_tasks by calling the EXISTING _run_backend()."""
```

**Thread body contract.** A `while not self._backend_stop.is_set()` loop that calls the existing
`self._run_backend()` unchanged, times it, and sleeps `BACKEND_IDLE_SLEEP_S` when there was nothing
to do. Do **not** reimplement or restructure `_run_backend` — it is already correct and already takes
every lock it needs.

```python
BACKEND_IDLE_SLEEP_S: float = 0.005   # module-level; poll gap when the task queue is empty
```

**Idle detection:** before each call, read the depth via `self._queue_depth()`; treat a pass as "did
work" when the depth was non-zero **or** the mode was `RELOC` (the RELOC branch of `_run_backend` does
work with an empty optimizer queue). Sleep only on an idle pass.

**Failure contract (CLAUDE.md rules 2-4).** Wrap the loop body in `try/except BaseException`. On an
exception: store `self.backend_failed = f"{type(exc).__name__}: {exc}"`, print a single
`*** CRITICAL: SLAM backend thread died (...) -> global optimization is STOPPED; tracking continues
DEGRADED. Set perception.backend_async=false to run it inline. ***`, and **exit the loop**. Do **not**
retry, do **not** revert to synchronous, do **not** re-raise into the void. `backend_mode` (C2) then
reports `"FAILED"` on every subsequent frame, which is how the operator finds out.

**`process()` change — exactly one line's worth of behaviour.** The existing anchor

```python
        _t_track = time.perf_counter()
        if not ran_init:
            self._run_backend()
        _t_backend = time.perf_counter()
```

becomes: call `self._run_backend()` only when `not ran_init and not self.backend_async`. The stamps
stay exactly where they are, so `backend_ms` keeps its meaning — **time the FRAME PATH spent in the
backend** — and is therefore `0.0` in async mode. This is deliberate: the `slam_ms` closure invariant
from session 62 (`track + backend + pose + kf_download ≈ slam_ms`) then holds unchanged in both
modes, and `perception_timing_report` needs no special case.

`_queue_depth()` is a small helper returning `len(states.global_optimizer_tasks)` under `states.lock`.

### C2 — `SlamResult` backend telemetry (CHUNK 2)

Four fields **appended** to the existing dataclass, after the session-63 three. Always present.

```python
    # Session 64 — the backend is no longer on the frame path, so its state must be reported
    # explicitly rather than inferred from a timing column that is now always 0.0 (CLAUDE.md 2+3).
    backend_mode: str = "SYNC"          # "SYNC" | "ASYNC" | "FAILED"
    backend_queue_depth: int = 0        # global_optimizer_tasks depth at this frame
    backend_thread_ms: float = 0.0      # the thread's most recent completed solve (0.0 in SYNC)
    backend_pose_clobbers: int = 0      # cumulative; see C4
```

`backend_mode` is `"FAILED"` whenever `self.backend_failed is not None`, `"ASYNC"` when the thread is
running, `"SYNC"` otherwise. **`backend_thread_ms` is diagnostic only and belongs to NO closure
invariant** — it measures a different thread's work over a different interval and must never be
summed against `slam_ms`.

### C3 — CSV + report extension (CHUNKS 3, 4)

`perception_worker.DIAG_PERF_FIELDS` is extended by **appending only** — the existing twenty names
and their order are frozen so every earlier flight stays readable by the same tool:

```python
    # --- Session 64: backend-thread state (NOT part of any closure; backend_ms stays the
    #     frame-path number and is 0.0 in ASYNC) ---
    "backend_mode", "backend_queue_depth", "backend_thread_ms", "backend_pose_clobbers",
```

`perception_timing_report.PHASE_COLUMNS` gains **`backend_thread_ms` only** — it is the one numeric
duration among the four. `backend_mode` / `backend_queue_depth` / `backend_pose_clobbers` are **not**
phase columns and must not be added to `PHASE_COLUMNS`, because `render_table` would present them as
millisecond medians, which two of them are not. They get their own one-line summary in `report()`
(see CHUNK 4).

### C4 — clobber detection (CHUNK 2)

After the backend's solve writes poses, record what it wrote; after the tracker returns, check whether
the newest keyframe's pose was overwritten.

- In `_backend_loop`, immediately **after** a completed `_run_backend()` pass that did work, under
  `_backend_meta_lock`, replace `_backend_last_written` with `{idx: T_WC_clone}` for the newest
  keyframe index only (`n = len(self.keyframes); idx = n - 1`), read under `keyframes.lock` and
  `.detach().clone()`d. One small tensor per solve — do not clone the whole buffer.
- In `process()`, immediately **after** `self.tracker.track(frame)` returns (TRACKING branch only),
  compare the current `keyframes.T_WC[idx]` against the recorded clone for that same `idx`. If a
  record exists for `idx` and the live value **differs** from it, the tracker overwrote the backend's
  pose: increment `_backend_pose_clobbers` and drop that record. Use
  `torch.equal(a, b)` — an exact comparison, since the question is "was this value replaced", not
  "did it drift".
- Guard every read/write of `_backend_last_written` and the counters with `_backend_meta_lock`. Never
  hold `_backend_meta_lock` while holding `keyframes.lock` (clone first, then take the meta lock) —
  a fixed acquisition order is what keeps this deadlock-free.
- This is a **counter, not a repair**. Do not restore the clobbered pose.

---

## CHUNK 1 — the backend thread

**Module Objective.** Run `_run_backend()` on its own thread so the frame path stops waiting for
global optimization, with a config kill switch and a clean shutdown.

**Required Context/Dependencies.** None (first chunk). Contract **C1**. `threading` is already
imported (`slam_engine.py:26`).

**Target File(s).** `slam_engine.py`, `config.yaml`

**Strict Interfaces.**

1. Add `BACKEND_IDLE_SLEEP_S`, the `__init__` fields, `start_backend()`, `close()`, `_backend_loop()`
   and `_queue_depth()` exactly as in **C1**.
2. Make the `process()` backend call conditional exactly as in **C1**. Change nothing else in
   `process()` in this chunk.
3. `config.yaml`, under the existing `perception:` section:
   ```yaml
   backend_async: true   # session 64: run MASt3R-SLAM's global optimization on its own thread
   ```
   with a comment carrying the evidence (64.4 % of the loop; upstream runs this in a separate
   process; `false` restores today's inline behaviour exactly).

**Prohibited.** Do not modify `_run_backend`, `_relocalization`, `process()`'s tracking/INIT/RELOC
branches, the tracker, or anything under `third_party/`. Do not add locking around `_run_backend` —
it already takes what it needs. Do not cap or drain the task queue. Do not start the thread from
`__init__` (the caller starts it — CHUNK 3).

**Acceptance Tests.** Add `_self_test_backend_thread()` to `perception_worker.py`, wired into
`run_self_test` in the established `ok = ...; print PASS/FAIL; assert ok` style. **No CUDA:** build
the engine with `object.__new__(slam_engine.SlamEngine)` and set only the attributes the thread body
touches, with a fake `states` exposing `lock`, `global_optimizer_tasks`, `get_mode()`, `is_paused()`
and a monkeypatched `_run_backend` that records calls.

- `BACKEND_IDLE_SLEEP_S` is a positive float under 0.05.
- `start_backend()` returns `False` and starts nothing when `backend_async` is False.
- `start_backend()` returns `True` and the thread is alive when `backend_async` is True; a second
  call is a no-op that still returns `True` and does not start a second thread.
- With three queued tasks, the fake `_run_backend` is called at least three times within a short
  bounded wait, and the loop keeps running afterwards (it polls, it does not exit on an empty queue).
- `close()` stops and joins the thread (`is_alive()` False afterwards); `close()` on an engine that
  never started is a no-op and does not raise; a second `close()` does not raise.
- A `_run_backend` that raises sets `backend_failed` to a non-empty string, stops the thread, and
  does **not** re-enter `_run_backend` afterwards.
- The thread is a **daemon**, so a hung join can never prevent process exit.

Run: `venv\Scripts\python.exe perception_worker.py --self-test`

---

## CHUNK 2 — backend telemetry and the clobber counter

**Module Objective.** Make the backend's state and the one accepted race visible per frame, since
`backend_ms` is now `0.0` and can no longer report either.

**Required Context/Dependencies.** CHUNK 1's thread and fields (already applied on disk). Contracts
**C2**, **C4**.

**Target File(s).** `slam_engine.py`

**Strict Interfaces.**

1. Add the four `SlamResult` fields from **C2**, appended after `tracker_ms`.
2. Add a module constant immediately after `SLAM_TRACK_PHASE_FIELDS`:
   ```python
   # Session 64: backend-thread state carried on every SlamResult. NOT phase timings -- two of the
   # four are not durations at all -- so they are deliberately kept out of SLAM_PHASE_FIELDS and
   # SLAM_TRACK_PHASE_FIELDS, whose members close against slam_ms and track_ms respectively.
   SLAM_BACKEND_FIELDS: tuple[str, ...] = ("backend_mode", "backend_queue_depth",
                                           "backend_thread_ms", "backend_pose_clobbers")
   ```
3. In `_backend_loop`, time each solve into `_backend_thread_ms`, increment `_backend_solves`, and
   record `_backend_last_written` per **C4**.
4. In `process()`, implement the clobber check per **C4** in the TRACKING branch only, and pass all
   four fields into the `SlamResult(...)` call after `tracker_ms=`.

**Prohibited.** Do not restore a clobbered pose. Do not add these to `SLAM_PHASE_FIELDS` or
`SLAM_TRACK_PHASE_FIELDS`. Do not change `backend_ms`'s meaning or its stamps.

**Acceptance Tests.** Extend `_self_test_backend_thread()`:

- `SLAM_BACKEND_FIELDS` is exactly the four names, and is disjoint from both existing phase tuples.
- A `SlamResult` built with only the pre-session-64 arguments has `backend_mode == "SYNC"`,
  `backend_queue_depth == 0`, `backend_thread_ms == 0.0`, `backend_pose_clobbers == 0`, with the
  right types.
- `backend_mode` resolves `"ASYNC"` with a live thread, `"SYNC"` with none, and `"FAILED"` whenever
  `backend_failed` is set — assert `"FAILED"` wins even while a thread object still exists.
- **Clobber accounting, with fake tensors** (`torch` is importable without a GPU; use small CPU
  tensors): a recorded write that is then *changed* counts exactly one clobber and clears the record
  (so a second check does not double-count); a recorded write that is *unchanged* counts zero; no
  record for that index counts zero.

Run: `venv\Scripts\python.exe perception_worker.py --self-test`

---

## CHUNK 3 — wire it into perception, the CSV and the console

**Module Objective.** Start and stop the thread with the pipeline's own lifecycle, and get the four
fields onto the per-frame record and the live console.

**Required Context/Dependencies.** CHUNKS 1-2. Contracts **C2**, **C3**.

**Target File(s).** `perception_worker.py`

**Strict Interfaces.**

1. `Pipeline.__init__` reads `bool(cfg["perception"].get("backend_async", True))` and passes it as
   `slam_engine.SlamEngine(conf_thresh=conf_thresh, backend_async=...)`, then calls
   `self.slam.start_backend()`. Print one startup line naming the resolved mode — the operator must
   be able to tell from the console which mode a flight ran in.
2. Add `Pipeline.close_slam()` calling `self.slam.close()`, and call it from `run_live`'s existing
   `finally` **before** `pipe.close_diag()`, so the thread is stopped before the logs close.
3. Extend `DIAG_PERF_FIELDS` per **C3** (append only), and the `diag_perf.row(...)` call at the end
   of `step()` with the four keywords read from `res` (`backend_thread_ms=round(res.backend_thread_ms, 1)`;
   the other three verbatim). Keep every existing keyword unchanged.
4. Extend the 1 Hz console line with one segment after the session-63 `(frm … inf … trk2 …)` segment:
   ```python
   f"bk[{res.backend_mode} q{res.backend_queue_depth} {res.backend_thread_ms:.0f}ms"
   f"{f' clob{res.backend_pose_clobbers}' if res.backend_pose_clobbers else ''}] | "
   ```
   Keep every existing segment in its existing order. This one reads from `res` directly, **not**
   from the sticky `_last_phase_ms` dict — it is per-frame truth, not a console convenience.

**Prohibited.** Do not reorder or rename existing CSV columns. Do not start the thread anywhere but
`Pipeline.__init__`. Do not swallow a `close()` failure.

**Acceptance Tests.** Extend `_self_test_phase_timing(cfg)` (do not add a new function):

- `DIAG_PERF_FIELDS[:20]` is unchanged from the session-63 twenty, in order.
- All four names in `slam_engine.SLAM_BACKEND_FIELDS` appear in `DIAG_PERF_FIELDS`, and
  `len(set(DIAG_PERF_FIELDS)) == len(DIAG_PERF_FIELDS)`.
- A `DiagLog` round-trip in a `tempfile.mkdtemp()` directory (cleaned up in a `finally`) with one
  fully-populated row yields a header equal to `list(DIAG_PERF_FIELDS)` and the exact values written,
  **including `backend_mode` round-tripping as the string `"ASYNC"`**, not a number.
- A row written with the four omitted yields blank (`""`) cells, not `"0"`/`"SYNC"`.
- The console segment renders for a `SimpleNamespace` stand-in in both `FAILED` and `ASYNC` modes,
  contains `"bk["`, and shows `clob` **only** when the count is non-zero.

Run: `venv\Scripts\python.exe perception_worker.py --self-test`

---

## CHUNK 4 — report the backend's state

**Module Objective.** Let the timing report say which mode a flight ran in and how the backend
behaved, without pretending non-durations are phase timings.

**Required Context/Dependencies.** CHUNK 3's columns. Contract **C3**. **Import neither**
`slam_engine` nor `perception_worker` — this tool stays pure stdlib so it runs with no venv and no
torch. Re-declare names locally, as the module already does for the other two mirrors.

**Target File(s).** `perception_timing_report.py`

**Strict Interfaces.**

1. Append **`backend_thread_ms` only** to `PHASE_COLUMNS`.
2. Add `_BACKEND_STATE_COLUMNS: tuple[str, ...] = ("backend_mode", "backend_queue_depth",
   "backend_pose_clobbers")` with a comment saying why they are deliberately not phase columns.
3. Add `backend_summary(rows) -> str`: one line reporting the distinct `backend_mode` values with
   their row counts, the median and max `backend_queue_depth`, and the final (maximum)
   `backend_pose_clobbers`. Absent columns → the exact string
   `"backend state: unavailable (file predates session 64)"`. Blank/unparseable cells are counted and
   named, never coerced.
4. `report()` prints that line immediately under the existing track-closure line.

**Prohibited.** Pure stdlib only. Do not add the three state columns to `PHASE_COLUMNS`. Do not change
`phase_closure`, `track_closure`, `PhaseStats`, or any existing signature.

**Acceptance Tests.** Extend `run_self_test()`, building synthetic CSVs with `csv.DictWriter` in a
`tempfile.mkdtemp()` directory cleaned up in a `finally`:

- A modern CSV with mixed `backend_mode` values yields a summary naming each mode with its count.
- Queue depth median/max are computed over parseable rows only; blanks are counted, not zeroed.
- `backend_pose_clobbers` is reported as the **final/maximum** value (it is cumulative, so a mean or
  median would be meaningless).
- A pre-session-64 CSV yields exactly the `"unavailable (file predates session 64)"` string, and
  `report(path)` **still prints both closure lines normally** — one missing feature must not suppress
  another.
- `PHASE_COLUMNS` contains `backend_thread_ms` and does **not** contain `backend_mode`.
- Every session-62/63 test still passes unchanged.

Run: `venv\Scripts\python.exe perception_timing_report.py --self-test`
and `python perception_timing_report.py --self-test` (bare system interpreter, no venv, no torch).

---

## CHUNK 5 — surface it in the visualizer

**Module Objective.** Put the backend's state in front of the operator in flight, as `CLAUDE.md`
rule 3 requires for any degraded-path state.

**Required Context/Dependencies.** CHUNKS 2-3. `_map_payload` (`perception_worker.py`) already
carries `tracking_mode` on `TOPIC_MAP`, and the visualizer already renders a status strip.

**Target File(s).** `perception_worker.py`, `visualizer.py`

**Strict Interfaces.**

1. `_map_payload` adds `"backend_mode": res.backend_mode`, `"backend_queue_depth":
   res.backend_queue_depth`, `"backend_pose_clobbers": res.backend_pose_clobbers`.
2. `visualizer.py` gains `backend_status_text(map_payload) -> str`, returning `""` when the payload
   carries no `backend_mode` (an older flight or an older perception), otherwise a compact
   `bk=ASYNC q2` / `bk=ASYNC q2 clob7` / `bk=FAILED`.
3. Render it in the existing top status strip. **`FAILED` renders red**; `ASYNC`/`SYNC` render in the
   strip's normal colour. A non-zero clobber count is shown but is **not** an alarm colour — it is an
   expected, self-correcting event, and colouring it red would train the operator to ignore red.

**Prohibited.** Do not add a panel or change the dashboard layout — `CANVAS_W`/`CANVAS_H`
(`visualizer.py:73-74`) must not change. Do not render anything when the field is absent.

**Acceptance Tests.** Extend `visualizer.py`'s `run_self_test()` in its `case(...)` style:

- `backend_status_text({})` and `backend_status_text(None)` both return `""` (older flights).
- `ASYNC` with depth 2 renders a string containing `"ASYNC"` and `"q2"` and **no** `"clob"`.
- A non-zero clobber count adds `"clob"`; `FAILED` renders `"FAILED"`.
- Rendering a `FAILED` payload puts red in the status strip; rendering an `ASYNC` payload with a
  non-zero clobber count puts **no** red there — crop to the strip rows before testing colour, the
  way the session-62 recovery-row test does.

Run: `venv\Scripts\python.exe visualizer.py --self-test`

---

## POST-RUN (operator, not Sonnet)

1. **Bench before flying.** `venv\Scripts\python.exe perception_worker.py --video <recorded mp4>
   --no-display --log --max-frames 60`, then `venv\Scripts\python.exe perception_timing_report.py`.
   Confirm `backend_mode=ASYNC`, `backend_ms` now `0.0`, `backend_thread_ms` non-zero, both closure
   residuals still near 0, and note the clobber count.
2. **Fly once.** Success looks like: `slam_ms` no longer tracking `backend_ms`; the per-frame cadence
   holding near `track_ms` (~2 s) instead of spiking to 20-40 s; `HOLD_LOST` + `FALLBACK` time falling
   sharply. Watch `q` in the console — a queue depth that climbs monotonically means the backend
   cannot keep up, which is information, not a bug.
3. **Then decide the next target with real numbers:** `tracker_ms` (85 % of TRACKING's `track_ms`, and
   it grew 405 → 5177 ms over one flight — still unexplained), `infer_ms` (99 % of RELOC's), or
   Stage B if the autopilot is still starved despite a steady cadence.

## Closing steps (non-negotiable, per CLAUDE.md)

1. Fold the outcome into `PROGRESS.md`'s narrative and refresh `STATE.md`'s resume pointer; archive
   this spec as `plans/session64-spec.md`.
2. Leave the tree self-describing for a cold resume from `STATE.md` alone.
