# Cartographer — State (read this first)

**This file holds the ONE thing we were last working on.** Everything else — full history, every open
issue, the backlog, the measured numbers — is in `PROGRESS.md`. Per-session technical design is in
`plans/*.md`. If it is not the live item, it does not belong here.

## What this project is (one paragraph — the full version is in `PROGRESS.md`)
Assessment task: autonomously map the black-box **XLAB** Unity sim from one monocular drone feed and
report a target object's 3D location with an uncertainty. Five Python processes over a ZMQ bus,
launched by `python fly.py`, on a Lenovo ThinkPad P1 Gen 4 (i9-11950H + RTX 3080 Laptop, 16 GB)
**shared with the sim** — a thin chassis that thermal-limits both chips under this load. Grading is
internal consistency; metric scale and compute efficiency are not graded. Full task statement,
architecture, run procedure and the key technical facts: `PROGRESS.md`'s `## What this project is`,
`## Architecture`, `## What's built`, `## Reference — don't re-derive`.

Branch **`all-bets-are-off`**.

## Current status

### Where the "accumulator" stands after session 69 (2026-09-17)
The multi-session slowdown was three stacked things, all now measured (numbers: `PROGRESS.md` →
`Measured numbers` → session 68/69 tables):
1. **GPU thermal throttle — CLOSED.** Fans + cleaned intakes: peak 96 C -> 75-83 C, clock floor
   210 -> 780 MHz, `trk_pre_ms` flat at ~340 ms for 7 min. The chassis still thermal-limits the GPU
   (hot-spot limiter 65 % of a flight) and the CPU (100 C, throttling 22 %); that is hardware, and
   the remaining levers are in backlog **J**.
2. **VRAM paging — NAMED, fix approved, NOT YET WRITTEN.** The live item below.
3. **Unbounded backend window** (`backend_window_mode: OFF`): `backend_ms` 2 s -> 13.7 s over 100
   keyframes on the healthy baseline. Backlog **A1**, now the largest remaining per-frame cost.

### >>> LIVE ITEM: put `torch.cuda.empty_cache()` at the end of `_run_backend()`, then fly. <<<
**What:** on Windows/WDDM, PyTorch's caching allocator never flushes (cudaMalloc never fails, it
pages), and every global-solve pass needs a slightly bigger working block (~2.3 MB x edges) than the
last, so `cuda_reserved_mb` grows quadratically — flight 2: 6.9 -> **29.5 GB** reserved on a 16 GB
card, 15 GB paged, for 8.2 GB peak live. `expandable_segments` is Linux-only.
**Fix:** one explicit `torch.cuda.empty_cache()` after each backend pass in `slam_engine.py`
`_run_backend()`. Log it once at startup so the behaviour is visible. Not a fallback.
**Test:** next flight with `gpu_probe.py` running, then `gpu_probe.py --report` — `t_resv` should
track `t_alloc` + ~1 GB instead of climbing; `spill` should stay at its ~200 MB baseline. Closes
on that flight: confirmed -> one line in `PROGRESS.md`; broken -> its own plan.

### Standing habits (session 68-69)
- **Run `venv\Scripts\python.exe gpu_probe.py` alongside every flight**, and HWiNFO64 with
  `GPU Performance Limiters` and the CPU `DTS` section expanded (reset Min/Max before takeoff).
  `--report` joins GPU state + torch's memory split (`t_alloc/t_resv/t_peak/fg_edge`) to `trk_pre_ms`.
- The park 2 / fly 4 / park 2 / fly 4 manual profile for any "does X accumulate?" question.

## Everything else
There is no second live item. Open work, watch lists, deferred designs and the measured numbers all
live in `PROGRESS.md`:
- `## Future (backlog)` — the **TRIAGE TABLE** (A-K, with session 69's update line), then every
  open item. **Read this before picking up anything new.**
- `## Reference — don't re-derive` → `### Measured numbers` — the throttled-flight table, the two
  session-69 flights side by side, the torch memory split, the HWiNFO limiter readings, plus the
  older choke attribution and the two measurement traps.
- `## Session Log (newest first)` — the narrative of what was tried and why.

## Standing rules
`CLAUDE.md` carries the durable rules (NO SILENT FALLBACKS, image-integrity guardrail, NO
MANUAL-FLIGHT DATA LEAKAGE into autonomy limits, the PROGRESS.md/STATE.md update-then-commit-then-push
rule, task-list discipline, the PRUNE rule) — read it, don't re-derive it here. One live exception on
record: branch `all-bets-are-off` session 44 hardcoded two TRIM `pos_y` thresholds, an explicit
operator-approved override of the no-leakage rule, scoped to that branch only.
