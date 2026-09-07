# Cartographer — State (read this first)

**This file holds the ONE thing we were last working on.** Everything else — full history, every open
issue, the backlog, the measured numbers — is in `PROGRESS.md`. Per-session technical design is in
`plans/*.md`. If it is not the live item, it does not belong here.

## What this project is (one paragraph — the full version is in `PROGRESS.md`)
Assessment task: autonomously map the black-box **XLAB** Unity sim from one monocular drone feed and
report a target object's 3D location with an uncertainty. Five Python processes over a ZMQ bus,
launched by `python fly.py`, on an RTX 3080 Laptop **shared with the sim** — which turns out to matter
more than anything else (see below). Grading is internal consistency; metric scale and compute
efficiency are not graded. Full task statement, architecture, run procedure and the key technical
facts (control mechanic, build quirks, world-frame convention): `PROGRESS.md`'s
`## What this project is`, `## Architecture`, `## What's built`, `## Reference — don't re-derive`.

Branch **`all-bets-are-off`**.

## Current status

### >>> SOLVED (session 68): THE "ACCUMULATOR" IS A THERMALLY THROTTLED GPU. <<<
**It was never an accumulator, and it was never in our code.** Measured by `gpu_probe.py` on the
manual flight of 2026-09-07 (`OUTPUT/diag/20260907_122008_gpu.csv`):

| t | sm clock | temp | throttle reason |
|---|---|---|---|
| 17 s | **1725 MHz** | 73 C | none |
| 54 s | 780 MHz | 79 C | **SwThermalSlowdown** |
| 123 s | 450 MHz | 89 C | SwThermalSlowdown |
| 146 s -> end | **210 MHz** | 88-96 C | SwThermalSlowdown |

`SwThermalSlowdown` active in **235 of 247 samples**; median clock **210 MHz against a 2100 MHz
ceiling — 10% of rated**; peak 96 C. The old headline ("steps up 7-17x within two minutes of takeoff,
on every flight") **is the GPU's thermal time constant, not takeoff** — the operator's parked first
phase went 350 -> 2000 ms *without the drone moving*.

Session 67's instrumentation is what made this findable and it closed its own question too: **GN
iterations are FLAT** all flight (median 4-7; one step, 1 -> 6 at t=22 s when the drone first moves,
because a stationary camera converges in one step). **The warm-start `idx_f2k` feedback loop is dead
as a hypothesis.** What grew was `trk_pre_ms` 341 -> 3024 ms and normalised solver cost 13.1 -> 138.7
ms/iter/100k pts (10.6x) — two workloads sharing no code, no kernels and no data, moving in lockstep
(r = +0.878) and recovering together, while CPU `frame_ms` stayed 4.8 -> 6-9 ms.

**Still unexplained: ~1.9x of the ~7x.** Cycle-count test (time x clock = cycles, constant for fixed
work): **257M** at 600-1000 MHz, **263M** at 300-600 MHz, **482M** below 300 MHz. Untested candidates:
VRAM spill (rose +210 MB; peak 10.2/16.4 GB), Unity's rising share, sub-300 MHz cliffs.

### >>> IMMEDIATE NEXT — a 10-minute THERMAL experiment, before any more code. <<<
The GPU **idles at 64-65 C** with nothing running (should be 35-45 C) and draws only **~50 W at 96 C**
(that chip sustains 80-150 W). The cooling is not removing 50 watts — dust, thermal paste, blocked
intake, a soft surface, or a fan curve.

1. Raise / actively cool the laptop, clean the intakes.
2. Start `venv\Scripts\python.exe gpu_probe.py`, launch, and fly **just the 2-minute park**.
3. `venv\Scripts\python.exe gpu_probe.py --report` — watch **where `SwThermalSlowdown` engages** and
   what the clock floor becomes. Nothing else needs to change to know whether it worked.

Potential payoff up to **8x** — more than every optimisation session combined. **Sessions 63-66
optimised a GPU running at one tenth of its clock**: their gains are real (they were order-controlled)
but the absolute magnitudes are not what healthy hardware would show, so re-measure anything
load-bearing once the thermals are fixed. Self-reinforcing in our favour: less GPU work -> less heat
-> higher clock.

### Two things from session 68 that are now standing habits
- **`gpu_probe.py` (repo root)** — standalone, pure stdlib, imports nothing from the project and does
  no GPU work of its own, so it cannot perturb what it measures. Global state from `nvidia-smi`
  (clocks, temp, power, **throttle reason bits**); per-process VRAM and 3D/Compute engine share from
  Windows performance counters, because `nvidia-smi --query-compute-apps` reports per-process memory
  as `[N/A]` under WDDM. Rows carry `wall_ts` on `time.time()`, so `--report` joins to a flight CSV
  with no offset arithmetic and prints GPU state beside `trk_pre_ms`. Self-test:
  `gpu_probe.py --self-test`. **Run it alongside every flight** — one extra terminal, and it is the
  only thing in this stack that can see the GPU.
- **The park 2 / fly 4 / park 2 / fly 4 manual profile** is what made this decisive: a leak cannot
  un-leak while parked, so recoveries discriminate causes a monotone autonomous trend never could.
  Use it for any future "does X accumulate?" question.

## Everything else
There is no second live item. Open work, watch lists, deferred designs and the measured numbers all
live in `PROGRESS.md`:

- `## Future (backlog)` — opens with a **TRIAGE TABLE the operator has not yet decided on**
  (one row per group, with a suggested order); then every open item, grouped A-I. **Read this before
  picking up anything new.** It opens with the items moved out of this file in session 68 (the
  bounded window's remaining decisions, the unreviewed autopilot watch lists, housekeeping, the
  `main`-branch HEIGHT issue, and the three parked choke cures).
- `## Reference — don't re-derive` → `### Measured numbers — don't re-measure` — the choke
  attribution, the `track_ms` split, the historical degradation table, the bounded-window ABBA
  results, and the two measurement traps (run-order confound; no fixed `--stride` replay reproduces
  live flight).
- `## Session Log (newest first)` — the narrative of what was tried and why.

## Standing rules
`CLAUDE.md` carries the durable rules (NO SILENT FALLBACKS, image-integrity guardrail, NO
MANUAL-FLIGHT DATA LEAKAGE into autonomy limits, the PROGRESS.md/STATE.md update-then-commit-then-push
rule, task-list discipline) — read it, don't re-derive it here. One live exception on record: branch
`all-bets-are-off` session 44 hardcoded two TRIM `pos_y` thresholds, an explicit operator-approved
override of the no-leakage rule, scoped to that branch only.
