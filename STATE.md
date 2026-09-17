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

### Session 69 is DONE and committed (2026-09-17). Nothing is mid-flight.
What it closed (numbers and narrative: `PROGRESS.md` session-69 log entry + `Measured numbers`):
- GPU thermal cliff — fans; peak 96 -> 75-83 C, `trk_pre_ms` flat.
- VRAM paging — `torch.cuda.empty_cache()` after each backend pass; **confirmed on four flights**
  (reserved 6.6 -> 7.7 GB, spill flat). One unflown case: a 10+ min flight that never loses tracking.
- Relocalisation — instrumented (`slam_reloc_stats.py`, `reloc_*` CSV columns, a console line per
  attempt), measured, and the acceptance rule changed on the evidence: `reloc_strict: false`,
  `reloc_min_match_frac: 0.35` in `config.yaml` (over upstream's strict/0.3).
- Loss-recovery holds — every hold-still while lost is decided by reloc evidence
  (`reloc_hold.py`); TRIM is blocked on an unconfirmed re-lock. Last flight: 118 keyframes, three
  losses, three recoveries in <= 10 s each.

### >>> NEXT: pick the next item from the triage table (`PROGRESS.md` -> `## Future (backlog)`). <<<
Recommendation, in order:
1. **L** — TRIM oscillation (ended the last flight in STUCK; DOWN pulse overshoots into the UP band).
   Small, and the only thing that went wrong on an otherwise clean flight.
2. **A1** — fly the bounded window `ON W=30`. The unbounded backend is now the largest per-frame
   cost (2 -> 13.7 s over 100 keyframes; the map reached 118).
3. **M/G** — the planner's premature "mission complete" (its reasons must first ride the timeline).

Standing watch (closes on the next flight): `gpu_probe.py --report` -> `t_resv` stays within ~1 GB
of `t_alloc`; the perception console's `[slam] reloc acceptance: strict=False min_match_frac=0.35`
line is present; FALLBACK event lines name why each hold ended.

### Standing habits
- **Run `venv\Scripts\python.exe gpu_probe.py` alongside every flight**; `--report` joins GPU
  state + torch's memory split to `trk_pre_ms`. HWiNFO64 only if the clock leaves 780 MHz.
- Forced-loss test: fly ~3 min, force a loss, hold still on mapped ground, watch the `RELOC attempt`
  lines and the FALLBACK hold reasons.

## Everything else
There is no second live item. Open work, watch lists, deferred designs and the measured numbers all
live in `PROGRESS.md`:
- `## Future (backlog)` — the **TRIAGE TABLE** (A-N, with session 69's update line), then every
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
