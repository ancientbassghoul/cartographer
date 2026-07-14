# Plan — Blacklist region + counter robustness (session 14 diagnosis; build next session)

## Context
On flight `20260713_223231` the drone bounced at a glass wall for ~2 minutes (22:36:57 → 22:39:00) before it
finally blacklisted its way past. The 2-bump blacklist *worked* — it retired a region roughly every 2 s — but
two compounding failure modes stretched it into a 2-minute ordeal (confirmed from `planner_event` in the
timeline):

1. **Whack-a-mole on a dense frontier strip.** Every blacklist logged `(1 total)` — the exclusion region is
   *smaller than the spacing between frontier cells* along the glass. The planner kept handing goals ~5 cm
   apart (`[3.39,-1.32]`, `[3.40,-1.31]`, `[3.38,-1.26]`, …), each a "fresh" region needing its own 2 bumps.
   The drone chewed through ~15-20 micro-frontiers one at a time.
2. **Counter-reset thrash.** The planner also alternated between the glass corner and *distant* goals
   (`[-1.35,7.75]`, `[4.15,-3.95]`), and each swap logged `count=1/2 (RESET from prev goal … counter
   defeated)` — **209×** for one goal. So for the alternating goals the 2-bump counter never accumulated at
   all (the same "don't let a flicker reset progress" lesson as session 12, here driven by goal alternation).

## Fixes (frontier_planner.py + self-tests)
1. **Widen the blacklist exclusion region** so one blacklist covers the neighboring frontier cells along a
   wall — region-based (e.g. exclude a radius ~`stop_clearance_dist`, or a small multiple of the grid
   resolution, around a blacklisted goal), not a single cell. A blacklisted glass strip then stays dead
   instead of re-spawning a 5 cm-shifted frontier. Optional escalation: after N regional blacklists cluster
   in a small area, inflate the exclusion to the whole area (a wall is there).
2. **Per-region bump tallies** instead of a single counter that resets on every goal change: key the bump
   count by region (a coarse-grid cell), so bumping A, then B, then A **accumulates** A's count. A goal change
   no longer "defeats" the counter — only a genuinely different region gets its own independent tally.

## Self-tests (frontier_planner.py `--self-test`)
- **Dense strip retires bounded:** feed a packed strip of near-identical frontiers behind a wall; assert the
  planner retires the strip within a small, bounded number of blacklists (no whack-a-mole across dozens).
- **A/B/A alternation still reaches 2 bumps on A:** bump A (1/2), switch to a distant B, return to A → A
  reaches 2/2 and blacklists (the counter is NOT reset by the excursion to B).
- Keep the existing session-7/session-10 blacklist tests green (corners ignore the blacklist; a real wall
  retires via the fresh 2-bump).

## Verification
Offline self-tests green, then a live re-fly at the same glass wall: the drone should retire the wall in a
handful of blacklists (seconds, not minutes), with no `counter defeated` thrash in the log.
