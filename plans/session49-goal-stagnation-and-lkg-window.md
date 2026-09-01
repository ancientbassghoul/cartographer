# Session 49 — goal stagnation memory + the LKG debug window

## Origin

The operator ran a test flight and confirmed session 48's back-off grace is working. Separately, he
flagged the flight's tail: around `15:49:11` the drone reached a goal near a wall, tried it, bumped,
backed off — all correct — and then, at `15:50:14.315`, picked a new goal SO close to the last one
that he asked point-blank: "Haven't we fixed that like a thousand times?" and "don't we blacklist a
point after a few picks — if so, how many?"

Second, unrelated ask from the same flight: when the CV probe uses the LKG (last-known-good) frame,
open it in its own window so the operator can see what it's matching against.

## The trace

Flight `20260901_154648`. Between `15:48:47` and `15:50:52` the drone launched **five separate legs
at the same physical wall**, and only retired it at `15:57:27` — almost 8 minutes and ~40 plan losses
after the first bump.

**The answer to "how many picks":** there is no such single number. Three independent guards, each
with a second condition beyond the pick count:

| guard | threshold | extra condition | this flight |
|---|---|---|---|
| 2-BUMP | 2 bumps | two *physical* contacts within `goal_assoc_dist` | bump #1 `15:49:32`, bump #2 `15:57:15` → blacklisted `15:57:27` |
| LOOP | 3rd pick arms it (`goal_loop_min_picks: 2`) | **all** pick-time drone positions inside one `goal_loop_pos_dist` (1.0u) cluster | picks reached 3, but picked from spots 2.5u apart → vetoed |
| STALL | 2 strikes (`goal_strike_limit: 2`) | needs a *judged* hop | **zero strikes the entire flight** |

All three were structurally unable to count:

1. **The STALL guard never got a single data point.** `_enter("HOLD_LOST"/"SLAM_HOLD")`
   (`autopilot.py:2280`) clears `_hop_start_goal`, so a hop interrupted by a plan-loss is never
   judged — no strike, no reset, nothing. Whole-flight tally: **7 `[HOP_BASELINE]`, 2 `[HOP_JUDGE]`**
   — five hops evaporated before REPLAN could score them.
2. **The LOOP guard's history was split across two records.** The commitment tracks a live frontier
   centroid at `goal_assoc_dist` (1.0u) and re-snaps to it every tick (`_choose`,
   `frontier_planner.py`); the goals-DB disc a pick lands on is matched by `_db_entry` against a
   center **frozen at its creation point**, radius `goal_area_radius` (0.5u). One physical wall
   walked `[-2.05, 2.303] → [-2.4, 2.62] → [-2.5, 2.7]` (0.60u total drift) and ended the flight as
   **two discs**: `picks=3, bumps=1` and `picks=2` — 5 picks and 1 bump on one wall, neither record
   reaching its own threshold.
3. **Per-hop "progress" was a lie.** Every leg's *end-of-leg* distance read as closer than its start
   (3.85 → 2.99 → 1.68 → 3.55 → 2.21) because the SLAM pose jumped backwards between legs (the drone
   physically at `[-0.81, 2.08]`, then `[1.04, 1.41]`, then `[-0.44, 1.91]` — never continuously
   tracked across the intervening losses). Only a **best-ever** test sees the truth: the drone never
   got closer than 1.68u after leg 3.
4. `_best_dist` — the field whose own docstring already said "closest distance achieved toward the
   current committed goal (round-blacklist memory)" (`frontier_planner.py:92`) — **was read and reset
   in three places and written in none.** The stagnation memory was designed for and never wired up.

This is a *third*, distinct starvation class — not a regression of session 33 (blacklisted goal
re-picked through the clearance inset) or session 45 (goal the drone was standing on).

## Built

### 1. Drifting goals-DB discs (`frontier_planner.py`, `config.yaml`)

A goals-DB disc's `center` now **follows its goal** on every match, the same way the commitment
already re-snaps to a live frontier centroid — so one physical, drifting goal stays one accounting
record. Bounded: each disc gains an immutable `origin` (its first center); a candidate must stay
within `goal_disc_max_drift` (0.75u) of `origin` or a fresh disc starts instead. `goal_disc_max_drift
< goal_blacklist_radius` is enforced by a fail-fast `ValueError` in `__init__` (CLAUDE.md
no-silent-fallback), so a fully-drifted disc's `origin` is always still inside its own exclusion
ball once blacklisted — verified directly (`_excluded(origin)` reads True, not inferred from the
budget alone).

On this flight: `[-2.05, 2.303]` and `[-2.5, 2.7]` become **one** disc — 5 picks, 1 bump.

### 2. The `_best_dist` stagnation memory, finally wired up (`frontier_planner.py`, `config.yaml`)

`_select_reachable`'s commit site now writes the running-min `_best_dist` — the write that never
existed. `register_goal_pick` gains a stagnation test alongside the existing loop guard: a disc's
`best_ever`/`stagnant_legs` track whether the running best ever improved by more than
`goal_progress_eps` (0.2, an existing knob) since the last pick; `goal_stagnant_limit` (2) consecutive
non-improving legs → permanent blacklist, reason `"stagnant"`, through the same `_db_blacklist` store
every other mechanism uses. Indifferent to *why* a leg ended — a leg killed by a plan-loss now costs
the goal something, closing the exact gap the STALL guard's dead hops left open.

On this flight (once the disc-drift fix above unifies the record): legs close to 3.85 / 2.99 / 1.68 —
each improving, `stagnant_legs` stays 0, `best_ever` settles at 1.68 after leg 3. Legs 4 and 5
(3.55, 2.21) beat nothing → `stagnant_legs` reaches 2 → **blacklisted at leg 5, `15:50:38`**, instead
of the real flight's `15:57:27`. Seven minutes and one 18-cycle FALLBACK sweep saved.

Deliberately **not** built: an unconditional "N picks and you're dead" cap — a legitimate march to a
far goal accrues one pick per judged hop and would be killed by it. Stagnation (best-ever, not raw
pick count) is the discriminator that survives both a marching approach and a pose-jumping loss storm.

### 3. LKG debug canvas (`visual_recovery.py`)

`VisualRecoveryProbe.match()` gains `debug: bool = False` and `banner: str | None = None` (both
keyword-optional, every existing call site unaffected, zero cost when off). With `debug=True`,
`VisualMatch.debug_image` is populated on **every** return path that has an F_LKG — including every
failure path (no descriptors, <4 good matches, degenerate homography, below-threshold match) — since
seeing exactly where and why a match failed is the point of the window. `_compose_debug` builds
F_LKG (left) | live (right), with the RANSAC inlier correspondences drawn via `cv2.drawMatches` when
a homography survived, or a zero-padded `hstack` fallback otherwise, plus a two-line banner. Canvas
width is always exactly `w_lkg + w_live`; nothing is ever resized, cropped, or scaled (CLAUDE.md image
integrity) — proven by an exact-width self-test assertion, not just eyeballed.

### 4. The window, saved evidence, and the replay panel (`autopilot.py`, `config.yaml`, `flight_replay.py`)

`visrec_debug_window` (default off) opens an OS window (`cv2.imshow`) showing the composed canvas
live; a decision instant (the first tick of a loss episode, or a `VISUAL_RECOVERY` re-match) also
saves it to `OUTPUT/diag/<flight_ts>_visrec/<HH-MM-SS_mmm>.png`, capped at `visrec_save_max` (200)
per flight. A healthy tracking tick shows the bare cached reference at a throttled refresh instead of
running SIFT. The window half and the save half fail **independently** — a dead display must not
stop the PNG evidence (the half that survives the flight), and a disk failure must not close the
window — each sets its own flag (`visrec_window_failed`/`visrec_save_failed`), logs one `CRITICAL`
line, and is never retried; both flags ride the timeline so the replay shows which half degraded.
`flight_replay.py`'s existing Visual Recovery panel now renders the saved canvas as a thumbnail
(click to open full size) plus either failure flag.

## Verified

`python frontier_planner.py --self-test`, `python visual_recovery.py --self-test`,
`python autopilot.py --self-test`, `python flight_replay.py --self-test`: **ALL PASS, 0 failures**.
`venv/Scripts/python.exe perception_worker.py --self-test`: PASS (needs `torch`, only present in the
project venv). Every new behaviour proven against its own defect by reverting on a scratch copy:

| Revert | Caught by |
|---|---|
| `_db_entry`'s two-condition match → single condition (drift check dropped) | disc-follow + budget tests fail (two discs instead of one; drift not recorded) |
| `_best_dist` write at the commit site removed | the `select()`-integration test fails (bookkeeping-only tests, which drive `_best_dist` manually to isolate them from the data source, correctly stay green — this is a *tighter* signal than a single coupled assertion would give) |
| stagnation's `stagnant_legs >= limit` blacklist call disabled | the real-flight stagnation test fails while the never-blacklisted march test stays green |
| `cv2.resize` injected inside `_compose_debug` | the exact-width no-resize assertion fails |
| the sink's two independent `try/except` guards collapsed into one shared block | both the window-failure and save-failure isolation tests fail — a dead display now also poisons the save flag, exactly the coupling independent guards prevent |

One deviation from the external review that vetted this design before it was built: the review
proposed registering **both** a drifted disc's `center` and its `origin` in the blacklist store when
retiring it. That doesn't work here — `_blacklist_goal` **merges** any entry within
`blacklist_radius`, so a second entry near `origin` would fold into the first and *move* it, covering
less than before. The design instead makes the hazard impossible by construction: `goal_disc_max_drift`
is enforced strictly below `goal_blacklist_radius`, with the fail-fast `ValueError`. The review's own
requested assertion (`_excluded(origin)` reads True after a drifted disc is retired) is still in the
suite — it just passes by geometry rather than by bookkeeping.

## Next

**LIVE-FLY.** Watch for:

- A `PLANNER: ... reason=stagnant` line retiring a wall goal after 2 fruitless legs, instead of the
  ~8 minutes / 40-loss / 18-cycle-FALLBACK grind this flight showed.
- **No legitimate far goal retired mid-march.** This is the one accepted risk of the stagnation guard
  — if a genuinely progressing leg ever gets soft-killed, `goal_stagnant_limit` is the knob to loosen
  first, not the design.
- The goals-DB debugger panel showing **one** disc with a non-zero `drift` where this flight's log
  would have shown two dead-end records.
- The LKG window's drawn inlier correspondences visibly piling onto one flat surface when the drone
  is nose-to-a-wall — and the window itself surviving a display hiccup without losing the saved PNG
  evidence (or vice versa).

## Open items (deliberately not done here)

1. **`_enter("HOLD_LOST"/"SLAM_HOLD")` still discards the pending hop judgement**
   (`autopilot.py:2280`), so the STALL guard (2 consecutive no-progress *judged* hops) remains
   structurally starved under a loss storm — this session did not touch that clearing logic.
   Stagnation now covers the same ground from a different angle (best-ever across legs, immune to
   *why* a leg ended), so the STALL guard's blindness is no longer load-bearing for this failure
   mode — but it is still dead weight worth fixing on its own. Judging an interrupted hop *late*,
   once SLAM re-settles on the same goal, needs a trusted-pose story first: this flight's SLAM pose
   jumped 1.3u between consecutive ticks, so a naive late-judge would score hops against garbage
   displacement. Its own session.
2. **The 30 pre-existing session-history entries in `PROGRESS.md` were not touched** beyond the
   "Next" resume pointer and the plan-of-record list — per the file's own rule, only the just-finished
   work gets folded into the concise narrative this session.
