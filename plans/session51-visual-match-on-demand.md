# Session 51 — stop recomputing a visual match nobody reads (and the SIFT behind it)

## Origin

While chasing why SLAM chokes, the operator asked what in the autopilot might be starving it, and
separately proposed: "why calculate SIFT at all before we need to use it? Store the frame every tick,
run SIFT when you need the recovery manoeuvre."

Storing-the-frame-cheaply was already the design (`update_reference` is a `.copy()`, no SIFT). But the
instinct pointed at something real one level up.

## Two negative results first — record these so they are never re-chased

1. **The autopilot is not what chokes SLAM.** Loop rate measured from contiguous timeline runs on
   flight `20260901_172217`: **38.5 Hz** through the 91.6 s SETTLE wedge, **38.6 Hz** in `SLAM_HOLD`,
   **32.0 Hz** in a SIFT-heavy `HOLD_LOST`. It never bogged down while SLAM sat at 1995 ms.
2. **Unity focus is not either.** The 5.7x swing looked decisive (SLAM ran ~350 ms while the sim was
   unfocused, ~2000 ms with it focused) and the mechanism was plausible — Unity throttles rendering
   when it loses focus, freeing the GPU. The operator tested it directly: **SLAM re-chokes about two
   frames after refocus regardless.** Ruled out.

   Supporting evidence from the log, which had already undercut it: SLAM had *already* recovered to
   ~395 ms **before** the autonomy pause at 17:27:35.98, then degraded again to 1767 ms **during** the
   pause with the autopilot doing nothing at all, then recovered again. Mean across the whole paused
   window was 734 ms, not 350 ms. The by-state correlation (`SLAM_HOLD` 1815 ms, `SETTLE` 1934 ms
   median vs `ADVANCE`/`ORIENT` ~330-750 ms) is **reverse causality**: slow SLAM is what *puts* the
   drone in those states and keeps it there.

**The SLAM choke remains undiagnosed and open.** This session changes nothing about it, and is not
expected to. It is waste removal, and should be judged only as that.

## The waste

`run_explore` gated the match on `status in (LOST/NO-PLAN/STALE) or state == "VISUAL_RECOVERY"`, so a
full SIFT + brute-force `BFMatcher` + RANSAC fired **every tick of a loss at ~32 Hz** — about **380
complete matches across session 48's 12 s grace to make ONE decision**; the longest `HOLD_LOST` run in
that flight (24.3 s) is ~780. The `[VISREC]` log line is throttled to 0.5 s, which hides it: the
*computation* was never throttled.

And `match()` recomputed SIFT on the **reference** frame on every call — F_LKG only changes in
`update_reference`, and during a loss it never changes at all (it only caches while `plan_valid`).

The consumers turned out to be exactly two, both precisely guarded, which is what made this cheap:

- `_maybe_loss_snapshot_backoff` — dispatched from **both** call sites under
  `if not self._loss_snapshot_checked:`. Once the one-shot is spent, nothing reads a match again for
  the rest of the episode.
- `_step_visual_recovery`'s `MATCH` phase — one match per turn cycle, and its docstring demands the
  **freshest** one.

Everything else touching `visual_match` (timeline record, debug canvas) is observability and already
handles `None`.

## Built

### Gate A — exact, zero staleness
New `ExploreController.wants_visual_match()`: `(not _loss_snapshot_checked) or _visrec_phase == "MATCH"`.

**The trap, documented in its caller contract:** `_loss_snapshot_checked` starts `False` at
construction and is only re-armed at a fresh loss edge, so it reads `False` throughout healthy flight
before the first loss. Using the predicate *alone* would start running SIFT on **every healthy
tracking frame** — the exact opposite of the goal. It is an **AND-narrowing** of the original status
condition, never a replacement.

### Gate B — bounded staleness
New `visrec_match_min_interval_s` (0.5 s). While a consumer is live but the drone holds still against
a frozen F_LKG, the answer cannot change, so the memoised `VisualMatch` is reused. Four conditions
force a fresh compute: the **loss edge** (evidence must be from this episode), the probe's **MATCH
phase** (it re-matches *after a turn* — a pre-turn result is a different view entirely), **any
commanded motion** since the memo (a mid-loss `BLIND_BACKOFF` reverse changes what the camera sees),
and **a replaced F_LKG**.

This leans on session 48's own stated invariant — "the drone is holding still throughout, which
preserves the 'nothing has moved since the snapshot was taken' invariant this whole check is built
on" — so it memoises an unchanged value rather than serving a stale one.

Both gates live in a pure, unit-testable `_visrec_should_match(...)` helper, because `run_explore`
itself is never entered by the self-test suite.

### Lazy reference SIFT
`_lkg_feats` on the probe: computed on the first `match()` after a new reference, reused after.
`update_reference` clears it and now **returns `True` when it actually stored**, so `run_explore` keys
Gate B's invalidation off that edge instead of duplicating the condition.

**Trap:** `_keypoints` legitimately returns `(kp, None)` for a featureless frame, so the memo flag is
the *tuple slot* (`if self._lkg_feats is None:`) — keying on the descriptors would recompute forever
on a blank reference. Covered by its own test.

### Debug window
A skipped match now falls through to session 49's existing idle-refresh path (bare cached F_LKG, no
SIFT), with the banner stating the real reason so it can never claim "tracking OK" mid-loss.

## Verified

`visual_recovery.py`, `autopilot.py`, `flight_replay.py`, `frontier_planner.py` self-tests: **ALL
PASS**; `perception_worker.py` (project venv): PASS.

The headline assertion, counted: a simulated 12 s held-still loss at 32 Hz now costs
**24 real matches over 384 ticks** (was 384).

Every new behaviour proven against its own defect on a scratch copy:

| revert | caught by |
|---|---|
| GATE A dropped | `spent` controller with no/stale memo now matches — `healthy-flight trap=False` |
| GATE B rate-limit dropped | grace-window cost returns to **384/384 matches** |
| force conditions dropped | `all 3 force conditions=False` |
| reference-SIFT memo dropped | keypoint passes go 2→6 over three matches; the featureless case 2→4 |

**One weak test of mine was caught by this exercise and fixed:** the first Gate A assertion used a
*fresh* memo, so Gate B's rate-limit returned `False` anyway and masked the missing gate — the revert
passed when it should have failed. Re-asserted under conditions where Gate B would otherwise say yes
(`memo=None` and an aged memo), which isolates Gate A properly.

## Next

**LIVE-FLY** (no behaviour change expected). During a loss, `[VISREC]` lines should appear at roughly
the memo cadence rather than being throttle-limited; the `HOLD_LOST` loop rate should sit nearer
38 Hz than 32 Hz; and every decision in the log (grace notice, back-off, probe verdicts) must be
**identical** to previous flights. If a decision changes, the memo is serving something it shouldn't —
suspect the motion guard first.
