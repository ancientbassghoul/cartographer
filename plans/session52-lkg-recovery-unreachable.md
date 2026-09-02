# Session 52 — LKG visual recovery was structurally unreachable, TRIM starved under slow SLAM,
# a stale gate raised false alarms, and a permanently-blacklisted corner still got flown into

Diagnosed off flight `20260901_222552`. Branch `all-bets-are-off`. Built as a 10-chunk Sonnet-ready
implementation spec (contract-first: exact signatures in one shared section, chunks independently
reviewable/revertible). Chunks 1-9 are code; this file + the `PROGRESS.md` update are chunk 10.

## 1. The five operator-reported symptoms

1. The 15°-turn visual-recovery probe (session 36/42) had **never once executed a real MATCH on any
   real flight**, across every flight logged since it was built — the operator kept asking why the
   `[VISREC]` log never showed a probe turn actually happening.
2. Height kept drifting off-band for a full minute or more with TRIM never firing to correct it,
   even though the sag/high thresholds were clearly crossed.
3. A back-off (`BACKOFF`, full reverse for `backoff_hold_s`) sometimes threw the drone much farther
   than the situation called for — one push moved it ~1.3 u and left it far past a comfortable
   clearance margin.
4. A "LOSS-INSTANT BACK-OFF SUPPRESSED" notice sometimes fired with nothing behind it — no back-off
   had actually been attempted, evidence or not.
5. A sweep corner that had already been proven unreachable (its containing region permanently
   blacklisted) got flown into and hammered for minutes before a fresh 2-bump finally killed it —
   the exact "endless reselect" class the loop-blacklist exists to prevent, now happening through
   the corner-sweep's separate code path instead.

## 2. The four stacked LKG/visual-recovery defects (D1-D4)

All four had to be true simultaneously for the probe to be unreachable; fixing any three still
leaves it dead. Diagnosed together, fixed together in chunks 4 (D1-D3) and 5 (D4).

- **D1 — grace pre-emption.** `_maybe_loss_snapshot_backoff`'s one-shot flag
  (`_loss_snapshot_checked`) used to be set to `True` at the very top of the function, before the
  loss-recovery-grace check (session 48, `loss_backoff_grace_s`) ever ran. So the very first tick of
  a loss — even one that immediately deferred into the grace window with no evidence yet examined —
  permanently spent the one-shot. By the time the loss aged into `PLAN-STALE` (the probe's only
  entry point per session 42), the one-shot was already gone and the visual-recovery hand-off
  (Step 2c) could never fire. Fixed by moving `self._loss_snapshot_checked = True` to AFTER both the
  grace check and the post-backoff resolve gate (chunk 3's reordering), so the flag is only spent
  once the evidence has actually been read.
- **D2 — the 57/58 PLAN-LOST one-shot.** 57 of this flight's 58 loss episodes opened as `PLAN-LOST`,
  not `PLAN-STALE` — and `_maybe_loss_snapshot_backoff` is invoked (and spends the one-shot) for
  ANY status, `PLAN-LOST` included, per its own docstring ("Fires for ANY status" on Steps 1/2). So
  even with D1 fixed, the shared one-shot was almost always spent on the opening `PLAN-LOST` tick,
  starving the `PLAN-STALE`-only tail hand-off into the probe before it ever got a real shot. Fixed
  by giving the probe its OWN separate latch, `_visrec_probe_armed` (set on every fresh loss
  episode, cleared on a confirmed recovery or on a genuine back-off), polled LATE from `_step_stale`
  via `_maybe_enter_visual_probe` — independent of whatever the shared one-shot already decided.
- **D3 — the missing PLAN-LOST dispatch.** Even once the probe entered `VISUAL_RECOVERY`, the
  top-level `PLAN-LOST`/`NO-PLAN` branch of `step()` had no case for it — the same structural bug
  session 46 had already found and fixed for `FALLBACK`. Any `PLAN-LOST` flip one tick after entry
  (near-universal, given how loss episodes flicker) forced the state straight back to `HOLD_LOST`,
  discarding the in-progress probe before it ever reached its MATCH phase. This flight's one
  `VISUAL_RECOVERY` entry (`22:59:04`) was killed exactly this way 1.5s later (`22:59:06.265`).
  Fixed by adding an `st == "VISUAL_RECOVERY"` case to the PLAN-LOST branch, mirroring the
  session-46 `FALLBACK` case, dispatching to `_step_visual_recovery` so an in-flight probe survives
  the flicker.
- **D4 — F_LKG capture lag.** Even with D1-D3 fixed, F_LKG (the frozen reference frame the probe
  matches against) was cached as "whatever frame was live when `plan_valid` last ticked true" — not
  the frame SLAM's plan was actually computed FROM. At this flight's measured SLAM solve latency
  (median ~2s, worst 14.9s), those can be many frames and real seconds apart, so the "frozen
  reference" was silently drifting relative to the pose it was supposed to represent. Fixed in
  chunk 5 by keeping a ring of trailing `(frame_id, frame)` pairs (`visrec_lkg_ring_len`, 160 frames
  ≈ 16s of history) and resolving F_LKG by looking up the plan's own `frame_id` in it — identity, not
  freshness. See §5 below for why this is the general fix and not a workaround.

## 3. The chunk-3 false-alarm finding (a fifth, independent defect)

Distinct from D1-D4 above — this one produced a wrong NOTICE, not a missed probe entry. The
post-backoff SLAM re-solve gate (`_backoff_resolve_since`, session 47) used to sit ABOVE the
evidence clauses in `_maybe_loss_snapshot_backoff` and fire unconditionally: at `22:49:51.742` the
drone had already backed off to a clear reading (forward clearance `0.975 -> 1.625`, past
`stop_clearance_dist` 1.25), so `_would_react` was `False` and no back-off was ever actually being
contemplated — yet the old ordering spent the one-shot and printed "LOSS-INSTANT BACK-OFF
SUPPRESSED" anyway, on nothing. Worse, that false suppression swallowed the WHOLE episode's
evaluation (the cached-clearance check, the F_LKG visual check, AND the `VISUAL_RECOVERY` hand-off,
all of which live below the gate) for the rest of `backoff_resolve_budget_s`. Fixed in chunk 3 by
gating the resolve check behind `_would_react`: an evidence-free loss now falls straight through to
the visual-recovery hand-off instead of being silently parked. Four self-test cases
(`52-gate-1..4`) cover: no-evidence hand-off reached with no false alarm; real evidence still
suppressed with the one-shot left ARMED (not spent); the suppression notice not repeating every
tick; and the budget-timeout path still firing LOUDLY and letting the deferred back-off through.

## 4. TRIM starvation (chunk 2)

`TRIM`'s entry trigger carried an `and not self._slam_slow` conjunct — added at some point as a
"don't fight SLAM while it's already struggling" guard, but it made TRIM unreachable for exactly
the case it exists to correct. Flight `20260901_222552` logged **2749 consecutive SETTLE/ADVANCE
ticks with `slam_ms >= 1000` — zero fast frames** — while `pos_y` drifted past
`trim_high_trigger_y` for the entire last minute of that stretch, and TRIM never fired again for
the rest of it. A height correction is orthogonal to SLAM's solve latency; gating it on SLAM being
fast makes it depend on the same chronic choke (see §6) that everything else in this codebase is
already built to tolerate. Removed the conjunct. The other guards (band check, ceiling calibrated,
not mid-calibration) are untouched and still gate TRIM regardless of SLAM speed.

## 5. Back-off displacement and clearance figures (chunk 1)

`backoff_hold_s` (full reverse duration, clock starts at `BACKOFF` entry) was `2.0`. Measured live
on this flight: one push moved the drone ~1.3 u (SLAM pose `[1.686,-0.440] -> [2.388,0.652]`) and
forward clearance jumped `0.975 -> 1.625` — well past what a stand-off correction needs. Cut to
`1.0`. `backoff_reverse_mag` deliberately LEFT at its existing value (operator's explicit call:
shorter, not softer — a duration change, not a thrust change). This is a platform control DURATION,
not a room-specific answer, per CLAUDE.md's autonomy standard.

`visrec_debug_window` flipped to `true` by the same chunk (operator wants the LKG window open by
default while validating this session's fixes live) — surfaced the config-drift test class
described in §7.

## 6. Corner vs. permanent blacklist geometry (chunk 8)

The loop-blacklist (session 20) already marks a region PERMANENTLY unreachable after repeated
picks-with-no-progress. But the sweep-corner picker (`_pick_sweep_corner`) never consulted it —
corners deliberately ignore SOFT/this-round exclusions by design (a walled-off corner should still
get one fresh look, retired only by its own 2-bump), and that same carve-out was accidentally
shielding corners from PERMANENT exclusions too. Flight `20260901_222552`: `[3.5,-4.75]` was
loop-blacklisted at `22:37:47`; the tour then committed to corner `[4.1,-4.1]` — **0.885 u away,
inside `goal_blacklist_radius` (1.0)** — and hammered it for **3¾ minutes** before a fresh 2-bump
finally killed it, because nothing else was going to. Separately, `perception_worker.py`'s
pre-existing `"WARNING: pick landed on an ALREADY-excluded goal -> blacklist bypassed"` line fired
**eight times** across this same flight — the general symptom of goal selection routing around the
blacklist through a path that didn't check it.

Fix: `FrontierPlanner._excluded_permanent(center)` (mirrors `_excluded` but ignores soft/round
entries) and `_pick_sweep_corner` now force-retires a corner unseen when it sits inside a permanent
dead zone, logging `CORNER-SKIP goal=[...] inside a PERMANENT dead zone -> force-retired, tour
advances` — bookkeeping only (`corner_giveups`/`is_corner`, no `_blacklist_goal` call: a
force-retired corner is "given up on for this tour," not itself declared permanently unreachable).
Soft/round blacklists still don't touch corners — confirmed by a dedicated test
(`52-corner-2`) that a soft entry alone does NOT retire one.

## 7. §4b — the post-backoff re-solve budget — NOT BUILT (decision record)

**Explicitly out of scope for this session.** `backoff_resolve_budget_s` (session 47's upper bound
on the post-backoff SLAM re-solve wait) timed out on all three arms of this flight — 17.9 s,
14.9 s, and 21.0 s — well past a comfortable budget, but no fix for the budget mechanism itself
was built. Two candidates were drafted and both were **rejected by the operator**:

1. Raise the budget to some larger fixed number — rejected as a band-aid that just moves the same
   problem later, and risks the operator's next question being "why did the drone hold still for
   30 seconds."
2. Detect an approximate "SLAM looks re-solved enough" signal from partial/interim data and open
   the gate early — rejected as speculative: it would be inferring confidence in a solve that
   hasn't actually finished, defeating the whole point of the gate (session 47: never re-fire off
   evidence SLAM hasn't had a chance to update).

An external reviewer separately suggested requiring a fresh `cap_ts` frame before opening the gate.
**That suggestion is already the implemented mechanism** — `autopilot.py`'s `_backoff_resolve_since`
stamp (set at `BACKOFF` exit) plus the sole gate-opening site
(`if self._backoff_resolve_since is not None and cap is not None and cap >= self._backoff_resolve_since:`)
already require exactly that: a captured frame timestamped at or after the last back-off ended. The
reviewer's proposal's only actual delta from what's built is deleting the `backoff_resolve_budget_s`
upper bound entirely (wait indefinitely for a fresh capture, no timeout) — which is a different,
narrower change than either rejected draft and was not itself put to the operator this session.
**Record explicitly for the next session:** `_last_good_t` (the cache-write timestamp used elsewhere
for "how stale is this snapshot") is a cache-WRITE time, not a capture time, and must NOT be reused
as a substitute freshness test here — it would tell you when the code last touched the value, not
when the camera captured the frame it came from.

**Backlog only. Do not build without a fresh design conversation with the operator.**

## 8. Self-test results

All four self-test suites (`autopilot.py`, `frontier_planner.py`, `visual_recovery.py`,
`flight_replay.py`) run `--self-test` with **0 failures**, including every new session-52 case.
Every new assertion was proven against its own defect by reverting the fix on a scratch copy and
confirming the corresponding case FAILS there, then discarding the scratch copy — no reverted copy
or scratch script is left in the repo.

## 9. Still open (carried forward, not touched this session)

- **The SLAM choke itself remains undiagnosed.** Now measured at a **median 1943 ms across a whole
  flight**, worst inter-frame gap **13.6 s** — worse than session 50's ~2000 ms plateau. Two causes
  are already ruled out (the autopilot loop itself, and Unity focus loss) — see `PROGRESS.md`'s
  "Next" section for the untested leads.
- `_enter("HOLD_LOST"/"SLAM_HOLD")` still discards a plan-loss-interrupted hop judgement (the STALL
  guard's own starvation, session 49 already covers a different angle of the same symptom) — needs
  a trusted-pose story first.
- §7 above — the post-backoff re-solve budget's timeout value itself is unaddressed.
