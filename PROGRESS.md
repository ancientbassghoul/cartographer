# Cartographer — Progress & Resume Handoff

_Last updated **2026-09-02** (branch **`all-bets-are-off`**, session 54 **BUILT — self-tests ALL
GREEN (0 failures), LIVE-FLY PENDING**; `main` unaffected)._

_**Session 54 — the session-53 live-fly cleared the `SLAM_HOLD` limit cycle, then immediately hit
the SAME bug class one state over: `TRIM` parked for 73.9 seconds with no exit (FIXED, live-fly
PENDING).** Flight `20260902_155916` ran 5m28s. Session 53's fix worked exactly as designed — the
log shows `SLAM_HOLD still waiting (this hold 15.0s / episode 15.0s) -> forcing one hop`, both clocks
named, then the forced hop landed and TRIM fired correctly (`sag pos_y=-1.738 >= trim_sag_trigger_y
=-1.750 -> pulse up`, pulse released 61ms later). It just never left `TRIM`: the WAIT sub-phase's
`healthy` check required `not self._slam_slow`, SLAM was solving at a flat ~2700ms for the rest of
the flight, and TRIM has no wall-clock cap — so `ready` was arithmetically unreachable and the drone
hovered, holding neutral, until the operator killed the run. Same dead band as sessions 44/50/53:
too slow to satisfy the gate, frames arriving every ~2.86s stayed just inside `plan_timeout_s` (3.0s)
so plan status never left `OK` either, so the status-dispatch rescues (`HOLD_LOST`/`_step_stale`)
never got a chance. Root cause: session 52 removed the identical `not self._slam_slow` conjunct from
TRIM's *trigger*, arguing pos_y is the slowest-varying quantity SLAM publishes and its own comment
claimed only that one height-reading gate existed — but the WAIT phase's `healthy` reads the SAME
pos_y for the SAME reason (the "TRIM done ... post pos_y=" log line) and was left untouched. Session
52 made TRIM enterable under slow SLAM without making it exitable. Fixed both halves: dropped the
`_slam_slow` conjunct from WAIT's `healthy` (the real freshness proof is `cap_ts >= _trim_cmd_t0 +
trim_settle_s`, already present) — this alone would have exited this flight's TRIM in ~3s instead of
74s — plus a bounded forced exit reusing `slam_slow_hop_after_s`, mirroring `TRIM_RESUME_WAIT`'s
session-44 rescue verbatim (same knob, same `has_any_capture` blackout guard so a wall clock can't
paper over perception producing nothing), as a backstop against any OTHER unsatisfiable condition.
A follow-up audit of every state in `step()`/the `_step_*` helpers for this exact signature
(`_slam_fast_streak >= N` or `not self._slam_slow` as the ONLY exit predicate, no wall clock) found
**three more live instances, all worse than TRIM** — dispatched above the status router, so not even
a real `PLAN-LOST` rescues them: `CALIB_LOST_HOLD`, `CALIB_ESCAPE`, and `POSTLUDE_LOST_HOLD`
(residual — its 30s budget relaxes the required streak but not the speed bar, so it's still
unreachable at 2700ms). Plus one latent instance, the `SLAM_HOLD` legacy `use_slam_stepback_on_slow=
True` arm (off by default; the sessions-43/53 rescue lives only in the other arm). None of the four
fixed this session — operator's call on scope; recorded as backlog below with the grep signature.
Separately worth recording: SLAM latency on this flight degrades **monotonically with map size** —
mean ms/20-frame-block `489→462→452→485→702→866→3844→2468→3282→2716`, flight median 1184ms, i.e.
`_slam_slow` was true for **more than half the flight**. Direct support for the "MASt3R-SLAM's own
workload grows with the keyframe graph" lead already the top open question below — and the disease
behind all four of these dead-band bugs. Every new assertion proven against its own defect on a
scratch copy: reverting fix 1 (the dropped conjunct) fails only the fast-exit sub-check and flips the
suite to FAILURES PRESENT; reverting fix 2 (the backstop condition) fails only the forced-resolve
sub-check; both restored byte-identical. `python autopilot.py --self-test`, `frontier_planner.py`,
`visual_recovery.py`, `flight_replay.py`: **ALL PASS, 0 failures**. See
`plans/session54-trim-wait-no-exit.md` for the full trace + the four-state audit table. **NEXT =
LIVE-FLY.**_

_**Session 53 — the session-52 live-fly hit a NEW blocker before most of session 52 could even be
exercised: a `SLAM_HOLD` ↔ `HOLD_LOST` limit cycle that hovered the drone for 2m21s until the
operator killed the run (FIXED, live-fly PENDING).** Flight `20260902_143207` ran ~5 minutes total —
prelude, one short leg, then 43 straight `SLAM_HOLD`→`HOLD_LOST`→`SLAM_HOLD` bounces at a dead-steady
~3.3s period, nothing else. SLAM never lost tracking (`PLAN-STALE` count: **zero**, all flight) — it
was just slow, median 3155ms, so `plan_timeout_s` (3.0s) kept aging the plan out between frames.
The session-43 forced-hop rescue (`slam_slow_hop_after_s`=15.0s) exists for exactly this — and fired
correctly ONCE, early in the flight — then never again. Root cause: `_enter_slam_hold` re-stamps
`_slam_hold_start = now` on EVERY entry, so the rescue's `waited` clock got zeroed by the very
PLAN-LOST/HOLD_LOST bounce it was supposed to survive, capping out at ~3.0s against the 15.0s bar —
arithmetically unreachable, same shape as session 50's SETTLE dead band. The exact lesson was already
written in this file, just applied to the wrong field: the `_slam_stepback_count` comment right above
`_slam_hold_start`'s declaration already explains "resetting this on every fresh hold entry meant the
escalation could never reach its cap in exactly the scenario it exists to bound" — and was never
extended to the timer next to it. Fix: a new `_slam_hold_episode_t0` clock, stamped only on the FIRST
`_enter_slam_hold` of a bad patch (mirrors `_slam_stepback_count`'s own reset rule exactly — cleared
only in the REPLAN handler, `reset_leg()`, and the settle-gate-clear branch); `_slam_hold_start`
itself is untouched so the existing per-hold log wording doesn't silently change meaning. Scope
deliberately narrowed to `SLAM_HOLD` only (operator's call) — the identical hazard exists in `SETTLE`'s
session-50 escape and `TRIM_RESUME_WAIT`'s session-44 escape (both re-stamp on re-entry too) but
neither has manifested on a flight; left as backlog. No escalation past the forced hop was built
either (operator's call) — the hop is the only lever that might unstick a slow solve, and this flight
supplied zero evidence for picking a cap. New self-test modeled directly on the existing
`SLAM-STEPBACK counter PERSISTENCE` test drives the same OK/PLAN-LOST bounce and asserts the forced
hop fires; proven against its own defect by reverting the one-line fix in place and confirming ONLY
this new test fails (`bounce-preserves=False, forced-hop-fires=False`), then restored byte-identical
from backup. `python autopilot.py --self-test`, `frontier_planner.py`, `visual_recovery.py`,
`flight_replay.py`: **ALL PASS, 0 failures**. Because the flight was cut short by this bug, almost
none of session 52's own watch-list items got exercised (TRIM fired once, but for the sag/UP case,
not the slow-SLAM/DOWN case it was built for; `VISUAL_RECOVERY`, back-off, and the corner blacklist
saw no real evidence either way) — they are still themselves LIVE-FLY PENDING, not confirmed and not
refuted. See `plans/session53-slamhold-limit-cycle.md` for the full trace. **NEXT = LIVE-FLY.**_

_**Session 52 — the visual-recovery probe had never once run on a real flight, TRIM was starving
under slow SLAM, and a permanently-blacklisted corner still got flown into (BUILT, live-fly
PENDING).** Flight `20260901_222552` surfaced five symptoms at once. We wanted the 15° probe
(sessions 36/42) to actually execute a MATCH someday; we found it structurally couldn't — four
defects stacked to keep it unreachable (a one-shot flag spent too early by the loss-recovery grace;
the shared one-shot almost always pre-spent because 57 of 58 losses opened as `PLAN-LOST`; the
`PLAN-LOST` branch had no dispatch case for an in-progress probe, so any flicker killed it; and
even once entered, its frozen reference frame (F_LKG) was cached by tick-freshness instead of by
the SLAM frame_id the plan was actually computed from, silently drifting under this flight's up-to-
14.9s solve latency). Fixed all four together — a shared latch was not enough, the probe needed its
own. Separately, TRIM had an `and not self._slam_slow` guard that made it depend on the exact SLAM
choke it exists to correct — 2749 consecutive ticks, zero fast frames, TRIM dead the whole time; removed
the conjunct. A stale post-backoff gate was also raising "BACK-OFF SUPPRESSED" notices on episodes
where no back-off was ever contemplated (evidence-free), swallowing the whole episode's downstream
checks for `backoff_resolve_budget_s`; reordered it behind the evidence check. And a sweep corner
0.885u inside an already-permanently-blacklisted region got hammered for 3¾ minutes because corners
had a blanket carve-out from ALL blacklist checks, soft and permanent alike; scoped the carve-out to
soft/round-only. Also cut `backoff_hold_s` 2.0→1.0s (one push measured moving the drone ~1.3u, well
past a comfortable stand-off). Drafted a fix for the post-backoff re-solve budget's own timeout
(§4b) — operator rejected both drafts; an external reviewer's alternative turned out to already be
the implemented mechanism, its only real delta being to delete the timeout entirely, which was not
itself put to the operator this session — left as backlog. Every new assertion proven against its
own defect by reverting on a scratch copy. `python autopilot.py --self-test`, `frontier_planner.py`,
`visual_recovery.py`, `flight_replay.py`: **ALL PASS, 0 failures**. See
`plans/session52-lkg-recovery-unreachable.md` for the full four-defect trace + the corner/blacklist
geometry + the §4b decision record. **NEXT = LIVE-FLY.**_

_**Sessions 50-51 — the 91.6-second SETTLE, and two dead ends on the SLAM choke (BUILT, live-fly
PENDING).** Flight `20260901_172217`. The operator flew sessions 47-49, confirmed the back-off works,
then flagged two things. First: **the drone sat in `SETTLE` for 91.6 seconds** (3531 ticks) with plan
status `OK` the entire time. SLAM was alive and tracking, just slow — 42 frames, min 1834ms, max
2093ms, **mean 1995ms, ZERO under `slam_slow_ms` (1000)**. The settle-gate needs 6 CONSECUTIVE frames
under that bar, so at 2x over it was **arithmetically unreachable**; meanwhile frames arriving every
~2.2s stayed comfortably inside `plan_timeout_s` (3.0s), so the status never went LOST either. A
**dead band between two thresholds**: too slow to proceed, too alive to be rescued. `SETTLE`'s own
comment named the assumption that failed — "if SLAM stops delivering, the plan status goes STALE/LOST
and the step() top diverts to recovery" — but SLAM never stopped delivering. Unlike `ORIENT`/`ADVANCE`
(which divert to `_enter_slam_hold` on `_slam_slow`) and `SLAM_HOLD`/`TRIM_RESUME_WAIT` (which both
have a forced-resolve rescue), `SETTLE` had no upper bound at all; it escaped only by accident when one
3.006s inter-frame gap finally tripped `PLAN-LOST`. Same bug class session 44 fixed for
`TRIM_RESUME_WAIT` and never applied here. Built session 43's rule verbatim into `SETTLE` — plan OK +
stuck past `slam_slow_hop_after_s` (reusing the knob, not adding one) -> proceed anyway, loudly —
keeping `SLAM_HOLD`'s capture-blackout guard (a wall clock must not paper over perception producing
NOTHING) and its `_enter`-then-stamp ordering trap for the hop grace. Deliberately NOT a divert to
`SLAM_HOLD`: `SETTLE` is often entered FROM it, so that would ping-pong at tick rate. **Second**, the
operator asked what in the autopilot chokes SLAM — and the answer, measured, is **nothing**: the loop
ran at **38.5Hz through the entire wedge**, 38.6Hz in `SLAM_HOLD`, 32Hz in a SIFT-heavy `HOLD_LOST`.
The Unity-focus theory (SLAM ran ~350ms unfocused vs ~2000ms focused, and Unity throttles rendering
when it loses focus) looked decisive and was **tested and ruled out — SLAM re-chokes ~2 frames after
refocus**; the log had already undercut it (SLAM recovered BEFORE the pause, degraded to 1767ms DURING
it with the autopilot idle, mean 734ms not 350ms across the paused window; and the by-state
correlation is reverse causality — slow SLAM is what PUTS the drone in `SETTLE`/`SLAM_HOLD`). **The
choke is still undiagnosed and open.** What the measurement DID turn up is self-inflicted waste: the
visual match ran on EVERY tick of a loss — **~380 full SIFT+BFMatcher+RANSAC passes across session
48's 12s grace to make ONE decision** (the `[VISREC]` log is throttled to 0.5s, which hid it; the
computation never was), and `match()` recomputed SIFT on the REFERENCE frame every call even though
F_LKG is frozen for the whole loss. The consumers turned out to be exactly two, both already guarded
(`_maybe_loss_snapshot_backoff` under `not _loss_snapshot_checked` at both call sites, and the probe's
`MATCH` phase), so: **GATE A** (`wants_visual_match()`, exact, zero staleness) skips ticks nothing can
read — with the trap that `_loss_snapshot_checked` is False throughout healthy flight, so it must
AND-narrow the status condition, never replace it, or it would run SIFT on every tracking frame;
**GATE B** memoises across the held-still grace (`visrec_match_min_interval_s`=0.5) with forced
recomputes on the loss edge, the probe's MATCH phase, any commanded motion, and a replaced F_LKG;
plus a **lazy `_lkg_feats` memo** so the reference's SIFT is computed once per reference (flagged on
the TUPLE slot, since a featureless frame legitimately yields `(kp, None)` and would otherwise
recompute forever). Measured: a 12s held-still loss now costs **24 real matches over 384 ticks**,
was 384. **Expected to change nothing about SLAM** — it is hygiene, judged as such. Every new
behaviour proven against its own defect by reverting on a scratch copy, and **that exercise caught a
weak test of mine**: the first GATE A assertion used a fresh memo, so GATE B's rate-limit returned
False anyway and masked the missing gate — the revert PASSED when it should have failed; re-asserted
with no/stale memo so it isolates GATE A. `python autopilot.py --self-test`, `visual_recovery.py`,
`flight_replay.py`, `frontier_planner.py`: **ALL PASS, 0 failures**; `perception_worker.py` (venv):
PASS. See `plans/session50-settle-dead-band-escape.md` and
`plans/session51-visual-match-on-demand.md`. **NEXT = LIVE-FLY.**_

_**Session 49 — the same wall, five legs, one wasted 8 minutes: goal stagnation memory finally wired
up, plus an LKG debug window (BUILT, live-fly PENDING).** Sessions 47/48 flew — back-off is confirmed
working, operator's own call. He then flagged the SAME flight's tail: a goal near a wall got bumped
and backed off cleanly around `15:49:11`, then at `15:50:14.315` the drone picked a new goal SO close
to the last one he asked "haven't we fixed that like a thousand times?" and "don't we blacklist after
a few picks — how many?" Traced flight `20260901_154648`: the drone launched **five separate legs at
the same physical wall** between `15:48:47`-`15:50:52`, retired only at `15:57:27` (~8 minutes, ~40
losses later). No single "N picks" answer exists — three guards, each with a second condition, were
ALL structurally starved: **(1)** the STALL guard needs a *judged* hop, but `_enter("HOLD_LOST"/
"SLAM_HOLD")` clears the pending judgement on every plan-loss (`autopilot.py:2280`) — whole flight,
**7 `HOP_BASELINE`, 2 `HOP_JUDGE`**, zero strikes ever accrued; **(2)** the LOOP guard's history was
split in two — the commitment follows a live frontier centroid at `goal_assoc_dist` (1.0u) but the
goals-DB disc was frozen at `goal_area_radius` (0.5u) around its CREATION point, so one wall drifting
0.60u ended the flight as TWO discs, `[-2.05,2.303] picks=3 bumps=1` and `[-2.5,2.7] picks=2`, neither
reaching its own threshold; **(3)** per-hop "progress" was a lie — every leg read as closing distance
(3.85→2.99→1.68→3.55→2.21) because the SLAM pose jumped BACKWARDS between legs; only a best-EVER test
sees the truth (never closer than 1.68u after leg 3), and `_best_dist` — the field whose own
docstring already said "closest distance achieved toward the current committed goal" — was read and
reset in three places and **written in none**. A third, distinct starvation class, not a regression
of session 33 or 45. Built two fixes in `frontier_planner.py`: a goals-DB disc's `center` now FOLLOWS
its goal (bounded by a new `goal_disc_max_drift`=0.75u measured from an immutable `origin`, kept
strictly below `goal_blacklist_radius` by a fail-fast `ValueError` so a fully-drifted disc's origin
stays inside its own exclusion ball) — unifying that one wall into ONE disc; and `_best_dist` finally
gets WRITTEN at the commit site, feeding a new `goal_stagnant_limit`=2 (consecutive legs with no new
closest approach → permanent blacklist, reason `"stagnant"`) that survives both the pose-jump lie and
the STALL guard's dead hops. Replayed against the real flight's numbers: blacklist would land at
`15:50:38` instead of `15:57:27` — seven minutes and an 18-cycle FALLBACK sweep saved. Deliberately
NOT built: an unconditional pick-count cap (would kill a legitimate far march). Separately built the
operator's other ask — an LKG debug window. `visual_recovery.py`'s `match()` gains `debug=`/`banner=`
(zero cost when off) composing F_LKG|live with drawn RANSAC inliers on EVERY return path including
failures (seeing why a match failed is the point); `autopilot.py` opens it live (`visrec_debug_window`,
default off) and saves canvases to `OUTPUT/diag/<ts>_visrec/` at decision instants (capped by
`visrec_save_max`=200), with the window half and the save half failing INDEPENDENTLY (a dead display
must not stop the PNG evidence, a disk failure must not close the window) — each sets its own visible
flag + one CRITICAL log line, both ride the replay timeline; `flight_replay.py`'s existing Visual
Recovery panel now shows the saved canvas as a thumbnail. An external review vetted the design before
build; one of its two suggestions (register both a drifted disc's center AND origin in the blacklist
store) doesn't actually work — `_blacklist_goal` MERGES nearby entries, so a second one would move the
first, covering LESS — the design instead makes the hazard impossible by construction (the drift-budget
inequality above); its other suggestion (independent window/save failure isolation) was adopted as
specified. Every new behaviour proven against its own defect by reverting on a scratch copy: dropping
the disc-drift condition splits the disc back into two; removing the `_best_dist` write fails only the
`select()`-integration test (bookkeeping tests deliberately isolated from the data source stay green —
a tighter signal than one coupled assertion); disabling the stagnant-blacklist call fails the real-flight
case while the never-blacklisted march stays green; injecting a `cv2.resize` into the debug canvas
fails the exact-width no-scale assertion; collapsing the sink's two independent guards into one shared
`try/except` fails BOTH isolation tests at once (a dead display now also poisons the save flag). `python
frontier_planner.py --self-test`, `python visual_recovery.py --self-test`, `python autopilot.py
--self-test`, `python flight_replay.py --self-test`: **ALL PASS, 0 failures**. See
`plans/session49-goal-stagnation-and-lkg-window.md`. **NEXT = LIVE-FLY** — watch for a
`PLANNER: ... reason=stagnant` line retiring a wall after 2 fruitless legs instead of ~8 minutes; NO
legitimate far goal retired mid-march (the one accepted risk); the goals-DB panel showing ONE disc
with nonzero `drift` where this flight showed two; the LKG window's drawn inliers piling onto one flat
surface nose-to-a-wall. Still open, deliberately not touched: `autopilot.py:2280` still discards a
plan-loss-interrupted hop judgement, so the STALL guard itself remains starved — stagnation covers
that ground from a different angle now, but judging interrupted hops late is its own session (needs a
trusted-pose story first; this flight's SLAM pose jumped 1.3u between ticks)._

_**Session 48 — the back-off was reacting to a 3-second blip, not to being stuck (BUILT, live-fly
PENDING).** Session 47 stopped the back-off *loop*; the operator then asked the better question —
is reacting ~70ms after the plan goes `PLAN-LOST` too harsh? — proposed holding ~12s first to let
SLAM re-lock, and asked for statistics before committing to the number. Measured them across **all
128 flight logs, 2066 loss episodes**: with the drone HOLDING STILL a loss resolves by itself in a
median of **2.4s**, p95 **9.5s**, and **96.9% within 12s**; the 25 episodes where something DID
maneuver mid-loss have a median of 29.1s and a max of 183.6s (25 samples and partly reverse-causal,
so suggestive rather than proof — but the mechanism, moving while blind destroying SLAM's visual
continuity, is already documented in this codebase). Then the number that settled it, per-episode
across the two most recent flights: **seven of the eight back-offs fired into losses that
self-healed in under 1.1 seconds** (0.11 / 0.20 / 0.55 / 0.67 / 0.77 / 0.87 / 1.09s) — the plan was
green again before the 2s reverse push had even finished, including the one session 47 had called
legitimate. The cause is structural: `PLAN-LOST` is not an event but a 3.0s `plan_timeout_s`, and
the back-off fired one tick after it (66-78ms, 8 for 8). Built `loss_backoff_grace_s = 12.0`: a loss
must OUTLIVE the window before it earns a physical reaction, the one-shot stays ARMED (not spent)
while deferring and the `HOLD_LOST` tick re-runs it each tick — without that re-call the back-off
would be dead code rather than delayed — and the window is stamped per-episode AND cleared by a
genuine `OK`, so the 19-flip `OK`/`PLAN-LOST` oscillation of flight `20260901_142738` can never
accumulate into a spurious reaction (a slow-but-alive SLAM is not lost; `SLAM_HOLD`'s forced hop is
that remedy). Every assertion proven against its own defect by reverting on a scratch copy —
including the discovery that the two window guards are **redundant by design**, so no single-guard
revert can fail the oscillation assertion; breaking both together does, which is what proves it has
teeth. `python autopilot.py --self-test`: **ALL PASS, 0 failures**. See
`plans/session48-loss-recovery-grace.md`. **NEXT = LIVE-FLY** — and watch the hold itself: this
lengthens exactly the window in which the 20260718 flight drifted into a wall while parked, and the
intended guard for it is the one session 47 showed is structurally inert._

_**Session 47 — the back-off loop: SLAM never got to look at where the back-off put us (BUILT,
live-fly PENDING).** Session 46's first live test, flight `20260901_142738`, failed loudly: the
operator saw two or three back-offs in a row, then the drone put its back to a wall and answered
with another back-off. Session 46's chunk 1 had worked (real `reverse: 1.0` commanded at last,
against `fields={}` the flight before) — but chunks 2 and 3 were dead code, 0 `FALLBACK` and 0
`WEDGED` all flight. Why: session 46 hung its whole "stop repeating a reflex that isn't working"
escalation off `_blind_contact_backoff`, and **all 7 back-offs came through the other door** (the
loss-instant `F_LKG` visual check), which counted nothing. Worse, that escalation could never have
fired anyway — it is polled from `HOLD_LOST`/`SLAM_HOLD`, states that hold no directional command,
so the flow detector produces no verdict there at all: 12 `HOLD_LOST` + 14 `SLAM_HOLD` entries,
**zero** detector verdicts from either, one latched contact in the whole flight (a `CEILING` during
`ASCEND`). The operator then named the real fix himself: **SLAM never got an opportunity to recover
after a back-off, which is the whole point of backing off.** Confirmed exactly: `BACKOFF` → `SETTLE`,
whose exit needs 6 frames under 1000 ms while SLAM was solving at 3407/3547/3725 ms — impossible, and
`SETTLE` has no timeout by design, so the only thing that ever broke the deadlock was the next
`OK`→`PLAN-LOST` flip, and the recovery path's first act is another back-off. Each flip re-arms the
one-shot, and the trigger asks only "am I newly lost / does F_LKG read too-close" — neither of which
changes when a back-off fails, so the evidence was literally identical every time (538 → 577 → 479 →
704 → 444 → 452 → 412 inliers, `contained=True` on all seven; 19 status flips, one re-fire just
1.05 s after the previous back-off ended). Built three things: **(1)** a **post-backoff SLAM
re-solve gate** — no further loss-instant back-off until SLAM has solved a frame *captured after*
the last one ended (bounded by a budget, and the timeout is LOUD); **(2)** both loss-instant
triggers now feed the SAME wedge counter, so session 46's `FALLBACK` escalation is finally
reachable from the door the drone actually uses; **(3)** `BACKOFF` now receives `backwall_contact`
and stops pushing — it had streamed **31 `BACKWALL-WATCH` verdicts** from inside `BACKOFF`, `ratio=0.00`
while commanding full reverse, and ground the full 2.0 s into the wall anyway, seven times. Every
assertion proven against its own defect by reverting on a scratch copy; **one trap the suite missed
on the first pass** (a gate that opens on any fresh frame regardless of `cap_ts` — at 3.5 s latency,
pre-backoff captures are exactly what arrive first) was caught by rebuilding that revert, and a
dedicated assertion added. `python autopilot.py --self-test`: **ALL PASS, 0 failures**. See
`plans/session47-post-backoff-slam-resolve-gate.md`. **NEXT = LIVE-FLY.**_

_**Session 46 — wedged in a corner: the drone recognised it, reacted the only way it knew, and was
physically incapable of it (BUILT, live-fly PENDING).** The operator flagged the end of flight
`20260901_124211` — brief flashes of `BACKOFF` near the end, but the drone stayed stuck — and named
the missing piece precisely: "tried to backoff, can't, on hold too long, try something different."
Validated the wedge from the flight's own data before touching code: displacement per ~2s commanded
reverse push collapsed monotonically, `0.335u → 0.444u → … → 0.046u → 0.016u → **0.000u**` (final
strafe also `0.000u`), and the SLAM-independent flow detector independently latched `BACKWALL`
(flow ratio going *negative*, `contact_held=0.891`) — two signals agreeing the drone was physically
pinned, not just perceiving badly. Then found why nothing escalated: status oscillated `OK 3.0s /
PLAN-LOST 0.6s` for the last minute, so every escape keyed on continuous time-in-one-state reset
before firing. `FALLBACK` — the one mechanism that already does "try something different" (turn +
push a fresh random direction each cycle) — is dispatched only from the PLAN-STALE handler, and
this flight had **zero** PLAN-STALE events, only PLAN-LOST. `BACKOFF` fired 6 times and commanded
**zero** reverse every time — the top-level PLAN-LOST router wiped it to `HOLD_LOST` one tick after
entry, before its own phase-timer body (on a *later* tick) ever got to run, the identical structural
bug the `BLIND_BACKOFF` state already had a documented fix for. Built exactly what the operator
asked, plus what it structurally required: **(1)** extracted `BACKOFF`'s body into `_step_backoff`
and gave it status-ownership like `BLIND_BACKOFF`/`CALIB_ESCAPE` already have, so a backoff in
flight survives a `PLAN-LOST` flicker and actually commands reverse; **(2)** made the `FALLBACK`
sweep dispatchable under `PLAN-LOST` too (previously PLAN-STALE only), without touching its
existing PLAN-STALE path; **(3)** a new counter, `_blind_contact_reacts`, that escalates
`BLIND_BACKOFF` into the `FALLBACK` sweep after `blind_contact_escalate_after` (2) failed reflexes
against the same obstacle with no confirmed recovery between — reset ONLY at genuine
recovery/reset boundaries (never a bare status flip, never a same-disc goal re-commit), so the 3s
`OK`/`LOST` flicker that flight showed cannot silently clear it. Built as a 4-chunk Sonnet-ready
implementation spec (contract-first: exact signatures, a load-bearing reset-site table, explicit
"do NOT do X" guardrails for two traps a fresh implementer would hit — resetting `_fallback_cum_deg`
redundantly, and an unqualified REPLAN reset re-creating the exact pick-dedup starvation session 45
just fixed). One implementation bug caught by a *test's own* loop logic (not the revert exercise):
a test drove past its intended window straight into `BACKOFF`'s natural completion tick and
misread that as a regression — fixed by tightening the loop bound, not the code. Every fix proven
against its defect by reverting on a scratch copy and confirming FAIL, restored after. `python
autopilot.py --self-test`: **ALL PASS, 0 failures** (82 blocks). See
`plans/session46-wedged-corner-escalation.md`. **LIVE-FLY ATTEMPTED
(`20260901_142738`) — chunk 1 confirmed working, chunks 2+3 proven unreachable; see session 47
above, which makes them reachable.**_

_**Session 45 — the "stuck next to its own goal" stall: four defects, all fixed (BUILT, live-fly
PENDING).** The operator flagged the end of flight `20260901_112227` — the drone sat still for
minutes near its goal — and asked why neither guard he expected had caught it: a too-close goal
getting blacklisted, or the goal being marked reached. Answer, from the flight's own log: **one did
not exist, the other could not run.** Hard numbers: `pos=[-0.7592,-0.0389]` vs
`goal=[-0.5464,-0.2674]` = **0.312u apart with `goal_reach_dist`=1.0, for 221 seconds**, status `OK`
for ~2300 of those ticks. It cycled `SLAM_HOLD` →(15s forced hop)→ `REPLAN` → `ORIENT` (turn +30°) →
`PARALLAX_PUSH` → `SLAM_HOLD` ~30ms later, forever — **turning every ~25s and never translating**
(whole-flight `ADVANCE` count: 5). Four defects: **(1)** the planner had NO minimum-distance check
at all, while its utility divides by distance (so a frontier on top of the drone scores highest) and
the clearance inset walks goals *toward* the drone; **(2)** the "goal reached" test lived ONLY inside
the `ADVANCE` handler, which the stall never entered; **(3) the root cause** — `PARALLAX_PUSH` was
missing from the forced-hop grace window (`_enter()`'s exemption AND the `_slam_slow_hop_active`
guard `ADVANCE` has), so under chronically slow SLAM every forced hop died on entry, making the
session-35/43 rescue a no-op for any off-axis goal; **(4)** the pick-dedup starved the loop
blacklist — it suppresses a pick unless a hop was *judged*, no hop ever completed, so the hammered
disc ended at **`picks=1`** when the guard needs 3 (the same starvation class the code documents
fixing once already, 20260720). All four fixed; the too-close rejection is soft/round-scoped per the
operator's choice. **An external review caught a real trap in the draft**: the state-independent
reached check would have fired inside `SETTLE` too, and `_enter("SETTLE")` resets `_settle_ok`/the
gate while the drone doesn't move — an infinite SETTLE hover, worse than the original stall.
Verified in code, `SETTLE`/`REPLAN` (+ TRIM/recovery/postlude) excluded, with a dedicated regression
test. Every new test proven to catch its defect by reverting the fix and confirming FAIL. `python
autopilot.py --self-test`, `frontier_planner.py`, `flight_replay.py`, `map_store.py`,
`ground_grid.py`: **ALL PASS**. See `plans/session45-stuck-at-own-goal.md`. **NEXT = LIVE-FLY.**_

_Session 44 (previous): replaced TRIM's live-calibrated sag/high band with two
HARDCODED absolute SLAM pos_y thresholds (`trim_sag_trigger_y=-1.75`, `trim_high_trigger_y=-2.10`),
per the operator's explicit, knowing override of CLAUDE.md's "NO MANUAL-FLIGHT DATA LEAKAGE"
standing rule — flagged the conflict first, operator chose to proceed anyway. Found + fixed a real
regression while building it (removing the "must have calibrated once" precondition let TRIM
hijack the PRELUDE sequence, cascading into 4 unrelated self-test failures). Also traced (and
fixed) the two long-standing "pre-existing, unrelated" self-test failures this session, prompted
by the operator asking what they actually were: `explore ALTITUDE-LOCK` was genuine config drift
(`desired_height_override_y` left at `-1.9` live instead of its disabled default `0` — reset it).
`explore PRELUDE arm+takeoff+...` was initially (wrongly) called "pre-existing/unrelated" too —
corrected once actually tested against the clean base: it's a real session-44 regression (the
test's synthetic flat `pos_y=0.0` reads as permanently "sagged" against the new fixed threshold) —
fixed by disabling TRIM for that one test (it's about the takeoff sequence, not TRIM). **Then the
first actual live-fly attempt got stuck in `TRIM_RESUME_WAIT` forever right after takeoff** —
traced from the flight's own log (`20260901_103028`), not guessed: calibration and the TRIM pulse
both worked correctly (`pos_y` corrected from `-2.101` to `-1.878`, safely inside the band); the
REAL bug was that SLAM's solve times ran 900-1500ms for ~35s straight (plan status stayed `OK` the
whole time — a pure throughput patch, not a bad pose) and `TRIM_RESUME_WAIT` — unlike `SLAM_HOLD`
(sessions 35/43) — never had a timeout for sustained slowness, so it just hung. Fixed by giving
`TRIM_RESUME_WAIT` the identical `slam_slow_hop_after_s` forced-resolve rescue, with a new
self-test that reproduces the exact hang and is confirmed to fail without the fix. `python
autopilot.py --self-test`: **ALL PASS, 0 failures**. See
`plans/session44-hardcoded-height-trim-thresholds.md` for the full trace of both bugs. **This
hardcoded-threshold work is scoped to `all-bets-are-off` only — do not merge back to `main`
without re-deciding the exception there** (the `TRIM_RESUME_WAIT` timeout fix, however, is a
general robustness fix with no room-specific data in it — worth porting to `main` on its own
merits once proven live). Resume pointer for `main`/session 43 work is preserved below.

_Last updated (session 43, `main`) **2026-09-01** (sessions 40-43 **LIVE-FLY CONFIRMED —
tolerable overall; HEIGHT still open, see "Next"**). Resume from THIS file. Session 43 simplified
the `SLAM_HOLD` forced-hop rule
(`slam_slow_hop_after_s`) to fire on ANY sustained hold with plan OK — not just a plain mid-leg slow
hold — per the operator's explicit instruction, diagnosed off flight `20260723_000631`'s 31.5s
stall; kept one guard the self-test suite caught (a total capture blackout, `cap_ts` never fed,
must still not force a hop on zero live data). A live flight on 2026-09-01 confirmed sessions
20-43 together (TRIM vertical pulse, goal-distance readout, PLAN-LOST/VISUAL_RECOVERY scoping, and
this SLAM_HOLD forced-hop, plus the whole preceding BACKOFF/FALLBACK/homing backlog) fly
tolerably — operator's own call, no further per-session checklist needed. **The one open problem:
height is still off** — not yet diagnosed, next session's starting point — see "Next" below.** Plan
of record:
**`plans/session43-slam-hold-forced-hop-simplification.md`** (+
`plans/session42-plan-lost-visual-recovery-scoping.md`,
`plans/session41-visualizer-goal-distance.md`,
`plans/session40-trim-vertical-pulse.md`,
`plans/session39-return-to-origin-backoff-removal-and-done-loss-fix.md`,
`plans/session38-desired-height-override.md`, `plans/session37-visualizer-telemetry-panel.md`,
`plans/session36-visual-recovery-15deg-probe.md`,
`plans/session35-slam-slow-strategy-switch-and-recovering-fix.md`,
`plans/session34-proactive-clearance-while-blind.md`,
`plans/session33-goal-loop-clearance-inset-fix.md`,
`plans/session32-orient-home-ping-pong-and-home-refine.md`,
`plans/session31-rewind-off-simple-fallback-sweep.md`, `plans/session30-backoff-hard-gate.md`,
`plans/session29-clearance-tab-direction-cycling-fallback.md`,
`plans/session28-trim-resume-gate-clearance-vote.md`,
`plans/session27-video-recording-pointcloud-export-graceful-shutdown.md`,
`plans/session26-homing-backoff-settle-freshness-pick-dedup.md`,
`plans/session25-trim-macros-recovery-fixes-goaldb-schema-debugger-nav.md`,
`plans/session24-settle-gate-pick-dedup-corner-giveup.md`, `plans/session23-backwall-reaction-and-
parallax-retry.md`, `plans/session22-fixed-height-ref-and-bidirectional-trim.md`,
`plans/session21-restore-height-calib-and-trim.md`, `plans/session20-goal-db-loop-blacklist.md`)._

_**Session 43 — simplified the `SLAM_HOLD` forced-hop rule, diagnosed off flight `20260723_000631`
(BUILT — self-tests green, live-fly PENDING).** The operator flagged a 31.5s `SLAM_HOLD`
(`00:10:15.931`-`00:10:47.383`) with plan `OK` the whole time, despite `slam_slow_hop_after_s`
configured to 15s. Traced it: this was a *recovery* hold (entered after a `PLAN-LOST` recovered to
`OK`, which unconditionally routes any recovery-state resume into `SLAM_HOLD` with
`_slam_resume="SETTLE"`, `_recovering` left `True` — "a fresh RELOC pose is shaky"). The forced-hop
escape required `_slam_resume == "ADVANCE" and not self._recovering` — deliberately scoped (session
35) to a plain mid-leg slow hold only, never a recovery-settle hold — so the 15s knob never applied;
the hold instead waited out the settle-gate (6 consecutive frames under 1000ms), which took 31.5s
because SLAM's solves oscillated right around that line the whole time. **Operator's explicit
instruction, no hedging**: "IF SLAM'S PLAN IS OK, AND THE STATE IS SLAM_HOLD FOR OVER
slam_slow_hop_s — GO TO NEXT GOAL," dropping the resume-target/recovering distinction entirely,
since `PLAN-LOST`/a slow settle-gate is a perception throughput signal (sessions 28/42), not
evidence the pose is wrong. Built exactly that: the forced-hop condition is now just `waited >=
slam_slow_hop_after_s`; since firing it now bypasses the normal settle-gate-clear trust boundary, it
performs that same trust-restoration itself (`_recovering`/`_history_broken`/fallback-sweep/
visual-recovery/`command_history` all cleared at the moment it fires) so a forced hop never leaves
the flight stuck "untrusted." **One guard kept, found by the self-test suite, flagged to the
operator before proceeding rather than silently patched away**: the rewrite broke a pre-existing
test where a calibration that fails out to `STUCK` with `cap_ts` NEVER fed at all (a genuinely
worse failure — perception producing no timestamped output whatsoever, not just slow solves) was
now also force-hopping after 15s, flying on zero live capture evidence — exactly what that test's
session-15 protection ("must NOT fly to a goal on a stale pose") forbids. Fix: the forced hop
additionally requires at least one entry in the current SLAM-health window to carry a real `cap_ts`
("SLAM said SOMETHING, even if slow" vs. total capture blackout) — genuinely different from the
real flight (which had `cap_ts` on every frame) and not a reintroduction of the removed distinction.
Rewrote the session-35 "SLAM-slow strategy switch" self-test's case (c) to assert the NEW behavior
(a recovery-settle hold now hops and restores trust) instead of the old one it directly contradicted.
`python autopilot.py --self-test`: rewritten `SLAM-slow strategy switch` and `HEIGHT RE-CALIB
state-gated` blocks both PASS; the same two pre-existing, unrelated failures from sessions 39-42
(`explore ALTITUDE-LOCK`, `explore PRELUDE arm+takeoff+...`) remain, unaffected. See
`plans/session43-slam-hold-forced-hop-simplification.md` for the full trace + design. **NEXT =
LIVE-FLY** — confirm a future recovery-settle `SLAM_HOLD` past `slam_slow_hop_after_s` (15.0s) now
forces a hop instead of sitting indefinitely, and that `_recovering` visibly clears when it does._

_**Session 42 — scoped the `VISUAL_RECOVERY` hand-off to `PLAN-STALE` only, diagnosed off real
flight logs (`20260721_233244`, `20260722_124351`) (BUILT — self-tests green, live-fly PENDING).**
A follow-up question about whether F_LKG keeps updating during `VISUAL_RECOVERY` led to actually
pulling those two flights' timelines, which turned up something bigger: every `VISUAL_RECOVERY`
entry in both (62 total) was triggered by `PLAN-LOST`, none by `PLAN-STALE`, and every single one
reverted to `HOLD_LOST` exactly one tick later — the 15° turn probe has never actually executed on
a real flight. Root cause: `_step_visual_recovery` (the TURN→MATCH→WAIT_RECOVER machinery) is only
ever dispatched from `_step_stale`, itself only reached `if status == "PLAN-STALE"`; the top-level
`PLAN-LOST`/`NO-PLAN` branch has no equivalent dispatch for an in-progress `VISUAL_RECOVERY`, so it
falls into the generic "not HOLD_LOST -> enter HOLD_LOST" logic and gets swept straight back out.
The operator's read on the underlying question, which this evidence supports: `PLAN-LOST` means
perception itself stopped publishing — a throughput/backlog problem (session 28 already diagnosed
exactly this: a synchronous SLAM solve blocking the loop for 9-10s), not a "this viewpoint is
confusing" problem — so the only sound universal remedy is to hold still and wait, never to
actively search; `PLAN-STALE` (perception alive, SLAM explicitly reports not-tracking) is the case
where a different viewpoint is a coherent remedy and stays the only entry point. Decision: make
this the explicit design rather than an accidental one-tick artifact — `_maybe_loss_snapshot_backoff`
now takes a `status` parameter and gates ONLY the final `_enter_visual_recovery` hand-off on
`status == "PLAN-STALE"`; the two BACKOFF reactions ahead of it (cached-clearance-too-close, and
visual-too-close via a contained/planar-like F_LKG match) are untouched and still fire for
`PLAN-LOST` too, per the operator's explicit ask to keep those (they're one-shot DEFENSIVE
reactions to an already-known reading, not an active search). New self-test case mirrors the
existing "both loss-instant checks inconclusive" test but under `PLAN-LOST`, asserting `HOLD_LOST`
instead of `VISUAL_RECOVERY`. `python autopilot.py --self-test`: the rewritten VISUAL RECOVERY block
PASSES; the same two pre-existing, unrelated failures from sessions 39-41 (`explore ALTITUDE-LOCK`,
`explore PRELUDE arm+takeoff+...`) remain, unaffected. See
`plans/session42-plan-lost-visual-recovery-scoping.md` for the full trace + design. **NEXT =
LIVE-FLY** — confirm a future PLAN-LOST episode holds cleanly in HOLD_LOST with no VISUAL_RECOVERY
flicker in the log, and that a genuine PLAN-STALE episode still reaches VISUAL_RECOVERY and this
time actually executes a real turn (still unobserved on any real flight so far)._

_**Session 41 — added a live distance-to-goal readout to the visualizer's telemetry panel (operator
ask, no bug behind it).** The operator wanted the drone's straight-line distance to its current
frontier goal visible on the telemetry panel (session 37) whenever the plan is valid. `TOPIC_PLAN`
already carries everything needed — `pos` and `goal`, both `[x, z]` world coords, already used the
same way for the map panel's own goal marker — so this was a one-function, additive change in
`render_telemetry_panel` (`visualizer.py`): a new `GOAL     dist=<value>u` line in the existing
plan-valid branch, `--` if `goal` is `None` (e.g. `DONE`, no active goal), same NO-SILENT-FALLBACK
pattern as every other reading on the panel. No bus/topic/perception/autopilot change. Smoke-tested
by calling `render_telemetry_panel` directly against four synthetic payloads (valid+goal,
valid+no-goal/DONE, stale, no-control) — all four composed without error. See
`plans/session41-visualizer-goal-distance.md`. **Doesn't change what's next** — session 40 (TRIM's
vertical-pulse rebuild) is still the pending live-fly item; just watch the new GOAL line track
distance shrinking during that flight too._

_**Session 40 — replaced TRIM's pitch-aim+forward-push+ring-gate mechanism with a direct vertical
pulse, diagnosed off flight `20260722_124351` (BUILT — self-tests green, live-fly PENDING).** The
operator asked about `slam_slow_hop_after_s`'s counter behavior after a long stuck episode,
proposing a `settle_trust_s` cumulative-slow-time fix. Traced the flight precisely: the 24.5s
`SLAM_HOLD` wait resumed to `"SETTLE"` (a recovery path explicitly excluded from
`slam_slow_hop_after_s`'s forced-hop rescue, which is scoped to `_slam_resume == "ADVANCE"` only)
— so the proposed fix wouldn't have applied. The *actual* multi-minute stall turned out to be
`TRIM`'s `"ring blocked fwd+back+sides -> skip trim (pray)"` abort re-triggering in a loop with no
give-up cap, every time the height sag re-fired, while the sag ratio kept worsening because the
abort never corrects anything. (Also confirmed `slam_slow_hop_after_s` structurally cannot fire
during `RETURN_TO_ORIGIN`/any postlude state — those losses divert to `POSTLUDE_LOST_HOLD` before
ever reaching `SLAM_HOLD`.) The operator then asked why TRIM needs horizontal room at all instead
of a brief direct vertical nudge. History check: session 14 built the pitch+push trick specifically
because a pure `joy_vertical` pulse was found to choke SLAM — but `DOCK_FLOOR` already uses direct
`joy_vertical` pulses successfully today, because it was later rebuilt around a proper
pulse→settle-gate→re-measure cycle instead of a continuous/un-gated push. The session-14 finding
was about a *continuous* push, not a brief, gated one — so the operator's proposal reuses an
already-validated pattern rather than reopening a settled risk. Deleted the pitch-aim/REPOS/
ring-gate machinery entirely; TRIM is now one short `joy_vertical` pulse (`trim_pulse_s`, default
0.16 — the operator's own number, matching `home_refine_strafe_s`'s precedent) straight into the
existing WAIT/settle-gate, unchanged. The entry trigger and goal-preservation/blacklist-recheck
machinery (`_trim_exit`/`_trim_resolve_resume`/`TRIM_RESUME_WAIT`) are untouched — orthogonal to
how the correction is flown. Retired six now-meaningless knobs/fields (`trim_aim_s`/`trim_fwd_s`/
`trim_reposition_s`/`trim_pitch_up`/`trim_throttle`/`trim_reset_s`/`_trim_repos_move`) — confirmed
via grep unused elsewhere (the similarly-named `io_bridge.py` hits are the separate manual `t`/`g`
trim-macro system, left untouched and flagged to the operator as now diverging in feel from the
autonomous mechanism). Net effect: the "ring blocked -> pray" retry loop is structurally
impossible now, not just capped. Rewrote the `HEIGHT-TRIM` self-test (dropped the ring-gate-only
sub-tests, rebuilt the climb check to drive TRIM with the ring blocked on all four sides and
confirm it still pulses instead of aborting) and the SESSION-22 bidirectional-TRIM test's DOWN
check. `python autopilot.py --self-test`: both rewritten blocks PASS; the same two pre-existing,
unrelated failures from session 39 (`explore ALTITUDE-LOCK`, `explore PRELUDE arm+takeoff+...`)
remain untouched and still open. See `plans/session40-trim-vertical-pulse.md` for the full trace +
design. **NEXT = LIVE-FLY** — this reverses a documented session-14 finding under a different
justification (brief+gated vs. continuous), so watch SLAM tracking quality closely around the
first live TRIM firing after this change; confirm the pulse feels like a brief hop, not a lurch;
confirm goal re-aim after TRIM still lands cleanly. Also still open: the two pre-existing
self-test failures above, and the manual-macro/autonomous-mechanism divergence noted above._

_**Session 39 — removed `RETURN_TO_ORIGIN`'s `BACKOFF` sub-phase (operator's call) + fixed DONE
resurrecting the whole mission after a plan loss (diagnosed off flight `20260721_233244`) (BUILT —
self-tests green, live-fly PENDING).** Walked the operator through what the postlude ending
(`RETURN_TO_ORIGIN`→`ORIENT_HOME`→`HOME_REFINE`→`DOCK_FLOOR`→`LOW_STANDOFF`→`DONE`) actually does
after he flagged it "acting up." Two things came out of it. (1) **Removed homing's `BACKOFF`
sub-phase**: session 26 added a clearance-stand-off reaction to `RETURN_TO_ORIGIN`'s `ADVANCE`,
mirroring explore's own `ADVANCE->BACKOFF` — the operator's call: homing always turns to face the
true origin before advancing, so a properly-oriented leg isn't expected to hit a wall the way
frontier exploration can; removed the check and the sub-phase entirely (explore's own
`ADVANCE->BACKOFF` untouched). (2) **A real bug**: last night's flight reached `DONE` cleanly at
`23:53:58.784` ("EXPLORE COMPLETE" logged once), but a `PLAN-LOST` four seconds later (plausible
near the floor) fell through to the *generic* explore recovery path instead of the dedicated
`POSTLUDE_LOST_HOLD` every other postlude stage already uses — because `DONE` was missing from
`POSTLUDE_STATES`. Once `status` read `OK` again, the generic recovery convergence forced a
`REPLAN`, which re-committed a corner goal and resumed the *entire* explore FSM: the log shows
repeated `BUMP`/`BACKOFF` reverse-thrust cycles from `23:54:13` on (the operator's reported "flying
backwards like a maniac") and a `TRIM enter (DOWN)` at `23:55:03` firing because `pos_y` was
already near ceiling territory (the reported "jumped to the ceiling") — the mission un-retired
itself after already declaring itself complete. Fixed by adding `"DONE"` to `POSTLUDE_STATES`; the
existing `_step_postlude_lost` machinery already resumes whatever state it diverted from, so a
loss in `DONE` now just holds and quietly resumes `DONE` (no new resume-phase branch needed,
mirroring the session-24 `STUCK` corner-giveup precedent for a state that must own its own
recovery). New self-tests for both fixes; cross-checked by reverting both edits in a scratch copy
and confirming the new tests correctly flip to FAIL there, while two *pre-existing, unrelated*
failures (`explore ALTITUDE-LOCK`, `explore PRELUDE arm+takeoff+...`) reproduced identically on the
reverted copy too — confirming those predate this session and aren't a regression introduced here
(still open, still need their own diagnosis). `python autopilot.py --self-test`: all touched tests
PASS. See `plans/session39-return-to-origin-backoff-removal-and-done-loss-fix.md` for the full
trace + design. **NEXT = LIVE-FLY** — watch for: no `BACKOFF` phase logged during homing; a loss
while parked in `DONE` now logs "plan loss DURING DONE" and quietly resumes `DONE`, never
re-entering `REPLAN`/`BUMP`/`BACKOFF`/`TRIM`. Also still open: the two pre-existing self-test
failures above, unrelated to this session._

_**Session 38 — config override to force a fixed desired flying height instead of live calibration
(BUILT — self-tests green (incl. a fixed pre-existing config-drift regression), live-fly PENDING).** The
operator wanted a knob for repeatable test flights: `desired_height_override_y` (0 = disabled, use
`CALIB_VERIFY`'s live measurement as usual; non-zero = use that value directly). Design: override the
VALUE, not the calibration PROCESS — the ceiling-tap (ASCEND/DESCEND/`CALIB_VERIFY`) still runs and
`_ceiling_y` is still measured live (a legitimate self-calibrating platform signal TRIM's sag/high band
needs regardless), only the *desired_y* half of the measurement (`settled_y`) gets replaced when the
override is non-zero, so `_trim_delta = override - ceiling_y` still tracks the real room. Applied at both
places `target_altitude_y`/`_desired_y` get set (the primary `CALIB_VERIFY` PASS latch, and the rarely-hit
fallback latch used only when `CALIB_VERIFY` never ran, e.g. `--no-takeoff`). Flagged the tension with
CLAUDE.md's "NO MANUAL-FLIGHT DATA LEAKAGE" standard explicitly (a fixed height IS a room-specific answer)
— proceeding because it's an explicit, visible, opt-in OPERATOR override (default 0, same pattern as
`use_rewind_on_stale`/`use_visual_recovery_on_stale`), for testing only, never left on for a real survey.
While verifying, found + fixed an UNRELATED pre-existing regression: the operator had separately flipped
`use_visual_recovery_on_stale` true in `config.yaml` (to live-test session 36's path), and 8 legacy
recovery self-tests silently assumed the shipped-off default via the shared `cfg` — the exact "config-drift"
class of gap session 30 already hit once with `calibrate_on_goal_change`. Fixed the same way: force the
flag OFF on the self-test's own `copy.deepcopy(cfg)`, right next to session 30's identical fix for the other
knob (the dedicated VISUAL RECOVERY tests already use their own separate deepcopy forced back to `True`, so
their coverage is unaffected). `python autopilot.py --self-test` and `python flight_replay.py --self-test`:
ALL PASS. See `plans/session38-desired-height-override.md` for the full design. **NEXT = LIVE-FLY** — set
`desired_height_override_y` to a plausible negative value (e.g. -1.9) and confirm `target_altitude_y` reads
it immediately post-calibration instead of the measured settle, and that TRIM still corrects sag/high
relative to it._

_**Session 37 — replaced the visualizer's dead "DEPTH DISABLED" panel with live autopilot telemetry, built
on a separate branch/worktree while session 36 (above) ran concurrently in the main checkout (BUILT —
self-tests + rendering smoke-tests green, live-fly PENDING).** The operator wanted height (actual +
desired), plan status, and the current FSM state (`ADVANCE`/`TRIM`/`SETTLE`/...) visible at all times,
in the space DA-V2 depth used to occupy. Traced why none of it already reached `visualizer.py`: it only
ever subscribed to `perception_state_port` (pose/map/plan/target); the FSM state string and the
self-calibrated altitude-lock hold target (`ExploreController.target_altitude_y`) only ever rode
`TOPIC_CONTROL` on `autonomy_control_port`, read only by `io_bridge.py` (to drive Unity) — pos_y and
plan-status were already there via `TOPIC_PLAN`. Fix, no new bus/port: `autopilot.py`'s `_full_vector()`
now also carries `target_altitude_y`; `visualizer.py` opens a second, independent subscriber on the same
control-bus port (ZMQ PUB/SUB gives extra subscribers for free, same pattern already used for the frame
bus) and a new `render_telemetry_panel()` replaces `render_depth_panel()`. Built in a git worktree
(`worktree-visualizer-telemetry-panel`, branched from `leg-hops-and-goal-commit-fix`@`801e2f8`) to avoid
fighting the other session over the one checkout; merged `new-visual-recovery` in afterward via a clean
fast-forward + stash-pop once that session's work landed as a real commit (only conflict: both sessions
had written a "session 36" entry in this exact file — resolved by renumbering this one to 37). `python
autopilot.py --self-test`: ALL PASS (the new parameter is additive). `visualizer.py` has no self-test
scaffold; verified instead with a standalone script rendering the new panel against synthetic
control/plan payloads (no-control placeholder, control-without-plan, valid plan, `PLAN-STALE` plan) — all
composed without error. See `plans/session37-visualizer-telemetry-panel.md` for the full design. **NEXT =
LIVE-FLY** — does the panel visibly track FSM-state transitions and TRIM's height correction in real
time; does the "waiting for autopilot on the control bus" placeholder show correctly if `visualizer.py`
starts before `autopilot.py`._

_**Session 36 — image-based visual recovery on PLAN-STALE: a loss-instant SIFT check + a first-class 15°
rotational turn-probe, BUILT per an operator-approved design (self-tests green, live-fly PENDING).** The
operator's framing: FALLBACK's blind wait→turn→push sweep recovers a stale plan unreliably, and the answer
to "why did tracking drop and what do I do about it" is sitting in the live NDI image, never used for
recovery before this session. Two agents drafted competing plans (a "Sonnet" plan and this "ALT" one, see
`plans/session36-visual-recovery-15deg-probe.md` for the divergence notes); the operator picked the ALT for
its tighter integration with session 34's existing loss-instant check, its default-OFF safety, and its
independent 15° step structure. Built new `visual_recovery.py` (CPU-only SIFT+RANSAC — copied, not imported,
from `benchmark_detectors.SiftDetector`, so no torch/LightGlue loads for a module that must stay light while
SLAM is busy relocalizing): caches the last frame SLAM was TRACKING on ("F_LKG") every tick, and on request
matches a live frame against it (`matched`/`contained`/`planar_like`/`scale`). Wired into TWO points, per the
operator's decision tree: (1) `_maybe_loss_snapshot_backoff` (session 34's "Idea B") gains a visual clause
alongside its cached-clearance one — a CONTAINED (zoomed-in crop) or PLANAR-LIKE (flat-surface) match at the
loss instant is the same "too close" verdict, closing the exact gap Idea B's geometry-only check couldn't
(an unmapped wall reads "clear" by ray-cast clearance alone); (2) if THAT is also inconclusive, a new
`VISUAL_RECOVERY` state runs an explicit 15°-step open-loop turn-probe (re-matching after each turn) BEFORE
ever falling to the blind FALLBACK sweep — a re-match that reads CLOSER (scale ≥ 1.15) backs off, FARTHER
waits (bounded) for SLAM to re-anchor, and an exhausted budget (720° with no re-acquire) hands off to
FALLBACK with a LOUD event. Found + fixed two real bugs while building it: the visual "too-close" clause was
initially gated only on `visual_match is not None`, not on the `use_visual_recovery_on_stale` flag itself —
harmless in practice (the flag already gates whether `run_explore` ever computes a match) but broke the
flag-off regression as an explicit property of the function; and a TRUE-fresh `VISUAL_RECOVERY` entry
inherited whatever maneuver's `self._player` was mid-flight when the loss hit (e.g. an in-progress ORIENT
turn), silently skipping the probe's own 15° turn — fixed by clearing `self._player` on a genuinely fresh
episode, mirroring how BACKOFF/SETTLE already do this. Default OFF (`use_visual_recovery_on_stale`, mirrors
`use_rewind_on_stale`'s precedent). New `flight_replay.py` "Visual Recovery" floating panel (session-29
Clearance-tab pattern) shows the probe's phase + last match verdict at the cursor. `python
visual_recovery.py --self-test`, `python autopilot.py --self-test`, `python flight_replay.py --self-test`:
ALL PASS (plus `perception_worker.py`/`frontier_planner.py`/`map_store.py`/`io_bridge.py`/
`flow_contact_detector.py` confirmed unaffected). See `plans/session36-visual-recovery-15deg-probe.md` for
the full decision tree + design + divergence notes. **NEXT = LIVE-FLY** (flag OFF by default — flip
`use_visual_recovery_on_stale: true` for the first test flight) — see "Next" below._

_**Session 35 — fixed a real bug where `_recovering` could get structurally stuck for the rest of a flight,
plus a config switch between the classic SLAM-slow step-back and a new forced-hop escape (BUILT — self-tests
green, live-fly PENDING).** The operator asked why the "SLAM slow → REWIND step-back" mechanism had produced
zero events across the last 11 flights. Traced it: `_recovering` (armed on the first `PLAN-STALE` of a loss)
was only ever cleared by a confirmed `>=1.0u` displacement measured from `_recovery_adv_start` inside
`ADVANCE` — but `_enter()` wiped that anchor on every transition that wasn't `"ADVANCE"`/`"SLAM_HOLD"`,
which includes `SETTLE`/`REPLAN`/`ORIENT` — i.e. every ordinary hop boundary (`hop_duration_s`=2.0s). So the
confirm distance could only ever be measured within a SINGLE hop, never accumulated across several.
Verified directly against `20260721_134052`: 80 separate `ADVANCE` runs after the first `PLAN-STALE`, and in
every one the logged `pos` was identical start-to-end — confirmation was structurally impossible for the
rest of that flight, and since step-back is gated on `not self._recovering`, it stayed jammed for the same
reason. Discussed the fix with the operator and simplified rather than patched: `_recovering`/
`_history_broken` now clear as soon as a loss recovers to a genuinely SETTLED `OK` (the existing settle-gate
— several consecutive fast, fresh frames — already runs first regardless), not a further confirmed-motion
step on top of it — judged acceptable since `use_rewind_on_stale` already defaults off (session 31) so the
history-freeze protection this gates isn't consuming much today anyway, and the confirm-distance check
wasn't delivering it regardless (the bug above). Separately, added the earlier-discussed "SLAM slow + plan
OK for 30s → force one hop toward the current goal" idea as a genuine alternative to step-back, selected by
a new `use_slam_stepback_on_slow` switch (default `false`, mirrors `use_rewind_on_stale`'s exact pattern —
operator: "I want to eventually throw out that REWIND bullshit... but we might also want to bring it
back"). Found + fixed a real bug while building the forced-hop's self-expiring bypass window: setting
`_slam_slow_hop_deadline` before calling `_enter("REPLAN", now)` had it immediately wiped by that same call
(`"REPLAN"` isn't in `_enter()`'s exemption list) — fixed by reordering. Also closed a related leak: a
physical guard (clearance stand-off) cutting a forced hop short into `BACKOFF` now clears the deadline
immediately, so it can't linger into an unrelated later leg. `python autopilot.py --self-test`: ALL PASS.
See `plans/session35-slam-slow-strategy-switch-and-recovering-fix.md` for the full trace + design.
**NEXT = LIVE-FLY** — does `_recovering` now visibly clear at "SLAM settled" instead of staying stuck; does
a slow-but-OK patch force a hop after ~30s; flip the switch on one test flight to confirm step-back still
works now that trust restores faster._

_**Session 34 — two proactive clearance checks so nothing is ever completely blind to a close wall (diagnosed
off flight `20260721_014631`) (BUILT — self-tests green, live-fly PENDING).** The operator asked why the
drone sat 0.25-0.5 units from a wall (well inside `stop_clearance_dist: 1.0`) for minutes without the
clearance stand-off/BACKOFF ever re-firing, and whether the "SLAM too slow -> step back" protection had been
removed. Traced it: the clearance stand-off check lives entirely inside the `ADVANCE` state handler; that
flight spent almost no time in `ADVANCE` (336 ticks) and nearly all of it cycling
`SLAM_HOLD`/`HOLD_LOST`/`FALLBACK` (12893/3624/2787 ticks) — being that close to a wall degrades SLAM, which
keeps the drone bouncing through holds instead of ever completing a clean ADVANCE where the stand-off could
re-check. Nothing else watches wall proximity while holding — the flow contact detector needs ~1.2s of
*sustained motion* to latch, so a stationary, hovering drone never trips it either. (Separately confirmed the
SLAM-too-slow step-back is intact, not removed — it's gated by `not self._recovering`, and `_recovering` only
clears on a confirmed 1.0u ADVANCE, which the drone never achieved during the constant OK/LOST/STALE flicker
near this wall, so the gate silently suppressed it the whole time; a real gap, but a different, deliberate
one than "removed.") The operator proposed two ideas, both built this session: **Idea A** — at the exact
moment the post-recovery settle gate clears (the SAME "pose is trustworthy enough to resume" boundary the
code already uses), check the now-live clearance and back off immediately instead of falling through
SETTLE→REPLAN→ORIENT and only re-checking once ADVANCE resumes. **Idea B** — cache the last-known-good
position + clearance every valid tick (the map itself is already frozen the instant tracking drops, so it's
inherently still current); the INSTANT a loss is first detected, run one check against that cached snapshot
and back off right away, rather than waiting out a possibly-long blind period for a re-lock at all — scoped
to a single one-shot attempt at the very first tick of the loss episode (before any hold/recovery logic has
had a chance to command motion), which sidesteps needing to track every possible motion-causing state.
Both reuse the EXACT existing clearance-stand-off action ADVANCE already runs (`_register_bump` + `BACKOFF`)
— no new stopping mechanism, just two new trigger points. Idea B keeps `perception_worker.py`'s own
no-silent-fallback invariant untouched (perception still honestly reports nothing new during a loss) — the
caching + decision to act on an explicitly-labeled stale snapshot ("stale pose @ loss" in the event string)
is a distinct, visibly-logged autopilot-side judgment call. New self-test block covers both ideas plus two
regression cases (a plain mid-leg SLAM-slow hold resuming to ADVANCE is unaffected; a clear reading at the
recovery settle-gate-clear still resumes normally). `python autopilot.py --self-test`: ALL PASS. See
`plans/session34-proactive-clearance-while-blind.md` for the full trace + design. **NEXT = LIVE-FLY** — does
a loss near a wall now back off immediately instead of sitting through the blind period; watch for any
BACKOFF firing off a stale cached pose that turns out to be wrong (the one accepted risk from Idea B)._

_**Session 33 — a permanently-blacklisted goal kept getting re-picked because the clearance inset ran AFTER
the exclusion check (diagnosed off flight `20260721_005658`) (BUILT — self-tests green, live-fly PENDING).**
The operator flagged the end of that flight's timeline: an endless "goal reached → re-picked → reached
again" loop, and the Goals DB showed one disc with **49 picks** despite the loop guard correctly firing and
PERMANENTLY blacklisting it after just the 3rd pick. Traced it end to end in `frontier_planner.py`:
`_select_reachable()` filters candidate frontiers by `_excluded()` against each frontier's RAW centroid,
then runs the chosen one through the clearance-inset function (`ground_grid.inset_to_clearance`, wired up in
`perception_worker.py`), which walks the goal back TOWARD THE DRONE until it finds a free, buffered cell —
and that adjusted point, not the raw centroid, is what actually gets committed/published/logged. The
exclusion check never re-ran against it. In this flight the drone was pinned in one spot with its entire
reachable free space in that direction pinched into one small pocket — exactly where the already-dead
frontier sat — so every "different" (technically not-excluded) raw candidate the utility function turned up
got inset right back onto that same dead cell, defeating the blacklist entirely: each cycle looked like a
fresh pick, was already almost on top of the drone (`goal_reach_dist: 1.0` is generous) so it read as
instantly "reached," then REPLAN picked "again." Confirmed via the timeline's millimeter-scale `plan_goal`
drift cycle to cycle, and via the Goals DB's `is_corner` flag staying `False` throughout — ruling out the
corner-sweep tour (which deliberately bypasses `_excluded` by design, a different and intentional escape
hatch) as the source. **Fix:** `_select_reachable()` now re-checks `_excluded()` against the POST-inset
point; if it's still dead, that candidate is dropped and the next-best reachable frontier is tried instead,
looping until a genuinely clear one is found or the list is exhausted (falls through exactly as if nothing
had been reachable that cycle). Also added a structured `WARNING` `planner_event` (not a bare `print()` —
routed through the same mechanism `LOOP-BLACKLIST`/`BUMP` events use, so it shows up in both the console and
the timeline/`flight_replay.py` debugger) if a pick ever lands on an already-excluded goal again — a loud
canary rather than a silent repeat of this bug, per the operator's explicit ask. New self-tests in
`frontier_planner.py` reproduce the exact failure (a blacklisted dead spot + a genuinely-reachable candidate
whose injected clearance_fn collapses onto it) and confirm the fix drops it for the next-best candidate,
plus the all-candidates-collapse case correctly reports nothing reachable. `python frontier_planner.py
--self-test`, `python autopilot.py --self-test`, `python perception_worker.py --self-test`, and `python
ground_grid.py --self-test`: ALL PASS. See `plans/session33-goal-loop-clearance-inset-fix.md` for the full
trace + design. **NEXT = LIVE-FLY** — does a drone pinned in a corner with no genuinely reachable space now
fall through to the corner-sweep tour instead of looping; does any blacklisted disc's pick count stay flat
after being blacklisted; the new WARNING event should never fire._

_**Session 32 — ORIENT_HOME real-angle convergence (fixes a diagnosed live ping-pong) + a new HOME_REFINE
position-tightening stage + DOCK_FLOOR real settle-gate (BUILT — self-tests green, live-fly PENDING).**
The operator pointed at the tail of flight `20260720_223555`'s raw timeline: from `22:55:37.228`,
`ORIENT_HOME` alternated `turn +30 deg (err +15.1)` / `turn -30 deg (err -17.4)` / ... forever, never
reaching `DOCK_FLOOR`. Root cause: `ORIENT_HOME`'s turn command went through `_quantize_turn`, which snaps
the bearing error to the nearest whole `turn_step_deg` (30) — it could only ever command 0° or ±30°, never
the actual residual. Once that residual sat near half a step (~15°, exactly what the log shows), each
open-loop 30° turn overshot to the OTHER side by a similar margin, and the "done" check (`|err| < 15`) sat
on that same knife-edge — so it could loop forever. Fixed per the operator's own two-part diagnosis: (a)
turn by the REAL (still clamped to `turn_step_deg` for the same SLAM-survives-a-turn safety, but no longer
forced to a multiple of it) bearing error — the open-loop recipe already supported a continuous angle, the
quantization was self-imposed; (b) an explicit `orient_home_tol_deg` (5°) convergence tolerance, decoupled
from the turn clamp. Also added, per the operator's explicit request, a bounded `orient_home_max_s` (60s)
give-up cap mirroring `RETURN_TO_ORIGIN`'s own `home_max_s` idiom (VISIBLE, no silent fallback) so a
persistently-noisy heading can't hang the ending forever either. Explained to the operator what happens
next if this had resolved: `ORIENT_HOME` → `DOCK_FLOOR` (pulsed descent) → `LOW_STANDOFF` (up-nudge) →
`DONE` (terminal hover) — the "come home and land" tail; target localization itself already happened
earlier during mapping. Two follow-up asks landed in the same session: (1) a new **`HOME_REFINE`** state,
inserted between `ORIENT_HOME` and `DOCK_FLOOR`, that tightens the resting POSITION against the true origin
(config `home_fine_reach_dist`, 0.15) using ONLY short full-throttle push pulses — never a continuous
ADVANCE — picked by body-frame quadrant each cycle (forward/backward `home_refine_fwd_s`=0.32s, ramps as
usual; left/right strafe `home_refine_strafe_s`=0.16s, never ramped, per the operator's exact numbers),
settled between pushes, with its own `home_refine_max_s` (45s) give-up cap; (2) **`DOCK_FLOOR`** now settles
(6 fresh frames, the same primitive every other maneuver-loop already uses) after each descent micro-pulse
instead of waiting a fixed `dock_rest_s` timer and reading whatever pose happened to be sitting in `plan` —
the same class of stale-frame gap session 24's settle-gate rewrite fixed everywhere else, just never applied
here since `DOCK_FLOOR` was added later (mirroring `ASCEND`, which keeps its own fixed-timer REST — not
touched, not asked for, flagged as the same class of gap for later). New self-tests reproduce the diagnosed
bug directly (a bearing error starting just past half a turn step, under a REALISTIC noisy 1.4x-overshoot
open-loop turn model, converges in 2 turns instead of oscillating forever) plus quadrant-pick/convergence/
cap coverage for `HOME_REFINE`; three pre-existing postlude tests needed small `_max_s=0` overrides so their
synthetic (non-physical) drive loops don't burn their tick budget on a give-up cap that isn't what they're
testing. `python autopilot.py --self-test`: ALL PASS. See
`plans/session32-orient-home-ping-pong-and-home-refine.md` for the full trace + design. **NEXT = LIVE-FLY**
— none of this can be exercised without hardware in this dev environment._

_**Session 31 — REWIND killed (config-gated off), FALLBACK rebuilt as a simple 4-phase sweep, operator ask
off many real flights (BUILT — self-tests green, live-fly PENDING).** The operator: "That REWIND mechanism
is annoying the living fuck out of me. I didn't see it help a stale plan ONCE IN MY LIFE." Rather than
delete it, gated it behind `use_rewind_on_stale` (config.yaml, default `false`) — one edit to bring back,
per the operator's own suggested approach. Separately, session 29's FALLBACK (a shuffled-direction-queue
with per-direction tries + opposite-phase retry) tested badly live: locking a push direction across several
tries gave bad, unpredictable results. Replaced it with the operator's own exact algorithm: wait 20s → turn
15° → push a FRESH random direction (fwd/bkwd/lft/rt, re-rolled every cycle, no per-direction budget) → wait
10s → repeat until 720° cumulative rotation → STUCK. Kept the live wall/backwall-contact early-exit
("leave it for the slim chance we WILL sense the wall") — confirmed it needs ~1.2s sustained motion to
latch, which the new push durations (below) now comfortably clear. Push throttle/duration came from the
operator's own manual-flight comparison: forward/backward full throttle (1.0) held 2.0s including ramp-up;
left/right full magnitude (±1.0) held 0.5s (`joy_horizontal` isn't ramped, unlike `trigger`/`reverse`) — both
bypass the throttled knobs (`reverse_throttle`, `_strafe_mag`) that every other site still uses. While
wiring the REWIND gate, found + fixed a real bug: the first draft checked the REWIND flag before the
pre-existing `_ever_tracked` startup guard, so with REWIND off a PLAN-STALE at STARTUP (before SLAM ever
tracked) skipped WARMUP and went straight into a blind sweep — reordered so the startup guard always wins
regardless of the flag. Rewrote the FALLBACK self-test block entirely (initial wait, full TURN→PUSH→WAIT_POST
cycle, live-contact early-exit, 720° exhaustion, flicker-persistence across a `HOLD_LOST` bounce) and added
explicit `use_rewind_on_stale = True` overrides to every pre-existing test that still needs to exercise
REWIND now that it's default-off. `python autopilot.py --self-test` and `python io_bridge.py --self-test`:
ALL PASS. See `plans/session31-rewind-off-simple-fallback-sweep.md` for the full design. **NEXT = LIVE-FLY**
— does the push direction look genuinely randomized, does the 2s full-throttle push carry visible authority,
does recovery still cut the sweep short the instant status reads OK._

_**Session 30 — BACKOFF rebuilt as a phase-timer: hard gate cut + full-magnitude 2s reverse, diagnosed off
flight `20260720_210809` using the session-29 Clearance tab (BUILT — self-tests green, live-fly PENDING).**
The operator flagged a BACKOFF firing mid-ADVANCE and suspected it "never executes" (tied to several prior
crashed flights). Traced it: the autopilot's state machine and `cmd` output were fine (ADVANCE→BACKOFF
transitions cleanly, `reverse: 0.2` emitted for the whole 0.3s recipe) — the real lag was one layer down,
in `io_bridge.py`'s session-18 throttle smoothing (shared by manual AND autonomous flight). Going from
`trigger=1.0` to the old throttled `reverse=0.2` took trigger ~10 ticks (~167ms) to decay while reverse
only took ~4 ticks (~67ms) to ramp up, and the boolean thrust gate Unity actually gates on was *derived
from the ramped analog*, not the freshly-commanded boolean — so both gates could read `True` at once during
that window, eating a meaningful chunk of BACKOFF's already-short 0.3s reaction time. The operator then ran
a manual experiment (full throttle → release trigger → immediately hold reverse) and found it takes ~2
SECONDS of held reverse for the right effect — io_bridge's own ramp math only explains ~167ms of that; the
rest is very likely Unity's own physics/momentum once thrust reaches the sim (a black box from this side of
the socket). Rebuilt BACKOFF entirely around that finding, using the platform's OWN already-characterized
ramp rates (10 ticks down / 20 ticks up, unchanged) rather than inventing new ones: a new `gate_override`
flag (`io_bridge.py`, strictly opt-in, every other emit site unaffected) lets `trigger_down`/`reverse_down`
flip the INSTANT they're commanded instead of waiting for the ramped analog to catch up; BACKOFF itself is
now a phase-timer (not a `flight_playbook.json` recipe) — hard-cut trigger + full-magnitude (1.0, not the
throttled `reverse_throttle`) reverse held for `backoff_hold_s` (2.0), then release + a short open-loop
wait (`backoff_release_s`, 0.2) for the ramp-down to finish before SETTLE. Scoped to the top-level
`"BACKOFF"` state only (its 3 entry sites: clearance stand-off, wall-contact, leg-timeout) — homing's own
backoff sub-phase and `BLIND_BACKOFF` keep their current recipe-based behavior. Also found + fixed, while
extending self-test drive windows for the new ~2.2s duration: two OTHER pre-existing tests
(`RECOVERY control-space`'s FALLBACK case, `SESSION-12`'s consuming-REWIND-drain case) were silently broken
by a live `config.yaml` retune of `recovery_settle_max_s` (2.5→10.0) that happened between sessions — fixed
by giving both their own local override, same pattern several other tests already use, so they're robust to
future tuning of that knob instead of silently assuming its value. `python autopilot.py --self-test` and
`python io_bridge.py --self-test`: ALL PASS. See `plans/session30-backoff-hard-gate.md` for the full trace +
design. **NEXT = LIVE-FLY** — the 2-second hold duration came from exactly one manual test; expect to retune
`backoff_hold_s` after watching it live._

_**Session 29 — Clearance-detail debugger tab + direction-cycling blind recovery sweep, diagnosed off the
session-28-build flight `20260720_180112` (BUILT — self-tests green, live-fly PENDING).** The operator
asked about a ~113s stuck episode: plan recovered from PLAN-LOST, went PLAN-STALE a frame later, and the
drone spent the whole episode visibly turning/pushing/getting straightened back out by a wall (per the
operator's account — the telemetry itself was frozen the whole time, SLAM being blind) before giving up
into STUCK, which then sat next to solve-times-look-normal SLAM for ~83 more recorded seconds. Root cause
of the wall-bounce: `_begin_fallback()`'s push direction was picked from `self._last_ring`, a snapshot
frozen from BEFORE the loss and never refreshed — as the (correct, unidirectional) turn sweep accumulated
real heading change across 31 attempts, that stale judgment grew increasingly wrong and could repeatedly
push the drone right back into the same wall, which naturally re-aligns (straightens) the nose on contact,
erasing the sweep's own progress each cycle. **Fix:** replaced the ring-derived pick with a direction-
cycling search — cycle a shuffled [forward, backward, left, right] queue, `fallback_dir_tries` attempts per
direction then its opposite, a live wall/backwall contact ends a forward/backward attempt early (operator's
explicit call: forward pushes are now ALLOWED while blind — "we might as well be with our back to the wall
and a push forward will save us"; no live signal exists for left/right, so those run their full budget), 2
complete passes with no recovery -> STUCK. While rewriting this, found + fixed a THIRD bug: the existing
`fallback_max_attempts` cap was only ever checked from FALLBACK's own internal continuation path, never
from the top-level PLAN-STALE re-entry a flickering connection actually takes (exactly what this flight
did) — it silently reached 31 attempts against a configured cap of 16. Now unified inside `_begin_fallback`
itself, the one place every caller funnels through; `fallback_max_attempts` raised 16→70 in config.yaml so
the new 2-lap search (64-attempt worst case) can normally complete on its own terms, with the cap staying
as a backstop. The STUCK-next-to-"healthy"-SLAM question turned out to have a clean, non-bug explanation
(SLAM's solve TIMES looked normal but every solve reported `dx:+0.00 dy:+0.00` for ~83s straight — far more
consistent with a non-tracking/relocalizing mode repeating a frozen pose than genuine re-acquired tracking;
STUCK's own "logging paused" design is why the jsonl can't confirm this directly) — **operator declined a
fix to STUCK's logging this session.** Separately, built the requested **Clearance details tab**: a new
`detail=True` mode on `map_store.clearance()` exposes the raw ray-hit picture (hits/rays/fraction/closest/
farthest/the vote's outcome) behind a fwd/back/left/right judgment, published from `perception_worker.py`
and rendered in a new floating panel in `flight_replay.py` (mirrors the existing Goals DB panel). All
touched module self-tests green (`autopilot.py`, `map_store.py`, `perception_worker.py`, `flight_replay.py`
— new tests for both the direction-cycling FALLBACK mechanics and the clearance-detail plumbing). See
`plans/session29-clearance-tab-direction-cycling-fallback.md` for the full trace + design._

_**Session 28 — diagnosed the session-27 flight's loop bug (three parts) off `20260720_135307`'s raw
timeline; fixed two, documented one pending evidence (BUILT — self-tests green, live-fly PENDING).**
(1) The goals-DB's picks/strikes/bumps appearing to jump together in one tick, and a BUMP/BLACKLIST line
repeating 33x, turned out to be a pure OBSERVABILITY artifact, not a logic bug: two consecutive SLAM solves
took 10.48s and 9.13s back to back, and `perception_worker.py`'s main loop is fully SYNCHRONOUS — it can't
drain autopilot-event pulses or publish a fresh plan while blocked inside one slow solve, so ~19s of two
genuinely independent, correctly-decided real-time events (a hop-progress judgment, a live flow-based wall
bump -> 2-bump blacklist) only became visible in one batched tick once the solve finally returned; the
33x-repeat is the same mechanism at smaller scale (perception published once; the autopilot just re-logged
its still-held plan on every one of its own faster control ticks). No code change — this is almost
certainly also why bug (3) below can happen, but making SLAM solving async is a much bigger change than
this session. (2) **Found + FIXED why the drone kept flying at a goal it had just blacklisted**: the ONE
queued chance to REPLAN (adopting a fresh goal) after a SLAM-loss recovery got hijacked by the height-TRIM
trigger, which — on an at-entry ring-blocked abort — RESTORED `leg_goal` from a pre-blacklist snapshot and
re-aimed at it instantly, off whatever pose happened to be sitting in the current plan, never going through
REPLAN again because SLAM died for good ~4s later. A follow-up question ("shouldn't a REPLAN-class
transition require a SLAM frame captured after the last command, like the session-24 settle-gate already
does for the normal path?") sharpened the fix: TRIM's abort path was bypassing that exact gate. Rebuilt
`_trim_exit()` to hand off to a new `TRIM_RESUME_WAIT` state that waits for the settle-gate (a provably
fresh post-TRIM frame) before resolving, and re-validates the preserved goal against the live blacklist at
that point — a permanently-dead goal now falls through to the same SETTLE->REPLAN convergence a genuinely
new leg uses (which, via the existing session-24 pick-dedup, still avoids polluting the goals-DB when the
goal turns out unchanged — Trap B's original intent, preserved). (3) **plan-stale -> fallback -> spin ->
stuck against the wall — documented, NOT fixed** (operator's explicit ask: wants a visualizer clip before
finalizing a direction). Separately, while auditing TRIM's "ring blocked on all sides" judgment, found +
fixed a related gap in `map_store.clearance()`: it took the MIN hit across a ray fan, so ONE isolated (but
still `min_count`-qualified) noisy voxel was enough to call an entire direction blocked — added a
`min_hit_fraction` vote (config `clearance_min_hit_fraction: 0.3`, a general ratio not a room-specific
value) shared by the forward stand-off, the ring, TRIM, and PARALLAX_PUSH. All touched module self-tests
green (`autopilot.py`, `map_store.py` — which gained its first `run_self_test()` — `perception_worker.py`,
`frontier_planner.py`, `flight_replay.py`). See `plans/session28-trim-resume-gate-clearance-vote.md` for
the full trace + design. **NEXT = LIVE-FLY** (see "Next" below) — this is a genuine tradeoff (MIN-over-fan
was chosen to catch a thin/off-axis wall a single ray could thread) so watch for BOTH false-opens (ramming
a real thin wall) and whether the false-blocks the operator observed actually go away._

_**Session 27 — visualizer video recording + SLAM point-cloud export on quit + graceful shutdown for
all three processes (BUILT — self-verified, no GPU/hardware in this environment to live-fly it
directly).** Two feature requests: save the visualizer dashboard as video without adding GPU load, and
export the SLAM point cloud (Blender-loadable) on quit. Both easier than expected: `visualizer.py`
already owns no GPU (pure display, composes one BGR image/tick already) — recording is just a
`cv2.VideoWriter` fed the same composed frame, wall-clock throttled (`--record`/`--record-fps`, default
15). `map_store.py` already had a working `save_ply()` (Blender-loadable ASCII PLY, true-color voxels +
green flight path + magenta targets) plus `save_npz`/`render_topdown`, proven by the OFFLINE `--video`
export path — the real gap was that NONE of it ever ran for a LIVE flight, because `fly.py` hard-
`terminate()`s `perception_worker.py` (launched `--no-display`, so no `'q'`-quit path either) on stop,
skipping its `finally:` entirely. Gave it the same `--stop-file` sentinel `autopilot.py` already uses
for exactly this reason, and wired the three already-proven export calls into `finally:`
(`OUTPUT/diag/<ts>_livemap.{ply,npz}` + `_livemap_topdown.png`). Then found a bug in the FIRST feature's
own shutdown, in the very same session: `--record`'s video was left in `fly.py`'s generic
hard-terminated process list, so a normal `fly.py` stop corrupted the MP4 (confirmed by reproducing it
directly — a hard-killed writer leaves `mdat` with no `moov` atom, the frame index every player needs;
a cleanly-`release()`d one has both). Fixed by giving `visualizer.py` the identical `--stop-file`
treatment. `fly.py` now tracks `autopilot`/`perception`/`visualizer` as three separately-sequenced
graceful-stop steps (generic `processes` list hard-terminates only what's left: io_bridge + sim).
Verified end-to-end by actually hard-killing and gracefully-stopping the real modules and inspecting
the resulting MP4's box structure both ways (see the plan doc for the exact bytes). See
`plans/session27-video-recording-pointcloud-export-graceful-shutdown.md`. **NEXT: the operator has a
NEW bug from a live flight to diagnose (see top of file) — likely session 28.**_

_**Session 26 — homing back-off + settle-gate stale-frame fix + postlude recovery budget + pick-dedup
fix (BUILT — self-tests green).** Two more flights, diagnosed the same way as before (line-by-line off
the raw `_timeline.jsonl`, not the console `.log` — a couple of early wrong conclusions this session
came from under-checking a hypothesis, corrected once verified against the jsonl directly). Flight
`20260719_233845`: the drone hopelessly bounced a wall at the very end of `RETURN_TO_ORIGIN` homing (7
PLAN-LOST/OK flips in ~1m45s, pinned at clearance 0.25) and, mid-flight, repeated the same wall-bump
3× in a row on a corner goal. Three compounding causes: (1) homing's own `ADVANCE` sub-phase had NO
`back_off` reaction to a clearance stop (unlike explore's `ADVANCE->BACKOFF`) — new
`_home_phase=="BACKOFF"` sub-phase fixes it; (2) `SETTLE`'s "prequalified" freshness shortcut let it
finish having seen ZERO frames captured after the maneuver it was judging — `_slam_window_ready`
gained a `latest_since` check (keeps the shortcut for the bulk of the window, but the newest frame
must still postdate the gate); (3) the `home_max_s` safety cap couldn't fire because
`POSTLUDE_LOST_HOLD`'s stricter recovery-streak gate never got satisfied while SLAM kept flickering —
rejected forcing a blind state transition (would livelock against the `POSTLUDE_STATES` router +
violates no-blind-recovery) in favor of relaxing the streak requirement itself once a new
`postlude_recover_budget_s` is blown, still gated on `status=="OK"`. Flight `20260720_024455`: a
SEPARATE bug — the drone "reached" the same frontier 40+ times in a row without ever advancing,
because reaching a goal is unconditional progress (never a strike) and the ONE mechanism that could
have broken the loop (the goals-DB's picks-based circling guard) was starved — `REPLAN`'s pick-dedup
(session 24) suppressed every one of those 40+ genuinely-completed hops as if they were a single leg's
own re-orient sub-steps, since it only checked goal POSITION, not whether a hop had actually been
judged. Fixed by consulting `prev_goal` (`_hop_start_goal`), already computed right there. Found +
fixed two pre-existing self-tests that had baked each bug in as "expected" (`settle-gate two-gate
design (g2)`, `PICK DEDUP dup_suppressed_ok`) — both updated to verify the corrected behavior instead.
All self-tests green. See `plans/session26-homing-backoff-settle-freshness-pick-dedup.md` for the full
trace + evidence. **NEXT = LIVE-FLY** (alongside the still-pending sessions 20b-25 checklist)._

_**Session 25 — manual TRIM key-macros + three recovery-FSM bugs diagnosed off the `20260718_010045`
flight + goals-DB mechanism-split schema + debugger event-log navigation.** The operator flagged seven
things after replaying that flight; three turned out to be genuine bugs found by tracing the actual
timeline JSONL + autopilot.log line-by-line (not guesses). (1) **Manual `t`/`g` TRIM UP/DOWN key macros** —
new `trim_up`/`trim_down` recipes in `flight_playbook.json` (mirror the autonomous TRIM's AIM→FWD→RESET
motion, ring-gate/height-threshold decision stripped) played by `io_bridge.py` independent of
`autonomy_active`; freed `g` by rebinding object-detect to `h` (any manual flight key cancels an
in-progress macro). (2) **Lossy planner-event mailbox**: `perception_worker.py`'s `last_planner_event` was
a single overwritable string, destructively read-and-cleared once per SLAM solve — during a slow solve
(5-8s+), an earlier bump/strike message could be silently clobbered before ever being logged, which is
exactly the "goal jumps from 0 strikes straight to BLACKLISTED with nothing in between" the operator saw
at 01:05:07. Now an accumulating list, joined on consume; nothing is dropped. (3) **Blind-hold wall
contact was ignored**: the clearance/back-off check only ran from inside ADVANCE; a drone parked in
HOLD_LOST/SLAM_HOLD for 30-40s during a bad SLAM patch (confirmed 01:17:19-01:18:07) never got a chance to
react even though the flow contact detector (SLAM-independent) was firing the whole time. New
`BLIND_BACKOFF` state (owns every status while it plays, like CALIB_ESCAPE) reacts to a live wall/backwall
contact from either hold, plays `back_off`, then resumes the SAME hold — edge-triggered so a sustained pin
doesn't replay it every tick. (4) **SLAM_STEPBACK counter never escalated**: `_slam_stepback_count` reset
on every fresh `_enter_slam_hold`, but a genuinely bad SLAM patch always bounces PLAN-LOST→HOLD_LOST→OK
before the next hold (confirmed 01:31:09-01:32:33: `#1/3` fired three times running, never reaching `#2`
or `#3`). Now persists across that bounce, resetting only at a trusted REPLAN or a genuinely new committed
goal. (5) **Goals-DB mechanism-split schema** (operator ask, after auditing the corner exemption — it's
already proximity-gated, not blanket: a NEAR corner is bumped/struck exactly like a frontier, only a FAR
one gets the give-up counter) — every `_goal_db` disc now also carries `bumps`/`corner_giveups`/
`is_corner`, and every `_blacklist` entry records WHICH mechanism (`2bump`/`stall`/`loop`) killed it plus a
float-cast evidence dict (position/strikes/picks/spread/slam_ms); the debugger's Goals DB panel shows the
new columns + reason + evidence. (6) **Debugger event-log navigation** — a global `ALL_EVENTS` list (state/
planner/missed-bump/SLAM records, built once) makes every log line clickable (jumps the scrubber to its
time) and adds Prev/Next message buttons + an "incl. SLAM msgs" checkbox (off by default), so scrubbing
between the non-SLAM lines — previously the hard part — takes seconds instead of hours. New self-tests for
all of the above; all 6 module self-tests green. See
`plans/session25-trim-macros-recovery-fixes-goaldb-schema-debugger-nav.md` for the full design + file
list. **NEXT = LIVE-FLY** (alongside the still-pending sessions 20b/21/22/23/24 checklist) — watch for:
intermediate strike/bump messages now visible in the event log (no more single-tick blacklist jumps); a
`back_off` firing during a HOLD_LOST/SLAM_HOLD stretch if genuinely near a wall; `SLAM_STEPBACK #2/3`/
`#3/3` reachable on a sustained bad patch; the Goals DB panel's new bumps/giveups/reason columns; click/
Prev/Next navigation in the replay debugger; `t`/`g` trim macros in manual flight._

_**Session 24 — settle-gate rewrite (rolling-window two-gate design), pick-pulse dedup, bounded/scaled
far-corner exemption.** Four more issues off the same `20260717_102403` flight, independent of session 23.
(1) **Double SETTLE wait after a SLAM-loss recovery**: `SLAM_HOLD`'s exit (3 fast frames) already proved SLAM
healthy WHILE STATIONARY, but entering `SETTLE` then re-demanded 6 BRAND-NEW frames from scratch — a genuine
architecture problem (a single streak counter conflating "is SLAM healthy" with "has the airframe rested long
enough"), not a two-site patch. Rebuilt as a rolling `(slam_ms, cap_ts)` window decoupled into a FRESHNESS
gate (full + healthy + capture-timestamped — a stale/timestamp-less stream can never look "already clean",
caught in review) and a PHYSICAL-MOTION gate (`settle_gate_s` dwell, opened at the TRUE stationary-start
instant so a hold's own duration already counts toward it); `SLAM_HOLD`'s exit now uses this gate for EVERY
resume target (`SETTLE`, and — newly gated, previously a weaker no-dwell 3-frame check — `ADVANCE`/
`PARALLAX_PUSH`). Deliberately scoped OFF the calibration-recovery holds (`CALIB_LOST_HOLD`/`CALIB_ESCAPE`/
`POSTLUDE_LOST_HOLD` keep the old counter, per the operator — separate, already-validated mechanism).
(2/3) **LOOP-blacklist fired on a multi-step turn's own re-orient sub-steps**: every `REPLAN` re-commit
(including a same-goal one mid multi-turn ORIENT→PARALLAX_PUSH→SETTLE→REPLAN cycle) counted as a fresh
goals-DB "pick" — confirmed as the exact cause of `goal=[4.65, 8.25]` (a sweep corner) getting
`LOOP-BLACKLIST`ed while the drone kept flying toward it (corners ignore `_excluded()` by design, so the
blacklist was real but inert — just a misleading log line). Fixed per the operator's own proposed rule: a
same-goal re-commit (reusing the existing `goal_moved` check) suppresses only the PICK half of the pulse; the
hop-outcome/strike half still judges every hop. (4) **Far-corner exemption smarter + bounded**:
`corner_no_blacklist_dist` (flat 1.0u) is now overridden live by `corner_span_half` (half the room's own known
corner-to-corner diagonal, from `perception_worker`'s `bbox_corners`); a NEW persistent, proximity-keyed
give-up counter (not a single reset-on-switch slot — a reviewer caught that oscillating between two
unreachable corners would defeat that) force-retires a corner after `corner_giveup_limit` (10) give-ups,
same as a real 2-bump, without ending the mission by itself; the mission only ends in a HARD STUCK hold (not
the graceful dock) once EVERY corner is exhausted this way — caught a real bug while testing this: the
generic step()-top recovery convergence would otherwise immediately bounce this new terminal STUCK back out
since `done` stays permanently True, needed a `_corner_giveup_stuck` guard on BOTH that convergence and
STUCK's own resume check. New self-tests for every fix above; all 6 module suites green. See
`plans/session24-settle-gate-pick-dedup-corner-giveup.md` for the full design + file list.
**NEXT = LIVE-FLY** (alongside the still-pending sessions 20b/21/22/23 checklist)._

_**Session 23 — wired the flow BACKWALL detector into a real decision (was DETECTION-ONLY); PARALLAX_PUSH now
retries a side + remembers a give-up.** Diagnosed a ~30s stuck loop in flight `20260717_102403` (starts
`10:27:09.454`, ends `10:27:37`–`10:27:39` when an unrelated height-TRIM branch happened to break it): the
drone oriented away from a wall SLAM hadn't mapped yet, so the clearance ring at 180° read "open" and
`PARALLAX_PUSH` picked BACKWARD — the log's own BACKWALL detector fired twice (`10:27:23`, `10:27:35`) but was
logged `"detection-only, no reaction yet"`; every push instead ran the full 2.0s reverse timer into the wall,
re-oriented, and repeated (heading swung 132°→70°→93°, SLAM died twice, position barely moved). Built: (1)
`backwall_contact` is now a real `ExploreController.step()` input, mirroring `ceiling_contact`; (2)
`PARALLAX_PUSH`'s backward branch, on a ring block OR a live BACKWALL contact, calls a new shared
`_pick_ring_direction()` helper (extracted from the existing entry-tick backward/strafe/give-up pick)
EXCLUDING backward, and hands off to a side strafe IN-PLACE (same episode, no settle/replan/re-turn) — this
also upgrades the EXISTING ring-based mid-push block, which previously bailed straight to settle/replan
without ever trying a side; (3) `REVERSE_PROBE` (default-enabled on a forward WALL hit) now ends its reverse
recipe early on a live BACKWALL contact instead of only its fixed 4.0s timeout; (4) a give-up (backward AND
both sides blocked) LATCHES the drone's position (`_parallax_back_blocked`) so the next pick — even a leg
later, after settle/replan/re-orient — doesn't immediately retry backward at the same spot just because the
ring still (falsely) reads it as open; cleared once the drone has moved `parallax_min_clear` away
(SLAM-freeze-safe, mirrors `rearm_bump_if_disengaged`). New self-tests (retry->strafe, both-sides-blocked->
give-up, ring-only block also retries, give-up memory latch+clear, REVERSE-PROBE-BACKWALL); all 6 module
suites green. **NEXT = LIVE-FLY** (alongside the still-pending sessions 20b/21/22 checklist below) — watch
for a `parallax backward blocked (...) -> strafe_...` / `-> no room back/left/right either` line instead of
the old silent `"(timer)"` grind._

_**Session 22 — fixed height reference + BIDIRECTIONAL TRIM; the mid-flight ceiling re-tap is RETIRED.** The
session-21 live-fly (`20260717_004418`) hit a calibration death-loop (~2¼ min): the goal-change re-tap fired in
a SLAM-hostile corner, the vertical ASCEND lost the plan on EVERY attempt, each redo threw the height around,
and the drone ended GLUED AT THE CEILING (y≈-2.30 ≈ ceiling; desired was -1.855) with NOTHING able to bring it
down (altitude lock injects UP only; TRIM climbed only) — while the rolling median followed the error. The log
also CONFIRMED the operator's key hypothesis: SLAM's height read is STABLE within a flight (consistent pos_y
across every loss/re-lock). Rebuilt on that: (1) the periodic re-tap is OFF by default (code kept) — the
FIRST-takeoff calibration's `desired_y` is THE flight's height reference; (2) TRIM is now BIDIRECTIONAL — TRIM
UP on a sag (`pos_y > ceiling+1.2·delta`), TRIM DOWN when glued high (`pos_y < desired−0.2·delta`, new
`trim_high_ratio`), same goal-preserving machine with a mirrored pitch aim (+1.0), and `trim_aim_s` is now an
automatic 0.5 s platform constant (io_bridge's ±0.05/tick aim ramp saturates in ~0.33 s; the aim is held through
the push); (3) a **SLAM-COMFORT gate** — calibration redo/retry (and any re-enabled periodic tap) requires the
rolling average of healthy-frame latencies < `calib_slam_avg_ms` (666) on a full window, not merely "6 alive
frames" (the bad flight's redos passed on 616–797 ms marginal frames and died in every ASCEND); a redo gated
past `calib_gate_max_s` counts a failed attempt WITHOUT launching (escalates to CALIB_ESCAPE = relocate);
(4) **Y-DRIFT audit posture** — re-enable later with `calib_cooldown_s: 600` and every non-first PASS logs the
ceiling movement vs the first tap; (5) CALIB_VERIFY PASS latches `target_altitude_y = settled_y` (one verified
reference everywhere) + a once-per-flight LOUD `HEIGHT-REFERENCE DISAGREEMENT` notice if the median wanders >
delta from desired (visible drift backstop); (6) the debugger HEIGHT panel shows the full band (`trim-at-high` /
`trim-at-low`, pos_y red outside either side). New SESSION-22 self-test block (7 asserts); all 6 module suites
green. **NEXT = LIVE-FLY.**_

_**Session 21 — RESTORED the periodic height re-calibration + gradual TRIM + the height debugger panel.** The
drone does NOT hold altitude — it sags, wrecking flights. Session 17 deleted this machinery believing the sag
was self-inflicted (the unset `triggerDown`); live flights proved it real. Restored from `44b4fa6` and adapted
to the current branch: (1) the **periodic re-tap** — a genuine goal change (>1u) past a configurable
`calib_cooldown_s` (60 s) → `CALIBRATING_HEIGHT` → CALIB_VERIFY, whose PASS re-measures the three LIVE
references (ceiling_y from the ASCEND climb peak, desired_y = the settled post-descend pose, delta) and logs
them LOUD; (2) the **gradual TRIM** — pos_y sinking past `ceiling + 1.2·delta` in SETTLE/ADVANCE fires a
ring-gated PITCH-aim + forward climb (guards stay active; triggerDown derives centrally) that re-aims the SAME
snapshotted goal on exit; (3) the **debugger HEIGHT panel** — live pos_y (red past the sag threshold),
ceiling/desired/delta, the `trim-at` threshold, median, and a TRIM/CALIB activity flag. Session-20b
integrations: a recalib REPLAN emits a hop-outcome-ONLY pulse (the pick registers post-calib, once per leg);
TRIM entry clears the pending per-hop eval (a trimmed hop takes no strike). Review hardening: never-calibrated
(`_last_calib_t is None`, e.g. `--no-takeoff`) ALLOWS calibration instead of locking it out; the TRIM trigger
None-guards its refs (can't fire pre-calibration); the WAIT gate is phase-relative on cap_ts (stale frames can't
exit early); the post-calib resume is a θ≈0 'c'-only ORIENT (no thrash). New self-tests: HEIGHT-TRIM (9 asserts)
+ PERIODIC-RECALIB (4) — the harness disables the trigger globally so unrelated leg tests aren't diverted.
**NEXT = LIVE-FLY.**_

_**Session 20b — per-hop progress + strikes (kill the instant-stall death-loop) + goals DB in the debugger.**
Flight `20260716_140437` froze re-picking one goal forever ("leg STALL … 75.0s" every leg, never blacklisted).
Cause: the session-20 leg-stall guard fired the INSTANT ADVANCE began (its stall clock never reset across
same-region re-picks; the drone was farther than its stale best-dist), bailing to SETTLE **before emitting any
forward command** → the drone never moved; and neither blacklist path caught a stationary re-pick of one goal
(the 2-bump latch can't re-arm on a frozen drone; the goals-DB counted only a DIFFERENT disc). Rebuilt per the
operator's tightened rules: (1) a stall is now a MEASURED CONSEQUENCE — on ADVANCE entry snapshot the distance to
the goal; at the next REPLAN, a hop that closed < `hop_progress_eps` (0.2u) is a STALL. (2) The goals-DB is fed by
the AUTOPILOT once per leg (a combined pick+hop-outcome pulse on TOPIC_AUTOPILOT_EVENT, mirroring the bump pulse)
and holds THREE complementary, non-blocking guards, all writing the same permanent blacklist: **2-bump** (twice
physically touched), **strikes** (2 hops in a row no closer → dead; reset on real progress), **picks-loop** (≥3
picks with ALL drone-locs inside one 1u cluster → circling; TIGHTENED from "any pair <1u", which false-fired on a
legit marching approach over short hops — the debugger's per-pick drone-location rows made it visible). (3) A FAR
corner (>1u away) is exempt from strike + bump — a
corner is a reposition target flown from afar, unlike a nearby frontier. Removed the old leg-stall guard + its
trackers + region-gate. Also added the **goals DB to the replay debugger** — a draggable floating "Goals DB"
table (center / picks / strikes / locs / status) that updates as you scrub. New knobs `hop_progress_eps` (0.2),
`goal_strike_limit` (2). All 6 module self-tests green (rewrote the HOPS test → HOPS+PER-HOP-STRIKE; new planner
strike/loop tests db1–db5; flight_replay goal_db test). **NEXT = LIVE-FLY:** a blocked goal should strike 1→2
then blacklist (watch the floating table + a `STRIKE-BLACKLIST` / `LOOP-BLACKLIST` event), a far corner survives
a transient stall, and the drone never freezes on one goal._

_**Session 20 REV — de-commit the hops + a persistent goals database + corner-goal safety (BUILT on
`leg-hops-and-goal-commit-fix`; `main` untouched as the clean fallback).** The prior STEP-1 experiment
(committed-goal hops, below) flew badly, and the operator diagnosed WHY session-19 flies smooth: **SLAM is let to
re-pick its goal freely** — the drone must NOT harden its life by committing to one distant goal. But free
re-picking re-opens **goal ping-pong** (the planner oscillates between a few goals, the drone circles, and the
2-bump watcher goes blind because its counter resets on every goal change). Fix, three parts. (1) **Keep the
40-tick hop cadence, remove the COMMITMENT**: the post-hop SETTLE now routes to **REPLAN** (was resume-`ADVANCE`),
so every hop re-reads SLAM's current goal and, if it changed, adopts it — re-orient WITH the parallax scout →
hop — instead of finishing the old, unreached leg. (2) **A persistent goals DATABASE (`frontier_planner`)**:
each picked goal is a 0.5u DISC; a genuine goal-switch registers a "pick" (holding one goal across the 2 Hz
selects counts once); a disc picked ≥3× with any two pick-time drone locations <1u == circling → **PERMANENTLY
blacklist it** via the SAME store the 2-bump uses. The DB **persists the whole flight, never reset mid-flight** —
that is what lets a slow loop accumulate across goal changes (immune to the "counter defeated" hole). (3)
**Corner-goal safety**: SLAM stays free to find + adopt a frontier en route to a corner (free, since a corner
cruise is itself hopped + re-planned); and a sweep CORNER goal farther than `corner_no_blacklist_dist` (1.0) from
the drone can NEVER be bumped/blacklisted — a mildly-stuck-then-freed drone must not retire a far corner. Kept:
the **leg-stall guard** as a safety (its tracker reset is now region-gated so per-hop re-planning can't neuter
it) and **`forward_throttle: 1.0`**. All 6 module self-tests green (rewrote the hop test → HOPS-NO-COMMITMENT; new
goals-DB tests db1–db5). **NEXT = LIVE-FLY (`python fly.py`, m)** — watch each hop re-pick, a ping-pong loop retire
in a handful of picks via `LOOP-BLACKLIST` (no 3-min ram / "counter defeated" thrash), and a far corner survive a
transient stall. Return-to-origin (orient-to-north + gentle descent) remains a PRE-EXISTING bug for a later step._

_**Session 20 STEP 1 (SUPERSEDED by the REV above — was: committed-goal HOPS on main):** ADVANCE hopped
`hop_ticks` ticks then RESUMED the SAME committed `leg_goal` (`_settle_to="ADVANCE"`, no REPLAN). This
COMMITMENT is exactly what the REV removed (post-hop → REPLAN). The leg-stall guard + `forward_throttle 1.0`
carried forward; the `_settle_to="ADVANCE"` resume did not._

_Session-18 (below) + 17 are committed (a737aa4). Session-19 is on branch `session19-profiled-forward-leg`;
this work is on `leg-hops-and-goal-commit-fix`._

_Last updated **2026-07-15** (session 18 **BUILT — io_bridge + autopilot + flight_replay self-tests green;
LIVE-FLY PENDING**). Resume from THIS file. **NEXT = LIVE-FLY** (`python fly.py`, press `m`) to confirm session
18 AND the still-pending sessions 17/16/15/14/11-13 in one go; then **RE-TUNE the throttle knobs** (session-17
"lower the speed knobs"; turn durations are unaffected — yaw is no longer ramped). Plan of record:
**`plans/session18-command-smoothing-and-height-median.md`**._

_**Session 18 — manual-style command SMOOTHING for autonomy + a real height-median (BUILT):** the operator
noticed autonomous flight is height-erratic (hard brake + pitch-up + altitude jump on every stop / plan-loss)
while his manual flight is "very very controlled." Root cause (found by diffing the `20260715_001039` manual
command CSV): manual keys only toggle the `trigger_down`/`reverse_down` (and arrow) GATES, and io_bridge's 60 Hz
loop RAMPS the analog toward them (`+0.05`/tick attack, `−0.1`/tick decay; yaw/pitch `±0.05` aim). The autopilot
BYPASSED all of it — `_apply_autonomy_overlay` hard-wrote the analog after the ramp, and `_neutralize_autonomy`
snapped to 0. Fix: the autopilot's **`trigger`/`reverse`** are now RAMP TARGETS the existing loop
chases (new `_ramp` + `_auto_*_target`; `_update_controls`→testable `_step_controls`), so thrust eases in/out
like a hand-flown stick while KEEPING the throttle magnitudes; release decays smoothly (aim axes + gates
still snap for safety). **yaw/pitch are NOT ramped** — live-flight showed the turn is duration-not-magnitude (the
sim eases the aim itself and the drone only rotates once the aim REACHES ±1), so ramping stole ~0.33 s from every
turn (30°→~5°); they pass straight through, restoring the calibrated turn recipe. Also **re-added `--log-commands`** (the reverted session-17 outgoing-packet CSV) — now
permanent + always-on via `fly.py` — so MANUAL vs AUTO smoothing is diffable. Second, independent fix: the
debugger's **drone-height median** was appended every ~50 Hz tick with no frame dedup (re-appending one stale
pose ~25×) and seeded with ~0 ground samples pre-takeoff — hence the −0.008→−1.8 jump-with-no-new-frame and the
lag. Now it ingests ONE reading per FRESH SLAM frame (`frame_id` dedup), only after the first calibration
(`_height_calibrated`), frozen during any calibration; `MAPPING_ALT_STATES` retired. New io_bridge
`--self-test` (ramp) + rewritten autopilot ingest-gate test; all green. **LIVE-FLY PENDING. CAVEAT: smoothing
attenuates short pulses (a 1–2-frame reverse tap / brief turn reaches less than commanded before the next
command) — expect to RE-TUNE throttle knobs AND maneuver durations / back-off counts on the first flight.**_

_**Session 17 — THE BIG ONE (BUILT):** while diagnosing the broken height TRIM we built temporary
io_bridge diagnostics (a `t` trim macro, a `y` replay, a `--log-commands` full-packet CSV) and, by diffing a
hand-flown trim against the macro, found the root cause of MONTHS of pain: **the Unity sim gates real thrust on
the `triggerDown`/`reverseDown` BOOLEAN, NOT the analog `trigger`/`reverse`.** The autopilot had NEVER set it
(`AUTONOMY_FIELDS` omitted it; io_bridge even decays the analog to 0 unless the boolean is held), so every
autonomous forward/reverse ran with the gas button UNPRESSED — the near-certain explanation for the legendary
~0.02-0.04 u/s "crawl". The operator also confirmed the drone HOLDS ALTITUDE on its own during horizontal
flight; it only climbs uncontrollably when flying FORWARD or STRAFING into a wall (reverse doesn't) — so the
periodic height-calibration + TRIM were fighting a self-inflicted sag. Built: (a) `trigger_down`/`reverse_down`
added to `AUTONOMY_FIELDS` (io_bridge) + `_neutralize_autonomy`, and DERIVED CENTRALLY in `autopilot._full_vector`
(the single choke point) from the analog value — so EVERY forward/reverse emit site engages thrust; (b) KEPT the
first calibration + flight-height median + all calib recovery; (c) DELETED the periodic re-calibration trigger
and ALL of TRIM (state/trigger/exit/vars/config/self-test). All six module self-tests green. LIVE-FLY pending;
expect to re-tune speed knobs afterward. A wall-hit-triggered re-calibration is the next FUTURE item (the kept
`CALIBRATING_HEIGHT` machinery + median exist for exactly it). The Step-0 diagnostic scaffolding was reverted
(`git restore io_bridge.py`)._

_**Session 16** (`plans/session16-settle-between-stages-and-return-to-origin.md`, **BUILT + committed 44b4fa6,
live-fly PENDING** — will be confirmed on the same flight as session 17): a test flight's
return-to-origin fell apart — it "turned like a maniac," fired the reverse-list back-to-back with no settles,
then spun (no settles), declared STUCK, retried. One pattern in three places: commanded actions fire
back-to-back with no still window for monocular SLAM to re-lock. Built a **shared settle gate** (`_settle_begin`
/ `_settle_poll`, healthy + lost-SLAM flavors; the SETTLE state now calls it) and put a settle between EVERY
action: (1) **REWIND** inverse maneuvers, (2) **spin FALLBACK** attempts (both lost-SLAM flavor, bounded by
`recovery_settle_max_s` so a dead pipeline still re-exposes) — this resolves the session-15 parked
"reverse-without-settling". Also **flipped the FALLBACK order to turn→push** (was push→turn) so the parallax
translation is the LAST motion before the settle (rescues the rotation for RELOC; matches the 'c'-reset-then-push
recipe). And built the **full return-to-origin ending**: `home_reach_dist` 1.0→0.5, a new
**ORIENT_HOME** state facing the recorded `_takeoff_heading`, a **POSTLUDE_LOST_HOLD** so the dock survives a
SLAM loss (mirror of CALIB_LOST_HOLD), `_POSTLUDE_NOLOCK` to stop floor re-inflation, and homing settles
(`PLAN→TURN→SETTLE→ADVANCE→SETTLE→PLAN`). All module self-tests green; **live-fly PENDING.**_

_**Session 15** (`plans/session15-trim-and-settle-fixes.md`): six fixes off the session-14 TRIM
flight (`20260714_113312`). (1) TRIM **pitch was reversed** → `trim_pitch_up=-1.0` (now climbs). (2) A
calibration **endless loop** (finish→lose-plan→retry) is now bounded: after 3 consecutive failed attempts a
new **`CALIB_ESCAPE`** state does a ring-picked push to a fresh vantage + holds for SLAM (12 frames + OK) then
retries; 3 more fails → **STUCK** (logging paused). (3) **SETTLE** no longer flies on a stale pose — a
goal-flying settle waits for **6 SLAM frames captured AFTER the settle began** (`cap_ts ≥ entry`) and
<1000 ms; the vertical prelude routine is exempt. (3b) **CALIB_VERIFY**'s timeout no longer PASSes-and-flies on
a stale pose — it feeds the same escape/STUCK guard. (4) Debugger shows **live height numbers** (ceiling /
desired / delta / all-flight median), dropped Δpos/Δgoal. (5) **fly.py** console flood fixed (restored
`NEW_CONSOLE`). All module self-tests green; **live-fly PENDING**. Parked: reverse-without-settling — diagnose
on the next log (the SETTLE gate may already fix it). Sessions 11/12/13/14 items fold into the same re-fly._

_**Session 14** (`plans/gradual-height-trim.md`): flight `20260713_223231` flew great (height-calib + parallax
fixes worked; **`CALIB_LOST_HOLD` fired 3× live and recovered cleanly — session 13 LIVE-PROVEN**). Built the
operator's **gradual height TRIM**:
a fine, dose-able altitude correction BETWEEN calibrations that uses the sim's PITCH aim (pitch the aim UP +
push forward → fly toward the raised aim = a gradual climb; the forward part feeds SLAM parallax, unlike a
discrete full-thrust `joy_vertical` pulse that chokes SLAM). At each calibration we now record ceiling_y /
desired_y / delta; on a fresh healthy frame in SETTLE or ADVANCE, if `pos_y` sank past
`ceiling_y + 1.2*delta`, a ring-gated TRIM climbs back (fwd-open → climb; else reverse/strafe to open forward
room; else abort+pray), preserving the committed goal (re-aims ORIENT, never re-picks). Also diagnosed two
things and wrote them up for a fresh session: the **return-to-origin ending** (`plans/return-to-origin-and-
graceful-dock.md`) and the **2-minute glass-wall bounce** (`plans/blacklist-region-and-counter.md`). All
module self-tests green; **live-fly PENDING**. Sessions 11/12/13 items also fold into the same re-fly.
Flight `20260713_101220` flew well then **"lost its shit"** after a parallax strafe; we diagnosed it
fully, wrote **`plans/strafe-throttle-and-recovery-loop.md`** (D1–D5), and **BUILT all five** (49/49 autopilot
self-tests pass; flow/frontier/ground_grid/perception green). **NEXT = live re-fly** of the same far-corner
scenario to confirm the fixes (watch the D1 caveat: does 0.2 actually slow the strafe? and the D2 reposition
displacement). The diagnosis: a full-magnitude strafe (`joy_horizontal −1.0` — strafe was the one axis never throttled
to 0.2) into an UNMAPPED side, while the drone was yawed, **scraped the wall → spun the drone to face it →
monocular SLAM died** (the spin is invisible in the log: the pose froze while the real airframe rotated). Then
a **frantic HOLD_LOST↔REWIND loop for 100+ s that could never die**, because a flickering SLAM status
(PLAN-LOST↔PLAN-STALE) RESET the recovery FSM every ~3 s (`_fallback_attempts=0` + a fresh non-consuming
`_invert_history()`), making `STUCK` mathematically unreachable. Raycast never fired: the forward ray is blind
to a lateral strafe, and the side ring read `None` (unmapped ⇒ treated as open). Session-11 height-calib +
session-10 tour/floor-dock still await their own clean live confirmation._

**Status:** Phase-1 (manual map + target localization) done & hardware-verified. Phase-2 autonomous
**Map-mode explorer** (`autopilot.py --explore`) flies live — clean session-8 flight
(`20260708_195009`). Session 8 confirmed **turns work** (the earlier "no-op" was a stale-heading logging
artifact), made the flight log **trustworthy** (logs the controller's committed goal + data staleness),
and added **`[SLAM_TRACKER]`** telemetry so the async ~2 Hz SLAM ticks are visible in the terminal. Next:
item 2 (REPLAN dead-stall) then item 1 (height calibration).

This file is three-fold: **Next** (resume-after-clear pointer), **Future** (the concise backlog →
plan-of-record pointers), and **Documentation** (what we tried, in date order, below).

## Next (resume after a context clear)

### >>> IMMEDIATE NEXT TASK (branch `all-bets-are-off`) <<<

**LIVE-FLY session 54** (`python fly.py` — full stack), on top of sessions 49-53 (all already built,
all still live-fly PENDING). Watch list, in priority order:

1. **`TRIM` must exit within ~3s of its pulse, even while `slam_ms` reads 1500-2700ms.** Expect
   `TRIM done (...) -> wait for a fresh post-trim frame before resuming` promptly after `TRIM enter`.
   If instead you see `TRIM done (...): FORCED after N.Ns waiting for a post-trim frame`, that's the
   backstop doing its job (not a bug) — but it means the primary fix isn't landing; check `cap_ts` is
   actually advancing. Either way, `TRIM` must never again sit silent like flight `20260902_155916`
   did for 73.9s.
2. **The `SLAM_HOLD`↔`HOLD_LOST` limit cycle still cannot persist past ~18s** (session 53, confirmed
   working on this same flight — watch for it to keep holding, this is now just a regression check).
3. Because sessions 53 and 54's flights were both cut short by these bugs, session 52's own watch
   list (below) is STILL **completely unconfirmed** — re-watch all of it this flight:
   - **`VISUAL_RECOVERY` actually runs**: a loss opening as `PLAN-LOST` reaches the probe on the
     following `PLAN-STALE`, survives the flicker, logs a **MATCH-phase verdict**.
   - `TRIM enter (DOWN)` firing while `slam_ms` is 1500-2500 ms (every flight so far has only shown
     the sag/UP case).
   - **No `LOSS-INSTANT BACK-OFF SUPPRESSED` notice unless a back-off was genuinely about to fire.**
   - **No goal committed inside a permanently blacklisted region** — a `CORNER-SKIP ... force-retired`
     line, not `blacklist bypassed`.
   - An `autonomy OFF -> PAUSED` / `autonomy LIVE (paused N.Ns; recovery state reset)` pair on `m`.
   - The LKG debug window open, inliers drawn, `OUTPUT/diag/<ts>_visrec/` filling.

See `plans/session54-trim-wait-no-exit.md` for the session-54 trace (root cause, both fixes, the
four-state audit table below) and `plans/session52-lkg-recovery-unreachable.md` for the still-
unconfirmed session-52 watch list above.

Sessions 49-51 (below) are also still themselves LIVE-FLY PENDING — the detailed session 45-48
per-flight watch checklists that used to fill this section are superseded and folded into their dated
entries above, same pattern as the 2026-09-01 session-43 confirmation further down this file.

**Session 50** (`plans/session50-settle-dead-band-escape.md`) — watch for
`SETTLE gate blocked N.Ns by slow-but-ALIVE SLAM ... -> forcing REPLAN` in place of the 91.6s park
that flight `20260901_172217` showed. If it fires OFTEN that is not a bug in the fix: it is the honest
signal that SLAM is chronically over `slam_slow_ms`, and the thing to chase is the choke itself.

**Session 51** (`plans/session51-visual-match-on-demand.md`) — pure waste removal, **no behaviour
change expected**. `[VISREC]` lines should appear at roughly the memo cadence rather than being
throttle-limited, the `HOLD_LOST` loop rate should sit nearer 38Hz than 32Hz, and every decision in
the log (grace notice, back-off, probe verdicts) must be **identical** to previous flights. A changed
decision means the memo is serving something it shouldn't — suspect the motion guard first.

**>> STILL OPEN, TOP OF THE LIST: why does SLAM choke? <<** It runs ~350ms when happy and plateaus at
a flat ~2000ms for minutes at a time, and nothing we own explains it. Session 52 measured it across a
whole flight (`20260901_222552`): **median 1943 ms, worst inter-frame gap 13.6 s** — worse than
session 50's ~2000ms plateau, same open problem. **Two candidates are already
ruled out — do not re-chase them:** (1) the AUTOPILOT is not the cause (loop rate measured at 32-38.5Hz
throughout, including through the entire 91.6s wedge); (2) UNITY FOCUS is not the cause either
(operator tested directly: SLAM re-chokes ~2 frames after refocus, and the log shows SLAM recovering
BEFORE the autonomy pause and degrading to 1767ms DURING it while the autopilot was idle). Note the
plateau is suspiciously FLAT (1978/1992/1995/2003ms) — a stable equilibrium, not noisy contention —
and slow solves skip ~129 NDI frames vs ~46 for fast ones, a possible positive-feedback loop worth
probing. Untested leads: MASt3R-SLAM's own workload growth (keyframe graph / retrieval DB size as the
map grows), its backend/optimization thread, and GPU contention from the visualizer's `--record` MP4
encode (which runs regardless of autonomy state). Everything downstream — `slam_slow_ms`, the settle
gate, `slam_slow_hop_after_s` — has been tuned against a machine in this state.

Session 49 built off an earlier flight's tail: a goal-stagnation memory (`goal_disc_max_drift`,
`goal_stagnant_limit`, both new in `frontier_planner.py`/`config.yaml`) that retires a goal the
drone keeps re-attacking without ever getting closer — see `plans/session49-goal-stagnation-and-lkg-window.md`
for the full trace + design — plus an operator-requested LKG debug window
(`visrec_debug_window`, off by default, `autopilot.py`/`visual_recovery.py`/`flight_replay.py`).
Watch for:
- A `PLANNER: ... reason=stagnant` line retiring a wall goal after 2 fruitless legs (`goal_stagnant_limit`)
  instead of the ~8 minutes / ~40 losses / 18-cycle FALLBACK sweep the diagnosing flight showed.
- **No legitimate far goal retired mid-march** — the one accepted risk of the stagnation guard. If a
  genuinely progressing leg ever gets soft-killed, loosen `goal_stagnant_limit` first, not the design.
- The goals-DB debugger panel (`flight_replay.py`'s Goals DB floating table) showing **one** disc
  with a non-zero `drift` where a drifting wall used to show up as two dead-end records.
- If `visrec_debug_window: true` is flipped on for this flight (default false — flip it deliberately
  to test the window): the LKG window's drawn SIFT inlier correspondences visibly piling onto one
  flat surface when the drone is nose-to-a-wall; the window surviving a display hiccup without losing
  the saved PNG evidence under `OUTPUT/diag/<ts>_visrec/` (or vice versa) — both failure modes are
  independent and each logs its own `CRITICAL` line if it happens.
- Still open, deliberately not touched this session: `autopilot.py:2280` still discards a
  plan-loss-interrupted hop judgement, so the STALL guard itself remains starved (stagnation covers
  that failure mode from a different angle now, but the dead-hop gap is still there) — judging
  interrupted hops late needs a trusted-pose story first (this flight's SLAM pose jumped 1.3u between
  ticks), its own session.

Self-tests are fully green (`python frontier_planner.py --self-test`, `python visual_recovery.py
--self-test`, `python autopilot.py --self-test`, `python flight_replay.py --self-test`: 0 failures)
including every new case, each proven against its own defect by reverting on a scratch copy — see
the plan file for the full revert table.

### >>> NEXT TASK AFTER THAT (branch `main`) <<<

**Diagnose the HEIGHT issue.** A live flight on 2026-09-01 confirmed sessions 20-43 (the full
TRIM / BACKOFF / FALLBACK / homing / visual-recovery / SLAM_HOLD backlog previously tracked as a
long per-session live-fly checklist in this section) together fly *tolerably* -- operator's own
call, no further point-by-point re-verification needed. The one problem the operator flagged
coming out of that flight: **height is still an issue.** Not yet diagnosed -- no root cause, no
log pulled yet. Start here next session: get the flight's timeline/replay debugger open and look
at `pos_y` vs. `target_altitude_y` (the altitude-lock hold target, session 37's telemetry panel
shows both live) across the flight, and check whether TRIM (`trim_pulse_s`, currently `0.01` in
`config.yaml` -- much shorter than the `0.16` session 40 built and tested against) is even
correcting sag/high meaningfully at that duration, or whether the problem is elsewhere (calibration,
`desired_height_override_y`, ceiling-tap accuracy, DOCK_FLOOR/HOME_REFINE at the very end, etc.) --
scope still open, don't assume it's TRIM until the log says so.

Each dated session entry above (session 20 through 43) still documents what was BUILT and WHY --
that detail wasn't removed, just the redundant "watch for X on the next live-fly" checklists that
used to fill this section, now superseded by the 2026-09-01 confirmation above.

---

## Future (backlog)
- **The `_slam_fast_streak` dead-band class — three remaining sites (found session 54, not fixed).**
  Same signature each time: the ONLY exit predicate is `_slam_fast_streak >= N` or `not
  self._slam_slow`, with no wall-clock cap and no `has_any_capture` blackout guard — grep for that
  pattern to find more. All three are dispatched ABOVE `step()`'s status router and own every status
  themselves, so unlike `TRIM` (fixed this session) not even a genuine `PLAN-LOST` can rescue them:

  | State | Exit predicate | Why it can't fire at ~2700ms/frame |
  |---|---|---|
  | `CALIB_LOST_HOLD` (`_step_calib_lost`, `autopilot.py:1680`) | `_slam_fast_streak >= calib_lost_recover_frames` (6) + `status == "OK"` | `_slam_fast_streak` resets to 0 on every slow frame, pinned at 0. `calib_gate_max_s` bounds only the downstream comfort sub-gate, never reached. Docstring: "No time cap — the SLAM frame stream is the liveness signal (operator ask)" — that premise assumes slow eventually turns fast; this flight class disproves it. |
  | `CALIB_ESCAPE` (`_step_calib_escape`, `autopilot.py:1791`) | `_slam_fast_streak >= calib_escape_ok_frames` (12) + `status == "OK"` + `_calib_slam_comfortable()` | Same, stricter (12 fast frames + a latency-average bar). Comment: "no extra bound needed here". |
  | `POSTLUDE_LOST_HOLD` (`_step_postlude_lost`, `autopilot.py:1879-1881`) | `_slam_fast_streak >= required_streak` + `status == "OK"` | `postlude_recover_budget_s` (30s) relaxes the required streak 6→1 but NOT the speed bar — a fast frame is still required and there are none. Residual, not absent. Hangs at mission end, near the ground. |

  Plus one **latent** instance (off by default, so not yet bitten anyone): the `SLAM_HOLD` legacy
  `use_slam_stepback_on_slow=True` arm — the sessions-43/53 rescue lives only in the `if not
  self.use_slam_stepback_on_slow:` branch; the legacy arm returns "keep holding" with no clock at
  all. The un-fixed twin of a known-fixed bug, waiting for whoever flips that flag.

  Each fix should mirror `TRIM`'s session-54 idiom (or `TRIM_RESUME_WAIT`'s session-44 original): a
  bounded forced exit on `slam_slow_hop_after_s`, guarded by `has_any_capture` so a total capture
  blackout still isn't papered over by a wall clock. `CALIB_LOST_HOLD`/`CALIB_ESCAPE` both have an
  explicit "operator ask: no cap" comment on record — re-confirm the premise with the operator before
  changing either, since the no-cap choice was deliberate at the time, not an oversight (unlike
  `TRIM`'s, which was). See `plans/session54-trim-wait-no-exit.md` for the full audit table this was
  drawn from.
- **Session-22 — fixed height reference + BIDIRECTIONAL TRIM; mid-flight re-tap RETIRED — BUILT
  (`leg-hops-and-goal-commit-fix`), all 6 module self-tests green, UNCOMMITTED, LIVE-FLY PENDING**
  (`plans/session22-fixed-height-ref-and-bidirectional-trim.md`): the 20260717_004418 calibration death-loop +
  glued-at-ceiling diagnosis; `calibrate_on_goal_change: false` (first-takeoff `desired_y` = THE flight
  reference — SLAM Y stability confirmed in the log); TRIM DOWN (`trim_high_ratio` 0.2, mirrored pitch +1.0);
  `trim_aim_s` automatic (0.5 s platform constant); SLAM-COMFORT gate (`calib_slam_avg_ms` 666 on a 10-frame
  healthy-latency window, `calib_gate_max_s` 30 → escalate WITHOUT redo); Y-DRIFT audit posture (re-enable +
  `calib_cooldown_s` 600; non-first PASS logs the ceiling movement); PASS latches `target_altitude_y`; LOUD
  once-per-flight median-vs-desired disagreement notice; debugger trim-at-high/low band. **NEXT = live-fly.**
- **Session-21 — periodic height re-calibration + gradual TRIM + height debugger RESTORED — BUILT, then the
  re-tap RETIRED by session 22 after its live-fly (kept configurable)**
  (`plans/session21-restore-height-calib-and-trim.md`): goal-change re-tap (configurable `calib_cooldown_s` 60 s,
  `calib_goal_change_dist` 1.0); live refs ceiling_y/desired_y/delta re-measured each CALIB_VERIFY PASS; TRIM
  (pitch-aim + forward climb, ring-gated, goal-preserving) on `pos_y > ceiling + 1.2·delta` in SETTLE/ADVANCE;
  HEIGHT panel (pos_y/ceiling/desired/delta/trim-at/median/active). Session-20b integrations: recalib pulse is
  hop-outcome-only; TRIM clears the pending hop eval. Review hardening: never-calibrated allowed; None-guarded
  trigger; phase-relative WAIT gate; θ≈0 'c'-only resume. **NEXT = live-fly (with the 20b checklist).**
- **Session-20 REV — de-commit hops + persistent goals DB + corner safety — BUILT (`leg-hops-and-goal-commit-fix`),
  all 6 module self-tests green, UNCOMMITTED, LIVE-FLY PENDING** (`plans/session20-goal-db-loop-blacklist.md`):
  hop→REPLAN (re-pick every hop, adopt SLAM's new goal with parallax; `_settle_to="REPLAN"`); `frontier_planner`
  persistent `_goal_db` (goals-as-0.5u-discs, pick count + pick-time drone locs; ≥3 picks with any pair of
  drone-locs <1u → permanent loop-blacklist via the same store; NEVER reset mid-flight); `_register_bump`
  far-corner guard (`corner_no_blacklist_dist` 1.0); region-gated leg-stall tracker reset; kept the leg-stall
  guard + `forward_throttle 1.0`. New knobs under `autonomy.explore`. **NEXT = live-fly + watch LOOP-BLACKLIST +
  far-corner survival.** FOLLOW-UP: return-to-origin (orient-to-north + gentle stepped descent) still pre-existing.
- **Session-18 command smoothing + height-median — BUILT, self-tests green, LIVE-FLY PENDING**
  (`plans/session18-command-smoothing-and-height-median.md`): autopilot trigger/reverse/yaw/pitch are now RAMP
  TARGETS io_bridge's 60 Hz loop chases (manual constants) → smoothed flight + smooth release; `--log-commands`
  re-added (always-on via fly.py); height-median ingests one reading per FRESH SLAM frame after the first calib
  (frame dedup, `MAPPING_ALT_STATES` retired). **NEXT = live-fly + re-tune throttle knobs AND maneuver durations
  (smoothing attenuates short pulses).**
- **Session-17 triggerDown fix + height simplification — BUILT, all six module self-tests green, LIVE-FLY PENDING**
  (`plans/session17-triggerdown-and-height-simplification.md`): Unity gates thrust on the
  `triggerDown`/`reverseDown` BOOLEAN (autopilot never set it → the "crawl"). Added them to `AUTONOMY_FIELDS` +
  `_neutralize_autonomy` (io_bridge) and DERIVED centrally in `autopilot._full_vector` from the analog value;
  deleted the periodic re-calibration trigger + all of TRIM (state/trigger/exit/vars/config/self-test); kept the
  first calibration + flight-height median + calib recovery. **NEXT = live-fly + re-tune speed knobs.**
- **Wall-hit-triggered re-calibration — FUTURE (the next thing to build after session 17 flies).** The drone
  holds altitude on its own EXCEPT it climbs uncontrollably when flying forward/strafe INTO a wall (reverse
  doesn't). Session 17 kept the `CALIBRATING_HEIGHT` machinery + flight-height median (both unwired now)
  specifically so a wall-contact event can trigger a re-calibration judged against the median. To wire: on a
  forward/strafe wall-contact event, `self._recalibrating = True; self._enter("CALIBRATING_HEIGHT")`.
- **Session-16 settle-between-stages + return-to-origin — BUILT (committed 44b4fa6), LIVE-FLY PENDING**
  (`plans/session16-settle-between-stages-and-return-to-origin.md`): confirmed on the same flight as session 17.
- **Session-15 six fixes — BUILT, all module self-tests green, LIVE-FLY PENDING**
  (`plans/session15-trim-and-settle-fixes.md`): TRIM pitch sign (`trim_pitch_up=-1.0`); calibration
  escape/STUCK guard (`CALIB_ESCAPE` + `_calib_fail_escalate`, config `calib_escape_*`); SETTLE fresh-frame
  gate (`settle_fresh_frames`, `_SETTLE_EXEMPT_NXT`); CALIB_VERIFY no-fly-on-stale (`TIMEOUT_FAIL`→escalate);
  debugger live height numbers (`alt_*` + `_alt_median`); fly.py `NEW_CONSOLE`.
- **Settle between every recovery action + full return-to-origin ending — BUILT (session 16), LIVE-FLY PENDING**
  (`plans/session16-settle-between-stages-and-return-to-origin.md`): shared `_settle_begin`/`_settle_poll` gate;
  a settle between REWIND inverse maneuvers and between spin FALLBACK attempts (resolves the parked
  "reverse-without-settling"); `home_reach_dist` 0.5, `ORIENT_HOME`, `POSTLUDE_LOST_HOLD` (dock survives a SLAM
  loss), `_POSTLUDE_NOLOCK` (no floor re-inflation), homing `TURN→SETTLE→ADVANCE→SETTLE`. Knobs
  `recovery_settle_frames`/`recovery_settle_max_s`.
- **Gradual height TRIM — BUILT (session 14), pitch sign fixed session 15, LIVE-FLY PENDING**
  (`plans/gradual-height-trim.md`): PITCH-aim + forward climb between calibrations; 3-value capture
  (ceiling_y/desired_y/delta) at CALIB_VERIFY; `pos_y > ceiling_y + 1.2*delta` trigger in SETTLE/ADVANCE;
  ring-gated (reverse/strafe reposition, else abort); goal-preserving exit. Config knobs `trim_*`.
- **Return-to-origin + graceful dock — BUILT (session 16), LIVE-FLY PENDING**
  (`plans/return-to-origin-and-graceful-dock.md` = diagnosis of record; built per
  `plans/session16-settle-between-stages-and-return-to-origin.md`): home at altitude (`home_reach_dist` 0.5),
  `ORIENT_HOME` to the recorded take-off heading, `POSTLUDE_LOST_HOLD` (DOCK survives a SLAM loss),
  `_POSTLUDE_NOLOCK` kills the floor-level altitude-lock re-inflation.
- **Glass-wall bounce (blacklist region + counter) — DIAGNOSED (session 14), plan written, NOT BUILT**
  (`plans/blacklist-region-and-counter.md`): widen the blacklist region past the frontier spacing + per-region
  bump tallies (stop the `counter defeated` thrash).
- **Calibration survives a plan loss — BUILT (session 13), LIVE-PROVEN (session 14 flight, 3× recover)**
  (`plans/crystalline-swimming-floyd.md`): `CALIB_LOST_HOLD` + `_calib_interrupted`; redo on a 6-fast-frame +
  `status==OK` SLAM-pulse recovery, one DOWN bump if stuck, `status==OK`-gated exit (anti-flicker).
- **Height calibration — BUILT + FLEW (session 11), NOT confirmed good** (`plans/height-calib-state-gate-and-slam-debug.md`):
  state-gated `CALIB_VERIFY`/`ASCEND_ESCAPE`/`CALIB_TRANSLATE`. The operator is dissecting the `20260712`
  flight log; expect follow-up questions on whether the low-drone occupancy poisoning is actually solved.
- **Paired SLAM logging + timestamp fix — DONE (session 11)**; `fly.py` one-command launcher — DONE.
- **REPLAN dead-stall (item 2)** — no infinite idle when the planner returns no goal. Designed:
  `plans/replan-deadstall-sweep-and-slam-tracker.md` (bbox diagonal sweep + SLAM_TRACKER → replay HTML).
- **Per-goal height calibration (item 1)** — BUILT session 9, live-fly pending
  (`plans/glass-corner-blacklist-and-height-calib.md`).
- **Glass-corner blacklist escape (Bug A+B)** — built session 7, still needs a clean live confirm.
- **Phase-2b — dense low-altitude interior mapping, then detection.** Operator idea: map the inner
  room near ground level so the target can be found there later. Recommendation (see item-2 plan
  Part 3): a low-altitude interior traverse is worth it for denser geometry, but a *blind SLAM-off*
  flight drifts (no pose feedback). For detection, prefer **(a) offline cascade on the recorded
  map-mode footage** (reuses map-mode poses; no GPU contention — start here) or **(b) a temporally
  interleaved Scan mode** (SLAM navigate → pause → detect → resume), NOT a pure SLAM-off pass.
- Deferred: Scan mode (360° cascade with SLAM/GPU temporal separation); a glass-window altitude
  descend-probe; Phase-3 report polish + GUI.

---

## Documentation (what we tried)

### Session 18 (2026-07-15) — gave autonomy the manual stick-smoothing; fixed the nonsensical height-median  [BUILT; io_bridge + autopilot + flight_replay self-tests green; live-fly pending]
Post-session-17 the drone finally thrusts, but autonomous flight was height-erratic — a hard brake + pitch-up +
altitude jump on every stop and plan-loss — while the operator's MANUAL flight is "very very controlled." He
suspected the missing piece was the smoothing he feels manually, and he was right. Diffing his `20260715_001039`
manual command CSV against how the autopilot drives showed it exactly: manual keys only toggle the
`trigger_down`/`reverse_down` (and arrow) GATES, and io_bridge's 60 Hz loop RAMPS the analog toward them
(`+0.05`/tick attack, `−0.1`/tick decay; yaw/pitch `±0.05` aim). The autopilot BYPASSED all of it — the overlay
hard-wrote the analog *after* the ramp and `_neutralize_autonomy` snapped to 0 — so every scripted thrust was a
hard step and every release a hard zero (the jolt). We made the autopilot's THROTTLE (trigger/reverse) RAMP
TARGETS the existing loop chases (reusing the manual constants), so thrust now eases in/out like a hand-flown
stick while KEEPING its magnitudes; release decays smoothly (aim + gates still snap for safety). A first live-fly
then taught us to LEAVE yaw/pitch UN-ramped: the turn is duration-not-magnitude (the sim eases the aim itself,
and the drone only rotates once the aim REACHES ±1), so a yaw ramp merely delayed reaching ±1 and shrank every
turn (30°→~5° — visible in the command log: yaw took 0.33 s to reach 1.0, leaving ~0.17 s of a ~0.5 s hold at
full deflection) — and it was double-smoothing on top of Unity anyway. So ONLY throttle is ramped; yaw/pitch pass
straight through (one tick), restoring the calibrated `turn_left/right` recipe (`turn_recipe_deg=90`, hold
1.625 s). The same flight also showed the **plan-lost pitch-up is a Unity braking response, not a pitch we send** — the
outgoing command log has ZERO non-zero pitch rows all flight, and trigger DOES decay smoothly on neutral
(0.4→0 at 0.1/tick). BUT a follow-up code read found a GAS-GATE TIMING miss that likely CAUSES that brake: the
`trigger_down`/`reverse_down` boolean (which Unity gates thrust on) was set from the COMMANDED analog, so it
dropped to False the instant a stop was commanded while the analog was still decaying → Unity hard-cut the thrust
and the smooth decay never reached it. Fixed: io_bridge now derives the gate from its own RAMPED analog
(`gate = analog > 0`), holding it True until the throttle reaches 0 (hypothesis — confirm on the next flight that
the pitch-up softens; harmless if Unity actually follows the analog). We also
re-added the `--log-commands` outgoing-packet CSV (regretted reverting it in session 17) — now permanent and
always-on via fly.py — so MANUAL vs AUTO smoothing is directly diffable (it was the tool that proved both the
yaw-delay and the zero-pitch findings above). Separately, the operator couldn't make
sense of the debugger's drone-height median (it jumped −0.008→−1.8 with no new SLAM frame, then wouldn't reach
the current height). Root cause: it appended every ~50 Hz control tick with no frame dedup — re-adding one stale
pose ~25× — and was seeded with ~0 ground samples during the pre-takeoff SETTLE. Now it ingests ONE reading per
FRESH `frame_id`, only after the first calibration reports height-OK, frozen during any calibration; the old
`MAPPING_ALT_STATES` state-gate is retired (measure in any state). New io_bridge `--self-test` (ramp) + a
rewritten autopilot ingest-gate test; all green. **Lesson: to make a scripted actuator behave like a human's,
replicate the platform's OWN input-conditioning (its ramp/gate model) rather than writing raw setpoints — and a
rolling statistic must ingest once per real SAMPLE (dedup by frame id), not once per consumer tick. And know your
actuator's model before you smooth it — the sim's YAW isn't a magnitude axis (it rotates at a fixed rate once the
aim saturates), so "smoothing" it only stole turn time; smooth THROTTLE, pass AIM through. CAVEAT: throttle knobs
still want the session-17 "lower the speed knobs" pass; turn durations are UNAFFECTED (yaw no longer ramped).**

### Session 17 (2026-07-15) — the triggerDown discovery: autonomous thrust was never engaged; simplified the height system  [BUILT; all six module self-tests green; live-fly pending]
For MONTHS the autonomous drone "crawled" (~0.02-0.04 u/s) and the height sagged, and we blamed SLAM/geometry.
While diagnosing the broken height TRIM we finally instrumented the FULL outgoing control vector (a temporary
io_bridge `--log-commands` CSV + a `t` trim macro + a `y` replay of a hand-flown trim) and diffed a MANUAL trim
against the autopilot's macro. The manual packets carried `triggerDown=True`; the macro's carried `False`. That
was it: **Unity gates REAL thrust on the `triggerDown`/`reverseDown` BOOLEAN, not the analog `trigger`/`reverse`
we'd been driving.** The autopilot never set the boolean (`AUTONOMY_FIELDS` omitted it, and io_bridge's smoothing
DECAYS the analog to 0 unless the boolean is held), so every autonomous forward/reverse ever flown ran with the
gas button UNPRESSED — the whole "crawl." The operator confirmed two things in manual: with the boolean held the
`t` macro "plays beautiful," and the drone HOLDS ALTITUDE on its own in horizontal flight — it only climbs
uncontrollably when flying forward/strafe INTO a wall (reverse doesn't). So the periodic height re-calibration +
the gradual TRIM had been fighting a SELF-INFLICTED sag that only existed because thrust was never on. We fixed
the root cause once, centrally: added `trigger_down`/`reverse_down` to io_bridge's `AUTONOMY_FIELDS` +
`_neutralize_autonomy`, and DERIVED them in `autopilot._full_vector` — the single choke point every command
flows through — from the analog value (`trigger>0 → trigger_down=True`), so all emit sites (presets, parallax
pushes, back_off, rewind/fallback reverses, homing) engage thrust with one edit. Then we DELETED the now-pointless
machinery: the periodic per-goal re-calibration trigger and ALL of TRIM (state, sag trigger, `_trim_exit`, vars,
the 3 ceiling/desired/delta references, config, self-test). We KEPT the first-takeoff calibration, the
flight-height median, and all calibration-recovery states — retained (unwired) for a FUTURE wall-hit-triggered
re-calibration, which the wall-climb behaviour now motivates. All six module self-tests green; live-fly pending,
and the speed knobs will need lowering now that the drone actually thrusts. **Lesson: when a whole platform
"just moves badly," LOG THE LITERAL BYTES LEAVING YOUR PROCESS and diff them against a known-good manual action
before building elaborate compensation — months of height machinery were treating a symptom of one unset boolean.**

### Session 16 (2026-07-14) — a SETTLE between every action (recovery + postlude) + the full return-to-origin ending  [BUILT; all module self-tests green; live-fly pending]
A test flight finished its last corner, tried to return to origin, and fell apart — it "turned like a maniac,"
fired the reverse-list back-to-back with NO settles, exhausted itself, fell back to spinning (also no settles),
declared STUCK, then retried. We recognized ONE pattern in three places: commanded actions fire back-to-back
with no still window for monocular SLAM to re-lock, so the pose stays frozen/stale and the failure compounds —
the same thing session 15 fixed for the per-leg SETTLE, just never applied to the recovery mechanisms or the
postlude. So we generalized the session-15 SETTLE tracker into a **shared gate** (`_settle_begin` /
`_settle_poll`, two flavors — HEALTHY `require_fast=True` and a bounded LOST-SLAM `require_fast=False` that gates
on fresh CAPTURE only, since SLAM is STALE by definition during recovery) and refactored the SETTLE state onto
it (behavior-identical). Then we put a settle **between every REWIND inverse maneuver and every spin FALLBACK
attempt** (bounded by `recovery_settle_max_s` so a dead pipeline still re-exposes) — resolving the session-15
parked "reverse-without-settling." The operator then caught a related ordering bug: each spin FALLBACK attempt
was `push → turn`, leaving a BARE ROTATION as the last motion before the settle — exactly the SLAM-killer, right
when we ask it to re-lock. Flipped to **`turn → push`** so the parallax translation is last (rescues the
rotation for RELOC; also matches the established "reset attitude with 'c' BEFORE a push" recipe, since a turn is
`yaw + 'c'`). The operator also chose to build the **full return-to-origin ending** in the
same session (it's where the mess showed up): `home_reach_dist` 1.0→0.5 (so it homes at altitude instead of
docking 0.86u out), a new **ORIENT_HOME** state facing the recorded `_takeoff_heading`, a **POSTLUDE_LOST_HOLD**
(mirror of CALIB_LOST_HOLD) so a SLAM loss during the dock HOLDs + resumes instead of thrashing into recovery,
**`_POSTLUDE_NOLOCK`** to stop the flying-height altitude lock from re-inflating a floor-level drone, and homing
settles (`PLAN→TURN→SETTLE→ADVANCE→SETTLE→PLAN` — the direct "maniac turning" fix). All module self-tests green
(new inter-action-settle + ORIENT_HOME-bearing-wrap + DOCK-survives-loss + no-re-inflate tests). **Lesson: the
"settle so SLAM can re-lock" discipline isn't just for the mapping loop — every place that emits a maneuver
(recovery, homing, orient) must give the monocular solver a still window, or it thrashes; a bounded settle
(fresh-capture-verified, time-capped) is the general primitive.**

### Session 15 (2026-07-14) — TRIM pitch fix + calib escape/STUCK + SETTLE fresh-frame gate + debugger numbers  [BUILT; all module self-tests green; live-fly pending]
The session-14 TRIM flight (`20260714_113312`) surfaced six things. (1) The TRIM **pitch axis was inverted** —
`+1.0` aimed DOWN so the drone descended; flipped to `-1.0`. (2) When SLAM got badly confused a re-calibration
**looped forever** (finish/interrupt → lose plan → redo → …); we bounded it with a shared `_calib_fail_escalate`
counter and a new `CALIB_ESCAPE` state — after 3 consecutive failed attempts, push once to a fresh vantage +
hold for SLAM (12 fresh frames + OK) then retry; 3 more → `STUCK` (logging paused). (3) The operator caught a
`SETTLE` that fired `ORIENT` ~1 s later with the last SLAM solve ~2 s stale → a shaky pose → plan loss; a
**settle must SETTLE**. Fixed: a goal-flying settle now waits for **6 SLAM "done" frames CAPTURED after the
settle began** (`cap_ts ≥ entry`) and under 1000 ms — the running streak was stale-high (frames had stopped
arriving), so we count frames by their capture time, not a pre-existing streak. The operator challenged an
early claim that the prelude runs before SLAM tracks — the data proved him right (SLAM is solving from the
first ARM tick, `frame_id=670`), so the vertical prelude routine is exempt by role, not by track status. (3b)
`CALIB_VERIFY`'s 5 s timeout used to PASS-and-fly to a goal on a stale/absent pose — now it counts a failed
attempt and feeds the same escape/STUCK guard. (4) The debugger's useless `Δpos/Δgoal` was replaced with a
**HEIGHT CALIBRATION** number group (last ceiling/desired/delta + the all-flight rolling median CALIB_VERIFY
judges against). (5) The launcher console flooded because two services had lost their `NEW_CONSOLE`. All module
self-tests green (new SETTLE-gate + CALIB_ESCAPE tests; `_drive` now injects a live frame stream so the gate is
exercisable). **Lesson: a "settle" that trusts a running health streak can proceed on a frozen-but-recently-
healthy track — gate on frames whose CAPTURE time is after the settle started; and always CHECK THE DATA before
asserting what the prelude does.** Parked: reverse fired without settling — diagnose on the next log.

### Session 14 (2026-07-14) — gradual height TRIM (pitch-aim climb); diagnosed the ending + glass-wall bounce  [BUILT; all module self-tests green; live-fly pending]
Flight `20260713_223231` flew great and **live-proved `CALIB_LOST_HOLD`** (it fired 3× and recovered every
time — session 13 confirmed). But the drone still gradually LOST height: calibration only re-taps on a goal
change, and ~half the flight sat in SLAM_HOLD/HOLD_LOST where the discrete `joy_vertical` altitude-lock never
corrects. We wanted a FINE, dose-able vertical primitive. The operator's idea: use the sim's PITCH aim — pitch
the aim UP and push forward, and the drone flies toward the raised aim = a GRADUAL climb (rate = push
duration), the forward part feeding SLAM parallax (a pure `joy_vertical` pulse stretches vertical features and
chokes SLAM — exactly what bit DOCK_FLOOR this flight). Built it: at each calibration's `CALIB_VERIFY` pass we
record `ceiling_y` (climb peak), `desired_y` (settled), `delta`; on a fresh healthy frame in SETTLE/ADVANCE, if
`pos_y > ceiling_y + 1.2*delta` (== sunk >20% of the ceiling gap below desired) a **`TRIM`** state runs: a ring
gate picks a safe way to climb-forward (fwd-open → climb; else reverse to open forward room; else strafe to an
open side; else abort+"pray", all visible), then pitch-up (`trim_aim_s`) → forward push with pitch still up
(`trim_fwd_s`) → `c` reset → WAIT for a healthy frame CAPTURED ≥ `trim_cmd_t0 + trim_settle_s` (the async-SLAM
guard, same monotonic clock as CALIB_VERIFY) → LOG the post-trim height. It **preserves the committed goal**
(snapshots `leg_goal`, re-aims ORIENT at it on exit — never re-picks, so a trim can't pollute goal
commitment). Four review "traps" folded in: forward-push stays interruptible by the live flow/ram guards
(A); goal snapshot+restore (B); the cap_ts↔now monotonic baseline is the project's proven CALIB_VERIFY gate
(C); the 3 values are captured only at a settled CALIB_VERIFY pass, never mid-wobble (D). All module
self-tests green (incl. a new HEIGHT-TRIM test). **Also diagnosed but NOT built** (each its own plan for a
fresh session): the **return-to-origin ending** — `home_reach_dist=1.0` made it "reach" origin 0.86u out and
dock in place from flying height; the dock then lost SLAM → recovery loop; the flying-height altitude-lock
re-inflated the floor-level drone → land/crawl/jump (`plans/return-to-origin-and-graceful-dock.md`); and the
**2-minute glass-wall bounce** — the blacklist region is smaller than the frontier spacing (whack-a-mole,
every blacklist `1 total`) AND the 2-bump counter reset when the planner alternated to distant goals
(`counter defeated` ×209) (`plans/blacklist-region-and-counter.md`). **Lesson: `joy_vertical` being a discrete
full-thrust axis is WHY we had no gentle altitude trim; the pitch-aim + forward "fly toward your aim" trick is
a gradual, SLAM-friendly vertical primitive — and any brief interrupt maneuver must snapshot + restore the
committed goal so it doesn't pollute the mission's goal commitment.**

### Session 13 (2026-07-13) — a plan loss during a ceiling re-tap erased the calibration; built CALIB_LOST_HOLD  [BUILT; all self-tests green; live-fly pending]
Flight `20260713_163055`: a per-goal `CALIBRATING_HEIGHT` fired, and mid-ASCEND (flush at the ceiling) SLAM
ground on the frozen image for 2.8 s — long enough that `plan_age` crossed `plan_timeout_s` and a **brief
PLAN-LOST** fired. The global recovery guard forced `HOLD_LOST`, and when the plan returned ~0.28 s later the
normal path funnelled `SLAM_HOLD→SETTLE→REPLAN` — the mission leg loop, **with zero memory of the
calibration**. The DESCEND never ran, so the drone **stayed glued to the ceiling (`pos_y≈-2.2`) for the whole
rest of the flight**. We wanted the calibration to SURVIVE a loss. Built a dedicated, telemetry-visible
`CALIB_LOST_HOLD` state (`plans/crystalline-swimming-floyd.md`): on any loss (LOST/NO-PLAN/STALE) while
`_calib_active`, latch `_calib_interrupted`, release controls, and hold watching the SLAM frame pulse
(`slam_ms`, the true liveness signal — even when the coarse plan status lags). **Redo** the whole calibration
once ≥6 fresh frames solve <1000 ms **AND** `status==OK`; **bump DOWN once (max)** if either the SLAM solve
stays choked (≥6 slow frames → wake SLAM) OR it solves fast but the planner still won't lock a path.
**Two traps caught in review:** (1) the redo exit MUST be gated on `status==OK`, not the frame streak alone —
`status` is level-triggered and lags a healthy SLAM, so exiting on the streak would re-enter the guard on the
next (still-lost) tick, wipe the streaks, and **1-tick-oscillate `CALIBRATING_HEIGHT↔CALIB_LOST_HOLD` forever**;
(2) emit the descend bump's first frame on the trigger tick, not a wasted neutral tick. **Lesson: a maneuver
interrupted by a transient loss must remember it was mid-maneuver — dropping into generic recovery silently
abandons the sub-mission; and any exit gated on a fast signal (SLAM frames) must ALSO wait for the slow
level-triggered signal (plan status) to catch up, or the two race into an oscillation.**

### Session 12 (2026-07-13) — diagnosed the strafe scrape-spin + the un-killable recovery loop; built the fix  [BUILT; all self-tests green; live-fly pending]
Flight `20260713_101220` flew well, then died after a parallax strafe. We wanted to know why, and found two
distinct failures. **The death:** at a far, tightly-boxed corner the planner correctly chose `strafe_left` (back
+ right were too close to push into), but the strafe fired at FULL magnitude (`joy_horizontal −1.0`) — strafe
turned out to be the ONE control axis we never throttled to 0.2 like advance/reverse — into a side the map read
as `None` (unmapped ⇒ `_pushable` treats it as open room). Because the drone was yawed relative to that wall, a
full-tilt lateral shove **scraped the wall, torqued the airframe into a spin, and swung the camera to face the
wall → monocular SLAM died.** The spin never shows in the log because the last good pose freezes while the real
drone keeps rotating — a lesson in itself. Raycast couldn't have saved us: the forward clearance ray is blind to
a sideways strafe, and the side ring was `None`. **The frantic loop after:** the drone thrashed `HOLD_LOST↔REWIND`
for 100+ s and could never give up, because a flickering SLAM status (PLAN-LOST↔PLAN-STALE, ~every 3 s) RESET the
whole recovery FSM each cycle — `_fallback_attempts=0` + a fresh, non-consuming `_invert_history()` — so `STUCK`
was mathematically unreachable and the reverse-list never emptied (exactly the operator's intuition). We built
`plans/strafe-throttle-and-recovery-loop.md` (all self-tests green): throttle the strafe (`strafe_throttle` 0.2);
a gated forward-reposition out of a scrape-danger corner before strafing; a recovery that CONSUMES its
reverse-list and PERSISTS across the flicker so it marches REWIND→FALLBACK→STUCK; a "don't trust the re-lock
until we've flown ≥1u" rule (`_recovering`/`_history_broken` flags, confirming ADVANCE) with a ghost-path guard
(a secondary drop after the drone has moved unconfirmed clears the now-spatially-stale history and jumps straight
to the ring-picked fallback sweep at a gentle 15° step); and a graceful STUCK that latches the stuck interval,
pauses the per-step log spam, and reports+closes it at the normal mission-end home/dock. Live re-fly pending.
**Lesson: a wall CONTACT that
induces a SPIN is invisible to a pose-based log (the pose freezes) — and a recovery FSM whose progress + give-up
counter can be reset by the very status flicker a real loss produces can never terminate.**

### Session 11 (2026-07-12) — the height-calib bug was JUDGING TOO EARLY; state-gated it + paired SLAM spans  [built; self-test-green; FLEW 20260712, calib not yet confirmed]
Session-10 flew, but a per-goal re-calibration on `20260709_122349` left the drone ~0.5u LOW: it re-tapped
the ceiling, did its brief descend, but async SLAM only caught up mid-move and it sank to `pos_y=-1.768` in
`PARALLAX_PUSH` (which doesn't hold altitude) — and because occupancy is built from a slab relative to the
LIVE camera Y, a low drone clipped standoffs and blacklisted valid frontiers. The old defence (reject a
ceiling tap below the running median of TAPS) was wrong twice over: too few taps to know "normal", and it
judged AT the ceiling before the drone had settled. **So we stopped judging the tap and judged the RESULT
after the routine ends.** A continuous rolling baseline of NORMAL flying altitude
(`_mapping_altitude_history`, ingested only in steady mapping states at healthy SLAM, FROZEN during any
calibration) is the reference; a new `CALIB_VERIFY` holds neutral after the descend, waits a settlement gate
on the plumbed camera-capture timestamp (`cap_ts`, None-guarded so a dropped frame can't crash), then
compares the SETTLED `pos_y` to the frozen median — significantly lower ⇒ the calibration sank the drone ⇒
climb to clean airspace (`ASCEND_ESCAPE`) BEFORE sliding 1u sideways (`CALIB_TRANSLATE`, never translate
while sunk) ⇒ retry; else "height OK" unfreezes ingest. Separately, to see WHY SLAM spikes, the autopilot now
emits PAIRED `slam_start`(orange)/`slam_finish`(green) replay records keyed on `frame_id` — in the browser,
terminals stay clean. The first live flight (`20260712_123815`) exposed a timestamp bug: each record sat at
the frame's own `t_mono` but was LABELED with the ~0.6s-later processing wall-time, so the orange START read
"from the future" during playback. Fixed to a dead-simple convention: START is positioned + labeled at the
frame CAPTURE wall-time (derived from `cap_ts` via the loop-top monotonic→wall offset); FINISH is positioned
+ labeled at the log/`now` wall-time and states the capture time inline (`"… finished working on the frame
#N from: [capture] … Latency: Nms."`) — so nothing reads ahead of its playback slot. (Follow-up Q from the
operator: the green↔orange span (~2.4s) is much bigger than the `Latency:` number (~1.8s). Correct + by
design — the span is the FULL capture→controller latency; `Latency:` is only the SLAM solve (`slam_ms`
wraps just `slam.process`); the ~0.6s difference is transport + perception post-work + the 0.5s plan timer +
the controller's loop cadence. The frame bus is conflated so there's no giant FIFO backlog.) We also added a
one-command **`fly.py`** launcher that stops the autopilot GRACEFULLY via a stop-file sentinel (a parent
can't Ctrl+C a separate-console child on Windows), so the report keeps its shutdown-emitted occupancy-map
backdrop, then auto-compiles the replay. All six module self-tests green. **Height calibration flew but is
NOT yet confirmed — the operator is still dissecting the flight. Lesson: a settling maneuver must be judged
AFTER it settles, against a general "normal" baseline — not at the peak, and not against a handful of
samples that can't define normal. And a replay record's shown time must be the time of WHERE it sits, not
when it was written.**

### Session 10 (2026-07-09) — all-corners verification tour + a post-mission floor-dock postlude  [built; self-test-green; live-fly pending]
Session-9 flew fine but reconstruction was UNEVEN: the drone flew one main diagonal, so occupancy was
dense on that line and thin at the two off-path corners (`DEBUG_IMAGES/mission_complete__mapping_so_so.png`).
We wanted every corner mapped, and a graceful ending instead of a hover at ceiling height. So (A) we
generalized the single opposite-corner "sweep" into an **all-corners TOUR** — `ground_grid.bbox_corners`
returns the inset bbox corners and `frontier_planner.select` visits them farthest-first (opposite, then the
far one of the rest, then the last), each cached statically while flying to it. Per the operator, **corner
targets ignore the frontier blacklist** — a genuinely walled-off corner is retired by the SAME event-driven
2-bump that retires unreachable frontiers (marked "visited" in `note_wall_hit`), which keeps termination
without a stale filter suppressing a corner we simply haven't reached yet. And (B) a **floor-dock postlude**:
on `done`, fly home to the take-off origin, then descend GENTLY to the floor and stand by low. The descent
MIRRORS the two-phase ceiling ascent (DOWN micro-pulses metered by the live SLAM descent gain, then a
continuous latch hold) — a continuous plunge would stretch the vertical features and choke SLAM right at the
finish. This needed a NEW flow **FLOOR** detector (the exact mirror of CEILING: descending `|dy_med|`
collapses to ~0 on floor contact); since it's unvalidated (unlike CEILING/WALL) a `dock_max_s` cap is the
fail-safe. All six module self-tests green. **Lesson (caught in review): a homing branch that computes a
fresh bearing needs its own angle-wrap — the self-test only exercised the at-origin path, so a missing
`_wrap180` hid until we added a turn+advance homing test; always drive the branch that does the math.**

### Session 9 (2026-07-09) — killed the REPLAN dead-stall with a bbox diagonal sweep; SLAM_TRACKER → replay HTML  [built; self-test-green; live-fly pending]
The clean session-8 flight still ended "doing nothing in a loop": the planner returned
`goal=None, done=False` and the controller idled in REPLAN until SLAM drifted → HOLD_LOST. Root cause —
the done-**verification** stage EXISTED but was silently bypassed: it only started when a fragile gate
passed (`farthest_free` non-None, not excluded, **> verify_min_dist**), and when that gate failed
`select()` fell through to a silent `return None, False` idle. So we replaced the whole fragile path with
the operator's idea: a deterministic **diagonal sweep** — take the known bbox, fly to the corner
OPPOSITE the one nearest the drone, inset ~1 u so it's reachable; if the traverse surfaces new frontiers
resume exploring, else declare a visible **DONE**. Built as `ground_grid.sweep_corner` (per-axis inset
with a **midpoint clamp** on axes narrower than 2·inset, so a corridor never overshoots its short axis
out of bounds), a reworked `frontier_planner.select` (`sweeping`/`sweep_target`; guarantees it never
rests on `goal=None/!done`), perception passing the corner, and an autopilot fail-visible **bounded-idle
backstop** (`no_goal_idle_s`) + one-shot EXPLORE-COMPLETE log. Separately, per the operator's ask, the
`[SLAM_TRACKER]` per-pose stream was **moved out of the terminal into the replay HTML** (teal
`ev_kind:"slam"` records, interleaved by time). Also built **item 1 — per-goal height re-calibration**
(`CALIBRATING_HEIGHT`): on a genuine goal change (past a 60 s cooldown) the drone re-taps the ceiling
(reusing the two-phase ascend→descend), re-latches `target_altitude_y`, then orients to the goal; a tap
well below the LIVE running median of taps is a low object → nudge forward + re-ascend (bounded). All
offline self-tests green (planner/ground_grid/autopilot/flight_replay). **Lesson: a "verify then done"
stage guarded by a fragile distance gate can silently choose to do NOTHING — make the terminal branch
deterministic (a goal or a flagged done), never a bare no-op.**

### Session 8 (2026-07-08) — "turns are broken" was a logging lie; made the flight log trustworthy
First flight (`20260708_135719`): the heading changed ~0° during every ORIENT turn, and travel bearing
matched reported heading on every leg, so we *concluded the body wasn't rotating*. We instrumented the
turn (log-bomb "TRYING TO TURN") and re-flew (`20260708_154431`). **The operator watched the drone
physically TURN — the conclusion was wrong.** Root cause: `heading` is the SLAM pose heading, published
~2 Hz and barely resolvable during pure rotation, so a whole ~1 s turn completes inside one perception
interval — the log repeats the same heading, then jumps ~45° one update later (heading sweeps the full
±180° over the flight). The **real bug was the LOGGING:** the timeline logged perception's async plan
(goal/heading/pos), not the controller's acted-on state — so a "goal reached (d=0.55)" printed next to a
shown goal 3.65 u away (the shown goal was perception's newer pick; the drone reached its committed
`leg_goal`), and a goal "changed" mid-advance simply because a fresh plan replaced the held snapshot.
**Fixes:** (1) the timeline now logs the committed `leg_goal` as `goal` (+ `dist_to_goal`), keeps
perception's pick as `plan_goal`, and exposes staleness (`plan_age_s`, `frame_id`); `flight_replay`
renders the committed goal and greys held-stale pose. (2) a synchronous **`[SLAM_TRACKER]`** line prints
every fresh pose the autopilot accepts (`dx/dy/dYaw [mode] - SLAM Latency`) so the ~2 Hz SLAM ticks are no
longer dark between state logs. (3) small eases: SLAM-settle 3→6, reach 0.4→1.0, clearance 0.6→1.0,
plan-lost grey goal marker. A follow-up flight (`20260708_195009`) flew cleanly with the corrected,
readable telemetry. **Lesson: a held-stale ~2 Hz pose logged every ~33 Hz loop tick makes a fast maneuver
look motionless — log what the controller ACTS ON, and always expose data age.**

Also **diagnosed but NOT fixed** (queued as item 2): a "blacklist with nothing blocking" that ends in a
dead stall — the forward-clearance stand-off (fwd_clear≈0.5 < 0.6) counts as a blacklist *bump*, two in
~2 s retire a reachable goal, and once every reachable goal is blacklisted the planner returns
`goal=None, done=False` and the drone idles in REPLAN forever (`autopilot.py:1378`).

### Session 7 (2026-07-08) — glass-corner blacklist escape (Bug A+B) + frontier clearance buffer  [built; flew in the session-8 flights, glass-corner escape not yet specifically re-confirmed]
A glass corner still trapped the drone forever: it fired standoff stops "like crazy" yet never retired
the goal. Two coupled bugs. **Bug A** — when no frontier was reachable the planner flew to `farthest_free`
as a fixed verify target that NEVER consulted the blacklist, and `farthest_free` is a plain geometric
argmax, so it re-picked the SAME dead corner; the 2-bump blacklist fired but was a no-op. Fix: made
`farthest_free` blacklist-aware (an `exclude` predicate skips dead regions), and `select()` now abandons
a verify target the moment its region gets blacklisted, re-caching a fresh corner or declaring done — and
caches that corner pulled 25 % back toward the drone for a vantage off the wall. **Bug B** — once SLAM
mapped the wall, the clearance stand-off stopped ADVANCE and went straight to SETTLE, so the drone never
reversed/displaced and the bump latch never re-armed (counter stuck at 1). Fix: a small `back_off` on the
standoff stop (gated `backoff_on_standoff`) whose reverse re-arms the latch (and seeds SLAM parallax), so
a second standoff counts and the corner reaches 2 bumps. **Also** added a general goal-stalling guard: a
committed frontier goal is pulled back along the drone→goal axis to a map-validated FREE cell with a
clearance buffer (`inset_to_clearance`), publishing a visible `goal_clearance_ok` flag (no silent
fallback). All module self-tests green; **live re-fly still pending.**

### Session 6 (2026-07-08) — blacklist/telemetry observability + self-calibrating ram guard
We couldn't tell WHY goals were being blacklisted. We added per-bump logging (PLANNER / MISSED-BUMP +
a live 2-bump counter in the replay timeline) and a per-frame raw-telemetry panel to `flight_replay`
(SLAM x/y/z, yaw, the literal command dict sent to the sim, Δpos, dist-to-goal, plan status). The logs
proved the blacklists were FALSE: the ram guard demanded the drone close ~0.05 u/s toward the goal, but
the drone crawls at ~0.02–0.04 u/s, so in OPEN space (clear ahead, healthy SLAM) it kept firing
"invisible collider" and two such false stops retired a reachable goal. **Fix — self-calibrating ram
guard:** measure the drone's OWN nominal free-flight speed live (1 s into the first ADVANCE, sampled
≤5 s or until a SLAM event), then fire only when the live windowed speed drops below 33 % of nominal.
Re-flew: no ram-guard false positives. Deferred: the glass-corner blacklist bugs + Part 3 height
calibration (see the plan file).

### Session 5 (2026-07-07) — dropped depth-map height logic; two-phase gentle ceiling ascent
Because the sim can't physically crash, we removed all depth-based height keeping and freed the GPU
for SLAM.
- **Removed the depth-map height patches** (the "low inner wall" bump-up / BUMP state) from the
  autopilot and **disabled DA-V2 depth inference entirely** in perception (it only fed the removed
  bump-up + the dashboard). SLAM now owns the GPU alone — peak VRAM ~9.7 → 6.75 GB — and the wall
  stand-off already used the SLAM raycast, not depth. The visualizer shows an explicit
  "DEPTH DISABLED" panel (no silent hang). The SLAM-pose **altitude lock** stays.
- **Two-Phase Hybrid Ascent** replaces the old continuous full-thrust climb that built momentum and
  smashed the ceiling (hurting SLAM). `joy_vertical` is a DISCRETE ±1 axis (can't throttle), so:
  - **Phase 1** — short UP micro-pulses; after each pulse read the live SLAM altitude gain and keep
    pulsing while still rising, so the drone approaches the ceiling with near-zero momentum.
  - **Phase 2** — once the gain flattens (flush at the ceiling), hold UP continuously so the existing
    flow CEILING detector latches a clean, low-velocity contact. (A single continuous hold is needed
    because the detector only latches within one uninterrupted pulse.)
- **Baseline nudge** — after the ceiling tap + descend, a short horizontal translation seeds a SLAM
  translational baseline before the first turn (pure rotation is the known SLAM-killer).
- **Deferred — Part 3** (per-goal `CALIBRATING_HEIGHT`) — now item 1 in Next/Future.
- **Tests:** autopilot / flow / frontier / ground_grid / perception self-tests PASS.

### Session 4 (2026-07-06) — event-driven 2-bump blacklist (replaced a broken time-watchdog)
Symptom: at a glass wall the drone sat ~9 min never blacklisting the unreachable beyond-glass goals.
- **Root cause:** the unreachable-goal watchdog was a *time accumulator gated on SLAM health*. In the
  glass pocket SLAM ran hot but the drone kept flying on valid poses, so the accrual clock stayed
  frozen and never fired. **Lesson: time-accumulation proxies gated on SLAM health go blind exactly
  in the heavy glass/wall pockets.**
- **Fix — event-driven 2-bump rule:** the autopilot reports each discrete advance-blocked stop as a
  "bump"; TWO bumps on the same goal region permanently blacklist it (a bump elsewhere resets the
  count). Immune to SLAM-clock health; a kinematic latch makes one continuous contact = one bump.
- Also added reverse **BACKWALL** contact detection (detection-only; logs a reverse-into-wall).

### Session 3 (2026-07-06) — flight-replay debug tool
Built `flight_replay.py`: the autopilot writes a structured per-step `*_timeline.jsonl` on `--log`,
and the tool renders a self-contained animated HTML (top-down scene + scrubber + event log + SLAM-ms
sparkline) so a flight can be debugged without reading 2000-line text logs. Self-test-verified.

### Session 2 (2026-07-06) — corrected glass model + flight fixes
A live flight showed the earlier "glass-stuck" watchdog was built on a WRONG glass model.
- **Correction:** the monocular camera looks THROUGH clear glass and tracks features on the far side,
  so **SLAM stays healthy and the clearance ray reads clear** — the drone hits the invisible collider,
  bounces, pushes again (an "invisible treadmill"). A watchdog that required SLAM to choke + the path
  blocked was exactly backwards.
- Other fixes: a no-spin startup that holds for SLAM instead of a blind 360° sweep; and a pos-space
  **ram guard** that stops a slow ram into an opaque wall before the frozen image kills SLAM.

### Earlier (2026-06-27 → 07-05) — Phase-2 explorer build & the goal saga
- **Ceiling detector v1** (SLAM-pose rate/plateau) **failed twice live** — monocular pose is only
  ~1 Hz, so the rate window never armed. **Lesson: validate detectors on REAL captured data, not
  synthetic streams.** → pivoted to `flow_contact_detector.py` (CPU optical-flow, self-calibrating):
  CEILING = vertical flow collapses while ascending; WALL = radial looming collapses while moving
  forward. Validated on real flights.
- **Turns vs SLAM:** closed-loop-on-heading thrashed (heading goes stale mid-spin); a "pulsed" yaw was
  wrong (yaw latches). Settled on **open-loop quantized turns clamped to ≤45°** (a small turn doesn't
  kill SLAM; the per-leg replan is the outer correction). **[Session 8: verified these turns DO rotate
  the body — a live re-fly showed the drone turning; the earlier "no-op" reading was the SLAM heading
  lagging in the log (~2 Hz, pure rotation), not the drone.]**
- **Ramming a wall kills monocular SLAM** (no parallax freezes the image); reversing a dead track
  can't revive it. → the **forward-clearance stand-off** (SLAM raycast) is the primary wall stop; the
  flow WALL detector is the fallback.
- **Frontier planner** (`frontier_planner.py`): utility selection + strong commitment + done-
  verification (fly to the farthest free corner, then declare done) — fixed goal thrash and false
  "mission complete".
- **Control-space SLAM-loss recovery** (pose is invalid during a loss): PLAN-LOST → hard hover-hold;
  PLAN-STALE → replay the inverse of recent maneuvers to re-expose keyframes; history empty → a
  bounded ≤45° fallback sweep → STUCK.
- **The unreachable-goal saga:** a goal behind glass / a wall is never consumed, so the planner
  re-hands it forever. The handling went through several dead ends — a position-conditioned watchdog
  (an A→B→A **ping-pong**), a round-based permanent blacklist, then a distance-stagnation timer — each
  failing because it inferred "unreachable" from a proxy that went blind in the glass pocket. Session
  4's **event-driven 2-bump** rule finally holds.

### Open issues
- **Return-to-origin ending + inter-action settles (session 16) — BUILT + all self-tests green, LIVE-FLY
  PENDING.** New states `ORIENT_HOME` / `POSTLUDE_LOST_HOLD` and the recovery/postlude settles have never flown.
  Watch live: the ending homes AT altitude → faces the take-off heading → gentle dock → up-bump (no
  descend-in-place / no jump-up / no maniac turning); a SLAM loss mid-dock → `POSTLUDE_LOST_HOLD` → resume (not
  recovery); a neutral settle between every REWIND step + spin attempt. If `recovery_settle_max_s` (2.5) is too
  short/long or `home_reach_dist` (0.5) too tight, adjust. `_takeoff_heading` is captured from the first healthy
  post-prelude `heading_deg` — confirm it reads a stable heading, not a wobble.
- **`CALIB_LOST_HOLD` (session 13) — BUILT + self-test green, LIVE-FLY PENDING.** A plan loss during a
  ceiling re-tap no longer forgets the calibration. Watch live: on the loss → `CALIB_LOST_HOLD` (not
  `HOLD_LOST`); NO `CALIBRATING_HEIGHT↔CALIB_LOST_HOLD` oscillation while `status` lags; on recovery the
  altitude drops off the ceiling (no more `pos_y≈-2.2` glue). Knobs: `calib_lost_recover_frames`,
  `calib_lost_bump_slow_frames` (both 6). The one-bump-max is deliberate (a 2nd nudge risks hitting walls).
- **`CALIB_VERIFY`/`ASCEND_ESCAPE`/`CALIB_TRANSLATE` (session 11) — FLEW `20260712`, NOT confirmed good.**
  The operator is dissecting this flight's log; whether the low-drone occupancy poisoning is actually solved
  is still an open question (expect follow-up questions here). Watch the real per-goal re-calibration: the
  drone should never settle low; a bad result should climb + slide 1u + retry, and occupancy should stay
  clean. The settlement gate leans on the plumbed `cap_ts` (None-guarded).
- **FLOOR detector is NEW + UNVALIDATED (session 10) — watch the first live dock closely.** Unlike
  CEILING/WALL (flight-validated), the floor collapse (`CMD_DOWN`, descending `|dy_med|`→0) has never fired
  on real footage. `dock_max_s` is the fail-safe (log + proceed to LOW_STANDOFF). If it never latches, the
  de-risk fallback is a fixed pulsed-descent count instead of flow detection (`plans/all-corners-...md`).
- **Corner tour termination relies on the fresh 2-bump (session 10), not `_excluded`** — corners ignore the
  frontier blacklist by design, so a genuinely walled-off corner still ends the tour only via two bumps on
  it. Confirm live that a truly unreachable corner retires (doesn't loop).
- **REPLAN dead-stall (item 2) — FIXED in code (session 9), live-fly pending.** Was: `goal=None && !done`
  idled REPLAN forever. Now the corner tour always yields a goal or a visible DONE, with a fail-visible
  bounded-idle backstop. (Turns are fine — session 8.) The older "heading decided only at REPLAN, no
  mid-leg re-aim" is a separate, milder concern.
- **Deferred (session-10 plan):** plan-lost-too-often investigation (SLAM choking?); a parallax-strafe
  alongside each turn. **Earlier deferred:** Scan mode; a glass-window altitude descend-probe; Phase-3
  report polish + GUI.

---

## What this project is
Assessment task: from the black-box **XLAB** Unity sim's single monocular drone feed, autonomously
map the room and report the 3D location of a target object (+ uncertainty). Phases: 1 Human Recon →
2 Autonomous Survey → 3 Localize & Report → GUI. Grading = internal consistency (metric scale and
compute efficiency NOT graded). Local on an RTX 3080 Laptop (16 GB).

## Architecture (processes over a ZMQ bus)
- **P1 `io_bridge.py`** — NDI capture + 60 Hz TCP control to Unity + keyboard. Publishes 512×288
  transport frames (:5601) + hi-res 720p (:5605). Applies the autopilot's control ONLY while autonomy
  is ON (toggle `m`; any manual key aborts).
- **P2 `perception_worker.py`** — MASt3R-SLAM every frame → `MapStore` voxel map + `GroundGrid` 2D
  free/unknown/occupied. Publishes TOPIC_POSE/MAP/PLAN/TARGET (:5603); lifts detections into the map.
  (DA-V2 depth removed in session 5.)
- **P3 `visualizer.py`** — read-only dashboard (input | top-down map + path + frontiers/goal + target).
- **P4 `object_worker.py`** — 3-stage cascade detector; publishes TOPIC_DETECTION (:5604).
- **P5 `autopilot.py`** — CPU-only flight controller (optical-flow CEILING/WALL detector + playbook
  recipes). Modes: `--dry-run`, `--mission`, `--explore` (Map mode).
- **GPU note:** SLAM and the detection cascade **cannot share the GPU** (compute contention → SLAM
  RELOC spiral). Phase-2 separates them in time (Map mode = SLAM only; a future Scan mode pauses SLAM
  to run the cascade).

## What's built
**Phase 1 (done, hardware-verified):** io_bridge + bus + dashboard; SLAM + voxel map; **target
detector** = 3-stage cascade (GroundingDINO+OWLv2 propose → DINOv2 verify → SIFT/LightGlue geom gate)
— solved a small-object, mural-cluttered task that **every single-shot/VLM engine failed** (Qwen2.5-VL,
OWLv2, dense DINOv2/SIFT/LightGlue); **3D lift + consensus** (`target_estimator`) → confident
multi-target estimate. Offline E2E confirmed.

**Phase 2 — Map-mode explorer (`autopilot.py --explore`), flies live:**
- `ground_grid.py` — 2D grid + frontier extraction from SLAM points.
- `perception` publishes **TOPIC_PLAN** (pose/heading/goal/bearing/done + forward clearance + ring);
  goal = frontier planner pick; `plan_valid=false` when SLAM not TRACKING.
- `ExploreController`: **ARM → TAKEOFF → ASCEND (two-phase) → DESCEND → CALIB_VERIFY → BASELINE_NUDGE →**
  leg loop **REPLAN → ORIENT (open-loop ≤45° turn) → ADVANCE (forward until the clearance stand-off / flow
  WALL / self-calibrating ram guard) → SETTLE**; a per-goal **CALIBRATING_HEIGHT** re-tap routes ASCEND →
  DESCEND → **CALIB_VERIFY** (state-gated judge vs the frozen flying-height baseline; a sunk result →
  **ASCEND_ESCAPE → CALIB_TRANSLATE →** re-tap, session 11); a plan loss DURING any re-tap diverts to
  **CALIB_LOST_HOLD** (survive the loss → redo the calibration on a 6-fast-frame + `status==OK` SLAM-pulse
  recovery, one DOWN bump if stuck; session 13), and 3 consecutive failed calibrations divert to
  **CALIB_ESCAPE** (ring push + hold for SLAM → retry; 3 more → STUCK; session 15); a gradual-height **TRIM**
  (session 14) fires from SETTLE/ADVANCE when `pos_y` sinks past `ceiling_y + 1.2*delta` — ring-gated PITCH-aim
  + forward climb (`trim_pitch_up=-1.0`), goal-preserving; a leg **SETTLE waits for 6 fresh post-settle SLAM
  frames** before flying (session 15); on `done` the **floor-dock postlude
  RETURN_TO_ORIGIN → DOCK_FLOOR (two-phase pulsed descent) → LOW_STANDOFF → DONE** (session 10);
  control-space **recovery** on SLAM loss; **STUCK** hold; event-driven 2-bump blacklist for unreachable
  goals. **Ram guard is self-calibrating**; the clearance stand-off is the primary wall stop. (The ORIENT
  open-loop turn works — session-8 re-fly.)
- `flight_playbook.json` + `RecipePlayer` — control recipes as data (the tunable durations).
- `fly.py` — one-command stack launcher (perception + autopilot + visualizer + io_bridge + Xlab in separate
  windows), a graceful stop-file shutdown so the autopilot flushes its replay map backdrop, then auto-compiles
  + opens the flight report. The autopilot honours `--stop-file <path>` (polled `_FileStopEvent` → clean exit).

---

## Reference — don't re-derive

### Drone control mechanic
Yaw is a **"fly toward your aim"** scheme: yaw moves an aim crosshair, forward thrust flies toward it;
a **SUSTAINED yaw hold then `c` (reset)** rotates the body (turn ANGLE = hold duration) — **confirmed
live in session 8** (the drone visibly turns). NB: the SLAM *heading* in the log lags the turn badly
(pose is ~2 Hz and monocular SLAM barely resolves pure rotation), so a real turn looks motionless in the
timeline until the drone translates — do NOT read that as "the drone didn't turn". io_bridge applies
autopilot values directly (no ramp); yaw latches until `c`. `joy_vertical` is a **DISCRETE −1/0/+1 axis**
(up/down = full thrust, can't be throttled); trigger & reverse ARE continuous 0–1. The only Unity
telemetry back is `time` — everything else is vision. Calibration: ~90° at yaw 1.0 for ~1.625 s.

### Environment & build
- Tree: `D:\EXTEND\C2_SIM\XLAB\` → `XLAB\` (read-only sim: Xlab.exe, Sample_Drone_Interface.py,
  OUTPUT\*.mp4) + `cartographer\` (our repo). One venv `cartographer\venv` (py 3.11.9,
  torch 2.5.1+cu121) — run everything from it.
- **lietorch is a PATCHED LOCAL build** (`third_party/lietorch`) — NEVER pip-install upstream.
- MASt3R-SLAM rebuild: `build_mast3r_slam.bat` then `build_mast3r_slam_step23.bat`.
- SLAM quirks (`slam_engine.py`): `os.chdir` into the SLAM repo before loading; recover the 4×4 pose
  via **Act3 on origin+unit axes, NOT `T_WC.matrix()`** (matrix() corrupts the pose under patched
  lietorch).

### Key technical facts
- **Sim protocol** (`Sample_Drone_Interface.py`): Python is the TCP **server** (127.0.0.1:65432);
  Unity connects in. 60 Hz `control_state` JSON (trigger/reverse, joy_horizontal strafe, joy_vertical
  altitude [−1 up/+1 down], yaw, pitch). Video = NDI 1280×720@30. Keys: 1=arm, w/s, a/d strafe,
  e/f up/down, arrows yaw/pitch, b=land, c=reset attitude, space=full-res capture, g=detect.
- **Resolution:** transport 512×288 (16:9, never squash); the cascade runs on the hi-res (:5605) stream.
- **Ray lift:** world ray = `pose[:3,:3] @ ray_cam`; center ray ≈ [0,0,1]; raycast skip 0.25 u.
- **World frame is +Y DOWN** (camera convention) — a sinking drone has an INCREASING `pos_y`.
- **Recording is ~58 fps, not 30** — durations must come from keystroke `mono_ts`, never frame counts.

### Run procedure
1. Designate target once: `venv\Scripts\python.exe make_target.py` → `target.yaml`.
2. **One command: `python fly.py`** — spawns perception `--no-display` + autopilot
   `--explore --log --stop-file` + visualizer + io_bridge (separate windows) + `Xlab.exe`; press `m` on
   io_bridge to hand over; press ENTER in the launcher to stop CLEANLY (drops the stop-file so the autopilot
   flushes its replay map backdrop) → auto-compiles + opens the report. Manual equivalent: `Xlab.exe` →
   `python io_bridge.py` → `python perception_worker.py --no-display` → `python visualizer.py` →
   `python autopilot.py --explore --log`; press `m` to hand over.
3. Offline self-tests: `autopilot.py --self-test`, `flow_contact_detector.py --self-test`,
   `frontier_planner.py --self-test`, `ground_grid.py --self-test`, `perception_worker.py --self-test`.
   Offline SLAM+map E2E: `perception_worker.py --video OUTPUT\flight_<ts>.mp4 --no-display`.
4. Diagnostics: `--log` → `OUTPUT/diag/<ts>_autopilot.{log,csv}` + `<ts>_timeline.{jsonl,html}`
   (open the HTML in a browser).

---

## Standing rules (every change)
- **NO SILENT FALLBACKS:** fail-fast OR set a visible/logged/HUD flag; any fallback approved first.
- **NO manual-flight data leakage:** every autonomous limit is a LIVE self-calibrating signal;
  platform/signal characteristics (flow signatures, control magnitudes, turn calibration, the ~1 s
  healthy-SLAM compute time) are legitimate — this room's geometry is not. **One documented,
  operator-approved exception:** branch `all-bets-are-off`, session 44 — TRIM's trigger is two
  hardcoded absolute pos_y numbers, flagged and knowingly overridden by the operator. Scoped to
  that branch; re-decide before merging to `main`.
- Image integrity (no undisclosed downscaling); start multi-step work with a TaskCreate list;
  **never commit unless asked**; self-test offline before live.

## Milestones
Phase 1: models on GPU ✅ · io_bridge + bus ✅ · SLAM + map + dashboard ✅ · target cascade + 3D
localize ✅. Phase 2: mission runner flew live ✅ · Map-mode explorer flies (SLAM-safe turns,
clearance stand-off, control-space recovery, event-driven 2-bump blacklist, two-phase ascent,
self-calibrating ram guard) ✅ · rich flight-replay debugger ✅ · glass-corner blacklist escape (Bug
A+B) + frontier clearance buffer 🛠️ built, flew in the session-8 flights. **Session 8: confirmed turns
work (the "no-op" was a stale-heading logging artifact) + made the flight log trustworthy (committed goal
+ data staleness) + `[SLAM_TRACKER]` telemetry + reach/clearance/SLAM-settle eases + a plan-lost grey
marker; a clean flight (`20260708_195009`) followed.** **Session 9: killed the REPLAN dead-stall with a
bbox diagonal sweep (`ground_grid.sweep_corner` + reworked `frontier_planner.select` + autopilot
bounded-idle backstop + visible EXPLORE-COMPLETE DONE), moved `[SLAM_TRACKER]` into the replay HTML
(teal), and built per-goal height re-calibration (item 1 — `CALIBRATING_HEIGHT` on a goal change, low-
object-tap reject) 🛠️ all built, self-test-green, live-fly pending.** **Session 10: generalized the single
sweep into an ALL-CORNERS TOUR (`ground_grid.bbox_corners` + multi-corner `frontier_planner.select`, corners
ignore the blacklist / retired by a fresh 2-bump) + a post-mission floor-dock postlude (RETURN_TO_ORIGIN →
DOCK_FLOOR two-phase pulsed descent → LOW_STANDOFF → DONE) with a new flow FLOOR detector 🛠️ all built,
self-test-green, live-fly pending.** **Session 11: BUILT + FLEW (`20260712`) the three test-flight asks —
a state-gated height-calibration fix (frozen-during-calib `_mapping_altitude_history` baseline + post-descend
`CALIB_VERIFY` settlement gate on the plumbed `cap_ts` → `ASCEND_ESCAPE`/`CALIB_TRANSLATE` retry, retiring
`CALIB_NUDGE`), paired `slam_start`/`slam_finish` SLAM logging in the replay HTML (capture-wall START / log-
wall FINISH, timestamp bug found + fixed on this flight), and a `t_wall`/`t_mono` unify; plus a one-command
`fly.py` launcher with a graceful stop-file shutdown 🛠️ all built + all six module self-tests green + flew;
**height calibration NOT yet confirmed — operator dissecting the flight log.** Plan
`plans/height-calib-state-gate-and-slam-debug.md`.** **Session 13: diagnosed `20260713_163055` — a brief
PLAN-LOST during a per-goal ceiling re-tap made the drone forget it was calibrating, skip the DESCEND, and stay
glued to the ceiling (`pos_y≈-2.2`) for the whole flight. Built a dedicated `CALIB_LOST_HOLD` state (+
`_calib_interrupted` flag) that survives the loss, redoes the calibration on a 6-fast-frame + `status==OK`
SLAM-pulse recovery, bumps DOWN once if stuck, and gates the redo exit on `status==OK` to beat the
level-triggered flicker (avoids a 1-tick `CALIBRATING_HEIGHT↔CALIB_LOST_HOLD` oscillation) 🛠️ all built + all
six module self-tests green, live-fly pending. Plan `plans/crystalline-swimming-floyd.md`.**
