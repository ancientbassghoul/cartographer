# Session 58 — LKG window discipline + dead-goal bump guard

## Origin

Diagnosed off flight `OUTPUT/diag/20260903_223345_autopilot.log` (7 min, 25 loss episodes) — the
**first live fly of session 57**.

Session 57's headline fix **worked and is confirmed**: all 25 loss episodes ran LKG matching. The
diagnosing flight's "56 of 86 episodes ran zero matches" went to **0 of 25**. Watch-list item 1 from
`STATE.md` is settled.

But flying it surfaced four new problems. Three are fixed here; the fourth is written up below as the
session-59 starting point.

---

## Finding 1 — the 12 s grace is announced but not honoured for the camera

Every loss episode in the flight reads like this:

```
22:39:40.399  plan status: PLAN-LOST
22:39:40.399  *** LOSS-RECOVERY GRACE: holding still for 12s before any reaction
              (96.9% of held-still losses resolve inside that window). Looking at the camera after that. ***
22:39:40.490  [VISREC] has_lkg=True matched=True inliers=75 ... closer=EQUAL      <- 91 ms later
```

The notice promises "looking after 12 s"; SIFT fires 91 ms in, and then every 0.5 s for the whole
episode.

**Cause.** `ExploreController.wants_visual_match` (`autopilot.py:2006`):

```python
if (not self._loss_snapshot_checked) or self._visrec_phase == "MATCH":
    return True                                     # <- always True on the PLAN-LOST path
if (status in ("PLAN-LOST", "NO-PLAN") and now is not None and self._loss_episode_t0 is not None
        and (now - self._loss_episode_t0) >= self.loss_backoff_grace_s):
    return True                                     # <- session 57's clause: UNREACHABLE
return False
```

`_loss_snapshot_checked` is the pre-session-57 one-shot ticket. It is re-armed to `False` at every
loss edge (`:3140`), and session 57 removed the only code that ever *spent* it on this path —
`_maybe_loss_snapshot_backoff` (`:2860`) now serves `PLAN-STALE` only. So on PLAN-LOST the ticket is
permanently `False`, the first clause always short-circuits to `True`, and the grace clause session
57 wrote specifically to gate this path never executes.

**Measured cost on the flight.** 253 `[VISREC]` lines across 25 episodes:

| | count |
|---|---|
| matches inside the 12 s grace (no consumer could read them) | **233 (92 %)** |
| matches after a grace matured | 20 |
| decisions produced by those 20 (`LOST_VISUAL_HOLD` + loss-instant back-offs) | **0** |
| episodes that outlived the grace at all | 4 of 25 |

Episode durations: median 4.1 s, p90 13.1 s, max 18.7 s. So the overwhelming majority of episodes
resolve on their own well inside the grace — exactly what session 48 measured — and every SIFT +
BFMatcher + RANSAC pass run during them was thrown away. On the CPU. While SLAM was fighting to
relocalize at 8-10 s per solve.

That last point matters beyond waste: `visual_recovery.py`'s module docstring says the matcher is
CPU-only *precisely* so it does not contend with the relocalization it is waiting on. Running it ~230
extra times per flight is a live suspect for the still-open SLAM-choke question in `STATE.md`, whose
untested-leads list already names the session-49 LKG window.

---

## Finding 2 — the LKG window opens at arm and never closes

The operator's theory was a NO-PLAN status on the very first frame. The log rules that out: the
first `[VISREC]` of the flight is at `22:35:42`, **79 s after takeoff**, and `match()` returns at
`visual_recovery.py:232` before composing any canvas while `_lkg is None`. No match ran at startup.

**Cause** is session 49's **idle refresh** (`autopilot.py:5483`):

```python
elif (ctrl.visrec_debug_window and visrec_probe._lkg is not None
      and (now - visrec_last_idle_show) >= 0.5):
```

There is no loss gate on it at all. It fires as soon as the first reference is cached — which is the
first `status == "OK"` plan with `plan_valid`, at `22:34:21.384`, two seconds before takeoff — and
then every 0.5 s for the rest of the flight.

---

## Finding 3 — the window flickers because the idle canvas is `F_LKG | F_LKG`

`_compose_debug(self, frame, out, ...)` takes the **live** frame as its first positional argument.
The idle branch hands it the reference:

```python
idle_canvas = visrec_probe._compose_debug(
    visrec_probe._lkg, VisualMatch(has_lkg=True), banner=why)
```

With no keypoints it falls to `np.hstack([pad(lkg), pad(frame)])` — so the idle canvas draws F_LKG
twice and labels the right half "LIVE".

During a loss two independent 0.5 s timers then race into the same window:

| tick | branch | right-hand pane |
|---|---|---|
| t | `do_match` (memo aged past `visrec_match_min_interval_s` = 0.5) | live frame **+ drawn RANSAC inliers** |
| t + ~0.5 | idle refresh (its own 0.5 s gate) | **a second copy of F_LKG**, no lines |

That alternation at ~2 Hz is the flicker. It is worse than cosmetic: half the frames the operator is
using to judge "is F_LKG closer than the live view?" — the exact question session 57's `closer`
verdict exists to answer — are showing F_LKG on both sides.

---

## Finding 4 — bump-pulse latency (DEFERRED TO SESSION 59)

The bottom-right goal `[3.9, -3.6]` took **4 min 10 s** and three glass rams to retire. The operator
watched the drone ram, back off, blacklist, then apparently re-select the same spot and ram again.
Stage by stage:

**Phase A — two minutes to turn around (22:36:13 → 22:38:08).** Seven legs walking the bearing error
`+178.7 → +146.1 → +112.2 → +84.9 → +62.3 → +32.6 → +1.6`. `turn_step_deg: 30` caps each leg's aim
change and each leg costs ~20 s in the settle gate. Working as designed, just expensive.

**Phase B — the approach and three rams (22:38:08 → 22:39:27).**

| time | event | distance to goal |
|---|---|---|
| 22:38:09 | hop | 8.30 |
| 22:38:32 | hop | 6.69 → 6.15 |
| 22:38:55 | hop | 3.41 → **3.37** (closed 4 cm in 2 s — blocked) |
| 22:39:20 | hop | 3.15 → 2.79 |
| 22:39:24 | hop | 2.79 → SLAM dies |
| 22:39:27.510 | **PLAN-LOST** | |

None of the three rams registered a bump. There are **zero** flow-WALL fires and **zero**
`MISSED-BUMP` lines in the entire flight — glass gives `flow_contact_detector` nothing to collapse
(no texture, no radial expansion), so the map's forward clearance was the only signal that ever
worked here.

**Phase C — the bumps, and a 17.8 s blind spot (22:39:48 → 22:40:24).**

```
22:39:48.665  BUMP pulse #0  clearance 0.53 <= 1.25  -> BACKOFF
22:39:59.013  PLANNER: BUMP count=1/2                           (10.3 s later)
22:40:06.387  BUMP pulse #1  clearance 0.62 <= 1.25  -> BACKOFF
22:40:24.170  PLANNER: BUMP count=2/2 -> BLACKLIST PERMANENT    (17.8 s later)
```

**This is why it took so long.** The bump pulse is fire-and-forget over ZMQ; its *effect* only reaches
the autopilot when perception publishes its next plan. `perception_worker.run()` is
`recv frame -> drain pulses (:753-760) -> pipe.step()`, and `pipe.step()` was blocking on SLAM solves
of 8.2 / 8.3 / 8.0 / 9.7 s back to back. Pulse #1 missed frame #202's drain by 0.44 s, waited out that
8.0 s solve, was drained at the top of #203, then waited out another 9.7 s solve before the plan
carrying `BLACKLIST` was published: 0.44 + 8.0 + 9.7 ≈ 17.8 s. Every second of that, the autopilot
still believed the goal was live and kept committing legs to it.

Interleaved through the same window were four PLAN-LOST episodes (5.6 / 5.8 / 5.3 / 7.2 s), each
burning a grace it never outlived and each producing ~11 unusable SIFT matches — Finding 1's waste,
landing on the CPU while SLAM was already at 8 s per frame.

**Phase D — "it got selected again" (22:40:24 → 22:40:28).** Real, and *this half is fixed in this
session*:

```
22:40:24.170  BLACKLIST PERMANENT -> reselecting
22:40:25.165  BUMP pulse #2 goal=[3.9, -3.6]      <- the same goal, 1.0 s after it died
22:40:25.165  BACKOFF: clearance 0.97 <= 1.25
22:40:26.133  PLANNER: BUMP goal=[3.9,-3.6] count=1/2 (armed)   <- re-arms on a dead region
22:40:28.862  ORIENT toward [-1.0, 7.2]           <- finally a new goal
```

The planner never re-selected it. The autopilot's own `leg_goal` was still `[3.9, -3.6]`, and the
SLAM_HOLD post-recovery settle clearance check (`autopilot.py:3391-3394`) fired `_register_bump`
against it. `_register_bump` (`:2609`) checks `leg_goal is None` and the far-corner guard but **never
the live blacklist**, even though the arrays ride the plan and session 28 already built exactly that
re-check for the TRIM resume path (`:1528-1532`). In the replay this renders as an `active` marker
(drawn from `leg_goal` by `_timeline_goals`, `:136-161`) sitting on top of its own
`blacklist_permanent` ring for four seconds, with one more back-off against it.

**Phase E — the guard that should have caught this and didn't.** The STALL guard
(`goal_strike_limit: 2`) was structurally silent: the whole flight produced **5** `HOP_JUDGE` lines,
all `progressed=True`, zero strikes. A judge only happens at a REPLAN with an intact baseline, and the
losses kept destroying the pairing — including the 22:38:55 hop that closed 4 cm, the clearest stall
signal of the flight, which was never judged at all. So the 2-bump rule was the *only* live retirement
mechanism, running at one bump per ~14 s of SLAM latency.

### Session-59 design sketch (NOT built here)

Three candidates, in preference order. All three need the operator's call before any code.

1. **Local provisional 2-bump count in the autopilot.** The autopilot already knows when it publishes
   a pulse. Give it a local counter applying the planner's exact rule (same goal within
   `goal_assoc_dist`, reset on a different goal); at count 2 it drops `leg_goal` and forces
   SETTLE→REPLAN immediately, and refuses to re-commit within `goal_area_radius` of that point until
   the plan's blacklist confirms it. The planner stays the sole owner of the blacklist store — this
   is a provisional "stop flying at it", not a second blacklist. Divergence risk is low because both
   sides consume the identical pulse sequence in the identical order, and the count only increments
   where a pulse is actually published (so the far-corner guard and the disarmed latch suppress both
   sides alike). Biggest behavioural win; biggest surface area.
2. **Immediate `TOPIC_PLANNER_EVENT`.** perception publishes a blacklist notification the instant
   `note_wall_hit` returns `action == "blacklist"`, instead of letting it ride the next plan. Removes
   one full solve (~10 s of the 17.8 s) with no duplicated logic anywhere. Still one solve behind,
   because the drain itself is blocked by `pipe.step()`.
3. **Make the STALL guard survive a loss.** A hop whose baseline pairing is destroyed by a PLAN-LOST
   currently produces no `HOP_JUDGE` at all, so a 4 cm hop against glass is invisible to
   `register_hop_outcome`. If the baseline could be carried across a loss episode, the stall guard
   would have retired this goal long before the 2-bump rule did.

Also unresolved and worth stating plainly: **the flow contact detector cannot see glass.** Zero WALL
fires across a flight with three confirmed glass rams. Whatever session 59 does about latency, the
map clearance remains the only working glass signal, so nothing should be built that assumes a WALL
fire will arrive.

---

## Design (what session 58 actually changes)

### 1. Honour the grace before looking (`autopilot.py`)

`wants_visual_match` is restructured so the PLAN-LOST/NO-PLAN grace clause is the **only** authority
on that path and the never-spent ticket cannot bypass it. Every other path keeps today's answer:
`_visrec_phase == "MATCH"` still short-circuits True; `PLAN-STALE` and every no-arg caller still read
`not self._loss_snapshot_checked`.

`now`/`status` are additionally threaded into the memo-reuse call site, so a match cached from a
*previous* episode cannot be replayed into `visual_match` during a grace either. `_step_lost_recovery`
already returns before reading it, so the only observable effect is that `visual_match` stays `None`
through the wait — which is what the code says it does.

All six session-57 assertions still pass unchanged: they all build the controller with
`one_shot_spent=True`, which is exactly why this shipped. The new assertions use the real flight
condition — the ticket **armed**.

### 2. LKG window becomes loss-only and stops flickering (`autopilot.py`)

The idle-refresh branch is **deleted**. With it gone the window is driven by one branch only, so the
flicker is structurally impossible and the takeoff pop-up disappears with it. `_compose_debug`'s
no-keypoint path is not orphaned — `match()`'s four failure returns still exercise it.

The window is then scoped to a loss episode: a new `visrec_window_open` flag beside the existing
degradation flags, set by `_visrec_debug_sink` after a successful `imshow`, and a sibling
`_visrec_close_window` called on the recovery edge. Finding 1's fix is what makes this rate tolerable
— only 4 of 25 episodes on this flight outlived the grace, so the window would open ~4 times in 7
minutes, and its appearance now *means* "this loss has matured and we are looking at the camera".

**One trap, called out because it would fail silently.** The PNG save predicate is
`decision = loss_edge or ctrl._visrec_phase == "MATCH"`. After change 1 a `loss_edge` tick never runs
a match, so the only canvases written would be probe re-matches and PLAN-LOST evidence would vanish
from `OUTPUT/diag/<ts>_visrec/` with no error. It is redefined as *the first match of each episode*
or a probe re-match.

### 3. A blacklisted goal can no longer be bumped (`autopilot.py`)

The dead-goal predicate already exists at `_trim_resolve_resume:1528-1532`. It is lifted verbatim into
a reusable `_goal_is_blacklisted(plan, goal)` and called from both places — no new logic, no new
radius (it keeps `goal_area_radius`).

`_register_bump` gains the guard, which covers all eight of its call sites uniformly. It refuses the
pulse and stashes a `_missed_bump` marker naming the blacklist; the marker is already drained and
logged at `:5546-5550`, so the suppression is visible rather than silent. It deliberately performs
**no FSM mutation** — it exists to stash a pulse, and eight callers each have their own control flow.

The dead commitment is dropped at the one path that produced the observed event: the SLAM_HOLD
settle-resume branch (`:3391`) checks the blacklist *before* the clearance stand-off test and, when
the committed goal is dead, converges via SETTLE→REPLAN — the same route `_trim_resolve_resume` already
uses for a goal that died mid-TRIM. Confirmed against the real trace: the plan in hand at
`22:40:25.165` (frame #203) already carried `blacklist=[[3.9,-3.6]], blacklist_permanent=[True]`.

## Files touched

- `autopilot.py` — all three changes plus their self-test blocks.
- `PROGRESS.md`, `STATE.md` — session log and resume pointer.
- No config keys added. No changes to `visual_recovery.py`, `frontier_planner.py` or
  `perception_worker.py`.

## Implementation spec

`plans/session58-spec.md` — four chunks, run with
`python sonnet_runner.py --plan plans/session58-spec.md`.
