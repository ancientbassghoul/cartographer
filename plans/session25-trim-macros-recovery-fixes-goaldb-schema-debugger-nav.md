_Status: BUILT (session 25). All 6 module self-tests green. See PROGRESS.md's session-25 entry for the
concise narrative; this file is the original design._

# Trim key-macros, recovery-FSM bug fixes, goals-DB schema, and debugger navigation

## Context

This is a diagnostic follow-up to the `20260718_010045` flight (the one sitting on top of sessions 20-24,
all LIVE-FLY PENDING). The operator flagged seven things after replaying it. Three are outright feature
requests (manual trim macros, debugger navigation); four were framed as "explain this to me" questions
about the recovery/blacklist machinery, but digging into the actual timeline JSONL + autopilot.log turned
up concrete, reproducible bugs behind three of them (not just confusing logging). This plan explains all
seven and implements fixes for the ones confirmed as real bugs:

- Item 1 (manual trim keys) and item 7 (debugger nav) — build as requested.
- Item 2 (strikes/blacklist "jump") — root cause: a lossy single-slot event mailbox. Fixed.
- Item 3 (step-back timeout) — answered below; it's a frame-count gate, not a timer. No code change (the
  "smarter SLAM-difficulty map" idea is the operator's own open question, not an ask — left as a backlog
  idea only).
- Item 5 (clearance never fired near the wall) — root cause: the clearance/back-off check only runs
  inside `ADVANCE`; a drone parked in `HOLD_LOST`/`SLAM_HOLD` for 30-40s during a bad SLAM patch never
  gets to run it, and the flow-based (SLAM-independent) wall/backwall contact signal is read nowhere in
  those states either. Fixed.
- Item 6 (`SLAM_STEPBACK #1/3` repeating) — root cause confirmed against the log: the escalation counter
  resets on every fresh `SLAM_HOLD` entry, and a genuinely bad SLAM patch always bounces through
  `PLAN-LOST -> HOLD_LOST -> OK` before the next hold, which wipes it every time. Fixed, plus the
  goals-DB schema split the operator asked for (2-bump / strike / loop / corner-giveup each get their own
  recorded reason + evidence).

---

## Item 1 — Manual Trim Up / Trim Down key macros

Reuses the existing `FlightPlaybook`/`RecipePlayer` machinery (`flight_playbook.py`, already used by the
autopilot for `arm`/`back_off`/etc.). Two new recipes in `flight_playbook.json`, mirroring the autonomous
TRIM state machine's `AIM -> FWD -> RESET` sub-phases (`autopilot.py`'s `TRIM` handler) but skipping the
ring-gate/reposition decision and the height-threshold trigger entirely:

- `trim_up`: pitch aim held up (`-1.0`) for 0.5s -> forward push (`trigger=1.0`, `trigger_down=true`,
  pitch still held) for 0.25s -> `btnCdown` pulse for 0.15s.
- `trim_down`: identical but pitch mirrored (`+1.0`).

`io_bridge.py`: loads the playbook; `_on_key_event` starts a macro player on `t`/`g` rising edge;
`_step_controls` drives `control_state` from the player's `fields(now)` each tick when active (ramping
`trigger` via the existing `_ramp()` helper, same easing as every other maneuver), overriding manual keys
for the macro's duration; any OTHER manual flight key cancels it immediately (added to
`MANUAL_FLIGHT_KEYS`, matching the "a real flight input always wins" philosophy already used for the
autonomy abort). Prints each phase transition to the console.

**Key conflict resolved:** `g` was `detect_key` (Qwen VL object-detect). Rebound the default to `h`,
freeing `g` for Trim Down.

---

## Item 2 — The "0 strikes to blacklisted" jump at 01:05:07 (and the recurrence at 01:08:49)

**What "strikes" are:** `frontier_planner.register_hop_outcome()` is fed once per hop by the autopilot at
the `REPLAN` that judges the hop just ended: it snapshots the distance to the goal when `ADVANCE` starts a
hop and compares it to the distance at the next `REPLAN`. Closed `>= hop_progress_eps` (0.2u) or reached
the goal -> strikes reset to 0; otherwise `+1`. At `goal_strike_limit` (2) strikes, permanent blacklist.
A far corner is exempt. Separately, a **2-bump** blacklist fires on two consecutive physical contacts on
the same goal region, and a **loop/circling** blacklist fires on `>=3` picks of one goal from a clustered
drone position. All three write into the same permanent `_blacklist` store.

Entering a plan-loss recovery state (`HOLD_LOST`/`SLAM_HOLD`) already clears `_hop_start_goal`, so an
interrupted hop is correctly never judged — the strike logic itself is sound. Tracing the exact ticks the
operator pointed at (`timeline.jsonl:8452-8457`, `autopilot.log:472-482`) shows the committed leg never
even entered `ADVANCE` during the visible loss window — the drone was cycling `SLAM_HOLD <-> HOLD_LOST` on
one stale frame for the whole ~6s stretch (`plan_timeout_s: 3.0` aging the plan out repeatedly before the
next 5041ms SLAM solve landed). The strike that finished the goal off was judged BEFORE this window even
started.

**Root cause of "zero strikes, then blacklisted, no step in between":** a lossy single-slot mailbox.
`perception_worker.py`'s `PerceptionPipeline.last_planner_event` was a bare string, overwritten every time
a bump/pick/loop event was drained and destructively read-and-cleared exactly once per published plan
(`_consume_planner_event`, riding "ONE plan"). Because `pipe.step()` (where that read-and-clear happens)
is synchronous with the SLAM solve — which took 5-8+ seconds repeatedly in this stretch — any earlier
message that landed before the next `pipe.step()` call was silently overwritten by whatever fired after
it, never published, logged, or shown in the debugger. The autopilot's own log confirms this: `plan
status: OK` is immediately followed by `STRIKE-BLACKLIST ... strikes=2` with no `strikes=1` ever printed
anywhere. This is "the flaky blacklist mechanism" — the decision logic is fine, the visibility of it was
not.

**Fix (built):** `last_planner_event` is now an accumulating list; each set-site appends instead of
overwriting; `_consume_planner_event()` joins-and-clears the whole list into one string. Nothing is
silently dropped anymore.

**Also observed (minor, not fixed separately):** the committed `leg_goal` only swaps to the new pick at
the next `REPLAN`, not the instant the DB blacklists the currently-committed goal — a display-timing lag
that explains "the drone still moves toward the blacklisted target" for one tick, not a functional bug. It
resolves on its own now that the mailbox fix stops mid-air pick pulses from queueing anonymously.

---

## Item 3 — SLAM_STEPBACK timeout (explanation only, no code change)

Not a wall-clock timeout. `_update_slam()` counts consecutive freshly-arrived SLAM frames whose `slam_ms
>= slam_slow_ms` (1000ms). Once that streak hits `slam_stepback_after_frames` (10) while already parked in
`SLAM_HOLD`, the first step-back fires. Because each slow frame itself takes >=1s to arrive, 10 of them
naturally adds up to the 19.9s/31.2s/16.0s waits logged — bounded by SLAM's own solve time, not a clock.
`slam_stepback_max_steps` (3) caps how many step-backs happen per hold before it gives up and just keeps
holding.

Whether a live SLAM-difficulty heatmap would be a better escape strategy than a blind step-back is the
operator's own open question — logged as a backlog idea, not implemented here. The concrete bug in this
mechanism (the counter never actually reaching 2/3 or 3/3) is item 6, below.

---

## Item 5 — Clearance/wall-contact "never fired" near the wall at 01:17:19-01:18:07 (FIXED)

Traced in the timeline: `fwd_clear` legitimately crossed `stop_clearance_dist` (1.0) around 01:17:22 and
kept closing to 0.25 (wedged — back/left also ~0.25) while the drone spent that whole window bouncing
`SLAM_HOLD -> PLAN-LOST -> HOLD_LOST` on repeated 6+ second SLAM solves. It only reached `ORIENT ->
ADVANCE` again at 01:18:07.059, and the clearance/back-off check fired on the very next tick — the first
opportunity it had to run.

**Root cause:** the clearance/back-off check — and the flow-based `wall_contact`/`backwall_contact` read
— only exist inside the `ADVANCE`/`PARALLAX_PUSH` handlers. The top-of-`step()` status gate that routes
into `HOLD_LOST`, and the `SLAM_HOLD` wait loop, return early with zero use of those arguments, even
though `run_explore` computes them every tick independently of SLAM health (the whole point of
`flow_contact_detector.py`). So for the entire 30-40s a drone spends blind in `HOLD_LOST`/`SLAM_HOLD`, a
live wall-contact signal firing during that window was read by nobody — and the drone is not statically
hovering during that time (real forward drift continued between the loss and the next SLAM solve landing).

**Fix (built):** `HOLD_LOST` and the `SLAM_HOLD` wait loop now get a reactive, bounded response to
`wall_contact`/`backwall_contact` via `_blind_contact_backoff()` — reuses the existing `back_off`
recipe/`BACKOFF` machinery. Edge-triggered (`_blind_contact_armed`) so a sustained pin doesn't replay it
every tick. Plays via a new `BLIND_BACKOFF` state that "owns every status" while it plays (like
`CALIB_ESCAPE`) — it typically starts WHILE status is still LOST/STALE, so without owning every status it
would be swept straight back into a fresh `HOLD_LOST` after one tick, abandoning the recipe before it ever
moved. On completion it resumes the SAME hold it interrupted (not SETTLE — the plan is still
untrustworthy).

---

## Item 6 — `SLAM_STEPBACK #1/3` repeating forever + goals-DB schema split (FIXED)

**Confirmed against the log** (01:31:09-01:32:34): step-back #1/3 fires, plays its inverse, returns to
`SLAM_HOLD`. Before it can go slow again, the plan ages out (`PLAN-LOST -> HOLD_LOST`). SLAM finally
solves (24.7s latency) with `status: OK` — and because `HOLD_LOST` is in `_RECOVERY_STATES`, the generic
recovery convergence calls `_enter_slam_hold("SETTLE", ...)` again, a FRESH `SLAM_HOLD` entry, which
unconditionally reset `_slam_stepback_count = 0`. This repeats twice more before SLAM goes slow again
inside that fourth fresh hold — and the log shows `SLAM_STEPBACK #1/3` AGAIN, not `#4` or "give up." A
genuinely bad SLAM patch always produces exactly this bounce (a 20+ second solve always exceeds
`plan_timeout_s` (3.0) before finishing), so the counter could never reach its cap in precisely the
scenario it exists to bound.

**Fix (built):** stopped resetting `_slam_stepback_count` inside `_enter_slam_hold`. It now persists
across a `PLAN-LOST`/`HOLD_LOST` bounce (mirrors the existing `_recovering`/`_fallback_attempts`
persistence rule) and resets on either: (1) reaching `REPLAN` at all (a genuinely trusted recovery point —
only happens once the two-gate settle gate has cleared), or (2) — per the operator's explicit addition —
unconditionally whenever the planner commits to a materially NEW leg goal (the same `goal_moved` check
already computed in the `REPLAN` handler), even outside a recovery. Both are handled by one unconditional
reset at `REPLAN` entry, since case 1 is reached exactly when case 2's condition is checked anyway.

**Corner-goal audit (done before touching the schema, per the operator's ask):** corners are NOT
unconditionally exempt from blacklisting — the exemption is proximity-gated. `prev_strike_eligible` is
only False when the corner leg ends farther than the (live, room-scaled) far-corner exemption distance;
close, and it's strike-eligible like any frontier. `_register_bump` only diverts into the far-corner
give-up path when still far at bump time; once near, it falls straight through to the SAME 2-bump logic
every other goal uses. So the "strict environmental conditions" the operator wanted (repeated physical
bumps, or a distinct spatial lockout) already gate a near corner; only the FAR case gets the separate,
bounded give-up-counter safety net. No logic change needed here — a corner disc can legitimately pass
through BOTH phases (far give-ups, then near bumps/strikes) in one flight, so the schema below records
both rather than assuming one mechanism per corner.

**Goals-DB schema split (built):** `_blacklist_goal(goal, permanent=False, reason=None, evidence=None)`
now stores `reason` (`"2bump"|"stall"|"loop"`) and an `evidence` dict (position, strikes/picks/spread,
SLAM `slam_ms` where available — all explicitly `float()`-cast) on the blacklist entry; threaded through
from `note_wall_hit`, `_db_blacklist` (stall/loop), passed through the autopilot's bump/pick pulses
(`take_bump_pulse()` now returns `(goal, reason, pos, is_corner)`; the pick pulse carries `judge_pos`/
`judge_slam_ms`/`prev_is_corner`). The 2-bump and corner-giveup counters are folded into the SAME per-disc
`_goal_db` entries (new `bumps`/`corner_giveups`/`is_corner` fields) instead of three separate structures,
so `goal_db_snapshot()` is the one place that answers "why is this goal dead" (including a `blacklist_
reason`/`blacklist_evidence` looked up from the matching `_blacklist` entry). `force_retire_corner`
records its give-up on the same disc but deliberately does NOT call `_blacklist_goal` (a force-retired
corner is "given up on for this tour," not declared permanently unreachable — `_excluded()`'s corner
semantics are unchanged). `flight_replay.py`'s Goals DB panel shows the new columns + a short evidence
string on dead rows.

---

## Item 7 — Debugger navigation (clickable log, prev/next, SLAM filter) (BUILT)

A global `ALL_EVENTS` array (state/planner/missed-bump events + interleaved SLAM start/finish records,
tagged with `{t, stepIdx, isSlam, cls, html}`) is built ONCE at load instead of per-render. `render()`'s
event log now reads from it (filtered to `t <= cursor`), and every rendered line carries `data-gidx` so a
delegated click handler on `#events` can jump the scrubber to that message's time. New **Prev**/**Next**
buttons walk a pointer (`navPos`) through `ALL_EVENTS`, skipping SLAM records unless the new **"incl. SLAM
msgs"** checkbox (default unchecked) is ticked; toggling the checkbox re-anchors the pointer to whichever
eligible entry is closest to the current cursor time, so Prev/Next never jumps wildly on toggle.

---

## Verification

Ran on this session: all 6 module self-tests (`io_bridge.py`, `autopilot.py`, `frontier_planner.py`,
`flight_replay.py --self-test`; `flight_playbook.py --print`; `perception_worker.py` import) green,
including new asserts for every fix above. **NEXT = LIVE-FLY** to confirm in the real flight:
- Item 1: press `t`/`g` in manual flight, watch the console print each macro phase while the drone
  visibly pitches/pushes/resets.
- Items 2/5/6: no more single-jump blacklists in the Goals DB panel (intermediate strike/bump messages
  now visible); a `back_off` firing during a `HOLD_LOST`/`SLAM_HOLD` stretch if genuinely near a wall;
  `SLAM_STEPBACK #2/3`/`#3/3` reachable on a sustained bad patch instead of re-arming at `#1` forever.
- Item 7: click log lines to jump, use Prev/Next to step message-by-message, toggle the SLAM checkbox.
