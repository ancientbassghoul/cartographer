# Session 12 — Strafe throttle + kill the frantic SLAM-loss recovery loop

## Context

Test flight `20260713_101220` flew well, then "lost its shit" at **t≈28017.86s (step #12292)** — right after
a parallax **strafe**. Diagnosis from the timeline:

**The loss trigger.** Parked at a far corner `pos=[4.3, 6.3]`, chasing a goal 11.6u away with a ~−76° bearing
error, the drone was stuck in a mini-loop of `turn −30° → strafe_left → turn → strafe`. Ring at the incident:
`{fwd: None(open, 4.4 via raycast), back: 0.3, left: None, right: 0.65}`. Backward wasn't pushable
(`0.3 < 0.7`), right wasn't (`0.65 < 0.7`), so it repeatedly picked **strafe_left** — correct, because `left`
read `None` and `_pushable(None)=True` (unmapped ⇒ treated as open room). It strafed at **full magnitude
(`joy_horizontal: −1.0`)** into an unmapped left. Because the drone was almost certainly yawed relative to that
wall, the strafe drove its flank into the wall — a **scrape that torqued it into a spin**, swinging the camera
to face the wall → monocular SLAM died. The spin is invisible in the log because the pose froze at the last
good value while the real airframe kept rotating.

**Why it then looped forever.** After the loss, SLAM status thrashes `PLAN-LOST / PLAN-STALE / OK`. The recovery
FSM **resets on every flicker**: `PLAN-LOST` (`autopilot.py:1287`) clears `self._player` and enters `HOLD_LOST`,
destroying any in-progress REWIND/FALLBACK; then `PLAN-STALE` → `_step_stale` sees state `HOLD_LOST`, takes the
*fresh-entry* path (`_fallback_attempts = 0` + a fresh, **non-consuming** `_invert_history()`). So the rewind
restarts from full and the give-up counter zeroes every ~3 s. **`STUCK` (terminal give-up) is mathematically
unreachable** → `HOLD_LOST ↔ REWIND` for 100+ s, never dying.

**Why raycasting never saved us.** Two structural blind spots: (1) the forward clearance raycast casts a fan
**along the heading only** — blind to a lateral strafe (`fwd_clear` read 4.4 correctly the whole time);
(2) the side ring guard existed but `left` read `None` (unmapped) → permitted a full-commit strafe into unknown
space.

Intended outcome: (a) gentler strafe; (b) a reposition that removes the scrape geometry; (c) a recovery loop that
consumes its reverse-list, makes monotonic progress, and terminates instead of thrashing; (d) a reverse-list we
only trust once we've genuinely flown again; (e) logging that doesn't balloon when the drone is parked stuck.

---

## Decisions (all locked with the operator)

### D1 — Strafe throttle → 0.2 (like forward/reverse)
Strafe is currently full magnitude (`flight_playbook.json:79` `joy_horizontal: 1.0`, read into `self._strafe_mag`
at `autopilot.py:833`). Add a config `strafe_throttle` (default **0.2**) mirroring `forward_throttle` /
`reverse_throttle`, applied to `self._strafe_mag`. **Caveat (flag, live-test):** the sim documents
`joy_horizontal` as "(-1 to 1)" but documents `joy_vertical` identically, and `joy_vertical` is empirically a
discrete full-thrust axis — so 0.2 might not proportionally slow the strafe. Verify live; if discrete, revisit
(e.g. shorten the hold instead).

### D2 — Forward-reposition before a "scraping-danger" strafe
When a parallax push resolves to a **strafe** AND the **back ring is very close** (scraping danger) AND the
**forward raycast is clearly open**, first do a **~2.0 s forward push at 0.2 throttle** to translate out of the
tight corner, then strafe immediately (forward momentum coasts into a safe forward-left diagonal — good for
SLAM). Rationale: the death was a scrape-spin from a yawed, boxed pose, not raw speed alone; throttle-0.2 lowers
scrape energy but doesn't prevent the scrape.
- Gate on the forward raycast being clearly open (never reposition into a wall). Else skip → throttled strafe.
- "Back very close" = new **general** tunable `strafe_backwall_danger_dist` (~0.4 SLAM-units, below the ~0.5 push
  distance) — NOT the literal 0.3 observed (leakage-safe).
- Forward duration `~2.0 s` is a tunable knob (operator-derived from a ~1 s manual push at higher/unknown throttle,
  scaled for our gentler 0.2 + the airframe's slow acceleration-from-rest). Trim live.
- Altitude-sink from forward pitch is **not** a concern here (operator: negligible at these short durations, and
  height-calib is nearly solved).

### D3 — Recovery sweep uses the real ring-picked parallax push + smaller turn step
Replace `_begin_fallback`'s blind alternating fwd/back retreat with the **same ring-picked parallax-push
mechanism** used in normal scouting (backward-first if pushable, else strafe toward the roomier mapped side).
Use a **15° turn step in recovery** (safer than 30° for a SLAM-fragile re-lock; the unidirectional sweep just
needs more attempts). Add `recovery_turn_step_deg` (default 15.0).

### D4 — Kill the frantic loop; make give-up reachable; stop logging when parked stuck
Fix the FSM so recovery **persists across the PLAN-LOST/PLAN-STALE flicker** and progresses monotonically
`REWIND → FALLBACK → STUCK`, so `STUCK` is actually reachable (mechanics in D5). On reaching **terminal STUCK**:
- **Latch a "we were stuck" memory** — record each stuck interval's `[start, end]` wall-time.
- **Pause the repetitive log spam** while parked in STUCK (emit one clear `STUCK (recovery exhausted) → logging
  paused` record, then go quiet) so the operator can walk away and not return to a 200 GB log.
- **If a valid plan returns:** resume the mission **and resume logging** (STUCK already routes → REPLAN on a valid
  goal). Recovery is not yet "trusted" — see D5 (`_recovering` stays set until a confirming ADVANCE).
- On normal **mission completion**, the existing session-10 floor-dock postlude homes to origin
  (`RETURN_TO_ORIGIN → DOCK_FLOOR → LOW_STANDOFF → DONE`). At DONE, log a clear **mission-end summary including
  the stuck time-range(s)**, then **turn logging off**.

### D5 — Reverse-list lifecycle (the core redesign)
A `_recovering` flag governs both the reverse-list and the give-up counter, and **persists across HOLD_LOST /
PLAN-LOST flickers** (this is what breaks the frantic loop).

**On first PLAN-STALE (fresh recovery, `_recovering` False → True):**
- Set `_recovering = True`. This **freezes appends** to `command_history` (guard `_log_turn` / `_log_move` /
  `_log_move_push` to no-op while recovering).
- Enter a **consuming, pop-based REWIND**: pop the newest maneuver (`_pop_stepback`, already reverse-chrono),
  play its inverse, then re-check status at the `step()` top; repeat. **Drain the list to empty**, then →
  FALLBACK sweep (D3), then → STUCK once the give-up counter hits `fallback_max_attempts`.
  (Replaces the single-shot non-consuming `_invert_history()` RecipePlayer for the recovery path.)

**Persistence across flickers (the fix):**
- `PLAN-LOST → HOLD_LOST` still hovers neutral (blind ⇒ don't move), but must **NOT** reset `_recovering`, the
  give-up counter, `_history_broken`, or the (partially consumed) list. Re-entry to `_step_stale` while
  `_recovering` is already True **continues popping where it left off** — never a fresh `_invert_history()` +
  counter reset.
- Remove the `_fallback_attempts = 0` reset on OK-in-recovery (`autopilot.py:1299`) and on STUCK→recover
  (`:1744`). All resets are owned solely by the confirming ADVANCE below.

**OK returns while `_recovering` (re-lock, but NOT yet trusted):**
- Proceed SLAM_HOLD → SETTLE → REPLAN → ORIENT → (parallax) → ADVANCE to re-aim at the new goal, but the ORIENT
  and parallax moves are **not logged** (freeze still on) and the **counter does not reset** — a freshly
  re-locked pose isn't trustworthy yet.
- **The moment the drone enters ANY unconfirmed spatial state (ORIENT / PARALLAX_PUSH / ADVANCE) post-relock, set
  `_history_broken = True`** — the drone has physically moved (turned + translated) with those moves frozen out of
  the log, so the leftover pre-loss chain is now spatially decoupled from the drone's true pose. Replaying it
  would fly a displaced **"ghost path"** (inverted vectors from the wrong location) — likely straight into a wall.
- **Secondary drop before the confirming ADVANCE — branch on `_history_broken`:**
  - `_history_broken == False` (drop is still within the INITIAL rewind, before any re-aim motion — e.g. a
    `LOST↔STALE` flicker mid-drain): **continue consuming** the leftover pre-loss entries (chain still coherent).
  - `_history_broken == True` (drop after an unconfirmed re-aim moved the drone): **instantly clear the leftover
    history and BYPASS REWIND — drop directly into the D3 ring-picked FALLBACK sweep** (the safe, live-clearance
    parallax pushes). The counter keeps climbing → eventually STUCK. This is what terminates the loop without ever
    flying a ghost path.

**Confirming ADVANCE (the only place recovery is declared successful):**
- Track ADVANCE progress from leg-start pos. When a post-recovery ADVANCE **travels ≥ `recovery_confirm_dist`
  (1.0 SLAM unit)**: set `_recovering = False`, `_history_broken = False`, **reset the give-up counter**,
  **clear the old reverse-list**, and resume normal logging into a fresh list. (Progress-gated, not merely "the
  ADVANCE state ran" — a re-locked drone that instantly bumps a wall at 0 distance must NOT count as confirmed.)
- An ADVANCE that exits with < 1 u progress (wall/ram/leg_max) does not confirm; `_recovering` persists, the next
  leg retries, counter unchanged. (If SLAM stays OK but the drone simply never advances 1 u, `_recovering`
  lingers harmlessly — exploration continues, just unlogged; a later loss finds an empty list → straight to
  FALLBACK. Acceptable.)

New config (`autonomy.explore`): `recovery_confirm_dist: 1.0`, `recovery_turn_step_deg: 15.0`,
`strafe_throttle: 0.2`, `strafe_backwall_danger_dist: 0.4`, `strafe_reposition_fwd_s: 2.0`.

---

## Files in scope
- `flight_playbook.json` — `strafe` recipe magnitude (D1).
- `config.yaml` (`autonomy.explore`) — the five new knobs above.
- `autopilot.py`:
  - `_strafe_mag` wiring to `strafe_throttle` (D1).
  - PARALLAX_PUSH: reposition-before-strafe when back-close + forward-open (D2).
  - `_begin_fallback`: ring-picked push + 15° step (D3).
  - Recovery FSM: `_recovering` + `_history_broken` flags; consuming pop-based REWIND; persistence across
    HOLD_LOST; ghost-path guard (post-relock spatial move ⇒ secondary drop clears history + bypasses REWIND to
    FALLBACK); remove the counter resets at `:1299` / `:1744`; freeze `_log_*` while recovering; confirming-ADVANCE
    progress gate (≥1 u) that clears the list + resets the counter + drops both flags (D5).
  - STUCK: stuck-interval memory + log-spam pause; mission-end summary + logging-off at DONE (D4).

## Verification
- Offline self-tests green: `autopilot.py --self-test` (+ `flow_contact_detector`, `frontier_planner`,
  `ground_grid`, `perception_worker`).
- **New self-test coverage** (extend the recovery tests around `autopilot.py:3005-3055`):
  - A `PLAN-LOST ↔ PLAN-STALE` flicker DURING the initial rewind no longer resets recovery — `_recovering`, the
    popped-list position, and the counter persist; the drain continues; STUCK is reached after the cap.
  - REWIND is consuming: `command_history` drains to empty, then FALLBACK.
  - Ghost-path guard: after an OK-triggered ORIENT/ADVANCE sets `_history_broken`, a secondary PLAN-STALE
    **clears the leftover history and routes straight to FALLBACK** (no REWIND / no ghost-path pop).
  - `_log_*` are frozen while `_recovering`; ORIENT/parallax during post-recovery re-aim add nothing.
  - Confirming ADVANCE only on ≥1 u progress: a <1 u wall-bump ADVANCE does NOT clear `_recovering`/counter; a
    ≥1 u ADVANCE clears the list + resets the counter + drops the flag.
  - D2: reposition fires only when back-close AND forward-open; skipped otherwise.
- **Live re-fly** of the same far-corner scenario: confirm no frantic loop, a gentler strafe, a graceful STUCK
  with paused logging, and a bounded log file. Watch the D1 caveat (does 0.2 actually slow the strafe?) and the
  D2 reposition displacement (does 2.0 s at 0.2 move enough?).

## Final action item (do this before finishing / any context clear)
Update `PROGRESS.md` per its house style: fold session 12 into the concise "we tried X" narrative (the strafe
scrape-spin diagnosis + the reverse-list/recovery redesign), move resolved items out of "Next", and leave a
crisp resume pointer. Keep `plans/*.md` as the detailed design reference. (Operator idea to consider: a CLAUDE.md
rule that **every plan ends with a context-clear-prep / PROGRESS.md-update step** — flag it, don't add it without
the operator's explicit go-ahead.)
