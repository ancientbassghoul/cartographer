# SESSION 61 — Sonnet-Ready Implementation Specification
# The LKG panel tells the truth: live cadence · real inlier lines · a complete info block

Run with: `python sonnet_runner.py --plan C:\Users\owner\.claude\plans\cached-rolling-hearth.md`

---

## EXECUTION GUIDELINES (read before every chunk)

1. **Implement ONLY the current chunk.** Do not start, preview, refactor or "improve" any other
   chunk. Earlier chunks are already applied on disk — do not re-verify or re-implement them.
2. **Signatures are contracts.** Use the exact names, parameter names, parameter order, keyword-only
   markers, defaults, types and return shapes given in `SHARED CONTRACTS`. No renames, no extra
   parameters, no changed return shapes. If a contract looks wrong, implement it as written and say
   so in your report.
3. **No architectural changes.** Do not restructure the FSM, do not move functions between modules,
   do not introduce new classes, threads, processes, ports or dependencies beyond what a chunk names.
4. **Anchors are source strings, not line numbers.** `autopilot.py` is ~11k lines and a bare `Read`
   truncates it — use `Grep` to locate each anchor string, then `Edit`.
5. **NO SILENT FALLBACKS** (`CLAUDE.md`). Never swallow an error into a default. A degraded or
   suppressed path must set an explicit visible state flag, emit a log line, and be counted. In this
   session the specific trap is *absence of data*: an empty F_LKG, an absent live frame or a
   suppressed match must each be NAMED on screen, never rendered as a blank or a frozen image.
6. **IMAGE INTEGRITY** (`CLAUDE.md`). Never resize, crop or re-encode a frame that feeds a model. The
   ONE display-only scale in this session is the visualizer's fit-scale in `render_lkg_panel`,
   inherited from session 60 and disclosed in **C10** — do not add others. Compositions zero-pad
   (`_pad_to_width`), never scale.
7. **NO MANUAL-FLIGHT DATA LEAKAGE** (`CLAUDE.md`). Every new constant here is a general UI cadence,
   pixel geometry or timeout. Nothing derived from a specific flight or room.
8. **Comment in the surrounding style.** Dense "why", not "what", each tagged with its session
   number. Tag new blocks `Session 61:` and cite the evidence in `MISSION CONTEXT`.
9. **Do not commit, stage, stash, or otherwise mutate git state.**
10. **The gate runs nine suites under the project venv after every chunk** — `autopilot.py`,
    `frontier_planner.py`, `visual_recovery.py`, `flight_replay.py`, `ground_grid.py`,
    `map_store.py`, `salvage_flight.py`, `perception_worker.py`, `visualizer.py`. All nine are green
    at the start of this session. Breaking any one halts the run.
11. **Finish by running the self-test commands the chunk names**, and report the full PASS/FAIL list
    verbatim.
12. **Every chunk must change at least one file.** An empty diff is treated as a failed chunk.

---

## MISSION CONTEXT (why this work exists)

Diagnosed off flight `OUTPUT/diag/20260904_223410_*` (~7 min, 2026-09-04 22:34-22:41), the first live
fly of session 60, from an operator screenshot taken at ≈22:40:29.

Session 60's F_LKG plumbing **works**: the reference advanced through **135 distinct `slam:<id>`
values** up to `slam:20741`, there were **zero** `F_LKG AGE-OUT` lines (the message no longer exists)
and no `VISUAL_RECOVERY` anywhere. What is broken is the **panel that displays it**.

**Finding A — the canvas is published only at SIFT-match instants, so the panel freezes.**
`canvas_pub.publish(...)` sits inside `if do_match:` (`autopilot.py`, anchor
`canvas_pub.publish(` / `visrec_probe._compose_debug(frame, visual_match, banner=banner, stacked=True)`).
This flight's **last match was 22:39:39.272**. At 22:40:29 the panel was therefore showing a canvas
**~50 s and 8 SLAM solves old**, from the previous loss episode at a different heading — *both* halves
frozen, F_LKG and LIVE. Only **16 `[VISREC]` lines fired in 7 minutes**, so the freeze is the normal
case, not an edge case. Everything else on screen stayed live, which is what made the panel look like
it was lying:

| what the operator saw | source | age at 22:40:29 |
|---|---|---|
| map heading arrow | `overlay_plan` ← `plan["heading_deg"]`, SLAM frame #152 = NDI **19385** | current |
| telemetry `LKG=slam:19385` | the probe's live `_lkg_src` | current |
| **the F_LKG/LIVE images** | canvas latched at the **22:39:39** match instant | **~50 s, 8 solves** |

The operator read the arrow (pointing down-and-right, agreeing with the live input panel) against an
F_LKG image aimed at the upper right of the room, and correctly concluded the panel could not be
showing the frame SLAM had last tracked on.

**Finding B — why no match ran in that window (this is a FEATURE, and must be visible).** GATE A
(`_visrec_should_match` → `ctrl.wants_visual_match`) blocks all matching on `PLAN-LOST`/`NO-PLAN`
until the episode outlives `loss_backoff_grace_s: 12.0`. HOLD_LOST entered **22:40:17.768**, so the
grace would have matured at **22:40:29.77**; SLAM returned at **22:40:30.5**. Not one match was
permitted in the whole window. So during the first 12 s of every loss the panel legitimately has NO
inlier lines to draw — and must say so in words rather than looking like a failed match.

**Finding C — no inlier correspondence lines, ever.** The publish site re-composes from the returned
`VisualMatch` alone, and `kp0/kp1/good/mask` are `match()`-local, so `_compose_debug`'s stacked branch
always takes the no-lines path. Session 60 shipped this knowingly (its own comment says
*"No correspondence lines here (kp0/kp1/good/mask are match()-local…)"*). The operator wants them
back — they were visible in the retired standalone window. The side-by-side PNG evidence
(`OUTPUT/diag/20260904_223410_visrec/*.png`, **1024×322**) does have them.

**Finding D — the info line is clipped.** Transport frames are **512×288** (the 1024×322 PNGs prove
it: two frames side by side plus `BANNER_H = 34`). `_compose_debug`'s `line2` is ~135 chars ≈ 1080 px
at font 0.4, so in a **512 px**-wide stacked canvas it dies at "…planar_like=False sc" — losing
`scale`, `size`, `closer`, `lkg_src` and `age`, i.e. every field worth reading. The old side-by-side
canvas was 1024 wide, which is why it used to fit.

**Operator decisions on record (2026-09-04):**

- **Keep the panel live at all times; do NOT hide it outside loss episodes.** Hiding saves ~0.3 ms of
  a 26-31 ms autopilot tick and ~9 MB/s of localhost IPC against a frame bus already moving ~80 MB/s
  (0.44 MB × ~60 Hz × 3 subscribers), and touches **no GPU work at all** — the choke lives in
  `perception_worker`'s CUDA path in a different process, and session 58's removal of ~230 SIFT
  matches per flight already left it unchanged. Hiding is also the *more* bug-prone option:
  "stop publishing when fine" leaves the visualizer drawing its last canvas indefinitely — precisely
  the 22:40:29 failure — so it would need a stand-down message whose missed delivery restores the bug.
- **The stale-canvas placeholder must be impossible during `PLAN-LOST`/`PLAN-STALE`.** That is exactly
  when the operator needs F_LKG, LIVE and the lines. Grey must mean "the publisher went silent",
  never "the plan is fine".
- **Do NOT describe any of this as a SLAM-choke mitigation.** Total work goes slightly UP. The one
  real choke experiment remains a flight with `visrec_debug_window: false`.

Out of scope, stays parked in `STATE.md`: the top status strip reading a stale `SLAM=TRACKING` while
perception is mid-solve (at 22:40:29 that strip was itself the frozen 22:40:14 message, 15 s old,
because frame #153 took `slam_ms=15472.2`). The operator wants to discuss that separately.

---

## SHARED CONTRACTS

Read before every chunk.

### C1 — new `config.yaml` keys

Under `autonomy.explore:`, beside the existing `visrec_*` block (anchor:
`visrec_match_min_interval_s:`). General UI cadences — no room-specific values:

```yaml
    visrec_canvas_loss_interval_s: 0.1    # session 61: LKG-panel republish period while the plan is
                                          #   LOST/STALE and matching is not yet permitted (the 12s
                                          #   loss_backoff_grace_s window) -- the pair must stay live
    visrec_canvas_idle_interval_s: 0.5    # session 61: republish period while the plan is healthy
                                          #   (~2Hz; the reference is still worth watching, cheaply)
```

Read in `ExploreController.__init__` beside `self.visrec_match_min_interval_s` as
`self.visrec_canvas_loss_interval_s` / `self.visrec_canvas_idle_interval_s` (`float(...)`, same
`ex.get("<key>", <default>)` pattern as its neighbours).

`visrec_debug_window` stays the master kill switch: `false` must still disable compose + publish +
PNG saving entirely. `use_visual_matching` stays the matcher-construction gate.

### C2 — `VisualRecoveryProbe._last_draw` (`visual_recovery.py`)

New instance field, declared in `__init__` beside `self._lkg_feats`:

```python
# Session 61: THIS call's RANSAC draw set, so a caller composing its own canvas right after
# match() can draw the real inlier correspondences (they are match()-local; session 60's panel
# had none). Tuple of (kp0, kp1, good, mask) or None. Cleared at the TOP of every match() so it
# can never describe a previous call.
self._last_draw: "tuple | None" = None
```

Lifecycle, enforced exactly:

- `match()` sets `self._last_draw = None` as its **first statement** (before the `self._lkg is None`
  early return).
- In the two branches that own the RANSAC result — the `if not out.matched:` branch (anchor
  `out.debug_image = self._compose_debug(frame, out, kp0=kp0, kp1=kp1, good=good, mask=mask,`, the
  first occurrence) and the final matched path (the second occurrence) — set
  `self._last_draw = (kp0, kp1, good, mask)` **unconditionally, outside the `if debug:` guard**, so
  the draw set exists even when `visrec_debug_window` is off and the canvas is composed anyway.
- No other site writes it.

### C3 — `_draw_stacked_inliers` (`visual_recovery.py`)

Extract the manual line loop currently inside `_compose_debug`'s `if stacked:` branch into a
module-level private function (not a method — it takes no probe state):

```python
def _draw_stacked_inliers(body, h_top, kp0, kp1, good, mask) -> int:
    """Draw RANSAC-inlier correspondences on a STACKED (top=F_LKG / bottom=live) canvas, in place.

    `cv2.drawMatches` only ever builds a side-by-side canvas, so for a stacked one each inlier is
    drawn by hand: a line from (x_lkg, y_lkg) to (x_live, y_live + h_top).

    Args:
        body (np.ndarray): the stacked BGR canvas, modified IN PLACE.
        h_top (int): pixel height of the TOP (F_LKG) half — the y-offset applied to live points.
        kp0, kp1 (list[cv2.KeyPoint]): reference / live keypoints.
        good (list[cv2.DMatch]): ratio-test matches (queryIdx -> kp0, trainIdx -> kp1).
        mask (np.ndarray): boolean RANSAC inlier mask, one entry per `good`.

    Returns:
        int: how many lines were actually drawn (0 is a legitimate answer and MUST be reported by
        the caller rather than being indistinguishable from "not attempted").
    """
```

Colour `(0, 255, 0)`, thickness 1, `cv2.LINE_AA` — unchanged from session 60's loop.

### C4 — `VisualRecoveryProbe.compose_stacked_live` (`visual_recovery.py`)

The production composer for the panel. **Composes NO text** — the visualizer draws the info block at
panel resolution (C10), which is what makes it complete and crisp.

```python
def compose_stacked_live(self, live_frame, *, draw_lines: bool = False):
    """F_LKG (top) over `live_frame` (bottom), zero-padded to a common width, NO text baked in.

    Session 61: the panel canvas is now composed on a CADENCE rather than only at match instants
    (Finding A: the panel sat ~50s / 8 solves stale), so this must work with or without a match
    having just run.

    Args:
        live_frame (np.ndarray | None): the tick's live BGR frame.
        draw_lines (bool): draw THIS tick's `_last_draw` inlier correspondences. Only ever True on
            a tick whose `match()` just returned (see autopilot's publish site).

    Returns:
        tuple[np.ndarray, int] | None:
          • (canvas, n_lines) where canvas is (h_lkg + h_live, max(w_lkg, w_live), 3) when an F_LKG
            is held, or (h_live, w_live, 3) when there is NO F_LKG yet (live half only — the caller
            NAMES that state via banner_fields' `src=none`, never a blank panel);
          • None when `live_frame` is None (nothing honest to draw).
        `n_lines` is 0 whenever `draw_lines` is False or `_last_draw` is None.
    """
```

IMAGE INTEGRITY: uses `self._pad_to_width` only — no resize, no crop.

### C5 — `VisualRecoveryProbe.banner_fields` (`visual_recovery.py`)

The single source of truth for the info block, returned as **short segments** so the visualizer can
wrap between them and never clip mid-field (Finding D).

```python
def banner_fields(self, out, *, state_status, live_frame_id=None, n_lines=0,
                  lines_reason=None) -> list[str]:
    """The LKG panel's info block, as ORDERED short segments (never a single joined string).

    Returns EXACTLY 12 segments, always in this order and always present (placeholders, never
    omission — a missing field must read as "n/a", not vanish):

        [0]  "<state_status>"              e.g. "HOLD_LOST / PLAN-LOST"
        [1]  "src=<self._lkg_src>"         e.g. "src=slam:19385"  |  "src=none"
        [2]  "lkg_age=<age>"               "12.3s" from self._lkg_t, else "n/a"
        [3]  "live=#<live_frame_id>"       else "live=#n/a"
        [4]  "matched=<T|F>"
        [5]  "inliers=<int>"
        [6]  "cont=<T|F>"                  out.contained
        [7]  "planar=<T|F>"                out.planar_like
        [8]  "scale=<x.xx|n/a>"
        [9]  "size=<x.xx|n/a>"             out.size_ratio
        [10] "closer=<LIVE|LKG|EQUAL|UNKNOWN>"
        [11] "lines=<n>"  when n_lines > 0
             "lines=none (<lines_reason>)"  when n_lines == 0 and lines_reason is not None
             "lines=none"                   when n_lines == 0 and lines_reason is None

    Formatting rules (fixed, so the wrap arithmetic is predictable): bools render "T"/"F"; floats
    render f"{v:.2f}"; ages render f"{v:.1f}s"; None renders "n/a". `out` may be None (no match has
    ever run) -> segments [4]-[10] all render their "n/a"/"F"/0 placeholders.
    """
```

`_compose_debug` builds its own baked two-line banner from this same function (joined with two
spaces) so the PNG evidence and the panel can never disagree about a field's value.

### C6 — `_compose_debug` loses its `stacked` parameter (`visual_recovery.py`)

`compose_stacked_live` (C4) supersedes it, and two stacked composers with different text behaviour is
a bug farm. Restore `_compose_debug` to the side-by-side-only form (anchor: `def _compose_debug`):

- Delete the keyword-only `stacked: bool = False` parameter, the `if stacked:` composition branch and
  the `if stacked:` label-placement branch. Its manual line loop moves to `_draw_stacked_inliers`
  (C3), called by C4 — do not duplicate it.
- The PNG evidence path (`out.debug_image`, 1024×322, two baked banner lines, `cv2.drawMatches`) is
  **UNCHANGED** in shape and content, except that its `line2` text now comes from `banner_fields`.
- Retire the two session-60 `SESSION-60 LKG PANEL` self-test cases that exercise `stacked=True` /
  `stacked=False`, each with a comment naming session 61 and the reason. Do NOT weaken them into
  vacuous passes; keep every side-by-side assertion that still describes live behaviour.

### C7 — `_visrec_canvas_due` (`autopilot.py`, module-level pure function)

Placed beside the other pure decision helpers (anchor: `def _visrec_should_match`), so it is
unit-testable without entering `run_explore`.

```python
def _visrec_canvas_due(now, last_pub_t, idle_interval_s, loss_interval_s,
                       loss_now, fresh_match, matching_active) -> bool:
    """Session 61: should the LKG panel canvas be published on THIS tick?

    Pure; no I/O, no state. Rules, in order:
      1. `fresh_match`      -> True always. A lined canvas is never skipped, whatever the timers say.
      2. `matching_active`  -> False. A LINED canvas is due within visrec_match_min_interval_s, so
                               unlined intermediates are suppressed rather than flickering the lines
                               onto 1 published frame in 5. (`matching_active` is GATE A's own
                               predicate, so the two cadences can never disagree about whether a
                               match tick is coming.)
      3. otherwise          -> (now - last_pub_t) >= (loss_interval_s if loss_now
                                                      else idle_interval_s)

    INVARIANT (asserted in the self-test): no combination of inputs can leave a publish gap of
    LKG_CANVAS_STALE_S or more while `loss_now` — the panel must never grey out mid-loss
    (operator's requirement). Rule 2's worst case is visrec_match_min_interval_s (0.5s), rule 3's
    is loss_interval_s (0.1s); the visualizer's timeout is 2.0s.
    """
```

### C8 — `_visrec_no_match_reason` (`autopilot.py`, module-level pure function)

```python
def _visrec_no_match_reason(ctrl, *, needs_match, has_frame, matching_active,
                            now, status, memo_age_s) -> str | None:
    """Session 61: WHY no fresh match backs this tick's canvas — so "no lines" is a named state
    rather than a silent blank (Finding B: GATE A forbids matching for the first
    loss_backoff_grace_s of every loss episode, which is exactly the window the operator most needs
    to read).

    Returns None when a fresh match DID run this tick (the caller passes n_lines instead), else one
    of exactly these shapes:
        "plan OK"                              not needs_match
        "no live frame"                        needs_match and not has_frame
        "loss grace 7.2/12.0s"                 not matching_active, status in ("PLAN-LOST",
                                                 "NO-PLAN") and ctrl._loss_episode_t0 is not None
                                                 -> f"loss grace {now - ctrl._loss_episode_t0:.1f}/"
                                                    f"{ctrl.loss_backoff_grace_s:.1f}s"
        "loss grace pending"                   not matching_active, same statuses, but
                                                 _loss_episode_t0 is None (no episode stamped yet)
        "snapshot spent"                       not matching_active, any other status (GATE A's
                                                 one-shot ticket is used up for this episode)
        "verdict reused (0.3s)"                matching_active but GATE B reused the memo
                                                 -> f"verdict reused ({memo_age_s:.1f}s)"

    Evaluated in that order; the first matching clause wins.
    """
```

### C9 — telemetry `visrec_lkg` gains an age (`autopilot.py`, `visualizer.py`)

Payload shape (anchor: `visrec_lkg = {`):

```python
visrec_lkg = {
    "src": visrec_probe._lkg_src if visrec_probe is not None else "none",
    "age_s": (round(now - visrec_probe._lkg_t, 1)
              if visrec_probe is not None and visrec_probe._lkg_t is not None else None),
}
```

`visualizer.py`'s telemetry row (anchor: `lkg_txt = "LKG=--" if visrec_lkg is None else`) renders
`LKG=<src> age=<age_s>s`, and `LKG=<src> age=n/a` when `age_s` is None. Keep the existing position,
font and colour.

### C10 — the visualizer's LKG column (`visualizer.py`)

New module constants beside `PANEL_W, PANEL_H` (anchor: `PANEL_W, PANEL_H = 416, 234`):

```python
LKG_CANVAS_STALE_S = 2.0    # session 61: no canvas for this long -> grey it out. >= 4x the slowest
                            #   publish cadence (visrec_match_min_interval_s 0.5s), so this can only
                            #   fire when the autopilot really stopped publishing -- NEVER because
                            #   the plan is healthy (operator's requirement).
LKG_TEXT_SCALE = 0.42       # drawn at PANEL resolution, so this is the size actually seen
LKG_TEXT_LINE_H = 15
LKG_TEXT_PAD = 8
LKG_TEXT_MAX_LINES = 6
```

```python
def _wrap_text_segments(segments, max_px, font, scale) -> list[str]:
    """Greedily pack whole `segments` (list[str]) into lines whose rendered width, per
    cv2.getTextSize(line, font, scale, 1)[0][0], is <= max_px. Segments joined by two spaces.

    Session 61: segments are NEVER split mid-token, so no field can render half-visible (Finding D
    lost scale/size/closer/src/age off the right edge of a 512px canvas). NO SILENT FALLBACK: a lone
    segment wider than max_px gets its own hard-split line (like the existing char-based
    `_wrap_text`) — never a truncation. Returns [] for a falsy `segments`.
    """
```

```python
def render_lkg_panel(canvas, info=None, age_s=None, w=PANEL_W, h=MAP_SIZE):
    """... (keep session 60's docstring, including its IMAGE INTEGRITY disclosure) ...

    Session 61: `info` is the F_LKG/LIVE canvas's field segments (C5), drawn HERE at panel
    resolution instead of baked into the 512px canvas and then downscaled — crisp and complete.
    `age_s` is how long ago the canvas arrived; past LKG_CANVAS_STALE_S the panel greys out rather
    than showing a frozen image that reads as current (the 22:40:29 failure).
    """
```

Render order, exactly:

1. `age_s is not None and age_s > LKG_CANVAS_STALE_S` → `_placeholder(w, h, f"LKG canvas stale ({age_s:.1f}s)")`.
2. `canvas is None` → today's placeholder, text unchanged.
3. Otherwise: `lines = _wrap_text_segments(info, w - 12, cv2.FONT_HERSHEY_SIMPLEX, LKG_TEXT_SCALE)`
   truncated to `LKG_TEXT_MAX_LINES`; `text_h = len(lines) * LKG_TEXT_LINE_H + LKG_TEXT_PAD` (0 when
   `lines` is empty); fit-scale the canvas into `(w, h - text_h)` preserving aspect, letterboxed on
   the existing `30`-grey ground; draw the text lines at `x=6`, first baseline
   `y = LKG_TEXT_LINE_H`, colour `(220, 220, 220)`.

Geometry check to reproduce: `PANEL_W=416`, `PANEL_H=234`, `GAP=12`, `MAP_SIZE=480`. A 512×576 canvas
with a 4-line block (`text_h = 68`) fits `416×412` at `min(0.8125, 0.715) = 0.715` → `366×412`.
Composed dashboard width stays `PANEL_W + GAP + PANEL_W + GAP + MAP_SIZE` and height
`STATUS_H + MAP_SIZE` — `_open_video_writer` must NOT change.

`Dashboard` gains, beside `self.lkg_canvas = None`:

```python
self.lkg_canvas_info = None   # list[str] | None: the canvas's field segments (C5)
self.lkg_canvas_t = None      # float | None: time.monotonic() when the last canvas ARRIVED
```

`Dashboard.render()` (anchor: `lkg_p = render_lkg_panel(self.lkg_canvas)`) passes both plus
`age_s = None if self.lkg_canvas_t is None else time.monotonic() - self.lkg_canvas_t`.

`run()`'s canvas drain (anchor: `dash.lkg_canvas = cv_frame[0]`) also stores
`dash.lkg_canvas_info = (cv_frame[1] or {}).get("info") or []` and
`dash.lkg_canvas_t = time.monotonic()`.

### C11 — canvas message schema (`autopilot.py` → `visualizer.py`)

`frame_bus.FramePublisher.publish(frame: np.ndarray, meta: dict)` already carries a JSON meta dict
and `FrameSubscriber.recv` already returns `(frame, meta)` — no transport change. Meta shape:

```python
{"info": list[str]}        # exactly banner_fields()'s 12 segments, in order
```

### C12 — self-test conventions

`autopilot.py` / `visualizer.py` / `visual_recovery.py`: the file's existing accumulator style
(`ok = ok and <case>_ok`, or the local `case(...)` helper where one exists) with one
`print(f"[self-test] {'PASS' if X else 'FAIL'}  ...")` per case. Name new blocks `SESSION-61 <TOPIC>`.

---

## CHUNK 1 — the probe can compose the live pair, with real inlier lines

**Module Objective.** Give `VisualRecoveryProbe` everything a caller needs to build the panel canvas
on any tick: the retained draw set, a stacked composer that can draw it, and one authoritative field
list. Purely additive to the probe's API — no caller changes yet, so flight behaviour must not move.

**Required Context/Dependencies.** None. Contracts **C2**, **C3**, **C4**, **C5**, **C6**.

**Target Files.** `visual_recovery.py`.

**Strict Interfaces.** Exactly per C2-C6. Anchors: `self._lkg_feats = None` (field declaration);
`def match(self, frame, debug: bool = False` (the `_last_draw = None` first statement);
`out.debug_image = self._compose_debug(frame, out, kp0=kp0, kp1=kp1, good=good, mask=mask,`
(**occurs TWICE** — the `not out.matched` branch and the final matched path; both get the
`self._last_draw = (kp0, kp1, good, mask)` assignment, placed OUTSIDE the `if debug:` guard);
`def _compose_debug`; `if stacked:`; `# ---- SESSION-60 LKG PANEL` (the self-test block to retire
two cases from).

Order the work so nothing is duplicated: extract `_draw_stacked_inliers` from the existing
`if stacked:` loop FIRST, then write `compose_stacked_live` on top of it, then delete the `stacked`
parameter.

**Acceptance Tests.** New block `SESSION-61 STACKED LIVE CANVAS` in `visual_recovery.py`'s self-test,
using the existing `_textured_image` fixture helper:

1. `compose_stacked_live(live)` with an F_LKG held returns a canvas of shape
   `(h_lkg + h_live, max(w_lkg, w_live), 3)` and `n_lines == 0`.
2. `compose_stacked_live(None)` returns `None`.
3. A fresh probe with NO F_LKG returns the live-half-only shape `(h_live, w_live, 3)`, `n_lines == 0`
   — not `None`, not a blank.
4. `_last_draw` is `None` on a fresh probe, `None` after a `match()` that returns before RANSAC
   (flat-black frame → no descriptors), and a 4-tuple after a `match()` that produced a homography.
5. After a matched `match()`, `compose_stacked_live(live, draw_lines=True)` returns `n_lines > 0` and
   a canvas that differs pixel-wise from the same call with `draw_lines=False` (the lines are really
   drawn).
6. `draw_lines=True` on a probe whose `_last_draw` is `None` returns `n_lines == 0` and does not
   raise.
7. `banner_fields` returns exactly 12 segments, in C5's order, for (a) a matched `VisualMatch`,
   (b) `out=None`; `lines=` reads `"lines=7"` for `n_lines=7`, `"lines=none (plan OK)"` for
   `n_lines=0, lines_reason="plan OK"`, and `"lines=none"` for `n_lines=0, lines_reason=None`.
8. Regression: the side-by-side `debug_image` from a matched `match(debug=True)` still has shape
   `(max(h)+BANNER_H, w_lkg + w_live, 3)`.

**Verify.** `venv\Scripts\python.exe visual_recovery.py --self-test`.

---

## CHUNK 2 — the two pure cadence/reason decisions

**Module Objective.** The publish cadence and the "why no lines" text as pure, unit-testable
functions, before anything is rewired. No behaviour change on disk yet.

**Required Context/Dependencies.** Chunk 1 (none of its code is called here). Contracts **C1**,
**C7**, **C8**.

**Target Files.** `autopilot.py`, `config.yaml`.

**Strict Interfaces.** Add the two `config.yaml` keys per C1 and their `ExploreController.__init__`
reads (anchor: `self.visrec_match_min_interval_s = `). Add `_visrec_canvas_due` and
`_visrec_no_match_reason` per C7/C8, placed immediately after `_visrec_should_match` (anchor:
`def _visrec_debug_sink`, insert above it). Both are pure — no logging, no state, no `ctrl` mutation.
`_visrec_no_match_reason` reads only `ctrl._loss_episode_t0` and `ctrl.loss_backoff_grace_s`.

**Acceptance Tests.** New block `SESSION-61 CANVAS CADENCE`:

1. `fresh_match=True` → due, even with `last_pub_t == now` and `matching_active=True`.
2. `matching_active=True, fresh_match=False` → NOT due, even 10 s past both intervals.
3. `loss_now=True`, 0.2 s since the last publish, `matching_active=False` → due (loss interval).
4. `loss_now=False`, 0.2 s since the last publish → NOT due; at 0.6 s → due (idle interval).
5. INVARIANT: for every combination with `loss_now=True`, the longest gap the rules permit
   (`max(loss_interval_s, ctrl.visrec_match_min_interval_s)`) is `< visualizer.LKG_CANVAS_STALE_S`.
   Assert it by importing `visualizer` and comparing the real constants, so the two files cannot
   drift into greying the panel mid-loss.
6. `_visrec_no_match_reason` returns each of C8's six shapes for the input that selects it, in
   priority order — including the exact `"loss grace 7.2/12.0s"` formatting with
   `_loss_episode_t0` set 7.2 s back and `loss_backoff_grace_s = 12.0`, and `"loss grace pending"`
   when `_loss_episode_t0 is None`.
7. A controller built from the repo's `config.yaml` has `visrec_canvas_loss_interval_s == 0.1` and
   `visrec_canvas_idle_interval_s == 0.5`.

**Verify.** `venv\Scripts\python.exe autopilot.py --self-test`.

---

## CHUNK 3 — publish the panel on a cadence, not on a match

**Module Objective.** Replace the match-instant publish with the cadence publish, so the panel always
shows the CURRENT F_LKG over the CURRENT live frame (Finding A), carrying its fields as meta.

**Required Context/Dependencies.** Chunks 1-2: `compose_stacked_live`, `banner_fields`,
`_visrec_canvas_due`, `_visrec_no_match_reason`, the two config reads. Contracts **C9**, **C11**.

**Target Files.** `autopilot.py`.

**Strict Interfaces.**

- New loop-scoped state beside `visrec_memo, visrec_memo_t` (anchor: `visrec_memo = None`):
  `visrec_canvas_pub_t = 0.0` and `visrec_canvas_n = 0` (a flight gauge, logged once at shutdown
  beside the existing `visrec_matches` gauge if one is reported there).
- **DELETE** the old publish and its comment (anchors: `canvas_pub.publish(` and the comment line
  `# Session 60 (C9): a SEPARATE STACKED (F_LKG-over-LIVE) composition for the`), which live inside
  `if ctrl.visrec_debug_window and visual_match.debug_image is not None:`. The PNG saving in that same
  block (`_visrec_debug_sink`, `visrec_saved`, `visrec_episode_saved`, `visrec_last_saved_rel`) is
  **UNCHANGED**.
- Add ONE new publish site inside `if visrec_probe is not None:`, AFTER the whole `if do_match:` block
  and immediately BEFORE `visrec_prev_status = status` (so `do_match` and `visual_match` are in
  scope):
  - `matching_active = ctrl.wants_visual_match(now=now, status=status)` — the same pure predicate
    GATE A uses; computing it twice per tick is free and keeps the two cadences in agreement.
  - `fresh_match = bool(do_match)`.
  - Publish only when `ctrl.visrec_debug_window` and
    `_visrec_canvas_due(now, visrec_canvas_pub_t, ctrl.visrec_canvas_idle_interval_s,
    ctrl.visrec_canvas_loss_interval_s, loss_now, fresh_match, matching_active)`.
  - `composed = visrec_probe.compose_stacked_live(frame, draw_lines=fresh_match)`; when it returns
    `None` (no live frame) publish nothing and leave `visrec_canvas_pub_t` untouched — the
    visualizer's 2 s timeout is what surfaces a sustained frame starvation (C10), and it must be
    allowed to.
  - `reason = None if fresh_match else _visrec_no_match_reason(ctrl, needs_match=needs_match,
    has_frame=(frame is not None), matching_active=matching_active, now=now, status=status,
    memo_age_s=(now - visrec_memo_t))`.
  - `info = visrec_probe.banner_fields(visual_match, state_status=f"{ctrl.state} / {status}",
    live_frame_id=(meta or {}).get("frame_id"), n_lines=n_lines, lines_reason=reason)`.
  - `canvas_pub.publish(composed_canvas, {"info": info})`; then
    `visrec_canvas_pub_t, visrec_canvas_n = now, visrec_canvas_n + 1`.
- Telemetry `age_s` exactly per **C9**, and update the session-60 payload-shape self-test that
  currently asserts `src`-only.
- ONE startup line, printed beside the existing `[autopilot][explore] MAP MODE. PUB TOPIC_CONTROL`
  banner, when `ctrl.visrec_debug_window` is True but `visrec_probe is None`
  (`use_visual_matching: false`): state that the LKG panel will stay idle for the whole flight and
  why. NO SILENT FALLBACK — an unexplained grey panel is exactly the ambiguity this session removes.

**Acceptance Tests.** New block `SESSION-61 CANVAS PUBLISH`. `run_explore` is never entered by the
suite, so assert the pieces it composes from:

1. `visrec_lkg` round-trips `{"src", "age_s"}` through `_full_vector`, and `age_s` may be `None`.
2. `visualizer.render_telemetry_panel` renders with `visrec_lkg` of `{"src": "slam:42", "age_s": 1.4}`,
   of `{"src": "none", "age_s": None}`, and of `None`, without raising.
3. An end-to-end composition rehearsal with no processes: build a probe, `update_reference(frameA,
   True, src="slam:7")`, `match(frameB, debug=False)`, then
   `compose_stacked_live(frameB, draw_lines=True)` + `banner_fields(...)` — assert the canvas shape,
   `n_lines > 0`, 12 segments, and that segment [1] is `"src=slam:7"`. This is the exact call
   sequence the new publish site performs.
4. Grep-style guard: `"stacked=True"` no longer appears in `autopilot.py` (the session-60 call site is
   gone). Assert by reading the module's own source with `inspect.getsource` on the enclosing
   function, or by an explicit `open(__file__)` scan — state which in a comment.

**Verify.** `venv\Scripts\python.exe autopilot.py --self-test`,
`venv\Scripts\python.exe visual_recovery.py --self-test`.

---

## CHUNK 4 — the readable info block and the stale-canvas guard

**Module Objective.** Draw the fields crisply at panel resolution, wrapped so nothing can clip
(Finding D), and grey the panel out when — and only when — the publisher has actually gone silent.

**Required Context/Dependencies.** Chunk 3 publishes `{"info": [...]}`. Contracts **C10**, **C11**,
plus **C9**'s telemetry row.

**Target Files.** `visualizer.py`.

**Strict Interfaces.** Exactly per C10. Anchors: `PANEL_W, PANEL_H = 416, 234` (new constants);
`def _wrap_text(text, max_chars):` (insert `_wrap_text_segments` beside it — do NOT modify
`_wrap_text` or any of its callers); `def render_lkg_panel(canvas, w=PANEL_W, h=MAP_SIZE):`;
`self.lkg_canvas = None`; `lkg_p = render_lkg_panel(self.lkg_canvas)`;
`dash.lkg_canvas = cv_frame[0]`; `lkg_txt = "LKG=--" if visrec_lkg is None else`.

Do NOT touch `_open_video_writer`, `left = np.vstack([frame_p, col_gap, tel_p])`, or the composed
width/height arithmetic.

**Acceptance Tests.** New block `SESSION-61 LKG PANEL TEXT`:

1. `_wrap_text_segments`: every returned line measures `<= max_px` under `cv2.getTextSize`; every
   input segment appears somewhere in the joined output (nothing dropped); `[]` for `None` and for
   `[]`; a single 400-char segment is hard-split into lines each `<= max_px`.
2. A realistic 12-segment `banner_fields` list wraps to `<= LKG_TEXT_MAX_LINES` lines at
   `max_px = PANEL_W - 12`, `scale = LKG_TEXT_SCALE`.
3. `render_lkg_panel(canvas, info=<12 segments>, age_s=0.1)` returns exactly
   `(MAP_SIZE, PANEL_W, 3)` and its image region is non-empty (the fit-scaled canvas is at least
   1 px tall — the text block cannot squeeze it to nothing), for both a 512×576 canvas and a
   20-segment worst case.
4. `age_s = LKG_CANVAS_STALE_S + 0.1` with a canvas present renders the grey stale placeholder, and
   differs pixel-wise from the same call at `age_s = 0.1`.
5. `render_lkg_panel(None)` still returns the waiting placeholder at `(MAP_SIZE, PANEL_W, 3)`.
6. `Dashboard.render()` composes to width `PANEL_W + GAP + PANEL_W + GAP + MAP_SIZE` and height
   `STATUS_H + MAP_SIZE` in all three states: no canvas, fresh canvas + info, stale canvas.
7. Telemetry row per C9: renders `age=` for a float, `age=n/a` for None, and for a missing payload.

**Verify.** `venv\Scripts\python.exe visualizer.py --self-test`,
`venv\Scripts\python.exe autopilot.py --self-test`.

---

## CHUNK 5 — documentation, resume state, and archive this spec

**Module Objective.** Leave the tree self-describing (CLAUDE.md's two closing steps) and keep this
spec in the repo.

**Required Context/Dependencies.** Chunks 1-4.

**Target Files.** `PROGRESS.md`, `STATE.md`, `plans/session61-spec.md` (new).

**Strict Interfaces.**

1. Copy `C:\Users\owner\.claude\plans\cached-rolling-hearth.md` verbatim to `plans/session61-spec.md`.
2. `PROGRESS.md` — TWO concise narrative entries in the house voice ("We wanted X. We tried Y. It
   failed because Z. So we tried W."), no implementation detail:
   - **Session 60 was flown** (2026-09-04 22:34-22:41, `OUTPUT/diag/20260904_223410_*`). The F_LKG
     rework works — 135 distinct `slam:<id>` references, zero age-outs, no `VISUAL_RECOVERY`, no
     double bump pulses — but FALLBACK was never entered, so the new `SERVO` phase is still
     unobserved. The choke is undiminished: a 15.5 s solve at 22:40:30.
   - **Session 61** — the panel, not the plumbing, was lying: published only at match instants, it sat
     ~50 s and 8 solves stale while the map arrow and telemetry stayed live; it never drew the inlier
     lines (`kp/mask` are `match()`-local); and its info line was clipped at 74 of ~135 chars in a
     512 px canvas. Fixed by publishing on a cadence, composing the lines from the retained draw set,
     and moving the text into the visualizer. Record the operator's rejected alternative (hide the
     panels when the plan is healthy) and WHY: no GPU work, ~0.3 ms of a 26-31 ms tick, and its
     failure mode is the very bug being fixed. Reference `plans/session61-spec.md`.
3. `STATE.md` — keep ~150-200 lines. Replace the session-60 immediate-next block with session 61's
   watch list: the LIVE half moves continuously and the F_LKG half changes whenever telemetry's
   `src=slam:<id>` changes (no 50 s freeze); `LKG=<src> age=<n>s` with the age resetting each solve;
   green lines steady at ~2 Hz once a loss matures past the 12 s grace, and during the grace the pair
   live with `lines=none (loss grace N/12.0s)`; the info block complete — `closer`, `scale`, `size`,
   `src`, `age` all legible, nothing off the edge; **the panel never grey during PLAN-LOST/PLAN-STALE**
   (if it is, the publisher stalled); `visrec_debug_window: false` still kills compose + publish +
   PNG outright; `use_visual_matching: false` leaves an EXPLAINED idle panel.
4. `STATE.md` — move session 60's now-flown watch items into `PROGRESS.md`'s one-liner style and
   carry forward, unchanged in priority: the **SLAM choke** as the dominant open problem (keep the
   per-5-minute table, and add the 22:40 evidence: an 11.1 s solve at 22:40:06 and a 15.5 s solve at
   22:40:30 inside a 7-minute flight), the still-unobserved **FALLBACK `SERVO` phase**,
   **bump-pulse latency**, the **staleness UI** (now with the concrete case: at 22:40:29 the top strip
   showed `SLAM=TRACKING kf=27 slam=2921.5ms` from 15 s earlier while perception was mid-solve — the
   operator wants to discuss it before it is built), the **goal-management rewrite** decision rule,
   and the **adaptive back-off** and **dead `_backoff_resolve_since` gate** future ideas.

**Acceptance Tests.** None (documentation only). The nine-suite gate must still pass.

**Verify.** `venv\Scripts\python.exe autopilot.py --self-test`. Then confirm by inspection that
`STATE.md` alone is enough to resume cold.
