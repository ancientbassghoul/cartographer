# Session 66 — frame-exact replay, and the syncs that made the window look like a loss

## 1. The discovery that reframed everything: offline replay was never reproducing live

Every attempt to bench the bounded window on a recorded flight undershot badly. On
`flight_20260628_092640.mp4` at the default `--stride 3`: **3 keyframes in 200 frames**. At stride 9:
15. At stride 18: 33. Never near the 40-49 keyframe knee the live data shows.

It was not the footage. Measured on live flight `20260906_144839`:

| | value |
|---|---|
| `frame_id` gap between consecutive PROCESSED frames | median **112**, p90 **624**, max 1735 |
| wall gap | median 1.90 s, p90 10.32 s |
| NDI capture rate | ~58 fps (`io_bridge.py:964`) |
| keyframes | 52 in 395 frames = 7.6 frames/kf |

SLAM runs at ~0.2 Hz, so `io_bridge` hands it the newest frame and the backlog is **dropped**. Live
never sees smooth video — it sees snapshots ~1.9 s apart. Replaying at `--stride 3` gives ~19-38x
less motion per processed frame, so the match fraction never falls below `match_frac_thresh: 0.333`
and almost no keyframes form.

**Consequence beyond this bench: every offline replay in this project has been running in a different
regime from live flight.** `plans/slam_report.html`'s note that input-resolution work is "a bench
experiment, not a flight" because raw recordings exist is wrong on that basis — they exist but do not
reproduce live conditions at any fixed stride (the live gaps are wildly uneven: median 76, p90 543,
max 1510 on the session-66 flight).

## 2. Making replay exact

- **`rec_frame` added to `DIAG_PERF_FIELDS`.** It already rode `TOPIC_POSE`
  (`perception_worker.py:361`) but never reached the CSV. `io_bridge` writes EVERY NDI video frame to
  `flight_<ts>.mp4`, so this value IS the frame number in that file — no `frame_id`->video offset
  arithmetic, no assumption about when `r` was pressed. Blank (never 0) when not recording:
  `rec_frame` 0 is the first recorded frame, so `int(x or 0)` would invent data.
- **`--frame-list <perception.csv>`** on `perception_worker.py`, with `load_frame_list()`. Replays the
  EXACT frames a flight's SLAM consumed. Raises on a CSV with no `rec_frame` column (pre-session-66
  flight) or all-blank cells (flown without recording) rather than degrading to stride behaviour.
- Frames are read sequentially and discarded, not seeked — OpenCV seeking is unreliable on a 1.1 GB
  mp4. Each run decodes all 56 000 frames; that is the I/O floor, not SLAM.

## 3. The window's real cost: four GPU syncs per solve

First frame-exact A/B (`flight_20260906_165141`, 319 frames; both runs 64 kf, 38 RELOC — matched):
**ON was 10.3% slower** (451.3 s vs 409.2 s).

The curve was doing the right thing — ON flat at ~3000-4000 ms while OFF climbed 742 -> 5222 — but ON
cost ~1.8 s more per solve **even below the window**, where the mask is all-True and the cut is a
provable no-op. That isolated the overhead as a fixed per-solve cost, not an algorithmic one.

Cause: the selection asked CUDA tensors four separate questions per solve, each a device->host sync
that drains the whole queue before the solve can start —

1. `int(torch.maximum(ii.max(), jj.max()))`
2. `anchor_idx = active_kf[active_kf < window_lo]` (boolean indexing: data-dependent output shape)
3. `int(anchor_idx.numel())`
4. the caller's `bool(sel.mask.all())`

**Fix:** the edge lists are tiny (~1.3 KB at this flight's 167-edge max), so pull them to the host
**once** and do every selection decision there. `EdgeSelection` gained `n_active_edges` and
`all_active` so callers never re-ask the device. Result at 0-9 kf: **2517 ms -> 749 ms**, against
OFF's 742 ms — identical, as it must be where the window does nothing.

## 4. The confound: run order

Identical OFF code, identical input, different position in the run sequence:

| | loop | 0-9 kf `backend_ms` |
|---|---|---|
| OFF, position 1 | 409.2 s | 742 ms |
| OFF, position 2 | 496.8 s | **3345 ms** |

The second run of a pair is systematically slower — up to 4.5x at low keyframe counts, where a fixed
per-solve latency dominates a small solve. **This is the same signature originally attributed to sync
overhead**, so the first A/B's "+10.3%" was partly position. The sync overhead was real (the
2517->749 drop at FIXED position proves it) but was not the whole of it.

Lesson for any future bench here: **never compare two sequential runs without controlling order.**
Use an ABBA design (A,B,B,A) so a linear drift cancels, or randomise and repeat.

## 5. The anchor leak — the deferred CUDA `num_fix` trigger is now firing

On real data at W=30: `solve_kf` max **42** = 30 window + **12 anchors**; `anchors>1` on **25 of 102**
solve frames; `anchor_drift` up to **0.489**. Under the `anchored` policy that motion is discarded at
write-back, so those 12 keyframes are optimised and the answer thrown away — paid for, unused.

The bound is 30 but the real solve is 43: a 43% overshoot. See `plans/session65-spec.md`'s deferred
section for the fix (make `num_fix` a kernel parameter; anchors are always the lowest sorted indices,
so "fix the first k" is exactly "fix all anchors" with no reordering).

## 6. ABBA verdict — the window ships ON

Four runs, `ON,OFF,OFF,ON`, same frame list. ABBA cancels a linear drift; the within-config spread
shows why it was needed (identical `OFF` code: 358.8 s vs 453.0 s = **26.3%** apart; identical `ON`
code: 17.8% apart).

| | ON (p1, p4 avg) | OFF (p2, p3 avg) | |
|---|---|---|---|
| loop | **364.3 s** | 405.9 s | ON 10.3% faster |
| backend | **233.2 s** | 265.1 s | ON 12.0% faster |

Median `backend_ms` on global solves, by total keyframes:

| kf | ON avg | OFF avg | |
|---|---|---|---|
| 0-9 | 1704 | 1697 | identical (window is a no-op) |
| 10-19 | 2241 | 2231 | identical |
| 20-29 | 2571 | 2196 | 17% worse (boundary: anchors appear, little cut yet) |
| 30-39 | 2201 | 3816 | **42% better** |
| 40-49 | 1931 | 4018 | **52% better** |
| 50-59 | 2316 | 4651 | **50% better** |
| 60-69 | 1896 | 4168 | **55% better** |

`ON` is flat (~1700-2600 ms) across every bucket while `OFF` climbs 1697 -> 4651. The ordering holds
in each of the four individual runs, not only the drift-cancelled means.

**Whole-flight gain is diluted** because this flight spends most of its frames under 30 keyframes,
where the window does nothing by construction. A longer survey sits longer in the >30 kf region where
it wins ~50%, so the benefit grows with flight length — which is exactly the growth-curve property
the lever was chosen for.

Shipped: `backend_window_mode: "ON"`, `backend_window_kf: 30`, `backend_window_policy: "anchored"`.
