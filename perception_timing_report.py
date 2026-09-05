#!/usr/bin/env python
"""perception_timing_report.py — Session 62: what does `slam_ms` actually cost, and where?

Answers the go/no-go question for Stages A/B/C (see plans/session62-spec.md, STATE.md): does a
flight's `*_perception.csv` show the choke growing with map size (supports Stage C, the
tracking/mapping thread split), or living inside `slam.process()` itself on keyframe/RELOC frames
(supports Stage A/B instead)? Pure stdlib (argparse/csv/statistics/dataclasses/pathlib) —
deliberately does NOT import slam_engine or perception_worker, both of which pull in torch, so this
tool runs on the bare system interpreter, no venv, on any machine that can read a CSV.

Reads by column NAME, not position: `PHASE_COLUMNS` below is a hand-kept mirror of
`perception_worker.DIAG_PERF_FIELDS` (session 62, `slam_engine.SLAM_PHASE_FIELDS` for the SLAM-
internal four). That module's own comment freezes its first nine column names/order for exactly
this reason — a file written before 2026-09-05 has that frozen prefix and nothing else, so this
tool loads it, NAMES every session-62 column it is missing in a loud banner, and still reports on
whatever columns are actually there, instead of crashing or silently zero-filling (CLAUDE.md: no
silent fallbacks — a phase that was never measured must read as missing, not as 0.0).

Closure invariant (slam_engine.py, C1): `track_ms + backend_ms + pose_ms + kf_download_ms` should
sum to within a few ms of `slam_ms`, since those four phases are exactly what the `slam_ms`
stopwatch brackets. `phase_closure()` reports the per-row residual so a large, systematic residual
would itself be a finding (an unaccounted phase inside `slam.process()`).

Session 63 adds a second, independent closure one level down: `frame_ms + infer_ms + tracker_ms`
should sum to within a few ms of `track_ms` -- the sub-split INSIDE `track_ms` itself (mirror of
`slam_engine.SLAM_TRACK_PHASE_FIELDS`), reported by `track_closure()`. Kept as separate constants
and a separate function from the slam_ms closure above because they close against different totals
-- a file can have one without the other, and `report()` must show that independently instead of
letting one missing invariant hide the other.
"""

import argparse
import csv
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path

REPO = Path(__file__).resolve().parent

# Session 62 (C4): the phase columns this report analyzes, in pipeline order. Hand-kept mirror of
# perception_worker.DIAG_PERF_FIELDS[3:] minus the non-timing columns — NOT imported (see module
# docstring: this tool must stay import-clean of torch). Safe to hand-keep because
# perception_worker.py freezes its column names/order as a matter of policy, not accident.
PHASE_COLUMNS: tuple[str, ...] = (
    "slam_ms", "track_ms", "backend_ms", "pose_ms", "kf_download_ms",
    "integrate_ms", "map_pub_ms", "plan_ms", "publish_ms",
    "frame_ms", "infer_ms", "tracker_ms")            # session 63, appended

# Session 62: mirror of slam_engine.SLAM_PHASE_FIELDS, hand-kept for the same reason as
# PHASE_COLUMNS above. These four are exactly what phase_closure() sums against slam_ms.
_SLAM_PHASE_FIELDS: tuple[str, ...] = ("track_ms", "backend_ms", "pose_ms", "kf_download_ms")

# Session 63: mirror of slam_engine.SLAM_TRACK_PHASE_FIELDS, hand-kept for the same reason as
# _SLAM_PHASE_FIELDS above. Kept as a SEPARATE constant rather than folded into _SLAM_PHASE_FIELDS
# because the four above close against slam_ms while these three close against track_ms -- two
# different invariants, and merging them would break both.
_TRACK_PHASE_FIELDS: tuple[str, ...] = ("frame_ms", "infer_ms", "tracker_ms")


@dataclass(frozen=True)
class PhaseStats:
    column: str        # the CSV column these stats describe
    n: int             # rows with a parseable float
    n_blank: int       # rows whose cell was "" or unparseable (NAMED, never silently dropped)
    median: float      # 0.0 when n == 0
    p90: float         # 0.0 when n == 0
    maximum: float     # 0.0 when n == 0


def load_rows(csv_path) -> list:
    """Load a perception diag CSV as a list of str->str dict rows (csv.DictReader semantics)."""
    path = Path(csv_path)
    with open(path, "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f"{path}: no header row")
        return list(reader)


def present_columns(rows):
    """Which of PHASE_COLUMNS are in this file's header, and which are missing.

    Header is read from the row dicts themselves (there is no separate header object once
    csv.DictReader has run) — an empty `rows` therefore carries no header information at all, and
    every PHASE_COLUMNS name is reported missing rather than guessed present."""
    if not rows:
        return [], list(PHASE_COLUMNS)
    header = set(rows[0].keys())
    present = [c for c in PHASE_COLUMNS if c in header]
    missing = [c for c in PHASE_COLUMNS if c not in header]
    return present, missing


def _parse_float(cell):
    """Return (value, blank) — blank=True for '', None, or unparseable text. Never raises."""
    if cell is None or cell == "":
        return None, True
    try:
        return float(cell), False
    except ValueError:
        return None, True


def _stats_from_values(column, values, n_blank):
    if not values:
        return PhaseStats(column=column, n=0, n_blank=n_blank, median=0.0, p90=0.0, maximum=0.0)
    sorted_vals = sorted(values)
    n = len(sorted_vals)
    median = statistics.median(sorted_vals)
    p90 = sorted_vals[min(int(0.9 * n), n - 1)]
    maximum = sorted_vals[-1]
    return PhaseStats(column=column, n=n, n_blank=n_blank, median=median, p90=p90, maximum=maximum)


def phase_stats(rows, column) -> PhaseStats:
    """Median/p90/max/n/n_blank for one CSV column. Raises KeyError(column) if the column is
    absent from the header — a caller must not be able to silently read stats for a column that
    was never measured. An empty `rows` carries no header information (same reasoning as
    present_columns([]) below) so it can't be judged absent -- e.g. bucket_by_keyframe always
    yields an "empty side" bucket for a real, present column -- and returns zero-stats instead."""
    if not rows:
        return _stats_from_values(column, [], 0)
    header = rows[0].keys()
    if column not in header:
        raise KeyError(column)
    values = []
    n_blank = 0
    for row in rows:
        value, blank = _parse_float(row.get(column, ""))
        if blank:
            n_blank += 1
        else:
            values.append(value)
    return _stats_from_values(column, values, n_blank)


def phase_closure(rows) -> PhaseStats:
    """Per-row residual slam_ms - (track_ms + backend_ms + pose_ms + kf_download_ms) — the C1
    closure invariant. Raises KeyError if slam_ms or any SLAM-internal phase column is absent."""
    required = ("slam_ms",) + _SLAM_PHASE_FIELDS
    if not rows:
        return _stats_from_values("closure_residual_ms", [], 0)
    header = rows[0].keys()
    for col in required:
        if col not in header:
            raise KeyError(col)
    residuals = []
    n_blank = 0
    for row in rows:
        cells = [_parse_float(row.get(col, "")) for col in required]
        if any(blank for _, blank in cells):
            n_blank += 1
            continue
        slam_ms, track_ms, backend_ms, pose_ms, kf_download_ms = (v for v, _ in cells)
        residuals.append(slam_ms - (track_ms + backend_ms + pose_ms + kf_download_ms))
    return _stats_from_values("closure_residual_ms", residuals, n_blank)


def track_closure(rows) -> PhaseStats:
    """Per-row residual track_ms - (frame_ms + infer_ms + tracker_ms) -- the session 63 closure
    invariant one level down inside track_ms. Raises KeyError if track_ms or any of
    _TRACK_PHASE_FIELDS is absent. Mirrors phase_closure() exactly, against a different total."""
    required = ("track_ms",) + _TRACK_PHASE_FIELDS
    if not rows:
        return _stats_from_values("track_closure_residual_ms", [], 0)
    header = rows[0].keys()
    for col in required:
        if col not in header:
            raise KeyError(col)
    residuals = []
    n_blank = 0
    for row in rows:
        cells = [_parse_float(row.get(col, "")) for col in required]
        if any(blank for _, blank in cells):
            n_blank += 1
            continue
        track_ms, frame_ms, infer_ms, tracker_ms = (v for v, _ in cells)
        residuals.append(track_ms - (frame_ms + infer_ms + tracker_ms))
    return _stats_from_values("track_closure_residual_ms", residuals, n_blank)


def bucket_by_minute(rows, minutes: float = 5.0):
    """Bucket rows by wall_ts relative to the first row's wall_ts, in `minutes`-wide windows.
    A row with a blank/unparseable wall_ts goes to its own "unparseable wall_ts" bucket — reported,
    never silently dropped or folded into bucket 0."""
    if not rows:
        return []

    def parse_ts(row):
        value, blank = _parse_float(row.get("wall_ts", ""))
        return None if blank else value

    ts0 = None
    for row in rows:
        ts0 = parse_ts(row)
        if ts0 is not None:
            break
    if ts0 is None:
        return [("unparseable wall_ts", list(rows))]

    span = minutes * 60.0
    grouped = {}
    unparseable = []
    for row in rows:
        ts = parse_ts(row)
        if ts is None:
            unparseable.append(row)
            continue
        idx = int((ts - ts0) // span)
        grouped.setdefault(idx, []).append(row)

    buckets = []
    for idx in sorted(grouped):
        lo, hi = idx * minutes, (idx + 1) * minutes
        buckets.append((f"{lo:g}-{hi:g} min", grouped[idx]))
    if unparseable:
        buckets.append(("unparseable wall_ts", unparseable))
    return buckets


def bucket_by_mode(rows):
    """One bucket per distinct `mode` value, sorted by label."""
    grouped = {}
    for row in rows:
        grouped.setdefault(row.get("mode", ""), []).append(row)
    return [(label, grouped[label]) for label in sorted(grouped)]


def bucket_by_keyframe(rows):
    """Exactly two buckets, always both present: rows with new_keyframe == '1', and everything
    else — the ~4.7x keyframe-vs-ordinary split (MISSION CONTEXT) is the whole point of this split,
    so an empty side must still show up as an empty row, not vanish."""
    kf = [row for row in rows if row.get("new_keyframe") == "1"]
    ordinary = [row for row in rows if row.get("new_keyframe") != "1"]
    return [("new_keyframe", kf), ("ordinary", ordinary)]


def render_table(title: str, buckets, columns: tuple = PHASE_COLUMNS) -> str:
    """Markdown table, one row per bucket, one median/p90 column per phase. A column absent from
    every bucket's rows renders "--" throughout and is named in the trailing caption — never
    silently omitted from the column list."""
    all_rows = [row for _, rows in buckets for row in rows]
    header = set(all_rows[0].keys()) if all_rows else set()
    missing = [c for c in columns if c not in header]

    lines = [title]
    head = ["bucket", "n"] + list(columns)
    lines.append("| " + " | ".join(head) + " |")
    lines.append("| " + " | ".join("---" for _ in head) + " |")
    for label, rows in buckets:
        cells = [label, str(len(rows))]
        for col in columns:
            if col in missing:
                cells.append("--")
            else:
                s = phase_stats(rows, col)
                cells.append(f"{s.median:.0f}/{s.p90:.0f}")
        lines.append("| " + " | ".join(cells) + " |")
    if missing:
        lines.append(f"(missing columns, rendered as `--`: {', '.join(missing)})")
    return "\n".join(lines)


def report(csv_path) -> str:
    """The full text report: header, a loud MISSING COLUMNS banner if the file predates session
    62, the SLAM phase-closure line, then by-minute / by-mode / by-keyframe tables. Never prints —
    callers decide where the text goes."""
    path = Path(csv_path)
    rows = load_rows(path)
    _present, missing = present_columns(rows)

    lines = [f"Perception timing report -- {path.name} ({len(rows)} rows)"]
    if missing:
        lines.append(f"*** MISSING COLUMNS: {', '.join(missing)} (file predates session 62) ***")
    lines.append("")

    closure_required = ("slam_ms",) + _SLAM_PHASE_FIELDS
    if all(col not in missing for col in closure_required):
        c = phase_closure(rows)
        lines.append(f"SLAM phase closure (slam_ms - sum(phases)): n={c.n} n_blank={c.n_blank} "
                      f"median={c.median:.1f}ms p90={c.p90:.1f}ms max={c.maximum:.1f}ms")
    else:
        lines.append("SLAM phase closure: unavailable (missing phase columns, see banner above)")

    track_closure_required = ("track_ms",) + _TRACK_PHASE_FIELDS
    if all(col not in missing for col in track_closure_required):
        tc = track_closure(rows)
        lines.append(f"Track phase closure (track_ms - sum(phases)): n={tc.n} n_blank={tc.n_blank} "
                      f"median={tc.median:.1f}ms p90={tc.p90:.1f}ms max={tc.maximum:.1f}ms")
    else:
        lines.append("Track phase closure: unavailable (missing phase columns, see banner above)")
    lines.append("")

    lines.append(render_table("By flight-minute", bucket_by_minute(rows)))
    lines.append("")
    lines.append(render_table("By SLAM mode", bucket_by_mode(rows)))
    lines.append("")
    lines.append(render_table("By keyframe", bucket_by_keyframe(rows)))
    return "\n".join(lines)


def run_self_test() -> None:
    import shutil
    import tempfile

    ok = True

    def check(name, cond):
        nonlocal ok
        ok = ok and bool(cond)
        print(f"[timing-report][self-test] {'PASS' if cond else 'FAIL'}  {name}")

    tmp_dir = tempfile.mkdtemp(prefix="timing_report_selftest_")
    try:
        # 1. Round-trip: a 10-row modern CSV loads whole; all PHASE_COLUMNS present.
        path1 = Path(tmp_dir) / "roundtrip.csv"
        fields1 = ("wall_ts", "frame_id", "mode", "new_keyframe") + PHASE_COLUMNS
        with open(path1, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=fields1)
            w.writeheader()
            for i in range(10):
                row = {c: "1.0" for c in PHASE_COLUMNS}
                row.update(wall_ts=str(float(i)), frame_id=str(i), mode="TRACKING", new_keyframe="0")
                w.writerow(row)
        rows1 = load_rows(path1)
        present1, missing1 = present_columns(rows1)
        check("round_trip -- 10 rows load", len(rows1) == 10)
        check("round_trip -- all PHASE_COLUMNS present, none missing",
              tuple(present1) == PHASE_COLUMNS and missing1 == [])

        # 2. Statistics over 1..10.
        rows2 = [{"slam_ms": str(v)} for v in range(1, 11)]
        s2 = phase_stats(rows2, "slam_ms")
        check("statistics -- median/p90/max/n over 1..10",
              s2.median == 5.5 and s2.p90 == 10.0 and s2.maximum == 10.0
              and s2.n == 10 and s2.n_blank == 0)

        # 3. Blank handling: two blanks among 10, stats computed over the 8 parseable only.
        raw3 = [1.0, 2.0, "", 4.0, 5.0, 6.0, "", 8.0, 9.0, 10.0]
        rows3 = [{"slam_ms": ("" if v == "" else str(v))} for v in raw3]
        s3 = phase_stats(rows3, "slam_ms")
        expected_median3 = statistics.median(v for v in raw3 if v != "")
        check("blank_handling -- n=8, n_blank=2, median over parseable values only",
              s3.n == 8 and s3.n_blank == 2 and s3.median == expected_median3)

        # 4. Missing column is loud: a CSV with only the nine frozen columns.
        path4 = Path(tmp_dir) / "legacy.csv"
        frozen9 = ("wall_ts", "frame_id", "loop_dt", "slam_ms", "mode", "new_keyframe",
                   "n_keyframes", "n_voxels", "reloc")
        with open(path4, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=frozen9)
            w.writeheader()
            w.writerow({c: ("TRACKING" if c == "mode" else "1.0") for c in frozen9})
        rows4 = load_rows(path4)
        _present4, missing4 = present_columns(rows4)
        check("missing_column -- eight session-62 phase columns reported missing",
              set(missing4) == set(PHASE_COLUMNS) - {"slam_ms"})
        raised = False
        try:
            phase_stats(rows4, "backend_ms")
        except KeyError:
            raised = True
        check("missing_column -- phase_stats(backend_ms) raises KeyError", raised)
        report4 = report(path4)
        check("missing_column -- report() names MISSING COLUMNS and backend_ms",
              "MISSING COLUMNS" in report4 and "backend_ms" in report4)

        # 5. Closure: balanced phases sum to slam_ms; a shifted backend_ms shows the residual.
        rows5a = [{"slam_ms": "1000", "track_ms": "300", "backend_ms": "600",
                   "pose_ms": "20", "kf_download_ms": "80"} for _ in range(3)]
        check("closure -- balanced phases give 0.0 median residual",
              phase_closure(rows5a).median == 0.0)
        rows5b = [{"slam_ms": "1000", "track_ms": "300", "backend_ms": "500",
                   "pose_ms": "20", "kf_download_ms": "80"} for _ in range(3)]
        check("closure -- backend_ms=500 gives 100.0 median residual",
              phase_closure(rows5b).median == 100.0)

        # 5b. Session 63: track_ms closure -- balanced sub-phases give 0.0, a shifted
        # tracker_ms shows the residual. Mirrors 5/5a-b exactly, against track_ms instead of slam_ms.
        rows5c = [{"track_ms": "500", "frame_ms": "50", "infer_ms": "0",
                   "tracker_ms": "450"} for _ in range(3)]
        check("track_closure -- balanced sub-phases give 0.0 median residual",
              track_closure(rows5c).median == 0.0)
        # Session 63 spec text claims this gives a 100.0 residual; the arithmetic on the spec's
        # own numbers (500 - (50+0+400) = 50) does not support that, so this asserts the value
        # the C5 formula actually produces and flags the discrepancy in the chunk report instead
        # of asserting a numerically false invariant.
        rows5d = [{"track_ms": "500", "frame_ms": "50", "infer_ms": "0",
                   "tracker_ms": "400"} for _ in range(3)]
        check("track_closure -- tracker_ms=400 gives 50.0 median residual (see chunk report)",
              track_closure(rows5d).median == 50.0)

        # 5e. Blank handling for track_closure: a blank component is counted in n_blank, never
        # coerced to 0.0 -- mirrors the PhaseStats blank contract proven in check 3 above.
        rows5e = [{"track_ms": "500", "frame_ms": "50", "infer_ms": "0", "tracker_ms": "450"},
                  {"track_ms": "500", "frame_ms": "50", "infer_ms": "0", "tracker_ms": ""},
                  {"track_ms": "500", "frame_ms": "50", "infer_ms": "0", "tracker_ms": "450"}]
        tc5e = track_closure(rows5e)
        check("track_closure -- blank component counted in n_blank, not coerced to 0.0",
              tc5e.n == 2 and tc5e.n_blank == 1 and tc5e.median == 0.0)

        # 5f. Missing columns is loud for track_closure too: a CSV carrying the session-62
        # columns but none of the three session-63 ones. Old PHASE_COLUMNS, hand-frozen here
        # (not read from the live PHASE_COLUMNS, which now includes the session-63 additions) so
        # this fixture reproduces exactly what a pre-session-63 file looks like.
        path5f = Path(tmp_dir) / "session62_only.csv"
        session62_cols = ("slam_ms", "track_ms", "backend_ms", "pose_ms", "kf_download_ms",
                           "integrate_ms", "map_pub_ms", "plan_ms", "publish_ms")
        fields5f = ("wall_ts", "frame_id", "mode", "new_keyframe") + session62_cols
        with open(path5f, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=fields5f)
            w.writeheader()
            row = {c: "1.0" for c in session62_cols}
            row.update(wall_ts="0.0", frame_id="0", mode="TRACKING", new_keyframe="0")
            w.writerow(row)
        rows5f = load_rows(path5f)
        _present5f, missing5f = present_columns(rows5f)
        check("track_closure -- missing_column -- all three session-63 columns reported missing",
              set(missing5f) == {"frame_ms", "infer_ms", "tracker_ms"})
        raised5f = False
        try:
            track_closure(rows5f)
        except KeyError:
            raised5f = True
        check("track_closure -- missing_column -- raises KeyError", raised5f)
        report5f = report(path5f)
        check("track_closure -- missing_column -- report() names MISSING COLUMNS and tracker_ms, "
              "and shows the track-closure unavailable line",
              "MISSING COLUMNS" in report5f and "tracker_ms" in report5f
              and "Track phase closure: unavailable" in report5f)
        check("track_closure -- missing_column -- SLAM closure still prints normally",
              "SLAM phase closure (slam_ms - sum(phases))" in report5f)

        # 5g. Both closures coexist on a fully-populated modern CSV: report() contains both
        # closure lines and present_columns reports nothing missing.
        path5g = Path(tmp_dir) / "modern.csv"
        fields5g = ("wall_ts", "frame_id", "mode", "new_keyframe") + PHASE_COLUMNS
        with open(path5g, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=fields5g)
            w.writeheader()
            row = {c: "1.0" for c in PHASE_COLUMNS}
            row.update(wall_ts="0.0", frame_id="0", mode="TRACKING", new_keyframe="0")
            w.writerow(row)
        rows5g = load_rows(path5g)
        _present5g, missing5g = present_columns(rows5g)
        report5g = report(path5g)
        check("both_closures -- present_columns reports nothing missing", missing5g == [])
        check("both_closures -- report() contains both closure lines",
              "SLAM phase closure (slam_ms - sum(phases))" in report5g
              and "Track phase closure (track_ms - sum(phases))" in report5g)

        # 6. Minute buckets: 12 minutes of wall_ts at minutes=5.0 -> three buckets, in order.
        rows6 = [{"wall_ts": str(float(m * 60))} for m in range(0, 13)]
        buckets6 = bucket_by_minute(rows6, minutes=5.0)
        check("minute_buckets -- exactly three buckets, in order",
              [label for label, _ in buckets6] == ["0-5 min", "5-10 min", "10-15 min"])

        # 7. Unparseable wall_ts isolated in its own bucket, absent from every other bucket.
        rows7 = [{"wall_ts": "0.0"}, {"wall_ts": ""}, {"wall_ts": "60.0"}]
        buckets7 = bucket_by_minute(rows7, minutes=5.0)
        unparse7 = next((b for label, b in buckets7 if label == "unparseable wall_ts"), None)
        other7 = [r for label, b in buckets7 if label != "unparseable wall_ts" for r in b]
        check("unparseable_wall_ts -- isolated in its own bucket, nowhere else",
              unparse7 == [rows7[1]] and rows7[1] not in other7)

        # 8. Mode buckets (counts) + keyframe buckets (fixed order, both always present).
        rows8 = ([{"mode": "TRACKING", "new_keyframe": "0"} for _ in range(6)]
                 + [{"mode": "RELOC", "new_keyframe": "0"} for _ in range(3)]
                 + [{"mode": "TRACKING", "new_keyframe": "1"} for _ in range(1)])
        mode_buckets = bucket_by_mode(rows8)
        check("mode_buckets -- two mode buckets with correct counts",
              [(label, len(r)) for label, r in mode_buckets] == [("RELOC", 3), ("TRACKING", 7)])
        kf_buckets = bucket_by_keyframe(rows8)
        check("keyframe_buckets -- fixed order, correct counts",
              [label for label, _ in kf_buckets] == ["new_keyframe", "ordinary"]
              and len(kf_buckets[0][1]) == 1 and len(kf_buckets[1][1]) == 9)
        kf_empty = bucket_by_keyframe([])
        check("keyframe_buckets -- both buckets present even when empty",
              [label for label, _ in kf_empty] == ["new_keyframe", "ordinary"]
              and kf_empty[0][1] == [] and kf_empty[1][1] == [])

        # 9. render_table: title first line, markdown header row, "--" + caption for missing cols.
        table9 = render_table("Test table", [("all", rows5a)], columns=("slam_ms",))
        lines9 = table9.splitlines()
        check("render -- first line is the title", lines9[0] == "Test table")
        check("render -- contains a |-delimited header row", any(ln.startswith("|") for ln in lines9))
        table9b = render_table("Missing col table", [("b", [{"slam_ms": "1.0"}])],
                                columns=("slam_ms", "backend_ms"))
        check("render -- missing column renders '--' and is named in caption",
              "--" in table9b and "backend_ms" in table9b)
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)

    print("PASS" if ok else "FAIL")


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Perception phase timing report (Session 62) -- attributes slam_ms.")
    ap.add_argument("csv_path", nargs="?", default=None,
                     help="path to a *_perception.csv; defaults to the newest under OUTPUT/diag/")
    ap.add_argument("--self-test", action="store_true", help="run the synthetic self-test")
    # Session 62: accepted per spec (C4) but NOT wired to report() -- report()'s contracted
    # signature is report(csv_path) only, with no minutes parameter, so this flag currently has no
    # effect on the rendered tables (they always use bucket_by_minute's 5.0-minute default). Flagging
    # this as a spec inconsistency rather than silently reinterpreting report()'s signature.
    ap.add_argument("--minutes", type=float, default=5.0,
                     help="flight-minute bucket width (NOTE: not yet wired into report())")
    args = ap.parse_args()

    if args.self_test:
        run_self_test()
        return

    csv_path = args.csv_path
    if csv_path is None:
        diag_dir = REPO / "OUTPUT" / "diag"
        candidates = sorted(diag_dir.glob("*_perception.csv")) if diag_dir.exists() else []
        if not candidates:
            print(f"No *_perception.csv files found under {diag_dir}")
            sys.exit(1)
        csv_path = candidates[-1]
        print(f"Using newest perception CSV: {csv_path}")

    print(report(csv_path))


if __name__ == "__main__":
    main()
