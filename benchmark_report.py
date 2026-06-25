"""benchmark_report.py — build a single, shareable visual report from a benchmark run.

Reads OUTPUT/benchmark/{per_frame.json, summary.json} + the overlays/ PNGs written by
benchmark_detectors.py and emits ONE portable file: OUTPUT/benchmark/benchmark_report.html.
Each test frame's overlay (GT box = green, predicted = cyan when the center lands in GT, red when
it doesn't / when it false-fires on a negative) is downscaled, JPEG-encoded and base64-EMBEDDED, so
the html is self-contained (no external image files to ship). Next to each frame is a sidebar with
the frame id, engine, target, latency, score (abs_logit for OWLv2), IoU, etc. — so you can see WHY a
score happened, not just the aggregate.

Run benchmark_detectors.py first (WITHOUT --no-overlays). Then:
    venv\\Scripts\\python.exe benchmark_report.py
    venv\\Scripts\\python.exe benchmark_report.py --max-width 800
"""

import argparse
import base64
import html
import json
from pathlib import Path

import cv2

REPO = Path(__file__).resolve().parent

SUMMARY_COLS = [
    ("engine", "engine"), ("target", "target"), ("n_pos", "n_pos"),
    ("recall_found", "recall"), ("good_detect", "good"), ("loc_ok_of_found", "loc/found"),
    ("fp_rate_neg", "FP_neg"), ("pos_score_med", "pos_med"), ("neg_score_med", "neg_med"),
    ("median_ms", "ms"),
]

LEGEND = """
<b>recall</b> = fired on a target-present frame, regardless of WHERE (a fire at a blank wall still counts).
&nbsp;|&nbsp; <b>good</b> = fired AND predicted center inside the GT box (x1&le;cx&le;x2 and y1&le;cy&le;y2) — the real hit rate.
&nbsp;|&nbsp; <b>loc/found</b> = of the frames it fired on, how many were on target (reliability when it speaks).
&nbsp;|&nbsp; <b>FP_neg</b> = of empty (None) frames, how many it hallucinated a target on (1.00 = every one).
&nbsp;|&nbsp; <b>pos_med/neg_med</b> = median score on positives vs negatives; the gap is the separability.
"""

CSS = """
body{font-family:-apple-system,Segoe UI,Roboto,sans-serif;margin:0;background:#14161a;color:#e6e6e6}
h1{padding:16px 20px 0}h2{padding:18px 20px 4px;border-top:1px solid #2a2e35;margin-top:24px}
.wrap{padding:0 20px 40px}
.legend{background:#1d2027;border:1px solid #2a2e35;border-radius:8px;padding:10px 14px;margin:8px 0 4px;font-size:13px;line-height:1.7}
table.sum{border-collapse:collapse;margin:10px 0;font-size:13px}
table.sum th,table.sum td{border:1px solid #333;padding:4px 10px;text-align:right}
table.sum th{background:#22262e;text-align:center}
table.sum td.k{text-align:left}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(420px,1fr));gap:12px;margin-top:8px}
.card{display:flex;background:#1d2027;border:2px solid #444;border-radius:8px;overflow:hidden}
.card.good{border-color:#1f9d55}.card.bad{border-color:#d64550}.card.none{border-color:#555}
.card img{width:62%;object-fit:contain;background:#000}
.side{width:38%;padding:8px 10px;font-size:12px;line-height:1.55;word-break:break-word}
.side .f{color:#8fb6ff;font-weight:600}.side .lbl{color:#9aa3ad}
.tag{display:inline-block;padding:1px 6px;border-radius:4px;font-weight:600;font-size:11px}
.tag.y{background:#1f9d55}.tag.n{background:#555}.tag.r{background:#d64550}
"""


def thumb_b64(img_path: Path, max_w: int):
    img = cv2.imread(str(img_path))
    if img is None:
        return None
    h, w = img.shape[:2]
    if w > max_w:
        img = cv2.resize(img, (max_w, int(round(h * max_w / w))), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if not ok:
        return None
    return base64.b64encode(buf).decode("ascii")


def overlay_path(out_dir: Path, rec):
    sub = rec["target"] if rec["is_pos"] else f"None_vs_{rec['target']}"
    return out_dir / "overlays" / rec["engine"] / sub / rec["frame"]


def card_class(rec):
    if not rec["found"]:
        return "none" if rec["is_pos"] else "good"      # silent on a negative = correct
    if rec["is_pos"]:
        return "good" if rec["center_in_gt"] else "bad"
    return "bad"                                         # fired on a negative = false positive


def render_card(rec, b64):
    e = rec["engine"]
    score_lbl = "abs_logit" if e == "owlv2" else "score"
    rows = [
        ("frame", rec["frame"]),
        ("engine / target", f"{e} / {rec['target']}"),
        ("found", f"<span class='tag {'y' if rec['found'] else 'n'}'>{rec['found']}</span>"),
        (score_lbl, f"{rec['score']:.3f}"),
    ]
    if rec.get("norm_top_score") is not None:
        rows.append(("norm_top_score", f"{rec['norm_top_score']:.3f}"))
    if rec["is_pos"]:
        ok = rec["center_in_gt"]
        rows.append(("center_in_GT", f"<span class='tag {'y' if ok else 'r'}'>{ok}</span>"))
        rows.append(("IoU", f"{rec['iou']:.3f}"))
    else:
        fp = rec["found"]
        rows.append(("false_positive", f"<span class='tag {'r' if fp else 'y'}'>{fp}</span>"))
    rows += [
        ("gt_bbox", rec["gt_bbox"]),
        ("pred_bbox", rec["pred_bbox"]),
        ("latency", f"{rec['infer_ms']:.0f} ms"),
    ]
    side = "".join(f"<div><span class='lbl'>{html.escape(k)}:</span> {v}</div>" for k, v in rows)
    img = (f"<img src='data:image/jpeg;base64,{b64}'>" if b64
           else "<div class='side'>[overlay image missing]</div>")
    return f"<div class='card {card_class(rec)}'>{img}<div class='side'>{side}</div></div>"


def render_summary_table(summary):
    head = "".join(f"<th>{html.escape(lbl)}</th>" for _, lbl in SUMMARY_COLS)
    body = []
    for v in summary.values():
        tds = []
        for key, _ in SUMMARY_COLS:
            val = v.get(key, "")
            cls = " class='k'" if key in ("engine", "target") else ""
            tds.append(f"<td{cls}>{html.escape(str(val))}</td>")
        body.append("<tr>" + "".join(tds) + "</tr>")
    return f"<table class='sum'><tr>{head}</tr>{''.join(body)}</table>"


def main():
    ap = argparse.ArgumentParser(description="Build benchmark_report.html from a benchmark run.")
    ap.add_argument("--dir", default="OUTPUT/benchmark", help="benchmark output dir (under repo)")
    ap.add_argument("--max-width", type=int, default=600, help="embedded thumbnail max width (px)")
    args = ap.parse_args()

    out_dir = (REPO / args.dir).resolve()
    pf = out_dir / "per_frame.json"
    sm = out_dir / "summary.json"
    if not pf.exists() or not sm.exists():
        raise SystemExit(f"missing {pf.name}/{sm.name} in {out_dir} — run benchmark_detectors.py first.")
    records = json.loads(pf.read_text(encoding="utf-8"))
    summary = json.loads(sm.read_text(encoding="utf-8"))
    if not (out_dir / "overlays").exists():
        print(f"[report] WARNING: no overlays/ in {out_dir} — re-run benchmark_detectors.py WITHOUT "
              f"--no-overlays for images. Building a metadata-only report.")

    # group: engine -> [ (section_title, [records]) ], positives first then negatives, by target.
    engines = sorted({r["engine"] for r in records})
    targets = sorted({r["target"] for r in records})
    sections = []  # (engine, title, recs)
    n_missing = 0
    cache = {}
    for e in engines:
        for is_pos, label in ((True, "positives"), (False, "negatives (None)")):
            for t in targets:
                recs = [r for r in records if r["engine"] == e and r["is_pos"] == is_pos and r["target"] == t]
                if recs:
                    sections.append((e, f"{e} — {t} — {label}", recs))

    parts = [f"<!doctype html><html><head><meta charset='utf-8'><title>Detector benchmark</title>",
             f"<style>{CSS}</style></head><body>",
             "<h1>Detector benchmark — visual report</h1>",
             f"<div class='wrap'><div class='legend'>{LEGEND}</div>",
             render_summary_table(summary)]
    for _, title, recs in sections:
        parts.append(f"<h2>{html.escape(title)}</h2><div class='grid'>")
        for r in recs:
            p = overlay_path(out_dir, r)
            if p not in cache:
                cache[p] = thumb_b64(p, args.max_width)
            b64 = cache[p]
            if b64 is None:
                n_missing += 1
            parts.append(render_card(r, b64))
        parts.append("</div>")
    parts.append("</div></body></html>")

    report = out_dir / "benchmark_report.html"
    report.write_text("".join(parts), encoding="utf-8")
    size_mb = report.stat().st_size / 1e6
    print(f"[report] wrote {report}  ({len(records)} frames, {size_mb:.1f} MB"
          + (f", {n_missing} overlays missing)" if n_missing else ")"))


if __name__ == "__main__":
    main()
