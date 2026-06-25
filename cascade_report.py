"""cascade_report.py — one shareable, self-contained HTML report for a cascade_detector.py run.

Reads OUTPUT/cascade/{per_frame.json, summary.json} + the overlays/ written by cascade_detector.py
and emits OUTPUT/cascade/cascade_report.html. Each frame's composite overlay (GT=green,
proposals=gray, Stage-2 survivors=yellow, final=cyan-correct/red-wrong) is downscaled, JPEG-encoded
and base64-EMBEDDED, so the html ships as a single file. The sidebar shows the FUNNEL: the Stage-1
recall ceiling, then every candidate's per-stage scores (stage1 source/score -> DINOv2 cosine ->
geometric inliers) and its accept/reject reason — so you can see WHY each box lived or died.

Run cascade_detector.py first (without --no-overlays). Then:
    venv\\Scripts\\python.exe cascade_report.py
    venv\\Scripts\\python.exe cascade_report.py --max-width 800
"""

import argparse
import html
import json
from pathlib import Path

# Reuse the embedding + styling primitives from the benchmark report (import is side-effect-free).
from benchmark_report import CSS, thumb_b64

REPO = Path(__file__).resolve().parent

SUMMARY_COLS = [
    ("target", "target"), ("n_pos", "n_pos"),
    ("stage1_ceiling", "S1 ceiling"), ("stage1_ceiling_gd", "S1 gd"),
    ("stage1_ceiling_owlv2", "S1 owlv2"), ("stage2_good", "S2 good"), ("stage2_fp", "S2 fp"),
    ("final_good", "final good"), ("final_fp", "final fp"),
    ("dino_pos_med", "dino pos"), ("dino_neg_med", "dino neg"),
]

LEGEND = """
<b>S1 ceiling</b> = fraction of target-present frames where SOME proposal covered GT — the cascade's
recall ceiling (S1 gd / S1 owlv2 = each proposer alone).
&nbsp;|&nbsp; <b>S2 good / S2 fp</b> = after the DINOv2 verifier, before geometry (good = a survivor on target; fp = any survivor on an empty frame).
&nbsp;|&nbsp; <b>final good</b> = accepted box centered in GT. <b>final fp</b> = accepted box on an empty frame.
&nbsp;|&nbsp; <b>dino pos / neg</b> = median of the best candidate's DINOv2 cosine on positives vs negatives; the gap is the verifier's separability.
&nbsp;|&nbsp; Overlay: <span style='color:#0f0'>GT green</span>, proposals gray, <span style='color:#ff0'>Stage-2 survivors yellow</span>, final <span style='color:#0ff'>cyan=on-target</span>/<span style='color:#f55'>red=wrong</span>.
"""

EXTRA_CSS = """
.cand{border-top:1px solid #2a2e35;padding:3px 0;margin-top:3px}
.cand.acc{color:#bfeccb}.cand.rej{color:#c8a0a0}
.src{display:inline-block;min-width:46px;color:#8fb6ff;font-weight:600}
.rr{color:#d68a8a;font-style:italic}
.ok{color:#5fd08a}.no{color:#d06a6a}
"""


def overlay_path(out_dir: Path, rec):
    sub = rec["target"] if rec["is_pos"] else f"None_vs_{rec['target']}"
    return out_dir / "overlays" / sub / rec["frame"]


def card_class(rec):
    if not rec["found"]:
        return "none" if rec["is_pos"] else "good"      # silent on a negative = correct
    if rec["is_pos"]:
        return "good" if rec["center_in_gt"] else "bad"
    return "bad"                                         # fired on a negative = false positive


def render_cand(c):
    cls = "acc" if c["accepted"] else "rej"
    bits = [f"<span class='src'>{html.escape(str(c['source']))}</span>",
            f"s1={c['stage1_score']:.2f}",
            f"dino={c['dino_cls']:.2f}",
            f"({c['geom_engine']} {c['geom_inliers']})"]
    if c["covers_gt"]:
        bits.append("<span class='ok'>covers_GT</span>")
    tail = ("<span class='ok'>ACCEPTED</span>" if c["accepted"]
            else f"<span class='rr'>{html.escape(str(c['reject_reason']))}</span>")
    note = f" <span class='rr'>[{html.escape(c['geom_note'])}]</span>" if c.get("geom_note") else ""
    return f"<div class='cand {cls}'>{' '.join(bits)} &rarr; {tail}{note}</div>"


def render_card(rec, b64):
    s1 = rec.get("stage1_covers_gt")
    s1txt = ("n/a" if s1 is None
             else f"<span class='{'ok' if s1 else 'no'}'>{'covered' if s1 else 'MISSED'}</span>")
    head = [
        ("frame", rec["frame"]),
        ("target", rec["target"]),
        ("stage1 covers GT", s1txt),
        ("candidates", f"{rec['n_cands']} (stage2 survivors: {rec['n_stage2_survivors']})"),
        ("final", (f"<span class='ok'>{rec['final_source']} "
                   f"dino={rec['final_dino_cls']:.2f} geom={rec['final_geom_inliers']}</span>"
                   if rec["found"] else "<span class='no'>none</span>")),
    ]
    if rec["is_pos"]:
        ok = rec["center_in_gt"]
        head.append(("center_in_GT", f"<span class='{'ok' if ok else 'no'}'>{ok}</span> (IoU {rec['iou']:.2f})"))
    else:
        fp = rec["found"]
        head.append(("false_positive", f"<span class='{'no' if fp else 'ok'}'>{fp}</span>"))
    side = "".join(f"<div><span class='lbl'>{html.escape(k)}:</span> {v}</div>" for k, v in head)
    side += "<div style='margin-top:6px;color:#9aa3ad'>candidate funnel:</div>"
    side += "".join(render_cand(c) for c in rec["candidates"]) or "<div class='cand'>—</div>"
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
            cls = " class='k'" if key == "target" else ""
            tds.append(f"<td{cls}>{html.escape(str(val))}</td>")
        body.append("<tr>" + "".join(tds) + "</tr>")
    return f"<table class='sum'><tr>{head}</tr>{''.join(body)}</table>"


def main():
    ap = argparse.ArgumentParser(description="Build cascade_report.html from a cascade run.")
    ap.add_argument("--dir", default="OUTPUT/cascade", help="cascade output dir (under repo)")
    ap.add_argument("--max-width", type=int, default=640, help="embedded thumbnail max width (px)")
    args = ap.parse_args()

    out_dir = (REPO / args.dir).resolve()
    pf, sm = out_dir / "per_frame.json", out_dir / "summary.json"
    if not pf.exists() or not sm.exists():
        raise SystemExit(f"missing {pf.name}/{sm.name} in {out_dir} — run cascade_detector.py first.")
    records = json.loads(pf.read_text(encoding="utf-8"))
    summary = json.loads(sm.read_text(encoding="utf-8"))
    if not (out_dir / "overlays").exists():
        print(f"[report] WARNING: no overlays/ in {out_dir} — re-run cascade_detector.py WITHOUT "
              f"--no-overlays for images. Building a metadata-only report.")

    targets = sorted({r["target"] for r in records})
    sections = []
    for t in targets:
        for is_pos, label in ((True, "positives"), (False, "negatives (None)")):
            recs = [r for r in records if r["target"] == t and r["is_pos"] == is_pos]
            if recs:
                sections.append((f"{t} — {label}", recs))

    parts = ["<!doctype html><html><head><meta charset='utf-8'><title>Cascade detector</title>",
             f"<style>{CSS}{EXTRA_CSS}</style></head><body>",
             "<h1>Cascade detector — verified two/three-stage report</h1>",
             f"<div class='wrap'><div class='legend'>{LEGEND}</div>",
             render_summary_table(summary)]
    cache, n_missing = {}, 0
    for title, recs in sections:
        parts.append(f"<h2>{html.escape(title)}</h2><div class='grid'>")
        for r in recs:
            p = overlay_path(out_dir, r)
            if p not in cache:
                cache[p] = thumb_b64(p, args.max_width)
            if cache[p] is None:
                n_missing += 1
            parts.append(render_card(r, cache[p]))
        parts.append("</div>")
    parts.append("</div></body></html>")

    report = out_dir / "cascade_report.html"
    report.write_text("".join(parts), encoding="utf-8")
    print(f"[report] wrote {report}  ({len(records)} frames, {report.stat().st_size/1e6:.1f} MB"
          + (f", {n_missing} overlays missing)" if n_missing else ")"))


if __name__ == "__main__":
    main()
