#!/usr/bin/env python3
"""flight_replay.py — turn a flight's structured timeline (OUTPUT/diag/<ts>_timeline.jsonl, written by
`autopilot.py run_explore --log`, F8 Part A) into a SELF-CONTAINED animated HTML replay.

The HTML (vanilla JS + Canvas, no libraries, no server) shows a top-down 2D scene — room bbox / occupancy
outline, the flight-path trail, the drone (dot + heading arrow), and the goals colored by their state at the
cursor time (active = gold, soft-blacklist = orange, permanent = red X) — over a scrubbable timeline with
play/pause, plus a side panel with the autopilot event log and a SLAM frame-time sparkline at the cursor.

Usage:
    python flight_replay.py OUTPUT/diag/<ts>_timeline.jsonl [-o out.html] [--open] [--slam-slow-ms 1000]
    python flight_replay.py --self-test

Stdlib only (json, argparse, os, webbrowser). The replay is a POST-HOC tool from a log — a different use
case from the live visualizer.py, so it is a new file, not an edit (it reuses the visualizer's top-down
color/marker conventions: goal gold, permanent-dead red X).
"""
import argparse
import json
import os
import sys
import webbrowser


def load_timeline(path):
    """Read a <ts>_timeline.jsonl into a list of records. Each line is one JSON object — either a per-step
    record (has 'state'/'pos'/'goals') or a periodic map record (has 'map'). Blank lines are skipped;
    a malformed line fails fast (NO SILENT FALLBACK) with its line number."""
    records = []
    with open(path, "r", encoding="utf-8") as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError as e:
                raise ValueError(f"{path}:{lineno}: malformed JSON in timeline: {e}") from e
    return records


def build_html(records, slam_slow_ms=1000.0, title="flight replay"):
    """Embed the timeline records into the self-contained HTML template and return it as a string."""
    data_json = json.dumps(records, separators=(",", ":"))
    return (_HTML_TEMPLATE
            .replace("__TITLE__", title)
            .replace("__SLAM_SLOW_MS__", repr(float(slam_slow_ms)))
            .replace("__DATA__", data_json))


def render_file(jsonl_path, out_path=None, slam_slow_ms=1000.0, open_browser=False):
    """Build the HTML for a timeline JSONL and write it (next to the log by default). Returns out_path."""
    if jsonl_path.lower().endswith(".html"):
        raise ValueError(f"{jsonl_path}: that's the VIEWER output - open it in a browser (start <file>). "
                         f"Pass the <ts>_timeline.jsonl to this script instead.")
    records = load_timeline(jsonl_path)
    if not records:
        raise ValueError(f"{jsonl_path}: no records — nothing to replay")
    if out_path is None:
        base = os.path.splitext(jsonl_path)[0]
        out_path = base + ".html"
    title = os.path.basename(jsonl_path)
    html = build_html(records, slam_slow_ms=slam_slow_ms, title=title)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(html)
    n_steps = sum(1 for r in records if "state" in r)
    n_maps = sum(1 for r in records if "map" in r)
    print(f"[flight_replay] {jsonl_path}: {len(records)} records ({n_steps} steps, {n_maps} maps) "
          f"-> {out_path}")
    if open_browser:
        webbrowser.open("file://" + os.path.abspath(out_path))
    return out_path


# ============================================================================
# The HTML template. __DATA__ is replaced with a JSON array of the timeline records; __SLAM_SLOW_MS__ with
# the green/red slam-ms threshold (a platform compute characteristic, ~1000 ms — NOT room geometry).
# ============================================================================
_HTML_TEMPLATE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>__TITLE__</title>
<style>
  html, body { margin: 0; height: 100%; background: #111; color: #ddd;
               font-family: Consolas, "Courier New", monospace; }
  #wrap { display: flex; height: 100vh; }
  #left { flex: 1 1 auto; display: flex; flex-direction: column; min-width: 0; }
  #scene { flex: 1 1 auto; background: #161616; }
  #bar { flex: 0 0 auto; padding: 8px 10px; background: #1c1c1c; border-top: 1px solid #333;
         display: flex; align-items: center; gap: 10px; }
  #bar input[type=range] { flex: 1 1 auto; }
  #bar button { background: #2a2a2a; color: #ddd; border: 1px solid #444; padding: 4px 10px;
                cursor: pointer; font-family: inherit; }
  #bar button:hover { background: #3a3a3a; }
  #clock { min-width: 210px; font-size: 12px; color: #9ad; }
  #right { flex: 0 0 340px; display: flex; flex-direction: column; background: #191919;
           border-left: 1px solid #333; }
  #hud { padding: 8px 10px; font-size: 12px; border-bottom: 1px solid #333; line-height: 1.5; }
  #hud .k { color: #888; }
  #telemetry { padding: 8px 10px; font-size: 12px; border-bottom: 1px solid #333; line-height: 1.6; }
  #telemetry .grp { color: #6cf; font-size: 10px; letter-spacing: 0.5px; margin-top: 4px; }
  #telemetry .grp:first-child { margin-top: 0; }
  #telemetry .k { color: #888; }
  #telemetry .v { color: #ddd; }
  #telemetry .cmd { color: #7fd97f; }
  #telemetry .warn { color: #e0a020; }
  #telemetry .bad { color: #ff5b5b; }
  #slamwrap { padding: 8px 10px; border-bottom: 1px solid #333; }
  #slamval { font-size: 12px; margin-bottom: 4px; }
  #events { flex: 1 1 auto; overflow-y: auto; padding: 6px 10px; font-size: 11px; line-height: 1.45; }
  #events .ev { white-space: pre-wrap; cursor: pointer; }   /* clickable -> jumps the scrubber to it */
  #events .ev:hover { background: #262626; }
  #events .ev.navsel { outline: 1px solid #567; }           /* the Prev/Next pointer's current position */
  #events .cur { color: #fff; }
  #events .old { color: #777; }
  #events .plan { color: #c58bff; }          /* planner bump outcome (count / reset) */
  #events .plan.bl { color: #ff5b5b; font-weight: bold; }  /* the blacklist decision */
  #events .miss { color: #e0a020; }          /* a real contact that emitted no bump */
  #events .slam_start  { color: #e8891a; }    /* SLAM_ENGINE  [START]  frame ingested (orange) */
  #events .slam_finish { color: #33cc55; }    /* SLAM_TRACKER [FINISH] pose accepted + latency (green) */
  #events .trigger_event { color: #33cccc; }  /* TRIGGER engaged/released/hop-end (cyan) */
  #events .hop_baseline  { color: #6fa8ff; }  /* HOP_BASELINE: hop-start position bind (light blue) */
  #events .hop_judge     { color: #e0c020; font-weight: bold; }  /* HOP_JUDGE: the verdict line (gold) */
  #events .slam_gap      { color: #ff2b2b; font-weight: bold; }  /* SLAM_GAP: a dropped plan (warning red) */
  .legend { font-size: 11px; color: #999; padding: 6px 10px; border-bottom: 1px solid #333; }
  .sw { display: inline-block; width: 10px; height: 10px; margin: 0 3px 0 8px; vertical-align: middle; }
  /* Floating goals-DB table (toggled by the Goals DB button; draggable by its header) */
  #goaldb { position: fixed; top: 64px; left: 20px; width: 360px; max-height: 66vh; display: none;
            flex-direction: column; background: #141414; border: 1px solid #555; border-radius: 4px;
            box-shadow: 0 6px 24px rgba(0,0,0,.6); z-index: 50; font-size: 11px; }
  #goaldb h4 { margin: 0; padding: 6px 10px; background: #242424; border-bottom: 1px solid #444; cursor: move;
               font-size: 12px; color: #9ad; display: flex; justify-content: space-between; align-items: center; }
  #goaldb h4 .x { cursor: pointer; color: #888; padding: 0 4px; }
  #goaldb h4 .x:hover { color: #fff; }
  #goaldb .body { overflow: auto; }
  #goaldb table { border-collapse: collapse; width: 100%; }
  #goaldb th, #goaldb td { padding: 3px 8px; text-align: right; border-bottom: 1px solid #262626; white-space: nowrap; }
  #goaldb th { position: sticky; top: 0; background: #1c1c1c; color: #888; font-weight: normal; text-align: right; }
  #goaldb td.c { text-align: left; color: #ccc; }
  #goaldb tr.goal td { border-top: 1px solid #333; font-weight: 600; }
  #goaldb tr.loc td { color: #8a9; font-weight: normal; border-bottom: 1px solid #1e1e1e; }
  #goaldb tr.loc td.c { color: #667; padding-left: 16px; }
  #goaldb tr.loc td[colspan] { text-align: left; }
  #goaldb tr.dead td { color: #ff5b5b; }
  #goaldb tr.loc.dead td { color: #b56; }
  #goaldb tr.cur td { background: #2a2a1a; }
  #goaldb tr.goal.cur td.c { color: #d9b400; }
  #goaldb .mp { color: #777; font-weight: normal; font-size: 10px; margin-left: 4px; }
  #goaldb .empty { padding: 10px; color: #777; }
  /* Floating clearance-detail table (session 29): the raw ray-hit picture (hits/rays/fraction/closest/
     farthest/blocked) behind the fwd/back/left/right clearance judgment -- mirrors #goaldb's shell. */
  #clrdet { position: fixed; top: 64px; left: 400px; width: 300px; max-height: 40vh; display: none;
            flex-direction: column; background: #141414; border: 1px solid #555; border-radius: 4px;
            box-shadow: 0 6px 24px rgba(0,0,0,.6); z-index: 50; font-size: 11px; }
  #clrdet h4 { margin: 0; padding: 6px 10px; background: #242424; border-bottom: 1px solid #444; cursor: move;
               font-size: 12px; color: #9ad; display: flex; justify-content: space-between; align-items: center; }
  #clrdet h4 .x { cursor: pointer; color: #888; padding: 0 4px; }
  #clrdet h4 .x:hover { color: #fff; }
  #clrdet .body { overflow: auto; }
  #clrdet table { border-collapse: collapse; width: 100%; }
  #clrdet th, #clrdet td { padding: 3px 8px; text-align: right; border-bottom: 1px solid #262626; white-space: nowrap; }
  #clrdet th { position: sticky; top: 0; background: #1c1c1c; color: #888; font-weight: normal; text-align: right; }
  #clrdet td.c { text-align: left; color: #ccc; }
  #clrdet tr.blocked td { color: #ff5b5b; }
  #clrdet .empty { padding: 10px; color: #777; }
  /* Floating visual-recovery panel (session 35 ALT): the 15° probe's phase + the last SIFT match verdict
     against F_LKG -- mirrors #clrdet's shell. */
  #visrec { position: fixed; top: 64px; left: 720px; width: 280px; max-height: 40vh; display: none;
            flex-direction: column; background: #141414; border: 1px solid #555; border-radius: 4px;
            box-shadow: 0 6px 24px rgba(0,0,0,.6); z-index: 50; font-size: 11px; }
  #visrec h4 { margin: 0; padding: 6px 10px; background: #242424; border-bottom: 1px solid #444; cursor: move;
               font-size: 12px; color: #9ad; display: flex; justify-content: space-between; align-items: center; }
  #visrec h4 .x { cursor: pointer; color: #888; padding: 0 4px; }
  #visrec h4 .x:hover { color: #fff; }
  #visrec .body { overflow: auto; padding: 8px 10px; line-height: 1.7; }
  #visrec .k { color: #888; }
  #visrec .v { color: #ddd; }
  #visrec .yes { color: #7fd97f; }
  #visrec .no { color: #ff5b5b; }
  #visrec .empty { padding: 10px; color: #777; }
</style>
</head>
<body>
<div id="wrap">
  <div id="left">
    <canvas id="scene"></canvas>
    <div id="bar">
      <button id="play">&#9654; Play</button>
      <input id="scrub" type="range" min="0" max="0" value="0" step="1">
      <label style="font-size:12px;color:#999">speed
        <select id="speed">
          <option value="0.5">0.5x</option>
          <option value="1" selected>1x</option>
          <option value="2">2x</option>
          <option value="4">4x</option>
          <option value="8">8x</option>
        </select>
      </label>
      <button id="dbBtn" title="Persistent goals database (picks / strikes / blacklist) at the cursor">Goals DB</button>
      <button id="clrBtn" title="Fwd/back/left/right clearance ray-hit detail at the cursor">Clearance</button>
      <button id="visrecBtn" title="Visual recovery probe phase + SIFT match verdict against F_LKG at the cursor">Visual Recovery</button>
      <button id="evPrev" title="Jump to the previous log message">&#9664; Msg</button>
      <button id="evNext" title="Jump to the next log message">Msg &#9654;</button>
      <label style="font-size:12px;color:#999">
        <input id="slamFilter" type="checkbox"> incl. SLAM msgs
      </label>
      <span id="clock"></span>
    </div>
  </div>
  <div id="right">
    <div class="legend">
      <span class="sw" style="background:#d9b400"></span>active goal
      <span class="sw" style="background:#e07b1a"></span>soft
      <span class="sw" style="background:#d33"></span>permanent
      <span class="sw" style="background:#3aa0ff"></span>drone
    </div>
    <div id="hud"></div>
    <div id="telemetry"></div>
    <div id="slamwrap">
      <div id="slamval"></div>
      <canvas id="slam" height="46"></canvas>
    </div>
    <div id="events"></div>
  </div>
</div>
<div id="goaldb">
  <h4><span>Goals DB <span id="dbcount" style="color:#777"></span></span><span class="x" id="dbClose">&times;</span></h4>
  <div class="body" id="dbBody"></div>
</div>
<div id="clrdet">
  <h4><span>Clearance</span><span class="x" id="clrClose">&times;</span></h4>
  <div class="body" id="clrBody"></div>
</div>
<div id="visrec">
  <h4><span>Visual Recovery</span><span class="x" id="visrecClose">&times;</span></h4>
  <div class="body" id="visrecBody"></div>
</div>
<script>
const RECORDS = __DATA__;
const SLAM_SLOW_MS = __SLAM_SLOW_MS__;

// Split records into step records (the timeline), map records (occupancy snapshots), and SLAM_TRACKER
// paired SLAM logs (ev_kind:"slam_start"/"slam_finish") — the latter interleave into the event log by t_mono
// as an orange START / green FINISH pair keyed on frame_id, so a latency spike (long START->FINISH span) and
// multi-thread overlap (overlapping spans across frames) read at a glance.
const STEPS  = RECORDS.filter(r => r.state !== undefined);
const MAPS   = RECORDS.filter(r => r.map !== undefined);
const SLAMEV = RECORDS.filter(r => r.ev_kind === 'slam_start' || r.ev_kind === 'slam_finish');  // chronological
// Diagnostic-session markers (TRIGGER engage/release/hop-end, a dropped-plan SLAM_GAP warning, and the
// HOP_BASELINE/HOP_JUDGE position-state instants) — same "not a STEPS record" shape as the SLAM pair
// above, carried on a `msg` field instead of `slam`. Kept as a separate list from SLAMEV (rather than
// widening that filter) so the name stays accurate and the two remain independently maintainable.
const DIAG_EVENTS = RECORDS.filter(r => ['trigger_event', 'slam_gap', 'hop_baseline', 'hop_judge'].includes(r.ev_kind));

// Every navigable/loggable event across the WHOLE flight (state events, planner bump/strike/loop outcomes,
// missed-bump markers, and the paired SLAM start/finish records), built ONCE at load — not per-render — and
// sorted by time. Each entry carries `stepIdx` (the nearest STEPS index at-or-before its time, i.e. where
// the scrubber jumps to) and `isSlam`, so the event log (render(), filtered up to the cursor), the
// clickable-message jump, and the Prev/Next navigation all read from this ONE list instead of three
// separate ad-hoc scans.
const ALL_EVENTS = (function () {
  const out = [];
  for (let i = 0; i < STEPS.length; i++) {
    const st = STEPS[i], tw = st.t_wall || '';
    if (st.event) out.push({t: st.t_mono, stepIdx: i, isSlam: false, cls: '',
                            html: `${tw} ${st.state}: ${st.event}`});
    if (st.planner_event) {
      const bl = /BLACKLIST/.test(st.planner_event) ? ' bl' : '';
      out.push({t: st.t_mono, stepIdx: i, isSlam: false, cls: 'plan' + bl,
                html: `${tw} PLANNER: ${st.planner_event}`});
    }
    if (st.missed_bump) out.push({t: st.t_mono, stepIdx: i, isSlam: false, cls: 'miss',
                                  html: `${tw} MISSED-BUMP: ${st.missed_bump}`});
  }
  // SLAM start/finish records aren't STEPS entries; map each to the nearest STEPS index at-or-before its
  // capture time (SLAMEV and STEPS are both already chronological, so a single forward pointer suffices).
  let si = 0;
  for (const sv of SLAMEV) {
    while (si + 1 < STEPS.length && STEPS[si + 1].t_mono <= sv.t_mono) si++;
    out.push({t: sv.t_mono, stepIdx: si, isSlam: true, cls: sv.ev_kind,
              html: `${sv.t_wall ? sv.t_wall + ' ' : ''}${sv.slam}`});
  }
  // Diagnostic markers (TRIGGER/SLAM_GAP/HOP_BASELINE/HOP_JUDGE) get their own forward pointer — a
  // separate array from SLAMEV, so it must walk STEPS independently rather than share `si`.
  let di = 0;
  for (const sv of DIAG_EVENTS) {
    while (di + 1 < STEPS.length && STEPS[di + 1].t_mono <= sv.t_mono) di++;
    out.push({t: sv.t_mono, stepIdx: di, isSlam: false, cls: sv.ev_kind,
              html: `${sv.t_wall ? sv.t_wall + ' ' : ''}${sv.msg}`});
  }
  out.sort((a, b) => a.t - b.t);
  return out;
})();

// ---- static world extent (fit once so scrubbing never jumps the view) ----
function computeExtent() {
  let x0 = Infinity, x1 = -Infinity, z0 = Infinity, z1 = -Infinity;
  const ext = (x, z) => { if (x < x0) x0 = x; if (x > x1) x1 = x;
                          if (z < z0) z0 = z; if (z > z1) z1 = z; };
  for (const m of MAPS) { const b = m.map.bounds; if (b) { ext(b[0], b[2]); ext(b[1], b[3]); } }
  for (const s of STEPS) {
    if (s.pos) ext(s.pos[0], s.pos[1]);
    if (s.goal) ext(s.goal[0], s.goal[1]);
    if (s.goals) for (const g of s.goals) if (g.xz) ext(g.xz[0], g.xz[1]);
  }
  if (!isFinite(x0)) { x0 = -1; x1 = 1; z0 = -1; z1 = 1; }
  const padX = Math.max(0.5, (x1 - x0) * 0.08), padZ = Math.max(0.5, (z1 - z0) * 0.08);
  return { x0: x0 - padX, x1: x1 + padX, z0: z0 - padZ, z1: z1 + padZ };
}
const EXT = computeExtent();

const scene = document.getElementById('scene');
const sctx = scene.getContext('2d');
const slam = document.getElementById('slam');
const slctx = slam.getContext('2d');

// World X-Z -> canvas px, equal aspect, +Z up. Recomputed on resize.
let VP = null;
function fit() {
  const r = scene.getBoundingClientRect();
  scene.width = r.width; scene.height = r.height;
  slam.width = document.getElementById('slamwrap').clientWidth - 20;
  const w = EXT.x1 - EXT.x0, h = EXT.z1 - EXT.z0;
  const s = Math.min(scene.width / w, scene.height / h) * 0.92;
  const ox = (scene.width - w * s) / 2, oy = (scene.height - h * s) / 2;
  VP = { s, ox, oy };
  render(cur);
}
function P(x, z) {
  return [VP.ox + (x - EXT.x0) * VP.s,
          scene.height - VP.oy - (z - EXT.z0) * VP.s];  // flip so +Z is up
}

const CLS_COL = { 1: '#2c2c2c', 2: '#5a5a66', 3: '#0d5a66' };  // free / occ / frontier (unknown skipped)
function drawMap(tMono) {
  // newest map at/under the cursor time
  let m = null;
  for (const r of MAPS) { if (r.t_mono <= tMono) m = r.map; else break; }
  if (!m || !m.bounds) return;
  const [bx0, bx1, bz0, bz1] = m.bounds, rows = m.rows, cols = m.cols, cls = m.cls;
  const cw = (bx1 - bx0) / cols, ch = (bz1 - bz0) / rows;
  for (let ri = 0; ri < rows; ri++) {
    for (let ci = 0; ci < cols; ci++) {
      const v = cls[ri * cols + ci];
      const col = CLS_COL[v];
      if (!col) continue;
      const wx = bx0 + ci * cw;            // cell world X (left edge)
      const wz = bz1 - ri * ch;            // row 0 = +Z up (summary flips rows)
      const p0 = P(wx, wz), p1 = P(wx + cw, wz - ch);
      sctx.fillStyle = col;
      sctx.fillRect(p0[0], p0[1], Math.max(1, p1[0] - p0[0]), Math.max(1, p1[1] - p0[1]));
    }
  }
}

// active = the goal the CONTROLLER is committed to (leg_goal); plan_pick = perception's async frontier
// pick when it differs (drawn faint/hollow so it never masquerades as the goal the drone is flying to).
const GOAL_COL = { active: '#d9b400', plan_pick: '#8a7d3a', blacklist_soft: '#e07b1a', blacklist_permanent: '#d33' };
const PLAN_LOST = { 'PLAN-LOST': 1, 'PLAN-STALE': 1, 'NO-PLAN': 1 };   // plan not usable -> grey the goal
function drawGoals(step) {
  const gs = step.goals || [];
  const lost = !!PLAN_LOST[step.status];              // no live plan this frame
  for (const g of gs) {
    if (!g.xz) continue;
    const [px, py] = P(g.xz[0], g.xz[1]);
    const col = lost ? '#888' : (GOAL_COL[g.state] || '#888');   // grey when the plan is lost
    if (!lost && g.state === 'blacklist_permanent') { // red X
      sctx.strokeStyle = col; sctx.lineWidth = 2;
      sctx.beginPath(); sctx.moveTo(px-6, py-6); sctx.lineTo(px+6, py+6);
      sctx.moveTo(px+6, py-6); sctx.lineTo(px-6, py+6); sctx.stroke();
    } else if (!lost && g.state === 'plan_pick') {    // faint hollow ring: perception's live pick
      sctx.strokeStyle = col; sctx.lineWidth = 1.5;
      sctx.setLineDash([3, 3]);
      sctx.beginPath(); sctx.arc(px, py, 5, 0, 2*Math.PI); sctx.stroke();
      sctx.setLineDash([]);
    } else {
      sctx.fillStyle = col;
      sctx.beginPath(); sctx.arc(px, py, 5, 0, 2*Math.PI); sctx.fill();
      if (g.state === 'active') {                     // ring the current committed goal
        sctx.strokeStyle = col; sctx.lineWidth = 2;
        sctx.beginPath(); sctx.arc(px, py, 9, 0, 2*Math.PI); sctx.stroke();
      }
    }
  }
}

function drawPath(idx) {
  sctx.strokeStyle = '#3aa0ff'; sctx.lineWidth = 1.5;
  sctx.beginPath();
  let started = false;
  for (let i = 0; i <= idx; i++) {
    const p = STEPS[i].pos;
    if (!p) { started = false; continue; }
    const [x, y] = P(p[0], p[1]);
    if (!started) { sctx.moveTo(x, y); started = true; } else { sctx.lineTo(x, y); }
  }
  sctx.stroke();
}

function drawDrone(step) {
  if (!step.pos) return;
  const [x, y] = P(step.pos[0], step.pos[1]);
  sctx.fillStyle = '#3aa0ff';
  sctx.beginPath(); sctx.arc(x, y, 5, 0, 2*Math.PI); sctx.fill();
  if (step.heading !== null && step.heading !== undefined) {
    // heading_deg = atan2(dx, dz): 0 = +Z, 90 = +X (perception convention)
    const h = step.heading * Math.PI / 180.0;
    const dx = Math.sin(h), dz = Math.cos(h), L = 22;
    sctx.strokeStyle = '#9fd0ff'; sctx.lineWidth = 2;
    sctx.beginPath(); sctx.moveTo(x, y);
    sctx.lineTo(x + dx * L, y - dz * L); sctx.stroke();
  }
}

function render(idx) {
  if (!VP) return;
  idx = Math.max(0, Math.min(STEPS.length - 1, idx));
  const step = STEPS[idx];
  sctx.clearRect(0, 0, scene.width, scene.height);
  drawMap(step.t_mono);
  drawPath(idx);
  drawGoals(step);
  drawDrone(step);
  updatePanel(idx);
  updateTelemetry(idx);
  drawSlam(idx);
  updateGoalDb(idx);
  updateClearanceDetail(idx);
  updateVisualRecovery(idx);
}

// Floating visual-recovery panel (session 35 ALT): the 15° probe's phase (TURN/MATCH/WAIT_RECOVER) +
// the last SIFT match verdict against F_LKG (matched/inliers/contained/planar_like/scale) -- rides each
// step record as `visual_recovery_detail`. None/undefined -> feature off or an older log.
function updateVisualRecovery(idx) {
  const panel = document.getElementById('visrec');
  if (panel.style.display !== 'flex') return;          // skip work while hidden
  const s = STEPS[idx] || {};
  const vr = s.visual_recovery_detail;
  const body = document.getElementById('visrecBody');
  if (!vr) { body.innerHTML = '<div class="empty">no visual_recovery_detail (feature off, or an older flight log)</div>'; return; }
  const yn = (b) => b === null || b === undefined ? '<span class="v">—</span>'
                   : `<span class="${b ? 'yes' : 'no'}">${b ? 'yes' : 'no'}</span>`;
  body.innerHTML =
    `<span class="k">phase</span> <span class="v">${vr.phase || '—'}</span><br>` +
    `<span class="k">cum. turn</span> <span class="v">${fmt(vr.cum_deg, 1)}&deg;</span><br>` +
    `<span class="k">F_LKG cached</span> ${yn(vr.has_lkg)}<br>` +
    `<span class="k">matched</span> ${yn(vr.matched)} <span class="k">inliers</span> <span class="v">${vr.inliers != null ? vr.inliers : '—'}</span><br>` +
    `<span class="k">contained</span> ${yn(vr.contained)}  <span class="k">planar-like</span> ${yn(vr.planar_like)}<br>` +
    `<span class="k">scale</span> <span class="v">${fmt(vr.scale, 2)}</span>`;
}

// Floating clearance-detail table (session 29): the raw ray-hit picture (hits / total rays / fraction /
// closest / farthest / judged-blocked) behind the fwd/back/left/right clearance calls TRIM/PARALLAX_PUSH/
// FALLBACK actually consult — rides each step record as `clearance_detail`. Undefined on an older log
// (pre this session) -> shows an explanatory empty state rather than a blank table.
function updateClearanceDetail(idx) {
  const panel = document.getElementById('clrdet');
  if (panel.style.display !== 'flex') return;          // skip work while hidden
  const s = STEPS[idx] || {};
  const cd = s.clearance_detail || null;
  const body = document.getElementById('clrBody');
  if (!cd) { body.innerHTML = '<div class="empty">no clearance_detail in this record (older flight log)</div>'; return; }
  const LABEL = { fwd: 'Forward', back: 'Backward', left: 'Left', right: 'Right' };
  let rows = '';
  for (const tag of ['fwd', 'back', 'left', 'right']) {
    const d = cd[tag];
    if (!d) continue;
    rows += `<tr class="${d.blocked ? 'blocked' : ''}"><td class="c">${LABEL[tag]}</td>` +
            `<td>${d.n_hits}/${d.n_rays}</td><td>${fmt(d.fraction, 2)}</td>` +
            `<td>${fmt(d.min_dist)}</td><td>${fmt(d.max_dist)}</td>` +
            `<td>${d.blocked ? 'BLOCKED' : 'open'}</td></tr>`;
  }
  body.innerHTML = '<table><thead><tr><th class="c">dir</th><th>hits</th><th>frac</th>' +
                   '<th>closest</th><th>farthest</th><th>judged</th></tr></thead><tbody>' +
                   rows + '</tbody></table>';
}

// Floating goals-DB table: the planner's persistent per-disc picks / strikes / bumps / corner-giveups /
// blacklist state at the cursor (rides each step record as `goal_db`, schema-split per mechanism so the
// operator can see WHICH one killed a goal and on what evidence, not just that it's dead). Updates live
// while scrubbing/playing. `bumps`/`corner_giveups`/`is_corner`/`blacklist_reason`/`blacklist_evidence`
// are undefined on an OLDER log (pre schema-split) — every read below is optional-chained/defaulted so
// those logs still render (just without the new columns' detail).
function updateGoalDb(idx) {
  const panel = document.getElementById('goaldb');
  if (panel.style.display !== 'flex') return;          // skip work while hidden
  const s = STEPS[idx] || {};
  const db = s.goal_db || null;
  const active = s.goal || null;
  const near = (c) => active && Math.abs(c[0]-active[0]) <= 0.5 && Math.abs(c[1]-active[1]) <= 0.5;
  document.getElementById('dbcount').textContent = db ? `(${db.length})` : '';
  const body = document.getElementById('dbBody');
  if (!db) { body.innerHTML = '<div class="empty">no goal_db in this record (older flight log)</div>'; return; }
  if (!db.length) { body.innerHTML = '<div class="empty">empty — no goal picked yet</div>'; return; }
  // max pairwise spread among a goal's drone-locations = the cluster DIAMETER the loop test compares to 1u
  // (loop fires only when this is <= goal_loop_pos_dist, i.e. ALL locs clustered); null if <2 locs.
  const maxSpread = (L) => { let m = null; for (let i=0;i<L.length;i++) for (let j=i+1;j<L.length;j++) {
    const d = Math.hypot(L[i][0]-L[j][0], L[i][1]-L[j][1]); if (m===null||d>m) m=d; } return m; };
  // A short inline rendering of the blacklist evidence dict — whatever the mechanism recorded (pos,
  // strikes/picks/spread/bumps, slam_ms) — so the reason a goal died is visible without opening a console.
  const evidenceTxt = (ev) => {
    if (!ev) return '';
    const parts = [];
    if (ev.pos) parts.push(`pos [${fmt(ev.pos[0])}, ${fmt(ev.pos[1])}]`);
    if (ev.slam_ms != null) parts.push(`slam ${fmt(ev.slam_ms, 0)}ms`);
    if (ev.spread != null) parts.push(`spread ${fmt(ev.spread)}u`);
    return parts.length ? ` (${parts.join(', ')})` : '';
  };
  let rows = '';
  for (const e of db) {
    const dead = e.blacklisted ? ' dead' : '', cur = near(e.center) ? ' cur' : '';
    const locs = e.drone_locs || [];
    const mp = maxSpread(locs), mpTxt = (mp===null) ? '' : ` spread ${fmt(mp)}u`;
    const cornerTag = e.is_corner ? ' <span class="mp">corner</span>' : '';
    const bumps = e.bumps != null ? e.bumps : '—', giveups = e.corner_giveups != null ? e.corner_giveups : '—';
    const status = e.blacklisted
      ? `BLACKLIST${e.blacklist_reason ? ' (' + e.blacklist_reason + ')' : ''}` +
        `<span class="mp">${evidenceTxt(e.blacklist_evidence)}</span>`
      : `active<span class="mp">${mpTxt}</span>`;
    // goal (parent) row: center(+corner tag) · picks · strikes · bumps · giveups · status(+reason/evidence)
    rows += `<tr class="goal${dead+cur}"><td class="c">[${fmt(e.center[0])}, ${fmt(e.center[1])}]${cornerTag}</td>` +
            `<td>${e.picks}</td><td>${e.strikes}</td><td>${bumps}</td><td>${giveups}</td>` +
            `<td>${status}</td></tr>`;
    // one sub-row per DRONE LOCATION at a pick (what the <1u clustering test runs on)
    for (let i=0;i<locs.length;i++) {
      rows += `<tr class="loc${dead}"><td class="c">&#8627; loc ${i+1}</td>` +
              `<td colspan="5">[${fmt(locs[i][0])}, ${fmt(locs[i][1])}]</td></tr>`;
    }
  }
  body.innerHTML = '<table><thead><tr><th class="c">goal center (x,z) / drone loc</th><th>picks</th>' +
                   '<th>strikes</th><th>bumps</th><th>giveups</th><th>status</th></tr></thead><tbody>' +
                   rows + '</tbody></table>';
}

const fmt = (v, d=2) => (v === null || v === undefined) ? '—' : (+v).toFixed(d);
function updatePanel(idx) {
  const s = STEPS[idx];
  document.getElementById('clock').textContent =
    `${s.t_wall || ''}  t=${fmt(s.t_mono, 2)}s  #${idx+1}/${STEPS.length}` +
    (s.rec_frame != null ? `  frame ${s.rec_frame}` : '');
  const hud = document.getElementById('hud');
  // pose/heading come from the SLAM plan (~2 Hz); when it's been held a while the pose is STALE (a real
  // turn looks motionless). Grey the held pose + show its age/frame so it never reads as a stuck drone.
  const stale = (s.plan_age_s != null && s.plan_age_s > 0.6);
  const po = stale ? ' style="opacity:.5"' : '';
  // goal fields: `goal` = the committed leg_goal (what "reached" is measured against); plan_goal = the
  // planner's async pick (shown only when it differs, so the mismatch is explicit, not a phantom change).
  const gsame = (s.goal && s.plan_goal && Math.abs(s.goal[0]-s.plan_goal[0])<1e-6 && Math.abs(s.goal[1]-s.plan_goal[1])<1e-6);
  const planPick = (s.plan_goal && !gsame)
    ? ` <span class="k">plan_pick</span> [${fmt(s.plan_goal[0])}, ${fmt(s.plan_goal[1])}]` : '';
  hud.innerHTML =
    `<span class="k">state</span> ${s.state}  <span class="k">status</span> ${s.status||'—'}<br>` +
    `<span${po}><span class="k">pos</span> ${s.pos?('['+fmt(s.pos[0])+', '+fmt(s.pos[1])+']'):'—'} ` +
    `<span class="k">hdg</span> ${fmt(s.heading,1)}&deg;</span>` +
    `  <span class="k">plan_age</span> ${fmt(s.plan_age_s,2)}s` +
    `${stale?' <span class="bad">STALE</span>':''} <span class="k">fid</span> ${s.frame_id!=null?s.frame_id:'—'}<br>` +
    `<span class="k">y</span> ${fmt(s.pos_y)} <span class="k">plan_berr</span> ${fmt(s.plan_bearing_err,1)}&deg; ` +
    `<span class="k">fwd_clear</span> ${fmt(s.fwd_clear)}<br>` +
    // The four push-relevant ring reads (null = open near-field = room) — what a parallax push decision saw.
    `<span class="k">ring</span> f ${fmt(s.ring_clear&&s.ring_clear.fwd)} b ${fmt(s.ring_clear&&s.ring_clear.back)} ` +
    `l ${fmt(s.ring_clear&&s.ring_clear.left)} r ${fmt(s.ring_clear&&s.ring_clear.right)}<br>` +
    `<span class="k">goal</span> ${s.goal?('['+fmt(s.goal[0])+', '+fmt(s.goal[1])+']'):'—'} ` +
    `<span class="k">d</span> ${fmt(s.dist_to_goal)}${planPick}<br>` +
    // 2-bump blacklist counter: how close the CURRENT goal region is to being retired (2 = blacklist).
    `<span class="k">bump</span> ${(s.wall_hit_count!=null?s.wall_hit_count:0)}/2` +
    ` ${s.wall_hit_goal?('@['+fmt(s.wall_hit_goal[0])+', '+fmt(s.wall_hit_goal[1])+']'):''}`;
  // event log up to the cursor, read from the single global ALL_EVENTS list (state events + the planner's
  // bump outcomes (PLANNER) + un-counted contacts (MISSED-BUMP) + the interleaved paired SLAM start/finish
  // records), so the blacklist mechanism the flight log used to hide is visible AND every entry is
  // clickable (data-gidx indexes back into ALL_EVENTS for the jump-to-message handler below).
  const ev = document.getElementById('events');
  const tCur = STEPS[idx] ? STEPS[idx].t_mono : Infinity;
  let evHtml = '';
  for (let gi = 0; gi < ALL_EVENTS.length; gi++) {
    const e = ALL_EVENTS[gi];
    if (e.t > tCur) break;   // ALL_EVENTS is sorted by time, so the rest are all still in the future
    const cls = (e.stepIdx === idx) ? 'cur' : 'old';
    const sel = (gi === navPos) ? ' navsel' : '';
    evHtml += `<div class="ev ${cls}${e.cls ? ' ' + e.cls : ''}${sel}" data-gidx="${gi}">${e.html}</div>`;
  }
  ev.innerHTML = evHtml;
  ev.scrollTop = ev.scrollHeight;
}

// Per-frame RAW spatial telemetry: translation [X,Y,Z], yaw, the literal command dict sent to the sim,
// and step deltas (world displacement + distance closed to the goal) computed from consecutive frames.
const dist2 = (a, b) => (a && b) ? Math.hypot(a[0]-b[0], a[1]-b[1]) : null;
function updateTelemetry(idx) {
  const s = STEPS[idx];
  const X = s.pos ? s.pos[0] : null, Z = s.pos ? s.pos[1] : null, Y = s.pos_y;
  const dg = s.goal ? dist2(s.pos, s.goal) : null;
  // raw command dict -> key:value string ({} = hover; undefined = not recorded in this log)
  let cmdStr;
  if (s.cmd === undefined) cmdStr = '<span class="k">— (not recorded)</span>';
  else if (Object.keys(s.cmd).length === 0) cmdStr = '<span class="k">hover (neutral)</span>';
  else cmdStr = Object.entries(s.cmd)
        .map(([k, v]) => `<span class="k">${k}</span> <span class="cmd">${(typeof v === 'number') ? (+v).toFixed(2) : v}</span>`)
        .join('  ');
  const stCol = (s.status === 'OK' || s.status == null) ? 'v' : (s.status === 'PLAN-LOST' ? 'bad' : 'warn');
  // TRIM band (session 22, bidirectional): LOW threshold = ceiling + 1.2*delta (sagged -> TRIM UP), HIGH
  // threshold = desired - 0.2*delta (glued near the ceiling -> TRIM DOWN). Ratios mirror the config defaults
  // (trim_sag_ratio 1.2 / trim_high_ratio 0.2). pos_y reddens when OUTSIDE the band on either side.
  const sagThr = (s.alt_ceiling != null && s.alt_delta != null) ? (s.alt_ceiling + 1.2 * s.alt_delta) : null;
  const sagHThr = (s.alt_desired != null && s.alt_delta != null) ? (s.alt_desired - 0.2 * s.alt_delta) : null;
  const sagBad = (s.pos_y != null && ((sagThr != null && s.pos_y > sagThr)
                                      || (sagHThr != null && s.pos_y < sagHThr)));
  const t = document.getElementById('telemetry');
  t.innerHTML =
    `<div class="grp">RAW TRANSLATION (world, +Y DOWN)</div>` +
    `<span class="k">X</span> <span class="v">${fmt(X,3)}</span>  ` +
    `<span class="k">Y</span> <span class="v">${fmt(Y,3)}</span>  ` +
    `<span class="k">Z</span> <span class="v">${fmt(Z,3)}</span>` +
    `<div class="grp">RAW ORIENTATION</div>` +
    `<span class="k">yaw</span> <span class="v">${fmt(s.heading,1)}&deg;</span>` +
    `<div class="grp">RAW COMMAND (sent to sim)</div>${cmdStr}` +
    `<div class="grp">DIST &rarr; GOAL (SLAM units)</div>` +
    `<span class="v" style="font-size:14px">${fmt(dg,3)}</span>` +
    `<div class="grp">SPEED (world, u/s)</div>` +
    `<span class="k">live</span> <span class="v">${fmt(s.speed,3)}</span>  ` +
    `<span class="k">nominal</span> <span class="v">${fmt(s.nominal_speed,3)}</span>  ` +
    `<span class="k">ram&lt;33%</span> <span class="${(s.speed!=null&&s.nominal_speed!=null&&s.speed<0.33*s.nominal_speed)?'bad':'v'}">` +
      `${(s.nominal_speed!=null)?(s.speed!=null?((s.speed<0.33*s.nominal_speed)?'STALLED':'ok'):'—'):'calibrating'}</span>` +
    `<div class="grp">HEIGHT CALIBRATION (+Y DOWN)</div>` +
    // Live pos_y vs the three per-calibration references (ceiling/desired/delta) + the TRIM sag threshold
    // (ceiling + 1.2*delta): pos_y beyond the threshold = the drone SANK enough that a TRIM should fire.
    // trim_on/calib_on flag the machinery actually running at this frame. Old logs (no fields) show —.
    `<span class="k">pos_y</span> <span class="${sagBad?'bad':'v'}">${fmt(s.pos_y,3)}</span>  ` +
    `<span class="k">ceiling</span> <span class="v">${fmt(s.alt_ceiling,3)}</span>  ` +
    `<span class="k">desired</span> <span class="v">${fmt(s.alt_desired,3)}</span>  ` +
    `<span class="k">delta</span> <span class="v">${fmt(s.alt_delta,3)}</span><br>` +
    `<span class="k">trim-at-high</span> <span class="v">${fmt(sagHThr,3)}</span>  ` +
    `<span class="k">trim-at-low</span> <span class="v">${fmt(sagThr,3)}</span>  ` +
    `<span class="k">median</span> <span class="v">${fmt(s.alt_median,3)}</span>  ` +
    `<span class="k">active</span> <span class="${(s.trim_on||s.calib_on)?'warn':'v'}">` +
      `${s.trim_on?'TRIM':(s.calib_on?'CALIB':'—')}</span>` +
    `<div class="grp">PLAN STATUS</div>` +
    `<span class="${stCol}">${s.status || 'OK'}</span>`;
}

function drawSlam(idx) {
  const s = STEPS[idx];
  const ms = s.slam_ms;
  const col = (ms == null) ? '#888' : (ms < SLAM_SLOW_MS ? '#3c3' : '#e33');
  document.getElementById('slamval').innerHTML =
    `<span style="color:#888">slam_ms</span> <span style="color:${col}">${fmt(ms,0)}</span>` +
    ` <span style="color:#555">(slow &ge; ${SLAM_SLOW_MS})</span>`;
  const W = slam.width, H = slam.height, N = 120;
  slctx.clearRect(0, 0, W, H);
  const lo = Math.max(0, idx - N + 1);
  const vals = [];
  for (let i = lo; i <= idx; i++) vals.push(STEPS[i].slam_ms);
  let mx = SLAM_SLOW_MS;
  for (const v of vals) if (v != null && v > mx) mx = v;
  // threshold line
  slctx.strokeStyle = '#444'; slctx.lineWidth = 1;
  const ty = H - (SLAM_SLOW_MS / mx) * (H - 2) - 1;
  slctx.beginPath(); slctx.moveTo(0, ty); slctx.lineTo(W, ty); slctx.stroke();
  slctx.lineWidth = 1.5; slctx.beginPath();
  let started = false;
  for (let i = 0; i < vals.length; i++) {
    const v = vals[i]; if (v == null) { started = false; continue; }
    const x = (vals.length <= 1) ? 0 : (i / (vals.length - 1)) * W;
    const y = H - (v / mx) * (H - 2) - 1;
    if (!started) { slctx.moveTo(x, y); started = true; } else { slctx.lineTo(x, y); }
  }
  slctx.strokeStyle = (s.slam_ms != null && s.slam_ms >= SLAM_SLOW_MS) ? '#e33' : '#3c3';
  slctx.stroke();
}

// ---- timeline controls ----
let cur = 0, playing = false, timer = null;
const scrub = document.getElementById('scrub');
scrub.max = Math.max(0, STEPS.length - 1);
scrub.addEventListener('input', () => { cur = +scrub.value; render(cur); });

function tick() {
  const sp = +document.getElementById('speed').value;
  cur += Math.max(1, Math.round(sp));
  if (cur >= STEPS.length - 1) { cur = STEPS.length - 1; setPlaying(false); }
  scrub.value = cur; render(cur);
}
function setPlaying(p) {
  playing = p;
  document.getElementById('play').innerHTML = p ? '&#10073;&#10073; Pause' : '&#9654; Play';
  if (timer) { clearInterval(timer); timer = null; }
  if (p) {
    if (cur >= STEPS.length - 1) { cur = 0; }
    timer = setInterval(tick, 100);
  }
}
document.getElementById('play').addEventListener('click', () => setPlaying(!playing));
window.addEventListener('resize', fit);

// ---- Event-log navigation: click a message to jump to its wall-clock time; Prev/Next step message-by-
// message (optionally skipping SLAM start/finish records via the checkbox), all reading from ALL_EVENTS. ----
let navPos = -1;   // pointer into ALL_EVENTS (the Prev/Next cursor); -1 = nothing selected yet
const slamFilter = document.getElementById('slamFilter');
function includeSlam() { return slamFilter.checked; }
function jumpToEvent(gi) {
  const e = ALL_EVENTS[gi];
  if (!e) return;
  navPos = gi;
  cur = e.stepIdx;
  scrub.value = cur;
  render(cur);
}
document.getElementById('events').addEventListener('click', (e) => {
  const div = e.target.closest('[data-gidx]');
  if (!div) return;
  jumpToEvent(+div.dataset.gidx);
});
function navStep(dir) {   // dir = +1 (next) or -1 (prev); scans past entries the SLAM filter excludes
  let i = navPos;
  for (;;) {
    i += dir;
    if (i < 0 || i >= ALL_EVENTS.length) return;   // nothing further in that direction
    if (includeSlam() || !ALL_EVENTS[i].isSlam) { jumpToEvent(i); return; }
  }
}
document.getElementById('evPrev').addEventListener('click', () => navStep(-1));
document.getElementById('evNext').addEventListener('click', () => navStep(1));
// Toggling the filter re-anchors the pointer to whichever (now-eligible) entry is closest to the CURRENT
// cursor position, so Prev/Next never jumps wildly to wherever navPos happened to sit before the toggle.
slamFilter.addEventListener('change', () => {
  const inc = includeSlam();
  let best = -1, bestDist = Infinity;
  for (let i = 0; i < ALL_EVENTS.length; i++) {
    if (!inc && ALL_EVENTS[i].isSlam) continue;
    const d = Math.abs(ALL_EVENTS[i].stepIdx - cur);
    if (d < bestDist) { bestDist = d; best = i; }
  }
  navPos = best;
  render(cur);   // refresh the .navsel highlight in the event log
});

// ---- Goals-DB floating panel: toggle + drag by its header ----
const dbPanel = document.getElementById('goaldb');
function toggleDb() {
  dbPanel.style.display = (dbPanel.style.display === 'flex') ? 'none' : 'flex';
  updateGoalDb(cur);
}
document.getElementById('dbBtn').addEventListener('click', toggleDb);
document.getElementById('dbClose').addEventListener('click', () => { dbPanel.style.display = 'none'; });
(function () {
  const h = dbPanel.querySelector('h4');
  let dx = 0, dy = 0, drag = false;
  h.addEventListener('mousedown', (e) => {
    if (e.target.classList.contains('x')) return;
    drag = true; const r = dbPanel.getBoundingClientRect();
    dx = e.clientX - r.left; dy = e.clientY - r.top;
    dbPanel.style.right = 'auto'; e.preventDefault();
  });
  window.addEventListener('mousemove', (e) => {
    if (!drag) return;
    dbPanel.style.left = Math.max(0, e.clientX - dx) + 'px';
    dbPanel.style.top = Math.max(0, e.clientY - dy) + 'px';
  });
  window.addEventListener('mouseup', () => { drag = false; });
})();

// ---- Clearance-detail floating panel (session 29): toggle + drag by its header ----
const clrPanel = document.getElementById('clrdet');
function toggleClr() {
  clrPanel.style.display = (clrPanel.style.display === 'flex') ? 'none' : 'flex';
  updateClearanceDetail(cur);
}
document.getElementById('clrBtn').addEventListener('click', toggleClr);
document.getElementById('clrClose').addEventListener('click', () => { clrPanel.style.display = 'none'; });
(function () {
  const h = clrPanel.querySelector('h4');
  let dx = 0, dy = 0, drag = false;
  h.addEventListener('mousedown', (e) => {
    if (e.target.classList.contains('x')) return;
    drag = true; const r = clrPanel.getBoundingClientRect();
    dx = e.clientX - r.left; dy = e.clientY - r.top;
    clrPanel.style.right = 'auto'; e.preventDefault();
  });
  window.addEventListener('mousemove', (e) => {
    if (!drag) return;
    clrPanel.style.left = Math.max(0, e.clientX - dx) + 'px';
    clrPanel.style.top = Math.max(0, e.clientY - dy) + 'px';
  });
  window.addEventListener('mouseup', () => { drag = false; });
})();

// ---- Visual-recovery floating panel (session 35 ALT): toggle + drag by its header ----
const visrecPanel = document.getElementById('visrec');
function toggleVisrec() {
  visrecPanel.style.display = (visrecPanel.style.display === 'flex') ? 'none' : 'flex';
  updateVisualRecovery(cur);
}
document.getElementById('visrecBtn').addEventListener('click', toggleVisrec);
document.getElementById('visrecClose').addEventListener('click', () => { visrecPanel.style.display = 'none'; });
(function () {
  const h = visrecPanel.querySelector('h4');
  let dx = 0, dy = 0, drag = false;
  h.addEventListener('mousedown', (e) => {
    if (e.target.classList.contains('x')) return;
    drag = true; const r = visrecPanel.getBoundingClientRect();
    dx = e.clientX - r.left; dy = e.clientY - r.top;
    visrecPanel.style.right = 'auto'; e.preventDefault();
  });
  window.addEventListener('mousemove', (e) => {
    if (!drag) return;
    visrecPanel.style.left = Math.max(0, e.clientX - dx) + 'px';
    visrecPanel.style.top = Math.max(0, e.clientY - dy) + 'px';
  });
  window.addEventListener('mouseup', () => { drag = false; });
})();

if (STEPS.length === 0) {
  document.getElementById('hud').textContent = 'No step records in this timeline.';
} else {
  fit();
}
</script>
</body>
</html>
"""


# ============================================================================
def _self_test():
    """Offline smoke test: synthesize a tiny timeline (pose + a goal that flips active->soft->permanent,
    a slam_ms spike, a map record, an event line), build the HTML, and assert the embedded record count
    matches and the key scene pieces are present. No hardware."""
    import tempfile
    recs = [
        {"t_mono": 0.0, "map": {"bounds": [-2.0, 2.0, -2.0, 2.0], "rows": 2, "cols": 2,
                                "cls": [1, 1, 2, 3]}},
        {"t_wall": "00:00:00.000", "t_mono": 0.0, "rec_frame": 0, "state": "REPLAN", "event": None,
         "status": "OK", "pos": [0.0, 0.0], "heading": 0.0, "pos_y": -1.0, "slam_ms": 400.0,
         "fwd_clear": 1.5, "goal": [1.0, 1.0], "bearing_err": 5.0,
         "goals": [{"xz": [1.0, 1.0], "state": "active"}]},
        {"t_wall": "00:00:01.000", "t_mono": 1.0, "rec_frame": 5, "state": "ADVANCE",
         "event": "leg start", "status": "OK", "pos": [0.3, 0.3], "heading": 45.0, "pos_y": -1.0,
         "slam_ms": 480.0, "fwd_clear": 1.0, "goal": [1.0, 1.0], "plan_bearing_err": 2.0,
         "plan_goal": [2.5, 0.5], "dist_to_goal": 0.99, "plan_age_s": 0.2, "frame_id": 5,
         "goals": [{"xz": [1.0, 1.0], "state": "active"}, {"xz": [2.5, 0.5], "state": "plan_pick"}],
         "cmd": {"trigger": 0.2}, "speed": 0.42, "nominal_speed": 0.45,
         "alt_median": -1.85, "alt_ceiling": -2.2, "alt_desired": -1.9, "alt_delta": 0.3,
         "trim_on": False, "calib_on": False,
         "goal_db": [{"center": [1.0, 1.0], "picks": 2, "strikes": 1, "bumps": 1, "corner_giveups": 0,
                      "is_corner": False, "drone_locs": [[0.30, 0.30], [0.34, 0.28]],
                      "blacklisted": False, "blacklist_reason": None, "blacklist_evidence": {}},
                     {"center": [2.5, 0.5], "picks": 1, "strikes": 0, "bumps": 2, "corner_giveups": 3,
                      "is_corner": True, "drone_locs": [[0.30, 0.30]], "blacklisted": True,
                      "blacklist_reason": "2bump", "blacklist_evidence": {"pos": [2.4, 0.6]}}],
         "clearance_detail": {
             "fwd": {"dist": None, "n_hits": 1, "n_rays": 10, "fraction": 0.1, "min_dist": 2.5,
                     "max_dist": 2.5, "blocked": False},
             "back": {"dist": 0.8, "n_hits": 4, "n_rays": 10, "fraction": 0.4, "min_dist": 0.8,
                      "max_dist": 1.2, "blocked": True},
             "left": {"dist": None, "n_hits": 0, "n_rays": 10, "fraction": 0.0, "min_dist": None,
                      "max_dist": None, "blocked": False},
             "right": {"dist": 1.1, "n_hits": 5, "n_rays": 10, "fraction": 0.5, "min_dist": 1.1,
                       "max_dist": 1.4, "blocked": True}},
         "visual_recovery_detail": {"phase": "MATCH", "cum_deg": 15.0, "has_lkg": True, "matched": True,
                                    "inliers": 34, "contained": False, "planar_like": True, "scale": 1.02}},
        {"t_wall": "", "t_mono": 1.5, "ev_kind": "slam_start", "frame_id": 6, "slam_ms": 700.0,
         "slam": "[00:00:01.100] SLAM had currently began working on this frame. (#6)"},
        {"t_wall": "", "t_mono": 2.2, "ev_kind": "slam_finish", "frame_id": 6, "slam_ms": 700.0,
         "slam": "[00:00:01.800]. SLAM had just finished working on the frame #6 from: [00:00:01.100]. "
                 "The deltas are: (dx: +0.10 dy: +0.03) Latency: 700ms."},
        {"t_wall": "00:00:00.400", "t_mono": 0.4, "ev_kind": "trigger_event", "msg": "[TRIGGER] engaged"},
        {"t_wall": "00:00:00.900", "t_mono": 0.9, "ev_kind": "hop_baseline",
         "msg": "[HOP_BASELINE] pos=[0.30, 0.30] cap_ts=0.90 frame_id=5 bound against goal=[1.0, 1.0] dist=0.99"},
        {"t_wall": "00:00:01.700", "t_mono": 1.7, "ev_kind": "slam_gap",
         "msg": "[SLAM_GAP] slam_seq jumped 6 -> 8 (2 dropped plans)"},
        {"t_wall": "00:00:02.500", "t_mono": 2.5, "ev_kind": "hop_judge",
         "msg": "[HOP_JUDGE] pos=[0.60, 0.60] cap_ts=2.50 frame_id=8 prev_goal=[1.0, 1.0] start_dist=0.99 "
                "end_dist=0.50 closed=0.49 progressed=True"},
        {"t_wall": "00:00:02.000", "t_mono": 2.0, "rec_frame": 10, "state": "SETTLE",
         "event": "SLAM spike", "status": "PLAN-STALE", "pos": [0.6, 0.6], "heading": 45.0,
         "pos_y": -1.0, "slam_ms": 2200.0, "fwd_clear": 0.5, "goal": [1.0, 1.0],
         "plan_bearing_err": 2.0, "plan_age_s": 1.5, "frame_id": 8,
         "goals": [{"xz": [1.0, 1.0], "state": "blacklist_soft"}],
         "wall_hit_count": 1, "wall_hit_goal": [1.0, 1.0],
         "missed_bump": "flow WALL contact (latch disarmed — drone hasn't disengaged since the last bump)"},
        {"t_wall": "00:00:03.000", "t_mono": 3.0, "rec_frame": 15, "state": "REPLAN",
         "event": "goal UNREACHABLE -> permanent", "status": "OK", "pos": [0.6, 0.6], "heading": 45.0,
         "pos_y": -1.0, "slam_ms": 500.0, "fwd_clear": 0.5, "goal": None,
         "bearing_err": None, "goals": [{"xz": [1.0, 1.0], "state": "blacklist_permanent"}],
         "wall_hit_count": 0, "wall_hit_goal": None,
         "planner_event": "BUMP goal=[1.0, 1.0] count=2/2 -> BLACKLIST PERMANENT (1 total) -> reselecting"},
    ]
    ok = True
    with tempfile.TemporaryDirectory() as d:
        jl = os.path.join(d, "t_timeline.jsonl")
        with open(jl, "w", encoding="utf-8") as f:
            for r in recs:
                f.write(json.dumps(r) + "\n")
        loaded = load_timeline(jl)
        c1 = (len(loaded) == len(recs))
        print(f"[self-test] {'PASS' if c1 else 'FAIL'}  load_timeline round-trips {len(recs)} records "
              f"(got {len(loaded)})")
        ok = ok and c1

        out = os.path.join(d, "t.html")
        render_file(jl, out_path=out)
        html = open(out, "r", encoding="utf-8").read()
        # every record's JSON must be embedded (record count preserved through the template)
        embedded = json.loads(html.split("const RECORDS = ", 1)[1].split(";\n", 1)[0])
        c2 = (len(embedded) == len(recs))
        print(f"[self-test] {'PASS' if c2 else 'FAIL'}  embedded record count matches ({len(embedded)})")
        ok = ok and c2

        n_steps = sum(1 for r in embedded if "state" in r)
        n_maps = sum(1 for r in embedded if "map" in r)
        c3 = (n_steps == 4 and n_maps == 1)
        print(f"[self-test] {'PASS' if c3 else 'FAIL'}  split: {n_steps} steps + {n_maps} map "
              f"(expected 4 + 1)")
        ok = ok and c3

        # the goal state transition (active -> soft -> permanent) survived into the embedded data
        states = [r["goals"][0]["state"] for r in embedded if r.get("goals")]
        c4 = states == ["active", "active", "blacklist_soft", "blacklist_permanent"]
        print(f"[self-test] {'PASS' if c4 else 'FAIL'}  goal state timeline {states}")
        ok = ok and c4

        # the slam spike + the canvas scene scaffolding are present in the HTML
        c5 = ("2200" in html and "drawSlam" in html and "drawGoals" in html and "<canvas id=\"scene\"" in html)
        print(f"[self-test] {'PASS' if c5 else 'FAIL'}  HTML carries slam spike + scene/goal/slam draw code")
        ok = ok and c5

        # goals-DB floating table: the goal_db field survived load + the render/toggle code is wired in
        db_rec = next((r for r in embedded if r.get("goal_db")), None)
        c_db = (db_rec is not None and db_rec["goal_db"][0]["strikes"] == 1
                and db_rec["goal_db"][0]["drone_locs"][0] == [0.30, 0.30]
                and "updateGoalDb" in html and 'id="goaldb"' in html and 'id="dbBtn"' in html
                and 'loc ${i+1}' in html and 'e.drone_locs' in html)   # per-loc coordinate sub-rows render
        print(f"[self-test] {'PASS' if c_db else 'FAIL'}  goals-DB table (drone_locs survive + loc-rows render)")
        ok = ok and c_db

        # goals-DB schema split: bumps/corner_giveups/is_corner/blacklist_reason/evidence survive load +
        # the new columns' render code is wired in (corner tag, reason-in-status, evidence text).
        c_db2 = (db_rec["goal_db"][1]["bumps"] == 2 and db_rec["goal_db"][1]["corner_giveups"] == 3
                 and db_rec["goal_db"][1]["is_corner"] is True
                 and db_rec["goal_db"][1]["blacklist_reason"] == "2bump"
                 and db_rec["goal_db"][1]["blacklist_evidence"]["pos"] == [2.4, 0.6]
                 and "e.corner_giveups" in html and "e.blacklist_reason" in html
                 and "evidenceTxt" in html and "corner</span>" in html)
        print(f"[self-test] {'PASS' if c_db2 else 'FAIL'}  goals-DB schema split "
              f"(bumps/corner_giveups/is_corner/reason/evidence survive + render code wired)")
        ok = ok and c_db2

        # Clearance-detail floating table (session 29): fwd/back/left/right ray-hit stats survive load +
        # the render/toggle code is wired in.
        cd_rec = next((r for r in embedded if r.get("clearance_detail")), None)
        c_clr = (cd_rec is not None and cd_rec["clearance_detail"]["back"]["n_hits"] == 4
                 and cd_rec["clearance_detail"]["back"]["blocked"] is True
                 and cd_rec["clearance_detail"]["left"]["n_hits"] == 0
                 and "updateClearanceDetail" in html and 'id="clrdet"' in html and 'id="clrBtn"' in html
                 and "d.min_dist" in html and "d.max_dist" in html and "d.fraction" in html)
        print(f"[self-test] {'PASS' if c_clr else 'FAIL'}  clearance-detail table "
              f"(fwd/back/left/right ray stats survive + render code wired)")
        ok = ok and c_clr

        # Visual-recovery floating panel (session 35 ALT): phase + SIFT match verdict survive load + the
        # render/toggle code is wired in.
        vr_rec = next((r for r in embedded if r.get("visual_recovery_detail")), None)
        c_visrec = (vr_rec is not None and vr_rec["visual_recovery_detail"]["phase"] == "MATCH"
                    and vr_rec["visual_recovery_detail"]["planar_like"] is True
                    and vr_rec["visual_recovery_detail"]["scale"] == 1.02
                    and "updateVisualRecovery" in html and 'id="visrec"' in html and 'id="visrecBtn"' in html
                    and "vr.matched" in html and "vr.scale" in html and "F_LKG cached" in html)
        print(f"[self-test] {'PASS' if c_visrec else 'FAIL'}  visual-recovery panel "
              f"(phase/match verdict survive + render code wired)")
        ok = ok and c_visrec

        # Paired SLAM logs (ev_kind:"slam_start"/"slam_finish") carried + the orange/green interleave render path
        n_slam = sum(1 for r in embedded if r.get("ev_kind") in ("slam_start", "slam_finish"))
        c_slam = (n_slam == 2 and "SLAM had currently began working" in html
                  and "SLAM had just finished working" in html and "SLAMEV" in html
                  and "#events .slam_start" in html and "#events .slam_finish" in html)
        print(f"[self-test] {'PASS' if c_slam else 'FAIL'}  paired SLAM START/FINISH records carried ({n_slam}) + orange/green event-log render path present")
        ok = ok and c_slam

        # the new blacklist-observability fields survived + the render code that surfaces them is present
        by_frame = {r.get("rec_frame"): r for r in embedded if "state" in r}
        c6 = (by_frame[10].get("wall_hit_count") == 1 and by_frame[10].get("missed_bump")
              and "BLACKLIST" in (by_frame[15].get("planner_event") or "")
              and "PLANNER:" in html and "MISSED-BUMP:" in html and "bump</span>" in html)
        print(f"[self-test] {'PASS' if c6 else 'FAIL'}  bump counter + PLANNER/MISSED-BUMP fields render")
        ok = ok and c6

        # the raw per-frame telemetry panel (translation/yaw/command/delta/status) is wired in
        c7 = (by_frame[5].get("cmd") == {"trigger": 0.2}
              and by_frame[5].get("speed") == 0.42 and by_frame[5].get("nominal_speed") == 0.45
              and "updateTelemetry" in html and "RAW TRANSLATION" in html and "RAW COMMAND" in html
              and "HEIGHT CALIBRATION (+Y DOWN)" in html and 'id="telemetry"' in html
              # session 21/22: the ceiling/desired/delta references survive load + BOTH trim thresholds render
              and by_frame[5].get("alt_ceiling") == -2.2 and by_frame[5].get("alt_delta") == 0.3
              and "alt_ceiling" in html and "alt_desired" in html and "alt_delta" in html
              and "trim-at-low" in html and "trim-at-high" in html and "sagThr" in html and "sagHThr" in html
              and "DIST &rarr; GOAL (SLAM units)" in html and "SPEED (world, u/s)" in html)
        print(f"[self-test] {'PASS' if c7 else 'FAIL'}  raw telemetry panel (translation/cmd/dist/speed/height-calib) wired")
        ok = ok and c7

        # NEW: committed-goal vs plan-pick separation + staleness exposure survived + the render code is present
        c8 = (by_frame[5].get("plan_goal") == [2.5, 0.5] and by_frame[5].get("dist_to_goal") == 0.99
              and by_frame[5].get("plan_age_s") == 0.2 and by_frame[5].get("frame_id") == 5
              and any(g.get("state") == "plan_pick" for g in by_frame[5].get("goals", []))
              and "plan_pick" in html and "plan_age" in html and "STALE" in html and "plan_berr" in html)
        print(f"[self-test] {'PASS' if c8 else 'FAIL'}  committed-goal vs plan_pick + staleness (plan_age/frame_id/STALE) render")
        ok = ok and c8

        # NEW: event-log navigation — the global ALL_EVENTS list, clickable-message wiring (data-gidx +
        # delegated click handler), Prev/Next buttons, and the SLAM-filter checkbox are all present.
        c9 = ("const ALL_EVENTS" in html and "stepIdx" in html and "isSlam" in html
              and "data-gidx" in html and "jumpToEvent" in html and "function navStep" in html
              and 'id="evPrev"' in html and 'id="evNext"' in html and 'id="slamFilter"' in html
              and "navPos" in html)
        print(f"[self-test] {'PASS' if c9 else 'FAIL'}  event-log navigation (ALL_EVENTS + click-to-jump + "
              f"Prev/Next + SLAM-filter checkbox wired)")
        ok = ok and c9

        # NEW: diagnostic-session markers (trigger_event/hop_baseline/slam_gap/hop_judge) carried through
        # load, and DIAG_EVENTS + the per-kind CSS classes are wired into the generated HTML.
        n_diag = sum(1 for r in embedded if r.get("ev_kind") in
                     ("trigger_event", "hop_baseline", "slam_gap", "hop_judge"))
        c10 = (n_diag == 4 and "[TRIGGER] engaged" in html and "[HOP_BASELINE]" in html
               and "[SLAM_GAP]" in html and "[HOP_JUDGE]" in html and "const DIAG_EVENTS" in html
               and "#events .trigger_event" in html and "#events .hop_baseline" in html
               and "#events .slam_gap" in html and "#events .hop_judge" in html)
        print(f"[self-test] {'PASS' if c10 else 'FAIL'}  diagnostic markers (TRIGGER/HOP_BASELINE/SLAM_GAP/"
              f"HOP_JUDGE) carried ({n_diag}) + DIAG_EVENTS/CSS wired")
        ok = ok and c10

    print(f"[self-test] {'ALL PASS' if ok else 'FAILURES'} (flight_replay)")
    return ok


def main():
    ap = argparse.ArgumentParser(description="Animated HTML replay of a flight timeline JSONL.")
    ap.add_argument("jsonl", nargs="?", help="OUTPUT/diag/<ts>_timeline.jsonl")
    ap.add_argument("-o", "--out", help="output .html path (default: next to the JSONL)")
    ap.add_argument("--open", action="store_true", dest="open_browser", help="open the HTML in a browser")
    ap.add_argument("--slam-slow-ms", type=float, default=1000.0,
                    help="green/red slam_ms threshold in the sparkline (platform compute characteristic)")
    ap.add_argument("--self-test", action="store_true", help="run the offline smoke test and exit")
    args = ap.parse_args()

    if args.self_test:
        sys.exit(0 if _self_test() else 1)
    if not args.jsonl:
        ap.error("a timeline JSONL path is required (or use --self-test)")
    render_file(args.jsonl, out_path=args.out, slam_slow_ms=args.slam_slow_ms,
                open_browser=args.open_browser)


if __name__ == "__main__":
    main()
