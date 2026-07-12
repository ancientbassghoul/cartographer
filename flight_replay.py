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
  #events .ev { white-space: pre-wrap; }
  #events .cur { color: #fff; }
  #events .old { color: #777; }
  #events .plan { color: #c58bff; }          /* planner bump outcome (count / reset) */
  #events .plan.bl { color: #ff5b5b; font-weight: bold; }  /* the blacklist decision */
  #events .miss { color: #e0a020; }          /* a real contact that emitted no bump */
  #events .slam_start  { color: #e8891a; }    /* SLAM_ENGINE  [START]  frame ingested (orange) */
  #events .slam_finish { color: #33cc55; }    /* SLAM_TRACKER [FINISH] pose accepted + latency (green) */
  .legend { font-size: 11px; color: #999; padding: 6px 10px; border-bottom: 1px solid #333; }
  .sw { display: inline-block; width: 10px; height: 10px; margin: 0 3px 0 8px; vertical-align: middle; }
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
    `<span class="k">goal</span> ${s.goal?('['+fmt(s.goal[0])+', '+fmt(s.goal[1])+']'):'—'} ` +
    `<span class="k">d</span> ${fmt(s.dist_to_goal)}${planPick}<br>` +
    // 2-bump blacklist counter: how close the CURRENT goal region is to being retired (2 = blacklist).
    `<span class="k">bump</span> ${(s.wall_hit_count!=null?s.wall_hit_count:0)}/2` +
    ` ${s.wall_hit_goal?('@['+fmt(s.wall_hit_goal[0])+', '+fmt(s.wall_hit_goal[1])+']'):''}`;
  // event log up to the cursor: state events + the planner's bump outcomes (PLANNER) + un-counted
  // contacts (MISSED-BUMP), so the blacklist mechanism the flight log used to hide is now visible.
  const ev = document.getElementById('events');
  const tCur = STEPS[idx] ? STEPS[idx].t_mono : Infinity;
  const entries = [];
  for (let i = 0; i <= idx; i++) {
    const st = STEPS[i], cls = (i === idx) ? 'cur' : 'old', tw = st.t_wall || '';
    if (st.event) entries.push({t: st.t_mono, html: `<div class="ev ${cls}">${tw} ${st.state}: ${st.event}</div>`});
    if (st.planner_event) {
      const bl = /BLACKLIST/.test(st.planner_event) ? ' bl' : '';
      entries.push({t: st.t_mono, html: `<div class="ev ${cls} plan${bl}">${tw} PLANNER: ${st.planner_event}</div>`});
    }
    if (st.missed_bump) entries.push({t: st.t_mono, html: `<div class="ev ${cls} miss">${tw} MISSED-BUMP: ${st.missed_bump}</div>`});
  }
  // Interleave the paired SLAM logs up to the cursor time (orange START / green FINISH), so the ~2 Hz
  // pipeline spans sit between the state events chronologically instead of scrolling past in the terminal.
  for (const sv of SLAMEV) {
    if (sv.t_mono > tCur) break;
    // The paired-SLAM string carries its own bracketed wall-time, so t_wall is "" (no double timestamp).
    entries.push({t: sv.t_mono, html: `<div class="ev ${sv.ev_kind}">${sv.t_wall ? sv.t_wall + ' ' : ''}${sv.slam}</div>`});
  }
  entries.sort((a, b) => a.t - b.t);
  ev.innerHTML = entries.map(e => e.html).join('');
  ev.scrollTop = ev.scrollHeight;
}

// Per-frame RAW spatial telemetry: translation [X,Y,Z], yaw, the literal command dict sent to the sim,
// and step deltas (world displacement + distance closed to the goal) computed from consecutive frames.
const dist2 = (a, b) => (a && b) ? Math.hypot(a[0]-b[0], a[1]-b[1]) : null;
function updateTelemetry(idx) {
  const s = STEPS[idx];
  const prev = idx > 0 ? STEPS[idx-1] : null;
  const X = s.pos ? s.pos[0] : null, Z = s.pos ? s.pos[1] : null, Y = s.pos_y;
  // deltas (raw world step displacement + distance closed to the goal this step)
  const dpos = (prev && prev.pos && s.pos) ? dist2(s.pos, prev.pos) : null;
  const dg = s.goal ? dist2(s.pos, s.goal) : null;
  const dgPrev = (prev && prev.goal && prev.pos) ? dist2(prev.pos, prev.goal) : null;
  const dgClosed = (dg != null && dgPrev != null) ? (dgPrev - dg) : null;   // + = got closer
  // raw command dict -> key:value string ({} = hover; undefined = not recorded in this log)
  let cmdStr;
  if (s.cmd === undefined) cmdStr = '<span class="k">— (not recorded)</span>';
  else if (Object.keys(s.cmd).length === 0) cmdStr = '<span class="k">hover (neutral)</span>';
  else cmdStr = Object.entries(s.cmd)
        .map(([k, v]) => `<span class="k">${k}</span> <span class="cmd">${(typeof v === 'number') ? (+v).toFixed(2) : v}</span>`)
        .join('  ');
  const stCol = (s.status === 'OK' || s.status == null) ? 'v' : (s.status === 'PLAN-LOST' ? 'bad' : 'warn');
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
    `<div class="grp">DELTA (this step)</div>` +
    `<span class="k">&Delta;pos</span> <span class="v">${fmt(dpos,3)}</span>  ` +
    `<span class="k">&Delta;goal</span> <span class="v">${fmt(dgClosed,3)}</span>` +
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
         "cmd": {"trigger": 0.2}, "speed": 0.42, "nominal_speed": 0.45},
        {"t_wall": "", "t_mono": 1.5, "ev_kind": "slam_start", "frame_id": 6, "slam_ms": 700.0,
         "slam": "[00:00:01.100] SLAM had currently began working on this frame. (#6)"},
        {"t_wall": "", "t_mono": 2.2, "ev_kind": "slam_finish", "frame_id": 6, "slam_ms": 700.0,
         "slam": "[00:00:01.800]. SLAM had just finished working on the frame #6 from: [00:00:01.100]. "
                 "The deltas are: (dx: +0.10 dy: +0.03) Latency: 700ms."},
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
              and "DELTA (this step)" in html and 'id="telemetry"' in html
              and "DIST &rarr; GOAL (SLAM units)" in html and "SPEED (world, u/s)" in html)
        print(f"[self-test] {'PASS' if c7 else 'FAIL'}  raw telemetry panel (translation/cmd/dist/speed/delta) wired")
        ok = ok and c7

        # NEW: committed-goal vs plan-pick separation + staleness exposure survived + the render code is present
        c8 = (by_frame[5].get("plan_goal") == [2.5, 0.5] and by_frame[5].get("dist_to_goal") == 0.99
              and by_frame[5].get("plan_age_s") == 0.2 and by_frame[5].get("frame_id") == 5
              and any(g.get("state") == "plan_pick" for g in by_frame[5].get("goals", []))
              and "plan_pick" in html and "plan_age" in html and "STALE" in html and "plan_berr" in html)
        print(f"[self-test] {'PASS' if c8 else 'FAIL'}  committed-goal vs plan_pick + staleness (plan_age/frame_id/STALE) render")
        ok = ok and c8

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
