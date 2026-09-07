"""Session 68 -- what the GPU is actually doing during a flight, and WHO is using it.

Session 67 measured inside `tracker.track()` and eliminated the solver: GN iteration counts are
flat all flight while two unrelated GPU workloads -- a ViT-Large forward pass and a 7-DOF
Gauss-Newton step -- slow by the SAME 5-10x factor, in lockstep, and recover together. CPU work
(`frame_ms`) is unaffected. That is not an accumulator inside SLAM; the GPU is delivering a
fraction of its throughput and SLAM is merely the thing measuring it.

Nothing in this stack logs the GPU, so three candidates remain indistinguishable:

  (a) CONTENTION      -- Xlab.exe (the Unity sim) taking a larger share as it renders more
  (b) VRAM EXHAUSTION -- dedicated VRAM full, allocations spilling to system RAM over PCIe
  (c) THROTTLING      -- thermal or power capping cutting the clocks

This probe settles all three from OUTSIDE the flight. It imports nothing from the project, touches
no flight code, and does no GPU work of its own -- so it cannot perturb what it measures.

  (a) per-process GPU engine utilisation, split 3D (Unity's renderer) vs Compute (our CUDA)
  (b) `vram_nonlocal_*` -- non-local usage IS the spill-to-system-RAM indicator; > 0 means paging
  (c) `throttle_names` -- nvidia-smi's own reason bits, plus sm_clock against sm_clock_max

`nvidia-smi --query-compute-apps` reports per-process memory as [N/A] on Windows (a WDDM
limitation, not a driver bug), so per-process numbers come from Windows' own performance counters
-- the same source Task Manager's GPU column reads.

Rows carry `wall_ts` from `time.time()`, the same clock `perception_worker` stamps its CSV with, so
a probe run joins to a flight by time with no offset arithmetic. `--report` does that join and puts
GPU state next to `trk_pre_ms`, session 67's fixed-work speedometer.

Usage:
    venv\\Scripts\\python.exe gpu_probe.py                 # log until Ctrl-C
    venv\\Scripts\\python.exe gpu_probe.py --report        # newest gpu csv + newest flight
    venv\\Scripts\\python.exe gpu_probe.py --self-test
"""

import argparse
import csv
import glob
import json
import os
import statistics
import subprocess
import sys
import tempfile
import time
from datetime import datetime

REPO = os.path.dirname(os.path.abspath(__file__))
DIAG = os.path.join(REPO, "OUTPUT", "diag")

# nvidia-smi's clock-throttle reason bits. The whole point of branch (c): a set bit here is the GPU
# telling us in its own words that it capped itself, which no timing measurement can ever prove.
THROTTLE_BITS = (
    (0x0001, "GpuIdle"),
    (0x0002, "AppClocksSetting"),
    (0x0004, "SwPowerCap"),
    (0x0008, "HwSlowdown"),
    (0x0010, "SyncBoost"),
    (0x0020, "SwThermalSlowdown"),
    (0x0040, "HwThermalSlowdown"),
    (0x0080, "HwPowerBrakeSlowdown"),
    (0x0100, "DisplayClockSetting"),
)

# Queried in this order, parsed positionally. `noheader,nounits` keeps the parse trivial.
_SMI_FIELDS = (
    "utilization.gpu", "memory.used", "memory.total", "memory.reserved",
    "clocks.sm", "clocks.max.sm", "clocks.mem",
    "temperature.gpu", "power.draw", "clocks_throttle_reasons.active",
)

# One PowerShell pass over the three per-process counter sets. Written to a temp .ps1 once and
# invoked with -File per sample: -Command quoting through cmd is a minefield, -File is not.
# Get-Counter re-enumerates instances on every call, so processes that start mid-flight (all five
# of ours, launched by fly.py) are picked up without restarting the probe.
_PS_SCRIPT = r"""
$ErrorActionPreference = 'SilentlyContinue'
$acc = @{}
function Add-Val($procId, $key, $val) {
  if (-not $acc.ContainsKey($procId)) {
    $acc[$procId] = @{ util_3d = 0.0; util_compute = 0.0; util_other = 0.0; loc = 0.0; non = 0.0 }
  }
  $acc[$procId][$key] += $val
}
foreach ($s in (Get-Counter '\GPU Engine(*)\Utilization Percentage').CounterSamples) {
  if ($s.InstanceName -match '^pid_(\d+)_') {
    $p = $matches[1]
    if ($s.InstanceName -match 'engtype_3D') { Add-Val $p 'util_3d' $s.CookedValue }
    elseif ($s.InstanceName -match 'engtype_Compute') { Add-Val $p 'util_compute' $s.CookedValue }
    else { Add-Val $p 'util_other' $s.CookedValue }
  }
}
foreach ($s in (Get-Counter '\GPU Process Memory(*)\Local Usage').CounterSamples) {
  if ($s.InstanceName -match '^pid_(\d+)_') { Add-Val $matches[1] 'loc' $s.CookedValue }
}
foreach ($s in (Get-Counter '\GPU Process Memory(*)\Non Local Usage').CounterSamples) {
  if ($s.InstanceName -match '^pid_(\d+)_') { Add-Val $matches[1] 'non' $s.CookedValue }
}
$names = @{}
foreach ($pr in (Get-Process)) { $names[[string]$pr.Id] = $pr.ProcessName }
$out = @()
foreach ($k in $acc.Keys) {
  $a = $acc[$k]
  $nm = $names[$k]
  if (-not $nm) { $nm = 'unknown' }
  $out += [pscustomobject]@{
    pid = [int]$k; name = $nm
    util_3d = [math]::Round($a.util_3d, 2)
    util_compute = [math]::Round($a.util_compute, 2)
    util_other = [math]::Round($a.util_other, 2)
    loc_mb = [math]::Round($a.loc / 1MB, 1)
    non_mb = [math]::Round($a.non / 1MB, 1)
  }
}
ConvertTo-Json -InputObject @($out) -Compress -Depth 3
"""


def decode_throttle(raw):
    """nvidia-smi's active-reason bitmask -> (hex string, '+'-joined names).

    Returns ("", "") for a blank/unreadable value rather than inventing a 0 -- a GPU that did not
    answer is not a GPU reporting 'no throttling' (CLAUDE.md: NO SILENT FALLBACKS).
    """
    s = str(raw).strip()
    if not s or s.upper().startswith("[N/A]") or s.upper() == "N/A":
        return "", ""
    try:
        val = int(s, 16) if s.lower().startswith("0x") else int(s)
    except ValueError:
        return s, "unparseable"
    names = [n for bit, n in THROTTLE_BITS if val & bit]
    return hex(val), "+".join(names) if names else "none"


def sample_global():
    """One nvidia-smi snapshot. Raises if nvidia-smi is missing or the field count changes -- a
    probe that silently logs blanks is worse than one that refuses to start."""
    out = subprocess.run(
        ["nvidia-smi", "--query-gpu=" + ",".join(_SMI_FIELDS), "--format=csv,noheader,nounits"],
        capture_output=True, text=True, timeout=15, check=True).stdout.strip().splitlines()[0]
    parts = [p.strip() for p in out.split(",")]
    if len(parts) != len(_SMI_FIELDS):
        raise RuntimeError(
            f"nvidia-smi returned {len(parts)} fields, expected {len(_SMI_FIELDS)}: {out!r}")

    def num(x):
        try:
            return float(x)
        except ValueError:
            return ""

    thr_hex, thr_names = decode_throttle(parts[9])
    return {
        "gpu_util_pct": num(parts[0]), "gpu_mem_used_mb": num(parts[1]),
        "gpu_mem_total_mb": num(parts[2]), "gpu_mem_reserved_mb": num(parts[3]),
        "sm_clock_mhz": num(parts[4]), "sm_clock_max_mhz": num(parts[5]),
        "mem_clock_mhz": num(parts[6]), "temp_c": num(parts[7]), "power_w": num(parts[8]),
        "throttle_hex": thr_hex, "throttle_names": thr_names,
    }


def sample_procs(ps1_path):
    """Per-process GPU engine utilisation and memory, via Windows performance counters."""
    r = subprocess.run(["powershell", "-NoProfile", "-NonInteractive", "-File", ps1_path],
                       capture_output=True, text=True, timeout=30)
    txt = (r.stdout or "").strip()
    if not txt:
        return []
    data = json.loads(txt)
    return data if isinstance(data, list) else [data]


def fields(track):
    base = ["wall_ts", "iso", "gpu_util_pct", "gpu_mem_used_mb", "gpu_mem_total_mb",
            "gpu_mem_reserved_mb", "sm_clock_mhz", "sm_clock_max_mhz", "mem_clock_mhz",
            "temp_c", "power_w", "throttle_hex", "throttle_names",
            "vram_local_total_mb", "vram_nonlocal_total_mb"]
    for g in list(track) + ["other"]:
        base += [f"{g}_vram_mb", f"{g}_nonlocal_mb", f"{g}_util_3d", f"{g}_util_compute"]
    base += ["top1_name", "top1_util", "top2_name", "top2_util", "n_gpu_procs"]
    return base


def fold(procs, track):
    """Fold the per-process list into one row: a column group per tracked name, everything else
    summed into `other`, plus the top two consumers BY NAME so nothing large can hide inside the
    `other` bucket."""
    low = [t.lower() for t in track]
    groups = {g: {"vram": 0.0, "non": 0.0, "u3d": 0.0, "uc": 0.0} for g in list(track) + ["other"]}
    for p in procs:
        name = str(p.get("name", "unknown"))
        g = "other"
        for t, t_low in zip(track, low):
            if t_low in name.lower():
                g = t
                break
        groups[g]["vram"] += float(p.get("loc_mb", 0) or 0)
        groups[g]["non"] += float(p.get("non_mb", 0) or 0)
        groups[g]["u3d"] += float(p.get("util_3d", 0) or 0)
        groups[g]["uc"] += float(p.get("util_compute", 0) or 0)

    row = {}
    for g, v in groups.items():
        row[f"{g}_vram_mb"] = round(v["vram"], 1)
        row[f"{g}_nonlocal_mb"] = round(v["non"], 1)
        row[f"{g}_util_3d"] = round(v["u3d"], 2)
        row[f"{g}_util_compute"] = round(v["uc"], 2)

    def total_util(p):
        return (float(p.get("util_3d", 0) or 0) + float(p.get("util_compute", 0) or 0)
                + float(p.get("util_other", 0) or 0))

    ranked = sorted(procs, key=total_util, reverse=True)
    for i in (0, 1):
        if i < len(ranked):
            row[f"top{i+1}_name"] = ranked[i].get("name", "")
            row[f"top{i+1}_util"] = round(total_util(ranked[i]), 2)
        else:
            row[f"top{i+1}_name"] = ""
            row[f"top{i+1}_util"] = ""
    row["vram_local_total_mb"] = round(sum(float(p.get("loc_mb", 0) or 0) for p in procs), 1)
    row["vram_nonlocal_total_mb"] = round(sum(float(p.get("non_mb", 0) or 0) for p in procs), 1)
    row["n_gpu_procs"] = len(procs)
    return row


def run_probe(interval, out_path, track):
    fd, ps1 = tempfile.mkstemp(suffix=".ps1", text=True)
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write(_PS_SCRIPT)
    cols = fields(track)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    print(f"[gpu_probe] -> {out_path}")
    print(f"[gpu_probe] interval={interval}s  tracking={list(track)}  (Ctrl-C to stop)", flush=True)
    n = 0
    try:
        with open(out_path, "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=cols)
            w.writeheader()
            while True:
                t = time.time()
                row = {"wall_ts": round(t, 4),
                       "iso": datetime.fromtimestamp(t).strftime("%H:%M:%S")}
                row.update(sample_global())
                row.update(fold(sample_procs(ps1), track))
                w.writerow({k: row.get(k, "") for k in cols})
                fh.flush()
                n += 1
                if n == 1 or n % 15 == 0:
                    per = " ".join(
                        f"{g}:{row.get(f'{g}_vram_mb', 0):.0f}MB/3d{row.get(f'{g}_util_3d', 0):.0f}%"
                        f"/cu{row.get(f'{g}_util_compute', 0):.0f}%" for g in track)
                    print(f"[{row['iso']}] vram {row['gpu_mem_used_mb']:.0f}/"
                          f"{row['gpu_mem_total_mb']:.0f}MB spill {row['vram_nonlocal_total_mb']}MB | "
                          f"sm {row['sm_clock_mhz']:.0f}/{row['sm_clock_max_mhz']:.0f}MHz "
                          f"{row['temp_c']:.0f}C {row['power_w']:.0f}W | "
                          f"throttle={row['throttle_names'] or '-'} | {per}", flush=True)
                dt = interval - (time.time() - t)
                if dt > 0:
                    time.sleep(dt)
    except KeyboardInterrupt:
        print(f"\n[gpu_probe] stopped after {n} samples -> {out_path}")
    finally:
        try:
            os.unlink(ps1)
        except OSError:
            pass


# ----------------------------------------------------------------------------- report

def _newest(pattern):
    hits = sorted(glob.glob(os.path.join(DIAG, pattern)))
    return hits[-1] if hits else None


def _med(rows, key):
    vals = []
    for r in rows:
        v = r.get(key, "")
        if v not in ("", None):
            try:
                vals.append(float(v))
            except ValueError:
                pass
    return statistics.median(vals) if vals else float("nan")


def report(gpu_csv, perception_csv=None, buckets=12):
    """Bucket the probe by time and, when a flight CSV is given, join session 67's fixed-work
    speedometer (`trk_pre_ms`) alongside -- so GPU state and SLAM throughput are read together."""
    g = list(csv.DictReader(open(gpu_csv, encoding="utf-8")))
    if not g:
        print(f"{os.path.basename(gpu_csv)}: no samples")
        return
    t0 = float(g[0]["wall_ts"])
    span = float(g[-1]["wall_ts"]) - t0
    track = [c[:-len("_vram_mb")] for c in g[0] if c.endswith("_vram_mb")]
    track = [t for t in track if t != "other"]

    pre_by_bucket = {}
    if perception_csv:
        p = [r for r in csv.DictReader(open(perception_csv, encoding="utf-8"))
             if r.get("trk_gn_exit", "") not in ("", "skipped", "error")]
        for r in p:
            k = int((float(r["wall_ts"]) - t0) / max(span / buckets, 1e-9))
            if 0 <= k < buckets:
                pre_by_bucket.setdefault(k, []).append(float(r["trk_pre_ms"]))

    print(f"\nGPU probe -- {os.path.basename(gpu_csv)} ({len(g)} samples, {span/60:.1f} min)")
    if perception_csv:
        print(f"joined against {os.path.basename(perception_csv)} on wall_ts")
    # Non-local usage is never zero on a live desktop -- the compositor and browsers hold tens of MB
    # there at idle. What indicates OUR workload paging is the RISE above the run's own baseline, so
    # the verdict is stated against that, not against an absolute that false-alarms on sample one.
    spills = [float(r["vram_nonlocal_total_mb"] or 0) for r in g]
    base = statistics.median(spills[:3]) if spills else 0.0
    peak = max(spills, default=0.0)
    rise = peak - base
    verdict = ("PAGING -- branch (b) is live" if rise > 256 else
               "possible, watch it" if rise > 64 else
               "no meaningful rise -- branch (b) is ruled out")
    print(f"VRAM spill to system RAM: baseline {base:.0f} MB, peak {peak:.0f} MB, "
          f"rise {rise:+.0f} MB ({verdict})")
    peak_used = max((float(r["gpu_mem_used_mb"] or 0) for r in g), default=0.0)
    total = _med(g, "gpu_mem_total_mb")
    print(f"dedicated VRAM: peak {peak_used:.0f} / {total:.0f} MB ({peak_used / total * 100:.0f}% full)"
          if total == total and total else "")
    thr = sorted({r["throttle_names"] for r in g
                  if r["throttle_names"] not in ("", "none", "GpuIdle")})
    print(f"throttle reasons seen (GpuIdle ignored -- it means the GPU had nothing to do): "
          f"{', '.join(thr) if thr else 'none -- branch (c) is ruled out'}")

    hdr = ["min", "n", "vram_MB", "spill", "sm_MHz", "degC", "W"]
    for t in track:
        hdr += [f"{t}_MB", f"{t}_3d%", f"{t}_cu%"]
    if perception_csv:
        hdr += ["trk_pre_ms"]
    print("| " + " | ".join(hdr) + " |")
    print("|" + "|".join("---" for _ in hdr) + "|")
    for k in range(buckets):
        lo, hi = t0 + k * span / buckets, t0 + (k + 1) * span / buckets
        b = [r for r in g if lo <= float(r["wall_ts"]) < hi]
        if not b:
            continue
        cells = [f"{(lo - t0) / 60:.0f}", str(len(b)),
                 f"{_med(b, 'gpu_mem_used_mb'):.0f}", f"{_med(b, 'vram_nonlocal_total_mb'):.0f}",
                 f"{_med(b, 'sm_clock_mhz'):.0f}", f"{_med(b, 'temp_c'):.0f}",
                 f"{_med(b, 'power_w'):.0f}"]
        for t in track:
            cells += [f"{_med(b, f'{t}_vram_mb'):.0f}", f"{_med(b, f'{t}_util_3d'):.0f}",
                      f"{_med(b, f'{t}_util_compute'):.0f}"]
        if perception_csv:
            v = pre_by_bucket.get(k, [])
            cells += [f"{statistics.median(v):.0f}" if v else "--"]
        print("| " + " | ".join(cells) + " |")


# ----------------------------------------------------------------------------- self-test

def run_self_test():
    ok = True

    def case(name, cond):
        nonlocal ok
        print(f"  {'PASS' if cond else 'FAIL'}  {name}")
        ok = ok and bool(cond)

    print("gpu_probe self-test")
    case("throttle 0x0 -> none", decode_throttle("0x0") == ("0x0", "none"))
    case("throttle 0x1 -> GpuIdle", decode_throttle("0x1")[1] == "GpuIdle")
    case("throttle 0x60 -> both thermal bits",
         decode_throttle("0x60")[1] == "SwThermalSlowdown+HwThermalSlowdown")
    case("throttle [N/A] -> blank, not a zero", decode_throttle("[N/A]") == ("", ""))
    case("throttle garbage -> flagged, not swallowed", decode_throttle("zzz")[1] == "unparseable")

    cols = fields(["Xlab", "python"])
    case("fields unique", len(set(cols)) == len(cols))
    case("fields carry both groups plus other",
         all(f"{g}_vram_mb" in cols for g in ("Xlab", "python", "other")))
    case("spill column present", "vram_nonlocal_total_mb" in cols)

    procs = [{"name": "Xlab", "pid": 1, "util_3d": 60.0, "util_compute": 0.0, "util_other": 0.0,
              "loc_mb": 3000.0, "non_mb": 0.0},
             {"name": "python", "pid": 2, "util_3d": 0.0, "util_compute": 30.0, "util_other": 0.0,
              "loc_mb": 5000.0, "non_mb": 250.0},
             {"name": "chrome", "pid": 3, "util_3d": 5.0, "util_compute": 0.0, "util_other": 0.0,
              "loc_mb": 100.0, "non_mb": 0.0}]
    row = fold(procs, ["Xlab", "python"])
    case("Unity 3d util routed to Xlab", row["Xlab_util_3d"] == 60.0)
    case("CUDA util routed to python compute", row["python_util_compute"] == 30.0)
    case("untracked process lands in other", row["other_vram_mb"] == 100.0)
    case("spill totalled across processes", row["vram_nonlocal_total_mb"] == 250.0)
    case("top1 is the biggest consumer by name", row["top1_name"] == "Xlab")
    case("empty process list does not crash", fold([], ["Xlab"])["n_gpu_procs"] == 0)

    # A python process matched before Xlab must not steal Xlab's rows: first match in `track` wins.
    case("group match order is deterministic",
         fold([{"name": "Xlab", "loc_mb": 1.0}], ["python", "Xlab"])["Xlab_vram_mb"] == 1.0)

    try:
        gl = sample_global()
        case("live nvidia-smi parses", isinstance(gl["gpu_mem_total_mb"], float))
        print(f"        (live: {gl['gpu_mem_used_mb']:.0f}/{gl['gpu_mem_total_mb']:.0f} MB, "
              f"sm {gl['sm_clock_mhz']:.0f}/{gl['sm_clock_max_mhz']:.0f} MHz, "
              f"{gl['temp_c']:.0f}C, throttle={gl['throttle_names']})")
    except Exception as exc:
        case(f"live nvidia-smi parses ({type(exc).__name__}: {exc})", False)

    print("SELF-TEST", "PASS" if ok else "FAIL")
    return ok


def main():
    ap = argparse.ArgumentParser(description="Log GPU state and per-process GPU usage during a flight")
    ap.add_argument("--interval", type=float, default=2.0, help="seconds between samples")
    ap.add_argument("--out", default=None, help="output CSV (default OUTPUT/diag/<ts>_gpu.csv)")
    ap.add_argument("--track", default="Xlab,python",
                    help="comma-separated process-name substrings to break out (Xlab = the Unity sim)")
    ap.add_argument("--report", nargs="?", const="__newest__", default=None,
                    help="report on a gpu CSV instead of logging (no arg = newest)")
    ap.add_argument("--perception", default=None,
                    help="flight perception CSV to join on wall_ts (default: newest)")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()

    if args.self_test:
        sys.exit(0 if run_self_test() else 1)

    track = [t.strip() for t in args.track.split(",") if t.strip()]

    if args.report:
        gpu_csv = _newest("*_gpu.csv") if args.report == "__newest__" else args.report
        if not gpu_csv:
            sys.exit("no *_gpu.csv found in OUTPUT/diag")
        perc = args.perception or _newest("*_perception.csv")
        report(gpu_csv, perc)
        return

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_probe(args.interval, args.out or os.path.join(DIAG, f"{ts}_gpu.csv"), track)


if __name__ == "__main__":
    main()
