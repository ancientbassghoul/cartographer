"""diag_log.py — lightweight CSV diagnostic logger for live-flight debugging.

One file per role under OUTPUT/diag/, flushed per row so a crash/kill never loses data. Used by
object_worker (detection cadence/timing) and perception_worker (per-frame SLAM/loop timing + per-lift
hit geometry) to diagnose the first live-flight issues (detection cadence, flight-path cadence, target
placement). Pure stdlib; no GPU/torch. Disabled = a no-op object (writes nothing).
"""

import csv
from datetime import datetime
from pathlib import Path

REPO = Path(__file__).resolve().parent


class DiagLog:
    """Append-only CSV with a fixed header. `row(**kw)` writes one record (missing fields -> blank)."""

    def __init__(self, role, fields, out_dir=None, ts=None):
        ts = ts or datetime.now().strftime("%Y%m%d_%H%M%S")
        d = Path(out_dir) if out_dir else REPO / "OUTPUT" / "diag"
        d.mkdir(parents=True, exist_ok=True)
        self.path = d / f"{ts}_{role}.csv"
        self.fields = list(fields)
        self._f = open(self.path, "w", newline="", encoding="utf-8")
        self._w = csv.DictWriter(self._f, fieldnames=self.fields)
        self._w.writeheader()
        self._f.flush()
        print(f"[diag] {role} log -> {self.path}", flush=True)

    def row(self, **kw):
        self._w.writerow({k: kw.get(k, "") for k in self.fields})
        self._f.flush()

    def close(self):
        try:
            self._f.close()
        except Exception:
            pass


class NullLog:
    """No-op logger used when --log is off."""
    path = None

    def row(self, **kw):
        pass

    def close(self):
        pass
