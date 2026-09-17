"""Session 69: decide how long to hold still while SLAM is lost, from what SLAM is actually seeing.

Every hold in the autopilot's loss recovery -- the LOSS-RECOVERY GRACE before FALLBACK, FALLBACK's
INITIAL_WAIT, the dwell after the first back-off (BACKOFF_WAIT), the settle after each sweep push
(WAIT_POST), and SERVO's hold at the F_LKG viewpoint -- used to be a fixed timer. Two instrumented
flights (2026-09-17, perception CSV `reloc_*` columns) showed the timer answering the wrong question:
facing a textureless wall, every relocalisation attempt returns ZERO retrieval candidates ("I do not
recognise this view at all") for as long as the drone stays put -- 16, 29, 16 consecutive empties in
three episodes, ending only when the drone moved -- so 32 s of hovering there (12 s grace + 20 s
initial wait) could never have worked. Conversely, once candidates appear, `best` climbs over a
handful of attempts and recovery follows, and THAT is when holding still pays; the old timers cut
those holds short (SERVO abandoned after 1.5 s -- less than one 1.7 s attempt).

`RelocHoldGate` replaces "how long has it been" with "what did the last few attempts see":
  * fewer than `streak` attempts observed during this hold -> no evidence yet -> hold, up to the
    site's own LEGACY limit (so a hold with no attempts at all -- SLAM slow but still TRACKING, or
    perception silent -- behaves exactly as before; nothing here is a silent change of that case);
  * an attempt is WEAK if it returned 0 candidates OR its best mutual match fraction is below
    `weak_frac` x the acceptance threshold SLAM publishes on the plan (`reloc_min_match_frac`).
    Flight 20260917_222824 showed why "any candidate" is not evidence: retrieval's own threshold is
    so low it returns SOMETHING almost anywhere, and the gate v1 held 20 s caps on a candidate flat
    at 0.08 -- ~80 s of the flight's last episode. Every real recovery in three instrumented flights
    came from a best >= ~0.18 that was CLIMBING within a few attempts.
  * the last `streak` attempts of this hold all WEAK, and at least `min_s` elapsed -> LEAVE
    ("no usable candidates; change the view");
  * strong candidates present -> KEEP while the best is still IMPROVING (max of the last `streak`
    attempts vs the `streak` before them, by more than `progress_delta`); once `2*streak` attempts
    show no improvement -> LEAVE ("stagnant") -- with non-strict acceptance a good match recovers
    within an attempt or two, so a flat score means this pose will not do it; `cap_s` still bounds.
Attempts are counted only if they arrived during the hold (`t >= hold_t0`): a streak of empties from
BEFORE a push must not decide a hold that starts AFTER it -- the push changed the view.

`min_s`, `cap_s`, `streak`, `weak_frac` and `progress_delta` are general robustness parameters
(durations, a count, a ratio of SLAM's own threshold, a score delta), not a room's answer; the
decision itself comes from live retrieval evidence (CLAUDE.md autonomy standard).

Pure stdlib. `python reloc_hold.py --self-test`.
"""

from collections import deque
from dataclasses import dataclass


@dataclass(frozen=True)
class RelocAttempt:
    t: float            # autopilot clock when the plan carrying it arrived
    attempt: int        # slam_engine's cumulative attempt counter (dedupe key)
    n_cand: int         # retrieval candidates the attempt had
    best: float         # best candidate's mutual match fraction


@dataclass(frozen=True)
class HoldVerdict:
    leave: bool
    reason: str         # one short clause, for the event line and the panel
    attempts: int       # attempts observed during this hold
    weak: int           # trailing run of WEAK attempts (0 candidates or best < weak threshold), capped at streak
    best: float         # best fraction seen during this hold


class RelocHoldGate:
    def __init__(self, min_s: float = 3.0, cap_s: float = 20.0, streak: int = 3,
                 weak_frac: float = 0.5, progress_delta: float = 0.02, maxlen: int = 64):
        assert min_s >= 0.0 and cap_s > 0.0 and streak >= 1, (min_s, cap_s, streak)
        assert 0.0 <= weak_frac <= 1.0 and progress_delta >= 0.0, (weak_frac, progress_delta)
        self.min_s = float(min_s)
        self.cap_s = float(cap_s)
        self.streak = int(streak)
        self.weak_frac = float(weak_frac)
        self.progress_delta = float(progress_delta)
        self._hist: deque = deque(maxlen=maxlen)
        self._last_attempt = 0
        self._thr = None            # SLAM's reloc.min_match_frac, as last published on the plan

    @property
    def weak_thr(self) -> float:
        """best fraction under which a candidate is not evidence. 0.0 until SLAM has published its
        threshold (then any candidate counts, and the verdict reason says so)."""
        return 0.0 if self._thr is None else self.weak_frac * self._thr

    # ---------------------------------------------------------------- intake
    def note_plan(self, plan: dict, now: float) -> bool:
        """Feed one TOPIC_PLAN payload. Returns True when it carried a NEW attempt. The plan republishes
        on a timer, so the same attempt arrives many times -- dedupe on the cumulative counter. A plan
        from before the reloc columns existed (no key) is simply not an attempt."""
        thr = plan.get("reloc_min_match_frac")
        if thr is not None:
            self._thr = float(thr)
        att = plan.get("reloc_attempt")
        if not att:
            return False
        att = int(att)
        if att <= self._last_attempt:
            return False
        self._last_attempt = att
        self._hist.append(RelocAttempt(t=float(now), attempt=att,
                                       n_cand=int(plan.get("reloc_n_cand") or 0),
                                       best=float(plan.get("reloc_best_frac") or 0.0)))
        return True

    def attempts_since(self, t0: float) -> int:
        return sum(1 for a in self._hist if a.t >= t0)

    # ---------------------------------------------------------------- verdict
    def verdict(self, now: float, hold_t0: float, legacy_limit_s: float,
                cap_s: "float | None" = None) -> HoldVerdict:
        """Should a hold that began at `hold_t0` end now? `legacy_limit_s` is what the site used to
        wait when it had no evidence; `cap_s` overrides the gate's cap for this site (SERVO's 12 s)."""
        cap = self.cap_s if cap_s is None else float(cap_s)
        K = self.streak
        wthr = self.weak_thr
        elapsed = now - hold_t0
        since = [a for a in self._hist if a.t >= hold_t0]
        best = max((a.best for a in since), default=0.0)
        weak = lambda a: a.n_cand == 0 or a.best < wthr
        run = 0
        for a in reversed(since):
            if not weak(a):
                break
            run += 1
            if run >= K:
                break
        n = len(since)
        if n < K:
            # not enough evidence either way -> the site's own legacy timer decides
            if elapsed >= legacy_limit_s:
                return HoldVerdict(True, f"legacy limit {legacy_limit_s:.0f}s reached with "
                                         f"{n} reloc attempt(s) observed", n, run, best)
            return HoldVerdict(False, "waiting for reloc evidence", n, run, best)
        if run >= K:
            thr_txt = f"best < {wthr:.2f}" if wthr > 0 else "thr unknown"
            if elapsed >= self.min_s:
                return HoldVerdict(True, f"no usable candidates: last {K} reloc attempts empty or "
                                         f"{thr_txt}", n, run, best)
            return HoldVerdict(False, "min hold", n, run, best)
        # strong candidates in the tail: hold while the best is still improving
        recent = max(a.best for a in since[-K:])
        if n >= 2 * K and elapsed >= self.min_s:
            prior = max(a.best for a in since[-2 * K:-K])
            if recent < prior + self.progress_delta:
                return HoldVerdict(True, f"stagnant: best {recent:.2f} not improving on {prior:.2f} "
                                         f"over the last {2 * K} attempts", n, run, best)
        if elapsed >= cap:
            return HoldVerdict(True, f"cap {cap:.0f}s reached with candidates present (best {best:.2f})",
                               n, run, best)
        return HoldVerdict(False, f"candidates present and improving (best {best:.2f})", n, run, best)


# ----------------------------------------------------------------------------- self-test
def run_self_test() -> None:
    ok = True

    def check(label, cond):
        nonlocal ok
        ok = ok and bool(cond)
        print(f"[reloc-hold][self-test] {'PASS' if cond else 'FAIL'}  {label}")

    def feed(g, t, att, n, best=0.0, thr=0.35):
        return g.note_plan({"reloc_attempt": att, "reloc_n_cand": n, "reloc_best_frac": best,
                            "reloc_min_match_frac": thr}, t)

    # 1. no attempts at all -> legacy timer, exactly as before
    g = RelocHoldGate(min_s=3.0, cap_s=20.0, streak=3)
    check("no_evidence_keeps_until_legacy", not g.verdict(5.0, 0.0, 12.0).leave)
    check("no_evidence_leaves_at_legacy", g.verdict(12.0, 0.0, 12.0).leave)
    check("no_evidence_reason_names_legacy", "legacy" in g.verdict(12.0, 0.0, 12.0).reason)

    # 2. blank wall: three empties -> leave after min_s, long before legacy
    g = RelocHoldGate(min_s=3.0, cap_s=20.0, streak=3)
    for i, t in enumerate((0.5, 1.0, 1.5), start=1):
        feed(g, t, i, 0)
    check("empties_before_min_hold_keep", not g.verdict(2.0, 0.0, 20.0).leave)
    v = g.verdict(3.0, 0.0, 20.0)
    check("empties_after_min_hold_leave", v.leave and "no usable candidates" in v.reason and v.weak == 3)

    # 3. junk candidates (flight 20260917_222824: 2c/0.08 x16) count as weak below 0.5 x 0.35
    g = RelocHoldGate(min_s=3.0, cap_s=20.0, streak=3)
    check("weak_thr_from_plan", g.weak_thr == 0.0)
    for i, t in enumerate((0.5, 2.0, 3.5), start=1):
        feed(g, t, i, 2, best=0.08)
    check("weak_thr_after_plan", abs(g.weak_thr - 0.175) < 1e-9)
    v = g.verdict(4.0, 0.0, 20.0)
    check("junk_candidates_leave", v.leave and "best < 0.17" in v.reason or "best < 0.18" in v.reason)

    # 4. strong and improving -> keep past legacy; flat -> stagnant leave; cap bounds a slow climb
    g = RelocHoldGate(min_s=3.0, cap_s=20.0, streak=3, progress_delta=0.02)
    for i, (t, b) in enumerate(((0.5, 0.20), (2.0, 0.24), (3.5, 0.30)), start=1):
        feed(g, t, i, 3, best=b)
    v = g.verdict(4.0, 0.0, 3.0)
    check("improving_keeps_past_legacy", not v.leave and "improving" in v.reason and abs(v.best - 0.30) < 1e-9)
    for i, (t, b) in enumerate(((5.0, 0.31), (6.5, 0.30), (8.0, 0.31)), start=4):
        feed(g, t, i, 3, best=b)
    v = g.verdict(8.5, 0.0, 3.0)
    check("flat_is_stagnant_leave", v.leave and "stagnant" in v.reason)
    g2 = RelocHoldGate(min_s=3.0, cap_s=20.0, streak=3, progress_delta=0.02)
    for i in range(1, 15):     # keeps improving by 0.03 every attempt -> only the cap ends it
        feed(g2, 1.5 * i, i, 3, best=0.18 + 0.03 * i)
    check("slow_climb_keeps_until_cap", not g2.verdict(19.9, 0.0, 3.0).leave and g2.verdict(20.0, 0.0, 3.0).leave)
    check("site_cap_override", g2.verdict(12.0, 0.0, 3.0, cap_s=12.0).leave and not g2.verdict(11.0, 0.0, 3.0, cap_s=12.0).leave)

    # 5. the recovery shape (21:36:00 0.08 0.10 0.56): weak, weak, then strong -> keep (streak broken)
    g = RelocHoldGate(min_s=0.0, cap_s=20.0, streak=3)
    feed(g, 0.5, 1, 3, 0.08); feed(g, 1.0, 2, 3, 0.10); feed(g, 1.5, 3, 3, 0.56)
    v = g.verdict(2.0, 0.0, 20.0)
    check("trailing_strong_breaks_weak_run", not v.leave and v.weak == 0)

    # 6. attempts from BEFORE the hold do not count (a push changed the view)
    g = RelocHoldGate(min_s=0.0, cap_s=20.0, streak=3)
    feed(g, 0.5, 1, 0); feed(g, 1.0, 2, 0); feed(g, 1.5, 3, 0)
    v = g.verdict(2.5, 2.0, 10.0)
    check("pre_hold_attempts_ignored", not v.leave and v.attempts == 0)
    check("attempts_since_counts", g.attempts_since(0.0) == 3 and g.attempts_since(2.0) == 0)

    # 7. dedupe: the plan republishes the same attempt on a timer
    g = RelocHoldGate()
    check("first_arrival_is_new", feed(g, 0.0, 7, 3, 0.4))
    check("republish_is_not_new", not feed(g, 0.5, 7, 3, 0.4) and not feed(g, 0.6, 6, 3, 0.4))
    check("absent_key_is_not_an_attempt", not g.note_plan({"slam_ms": 300.0}, 1.0) and not g.note_plan({"reloc_attempt": 0}, 1.0))

    print("[reloc-hold][self-test]", "ALL PASS" if ok else "FAILURES")
    if not ok:
        raise SystemExit(1)


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args()
    if a.self_test:
        run_self_test()
    else:
        ap.print_help()
