"""visual_recovery.py — CPU-only SIFT-based visual loss-recovery probe (session 35 ALT).

The operator's vision: "the live NDI image tells us why tracking dropped and what to do about it." A
PLAN-STALE almost always means the drone got too close to something, or turned to a bad angle — and the
answer is in the live image, never used for recovery before this module.

Caches the last frame SLAM was actually TRACKING on ("F_LKG" — last known good), then matches a live
frame against it with the SAME validated classical-CV primitive already used elsewhere in this project
(`benchmark_detectors.SiftDetector`: cv2 SIFT + Lowe ratio test + RANSAC homography, `SIFT_MIN_INLIERS`
confidence). The ~25 lines are COPIED here (not imported from benchmark_detectors, whose top level pulls
torch/LightGlue for the GPU engines this module deliberately has no reason to load).

================================ CPU-ONLY, ON PURPOSE ================================
SLAM keeps trying to relocalize every stale frame — it is NOT idle while we're deciding what to do. A
GPU-heavy matcher (LightGlue) would contend with the very relocalization we're waiting on. SIFT on the
512x288 transport frame is tens of ms; the drone is hovering/turning slowly during recovery, so a slower
tick here is an acceptable trade. A GPU escalation (LightGlue) stays a noted future option, not built.
========================================================================================

Consumed by `autopilot.py`'s `ExploreController`, gated behind `use_visual_recovery_on_stale`
(config.yaml `autonomy.explore`, default OFF — mirrors `use_rewind_on_stale`'s precedent that a new
stale-recovery path ships live-fly-untested). `ExploreController` stays a pure state machine fed small
verdict values (this module owns ALL image handling), exactly like it already consumes
`wall_contact`/`backwall_contact` from `flow_contact_detector.py`.
"""

import argparse
import time
from dataclasses import dataclass

import cv2
import numpy as np

SIFT_RATIO = 0.75          # Lowe ratio test (matches benchmark_detectors.SiftDetector)
SIFT_MIN_INLIERS = 12      # RANSAC inliers to call it a find (matches benchmark_detectors.SiftDetector)

# Session 49: operator-visible debug canvas (F_LKG | LIVE + drawn RANSAC inliers), built only when
# match(..., debug=True) is passed -- see VisualRecoveryProbe._compose_debug.
BANNER_H = 34          # px of black header strip; NEVER overlays image content
BANNER_FONT = cv2.FONT_HERSHEY_SIMPLEX


@dataclass
class VisualMatch:
    """One verdict from matching a live frame against F_LKG (the cached last-known-good frame).

    Session 57: `scale` (the homography's linear determinant) is a poor direction estimator on its
    own -- diagnosed off a real flight where it read 1.44 -> 1.31 -> 0.53 -> 0.59 -> 0.67 -> 1.33 ->
    1.85 within one second on ~30 inliers, because it conflates perspective foreshortening with a
    weak-fit homography. `spread_lkg`/`spread_live`/`size_ratio`/`closer` add a LINEAR, RANSAC-inlier-
    only measure (RMS distance from centroid) of how much screen area the matched features actually
    span in each frame -- far less sensitive to one stray inlier, and directly answers "which frame is
    the feature cluster bigger in" without going through a projective determinant at all. `scale` is
    kept (never removed) so the two estimators can be compared side by side on real flights."""
    has_lkg: bool                  # a reference frame exists to match against at all
    matched: bool = False          # inliers >= min_inliers
    inliers: int = 0
    contained: bool = False        # F_live is a zoomed-in crop of F_LKG (2a: nose closer to the same surface)
    planar_like: bool = False      # high inlier ratio -> nose-to-a-flat-surface (2b)
    scale: float | None = None     # homography linear scale (LKG->live); only meaningful when matched
    # --- session 57 ---
    spread_lkg: float | None = None    # RMS px distance of INLIER keypoints from their centroid, in F_LKG
    spread_live: float | None = None   # same, in the live frame
    size_ratio: float | None = None    # spread_live / spread_lkg; None unless both spreads are usable
    closer: str = "UNKNOWN"            # "LIVE" | "LKG" | "EQUAL" | "UNKNOWN"
    debug_image: "np.ndarray | None" = None   # BGR canvas (F_LKG | live + inliers), only when debug=True


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
    n = 0
    for m, keep in zip(good, mask):
        if not keep:
            continue
        x0, y0 = kp0[m.queryIdx].pt
        x1, y1 = kp1[m.trainIdx].pt
        cv2.line(body, (int(round(x0)), int(round(y0))),
                (int(round(x1)), int(round(y1)) + h_top), (0, 255, 0), 1, cv2.LINE_AA)
        n += 1
    return n


class VisualRecoveryProbe:
    """Caches F_LKG (the most recent frame SLAM was TRACKING on) and matches later frames against it.
    See the module docstring for why this is CPU-only SIFT, copied (not imported) from
    `benchmark_detectors.SiftDetector`."""

    def __init__(self, *, min_inliers=SIFT_MIN_INLIERS, planar_inlier_ratio=0.85,
                 contain_margin_frac=0.02,
                 size_ratio_hi: float = 1.25, size_ratio_lo: float = 0.80,
                 size_min_inliers: int = 20):
        self.min_inliers = int(min_inliers)
        self.planar_inlier_ratio = float(planar_inlier_ratio)
        self.contain_margin_frac = float(contain_margin_frac)
        # --- session 57: inlier-spread direction verdict thresholds, see `match()`'s `closer` field ---
        self.size_ratio_hi = float(size_ratio_hi)
        self.size_ratio_lo = float(size_ratio_lo)
        self.size_min_inliers = int(size_min_inliers)
        self.sift = cv2.SIFT_create()
        self.matcher = cv2.BFMatcher(cv2.NORM_L2)
        self._lkg = None    # cached BGR frame (last known good — SLAM was TRACKING when it was captured)
        # Session 52: which frame `_lkg` actually IS -- "live" (the tick's own frame, legacy behaviour) or
        # "slam:<frame_id>" (the exact frame the plan was computed from, pulled from run_explore's ring). A
        # LABEL ONLY: never read by match(), just surfaced on the debug banner. Session 56: an aged-out plan
        # frame no longer degrades this to a fake "live" reference (see run_explore) -- the caller simply
        # does not call update_reference that tick, so `_lkg_src` keeps naming whatever reference is still
        # held, and `_lkg_t` (below) lets the banner show how OLD it now is.
        self._lkg_src = "none"
        self._lkg_t = None   # monotonic timestamp of the last successful update_reference (age display)
        # Session 51: LAZY, memoised SIFT for the REFERENCE half of a match. `None` = not computed yet;
        # once computed it holds the (keypoints, descriptors) tuple for the CURRENT `_lkg` and is reused
        # until a new reference replaces it. The flag is the TUPLE SLOT, never the descriptors: a
        # featureless reference legitimately yields (kp, None), and keying on the descriptors would
        # recompute it forever.
        self._lkg_feats = None
        # Session 61: THIS call's RANSAC draw set, so a caller composing its own canvas right after
        # match() can draw the real inlier correspondences (they are match()-local; session 60's panel
        # had none). Tuple of (kp0, kp1, good, mask) or None. Cleared at the TOP of every match() so it
        # can never describe a previous call.
        self._last_draw: "tuple | None" = None

    def update_reference(self, frame, tracked, src: str = "live"):
        """Cache `frame` as F_LKG whenever `tracked` is True. Called every tick from run_explore, gated on
        the same `plan.get("plan_valid")` boundary session-34's own cached-pose snapshot uses.

        Session 52: `frame` is now WHICHEVER frame the caller resolved as the true reference -- run_explore
        keeps a short ring of recent (frame_id, frame) pairs and, when possible, passes the EXACT frame the
        SLAM plan was computed from (this flight measured solve latency up to 14.9s, so "the live frame at
        this tick" and "the frame the plan describes" can be seconds apart). `src` is a caller-supplied
        LABEL for which frame this is ("slam:<frame_id>" | "live") -- stored verbatim for the debug banner
        (`_compose_debug`) and never consulted by matching logic. Cheap (a copy); SIFT only runs at
        match() time.

        Session 51: returns True when it actually STORED a new reference (so run_explore can invalidate
        its own per-tick match memo on that exact edge rather than duplicating this condition), else
        False. Storing also drops the cached reference features — see `_lkg_feats`.

        Session 56: a `tracked=False` call (e.g. the caller's plan frame aged out of its ring) touches
        NEITHER `_lkg`, `_lkg_src`, `_lkg_t` NOR `_lkg_feats` -- the previous reference is kept exactly as
        it was, memoised features included, so a run of age-outs costs nothing extra and never churns the
        SIFT memo."""
        if tracked and frame is not None:
            self._lkg = frame.copy()
            self._lkg_feats = None      # new reference -> the memoised SIFT no longer describes it
            self._lkg_src = str(src)
            self._lkg_t = time.monotonic()
            return True
        return False

    def _reference_keypoints(self):
        """The CURRENT reference's (keypoints, descriptors), computed at most once per reference.

        Session 51: `match()` used to recompute SIFT on `self._lkg` on EVERY call. F_LKG only changes in
        `update_reference`, and during a loss it never changes at all (it only caches while `plan_valid`),
        so a 12s loss-recovery grace at ~32Hz was repeating the identical computation hundreds of times.
        The live frame's SIFT still runs per call -- that frame really is new every time."""
        if self._lkg_feats is None:
            self._lkg_feats = self._keypoints(self._lkg)
        return self._lkg_feats

    def _keypoints(self, bgr):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
        return self.sift.detectAndCompute(gray, None)

    @staticmethod
    def _inlier_spread(points: "np.ndarray") -> float | None:
        """RMS distance of `points` (N,2 float) from their centroid, in pixels.

        Returns None when fewer than 2 points, or when the result is not finite.
        A LINEAR measure of how much screen area the matched features span -- directly comparable
        to the homography `scale`, but far less sensitive to one stray inlier than a hull area.
        """
        if points is None or len(points) < 2:
            return None
        centroid = points.mean(axis=0)
        d = np.sqrt(np.mean(np.sum((points - centroid) ** 2, axis=1)))
        val = float(d)
        return val if np.isfinite(val) else None

    @staticmethod
    def _pad_to_height(img, target_h):
        """Zero-pad `img` at the bottom to `target_h` rows — NEVER resize/scale (IMAGE INTEGRITY,
        CLAUDE.md). A no-op when `img` is already tall enough."""
        h = img.shape[0]
        if h >= target_h:
            return img
        pad_shape = (target_h - h,) + img.shape[1:]
        return np.vstack([img, np.zeros(pad_shape, dtype=img.dtype)])

    @staticmethod
    def _pad_to_width(img, target_w):
        """Zero-pad `img` at the right to `target_w` columns — NEVER resize/scale (IMAGE INTEGRITY,
        CLAUDE.md). A no-op when `img` is already wide enough. Session 60 (C8): the STACKED layout
        pads columns (not rows) since the two frames sit one above the other."""
        w = img.shape[1]
        if w >= target_w:
            return img
        pad_shape = (img.shape[0], target_w - w) + img.shape[2:]
        return np.hstack([img, np.zeros(pad_shape, dtype=img.dtype)])

    def _compose_debug(self, frame, out, kp0=None, kp1=None, good=None, mask=None, banner=None):
        """Build the operator canvas: F_LKG (left) | live (right), with the RANSAC INLIER
        correspondences drawn when a homography survived, and a two-line header.

        IMAGE INTEGRITY (CLAUDE.md): NOTHING here resizes, crops or downscales either frame. The canvas
        width is exactly w_lkg + w_live. If the two frames differ in height (they do not in this
        pipeline — both are the 512x288 transport frame) the SHORTER one is zero-PADDED, never scaled.

        Session 61 (C6): the `stacked` parameter is retired — `compose_stacked_live` (C4) is now the
        visualizer panel's own composer, and keeping two stacked-canvas code paths with different text
        behaviour was a bug farm (Finding D). This function is side-by-side only again, feeding solely
        the PNG evidence trail. `line2` now comes from `banner_fields` (C5) so the PNG and the panel can
        never disagree about a field's value.

        Returns a BGR ndarray.
        """
        lkg = self._lkg
        if kp0 is not None and kp1 is not None and good is not None and mask is not None:
            # cv2.drawMatches already allocates a (max(h1,h2), w1+w2) canvas and top-left-aligns each
            # image into its half — the same zero-pad-not-scale behaviour _pad_to_height gives the
            # fallback branch below, done internally.
            body = cv2.drawMatches(lkg, kp0, frame, kp1, good, None,
                                   matchesMask=mask.astype(np.uint8).tolist(),
                                   flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        else:
            h_max = max(lkg.shape[0], frame.shape[0])
            body = np.hstack([self._pad_to_height(lkg, h_max), self._pad_to_height(frame, h_max)])
        banner_strip = np.zeros((BANNER_H, body.shape[1], 3), dtype=np.uint8)
        line1 = banner or ""
        line2 = "  ".join(self.banner_fields(out, state_status=line1))
        cv2.putText(banner_strip, line1, (6, 13), BANNER_FONT, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(banner_strip, line2, (6, 29), BANNER_FONT, 0.4, (200, 200, 200), 1, cv2.LINE_AA)
        canvas = np.vstack([banner_strip, body])
        w_lkg = lkg.shape[1]
        cv2.putText(canvas, "F_LKG (reference)", (6, canvas.shape[0] - 8), BANNER_FONT, 0.45,
                   (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(canvas, "LIVE", (w_lkg + 6, canvas.shape[0] - 8), BANNER_FONT, 0.45,
                   (0, 255, 255), 1, cv2.LINE_AA)
        return canvas

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
        if live_frame is None:
            return None
        if self._lkg is None:
            return live_frame, 0
        w_max = max(self._lkg.shape[1], live_frame.shape[1])
        top = self._pad_to_width(self._lkg, w_max)
        bot = self._pad_to_width(live_frame, w_max)
        canvas = np.vstack([top, bot])
        n_lines = 0
        if draw_lines and self._last_draw is not None:
            kp0, kp1, good, mask = self._last_draw
            n_lines = _draw_stacked_inliers(canvas, top.shape[0], kp0, kp1, good, mask)
        return canvas, n_lines

    def banner_fields(self, out, *, state_status, live_frame_id=None, n_lines=0,
                      lines_reason=None) -> "list[str]":
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
        if out is None:
            matched_txt, inliers_txt, cont_txt, planar_txt = "F", "0", "F", "F"
            scale_txt, size_txt, closer_txt = "n/a", "n/a", "UNKNOWN"
        else:
            matched_txt = "T" if out.matched else "F"
            inliers_txt = str(out.inliers)
            cont_txt = "T" if out.contained else "F"
            planar_txt = "T" if out.planar_like else "F"
            scale_txt = f"{out.scale:.2f}" if out.scale is not None else "n/a"
            size_txt = f"{out.size_ratio:.2f}" if out.size_ratio is not None else "n/a"
            closer_txt = out.closer if out.closer else "UNKNOWN"
        lkg_age_txt = f"{time.monotonic() - self._lkg_t:.1f}s" if self._lkg_t is not None else "n/a"
        live_seg = f"live=#{live_frame_id}" if live_frame_id is not None else "live=#n/a"
        if n_lines > 0:
            lines_seg = f"lines={n_lines}"
        elif lines_reason is not None:
            lines_seg = f"lines=none ({lines_reason})"
        else:
            lines_seg = "lines=none"
        return [
            f"{state_status}",
            f"src={self._lkg_src}",
            f"lkg_age={lkg_age_txt}",
            live_seg,
            f"matched={matched_txt}",
            f"inliers={inliers_txt}",
            f"cont={cont_txt}",
            f"planar={planar_txt}",
            f"scale={scale_txt}",
            f"size={size_txt}",
            f"closer={closer_txt}",
            lines_seg,
        ]

    def match(self, frame, debug: bool = False, banner: str | None = None) -> VisualMatch:
        """SIFT+RANSAC-homography match of `frame` against the cached F_LKG. H maps LKG -> live (same
        src/dst convention as benchmark_detectors.SiftDetector: src=reference keypoints, dst=frame
        keypoints), so `scale = sqrt(|det(H[:2,:2])|)` reads >1 when the live frame shows a MAGNIFIED
        (closer) view of what F_LKG covered — the natural "moved closer" signal. `contained` needs the
        OPPOSITE direction (does the live frame's own extent sit entirely inside a crop of F_LKG?), so it
        explicitly inverts H rather than reusing it raw.

        Session 49: `debug=True` (default False, zero extra cost) additionally composes an operator-
        readable canvas (F_LKG | live + drawn inlier correspondences, see `_compose_debug`) on
        `out.debug_image` — on EVERY return path that has an F_LKG, including every failure path (no
        descriptors, <4 good matches, a degenerate homography, or a match below `min_inliers`), since
        seeing exactly where/why the match failed is the whole point of the window. `banner` is an
        optional caller-supplied context line (e.g. FSM state + plan status) rendered verbatim on the
        canvas; it never affects the match itself.

        Session 57: once matched, also computes `spread_lkg`/`spread_live` (RMS distance of the RANSAC
        INLIER points from their centroid, in each frame) and derives `size_ratio = spread_live /
        spread_lkg` and a `closer` verdict ("LIVE" | "LKG" | "EQUAL" | "UNKNOWN"). See the `VisualMatch`
        docstring for why this linear spread measure was added alongside `scale` rather than replacing
        it."""
        self._last_draw = None    # session 61 (C2): cleared FIRST so it can never describe a stale call
        if self._lkg is None or frame is None:
            return VisualMatch(has_lkg=self._lkg is not None)
        kp0, des0 = self._reference_keypoints()      # session 51: memoised per reference, not per call
        kp1, des1 = self._keypoints(frame)
        out = VisualMatch(has_lkg=True)
        if des0 is None or des1 is None or len(kp0) < 2 or len(kp1) < 2:
            if debug:
                out.debug_image = self._compose_debug(frame, out, banner=banner)
            return out
        good = []
        for m_n in self.matcher.knnMatch(des0, des1, k=2):
            if len(m_n) == 2 and m_n[0].distance < SIFT_RATIO * m_n[1].distance:
                good.append(m_n[0])
        if len(good) < 4:
            if debug:
                out.debug_image = self._compose_debug(frame, out, banner=banner)
            return out
        src = np.float32([kp0[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)   # F_LKG points
        dst = np.float32([kp1[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)   # F_live points
        H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)
        if H is None or mask is None:
            if debug:
                out.debug_image = self._compose_debug(frame, out, banner=banner)
            return out
        mask = mask.ravel().astype(bool)
        inliers = int(mask.sum())
        out.inliers = inliers
        out.matched = inliers >= self.min_inliers
        if not out.matched:
            self._last_draw = (kp0, kp1, good, mask)    # session 61 (C2): outside the debug guard
            if debug:
                out.debug_image = self._compose_debug(frame, out, kp0=kp0, kp1=kp1, good=good, mask=mask,
                                                       banner=banner)
            return out
        out.planar_like = (inliers / float(len(good))) >= self.planar_inlier_ratio
        # Session 57: direction verdict from RANSAC-inlier spread (see VisualMatch docstring, Finding 3
        # in the session-57 spec -- `scale` alone is too noisy to gate a physical back-off on).
        src_in = src.reshape(-1, 2)[mask]      # F_LKG inlier points
        dst_in = dst.reshape(-1, 2)[mask]      # live inlier points
        out.spread_lkg = self._inlier_spread(src_in)
        out.spread_live = self._inlier_spread(dst_in)
        if (out.spread_lkg is not None and out.spread_live is not None and out.spread_lkg > 1e-6):
            out.size_ratio = out.spread_live / out.spread_lkg
            if inliers < self.size_min_inliers:
                out.closer = "UNKNOWN"
            elif out.size_ratio > self.size_ratio_hi:
                out.closer = "LIVE"
            elif out.size_ratio < self.size_ratio_lo:
                out.closer = "LKG"
            else:
                out.closer = "EQUAL"
        try:
            out.scale = float(np.sqrt(abs(np.linalg.det(H[:2, :2]))))
        except (np.linalg.LinAlgError, ValueError):
            out.scale = None
        try:
            h_live, w_live = frame.shape[:2]
            h_lkg, w_lkg = self._lkg.shape[:2]
            H_inv = np.linalg.inv(H)                       # live -> LKG (the direction "contained" needs)
            corners = np.float32([[0, 0], [w_live, 0], [w_live, h_live], [0, h_live]]).reshape(-1, 1, 2)
            warped = cv2.perspectiveTransform(corners, H_inv).reshape(-1, 2)
            mx, my = self.contain_margin_frac * w_lkg, self.contain_margin_frac * h_lkg
            out.contained = bool(np.all(warped[:, 0] >= -mx) and np.all(warped[:, 0] <= w_lkg + mx)
                                  and np.all(warped[:, 1] >= -my) and np.all(warped[:, 1] <= h_lkg + my))
        except np.linalg.LinAlgError:
            out.contained = False       # a singular H (degenerate match) can't be inverted -> not contained
        self._last_draw = (kp0, kp1, good, mask)    # session 61 (C2): outside the debug guard
        if debug:
            out.debug_image = self._compose_debug(frame, out, kp0=kp0, kp1=kp1, good=good, mask=mask,
                                                   banner=banner)
        return out


# ==============================================================================
# Self-test: deterministic synthetic frame pairs (no hardware, no recorded flight).
# ==============================================================================
def _textured_image(w=480, h=360, seed=0, n_shapes=90):
    """A procedurally-textured BGR image with plenty of SIFT-friendly corners (random rects + circles over
    a mild noise floor), so warps/crops of it produce enough stable keypoints to match reliably."""
    rng = np.random.RandomState(seed)
    img = (rng.randint(20, 60, (h, w, 3))).astype(np.uint8)   # low-amplitude background texture
    for _ in range(n_shapes):
        x, y = int(rng.randint(0, w)), int(rng.randint(0, h))
        r = int(rng.randint(10, 34))
        color = tuple(int(c) for c in rng.randint(0, 255, 3))
        if rng.rand() < 0.5:
            cv2.rectangle(img, (x, y), (min(w - 1, x + r), min(h - 1, y + r)), color, -1)
        else:
            cv2.circle(img, (x, y), r, color, -1)
    return img


def run_self_test():
    ok = True

    def case(name, good):
        nonlocal ok
        ok = ok and good
        print(f"[self-test] {'PASS' if good else 'FAIL'}  {name}")

    base = _textured_image(seed=1)
    h, w = base.shape[:2]

    # 1. CONTAINED (zoom-in): F_live is a magnified central crop of F_LKG -> matched, contained, scale>=1.
    probe = VisualRecoveryProbe()
    probe.update_reference(base, True)
    cx0, cy0, cx1, cy1 = w // 4, h // 4, 3 * w // 4, 3 * h // 4
    crop = base[cy0:cy1, cx0:cx1]
    zoomed = cv2.resize(crop, (w, h), interpolation=cv2.INTER_LINEAR)
    vm_zoom = probe.match(zoomed)
    case(f"contained/zoom-in -> matched+contained+scale>=1.15 "
         f"(matched={vm_zoom.matched} contained={vm_zoom.contained} scale={vm_zoom.scale})",
         vm_zoom.matched and vm_zoom.contained and vm_zoom.scale is not None and vm_zoom.scale >= 1.15)

    # 2. PLANAR-LIKE (nose-to-a-flat-surface): a mild GLOBAL perspective warp of the full frame -> a single
    #    homography explains nearly every match -> high inlier ratio, no crop -> not contained.
    probe2 = VisualRecoveryProbe()
    probe2.update_reference(base, True)
    src_pts = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
    dst_pts = np.float32([[0.06 * w, 0.03 * h], [0.97 * w, 0.0], [1.0 * w, 0.98 * h], [0.02 * w, 1.0 * h]])
    Hp = cv2.getPerspectiveTransform(src_pts, dst_pts)
    planar_live = cv2.warpPerspective(base, Hp, (w, h), borderMode=cv2.BORDER_REPLICATE)
    vm_planar = probe2.match(planar_live)
    case(f"planar-like (flat-surface warp) -> matched+planar_like, not contained "
         f"(matched={vm_planar.matched} planar_like={vm_planar.planar_like} contained={vm_planar.contained})",
         vm_planar.matched and vm_planar.planar_like and not vm_planar.contained)

    # 3. GENUINE PARALLAX: two independently-shifted regions (simulates two depth planes moving
    #    differently) -> matched (enough total inliers), but NO single homography fits both regions well
    #    -> inlier ratio stays below the planar threshold, and scale reads ~1 (no overall zoom).
    probe3 = VisualRecoveryProbe()
    probe3.update_reference(base, True)
    parallax_live = base.copy()
    left = base[:, : w // 2]
    right = base[:, w // 2:]
    shifted_left = np.roll(left, 6, axis=1)
    shifted_right = np.roll(right, -18, axis=0)
    parallax_live[:, : w // 2] = shifted_left
    parallax_live[:, w // 2:] = shifted_right
    vm_parallax = probe3.match(parallax_live)
    case(f"genuine parallax (two depth planes) -> matched, NOT planar_like, scale~=1 "
         f"(matched={vm_parallax.matched} planar_like={vm_parallax.planar_like} scale={vm_parallax.scale})",
         vm_parallax.matched and not vm_parallax.planar_like
         and vm_parallax.scale is not None and 0.7 <= vm_parallax.scale <= 1.3)

    # 4. NO-OVERLAP: a wholly unrelated textured image -> no match.
    probe4 = VisualRecoveryProbe()
    probe4.update_reference(base, True)
    unrelated = _textured_image(seed=99)
    vm_none = probe4.match(unrelated)
    case(f"no-overlap (unrelated frame) -> not matched (inliers={vm_none.inliers})", not vm_none.matched)

    # 5. update_reference only caches on tracked=True; no reference -> has_lkg False, no crash.
    probe5 = VisualRecoveryProbe()
    vm_empty = probe5.match(base)
    probe5.update_reference(base, False)      # NOT tracked -> must NOT cache
    vm_still_empty = probe5.match(base)
    case("update_reference ignores tracked=False (no cache -> has_lkg stays False)",
         not vm_empty.has_lkg and not vm_still_empty.has_lkg)
    probe5.update_reference(base, True)
    vm_now = probe5.match(base)
    case("update_reference caches on tracked=True (self-match -> has_lkg + matched)",
         vm_now.has_lkg and vm_now.matched)

    # ---- SESSION 49 — debug canvas (F_LKG | LIVE + drawn inliers) --------------------------------------
    # Operator ask: see what the CV probe is matching against, in its own window. All image handling
    # stays in this module per its own docstring rule.

    # 6. NO-RESIZE CONTRACT: a genuine match (the CONTAINED case, so it exercises the cv2.drawMatches
    #    branch) -> the canvas is EXACTLY w_lkg + w_live wide and max(h_lkg, h_live) + BANNER_H tall.
    #    IMAGE INTEGRITY (CLAUDE.md): this is the assertion that proves nothing was resized anywhere.
    probe6 = VisualRecoveryProbe()
    probe6.update_reference(base, True)
    vm6 = probe6.match(zoomed, debug=True)
    shape6 = None if vm6.debug_image is None else vm6.debug_image.shape
    case(f"debug=True (matched case) -> canvas is EXACTLY w_lkg+w_live wide, h_max+BANNER_H tall "
         f"(shape={shape6})",
         vm6.debug_image is not None
         and vm6.debug_image.shape[1] == base.shape[1] + zoomed.shape[1]
         and vm6.debug_image.shape[0] == max(base.shape[0], zoomed.shape[0]) + BANNER_H)

    # 7. DEFAULT PATH PAYS NOTHING: debug=False -> debug_image is None, and debug rendering never
    #    perturbs the verdict (every OTHER field is identical to the debug=True call on the same input).
    probe7 = VisualRecoveryProbe()
    probe7.update_reference(base, True)
    vm7_debug = probe7.match(zoomed, debug=True)
    vm7_default = probe7.match(zoomed)
    case("debug=False -> debug_image is None, and verdict fields are IDENTICAL to the debug=True call "
         "(debug rendering never perturbs the match)",
         vm7_default.debug_image is None
         and vm7_default.has_lkg == vm7_debug.has_lkg
         and vm7_default.matched == vm7_debug.matched
         and vm7_default.inliers == vm7_debug.inliers
         and vm7_default.contained == vm7_debug.contained
         and vm7_default.planar_like == vm7_debug.planar_like
         and vm7_default.scale == vm7_debug.scale)

    # 8. FAILURE PATHS STILL DRAW: an unrelated image (not matched) still composes a canvas -- seeing
    #    exactly where/why the match FAILED is the whole point of the window.
    probe8 = VisualRecoveryProbe()
    probe8.update_reference(base, True)
    vm8 = probe8.match(_textured_image(seed=99), debug=True)
    case(f"failure path (unrelated image, not matched) still draws a debug canvas "
         f"(matched={vm8.matched}, debug_image={'set' if vm8.debug_image is not None else 'None'})",
         vm8.matched is False and vm8.debug_image is not None)

    # 9. NO REFERENCE: nothing cached yet -> debug=True must not crash, and there is nothing to draw.
    probe9 = VisualRecoveryProbe()
    vm9 = probe9.match(base, debug=True)
    case("no F_LKG cached -> debug=True yields debug_image=None (nothing to compare against), no crash",
         vm9.debug_image is None and vm9.has_lkg is False)

    # 10. BANNER: caller-supplied context line renders without raising; banner=None is also safe, and
    #     the banner text never changes the canvas SIZE (only its header pixels).
    probe10 = VisualRecoveryProbe()
    probe10.update_reference(base, True)
    vm10a = probe10.match(zoomed, debug=True, banner="HOLD_LOST / PLAN-LOST")
    vm10b = probe10.match(zoomed, debug=True, banner=None)
    case("banner text renders without raising; banner=None is safe; banner never changes canvas size",
         vm10a.debug_image is not None and vm10b.debug_image is not None
         and vm10a.debug_image.shape == vm10b.debug_image.shape)

    # ---- SESSION-60 LKG PANEL — RETIRED (session 61, C6): _compose_debug's `stacked` parameter is
    # gone. `compose_stacked_live` (C4) is now the panel's own composer — two stacked-canvas code
    # paths with different text behaviour was a bug farm (Finding D: the operator lost scale/size/
    # closer/lkg_src/age off the right edge of a 512px canvas). The side-by-side shape case (60-1)
    # asserted is already covered by test 6 (NO-RESIZE CONTRACT) above; compose_stacked_live itself
    # is covered by the SESSION-61 STACKED LIVE CANVAS block below. ------------------------------

    # ---- SESSION-61 STACKED LIVE CANVAS — compose_stacked_live / _last_draw / banner_fields
    # (Findings A-D: the panel must stay live on a cadence, draw real inlier lines, and never clip
    # a field off the right edge). --------------------------------------------------------------

    # 1. F_LKG held -> stacked shape is (h_lkg+h_live, max(w_lkg,w_live), 3), no lines without a match.
    probe61a = VisualRecoveryProbe()
    probe61a.update_reference(base, True)
    result61a = probe61a.compose_stacked_live(zoomed)
    h_lkg61, w_lkg61 = base.shape[:2]
    h_live61, w_live61 = zoomed.shape[:2]
    case(f"(61-1) compose_stacked_live with F_LKG held -> shape (h_lkg+h_live, max(w), 3), n_lines=0 "
         f"(result={None if result61a is None else result61a[0].shape})",
         result61a is not None
         and result61a[0].shape == (h_lkg61 + h_live61, max(w_lkg61, w_live61), 3)
         and result61a[1] == 0)

    # 2. compose_stacked_live(None) -> None (nothing honest to draw).
    case("(61-2) compose_stacked_live(None) -> None",
         probe61a.compose_stacked_live(None) is None)

    # 3. Fresh probe, NO F_LKG -> live-half-only shape, not None, not blank.
    probe61b = VisualRecoveryProbe()
    result61b = probe61b.compose_stacked_live(zoomed)
    case(f"(61-3) no F_LKG yet -> live-half-only shape (h_live,w_live,3), n_lines=0, not None "
         f"(result={None if result61b is None else result61b[0].shape})",
         result61b is not None and result61b[0].shape == (h_live61, w_live61, 3) and result61b[1] == 0)

    # 4. _last_draw lifecycle: None on a fresh probe; None after a match() that returns before RANSAC
    #    (a flat-black frame yields no descriptors); a 4-tuple after a homography survives.
    probe61c = VisualRecoveryProbe()
    last_draw_fresh = probe61c._last_draw
    probe61c.update_reference(base, True)
    probe61c.match(np.zeros((h, w, 3), dtype=np.uint8))     # flat black -> no descriptors, no RANSAC
    last_draw_no_desc = probe61c._last_draw
    probe61c.match(zoomed)                                    # real homography
    last_draw_matched = probe61c._last_draw
    case(f"(61-4) _last_draw: None on fresh probe, None after a no-descriptor match, 4-tuple after a "
         f"matched homography (fresh={last_draw_fresh}, no_desc={last_draw_no_desc}, "
         f"matched={'4-tuple' if isinstance(last_draw_matched, tuple) and len(last_draw_matched) == 4 else last_draw_matched})",
         last_draw_fresh is None and last_draw_no_desc is None
         and isinstance(last_draw_matched, tuple) and len(last_draw_matched) == 4)

    # 5. After a matched match(), draw_lines=True yields n_lines>0 and a canvas that differs from
    #    draw_lines=False (the lines are REALLY drawn, not just counted).
    probe61d = VisualRecoveryProbe()
    probe61d.update_reference(base, True)
    probe61d.match(zoomed)
    canvas_lines, n_lines61d = probe61d.compose_stacked_live(zoomed, draw_lines=True)
    canvas_nolines, n_nolines61d = probe61d.compose_stacked_live(zoomed, draw_lines=False)
    case(f"(61-5) draw_lines=True after a matched match() -> n_lines>0 and canvas pixel-differs from "
         f"draw_lines=False (n_lines={n_lines61d}, n_nolines={n_nolines61d})",
         n_lines61d > 0 and n_nolines61d == 0
         and not np.array_equal(canvas_lines, canvas_nolines))

    # 6. draw_lines=True with NO prior match (_last_draw is None) -> n_lines=0, no raise.
    probe61e = VisualRecoveryProbe()
    probe61e.update_reference(base, True)
    result61e = probe61e.compose_stacked_live(zoomed, draw_lines=True)
    case(f"(61-6) draw_lines=True with _last_draw=None -> n_lines=0, no raise (n_lines={result61e[1]})",
         result61e is not None and result61e[1] == 0)

    # 7. banner_fields: exactly 12 segments, in order, for a matched VisualMatch and for out=None;
    #    the `lines=` formatting rules.
    probe61f = VisualRecoveryProbe()
    probe61f.update_reference(base, True)
    vm61f = probe61f.match(zoomed)
    fields_matched = probe61f.banner_fields(vm61f, state_status="HOLD_LOST / PLAN-LOST", n_lines=7)
    fields_none = probe61f.banner_fields(None, state_status="OK")
    fields_reason = probe61f.banner_fields(vm61f, state_status="OK", n_lines=0, lines_reason="plan OK")
    fields_bare = probe61f.banner_fields(vm61f, state_status="OK", n_lines=0)
    case(f"(61-7a) banner_fields returns exactly 12 segments for a matched VisualMatch and for out=None "
         f"(matched_len={len(fields_matched)}, none_len={len(fields_none)})",
         len(fields_matched) == 12 and len(fields_none) == 12
         and fields_matched[0] == "HOLD_LOST / PLAN-LOST" and fields_none[0] == "OK")
    case(f"(61-7b) lines= formatting: 'lines=7' / 'lines=none (plan OK)' / 'lines=none' "
         f"(fields_matched[11]={fields_matched[11]!r}, fields_reason[11]={fields_reason[11]!r}, "
         f"fields_bare[11]={fields_bare[11]!r})",
         fields_matched[11] == "lines=7" and fields_reason[11] == "lines=none (plan OK)"
         and fields_bare[11] == "lines=none")

    # 8. REGRESSION: the side-by-side debug_image from a matched match(debug=True) still has shape
    #    (max(h)+BANNER_H, w_lkg+w_live, 3), unchanged by the _compose_debug rewrite (C6).
    probe61g = VisualRecoveryProbe()
    probe61g.update_reference(base, True)
    vm61g = probe61g.match(zoomed, debug=True)
    case(f"(61-8) regression: side-by-side debug_image shape unchanged after C6's rewrite "
         f"(shape={None if vm61g.debug_image is None else vm61g.debug_image.shape})",
         vm61g.debug_image is not None
         and vm61g.debug_image.shape == (max(base.shape[0], zoomed.shape[0]) + BANNER_H,
                                         base.shape[1] + zoomed.shape[1], 3))

    # ---- SESSION 51 — lazy, memoised SIFT on the REFERENCE half -----------------------------------------
    # match() used to recompute SIFT on _lkg every single call; during a loss the reference is frozen
    # (update_reference only caches while plan_valid), so a 12s grace at ~32Hz repeated it ~380 times.

    class _CountingProbe(VisualRecoveryProbe):
        """Counts _keypoints() calls so the memo can be asserted on behaviour, not on internals."""
        def __init__(self, **kw):
            super().__init__(**kw)
            self.kp_calls = 0

        def _keypoints(self, bgr):
            self.kp_calls += 1
            return super()._keypoints(bgr)

    # 11. The reference's SIFT is computed ONCE and reused; only the live frame's is paid per call.
    p11 = _CountingProbe()
    p11.update_reference(base, True)
    p11.match(zoomed)
    after_first = p11.kp_calls               # reference + live = 2
    p11.match(zoomed)
    p11.match(zoomed)
    after_three = p11.kp_calls               # + live only, twice = 4
    case(f"reference SIFT memoised: 1st match costs 2 keypoint passes, each later match costs 1 "
         f"(calls after 1={after_first}, after 3={after_three})",
         after_first == 2 and after_three == 4)

    # 12. A NEW reference invalidates the memo (otherwise we would match against a stale frame's features).
    p11.update_reference(_textured_image(seed=7), True)
    before_new = p11.kp_calls
    p11.match(zoomed)
    case(f"a new update_reference re-computes the reference features "
         f"(+{p11.kp_calls - before_new} passes, expected 2)", p11.kp_calls - before_new == 2)

    # 13. THE TRAP: a FEATURELESS reference yields (kp, None) legitimately. The memo flag is the tuple
    #     slot, so it must NOT recompute forever just because the descriptors are None.
    p13 = _CountingProbe()
    p13.update_reference(np.zeros((120, 160, 3), dtype=np.uint8), True)   # flat black -> no descriptors
    p13.match(base)
    blank_first = p13.kp_calls
    p13.match(base)
    blank_second = p13.kp_calls
    case(f"featureless reference (des=None) is still memoised, not recomputed forever "
         f"(after 1={blank_first}, after 2={blank_second})",
         p13._lkg_feats is not None and (blank_second - blank_first) <= 1)

    # 14. update_reference reports whether it actually STORED (run_explore keys its own memo off this).
    p14 = VisualRecoveryProbe()
    stored_no = p14.update_reference(base, False)
    stored_none = p14.update_reference(None, True)
    stored_yes = p14.update_reference(base, True)
    case("update_reference returns True only when it really cached a new reference",
         stored_no is False and stored_none is False and stored_yes is True)

    # ---- SESSION 52 — F_LKG selected by frame IDENTITY, not by tick freshness ---------------------------
    # `src` is a caller-supplied LABEL (run_explore resolves it from a short frame-id ring against the
    # plan's own frame_id) that must be visible for debugging but must NEVER influence matching.

    # (52-lkg-1) src is recorded and defaults safely; a tracked=False call touches neither _lkg nor _lkg_src.
    frameA, frameB, frameC = _textured_image(seed=21), _textured_image(seed=22), _textured_image(seed=23)
    p_lkg1 = VisualRecoveryProbe()
    p_lkg1.update_reference(frameA, True)
    case(f"(52-lkg-1a) default src is 'live' (src={p_lkg1._lkg_src!r})", p_lkg1._lkg_src == "live")
    p_lkg1.update_reference(frameB, True, src="slam:42")
    case(f"(52-lkg-1b) explicit src is stored verbatim (src={p_lkg1._lkg_src!r})",
         p_lkg1._lkg_src == "slam:42")
    p_lkg1.update_reference(frameC, False, src="slam:43")
    lkg_still_frameB = bool(np.array_equal(p_lkg1._lkg, frameB))
    case(f"(52-lkg-1c) tracked=False stores NEITHER frame nor src "
         f"(lkg_still_frameB={lkg_still_frameB}, src={p_lkg1._lkg_src!r})",
         lkg_still_frameB and p_lkg1._lkg_src == "slam:42")

    # (52-lkg-2) src never affects matching: two probes fed the IDENTICAL reference/live pair, differing
    # only in the src label, must agree on every match verdict field.
    p_lkg2_live = VisualRecoveryProbe()
    p_lkg2_live.update_reference(base, True, src="live")
    p_lkg2_slam = VisualRecoveryProbe()
    p_lkg2_slam.update_reference(base, True, src="slam:9")
    vm_lkg2_live = p_lkg2_live.match(zoomed)
    vm_lkg2_slam = p_lkg2_slam.match(zoomed)
    case("(52-lkg-2) src label never affects the match verdict (live vs slam:9 agree in full)",
         vm_lkg2_live.matched == vm_lkg2_slam.matched
         and vm_lkg2_live.inliers == vm_lkg2_slam.inliers
         and vm_lkg2_live.contained == vm_lkg2_slam.contained
         and vm_lkg2_live.planar_like == vm_lkg2_slam.planar_like
         and vm_lkg2_live.scale == vm_lkg2_slam.scale)

    # (52-lkg-3) the debug banner names the reference frame's src. cv2.putText draws to pixels (not
    # OCR-able), so intercept the text ARGUMENTS the same way the module composes them, capturing exactly
    # what _compose_debug hands to cv2.putText.
    p_lkg3 = VisualRecoveryProbe()
    p_lkg3.update_reference(base, True, src="slam:777")
    _put_text_calls = []
    _real_put_text = cv2.putText

    def _spy_put_text(img, text, *rest, **kw):
        _put_text_calls.append(text)
        return _real_put_text(img, text, *rest, **kw)

    cv2.putText = _spy_put_text
    try:
        vm_lkg3 = p_lkg3.match(zoomed, debug=True)
    finally:
        cv2.putText = _real_put_text
    banner_has_src = (vm_lkg3.debug_image is not None
                       and any("slam:777" in t for t in _put_text_calls))
    case(f"(52-lkg-3) debug banner names the reference src (lkg_src={p_lkg3._lkg_src!r})", banner_has_src)

    # ---- SESSION 56 F_LKG AGE-OUT — a tracked=False call (the caller's plan frame aged out of its ring)
    # must KEEP the previous reference untouched, not silently substitute whatever frame WAS live. Also
    # proves the memoised SIFT features on the kept reference are not churned by the no-op call.
    p56_1 = VisualRecoveryProbe()
    p56_1.update_reference(frameA, True)          # store frame A as the reference
    p56_1.match(frameB)                            # force the reference SIFT memo to be computed
    memo_before = p56_1._lkg_feats
    stored_56 = p56_1.update_reference(frameB, False)   # caller's age-out branch: tracked=False
    lkg_still_frameA = bool(np.array_equal(p56_1._lkg, frameA))
    memo_unchanged = p56_1._lkg_feats is memo_before
    case(f"(56-lkg-1) update_reference_false_keeps_previous: tracked=False returns False, keeps F_LKG "
         f"unchanged, and does NOT invalidate the reference SIFT memo "
         f"(stored={stored_56}, lkg_still_frameA={lkg_still_frameA}, memo_unchanged={memo_unchanged})",
         stored_56 is False and lkg_still_frameA and memo_unchanged)

    # ---- SESSION 57 SPREAD RATIO — direction-aware "closer" verdict from RANSAC-inlier spread, in
    # place of the noisy homography `scale` (Finding 3: same flight, same second, ~30 inliers, `scale`
    # read 1.44 -> 1.31 -> 0.53 -> 0.59 -> 0.67 -> 1.33 -> 1.85). ------------------------------------------

    ref57 = _textured_image(seed=7)
    h57, w57 = ref57.shape[:2]
    center57 = (w57 / 2.0, h57 / 2.0)

    # 57-1/2. zoom_in / zoom_out: a KNOWN-scale affine warp about the frame centre (no source resize --
    # cv2.warpAffine renders the magnified/shrunk content directly into a same-size canvas).
    zoom_factor = 1.6
    M_in = cv2.getRotationMatrix2D(center57, 0, zoom_factor)
    live_zoom_in = cv2.warpAffine(ref57, M_in, (w57, h57), borderMode=cv2.BORDER_REPLICATE)
    M_out = cv2.getRotationMatrix2D(center57, 0, 1.0 / zoom_factor)
    live_zoom_out = cv2.warpAffine(ref57, M_out, (w57, h57), borderMode=cv2.BORDER_REPLICATE)

    probe57_in = VisualRecoveryProbe()
    probe57_in.update_reference(ref57, True)
    vm57_in = probe57_in.match(live_zoom_in)
    case(f"(57-1) zoom_in -> matched, size_ratio>1.0, closer=LIVE "
         f"(matched={vm57_in.matched} size_ratio={vm57_in.size_ratio} closer={vm57_in.closer})",
         vm57_in.matched and vm57_in.size_ratio is not None and vm57_in.size_ratio > 1.0
         and vm57_in.closer == "LIVE")

    probe57_out = VisualRecoveryProbe()
    probe57_out.update_reference(ref57, True)
    vm57_out = probe57_out.match(live_zoom_out)
    case(f"(57-2) zoom_out -> matched, size_ratio<1.0, closer=LKG "
         f"(matched={vm57_out.matched} size_ratio={vm57_out.size_ratio} closer={vm57_out.closer})",
         vm57_out.matched and vm57_out.size_ratio is not None and vm57_out.size_ratio < 1.0
         and vm57_out.closer == "LKG")

    # 57-3. identical: reference matched against itself -> size_ratio ~1, closer=EQUAL.
    probe57_id = VisualRecoveryProbe()
    probe57_id.update_reference(ref57, True)
    vm57_id = probe57_id.match(ref57)
    case(f"(57-3) identical frame -> size_ratio in [0.95, 1.05], closer=EQUAL "
         f"(size_ratio={vm57_id.size_ratio} closer={vm57_id.closer})",
         vm57_id.size_ratio is not None and 0.95 <= vm57_id.size_ratio <= 1.05
         and vm57_id.closer == "EQUAL")

    # 57-4. unmatched_is_unknown: no correspondence at all -> size_ratio/spreads None, closer=UNKNOWN.
    probe57_un = VisualRecoveryProbe()
    probe57_un.update_reference(ref57, True)
    vm57_un = probe57_un.match(_textured_image(seed=123))
    case(f"(57-4) unmatched -> matched=False, size_ratio=None, closer=UNKNOWN, spreads None "
         f"(matched={vm57_un.matched} size_ratio={vm57_un.size_ratio} closer={vm57_un.closer} "
         f"spread_lkg={vm57_un.spread_lkg} spread_live={vm57_un.spread_live})",
         vm57_un.matched is False and vm57_un.size_ratio is None and vm57_un.closer == "UNKNOWN"
         and vm57_un.spread_lkg is None and vm57_un.spread_live is None)

    # 57-5. low_inliers_is_unknown: an absurdly high size_min_inliers forces UNKNOWN even on a clean
    # match, while size_ratio itself is still populated (weak CV must not fabricate certainty).
    probe57_lo = VisualRecoveryProbe(size_min_inliers=10_000)
    probe57_lo.update_reference(ref57, True)
    vm57_lo = probe57_lo.match(live_zoom_in)
    case(f"(57-5) size_min_inliers=10000 -> closer=UNKNOWN despite a normal match, size_ratio populated "
         f"(matched={vm57_lo.matched} closer={vm57_lo.closer} size_ratio={vm57_lo.size_ratio})",
         vm57_lo.matched and vm57_lo.closer == "UNKNOWN" and vm57_lo.size_ratio is not None)

    # 57-6. spread_none_on_degenerate: _inlier_spread returns None for <2 points.
    one_pt = np.array([[1.0, 2.0]])
    empty_pt = np.zeros((0, 2))
    case("(57-6) _inlier_spread(1 point)=None, _inlier_spread(empty)=None",
         VisualRecoveryProbe._inlier_spread(one_pt) is None
         and VisualRecoveryProbe._inlier_spread(empty_pt) is None)

    print(f"\n[self-test] {'ALL PASS' if ok else 'FAILURES PRESENT'}")
    return ok


def main():
    ap = argparse.ArgumentParser(description="CPU-only SIFT visual loss-recovery probe")
    ap.add_argument("--self-test", action="store_true", help="synthetic frame-pair validation, no hardware")
    args = ap.parse_args()
    if args.self_test:
        raise SystemExit(0 if run_self_test() else 1)
    ap.error("nothing to do: pass --self-test")


if __name__ == "__main__":
    main()
