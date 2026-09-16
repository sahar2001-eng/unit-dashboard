"""
engine.py -- rotation analysis for the grid localizer.

Same two-stage estimator as rotation_analyze.py:

    M1  global tape angle from image gradients (angle quadrupling, Scharr).
        Never fails, works at every angle, no state. Gives theta mod 90.
    M2  line fit along scan lines rotated to M1's angle.
        Refined theta plus the junction position. Allowed to fail.

then unwrap -> continuous heading -> alpha-beta filter.

This module is import-only; the dashboard calls analyze().
"""

import json
import math
import os
import shutil
import subprocess
import time

import cv2
import numpy as np

# --------------------------------------------------------------- calibration
MM_PER_PIXEL = 0.223
JUNCTION_SPACING_MM = 80.0

# ------------------------------------------------------------------ M2 config
SCAN_SPACING = 4
SCAN_HALF_LEN = 190
SCAN_HALF_SPAN = 150
THICK_LO, THICK_HI = 0.60, 1.40
MIN_POINTS = 8
MAD_ITERS = 3
MAX_FIT_MAD_PX = 5.0
MIN_THICK_PX, MAX_THICK_PX = 8, 60
MIN_GAP_PX = 12           # the mid-cell gap is about 23 px
GAP_TAPE_RATIO = 2.5      # tape either side must be this much longer than the gap
MINI_JUNCTION_OFFSET_MM = 40.0
MAX_STEP_MM = 30.0        # impossible-jump filter, as in the flight code

# ---------------------------------------------------------------- turn finder
TURN_RATE_ON = 6.0        # deg/s to be considered turning
TURN_MIN_S = 0.15         # shorter bursts are noise
TURN_MERGE_S = 0.70       # bridge short pauses inside one turn
TURN_MIN_DEG = 10.0       # ignore tiny wobbles

# --------------------------------------------------------------------- brand
BLUE = (185, 85, 51)        # BGR of #3355B9
BLUE_LT = (222, 168, 122)
AMBER = (60, 175, 245)
GREEN = (120, 200, 120)
RED = (70, 70, 235)
WHITE = (255, 255, 255)
GREY = (150, 150, 150)
BG = (28, 24, 20)

ENGINE_VERSION = "2026-09-06.2"   # bump when engine.py changes; shown in the
                                  # UI and terminal banner so a stale copy is
                                  # obvious rather than a mystery


def _sanitize(obj):
    """Recursively replace numpy scalar/array types with native Python ones.

    Belt-and-suspenders: every numeric value here should already be a plain
    float/int by the time it reaches this function, but a single missed
    numpy.float32 anywhere upstream crashes json.dump with a confusing
    error. Running the whole summary/series through this before writing
    means that class of bug can't reach the user again, even from code
    added later.
    """
    if isinstance(obj, dict):
        return {k: _sanitize(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_sanitize(v) for v in obj]
    if isinstance(obj, np.generic):      # any numpy scalar: float32, int64, bool_...
        return obj.item()
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    return obj


# ===========================================================================
# M1
# ===========================================================================
def m1_angle(gray, step=2):
    """Tape angle in degrees, in [-45, +45). Never fails.

    Gradient arrows on a grid point four ways, 90 deg apart, so they cancel
    if averaged. Multiplying every angle by 4 makes all four agree; then a
    plain vector sum works. Multiplying by 4 = squaring the (gx, gy) pair
    twice, which is integer arithmetic. One atan2 per frame.

    Scharr weights (3, 10, 3), not a plain difference: measured against the
    line fit, a 2-tap difference carries a systematic 4.5 deg error following
    sin(4*theta); Sobel 1.7 deg; Scharr 0.16 deg.
    """
    a = gray[::step, ::step].astype(np.int32)
    gx = (3 * (a[:-2, 2:] - a[:-2, :-2])
          + 10 * (a[1:-1, 2:] - a[1:-1, :-2])
          + 3 * (a[2:, 2:] - a[2:, :-2])) >> 2
    gy = (3 * (a[2:, :-2] - a[:-2, :-2])
          + 10 * (a[2:, 1:-1] - a[:-2, 1:-1])
          + 3 * (a[2:, 2:] - a[:-2, 2:])) >> 2
    gx = gx.astype(np.int64)
    gy = gy.astype(np.int64)
    p = gx * gx - gy * gy
    q = 2 * gx * gy
    re = p * p - q * q
    im = 2 * p * q
    ang = math.degrees(math.atan2(im.sum(), re.sum())) / 4.0
    return ((ang + 45.0) % 90.0) - 45.0


# ===========================================================================
# M2
# ===========================================================================
def _fit_robust(s, t, iters=MAD_ITERS):
    s = s.astype(np.float64)
    t = t.astype(np.float64)
    keep = np.ones(len(s), dtype=bool)
    for _ in range(iters + 1):
        if keep.sum() < MIN_POINTS:
            return None
        ss, tt = s[keep], t[keep]
        n = len(ss)
        den = n * (ss * ss).sum() - ss.sum() ** 2
        if abs(den) < 1e-9:
            return None
        b = (n * (ss * tt).sum() - ss.sum() * tt.sum()) / den
        a = (tt.sum() - b * ss.sum()) / n
        res = np.abs(t - (a + b * s))
        mad = np.median(res[keep])
        if mad < 1e-6:
            break
        new = res < max(3.0 * mad, 0.5)
        if (new == keep).all():
            break
        keep = new
    ss, tt = s[keep], t[keep]
    if len(ss) < MIN_POINTS:
        return None
    n = len(ss)
    den = n * (ss * ss).sum() - ss.sum() ** 2
    if abs(den) < 1e-9:
        return None
    b = (n * (ss * tt).sum() - ss.sum() * tt.sum()) / den
    a = (tt.sum() - b * ss.sum()) / n
    mad = float(np.median(np.abs(tt - (a + b * ss))))
    return a, b, mad, n


def _scan_family(mask, centre, u, n_vec):
    """Scan lines spaced along u, sampled across along n_vec.

    The rotated equivalent of walking down image columns. The image itself
    is never rotated -- only the direction we step in.
    """
    h, w = mask.shape
    cx, cy = centre
    s = np.arange(-SCAN_HALF_SPAN, SCAN_HALF_SPAN + 1, SCAN_SPACING, dtype=np.float32)
    t = np.arange(-SCAN_HALF_LEN, SCAN_HALF_LEN + 1, 1.0, dtype=np.float32)
    S, T = np.meshgrid(s, t, indexing="ij")
    px = cx + S * u[0] + T * n_vec[0]
    py = cy + S * u[1] + T * n_vec[1]
    inside = (px >= 0) & (px <= w - 1) & (py >= 0) & (py <= h - 1)
    samp = cv2.remap(mask, px, py, interpolation=cv2.INTER_NEAREST,
                     borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    bright = (samp > 127) & inside

    out_s, out_t, out_len = [], [], []
    for i in range(bright.shape[0]):
        row = bright[i]
        if not row.any():
            continue
        d = np.diff(np.concatenate(([0], row.view(np.int8), [0])))
        starts = np.flatnonzero(d == 1)
        ends = np.flatnonzero(d == -1)
        lens = ends - starts
        k = int(np.argmax(lens))
        L = int(lens[k])
        if L < MIN_THICK_PX or L > MAX_THICK_PX:
            continue
        out_s.append(s[i])
        out_t.append(t[0] + (starts[k] + ends[k] - 1) * 0.5)
        out_len.append(L)
    if not out_s:
        return None
    return np.array(out_s), np.array(out_t), np.array(out_len, dtype=np.float64)


def _find_gap(s_valid, spacing=SCAN_SPACING):
    """The mid-cell gap: a break in the tape with long tape either side.

    Between two junctions the horizontal tape continues but there is no
    vertical piece, so a full junction fix is impossible. The break in the
    middle of the cell is 40 mm from either junction, which is enough for X.

    The 2.5x rule is what separates the real mid-cell gap (long tape both
    sides) from the small gaps that sit either side of every junction.
    """
    if len(s_valid) < 4:
        return None
    s_valid = np.sort(s_valid)
    groups = []
    start = prev = s_valid[0]
    for v in s_valid[1:]:
        if v - prev > spacing * 1.5:
            groups.append((start, prev))
            start = v
        prev = v
    groups.append((start, prev))
    best = None
    for i in range(len(groups) - 1):
        a_lo, a_hi = groups[i]
        b_lo, b_hi = groups[i + 1]
        gap = b_lo - a_hi
        if gap < MIN_GAP_PX:
            continue
        if (a_hi - a_lo) < GAP_TAPE_RATIO * gap or (b_hi - b_lo) < GAP_TAPE_RATIO * gap:
            continue
        centre = 0.5 * (a_hi + b_lo)
        if best is None or gap > best[1]:
            best = (float(centre), float(gap))
    return best


def _thickness_filter(s, t, lens):
    med = float(np.median(lens))
    keep = (lens >= THICK_LO * med) & (lens <= THICK_HI * med)
    return s[keep], t[keep], med


def m2_fit(gray, theta_deg, centre):
    _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    r = math.radians(theta_deg)
    u = np.array([math.cos(r), math.sin(r)], dtype=np.float32)
    n_vec = np.array([-math.sin(r), math.cos(r)], dtype=np.float32)

    res = {"theta": theta_deg, "tier": 0, "junction": None, "gap": None,
           "xline": None, "yline": None, "mad_x": None, "mad_y": None,
           "thick": None}
    sx_valid = None

    fx = fy = None
    got = _scan_family(mask, centre, u, n_vec)
    if got is not None:
        s, t, lens = got
        s, t, thick = _thickness_filter(s, t, lens)
        if len(s) >= MIN_POINTS:
            f = _fit_robust(s, t)
            if f is not None and f[2] <= MAX_FIT_MAD_PX:
                fx = f
                sx_valid = s
                res["xline"] = (f[0], f[1])
                res["mad_x"] = f[2]
                res["thick"] = thick

    got = _scan_family(mask, centre, n_vec, u)
    if got is not None:
        s, t, lens = got
        s, t, thick = _thickness_filter(s, t, lens)
        if len(s) >= MIN_POINTS:
            f = _fit_robust(s, t)
            if f is not None and f[2] <= MAX_FIT_MAD_PX:
                fy = f
                res["yline"] = (f[0], f[1])
                res["mad_y"] = f[2]

    if fx is None and fy is None:
        return None

    corr = []
    if fx is not None:
        corr.append(math.degrees(math.atan(fx[1])))
    if fy is not None:
        corr.append(-math.degrees(math.atan(fy[1])))
    res["theta"] = theta_deg + sum(corr) / len(corr)

    if fx is not None and fy is not None:
        a1, b1 = fx[0], fx[1]
        a2, b2 = fy[0], fy[1]
        den = 1.0 - b1 * b2
        if abs(den) > 1e-9:
            alpha = (a2 + b2 * a1) / den
            beta = a1 + b1 * alpha
            res["junction"] = (alpha, beta)
            res["tier"] = 1
            return res

    # no crossing bar: try the mid-cell gap instead (tier 2)
    if fx is not None and sx_valid is not None:
        g = _find_gap(sx_valid)
        if g is not None:
            res["gap"] = g[0]
            res["gap_width"] = g[1]
            res["tier"] = 2
            return res

    res["tier"] = 3
    return res


# ===========================================================================
# temporal
# ===========================================================================
class Unwrapper:
    """theta lives on a loop repeating every 90 deg. Between frames the robot
    cannot really turn more than 45, so a bigger jump means it wrapped."""

    def __init__(self):
        self.prev = None
        self.total = 0.0

    def update(self, raw):
        if self.prev is None:
            self.prev = raw
            return 0.0
        d = raw - self.prev
        while d > 45.0:
            d -= 90.0
        while d < -45.0:
            d += 90.0
        self.total += d
        self.prev = raw
        return self.total


class AlphaBeta:
    """Two-state tracker on (heading, rate)."""

    def __init__(self, dt, alpha=0.45, beta=0.08, max_step=25.0):
        self.dt, self.alpha, self.beta, self.max_step = dt, alpha, beta, max_step
        self.x = None
        self.v = 0.0
        self.rejected = 0

    def update(self, z):
        if self.x is None:
            self.x = z
            return z
        xp = self.x + self.v * self.dt
        r = z - xp
        if abs(r) > self.max_step:
            self.rejected += 1
            self.x = xp
            return self.x
        self.x = xp + self.alpha * r
        self.v += self.beta * r / self.dt
        return self.x

    @property
    def rate(self):
        return self.v


# ===========================================================================
# drawing
# ===========================================================================
def _line_pts(centre, u, n_vec, a, b, half=400):
    out = []
    for al in (-half, half):
        be = a + b * al
        out.append((int(round(centre[0] + al * u[0] + be * n_vec[0])),
                    int(round(centre[1] + al * u[1] + be * n_vec[1]))))
    return out[0], out[1]


def _gauge(frame, heading, rate):
    h, w = frame.shape[:2]
    ox, oy, R = w - 46, 46, 33
    cv2.circle(frame, (ox, oy), R + 6, BG, -1)
    cv2.circle(frame, (ox, oy), R, (90, 80, 70), 1, cv2.LINE_AA)
    for k in range(4):
        a = math.radians(90 * k)
        cv2.line(frame,
                 (int(ox + (R - 4) * math.sin(a)), int(oy - (R - 4) * math.cos(a))),
                 (int(ox + R * math.sin(a)), int(oy - R * math.cos(a))),
                 (120, 110, 100), 1, cv2.LINE_AA)
    span = max(-359.0, min(359.0, heading))
    if abs(span) > 0.5:
        cv2.ellipse(frame, (ox, oy), (R - 14, R - 14), -90.0, 0.0, span,
                    AMBER, 1, cv2.LINE_AA)
    cv2.arrowedLine(frame, (ox, oy), (ox, oy - (R - 6)), WHITE, 1,
                    cv2.LINE_AA, tipLength=0.22)
    a = math.radians(heading)
    cv2.arrowedLine(frame, (ox, oy),
                    (int(ox + (R - 6) * math.sin(a)), int(oy - (R - 6) * math.cos(a))),
                    AMBER, 2, cv2.LINE_AA, tipLength=0.25)
    cv2.circle(frame, (ox, oy), 2, WHITE, -1, cv2.LINE_AA)
    cv2.putText(frame, "%+.1f" % heading, (ox - 26, oy + R + 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, AMBER, 1, cv2.LINE_AA)
    cv2.putText(frame, "%+.0f deg/s" % rate, (ox - 28, oy + R + 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.32, GREY, 1, cv2.LINE_AA)


def annotate(frame, centre, theta, fit, heading, rate, x_next, y_off, frame_no):
    h, w = frame.shape[:2]
    r = math.radians(theta)
    u = np.array([math.cos(r), math.sin(r)])
    n_vec = np.array([-math.sin(r), math.cos(r)])
    cx, cy = centre

    if fit is not None:
        if fit["xline"] is not None:
            p, q = _line_pts(centre, u, n_vec, *fit["xline"])
            cv2.line(frame, p, q, GREEN, 1, cv2.LINE_AA)
        if fit["yline"] is not None:
            a2, b2 = fit["yline"]
            pts = []
            for be in (-400, 400):
                al = a2 + b2 * be
                pts.append((int(round(cx + al * u[0] + be * n_vec[0])),
                            int(round(cy + al * u[1] + be * n_vec[1]))))
            cv2.line(frame, pts[0], pts[1], BLUE_LT, 1, cv2.LINE_AA)
        if fit["junction"] is not None:
            al, be = fit["junction"]
            J = (int(round(cx + al * u[0] + be * n_vec[0])),
                 int(round(cy + al * u[1] + be * n_vec[1])))
            cv2.drawMarker(frame, J, RED, cv2.MARKER_CROSS, 14, 1, cv2.LINE_AA)
            cv2.circle(frame, J, 8, RED, 1, cv2.LINE_AA)
        elif fit["gap"] is not None:
            # mid-cell gap: mark the break, and the junction it implies
            al = fit["gap"]
            be = fit["xline"][0] + fit["xline"][1] * al
            G = (int(round(cx + al * u[0] + be * n_vec[0])),
                 int(round(cy + al * u[1] + be * n_vec[1])))
            cv2.drawMarker(frame, G, AMBER, cv2.MARKER_TILTED_CROSS, 12, 1, cv2.LINE_AA)
            cv2.circle(frame, G, 7, AMBER, 1, cv2.LINE_AA)

    _gauge(frame, heading, rate)

    tier = fit["tier"] if fit else 0
    tier_txt = {0: "angle only", 1: "junction",
                2: "mini junction", 3: "single bar"}.get(tier, "?")
    lines = ["turned %+7.2f deg" % heading,
             "tape   %+7.2f deg  (%s)" % (theta, tier_txt),
             "X %s   Y %s" % (
                 "  --  " if x_next is None else "%5.1f" % x_next,
                 "  --  " if y_off is None else "%+5.1f" % y_off)]
    y0 = h - 8 - 14 * (len(lines) - 1)
    cv2.rectangle(frame, (4, y0 - 14), (212, h - 2), BG, -1)
    for i, l in enumerate(lines):
        cv2.putText(frame, l, (9, y0 + 14 * i - 1),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, WHITE, 1, cv2.LINE_AA)
    cv2.putText(frame, "#%d" % frame_no, (w - 52, h - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.36, GREY, 1, cv2.LINE_AA)
    return frame


# ===========================================================================
# analytics
# ===========================================================================
def _jitter(x, w=9):
    """Deviation from local smooth motion -- the repeatability of the estimate."""
    x = np.asarray(x, dtype=float)
    if len(x) < w + 2:
        return None
    res = []
    idx = np.arange(w)
    for i in range(w // 2, len(x) - w // 2):
        seg = x[i - w // 2:i + w // 2 + 1]
        res.append(seg[w // 2] - np.polyval(np.polyfit(idx, seg, 2), w // 2))
    return float(np.std(res))


def _find_turns(t, heading, rate, fps):
    """Segment the clip into distinct turning manoeuvres."""
    rate = np.asarray(rate, float)
    k = max(1, int(round(fps * 0.1)))
    sm = np.convolve(np.abs(rate), np.ones(k) / k, mode="same")
    on = sm > TURN_RATE_ON
    segs = []
    i = 0
    n = len(on)
    while i < n:
        if not on[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and on[j + 1]:
            j += 1
        segs.append([i, j])
        i = j + 1
    merged = []
    for s in segs:
        if merged and (s[0] - merged[-1][1]) / fps < TURN_MERGE_S:
            merged[-1][1] = s[1]
        else:
            merged.append(s)
    out = []
    for a, b in merged:
        if (b - a) / fps < TURN_MIN_S:
            continue
        deg = heading[b] - heading[a]
        if abs(deg) < TURN_MIN_DEG:
            continue
        seg_rate = rate[a:b + 1]
        quarter = int(round(deg / 90.0))
        near = abs(deg - quarter * 90.0) < 25.0 and quarter != 0
        out.append({
            "start_frame": int(a), "end_frame": int(b),
            "start_s": round(float(t[a]), 3), "end_s": round(float(t[b]), 3),
            "duration_s": round(float(t[b] - t[a]), 3),
            "degrees": round(float(deg), 2),
            "peak_rate": round(float(seg_rate[np.argmax(np.abs(seg_rate))]), 1),
            "quarter_turn": bool(near),
            "nearest_90": quarter * 90 if near else None,
            "error_vs_90": (round(float(deg - quarter * 90.0), 2)
                            if near else None),
        })
    return out


# ===========================================================================
# main entry point
# ===========================================================================
def _h264(src, dst):
    """Re-encode to H.264 so browsers can play it. Uses ffmpeg if available
    (system, or the copy that ships with imageio-ffmpeg)."""
    exe = shutil.which("ffmpeg")
    if exe is None:
        try:
            from imageio_ffmpeg import get_ffmpeg_exe
            exe = get_ffmpeg_exe()
        except Exception:
            return False
    try:
        subprocess.run([exe, "-y", "-loglevel", "error", "-i", src,
                        "-c:v", "libx264", "-pix_fmt", "yuv420p",
                        "-preset", "veryfast", "-crf", "20",
                        "-movflags", "+faststart", dst],
                       check=True, timeout=900)
        return os.path.exists(dst) and os.path.getsize(dst) > 0
    except Exception:
        return False


def analyze(video_path, out_dir, step=2, scale=3, smooth=True,
            progress=None):
    """Run the estimator over a video.

    Writes annotated.mp4, frames.csv and summary.json into out_dir and
    returns the summary dict. `progress` is called with (done, total).
    """
    os.makedirs(out_dir, exist_ok=True)
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise RuntimeError("could not open the video file")

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 0
    centre = (w / 2.0, h / 2.0)
    sc = max(1, int(scale))

    raw_mp4 = os.path.join(out_dir, "_raw.mp4")
    writer = cv2.VideoWriter(raw_mp4, cv2.VideoWriter_fourcc(*"mp4v"),
                             fps, (w * sc, h * sc))

    unw = Unwrapper()
    ab = AlphaBeta(1.0 / fps) if smooth else None

    prev_x = None
    travel = 0.0
    jump_rejects = 0
    crossings = []
    rows = []
    t0 = time.time()
    n = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        th1 = m1_angle(gray, step)
        fit = m2_fit(gray, th1, centre)
        theta = fit["theta"] if fit else th1
        otsu, _ = cv2.threshold(gray, 0, 255,
                                cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        sharp = float(cv2.Laplacian(gray, cv2.CV_32F).var())

        heading_raw = unw.update(theta)
        heading = ab.update(heading_raw) if ab else heading_raw
        rate = ab.rate if ab else 0.0

        # ---- position, in the flight code's convention:
        #      X = distance to the next junction ahead, 0..80 mm, counting
        #      down as the robot drives and resetting at each junction.
        x_next = None       # mm to the next junction
        y_off = None        # lateral offset from the tape centreline, mm
        alpha_j = None
        if fit and fit["junction"] is not None:
            alpha_j, beta = fit["junction"]
            y_off = -beta * MM_PER_PIXEL
        elif fit and fit["gap"] is not None:
            # the gap centre sits 40 mm from the junction ahead
            alpha_j = fit["gap"] + MINI_JUNCTION_OFFSET_MM / MM_PER_PIXEL
            y_off = -fit["xline"][0] * MM_PER_PIXEL
        elif fit and fit["xline"] is not None:
            y_off = -fit["xline"][0] * MM_PER_PIXEL

        if alpha_j is not None:
            x_next = (alpha_j * MM_PER_PIXEL) % JUNCTION_SPACING_MM
            if x_next <= 0:
                x_next += JUNCTION_SPACING_MM

        # ---- impossible-jump filter, compared modulo the cell pitch so a
        #      normal wrap is not flagged
        rejected_here = False
        if x_next is not None and prev_x is not None:
            d = (prev_x - x_next) % JUNCTION_SPACING_MM
            if d > JUNCTION_SPACING_MM / 2:
                d -= JUNCTION_SPACING_MM
            if abs(d) > MAX_STEP_MM:
                rejected_here = True
                jump_rejects += 1
            else:
                travel += d
                if d > 0 and (prev_x - d) < 0.0:
                    pass
                if x_next > prev_x + 1.0:      # counted down past a junction
                    crossings.append(n)
        if x_next is not None and not rejected_here:
            prev_x = x_next

        vis = annotate(frame.copy(), centre, theta, fit, heading, rate,
                       x_next, y_off, n)
        if sc > 1:
            vis = cv2.resize(vis, (w * sc, h * sc), interpolation=cv2.INTER_NEAREST)
        writer.write(vis)

        rows.append({
            "frame": n, "t": n / fps,
            "x_next": x_next, "y_off": y_off, "travel": travel,
            "otsu": float(otsu), "sharp": sharp,
            "rejected": rejected_here,
            "theta_m1": th1, "theta": theta,
            "heading_raw": heading_raw, "heading": heading, "rate": rate,
            "tier": fit["tier"] if fit else 0,
            "x_mm": x_next, "y_mm": y_off,
            "mad": (fit["mad_x"] if fit and fit["mad_x"] is not None else
                    (fit["mad_y"] if fit and fit["mad_y"] is not None else None)),
        })
        n += 1
        if progress and (n % 10 == 0):
            progress(n, total or n)

    cap.release()
    writer.release()

    # ---- browser-playable copy
    mp4 = os.path.join(out_dir, "annotated.mp4")
    if _h264(raw_mp4, mp4):
        os.remove(raw_mp4)
        playable = True
    else:
        os.replace(raw_mp4, mp4)
        playable = False

    # ---- csv
    with open(os.path.join(out_dir, "frames.csv"), "w") as f:
        f.write("frame,t_s,theta_m1,theta,heading_raw,heading,rate_dps,tier,"
                "x_to_next_mm,y_offset_mm,travel_mm,otsu,sharpness,"
                "jump_rejected,fit_residual_px\n")
        for r in rows:
            f.write("%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%d,%s,%s,%.2f,%.0f,%.1f,%d,%s\n" % (
                r["frame"], r["t"], r["theta_m1"], r["theta"],
                r["heading_raw"], r["heading"], r["rate"], r["tier"],
                "" if r["x_next"] is None else "%.3f" % r["x_next"],
                "" if r["y_off"] is None else "%.3f" % r["y_off"],
                r["travel"], r["otsu"], r["sharp"], 1 if r["rejected"] else 0,
                "" if r["mad"] is None else "%.3f" % r["mad"]))

    # ---- summary
    t = np.array([r["t"] for r in rows])
    heading = np.array([r["heading"] for r in rows])
    heading_raw = np.array([r["heading_raw"] for r in rows])
    rate = np.array([r["rate"] for r in rows])
    tier = np.array([r["tier"] for r in rows])
    otsu = np.array([r["otsu"] for r in rows])
    sharp = np.array([r["sharp"] for r in rows])
    travel_arr = np.array([r["travel"] for r in rows])
    xnext = np.array([np.nan if r["x_next"] is None else r["x_next"] for r in rows])
    yoff = np.array([np.nan if r["y_off"] is None else r["y_off"] for r in rows])
    mad = np.array([r["mad"] for r in rows if r["mad"] is not None])
    theta_all = np.array([r["theta"] for r in rows])

    # how well the estimator holds up across the whole angle range
    bins = np.arange(-45, 46, 15)
    cov = []
    for i in range(len(bins) - 1):
        sel = (theta_all >= bins[i]) & (theta_all < bins[i + 1])
        cov.append({
            "from": int(bins[i]), "to": int(bins[i + 1]),
            "frames": int(sel.sum()),
            "junction_pct": round(100.0 * float((tier[sel] == 1).sum()) /
                                  max(int(sel.sum()), 1), 1),
            "fix_pct": round(100.0 * float(((tier[sel] == 1) |
                                            (tier[sel] == 2)).sum()) /
                             max(int(sel.sum()), 1), 1),
        })

    summary = {
        "frames": n,
        "fps": round(float(fps), 3),
        "duration_s": round(float(n / fps), 2),
        "width": w, "height": h,
        "scale": sc, "m1_step": step, "smoothing": bool(smooth),
        "browser_playable": playable,
        "processing_s": round(time.time() - t0, 1),
        "ms_per_frame": round(1000.0 * (time.time() - t0) / max(n, 1), 1),

        "total_rotation_deg": round(float(unw.total), 2),
        "heading_min": round(float(heading.min()), 2) if n else 0,
        "heading_max": round(float(heading.max()), 2) if n else 0,
        "peak_rate_dps": round(float(np.abs(rate).max()), 1) if n else 0,

        "junction_pct": round(100.0 * float((tier == 1).sum()) / max(n, 1), 1),
        "mini_junction_pct": round(100.0 * float((tier == 2).sum()) / max(n, 1), 1),
        "single_bar_pct": round(100.0 * float((tier == 3).sum()) / max(n, 1), 1),
        "m1_only_pct": round(100.0 * float((tier == 0).sum()) / max(n, 1), 1),
        "x_available_pct": round(100.0 * float(np.isfinite(xnext).sum()) / max(n, 1), 1),
        "y_available_pct": round(100.0 * float(np.isfinite(yoff).sum()) / max(n, 1), 1),
        "heading_available_pct": 100.0,

        "travel_mm": round(float(travel_arr[-1]), 1) if n else 0.0,
        "junction_crossings": len(crossings),
        "mean_speed_mms": round(float(abs(travel_arr[-1]) / max(n / fps, 1e-6)), 1) if n else 0.0,
        "jumps_rejected": jump_rejects,
        "mode_switches": int((np.diff(tier) != 0).sum()) if n > 1 else 0,

        "otsu_min": int(otsu.min()) if n else 0,
        "otsu_max": int(otsu.max()) if n else 0,
        "otsu_swing": int(otsu.max() - otsu.min()) if n else 0,
        "sharpness_median": round(float(np.median(sharp)), 1) if n else 0,
        "sharpness_worst": round(float(sharp.min()), 1) if n else 0,
        "lateral_rms_mm": (round(float(np.sqrt(np.nanmean(yoff ** 2))), 2)
                           if np.isfinite(yoff).any() else None),

        "median_residual_px": round(float(np.median(mad)), 3) if len(mad) else None,
        "p95_residual_px": round(float(np.percentile(mad, 95)), 3) if len(mad) else None,
        "jitter_raw_deg": round(_jitter(heading_raw), 4) if n > 12 else None,
        "jitter_smoothed_deg": round(_jitter(heading), 4) if n > 12 else None,
        "outliers_rejected": ab.rejected if ab else 0,

        "coverage": cov,
        "turns": _find_turns(t, heading, rate, fps),
    }

    if np.isfinite(yoff).any():
        summary["y_range_mm"] = [round(float(np.nanmin(yoff)), 1),
                                 round(float(np.nanmax(yoff)), 1)]

    summary["engine_version"] = ENGINE_VERSION
    summary = _sanitize(summary)          # neutralize any stray numpy type

    with open(os.path.join(out_dir, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)

    # compact series for the browser (no need to ship every field)
    series = {
        "t": [round(float(x), 3) for x in t],
        "heading": [round(float(x), 3) for x in heading],
        "heading_raw": [round(float(x), 3) for x in heading_raw],
        "rate": [round(float(x), 2) for x in rate],
        "theta": [round(float(x), 3) for x in theta_all],
        "tier": [int(x) for x in tier],
        "x": [None if r["x_next"] is None else round(r["x_next"], 2) for r in rows],
        "y": [None if r["y_off"] is None else round(r["y_off"], 2) for r in rows],
        "travel": [round(r["travel"], 2) for r in rows],
        "otsu": [int(r["otsu"]) for r in rows],
        "sharp": [round(r["sharp"], 1) for r in rows],
        "res": [None if r["mad"] is None else round(r["mad"], 3) for r in rows],
        "crossings": crossings,
    }
    series = _sanitize(series)            # same safety net

    with open(os.path.join(out_dir, "series.json"), "w") as f:
        json.dump(series, f, separators=(",", ":"))

    return summary
