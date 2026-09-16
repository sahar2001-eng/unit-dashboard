#!/usr/bin/env python3
"""
rotation_analyze.py -- offline rotation analysis for the grid localizer.

Runs the two-stage rotation estimator over a recorded video and writes an
annotated video plus a CSV of every frame.

    STAGE M1  global tape angle from image gradients (angle quadrupling).
              Never fails, works at every angle, ~0.15 deg, no state.
              Gives theta mod 90 only.

    STAGE M2  line fit along scan lines ROTATED to M1's angle.
              Gives a refined theta plus the junction position (X, Y).
              Allowed to fail; when it does, M1's angle is still published.

Then: unwrap M1 across frames -> continuous heading + turn progress counter.

Usage
    python rotation_analyze.py input.mp4
    python rotation_analyze.py input.mp4 -o out.mp4 --csv out.csv
    python rotation_analyze.py input.mp4 --step 2 --preview

Requires: opencv-python, numpy
    pip install opencv-python numpy
"""

import argparse
import math
import os
import sys

import cv2
import numpy as np

# --------------------------------------------------------------- calibration
MM_PER_PIXEL = 0.223        # from the ruler test: 20 mm slide = 89.6 px
JUNCTION_SPACING_MM = 80.0  # grid pitch

# ------------------------------------------------------------------ M1 config
M1_STEP = 2                 # look at every Nth pixel. 1/2/4 -> 0.135/0.146/0.200 deg

# ------------------------------------------------------------------ M2 config
SCAN_SPACING = 4            # px between scan lines, along the tape
SCAN_HALF_LEN = 190         # px, how far each scan line reaches from centre
SCAN_HALF_SPAN = 150        # px, how far along the tape we place scan lines
THICK_LO = 0.60             # keep runs between these multiples of the
THICK_HI = 1.40             # measured tape thickness
MIN_POINTS = 8              # fewest points that can make a line
MAD_ITERS = 3               # outlier rejection rounds
MAX_FIT_MAD_PX = 5.0        # residual gate: real bar ~0.2 px, noise ~45 px
MIN_THICK_PX = 8            # a run thinner than this is not our tape
MAX_THICK_PX = 60           # nor thicker than this

# ------------------------------------------------------------------- drawing
COL_XTAPE = (80, 220, 80)    # BGR, green
COL_YTAPE = (255, 150, 60)   # blue
COL_JUNC = (60, 60, 255)     # red
COL_CAM = (255, 255, 255)    # white
COL_GRID = (0, 215, 255)     # yellow
COL_TEXT = (255, 255, 255)
COL_BG = (0, 0, 0)


# ===========================================================================
# STAGE M1 -- global tape angle
# ===========================================================================
def m1_angle(gray, step=M1_STEP):
    """Tape angle in degrees, in [-45, +45).  Never fails.

    Every pixel gets a gradient arrow.  On a grid those arrows point in four
    directions 90 deg apart, so they cancel if you average them.  Multiplying
    every angle by 4 makes all four agree; then a plain vector sum works.

    Multiplying an angle by 4 = squaring the (gx, gy) pair twice, which is
    pure integer arithmetic.  One atan2 per frame, at the very end.

    The gradient uses SCHARR weights (3, 10, 3), not a plain left-minus-right
    difference.  This matters more than it looks: measured against the line
    fit on rotation1.mp4, a plain 2-tap difference has a systematic error of
    4.5 deg that follows sin(4*theta), Sobel has 1.7 deg, and Scharr has
    0.16 deg.  Scharr's weights are chosen precisely to make the operator's
    response the same in every direction, which is exactly what an angle
    measurement needs.
    """
    a = gray[::step, ::step].astype(np.int32)

    #      -3  0  3            -3 -10 -3
    # gx = -10 0 10      gy =   0   0  0
    #      -3  0  3             3  10  3
    gx = (3 * (a[:-2, 2:] - a[:-2, :-2])
          + 10 * (a[1:-1, 2:] - a[1:-1, :-2])
          + 3 * (a[2:, 2:] - a[2:, :-2])) >> 2
    gy = (3 * (a[2:, :-2] - a[:-2, :-2])
          + 10 * (a[2:, 1:-1] - a[:-2, 1:-1])
          + 3 * (a[2:, 2:] - a[:-2, 2:])) >> 2
    # the >> 2 keeps the fourth powers inside 64 bits; on the camera it also
    # keeps them inside the accumulator you choose there.

    gx = gx.astype(np.int64)
    gy = gy.astype(np.int64)

    p = gx * gx - gy * gy                    # angle x2
    q = 2 * gx * gy

    re = p * p - q * q                       # angle x4
    im = 2 * p * q

    ang = math.degrees(math.atan2(im.sum(), re.sum())) / 4.0
    return ((ang + 45.0) % 90.0) - 45.0


# ===========================================================================
# STAGE M2 -- line fit along rotated scan lines
# ===========================================================================
def _fit_robust(s, t, iters=MAD_ITERS):
    """Least squares t = a + b*s with MAD outlier rejection.

    Returns (a, b, mad, n) or None.
    """
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
    n = len(ss)
    if n < MIN_POINTS:
        return None
    den = n * (ss * ss).sum() - ss.sum() ** 2
    if abs(den) < 1e-9:
        return None
    b = (n * (ss * tt).sum() - ss.sum() * tt.sum()) / den
    a = (tt.sum() - b * ss.sum()) / n
    mad = float(np.median(np.abs(tt - (a + b * ss))))
    return a, b, mad, n


def _scan_family(mask, centre, u, n_vec):
    """Walk scan lines perpendicular to a tape family and return the centre
    of the tape run found on each one.

    The scan lines are spaced along `u` (the tape direction) and each one is
    sampled along `n_vec` (across the tape).  This is the rotated equivalent
    of walking down image columns -- the image itself is never rotated.

    Returns (s, t, thickness) arrays in the rotated (u, n) frame.
    """
    h, w = mask.shape
    cx, cy = centre

    s = np.arange(-SCAN_HALF_SPAN, SCAN_HALF_SPAN + 1, SCAN_SPACING, dtype=np.float32)
    t = np.arange(-SCAN_HALF_LEN, SCAN_HALF_LEN + 1, 1.0, dtype=np.float32)
    S, T = np.meshgrid(s, t, indexing="ij")            # (nlines, nsamples)

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
        # longest contiguous run of bright samples on this scan line
        d = np.diff(np.concatenate(([0], row.view(np.int8), [0])))
        starts = np.flatnonzero(d == 1)
        ends = np.flatnonzero(d == -1)
        lens = ends - starts
        k = int(np.argmax(lens))
        L = int(lens[k])
        if L < MIN_THICK_PX or L > MAX_THICK_PX:
            continue
        centre_idx = (starts[k] + ends[k] - 1) * 0.5
        out_s.append(s[i])
        out_t.append(t[0] + centre_idx)
        out_len.append(L)

    if not out_s:
        return None
    return (np.array(out_s), np.array(out_t), np.array(out_len, dtype=np.float64))


def _thickness_filter(s, t, lens):
    """Keep only runs whose length agrees with the most common one."""
    med = float(np.median(lens))
    keep = (lens >= THICK_LO * med) & (lens <= THICK_HI * med)
    return s[keep], t[keep], med


def m2_fit(gray, theta_deg, centre):
    """Rotated line fit.  Returns a dict, or None if it cannot fit anything.

    Coordinates: a point is  C + alpha*u + beta*n  where u points along the
    X-tape and n across it.
    """
    _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    r = math.radians(theta_deg)
    u = np.array([math.cos(r), math.sin(r)], dtype=np.float32)
    n_vec = np.array([-math.sin(r), math.cos(r)], dtype=np.float32)

    res = {"theta": theta_deg, "tier": 0, "junction": None,
           "xline": None, "yline": None, "mad_x": None, "mad_y": None}

    # ---- X-tape: scan lines spaced along u, sampled along n
    fx = None
    got = _scan_family(mask, centre, u, n_vec)
    if got is not None:
        s, t, lens = got
        s, t, thick = _thickness_filter(s, t, lens)
        if len(s) >= MIN_POINTS:
            f = _fit_robust(s, t)
            if f is not None and f[2] <= MAX_FIT_MAD_PX:
                fx = f
                res["xline"] = (f[0], f[1])
                res["mad_x"] = f[2]

    # ---- Y-tape: scan lines spaced along n, sampled along u
    fy = None
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

    # ---- refined angle: each family votes on the correction to M1's angle
    corr = []
    if fx is not None:
        corr.append(math.degrees(math.atan(fx[1])))
    if fy is not None:
        corr.append(-math.degrees(math.atan(fy[1])))
    res["theta"] = theta_deg + sum(corr) / len(corr)

    # ---- junction = intersection of the two fitted lines
    if fx is not None and fy is not None:
        a1, b1 = fx[0], fx[1]        # beta  = a1 + b1*alpha
        a2, b2 = fy[0], fy[1]        # alpha = a2 + b2*beta
        den = 1.0 - b1 * b2
        if abs(den) > 1e-9:
            alpha = (a2 + b2 * a1) / den
            beta = a1 + b1 * alpha
            res["junction"] = (alpha, beta)
            res["tier"] = 1
        else:
            res["tier"] = 3
    else:
        res["tier"] = 3
    return res


# ===========================================================================
# unwrapping -- turning "theta mod 90" into a continuous heading
# ===========================================================================
class Unwrapper:
    """The measured angle lives on a loop that repeats every 90 degrees.
    Between two frames the robot cannot really have turned more than 45, so a
    bigger apparent jump means the value wrapped.  Undo it and accumulate.
    """

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
    """Light two-state tracker on (heading, rate).

    Predicts where the heading should be from the last rate, compares with the
    measurement, and moves part of the way toward it.  Smooths the residual
    jitter without lagging a real turn, and rejects single-frame outliers.
    """

    def __init__(self, dt, alpha=0.45, beta=0.08, max_step=25.0):
        self.dt = dt
        self.alpha = alpha
        self.beta = beta
        self.max_step = max_step
        self.x = None
        self.v = 0.0
        self.rejected = 0

    def update(self, z):
        if self.x is None:
            self.x = z
            return z
        xp = self.x + self.v * self.dt          # predict
        r = z - xp                              # innovation
        if abs(r) > self.max_step:              # impossible jump -> coast
            self.rejected += 1
            self.x = xp
            return self.x
        self.x = xp + self.alpha * r            # correct
        self.v += self.beta * r / self.dt
        return self.x

    @property
    def rate(self):
        return self.v


# ===========================================================================
# drawing
# ===========================================================================
def _line_pts(centre, u, n_vec, a, b, half=400):
    """Endpoints, in image coords, of the line  beta = a + b*alpha."""
    out = []
    for al in (-half, half):
        be = a + b * al
        out.append((centre[0] + al * u[0] + be * n_vec[0],
                    centre[1] + al * u[1] + be * n_vec[1]))
    return (int(round(out[0][0])), int(round(out[0][1]))), \
           (int(round(out[1][0])), int(round(out[1][1])))


def annotate(frame, centre, theta, fit, heading, rate, progress, xy_mm, frame_no):
    h, w = frame.shape[:2]
    r = math.radians(theta)
    u = np.array([math.cos(r), math.sin(r)])
    n_vec = np.array([-math.sin(r), math.cos(r)])
    cx, cy = centre

    if fit is not None:
        if fit["xline"] is not None:
            p, q = _line_pts(centre, u, n_vec, *fit["xline"])
            cv2.line(frame, p, q, COL_XTAPE, 1, cv2.LINE_AA)
        if fit["yline"] is not None:
            a2, b2 = fit["yline"]
            pts = []
            for be in (-400, 400):
                al = a2 + b2 * be
                pts.append((int(round(cx + al * u[0] + be * n_vec[0])),
                            int(round(cy + al * u[1] + be * n_vec[1]))))
            cv2.line(frame, pts[0], pts[1], COL_YTAPE, 1, cv2.LINE_AA)
        if fit["junction"] is not None:
            al, be = fit["junction"]
            jx = cx + al * u[0] + be * n_vec[0]
            jy = cy + al * u[1] + be * n_vec[1]
            J = (int(round(jx)), int(round(jy)))
            cv2.drawMarker(frame, J, COL_JUNC, cv2.MARKER_CROSS, 14, 1, cv2.LINE_AA)
            cv2.circle(frame, J, 8, COL_JUNC, 1, cv2.LINE_AA)

    _draw_gauge(frame, heading, rate)

    tier = fit["tier"] if fit else 0
    tier_txt = {0: "M1 only", 1: "junction", 3: "single bar"}.get(tier, "?")
    lines = ["turned %+7.2f deg" % heading,
             "tape   %+7.2f deg  (%s)" % (theta, tier_txt)]
    if xy_mm is not None:
        lines.append("X %+6.1f   Y %+6.1f mm" % xy_mm)
    else:
        lines.append("X    --     Y    --")

    y0 = h - 8 - 14 * (len(lines) - 1)
    cv2.rectangle(frame, (4, y0 - 14), (4 + 208, h - 2), COL_BG, -1)
    for i, l in enumerate(lines):
        cv2.putText(frame, l, (9, y0 + 14 * i - 1),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, COL_TEXT, 1, cv2.LINE_AA)
    cv2.putText(frame, "#%d" % frame_no, (w - 52, h - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.36, (150, 150, 150), 1, cv2.LINE_AA)
    return frame


def _draw_gauge(frame, heading, rate):
    """Turn gauge, top-right.  White = heading at the start of the clip,
    yellow = heading now, arc between them = how far the robot has turned.
    Drawn away from the image so it never sits on top of the junction."""
    h, w = frame.shape[:2]
    ox, oy, R = w - 46, 46, 33

    cv2.circle(frame, (ox, oy), R + 5, COL_BG, -1)
    cv2.circle(frame, (ox, oy), R, (80, 80, 80), 1, cv2.LINE_AA)
    for k in range(4):                                   # 90 deg ticks
        a = math.radians(90 * k)
        cv2.line(frame,
                 (int(ox + (R - 4) * math.sin(a)), int(oy - (R - 4) * math.cos(a))),
                 (int(ox + R * math.sin(a)), int(oy - R * math.cos(a))),
                 (110, 110, 110), 1, cv2.LINE_AA)

    span = max(-359.0, min(359.0, heading))              # arc of travel
    if abs(span) > 0.5:
        cv2.ellipse(frame, (ox, oy), (R - 14, R - 14), -90.0,
                    0.0, span, COL_GRID, 1, cv2.LINE_AA)

    cv2.arrowedLine(frame, (ox, oy), (ox, oy - (R - 6)),
                    COL_CAM, 1, cv2.LINE_AA, tipLength=0.22)
    a = math.radians(heading)
    cv2.arrowedLine(frame, (ox, oy),
                    (int(ox + (R - 6) * math.sin(a)), int(oy - (R - 6) * math.cos(a))),
                    COL_GRID, 2, cv2.LINE_AA, tipLength=0.25)
    cv2.circle(frame, (ox, oy), 2, COL_CAM, -1, cv2.LINE_AA)

    cv2.putText(frame, "%+.1f" % heading, (ox - 26, oy + R + 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, COL_GRID, 1, cv2.LINE_AA)
    cv2.putText(frame, "%+.0f deg/s" % rate, (ox - 28, oy + R + 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.32, (150, 150, 150), 1, cv2.LINE_AA)


# ===========================================================================
# main
# ===========================================================================
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("video", help="input video file")
    ap.add_argument("-o", "--out", help="annotated output video "
                                        "(default: <input>_annotated.mp4)")
    ap.add_argument("--csv", help="per-frame CSV (default: <input>_rotation.csv)")
    ap.add_argument("--step", type=int, default=M1_STEP,
                    help="M1 pixel step: 1 finest, 2 default, 4 fastest")
    ap.add_argument("--scale", type=int, default=3,
                    help="enlarge the output video by this factor (default 3)")
    ap.add_argument("--no-m2", action="store_true", help="run M1 only")
    ap.add_argument("--no-smooth", action="store_true",
                    help="skip the alpha-beta filter on the heading")
    ap.add_argument("--preview", action="store_true",
                    help="show a live window while processing")
    args = ap.parse_args()

    if not os.path.exists(args.video):
        sys.exit("no such file: %s" % args.video)

    stem = os.path.splitext(args.video)[0]
    out_path = args.out or (stem + "_annotated.mp4")
    csv_path = args.csv or (stem + "_rotation.csv")

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        sys.exit("could not open %s" % args.video)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    centre = (w / 2.0, h / 2.0)
    sc = max(1, args.scale)

    writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"mp4v"),
                             fps, (w * sc, h * sc))
    csv = open(csv_path, "w")
    csv.write("frame,t_s,theta_m1,theta_m2,heading_raw,heading,rate_dps,tier,"
              "x_mm,y_mm,cell_x_mm,cell_y_mm,mad_x,mad_y\n")

    unw = Unwrapper()
    ab = None if args.no_smooth else AlphaBeta(1.0 / fps)
    n = 0
    n_junc = n_bar = 0

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # ---- M1: always
        th1 = m1_angle(gray, args.step)

        # ---- M2: allowed to fail
        fit = None if args.no_m2 else m2_fit(gray, th1, centre)
        theta = fit["theta"] if fit else th1

        # ---- unwrap the REFINED angle (falls back to M1 when M2 failed),
        #      then smooth.  Unwrapping on the refined value keeps M2's
        #      precision in the continuous heading instead of throwing it away.
        heading_raw = unw.update(theta)
        heading = ab.update(heading_raw) if ab else heading_raw
        rate = ab.rate if ab else 0.0

        xy_mm = None
        cell = None
        if fit and fit["junction"] is not None:
            al, be = fit["junction"]
            # camera centre relative to the junction, along the tape axes
            x_mm = -al * MM_PER_PIXEL
            y_mm = -be * MM_PER_PIXEL
            xy_mm = (x_mm, y_mm)
            cell = (x_mm % JUNCTION_SPACING_MM, y_mm % JUNCTION_SPACING_MM)
            n_junc += 1
        elif fit:
            n_bar += 1

        vis = frame if frame.ndim == 3 else cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        vis = vis.copy()
        vis = annotate(vis, centre, theta, fit, heading, rate, cell, xy_mm, n)
        if sc > 1:
            vis = cv2.resize(vis, (w * sc, h * sc), interpolation=cv2.INTER_NEAREST)
        writer.write(vis)

        csv.write("%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%d,%s,%s,%s,%s,%s,%s\n" % (
            n, n / fps, th1, theta, heading_raw, heading, rate,
            fit["tier"] if fit else 0,
            "%.3f" % xy_mm[0] if xy_mm else "",
            "%.3f" % xy_mm[1] if xy_mm else "",
            "%.3f" % cell[0] if cell else "",
            "%.3f" % cell[1] if cell else "",
            "%.3f" % fit["mad_x"] if fit and fit["mad_x"] is not None else "",
            "%.3f" % fit["mad_y"] if fit and fit["mad_y"] is not None else ""))

        if args.preview:
            cv2.imshow("rotation", vis)
            if cv2.waitKey(1) & 0xFF == 27:
                break
        n += 1

    cap.release()
    writer.release()
    csv.close()
    if args.preview:
        cv2.destroyAllWindows()

    print("frames            %d" % n)
    print("full junction     %d  (%.0f%%)" % (n_junc, 100.0 * n_junc / max(n, 1)))
    print("single bar only   %d  (%.0f%%)" % (n_bar, 100.0 * n_bar / max(n, 1)))
    print("total turned      %+.2f deg" % unw.total)
    if ab:
        print("outliers rejected %d" % ab.rejected)
    print("video             %s" % out_path)
    print("csv               %s" % csv_path)


if __name__ == "__main__":
    main()