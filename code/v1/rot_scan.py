# rot_scan.py -- scanning along a rotated direction.
#
# Answers: "we have the angle; how do we now scan lines/columns at that angle?"
#
# The image is NEVER rotated. Rotating a 320x200 buffer costs more than the
# whole pipeline and smears the sub-pixel edges the fit depends on. What
# rotates is the *sampling pattern*: we walk in straight lines that happen to
# be diagonal in pixel space, using a fixed-point DDA (integer Bresenham-style
# stepping). The inner loop is still "read one byte, compare to threshold" --
# exactly what the existing column sweep does. The only difference is that the
# step from one sample to the next is (dx, dy) instead of (0, 1).
#
# Fixed point: 16.16. FX = 65536. A position is an int; the pixel index is
# (pos >> 16). Adding a constant step each iteration is one integer add, no
# multiply and no float in the loop.

import math
from array import array

try:
    import micropython
    VIPER_OK = True
except ImportError:
    VIPER_OK = False

FX = 16          # fractional bits
ONE = 1 << FX
MAX_RUNS = 64    # per scan line


# Scratch buffer for scan_ray output, allocated once at import so the frame
# loop never allocates. Viper needs a real int32 buffer here -- a Python list
# raises "can't convert list to int" on the device (it is fine on the PC
# fallback, which is why this only showed up on hardware).
_OUT = array('i', [0] * (MAX_RUNS * 3))


# --------------------------------------------------------------------------
# One scan line: start at (x0, y0) in 16.16, step by (dx, dy) for n samples,
# record runs of pixels >= th. Writes run midpoints (in 16.16 image coords)
# and run lengths into out[]; returns the number of runs found.
#
# out layout, 3 ints per run: [mid_x, mid_y, length_in_samples]
# --------------------------------------------------------------------------
if VIPER_OK:
    @micropython.viper
    def scan_ray(buf: ptr8, W: int, H: int,
                 x0: int, y0: int, dx: int, dy: int, n: int,
                 th: int, out: ptr32, max_runs: int,
                 min_len: int, max_len: int) -> int:
        # First threshold crossing opens a run (sx,sy); the next crossing
        # back down closes it -- that gap, "run", is the run's width in
        # samples. If that width is not between min_len and max_len, this
        # is not a real crossing of the tape (a hole edge, a speck, two
        # bars grazed at once) -- throw the point away, don't record it.
        # A run that DOES pass stops the ray right there: one valid point
        # per ray, the first one found walking outward from the centre.
        x = x0
        y = y0
        nr = 0
        run = 0
        sx = 0
        sy = 0
        i = 0
        done = 0
        while i < n and done == 0:
            px = x >> 16
            py = y >> 16
            inside = 1 if (px >= 0 and px < W and py >= 0 and py < H) else 0
            hot = 0
            if inside != 0:
                if int(buf[py * W + px]) >= th:
                    hot = 1
            if hot != 0:
                if run == 0:
                    sx = x
                    sy = y
                run += 1
            else:
                if run > 0:
                    if run >= min_len and run < max_len:
                        if nr < max_runs:
                            k = nr * 3
                            out[k] = (sx + x) >> 1     # midpoint, 16.16
                            out[k + 1] = (sy + y) >> 1
                            out[k + 2] = run
                            nr += 1
                        done = 1
                    elif run >= max_len:
                        done = 1        # too wide to be real -- stop
                    run = 0
            x += dx
            y += dy
            i += 1
        if done == 0 and run > 0 and run >= min_len and run < max_len:
            if nr < max_runs:
                k = nr * 3
                out[k] = (sx + x) >> 1
                out[k + 1] = (sy + y) >> 1
                out[k + 2] = run
                nr += 1
        return nr
else:
    def scan_ray(buf, W, H, x0, y0, dx, dy, n, th, out, max_runs,
                 min_len, max_len):
        x, y, nr, run, sx, sy = x0, y0, 0, 0, 0, 0
        done = False
        i = 0
        while i < n and not done:
            px, py = x >> 16, y >> 16
            hot = (0 <= px < W and 0 <= py < H and buf[py * W + px] >= th)
            if hot:
                if run == 0:
                    sx, sy = x, y
                run += 1
            else:
                if run > 0:
                    if min_len <= run < max_len:
                        if nr < max_runs:
                            k = nr * 3
                            out[k] = (sx + x) >> 1
                            out[k + 1] = (sy + y) >> 1
                            out[k + 2] = run
                            nr += 1
                        done = True
                    elif run >= max_len:
                        done = True
                    run = 0
            x += dx
            y += dy
            i += 1
        if not done and run > 0 and min_len <= run < max_len and nr < max_runs:
            k = nr * 3
            out[k], out[k + 1], out[k + 2] = (sx + x) >> 1, (sy + y) >> 1, run
            nr += 1
        return nr


# A valid run's width must be in this range, checked right inside scan_ray
# as each run closes -- not after the fact.
#
# THESE WERE 6 AND 70 AND THAT WAS THE BUG. The real tape is ~26-29 px, so
# a 6..70 window let a 9 px hole edge and a 60 px merged blob through
# untouched -- the check ran, and rejected essentially nothing. Bounds have
# to actually bracket the real bar width to do any work.
NOMINAL_THICK_PX = 29  # measured tape thickness on this rig
#
# The window is deliberately TIGHT. Because every ray is perpendicular to
# the bar by construction (that is the whole point of the rotated scan),
# a genuine crossing measures the true thickness no matter what angle the
# grid is at -- unlike DRIVE's axis-aligned col_scan/row_scan, where the
# run stretches as 1/cos and the window there has to stay loose.
#
# Measured spread of real crossings across 0-88 deg and 3 blur levels:
#     5th pct 28 px | median 29 px | 95th pct 30 px
# Going from 18..42 down to 25..34 drops only ~0.5% of genuine points
# while rejecting a lot more junk -- losing points is the cheaper mistake.
THICK_LO_FRAC = 0.85   # -> 24 px at nominal 29
THICK_HI_FRAC = 1.20   # -> 34 px at nominal 29
MIN_THICK_PX = int(NOMINAL_THICK_PX * THICK_LO_FRAC)
MAX_THICK_PX = int(NOMINAL_THICK_PX * THICK_HI_FRAC)

# Sweep geometry.
#
# SWEEP_HALF_LEN IS A HARD LIMIT ON MEASURABLE RANGE, not just a speed
# knob. A ray reaches half_len px from the frame centre; a bar further out
# than that cannot be seen at all. I cut it 120 -> 70 for speed and that
# silently capped the junction range at 70 * 0.223 = 15.6 mm, so X died
# past ~10 mm on real video. The synthetic tests never caught it because
# they placed junctions at most 34 px off-centre.
#
# It must cover X_RANGE_MM (30 mm) with margin:
#     30 mm / 0.223 mm per px = 135 px minimum
# 145 gives ~32 mm of reach. Going past 160 is pointless -- that is the
# frame half-width, nothing beyond it is in the image anyway.
SWEEP_STEP = 8         # px between scan lines
SWEEP_SPAN = 300       # px along the bar that scan lines cover
SWEEP_HALF_LEN = 145   # px each ray reaches from centre. SEE ABOVE before
                       # lowering this -- it costs measurable range, not
                       # just microseconds.


# --------------------------------------------------------------------------
# sweep_all: the whole family in ONE viper call.
#
# This replaces ~43 separate python->viper scan_ray() calls per family plus
# the separate _to_rot() pass over the results. DRIVE's col_scan/row_scan
# have always done all their scan lines inside a single viper call; Stage 2
# was paying python call overhead per ray, which is why it cost 11-13 ms
# against DRIVE's 3-4 ms for comparable work.
#
# Emits (al, be) rotated-frame coords as 16.16 fixed point, so the caller
# does not walk the points again to rotate them.
#
# ux,uy,vx,vy are 16.16. s0/sstep are whole pixels along the bar.
# --------------------------------------------------------------------------
if VIPER_OK:
    @micropython.viper
    def sweep_all(buf: ptr8, W: int, H: int, th: int,
                  prm: ptr32, out: ptr32, max_pts: int) -> int:
        # prm layout, all 16.16 except the last five which are whole ints:
        #  0,1  step direction along the bar   (where each ray starts)
        #  2,3  scan direction across the bar  (which way the ray walks)
        #  4,5  OUTPUT basis u -- always family 0's, so both families
        #  6,7  OUTPUT basis v -- report in one shared frame
        #  8    s0, 9 sstep, 10 nlines, 11 half_len, 12 min_len, 13 max_len
        ux = int(prm[0])
        uy = int(prm[1])
        vx = int(prm[2])
        vy = int(prm[3])
        oux = int(prm[4])
        ouy = int(prm[5])
        ovx = int(prm[6])
        ovy = int(prm[7])
        s0 = int(prm[8])
        sstep = int(prm[9])
        nlines = int(prm[10])
        half_len = int(prm[11])
        min_len = int(prm[12])
        max_len = int(prm[13])
        npts = 0
        cx = (W << 15)                 # W/2 in 16.16
        cy = (H << 15)
        li = 0
        while li < nlines and npts < max_pts:
            s = s0 + li * sstep
            # ray origin = centre + s*u - half_len*v   (all 16.16)
            ox = cx + s * ux - half_len * vx
            oy = cy + s * uy - half_len * vy
            x = ox
            y = oy
            run = 0
            sx = 0
            sy = 0
            i = 0
            n = half_len << 1
            done = 0
            while i < n and done == 0:
                px = x >> 16
                py = y >> 16
                hot = 0
                if px >= 0:
                    if px < W:
                        if py >= 0:
                            if py < H:
                                if int(buf[py * W + px]) >= th:
                                    hot = 1
                if hot != 0:
                    if run == 0:
                        sx = x
                        sy = y
                    run += 1
                else:
                    if run > 0:
                        if run >= min_len and run < max_len:
                            mx = (sx + x) >> 1
                            my = (sy + y) >> 1
                            # rotate into (al, be) about the frame centre
                            ddx = (mx - cx) >> 8
                            ddy = (my - cy) >> 8
                            # two 8.8 values multiplied is already 16.16
                            al = ddx * (oux >> 8) + ddy * (ouy >> 8)
                            be = ddx * (ovx >> 8) + ddy * (ovy >> 8)
                            k = npts * 3
                            out[k] = al
                            out[k + 1] = be
                            out[k + 2] = run
                            npts += 1
                            done = 1
                        elif run >= max_len:
                            done = 1
                        run = 0
                x += vx
                y += vy
                i += 1
            li += 1
        return npts
else:
    def sweep_all(buf, W, H, th, prm, out, max_pts):
        (ux, uy, vx, vy, oux, ouy, ovx, ovy,
         s0, sstep, nlines, half_len, min_len, max_len) = [prm[i] for i in range(14)]
        npts = 0
        cx = W << 15
        cy = H << 15
        for li in range(nlines):
            if npts >= max_pts:
                break
            s = s0 + li * sstep
            x = cx + s * ux - half_len * vx
            y = cy + s * uy - half_len * vy
            run = 0
            sx = sy = 0
            n = half_len << 1
            done = False
            i = 0
            while i < n and not done:
                px, py = x >> 16, y >> 16
                hot = (0 <= px < W and 0 <= py < H and buf[py * W + px] >= th)
                if hot:
                    if run == 0:
                        sx, sy = x, y
                    run += 1
                else:
                    if run > 0:
                        if min_len <= run < max_len:
                            mx = (sx + x) >> 1
                            my = (sy + y) >> 1
                            ddx = (mx - cx) >> 8
                            ddy = (my - cy) >> 8
                            # two 8.8 values multiplied is already 16.16
                            al = ddx * (oux >> 8) + ddy * (ouy >> 8)
                            be = ddx * (ovx >> 8) + ddy * (ovy >> 8)
                            k = npts * 3
                            out[k], out[k + 1], out[k + 2] = al, be, run
                            npts += 1
                            done = True
                        elif run >= max_len:
                            done = True
                        run = 0
                x += vx
                y += vy
                i += 1
        return npts


_SWEEP_OUT = array('i', [0] * (256 * 3))
_SWEEP_PRM = array('i', [0] * 14)


# --------------------------------------------------------------------------
# A full rotated sweep of one bar family.
#
# theta_deg  : tape angle from Algorithm 2 (mod 90, so it names both families)
# family     : 0 scans ACROSS the bar whose direction is theta
#              1 scans ACROSS the perpendicular bar (theta + 90)
# step       : spacing between scan lines, in pixels along the bar
#
# Returns a list of (x, y, runlen) centreline points in float pixels --
# the same thing the existing column sweep produces, so the existing
# MAD-robust line fit consumes it unchanged.
# --------------------------------------------------------------------------
def sweep(buf, W, H, th, theta_deg, family=0, step=5, span=None,
          half_len=None, min_len=MIN_THICK_PX, max_len=MAX_THICK_PX):
    """Walk tilted scan lines across one bar family and return run midpoints.

    Two things keep this affordable on the camera:

    1. EACH RAY IS CLIPPED to the image before it is walked. A ray crossing a
       320x200 frame at an angle is up to 380 samples long, but only ~44% of
       those land inside the frame -- the rest used to be walked and thrown
       away one pixel at a time. Clipping is a few floats per ray instead.
    2. SPAN AND LENGTH ARE BOUNDED (half_len, span). The junction is near the
       centre of the frame; scanning the far corners buys nothing.

    Returns (x, y, runlen) in float pixels.
    """
    a = math.radians(theta_deg + (90.0 if family else 0.0))
    ux, uy = math.cos(a), math.sin(a)        # along the bar
    vx, vy = -uy, ux                         # across the bar = scan direction

    diag = int(math.sqrt(W * W + H * H)) + 2
    if half_len is None:
        half_len = diag * 0.5
    if span is None:
        span = diag
    cx, cy = W * 0.5, H * 0.5

    dx = int(vx * ONE)
    dy = int(vy * ONE)
    out = _OUT
    pts = []
    xmax = W - 1
    ymax = H - 1
    s = -(span // 2)
    send = span // 2
    while s < send:
        # ray origin at the far end of the scan direction
        px = cx + s * ux - half_len * vx
        py = cy + s * uy - half_len * vy
        n = int(2 * half_len)

        # --- clip [0, n) against the image rectangle -------------------
        t0 = 0.0
        t1 = float(n)
        if vx > 1e-9 or vx < -1e-9:
            a0 = (0.0 - px) / vx
            a1 = (xmax - px) / vx
            if a0 > a1:
                a0, a1 = a1, a0
            if a0 > t0:
                t0 = a0
            if a1 < t1:
                t1 = a1
        elif px < 0.0 or px > xmax:
            t1 = -1.0
        if vy > 1e-9 or vy < -1e-9:
            a0 = (0.0 - py) / vy
            a1 = (ymax - py) / vy
            if a0 > a1:
                a0, a1 = a1, a0
            if a0 > t0:
                t0 = a0
            if a1 < t1:
                t1 = a1
        elif py < 0.0 or py > ymax:
            t1 = -1.0

        if t1 > t0:
            nclip = int(t1 - t0) + 1
            sx = int((px + t0 * vx) * ONE)
            sy = int((py + t0 * vy) * ONE)
            nr = scan_ray(buf, W, H, sx, sy, dx, dy, nclip, th, out, MAX_RUNS,
                          min_len, max_len)
            for k in range(nr):
                pts.append((out[k * 3] / ONE, out[k * 3 + 1] / ONE,
                            out[k * 3 + 2]))
        s += step
    return pts


def fit_line(pts):
    """Plain least squares in the rotated frame. Returns (angle_deg, n).
    Only here so the sweep can be checked on its own; the real pipeline
    uses the existing MAD-gated fit."""
    n = len(pts)
    if n < 4:
        return None, n
    sx = sum(p[0] for p in pts)
    sy = sum(p[1] for p in pts)
    mx, my = sx / n, sy / n
    sxx = sum((p[0] - mx) ** 2 for p in pts)
    syy = sum((p[1] - my) ** 2 for p in pts)
    sxy = sum((p[0] - mx) * (p[1] - my) for p in pts)
    # total least squares: principal direction of the scatter
    ang = 0.5 * math.atan2(2 * sxy, sxx - syy)
    return math.degrees(ang), n


# ==========================================================================
# STAGE 2 -- full frame fit at a known angle.
#
# Given theta from Stage 1, find both bars and their crossing point. This is
# scan-fit-intersect, exactly like the driving algorithm, except the scan
# direction is tilted to theta instead of locked to the image axes.
#
# Everything is expressed in the ROTATED frame (al, be):
#     al = distance along the theta direction
#     be = distance across it
# The frame centre is (al, be) = (0, 0). Converting back to pixels is a
# single rotation, done only when drawing.
# ==========================================================================

THICK_LO = 0.55        # keep runs between these multiples of the measured
THICK_HI = 1.60        # tape thickness -- rejects specks and merged blobs
MIN_POINTS = 6
MAD_ITERS = 2
MAX_FIT_MAD = 6.0      # px. A real bar sits near 0.2; noise sits near 45.


def _median(v):
    s = sorted(v)
    return s[len(s) >> 1]


def _fit(s, t):
    """Robust least squares t = a + b*s, MAD-gated. Returns (a, b, mad, n)."""
    keep = list(range(len(s)))
    a = b = 0.0
    for _ in range(MAD_ITERS + 1):
        n = len(keep)
        if n < MIN_POINTS:
            return None
        ss = st = sss = sst = 0.0
        for i in keep:
            ss += s[i]
            st += t[i]
            sss += s[i] * s[i]
            sst += s[i] * t[i]
        den = n * sss - ss * ss
        if -1e-9 < den < 1e-9:
            return None
        b = (n * sst - ss * st) / den
        a = (st - b * ss) / n
        res = [abs(t[i] - (a + b * s[i])) for i in keep]
        mad = _median(res)
        if mad < 1e-6:
            break
        lim = 3.0 * mad
        if lim < 0.5:
            lim = 0.5
        nk = [keep[i] for i in range(len(keep)) if res[i] < lim]
        if len(nk) == len(keep):
            break
        keep = nk
    n = len(keep)
    if n < MIN_POINTS:
        return None
    res = [abs(t[i] - (a + b * s[i])) for i in keep]
    return a, b, _median(res), n


def _to_rot(pts, cx, cy, ux, uy, vx, vy):
    al, be, ln = [], [], []
    for (x, y, L) in pts:
        dx = x - cx
        dy = y - cy
        al.append(dx * ux + dy * uy)
        be.append(dx * vx + dy * vy)
        ln.append(L)
    return al, be, ln


def _thickness_gate(al, be, ln):
    if not ln:
        return [], []
    # Take the median over runs that are already physically plausible. A raw
    # median is not safe here: a scan line that grazes a bar edge produces a
    # 1-2 px speck, and there are enough of those to drag the median down to
    # 7 px and reject the real 26 px tape with it.
    cand = [L for L in ln if MIN_THICK_PX <= L <= MAX_THICK_PX]
    if len(cand) < MIN_POINTS:
        return [], []
    med = _median(cand)
    lo = THICK_LO * med
    hi = THICK_HI * med
    ka, kb = [], []
    for i in range(len(ln)):
        if lo <= ln[i] <= hi:
            ka.append(al[i])
            kb.append(be[i])
    return ka, kb


def fit_frame(buf, W, H, th, theta_deg, step=None, span=None,
              half_len=None):
    """Full Stage 2. Returns a dict:
        xline    (a, b) -- bar along theta:      be = a + b*al
        yline    (a, b) -- bar across theta:     al = a + b*be
        junction (al, be) -- where they cross, or None
        theta    refined angle in degrees (from the line fit, not the gradient)
        tier     1 both bars + junction, 3 one bar only, 0 nothing
        mad, n   fit quality of the better bar
    All coordinates are in the rotated frame with the image centre at 0, 0.

    Uses sweep_all(): ONE viper call per family, emitting rotated coords
    directly. The old path made a python->viper call per scan ray (~43 per
    family) and then walked every point again in python to rotate it.
    """
    if step is None:
        step = SWEEP_STEP
    if span is None:
        span = SWEEP_SPAN
    if half_len is None:
        half_len = SWEEP_HALF_LEN

    out = {"xline": None, "yline": None, "junction": None,
           "theta": theta_deg, "tier": 0, "mad": 0.0, "n": 0}

    nlines = span // step
    s0 = -(span // 2)
    fits = [None, None]

    a0 = math.radians(theta_deg)
    u0x = int(math.cos(a0) * ONE)
    u0y = int(math.sin(a0) * ONE)
    v0x = -u0y
    v0y = u0x

    for family in (0, 1):
        if family == 0:
            sux, suy = u0x, u0y          # lines march along the bar
            svx, svy = v0x, v0y          # rays cross it
        else:
            sux, suy = v0x, v0y          # perpendicular bar: swap
            svx, svy = -u0x, -u0y
        p = _SWEEP_PRM
        p[0], p[1], p[2], p[3] = sux, suy, svx, svy
        # OUTPUT basis is family 0's for BOTH families, so the two fits
        # live in one shared frame and the junction solve is valid.
        p[4], p[5], p[6], p[7] = u0x, u0y, v0x, v0y
        p[8], p[9], p[10], p[11] = s0, step, nlines, half_len
        p[12], p[13] = MIN_THICK_PX, MAX_THICK_PX
        n = sweep_all(buf, W, H, th, p, _SWEEP_OUT, 256)
        if n < MIN_POINTS:
            continue
        # Points arrive already rotated. Family 0's bar is nearly constant
        # in `be`, so fit be(al); family 1's is constant in `al`, so the
        # roles swap. No second pass over the data either way.
        sv = []
        tv = []
        for k in range(n):
            al = _SWEEP_OUT[k * 3] / ONE
            be = _SWEEP_OUT[k * 3 + 1] / ONE
            if family:
                sv.append(be)
                tv.append(al)
            else:
                sv.append(al)
                tv.append(be)
        f = _fit(sv, tv)
        if f and f[2] <= MAX_FIT_MAD:
            fits[family] = f

    if fits[0]:
        out["xline"] = (fits[0][0], fits[0][1])
    if fits[1]:
        out["yline"] = (fits[1][0], fits[1][1])

    if out["xline"] and out["yline"]:
        a1, b1 = out["xline"]           # be = a1 + b1*al
        a2, b2 = out["yline"]           # al = a2 + b2*be
        den = 1.0 - b1 * b2
        if abs(den) > 1e-6:
            jal = (a2 + b2 * a1) / den
            jbe = a1 + b1 * jal
            out["junction"] = (jal, jbe)
            out["tier"] = 1
        out["theta"] = theta_deg + math.degrees(math.atan(b1))
    elif out["xline"]:
        out["tier"] = 3
        out["theta"] = theta_deg + math.degrees(math.atan(out["xline"][1]))
    elif out["yline"]:
        out["tier"] = 3

    best = fits[0] if (fits[0] and (not fits[1] or fits[0][2] <= fits[1][2])) \
        else fits[1]
    if best:
        out["mad"] = best[2]
        out["n"] = best[3]
    return out
