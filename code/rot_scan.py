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
                 th: int, out: ptr32, max_runs: int) -> int:
        x = x0
        y = y0
        nr = 0
        run = 0
        sx = 0
        sy = 0
        i = 0
        while i < n:
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
                if run > 0 and nr < max_runs:
                    k = nr * 3
                    out[k] = (sx + x) >> 1        # midpoint, still 16.16
                    out[k + 1] = (sy + y) >> 1
                    out[k + 2] = run
                    nr += 1
                run = 0
            x += dx
            y += dy
            i += 1
        if run > 0 and nr < max_runs:
            k = nr * 3
            out[k] = (sx + x) >> 1
            out[k + 1] = (sy + y) >> 1
            out[k + 2] = run
            nr += 1
        return nr
else:
    def scan_ray(buf, W, H, x0, y0, dx, dy, n, th, out, max_runs):
        x, y, nr, run, sx, sy = x0, y0, 0, 0, 0, 0
        for _ in range(n):
            px, py = x >> 16, y >> 16
            hot = (0 <= px < W and 0 <= py < H and buf[py * W + px] >= th)
            if hot:
                if run == 0:
                    sx, sy = x, y
                run += 1
            else:
                if run > 0 and nr < max_runs:
                    k = nr * 3
                    out[k] = (sx + x) >> 1
                    out[k + 1] = (sy + y) >> 1
                    out[k + 2] = run
                    nr += 1
                run = 0
            x += dx
            y += dy
        if run > 0 and nr < max_runs:
            k = nr * 3
            out[k], out[k + 1], out[k + 2] = (sx + x) >> 1, (sy + y) >> 1, run
            nr += 1
        return nr


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
          half_len=None):
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
            nr = scan_ray(buf, W, H, sx, sy, dx, dy, nclip, th, out, MAX_RUNS)
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
MIN_THICK_PX = 6
MAX_THICK_PX = 70
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


def fit_frame(buf, W, H, th, theta_deg, step=5, span=None,
              half_len=None):
    """Full Stage 2. Returns a dict:
        xline    (a, b) -- bar along theta:      be = a + b*al
        yline    (a, b) -- bar across theta:     al = a + b*be
        junction (al, be) -- where they cross, or None
        theta    refined angle in degrees (from the line fit, not the gradient)
        tier     1 both bars + junction, 3 one bar only, 0 nothing
        mad, n   fit quality of the better bar
    All coordinates are in the rotated frame with the image centre at 0, 0.
    """
    a = math.radians(theta_deg)
    ux, uy = math.cos(a), math.sin(a)
    vx, vy = -uy, ux
    cx, cy = W * 0.5, H * 0.5

    out = {"xline": None, "yline": None, "junction": None,
           "theta": theta_deg, "tier": 0, "mad": 0.0, "n": 0}

    # --- bar A: runs along theta, so it is crossed by scanning along v
    ptsA = sweep(buf, W, H, th, theta_deg, family=0, step=step, span=span,
                 half_len=half_len)
    alA, beA, lnA = _to_rot(ptsA, cx, cy, ux, uy, vx, vy)
    sA, tA = _thickness_gate(alA, beA, lnA)
    fA = _fit(sA, tA) if len(sA) >= MIN_POINTS else None
    if fA and fA[2] <= MAX_FIT_MAD:
        out["xline"] = (fA[0], fA[1])

    # --- bar B: runs across theta
    ptsB = sweep(buf, W, H, th, theta_deg, family=1, step=step, span=span,
                 half_len=half_len)
    alB, beB, lnB = _to_rot(ptsB, cx, cy, ux, uy, vx, vy)
    # for this bar the roles swap: it is nearly constant in al, so fit al(be)
    sB, tB = _thickness_gate(beB, alB, lnB)
    fB = _fit(sB, tB) if len(sB) >= MIN_POINTS else None
    if fB and fB[2] <= MAX_FIT_MAD:
        out["yline"] = (fB[0], fB[1])

    if out["xline"] and out["yline"]:
        a1, b1 = out["xline"]           # be = a1 + b1*al
        a2, b2 = out["yline"]           # al = a2 + b2*be
        den = 1.0 - b1 * b2
        if abs(den) > 1e-6:
            jal = (a2 + b2 * a1) / den
            jbe = a1 + b1 * jal
            out["junction"] = (jal, jbe)
            out["tier"] = 1
        # refine theta: b1 is the residual tilt of bar A inside the rotated
        # frame, so the true angle is theta plus that.
        out["theta"] = theta_deg + math.degrees(math.atan(b1))
    elif out["xline"]:
        out["tier"] = 3
        out["theta"] = theta_deg + math.degrees(math.atan(out["xline"][1]))
    elif out["yline"]:
        out["tier"] = 3

    best = fA if (fA and (not fB or fA[2] <= fB[2])) else fB
    if best:
        out["mad"] = best[2]
        out["n"] = best[3]
    return out
