# main.py -- grid-following camera for the OpenMV AE3.  Production build.
#
# WHAT IT DOES
#   Looks straight down at a taped floor grid (80 mm pitch) and reports,
#   every frame, where the camera is relative to the nearest junction:
#       X      signed mm along the bar, 0 -> -30 leaving a junction, then
#              null across the middle of the cell, then +30 -> 0 arriving
#       Y      signed mm across the bar
#       THETA  grid tilt in degrees
#   Two modes, switched by the host over UART ("TURN L" / "TURN R" / "DRIVE"):
#       DRIVE  Algorithm 1: axis-aligned column/row scans, MAD-robust line
#              fits, junction = intersection. Runs at ~105 fps on the AE3.
#       TURN   Algorithm 2 (turn_sm.py): whole-frame gradient angle, unwrapped
#              and filtered into a continuous heading; plus Stage 2
#              (rot_scan.py), a tilted scan that gives X/Y/THETA while the
#              axis-aligned scan is blind. ~93 fps.
#
# FILES
#   main.py      this file -- Algorithm 1, the frame loop, the terminal line
#   turn_sm.py   Algorithm 2, the DRIVE/TURN state machine, the host link
#   rot_scan.py  Stage 2, the tilted scan
#   hud.py       the on-screen overlay (gauge, tape lines)
#
# TERMINAL, once per second, same fields in both modes:
#   frame 4231  junction  X-27.5  Y+0.8  THETA+1.2  DRV  fps105
#   frame 4890  X-2.8  Y+0.4  THETA-46.2  ROTATION_PER_SEC+2.0  ROT  fps93
#   "--" means not measured this second. fps is the camera rate (draw and
#   flush excluded). In ROT, THETA is the continuous heading. Events
#   (TURN_START, TURN_SEED, JUNCTION_LANDED, faults) print when they happen.
#
# KNOBS YOU WILL TOUCH
#   DRAW          0 on the robot (default). 1 to see the overlay in the IDE;
#                 costs ~12.6 ms per frame, more than the whole algorithm.
#   PERF          1 to add a per-stage timing line to the terminal.
#   BENCH         1 to benchmark this file against itself and stop.
#   MM_PER_PIXEL  0.244, measured. Every mm number scales with it.
#   SIM_CMDS      [] on the robot. [(frame, "TURN L")] to force a turn on
#                 the bench.
#
# Measurement history, settled facts and the reasoning behind each constant
# live in NOTES.md, not here.

import gc
import math
import os
import time
from array import array

import image
import hud
from turn_sm import TurnSM, HostLink, wrap90

ITERS = 3            # MAD rejection rounds
WSIGMA = 30          # Gaussian width of the junction-local fit, px
BAR_WIDTH_MM = 13.5  # only used if MM_PER_PIXEL is 0; unverified
MM_PER_PIXEL = 0.244 # measured: 71.5 mm spanned 293 px, twice (NOTES.md)


MAX_STEP_MM = 30.0         # a fix further than this from the last one is a bad fit, not motion; 0 = off
# ======================= X ODOMETRY (new scheme) =========================
# X is no longer "distance to the next junction, 0..80". It is now signed
# and relative to whichever junction is nearer, with a dead zone in the
# middle of the cell where no honest X exists:
#
#      junction                                              junction
#         |                                                     |
#    0 -> -30 mm ......... null (no X) ......... +30 mm ->  0
#         leaving                                    approaching
#
#   * Just after leaving a junction X counts DOWN from 0 to -30 mm.
#   * Past -30 mm the junction behind is too far to measure -> X = None.
#   * Once the junction ahead comes within 30 mm, X jumps to +30 and
#     counts down to 0 as you arrive. Then the cycle repeats.
#
# The Y line is still drawn and still reported while X is null -- only the
# X NUMBER goes away, not the tracking.
X_RANGE_MM = 30.0          # how far either side of a junction X is valid
X_DIRECTION = +1           # -1 if the camera is mounted 180 deg the other way

JUNCTION_SPACING_MM = 80.0 # grid pitch, centre to centre
TILT_DEBUG = 0             # 1 = once a second, print the tilt probe on a full junction
MIN_V_SPAN_PX = 6          # a real vertical bar spans >= this many columns (glare spans 1-5)
THICK_MATCH_FRAC = 0.55    # vertical bar width must be >= this x the horizontal's (same tape)
REACQUIRE_MINI_EVERY = 12  # between junctions, run a full-width scan every N frames

PERF = 0             # 1 = add a per-stage timing line to the terminal, once a second
GC_EVERY = 30        # gc.collect() every N frames; every frame cost ~1 ms
OTSU_EVERY = 3       # recompute the threshold every N frames
FLUSH = 1            # flush to the IDE even with DRAW = 0 (frame buffer only)
DRAW = 1             # 1 = overlay in the IDE. 0 on the robot: costs ~12.6 ms/frame
COLOR_OVERLAY = 0    # 1 = capture RGB565 so the overlay is in colour (costs a convert)
COLOR_X = (255, 255, 255)  # X line -- white (grayscale capture)
COLOR_Y = (90, 90, 90)     # Y line, dim grey, tellable from the X line
COLOR_MARK = (255, 255, 255)   # cross + text

SUMMARY_MS = 1000    # print interval

EXPOSURE_US = 3000   # honoured only on dev firmware 631681e5ac; cannot exceed a frame period
GAIN_DB = 18.0       # the real brightness control: mean 42 @12 dB, 71 @18, 106 @24
GAIN_CEILING_DB = 8  # unused while EXPOSURE_US is set
FRAMERATE = 240      # must precede auto_exposure; sensor measured at 120 fps regardless
FRAMEBUFFERS = 0     # 0 = leave alone

HOST_UART_ID = None   # e.g. 1 -> machine.UART(1, HOST_BAUD). None = bench mode:
HOST_BAUD = 115200    # commands come only from SIM_CMDS / SM.link.inject().
SIM_CMDS = []         # [] on the robot. [(frame, "TURN L")] forces a turn on the bench

# ===================== BENCH MODE ==========================================
# BENCH = 1: capture BENCH_FRAMES still frames, time every stage of THIS
# file's pipeline on them (capture, otsu, scan, fit, grad4, Stage 2, overlay),
# report the sd of X/Y/THETA over the still frames (= the pipeline's own
# noise), try cheaper grad4 and Stage 2 settings on the same frames, print
# three tables and stop. Hold the camera still on a junction. Set back to 0.
BENCH = 0
BENCH_FRAMES = 16      # ~1 MB at 320x200; the AE3 has 13.5 MB
BENCH_REPEATS = 20     # timing calls per stage, first one discarded
print("=== scan_fit_fps_live.py -- VERSION: mini-junction-v17-fast ===")
print("=== if you do NOT see 'v4' on the line above you are running")
print("=== an old file. Close the tab in the IDE, reopen the download.")

# ---------------------------------------------------------- VERIFY: sensor
try:
    import csi
    csi0 = csi.CSI()
    csi0.reset()
    if COLOR_OVERLAY:
        csi0.pixformat(csi.RGB565)
    else:
        csi0.pixformat(csi.GRAYSCALE)
    csi0.framesize(csi.QVGA)
    if FRAMERATE:
        try:
            csi0.framerate(FRAMERATE)
        except Exception as _e:
            print("framerate(%d) failed (%s)" % (FRAMERATE, _e))
    if EXPOSURE_US:
        try:
            csi0.auto_exposure(False, exposure_us=EXPOSURE_US)
        except Exception as _e:
            print("auto_exposure failed (%s) -- still on auto" % _e)
        try:
            csi0.auto_gain(False, gain_db=GAIN_DB)
        except Exception as _e:
            print("auto_gain failed (%s)" % _e)
    elif GAIN_CEILING_DB:
        try:
            csi0.auto_gain(True, gain_db_ceiling=GAIN_CEILING_DB)
        except Exception as _e:
            print("auto_gain ceiling failed (%s)" % _e)
    if FRAMEBUFFERS:
        try:
            csi0.framebuffers(FRAMEBUFFERS)
        except Exception as _e:
            print("framebuffers(%d) failed (%s)" % (FRAMEBUFFERS, _e))
    csi0.snapshot(time=500)
    HAVE_SENSOR = True
except Exception as e:
    HAVE_SENSOR = False
    print("csi init failed: %s" % e)

# --------------------------------------------------------- settled config
MIN_LEN = 4
LONG_CUT = 60
EXTRA = 8
MAXPTS = 2000
THICK_LO = 0.60
THICK_HI = 1.35
NOMINAL_THICK_PX = 29  # measured tape thickness on this rig, px
MAD_SHIFT = 9
MAD_BINS = 256
WTAB_N = 256
WMAX = 64
NOGATE = 1 << 28
MIN_PTS = 8
MAX_JOINT_MAD_PX = 3.0 # both-bar fit residual gate, px; real bars ~0.2, noise ~45
MAX_FIT_MAD_PX = 5.0   # the fallback's real "is there a bar?" test.
COVERAGE_FRAC = 0.25
ACQUIRE_STEP = 2       # column/row spacing for the full re-acquire scan (tracked uses STEP)
REACQUIRE_EVERY = 30
OTSU_SCALE = 0.25
STEP = 5
PRINT_MS = 15


def _resolve_gray():
    """Pick a working RGB565 -> GRAYSCALE conversion for this firmware.

    Tried in order against a real frame rather than assumed, because the
    method name has moved between OpenMV versions. Returns a function,
    or None if the frame is already grayscale / nothing works.
    """
    if not COLOR_OVERLAY:
        return None
    probe = csi0.snapshot()

    def m1(im):
        return im.to_grayscale(copy=True)

    def m2(im):
        return im.copy(copy_to_fb=False, pixformat=image.GRAYSCALE)

    def m3(im):
        return im.copy().to_grayscale()

    for fn, name in ((m1, "to_grayscale(copy=True)"),
                     (m2, "copy(pixformat=GRAYSCALE)"),
                     (m3, "copy().to_grayscale()")):
        try:
            g = fn(probe)
            n = len(g.bytearray())
            if n == probe.width() * probe.height():
                print("grayscale conversion: %s   OK (%d bytes)" % (name, n))
                return fn
            print("grayscale conversion: %s gave %d bytes, want %d"
                  % (name, n, probe.width() * probe.height()))
        except Exception as _e:
            print("grayscale conversion: %s failed (%s)" % (name, _e))
    print("NO working RGB565->GRAYSCALE conversion found.")
    print("Set COLOR_OVERLAY = 0 at the top of this file and re-run;")
    print("you will get white / dim-grey lines instead of green / blue.")
    return None


def _dim(o, n, d):
    a = getattr(o, n, None)
    if a is None:
        return d
    try:
        return int(a() if callable(a) else a)
    except Exception:
        return d


def _get(o, n, d="?"):
    a = getattr(o, n, None)
    if a is None:
        return d
    try:
        return a() if callable(a) else a
    except Exception:
        return d


if HAVE_SENSOR:
    W = _dim(csi0, "width", 320)
    H = _dim(csi0, "height", 200)
    print("frame size from sensor: %dx%d" % (W, H))
    _exp = _get(csi0, "exposure_us")
    print("exposure requested %s us -> applied %s us   gain %s dB"
          % (EXPOSURE_US, _exp, _get(csi0, "gain_db")))
    TO_GRAY = _resolve_gray()
    if COLOR_OVERLAY and TO_GRAY is None:
        raise SystemExit("stopping: see the message above")
    _im = csi0.snapshot()
    if TO_GRAY:
        _im = TO_GRAY(_im)
    _sm = _im.copy(x_scale=OTSU_SCALE, y_scale=OTSU_SCALE)
    _th = int(_get(_sm.get_histogram().get_threshold(), "value", -1))
    del _sm
    _st = _im.get_statistics()
    _mx = _get(_st, "max", -1)
    _mn = _get(_st, "mean", -1)
    del _im
    try:
        _head = int(_mx) - _th
    except Exception:
        _head = -1
    print("brightness: mean %s  max %s  otsu %d  head %s"
          % (_mn, _mx, _th, _head))
    if isinstance(_head, int) and 0 <= _head < 40:
        print("  WARNING: only %d grey levels between marker and Otsu cut."
              % _head)
        print("  Raise GAIN_DB (try 12-18) and re-check this line.")
else:
    W, H = 320, 200

REJECT = 2.5

try:
    @micropython.viper
    def _p(x: int) -> int:
        return x + 1
    _p(1)
    VIPER = True
except Exception as e:
    VIPER = False
    print("viper unavailable: %s" % e)


if VIPER:
    @micropython.viper
    def col_scan(buf: ptr8, w: int, h: int, th: int, minlen: int,
                 longcut: int, cols: ptr32, nc: int, out: ptr32,
                 maxn: int, res: ptr32, hist: ptr32):
        # FIRST VALID-WIDTH RUN WINS. A column can cross more than one
        # bright run whose length happens to fall inside [minlen, longcut)
        # -- floor texture, a mounting hole edge, a second bar further
        # down. Every one of those used to become its own dot, all feeding
        # the same fit. Now: the first run whose width is in-band is kept
        # (this is "the point from the left" for a column: topmost), and
        # scanning that column stops right there. A run that is too SHORT
        # (below minlen) is still skipped over silently -- that is noise,
        # keep looking further down for the real bar.
        n = 0
        bx0 = 99999
        bx1 = -1
        ci = 0
        while ci < nc:
            x = int(cols[ci])
            s = -1
            y = 0
            done = 0
            got = 0      # a point has already been taken for this column.
                         # The first valid-width run still WINS as the
                         # column's point, but it no longer ENDS the column
                         # scan: we keep going to look for a LONG run, the
                         # vertical bar. See the note above col_scan.
            while y < h and done == 0:
                if int(buf[y * w + x]) >= th:
                    if s < 0:
                        s = y
                else:
                    if s >= 0:
                        L = y - s
                        e = y - 1
                        if L >= longcut:
                            if x < bx0:
                                bx0 = x
                            if x > bx1:
                                bx1 = x
                            done = 1
                        elif L >= minlen and got == 0:
                            if n < maxn:
                                top = s * 256
                                if s > 0:
                                    a = int(buf[(s - 1) * w + x])
                                    b = int(buf[s * w + x])
                                    if b > a:
                                        top = (s - 1) * 256 + \
                                              ((th - a) * 256) // (b - a)
                                bot = e * 256
                                if e < h - 1:
                                    a = int(buf[e * w + x])
                                    b = int(buf[(e + 1) * w + x])
                                    if a > b:
                                        bot = e * 256 + \
                                              ((a - th) * 256) // (a - b)
                                out[n * 3] = x
                                out[n * 3 + 1] = (top + bot) >> 5
                                out[n * 3 + 2] = L
                                if L < 256:
                                    hist[L] = int(hist[L]) + 1
                                n += 1
                            got = 1
                        s = -1
                y += 1
            if done == 0 and s >= 0:
                L = h - s
                if L >= longcut:
                    if x < bx0:
                        bx0 = x
                    if x > bx1:
                        bx1 = x
                elif L >= minlen and got == 0 and n < maxn:
                    out[n * 3] = x
                    out[n * 3 + 1] = (s * 16 + (h - 1) * 16) >> 1
                    out[n * 3 + 2] = L
                    if L < 256:
                        hist[L] = int(hist[L]) + 1
                    n += 1
            ci += 1
        res[0] = n
        res[1] = bx0
        res[2] = bx1

    @micropython.viper
    def row_scan(buf: ptr8, w: int, h: int, th: int, minlen: int,
                 maxlen: int, x0: int, x1: int, rows: ptr32, nr: int,
                 out: ptr32, maxn: int, hist: ptr32) -> int:
        # Same rule as col_scan: first run whose width lands in
        # [minlen, maxlen) wins, scanning that row stops there -- "the
        # point from the left". maxlen is new; row_scan previously had no
        # upper width bound at all, so a too-wide spurious run (e.g. two
        # nearby teeth merged into one bright stretch) could still get in.
        n = 0
        ri = 0
        while ri < nr:
            y = int(rows[ri])
            base = y * w
            s = -1
            x = x0
            done = 0
            while x < x1 and done == 0:
                if int(buf[base + x]) >= th:
                    if s < 0:
                        s = x
                else:
                    if s >= 0:
                        L = x - s
                        e = x - 1
                        if L >= minlen and L < maxlen:
                            if n < maxn:
                                lf = s * 256
                                if s > x0:
                                    a = int(buf[base + s - 1])
                                    b = int(buf[base + s])
                                    if b > a:
                                        lf = (s - 1) * 256 + \
                                             ((th - a) * 256) // (b - a)
                                rt = e * 256
                                if e < x1 - 1:
                                    a = int(buf[base + e])
                                    b = int(buf[base + e + 1])
                                    if a > b:
                                        rt = e * 256 + \
                                             ((a - th) * 256) // (a - b)
                                out[n * 3] = y
                                out[n * 3 + 1] = (lf + rt) >> 5
                                out[n * 3 + 2] = L
                                if L < 256:
                                    hist[L] = int(hist[L]) + 1
                                n += 1
                            done = 1
                        elif L >= maxlen:
                            # too wide to be a real single crossing -- stop
                            # here rather than let a later, narrower, valid
                            # run further right get picked up out of order.
                            done = 1
                        s = -1
                x += 1
            if done == 0 and s >= 0:
                L = x1 - s
                if L >= minlen and L < maxlen and n < maxn:
                    out[n * 3] = y
                    out[n * 3 + 1] = (s * 16 + (x1 - 1) * 16) >> 1
                    out[n * 3 + 2] = L
                    if L < 256:
                        hist[L] = int(hist[L]) + 1
                    n += 1
            ri += 1
        return n

    @micropython.viper
    def hist_peak(hist: ptr32, lo: int, hi: int) -> int:
        best = 0
        bc = 0
        L = lo
        while L <= hi:
            c = int(hist[L])
            if c > bc:
                bc = c
                best = L
            L += 1
        return best

    @micropython.viper
    def hist_peak_count(hist: ptr32, lo: int, hi: int) -> int:
        bc = 0
        L = lo
        while L <= hi:
            c = int(hist[L])
            if c > bc:
                bc = c
            L += 1
        return bc

    @micropython.viper
    def lsq(arr: ptr32, n: int, lo: int, hi: int, b_fx: int, a_fx: int,
            gate: int, acc: ptr32) -> int:
        cnt = 0
        su = 0
        sv = 0
        suu = 0
        suv = 0
        i = 0
        while i < n:
            L = int(arr[i * 3 + 2])
            if L >= lo:
                if L <= hi:
                    u = int(arr[i * 3])
                    v = int(arr[i * 3 + 1])
                    keep = 1
                    if gate >= 0:
                        r = (v << 8) - (a_fx + ((b_fx * u) >> 8))
                        if r < 0:
                            r = -r
                        if r > gate:
                            keep = 0
                    if keep != 0:
                        cnt += 1
                        su += u
                        sv += v
                        suu += u * u
                        suv += u * v
            i += 1
        acc[0] = cnt
        acc[1] = su
        acc[2] = sv
        acc[3] = suu
        acc[4] = suv
        return cnt

    @micropython.viper
    def resid_hist(arr: ptr32, n: int, lo: int, hi: int, b_fx: int,
                   a_fx: int, hist: ptr32, shift: int,
                   nbins: int) -> int:
        i = 0
        while i < nbins:
            hist[i] = 0
            i += 1
        cnt = 0
        i = 0
        while i < n:
            L = int(arr[i * 3 + 2])
            if L >= lo:
                if L <= hi:
                    u = int(arr[i * 3])
                    v = int(arr[i * 3 + 1])
                    r = (v << 8) - (a_fx + ((b_fx * u) >> 8))
                    if r < 0:
                        r = -r
                    q = r >> shift
                    if q >= nbins:
                        q = nbins - 1
                    hist[q] = int(hist[q]) + 1
                    cnt += 1
            i += 1
        return cnt

    @micropython.viper
    def hist_median(hist: ptr32, nbins: int, tot: int) -> int:
        if tot <= 0:
            return 0
        half = tot >> 1
        c = 0
        i = 0
        while i < nbins:
            c += int(hist[i])
            if c > half:
                return i
            i += 1
        return nbins - 1

    @micropython.viper
    def wlsq(arr: ptr32, n: int, prm: ptr32, wtab: ptr32,
             acc: ptr32) -> int:
        lo = int(prm[0])
        hi = int(prm[1])
        b_fx = int(prm[2])
        a_fx = int(prm[3])
        gate = int(prm[4])
        uc = int(prm[5])
        vc = int(prm[6])
        nt = int(prm[7])
        sw = 0
        sdu = 0
        sdv = 0
        sduu = 0
        sduv = 0
        cnt = 0
        i = 0
        while i < n:
            L = int(arr[i * 3 + 2])
            if L >= lo:
                if L <= hi:
                    u = int(arr[i * 3])
                    du = u - uc
                    if du < 0:
                        ad = -du
                    else:
                        ad = du
                    if ad < nt:
                        w = int(wtab[ad])
                        if w > 0:
                            v = int(arr[i * 3 + 1])
                            r = (v << 8) - (a_fx + ((b_fx * u) >> 8))
                            if r < 0:
                                r = -r
                            if r <= gate:
                                dv = v - vc
                                sw += w
                                sdu += w * du
                                sdv += w * dv
                                sduu += w * du * du
                                sduv += w * du * dv
                                cnt += 1
            i += 1
        acc[0] = sw
        acc[1] = sdu
        acc[2] = sdv
        acc[3] = sduu
        acc[4] = sduv
        acc[7] = cnt
        return cnt


def P(s):
    print(s)
    time.sleep_ms(PRINT_MS)


def V(o, n, d=-1):
    a = getattr(o, n, None)
    if a is None:
        return d
    try:
        return a() if callable(a) else a
    except Exception:
        return d


def spaced(step, total):
    if step <= 1:
        return list(range(total))
    off = (total - 1 - ((total - 1) // step) * step) // 2
    return list(range(off, total, step))


def solve(acc):
    n = acc[0]
    if n < 4:
        return None
    su = acc[1]
    sv = acc[2]
    den = n * acc[3] - su * su
    if abs(den) < 1:
        return None
    bp = float(n * acc[4] - su * sv) / float(den)
    ap = (float(sv) - bp * float(su)) / float(n)
    return bp / 16.0, ap / 16.0, bp, ap


def solve_w(acc, uc, vc):
    sw = acc[0]
    if sw < 1 or acc[7] < 4:
        return None
    sdu = acc[1]
    sdv = acc[2]
    den = sw * acc[3] - sdu * sdu
    if abs(den) < 1:
        return None
    bp = float(sw * acc[4] - sdu * sdv) / float(den)
    dv = (float(sdv) - bp * float(sdu)) / float(sw)
    return bp / 16.0, ((float(vc) + dv) - bp * float(uc)) / 16.0


if not VIPER:
    P("viper unavailable, stopping.")
elif not HAVE_SENSOR:
    P("no working sensor init, stopping. Fix the block marked VERIFY.")
else:
    gc.collect()
    CO = array('i', [0] * (MAXPTS * 3))
    RO = array('i', [0] * (MAXPTS * 3))
    res = array('i', [0, 0, 0])
    acc = array('i', [0] * 8)
    prm = array('i', [0] * 8)
    mh = array('i', [0] * MAD_BINS)
    hc = array('i', [0] * 256)
    hr = array('i', [0] * 256)

    WTAB = array('i', [0] * WTAB_N)
    for d in range(WTAB_N):
        WTAB[d] = int(WMAX * math.exp(-((d / float(WSIGMA)) ** 2)) + 0.5)

    acl = spaced(ACQUIRE_STEP, W)
    arl = spaced(ACQUIRE_STEP, H)
    allc = array('i', acl)
    allr = array('i', arl)
    NAC, NAR = len(acl), len(arl)
    cl = spaced(STEP, W)
    rl = spaced(STEP, H)
    ca = array('i', cl)
    ra = array('i', rl)
    NC, NR = len(cl), len(rl)

    def fit_line(arr, n, lo, hi, iters, reject=REJECT):
        if int(lsq(arr, n, lo, hi, 0, 0, -1, acc)) < 4:
            return None
        avail = acc[0]
        s = solve(acc)
        if s is None:
            return None
        b, a, bp, ap = s
        mad = 0.0
        for _ in range(iters):
            b_fx = int(bp * 65536.0)
            a_fx = int(ap * 256.0)
            nh = int(resid_hist(arr, n, lo, hi, b_fx, a_fx, mh,
                                MAD_SHIFT, MAD_BINS))
            mad = (int(hist_median(mh, MAD_BINS, nh)) + 0.5) / 8.0
            gate = int(reject * 1.4826 * mad * 4096.0)
            if gate < 1024:
                gate = 1024
            if int(lsq(arr, n, lo, hi, b_fx, a_fx, gate, acc)) < 4:
                break
            s2 = solve(acc)
            if s2 is None:
                break
            b, a, bp, ap = s2
        if iters:
            gate = int(reject * 1.4826 * mad * 4096.0)
            if gate < 1024:
                gate = 1024
        else:
            gate = NOGATE
        return b, a, gate, avail, mad

    def fit_weighted(arr, n, lo, hi, b, a, gate, uc, wtab):
        uci = int(uc + 0.5)
        vc = int((a + b * uci) * 16.0 + 0.5)
        prm[0] = lo
        prm[1] = hi
        prm[2] = int(b * 16.0 * 65536.0)
        prm[3] = int(a * 16.0 * 256.0)
        prm[4] = gate
        prm[5] = uci
        prm[6] = vc
        prm[7] = WTAB_N
        if int(wlsq(arr, n, prm, wtab, acc)) < 4:
            return None
        return solve_w(acc, uci, vc)

    def hud_line(x_mm, y_mm, t_deg, tag):
        """The single top-left line. IDENTICAL in drive and in rotation --
        that is the whole point of it existing. Any of the three values may
        be None (X in the null zone, a missing bar, no fit at all during
        rotation); "%+.1f" % None raises, the frame try/except swallows it
        as a FRAME ERROR and the whole frame is lost, so every field goes
        through the same None check in one place."""
        return "%s %s %s %s" % (
            "X  --  " if x_mm is None else "X%+.1f" % x_mm,
            "Y  --  " if y_mm is None else "Y%+.1f" % y_mm,
            "T  --  " if t_deg is None else "T%+.1f" % t_deg,
            tag)

    def signed_x(jx_px, mmpx):
        """Raw junction offset in px -> the new signed X, or None.

        d is how far the fitted junction sits from the camera centre:
        negative once we have driven past it, positive while it is still
        ahead. That maps straight onto the scheme:

            d in [-30, 0]  -> just left it, X = d      (0 down to -30)
            d in (-50, -30)-> too far from either      -> None
            d in [-80,-50] -> next one is within 30,   X = d + 80 (+30 -> 0)

        Everything is measured against the junction we actually fitted,
        so no dead reckoning and no accumulated drift.
        """
        d = (jx_px - W * 0.5) * mmpx
        # fold into (-JUNCTION_SPACING, 0] so "the junction behind us"
        # is always the reference
        while d > 0.0:
            d -= JUNCTION_SPACING_MM
        while d <= -JUNCTION_SPACING_MM:
            d += JUNCTION_SPACING_MM
        if d >= -X_RANGE_MM:
            x = d                                   # leaving: 0 -> -30
        elif d <= -(JUNCTION_SPACING_MM - X_RANGE_MM):
            x = d + JUNCTION_SPACING_MM             # approaching: +30 -> 0
        else:
            return None                             # null zone
        return x * X_DIRECTION

    def mm_scale(t):
        if MM_PER_PIXEL:
            return MM_PER_PIXEL
        return BAR_WIDTH_MM / t if t else 0.0

    def estimate(n_c, n_r, tc, tr):
        hlo, hhi = int(tc * THICK_LO), int(tc * THICK_HI)
        vlo, vhi = int(tr * THICK_LO), int(tr * THICK_HI)
        fh = fit_line(CO, n_c, hlo, hhi, ITERS)
        fv = fit_line(RO, n_r, vlo, vhi, ITERS)
        if fh is None or fv is None:
            return None
        if fh[4] > MAX_JOINT_MAD_PX or fv[4] > MAX_JOINT_MAD_PX:
            return None      # blurred / contaminated -- let the
                             # single-bar fallback handle this frame
        den = 1.0 - fh[0] * fv[0]
        if abs(den) < 1e-9:
            return None
        ux = (fv[1] + fv[0] * fh[1]) / den
        uy = fh[1] + fh[0] * ux
        jx, jy = ux, uy
        bh, ah = fh[0], fh[1]
        bv, av = fv[0], fv[1]
        wh = fit_weighted(CO, n_c, hlo, hhi, fh[0], fh[1], fh[2], ux, WTAB)
        wv = fit_weighted(RO, n_r, vlo, vhi, fv[0], fv[1], fv[2], uy, WTAB)
        if wh and wv:
            den = 1.0 - wh[0] * wv[0]
            if abs(den) > 1e-9:
                jx = (wv[1] + wv[0] * wh[1]) / den
                jy = wh[1] + wh[0] * jx
                bh, ah = wh[0], wh[1]
                bv, av = wv[0], wv[1]
        thh = math.degrees(math.atan(fh[0]))
        thv = -math.degrees(math.atan(fv[0]))
        return jx, jy, (thh + thv) * 0.5, thh - thv, bh, ah, bv, av

    def tilt_probe(n_c, tc, jx):
        """Least-squares slope of the horizontal bar on each side of
        the junction, from the points already in CO. Pure python and
        only run once a second, so it costs nothing in the hot loop."""
        lo, hi = int(tc * THICK_LO), int(tc * THICK_HI)
        out = []
        for side in (0, 1):
            n = 0
            su = sv = suu = suv = 0.0
            i = 0
            while i < n_c:
                L = CO[i * 3 + 2]
                if lo <= L <= hi:
                    u = CO[i * 3]
                    ok_side = (u < jx - 20) if side == 0 else (u > jx + 20)
                    if ok_side:
                        v = CO[i * 3 + 1] / 16.0
                        n += 1
                        su += u
                        sv += v
                        suu += u * u
                        suv += u * v
                i += 1
            if n < 6:
                out.append((None, None, n))
                continue
            den = n * suu - su * su
            if abs(den) < 1e-6:
                out.append((None, None, n))
                continue
            b = (n * suv - su * sv) / den
            a = (sv - b * su) / n
            out.append((math.degrees(math.atan(b)), a + b * jx, n))
        return out

    def plausible(x_new):
        """Reject a reading the camera could not physically have moved
        to since the last one. Compared modulo the cell spacing, so a
        genuine wrap (X running down to 0 and resetting to 80) is a
        zero-length move here and never flagged."""
        global last_X
        if not MAX_STEP_MM or last_X is None:
            last_X = x_new
            return True
        d = x_new - last_X
        half = JUNCTION_SPACING_MM * 0.5
        while d < -half:
            d += JUNCTION_SPACING_MM
        while d > half:
            d -= JUNCTION_SPACING_MM
        if d < 0:
            d = -d
        if d > MAX_STEP_MM:
            return False
        last_X = x_new
        return True

    def clear_hist():
        for k in range(256):
            hc[k] = 0
            hr[k] = 0

    def do_scan(buf, th, cols, nc, rows, nr, x0, x1):
        clear_hist()
        col_scan(buf, W, H, th, MIN_LEN, LONG_CUT, cols, nc, CO, MAXPTS,
                 res, hc)
        n_c, bx0, bx1 = res[0], res[1], res[2]
        n_r = int(row_scan(buf, W, H, th, MIN_LEN, LONG_CUT, x0, x1, rows, nr, RO,
                           MAXPTS, hr))
        tc = int(hist_peak(hc, MIN_LEN, LONG_CUT - 1))
        tr = int(hist_peak(hr, MIN_LEN, LONG_CUT - 1))
        tc_cnt = int(hist_peak_count(hc, MIN_LEN, LONG_CUT - 1))
        tr_cnt = int(hist_peak_count(hr, MIN_LEN, LONG_CUT - 1))
        # Sanity anchor -- see NOMINAL_THICK_PX above. A measured peak more
        # than 2x off the known real thickness is almost certainly locked
        # onto teeth/texture, not the bar. tc_cnt/tr_cnt (how many dots
        # supported the peak) are left untouched -- only the width value
        # itself is corrected.
        if tc < (NOMINAL_THICK_PX >> 1) or tc > (NOMINAL_THICK_PX << 1):
            tc = NOMINAL_THICK_PX
        if tr < (NOMINAL_THICK_PX >> 1) or tr > (NOMINAL_THICK_PX << 1):
            tr = NOMINAL_THICK_PX
        return n_c, n_r, tc, tr, bx0, bx1, tc_cnt, tr_cnt

    band = None
    frame_id = 0

    S = [0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    Wn = [0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0]  # [7] = frames with a real X
    last_partial = None
    last_raw_jx = None
    last_gap = None       # (gap_px, gap_mm, n_c) for the debug line
    last_X = None         # last accepted X, for the plausibility check
    n_rejected = 0        # how many readings the check threw out

    def acc_add(A, jx, jy, thd):
        # jx may be None -- the null zone between junctions is a valid
        # state, not a dropped frame. Y and theta keep accumulating; only
        # the X sums skip, counted separately in A[7].
        A[0] += 1
        if jx is not None:
            A[1] += jx
            A[2] += jx * jx
            A[7] += 1
        A[3] += jy
        A[4] += jy * jy
        A[5] += thd
        A[6] += thd * thd

    def acc_clear(A):
        for k in range(8):
            A[k] = 0 if k in (0, 7) else 0.0

    def acc_stats(A):
        n = A[0]
        if n < 1:
            return None
        nx = A[7]
        my = A[3] / n
        mt = A[5] / n
        vy = A[4] / n - my * my
        vt = A[6] / n - mt * mt
        if nx < 1:
            mx = None           # spent the whole window in the null zone
            sx = 0.0
        else:
            mx = A[1] / nx
            vx = A[2] / nx - mx * mx
            sx = math.sqrt(vx) if vx > 0 else 0.0
        return (mx, sx,
                my, math.sqrt(vy) if vy > 0 else 0.0,
                mt, math.sqrt(vt) if vt > 0 else 0.0)

    def estimate_h_only(n_c, tc):
        hlo, hhi = int(tc * THICK_LO), int(tc * THICK_HI)
        fh = fit_line(CO, n_c, hlo, hhi, ITERS)
        if fh is None:
            return None
        if fh[4] > MAX_FIT_MAD_PX:
            return None          # points are scattered, not a bar
        b, a = fh[0], fh[1]
        y_at_center = a + b * (W * 0.5)
        theta = math.degrees(math.atan(b))
        return y_at_center, theta, b, a

    def estimate_v_only(n_r, tr):
        vlo, vhi = int(tr * THICK_LO), int(tr * THICK_HI)
        fv = fit_line(RO, n_r, vlo, vhi, ITERS)
        if fv is None:
            return None
        if fv[4] > MAX_FIT_MAD_PX:
            return None
        b, a = fv[0], fv[1]
        x_at_center = a + b * (H * 0.5)
        theta = -math.degrees(math.atan(b))
        return x_at_center, theta, b, a

    clock = time.clock()
    t_win = time.ticks_ms()
    th = 0
    T = [0, 0, 0, 0, 0, 0, 0]   # cap, gray, otsu, scan+fit, draw, flush, gc
    T_n = 0
    T2 = [0, 0, 0, 0, 0]       # TURN: cap, gray, grad4_angle, draw+flush, stage2
    T2_n = 0

    _uart = None
    if HOST_UART_ID is not None:
        try:
            from machine import UART
            _uart = UART(HOST_UART_ID, HOST_BAUD)
        except Exception as _e:
            print("host UART init failed: %s -- running in bench mode" % _e)
    SM = TurnSM(HostLink(_uart, sim_cmds=SIM_CMDS), W, H)
    print("state machine: %s" % ("UART host" if _uart else "bench mode"))
    if not DRAW:
        print("DRAW = 0: no overlay is drawn (this is the robot setting)."
              " Set DRAW = 1 at the top of the file to see the picture in"
              " the IDE. Overlay + flush costs ~12.6 ms per frame.")

    if BENCH:
        import rot_scan as _rs
        import turn_sm as _ts

        def _stats(v):
            n = len(v)
            if n < 2:
                return 0.0, 0.0
            m = sum(v) / n
            return m, math.sqrt(sum((x - m) * (x - m) for x in v) / n)

        def _timeit(fn, *a):
            fn(*a)
            t0 = time.ticks_us()
            for _ in range(BENCH_REPEATS):
                fn(*a)
            return time.ticks_diff(time.ticks_us(), t0) / BENCH_REPEATS

        print("")
        print("=== BENCH: hold still. capturing %d frames..." % BENCH_FRAMES)
        _bimgs = []
        _bbufs = []
        for _ in range(BENCH_FRAMES):
            _bi = csi0.snapshot()
            _bg = TO_GRAY(_bi) if TO_GRAY else _bi
            _bbufs.append(bytearray(_bg.bytearray()))
        gc.collect()
        _img = csi0.snapshot()
        _gimg = TO_GRAY(_img) if TO_GRAY else _img
        _buf = _bbufs[0]
        _sm = _gimg.copy(x_scale=OTSU_SCALE, y_scale=OTSU_SCALE)
        _th = int(_get(_sm.get_histogram().get_threshold(), "value", -1))
        del _sm
        print("captured. otsu %d  mem_free %d" % (_th, gc.mem_free()))

        # ---------------------------------------------------------- TIME
        # a first full acquire so band / bx0 / bx1 are real for the banded
        # timings below
        (_nc, _nr, _tc, _tr, _bx0, _bx1, _, _) = do_scan(
            _buf, _th, allc, NAC, allr, NAR, 0, W)
        _band = (max(0, _bx0 - EXTRA), min(W, _bx1 + EXTRA)) \
            if _bx1 >= 0 else (0, W)

        def _otsu():
            sm = _gimg.copy(x_scale=OTSU_SCALE, y_scale=OTSU_SCALE)
            v = int(_get(sm.get_histogram().get_threshold(), "value", -1))
            del sm
            return v

        def _acq():
            return do_scan(_buf, _th, allc, NAC, allr, NAR, 0, W)

        def _trk():
            return do_scan(_buf, _th, ca, NC, ra, NR, _band[0], _band[1])

        def _fit():
            return estimate(_nc, _nr, _tc, _tr)

        def _g4():
            return _ts.grad4_angle(_buf, W, H)

        def _s2(step, span, half):
            return _rs.fit_frame(_buf, W, H, _th, 0.0, step, span, half)

        def _overlay():
            _img.draw_line((0, 100, W - 1, 100), color=COLOR_X)
            _img.draw_line((160, 0, 160, H - 1), color=COLOR_Y)
            _img.draw_cross((160, 100), color=COLOR_MARK, size=6)
            _img.draw_string((2, 2), "X+00.0 Y+00.0 T+00.0 DRV",
                             color=COLOR_MARK)
            hud.draw_gauge(_img, W, H, 0.0, 0.0)

        def _flush():
            csi0.flush()

        rows = []
        rows.append(("otsu (downscaled)", _timeit(_otsu)))
        rows.append(("scan, ACQUIRE (step %d, full width)" % ACQUIRE_STEP,
                     _timeit(_acq)))
        rows.append(("scan, TRACKED (step %d, banded)" % STEP, _timeit(_trk)))
        rows.append(("fit + weighted fit", _timeit(_fit)))
        rows.append(("grad4_angle (step %d tap %d)"
                     % (_ts.GRAD_STEP, _ts.GRAD_TAP), _timeit(_g4)))
        rows.append(("stage2 fit_frame (step %d span %d half %d)"
                     % (_rs.SWEEP_STEP, _rs.SWEEP_SPAN, _rs.SWEEP_HALF_LEN),
                     _timeit(_s2, _rs.SWEEP_STEP, _rs.SWEEP_SPAN,
                             _rs.SWEEP_HALF_LEN)))
        rows.append(("overlay draw (lines+text+gauge)", _timeit(_overlay)))
        rows.append(("flush (USB framebuffer push)", _timeit(_flush)))

        # capture cost: what snapshot() blocks for
        _t0 = time.ticks_us()
        for _ in range(BENCH_REPEATS):
            csi0.snapshot()
        _cap = time.ticks_diff(time.ticks_us(), _t0) / BENCH_REPEATS
        rows.insert(0, ("capture (snapshot blocks for)", _cap))

        _drive_frame = _cap + rows[1][1] + rows[3][1] + rows[4][1]
        _turn_frame = _cap + rows[5][1] + rows[6][1] / _ts.STAGE2_EVERY
        print("")
        print("TIME (us per call, %d repeats, first discarded)" % BENCH_REPEATS)
        print("%-46s %9s" % ("stage", "us"))
        print("-" * 58)
        for nm, us in rows:
            print("%-46s %9.0f" % (nm, us))
        print("-" * 58)
        print("%-46s %9.0f  = %.0f fps"
              % ("DRIVE frame, tracked (cap+scan+fit+grad4)",
                 _drive_frame, 1e6 / _drive_frame))
        print("%-46s %9.0f  = %.0f fps"
              % ("DRIVE frame, acquiring (cap+ACQ scan+fit+grad4)",
                 _cap + rows[2][1] + rows[4][1] + rows[5][1],
                 1e6 / (_cap + rows[2][1] + rows[4][1] + rows[5][1])))
        print("%-46s %9.0f  = %.0f fps"
              % ("TURN frame (cap+grad4+stage2/%d)" % _ts.STAGE2_EVERY,
                 _turn_frame, 1e6 / _turn_frame))
        print("%-46s %9.0f  (DRAW=1 only)"
              % ("  + overlay + flush", rows[7][1] + rows[8][1]))

        # ---------------------------------------------------------- NOISE
        Xs, Ys, Ts, G4, S2x, S2y, S2t = [], [], [], [], [], [], []
        for b in _bbufs:
            (nc, nr, tc, tr, bx0, bx1, _, _) = do_scan(
                b, _th, allc, NAC, allr, NAR, 0, W)
            if (bx1 >= 0 and (bx1 - bx0) >= MIN_V_SPAN_PX and tc >= MIN_LEN
                    and tr >= MIN_LEN and nc >= MIN_PTS and nr >= MIN_PTS):
                out = estimate(nc, nr, tc, tr)
                if out is not None:
                    jx, jy, thd = out[0], out[1], out[2]
                    Xs.append((jx - W * 0.5) * MM_PER_PIXEL)
                    Ys.append((jy - H * 0.5) * MM_PER_PIXEL)
                    Ts.append(thd)
            G4.append(_ts.grad4_angle(b, W, H)[0])
            f = _rs.fit_frame(b, W, H, _th, G4[-1])
            if f is not None and f["junction"] is not None:
                S2x.append(f["junction"][0] * MM_PER_PIXEL)
                S2y.append(f["junction"][1] * MM_PER_PIXEL)
                S2t.append(f["theta"])
        print("")
        print("NOISE over %d STATIC frames (sd = the pipeline's own noise)"
              % BENCH_FRAMES)
        print("%-30s %6s %10s %10s" % ("output", "n", "mean", "sd"))
        print("-" * 60)
        for nm, v in (("drive X (mm)", Xs), ("drive Y (mm)", Ys),
                      ("drive T (deg)", Ts), ("grad4 angle (deg)", G4),
                      ("stage2 X (mm)", S2x), ("stage2 Y (mm)", S2y),
                      ("stage2 T (deg)", S2t)):
            m, sd = _stats(v)
            print("%-30s %6d %+10.3f %10.4f" % (nm, len(v), m, sd))

        # ---------------------------------------------------------- VARIANTS
        print("")
        print("VARIANTS -- same frames, cheaper settings")
        print("%-40s %9s %10s %12s" % ("setting", "us", "sd", "vs default"))
        print("-" * 74)
        ref = _stats(G4)[0]
        for st, tp in ((3, 3), (5, 3), (7, 3), (8, 3), (5, 2)):
            v = [_ts.grad4_angle(b, W, H, st, tp)[0] for b in _bbufs]
            m, sd = _stats(v)
            us = _timeit(lambda: _ts.grad4_angle(_buf, W, H, st, tp))
            print("%-40s %9.0f %10.4f %+12.3f"
                  % ("grad4 step %d tap %d" % (st, tp), us, sd, m - ref))
        ref = _stats(S2t)[0]
        for st, sp, hf in ((_rs.SWEEP_STEP, _rs.SWEEP_SPAN, _rs.SWEEP_HALF_LEN),
                           (8, 260, 120), (10, 260, 120), (12, 220, 100)):
            v = []
            for k in range(len(_bbufs)):
                f = _rs.fit_frame(_bbufs[k], W, H, _th, G4[k], st, sp, hf)
                if f is not None and f["junction"] is not None:
                    v.append(f["theta"])
            m, sd = _stats(v)
            us = _timeit(_s2, st, sp, hf)
            print("%-40s %9.0f %10.4f %+12.3f"
                  % ("stage2 step %d span %d half %d" % (st, sp, hf), us, sd,
                     m - ref))
        print("")
        print("=== BENCH done. Set BENCH = 0 to drive. ===")
        raise SystemExit

    while True:
        clock.tick()
        try:
            _t0 = time.ticks_us()
            img = csi0.snapshot()
            _t1 = time.ticks_us()
            # gimg is what every scan/fit below reads: 1 byte per pixel.
            # img stays RGB565 and only ever gets drawn on.
            gimg = TO_GRAY(img) if TO_GRAY else img
            buf = gimg.bytearray()
            _t2 = time.ticks_us()

            _now = time.ticks_ms()

            # ---- ANGLE GAUGE: drawn FIRST, every single frame -----------
            # It has to go before anything else draws, because each drive
            # branch below ends with its own csi0.flush(). Anything drawn
            # after that lands on a frame already sent, which is why the
            # gauge kept disappearing. SM.heading() gives the live value
            # while the rotation layer tracks and the last known value
            # otherwise, so the wheel is permanent and never blinks.
            if DRAW:
                _ph, _om = SM.heading()
                hud.draw_gauge(img, W, H, _ph, _om)

            SM.poll_cmd(frame_id, _now)           # hook 1: host commands
            _fix = None
            if not SM.alg1_active():
                # ---- Phase B: TURN. Algorithm 2 only. No Otsu, no scan.
                _t3 = time.ticks_us()
                SM.turn_frame(buf, _now)          # hook 2 -- Alg 2's own cost
                _t4 = time.ticks_us()
                if DRAW:
                    # ---- ROTATION OVERLAY = THE DRIVE OVERLAY -----------
                    # Same tape lines, same junction cross, same top-left
                    # "X Y T" line, same place, same format. Only the
                    # SOURCE differs: Algorithm 1 is not running here, so
                    # X, Y and theta come from Stage 2's tilted fit instead
                    # of the axis-aligned one. The gauge is already on the
                    # frame -- it is drawn at the top of the loop, every
                    # frame, in both modes.
                    #
                    # hud.draw_turn() is no longer used: it drew a
                    # three-line panel at the BOTTOM plus its own second
                    # gauge, which is why rotation looked nothing like
                    # drive.
                    #
                    # SM.fit is HELD across turns -- turn_sm never clears
                    # it at entry -- so without the last_fit_frame guard
                    # the first frames of a turn would show the PREVIOUS
                    # turn's junction as if it were current.
                    _rf = None
                    _tt = SM.turn
                    if (_tt is not None and SM.fit is not None
                            and _tt.get("last_fit_frame", 0) >= 0):
                        _rf = SM.fit
                    if _rf is not None:
                        hud.draw_tape_lines(img, W, H, _rf["theta"],
                                            _rf["xline"], _rf["yline"],
                                            _rf["junction"])
                    _rx = None
                    _ry = None
                    _rt = None
                    if _rf is not None and _rf["junction"] is not None:
                        _al, _be = _rf["junction"]
                        # Stage 2 returns the junction as an offset from the
                        # frame centre, in px. signed_x() wants a pixel
                        # column, so hand it the centre plus that offset --
                        # then X obeys exactly the same 0/-30/null/+30
                        # convention here as in drive, X_DIRECTION included,
                        # instead of being a second scheme.
                        _rx = signed_x(W * 0.5 + _al, MM_PER_PIXEL)
                        _ry = _be * MM_PER_PIXEL
                        # turn_sm v2 aims Stage 2 from the continuous
                        # angle, so the fit's theta can read -88 mid-turn.
                        # The LINE DRAWING above needs that value; the
                        # number on screen is the tape tilt, so fold it.
                        _rt = wrap90(_rf["theta"])
                    img.draw_string((2, 2), hud_line(_rx, _ry, _rt, "ROT"),
                                    color=COLOR_MARK)
                if DRAW or FLUSH:
                    csi0.flush()
                _t5 = time.ticks_us()
                if SM.state == 1 or SM.state == 2:   # TURN_INIT / TURNING only
                    T2[0] += time.ticks_diff(_t1, _t0)      # capture
                    T2[1] += time.ticks_diff(_t2, _t1)      # gray
                    T2[4] += SM.stage2_us                   # Stage 2 (tilted scan)
                    T2[2] += (time.ticks_diff(_t4, _t3)
                              - SM.stage2_us)               # grad4_angle alone
                    T2[3] += time.ticks_diff(_t5, _t4)      # draw+flush
                    SM.stage2_us = 0
                    T2_n += 1
            else:
                # ---- Phase A: DRIVE / REACQUIRE. Algorithm 1, unchanged.
                if SM.reseed_pending:
                    # back from a turn: the image axes now look at the
                    # other floor tape. Everything axis-specific goes.
                    band = None
                    last_X = None          # one frame without MAX_STEP gate
                    last_partial = None
                    last_raw_jx = None
                    last_gap = None
                    acc_clear(Wn)
                    n_rejected = 0
                    SM.reseed_pending = False
                if SM.force_full_scan():
                    band = None            # full ACQUIRE_STEP scan each frame
                if OTSU_EVERY <= 1 or frame_id % OTSU_EVERY == 0 or th == 0:
                    sm = gimg.copy(x_scale=OTSU_SCALE, y_scale=OTSU_SCALE)
                    th = int(V(sm.get_histogram().get_threshold(), "value"))
                    del sm
                _t3 = time.ticks_us()

                status = "track"
                col_step = 1
                if band is None:
                    status = "acquire"
                    (n_c, n_r, tc, tr, bx0, bx1,
                     tc_cnt, tr_cnt) = do_scan(buf, th, allc, NAC, allr, NAR, 0, W)
                    nc_used, nr_used = NAC, NAR
                    col_step = 1
                else:
                    x0, x1 = band
                    (n_c, n_r, tc, tr, bx0, bx1,
                     tc_cnt, tr_cnt) = do_scan(buf, th, ca, NC, ra, NR, x0, x1)
                    nc_used, nr_used = NC, NR
                    col_step = STEP

                    # BUG 3 FIX. Between two junctions bx1 is -1 by
                    # definition, so the old unconditional re-acquire ran a
                    # second, full-width scan on EVERY frame of the mini
                    # region. Only escalate if the cheap scan cannot even
                    # support the horizontal-only fallback -- or on a slow
                    # cadence, to notice a vertical bar coming back.
                    h_usable = (tc >= MIN_LEN and n_c >= MIN_PTS
                                and n_c >= nc_used * COVERAGE_FRAC)
                    need_full = (bx1 < 0 or n_c < MIN_PTS or n_r < MIN_PTS)
                    if need_full and (not h_usable
                                      or frame_id % REACQUIRE_MINI_EVERY == 0):
                        status = "reacquire"
                        (n_c, n_r, tc, tr, bx0, bx1,
                         tc_cnt, tr_cnt) = do_scan(buf, th, allc, NAC,
                                                   allr, NAR, 0, W)
                        nc_used, nr_used = NAC, NAR
                        col_step = 1
                    elif (not need_full) and frame_id % REACQUIRE_EVERY == 0:
                        (n_c2, n_r2, tc2, tr2, bx02, bx12,
                         tc_cnt2, tr_cnt2) = do_scan(buf, th, allc, NAC,
                                                     allr, NAR, 0, W)
                        if bx12 >= 0:
                            n_c, n_r, tc, tr, bx0, bx1 = (n_c2, n_r2, tc2,
                                                          tr2, bx02, bx12)
                            tc_cnt, tr_cnt = tc_cnt2, tr_cnt2
                            nc_used, nr_used = NAC, NAR
                            col_step = 1
                            status = "periodic"

                # Full-junction validity: the original, proven check.
                _t4 = time.ticks_us()
                # Two gates added in v6, both measured off middle4.mp4,
                # where the junction path was firing on lens glare at the
                # frame edge ~47% of the time and reporting a confident,
                # wrong X. That -- not the mini logic -- was the flicker.
                # Both gates are physical, not tuned: a vertical bar spans
                # several columns, and it is the same tape as the
                # horizontal bar so it must be about as wide.
                ok = (bx1 >= 0 and (bx1 - bx0) >= MIN_V_SPAN_PX
                      and tc >= MIN_LEN and tr >= MIN_LEN
                      and n_c >= MIN_PTS and n_r >= MIN_PTS
                      and tr >= tc * THICK_MATCH_FRAC)

                out = None
                if ok:
                    band = (max(0, bx0 - EXTRA), min(W, bx1 + EXTRA))
                    out = estimate(n_c, n_r, tc, tr)
                    if out is None:
                        ok = False

                if ok:
                    # ---- tier 1: full junction (both bars). UNCHANGED.
                    # BUG 1 FIX: the stray `y_at_mid = a + b * mid_px` /
                    # draw_cross block that used to sit here referred to
                    # names that only exist in the mini branch, so this
                    # whole branch raised NameError on every junction.
                    jx, jy, thd, hv, bh, ah, bv, av = out
                    mmpx = mm_scale(tc)
                    # same quantity the mini path reports: distance to the
                    # junction AHEAD. If the junction we just fitted is
                    # behind us, the one ahead is a full cell further on.
                    X_mm = signed_x(jx, mmpx)      # may be None in the
                                                   # null zone -- that is a
                                                   # valid answer, not a
                                                   # failure.
                    Y_mm = (jy - H * 0.5) * mmpx
                    sus = X_mm is not None and not plausible(X_mm)
                    if sus:
                        n_rejected += 1
                    else:
                        acc_add(Wn, X_mm, Y_mm, thd)
                        _fix = (1, X_mm, Y_mm, thd)
                    last_partial = None
                    last_gap = None
                    last_raw_jx = jx
                    if TILT_DEBUG and frame_id % 60 == 0:
                        _pl, _pr = tilt_probe(n_c, tc, jx)
                        if _pl[0] is not None and _pr[0] is not None:
                            print("  TILT  left %+.2fdeg (n%d)  right %+.2fdeg"
                                  " (n%d)  combined %+.2fdeg   step %+.1fpx"
                                  % (_pl[0], _pl[2], _pr[0], _pr[2], thd,
                                     _pr[1] - _pl[1]))
                        else:
                            print("  TILT  not enough points one side:"
                                  " left n%d  right n%d" % (_pl[2], _pr[2]))
                    if DRAW:

                        img.draw_line((0, int(ah + 0.5),
                                   W - 1, int(ah + bh * (W - 1) + 0.5)),
                                  color=COLOR_X)           # x line
                    if DRAW:

                        img.draw_line((int(av + 0.5), 0,
                                   int(av + bv * (H - 1) + 0.5), H - 1),
                                  color=COLOR_Y)           # y line
                    if DRAW:

                        img.draw_cross((int(jx + 0.5), int(jy + 0.5)),
                                   color=(255, 255, 255), size=8)
                    if DRAW:

                        # X_mm is None inside the null zone. "%+.1f" % None
                        # raises, the frame try/except swallows it as a
                        # FRAME ERROR, and the whole frame is lost -- which
                        # is why X and Y disappeared from the overlay there.
                        img.draw_string((2, 2),
                                    hud_line(X_mm, Y_mm, thd, "DRV"),
                                    color=COLOR_MARK)
                    if DRAW or FLUSH:
                        csi0.flush()
                else:
                    band = None
                    # tc_cnt/THICKNESS_FRAC deliberately NOT used here any
                    # more -- see MAX_FIT_MAD_PX. The fit residual test
                    # inside estimate_h_only does this job properly.
                    h_ok_fb = (tc >= MIN_LEN and n_c >= MIN_PTS
                               and n_c >= nc_used * COVERAGE_FRAC)
                    v_ok_fb = (bx1 >= 0 and tr >= MIN_LEN and n_r >= MIN_PTS
                               and n_r >= nr_used * COVERAGE_FRAC)
                    if h_ok_fb:
                        # ---- tier 2/3: horizontal bar only.
                        r = estimate_h_only(n_c, tc)
                        if r is not None:
                            # keep the cheap STEP-spaced scan alive
                            band = (0, W)
                            y_px, thd, b, a = r
                            mmpx = mm_scale(tc)
                            Y_mm = (y_px - H * 0.5) * mmpx
                            # the "x line": drawn in BOTH sub-cases, since
                            # the horizontal fit is what we do have.
                            if DRAW:

                                img.draw_line((0, int(a + 0.5),
                                           W - 1, int(a + b * (W - 1) + 0.5)),
                                          color=COLOR_X)
                            # NO vertical line is drawn here, by design --
                            # there is no vertical bar to draw.

                            # ---- mini junction REMOVED -------------------
                            # It existed to synthesise an X from the 40 mm
                            # void when no vertical bar was visible. Under
                            # the new scheme that region IS the null zone --
                            # X is legitimately absent there, so inventing
                            # one is exactly the wrong thing to do. The
                            # horizontal (X) bar is still fitted and still
                            # drawn above; only the number is withheld.
                            last_partial = ("Y", Y_mm, thd)
                            last_gap = None
                            if DRAW:
                                # The X line is ALREADY drawn a few lines
                                # above from (a, b). I added a second draw
                                # here using ah/bh -- names that do not
                                # exist in this scope -- so every null-zone
                                # frame raised NameError, got swallowed by
                                # the frame try/except, and was lost whole.
                                # That is why the line disappeared. Removed.
                                img.draw_string((2, 2),
                                                hud_line(None, Y_mm, thd,
                                                         "DRV"),
                                                color=COLOR_MARK)
                            if DRAW or FLUSH:
                                csi0.flush()
                        else:
                            last_partial = None
                    elif v_ok_fb:
                        r = estimate_v_only(n_r, tr)
                        if r is not None:
                            x_px, thd, b, a = r
                            mmpx = mm_scale(tr)
                            X_mm = (x_px - W * 0.5) * mmpx
                            last_partial = ("X", X_mm, thd)
                            last_gap = None
                            if DRAW:

                                img.draw_line((int(a + 0.5), 0,
                                           int(a + b * (H - 1) + 0.5), H - 1),
                                          color=COLOR_Y)
                            if DRAW:

                                # same None guard as the junction branch
                                img.draw_string((2, 2),
                                hud_line(X_mm, None, thd, "DRV"),
                                color=COLOR_MARK)
                            if DRAW or FLUSH:
                                csi0.flush()
                        else:
                            last_partial = None
                    else:
                        last_partial = None
                        last_gap = None
                _t5 = time.ticks_us()
                SM.drive_result(_fix, _now)       # hook 3

            T[0] += time.ticks_diff(_t1, _t0)
            T[1] += time.ticks_diff(_t2, _t1)
            T[2] += time.ticks_diff(_t3, _t2)
            T[3] += time.ticks_diff(_t4, _t3)
            T[4] += time.ticks_diff(_t5, _t4)
            T_n += 1
        except KeyboardInterrupt:
            # The IDE's STOP button. Landing mid-function is normal.
            print("stopped by user.")
            break
        except Exception as e:
            print("FRAME ERROR: %s: %s   frame=%d  mem_free=%d"
                  % (type(e).__name__, e, frame_id, gc.mem_free()))
            out = None
            band = None
            gc.collect()

        frame_id += 1

        now = time.ticks_ms()
        wdt = time.ticks_diff(now, t_win)
        if wdt >= SUMMARY_MS:
            # ---- ONE STATUS LINE PER SECOND ------------------------------
            # Same fields in both modes, spelled out, so a host (or a
            # person) can read it without a decoder:
            #   frame N  <tier>  X..  Y..  THETA..  DRV|ROT  fps..
            # X/Y/THETA are "--" when not measured. fps is the CAMERA rate:
            # frame time with draw+flush excluded, i.e. what the robot gets
            # with DRAW = 0, not what the IDE shows with it on.
            _fps_cam = 0.0
            if T2_n and not SM.alg1_active():
                _tt = (T2[0] + T2[1] + T2[2] + T2[3] + T2[4]) // T2_n
                _ttcam = _tt - (T2[3] // T2_n)
                _fps_cam = 1e6 / _ttcam if _ttcam else 0.0
            elif T_n:
                _tot = (T[0] + T[1] + T[2] + T[3] + T[4] + T[6]) // T_n
                _camtot = _tot - (T[4] // T_n)
                _fps_cam = 1e6 / _camtot if _camtot else 0.0

            if not SM.alg1_active():
                # rotation: X/Y from Stage 2, THETA = the continuous heading
                _tt_ = SM.turn
                _rx = None
                _ry = None
                if (_tt_ is not None and SM.fit is not None
                        and _tt_.get("last_fit_frame", 0) >= 0
                        and SM.fit["junction"] is not None):
                    _al, _be = SM.fit["junction"]
                    _rx = signed_x(W * 0.5 + _al, MM_PER_PIXEL)
                    _ry = _be * MM_PER_PIXEL
                _phi = _tt_["phi"] if _tt_ is not None else None
                _om = _tt_["omega"] if _tt_ is not None else None
                print("frame %d  %s  %s  %s  ROTATION_PER_SEC%s  ROT  fps%.0f"
                      % (frame_id,
                         "X--" if _rx is None else "X%+.1f" % _rx,
                         "Y--" if _ry is None else "Y%+.1f" % _ry,
                         "THETA--" if _phi is None else "THETA%+.1f" % _phi,
                         "--" if _om is None else "%+.1f" % _om,
                         _fps_cam))
            else:
                st = acc_stats(Wn)
                if st is not None:
                    _tier, _x, _y, _t = "junction", st[0], st[2], st[4]
                elif last_partial is not None:
                    kind, val, thd = last_partial
                    if kind == "Y":
                        _tier, _x, _y, _t = "Y-only", None, val, thd
                    else:
                        _tier, _x, _y, _t = "X-only", val, None, thd
                else:
                    _tier, _x, _y, _t = "none", None, None, None
                print("frame %d  %s  %s  %s  %s  DRV  fps%.0f"
                      % (frame_id, _tier,
                         "X--" if _x is None else "X%+.1f" % _x,
                         "Y--" if _y is None else "Y%+.1f" % _y,
                         "THETA--" if _t is None else "THETA%+.1f" % _t,
                         _fps_cam))

            # ---- events, only when they happened ------------------------
            if n_rejected:
                print("   %d reading(s) rejected as physically impossible"
                      " (>%.0fmm since the last fix) -- see MAX_STEP_MM"
                      % (n_rejected, MAX_STEP_MM))
                n_rejected = 0

            # ---- PERF = 1: where the frame time goes -------------------
            if PERF and T_n:
                _tot = (T[0] + T[1] + T[2] + T[3] + T[4] + T[6]) // T_n
                _camtot = _tot - (T[4] // T_n)
                print("   drive us/frame  capture %d  gray %d  otsu %d"
                      "  scan+fit %d  draw+flush %d  gc %d   total %d"
                      "  -> %.1f fps IDE, %.1f fps camera"
                      % (T[0] // T_n, T[1] // T_n, T[2] // T_n,
                         T[3] // T_n, T[4] // T_n, T[6] // T_n, _tot,
                         1e6 / _tot if _tot else 0,
                         1e6 / _camtot if _camtot else 0))
            if PERF and T2_n:
                _tt = (T2[0] + T2[1] + T2[2] + T2[3] + T2[4]) // T2_n
                _ttcam = _tt - (T2[3] // T2_n)
                print("   turn  us/frame  capture %d  gray %d  grad4 %d"
                      "  stage2 %d  draw+flush %d   total %d"
                      "  -> %.1f fps IDE, %.1f fps camera"
                      % (T2[0] // T2_n, T2[1] // T2_n, T2[2] // T2_n,
                         T2[4] // T2_n, T2[3] // T2_n, _tt,
                         1e6 / _tt if _tt else 0,
                         1e6 / _ttcam if _ttcam else 0))
            for _k in range(7):
                T[_k] = 0
            T_n = 0
            for _k in range(5):
                T2[_k] = 0
            T2_n = 0
            acc_clear(Wn)
            t_win = now

        if GC_EVERY <= 1 or frame_id % GC_EVERY == 0:
            _g0 = time.ticks_us()
            gc.collect()
            T[6] += time.ticks_diff(time.ticks_us(), _g0)
