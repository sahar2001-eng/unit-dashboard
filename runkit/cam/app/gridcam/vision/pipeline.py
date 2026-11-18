# Grid (X, Y, theta) detection pipeline — VENDORED, see docs/pipeline_sync.md.
#
# Port of reference/scan_fit_fps_live.py (delivered as v1_2 on 2026-09-09;
# method doc reference/pipeline_v2-2_method.pdf). This is Algorithm 1; the
# turn machinery from the same drop lives in vision/turn/.
# Only the algorithm came across: camera setup, the overlay and the serial
# summary stayed in the POC.
#
# Stages (per frame):
#   1. Otsu threshold on a quarter-scale copy — computed by the CALLER
#      (Camera.otsu) because it needs the image API; cached every
#      cfg['otsu_every'] frames.
#   2. viper column scan of the whole frame -> sub-pixel run centres (CO),
#      run-length histogram (tape thickness = histogram peak), and the x-band
#      of any LONG runs (the vertical bar's location, for free).
#   3. viper row scan restricted to that band -> RO.
#   4. Integer least squares, `iters` MAD-gated rounds, then a Gaussian
#      junction-weighted refit (sigma `wsigma` px) for the position.
#   5. Tiers: full junction (both bars) -> Y+theta only -> X only. The
#      mini-junction tier was REMOVED UPSTREAM in v1_2.
#   6. X is SIGNED against the nearer junction and is None in the dead zone
#      mid-cell: 0 -> -30 leaving it, None, +30 -> 0 approaching the next
#      (see signed_x). Y = junction / bar offset from the frame centre. Both
#      in mm using the mm/px the caller passes (config camera.pixels_per_80mm).
#      A None X is a VALID reading meaning "between junctions", distinct from
#      `suspect`, which means a reading was computed and then refused.
#
# STATEFUL: band tracking (STEP-spaced scan inside the last band), periodic
# reacquire, and a physical-plausibility gate on X between frames. One
# GridTracker per camera; build a fresh one on a pipeline.* config change.
#
# The eight viper routines plus spaced/solve/solve_w are the SHARED surface
# host/sync_pipeline.py diffs against the POC — keep them verbatim. The class
# below is the POC's nested closures and loop body restructured; it is merged
# by hand (reference/pipeline_sync.json "restructured" lists the mapping).
#
# viper ints are 32-bit and wrap silently. Worst accumulator is wlsq's
# sduu = sum(w*du*du): max per point 64*du^2*exp(-(du/30)^2) ~ 21000 at
# du=30, times MAXPTS 2000 = 4.2e7 — well inside int32. lsq's suu at
# u<320, n<=2000 is 2e8. MAXPTS / WMAX / WTAB_N are therefore module constants,
# not tunables: raising them can overflow the fixed-point maths on target
# while the desktop tests (plain Python ints, no wrap) stay green.

import math
from array import array

# The decorator MUST be spelled `@micropython.viper` literally: MicroPython's
# compiler recognises that syntax and emits native code; an alias such as
# `_viper = micropython.viper` is applied at run time as a no-op and the
# kernels silently run as bytecode (measured on the AE3: 259 ms/frame instead
# of ~4 ms, with the VIPER probe still reporting True). On CPython without
# the desktop mock a stub module keeps the module importable.
try:
    import micropython
except ImportError:                          # bare CPython: no viper anywhere
    import sys as _sys
    import types as _types
    micropython = _types.SimpleNamespace(viper=lambda f: f, native=lambda f: f)
    _sys.modules['micropython'] = micropython

try:
    import time
    _us = time.ticks_us
    _us_diff = time.ticks_diff
except AttributeError:                       # CPython (desktop tests)
    def _us():
        return int(time.monotonic() * 1000000)

    def _us_diff(a, b):
        return a - b


def _mark(prof, key, t0):
    now = _us()
    prof[key] = prof.get(key, 0) + _us_diff(now, t0)
    return now


# Fixed-point / buffer-sizing constants. Not tunables — see the header.
MAD_SHIFT = 9
MAD_BINS = 256
WTAB_N = 256
WMAX = 64
NOGATE = 1 << 28
MAXPTS = 2000
# LOCAL (not a POC constant): frames without an accepted X after which the
# jump gate reseeds rather than judging the next fix against a stale one. 2
# frames is ~25 mm at 1 m/s, so a single dropped frame keeps the gate armed.
PLAUSIBLE_GAP_FRAMES = 2

# Probe viper the way the POC does: a firmware without it must be refused at
# GridTracker() rather than silently running the scans as interpreted Python
# (~250 ms/frame). On the desktop the micropython mock makes this pass.
try:
    @micropython.viper
    def _p(x: int) -> int:
        return x + 1
    _p(1)
    VIPER = True
    VIPER_ERR = ''
except Exception as _e:
    VIPER = False
    VIPER_ERR = str(_e)


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

# LOCAL (not in the POC): drop scan points whose run touches the frame edge.
# out[] holds (line, centre*16, length) triples. A run that starts on the
# first pixel or ends on the last one was cut off by the frame: its real
# length and centre are unknown, so it is not a measurement. Without this,
# a vertical arm running off the TOP of the frame is 40-42 px long -- inside
# the tape-width window and under long_cut -- and the column scan took it as
# the horizontal band, 60 px from the real one (robot 0302, 2026-11-01: the
# band fit failed and the frame gave no result). hi16 = (last pixel)*16.
@micropython.viper
def drop_cut_runs(out: ptr32, n: int, hi16: int) -> int:
    k = 0
    i = 0
    while i < n:
        c = out[3 * i + 1]
        h8 = out[3 * i + 2] << 3
        if c - h8 > 8 and c + h8 < hi16 - 8:
            out[3 * k] = out[3 * i]
            out[3 * k + 1] = c
            out[3 * k + 2] = out[3 * i + 2]
            k += 1
        i += 1
    return k

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


# LOCAL (gridcam 1.27.0, not a POC change): the 8 kernels above can run as C.
# openmv/bench/native/t1c/t1c.c holds them under the same names and argument
# order; the bench proved them byte-equal to these exact viper bodies
# (bench test t02). tests/desktop/test_c_kernels.py fails if any of them
# changes here without t1c.c. The call sites look the names up as module
# globals at call time, so use_kernels() just rebinds them. drop_cut_runs is
# gridcam's own (1.25.3) and has no C twin: it stays viper.
C_KERNELS = ('col_scan', 'row_scan', 'hist_peak', 'hist_peak_count', 'lsq',
             'resid_hist', 'hist_median', 'wlsq')
T1C_API = 1                                  # t1c.API this file expects
_VIPER = {k: globals()[k] for k in C_KERNELS}
_VIPER_DROP = drop_cut_runs
KERNELS = 'viper'                            # what runs now: 'c' or 'viper'
KERNELS_ERR = ''                             # why C was refused, '' if not


def _import_t1c():
    """The t1c .mpy: next to this file in the app slot, else top-level."""
    try:
        from . import t1c
    except ImportError:
        import t1c
    return t1c


def use_kernels(c, mod=None):
    """Run the scan/fit kernels as C (c=True) or viper. Returns what runs.

    A missing t1c, a wrong build or a wrong API keeps viper and leaves the
    reason in KERNELS_ERR; it never raises."""
    global KERNELS, KERNELS_ERR
    g = globals()
    KERNELS_ERR = ''
    if c:
        try:
            m = mod if mod is not None else _import_t1c()
            api = getattr(m, 'API', None)
            if api != T1C_API:
                raise ValueError('t1c API %s, need %d' % (api, T1C_API))
            fns = [getattr(m, k) for k in C_KERNELS]
        except Exception as e:               # ImportError, ValueError (.mpy arch)
            KERNELS_ERR = str(e) or e.__class__.__name__
            c = False
    for i, k in enumerate(C_KERNELS):
        g[k] = fns[i] if c else _VIPER[k]
    KERNELS = 'c' if c else 'viper'
    return KERNELS


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


TIER_JUNCTION = 'junction'
TIER_Y = 'y'
TIER_X = 'x'


class GridTracker:
    """Per-camera pipeline state. See the module header.

    analyse(buf, th, mm_per_px, prof) is the per-frame entry point; buf is
    the grayscale framebuffer (1 byte/px, w*h), th the Otsu threshold.
    """

    def __init__(self, w, h, cfg):
        if not VIPER:
            raise RuntimeError('viper unavailable: %s' % VIPER_ERR)
        self.W = w
        self.H = h
        self.cfg = cfg
        self.CO = array('i', [0] * (MAXPTS * 3))
        self.RO = array('i', [0] * (MAXPTS * 3))
        self.res = array('i', [0, 0, 0])
        self.acc = array('i', [0] * 8)
        self.prm = array('i', [0] * 8)
        self.mh = array('i', [0] * MAD_BINS)
        self.hc = array('i', [0] * 256)
        self.hr = array('i', [0] * 256)

        self.WTAB = array('i', [0] * WTAB_N)
        wsigma = float(cfg['wsigma'])
        for d in range(WTAB_N):
            self.WTAB[d] = int(WMAX * math.exp(-((d / wsigma) ** 2)) + 0.5)

        acl = spaced(cfg['acquire_step'], w)
        arl = spaced(cfg['acquire_step'], h)
        self.allc = array('i', acl)
        self.allr = array('i', arl)
        self.NAC, self.NAR = len(acl), len(arl)
        cl = spaced(cfg['step'], w)
        rl = spaced(cfg['step'], h)
        self.ca = array('i', cl)
        self.ra = array('i', rl)
        self.NC, self.NR = len(cl), len(rl)
        self.reset()

    def reset(self):
        """Forget the tracked band and the last X (stream restart, config)."""
        self.band = None
        self.frame_id = 0
        self.last_X = None
        self.last_X_frame = 0
        self.n_rejected = 0
        self.status = ''
        self.mad_h = None
        self.mad_v = None
        self.est_why = None

    # The two turn hooks the vision loop calls (Tile 2's tracker has its
    # own; vision_service no longer reaches into a tracker's fields).

    def reseed(self):
        """Back from a turn: the band and last X describe a world that has
        rotated 90 degrees away, so forget them."""
        self.reset()

    def force_full(self):
        """REACQUIRE: scan the whole frame, not the tracked band."""
        self.band = None

    # ---- ported from the POC's nested functions (self.* instead of globals)

    def fit_line(self, arr, n, lo, hi, iters, reject):
        acc, mh = self.acc, self.mh
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

    def fit_weighted(self, arr, n, lo, hi, b, a, gate, uc):
        prm = self.prm
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
        if int(wlsq(arr, n, prm, self.WTAB, self.acc)) < 4:
            return None
        return solve_w(self.acc, uci, vc)

    def estimate(self, n_c, n_r, tc, tr):
        c = self.cfg
        hlo, hhi = int(tc * c['thick_lo']), int(tc * c['thick_hi'])
        vlo, vhi = int(tr * c['thick_lo']), int(tr * c['thick_hi'])
        fh = self.fit_line(self.CO, n_c, hlo, hhi, c['iters'], c['reject'])
        fv = self.fit_line(self.RO, n_r, vlo, vhi, c['iters'], c['reject'])
        if fh is None or fv is None:
            self.est_why = 'fit_pts'
            return None
        self.mad_h, self.mad_v = fh[4], fv[4]
        if fh[4] > c['max_joint_mad_px'] or fv[4] > c['max_joint_mad_px']:
            self.est_why = 'fit_mad'
            return None      # blurred / contaminated -- let the
                             # single-bar fallback handle this frame
        den = 1.0 - fh[0] * fv[0]
        if abs(den) < 1e-9:
            self.est_why = 'parallel'
            return None
        ux = (fv[1] + fv[0] * fh[1]) / den
        uy = fh[1] + fh[0] * ux
        jx, jy = ux, uy
        bh, ah = fh[0], fh[1]
        bv, av = fv[0], fv[1]
        wh = self.fit_weighted(self.CO, n_c, hlo, hhi, fh[0], fh[1], fh[2], ux)
        wv = self.fit_weighted(self.RO, n_r, vlo, vhi, fv[0], fv[1], fv[2], uy)
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

    def estimate_h_only(self, n_c, tc):
        c = self.cfg
        hlo, hhi = int(tc * c['thick_lo']), int(tc * c['thick_hi'])
        fh = self.fit_line(self.CO, n_c, hlo, hhi, c['iters'], c['reject'])
        if fh is None:
            return None
        self.mad_h = fh[4]
        if fh[4] > c['max_fit_mad_px']:
            return None          # points are scattered, not a bar
        b, a = fh[0], fh[1]
        y_at_center = a + b * (self.W * 0.5)
        theta = math.degrees(math.atan(b))
        return y_at_center, theta, b, a

    def estimate_v_only(self, n_r, tr):
        c = self.cfg
        vlo, vhi = int(tr * c['thick_lo']), int(tr * c['thick_hi'])
        fv = self.fit_line(self.RO, n_r, vlo, vhi, c['iters'], c['reject'])
        if fv is None:
            return None
        self.mad_v = fv[4]
        if fv[4] > c['max_fit_mad_px']:
            return None
        b, a = fv[0], fv[1]
        x_at_center = a + b * (self.H * 0.5)
        theta = -math.degrees(math.atan(b))
        return x_at_center, theta, b, a

    def plausible(self, x_new):
        """Reject a reading the camera could not physically have moved
        to since the last one. Compared modulo the cell spacing, so
        crossing a junction (X running to 0 and reappearing at the other
        end of the range) is a zero-length move here and never flagged.
        Never called with None — the dead zone is handled by the caller.

        LOCAL DELTA vs the POC: the reference is only trusted while it is
        CURRENT. After more than PLAUSIBLE_GAP_FRAMES frames with no accepted
        X, the next one reseeds instead of being judged. The POC kept last_X
        across the whole dead zone, so a junction left at -14 mm made the next
        one at +24 mm a 38 mm "jump" and X was refused until ~+16 mm (robot
        0302, 2026-11-01). The gate still applies between consecutive fixes."""
        c = self.cfg
        max_step = c['max_step_mm']
        spacing = c['junction_spacing_mm']
        if (not max_step or self.last_X is None
                or self.frame_id - self.last_X_frame > PLAUSIBLE_GAP_FRAMES):
            self.last_X = x_new
            self.last_X_frame = self.frame_id
            return True
        d = x_new - self.last_X
        half = spacing * 0.5
        while d < -half:
            d += spacing
        while d > half:
            d -= spacing
        if d < 0:
            d = -d
        if d > max_step:
            return False
        self.last_X = x_new
        self.last_X_frame = self.frame_id
        return True

    def signed_x(self, jx_px, mm_per_px):
        """Junction offset in px -> the signed X in mm, or None.

        d is how far the fitted junction sits from the camera centre:
        negative once we have driven past it, positive while it is still
        ahead. Folded into (-spacing, 0] so the junction BEHIND us is
        always the reference, then:

            d in [-30, 0]   -> just left it,     X = d          (0 -> -30)
            d in (-50, -30) -> too far from either             -> None
            d in [-80, -50] -> next one is near, X = d + 80     (+30 -> 0)

        Measured against the junction actually fitted, so there is no dead
        reckoning and no accumulated drift. `None` is a VALID reading
        meaning "between junctions", not a failure.
        """
        c = self.cfg
        spacing = c['junction_spacing_mm']
        rng = c['x_range_mm']
        d = (jx_px - self.W * 0.5) * mm_per_px
        while d > 0.0:
            d -= spacing
        while d <= -spacing:
            d += spacing
        if d >= -rng:
            x = d
        elif d <= -(spacing - rng):
            x = d + spacing
        else:
            return None
        return x * c['x_direction']

    def clear_hist(self):
        hc, hr = self.hc, self.hr
        for k in range(256):
            hc[k] = 0
            hr[k] = 0

    def do_scan(self, buf, th, cols, nc, rows, nr, x0, x1):
        c = self.cfg
        min_len, long_cut = c['min_len'], c['long_cut']
        res, hc, hr = self.res, self.hc, self.hr
        self.clear_hist()
        col_scan(buf, self.W, self.H, th, min_len, long_cut, cols, nc,
                 self.CO, MAXPTS, res, hc)
        n_c, bx0, bx1 = res[0], res[1], res[2]
        n_r = int(row_scan(buf, self.W, self.H, th, min_len, long_cut,
                           x0, x1, rows, nr, self.RO, MAXPTS, hr))
        # LOCAL DELTA vs the POC: runs cut off by the frame edge are not
        # points (see drop_cut_runs). Columns end at the top/bottom rows,
        # rows at the left/right columns.
        n_c = int(drop_cut_runs(self.CO, n_c, (self.H - 1) * 16))
        n_r = int(drop_cut_runs(self.RO, n_r, (self.W - 1) * 16))
        tc = int(hist_peak(hc, min_len, long_cut - 1))
        tr = int(hist_peak(hr, min_len, long_cut - 1))
        tc_cnt = int(hist_peak_count(hc, min_len, long_cut - 1))
        tr_cnt = int(hist_peak_count(hr, min_len, long_cut - 1))
        # Anchor the measured thickness to the real tape width. hist_peak can
        # lock onto the width of a serrated edge's teeth instead of the tape;
        # thick_lo/thick_hi would then filter FOR the teeth. Only the width
        # value is corrected — tc_cnt/tr_cnt (how many runs supported the
        # peak) are deliberately left alone.
        nominal = c['nominal_thick_px']
        if tc < (nominal >> 1) or tc > (nominal << 1):
            tc = nominal
        if tr < (nominal >> 1) or tr > (nominal << 1):
            tr = nominal
        return n_c, n_r, tc, tr, bx0, bx1, tc_cnt, tr_cnt

    # ---- the POC's loop body, minus drawing and printing

    def analyse(self, buf, th, mm_per_px, prof=None):
        c = self.cfg
        W, H = self.W, self.H
        frame_id = self.frame_id
        min_pts, min_len = c['min_pts'], c['min_len']
        coverage = c['coverage_frac']
        t0 = _us() if prof is not None else 0
        self.mad_h = self.mad_v = None

        status = 'track'
        if self.band is None:
            status = 'acquire'
            (n_c, n_r, tc, tr, bx0, bx1,
             tc_cnt, tr_cnt) = self.do_scan(buf, th, self.allc, self.NAC,
                                            self.allr, self.NAR, 0, W)
            nc_used, nr_used = self.NAC, self.NAR
        else:
            x0, x1 = self.band
            (n_c, n_r, tc, tr, bx0, bx1,
             tc_cnt, tr_cnt) = self.do_scan(buf, th, self.ca, self.NC,
                                            self.ra, self.NR, x0, x1)
            nc_used, nr_used = self.NC, self.NR

            # Between two junctions bx1 is -1 by definition, so an
            # unconditional re-acquire would run a second, full-width scan
            # on EVERY frame of the mini region. Only escalate if the cheap
            # scan cannot even support the horizontal-only fallback -- or on
            # a slow cadence, to notice a vertical bar coming back.
            h_usable = (tc >= min_len and n_c >= min_pts
                        and n_c >= nc_used * coverage)
            need_full = (bx1 < 0 or n_c < min_pts or n_r < min_pts)
            if need_full and (not h_usable
                              or frame_id % c['reacquire_mini_every'] == 0):
                status = 'reacquire'
                (n_c, n_r, tc, tr, bx0, bx1,
                 tc_cnt, tr_cnt) = self.do_scan(buf, th, self.allc,
                                                self.NAC, self.allr,
                                                self.NAR, 0, W)
                nc_used, nr_used = self.NAC, self.NAR
            elif (not need_full) and frame_id % c['reacquire_every'] == 0:
                (n_c2, n_r2, tc2, tr2, bx02, bx12,
                 tc_cnt2, tr_cnt2) = self.do_scan(buf, th, self.allc,
                                                  self.NAC, self.allr,
                                                  self.NAR, 0, W)
                if bx12 >= 0:
                    n_c, n_r, tc, tr, bx0, bx1 = (n_c2, n_r2, tc2,
                                                  tr2, bx02, bx12)
                    tc_cnt, tr_cnt = tc_cnt2, tr_cnt2
                    nc_used, nr_used = self.NAC, self.NAR
                    status = 'periodic'
        if prof is not None:
            t0 = _mark(prof, 'scan', t0)

        # Full-junction validity. Two physical gates beyond the counts: a
        # vertical bar spans several columns, and it is the same tape as the
        # horizontal bar so it must be about as wide (lens glare at the frame
        # edge fails both).
        ok = (bx1 >= 0 and (bx1 - bx0) >= c['min_v_span_px']
              and tc >= min_len and tr >= min_len
              and n_c >= min_pts and n_r >= min_pts
              and tr >= tc * c['thick_match_frac'])

        # LOCAL (diagnostic, not in the POC): which gate refused the
        # junction, and below why the horizontal-only fallback failed too.
        # Read by vision/points.py; changes no decision.
        why = None
        if not ok:
            why = ('no_vbar' if bx1 < 0
                   else 'vspan' if (bx1 - bx0) < c['min_v_span_px']
                   else 'thin' if (tc < min_len or tr < min_len)
                   else 'pts' if (n_c < min_pts or n_r < min_pts)
                   else 'thick_match')

        out = None
        if ok:
            extra = c['extra']
            self.band = (max(0, bx0 - extra), min(W, bx1 + extra))
            self.est_why = None
            out = self.estimate(n_c, n_r, tc, tr)
            if out is None:
                ok = False
                why = self.est_why

        r = {'tier': None, 'status': status,
             'theta_deg': None, 'x_mm': None, 'y_mm': None,
             'theta_split': None, 'node': None,
             'line_x': None, 'line_y': None, 'suspect': False,
             'n_c': n_c, 'n_r': n_r, 'tc': tc, 'tr': tr,
             'mad_h': self.mad_h, 'mad_v': self.mad_v,
             'threshold': th, 'centre': (W * 0.5, H * 0.5),
             'why': why, 'why_h': None}
        mmpx = mm_per_px

        if ok:
            # ---- tier 1: full junction (both bars)
            jx, jy, thd, hv, bh, ah, bv, av = out
            X_mm = self.signed_x(jx, mmpx)
            Y_mm = (jy - H * 0.5) * mmpx
            # X_mm is None in the dead zone between junctions. That is a
            # valid reading, so it must not go through the plausibility
            # gate and must not count as a rejection.
            sus = X_mm is not None and not self.plausible(X_mm)
            if sus:
                self.n_rejected += 1
            r['tier'] = TIER_JUNCTION
            r['theta_deg'] = thd
            r['theta_split'] = hv
            # LOCAL: the POC keeps drawing a suspect X and only excludes it
            # from its summary; on the wire a number we know is wrong must
            # not travel.
            r['x_mm'] = None if sus else X_mm
            r['y_mm'] = Y_mm
            r['node'] = (jx, jy)
            r['line_x'] = (bh, ah)
            r['line_y'] = (bv, av)
            r['suspect'] = sus
        else:
            self.band = None
            h_ok_fb = (tc >= min_len and n_c >= min_pts
                       and n_c >= nc_used * coverage)
            v_ok_fb = (bx1 >= 0 and tr >= min_len and n_r >= min_pts
                       and n_r >= nr_used * coverage)
            if not h_ok_fb:
                r['why_h'] = 'h_cov'
            if h_ok_fb:
                # ---- tier 2/3: horizontal bar only.
                e = self.estimate_h_only(n_c, tc)
                if e is None:
                    r['why_h'] = 'h_fit'
                if e is not None:
                    # keep the cheap STEP-spaced scan alive
                    self.band = (0, W)
                    y_px, thd, b, a = e
                    Y_mm = (y_px - H * 0.5) * mmpx
                    # ---- tier 2: Y + theta only.
                    # The mini-junction tier that used to live here was
                    # REMOVED UPSTREAM in v1_2 (find_mini_gap kept but with
                    # zero call sites, marked DEAD CODE). With signed X the
                    # cell midpoint is the null zone, so a void there no
                    # longer needs to produce a number.
                    r['theta_deg'] = thd
                    r['y_mm'] = Y_mm
                    r['line_x'] = (b, a)
                    r['tier'] = TIER_Y
            elif v_ok_fb:
                e = self.estimate_v_only(n_r, tr)
                if e is not None:
                    x_px, thd, b, a = e
                    X_mm = (x_px - W * 0.5) * mmpx
                    r['tier'] = TIER_X
                    r['theta_deg'] = thd
                    r['x_mm'] = X_mm
                    r['line_y'] = (b, a)
        if prof is not None:
            t0 = _mark(prof, 'fit', t0)

        r['mad_h'], r['mad_v'] = self.mad_h, self.mad_v
        self.status = status
        self.frame_id = frame_id + 1
        return r
