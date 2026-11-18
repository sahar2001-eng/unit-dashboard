# Frame quality score: how much to trust this frame, and if not, why.
#
# One number per frame, q = 0..100, made of three parts:
#
#   img   the picture itself: light, contrast, glare, flicker, blur
#   line  the line fit: how many points, how tight, tape width, how square
#   rec   the recognition: lost, refused X, a junction in view but not found,
#         constant re-searching
#
# q follows the WEAKEST part: a frame is only as good as its weakest part.
# One exception: a bad picture on its own pulls q down only to `img_floor`
# (40). A blurred frame whose lines still fit well is a warning, not a
# failure; when the picture is really bad the fit fails too and q drops.
# `why` names the single worst signal ('few_points', 'blur', 'glare', ...), so
# a low score always comes with a reason.
#
# "Normal" is LEARNED PER ROBOT. A robot that usually gets 30 points on a line
# and suddenly gets 6 is in trouble, even if 6 would be fine on another robot.
# Each signal keeps a slow running average of its value on GOOD frames only
# (so a bad stretch is never learned as normal), and every learned value is
# kept within [learn_lo, learn_hi] x its seed in config, so a robot that is bad
# from the very first frame cannot learn that bad is normal either.
#
# Cost: the per-frame part is a few dozen float operations on numbers the
# pipeline already made (no image access). The light part reads the 256-bin
# histogram of the quarter-scale Otsu copy, and only every `light_every`
# frames. Nothing here changes what the pipeline outputs; it only scores it.
#
# prec_*: the STATISTICAL precision of this frame's fit (1 sigma), from the
# point scatter and the point count. It says how repeatable the number is. It
# does NOT include calibration error (mm/px, lens), which the camera cannot
# see on its own.
#
# Tile 2 (UNIT16 hole grid, gridcam 1.32.0): its own scorer, frame_sig(). It
# takes a dict of plain numbers that tile2.Tile2Tracker.signals() builds from
# the Pipe (no image access), with the limits of config quality.tile2 laid
# over the rest (make_quality). Ported from the bench (openmv/bench/bench/
# quality.py frame_sig); the one change: `why` is filled below `ok` (60), as
# on Tile 1, not below the bench's 85. frame() (Tile 1) scores exactly as
# before (its end now shares _total / _avg_events with frame_sig;
# tests/desktop/test_quality_t2.py pins its output against 1.31.0).

import math
from array import array

# Same pattern as pipeline.py: the decorator must be spelled
# `@micropython.viper` literally (an alias silently runs as bytecode), and a
# stub keeps the module importable on bare CPython.
try:
    import micropython
except ImportError:
    import sys as _sys
    import types as _types
    micropython = _types.SimpleNamespace(viper=lambda f: f, native=lambda f: f)
    _sys.modules['micropython'] = micropython

TIER_JUNCTION = 'junction'
TIER_Y = 'y'
TIER_X = 'x'


def ramp(v, bad, good):
    """0.0 at `bad`, 1.0 at `good`, straight line between, clamped.

    Works in both directions: good > bad for "more is better" (points,
    contrast), good < bad for "less is better" (scatter, glare)."""
    if good == bad:
        return 1.0 if v == good else 0.0
    t = (v - bad) / (good - bad)
    if t <= 0.0:
        return 0.0
    if t >= 1.0:
        return 1.0
    return t


def light_from_bins(b, th):
    """Tape / floor brightness from a 256-bin histogram split at Otsu `th`.

    `b` is the list image.get_histogram().bins() returns (fractions that sum
    to 1). Returns (tape_mean, floor_mean, clip_pct, tape_frac):
      clip_pct  % of TAPE pixels at 250 or more (burnt white = glare)
      tape_frac fraction of the picture that is tape
    Same split as light_cal.py, which was run on the AE3.
    """
    hi = lo = nhi = nlo = 0.0
    th = int(th)
    for i in range(256):
        f = b[i]
        if i > th:
            hi += f * i
            nhi += f
        else:
            lo += f * i
            nlo += f
    tape = hi / nhi if nhi > 0 else 0.0
    floor = lo / nlo if nlo > 0 else 0.0
    clip = 0.0
    if nhi > 0:
        s = 0.0
        for i in range(250, 256):
            s += b[i]
        clip = 100.0 * s / nhi
    return tape, floor, clip, nhi


@micropython.viper
def _hist_peak_near(hist: ptr32, lo: int, hi: int, half: int, out: ptr32):
    best = lo
    bc = -1
    tot = 0
    L = lo
    while L <= hi:
        c = int(hist[L])
        tot += c
        if c > bc:
            bc = c
            best = L
        L += 1
    a = best - half
    if a < lo:
        a = lo
    z = best + half
    if z > hi:
        z = hi
    near = 0
    L = a
    while L <= z:
        near += int(hist[L])
        L += 1
    out[0] = best
    out[1] = near
    out[2] = tot


_HPN = array('i', [0, 0, 0])


def hist_peak_near(hist, lo, hi, half):
    """(peak width, runs within +-half of it, all runs in [lo, hi]).
    viper: a Python loop over the ~40 bins was 60% of the score's cost."""
    _hist_peak_near(hist, lo, hi, half, _HPN)
    return _HPN[0], _HPN[1], _HPN[2]


def bar_position(ro, n, tol):
    """Median x of the row-scan points, and how many lie within +-tol px.

    `ro` is GridTracker.RO: [row, centre x * 16, width] per point. Median,
    because a row can also hit a piece of the bar at the frame edge (seen on
    the robot video). Only called on the rare frames that have many row
    points but no junction, so the sort is affordable."""
    xs = sorted(ro[i * 3 + 1] for i in range(n))
    m = xs[n // 2]
    t = tol * 16
    near = 0
    for x in xs:
        if m - t <= x <= m + t:
            near += 1
    return m / 16.0, near


@micropython.viper
def edge_grad(buf: ptr8, w: int, h: int, arr: ptr32, n: int, cols: int,
              step: int, win: int, out: ptr32):
    # How steep the tape edges are, at every `step`-th scan point.
    # arr is GridTracker.CO (cols=1: [x, centre*16, width], edges run along
    # y) or .RO (cols=0: [y, centre*16, width], edges along x). For each of
    # the point's two edges: the largest |p(i+1) - p(i-1)| within +-win px,
    # divided by 2 x (tape - floor) measured just inside and just outside.
    # A perfect step edge gives ~0.5; blur over k px gives ~1/k.
    # out[0] = sum of the ratios x 1000, out[1] = edges measured.
    lim = h
    if cols == 0:
        lim = w
    tot = 0
    m = 0
    i = 0
    while i < n:
        a = int(arr[i * 3])
        c16 = int(arr[i * 3 + 1])
        L = int(arr[i * 3 + 2])
        ci = (c16 + 8) >> 4
        k = 0
        while k < 2:
            if k == 0:
                e = (c16 - L * 8 + 8) >> 4
                po = e - win - 1
            else:
                e = (c16 + L * 8 + 8) >> 4
                po = e + win + 1
            lo = e - win
            hi = e + win
            if lo >= 1 and hi <= lim - 2 and po >= 0 and po < lim \
                    and ci >= 0 and ci < lim:
                if cols:
                    C = int(buf[ci * w + a]) - int(buf[po * w + a])
                else:
                    C = int(buf[a * w + ci]) - int(buf[a * w + po])
                if C >= 20:
                    g = 0
                    j = lo
                    while j <= hi:
                        if cols:
                            d = int(buf[(j + 1) * w + a]) - \
                                int(buf[(j - 1) * w + a])
                        else:
                            d = int(buf[a * w + j + 1]) - int(buf[a * w + j - 1])
                        if d < 0:
                            d = -d
                        if d > g:
                            g = d
                        j += 1
                    tot += (g * 1000) // (2 * C)
                    m += 1
            k += 1
        i += step
    out[0] = tot
    out[1] = m


# Signal names -> what they mean, for logs and the web page.
WHY_TEXT = {
    'few_points': 'few points on the line',
    'scatter': 'points do not sit on a line',
    'blur': 'tape edges are smeared (blur)',
    'width': 'tape looks the wrong width',
    'not_square': 'the two bars are not at 90 deg',
    'low_contrast': 'tape and floor too close in brightness',
    'dark': 'picture too dark',
    'glare': 'tape burnt white (glare)',
    'flicker': 'brightness jumps between frames (flicker)',
    'lost': 'no line found',
    'missed_junction': 'a crossing bar is in view but no junction was found',
    'x_refused': 'X refused as an impossible jump',
    'often_lost': 'line lost often in the last moments',
    'researching': 'tracking keeps falling back to a full search',
    'damaged_tape': 'part of the tape is the wrong width (damaged or painted)',
    'code_unreadable': 'code junction in view but its code cannot be read',
}


# Reason codes on the wire (UART frame 0xA9). Append only: the STM32 keeps
# its own copy of this table (stm/Core/Inc/cam_proto.h, CAM_Q_*).
# 15 damaged_tape: the bench's Tile 1 scorer sends it; the app's frame()
# does not (yet), the code is kept so the tables stay the same.
# 16 code_unreadable: Tile 2. The STM32 names stop at 14 (prints '?' for
# 15 and 16) until its table gets the two lines.
WHY_CODES = ('', 'lost', 'missed_junction', 'x_refused', 'often_lost',
             'researching', 'few_points', 'scatter', 'width', 'not_square',
             'blur', 'dark', 'low_contrast', 'glare', 'flicker',
             'damaged_tape', 'code_unreadable')

# LOCAL (gridcam 1.27.2): the two pixel loops above can run as C. t1c
# (openmv/bench/native/t1c) has them as hist_peak_near / edge_grad, same
# arguments; the bench proved them equal to these viper bodies (test t02).
# tests/desktop/test_quality_c.py fails if either changes here without
# t1c.c. Callers look the names up at call time, so this just rebinds them.
C_KERNELS = {'_hist_peak_near': 'hist_peak_near', 'edge_grad': 'edge_grad'}
_VIPER = {k: globals()[k] for k in C_KERNELS}
KERNELS = 'viper'                            # what runs now: 'c' or 'viper'
KERNELS_ERR = ''                             # why C was refused, '' if not


def use_kernels(c, mod=None):
    """Run the score's pixel loops as C (c=True) or viper; returns which.
    A missing t1c or a wrong API keeps viper, reason in KERNELS_ERR."""
    global KERNELS, KERNELS_ERR
    from . import pipeline                  # one place that imports t1c
    g = globals()
    KERNELS_ERR = ''
    if c:
        try:
            m = mod if mod is not None else pipeline._import_t1c()
            api = getattr(m, 'API', None)
            if api != pipeline.T1C_API:
                raise ValueError('t1c API %s, need %d'
                                 % (api, pipeline.T1C_API))
            fns = dict((k, getattr(m, v)) for k, v in C_KERNELS.items())
        except Exception as e:
            KERNELS_ERR = str(e) or e.__class__.__name__
            c = False
    for k in C_KERNELS:
        g[k] = fns[k] if c else _VIPER[k]
    KERNELS = 'c' if c else 'viper'
    return KERNELS


# Levels, used on the stream, the web page and the explainer page.
LEVELS = ((85, 'excellent'), (60, 'good'), (40, 'warning'), (0, 'bad'))


def level(q):
    """'excellent' 85-100, 'good' 60-84, 'warning' 40-59, 'bad' 0-39."""
    if q is None:
        return ''
    for lo, name in LEVELS:
        if q >= lo:
            return name
    return 'bad'


def why_code(why):
    try:
        return WHY_CODES.index(why)
    except ValueError:
        return 0


class Quality:
    """Scores frames. One per tile's tracker. Reads, never writes, the
    tracker and the frame. Tile 1 calls frame(), Tile 2 frame_sig()
    (cfg['tile'] == 2, see make_quality); skip() works on both."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.tile = 2 if cfg.get('tile') == 2 else 1
        seeds = cfg['normal']
        self.seed = {}
        self.base = {}
        for k in seeds:
            self.seed[k] = float(seeds[k])
            self.base[k] = float(seeds[k])
        self.light_every = int(cfg['light_every'])
        self._eo = array('i', [0, 0])
        self._ar = 1.0 / max(1, int(cfg['recent_frames']))
        self._aq = 1.0 / max(1, int(cfg['avg_frames']))
        self.reset()

    def reset(self):
        # this frame
        self.q = None
        self.q_img = 1.0
        self.q_line = None
        self.q_rec = None
        self.why = ''
        self.prec_x_mm = None
        self.prec_y_mm = None
        self.prec_th_deg = None
        # light (held between measurements)
        self.light_ok = False
        self.tape = self.floor = self.contrast = self.clip = 0.0
        self.tape_frac = 0.0
        self._img_why = ''
        self._th_prev = None
        self.th_jitter = 0.0          # running average of |otsu change|
        # recent history
        self.lost_rate = 0.0
        self.research_rate = 0.0
        self.refused_rate = 0.0
        self.q_avg = None
        # counters
        self.frames = 0
        self.low_frames = 0
        self.clean_frames = 0
        self.low_events = 0
        self.low_why = {}             # reason -> low frames it caused
        self._low = False
        self._event = None
        self._vals = {}               # last value of every signal (for logs)
        self._img_now = 1.0
        self._img_why_now = ''
        self.last_why = ''            # last non-empty reason
        # Tile 2 only (frame_sig)
        self.unread_run = 0           # frames in a row with an unread code
        self.zone = ''                # 'junction' or 'road'
        # per zone: [frames, sum of q, frames under `low`]
        self.zone_stats = {'junction': [0, 0, 0], 'road': [0, 0, 0]}

    # ---- learning ------------------------------------------------------

    def _learn(self, key, v):
        c = self.cfg
        s = self.seed[key]
        b = self.base[key]
        b += c['learn_alpha'] * (v - b)
        lo, hi = s * c['learn_lo'], s * c['learn_hi']
        if b < lo:
            b = lo
        elif b > hi:
            b = hi
        self.base[key] = b

    # ---- inputs --------------------------------------------------------

    def otsu(self, th):
        """Every new Otsu threshold. A jumping threshold on a still or slowly
        moving scene means the light itself is changing (LED flicker)."""
        p = self._th_prev
        self._th_prev = th
        if p is None:
            return
        d = th - p
        if d < 0:
            d = -d
        self.th_jitter += 0.1 * (d - self.th_jitter)

    def light(self, bins, th):
        """Light measurement from the Otsu copy's histogram (slow cadence)."""
        tape, floor, clip, frac = light_from_bins(bins, th)
        self.tape, self.floor, self.clip, self.tape_frac = tape, floor, clip, frac
        self.contrast = tape - floor
        self.light_ok = True
        self._score_light()

    def _score_light(self):
        c = self.cfg
        b = self.base
        worst, why = 1.0, ''
        C = self.contrast
        s = ramp(C, c['contrast_bad'], c['contrast_good'])
        s2 = ramp(C / b['contrast'], c['rel_bad'], c['rel_good'])
        if s2 < s:
            s = s2
        if s < worst:
            worst, why = s, 'low_contrast'
        s = ramp(self.tape, c['dark_bad'], c['dark_good'])
        if s < worst:
            worst, why = s, 'dark'
        s = ramp(self.clip, c['clip_bad'], c['clip_good'])
        if s < worst:
            worst, why = s, 'glare'
        self.q_img, self._img_why = worst, why

    def _flicker(self):
        c = self.cfg
        bj = self.base['th_jitter']
        return ramp(self.th_jitter, max(c['jitter_bad'], 4.0 * bj),
                    max(c['jitter_good'], 2.0 * bj))

    def skip(self):
        """A frame Algorithm 1 did not run on (turning): no score."""
        self.q = None
        self.q_line = self.q_rec = None
        self.why = ''
        self.prec_x_mm = self.prec_y_mm = self.prec_th_deg = None

    def _edges(self, buf, trk, arr, n, cols):
        """Mean edge steepness of up to `edge_samples` scan points, or None."""
        ns = self.cfg['edge_samples']
        if n < 2:
            return None
        step = n // ns if n > ns else 1
        eo = self._eo
        edge_grad(buf, trk.W, trk.H, arr, n, cols, step,
                  self.cfg['edge_win'], eo)
        if eo[1] < 2:
            return None
        return eo[0] / (1000.0 * eo[1])

    def frame(self, res, trk, mm_per_px, buf=None):
        """Score one analysed frame. `res` is GridTracker.analyse()'s dict,
        `trk` the tracker (its scan sizes, run-width histogram, row points),
        `buf` the frame (only a few dozen pixels next to the scan points are
        read, for the edge steepness; None skips that check)."""
        c = self.cfg
        b = self.base
        v = self._vals
        tier = res['tier']
        status = res['status']
        n_c, n_r = res['n_c'], res['n_r']
        mh, mv = res['mad_h'], res['mad_v']
        self.frames += 1
        ar = self._ar
        lost = tier is None
        refused = res['suspect']
        full = status != 'track'
        nc_used = trk.NAC if full else trk.NC
        nr_used = trk.NAR if full else trk.NR
        cov_c = n_c / nc_used if nc_used else 0.0
        cov_r = n_r / nr_used if nr_used else 0.0
        mad = sharp = width = split = edge_c = edge_r = None

        # ---- picture: light (held from the last light measurement),
        # flicker, and blur. Blur is seen two ways:
        #  - edge steepness at the scan points, across the horizontal bar
        #    (columns) and, at a junction, across the vertical bar (rows).
        #    Driving smears the edges that run across the direction of motion.
        #  - the run widths of the column scan: on a sharp picture most runs
        #    are exactly the tape width; blur spreads them out.
        img, iwhy = self.q_img, self._img_why
        s = self._flicker()
        if s < img:
            img, iwhy = s, 'flicker'
        ml, lc = trk.cfg['min_len'], trk.cfg['long_cut']
        pk, near, tot = hist_peak_near(trk.hc, ml, lc - 1, c['sharp_half'])
        if not lost and tot >= c['sharp_min_runs']:
            sharp = near / tot
            width = pk
            s = ramp(sharp / b['sharp'], c['rel_bad'], c['rel_good'])
            if s < img:
                img, iwhy = s, 'blur'
        if buf is not None and not lost:
            rb, rg = c['edge_bad'], c['edge_good']
            edge_c = self._edges(buf, trk, trk.CO, n_c, 1)
            if edge_c is not None:
                s = ramp(edge_c / b['edge_c'], rb, rg)
                if s < img:
                    img, iwhy = s, 'blur'
            if tier == TIER_JUNCTION:
                edge_r = self._edges(buf, trk, trk.RO, n_r, 0)
                if edge_r is not None:
                    s = ramp(edge_r / b['edge_r'], rb, rg)
                    if s < img:
                        img, iwhy = s, 'blur'

        # ---- recognition
        self.lost_rate += ar * ((1.0 if lost else 0.0) - self.lost_rate)
        self.refused_rate += ar * ((1.0 if refused else 0.0) - self.refused_rate)
        self.research_rate += ar * ((1.0 if full else 0.0) - self.research_rate)
        rec, rwhy = 1.0, ''
        if lost:
            rec, rwhy = 0.0, 'lost'
        elif refused:
            rec, rwhy = c['refused_score'], 'x_refused'
        s = ramp(self.lost_rate, c['lost_rate_bad'], c['lost_rate_good'])
        if s < rec:
            rec, rwhy = s, 'often_lost'
        s = ramp(self.research_rate, c['research_bad'], c['research_good'])
        if s < rec:
            rec, rwhy = s, 'researching'
        # A crossing bar in view but no junction found: most row lines put
        # their point in one narrow strip away from the frame edge, yet the
        # column scan did not mark it as the crossing bar. This is how
        # long_cut 60 failed on the real floor, and nothing else here sees
        # it: such a frame looks like a clean Y.
        if (tier != TIER_JUNCTION and cov_r >= c['bar_min_cov']
                and n_r >= 3):
            mx, nb = bar_position(trk.RO, n_r, c['bar_tol'])
            W = trk.W
            if (nb >= c['bar_min_cov'] * nr_used
                    and W * c['bar_edge'] <= mx <= W * (1.0 - c['bar_edge'])):
                if c['missed_score'] < rec:
                    rec, rwhy = c['missed_score'], 'missed_junction'

        # ---- line fit
        line, lwhy = (0.0, 'lost') if lost else (1.0, '')
        self.prec_x_mm = self.prec_y_mm = self.prec_th_deg = None
        if not lost:
            pb, pg = c['pts_bad'], c['pts_good']
            if tier == TIER_JUNCTION:
                s = ramp(cov_c / b['cov_c_j'], pb, pg)
                s2 = ramp(cov_r / b['cov_r_j'], pb, pg)
                if s2 < s:
                    s = s2
            elif tier == TIER_Y:
                s = ramp(cov_c / b['cov_c_y'], pb, pg)
            else:                                   # X only (rare)
                s = ramp(cov_r / b['cov_r_j'], pb, pg)
            if s < line:
                line, lwhy = s, 'few_points'

            # Fit spread. The horizontal bar (tracked, always through the
            # middle of the picture) is held to its learned normal. The
            # vertical bar gets softer limits: with the junction near the
            # frame edge the lens bends it, and ~2 px there is normal
            # (robot video). Only counted when that bar is actually used.
            mad = mh if mh is not None else mv
            if mad is not None:
                bm = b['mad']
                s = ramp(mad, max(c['mad_bad'], 4.0 * bm),
                         max(c['mad_good'], 2.0 * bm))
                if s < line:
                    line, lwhy = s, 'scatter'
            if tier == TIER_JUNCTION and mv is not None:
                s = ramp(mv, c['mad_v_bad'], c['mad_v_good'])
                if s < line:
                    line, lwhy = s, 'scatter'

            if width is not None:
                dev = width / b['width'] - 1.0
                if dev < 0:
                    dev = -dev
                s = ramp(dev, c['width_bad'], c['width_good'])
                if s < line:
                    line, lwhy = s, 'width'

            if tier == TIER_JUNCTION and res['theta_split'] is not None:
                split = res['theta_split']
                if split < 0:
                    split = -split
                s = ramp(split - b['split'], c['split_bad'], c['split_good'])
                if s < line:
                    line, lwhy = s, 'not_square'

            # statistical precision of this frame's fit (1 sigma)
            if n_c > 0 and mh is not None:
                sig = 1.4826 * mh
                if sig < 0.1:
                    sig = 0.1            # sub-pixel step of the scan
                rn = math.sqrt(n_c)
                self.prec_y_mm = sig / rn * mm_per_px
                self.prec_th_deg = math.degrees(sig * 3.4641 / (rn * trk.W))
            if tier == TIER_JUNCTION and n_r > 0 and mv is not None:
                sv = 1.4826 * mv
                if sv < 0.1:
                    sv = 0.1
                self.prec_x_mm = sv / math.sqrt(n_r) * mm_per_px

        why = self._total(line, lwhy, rec, rwhy, img, iwhy, None)

        v['cov_c'], v['cov_r'], v['mad'], v['mad_v'] = cov_c, cov_r, mh, mv
        v['sharp'], v['width'], v['split'] = sharp, width, split
        v['edge_c'], v['edge_r'] = edge_c, edge_r

        # ---- learn "normal" from clean frames only
        clean = (not lost and not refused and rwhy != 'missed_junction'
                 and img >= c['learn_min_q'] and line >= c['learn_min_q']
                 and mh is not None and mh <= c['mad_good'])
        if clean:
            self.clean_frames += 1
            if tier == TIER_JUNCTION:
                self._learn('cov_c_j', cov_c)
                self._learn('cov_r_j', cov_r)
                if split is not None:
                    self._learn('split', split)
            elif tier == TIER_Y:
                self._learn('cov_c_y', cov_c)
            self._learn('mad', mh)
            if sharp is not None:
                self._learn('sharp', sharp)
                self._learn('width', width)
            if edge_c is not None:
                self._learn('edge_c', edge_c)
            if edge_r is not None:
                self._learn('edge_r', edge_r)
            self._learn('th_jitter', self.th_jitter)
            if self.light_ok and self.clip <= c['clip_good']:
                self._learn('contrast', self.contrast)
        return self._avg_events(why)

    def frame_sig(self, s, mm_per_px):
        """Score one Tile 2 frame (not turning). `s` is
        tile2.Tile2Tracker.signals()'s dict (no image access here):
          lost, hold, research, jz   bools: no line / X held as a jump /
                                     a tracked line was searched again /
                                     a code junction is in view (the zone)
          cov_r, cov_x   points kept / strips laid, road and cross bar (None
                         when that bar is not there)
          rms_r, rms_x   fit rms, px (None when that bar is not there)
          n_r, n_x       points kept on each bar
          perp           |90 - angle between the bars|, deg (None unless both)
          code_full      a full code block is in view and was read clearly
          named          the junction X is measured from has a name
        mm_per_px is Tile 2's (config tile2.mm_per_px), not the camera's.
        Same three parts as frame(), from what Tile 2 can see:
          picture  light and flicker (light() / otsu(), as on Tile 1)
          rec      lost, x_refused (a held X in the zone), often_lost,
                   researching, code_unreadable (unread_frames frames in a
                   row with a full code block in view but no name)
          line     few_points (road always, cross in the zone), scatter,
                   not_square (in the zone)
        Tile 2 has no blur, tape width, damaged tape or missed junction
        signal. Two zones: in the junction zone (a code junction within
        27 mm of the centre) X must be measured, so the cross bar counts;
        on the road only the road bar counts."""
        c = self.cfg
        b = self.base
        v = self._vals
        self.frames += 1
        ar = self._ar
        lost = s['lost']
        jz = s['jz']
        zone = 'junction' if jz else 'road'
        refused = s['hold'] and jz
        rms_r, rms_x = s['rms_r'], s['rms_x']

        # ---- picture: light (held from the last light measurement), flicker
        img, iwhy = self.q_img, self._img_why
        x = self._flicker()
        if x < img:
            img, iwhy = x, 'flicker'

        # ---- recognition
        self.lost_rate += ar * ((1.0 if lost else 0.0) - self.lost_rate)
        self.refused_rate += ar * ((1.0 if refused else 0.0) - self.refused_rate)
        self.research_rate += ar * ((1.0 if s['research'] else 0.0)
                                    - self.research_rate)
        rec, rwhy = 1.0, ''
        if lost:
            rec, rwhy = 0.0, 'lost'
        elif refused:
            rec, rwhy = c['refused_score'], 'x_refused'
        x = ramp(self.lost_rate, c['lost_rate_bad'], c['lost_rate_good'])
        if x < rec:
            rec, rwhy = x, 'often_lost'
        x = ramp(self.research_rate, c['research_bad'], c['research_good'])
        if x < rec:
            rec, rwhy = x, 'researching'
        # A full code block in view but no name for it: X is measured, but
        # from a junction nobody knows. A warning (unread_score), after
        # unread_frames frames in a row, so a single bad read does not count.
        if jz and s['code_full'] and not s['named']:
            self.unread_run += 1
        else:
            self.unread_run = 0
        if self.unread_run >= c['unread_frames'] and c['unread_score'] < rec:
            rec, rwhy = c['unread_score'], 'code_unreadable'

        # ---- line fit
        line, lwhy = (0.0, 'lost') if lost else (1.0, '')
        self.prec_x_mm = self.prec_y_mm = self.prec_th_deg = None
        split = None
        if not lost:
            pb, pg = c['pts_bad'], c['pts_good']
            sc = 1.0
            if s['cov_r'] is not None:
                sc = ramp(s['cov_r'] / b['cov_r'], pb, pg)
            if jz and s['cov_x'] is not None:
                x = ramp(s['cov_x'] / b['cov_x'], pb, pg)
                if x < sc:
                    sc = x
            if sc < line:
                line, lwhy = sc, 'few_points'
            # fit spread: the road bar against its learned normal, the cross
            # bar (X comes from it) with fixed limits, in the zone only
            m = rms_r if rms_r is not None else rms_x
            if m is not None:
                bm = b['mad']
                x = ramp(m, max(c['mad_bad'], 4.0 * bm),
                         max(c['mad_good'], 2.0 * bm))
                if x < line:
                    line, lwhy = x, 'scatter'
            if jz and rms_x is not None:
                x = ramp(rms_x, c['mad_v_bad'], c['mad_v_good'])
                if x < line:
                    line, lwhy = x, 'scatter'
            if jz and s['perp'] is not None:
                split = s['perp']
                x = ramp(split - b['split'], c['split_bad'], c['split_good'])
                if x < line:
                    line, lwhy = x, 'not_square'
            # statistical precision of this frame's fit (1 sigma)
            if rms_r is not None and s['n_r'] > 0:
                self.prec_y_mm = (max(rms_r, 0.1) / math.sqrt(s['n_r'])
                                  * mm_per_px)
            if jz and rms_x is not None and s['n_x'] > 0:
                self.prec_x_mm = (max(rms_x, 0.1) / math.sqrt(s['n_x'])
                                  * mm_per_px)

        why = self._total(line, lwhy, rec, rwhy, img, iwhy, zone)
        v['cov_r'], v['cov_x'] = s['cov_r'], s['cov_x']
        v['rms_r'], v['rms_x'], v['split'] = rms_r, rms_x, split

        # ---- learn "normal" from clean frames only, judged on absolute
        # terms; the learn_lo..learn_hi clamp keeps bad from becoming normal
        clean = (not lost and not refused and rwhy != 'code_unreadable'
                 and self.q_img >= c['learn_min_q']
                 and rms_r is not None and rms_r <= c['mad_good'])
        if clean:
            self.clean_frames += 1
            if s['cov_r'] is not None:
                self._learn('cov_r', s['cov_r'])
            if jz and s['cov_x'] is not None:
                self._learn('cov_x', s['cov_x'])
            if split is not None:
                self._learn('split', split)
            self._learn('mad', rms_r)
            self._learn('th_jitter', self.th_jitter)
            if self.light_ok and self.clip <= c['clip_good']:
                self._learn('contrast', self.contrast)
        return self._avg_events(why)

    def _total(self, line, lwhy, rec, rwhy, img, iwhy, zone):
        """q and why from the three parts. Fit and recognition: the weakest
        decides, down to 0. The picture alone can pull q down only to
        img_floor: a blurred or dim frame whose lines still fit well is a
        warning, not a failure. (When the picture is really bad, the fit
        fails too and q drops.) `why` is filled below `ok`. zone: Tile 2's
        'junction' / 'road' for the per-zone counters, None on Tile 1.
        Returns the reason of the weakest part even when q is too high to
        show it (the low counters use it)."""
        c = self.cfg
        q, why = line, lwhy
        if rec < q:
            q, why = rec, rwhy
        f = c['img_floor']
        si = f + (1.0 - f) * img
        if si < q:
            q, why = si, iwhy
        self.q = int(100.0 * q + 0.5)
        self.why = why if self.q < c['ok'] else ''
        if self.why:
            self.last_why = self.why
        self.q_line, self.q_rec = line, rec
        self._img_now = img
        self._img_why_now = iwhy
        if zone is not None:
            self.zone = zone
            zs = self.zone_stats[zone]
            zs[0] += 1
            zs[1] += self.q
            if self.q < c['low']:
                zs[2] += 1
        return why

    def _avg_events(self, why):
        """Running average of q and the low / recovered events (with
        hysteresis). `why` is _total()'s reason. Returns q."""
        c = self.cfg
        qa = self.q_avg
        qa = float(self.q) if qa is None else qa + self._aq * (self.q - qa)
        self.q_avg = qa
        if self.q < c['low']:
            self.low_frames += 1
            if why:
                self.low_why[why] = self.low_why.get(why, 0) + 1
        if not self._low and qa < c['low']:
            self._low = True
            self.low_events += 1
            self._event = ('warn', 'QUALITY LOW %d: %s'
                           % (int(qa), self.detail(self.last_why)))
        elif self._low and qa >= c['ok']:
            self._low = False
            self._event = ('info', 'quality ok again (%d)' % int(qa))
        return self.q

    # ---- outputs -------------------------------------------------------

    def take_event(self):
        """('warn'|'info', text) once when the average goes low / recovers."""
        e, self._event = self._event, None
        return e

    def detail(self, why=None):
        """A reason in words, with the value against normal."""
        if why is None:
            why = self.why
        if not why:
            return 'ok'
        v, b = self._vals, self.base
        txt = WHY_TEXT.get(why, why)

        def pct(x):
            return '-' if x is None else '%d%%' % int(100 * x + 0.5)
        if self.tile == 2 and why == 'few_points':
            return '%s (road %s, cross %s of strips; normal %s / %s)' % (
                txt, pct(v.get('cov_r')), pct(v.get('cov_x')),
                pct(b['cov_r']), pct(b['cov_x']))
        if self.tile == 2 and why == 'scatter':
            return '%s (rms %.2f px road, %.2f px cross; normal %.2f)' % (
                txt, v.get('rms_r') or 0, v.get('rms_x') or 0, b['mad'])
        if why == 'few_points':
            return '%s (cols %s, rows %s of scan lines; normal %s / %s)' % (
                txt, pct(v.get('cov_c')), pct(v.get('cov_r')),
                pct(b['cov_c_j']), pct(b['cov_r_j']))
        if why == 'scatter':
            return '%s (spread %.2f px across, %.2f px along; normal %.2f)' % (
                txt, v.get('mad') or 0, v.get('mad_v') or 0, b['mad'])
        if why == 'blur':
            def ed(x, n):
                return '-' if x is None else '%.2f (normal %.2f)' % (x, n)
            return '%s (edge steepness across %s, along %s; %s of runs at ' \
                   'tape width, normal %s)' % (
                       txt, ed(v.get('edge_c'), b['edge_c']),
                       ed(v.get('edge_r'), b['edge_r']),
                       pct(v.get('sharp')), pct(b['sharp']))
        if why == 'width':
            return '%s (%s px, normal %.0f)' % (txt, v.get('width'), b['width'])
        if why == 'not_square':
            return '%s (%.1f deg off)' % (txt, v.get('split') or 0)
        if why in ('low_contrast', 'dark', 'glare'):
            return '%s (tape %.0f, floor %.0f, glare %.1f%%)' % (
                txt, self.tape, self.floor, self.clip)
        if why == 'flicker':
            return '%s (otsu moves %.1f per update, normal %.1f)' % (
                txt, self.th_jitter, b['th_jitter'])
        if why == 'often_lost':
            return '%s (%s of recent frames)' % (txt, pct(self.lost_rate))
        if why == 'researching':
            return '%s (%s of recent frames)' % (txt, pct(self.research_rate))
        return txt

    def stats(self):
        """For /telemetry."""
        def r(x, n=2):
            return None if x is None else round(x, n)
        f = self.frames or 1
        d = {
            'q': self.q, 'q_avg': r(self.q_avg, 1), 'why': self.why,
            'why_text': self.detail(),
            'img': r(self._img_now if self.frames else self.q_img),
            'line': r(self.q_line), 'rec': r(self.q_rec),
            'prec': {'x_mm': r(self.prec_x_mm, 3), 'y_mm': r(self.prec_y_mm, 3),
                     'theta_deg': r(self.prec_th_deg, 3)},
            'light': {'tape': r(self.tape, 1), 'floor': r(self.floor, 1),
                      'contrast': r(self.contrast, 1), 'glare_pct': r(self.clip, 1),
                      'otsu_jitter': r(self.th_jitter, 2)} if self.light_ok else None,
            'normal': dict((k, r(self.base[k], 3)) for k in self.base),
            'learning': self.clean_frames < self.cfg['warm_frames'],
            'frames': self.frames,
            'low_pct': round(100.0 * self.low_frames / f, 1),
            'low_events': self.low_events,
            'low_why': self.low_why,
            'lost_pct': round(100.0 * self.lost_rate, 1),
        }
        if self.tile == 2:
            # Tile 2: this frame's zone and the per-zone counters
            d['tile'] = 2
            d['zone'] = self.zone
            zones = {}
            for z in self.zone_stats:
                st = self.zone_stats[z]
                n = st[0]
                zones[z] = {'frames': n,
                            'q_mean': round(st[1] / n, 1) if n else None,
                            'low_pct': round(100.0 * st[2] / n, 1) if n else None}
            d['zones'] = zones
        return d


def make_quality(tile, q):
    """The scorer for `tile`, or None when quality.enabled is off.

    q is the 'quality' config section (config.section('quality')). Tile 1
    gets it as it is (the same Quality(q) as before 1.32.0). Tile 2 gets a
    copy with quality.tile2 laid over it, one level deep, so its 'normal'
    REPLACES Tile 1's (other keys there are Tile 1 signals Tile 2 does not
    have), and tile = 2 for frame_sig's texts and stats. The config section
    itself is never changed."""
    if not q.get('enabled', True):
        return None
    if tile != 2:
        return Quality(q)
    cfg = dict(q)
    cfg.update(q.get('tile2') or {})
    cfg['tile'] = 2
    return Quality(cfg)
