# Tile 2 (UNIT16 hole grid) for the vision loop -- vision.tile = 2.
# gridcam 1.29.0 drove on it; 1.32.0 adds turns (turn_step) and the live
# switch: the vision loop loads this at setup on BOTH tiles (load() --
# import first, before the heap fragments) and keeps one tracker per tile.
#
# The Tile 2 code is the bench's, unchanged: u16pipe.py, u16c.mpy (C),
# turn_sm.py (for fold_x) and unit16_codes.csv. They are NOT in app/: the
# bundle and deploy.sh put them in vision/t2lib/ straight from openmv/bench,
# so a change there is a change here (one copy). Their own imports are
# top-level (import u16c, from turn_sm import fold_x), so t2lib/ goes on
# sys.path; gridcam's own turn_sm is gridcam.vision.turn.turn_sm, a different
# name, so the two never meet.
#
# u16pipe.step() already gives X, Y and theta in the Tile 1 meaning (x_mm
# signed against the nearer code junction and None mid-cell, y_mm, tdeg), so
# the main board gets the same 0xAA frames on both tiles. The tier rule is
# the bench's (tile2.Tile2.drive).
#
# Quality score (1.32.0, the bench's Tile2.prepare / signals): the vision
# loop calls, on a Tile 2 frame that is not turning and only when a scorer
# exists:
#   prepare_quality(img, frame_i, qs, otsu)  light and flicker signals
#   analyse(...)                             the Pipe step, as before
#   qs.frame_sig(signals(), mm)              the score (mm = self.mm)
# With quality off nothing of this runs: no Otsu copy at all on Tile 2 (the
# Pipe does not use a threshold).

import math
import sys

LIB = 't2lib'
CODES = 'unit16_codes.csv'
U16C_API = 2               # the u16c API the bench's u16pipe needs (bench
                           # main.py NATIVE). API 2 (bench 2.1.0) has the
                           # clean bar check; an older u16c.mpy would ignore
                           # it and give other lines without any error.

# quality signals (the bench's tile2.py T2_*)
OTSU_SCALE = 0.25          # the quarter-size copy for the Otsu threshold
OTSU_EVERY = 3             # light signals every 3rd frame: at 15 one
                           # brightness step kept 'flicker' up for ~150 frames
ZONE_MM = 27.0             # a code junction within this of the centre: the
                           # junction zone (X must be measured there)


def _lib_dir():
    here = __file__.rsplit('/', 1)[0] if '/' in __file__ else '.'
    return here + '/' + LIB


def load_codes(path):
    """CSV bits,node,heading,deg,hex -> (code words, names like '13E').
    The bench's tile2.load_codes, minus its folder search."""
    from array import array
    tab = array('i')
    names = []
    with open(path) as f:
        for ln in f.readlines()[1:]:
            q = ln.strip().split(',')
            if len(q) >= 5:
                tab.append(int(q[4], 16))
                names.append(q[1] + q[2])
    return tab, names


def _load():
    """(u16pipe module, code words, names). Raises if anything is missing,
    or if u16c.mpy is not the API u16pipe needs (U16C_API). Called once, at
    setup (vision_service): u16c and the Pipe's ~26 KB are asked for while
    the heap is still in one piece."""
    d = _lib_dir()
    if d not in sys.path:
        sys.path.insert(0, d)   # first: a stray turn_sm.py elsewhere loses
    import u16c
    api = getattr(u16c, 'API', None)
    if api != U16C_API:
        raise RuntimeError('u16c.mpy has API %s, Tile 2 needs %d: deploy '
                           'the u16c.mpy of this release' % (api, U16C_API))
    import u16pipe
    tab, names = load_codes(d + '/' + CODES)
    return u16pipe, tab, names


# the tile2.* keys the Pipe is built with, in its argument order
PIPE_KEYS = ('mm_per_px', 'x_direction', 'x_range_mm', 'spacing_mm',
             'lens_k1')


def pipe_args(cfg):
    """The tile2.* settings the Pipe is built with. A change to any of
    them needs a new Pipe; the light keys do not."""
    return tuple(cfg[k] for k in PIPE_KEYS)


def grad4_angle(G):
    """(angle mod 90 in [-45, 45), edge energy, pixel count) from the sums
    of u16c.grad4: G = [sre >> k, sim >> k, n, k]. The bench's
    turn_sm.grad4_angle(G), and the same numbers the app's TurnSM gets from
    its own grad4 (same kernel and settings)."""
    sre = G[0] << G[3]
    sim = G[1] << G[3]
    if sre == 0 and sim == 0:
        return 0.0, 0.0, 0
    th = math.degrees(math.atan2(sim, sre)) * 0.25
    th = ((th + 45.0) % 90.0) - 45.0
    return th, math.sqrt(float(sre) * sre + float(sim) * sim), G[2]


def empty_result(w, h, th, status):
    """A frame that reads as nothing, in the analyse() form."""
    return {'tier': None, 'suspect': False, 'x_mm': None, 'y_mm': None,
            'theta_deg': None, 'theta_split': None, 'node': None,
            't2_code': None, 'n_c': 0, 'n_r': 0, 'mad_h': None,
            'mad_v': None, 'tc': None, 'tr': None, 'threshold': th,
            'status': status, 'why': status, 'line_x': None, 'line_y': None,
            'centre': (w // 2, h // 2)}


class Tile2Tracker:
    """Same analyse() contract as pipeline.GridTracker, for Tile 2."""

    def __init__(self, w, h, cfg, loaded=None):
        lib, tab, names = loaded if loaded is not None else _load()
        # u16pipe takes one frame size only (the bench's Tile2 check)
        fw, fh = getattr(lib, 'FRAME_W', 320), getattr(lib, 'FRAME_H', 200)
        if (w, h) != (fw, fh):
            raise ValueError('Tile 2 needs a %dx%d frame (sensor gives %dx%d)'
                             % (fw, fh, w, h))
        self.lib = lib
        self.W, self.H = w, h
        self.args = pipe_args(cfg)
        self.pipe = lib.Pipe(tab, names, *self.args)
        self.status = ''
        self.n_rejected = 0
        self.last = None            # the last Pipe.step() result (telemetry)
        self.NC = self.NR = self.NAC = self.NAR = 0   # Tile 1 scan lines: none
        self.mm = cfg['mm_per_px']  # Tile 2 mm per corrected px (quality)
        self.th = 0                 # last Otsu threshold (prepare_quality)
        self._th_new = True         # take a threshold on the next call
        self._light_due = 0         # frame index of the next light read
        # signals() fills this one dict every frame (no new dict a frame)
        self._sig = {'lost': True, 'hold': False, 'research': False,
                     'jz': False, 'cov_r': None, 'cov_x': None,
                     'rms_r': None, 'rms_x': None, 'n_r': 0, 'n_x': 0,
                     'perp': None, 'code_full': False, 'named': False}

    def reset(self):
        """A tile switch, a light change or a frame error: the Pipe back to
        new, with no memory, and a new Otsu threshold and light read on the
        next prepare_quality(). The scorer is the loop's; it resets that
        itself."""
        self.pipe.reset()
        self.last = None
        self.th = 0
        self._th_new = True
        self._light_due = 0

    def reseed(self):
        """Back from a turn: nothing to clear, the Pipe tracks through it
        (bench Tile2.reseed). A reset would lose the quarter index and the
        junction name."""

    def force_full(self):
        """REACQUIRE: nothing; forcing searches would drop good windows
        (bench Tile2.force_full)."""

    def turn_step(self, buf):
        """A turn frame: the one Pipe.step() of this frame (the Pipe counts
        frames for its search rhythm, so never twice), so it tracks through
        the turn. Returns its grad4 angle as (raw, E, n) for the TurnSM, or
        None when the frame has no edges (the TurnSM skips it)."""
        r = self.pipe.step(buf)
        self.last = r
        self.status = r['mode']
        G = self.pipe.G
        if G[2] == 0:
            return None
        return grad4_angle(G)

    def analyse(self, buf, th, mm_per_px, prof=None):
        r = self.pipe.step(buf)
        self.last = r
        lines = r['lines']
        hold = (not r['ok']) and lines is not None
        tier = None
        if lines == 'both' and r['y_mm'] is not None:
            tier = 'junction'
        elif hold:
            tier = None
        elif r['y_mm'] is not None:
            tier = 'y'
        elif r['x_mm'] is not None:
            tier = 'x'
        if hold:
            self.n_rejected += 1
        st = self.pipe.st
        self.status = r['mode']
        lx, ly, node = self._lines(r)
        return {'tier': tier, 'suspect': hold,
                'x_mm': r['x_mm'], 'y_mm': r['y_mm'], 'theta_deg': r['tdeg'],
                'theta_split': None, 'node': node, 't2_code': r['jname'],
                'n_c': st[0], 'n_r': st[3], 'mad_h': None, 'mad_v': None,
                'tc': None, 'tr': None, 'threshold': th,
                'status': r['mode'], 'why': None if tier else r['mode'],
                'line_x': lx, 'line_y': ly,
                'centre': (self.W // 2, self.H // 2)}

    def _lines(self, r):
        """The road and cross bar for the overlay, as the bench draws them
        (tile2.Tile2._draw_lines): through the junction when there is one,
        else through the point nearest the centre. Returned in the Tile 1
        overlay form: line_x = (b, a) for y = a + b*x, line_y = (b, a) for
        x = a + b*y; each bar goes in the slot its slope fits."""
        if not r['ok'] or r['lines'] is None or r.get('th') is None:
            return None, None, None
        th = r['th'] - r['quarter'] * (math.pi * 0.5)
        c, s = math.cos(th), math.sin(th)
        node = None
        if r['jx'] is not None:
            x, y = r['jx'], r['jy']
            node = (x, y)
        elif r['lines'] == 'road':
            x, y = self.lib.CXI + r['Y'] * s, self.lib.CYI - r['Y'] * c
        else:
            x, y = self.lib.CXI - r['X'] * c, self.lib.CYI - r['X'] * s
        lx = ly = None
        for dx, dy, on in ((c, s, r['lines'] != 'cross'),
                           (-s, c, r['lines'] != 'road')):
            if not on:
                continue
            if abs(dx) >= abs(dy):
                b = dy / dx
                lx = (b, y - b * x)
            else:
                b = dx / dy
                ly = (b, x - b * y)
        return lx, ly, node


    def debug(self):
        """The bench's own reading of the last frame, for telemetry: why a
        junction code is (not) read -- presence score (>= 70 = code block),
        this frame's raw read and its bit distance to the nearest code.
        uc: lines whose clean bar points failed, fitted again from every
        point (bench 2.1.0, G2), since the Pipe's last reset."""
        r = self.last or {}
        p = self.pipe
        return {'mode': r.get('mode'), 'lines': r.get('lines'),
                'pres': r.get('pres'), 'code_raw': r.get('code_raw'),
                'cdist': r.get('cdist'), 'code': r.get('code'),
                'node': r.get('node'), 'xnode': r.get('xnode'),
                'quarter': r.get('quarter'),
                'rej': [getattr(p, 'rej_angle', 0), getattr(p, 'rej_perp', 0),
                        getattr(p, 'rej_jump', 0), getattr(p, 'rej_plain', 0)],
                'uc': getattr(p, 'unclean', 0)}

    # ---- quality score (the bench's Tile2.prepare / Tile2.signals) -------

    def prepare_quality(self, img, frame_i, qs, otsu):
        """The score's light and flicker signals, BEFORE analyse(), every
        OTSU_EVERY frames (and on the first call after a reset): the Otsu
        threshold of a quarter-size copy feeds qs.otsu() (flicker), its
        histogram qs.light() every qs.light_every frames. Nothing here
        changes what the Pipe measures.
          img      this frame (image or camera.GrayFrame)
          frame_i  frames since this tile became active (the loop's
                   _tile_fi: 0 at boot and again on a switch)
          qs       this tile's quality.Quality, or None (then nothing runs)
          otsu     camera.Camera.otsu, passed in so this file does not
                   import the camera
        A threshold of 0 is a real value here (a black frame), so a reset
        is marked by a flag, not by th == 0. Returns True when Otsu ran
        (the loop's 'otsu' profile bucket)."""
        if qs is None:
            return False
        if not self._th_new and frame_i % OTSU_EVERY:
            return False
        self._th_new = False
        if frame_i >= self._light_due:
            th, bins = otsu(img, OTSU_SCALE, True)
            qs.light(bins, th)
            self._light_due = frame_i + qs.light_every
        else:
            th = otsu(img, OTSU_SCALE)
        self.th = th
        qs.otsu(th)
        return True

    def signals(self):
        """The quality signals of the last analyse() for
        quality.Quality.frame_sig(), from the Pipe result and its per-family
        numbers pipe.st (st[3f] points kept, st[3f+1] strips laid, st[3f+2]
        rms x65536, st[6] |90 - angle| x100, st[7] code margin). The same
        dict every frame: read it before the next call."""
        s = self._sig
        r = self.last
        if r is None:
            s['lost'] = True
            s['hold'] = s['research'] = s['jz'] = False
            s['cov_r'] = s['cov_x'] = s['rms_r'] = s['rms_x'] = None
            s['n_r'] = s['n_x'] = 0
            s['perp'] = None
            s['code_full'] = s['named'] = False
            return s
        p = self.pipe
        S = p.st
        ln = r['lines']
        lost = ln is None
        rf = p.quarter & 1         # bar family in the road role
        xf = rf ^ 1                # bar family in the cross role
        has_r = ln == 'both' or ln == 'road'
        has_x = ln == 'both' or ln == 'cross'
        cov_r = rms_r = cov_x = rms_x = None
        if has_r and S[3 * rf + 1]:
            cov_r = S[3 * rf] / S[3 * rf + 1]
            rms_r = S[3 * rf + 2] / 65536.0
        if has_x and S[3 * xf + 1]:
            cov_x = S[3 * xf] / S[3 * xf + 1]
            rms_x = S[3 * xf + 2] / 65536.0
        X = r['X']
        s['lost'] = lost
        s['hold'] = not r['ok'] and not lost
        s['research'] = r['research'] == 1
        s['jz'] = has_x and X is not None and abs(X * self.mm) <= ZONE_MM
        s['cov_r'], s['cov_x'] = cov_r, cov_x
        s['rms_r'], s['rms_x'] = rms_r, rms_x
        s['n_r'] = S[3 * rf] if has_r else 0
        s['n_x'] = S[3 * xf] if has_x else 0
        s['perp'] = S[6] / 100.0 if ln == 'both' else None
        s['code_full'] = (ln == 'both' and r['pres'] is not None
                          and r['pres'] >= self.lib.PRES_MIN
                          and S[7] >= self.lib.CODE_MARGIN)
        s['named'] = r['xnode'] is not None
        return s


class DeadTracker:
    """Tile 2 asked for but not loadable: every frame reads as nothing.
    Never Tile 1 instead -- on a Tile 2 floor that would steer on noise."""

    status = 'tile2-missing'
    n_rejected = 0
    NC = NR = NAC = NAR = 0

    th = 0
    mm = 0.0
    # every frame is lost (a quality score on it reads 0, 'lost')
    _LOST = {'lost': True, 'hold': False, 'research': False, 'jz': False,
             'cov_r': None, 'cov_x': None, 'rms_r': None, 'rms_x': None,
             'n_r': 0, 'n_x': 0, 'perp': None, 'code_full': False,
             'named': False}

    def __init__(self, w, h):
        self.W, self.H = w, h

    def reset(self):
        pass

    def reseed(self):
        pass

    def force_full(self):
        pass

    def turn_step(self, buf):
        return None

    def prepare_quality(self, img, frame_i, qs, otsu):
        return False

    def signals(self):
        return self._LOST

    def analyse(self, buf, th, mm_per_px, prof=None):
        return empty_result(self.W, self.H, th, self.status)
