# u16pipe.py -- UNIT16 per-frame pipeline, stages 1-5, using the C module u16c.mpy
#
#   p = Pipe(TAB, NAMES, mm_per_px=0.220, x_direction=1, x_range_mm=30.0,
#            spacing_mm=80.0, lens_k1=0.15)
#                                 TAB = array('i') of the 400 code words, NAMES = 'NNH'.
#                                 The other arguments come from the main.py
#                                 settings (T2_*); the defaults are the old values.
#   r = p.step(frame_bytes)       one 320x200 gray frame -> dict (see step())
#   p.reset()                     back to a new Pipe, with no new memory
#   p.st                          quality numbers of the last frame (see __init__)
#
# Stages: (1) strips + edges  (2) line fit  (3) bad-line rule  (4) code read
#         (5) X, Y, angle + tracking + junction count + node memory
#         (6) turns: grid angle from grad4 (mod 90, unwrapped), and the strips are
#             laid out at that angle (rotated scan), so driving and turning use the
#             same code with no 45 deg blind spot
# Lens: line points are corrected for the lens bend (lens_k1) before the fit, so X, Y are
# in corrected pixels; 'jx', 'jy' (junction) are where it is in the camera image.
# Units: pixels inside, mm only at the output (mm_per_px is a PLACEHOLDER until
# the real plate sizes are known).
# Roles: the quarter-turn index of the grid angle says which bar family is the
# road (along the travel) and which is the cross bar: family 0 is the road at
# quarter 0, family 1 after a quarter turn. X, Y and 'lines' are by role, so
# after a turn X is still along the travel.
# Tile 1 meaning: 'x_mm', 'y_mm', 'tdeg' give X, Y, theta as Tile 1 does (to_t1).
import math
from array import array
import u16c
from turn_sm import fold_x

FRAME_W, FRAME_H = 320, 200   # the only frame size the pipeline takes
WIN = 25                     # +-px window around a tracked line
TCOEF = int(4 * 1.4826 * 1024 + 0.5)
REL = 90                     # 35% rule
STRIDE, HW = 7, 1            # stage 1+2 choice: every 7th column/row, 3 px strips
GATE_N, GATE_SPAN, GATE_RMS = 6, 80, int(1.5 * 65536)
GATE_RMS_TRACK = int(2.5 * 65536)   # a line already tracked from the last frame may be less straight
                                    # (lens bends lines far from the centre): stops on/off flicker
CODE_SLIDE, CODE_STEP, CODE_MARGIN, CODE_MAXD = 6, 2, 8, 1
CXI, CYI = 159.5, 99.5       # image centre (camera axis)
G4_STEP, G4_TAP, G4_MIN, G4_SH = 5, 3, 24, 11   # grad4 settings (old turn_sm values)
SEAM = 5                     # px: a dark gap this thin inside a bar is a seam (plate joint), not a hole
FIRSTRISE = 1                # 1 = a 2nd rising edge before a falling one is inside the bar (seam)
CLEAN_D0, CLEAN_D1, CLEAN_K, CLEAN_KF = 5, 14, 141, 77
                             # clean-bar check (u16c rscan P[6]): a bar point counts only
                             # when the profile is dark from 5 to 14 px outside BOTH edges.
                             # A big square hole is ~37 px wide, so a real bar passes; a
                             # bright piece between two small code holes (inside the code
                             # block) or next to a crossing bar is bright there and is
                             # dropped. A band is dark when its mean is <= 141/256 = 55% of
                             # the bar, OR when it is flat near its own darkest sample
                             # (mean - darkest <= 77/256 = 30% of bar - darkest, and the bar
                             # is at least half an edge threshold brighter). The second test
                             # keeps real bars under a glare spot, which lifts the holes.
CLEAN_NMIN = 10              # a clean line needs this many points (else: again with every point);
                             # 6-9 clean points at the image edge gave lines a few deg off
CLEAN = CLEAN_D0 | (CLEAN_D1 << 8) | (CLEAN_K << 16) | (CLEAN_KF << 24)
MISS_EVERY = 3               # one line tracked, the other missing: look for it every Nth frame
# ---- robustness (no false positives)
ANG_TOL = 5.0                # deg: a line must lie within this of the grid angle (step 0)
PERP_TOL = 4.0               # deg: road and cross must be perpendicular within this
HALF = 175                   # px: distance between two wide cross bars (measured on video).
                             # Only code junctions give X: the plain wide bar half way
                             # between two of them is not used.
PRES_MIN = 70                # presence score: code block >= 100 (seen 100-200), plain junction
                             # (diagonals in big holes) <= 40. Below this -> plain bar, not used.
JUMP_MAX = 60                # px: X off the prediction by more than this -> frame not trusted
GAP_SAME = 8                 # frames: X lost longer than this -> maybe another junction
Q_HYST = 3.0                 # deg: the quarter-turn index changes only when theta is this far
                             # past the 45 deg middle (about 8x the grad4 frame noise)
_CS = ((1, 0), (0, 1), (-1, 0), (0, -1))     # cos, sin of k quarter turns


def turn_xy(X, Y, k):
    # (X, Y) turned by k quarter turns; None stays None
    c, s = _CS[k & 3]
    if s == 0:
        return (None if X is None else c * X), (None if Y is None else c * Y)
    return (None if Y is None else -s * Y), (None if X is None else s * X)


def to_t1(X, Y, mm, xdir, xr, sp):
    # Tile 2 X, Y (corrected px, travel frame) -> Tile 1 (x_mm, y_mm). Tile 2's
    # (X, Y) is minus Tile 1's offset, so the sign flips; X is folded as on
    # Tile 1 (None in the null zone) and then times xdir.
    x = None if X is None else fold_x(-X * mm, xr, sp)
    return (None if x is None else x * xdir), (None if Y is None else -Y * mm)


class Pipe:
    def __init__(self, tab, names, mm_per_px=0.220, x_direction=1, x_range_mm=30.0,
                 spacing_mm=80.0, lens_k1=0.15):
        # mm_per_px   mm per lens-corrected px (PLACEHOLDER: from the plate on our
        #             videos, CAD hole pitch 9.953 mm; the exact value needs the
        #             robot camera height)
        # x_direction +1 or -1: flips the Tile 1 X only
        # x_range_mm  X is given within this distance of a code junction (Tile 1 rule)
        # spacing_mm  code junction to code junction, the fold period (PLACEHOLDER)
        # lens_k1     lens: corrected = c + d * (1 + K1 * (|d|/160)^2). Estimated from how
        #             straight the bars become on our videos (0.12-0.18 all similar).
        #             0 = off; at most 1.0 (the u16c.fit limit)
        if len(tab) != len(names) or not len(tab):
            raise ValueError('Pipe: tab and names must be the same length, not 0 (%d, %d)'
                             % (len(tab), len(names)))
        if not mm_per_px > 0:
            raise ValueError('Pipe: mm_per_px must be > 0 (is %r)' % (mm_per_px,))
        if x_direction not in (1, -1):
            raise ValueError('Pipe: x_direction must be +1 or -1 (is %r)' % (x_direction,))
        if not 0 <= x_range_mm:
            raise ValueError('Pipe: x_range_mm must be >= 0 (is %r)' % (x_range_mm,))
        if not 1.0 <= spacing_mm <= 1e4:
            raise ValueError('Pipe: spacing_mm must be 1..10000 (is %r)' % (spacing_mm,))
        if not 0 <= lens_k1 <= 1.0:
            raise ValueError('Pipe: lens_k1 must be 0..1 (is %r)' % (lens_k1,))
        self.tab = tab
        self.names = names
        self.mm = mm_per_px
        self.xdir = x_direction
        self.xr = x_range_mm
        self.sp = spacing_mm
        self.K1S = lens_k1 / 25600.0
        self.k1i = int(lens_k1 * 65536)
        # work arrays, made once (layouts: the comment above each u16c function)
        self.P = array('i', [0] * 20)
        self.R = (array('i', [0] * 3300), array('i', [0] * 3300))
        self.IO = array('i', [0] * 10)
        self.P4 = array('i', [0] * 8)
        self.O4 = array('i', [0] * 8)
        self.L4 = array('i', [0] * 4)
        self.PR = array('i', [0] * 20)
        self.G = array('i', [0] * 4)
        self.GP = array('i', [FRAME_W, FRAME_H, G4_STEP, G4_MIN, G4_SH, G4_TAP, 0, 0])   # settings, never changed
        # quality numbers of the last frame, by bar FAMILY f (0 or 1):
        #   st[3f]   points kept by the fit (0 = the fit failed)
        #   st[3f+1] strips laid
        #   st[3f+2] fit rms x65536 px (0 = the fit failed)
        #   st[6]    |90 - angle between the two lines| x100 deg (0 = not both fits passed)
        #   st[7]    code margin px (-1 = no code read this frame)
        #   st[8]    code score
        self.st = array('i', [0] * 9)
        self.reset()

    def reset(self):
        # Back to the state of a new Pipe, with no new memory (a second Pipe
        # would cost about 26 KB). main.py calls it after the power-on light
        # search, which runs this same Pipe on its test frames.
        for a in (self.P, self.R[0], self.R[1], self.IO, self.P4, self.O4, self.L4, self.PR, self.G, self.st):
            for i in range(len(a)):
                a[i] = 0
        self.win = None          # [(lo,hi) road, (lo,hi) cross] in 1/256 px, from the last frame
        self.code = None         # last code read
        self.theta = None        # continuous grid angle (deg), unwrapped across turns
        self.nframe = 0
        self.last_read = None    # code read on the previous frame (for the 2-frame check)
        self.bad_run = 0
        self.lastX = None        # last X (px) and its frame, speed (px/frame)
        self.lastY = None        # Y (px) of the frame of lastX: lastX is turned with it
        self.lastX_frame = 0
        self.vX = 0.0
        self.xnode = None        # code of the junction X is measured from (None = not read yet)
        self.cross_ok = False    # the tracked cross bar was checked to be a code junction
        self.avoid = None        # px: a plain cross bar seen here -> next search looks one bar away
        self.quarter = 0         # quarter turns of theta (with Q_HYST): road family = quarter & 1
        self.research = 0        # 1 = a tracked line was lost and searched again this frame
        # counters (the replay record prints them; unclean only the app's
        # telemetry, vision.tile2.uc)
        self.searches = 0
        self.rej_angle = self.rej_perp = self.rej_jump = self.rej_plain = 0
        self.conflicts = 0
        self.unclean = 0         # clean line failed, done again with every point

    # ---------------------------------------------------------------- helpers
    def grid_angle(self, buf):
        # grad4: grid angle mod 90 from the whole frame (Scharr gradient, angle x4)
        u16c.grad4(buf, self.GP, self.G)
        if self.G[2] == 0:
            return None
        a = math.degrees(math.atan2(self.G[1], self.G[0])) * 0.25
        return ((a + 45.0) % 90.0) - 45.0

    def _r(self, fam, anchor, stride, hw, lo, hi, tfix, allmode, cs, sn):
        # strips only where the image is: half-extent of the image along the strip axis
        ac, asn = abs(cs) / 65536, abs(sn) / 65536
        ext = int((160 * ac + 100 * asn) if fam == 0 else (160 * asn + 100 * ac)) + 2
        a0 = anchor - (160 if fam == 0 else 100)           # same strips as the axis scan at 0 deg
        smin = a0 - stride * ((a0 + ext) // stride)
        P = self.PR
        P[0] = FRAME_W; P[1] = FRAME_H; P[2] = fam; P[3] = smin; P[4] = stride; P[5] = hw
        P[7] = ext; P[8] = lo; P[9] = hi; P[10] = 14 * 256; P[11] = 26 * 256
        P[6] = 0                      # clean check off (lines() turns it on)
        P[13] = tfix; P[14] = allmode; P[15] = 1100; P[16] = REL; P[17] = cs; P[18] = sn
        P[19] = SEAM * 256 + (FIRSTRISE << 16)

    def search_fam(self, buf, fam, cs, sn):
        # coarse rotated pass for ONE bar family: every 20th strip, 5 px wide, all wide bars, vote
        self.searches += 1
        P = self.P
        P[0] = FRAME_W; P[1] = FRAME_H; P[5] = 2; P[12] = TCOEF
        tfix = u16c.noise(buf, P)
        self._r(fam, 10, 20, 2, -(1 << 20), 1 << 20, tfix, 1, cs, sn)
        m = u16c.rscan(buf, self.PR, self.R[fam])
        centre = 0
        if fam == (self.quarter & 1) ^ 1 and self.avoid is not None:
            # the bar near here is a plain one: the code junction is one bar further
            centre = int((self.avoid - HALF if self.avoid > 0 else self.avoid + HALF) * 256)
            self.avoid = None
        c = u16c.pickwin(self.R[fam], m, centre)
        return None if c == -1 else (c - WIN * 256, c + WIN * 256)

    def lines(self, buf, wins, cs, sn, tracked=(False, False)):
        # One line per family, from one scan. rscan puts the CLEAN bar points
        # (dark big hole beside both edges, see CLEAN) first and gives their
        # number in PR[12]; then one point for each strip with bars but no clean
        # one. The line is fitted from the clean points. It is fitted again with
        # every point when there are other points, and that line is used if the
        # clean line fails the bad-line rule or has fewer than CLEAN_NMIN points
        # (e.g. a bar at the image edge or beside a plate edge), or if it keeps at
        # least as many points with no larger rms (the other points agree).
        P = self.P
        P[0] = FRAME_W; P[1] = FRAME_H; P[5] = HW; P[12] = TCOEF
        tfix = u16c.noise(buf, P)
        out = []
        for fam in (0, 1):
            w = wins[fam]
            if w is None:
                out.append(None)
                continue
            self._r(fam, STRIDE // 2, STRIDE, HW, w[0], w[1], tfix, 0, cs, sn)
            PR = self.PR
            PR[6] = CLEAN
            m = u16c.rscan(buf, PR, self.R[fam])
            IO = self.IO
            n = PR[12]                    # the clean points come first in R
            IO[0] = n; IO[1] = 0
            IO[6] = fam; IO[7] = cs; IO[8] = sn; IO[9] = self.k1i
            u16c.fit(self.R[fam], IO)
            rmax = GATE_RMS_TRACK if tracked[fam] else GATE_RMS
            good = IO[0] and IO[3] >= CLEAN_NMIN and IO[5] >= GATE_SPAN and IO[4] <= rmax
            if m > n:
                # every point (the strips with no clean bar too): used when the clean
                # line failed, or when the robust fit keeps at least as many points
                # with no larger rms (the other points agree with the line)
                c1, c2, c3, c4, c5 = IO[1], IO[2], IO[3], IO[4], IO[5]
                IO[0] = m; IO[1] = 0
                u16c.fit(self.R[fam], IO)
                if good and not (IO[0] and IO[3] >= c3 and IO[4] <= c4 and IO[5] >= GATE_SPAN):
                    IO[0] = 1; IO[1] = c1; IO[2] = c2; IO[3] = c3; IO[4] = c4; IO[5] = c5
                elif not good:
                    self.unclean += 1
                    good = IO[0] and IO[3] >= GATE_N and IO[5] >= GATE_SPAN and IO[4] <= rmax
            elif not good:
                self.unclean += 1         # no other points: the same line, the normal minimum
                good = IO[0] and IO[3] >= GATE_N and IO[5] >= GATE_SPAN and IO[4] <= rmax
            S = self.st
            S[3 * fam] = IO[3] if IO[0] else 0
            S[3 * fam + 1] = (2 * self.PR[7]) // STRIDE + 1
            S[3 * fam + 2] = IO[4] if IO[0] else 0
            if not good:
                out.append(None)          # bad-line rule
            elif abs(math.degrees(math.atan(IO[2] / 16777216))) > ANG_TOL:
                out.append(None)          # not along the grid angle -> not a grid bar
                self.rej_angle += 1
            else:
                out.append((IO[1] / 65536, IO[2] / 16777216, IO[4] / 65536))
        return out

    def code_call(self, buf, x, y, th):
        # one u16c.code call: fills O4 (code bits + presence score in O4[7])
        P4 = self.P4
        P4[0] = FRAME_W; P4[1] = FRAME_H; P4[2] = int(x * 256 + 0.5); P4[3] = int(y * 256 + 0.5)
        P4[4] = int(math.cos(th) * 16384 + 0.5)
        s = math.sin(th) * 16384
        P4[5] = int(s + 0.5) if s >= 0 else -int(-s + 0.5)
        P4[6] = CODE_SLIDE; P4[7] = CODE_STEP
        u16c.code(buf, P4, self.O4)

    def decode(self):
        O4 = self.O4
        if not O4[0] or not O4[5] or O4[6] < CODE_MARGIN:
            return None, None
        u16c.lookup(O4[1], self.tab, self.L4)
        if self.L4[1] > CODE_MAXD:
            return None, self.L4[1]
        return self.names[self.L4[0]], self.L4[1]

    def to_image(self, x, y):
        # corrected pixels -> where it really is in the camera image (inverse of the lens fix)
        dx, dy = x - CXI, y - CYI
        ex, ey = dx, dy
        K1S = self.K1S
        for _ in range(3):
            f = 1.0 + K1S * (ex * ex + ey * ey)
            ex, ey = dx / f, dy / f
        return CXI + ex, CYI + ey

    def win_at(self, d):
        # line at corrected distance d from the centre -> scan window in image pixels
        d = d / (1.0 + self.K1S * d * d)
        return (int((d - WIN) * 256), int((d + WIN) * 256))

    # ---------------------------------------------------------------- one frame
    def _junction(self, road, cross):
        ar, br = road[0], road[1]
        ac, bc = cross[0], cross[1]
        su = (ac + bc * ar) / (1 - bc * br)
        return su, ar + br * su              # junction in the turned frame (s along u, t along v)

    # step() returns a dict with the same 24 keys on every frame; a value that
    # does not exist on this frame is None.
    #   ok        False = do not trust this frame (no line, or X jumped: HOLD)
    #   mode      'track' / 'search' (a line was searched this frame) / 'noedges'
    #   lines     'both' / 'road' (the road bar only) / 'cross' (the cross bar only) / None
    #   jx, jy    code junction in the camera image, px
    #   th, deg   angle of bar family 0, continuous across turns (rad / deg)
    #   grid      grad4 grid angle mod 90, deg in [-45, 45)
    #   X, Y      corrected px, robot frame: X along the travel (from the cross
    #             bar), Y across it (from the road)
    #   Xmm, Ymm  X, Y times mm_per_px (not folded)
    #   code      code accepted this frame (2 frames in a row agree)
    #   code_raw  this frame's read; cdist its bit distance to the nearest code
    #   pres      code block presence score; -1 = not checked
    #   node      last accepted code (kept); xnode the junction X is measured from
    # The Tile 1 meaning (X along the travel, Y across it, then as Tile 1):
    #   quarter   quarter-turn index of theta; None only before the first angle
    #   tdeg      deg - 90*quarter (the Tile 1 THETA)
    #   x_mm      Tile 1 X: to_t1 of X (folded, null zone, x_direction); None
    #             when there is no X or it is in the null zone
    #   y_mm      Tile 1 Y = -Y * mm_per_px
    #   jname     xnode when x_mm is not None
    #   research  1 if a tracked line was lost and searched again this frame
    def _none(self, mode):
        # the form for a frame with no line
        return {'ok': False, 'mode': mode, 'lines': None, 'jx': None, 'jy': None,
                'th': None, 'deg': None, 'grid': None, 'X': None, 'Y': None,
                'Xmm': None, 'Ymm': None, 'code': None, 'code_raw': None, 'cdist': None,
                'pres': None, 'node': None, 'xnode': None,
                'quarter': None if self.theta is None else self.quarter, 'tdeg': None,
                'x_mm': None, 'y_mm': None, 'jname': None, 'research': self.research}

    def step(self, buf):
        self.nframe += 1
        S = self.st
        for i in range(9):
            S[i] = 0
        S[7] = -1
        self.research = 0
        g = self.grid_angle(buf)
        if g is None:
            self.win = None
            self.last_read = None
            return self._none('noedges')
        if self.theta is None:
            self.theta = g
        else:
            d = g - self.theta
            while d >= 45.0:
                d -= 90.0
            while d < -45.0:
                d += 90.0
            self.theta += d                  # unwrap: continuous across turns
        q_old = self.quarter
        if abs(self.theta - 90.0 * q_old) > 45.0 + Q_HYST:
            self.quarter = int(math.floor(self.theta / 90.0 + 0.5))
        q = self.quarter
        xf = (q & 1) ^ 1                     # bar family in the cross role
        if q != q_old and self.lastX is not None:
            # the same junction, seen a quarter turn further: turn the kept X too
            if self.lastY is None:
                self.lastX = None            # cannot turn it: the junction counts as new
            else:
                self.lastX, self.lastY = turn_xy(self.lastX, self.lastY, q - q_old)
            self.vX = 0.0
        th0 = math.radians(self.theta)
        cs = int(math.cos(th0) * 65536); sn = int(math.sin(th0) * 65536)
        cf, sf = cs / 65536, sn / 65536
        wins = list(self.win) if self.win else [None, None]
        tracked = [w is not None for w in wins]
        mode = 'track'
        for f in (0, 1):
            if wins[f] is None and (not tracked[1 - f] or self.nframe % MISS_EVERY == 0):
                wins[f] = self.search_fam(buf, f, cs, sn)
                mode = 'search'
        lines = self.lines(buf, wins, cs, sn, tracked)
        for f in (0, 1):                     # a tracked line was lost: search it again now
            if tracked[f] and lines[f] is None:
                wins[f] = self.search_fam(buf, f, cs, sn)
                mode = 'search'
                self.research = 1
                lines[f] = self.lines(buf, [wins[0] if f == 0 else None, wins[1] if f == 1 else None], cs, sn)[f]
        # check: road and cross must be perpendicular; if not, drop the worse line
        held = None
        if lines[0] is not None and lines[1] is not None:
            perp = abs(math.degrees(math.atan(lines[0][1]) + math.atan(lines[1][1])))
            S[6] = int(perp * 100)
            if perp > PERP_TOL:
                k = 0 if lines[0][2] > lines[1][2] else 1
                held = (k, lines[k])         # not used now, but keep looking there next frame
                lines[k] = None
                self.rej_perp += 1
        L = lines                            # by family; L[xf] is the cross bar
        # only code junctions give X: check the cross bar with the presence score
        pres = -1
        raw = dist = None
        x = y = None
        if L[0] is not None and L[1] is not None:
            road = L[0]
            su, tv = self._junction(L[0], L[1])
            x = 160 + su * cf - tv * sf      # junction, back to image pixels
            y = 100 + su * sf + tv * cf
            thc = math.degrees(th0 + math.atan(road[1]))
            thc = ((thc + 45.0) % 90.0) - 45.0
            ix, iy = self.to_image(x, y)
            self.code_call(buf, ix, iy, math.radians(thc))
            pres = self.O4[7] if self.O4[0] else -1
            if self.O4[0]:
                S[7] = self.O4[6]
                S[8] = self.O4[4]
            if pres >= PRES_MIN:
                self.cross_ok = True
                raw, dist = self.decode()
            elif pres >= 0 or not self.cross_ok:
                # plain wide bar (or not checkable and never checked): not used
                if pres >= 0:
                    cr = L[xf]
                    d_ = cr[0] / math.sqrt(1 + cr[1] * cr[1])
                    self.avoid = d_ / (1.0 + self.K1S * d_ * d_)     # in image pixels
                    self.rej_plain += 1
                L[xf] = None
                x = y = None
                self.cross_ok = False
        elif L[xf] is not None:
            if not (self.cross_ok and tracked[xf]):
                L[xf] = None                 # a cross bar alone is only used when it was checked
        if L[xf] is None and not (held is not None and held[0] == xf and self.cross_ok):
            self.cross_ok = False
        if held is not None and held[0] == xf and not self.cross_ok:
            held = None
        road, cross = L[0], L[1]             # geometry from here on: family 0 / family 1
        if road is None and cross is None:
            self.win = None
            self.last_read = None
            return self._none(mode)
        # windows for the next frame, at each line's distance from the centre
        nw = [None, None]
        if road is not None:
            ar, br = road[0], road[1]
            dr = ar / math.sqrt(1 + br * br)
            nw[0] = self.win_at(dr)
            th = th0 + math.atan(br)         # refined road angle (continuous)
        if cross is not None:
            ac, bc = cross[0], cross[1]
            dc = ac / math.sqrt(1 + bc * bc)
            nw[1] = self.win_at(dc)
            if road is None:
                th = th0 - math.atan(bc)     # road angle from the cross line (perpendicular)
        if held is not None:
            a_, b_ = held[1][0], held[1][1]
            d_ = a_ / math.sqrt(1 + b_ * b_)
            nw[held[0]] = self.win_at(d_)
        self.win = nw
        c, s = math.cos(th), math.sin(th)
        X = Y = None
        name = None
        if road is not None and cross is not None:
            X = (CXI - x) * c + (CYI - y) * s
            Y = -(CXI - x) * s + (CYI - y) * c
        elif road is not None:               # only family 0 -> Y and angle
            px = 160 - ar * sf; py = 100 + ar * cf
            Y = -(CXI - px) * s + (CYI - py) * c
        else:                                # only family 1 -> X (distance to it) and angle
            qx = 160 + ac * cf; qy = 100 + ac * sf
            X = (CXI - qx) * c + (CYI - qy) * s
        # X, Y above are in the frame of family 0; turn them to the robot frame
        # (X along the travel, whatever family the road is)
        X, Y = turn_xy(X, Y, q)
        # which junction is X measured from? Every junction we use has a code, so the code
        # read there names it. While X is seen continuously (short gaps, no jump) it is the
        # same junction and keeps its name. After a long gap it may be another junction:
        # its name is unknown ('?') until its code is read (2 frames in a row).
        trusted = True
        if X is not None:
            gap = self.nframe - self.lastX_frame
            same = False
            if self.lastX is not None and gap <= GAP_SAME:
                pred = self.lastX + self.vX * gap
                if abs(X - pred) <= JUMP_MAX:
                    same = True
                elif self.bad_run < 2:
                    trusted = False          # jumped: hold, give no output this frame
                    self.rej_jump += 1
                    self.bad_run += 1
            if trusted:
                if same:
                    v = max(-60.0, min(60.0, (X - self.lastX) / gap))
                    self.vX = 0.5 * self.vX + 0.5 * v
                else:
                    self.vX = 0.0
                    self.xnode = None        # maybe another junction: name unknown until read
                self.lastX = X
                self.lastY = Y
                self.lastX_frame = self.nframe
                self.bad_run = 0
        # code: accepted only when 2 frames in a row agree
        if not trusted:
            raw = None
        if raw is not None and raw == self.last_read:
            name = raw
            if self.xnode is not None and self.xnode[:-1] != name[:-1]:
                self.conflicts += 1          # same junction, other node number: should never happen
            self.xnode = name
            self.code = name
        self.last_read = raw
        mm = lambda v: None if v is None else v * self.mm
        x_mm, y_mm = to_t1(X, Y, self.mm, self.xdir, self.xr, self.sp)
        xnode = self.xnode if X is not None else None
        deg = math.degrees(th)
        return {'ok': trusted, 'mode': mode, 'lines': 'both' if (road and cross) else ('road' if L[q & 1] else 'cross'),
                'jx': ix if x is not None else None, 'jy': iy if x is not None else None,
                'th': th, 'deg': deg, 'grid': g,
                'X': X, 'Y': Y, 'Xmm': mm(X), 'Ymm': mm(Y),
                'code': name, 'code_raw': raw, 'cdist': dist, 'pres': pres,
                'node': self.code, 'xnode': xnode,
                'quarter': q, 'tdeg': deg - 90.0 * q, 'x_mm': x_mm, 'y_mm': y_mm,
                'jname': xnode if x_mm is not None else None, 'research': self.research}
