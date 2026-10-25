# u16pipe.py -- UNIT16 per-frame pipeline, stages 1-5, using the C module u16c.mpy
#
#   p = Pipe(TAB, NAMES)          TAB = array('i') of the 400 code words, NAMES = 'NNH'
#   r = p.step(frame_bytes)       one 320x200 gray frame -> dict (see step())
#
# Stages: (1) strips + edges  (2) line fit  (3) bad-line rule  (4) code read
#         (5) X, Y, angle + tracking + junction count + node memory
#         (6) turns: grid angle from grad4 (mod 90, unwrapped), and the strips are
#             laid out at that angle (rotated scan), so driving and turning use the
#             same code with no 45 deg blind spot
# Lens: line points are corrected for the lens bend (LENS_K1) before the fit, so X, Y are
# in corrected pixels; 'jx', 'jy' (junction) are where it is in the camera image.
# Units: pixels inside, mm only at the output (MM_PER_PX is a PLACEHOLDER until
# the real plate sizes are known).
import math
from array import array
import u16c

W, H = 320, 200
MM_PER_PX = 0.244            # placeholder (old project). Change when sizes are known.
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
MISS_EVERY = 3               # one line tracked, the other missing: look for it every Nth frame
# ---- robustness (no false positives)
ANG_TOL = 5.0                # deg: a line must lie within this of the grid angle (step 0)
PERP_TOL = 4.0               # deg: road and cross must be perpendicular within this
HALF = 175                   # px: distance between two wide cross bars (measured on video)
SPACING = 2 * HALF           # px: code junction to code junction. Only code junctions give X:
                             # the plain wide bar half way between them is not used.
PRES_MIN = 70                # presence score: code block >= 100 (seen 100-200), plain junction
                             # (diagonals in big holes) <= 40. Below this -> plain bar, not used.
JUMP_MAX = 60                # px: X off the prediction by more than this -> frame not trusted
LENS_K1 = 0.15               # lens: corrected = c + d * (1 + K1 * (|d|/160)^2). Estimated from how
                             # straight the bars become on our videos (0.12-0.18 all similar).
                             # Replace with the calibrated value later. 0 = off.
K1S = LENS_K1 / 25600.0
GAP_SAME = 8                 # frames: X lost longer than this -> maybe another junction


class Pipe:
    def __init__(self, tab, names):
        self.P = array('i', [0] * 20)
        self.R = (array('i', [0] * 3300), array('i', [0] * 3300))
        self.IO = array('i', [0] * 10)
        self.P4 = array('i', [0] * 8)
        self.O4 = array('i', [0] * 8)
        self.L4 = array('i', [0] * 4)
        self.tab = tab
        self.names = names
        self.win = None          # [(lo,hi) road, (lo,hi) cross] in 1/256 px, from the last frame
        self.code = None         # last code read
        self.searches = 0
        self.PR = array('i', [0] * 20)
        self.G = array('i', [0] * 4)
        self.GP = array('i', [W, H, G4_STEP, G4_MIN, G4_SH, G4_TAP, 0, 0])
        self.theta = None        # continuous grid angle (deg), unwrapped across turns
        self.nframe = 0
        self.last_read = None    # code read on the previous frame (for the 2-frame check)
        self.bad_run = 0
        self.rej_angle = self.rej_perp = self.rej_jump = self.switches = 0
        self.rej_plain = 0
        self.lastX = None        # last X (px) and its frame, speed (px/frame)
        self.lastX_frame = 0
        self.vX = 0.0
        self.xnode = None        # code of the junction X is measured from (None = not read yet)
        self.conflicts = 0
        self.cross_ok = False    # the tracked cross bar was checked to be a code junction
        self.avoid = None        # px: a plain cross bar seen here -> next search looks one bar away

    # ---------------------------------------------------------------- helpers
    def _p(self, axis, start, stride, hw, lo, hi, tfix, allmode):
        P = self.P
        P[0] = W; P[1] = H; P[2] = axis; P[3] = start; P[4] = stride; P[5] = hw
        P[6] = 0; P[7] = 0; P[8] = lo if lo > 0 else 0; P[9] = hi
        P[10] = 14 * 256; P[11] = 26 * 256; P[12] = TCOEF; P[13] = tfix
        P[14] = allmode; P[15] = 1100; P[16] = REL

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
        P[0] = W; P[1] = H; P[2] = fam; P[3] = smin; P[4] = stride; P[5] = hw
        P[7] = ext; P[8] = lo; P[9] = hi; P[10] = 14 * 256; P[11] = 26 * 256
        P[13] = tfix; P[14] = allmode; P[15] = 1100; P[16] = REL; P[17] = cs; P[18] = sn
        P[19] = SEAM * 256 + (FIRSTRISE << 16)

    def search_fam(self, buf, fam, cs, sn):
        # coarse rotated pass for ONE bar family: every 20th strip, 5 px wide, all wide bars, vote
        self.searches += 1
        P = self.P
        P[0] = W; P[1] = H; P[5] = 2; P[12] = TCOEF
        tfix = u16c.noise(buf, P)
        self._r(fam, 10, 20, 2, -(1 << 20), 1 << 20, tfix, 1, cs, sn)
        m = u16c.rscan(buf, self.PR, self.R[fam])
        centre = 0
        if fam == 1 and self.avoid is not None:
            # the bar near here is a plain one: the code junction is one bar further
            centre = int((self.avoid - HALF if self.avoid > 0 else self.avoid + HALF) * 256)
            self.avoid = None
        c = u16c.pickwin(self.R[fam], m, centre)
        return None if c == -1 else (c - WIN * 256, c + WIN * 256)

    def lines(self, buf, wins, cs, sn, tracked=(False, False)):
        P = self.P
        P[0] = W; P[1] = H; P[5] = HW; P[12] = TCOEF
        tfix = u16c.noise(buf, P)
        out = []
        for fam in (0, 1):
            w = wins[fam]
            if w is None:
                out.append(None)
                continue
            self._r(fam, STRIDE // 2, STRIDE, HW, w[0], w[1], tfix, 0, cs, sn)
            m = u16c.rscan(buf, self.PR, self.R[fam])
            IO = self.IO
            IO[0] = m; IO[1] = 0
            IO[6] = fam; IO[7] = cs; IO[8] = sn; IO[9] = int(LENS_K1 * 65536)
            u16c.fit(self.R[fam], IO)
            if not IO[0] or IO[3] < GATE_N or IO[5] < GATE_SPAN or \
                    IO[4] > (GATE_RMS_TRACK if tracked[fam] else GATE_RMS):
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
        P4[0] = W; P4[1] = H; P4[2] = int(x * 256 + 0.5); P4[3] = int(y * 256 + 0.5)
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

    def read_code(self, buf, x, y, th):
        P4 = self.P4
        P4[0] = W; P4[1] = H; P4[2] = int(x * 256 + 0.5); P4[3] = int(y * 256 + 0.5)
        P4[4] = int(math.cos(th) * 16384 + 0.5)
        s = math.sin(th) * 16384
        P4[5] = int(s + 0.5) if s >= 0 else -int(-s + 0.5)
        P4[6] = CODE_SLIDE; P4[7] = CODE_STEP
        O4 = self.O4
        u16c.code(buf, P4, O4)
        if not O4[0] or not O4[5] or O4[6] < CODE_MARGIN:
            return None, None
        u16c.lookup(O4[1], self.tab, self.L4)
        if self.L4[1] > CODE_MAXD:
            return None, self.L4[1]
        return self.names[self.L4[0]], self.L4[1]

    @staticmethod
    def to_image(x, y):
        # corrected pixels -> where it really is in the camera image (inverse of the lens fix)
        dx, dy = x - CXI, y - CYI
        ex, ey = dx, dy
        for _ in range(3):
            f = 1.0 + K1S * (ex * ex + ey * ey)
            ex, ey = dx / f, dy / f
        return CXI + ex, CYI + ey

    @staticmethod
    def win_at(d):
        # line at corrected distance d from the centre -> scan window in image pixels
        d = d / (1.0 + K1S * d * d)
        return (int((d - WIN) * 256), int((d + WIN) * 256))

    # ---------------------------------------------------------------- one frame
    def _junction(self, road, cross):
        ar, br = road[0], road[1]
        ac, bc = cross[0], cross[1]
        su = (ac + bc * ar) / (1 - bc * br)
        return su, ar + br * su              # junction in the turned frame (s along u, t along v)

    def step(self, buf):
        self.nframe += 1
        g = self.grid_angle(buf)
        if g is None:
            self.win = None
            self.last_read = None
            return {'ok': False, 'mode': 'noedges'}
        if self.theta is None:
            self.theta = g
        else:
            d = g - self.theta
            while d >= 45.0:
                d -= 90.0
            while d < -45.0:
                d += 90.0
            self.theta += d                  # unwrap: continuous across turns
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
                lines[f] = self.lines(buf, [wins[0] if f == 0 else None, wins[1] if f == 1 else None], cs, sn)[f]
        # check: road and cross must be perpendicular; if not, drop the worse line
        held = None
        if lines[0] is not None and lines[1] is not None:
            if abs(math.degrees(math.atan(lines[0][1]) + math.atan(lines[1][1]))) > PERP_TOL:
                k = 0 if lines[0][2] > lines[1][2] else 1
                held = (k, lines[k])         # not used now, but keep looking there next frame
                lines[k] = None
                self.rej_perp += 1
        road, cross = lines
        # only code junctions give X: check the cross bar with the presence score
        pres = -1
        raw = dist = None
        x = y = None
        if road is not None and cross is not None:
            su, tv = self._junction(road, cross)
            x = 160 + su * cf - tv * sf      # junction, back to image pixels
            y = 100 + su * sf + tv * cf
            thc = math.degrees(th0 + math.atan(road[1]))
            thc = ((thc + 45.0) % 90.0) - 45.0
            ix, iy = self.to_image(x, y)
            self.code_call(buf, ix, iy, math.radians(thc))
            pres = self.O4[7] if self.O4[0] else -1
            if pres >= PRES_MIN:
                self.cross_ok = True
                raw, dist = self.decode()
            elif pres >= 0 or not self.cross_ok:
                # plain wide bar (or not checkable and never checked): not used
                if pres >= 0:
                    d_ = cross[0] / math.sqrt(1 + cross[1] * cross[1])
                    self.avoid = d_ / (1.0 + K1S * d_ * d_)     # in image pixels
                    self.rej_plain += 1
                cross = None
                x = y = None
                self.cross_ok = False
        elif cross is not None:
            if not (self.cross_ok and tracked[1]):
                cross = None                 # a cross bar alone is only used when it was checked
        if cross is None and not (held is not None and held[0] == 1 and self.cross_ok):
            self.cross_ok = False
        if held is not None and held[0] == 1 and not self.cross_ok:
            held = None
        if road is None and cross is None:
            self.win = None
            self.last_read = None
            return {'ok': False, 'mode': mode, 'theta': self.theta}
        switched = 0
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
            ref = (x, y)
        elif road is not None:               # between junctions: only the road -> Y and angle
            px = 160 - ar * sf; py = 100 + ar * cf
            Y = -(CXI - px) * s + (CYI - py) * c
            ref = None
        else:                                # only a cross bar -> X (distance to it) and angle
            qx = 160 + ac * cf; qy = 100 + ac * sf
            X = (CXI - qx) * c + (CYI - qy) * s
            ref = (qx, qy)
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
        mm = lambda v: None if v is None else v * MM_PER_PX
        return {'ok': trusted, 'mode': mode, 'lines': 'both' if (road and cross) else ('road' if road else 'cross'),
                'jx': ix if x is not None else None, 'jy': iy if x is not None else None, 'th': th, 'X': X, 'Y': Y, 'Xmm': mm(X), 'Ymm': mm(Y),
                'deg': math.degrees(th), 'grid': g, 'code': name, 'code_raw': raw, 'cdist': dist, 'pres': pres,
                'node': self.code, 'xnode': self.xnode if X is not None else None}
