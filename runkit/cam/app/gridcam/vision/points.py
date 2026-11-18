# How many scan points each frame produced -- the evidence behind a lost X.
#
# Algorithm 1 takes one sub-pixel run centre per scanned line ("first valid
# width run wins"): n_c from the column scan (points on the horizontal band,
# what Y and theta are fitted from) and n_r from the row scan (points on the
# vertical bar, what X is fitted from). A junction needs n_c and n_r both
# >= pipeline.min_pts (8) plus the coverage / residual gates, so the spread
# of these two numbers while driving says how close each frame came to
# losing the junction.
#
# Kept per (tier, scan) because the two are not comparable otherwise: a FULL
# scan (acquire / reacquire / periodic) steps 2 px and visits ~2.5x the lines
# a TRACKED scan (step 5) does.
#
# Cost: a few integer updates per frame, no allocation. Reset with
# POST /cmd {"cmd": "points_reset"}; read from /telemetry vision.points.

_TIERS = ('junction', 'y', 'none')
_SCANS = ('track', 'full')


class _Acc:
    __slots__ = ('n', 'sum', 'min', 'max', 'under')

    def __init__(self):
        self.reset()

    def reset(self):
        self.n = 0
        self.sum = 0
        self.min = 0
        self.max = 0
        self.under = 0          # frames below min_pts

    def add(self, v, floor):
        if self.n == 0 or v < self.min:
            self.min = v
        if v > self.max:
            self.max = v
        self.n += 1
        self.sum += v
        if v < floor:
            self.under += 1

    def as_dict(self):
        if not self.n:
            return None
        return {'n': self.n, 'min': self.min, 'max': self.max,
                'mean': round(self.sum / self.n, 1), 'under_min': self.under}


class PointStats:
    def __init__(self):
        self._acc = {}
        for t in _TIERS:
            for s in _SCANS:
                self._acc[(t, s)] = (_Acc(), _Acc())     # (n_c, n_r)
        self.frames = 0
        self.why = {}           # 'tier:why/why_h' -> frames
        self.last_reject = None

    def reset(self):
        for a in self._acc.values():
            a[0].reset()
            a[1].reset()
        self.frames = 0
        self.why = {}
        self.last_reject = None

    def note(self, res, frame):
        """Why a frame was not a junction. Only non-junction frames."""
        if res['tier'] == 'junction':
            return
        k = '%s:%s/%s' % (res['tier'] or 'none', res.get('why'), res.get('why_h'))
        self.why[k] = self.why.get(k, 0) + 1
        if res['tier'] is None:
            self.last_reject = {
                'frame': frame, 'why': res.get('why'), 'why_h': res.get('why_h'),
                'n_c': res['n_c'], 'n_r': res['n_r'], 'tc': res['tc'], 'tr': res['tr'],
                'mad_h': res['mad_h'], 'mad_v': res['mad_v'], 'th': res['threshold']}

    def add(self, tier, status, n_c, n_r, min_pts):
        t = tier if tier in ('junction', 'y') else 'none'
        s = 'track' if status == 'track' else 'full'
        c, r = self._acc[(t, s)]
        c.add(n_c, min_pts)
        r.add(n_r, min_pts)
        self.frames += 1

    def as_dict(self, lines=None):
        out = {'frames': self.frames}
        for (t, s), (c, r) in self._acc.items():
            if c.n:
                out['%s/%s' % (t, s)] = {'cols': c.as_dict(), 'rows': r.as_dict()}
        if self.why:
            out['why'] = self.why
        if self.last_reject is not None:
            out['last_reject'] = self.last_reject
        if lines is not None:
            out['lines'] = lines
        return out
