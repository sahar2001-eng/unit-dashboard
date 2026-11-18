# Loop-stall monitor: who held the asyncio loop, and for how long.
#
# There is no _thread on this port, so every service shares ONE asyncio loop,
# and the vision loop is what writes the frames to the STM32
# (vision_service -> bus 'frame' -> uart_link). Any task that runs without
# awaiting silences that link for its whole duration; the STM32 counts more
# than 2 s of silence as a link drop and refuses motion that needs the camera.
# On 2026-10-05 the culprit was NTP's DNS lookup in AP mode, every ~67 s, and
# it took a once-a-second poll of the STM32 to find it. This makes the next
# one name itself.
#
# Two measurements, kept apart because they need different fixes:
#   gap  -- time the vision loop spent yielded to OTHER tasks (note_gap)
#   self -- the vision loop's own time per frame, i.e. the frame period minus
#           the gap (note_self): snapshot, analyse, gc, a config re-latch
#
# Attribution is by explicit sections: known blocking work runs inside
# `with stall.section(app, 'ntp'):`. A gap is blamed on the section that
# overlaps it most; a gap no section covers is reported as 'unmarked', which
# is itself the useful answer -- it says where to add the next marker.
# A section must not span an await, or it would absorb other tasks' time.
#
# Cost on the hot path: one subtraction and one compare per frame.

from . import util

_RECENT = 8          # sections remembered for attribution


class _Section:
    def __init__(self, mon, label):
        self._mon = mon
        self._label = label
        self._t0 = 0

    def __enter__(self):
        self._t0 = self._mon._ticks()
        return self

    def __exit__(self, *exc):
        self._mon._record(self._label, self._t0, self._mon._ticks())
        return False                 # never swallow the body's exception


class _NullSection:
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


_NULL = _NullSection()


def section(app, label):
    """Mark a blocking stretch. A no-op when the app has no monitor (unit
    tests, or a monitor that failed to build)."""
    mon = getattr(app, 'stall', None)
    return mon.section(label) if mon is not None else _NULL


class StallMonitor:
    def __init__(self, threshold_ms=100, warn_ms=500, log=None, ticks=None):
        self._ticks = ticks or util.ticks_ms
        self.log = log
        self.configure(threshold_ms, warn_ms)
        self._recent = []                # (label, t0, t1), newest last
        self._sections = {}              # label -> [n, max_ms]
        self._stalls = 0
        self._over_warn = 0
        self._over_2s = 0
        self._max_gap = 0
        self._max_self = 0
        self._last = None

    def configure(self, threshold_ms, warn_ms):
        self._threshold = threshold_ms
        self._warn = warn_ms

    def section(self, label):
        return _Section(self, label)

    def _record(self, label, t0, t1):
        ms = util.ticks_diff(t1, t0)
        s = self._sections.get(label)
        if s is None:
            self._sections[label] = [1, ms]
        else:
            s[0] += 1
            if ms > s[1]:
                s[1] = ms
        self._recent.append((label, t0, t1))
        if len(self._recent) > _RECENT:
            self._recent.pop(0)

    def _suspect(self, t0, t1):
        best, best_ov = 'unmarked', 0
        for label, s0, s1 in self._recent:
            lo = s0 if util.ticks_diff(s0, t0) > 0 else t0
            hi = s1 if util.ticks_diff(s1, t1) < 0 else t1
            ov = util.ticks_diff(hi, lo)
            if ov > best_ov:
                best, best_ov = label, ov
        return best

    def note_gap(self, t0, t1):
        """The vision loop yielded at t0 and got the CPU back at t1 (ms)."""
        ms = util.ticks_diff(t1, t0)
        if ms > self._max_gap:
            self._max_gap = ms
        if ms < self._threshold:
            return
        suspect = self._suspect(t0, t1)
        self._stalls += 1
        if ms >= 2000:
            self._over_2s += 1
        self._last = {'ms': ms, 'at_ms': t1, 'suspect': suspect}
        if ms >= self._warn:
            self._over_warn += 1
            if self.log is not None:
                self.log.warn('loop held %d ms by %s' % (ms, suspect))

    def note_self(self, ms):
        """The vision loop's own time for one frame, gap excluded (ms)."""
        if ms > self._max_self:
            self._max_self = ms

    def stats(self):
        return {'stalls': self._stalls, 'over_warn': self._over_warn,
                'over_2s': self._over_2s, 'max_gap_ms': self._max_gap,
                'max_self_ms': self._max_self,
                'threshold_ms': self._threshold, 'warn_ms': self._warn,
                'last': self._last,
                'sections': {k: {'n': v[0], 'max_ms': v[1]}
                             for k, v in self._sections.items()}}
