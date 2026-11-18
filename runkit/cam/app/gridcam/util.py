# Small portability helpers shared across services.
#
# MicroPython has time.ticks_ms()/ticks_diff() (wrap-safe millisecond
# counters); CPython (desktop tests) does not. Everything in gridcam goes
# through these wrappers so the same code runs in both worlds.

import time

try:
    ticks_ms = time.ticks_ms
    ticks_us = time.ticks_us
    ticks_diff = time.ticks_diff
    ticks_add = time.ticks_add
except AttributeError:                    # CPython (desktop tests)
    _t0 = time.monotonic()

    def ticks_ms():
        return int((time.monotonic() - _t0) * 1000)

    def ticks_us():
        return int((time.monotonic() - _t0) * 1000000)

    def ticks_diff(a, b):
        return a - b

    def ticks_add(t, delta):
        return t + delta


def uptime_s():
    return ticks_ms() / 1000.0


def isdir(path):
    import os
    try:
        return (os.stat(path)[0] & 0x4000) != 0
    except OSError:
        return False


def makedirs(path):
    """mkdir -p (MicroPython os.mkdir has no parents/exist_ok)."""
    import os
    parts = path.strip('/').split('/')
    cur = ''
    for p in parts:
        cur = cur + '/' + p
        try:
            os.mkdir(cur)
        except OSError:
            pass


def file_size(path):
    import os
    try:
        return os.stat(path)[6]
    except OSError:
        return -1
