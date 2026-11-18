# Crash handling: persisted crash reports ("coredumps") and reboot counters.
#
# CrashGuard.capture() writes a full traceback plus a system snapshot to
# /flash/crash/crash_<n>.txt so a crash in the field can be diagnosed after
# the fact (files are served by GET /crash and survive reboots).
#
# RtcCounters keeps (boot_count, crash_count) in the RTC's 4 KB backup RAM
# when available (survives resets without flash wear); otherwise it falls
# back to a small file. app/main.py (the bootstrap) contains its own private
# copy of the same layout — keep MAGIC/format in sync with main.py.

import gc
import sys
import time

from . import util

MAGIC = 0x47524343                     # "GRCC"
_FMT_LEN = 16                          # 4 x uint32: magic, boots, crashes, spare
CRASH_DIR = '/flash/crash'
COUNTER_FILE = '/flash/etc/counters.bin'


def _pack(magic, boots, crashes):
    import struct
    return struct.pack('<IIII', magic, boots, crashes, 0)


def _unpack(buf):
    import struct
    if len(buf) < _FMT_LEN:
        return None
    magic, boots, crashes, _ = struct.unpack('<IIII', buf[:_FMT_LEN])
    if magic != MAGIC:
        return None
    return boots, crashes


class RtcCounters:
    """Boot/crash counters in RTC backup RAM, file fallback."""

    def __init__(self):
        self._rtc = None
        try:
            import machine
            rtc = machine.RTC()
            rtc.memory()               # probe: raises if unsupported
            self._rtc = rtc
        except Exception:
            pass
        self.boots, self.crashes = self._load()

    def _load(self):
        raw = None
        if self._rtc is not None:
            try:
                raw = self._rtc.memory()
            except Exception:
                raw = None
        if raw is None:
            try:
                with open(COUNTER_FILE, 'rb') as f:
                    raw = f.read()
            except OSError:
                raw = b''
        vals = _unpack(raw)
        return vals if vals is not None else (0, 0)

    def _store(self):
        buf = _pack(MAGIC, self.boots, self.crashes)
        if self._rtc is not None:
            try:
                self._rtc.memory(buf)
                return
            except Exception:
                pass
        try:
            util.makedirs('/flash/etc')
            with open(COUNTER_FILE, 'wb') as f:
                f.write(buf)
        except OSError:
            pass

    def note_boot(self):
        self.boots += 1
        self._store()

    def note_crash(self):
        self.crashes += 1
        self._store()

    def clear_crashes(self):
        if self.crashes:
            self.crashes = 0
            self._store()


class CrashGuard:
    """Writes crash reports and rotates old ones."""

    def __init__(self, keep=8, crash_dir=CRASH_DIR):
        self._keep = keep
        self._dir = crash_dir

    def _next_index(self):
        import os
        try:
            names = sorted(n for n in os.listdir(self._dir)
                           if n.startswith('crash_'))
        except OSError:
            names = []
        # prune oldest so there is room for the report about to be written
        while len(names) >= self._keep:
            try:
                os.remove('%s/%s' % (self._dir, names.pop(0)))
            except OSError:
                break
        if not names:
            return 0
        try:
            return int(names[-1][6:-4]) + 1
        except ValueError:
            return len(names)

    def capture(self, exc, context=''):
        """Persist traceback + system snapshot. Never raises."""
        try:
            util.makedirs(self._dir)
            path = '%s/crash_%03d.txt' % (self._dir, self._next_index())
            with open(path, 'w') as f:
                f.write('context : %s\n' % context)
                f.write('uptime_s: %.1f\n' % util.uptime_s())
                f.write('time    : %s\n' % str(time.time()))
                try:
                    from . import VERSION, BUILD_DATE, BUILD_COMMIT
                    f.write('version : %s\n' % VERSION)
                    f.write('build   : %s %s\n' % (BUILD_DATE, BUILD_COMMIT))
                except Exception:
                    pass
                if hasattr(gc, 'mem_free'):
                    f.write('mem_free: %d\n' % gc.mem_free())
                f.write('--- traceback ---\n')
                if hasattr(sys, 'print_exception'):
                    sys.print_exception(exc, f)
                else:                              # CPython (desktop tests)
                    import traceback
                    traceback.print_exception(
                        type(exc), exc, exc.__traceback__, file=f)
            return path
        except Exception:
            return None

    def mark(self, name):
        """Leave a one-shot mark for the next boot (take_mark): a tiny
        file next to the reports, not listed with them. Never raises."""
        try:
            util.makedirs(self._dir)
            with open('%s/%s' % (self._dir, name), 'w') as f:
                f.write('1')
        except Exception:
            pass

    def take_mark(self, name):
        """True when mark(name) was left; the mark is removed, so it is
        read once, by the boot right after the one that left it."""
        import os
        path = '%s/%s' % (self._dir, name)
        try:
            os.stat(path)
        except OSError:
            return False
        try:
            os.remove(path)
        except OSError:
            pass
        return True

    def list_reports(self):
        """All persisted reports: service crashes (crash_*) AND bootstrap
        crash dumps (boot_*, written by /flash/main.py)."""
        import os
        try:
            return sorted(n for n in os.listdir(self._dir)
                          if n.startswith('crash_') or n.startswith('boot_'))
        except OSError:
            return []

    def read_report(self, name):
        if '/' in name or '..' in name:
            return None
        try:
            with open('%s/%s' % (self._dir, name)) as f:
                return f.read()
        except OSError:
            return None
