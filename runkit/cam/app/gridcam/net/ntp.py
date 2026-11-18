# NTP wall-clock sync + local-time formatting. Only meaningful with WiFi up,
# so the service is only built when the WiFi service exists (see app.py) and
# it waits for the 'wifi_up' bus event before its first attempt.
#
# Without this the board boots at 1970-01-01 (this port's epoch is Unix, not
# MicroPython's usual 2000), so logs and crash dumps carry only an uptime
# counter and cannot be correlated with anything off-board.
#
# THE CLOCK IS KEPT IN UTC. ntptime.settime() sets the RTC to UTC and we
# leave it there — a monotonic, DST-free system clock is what timestamps and
# interval arithmetic want. The timezone is applied only when formatting for
# humans, so a DST changeover can never make time appear to run backwards.
#
# COST: ntptime.settime() is a BLOCKING socket call, and timeout_s bounds
# only the socket wait -- NOT the DNS lookup of `host`, which can block far
# longer when there is no upstream. While it blocks, the one asyncio loop is
# held, the vision loop stops, and so do the frames to the STM32: more than
# 2 s and the STM32 counts a link drop. In AP mode that happened every ~67 s
# (2026-10-05), so:
#   - it is never attempted in AP mode;
#   - `mode` 'boot' (the default) syncs once after boot and stops, giving up
#     after `boot_attempts` failures; 'periodic' is the old behaviour; 'off'
#     never syncs.
# Every attempt runs inside a stall-monitor section, so if it does stall the
# loop, /telemetry names it.
#
# TODO (CLAUDE.md): take the time from the ESP32 over the UART and drop NTP
# from the camera altogether.

import time

from .. import stall, util
from ..service import Service

MODES = ('off', 'boot', 'periodic')


# -- calendar helpers -------------------------------------------------------
# Pure integer arithmetic rather than time.mktime(): mktime interprets its
# argument in the HOST timezone on CPython, which would make the desktop
# tests depend on where they run.

def _days_from_civil(y, m, d):
    """Days since 1970-01-01 for a proleptic Gregorian date."""
    y -= 1 if m <= 2 else 0
    era = (y if y >= 0 else y - 399) // 400
    yoe = y - era * 400
    doy = (153 * (m + (-3 if m > 2 else 9)) + 2) // 5 + d - 1
    doe = yoe * 365 + yoe // 4 - yoe // 100 + doy
    return era * 146097 + doe - 719468


def _utc_ts(y, m, d, hh=0, mm=0, ss=0):
    return _days_from_civil(y, m, d) * 86400 + hh * 3600 + mm * 60 + ss


def _weekday(y, m, d):
    """0 = Monday .. 6 = Sunday (1970-01-01 was a Thursday)."""
    return (_days_from_civil(y, m, d) + 3) % 7


def _last_sunday(y, m, last_day):
    return last_day - ((_weekday(y, m, last_day) + 1) % 7)


def israel_dst(utc_ts):
    """Is Israel Daylight Time in effect at this UTC timestamp?

    Rule in force since 2013: IDT starts on the FRIDAY BEFORE the last
    Sunday of March at 02:00 IST (= 00:00 UTC), and ends on the last Sunday
    of October at 02:00 IDT (= 23:00 UTC the previous day).
    """
    y = time.gmtime(utc_ts)[0]
    start = _utc_ts(y, 3, _last_sunday(y, 3, 31) - 2, 0)
    end = _utc_ts(y, 10, _last_sunday(y, 10, 31) - 1, 23)
    return start <= utc_ts < end


DST_RULES = {'israel': israel_dst, 'none': lambda _ts: False}


def offset_min(utc_ts, std_offset_min, dst_rule):
    """Total local offset from UTC in minutes, DST included."""
    rule = DST_RULES.get(dst_rule, DST_RULES['none'])
    return std_offset_min + (60 if rule(utc_ts) else 0)


def stamp(utc_ts, off_min=0):
    """'YYYY-MM-DD HH:MM:SS' for a UTC timestamp shifted by off_min."""
    tm = time.gmtime(int(utc_ts) + off_min * 60)
    return '%04d-%02d-%02d %02d:%02d:%02d' % (tm[0], tm[1], tm[2],
                                              tm[3], tm[4], tm[5])


def fmt_offset(off_min):
    sign = '+' if off_min >= 0 else '-'
    a = abs(off_min)
    return 'UTC%s%02d:%02d' % (sign, a // 60, a % 60)


class NtpService(Service):
    name = "ntp"
    critical = False               # a wrong clock must never stop the robot

    def setup(self):
        self._read_cfg()
        self._synced = False
        self._syncs = 0
        self._failures = 0
        self._attempts = 0             # since boot, for 'boot' mode
        self._last_offset_s = None
        self._last_sync_ms = None
        self._next_try_ms = util.ticks_ms()
        self.app.bus.subscribe('wifi_up', self._on_wifi_up)
        self.app.config.on_change(self._on_config)

    def _read_cfg(self):
        cfg = self.app.config.section('ntp')
        self._host = cfg.get('host', 'pool.ntp.org')
        self._timeout_s = cfg.get('timeout_s', 2)
        self._resync_s = cfg.get('resync_h', 12) * 3600
        self._retry_s = cfg.get('retry_s', 60)
        self._boot_attempts = cfg.get('boot_attempts', 3)
        mode = cfg.get('mode', 'boot')
        if mode not in MODES:
            if self.log is not None:
                self.log.warn('ntp mode %r unknown, using boot' % (mode,))
            mode = 'boot'
        self._mode = mode
        self._tz = cfg.get('tz', 'Asia/Jerusalem')
        self._tz_offset_min = cfg.get('tz_offset_min', 120)
        self._tz_dst = cfg.get('tz_dst', 'israel')

    def _on_config(self, path, value):
        touched = (path is None and isinstance(value, dict) and 'ntp' in value) \
            or (path is not None and path.startswith('ntp.'))
        if touched:
            old = self._mode
            self._read_cfg()
            if self._mode != old:
                # A mode change is a fresh start: a board switched to
                # 'periodic' after its one boot sync should resync on the
                # new schedule, not wait for a reboot.
                self._attempts = 0
                self._next_try_ms = util.ticks_ms()

    def _on_wifi_up(self, _ip):
        # Only force a sync if we have never had one. The run loop can beat
        # this event to the first sync (isconnected() goes true just before
        # 'wifi_up' is published), and unconditionally rearming here then
        # fired a second query ~5 s later — seen in the boot logs.
        if not self._synced:
            self._next_try_ms = util.ticks_ms()

    def _wifi_ready(self):
        wifi = getattr(self.app, 'wifi', None)
        try:
            if wifi is None:
                return False
            # In AP mode is_connected() is True, but nothing is upstream: the
            # DNS lookup blocks the loop and then fails, every retry_s.
            in_ap = getattr(wifi, 'in_ap_mode', None)
            if in_ap is not None and in_ap():
                return False
            return wifi.is_connected()
        except Exception:
            return False

    def _gave_up(self):
        return (self._mode == 'boot' and not self._synced
                and self._attempts >= self._boot_attempts)

    def _due(self):
        """Whether the mode and schedule call for an attempt now."""
        if self._mode == 'off':
            return False
        if self._mode == 'boot' and (self._synced or self._gave_up()):
            return False
        return util.ticks_diff(util.ticks_ms(), self._next_try_ms) >= 0

    def local_offset_min(self, utc_ts=None):
        if utc_ts is None:
            utc_ts = time.time()
        return offset_min(utc_ts, self._tz_offset_min, self._tz_dst)

    def local_stamp(self, utc_ts=None):
        if utc_ts is None:
            utc_ts = time.time()
        return stamp(utc_ts, self.local_offset_min(utc_ts))

    def sync(self):
        """One blocking NTP round trip. Returns True on success."""
        import ntptime
        before = time.time()
        self._attempts += 1
        try:
            ntptime.host = self._host
            ntptime.timeout = self._timeout_s
            with stall.section(self.app, 'ntp'):
                ntptime.settime()                # sets the RTC to UTC
        except Exception as e:
            self._failures += 1
            self._next_try_ms = util.ticks_add(util.ticks_ms(),
                                               self._retry_s * 1000)
            if self._gave_up():
                self.log.warn('NTP sync failed via %s: %s (giving up until '
                              'reboot after %d tries)'
                              % (self._host, e, self._attempts))
            else:
                self.log.warn('NTP sync failed via %s: %s (retry in %d s)'
                              % (self._host, e, self._retry_s))
            return False

        after = time.time()
        step = after - before
        off = self.local_offset_min(after)
        self._syncs += 1
        self._last_offset_s = step
        self._last_sync_ms = util.ticks_ms()
        self._next_try_ms = util.ticks_add(util.ticks_ms(),
                                           self._resync_s * 1000)
        what = 'time set' if not self._synced else 'resync'
        self._synced = True
        self.log.info('NTP %s from %s: %s %s (%s) | %s UTC | clock moved %+d s'
                      % (what, self._host, self.local_stamp(after), self._tz,
                         fmt_offset(off), stamp(after), step))
        return True

    async def run(self):
        import asyncio
        while True:
            if self._due() and self._wifi_ready():
                self.sync()
            await asyncio.sleep(5)
            self.heartbeat()

    def stats(self):
        s = {'synced': self._synced, 'syncs': self._syncs,
             'failures': self._failures, 'host': self._host,
             'tz': self._tz, 'mode': self._mode,
             'attempts': self._attempts, 'gave_up': self._gave_up()}
        if self._synced:
            now = time.time()
            s['local'] = self.local_stamp(now)
            s['utc'] = stamp(now)
            s['utc_offset_min'] = self.local_offset_min(now)
            s['dst'] = self.local_offset_min(now) != self._tz_offset_min
            s['last_offset_s'] = self._last_offset_s
            s['age_s'] = (util.ticks_diff(util.ticks_ms(),
                                          self._last_sync_ms) / 1000.0
                          if self._last_sync_ms is not None else None)
        return s

    def healthy(self):
        return True                # never fails the board over a clock
