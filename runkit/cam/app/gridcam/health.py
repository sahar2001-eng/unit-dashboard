# Health / environment monitoring and the hardware watchdog.
#
# Samples memory + gc statistics and per-service stats for /telemetry, and
# owns the WDT: it is fed ONLY while every critical service has heartbeated
# within its deadline — a hung vision loop therefore hard-resets the board,
# which routes through main.py's crash-loop counters.
#
# After `stable_after_s` of continuous health it calls app.mark_stable(),
# which commits an OTA trial slot and clears the crash counters.

import gc

from . import util
from .service import Service


# Manual-override colours: name -> which of led.colors to light together.
MIXES = {
    'off': (),
    'red': ('red',),
    'green': ('green',),
    'blue': ('blue',),
    'yellow': ('red', 'green'),
    'cyan': ('green', 'blue'),
    'magenta': ('red', 'blue'),
    'white': ('red', 'green', 'blue'),
}
MODES = ('on', 'off', 'slow', 'fast')


class HealthService(Service):
    name = "health"

    def setup(self):
        cfg = self.app.config.section('health')
        wcfg = self.app.config.section('watchdog')
        self._period_s = cfg['period_s']
        self._stable_after_s = cfg['stable_after_s']
        self._deadline_ms = wcfg['beat_deadline_ms']
        self._t0 = util.ticks_ms()
        self._stable_done = False
        self._last_ok = True
        self._led_state = 0
        self._tick = 0
        self._override = None       # None = service drives the LED
        self._open_leds()
        self.app.config.on_change(self._on_config)

        self._wdt = None
        if wcfg['enabled']:
            try:
                import machine
                self._wdt = machine.WDT(timeout=wcfg['timeout_ms'])
                self.log.info('WDT armed, timeout %d ms' % wcfg['timeout_ms'])
            except Exception as e:
                self.log.warn('WDT unavailable: %s' % e)

    def _open_leds(self):
        lcfg = self.app.config.section('led')
        enabled = lcfg['enabled']
        self._blink_ms = lcfg['blink_ms']
        # Measured on the AE3: the status LEDs are active LOW.
        self._on, self._off = (0, 1) if lcfg['active_low'] else (1, 0)
        self._slow_blink_ms = lcfg['slow_blink_ms']
        self._led_ok = self._led(lcfg['ok_pin']) if enabled else None
        self._led_fault = self._led(lcfg['fault_pin']) if enabled else None
        # Separate handles for manual override; same physical pins as
        # ok/fault plus blue, which the service itself never drives.
        self._led_colors = {}
        if enabled:
            for name, pin_name in lcfg['colors'].items():
                pin = self._led(pin_name)
                if pin is not None:
                    self._led_colors[name] = pin
        if not enabled:
            self.stop()                 # make sure both go dark now

    def _on_config(self, path, value):
        if path is None or (path and path.startswith('led.')):
            prev_ok, prev_fault = self._led_ok, self._led_fault
            self._led_ok = self._led_fault = None
            for led in (prev_ok, prev_fault):
                try:
                    if led is not None:
                        led.value(self._off)
                except Exception:
                    pass
            self._open_leds()

    @staticmethod
    def _led(name):
        """Open a status LED; None when unavailable (desktop, mocks)."""
        try:
            import machine
            return machine.Pin(name, machine.Pin.OUT)
        except Exception:
            return None

    # -- manual override -------------------------------------------------
    def set_led_override(self, color, mode):
        """Take the status LED away from the service. Raises ValueError on a
        bad request so the caller can answer 400."""
        if color not in MIXES:
            raise ValueError('color must be one of: %s'
                             % ', '.join(sorted(MIXES)))
        if mode not in MODES:
            raise ValueError('mode must be one of: %s' % ', '.join(MODES))
        self._override = {'color': color, 'mode': mode}
        self._apply_mix('off')          # start from a known state
        self._set_leds(self._last_ok)   # render immediately, don't wait a tick
        self.log.info('LED override: %s %s' % (color, mode))

    def clear_led_override(self):
        """Hand the LED back to the health heartbeat."""
        self._override = None
        self._apply_mix('off')          # blue is not driven by the service
        self.log.info('LED override cleared')

    def led_override(self):
        return self._override

    def _apply_mix(self, color):
        lit = MIXES.get(color, ())
        for name, pin in self._led_colors.items():
            try:
                pin.value(self._on if name in lit else self._off)
            except Exception:
                pass

    def _render_override(self):
        mode = self._override['mode']
        if mode == 'on':
            lit = True
        elif mode == 'off':
            lit = False
        elif mode == 'fast':
            lit = bool(self._tick & 1)
        else:                           # slow: divide the tick rate down
            div = max(1, self._slow_blink_ms // max(self._blink_ms, 1))
            lit = bool((self._tick // div) & 1)
        self._apply_mix(self._override['color'] if lit else 'off')

    def _set_leds(self, healthy):
        # green heartbeat while healthy, red solid while unhealthy
        # (red BLINK is safe mode's pattern — see safe_mode.py)
        try:
            self._tick += 1
            if self._override is not None:
                self._render_override()
                return
            if healthy:
                self._led_state ^= 1
                if self._led_ok is not None:
                    self._led_ok.value(self._on if self._led_state else self._off)
                if self._led_fault is not None:
                    self._led_fault.value(self._off)
            else:
                if self._led_ok is not None:
                    self._led_ok.value(self._off)
                if self._led_fault is not None:
                    self._led_fault.value(self._on)
        except Exception:
            pass

    def _services_alive(self):
        now = util.ticks_ms()
        for svc in self.app.services:
            if not svc.critical:
                continue
            if util.ticks_diff(now, svc.last_beat_ms()) > self._deadline_ms:
                return False, svc.name
            if not svc.healthy():
                return False, svc.name
        return True, None

    async def run(self):
        # The loop ticks at the LED blink rate and does the (comparatively
        # expensive) health check only every period_s. Driving the blink from
        # period_s directly made the heartbeat too slow to read.
        import asyncio
        last_check = util.ticks_ms()
        while True:
            await asyncio.sleep(max(self._blink_ms, 20) / 1000.0)
            self._set_leds(self._last_ok)

            now = util.ticks_ms()
            if util.ticks_diff(now, last_check) < int(self._period_s * 1000):
                continue
            last_check = now

            ok, culprit = self._services_alive()
            if ok:
                if self._wdt is not None:
                    self._wdt.feed()
            elif self._last_ok:
                self.log.error('service unhealthy: %s — WDT not fed' % culprit)
            self._last_ok = ok
            self.heartbeat()

            app_up_s = util.ticks_diff(util.ticks_ms(), self._t0) / 1000.0
            if (ok and not self._stable_done
                    and app_up_s >= self._stable_after_s):
                self._stable_done = True
                self.app.mark_stable()

    def snapshot(self):
        """Full telemetry dict (also used by the HTTP API)."""
        mem = {}
        if hasattr(gc, 'mem_free'):
            free, alloc = gc.mem_free(), gc.mem_alloc()
            mem = {'free': free, 'alloc': alloc, 'total': free + alloc}
        flash = {}
        try:
            import os
            st = os.statvfs('/flash')
            flash = {'total': st[0] * st[2], 'free': st[0] * st[3]}
            flash['used'] = flash['total'] - flash['free']
        except (ImportError, AttributeError, OSError):
            pass
        per_service = {}
        for svc in self.app.services:
            try:
                per_service[svc.name] = svc.stats()
            except Exception:
                per_service[svc.name] = {'error': 'stats failed'}
        from . import VERSION, BUILD_DATE, BUILD_COMMIT
        return {
            'version': VERSION,
            'build': ('%s %s' % (BUILD_DATE, BUILD_COMMIT)).strip(),
            'uptime_s': util.ticks_diff(util.ticks_ms(), self._t0) / 1000.0,
            'mem': mem,
            'flash': flash,
            'healthy': self._last_ok,
            'stable': self._stable_done,
            'led_override': self._override,     # None = service-driven
            'reset': getattr(self.app, 'reset_info', None),
            'stall': (self.app.stall.stats()
                      if getattr(self.app, 'stall', None) else None),
            'services': per_service,
        }

    def stop(self):
        for led in (self._led_ok, self._led_fault):
            try:
                if led is not None:
                    led.value(self._off)
            except Exception:
                pass
        self._apply_mix('off')          # includes blue

    def stats(self):
        return {'wdt': self._wdt is not None, 'ok': self._last_ok}
