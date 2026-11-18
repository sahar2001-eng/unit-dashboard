# Application — composes all services on one asyncio loop and supervises them.
#
# Entry point: gridcam.app.run() (called by /flash/main.py after slot
# selection). Service order matters: config and logging first, then the
# hot path (vision, uart), then network extras — a WiFi or HTTP failure
# must never stop the robot from navigating.

import gc
import sys

from . import VERSION, util
from .config import ConfigService
from .crash import CrashGuard, RtcCounters
from .health import HealthService
from .logsvc import Logger, LogService
from .stall import StallMonitor


class Bus:
    """Minimal in-process pub/sub: topic string -> list of callbacks."""

    def __init__(self, log):
        self._subs = {}
        self._log = log

    def subscribe(self, topic, callback):
        self._subs.setdefault(topic, []).append(callback)

    def publish(self, topic, payload):
        for cb in self._subs.get(topic, ()):
            try:
                cb(payload)
            except Exception as e:
                self._log.error('bus %s handler: %s' % (topic, e))


class Application:
    def __init__(self):
        self.log = Logger()
        self.ticks_ms = util.ticks_ms
        self.bus = Bus(self.log.tagged('bus'))
        self.counters = RtcCounters()
        self.crash_guard = CrashGuard()
        self.config = ConfigService(self)
        # Built before any service so each can mark its blocking sections;
        # thresholds come from config once it has loaded (_build).
        self.stall = StallMonitor(log=self.log.tagged('stall'))
        self.services = []
        self._applog = self.log.tagged('app')
        self._stable = False

    # -- composition ----------------------------------------------------
    def _build(self):
        self.config.setup()
        self.services.append(self.config)
        self._configure_stall()
        self.config.on_change(self._on_stall_config)

        logsvc = LogService(self)
        logsvc.setup()
        self.services.append(logsvc)
        self.logsvc = logsvc

        self._applog.info('gridcam %s starting (boot %d, crashes %d)'
                          % (VERSION, self.counters.boots,
                             self.counters.crashes))

        # Read BEFORE the log service can flush this boot's own lines: the
        # previous run's last timestamp is the only evidence of how far it
        # got, and a power loss leaves nothing else behind. See reset_info.
        from . import reset_info
        self.reset_info = reset_info.probe(self.counters.boots,
                                           self.counters.crashes)
        self._applog.info('last restart: %s' % self.reset_info['verdict'])

        from .vision.vision_service import VisionService
        self.vision = VisionService(self)
        self.vision.setup()
        self.services.append(self.vision)

        if self.config.get('uart.enabled'):
            try:
                from .comms.uart_link import UartLinkService
                self.uart = UartLinkService(self)
                self.uart.setup()
                self.services.append(self.uart)
            except Exception as e:
                self.uart = None
                self._applog.error('uart disabled: %s' % e)
        else:
            self.uart = None

        self.apriltag = None
        if self.config.get('apriltag.enabled'):
            try:
                from .vision.apriltag_service import AprilTagService
                self.apriltag = AprilTagService(self)
                self.apriltag.setup()
                self.services.append(self.apriltag)
            except Exception as e:
                self._applog.error('apriltag disabled: %s' % e)

        # The turn state machine reports through the uart link and is driven
        # by inbound V 20, so it is built AFTER uart. Non-critical: losing
        # rotation estimation must not reboot a board that is driving fine.
        self.turn = None
        if self.config.get('turn.enabled', True):
            try:
                from .vision.turn_service import TurnService
                self.turn = TurnService(self)
                self.turn.setup()
                self.services.append(self.turn)
            except Exception as e:
                self.turn = None
                self._applog.error('turn disabled: %s' % e)

        # Camera illumination lives on the STM32 (six white LEDs), so this
        # must be built AFTER the uart link it pushes through.
        self.camled = None
        try:
            from .drivers.camled import CamLedService
            self.camled = CamLedService(self)
            self.camled.setup()
            self.services.append(self.camled)
        except Exception as e:
            self.camled = None
            self._applog.error('camled disabled: %s' % e)


        self.health = HealthService(self)
        self.health.setup()
        self.services.append(self.health)

        # network stack is strictly optional
        self.wifi = None
        self.httpd = None
        self.ntp = None
        if self.config.get('wifi.enabled'):
            try:
                from .net.wifi import WifiService
                self.wifi = WifiService(self)
                self.wifi.setup()
                self.services.append(self.wifi)
            except Exception as e:
                self._applog.error('wifi disabled: %s' % e)
        if self.wifi is not None and self.config.get('http.enabled'):
            try:
                from .net.httpd import HttpService
                self.httpd = HttpService(self)
                self.httpd.setup()
                self.services.append(self.httpd)
            except Exception as e:
                self._applog.error('http disabled: %s' % e)
        if self.wifi is not None and self.config.get('mdns.enabled'):
            try:
                from .net.mdns import MdnsService
                self.mdns = MdnsService(self)
                self.mdns.setup()
                self.services.append(self.mdns)
            except Exception as e:
                self._applog.error('mdns disabled: %s' % e)
        if self.wifi is not None and self.config.get('ntp.enabled'):
            try:
                from .net.ntp import NtpService
                self.ntp = NtpService(self)
                self.ntp.setup()
                self.services.append(self.ntp)
            except Exception as e:
                self._applog.error('ntp disabled: %s' % e)

    def _configure_stall(self):
        sc = self.config.section('stall')
        self.stall.configure(sc['threshold_ms'], sc['warn_ms'])

    def _on_stall_config(self, path, value):
        # Re-read on change, or POST /config would appear to succeed and do
        # nothing (the cache-and-forget pattern in openmv/CLAUDE.md).
        if (path is None and isinstance(value, dict) and 'stall' in value) \
                or (path is not None and path.startswith('stall.')):
            self._configure_stall()

    # -- supervision ----------------------------------------------------
    async def _supervise(self, svc):
        """Run a service task; a dead critical task reboots the board."""
        import asyncio
        try:
            await svc.run()
            self._applog.warn('service %s exited' % svc.name)
        except asyncio.CancelledError:
            raise
        except Exception as e:
            self._applog.error('service %s crashed: %s' % (svc.name, e))
            ctx = 'service:%s' % svc.name
            note = getattr(svc, 'crash_note', None)
            if note is not None:
                try:
                    ctx += ' ' + note()       # vision: 'tile=2'
                except Exception:
                    pass
            self.crash_guard.capture(e, context=ctx)
            if svc.critical:
                raise                     # escalate -> reboot via run()
        # non-critical: stay down, health reports it

    async def _main(self):
        import asyncio
        from .service import Service
        tasks = []
        for svc in self.services:
            if type(svc).run is Service.run:
                continue                  # passive service, no task
            tasks.append(asyncio.create_task(self._supervise(svc)))
        await asyncio.gather(*tasks)

    def mark_stable(self):
        """Called by HealthService after a healthy startup period."""
        if self._stable:
            return
        self._stable = True
        self.counters.clear_crashes()
        try:
            from .ota.app_ota import commit_trial
            if commit_trial():
                self._applog.info('OTA trial committed')
        except Exception as e:
            self._applog.error('OTA commit failed: %s' % e)
        self._applog.info('marked stable')

    def telemetry(self):
        return self.health.snapshot()

    def shutdown(self):
        for svc in self.services:
            try:
                svc.stop()
            except Exception:
                pass

    def run(self):
        import asyncio
        self.counters.note_boot()
        # White blink until every service is up; the green heartbeat then takes
        # over. DEFAULTS rather than self.config: the LED has to be lit before
        # ConfigService exists, and it must work even if the overlay is broken.
        from .config import DEFAULTS
        from . import bootled
        led_cfg = DEFAULTS['led']
        if led_cfg['enabled']:
            bootled.start(led_cfg['boot_pins'], led_cfg['boot_blink_ms'],
                          led_cfg['active_low'])
        try:
            self._build()
        finally:
            bootled.stop()
        gc.collect()
        try:
            asyncio.run(self._main())
        finally:
            self.shutdown()


def run():
    """Module-level entry point used by the bootstrap."""
    Application().run()
