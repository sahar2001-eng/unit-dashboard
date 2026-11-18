# Camera illumination light.
#
# This replaces the on-board WS2812 strip as the camera's light source. The
# lamp is not on this board at all: it is the six white LEDs on the STM32 main
# board, driven by the IS31FL3207. The camera owns the SETTING -- it is the
# thing that knows whether the frame is too dark -- and pushes it to the STM32
# over the UART link.
#
# The wire command is ONE line, "L 11 <level> <p4> <p5> <p6> <p7> <p8> <p9>":
# the brightness and each of the six white LEDs (PCB names LED4..LED9) as a
# percent of it. The camera's channel may use it (motion commands on that
# channel are refused, but illumination is not motion). It replaced
# "L 5 <level>" (2026-10-28), which the STM32 still accepts as level-only;
# an STM32 firmware older than that nacks L 11 and the lamp stays dark, so
# update the main board first.
#
# WHY NOT DRIVE A LOCAL STRIP: the useful light for a downward grid camera sits
# with the camera's field of view, which is the main board's LED ring, not a
# strip on the sensor module. Keeping the setting here and the hardware there
# means one owner for the value and one place that knows the wiring.
#
# COST: one short ASCII line, sent only when the level actually changes. At
# 115200 that is ~1 ms of blocking write, so it must never be issued per frame
# -- see _apply(), which is a no-op when nothing changed.
#
# WHAT PERSISTS: `brightness` does; `enabled` does NOT. The lamp therefore
# always comes up ON at the last brightness, and switching it off is a
# session-only act that a power cycle undoes.
#
# The alternative -- persisting "off" -- means a robot can boot dark and look
# broken, with the cause buried in /flash/etc/config.json. That happened during
# bring-up: a test sweep ended on 0, the off was saved, and every reboot after
# it came up dark. A light that always returns is the safer default; you can
# always turn it off again, but you cannot see that it is off.
#
# RE-ASSERT: the level is ALSO pushed every `reassert_s` seconds regardless.
# The STM32 comes up dark after any reset or reflash, and it has no idea what
# the level should be -- the camera owns that value. Without a periodic
# re-assert the cache says "already sent 128" and the lamp stays off until
# something happens to change it. Observed exactly that on hardware after
# reflashing the main board. One short line every 10 s is ~0.1% duty.
#
# PER TILE (1.32.0): Tile 2 may have its own level and LED percents
# (tile2.led_level / tile2.leds). The vision service hands them over with
# set_tile_light() on a tile switch; they are runtime only, never written to
# camled.*, and the re-assert sends them too. Back on Tile 1 they are
# cleared, so camled.* is what goes out again.

from .. import util
from ..service import Service

LEVEL_MAX = 255
LED_NAMES = ('led4', 'led5', 'led6', 'led7', 'led8', 'led9')


def clamp_level(value):
    """Clamp to the 0..255 the IS31 expects.

    Out-of-range is clamped rather than rejected: a caller asking for 300 wants
    'as bright as possible', and refusing would leave the lamp at whatever it
    was, which is the more surprising outcome.
    """
    try:
        v = int(value)
    except (TypeError, ValueError):
        return 0
    if v < 0:
        return 0
    if v > LEVEL_MAX:
        return LEVEL_MAX
    return v


def clamp_pct(value):
    """0..100, clamped like the level; junk reads 0 rather than raising."""
    try:
        v = int(value)
    except (TypeError, ValueError):
        return 0
    return 0 if v < 0 else (100 if v > 100 else v)


def led_percents(leds):
    """LED4..LED9 in wire order from the camled.leds dict."""
    from ..config import DEFAULTS
    dflt = DEFAULTS['camled']['leds']
    leds = leds if isinstance(leds, dict) else {}
    return tuple(clamp_pct(leds.get(n, dflt[n])) for n in LED_NAMES)


def wire_line(level, pct):
    return 'L 11 %d %s' % (level, ' '.join('%d' % p for p in pct))


def effective_level(enabled, brightness):
    """The level that actually goes on the wire. Off means 0, not 'skip'."""
    return clamp_level(brightness) if enabled else 0


class CamLedService(Service):
    """Owns the illumination setting; the STM32 owns the hardware."""

    name = "camled"

    def setup(self):
        from ..config import DEFAULTS
        cfg = self.app.config.section('camled')
        # Read the boot state from DEFAULTS, NOT the merged config: `enabled`
        # is deliberately not a persisted setting, so a stored value must not
        # win. Anything a previous build left behind is cleaned up below.
        self._boot_enabled = bool(DEFAULTS['camled'].get('enabled', True))
        self._enabled = self._boot_enabled
        self._brightness = clamp_level(cfg.get('brightness', 128))
        self._pct = led_percents(cfg.get('leds'))
        self._clear_persisted_enabled()
        self._sent = None          # last level actually put on the wire
        self._writes = 0
        self._reassert_s = int(cfg.get('reassert_s', 10))
        self._last_assert_ms = 0
        # the active tile's own light (None = camled.*); vision is built first
        self._tile_level = None
        self._tile_pct = None
        vis = getattr(self.app, 'vision', None)
        if vis is not None and hasattr(vis, 'tile_led'):
            lv, pct = vis.tile_led()
            self._set_tile(lv, pct)
        self.app.config.on_change(self._on_config)
        # Push the boot state so the lamp matches the config even if the STM32
        # restarted independently and came up dark.
        self._apply(force=True)

    def _clear_persisted_enabled(self):
        """Drop a stored camled.enabled left by an older build.

        Writing the default value is what removes it: config.set() prunes any
        overlay entry equal to its default, so this both cleans the file and
        leaves the effective value correct. Only written when it differs, so a
        clean board takes no flash write at boot.
        """
        try:
            if self.app.config.get('camled.enabled') != self._boot_enabled:
                self.app.config.set('camled.enabled', self._boot_enabled,
                                    persist=True)
                self.log.info('cleared persisted camled.enabled')
        except Exception as e:
            self.log.warn('could not clear camled.enabled: %s' % e)

    def _on_config(self, path, value):
        touched = (path is None and isinstance(value, dict) and 'camled' in value) \
            or (path is not None and path.startswith('camled.'))
        if not touched:
            return
        cfg = self.app.config.section('camled')
        # brightness only -- `enabled` is runtime state and must survive a
        # config change, or setting the brightness would silently switch a
        # deliberately-off lamp back on.
        self._brightness = clamp_level(cfg.get('brightness', 128))
        self._pct = led_percents(cfg.get('leds'))
        self._apply()

    def _set_tile(self, level, pct):
        self._tile_level = None if level is None else clamp_level(level)
        self._tile_pct = (None if pct is None
                          else tuple(clamp_pct(p) for p in pct))

    def set_tile_light(self, level=None, pct=None):
        """The active tile's own level and LED percents (LED4..LED9), None =
        camled.*. Runtime only (persists nothing); sent at once if that
        changes the lamp."""
        self._set_tile(level, pct)
        self._apply()

    def _wire_state(self):
        """(level, percents) that go on the wire now."""
        b = self._brightness if self._tile_level is None else self._tile_level
        pct = self._pct if self._tile_pct is None else self._tile_pct
        return effective_level(self._enabled, b), pct

    def _apply(self, force=False):
        """Send "L 11 <level> <p4>..<p9>" if the level or a % changed.

        Gated on change because the write blocks for the full wire time; a
        per-frame push would come straight off the camera's frame rate.
        """
        state = self._wire_state()
        level, pct = state
        if not force and state == self._sent:
            return
        link = getattr(self.app, 'uart', None)
        if link is None:
            return                     # link disabled or failed to open
        try:
            link.send_command(wire_line(level, pct))
        except Exception as e:
            self.log.error('camled write failed: %s' % e)
            return
        self._sent = state
        self._writes += 1
        self._last_assert_ms = util.ticks_ms()

    # -- public API, used by /cmd and the dispatcher ----------------------
    def set(self, enabled=None, brightness=None, persist=True):
        """Change either or both. Applied as ONE config patch.

        Setting the two keys separately fires the listener twice, and the first
        pass sees the new `enabled` against the OLD brightness -- so turning the
        lamp on at a new level briefly lit it at the previous one and cost two
        blocking writes instead of one. patch() collapses that into a single
        change.
        """
        if enabled is not None:
            self._enabled = bool(enabled)   # runtime only, never persisted
        if brightness is not None:
            self.app.config.patch({'camled': {'brightness':
                                              clamp_level(brightness)}},
                                  persist=persist)
        # patch() fires _on_config, which applies -- but a value equal to the
        # current one still needs a reconcile if a previous write failed.
        self._apply()

    async def run(self):
        """Re-assert the level periodically so the lamp survives an STM32
        reset. Cheap, and the only self-healing path: the main board cannot
        ask, because it does not know what the level should be."""
        import asyncio
        while True:
            await asyncio.sleep(max(1, self._reassert_s))
            if util.ticks_diff(util.ticks_ms(),
                               self._last_assert_ms) >= self._reassert_s * 1000:
                self._apply(force=True)
            self.heartbeat()

    def off(self):
        self.set(enabled=False)

    def level(self):
        return self._wire_state()[0]

    def stop(self):
        """Leave the bench dark rather than drawing current indefinitely."""
        try:
            link = getattr(self.app, 'uart', None)
            if link is not None:
                link.send_command(wire_line(0, self._pct))
        except Exception:
            pass

    def stats(self):
        sent = self._sent[0] if self._sent else None
        return {'enabled': self._enabled, 'brightness': self._brightness,
                'leds': dict(zip(LED_NAMES, self._pct)),
                'level': self.level(), 'sent': sent,
                'writes': self._writes,
                'tile_level': self._tile_level,
                'tile_leds': (None if self._tile_pct is None
                              else dict(zip(LED_NAMES, self._tile_pct))),
                # enabled is session state; brightness is what survives a boot
                'persisted': {'brightness': self._brightness}}
