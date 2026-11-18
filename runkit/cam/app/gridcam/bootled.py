# Boot-phase status LED: fast WHITE blink while the app builds its services.
#
# Application._build() is synchronous (camera reset, WiFi join), so nothing in
# the app can toggle a pin during it — the blink is driven by machine.Timer.
# Ports without machine.Timer fall back to a solid light rather than nothing.
#
# "White" is produced by driving every colour of the RGB status LED at once;
# any pin name the board does not have is simply skipped, so a board with only
# red+green still gives a distinct boot colour.
#
# HealthService takes the LED over once the app is running (green heartbeat).

_timer = None
_pins = []
_on, _off = 0, 1        # AE3 status LEDs are active LOW (measured)


def _open(names):
    pins = []
    try:
        import machine
    except Exception:
        return pins
    for name in names:
        try:
            pins.append(machine.Pin(name, machine.Pin.OUT))
        except Exception:
            pass                # colour absent on this board — skip it
    return pins


def _write(value):
    for pin in _pins:
        try:
            pin.value(value)
        except Exception:
            pass


def _toggle(_timer_arg=None):
    # Runs in timer context: must not allocate or raise.
    for pin in _pins:
        try:
            pin.value(_on if pin.value() == _off else _off)
        except Exception:
            pass


def start(names, period_ms, active_low=True):
    """Begin the boot indication. Safe to call more than once."""
    global _timer, _pins, _on, _off
    stop()
    _on, _off = (0, 1) if active_low else (1, 0)
    _pins = _open(names)
    if not _pins:
        return
    _write(_on)
    try:
        import machine
        timer = machine.Timer(-1)
        timer.init(period=period_ms, mode=machine.Timer.PERIODIC,
                   callback=_toggle)
        _timer = timer
    except Exception:
        _timer = None           # no usable timer: stay solid, still visible


def stop():
    """Release the LED and leave it dark, for the heartbeat to take over."""
    global _timer, _pins
    if _timer is not None:
        try:
            _timer.deinit()
        except Exception:
            pass
        _timer = None
    _write(_off)
    _pins = []
