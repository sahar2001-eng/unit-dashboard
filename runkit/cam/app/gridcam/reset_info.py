# reset_info.py -- why did the board start again?
#
# The board restarts for several reasons and, until this existed, they were
# indistinguishable from outside: /crash only records an EXCEPTION, and a
# board that loses power writes nothing at all, so the log simply stops
# mid-line and the next boot looks like any other.
#
# That cost a real diagnosis on 2026-09-10: the camera dropped out in the
# middle of a commanded rotation, the main board over-rotated because its
# cached angle went stale, and finding out why took reading the flushed log
# by hand and noticing that the boot counter had moved.
#
# Three independent pieces of evidence, none of which needs a write:
#
#   cause      machine.reset_cause(), when the port has it. Not all do -- this
#              one has no machine.WDT and no RTC().memory() either -- so it is
#              probed, never assumed.
#   boots /
#   crashes    the bootstrap's counters. A crash is an exception the guard
#              caught; a restart WITHOUT one is either commanded or power.
#   prev_run_s how far the previous run got, taken from the last timestamp
#              already in the log file. Deliberately NOT a heartbeat file:
#              rewriting one every few seconds to catch a rare event would
#              cost continuous flash wear, and the log is already there.
#
# The verdict combines them. It is honest about uncertainty: on a port with no
# reset_cause, "power or reset" is as far as the evidence goes, and saying so
# is more useful than picking one.

_UNKNOWN = 'unknown'

# machine.reset_cause() codes are port-specific; these are the common
# MicroPython ones. An unrecognised value is reported as its number rather
# than mapped to a guess.
#
# MEASURED ON THIS BOARD (AE3, fw 5.0.1, 2026-09-10): machine.reset_cause()
# DOES exist here -- unlike machine.WDT, _thread and RTC().memory(), which do
# not -- and it returned 0 after a commanded machine.reset(). 0 is not one of
# the standard codes, so the port appears not to populate it on that path.
# That makes a LATER non-zero reading genuinely informative: if a power-loss
# boot reports something other than 0, the two become distinguishable and the
# verdict below can stop hedging. Until one is observed, it stays a hedge.
_CAUSE_NAMES = {
    0: 'not reported by this port',
    1: 'power-on',
    2: 'hard reset',
    3: 'watchdog',
    4: 'deep sleep',
    5: 'soft reset',
}


def _probe_cause():
    """(code, name). Absent on ports without it -- see the header."""
    try:
        import machine
    except ImportError:
        return None, 'no machine module'
    fn = getattr(machine, 'reset_cause', None)
    if fn is None:
        return None, 'unavailable on this port'
    try:
        code = fn()
    except Exception as e:
        return None, 'probe failed: %s' % e
    return code, _CAUSE_NAMES.get(code, 'code %s' % code)


def _prev_run_s(log_path):
    """Seconds the previous run reached, from the last timestamp in the log.

    Must be read BEFORE this boot's own lines are flushed, which is why
    probe() is called during startup rather than lazily on the first request.
    Returns None when there is nothing to read -- a first boot, a wiped
    filesystem, or a log that has just rotated.
    """
    try:
        with open(log_path) as f:
            tail = ''
            while True:
                chunk = f.read(1024)
                if not chunk:
                    break
                tail = (tail + chunk)[-1024:]
    except OSError:
        return None
    for line in reversed(tail.split('\n')):
        line = line.strip()
        if not line:
            continue
        try:
            return float(line.split(None, 1)[0])
        except (ValueError, IndexError):
            continue
    return None


def probe(boots=None, crashes=None, log_path='/flash/log/app.0.log'):
    """Gather the evidence once, at startup. Returns a dict for telemetry."""
    code, name = _probe_cause()
    prev = _prev_run_s(log_path)

    if crashes:
        verdict = 'crash (%d recorded)' % crashes
    elif code in (3,):
        verdict = 'watchdog'
    elif code in (5,):
        verdict = 'commanded reset'
    elif code in (1, 2):
        verdict = 'power-on'
    elif code == 0 and prev is not None:
        # Measured: this port reports 0 after a commanded reset, i.e. it is
        # "no reason recorded", not "power-on". Fall through to the duration,
        # which is the part that actually helps.
        verdict = 'power loss or reset (ran %.0f s)' % prev
    elif prev is None:
        verdict = _UNKNOWN
    else:
        # No reset_cause on this port. The previous run ended without writing
        # anything, which is what both a power loss and a machine.reset()
        # look like from here -- say so rather than pick one.
        verdict = 'power loss or reset (ran %.0f s)' % prev

    return {
        'cause_code': code,
        'cause': name,
        'verdict': verdict,
        'boots': boots,
        'crashes': crashes,
        'prev_run_s': prev,
    }
