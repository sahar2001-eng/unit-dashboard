# TurnService — owns the DRIVE/TURN state machine and puts it on the wire.
#
# WHERE THE PIECES SIT
#   vision/turn/turn_sm.py   the vendored state machine and Algorithm 2
#   this file                the app's half: transport, config, telemetry
#
# The upstream drop drove the machine from a bench capture loop with its own
# UART class. Here the loop is VisionService and the transport is the app's
# CRC-checked link, so this service is the seam between them. It follows
# AprilTagService, the existing precedent for a service that changes what the
# vision loop does.
#
# THE CAMERA DOES NOT DRIVE. Upstream is explicit — "the turn is ended by the
# host sending DRIVE, never by the camera" — and nothing here calls a motion
# command. The STM32 decides when to turn and when to stop; we measure the
# rotation and report it. The camera's command context on the STM32 refuses
# C/N/K/M 30 anyway, so this is belt and braces.

from .. import util
from ..service import Service
from .turn.turn_sm import (DRIVE, FAULT, REACQUIRE, STATE_NAME, TURN_INIT,
                           TURNING, TurnLink, TurnSM)

# Wire values of V 20, chosen so 0 is the safe one.
CMD_DRIVE, CMD_LEFT, CMD_RIGHT, CMD_STOP = 0, 1, 2, 3
_CMD_TEXT = {CMD_DRIVE: 'DRIVE', CMD_LEFT: 'TURN L',
             CMD_RIGHT: 'TURN R', CMD_STOP: 'STOP'}


class _Link(TurnLink):
    """Routes the machine's events onto the UART as CRC'd V 0 21 lines.

    Events are low rate (a handful per turn), so ASCII is affordable here —
    unlike the per-frame status, which goes out as a binary frame. See
    comms/uart_link.py for the measured cost of a blocking ASCII write.
    """

    def __init__(self, service):
        TurnLink.__init__(self)
        self._svc = service
        self.events_sent = 0
        self.last_event = None

    def send_event(self, name, **fields):
        self.last_event = name
        self.events_sent += 1
        self._svc._emit_event(name, fields)


class TurnService(Service):
    name = "turn"
    critical = False        # losing turn estimation must not reboot the board


    def setup(self):
        cfg = self.app.config
        self._enabled = cfg.get('turn.enabled', True)
        self._report_every = cfg.get('turn.report_every', 1)
        self.link = _Link(self)
        self.sm = None                  # built on the first frame (needs W/H)
        self._seq = 0
        self._frames = 0
        self._reports = 0
        self._commands = 0
        self._refused = 0
        self._fault_recover_ms = cfg.get('turn.fault_recover_ms', 1000)
        self._max_turn_ms = cfg.get('turn.max_turn_ms', 20000)
        self._seen = DRIVE              # last state observed, for the timers
        self._since = None              # when _seen was entered (ticks_ms)
        self._recoveries = 0
        self._turn_timeouts = 0
        # A tile switch (and a camera re-latch) waits switch_quiet_ms after
        # a turn ends. The STM32 runs a 180/360 as 90-degree sub-turns: it
        # sends the next V 20 a tick or two after the camera is back in
        # DRIVE, so DRIVE alone does not mean the rotation is over. Inside
        # this window a V 20 is the rest of that rotation, even with a
        # switch pending.
        self._quiet_ms = cfg.get('turn.switch_quiet_ms', 1000)
        self._turn_end_ms = None        # when the last turn went back to DRIVE
        cfg.on_change(self._on_config)

    def _on_config(self, path, value):
        def touched(section):
            if path is None:
                return isinstance(value, dict) and section in value
            return path.startswith(section + '.')
        if touched('turn'):
            cfg = self.app.config
            self._enabled = cfg.get('turn.enabled', True)
            self._report_every = cfg.get('turn.report_every', 1)
            self._fault_recover_ms = cfg.get('turn.fault_recover_ms', 1000)
            self._max_turn_ms = cfg.get('turn.max_turn_ms', 20000)
            self._quiet_ms = cfg.get('turn.switch_quiet_ms', 1000)

    # ------------------------------------------------------------- commands

    def command(self, code):
        """Inbound V 20. Returns the state name, or None if refused.

        Called synchronously from cmd_dispatch inside the UART poll, so it
        only queues — the machine acts on it in the next frame.
        """
        text = _CMD_TEXT.get(code)
        # Both tiles turn (1.32.0). While a tile switch is asked for and not
        # applied yet, a NEW turn would start on the old tile's tracker:
        # refused (DRIVE is always safe). The next sub-turn of a 180/360
        # (just after a turn) is not new: it runs, and the switch waits.
        vis = getattr(self.app, 'vision', None)
        pending = (vis is not None
                   and getattr(vis, 'tile_pending', lambda: False)()
                   and not self._just_turned())
        if (text is None or self.sm is None or not self._enabled
                or (pending and code != CMD_DRIVE)):
            self._refused += 1
            return None
        self._commands += 1
        if code == CMD_DRIVE and self.sm.state == FAULT:
            # The vendored machine would go to REACQUIRE and repeat the
            # 5-degree cross-check that already failed: FAULT again 1.5 s
            # later, for ever. The host says drive; drive.
            self._recover('host DRIVE')
            return 'DRIVE'
        self.link.inject(text)
        return STATE_NAME[self.sm.state]

    def state_name(self):
        return STATE_NAME[self.sm.state] if self.sm else 'DRIVE'

    def busy(self):
        """True while a tile switch (or a camera re-latch) must wait: not
        in DRIVE, a command queued that the machine has not acted on yet,
        or a turn ended less than turn.switch_quiet_ms ago."""
        sm = self.sm
        return sm is not None and (sm.state != DRIVE or bool(self.link._q)
                                   or self._just_turned())

    def _just_turned(self):
        """A turn went back to DRIVE less than switch_quiet_ms ago."""
        t = self._turn_end_ms
        if t is None:
            return False
        if util.ticks_diff(util.ticks_ms(), t) < self._quiet_ms:
            return True
        self._turn_end_ms = None        # past: no clock read next time
        return False

    def tile_switched(self):
        """The vision loop switched tiles (only in DRIVE): nothing seen on
        the old tile may seed a turn or count as landed on the new one."""
        sm = self.sm
        if sm is None:
            return
        sm.reseed_pending = False
        sm.last_fix = None
        sm._landed_n = 0
        sm._landed_sent = False

    # ---------------------------------------------- hooks for VisionService

    def ensure(self, w, h):
        if self.sm is None:
            self.sm = TurnSM(self.link, w, h)
            self.log.info('turn state machine %dx%d' % (w, h))
        return self.sm

    def alg1_active(self):
        """False while turning: Algorithm 1's fit is meaningless with the
        grid sweeping past, so the vision loop skips it entirely."""
        return True if self.sm is None else self.sm.alg1_active()

    def force_full_scan(self):
        return False if self.sm is None else self.sm.force_full_scan()

    def take_reseed(self):
        """One-shot: the caller must clear Algorithm 1's carried state after
        a turn. Upstream leaves this flag to the caller too."""
        if self.sm is not None and self.sm.reseed_pending:
            self.sm.reseed_pending = False
            return True
        return False

    def poll(self, frame_id, now_ms):
        if self.sm is not None:
            self.sm.poll_cmd(frame_id, now_ms)
            self._observe(now_ms)
            self._watchdog(now_ms)

    # --------------------------------------------- never stranded in a turn
    #
    # 2026-10-07, robot 0300: FAULT for 15 min, phi -18.4, no x/y, and only a
    # restart got out. The vendored machine (AUTO_MODE False) leaves FAULT
    # only on a host DRIVE that must then pass the cross-check that already
    # failed, and TURN_FREERUN disables its own turn timeout. This service is
    # app code, so the way out lives here rather than in the vendored file.
    #
    # The camera never drives, so returning to DRIVE is always safe: the
    # worst case is a heading that was not verified, which the STM32 hears
    # about (TURN_RECOVERED). No x/y at all is the worse failure.

    def _observe(self, now_ms):
        st = self.sm.state
        if st != self._seen or self._since is None:
            left_turn = self._seen != DRIVE and st == DRIVE
            self._seen = st
            self._since = now_ms
            if left_turn:
                self._turn_end_ms = now_ms
                # The 0xA8 status stops in DRIVE; one last frame saying so
                # lets the STM32 accept x/y at once instead of waiting for
                # the old status to age out.
                self._send_status()

    def _watchdog(self, now_ms):
        st = self.sm.state
        age = util.ticks_diff(now_ms, self._since)
        if st == FAULT and self._fault_recover_ms and \
                age >= self._fault_recover_ms:
            # Long enough for the STM32 to see FAULT in the status frames,
            # fail its rotation and power-cycle us itself if it wants to.
            self._recover('fault %d ms' % age)
        elif st in (TURN_INIT, TURNING) and self._max_turn_ms and \
                age >= self._max_turn_ms:
            self._turn_timeouts += 1
            self._recover('turn %d ms with no DRIVE' % age)

    def _recover(self, why):
        sm = self.sm
        phi = sm.turn['phi'] if sm.turn else 0.0
        # The same reset the vendored AUTO_MODE branch of _fault() does.
        sm.turn = None
        sm.last_fix = None
        sm._reacq_good = 0
        sm._landed_n = 0
        sm._landed_sent = True
        sm._drv_fail = 0
        sm.reseed_pending = True        # the tracker's band predates the turn
        sm.state = DRIVE
        self._recoveries += 1
        self._turn_end_ms = util.ticks_ms()
        self.log.warn('turn: back to DRIVE from %s (%s), heading unverified'
                      % (STATE_NAME[self._seen], why))
        sm.link.send('TURN_RECOVERED phi=%.1f' % phi)
        self._seen = DRIVE
        self._send_status()

    def turn_frame(self, buf, now_ms, angle=None):
        """Run Algorithm 2 on this frame. Does NOT transmit — see report().

        Reporting used to live here, and that was wrong: this is called from
        TWO places, and only one of them is a turn. The other is the rotation
        layer, which runs Algorithm 2 *on top of* Algorithm 1 near a junction
        as an internal measurement aid. Parked on a junction that is every
        frame, so the host was sent a status frame saying "phi 0.0, state
        DRIVE" at the full frame rate — measured 2026-09-09 as 173 frames/s on
        a link carrying 85 analysed frames/s, i.e. half the traffic carried no
        information. Transmitting is not this function's job.
        """
        if self.sm is None:
            return
        # angle: Tile 2's (raw, E, n) from its Pipe; None = measure here
        self.sm.turn_frame(buf, now_ms, angle)
        self._frames += 1

    def report(self):
        """Send the turn status. Called ONLY while a turn is in progress.

        That is also when Algorithm 1 is not running and no position frame is
        going out, so one status per frame leaves the link rate unchanged.
        """
        if self.sm is None or not self._report_every:
            return
        self._reports += 1
        if self._reports % self._report_every == 0:
            self._send_status()

    def drive_result(self, fix, now_ms):
        if self.sm is not None:
            self.sm.drive_result(fix, now_ms)
            if self.sm.state == REACQUIRE:
                # Algorithm 1 is running, so x/y are going out, but the turn
                # is NOT confirmed yet. Say so, or the STM32 takes the x/y as
                # the confirmation and is idle when the cross-check fails.
                self._send_status()
            self._observe(now_ms)

    def heading(self):
        return self.sm.heading() if self.sm else (0.0, 0.0)

    def fit(self):
        return getattr(self.sm, 'fit', None) if self.sm else None

    # -------------------------------------------------------------- the wire

    def _uart(self):
        return getattr(self.app, 'uart', None)

    def _send_status(self):
        """Per-frame turn status as the 0xA8 binary frame."""
        u = self._uart()
        if u is None:
            return
        phi, omega = self.sm.heading()
        self._seq = (self._seq + 1) & 0xFFFF
        from ..comms import protocol
        # through the UART FIFO when uart.fast_tx is on (no wait for the
        # wire), else the normal write; see UartLinkService.send_bytes
        u.send_bytes('turn', protocol.encode_turn(self._seq, phi, omega,
                                                  self.sm.state))

    def _emit_event(self, name, fields):
        # The ASCII event line stays on the normal write: a handful per turn,
        # and most are longer than the whole 32-byte FIFO.
        u = self._uart()
        if u is None:
            return
        from ..comms import protocol
        u._write(protocol.encode_turn_event(name, fields))

    # ------------------------------------------------------------ telemetry

    def stats(self):
        phi, omega = self.heading()
        return {'state': self.state_name(),
                'phi_deg': phi, 'omega_dps': omega,
                'frames': self._frames,
                'commands': self._commands, 'refused': self._refused,
                'reports_sent': self._reports,
                'events_sent': self.link.events_sent,
                'last_event': self.link.last_event,
                'fault_recoveries': self._recoveries,
                'turn_timeouts': self._turn_timeouts,
                'enabled': self._enabled}
