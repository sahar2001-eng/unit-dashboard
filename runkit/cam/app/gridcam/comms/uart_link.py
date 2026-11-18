# UART link to the STM32 main board.
#
# WIRING: UART1 is board pins P4/P5 — the QWIIC connector's two signal
# lines (P4 = QWIIC SCL = TX = MCU P0_5, P5 = QWIIC SDA = RX = MCU P0_4).
# machine.UART(1) selects them by default; confirmed on fw v5.0 and
# exercised by host/uart_loopback.sh (bench jumper test, not production).
#
# Subscribes to 'frame' on the app bus and writes one measurement line per
# frame. A heartbeat goes out whenever no frame has been sent for
# `heartbeat_ms`, so the STM side can distinguish "no grid seen" from
# "camera dead".
#
# TX COST — measured, not assumed: machine.UART.write() on this port BLOCKS
# for the full wire time (~2.9 ms for a 34-byte line at 115200, matching the
# theoretical 34*10/115200). There is no way around it here: the `txbuf`
# kwarg is rejected and asyncio.StreamWriter.write() blocks identically. So
# one measurement line costs the vision loop ~3 ms per frame — roughly 5% of
# the frame period. Raising the baud is the only real lever, and that needs
# the STM32 side to agree. Do not describe this path as fire-and-forget.
#
# FAST TX (uart.fast_tx, 1.27.1): the per-frame 0xAA + 0xA9 go straight into
# UART1's 32-byte hardware FIFO (comms/fifo_tx.py) with no wait for the wire;
# since 1.32.0 the 0xA8 turn status and the keep-alives (uptime + IP, the
# heartbeat line) do too, all through send_bytes(). A FIFO without room for
# all of it sends it with the normal write; byte order is kept either way.
#
# FRAME FORMAT: the hot path sends the 8-byte binary frame (protocol.encode_xy)
# rather than the 39-byte ASCII line, because the blocking write above is
# charged directly against the frame rate: 0.69 ms instead of 3.39 ms at
# 115200. 'uart.frame_format' can select 'ascii' for bench debugging, where
# being able to read the traffic on a terminal is worth the cost.
#
# STREAM CONTROL: the STM32 is master. Streaming starts enabled but "V 2 0"
# stops it and "V 2 1" resumes; heartbeats keep flowing either way so silence
# still means the camera is gone rather than merely quiet.
#
# RX bytes are run through the CRC-checked LineDecoder and dispatched by
# cmd_dispatch.py — this is the first consumer of 'uart_cmd', which until now
# was published to nobody.

from .. import util
from ..service import Service
from . import protocol


class UartLinkService(Service):
    name = "uart"
    critical = True

    def setup(self):
        cfg = self.app.config.section('uart')
        import machine
        # rxbuf explicitly: the driver default (256 B) is smaller than one
        # rx_poll_ms window of continuous traffic at this baud, so bytes
        # would be dropped between polls. See DEFAULTS['uart'].
        self._uart = machine.UART(cfg['id'], cfg['baud'],
                                  rxbuf=cfg.get('rxbuf', 1024))
        self._heartbeat_ms = cfg['heartbeat_ms']
        self._rx_poll_ms = cfg['rx_poll_ms']
        self._decoder = protocol.LineDecoder()
        self._last_tx_ms = util.ticks_ms()
        self._seq = 0
        self._sent = 0
        self._dropped = 0
        self._xy_seq = 0
        self._frame_i = 0
        self._ka_seq = 0
        self._last_ka_ms = util.ticks_ms()
        self._boot_ms = util.ticks_ms()
        self._uart_id = cfg['id']
        self._fifo = None              # FifoTx once uart.fast_tx proved it
        self._fast_err = ''            # why the FIFO path was refused
        # per kind: [sent through the FIFO, found it full (sent normally)]
        self._fast = {'frame': [0, 0], 'turn': [0, 0],
                      'keepalive': [0, 0], 'heartbeat': [0, 0]}
        self._read_runtime_cfg(cfg)
        from .cmd_dispatch import CmdDispatch
        self._dispatch = CmdDispatch(self.app, self)
        self.app.bus.subscribe('frame', self._on_frame)
        self.app.bus.subscribe('uart_cmd', self._dispatch.handle)
        # Without this the uart.* settings are read once and never again --
        # the documented cache-and-forget trap, where POST /config appears to
        # succeed and silently does nothing. See vision_service._on_config.
        self.app.config.on_change(self._on_config)
        # log the driver's own view: it names the actual tx/rx pins, which is
        # the fastest way to diagnose "nothing arrives at the STM32"
        self.log.info('uart%d @ %d baud: %r' % (cfg['id'], cfg['baud'],
                                                self._uart))

    def _read_runtime_cfg(self, cfg):
        """Every uart.* setting that can change at run time, in one place.

        Deliberately re-read wholesale rather than per-key: missing one here is
        invisible -- the value simply becomes unchangeable, which is exactly
        how vision.print_csv went unnoticed.
        """
        self._stream_enabled = cfg.get('stream_enabled', True)
        self._keepalive_ms = int(cfg.get('keepalive_ms', 1000))
        self._divisor = max(1, int(cfg.get('divisor', 1)))
        self._quality_frame = bool(cfg.get('quality_frame', True))
        self._fast_tx = bool(cfg.get('fast_tx', False))
        if self._fast_tx and self._fifo is None and not self._fast_err:
            from .fifo_tx import FifoTx
            try:
                self._fifo = FifoTx(self._uart_id)
                self.log.info('fast_tx: UART%d FIFO %d B'
                              % (self._uart_id, self._fifo.depth))
            except Exception as e:
                self._fast_err = str(e) or e.__class__.__name__
                self.log.warn('fast_tx refused: %s' % self._fast_err)
        self._format = cfg.get('frame_format', 'binary')
        # Degrees of band-fit disagreement above which the heading is sent
        # as absent rather than steered on. 0 disables the gate.
        self._theta_max_split = float(cfg.get('theta_max_split_deg', 0))

    def _on_config(self, path, value):
        touched = (path is None and isinstance(value, dict) and 'uart' in value) \
            or (path is not None and path.startswith('uart.'))
        if not touched:
            return
        cfg = self.app.config.section('uart')
        self._read_runtime_cfg(cfg)
        # baud/id/rxbuf are NOT re-applied here: reopening the port mid-stream
        # would drop bytes, and a wrong baud would strand the link. The STM32
        # drives a rate change explicitly through cmd_dispatch.set_baud().
        self.log.info('uart cfg: stream=%s div=%d fmt=%s'
                      % (self._stream_enabled, self._divisor, self._format))

    def set_stream(self, on):
        """Called by the dispatcher when the STM32 sends V 2."""
        self._stream_enabled = bool(on)

    def streaming(self):
        return self._stream_enabled

    def send_command(self, body):
        """Send an ASCII command to the STM32, CRC appended.

        `body` is everything before the CRC token, e.g. "L 5 128". This is the
        seam anything on the camera uses to drive the main board; it shares the
        wire with the binary x/y frames, which the STM32 tells apart by the
        first byte.

        ASCII lines (this, command replies, V 0 21 turn events) stay on the
        normal write, not uart.fast_tx: they are rare (one per command, a
        few per turn), not per frame, and many are longer than the whole
        32-byte FIFO, so they could not go through it anyway. Mixing the two
        paths keeps byte order (see send_bytes).
        """
        self._write(protocol.encode_line(body.split(' ')))

    def _own_ip(self):
        """Our address, or 0.0.0.0 when we are on no network.

        Read live rather than cached: the whole point is to track a change, and
        an AP fallback happens long after setup() has run.
        """
        try:
            wifi = getattr(self.app, 'wifi', None)
            if wifi is not None:
                ip = wifi.stats().get('ip')
                if ip:
                    return ip
        except Exception:
            pass
        return '0.0.0.0'

    def uptime_s(self):
        """Seconds since this service started.

        ticks_ms wraps, so measure with ticks_diff rather than subtracting --
        a raw subtraction gives a large negative once the counter rolls over.
        """
        return util.ticks_diff(util.ticks_ms(), self._boot_ms) // 1000

    def send_bytes(self, kind, a, b=None):
        """Send frame `a` (and `b` right behind it) through the FIFO when
        uart.fast_tx is on, else with the normal write. Returns True when
        the FIFO took them.

        All or nothing: `a` and `b` go into the FIFO together or not at all,
        so if `a` falls back to the normal write, `b` does too.

        Byte order on the wire is safe both ways. The normal write returns
        only when its last byte is in the FIFO (ports/alif/mpuart.c,
        mp_uart_tx_data; on its timeout the TX IRQ is turned off and the rest
        is dropped, never sent later), so nothing waits in a slow path when
        we write here, and the FIFO sends everything in the order written.

        `kind` names the counter: 'frame', 'turn', 'keepalive', 'heartbeat'.
        """
        if self._fast_tx and self._fifo is not None:
            n = self._fast[kind]
            if self._fifo.write(a if b is None else a + b):
                n[0] += 1
                self._sent += 1 if b is None else 2
                self._last_tx_ms = util.ticks_ms()
                return True
            n[1] += 1
        self._write(a)
        if b is not None:
            self._write(b)
        return False

    def _write(self, data):
        try:
            n = self._uart.write(data)
        except OSError:
            n = None
        if n == len(data):
            self._sent += 1
            self._last_tx_ms = util.ticks_ms()
        else:
            self._dropped += 1

    def _on_frame(self, r):
        # Runs on the VISION task's stack (the bus is a synchronous fan-out),
        # so everything here is charged against the frame rate. Keep it short.
        self._frame_i += 1
        if not self._stream_enabled:
            return
        if self._divisor > 1 and (self._frame_i % self._divisor):
            return

        self._xy_seq = (self._xy_seq + 1) & 0xFFFF
        if self._format == 'ascii':
            self._write(protocol.encode_measurement(
                r.frame_id, r.theta_deg, r.y_mm, r.x_mm,
                r.n_c, r.n_r, r.fps))
            return

        # mm -> hundredths of a mm. None stays None so encode_xy maps it onto
        # the invalid sentinel and the STM32 skips the sample rather than
        # steering on a fabricated zero. x is SIGNED against the nearer
        # junction and is None in the dead zone mid-cell; y is the
        # bar/junction offset — see protocol.py.
        x = None if r.x_mm is None else r.x_mm * 100.0
        y = None if r.y_mm is None else r.y_mm * 100.0

        # Heading rides in the SAME frame as x/y, not its own. It is what the
        # main board's line follower steers on alongside cross-track y, and
        # fusing the two only makes sense if they came from one image -- two
        # frames let a single CRC error pair a heading with the previous
        # frame's position, with nothing on the wire saying so.
        th = None if r.theta_deg is None else r.theta_deg * 100.0

        # Optional quality gate. `theta_split` is how far the two band fits
        # disagreed and only exists on the junction tier; a large value is a
        # frame whose angle should not be steered on. Default 0 = no gate,
        # deliberately: a threshold invented before anyone has seen real split
        # values would silently discard good frames, and "absent" is a reading
        # the main board already handles.
        if (th is not None and self._theta_max_split
                and r.theta_split is not None
                and abs(r.theta_split) > self._theta_max_split):
            th = None

        meas = protocol.encode_meas(self._xy_seq, x, y, th)
        # Frame quality right behind it, same seq. Another 8 bytes = another
        # ~0.69 ms of blocking write per frame (see uart.quality_frame).
        qual = None
        if self._quality_frame and getattr(r, 'q', None) is not None:
            qual = protocol.encode_quality(
                self._xy_seq, r.q, r.q_code, r.q_img, r.q_line, r.q_rec,
                r.q_flags)
        # uart.fast_tx: both frames into the hardware FIFO in one go, the
        # same bytes in the same order, and no wait for the wire. All or
        # nothing: if they do not all fit, the normal writes send them.
        self.send_bytes('frame', meas, qual)

    async def run(self):
        import asyncio
        poll_s = self._rx_poll_ms / 1000.0
        while True:
            await asyncio.sleep(poll_s)

            now = util.ticks_ms()

            # Keep-alive: unconditional, unlike the ASCII heartbeat below which
            # is suppressed while measurements flow. The STM32 needs a positive
            # "still here" even at full frame rate, because a stopped stream and
            # a dead camera are otherwise indistinguishable.
            if self._keepalive_ms and \
                    util.ticks_diff(now, self._last_ka_ms) >= self._keepalive_ms:
                self._last_ka_ms = now
                self._ka_seq = (self._ka_seq + 1) & 0xFFFF
                # Address report rides the same cadence. Cheap (8 bytes) and it
                # has to repeat: DHCP can move us, and an AP fallback moves us
                # to a different subnet entirely, with nothing to announce it.
                # Both through the FIFO together, or both the normal way.
                self.send_bytes('keepalive',
                                protocol.encode_uptime(self._ka_seq,
                                                       self.uptime_s()),
                                protocol.encode_ip(self._ka_seq,
                                                   self._own_ip()))

            if util.ticks_diff(now, self._last_tx_ms) >= self._heartbeat_ms:
                self._seq += 1
                self.send_bytes('heartbeat',
                                protocol.encode_heartbeat(self._seq))

            try:
                n = self._uart.any()
            except (OSError, AttributeError):
                n = 0
            if n:
                data = self._uart.read(n)
                if data:
                    for tokens in self._decoder.feed(data):
                        self.log.debug('rx cmd: %s' % ' '.join(tokens))
                        self.app.bus.publish('uart_cmd', tokens)
            self.heartbeat()

    def stats(self):
        # overruns was previously computed by the decoder and never surfaced,
        # so a link losing whole lines looked identical to a healthy one.
        return {'sent': self._sent, 'dropped': self._dropped,
                'bad_crc': self._decoder.bad_crc,
                'overruns': self._decoder.overruns,
                'seq': self._xy_seq, 'streaming': self._stream_enabled,
                'uptime_s': self.uptime_s(),
                'format': self._format, 'divisor': self._divisor,
                'fast_tx': ('refused' if self._fast_err else
                            'on' if self._fast_tx and self._fifo else 'off'),
                'fast_err': self._fast_err,
                'fast_frames': self._fast['frame'][0],
                'fast_full': self._fast['frame'][1],
                # every kind: [through the FIFO, found it full]
                'fast_kinds': dict((k, list(v)) for k, v in
                                   self._fast.items()),
                'quality_frame': self._quality_frame}
