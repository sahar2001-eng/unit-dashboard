# Wire protocol to the STM32 main board — pure, board-free, fully testable.
#
# Deliberately WIRE-COMPATIBLE with the robot's existing host command
# protocol (fw/stm_robot_fw App/cmd/): ASCII lines of space-separated
# tokens with a trailing decimal CRC-8 (poly 0x07, init 0x00) computed over
# the space-joined tokens before it:
#
#     "<letter> <subcommand> [args...] <crc>\n"
#
# so the STM side parses camera traffic with its existing cmd_feed()/crc8.
#
# Vision family: letter 'V'
#   V 0 <seq>                                   heartbeat (no fresh frames)
#   V 1 <frame> <flags> <theta_mdeg> <offset_um> <along_um>
#       <x_used> <y_used> <fps_x10>             measurement
#
# All values are integers (fixed point): theta millidegrees, distances
# micrometres, fps x10. flags: bit0 = theta valid, bit1 = X valid.
# Invalid fields are sent as 0 with their flag bit clear.
#
# Since gridcam 1.14 (pipeline v1_2): X is SIGNED against the NEARER junction,
# about -30 to +30 mm, and is absent in the dead zone mid-cell (bit1 clear).
# Y is the junction / bar offset from the frame centre. Both keep the field
# names below for wire compatibility. The mini-junction flags that briefly
# lived here went with the tier upstream removed.

FLAG_THETA_VALID = 0x01
FLAG_NODE_VALID = 0x02

LETTER = 'V'
SUB_HEARTBEAT = 0
SUB_MEASUREMENT = 1
SUB_TURN = 20           # host -> camera: 0 drive, 1 left, 2 right, 3 stop
SUB_TURN_EVENT = 21     # camera -> host: TURN_START / TURN_END / ...


def crc8(data):
    """CRC-8, polynomial 0x07, init 0x00 — matches App/cmd/crc8.c."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def encode_line(tokens):
    """Join tokens, append decimal CRC token and newline. Returns bytes."""
    body = ' '.join(str(t) for t in tokens)
    return ('%s %d\n' % (body, crc8(body.encode()))).encode()


# --- binary x/y frame (the high-rate hot path) ---------------------------
#
# The ASCII line above is readable and reuses the STM32's command parser, but
# it costs 39 bytes. At 200 fps that is 3.4 ms of blocking UART.write() out of
# a 5 ms frame budget — the link would become the bottleneck before the vision
# did. This fixed 8-byte frame costs 0.69 ms instead.
#
#   off  0     1       2       3      4      5      6      7
#       [A5] [seq_lo][seq_hi][x_lo] [x_hi] [y_lo] [y_hi] [crc8 over 0..6]
#
# Both framings share the wire. The STM32 tells them apart by the first byte:
# 0xA5 can never start an ASCII command line (those are 7-bit and begin with a
# letter), and the frame is fixed length, so a payload byte that happens to be
# 0x0A is not mistaken for a line terminator. See stm/Core/Inc/cam_proto.h.
#
# Units are hundredths of a millimetre, int16, so +/- 327.67 mm at 10 um. That
# is far finer than the sensor gives; the limit is the uncalibrated px->mm
# constant, not the quantisation.

FRAME_XY = 0xA5
FRAME_UPTIME = 0xA6
FRAME_IP = 0xA7
FRAME_TURN = 0xA8
FRAME_MEAS = 0xAA
FRAME_LEN = 8
# The combined measurement frame is 10 bytes, not 8: one sequence number and
# one CRC covering x, y AND theta. Sending theta as its own frame cost 16 bytes
# a frame against 10, and let a CRC error deliver a heading from one image
# paired with an x/y from the previous one -- which a follower fusing the two
# has no way to notice.
FRAME_MEAS_LEN = 10
XY_INVALID = -32768  # INT16_MIN — "no reading on this axis"; STM32 skips it

def _i16(v):
    """Clamp to int16, mapping None and out-of-range onto the invalid sentinel.

    Saturating would be worse than declaring the sample unusable: a clamped
    coordinate is indistinguishable from a real one at the edge of the frame,
    and the STM32 would steer on it.
    """
    if v is None:
        return XY_INVALID
    v = int(round(v))
    if v <= XY_INVALID or v > 32767:
        return XY_INVALID
    return v


def encode_xy(seq, x_cmm, y_cmm):
    """One 8-byte binary measurement frame. x/y in hundredths of a mm."""
    x = _i16(x_cmm) & 0xFFFF
    y = _i16(y_cmm) & 0xFFFF
    seq &= 0xFFFF
    buf = bytearray(FRAME_LEN)
    buf[0] = FRAME_XY
    buf[1] = seq & 0xFF
    buf[2] = (seq >> 8) & 0xFF
    buf[3] = x & 0xFF
    buf[4] = (x >> 8) & 0xFF
    buf[5] = y & 0xFF
    buf[6] = (y >> 8) & 0xFF
    buf[7] = crc8(buf[:FRAME_LEN - 1])
    return bytes(buf)


def encode_meas(seq, x_cmm, y_cmm, theta_cdeg):
    """The measurement frame: x, y and heading from ONE image.

        0     1       2       3     4     5     6     7      8      9
      [AA] [seq_lo][seq_hi][x_lo][x_hi][y_lo][y_hi][th_lo][th_hi][crc8 0..8]

    x/y in hundredths of a mm, theta in hundredths of a degree, each carrying
    XY_INVALID on its own when there is no reading. None of the three is ever
    sent as 0 for "absent": zero is legitimate for all of them -- dead centre
    on the line, or square to it -- so a fabricated zero reads as perfect at
    the exact moment the measurement was lost.

    WHY ONE FRAME AND NOT TWO. The camera's UART.write() blocks for the full
    wire time, so bytes come straight off the frame rate: 10 bytes is 0.87 ms
    where two 8-byte frames were 1.39 ms. And one sequence number with one CRC
    means the three values are provably from the same image -- with two frames
    a single CRC error silently pairs a heading with the previous frame's x/y.
    """
    x = _i16(x_cmm) & 0xFFFF
    y = _i16(y_cmm) & 0xFFFF
    th = _i16(theta_cdeg) & 0xFFFF
    seq &= 0xFFFF
    buf = bytearray(FRAME_MEAS_LEN)
    buf[0] = FRAME_MEAS
    buf[1] = seq & 0xFF
    buf[2] = (seq >> 8) & 0xFF
    buf[3] = x & 0xFF
    buf[4] = (x >> 8) & 0xFF
    buf[5] = y & 0xFF
    buf[6] = (y >> 8) & 0xFF
    buf[7] = th & 0xFF
    buf[8] = (th >> 8) & 0xFF
    buf[FRAME_MEAS_LEN - 1] = crc8(buf[:FRAME_MEAS_LEN - 1])
    return bytes(buf)


def encode_uptime(seq, seconds):
    """Keep-alive frame: same 8-byte envelope, uint32 uptime in seconds.

    Sent whether or not the x/y stream is running, so silence on the wire means
    the camera is gone rather than merely seeing no line. The STM32 watches the
    value for a DECREASE, which is how it detects that the camera restarted --
    the confirmation that a commanded power-cycle actually took effect.
    """
    seq &= 0xFFFF
    u = int(seconds) & 0xFFFFFFFF
    buf = bytearray(FRAME_LEN)
    buf[0] = FRAME_UPTIME
    buf[1] = seq & 0xFF
    buf[2] = (seq >> 8) & 0xFF
    buf[3] = u & 0xFF
    buf[4] = (u >> 8) & 0xFF
    buf[5] = (u >> 16) & 0xFF
    buf[6] = (u >> 24) & 0xFF
    buf[7] = crc8(buf[:FRAME_LEN - 1])
    return bytes(buf)


def encode_ip(seq, ip):
    """Report our own address. `ip` is a dotted string or a 4-tuple.

    Sent whether or not we are connected -- 0.0.0.0 is a real answer meaning
    "on no network", and the STM32 needs to be able to tell that apart from
    "the camera has not said". This is the only way to find the web UI without
    a serial console: DHCP moves the address, and a failed join drops us onto
    our own AP at a different one entirely.
    """
    if isinstance(ip, str):
        parts = ip.split('.')
        octets = [int(p) & 0xFF for p in parts] if len(parts) == 4 else [0, 0, 0, 0]
    else:
        octets = [int(v) & 0xFF for v in ip][:4]
        while len(octets) < 4:
            octets.append(0)
    seq &= 0xFFFF
    buf = bytearray(FRAME_LEN)
    buf[0] = FRAME_IP
    buf[1] = seq & 0xFF
    buf[2] = (seq >> 8) & 0xFF
    buf[3], buf[4], buf[5], buf[6] = octets
    buf[7] = crc8(buf[:FRAME_LEN - 1])
    return bytes(buf)


def _i16c(v):
    """Degrees (or deg/s) -> hundredths, clamped into int16.

    Clamped rather than sentinelled: unlike a coordinate, a saturated angle
    is still directionally right, and the state byte says whether a turn is
    even running.
    """
    v = int(round(v * 100.0))
    if v < -32768:
        return -32768
    if v > 32767:
        return 32767
    return v


def encode_turn(seq, phi_deg, omega_dps, state):
    """Per-frame turn status: 8-byte frame, same envelope and CRC as the rest.

        [A8][seq_lo][seq_hi][phi_lo][phi_hi][omega_lo][omega_hi][crc8]

    phi and omega are hundredths of a degree (and deg/s) as int16; the state
    rides in the top 3 bits of the sequence's high byte, which is free
    because seq only needs to detect gaps.

    Binary rather than an ASCII line because this goes out on EVERY turn
    frame: 0.69 ms of blocking write against 3.4 ms. A new frame type is
    safe against an older STM32 — cam_proto consumes and counts an unknown
    high-byte type rather than leaking it into the ASCII command parser.
    """
    seq &= 0x1FFF
    phi = _i16c(phi_deg) & 0xFFFF
    om = _i16c(omega_dps) & 0xFFFF
    hi = ((seq >> 8) & 0x1F) | ((int(state) & 0x07) << 5)
    buf = bytearray(FRAME_LEN)
    buf[0] = FRAME_TURN
    buf[1] = seq & 0xFF
    buf[2] = hi
    buf[3] = phi & 0xFF
    buf[4] = (phi >> 8) & 0xFF
    buf[5] = om & 0xFF
    buf[6] = (om >> 8) & 0xFF
    buf[7] = crc8(buf[:FRAME_LEN - 1])
    return bytes(buf)


FRAME_QUALITY = 0xA9
Q_NONE = 255            # "no score this frame" (turning, or the score is off)


def _nib(part):
    """A part score 0.0..1.0 -> 0..15."""
    if part is None:
        return 0
    v = int(part * 15.0 + 0.5)
    return 0 if v < 0 else (15 if v > 15 else v)


def encode_quality(seq, q, why, img, line, rec, flags=0):
    """Frame quality (vision/quality.py): 8 bytes, same envelope and CRC.

        [A9][seq_lo][seq_hi][q][why][img<<4 | line][rec<<4 | flags][crc8]

    seq   the SAME sequence number as the measurement frame (0xAA, or 0xA5
          on the old x/y-only format) of this camera
          frame, so the STM32 can pair a score with its sample
    q     0..100, or Q_NONE (255)
    why   reason code, index into quality.WHY_CODES (0 = fine)
    img/line/rec  the three parts, 0..15 each (15 = perfect)
    flags bit0 still learning "normal", bit1 the low alarm is on

    Sent right after the x/y frame it belongs to. An older STM32 that does
    not know 0xA9 consumes and counts it (cam_proto: any byte >= 0x80
    starts an 8-byte frame), so this is safe to send to any main board.
    """
    seq &= 0xFFFF
    if q is None:
        q = Q_NONE
    buf = bytearray(FRAME_LEN)
    buf[0] = FRAME_QUALITY
    buf[1] = seq & 0xFF
    buf[2] = (seq >> 8) & 0xFF
    buf[3] = int(q) & 0xFF
    buf[4] = int(why) & 0xFF
    buf[5] = (_nib(img) << 4) | _nib(line)
    buf[6] = (_nib(rec) << 4) | (int(flags) & 0x0F)
    buf[7] = crc8(buf[:FRAME_LEN - 1])
    return bytes(buf)


def encode_turn_event(name, fields=None):
    """A turn event as a CRC'd ASCII line: `V 0 21 <NAME> [k=v ...]`.

    Events are a handful per turn, so the ASCII cost is affordable and being
    readable on the debug bridge is worth more than the bytes. Field values
    are already formatted strings from the state machine.
    """
    tokens = [LETTER, 0, SUB_TURN_EVENT, name]
    for k in sorted(fields or {}):
        tokens.append('%s=%s' % (k, fields[k]))
    return encode_line(tokens)


def encode_heartbeat(seq):
    return encode_line((LETTER, SUB_HEARTBEAT, seq & 0xFFFF))


def encode_measurement(frame_id, theta_deg, offset_mm, along_mm,
                       x_used, y_used, fps):
    """Fixed-point measurement line; None fields -> 0 with flag clear."""
    flags = 0
    theta_mdeg = offset_um = along_um = 0
    if theta_deg is not None:
        flags |= FLAG_THETA_VALID
        theta_mdeg = int(round(theta_deg * 1000.0))
    if offset_mm is not None:
        offset_um = int(round(offset_mm * 1000.0))
    if along_mm is not None:
        flags |= FLAG_NODE_VALID
        along_um = int(round(along_mm * 1000.0))
    return encode_line((LETTER, SUB_MEASUREMENT, frame_id & 0xFFFFFFFF, flags,
                        theta_mdeg, offset_um, along_um,
                        x_used, y_used, int(round(fps * 10.0))))


class LineDecoder:
    """Reassembles RX bytes into CRC-checked token lists.

    feed() returns a list of token-lists (CRC token stripped); lines with a
    bad CRC are counted and dropped.
    """

    MAX_LINE = 128

    def __init__(self):
        self._buf = b''
        self.bad_crc = 0
        self.overruns = 0

    def feed(self, data):
        out = []
        self._buf += data
        while True:
            i = self._buf.find(b'\n')
            if i < 0:
                if len(self._buf) > self.MAX_LINE:
                    self._buf = b''
                    self.overruns += 1
                return out
            line, self._buf = self._buf[:i], self._buf[i + 1:]
            line = line.strip()
            if not line:
                continue
            tokens = line.split()
            if len(tokens) < 2:
                self.bad_crc += 1
                continue
            body = b' '.join(tokens[:-1])
            try:
                rx_crc = int(tokens[-1])
            except ValueError:
                self.bad_crc += 1
                continue
            if rx_crc != crc8(body):
                self.bad_crc += 1
                continue
            out.append([t.decode() for t in tokens[:-1]])
        return out
