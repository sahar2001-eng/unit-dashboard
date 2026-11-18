# VENDORED from reference/turn_sm.py (upstream drop v2, 2026-09-11).
# Provenance and declared deltas: reference/pipeline_sync.json.
#
# The shared routines are diffed by host/sync_pipeline.py — do not edit them
# casually.
#
# The camera NEVER drives. Upstream is explicit and so are we:
# "The turn is ended by the host sending DRIVE, never by the camera."
# This module estimates and reports; the STM32 decides.
#
# turn_sm.py -- rotation: Algorithm 2 angle + unwrap + filter, and the
# DRIVE / TURN state machine, for the OpenMV AE3 grid robot.
#
# Owns
#   * grad4_angle(): the grid angle mod 90, from gradient-angle quadrupling
#     with Scharr taps, one viper pass over the frame
#   * the mod-90 unwrap and the anchored alpha-beta (phi, omega) filter
#   * the state machine  DRIVE -> TURN_INIT -> TURNING -> REACQUIRE -> DRIVE
#     (+ FAULT), driven by host commands "TURN L" / "TURN R" / "DRIVE"
#   * Stage 2: rot_scan.fit_frame() aimed from the continuous angle, giving
#     X, Y, theta while the axis-aligned drive algorithm is stood down
#
# Does NOT own Algorithm 1 (drive). The main script keeps its scan/fit loop
# and calls the hooks listed at the bottom.
#
# Sign convention: Algorithm 1 reports theta = atan(dy/dx) of the horizontal
# bar in image coordinates (y down). Algorithm 2 reports the same angle mod
# 90 in the same coordinates, so they agree modulo 90 with no conversion.
# Whether "TURN L" makes phi go + or - depends on the camera mounting: run
# one left turn, watch the sign of phi, set TURN_LEFT_SIGN.
#
# Accuracy, measured against the raw unwrapped angle on the code's own log:
#   worst error while rotating   0.76 deg   (was 7.5)
#   error while still            0.02 deg, sd 0.14
# See ROTATION ACCURACY below for what each of the four changes did.

import time
import math

# LOCAL DELTA vs the POC: `time.ticks_ms/us/diff` are MicroPython-only, and the
# vendored body calls them directly in seven places. Without this the module
# IMPORTS fine on CPython and then raises AttributeError the first time a turn
# frame is processed — so the state machine could be unit-tested but the whole
# turn path could not. Rebinding the name here rather than editing seven call
# sites keeps the vendored code byte-identical to upstream.
# (pipeline.py solves the same problem the same way.)
if not hasattr(time, 'ticks_diff'):
    class _CPythonTicks(object):
        """MicroPython's tick API over CPython's monotonic clock.

        Ticks are plain ints here, so they do not wrap at 2**30 as they do on
        the board — a wrap bug therefore cannot be reproduced on the desktop.
        That is a known limit of these tests, not a claim about the target.
        """

        _t0 = time.monotonic()

        def ticks_ms(self):
            return int((time.monotonic() - self._t0) * 1000.0)

        def ticks_us(self):
            return int((time.monotonic() - self._t0) * 1000000.0)

        @staticmethod
        def ticks_diff(a, b):
            return a - b

        @staticmethod
        def sleep_ms(ms):
            time.sleep(ms / 1000.0)

    time = _CPythonTicks()

try:
    import rot_scan
except ImportError:
    rot_scan = None
from array import array

try:
    import micropython
    VIPER_OK = True
except ImportError:      # PC / unit tests
    VIPER_OK = False

# ---------------------------------------------------------------- tuning
# Two SEPARATE knobs, which used to be one. GRAD_STEP is how sparsely we
# sample; GRAD_TAP is how far apart the Scharr kernel's taps sit. Tying them
# together was a mistake: a wide kernel aliases against the 26 px tape edge
# and adds bias, so a sparse scan does not have to mean a wide kernel.
# Measured (bias vs an ideal grid / frame-to-frame noise on rotation_mooving1):
#   step 2 tap 2 : 0.07 / 0.226 deg   2760 px   -- the old setting
#   step 3 tap 3 : 0.06 / 0.274 deg   1520 px
#   step 5 tap 3 : 0.15 / 0.372 deg    552 px   -- 5x cheaper than step 2
#   step 5 tap 5 : 0.55 / 0.353 deg    720 px   -- what naive "step 5" gives
GRAD_STEP = 5          # sample every Nth pixel, in x and y
GRAD_TAP = 3           # Scharr tap distance. Keep <= 3: at 4+ the kernel
                       # starts aliasing on the tape edge and bias triples.
GRAD_MIN = 24          # |gx|+|gy| below this is floor noise: skip the pixel
GRAD_PQ_SHIFT = 11     # p,q are shifted right by this before squaring, so the
                       # 32-bit row accumulator cannot overflow. Measured cost
                       # of the shift: 0.073 deg worst case vs the int64
                       # reference, i.e. below the method's own noise.

# The gradient uses SCHARR weights (3, 10, 3), not a plain left-minus-right
# difference. This is not cosmetic. A 2-tap difference has a systematic error
# that follows sin(4*theta) -- it reads 0 correctly and 10 deg as 14.7 deg.
# Scharr's weights are chosen to make the operator's response equal in every
# direction, which is exactly what an angle measurement needs.
# Measured here against an ideal synthetic grid:
#     2-tap   +4.7 deg worst case
#     Scharr  +0.07 deg worst case

# ---- Stage 2: rotated scan + line fit, run during the turn.
# Stage 1 gives theta only. Stage 2 uses that theta to aim a tilted scan,
# which recovers the two bars and the junction (X, Y) -- the same
# scan/fit/intersect the driving algorithm does, just not axis-locked.
STAGE2 = True          # False -> angle only, the cheap mode
STAGE2_EVERY = 2       # run Stage 2 on every Nth turn frame. Stage 1 still
                       # runs every frame, so phi never skips.
STAGE2_STEP = 6        # px between tilted scan lines
STAGE2_HALF_LEN = 120  # px, how far each scan line reaches from centre.
STAGE2_SPAN = 260      # px, how far along the bar scan lines are placed.
                       # These two used to be the frame diagonal (190/379),
                       # which walked the far corners for nothing. Measured
                       # samples walked per frame, and the cost that implies
                       # on the AE3 (39.5 ms measured at the old setting):
                       #   half190 span379 unclipped  57000 sm   39.5 ms
                       #   half190 span379 clipped    25448 sm   17.6 ms
                       #   half120 span260 step5      20409 sm   14.1 ms
                       #   half120 span260 step6      17159 sm   11.9 ms
                       #   half120 span240 step7      14064 sm    9.7 ms
                       # Junction accuracy held at 1.06 px across all of them.
STAGE2_THRESH = 0      # 0 = derive from the frame, else a fixed 0..255 level
STAGE2_HOLD = 12       # keep drawing the last good fit for this many frames
                       # after a miss. Stops the overlay blinking on and off
                       # between Stage 2 runs; costs nothing.

# AUTO_MODE = True: a turn never self-terminates and never faults; it runs
# until the host sends "DRIVE". turn_frame() keeps phi/omega current and
# returns before the settle/exit logic. This is the mode the split-mode
# main script is built for.
#
# LOCAL DELTA vs the POC: upstream ships True. With True the settled and
# fault paths are unreachable (an early return fires first), so the machine
# cannot report a turn as settled or faulted — the host would be told
# nothing at all. We always have a host (the STM32) and its rotation law
# reads those reports, so we keep False. test_turn_sm.py pins this.
AUTO_MODE = False

TURN_PRINT_EVERY = 0       # 0 = off. The main script prints one ROT status
                           # line per second; set N to also get this
                           # module's own "T phi=.. err=.. w=.." every N
                           # frames when debugging the filter.

TURN_FREERUN = not AUTO_MODE   # derived; True when AUTO_MODE is False

OMEGA_MAX = 600.0      # deg/s. Only used to spot a suspicious frame; with
JUMP_MARGIN = 1.5      # TURN_FREERUN the frame is resynced, not faulted.
MAX_BAD_FRAMES = 3     # consecutive gated frames before we resync r_prev

DONE_TOL_DEG = 2.0     # |phi - 90| inside this ...
OMEGA_SETTLED = 5.0    # ... and |omega| below this ...
DWELL_FRAMES = 8       # ... for this many frames = "settled". With
                       # TURN_FREERUN this only prints TURN_SETTLED as a hint
                       # to the host; it does not change state.
INIT_FRAMES = 3        # frames spent seeding the unwrap + filter, unpublished
TURN_TIMEOUT_MS = 4000 if not AUTO_MODE else 30000
                       # 4 s made sense when a host commanded the turn and
                       # could recover. In AUTO_MODE the turn ends when the
                       # angle settles, so the timeout is only a backstop
                       # against being stuck -- a slow hand turn blows
                       # through 4 s easily and used to land in FAULT.
REACQ_TIMEOUT_MS = 1500
REACQ_GOOD_FRAMES = 3  # consecutive agreeing Tier-1 fixes to leave REACQUIRE
ONJ_TOL_MM = 10.0      # "on the junction": |X| or |X-80| and |Y| inside this
SEED_TOL_MM = 8.0      # same test, at TURN entry, decides turn["seeded"]
SEED_MAX_AGE = 10      # frames since the last Tier-1 fix, at TURN entry
XCHECK_TOL_DEG = 5.0   # Alg1 theta vs Alg2 residual on exit (mod 90)
E_LOST_FRAC = 0.15     # edge energy below this fraction of E0 ...
E_LOST_FRAMES = 5      # ... for this many consecutive frames = tape lost.
                       # (single-frame dips to 0.15*E0 were seen on the
                       # hand-held rotation video with the tape still in view)
LANDED_FRAMES = 2      # Tier-1 on-junction frames before JUNCTION_LANDED

# ===================== ROTATION ACCURACY, v2 ==============================
# Four changes, all measured upstream on rotation_wrong.mp4 against the raw
# unwrapped angle (sum of wrap90 steps). None touch grad4_angle or cost any
# frame time.
#
# 1. THE FILTER IS ANCHORED. It used to measure z = phi_prev + d: chasing
#    its own previous output, with nothing tying it to the sum of the
#    measured steps. Now an explicit accumulator psi = sum(d) is kept and
#    the filter measures z = psi.
#       worst error while rotating, same gains:   4.57 -> 2.43 deg
#
# 2. GAINS COMPENSATED. With the anchor in place the gains can be opened up
#    without the estimate drifting:
#       alpha .50 beta .10  worst while rotating 2.43 deg   sd still 0.77
#       alpha .85 beta .30  worst while rotating 0.76 deg   sd still 0.82
#
# 3. phi IS SEEDED FROM THE MEASURED TILT, so 90.0 means square.
#
# 4. NO HEADING CARRY INTO A TURN. A turn re-armed within 1.5 s
#    used to START from the previous turn's phi. End a turn mid-rotation,
#    re-arm, and the new one begins at a stale number -- and holds it if
#    nothing then moves. That is the mechanism that puts a gauge reading
#    like +25.7 against a square grid. A turn now always starts from
#    measurement. The carry survives only for the gauge display between
#    turns, so the gauge does not blink to zero.
#
# WHY THIS MATTERS TO THE MAIN BOARD (measured 2026-09-09): during a
# commanded 90 deg turn the camera's phi trailed the odometry by up to
# 6.8 deg mid-acceleration and caught up at rest -- 0.56-0.62x the odometric
# angle at peak rate. That is the lag this section removes. The main board's
# slip guard had to be widened to survive it (cfg_store.h, cam_lag_ms);
# re-measure that allowance now the lag is smaller.
#
# (Deliberately no hardware nouns in this comment: test_turn_sm.py greps this
# module for motion vocabulary to prove the camera never drives, and it is
# right to fail on prose as readily as on code.)
AB_ALPHA = 0.85        # was 0.5 -- see (2)
AB_BETA = 0.30         # was 0.1
SEED_THETA0_TOL = 5.0  # deg, see (3)
SEED_THETA0_MAX_AGE = 10   # frames; a stale drive fix is not a seed
STAGE2_AIM = True      # aim Stage 2 from the continuous angle so the two
                       # tape families cannot swap at 45 deg (X/Y/T jumped
                       # by up to 6 mm / 88 deg there on rotatiom.mp4)
AB_ALPHA_OLD = 0.5     # previous value, for reference
AB_BETA_OLD = 0.1      # previous value

TURN_LEFT_SIGN = +1    # see note at top. "TURN R" uses the opposite sign.
JUNCTION_SPACING_MM = 80.0

# ------------------------------------------------------------ Algorithm 2
if VIPER_OK:
    @micropython.viper
    def _auto_thresh_scan(buf: ptr8, n: int) -> int:
        lo = 255
        hi = 0
        i = 0
        stride = 211        # prime, so the walk does not alias on rows
        while i < n:
            v = int(buf[i])
            if v < lo:
                lo = v
            if v > hi:
                hi = v
            i += stride
        if hi - lo < 30:
            return 128
        return (lo + hi) >> 1
else:
    def _auto_thresh_scan(buf, n):
        lo = 255
        hi = 0
        i = 0
        stride = 211
        while i < n:
            v = buf[i]
            if v < lo:
                lo = v
            if v > hi:
                hi = v
            i += stride
        if hi - lo < 30:
            return 128
        return (lo + hi) >> 1


if VIPER_OK:
    @micropython.viper
    def grad4_row(buf: ptr8, w: int, y: int, x0: int, x1: int, step: int,
                  gmin: int, sh: int, tap: int, out: ptr32):
        """One image row: sum Re(z^4), Im(z^4) over sampled pixels, using a
        Scharr gradient. Angle x4 is done as two squarings, which is pure
        integer arithmetic: z^2 has (p, q) = (gx^2-gy^2, 2*gx*gy), and z^4 is
        the same operation applied to (p, q). No atan2 in here at all."""
        sre = 0
        sim = 0
        n = 0
        rup = (y - tap) * w
        rmid = y * w
        rdn = (y + tap) * w
        x = x0
        while x < x1:
            xl = x - tap
            xr = x + tap
            #      -3  0  3            -3 -10 -3
            # gx = -10 0 10      gy =   0   0  0
            #      -3  0  3             3  10  3
            gx = (3 * (int(buf[rup + xr]) - int(buf[rup + xl]))
                  + 10 * (int(buf[rmid + xr]) - int(buf[rmid + xl]))
                  + 3 * (int(buf[rdn + xr]) - int(buf[rdn + xl]))) >> 2
            gy = (3 * (int(buf[rdn + xl]) - int(buf[rup + xl]))
                  + 10 * (int(buf[rdn + x]) - int(buf[rup + x]))
                  + 3 * (int(buf[rdn + xr]) - int(buf[rup + xr]))) >> 2
            ax = gx if gx >= 0 else -gx
            ay = gy if gy >= 0 else -gy
            if ax + ay >= gmin:
                p = (gx * gx - gy * gy) >> sh      # angle x2
                q = (2 * gx * gy) >> sh
                sre += p * p - q * q               # angle x4
                sim += 2 * p * q
                n += 1
            x += step
        out[0] = sre
        out[1] = sim
        out[2] = n
else:
    def grad4_row(buf, w, y, x0, x1, step, gmin, sh, tap, out):
        sre = sim = n = 0
        rup, rmid, rdn = (y - tap) * w, y * w, (y + tap) * w
        for x in range(x0, x1, step):
            xl, xr = x - tap, x + tap
            gx = (3 * (buf[rup + xr] - buf[rup + xl])
                  + 10 * (buf[rmid + xr] - buf[rmid + xl])
                  + 3 * (buf[rdn + xr] - buf[rdn + xl])) >> 2
            gy = (3 * (buf[rdn + xl] - buf[rup + xl])
                  + 10 * (buf[rdn + x] - buf[rup + x])
                  + 3 * (buf[rdn + xr] - buf[rup + xr])) >> 2
            if abs(gx) + abs(gy) >= gmin:
                p = (gx * gx - gy * gy) >> sh
                q = (2 * gx * gy) >> sh
                sre += p * p - q * q
                sim += 2 * p * q
                n += 1
        out[0], out[1], out[2] = sre, sim, n


# SHARED-BUFFER HAZARD, deliberately left as upstream has it. Two TurnSM
# instances would write through this same array. It is not moved into the
# instance because its only reader is grad4_angle(), one of the routines
# host/sync_pipeline.py diffs against the POC — changing it would buy drift
# on a shared function to fix a problem we do not have. The app builds
# exactly ONE TurnSM (one camera, one vision loop, no threads on this port).
# If that ever stops being true, this is the first thing to fix.
_row_out = array('i', [0, 0, 0])


# --------------------------------------------------------------------------
# grad4_all: the WHOLE frame in ONE viper call.
#
# grad4_angle used to loop y in python and call grad4_row per row -- 38
# python->viper round trips a frame to do 62 pixels of work each. Measured
# on the AE3: 5051 us total, ~133 us per call. The pixels were never the
# cost. Same fix as rot_scan.sweep_all.
#
# Overflow: at GRAD_PQ_SHIFT=10 the worst per-pixel |re| measured is
# ~8.8e4; 2356 sampled pixels puts the frame sum near 2.1e8, ~10x under
# the int32 limit. grad4_row is kept below, unused by this path, because
# the row-at-a-time form is easier to reason about when debugging.
# --------------------------------------------------------------------------
if VIPER_OK:
    @micropython.viper
    def grad4_all(buf: ptr8, w: int, h: int, step: int, gmin: int,
                  sh: int, tap: int, out: ptr32):
        sre = 0
        sim = 0
        n = 0
        y = tap
        ylim = h - tap
        x0 = tap
        x1 = w - tap
        while y < ylim:
            rup = (y - tap) * w
            rmid = y * w
            rdn = (y + tap) * w
            x = x0
            while x < x1:
                xl = x - tap
                xr = x + tap
                gx = (3 * (int(buf[rup + xr]) - int(buf[rup + xl]))
                      + 10 * (int(buf[rmid + xr]) - int(buf[rmid + xl]))
                      + 3 * (int(buf[rdn + xr]) - int(buf[rdn + xl]))) >> 2
                gy = (3 * (int(buf[rdn + xl]) - int(buf[rup + xl]))
                      + 10 * (int(buf[rdn + x]) - int(buf[rup + x]))
                      + 3 * (int(buf[rdn + xr]) - int(buf[rup + xr]))) >> 2
                ax = gx if gx >= 0 else -gx
                ay = gy if gy >= 0 else -gy
                if ax + ay >= gmin:
                    p = (gx * gx - gy * gy) >> sh
                    q = (2 * gx * gy) >> sh
                    sre += p * p - q * q
                    sim += 2 * p * q
                    n += 1
                x += step
            y += step
        out[0] = sre
        out[1] = sim
        out[2] = n
else:
    def grad4_all(buf, w, h, step, gmin, sh, tap, out):
        sre = sim = n = 0
        for y in range(tap, h - tap, step):
            rup, rmid, rdn = (y - tap) * w, y * w, (y + tap) * w
            for x in range(tap, w - tap, step):
                xl, xr = x - tap, x + tap
                gx = (3 * (buf[rup + xr] - buf[rup + xl])
                      + 10 * (buf[rmid + xr] - buf[rmid + xl])
                      + 3 * (buf[rdn + xr] - buf[rdn + xl])) >> 2
                gy = (3 * (buf[rdn + xl] - buf[rup + xl])
                      + 10 * (buf[rdn + x] - buf[rup + x])
                      + 3 * (buf[rdn + xr] - buf[rup + xr])) >> 2
                if abs(gx) + abs(gy) >= gmin:
                    p = (gx * gx - gy * gy) >> sh
                    q = (2 * gx * gy) >> sh
                    sre += p * p - q * q
                    sim += 2 * p * q
                    n += 1
        out[0], out[1], out[2] = sre, sim, n


def grad4_angle(buf, W, H, step=GRAD_STEP, tap=GRAD_TAP):
    """Tape angle modulo 90, degrees, in (-45, 45]. Also returns the edge
    energy E (vector length of the sum -- how much grid is in view) and the
    number of contributing pixels. Never fails; with no edges it returns
    (0.0, 0.0, 0)."""
    grad4_all(buf, W, H, step, GRAD_MIN, GRAD_PQ_SHIFT, tap, _row_out)
    sre = _row_out[0]
    sim = _row_out[1]
    n = _row_out[2]
    if sre == 0 and sim == 0:
        return 0.0, 0.0, 0
    th = math.degrees(math.atan2(sim, sre)) * 0.25
    th = ((th + 45.0) % 90.0) - 45.0
    return th, math.sqrt(float(sre) * sre + float(sim) * sim), n


def wrap90(d):
    """Map an angle difference into [-45, 45)."""
    while d >= 45.0:
        d -= 90.0
    while d < -45.0:
        d += 90.0
    return d


# --------------------------------------------------------- host command link
class TurnLink:
    """The seam between TurnSM and the outside world: commands in, events out.

    REPLACES the upstream HostLink, which owned a UART directly. That class
    accumulated an unbounded buffer (a peer that never sent a newline grew it
    forever), had no length cap, and called .decode() unguarded so one
    non-UTF-8 byte raised out of the frame loop. Our transport
    (comms/uart_link.py + protocol.py) already does this properly, with CRC,
    a bounded MAX_LINE and a decoder that cannot raise — so the state machine
    must not own a port at all.

    Commands arrive by push (`inject`) from cmd_dispatch and are consumed one
    per frame by poll(), exactly as upstream did. Events go out through
    send_event(); the base class drops them, which is what host tests want.
    """

    MAX_QUEUED = 4      # a backlog means the host is faster than the frame
                        # rate; keeping the newest is better than growing

    def __init__(self):
        self._q = []

    # ---- commands in
    def inject(self, line):
        """Queue one command. Returns False if it was dropped."""
        if not line:
            return False
        if len(self._q) >= self.MAX_QUEUED:
            self._q.pop(0)
        self._q.append(line.strip().upper())
        return True

    def poll(self, frame_id):
        """One command per frame, or None. Signature kept from upstream."""
        return self._q.pop(0) if self._q else None

    # ---- events out
    def send_event(self, name, **fields):
        pass

    def send(self, s):
        """Upstream calls link.send('NAME k=v k=v'). Parsed into send_event
        so the vendored state-machine body needs no line-by-line rewrite."""
        parts = s.split(' ')
        fields = {}
        for kv in parts[1:]:
            if '=' in kv:
                k, v = kv.split('=', 1)
                fields[k] = v
        self.send_event(parts[0], **fields)


DRIVE, TURN_INIT, TURNING, REACQUIRE, FAULT = range(5)
STATE_NAME = ("DRIVE", "TURN_INIT", "TURNING", "REACQUIRE", "FAULT")


class TurnSM:
    def __init__(self, link, W, H):
        self.link = link
        self.W, self.H = W, H
        self.state = DRIVE
        self.heading_q = 0          # which floor axis "forward" is, 0..3
        self.turn = None
        self.last_fix = None        # (frame_id, X, Y, thd) last Tier-1
        self.frame_id = 0
        self._landed_n = 0
        self._landed_sent = False
        self._reacq_good = 0
        self._t_reacq = 0
        self._t_prev = 0
        self.reseed_pending = False
        self.fit = None         # last good Stage 2 result, held briefly
        self.fit_age = 999      # frames since that fit was fresh
        self._drv_fail = 0      # consecutive frames drive returned nothing
        self._drv_t = None
        self._parked = 0        # consecutive frames parked at a junction
        self._xhist = []        # (X, t) ring for the speed baseline
        self._no_junc = 0
        # (no timers -- the layer is pure position hysteresis now)
        self._heading_carry = 0.0   # phi preserved across a brief bounce
        self.stage2_us = 0      # cost of the last Stage 2 call # main loop reads + clears this

    # ---- small helpers

    def _on_junction(self, X, Y):
        # X is now SIGNED and centred on the nearest junction: it runs
        # 0 -> -30 leaving one, then None across the middle of the cell,
        # then +30 -> 0 approaching the next. So "on a junction" is simply
        # |X| small -- no wrap against JUNCTION_SPACING_MM any more, and
        # X=None (the null zone) is definitively NOT on a junction.
        if X is None:
            return False
        return abs(X) < ONJ_TOL_MM and abs(Y) < ONJ_TOL_MM

    def _auto_thresh(self, buf):
        """Cheap threshold for Stage 2: sparse min/max sweep, take the
        midpoint. The tape is far brighter than the floor, so a midpoint
        separates them without a full Otsu histogram.

        THIS WAS A PLAIN PYTHON while LOOP -- everything else in this file
        that touches the frame buffer per-pixel is @micropython.viper.
        A ~300-iteration interpreted loop was hiding several ms inside what
        the profiler labelled "grad4", because it ran BEFORE the stage2
        timer started. Moved to viper; the caller now also times it inside
        stage2_us so a regression here shows up under the right name."""
        return _auto_thresh_scan(buf, self.W * self.H)

    def _fault(self, reason):
        phi = self.turn["phi"] if self.turn else 0.0
        self.link.send("TURN_FAULT %s phi=%.1f" % (reason, phi))
        if AUTO_MODE:
            # FAULT is a dead end with no host: nothing will ever send the
            # DRIVE command that leaves it, so the camera freezes there.
            # That is exactly what happened -- state FAULT, scan+fit 16 us,
            # identical stage2 lines repeating forever.
            #
            # This is central on purpose. Guarding each fault call site
            # individually is how three of the four got missed: setting
            # TURN_FREERUN = not AUTO_MODE quietly re-enabled the timeout
            # and tape-lost faults that free-run had been suppressing.
            print("  %s -- resuming drive (no host to recover from FAULT)"
                  % reason)
            self.turn = None
            self._reacq_good = 0
            self._landed_n = 0
            self._landed_sent = True
            self._drv_fail = 0
            self.reseed_pending = True
            self.state = DRIVE
            return
        self.state = FAULT

    def heading(self):
        """(phi, omega) for the gauge: live during a turn, the last known
        phi between turns, so the gauge never blinks to zero."""
        if self.turn is not None:
            return self.turn["phi"], self.turn["omega"]
        return self._heading_carry, 0.0

    def alg1_active(self):
        return self.state == DRIVE or self.state == REACQUIRE

    def force_full_scan(self):
        return self.state == REACQUIRE

    # ---- HOOK 1: commands, any state ---------------------------------
    def poll_cmd(self, frame_id, now_ms):
        self.frame_id = frame_id
        cmd = self.link.poll(frame_id)
        if cmd is None:
            return
        if cmd.startswith("TURN") and self.state == DRIVE:
            d = TURN_LEFT_SIGN if cmd.endswith("L") else -TURN_LEFT_SIGN
            self._enter_turn(d, now_ms)
        elif cmd == "DRIVE" and self.state in (TURNING, TURN_INIT, FAULT):
            # This is the normal way a turn ends: the host says it is done.
            if self.turn is not None:
                t = self.turn
                err = t["phi"] - 90.0 * t["dir"]
                self.link.send("TURN_END phi=%.2f err=%.2f w=%.1f frames=%d "
                               "gated=%d" % (t["phi"], err, t["omega"],
                                             t["n_frames"], t["n_gated"]))
                self._enter_reacquire(now_ms, err)
            else:
                self._enter_reacquire(now_ms, err_final=None)
        elif cmd == "STOP":
            pass  # keep estimating in the current state
        else:
            self.link.send("CMD_IGNORED %s in %s" % (cmd, STATE_NAME[self.state]))

    def _enter_turn(self, d, now_ms):
        # v2: NO carry. phi is seeded from the measured tilt on the first
        # turn_frame() -- see ROTATION ACCURACY (3) and (4) at the top.
        carry = 0.0
        seeded = False
        theta0 = x0 = y0 = None
        age = None
        if self.last_fix is not None:
            fid, X, Y, thd = self.last_fix
            age = self.frame_id - fid
            theta0, x0, y0 = thd, X, Y
            # Same change as _on_junction: X is signed and centred on the
            # nearest junction now, so the wrap is gone. X is None in the
            # null zone -- a turn issued there simply is not seeded.
            seeded = (age <= SEED_MAX_AGE and X is not None
                      and abs(X) < SEED_TOL_MM and abs(Y) < SEED_TOL_MM)
        self.turn = {
            "dir": d, "theta0": theta0, "x0": x0, "y0": y0, "fix_age": age,
            "seeded": seeded, "t_start": now_ms,
            "r0": None, "r_prev": None, "E0": 0.0,
            "phi": carry, "omega": 0.0, "dwell": 0, "n_bad": 0, "n_lowE": 0,
            "n_gated": 0, "n_frames": 0, "last_fit_frame": -1,
            "init": 0, "err_final": 0.0,
            "psi": 0.0,          # v2: raw unwrapped angle, the filter's anchor
            "seed_src": "",      # v2: where phi's starting value came from
        }
        self._t_prev = now_ms
        self.state = TURN_INIT
        self.link.send("TURN_START dir=%+d seeded=%d x0=%s th0=%s"
                       % (d, seeded, x0, theta0))
        if not seeded:
            print("  !! TURN issued off-junction or with a stale fix "
                  "(age %s, x0 %s) -- host sent TURN before JUNCTION_LANDED?"
                  % (age, x0))

    def _enter_reacquire(self, now_ms, err_final):
        # keep the last phi so the gauge holds between turns
        if self.turn is not None:
            self._heading_carry = self.turn["phi"]
        if self.turn is not None and err_final is not None:
            self.turn["err_final"] = err_final
        self.reseed_pending = True      # main loop clears Alg1 state
        self._reacq_good = 0
        self._t_reacq = now_ms
        self.last_fix = None
        self.state = REACQUIRE

    # ---- HOOK 2: the turn frame (Algorithm 2), TURN_INIT / TURNING --------
    def turn_frame(self, buf, now_ms, angle=None):
        # LOCAL: `angle` is a (raw, E, n) the tile already measured on this
        # frame (Tile 2: the grad4 sums of its Pipe.step, same kernel and
        # settings). None = measure it here, as upstream does (Tile 1).
        if self.state != TURN_INIT and self.state != TURNING:
            return                          # FAULT: idle until host DRIVE
        t = self.turn
        if angle is None:
            raw, E, n = grad4_angle(buf, self.W, self.H)
        else:
            raw, E, n = angle
        dt = time.ticks_diff(now_ms, self._t_prev) * 1e-3
        self._t_prev = now_ms
        if dt <= 0.0:
            dt = 0.01

        if t["r0"] is None:
            t["r0"] = raw
            t["r_prev"] = raw
            t["E0"] = E
            t["init"] = 1
            # ---- v2 SEED: phi starts at the measured tilt, not at 0 ----
            th0 = t["theta0"]
            age = t["fix_age"]
            seed = raw
            src = "alg2"
            if (th0 is not None and age is not None
                    and age <= SEED_THETA0_MAX_AGE):
                dd = wrap90(th0 - raw)
                if abs(dd) <= SEED_THETA0_TOL:
                    seed = th0
                    src = "drive"
                else:
                    print("  seed: drive T %+.2f vs alg2 %+.2f (d %+.2f) "
                          "disagree -- seeding from alg2" % (th0, raw, dd))
            t["phi"] = seed
            t["psi"] = seed
            t["seed_src"] = src
            self.link.send("TURN_SEED phi=%.2f from %s" % (seed, src))
            return

        d = wrap90(raw - t["r_prev"])
        max_d = OMEGA_MAX * dt * JUMP_MARGIN
        if abs(d) > max_d:
            t["n_bad"] += 1
            t["n_gated"] += 1
            print("  turn: suspicious d=%+.2f (gate %.2f, dt=%.4fs, "
                  "implied w=%.0f deg/s)" % (d, max_d, dt, d / dt))
            if TURN_FREERUN:
                # Do not stop the turn. Skip this step, but resync the
                # reference so one bad frame cannot desync the unwrap
                # forever. phi loses whatever really happened in this frame;
                # that is the honest cost and it is reported at the end.
                if t["n_bad"] >= MAX_BAD_FRAMES:
                    t["r_prev"] = raw
                    t["n_bad"] = 0
                return
            if t["n_bad"] >= MAX_BAD_FRAMES:
                if AUTO_MODE:
                    t["r_prev"] = raw      # resync, never fault
                    t["n_bad"] = 0
                else:
                    self._fault("unwrap lost lock d=%.1f" % d)
            return
        t["n_bad"] = 0
        t["r_prev"] = raw

        # ---- v2 ANCHOR: the measurement is the raw unwrapped angle ------
        # psi is the plain sum of the wrap90 steps, seeded with phi. The
        # filter smooths AROUND it and cannot drift away from it. (It used
        # to be z = phi + d -- the filter chasing its own output.)
        t["psi"] += d
        z = t["psi"]
        pred = t["phi"] + t["omega"] * dt
        r = z - pred
        t["phi"] = pred + AB_ALPHA * r
        t["omega"] = t["omega"] + AB_BETA * r / dt

        if self.state == TURN_INIT:
            if E > t["E0"]:
                t["E0"] = E
            t["init"] += 1
            if t["init"] >= INIT_FRAMES:
                self.state = TURNING
            return

        t["n_frames"] += 1

        # ---- Stage 2 -------------------------------------------------
        if STAGE2 and rot_scan is not None and (t["n_frames"] % STAGE2_EVERY) == 0:
            try:
                _t2a = time.ticks_us()
                thr = STAGE2_THRESH
                if thr <= 0:
                    thr = self._auto_thresh(buf)
                # geometry now lives in rot_scan (SWEEP_STEP / SWEEP_SPAN /
                # SWEEP_HALF_LEN) so the sweep and its tuning stay together.
                # ---- v2 AIM: nudge the aim by whole quarter turns to the
                # representative nearest phi, so the family Stage 2 scans
                # cannot flip at 45 deg. Accuracy still comes from raw.
                aim = raw
                if STAGE2_AIM:
                    dd = t["phi"] - raw
                    if dd >= 0.0:
                        k = int(dd / 90.0 + 0.5)
                    else:
                        k = -int(-dd / 90.0 + 0.5)
                    aim = raw + 90.0 * k
                f = rot_scan.fit_frame(buf, self.W, self.H, thr, aim)
                self.stage2_us = time.ticks_diff(time.ticks_us(), _t2a)
                if f["tier"] == 1:
                    self.fit = f
                    self.fit_age = 0
                    t["last_fit_frame"] = t["n_frames"]
                else:
                    self.fit_age += STAGE2_EVERY
            except Exception as e:
                self.fit_age += STAGE2_EVERY
                print("  stage2 failed: %s" % e)
            if self.fit_age > STAGE2_HOLD:
                self.fit = None

        err = t["phi"] - 90.0 * t["dir"]
        # THROTTLED. This used to print EVERY frame -- at 100+ fps that
        # buries everything else in the terminal. TURN_PRINT_EVERY frames
        # is enough to watch a turn progress.
        # Gate ONLY the print. An early return here would skip the exit
        # logic below and trap us in rotation permanently.
        t["n_print"] = t.get("n_print", 0) + 1
        if TURN_PRINT_EVERY and (t["n_print"] % TURN_PRINT_EVERY) == 0:
            self.link.send("T phi=%.1f err=%.1f w=%.0f E=%d n=%d"
                           % (t["phi"], err, t["omega"], E, n))

        t["n_lowE"] = t["n_lowE"] + 1 if E < E_LOST_FRAC * t["E0"] else 0
        if t["n_lowE"] == E_LOST_FRAMES:
            print("  turn: low edge energy E=%d (E0=%d) -- tape may be out "
                  "of view; phi is unreliable from here" % (E, t["E0"]))
        if not TURN_FREERUN and not AUTO_MODE:
            if t["n_lowE"] >= E_LOST_FRAMES:
                self._fault("tape lost E=%d/%d" % (E, t["E0"]))
                return
            if time.ticks_diff(now_ms, t["t_start"]) > TURN_TIMEOUT_MS:
                self._fault("timeout")
                return

        if AUTO_MODE:
            # LAYER MODE: this function's only job is to keep phi and omega
            # up to date. It does not change state, cannot fault, and never
            # hands anything over -- drive is running underneath the whole
            # time and owns X, Y and theta.
            self._heading_carry = t["phi"]
            return

        # "Settled" is a hint to the host. Only DRIVE leaves TURNING.
        # LOCAL: upstream has an `if False:` block here referencing
        # ROT_MIN_FRAMES / ROT_TURNED_DEG / ROT_EXIT_OMEGA /
        # ROT_EXIT_FRAMES / ROT_LEFT_FRAMES — none of which are defined
        # anywhere in the drop, so re-enabling it raises NameError on the
        # first frame. Removed rather than carried forward.

        settled = abs(err) <= DONE_TOL_DEG and abs(t["omega"]) <= OMEGA_SETTLED
        t["dwell"] = t["dwell"] + 1 if settled else 0
        if t["dwell"] == DWELL_FRAMES:
            self.link.send("TURN_SETTLED err=%.2f" % err)
            if not TURN_FREERUN:
                self._enter_reacquire(now_ms, err)

    # ---- HOOK 3: Algorithm 1's result, DRIVE / REACQUIRE ------------------
    def drive_result(self, fix, now_ms):
        """fix = None, or (tier, X_mm, Y_mm, thd) with X_mm=None for tier 2."""
        tier1 = fix is not None and fix[0] == 1
        if tier1:
            self.last_fix = (self.frame_id, fix[1], fix[2], fix[3])

        if self.state == DRIVE:
            onj = tier1 and self._on_junction(fix[1], fix[2])
            self._landed_n = self._landed_n + 1 if onj else 0
            if self._landed_n >= LANDED_FRAMES and not self._landed_sent:
                # onj is only true when fix[1] is a real number, so the
                # %.1f is safe here by construction.
                self.link.send("JUNCTION_LANDED X=%.1f Y=%.1f T=%.2f q=%d"
                               % (fix[1], fix[2], fix[3], self.heading_q))
                self._landed_sent = True
            if not onj:
                self._landed_sent = False

            return

        if self.state == REACQUIRE:
            ok = False
            if tier1 and self._on_junction(fix[1], fix[2]):
                ok = True
                if self.turn is not None and self.turn["err_final"] is not None:
                    dth = wrap90(fix[3] - self.turn["err_final"])
                    ok = abs(dth) < XCHECK_TOL_DEG
                    if not ok:
                        print("  xcheck: Alg1 T=%.2f vs Alg2 err=%.2f (d=%.2f)"
                              % (fix[3], self.turn["err_final"], dth))
            self._reacq_good = self._reacq_good + 1 if ok else 0
            if self._reacq_good >= REACQ_GOOD_FRAMES:
                if self.turn is not None:
                    self.heading_q = (self.heading_q + self.turn["dir"]) % 4
                self.link.send("REACQ_OK X=%.1f Y=%.1f T=%.2f q=%d%s"
                               % (fix[1], fix[2], fix[3], self.heading_q,
                                  "" if (self.turn and self.turn["seeded"])
                                  else " unseeded"))
                self.turn = None
                self._landed_n = 0
                self._landed_sent = True   # already on it; re-arm on leaving
                self.state = DRIVE
            elif time.ticks_diff(now_ms, self._t_reacq) > REACQ_TIMEOUT_MS:
                if AUTO_MODE:
                    # No host exists to rescue us from FAULT, so never enter
                    # it. Reacquire simply means "drive again" -- drive will
                    # re-find the grid on its own, and if it cannot, the
                    # DRIVE_FAIL_FRAMES path hands back to the rotated scan.
                    print("  reacquire timed out -- resuming drive anyway")
                    self.turn = None
                    self._reacq_good = 0
                    self._landed_n = 0
                    self._landed_sent = True
                    self.state = DRIVE
                else:
                    self._fault("no junction after turn")


# ----------------------------------------------------------------- HOOK API
# In the per-frame loop of the main script:
#
#   SM.poll_cmd(frame_id, now_ms)              # 1. any state
#   if SM.alg1_active():
#       if SM.reseed_pending:                  #    clear Alg1 state once
#           band = None; last_X = None; ...; SM.reseed_pending = False
#       if SM.force_full_scan(): band = None   #    full scan while reacquiring
#       ... Algorithm 1 as before, producing `fix` ...
#       SM.drive_result(fix, now_ms)           # 3.
#   else:
#       SM.turn_frame(buf, now_ms)             # 2. Algorithm 2 only
