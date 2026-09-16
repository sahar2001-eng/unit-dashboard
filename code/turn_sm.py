# turn_sm.py -- DRIVE / TURN state machine for the OpenMV AE3 grid robot.
#
# Owns:
#   * Algorithm 2: gradient-orientation angle with angle quadrupling (viper)
#   * the mod-90 unwrap and the alpha-beta (theta, omega) filter
#   * the state machine  DRIVE -> TURN_INIT -> TURNING -> REACQUIRE -> DRIVE
#     (+ FAULT)
#   * the host command link (UART, or simulated for bench tests)
#
# It does NOT own Algorithm 1. The main script (v1_2.py) keeps its scan/fit
# loop and calls three hooks per frame -- see "HOOK API" at the bottom.
#
# Sign convention. Algorithm 1 reports theta = atan(dy/dx) of the horizontal
# bar in image coordinates (y down). Algorithm 2 reports the same angle mod
# 90, in the same coordinates, so the two agree modulo 90 with no conversion.
# Whether "TURN L" makes phi go + or - depends on how the camera is mounted:
# run one left turn, watch the sign of phi in the T lines, and set
# TURN_LEFT_SIGN accordingly.

import time
import math

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

# The turn is ended by the host sending DRIVE, never by the camera.
# ---------------------------- AUTOMATIC MODE SWITCHING -------------------
# There is no host issuing TURN/DRIVE, so the camera decides for itself
# which algorithm to run. Both algorithms are UNCHANGED -- this only picks
# between them.
#
# The trigger is ANGLE, not position. Proximity to a junction looks like
# the natural switch but is not: with X_RANGE_MM=30 and 80 mm spacing, X is
# non-null for 60 of every 80 mm, so "near a junction" would be true 75% of
# the time and we would pay rotation's cost almost always.
#
# Drive's ONLY failure mode is the 45 deg blind spot, and it only gets near
# 45 deg when the robot is actually turning. So: leave drive when the angle
# or the turn rate says we are turning, come back when it settles.
# Hysteresis on both edges so it cannot flap at the boundary.
AUTO_MODE = True
# ROTATION AS AN ADDITIVE LAYER, NOT A SEPARATE MODE.
#
# Algorithm 1 (drive) runs EVERY frame, always. It is accurate and it owns
# X, Y and theta -- that never changes and never hands over. When we are
# near a junction and slow, Algorithm 2 is switched on ON TOP to track
# rotation, and switched off again after. Nothing is replaced, so nothing
# flickers: the numbers and the overlay come from the same place all the
# time.
#
# The previous design swapped one algorithm for the other, which meant the
# X/Y/theta source changed mid-run, the HUD changed with it, and the turn
# count restarted on every bounce. That was the flicker.
# ARM WHEN CLOSE TO A JUNCTION, DISARM WHEN WE LEAVE. That is all.
# No dwell timer, no speed estimate -- both were tried and both were worse:
# the speed estimate differentiated a 1.8mm-noisy X into 324 mm/s of
# garbage, and the dwell timer armed on any crawl under 120 mm/s anyway.
# Position with hysteresis is deterministic and has no failure mode.
LAYER_NEAR_MM = 22.0       # |X| under this -> arm the rotation layer
LAYER_FAR_MM = 38.0        # |X| over this (or X null) -> disarm.
                           # The 16mm gap is the hysteresis. X carries about
                           # 1.8mm rms of noise, so a narrow gap chatters:
                           # measured parked at the midpoint of a 25/32
                           # window, the layer flipped 9 times in 400
                           # frames. At 22/38 it does not flip at all.

TURN_PRINT_EVERY = 15      # print the per-frame rotation line only this
                           # often -- every frame floods the terminal at
                           # 100+ fps.
HEADING_CARRY_MS = 1500    # rotation re-armed within this long continues the
                           # turn count instead of restarting at zero.

TURN_FREERUN = not AUTO_MODE   # in AUTO_MODE the turn MUST self-terminate;
                               # free-run only makes sense with a host    # True: TURNING never self-terminates and never faults.
                       #       Every anomaly is a printed warning only.

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

AB_ALPHA = 0.5         # alpha-beta gains. alpha high: Alg2 is low-noise
AB_BETA = 0.1          # (0.15 deg) so trust the measurement.

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
class HostLink:
    """Line protocol, ASCII, newline-terminated:
         host -> camera : TURN L | TURN R | DRIVE | STOP
         camera -> host : one status line per frame + events (see send()).
    uart=None means bench mode: commands come from inject() or from the
    sim_cmds list [(frame_id, "TURN L"), ...]."""

    def __init__(self, uart=None, sim_cmds=None, echo=True):
        self.uart = uart
        self.sim = list(sim_cmds or [])
        self.echo = echo
        self._buf = b""
        self._q = []

    def inject(self, line):
        self._q.append(line.strip().upper())

    def poll(self, frame_id):
        while self.sim and self.sim[0][0] <= frame_id:
            self._q.append(self.sim.pop(0)[1].strip().upper())
        if self.uart is not None and self.uart.any():
            self._buf += self.uart.read()
            while b"\n" in self._buf:
                line, self._buf = self._buf.split(b"\n", 1)
                s = line.strip().decode().upper()
                if s:
                    self._q.append(s)
        if self._q:
            return self._q.pop(0)
        return None

    def send(self, s):
        if self.uart is not None:
            self.uart.write(s + "\n")
        if self.echo:
            print(s)


# --------------------------------------------------------- state machine
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
        self._layer = False     # is the rotation layer switched on?
        # (no timers -- the layer is pure position hysteresis now)
        self._layer_start = None
        self._heading_carry = 0.0   # phi preserved across a brief bounce
        self._rot_left_ms = None    # when rotation last ended
        self.stage2_us = 0      # cost of the last Stage 2 call # main loop reads + clears this

    # ---- small helpers
    # _auto_enter() DELETED. It was the old mode-switching entry point and
    # it was still calling _enter_turn(), which sets state to TURN_INIT and
    # makes the main loop draw the rotation HUD instead of drive's -- so
    # X and Y vanished from the overlay and the display looked like it had
    # switched algorithms. layer_update() replaces it entirely: it never
    # touches self.state.

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
        """Accumulated rotation, always available. Returns the live value
        while the rotation layer is tracking, and the last known value
        otherwise -- so the gauge can be drawn every frame without it
        blinking in and out as the layer arms and disarms."""
        if self.turn is not None:
            return self.turn["phi"], self.turn["omega"]
        return self._heading_carry, 0.0

    def layer_active(self):
        """Is the rotation layer switched on? Drive runs regardless."""
        return self._layer

    def layer_update(self, fix, now_ms):
        """Arm when |X| is inside LAYER_NEAR_MM, stay armed until it leaves
        LAYER_FAR_MM. Nothing else -- no timer, no speed."""
        X = fix[1] if fix is not None else None
        if not self._layer:
            if X is not None and abs(X) < LAYER_NEAR_MM:
                self._layer = True
                if self.turn is None:
                    self._start_layer(now_ms)
        else:
            if X is None or abs(X) > LAYER_FAR_MM:
                self._layer = False
        return self._layer

    def _start_layer(self, now_ms):
        """Fresh rotation tracking. phi continues from where it left off if
        we were tracking moments ago -- see HEADING_CARRY_MS."""
        carry = 0.0
        if (self._rot_left_ms is not None
                and time.ticks_diff(now_ms, self._rot_left_ms)
                < HEADING_CARRY_MS):
            carry = self._heading_carry
        self.turn = {
            "dir": 1, "theta0": None, "x0": None, "y0": None, "fix_age": None,
            "seeded": True, "t_start": now_ms,
            "r0": None, "r_prev": None, "E0": 0.0,
            "phi": carry, "omega": 0.0, "dwell": 0, "n_bad": 0, "n_lowE": 0,
            "n_gated": 0, "n_frames": 0, "n_print": 0,
            "last_fit_frame": -1, "err_final": None,
        }

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
        # Continue the turn count if we were in rotation moments ago -- see
        # HEADING_CARRY_MS. A genuinely new turn starts from zero.
        carry = 0.0
        if (self._rot_left_ms is not None
                and time.ticks_diff(now_ms, self._rot_left_ms)
                < HEADING_CARRY_MS):
            carry = self._heading_carry
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
        # Remember where the turn count got to, so a bounce straight back
        # into rotation continues rather than restarting -- HEADING_CARRY_MS.
        if self.turn is not None:
            self._heading_carry = self.turn["phi"]
        self._rot_left_ms = now_ms
        if self.turn is not None and err_final is not None:
            self.turn["err_final"] = err_final
        self.reseed_pending = True      # main loop clears Alg1 state
        self._reacq_good = 0
        self._t_reacq = now_ms
        self.last_fix = None
        self.state = REACQUIRE

    # ---- HOOK 2: the turn frame (Algorithm 2), TURN_INIT / TURNING --------
    def turn_frame(self, buf, now_ms):
        if self.state != TURN_INIT and self.state != TURNING:
            return                          # FAULT: idle until host DRIVE
        t = self.turn
        raw, E, n = grad4_angle(buf, self.W, self.H)
        dt = time.ticks_diff(now_ms, self._t_prev) * 1e-3
        self._t_prev = now_ms
        if dt <= 0.0:
            dt = 0.01

        if t["r0"] is None:
            t["r0"] = raw
            t["r_prev"] = raw
            t["E0"] = E
            t["init"] = 1
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

        # alpha-beta on the integrated measurement z = phi_meas
        z = t["phi"] + d              # measurement: previous filtered + step
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
                f = rot_scan.fit_frame(buf, self.W, self.H, thr, raw)
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
        if (t["n_print"] % TURN_PRINT_EVERY) == 0:
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
            self._rot_left_ms = now_ms
            return

        # "Settled" is a hint to the host. Only DRIVE leaves TURNING.
        if False:
            t["n_frames"] = t.get("n_frames", 0)
            if t["n_frames"] < ROT_MIN_FRAMES:
                return
            # Leave only once a real turn has HAPPENED and then stopped.
            # Merely being calm is not enough -- we were calm on arrival.
            turned = abs(t["phi"]) >= ROT_TURNED_DEG
            calm = abs(t["omega"]) < ROT_EXIT_OMEGA
            t["dwell"] = t["dwell"] + 1 if (turned and calm) else 0
            if t["dwell"] >= ROT_EXIT_FRAMES:
                self._enter_reacquire(now_ms, err)
                return
            # ESCAPE: parked, then drove on without ever turning. Without
            # this the "must have turned" rule above would hold us in
            # rotation forever. Stage 2 keeps reporting a junction while
            # one is in view, so losing it means we have driven away.
            if self.fit is None or self.fit.get("junction") is None:
                self._no_junc = self._no_junc + 1
                if self._no_junc >= ROT_LEFT_FRAMES:
                    print("  left the junction without turning -- back to drive")
                    self._enter_reacquire(now_ms, err)
            else:
                self._no_junc = 0
            return

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

            # layer_update() is called from the main loop instead; there is
            # no mode to enter any more.
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
# In the per-frame loop of the main script (see v1_2.py):
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
