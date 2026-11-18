# turn_sm.py -- rotation: Algorithm 2 angle + unwrap + filter, and the
# DRIVE / TURN state machine, for the OpenMV AE3 grid robot.
#
# Owns
#   * grad4_angle(G): the grid angle mod 90 from the sums of the C grad4
#     kernel (gradient-angle quadrupling with Scharr taps; the tile runs
#     the kernel)
#   * the mod-90 unwrap and the anchored alpha-beta (phi, omega) filter
#   * fold_x(d, x_range, spacing): the Tile 1 X scheme (fold around the
#     junction behind, null zone between), shared by both tiles
#   * the state machine  DRIVE -> TURN_INIT -> TURNING -> REACQUIRE -> DRIVE
#     (+ FAULT), driven by host commands "TURN L" / "TURN R" / "DRIVE";
#     "LIGHT" (DRIVE only) asks main.py for the light setup
#   * when Stage 2 runs: the tile's stage2() (Tile 1: a tilted scan, in
#     tile1.py) aimed from the continuous angle, giving X, Y, theta while the
#     axis-aligned drive algorithm is stood down
#
# Does NOT own Algorithm 1 (drive) or any pixel work. The tile object
# (tile1.Tile1) measures: turn_angle(buf) gives the angle of one frame,
# stage2(buf, aim) the tilted fit. main.py calls the hooks listed at the
# bottom.
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


# ---- Stage 2: rotated scan + line fit, run during the turn.
# Stage 1 gives theta only. Stage 2 uses that theta to aim a tilted scan,
# which recovers the two bars and the junction (X, Y) -- the same
# scan/fit/intersect the driving algorithm does, just not axis-locked.
STAGE2 = True          # False -> angle only, the cheap mode. How often
                       # Stage 2 runs is the tile's STAGE2_EVERY (Tile 1: on
                       # every 2nd turn frame); Stage 1 still runs every
                       # frame, so phi never skips.
STAGE2_HOLD = 12       # keep drawing the last good fit for this many frames
                       # after a miss. Stops the overlay blinking on and off
                       # between Stage 2 runs; costs nothing.

# AUTO_MODE = True: a turn never self-terminates and never faults; it runs
# until the host sends "DRIVE". turn_frame() keeps phi/omega current and
# returns before the settle/exit logic. This is the mode the split-mode
# main script is built for.
AUTO_MODE = True

TURN_PRINT_EVERY = 0       # 0 = off. The main script prints one ROT status
                           # line per second; set N to also get this
                           # module's own "T phi=.. err=.. w=.." every N
                           # frames when debugging the filter.

TURN_FREERUN = not AUTO_MODE   # derived; False in AUTO_MODE

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
# Four changes, all measured on rotation_wrong.mp4 against the raw unwrapped
# angle (sum of wrap90 steps, which never folded on that clip -- largest
# step 5.5 deg against a 45 deg limit). None touch grad4_angle or cost any
# frame time.
#
# 1. THE FILTER IS ANCHORED. It used to measure z = phi_prev + d: chasing
#    its own previous output, with nothing tying it to the sum of the
#    measured steps. Now an explicit accumulator psi = sum(d) is kept and
#    the filter measures z = psi. Same alpha-beta, one float add -- but the
#    estimate can no longer wander off the measurement.
#       worst error while rotating, same gains:   4.57 -> 2.43 deg
#    When still it was already right (+0.17 / -0.25 deg); unchanged.
#
# 2. GAINS COMPENSATED. With the anchor in place the gains can be opened
#    up without the estimate drifting:
#       alpha .50 beta .10  worst while rotating 2.43 deg   sd still 0.77
#       alpha .85 beta .30  worst while rotating 0.76 deg   sd still 0.82
#    The raw angle's own sd on that plateau was 0.86, so .85/.30 costs
#    0.05 deg of smoothing to buy 1.7 deg of lag.
#
# 3. phi IS SEEDED FROM THE MEASURED TILT, so 90.0 means square. It used
#    to start at 0 wherever the grid was; that clip started at -1.46 deg,
#    so 'phi = 90' would have left 1.46 deg of tilt. The seed is drive's
#    theta if it agrees with Algorithm 2's first reading within
#    SEED_THETA0_TOL, else Algorithm 2's reading (measured earlier: the
#    two disagree by sd 2.0 deg, 15.8 worst; a bad seed is a permanent
#    offset).
#
# 4. NO HEADING CARRY INTO A TURN. A turn re-armed within 1.5 s
#    used to START from the previous turn's phi. End a turn mid-rotation,
#    re-arm, and the new one begins at a stale number -- and holds it if
#    the robot then sits still. That is the mechanism that puts a gauge
#    reading like +25.7 against a square grid. A turn now always starts
#    from measurement. The carry survives only for the gauge display
#    between turns, so the wheel does not blink to zero.
AB_ALPHA = 0.85        # was 0.5 -- see (2)
AB_BETA = 0.30         # was 0.1
SEED_THETA0_TOL = 5.0  # deg, see (3)
SEED_THETA0_MAX_AGE = 10   # frames; a stale drive fix is not a seed
STAGE2_AIM = True      # aim Stage 2 from the continuous angle so the two
                       # tape families cannot swap at 45 deg (X/Y/T jumped
                       # by up to 6 mm / 88 deg there on rotatiom.mp4)

TURN_LEFT_SIGN = +1    # see note at top. "TURN R" uses the opposite sign.

# state machine states
DRIVE, TURN_INIT, TURNING, REACQUIRE, FAULT = range(5)
STATE_NAME = ("DRIVE", "TURN_INIT", "TURNING", "REACQUIRE", "FAULT")

# ------------------------------------------------------------ Algorithm 2
def grad4_angle(G):
    """G = array('i', 4) from the C grad4: [sre>>k, sim>>k, n, k]. Returns (th, E, n)."""
    sre = G[0] << G[3]
    sim = G[1] << G[3]
    if sre == 0 and sim == 0:
        return 0.0, 0.0, 0
    th = math.degrees(math.atan2(sim, sre)) * 0.25
    th = ((th + 45.0) % 90.0) - 45.0
    return th, math.sqrt(float(sre) * sre + float(sim) * sim), G[2]


def wrap90(d):
    """Map an angle difference into [-45, 45)."""
    while d >= 45.0:
        d -= 90.0
    while d < -45.0:
        d += 90.0
    return d


def fold_x(d, x_range, spacing):
    """d (mm, + = junction ahead) -> X in the Tile 1 scheme, or None (null zone).

    d is folded into (-spacing, 0], so the junction behind is the reference:
        d in [-x_range, 0]                  -> X = d           (leaving: 0 -> -30)
        d in [-spacing, -(spacing-x_range)] -> X = d + spacing (approaching: +30 -> 0)
        in between                          -> None            (null zone)
    The caller multiplies by its X direction. spacing must be >= 1 (mm).
    """
    if d != d or d > 1e6 or d < -1e6:      # nan / inf: the loops below would never end
        return None
    while d > 0.0:
        d -= spacing
    while d <= -spacing:
        d += spacing
    if d >= -x_range:
        return d
    if d <= -(spacing - x_range):
        return d + spacing
    return None


# --------------------------------------------------------- one DRIVE frame
class FrameResult:
    """What the tile measured on one DRIVE frame, in the Tile 1 meaning for
    both tiles (DESIGN 3.3). main.py makes one at power-on and calls clear()
    before every tile drive(); the tile only sets the fields, and the loop
    reads only these fields, never the tile's insides.
      tier     0 nothing, 1 junction, 2 Y only (one bar across), 3 X only
      X, Y, T  mm, mm, deg, or None (X is None in the null zone)
      fix      (1, X, Y, T) for a junction that passed the checks, else None
      suspect  the reading was refused (it jumped too far)
      research the frame needed a full scan / search
      status   the scan status text
      node     name of the junction X is measured from, or None
      q        this frame's quality inputs (for the tile's score only)"""

    def __init__(self):
        self.clear()

    def clear(self):
        self.tier = 0
        self.X = self.Y = self.T = None
        self.fix = None
        self.suspect = False
        self.research = False
        self.status = ''
        self.node = None
        self.q = None


# --------------------------------------------------------- host command link
class HostLink:
    """Line protocol, ASCII, newline-terminated:
         host -> camera : TURN L | TURN R | DRIVE | STOP | LIGHT
         camera -> host : one status line per frame + events (see send()).
    uart=None means bench mode: commands come only from the sim_cmds list
    [(frame_id, "TURN L"), ...]."""

    def __init__(self, uart=None, sim_cmds=None, echo=True):
        self.uart = uart
        self.sim = list(sim_cmds or [])
        self.echo = echo
        self._buf = b""
        self._q = []

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


class TurnSM:
    def __init__(self, link, W, H, tile):
        self.link = link
        self.tile = tile            # measures each turn frame (turn_angle, stage2)
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
        self.light_pending = False  # LIGHT taken: main runs the light setup after this frame
        self.fit = None         # last good Stage 2 result, held briefly
        self.fit_age = 999      # frames since that fit was fresh
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
            self.reseed_pending = True
            self.state = DRIVE
            return
        self.state = FAULT

    def heading(self):
        """(phi, omega) for the gauge: live during a turn, the last known
        phi between turns, so the wheel never blinks to zero."""
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
        if cmd == "LIGHT" and self.state == DRIVE:
            # the light setup, on demand: main runs it at the end of this
            # frame and then calls light_done(). In any other state LIGHT
            # gets the CMD_IGNORED line below.
            self.light_pending = True
        elif cmd.startswith("TURN") and self.state == DRIVE:
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

    def light_done(self):
        """After the LIGHT setup (main.py): the frame loop waited 2-5 s, so
        the drive tracking starts again, as after a turn: main clears its
        drive state on the next frame (reseed_pending), the last fix is
        gone (a TURN right after needs a fresh one) and JUNCTION_LANDED
        counts its frames again. Stays in DRIVE."""
        self.light_pending = False
        self.reseed_pending = True
        self.last_fix = None
        self._landed_n = 0

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
    def turn_frame(self, buf, now_ms):
        if self.state != TURN_INIT and self.state != TURNING:
            return                          # FAULT: idle until host DRIVE
        t = self.turn
        a = self.tile.turn_angle(buf)
        if a is None:
            # no edges this frame (a tile may say so): skip it before dt,
            # so the next dt spans the gap
            return
        raw, E, n = a
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
        every = self.tile.STAGE2_EVERY
        if STAGE2 and (t["n_frames"] % every) == 0:
            try:
                # stage2_us times the whole call, the tile's threshold too
                _t2a = time.ticks_us()
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
                f = self.tile.stage2(buf, aim)
                self.stage2_us = time.ticks_diff(time.ticks_us(), _t2a)
                if f is not None and f["tier"] == 1:
                    self.fit = f
                    self.fit_age = 0
                    t["last_fit_frame"] = t["n_frames"]
                else:
                    self.fit_age += every
            except Exception as e:
                self.fit_age += every
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
                    # re-find the grid on its own.
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
#   SM.poll_cmd(frame_id, now_ms)              # 1. any state (LIGHT sets light_pending)
#   if SM.alg1_active():
#       if SM.reseed_pending:                  #    clear Alg1 state once
#           TL.reseed(); ...; SM.reseed_pending = False
#       if SM.force_full_scan(): TL.force_full()   # full scan while reacquiring
#       ... TL.prepare + TL.drive fill R (FrameResult) ...
#       SM.drive_result(R.fix, now_ms)         # 3.
#   else:
#       SM.turn_frame(buf, now_ms)             # 2. the tile's turn_angle + stage2
#   if SM.light_pending:                       # end of the frame: the light setup,
#       ... ; SM.light_done()                  #    then the tracking starts again
