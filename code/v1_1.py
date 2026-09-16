# scan_fit_fps_live.py -- mini-junction v4  +  DRIVE/TURN state machine (v1_2)
#
# v1_2: Algorithm 1 (everything below) is unchanged. A state machine from
# turn_sm.py decides, per frame, whether Algorithm 1 runs (DRIVE, REACQUIRE)
# or Algorithm 2 runs (TURN_INIT, TURNING). Search for "SM." to see the
# three hooks. Design: drive_turn_state_machine.md.
#
# Bench test without a host: set SIM_CMDS = [(300, "TURN L")] and play the
# rotation video; the turn starts at frame 300.
#
# WHAT CHANGED vs v3 (mini-junction-v3-co-based), and why:
#
# BUG 1 (fatal): the full-junction branch contained a pasted-in block
#   referring to `a`, `b` and `mid_px` -- names that only ever exist
#   inside the mini-junction branch. On the first real junction frame
#   that raises NameError, which the frame try/except swallows into
#   "FRAME ERROR", sets band=None and throws the frame's result away.
#   So EVERY full junction frame was being lost. Removed.
#
# BUG 2: the cross marker had been moved (by the same bad paste) OUT of
#   the mini branch and INTO the full-junction branch. In mini mode
#   nothing was drawn at the computed point at all -- which is exactly
#   the "I can't see where it thinks the point is" symptom. Put back,
#   plus two short tick marks at the measured gap edges so it is obvious
#   what was actually measured.
#
# BUG 3 (fps / "choppy in the middle"): in track mode the loop did
#       if bx1 < 0 or n_c < MIN_PTS or n_r < MIN_PTS: <full re-acquire>
#   Between two junctions bx1 is ALWAYS -1 (that is the definition of
#   being between junctions), so every single frame in the mini region
#   ran TWO scans: the cheap STEP-spaced one and then a full-width one.
#   Setting band=(0,W) in the mini branch did not help, because this
#   test fires before it is ever consulted. Now: if the cheap scan
#   already supports the horizontal fallback, no full re-acquire is
#   forced; a full scan still runs every REACQUIRE_MINI_EVERY frames so
#   a reappearing vertical bar is picked up quickly. Note the cheap
#   column scan already covers the FULL width (only row_scan is band
#   limited), so a vertical bar is not missed in between.
#
# BUG 4 (false mini-junctions): largest_co_gap() returned the largest
#   x-jump between collected points with no further checks. In track
#   mode points are STEP=5 px apart, so ONE dropped column is a 10 px
#   "gap" -- and 10 px * ~0.43 mm/px = 4.3 mm, which passed the 4.0 mm
#   threshold. Any single noisy column anywhere along a normal bar
#   produced a mini-junction with a bogus X. Replaced with
#   find_mini_gap(), which additionally requires: gap wider than
#   MINI_GAP_STEP_MULT * sample spacing, an absolute pixel floor, real
#   points on BOTH sides, and each side spanning a real distance (so a
#   single stray point can no longer act as one "side").
#
# Everything else -- the scan/fit viper routines, config B, the
# full-junction path -- is untouched.

ITERS = 3            # MAD rejection rounds -- config B
WSIGMA = 30          # Gaussian width of the junction-local fit, px
BAR_WIDTH_MM = 13.5  # UNVERIFIED placeholder -- see MM_PER_PIXEL below.
MM_PER_PIXEL = 0.223     # 0 = use BAR_WIDTH_MM/tc (recomputed every frame
                     # from the noisy tape-thickness measurement).
                     # SET THIS: calibrate off the 80 mm junction
                     # spacing, which is a far better reference than a
                     # thin tape's own width.
                     #   1. note raw jx at one junction (printed below)
                     #   2. move to the next junction, note jx again
                     #   3. MM_PER_PIXEL = 80.0 / abs(jx2 - jx1)
                     # Until this is set, every mm number below --
                     # including the mini-junction X -- is scaled by a
                     # value that jitters frame to frame.

MINI_JUNCTION_OFFSET_MM = 40.0  # half the 80 mm junction spacing.
                     # Convention check (this is worth understanding,
                     # it is what makes mini X agree with junction X):
                     # let d = (mid_px - W/2) * mmpx = signed mm from
                     # image centre to the gap midpoint. The junction
                     # to the RIGHT is at d + 40. So
                     #     X = 40 + d
                     # is the signed distance to the next junction
                     # ahead, and it decays smoothly to 0 exactly as
                     # that junction arrives under the camera centre --
                     # i.e. it joins continuously onto the full
                     # junction X, no 80 mm step at the handover.

# --- mini-junction gap acceptance (BUG 4) ---
MINI_GAP_MIN_MM = 4.0      # physical floor; kept permissive because the
                           # true gap size (44 mm if segments are centred
                           # on each dot, ~8 mm if they extend inward) is
                           # still unconfirmed.
MINI_GAP_MIN_PX = 12       # absolute floor, independent of the mm scale
                           # (which is unreliable until MM_PER_PIXEL is
                           # calibrated). A real void is tens of px.
MINI_GAP_STEP_MULT = 3.0   # gap must exceed this many column sample
                           # spacings -- kills the single-dropped-column
                           # false positive that 4.0 mm let through.
MINI_SIDE_MIN_PTS = 4      # real points required on EACH side of the gap
MINI_SIDE_MIN_SPAN_PX = 20 # each side must span this much x, so one
                           # stray point cannot count as a segment
MINI_SIDE_MIN_RATIO = 2.5  # THE important one. Measured off the real
                           # video: the floor has small ~5mm gaps on
                           # BOTH SIDES of every junction as well as one
                           # at the cell midpoint. The junction ones sit
                           # either side of a short ~23px stub, the
                           # midpoint one has ~90-140px segments either
                           # side. Requiring each neighbouring run to be
                           # at least this many times the gap width is
                           # what separates them -- and it needs no mm
                           # calibration to work.
MINI_SIDE_ABS_PX = 70      # ceiling on the ratio test above: a side
                           # this long is convincing on its own. Makes
                           # no difference on the recorded video (tested
                           # 60/70/80/no-cap, identical), but stops the
                           # ratio rule over-reaching if perspective
                           # ever widens the void.
MAX_STEP_MM = 30.0         # physically impossible jump filter. At the
                           # loop rate the camera cannot move further
                           # than this between frames: measured on
                           # fast_ramp1, hand-waving at up to 1.56 m/s
                           # gave a median 9.8 mm and a maximum 26.0 mm
                           # of travel per frame. A robot on wheels
                           # will be well under that. Anything larger
                           # is a bad fit, not motion. Compared modulo
                           # the cell spacing so a legitimate wrap is
                           # never flagged. Set to 0 to disable.
WRAP_DEADBAND_MM = 6.0     # do not wrap X to ~80 the instant the
                           # junction slips a hair behind the camera
                           # centre. Sitting ON a junction gives d
                           # jittering around 0, so a bare "d < 0 ->
                           # d+80" test flips between ~0.3 and ~79.7
                           # frame to frame -- exactly what X+79.7 in
                           # the annotated video was. Junction X noise
                           # is ~0.3mm, so a 6mm deadband cannot be
                           # crossed by noise, and X simply reads a
                           # small negative number while you straddle
                           # the junction.
# ======================= X ODOMETRY (new scheme) =========================
# X is no longer "distance to the next junction, 0..80". It is now signed
# and relative to whichever junction is nearer, with a dead zone in the
# middle of the cell where no honest X exists:
#
#      junction                                              junction
#         |                                                     |
#    0 -> -30 mm ......... null (no X) ......... +30 mm ->  0
#         leaving                                    approaching
#
#   * Just after leaving a junction X counts DOWN from 0 to -30 mm.
#   * Past -30 mm the junction behind is too far to measure -> X = None.
#   * Once the junction ahead comes within 30 mm, X jumps to +30 and
#     counts down to 0 as you arrive. Then the cycle repeats.
#
# The Y line is still drawn and still reported while X is null -- only the
# X NUMBER goes away, not the tracking.
X_RANGE_MM = 30.0          # how far either side of a junction X is valid
X_DIRECTION = +1           # flip to -1 if a camera is mounted 180 deg from
                           # the other, so "leaving" and "approaching" swap.
                           # Applied to the sign of X only.

JUNCTION_SPACING_MM = 80.0 # centre-to-centre. X below is reported as
                           # the distance to the NEXT junction ahead,
                           # in (0, 80]. Any grid position has to wrap
                           # once per cell; this puts the wrap at the
                           # junction itself -- a fast, precisely
                           # detected transit -- instead of at the cell
                           # midpoint. v5 had it at the midpoint, which
                           # is exactly where the camera lingers, so X
                           # flipped between -40 and +40 frame to
                           # frame. Measured on both videos: worst
                           # frame-to-frame jump 79.2/79.8mm before,
                           # 2.5/2.9mm after.
TILT_DEBUG = 0             # once a second, when a full junction is in
                          # view, fit the horizontal bar SEPARATELY on
                          # each side of the junction and print both
                          # slopes. If the two sides disagree, the tape
                          # itself is not collinear and no fit can be
                          # right. If they agree with each other but
                          # not with the combined fit, something near
                          # the junction (the round centre dot) is
                          # pulling it. Different causes, different
                          # fixes -- this tells them apart without a
                          # video.
MINI_DEBUG = 1             # print measured gap px/mm once per summary;
                           # this is how the true gap size gets known.
MIN_V_SPAN_PX = 6          # a real vertical bar covers a RANGE of
                           # columns. Measured on middle4.mp4: the
                           # false detections were 1 and 5 columns wide
                           # (lens glare at the frame edge); every real
                           # junction was 9-28 columns wide.
THICK_MATCH_FRAC = 0.55    # the vertical bar is the SAME TAPE as the
                           # horizontal one, so its measured width must
                           # be comparable. Real junctions measured
                           # tr/tc = 0.60-1.10 across both videos; the
                           # glare detections came in at 0.20-0.45.
REACQUIRE_MINI_EVERY = 12  # full-width rescan cadence while in mini mode

PERF = 1             # once a second, print where the loop time ACTUALLY
                     # goes: capture / grayscale convert / otsu / scan
                     # / fit / draw / flush / gc. Costs a few
                     # microseconds of ticks_us calls and settles the
                     # question instead of us guessing at it. Turn off
                     # once tuned.
GC_EVERY = 30        # gc.collect() was running EVERY frame. On
                     # MicroPython a full collect is milliseconds, and
                     # at a 10ms budget that alone can be a third of
                     # the loop. Every 30 frames is plenty here: the
                     # loop allocates almost nothing per iteration
                     # (the arrays are preallocated once). Set to 1 to
                     # restore the old behaviour.
OTSU_EVERY = 3       # recompute the threshold every N frames. The
                     # copy+histogram is a real per-frame cost. Otsu
                     # was measured swinging 22 grey levels frame to
                     # frame anyway, so reusing it for 2-3 frames
                     # loses very little -- but measure with PERF
                     # before turning this up.
FLUSH = 1            # keep csi0.flush() even when DRAW is 0.
                     # WHY THIS IS SEPARATE: with DRAW=0 the measured
                     # rate FELL to 40-50 fps from ~80. Not drawing
                     # cannot cost time, so the suspect is flush(),
                     # which was guarded together with the drawing.
                     # flush() does not only push pixels to the IDE --
                     # it also releases the frame buffer, and without
                     # that release snapshot() may stall waiting for a
                     # free one. Test the three combinations below and
                     # keep whichever is fastest ON THE ROBOT, with the
                     # IDE disconnected, since that is the case that
                     # matters:
                     #     DRAW=1 FLUSH=1   full overlay, ~80 fps
                     #     DRAW=0 FLUSH=0   measured 40-50 fps
                     #     DRAW=0 FLUSH=1   <- the one still untested
DRAW = 1             # 1 = draw the overlay and flush to the IDE.
                     # SET TO 0 ON THE ROBOT. csi0.flush() pushes the
                     # whole frame over USB every iteration and it sits
                     # inside the timed loop: the IDE reported 137 fps
                     # at the camera against 66 for the loop. The
                     # overlay exists for you, not for the robot, and
                     # halving the frame period also halves the motion
                     # blur per the note above -- this is the single
                     # biggest live win available.
COLOR_OVERLAY = 0    # 1 = capture RGB565 so the two guide lines can be
                     # drawn in real colour. Every scan and fit routine
                     # in this file indexes the frame as ONE BYTE PER
                     # PIXEL, so they must never see the RGB buffer --
                     # each frame is converted to a grayscale copy for
                     # processing, and only the overlay is drawn on the
                     # colour original. Set to 0 to go back to
                     # GRAYSCALE capture (lines become white / dim grey).
COLOR_X = (255, 255, 255)  # X line -- white (grayscale capture)
COLOR_Y = (90, 90, 90)     # Y line -- dim grey, so the two
                           # lines are still tellable apart
COLOR_MARK = (255, 255, 255)   # cross + text

SUMMARY_MS = 1000    # print interval

EXPOSURE_US = 3000   # WORKS NOW -- fixed by the Sept development firmware
                     # (build 631681e5ac). Was a no-op on release v5.0.0;
                     # confirmed tracking linearly: 1000us->mean15,
                     # 2000us->mean30. 1500 splits the difference: ~5px
                     # blur at 0.75 m/s (was 28px at the old stuck 8328us).
                     # Global shutter readout floors fps near 60 regardless
                     # of this value -- shorter exposure does NOT cost fps
                     # on this sensor, unlike a rolling shutter.
GAIN_DB = 18.0       # your only real brightness control. Measured:
                     # mean 42 at 12 dB, 71 at 18, 106 at 24, with
                     # headroom staying above 100 throughout.
GAIN_CEILING_DB = 8  # unused while EXPOSURE_US is set
FRAMERATE = 240      # ACCEPTED BUT NOT HONOURED. The driver reports
                     # back 240, yet the applied exposure is 8328 us =
                     # 1/120 s, so the sensor really runs at 120 fps.
                     # Windowing to 320x100 did not change it either
                     # (the crop is applied after readout).
                     #
                     # WHY THIS MATTERS: exposure == frame period on
                     # this sensor, and blur = exposure x speed. That
                     # is 8328 us, which at 0.75 m/s smears the bar
                     # 28 px -- confirmed against fast_ramp1, where
                     # measured edge width tracked prediction (7.0 vs
                     # 8.2 px, 14.0 vs 13.7 px). Blur is therefore NOT
                     # fixable in software on this firmware. The
                     # levers are: move slower, light the floor
                     # better, or take it up with OpenMV.
FRAMEBUFFERS = 0     # 0 = leave alone

import time
from turn_sm import TurnSM, HostLink
import hud

HOST_UART_ID = None   # e.g. 1 -> machine.UART(1, HOST_BAUD). None = bench mode:
HOST_BAUD = 115200    # commands come only from SIM_CMDS / SM.link.inject().
SIM_CMDS = [(0,"TURN L")]         # MUST be empty. This was [(0, "TURN L")] left over
                      # from the rotation benchmark, which forced the board
                      # into TURNING on frame 0 and kept it there -- which
                      # is why the rotation HUD kept showing and the X line
                      # never appeared. Only set this for bench testing.

# HAND-TEST BUTTON: press once to start a turn (direction alternates L/R
# with each press), press again to send DRIVE and end it. No wiring needed
# beyond one GPIO pin to a button that pulls to ground -- change BUTTON_PIN
# to whatever pin you actually have one on. None = disabled.
BUTTON_PIN = None      # e.g. "P0"
BUTTON_DEBOUNCE_MS = 250
print("=== scan_fit_fps_live.py -- VERSION: mini-junction-v17-fast ===")
print("=== if you do NOT see 'v4' on the line above you are running")
print("=== an old file. Close the tab in the IDE, reopen the download.")
import gc
import math
import os
import image
from array import array

# ---------------------------------------------------------- VERIFY: sensor
try:
    import csi
    csi0 = csi.CSI()
    csi0.reset()
    if COLOR_OVERLAY:
        csi0.pixformat(csi.RGB565)
    else:
        csi0.pixformat(csi.GRAYSCALE)
    csi0.framesize(csi.QVGA)
    if FRAMERATE:
        try:
            csi0.framerate(FRAMERATE)
        except Exception as _e:
            print("framerate(%d) failed (%s)" % (FRAMERATE, _e))
    if EXPOSURE_US:
        try:
            csi0.auto_exposure(False, exposure_us=EXPOSURE_US)
        except Exception as _e:
            print("auto_exposure failed (%s) -- still on auto" % _e)
        try:
            csi0.auto_gain(False, gain_db=GAIN_DB)
        except Exception as _e:
            print("auto_gain failed (%s)" % _e)
    elif GAIN_CEILING_DB:
        try:
            csi0.auto_gain(True, gain_db_ceiling=GAIN_CEILING_DB)
        except Exception as _e:
            print("auto_gain ceiling failed (%s)" % _e)
    if FRAMEBUFFERS:
        try:
            csi0.framebuffers(FRAMEBUFFERS)
        except Exception as _e:
            print("framebuffers(%d) failed (%s)" % (FRAMEBUFFERS, _e))
    csi0.snapshot(time=500)
    HAVE_SENSOR = True
except Exception as e:
    HAVE_SENSOR = False
    print("csi init failed: %s" % e)

# --------------------------------------------------------- settled config
MIN_LEN = 4
LONG_CUT = 60
EXTRA = 8
MAXPTS = 2000
THICK_LO = 0.60
THICK_HI = 1.35
NOMINAL_THICK_PX = 29  # measured tape thickness on this rig (from the
                       # background docs). hist_peak() below just counts
                       # which run length shows up MOST OFTEN this frame --
                       # it has no idea which width is the real bar and
                       # which is a tooth on a serrated edge. If teeth
                       # outnumber genuine full-width crossings, hist_peak
                       # locks onto the TOOTH width, and THICK_LO/HI then
                       # filters FOR teeth and AGAINST the real bar, on
                       # purpose, every frame. This anchor catches that:
                       # a measured peak wildly off from the known real
                       # thickness is discarded in favour of the known
                       # value, rather than trusted blindly.
MAD_SHIFT = 9
MAD_BINS = 256
WTAB_N = 256
WMAX = 64
NOGATE = 1 << 28
MIN_PTS = 8
MAX_JOINT_MAD_PX = 3.0 # same test as MAX_FIT_MAD_PX below, but for the
                       # FULL-JUNCTION path. Measured on fast_ramp1:
                       # during fast motion the frames blur and the
                       # VERTICAL fit degrades badly (residual median
                       # 2.05px, p90 15.1px) while the horizontal fit
                       # stays clean (0.26px). The junction path was
                       # claiming those frames and reporting nonsense
                       # (T+12.0, Y+21.6) when the mini/Y fallback would
                       # have been accurate on the very same frame.
                       # Rejecting here hands the frame to that
                       # fallback instead of trusting a bad fit.
MAX_FIT_MAD_PX = 5.0   # the fallback's real "is there a bar?" test.
                       # Replaces the old peak-bin thickness-consensus
                       # check, which was measured to be BACKWARDS: on
                       # 851 recorded frames a real bar scored 0.29 and
                       # pure noise scored 0.50, so it rejected 55% of
                       # good frames (median 0.29 against a 0.30
                       # threshold -- a coin flip every frame) while
                       # letting noise through. What actually separates
                       # them is whether the points lie on a LINE:
                       # real bar median fit residual 0.18-0.23 px,
                       # worst 0.49; noise 42-49 px. 5.0 px sits a
                       # factor of ten clear of both.
THICKNESS_FRAC = 0.3
COVERAGE_FRAC = 0.25
ACQUIRE_STEP = 2       # column/row spacing for the FULL re-acquire
                       # scan. Was 1, i.e. every single column -- 320
                       # of them against 64 for the tracked scan, which
                       # is why scan+fit balloons whenever the lock is
                       # lost. 2 halves that cost. The acquire pass
                       # only has to FIND the marker well enough to set
                       # a band; the tracked pass does the accurate
                       # work. Raise to 3-4 if you need more, but check
                       # the marker is still picked up reliably.
REACQUIRE_EVERY = 30
OTSU_SCALE = 0.25
STEP = 5
PRINT_MS = 15
MAXFRAMES = 4000


def _resolve_gray():
    """Pick a working RGB565 -> GRAYSCALE conversion for this firmware.

    Tried in order against a real frame rather than assumed, because the
    method name has moved between OpenMV versions. Returns a function,
    or None if the frame is already grayscale / nothing works.
    """
    if not COLOR_OVERLAY:
        return None
    probe = csi0.snapshot()

    def m1(im):
        return im.to_grayscale(copy=True)

    def m2(im):
        return im.copy(copy_to_fb=False, pixformat=image.GRAYSCALE)

    def m3(im):
        return im.copy().to_grayscale()

    for fn, name in ((m1, "to_grayscale(copy=True)"),
                     (m2, "copy(pixformat=GRAYSCALE)"),
                     (m3, "copy().to_grayscale()")):
        try:
            g = fn(probe)
            n = len(g.bytearray())
            if n == probe.width() * probe.height():
                print("grayscale conversion: %s   OK (%d bytes)" % (name, n))
                return fn
            print("grayscale conversion: %s gave %d bytes, want %d"
                  % (name, n, probe.width() * probe.height()))
        except Exception as _e:
            print("grayscale conversion: %s failed (%s)" % (name, _e))
    print("NO working RGB565->GRAYSCALE conversion found.")
    print("Set COLOR_OVERLAY = 0 at the top of this file and re-run;")
    print("you will get white / dim-grey lines instead of green / blue.")
    return None


def _dim(o, n, d):
    a = getattr(o, n, None)
    if a is None:
        return d
    try:
        return int(a() if callable(a) else a)
    except Exception:
        return d


def _get(o, n, d="?"):
    a = getattr(o, n, None)
    if a is None:
        return d
    try:
        return a() if callable(a) else a
    except Exception:
        return d


if HAVE_SENSOR:
    W = _dim(csi0, "width", 320)
    H = _dim(csi0, "height", 200)
    print("frame size from sensor: %dx%d" % (W, H))
    _exp = _get(csi0, "exposure_us")
    print("exposure requested %s us -> applied %s us   gain %s dB"
          % (EXPOSURE_US, _exp, _get(csi0, "gain_db")))
    TO_GRAY = _resolve_gray()
    if COLOR_OVERLAY and TO_GRAY is None:
        raise SystemExit("stopping: see the message above")
    _im = csi0.snapshot()
    if TO_GRAY:
        _im = TO_GRAY(_im)
    _sm = _im.copy(x_scale=OTSU_SCALE, y_scale=OTSU_SCALE)
    _th = int(_get(_sm.get_histogram().get_threshold(), "value", -1))
    del _sm
    _st = _im.get_statistics()
    _mx = _get(_st, "max", -1)
    _mn = _get(_st, "mean", -1)
    del _im
    try:
        _head = int(_mx) - _th
    except Exception:
        _head = -1
    print("brightness: mean %s  max %s  otsu %d  head %s"
          % (_mn, _mx, _th, _head))
    if isinstance(_head, int) and 0 <= _head < 40:
        print("  WARNING: only %d grey levels between marker and Otsu cut."
              % _head)
        print("  Raise GAIN_DB (try 12-18) and re-check this line.")
else:
    W, H = 320, 200

REJECT = 2.5

try:
    @micropython.viper
    def _p(x: int) -> int:
        return x + 1
    _p(1)
    VIPER = True
except Exception as e:
    VIPER = False
    print("viper unavailable: %s" % e)


if VIPER:
    @micropython.viper
    def col_scan(buf: ptr8, w: int, h: int, th: int, minlen: int,
                 longcut: int, cols: ptr32, nc: int, out: ptr32,
                 maxn: int, res: ptr32, hist: ptr32):
        # FIRST VALID-WIDTH RUN WINS. A column can cross more than one
        # bright run whose length happens to fall inside [minlen, longcut)
        # -- floor texture, a mounting hole edge, a second bar further
        # down. Every one of those used to become its own dot, all feeding
        # the same fit. Now: the first run whose width is in-band is kept
        # (this is "the point from the left" for a column: topmost), and
        # scanning that column stops right there. A run that is too SHORT
        # (below minlen) is still skipped over silently -- that is noise,
        # keep looking further down for the real bar.
        n = 0
        bx0 = 99999
        bx1 = -1
        ci = 0
        while ci < nc:
            x = int(cols[ci])
            s = -1
            y = 0
            done = 0
            while y < h and done == 0:
                if int(buf[y * w + x]) >= th:
                    if s < 0:
                        s = y
                else:
                    if s >= 0:
                        L = y - s
                        e = y - 1
                        if L >= longcut:
                            if x < bx0:
                                bx0 = x
                            if x > bx1:
                                bx1 = x
                            done = 1
                        elif L >= minlen:
                            if n < maxn:
                                top = s * 256
                                if s > 0:
                                    a = int(buf[(s - 1) * w + x])
                                    b = int(buf[s * w + x])
                                    if b > a:
                                        top = (s - 1) * 256 + \
                                              ((th - a) * 256) // (b - a)
                                bot = e * 256
                                if e < h - 1:
                                    a = int(buf[e * w + x])
                                    b = int(buf[(e + 1) * w + x])
                                    if a > b:
                                        bot = e * 256 + \
                                              ((a - th) * 256) // (a - b)
                                out[n * 3] = x
                                out[n * 3 + 1] = (top + bot) >> 5
                                out[n * 3 + 2] = L
                                if L < 256:
                                    hist[L] = int(hist[L]) + 1
                                n += 1
                            done = 1
                        s = -1
                y += 1
            if done == 0 and s >= 0:
                L = h - s
                if L >= longcut:
                    if x < bx0:
                        bx0 = x
                    if x > bx1:
                        bx1 = x
                elif L >= minlen and n < maxn:
                    out[n * 3] = x
                    out[n * 3 + 1] = (s * 16 + (h - 1) * 16) >> 1
                    out[n * 3 + 2] = L
                    if L < 256:
                        hist[L] = int(hist[L]) + 1
                    n += 1
            ci += 1
        res[0] = n
        res[1] = bx0
        res[2] = bx1

    @micropython.viper
    def row_scan(buf: ptr8, w: int, h: int, th: int, minlen: int,
                 maxlen: int, x0: int, x1: int, rows: ptr32, nr: int,
                 out: ptr32, maxn: int, hist: ptr32) -> int:
        # Same rule as col_scan: first run whose width lands in
        # [minlen, maxlen) wins, scanning that row stops there -- "the
        # point from the left". maxlen is new; row_scan previously had no
        # upper width bound at all, so a too-wide spurious run (e.g. two
        # nearby teeth merged into one bright stretch) could still get in.
        n = 0
        ri = 0
        while ri < nr:
            y = int(rows[ri])
            base = y * w
            s = -1
            x = x0
            done = 0
            while x < x1 and done == 0:
                if int(buf[base + x]) >= th:
                    if s < 0:
                        s = x
                else:
                    if s >= 0:
                        L = x - s
                        e = x - 1
                        if L >= minlen and L < maxlen:
                            if n < maxn:
                                lf = s * 256
                                if s > x0:
                                    a = int(buf[base + s - 1])
                                    b = int(buf[base + s])
                                    if b > a:
                                        lf = (s - 1) * 256 + \
                                             ((th - a) * 256) // (b - a)
                                rt = e * 256
                                if e < x1 - 1:
                                    a = int(buf[base + e])
                                    b = int(buf[base + e + 1])
                                    if a > b:
                                        rt = e * 256 + \
                                             ((a - th) * 256) // (a - b)
                                out[n * 3] = y
                                out[n * 3 + 1] = (lf + rt) >> 5
                                out[n * 3 + 2] = L
                                if L < 256:
                                    hist[L] = int(hist[L]) + 1
                                n += 1
                            done = 1
                        elif L >= maxlen:
                            # too wide to be a real single crossing -- stop
                            # here rather than let a later, narrower, valid
                            # run further right get picked up out of order.
                            done = 1
                        s = -1
                x += 1
            if done == 0 and s >= 0:
                L = x1 - s
                if L >= minlen and L < maxlen and n < maxn:
                    out[n * 3] = y
                    out[n * 3 + 1] = (s * 16 + (x1 - 1) * 16) >> 1
                    out[n * 3 + 2] = L
                    if L < 256:
                        hist[L] = int(hist[L]) + 1
                    n += 1
            ri += 1
        return n

    @micropython.viper
    def hist_peak(hist: ptr32, lo: int, hi: int) -> int:
        best = 0
        bc = 0
        L = lo
        while L <= hi:
            c = int(hist[L])
            if c > bc:
                bc = c
                best = L
            L += 1
        return best

    @micropython.viper
    def hist_peak_count(hist: ptr32, lo: int, hi: int) -> int:
        bc = 0
        L = lo
        while L <= hi:
            c = int(hist[L])
            if c > bc:
                bc = c
            L += 1
        return bc

    @micropython.viper
    def lsq(arr: ptr32, n: int, lo: int, hi: int, b_fx: int, a_fx: int,
            gate: int, acc: ptr32) -> int:
        cnt = 0
        su = 0
        sv = 0
        suu = 0
        suv = 0
        i = 0
        while i < n:
            L = int(arr[i * 3 + 2])
            if L >= lo:
                if L <= hi:
                    u = int(arr[i * 3])
                    v = int(arr[i * 3 + 1])
                    keep = 1
                    if gate >= 0:
                        r = (v << 8) - (a_fx + ((b_fx * u) >> 8))
                        if r < 0:
                            r = -r
                        if r > gate:
                            keep = 0
                    if keep != 0:
                        cnt += 1
                        su += u
                        sv += v
                        suu += u * u
                        suv += u * v
            i += 1
        acc[0] = cnt
        acc[1] = su
        acc[2] = sv
        acc[3] = suu
        acc[4] = suv
        return cnt

    @micropython.viper
    def resid_hist(arr: ptr32, n: int, lo: int, hi: int, b_fx: int,
                   a_fx: int, hist: ptr32, shift: int,
                   nbins: int) -> int:
        i = 0
        while i < nbins:
            hist[i] = 0
            i += 1
        cnt = 0
        i = 0
        while i < n:
            L = int(arr[i * 3 + 2])
            if L >= lo:
                if L <= hi:
                    u = int(arr[i * 3])
                    v = int(arr[i * 3 + 1])
                    r = (v << 8) - (a_fx + ((b_fx * u) >> 8))
                    if r < 0:
                        r = -r
                    q = r >> shift
                    if q >= nbins:
                        q = nbins - 1
                    hist[q] = int(hist[q]) + 1
                    cnt += 1
            i += 1
        return cnt

    @micropython.viper
    def hist_median(hist: ptr32, nbins: int, tot: int) -> int:
        if tot <= 0:
            return 0
        half = tot >> 1
        c = 0
        i = 0
        while i < nbins:
            c += int(hist[i])
            if c > half:
                return i
            i += 1
        return nbins - 1

    @micropython.viper
    def wlsq(arr: ptr32, n: int, prm: ptr32, wtab: ptr32,
             acc: ptr32) -> int:
        lo = int(prm[0])
        hi = int(prm[1])
        b_fx = int(prm[2])
        a_fx = int(prm[3])
        gate = int(prm[4])
        uc = int(prm[5])
        vc = int(prm[6])
        nt = int(prm[7])
        sw = 0
        sdu = 0
        sdv = 0
        sduu = 0
        sduv = 0
        cnt = 0
        i = 0
        while i < n:
            L = int(arr[i * 3 + 2])
            if L >= lo:
                if L <= hi:
                    u = int(arr[i * 3])
                    du = u - uc
                    if du < 0:
                        ad = -du
                    else:
                        ad = du
                    if ad < nt:
                        w = int(wtab[ad])
                        if w > 0:
                            v = int(arr[i * 3 + 1])
                            r = (v << 8) - (a_fx + ((b_fx * u) >> 8))
                            if r < 0:
                                r = -r
                            if r <= gate:
                                dv = v - vc
                                sw += w
                                sdu += w * du
                                sdv += w * dv
                                sduu += w * du * du
                                sduv += w * du * dv
                                cnt += 1
            i += 1
        acc[0] = sw
        acc[1] = sdu
        acc[2] = sdv
        acc[3] = sduu
        acc[4] = sduv
        acc[7] = cnt
        return cnt


def P(s):
    print(s)
    time.sleep_ms(PRINT_MS)


def V(o, n, d=-1):
    a = getattr(o, n, None)
    if a is None:
        return d
    try:
        return a() if callable(a) else a
    except Exception:
        return d


def spaced(step, total):
    if step <= 1:
        return list(range(total))
    off = (total - 1 - ((total - 1) // step) * step) // 2
    return list(range(off, total, step))


def solve(acc):
    n = acc[0]
    if n < 4:
        return None
    su = acc[1]
    sv = acc[2]
    den = n * acc[3] - su * su
    if abs(den) < 1:
        return None
    bp = float(n * acc[4] - su * sv) / float(den)
    ap = (float(sv) - bp * float(su)) / float(n)
    return bp / 16.0, ap / 16.0, bp, ap


def solve_w(acc, uc, vc):
    sw = acc[0]
    if sw < 1 or acc[7] < 4:
        return None
    sdu = acc[1]
    sdv = acc[2]
    den = sw * acc[3] - sdu * sdu
    if abs(den) < 1:
        return None
    bp = float(sw * acc[4] - sdu * sdv) / float(den)
    dv = (float(sdv) - bp * float(sdu)) / float(sw)
    return bp / 16.0, ((float(vc) + dv) - bp * float(uc)) / 16.0


if not VIPER:
    P("viper unavailable, stopping.")
elif not HAVE_SENSOR:
    P("no working sensor init, stopping. Fix the block marked VERIFY.")
else:
    gc.collect()
    CO = array('i', [0] * (MAXPTS * 3))
    RO = array('i', [0] * (MAXPTS * 3))
    res = array('i', [0, 0, 0])
    acc = array('i', [0] * 8)
    prm = array('i', [0] * 8)
    mh = array('i', [0] * MAD_BINS)
    hc = array('i', [0] * 256)
    hr = array('i', [0] * 256)

    WTAB = array('i', [0] * WTAB_N)
    for d in range(WTAB_N):
        WTAB[d] = int(WMAX * math.exp(-((d / float(WSIGMA)) ** 2)) + 0.5)

    acl = spaced(ACQUIRE_STEP, W)
    arl = spaced(ACQUIRE_STEP, H)
    allc = array('i', acl)
    allr = array('i', arl)
    NAC, NAR = len(acl), len(arl)
    cl = spaced(STEP, W)
    rl = spaced(STEP, H)
    ca = array('i', cl)
    ra = array('i', rl)
    NC, NR = len(cl), len(rl)

    def fit_line(arr, n, lo, hi, iters, reject=REJECT):
        if int(lsq(arr, n, lo, hi, 0, 0, -1, acc)) < 4:
            return None
        avail = acc[0]
        s = solve(acc)
        if s is None:
            return None
        b, a, bp, ap = s
        mad = 0.0
        for _ in range(iters):
            b_fx = int(bp * 65536.0)
            a_fx = int(ap * 256.0)
            nh = int(resid_hist(arr, n, lo, hi, b_fx, a_fx, mh,
                                MAD_SHIFT, MAD_BINS))
            mad = (int(hist_median(mh, MAD_BINS, nh)) + 0.5) / 8.0
            gate = int(reject * 1.4826 * mad * 4096.0)
            if gate < 1024:
                gate = 1024
            if int(lsq(arr, n, lo, hi, b_fx, a_fx, gate, acc)) < 4:
                break
            s2 = solve(acc)
            if s2 is None:
                break
            b, a, bp, ap = s2
        if iters:
            gate = int(reject * 1.4826 * mad * 4096.0)
            if gate < 1024:
                gate = 1024
        else:
            gate = NOGATE
        return b, a, gate, avail, mad

    def fit_weighted(arr, n, lo, hi, b, a, gate, uc, wtab):
        uci = int(uc + 0.5)
        vc = int((a + b * uci) * 16.0 + 0.5)
        prm[0] = lo
        prm[1] = hi
        prm[2] = int(b * 16.0 * 65536.0)
        prm[3] = int(a * 16.0 * 256.0)
        prm[4] = gate
        prm[5] = uci
        prm[6] = vc
        prm[7] = WTAB_N
        if int(wlsq(arr, n, prm, wtab, acc)) < 4:
            return None
        return solve_w(acc, uci, vc)

    def signed_x(jx_px, mmpx):
        """Raw junction offset in px -> the new signed X, or None.

        d is how far the fitted junction sits from the camera centre:
        negative once we have driven past it, positive while it is still
        ahead. That maps straight onto the scheme:

            d in [-30, 0]  -> just left it, X = d      (0 down to -30)
            d in (-50, -30)-> too far from either      -> None
            d in [-80,-50] -> next one is within 30,   X = d + 80 (+30 -> 0)

        Everything is measured against the junction we actually fitted,
        so no dead reckoning and no accumulated drift.
        """
        d = (jx_px - W * 0.5) * mmpx
        # fold into (-JUNCTION_SPACING, 0] so "the junction behind us"
        # is always the reference
        while d > 0.0:
            d -= JUNCTION_SPACING_MM
        while d <= -JUNCTION_SPACING_MM:
            d += JUNCTION_SPACING_MM
        if d >= -X_RANGE_MM:
            x = d                                   # leaving: 0 -> -30
        elif d <= -(JUNCTION_SPACING_MM - X_RANGE_MM):
            x = d + JUNCTION_SPACING_MM             # approaching: +30 -> 0
        else:
            return None                             # null zone
        return x * X_DIRECTION

    def mm_scale(t):
        if MM_PER_PIXEL:
            return MM_PER_PIXEL
        return BAR_WIDTH_MM / t if t else 0.0

    def estimate(n_c, n_r, tc, tr):
        hlo, hhi = int(tc * THICK_LO), int(tc * THICK_HI)
        vlo, vhi = int(tr * THICK_LO), int(tr * THICK_HI)
        fh = fit_line(CO, n_c, hlo, hhi, ITERS)
        fv = fit_line(RO, n_r, vlo, vhi, ITERS)
        if fh is None or fv is None:
            return None
        if fh[4] > MAX_JOINT_MAD_PX or fv[4] > MAX_JOINT_MAD_PX:
            return None      # blurred / contaminated -- let the
                             # single-bar fallback handle this frame
        den = 1.0 - fh[0] * fv[0]
        if abs(den) < 1e-9:
            return None
        ux = (fv[1] + fv[0] * fh[1]) / den
        uy = fh[1] + fh[0] * ux
        jx, jy = ux, uy
        bh, ah = fh[0], fh[1]
        bv, av = fv[0], fv[1]
        wh = fit_weighted(CO, n_c, hlo, hhi, fh[0], fh[1], fh[2], ux, WTAB)
        wv = fit_weighted(RO, n_r, vlo, vhi, fv[0], fv[1], fv[2], uy, WTAB)
        if wh and wv:
            den = 1.0 - wh[0] * wv[0]
            if abs(den) > 1e-9:
                jx = (wv[1] + wv[0] * wh[1]) / den
                jy = wh[1] + wh[0] * jx
                bh, ah = wh[0], wh[1]
                bv, av = wv[0], wv[1]
        thh = math.degrees(math.atan(fh[0]))
        thv = -math.degrees(math.atan(fv[0]))
        return jx, jy, (thh + thv) * 0.5, thh - thv, bh, ah, bv, av

    def tilt_probe(n_c, tc, jx):
        """Least-squares slope of the horizontal bar on each side of
        the junction, from the points already in CO. Pure python and
        only run once a second, so it costs nothing in the hot loop."""
        lo, hi = int(tc * THICK_LO), int(tc * THICK_HI)
        out = []
        for side in (0, 1):
            n = 0
            su = sv = suu = suv = 0.0
            i = 0
            while i < n_c:
                L = CO[i * 3 + 2]
                if lo <= L <= hi:
                    u = CO[i * 3]
                    ok_side = (u < jx - 20) if side == 0 else (u > jx + 20)
                    if ok_side:
                        v = CO[i * 3 + 1] / 16.0
                        n += 1
                        su += u
                        sv += v
                        suu += u * u
                        suv += u * v
                i += 1
            if n < 6:
                out.append((None, None, n))
                continue
            den = n * suu - su * su
            if abs(den) < 1e-6:
                out.append((None, None, n))
                continue
            b = (n * suv - su * sv) / den
            a = (sv - b * su) / n
            out.append((math.degrees(math.atan(b)), a + b * jx, n))
        return out

    def plausible(x_new):
        """Reject a reading the camera could not physically have moved
        to since the last one. Compared modulo the cell spacing, so a
        genuine wrap (X running down to 0 and resetting to 80) is a
        zero-length move here and never flagged."""
        global last_X
        if not MAX_STEP_MM or last_X is None:
            last_X = x_new
            return True
        d = x_new - last_X
        half = JUNCTION_SPACING_MM * 0.5
        while d < -half:
            d += JUNCTION_SPACING_MM
        while d > half:
            d -= JUNCTION_SPACING_MM
        if d < 0:
            d = -d
        if d > MAX_STEP_MM:
            return False
        last_X = x_new
        return True

    def clear_hist():
        for k in range(256):
            hc[k] = 0
            hr[k] = 0

    def find_mini_gap(n_c, lo, hi, step, mmpx):
        # DEAD CODE as of the signed-X scheme. Nothing calls this any more:
        # the 40 mm void it used to turn into an X is now the null zone,
        # where the honest answer is "no X". Left in place rather than
        # deleted so the rejection history stays readable; delete freely.
        """Locate the empty floor between two horizontal bar segments.

        Returns (x_left_edge, x_right_edge, width_px, n_candidates)
        or None.

        Method: take the x of every thickness-consistent point already
        collected in CO by the column scan, split that into contiguous
        RUNS (a break = a gap wider than one or two missed samples),
        and treat each break between two substantial runs as a
        candidate void.

        Why not just 'the largest jump' (v3): with STEP=5 sampling a
        single dropped column is already a 10 px jump, and 10 px at
        ~0.43 mm/px is 4.3 mm, which passed the old 4.0 mm test. That
        put a mini-junction, with a bogus X, anywhere a column happened
        to drop out along a perfectly normal bar.

        Why nearest-to-centre and not widest: with a wide field of view
        two real voids can be in frame at once (one either side of a
        junction). They are 80 mm apart, so choosing the wrong one is
        an 80 mm error. The one nearest the image centre is the one the
        camera is actually over, and it keeps X continuous as you
        drive. n_candidates > 1 is reported so the caller can flag the
        reading as ambiguous.
        """
        xs = []
        last = -1
        i = 0
        while i < n_c:
            L = CO[i * 3 + 2]
            if lo <= L <= hi:
                x = CO[i * 3]
                if x != last:          # dedupe: a column may yield 2 runs
                    xs.append(x)
                    last = x
            i += 1
        if len(xs) < 2 * MINI_SIDE_MIN_PTS:
            return None
        xs.sort()

        # a break must be wider than a couple of missed samples AND
        # wider than an absolute pixel floor. The px floor matters
        # because mmpx is unreliable until MM_PER_PIXEL is calibrated.
        min_px = MINI_GAP_MIN_PX
        if step * MINI_GAP_STEP_MULT > min_px:
            min_px = int(step * MINI_GAP_STEP_MULT)

        # split into contiguous runs
        runs = []                       # (start_idx, end_idx) inclusive
        s = 0
        for i in range(len(xs) - 1):
            if xs[i + 1] - xs[i] >= min_px:
                runs.append((s, i))
                s = i + 1
        runs.append((s, len(xs) - 1))
        if len(runs) < 2:
            return None

        def solid(r):
            a0, a1 = r
            if (a1 - a0 + 1) < MINI_SIDE_MIN_PTS:
                return False
            if (xs[a1] - xs[a0]) < MINI_SIDE_MIN_SPAN_PX:
                return False
            return True

        cx = W * 0.5
        best = None
        best_d = 1e9
        n_cand = 0
        for k in range(len(runs) - 1):
            if not solid(runs[k]) or not solid(runs[k + 1]):
                continue
            gx0 = xs[runs[k][1]]
            gx1 = xs[runs[k + 1][0]]
            gw = gx1 - gx0
            if mmpx > 0 and gw * mmpx < MINI_GAP_MIN_MM:
                continue
            # reject the gaps that flank a junction: those have a short
            # stub on one side. The midpoint void has long bar either
            # side. Scale-free, so it survives a wrong mm calibration.
            ls = xs[runs[k][1]] - xs[runs[k][0]]
            rs = xs[runs[k + 1][1]] - xs[runs[k + 1][0]]
            short = ls if ls < rs else rs
            need = MINI_SIDE_MIN_RATIO * gw
            if need > MINI_SIDE_ABS_PX:
                need = MINI_SIDE_ABS_PX
            if short < need:
                continue
            n_cand += 1
            d = (gx0 + gx1) * 0.5 - cx
            if d < 0:
                d = -d
            if d < best_d:
                best_d = d
                best = (gx0, gx1, gw)
        if best is None:
            return None
        return best[0], best[1], best[2], n_cand

    def do_scan(buf, th, cols, nc, rows, nr, x0, x1):
        clear_hist()
        col_scan(buf, W, H, th, MIN_LEN, LONG_CUT, cols, nc, CO, MAXPTS,
                 res, hc)
        n_c, bx0, bx1 = res[0], res[1], res[2]
        n_r = int(row_scan(buf, W, H, th, MIN_LEN, LONG_CUT, x0, x1, rows, nr, RO,
                           MAXPTS, hr))
        tc = int(hist_peak(hc, MIN_LEN, LONG_CUT - 1))
        tr = int(hist_peak(hr, MIN_LEN, LONG_CUT - 1))
        tc_cnt = int(hist_peak_count(hc, MIN_LEN, LONG_CUT - 1))
        tr_cnt = int(hist_peak_count(hr, MIN_LEN, LONG_CUT - 1))
        # Sanity anchor -- see NOMINAL_THICK_PX above. A measured peak more
        # than 2x off the known real thickness is almost certainly locked
        # onto teeth/texture, not the bar. tc_cnt/tr_cnt (how many dots
        # supported the peak) are left untouched -- only the width value
        # itself is corrected.
        if tc < (NOMINAL_THICK_PX >> 1) or tc > (NOMINAL_THICK_PX << 1):
            tc = NOMINAL_THICK_PX
        if tr < (NOMINAL_THICK_PX >> 1) or tr > (NOMINAL_THICK_PX << 1):
            tr = NOMINAL_THICK_PX
        return n_c, n_r, tc, tr, bx0, bx1, tc_cnt, tr_cnt

    band = None
    frame_id = 0

    S = [0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    Wn = [0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0]  # [7] = frames with a real X
    last_partial = None
    last_raw_jx = None
    last_gap = None       # (gap_px, gap_mm, n_c) for the debug line
    last_X = None         # last accepted X, for the plausibility check
    n_rejected = 0        # how many readings the check threw out

    def acc_add(A, jx, jy, thd):
        # jx may be None -- the null zone between junctions is a valid
        # state, not a dropped frame. Y and theta keep accumulating; only
        # the X sums skip, counted separately in A[7].
        A[0] += 1
        if jx is not None:
            A[1] += jx
            A[2] += jx * jx
            A[7] += 1
        A[3] += jy
        A[4] += jy * jy
        A[5] += thd
        A[6] += thd * thd

    def acc_clear(A):
        for k in range(8):
            A[k] = 0 if k in (0, 7) else 0.0

    def acc_stats(A):
        n = A[0]
        if n < 1:
            return None
        nx = A[7]
        my = A[3] / n
        mt = A[5] / n
        vy = A[4] / n - my * my
        vt = A[6] / n - mt * mt
        if nx < 1:
            mx = None           # spent the whole window in the null zone
            sx = 0.0
        else:
            mx = A[1] / nx
            vx = A[2] / nx - mx * mx
            sx = math.sqrt(vx) if vx > 0 else 0.0
        return (mx, sx,
                my, math.sqrt(vy) if vy > 0 else 0.0,
                mt, math.sqrt(vt) if vt > 0 else 0.0)

    def estimate_h_only(n_c, tc):
        hlo, hhi = int(tc * THICK_LO), int(tc * THICK_HI)
        fh = fit_line(CO, n_c, hlo, hhi, ITERS)
        if fh is None:
            return None
        if fh[4] > MAX_FIT_MAD_PX:
            return None          # points are scattered, not a bar
        b, a = fh[0], fh[1]
        y_at_center = a + b * (W * 0.5)
        theta = math.degrees(math.atan(b))
        return y_at_center, theta, b, a

    def estimate_v_only(n_r, tr):
        vlo, vhi = int(tr * THICK_LO), int(tr * THICK_HI)
        fv = fit_line(RO, n_r, vlo, vhi, ITERS)
        if fv is None:
            return None
        if fv[4] > MAX_FIT_MAD_PX:
            return None
        b, a = fv[0], fv[1]
        x_at_center = a + b * (H * 0.5)
        theta = -math.degrees(math.atan(b))
        return x_at_center, theta, b, a

    clock = time.clock()
    t_win = time.ticks_ms()
    th = 0
    T = [0, 0, 0, 0, 0, 0, 0]   # cap, gray, otsu, scan+fit, draw, flush, gc
    T_n = 0
    T2 = [0, 0, 0, 0, 0]       # TURN: cap, gray, grad4_angle, draw+flush, stage2
    T2_n = 0

    _uart = None
    if HOST_UART_ID is not None:
        try:
            from machine import UART
            _uart = UART(HOST_UART_ID, HOST_BAUD)
        except Exception as _e:
            print("host UART init failed: %s -- running in bench mode" % _e)
    SM = TurnSM(HostLink(_uart, sim_cmds=SIM_CMDS), W, H)
    print("state machine: %s" % ("UART host" if _uart else "bench mode"))

    _btn = None
    _btn_last_ms = 0
    _btn_next_dir = "L"
    if BUTTON_PIN is not None:
        from machine import Pin
        _btn = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_UP)
        print("hand-test button armed on %s (active low)" % BUTTON_PIN)

    while True:
        clock.tick()
        if _btn is not None and _btn.value() == 0:
            _now = time.ticks_ms()
            if time.ticks_diff(_now, _btn_last_ms) > BUTTON_DEBOUNCE_MS:
                _btn_last_ms = _now
                if SM.state == DRIVE:
                    SM.link.inject("TURN " + _btn_next_dir)
                    _btn_next_dir = "R" if _btn_next_dir == "L" else "L"
                else:
                    SM.link.inject("DRIVE")
        try:
            _t0 = time.ticks_us()
            img = csi0.snapshot()
            _t1 = time.ticks_us()
            # gimg is what every scan/fit below reads: 1 byte per pixel.
            # img stays RGB565 and only ever gets drawn on.
            gimg = TO_GRAY(img) if TO_GRAY else img
            buf = gimg.bytearray()
            _t2 = time.ticks_us()

            _now = time.ticks_ms()

            # ---- ANGLE GAUGE: drawn FIRST, every single frame -----------
            # It has to go before anything else draws, because each drive
            # branch below ends with its own csi0.flush(). Anything drawn
            # after that lands on a frame already sent, which is why the
            # gauge kept disappearing. SM.heading() gives the live value
            # while the rotation layer tracks and the last known value
            # otherwise, so the wheel is permanent and never blinks.
            if DRAW:
                _ph, _om = SM.heading()
                hud.draw_gauge(img, W, H, _ph, _om)

            SM.poll_cmd(frame_id, _now)           # hook 1: host commands
            _fix = None
            if not SM.alg1_active():    # never true in layer mode
                # ---- Phase B: TURN. Algorithm 2 only. No Otsu, no scan.
                _t3 = time.ticks_us()
                SM.turn_frame(buf, _now)          # hook 2 -- Alg 2's own cost
                _t4 = time.ticks_us()
                if DRAW:
                    _tt = SM.turn
                    hud.draw_turn(img, W, H,
                                  ("DRIVE", "TURN_INIT", "TURNING",
                                   "REACQUIRE", "FAULT")[SM.state],
                                  _tt["r_prev"] if _tt else 0.0,
                                  _tt["phi"] if _tt else 0.0,
                                  _tt["omega"] if _tt else 0.0,
                                  tier=(SM.fit["tier"] if SM.fit else 0),
                                  fit=SM.fit)
                if DRAW or FLUSH:
                    csi0.flush()
                _t5 = time.ticks_us()
                if (SM.fit is not None and SM.fit["junction"] is not None
                        and (frame_id % 60) == 0):
                    _al, _be = SM.fit["junction"]
                    print("   stage2  X %+6.1fmm Y %+6.1fmm T %+6.2fdeg "
                          "mad %.2f n %d"
                          % (_al * MM_PER_PIXEL, _be * MM_PER_PIXEL,
                             SM.fit["theta"], SM.fit["mad"], SM.fit["n"]))
                if SM.state == 1 or SM.state == 2:   # TURN_INIT / TURNING only
                    T2[0] += time.ticks_diff(_t1, _t0)      # capture
                    T2[1] += time.ticks_diff(_t2, _t1)      # gray
                    T2[4] += SM.stage2_us                   # Stage 2 (tilted scan)
                    T2[2] += (time.ticks_diff(_t4, _t3)
                              - SM.stage2_us)               # grad4_angle alone
                    T2[3] += time.ticks_diff(_t5, _t4)      # draw+flush
                    SM.stage2_us = 0
                    T2_n += 1
            else:
                # ---- Phase A: DRIVE / REACQUIRE. Algorithm 1, unchanged.
                if SM.reseed_pending:
                    # back from a turn: the image axes now look at the
                    # other floor tape. Everything axis-specific goes.
                    band = None
                    last_X = None          # one frame without MAX_STEP gate
                    last_partial = None
                    last_raw_jx = None
                    last_gap = None
                    acc_clear(Wn)
                    n_rejected = 0
                    SM.reseed_pending = False
                if SM.force_full_scan():
                    band = None            # full ACQUIRE_STEP scan each frame
                if OTSU_EVERY <= 1 or frame_id % OTSU_EVERY == 0 or th == 0:
                    sm = gimg.copy(x_scale=OTSU_SCALE, y_scale=OTSU_SCALE)
                    th = int(V(sm.get_histogram().get_threshold(), "value"))
                    del sm
                _t3 = time.ticks_us()

                status = "track"
                col_step = 1
                if band is None:
                    status = "acquire"
                    (n_c, n_r, tc, tr, bx0, bx1,
                     tc_cnt, tr_cnt) = do_scan(buf, th, allc, NAC, allr, NAR, 0, W)
                    nc_used, nr_used = NAC, NAR
                    col_step = 1
                else:
                    x0, x1 = band
                    (n_c, n_r, tc, tr, bx0, bx1,
                     tc_cnt, tr_cnt) = do_scan(buf, th, ca, NC, ra, NR, x0, x1)
                    nc_used, nr_used = NC, NR
                    col_step = STEP

                    # BUG 3 FIX. Between two junctions bx1 is -1 by
                    # definition, so the old unconditional re-acquire ran a
                    # second, full-width scan on EVERY frame of the mini
                    # region. Only escalate if the cheap scan cannot even
                    # support the horizontal-only fallback -- or on a slow
                    # cadence, to notice a vertical bar coming back.
                    h_usable = (tc >= MIN_LEN and n_c >= MIN_PTS
                                and n_c >= nc_used * COVERAGE_FRAC)
                    need_full = (bx1 < 0 or n_c < MIN_PTS or n_r < MIN_PTS)
                    if need_full and (not h_usable
                                      or frame_id % REACQUIRE_MINI_EVERY == 0):
                        status = "reacquire"
                        (n_c, n_r, tc, tr, bx0, bx1,
                         tc_cnt, tr_cnt) = do_scan(buf, th, allc, NAC,
                                                   allr, NAR, 0, W)
                        nc_used, nr_used = NAC, NAR
                        col_step = 1
                    elif (not need_full) and frame_id % REACQUIRE_EVERY == 0:
                        (n_c2, n_r2, tc2, tr2, bx02, bx12,
                         tc_cnt2, tr_cnt2) = do_scan(buf, th, allc, NAC,
                                                     allr, NAR, 0, W)
                        if bx12 >= 0:
                            n_c, n_r, tc, tr, bx0, bx1 = (n_c2, n_r2, tc2,
                                                          tr2, bx02, bx12)
                            tc_cnt, tr_cnt = tc_cnt2, tr_cnt2
                            nc_used, nr_used = NAC, NAR
                            col_step = 1
                            status = "periodic"

                # Full-junction validity: the original, proven check.
                _t4 = time.ticks_us()
                # Two gates added in v6, both measured off middle4.mp4,
                # where the junction path was firing on lens glare at the
                # frame edge ~47% of the time and reporting a confident,
                # wrong X. That -- not the mini logic -- was the flicker.
                # Both gates are physical, not tuned: a vertical bar spans
                # several columns, and it is the same tape as the
                # horizontal bar so it must be about as wide.
                ok = (bx1 >= 0 and (bx1 - bx0) >= MIN_V_SPAN_PX
                      and tc >= MIN_LEN and tr >= MIN_LEN
                      and n_c >= MIN_PTS and n_r >= MIN_PTS
                      and tr >= tc * THICK_MATCH_FRAC)

                out = None
                if ok:
                    band = (max(0, bx0 - EXTRA), min(W, bx1 + EXTRA))
                    out = estimate(n_c, n_r, tc, tr)
                    if out is None:
                        ok = False

                if ok:
                    # ---- tier 1: full junction (both bars). UNCHANGED.
                    # BUG 1 FIX: the stray `y_at_mid = a + b * mid_px` /
                    # draw_cross block that used to sit here referred to
                    # names that only exist in the mini branch, so this
                    # whole branch raised NameError on every junction.
                    jx, jy, thd, hv, bh, ah, bv, av = out
                    mmpx = mm_scale(tc)
                    # same quantity the mini path reports: distance to the
                    # junction AHEAD. If the junction we just fitted is
                    # behind us, the one ahead is a full cell further on.
                    X_mm = signed_x(jx, mmpx)      # may be None in the
                                                   # null zone -- that is a
                                                   # valid answer, not a
                                                   # failure.
                    Y_mm = (jy - H * 0.5) * mmpx
                    sus = X_mm is not None and not plausible(X_mm)
                    if sus:
                        n_rejected += 1
                    else:
                        acc_add(Wn, X_mm, Y_mm, thd)
                        _fix = (1, X_mm, Y_mm, thd)
                    last_partial = None
                    last_gap = None
                    last_raw_jx = jx
                    if TILT_DEBUG and frame_id % 60 == 0:
                        _pl, _pr = tilt_probe(n_c, tc, jx)
                        if _pl[0] is not None and _pr[0] is not None:
                            print("  TILT  left %+.2fdeg (n%d)  right %+.2fdeg"
                                  " (n%d)  combined %+.2fdeg   step %+.1fpx"
                                  % (_pl[0], _pl[2], _pr[0], _pr[2], thd,
                                     _pr[1] - _pl[1]))
                        else:
                            print("  TILT  not enough points one side:"
                                  " left n%d  right n%d" % (_pl[2], _pr[2]))
                    if DRAW:

                        img.draw_line((0, int(ah + 0.5),
                                   W - 1, int(ah + bh * (W - 1) + 0.5)),
                                  color=COLOR_X)           # x line
                    if DRAW:

                        img.draw_line((int(av + 0.5), 0,
                                   int(av + bv * (H - 1) + 0.5), H - 1),
                                  color=COLOR_Y)           # y line
                    if DRAW:

                        img.draw_cross((int(jx + 0.5), int(jy + 0.5)),
                                   color=(255, 255, 255), size=8)
                    if DRAW:

                        # X_mm is None inside the null zone. "%+.1f" % None
                        # raises, the frame try/except swallows it as a
                        # FRAME ERROR, and the whole frame is lost -- which
                        # is why X and Y disappeared from the overlay there.
                        _xs = ("X  --  " if X_mm is None
                               else "X%+.1f" % X_mm)
                        img.draw_string((2, 2),
                                    "%s Y%+.1f T%+.1f" % (_xs, Y_mm, thd),
                                    color=(255, 255, 255))
                    if DRAW or FLUSH:
                        csi0.flush()
                else:
                    band = None
                    # tc_cnt/THICKNESS_FRAC deliberately NOT used here any
                    # more -- see MAX_FIT_MAD_PX. The fit residual test
                    # inside estimate_h_only does this job properly.
                    h_ok_fb = (tc >= MIN_LEN and n_c >= MIN_PTS
                               and n_c >= nc_used * COVERAGE_FRAC)
                    v_ok_fb = (bx1 >= 0 and tr >= MIN_LEN and n_r >= MIN_PTS
                               and n_r >= nr_used * COVERAGE_FRAC)
                    if h_ok_fb:
                        # ---- tier 2/3: horizontal bar only.
                        r = estimate_h_only(n_c, tc)
                        if r is not None:
                            # keep the cheap STEP-spaced scan alive
                            band = (0, W)
                            y_px, thd, b, a = r
                            mmpx = mm_scale(tc)
                            Y_mm = (y_px - H * 0.5) * mmpx
                            # the "x line": drawn in BOTH sub-cases, since
                            # the horizontal fit is what we do have.
                            if DRAW:

                                img.draw_line((0, int(a + 0.5),
                                           W - 1, int(a + b * (W - 1) + 0.5)),
                                          color=COLOR_X)
                            # NO vertical line is drawn here, by design --
                            # there is no vertical bar to draw.

                            # ---- mini junction REMOVED -------------------
                            # It existed to synthesise an X from the 40 mm
                            # void when no vertical bar was visible. Under
                            # the new scheme that region IS the null zone --
                            # X is legitimately absent there, so inventing
                            # one is exactly the wrong thing to do. The
                            # horizontal (X) bar is still fitted and still
                            # drawn above; only the number is withheld.
                            last_partial = ("Y", Y_mm, thd)
                            last_gap = None
                            if DRAW:
                                # The X line is ALREADY drawn a few lines
                                # above from (a, b). I added a second draw
                                # here using ah/bh -- names that do not
                                # exist in this scope -- so every null-zone
                                # frame raised NameError, got swallowed by
                                # the frame try/except, and was lost whole.
                                # That is why the line disappeared. Removed.
                                img.draw_string((2, 2),
                                                "X  --   Y%+.1f T%+.1f"
                                                % (Y_mm, thd),
                                                color=(255, 255, 255))
                            if DRAW or FLUSH:
                                csi0.flush()
                        else:
                            last_partial = None
                    elif v_ok_fb:
                        r = estimate_v_only(n_r, tr)
                        if r is not None:
                            x_px, thd, b, a = r
                            mmpx = mm_scale(tr)
                            X_mm = (x_px - W * 0.5) * mmpx
                            last_partial = ("X", X_mm, thd)
                            last_gap = None
                            if DRAW:

                                img.draw_line((int(a + 0.5), 0,
                                           int(a + b * (H - 1) + 0.5), H - 1),
                                          color=COLOR_Y)
                            if DRAW:

                                # same None guard as the junction branch
                                _xs = ("X  --  " if X_mm is None
                                       else "X%+.1f" % X_mm)
                                img.draw_string((2, 2),
                                "%s Y  --   T%+.1f"
                                % (_xs, thd), color=(255, 255, 255))
                            if DRAW or FLUSH:
                                csi0.flush()
                        else:
                            last_partial = None
                    else:
                        last_partial = None
                        last_gap = None
                _t5 = time.ticks_us()
                SM.drive_result(_fix, _now)       # hook 3

                # ---- ROTATION LAYER ------------------------------------
                # Drive above has already produced X, Y and theta and they
                # are what gets reported, always. This only ADDS rotation
                # tracking when we are near a junction and slow. Nothing is
                # replaced, so nothing flickers.
                if SM.layer_update(_fix, _now):
                    SM.turn_frame(buf, _now)

            T[0] += time.ticks_diff(_t1, _t0)
            T[1] += time.ticks_diff(_t2, _t1)
            T[2] += time.ticks_diff(_t3, _t2)
            T[3] += time.ticks_diff(_t4, _t3)
            T[4] += time.ticks_diff(_t5, _t4)
            T_n += 1
        except KeyboardInterrupt:
            # The IDE's STOP button. Landing mid-function is normal.
            print("stopped by user.")
            break
        except Exception as e:
            print("FRAME ERROR: %s: %s   frame=%d  mem_free=%d"
                  % (type(e).__name__, e, frame_id, gc.mem_free()))
            out = None
            band = None
            gc.collect()

        frame_id += 1

        now = time.ticks_ms()
        wdt = time.ticks_diff(now, t_win)
        if wdt >= SUMMARY_MS:
            st = acc_stats(Wn)
            wfps = clock.fps()
            if st is not None:
                # st[0] is None while we are in the null zone between
                # junctions -- print "X --" rather than a made-up number.
                xtxt = ("X   --   " if st[0] is None
                        else "X %+6.1fmm" % st[0])
                print("%s  Y %+6.1fmm  T %+6.2fdeg   otsu %d"
                      "   fps %.1f   mem_free %d   raw_jx %.1fpx"
                      % (xtxt, st[2], st[4], th, wfps, gc.mem_free(),
                         last_raw_jx if last_raw_jx is not None else -1))
            elif last_partial is not None:
                if False:
                    pass
                else:
                    kind, val, thd = last_partial
                    other = "Y" if kind == "X" else "X"
                    print("%s %+6.1fmm  %s:--   T %+6.2fdeg   otsu %d"
                          "   fps %.1f   mem_free %d  (single bar only)"
                          % (kind, val, other, thd, th, wfps,
                             gc.mem_free()))
            else:
                print("no marker   otsu %d   fps %.1f   mem_free %d   state %s"
                      % (th, wfps, gc.mem_free(),
                         ("DRIVE","TURN_INIT","TURNING","REACQUIRE","FAULT")[SM.state]))
            if n_rejected:
                print("   %d reading(s) rejected as physically impossible"
                      " (>%.0fmm since the last fix) -- see MAX_STEP_MM"
                      % (n_rejected, MAX_STEP_MM))
                n_rejected = 0
            if PERF and T_n:
                _tot = (T[0] + T[1] + T[2] + T[3] + T[4] + T[6]) // T_n
                _camtot = _tot - (T[4] // T_n)      # minus draw+flush
                print("   us/frame  capture %d  gray %d  otsu %d"
                      "  scan+fit %d  draw+flush %d  gc %d   total %d"
                      % (T[0] // T_n, T[1] // T_n, T[2] // T_n,
                         T[3] // T_n, T[4] // T_n, T[6] // T_n, _tot))
                print("      -> %.1f fps as measured (IDE attached), "
                      "%.1f fps CAM  <- what the robot actually gets, "
                      "draw+flush excluded"
                      % (1e6 / _tot if _tot else 0,
                         1e6 / _camtot if _camtot else 0))
                for _k in range(7):
                    T[_k] = 0
                T_n = 0
            if PERF and T2_n:
                _tt = (T2[0] + T2[1] + T2[2] + T2[3] + T2[4]) // T2_n
                _ttcam = _tt - (T2[3] // T2_n)      # minus draw+flush
                print("   TURN us/frame  capture %d  gray %d  grad4 %d"
                      "  stage2 %d  draw+flush %d   total %d  (n=%d)"
                      % (T2[0] // T2_n, T2[1] // T2_n, T2[2] // T2_n,
                         T2[4] // T2_n, T2[3] // T2_n, _tt, T2_n))
                print("      -> %.1f fps as measured (IDE attached), "
                      "%.1f fps CAM  <- what the robot actually gets, "
                      "draw+flush excluded"
                      % (1e6 / _tt if _tt else 0,
                         1e6 / _ttcam if _ttcam else 0))
                for _k in range(5):
                    T2[_k] = 0
                T2_n = 0
            acc_clear(Wn)
            t_win = now

        if GC_EVERY <= 1 or frame_id % GC_EVERY == 0:
            _g0 = time.ticks_us()
            gc.collect()
            T[6] += time.ticks_diff(time.ticks_us(), _g0)
