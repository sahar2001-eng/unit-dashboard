# NOTES.md -- measurement history and settled facts for the grid camera

This file exists so main.py can stay readable. Nothing here is needed to
run the code; all of it was needed to arrive at it.

## Settled facts -- do not re-derive

- Board: OpenMV AE3, PAG7936 global-shutter sensor, 320x200 grayscale.
- Exposure only works on the development firmware build 631681e5ac. On
  release v5.0.0 `auto_exposure(False, exposure_us=N)` accepts N and silently
  discards it. `csi0.framerate(240)` must be called before `auto_exposure`.
- Exposure cannot exceed one frame period. At 240 fps that is 4167 us; the
  sensor was measured actually running at 120 fps (8328 us). Asking for more
  comes back clamped, silently. Use GAIN_DB for brightness.
- MM_PER_PIXEL = 0.244. Measured 2026-09-10 with a 71.5 mm reference (iPhone
  17 width, Apple spec): 293.0 px and 292.8 px in two shots at different
  floor positions. The old 0.223 was 9% low; the repo's 0.251 is 3% high.
- Junction pitch 80 mm centre to centre. At 0.244 mm/px that is 328 px; the
  frame is 320 px wide, so at most one junction is in view at a time.
- Never estimate speed by differentiating X (1.8 mm of noise per frame at
  130 fps is 324 mm/s of noise). Never trigger on angle rate (0.18 deg noise
  at 130 fps is 23 deg/s). Position hysteresis instead.
- Draw before `csi0.flush()`: anything drawn after is not displayed.
- Overlay + flush costs ~12.6 ms per frame on the AE3 -- more than the whole
  algorithm. DRAW = 0 on the robot.
- `.value` on the threshold object is an attribute on this firmware, not a
  method: read it through `_get(obj, "value")`, never `.value()`.
- MicroPython small ints are 31-bit: a literal like 2147483647 inside a
  viper function is a boxed object and viper refuses to assign an int to it.
- Anything inside a `@micropython.viper` function is only type-checked on
  the board. The PC harness cannot see those errors.

## Measured on the AE3 (BENCH = 1, still frames, 2026-09-10)

| stage                                   | us      |
|-----------------------------------------|---------|
| capture (snapshot blocks for)           | 3963    |
| otsu (downscaled, every 3rd frame)      | 1195    |
| scan, acquiring (step 2, full width)    | 7897    |
| scan, tracked (step 5, banded)          | 2921    |
| fit + weighted fit                      | 2202    |
| grad4_angle (step 5 tap 3)              | 2192    |
| stage2 fit_frame (step 8/300/145)       | 9103    |
| overlay draw                            | 12629   |
| flush                                   | 11      |

Drive tracked ~105 fps, drive acquiring ~69 fps, turn ~93 fps, all with
DRAW = 0. Noise while still: drive X sd 0.002 mm, Y 0.001 mm, THETA 0.03
deg; grad4 0.05 deg; Stage 2 THETA 0.16 deg.

## Rotation accuracy (turn_sm.py v2)

Measured against the raw unwrapped angle on the code's own per-frame log,
rotation_wrong.mp4: worst error while rotating 7.5 deg -> 0.76 deg; while
still +0.17 / -0.25 -> +0.02, sd 0.14. The four changes: the alpha-beta
filter measures the raw accumulated angle (anchored) instead of its own
previous output; gains .85/.30; phi seeded from the measured tilt so 90.0
means square; no heading carry into a new turn. Stage 2 is aimed from the
continuous angle so its two tape families cannot swap at 45 deg.

Drive's theta and Algorithm 2 disagree on the absolute tilt by ~2 deg sd
(15.8 worst). A protractor test at 22.5 deg put every method within 0.2-3
deg of the target, inside the 5 deg SEED_THETA0_TOL guard. One hand
rotation cannot separate "the rig was off by a degree" from "this method is
biased by a degree", so the guard stays and the seed source stays drive.

## Known costs and open items

- The col_scan `got` fix (keep scanning a column for a long run after taking
  its first short run) reads 1.4-1.6x more rows per column on every drive
  frame. Worth ~10 fps. A cap of ~60 extra rows would recover most of it.
  Not applied.
- The acquiring path (7897 us) is the expensive one; it only runs with no
  lock. If it shows up often in real driving, that is the next target.
- grad4's angle shifts by up to 3 deg with the sampling stride (step 3/5/7/8
  on identical frames). Deterministic, from which pixels the stride lands
  on. Dithering the phase would average it out. Not applied.
- STAGE2_HOLD re-displays a fit up to 12 frames old while the robot is
  still moving into a junction: up to 14 mm of X step on rotatiom.mp4.
  Display only, pre-existing.
- The +25.7 deg gauge-against-a-square-grid screenshot was never reproduced.
  The heading carry that most likely caused it is removed.

## Change history carried over from the old file header

```
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
```

## Constants -- the full reasoning behind each value

**`MM_PER_PIXEL = 0.244`**  
MEASURED 2026-09-10: a 71.5 mm reference (iPhone 17 width, Apple spec) spanned 293.0 and 292.8 px in two shots at different floor positions -> 0.2440 and 0.2442. Replaces 0.223, which was 9% low. Every mm number in this file scales with this one constant.

**`MAX_STEP_MM = 30.0`**  
physically impossible jump filter. At the loop rate the camera cannot move further than this between frames: measured on fast_ramp1, hand-waving at up to 1.56 m/s gave a median 9.8 mm and a maximum 26.0 mm of travel per frame. A robot on wheels will be well under that. Anything larger is a bad fit, not motion. Compared modulo the cell spacing so a legitimate wrap is never flagged. Set to 0 to disable.

**`X_DIRECTION = +1`**  
flip to -1 if a camera is mounted 180 deg from the other, so "leaving" and "approaching" swap. Applied to the sign of X only.

**`JUNCTION_SPACING_MM = 80.0`**  
centre-to-centre. X below is reported as the distance to the NEXT junction ahead, in (0, 80]. Any grid position has to wrap once per cell; this puts the wrap at the junction itself -- a fast, precisely detected transit -- instead of at the cell midpoint. v5 had it at the midpoint, which is exactly where the camera lingers, so X flipped between -40 and +40 frame to frame. Measured on both videos: worst frame-to-frame jump 79.2/79.8mm before, 2.5/2.9mm after.

**`TILT_DEBUG = 0`**  
once a second, when a full junction is in view, fit the horizontal bar SEPARATELY on each side of the junction and print both slopes. If the two sides disagree, the tape itself is not collinear and no fit can be right. If they agree with each other but not with the combined fit, something near the junction (the round centre dot) is pulling it. Different causes, different fixes -- this tells them apart without a video.

**`MIN_V_SPAN_PX = 6`**  
a real vertical bar covers a RANGE of columns. Measured on middle4.mp4: the false detections were 1 and 5 columns wide (lens glare at the frame edge); every real junction was 9-28 columns wide.

**`THICK_MATCH_FRAC = 0.55`**  
the vertical bar is the SAME TAPE as the horizontal one, so its measured width must be comparable. Real junctions measured tr/tc = 0.60-1.10 across both videos; the glare detections came in at 0.20-0.45.

**`REACQUIRE_MINI_EVERY = 12`**  
between junctions (no vertical bar in view) a full-width re-acquire scan runs every N frames, to notice the next junction arriving

**`PERF = 0`**  
once a second, print where the loop time ACTUALLY goes: capture / grayscale convert / otsu / scan / fit / draw / flush / gc. Costs a few microseconds of ticks_us calls and settles the question instead of us guessing at it. Turn off once tuned.

**`GC_EVERY = 30`**  
gc.collect() was running EVERY frame. On MicroPython a full collect is milliseconds, and at a 10ms budget that alone can be a third of the loop. Every 30 frames is plenty here: the loop allocates almost nothing per iteration (the arrays are preallocated once). Set to 1 to restore the old behaviour.

**`OTSU_EVERY = 3`**  
recompute the threshold every N frames. The copy+histogram is a real per-frame cost. Otsu was measured swinging 22 grey levels frame to frame anyway, so reusing it for 2-3 frames loses very little -- but measure with PERF before turning this up.

**`FLUSH = 1`**  
keep csi0.flush() even when DRAW is 0. WHY THIS IS SEPARATE: with DRAW=0 the measured rate FELL to 40-50 fps from ~80. Not drawing cannot cost time, so the suspect is flush(), which was guarded together with the drawing. flush() does not only push pixels to the IDE -- it also releases the frame buffer, and without that release snapshot() may stall waiting for a free one. Test the three combinations below and keep whichever is fastest ON THE ROBOT, with the IDE disconnected, since that is the case that matters:     DRAW=1 FLUSH=1   full overlay, ~80 fps     DRAW=0 FLUSH=0   measured 40-50 fps     DRAW=0 FLUSH=1   <- the one still untested

**`DRAW = 0`**  
1 = draw the overlay and flush to the IDE. SET TO 0 ON THE ROBOT. csi0.flush() pushes the whole frame over USB every iteration and it sits inside the timed loop: the IDE reported 137 fps at the camera against 66 for the loop. The overlay exists for you, not for the robot, and halving the frame period also halves the motion blur per the note above -- this is the single biggest live win available.

**`COLOR_OVERLAY = 0`**  
1 = capture RGB565 so the two guide lines can be drawn in real colour. Every scan and fit routine in this file indexes the frame as ONE BYTE PER PIXEL, so they must never see the RGB buffer -- each frame is converted to a grayscale copy for processing, and only the overlay is drawn on the colour original. Set to 0 to go back to GRAYSCALE capture (lines become white / dim grey).

**`COLOR_Y = (90, 90, 90)`**  
Y line -- dim grey, so the two lines are still tellable apart

**`EXPOSURE_US = 3000`**  
WORKS NOW -- fixed by the Sept development firmware (build 631681e5ac). Was a no-op on release v5.0.0; confirmed tracking linearly: 1000us->mean15, 2000us->mean30. 1500 splits the difference: ~5px blur at 0.75 m/s (was 28px at the old stuck 8328us). Global shutter readout floors fps near 60 regardless of this value -- shorter exposure does NOT cost fps on this sensor, unlike a rolling shutter.

**`GAIN_DB = 18.0`**  
your only real brightness control. Measured: mean 42 at 12 dB, 71 at 18, 106 at 24, with headroom staying above 100 throughout.

**`FRAMERATE = 240`**  
ACCEPTED BUT NOT HONOURED. The driver reports back 240, yet the applied exposure is 8328 us = 1/120 s, so the sensor really runs at 120 fps. Windowing to 320x100 did not change it either (the crop is applied after readout).  WHY THIS MATTERS: exposure == frame period on this sensor, and blur = exposure x speed. That is 8328 us, which at 0.75 m/s smears the bar 28 px -- confirmed against fast_ramp1, where measured edge width tracked prediction (7.0 vs 8.2 px, 14.0 vs 13.7 px). Blur is therefore NOT fixable in software on this firmware. The levers are: move slower, light the floor better, or take it up with OpenMV.

**`SIM_CMDS = []`**  
MUST be empty. This was [(0, "TURN L")] left over from the rotation benchmark, which forced the board into TURNING on frame 0 and kept it there -- which is why the rotation HUD kept showing and the X line never appeared. Only set this for bench testing.

**`NOMINAL_THICK_PX = 29`**  
measured tape thickness on this rig (from the background docs). hist_peak() below just counts which run length shows up MOST OFTEN this frame -- it has no idea which width is the real bar and which is a tooth on a serrated edge. If teeth outnumber genuine full-width crossings, hist_peak locks onto the TOOTH width, and THICK_LO/HI then filters FOR teeth and AGAINST the real bar, on purpose, every frame. This anchor catches that: a measured peak wildly off from the known real thickness is discarded in favour of the known value, rather than trusted blindly.

**`MAX_JOINT_MAD_PX = 3.0`**  
same test as MAX_FIT_MAD_PX below, but for the FULL-JUNCTION path. Measured on fast_ramp1: during fast motion the frames blur and the VERTICAL fit degrades badly (residual median 2.05px, p90 15.1px) while the horizontal fit stays clean (0.26px). The junction path was claiming those frames and reporting nonsense (T+12.0, Y+21.6) when the mini/Y fallback would have been accurate on the very same frame. Rejecting here hands the frame to that fallback instead of trusting a bad fit.

**`MAX_FIT_MAD_PX = 5.0`**  
the fallback's real "is there a bar?" test. Replaces the old peak-bin thickness-consensus check, which was measured to be BACKWARDS: on 851 recorded frames a real bar scored 0.29 and pure noise scored 0.50, so it rejected 55% of good frames (median 0.29 against a 0.30 threshold -- a coin flip every frame) while letting noise through. What actually separates them is whether the points lie on a LINE: real bar median fit residual 0.18-0.23 px, worst 0.49; noise 42-49 px. 5.0 px sits a factor of ten clear of both.

**`ACQUIRE_STEP = 2`**  
column/row spacing for the FULL re-acquire scan. Was 1, i.e. every single column -- 320 of them against 64 for the tracked scan, which is why scan+fit balloons whenever the lock is lost. 2 halves that cost. The acquire pass only has to FIND the marker well enough to set a band; the tracked pass does the accurate work. Raise to 3-4 if you need more, but check the marker is still picked up reliably.
