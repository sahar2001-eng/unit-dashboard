# Grid localization analytics

A local dashboard for reviewing the robot's grid-localization pipeline end to
end: where it thinks it is, which way it is pointing, and how much to trust
either. Upload a clip from the downward camera and it runs the whole estimator
over every frame, writes an annotated video, and shows the traces beside it on
a shared time axis.

---

## Running it

**First time only**

```bash
cd unit-rotation-dashboard
pip install -r requirements.txt
```

**Every time after that**

```bash
cd unit-rotation-dashboard
python app.py
```

Then open **http://127.0.0.1:5000** in Chrome or Safari.

Leave the terminal window open while you use it. Press `Ctrl+C` in that window
to stop the server. Your previous runs are still there next time you start it.

If `python` is not found, use `python3`. If port 5000 is busy, run
`PORT=5001 python app.py` and open the matching address.

Everything runs on your machine. No clip is uploaded anywhere; runs live in the
`data/` folder next to the code.

---

## What it analyses

The full pipeline, in four tiers, exactly as the flight code reports them.

| Tier | What was found | What comes out |
|---|---|---|
| Full junction | both tapes, crossing located | X, Y and heading |
| Mini junction | one tape plus the mid-cell gap | X, Y and heading |
| Single bar | one tape only | Y and heading |
| Angle only | no usable tape | heading |

The heading always comes out, because the global gradient stage cannot fail.
Position degrades gracefully through the tiers rather than dropping out.

**X** is the distance to the next junction ahead, 0 to 80 mm, counting down as
the robot drives and resetting when it passes one — the same convention as the
camera script. **Y** is the offset from the tape centreline. The dashboard also
integrates X into total distance travelled, so a wandering sawtooth becomes a
straight line you can read at a glance.

---

## Reading the page

**Where the robot is.** Distance to the next junction, with red marks at each
crossing. Distance travelled along the tape. Offset from the centreline — if
this drifts, the robot is creeping sideways.

**Which way it is pointing.** Heading, continuous through the 90° wrap, with
the unsmoothed measurement ghosted behind it and shaded bands over each turn.
Turn rate, for sizing gates and checking acceleration. The raw tape angle,
which wraps every 90° by nature — the breaks are wraps, not dropouts.

**How much to trust it.** Line-fit residual, the real "is there a marker" test:
a genuine bar fits to about 0.2 px, noise to about 45. The Otsu threshold,
which swings between frames. Image sharpness, which falls when the tape smears
from motion blur.

**Lock quality per frame** is one coloured bar per frame showing which tier
produced that frame.

Click or drag on any trace to jump the video there. The readout beside the
video follows the playhead, and clicking a row in the turns table jumps to that
turn.

**Turns detected** lists each sustained rotation with its measured size and how
far it landed from a clean 90°. That last column is what a turn controller gets
judged on.

**Position fix across the angle range** breaks the fix rate down by tape angle.
The 30°–45° row is the one that matters — it is where the current column-scan
approach fails completely.

---

## Options when you upload

- **Detail** — how many pixels the angle stage looks at. Every 2nd pixel is the
  default and costs about 0.15° of noise; every 4th is four times faster at
  0.20°.
- **Output size** — how much the annotated video is enlarged. Cosmetic.
- **Smooth the heading** — the α–β filter. On the sample clip it takes jitter
  from 0.123° to 0.054°, at the cost of 0.89° of lag at peak rate.

---

## Files each run produces

Under `data/<run-id>/`: the source clip, `annotated.mp4`, `frames.csv` with
every field for every frame, `summary.json`, and your notes. Deleting a run
from the dashboard deletes the folder.

`frames.csv` columns: frame, t_s, theta_m1, theta, heading_raw, heading,
rate_dps, tier, x_to_next_mm, y_offset_mm, travel_mm, otsu, sharpness,
jump_rejected, fit_residual_px.

---

## Calibration

`MM_PER_PIXEL` and `JUNCTION_SPACING_MM` are at the top of `engine.py`, along
with the scan and gate constants. Change them there if the camera height or the
grid pitch changes.

The annotated video is re-encoded to H.264 so browsers can play it; the
`imageio-ffmpeg` package in requirements.txt provides the encoder, so no
separate ffmpeg install is needed.
