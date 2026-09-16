# Grid localization analytics

A local dashboard for reviewing recorded clips from the robot's downward
camera. Upload a clip, tag it with the use case it was recorded for, and the
dashboard runs **the robot's own camera code** over every frame — the same
`main.py`, `turn_sm.py`, `rot_scan.py` and `hud.py` that run on the OpenMV
board — and shows what that code drew and printed.

Nothing in this tool re-implements the algorithm. The scan, the fits, the
gates, the angle tracker, the overlay: all of it is the flight code, executed
unchanged. What you see is what the robot would have computed on that clip.

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

Then open **http://127.0.0.1:5000** in Chrome or Safari. Leave the terminal
open while you use it; `Ctrl+C` stops it. Runs are kept in `data/` next to
the code and are there next time. If `python` is not found use `python3`;
if port 5000 is busy, `PORT=5001 python app.py`.

---

## Uploading a clip

1. **Use case** — pick the case this clip tests, from the list (the *Use
   cases* button in the header shows the whole list with how many runs each
   has). You can change it later from the run page.
2. **Turn in this clip?** — the robot never decides to turn on its own; the
   host sends `TURN L` / `TURN R` and later `DRIVE`. If the clip has a turn,
   enter the frame the host would have sent the command on. The same
   command is fed to the real state machine. Leave it empty for a
   driving-only clip. Don't know the frame yet? Upload without it, scrub to
   the spot, and use **Re-run** on the run page — it re-analyses the same
   clip with the command at the current frame.
3. **Output size** — how much the annotated video is enlarged. Cosmetic.

Record clips with `DRAW = 0` on the camera if you can. A clip recorded with
the overlay on still analyses fine, but the recording's own text will be
visible under the fresh overlay.

---

## Reading the page

**The video** is the clip with the robot's overlay drawn on it: the
top-left `X Y T DRV|ROT` line, the heading gauge top-right, the fitted tape
lines and the junction cross — drawn by the flight code's own `draw_*`
calls, at the robot's own pixel positions. The readout beside it follows the
playhead. Click or drag on any chart to jump the video.

**X, Y** — as the robot reports them, in mm. X follows the robot's
convention: 0 → −30 leaving a junction, null across the middle of the
cell, +30 → 0 arriving at the next. Blank where the robot reported `--`.
Shaded bands mark frames where the state machine was in TURN.

**THETA** — the tape tilt in that frame, from the line fit while driving or
from the tilted scan while turning. It wraps at ±45° by definition; the
breaks are wraps, not dropouts.

**Heading** — the number on the robot's gauge: the continuous angle it
tracks through a turn, seeded from the tape tilt so 90.0 means square.

**Fix per frame** — one bar per frame in the robot's own tiers: junction /
Y only / X only / one bar (turning) / none.

**What the robot produced** — the tier split while driving and while
turning. **Run** — the turn command used, how much of the clip was in each
mode, how often X/Y/THETA were reported, heading start → end, and the
fingerprint of the flight code that ran. **Robot terminal** — the events the
camera would have printed (`TURN_START`, `TURN_SEED`, `TURN_END`,
`JUNCTION_LANDED`, rejections, faults); the full once-a-second status log is
a download.

---

## Files each run produces

Under `data/<run-id>/`: the source clip, `annotated.mp4`, `frames.csv`
(frame, t_s, mode, tier, x_mm, y_mm, theta_deg, heading_deg — one row per
frame, exactly as reported), `summary.json`, `series.json` (what the page
reads), `terminal.txt` (the robot's terminal output), `_main_as_run.py`
(the flight `main.py` with the two bench knobs set for this run — `DRAW = 1`
and the `SIM_CMDS` turn command — and nothing else changed), and your notes.

---

## The flight code

`flight/` holds the four camera files **verbatim**. The page footer and every
`summary.json` carry a fingerprint (short hash) of them, so a run always
records exactly which code produced it. When the camera code changes,
replace all four files together and the fingerprint changes with them.

The only edits ever made to the flight code, and only to a private copy per
run: `DRAW = 0` → `1` so the overlay is drawn, and `SIM_CMDS = []` → the
turn command(s) you entered. Both are knobs `main.py` itself exposes for
bench use. `flight/main.py` on disk is never modified.

`flight_runner.py` supplies the OpenMV runtime the flight code expects
(`csi`, `image`, `time.ticks_ms`, `micropython.viper`) as look-alikes that
feed video frames in and record draw calls out. The viper kernels run as
plain Python, so a clip analyses at roughly 100–200 frames per second on a
laptop rather than the board's real-time rate — the numbers are the same,
only the speed differs.

---

## The use-case list

`cases.json` — 7 sections, 29 cases, taken as-is from the *It's all about
robustness* page. Edit that file to change the list; the dashboard reads it
on start.
