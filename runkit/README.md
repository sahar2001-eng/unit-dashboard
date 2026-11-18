# Run kit

Makes runs for the site's **Runs** page, and uploads them. The site only
shows runs; all the work happens here, with the camera's own code.

## One-time setup (Mac)

1. Install **Docker Desktop** (docker.com) and open it once.
   The camera code is built for Intel-type (x64) chips. Docker runs it on an
   M-chip Mac.
2. That's all. `run.py` needs only the Python 3 that comes with the Mac.
   The first `video` or `record` builds the kit's Docker image (a few
   minutes, once).

Optional: put the upload password in the terminal once, so it doesn't ask:

```
export RUNS_TOKEN=the-password
```

## Make a run from a video

```
cd runkit
python3 run.py video ~/Movies/clip.mp4 --tile 2
```

- `--tile 1` or `--tile 2`: which tile the video shows.
- Turns: `--turn 120 L` (or `R`) and `--drive 300`, as if the STM32 had sent
  `V 20 1/2` and `V 20 0` on those frames.
- `--name "glare test"`, `--notes "..."`, `--no-quality` (quality score off).
- `--upload` uploads right after.

The run goes into `runkit/runs/<date>-<name>/`.

## Record a robot drive

```
python3 run.py record --cam 192.168.4.1 --name "drive 1"
```

This saves the camera's live stream: every picture with the numbers the
camera sent with it (frame, x, y, angle, code, fps). Press **Ctrl+C** to stop.
`--seconds 30` stops by itself. Nothing on the camera changes. Recording
uses the stream the page shows, so don't keep the camera page open at the
same time (a new viewer takes the stream).

## Upload

```
python3 run.py upload runs/20261118-1030-drive-1
```

Add `--case 2.3` to tag a use case. You can also do that later on the page.
The command prints the link to the run.

## What a run folder holds

| File | What |
|---|---|
| `video.mp4` | the pictures, with the camera's overlay drawn on them |
| `frames.csv` | one row per picture, in the same order |
| `info.json` | tile, source, camera app version, pictures per second |
| `log.txt` | the camera code's summary (video runs) |

`frames.csv` columns:

- `frame`, `t_s`, `tile`
- `tier`: junction / y / x / empty
- `suspect`, `x_mm`, `y_mm`, `theta_deg`
- `code`: Tile 2 junction code
- `q`, `why`: quality score and reason
- `mode`: DRIVE / TURNING / ...
- `heading`: turn angle, during turns

Recordings also have `fps` and `ts_ms`. Recordings have no `q`, `why`,
`mode` or `heading` yet: the stream doesn't send them.

## When the camera code changes

`cam/` holds a copy of the camera app (`cam/app/gridcam`, `cam/t2lib`) and
its PC builds (`cam/pcx64/*.mpy`, `cam/micropython`). `kit.json` says which
version it is: today gridcam 1.32.0, ROBOT3 2ee7c79f. Replace those files
together and change `kit.json`. The next run rebuilds the Docker image by
itself.

## Testing without Docker (Linux x64 only)

`RUNKIT_LOCAL=1 python3 run.py ...` runs the same `engine.py` directly.
It needs `numpy`, `opencv-python-headless` and `imageio-ffmpeg`.
