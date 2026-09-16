#!/usr/bin/env python3
"""
flight_runner.py -- run the REAL camera code on a recorded clip.

This is the only place the dashboard "analyses" anything, and it does not
analyse: it executes flight/main.py -- the same file that runs on the robot
-- against the frames of a video, and writes down what that code drew and
printed. Nothing here re-implements a scan, a fit, a filter or a threshold.

How the flight code is run unchanged
    main.py expects an OpenMV runtime: modules `csi`, `image`, `time` with
    ticks_ms/ticks_us, `micropython.viper`, `gc.mem_free`. This file supplies
    look-alikes that feed video frames in and record draw_* calls out. The
    viper kernels run as plain Python (the decorator is the identity), so
    they are slow here but produce the same numbers.

    Two bench knobs the flight code itself exposes are set by rewriting one
    line each in a private copy of main.py (flight/main.py is never edited):
        DRAW = 1       so the overlay is drawn (the robot runs with 0)
        SIM_CMDS = ... the turn command(s), exactly as a host would send them

What comes out (in --out)
    annotated_raw.mp4   every frame with the overlay the robot drew on it
    frames.jsonl        one record per video frame: what the overlay said
    terminal.txt        what the robot would have printed to its terminal
    progress.json       {done, total}, refreshed every few frames
    error.json          only if the flight code raised

Frame numbering: the flight code consumes two frames at start-up (its own
init probes) before its loop begins, so its frame_id is the video index
minus 2. Records here are keyed by VIDEO index; SIM_CMDS frames are passed
in already converted by the caller.

Usage
    python flight_runner.py --flight DIR --video CLIP --out DIR
                            [--scale 3] [--sim '[[38,"TURN L"],[300,"DRIVE"]]']
"""

import sys
import os
import re
import json
import types
import builtins
import argparse
import runpy
import time as _rtime

import numpy as np
import cv2

# ---------------------------------------------------------------- args
ap = argparse.ArgumentParser()
ap.add_argument("--flight", required=True)
ap.add_argument("--video", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--scale", type=int, default=3)
ap.add_argument("--sim", default="[]")
A = ap.parse_args()
os.makedirs(A.out, exist_ok=True)

SIM_CMDS = [(int(f), str(c)) for f, c in json.loads(A.sim)]


def write_json(name, obj):
    with open(os.path.join(A.out, name), "w") as f:
        json.dump(obj, f)


# ---------------------------------------------------------------- micropython
mp = types.ModuleType("micropython")
mp.viper = lambda f: f
mp.native = lambda f: f
sys.modules["micropython"] = mp
builtins.micropython = mp
builtins.ptr8 = object
builtins.ptr16 = object
builtins.ptr32 = object

# ---------------------------------------------------------------- time
SIM = {"ms": 0.0, "fps": 30.0}


class _Clock:
    def tick(self):
        pass

    def fps(self):
        return SIM["fps"]


_time = types.ModuleType("time")
_time.ticks_ms = lambda: int(SIM["ms"])
_time.ticks_us = lambda: int(SIM["ms"] * 1000)
_time.ticks_diff = lambda a, b: a - b
_time.ticks_add = lambda a, b: a + b
_time.sleep_ms = lambda n: None
_time.sleep = lambda n: None
_time.clock = _Clock
_time.time = _rtime.time
sys.modules["time"] = _time

import gc                       # noqa: E402
gc.mem_free = lambda: 0

# ---------------------------------------------------------------- image / csi


class _Val:
    def __init__(self, v):
        self.value = v          # attribute, as on this firmware


class _Hist:
    def __init__(self, arr):
        self.arr = arr

    def get_threshold(self):
        t, _ = cv2.threshold(self.arr, 0, 255,
                             cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        return _Val(int(t))


class _Stats:
    def __init__(self, arr):
        self.arr = arr

    def max(self):
        return int(self.arr.max())

    def mean(self):
        return int(self.arr.mean())

    def min(self):
        return int(self.arr.min())


class Image:
    """Grayscale image, one byte per pixel. Drawing goes onto a BGR canvas
    and is also logged, so the record of a frame is literally the list of
    draw calls the flight code made."""

    def __init__(self, gray, canvas=None, log=None):
        self.g = gray
        self.canvas = canvas
        self.log = log
        self.texts = []

    def width(self):
        return self.g.shape[1]

    def height(self):
        return self.g.shape[0]

    def bytearray(self):
        return bytearray(self.g.tobytes())

    def copy(self, x_scale=1.0, y_scale=1.0, copy_to_fb=False, pixformat=None):
        if x_scale == 1.0 and y_scale == 1.0:
            return Image(self.g.copy())
        h, w = self.g.shape
        s = cv2.resize(self.g, (max(1, int(w * x_scale)), max(1, int(h * y_scale))),
                       interpolation=cv2.INTER_AREA)
        return Image(s)

    def get_histogram(self):
        return _Hist(self.g)

    def get_statistics(self):
        return _Stats(self.g)

    @staticmethod
    def _c(color):
        if isinstance(color, (tuple, list)):
            r, g, b = color[:3]
            return (int(b), int(g), int(r))
        v = int(color)
        return (v, v, v)

    def draw_line(self, t, color=(255, 255, 255), thickness=1):
        x0, y0, x1, y1 = [int(v) for v in t]
        if self.log is not None:
            self.log.append(("line", x0, y0, x1, y1))
        if self.canvas is not None:
            cv2.line(self.canvas, (x0, y0), (x1, y1), self._c(color), thickness)

    def draw_rectangle(self, t, color=(255, 255, 255), fill=False, thickness=1):
        x, y, w, h = [int(v) for v in t]
        if self.canvas is not None:
            cv2.rectangle(self.canvas, (x, y), (x + w, y + h), self._c(color),
                          -1 if fill else thickness)

    def draw_cross(self, t, color=(255, 255, 255), size=5, thickness=1):
        x, y = int(t[0]), int(t[1])
        if self.log is not None:
            self.log.append(("cross", x, y))
        if self.canvas is not None:
            cv2.line(self.canvas, (x - size, y), (x + size, y), self._c(color), thickness)
            cv2.line(self.canvas, (x, y - size), (x, y + size), self._c(color), thickness)

    def draw_string(self, t, text, color=(255, 255, 255), scale=1):
        # logged now, rendered by CSI._emit() AFTER the upscale so the
        # letters are crisp; lines and crosses stay at native pixels
        x, y = int(t[0]), int(t[1])
        if self.log is not None:
            self.log.append(("text", x, y, text))
        self.texts.append((x, y, text, self._c(color)))


class EndOfVideo(KeyboardInterrupt):
    """The flight loop catches KeyboardInterrupt and exits cleanly."""


class CSI:
    GRAYSCALE = "GRAYSCALE"
    RGB565 = "RGB565"
    QVGA = "QVGA"

    def __init__(self):
        self.frames = []
        self.writer = None
        self.frame_no = -1
        self.cur = None
        self.logs = []                # per video frame: list of draw calls
        self.flushed = []
        self.scale = A.scale

    def load(self, path, out_path):
        cap = cv2.VideoCapture(path)
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        SIM["fps"] = fps
        while True:
            ok, fr = cap.read()
            if not ok:
                break
            self.frames.append(cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY))
        cap.release()
        if not self.frames:
            raise RuntimeError("no frames decoded from %s" % path)
        h, w = self.frames[0].shape
        self.writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"mp4v"),
                                      fps, (w * self.scale, h * self.scale))

    # sensor config: accept everything
    def reset(self): pass
    def pixformat(self, *a): pass
    def framesize(self, *a): pass
    def framerate(self, *a): pass
    def auto_exposure(self, *a, **k): pass
    def auto_gain(self, *a, **k): pass
    def framebuffers(self, *a): pass
    def width(self): return self.frames[0].shape[1]
    def height(self): return self.frames[0].shape[0]
    def exposure_us(self): return 3000
    def gain_db(self): return 18.0

    def _emit(self):
        if self.cur is None:
            return
        canvas = self.cur.canvas
        s = self.scale
        if s != 1:
            canvas = cv2.resize(canvas, None, fx=s, fy=s,
                                interpolation=cv2.INTER_NEAREST)
        # OpenMV's font is 8 px tall; match that height at this scale
        fs = 0.32 * s
        for x, y, text, col in self.cur.texts:
            cv2.putText(canvas, text, (x * s, (y + 7) * s), cv2.FONT_HERSHEY_SIMPLEX,
                        fs, (0, 0, 0), max(1, s), cv2.LINE_AA)
            cv2.putText(canvas, text, (x * s, (y + 7) * s), cv2.FONT_HERSHEY_SIMPLEX,
                        fs, col, max(1, s // 2), cv2.LINE_AA)
        self.writer.write(canvas)
        self.cur = None

    def snapshot(self, time=None):
        self._emit()
        self.frame_no += 1
        if self.frame_no % 10 == 0:
            write_json("progress.json", {"done": self.frame_no,
                                         "total": len(self.frames)})
        if self.frame_no >= len(self.frames):
            self.frame_no = len(self.frames) - 1
            raise EndOfVideo()
        SIM["ms"] = self.frame_no * 1000.0 / SIM["fps"]
        g = self.frames[self.frame_no]
        log = []
        self.logs.append(log)
        self.flushed.append(0)
        self.cur = Image(g, cv2.cvtColor(g, cv2.COLOR_GRAY2BGR), log)
        return self.cur

    def flush(self):
        if self.cur is not None:
            self.flushed[-1] += 1

    def close(self):
        self._emit()
        if self.writer is not None:
            self.writer.release()


DEV = CSI()
csi = types.ModuleType("csi")
csi.CSI = lambda: DEV
csi.GRAYSCALE = CSI.GRAYSCALE
csi.RGB565 = CSI.RGB565
csi.QVGA = CSI.QVGA
sys.modules["csi"] = csi

image = types.ModuleType("image")
image.GRAYSCALE = "GRAYSCALE"
image.Image = Image
sys.modules["image"] = image

# ---------------------------------------------------------------- run
DEV.load(A.video, os.path.join(A.out, "annotated_raw.mp4"))
write_json("progress.json", {"done": 0, "total": len(DEV.frames)})

src = open(os.path.join(A.flight, "main.py")).read()
src, n1 = re.subn(r"^DRAW = 0\b", "DRAW = 1", src, count=1, flags=re.M)
src, n2 = re.subn(r"^SIM_CMDS = \[\]", "SIM_CMDS = %r" % (SIM_CMDS,), src,
                  count=1, flags=re.M)
if n1 != 1 or n2 != 1:
    write_json("error.json", {"error": "flight/main.py: could not find the "
                              "DRAW / SIM_CMDS lines to set (%d, %d)" % (n1, n2)})
    sys.exit(2)
patched = os.path.join(A.out, "_main_as_run.py")
open(patched, "w").write(src)

sys.path.insert(0, A.flight)
term_path = os.path.join(A.out, "terminal.txt")
term = open(term_path, "w")
old_stdout = sys.stdout
sys.stdout = term
err = None
t0 = _rtime.time()
try:
    runpy.run_path(patched, run_name="__main__")
except SystemExit:
    pass
except Exception as e:                 # noqa: BLE001
    err = "%s: %s" % (type(e).__name__, e)
finally:
    sys.stdout = old_stdout
    term.close()
    DEV.close()
elapsed = _rtime.time() - t0

if err:
    write_json("error.json", {"error": err})
    sys.exit(1)

# ---------------------------------------------------------------- records
# One record per video frame, taken from what the flight code DREW. The
# top-left text line is its own report of X, Y, T and mode; the gauge text
# is the heading; a drawn cross means the junction path produced a fix.
R_LINE = re.compile(r"X(\s*--\s*|[+-]\d+\.\d)\s+Y(\s*--\s*|[+-]\d+\.\d)\s+T(\s*--\s*|[+-]\d+\.\d)\s+(DRV|ROT)")
R_GAUGE = re.compile(r"^[+-]\d+\.\d$")


def _num(s):
    return None if "--" in s else float(s)


recs = []
for i, log in enumerate(DEV.logs):
    x = y = th = None
    mode = None
    heading = None
    cross = False
    n_lines = 0
    for call in log:
        if call[0] == "text":
            m = R_LINE.search(call[3])
            if m:
                x, y, th, mode = _num(m.group(1)), _num(m.group(2)), _num(m.group(3)), m.group(4)
            elif R_GAUGE.match(call[3].strip()):
                heading = float(call[3])
        elif call[0] == "cross":
            cross = True
        elif call[0] == "line":
            n_lines += 1
    # tier, from the robot's own outputs, in the robot's own words
    if mode == "ROT":
        tier = "junction" if cross else ("one bar" if n_lines else "none")
    elif mode == "DRV":
        if cross:
            tier = "junction"
        elif y is not None and x is None:
            tier = "Y-only"
        elif x is not None and y is None:
            tier = "X-only"
        elif x is None and y is None:
            tier = "none"
        else:
            tier = "junction"
    else:
        tier = "none"
        # Drive draws its line only when it has something to report; the
        # rotation overlay always draws. So a frame with no line, after the
        # two start-up frames, is drive (or REACQUIRE, which runs Algorithm
        # 1 too) with nothing found.
        if i >= 2:
            mode = "DRV"
    recs.append({"frame": i, "t": round(i / SIM["fps"], 4), "mode": mode,
                 "tier": tier, "x": x, "y": y, "theta": th, "heading": heading,
                 "flushed": DEV.flushed[i] if i < len(DEV.flushed) else 0})

with open(os.path.join(A.out, "frames.jsonl"), "w") as f:
    for r in recs:
        f.write(json.dumps(r) + "\n")

write_json("progress.json", {"done": len(DEV.frames), "total": len(DEV.frames)})
write_json("run.json", {"frames": len(DEV.frames), "fps": SIM["fps"],
                        "width": DEV.width(), "height": DEV.height(),
                        "scale": A.scale, "elapsed_s": round(elapsed, 2),
                        "sim_cmds": SIM_CMDS})
