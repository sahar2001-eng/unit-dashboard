#!/usr/bin/env python3
"""engine.py -- the work inside the run kit's container (Linux x64).

  engine.py video <clip> <out dir> --tile N [--cmd FRAME "V 20 1"]... [--quality 0|1]
      The REAL gridcam camera loop (cam/mp_run.py, unix MicroPython, the
      camera's own .mpy C modules) over every frame of the clip. Writes a
      run folder: video.mp4 (the camera's own overlay drawn on each frame),
      frames.csv (one row per picture), info.json, log.txt.

  engine.py pack <out dir>
      A recorded robot drive (run.py record): <out dir>/raw/ holds the
      stream's JPEGs and meta.jsonl. Writes video.mp4 + frames.csv + info.json.

Nothing here measures anything: the numbers are the camera code's.
"""

import argparse
import csv
import json
import os
import subprocess
import sys
import tempfile
import time

import cv2
import numpy as np
import imageio_ffmpeg

HERE = os.path.dirname(os.path.abspath(__file__))
CAM = os.path.join(HERE, "cam")
KIT = json.load(open(os.path.join(HERE, "kit.json")))
W, H = 320, 200
UP = 3                                  # video size: 960 x 600
COLS = ["frame", "t_s", "tile", "tier", "suspect", "x_mm", "y_mm", "theta_deg",
        "code", "q", "why", "mode", "heading"]

# the camera draws in gray levels; on the PC each gets a colour
LINE_COL = {255: (0, 220, 255), 90: (255, 80, 220)}      # X line, Y line (BGR)
CROSS_COL = {255: (60, 230, 60), 128: (170, 170, 170)}   # junction, centre


def say(*a):
    print(*a, flush=True)


# ------------------------------------------------------------------ video in
def read_clip(path, raw_path):
    """Gray 320x200 frames of the clip into raw_path. (n, fps, note)."""
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise SystemExit("cannot open the video: %s" % path)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    if not (1 <= fps <= 1000):
        fps = 30.0
    n, note = 0, ""
    with open(raw_path, "wb") as f:
        while True:
            ok, fr = cap.read()
            if not ok:
                break
            g = fr if fr.ndim == 2 else cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY)
            h, w = g.shape
            if (w, h) != (W, H):
                if (w, h) == (W, 240):          # 320x240: keep the middle 200 rows
                    g = g[20:220, :]
                    note = "320x240 clip: the middle 320x200 was used"
                else:
                    g = cv2.resize(g, (W, H), interpolation=cv2.INTER_AREA)
                    note = "%dx%d clip resized to 320x200 (mm values assume the camera's scale)" % (w, h)
            f.write(np.ascontiguousarray(g, dtype=np.uint8).tobytes())
            n += 1
    cap.release()
    if not n:
        raise SystemExit("no frames could be read from %s" % path)
    return n, fps, note


# ------------------------------------------------------------------ drawing
def draw_ops(img, ops, s):
    """The camera's overlay calls (overlay.draw on a recording image),
    drawn on the PC picture at scale s."""
    P = lambda x, y: (int(round(x * s)), int(round(y * s)))  # noqa: E731
    for op in ops:
        kind = op[0]
        if kind == "line":
            x0, y0, x1, y1 = op[1]
            cv2.line(img, P(x0, y0), P(x1, y1), LINE_COL.get(op[2], (255, 255, 255)), 2, cv2.LINE_AA)
        elif kind == "cross":
            (x, y), c, size = op[1], op[2], op[3]
            col = CROSS_COL.get(c, (255, 255, 255))
            cx, cy = P(x, y)
            r = int(size * s)
            cv2.line(img, (cx - r, cy), (cx + r, cy), col, 2, cv2.LINE_AA)
            cv2.line(img, (cx, cy - r), (cx, cy + r), col, 2, cv2.LINE_AA)
        elif kind == "rect":
            x, y, w, h = op[1]
            c = op[2]
            cv2.rectangle(img, P(x, y), P(x + w, y + h), (c, c, c), -1 if op[3] else 1)
        elif kind == "text":
            (x, y), text, c, sc = op[1], op[2], op[3], op[4]
            fs = 0.38 * s * sc / 1.0 * 0.85      # ~ the camera's 8x10 font, scaled
            org = P(x, y + 9 * sc)
            # a dark box behind the text (an outline drifts: OpenCV's letter
            # spacing grows with the stroke width)
            (tw, th), bl = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, fs, 1)
            x0, y0 = org[0] - 2, org[1] - th - 3
            x1, y1 = org[0] + tw + 2, org[1] + bl + 1
            x0, y0 = max(0, x0), max(0, y0)
            roi = img[y0:y1, x0:x1]
            if roi.size:
                img[y0:y1, x0:x1] = (roi * 0.35).astype(img.dtype)
            cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, fs, (c, c, c), 1, cv2.LINE_AA)


def writer(path, size, fps):
    w = imageio_ffmpeg.write_frames(path, size, fps=fps, codec="libx264", pix_fmt_in="bgr24",
                                    pix_fmt_out="yuv420p", quality=7,
                                    output_params=["-movflags", "+faststart"], macro_block_size=8)
    w.send(None)
    return w


def fmt(v):
    if v is None:
        return ""
    if isinstance(v, float):
        return "%.3f" % v
    return str(v)


# ------------------------------------------------------------------ video
def cmd_video(a):
    out = a.out
    os.makedirs(out, exist_ok=True)
    tmp = tempfile.mkdtemp()
    raw = os.path.join(tmp, "clip.raw")
    say("reading %s ..." % os.path.basename(a.clip))
    n, fps, note = read_clip(a.clip, raw)
    say("  %d frames, %.1f fps%s" % (n, fps, ("  (" + note + ")") if note else ""))

    cmds = [[int(k), c] for k, c in (a.cmd or [])]
    job = {"app": os.path.join(CAM, "app"), "t2lib": os.path.join(CAM, "t2lib"),
           "pcx64": os.path.join(CAM, "pcx64"), "raw": raw, "n": n, "tile": a.tile,
           "clock_us": int(round(1e6 / fps)), "cmds": cmds,
           "set": {"quality.enabled": bool(a.quality)},
           "out": os.path.join(tmp, "rows.jsonl")}
    jf = os.path.join(tmp, "job.json")
    json.dump(job, open(jf, "w"))
    say("running the camera code (gridcam %s) ..." % KIT["app_version"])
    env = dict(os.environ)
    env.pop("MICROPYPATH", None)
    t0 = time.time()
    p = subprocess.run([os.path.join(CAM, "micropython"), "-X", "heapsize=64M",
                        os.path.join(CAM, "mp_run.py"), jf], cwd=tmp, env=env,
                       capture_output=True, text=True)
    if p.returncode:
        sys.stderr.write(p.stdout[-3000:] + p.stderr[-3000:])
        raise SystemExit("the camera code stopped with an error (above)")
    lines = open(job["out"]).read().splitlines()
    summary = json.loads(lines[-1])["summary"]
    rows = [json.loads(l) for l in lines[:-1]]
    say("  %d frames in %.1f s" % (len(rows), time.time() - t0))
    if summary.get("tile_err"):
        say("  camera says: " + summary["tile_err"])

    say("drawing the video ...")
    size = (W * UP, H * UP)
    vw = writer(os.path.join(out, "video.mp4"), size, fps)
    with open(raw, "rb") as f, open(os.path.join(out, "frames.csv"), "w", newline="") as cf:
        cw = csv.writer(cf)
        cw.writerow(COLS)
        for r in rows:
            f.seek(r["ck"] * W * H)
            g = np.frombuffer(f.read(W * H), dtype=np.uint8).reshape(H, W)
            img = cv2.cvtColor(cv2.resize(g, size, interpolation=cv2.INTER_LINEAR), cv2.COLOR_GRAY2BGR)
            draw_ops(img, r["ops"], UP)
            vw.send(np.ascontiguousarray(img))
            cw.writerow([r["ck"], "%.4f" % (r["ck"] / fps)] + [fmt(r.get(c)) for c in COLS[2:]])
    vw.close()

    with open(os.path.join(out, "log.txt"), "w") as lf:
        lf.write("camera summary:\n%s\n" % json.dumps(summary, indent=1))
    info = {"format": 1, "name": a.name or os.path.splitext(os.path.basename(a.clip))[0],
            "source": "video", "source_file": os.path.basename(a.clip), "tile": a.tile,
            "created": time.time(), "frames": len(rows), "fps": round(fps, 3),
            "video_fps": round(fps, 3), "video_size": list(size), "note": note,
            "cmds": cmds, "quality": bool(a.quality),
            "app_version": KIT["app_version"], "app_commit": KIT["app_commit"],
            "kit_version": KIT["kit_version"], "notes": a.notes or ""}
    json.dump(info, open(os.path.join(out, "info.json"), "w"), indent=1)
    say("done: %s" % out)


# ------------------------------------------------------------------ pack
def cmd_pack(a):
    out = a.out
    rawd = os.path.join(out, "raw")
    metas = [json.loads(l) for l in open(os.path.join(rawd, "meta.jsonl")) if l.strip()]
    if not metas:
        raise SystemExit("nothing was recorded")
    hdr = json.load(open(os.path.join(rawd, "header.json")))
    t0 = metas[0]["pc_t"]
    # time: the camera's own clock (ts_ms) when it is there, else arrival time
    ts = [m.get("kv", {}).get("ts_ms") for m in metas]
    cam_clock = all(v not in (None, "-", "") for v in ts)
    if cam_clock:
        ts0 = int(ts[0])
    dur = max(1e-3, metas[-1]["pc_t"] - t0)
    vfps = round(max(1.0, (len(metas) - 1) / dur), 2) if len(metas) > 1 else 10.0
    first = cv2.imdecode(np.fromfile(os.path.join(rawd, metas[0]["file"]), np.uint8), cv2.IMREAD_COLOR)
    up = max(1, int(round(640 / first.shape[1])))
    size = (first.shape[1] * up, first.shape[0] * up)
    say("packing %d pictures (%.1f s, %.1f pictures/s) ..." % (len(metas), dur, vfps))
    vw = writer(os.path.join(out, "video.mp4"), size, vfps)
    tiles = set()
    with open(os.path.join(out, "frames.csv"), "w", newline="") as cf:
        cw = csv.writer(cf)
        cw.writerow(COLS + ["fps", "ts_ms"])
        for m in metas:
            img = cv2.imdecode(np.fromfile(os.path.join(rawd, m["file"]), np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                img = np.zeros((size[1], size[0], 3), np.uint8)
            else:
                img = cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)
            vw.send(np.ascontiguousarray(img))
            k = m.get("kv", {})
            num = lambda key: None if k.get(key) in (None, "-") else float(k[key])  # noqa: E731
            tier = k.get("tier", "-")
            sus = tier.endswith("!")
            tier = tier.rstrip("!")
            tile = int(k["tile"]) if k.get("tile", "-") != "-" else None
            tiles.add(tile)
            t_s = (int(k["ts_ms"]) - ts0) / 1000.0 if cam_clock else m["pc_t"] - t0
            cw.writerow([k.get("frame", ""), "%.4f" % t_s, fmt(tile),
                         "" if tier == "-" else tier, 1 if sus else 0,
                         fmt(num("x")), fmt(num("y")), fmt(num("th")),
                         "" if k.get("code", "-") == "-" else k["code"], "", "", "", "",
                         fmt(num("fps")), k.get("ts_ms", "")])
    vw.close()
    tiles.discard(None)
    info = {"format": 1, "name": hdr.get("name") or os.path.basename(out.rstrip("/")),
            "source": "robot", "tile": (sorted(tiles)[0] if len(tiles) == 1 else (hdr.get("tile") or 0)),
            "tiles_seen": sorted(tiles), "created": hdr.get("created", time.time()),
            "frames": len(metas), "video_fps": vfps, "video_size": list(size),
            "camera": hdr.get("cam"), "app_version": hdr.get("app_version") or "",
            "app_commit": "", "kit_version": KIT["kit_version"], "notes": hdr.get("notes", "")}
    json.dump(info, open(os.path.join(out, "info.json"), "w"), indent=1)
    say("done: %s" % out)


def main():
    ap = argparse.ArgumentParser(prog="engine.py")
    sub = ap.add_subparsers(dest="what", required=True)
    v = sub.add_parser("video")
    v.add_argument("clip")
    v.add_argument("out")
    v.add_argument("--tile", type=int, choices=(1, 2), required=True)
    v.add_argument("--cmd", nargs=2, action="append", metavar=("FRAME", "LINE"))
    v.add_argument("--quality", type=int, default=1)
    v.add_argument("--name")
    v.add_argument("--notes")
    p = sub.add_parser("pack")
    p.add_argument("out")
    a = ap.parse_args()
    {"video": cmd_video, "pack": cmd_pack}[a.what](a)


if __name__ == "__main__":
    main()
