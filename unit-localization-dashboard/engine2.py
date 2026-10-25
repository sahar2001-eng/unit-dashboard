"""
engine2.py -- Tile 2 (UNIT16 hole grid): run the camera pipeline on a clip.

Same idea as engine.py for Tile 1: there is no estimator in this file.
analyze() turns the clip into 320x200 gray frames, runs the AE3's own
u16pipe.py + u16c module over them with a real MicroPython binary
(t2/micropython, t2/t2_runner.py), then draws the overlay and counts what the
pipeline printed. Every number on the page comes from the camera code.

Files written into out_dir: annotated.mp4, frames.csv, series.json,
summary.json, u16pipe_as_run.py.
"""

import csv
import hashlib
import json
import math
import os
import re
import shutil
import stat
import subprocess
import time

import cv2
import numpy as np

import engine            # only for engine._h264 (H.264 re-encode) and _sanitize

HERE = os.path.dirname(os.path.abspath(__file__))
T2 = os.path.join(HERE, "t2")
MPY = os.path.join(T2, "micropython")
RUNNER = os.path.join(T2, "t2_runner.py")

ENGINE2_VERSION = "2026-10-25.1"
W, H = 320, 200

# status per frame, used by the page (series.status)
ST_NONE, ST_ROAD, ST_CROSS, ST_BOTH, ST_HOLD = 0, 1, 2, 3, 4


def pipeline_fingerprint():
    """Short hash of the camera files, so each run records which code made it."""
    h = hashlib.sha256()
    files = {}
    for name in ("u16pipe.py", "u16c.mpy", "unit16_codes.csv"):
        p = os.path.join(T2, name)
        if os.path.exists(p):
            b = open(p, "rb").read()
            files[name] = hashlib.sha256(b).hexdigest()[:12]
            h.update(b)
    return {"combined": h.hexdigest()[:12], "files": files}


def pipeline_constants():
    """A few settings read straight from u16pipe.py (shown on the page)."""
    out = {}
    try:
        src = open(os.path.join(T2, "u16pipe.py")).read()
        for key in ("MM_PER_PX", "LENS_K1", "PRES_MIN", "ANG_TOL", "PERP_TOL",
                    "JUMP_MAX", "STRIDE"):
            m = re.search(r"^%s\s*(?:,\s*\w+\s*)?=\s*([0-9.]+)" % key, src, re.M)
            if m:
                out[key] = float(m.group(1))
    except OSError:
        pass
    return out


def _mpy_ready():
    if not os.path.exists(MPY):
        raise RuntimeError("t2/micropython is missing")
    mode = os.stat(MPY).st_mode
    if not mode & stat.S_IXUSR:
        try:
            os.chmod(MPY, mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
        except OSError:
            pass


# ------------------------------------------------------------------ overlay
def _draw(img, r, s):
    """Overlay for one frame (img is already scaled by s). Same picture as
    u16_live.py draws on the camera: road white, cross grey, junction cross."""
    th = r.get("th")
    lines = r.get("lines")
    if th is None or not lines:
        return
    c, sn = math.cos(th), math.sin(th)
    L = 400
    road_col, cross_col = (255, 255, 255), (60, 190, 255)
    P = lambda x, y: (int(round(x * s)), int(round(y * s)))
    if r.get("jx") is not None:
        x, y = r["jx"], r["jy"]
        cv2.line(img, P(x - L * c, y - L * sn), P(x + L * c, y + L * sn), road_col, 1, cv2.LINE_AA)
        cv2.line(img, P(x + L * sn, y - L * c), P(x - L * sn, y + L * c), cross_col, 1, cv2.LINE_AA)
        cx, cy = P(x, y)
        cv2.drawMarker(img, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 8 * s, 2)
        lab = r.get("code") or r.get("xnode")
        if lab:
            cv2.putText(img, lab, (cx + 6 * s, cy + 12 * s), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45 * s, (255, 255, 255), 1, cv2.LINE_AA)
    elif lines == "road" and r.get("Y") is not None:
        Y = r["Y"]
        x, y = 159.5 + Y * sn, 99.5 - Y * c
        cv2.line(img, P(x - L * c, y - L * sn), P(x + L * c, y + L * sn), road_col, 1, cv2.LINE_AA)
    elif lines == "cross" and r.get("X") is not None:
        X = r["X"]
        x, y = 159.5 - X * c, 99.5 - X * sn
        cv2.line(img, P(x + L * sn, y - L * c), P(x - L * sn, y + L * c), cross_col, 1, cv2.LINE_AA)


def _hud(img, r, k, s):
    f = lambda v: "--" if v is None else "%+.1f" % v
    if r.get("ok"):
        at = "" if r.get("X") is None else " @" + (r.get("xnode") or "?")
        txt = "X %s%s  Y %s  ang %+.1f  %s" % (f(r.get("Xmm")), at, f(r.get("Ymm")),
                                              r.get("deg") or 0.0, r.get("node") or "--")
        col = (255, 255, 255)
    elif r.get("lines"):
        txt, col = "HOLD (does not fit the expected position)", (60, 140, 255)
    else:
        txt, col = "no lock", (80, 80, 255)
    sc = 0.42 * s
    (tw, tht), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, sc, 1)
    cv2.rectangle(img, (0, 0), (tw + 10, tht + 10), (0, 0, 0), -1)
    cv2.putText(img, txt, (5, tht + 5), cv2.FONT_HERSHEY_SIMPLEX, sc, col, 1, cv2.LINE_AA)
    fr = "%d" % k
    (tw2, _), _ = cv2.getTextSize(fr, cv2.FONT_HERSHEY_SIMPLEX, sc, 1)
    cv2.putText(img, fr, (img.shape[1] - tw2 - 5, tht + 5), cv2.FONT_HERSHEY_SIMPLEX, sc,
                (200, 200, 200), 1, cv2.LINE_AA)


# ------------------------------------------------------------------ analyse
def analyze(video_path, out_dir, scale=3, progress=None):
    t0 = time.time()
    _mpy_ready()
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise RuntimeError("cannot open the video")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    if not (1 <= fps <= 1000):
        fps = 30.0
    raw_path = os.path.join(out_dir, "frames.raw")
    n = 0
    src_w = src_h = None
    resized = False
    with open(raw_path, "wb") as fo:
        while True:
            ok, fr = cap.read()
            if not ok:
                break
            g = cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY) if fr.ndim == 3 else fr
            if src_w is None:
                src_h, src_w = g.shape[:2]
            if g.shape[:2] != (H, W):
                g = cv2.resize(g, (W, H), interpolation=cv2.INTER_AREA)
                resized = True
            fo.write(np.ascontiguousarray(g, dtype=np.uint8).tobytes())
            n += 1
    cap.release()
    if n == 0:
        raise RuntimeError("no frames could be read from the video")
    if progress:
        progress(0, n)

    out_jsonl = os.path.join(out_dir, "frames.jsonl")
    prog_path = os.path.join(out_dir, "progress.json")
    t1 = time.time()
    proc = subprocess.Popen([MPY, RUNNER, raw_path, str(n), out_jsonl, prog_path],
                            cwd=T2, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    while proc.poll() is None:
        if progress and os.path.exists(prog_path):
            try:
                with open(prog_path) as f:
                    p = json.load(f)
                progress(int(0.8 * p.get("done", 0)), n)
            except Exception:
                pass
        time.sleep(0.25)
    out, err = proc.communicate()
    t_pipe = time.time() - t1
    if proc.returncode != 0:
        raise RuntimeError("camera pipeline failed: " + (err or out or "").strip()[-600:])

    recs = []
    with open(out_jsonl) as f:
        for ln in f:
            ln = ln.strip()
            if ln:
                recs.append(json.loads(ln))
    if len(recs) != n:
        raise RuntimeError("pipeline returned %d of %d frames" % (len(recs), n))

    # ---- annotated video
    s = int(scale)
    tmp_vid = os.path.join(out_dir, "_annot_raw.mp4")
    vw = cv2.VideoWriter(tmp_vid, cv2.VideoWriter_fourcc(*"mp4v"), fps, (W * s, H * s))
    with open(raw_path, "rb") as fi:
        for k in range(n):
            g = np.frombuffer(fi.read(W * H), dtype=np.uint8).reshape(H, W)
            img = cv2.cvtColor(cv2.resize(g, (W * s, H * s), interpolation=cv2.INTER_LINEAR),
                               cv2.COLOR_GRAY2BGR)
            r = recs[k]
            _draw(img, r, s)
            _hud(img, r, k, s)
            vw.write(img)
            if progress and k % 20 == 0:
                progress(int(0.8 * n + 0.2 * k), n)
    vw.release()
    final = os.path.join(out_dir, "annotated.mp4")
    playable = engine._h264(tmp_vid, final)
    if playable:
        os.remove(tmp_vid)
    else:
        shutil.move(tmp_vid, final)
    os.remove(raw_path)
    for stale in ("progress.json",):
        p = os.path.join(out_dir, stale)
        if os.path.exists(p):
            os.remove(p)

    # ---- per-frame series + csv
    t = [round(k / fps, 4) for k in range(n)]

    def status(r):
        if not r.get("ok"):
            return ST_HOLD if r.get("lines") else ST_NONE
        return {"both": ST_BOTH, "road": ST_ROAD, "cross": ST_CROSS}.get(r.get("lines"), ST_NONE)

    series = {
        "t": t,
        "x": [r["Xmm"] if r.get("ok") else None for r in recs],
        "y": [r["Ymm"] if r.get("ok") else None for r in recs],
        "deg": [r["deg"] if r.get("ok") else None for r in recs],
        "pres": [r["pres"] if (r.get("pres") is not None and r["pres"] >= 0) else None for r in recs],
        "status": [status(r) for r in recs],
        "code": [r.get("code") for r in recs],
        "node": [r.get("node") for r in recs],
        "xnode": [r.get("xnode") if (r.get("ok") and r.get("X") is not None) else None for r in recs],
    }
    with open(os.path.join(out_dir, "series.json"), "w") as f:
        json.dump(series, f)

    cols = ["frame", "t_s", "ok", "status", "lines", "mode", "X_mm", "Y_mm", "X_px", "Y_px",
            "angle_deg", "junction_x_px", "junction_y_px", "code_this_frame", "code_raw",
            "presence", "x_from_junction", "last_code"]
    with open(os.path.join(out_dir, "frames.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        names = {ST_NONE: "none", ST_ROAD: "road", ST_CROSS: "cross", ST_BOTH: "both", ST_HOLD: "hold"}
        for k, r in enumerate(recs):
            w.writerow([k, t[k], r.get("ok"), names[series["status"][k]], r.get("lines") or "",
                        r.get("mode") or "", r.get("Xmm"), r.get("Ymm"), r.get("X"), r.get("Y"),
                        r.get("deg"), r.get("jx"), r.get("jy"), r.get("code") or "",
                        r.get("raw") or "", r.get("pres"), series["xnode"][k] or "",
                        r.get("node") or ""])

    shutil.copyfile(os.path.join(T2, "u16pipe.py"), os.path.join(out_dir, "u16pipe_as_run.py"))

    # ---- summary
    st = series["status"]
    cnt = {k: st.count(v) for k, v in
           (("both", ST_BOTH), ("road", ST_ROAD), ("cross", ST_CROSS), ("hold", ST_HOLD), ("none", ST_NONE))}
    pct = lambda a: round(100.0 * a / n, 1)
    codes = {}
    for k, r in enumerate(recs):
        c = r.get("code")
        if c:
            e = codes.setdefault(c, {"name": c, "frames": 0, "first_frame": k, "first_t": t[k]})
            e["frames"] += 1
    last = recs[-1]
    xs = [v for v in series["x"] if v is not None]
    ys = [v for v in series["y"] if v is not None]
    degs = [v for v in series["deg"] if v is not None]
    summary = {
        "frames": n, "fps": round(fps, 2), "duration_s": round(n / fps, 2),
        "width": src_w, "height": src_h, "resized": resized,
        "processing_s": round(time.time() - t0, 1),
        "pipeline_s": round(t_pipe, 2), "ms_per_frame": round(1000.0 * t_pipe / n, 2),
        "browser_playable": bool(playable),
        "ok_pct": pct(sum(1 for r in recs if r.get("ok"))),
        "counts": cnt, "pcts": {k: pct(v) for k, v in cnt.items()},
        "x_pct": pct(len(xs)), "y_pct": pct(len(ys)),
        "x_unnamed_pct": pct(sum(1 for k in range(n) if series["x"][k] is not None and not series["xnode"][k])),
        "angle_start": degs[0] if degs else None, "angle_end": degs[-1] if degs else None,
        "angle_min": min(degs) if degs else None, "angle_max": max(degs) if degs else None,
        "codes": sorted(codes.values(), key=lambda e: e["first_frame"]),
        "guards": {"angle": last["ra"], "perp": last["rp"], "jump": last["rj"],
                   "plain": last["rpl"], "conflicts": last["cf"], "searches": last["se"]},
        "pipeline": pipeline_fingerprint(), "constants": pipeline_constants(),
        "engine2": ENGINE2_VERSION,
    }
    summary = engine._sanitize(summary)
    with open(os.path.join(out_dir, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    os.remove(out_jsonl)
    return summary
