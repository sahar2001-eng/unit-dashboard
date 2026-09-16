"""
engine.py -- run the flight code on a clip and package what it produced.

There is no estimator in this file. analyze() launches flight_runner.py in a
subprocess (the flight code replaces `time`, so it must not share a process
with Flask), waits for it, then turns its per-frame records into the files
the page reads. Every number on the page is a number the robot's own code
printed or drew; this file only counts them.

Turn commands: the robot does not detect turns, a host tells it. So a clip
with a turn in it is uploaded with the frame the host would have sent
"TURN L" (or R) on, and optionally the frame it would have sent "DRIVE".
Those go to the flight code as SIM_CMDS, the bench hook it has for exactly
this.
"""

import csv
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
FLIGHT = os.path.join(HERE, "flight")
RUNNER = os.path.join(HERE, "flight_runner.py")

ENGINE_VERSION = "2026-09-16.1"   # this file. The flight code has its own
                                  # fingerprint, see flight_fingerprint()

# The flight code consumes two frames at start-up before its loop begins,
# so its frame_id is the video index minus this.
FLIGHT_FRAME_OFFSET = 2

TIER_CODES = {"none": 0, "junction": 1, "Y-only": 2, "X-only": 3, "one bar": 4}


def flight_fingerprint():
    """Short hash of the four flight files, so a run records exactly which
    code produced it."""
    h = hashlib.sha256()
    files = {}
    for name in ("main.py", "turn_sm.py", "rot_scan.py", "hud.py"):
        p = os.path.join(FLIGHT, name)
        if os.path.exists(p):
            b = open(p, "rb").read()
            files[name] = hashlib.sha256(b).hexdigest()[:12]
            h.update(b)
    return {"combined": h.hexdigest()[:12], "files": files}


def _sanitize(obj):
    """Plain JSON only (no numpy types)."""
    if isinstance(obj, dict):
        return {k: _sanitize(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_sanitize(v) for v in obj]
    if hasattr(obj, "item"):
        return obj.item()
    return obj


def _h264(src, dst):
    exe = shutil.which("ffmpeg")
    if exe is None:
        try:
            from imageio_ffmpeg import get_ffmpeg_exe
            exe = get_ffmpeg_exe()
        except Exception:
            return False
    try:
        subprocess.run([exe, "-y", "-loglevel", "error", "-i", src,
                        "-c:v", "libx264", "-pix_fmt", "yuv420p",
                        "-preset", "veryfast", "-crf", "20",
                        "-movflags", "+faststart", dst],
                       check=True, timeout=900)
        return os.path.exists(dst) and os.path.getsize(dst) > 0
    except Exception:
        return False


def analyze(video_path, out_dir, scale=3, turn=None, progress=None):
    """Run the flight code over `video_path`, write into `out_dir`:
    annotated.mp4, frames.csv, series.json, summary.json, terminal.txt.
    `turn` is None or {"start_frame", "dir": "L"|"R", "end_frame"|None},
    in VIDEO frame numbers. Returns the summary dict."""
    t0 = time.time()
    sim = []
    if turn and turn.get("start_frame") is not None:
        sf = max(0, int(turn["start_frame"]) - FLIGHT_FRAME_OFFSET)
        sim.append([sf, "TURN %s" % ("R" if turn.get("dir") == "R" else "L")])
        if turn.get("end_frame") is not None:
            ef = max(sf + 1, int(turn["end_frame"]) - FLIGHT_FRAME_OFFSET)
            sim.append([ef, "DRIVE"])

    for stale in ("progress.json", "error.json", "frames.jsonl", "run.json"):
        p = os.path.join(out_dir, stale)
        if os.path.exists(p):
            os.remove(p)

    proc = subprocess.Popen(
        [sys.executable, RUNNER, "--flight", FLIGHT, "--video", video_path,
         "--out", out_dir, "--scale", str(int(scale)), "--sim", json.dumps(sim)],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    prog_path = os.path.join(out_dir, "progress.json")
    while proc.poll() is None:
        if progress and os.path.exists(prog_path):
            try:
                with open(prog_path) as f:
                    p = json.load(f)
                progress(p.get("done", 0), p.get("total", 0))
            except Exception:
                pass
        time.sleep(0.25)
    _, err_out = proc.communicate()

    err_path = os.path.join(out_dir, "error.json")
    if os.path.exists(err_path):
        with open(err_path) as f:
            raise RuntimeError("flight code: " + json.load(f).get("error", "?"))
    if proc.returncode != 0:
        raise RuntimeError("flight runner exited %d: %s"
                           % (proc.returncode, (err_out or "")[-600:]))

    with open(os.path.join(out_dir, "run.json")) as f:
        run = json.load(f)
    recs = [json.loads(l) for l in open(os.path.join(out_dir, "frames.jsonl"))]
    n = len(recs)
    fps = float(run["fps"])
    if progress:
        progress(n, n)

    # ---- video
    raw = os.path.join(out_dir, "annotated_raw.mp4")
    playable = _h264(raw, os.path.join(out_dir, "annotated.mp4"))
    if playable:
        os.remove(raw)
    else:
        os.replace(raw, os.path.join(out_dir, "annotated.mp4"))

    # ---- csv: every record, as the robot reported it
    with open(os.path.join(out_dir, "frames.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "t_s", "mode", "tier", "x_mm", "y_mm",
                    "theta_deg", "heading_deg"])
        for r in recs:
            w.writerow([r["frame"], r["t"], r["mode"] or "", r["tier"],
                        "" if r["x"] is None else r["x"],
                        "" if r["y"] is None else r["y"],
                        "" if r["theta"] is None else r["theta"],
                        "" if r["heading"] is None else r["heading"]])

    # ---- counts (nothing derived, just tallies of the robot's outputs)
    def pct(k):
        return round(100.0 * k / max(n, 1), 1)

    drv = [r for r in recs if r["mode"] == "DRV"]
    rot = [r for r in recs if r["mode"] == "ROT"]
    nd = max(len(drv), 1)
    nr = max(len(rot), 1)
    headings = [r["heading"] for r in recs if r["heading"] is not None]

    # events straight from the terminal
    events = []
    term_path = os.path.join(out_dir, "terminal.txt")
    if os.path.exists(term_path):
        for line in open(term_path, errors="replace"):
            s = line.strip()
            for key in ("TURN_START", "TURN_SEED", "TURN_END", "JUNCTION_LANDED",
                        "REACQUIRE", "FAULT", "seed:", "rejected", "WARNING"):
                if key in s:
                    events.append(s[:160])
                    break

    # mode windows, for shading the charts
    windows = []
    cur = None
    for r in recs:
        m = r["mode"]
        if cur is None or cur["mode"] != m:
            if cur is not None:
                windows.append(cur)
            cur = {"mode": m, "start": r["frame"], "end": r["frame"]}
        else:
            cur["end"] = r["frame"]
    if cur is not None:
        windows.append(cur)

    summary = {
        "frames": n, "fps": round(fps, 3), "duration_s": round(n / fps, 2),
        "width": run["width"], "height": run["height"], "scale": run["scale"],
        "browser_playable": playable,
        "processing_s": round(time.time() - t0, 1),
        "ms_per_frame": round(1000.0 * (time.time() - t0) / max(n, 1), 1),

        "turn": turn or None,
        "sim_cmds": run.get("sim_cmds", []),

        "drv_pct": pct(len(drv)), "rot_pct": pct(len(rot)),

        "drive_junction_pct": round(100.0 * sum(1 for r in drv if r["tier"] == "junction") / nd, 1),
        "drive_y_only_pct": round(100.0 * sum(1 for r in drv if r["tier"] == "Y-only") / nd, 1),
        "drive_x_only_pct": round(100.0 * sum(1 for r in drv if r["tier"] == "X-only") / nd, 1),
        "drive_none_pct": round(100.0 * sum(1 for r in drv if r["tier"] == "none") / nd, 1),
        "rot_junction_pct": round(100.0 * sum(1 for r in rot if r["tier"] == "junction") / nr, 1),
        "rot_one_bar_pct": round(100.0 * sum(1 for r in rot if r["tier"] == "one bar") / nr, 1),
        "rot_none_pct": round(100.0 * sum(1 for r in rot if r["tier"] == "none") / nr, 1),

        "x_available_pct": pct(sum(1 for r in recs if r["x"] is not None)),
        "y_available_pct": pct(sum(1 for r in recs if r["y"] is not None)),
        "theta_available_pct": pct(sum(1 for r in recs if r["theta"] is not None)),

        "heading_start": headings[0] if headings else None,
        "heading_end": headings[-1] if headings else None,
        "heading_min": min(headings) if headings else None,
        "heading_max": max(headings) if headings else None,

        "events": events[:200],
        "mode_windows": windows,
        "engine_version": ENGINE_VERSION,
        "flight": flight_fingerprint(),
    }
    summary = _sanitize(summary)
    with open(os.path.join(out_dir, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)

    series = {
        "t": [r["t"] for r in recs],
        "mode": [1 if r["mode"] == "ROT" else (0 if r["mode"] == "DRV" else -1) for r in recs],
        "tier": [TIER_CODES.get(r["tier"], 0) for r in recs],
        "x": [r["x"] for r in recs],
        "y": [r["y"] for r in recs],
        "theta": [r["theta"] for r in recs],
        "heading": [r["heading"] for r in recs],
        "windows": windows,
    }
    with open(os.path.join(out_dir, "series.json"), "w") as f:
        json.dump(_sanitize(series), f, separators=(",", ":"))

    return summary
