"""
run_on_video.py -- run the REAL camera script over recorded MP4s and
write an annotated video for each one.

The point of this file: it does NOT reimplement the algorithm. It boots
scan_fit_fps_live.py unmodified, with a fake OpenMV camera that serves
frames from a video file instead of the sensor. Every scan, fit,
threshold and gate is literally the code that runs on the AE3, so the
two can never drift apart. This file only decodes video, records what
the script asked to draw, and paints it back on.

USAGE
    python3 run_on_video.py                      # whole IN_DIR
    python3 run_on_video.py path/to/one.mp4      # a single file
    python3 run_on_video.py path/to/folder       # every mp4 in it

OUTPUT, per input video, in OUT_DIR:
    <name>_annotated.mp4   the frames with the overlay drawn on
    <name>.csv             frame, mode, X_mm, Y_mm, theta_deg, gap_px
    <name>.log             everything the script printed
    <name>.diag            per-side tilt diagnosis, measured on the RAW
                           decoded frames (before any re-encoding)

Input frame rate is read from each file and reused for the output, so
30 fps and 60 fps sources both come out at their own speed. Nothing in
the algorithm depends on timing -- it is frame by frame, and the only
counters in it (REACQUIRE_EVERY, REACQUIRE_MINI_EVERY) count frames.

REQUIREMENTS
    pip3 install opencv-python numpy

NOTE ON RE-ANALYSING IDE RECORDINGS
If the input was recorded from the OpenMV IDE while the script was
running, the old overlay is already burned into those pixels. The lines
are thin and it still works, but the cleanest input is a recording made
with the frame buffer overlay off.
"""

import os
import sys
import io
import glob
import contextlib
import re

import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import mock_openmv as M

# ------------------------------------------------------------ EDIT THESE
IN_DIR = ("/Users/saharfishler/Documents/Documents - sahar's MacBook Pro"
          "/robo_ae3/tests")
OUT_DIR = os.path.join(IN_DIR, "output vids")
SCRIPT = os.path.join(HERE, "scan_fit_fps_live.py")
SCALE = 3           # upscale the output video so the overlay is legible
DISPLAY_STRETCH = 1 # brighten the PICTURE ONLY. Your frames are dark
                    # (mean ~41, max ~204), which makes the annotated
                    # video look murky even when detection is perfect.
                    # This is a display-time contrast stretch applied
                    # AFTER the algorithm has already run on the
                    # original pixels, so it cannot change a single
                    # detection -- it only makes the video watchable.
TRACE = 1           # draw a rolling X trace along the bottom, so
                    # flicker is visible at a glance instead of having
                    # to watch the number
COL = {"mini": (120, 255, 120), "junction": (120, 200, 255),
       "single-bar": (60, 200, 255), "none": (120, 120, 120)}
# ----------------------------------------------------------------------

W, H = 320, 200


def load(path):
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise IOError("could not open %s" % path)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    out = []
    while True:
        ok, fr = cap.read()
        if not ok:
            break
        g = cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY)
        if g.shape != (H, W):
            g = cv2.resize(g, (W, H), interpolation=cv2.INTER_AREA)
        out.append(g)
    cap.release()
    return out, fps


def bgr(c):
    """OpenMV passes (r,g,b); OpenCV wants (b,g,r)."""
    if c is None:
        return (255, 255, 255)
    r, g, b = c
    return (int(b), int(g), int(r))


def stretch(gray):
    """Display-only contrast stretch. Never fed back to the algorithm."""
    lo = float(np.percentile(gray, 2))
    hi = float(np.percentile(gray, 99.5))
    if hi - lo < 12:
        return gray
    out = (gray.astype(np.float32) - lo) * (255.0 / (hi - lo))
    return np.clip(out, 0, 255).astype(np.uint8)


def paint(gray, log, scale=SCALE, mode="none", trace=None, idx=0):
    shown = stretch(gray) if DISPLAY_STRETCH else gray
    im = cv2.cvtColor(shown, cv2.COLOR_GRAY2BGR)
    im = cv2.resize(im, (W * scale, H * scale),
                    interpolation=cv2.INTER_LANCZOS4)
    for t, c in log.get("lines", []):
        x1, y1, x2, y2 = [int(v) for v in t]
        cv2.line(im, (x1 * scale, y1 * scale), (x2 * scale, y2 * scale),
                 (0, 0, 0), max(2, scale), cv2.LINE_AA)      # halo
        cv2.line(im, (x1 * scale, y1 * scale), (x2 * scale, y2 * scale),
                 bgr(c), max(1, scale // 2), cv2.LINE_AA)
    for (pt, c, size) in log.get("crosses", []):
        x, y = int(pt[0]) * scale, int(pt[1]) * scale
        s = int(size) * scale
        cv2.line(im, (x - s, y), (x + s, y), bgr(c), max(1, scale // 2))
        cv2.line(im, (x, y - s), (x, y + s), bgr(c), max(1, scale // 2))
        cv2.circle(im, (x, y), max(2, scale), bgr(c), -1)
    # The script's own text goes on a clean black band above the frame
    # rather than on top of the picture. Recorded IDE videos usually
    # already have the old overlay burned in, and text on text is
    # unreadable.
    band = np.zeros((22 * scale, W * scale, 3), np.uint8)
    txt = log.get("strings_full", [])
    col = COL.get(mode, (200, 200, 200))
    cv2.rectangle(band, (0, 0), (6 * scale, 22 * scale), col, -1)
    label = txt[-1][1] if txt else "no marker"
    cv2.putText(band, label, (11 * scale, 15 * scale),
                cv2.FONT_HERSHEY_SIMPLEX, 0.30 * scale, col,
                max(1, scale // 3), cv2.LINE_AA)
    parts = [band, im]

    if TRACE and trace is not None:
        th_ = 30 * scale
        foot = np.zeros((th_, W * scale, 3), np.uint8)
        seen = [(i, v, m) for i, v, m in trace if v is not None]
        if seen:
            span = 120                     # frames of history shown
            i0 = max(0, idx - span)
            for gy in (0.0, 0.5, 1.0):
                y = int(th_ - 4 - gy * (th_ - 8))
                cv2.line(foot, (0, y), (W * scale, y), (40, 40, 40), 1)
            pts = [(i, v, m) for i, v, m in seen if i >= i0]
            prev = None
            for i, v, m in pts:
                x = int((i - i0) / float(span) * (W * scale - 1))
                y = int(th_ - 4 - (v / 80.0) * (th_ - 8))
                y = max(2, min(th_ - 2, y))
                c = COL.get(m, (200, 200, 200))
                if prev is not None and abs(prev[2] - v) < 20:
                    cv2.line(foot, (prev[0], prev[1]), (x, y), c, 1,
                             cv2.LINE_AA)
                cv2.circle(foot, (x, y), 1, c, -1)
                prev = (x, y, v)
        cv2.putText(foot, "X 0-80mm", (4, 12), cv2.FONT_HERSHEY_SIMPLEX,
                    0.32, (110, 110, 110), 1, cv2.LINE_AA)
        parts.append(foot)
    return np.vstack(parts)


MIN_LEN, LONG_CUT, STEP = 4, 60, 5
THICK_LO, THICK_HI = 0.60, 1.35


def _otsu(a):
    h = np.bincount(a.ravel(), minlength=256).astype(float)
    tot = h.sum(); om = np.cumsum(h); mu = np.cumsum(h * np.arange(256))
    den = om * (tot - om); den[den == 0] = 1e-9
    return int(np.argmax((mu[-1] * om - mu * tot) ** 2 / den))


def diagnose(frames):
    """Fit the horizontal bar SEPARATELY each side of the junction.

    Run on the raw decoded frames, using the same point selection the
    camera script uses. If the two sides disagree, the tape is not
    collinear and no single fit can be right. If they agree with each
    other but not with the combined fit, something near the junction is
    pulling it. Different causes, different fixes.
    """
    rows = []
    heads = []
    for gray in frames:
        th = _otsu(gray[::4, ::4])
        b = gray > th
        heads.append(int(gray.max()) - th)
        P = []; hist = np.zeros(256, int); vc = []
        for x in range(2, W, STEP):
            c = b[:, x]
            d = np.diff(np.concatenate(([0], c.astype(int), [0])))
            for p, q in zip(np.where(d == 1)[0], np.where(d == -1)[0]):
                L = q - p
                if L >= LONG_CUT:
                    vc.append(x)
                elif L >= MIN_LEN:
                    hist[L] += 1; P.append((x, (p + q) / 2.0, L))
        if not P or not vc:
            continue
        tc = int(hist.argmax())
        lo, hi = int(tc * THICK_LO), int(tc * THICK_HI)
        Q = np.array([(x, y) for x, y, L in P if lo <= L <= hi], float)
        if len(Q) < 25:
            continue
        b0, b1 = min(vc), max(vc)
        X, Y = Q[:, 0], Q[:, 1]
        lm = X < b0 - 10; rm = X > b1 + 10
        if lm.sum() < 8 or rm.sum() < 8:
            continue
        cl = np.polyfit(X[lm], Y[lm], 1)
        cr = np.polyfit(X[rm], Y[rm], 1)
        ca = np.polyfit(X, Y, 1)
        jx = (b0 + b1) / 2.0
        rows.append((np.degrees(np.arctan(cl[0])),
                     np.degrees(np.arctan(cr[0])),
                     np.degrees(np.arctan(ca[0])),
                     np.polyval(cr, jx) - np.polyval(cl, jx),
                     np.std(Y - np.polyval(ca, X)), tc,
                     int(lm.sum()), int(rm.sum())))
    if not rows:
        return "no frames with a junction and bar on both sides\n"
    A = np.array(rows)
    d = A[:, 0] - A[:, 1]
    out = []
    out.append("frames analysed with a junction: %d" % len(A))
    out.append("bar thickness tc: median %d px" % np.median(A[:, 5]))
    out.append("brightness headroom above Otsu: median %d" % np.median(heads))
    out.append("")
    out.append("LEFT  segment tilt : median %+.2f   IQR %+.2f .. %+.2f deg"
               % (np.median(A[:, 0]), *np.percentile(A[:, 0], [25, 75])))
    out.append("RIGHT segment tilt : median %+.2f   IQR %+.2f .. %+.2f deg"
               % (np.median(A[:, 1]), *np.percentile(A[:, 1], [25, 75])))
    out.append("COMBINED fit tilt  : median %+.2f deg" % np.median(A[:, 2]))
    out.append("LEFT minus RIGHT   : median %+.2f deg,  |diff|>1deg on %.0f%%"
               "  of frames, >3deg on %.0f%%"
               % (np.median(d), 100 * (np.abs(d) > 1).mean(),
                  100 * (np.abs(d) > 3).mean()))
    out.append("step at junction   : median %+.1f px" % np.median(A[:, 3]))
    out.append("point scatter about the combined fit: median %.2f px,"
               " p90 %.2f px" % (np.median(A[:, 4]),
                                 np.percentile(A[:, 4], 90)))
    out.append("points per side    : left median %d, right median %d"
               % (np.median(A[:, 6]), np.median(A[:, 7])))
    out.append("")
    out.append("READING IT:")
    out.append("  scatter under ~1px  -> points are clean, trust the rest")
    out.append("  scatter over ~3px   -> the points themselves are noisy;")
    out.append("                         fix that before chasing tilt")
    out.append("  |LEFT-RIGHT| small, COMBINED differs -> something at the")
    out.append("                         junction is pulling the fit")
    out.append("  |LEFT-RIGHT| large  -> the two tape pieces disagree;")
    out.append("                         no single line can fit both")
    return "\n".join(out) + "\n"


PAT_X = re.compile(r"X([+-][\d.]+)")
PAT_Y = re.compile(r"Y([+-][\d.]+)")
PAT_T = re.compile(r"T([+-][\d.]+)")
PAT_G = re.compile(r"g(\d+)px")


def parse(log):
    s = log.get("strings", [None])[-1] if log.get("strings") else None
    if not s:
        return ("none", "", "", "", "")
    mode = ("mini" if "(mini" in s
            else "junction" if "(J)" in s
            else "single-bar")
    def f(p):
        m = p.search(s)
        return m.group(1) if m else ""
    return (mode, f(PAT_X), f(PAT_Y), f(PAT_T), f(PAT_G))


def process(path, out_dir):
    name = os.path.splitext(os.path.basename(path))[0]
    frames, fps = load(path)
    if not frames:
        print("  %s: no frames decoded, skipped" % name)
        return
    drv = M.VideoDriver(frames)
    # drive the fake clock at the video's real frame period so the
    # once-a-second summary lines in the .log land where they would
    # have on the camera. NOTE: the "fps" printed in that log is the
    # RECORDING rate of the video, not how fast this PC processed it,
    # and not the camera's live rate either.
    M.install(None, None, ms_per_frame=max(1, int(round(1000.0 / fps))),
              driver=drv)
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        try:
            M.run_script(SCRIPT, drv)
        except SystemExit as e:
            print("script stopped: %s" % e)
    log_txt = buf.getvalue()

    os.makedirs(out_dir, exist_ok=True)
    vpath = os.path.join(out_dir, name + "_annotated.mp4")
    vw = cv2.VideoWriter(vpath, cv2.VideoWriter_fourcc(*"mp4v"),
                         fps, (W * SCALE,
                               (H + 22) * SCALE + (30 * SCALE if TRACE else 0)))
    rows = []
    counts = {}
    parsed = [parse(lg) for lg in drv.frames]
    trace = [(i, (float(p[1]) if p[1] else None), p[0])
             for i, p in enumerate(parsed)]
    for i, lg in enumerate(drv.frames):
        mode, x, y, t, g = parsed[i]
        vw.write(paint(frames[i], lg, mode=mode, trace=trace, idx=i))
        counts[mode] = counts.get(mode, 0) + 1
        rows.append("%d,%s,%s,%s,%s,%s" % (i, mode, x, y, t, g))
    vw.release()

    with open(os.path.join(out_dir, name + ".csv"), "w") as fh:
        fh.write("frame,mode,X_mm,Y_mm,theta_deg,gap_px\n")
        fh.write("\n".join(rows) + "\n")
    with open(os.path.join(out_dir, name + ".log"), "w") as fh:
        fh.write(log_txt)
    with open(os.path.join(out_dir, name + ".diag"), "w") as fh:
        fh.write(diagnose(frames))

    xs = [float(r.split(",")[2]) for r in rows if r.split(",")[2]]
    jump = max([abs(xs[i] - xs[i - 1]) for i in range(1, len(xs))],
               default=0.0)
    print("  %-28s %4d frames  %-46s  X on %d%% of frames  worst step %.1f mm"
          % (name, len(frames), counts, 100 * len(xs) // max(1, len(rows)),
             jump))


def main():
    args = sys.argv[1:]
    target = args[0] if args else IN_DIR
    if os.path.isdir(target):
        vids = sorted(glob.glob(os.path.join(target, "*.mp4")))
        out = (OUT_DIR if os.path.abspath(target) == os.path.abspath(IN_DIR)
               else os.path.join(target, "output vids"))
    else:
        vids = [target]
        out = OUT_DIR if os.path.isdir(OUT_DIR) else os.path.join(
            os.path.dirname(target), "output vids")
    if not vids:
        print("no .mp4 found in %s" % target)
        return
    print("script : %s" % SCRIPT)
    print("output : %s\n" % out)
    for v in vids:
        try:
            process(v, out)
        except Exception as e:
            print("  %-28s FAILED: %s: %s"
                  % (os.path.basename(v), type(e).__name__, e))


if __name__ == "__main__":
    main()