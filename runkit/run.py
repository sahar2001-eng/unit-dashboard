#!/usr/bin/env python3
"""run.py -- make runs for the site's Runs page, and upload them.

  python3 run.py video clip.mp4 --tile 2
        The camera's own code (gridcam, the version in kit.json) over every
        frame of the video. Turns: --turn FRAME L|R and --drive FRAME, as if
        the STM32 had sent them (V 20 1/2 and V 20 0).
  python3 run.py record --cam 192.168.4.1
        Saves the camera's live stream while the robot drives. Ctrl+C stops.
  python3 run.py upload runs/<folder>
        Sends a run folder to the site. Asks for the upload password
        (or set it once in the terminal: export RUNS_TOKEN=...).

Add --upload to video/record to upload right after.

'video' and 'record' need Docker Desktop (the camera code is built for
Intel-type x64 chips; Docker runs it on an M-chip Mac). Only Python 3 is
needed on the Mac itself, nothing to pip install.
"""

import argparse
import getpass
import hashlib
import http.client
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.parse
import urllib.request
import uuid
import zipfile

KIT = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(KIT, "runs")
SITE = "https://unit-rotation-analytics.onrender.com"
RUN_FILES = ("info.json", "frames.csv", "video.mp4", "log.txt")


def say(*a):
    print(*a, flush=True)


def die(msg):
    say("STOP: " + msg)
    sys.exit(1)


def slug(s):
    s = re.sub(r"[^A-Za-z0-9._-]+", "-", s).strip("-")
    return s[:60] or "run"


def new_folder(name):
    os.makedirs(RUNS, exist_ok=True)
    d = os.path.join(RUNS, time.strftime("%Y%m%d-%H%M%S-") + slug(name))
    os.makedirs(d)
    return d


# ------------------------------------------------------------------ docker
def kit_hash():
    h = hashlib.sha1()
    for base in ("Dockerfile", "engine.py", "kit.json"):
        h.update(open(os.path.join(KIT, base), "rb").read())
    for root, dirs, files in os.walk(os.path.join(KIT, "cam")):
        dirs[:] = sorted(d for d in dirs if d != "__pycache__")
        for f in sorted(files):
            p = os.path.join(root, f)
            h.update(os.path.relpath(p, KIT).encode())
            h.update(open(p, "rb").read())
    return h.hexdigest()[:10]


def image():
    """The kit's Docker image, built the first time (and after a kit change)."""
    if os.environ.get("RUNKIT_LOCAL"):      # testing on a Linux x64 PC: no Docker
        return None
    if not shutil.which("docker"):
        die("Docker is not installed. Install Docker Desktop (docker.com), open it once, then run this again.")
    if subprocess.run(["docker", "info"], capture_output=True).returncode:
        die("Docker is installed but not running. Open Docker Desktop, wait until it says it is running, then try again.")
    tag = "gridcam-runkit:" + kit_hash()
    if subprocess.run(["docker", "image", "inspect", tag], capture_output=True).returncode:
        say("building the run kit image (first time only, a few minutes) ...")
        r = subprocess.run(["docker", "build", "--platform", "linux/amd64", "-t", tag, KIT])
        if r.returncode:
            die("the Docker build failed (see above)")
    return tag


def in_docker(tag, mounts, args):
    if tag is None:                         # RUNKIT_LOCAL: same engine, here
        for host, inside, _ in mounts:
            args = [host + x[len(inside):] if x.startswith(inside) else x for x in args]
        r = subprocess.run([sys.executable, os.path.join(KIT, "engine.py")] + args)
        if r.returncode:
            die("the run kit stopped with an error (see above)")
        return
    cmd = ["docker", "run", "--rm", "--platform", "linux/amd64"]
    for host, inside, ro in mounts:
        cmd += ["-v", "%s:%s%s" % (host, inside, ":ro" if ro else "")]
    r = subprocess.run(cmd + [tag] + args)
    if r.returncode:
        die("the run kit stopped with an error (see above)")


# ------------------------------------------------------------------ video
def cmd_video(a):
    clip = os.path.abspath(a.clip)
    if not os.path.isfile(clip):
        die("no such file: " + a.clip)
    cmds = []
    for f, d in a.turn or []:
        d = d.upper()
        if d not in ("L", "R"):
            die("--turn FRAME L|R (L = left, R = right)")
        cmds.append([int(f), "V 20 %d" % (1 if d == "L" else 2)])
    for f in a.drive or []:
        cmds.append([int(f), "V 20 0"])
    for f, line in a.cmd or []:
        cmds.append([int(f), line])
    cmds.sort(key=lambda c: c[0])
    name = a.name or os.path.splitext(os.path.basename(clip))[0]
    tag = image()
    out = new_folder(name)
    args = ["video", "/in/" + os.path.basename(clip), "/out", "--tile", str(a.tile),
            "--name", name, "--quality", "0" if a.no_quality else "1"]
    if a.notes:
        args += ["--notes", a.notes]
    for f, line in cmds:
        args += ["--cmd", str(f), line]
    in_docker(tag, [(os.path.dirname(clip), "/in", True), (out, "/out", False)], args)
    say("run folder: " + out)
    if a.upload:
        upload(out, a.site, a.case)


# ------------------------------------------------------------------ record
def get_json(url, timeout=3):
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return json.loads(r.read().decode())
    except Exception:                       # noqa: BLE001
        return None


def cmd_record(a):
    host = a.cam
    if not host.startswith("http"):
        host = "http://" + host
    host = host.rstrip("/")
    tel = get_json(host + "/telemetry") or {}
    ver = ""
    if isinstance(tel, dict):
        ver = str(tel.get("version") or (tel.get("app") or {}).get("version") or "")
    name = a.name or "drive"
    out = new_folder(name)
    rawd = os.path.join(out, "raw")
    os.makedirs(rawd)
    hdr = {"name": name, "cam": host, "created": time.time(), "app_version": ver,
           "tile": a.tile or 0, "notes": a.notes or ""}
    json.dump(hdr, open(os.path.join(rawd, "header.json"), "w"), indent=1)
    say("connecting to %s/stream ..." % host)
    try:
        resp = urllib.request.urlopen(host + "/stream", timeout=10)
    except Exception as e:                  # noqa: BLE001
        shutil.rmtree(out, ignore_errors=True)
        die("cannot open the camera stream: %s" % e)
    say("recording. Press Ctrl+C to stop.")
    meta = open(os.path.join(rawd, "meta.jsonl"), "w")
    n, t_end = 0, (time.time() + a.seconds) if a.seconds else None
    try:
        while t_end is None or time.time() < t_end:
            line = resp.readline()
            if not line:
                say("the camera closed the stream")
                break
            if not line.startswith(b"--"):
                continue
            heads = {}
            while True:
                h = resp.readline()
                if not h or h in (b"\r\n", b"\n"):
                    break
                k, _, v = h.decode("latin-1").partition(":")
                heads[k.strip().lower()] = v.strip()
            ln = int(heads.get("content-length", "0"))
            if ln <= 0:
                continue
            jpg = resp.read(ln)
            t = time.time()
            kv = dict(p.split("=", 1) for p in heads.get("x-gridcam", "").split() if "=" in p)
            fn = "%06d.jpg" % n
            with open(os.path.join(rawd, fn), "wb") as f:
                f.write(jpg)
            meta.write(json.dumps({"file": fn, "pc_t": t, "kv": kv}) + "\n")
            n += 1
            if n % 50 == 0:
                say("  %d pictures" % n)
    except KeyboardInterrupt:
        pass
    finally:
        meta.close()
        try:
            resp.close()
        except Exception:                   # noqa: BLE001
            pass
    say("stopped: %d pictures" % n)
    if not n:
        shutil.rmtree(out, ignore_errors=True)
        die("nothing was recorded")
    in_docker(image(), [(out, "/out", False)], ["pack", "/out"])
    say("run folder: " + out)
    if a.upload:
        upload(out, a.site, a.case)


# ------------------------------------------------------------------ upload
def token():
    t = os.environ.get("RUNS_TOKEN", "")
    if not t:
        t = getpass.getpass("Upload password (nothing shows while you type): ")
    return t.strip()


def upload(folder, site, case=None, name=None):
    folder = os.path.abspath(folder)
    miss = [f for f in RUN_FILES[:3] if not os.path.isfile(os.path.join(folder, f))]
    if miss:
        die("%s is not a run folder (missing %s)" % (folder, ", ".join(miss)))
    tok = token()
    zpath = os.path.join(tempfile.mkdtemp(), os.path.basename(folder) + ".zip")
    with zipfile.ZipFile(zpath, "w", zipfile.ZIP_STORED) as z:   # mp4 is compressed already
        for f in RUN_FILES:
            p = os.path.join(folder, f)
            if os.path.isfile(p):
                z.write(p, f)
    size = os.path.getsize(zpath)
    say("uploading %s (%.1f MB) to %s ..." % (os.path.basename(folder), size / 1e6, site))
    b = uuid.uuid4().hex
    fields = []
    if case:
        fields.append(("case", case))
    if name:
        fields.append(("name", name))
    pre = b"".join(("--%s\r\nContent-Disposition: form-data; name=\"%s\"\r\n\r\n%s\r\n" % (b, k, v)).encode()
                   for k, v in fields)
    pre += ("--%s\r\nContent-Disposition: form-data; name=\"run\"; filename=\"%s\"\r\n"
            "Content-Type: application/zip\r\n\r\n" % (b, os.path.basename(zpath))).encode()
    post = ("\r\n--%s--\r\n" % b).encode()
    u = urllib.parse.urlparse(site)
    C = http.client.HTTPSConnection if u.scheme == "https" else http.client.HTTPConnection
    c = C(u.netloc, timeout=600)
    c.putrequest("POST", (u.path.rstrip("/") or "") + "/api/runs/upload")
    c.putheader("Content-Type", "multipart/form-data; boundary=" + b)
    c.putheader("Content-Length", str(len(pre) + size + len(post)))
    c.putheader("X-Runs-Token", tok)
    c.endheaders()
    c.send(pre)
    sent = 0
    with open(zpath, "rb") as f:
        while True:
            chunk = f.read(1 << 20)
            if not chunk:
                break
            c.send(chunk)
            sent += len(chunk)
            sys.stdout.write("\r  %d%%" % (100 * sent // size))
            sys.stdout.flush()
    c.send(post)
    say("")
    r = c.getresponse()
    body = r.read().decode("utf-8", "replace")
    try:
        j = json.loads(body)
    except ValueError:
        die("the site answered %d: %s" % (r.status, body[:300]))
    if j.get("error"):
        die("the site said: " + j["error"])
    say("uploaded: %s/runs?run=%s" % (site.rstrip("/"), j["id"]))


def cmd_upload(a):
    upload(a.folder, a.site, a.case, a.name)


def main():
    ap = argparse.ArgumentParser(prog="run.py", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="what", required=True)

    def common(p, up=True):
        p.add_argument("--site", default=os.environ.get("RUNS_SITE", SITE), help="the site (default %(default)s)")
        p.add_argument("--case", help="use case id to tag the run with (e.g. 2.3)")
        if up:
            p.add_argument("--upload", action="store_true", help="upload right after")
            p.add_argument("--notes", help="a note saved with the run")

    v = sub.add_parser("video", help="the camera code over a video")
    v.add_argument("clip")
    v.add_argument("--tile", type=int, choices=(1, 2), required=True)
    v.add_argument("--turn", nargs=2, action="append", metavar=("FRAME", "L|R"),
                   help="turn command at this frame (V 20 1 = L, V 20 2 = R)")
    v.add_argument("--drive", type=int, action="append", metavar="FRAME",
                   help="back to drive at this frame (V 20 0)")
    v.add_argument("--cmd", nargs=2, action="append", metavar=("FRAME", "LINE"),
                   help='any V command at this frame, e.g. --cmd 300 "V 5 vision.tile 2"')
    v.add_argument("--name")
    v.add_argument("--no-quality", action="store_true", help="quality score off")
    common(v)

    r = sub.add_parser("record", help="save the live stream of a robot drive")
    r.add_argument("--cam", required=True, help="camera address, e.g. 192.168.4.1")
    r.add_argument("--name")
    r.add_argument("--tile", type=int, choices=(1, 2))
    r.add_argument("--seconds", type=float, help="stop after this many seconds")
    common(r)

    u = sub.add_parser("upload", help="send a run folder to the site")
    u.add_argument("folder")
    u.add_argument("--name")
    common(u, up=False)

    a = ap.parse_args()
    {"video": cmd_video, "record": cmd_record, "upload": cmd_upload}[a.what](a)


if __name__ == "__main__":
    main()
