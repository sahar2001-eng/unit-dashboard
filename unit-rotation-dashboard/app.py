#!/usr/bin/env python3
"""
UNIT -- live grid camera dashboard.

    .venv/bin/python app.py        (or: python3 app.py after pip install -r requirements.txt)

then open http://127.0.0.1:5050

Live x / y / theta from the camera over USB serial, live video from its MJPEG
stream over WiFi, and recordings saved under data/<id>/ with a name and notes.
See live.py for why the two feeds use two different links.
"""

import json
import os
import shutil
import time

from flask import (Flask, Response, abort, jsonify, render_template, request,
                   send_from_directory)
from werkzeug.utils import secure_filename

import live

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "data")
SETTINGS = os.path.join(HERE, "settings.json")
os.makedirs(DATA, exist_ok=True)

DEFAULT_SETTINGS = {
    # the camera's mDNS name is in its boot log: "hostname: gridcam-6020"
    "camera_url": os.environ.get("CAM_URL", "http://gridcam-6020.local:8080"),
    "serial_port": os.environ.get("CAM_SERIAL", ""),   # '' = find by USB ID
    "window_s": 20,                                     # chart history
}


def load_settings():
    s = dict(DEFAULT_SETTINGS)
    try:
        with open(SETTINGS) as f:
            s.update(json.load(f))
    except (OSError, ValueError):
        pass
    return s


def save_settings(s):
    with open(SETTINGS, "w") as f:
        json.dump(s, f, indent=2)


settings = load_settings()
serial_reader = live.SerialReader(settings["serial_port"])
stream_reader = live.StreamReader(settings["camera_url"])
recorder = live.Recorder(DATA, serial_reader, stream_reader)
serial_reader.start()
stream_reader.start()

app = Flask(__name__)


# ------------------------------------------------------------------ helpers
def session_dir(sid):
    d = os.path.join(DATA, secure_filename(sid))
    if not d.startswith(DATA):
        abort(400)
    return d


def read_meta(sid):
    return recorder.read_meta(secure_filename(sid))


def write_meta(sid, meta):
    recorder.write_meta(secure_filename(sid), meta)


def list_sessions():
    out = []
    for sid in os.listdir(DATA):
        m = read_meta(sid)
        if m and m.get("kind") == "live":
            out.append(m)
    out.sort(key=lambda m: m.get("created", 0), reverse=True)
    return out


def status():
    return {"serial": serial_reader.status(),
            "camera": stream_reader.status(),
            "recording": recorder.status(),
            "time": time.time()}


# -------------------------------------------------------------------- pages
@app.route("/")
def index():
    return render_template("index.html")


# --------------------------------------------------------------------- live
@app.route("/api/status")
def api_status():
    return jsonify(status())


@app.route("/api/live/events")
def api_live_events():
    """Server-sent events: batches of data rows every 50 ms, status every 1 s.

    Rows are compact arrays [t, x, y, theta, tier, fps, frame] with t on the
    host monotonic clock -- the same clock a recording's frames.csv uses.
    """
    def gen():
        # prime the charts with the last few seconds so they are not empty
        window = float(settings.get("window_s", 20))
        now = time.monotonic()
        last = 0
        first = [r for r in serial_reader.since(0) if now - r[1] <= window]
        if first:
            last = first[-1][0]
            yield "data: " + json.dumps({"rows": _pack(first)}) + "\n\n"
        t_status = 0.0
        while True:
            rows = serial_reader.since(last)
            payload = {}
            if rows:
                last = rows[-1][0]
                payload["rows"] = _pack(rows)
            if time.monotonic() - t_status >= 1.0:
                payload["status"] = status()
                t_status = time.monotonic()
            if payload:
                yield "data: " + json.dumps(payload) + "\n\n"
            else:
                yield ": keepalive\n\n"
            time.sleep(0.05)

    return Response(gen(), mimetype="text/event-stream",
                    headers={"Cache-Control": "no-cache",
                             "X-Accel-Buffering": "no"})


def _pack(rows):
    return [[round(host_t, 4), r["x"], r["y"], r["theta"], r["tier"],
             r["fps"], r["frame"]] for (_seq, host_t, _wall, r) in rows]


@app.route("/live/stream")
def live_stream():
    """MJPEG for the browser, re-served from the ONE connection the
    dashboard keeps to the camera."""
    boundary = b"unitframe"

    def gen():
        tok = stream_reader.acquire()
        n = 0
        try:
            while True:
                stream_reader.touch(tok)     # the lease is the liveness proof
                n2, jpeg = stream_reader.wait_frame(n, timeout=1.0)
                if n2 == n or jpeg is None:
                    # Heartbeat. A bare CRLF between parts is harmless to the
                    # multipart parser, and writing SOMETHING is the only way
                    # a server notices a browser has gone: with no frames
                    # (camera unreachable) this generator would otherwise
                    # never write, never fail, and leak one viewer per page
                    # load -- keeping the camera reader busy for ever.
                    yield b"\r\n"
                    continue
                n = n2
                yield (b"--" + boundary + b"\r\nContent-Type: image/jpeg\r\n"
                       + ("Content-Length: %d\r\n\r\n" % len(jpeg)).encode()
                       + jpeg + b"\r\n")
        finally:
            stream_reader.release(tok)

    return Response(gen(),
                    mimetype="multipart/x-mixed-replace; boundary=unitframe",
                    headers={"Cache-Control": "no-cache"})


@app.route("/live/shot")
def live_shot():
    if stream_reader.jpeg is None:
        abort(404)
    return Response(stream_reader.jpeg, mimetype="image/jpeg",
                    headers={"Cache-Control": "no-cache"})


# ---------------------------------------------------------------- recording
@app.route("/api/record/start", methods=["POST"])
def api_record_start():
    name = ((request.json or {}).get("name") or "").strip()[:120]
    sid, err = recorder.start(name)
    if err:
        return jsonify({"error": err}), 409
    return jsonify({"id": sid})


@app.route("/api/record/stop", methods=["POST"])
def api_record_stop():
    sid, err = recorder.stop()
    if err:
        return jsonify({"error": err}), 409
    m = read_meta(sid) or {}
    return jsonify({"id": sid, "video": bool(m.get("video"))})


# ------------------------------------------------------- camera stream knobs
# These live on the CAMERA, not here, and they are the fps/quality trade-off:
# every streamed frame is compressed inside the vision loop, so picture costs
# data rate. Measured 2026-09-11 (see README) -- divisor is the big lever,
# resolution barely moves fps any more.
STREAM_KEYS = ("mjpeg_divisor", "mjpeg_quality", "mjpeg_scale")


def _cam_request(path, payload=None, timeout=8):
    import urllib.request
    url = settings["camera_url"] + path
    if payload is None:
        req = urllib.request.Request(url)
    else:
        req = urllib.request.Request(
            url, data=json.dumps(payload).encode(),
            headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode() or "{}")


@app.route("/api/camera/stream", methods=["GET", "POST"])
def api_camera_stream():
    """Read or set the camera's MJPEG knobs, and say whether they persist.

    Without `persist` the camera keeps them until its next reboot, which is
    the right default for trying settings out.
    """
    try:
        if request.method == "POST":
            body = request.json or {}
            patch = {}
            for k in STREAM_KEYS:
                if k in body and body[k] not in (None, ""):
                    patch[k] = (float(body[k]) if k == "mjpeg_scale"
                                else int(body[k]))
            if not patch:
                return jsonify({"error": "nothing to set"}), 400
            _cam_request("/config", {"patch": {"http": patch},
                                     "persist": bool(body.get("persist"))})
        cfg = _cam_request("/config").get("http", {})
        return jsonify({k: cfg.get(k) for k in STREAM_KEYS})
    except Exception as exc:                             # noqa: BLE001
        # the camera not being on the network is the normal case here, not a
        # crash -- the page shows it as a message beside the controls
        return jsonify({"error": str(exc)}), 502


# ----------------------------------------------------------------- settings
@app.route("/api/settings", methods=["GET", "POST"])
def api_settings():
    global settings
    if request.method == "POST":
        body = request.json or {}
        cam = (body.get("camera_url") or settings["camera_url"]).strip()
        if not cam.startswith("http"):
            cam = "http://" + cam
        port = (body.get("serial_port") or "").strip()
        settings["camera_url"] = cam.rstrip("/")
        settings["serial_port"] = port
        try:
            settings["window_s"] = max(5, min(120, int(body.get("window_s", settings["window_s"]))))
        except (TypeError, ValueError):
            pass
        save_settings(settings)
        stream_reader.reconfigure(settings["camera_url"])
        serial_reader.reconfigure(settings["serial_port"])
    return jsonify(settings)


# ----------------------------------------------------------------- sessions
@app.route("/api/sessions")
def api_sessions():
    return jsonify(list_sessions())


@app.route("/api/session/<sid>")
def api_session(sid):
    m = read_meta(sid)
    if not m:
        return jsonify({"error": "not found"}), 404
    m["encoding"] = recorder.encoding.get(m["id"])
    return jsonify(m)


@app.route("/api/session/<sid>/rename", methods=["POST"])
def api_rename(sid):
    m = read_meta(sid)
    if not m:
        return jsonify({"error": "not found"}), 404
    name = ((request.json or {}).get("name") or "").strip()
    if name:
        m["name"] = name[:120]
        write_meta(sid, m)
    return jsonify({"ok": True, "name": m["name"]})


@app.route("/api/session/<sid>/notes", methods=["POST"])
def api_notes(sid):
    m = read_meta(sid)
    if not m:
        return jsonify({"error": "not found"}), 404
    m["notes"] = ((request.json or {}).get("notes") or "")[:4000]
    write_meta(sid, m)
    return jsonify({"ok": True})


@app.route("/api/session/<sid>", methods=["DELETE"])
def api_delete(sid):
    rec = recorder.status()
    if rec and rec["id"] == sid:
        return jsonify({"error": "still recording"}), 409
    d = session_dir(sid)
    if os.path.isdir(d):
        shutil.rmtree(d)
        return jsonify({"ok": True})
    return jsonify({"error": "not found"}), 404


@app.route("/media/<sid>/<path:fn>")
def media(sid, fn):
    d = session_dir(sid)
    if not os.path.isdir(d):
        abort(404)
    # conditional=True -> HTTP range requests, which the player needs to scrub
    return send_from_directory(d, fn, conditional=True)


if __name__ == "__main__":
    # 5050, not 5000: macOS ControlCenter (AirPlay Receiver) listens on
    # *:5000 on every Mac. Binding 127.0.0.1:5000 next to it works, but two
    # listeners on one port is exactly the kind of thing that costs an hour.
    port = int(os.environ.get("PORT", 5050))
    print("\n  UNIT live dashboard -> http://127.0.0.1:%d" % port)
    print("  camera: %s   serial: %s\n"
          % (settings["camera_url"], settings["serial_port"] or "auto (USB 37c5:16e3)"))
    app.run(host="127.0.0.1", port=port, debug=False, threaded=True)
