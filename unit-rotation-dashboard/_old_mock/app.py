#!/usr/bin/env python3
"""
UNIT -- Grid Rotation Analytics
Local web dashboard for the robot's grid-localization rotation estimator.

    pip install -r requirements.txt
    python app.py

then open http://127.0.0.1:5000
"""

import json
import os
import shutil
import threading
import time
import uuid

from flask import (Flask, jsonify, render_template, request,
                   send_from_directory, abort)
from werkzeug.utils import secure_filename

import engine

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "data")
os.makedirs(DATA, exist_ok=True)

ALLOWED = {".mp4", ".mov", ".avi", ".mkv", ".m4v", ".webm"}
MAX_MB = 2048

app = Flask(__name__)
app.config["MAX_CONTENT_LENGTH"] = MAX_MB * 1024 * 1024

JOBS = {}          # id -> {state, done, total, error}
JOBS_LOCK = threading.Lock()


# ------------------------------------------------------------------ helpers
def session_dir(sid):
    d = os.path.join(DATA, secure_filename(sid))
    if not d.startswith(DATA):
        abort(400)
    return d


def read_meta(sid):
    p = os.path.join(session_dir(sid), "meta.json")
    if not os.path.exists(p):
        return None
    with open(p) as f:
        return json.load(f)


def write_meta(sid, meta):
    with open(os.path.join(session_dir(sid), "meta.json"), "w") as f:
        json.dump(engine._sanitize(meta), f, indent=2)


def list_sessions():
    out = []
    for sid in os.listdir(DATA):
        m = read_meta(sid)
        if m:
            out.append(m)
    out.sort(key=lambda m: m.get("created", 0), reverse=True)
    return out


# -------------------------------------------------------------------- pages
@app.route("/")
def index():
    return render_template("index.html")


# --------------------------------------------------------------------- api
@app.route("/api/version")
def api_version():
    return jsonify({"engine_version": engine.ENGINE_VERSION})


@app.route("/api/sessions")
def api_sessions():
    return jsonify(list_sessions())


@app.route("/api/upload", methods=["POST"])
def api_upload():
    f = request.files.get("video")
    if not f or not f.filename:
        return jsonify({"error": "no file"}), 400
    ext = os.path.splitext(f.filename)[1].lower()
    if ext not in ALLOWED:
        return jsonify({"error": "unsupported file type %s" % ext}), 400

    sid = time.strftime("%Y%m%d-%H%M%S-") + uuid.uuid4().hex[:6]
    d = session_dir(sid)
    os.makedirs(d, exist_ok=True)
    src = os.path.join(d, "source" + ext)
    f.save(src)

    step = int(request.form.get("step", 2))
    scale = int(request.form.get("scale", 3))
    smooth = request.form.get("smooth", "1") == "1"

    meta = {"id": sid, "name": os.path.basename(f.filename),
            "created": time.time(), "state": "running",
            "step": step, "scale": scale, "smooth": smooth}
    write_meta(sid, meta)

    with JOBS_LOCK:
        JOBS[sid] = {"state": "running", "done": 0, "total": 0, "error": None}

    def work():
        def prog(done, total):
            with JOBS_LOCK:
                JOBS[sid]["done"] = done
                JOBS[sid]["total"] = total
        try:
            summary = engine.analyze(src, d, step=step, scale=scale,
                                     smooth=smooth, progress=prog)
            m = read_meta(sid)
            m["state"] = "done"
            m["summary"] = summary
            write_meta(sid, m)
            with JOBS_LOCK:
                JOBS[sid]["state"] = "done"
        except Exception as exc:                    # noqa: BLE001
            m = read_meta(sid) or {"id": sid}
            m["state"] = "error"
            m["error"] = str(exc)
            write_meta(sid, m)
            with JOBS_LOCK:
                JOBS[sid]["state"] = "error"
                JOBS[sid]["error"] = str(exc)

    threading.Thread(target=work, daemon=True).start()
    return jsonify({"id": sid})


@app.route("/api/progress/<sid>")
def api_progress(sid):
    with JOBS_LOCK:
        j = dict(JOBS.get(sid, {}))
    if not j:
        m = read_meta(sid)
        if m:
            return jsonify({"state": m.get("state", "done"),
                            "done": 1, "total": 1,
                            "error": m.get("error")})
        return jsonify({"error": "unknown id"}), 404
    return jsonify(j)


@app.route("/api/session/<sid>")
def api_session(sid):
    m = read_meta(sid)
    if not m:
        return jsonify({"error": "not found"}), 404
    p = os.path.join(session_dir(sid), "series.json")
    if os.path.exists(p):
        with open(p) as f:
            m["series"] = json.load(f)
    return jsonify(m)


@app.route("/api/session/<sid>/rename", methods=["POST"])
def api_rename(sid):
    m = read_meta(sid)
    if not m:
        return jsonify({"error": "not found"}), 404
    name = (request.json or {}).get("name", "").strip()
    if name:
        m["name"] = name[:120]
        write_meta(sid, m)
    return jsonify({"ok": True, "name": m["name"]})


@app.route("/api/session/<sid>/notes", methods=["POST"])
def api_notes(sid):
    m = read_meta(sid)
    if not m:
        return jsonify({"error": "not found"}), 404
    m["notes"] = (request.json or {}).get("notes", "")[:4000]
    write_meta(sid, m)
    return jsonify({"ok": True})


@app.route("/api/session/<sid>", methods=["DELETE"])
def api_delete(sid):
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
    # conditional=True gives HTTP range requests, which the player needs
    # for scrubbing
    return send_from_directory(d, fn, conditional=True)


if __name__ == "__main__":
    port = int(os.environ.get("PORT", 5000))
    print("\n  UNIT rotation analytics -> http://127.0.0.1:%d" % port)
    print("  engine version: %s\n" % engine.ENGINE_VERSION)
    app.run(host="127.0.0.1", port=port, debug=False, threaded=True)
