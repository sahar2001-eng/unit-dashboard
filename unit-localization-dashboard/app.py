#!/usr/bin/env python3
"""
UNIT -- Grid localization analytics
Local dashboard that runs the robot's OWN camera code on recorded clips.

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

CASES_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cases.json")
with open(CASES_PATH, encoding="utf-8") as _f:
    CASES = json.load(_f)
CASE_IDS = {c["id"] for sec in CASES["sections"] for c in sec["cases"]}

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
    resp = jsonify({"engine_version": engine.ENGINE_VERSION,
                    "flight": engine.flight_fingerprint()})
    resp.headers["Cache-Control"] = "no-store"
    return resp


@app.route("/api/sessions")
def api_sessions():
    return jsonify(list_sessions())


@app.route("/api/cases")
def api_cases():
    """The use-case list, plus how many runs are tagged with each case."""
    counts = {}
    for m in list_sessions():
        c = m.get("case")
        if c:
            counts[c] = counts.get(c, 0) + 1
    out = json.loads(json.dumps(CASES))
    for sec in out["sections"]:
        for c in sec["cases"]:
            c["runs"] = counts.get(c["id"], 0)
    return jsonify(out)


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

    scale = int(request.form.get("scale", 3))
    case = request.form.get("case", "").strip()
    if case and case not in CASE_IDS:
        case = ""

    # the turn command(s) a host would have sent, in video frame numbers
    turn = None
    ts = request.form.get("turn_start", "").strip()
    if ts:
        try:
            turn = {"start_frame": int(ts),
                    "dir": "R" if request.form.get("turn_dir", "L") == "R" else "L",
                    "end_frame": None}
            te = request.form.get("turn_end", "").strip()
            if te:
                turn["end_frame"] = int(te)
        except ValueError:
            return jsonify({"error": "turn frames must be whole numbers"}), 400

    meta = {"id": sid, "name": os.path.basename(f.filename),
            "created": time.time(), "state": "running",
            "scale": scale, "case": case, "turn": turn}
    write_meta(sid, meta)

    with JOBS_LOCK:
        JOBS[sid] = {"state": "running", "done": 0, "total": 0, "error": None}

    def work():
        def prog(done, total):
            with JOBS_LOCK:
                JOBS[sid]["done"] = done
                JOBS[sid]["total"] = total
        try:
            summary = engine.analyze(src, d, scale=scale, turn=turn,
                                     progress=prog)
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


@app.route("/api/session/<sid>/rerun", methods=["POST"])
def api_rerun(sid):
    """Analyse the same clip again with different turn command(s), without
    re-uploading. Makes a new run; the old one is untouched."""
    m = read_meta(sid)
    if not m:
        return jsonify({"error": "not found"}), 404
    src_dir = session_dir(sid)
    src = None
    for fn in os.listdir(src_dir):
        if fn.startswith("source."):
            src = os.path.join(src_dir, fn)
    if not src:
        return jsonify({"error": "source clip missing"}), 404
    body = request.json or {}
    turn = None
    if body.get("turn_start") not in (None, ""):
        try:
            turn = {"start_frame": int(body["turn_start"]),
                    "dir": "R" if body.get("turn_dir") == "R" else "L",
                    "end_frame": (int(body["turn_end"])
                                  if body.get("turn_end") not in (None, "") else None)}
        except (TypeError, ValueError):
            return jsonify({"error": "turn frames must be whole numbers"}), 400

    nid = time.strftime("%Y%m%d-%H%M%S-") + uuid.uuid4().hex[:6]
    nd = session_dir(nid)
    os.makedirs(nd, exist_ok=True)
    nsrc = os.path.join(nd, os.path.basename(src))
    shutil.copyfile(src, nsrc)
    scale = int(m.get("scale", 3))
    meta = {"id": nid, "name": m["name"], "created": time.time(),
            "state": "running", "scale": scale, "case": m.get("case", ""),
            "turn": turn, "rerun_of": sid, "notes": m.get("notes", "")}
    write_meta(nid, meta)
    with JOBS_LOCK:
        JOBS[nid] = {"state": "running", "done": 0, "total": 0, "error": None}

    def work():
        def prog(done, total):
            with JOBS_LOCK:
                JOBS[nid]["done"] = done
                JOBS[nid]["total"] = total
        try:
            summary = engine.analyze(nsrc, nd, scale=scale, turn=turn,
                                     progress=prog)
            mm = read_meta(nid)
            mm["state"] = "done"
            mm["summary"] = summary
            write_meta(nid, mm)
            with JOBS_LOCK:
                JOBS[nid]["state"] = "done"
        except Exception as exc:                    # noqa: BLE001
            mm = read_meta(nid) or {"id": nid}
            mm["state"] = "error"
            mm["error"] = str(exc)
            write_meta(nid, mm)
            with JOBS_LOCK:
                JOBS[nid]["state"] = "error"
                JOBS[nid]["error"] = str(exc)

    threading.Thread(target=work, daemon=True).start()
    return jsonify({"id": nid})


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


@app.route("/api/session/<sid>/case", methods=["POST"])
def api_case(sid):
    m = read_meta(sid)
    if not m:
        return jsonify({"error": "not found"}), 404
    case = ((request.json or {}).get("case") or "").strip()
    if case and case not in CASE_IDS:
        return jsonify({"error": "unknown case id"}), 400
    m["case"] = case
    write_meta(sid, m)
    return jsonify({"ok": True, "case": case})


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
    print("\n  UNIT grid localization analytics -> http://127.0.0.1:%d" % port)
    print("  engine %s   flight code %s\n"
          % (engine.ENGINE_VERSION, engine.flight_fingerprint()["combined"]))
    app.run(host="127.0.0.1", port=port, debug=False, threaded=True)
