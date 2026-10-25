"""
tile2.py -- the Tile 2 (UNIT16 hole grid) section of the site.

A Flask blueprint, separate from the Tile 1 code in app.py: own page
(/tile2), own API (/api/t2/...), own runs folder (data/_t2/). Tile 1 files
are not touched.
"""

import json
import os
import shutil
import threading
import time
import uuid

from flask import Blueprint, jsonify, render_template, request, send_from_directory, abort
from werkzeug.utils import secure_filename

import engine
import engine2

HERE = os.path.dirname(os.path.abspath(__file__))
DATA2 = os.path.join(HERE, "data", "_t2")
os.makedirs(DATA2, exist_ok=True)

# Use cases: the list in the repo (cases2.json) is the starting point. Once
# someone adds or deletes a case on the page, the edited list is kept on the
# server disk (data/_t2/cases.json) and used from then on.
CASES_DEFAULT = os.path.join(HERE, "cases2.json")
CASES_DISK = os.path.join(DATA2, "cases.json")
CASES_LOCK = threading.Lock()


def load_cases():
    p = CASES_DISK if os.path.exists(CASES_DISK) else CASES_DEFAULT
    with open(p, encoding="utf-8") as f:
        return json.load(f)


def save_cases(d):
    tmp = CASES_DISK + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(d, f, indent=1, ensure_ascii=False)
    os.replace(tmp, CASES_DISK)


def case_ids():
    return {c["id"] for sec in load_cases()["sections"] for c in sec["cases"]}

ALLOWED = {".mp4", ".mov", ".avi", ".mkv", ".m4v", ".webm"}

bp = Blueprint("tile2", __name__)

JOBS = {}
JOBS_LOCK = threading.Lock()
RUN_LOCK = threading.Lock()      # one analysis at a time (small server)


def sdir(sid):
    d = os.path.join(DATA2, secure_filename(sid))
    if not d.startswith(DATA2) or d == DATA2:
        abort(400)
    return d


def read_meta(sid):
    p = os.path.join(sdir(sid), "meta.json")
    if not os.path.exists(p):
        return None
    with open(p) as f:
        return json.load(f)


def write_meta(sid, meta):
    with open(os.path.join(sdir(sid), "meta.json"), "w") as f:
        json.dump(engine._sanitize(meta), f, indent=2)


def list_runs():
    out = []
    for sid in os.listdir(DATA2):
        if os.path.isdir(os.path.join(DATA2, sid)):
            m = read_meta(sid)
            if m:
                out.append(m)
    out.sort(key=lambda m: m.get("created", 0), reverse=True)
    return out


# ------------------------------------------------------------------ pages
@bp.route("/tile2")
def page():
    return render_template("tile2.html", active="tile2")


# -------------------------------------------------------------------- api
@bp.route("/api/t2/version")
def version():
    r = jsonify({"engine2": engine2.ENGINE2_VERSION,
                 "pipeline": engine2.pipeline_fingerprint()["combined"]})
    r.headers["Cache-Control"] = "no-store"
    return r


@bp.route("/api/t2/sessions")
def sessions():
    return jsonify(list_runs())


@bp.route("/api/t2/cases")
def cases():
    counts = {}
    for m in list_runs():
        c = m.get("case")
        if c:
            counts[c] = counts.get(c, 0) + 1
    out = load_cases()
    for sec in out["sections"]:
        for c in sec["cases"]:
            c["runs"] = counts.get(c["id"], 0)
    return jsonify(out)


def _start(sid, src, scale):
    with JOBS_LOCK:
        JOBS[sid] = {"state": "running", "done": 0, "total": 0, "error": None}

    def work():
        def prog(done, total):
            with JOBS_LOCK:
                JOBS[sid]["done"] = done
                JOBS[sid]["total"] = total
        try:
            with RUN_LOCK:
                summary = engine2.analyze(src, sdir(sid), scale=scale, progress=prog)
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


def _num(i):
    try:
        return int(str(i).split(".")[-1])
    except ValueError:
        return 0


@bp.route("/api/t2/cases/case", methods=["POST"])
def case_add():
    b = request.json or {}
    title = str(b.get("title", "")).strip()[:200]
    if not title:
        return jsonify({"error": "write the use case first"}), 400
    with CASES_LOCK:
        d = load_cases()
        sec = next((x for x in d["sections"] if x["id"] == str(b.get("section"))), None)
        if sec is None:
            return jsonify({"error": "unknown section"}), 400
        # never reuse a number, so runs tagged with a deleted case don't get a new meaning
        n = max([_num(c["id"]) for c in sec["cases"]] + [sec.get("last", 0)]) + 1
        sec["last"] = n
        c = {"id": "%s.%d" % (sec["id"], n), "title": title}
        sec["cases"].append(c)
        save_cases(d)
    return jsonify(c)


@bp.route("/api/t2/cases/case/<cid>", methods=["PUT", "DELETE"])
def case_edit(cid):
    with CASES_LOCK:
        d = load_cases()
        for sec in d["sections"]:
            for i, c in enumerate(sec["cases"]):
                if c["id"] == cid:
                    if request.method == "DELETE":
                        sec["last"] = max(sec.get("last", 0), _num(cid))
                        sec["cases"].pop(i)
                    else:
                        t = str((request.json or {}).get("title", "")).strip()[:200]
                        if not t:
                            return jsonify({"error": "empty title"}), 400
                        c["title"] = t
                    save_cases(d)
                    return jsonify({"ok": True})
    return jsonify({"error": "not found"}), 404


@bp.route("/api/t2/cases/section", methods=["POST"])
def section_add():
    name = str((request.json or {}).get("name", "")).strip()[:120]
    if not name:
        return jsonify({"error": "write the section name first"}), 400
    with CASES_LOCK:
        d = load_cases()
        n = max([_num(s["id"]) for s in d["sections"]] + [d.get("last_section", 0)]) + 1
        d["last_section"] = n
        sec = {"id": str(n), "name": name, "cases": []}
        d["sections"].append(sec)
        save_cases(d)
    return jsonify(sec)


@bp.route("/api/t2/cases/section/<sid>", methods=["DELETE"])
def section_delete(sid):
    with CASES_LOCK:
        d = load_cases()
        sec = next((x for x in d["sections"] if x["id"] == sid), None)
        if sec is None:
            return jsonify({"error": "not found"}), 404
        if sec["cases"]:
            return jsonify({"error": "delete the cases in this section first"}), 400
        d["last_section"] = max(d.get("last_section", 0), _num(sid))
        d["sections"].remove(sec)
        save_cases(d)
    return jsonify({"ok": True})


@bp.route("/api/t2/upload", methods=["POST"])
def upload():
    f = request.files.get("video")
    if not f or not f.filename:
        return jsonify({"error": "no file"}), 400
    ext = os.path.splitext(f.filename)[1].lower()
    if ext not in ALLOWED:
        return jsonify({"error": "unsupported file type %s" % ext}), 400
    sid = time.strftime("%Y%m%d-%H%M%S-") + uuid.uuid4().hex[:6]
    d = sdir(sid)
    os.makedirs(d, exist_ok=True)
    src = os.path.join(d, "source" + ext)
    f.save(src)
    try:
        scale = max(1, min(4, int(request.form.get("scale", 3))))
    except ValueError:
        scale = 3
    case = request.form.get("case", "").strip()
    if case not in case_ids():
        case = ""
    write_meta(sid, {"id": sid, "name": os.path.basename(f.filename), "created": time.time(),
                     "state": "running", "scale": scale, "case": case})
    _start(sid, src, scale)
    return jsonify({"id": sid})


@bp.route("/api/t2/session/<sid>/rerun", methods=["POST"])
def rerun(sid):
    """Same clip again with the CURRENT pipeline (e.g. after u16pipe.py was
    updated). Makes a new run; the old one stays for comparison."""
    m = read_meta(sid)
    if not m:
        return jsonify({"error": "not found"}), 404
    src = None
    for fn in os.listdir(sdir(sid)):
        if fn.startswith("source."):
            src = os.path.join(sdir(sid), fn)
    if not src:
        return jsonify({"error": "source clip missing"}), 404
    nid = time.strftime("%Y%m%d-%H%M%S-") + uuid.uuid4().hex[:6]
    nd = sdir(nid)
    os.makedirs(nd, exist_ok=True)
    nsrc = os.path.join(nd, os.path.basename(src))
    shutil.copyfile(src, nsrc)
    write_meta(nid, {"id": nid, "name": m["name"], "created": time.time(), "state": "running",
                     "scale": m.get("scale", 3), "case": m.get("case", ""),
                     "notes": m.get("notes", ""), "rerun_of": sid})
    _start(nid, nsrc, int(m.get("scale", 3)))
    return jsonify({"id": nid})


@bp.route("/api/t2/progress/<sid>")
def progress(sid):
    with JOBS_LOCK:
        j = dict(JOBS.get(sid, {}))
    if not j:
        m = read_meta(sid)
        if m:
            return jsonify({"state": m.get("state", "done"), "done": 1, "total": 1,
                            "error": m.get("error")})
        return jsonify({"error": "unknown id"}), 404
    return jsonify(j)


@bp.route("/api/t2/session/<sid>")
def session(sid):
    m = read_meta(sid)
    if not m:
        return jsonify({"error": "not found"}), 404
    p = os.path.join(sdir(sid), "series.json")
    if os.path.exists(p):
        with open(p) as f:
            m["series"] = json.load(f)
    return jsonify(m)


@bp.route("/api/t2/session/<sid>/rename", methods=["POST"])
def rename(sid):
    m = read_meta(sid)
    if not m:
        return jsonify({"error": "not found"}), 404
    name = (request.json or {}).get("name", "").strip()
    if name:
        m["name"] = name[:120]
        write_meta(sid, m)
    return jsonify({"ok": True, "name": m["name"]})


@bp.route("/api/t2/session/<sid>/case", methods=["POST"])
def set_case(sid):
    m = read_meta(sid)
    if not m:
        return jsonify({"error": "not found"}), 404
    case = ((request.json or {}).get("case") or "").strip()
    if case and case not in case_ids():
        return jsonify({"error": "unknown case id"}), 400
    m["case"] = case
    write_meta(sid, m)
    return jsonify({"ok": True, "case": case})


@bp.route("/api/t2/session/<sid>/notes", methods=["POST"])
def notes(sid):
    m = read_meta(sid)
    if not m:
        return jsonify({"error": "not found"}), 404
    m["notes"] = (request.json or {}).get("notes", "")[:4000]
    write_meta(sid, m)
    return jsonify({"ok": True})


@bp.route("/api/t2/session/<sid>", methods=["DELETE"])
def delete(sid):
    d = sdir(sid)
    if os.path.isdir(d):
        shutil.rmtree(d)
        return jsonify({"ok": True})
    return jsonify({"error": "not found"}), 404


@bp.route("/media2/<sid>/<path:fn>")
def media(sid, fn):
    d = sdir(sid)
    if not os.path.isdir(d):
        abort(404)
    return send_from_directory(d, fn, conditional=True)
