"""
runs.py -- the Runs section of the site (/runs).

The site does NOT analyse anything here. A run is made on a PC (runkit/run.py:
the camera's own code over a video, or a recorded robot drive) and uploaded
as one zip with three files:

    info.json    what the run is (tile, source, app version, fps, ...)
    frames.csv   one row per picture in video.mp4, in the same order
    video.mp4    the pictures, with the camera's overlay drawn on them

This file only stores runs and hands them to the page. Own folder
(data/_runs/), own page and API; Tile 1, Tile 2 and Updates are not touched.

Upload and delete need the password in the RUNS_TOKEN environment variable
(set it on Render). Without it, uploading is off.
"""

import csv
import hmac
import json
import os
import shutil
import time
import uuid
import zipfile

from flask import Blueprint, jsonify, render_template, request, send_from_directory, abort
from werkzeug.utils import secure_filename

HERE = os.path.dirname(os.path.abspath(__file__))
DATA3 = os.path.join(HERE, "data", "_runs")
os.makedirs(DATA3, exist_ok=True)

FORMAT = 1                       # the run folder format this page reads
NEEDED = ("info.json", "frames.csv", "video.mp4")
OPTIONAL = ("log.txt",)
MAX_UNZIPPED = 1500 * 1024 * 1024
# frames.csv columns the page knows; any other column is kept and ignored
NUM_COLS = ("t_s", "x_mm", "y_mm", "theta_deg", "q", "fps", "heading", "ts_ms")
INT_COLS = ("frame", "tile", "suspect")

bp = Blueprint("runs", __name__)


# ---------------------------------------------------------------- helpers
def rdir(rid):
    d = os.path.join(DATA3, secure_filename(rid))
    if not d.startswith(DATA3) or d == DATA3:
        abort(400)
    return d


def read_meta(rid):
    p = os.path.join(rdir(rid), "meta.json")
    if not os.path.exists(p):
        return None
    with open(p, encoding="utf-8") as f:
        return json.load(f)


def write_meta(rid, meta):
    p = os.path.join(rdir(rid), "meta.json")
    with open(p + ".tmp", "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=1)
    os.replace(p + ".tmp", p)


def list_runs():
    out = []
    for rid in os.listdir(DATA3):
        if os.path.isdir(os.path.join(DATA3, rid)):
            m = read_meta(rid)
            if m:
                out.append(m)
    out.sort(key=lambda m: m.get("created", 0), reverse=True)
    return out


def token_ok():
    want = os.environ.get("RUNS_TOKEN", "")
    got = request.headers.get("X-Runs-Token", "") or request.form.get("token", "")
    return bool(want) and hmac.compare_digest(want.encode(), got.encode())


def no_token():
    if not os.environ.get("RUNS_TOKEN"):
        return jsonify({"error": "uploading is off: RUNS_TOKEN is not set on the server"}), 403
    return jsonify({"error": "wrong password"}), 403


def cases_for(tile):
    """The use-case list of that tile: the same lists the Tile 1 and Tile 2
    pages use (Tile 2's may have been edited on the page)."""
    if int(tile or 0) == 2:
        import tile2
        return tile2.load_cases()
    with open(os.path.join(HERE, "cases.json"), encoding="utf-8") as f:
        return json.load(f)


def case_ids(tile):
    return {c["id"] for sec in cases_for(tile)["sections"] for c in sec["cases"]}


def _num(v):
    if v in ("", "-", None):
        return None
    try:
        return float(v)
    except ValueError:
        return None


def read_columns(path):
    """frames.csv as columns: {name: [values]}. Numbers as numbers, empty or
    '-' as null. Text columns (tier, code, why, mode) stay text."""
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    cols = {}
    if not rows:
        return cols
    for name in rows[0].keys():
        if name in NUM_COLS:
            cols[name] = [_num(r[name]) for r in rows]
        elif name in INT_COLS:
            cols[name] = [None if _num(r[name]) is None else int(_num(r[name])) for r in rows]
        else:
            cols[name] = [r[name] if r[name] not in ("", "-") else None for r in rows]
    return cols


def check_zip(zf):
    """(names to extract {zip name: file name}, error or '')."""
    found = {}
    total = 0
    for zi in zf.infolist():
        if zi.is_dir():
            continue
        base = os.path.basename(zi.filename)
        if base in NEEDED + OPTIONAL:
            if base in found:
                return None, "two files called %s in the zip" % base
            found[base] = zi.filename
            total += zi.file_size
    miss = [n for n in NEEDED if n not in found]
    if miss:
        return None, "the zip is missing: " + ", ".join(miss)
    if total > MAX_UNZIPPED:
        return None, "the run is too big (over %d MB)" % (MAX_UNZIPPED // 2**20)
    return found, ""


# ------------------------------------------------------------------ pages
@bp.route("/runs")
def page():
    return render_template("runs.html", active="runs")


# -------------------------------------------------------------------- api
@bp.route("/api/runs")
def runs():
    return jsonify({"runs": list_runs(),
                    "upload_on": bool(os.environ.get("RUNS_TOKEN"))})


@bp.route("/api/runs/cases/<int:tile>")
def cases(tile):
    d = json.loads(json.dumps(cases_for(tile)))
    counts = {}
    for m in list_runs():
        if int(m.get("tile") or 0) == tile and m.get("case"):
            counts[m["case"]] = counts.get(m["case"], 0) + 1
    for sec in d["sections"]:
        for c in sec["cases"]:
            c["runs"] = counts.get(c["id"], 0)
    return jsonify(d)


@bp.route("/api/runs/upload", methods=["POST"])
def upload():
    if not token_ok():
        return no_token()
    f = request.files.get("run")
    if not f or not f.filename:
        return jsonify({"error": "no file"}), 400
    rid = time.strftime("%Y%m%d-%H%M%S-") + uuid.uuid4().hex[:6]
    d = rdir(rid)
    os.makedirs(d)
    tmp = os.path.join(d, "_upload.zip")       # on disk, not in memory
    try:
        f.save(tmp)
        try:
            zf = zipfile.ZipFile(tmp)
        except zipfile.BadZipFile:
            raise ValueError("not a zip file (make it with: run.py upload <folder>)")
        with zf:
            names, err = check_zip(zf)
            if err:
                raise ValueError(err)
            try:
                info = json.loads(zf.read(names["info.json"]).decode("utf-8"))
            except ValueError:
                raise ValueError("info.json is not valid JSON")
            if not isinstance(info, dict):
                raise ValueError("info.json is not an object")
            if int(info.get("format", 0) or 0) > FORMAT:
                raise ValueError("this run is format %s; the site reads up to %d. "
                                 "Update the site." % (info.get("format"), FORMAT))
            for base, zname in names.items():
                with zf.open(zname) as src, open(os.path.join(d, base), "wb") as dst:
                    shutil.copyfileobj(src, dst, 1 << 20)
        os.remove(tmp)
        cols = read_columns(os.path.join(d, "frames.csv"))
        nrows = len(cols.get("frame", []))
        if not nrows:
            raise ValueError("frames.csv has no rows (or no 'frame' column)")
    except Exception as e:                       # noqa: BLE001
        shutil.rmtree(d, ignore_errors=True)
        return jsonify({"error": "bad run: %s" % e}), 400

    try:
        tile = int(info.get("tile") or 0)
    except (TypeError, ValueError):
        tile = 0
    case = (request.form.get("case") or info.get("case") or "").strip()
    if case and case not in case_ids(tile):
        case = ""
    meta = {
        "id": rid,
        "name": (request.form.get("name") or info.get("name") or f.filename)[:120],
        "created": time.time(),
        "recorded": info.get("created"),
        "tile": tile,
        "source": info.get("source", ""),
        "case": case,
        "notes": (info.get("notes") or "")[:4000],
        "rows": nrows,
        "info": info,
        "has_log": "log.txt" in names,
    }
    write_meta(rid, meta)
    return jsonify({"id": rid, "rows": nrows})


@bp.route("/api/runs/<rid>")
def run(rid):
    m = read_meta(rid)
    if not m:
        return jsonify({"error": "not found"}), 404
    m["cols"] = read_columns(os.path.join(rdir(rid), "frames.csv"))
    return jsonify(m)


@bp.route("/api/runs/<rid>/rename", methods=["POST"])
def rename(rid):
    m = read_meta(rid)
    if not m:
        return jsonify({"error": "not found"}), 404
    name = ((request.json or {}).get("name") or "").strip()
    if name:
        m["name"] = name[:120]
        write_meta(rid, m)
    return jsonify({"ok": True, "name": m["name"]})


@bp.route("/api/runs/<rid>/case", methods=["POST"])
def set_case(rid):
    m = read_meta(rid)
    if not m:
        return jsonify({"error": "not found"}), 404
    case = ((request.json or {}).get("case") or "").strip()
    if case and case not in case_ids(m.get("tile")):
        return jsonify({"error": "unknown use case for this tile"}), 400
    m["case"] = case
    write_meta(rid, m)
    return jsonify({"ok": True, "case": case})


@bp.route("/api/runs/<rid>/notes", methods=["POST"])
def notes(rid):
    m = read_meta(rid)
    if not m:
        return jsonify({"error": "not found"}), 404
    m["notes"] = ((request.json or {}).get("notes") or "")[:4000]
    write_meta(rid, m)
    return jsonify({"ok": True})


@bp.route("/api/runs/<rid>", methods=["DELETE"])
def delete(rid):
    if not token_ok():
        return no_token()
    d = rdir(rid)
    if os.path.isdir(d):
        shutil.rmtree(d)
        return jsonify({"ok": True})
    return jsonify({"error": "not found"}), 404


@bp.route("/media3/<rid>/<path:fn>")
def media(rid, fn):
    d = rdir(rid)
    if not os.path.isdir(d) or fn not in NEEDED + OPTIONAL:
        abort(404)
    return send_from_directory(d, fn, conditional=True)
