"""
updates.py -- the shared Updates diary: where we are, what we did, what
changed, what is next. Anyone with the link can read and write.

Entries are kept in data/_updates/updates.json (on the server disk, so they
survive restarts and deploys). Files attached to an entry (Word docs, PDFs,
pictures, videos...) are kept in data/_updates/files/<entry id>/.
"""

import json
import os
import shutil
import threading
import time
import uuid

from flask import Blueprint, abort, jsonify, render_template, request, send_from_directory
from werkzeug.utils import secure_filename

HERE = os.path.dirname(os.path.abspath(__file__))
UPD_DIR = os.path.join(HERE, "data", "_updates")
UPD_FILE = os.path.join(UPD_DIR, "updates.json")
FILES_DIR = os.path.join(UPD_DIR, "files")
os.makedirs(FILES_DIR, exist_ok=True)

MAX_FILE_MB = 200
# shown in the browser; everything else is downloaded (safer, and Word/Excel
# files open in their own app anyway)
INLINE = {".png", ".jpg", ".jpeg", ".gif", ".webp", ".pdf", ".mp4", ".mov", ".webm", ".txt"}

AREAS = ["Tile 1", "Tile 2", "Dashboard", "General"]
FIELDS = {"title": 160, "author": 60, "where": 3000, "did": 6000, "changed": 6000,
          "next": 3000, "date": 10, "area": 20}

bp = Blueprint("updates", __name__)
LOCK = threading.Lock()

SEED = [
    {
        "date": "2026-10-25", "author": "Sahar", "area": "Tile 2",
        "title": "Robustness: X only from code junctions, lens correction",
        "where": "Tile 2 (UNIT16) localization runs live on the AE3. All stages work "
                 "(grid angle, strips, line fit, code read, X/Y/angle, turns). Waiting for "
                 "the real plate measurements; robot integration later.",
        "did": "Added guards against false positives: line angle check, road/cross "
               "perpendicular check, position-jump check (HOLD), code accepted only when "
               "read 2 frames in a row.\n"
               "X now comes only from code junctions; the plain wide bar half way between "
               "them is dropped (presence check of the code block).\n"
               "Lens correction of the line points before the fit (K1 = 0.15, estimated).",
        "changed": "X flicker on the old recording: 42 on/off switches -> 6.\n"
                   "Each X is shown with the junction it is measured from (@13E, or @? until "
                   "its code is read).\n"
                   "No code mix-ups on the test videos.",
        "next": "Live test of the flicker case. Code spot offsets (+-12.4 / +-34.8 px). "
                "Faster straight driving (axis scan below 10 deg). Real mm scale and lens "
                "value once the measurements arrive.",
    },
    {
        "date": "2026-10-25", "author": "Sahar", "area": "Dashboard",
        "title": "Site split into Tile 1, Tile 2 and Updates",
        "where": "Tile 1 analytics unchanged.",
        "did": "Added a Tile 2 page that runs the AE3's own UNIT16 code (u16pipe.py + u16c) "
               "on uploaded clips with a real MicroPython, and this Updates diary.",
        "changed": "Top menu with three sections.",
        "next": "Upload Tile 2 clips recorded with DRAW = 0 (no overlay in the image).",
    },
]


def _load():
    if not os.path.exists(UPD_FILE):
        now = time.time()
        items = []
        for i, e in enumerate(SEED):
            e = dict(e)
            e["id"] = uuid.uuid4().hex[:10]
            e["created"] = e["updated"] = now - i
            items.append(e)
        _save(items)
        return items
    with open(UPD_FILE, encoding="utf-8") as f:
        return json.load(f)


def _save(items):
    tmp = UPD_FILE + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(items, f, indent=1, ensure_ascii=False)
    os.replace(tmp, UPD_FILE)


def _clean(body, old=None):
    e = dict(old or {})
    for k, n in FIELDS.items():
        if k in body:
            e[k] = str(body.get(k) or "").strip()[:n]
    if e.get("area") not in AREAS:
        e["area"] = "General"
    if not e.get("date"):
        e["date"] = time.strftime("%Y-%m-%d")
    return e


def _sorted(items):
    return sorted(items, key=lambda e: (e.get("date", ""), e.get("created", 0)), reverse=True)


@bp.route("/updates")
def page():
    return render_template("updates.html", active="updates")


@bp.route("/api/updates")
def list_updates():
    with LOCK:
        items = _load()
    return jsonify({"areas": AREAS, "items": _sorted(items)})


@bp.route("/api/updates", methods=["POST"])
def add_update():
    body = request.json or {}
    if not str(body.get("title", "")).strip():
        return jsonify({"error": "a title is needed"}), 400
    e = _clean(body)
    e["id"] = uuid.uuid4().hex[:10]
    e["created"] = e["updated"] = time.time()
    with LOCK:
        items = _load()
        items.append(e)
        _save(items)
    return jsonify(e)


@bp.route("/api/updates/<uid>", methods=["PUT"])
def edit_update(uid):
    body = request.json or {}
    with LOCK:
        items = _load()
        for i, e in enumerate(items):
            if e.get("id") == uid:
                n = _clean(body, e)
                if not n.get("title"):
                    return jsonify({"error": "a title is needed"}), 400
                n["updated"] = time.time()
                items[i] = n
                _save(items)
                return jsonify(n)
    return jsonify({"error": "not found"}), 404


@bp.route("/api/updates/<uid>", methods=["DELETE"])
def delete_update(uid):
    with LOCK:
        items = _load()
        keep = [e for e in items if e.get("id") != uid]
        if len(keep) == len(items):
            return jsonify({"error": "not found"}), 404
        _save(keep)
    d = _files_dir(uid)
    if os.path.isdir(d):
        shutil.rmtree(d)
    return jsonify({"ok": True})


# ------------------------------------------------------------ attached files
def _files_dir(uid):
    d = os.path.join(FILES_DIR, secure_filename(uid))
    if not d.startswith(FILES_DIR + os.sep):
        abort(400)
    return d


def _safe_name(name, taken):
    # keeps the extension even for names secure_filename would empty (e.g. Hebrew);
    # the original name is still shown and used for the download
    stem0, ext0 = os.path.splitext(os.path.basename(name))
    e = secure_filename(ext0.lstrip(".")).lower()[:10]
    ext = "." + e if e else ""
    stem = (secure_filename(stem0) or "file")[:80]
    base = stem + ext
    n, i = base, 2
    while n in taken:
        n = "%s_%d%s" % (stem, i, ext)
        i += 1
    return n


@bp.route("/api/updates/<uid>/files", methods=["POST"])
def add_files(uid):
    files = [f for f in request.files.getlist("files") if f and f.filename]
    if not files:
        return jsonify({"error": "no file"}), 400
    with LOCK:
        items = _load()
        e = next((x for x in items if x.get("id") == uid), None)
        if e is None:
            return jsonify({"error": "not found"}), 404
        d = _files_dir(uid)
        os.makedirs(d, exist_ok=True)
        lst = e.setdefault("files", [])
        taken = {f["file"] for f in lst}
        added, too_big = [], []
        for f in files:
            fn = _safe_name(f.filename, taken)
            p = os.path.join(d, fn)
            f.save(p)
            size = os.path.getsize(p)
            if size > MAX_FILE_MB * 1024 * 1024:
                os.remove(p)
                too_big.append(f.filename)
                continue
            taken.add(fn)
            rec = {"file": fn, "name": os.path.basename(f.filename)[:160], "size": size,
                   "added": time.time()}
            lst.append(rec)
            added.append(rec)
        e["updated"] = time.time()
        _save(items)
    out = {"ok": True, "added": added, "files": lst}
    if too_big:
        out["error"] = "too big (max %d MB): %s" % (MAX_FILE_MB, ", ".join(too_big))
    return jsonify(out)


@bp.route("/api/updates/<uid>/files/<fn>", methods=["DELETE"])
def delete_file(uid, fn):
    with LOCK:
        items = _load()
        e = next((x for x in items if x.get("id") == uid), None)
        if e is None:
            return jsonify({"error": "not found"}), 404
        lst = e.get("files", [])
        keep = [f for f in lst if f["file"] != fn]
        if len(keep) == len(lst):
            return jsonify({"error": "file not found"}), 404
        p = os.path.join(_files_dir(uid), secure_filename(fn))
        if os.path.exists(p):
            os.remove(p)
        e["files"] = keep
        e["updated"] = time.time()
        _save(items)
    return jsonify({"ok": True, "files": keep})


@bp.route("/updfiles/<uid>/<fn>")
def get_file(uid, fn):
    d = _files_dir(uid)
    fn = secure_filename(fn)
    if not os.path.exists(os.path.join(d, fn)):
        abort(404)
    with LOCK:
        e = next((x for x in _load() if x.get("id") == uid), None)
    name = fn
    if e:
        name = next((f["name"] for f in e.get("files", []) if f["file"] == fn), fn)
    inline = os.path.splitext(fn)[1].lower() in INLINE and request.args.get("dl") != "1"
    r = send_from_directory(d, fn, conditional=True, as_attachment=not inline,
                            download_name=name)
    r.headers["X-Content-Type-Options"] = "nosniff"
    return r
