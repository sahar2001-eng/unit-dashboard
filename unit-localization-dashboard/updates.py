"""
updates.py -- the shared Updates diary: where we are, what we did, what
changed, what is next. Anyone with the link can read and write.

Entries are kept in data/_updates/updates.json (on the server disk, so they
survive restarts and deploys).
"""

import json
import os
import threading
import time
import uuid

from flask import Blueprint, jsonify, render_template, request

HERE = os.path.dirname(os.path.abspath(__file__))
UPD_DIR = os.path.join(HERE, "data", "_updates")
UPD_FILE = os.path.join(UPD_DIR, "updates.json")
os.makedirs(UPD_DIR, exist_ok=True)

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
    return jsonify({"ok": True})
