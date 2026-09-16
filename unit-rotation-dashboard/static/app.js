// UNIT live grid camera dashboard.
//
// Data arrives as server-sent events (batches of [t, x, y, theta, tier, fps,
// frame] rows); video is an <img> on the MJPEG proxy. Three rolling charts,
// a readout, a record button, and playback of saved recordings with the
// readout following the playhead.

const $ = (s) => document.querySelector(s);
const api = (u, o) => fetch(u, o).then((r) => r.json());
const fmt = (v, d = 1) => (v == null || Number.isNaN(v)) ? "—" : (v >= 0 ? "+" : "") + v.toFixed(d);
const TIER = { junction: "Junction — X, Y, θ", y: "Bar only — Y, θ", x: "X only",
               none: "No fix (turning, or no grid)" };

let WINDOW_S = 20;
let SETTINGS = {};
let SESSION = null;
let RECORDING = null;

// ------------------------------------------------------------- rolling chart
class Chart {
  constructor(canvas, key, opts = {}) {
    this.c = canvas; this.ctx = canvas.getContext("2d");
    this.key = key; this.minSpan = opts.minSpan || 4;
    this.color = opts.color || "#ff7a1a";
    this.buf = []; this.t0 = null; this.static = false;
  }
  push(row) { this.buf.push(row); }
  prune(now) {
    if (this.static) return;
    const cut = now - WINDOW_S;
    let i = 0; while (i < this.buf.length && this.buf[i].t < cut) i++;
    if (i) this.buf.splice(0, i);
  }
  setStatic(rows) { this.buf = rows; this.static = true; }
  draw(now, cursorT) {
    const c = this.c, dpr = window.devicePixelRatio || 1;
    const W = c.clientWidth, H = c.clientHeight;
    if (!W || !H) return;
    if (c.width !== W * dpr || c.height !== H * dpr) { c.width = W * dpr; c.height = H * dpr; }
    const g = this.ctx; g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, W, H);
    const padL = 44, padR = 12, padT = 8, padB = 18;
    const w = W - padL - padR, h = H - padT - padB;

    let tmin, tmax;
    if (this.static) { tmin = this.buf.length ? this.buf[0].t : 0; tmax = this.buf.length ? this.buf[this.buf.length - 1].t : 1; }
    else { tmax = now; tmin = now - WINDOW_S; }
    if (tmax - tmin < 0.5) tmax = tmin + 0.5;

    // y range from the visible data, with a floor on the span so noise
    // does not fill the whole chart
    let lo = Infinity, hi = -Infinity;
    for (const r of this.buf) { const v = r[this.key]; if (v != null && r.t >= tmin) { if (v < lo) lo = v; if (v > hi) hi = v; } }
    if (!isFinite(lo)) { lo = -this.minSpan / 2; hi = this.minSpan / 2; }
    let mid = (lo + hi) / 2, span = Math.max(hi - lo, this.minSpan) * 1.15;
    lo = mid - span / 2; hi = mid + span / 2;

    const X = (t) => padL + (t - tmin) / (tmax - tmin) * w;
    const Y = (v) => padT + (1 - (v - lo) / (hi - lo)) * h;

    // grid + ticks
    g.font = "11px Inter, system-ui"; g.fillStyle = "#8a93a3"; g.strokeStyle = "#2a3140"; g.lineWidth = 1;
    for (const tv of niceTicks(lo, hi, 4)) {
      const y = Y(tv); g.beginPath(); g.moveTo(padL, y); g.lineTo(W - padR, y); g.stroke();
      g.textAlign = "right"; g.fillText(tv.toFixed(Math.abs(hi - lo) < 5 ? 1 : 0), padL - 6, y + 4);
    }
    if (lo < 0 && hi > 0) { g.strokeStyle = "#3d475a"; g.beginPath(); g.moveTo(padL, Y(0)); g.lineTo(W - padR, Y(0)); g.stroke(); }
    g.textAlign = "center";
    const tStep = this.static ? niceStep((tmax - tmin) / 6) : 5;
    for (let t = Math.ceil(tmin / tStep) * tStep; t <= tmax; t += tStep) {
      const dp = tStep < 1 ? 1 : 0;   // sub-second ticks need a decimal or they all read "0s"
      const label = this.static ? t.toFixed(dp) + "s" : (t - tmax).toFixed(0) + "s";
      g.fillText(label, X(t), H - 4);
    }

    // trace with gaps where the value is null
    g.strokeStyle = this.color; g.lineWidth = 1.6; g.beginPath(); let pen = false;
    for (const r of this.buf) {
      const v = r[this.key];
      if (v == null || r.t < tmin) { pen = false; continue; }
      const x = X(r.t), y = Y(v);
      if (!pen) { g.moveTo(x, y); pen = true; } else g.lineTo(x, y);
    }
    g.stroke();

    // gaps shaded (no fix)
    g.fillStyle = "rgba(255,122,26,0.08)"; let gapStart = null;
    for (const r of this.buf) {
      if (r.t < tmin) continue;
      if (r[this.key] == null) { if (gapStart == null) gapStart = r.t; }
      else if (gapStart != null) { g.fillRect(X(gapStart), padT, Math.max(1, X(r.t) - X(gapStart)), h); gapStart = null; }
    }
    if (gapStart != null) g.fillRect(X(gapStart), padT, Math.max(1, X(tmax) - X(gapStart)), h);

    if (cursorT != null) { g.strokeStyle = "#e8ecf2"; g.lineWidth = 1; g.beginPath(); g.moveTo(X(cursorT), padT); g.lineTo(X(cursorT), padT + h); g.stroke(); }
  }
}
function niceStep(raw) { const p = Math.pow(10, Math.floor(Math.log10(raw))); const m = raw / p; return (m < 1.5 ? 1 : m < 3.5 ? 2 : m < 7.5 ? 5 : 10) * p; }
function niceTicks(lo, hi, n) { const s = niceStep((hi - lo) / n); const out = []; for (let v = Math.ceil(lo / s) * s; v <= hi; v += s) out.push(v); return out; }

// ----------------------------------------------------------------- live view
const charts = [new Chart($("#cX"), "x", { minSpan: 4 }),
                new Chart($("#cY"), "y", { minSpan: 4 }),
                new Chart($("#cT"), "theta", { minSpan: 2 })];
let lastRow = null, lastRowAt = 0, clockOffset = null;   // host monotonic -> performance.now()

function onRows(rows) {
  const nowPerf = performance.now() / 1000;
  for (const a of rows) {
    const r = { t: a[0], x: a[1], y: a[2], theta: a[3], tier: a[4], fps: a[5], frame: a[6] };
    for (const c of charts) c.push(r);
    lastRow = r;
  }
  if (rows.length) { clockOffset = nowPerf - rows[rows.length - 1][0]; lastRowAt = nowPerf; }
}
function hostNow() { return clockOffset == null ? 0 : performance.now() / 1000 - clockOffset; }

function renderLive() {
  if (!$("#livePane").hidden) {
    const now = hostNow();
    for (const c of charts) { c.prune(now); c.draw(now, null); }
    const fresh = lastRow && (performance.now() / 1000 - lastRowAt) < 2;
    $("#rX").textContent = fresh ? fmt(lastRow.x) : "—";
    $("#rY").textContent = fresh ? fmt(lastRow.y) : "—";
    $("#rTheta").textContent = fresh ? fmt(lastRow.theta, 2) : "—";
    $("#rTier").textContent = fresh ? (TIER[lastRow.tier] || lastRow.tier) : "no data";
    $("#rFps").textContent = fresh ? lastRow.fps.toFixed(0) + " fps" : "—";
    $("#rFrame").textContent = fresh ? lastRow.frame : "—";
  }
  requestAnimationFrame(renderLive);
}
requestAnimationFrame(renderLive);

// -- events --------------------------------------------------------------
function connectEvents() {
  const es = new EventSource("/api/live/events");
  es.onmessage = (e) => {
    const m = JSON.parse(e.data);
    if (m.rows) onRows(m.rows);
    if (m.status) applyStatus(m.status);
  };
  es.onerror = () => { setPill("#pillSerial", "off", "Serial: dashboard offline"); };
}
connectEvents();

function setPill(sel, cls, text) { const p = $(sel); p.className = "pill " + cls + (sel === "#pillRec" ? " rec" : ""); p.lastChild.textContent = text; }

function applyStatus(s) {
  const se = s.serial, cam = s.camera;
  if (se.state === "connected" && se.age_s != null && se.age_s < 2)
    setPill("#pillSerial", "ok", `Serial ${se.port.replace("/dev/", "")} · ${se.fps} rows/s`);
  else if (se.state === "connected") setPill("#pillSerial", "warn", `Serial ${se.port.replace("/dev/", "")} · silent`);
  else setPill("#pillSerial", "off", "Serial: " + (se.state === "searching" ? "no camera on USB" : se.state));

  if (cam.state === "connected") setPill("#pillCam", "ok", `Camera · ${cam.fps} fps`);
  else if (cam.state === "idle") setPill("#pillCam", "warn", "Camera · idle");
  else setPill("#pillCam", "off", "Camera: " + cam.state);
  camRetry = cam.state === "unreachable" ? 15000 : 3000;
  const off = $("#camOffline");
  off.hidden = cam.state === "connected";
  $("#camOfflineWhy").textContent = cam.state === "unreachable"
    ? `${cam.url} is not reachable. Is the camera on this WiFi?` + (cam.error ? " (" + cam.error + ")" : "")
    : cam.state === "idle" ? "Connecting…" : (cam.error || "Waiting for the camera.");

  RECORDING = s.recording;
  $("#pillRec").hidden = !RECORDING;
  const b = $("#recBtn");
  if (RECORDING) {
    b.textContent = "■ Stop"; b.classList.add("on"); $("#recName").disabled = true;
    $("#recInfo").textContent = `Recording "${RECORDING.name}" · ${RECORDING.elapsed_s.toFixed(0)} s · ${RECORDING.rows} rows · ${RECORDING.frames} frames`;
  } else {
    b.textContent = "● Record"; b.classList.remove("on"); $("#recName").disabled = false;
  }
}

// -- video ----------------------------------------------------------------
function startLiveVideo() { $("#liveImg").src = "/live/stream?" + Date.now(); }
// Back off when the camera is known unreachable: a 3 s retry against a
// camera that is not on the network is just a failed request every 3 s for
// ever (and a console error each time).
let camRetry = 3000;
$("#liveImg").onerror = () => setTimeout(() => { if (!$("#livePane").hidden) startLiveVideo(); }, camRetry);
function stopLiveVideo() { $("#liveImg").src = ""; }
// Start the MJPEG <img> only AFTER the page's load event. A multipart stream
// never ends, so an <img> on it that starts during load keeps the document
// in "loading" for ever -- the tab spinner never stops and anything waiting
// for load (automation, some extensions) hangs.
if (document.readyState === "complete") startLiveVideo();
else window.addEventListener("load", startLiveVideo);

// -- record ----------------------------------------------------------------
$("#recBtn").onclick = async () => {
  if (RECORDING) {
    const r = await api("/api/record/stop", { method: "POST" });
    $("#recInfo").textContent = r.error || (r.video ? "Saved. Encoding video…" : "Saved (data only — no video: camera was not reachable).");
    $("#recName").value = "";
    setTimeout(loadSessions, 300);
  } else {
    const name = $("#recName").value.trim();
    const r = await api("/api/record/start", { method: "POST", headers: { "Content-Type": "application/json" },
                                               body: JSON.stringify({ name }) });
    $("#recInfo").textContent = r.error ? r.error : "Recording…";
    setTimeout(loadSessions, 300);
  }
};

// ------------------------------------------------------------- recordings
async function loadSessions() {
  const list = await api("/api/sessions");
  const ul = $("#sessions"); ul.innerHTML = "";
  $("#runCount").textContent = list.length || "";
  $("#railEmpty").hidden = list.length > 0;
  for (const m of list) {
    const li = document.createElement("li");
    li.className = (SESSION && SESSION.id === m.id) ? "sel" : "";
    const when = new Date((m.created || 0) * 1000);
    const dur = m.duration_s != null ? ` · ${m.duration_s.toFixed(0)} s` : "";
    const st = m.state === "recording" ? " · recording" : m.state === "encoding" ? " · encoding…" : m.state === "error" ? " · error" : "";
    li.innerHTML = `<b>${esc(m.name)}</b><span>${when.toLocaleDateString()} ${when.toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" })}${dur}${st}</span>`;
    li.onclick = () => openSession(m.id);
    ul.appendChild(li);
  }
  if (list.some((m) => m.state === "encoding" || m.state === "recording")) setTimeout(loadSessions, 2000);
}
const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));

const pcharts = [new Chart($("#pcX"), "x", { minSpan: 4 }),
                 new Chart($("#pcY"), "y", { minSpan: 4 }),
                 new Chart($("#pcT"), "theta", { minSpan: 2 })];
let PROWS = [], PVID0 = 0;

async function openSession(id) {
  const m = await api("/api/session/" + id);
  if (m.error) return;
  SESSION = m;
  $("#livePane").hidden = true; $("#sessionPane").hidden = false; stopLiveVideo();
  $("#runName").textContent = m.name;
  const when = new Date((m.created || 0) * 1000);
  let meta = `${when.toLocaleString()} · ${m.duration_s != null ? m.duration_s.toFixed(1) + " s" : m.state} · ${m.rows || 0} rows at ${m.data_fps || 0}/s` + (m.video ? ` · ${m.frames} frames at ${m.video_fps}/s` : " · no video");
  if (m.gaps) meta += ` · ⚠ ${m.gaps} data gap${m.gaps > 1 ? "s" : ""}, worst ${m.max_gap_s}s`;
  if (m.reconnects) meta += ` · ${m.reconnects} serial reconnect(s)`;
  $("#runMeta").textContent = meta;
  $("#notes").value = m.notes || "";
  $("#dlCsv").href = `/media/${id}/frames.csv`; $("#dlCsv").download = `${safe(m.name)}-data.csv`;
  $("#dlIdx").href = `/media/${id}/video_index.csv`; $("#dlIdx").download = `${safe(m.name)}-video-timing.csv`;
  $("#dlIdx").hidden = !m.video;
  $("#dlVideo").hidden = !(m.video && m.state === "done");
  $("#dlVideo").href = `/media/${id}/video.mp4`; $("#dlVideo").download = `${safe(m.name)}.mp4`;
  $("#fileNote").textContent = `data/${id}/`;
  $("#encMsg").hidden = m.state !== "encoding";
  $("#noVideo").hidden = !!m.video;
  const v = $("#video"); v.hidden = !(m.video && m.state === "done");
  v.src = (m.video && m.state === "done") ? `/media/${id}/video.mp4` : "";

  // data for the playback readout and charts
  PROWS = parseCsv(await fetch(`/media/${id}/frames.csv`).then((r) => r.text()));
  PVID0 = 0;
  if (m.video) {
    const idx = await fetch(`/media/${id}/video_index.csv`).then((r) => r.text());
    const first = idx.split("\n")[1]; if (first) PVID0 = parseFloat(first.split(",")[1]) || 0;
  }
  for (const c of pcharts) c.setStatic(PROWS);
  drawPlayback(PVID0);
  if (m.state === "encoding") setTimeout(() => { if (SESSION && SESSION.id === id) openSession(id); }, 3000);
  loadSessions();
}
const safe = (s) => String(s).replace(/[^\w\-]+/g, "_").slice(0, 60);

function parseCsv(text) {
  const out = []; const lines = text.split("\n");
  for (let i = 1; i < lines.length; i++) {
    const p = lines[i].split(","); if (p.length < 12) continue;
    const f = (s) => s === "" ? null : parseFloat(s);
    out.push({ t: parseFloat(p[0]), frame: p[2], tier: p[3] || "none", theta: f(p[4]), x: f(p[5]), y: f(p[6]), fps: f(p[11]) });
  }
  return out;
}
function rowAt(t) {   // binary search on t
  let lo = 0, hi = PROWS.length - 1; if (!PROWS.length) return null;
  while (lo < hi) { const mid = (lo + hi + 1) >> 1; if (PROWS[mid].t <= t) lo = mid; else hi = mid - 1; }
  return PROWS[lo];
}
function drawPlayback(t) {
  for (const c of pcharts) c.draw(0, t);
  const r = rowAt(t);
  $("#pTime").textContent = t.toFixed(2) + " s";
  if (!r || Math.abs(r.t - t) > 0.5) { $("#pX").textContent = $("#pY").textContent = $("#pTheta").textContent = "—"; $("#pTier").textContent = "—"; return; }
  $("#pX").textContent = fmt(r.x); $("#pY").textContent = fmt(r.y); $("#pTheta").textContent = fmt(r.theta, 2);
  $("#pTier").textContent = TIER[r.tier] || r.tier;
}
$("#video").addEventListener("timeupdate", () => drawPlayback($("#video").currentTime + PVID0));
setInterval(() => { if (!$("#sessionPane").hidden && !$("#video").paused) drawPlayback($("#video").currentTime + PVID0); }, 50);
// click a chart to seek
for (const c of pcharts) c.c.addEventListener("click", (e) => {
  if (!PROWS.length) return;
  const rect = c.c.getBoundingClientRect(); const fr = (e.clientX - rect.left - 44) / (rect.width - 56);
  const t0 = PROWS[0].t, t1 = PROWS[PROWS.length - 1].t; const t = Math.max(t0, Math.min(t1, t0 + fr * (t1 - t0)));
  const v = $("#video"); if (!v.hidden && v.src) v.currentTime = Math.max(0, t - PVID0); else drawPlayback(t);
});

$("#goLive").onclick = () => { SESSION = null; $("#sessionPane").hidden = true; $("#livePane").hidden = false; startLiveVideo(); loadSessions(); };

$("#runName").addEventListener("blur", () => {
  if (!SESSION) return; const name = $("#runName").textContent.trim(); if (!name || name === SESSION.name) return;
  fetch(`/api/session/${SESSION.id}/rename`, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ name }) })
    .then(() => { SESSION.name = name; loadSessions(); });
});
$("#runName").addEventListener("keydown", (e) => { if (e.key === "Enter") { e.preventDefault(); e.target.blur(); } });
let noteTimer = null;
$("#notes").addEventListener("input", () => {
  clearTimeout(noteTimer);
  noteTimer = setTimeout(() => SESSION && fetch(`/api/session/${SESSION.id}/notes`, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ notes: $("#notes").value }) }), 600);
});
$("#delRun").onclick = () => {
  if (!SESSION || !confirm(`Delete "${SESSION.name}" and its files?`)) return;
  fetch("/api/session/" + SESSION.id, { method: "DELETE" }).then(() => $("#goLive").onclick());
};

// ---------------------------------------------------------------- settings
// Pick the option closest to `v` — the camera may hold a value that is not
// one of our presets (someone set it over HTTP), and a <select> that silently
// shows the wrong thing would make the next save change something the user
// never touched.
function setNearest(sel, v) {
  if (v == null) return;
  const opts = [...sel.options].map((o) => parseFloat(o.value));
  let best = 0;
  for (let i = 1; i < opts.length; i++)
    if (Math.abs(opts[i] - v) < Math.abs(opts[best] - v)) best = i;
  sel.selectedIndex = best;
}

async function loadStreamCfg() {
  const msg = $("#streamMsg");
  const c = await api("/api/camera/stream");
  if (c.error) {
    msg.textContent = "Camera not reachable — these cannot be read or changed right now.";
    msg.className = "note err";
    for (const id of ["#sDiv", "#sScale", "#sQual", "#sPersist"]) $(id).disabled = true;
    return;
  }
  for (const id of ["#sDiv", "#sScale", "#sQual", "#sPersist"]) $(id).disabled = false;
  setNearest($("#sDiv"), c.mjpeg_divisor);
  setNearest($("#sScale"), c.mjpeg_scale);
  setNearest($("#sQual"), c.mjpeg_quality);
  msg.textContent = "Applies immediately. Watch the Camera pill to see the effect.";
  msg.className = "note";
}

$("#settingsBtn").onclick = async () => {
  SETTINGS = await api("/api/settings");
  $("#sCam").value = SETTINGS.camera_url || ""; $("#sSerial").value = SETTINGS.serial_port || ""; $("#sWindow").value = SETTINGS.window_s || 20;
  $("#sPersist").checked = false;     // default to a try-it-out change
  $("#settings").showModal();
  loadStreamCfg();
};
$("#sSave").onclick = async (e) => {
  e.preventDefault();
  const s = await api("/api/settings", { method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ camera_url: $("#sCam").value, serial_port: $("#sSerial").value, window_s: $("#sWindow").value }) });
  SETTINGS = s; WINDOW_S = s.window_s || 20;
  if (!$("#sDiv").disabled) {
    const r = await api("/api/camera/stream", { method: "POST", headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ mjpeg_divisor: $("#sDiv").value, mjpeg_scale: $("#sScale").value,
                               mjpeg_quality: $("#sQual").value, persist: $("#sPersist").checked }) });
    if (r.error) { $("#streamMsg").textContent = "Could not apply: " + r.error; $("#streamMsg").className = "note err"; return; }
  }
  $("#settings").close();
  // the picture size may have changed, so pull a fresh stream
  stopLiveVideo(); setTimeout(startLiveVideo, 200);
};
api("/api/settings").then((s) => { SETTINGS = s; WINDOW_S = s.window_s || 20; });

loadSessions();
