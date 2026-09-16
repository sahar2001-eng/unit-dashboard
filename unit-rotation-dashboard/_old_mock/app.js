/* UNIT grid rotation analytics -- front end.
   Charts are hand-drawn SVG so the page works with no network access. */

const $ = (s) => document.querySelector(s);
const api = (u, o) => fetch(u, o).then((r) => r.json());

let SESSION = null;      // current session meta + series
let CHARTS = [];         // registered chart drawers
let POLL = null;

/* ------------------------------------------------------------- sessions */
async function loadSessions(selectId) {
  const list = await api("/api/sessions");
  const ul = $("#sessions");
  ul.innerHTML = "";
  $("#runCount").textContent = list.length ? list.length : "";
  $("#railEmpty").hidden = list.length > 0;

  list.forEach((m) => {
    const li = document.createElement("li");
    li.dataset.id = m.id;
    if (SESSION && SESSION.id === m.id) li.classList.add("on");
    const when = new Date(m.created * 1000);
    const s = m.summary;
    li.innerHTML =
      `<span class="nm"></span>
       <span class="sub">${when.toLocaleDateString()} ${when.toLocaleTimeString(
        [], { hour: "2-digit", minute: "2-digit" })}</span>` +
      (m.state === "done" && s
        ? `<span class="sub">${s.total_rotation_deg > 0 ? "+" : ""}${
            s.total_rotation_deg}° · ${s.duration_s}s · ${s.junction_pct}% lock</span>`
        : m.state === "running"
        ? `<span class="badge run">analysing…</span>`
        : `<span class="badge err">failed</span>`);
    li.querySelector(".nm").textContent = m.name;
    li.onclick = () => openSession(m.id);
    ul.appendChild(li);
  });
  if (selectId) openSession(selectId);
}

async function openSession(id) {
  const m = await api("/api/session/" + id);
  if (m.error) return;
  SESSION = m;
  document.querySelectorAll(".sessions li").forEach((li) =>
    li.classList.toggle("on", li.dataset.id === id));

  if (m.state === "running") { watch(id); return; }

  $("#uploadPane").hidden = true;
  $("#resultPane").hidden = false;

  if (m.state === "error") {
    $("#runName").textContent = m.name;
    $("#runMeta").textContent = "Analysis failed: " + (m.error || "unknown error");
    return;
  }

  const s = m.summary;
  $("#runName").textContent = m.name;
  $("#runMeta").textContent =
    `${s.frames} frames · ${s.fps} fps · ${s.width}×${s.height} · ` +
    `analysed in ${s.processing_s}s (${s.ms_per_frame} ms/frame)`;

  const v = $("#video");
  v.src = `/media/${id}/annotated.mp4`;
  v.load();
  v.addEventListener("loadeddata", () => { if (v.currentTime === 0) v.currentTime = 0.001; },
                     { once: true });
  $("#codecWarn").hidden = s.browser_playable !== false;

  $("#dlVideo").href = `/media/${id}/annotated.mp4`;
  $("#dlCsv").href = `/media/${id}/frames.csv`;
  $("#dlJson").href = `/media/${id}/summary.json`;
  $("#notes").value = m.notes || "";

  drawHealth(s);
  drawCoverage(s);
  drawTurns(s);
  buildCharts(m.series, s);
  readoutAt(0);
}

/* -------------------------------------------------------------- upload */
function startUpload(file) {
  const fd = new FormData();
  fd.append("video", file);
  fd.append("step", $("#step").value);
  fd.append("scale", $("#scale").value);
  fd.append("smooth", $("#smooth").checked ? "1" : "0");

  $("#uploadError").hidden = true;
  $("#progress").hidden = false;
  $("#barFill").style.width = "3%";
  $("#progressText").textContent = "Uploading " + file.name + "…";

  fetch("/api/upload", { method: "POST", body: fd })
    .then((r) => r.json())
    .then((r) => {
      if (r.error) throw new Error(r.error);
      fetch("/api/version").then(r=>r.json()).then(v=>{
  $("#verTag").textContent = "engine " + v.engine_version;
});
loadSessions();
      watch(r.id);
    })
    .catch((e) => {
      $("#progress").hidden = true;
      $("#uploadError").hidden = false;
      $("#uploadError").textContent = e.message;
    });
}

function watch(id) {
  $("#uploadPane").hidden = false;
  $("#resultPane").hidden = true;
  $("#progress").hidden = false;
  clearInterval(POLL);
  POLL = setInterval(async () => {
    const p = await api("/api/progress/" + id);
    if (p.state === "done") {
      clearInterval(POLL);
      $("#progress").hidden = true;
      SESSION = null;
      loadSessions(id);
    } else if (p.state === "error") {
      clearInterval(POLL);
      $("#progress").hidden = true;
      $("#uploadError").hidden = false;
      $("#uploadError").textContent = p.error || "Analysis failed.";
      fetch("/api/version").then(r=>r.json()).then(v=>{
  $("#verTag").textContent = "engine " + v.engine_version;
});
loadSessions();
    } else {
      const pc = p.total ? Math.max(3, (100 * p.done) / p.total) : 6;
      $("#barFill").style.width = pc + "%";
      $("#progressText").textContent = p.total
        ? `Analysing frame ${p.done} of ${p.total}`
        : "Analysing…";
    }
  }, 500);
}

/* -------------------------------------------------------------- charts */
const NS = "http://www.w3.org/2000/svg";
const el = (n, a = {}) => {
  const e = document.createElementNS(NS, n);
  for (const k in a) e.setAttribute(k, a[k]);
  return e;
};

function niceTicks(lo, hi, n = 4) {
  const span = hi - lo || 1;
  const raw = span / n;
  const mag = Math.pow(10, Math.floor(Math.log10(raw)));
  const step = [1, 2, 2.5, 5, 10].map((m) => m * mag)
    .find((s) => s >= raw) || 10 * mag;
  const out = [];
  for (let v = Math.ceil(lo / step) * step; v <= hi + 1e-9; v += step) out.push(v);
  return out;
}

function buildCharts(series, summary) {
  const host = $("#charts");
  host.innerHTML = "";
  CHARTS = [];

  const t = series.t;
  const tmax = t[t.length - 1] || 1;

  const groups = [
    { name: "Where the robot is",
      note: "X counts down to the next junction and resets to 80 when it passes one.",
      specs: [
        { title: "Distance to the next junction", unit: "millimetres", h: 96,
          marks: series.crossings,
          series: [{ d: series.x, cls: "trace", color: "var(--blue-vivid)" }] },
        { title: "Distance travelled along the tape", unit: "millimetres", h: 72,
          zero: true,
          series: [{ d: series.travel, cls: "trace", color: "#5B7BD5" }] },
        { title: "Offset from the tape centreline", unit: "millimetres", h: 72,
          zero: true,
          series: [{ d: series.y, cls: "trace", color: "var(--mint)" }] },
      ] },
    { name: "Which way it is pointing",
      note: "Shaded bands are the turns the tool found, labelled with their measured size.",
      specs: [
        { title: "Heading — how far the robot has turned", unit: "degrees",
          h: 128, bands: true, series: [
            { d: series.heading_raw, cls: "trace raw" },
            { d: series.heading, cls: "trace", color: "var(--blue-vivid)" }] },
        { title: "Turn rate", unit: "degrees / second", h: 72, zero: true,
          series: [{ d: series.rate, cls: "trace", color: "var(--amber)" }] },
        { title: "Tape angle as measured — wraps every 90°", unit: "degrees",
          h: 72, zero: true, gaps: true,
          series: [{ d: series.theta, cls: "trace", color: "#7B8AB8" }] },
      ] },
    { name: "How much to trust it",
      note: "A real bar fits a line to about 0.2 px; noise fits at about 45 px.",
      specs: [
        { title: "Line-fit residual", unit: "pixels", h: 68,
          series: [{ d: series.res, cls: "trace", color: "#B0B8CC" }] },
        { title: "Otsu threshold", unit: "grey level", h: 60,
          series: [{ d: series.otsu, cls: "trace", color: "#9B6BD6" }] },
        { title: "Image sharpness — drops when the tape smears", unit: "variance",
          h: 60, series: [{ d: series.sharp, cls: "trace", color: "#C08A4A" }] },
      ] },
  ];

  groups.forEach((g) => {
    const sec = document.createElement("section");
    sec.className = "chart-group";
    sec.innerHTML = `<div class="group-head"><h3></h3><p></p></div>`;
    sec.querySelector("h3").textContent = g.name;
    sec.querySelector("p").textContent = g.note;
    g.specs.forEach((sp) => sec.appendChild(makeChart(sp, t, tmax, summary)));
    host.appendChild(sec);
  });

  const strip = document.createElement("section");
  strip.className = "chart-group";
  strip.appendChild(makeLockStrip(series, t, tmax));
  host.appendChild(strip);
}

function makeChart(sp, t, tmax, summary) {
  const wrap = document.createElement("div");
  wrap.className = "chart";
  wrap.innerHTML =
    `<div class="chart-head"><h4></h4><span class="unit">${sp.unit}</span></div>`;
  wrap.querySelector("h4").textContent = sp.title;

  const W = 1000, H = sp.h, PAD_L = 44, PAD_R = 10, PAD_T = 8, PAD_B = 16;
  const svg = el("svg", { viewBox: `0 0 ${W} ${H}`, preserveAspectRatio: "none" });
  svg.style.height = H + "px";

  let lo = Infinity, hi = -Infinity;
  sp.series.forEach((s) =>
    s.d.forEach((v) => { if (v != null) { if (v < lo) lo = v; if (v > hi) hi = v; } }));
  if (!isFinite(lo)) { lo = 0; hi = 1; }
  if (hi - lo < 1e-6) { hi += 0.5; lo -= 0.5; }
  const pad = (hi - lo) * 0.12;
  lo -= pad; hi += pad;

  const X = (i) => PAD_L + ((W - PAD_L - PAD_R) * t[i]) / tmax;
  const Y = (v) => PAD_T + (H - PAD_T - PAD_B) * (1 - (v - lo) / (hi - lo));

  if (sp.bands && summary.turns) {
    summary.turns.forEach((tn, i) => {
      const x1 = X(tn.start_frame), x2 = X(tn.end_frame);
      svg.appendChild(el("rect", { class: "turn-band", x: x1, y: PAD_T,
        width: Math.max(1, x2 - x1), height: H - PAD_T - PAD_B }));
      const lab = el("text", { class: "turn-label", x: (x1 + x2) / 2,
        y: PAD_T + 11, "text-anchor": "middle" });
      lab.textContent = (tn.degrees > 0 ? "+" : "") + tn.degrees.toFixed(1) + "°";
      svg.appendChild(lab);
    });
  }

  if (sp.marks) {
    sp.marks.forEach((i) => {
      svg.appendChild(el("line", { x1: X(i), x2: X(i), y1: PAD_T, y2: H - PAD_B,
        stroke: "#D0332E", "stroke-width": 1, opacity: .5 }));
    });
  }

  niceTicks(lo, hi).forEach((v) => {
    svg.appendChild(el("line", { class: v === 0 ? "zero-line" : "grid-line",
      x1: PAD_L, x2: W - PAD_R, y1: Y(v), y2: Y(v) }));
    const tx = el("text", { class: "axis-text", x: PAD_L - 6, y: Y(v) + 3,
      "text-anchor": "end" });
    tx.textContent = Math.abs(v) >= 100 ? v.toFixed(0) : v.toFixed(1);
    svg.appendChild(tx);
  });

  sp.series.forEach((s) => {
    let d = "", pen = false;
    s.d.forEach((v, i) => {
      if (v == null) { pen = false; return; }
      if (sp.gaps && i > 0 && s.d[i - 1] != null &&
          Math.abs(v - s.d[i - 1]) > 45) pen = false;    // don't draw the wrap
      d += (pen ? "L" : "M") + X(i).toFixed(1) + " " + Y(v).toFixed(1) + " ";
      pen = true;
    });
    const p = el("path", { class: s.cls, d });
    if (s.color) p.setAttribute("stroke", s.color);
    svg.appendChild(p);
  });

  const head = el("line", { class: "playhead", x1: PAD_L, x2: PAD_L,
    y1: PAD_T, y2: H - PAD_B, opacity: 0 });
  svg.appendChild(head);

  const seek = (ev) => {
    const r = svg.getBoundingClientRect();
    const frac = (ev.clientX - r.left) / r.width;
    const px = frac * W;
    const tt = Math.max(0, Math.min(tmax, ((px - PAD_L) / (W - PAD_L - PAD_R)) * tmax));
    readoutAt(Math.round((tt / tmax) * (t.length - 1)));   // always responds
    const v = $("#video");
    if (v.duration) v.currentTime = Math.min(v.duration, tt);
  };
  svg.addEventListener("pointerdown", (e) => {
    try { svg.setPointerCapture(e.pointerId); } catch (_) { /* not fatal */ }
    seek(e);
  });
  svg.addEventListener("pointermove", (e) => { if (e.buttons) seek(e); });

  CHARTS.push({ setHead: (time) => {
    const x = PAD_L + ((W - PAD_L - PAD_R) * time) / tmax;
    head.setAttribute("x1", x); head.setAttribute("x2", x);
    head.setAttribute("opacity", 1);
  }});

  wrap.appendChild(svg);
  return wrap;
}

function makeLockStrip(series, t, tmax) {
  const wrap = document.createElement("div");
  wrap.className = "chart";
  wrap.innerHTML =
    `<div class="chart-head"><h4>Lock quality per frame</h4>
     <span class="unit">green: junction · blue: mini junction · amber: one bar · red: angle only</span></div>`;
  const W = 1000, H = 22, PAD_L = 44, PAD_R = 10;
  const svg = el("svg", { viewBox: `0 0 ${W} ${H}`, preserveAspectRatio: "none" });
  svg.style.height = H + "px";
  const col = { 1: "#1C8F63", 2: "#3355E0", 3: "#D9840A", 0: "#D0332E" };
  const X = (i) => PAD_L + ((W - PAD_L - PAD_R) * t[i]) / tmax;
  let i = 0;
  while (i < series.tier.length) {
    let j = i;
    while (j + 1 < series.tier.length && series.tier[j + 1] === series.tier[i]) j++;
    svg.appendChild(el("rect", { x: X(i), y: 4,
      width: Math.max(0.8, X(j) - X(i) + 0.8), height: 12,
      fill: col[series.tier[i]] || "#ccc", rx: 1 }));
    i = j + 1;
  }
  const head = el("line", { class: "playhead", x1: PAD_L, x2: PAD_L, y1: 0, y2: H, opacity: 0 });
  svg.appendChild(head);
  CHARTS.push({ setHead: (time) => {
    const x = PAD_L + ((W - PAD_L - PAD_R) * time) / tmax;
    head.setAttribute("x1", x); head.setAttribute("x2", x);
    head.setAttribute("opacity", 1);
  }});
  wrap.appendChild(svg);
  return wrap;
}

/* ------------------------------------------------------------- readout */
function readoutAt(i) {
  if (!SESSION || !SESSION.series) return;
  const s = SESSION.series;
  i = Math.max(0, Math.min(s.t.length - 1, i));
  const f = (v, d, u) => (v == null ? "—" : (v > 0 ? "+" : "") + v.toFixed(d) + u);
  $("#rHeading").textContent = f(s.heading[i], 2, "°");
  $("#rTheta").textContent = f(s.theta[i], 2, "°");
  $("#rRate").textContent = f(s.rate[i], 0, " °/s");
  $("#rX").textContent = s.x[i] == null ? "—" : s.x[i].toFixed(1) + " mm";
  $("#rY").textContent = s.y[i] == null ? "—" : (s.y[i] > 0 ? "+" : "") +
    s.y[i].toFixed(1) + " mm";
  const tier = s.tier[i];
  const rt = $("#rTier");
  rt.textContent = { 1: "junction", 2: "mini junction",
                     3: "one bar", 0: "angle only" }[tier];
  rt.className = "lock-" + tier;
  $("#rRes").textContent = s.res[i] == null ? "—" : s.res[i].toFixed(2) + " px";
  CHARTS.forEach((c) => c.setHead(s.t[i]));
}

/* --------------------------------------------------------------- cards */
function drawHealth(s) {
  const rows = [
    ["Position fix", s.x_available_pct + "%"],
    ["Heading", s.heading_available_pct + "%"],
    ["Junction crossings", s.junction_crossings],
    ["Travelled", s.travel_mm + " mm"],
    ["Total rotation", (s.total_rotation_deg > 0 ? "+" : "") + s.total_rotation_deg + "°"],
    ["Peak turn rate", s.peak_rate_dps + " °/s"],
    ["Fit residual (median)", s.median_residual_px != null
      ? s.median_residual_px.toFixed(2) + " px" : "—"],
    ["Heading jitter", s.jitter_smoothed_deg != null
      ? s.jitter_smoothed_deg.toFixed(3) + "°" : "—"],
    ["Lateral RMS", s.lateral_rms_mm != null ? s.lateral_rms_mm + " mm" : "—"],
    ["Threshold swing", s.otsu_swing + " levels"],
    ["Jumps rejected", s.jumps_rejected],
    ["Mode switches", s.mode_switches],
  ];
  $("#health").innerHTML = rows.map(
    (r) => `<div><span class="k">${r[0]}</span><span class="v">${r[1]}</span></div>`
  ).join("");

  const tiers = [
    ["Full junction — X, Y and heading", s.junction_pct, "#1C8F63"],
    ["Mini junction — X from the mid-cell gap", s.mini_junction_pct, "#3355E0"],
    ["Single bar — Y and heading only", s.single_bar_pct, "#D9840A"],
    ["Angle only — no position", s.m1_only_pct, "#D0332E"],
  ];
  $("#tiers").innerHTML = tiers.map((r) => `
    <div class="cov-row tier-row">
      <span class="muted">${r[0]}</span>
      <span class="cov-bar"><i style="width:${r[1]}%;background:${r[2]}"></i></span>
      <span class="pct">${r[1]}%</span>
    </div>`).join("");
}

function drawCoverage(s) {
  $("#coverage").innerHTML = (s.coverage || []).map((c) => `
    <div class="cov-row">
      <span class="muted">${c.from}° to ${c.to}°</span>
      <span class="cov-bar"><i style="width:${c.fix_pct}%"></i></span>
      <span class="pct">${c.frames ? c.fix_pct + "%" : "—"}</span>
    </div>`).join("");
}

function drawTurns(s) {
  const tb = $("#turnsBody");
  tb.innerHTML = "";
  const turns = s.turns || [];
  $("#noTurns").hidden = turns.length > 0;
  turns.forEach((t, i) => {
    const e = t.error_vs_90;
    const off = e == null
      ? `<span class="muted">partial</span>`
      : `<span class="off ${Math.abs(e) < 1.5 ? "good" : "bad"}">${
          e > 0 ? "+" : ""}${e.toFixed(2)}°</span>`;
    const tr = document.createElement("tr");
    tr.innerHTML = `
      <td>${i + 1}</td>
      <td>${t.start_s.toFixed(2)}s</td>
      <td>${t.duration_s.toFixed(2)}s</td>
      <td class="deg">${t.degrees > 0 ? "+" : ""}${t.degrees.toFixed(2)}°</td>
      <td>${t.peak_rate} °/s</td>
      <td>${off}</td>`;
    tr.onclick = () => {
      readoutAt(t.start_frame);
      const v = $("#video");
      if (v.duration) v.currentTime = t.start_s;
    };
    tb.appendChild(tr);
  });
}

/* ---------------------------------------------------------------- wire */
$("#pickBtn").onclick = () => $("#file").click();
$("#file").onchange = (e) => e.target.files[0] && startUpload(e.target.files[0]);
$("#newRun").onclick = () => {
  SESSION = null;
  $("#resultPane").hidden = true;
  $("#uploadPane").hidden = false;
  $("#progress").hidden = true;
  document.querySelectorAll(".sessions li").forEach((li) => li.classList.remove("on"));
};

const drop = $("#drop");
["dragenter", "dragover"].forEach((ev) =>
  drop.addEventListener(ev, (e) => { e.preventDefault(); drop.classList.add("hot"); }));
["dragleave", "drop"].forEach((ev) =>
  drop.addEventListener(ev, (e) => { e.preventDefault(); drop.classList.remove("hot"); }));
drop.addEventListener("drop", (e) => {
  const f = e.dataTransfer.files[0];
  if (f) startUpload(f);
});

$("#video").addEventListener("timeupdate", () => {
  if (!SESSION || !SESSION.series) return;
  const s = SESSION.series;
  const dur = $("#video").duration || s.t[s.t.length - 1];
  readoutAt(Math.round(($("#video").currentTime / dur) * (s.t.length - 1)));
});

$("#runName").addEventListener("blur", () => {
  if (!SESSION) return;
  const name = $("#runName").textContent.trim();
  if (!name || name === SESSION.name) return;
  fetch(`/api/session/${SESSION.id}/rename`, {
    method: "POST", headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ name }),
  }).then(() => { SESSION.name = name; fetch("/api/version").then(r=>r.json()).then(v=>{
  $("#verTag").textContent = "engine " + v.engine_version;
});
loadSessions(); });
});
$("#runName").addEventListener("keydown", (e) => {
  if (e.key === "Enter") { e.preventDefault(); e.target.blur(); }
});

let noteTimer = null;
$("#notes").addEventListener("input", () => {
  clearTimeout(noteTimer);
  noteTimer = setTimeout(() => {
    if (!SESSION) return;
    fetch(`/api/session/${SESSION.id}/notes`, {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ notes: $("#notes").value }),
    });
  }, 600);
});

$("#delRun").onclick = () => {
  if (!SESSION) return;
  if (!confirm(`Delete “${SESSION.name}” and its analysis?`)) return;
  fetch("/api/session/" + SESSION.id, { method: "DELETE" }).then(() => {
    SESSION = null;
    $("#resultPane").hidden = true;
    $("#uploadPane").hidden = false;
    fetch("/api/version").then(r=>r.json()).then(v=>{
  $("#verTag").textContent = "engine " + v.engine_version;
});
loadSessions();
  });
};

document.addEventListener("keydown", (e) => {
  if (e.target.isContentEditable || e.target.tagName === "TEXTAREA") return;
  const v = $("#video");
  if (e.key === " ") { e.preventDefault(); v.paused ? v.play() : v.pause(); }
  if (e.key === "ArrowRight" && SESSION) v.currentTime += 1 / (SESSION.summary.fps || 30);
  if (e.key === "ArrowLeft" && SESSION) v.currentTime -= 1 / (SESSION.summary.fps || 30);
});

fetch("/api/version").then(r=>r.json()).then(v=>{
  $("#verTag").textContent = "engine " + v.engine_version;
});
loadSessions();
