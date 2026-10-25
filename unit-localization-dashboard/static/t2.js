/* Tile 2 (UNIT16 hole grid) analytics -- front end.
   Every value shown comes from the AE3's own u16pipe.py + u16c, run over the
   clip in MicroPython by t2/t2_runner.py. This file only draws.
   Separate from app.js (Tile 1), same look. */

const $ = (s) => document.querySelector(s);
const api = (u, o) => fetch(u, o).then((r) => r.json());
const esc = (s) => String(s == null ? "" : s).replace(/[&<>"']/g,
  (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));

let SESSION = null, CHARTS = [], POLL = null, CASES = null, CASE_FILTER = "";
const CASE_NAME = {};

const ST = { 0: ["none", "#D0332E"], 1: ["road only", "#3355E0"], 2: ["cross only", "#D9840A"],
             3: ["both (junction)", "#1C8F63"], 4: ["HOLD", "#8A5BD6"] };

function refreshVersionTag() {
  api("/api/t2/version").then((v) => {
    $("#verTag").textContent = "tile 2 engine " + v.engine2 + " · pipeline " + v.pipeline;
  }).catch(() => {});
}

/* ------------------------------------------------------------- runs */
async function loadSessions(selectId) {
  const all = await api("/api/t2/sessions");
  const list = CASE_FILTER ? all.filter((m) => m.case === CASE_FILTER) : all;
  const ul = $("#sessions");
  ul.innerHTML = "";
  $("#runCount").textContent = list.length ? list.length : "";
  $("#railEmpty").hidden = list.length > 0;
  $("#railFilter").hidden = !CASE_FILTER;
  if (CASE_FILTER) $("#railFilterText").textContent =
    "case " + CASE_FILTER + " · " + list.length + " of " + all.length;
  list.forEach((m) => {
    const li = document.createElement("li");
    li.dataset.id = m.id;
    if (SESSION && SESSION.id === m.id) li.classList.add("on");
    const when = new Date(m.created * 1000);
    const s = m.summary;
    li.innerHTML =
      `<span class="nm"></span>
       <span class="sub">${when.toLocaleDateString()} ${when.toLocaleTimeString([],
        { hour: "2-digit", minute: "2-digit" })}</span>` +
      (m.case ? `<span class="case-tag">${esc(m.case)}</span>` : "") +
      (m.state === "done" && s
        ? `<span class="sub">${s.duration_s}s · junction ${s.pcts.both}% · ${s.codes.length} code${s.codes.length === 1 ? "" : "s"}</span>`
        : m.state === "running" ? `<span class="badge run">analysing…</span>`
        : `<span class="badge err">failed</span>`);
    li.querySelector(".nm").textContent = m.name;
    li.onclick = () => openSession(m.id);
    ul.appendChild(li);
  });
  if (selectId) openSession(selectId);
}

async function openSession(id) {
  const m = await api("/api/t2/session/" + id);
  if (m.error) return;
  SESSION = m;
  document.querySelectorAll(".sessions li").forEach((li) =>
    li.classList.toggle("on", li.dataset.id === id));
  if (m.state === "running") { watch(id); return; }
  $("#uploadPane").hidden = true;
  $("#resultPane").hidden = false;
  $("#runName").textContent = m.name;
  if (m.state === "error") {
    $("#runMeta").textContent = "Analysis failed: " + (m.error || "unknown error");
    $("#charts").innerHTML = "";
    return;
  }
  const s = m.summary;
  $("#runMeta").textContent =
    `${s.frames} frames · ${s.fps} fps · source ${s.width}×${s.height}` +
    (s.resized ? " (resized to 320×200)" : "") +
    ` · pipeline ${s.ms_per_frame} ms/frame on the server` +
    (m.rerun_of ? " · re-run" : "");
  const v = $("#video");
  v.src = `/media2/${id}/annotated.mp4`;
  v.load();
  v.addEventListener("loadeddata", () => { if (v.currentTime === 0) v.currentTime = 0.001; },
                     { once: true });
  $("#codecWarn").hidden = s.browser_playable !== false;
  $("#dlVideo").href = `/media2/${id}/annotated.mp4`;
  $("#dlCsv").href = `/media2/${id}/frames.csv`;
  $("#dlJson").href = `/media2/${id}/summary.json`;
  $("#dlPipe").href = `/media2/${id}/u16pipe_as_run.py`;
  $("#notes").value = m.notes || "";
  drawCase(m);
  drawCards(s);
  buildCharts(m.series, s);
  readoutAt(0);
}

/* ------------------------------------------------------------- upload */
function startUpload(file) {
  const fd = new FormData();
  fd.append("video", file);
  fd.append("scale", $("#scale").value);
  fd.append("case", $("#caseSel").value);
  $("#uploadError").hidden = true;
  $("#progress").hidden = false;
  $("#barFill").style.width = "3%";
  $("#progressText").textContent = "Uploading " + file.name + "…";
  fetch("/api/t2/upload", { method: "POST", body: fd })
    .then((r) => r.json())
    .then((r) => { if (r.error) throw new Error(r.error); loadSessions(); watch(r.id); })
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
    const p = await api("/api/t2/progress/" + id);
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
      loadSessions();
    } else {
      const pc = p.total ? Math.max(3, (100 * p.done) / p.total) : 6;
      $("#barFill").style.width = pc + "%";
      $("#progressText").textContent = p.total ? `Analysing… ${Math.round(pc)}%` : "Reading the video…";
    }
  }, 500);
}

/* ------------------------------------------------------------- explainers */
const EXPLAIN = {
  x: ["X", "Distance along the road from the junction to the image centre, in mm " +
      "(MM_PER_PX is still a placeholder). X comes ONLY from code junctions: the plain " +
      "wide bar half way between them is not used, so between junctions X is empty."],
  y: ["Y", "Offset across the road (the wide bar the robot drives on), in mm."],
  deg: ["Angle", "Road angle, continuous through turns (it does not reset at 90° or 360°)."],
  pres: ["Presence score", "Checks that a junction is a code junction: the 4 diagonal points " +
      "around it must be solid plate. Code junctions score about 100–200, plain junctions " +
      "0–40. Below the dashed line (70) the cross bar is dropped."],
  status: ["Lines found per frame", "Green: road and cross bar (junction, X and Y). Blue: road " +
      "only (Y and angle). Amber: cross bar only (X and angle). Purple: HOLD — lines were found " +
      "but the position did not fit where we expected to be, so no output. Red: nothing."],
  codes: ["Codes", "Each tick is a frame where a code was accepted (the same code read 2 frames " +
      "in a row). The label is the code; the letter is the direction."],
};
function openInfo(key) {
  const [t, b] = EXPLAIN[key] || ["", ""];
  if (!t) return;
  $("#infoTitle").textContent = t;
  $("#infoBody").textContent = b;
  $("#infoModal").hidden = false;
}
const infoBtn = (k) => `<button class="info-btn" data-info="${k}" aria-label="What is this?">i</button>`;

/* ------------------------------------------------------------- charts */
const NS = "http://www.w3.org/2000/svg";
const el = (n, a = {}) => { const e = document.createElementNS(NS, n);
  for (const k in a) e.setAttribute(k, a[k]); return e; };

function niceTicks(lo, hi, n = 4) {
  const raw = (hi - lo || 1) / n;
  const mag = Math.pow(10, Math.floor(Math.log10(raw)));
  const step = [1, 2, 2.5, 5, 10].map((m) => m * mag).find((s) => s >= raw) || 10 * mag;
  const out = [];
  for (let v = Math.ceil(lo / step) * step; v <= hi + 1e-9; v += step) out.push(v);
  return out;
}

const GEO = { W: 1000, L: 44, R: 10 };

function seekHandlers(svg, t, tmax) {
  const seek = (ev) => {
    const r = svg.getBoundingClientRect();
    const px = ((ev.clientX - r.left) / r.width) * GEO.W;
    const tt = Math.max(0, Math.min(tmax, ((px - GEO.L) / (GEO.W - GEO.L - GEO.R)) * tmax));
    readoutAt(Math.round((tt / tmax) * (t.length - 1)));
    const v = $("#video");
    if (v.duration) v.currentTime = Math.min(v.duration, tt);
  };
  svg.addEventListener("pointerdown", (e) => {
    try { svg.setPointerCapture(e.pointerId); } catch (_) { /* fine */ }
    seek(e);
  });
  svg.addEventListener("pointermove", (e) => { if (e.buttons) seek(e); });
}

function addHead(svg, y1, y2, tmax) {
  const head = el("line", { class: "playhead", x1: GEO.L, x2: GEO.L, y1, y2, opacity: 0 });
  svg.appendChild(head);
  CHARTS.push({ setHead: (time) => {
    const x = GEO.L + ((GEO.W - GEO.L - GEO.R) * time) / tmax;
    head.setAttribute("x1", x); head.setAttribute("x2", x); head.setAttribute("opacity", 1);
  } });
}

function lineChart(sp, t, tmax) {
  const wrap = document.createElement("div");
  wrap.className = "chart";
  wrap.innerHTML = `<div class="chart-head"><h4>${sp.title}${infoBtn(sp.key)}</h4><span class="unit">${sp.unit}</span></div>`;
  const H = sp.h, T = 8, B = 16, W = GEO.W;
  const svg = el("svg", { viewBox: `0 0 ${W} ${H}`, preserveAspectRatio: "none" });
  svg.style.height = H + "px";
  let lo = Infinity, hi = -Infinity;
  sp.d.forEach((v) => { if (v != null) { lo = Math.min(lo, v); hi = Math.max(hi, v); } });
  if (sp.ref != null) { lo = Math.min(lo, sp.ref); hi = Math.max(hi, sp.ref); }
  if (!isFinite(lo)) { lo = 0; hi = 1; }
  if (hi - lo < 1e-6) { hi += 0.5; lo -= 0.5; }
  const pad = (hi - lo) * 0.12; lo -= pad; hi += pad;
  const X = (i) => GEO.L + ((W - GEO.L - GEO.R) * t[i]) / tmax;
  const Y = (v) => T + (H - T - B) * (1 - (v - lo) / (hi - lo));
  niceTicks(lo, hi).forEach((v) => {
    svg.appendChild(el("line", { class: Math.abs(v) < 1e-9 ? "zero-line" : "grid-line",
      x1: GEO.L, x2: W - GEO.R, y1: Y(v), y2: Y(v) }));
    const tx = el("text", { class: "axis-text", x: GEO.L - 6, y: Y(v) + 3, "text-anchor": "end" });
    tx.textContent = Math.abs(v) >= 100 ? v.toFixed(0) : v.toFixed(1);
    svg.appendChild(tx);
  });
  if (sp.ref != null) svg.appendChild(el("line", { x1: GEO.L, x2: W - GEO.R, y1: Y(sp.ref),
    y2: Y(sp.ref), stroke: "#D0332E", "stroke-dasharray": "5 4", "stroke-width": 1.2, opacity: .7 }));
  // pen lifts where there is no value (more than 1 frame missing)
  let d = "", last = -9;
  sp.d.forEach((v, i) => {
    if (v == null) return;
    d += (i - last > 2 ? "M" : "L") + X(i).toFixed(1) + " " + Y(v).toFixed(1) + " ";
    last = i;
  });
  const p = el("path", { class: "trace", d, stroke: sp.color });
  svg.appendChild(p);
  if (sp.dots) sp.d.forEach((v, i) => { if (v != null && sp.d[i - 1] == null && sp.d[i + 1] == null)
    svg.appendChild(el("circle", { cx: X(i), cy: Y(v), r: 1.8, fill: sp.color })); });
  addHead(svg, T, H - B, tmax);
  seekHandlers(svg, t, tmax);
  wrap.appendChild(svg);
  return wrap;
}

function statusStrip(series, t, tmax) {
  const wrap = document.createElement("div");
  wrap.className = "chart";
  wrap.innerHTML = `<div class="chart-head"><h4>Lines found per frame${infoBtn("status")}</h4>
    <span class="unit">${[3, 1, 2, 4, 0].map((k) => `<i class="dot" style="background:${ST[k][1]}"></i>${ST[k][0]}`).join(" · ")}</span></div>`;
  const W = GEO.W, H = 22;
  const svg = el("svg", { viewBox: `0 0 ${W} ${H}`, preserveAspectRatio: "none" });
  svg.style.height = H + "px";
  const X = (i) => GEO.L + ((W - GEO.L - GEO.R) * t[i]) / tmax;
  const st = series.status;
  let i = 0;
  while (i < st.length) {
    let j = i;
    while (j + 1 < st.length && st[j + 1] === st[i]) j++;
    svg.appendChild(el("rect", { x: X(i), y: 4, width: Math.max(0.8, X(j) - X(i) + 0.8),
      height: 12, fill: ST[st[i]][1], rx: 1 }));
    i = j + 1;
  }
  addHead(svg, 0, H, tmax);
  seekHandlers(svg, t, tmax);
  wrap.appendChild(svg);
  return wrap;
}

function codeStrip(series, t, tmax) {
  const wrap = document.createElement("div");
  wrap.className = "chart";
  wrap.innerHTML = `<div class="chart-head"><h4>Codes accepted${infoBtn("codes")}</h4><span class="unit">code read 2 frames in a row</span></div>`;
  const W = GEO.W, H = 34;
  const svg = el("svg", { viewBox: `0 0 ${W} ${H}`, preserveAspectRatio: "none" });
  svg.style.height = H + "px";
  const X = (i) => GEO.L + ((W - GEO.L - GEO.R) * t[i]) / tmax;
  let lastLab = null, lastX = -999;
  series.code.forEach((c, i) => {
    if (!c) return;
    svg.appendChild(el("line", { x1: X(i), x2: X(i), y1: 18, y2: 30, stroke: "#16215C", "stroke-width": 1.2 }));
    if (c !== lastLab || X(i) - lastX > 60) {
      const tx = el("text", { x: X(i), y: 13, "text-anchor": "middle", class: "turn-label" });
      tx.textContent = c; svg.appendChild(tx);
      lastLab = c; lastX = X(i);
    }
  });
  addHead(svg, 0, H, tmax);
  seekHandlers(svg, t, tmax);
  wrap.appendChild(svg);
  return wrap;
}

function buildCharts(series, s) {
  const host = $("#charts");
  host.innerHTML = "";
  CHARTS = [];
  const t = series.t, tmax = t[t.length - 1] || 1;
  const g1 = document.createElement("section");
  g1.className = "chart-group";
  g1.innerHTML = `<div class="group-head"><h3>Where the robot is</h3><p>X and Y as the camera code reported them. Empty = not reported that frame.</p></div>`;
  g1.appendChild(lineChart({ title: "X", key: "x", unit: "mm (from the code junction)", h: 96, d: series.x, color: "#3355E0", dots: true }, t, tmax));
  g1.appendChild(lineChart({ title: "Y", key: "y", unit: "mm", h: 72, d: series.y, color: "#1C8F63" }, t, tmax));
  g1.appendChild(lineChart({ title: "Angle", key: "deg", unit: "degrees", h: 88, d: series.deg, color: "#16215C" }, t, tmax));
  host.appendChild(g1);
  const g2 = document.createElement("section");
  g2.className = "chart-group";
  g2.innerHTML = `<div class="group-head"><h3>How much to trust it</h3><p>Which lines were found, which junctions are code junctions, and which codes were read.</p></div>`;
  g2.appendChild(statusStrip(series, t, tmax));
  g2.appendChild(lineChart({ title: "Presence score", key: "pres", unit: "dashed: limit " + ((s.constants || {}).PRES_MIN || 70),
    h: 72, d: series.pres, color: "#D9840A", ref: (s.constants || {}).PRES_MIN || 70, dots: true }, t, tmax));
  g2.appendChild(codeStrip(series, t, tmax));
  host.appendChild(g2);
  host.querySelectorAll("[data-info]").forEach((b) => b.addEventListener("click", () => openInfo(b.dataset.info)));
}

/* ------------------------------------------------------------- readout */
function readoutAt(i) {
  if (!SESSION || !SESSION.series) return;
  const s = SESSION.series;
  i = Math.max(0, Math.min(s.t.length - 1, i));
  const sg = (v, d, u) => (v == null ? "—" : (v > 0 ? "+" : "") + v.toFixed(d) + u);
  $("#rFrame").textContent = `${i}  (${s.t[i].toFixed(2)} s)`;
  const st = s.status[i];
  const rl = $("#rLines");
  rl.textContent = ST[st][0];
  rl.style.color = ST[st][1];
  $("#rX").textContent = sg(s.x[i], 1, " mm");
  $("#rY").textContent = sg(s.y[i], 1, " mm");
  $("#rDeg").textContent = sg(s.deg[i], 1, "°");
  $("#rXnode").textContent = s.x[i] == null ? "—" : (s.xnode[i] || "? (not read yet)");
  $("#rNode").textContent = s.node[i] || "—";
  $("#rPres").textContent = s.pres[i] == null ? "—" : s.pres[i];
  CHARTS.forEach((c) => c.setHead(s.t[i]));
}

/* ------------------------------------------------------------- cards */
function kv(host, rows) {
  host.innerHTML = rows.map(([k, v]) => `<div><span class="k">${esc(k)}</span><span class="v">${esc(v)}</span></div>`).join("");
}

function drawCards(s) {
  const order = [["both", 3], ["road", 1], ["cross", 2], ["hold", 4], ["none", 0]];
  $("#statusBars").innerHTML = order.map(([k, c]) =>
    `<div class="cov-row tier-row"><span>${ST[c][0]}</span>
      <span class="cov-bar"><i style="width:${s.pcts[k]}%;background:${ST[c][1]}"></i></span>
      <span class="pct">${s.pcts[k]}%</span></div>`).join("");
  const tb = $("#codesBody");
  tb.innerHTML = "";
  $("#noCodes").hidden = s.codes.length > 0;
  s.codes.forEach((c) => {
    const tr = document.createElement("tr");
    tr.innerHTML = `<td class="deg">${esc(c.name)}</td><td>frame ${c.first_frame} · ${c.first_t.toFixed(2)} s</td><td>${c.frames}</td>`;
    tr.onclick = () => {
      readoutAt(c.first_frame);
      const v = $("#video");
      if (v.duration) v.currentTime = Math.min(v.duration, c.first_t);
    };
    tb.appendChild(tr);
  });
  const f = (v) => (v == null ? "—" : v.toFixed(1) + "°");
  kv($("#health"), [
    ["Frames", `${s.frames} · ${s.fps} fps · ${s.duration_s}s`],
    ["Result given", s.ok_pct + "% of frames"],
    ["X reported", s.x_pct + "% of frames"],
    ["Y reported", s.y_pct + "% of frames"],
    ["X with junction not named yet", s.x_unnamed_pct + "% of frames"],
    ["Angle start → end", s.angle_start == null ? "—" : `${f(s.angle_start)} → ${f(s.angle_end)}`],
    ["Analysed in", `${s.processing_s}s`],
    ["Pipeline", s.pipeline ? s.pipeline.combined : "—"],
  ]);
  const g = s.guards || {};
  kv($("#guards"), [
    ["Wrong angle", g.angle], ["Not perpendicular", g.perp],
    ["Position jump (HOLD)", g.jump], ["Plain bar dropped", g.plain],
    ["Code mix-ups", g.conflicts], ["Full searches", g.searches],
  ]);
  const c = s.constants || {};
  kv($("#consts"), [
    ["mm per pixel", c.MM_PER_PX != null ? c.MM_PER_PX + " (placeholder)" : "—"],
    ["Lens K1", c.LENS_K1 != null ? c.LENS_K1 : "—"],
    ["Presence limit", c.PRES_MIN != null ? c.PRES_MIN : "—"],
    ["Angle check", c.ANG_TOL != null ? "±" + c.ANG_TOL + "°" : "—"],
    ["Right-angle check", c.PERP_TOL != null ? "±" + c.PERP_TOL + "°" : "—"],
    ["Jump limit", c.JUMP_MAX != null ? c.JUMP_MAX + " px" : "—"],
  ]);
  const files = s.pipeline && s.pipeline.files ? Object.entries(s.pipeline.files).map(([k, v]) => k + " " + v).join(" · ") : "";
  $("#fpText").textContent = "Files used: " + files;
}

/* ------------------------------------------------------------- use cases */
async function loadCases() {
  CASES = await api("/api/t2/cases");
  const sel = $("#caseSel");
  sel.innerHTML = `<option value="">— none —</option>`;
  CASES.sections.forEach((sec) => {
    const og = document.createElement("optgroup");
    og.label = sec.id + ". " + sec.name;
    sec.cases.forEach((c) => {
      CASE_NAME[c.id] = c.title;
      const o = document.createElement("option");
      o.value = c.id; o.textContent = c.id + "  " + c.title;
      og.appendChild(o);
    });
    sel.appendChild(og);
  });
}

function drawCasesModal() {
  const host = $("#casesList");
  $("#casesTitle").textContent = CASES.title;
  host.innerHTML = "";
  CASES.sections.forEach((sec) => {
    const d = document.createElement("div");
    d.className = "case-sec";
    d.innerHTML = `<h4>${esc(sec.id)}. ${esc(sec.name)}</h4>`;
    sec.cases.forEach((c) => {
      const b = document.createElement("button");
      b.className = "case-row" + (CASE_FILTER === c.id ? " on" : "");
      b.innerHTML = `<span class="case-id">${esc(c.id)}</span><span>${esc(c.title)}</span>
        <span class="case-n ${c.runs ? "" : "zero"}">${c.runs} run${c.runs === 1 ? "" : "s"}</span>`;
      b.onclick = () => { CASE_FILTER = c.id; $("#casesModal").hidden = true; loadSessions(); };
      d.appendChild(b);
    });
    host.appendChild(d);
  });
}

function drawCase(m) {
  const host = $("#runCase");
  host.innerHTML = "";
  const sel = document.createElement("select");
  sel.className = "case-inline";
  sel.innerHTML = `<option value="">Use case: none</option>` +
    Object.keys(CASE_NAME).map((id) => `<option value="${esc(id)}">${esc(id)}  ${esc(CASE_NAME[id])}</option>`).join("");
  sel.value = m.case || "";
  sel.onchange = async () => {
    await api(`/api/t2/session/${m.id}/case`, { method: "POST",
      headers: { "Content-Type": "application/json" }, body: JSON.stringify({ case: sel.value }) });
    m.case = sel.value;
    loadSessions();
  };
  host.appendChild(sel);
}

/* ------------------------------------------------------------- wiring */
function init() {
  refreshVersionTag();
  loadCases().then(() => loadSessions());
  $("#newRun").onclick = () => {
    SESSION = null;
    document.querySelectorAll(".sessions li").forEach((li) => li.classList.remove("on"));
    $("#uploadPane").hidden = false; $("#resultPane").hidden = true;
    $("#progress").hidden = true; $("#uploadError").hidden = true;
  };
  $("#pickBtn").onclick = () => $("#file").click();
  $("#file").onchange = (e) => { if (e.target.files[0]) startUpload(e.target.files[0]); };
  const drop = $("#drop");
  ["dragenter", "dragover"].forEach((ev) => drop.addEventListener(ev, (e) => { e.preventDefault(); drop.classList.add("hot"); }));
  ["dragleave", "drop"].forEach((ev) => drop.addEventListener(ev, (e) => { e.preventDefault(); drop.classList.remove("hot"); }));
  drop.addEventListener("drop", (e) => { const f = e.dataTransfer.files[0]; if (f) startUpload(f); });
  $("#video").addEventListener("timeupdate", () => {
    if (!SESSION || !SESSION.series) return;
    const v = $("#video"), s = SESSION.series;
    const tmax = s.t[s.t.length - 1] || 1;
    readoutAt(Math.round((v.currentTime / tmax) * (s.t.length - 1)));
  });
  $("#runName").addEventListener("blur", () => {
    if (!SESSION) return;
    api(`/api/t2/session/${SESSION.id}/rename`, { method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ name: $("#runName").textContent }) }).then(() => loadSessions());
  });
  $("#runName").addEventListener("keydown", (e) => { if (e.key === "Enter") { e.preventDefault(); e.target.blur(); } });
  let nt = null;
  $("#notes").addEventListener("input", () => {
    clearTimeout(nt);
    nt = setTimeout(() => SESSION && api(`/api/t2/session/${SESSION.id}/notes`, { method: "POST",
      headers: { "Content-Type": "application/json" }, body: JSON.stringify({ notes: $("#notes").value }) }), 600);
  });
  $("#delRun").onclick = async () => {
    if (!SESSION || !confirm("Delete this run and its files?")) return;
    await api("/api/t2/session/" + SESSION.id, { method: "DELETE" });
    SESSION = null;
    $("#newRun").click();
    loadSessions();
  };
  $("#rrGo").onclick = async () => {
    if (!SESSION) return;
    const r = await api(`/api/t2/session/${SESSION.id}/rerun`, { method: "POST" });
    if (r.id) { loadSessions(); watch(r.id); }
  };
  $("#casesBtn").onclick = async () => { await loadCases(); CASES = await api("/api/t2/cases"); drawCasesModal(); $("#casesModal").hidden = false; };
  $("#railFilterClear").onclick = () => { CASE_FILTER = ""; loadSessions(); };
  document.querySelectorAll(".info-modal").forEach((m) => {
    m.querySelector(".info-backdrop").onclick = () => (m.hidden = true);
    m.querySelector(".info-close").onclick = () => (m.hidden = true);
  });
  document.addEventListener("keydown", (e) => { if (e.key === "Escape") document.querySelectorAll(".info-modal").forEach((m) => (m.hidden = true)); });
}
init();
