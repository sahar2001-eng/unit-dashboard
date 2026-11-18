/* Runs -- front end. Shows runs made on a PC (runkit/run.py) with the
   camera's own code. Nothing is computed here except simple counts.
   Row i of frames.csv is picture i of video.mp4. Same look as t2.js. */

const $ = (s) => document.querySelector(s);
const api = (u, o) => fetch(u, o).then((r) => r.json());
const jpost = (u, body) => api(u, { method: "POST", headers: { "Content-Type": "application/json" },
  body: JSON.stringify(body) });
const esc = (s) => String(s == null ? "" : s).replace(/[&<>"']/g,
  (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));

let RUN = null, CMP = null, CHARTS = [], ALL = [], TILE = 0, CASE_FILTER = "", CUR = 0;
const CASES = { 1: null, 2: null };
let CASE_TILE = 1;

// what the camera found, per frame (frames.csv "tier")
const TIER = {
  junction: ["junction (X and Y)", "#1C8F63"],
  y: ["road only (Y)", "#3355E0"],
  x: ["cross only (X)", "#D9840A"],
  none: ["nothing", "#D0332E"],
};
const tierOf = (v) => TIER[v] || [v || "nothing", v ? "#8A5BD6" : "#D0332E"];
const MODE_COL = { DRIVE: "#CBD3E8", TURN_INIT: "#D9840A", TURNING: "#3355E0",
                   REACQUIRE: "#8A5BD6", FAULT: "#D0332E" };

const tokenGet = () => { try { return localStorage.getItem("runsToken") || ""; } catch (_) { return ""; } };
const tokenSet = (v) => { try { localStorage.setItem("runsToken", v); } catch (_) { /* fine */ } };

/* ------------------------------------------------------------- list */
async function loadRuns(selectId) {
  const r = await api("/api/runs");
  ALL = r.runs;
  $("#uploadOff").hidden = r.upload_on;
  let list = TILE ? ALL.filter((m) => m.tile === TILE) : ALL;
  if (CASE_FILTER) list = list.filter((m) => m.case === CASE_FILTER);
  const ul = $("#sessions");
  ul.innerHTML = "";
  $("#runCount").textContent = list.length || "";
  $("#railEmpty").hidden = list.length > 0;
  $("#railFilter").hidden = !CASE_FILTER;
  if (CASE_FILTER) $("#railFilterText").textContent = "case " + CASE_FILTER + " · " + list.length;
  list.forEach((m) => {
    const li = document.createElement("li");
    li.dataset.id = m.id;
    if (RUN && RUN.id === m.id) li.classList.add("on");
    const when = new Date((m.recorded || m.created) * 1000);
    const src = m.source === "robot" ? "robot drive" : m.source === "video" ? "video" : m.source;
    li.innerHTML = `<span class="nm"></span>
      <span class="sub">Tile ${m.tile || "?"} · ${esc(src)} · ${m.rows} frames</span>
      <span class="sub">${when.toLocaleDateString()} ${when.toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" })}</span>` +
      (m.case ? `<span class="case-tag">${esc(m.case)}</span>` : "");
    li.querySelector(".nm").textContent = m.name;
    li.onclick = () => openRun(m.id);
    ul.appendChild(li);
  });
  if (selectId) openRun(selectId);
}

/* ------------------------------------------------------------- one run */
function prep(m) {
  // time axis in seconds from the first row; row index when there is no t_s
  const c = m.cols, n = c.frame.length;
  const fps = (m.info && (m.info.video_fps || m.info.fps)) || 30;
  let t = c.t_s;
  if (!t || t.every((v) => v == null)) t = c.frame.map((_, i) => i / fps);
  const t0 = t.find((v) => v != null) || 0;
  m.t = t.map((v, i) => (v == null ? i / fps : v - t0));
  m.n = n;
  m.vfps = (m.info && m.info.video_fps) || fps;
  m.tmax = m.t[n - 1] || 1;
  return m;
}

async function openRun(id) {
  const m = await api("/api/runs/" + id);
  if (m.error) return;
  RUN = prep(m);
  CMP = null;
  document.querySelectorAll(".sessions li").forEach((li) => li.classList.toggle("on", li.dataset.id === id));
  $("#uploadPane").hidden = true;
  $("#resultPane").hidden = false;
  $("#runName").textContent = m.name;
  const i = m.info || {};
  $("#runMeta").textContent = `Tile ${m.tile || "?"} · ${m.source === "robot" ? "robot drive" : "from a video"} · ` +
    `${m.n} frames · ${m.tmax.toFixed(1)} s` + (i.app_version ? ` · gridcam ${i.app_version}` : "");
  const v = $("#video");
  v.src = `/media3/${id}/video.mp4`;
  v.load();
  $("#cmpBox").hidden = true;
  $("#video2").removeAttribute("src");
  $("#dlVideo").href = `/media3/${id}/video.mp4`;
  $("#dlCsv").href = `/media3/${id}/frames.csv`;
  $("#dlInfo").href = `/media3/${id}/info.json`;
  $("#dlLog").hidden = !m.has_log;
  $("#dlLog").href = `/media3/${id}/log.txt`;
  $("#notes").value = m.notes || "";
  await drawCase(m);
  fillCmpSel();
  drawCards(m);
  buildCharts();
  showRow(0, false);
}

/* ------------------------------------------------------------- compare */
function fillCmpSel() {
  const sel = $("#cmpSel");
  sel.innerHTML = `<option value="">— nothing —</option>` + ALL.filter((m) => m.id !== RUN.id &&
    (!RUN.tile || m.tile === RUN.tile)).map((m) => `<option value="${esc(m.id)}">${esc(m.name)}</option>`).join("");
}

async function setCompare(id) {
  if (!id) {
    CMP = null;
    $("#cmpBox").hidden = true;
    $("#video2").removeAttribute("src");
  } else {
    const m = await api("/api/runs/" + id);
    if (m.error) return;
    CMP = prep(m);
    $("#cmpName").textContent = m.name;
    $("#cmpBox").hidden = false;
    const v2 = $("#video2");
    v2.src = `/media3/${id}/video.mp4`;
    v2.load();
  }
  buildCharts();
  showRow(CUR, true);
}

/* ------------------------------------------------------------- explainers */
const EXPLAIN = {
  x: ["X", "Distance along the road to the junction, in mm, as the camera reported it. Empty = not reported."],
  y: ["Y", "Offset across the road, in mm, as the camera reported it."],
  deg: ["Angle", "The tape angle the camera reported, in degrees."],
  head: ["Turn angle", "How far the robot has turned, as the camera tracks it during a turn (the 0xA8 report). Empty while driving."],
  q: ["Quality", "The camera's frame quality score, 0 to 100 (higher is better). Empty when the score was off."],
  tier: ["What was found", "Green: junction (X and Y). Blue: road only (Y). Amber: cross only (X). Red: nothing."],
  mode: ["Mode", "Grey: DRIVE. Amber: turn starting. Blue: turning. Purple: finding the lines again after a turn. Red: fault."],
  codes: ["Codes", "Each tick is a frame where the camera read a junction code (Tile 2)."],
  cmp: ["Compare", "The other run is drawn as a grey dashed line, on the same time from its start."],
};
const infoBtn = (k) => `<button class="info-btn" data-info="${k}" aria-label="What is this?">i</button>`;
function openInfo(k) {
  const [t, b] = EXPLAIN[k] || ["", ""];
  if (!t) return;
  $("#infoTitle").textContent = t; $("#infoBody").textContent = b;
  $("#infoModal").hidden = false;
}

/* ------------------------------------------------------------- charts */
const NS = "http://www.w3.org/2000/svg";
const el = (n, a = {}) => { const e = document.createElementNS(NS, n);
  for (const k in a) e.setAttribute(k, a[k]); return e; };
const GEO = { W: 1000, L: 44, R: 10 };

function niceTicks(lo, hi, n = 4) {
  const raw = (hi - lo || 1) / n;
  const mag = Math.pow(10, Math.floor(Math.log10(raw)));
  const step = [1, 2, 2.5, 5, 10].map((m) => m * mag).find((s) => s >= raw) || 10 * mag;
  const out = [];
  for (let v = Math.ceil(lo / step) * step; v <= hi + 1e-9; v += step) out.push(v);
  return out;
}

function tspan() { return Math.max(RUN.tmax, CMP ? CMP.tmax : 0) || 1; }
const XT = (t) => GEO.L + ((GEO.W - GEO.L - GEO.R) * t) / tspan();

function rowAt(m, t) {         // the row whose time is nearest t
  let lo = 0, hi = m.n - 1;
  while (lo < hi) { const mid = (lo + hi) >> 1; if (m.t[mid] < t) lo = mid + 1; else hi = mid; }
  if (lo > 0 && Math.abs(m.t[lo - 1] - t) <= Math.abs(m.t[lo] - t)) lo--;
  return lo;
}

function seekHandlers(svg) {
  const seek = (ev) => {
    const r = svg.getBoundingClientRect();
    const px = ((ev.clientX - r.left) / r.width) * GEO.W;
    const t = Math.max(0, Math.min(tspan(), ((px - GEO.L) / (GEO.W - GEO.L - GEO.R)) * tspan()));
    showRow(rowAt(RUN, t), true);
  };
  svg.addEventListener("pointerdown", (e) => { try { svg.setPointerCapture(e.pointerId); } catch (_) { /* fine */ } seek(e); });
  svg.addEventListener("pointermove", (e) => { if (e.buttons) seek(e); });
}

function addHead(svg, y1, y2) {
  const head = el("line", { class: "playhead", x1: GEO.L, x2: GEO.L, y1, y2, opacity: 0 });
  svg.appendChild(head);
  CHARTS.push((t) => { const x = XT(t); head.setAttribute("x1", x); head.setAttribute("x2", x); head.setAttribute("opacity", 1); });
}

function chartWrap(title, key, unit, H) {
  const wrap = document.createElement("div");
  wrap.className = "chart";
  wrap.innerHTML = `<div class="chart-head"><h4>${title}${infoBtn(key)}</h4><span class="unit">${unit}</span></div>`;
  const svg = el("svg", { viewBox: `0 0 ${GEO.W} ${H}`, preserveAspectRatio: "none" });
  svg.style.height = H + "px";
  wrap.appendChild(svg);
  return [wrap, svg];
}

function pathOf(m, d, Y) {
  let s = "", last = -9;
  d.forEach((v, i) => {
    if (v == null) return;
    s += (i - last > 2 ? "M" : "L") + XT(m.t[i]).toFixed(1) + " " + Y(v).toFixed(1) + " ";
    last = i;
  });
  return s;
}

function lineChart(title, key, unit, col, color, H, fixed) {
  const d = RUN.cols[col];
  if (!d || d.every((v) => v == null)) return null;
  const d2 = CMP && CMP.cols[col];
  const [wrap, svg] = chartWrap(title, key, unit + (d2 ? " · grey dashed: compared run" : ""), H);
  const T = 8, B = 16;
  let lo = Infinity, hi = -Infinity;
  [d, d2 || []].forEach((a) => a.forEach((v) => { if (v != null) { lo = Math.min(lo, v); hi = Math.max(hi, v); } }));
  if (fixed) { lo = fixed[0]; hi = fixed[1]; }
  if (!isFinite(lo)) { lo = 0; hi = 1; }
  if (hi - lo < 1e-6) { hi += 0.5; lo -= 0.5; }
  if (!fixed) { const pad = (hi - lo) * 0.12; lo -= pad; hi += pad; }
  const Y = (v) => T + (H - T - B) * (1 - (v - lo) / (hi - lo));
  niceTicks(lo, hi).forEach((v) => {
    svg.appendChild(el("line", { class: Math.abs(v) < 1e-9 ? "zero-line" : "grid-line", x1: GEO.L, x2: GEO.W - GEO.R, y1: Y(v), y2: Y(v) }));
    const tx = el("text", { class: "axis-text", x: GEO.L - 6, y: Y(v) + 3, "text-anchor": "end" });
    tx.textContent = Math.abs(v) >= 100 ? v.toFixed(0) : v.toFixed(1);
    svg.appendChild(tx);
  });
  if (d2) svg.appendChild(el("path", { class: "trace cmp", d: pathOf(CMP, d2, Y) }));
  svg.appendChild(el("path", { class: "trace", d: pathOf(RUN, d, Y), stroke: color }));
  d.forEach((v, i) => { if (v != null && d[i - 1] == null && d[i + 1] == null)
    svg.appendChild(el("circle", { cx: XT(RUN.t[i]), cy: Y(v), r: 1.8, fill: color })); });
  addHead(svg, T, H - B);
  seekHandlers(svg);
  return wrap;
}

function strip(title, key, unit, vals, colorOf, H = 22) {
  const [wrap, svg] = chartWrap(title, key, unit, H);
  let i = 0;
  while (i < vals.length) {
    let j = i;
    while (j + 1 < vals.length && vals[j + 1] === vals[i]) j++;
    const x1 = XT(RUN.t[i]), x2 = XT(RUN.t[j]);
    svg.appendChild(el("rect", { x: x1, y: 4, width: Math.max(0.8, x2 - x1 + 0.8), height: H - 10, fill: colorOf(vals[i]), rx: 1 }));
    i = j + 1;
  }
  addHead(svg, 0, H);
  seekHandlers(svg);
  return wrap;
}

function codeStrip() {
  const code = RUN.cols.code;
  if (!code || code.every((c) => !c)) return null;
  const [wrap, svg] = chartWrap("Codes read", "codes", "Tile 2 junction codes", 34);
  let lastLab = null, lastX = -999;
  code.forEach((c, i) => {
    if (!c) return;
    const x = XT(RUN.t[i]);
    svg.appendChild(el("line", { x1: x, x2: x, y1: 18, y2: 30, stroke: "#16215C", "stroke-width": 1.2 }));
    if (c !== lastLab || x - lastX > 60) {
      const tx = el("text", { x, y: 13, "text-anchor": "middle", class: "turn-label" });
      tx.textContent = c; svg.appendChild(tx);
      lastLab = c; lastX = x;
    }
  });
  addHead(svg, 0, 34);
  seekHandlers(svg);
  return wrap;
}

function group(host, title, sub, items) {
  items = items.filter(Boolean);
  if (!items.length) return;
  const g = document.createElement("section");
  g.className = "chart-group";
  g.innerHTML = `<div class="group-head"><h3>${title}</h3><p>${sub}</p></div>`;
  items.forEach((x) => g.appendChild(x));
  host.appendChild(g);
}

function buildCharts() {
  const host = $("#charts");
  host.innerHTML = "";
  CHARTS = [];
  const c = RUN.cols;
  group(host, "Where the robot is", "As the camera reported it. Empty = not reported that frame. Click any chart to jump there.", [
    lineChart("X", "x", "mm", "x_mm", "#3355E0", 96),
    lineChart("Y", "y", "mm", "y_mm", "#1C8F63", 72),
    lineChart("Angle", "deg", "degrees", "theta_deg", "#16215C", 88),
    lineChart("Turn angle", "head", "degrees turned (only while turning)", "heading", "#8A5BD6", 88),
  ]);
  const tiers = (c.tier || []).map((v) => v || "none");
  const modes = c.mode && c.mode.some((v) => v) ? c.mode.map((v) => v || "DRIVE") : null;
  group(host, "How much to trust it", "What the camera found in each frame, its quality score, and the codes it read.", [
    c.tier ? strip("What was found", "tier",
      ["junction", "y", "x", "none"].map((k) => `<i class="dot" style="background:${TIER[k][1]}"></i>${TIER[k][0]}`).join(" · "),
      tiers, (v) => tierOf(v === "none" ? null : v)[1]) : null,
    lineChart("Quality", "q", "0–100", "q", "#D9840A", 72, [0, 100]),
    codeStrip(),
    modes ? strip("Mode", "mode", "grey drive · blue turning · purple finding lines · red fault", modes,
      (v) => MODE_COL[v] || "#AEB9DC") : null,
  ]);
  host.querySelectorAll("[data-info]").forEach((b) => b.addEventListener("click", () => openInfo(b.dataset.info)));
}

/* ------------------------------------------------------------- readout */
function showRow(i, seekVideo) {
  if (!RUN) return;
  i = Math.max(0, Math.min(RUN.n - 1, i));
  CUR = i;
  const c = RUN.cols, sg = (v, d, u) => (v == null ? "—" : (v > 0 ? "+" : "") + v.toFixed(d) + u);
  $("#rFrame").textContent = `${c.frame[i]}  (${RUN.t[i].toFixed(2)} s)`;
  const tr = tierOf(c.tier ? c.tier[i] : null);
  $("#rTier").textContent = tr[0] + (c.suspect && c.suspect[i] ? " !" : "");
  $("#rTier").style.color = tr[1];
  $("#rX").textContent = sg(c.x_mm && c.x_mm[i], 1, " mm");
  $("#rY").textContent = sg(c.y_mm && c.y_mm[i], 1, " mm");
  $("#rDeg").textContent = sg(c.theta_deg && c.theta_deg[i], 1, "°");
  $("#rCode").textContent = (c.code && c.code[i]) || "—";
  const q = c.q && c.q[i];
  $("#rQ").textContent = q == null ? "—" : q.toFixed(0) + ((c.why && c.why[i]) ? "  " + c.why[i].replace(/_/g, " ") : "");
  $("#rMode").textContent = ((c.mode && c.mode[i]) || "—") +
    (c.heading && c.heading[i] != null ? "  " + c.heading[i].toFixed(1) + "°" : "");
  CHARTS.forEach((f) => f(RUN.t[i]));
  if (seekVideo) {
    const v = $("#video");
    if (v.readyState) v.currentTime = (i + 0.5) / RUN.vfps;
  }
  syncCmp();
}

function syncCmp() {
  if (!CMP) return;
  const v2 = $("#video2");
  if (!v2.readyState) return;
  const j = rowAt(CMP, RUN.t[CUR]);
  const want = (j + 0.5) / CMP.vfps;
  if (Math.abs(v2.currentTime - want) > 0.6 / CMP.vfps) v2.currentTime = want;
}

/* ------------------------------------------------------------- cards */
function kv(host, rows) {
  host.innerHTML = rows.map(([k, v]) => `<div><span class="k">${esc(k)}</span><span class="v">${esc(v)}</span></div>`).join("");
}
const pct = (a, n) => (n ? Math.round((1000 * a) / n) / 10 : 0);

function drawCards(m) {
  const c = m.cols, n = m.n;
  const cnt = { junction: 0, y: 0, x: 0, none: 0 }, other = {};
  (c.tier || []).forEach((v) => { if (!v) cnt.none++; else if (v in cnt) cnt[v]++; else other[v] = (other[v] || 0) + 1; });
  const rows = Object.entries(cnt).concat(Object.entries(other));
  $("#tierBars").innerHTML = c.tier ? rows.map(([k, a]) => { const [lab, col] = tierOf(k === "none" ? null : k);
    return `<div class="cov-row tier-row"><span>${esc(lab)}</span><span class="cov-bar"><i style="width:${pct(a, n)}%;background:${col}"></i></span><span class="pct">${pct(a, n)}%</span></div>`; }).join("")
    : `<p class="muted small">No "tier" column in this run.</p>`;
  // codes: first frame and how many frames each code was read on
  const codes = {};
  (c.code || []).forEach((v, i) => { if (!v) return; if (!codes[v]) codes[v] = { first: i, n: 0 }; codes[v].n++; });
  const tb = $("#codesBody");
  tb.innerHTML = "";
  const list = Object.entries(codes).sort((a, b) => a[1].first - b[1].first);
  $("#noCodes").hidden = list.length > 0;
  list.forEach(([name, o]) => {
    const tr = document.createElement("tr");
    tr.innerHTML = `<td class="deg">${esc(name)}</td><td>frame ${c.frame[o.first]} · ${m.t[o.first].toFixed(2)} s</td><td>${o.n}</td>`;
    tr.onclick = () => showRow(o.first, true);
    tb.appendChild(tr);
  });
  const has = (a) => (a ? a.filter((v) => v != null).length : 0);
  const qs = (c.q || []).filter((v) => v != null);
  const qavg = qs.length ? Math.round(qs.reduce((a, b) => a + b, 0) / qs.length) : null;
  const i = m.info || {};
  kv($("#health"), [
    ["Frames", `${n} · ${m.tmax.toFixed(1)} s`],
    ["Pictures per second", i.video_fps ? String(i.video_fps) : "—"],
    ["X reported", pct(has(c.x_mm), n) + "% of frames"],
    ["Y reported", pct(has(c.y_mm), n) + "% of frames"],
    ["Angle reported", pct(has(c.theta_deg), n) + "% of frames"],
    ["Quality average", qavg == null ? "off" : String(qavg)],
  ]);
  kv($("#made"), [
    ["Source", m.source === "robot" ? "robot drive (camera stream)" : "video: " + (i.source_file || "?")],
    ["Tile", String(m.tile || "?")],
    ["Camera app", i.app_version ? "gridcam " + i.app_version : "—"],
    ["Code version", i.app_commit || "—"],
    ["Run kit", i.kit_version || "—"],
    ["Recorded", i.created ? new Date(i.created * 1000).toLocaleString() : "—"],
  ]);
}

/* ------------------------------------------------------------- use cases */
async function getCases(tile) {
  CASES[tile] = await api("/api/runs/cases/" + tile);
  return CASES[tile];
}

async function drawCase(m) {
  const host = $("#runCase");
  host.innerHTML = "";
  if (m.tile !== 1 && m.tile !== 2) return;
  const cs = await getCases(m.tile);
  const sel = document.createElement("select");
  sel.className = "case-inline";
  sel.innerHTML = `<option value="">Use case (Tile ${m.tile}): none</option>` + cs.sections.map((sec) =>
    `<optgroup label="${esc(sec.id + ". " + sec.name)}">` + sec.cases.map((c) =>
      `<option value="${esc(c.id)}">${esc(c.id)}  ${esc(c.title)}</option>`).join("") + `</optgroup>`).join("");
  sel.value = m.case || "";
  sel.onchange = async () => {
    const r = await jpost(`/api/runs/${m.id}/case`, { case: sel.value });
    if (r.error) { alert(r.error); sel.value = m.case || ""; return; }
    m.case = sel.value;
    loadRuns();
  };
  host.appendChild(sel);
}

async function drawCasesModal() {
  const cs = await getCases(CASE_TILE);
  document.querySelectorAll("[data-ctile]").forEach((b) => b.classList.toggle("on", +b.dataset.ctile === CASE_TILE));
  $("#casesTitle").textContent = cs.title || "Use cases";
  const host = $("#casesList");
  host.innerHTML = "";
  cs.sections.forEach((sec) => {
    const d = document.createElement("div");
    d.className = "case-sec";
    d.innerHTML = `<h4>${esc(sec.id)}. ${esc(sec.name)}</h4>`;
    sec.cases.forEach((c) => {
      const b = document.createElement("button");
      b.className = "case-row" + (CASE_FILTER === c.id ? " on" : "");
      b.innerHTML = `<span class="case-id">${esc(c.id)}</span><span>${esc(c.title)}</span>
        <span class="case-n ${c.runs ? "" : "zero"}">${c.runs} run${c.runs === 1 ? "" : "s"}</span>`;
      b.onclick = () => { CASE_FILTER = c.id; setTile(CASE_TILE); $("#casesModal").hidden = true; };
      d.appendChild(b);
    });
    host.appendChild(d);
  });
}

function setTile(t) {
  TILE = t;
  document.querySelectorAll("[data-tile]").forEach((b) => b.classList.toggle("on", +b.dataset.tile === t));
  loadRuns();
}

/* ------------------------------------------------------------- upload */
function upload(file) {
  const tok = $("#token").value.trim();
  if (!tok) { showErr("Enter the upload password first."); return; }
  tokenSet(tok);
  const fd = new FormData();
  fd.append("run", file);
  $("#uploadError").hidden = true;
  $("#progress").hidden = false;
  $("#barFill").style.width = "2%";
  const x = new XMLHttpRequest();
  x.open("POST", "/api/runs/upload");
  x.setRequestHeader("X-Runs-Token", tok);
  x.upload.onprogress = (e) => {
    if (!e.lengthComputable) return;
    const p = (100 * e.loaded) / e.total;
    $("#barFill").style.width = p + "%";
    $("#progressText").textContent = p < 100 ? `Uploading ${file.name}… ${Math.round(p)}%` : "Checking the run…";
  };
  x.onload = () => {
    $("#progress").hidden = true;
    let r = {};
    try { r = JSON.parse(x.responseText); } catch (_) { r = { error: "server error " + x.status }; }
    if (r.error) { showErr(r.error); return; }
    loadRuns(r.id);
  };
  x.onerror = () => { $("#progress").hidden = true; showErr("Upload failed (network)."); };
  x.send(fd);
}
function showErr(t) { $("#uploadError").hidden = false; $("#uploadError").textContent = t; }

/* ------------------------------------------------------------- wiring */
function init() {
  $("#token").value = tokenGet();
  loadRuns(new URLSearchParams(location.search).get("run") || undefined);
  $("#newRun").onclick = () => {
    RUN = null; CMP = null;
    document.querySelectorAll(".sessions li").forEach((li) => li.classList.remove("on"));
    $("#uploadPane").hidden = false; $("#resultPane").hidden = true;
    $("#progress").hidden = true; $("#uploadError").hidden = true;
  };
  document.querySelectorAll("[data-tile]").forEach((b) => (b.onclick = () => setTile(+b.dataset.tile)));
  document.querySelectorAll("[data-ctile]").forEach((b) => (b.onclick = () => { CASE_TILE = +b.dataset.ctile; drawCasesModal(); }));
  $("#railFilterClear").onclick = () => { CASE_FILTER = ""; loadRuns(); };
  $("#casesBtn").onclick = async () => { await drawCasesModal(); $("#casesModal").hidden = false; };
  $("#pickBtn").onclick = () => $("#file").click();
  $("#file").onchange = (e) => { if (e.target.files[0]) upload(e.target.files[0]); e.target.value = ""; };
  const drop = $("#drop");
  ["dragenter", "dragover"].forEach((ev) => drop.addEventListener(ev, (e) => { e.preventDefault(); drop.classList.add("hot"); }));
  ["dragleave", "drop"].forEach((ev) => drop.addEventListener(ev, (e) => { e.preventDefault(); drop.classList.remove("hot"); }));
  drop.addEventListener("drop", (e) => { const f = e.dataTransfer.files[0]; if (f) upload(f); });

  const v = $("#video");
  v.addEventListener("timeupdate", () => { if (RUN && !v.seeking) showRow(Math.floor(v.currentTime * RUN.vfps), false); });
  v.addEventListener("seeked", () => { if (RUN) showRow(Math.floor(v.currentTime * RUN.vfps), false); });
  v.addEventListener("play", () => { if (CMP) $("#video2").play().catch(() => {}); });
  v.addEventListener("pause", () => { if (CMP) { $("#video2").pause(); syncCmp(); } });
  v.addEventListener("loadeddata", () => { if (v.currentTime === 0) v.currentTime = 0.5 / (RUN ? RUN.vfps : 30); });
  $("#prevF").onclick = () => { v.pause(); showRow(CUR - 1, true); };
  $("#nextF").onclick = () => { v.pause(); showRow(CUR + 1, true); };
  document.addEventListener("keydown", (e) => {
    if (e.key === "Escape") document.querySelectorAll(".info-modal").forEach((m) => (m.hidden = true));
    if (!RUN || $("#resultPane").hidden) return;
    const tag = (e.target.tagName || "").toLowerCase();
    if (tag === "input" || tag === "textarea" || e.target.isContentEditable) return;
    if (e.key === "ArrowLeft") { e.preventDefault(); v.pause(); showRow(CUR - 1, true); }
    if (e.key === "ArrowRight") { e.preventDefault(); v.pause(); showRow(CUR + 1, true); }
  });
  $("#cmpSel").onchange = (e) => setCompare(e.target.value);

  $("#runName").addEventListener("blur", () => {
    if (!RUN) return;
    jpost(`/api/runs/${RUN.id}/rename`, { name: $("#runName").textContent }).then(() => loadRuns());
  });
  $("#runName").addEventListener("keydown", (e) => { if (e.key === "Enter") { e.preventDefault(); e.target.blur(); } });
  let nt = null;
  $("#notes").addEventListener("input", () => {
    clearTimeout(nt);
    nt = setTimeout(() => RUN && jpost(`/api/runs/${RUN.id}/notes`, { notes: $("#notes").value }), 600);
  });
  $("#delRun").onclick = async () => {
    if (!RUN) return;
    let tok = tokenGet();
    if (!tok) tok = prompt("Upload password (needed to delete):") || "";
    if (!tok || !confirm("Delete this run and its files?")) return;
    const r = await api("/api/runs/" + RUN.id, { method: "DELETE", headers: { "X-Runs-Token": tok } });
    if (r.error) { alert(r.error); return; }
    tokenSet(tok);
    $("#newRun").click();
    loadRuns();
  };
  document.querySelectorAll(".info-modal").forEach((m) => {
    m.querySelector(".info-backdrop").onclick = () => (m.hidden = true);
    m.querySelector(".info-close").onclick = () => (m.hidden = true);
  });
}
init();
