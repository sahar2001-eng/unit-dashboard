/* UNIT grid localization analytics -- front end.
   Every value shown comes from the robot's own flight code, run over the
   clip by flight_runner.py. This file only draws. Charts are hand-drawn SVG
   so the page works with no network access. */

const $ = (s) => document.querySelector(s);
const api = (u, o) => fetch(u, o).then((r) => r.json());

function refreshVersionTag() {
  fetch("/api/version", { cache: "no-store" })
    .then((r) => r.json())
    .then((v) => { $("#verTag").textContent =
      "engine " + v.engine_version + " · flight code " + (v.flight ? v.flight.combined : "?"); })
    .catch(() => {});
}

let SESSION = null;      // current session meta + series
let CHARTS = [];         // registered chart drawers
let POLL = null;
let CASES = null;        // the use-case list, from /api/cases
let CUR_FRAME = 0;       // frame under the playhead
let CASE_FILTER = "";    // only show runs tagged with this case id
const CASE_NAME = {};    // id -> title

/* ------------------------------------------------------------- sessions */
async function loadSessions(selectId) {
  const all = await api("/api/sessions");
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
       <span class="sub">${when.toLocaleDateString()} ${when.toLocaleTimeString(
        [], { hour: "2-digit", minute: "2-digit" })}</span>` +
      (m.case ? `<span class="case-tag">${m.case}</span>` : "") +
      (m.state === "done" && s
        ? `<span class="sub">${s.duration_s}s · ${s.drive_junction_pct}% junction` +
          `${s.rot_pct ? " · turn " + s.rot_pct + "%" : ""}</span>`
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
  $("#dlTerm").href = `/media/${id}/terminal.txt`;
  $("#dlMain").href = `/media/${id}/_main_as_run.py`;
  $("#notes").value = m.notes || "";
  drawCase(m);
  $("#rrStart").value = (m.turn && m.turn.start_frame != null) ? m.turn.start_frame : "";
  $("#rrEnd").value = (m.turn && m.turn.end_frame != null) ? m.turn.end_frame : "";
  $("#rrDir").value = (m.turn && m.turn.dir) || "L";

  drawHealth(s);
  drawEvents(s);
  buildCharts(m.series, s);
  readoutAt(0);
}

/* -------------------------------------------------------------- upload */
function startUpload(file) {
  const fd = new FormData();
  fd.append("video", file);
  fd.append("scale", $("#scale").value);
  fd.append("case", $("#caseSel").value);
  fd.append("turn_start", $("#turnStart").value.trim());
  fd.append("turn_dir", $("#turnDir").value);
  fd.append("turn_end", $("#turnEnd").value.trim());

  $("#uploadError").hidden = true;
  $("#progress").hidden = false;
  $("#barFill").style.width = "3%";
  $("#progressText").textContent = "Uploading " + file.name + "…";

  fetch("/api/upload", { method: "POST", body: fd })
    .then((r) => r.json())
    .then((r) => {
      if (r.error) throw new Error(r.error);
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

/* ------------------------------------------------------------ explainers */
const EXPLAIN = {
  group_position: ["Where the robot is",
    "X and Y exactly as the flight code reports them, read off the overlay " +
    "it drew on each frame. Blank where it reported \"--\": either the X null " +
    "zone across the middle of a cell, or no fix that frame."],
  x: ["X",
    "The robot's own X: 0 to -30 mm leaving a junction, null across the middle " +
    "of the cell, +30 to 0 arriving at the next. During a turn it comes from " +
    "Stage 2's tilted scan instead of the axis-aligned one, with the same " +
    "convention. Shaded bands are the frames the state machine was in TURN."],
  y: ["Y",
    "Offset across the bar, as reported. Positive/negative follow the image's " +
    "y axis (down is positive)."],
  group_heading: ["Which way it is pointing",
    "THETA is the tape tilt in this frame, -45 to +45 by definition (a square " +
    "grid looks the same every 90). Heading is the continuous angle the " +
    "robot keeps through a turn -- the number on its gauge."],
  theta: ["THETA",
    "The tilt of the tape in this frame, from the robot's line fit (driving) or " +
    "its tilted scan (turning). It wraps at +-45 by definition; the breaks are " +
    "wraps, not dropouts."],
  heading: ["Heading",
    "The value on the robot's gauge: the unwrapped, filtered angle it tracks " +
    "through a turn. Seeded from the tape tilt when the turn starts, so 90.0 " +
    "means square to the grid. Between turns it holds its last value."],
  lock: ["Fix per frame",
    "One bar per frame, in the robot's own tiers. Green: junction (both bars, " +
    "crossing found). Amber: Y only -- the vertical bar was not found. Orange: " +
    "X only. Purple: one bar during a turn. Red: nothing. Striped: turning."],
};

function openInfo(key) {
  const [title, body] = EXPLAIN[key] || ["", ""];
  if (!title) return;
  $("#infoTitle").textContent = title;
  $("#infoBody").textContent = body;
  $("#infoModal").hidden = false;
}
function infoBtn(key) {
  return `<button class="info-btn" data-info="${key}" aria-label="What is this?">i</button>`;
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

  const bands = (series.windows || []).filter((w) => w.mode === "ROT");
  const groups = [
    { name: "Where the robot is", key: "group_position",
      note: "X and Y as the robot reported them. Shaded: the state machine was in TURN.",
      specs: [
        { title: "X", key: "x", unit: "mm", h: 96, zero: true, bands,
          series: [{ d: series.x, cls: "trace", color: "var(--blue-vivid)" }] },
        { title: "Y", key: "y", unit: "mm", h: 72, zero: true, bands,
          series: [{ d: series.y, cls: "trace", color: "var(--mint)" }] },
      ] },
    { name: "Which way it is pointing", key: "group_heading",
      note: "THETA is this frame's tape tilt (wraps at 45). Heading is the robot's gauge.",
      specs: [
        { title: "THETA", key: "theta", unit: "degrees", h: 72, zero: true, gaps: true, bands,
          series: [{ d: series.theta, cls: "trace", color: "#7B8AB8" }] },
        { title: "Heading", key: "heading", unit: "degrees", h: 128, zero: true, bands,
          series: [{ d: series.heading, cls: "trace", color: "var(--blue-vivid)" }] },
      ] },
  ];

  groups.forEach((g) => {
    const sec = document.createElement("section");
    sec.className = "chart-group";
    sec.innerHTML = `<div class="group-head"><h3></h3><p></p></div>`;
    sec.querySelector("h3").innerHTML = g.name + infoBtn(g.key);
    sec.querySelector("p").textContent = g.note;
    g.specs.forEach((sp) => sec.appendChild(makeChart(sp, t, tmax, summary)));
    host.appendChild(sec);
  });

  const strip = document.createElement("section");
  strip.className = "chart-group";
  strip.appendChild(makeLockStrip(series, t, tmax));
  host.appendChild(strip);

  host.querySelectorAll("[data-info]").forEach((b) =>
    b.addEventListener("click", () => openInfo(b.dataset.info)));
}

function makeChart(sp, t, tmax, summary) {
  const wrap = document.createElement("div");
  wrap.className = "chart";
  wrap.innerHTML =
    `<div class="chart-head"><h4></h4><span class="unit">${sp.unit}</span></div>`;
  wrap.querySelector("h4").innerHTML = sp.title + (sp.key ? infoBtn(sp.key) : "");

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

  if (sp.bands && sp.bands.length) {
    sp.bands.forEach((w) => {
      const x1 = X(w.start), x2 = X(w.end);
      svg.appendChild(el("rect", { class: "turn-band", x: x1, y: PAD_T,
        width: Math.max(1, x2 - x1), height: H - PAD_T - PAD_B }));
      const lab = el("text", { class: "turn-label", x: (x1 + x2) / 2,
        y: PAD_T + 11, "text-anchor": "middle" });
      lab.textContent = "TURN";
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

  // ---- The line is drawn wherever a position exists, and breaks ONLY
  //      where there was genuinely no position that frame (nothing null-
  //      to-null is ever bridged, but a single missing frame between two
  //      real ones is). A sawtooth wrap (X snapping from ~0 back up to
  //      ~80 as the robot passes a junction) is NOT a break -- it's a real
  //      jump, drawn as one connected straight leap. Only the smoothing is
  //      suppressed across that leap so it stays a clean vertical line
  //      instead of arcing. Every plotted point is exactly the engine's
  //      value; this only controls what the drawn line connects.
  sp.series.forEach((s) => {
    // Build the point list, marking each point as either a smooth
    // continuation or the start of a straight jump (after a wrap).
    const pts = [];      // [x, y, jump]  jump=true means "reach this point
                         //               with a straight line, no curve"
    let lastIdx = null;
    s.d.forEach((v, i) => {
      if (v == null) return;                 // no position -> genuine break
      let jump = false;
      if (lastIdx != null && sp.gaps && Math.abs(v - s.d[lastIdx]) > 45) {
        jump = true;                         // sawtooth wrap: connect, but straight
      }
      pts.push([X(i), Y(v), jump]);
      lastIdx = i;
    });

    // Emit a path: straight L across jumps and across single-frame holes,
    // smooth C between ordinary neighbours. Never lift the pen except at
    // the very start.
    let d = "";
    for (let i = 0; i < pts.length; i++) {
      const [x, y, jump] = pts[i];
      if (i === 0) { d += `M${x.toFixed(1)} ${y.toFixed(1)} `; continue; }
      if (jump) { d += `L${x.toFixed(1)} ${y.toFixed(1)} `; continue; }
      // smooth toward this point from the previous one, but only if
      // neither end is a jump boundary (so curves never bleed across a
      // wrap)
      const prev = pts[i - 1];
      const next = pts[i + 1];
      const p0 = (i >= 2 && !prev[2]) ? pts[i - 2] : prev;
      const p1 = prev;
      const p2 = pts[i];
      const p3 = (next && !next[2]) ? next : p2;
      const c1x = p1[0] + (p2[0] - p0[0]) / 6;
      const c1y = p1[1] + (p2[1] - p0[1]) / 6;
      const c2x = p2[0] - (p3[0] - p1[0]) / 6;
      const c2y = p2[1] - (p3[1] - p1[1]) / 6;
      d += `C${c1x.toFixed(1)} ${c1y.toFixed(1)} `
         + `${c2x.toFixed(1)} ${c2y.toFixed(1)} `
         + `${p2[0].toFixed(1)} ${p2[1].toFixed(1)} `;
    }
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
    `<div class="chart-head"><h4>Fix per frame${infoBtn("lock")}</h4>
     <span class="unit">green: junction · amber: Y only · orange: X only · purple: one bar (turn) · red: none</span></div>`;
  const W = 1000, H = 22, PAD_L = 44, PAD_R = 10;
  const svg = el("svg", { viewBox: `0 0 ${W} ${H}`, preserveAspectRatio: "none" });
  svg.style.height = H + "px";
  const col = { 1: "#1C8F63", 2: "#D9B00A", 3: "#D9840A", 4: "#8A5BD6", 0: "#D0332E" };
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
  const sg = (v, d, u) => (v == null ? "—" : (v > 0 ? "+" : "") + v.toFixed(d) + u);
  const mode = s.mode[i];
  $("#rFrame").textContent = `${i}  (${s.t[i].toFixed(2)} s)`;
  CUR_FRAME = i;
  $("#rMode").textContent = mode === 1 ? "ROT" : mode === 0 ? "DRV" : "—";
  const tier = s.tier[i];
  const rt = $("#rTier");
  rt.textContent = { 1: "junction", 2: "Y only", 3: "X only", 4: "one bar", 0: "none" }[tier];
  rt.className = "lock-" + tier;
  $("#rX").textContent = sg(s.x[i], 1, " mm");
  $("#rY").textContent = sg(s.y[i], 1, " mm");
  $("#rTheta").textContent = sg(s.theta[i], 1, "°");
  $("#rHeading").textContent = sg(s.heading[i], 1, "°");
  CHARTS.forEach((c) => c.setHead(s.t[i]));
}

/* --------------------------------------------------------------- cards */
function drawHealth(s) {
  const turnTxt = s.turn
    ? `TURN ${s.turn.dir} at frame ${s.turn.start_frame}` +
      (s.turn.end_frame != null ? `, DRIVE at ${s.turn.end_frame}` : ", to end")
    : "none (driving only)";
  const rows = [
    ["Frames", `${s.frames} · ${s.fps} fps · ${s.duration_s}s`],
    ["Turn command", turnTxt],
    ["Driving / turning", `${s.drv_pct}% / ${s.rot_pct}%`],
    ["X reported", s.x_available_pct + "% of frames"],
    ["Y reported", s.y_available_pct + "% of frames"],
    ["THETA reported", s.theta_available_pct + "% of frames"],
    ["Heading start → end", s.heading_start == null ? "—" :
      `${s.heading_start.toFixed(1)}° → ${s.heading_end.toFixed(1)}°`],
    ["Heading range", s.heading_min == null ? "—" :
      `${s.heading_min.toFixed(1)}° to ${s.heading_max.toFixed(1)}°`],
    ["Analysed in", `${s.processing_s}s (${s.ms_per_frame} ms/frame on this PC)`],
    ["Flight code", s.flight ? s.flight.combined : "—"],
  ];
  $("#health").innerHTML = rows.map(
    (r) => `<div><span class="k">${r[0]}</span><span class="v">${r[1]}</span></div>`
  ).join("");

  const bar = (rows) => rows.map((r) => `
    <div class="cov-row tier-row">
      <span class="muted">${r[0]}</span>
      <span class="cov-bar"><i style="width:${r[1]}%;background:${r[2]}"></i></span>
      <span class="pct">${r[1]}%</span>
    </div>`).join("");
  $("#tiersDrv").innerHTML = s.drv_pct ? bar([
    ["junction — X, Y, THETA", s.drive_junction_pct, "#1C8F63"],
    ["Y only — vertical bar not found", s.drive_y_only_pct, "#D9B00A"],
    ["X only — horizontal bar not found", s.drive_x_only_pct, "#D9840A"],
    ["none", s.drive_none_pct, "#D0332E"],
  ]) : `<p class="muted small">No driving frames.</p>`;
  $("#tiersRot").innerHTML = s.rot_pct ? bar([
    ["junction — X, Y, THETA from Stage 2", s.rot_junction_pct, "#1C8F63"],
    ["one bar", s.rot_one_bar_pct, "#8A5BD6"],
    ["none", s.rot_none_pct, "#D0332E"],
  ]) : `<p class="muted small">No turn in this clip.</p>`;
}

function drawEvents(s) {
  const ev = s.events || [];
  $("#events").textContent = ev.length ? ev.join("\n") : "(no events)";
}

function drawCase(m) {
  const p = $("#runCase");
  if (m.case) {
    p.innerHTML = `<span class="case-tag">${m.case}</span> ${CASE_NAME[m.case] || ""}`;
  } else {
    p.innerHTML = `<span class="muted">no use case tagged</span>`;
  }
  const sel = document.createElement("select");
  sel.className = "case-inline";
  sel.innerHTML = `<option value="">— tag a use case —</option>` +
    (CASES ? CASES.sections.map((sec) =>
      `<optgroup label="${sec.id}. ${sec.name}">` +
      sec.cases.map((c) => `<option value="${c.id}" ${c.id === m.case ? "selected" : ""}>${c.id} ${c.title}</option>`).join("") +
      `</optgroup>`).join("") : "");
  sel.onchange = () => {
    fetch(`/api/session/${m.id}/case`, {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ case: sel.value }),
    }).then(() => { m.case = sel.value; drawCase(m); loadSessions(); });
  };
  p.appendChild(sel);
}

/* --------------------------------------------------------------- cases */
async function loadCases() {
  CASES = await api("/api/cases");
  const sel = $("#caseSel");
  sel.innerHTML = `<option value="">— none —</option>`;
  CASES.sections.forEach((sec) => {
    const og = document.createElement("optgroup");
    og.label = `${sec.id}. ${sec.name}`;
    sec.cases.forEach((c) => {
      CASE_NAME[c.id] = c.title;
      const o = document.createElement("option");
      o.value = c.id; o.textContent = `${c.id} ${c.title}`;
      og.appendChild(o);
    });
    sel.appendChild(og);
  });
}

async function openCases() {
  CASES = await api("/api/cases");
  $("#casesTitle").textContent = CASES.title || "Use cases";
  $("#casesList").innerHTML = CASES.sections.map((sec) => `
    <div class="case-sec">
      <h4>${sec.id}. ${sec.name}</h4>
      ${sec.cases.map((c) => `
        <button class="case-row ${c.id === CASE_FILTER ? "on" : ""}" data-case="${c.id}">
          <span class="case-id">${c.id}</span>
          <span class="case-title">${c.title}</span>
          <span class="case-n ${c.runs ? "" : "zero"}">${c.runs ? c.runs + " run" + (c.runs > 1 ? "s" : "") : "no runs"}</span>
        </button>`).join("")}
    </div>`).join("");
  $("#casesList").querySelectorAll(".case-row").forEach((b) => b.onclick = () => {
    CASE_FILTER = b.dataset.case === CASE_FILTER ? "" : b.dataset.case;
    $("#casesModal").hidden = true;
    loadSessions();
  });
  $("#casesModal").hidden = false;
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
  }).then(() => { SESSION.name = name; loadSessions(); });
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
    loadSessions();
  });
};

$("#infoClose").onclick = () => { $("#infoModal").hidden = true; };
$("#casesBtn").onclick = openCases;
$("#rrHere").onclick = () => { $("#rrStart").value = CUR_FRAME; };
$("#rrGo").onclick = () => {
  if (!SESSION) return;
  const body = { turn_start: $("#rrStart").value.trim(), turn_dir: $("#rrDir").value,
                 turn_end: $("#rrEnd").value.trim() };
  if (!body.turn_start) { alert("Enter the frame the TURN command is sent on."); return; }
  fetch(`/api/session/${SESSION.id}/rerun`, {
    method: "POST", headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  }).then((r) => r.json()).then((r) => {
    if (r.error) { alert(r.error); return; }
    loadSessions();
    watch(r.id);
  });
};
$("#casesClose").onclick = () => { $("#casesModal").hidden = true; };
$("#casesModal .info-backdrop").onclick = () => { $("#casesModal").hidden = true; };
$("#railFilterClear").onclick = () => { CASE_FILTER = ""; loadSessions(); };
$(".info-backdrop") && ($(".info-backdrop").onclick = () => { $("#infoModal").hidden = true; });
document.addEventListener("keydown", (e) => {
  if (e.key === "Escape" && !$("#infoModal").hidden) { $("#infoModal").hidden = true; return; }
  if (e.key === "Escape" && !$("#casesModal").hidden) { $("#casesModal").hidden = true; return; }
  if (e.target.isContentEditable || e.target.tagName === "TEXTAREA") return;
  const v = $("#video");
  if (e.key === " ") { e.preventDefault(); v.paused ? v.play() : v.pause(); }
  if (e.key === "ArrowRight" && SESSION) v.currentTime += 1 / (SESSION.summary.fps || 30);
  if (e.key === "ArrowLeft" && SESSION) v.currentTime -= 1 / (SESSION.summary.fps || 30);
});

refreshVersionTag();
loadCases().then(() => loadSessions());
