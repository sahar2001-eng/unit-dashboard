/* Updates diary -- front end. Entries live on the server (data/_updates). */

const $ = (s) => document.querySelector(s);
const esc = (s) => String(s == null ? "" : s).replace(/[&<>"']/g,
  (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));

let ITEMS = [], AREAS = [], AREA = "", EDIT = null;
const SECTIONS = [["where", "Where we are"], ["did", "What we did"],
                  ["changed", "What changed"], ["next", "Next"]];
const AREA_CLS = { "Tile 1": "a1", "Tile 2": "a2", "Dashboard": "a3", "General": "a4" };

function fmtDate(d) {
  if (!d) return "";
  const [y, m, dd] = d.split("-").map(Number);
  return new Date(y, m - 1, dd).toLocaleDateString(undefined,
    { day: "numeric", month: "short", year: "numeric" });
}

async function load() {
  const r = await fetch("/api/updates", { cache: "no-store" }).then((x) => x.json());
  ITEMS = r.items; AREAS = r.areas;
  drawChips(); drawLatest(); drawList();
}

function drawChips() {
  const host = $("#areaChips");
  host.innerHTML = "";
  ["", ...AREAS].forEach((a) => {
    const b = document.createElement("button");
    const n = a ? ITEMS.filter((e) => e.area === a).length : ITEMS.length;
    b.className = "chip" + (AREA === a ? " on" : "");
    b.textContent = (a || "All") + "  " + n;
    b.onclick = () => { AREA = a; drawChips(); drawList(); };
    host.appendChild(b);
  });
}

function drawLatest() {
  const host = $("#latest");
  host.innerHTML = "";
  AREAS.forEach((a) => {
    const e = ITEMS.find((x) => x.area === a);
    if (!e) return;
    const d = document.createElement("button");
    d.className = "latest-card";
    d.innerHTML = `<span class="area-tag ${AREA_CLS[a]}">${esc(a)}</span>
      <span class="lt-title">${esc(e.title)}</span>
      <span class="lt-sub">${esc(fmtDate(e.date))}${e.author ? " · " + esc(e.author) : ""}</span>
      ${e.where ? `<span class="lt-where">${esc(e.where)}</span>` : ""}`;
    d.onclick = () => {
      const c = document.getElementById("u-" + e.id);
      if (c) { c.scrollIntoView({ behavior: "smooth", block: "start" }); c.classList.add("flash");
        setTimeout(() => c.classList.remove("flash"), 1200); }
    };
    host.appendChild(d);
  });
}

function drawList() {
  const q = $("#search").value.trim().toLowerCase();
  const list = ITEMS.filter((e) => (!AREA || e.area === AREA) &&
    (!q || JSON.stringify(e).toLowerCase().includes(q)));
  const host = $("#list");
  host.innerHTML = "";
  $("#empty").hidden = list.length > 0;
  let lastMonth = "";
  list.forEach((e) => {
    const month = (e.date || "").slice(0, 7);
    if (month !== lastMonth) {
      const h = document.createElement("h3");
      h.className = "month";
      const [y, m] = month.split("-").map(Number);
      h.textContent = month ? new Date(y, m - 1, 1).toLocaleDateString(undefined, { month: "long", year: "numeric" }) : "";
      host.appendChild(h);
      lastMonth = month;
    }
    const art = document.createElement("article");
    art.className = "upd-card";
    art.id = "u-" + e.id;
    const body = SECTIONS.filter(([k]) => e[k]).map(([k, lab]) =>
      `<div class="upd-sec"><h4>${lab}</h4><p>${esc(e[k])}</p></div>`).join("");
    art.innerHTML = `
      <div class="upd-head">
        <div class="upd-when"><span class="d">${esc(fmtDate(e.date))}</span>
          <span class="area-tag ${AREA_CLS[e.area] || "a4"}">${esc(e.area)}</span></div>
        <div class="upd-act">
          <button class="link small" data-edit>Edit</button>
          <button class="link small danger" data-del>Delete</button>
        </div>
      </div>
      <h3 class="upd-title">${esc(e.title)}</h3>
      ${e.author ? `<p class="upd-by">by ${esc(e.author)}${e.updated && e.created && e.updated - e.created > 60 ? " · edited" : ""}</p>` : ""}
      <div class="upd-body">${body}</div>`;
    art.querySelector("[data-edit]").onclick = () => openForm(e);
    art.querySelector("[data-del]").onclick = async () => {
      if (!confirm("Delete this update?")) return;
      await fetch("/api/updates/" + e.id, { method: "DELETE" });
      load();
    };
    host.appendChild(art);
  });
}

function openForm(e) {
  EDIT = e || null;
  const f = $("#form");
  f.area.innerHTML = AREAS.map((a) => `<option>${esc(a)}</option>`).join("");
  const today = new Date();
  const iso = new Date(today.getTime() - today.getTimezoneOffset() * 60000).toISOString().slice(0, 10);
  f.date.value = e ? e.date : iso;
  f.area.value = e ? e.area : (AREA || "Tile 2");
  let who = "";
  try { who = localStorage.getItem("upd_author") || ""; } catch (_) { /* private mode */ }
  f.author.value = e ? (e.author || "") : who;
  ["title", "where", "did", "changed", "next"].forEach((k) => (f[k].value = e ? (e[k] || "") : ""));
  $("#formTitle").textContent = e ? "Edit update" : "New update";
  $("#formErr").hidden = true;
  $("#formModal").hidden = false;
  f.title.focus();
}

async function save(ev) {
  ev.preventDefault();
  const f = $("#form");
  const body = {};
  ["date", "area", "author", "title", "where", "did", "changed", "next"].forEach((k) => (body[k] = f[k].value));
  try { localStorage.setItem("upd_author", body.author); } catch (_) { /* fine */ }
  const r = await fetch(EDIT ? "/api/updates/" + EDIT.id : "/api/updates", {
    method: EDIT ? "PUT" : "POST", headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body) }).then((x) => x.json());
  if (r.error) { $("#formErr").textContent = r.error; $("#formErr").hidden = false; return; }
  $("#formModal").hidden = true;
  load();
}

$("#newUpd").onclick = () => openForm(null);
$("#form").onsubmit = save;
$("#cancel").onclick = () => ($("#formModal").hidden = true);
$("#formModal .info-backdrop").onclick = () => ($("#formModal").hidden = true);
$("#formModal .info-close").onclick = () => ($("#formModal").hidden = true);
$("#search").oninput = drawList;
document.addEventListener("keydown", (e) => { if (e.key === "Escape") $("#formModal").hidden = true; });
load();
