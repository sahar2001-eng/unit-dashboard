/* Updates diary -- front end. Entries live on the server (data/_updates). */

const $ = (s) => document.querySelector(s);
const esc = (s) => String(s == null ? "" : s).replace(/[&<>"']/g,
  (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));

let ITEMS = [], AREAS = [], AREA = "", EDIT = null;
let PENDING = [];          // new files chosen in the form, not uploaded yet
let REMOVE = new Set();    // stored files marked for removal in the form

const IMG = /\.(png|jpe?g|gif|webp)$/i;
function fileKind(n) {
  const e = (n.split(".").pop() || "").toLowerCase();
  if (["doc", "docx", "odt", "rtf"].includes(e)) return ["W", "k-word"];
  if (["xls", "xlsx", "csv", "ods"].includes(e)) return ["X", "k-xls"];
  if (["ppt", "pptx", "key", "odp"].includes(e)) return ["P", "k-ppt"];
  if (e === "pdf") return ["PDF", "k-pdf"];
  if (["png", "jpg", "jpeg", "gif", "webp", "svg", "heic"].includes(e)) return ["IMG", "k-img"];
  if (["mp4", "mov", "avi", "mkv", "webm", "m4v"].includes(e)) return ["VID", "k-vid"];
  if (["zip", "rar", "7z", "gz"].includes(e)) return ["ZIP", "k-zip"];
  if (["py", "js", "c", "h", "json", "txt", "md"].includes(e)) return ["TXT", "k-txt"];
  return [e ? e.slice(0, 4).toUpperCase() : "FILE", "k-other"];
}
function fmtSize(b) {
  if (b < 1024) return b + " B";
  if (b < 1048576) return (b / 1024).toFixed(0) + " KB";
  return (b / 1048576).toFixed(1) + " MB";
}
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
      ${e.where ? `<span class="lt-where">${esc(e.where)}</span>` : ""}
      ${(e.files || []).length ? `<span class="lt-sub">${e.files.length} file${e.files.length > 1 ? "s" : ""} attached</span>` : ""}`;
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
      <div class="upd-body">${body}</div>
      ${filesHtml(e)}`;
    art.querySelector("[data-edit]").onclick = () => openForm(e);
    art.querySelector("[data-del]").onclick = async () => {
      if (!confirm("Delete this update?")) return;
      await fetch("/api/updates/" + e.id, { method: "DELETE" });
      load();
    };
    host.appendChild(art);
  });
}

function filesHtml(e) {
  const fs = e.files || [];
  if (!fs.length) return "";
  const pics = fs.filter((f) => IMG.test(f.file));
  const url = (f) => `/updfiles/${encodeURIComponent(e.id)}/${encodeURIComponent(f.file)}`;
  return `<div class="upd-files">
    <h4>Files</h4>
    ${pics.length ? `<div class="thumbs">${pics.map((f) =>
      `<a href="${url(f)}" target="_blank" rel="noopener" title="${esc(f.name)}"><img src="${url(f)}" alt="${esc(f.name)}" loading="lazy"></a>`).join("")}</div>` : ""}
    <div class="file-chips">${fs.map((f) => { const [k, c] = fileKind(f.name);
      return `<a class="file-chip" href="${url(f)}?dl=1" title="Download ${esc(f.name)}">
        <span class="kind ${c}">${k}</span><span class="fn">${esc(f.name)}</span>
        <span class="fs">${fmtSize(f.size)}</span></a>`; }).join("")}</div>
  </div>`;
}

function drawFileList() {
  const ul = $("#fileList");
  ul.innerHTML = "";
  const stored = (EDIT && EDIT.files) || [];
  stored.forEach((f) => {
    const li = document.createElement("li");
    const gone = REMOVE.has(f.file);
    const [k, c] = fileKind(f.name);
    li.className = gone ? "gone" : "";
    li.innerHTML = `<span class="kind ${c}">${k}</span><span class="fn">${esc(f.name)}</span>
      <span class="fs">${gone ? "will be removed" : fmtSize(f.size)}</span>
      <button type="button" class="link small ${gone ? "" : "danger"}">${gone ? "Keep" : "Remove"}</button>`;
    li.querySelector("button").onclick = () => {
      if (gone) REMOVE.delete(f.file); else REMOVE.add(f.file);
      drawFileList();
    };
    ul.appendChild(li);
  });
  PENDING.forEach((f, i) => {
    const li = document.createElement("li");
    const [k, c] = fileKind(f.name);
    const big = f.size > 200 * 1048576;
    li.className = "new" + (big ? " gone" : "");
    li.innerHTML = `<span class="kind ${c}">${k}</span><span class="fn">${esc(f.name)}</span>
      <span class="fs">${big ? "too big (max 200 MB)" : "new · " + fmtSize(f.size)}</span>
      <button type="button" class="link small danger">Remove</button>`;
    li.querySelector("button").onclick = () => { PENDING.splice(i, 1); drawFileList(); };
    ul.appendChild(li);
  });
}

function addPending(list) {
  for (const f of list) PENDING.push(f);
  drawFileList();
}

function uploadFiles(uid, files) {
  return new Promise((resolve) => {
    const fd = new FormData();
    files.forEach((f) => fd.append("files", f, f.name));
    const x = new XMLHttpRequest();
    x.open("POST", `/api/updates/${encodeURIComponent(uid)}/files`);
    $("#upProg").hidden = false;
    $("#upBar").style.width = "2%";
    $("#upText").textContent = `Uploading ${files.length} file${files.length > 1 ? "s" : ""}…`;
    x.upload.onprogress = (ev) => {
      if (!ev.lengthComputable) return;
      const pc = (100 * ev.loaded) / ev.total;
      $("#upBar").style.width = Math.max(2, pc) + "%";
      $("#upText").textContent = `Uploading… ${Math.round(pc)}%  (${fmtSize(ev.loaded)} of ${fmtSize(ev.total)})`;
    };
    x.onload = () => { let r = {}; try { r = JSON.parse(x.responseText); } catch (_) { r = { error: "upload failed (" + x.status + ")" }; }
      resolve(r); };
    x.onerror = () => resolve({ error: "upload failed — check the connection" });
    x.send(fd);
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
  PENDING = []; REMOVE = new Set();
  $("#upProg").hidden = true;
  drawFileList();
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
  const uid = r.id;
  $("#save").disabled = true;
  for (const fn of REMOVE) {
    await fetch(`/api/updates/${encodeURIComponent(uid)}/files/${encodeURIComponent(fn)}`, { method: "DELETE" });
  }
  const send = PENDING.filter((f) => f.size <= 200 * 1048576);
  if (send.length) {
    EDIT = r;     // a new entry is now saved: a retry must not make a second one
    const u = await uploadFiles(uid, send);
    if (u.error) {
      $("#save").disabled = false;
      $("#formErr").textContent = "The update is saved, but: " + u.error;
      $("#formErr").hidden = false;
      PENDING = PENDING.filter((f) => !(u.added || []).some((a) => a.name === f.name));
      EDIT.files = u.files || EDIT.files;
      drawFileList();
      load();
      return;
    }
  }
  $("#save").disabled = false;
  $("#formModal").hidden = true;
  load();
}

$("#newUpd").onclick = () => openForm(null);
$("#form").onsubmit = save;
$("#cancel").onclick = () => ($("#formModal").hidden = true);
$("#formModal .info-backdrop").onclick = () => ($("#formModal").hidden = true);
$("#formModal .info-close").onclick = () => ($("#formModal").hidden = true);
$("#search").oninput = drawList;
$("#filePick").onclick = () => $("#fileInput").click();
$("#fileInput").onchange = (ev) => { addPending(ev.target.files); ev.target.value = ""; };
const FD = $("#fileDrop");
["dragenter", "dragover"].forEach((t) => FD.addEventListener(t, (ev) => { ev.preventDefault(); FD.classList.add("hot"); }));
["dragleave", "drop"].forEach((t) => FD.addEventListener(t, (ev) => { ev.preventDefault(); FD.classList.remove("hot"); }));
FD.addEventListener("drop", (ev) => addPending(ev.dataTransfer.files));
document.addEventListener("keydown", (e) => { if (e.key === "Escape") $("#formModal").hidden = true; });
load();
