// Mission-control dashboard. Draws the snapshot the server streams on /events (4x a second).
"use strict";
const $ = (id) => document.getElementById(id);
const NS = "http://www.w3.org/2000/svg";          // SVG namespace name (never fetched)
const UNITS = ["rover", "drone_1", "drone_2", "drone_3", "drone_4", "base_a", "base_b"];
const X0 = -56, X1 = 56, Z0 = -1, Z1 = 22;           // map window (metres)
const ANT = { rover: 1.3, base_a: 10.5, base_b: 10.5 };
let snap = null, lastMsg = 0, firstTime = null;

function svg(tag, attrs, parent) {
  const e = document.createElementNS(NS, tag);
  for (const k in attrs) e.setAttribute(k, attrs[k]);
  if (parent) parent.appendChild(e);
  return e;
}
const mx = (x) => ((x - X0) / (X1 - X0)) * 1000;
const mz = (z) => 300 - ((z - Z0) / (Z1 - Z0)) * 300;
const pct = (a, b) => (b ? Math.min(100, Math.round((100 * a) / b)) + " %" : "-");   // counters arrive 1/s per node
const fmtT = (s) => { s = Math.max(0, Math.floor(s)); return String(Math.floor(s / 60)).padStart(2, "0") + ":" + String(s % 60).padStart(2, "0"); };
const short = (u) => (u || "").replace("drone_", "D").replace("base_", "BASE ").toUpperCase();

// ---------------------------------------------------------------- map
function drawMap(s) {
  const m = $("map");
  m.textContent = "";
  if (s.terrain && s.terrain.length) {
    const pts = s.terrain.map(([x, z]) => `${mx(x)},${mz(z)}`).join(" ");
    svg("polygon", { points: `0,300 ${pts} 1000,300`, fill: "var(--ground)", stroke: "var(--ground-edge)" }, m);
  }
  const pos = s.positions || {};
  const ant = (u) => pos[u] && [mx(pos[u][0]), mz(pos[u][2] + (ANT[u] || 0))];
  for (const l of s.links || []) {
    if (!l.up) continue;
    const a = ant(l.a), b = ant(l.b);
    if (a && b) svg("line", { x1: a[0], y1: a[1], x2: b[0], y2: b[1], stroke: "#2e4a3a", "stroke-width": 1.5 }, m);
  }
  const route = (r, color, dash) => {
    if (!r || r.length < 2) return;
    const pts = r.map(ant).filter(Boolean).map((p) => p.join(",")).join(" ");
    svg("polyline", { points: pts, fill: "none", stroke: color, "stroke-width": 4, "stroke-dasharray": dash, opacity: 0.9 }, m);
  };
  if (s.ack_age !== null && s.ack_age < 3 && s.last_ack) route(s.last_ack.cmd_route, "var(--cmd)", "0");
  if (s.odom_age !== null && s.odom_age < 3 && s.last_odom) route(s.last_odom.route, "var(--tel)", "10 6");
  for (const u of UNITS) {
    const p = pos[u];
    if (!p) continue;
    const x = mx(p[0]), z = mz(p[2]);
    const st = (s.stats || {})[u] || {};
    const dead = st.killed;
    if (u.startsWith("base")) {
      svg("rect", { x: x - 18, y: z - 4, width: 36, height: 6, fill: "#9fb3d1" }, m);
      svg("line", { x1: x, y1: z, x2: x, y2: mz(p[2] + ANT[u]), stroke: "#9fb3d1", "stroke-width": 2 }, m);
      svg("circle", { cx: x, cy: mz(p[2] + ANT[u]), r: 5, fill: "#9fb3d1" }, m);
    } else if (u === "rover") {
      svg("rect", { x: x - 8, y: z - 8, width: 16, height: 8, rx: 2, fill: "#ff7a1a" }, m);
    } else {
      const lead = s.coord && s.coord.leader === u;
      svg("circle", { cx: x, cy: z, r: 7, fill: dead ? "#552222" : "#7cc4ff",
        stroke: lead ? "#fff" : "none", "stroke-width": 2 }, m);
      if (lead) svg("text", { x: x, y: z - 22, fill: "#fff", "font-size": 12, "text-anchor": "middle" }, m).textContent = "★ LEADER";
    }
    svg("text", { x: x, y: Math.min(z + 22, 296), fill: dead ? "var(--bad)" : "var(--dim)", "font-size": 12,
      "text-anchor": "middle" }, m).textContent = short(u) + (dead ? " ✕" : "");
    const held = st.held ? (st.held[0] || 0) + (st.held[1] || 0) + (st.held[2] || 0) : 0;
    if (held > 0) {
      svg("rect", { x: x + 10, y: z + 2, width: 34, height: 16, rx: 3, fill: "var(--warn)" }, m);
      svg("text", { x: x + 27, y: z + 14, fill: "#000", "font-size": 11, "text-anchor": "middle" }, m).textContent = "▣" + held;
    }
  }
}

// ---------------------------------------------------------------- odometry track
function drawOdom(s) {
  const o = $("odom");
  o.textContent = "";
  const pts = s.odom || [];
  if (pts.length < 2) { $("odomFacts").textContent = "no odometry yet"; return; }
  const t0 = pts[0].t, t1 = Math.max(pts[pts.length - 1].t, t0 + 1);
  const ox = (t) => ((t - t0) / (t1 - t0)) * 1000, oy = (x) => 150 - ((x - X0) / (X1 - X0)) * 140;
  svg("line", { x1: 0, y1: oy(0), x2: 1000, y2: oy(0), stroke: "#1d2a3d", "stroke-dasharray": "4 4" }, o);
  let late = 0;
  for (let i = 1; i < pts.length; i++) {
    const a = pts[i - 1], b = pts[i];
    if (b.t - a.t > 3) continue;                         // real gap: nothing was sent
    if (b.late) late++;
    svg("line", { x1: ox(a.t), y1: oy(a.x), x2: ox(b.t), y2: oy(b.x),
      stroke: b.late ? "var(--warn)" : "var(--ok)", "stroke-width": 3 }, o);
  }
  $("odomFacts").textContent = `${pts.length} samples · ${late} late · newest x = ${pts[pts.length - 1].x.toFixed(1)} m`;
}

// ---------------------------------------------------------------- panels
function drawPanels(s) {
  const c = s.coord || {};
  const on = c.mode === "on";
  $("meshBtn").textContent = on ? "MESH ON" : "MESH OFF";
  $("meshBtn").setAttribute("aria-pressed", String(on));
  $("leader").textContent = c.leader ? short(c.leader) : "-";
  $("term").textContent = c.term ?? "-";
  $("lease").textContent = c.time ? (s.now - c.time).toFixed(1) + " s" : "-";
  if (firstTime === null) firstTime = s.now;
  $("clock").textContent = fmtT(s.now - (s.started || firstTime));

  const aOk = s.ack_age !== null && s.ack_age < 3, bOk = s.odom_age !== null && s.odom_age < 3;
  $("aState").textContent = aOk ? "CONNECTED" : "NO CONNECTION";
  $("aState").className = aOk ? "ok" : "bad";
  $("aRoute").textContent = aOk ? `${s.last_ack.cmd_route.map(short).join(" → ")} · rtt ${s.last_ack.rtt_ms} ms` : "waiting for an ACK…";
  $("bState").textContent = bOk ? "RECEIVING" : "NO DATA";
  $("bState").className = bOk ? "ok" : "bad";
  $("bRoute").textContent = bOk ? `${s.last_odom.route.map(short).join(" → ")} · ${s.last_odom.delay_ms} ms` : "nothing arriving";

  const rows = [["COMMAND", "commands"], ["TELEMETRY", "odometry"], ["PHOTO_CHUNK", "photo"]];
  $("deliv").innerHTML = "";
  for (const [k, name] of rows) {
    const d = (s.delivery || {})[k] || { sent: 0, delivered: 0, expired: 0 };
    const tr = document.createElement("tr");
    for (const v of [name, d.sent, d.delivered, d.expired, pct(d.delivered, d.sent)]) {
      const td = document.createElement("td"); td.textContent = v; tr.appendChild(td);
    }
    $("deliv").appendChild(tr);
  }

  const held = $("held");
  held.innerHTML = "";
  const max = Math.max(20, ...UNITS.map((u) => { const h = ((s.stats || {})[u] || {}).held || {}; return (h[0] || 0) + (h[1] || 0) + (h[2] || 0); }));
  for (const u of UNITS) {
    const st = (s.stats || {})[u] || {}, h = st.held || {};
    const n = (h[0] || 0) + (h[1] || 0) + (h[2] || 0);
    const row = document.createElement("div");
    row.className = "hrow" + (st.killed ? " dead" : "");
    row.innerHTML = `<span>${short(u)}</span><div class="hbar"></div><span>${st.killed ? "DEAD" : n}</span>`;
    for (const p of [0, 1, 2]) {
      const seg = document.createElement("div");
      seg.className = "p" + p;
      seg.style.width = ((h[p] || 0) / max) * 100 + "%";
      row.children[1].appendChild(seg);
    }
    held.appendChild(row);
  }

  const ph = s.photo;
  if (ph && ph.total) {
    $("photoBar").style.width = (100 * ph.received) / ph.total + "%";
    $("photoText").textContent = ph.done ? `${ph.total} chunks · sha256 ${ph.ok ? "OK ✓" : "MISMATCH ✕"}` : `${ph.received} / ${ph.total} chunks`;
    $("photoText").className = "sub " + (ph.done ? (ph.ok ? "ok" : "bad") : "");
    if (ph.done && ph.image_b64 && $("photoImg").dataset.id !== String(ph.photo_id)) {
      $("photoImg").src = "data:image/png;base64," + ph.image_b64;
      $("photoImg").dataset.id = String(ph.photo_id);
      $("photoImg").hidden = false;
    }
  }

  const f = s.failure || {};
  syncSlider("drop", f.drop_probability), syncSlider("corrupt", f.corrupt_probability);
  const sel = $("unit");
  if (!sel.options.length) for (const u of UNITS) sel.add(new Option(short(u), u));
  if (!sel.dataset.touched && c.leader) sel.value = c.leader;
}

function syncSlider(id, v) {
  if (v === undefined || document.activeElement === $(id)) return;
  $(id).value = Math.round(v * 100);
  $(id + "Out").textContent = Math.round(v * 100) + " %";
}

function addLog(e) {
  const log = $("log");
  const top = log.firstElementChild;
  if (e.event === "route_change" && top && top.dataset.kind === "route") {   // merge the burst into one line
    const units = new Set(top.dataset.units.split(",").concat(e.unit));
    top.dataset.units = [...units].join(",");
    top.children[2].textContent = `route tables updated (${units.size} units)`;
    return;
  }
  const li = document.createElement("li");
  const t = new Date(e.time * 1000).toLocaleTimeString([], { hour12: false });
  const what = { expired: `expired ${e.type} #${e.seq} (${e.where})`, retransmit: `resend ${e.type} #${e.seq} → ${e.to}`,
    election: `election: term ${e.term}`, leader: `now LEADER (term ${e.term})`, killed: "KILLED", revived: "revived",
    route_change: "route table changed", dropped_full: `store full, dropped ${e.type} #${e.seq}`,
    photo: `photo ${e.received}/${e.total}` }[e.event] || e.event;
  li.innerHTML = "<time></time><b></b> <span></span>";
  li.children[0].textContent = t;
  li.children[1].textContent = short(e.unit);
  li.children[2].textContent = what;
  if (e.event === "route_change") { li.dataset.kind = "route"; li.dataset.units = e.unit; }
  if (["expired", "killed", "dropped_full"].includes(e.event)) li.className = "bad";
  else if (["election", "leader", "revived"].includes(e.event)) li.className = "ok";
  log.prepend(li);
  while (log.children.length > 200) log.lastChild.remove();
}

// ---------------------------------------------------------------- wiring
async function post(path, body) {
  try {
    await fetch(path, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
  } catch (err) { console.warn(err); }
}
$("meshBtn").onclick = () => post("/api/mesh", { on: $("meshBtn").getAttribute("aria-pressed") !== "true" });
$("photoBtn").onclick = () => post("/api/photo", {});
for (const id of ["drop", "corrupt"]) {
  $(id).oninput = () => ($(id + "Out").textContent = $(id).value + " %");
  $(id).onchange = () => post("/api/failure", { [id + "_probability"]: Number($(id).value) / 100 });
}
$("unit").onchange = () => ($("unit").dataset.touched = "1");
$("killBtn").onclick = () => post("/api/failure", { kill: $("unit").value });
$("reviveBtn").onclick = () => post("/api/failure", { revive: $("unit").value });

function connect() {
  const es = new EventSource("/events");
  es.addEventListener("snapshot", (ev) => {
    snap = JSON.parse(ev.data);
    lastMsg = Date.now();
    drawPanels(snap); drawMap(snap); drawOdom(snap);
  });
  es.addEventListener("log", (ev) => addLog(JSON.parse(ev.data)));
}
setInterval(() => {
  const ok = Date.now() - lastMsg < 2000;
  $("conn").textContent = ok ? "● LIVE" : "● OFFLINE";
  $("conn").className = ok ? "ok" : "bad";
}, 500);
connect();
