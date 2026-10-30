// Mission-control dashboard. Draws the snapshot the server streams on /events (4x a second).
"use strict";
const $ = (id) => document.getElementById(id);
const NS = "http://www.w3.org/2000/svg";          // SVG namespace name (never fetched)
const UNITS = ["rover", "drone_1", "drone_2", "drone_3", "drone_4", "base"];
const X0 = -56, X1 = 56, Z0 = -1, Z1 = 22;           // map window (metres)
const ANT = { rover: 1.3, base: 10.5 };
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
  if (s.target && s.terrain && s.terrain.length) {          // the science target: a red flag on the ground
    const tx = mx(s.target[0]);
    const ground = s.terrain.reduce((a, p) => (Math.abs(p[0] - s.target[0]) < Math.abs(a[0] - s.target[0]) ? p : a));
    const tz = mz(ground[1]);
    svg("line", { x1: tx, y1: tz, x2: tx, y2: tz - 30, stroke: "#fff", "stroke-width": 2 }, m);
    svg("polygon", { points: `${tx},${tz - 30} ${tx + 18},${tz - 24} ${tx},${tz - 18}`, fill: "#ff3030" }, m);
    svg("text", { x: tx, y: tz - 36, fill: "#ff6060", "font-size": 12, "text-anchor": "middle" }, m).textContent = "TARGET";
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

  const fd = s.feed;
  if (fd) {
    const age = s.now - fd.t;                    // a frozen picture is shown as such, never as live
    if ($("feedImg").dataset.t !== String(fd.t)) {           // only a new frame is decoded again
      $("feedImg").src = "data:image/jpeg;base64," + fd.jpg_b64;
      $("feedImg").dataset.t = String(fd.t);
    }
    $("feedImg").hidden = false;
    $("feedImg").style.opacity = age < 2 ? 1 : 0.35;
    $("feedText").textContent = age < 2 ? "live" : `link down, frozen ${age.toFixed(0)} s ago`;
  }
  lastPhotoId = s.photo ? s.photo.photo_id : null;
  const ph = s.photo && s.photo.photo_id !== ignoreId ? s.photo : null;   // after TAKE PHOTO the old picture is not shown
  if (ph && ph.total) {
    $("photoBar").style.width = (100 * ph.received) / ph.total + "%";
    $("photoText").textContent = ph.done ? `Photo received ✓ ${ph.total} chunks · sha256 ${ph.ok ? "OK ✓" : "MISMATCH ✕"} · ${$("photoImg").dataset.at || ""}`
                                         : `Photo taken, travelling over the mesh: ${ph.received} / ${ph.total} chunks`;
    $("photoText").className = "sub " + (ph.done ? (ph.ok ? "ok" : "bad") : "");
    if (ph.done && ph.image_b64 && $("photoImg").dataset.id !== String(ph.photo_id)) {
      $("photoImg").src = `data:image/${ph.image_b64.startsWith("/9j") ? "jpeg" : "png"};base64,` + ph.image_b64;
      $("photoImg").dataset.id = String(ph.photo_id);
      $("photoImg").dataset.at = "taken " + new Date().toLocaleTimeString([], { hour12: false });
      $("photoImg").hidden = false;
    }
  }

  renderUplink(s);

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

// The rover's screen: the picture mission control sends it. With no link the base holds the chunks, so the
// picture only forms (from the top down) as chunks get through, and it stays unfinished while the link is down.
function renderUplink(s) {
  const up = s.uplink, us = s.uplink_sent, img = $("upImg"), txt = $("upText");
  const pending = us && (!up || us.t > up.t);
  let state = "";
  if (pending) {
    $("upBar").style.width = "0%";
    img.hidden = true; img.dataset.key = "";
    txt.textContent = `Sent from the base (${us.chunks} chunks). The rover has received nothing yet: waiting for a link…`;
  } else if (up && up.total) {
    $("upBar").style.width = (100 * up.received) / up.total + "%";
    if (up.done) {
      if (up.image_b64 && img.dataset.key !== "done" + up.photo_id) {
        img.src = `data:image/jpeg;base64,${up.image_b64}`;
        img.dataset.key = "done" + up.photo_id; img.hidden = false;
      }
      state = up.ok ? "ok" : "bad";
      txt.textContent = `Received by the rover ✓ ${up.total} chunks · sha256 ${up.ok ? "OK ✓" : "MISMATCH ✕"}`;
    } else {
      const key = `p${up.photo_id}-${up.received}`;
      if (up.partial_b64 && img.dataset.key !== key) {
        img.src = `data:image/jpeg;base64,${up.partial_b64}`;
        img.dataset.key = key; img.hidden = false;
      }
      const stalled = s.now - up.t > 3;
      state = stalled ? "bad" : "";
      txt.textContent = `Receiving: ${up.received} / ${up.total} chunks` + (stalled ? " · link lost, the rest is waiting at the base…" : "");
    }
  }
  $("upScreen").classList.toggle("forming", !(up && up.done && !pending));     // static until the picture is complete
  txt.className = ("sub " + state).trim();
}

let seenEpoch = null;
let lastPhotoId = null, ignoreId = null;   // see the TAKE PHOTO button

function addLog(e) {
  const log = $("log");
  const top = log.firstElementChild;
  if (e.event === "packet") {                      // camera feed / photo packets: one line per kind, with a counter
    const key = "pkt-" + e.type, hops = e.route.map(short).join(" → ");
    const n = top && top.dataset.kind === key ? Number(top.dataset.n) + 1 : 1;
    const li = n > 1 ? top : document.createElement("li");
    if (n === 1) { li.innerHTML = "<time></time><b></b> <span></span>"; li.dataset.kind = key; log.prepend(li); }
    li.dataset.n = n;
    li.children[0].textContent = new Date(e.time * 1000).toLocaleTimeString([], { hour12: false });
    li.children[1].textContent = "BASE";
    li.children[2].textContent = `${e.type === "FEED" ? "camera frame" : "photo chunk"} #${e.seq} via ${hops} · ${e.delay_ms} ms` + (n > 1 ? `  (×${n})` : "");
    while (log.children.length > 200) log.lastChild.remove();
    return;
  }
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
    photo: `photo ${e.received}/${e.total}`,
    uplink_sent: `message from base: ${e.total} chunks queued`,
    uplink: `rover received ${e.received}/${e.total} of the message${e.ok === true ? " ✓" : ""}` }[e.event] || e.event;
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
// Controls are locked by a passcode the server checks on every POST. It is never stored in this
// file: the browser asks once per tab (sessionStorage), typed into a masked box, and the header
// travels in clear text (put TLS in front on an untrusted network).
const PASS_KEY = "meshPasscode";
let passAsk = null;

function askPasscode(wrong) {
  if (passAsk) return passAsk;                     // several clicks at once share one dialog
  const dlg = $("passDlg"), input = $("passIn");
  $("passErr").hidden = !wrong;
  input.value = "";
  passAsk = new Promise((resolve) => {
    dlg.addEventListener("close", () => {
      const p = dlg.returnValue === "ok" ? input.value : "";
      input.value = "";
      passAsk = null;
      if (p) sessionStorage.setItem(PASS_KEY, p);
      resolve(p);
    }, { once: true });
    dlg.returnValue = "";
    dlg.showModal();
    input.focus();
  });
  return passAsk;
}

async function post(path, body, wrong) {
  try {
    const r = await fetch(path, {
      method: "POST",
      headers: { "Content-Type": "application/json", "X-Passcode": sessionStorage.getItem(PASS_KEY) || "" },
      body: JSON.stringify(body),
    });
    if (r.status !== 401) return;
    sessionStorage.removeItem(PASS_KEY);
    if (await askPasscode(wrong)) return post(path, body, true);   // Cancel leaves the controls locked
  } catch (err) { console.warn(err); }
}
$("meshBtn").onclick = () => post("/api/mesh", { on: $("meshBtn").getAttribute("aria-pressed") !== "true" });
$("photoBtn").onclick = () => {
  ignoreId = lastPhotoId;                          // wait for a NEW picture
  $("photoImg").hidden = true;
  $("photoBar").style.width = "0";
  $("photoText").className = "sub";
  $("photoText").textContent = "Photo requested, waiting for the rover...";
  post("/api/photo", {});
};
fetch("/phone_url").then((r) => r.text()).then((u) => { $("qrText").textContent = u; }).catch(() => {});
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
    const s = JSON.parse(ev.data);
    if (seenEpoch === null) seenEpoch = s.epoch;             // a new epoch = the simulation was restarted: start the page fresh
    else if (s.epoch !== seenEpoch) { location.reload(); return; }
    snap = s;
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
