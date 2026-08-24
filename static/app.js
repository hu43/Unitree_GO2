/* Go2 web console frontend: map render + joystick + waypoints + tour */
"use strict";

// ---------------- state ----------------
const state = {
  ws: null,
  connected: false,
  mapPoints: [],
  pos: { x: 0, y: 0, z: 0 },
  yaw: 0,
  waypoints: [],
  tour: { active: false, idx: -1, total: 0, current: null },
  controlMode: "joystick",   // "joystick" | "tour"
  maxSpeed: 0.5,
  mode: "goal",        // "goal" | "addpoint"
  view: { cx: 0, cy: 0, scale: 30 },  // world->screen px per meter
  dragging: false,
  joystick: { active: false, dx: 0, dy: 0 },
  selected: null,
};

// ---------------- DOM ----------------
const $ = (id) => document.getElementById(id);
const canvas = $("map");
const ctx = canvas.getContext("2d");

// ---------------- WebSocket ----------------
function connect() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  const url = `${proto}://${location.host}/ws`;
  state.ws = new WebSocket(url);

  state.ws.onopen = () => {
    state.connected = true;
    setConn("已连接", "ok");
  };
  state.ws.onclose = () => {
    state.connected = false;
    setConn("已断开，重连中…", "bad");
    setTimeout(connect, 2000);
  };
  state.ws.onerror = () => { state.ws.close(); };
  state.ws.onmessage = (ev) => {
    let msg;
    try { msg = JSON.parse(ev.data); } catch { return; }
    handleMsg(msg);
  };
}

function setConn(text, cls) {
  const el = $("conn");
  el.textContent = `● ${text}`;
  el.className = "conn " + (cls || "");
}

function send(obj) {
  if (state.ws && state.ws.readyState === WebSocket.OPEN) {
    state.ws.send(JSON.stringify(obj));
  }
}

function handleMsg(msg) {
  if (msg.type === "init") {
    state.waypoints = msg.waypoints || [];
    state.maxSpeed = msg.config?.max_speed ?? 0.5;
    $("speed-slider").value = state.maxSpeed;
    $("speed-val").textContent = state.maxSpeed.toFixed(2);
    renderWaypoints();
  } else if (msg.type === "state") {
    state.mapPoints = msg.map || [];
    state.pos = msg.pos || { x: 0, y: 0, z: 0 };
    state.yaw = msg.yaw || 0;
    state.tour = msg.tour || state.tour;
    state.controlMode = msg.control_mode || "joystick";
    $("st-map").textContent = state.mapPoints.length;
    updateJoystickEnabled();
    updateStatus();
    draw();
  } else if (msg.type === "ack") {
    if (msg.waypoints) {
      state.waypoints = msg.waypoints;
      renderWaypoints();
    }
    if (msg.running !== undefined && document.querySelector("#scan-status")) {
      updateScanStatus(!!msg.running);
    }
    if (msg.ok === false) console.warn("cmd rejected:", msg.error);
  }
}

// ---------------- map rendering ----------------
function resize() {
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  canvas.width = rect.width * dpr;
  canvas.height = rect.height * dpr;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}

function wx(x) { return (x - state.view.cx) * state.view.scale + canvas.clientWidth / 2; }
function wy(y) { return (state.view.cy - y) * state.view.scale + canvas.clientHeight / 2; }
function worldFromScreen(px, py) {
  const x = (px - canvas.clientWidth / 2) / state.view.scale + state.view.cx;
  const y = state.view.cy - (py - canvas.clientHeight / 2) / state.view.scale;
  return { x, y };
}

function draw() {
  const W = canvas.clientWidth, H = canvas.clientHeight;
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = "#0b0f18";
  ctx.fillRect(0, 0, W, H);

  drawGrid(W, H);

  // map points (occupancy) - colored by height
  const pts = state.mapPoints;
  const step = Math.max(1, Math.floor(pts.length / 6000));
  for (let i = 0; i < pts.length; i += step) {
    const p = pts[i];
    const sx = wx(p[0]), sy = wy(p[1]);
    if (sx < -5 || sy < -5 || sx > W + 5 || sy > H + 5) continue;
    ctx.fillStyle = colorByHeight(p[2]);
    ctx.fillRect(sx, sy, 2, 2);
  }

  // waypoints
  for (const wp of state.waypoints) {
    drawWaypoint(wp);
  }

  // tour target line
  if (state.tour.active && state.tour.current) {
    drawTourLink(state.tour.current);
  }

  // robot
  drawRobot(state.pos.x, state.pos.y, state.yaw);
}

function drawGrid(W, H) {
  ctx.strokeStyle = "rgba(60,70,95,.35)";
  ctx.lineWidth = 1;
  const step = 1; // meter
  const gpx = step * state.view.scale;
  const ox = wx(0) % gpx, oy = wy(0) % gpx;
  ctx.beginPath();
  for (let x = ox; x < W; x += gpx) { ctx.moveTo(x, 0); ctx.lineTo(x, H); }
  for (let y = oy; y < H; y += gpx) { ctx.moveTo(0, y); ctx.lineTo(W, y); }
  ctx.stroke();
  // origin label
  ctx.fillStyle = "#5a6a8f";
  ctx.font = "10px sans-serif";
  ctx.fillText("odom(0,0)", wx(0) + 4, wy(0) - 4);
}

function colorByHeight(z) {
  // cyan(floor) -> blue -> orange -> red
  const t = Math.max(0, Math.min(1, (z + 0.5) / 1.5));
  const r = Math.round(80 + t * 175);
  const g = Math.round(190 - t * 150);
  const b = Math.round(220 - t * 160);
  return `rgb(${r},${g},${b})`;
}

function drawWaypoint(wp) {
  const sx = wx(wp.x), sy = wy(wp.y);
  const sel = state.selected === wp.id;
  ctx.beginPath();
  ctx.arc(sx, sy, sel ? 9 : 7, 0, Math.PI * 2);
  ctx.fillStyle = sel ? "#ffd166" : "#4a9eff";
  ctx.fill();
  ctx.strokeStyle = "#fff";
  ctx.lineWidth = 1.5;
  ctx.stroke();
  ctx.fillStyle = "#fff";
  ctx.font = "10px sans-serif";
  ctx.fillText(`#${wp.id}`, sx + 10, sy - 8);
  if (wp.name) {
    ctx.fillText(wp.name, sx + 10, sy + 4);
  }
}

function drawTourLink(wp) {
  const sx = wx(wp.x), sy = wy(wp.y);
  ctx.beginPath();
  ctx.setLineDash([6, 4]);
  ctx.strokeStyle = "#34d399";
  ctx.lineWidth = 2;
  ctx.moveTo(wx(state.pos.x), wy(state.pos.y));
  ctx.lineTo(sx, sy);
  ctx.stroke();
  ctx.setLineDash([]);
}

function drawRobot(x, y, yaw) {
  const sx = wx(x), sy = wy(y);
  // heading arrow
  ctx.save();
  ctx.translate(sx, sy);
  ctx.rotate(-yaw);
  ctx.beginPath();
  ctx.moveTo(12, 0);
  ctx.lineTo(-10, -8);
  ctx.lineTo(-10, 8);
  ctx.closePath();
  ctx.fillStyle = "#34d399";
  ctx.fill();
  ctx.strokeStyle = "#0f7a52";
  ctx.lineWidth = 1.5;
  ctx.stroke();
  ctx.restore();
  // position dot
  ctx.beginPath();
  ctx.arc(sx, sy, 4, 0, Math.PI * 2);
  ctx.fillStyle = "#34d399";
  ctx.fill();
}

function updateStatus() {
  $("st-mode").textContent = state.controlMode === "tour" ? "导览(SCAN-Planner)" : "摇杆";
  $("st-pos").textContent = `${state.pos.x.toFixed(2)}, ${state.pos.y.toFixed(2)}`;
  $("st-yaw").textContent = `${(state.yaw * 180 / Math.PI).toFixed(1)}°`;
  const st = state.tour;
  const introRow = $("intro-row");
  const mapState = {
    turning: "🔄 转向中", acting: "🤖 动作中", introducing: "🔊 介绍中", traveling: "🚶 前往"
  };
  const phase = mapState[st.state] || "进行中";
  if (st.active && st.state !== "traveling") {
    const cur = st.current || {};
    $("st-tour").textContent = `${phase}：${cur.name || "展品"} ${st.state === "introducing" ? "→ " + st.intro : ""}`;
    $("tour-status").textContent = `${phase} ${cur.name || "展品"}` +
      (st.intro ? `：${st.intro}` : "");
    introRow.style.display = "flex";
  } else if (st.active) {
    const cur = st.current || {};
    $("st-tour").textContent = `${phase} 第 ${st.idx + 1}/${st.total} 点 → ${cur.name || ""}`;
    $("tour-status").textContent = `${phase} ${st.idx + 1}/${st.total}（${cur.name || ""}）`;
    introRow.style.display = "none";
  } else {
    $("st-tour").textContent = "未导览";
    $("tour-status").textContent = "未开始";
    introRow.style.display = "none";
  }
}

// ---------------- map interaction ----------------
function onPointerDown(e) {
  const rect = canvas.getBoundingClientRect();
  const px = e.clientX - rect.left, py = e.clientY - rect.top;

  // check waypoint hit first
  for (const wp of state.waypoints) {
    const sx = wx(wp.x), sy = wy(wp.y);
    if (Math.hypot(px - sx, py - sy) < 14) {
      state.selected = wp.id;
      renderWaypoints();
      draw();
      return;
    }
  }
  state.selected = null;

  // mode action on click (drag = pan)
  state.dragStart = { x: e.clientX, y: e.clientY, viewCx: state.view.cx, viewCy: state.view.cy };
  state.dragging = true;
  canvas.setPointerCapture(e.pointerId);
}

function onPointerMove(e) {
  if (!state.dragging || !state.dragStart) return;
  const rect = canvas.getBoundingClientRect();
  const dx = e.clientX - state.dragStart.x;
  const dy = e.clientY - state.dragStart.y;
  state.view.cx = state.dragStart.viewCx - dx / state.view.scale;
  state.view.cy = state.dragStart.viewCy + dy / state.view.scale;
  draw();
}

function onPointerUp(e) {
  if (!state.dragging) return;
  const rect = canvas.getBoundingClientRect();
  const moved = Math.hypot(e.clientX - state.dragStart.x, e.clientY - state.dragStart.y);
  if (moved < 6) {
    // treat as click
    const p = worldFromScreen(e.clientX - rect.left, e.clientY - rect.top);
    if (state.mode === "goal") {
      send({ type: "goal", x: p.x, y: p.y });
    } else if (state.mode === "addpoint") {
      send({ type: "waypoint", op: "add", x: p.x, y: p.y, name: `展品${state.waypoints.length + 1}` });
    }
  }
  state.dragging = false;
}

canvas.addEventListener("pointerdown", onPointerDown);
canvas.addEventListener("pointermove", onPointerMove);
canvas.addEventListener("pointerup", onPointerUp);
canvas.addEventListener("wheel", (e) => {
  e.preventDefault();
  const rect = canvas.getBoundingClientRect();
  const p = worldFromScreen(e.clientX - rect.left, e.clientY - rect.top);
  const factor = e.deltaY < 0 ? 1.15 : 0.87;
  state.view.scale = Math.max(5, Math.min(200, state.view.scale * factor));
  // keep cursor anchored
  state.view.cx = p.x - (e.clientX - rect.left - canvas.clientWidth / 2) / state.view.scale;
  state.view.cy = p.y + (e.clientY - rect.top - canvas.clientHeight / 2) / state.view.scale;
  draw();
}, { passive: false });

function updateJoystickEnabled() {
  const disabled = state.controlMode === "tour";
  joy.classList.toggle("disabled", disabled);
  if (disabled && state.joystick.active) {
    // tour took over while the stick was held -> release it
    state.joystick.active = false;
    state.joystick.dx = 0;
    state.joystick.dy = 0;
    knob.style.left = "50%";
    knob.style.top = "50%";
  }
}

// ---------------- joystick ----------------
const joy = $("joystick");
const knob = $("stick-knob");
const JOY_R = 40;

function joyStart(e) {
  if (state.controlMode === "tour") return; // disabled during tour
  e.preventDefault();
  state.joystick.active = true;
  joy.setPointerCapture(e.pointerId);
}
function joyMove(e) {
  if (!state.joystick.active) return;
  const rect = joy.getBoundingClientRect();
  let dx = e.clientX - (rect.left + rect.width / 2);
  let dy = e.clientY - (rect.top + rect.height / 2);
  const len = Math.hypot(dx, dy);
  if (len > JOY_R) { dx = dx / len * JOY_R; dy = dy / len * JOY_R; }
  state.joystick.dx = dx / JOY_R;
  state.joystick.dy = dy / JOY_R;
  knob.style.left = `calc(50% + ${dx}px)`;
  knob.style.top = `calc(50% + ${dy}px)`;
  sendJoystick();
}
function joyEnd(e) {
  state.joystick.active = false;
  state.joystick.dx = 0;
  state.joystick.dy = 0;
  knob.style.left = "50%";
  knob.style.top = "50%";
  send({ type: "cmd_vel", vx: 0, vy: 0, vyaw: 0 });
}

function sendJoystick() {
  const vx = -state.joystick.dy * state.maxSpeed;      // up = forward (screen Y down)
  const vy = -state.joystick.dx * state.maxSpeed * 0.6; // right = strafe right
  const vyaw = -state.joystick.dx * 0.8;              // right = turn right
  send({ type: "cmd_vel", vx: vx, vy: vy, vyaw: vyaw });
}

joy.addEventListener("pointerdown", joyStart);
joy.addEventListener("pointermove", joyMove);
joy.addEventListener("pointerup", joyEnd);
joy.addEventListener("pointercancel", joyEnd);

// ---------------- buttons ----------------
document.querySelectorAll(".act").forEach((btn) => {
  btn.addEventListener("click", () => {
    const act = btn.dataset.act;
    if (act === "stop") {
      // emergency: stop everything + damp (backend handles it)
      send({ type: "emergency" });
      return;
    }
    send({ type: "sport", action: act });
  });
});

$("speed-slider").addEventListener("input", (e) => {
  state.maxSpeed = parseFloat(e.target.value);
  $("speed-val").textContent = state.maxSpeed.toFixed(2);
});

// ---------------- tools (goal / addpoint) ----------------
function setMode(mode) {
  state.mode = mode;
  $("btn-goal").classList.toggle("active", mode === "goal");
  $("btn-addpoint").classList.toggle("active", mode === "addpoint");
  $("map-hint").textContent = mode === "goal"
    ? "点击地图设置导航目标（机器人自主避障前往）"
    : "点击地图添加路径点";
}
$("btn-goal").addEventListener("click", () => setMode("goal"));
$("btn-addpoint").addEventListener("click", () => setMode("addpoint"));
$("btn-recenter").addEventListener("click", () => {
  state.view.cx = state.pos.x;
  state.view.cy = state.pos.y;
  draw();
});

// ---------------- waypoints ----------------
function renderWaypoints() {
  const list = $("wp-list");
  list.innerHTML = "";
  const actionOpts = [
    ["", "无动作"], ["hello", "👋 打招呼"], ["heart", "❤️ 比心"],
    ["dance1", "💃 舞蹈1"], ["dance2", "🕺 舞蹈2"],
    ["stretch", "🧘 伸懒腰"], ["scrape", "🙏 拜年作揖"], ["balance", "⚖️ 平衡站"]
  ];
  state.waypoints.forEach((wp) => {
    const li = document.createElement("li");
    li.className = "wp-item";
    li.innerHTML = `
      <input type="checkbox" data-id="${wp.id}" ${state.selected === wp.id ? "checked" : ""}>
      <span class="wpid">#${wp.id}</span>
      <span class="wpname">${wp.name}</span>
      <span class="wpcoord">(${wp.x.toFixed(1)}, ${wp.y.toFixed(1)})</span>
      <button class="del" data-id="${wp.id}">✕</button>
      <div class="wp-intro">
        <input type="text" class="intro-input" data-id="${wp.id}"
               placeholder="输入展品介绍文字（TTS 语音）"
               value="${escHtml(wp.intro || "")}">
        <button class="preview" data-id="${wp.id}">🔊 试听</button>
      </div>
      <div class="wp-intro">
        <label class="mini">转向°</label>
        <input type="number" class="yaw-input" data-id="${wp.id}" step="5"
               value="${wp.yaw_at !== undefined ? wp.yaw_at : 0}" title="到达后转向到的角度(度,相对地图)">
        <label class="mini">动作</label>
        <select class="action-select" data-id="${wp.id}">
          ${actionOpts.map(([v, label]) =>
            `<option value="${v}" ${(wp.action || "") === v ? "selected" : ""}>${label}</option>`).join("")}
        </select>
      </div>`;
    list.appendChild(li);
  });
}

function escHtml(s) {
  return String(s).replace(/&/g, "&amp;").replace(/</g, "&lt;")
    .replace(/>/g, "&gt;").replace(/"/g, "&quot;");
}

$("wp-list").addEventListener("click", (e) => {
  const del = e.target.closest(".del");
  if (del) {
    send({ type: "waypoint", op: "del", id: parseInt(del.dataset.id) });
    return;
  }
  const prev = e.target.closest(".preview");
  if (prev) {
    const id = parseInt(prev.dataset.id);
    const input = document.querySelector(`.intro-input[data-id="${id}"]`);
    const text = input ? input.value : "";
    send({ type: "tts", op: "preview", id: id, text: text });
  }
});

$("wp-list").addEventListener("change", (e) => {
  const inp = e.target.closest(".intro-input");
  if (inp) {
    const id = parseInt(inp.dataset.id);
    send({ type: "tts", op: "set_intro", id: id, intro: inp.value });
    return;
  }
  const yaw = e.target.closest(".yaw-input");
  if (yaw) {
    send({ type: "waypoint", op: "set_yaw_at", id: parseInt(yaw.dataset.id), yaw_at: parseFloat(yaw.value) });
    return;
  }
  const act = e.target.closest(".action-select");
  if (act) {
    send({ type: "waypoint", op: "set_action", id: parseInt(act.dataset.id), action: act.value });
  }
});

$("btn-record").addEventListener("click", () => {
  send({ type: "waypoint", op: "record", name: `展品${state.waypoints.length + 1}` });
});

$("btn-save").addEventListener("click", () => {
  // server auto-saves on every change; this is a visual confirmation
  alert("点位已保存在服务器 config.json");
});

// ---------------- tour ----------------
$("btn-tour-start").addEventListener("click", () => {
  const checked = [...document.querySelectorAll("#wp-list input:checked")]
    .map((i) => parseInt(i.dataset.id));
  if (checked.length === 0) {
    alert("请先勾选要导览的展品点");
    return;
  }
  send({ type: "tour", op: "start", ids: checked });
});

$("btn-tour-stop").addEventListener("click", () => {
  send({ type: "tour", op: "stop" });
});

$("btn-skip-intro").addEventListener("click", () => {
  send({ type: "tour", op: "skip" });
});

// ---------------- SCAN-Planner system control ----------------
function updateScanStatus(running) {
  const el = $("scan-status");
  el.textContent = running ? "SCAN-Planner: 🟢 运行中" : "SCAN-Planner: ⚪ 已停止";
  el.style.color = running ? "var(--ok)" : "var(--muted)";
}
$("btn-scan-start").addEventListener("click", () => {
  send({ type: "scanner", op: "start" });
  $("scan-status").textContent = "SCAN-Planner: 启动中…";
});
$("btn-scan-stop").addEventListener("click", () => {
  send({ type: "scanner", op: "stop" });
});
// poll scanner status every 3s
setInterval(() => send({ type: "scanner", op: "status" }), 3000);

// ---------------- init ----------------
window.addEventListener("resize", () => { resize(); draw(); });
resize();
connect();
setInterval(() => { if (!document.hidden) draw(); }, 1000);
