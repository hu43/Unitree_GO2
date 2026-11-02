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
  wpSignature: "",     // DOM 列表只在点位集合变化时重建（避免输入框失焦）
  slam: { mode: "off", map: "", tf_ok: false, maps: [] },
  wpFrame: "odom",
  frame: "odom",       // 画布当前坐标系（选中持久地图时为 map）
  wpMap: "",
  poseOk: false,       // map 系下机器人位姿是否可信（TF 新鲜）
  tourNote: "",        // 导览暂停原因（如定位不稳定）
  staticMap: null, staticImg: null, staticSig: "",
  showCloud: true,     // 局部滑动点云叠加开关
  // 持久地图底图开关。默认关闭（先只看局部点云）；用户点工具栏「🗺️ 持久地图」可打开，
  // 选择记在 localStorage 里，刷新后保持。
  showBaseMap: (function () {
    try { return localStorage.getItem("go2.showBaseMap") === "1"; } catch (e) { return false; }
  })(),
  legacyWps: false,    // 持久地图开启、点位库却仍是 odom 库（过渡态）
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

function wpSignature(wps) {
  return wps.map((w) => `${w.id}:${w.name}`).join("|");
}

function applyWaypoints(wps, forceRender) {
  state.waypoints = wps || [];
  const sig = wpSignature(state.waypoints);
  if (forceRender || sig !== state.wpSignature) {
    state.wpSignature = sig;
    renderWaypoints();
    renderTourOrder();
  }
}

function acceptStaticMap(sm) {
  // sm === null 表示当前没有底图（地图被清掉/还没选）——先清状态再返回
  if (!sm) {
    if (state.staticSig) { state.staticSig = ""; state.staticMap = null; state.staticImg = null; draw(); }
    return;
  }
  const sig = sm.name + ":" + sm.width + "x" + sm.height + ":" + (sm.png_b64 || "").length;
  if (sig === state.staticSig) return;
  state.staticSig = sig;
  state.staticMap = sm;
  const im = new Image();
  im.onload = () => { state.staticImg = im; draw(); };
  im.src = "data:image/png;base64," + sm.png_b64;
}

function handleMsg(msg) {
  if (msg.type === "init") {
    state.maxSpeed = msg.config?.max_speed ?? 0.5;
    $("speed-slider").value = state.maxSpeed;
    $("speed-val").textContent = state.maxSpeed.toFixed(2);
    state.frame = msg.frame || "odom";
    state.wpFrame = msg.wp_frame || "odom";
    state.wpMap = msg.wp_map || "";
    // 坐标系与开关要先定，点位列表才不会被上一帧的旧值渲染
    applyPersistentMode(msg.persistent_map);
    state.legacyWps = state.persistentMap !== false && state.wpFrame !== "map";
    applyWaypoints(msg.waypoints, true);
    if (msg.static_map !== undefined) acceptStaticMap(msg.static_map);
    updateWpFrame();
  } else if (msg.type === "state") {
    state.mapPoints = msg.map || [];
    // 后端 pos 是 [x, y, z] 数组；统一转成 {x,y,z}，否则 state.pos.x 为
    // undefined，状态栏与机器人图标都会失效（NaN）。
    const rawPos = msg.pos;
    if (Array.isArray(rawPos)) {
      state.pos = { x: Number(rawPos[0]) || 0, y: Number(rawPos[1]) || 0, z: Number(rawPos[2]) || 0 };
    } else if (rawPos && typeof rawPos === "object") {
      state.pos = { x: Number(rawPos.x) || 0, y: Number(rawPos.y) || 0, z: Number(rawPos.z) || 0 };
    }
    state.yaw = Number(msg.yaw) || 0;
    state.tour = msg.tour || state.tour;
    state.controlMode = msg.control_mode || "joystick";
    state.frame = msg.frame || "odom";
    state.wpFrame = msg.wp_frame || "odom";
    state.wpMap = msg.wp_map || "";
    // 坐标系与开关要先定，点位列表才不会被上一帧的旧值渲染
    applyPersistentMode(msg.persistent_map);
    state.legacyWps = state.persistentMap !== false && state.wpFrame !== "map";
    if (msg.waypoints) applyWaypoints(msg.waypoints, false);
    if (msg.slam) { state.slam = msg.slam; updateSlamPane(); }
    state.poseOk = msg.pose_ok !== false;
    state.tourNote = msg.tour_note || "";
    if (msg.static_map !== undefined) acceptStaticMap(msg.static_map);
    updateWpFrame();
    $("st-map").textContent = state.mapPoints.length;
    const stFrame = $("st-frame");
    if (stFrame) stFrame.textContent = state.frame === "map"
      ? `地图(${state.wpMap || "map"})` : "里程计(odom)";
    updateJoystickEnabled();
    updateStatus();
    draw();
  } else if (msg.type === "ack") {
    if (msg.waypoints) applyWaypoints(msg.waypoints, true);
    if (msg.slam || msg.mode !== undefined) {
      if (msg.mode !== undefined) state.slam = { ...state.slam, ...msg };
      updateSlamPane();
    }
    if (msg.error && msg.ok === false) alert((msg.scope ? msg.scope + ": " : "") + msg.error);
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
  if (state.showBaseMap) drawStaticMap();

  // 局部滑动点云（SCAN-Planner 10m 窗口）。map 系下只有 TF 新鲜时点云
  // 才是 map 坐标；否则后端发的仍是 odom 坐标，画出来会错位，直接跳过。
  const cloudUsable = state.frame === "odom" ? true : state.poseOk;
  if (state.showCloud && cloudUsable) {
    const pts = state.mapPoints;
    const step = Math.max(1, Math.floor(pts.length / 6000));
    ctx.globalAlpha = (state.showBaseMap && state.staticImg) ? 0.3 : 1.0;
    for (let i = 0; i < pts.length; i += step) {
      const p = pts[i];
      const sx = wx(p[0]), sy = wy(p[1]);
      if (sx < -5 || sy < -5 || sx > W + 5 || sy > H + 5) continue;
      ctx.fillStyle = colorByHeight(p[2]);
      ctx.fillRect(sx, sy, 2, 2);
    }
    ctx.globalAlpha = 1.0;
  }

  // waypoints
  for (const wp of state.waypoints) {
    drawWaypoint(wp);
  }

  // tour target line
  if (state.tour.active && state.tour.current) {
    drawTourLink(state.tour.current);
  }

  // robot（map 系下 TF 不新鲜时不画：位置不可信，画出来会误导；
  // odom 系没有定位概念，照常画）
  if (state.frame === "odom" || state.poseOk) {
    drawRobot(state.pos.x, state.pos.y, state.yaw);
  }

  drawLegend(W, H);
}

function drawStaticMap() {
  const m = state.staticMap, img = state.staticImg;
  if (!m || !img) return;
  const [ox, oy] = m.origin;
  const x0 = wx(ox);
  const y0 = wy(oy + m.height * m.resolution);
  const w = m.width * m.resolution * state.view.scale;
  const h = m.height * m.resolution * state.view.scale;
  ctx.save();
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(img, x0, y0, w, h);
  ctx.restore();
}

// 持久地图关闭时（odom 模式，10-13 画法）：藏掉 SLAM 卡片，
// 且不画持久底图——只留裸点云 + 里程计原点。
function applyPersistentMode(on) {
  if (on === undefined) return;
  state.persistentMap = on !== false;
  const card = $("slam-card");
  if (card) card.style.display = state.persistentMap ? "" : "none";
  if (!state.persistentMap && state.showBaseMap) {
    state.showBaseMap = false;
    try { localStorage.setItem("go2.showBaseMap", "0"); } catch (e) {}
    const b = $("btn-basemap");
    if (b) b.classList.remove("active");
  }
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
  ctx.fillText(state.frame === "map" ? "map(0,0)" : "odom(0,0)", wx(0) + 4, wy(0) - 4);
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
  if (sx < -30 || sy < -30 || sx > canvas.clientWidth + 30 || sy > canvas.clientHeight + 30) return;
  const sel = state.selected === wp.id;
  // 半透明底衬：压在红色墙体/点云上也读得清
  ctx.beginPath();
  ctx.arc(sx, sy, (sel ? 9 : 7) + 4, 0, Math.PI * 2);
  ctx.fillStyle = "rgba(11,15,24,.45)";
  ctx.fill();
  ctx.beginPath();
  ctx.arc(sx, sy, sel ? 9 : 7, 0, Math.PI * 2);
  ctx.fillStyle = sel ? "#ffd166" : "#4a9eff";
  ctx.fill();
  ctx.strokeStyle = "#fff";
  ctx.lineWidth = 1.5;
  ctx.stroke();
  // label with shadow plate
  ctx.font = "10px sans-serif";
  const label = `#${wp.id}${wp.name ? " " + wp.name : ""}`;
  const tw = ctx.measureText(label).width;
  ctx.fillStyle = "rgba(11,15,24,.55)";
  ctx.fillRect(sx + 8, sy - 12, tw + 8, 22);
  ctx.fillStyle = "#fff";
  ctx.fillText(`#${wp.id}`, sx + 12, sy - 4);
  if (wp.name) {
    ctx.fillText(wp.name, sx + 12, sy + 8);
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

function drawLegend(W, H) {
  ctx.save();
  ctx.font = "11px sans-serif";
  const pad = 8, lh = 16;
  const rows = [];
  if (state.showBaseMap) {
    rows.push(["#e44a4a", "墙 / 展台"]);
    rows.push(["rgba(34,64,94,.9)", "可走区"]);
  }
  rows.push(["#4a9eff", "展品点"]);
  rows.push(["#34d399", "机器狗"]);
  if (state.showCloud) rows.push(["rgba(120,200,230,.9)", "局部点云"]);
  if (rows.length < 2) return;
  const bw = 108, bh = rows.length * lh + 2 * pad - 4;
  const x0 = 8, y0 = H - bh - 8;
  ctx.fillStyle = "rgba(10,14,22,.72)";
  ctx.strokeStyle = "rgba(70,84,110,.5)";
  ctx.beginPath();
  ctx.roundRect(x0, y0, bw, bh, 6);
  ctx.fill(); ctx.stroke();
  rows.forEach(([c, t], i) => {
    const y = y0 + pad + i * lh + 5;
    ctx.fillStyle = c;
    ctx.beginPath();
    ctx.arc(x0 + 12, y, 4, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = "#c9d3e4";
    ctx.fillText(t, x0 + 24, y + 4);
  });
  // scale bar (bottom right)
  const meters = state.view.scale >= 60 ? 1 : (state.view.scale >= 24 ? 2 : 5);
  const px = meters * state.view.scale;
  const sx1 = W - 16, sy1 = H - 14;
  ctx.strokeStyle = "#9fb0cc";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(sx1 - px, sy1); ctx.lineTo(sx1, sy1);
  ctx.moveTo(sx1 - px, sy1 - 4); ctx.lineTo(sx1 - px, sy1 + 4);
  ctx.moveTo(sx1, sy1 - 4); ctx.lineTo(sx1, sy1 + 4);
  ctx.stroke();
  ctx.fillStyle = "#9fb0cc";
  ctx.textAlign = "right";
  ctx.fillText(`${meters} m`, sx1, sy1 - 8);
  ctx.textAlign = "left";
  ctx.restore();
}

function updateStatus() {
  $("st-mode").textContent = state.controlMode === "tour" ? "导览(SCAN-Planner)" : "摇杆";
  $("st-pos").textContent = state.poseOk || state.frame === "odom"
    ? `${state.pos.x.toFixed(2)}, ${state.pos.y.toFixed(2)}`
    : "（未定位）";
  $("st-yaw").textContent = state.poseOk || state.frame === "odom"
    ? `${(state.yaw * 180 / Math.PI).toFixed(1)}°` : "-";
  const st = state.tour;
  const mapState = {
    turning: "🔄 转向中", acting: "🤖 动作中", introducing: "🔊 介绍中", traveling: "🚶 前往"
  };
  const phase = mapState[st.state] || "进行中";
  if (st.active && st.state !== "traveling") {
    const cur = st.current || {};
    $("st-tour").textContent = `${phase}：${cur.name || "展品"} ${st.state === "introducing" ? "→ " + st.intro : ""}`;
    $("tour-status").textContent = `${phase} ${cur.name || "展品"}` +
      (st.intro ? `：${st.intro}` : "");
  } else if (st.active) {
    const cur = st.current || {};
    $("st-tour").textContent = `${phase} 第 ${st.idx + 1}/${st.total} 点 → ${cur.name || ""}`;
    $("tour-status").textContent = `${phase} ${st.idx + 1}/${st.total}（${cur.name || ""}）`;
  } else {
    $("st-tour").textContent = "未导览";
    $("tour-status").textContent = state.tourNote || "未开始";
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
      send({ type: "goal", x: p.x, y: p.y, frame: state.frame });
    } else if (state.mode === "addpoint") {
      send({ type: "waypoint", op: "add", x: p.x, y: p.y,
             frame: state.frame, name: `展品${state.waypoints.length + 1}` });
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

function joyStart(e) {
  if (state.controlMode === "tour") return;
  state.joystick.active = true;
  joy.setPointerCapture(e.pointerId);
  joyMove(e);
}
function joyMove(e) {
  if (!state.joystick.active) return;
  const rect = joy.getBoundingClientRect();
  let dx = e.clientX - (rect.left + rect.width / 2);
  let dy = e.clientY - (rect.top + rect.height / 2);
  const max = rect.width / 2 - 14;
  const d = Math.hypot(dx, dy);
  if (d > max) { dx = dx / d * max; dy = dy / d * max; }
  knob.style.left = `calc(50% + ${dx}px)`;
  knob.style.top = `calc(50% + ${dy}px)`;
  state.joystick.dx = dx / max;   // -1..1
  state.joystick.dy = dy / max;
  sendJoystick();
}
function joyEnd() {
  if (!state.joystick.active) return;
  state.joystick.active = false;
  state.joystick.dx = 0;
  state.joystick.dy = 0;
  knob.style.left = "50%";
  knob.style.top = "50%";
  sendJoystick();
}

function sendJoystick() {
  if (!state.joystick.active && !state.joystick.wasActive) {
    return;
  }
  state.joystick.wasActive = state.joystick.active;
  const vx = -state.joystick.dy * state.maxSpeed;       // up = forward
  const vy = -state.joystick.dx * state.maxSpeed * 0.6; // left = strafe left
  const vyaw = -state.joystick.dx * 0.8;                // right = turn right
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

// ---------------- tools (goal / addpoint / cloud toggle) ----------------
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

// 图层开关：持久地图底图 / 局部点云
// 两个都是独立图层——底图关掉就只剩实时局部点云，点云关掉就只剩底图。
(function () {
  const bar = document.querySelector(".map-tools");
  if (!bar) return;

  const baseBtn = document.createElement("button");
  baseBtn.id = "btn-basemap";
  baseBtn.className = "tool";
  baseBtn.textContent = "🗺️ 持久地图";
  baseBtn.title = "显示/隐藏已保存的持久地图底图（红墙）";
  baseBtn.classList.toggle("active", state.showBaseMap);
  baseBtn.addEventListener("click", () => {
    state.showBaseMap = !state.showBaseMap;
    baseBtn.classList.toggle("active", state.showBaseMap);
    try { localStorage.setItem("go2.showBaseMap", state.showBaseMap ? "1" : "0"); } catch (e) {}
    draw();
  });
  bar.insertBefore(baseBtn, $("btn-recenter"));

  const cloudBtn = document.createElement("button");
  cloudBtn.id = "btn-cloud";
  cloudBtn.className = "tool";
  cloudBtn.textContent = "🟦 局部点云";
  cloudBtn.title = "显示/隐藏 SCAN-Planner 的局部滑动栅格点云";
  cloudBtn.classList.toggle("active", state.showCloud);
  cloudBtn.addEventListener("click", () => {
    state.showCloud = !state.showCloud;
    cloudBtn.classList.toggle("active", state.showCloud);
    draw();
  });
  bar.insertBefore(cloudBtn, $("btn-recenter"));
})();

// ---------------- waypoints ----------------
function renderWaypoints() {
  const list = $("wp-list");
  list.innerHTML = "";
  const actionOpts = [
    ["", "无动作"], ["hello", "👋 打招呼"], ["heart", "❤️ 比心"],
    ["dance1", "💃 舞蹈1"], ["dance2", "🕺 舞蹈2"],
    ["stretch", "🧘 伸懒腰"], ["scrape", "🙏 拜年作揖"], ["balance", "⚖️ 平衡站"]
  ];
  if (state.legacyWps && state.persistentMap !== false
      && state.waypoints.length) {
    const note = document.createElement("li");
    note.className = "wp-legacy-note";
    note.textContent = "⚠️ 以下为旧版（里程计系）点位，仅供查看，不建议导览使用。加载持久地图定位后请重新记录。";
    list.appendChild(note);
  }
  state.waypoints.forEach((wp) => {
    const li = document.createElement("li");
    li.className = "wp-item" + (state.legacyWps ? " legacy" : "");
    li.innerHTML = `
      <input type="checkbox" class="wp-check" data-id="${wp.id}" ${isInOrder(wp.id) ? "checked" : ""}>
      <span class="wpid">#${wp.id}</span>
      <span class="wpname">${wp.name}</span>
      <span class="wpcoord">(${wp.x.toFixed(1)}, ${wp.y.toFixed(1)})</span>
      ${state.legacyWps ? '<span class="wp-badge">旧版</span>' : ""}
      <button class="repos" data-id="${wp.id}" title="用摇杆开到新位置后点此重录该点位">🔁 重录</button>
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

// ---------------- tour order (编排) ----------------
const tourOrder = [];   // waypoint ids in guided order

function isInOrder(id) {
  return tourOrder.includes(id);
}

function renderTourOrder() {
  const ol = $("tour-order");
  ol.innerHTML = "";
  if (tourOrder.length === 0) {
    ol.innerHTML = '<li class="tour-empty">未选择展品，勾选上方展品加入导览顺序</li>';
    return;
  }
  tourOrder.forEach((id, i) => {
    const wp = state.waypoints.find((w) => w.id === id);
    if (!wp) return;
    const li = document.createElement("li");
    li.innerHTML = `
      <span class="o-idx">${i + 1}</span>
      <span class="o-name">${wp.name || "#" + wp.id}</span>
      <span class="o-tools">
        <button class="mv up" data-id="${id}" ${i === 0 ? "disabled" : ""}>↑</button>
        <button class="mv down" data-id="${id}" ${i === tourOrder.length - 1 ? "disabled" : ""}>↓</button>
        <button class="rm" data-id="${id}">✕</button>
      </span>`;
    ol.appendChild(li);
  });
}

function orderSyncCheckboxes() {
  document.querySelectorAll(".wp-check").forEach((cb) => {
    cb.checked = isInOrder(parseInt(cb.dataset.id));
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
  const repos = e.target.closest(".repos");
  if (repos) {
    send({ type: "waypoint", op: "reposition", id: parseInt(repos.dataset.id) });
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

// checkbox -> join/leave tour order
$("wp-list").addEventListener("change", (e) => {
  const cb = e.target.closest(".wp-check");
  if (cb) {
    const id = parseInt(cb.dataset.id);
    const idx = tourOrder.indexOf(id);
    if (cb.checked && idx === -1) {
      tourOrder.push(id);
    } else if (!cb.checked && idx !== -1) {
      tourOrder.splice(idx, 1);
    }
    renderTourOrder();
    return;
  }
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

// tour order list: up/down/remove
$("tour-order").addEventListener("click", (e) => {
  const up = e.target.closest(".mv.up");
  const down = e.target.closest(".mv.down");
  const rm = e.target.closest(".rm");
  if (up) {
    const id = parseInt(up.dataset.id);
    const i = tourOrder.indexOf(id);
    if (i > 0) { [tourOrder[i - 1], tourOrder[i]] = [tourOrder[i], tourOrder[i - 1]]; }
  } else if (down) {
    const id = parseInt(down.dataset.id);
    const i = tourOrder.indexOf(id);
    if (i !== -1 && i < tourOrder.length - 1) {
      [tourOrder[i + 1], tourOrder[i]] = [tourOrder[i], tourOrder[i + 1]];
    }
  } else if (rm) {
    const id = parseInt(rm.dataset.id);
    const i = tourOrder.indexOf(id);
    if (i !== -1) tourOrder.splice(i, 1);
  } else {
    return;
  }
  renderTourOrder();
  orderSyncCheckboxes();
});

$("btn-record").addEventListener("click", () => {
  if (state.frame === "map" && !state.poseOk) {
    alert("定位未就绪：地图底图已显示，但还没在定位。请先点「📍 加载地图定位」再记录点位。");
    return;
  }
  send({ type: "waypoint", op: "record", name: `展品${state.waypoints.length + 1}` });
});

$("btn-save").addEventListener("click", () => {
  // server auto-saves on every change; this is a visual confirmation
  alert("点位已保存在服务器 config.json");
});

// ---------------- tour ----------------
$("btn-tour-start").addEventListener("click", () => {
  const checked = tourOrder;
  if (checked.length === 0) {
    alert("请先勾选要导览的展品（加入导览顺序）");
    return;
  }
  send({ type: "tour", op: "start", ids: checked });
});

// 全选：把所有展品按当前列表顺序加入导览（与"清空顺序"相反）
$("btn-tour-all").addEventListener("click", () => {
  const ids = state.waypoints.map((w) => w.id);
  if (ids.length === 0) {
    alert("还没有展品点位，请先记录或在地图上添加点位");
    return;
  }
  // 保留已有顺序在前，其余按列表顺序补到末尾，避免打乱用户已排好的次序
  const kept = tourOrder.filter((id) => ids.includes(id));
  const rest = ids.filter((id) => !kept.includes(id));
  tourOrder.length = 0;
  tourOrder.push(...kept, ...rest);
  renderTourOrder();
  orderSyncCheckboxes();
});

$("btn-tour-clear").addEventListener("click", () => {
  tourOrder.length = 0;
  renderTourOrder();
  orderSyncCheckboxes();
});

$("btn-tour-stop").addEventListener("click", () => {
  send({ type: "tour", op: "stop" });
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

// ---------------- SLAM persistent map ----------------
let slamMapsSig = "";
function updateSlamPane() {
  const s = state.slam || {};
  const el = $("slam-state");
  if (!el) return;
  const modeText = {
    off: state.wpMap ? "⚪ 未定位（仅查看地图）" : "⚪ 关闭（内置里程计）",
    starting: "⏳ 启动中…",
    mapping: "🧭 建图中…（走完展厅后点保存）",
    saving: "💾 保存地图中…（约 10-30 秒）",
    localization: `🟢 定位中（地图: ${s.map || "-"}）`,
  }[s.mode] || s.mode;
  let tf = s.mode === "localization"
    ? (s.tf_ok ? " · TF 正常" : " · TF 未就绪")
    : "";
  const q = s.quality;
  if (s.mode === "localization" && q){
    tf += ` · 匹配偏差 ${q.residual.toFixed(2)}m`;
    if (!s.quality_ok) tf += " ⚠️";
  }
  // 定位稳定性：TF 跳变频率决定 map 系坐标是否可信
  if (s.mode === "localization"){
    if (s.unstable) {
      tf += ` · 🚨 定位不稳定（近 10s ${s.jumps_recent} 次跳变）`;
    } else if (s.jumps_recent >= 4) {
      tf += ` · ⚠️ 定位波动（${s.jumps_recent} 次跳变）`;
    } else if (s.jumps_recent !== undefined) {
      tf += " · 稳定";
    }
  }
  if (s.mode === "localization" && s.quality_ok === false){
    tf += "（定位不准：请到地图查看器点选初始位姿）";
  }
  el.textContent = `状态: ${modeText}${tf}`;
  el.style.color = s.mode === "localization"
    ? (s.unstable ? "var(--danger)"
       : ((s.tf_ok && s.quality_ok !== false) ? "var(--ok)" : "var(--warn)"))
    : "var(--muted)";

  const maps = s.maps || [];
  const sig = maps.join("|");
  const sel = $("slam-maps");
  if (sel && sig !== slamMapsSig) {
    slamMapsSig = sig;
    const cur = sel.value;
    sel.innerHTML = maps.length
      ? maps.map((m) => `<option value="${escHtml(m)}">${escHtml(m)}</option>`).join("")
      : `<option value="">（暂无已保存地图）</option>`;
    if (cur && maps.includes(cur)) sel.value = cur;
    else if (state.wpMap && maps.includes(state.wpMap)) sel.value = state.wpMap;
    else if (s.map && maps.includes(s.map)) sel.value = s.map;
  } else if (sel && state.wpMap && maps.includes(state.wpMap) && sel.value !== state.wpMap) {
    sel.value = state.wpMap;   // 后端切换了当前地图 -> 下拉框跟随
  }

  const mapping = s.mode === "mapping";
  const localizing = s.mode === "localization";
  const busy = s.mode === "starting" || s.mode === "saving";
  $("btn-slam-map").disabled = s.mode !== "off";
  $("btn-slam-save").disabled = !mapping || busy;
  $("btn-slam-discard").disabled = !(mapping || s.mode === "starting");
  $("btn-slam-loc").disabled = s.mode !== "off" || maps.length === 0;
  $("btn-slam-locstop").disabled = !(localizing || s.mode === "starting");
}

function updateWpFrame() {
  const el = $("wp-frame");
  if (!el) return;
  if (state.wpFrame === "map") {
    el.textContent = `点位坐标系: 地图「${state.wpMap}」（抗漂移，已持久化）`;
    el.style.color = "var(--ok)";
  } else if (state.persistentMap === false) {
    // 持久地图已关闭：odom 是当前选定模式，不是降级
    el.textContent = "点位坐标系: 内置里程计（odom）";
    el.style.color = "";
  } else {
    el.textContent = "点位坐标系: 旧版内置里程计（odom）· 不可靠，仅查看";
    el.style.color = "var(--danger)";
  }
}

$("btn-slam-map").addEventListener("click", () => {
  if (!confirm("开始建图：请确认机器人已在展厅里、SCAN-Planner 已启动。继续？")) return;
  send({ type: "slam", op: "start_mapping" });
});
$("btn-slam-save").addEventListener("click", () => {
  const name = ($("slam-name").value || "venue").trim();
  if (!name) { alert("请填写地图名"); return; }
  if (!confirm(`保存为地图「${name}」并结束建图？`)) return;
  send({ type: "slam", op: "stop_mapping", name });
});
$("btn-slam-discard").addEventListener("click", () => {
  if (!confirm("放弃本次建图（不保存）？")) return;
  send({ type: "slam", op: "discard_mapping" });
});
// 下拉框选地图 = 只切换"当前地图"（底图 + 点位库），不启动定位。
// 要定位请再点「📍 加载地图定位」。
// 注意：定位正在另一张地图上运行时，后端会拒绝静默切换（否则机器人位姿
// 与底图会张冠李戴）。这里先征得用户同意，再"停止定位 → 切图"两步走。
$("slam-maps").addEventListener("change", (e) => {
  const name = e.target.value;
  if (!name) return;
  const localizing = (state.slam.mode === "localization" || state.slam.mode === "starting");
  const runningMap = state.slam.map || "";
  if (localizing && name !== runningMap) {
    const tourBusy = state.tour && state.tour.active;
    const msg = `定位正在「${runningMap}」上运行。\n\n` +
      `切换到「${name}」需要先停止定位（底图和点位库会切到 ${name}，` +
      `机器人位置在重新定位前不再显示）。` +
      (tourBusy ? "\n\n⚠️ 导览正在进行中，切换会中断导览。" : "") +
      "\n\n继续？";
    if (!confirm(msg)) {
      e.target.value = runningMap;   // 用户取消 -> 下拉框回到实际定位的地图
      return;
    }
    send({ type: "slam", op: "stop_localization" });
    setTimeout(() => send({ type: "slam", op: "select_map", name }), 700);
    return;
  }
  send({ type: "slam", op: "select_map", name });
});
$("btn-slam-loc").addEventListener("click", () => {
  const sel = $("slam-maps");
  const name = (sel && sel.value) || "";
  if (!name) { alert("还没有可用的地图，请先建图"); return; }
  if (!confirm(`加载地图「${name}」并启动定位？\n请把狗放在展厅内（任意位置均可，定位会自动收敛）。`)) return;
  send({ type: "slam", op: "start_localization", name });
});
$("btn-slam-locstop").addEventListener("click", () => {
  if (!confirm("停止定位？底图和点位库保持当前地图不变，但机器狗位置将不再显示。")) return;
  send({ type: "slam", op: "stop_localization" });
});

// ---------------- init ----------------
window.addEventListener("resize", () => { resize(); draw(); });
resize();
connect();
setInterval(() => { if (!document.hidden) draw(); }, 1000);
