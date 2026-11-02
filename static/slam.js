/* Go2 SLAM 建图独立页面：建图 / 重定位 / 初始位姿点选 */
"use strict";

const $ = (id) => document.getElementById(id);
const cv = $("cv");
const ctx = cv.getContext("2d");

const MODE_NAMES = {
  off: "关闭",
  starting: "启动中…",
  mapping: "建图中",
  saving: "保存中…",
  localization: "重定位中",
};

const state = {
  ws: null,
  slam: {
    mode: "off", map: "", maps: [], tf_ok: false,
    quality: null, quality_ok: true, unstable: false, jumps_recent: 0,
    odom_topic: "", odom_source: "", odom_ok: false, odom_age: null,
  },
  frame: "odom",
  mapName: "",
  wpFrame: "odom",
  wpMap: "",
  waypoints: [],
  points: [],
  pos: [0, 0, 0],
  yaw: 0,
  poseOk: false,
  robotMap: null,
  staticMap: null,
  staticImg: null,
  staticSig: "",
  persistentMap: true,
  view: { cx: 0, cy: 0, scale: 30 },
  pointers: new Map(),
  drag: null,
  pinch: null,
  picking: false,
  pickedPose: null,
  mapsSig: "",
  wpSig: "",
  fitPending: false,
};

// ---------------- WebSocket ----------------
function setConn(text, cls) {
  const el = $("conn");
  el.textContent = `● ${text}`;
  el.className = "conn " + (cls || "");
}

function connect() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  state.ws = new WebSocket(`${proto}://${location.host}/ws`);
  state.ws.onopen = () => setConn("已连接", "ok");
  state.ws.onclose = () => {
    setConn("已断开，重连中…", "bad");
    setTimeout(connect, 2000);
  };
  state.ws.onerror = () => state.ws.close();
  state.ws.onmessage = (ev) => {
    let msg;
    try { msg = JSON.parse(ev.data); } catch { return; }
    if (msg.type === "init") {
      applyInit(msg);
    } else if (msg.type === "state") {
      applyState(msg);
    } else if (msg.type === "ack") {
      handleAck(msg);
    }
  };
}

function send(obj) {
  if (state.ws && state.ws.readyState === WebSocket.OPEN) {
    state.ws.send(JSON.stringify(obj));
    return true;
  }
  log("未连接，指令未发送", "err");
  return false;
}

// ---------------- 状态应用 ----------------
function applyInit(msg) {
  state.frame = msg.frame || "odom";
  state.wpFrame = msg.wp_frame || "odom";
  state.wpMap = msg.wp_map || "";
  state.persistentMap = msg.persistent_map !== false;
  if (msg.waypoints) {
    state.waypoints = msg.waypoints;
    paintWaypoints();
  }
  if (msg.static_map !== undefined) acceptStaticMap(msg.static_map);
  paintStatus();
}

function applyState(msg) {
  if (Array.isArray(msg.map)) state.points = msg.map;
  if (Array.isArray(msg.pos)) {
    state.pos = [Number(msg.pos[0]) || 0, Number(msg.pos[1]) || 0, Number(msg.pos[2]) || 0];
  }
  state.yaw = Number(msg.yaw) || 0;
  state.frame = msg.frame || "odom";
  state.mapName = msg.map_name || "";
  state.wpFrame = msg.wp_frame || "odom";
  state.wpMap = msg.wp_map || "";
  state.poseOk = msg.pose_ok !== false;
  state.robotMap = msg.robot_map || null;
  state.persistentMap = msg.persistent_map !== false;
  if (msg.slam) state.slam = msg.slam;
  if (msg.waypoints) {
    state.waypoints = msg.waypoints;
    paintWaypoints();
  }
  if (msg.static_map !== undefined) acceptStaticMap(msg.static_map);
  paintStatus();
  paintMaps();
  draw();
}

function acceptStaticMap(sm) {
  if (!sm) {
    if (state.staticSig) {
      state.staticSig = "";
      state.staticMap = null;
      state.staticImg = null;
      draw();
    }
    return;
  }
  const sig = `${sm.name}:${sm.width}x${sm.height}:${(sm.png_b64 || "").length}`;
  if (sig === state.staticSig) return;
  const firstForMap = !state.staticMap || state.staticMap.name !== sm.name;
  state.staticSig = sig;
  state.staticMap = sm;
  const im = new Image();
  im.onload = () => {
    state.staticImg = im;
    if (firstForMap || state.fitPending) {
      state.fitPending = false;
      fitView();
    } else {
      draw();
    }
  };
  im.src = "data:image/png;base64," + sm.png_b64;
}

// ---------------- 状态显示 ----------------
function paintStatus() {
  const s = state.slam || {};

  // 里程计：明确显示狗的里程计来源与健康度
  const odomEl = $("sl-odom");
  const src = s.odom_source || s.odom_topic || "/sportmodestate";
  odomEl.textContent = s.odom_ok === false ? "无数据" : src;
  odomEl.className = "v " + (s.odom_ok === false ? "bad" : "ok");
  odomEl.title = s.odom_topic ? `TF 里程计话题 ${s.odom_topic}` : "";

  // SLAM 状态
  const modeEl = $("sl-mode");
  const mode = s.mode || "off";
  modeEl.textContent = MODE_NAMES[mode] || mode;
  modeEl.className = "v " + (
    mode === "localization" ? (s.tf_ok ? (s.unstable ? "warn" : "ok") : "warn")
      : mode === "mapping" ? "ok"
        : mode === "off" ? "" : "warn"
  );

  // 当前地图
  const mapEl = $("sl-map");
  const cur = state.mapName || s.map || "";
  mapEl.textContent = cur || "未选择";
  mapEl.className = "v" + (cur ? "" : " warn");

  // map->odom
  const tfEl = $("sl-tf");
  const localizing = mode === "localization" || mode === "starting";
  tfEl.textContent = !localizing ? "未定位" : (s.tf_ok ? "就绪" : "未就绪");
  tfEl.className = "v " + (!localizing ? "" : (s.tf_ok ? "ok" : "bad"));

  // 匹配偏差
  const qEl = $("sl-quality");
  if (mode === "localization" && s.quality && typeof s.quality.residual === "number") {
    qEl.textContent = `${s.quality.residual.toFixed(2)} m`;
    qEl.className = "v " + (s.quality_ok === false ? "warn" : "ok");
    qEl.title = `内点 ${(s.quality.inliers * 100).toFixed(0)}% · ${s.quality.n} 点`;
  } else {
    qEl.textContent = "-";
    qEl.className = "v";
    qEl.title = "";
  }

  // 定位稳定性
  const stEl = $("sl-stab");
  if (mode === "localization") {
    stEl.textContent = s.unstable ? `跳变 ${s.jumps_recent || 0} 次` : "稳定";
    stEl.className = "v " + (s.unstable ? "warn" : "ok");
  } else {
    stEl.textContent = "-";
    stEl.className = "v";
  }

  // 按钮可用性
  const mapping = mode === "mapping";
  $("btn-start-map").disabled = mode !== "off";
  $("btn-save-map").disabled = !mapping;
  $("btn-discard-map").disabled = !(mapping || mode === "starting");
  $("btn-loc-start").disabled = mode !== "off" || !$("map-select").value;
  $("btn-loc-stop").disabled = !(mode === "localization" || mode === "starting");
}

function paintMaps() {
  const sel = $("map-select");
  if (!sel) return;
  const maps = (state.slam && state.slam.maps) || [];
  const sig = maps.join("|");
  if (sig !== state.mapsSig) {
    state.mapsSig = sig;
    const keep = sel.value;
    sel.innerHTML = "";
    const none = document.createElement("option");
    none.value = "";
    none.textContent = "（不使用地图 · odom 点位）";
    sel.appendChild(none);
    for (const name of maps) {
      const o = document.createElement("option");
      o.value = name;
      o.textContent = name;
      sel.appendChild(o);
    }
    if (keep && maps.includes(keep)) sel.value = keep;
  }
  // 没有手动选择时，跟随后端当前地图
  if (!sel.value) {
    const cur = state.mapName || state.slam.map || "";
    if (cur && maps.includes(cur)) sel.value = cur;
  }
  paintStatus();
}

function paintWaypoints() {
  const list = $("wp-list");
  const wps = state.waypoints || [];
  const sig = wps.map((w) => `${w.id}:${w.x},${w.y}:${w.name || ""}`).join("|");
  if (sig === state.wpSig) return;
  state.wpSig = sig;
  list.innerHTML = "";
  if (!wps.length) {
    const li = document.createElement("li");
    li.innerHTML = '<span class="nm">暂无点位</span>';
    list.appendChild(li);
  } else {
    for (const w of wps) {
      const li = document.createElement("li");
      const id = document.createElement("span");
      id.className = "id";
      id.textContent = `#${w.id}`;
      const nm = document.createElement("span");
      nm.className = "nm";
      nm.textContent = w.name || "未命名";
      const xy = document.createElement("span");
      xy.className = "xy";
      xy.textContent = `${Number(w.x).toFixed(2)}, ${Number(w.y).toFixed(2)}`;
      li.append(id, nm, xy);
      list.appendChild(li);
    }
  }
  const fr = $("wp-frame");
  if (state.wpFrame === "map") {
    fr.textContent = `点位坐标系: 地图「${state.wpMap || state.mapName || "-"}」（map 系，抗漂移）`;
    fr.style.color = "var(--ok)";
  } else if (state.persistentMap === false) {
    fr.textContent = "点位坐标系: 内置里程计（odom）· 持久地图已关闭";
    fr.style.color = "var(--muted)";
  } else {
    fr.textContent = "点位坐标系: 内置里程计（odom）· 未选择地图，重启后可能偏移";
    fr.style.color = "var(--warn)";
  }
}

// ---------------- 日志 ----------------
function log(text, cls) {
  const box = $("sl-log");
  const line = document.createElement("div");
  line.className = cls || "";
  const t = new Date().toLocaleTimeString("zh-CN", { hour12: false });
  line.textContent = `[${t}] ${text}`;
  box.appendChild(line);
  box.scrollTop = box.scrollHeight;
  while (box.childElementCount > 60) box.removeChild(box.firstChild);
}

function handleAck(msg) {
  if (msg.ok === false) {
    log(msg.error || "指令失败", "err");
    return;
  }
  if (msg.msg) log(msg.msg, "ok");
  else if (msg.error) log(msg.error, "err");
  else if (msg.map) log(`当前地图: ${msg.map}`, "ok");
  else if (msg.mode) log(`SLAM: ${MODE_NAMES[msg.mode] || msg.mode}`, "info");
}

// ---------------- 画布 ----------------
function dpr() { return window.devicePixelRatio || 1; }
function cssW() { return cv.clientWidth || 1; }
function cssH() { return cv.clientHeight || 1; }

function resize() {
  const r = cv.getBoundingClientRect();
  const d = dpr();
  cv.width = Math.max(1, Math.round(r.width * d));
  cv.height = Math.max(1, Math.round(r.height * d));
  draw();
}

function wx(x) { return (x - state.view.cx) * state.view.scale + cssW() / 2; }
function wy(y) { return cssH() / 2 - (y - state.view.cy) * state.view.scale; }

function worldFromScreen(px, py) {
  return {
    x: (px - cssW() / 2) / state.view.scale + state.view.cx,
    y: state.view.cy - (py - cssH() / 2) / state.view.scale,
  };
}

function mapExtent() {
  const m = state.staticMap;
  if (m && m.width && m.height && m.resolution && m.origin) {
    const [ox, oy] = m.origin;
    return [ox, oy, ox + m.width * m.resolution, oy + m.height * m.resolution];
  }
  // 没有底图时用点云/点位/机器人估算范围
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
  const add = (x, y) => {
    if (!Number.isFinite(x) || !Number.isFinite(y)) return;
    minX = Math.min(minX, x); maxX = Math.max(maxX, x);
    minY = Math.min(minY, y); maxY = Math.max(maxY, y);
  };
  const step = Math.max(1, Math.floor((state.points.length || 0) / 3000));
  for (let i = 0; i < state.points.length; i += step) {
    const p = state.points[i];
    if (p) add(p[0], p[1]);
  }
  for (const w of state.waypoints) add(w.x, w.y);
  if (state.frame === "map" && state.robotMap) add(state.robotMap[0], state.robotMap[1]);
  else add(state.pos[0], state.pos[1]);
  if (!Number.isFinite(minX)) return [-5, -5, 5, 5];
  if (maxX - minX < 1) { minX -= 1; maxX += 1; }
  if (maxY - minY < 1) { minY -= 1; maxY += 1; }
  return [minX, minY, maxX, maxY];
}

function fitView() {
  const [x0, y0, x1, y1] = mapExtent();
  const W = cssW(), H = cssH();
  const sx = W / Math.max(0.01, x1 - x0);
  const sy = H / Math.max(0.01, y1 - y0);
  state.view.scale = Math.max(1, Math.min(sx, sy) * 0.9);
  state.view.cx = (x0 + x1) / 2;
  state.view.cy = (y0 + y1) / 2;
  draw();
}

function draw() {
  const d = dpr();
  ctx.setTransform(d, 0, 0, d, 0, 0);
  const W = cssW(), H = cssH();
  ctx.fillStyle = "#0b0f18";
  ctx.fillRect(0, 0, W, H);

  drawGrid(W, H);
  drawStaticMap();
  drawPoints(W, H);
  drawWaypoints();
  drawRobot();
  drawPickedPose();
  drawScaleBar(W, H);
}

function drawGrid(W, H) {
  let step = 1;
  const s = state.view.scale;
  while (step * s < 45) step *= 2;
  while (step * s > 220) step /= 2;
  const tl = worldFromScreen(0, 0);
  const br = worldFromScreen(W, H);
  const xMin = Math.floor(Math.min(tl.x, br.x) / step) * step;
  const xMax = Math.ceil(Math.max(tl.x, br.x) / step) * step;
  const yMin = Math.floor(Math.min(tl.y, br.y) / step) * step;
  const yMax = Math.ceil(Math.max(tl.y, br.y) / step) * step;

  ctx.save();
  ctx.lineWidth = 1;
  ctx.font = "11px ui-monospace, monospace";
  for (let gx = xMin; gx <= xMax + 1e-6; gx += step) {
    const X = wx(gx);
    ctx.strokeStyle = Math.abs(gx) < 1e-9 ? "rgba(255,200,0,.40)" : "rgba(70,95,140,.30)";
    ctx.beginPath(); ctx.moveTo(X, 0); ctx.lineTo(X, H); ctx.stroke();
    ctx.fillStyle = "rgba(150,180,220,.65)";
    ctx.fillText(gx.toFixed(step < 1 ? 1 : 0), X + 3, 12);
  }
  for (let gy = yMin; gy <= yMax + 1e-6; gy += step) {
    const Y = wy(gy);
    ctx.strokeStyle = Math.abs(gy) < 1e-9 ? "rgba(255,200,0,.40)" : "rgba(70,95,140,.30)";
    ctx.beginPath(); ctx.moveTo(0, Y); ctx.lineTo(W, Y); ctx.stroke();
    ctx.fillStyle = "rgba(150,180,220,.65)";
    ctx.fillText(gy.toFixed(step < 1 ? 1 : 0), 3, Y - 3);
  }
  ctx.restore();
}

function drawStaticMap() {
  const m = state.staticMap, img = state.staticImg;
  if (!m || !img) return;
  const [ox, oy] = m.origin;
  const X = wx(ox);
  const Y = wy(oy + m.height * m.resolution);
  const w = m.width * m.resolution * state.view.scale;
  const h = m.height * m.resolution * state.view.scale;
  ctx.save();
  ctx.imageSmoothingEnabled = false;
  ctx.globalAlpha = 0.9;
  ctx.drawImage(img, X, Y, w, h);
  ctx.restore();
}

function drawPoints(W, H) {
  const pts = state.points;
  if (!pts || !pts.length) return;
  // map 系下只有 TF 新鲜时点云才是 map 坐标，否则跳过（避免错位）
  if (state.frame !== "odom" && !state.poseOk) return;
  const step = Math.max(1, Math.floor(pts.length / 6000));
  ctx.save();
  for (let i = 0; i < pts.length; i += step) {
    const p = pts[i];
    if (!p) continue;
    const x = wx(p[0]), y = wy(p[1]);
    if (x < -5 || y < -5 || x > W + 5 || y > H + 5) continue;
    ctx.fillStyle = colorByHeight(p[2]);
    ctx.fillRect(x, y, 2, 2);
  }
  ctx.restore();
}

function colorByHeight(z) {
  const t = Math.max(0, Math.min(1, ((Number(z) || 0) + 0.5) / 1.5));
  const r = Math.round(80 + t * 175);
  const g = Math.round(190 - t * 150);
  const b = Math.round(220 - t * 160);
  return `rgb(${r},${g},${b})`;
}

function drawWaypoints() {
  const wps = state.waypoints || [];
  ctx.save();
  ctx.font = "11px system-ui, sans-serif";
  for (const w of wps) {
    const X = wx(w.x), Y = wy(w.y);
    if (X < -40 || Y < -40 || X > cssW() + 40 || Y > cssH() + 40) continue;
    ctx.beginPath(); ctx.arc(X, Y, 6, 0, Math.PI * 2);
    ctx.fillStyle = "rgba(52,211,153,.28)"; ctx.fill();
    ctx.strokeStyle = "#34d399"; ctx.lineWidth = 1.6; ctx.stroke();
    ctx.fillStyle = "#9fe8c8";
    ctx.fillText(w.name || `点${w.id}`, X + 9, Y - 5);
  }
  ctx.restore();
}

function drawRobot() {
  let x, y, yaw;
  if (state.frame === "map") {
    if (!state.robotMap) return;   // 定位未就绪：不画，避免误导
    [x, y, yaw] = state.robotMap;
  } else {
    x = state.pos[0]; y = state.pos[1]; yaw = state.yaw;
  }
  const X = wx(x), Y = wy(y);
  ctx.save();
  ctx.beginPath(); ctx.arc(X, Y, 8, 0, Math.PI * 2);
  ctx.fillStyle = "rgba(74,158,255,.22)"; ctx.fill();
  ctx.strokeStyle = "#4a9eff"; ctx.lineWidth = 2; ctx.stroke();
  ctx.beginPath(); ctx.moveTo(X, Y);
  ctx.lineTo(X + 22 * Math.cos(-yaw), Y + 22 * Math.sin(-yaw));
  ctx.strokeStyle = "#4a9eff"; ctx.lineWidth = 2.4; ctx.stroke();
  ctx.restore();
}

function drawPickedPose() {
  const p = state.pickedPose;
  if (!p) return;
  const X = wx(p.x), Y = wy(p.y);
  const yaw = (parseFloat($("pose-yaw").value) || 0) * Math.PI / 180;
  ctx.save();
  ctx.strokeStyle = "#fbbf24"; ctx.lineWidth = 2.2;
  ctx.beginPath(); ctx.arc(X, Y, 9, 0, Math.PI * 2); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(X, Y);
  ctx.lineTo(X + 24 * Math.cos(-yaw), Y + 24 * Math.sin(-yaw)); ctx.stroke();
  ctx.fillStyle = "#fbbf24"; ctx.font = "11px ui-monospace, monospace";
  ctx.fillText(`(${p.x.toFixed(2)}, ${p.y.toFixed(2)})`, X + 11, Y + 16);
  ctx.restore();
}

function drawScaleBar(W, H) {
  const s = state.view.scale;
  let meters = 1;
  while (meters * s < 70) meters *= 2;
  while (meters * s > 260) meters /= 2;
  const x = W - 20 - meters * s, y = H - 26;
  ctx.save();
  ctx.strokeStyle = "#8b95a8"; ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(x, y); ctx.lineTo(x + meters * s, y);
  ctx.moveTo(x, y - 5); ctx.lineTo(x, y + 5);
  ctx.moveTo(x + meters * s, y - 5); ctx.lineTo(x + meters * s, y + 5);
  ctx.stroke();
  ctx.fillStyle = "#8b95a8"; ctx.font = "12px ui-monospace, monospace";
  ctx.fillText(meters + " m", x + meters * s / 2 - 10, y - 9);
  ctx.restore();
}

// ---------------- 画布交互 ----------------
function pointerPos(e) {
  const r = cv.getBoundingClientRect();
  return { x: e.clientX - r.left, y: e.clientY - r.top };
}

function zoomAt(px, py, factor) {
  const before = worldFromScreen(px, py);
  const next = Math.max(2, Math.min(120, state.view.scale * factor));
  if (next === state.view.scale) return;
  state.view.scale = next;
  state.view.cx = before.x - (px - cssW() / 2) / next;
  state.view.cy = before.y + (py - cssH() / 2) / next;
  draw();
}

cv.addEventListener("pointerdown", (e) => {
  cv.setPointerCapture(e.pointerId);
  state.pointers.set(e.pointerId, pointerPos(e));
  if (state.pointers.size === 1) {
    state.drag = { last: pointerPos(e), moved: 0 };
    cv.classList.add("dragging");
  } else if (state.pointers.size === 2) {
    state.drag = null;
    const [a, b] = [...state.pointers.values()];
    state.pinch = {
      dist: Math.hypot(a.x - b.x, a.y - b.y) || 1,
      mid: { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 },
    };
  }
});

cv.addEventListener("pointermove", (e) => {
  if (!state.pointers.has(e.pointerId)) return;
  state.pointers.set(e.pointerId, pointerPos(e));

  if (state.pointers.size >= 2 && state.pinch) {
    const [a, b] = [...state.pointers.values()];
    const dist = Math.hypot(a.x - b.x, a.y - b.y) || 1;
    const mid = { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
    const before = worldFromScreen(state.pinch.mid.x, state.pinch.mid.y);
    const factor = dist / state.pinch.dist;
    const next = Math.max(2, Math.min(120, state.view.scale * factor));
    state.view.scale = next;
    state.view.cx = before.x - (mid.x - cssW() / 2) / next;
    state.view.cy = before.y + (mid.y - cssH() / 2) / next;
    state.pinch = { dist, mid };
    draw();
    return;
  }

  if (!state.drag) return;
  const p = pointerPos(e);
  const dx = p.x - state.drag.last.x;
  const dy = p.y - state.drag.last.y;
  state.drag.moved += Math.abs(dx) + Math.abs(dy);
  state.drag.last = p;
  state.view.cx -= dx / state.view.scale;
  state.view.cy += dy / state.view.scale;
  draw();
});

function endPointer(e) {
  const p = pointerPos(e);
  const wasDrag = state.drag;
  state.pointers.delete(e.pointerId);
  if (state.pointers.size < 2) state.pinch = null;
  if (state.pointers.size === 0) {
    state.drag = null;
    cv.classList.remove("dragging");
  }
  // 点选模式：没怎么移动才算一次点击
  if (state.picking && wasDrag && wasDrag.moved < 8) {
    const w = worldFromScreen(p.x, p.y);
    state.pickedPose = { x: w.x, y: w.y };
    $("pose-picked").textContent = `已点选: (${w.x.toFixed(2)}, ${w.y.toFixed(2)})`;
    $("btn-apply-pose").disabled = false;
    draw();
  }
}

cv.addEventListener("pointerup", endPointer);
cv.addEventListener("pointercancel", (e) => {
  state.pointers.delete(e.pointerId);
  if (state.pointers.size < 2) state.pinch = null;
  if (state.pointers.size === 0) {
    state.drag = null;
    cv.classList.remove("dragging");
  }
});

cv.addEventListener("wheel", (e) => {
  e.preventDefault();
  const p = pointerPos(e);
  zoomAt(p.x, p.y, e.deltaY < 0 ? 1.12 : 1 / 1.12);
}, { passive: false });

cv.addEventListener("dblclick", () => fitView());

// ---------------- 控件 ----------------
$("btn-zoom-in").addEventListener("click", () => zoomAt(cssW() / 2, cssH() / 2, 1.3));
$("btn-zoom-out").addEventListener("click", () => zoomAt(cssW() / 2, cssH() / 2, 1 / 1.3));
$("btn-fit").addEventListener("click", () => fitView());

$("btn-pick").addEventListener("click", () => {
  state.picking = !state.picking;
  $("btn-pick").classList.toggle("active", state.picking);
  cv.classList.toggle("picking", state.picking);
  $("map-hint").textContent = state.picking
    ? "在地图上点狗的实际位置（再填朝向后应用）"
    : "拖拽平移 · 滚轮/双指缩放";
  if (!state.picking) {
    state.pickedPose = null;
    $("pose-picked").textContent = "尚未点选位置";
    $("btn-apply-pose").disabled = true;
    draw();
  }
});

$("btn-start-map").addEventListener("click", () => {
  if (!confirm("开始建图？\n\n会停止当前定位并启动建图进程，请确保场地内可安全走动。")) return;
  send({ type: "slam", op: "start_mapping" });
});

$("btn-save-map").addEventListener("click", () => {
  const name = ($("map-name").value || "venue").trim();
  if (!/^[A-Za-z0-9_-]+$/.test(name)) {
    log("地图名只能用字母、数字、- 和 _", "err");
    return;
  }
  if (!confirm(`完成建图并保存为「${name}」？\n\n保存约需 10-30 秒。`)) return;
  send({ type: "slam", op: "stop_mapping", name });
});

$("btn-discard-map").addEventListener("click", () => {
  if (!confirm("放弃本次建图？\n\n不会保存地图。")) return;
  send({ type: "slam", op: "discard_mapping" });
});

$("map-select").addEventListener("change", (e) => {
  const name = e.target.value;
  const s = state.slam || {};
  const localizing = s.mode === "localization" || s.mode === "starting";
  const runningMap = s.map || "";
  // 定位运行中切换/清空地图：后端会拒绝非当前地图，且 current_map 为空时
  // 还会回退到定位中的地图名。所以必须先停定位，切换才会真正生效。
  if (localizing && runningMap && name !== runningMap) {
    const what = name ? `切换到「${name}」` : "回到 odom 点位库";
    if (!confirm(`当前正在「${runningMap}」上重定位。\n\n${what}需要先停止重定位，继续？`)) {
      e.target.value = state.mapName || runningMap;
      return;
    }
    send({ type: "slam", op: "stop_localization" });
    setTimeout(() => send({ type: "slam", op: "select_map", name }), 700);
    return;
  }
  send({ type: "slam", op: "select_map", name });
});

$("btn-loc-start").addEventListener("click", () => {
  const name = $("map-select").value;
  if (!name) {
    log("请先选择要加载的地图", "err");
    return;
  }
  if (!confirm(`加载「${name}」并开始重定位？\n\n激光会扫描匹配该地图，修正里程计漂移。`)) return;
  send({ type: "slam", op: "start_localization", name });
});

$("btn-loc-stop").addEventListener("click", () => {
  if (!confirm("停止重定位？\n\n停止后点位仍保留，但机器人位姿不再被修正。")) return;
  send({ type: "slam", op: "stop_localization" });
});

$("btn-apply-pose").addEventListener("click", () => {
  const p = state.pickedPose;
  if (!p) return;
  const name = $("map-select").value || state.mapName || (state.slam && state.slam.map) || "";
  if (!name) {
    log("请先选择地图", "err");
    return;
  }
  const yaw = (parseFloat($("pose-yaw").value) || 0) * Math.PI / 180;
  if (!confirm(`按点选位置重新定位「${name}」？\n\n位置 (${p.x.toFixed(2)}, ${p.y.toFixed(2)})，朝向 ${(yaw * 180 / Math.PI).toFixed(0)}°`)) return;
  if (send({ type: "slam", op: "set_initial_pose", name, x: p.x, y: p.y, yaw })) {
    log("已发送初始位姿，正在重新定位…", "info");
  }
});

$("pose-yaw").addEventListener("input", () => draw());

// ---------------- 启动 ----------------
window.addEventListener("resize", resize);
resize();
connect();
// 主动取一次状态（init 包里没有 slam 字段）
setTimeout(() => send({ type: "slam", op: "status" }), 600);
