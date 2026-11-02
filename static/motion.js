/* Go2 运动模式独立页面：复用控制台的 WebSocket 协议 */
"use strict";

const $ = (id) => document.getElementById(id);

const MODE_NAMES = {
  0: "站立", 1: "平衡站", 2: "摆姿势", 3: "运动", 4: "保留",
  5: "趴下", 6: "关节锁定", 7: "阻尼", 8: "恢复站立", 9: "保留",
  10: "坐下", 11: "前空翻", 12: "前跳", 13: "前扑",
};
const GAIT_NAMES = {
  0: "idle", 1: "trot", 2: "run", 3: "爬楼(正向)", 4: "下楼(逆向)", 9: "adjust",
};
const MODE_LABELS = {
  walkupright: "后腿直立", crossstep: "交叉步", pose: "摆姿势",
  freebound: "并腿跑", freejump: "跳跃", freeavoid: "闪避",
  classicwalk: "经典步态",
};

const state = {
  ws: null,
  motion: {},
  activeModes: [],
  freewalk: false,
  busy: false,
  maxSpeed: 0.5,
  joyActive: false,
  joyDx: 0,
  joyDy: 0,
  joyWasActive: false,
  joyTimer: null,
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
    joyEnd();  // 断线时把摇杆复位，别让复位的速度指令排队等重连
    setTimeout(connect, 2000);
  };
  state.ws.onerror = () => state.ws.close();
  state.ws.onmessage = (ev) => {
    let msg;
    try { msg = JSON.parse(ev.data); } catch { return; }
    if (msg.type === "state") {
      if (msg.motion) {
        state.motion = msg.motion;
        if (Array.isArray(msg.motion.active_modes)) {
          state.activeModes = msg.motion.active_modes;
          paintActive();
        }
        if (msg.motion.freewalk !== undefined) {
          state.freewalk = !!msg.motion.freewalk;
          paintFreewalk();
        }
        paintStatus();
      }
    } else if (msg.type === "init") {
      // 速度上限由后端配置决定，和导览控制台保持一致
      if (msg.config && msg.config.max_speed != null) {
        state.maxSpeed = Number(msg.config.max_speed);
        paintSpeed();
      }
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

// ---------------- status ----------------
function paintStatus() {
  const m = state.motion || {};
  $("mt-mode").textContent = MODE_NAMES[m.mode] ?? (m.mode ?? "-");
  $("mt-gait").textContent = GAIT_NAMES[m.gait_type] ?? (m.gait_type ?? "-");
  $("mt-bh").textContent = m.body_height != null
    ? `${Number(m.body_height).toFixed(3)} m` : "-";

  const errEl = $("mt-err");
  const code = m.error_code;
  errEl.textContent = code == null ? "-" : code;
  // 100 = 正常；其余是固件错误（如 2017/2019/2021 表示该模式切换被拒）
  errEl.className = "v" + (code == null || code === 100 ? "" : " warn");

  const active = state.activeModes || [];
  $("mt-active").textContent = active.length
    ? active.map((n) => MODE_LABELS[n] || n).join("、") : "无";
}

function paintActive() {
  const active = new Set(state.activeModes || []);
  document.querySelectorAll(".onoff").forEach((btn) => {
    btn.classList.toggle("on", active.has(btn.dataset.mode));
  });
}

function paintFreewalk() {
  const btn = $("btn-stair");
  btn.classList.toggle("on", state.freewalk);
  btn.textContent = state.freewalk ? "🌊 退出灵动模式" : "🌊 开启灵动模式";
  btn.title = state.freewalk
    ? "当前处于灵动模式，点击退回普通站立步态"
    : "进入自由行走步态，可直接上下楼梯";
}

// ---------------- log ----------------
function log(text, cls) {
  const box = $("mt-log");
  const line = document.createElement("div");
  line.className = cls || "";
  const t = new Date().toLocaleTimeString("zh-CN", { hour12: false });
  line.textContent = `[${t}] ${text}`;
  box.appendChild(line);
  box.scrollTop = box.scrollHeight;
  while (box.childElementCount > 60) box.removeChild(box.firstChild);
}

function releaseButtons() {
  state.busy = false;
  document.querySelectorAll(".mt-btn").forEach((b) => { b.disabled = false; });
}

// ---------------- ack ----------------
function handleAck(msg) {
  // 摇杆的 cmd_vel 回执只有 {ok:true}：10Hz 的这类回执不该触发按钮解锁/日志
  if (msg.ok !== false && msg.action === undefined && msg.msg === undefined
      && msg.closed === undefined && msg.enabled === undefined
      && msg.error === undefined) {
    return;
  }
  releaseButtons();

  if (msg.ok === false) {
    log(msg.error || "指令失败", "err");
    return;
  }
  if (msg.closed) {
    log(msg.msg || "已退出所有模式", "ok");
    state.activeModes = [];
    state.freewalk = false;
    paintActive();
    paintFreewalk();
    return;
  }
  if (msg.enabled !== undefined && msg.api_id != null) {
    state.freewalk = !!msg.enabled;
    paintFreewalk();
  }
  if (msg.msg) log(msg.msg, "ok");
  else if (msg.action) log(`${msg.action} 已执行`, "ok");
}

// ---------------- events ----------------
$("btn-stair").addEventListener("click", () => {
  send({ type: "motion", op: "freewalk", enable: !state.freewalk });
});

document.querySelectorAll("[data-sport]").forEach((btn) => {
  btn.addEventListener("click", () => {
    send({ type: "sport", action: btn.dataset.sport });
  });
});

// 阻尼急停只让机器人进入阻尼，不杀导航栈（那是主控制台的「急停」按钮）
document.querySelectorAll("[data-act='damp']").forEach((btn) => {
  btn.addEventListener("click", () => {
    if (!confirm("阻尼急停会让机器狗立即失去支撑，确定？")) return;
    send({ type: "sport", action: "damp" });
  });
});

document.querySelectorAll("[data-action]").forEach((btn) => {
  btn.addEventListener("click", () => {
    if (state.busy) { log("上一个指令还在执行", "info"); return; }
    state.busy = true;
    btn.disabled = true;
    send({ type: "motion", op: "action", name: btn.dataset.action });
  });
});

document.querySelectorAll(".onoff").forEach((btn) => {
  btn.addEventListener("click", () => {
    const mode = btn.dataset.mode;
    const enable = !(state.activeModes || []).includes(mode);
    send({ type: "motion", op: "action", name: mode, enable });
  });
});

$("btn-exitall").addEventListener("click", () => {
  send({ type: "motion", op: "exit_all" });
});

// ---------------- 遥控轮盘 ----------------
// 与导览控制台（app.js）同一套映射：摇杆归一化坐标 -> sport Move(x, y, z)。
const joy = $("joystick");
const knob = $("stick-knob");

function paintSpeed() {
  $("speed-val").textContent = state.maxSpeed.toFixed(2);
  const slider = $("speed-slider");
  if (slider && Math.abs(parseFloat(slider.value) - state.maxSpeed) > 0.001) {
    slider.value = String(state.maxSpeed);
  }
}

function velFromStick() {
  return {
    vx: -state.joyDy * state.maxSpeed,        // 上推 = 前进
    vy: -state.joyDx * state.maxSpeed * 0.6,  // 左推 = 左平移
    vyaw: -state.joyDx * 0.8,                 // 右推 = 右转
  };
}

function sendJoystick() {
  if (!state.joyActive && !state.joyWasActive) return;
  state.joyWasActive = state.joyActive;
  const v = velFromStick();
  send({ type: "cmd_vel", vx: v.vx, vy: v.vy, vyaw: v.vyaw });
  $("joy-readout").textContent =
    `vx ${v.vx.toFixed(2)} · vy ${v.vy.toFixed(2)} · vyaw ${v.vyaw.toFixed(2)}`;
}

// 手指按住不动时不会有 pointermove，但固件里的 Move 会衰减到停，
// 所以按住期间按 10Hz 重发当前速度；松手立刻发 0 并停掉定时器。
function startJoyTimer() {
  stopJoyTimer();
  state.joyTimer = setInterval(() => {
    // 断线时不能继续发旧速度：重连后会把过期的摇杆值再推出去
    if (!state.ws || state.ws.readyState !== WebSocket.OPEN) { joyEnd(); return; }
    sendJoystick();
  }, 100);
}
function stopJoyTimer() {
  if (state.joyTimer) {
    clearInterval(state.joyTimer);
    state.joyTimer = null;
  }
}

function joyStart(e) {
  state.joyActive = true;
  // setPointerCapture 在部分浏览器/环境下会抛（指针已释放、DOM 状态特殊），
  // 抛了就不该让整个按下流程中断，否则轮盘看起来按下去但完全没反应。
  try { joy.setPointerCapture(e.pointerId); } catch (err) { /* 退化为普通拖动 */ }
  joyMove(e);
  startJoyTimer();
}

function joyMove(e) {
  if (!state.joyActive) return;
  const rect = joy.getBoundingClientRect();
  let dx = e.clientX - (rect.left + rect.width / 2);
  let dy = e.clientY - (rect.top + rect.height / 2);
  const max = rect.width / 2 - 14;
  const d = Math.hypot(dx, dy);
  if (d > max) { dx = dx / d * max; dy = dy / d * max; }
  knob.style.left = `calc(50% + ${dx}px)`;
  knob.style.top = `calc(50% + ${dy}px)`;
  state.joyDx = dx / max;
  state.joyDy = dy / max;
  sendJoystick();
}

function joyEnd() {
  if (!state.joyActive) return;
  state.joyActive = false;
  state.joyDx = 0;
  state.joyDy = 0;
  knob.style.left = "50%";
  knob.style.top = "50%";
  stopJoyTimer();
  sendJoystick();
}

joy.addEventListener("pointerdown", joyStart);
joy.addEventListener("pointermove", joyMove);
joy.addEventListener("pointerup", joyEnd);
joy.addEventListener("pointercancel", joyEnd);
// 指针被系统抢占（来电、切应用）时也要保证停下来
window.addEventListener("blur", joyEnd);

$("speed-slider").addEventListener("input", (e) => {
  state.maxSpeed = parseFloat(e.target.value);
  paintSpeed();
});

$("btn-stopmove").addEventListener("click", () => {
  joyEnd();
  send({ type: "sport", action: "stopmove" });
  log("停止移动", "ok");
});

connect();
paintFreewalk();
paintSpeed();
