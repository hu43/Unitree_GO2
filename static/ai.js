/* AI 控制界面
 * 模块1 基础对话（Ollama 流式） / 模块2 离线语音对话（ASR+TTS，可开关）
 * 模块3 智能 AI 控制（Agent 工具调用）
 */
"use strict";

const $ = (id) => document.getElementById(id);

const state = {
  ws: null,
  chatOpen: false,   // ollama 会话是否开启
  streamingEl: null, // 当前流式回复的 DOM 元素
  aiEnabled: true,
  agentEnabled: true,
  agentBusy: false,
  voiceBusy: false,
  micLive: false,
};

// ---------------- WebSocket ----------------
function connect() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${proto}://${location.host}/ws`);
  state.ws = ws;
  ws.onclose = () => setTimeout(connect, 2000);
  ws.onopen = () => {
    sendAi({ op: "models" });   // 刷新模型列表
    sendAi({ op: "status" });   // 刷新开关/语音状态
  };
  ws.onmessage = (ev) => {
    let m; try { m = JSON.parse(ev.data); } catch { return; }
    if (m.type === "ai") handleAi(m);
    else if (m.type === "voice") handleVoice(m);
    else if (m.type === "agent") handleAgent(m);
    else if (m.type === "init") applyInit(m);
    else if (m.type === "state") handleState(m);
    else if (m.type === "ack") handleAck(m);
  };
}
connect();

function sendMsg(type, obj) {
  if (state.ws && state.ws.readyState === WebSocket.OPEN) {
    state.ws.send(JSON.stringify({ type, ...obj }));
  }
}
const sendAi = (o) => sendMsg("ai", o);
const sendVoice = (o) => sendMsg("voice", o);
const sendAgent = (o) => sendMsg("agent", o);

// ---------------- 状态/开关 ----------------
function voiceStatusText(v) {
  const mic = v.mic_alive ? "麦克风在线" : "麦克风无数据";
  const asr = v.asr_ready ? "识别就绪"
    : (v.asr_available ? "识别待加载" : "识别未安装");
  const tts = v.tts_ready ? "离线合成就绪"
    : (v.tts_available ? "合成待加载" : "合成走 edge-tts");
  return `${mic} · ${asr} · ${tts}`;
}

function applyStatus(st) {
  if (!st) return;
  state.aiEnabled = !!st.enabled;
  state.agentEnabled = !!st.agent_enabled;
  const tg = $("ai-toggle");
  if (tg) tg.checked = state.aiEnabled;
  const tag = $("ai-switch-state");
  if (tag) {
    tag.textContent = state.aiEnabled ? "已开启" : "已关闭";
    tag.classList.toggle("ok", state.aiEnabled);
  }
  const v = st.voice || {};
  const vs = $("voice-status");
  if (vs) vs.textContent = voiceStatusText(v);
  if (typeof st.volume === "number" && $("vol-slider")) {
    $("vol-slider").value = st.volume;
    $("vol-val").textContent = st.volume + "%";
  }
  if (st.system_prompt !== undefined && $("ai-prompt")
      && document.activeElement !== $("ai-prompt")) {
    $("ai-prompt").value = st.system_prompt || "";
  }
  const a = st.agent || {};
  if (typeof a.max_speed === "number" && $("agent-speed")) {
    $("agent-speed").value = Math.round(a.max_speed * 100);
    $("agent-speed-val").textContent = a.max_speed.toFixed(2);
  }
  applyAiEnabled();
}

function applyAiEnabled() {
  const on = state.aiEnabled;
  const mic = $("btn-mic");
  if (mic) mic.disabled = !on;
  ["voice-input", "btn-voice-send", "agent-task", "btn-agent-start"]
    .forEach((id) => { const el = $(id); if (el) el.disabled = !on; });
  document.querySelectorAll("[data-quick]").forEach((b) => {
    b.disabled = !on;
  });
  const agentTag = $("agent-tag");
  if (agentTag) {
    agentTag.textContent = on ? "Agent · Ollama 工具调用" : "已关闭";
    agentTag.classList.toggle("ok", on);
  }
  if (!on) {
    $("btn-mic") && $("btn-mic").classList.remove("live");
    state.micLive = false;
  }
}

function applyInit(m) {
  if (m.ai) applyStatus({ ...m.ai.status, enabled: m.ai.enabled,
                          agent_enabled: m.ai.agent_enabled });
}

function handleState(m) {
  const L = m.lidar || {};
  const box = $("lidar-bars");
  if (box && (L.front !== undefined)) {
    const vals = box.querySelectorAll("b");
    const f = (x) => (x === undefined || x === null) ? "--" : Number(x).toFixed(2);
    if (vals.length === 4) {
      vals[0].textContent = f(L.front);
      vals[1].textContent = f(L.back);
      vals[2].textContent = f(L.left);
      vals[3].textContent = f(L.right);
    }
  }
}

function handleAck(m) {
  if (m.ok === false && m.error) {
    addAgentLog(m.error, "err");
    if (/AI|语音|模型|离线/.test(m.error)) {
      addVoiceLine("提示: " + m.error, "sys");
    }
  } else if (m.status) {
    applyStatus(m.status);
  } else if (typeof m.volume === "number") {
    $("vol-slider").value = m.volume;
    $("vol-val").textContent = m.volume + "%";
  } else if (m.voice) {
    const vs = $("voice-status");
    if (vs) vs.textContent = voiceStatusText(m.voice);
  }
}

// ---------------- 模块1：基础对话 ----------------
function handleAi(m) {
  const box = $("chat-box");
  if (m.op === "output") {
    if (m.final) {
      if (m.text) {
        if (!state.streamingEl) {
          state.streamingEl = document.createElement("div");
          state.streamingEl.className = "ai";
          box.appendChild(state.streamingEl);
        }
        state.streamingEl.textContent += m.text;
      }
      state.streamingEl = null;
      box.scrollTop = box.scrollHeight;
    } else {
      if (!state.streamingEl) {
        state.streamingEl = document.createElement("div");
        state.streamingEl.className = "ai";
        box.appendChild(state.streamingEl);
      }
      state.streamingEl.textContent += (m.text || "");
      box.scrollTop = box.scrollHeight;
    }
  } else if (m.op === "models") {
    const sel = $("ai-model");
    if (sel && Array.isArray(m.models)) {
      const cur = sel.value;
      sel.innerHTML = "";
      m.models.forEach((n) => {
        const o = document.createElement("option");
        o.value = n; o.textContent = n;
        sel.appendChild(o);
      });
      sel.value = (m.default && m.models.includes(m.default)) ? m.default : cur;
      if (!sel.value && sel.options.length) sel.selectedIndex = 0;
    }
  } else if (m.op === "started") {
    state.chatOpen = true;
    setUi(true);
    addSys("模型已启动（" + (m.model || "ollama") + "），可以开始对话。支持发送图片分析。");
    $("ai-status").textContent = "对话中（输入 /bye 结束）";
  } else if (m.op === "ended") {
    state.chatOpen = false;
    setUi(false);
    addSys("对话已结束。");
    $("ai-status").textContent = "未启动";
  } else if (m.op === "saved") {
    addSys("已接收，AI 分析中…（本地推理需要一些时间）");
  } else if (m.op === "enabled") {
    applyStatus({ ...m.status, enabled: m.enabled, agent_enabled: m.agent_enabled });
  } else if (m.op === "error") {
    addSys("错误: " + (m.error || "未知"));
    $("ai-status").textContent = "错误";
  }
}

function addSys(text) {
  const box = $("chat-box");
  const div = document.createElement("div");
  div.className = "sys";
  div.textContent = text;
  box.appendChild(div);
  box.scrollTop = box.scrollHeight;
}

function setUi(open) {
  $("ai-model").disabled = open;
  $("btn-ai-start").disabled = open;
  $("btn-ai-end").disabled = !open;
  $("ai-input").disabled = !open;
  $("btn-ai-send").disabled = !open;
}

// ---------------- controls ----------------
$("btn-ai-start").addEventListener("click", () => {
  const model = ($("ai-model") && $("ai-model").value) || "";
  addSys("正在启动 Ollama 模型（" + (model || "默认") + "）…");
  $("ai-status").textContent = "启动中…";
  sendAi({ op: "start", model: model });
});

$("btn-ai-end").addEventListener("click", () => {
  sendAi({ op: "end" });
});

function sendMsgText() {
  const input = $("ai-input");
  const text = input.value.trim();
  if (!text || !state.chatOpen) return;
  const box = $("chat-box");
  const div = document.createElement("div");
  div.className = "user";
  div.textContent = "我: " + text;
  box.appendChild(div);
  box.scrollTop = box.scrollHeight;
  input.value = "";
  sendAi({ op: "send", text: text });
}

$("btn-ai-send").addEventListener("click", sendMsgText);
$("ai-input").addEventListener("keydown", (e) => {
  if (e.key === "Enter") sendMsgText();
  if (e.key === "d" && e.ctrlKey) {
    e.preventDefault();
    sendAi({ op: "end" });
  }
});

// ---------------- image/video import ----------------
function setMediaUi(open) {
  $("btn-ai-photo").disabled = !open;
  $("btn-ai-file").disabled = !open;
}

const _origSetUi = setUi;
setUi = function (open) {
  _origSetUi(open);
  setMediaUi(open);
};
setMediaUi(false);

$("btn-ai-photo").addEventListener("click", () => $("ai-file-cam").click());
$("btn-ai-file").addEventListener("click", () => $("ai-file-pick").click());

function handleFile(file) {
  if (!file || !state.chatOpen) return;
  const isVideo = file.type.startsWith("video/");
  const maxBytes = isVideo ? 40 * 1024 * 1024 : 12 * 1024 * 1024;
  if (file.size > maxBytes) {
    addSys(`文件过大（${(file.size / 1048576).toFixed(1)}MB），上限 ${isVideo ? 40 : 12}MB`);
    return;
  }
  addSys(`正在上传${isVideo ? "视频" : "图片"}: ${file.name || "拍摄"}…`);
  const reader = new FileReader();
  reader.onload = () => {
    const b64 = reader.result.split(",")[1];
    sendAi({ op: isVideo ? "video" : "image", data: b64, name: file.name || "camera.jpg" });
    $("ai-status").textContent = isVideo ? "视频分析中（抽帧）…" : "图片分析中…";
  };
  reader.readAsDataURL(file);
}

$("ai-file-cam").addEventListener("change", (e) => handleFile(e.target.files[0]));
$("ai-file-pick").addEventListener("change", (e) => handleFile(e.target.files[0]));

// ---------------- 模块2：离线语音 ----------------
function addVoiceLine(text, cls) {
  const box = $("voice-box");
  const div = document.createElement("div");
  div.className = cls;
  div.textContent = text;
  box.appendChild(div);
  box.scrollTop = box.scrollHeight;
}

function setLevel(rms) {
  const pct = Math.max(0, Math.min(100, (rms / 5000) * 100));
  const el = $("level-fill");
  if (el) el.style.width = pct.toFixed(0) + "%";
}

function handleVoice(m) {
  const btnStop = $("btn-voice-stop");
  if (m.op === "listening") {
    state.voiceBusy = true;
    btnStop.disabled = false;
    $("voice-status").textContent = "🎤 正在听…（松开按钮或静音 0.8s 结束）";
  } else if (m.op === "level") {
    setLevel(m.rms || 0);
    $("voice-status").textContent =
      `🎤 正在听… ${(m.seconds || 0).toFixed(1)}s`;
  } else if (m.op === "asr") {
    setLevel(0);
    if (!m.text) {
      addVoiceLine("（没听清，再说一次）", "sys");
    } else {
      addVoiceLine("我: " + m.text, "user");
      $("voice-status").textContent = "识别完成，正在思考…";
    }
  } else if (m.op === "reply") {
    addVoiceLine("机器人: " + m.text, "ai");
    $("voice-status").textContent = "正在朗读…";
  } else if (m.op === "done") {
    state.voiceBusy = false;
    btnStop.disabled = true;
    $("voice-status").textContent = "空闲";
    setLevel(0);
  } else if (m.op === "error") {
    state.voiceBusy = false;
    btnStop.disabled = true;
    addVoiceLine("错误: " + (m.error || "未知"), "sys");
    $("voice-status").textContent = "错误";
  }
}

const micBtn = $("btn-mic");
function startListen(e) {
  if (e) e.preventDefault();
  if (!state.aiEnabled || state.micLive) return;
  state.micLive = true;
  micBtn.classList.add("live");
  micBtn.textContent = "🎙️ 松开结束";
  setLevel(0);
  sendVoice({ op: "listen_start" });
}
function stopListen() {
  if (!state.micLive) return;
  state.micLive = false;
  micBtn.classList.remove("live");
  micBtn.textContent = "🎤 按住说话";
  sendVoice({ op: "listen_stop" });
}
micBtn.addEventListener("pointerdown", startListen);
["pointerup", "pointerleave", "pointercancel"].forEach((ev) =>
  micBtn.addEventListener(ev, stopListen));
// 键盘可访问：空格按住
micBtn.addEventListener("keydown", (e) => { if (e.code === "Space") startListen(e); });
micBtn.addEventListener("keyup", (e) => { if (e.code === "Space") stopListen(e); });

function sendVoiceText() {
  const el = $("voice-input");
  const text = el.value.trim();
  if (!text || !state.aiEnabled) return;
  el.value = "";
  addVoiceLine("我: " + text, "user");
  sendVoice({ op: "chat", text: text });
}
$("btn-voice-send").addEventListener("click", sendVoiceText);
$("voice-input").addEventListener("keydown", (e) => {
  if (e.key === "Enter") sendVoiceText();
});
$("btn-voice-stop").addEventListener("click", () => {
  stopListen();
  sendVoice({ op: "stop" });
  state.voiceBusy = false;
  $("btn-voice-stop").disabled = true;
  $("voice-status").textContent = "已停止";
});

// 总开关
$("ai-toggle").addEventListener("change", (e) => {
  const on = e.target.checked;
  sendAi({ op: "set_enabled", enabled: on });
  state.aiEnabled = on;
  applyAiEnabled();
});

// 音量滑块：拖动即时显示，松手保存
const volEl = $("vol-slider");
volEl.addEventListener("input", () => {
  $("vol-val").textContent = volEl.value + "%";
});
volEl.addEventListener("change", () => {
  sendVoice({ op: "set_volume", value: Number(volEl.value) });
});

// 人设保存
$("btn-ai-prompt-save").addEventListener("click", () => {
  sendAi({ op: "set_prompt", text: $("ai-prompt").value });
  $("ai-prompt-status").textContent = "已保存";
  setTimeout(() => { $("ai-prompt-status").textContent = ""; }, 2000);
});

// ---------------- 模块3：Agent ----------------
function addAgentLog(text, cls) {
  const box = $("agent-log");
  const div = document.createElement("div");
  div.className = cls || "step";
  div.textContent = text;
  box.appendChild(div);
  box.scrollTop = box.scrollHeight;
  while (box.childElementCount > 200) box.removeChild(box.firstChild);
}

function setAgentBusy(busy) {
  state.agentBusy = busy;
  $("btn-agent-start").disabled = busy || !state.aiEnabled;
  $("btn-agent-stop").disabled = !busy;
  $("agent-task").disabled = busy || !state.aiEnabled;
  $("agent-status").textContent = busy ? "执行中…" : "空闲";
}

function handleAgent(m) {
  if (m.op === "started") {
    $("agent-log").innerHTML = "";
    setAgentBusy(true);
    addAgentLog("任务: " + m.task, "say");
  } else if (m.op === "thinking") {
    addAgentLog(`· 第 ${m.step}/${m.max} 步：思考中…`, "step");
  } else if (m.op === "tool") {
    const args = m.args && Object.keys(m.args).length
      ? " " + JSON.stringify(m.args) : "";
    addAgentLog(`🔧 ${m.name}${args}`, "tool");
  } else if (m.op === "result") {
    addAgentLog("→ " + m.result, "ok");
  } else if (m.op === "assistant") {
    addAgentLog("💬 " + m.text, "say");
  } else if (m.op === "log") {
    addAgentLog("· " + m.text, "step");
  } else if (m.op === "frame") {
    $("agent-frame").src = "data:image/jpeg;base64," + m.b64;
  } else if (m.op === "done") {
    addAgentLog(`✅ 完成（${m.steps} 步，${m.seconds}s）：${m.text}`, "ok");
    setAgentBusy(false);
    $("agent-status").textContent = "完成";
  } else if (m.op === "error") {
    addAgentLog("❌ " + m.error, "err");
    setAgentBusy(false);
    $("agent-status").textContent = "出错";
  }
}

$("btn-agent-start").addEventListener("click", () => {
  const task = $("agent-task").value.trim();
  if (!task) return;
  addAgentLog("启动任务: " + task, "say");
  sendAgent({ op: "start", task: task });
});

$("btn-agent-stop").addEventListener("click", () => {
  sendAgent({ op: "stop" });
  addAgentLog("已请求停止", "err");
  setAgentBusy(false);
});

document.querySelectorAll(".chip").forEach((b) => {
  b.addEventListener("click", () => {
    $("agent-task").value = b.dataset.task;
  });
});

document.querySelectorAll("[data-quick]").forEach((b) => {
  b.addEventListener("click", () => {
    sendAgent({ op: "quick", action: b.dataset.quick });
    addAgentLog("快捷动作: " + b.textContent.trim(), "step");
  });
});

const speedEl = $("agent-speed");
speedEl.addEventListener("input", () => {
  $("agent-speed-val").textContent = (speedEl.value / 100).toFixed(2);
});
speedEl.addEventListener("change", () => {
  sendAgent({ op: "config", key: "max_speed", value: speedEl.value / 100 });
});

// 执行中每 2 秒刷新一次相机缩略图
setInterval(() => {
  if (state.agentBusy) {
    $("agent-frame").src = "/api/ai/frame.jpg?t=" + Date.now();
  }
}, 2000);

applyAiEnabled();
