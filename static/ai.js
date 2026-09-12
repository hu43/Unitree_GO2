/* AI 控制界面：基础 AI 对话（Ollama run gemma3:4b 会话）+ 占位模块 */
"use strict";

const $ = (id) => document.getElementById(id);

const state = {
  ws: null,
  chatOpen: false,   // ollama 会话是否开启
  streamingEl: null, // 当前流式回复的 DOM 元素
};

// ---------------- WebSocket ----------------
function connect() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${proto}://${location.host}/ws`);
  state.ws = ws;
  ws.onclose = () => setTimeout(connect, 2000);
  ws.onmessage = (ev) => {
    let m; try { m = JSON.parse(ev.data); } catch { return; }
    if (m.type !== "ai") return;
    handleAi(m);
  };
}
connect();

function sendAi(obj) {
  if (state.ws && state.ws.readyState === WebSocket.OPEN) {
    state.ws.send(JSON.stringify({ type: "ai", ...obj }));
  }
}

function handleAi(m) {
  const box = $("chat-box");
  if (m.op === "output") {
    if (m.final) {
      // 流结束：若 final 消息本身带文本（非流式后端兼容），先补上
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
      // 流式块：追加到当前 AI 消息（没有则新建）
      if (!state.streamingEl) {
        state.streamingEl = document.createElement("div");
        state.streamingEl.className = "ai";
        box.appendChild(state.streamingEl);
      }
      state.streamingEl.textContent += (m.text || "");
      box.scrollTop = box.scrollHeight;
    }
  } else if (m.op === "started") {
    state.chatOpen = true;
    setUi(true);
    addSys("模型已启动（gemma3:4b），可以开始对话。支持发送图片分析。");
    $("ai-status").textContent = "对话中（输入 /bye 结束）";
  } else if (m.op === "ended") {
    state.chatOpen = false;
    setUi(false);
    addSys("对话已结束。");
    $("ai-status").textContent = "未启动";
  } else if (m.op === "saved") {
    addSys("已接收，AI 分析中…（本地推理需要一些时间）");
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
  $("btn-ai-start").disabled = open;
  $("btn-ai-end").disabled = !open;
  $("ai-input").disabled = !open;
  $("btn-ai-send").disabled = !open;
}

// ---------------- controls ----------------
$("btn-ai-start").addEventListener("click", () => {
  addSys("正在启动 Ollama 模型（gemma3:4b）…首次拉取需要数分钟，请耐心等待。");
  $("ai-status").textContent = "启动中…";
  sendAi({ op: "start", model: "gemma3:4b" });
});

$("btn-ai-end").addEventListener("click", () => {
  sendAi({ op: "end" });
});

function sendMsg() {
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

$("btn-ai-send").addEventListener("click", sendMsg);
$("ai-input").addEventListener("keydown", (e) => {
  if (e.key === "Enter") sendMsg();
  // Ctrl+D 结束对话
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

// keep the original setUi in sync
const _origSetUi = setUi;
setUi = function(open) {
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
    const b64 = reader.result.split(",")[1];  // strip data: prefix
    sendAi({ op: isVideo ? "video" : "image", data: b64, name: file.name || "camera.jpg" });
    $("ai-status").textContent = isVideo ? "视频分析中（抽帧）…" : "图片分析中…";
  };
  reader.readAsDataURL(file);
}

$("ai-file-cam").addEventListener("change", (e) => handleFile(e.target.files[0]));
$("ai-file-pick").addEventListener("change", (e) => handleFile(e.target.files[0]));
