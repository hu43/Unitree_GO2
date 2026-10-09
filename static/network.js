/* 网络设置页：查看/切换 Jetson 的网络连接方式（机器狗热点 / 外部 WiFi） */
"use strict";

const $ = (id) => document.getElementById(id);
const state = { ws: null };

function connect() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${proto}://${location.host}/ws`);
  state.ws = ws;
  ws.onopen = () => send({ type: "net", op: "status" });
  ws.onclose = () => setTimeout(connect, 2000);
  ws.onmessage = (ev) => {
    let m; try { m = JSON.parse(ev.data); } catch { return; }
    if (m.type === "init") { if (m.net) applyNet(m.net, true); }
    else if (m.type === "state") { if (m.net) applyNet(m.net, false); }
    else if (m.type === "net") handleNet(m);
    else if (m.type === "ack") {
      if (m.error) setStatus("错误: " + m.error);
      else if (m.msg) setStatus(m.msg);
    }
  };
}

function send(obj) {
  if (state.ws && state.ws.readyState === WebSocket.OPEN) state.ws.send(JSON.stringify(obj));
}

function setStatus(t) { $("nw-status").textContent = t; }

function applyNet(n, syncRadio) {
  const mode = n.mode || "ap";
  $("nw-mode").textContent = mode === "ap" ? "机器狗AP直连" : "外部 WiFi";
  $("nw-ssid").textContent = n.wlan0_ssid || "-";
  $("nw-wstate").textContent = n.wlan0_state || "-";
  $("nw-robot-ip").textContent = n.robot_ip || "-";
  $("nw-jetson-ip").textContent = n.jetson_ip || "-";
  if (syncRadio) {
    const r = document.querySelector(`input[name="mode"][value="${mode}"]`);
    if (r) { r.checked = true; syncMode(); }
  }
}

function handleNet(m) {
  if (m.op === "status") {
    applyNet(m, true);
  } else if (m.op === "scan_result") {
    const sel = $("nw-ssid-list");
    if (Array.isArray(m.ssids) && m.ssids.length) {
      sel.innerHTML = "";
      m.ssids.forEach((s) => {
        const o = document.createElement("option");
        o.value = s.ssid;
        o.textContent = `${s.ssid}  (信号 ${s.signal || "?"}${s.security && s.security !== "--" ? " · " + s.security : ""})`;
        sel.appendChild(o);
      });
      sel.style.display = "";
      sel.onchange = () => { $("nw-ssid-input").value = sel.value; };
      setStatus(`扫描到 ${m.ssids.length} 个 WiFi，可在下拉框选择`);
    } else {
      setStatus("未扫描到 WiFi（或扫描失败）");
    }
  } else if (m.op === "switch_result") {
    if (m.ok) {
      setStatus("已切换：" + (m.mode === "ap" ? "机器狗AP直连" : "外部 WiFi"));
    } else {
      setStatus("切换失败: " + (m.error || "未知"));
    }
  } else if (m.op === "error") {
    setStatus("错误: " + (m.error || "未知"));
  }
}

// ---- mode radio ----
function syncMode() {
  const v = document.querySelector('input[name="mode"]:checked').value;
  $("opt-ap").classList.toggle("sel", v === "ap");
  $("opt-wifi").classList.toggle("sel", v === "wifi");
  $("wifi-fields").style.display = v === "wifi" ? "" : "none";
}
document.querySelectorAll('input[name="mode"]').forEach((r) => r.addEventListener("change", syncMode));

// ---- buttons ----
$("btn-scan").addEventListener("click", () => { setStatus("扫描中…"); send({ type: "net", op: "scan" }); });
$("btn-refresh").addEventListener("click", () => send({ type: "net", op: "status" }));

$("btn-apply").addEventListener("click", () => {
  const v = document.querySelector('input[name="mode"]:checked').value;
  if (v === "ap") {
    if (!confirm("切换到机器狗 AP 直连？\n\n会断开当前网络。若机器狗热点未开启，本机将失去网络。")) return;
    setStatus("正在切换到机器狗热点…");
    send({ type: "net", op: "set_mode", mode: "ap" });
  } else {
    const ssid = $("nw-ssid-input").value.trim();
    const pwd = $("nw-pass-input").value;
    if (!ssid) { setStatus("请填写 WiFi 名称"); return; }
    if (!confirm(`连接到 WiFi「${ssid}」？\n\n会断开当前网络。`)) return;
    setStatus("正在连接到 " + ssid + " …");
    send({ type: "net", op: "set_mode", mode: "wifi", ssid: ssid, password: pwd });
  }
});

syncMode();
connect();
