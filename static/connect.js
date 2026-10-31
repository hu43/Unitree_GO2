// 连接机器人向导 —— 逻辑对齐 APP-G1 的 connect.vue
//
// 传输层有三种，按可用性自动选：
//   1. 浏览器蓝牙（Web Bluetooth / BLE）—— 手机直接连机器人，不依赖 HTTP 通路。
//      切 WiFi / 开 AP 时 HTTP 会断，BLE 走另一条射频不会断，所以最稳。
//      前提：页面必须是 https://（navigator.bluetooth 只在安全上下文里存在）。
//   2. 面板后端 /api/net/* —— 由宿主机的 go2-netd 执行（有 SPP 会话就转 {"cmd":...}，
//      没有就直接用本机 nmcli）。面板由机器人自己提供时这就是"本机通道"。
//   3. App 版用经典蓝牙 SPP 直连（本页面不涉及）。
const $ = (id) => document.getElementById(id);

// 参考实现里默认填的是 G1 的 MAC（94:BA:06:F8:23:B6）。Go2 上那个地址没有意义，
// 只会让人以为填对了，所以这里只从上次保存的记录里恢复，没有就留空。
const LS_MAC = 'go2_bt_mac';
const LS_IP = 'go2_server_ip';
const LS_HIST = 'go2_ip_history';

const S = {
  mode: '',            // '' | 'wifi' | 'ap' | 'manual'
  step: 1,
  btMac: '',
  adapterAddr: '',     // 机器人自己的蓝牙地址（方式 B 不能连自己）
  btConnected: false,  // 是否已建立到机器人的蓝牙 SPP 会话
  ble: null,           // Web Bluetooth 会话（浏览器直连机器人）
  devices: [],
  aps: [],
  selectedSsid: '',
  robotIp: '',
  connectedSsid: '',
  busy: false,
};

// ------------------------------------------------------------ 基础工具

function addLog(type, msg) {
  const d = new Date();
  const t = [d.getHours(), d.getMinutes(), d.getSeconds()]
    .map((n) => String(n).padStart(2, '0')).join(':');
  const item = document.createElement('div');
  item.className = 'log-item log-' + type;
  item.innerHTML = '<span class="log-time"></span><span class="log-msg"></span>';
  item.querySelector('.log-time').textContent = t;
  item.querySelector('.log-msg').textContent = msg;
  const area = $('log-area');
  area.appendChild(item);
  while (area.children.length > 100) area.removeChild(area.firstChild);
  area.scrollTop = area.scrollHeight;
  $('card-log').hidden = false;
}

let toastTimer = null;
function toast(msg, ms = 2600) {
  $('toast-text').textContent = msg;
  $('toast').hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { $('toast').hidden = true; }, ms);
}

async function api(sub, params) {
  const qs = params ? '?' + new URLSearchParams(params).toString() : '';
  const r = await fetch('/api/net' + sub + qs, { cache: 'no-store' });
  let data;
  try {
    data = await r.json();
  } catch (e) {
    throw new Error('后端返回异常（HTTP ' + r.status + '）');
  }
  return data;
}

// 统一取 IP：蓝牙 daemon 和本机后端都返回 host_ip，个别接口给 ip
const ipOf = (r) => (r && (r.ip || r.host_ip)) || '';

// ------------------------------------------------------------ 传输层选择
// 有蓝牙会话 -> 走 {"cmd":...} 协议；否则直接用本机后端

// Web Bluetooth 的 UUID（与机器人侧 go2-ble.py 保持一致，Nordic UART Service）
const BLE_SERVICE = '6e400001-b5a3-f393-e0a9-e50e24dcca9e';
const BLE_RX = '6e400002-b5a3-f393-e0a9-e50e24dcca9e';  // 写：命令
const BLE_TX = '6e400003-b5a3-f393-e0a9-e50e24dcca9e';  // 通知：响应

async function netStatus() {
  if (S.ble) return bleCmd({ cmd: 'status' });
  return S.btConnected ? api('/bt/cmd', { cmd: 'status' }) : api('/status');
}
async function netScanWifi() {
  if (S.ble) return bleCmd({ cmd: 'scan' }, 40000);
  return S.btConnected ? api('/bt/cmd', { cmd: 'scan' }) : api('/wifi/scan');
}
async function netConnectWifi(ssid, psk) {
  if (S.ble) return bleCmd({ cmd: 'connect', ssid, psk }, 120000);
  return S.btConnected
    ? api('/bt/cmd', { cmd: 'connect', ssid, psk })
    : api('/wifi/connect', { ssid, psk });
}
async function netOpenAp() {
  if (S.ble) return bleCmd({ cmd: 'ap' }, 60000);
  return S.btConnected ? api('/bt/cmd', { cmd: 'ap' }) : api('/ap/on');
}

// ------------------------------------------------------------ 浏览器蓝牙（BLE）

function bleSupported() {
  return !!(navigator.bluetooth && window.isSecureContext);
}

// 机器人侧 BLE 的 MTU 默认 23（载荷 20 字节），收发两端都要按 20 字节切片。
const BLE_CHUNK = 20;

async function bleConnect() {
  if (!navigator.bluetooth) {
    throw new Error(window.isSecureContext
      ? '当前浏览器不支持 Web Bluetooth（iOS Safari 不支持，请用 Android Chrome）'
      : '需要 HTTPS 才能用浏览器蓝牙，请改用 https:// 地址打开本页');
  }
  addLog('info', '正在请求蓝牙设备（浏览器会弹选择框）…');
  const device = await navigator.bluetooth.requestDevice({
    filters: [{ services: [BLE_SERVICE] }],
    optionalServices: [BLE_SERVICE],
  });
  addLog('info', '已选择：' + (device.name || device.id) + '，正在连接…');
  device.addEventListener('gattserverdisconnected', () => {
    addLog('warn', '浏览器蓝牙已断开');
    S.ble = null;
    paintBleState();
  });

  const server = await device.gatt.connect();
  const svc = await server.getPrimaryService(BLE_SERVICE);
  const rx = await svc.getCharacteristic(BLE_RX);
  const tx = await svc.getCharacteristic(BLE_TX);

  const session = { device, rx, tx, buf: [], waiters: [] };
  await tx.startNotifications();
  tx.addEventListener('characteristicvaluechanged', (ev) => {
    const dv = ev.target.value;
    for (let i = 0; i < dv.byteLength; i++) {
      const byte = dv.getUint8(i);
      if (byte === 0x0a) {
        const raw = new TextDecoder('utf-8', { fatal: false }).decode(new Uint8Array(session.buf));
        session.buf = [];
        const w = session.waiters.shift();
        if (w) { clearTimeout(w.timer); w.resolve(raw); }
      } else {
        session.buf.push(byte);
      }
    }
  });

  S.ble = session;
  paintBleState();
  return device;
}

// 发一行 JSON，等一行 JSON 回来
function bleCmd(obj, timeoutMs = 20000) {
  const s = S.ble;
  if (!s) return Promise.reject(new Error('浏览器蓝牙未连接'));
  const text = JSON.stringify(obj) + '\n';
  const bytes = new TextEncoder().encode(text);
  const done = new Promise((resolve, reject) => {
    const timer = setTimeout(() => {
      const i = s.waiters.findIndex((w) => w.timer === timer);
      if (i >= 0) s.waiters.splice(i, 1);
      reject(new Error('蓝牙响应超时（' + Math.round(timeoutMs / 1000) + 's）'));
    }, timeoutMs);
    s.waiters.push({ resolve, reject, timer });
  });
  (async () => {
    for (let i = 0; i < bytes.length; i += BLE_CHUNK) {
      const part = bytes.slice(i, i + BLE_CHUNK);
      if (s.rx.writeValueWithoutResponse) {
        await s.rx.writeValueWithoutResponse(part);
      } else {
        await s.rx.writeValue(part);
      }
    }
  })().catch((e) => {
    const w = s.waiters.shift();
    if (w) { clearTimeout(w.timer); w.reject(e); }
  });
  return done.then((raw) => {
    let data;
    try { data = JSON.parse(raw); } catch (e) { throw new Error('蓝牙返回格式异常: ' + raw.slice(0, 80)); }
    return data;
  });
}

function bleDisconnect() {
  if (!S.ble) return;
  try { S.ble.device.gatt.disconnect(); } catch (e) {}
  S.ble = null;
  paintBleState();
}

function paintBleState() {
  const box = $('ble-state');
  if (!box) return;
  if (S.ble) {
    box.innerHTML = '<span class="ok-txt">● 已通过浏览器蓝牙连接 ' +
      (S.ble.device.name || '') + '</span>';
  } else if (!bleSupported()) {
    box.innerHTML = '<span class="warn-txt">当前环境用不了浏览器蓝牙' +
      (window.isSecureContext ? '（浏览器不支持，请用 Android Chrome）' : '（需要 HTTPS 地址）') +
      '</span><br><span class="bt-hint" style="margin:4px 0 0">当前 UA：' +
      navigator.userAgent.replace(/[<>&]/g, '') + '</span>';
  } else {
    box.textContent = '可用：手机直接连机器人 BLE，不依赖当前网络';
  }
}

// 诊断：列出所有 BLE 设备（不按服务过滤）。
// 用来区分是"手机/权限根本扫不到设备"还是"按 UUID 过滤没匹配上"。
async function bleAnyScan() {
  if (!bleSupported()) throw new Error('当前环境不支持浏览器蓝牙');
  const device = await navigator.bluetooth.requestDevice({
    acceptAllDevices: true,
    optionalServices: [BLE_SERVICE],
  });
  addLog('info', '诊断：选中 ' + (device.name || '(无名)') + ' / ' + device.id);
  const server = await device.gatt.connect();
  const svcs = await server.getPrimaryServices();
  const uuids = svcs.map((s) => String(s.uuid).toLowerCase());
  addLog('info', '诊断：该设备服务 ' + (uuids.join(', ') || '（空）'));
  if (uuids.indexOf(BLE_SERVICE) >= 0) {
    addLog('success', '诊断：包含机器人 BLE 服务，链路正常，可直接连');
  } else {
    addLog('warn', '诊断：该设备不含 6e400001 服务，连错了设备');
  }
  try { device.gatt.disconnect(); } catch (e) {}
  return device;
}

// ------------------------------------------------------------ 步骤/卡片切换

function paint() {
  const inWizard = S.mode === 'wifi' || S.mode === 'ap';
  $('step-bar').hidden = !inWizard;
  $('step-2-text').textContent = S.mode === 'ap' ? 'AP' : 'WiFi';
  for (const n of [1, 2, 3]) {
    $('step-' + n).classList.toggle('active', S.step >= n);
    if (n < 3) $('line-' + n).classList.toggle('active', S.step >= n + 1);
  }
  $('card-mode').hidden = S.mode !== '';
  $('card-bt').hidden = !(inWizard && S.step === 1);
  $('card-wifi').hidden = !(S.mode === 'wifi' && S.step === 2);
  $('card-ap').hidden = !(S.mode === 'ap' && S.step === 2);
  $('card-done').hidden = !(inWizard && S.step === 3);
  $('card-manual').hidden = S.mode !== 'manual';
}

function setBusy(on) {
  S.busy = on;
  document.querySelectorAll('.btn').forEach((b) => {
    b.disabled = on || (b.id === 'btn-wifi-connect' && !S.selectedSsid);
  });
}

function resetWizard() {
  S.mode = ''; S.step = 1; S.devices = []; S.aps = [];
  S.selectedSsid = ''; S.robotIp = ''; S.connectedSsid = '';
  $('bt-list').innerHTML = '';
  $('wifi-list').innerHTML = '';
  $('wifi-psk-row').hidden = true;
  $('wifi-psk').value = '';
  $('btn-wifi-connect').disabled = true;
  paint();
}

// ------------------------------------------------------------ 步骤 1：蓝牙

function renderDevices(list) {
  const box = $('bt-list');
  box.innerHTML = '';
  if (!list.length) {
    box.innerHTML = '<div class="info-box">没有找到蓝牙设备。机器人蓝牙需处于可发现状态；已配对设备可直接在上方列表里选。</div>';
    return;
  }
  const nearby = list.filter((d) => !d.bonded);
  const bonded = list.filter((d) => d.bonded);
  const add = (title, arr) => {
    if (!arr.length) return;
    const t = document.createElement('span');
    t.className = 'list-title';
    t.textContent = title;
    box.appendChild(t);
    arr.forEach((d) => {
      const el = document.createElement('div');
      el.className = 'dev-item' + (d.address === S.btMac ? ' active' : '');
      el.innerHTML = '<span class="dev-name"></span><span class="dev-meta"></span>';
      el.querySelector('.dev-name').textContent = d.name;
      const meta = [d.address];
      if (d.bonded) meta.push('已配对');
      meta.push(typeof d.rssi === 'number' ? '信号 ' + d.rssi + ' dBm' : '本次未扫到');
      el.querySelector('.dev-meta').textContent = meta.join(' · ');
      el.addEventListener('click', () => {
        S.btMac = d.address;
        $('bt-mac').value = d.address;
        renderDevices(S.devices);
        addLog('info', '已选择 ' + d.name + ' (' + d.address + ')');
      });
      box.appendChild(el);
    });
  };
  add('附近设备', nearby);
  add('已配对设备', bonded);
}

async function loadLocalChannel() {
  try {
    const r = await api('/status');
    if (!r.ok) throw new Error(r.msg || '未知错误');
    const ip = ipOf(r);
    S.localIp = ip;
    $('local-ip').textContent = ip || '未获取到 IP';
    const mode = r.wlan_mode === 'ap' ? 'AP 热点' : (r.wlan_mode === 'wifi' ? 'WiFi' : '未联网');
    $('local-ssid').textContent = '网络：' + (r.ssid || '-') + '　模式：' + mode;
    if (r.ap_ssid) $('ap-ssid').textContent = r.ap_ssid;
    if (r.ap_psk) $('ap-psk').textContent = r.ap_psk;
    // 已经处在热点模式时，才给「关热点恢复 WiFi」的出口
    $('btn-ap-off').hidden = r.wlan_mode !== 'ap';
  } catch (e) {
    $('local-ip').textContent = '网络配置服务不可用';
    $('local-ssid').textContent = e.message || '';
  }
}

// 本机蓝牙适配器状态：手机在系统蓝牙里搜不到机器，原因通常就在这一行
async function loadBtAdapter() {
  const box = $('bt-adapter');
  box.textContent = '本机蓝牙：检测中…';
  try {
    const r = await api('/bt/info');
    const ad = (r && r.adapter) || {};
    if (!r.ok || !ad.address) {
      box.innerHTML = '<span class="warn-txt">本机没有可用的蓝牙适配器</span>';
      return;
    }
    const flags = [ad.powered ? '已开启' : '未开启',
                   ad.discoverable ? '可被发现' : '不可被发现',
                   ad.pairable ? '可配对' : '不可配对'];
    S.adapterAddr = String(ad.address || '').toUpperCase();
    box.innerHTML = '本机适配器 <b>' + (ad.name || '-') + '</b>（' + ad.address + '）：' +
      flags.join('、') + '，已配对 ' + (r.paired || 0) + ' 个';
    if (!ad.discoverable) {
      box.innerHTML += '<br><span class="warn-txt">手机在系统蓝牙里搜不到这台机器，就是因为它不可被发现。</span>';
    }
  } catch (e) {
    box.textContent = '蓝牙状态读取失败：' + (e.message || e);
  }
}

async function scanBt() {
  if (S.busy) return;
  setBusy(true);
  addLog('info', '正在扫描蓝牙设备（约 12 秒）…');
  toast('正在扫描蓝牙设备…');
  try {
    const r = await api('/bt/scan', { timeout: 12 });
    if (!r.ok) throw new Error(r.msg || '扫描失败');
    S.devices = r.devices || [];
    renderDevices(S.devices);
    const nearby = S.devices.filter((d) => !d.bonded).length;
    if (!S.devices.length) {
      addLog('warn', '没有发现任何蓝牙设备' + (r.msg ? '（' + r.msg + '）' : ''));
      addLog('warn', '机器人侧没有 bt_daemon 时蓝牙通道是走不通的，点上方「跳过蓝牙，直接用当前连接」即可。');
      toast('未发现蓝牙设备', 3200);
    } else {
      addLog('success', '附近 ' + nearby + ' 个，已配对 ' + (S.devices.length - nearby) + ' 个');
      toast('扫描到 ' + S.devices.length + ' 个蓝牙设备');
    }
  } catch (e) {
    addLog('error', '蓝牙扫描失败：' + (e.message || e));
    toast('蓝牙扫描失败');
  } finally {
    setBusy(false);
  }
}

async function listPaired() {
  if (S.busy) return;
  setBusy(true);
  addLog('info', '读取已配对设备…');
  try {
    const r = await api('/bt/paired');
    if (!r.ok) throw new Error(r.msg || '读取失败');
    S.devices = (r.devices || []).map((d) => ({ ...d, bonded: true }));
    renderDevices(S.devices);
    addLog('success', '已配对设备 ' + S.devices.length + ' 个');
    toast('已配对设备 ' + S.devices.length + ' 个');
  } catch (e) {
    addLog('error', '读取已配对设备失败：' + (e.message || e));
    toast('读取已配对设备失败');
  } finally {
    setBusy(false);
  }
}

async function connectBt() {
  if (S.busy) return;
  const mac = ($('bt-mac').value || '').trim().toUpperCase();
  if (!mac) { toast('请输入 MAC'); return; }
  // 方式 B 是"机器人主动去连另一台设备"。把机器人自己的地址填进来必然失败
  // （自己连自己，SPP 会报 No route to host），这里直接挡掉。
  if (S.adapterAddr && mac === S.adapterAddr) {
    addLog('warn', '这是面板所在机器人自己的蓝牙地址，方式 B 会让机器人去连别的设备，不能填自己。');
    addLog('info', '手机直连机器人请用上面的「方式 A：浏览器蓝牙（BLE）」。');
    toast('不能连自己：手机直连请用方式 A', 3600);
    return;
  }
  const ok = confirm(
    '接下来会让【机器人】主动去连 ' + mac + '。\n\n' +
    '如果这是你的手机：手机不能作为 SPP 服务端，一定会 Connection refused。\n' +
    '手机连机器人请用上面的「方式 A：浏览器蓝牙」。\n\n' +
    '确定仍要让机器人连这台设备？');
  if (!ok) return;
  setBusy(true);
  addLog('info', '正在连接蓝牙 ' + mac + '…');
  try {
    const r = await api('/bt/connect', { mac });
    if (!r.ok) throw new Error(r.msg || '连接失败');
    S.btConnected = true;
    S.btMac = mac;
    localStorage.setItem(LS_MAC, mac);
    addLog('success', '蓝牙连接成功，机器人已回报状态');
    toast('蓝牙连接成功');
    S.step = 2;
    paint();
  } catch (e) {
    addLog('error', '蓝牙连接失败：' + (e.message || e));
    addLog('warn', '可以点「跳过蓝牙，直接用当前连接」，用本机后端继续配置网络');
    toast('连接失败：' + String(e.message || '').slice(0, 60), 3600);
  } finally {
    setBusy(false);
  }
  // 蓝牙通道建立后顺手扫一次 WiFi（放到本轮任务之外，避免被 busy 挡住）
  if (S.btConnected && S.mode === 'wifi') setTimeout(scanWifi, 0);
}

// ------------------------------------------------------------ 步骤 2：WiFi

function renderAps() {
  const box = $('wifi-list');
  box.innerHTML = '';
  if (!S.aps.length) {
    box.innerHTML = '<div class="info-box">没有扫描到 WiFi，请再点一次「扫描 WiFi」。</div>';
    return;
  }
  const title = document.createElement('span');
  title.className = 'list-title';
  title.textContent = '选择 WiFi：';
  box.appendChild(title);
  S.aps.forEach((ap) => {
    const el = document.createElement('div');
    el.className = 'dev-item' + (ap.ssid === S.selectedSsid ? ' active' : '');
    el.innerHTML = '<span class="dev-name"></span><span class="dev-meta"></span>';
    el.querySelector('.dev-name').textContent = ap.ssid;
    el.querySelector('.dev-meta').textContent =
      '信号 ' + ap.signal + '　' + (ap.security || '开放');
    el.addEventListener('click', () => {
      S.selectedSsid = ap.ssid;
      $('wifi-psk-row').hidden = false;
      $('btn-wifi-connect').disabled = false;
      renderAps();
      addLog('info', '已选择 WiFi：' + ap.ssid);
    });
    box.appendChild(el);
  });
}

async function scanWifi() {
  if (S.busy) return;
  setBusy(true);
  addLog('info', '正在扫描 WiFi…');
  try {
    const r = await netScanWifi();
    if (!r.ok) throw new Error(r.msg || '扫描失败');
    S.aps = r.aps || [];
    renderAps();
    addLog('success', '扫描到 ' + S.aps.length + ' 个 WiFi');
    toast('扫描到 ' + S.aps.length + ' 个 WiFi');
  } catch (e) {
    addLog('error', '扫描 WiFi 失败：' + (e.message || e));
    toast('扫描 WiFi 失败');
  } finally {
    setBusy(false);
  }
}

async function connectWifi() {
  if (S.busy || !S.selectedSsid) return;
  const psk = $('wifi-psk').value || '';
  if (!confirm('让机器人连接「' + S.selectedSsid + '」？\n\n切换 WiFi 会断开当前网络，本页面可能失联。')) return;
  setBusy(true);
  addLog('info', '正在连接 WiFi ' + S.selectedSsid + '…');
  toast('命令已发送，等待机器人连上');
  try {
    const r = await netConnectWifi(S.selectedSsid, psk);
    if (!r.ok) throw new Error(r.msg || '连接失败');
    S.robotIp = ipOf(r);
    S.connectedSsid = r.ssid || S.selectedSsid;
    addLog('success', 'WiFi 已连接：' + S.connectedSsid + '，IP ' + (S.robotIp || '未知'));
    toast('已连接 ' + S.connectedSsid);
    paintDone();
    if (!S.robotIp) fetchStatus();
  } catch (e) {
    // 切换过程中页面失联是预期行为，不是逻辑错误
    addLog('error', '连接 WiFi 失败：' + (e.message || e));
    addLog('warn', '如果页面刚才是断开的，说明机器人已经切到新网络：请连上同一 WiFi 后重新打开面板。');
    toast('连接 WiFi 失败');
  } finally {
    setBusy(false);
  }
}

// ------------------------------------------------------------ 步骤 2：AP

async function openAp() {
  if (S.busy) return;
  if (!confirm('开启热点会关闭机器人当前 WiFi，本页面会失联。继续？')) return;
  setBusy(true);
  addLog('info', '正在开启 AP 热点…');
  try {
    const r = await netOpenAp();
    if (!r.ok) throw new Error(r.msg || '开启失败');
    S.robotIp = ipOf(r) || '192.168.124.1';
    addLog('success', 'AP 热点已开启：' + (r.ssid || '') + '，IP ' + S.robotIp);
    toast('AP 热点已开启');
    paintDone();
  } catch (e) {
    addLog('error', '开启 AP 失败：' + (e.message || e));
    addLog('warn', '如果页面刚才是断开的，热点可能已经开好：请连上热点后重新打开面板。');
    toast('开启 AP 失败');
  } finally {
    setBusy(false);
  }
}

// 从热点模式退回 WiFi（对应参考实现里的 closeAp / {"cmd":"wifi"}）
async function closeAp() {
  if (S.busy) return;
  if (!confirm('关闭热点并恢复之前的 WiFi？\n\n本页面会失联，请确认手机已连回原 WiFi。')) return;
  setBusy(true);
  addLog('info', '正在关闭热点、恢复 WiFi…');
  try {
    const r = S.ble
      ? await bleCmd({ cmd: 'wifi' }, 60000)
      : (S.btConnected ? await api('/bt/cmd', { cmd: 'wifi' }) : await api('/ap/off'));
    if (!r.ok) throw new Error(r.msg || '关闭失败');
    addLog('success', '热点已关闭，恢复连接：' + (r.restored || '（未记录原连接）') +
      '，IP ' + (ipOf(r) || '未知'));
    toast('热点已关闭');
    $('btn-ap-off').hidden = true;
  } catch (e) {
    addLog('error', '关闭热点失败：' + (e.message || e));
    addLog('warn', '如果页面刚断开，说明热点已关闭：请连回原 WiFi 后重新打开面板。');
    toast('关闭热点失败');
  } finally {
    setBusy(false);
  }
}

// ------------------------------------------------------------ 步骤 3

async function fetchStatus() {
  if (S.busy) return;
  setBusy(true);
  addLog('info', '正在获取机器人 IP…');
  try {
    const r = await netStatus();
    if (!r.ok) throw new Error(r.msg || '获取失败');
    S.robotIp = ipOf(r);
    S.connectedSsid = r.ssid || S.connectedSsid;
    addLog('success', '获取到 IP：' + S.robotIp + '，模式：' + (r.wlan_mode || 'unknown'));
    toast('获取到 IP：' + S.robotIp);
    paintDone();
  } catch (e) {
    addLog('error', '获取 IP 失败：' + (e.message || e));
    toast('获取 IP 失败');
  } finally {
    setBusy(false);
  }
}

function paintDone() {
  $('result-ip').textContent = S.robotIp || '-';
  $('result-mode').textContent =
    S.mode === 'ap' ? 'AP 直连模式' : (S.mode === 'wifi' ? 'WiFi 模式' : '');
  $('done-info').innerHTML = '';
  const lines = [];
  if (S.mode === 'ap') {
    lines.push('请打开手机「设置 → WiFi」，连接 <b>' + $('ap-ssid').textContent + '</b> 热点');
    lines.push('连接成功后用 <b>' + (S.robotIp || '-') + '</b> 打开面板');
  } else {
    lines.push('机器人已连 WiFi：<b>' + (S.connectedSsid || '-') + '</b>');
    lines.push('请确保手机与机器人在同一 WiFi 网络');
  }
  $('done-info').innerHTML = lines.join('<br>');
  S.step = 3;
  paint();
}

// ------------------------------------------------------------ 保存

function loadHistory() {
  try {
    const h = JSON.parse(localStorage.getItem(LS_HIST) || '[]');
    return Array.isArray(h) ? h : [];
  } catch (e) { return []; }
}

function addHistory(ip) {
  if (!ip) return;
  const h = loadHistory().filter((x) => x !== ip);
  h.unshift(ip);
  localStorage.setItem(LS_HIST, JSON.stringify(h.slice(0, 10)));
}

function renderHistory() {
  const box = $('ip-history');
  box.innerHTML = '';
  const h = loadHistory();
  if (!h.length) return;
  const title = document.createElement('span');
  title.className = 'list-title';
  title.textContent = '历史 IP（点选使用）';
  box.appendChild(title);
  h.forEach((ip) => {
    const el = document.createElement('div');
    el.className = 'ip-item';
    el.innerHTML = '<span class="ip-value"></span><button class="ip-del">删除</button>';
    el.querySelector('.ip-value').textContent = ip;
    el.querySelector('.ip-value').addEventListener('click', () => {
      $('manual-ip').value = ip;
      addLog('info', '已选择历史 IP：' + ip);
    });
    el.querySelector('.ip-del').addEventListener('click', () => {
      localStorage.setItem(LS_HIST, JSON.stringify(loadHistory().filter((x) => x !== ip)));
      renderHistory();
    });
    box.appendChild(el);
  });
}

// 探测某个 IP 是否能打开面板；能打开就把页面切过去
async function probeAndGo(ip) {
  if (ip === location.hostname) {
    location.href = '/';
    return;
  }
  const ctl = new AbortController();
  const timer = setTimeout(() => ctl.abort(), 3000);
  try {
    await fetch('http://' + ip + '/api/net/status', { signal: ctl.signal, cache: 'no-store' });
    clearTimeout(timer);
    location.href = 'http://' + ip + '/';
  } catch (e) {
    clearTimeout(timer);
    toast('已保存 ' + ip + '，但当前访问不到，请确认手机在同一网络', 4200);
    addLog('warn', '已保存 ' + ip + '，但探测失败：' + (e.message || e));
    setTimeout(() => { location.href = '/'; }, 1800);
  }
}

async function saveAndBack() {
  if (!S.robotIp) return;
  addHistory(S.robotIp);
  localStorage.setItem(LS_IP, S.robotIp);
  addLog('success', '已保存后端地址 ' + S.robotIp);
  toast('已保存 ' + S.robotIp);
  if (S.ble) bleDisconnect();
  if (S.btConnected) api('/bt/disconnect').catch(() => {});
  setTimeout(() => probeAndGo(S.robotIp), 400);
}

function saveManualIp() {
  const ip = ($('manual-ip').value || '').trim();
  if (!ip) { toast('请输入 IP'); return; }
  addHistory(ip);
  localStorage.setItem(LS_IP, ip);
  addLog('success', '已保存后端地址 ' + ip);
  toast('已保存 ' + ip);
  setTimeout(() => probeAndGo(ip), 300);
}

// ------------------------------------------------------------ 事件绑定

document.querySelectorAll('.mode-item').forEach((el) => {
  el.addEventListener('click', () => {
    S.mode = el.dataset.mode;
    if (S.mode === 'manual') {
      $('manual-ip').value = localStorage.getItem(LS_IP) || location.hostname;
      renderHistory();
      paint();
      return;
    }
    S.step = 1;
    paint();
    addLog('info', S.mode === 'wifi' ? '已选择：WiFi 连接' : '已选择：AP 直连');
    loadLocalChannel();
    loadBtAdapter();
  });
});

$('btn-back').addEventListener('click', () => {
  if (S.mode !== '') { resetWizard(); return; }
  location.href = '/';
});
$('btn-skip-bt').addEventListener('click', () => {
  S.btConnected = false;
  addLog('info', '跳过蓝牙，改用本机后端继续');
  S.step = 2;
  paint();
  if (S.mode === 'wifi') scanWifi();
});
$('btn-ble-connect').addEventListener('click', async () => {
  if (S.busy) return;
  setBusy(true);
  try {
    const dev = await bleConnect();
    addLog('success', '浏览器蓝牙已连接：' + (dev.name || dev.id));
    toast('蓝牙已连接');
    // 连上就查一次状态，顺便验证协议通
    try {
      const st = await bleCmd({ cmd: 'status' }, 15000);
      if (st.ok) {
        addLog('success', '机器人回报：模式 ' + (st.wlan_mode || '?') +
          '，IP ' + (ipOf(st) || '未分配'));
      } else {
        addLog('warn', '状态查询返回失败：' + (st.msg || ''));
      }
    } catch (e) {
      addLog('warn', '状态查询失败：' + (e.message || e));
    }
    S.step = 2;
    paint();
    if (S.mode === 'wifi') setTimeout(scanWifi, 0);
  } catch (e) {
    if (e && (e.name === 'NotFoundError' || /cancel/i.test(String(e.message)))) {
      addLog('info', '选择框已关闭（未选设备）');
      addLog('warn', '如果列表是空的：手机要打开"定位服务"，并给 Chrome "附近设备/位置"权限；' +
        '蓝牙已配对过旧设备时，先关掉再打开手机蓝牙。');
    } else {
      addLog('error', '浏览器蓝牙连接失败：' + (e.message || e));
      toast('蓝牙连接失败');
    }
  } finally {
    setBusy(false);
  }
});
$('btn-ble-any') && $('btn-ble-any').addEventListener('click', async () => {
  if (S.busy) return;
  setBusy(true);
  try {
    await bleAnyScan();
    toast('诊断完成，结果见日志');
  } catch (e) {
    if (e && (e.name === 'NotFoundError' || /cancel/i.test(String(e.message)))) {
      addLog('warn', '诊断：一个设备都没扫到。检查手机定位服务是否开启、Chrome 是否有"附近设备"权限。');
    } else {
      addLog('error', '诊断失败：' + (e.message || e));
    }
  } finally {
    setBusy(false);
  }
});
$('btn-ble-disconnect') && $('btn-ble-disconnect').addEventListener('click', () => {
  bleDisconnect();
  addLog('info', '已断开浏览器蓝牙');
});
$('btn-bt-scan').addEventListener('click', scanBt);
$('btn-bt-paired').addEventListener('click', listPaired);
$('btn-bt-connect').addEventListener('click', connectBt);
$('btn-wifi-scan').addEventListener('click', scanWifi);
$('btn-wifi-connect').addEventListener('click', connectWifi);
$('btn-ap-on').addEventListener('click', openAp);
$('btn-ap-off').addEventListener('click', closeAp);
$('btn-save').addEventListener('click', saveAndBack);
$('btn-refetch').addEventListener('click', fetchStatus);
$('btn-manual-save').addEventListener('click', saveManualIp);
$('btn-log-clear').addEventListener('click', () => {
  $('log-area').innerHTML = '';
  $('card-log').hidden = true;
});

// ------------------------------------------------------------ 启动

(function init() {
  const savedMac = localStorage.getItem(LS_MAC);
  if (savedMac) S.btMac = savedMac;
  $('bt-mac').value = S.btMac;
  paint();
  paintBleState();
  addLog('info', '面板地址：' + location.host +
    (window.isSecureContext ? '（安全上下文）' : '（非安全上下文，浏览器蓝牙不可用）'));
  loadLocalChannel();
})();
