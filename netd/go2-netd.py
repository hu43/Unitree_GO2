#!/usr/bin/env python3
"""Go2 网络/蓝牙配置守护进程。

跑在宿主机（Jetson）上、以 root 运行、只监听 127.0.0.1:8099。

为什么要单独一个进程：web 后端 app.py 跑在 go2_humble 容器里（host 网络，
但镜像是精简的 ros:humble-base —— 没有 nmcli / bluetoothctl，也没挂 D-Bus），
拿不到 NetworkManager 和 BlueZ。所以 WiFi 扫描连接、AP 热点、蓝牙扫描与
SPP 会话都在宿主机这一侧做，容器通过 /api/net/* 代理过来。

命令协议与 G1 的 bt_daemon 对齐，前端无论走蓝牙还是走本机后端都拿到同样的结构：
    {"cmd":"status"}                        -> host_ip / gateway / wlan_mode / ssid
    {"cmd":"scan"}                          -> aps[]
    {"cmd":"connect","ssid":..,"psk":..}    -> msg
    {"cmd":"ap"} / {"cmd":"wifi"}           -> msg
"""

import json
import os
import re
import socket
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

LISTEN = ("127.0.0.1", 8099)
IFACE = "wlan0"
AP_CON = "go2-ap"
AP_SSID = "Unitree-Go2"
AP_PSK = "12345678"
AP_IP = "192.168.124.1"
SPP_CHANNEL = 1
STATE_FILE = "/run/go2-netd-state.json"

ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
BT_DEV = re.compile(r"Device\s+([0-9A-F]{2}(?::[0-9A-F]{2}){5})(?:\s+(.*))?$")


# ---------------------------------------------------------------- 基础工具

def run(cmd, timeout=20, stdin_data=None):
    """执行命令，返回 (rc, stdout, stderr)，不抛异常。"""
    try:
        p = subprocess.run(cmd, capture_output=True, text=True,
                           timeout=timeout, input=stdin_data)
        return p.returncode, p.stdout or "", p.stderr or ""
    except subprocess.TimeoutExpired:
        return 124, "", "timeout after %ss" % timeout
    except FileNotFoundError:
        return 127, "", "%s: not found" % cmd[0]


def nmcli(args, timeout=20):
    return run(["nmcli"] + args, timeout=timeout)


def split_terse(line):
    """拆 nmcli -t 输出：':' 分隔，'\\:' 是转义冒号，'\\\\' 是转义反斜杠。"""
    out, cur, esc = [], [], False
    for ch in line:
        if esc:
            cur.append(ch)
            esc = False
        elif ch == "\\":
            esc = True
        elif ch == ":":
            out.append("".join(cur))
            cur = []
        else:
            cur.append(ch)
    out.append("".join(cur))
    return out


def load_state():
    try:
        with open(STATE_FILE, "r", encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def save_state(patch):
    st = load_state()
    st.update(patch)
    try:
        with open(STATE_FILE, "w", encoding="utf-8") as f:
            json.dump(st, f, ensure_ascii=False)
    except OSError:
        pass
    return st


def port_open(port, host="127.0.0.1"):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(0.4)
    try:
        s.connect((host, port))
        return True
    except OSError:
        return False
    finally:
        s.close()


# ---------------------------------------------------------------- WiFi

def wifi_ip():
    _, out, _ = nmcli(["-g", "IP4.ADDRESS", "device", "show", IFACE])
    for ln in out.splitlines():
        ln = ln.strip()
        if ln:
            return ln.split("/")[0]
    return ""


def wifi_gateway():
    _, out, _ = nmcli(["-g", "IP4.GATEWAY", "device", "show", IFACE])
    for ln in out.splitlines():
        ln = ln.strip()
        if ln:
            return ln
    return ""


def wifi_conn_name():
    _, out, _ = nmcli(["-g", "GENERAL.CONNECTION", "device", "show", IFACE])
    return out.strip().splitlines()[0].strip() if out.strip() else ""


def wifi_ssid():
    _, out, _ = nmcli(["-t", "-f", "ACTIVE,SSID", "dev", "wifi"])
    for ln in out.splitlines():
        parts = split_terse(ln)
        if len(parts) >= 2 and parts[0] == "yes":
            return parts[1]
    return ""


def wifi_status():
    ip = wifi_ip()
    con = wifi_conn_name()
    if con == AP_CON or ip.startswith("192.168.124."):
        mode = "ap"
    elif ip:
        mode = "wifi"
    else:
        mode = "unknown"
    return {
        "ok": True,
        "cmd": "status",
        "wlan_mode": mode,
        "host_ip": ip,
        "gateway": wifi_gateway(),
        "ssid": wifi_ssid() or (con or None),
        "conn": con,
        "iface": IFACE,
        "ap_ssid": AP_SSID,
        "ap_psk": AP_PSK,
        "app_py_running": port_open(8080),
        "hostname": socket.gethostname(),
    }


def wifi_scan(wait=12):
    """让 wlan0 重新扫描并返回去重后的 AP 列表（信号最强的那条）。"""
    nmcli(["dev", "wifi", "rescan"], timeout=10)
    deadline = time.time() + wait
    best = {}
    while True:
        _, out, _ = nmcli(["-t", "-f", "SSID,SIGNAL,SECURITY", "dev", "wifi", "list"],
                          timeout=15)
        best = {}
        for ln in out.splitlines():
            parts = split_terse(ln)
            if len(parts) < 3:
                continue
            ssid, signal, sec = parts[0], parts[1], parts[2]
            if not ssid:
                continue  # 隐藏网络
            try:
                sig = int(signal)
            except ValueError:
                sig = 0
            if ssid not in best or sig > best[ssid]["signal"]:
                best[ssid] = {"ssid": ssid, "signal": sig,
                              "security": "" if sec == "--" else sec}
        if best or time.time() >= deadline:
            break
        time.sleep(1.5)
    aps = sorted(best.values(), key=lambda a: -a["signal"])
    return {"ok": True, "cmd": "scan", "aps": aps}


def wifi_connect(ssid, psk, dry_run=False):
    """让 wlan0 连指定 WiFi。失败自动回滚到原连接，避免机器人彻底失联。"""
    if not ssid:
        return {"ok": False, "msg": "缺少 SSID"}
    prev = wifi_conn_name()
    args = ["dev", "wifi", "connect", ssid, "ifname", IFACE]
    if psk:
        args += ["password", psk]
    if dry_run:
        return {"ok": True, "msg": "dry-run，未执行", "cmd": ["nmcli"] + args,
                "prev": prev}

    t0 = time.time()
    rc, out, err = nmcli(args, timeout=70)
    msg = (out or err).strip().splitlines()
    msg = msg[-1] if msg else ""
    ip = wifi_ip()
    if rc != 0 or not ip:
        # 回滚：把原来的连接重新拉起来
        if prev and prev != AP_CON:
            nmcli(["con", "up", prev], timeout=40)
        result = {"ok": False, "msg": "连接失败：%s" % (msg or "超时未获取 IP"),
                  "ssid": ssid, "ip": wifi_ip(), "rolled_back": bool(prev)}
    else:
        result = {"ok": True, "msg": "已连接", "ssid": ssid, "ip": ip,
                  "gateway": wifi_gateway()}
    result.update({"cmd": "connect", "elapsed": round(time.time() - t0, 1)})
    save_state({"last_wifi": dict(result, at=time.time())})
    return result


def ap_on(ssid=None, psk=None, dry_run=False, revert_after=0):
    """把 wlan0 切成 AP 热点（NetworkManager ipv4 shared，自带 DHCP）。"""
    ssid = ssid or AP_SSID
    psk = psk or AP_PSK
    prev = wifi_conn_name()
    if prev and prev != AP_CON:
        save_state({"prev_conn": prev})
    add = ["con", "add", "type", "wifi", "ifname", IFACE, "con-name", AP_CON,
           "autoconnect", "no", "ssid", ssid,
           "802-11-wireless.mode", "ap", "802-11-wireless.band", "bg",
           "ipv4.method", "shared", "ipv4.addresses", AP_IP + "/24"]
    if psk:
        add += ["wifi-sec.key-mgmt", "wpa-psk", "wifi-sec.psk", psk]
    if dry_run:
        return {"ok": True, "msg": "dry-run，未执行", "cmd": ["nmcli"] + add,
                "prev": prev}
    nmcli(["con", "delete", AP_CON], timeout=15)
    rc, out, err = nmcli(add, timeout=25)
    if rc != 0:
        return {"ok": False, "msg": "创建热点配置失败：%s" % ((err or out).strip())}
    rc, out, err = nmcli(["con", "up", AP_CON], timeout=45)
    if rc != 0:
        nmcli(["con", "delete", AP_CON], timeout=15)
        return {"ok": False, "msg": "开启热点失败：%s" % ((err or out).strip())}
    result = {"ok": True, "cmd": "ap", "msg": "热点已开启",
              "ssid": ssid, "psk": psk, "ip": AP_IP,
              "host_ip": AP_IP, "gateway": AP_IP, "wlan_mode": "ap",
              "revert_after": revert_after}
    save_state({"last_ap": dict(result, at=time.time())})
    if revert_after and revert_after > 0:
        threading.Timer(revert_after, ap_off).start()
    return result


def ap_off():
    """关掉热点，回到之前的 WiFi。"""
    prev = load_state().get("prev_conn") or ""
    nmcli(["con", "down", AP_CON], timeout=25)
    nmcli(["con", "delete", AP_CON], timeout=15)
    back = ""
    if prev and prev != AP_CON:
        rc, out, err = nmcli(["con", "up", prev], timeout=45)
        if rc == 0:
            back = prev
    result = {"ok": True, "cmd": "wifi", "msg": "热点已关闭",
              "restored": back, "host_ip": wifi_ip(),
              "gateway": wifi_gateway(),
              "wlan_mode": "wifi" if wifi_ip() else "unknown"}
    save_state({"last_ap": dict(result, at=time.time())})
    return result


# ---------------------------------------------------------------- 蓝牙

def _bt_lines(cmd, timeout=20):
    rc, out, err = run(["bluetoothctl"] + cmd, timeout=timeout)
    return [ANSI.sub("", ln).replace("\r", "").strip()
            for ln in out.splitlines()]


def bt_paired():
    devs = []
    for ln in _bt_lines(["paired-devices"], timeout=15):
        m = BT_DEV.search(ln)
        if m:
            devs.append({"address": m.group(1),
                         "name": (m.group(2) or "").strip() or "未知设备",
                         "bonded": True})
    return devs


def bt_adapter_info():
    """本机蓝牙适配器的状态。机器人侧没有 bt_daemon 时，这里是唯一的诊断入口。"""
    rc, out, err = run(["bluetoothctl", "show"], timeout=15)
    txt = ANSI.sub("", (out or "") + (err or ""))
    info = {"address": "", "name": "", "alias": "", "powered": None,
            "discoverable": None, "pairable": None}
    for ln in txt.splitlines():
        s = ln.strip()
        low = s.lower()
        if s.startswith("Controller "):
            parts = s.split()
            if len(parts) > 1:
                info["address"] = parts[1]
        elif low.startswith("name:"):
            info["name"] = s.split(":", 1)[1].strip()
        elif low.startswith("alias:"):
            info["alias"] = s.split(":", 1)[1].strip()
        elif low.startswith("powered:"):
            info["powered"] = s.split(":", 1)[1].strip().lower() == "yes"
        elif low.startswith("discoverable:"):
            info["discoverable"] = s.split(":", 1)[1].strip().lower() == "yes"
        elif low.startswith("pairable:"):
            info["pairable"] = s.split(":", 1)[1].strip().lower() == "yes"
    # 手机在蓝牙列表里看到的是 Alias，不是内核的 Name（Go2 上 Name 仍是 ubuntu）
    info["name"] = info["alias"] or info["name"]
    return info


def bt_info():
    ad = bt_adapter_info()
    return {"ok": bool(ad["address"]), "adapter": ad,
            "paired": len(bt_paired()),
            "msg": "" if ad["address"] else "本机没有可用的蓝牙适配器"}


def bt_scan(timeout=12):
    """开一轮 discovery，结束后读 bluetoothd 的缓存列表。

    BlueZ 5.53 没有 `bluetoothctl --timeout`，所以用交互式管道：
    scan on -> 等待 -> devices -> scan off/quit。
    管道随时可能因为 bluetoothd 没起来而断，任何一步失败都不抛异常，
    改用 bluetoothd 缓存里的设备列表兜底，并把原因放进 msg。
    """
    err_note = ""
    out = ""
    try:
        p = subprocess.Popen(["bluetoothctl"], stdin=subprocess.PIPE,
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             text=True)
    except OSError as e:
        p = None
        err_note = "bluetoothctl 无法启动：%s" % e

    if p:
        try:
            p.stdin.write("power on\nscan on\n")
            p.stdin.flush()
        except OSError as e:
            err_note = "开启扫描失败：%s" % e
        if not err_note:
            time.sleep(timeout)
            for cmd in ("devices\n", "scan off\nquit\n"):
                try:
                    p.stdin.write(cmd)
                    p.stdin.flush()
                except OSError:
                    break
                if cmd.startswith("devices"):
                    time.sleep(0.8)
        try:
            out = p.communicate(timeout=10)[0] or ""
        except subprocess.TimeoutExpired:
            try:
                p.kill()
            except OSError:
                pass
            out = ""
            err_note = err_note or "bluetoothctl 未按时退出"

    bonded = {d["address"] for d in bt_paired()}
    # bluetoothd 的 devices 列表包含历史缓存，只有本轮 CHG 事件里出现过的才有 RSSI。
    # 用 RSSI 区分「现在就在附近」和「以前见过」，否则列表里会混进几十个旧设备。
    rssi = {}
    names = {}
    for ln in ANSI.sub("", out).replace("\r", "").splitlines():
        m = BT_DEV.search(ln.strip())
        if not m:
            continue
        addr, rest = m.group(1), (m.group(2) or "").strip()
        rm = re.search(r"RSSI:\s*(-?\d+)", rest)
        if rm:
            rssi[addr] = int(rm.group(1))
            rest = rest[:rm.start()].strip()
        # 只从 devices 的输出里取名字；[CHG] 行带的是属性（LegacyPairing: no 之类），
        # 当成设备名会污染列表
        if rest and "[CHG]" not in ln:
            names[addr] = rest

    found = {}
    for addr in set(names) | set(rssi):
        found[addr] = {"address": addr,
                       "name": names.get(addr) or "未知设备",
                       "bonded": addr in bonded,
                       "rssi": rssi.get(addr)}
    # 已配对的排最前，其次是本轮扫到的（信号强的在前），最后是历史缓存
    for d in bt_paired():
        found.setdefault(d["address"], dict(d, rssi=None))
    devs = sorted(found.values(),
                  key=lambda d: (not d["bonded"], d["rssi"] is None,
                                 -(d["rssi"] or -999), d["name"]))
    return {"ok": True, "cmd": "scan", "devices": devs,
            "adapter": bt_adapter_info(), "msg": err_note}


class BtSession:
    """一条 RFCOMM SPP 会话，一行 JSON 请求 -> 一行 JSON 响应。"""

    def __init__(self, sock, mac):
        self.sock = sock
        self.mac = mac
        self.lock = threading.Lock()

    def send(self, cmd, args=None, timeout=60):
        req = {"cmd": cmd}
        if args:
            req.update(args)
        payload = (json.dumps(req, ensure_ascii=False) + "\n").encode()
        with self.lock:
            self.sock.settimeout(timeout)
            self.sock.sendall(payload)
            deadline = time.time() + timeout
            buf = b""
            while True:
                remain = deadline - time.time()
                if remain <= 0:
                    raise TimeoutError("读取超时 %ss" % timeout)
                self.sock.settimeout(remain)
                chunk = self.sock.recv(4096)
                if not chunk:
                    raise ConnectionError("对端关闭连接")
                buf += chunk
                if b"\n" in buf:
                    line = buf.split(b"\n", 1)[0]
                    return json.loads(line.decode("utf-8", "replace").strip())

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


_session = None
_session_lock = threading.Lock()


def bt_current():
    return _session


def bt_connect(mac):
    global _session
    mac = (mac or "").strip().upper()
    if not re.match(r"^[0-9A-F]{2}(:[0-9A-F]{2}){5}$", mac):
        return {"ok": False, "msg": "MAC 格式不对：%s" % mac}
    if not hasattr(socket, "AF_BLUETOOTH"):
        return {"ok": False, "msg": "本机 Python 不支持蓝牙 socket"}

    steps = []
    bonded = {d["address"] for d in bt_paired()}
    if mac not in bonded:
        rc, out, err = run(["bluetoothctl", "pair", mac], timeout=45)
        steps.append("pair rc=%s %s" % (rc, (out or err).strip().splitlines()[-1]
                                        if (out or err).strip() else ""))
        run(["bluetoothctl", "trust", mac], timeout=15)

    with _session_lock:
        if _session:
            _session.close()
            _session = None

    sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM,
                         socket.BTPROTO_RFCOMM)
    try:
        sock.settimeout(20)
        sock.connect((mac, SPP_CHANNEL))
    except OSError as e:
        sock.close()
        return {"ok": False, "msg": "SPP 连接失败：%s" % e, "steps": steps}

    sess = BtSession(sock, mac)
    try:
        r = sess.send("status", timeout=8)
    except Exception as e:
        sess.close()
        return {"ok": False,
                "msg": "SPP 已建立但对端没回 status（bt_daemon 没跑？）：%s" % e,
                "steps": steps}

    with _session_lock:
        _session = sess
    return {"ok": True, "msg": "蓝牙连接成功", "mac": mac,
            "status": r, "steps": steps}


def bt_cmd(cmd, args=None, timeout=60):
    sess = bt_current()
    if not sess:
        return {"ok": False, "msg": "蓝牙未连接"}
    try:
        return sess.send(cmd, args, timeout=timeout)
    except Exception as e:
        return {"ok": False, "msg": "蓝牙通信失败：%s" % e}


def bt_disconnect():
    global _session
    with _session_lock:
        if _session:
            _session.close()
            _session = None
    return {"ok": True, "msg": "已断开蓝牙"}


# ---------------------------------------------------------------- HTTP

ROUTES = {
    "/status": lambda q: wifi_status(),
    "/wifi/scan": lambda q: wifi_scan(),
    "/wifi/connect": lambda q: wifi_connect(q.get("ssid", [""])[0],
                                            q.get("psk", [""])[0],
                                            dry_run=q.get("dry_run", ["0"])[0] == "1"),
    "/wifi/last": lambda q: {"ok": True, "last": load_state().get("last_wifi")},
    "/ap/on": lambda q: ap_on(q.get("ssid", [None])[0], q.get("psk", [None])[0],
                              dry_run=q.get("dry_run", ["0"])[0] == "1",
                              revert_after=int(q.get("revert_after", ["0"])[0] or 0)),
    "/ap/off": lambda q: ap_off(),
    "/bt/scan": lambda q: bt_scan(int(q.get("timeout", ["12"])[0] or 12)),
    "/bt/info": lambda q: bt_info(),
    "/bt/paired": lambda q: {"ok": True, "devices": bt_paired()},
    "/bt/connect": lambda q: bt_connect(q.get("mac", [""])[0]),
    "/bt/cmd": lambda q: bt_cmd(q.get("cmd", ["status"])[0],
                                {k: v[0] for k, v in q.items()
                                 if k not in ("cmd", "timeout")},
                                timeout=int(q.get("timeout", ["60"])[0] or 60)),
    "/bt/disconnect": lambda q: bt_disconnect(),
    "/health": lambda q: {"ok": True, "pid": os.getpid()},
}


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *a):  # 静音默认访问日志
        pass

    def _reply(self, obj, code=200):
        body = json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _dispatch(self):
        u = urlparse(self.path)
        fn = ROUTES.get(u.path)
        if not fn:
            self._reply({"ok": False, "msg": "unknown endpoint %s" % u.path}, 404)
            return
        try:
            self._reply(fn(parse_qs(u.query)))
        except Exception as e:
            self._reply({"ok": False, "msg": "%s: %s" % (type(e).__name__, e)}, 500)

    do_GET = _dispatch
    do_POST = _dispatch


def main():
    srv = ThreadingHTTPServer(LISTEN, Handler)
    srv.daemon_threads = True
    print("go2-netd listening on http://%s:%d" % LISTEN, flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
