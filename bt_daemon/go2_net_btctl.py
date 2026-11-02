#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
go2_net_btctl.py — Go2 网络模式蓝牙守护进程（常驻，systemd 管理，root 运行）

迁移自 G1 的 g1_net_btctl.py（/home/unitree/huang/bt_daemon/），
蓝牙协议保持一致：一行 JSON 请求 → 一行 JSON 响应（\\n 分隔）。

    {"cmd":"ap"}                           开 AP 热点
    {"cmd":"wifi"}                         关 AP 回 WiFi
    {"cmd":"scan"}                         扫描可见 WiFi
    {"cmd":"connect","ssid":"x","psk":"y"} 连指定 WiFi
    {"cmd":"status"}                       查 wlan0 模式 + 面板状态
    {"cmd":"reboot_app"}                   重启 Go2 web 面板

与 G1 版本的差异，都是因为 Go2 机载环境不同：
  - AP 用本目录的 setup_ap.sh / teardown_ap.sh，只把 wlan0 从 NetworkManager
    摘出来，不停 NetworkManager 本身。Go2 的 eth0 是机器人内网控制链路
    （192.168.123.18），整体停 NM 有断链风险；G1 那边是整体停 NM。
  - SSID 用 Unitree-Go2，适配器广播名也设成 Unitree-Go2。
  - G1 的 handshake 是机械臂动作，Go2 没有机械臂，返回明确的不支持。
  - reboot_app 重启的是 Go2 的 web 面板（web/app.py），systemd 会自动拉起。
  - nmcli 一律走 argv 列表，不再拼 shell 字符串，SSID/密码里带引号也不会注入。

架构（沿用 G1）：
  - BlueZ Profile1（RegisterProfile）注册 SPP UUID → 手机连入时 BlueZ 自动
    accept 并通过 NewConnection(fd, path, opts) 把 fd 推给本进程（推模式，
    不用自己 bind/listen）。
  - Agent1 无头自动配对，Android 弹出的确认框由系统侧确认，本进程自动应答。
"""
import os
import sys
import json
import time
import socket
import logging
import subprocess
import threading
import dbus
import dbus.mainloop.glib
import dbus.service
from gi.repository import GLib

# ---------- 路径常量 ----------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SETUP_AP = os.path.join(SCRIPT_DIR, "setup_ap.sh")
TEARDOWN_AP = os.path.join(SCRIPT_DIR, "teardown_ap.sh")

AP_IFACE = "wlan0"
AP_GATEWAY = "192.168.124.1"
AP_SSID = "Unitree-Go2"
AP_PSK = "12345678"
ADAPTER_ALIAS = "Unitree-Go2"
WEB_PORT = 8080
SPP_UUID = "00001101-0000-1000-8000-00805F9B34FB"
RFCOMM_CHANNEL = 1
SERVICE_NAME = "Go2 Net BtCtl"
PROFILE_PATH = "/org/go2/NetBtCtlProfile"

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(levelname)s] %(message)s",
                    datefmt="%H:%M:%S")
log = logging.getLogger("go2btd")

state_lock = threading.Lock()
mode_state = {"mode": "wifi", "ssid": None}


def run(cmd, timeout=30):
    """跑命令（本进程是 root，不需 sudo）。cmd 可以是列表或字符串。

    列表形式不经过 shell，SSID/密码里的引号、分号都不会被解释。
    """
    try:
        p = subprocess.run(cmd, shell=isinstance(cmd, str),
                           capture_output=True, text=True, timeout=timeout)
        return p.returncode, (p.stdout + p.stderr).strip()
    except Exception as e:
        return -1, str(e)


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


def get_wifi_ssid():
    rc, out = run(["iw", "dev", AP_IFACE, "link"], timeout=10)
    for ln in out.splitlines():
        if "SSID:" in ln:
            return ln.split("SSID:", 1)[1].strip().strip('"')
    return None


def get_wlan_mode():
    """返回 wlan0 当前模式: wifi / ap / unknown"""
    rc, out = run(["iw", "dev", AP_IFACE, "info"], timeout=10)
    if rc == 0:
        if "type AP" in out:
            return "ap"
        if "type managed" in out:
            return "wifi"
    return "unknown"


def get_lan_ip():
    """wlan0 的 IPv4 地址。手机要和机器人在同一 WiFi，只有 wlan0 的地址够得到；
    eth0 的 192.168.123.18 是机内总线，手机连不上。未就绪时返回 None。"""
    for ip_cmd in ("/usr/sbin/ip", "/sbin/ip", "ip"):
        rc, out = run([ip_cmd, "-4", "-o", "addr", "show", AP_IFACE], timeout=10)
        if rc == 0:
            for ln in out.splitlines():
                parts = ln.split()
                for i, tok in enumerate(parts):
                    if tok == "inet" and i + 1 < len(parts):
                        ip = parts[i + 1].split("/")[0]
                        if ip and ip != "127.0.0.1":
                            return ip
    return None


def web_running():
    """Go2 web 面板是否在监听（跑在容器里，pgrep 在宿主侧不可靠，改探端口）"""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(0.4)
    try:
        s.connect(("127.0.0.1", WEB_PORT))
        return True
    except OSError:
        return False
    finally:
        s.close()


# ---------- 命令处理 ----------
def cmd_ap(args):
    """开 AP 热点（setup_ap.sh 里 hostapd 前台阻塞，所以必须后台跑）"""
    if get_wlan_mode() == "ap":
        return {"ok": True, "msg": "已在 AP 模式", "gateway": AP_GATEWAY,
                "ssid": AP_SSID, "host_ip": AP_GATEWAY, "wlan_mode": "ap"}
    run("nohup bash %s %s %s > /tmp/setup_ap.log 2>&1 &"
        % (SETUP_AP, AP_IFACE, AP_GATEWAY), timeout=2)
    time.sleep(6)
    if get_wlan_mode() == "ap":
        with state_lock:
            mode_state["mode"] = "ap"
            mode_state["ssid"] = None
        return {"ok": True, "msg": "AP 已开", "gateway": AP_GATEWAY,
                "ssid": AP_SSID, "psk": AP_PSK,
                "host_ip": AP_GATEWAY, "wlan_mode": "ap"}
    return {"ok": False, "msg": "AP 启动失败，见 /tmp/setup_ap.log"}


def cmd_wifi(args):
    """关 AP，回 WiFi"""
    if get_wlan_mode() != "ap":
        with state_lock:
            mode_state["mode"] = "wifi"
            mode_state["ssid"] = get_wifi_ssid()
        return {"ok": True, "msg": "未在 AP 模式，已是 WiFi",
                "ssid": get_wifi_ssid(), "host_ip": get_lan_ip(),
                "wlan_mode": "wifi"}
    rc, out = run(["bash", TEARDOWN_AP, AP_IFACE], timeout=40)
    time.sleep(3)
    with state_lock:
        mode_state["mode"] = "wifi"
        mode_state["ssid"] = get_wifi_ssid()
    return {"ok": rc == 0, "msg": out or "已回 WiFi",
            "ssid": get_wifi_ssid(), "host_ip": get_lan_ip(),
            "wlan_mode": get_wlan_mode()}


def cmd_scan(args, wait=14):
    """扫描可见 WiFi。

    必须先 rescan：nmcli 的列表默认只有缓存（通常就当前连着的那个 AP）。
    rescan 之后驱动要几秒才出结果，所以轮询到有结果或超时，不能固定 sleep。
    """
    run(["nmcli", "dev", "wifi", "rescan"], timeout=10)
    deadline = time.time() + wait
    aps = []
    while True:
        rc, out = run(["nmcli", "-t", "-f", "SSID,SIGNAL,SECURITY",
                       "dev", "wifi", "list"], timeout=20)
        if rc != 0:
            return {"ok": False, "msg": out}
        aps = []
        seen = set()
        for line in out.splitlines():
            parts = split_terse(line)
            ssid = parts[0] if parts else ""
            if not ssid or ssid in seen:
                continue
            seen.add(ssid)
            sec = parts[2] if len(parts) > 2 else ""
            aps.append({
                "ssid": ssid,
                "signal": parts[1] if len(parts) > 1 else "",
                "security": "" if sec == "--" else sec,
            })
        if len(aps) > 1 or time.time() >= deadline:
            break
        time.sleep(1.5)
    return {"ok": True, "aps": aps}


def cmd_connect(args):
    """连接 WiFi（陌生网络带密码）。失败回滚到原连接，避免机器人失联。"""
    ssid = (args.get("ssid") or "").strip()
    psk = args.get("psk", "")
    if not ssid:
        return {"ok": False, "msg": "缺少 ssid"}
    if get_wlan_mode() == "ap":
        run(["bash", TEARDOWN_AP, AP_IFACE], timeout=40)
        time.sleep(3)

    prev = get_wifi_ssid()
    run(["nmcli", "connection", "delete", ssid], timeout=10)
    cmd = ["nmcli", "dev", "wifi", "connect", ssid, "ifname", AP_IFACE]
    if psk:
        cmd += ["password", psk]
    rc, out = run(cmd, timeout=70)

    ip = get_lan_ip()
    if rc == 0 and ip:
        with state_lock:
            mode_state["mode"] = "wifi"
            mode_state["ssid"] = ssid
        return {"ok": True, "msg": "已连接 %s" % ssid, "ssid": ssid,
                "host_ip": ip, "wlan_mode": "wifi"}
    if prev and prev != ssid:
        run(["nmcli", "connection", "up", prev], timeout=40)
    return {"ok": False, "msg": out, "rolled_back": bool(prev),
            "host_ip": get_lan_ip()}


def cmd_status(args):
    mode = get_wlan_mode()
    ssid = get_wifi_ssid() if mode == "wifi" else None
    if mode == "ap":
        host_ip = AP_GATEWAY
    elif mode == "wifi":
        host_ip = get_lan_ip()
    else:
        host_ip = None
    return {
        "ok": True,
        "wlan_mode": mode,
        "ssid": ssid,
        "app_py_running": web_running(),
        "gateway": AP_GATEWAY if mode == "ap" else None,
        "host_ip": host_ip,
        "ap_ssid": AP_SSID,
        "ap_psk": AP_PSK,
    }


def cmd_handshake(args):
    """G1 的机械臂握手动作。Go2 没有机械臂，明确返回不支持，
    保留命令名是为了让老客户端拿到可读的失败原因而不是「未知命令」。"""
    return {"ok": False,
            "msg": "Go2 没有机械臂，handshake 不适用；动作请用面板的运动模式页",
            "action": "shake hand"}


def cmd_reboot_app(args):
    """重启 Go2 web 面板：杀掉容器里的 app.py，go2-web.service 约 2s 自动拉起"""
    rc, out = run(["pkill", "-f", "web/app.py"], timeout=10)
    time.sleep(1)
    return {"ok": True, "msg": "已重启 web 面板（systemd 会自动拉起）",
            "was_running": rc == 0}


DISPATCH = {
    "ap": cmd_ap, "wifi": cmd_wifi, "scan": cmd_scan, "connect": cmd_connect,
    "status": cmd_status, "handshake": cmd_handshake, "reboot_app": cmd_reboot_app,
}


def handle_request(line):
    """处理一行 JSON 请求，返回一行 JSON 响应"""
    try:
        req = json.loads(line)
    except Exception as e:
        return json.dumps({"ok": False, "msg": "JSON 解析失败: %s" % e})
    cmd = req.get("cmd")
    fn = DISPATCH.get(cmd)
    if fn is None:
        return json.dumps({"ok": False, "msg": "未知命令: %s" % cmd})
    try:
        res = fn(req)
    except Exception as e:
        res = {"ok": False, "msg": "执行异常: %s" % e}
    return json.dumps(res, ensure_ascii=False)


# ---------- BlueZ Profile1 (D-Bus) ----------
class BtProfile(dbus.service.Object):
    """org.bluez.Profile1 实现 — BlueZ 自动 accept 后通过 NewConnection 推 fd"""

    @dbus.service.method("org.bluez.Profile1", in_signature="", out_signature="")
    def Release(self):
        log.info("Profile Release")

    @dbus.service.method("org.bluez.Profile1", in_signature="oha{sv}", out_signature="")
    def NewConnection(self, path, fd, opts):
        # 注意: 实测 BlueZ 5.53 + dbus-python 实际传参顺序是 (path, fd, opts)，
        # 即第 1 个是 ObjectPath(设备路径)，第 2 个才是 UnixFd(文件描述符)。
        log.info("NewConnection path=%s fd=%r opts=%s", path, fd, opts)
        try:
            # dbus 对签名 'h' 传的是 dbus.UnixFd 包装对象，socket.fromfd 要裸 int
            if isinstance(fd, dbus.types.UnixFd):
                real_fd = fd.take()
            else:
                real_fd = int(fd)
            log.info("  real_fd=%s (path=%s)", real_fd, path)
            client = socket.fromfd(real_fd, socket.AF_BLUETOOTH, socket.SOCK_STREAM, 0)
            # 30s 不够：nmcli 连 WiFi 最长 70s（失败还要回滚），扫描 14s。
            # G1 原版是 30s，慢命令会来不及把响应写回去。客户端侧读超时是 60s+。
            client.settimeout(120)
            t = threading.Thread(target=self._serve_fd, args=(client, path), daemon=True)
            t.start()
        except Exception as e:
            log.warning("NewConnection fromfd 失败: %s", e)
            try:
                if isinstance(fd, dbus.types.UnixFd):
                    os.close(fd.take())
                else:
                    os.close(int(fd))
            except Exception:
                pass

    @dbus.service.method("org.bluez.Profile1", in_signature="o", out_signature="")
    def RequestDisconnection(self, path):
        log.info("RequestDisconnection %s", path)

    def _serve_fd(self, client, path):
        """服务 BlueZ 推来的 fd: 收一行 JSON → 回一行 JSON，保持连接"""
        try:
            data = client.recv(4096)
            if not data:
                return
            line = data.decode("utf-8", errors="replace").strip()
            log.info("← [%s]: %s", path, line)
            resp = handle_request(line)
            log.info("→ [%s]: %s", path, resp)
            client.send((resp + "\n").encode("utf-8"))
            buf = b""
            while True:
                data = client.recv(4096)
                if not data:
                    break
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.decode("utf-8", errors="replace").strip()
                    if not line:
                        continue
                    log.info("← [%s]: %s", path, line)
                    resp = handle_request(line)
                    log.info("→ [%s]: %s", path, resp)
                    client.send((resp + "\n").encode("utf-8"))
        except Exception as e:
            log.warning("serve_fd 异常: %s", e)
        finally:
            try:
                client.close()
            except Exception:
                pass
            log.info("连接关闭 [%s]", path)


# ---------- BlueZ Agent1 (无头自动配对) ----------
class PairingAgent(dbus.service.Object):
    """org.bluez.Agent1 实现 — 自动接受配对，无需交互终端。

    Android 发起配对时协商路径不同，全部自动应答：
      - Numeric Comparison（最常见，Android 显示 6 位数字）→ RequestConfirmation 自动确认
      - Passkey Entry / PIN → RequestPinCode/RequestPasskey 返回固定值
      - SPP 连接鉴权 → AuthorizeService 自动放行
    """
    IFACE = "org.bluez.Agent1"
    PATH = "/org/go2/NetBtCtlAgent"
    PIN = "1234"
    PASSKEY = 123456

    @dbus.service.method(IFACE, in_signature="", out_signature="")
    def Release(self):
        log.info("PairingAgent Release")

    @dbus.service.method(IFACE, in_signature="os", out_signature="")
    def AuthorizeService(self, device, uuid):
        log.info("AuthorizeService %s %s → 自动放行", device, uuid)

    @dbus.service.method(IFACE, in_signature="o", out_signature="s")
    def RequestPinCode(self, device):
        log.info("RequestPinCode %s → %s", device, self.PIN)
        return self.PIN

    @dbus.service.method(IFACE, in_signature="os", out_signature="")
    def DisplayPinCode(self, device, pincode):
        log.info("DisplayPinCode %s %s", device, pincode)

    @dbus.service.method(IFACE, in_signature="o", out_signature="u")
    def RequestPasskey(self, device):
        log.info("RequestPasskey %s → %s", device, self.PASSKEY)
        return dbus.UInt32(self.PASSKEY)

    @dbus.service.method(IFACE, in_signature="ouu", out_signature="")
    def DisplayPasskey(self, device, passkey, entered):
        log.info("DisplayPasskey %s %s entered=%s", device, passkey, entered)

    @dbus.service.method(IFACE, in_signature="ou", out_signature="")
    def RequestConfirmation(self, device, passkey):
        log.info("RequestConfirmation %s %s → 自动确认", device, passkey)

    @dbus.service.method(IFACE, in_signature="o", out_signature="")
    def RequestAuthorization(self, device):
        log.info("RequestAuthorization %s → 自动授权", device)

    @dbus.service.method(IFACE, in_signature="", out_signature="")
    def Cancel(self):
        log.info("PairingAgent Cancel")


def setup_adapter(bus):
    """把适配器设成手机能搜到、能配对的状态。

    改 /etc/bluetooth/main.conf 要重启 bluetoothd，会连带丢掉刚注册的 profile；
    这里用 D-Bus 运行时设置 Alias/Discoverable/Pairable，效果一样且不用重启。
    """
    try:
        props = dbus.Interface(
            bus.get_object("org.bluez", "/org/bluez/hci0"),
            "org.freedesktop.DBus.Properties")
        props.Set("org.bluez.Adapter1", "Powered", dbus.Boolean(True))
        props.Set("org.bluez.Adapter1", "Alias", dbus.String(ADAPTER_ALIAS))
        props.Set("org.bluez.Adapter1", "DiscoverableTimeout", dbus.UInt32(0))
        props.Set("org.bluez.Adapter1", "Discoverable", dbus.Boolean(True))
        props.Set("org.bluez.Adapter1", "PairableTimeout", dbus.UInt32(0))
        props.Set("org.bluez.Adapter1", "Pairable", dbus.Boolean(True))
        log.info("适配器已设为 %s（可被发现/可配对）", ADAPTER_ALIAS)
    except Exception as e:
        log.warning("设置适配器属性失败: %s", e)


def main():
    # 避免 systemd 环境下 Python 异常触发 apport → apt → SEGV
    def _eh(typ, val, tb):
        import traceback
        traceback.print_exception(typ, val, tb)
    sys.excepthook = _eh

    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()
    setup_adapter(bus)

    profile = BtProfile(bus, PROFILE_PATH)
    manager = dbus.Interface(
        bus.get_object("org.bluez", "/org/bluez"),
        "org.bluez.ProfileManager1")
    opts = {
        "Name": SERVICE_NAME,
        "Service": SPP_UUID,
        "Role": "server",
        "Channel": dbus.UInt16(RFCOMM_CHANNEL),
        "AutoConnect": dbus.Boolean(False),
    }
    manager.RegisterProfile(PROFILE_PATH, SPP_UUID, opts)
    log.info("已注册 SPP profile (UUID=%s, channel=%s)", SPP_UUID, RFCOMM_CHANNEL)

    agent = PairingAgent(bus, PairingAgent.PATH)
    agent_mgr = dbus.Interface(
        bus.get_object("org.bluez", "/org/bluez"),
        "org.bluez.AgentManager1")
    agent_mgr.RegisterAgent(PairingAgent.PATH, "KeyboardDisplay")
    agent_mgr.RequestDefaultAgent(PairingAgent.PATH)
    log.info("已注册无头自动配对 agent (KeyboardDisplay)")

    log.info("go2_net_btctl 守护进程就绪 (root)，等待蓝牙连接…")
    GLib.MainLoop().run()


if __name__ == "__main__":
    main()
