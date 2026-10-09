#!/usr/bin/env python3
"""Host-side network helper for the Go2 web console.

The web app runs inside Docker (host network, but no nmcli/dbus), so it cannot
talk to NetworkManager itself. Instead it drops request files into
<go2>/net-ctl/queue/ and reads <go2>/net-ctl/status.json + results/<nonce>.json.

This helper runs as root (via a systemd timer) and:
  - refreshes net-ctl/status.json (mode, wlan0/eth0 state, IPs)
  - executes queued requests: scan / set_mode (ap|wifi)

Usage:
    go2-net-helper.py tick     # refresh status + process the queue
"""
import glob
import json
import os
import subprocess
import sys
import time
import uuid

GO2 = "/home/unitree/hu/go2"
CONFIG = os.path.join(GO2, "web", "config.json")
NET = os.path.join(GO2, "net-ctl")
QUEUE = os.path.join(NET, "queue")
RESULTS = os.path.join(NET, "results")
STATUS = os.path.join(NET, "status.json")
IFACE = "wlan0"
ETH = "eth0"
NM_DIR = "/etc/NetworkManager/system-connections"


def log(msg):
    print("[go2-net-helper] %s" % msg, flush=True)


def run(args, timeout=45):
    try:
        p = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
        return p.returncode, p.stdout.strip(), p.stderr.strip()
    except Exception as e:
        return 1, "", str(e)


def load_cfg():
    try:
        with open(CONFIG, encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {}


def nm_dev(iface):
    """Return {'state','conn','ip','gw'} for an interface via nmcli."""
    d = {"state": "", "conn": "", "ip": "", "gw": ""}
    rc, out, _ = run(["nmcli", "-t", "-f",
                      "GENERAL.STATE,GENERAL.CONNECTION,IP4.ADDRESS,IP4.GATEWAY",
                      "dev", "show", iface], timeout=15)
    for line in out.splitlines():
        if line.startswith("GENERAL.STATE:"):
            d["state"] = line.split(":", 1)[1].strip()
        elif line.startswith("GENERAL.CONNECTION:"):
            d["conn"] = line.split(":", 1)[1].strip()
        elif line.startswith("IP4.ADDRESS") and not d["ip"]:
            d["ip"] = line.split(":", 1)[1].split("/")[0].strip()
        elif line.startswith("IP4.GATEWAY:"):
            d["gw"] = line.split(":", 1)[1].strip()
    return d


def carrier(iface):
    try:
        with open("/sys/class/net/%s/carrier" % iface) as f:
            return f.read().strip()
    except Exception:
        return "0"


def atomic_write(path, data):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False)
    os.replace(tmp, path)


def write_status():
    cfg = load_cfg()
    mode = cfg.get("net", {}).get("mode", "ap")
    w = nm_dev(IFACE)
    e = nm_dev(ETH)
    eth_carrier = carrier(ETH)
    # robot IP: over the cable it's the configured internal address; on the Go2
    # hotspot the robot IS the AP, i.e. wlan0's gateway.
    if eth_carrier == "1":
        robot_ip = cfg.get("net", {}).get("robot_ip_eth", "192.168.123.161")
    elif w["gw"]:
        robot_ip = w["gw"]
    else:
        robot_ip = ""
    jetson_ip = w["ip"] or e["ip"]
    atomic_write(STATUS, {
        "mode": mode,
        "wlan0_state": w["state"], "wlan0_ssid": w["conn"],
        "wlan0_ip": w["ip"], "wlan0_gw": w["gw"],
        "eth0_state": e["state"], "eth0_ip": e["ip"], "eth0_carrier": eth_carrier,
        "jetson_ip": jetson_ip, "robot_ip": robot_ip,
        "ts": time.time(),
    })


def write_result(nonce, res):
    atomic_write(os.path.join(RESULTS, nonce + ".json"), res)


def do_scan():
    run(["nmcli", "dev", "wifi", "rescan"], timeout=20)
    time.sleep(3)
    rc, out, err = run(["nmcli", "-t", "-f", "SSID,SIGNAL,SECURITY",
                        "dev", "wifi", "list"], timeout=20)
    if rc != 0:
        return {"ok": False, "error": err or "nmcli scan failed"}
    ssids, seen = [], set()
    for line in out.splitlines():
        # nmcli -t escapes ':' inside values as '\:'
        parts = line.replace("\\:", "\x00").split(":")
        parts = [p.replace("\x00", ":") for p in parts]
        if len(parts) < 3:
            continue
        ssid = parts[0].strip()
        if not ssid or ssid in seen:
            continue
        seen.add(ssid)
        try:
            sig = int(parts[1])
        except ValueError:
            sig = 0
        ssids.append({"ssid": ssid, "signal": sig, "security": ":".join(parts[2:]).strip()})
    ssids.sort(key=lambda s: -s["signal"])
    return {"ok": True, "ssids": ssids[:40]}


def write_keyfile(profile, ssid, psk):
    """Write a NetworkManager keyfile directly (0600) so the PSK never appears
    on a command line / in `ps`."""
    path = os.path.join(NM_DIR, "%s.nmconnection" % profile)
    sec = ""
    if psk:
        sec = "key-mgmt=wpa-psk\npsk=%s\n" % psk
    else:
        sec = "key-mgmt=none\n"
    content = (
        "[connection]\n"
        "id=%s\nuuid=%s\ntype=wifi\ninterface-name=%s\nautoconnect-priority=90\n\n"
        "[wifi]\nmode=infrastructure\nssid=%s\n\n"
        "[wifi-security]\n%s\n"
        "[ipv4]\nmethod=auto\n\n"
        "[ipv6]\nmethod=auto\n"
    ) % (profile, str(uuid.uuid4()), IFACE, ssid, sec)
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write(content)


def ensure_ap_profile(net):
    """Create the Go2-AP profile from config if it doesn't exist yet."""
    profile = net.get("ap_profile", "Go2-AP")
    ssid = net.get("ap_ssid", "Go2_Asano")
    psk = net.get("ap_psk", "")
    rc, out, _ = run(["nmcli", "-t", "-f", "NAME", "con", "show"], timeout=15)
    if profile in [n for n in out.splitlines()]:
        return profile
    args = ["nmcli", "con", "add", "type", "wifi", "con-name", profile,
            "ifname", IFACE, "ssid", ssid]
    if psk:
        args += ["wifi-sec.key-mgmt", "wpa-psk", "wifi-sec.psk", psk]
    rc, out, err = run(args, timeout=20)
    if rc != 0:
        log("failed to create AP profile: %s" % err)
        return None
    # do NOT autoconnect by default: the Go2 hotspot may be off, and we do not
    # want the Jetson to fight for it on every boot. Flip this to yes (and set
    # a priority) once the hotspot is confirmed to work.
    run(["nmcli", "con", "modify", profile,
         "connection.autoconnect", "no"], timeout=15)
    return profile


def do_set_mode(req):
    cfg = load_cfg()
    net = cfg.get("net", {})
    mode = req.get("mode")
    if mode == "ap":
        profile = ensure_ap_profile(net)
        if not profile:
            return {"ok": False, "error": "无法创建机器狗热点配置（SSID/密码见 config.json net）"}
    elif mode == "wifi":
        ssid = str(req.get("ssid", "")).strip()
        psk = str(req.get("password", ""))
        if not ssid:
            return {"ok": False, "error": "SSID 为空"}
        profile = net.get("wifi_profile", "go2-wifi-ext")
        write_keyfile(profile, ssid, psk)
        run(["nmcli", "con", "reload"], timeout=15)
    else:
        return {"ok": False, "error": "未知模式"}
    rc, out, err = run(["nmcli", "con", "up", profile], timeout=60)
    if rc != 0:
        return {"ok": False, "mode": mode, "error": err or "连接失败"}
    return {"ok": True, "mode": mode}


def process_queue():
    os.makedirs(QUEUE, exist_ok=True)
    os.makedirs(RESULTS, exist_ok=True)
    for path in sorted(glob.glob(os.path.join(QUEUE, "*.json"))):
        try:
            with open(path, encoding="utf-8") as f:
                req = json.load(f)
        except Exception:
            os.remove(path)
            continue
        try:
            os.remove(path)  # may contain a password; drop it immediately
        except OSError:
            pass
        nonce = str(req.get("nonce", ""))
        if not nonce:
            continue
        if time.time() - float(req.get("ts", 0)) > 60:
            write_result(nonce, {"ok": False, "error": "请求超时（已忽略）"})
            continue
        op = req.get("op")
        log("processing %s (%s)" % (op, nonce))
        if op == "scan":
            res = do_scan()
        elif op == "set_mode":
            res = do_set_mode(req)
        else:
            res = {"ok": False, "error": "未知操作"}
        write_result(nonce, res)
    # prune stale result files (>10 min)
    now = time.time()
    for path in glob.glob(os.path.join(RESULTS, "*.json")):
        try:
            if now - os.path.getmtime(path) > 600:
                os.remove(path)
        except OSError:
            pass


def main():
    os.makedirs(QUEUE, exist_ok=True)
    os.makedirs(RESULTS, exist_ok=True)
    if len(sys.argv) > 1 and sys.argv[1] == "status":
        write_status()
        return
    process_queue()
    write_status()


if __name__ == "__main__":
    main()
