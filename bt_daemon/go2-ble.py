#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""go2-ble.py — 在 hci0 上开一个 BLE GATT 外设，让浏览器用 Web Bluetooth 直连。

为什么需要它：
  手机浏览器拿不到经典蓝牙（SPP），只有 Web Bluetooth（BLE）。App 走 SPP，
  网页走 BLE，两条通道说的是同一套协议：

      一行 JSON 请求  →  一行 JSON 响应   （\\n 分隔）

  命令实现直接复用 go2_net_btctl.handle_request，所以 scan/connect/ap/status
  的行为、返回字段和 SPP 通道完全一致。

比 SPP 多出来的好处：
  切 WiFi / 开 AP 时 HTTP 会断（网卡换网络），BLE 走的是另一条射频，不会断。
  所以用 BLE 下发切换命令、再拿新 IP，比走 HTTP 稳。

UUID 用 Nordic UART Service（NUS），手机端工具（nRF Connect 等）也能直接调：
  服务      6e400001-b5a3-f393-e0a9-e50e24dcca9e
  RX (写)   6e400002-b5a3-f393-e0a9-e50e24dcca9e   手机 → 机器人
  TX (通知) 6e400003-b5a3-f393-e0a9-e50e24dcca9e   机器人 → 手机

注意 BLE 的 MTU 很小（默认 23，载荷 20 字节），所以两个方向都要分片：
  - 写：浏览器按 20 字节切片发，本进程攒到 \\n 才当一条命令
  - 通知：本进程按 MTU-3 切片发，浏览器攒到 \\n 才当一条响应
"""
import os
import sys
import json
import time
import subprocess
import threading
import logging

import dbus
import dbus.service
import dbus.mainloop.glib
from gi.repository import GLib

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

# 复用 SPP 那套命令实现（nmcli / AP 脚本 / status）
import go2_net_btctl as btctl

BLUEZ = "org.bluez"
ADAPTER_PATH = "/org/bluez/hci0"
APP_PATH = "/org/go2/ble"
ADV_PATH = "/org/go2/ble/advertisement0"

SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
CCCD_UUID = "00002902-0000-1000-8000-00805f9b34fb"

LOCAL_NAME = "Unitree-Go2-BLE"
DEFAULT_CHUNK = 20          # MTU 23 - 3
MAX_REQUEST = 8192          # 防御：客户端不发 \n 时不无限攒
REQUEST_TIMEOUT_HINT = 120  # 仅用于日志说明

log = logging.getLogger("go2ble")
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)


# ---------------------------------------------------------------- GATT 对象
class Application(dbus.service.Object):
    """org.bluez.GattApplication1 —— BlueZ 通过 ObjectManager 拉取整棵 GATT 树"""

    def __init__(self, bus):
        self.services = []
        dbus.service.Object.__init__(self, bus, APP_PATH)

    def add_service(self, service):
        self.services.append(service)

    @dbus.service.method("org.freedesktop.DBus.ObjectManager",
                         in_signature="", out_signature="a{oa{sa{sv}}}")
    def GetManagedObjects(self):
        out = {}
        for svc in self.services:
            out[svc.path] = svc.get_properties()
            for chrc in svc.characteristics:
                out[chrc.path] = chrc.get_properties()
                for desc in chrc.descriptors:
                    out[desc.path] = desc.get_properties()
        return out


class Service(dbus.service.Object):
    def __init__(self, bus, index, uuid, primary):
        self.path = "%s/service%d" % (APP_PATH, index)
        self.uuid = uuid
        self.primary = primary
        self.characteristics = []
        dbus.service.Object.__init__(self, bus, self.path)

    def add_characteristic(self, chrc):
        self.characteristics.append(chrc)

    def get_properties(self):
        return {
            "org.bluez.GattService1": {
                "UUID": self.uuid,
                "Primary": dbus.Boolean(self.primary),
                "Characteristics": dbus.Array(
                    [c.path for c in self.characteristics], signature="o"),
            }
        }


class Characteristic(dbus.service.Object):
    def __init__(self, bus, index, uuid, flags, service):
        self.path = "%s/char%d" % (service.path, index)
        self.uuid = uuid
        self.flags = flags
        self.service = service
        self.descriptors = []
        self.value = b""
        self.notifying = False
        self.mtu = 23
        dbus.service.Object.__init__(self, bus, self.path)

    def add_descriptor(self, desc):
        self.descriptors.append(desc)

    def get_properties(self):
        return {
            "org.bluez.GattCharacteristic1": {
                "UUID": self.uuid,
                "Service": dbus.ObjectPath(self.service.path),
                "Flags": dbus.Array(self.flags, signature="s"),
                "Descriptors": dbus.Array([d.path for d in self.descriptors],
                                          signature="o"),
                "Value": dbus.ByteArray(self.value),
                "Notifying": dbus.Boolean(self.notifying),
            }
        }

    @dbus.service.method("org.bluez.GattCharacteristic1",
                         in_signature="a{sv}", out_signature="ay")
    def ReadValue(self, options):
        return dbus.ByteArray(self.value)

    @dbus.service.method("org.bluez.GattCharacteristic1",
                         in_signature="aya{sv}", out_signature="")
    def WriteValue(self, value, options):
        pass

    @dbus.service.method("org.bluez.GattCharacteristic1",
                         in_signature="", out_signature="")
    def StartNotify(self):
        self.notifying = True
        log.info("客户端已订阅通知 (%s)", self.uuid)

    @dbus.service.method("org.bluez.GattCharacteristic1",
                         in_signature="", out_signature="")
    def StopNotify(self):
        self.notifying = False
        log.info("客户端取消订阅 (%s)", self.uuid)

    @dbus.service.signal("org.freedesktop.DBus.Properties",
                         signature="sa{sv}as")
    def PropertiesChanged(self, interface, changed, invalidated):
        pass

    def notify(self, data):
        """按 MTU 切片发通知（BlueZ 不替我们分片）"""
        if not self.notifying:
            log.warning("TX 未订阅，丢弃 %d 字节响应", len(data))
            return
        size = max(DEFAULT_CHUNK, self.mtu - 3)
        for i in range(0, len(data), size):
            chunk = data[i:i + size]
            self.value = chunk
            self.PropertiesChanged(
                "org.bluez.GattCharacteristic1",
                {"Value": dbus.ByteArray(chunk)}, [])


class RxCharacteristic(Characteristic):
    """写特性：浏览器按 MTU 分片写进来，这里转给 Bridge 攒行。

    dbus-python 的方法在导出时就固定了，不能事后替换实例方法，
    所以用子类 + 回调，而不是给 Characteristic.WriteValue 打补丁。
    """

    def __init__(self, bus, index, uuid, flags, service, on_data):
        self.on_data = on_data
        Characteristic.__init__(self, bus, index, uuid, flags, service)

    @dbus.service.method("org.bluez.GattCharacteristic1",
                         in_signature="aya{sv}", out_signature="")
    def WriteValue(self, value, options):
        self.on_data(bytes(value), options.get("mtu", 23))


class Descriptor(dbus.service.Object):
    """CCCD（0x2902）—— notify 特性必须有它，BlueZ 才会走 StartNotify"""

    def __init__(self, bus, index, uuid, flags, characteristic):
        self.path = "%s/desc%d" % (characteristic.path, index)
        self.uuid = uuid
        self.flags = flags
        self.characteristic = characteristic
        self.value = b""
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        return {
            "org.bluez.GattDescriptor1": {
                "UUID": self.uuid,
                "Characteristic": dbus.ObjectPath(self.characteristic.path),
                "Flags": dbus.Array(self.flags, signature="s"),
                "Value": dbus.ByteArray(self.value),
            }
        }

    @dbus.service.method("org.bluez.GattDescriptor1",
                         in_signature="a{sv}", out_signature="ay")
    def ReadValue(self, options):
        return dbus.ByteArray(self.value)

    @dbus.service.method("org.bluez.GattDescriptor1",
                         in_signature="aya{sv}", out_signature="")
    def WriteValue(self, value, options):
        self.value = bytes(value)


class Advertisement(dbus.service.Object):
    def __init__(self, bus, index, advertising_type):
        self.path = "%s/advertisement%d" % (APP_PATH, index)
        self.ad_type = advertising_type
        self.service_uuids = None
        self.local_name = None
        self.include_tx_power = False
        self.manufacturer_data = None
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        props = {"Type": self.ad_type}
        if self.service_uuids:
            props["ServiceUUIDs"] = dbus.Array(self.service_uuids, signature="s")
        if self.local_name:
            props["LocalName"] = dbus.String(self.local_name)
        if self.include_tx_power:
            props["Includes"] = dbus.Array(["tx-power"], signature="s")
        if self.manufacturer_data is not None:
            props["ManufacturerData"] = dbus.Dictionary(
                self.manufacturer_data, signature="qv")
        return {"org.bluez.LEAdvertisement1": props}

    @dbus.service.method("org.freedesktop.DBus.Properties",
                         in_signature="ss", out_signature="v")
    def Get(self, interface, prop):
        props = self.get_properties().get(interface)
        if props is None or prop not in props:
            raise dbus.exceptions.DBusException(
                "org.freedesktop.DBus.Error.InvalidArgs",
                "No such property %s" % prop)
        return props[prop]

    @dbus.service.method("org.freedesktop.DBus.Properties",
                         in_signature="s", out_signature="a{sv}")
    def GetAll(self, interface):
        """BlueZ 通过 GetAll 读 LEAdvertisement1 的属性来拼广播数据。

        少了这个，RegisterAdvertisement 仍会成功，但广播数据是空的
        （btmon 里 Add Advertising 的 data length = 0），手机端扫不到名字和 UUID。
        """
        props = self.get_properties().get(interface)
        if props is None:
            return {}
        return props

    @dbus.service.method("org.bluez.LEAdvertisement1",
                         in_signature="", out_signature="")
    def Release(self):
        # BlueZ 会在适配器复位、bluetoothd 重启等情况下主动释放广播。
        # 只记日志的话广播就永久消失了（手机侧表现为"能选到设备但连不上"），
        # 所以这里必须回调重新注册。
        log.info("LE 广播被 BlueZ 释放，稍后重新注册")
        cb = getattr(self, "on_release", None)
        if cb is not None:
            cb()


# ---------------------------------------------------------------- 协议收发
class Bridge(object):
    """把 GATT 写/通知接到 handle_request 上（一行 JSON 进，一行 JSON 出）"""

    def __init__(self):
        self.buf = bytearray()
        self.lock = threading.Lock()
        self.tx = None

    def on_write(self, data, mtu):
        if self.tx is not None and mtu:
            self.tx.mtu = int(mtu)
        with self.lock:
            self.buf.extend(data)
            if len(self.buf) > MAX_REQUEST:
                log.warning("请求缓冲超限（%d 字节且无换行），丢弃", len(self.buf))
                self.buf = bytearray()
                return
            while True:
                idx = self.buf.find(b"\n")
                if idx < 0:
                    break
                line = bytes(self.buf[:idx])
                del self.buf[:idx + 1]
                if line.strip():
                    self._dispatch(line)

    def _dispatch(self, line):
        text = line.decode("utf-8", "replace").strip()
        log.info("← %s", text)
        # 命令可能跑几十秒（nmcli 连接 70s），绝不能阻塞 D-Bus 主循环
        threading.Thread(target=self._run, args=(text,), daemon=True).start()

    def _run(self, text):
        try:
            resp = btctl.handle_request(text)
        except Exception as e:
            resp = json.dumps({"ok": False, "msg": "内部错误: %s" % e},
                              ensure_ascii=False)
        log.info("→ %s", resp[:400])
        GLib.idle_add(self._send, (resp + "\n").encode("utf-8"))

    def _send(self, data):
        if self.tx is not None:
            self.tx.notify(data)
        return False  # idle_add 只跑一次


# ---------------------------------------------------------------- 启动
def main():
    def _eh(typ, val, tb):
        import traceback
        traceback.print_exception(typ, val, tb)
    sys.excepthook = _eh

    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()

    bridge = Bridge()

    app = Application(bus)
    service = Service(bus, 0, SERVICE_UUID, True)
    app.add_service(service)

    rx = RxCharacteristic(bus, 0, RX_UUID, ["write", "write-without-response"],
                          service, bridge.on_write)
    tx = Characteristic(bus, 1, TX_UUID, ["notify", "read"], service)
    tx.add_descriptor(Descriptor(bus, 0, CCCD_UUID, ["read", "write"], tx))
    service.add_characteristic(rx)
    service.add_characteristic(tx)
    bridge.tx = tx

    adv = Advertisement(bus, 0, "peripheral")
    adv.service_uuids = [SERVICE_UUID]
    # 名字可选：带上 LocalName 时 BlueZ 会要求"作为可发现设备广播"，
    # 而适配器本身 Discoverable=yes 已占住那个槽位，会报 Busy 注册失败。
    # 用环境变量控制，方便现场取舍。
    if os.environ.get("GO2_BLE_NAME", "1") == "1":
        adv.local_name = LOCAL_NAME
    adv.include_tx_power = True

    state = {"gatt": False, "adv": False,
             "gatt_retry": False, "adv_retry": False,
             "adv_tries": 0, "power_cycle": False,
             "rebuilding": False, "adv_at": 0.0}

    def _iface(name):
        return dbus.Interface(bus.get_object(BLUEZ, ADAPTER_PATH), name)

    def _aprop():
        return dbus.Interface(bus.get_object(BLUEZ, ADAPTER_PATH),
                              "org.freedesktop.DBus.Properties")

    # ---- GATT 应用 ----
    def gatt_registered():
        state["gatt"] = True
        state["gatt_retry"] = False
        log.info("GATT 应用已注册（%s）", APP_PATH)

    def gatt_failed(e):
        state["gatt"] = False
        msg = str(e)
        if "AlreadyExists" in msg:
            # 重启时上一轮进程的注册还挂在 BlueZ 里。主动摘掉再注册，
            # 否则会一直 AlreadyExists（旧对象其实已经没人服务了）。
            try:
                _iface("org.bluez.GattManager1").UnregisterApplication(APP_PATH)
                log.info("已摘除残留 GATT 注册")
            except dbus.exceptions.DBusException as ex:
                log.info("摘除残留 GATT 忽略: %s", ex.get_dbus_name())
        if state["gatt_retry"]:
            return
        state["gatt_retry"] = True
        delay = 2000 if "AlreadyExists" in msg else 3000
        log.warning("GATT 注册失败，%ds 后重试: %s", delay // 1000, e)
        GLib.timeout_add(delay, lambda: (register_gatt(), False)[1])

    def register_gatt():
        state["gatt_retry"] = False
        try:
            mgr = _iface("org.bluez.GattManager1")
        except dbus.exceptions.DBusException as e:
            state["gatt_retry"] = True
            log.warning("拿不到 GATT 管理器（bluetoothd 未就绪？）: %s", e)
            GLib.timeout_add(3000, lambda: (register_gatt(), False)[1])
            return
        mgr.RegisterApplication(APP_PATH, {},
                                reply_handler=gatt_registered,
                                error_handler=gatt_failed)

    # ---- LE 广播 ----
    def adv_registered():
        state["adv"] = True
        state["adv_retry"] = False
        state["adv_tries"] = 0
        state["adv_at"] = time.time()
        log.info("LE 广播已开启：%s (%s)", LOCAL_NAME, SERVICE_UUID)

    def adv_failed(e):
        state["adv"] = False
        state["adv_tries"] += 1
        n = state["adv_tries"]
        log.warning("LE 广播注册失败（第 %d 次）：%s", n, e)
        if state["adv_retry"]:
            return
        state["adv_retry"] = True
        delay = min(30, 2 * n)
        GLib.timeout_add(delay * 1000, lambda: (adv_recover(n), False)[1])

    def adv_recover(n):
        """清掉内核/BlueZ 里的残留广播注册再重试。"""
        state["adv_retry"] = False
        try:
            subprocess.run(["btmgmt", "clr-adv"], capture_output=True, timeout=10)
        except Exception:
            pass
        try:
            _iface("org.bluez.LEAdvertisingManager1").UnregisterAdvertisement(
                ADV_PATH)
            log.info("已摘除残留广播注册")
        except dbus.exceptions.DBusException:
            pass
        if n >= 4 and not state["power_cycle"]:
            power_cycle(register_adv)
        else:
            register_adv()

    def register_adv():
        state["adv_retry"] = False
        try:
            mgr = _iface("org.bluez.LEAdvertisingManager1")
        except dbus.exceptions.DBusException as e:
            state["adv_retry"] = True
            log.warning("拿不到广播管理器: %s", e)
            GLib.timeout_add(3000, lambda: (register_adv(), False)[1])
            return
        mgr.RegisterAdvertisement(adv.path, {},
                                  reply_handler=adv_registered,
                                  error_handler=adv_failed)

    def power_cycle(then):
        """适配器电源循环。必须保证最后重新上电，否则整个蓝牙都没了。"""
        state["power_cycle"] = True
        log.warning("广播连续失败，重启适配器电源")

        def step_off():
            try:
                _aprop().Set("org.bluez.Adapter1", "Powered", dbus.Boolean(False))
            except Exception as ex:
                log.warning("适配器断电失败: %s", ex)
            GLib.timeout_add(1500, step_on)
            return False

        def step_on():
            try:
                _aprop().Set("org.bluez.Adapter1", "Powered", dbus.Boolean(True))
            except Exception as ex:
                log.warning("适配器上电失败: %s", ex)
            GLib.timeout_add(3000, verify_on)
            return False

        def verify_on():
            powered = False
            try:
                powered = bool(_aprop().Get("org.bluez.Adapter1", "Powered"))
            except Exception:
                pass
            if not powered:
                log.error("适配器仍断电，强制上电")
                try:
                    _aprop().Set("org.bluez.Adapter1", "Powered",
                                 dbus.Boolean(True))
                except Exception as ex:
                    log.error("强制上电失败: %s", ex)
            for prop, val in (("Discoverable", True), ("Pairable", True)):
                try:
                    _aprop().Set("org.bluez.Adapter1", prop, dbus.Boolean(val))
                except Exception:
                    pass
            state["power_cycle"] = False
            then()
            return False

        GLib.timeout_add(300, step_off)

    def on_adv_release():
        # 我们自己主动摘除时（rebuild_adv）不需要再排一次注册
        if state["rebuilding"]:
            return
        state["adv"] = False
        GLib.timeout_add(1000, lambda: (register_adv(), False)[1])

    adv.on_release = on_adv_release

    def rebuild_adv(reason):
        """把广播重新下发到控制器。

        适配器断电/上电后，BlueZ 会把注册留在自己的表里（ActiveInstances 仍是 1，
        重复 RegisterAdvertisement 只回 AlreadyExists），但内核里那条广播已经没了。
        只有 Unregister + Register 才能让它重新下发，所以确定性刷新只能这么做。
        """
        if state["rebuilding"] or state["power_cycle"]:
            return
        state["rebuilding"] = True
        log.info("重新下发广播（%s）", reason)

        def do_register():
            state["rebuilding"] = False
            register_adv()
            return False

        def do_unregister():
            try:
                _iface("org.bluez.LEAdvertisingManager1").UnregisterAdvertisement(
                    ADV_PATH,
                    reply_handler=lambda: GLib.timeout_add(400, do_register),
                    error_handler=lambda e: do_register())
            except dbus.exceptions.DBusException as e:
                log.info("摘除广播时出错（继续注册）: %s", e.get_dbus_name())
                do_register()
            return False

        GLib.timeout_add(10, do_unregister)

    def probe_adv():
        """广播存活自检：重复注册能补回被 BlueZ 丢掉注册的情况。"""
        if state["power_cycle"] or state["adv_retry"] or state["rebuilding"]:
            return
        try:
            mgr = _iface("org.bluez.LEAdvertisingManager1")
        except dbus.exceptions.DBusException as e:
            log.warning("广播自检拿不到管理器: %s", e)
            return

        def ok():
            state["adv"] = True
            state["adv_at"] = time.time()
            state["adv_tries"] = 0
            log.info("广播自检：此前已丢失，现已重新注册")

        def err(e):
            if "AlreadyExists" in str(e):
                state["adv"] = True
                return
            adv_failed(e)

        mgr.RegisterAdvertisement(adv.path, {}, reply_handler=ok,
                                  error_handler=err)

    def watchdog():
        probe_adv()
        # 注册表还在但内核没在广播的情况 D-Bus 看不出来（ActiveInstances
        # 仍是 1、重复注册只回 AlreadyExists），所以隔一段时间主动重新下发。
        # 间隔取 5 分钟：既能兜住未知失效，又不会频繁制造广播空窗。
        if (not state["power_cycle"] and not state["rebuilding"]
                and time.time() - state["adv_at"] > 300):
            rebuild_adv("定期刷新")
        if not state["gatt"] and not state["gatt_retry"]:
            register_gatt()
        return True

    GLib.timeout_add_seconds(15, watchdog)

    # 适配器重新上电后立刻重新下发广播（这是最常见的失效场景）
    def on_adapter_props(iface, changed, invalidated=None, **kw):
        if iface != "org.bluez.Adapter1":
            return
        if "Powered" in changed and bool(changed["Powered"]):
            log.info("适配器已上电，1s 后重新下发广播")
            state["adv"] = False
            GLib.timeout_add(1000, lambda: (rebuild_adv("适配器上电"), False)[1])

    bus.add_signal_receiver(on_adapter_props,
                            signal_name="PropertiesChanged",
                            dbus_interface="org.freedesktop.DBus.Properties",
                            path=ADAPTER_PATH)

    register_gatt()
    register_adv()

    def on_owner_changed(name, old_owner, new_owner):
        if name != BLUEZ:
            return
        if new_owner:
            log.info("bluetoothd 已重启（%s），5s 后重新注册", new_owner)
            state["gatt"] = False
            state["adv"] = False
            state["adv_tries"] = 0
            GLib.timeout_add(5000, lambda: (register_gatt(), register_adv(), False)[2])
        else:
            log.warning("bluetoothd 退出，等待其恢复")

    bus.add_signal_receiver(
        on_owner_changed,
        signal_name="NameOwnerChanged",
        dbus_interface="org.freedesktop.DBus",
        arg0=BLUEZ,
    )

    log.info("go2-ble 就绪，等待浏览器（Web Bluetooth）连接…")
    GLib.MainLoop().run()


if __name__ == "__main__":
    main()
