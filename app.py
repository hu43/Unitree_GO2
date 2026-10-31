#!/usr/bin/env python3
"""Go2 web console backend.

- Serves the frontend (static/)
- WebSocket: pushes map/robot state to clients, receives control commands
- Tour state machine: send waypoint goals in sequence, advance on arrival

Lives entirely inside hu/go2/web. Talks to the running SCAN-Planner stack
through ros_bridge.RosBridge over standard ROS2 topics.
"""

import asyncio
import base64
import faulthandler
import json
import logging
import math
import os
import re
import signal
import ssl
import subprocess
import sys
import threading
import time
import urllib.request

import aiohttp

# 诊断：每 20s 把所有线程栈 dump 到 threads.log（定位阻塞用）
_fh = open("/home/unitree/hu/go2/threads.log", "w")
faulthandler.enable()  # 段错误/abort 等致命信号转储到 stderr -> web.log
faulthandler.dump_traceback_later(20, repeat=True, file=_fh)

import websockets
from websockets.datastructures import Headers
from websockets.http11 import Response as WsResponse

import ros_bridge as rb
import tts
import maprender
from odom_bridge_health import is_odom_bridge_healthy

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(BASE_DIR, "config.json")
STATIC_DIR = os.path.join(BASE_DIR, "static")
UPLOAD_DIR = os.path.join(BASE_DIR, "uploads")

log = logging.getLogger("go2web")

DEFAULT_CONFIG = {
    "host": "0.0.0.0",
    "port": 8080,
    "topics": {
        "map": "/grid_map/occupancy",
        # 里程计源：/sportmodestate 由 sport_odom_bridge.py 转成 Odometry
        "odom": "/sportmodestate_odom",
        "goal": "/move_base_simple/goal",
        "cmd_vel": "/cmd_vel",
        "sport_request": "/api/sport/request",
    },
    "map_downsample": 4000,
    "push_hz": 10,
    "reach_threshold": 0.4,
    "max_speed": 0.5,
    "waypoints": [],
}


def load_config():
    if os.path.exists(CONFIG_PATH):
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            data = json.load(f)
        merged = dict(DEFAULT_CONFIG)
        merged.update(data)
        return merged
    return dict(DEFAULT_CONFIG)


def save_config(cfg):
    with open(CONFIG_PATH, "w", encoding="utf-8") as f:
        json.dump(cfg, f, ensure_ascii=False, indent=2)


def _reexec():
    """Replace this process with a fresh app.py (same env, same PID).

    The rclpy/CycloneDDS participant can come up broken (e.g. the app gets no
    odom while a fresh node sees it at full rate); re-exec re-initialises it
    without disturbing the robot link or SCAN-Planner.
    """
    try:
        os.execv(sys.executable, [sys.executable, os.path.abspath(__file__)])
    except Exception as e:
        log.error("re-exec failed: %s", e)
        os._exit(1)  # let the host watchdog respawn us


def persistent_map_enabled(cfg):
    """持久地图（slam_toolbox + map 系点位）总开关。

    置 false = 回到 10-13 的纯 odom 模式：点位与画布都用内置里程计坐标，
    不启动 slam_toolbox，也不做 odom<->map 换算。config.json 里的 maps
    数据原样保留，把 slam.persistent_map 改回 true 即可恢复。
    """
    return bool((cfg.get("slam") or {}).get("persistent_map", True))


def current_map_name(cfg, slam=None):
    """当前选中的持久地图名。以 cfg.slam.current_map 为准（用户在控制台
    下拉框选的），没有再退到 slam.map_name（定位进程记录的）。"""
    if not persistent_map_enabled(cfg):
        return ""
    name = (cfg.get("slam") or {}).get("current_map") or ""
    if name:
        return name
    if slam is not None and slam.map_name:
        return slam.map_name
    return ""


def set_current_map(cfg, name):
    cfg.setdefault("slam", {})["current_map"] = name
    if name:
        cfg["slam"]["last_map"] = name


def ensure_current_map(cfg):
    """开机时保证 current_map 有值：last_map -> 第一张可用地图。"""
    if not persistent_map_enabled(cfg):
        return
    if current_map_name(cfg):
        return
    name = (cfg.get("slam") or {}).get("last_map") or ""
    if not name or not os.path.exists(os.path.join(SLAM_DIR, name + ".pgm")):
        try:
            names = sorted(f[:-len(".pgm")] for f in os.listdir(SLAM_DIR)
                           if f.endswith(".pgm"))
        except OSError:
            names = []
        name = names[0] if names else ""
    if name:
        set_current_map(cfg, name)
        save_config(cfg)
        log.info("slam: current_map -> %s", name)


def active_waypoints(cfg, bridge, slam):
    """当前生效的点位库：选中了持久地图 -> 该地图的 map 系点位；
    一张持久地图都没有时才退回旧版内置 odom 点位（旧数据，UI 已标注）。

    返回 (waypoints列表, 系别)。
    """
    name = current_map_name(cfg, slam)
    if name:
        maps = cfg.setdefault("maps", {})
        store = maps.setdefault(name, {"waypoints": []})
        return store.setdefault("waypoints", []), "map"
    return cfg.setdefault("waypoints", []), "odom"


# --------------------------------------------------------------------------
# SLAM（slam_toolbox 持久地图）
# --------------------------------------------------------------------------
SLAM_DIR = "/home/unitree/hu/go2/slam_maps"
SLAM_LAUNCH_DIR = os.path.join(BASE_DIR, "slam")
ODOM_TF_BIN = ("/home/unitree/hu/go2/install/go2_cmd_vel_bridge"
               "/lib/go2_cmd_vel_bridge/odom_tf_broadcaster")


class SlamManager:
    """slam_toolbox 建图/定位子进程管理。

    漂移问题的解法：定位激活时 ros_bridge 从 /tf 拿 map->odom，导览点位
    以 map 系持久化在 cfg["maps"][地图名]["waypoints"]，存点/发目标由
    后端做 odom<->map 换算——里程计漂移多少都不影响点位对应物理位置。

    依赖 odom->base_link TF（odom_tf_broadcaster）：SCAN-Planner 栈在跑
    就复用，没有才自行拉起一个。
    """

    def __init__(self, bridge, cfg):
        self.bridge = bridge
        self.cfg = cfg
        self._odom_bridge_lock = threading.Lock()
        self.mode = "off"       # off | mapping | localization
        self.map_name = ""
        self._last_error = ""

    def _restore_map(self):
        """重启后恢复"当前使用的地图"，并在允许时自动恢复定位。

        有地图但没定位时，map 系没有实时的 map->odom，点位无法正确
        落在画布上；因此默认自动起定位（用户点过"停止定位"则不自动起）。
        """
        if not persistent_map_enabled(self.cfg):
            return
        if self.map_name:
            return
        name = (self.cfg.get("slam", {}).get("current_map")
                or self.cfg.get("slam", {}).get("last_map") or "")
        if not name:
            return
        self._set_state(self.mode, name)
        log.info("slam: restored current map (%s)", name)
        if self.mode == "off" and self.cfg.get("slam", {}).get("auto_localize", True):
            if os.path.exists(os.path.join(SLAM_DIR, name + ".posegraph")):
                log.info("slam: auto-starting localization (%s)", name)
                self._bg(self._do_start_localization, name, None)

    def _adopt_running(self):
        """app 重启（systemd/看门狗）后，slam 子进程还在容器里活着——
        探测并接管状态，避免双实例和状态错乱。"""
        if not persistent_map_enabled(self.cfg):
            return
        resume = self.cfg.get("slam", {}).get("resume", "")
        if self._alive("localization_slam_toolbox") and resume:
            self._set_state("localization", resume)
            log.info("slam: adopted running localization (%s)", resume)
        elif self._alive("async_slam_toolbox"):
            self._set_state("mapping")
            log.info("slam: adopted running mapping session")

    # ---- 进程工具 ----
    @staticmethod
    def _pkill(*patterns):
        for pat in patterns:
            subprocess.run(["pkill", "-f", pat], check=False,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    @staticmethod
    def _alive(pattern):
        r = subprocess.run(["pgrep", "-f", pattern], capture_output=True)
        return r.returncode == 0

    def _spawn(self, cmd, logfile, extra_env=None):
        env = dict(os.environ)
        if extra_env:
            env.update(extra_env)
        f = open(logfile, "a")
        p = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT,
                             env=env, start_new_session=True)
        f.close()
        return p

    def _odom_topic(self):
        return self.cfg.get("topics", {}).get("odom", "/sportmodestate_odom")

    def _existing_tf_ok(self):
        """已有 odom_tf_broadcaster 是否已经在用配置的里程计话题。

        来自 SCAN-Planner launch 的实例把话题写在 --params-file 里，需要读文件
        判断；否则直接看命令行。用对了就复用——误杀它会连带 launch 树退出。
        """
        topic = self._odom_topic()
        r = subprocess.run(["pgrep", "-af", "odom_tf_broadcaster"],
                           capture_output=True, text=True)
        for line in r.stdout.splitlines():
            if "pgrep" in line:
                continue
            if topic in line:
                return True
            m = re.search(r"--params-file\s+(\S+)", line)
            if m and os.path.exists(m.group(1)):
                try:
                    if topic in open(m.group(1), errors="replace").read():
                        return True
                except OSError:
                    pass
        return False

    def _ensure_odom_tf(self):
        # TF 必须和 web/点位使用同一个里程计源，否则目标点会落在错误的坐标系。
        if self._existing_tf_ok():
            log.info("slam: reuse odom_tf_broadcaster (%s)", self._odom_topic())
            return
        self._pkill("odom_tf_broadcaster")
        time.sleep(0.5)
        self._spawn(
            [ODOM_TF_BIN, "--ros-args", "-r", "__node:=odom_tf_broadcaster",
             "-p", f"odom_topic:={self._odom_topic()}",
             "-p", "odom_frame:=odom", "-p", "child_frame:=base_link",
             "-p", "publish_rate:=50.0"],
            os.path.join(SLAM_DIR, "odom_tf.log"))
        log.info("slam: odom_tf_broadcaster -> %s", self._odom_topic())

    @staticmethod
    def _odom_bridge_pids():
        """Find bridge processes by their exact script argument."""
        script = os.path.realpath(os.path.join(BASE_DIR, "sport_odom_bridge.py"))
        pids = []
        try:
            entries = os.listdir("/proc")
        except OSError:
            return pids
        for entry in entries:
            if not entry.isdigit():
                continue
            try:
                with open(f"/proc/{entry}/cmdline", "rb") as f:
                    args = [os.fsdecode(arg) for arg in f.read().split(b"\0") if arg]
            except OSError:
                continue
            if script in args:
                pids.append(int(entry))
        return pids

    def _stop_odom_bridges(self):
        """Stop bridge instances that may be attached to an obsolete DDS network."""
        pids = self._odom_bridge_pids()
        for pid in pids:
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            except OSError as e:
                log.warning("slam: cannot stop stale odom bridge pid %s: %s", pid, e)

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            remaining = [pid for pid in pids if os.path.exists(f"/proc/{pid}")]
            if not remaining:
                return
            time.sleep(0.05)
        for pid in remaining:
            try:
                os.kill(pid, signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass

    def _odom_bridge_ok(self):
        """Require a live bridge, a visible ROS publisher, and fresh odometry."""
        receive_time = self.bridge.snapshot().get("odom_receive_time", 0.0)
        return is_odom_bridge_healthy(
            bool(self._odom_bridge_pids()),
            self.bridge.topic_publisher_count(self._odom_topic()),
            receive_time,
            now=time.time(),
        )

    def _ensure_odom_bridge(self):
        """Ensure the sport odometry relay uses this app's ROS/DDS environment."""
        with self._odom_bridge_lock:
            if self._odom_bridge_ok():
                return True

            old_pids = self._odom_bridge_pids()
            if old_pids:
                log.warning("slam: odom bridge is not healthy; restarting pids=%s", old_pids)
                self._stop_odom_bridges()

            self._spawn([sys.executable, os.path.join(BASE_DIR, "sport_odom_bridge.py")],
                        os.path.join(SLAM_DIR, "sport_odom_bridge.log"))
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                if self._odom_bridge_ok():
                    log.info("slam: sport_odom_bridge healthy on %s", self._odom_topic())
                    return True
                time.sleep(0.25)

            log.error("slam: sport_odom_bridge failed health check on %s; "
                      "verify /sportmodestate and CYCLONEDDS_URI", self._odom_topic())
            return False

    def _set_state(self, mode, map_name=""):
        self.mode = mode
        self.map_name = map_name
        self.bridge.slam_mode = mode
        self.bridge.slam_map = map_name

    def _bg(self, fn, *args):
        threading.Thread(target=fn, args=args, daemon=True).start()

    # ---- 对外操作（立即返回，耗时动作走后台线程，避免阻塞事件循环） ----
    def start_mapping(self):
        if self.mode != "off":
            return {"ok": False, "error": f"slam busy: {self.mode}"}
        os.makedirs(SLAM_DIR, exist_ok=True)
        self._set_state("starting")
        self._bg(self._do_start_mapping)
        return {"ok": True, "msg": "建图启动中…"}

    def _do_start_mapping(self):
        try:
            self._pkill("slam_toolbox", "pointcloud_to_laserscan",
                        "slam_mapping.launch.py", "slam_localization.launch.py")
            time.sleep(1.5)
            if not self._ensure_odom_bridge():
                raise RuntimeError("sport_odom_bridge is not healthy")
            self._ensure_odom_tf()
            self._spawn(["ros2", "launch",
                         os.path.join(SLAM_LAUNCH_DIR, "slam_mapping.launch.py")],
                        os.path.join(SLAM_DIR, "runtime.log"))
            # map 系原点≈建图起始时的 odom 位姿，存给以后定位当初始先验
            snap = self.bridge.snapshot()
            self.cfg.setdefault("slam", {})["start_pose"] = [
                round(snap["position"][0], 3),
                round(snap["position"][1], 3),
                round(snap["yaw"], 3),
            ]
            save_config(self.cfg)
            self._set_state("mapping")
            log.info("slam: mapping started")
        except Exception as e:
            self._last_error = f"start_mapping: {e}"
            log.exception("slam: start_mapping failed")
            self._set_state("off")

    def stop_mapping(self, name, save=True):
        if self.mode not in ("mapping", "starting"):
            return {"ok": False, "error": "not mapping"}
        name = self._safe_name(name or "venue")
        self._set_state("saving", name)
        self._bg(self._do_stop_mapping, name, save)
        return {"ok": True, "msg": "保存中…（约需 10-30 秒）"}

    def _do_stop_mapping(self, name, save):
        saved, err = False, ""
        try:
            if save:
                r = subprocess.run(
                    ["bash", os.path.join(SLAM_LAUNCH_DIR, "slam_save_map.sh"), name],
                    capture_output=True, timeout=90)
                out = r.stdout.decode(errors="replace")
                pgm = os.path.join(SLAM_DIR, name + ".pgm")
                saved = os.path.exists(pgm) and "result=0" in out
                if not saved:
                    err = "save failed: " + out[-200:]
            self._pkill("slam_toolbox", "pointcloud_to_laserscan",
                        "slam_mapping.launch.py", "slam_localization.launch.py")
            if saved:
                maps = self.cfg.setdefault("maps", {})
                maps.setdefault(name, {"waypoints": []})
                self.cfg.setdefault("slam", {})["last_map"] = name
                self.cfg.setdefault("slam", {})["auto_localize"] = True
                save_config(self.cfg)
            if err:
                self._last_error = err
            keep = name if saved else self.map_name
            self._set_state("off", keep)
            log.info("slam: mapping stopped, saved=%s (%s)", saved, name)
        except Exception as e:
            self._last_error = f"stop_mapping: {e}"
            log.exception("slam: stop_mapping failed")
            self._set_state("off")

    def start_localization(self, name, initial_pose=None):
        name = self._safe_name(name)
        if not os.path.exists(os.path.join(SLAM_DIR, name + ".posegraph")):
            return {"ok": False, "error": f"map not found: {name}"}
        if initial_pose is None:
            initial_pose = self.cfg.get("maps", {}).get(name, {}).get("initial_pose")
        if self.mode != "off":
            self._pkill("slam_toolbox", "pointcloud_to_laserscan",
                        "slam_mapping.launch.py", "slam_localization.launch.py")
            time.sleep(0.5)
        os.makedirs(SLAM_DIR, exist_ok=True)
        self.cfg.setdefault("slam", {})["auto_localize"] = True
        save_config(self.cfg)
        self._set_state("starting", name)
        self._bg(self._do_start_localization, name, initial_pose)
        return {"ok": True, "map": name, "msg": "定位加载中…"}

    def _do_start_localization(self, name, initial_pose=None):
        try:
            self._pkill("slam_toolbox", "pointcloud_to_laserscan",
                        "slam_mapping.launch.py", "slam_localization.launch.py")
            time.sleep(1.5)
            if not self._ensure_odom_bridge():
                raise RuntimeError("sport_odom_bridge is not healthy")
            self._ensure_odom_tf()
            # slam_toolbox 的 localization 模式不做全局重定位：没有先验时
            # map->odom 恒等，初始偏差 0.5m 以上就再也拉不回来。所以把用户
            # 在地图上点选的"狗当前在 map 的位置"作为 map_start_pose 传入。
            env = {"SLAM_MAP_NAME": name}
            if initial_pose:
                env["SLAM_START_POSE"] = json.dumps([float(v) for v in initial_pose])
            self._spawn(
                ["ros2", "launch",
                 os.path.join(SLAM_LAUNCH_DIR, "slam_localization.launch.py")],
                os.path.join(SLAM_DIR, "runtime.log"),
                extra_env=env)
            self.cfg.setdefault("slam", {})["resume"] = name
            save_config(self.cfg)
            # 等 map->odom 出现（首帧扫描匹配完成），之后进入 localization
            for _ in range(60):
                if self.bridge.slam_tf() is not None:
                    self._set_state("localization", name)
                    log.info("slam: localization active (%s)", name)
                    return
                time.sleep(0.5)
            self._last_error = "定位已启动但 map->odom TF 未出现"
            log.warning("slam: localization started but TF missing (%s)", name)
            self._set_state("localization", name)  # 仍标记，前端显示 TF 未就绪
        except Exception as e:
            self._last_error = f"start_localization: {e}"
            log.exception("slam: start_localization failed")
            self._set_state("off")

    def stop_localization(self):
        if self.mode not in ("localization", "starting"):
            return {"ok": False, "error": "not localizing"}
        self._pkill("slam_toolbox", "pointcloud_to_laserscan",
                    "slam_mapping.launch.py", "slam_localization.launch.py")
        # 保留 map_name：点位库与显示继续用该地图的 map 系坐标，
        # 只是不再有实时修正（用最后已知 TF），避免点位整批跳变。
        keep = self.map_name or self.cfg.get("slam", {}).get("last_map", "")
        if keep:
            self.cfg.setdefault("slam", {})["last_map"] = keep
        self.cfg.setdefault("slam", {})["auto_localize"] = False
        self._set_state("off", keep)
        save_config(self.cfg)
        log.info("slam: localization stopped (map kept: %s)", keep)
        return {"ok": True, "msg": "定位已停止，当前地图保持为 " + (keep or "（无）")}

    def select_map(self, name):
        """切换"当前地图"（底图 + 点位库），不启动/停止定位进程。

        只影响显示与点位库归属；导览/记录仍要求该地图处于定位状态。
        """
        name = self._safe_name(name or "")
        if name and not os.path.exists(os.path.join(SLAM_DIR, name + ".pgm")):
            return {"ok": False, "error": f"map not found: {name}"}
        if name and self.mode == "localization" and name != self.map_name:
            return {"ok": False,
                    "error": f"定位运行中（{self.map_name}），先停止定位再切换"}
        set_current_map(self.cfg, name)
        save_config(self.cfg)
        log.info("slam: selected map -> %s", name or "(none)")
        return {"ok": True, "map": name, "msg": f"当前地图: {name or '（无）'}"}

    def status(self):
        try:
            maps = sorted(f[:-len(".posegraph")] for f in os.listdir(SLAM_DIR)
                          if f.endswith(".posegraph"))
        except OSError:
            maps = []
        # 残差计算有成本，限流到 2 秒一次
        now = time.time()
        if self.mode == "localization" and now - getattr(self, "_q_ts", 0) > 2.0:
            self._q_ts = now
            try:
                q = self.match_quality()
                if q is not None:
                    self._quality = q
            except Exception as e:
                log.warning("slam: match_quality failed: %s", e)
        elif self.mode != "localization":
            self._quality = None

        q = getattr(self, "_quality", None)
        stab = self.bridge.slam_stability()
        return {
            "mode": self.mode,
            "map": self.map_name,
            "tf_ok": self.bridge.slam_tf() is not None,
            "maps": maps,
            "error": self._last_error,
            "quality": q,
            "quality_ok": (q is None) or (q["residual"] < 0.25),
            # 定位稳定性（TF 跳变频率）：前端据此显示绿/黄/红
            "unstable": stab["unstable"],
            "jumps_recent": stab["jumps_recent"],
            "tf_stats": stab["stats"],
        }

    # ---- 定位置信度自检 ----
    def _occupancy(self, name):
        """加载地图占用格坐标（缓存，按 pgm 修改时间失效）。"""
        import numpy as np
        pgm = os.path.join(SLAM_DIR, name + ".pgm")
        try:
            mtime = os.path.getmtime(pgm)
        except OSError:
            return None
        c = getattr(self, "_occ_cache", None)
        if c and c[0] == name and c[1] == mtime:
            return c[2]
        meta = _map_meta(name)
        if not meta:
            return None
        w, h, px, _ = _read_pgm(pgm)
        arr = np.frombuffer(px, dtype=np.uint8).reshape(h, w)
        # 图像行 0 对应地图最上方；转成世界坐标后取占用格
        rows, cols = np.nonzero(arr < 60)
        res = meta["resolution"]; ox, oy = meta["origin"]
        occ = np.stack([cols * res + ox, (h - rows - 1) * res + oy], axis=1)
        occ = occ[::2]  # 降采样，够用且省 CPU
        self._occ_cache = (name, mtime, occ)
        return occ

    def match_quality(self):
        """当前扫描与地图的配准残差（中位数，米）。None=不可用。"""
        import numpy as np
        if self.mode != "localization" or not self.map_name:
            log.warning("slam quality: mode=%s map=%s", self.mode, self.map_name)
            return None
        snap = self.bridge.snapshot()
        scan = snap.get("scan")
        if not scan:
            log.warning("slam quality: no scan yet")
            return None
        ranges, a0, da, rmin, rmax = scan
        pose = self.bridge.odom_to_map(*snap["position"][:2], snap["yaw"])
        if pose is None:
            log.warning("slam quality: no map<->odom tf")
            return None
        occ = self._occupancy(self.map_name)
        if occ is None or len(occ) == 0:
            log.warning("slam quality: no occupancy for %s", self.map_name)
            return None
        mx, my, myaw = pose
        c, s_ = math.cos(myaw), math.sin(myaw)
        pts = []
        for i, r in enumerate(ranges):
            if not (math.isfinite(r) and rmin < r < min(rmax, 12.0)):
                continue
            a = a0 + i * da
            bx, by = r * math.cos(a), r * math.sin(a)
            pts.append((mx + c * bx - s_ * by, my + s_ * bx + c * by))
        if len(pts) < 20:
            log.warning("slam quality: only %d valid scan points", len(pts))
            return None
        p = np.array(pts)
        d = np.sqrt(((p[:, None, :] - occ[None, :, :]) ** 2).sum(axis=2)).min(axis=1)
        med = float(np.median(d))
        share = float((d < 0.2).mean())
        return {"residual": round(med, 3), "inliers": round(share, 3),
                "n": len(pts)}

    @staticmethod
    def _safe_name(name):
        name = str(name).strip()
        return "".join(c for c in name if c.isalnum() or c in "-_") or "venue"

    def auto_resume(self):
        """开机/重启后恢复上次的定位（后台线程调用）。"""
        if not persistent_map_enabled(self.cfg):
            return
        name = self.cfg.get("slam", {}).get("resume", "")
        if not name or not os.path.exists(os.path.join(SLAM_DIR, name + ".posegraph")):
            return
        if self.mode != "off":
            return  # 已接管还活着的实例，不重复拉起
        time.sleep(8)  # 等 DDS/odom 稳定
        try:
            res = self.start_localization(name)
            log.info("slam: auto-resume %s -> %s", name, res)
        except Exception as e:
            log.error("slam: auto-resume failed: %s", e)


# --------------------------------------------------------------------------
# Tour (waypoint-guided autonomous navigation + turn + action + narration)
# --------------------------------------------------------------------------
class Tour:
    """Guides the robot through waypoints. On reaching a waypoint, runs the
    configured exhibit action AND the voice introduction in parallel, and only
    then advances to the next waypoint."""

    def __init__(self, bridge, cfg):
        self.bridge = bridge
        self.cfg = cfg
        self.slam = None        # SlamManager，WebApp 装配；决定点位库与坐标换算
        self.active = False
        self.ids = []            # waypoint ids in order
        self.idx = -1
        self.state = "idle"      # idle | traveling | turning | acting | done
        self.intro_text = ""
        self.intro_wp = None
        self._stage_task = None  # current async stage (turn / act+intro)
        self._turn_task = None
        self._intro_task = None  # current narration playback task
        self._yaw_tolerance = math.radians(4.0)
        self._yaw_k = 1.0        # proportional turn gain
        self._max_vyaw = 0.6     # max turn rate
        self._goal_tf = None     # 下发当前目标时用的 map->odom 快照
        self._tf_lost = False    # 定位是否处于中断状态（用于只告警一次）

    def start(self, ids):
        if not ids:
            return {"ok": False, "error": "empty waypoint list"}
        self.ids = list(ids)
        self.idx = 0
        self.active = True
        self.state = "traveling"
        self._send_current()
        return {"ok": True, "msg": f"tour started, {len(ids)} points"}

    def stop(self):
        self.active = False
        self.state = "idle"
        self.intro_text = ""
        for task in (self._stage_task, self._turn_task, self._intro_task):
            if task:
                task.cancel()
        self._stage_task = None
        self._turn_task = None
        self._intro_task = None
        # Halt the robot so the joystick can take over cleanly, then publish a
        # goal at the current position so SCAN-Planner finishes any running
        # trajectory and returns to WAIT_TARGET (releasing /cmd_vel).
        try:
            self.bridge.send_pause(False)
            self.bridge.send_stopmove()
            self.bridge.send_cmd_vel(0, 0, 0)
            pos = self.bridge.snapshot()["position"]
            self.bridge.send_goal(pos[0], pos[1], 0.0)
        except Exception:
            pass
        return {"ok": True, "msg": "tour stopped"}

    def skip_intro(self):
        """Skip the current action+intro stage and advance to the next waypoint."""
        if self.state in ("acting", "introducing") and self._stage_task:
            self._stage_task.cancel()
            self._stage_task = None
            self._advance()
            return {"ok": True, "msg": "stage skipped"}
        return {"ok": False, "error": "not introducing"}

    def _send_current(self):
        if not self.active or self.idx < 0 or self.idx >= len(self.ids):
            return
        wp = self._find(self.ids[self.idx])
        if not wp:
            return
        goal = self._goal_in_odom(wp)
        if goal is None:
            log.warning("tour: map->odom TF 丢失，点位 %s 暂无法下发", wp.get("name"))
            self._goal_tf = None
            return
        x, y, yaw = goal
        self.bridge.send_goal(x, y, yaw)
        # 记下这次换算用的 TF：之后里程计漂移导致 TF 改写时，要用它判断
        # 是否需要把目标重发到新的 odom 坐标。
        self._goal_tf = self._current_tf()
        log.info("tour -> waypoint %s (%s) odom=(%.2f, %.2f, %.2f) tf=%s",
                 wp.get("name", wp["id"]), self.ids[self.idx], x, y, yaw,
                 None if self._goal_tf is None else tuple(round(v, 3) for v in self._goal_tf))

    def _current_tf(self):
        """当前 map->odom；odom 系点位时没有 TF，返回恒等。"""
        _, frame = active_waypoints(self.cfg, self.bridge, self.slam)
        if frame != "map":
            return (0.0, 0.0, 0.0)
        return self.bridge.slam_tf()

    @staticmethod
    def _tf_changed(old, new, dpos=0.30, dyaw=math.radians(10.0)):
        """map->odom 是否变化到需要重发目标的程度。

        这里的 TF 是已过滤的平滑值（ros_bridge.TfFilter），所以阈值可以
        放宽到 0.30m / 10°：小于这个量级的差异对展品点位没有实际影响，
        频繁重发只会让 SCAN-Planner 反复重新规划。"""
        if old is None or new is None:
            return True
        if math.hypot(old[0] - new[0], old[1] - new[1]) > dpos:
            return True
        dy = abs(math.atan2(math.sin(old[2] - new[2]), math.cos(old[2] - new[2])))
        return dy > dyaw

    def _find(self, wid):
        wps, _frame = active_waypoints(self.cfg, self.bridge, self.slam)
        for wp in wps:
            if wp["id"] == wid:
                return wp
        return None

    def _goal_in_odom(self, wp):
        """把点位换算到 odom 系再下发给 SCAN-Planner。map 系点位用实时
        TF 换算——里程计漂移被持续修正，点位始终指向物理展品位置。"""
        _, frame = active_waypoints(self.cfg, self.bridge, self.slam)
        if frame != "map":
            return wp["x"], wp["y"], wp.get("yaw", 0.0)
        return self.bridge.map_to_odom(wp["x"], wp["y"], wp.get("yaw", 0.0))

    async def tick(self, pos):
        """Advance when the robot reaches the current waypoint, then turn to the
        configured facing angle, then run action + intro in parallel."""
        if not self.active or self.idx < 0 or self.idx >= len(self.ids):
            return
        if self.state == "traveling":
            wp = self._find(self.ids[self.idx])
            if not wp:
                self._advance()
                return
            tf_now = self._current_tf()
            if tf_now is None:
                if not self._tf_lost:
                    log.warning("tour: 定位中断（map->odom 消失），暂停在 %s 等待恢复",
                                wp.get("name", wp["id"]))
                    self._tf_lost = True
                return
            self._tf_lost = False
            # 定位近期频繁跳变 -> 地图坐标不可信，暂停导览而不是追着噪声走。
            stab = self.bridge.slam_stability()
            if stab["unstable"]:
                if not getattr(self, "_unstable_warned", False):
                    log.warning("tour: 定位跳变频繁（%d 次/10s），暂停导览等待稳定",
                                stab["jumps_recent"])
                    self._unstable_warned = True
                self._tour_note = "定位不稳定，已暂停导览"
                return
            self._unstable_warned = False
            self._tour_note = ""
            # 里程计漂移后 slam_toolbox 会改写 map->odom；之前下发的 odom
            # 目标已经不对应当前 map 点位了，必须重发，否则狗会走向错误的
            # 物理位置（到达判定却用新目标，表现为"永远到不了 / 走过头"）。
            if self._goal_tf is not None and self._tf_changed(self._goal_tf, tf_now):
                log.info("tour: map->odom 变化超过阈值，重发目标（%s）",
                         wp.get("name", wp["id"]))
                self._send_current()
            goal = self._goal_in_odom(wp)
            if goal is None:
                return  # TF 丢失时等下一拍，不误判到达
            gx, gy, _ = goal
            dx = pos[0] - gx
            dy = pos[1] - gy
            if math.hypot(dx, dy) < self.cfg.get("reach_threshold", 0.4):
                self.intro_wp = wp
                self.intro_text = wp.get("intro", "").strip() or tts.DEFAULT_INTRO
                log.info("reached exhibit %s", wp.get("name", wp["id"]))
                if wp.get("yaw_at") is not None and float(wp.get("yaw_at", 0.0)) != 0.0:
                    self.state = "turning"
                    self._turn_task = asyncio.create_task(self._stage_turn(wp))
                else:
                    self.state = "acting"
                    self._stage_task = asyncio.create_task(self._stage_act_intro(wp))

    @staticmethod
    def _wrap_pi(a):
        return (a + math.pi) % (2 * math.pi) - math.pi

    async def _turn_to(self, target_yaw, max_vyaw=0.5, timeout=10.0):
        """Open-loop turn in place to target_yaw.

        Uses the configured turn speed and the angle to compute how long to
        command yaw rotation, instead of waiting for the LIO odometry yaw
        feedback (which updates slowly during in-place turns and caused the
        turn loop to time out without actually rotating).
        """
        self.bridge.send_pause(True)  # stop the bridge overriding the turn
        try:
            yaw0 = self.bridge.snapshot()["yaw"]
            err = self._wrap_pi(target_yaw - yaw0)
            if abs(err) < self._yaw_tolerance:
                log.info("already facing target %.1f deg", math.degrees(target_yaw))
                return
            dur = min(abs(err) / max_vyaw, timeout)
            vyaw = math.copysign(max_vyaw, err)
            log.info("turning %.1f deg at %.2f rad/s for %.1fs",
                     math.degrees(err), vyaw, dur)
            t0 = time.time()
            while time.time() - t0 < dur:
                self.bridge.send_move(0.0, 0.0, vyaw)
                await asyncio.sleep(0.05)
        finally:
            self.bridge.send_move(0.0, 0.0, 0.0)
        log.info("turn finished (target %.1f deg)", math.degrees(target_yaw))

    async def _stage_turn(self, wp):
        try:
            target = math.radians(float(wp.get("yaw_at", 0.0)))
            _, frame = active_waypoints(self.cfg, self.bridge, self.slam)
            if frame == "map":
                # yaw_at 存的是 map 系朝向，转到 odom 系再执行原地转向
                m = self.bridge.map_to_odom(0.0, 0.0, target)
                if m is not None:
                    target = m[2]
            await self._turn_to(target)
        except asyncio.CancelledError:
            pass
        except Exception as e:
            log.warning("turn error: %s", e)
        if not self.active:
            return
        self._turn_task = None
        self.state = "acting"
        self._stage_task = asyncio.create_task(self._stage_act_intro(wp))

    async def _stage_act_intro(self, wp):
        """Run the exhibit action and the voice introduction in parallel; advance
        only when both have finished."""
        # Pause the bridge (stop its repeated Move commands) so the action is
        # not overridden, and release SCAN-Planner's control by sending a goal
        # at the current position so its closed-loop controller stops /cmd_vel.
        try:
            self.bridge.send_pause(True)
            self.bridge.send_stopmove()
            self.bridge.send_cmd_vel(0, 0, 0)
            pos = self.bridge.snapshot()["position"]
            self.bridge.send_goal(pos[0], pos[1], 0.0)
            await asyncio.sleep(1.0)
        except Exception:
            pass
        # then action + intro in parallel
        try:
            action_name = wp.get("action", "") or ""
            _, action_dur = self.bridge.send_action(action_name)
            # start intro playback concurrently
            intro_task = asyncio.create_task(self._play_intro(wp))
            self._intro_task = intro_task
            tasks = [intro_task]
            if action_dur > 0:
                # wait for both: action duration and intro playback
                sleep_task = asyncio.ensure_future(asyncio.sleep(action_dur))
                tasks.append(sleep_task)
                done, pending = await asyncio.wait(tasks, return_when=asyncio.ALL_COMPLETED)
                # keep intro running if it outlasts the action
                for t in pending:
                    await t
            else:
                await intro_task
        except asyncio.CancelledError:
            pass
        except Exception as e:
            log.warning("act+intro error: %s", e)
        finally:
            # stop narration immediately if it is still playing (tour stopped /
            # stage cancelled); tts kills ffplay on cancellation
            if self._intro_task and not self._intro_task.done():
                self._intro_task.cancel()
            self._intro_task = None
            # resume the bridge so the next leg of the tour can be controlled
            try:
                self.bridge.send_pause(False)
            except Exception:
                pass
            if self.active:
                self._advance()

    async def _play_intro(self, wp):
        try:
            # Streaming TTS: speak() pipes edge-tts chunks straight into ffplay,
            # so narration starts as soon as the first chunk arrives.
            text = wp.get("intro", "").strip() or tts.DEFAULT_INTRO
            await tts.speak(text, tts.target_audio(wp))
        except asyncio.CancelledError:
            pass
        except Exception as e:
            log.warning("intro playback error: %s", e)

    def _advance(self):
        self._stage_task = None
        if not self.active:
            return
        self.idx += 1
        if self.idx >= len(self.ids):
            self.state = "done"
            self.active = False
            self.bridge.send_cmd_vel(0, 0, 0)
            log.info("tour finished")
        else:
            self.state = "traveling"
            self.intro_text = ""
            self._send_current()

    def status(self):
        if not self.active:
            return {"active": False, "idx": -1, "total": len(self.ids),
                    "state": self.state, "intro": ""}
        wp = self._find(self.ids[self.idx]) if self.idx < len(self.ids) else None
        return {
            "active": True,
            "idx": self.idx,
            "total": len(self.ids),
            "state": self.state,
            "current": wp,
            "intro": self.intro_text if self.state in ("acting", "introducing") else "",
        }


# --------------------------------------------------------------------------
# WebSocket handler
# --------------------------------------------------------------------------
# SCAN-Planner launch command (real-robot, 2D Nav Goal mode, bridge on)
SCAN_LAUNCH_CMD = (
    "source /opt/ros/humble/setup.bash && "
    "source /home/unitree/hu/go2/install/setup.bash && "
    "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp "
    "CYCLONEDDS_URI=file:///home/unitree/hu/go2/cyclonedds_eth0.xml && "
    # body_pose 用 sportmodestate 转发后的 Odometry（足端里程计，长距离漂移小于 LIO）
    "ros2 launch scan_planner real_robot.launch.py navi_mode:=1 bridge_enable:=true rviz:=false "
    "body_pose_topic:=/sportmodestate_odom"
)
SCAN_LOG_PATH = os.path.join(BASE_DIR, "log", "scan_planner.log")


class WebApp:
    def __init__(self, cfg, bridge):
        self.cfg = cfg
        self.bridge = bridge
        self.tour = Tour(bridge, cfg)
        self.slam = SlamManager(bridge, cfg)
        self.tour.slam = self.slam
        ensure_current_map(cfg)
        self.slam._adopt_running()
        self.slam._restore_map()
        self.bridge.load_last_tf()
        self.clients = set()
        self.control_mode = "joystick"  # "joystick" (sport Move) or "tour" (SCAN-Planner)
        self._active_modes = set()      # 运动模式页当前开启的 bool 型模式
        self._freewalk_on = False        # 灵动模式是否处于开启状态
        self.scan_proc = None           # SCAN-Planner launch subprocess
        self.ai_history = []            # AI chat history [{role, content}]
        self.ai_client = None           # websocket that owns the AI session
        self.ai_started = False
        self.ai_busy = False
        self.ai_model = self.AI_MODEL   # 当前会话模型，可由前端指定

    def waypoints_view(self):
        """给前端显示的点位。

        定位激活 -> 返回 map 系点位原值（画布也切到 map 系，点位因此固定不动）；
        否则返回内置 odom 点位。
        """
        wps, frame = active_waypoints(self.cfg, self.bridge, self.slam)
        return wps, frame, self.slam.map_name

    def _points_to_map(self, points):
        """odom 系点云 -> map 系，numpy 向量化。"""
        tf = self.bridge.slam_tf() or self.bridge.slam_tf_last()
        if tf is None or not points:
            return points
        try:
            import numpy as np
            a = np.asarray(points, dtype=float)
            c, s_ = math.cos(tf[2]), math.sin(tf[2])
            out = np.empty_like(a)
            out[:, 0] = tf[0] + c * a[:, 0] - s_ * a[:, 1]
            out[:, 1] = tf[1] + s_ * a[:, 0] + c * a[:, 1]
            if a.shape[1] > 2:
                out[:, 2] = a[:, 2]
            return out.round(3).tolist()
        except Exception:
            return points

    def _persistent_map_payload(self, name):
        """从 slam_maps/<name>.pgm 渲染持久地图底图（红墙+轮廓），缓存。

        与定位进程无关：定位没开也能显示底图。缓存键 = (name, pgm mtime)。
        """
        import base64
        pgm = os.path.join(SLAM_DIR, name + ".pgm")
        try:
            mtime = os.path.getmtime(pgm)
        except OSError:
            return None
        cache = getattr(self, "_persist_cache", None)
        if cache and cache[0] == name and cache[1] == mtime:
            return cache[2]
        rendered = maprender.render_map_file(SLAM_DIR, name)
        if rendered is None:
            return None
        png, w, h = rendered
        meta = _map_meta(name)
        if not meta:
            return None
        payload = {"name": name, "width": w, "height": h,
                   "resolution": meta["resolution"], "origin": meta["origin"],
                   "png_b64": base64.b64encode(png).decode()}
        self._persist_cache = (name, mtime, payload)
        return payload

    def _live_map_payload(self):
        """建图中（还没保存 pgm）用 slam_toolbox 的 /map 当底图。"""
        import base64
        import struct
        import zlib
        import numpy as np
        raw = self.bridge.static_map()
        if raw is None:
            return None
        w, h, res, ox, oy, data, stamp = raw
        cache = getattr(self, "_live_cache", None)
        if cache and cache[0] == stamp:
            return cache[1]
        try:
            arr = np.frombuffer(data, dtype=np.int8).reshape(h, w)
        except ValueError:
            return None
        occ = arr >= 65
        free = arr == 0

        img = np.zeros((h, w, 4), dtype=np.uint8)
        img[free] = (34, 64, 94, 200)
        img[occ] = (228, 74, 74, 235)

        rawb = bytearray()
        for y in range(h):
            rawb.append(0)
            rawb += img[y].tobytes()

        def chunk(tag, payload):
            c = struct.pack(">I", len(payload)) + tag + payload
            return c + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF)

        png = b"\x89PNG\r\n\x1a\n"
        png += chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 6, 0, 0, 0))
        png += chunk(b"IDAT", zlib.compress(bytes(rawb), 6))
        png += chunk(b"IEND", b"")

        payload = {"name": self.slam.map_name, "width": w, "height": h,
                   "resolution": res, "origin": [ox, oy],
                   "png_b64": base64.b64encode(png).decode()}
        self._live_cache = (stamp, payload)
        return payload

    def _static_map_for(self):
        """当前应当下发给前端的底图。选中地图有 pgm -> 持久底图；
        建图中（pgm 还没存）-> slam_toolbox 实时 /map；否则 None。"""
        name = current_map_name(self.cfg, self.slam)
        if name:
            payload = self._persistent_map_payload(name)
            if payload is not None:
                return payload
        if self.slam.mode == "mapping":
            return self._live_map_payload()
        return None

    def _static_map_sig(self):
        """底图签名：地图名 + pgm mtime（或建图时 /map 的刷新时间）。"""
        name = current_map_name(self.cfg, self.slam)
        if name:
            try:
                return f"{name}:{int(os.path.getmtime(os.path.join(SLAM_DIR, name + '.pgm')))}"
            except OSError:
                pass
        if self.slam.mode == "mapping":
            raw = self.bridge.static_map()
            if raw:
                return f"live:{raw[-1]:.0f}"
        return ""

    async def broadcast_state(self):
        """Push the current map + robot state to all clients."""
        snap = self.bridge.snapshot()
        wps, wp_frame, wp_map = self.waypoints_view()
        map_pts = snap["map_points"]
        pos, yaw = snap["position"], snap["yaw"]
        map_name = current_map_name(self.cfg, self.slam)
        pose_ok = False
        if map_name:
            # 选中了持久地图：画布固定 map 系（底图+点位都钉死在地图上）。
            # 机器人/实时点云只有拿到新鲜 TF 才可信；定位没开就不画它们，
            # 绝不退回 odom 系——否则整批点位会瞬间跳到另一套坐标。
            wp_frame, wp_map = "map", map_name
            tf = self.bridge.slam_tf()
            if tf is not None:
                c, s_ = math.cos(tf[2]), math.sin(tf[2])
                pose = (tf[0] + c * pos[0] - s_ * pos[1],
                        tf[1] + s_ * pos[0] + c * pos[1],
                        math.atan2(math.sin(tf[2] + yaw), math.cos(tf[2] + yaw)))
                map_pts = self._points_to_map(map_pts)
                pos = [pose[0], pose[1], pos[2]]
                yaw = pose[2]
                pose_ok = True
        # 底图只在签名（地图名+mtime）变化时随 state 发一次，避免 base64
        # 以 push_hz 的频率重复广播；后连上的客户端从 init 消息里拿。
        include_static = False
        static_map = None
        try:
            sig = self._static_map_sig() or None
            if sig != getattr(self, "_static_sig_sent", None):
                static_map = self._static_map_for() if sig else None
                self._static_sig_sent = sig
                include_static = True
        except Exception as e:
            log.warning("static map build failed: %s", e)
        msg = {
            "type": "state",
            "frame": wp_frame,
            "map_name": map_name,
            "pose_ok": pose_ok,
            "tour_note": getattr(self.tour, "_tour_note", ""),
            "map": map_pts,
            "map_stamp": snap["map_stamp"],
            "pos": pos,
            "yaw": yaw,
            "odom_stamp": snap["odom_stamp"],
            "persistent_map": persistent_map_enabled(self.cfg),
            "tour": self.tour.status(),
            "control_mode": self.control_mode,
            "waypoints": wps,
            "slam": self.slam.status(),
            "wp_frame": wp_frame,
            "wp_map": wp_map,
            "motion": self._motion_state(),
        }
        if include_static:
            msg["static_map"] = static_map  # None = 让前端清掉底图
        # 定位时附上 map 系位姿，供地图查看器绘制机器人实际位置
        if wp_frame == "map" and pose_ok:
            m = self.bridge.odom_to_map(*snap["position"][:2], snap["yaw"])
            if m is not None:
                msg["robot_map"] = [round(m[0], 3), round(m[1], 3), round(m[2], 4)]
        if self.clients:
            data = json.dumps(msg)
            await asyncio.gather(
                *(c.send(data) for c in list(self.clients)), return_exceptions=True
            )

    async def loop(self):
        hz = self.cfg.get("push_hz", 10)
        while True:
            try:
                await self.tour.tick(self.bridge.snapshot()["position"])
            except Exception:
                pass
            await self.broadcast_state()
            await asyncio.sleep(1.0 / hz)

    # ---- command handling ----
    def handle_message(self, data, websocket=None):
        try:
            msg = json.loads(data)
        except Exception:
            return {"ok": False, "error": "bad json"}
        mtype = msg.get("type")

        if mtype == "cmd_vel":
            # Joystick control -> sport Move channel (like the official remote
            # controller). Independent of /cmd_vel used by SCAN-Planner tours.
            # Taking the joystick always stops an active tour first.
            if self.tour.active:
                self.tour.stop()
            self.control_mode = "joystick"
            vx = float(msg.get("vx", 0.0))
            vy = float(msg.get("vy", 0.0))
            vyaw = float(msg.get("vyaw", 0.0))
            # clamp to max_speed
            m = self.cfg.get("max_speed", 0.5)
            vx = max(-m, min(m, vx))
            vy = max(-m, min(m, vy))
            vyaw = max(-1.2, min(1.2, vyaw))
            self.bridge.send_move(vx, vy, vyaw)
            return {"ok": True}

        if mtype == "goal":
            self.tour.stop()
            x, y, yw = float(msg["x"]), float(msg["y"]), float(msg.get("yaw", 0.0))
            if msg.get("frame") == "map":
                m = self.bridge.map_to_odom(x, y, yw)
                if m is None:
                    return {"ok": False, "error": "定位 TF 未就绪"}
                x, y, yw = m
            self.bridge.send_goal(x, y, yw)
            return {"ok": True}

        if mtype == "sport":
            action = msg.get("action")
            api = {
                "standup": rb.SPORT_STANDUP,
                "standdown": rb.SPORT_STANDDOWN,
                "damp": rb.SPORT_DAMP,
                "sit": rb.SPORT_SIT,
                "risesit": rb.SPORT_RISESIT,
                "balance": rb.SPORT_BALANCESTAND,
                "stop": rb.SPORT_STOPMOVE,
                "stopmove": rb.SPORT_STOPMOVE,
                "recovery": rb.SPORT_RECOVERYSTAND,
                "heart": rb.SPORT_HEART,
                "freewalk_on": (rb.SPORT_FREEWALK, "true"),
                "freewalk_off": (rb.SPORT_FREEWALK, "false"),
            }.get(action)
            if api is not None:
                if isinstance(api, tuple):
                    self.bridge.send_sport(api[0], param=api[1])
                else:
                    self.bridge.send_sport(api)
                return {"ok": True}
            # 运动模式页的动作/模式名（hello/dance1/walkupright/...）
            if action in rb.MOTION_ACTIONS:
                return self._handle_motion_action(action, msg.get("enable"))
            return {"ok": False, "error": f"unknown action {action}"}

        if mtype == "motion":
            return self._handle_motion(msg)

        if mtype == "waypoint":
            return self._handle_waypoint(msg)

        if mtype == "tour":
            return self._handle_tour(msg)

        if mtype == "slam":
            return self._handle_slam(msg)

        if mtype == "tts":
            return self._handle_tts(msg)

        if mtype == "scanner":
            return self._handle_scanner(msg)

        if mtype == "emergency":
            return self._handle_emergency()

        if mtype == "ai":
            return self._handle_ai(msg, websocket)

        if mtype == "config":
            return self._handle_config(msg)

        if mtype == "restart":
            return self._handle_restart()

        return {"ok": False, "error": f"unknown type {mtype}"}

    # ---- 运动模式页 ----
    # 独立页面 /motion 用的指令通道。与导览控制台共用同一套 sport 发送路径，
    # 但额外做三件事：模式互斥、回执校验、爬楼能力探测。

    def _motion_state(self):
        st = dict(getattr(self.bridge, "sport_state", {}) or {})
        st["active_modes"] = sorted(self._active_modes)
        st["freewalk"] = self._freewalk_on
        return st

    def _await_sport_result(self, api_id, since, timeout=1.2):
        """等 since 之后、api_id 对应的回执，返回 (code, data)；超时返回 None。

        since 必须在「发送之前」取：固件回执常常在 send_sport 返回前就到了，
        以发送后的时刻为基准会把这条回执当成旧的丢掉，界面就会误报「无响应」。

        固件对未知 api_id 回 3203、参数不对回 3202，成功通常是 0 或 -1。
        界面依赖这个结果如实报告，不能把 publish 成功当成执行成功。
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            resp = getattr(self.bridge, "last_sport_resp", None)
            if resp and resp.get("api_id") == api_id \
                    and resp.get("ts", 0) > since:
                return resp["code"], resp.get("data", "")
            time.sleep(0.01)
        return None

    def _send_and_await(self, api_id, flag=None):
        """发一条 sport 指令并等它的回执。flag=None 表示不带参数。"""
        since = time.time()
        if flag is None:
            self.bridge.send_sport(api_id)
        else:
            self.bridge.send_sport_bool(api_id, flag)
        return self._await_sport_result(api_id, since)

    def _send_mode(self, name, enable):
        """进入/退出一个 bool 型模式，返回 (ok, msg)。"""
        api_id = rb.BOOL_MODES[name][0] if name in rb.BOOL_MODES else None
        if api_id is None:
            return False, f"未知模式 {name}"
        res = self._send_and_await(api_id, enable)
        if res is None:
            return False, "机器人无响应（指令已发出但没有回执）"
        code, data = res
        if code == 3203:
            return False, f"该固件不支持此模式（api {api_id} 未实现）"
        if code == 3202:
            return False, "参数格式被固件拒绝"
        if code not in (0, -1):
            return False, f"固件返回错误码 {code}"
        if enable:
            self._active_modes.add(name)
        else:
            self._active_modes.discard(name)
        return True, ""

    def _handle_motion_action(self, name, enable=None):
        """运动模式页的动作按钮：一次性动作或 bool 型模式开关。"""
        spec = rb.MOTION_ACTIONS.get(name)
        if spec is None:
            return {"ok": False, "error": f"未知动作 {name}"}
        api_id, is_bool, _on, label = spec
        if is_bool:
            if enable is None:
                enable = name not in self._active_modes
            enable = bool(enable)
            # 灵动子模式互斥：开启新的之前先关掉上一个
            if enable and name in rb.EXCLUSIVE_MODES:
                for other in list(self._active_modes):
                    if other != name and other in rb.EXCLUSIVE_MODES:
                        self._send_mode(other, False)
            ok, err = self._send_mode(name, enable)
            return {"ok": ok, "action": name, "enabled": enable,
                    "error": err or None,
                    "msg": f"{label}{'已开启' if enable else '已关闭'}" if ok else err}
        # 一次性动作：动画期间自身会报进度，重复点击由前端节流
        res = self._send_and_await(api_id)
        if res is not None and res[0] == 3203:
            return {"ok": False, "action": name,
                    "error": f"该固件不支持此动作（api {api_id} 未实现）"}
        if res is not None and res[0] not in (0, -1):
            return {"ok": False, "action": name,
                    "error": f"固件返回错误码 {res[0]}"}
        return {"ok": True, "action": name, "msg": f"{label} 已下发"}

    def _handle_motion(self, msg):
        op = msg.get("op")
        if op == "state":
            return {"ok": True, "motion": self._motion_state()}
        if op == "action":
            return self._handle_motion_action(msg.get("name"),
                                             msg.get("enable"))
        if op == "freewalk":
            return self._handle_freewalk(msg.get("enable", True))
        if op == "exit_all":
            return self._handle_exit_all_modes()
        return {"ok": False, "error": f"unknown motion op {op}"}

    def _handle_exit_all_modes(self):
        """关闭所有已打开的模式，回到普通站立状态。"""
        done = []
        for name in list(self._active_modes):
            ok, err = self._send_mode(name, False)
            if ok:
                done.append(name)
        self._active_modes.clear()
        # 灵动模式不是 bool 开关，退出方式是 StandUp
        if self._freewalk_on:
            self.bridge.send_sport(rb.SPORT_STANDUP)
            self._freewalk_on = False
            done.append("freewalk")
        self.bridge.send_sport(rb.SPORT_STOPMOVE)
        return {"ok": True, "closed": done,
                "msg": f"已关闭 {len(done)} 个模式" if done else "当前没有开启的模式"}

    def _handle_freewalk(self, enable):
        """灵动模式开关。

        FreeWalk(2045) 在 Go2 上是一次性「进入自由行走」命令，不带参数、也不
        存在对应的退出命令；退出用 StandUp 回到普通站立。所以这里把「开关」
        拆成两条不同的指令，而不是给 2045 传 bool。
        """
        if enable:
            res = self._send_and_await(rb.SPORT_FREEWALK)
            if res is None:
                return {"ok": False, "error": "机器人无响应"}
            code = res[0]
            if code == 3203:
                return {"ok": False, "error": "该固件不支持灵动模式（api 2045 未实现）"}
            if code == 3202:
                return {"ok": False, "error": "参数格式被固件拒绝"}
            if code not in (0, -1):
                return {"ok": False, "error": f"固件返回错误码 {code}"}
            self._freewalk_on = True
            return {"ok": True, "enabled": True, "api_id": rb.SPORT_FREEWALK,
                    "msg": "灵动模式已开启，可直接走楼梯"}

        if not self._freewalk_on:
            return {"ok": True, "enabled": False, "msg": "灵动模式本来就没有开启"}
        res = self._send_and_await(rb.SPORT_STANDUP)
        if res is not None and res[0] not in (0, -1):
            return {"ok": False, "error": f"退出灵动失败，固件返回 {res[0]}"}
        self._freewalk_on = False
        return {"ok": True, "enabled": False, "api_id": rb.SPORT_STANDUP,
                "msg": "已退出灵动模式，回到普通站立"}

    # ---- emergency stop: stop everything and damp ----
    def _handle_emergency(self):
        # 1) stop the tour / any running action
        self.tour.stop()
        # 2) stop the joystick
        self.bridge.send_cmd_vel(0, 0, 0)
        self.bridge.send_move(0.0, 0.0, 0.0)
        # 3) shut down SCAN-Planner (kill all planner nodes)
        self._handle_scanner({"op": "stop"})
        # 4) drop into damping mode immediately
        self.bridge.send_damp()
        self.control_mode = "joystick"
        log.warning("EMERGENCY STOP: all stopped, robot in damping mode")
        return {"ok": True, "msg": "急停：已停止所有程序，机器人进入阻尼状态"}

    # ---- AI chat session (Ollama REST API on host: 11434) ----
    OLLAMA_API = "http://127.0.0.1:11434"
    AI_MODEL = "gemma4:e2b-it-qat"

    def _ollama_post(self, path, payload, timeout=120):
        req = urllib.request.Request(
            self.OLLAMA_API + path,
            data=json.dumps(payload).encode(),
            headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read())

    def _ollama_get(self, path, timeout=10):
        req = urllib.request.Request(self.OLLAMA_API + path)
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read())

    async def _ai_push(self, payload, client=None):
        target = client or self.ai_client
        if target:
            try:
                await target.send(json.dumps({"type": "ai", **payload}))
            except Exception:
                pass

    async def _ai_check_and_start(self):
        try:
            timeout = aiohttp.ClientTimeout(total=15)
            async with aiohttp.ClientSession(timeout=timeout) as session:
                async with session.get(self.OLLAMA_API + "/api/tags") as r:
                    tags = await r.json()
            models = [m.get("name", "") for m in tags.get("models", [])]
            if not any(m.split(":")[0] == self.ai_model.split(":")[0] for m in models):
                await self._ai_push({
                    "op": "error",
                    "error": f"模型 {self.ai_model} 未下载。请在 Jetson 上运行: ollama pull {self.ai_model}"})
                return
            self.ai_started = True
            await self._ai_push({"op": "started", "model": self.ai_model})
        except Exception as e:
            await self._ai_push({"op": "error", "error": f"ollama 服务不可达: {e}"})

    async def _ai_list_models(self, client=None):
        """把 ollama 已下载的模型列表推给前端，供模型下拉框使用。"""
        try:
            timeout = aiohttp.ClientTimeout(total=10)
            async with aiohttp.ClientSession(timeout=timeout) as session:
                async with session.get(self.OLLAMA_API + "/api/tags") as r:
                    tags = await r.json()
            models = [m.get("name", "") for m in tags.get("models", []) if m.get("name")]
            await self._ai_push(
                {"op": "models", "models": models, "default": self.ai_model}, client=client)
        except Exception as e:
            await self._ai_push({"op": "models", "models": [], "error": str(e)}, client=client)

    def _handle_ai(self, msg, websocket):
        op = msg.get("op")
        if op == "start":
            self.ai_client = websocket
            self.ai_history = []
            self.ai_model = msg.get("model") or self.AI_MODEL
            asyncio.ensure_future(self._ai_check_and_start())
            return {"ok": True, "model": self.ai_model}
        if op == "models":
            asyncio.ensure_future(self._ai_list_models(client=websocket))
            return {"ok": True}
        if op == "end":
            # 先捕获客户端再清空，否则 ended 消息推不出去
            client = self.ai_client
            self.ai_client = None
            self.ai_started = False
            self.ai_history = []
            asyncio.ensure_future(self._ai_push({"op": "ended"}, client=client))
            return {"ok": True}
        if not self.ai_started:
            return {"ok": False, "error": "AI 会话未启动，请先启动对话"}
        if op == "send":
            asyncio.ensure_future(self._ai_chat(msg.get("text", "")))
            return {"ok": True}
        if op == "image":
            asyncio.ensure_future(
                self._ai_image(msg.get("data", ""), msg.get("name", "camera.jpg")))
            return {"ok": True}
        if op == "video":
            asyncio.ensure_future(
                self._ai_video(msg.get("data", ""), msg.get("name", "camera.mp4")))
            return {"ok": True}
        return {"ok": False, "error": f"unknown ai op {op}"}

    async def _ai_stream(self):
        """向 ollama 发起流式 chat（messages 取 self.ai_history），逐段推给前端。
        返回完整回复文本。"""
        timeout = aiohttp.ClientTimeout(total=300)
        full = ""
        async with aiohttp.ClientSession(timeout=timeout) as session:
            async with session.post(
                    self.OLLAMA_API + "/api/chat",
                    json={"model": self.ai_model,
                          "messages": self.ai_history,
                          "stream": True,
                          "think": False}) as resp:
                resp.raise_for_status()
                async for raw in resp.content:
                    line = raw.decode(errors="replace").strip()
                    if not line:
                        continue
                    chunk = json.loads(line)
                    piece = chunk.get("message", {}).get("content", "")
                    if piece:
                        full += piece
                        await self._ai_push(
                            {"op": "output", "text": piece, "final": False})
                    if chunk.get("done", False):
                        break
        return full

    async def _ai_chat(self, text):
        if self.ai_busy:
            await self._ai_push({"op": "error", "error": "上一条还在回复中"})
            return
        self.ai_busy = True
        try:
            self.ai_history.append({"role": "user", "content": text})
            full = await self._ai_stream()
            if full.strip():
                self.ai_history.append({"role": "assistant", "content": full.strip()})
            await self._ai_push({"op": "output", "text": "", "final": True})
            log.info("ai chat done (%d chars)", len(full))
        except Exception as e:
            if self.ai_history and self.ai_history[-1].get("role") == "user":
                self.ai_history.pop()
            await self._ai_push({"op": "output", "text": "", "final": True})
            await self._ai_push({"op": "error", "error": str(e)})
            log.error("ai chat failed: %s", e)
        finally:
            self.ai_busy = False

    async def _ai_chat_images(self, prompt, image_paths):
        """图片/视频帧分析：本地图片转 base64，流式交给 ollama 逐段返回。"""
        if self.ai_busy:
            await self._ai_push({"op": "error", "error": "上一条还在回复中"})
            return
        self.ai_busy = True
        try:
            images = []
            for p in image_paths:
                with open(p, "rb") as f:
                    images.append(base64.b64encode(f.read()).decode())
            self.ai_history.append({"role": "user", "content": prompt, "images": images})
            full = await self._ai_stream()
            if full.strip():
                self.ai_history.append({"role": "assistant", "content": full.strip()})
            await self._ai_push({"op": "output", "text": "", "final": True})
            log.info("ai image analysis done (%d chars)", len(full))
        except Exception as e:
            if self.ai_history and self.ai_history[-1].get("images"):
                self.ai_history.pop()
            await self._ai_push({"op": "output", "text": "", "final": True})
            await self._ai_push({"op": "error", "error": str(e)})
            log.error("ai image analysis failed: %s", e)
        finally:
            self.ai_busy = False

    async def _ai_image(self, b64data, name):
        try:
            os.makedirs(UPLOAD_DIR, exist_ok=True)
            ts = int(time.time() * 1000)
            path = os.path.join(UPLOAD_DIR, f"img_{ts}.jpg")
            with open(path, "wb") as f:
                f.write(base64.b64decode(b64data))
            await self._ai_push({"op": "saved"})
            log.info("image saved: %s", path)
            await self._ai_chat_images("请用中文描述这张图片的内容。", [path])
        except Exception as e:
            await self._ai_push({"op": "error", "error": f"图片处理失败: {e}"})

    async def _ai_video(self, b64data, name):
        """Save video, extract 3 key frames with ffmpeg, analyze them together."""
        try:
            os.makedirs(UPLOAD_DIR, exist_ok=True)
            ts = int(time.time() * 1000)
            ext = os.path.splitext(name)[1] or ".mp4"
            vpath = os.path.join(UPLOAD_DIR, f"vid_{ts}{ext}")
            with open(vpath, "wb") as f:
                f.write(base64.b64decode(b64data))
            await self._ai_push({"op": "saved"})
            # duration via ffprobe
            try:
                out = subprocess.run(
                    ["ffprobe", "-v", "quiet", "-show_entries", "format=duration",
                     "-of", "csv=p=0", vpath],
                    capture_output=True, text=True, timeout=15).stdout.strip()
                dur = max(float(out), 1.0)
            except Exception:
                dur = 10.0
            frames = []
            for i, frac in enumerate((0.25, 0.5, 0.75)):
                fp = os.path.join(UPLOAD_DIR, f"frame_{ts}_{i}.jpg")
                t = min(dur * frac, max(dur - 0.1, 0.0))
                subprocess.run(["ffmpeg", "-y", "-loglevel", "quiet",
                                "-ss", str(t), "-i", vpath, "-frames:v", "1", fp],
                               capture_output=True, timeout=30)
                if os.path.isfile(fp):
                    frames.append(fp)
            if not frames:
                await self._ai_push({"op": "error", "error": "视频抽帧失败"})
                return
            log.info("video frames extracted: %s", frames)
            await self._ai_chat_images(
                "这是同一段视频在不同时间点的三帧画面，请用中文描述这段视频的内容。", frames)
        except Exception as e:
            await self._ai_push({"op": "error", "error": f"视频处理失败: {e}"})

    # ---- SCAN-Planner process control ----
    def scan_running(self):
        return self.scan_proc is not None and self.scan_proc.poll() is None

    def _handle_scanner(self, msg):
        op = msg.get("op")
        if op == "start":
            if self.scan_running():
                return {"ok": True, "msg": "SCAN-Planner already running", "running": True}
            try:
                os.makedirs(os.path.dirname(SCAN_LOG_PATH), exist_ok=True)
                logf = open(SCAN_LOG_PATH, "ab")
            except Exception:
                logf = subprocess.DEVNULL
            # start in its own session so the whole launch tree can be killed
            self.scan_proc = subprocess.Popen(
                ["bash", "-c", SCAN_LAUNCH_CMD],
                stdout=logf, stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            log.info("SCAN-Planner launch started (pid %s)", self.scan_proc.pid)
            return {"ok": True, "msg": "SCAN-Planner 启动中", "running": True}

        if op == "stop":
            stopped = False
            if self.scan_proc:
                try:
                    os.killpg(os.getpgid(self.scan_proc.pid), signal.SIGTERM)
                    stopped = True
                except Exception:
                    pass
                self.scan_proc = None
            # fallback: kill any leftover planner nodes
            for pat in ("scan_planner_node", "closed_loop_controller",
                        "odom_tf_broadcaster", "go2_cmd_vel_bridge_node",
                        "real_robot.launch", "scan_planner.*launch"):
                try:
                    subprocess.run(["pkill", "-f", pat],
                                   capture_output=True, timeout=5)
                except Exception:
                    pass
            log.info("SCAN-Planner launch stopped")
            return {"ok": True, "msg": "SCAN-Planner 已停止" if stopped else "已停止(无运行进程)",
                    "running": False}

        if op == "status":
            return {"ok": True, "running": self.scan_running()}

        return {"ok": False, "error": f"unknown scanner op {op}"}

    def _handle_tts(self, msg):
        """Voice actions: preview an exhibit intro, or set a waypoint's intro text."""
        op = msg.get("op")
        if op == "preview":
            wid = int(msg.get("id", 0))
            wps, _frame = active_waypoints(self.cfg, self.bridge, self.slam)
            wp = next((w for w in wps if w["id"] == wid), None)
            if not wp:
                return {"ok": False, "error": f"no waypoint {wid}"}
            text = (msg.get("text") or wp.get("intro", "")).strip() or tts.DEFAULT_INTRO

            async def _preview():
                try:
                    # Prefer the waypoint's own cache (the same file the tour
                    # plays) so preview works offline; only synthesize when the
                    # text has no cached audio yet.
                    path = tts.audio_path(wp.get("id", 0), tts._text_hash(text))
                    if os.path.isfile(path):
                        await tts.play(path)
                        return
                    if await tts.speak(text, path):
                        return
                    # online TTS unavailable -> fall back to the waypoint's own
                    # cached narration (its saved intro) if there is one
                    fallback = tts.target_audio(wp)
                    if fallback != path and os.path.isfile(fallback):
                        log.warning("preview: TTS unavailable, playing cached %s", fallback)
                        await tts.play(fallback)
                    else:
                        log.warning("preview: TTS unavailable and no cache for waypoint %s", wid)
                except Exception as e:
                    log.warning("preview error: %s", e)
            asyncio.create_task(_preview())
            return {"ok": True, "msg": "preview playing"}

        if op == "set_intro":
            wid = int(msg.get("id", 0))
            wps, _frame = active_waypoints(self.cfg, self.bridge, self.slam)
            for w in wps:
                if w["id"] == wid:
                    w["intro"] = msg.get("intro", "")
                    save_config(self.cfg)
                    view, _, _ = self.waypoints_view()
                    return {"ok": True, "waypoints": view}
            return {"ok": False, "error": f"no waypoint {wid}"}

        return {"ok": False, "error": f"unknown tts op {op}"}

    def _handle_waypoint(self, msg):
        op = msg.get("op")
        wps, frame = active_waypoints(self.cfg, self.bridge, self.slam)

        if frame == "map" and op in ("add", "record", "reposition"):
            # 记录点必须用新鲜 TF：里程计会跳变，用过期 TF 换算出来的坐标
            # 会静默错位，之后导览就往错的地方跑。
            if self.bridge.slam_tf() is None:
                return {"ok": False,
                        "error": "定位未就绪（地图已选但没在定位）。请先「加载地图定位」再记录点位"}

        def _store_xy(x, y, yaw):
            """把前端画布坐标换算到点位库坐标系。

            画布与点位库同系时原样入库（最常见）；只有"画布还画在 odom、
            点位库已是 map"这种过渡瞬间才需要换算。换算失败返回 None，
            由调用方拒绝入库——宁可报错也不能把 odom 坐标写进 map 库。
            """
            if frame != "map" or msg.get("frame") == "map":
                return x, y, yaw
            m = self.bridge.odom_to_map(x, y, yaw)
            return (m[0], m[1], m[2]) if m else None

        def _resp(extra=None, wp=None):
            view, _, _ = self.waypoints_view()
            r = {"ok": True, "waypoints": view, "wp_frame": frame}
            if extra:
                r.update(extra)
            return r

        if op == "add":  # add by map click
            conv = _store_xy(float(msg["x"]), float(msg["y"]),
                             float(msg.get("yaw", 0.0)))
            if conv is None:
                return {"ok": False, "error": "定位 TF 未就绪，无法换算到地图坐标"}
            x, y, yaw = conv
            new_id = max([w["id"] for w in wps], default=0) + 1
            wps.append({
                "id": new_id,
                "name": msg.get("name", f"点{new_id}"),
                "x": round(x, 3),
                "y": round(y, 3),
                "z": float(msg.get("z", 0.3)),
                "yaw": round(yaw, 3),
                "yaw_at": float(msg.get("yaw_at", 0.0)),
                "action": msg.get("action", ""),
            })
            save_config(self.cfg)
            return _resp({"waypoint": wps[-1]}, wp=wps[-1])

        if op == "record":  # record current robot position
            snap_rec = self.bridge.snapshot()
            pos = snap_rec["position"]
            yaw = snap_rec["yaw"]
            if frame == "map":
                # 与画布一致：用 map 系当前位姿直接入库，不做二次换算，
                # 保证"看到在哪、存的就是哪"。
                tf_rec = self.bridge.slam_tf()
                if tf_rec is None:
                    return {"ok": False,
                            "error": "定位未就绪，点位无法写入地图坐标。请先「加载地图定位」"}
                c_r, s_r = math.cos(tf_rec[2]), math.sin(tf_rec[2])
                x = tf_rec[0] + c_r * pos[0] - s_r * pos[1]
                y = tf_rec[1] + s_r * pos[0] + c_r * pos[1]
                yaw = math.atan2(math.sin(tf_rec[2] + yaw), math.cos(tf_rec[2] + yaw))
            else:
                x, y, yaw = _store_xy(pos[0], pos[1], yaw)
            new_id = max([w["id"] for w in wps], default=0) + 1
            wps.append({
                "id": new_id,
                "name": msg.get("name", f"点{new_id}"),
                "x": round(x, 3),
                "y": round(y, 3),
                "z": round(pos[2], 3),
                "yaw": round(yaw, 3),
                "yaw_at": 0.0,
                "action": "",
            })
            save_config(self.cfg)
            return _resp({"waypoint": wps[-1]}, wp=wps[-1])

        if op == "del":
            wid = int(msg["id"])
            keep = [w for w in wps if w["id"] != wid]
            wps[:] = keep
            save_config(self.cfg)
            return _resp()
            return {"ok": True, "waypoints": self.cfg["waypoints"]}

        if op == "rename":
            wid = int(msg["id"])
            for w in wps:
                if w["id"] == wid:
                    w["name"] = msg.get("name", w["name"])
            save_config(self.cfg)
            return _resp()

        if op == "set_yaw_at":
            wid = int(msg["id"])
            val = msg.get("yaw_at")
            try:
                val = float(val) if val is not None else 0.0
            except (TypeError, ValueError):
                val = 0.0
            for w in wps:
                if w["id"] == wid:
                    w["yaw_at"] = val
            save_config(self.cfg)
            return _resp()

        if op == "set_action":
            wid = int(msg["id"])
            for w in wps:
                if w["id"] == wid:
                    w["action"] = msg.get("action", "")
            save_config(self.cfg)
            return _resp()

        if op == "reposition":
            # Re-record an existing waypoint at the robot's current position
            # (used when an exhibit spot is unusable and needs re-placing).
            wid = int(msg["id"])
            pos = self.bridge.snapshot()["position"]
            yaw = self.bridge.snapshot()["yaw"]
            if not math.isfinite(pos[0]) and not math.isfinite(pos[1]):
                return {"ok": False, "error": "no valid odometry to record"}
            if frame == "map":
                tf_r = self.bridge.slam_tf()
                if tf_r is None:
                    return {"ok": False,
                            "error": "定位未就绪，点位无法写入地图坐标。请先「加载地图定位」"}
                c_r, s_r = math.cos(tf_r[2]), math.sin(tf_r[2])
                x = tf_r[0] + c_r * pos[0] - s_r * pos[1]
                y = tf_r[1] + s_r * pos[0] + c_r * pos[1]
                yaw = math.atan2(math.sin(tf_r[2] + yaw), math.cos(tf_r[2] + yaw))
            else:
                x, y, yaw = _store_xy(pos[0], pos[1], yaw)
            moved = None
            for w in wps:
                if w["id"] == wid:
                    w["x"] = round(x, 3)
                    w["y"] = round(y, 3)
                    w["z"] = round(pos[2], 3)
                    w["yaw"] = round(yaw, 3)
                    moved = w
            save_config(self.cfg)
            return _resp(wp=moved)

        return {"ok": False, "error": f"unknown waypoint op {op}"}

    def _handle_tour(self, msg):
        op = msg.get("op")
        if op == "start":
            res = self.tour.start([int(i) for i in msg.get("ids", [])])
            if res.get("ok"):
                self.control_mode = "tour"
            return res
        if op == "stop":
            res = self.tour.stop()
            self.control_mode = "joystick"
            return res
        if op == "skip":
            return self.tour.skip_intro()
        return {"ok": False, "error": f"unknown tour op {op}"}

    def _handle_slam(self, msg):
        """SLAM 管理：建图 / 保存 / 定位加载 / 停止。

        定位激活时点位库自动切到当前地图（map 系持久化），停止定位后
        回到旧版内置点位（odom 系）。"""
        op = msg.get("op")
        s = self.slam
        if op != "status" and not persistent_map_enabled(self.cfg):
            return {"ok": False,
                    "error": "持久地图已关闭（slam.persistent_map=false），"
                             "当前为 odom 模式"}
        if op == "start_mapping":
            return s.start_mapping()
        if op == "stop_mapping":
            return s.stop_mapping(msg.get("name", ""), save=True)
        if op == "discard_mapping":
            return s.stop_mapping("", save=False)
        if op == "start_localization":
            return s.start_localization(msg.get("name", ""))
        if op == "stop_localization":
            return s.stop_localization()
        if op == "select_map":
            return s.select_map(msg.get("name", ""))
        if op == "set_initial_pose":
            name = s._safe_name(msg.get("name", "") or s.map_name or "")
            try:
                pose = [float(msg["x"]), float(msg["y"]), float(msg.get("yaw", 0.0))]
            except (KeyError, TypeError, ValueError):
                return {"ok": False, "error": "需要 x / y / yaw"}
            store = self.cfg.setdefault("maps", {}).setdefault(name, {"waypoints": []})
            store["initial_pose"] = [round(v, 4) for v in pose]
            save_config(self.cfg)
            log.info("slam: initial pose for %s -> %s", name, pose)
            res = s.start_localization(name, initial_pose=pose)
            return {**res, "ok": True, "msg": "已按初始位姿重新定位"}

        if op == "status":
            return {"ok": True, **s.status()}
        if op == "delete_map":
            name = s._safe_name(msg.get("name", ""))
            import glob as _glob
            removed = []
            for f in _glob.glob(os.path.join(SLAM_DIR, name + ".*")):
                if f.endswith((".posegraph", ".pgm", ".yaml", ".data")):
                    try:
                        os.remove(f)
                        removed.append(os.path.basename(f))
                    except OSError:
                        pass
            if self.cfg.get("maps", {}).pop(name, None) is not None:
                save_config(self.cfg)
            return {"ok": True, "removed": removed, **s.status()}
        return {"ok": False, "error": f"unknown slam op {op}"}

    def _handle_config(self, msg):
        key = msg.get("key")
        value = msg.get("value")
        if key in ("max_speed", "reach_threshold"):
            self.cfg[key] = float(value)
            save_config(self.cfg)
            return {"ok": True, key: self.cfg[key]}
        return {"ok": False, "error": f"unknown config key {key}"}

    def _handle_restart(self):
        """Restart this process in place (re-initialises the ROS/DDS bridge)."""
        log.warning("restart requested via web UI")
        asyncio.get_event_loop().call_later(1.0, _reexec)
        return {"ok": True, "msg": "正在重启…"}


# --------------------------------------------------------------------------
# HTTP static server + entry
# --------------------------------------------------------------------------
CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".js": "application/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".png": "image/png",
    ".svg": "image/svg+xml",
    ".json": "application/json; charset=utf-8",
}


def _read_pgm(path):
    """读 P5/P2 PGM -> (width, height, grayscale bytes, maxval)。"""
    with open(path, "rb") as f:
        data = f.read()
    tokens, i = [], 0
    while len(tokens) < 4 and i < len(data):
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if i < len(data) and data[i:i + 1] == b"#":
            while i < len(data) and data[i:i + 1] != b"\n":
                i += 1
            continue
        start = i
        while i < len(data) and not data[i:i + 1].isspace():
            i += 1
        if start == i:
            break
        tokens.append(data[start:i])
    if len(tokens) < 4:
        raise ValueError("bad pgm header")
    magic, w, h, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    i += 1  # maxval 后的单个空白
    body = data[i:]
    if magic == b"P5":
        px = bytearray(body[:w * h])
    else:
        vals = body.split()
        px = bytearray(int(v) * 255 // maxval for v in vals[:w * h])
    if maxval not in (0, 255):
        px = bytearray(v * 255 // maxval for v in px)
    return w, h, bytes(px), maxval


def _map_png(name):
    """把 slam_maps/<name>.pgm 转成 PNG 字节（不依赖 Pillow）。"""
    import struct
    import zlib
    w, h, px, _ = _read_pgm(os.path.join(SLAM_DIR, name + ".pgm"))
    raw = bytearray()
    for y in range(h):
        raw.append(0)  # filter type 0
        raw += px[y * w:(y + 1) * w]

    def chunk(tag, payload):
        c = struct.pack(">I", len(payload)) + tag + payload
        return c + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF)

    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(bytes(raw), 6))
    png += chunk(b"IEND", b"")
    return w, h, png


def cfg_maps():
    try:
        return WebApp.inst.cfg.get("maps", {})
    except Exception:
        return {}


def _map_meta(name):
    """读 <name>.yaml 得到分辨率/原点；文件不全返回 None。"""
    ypath = os.path.join(SLAM_DIR, name + ".yaml")
    pgm = os.path.join(SLAM_DIR, name + ".pgm")
    if not (os.path.isfile(ypath) and os.path.isfile(pgm)):
        return None
    res, ox, oy, negate = 0.05, 0.0, 0.0, 0
    try:
        with open(ypath, "r", errors="replace") as f:
            for line in f:
                if ":" not in line:
                    continue
                k, v = line.split(":", 1)
                k, v = k.strip(), v.strip()
                try:
                    if k == "resolution":
                        res = float(v)
                    elif k == "origin":
                        nums = [float(x) for x in v.strip("[]").split(",")]
                        ox, oy = nums[0], nums[1]
                    elif k == "negate":
                        negate = int(float(v))
                except ValueError:
                    pass
        w, h, _, _ = _read_pgm(pgm)
    except (OSError, ValueError):
        return None
    return {"name": name, "width": w, "height": h, "resolution": res,
            "origin": [ox, oy], "negate": negate,
            "extent": [ox, oy, ox + w * res, oy + h * res],
            "mtime": os.path.getmtime(pgm),
            "waypoints": len(cfg_maps().get(name, {}).get("waypoints", []))}


def _map_list():
    try:
        names = sorted(f[:-len(".pgm")] for f in os.listdir(SLAM_DIR)
                       if f.endswith(".pgm"))
    except OSError:
        return []
    out = []
    for n in names:
        m = _map_meta(n)
        if m:
            out.append(m)
    return out


NETD_BASE = "http://127.0.0.1:8099"


def _netd_call(sub, query=""):
    """转发到宿主机的 go2-netd（nmcli / BlueZ 特权操作都在那边）。"""
    url = NETD_BASE + sub + (("?" + query) if query else "")
    with urllib.request.urlopen(url, timeout=120) as r:
        return json.loads(r.read().decode("utf-8"))


async def _netd_proxy(sub, query):
    loop = asyncio.get_event_loop()
    try:
        data = await loop.run_in_executor(None, _netd_call, sub, query)
        code, phrase = 200, "OK"
    except Exception as e:
        data = {"ok": False, "msg": "网络配置服务不可用：%s" % e}
        code, phrase = 502, "Bad Gateway"
    body = json.dumps(data, ensure_ascii=False).encode()
    return WsResponse(code, phrase,
                      Headers({"Content-Type": "application/json; charset=utf-8",
                               "Content-Length": str(len(body)),
                               "Cache-Control": "no-store"}), body)


async def http_handler(connection, request):
    """Serve static files from static/. Return None for /ws (WebSocket upgrade)."""
    path = request.path
    # 剥离查询串：静态/API 匹配只看路径部分（网络代理要把查询串原样转发）
    query = ""
    if "?" in path:
        path, query = path.split("?", 1)
    if path in ("/ws", "/ws/"):
        return None  # let the WebSocket handshake proceed
    # route: "/" -> main panel, "/console" -> tour console, "/ai" -> AI panel
    if path in ("/", ""):
        path = "/index.html"
    elif path in ("/console", "/console/"):
        path = "/console.html"
    elif path in ("/ai", "/ai/"):
        path = "/ai.html"
    elif path in ("/mapview", "/mapview/"):
        path = "/mapview.html"
    elif path in ("/motion", "/motion/"):
        path = "/motion.html"
    elif path in ("/connect", "/connect/"):
        path = "/connect.html"

    # ---- 连接向导：网络/蓝牙配置（转发宿主 go2-netd）----
    if path.startswith("/api/net/"):
        return await _netd_proxy(path[len("/api/net"):], query)

    # ---- 地图查看器 API ----
    if path == "/api/maps":
        state = WebApp.inst.slam.status() if hasattr(WebApp, "inst") else {}
        body = json.dumps({"maps": _map_list(),
                           "active": state.get("map", ""),
                           "mode": state.get("mode", "off")},
                          ensure_ascii=False).encode()
        return WsResponse(200, "OK",
                          Headers({"Content-Type": "application/json; charset=utf-8",
                                   "Cache-Control": "no-store"}), body)

    if path.startswith("/api/map/") and path.endswith(".png"):
        name = os.path.basename(path[len("/api/map/"):-len(".png")])
        full = os.path.join(SLAM_DIR, name + ".pgm")
        if not name or not os.path.isfile(full):
            return WsResponse(404, "Not Found", Headers({"Content-Type": "text/plain"}),
                              b"no such map")
        try:
            _, _, png = _map_png(name)
        except Exception as e:
            return WsResponse(500, "Internal Error",
                              Headers({"Content-Type": "text/plain"}),
                              f"pgm decode failed: {e}".encode())
        # websockets 的 WsResponse 不带 Content-Length，部分浏览器会中止
        # 图片解码；这里改为 JSON + base64 data URL，稳定且便于缓存控制。
        import base64 as _b64
        meta = _map_meta(name) or {}
        payload = json.dumps({"name": name,
                              "width": meta.get("width"),
                              "height": meta.get("height"),
                              "png_b64": _b64.b64encode(png).decode()},
                             ensure_ascii=False).encode()
        return WsResponse(200, "OK",
                          Headers({"Content-Type": "application/json; charset=utf-8",
                                   "Content-Length": str(len(payload)),
                                   "Cache-Control": "no-store"}), payload)

    if path.startswith("/api/map/") and path.endswith(".json"):
        name = os.path.basename(path[len("/api/map/"):-len(".json")])
        meta = _map_meta(name) if name else None
        if not meta:
            return WsResponse(404, "Not Found",
                              Headers({"Content-Type": "text/plain"}), b"no such map")
        meta["waypoints"] = cfg_maps().get(name, {}).get("waypoints", [])
        return WsResponse(200, "OK",
                          Headers({"Content-Type": "application/json; charset=utf-8",
                                   "Cache-Control": "no-store"}),
                          json.dumps(meta, ensure_ascii=False).encode())
    # basic path traversal guard
    full = os.path.normpath(os.path.join(STATIC_DIR, path.lstrip("/")))
    if not full.startswith(STATIC_DIR) or not os.path.isfile(full):
        body = b"Not found"
        return WsResponse(404, "Not Found", Headers({"Content-Type": "text/plain"}), body)
    ext = os.path.splitext(full)[1]
    ctype = CONTENT_TYPES.get(ext, "application/octet-stream")
    with open(full, "rb") as f:
        body = f.read()
    return WsResponse(200, "OK", Headers({"Content-Type": ctype}), body)


async def ws_handler(websocket):
    app = WebApp.inst
    app.clients.add(websocket)
    log.info("client connected: %s", websocket.remote_address)
    try:
        # 初始包：当前地图点位库 + 当前底图（新连上的客户端拿不到历史广播）
        wps, frame = active_waypoints(app.cfg, app.bridge, app.slam)
        init = {
            "type": "init",
            "config": {
                "max_speed": app.cfg.get("max_speed", 0.5),
                "reach_threshold": app.cfg.get("reach_threshold", 0.4),
            },
            "waypoints": wps,
            "wp_frame": frame,
            "wp_map": current_map_name(app.cfg, app.slam),
            "frame": frame,
            "persistent_map": persistent_map_enabled(app.cfg),
        }
        try:
            if current_map_name(app.cfg, app.slam):
                init["static_map"] = app._static_map_for()
        except Exception as e:
            log.warning("init static map failed: %s", e)
        await websocket.send(json.dumps(init))
        loop = asyncio.get_event_loop()
        async for raw in websocket:
            # 运动模式指令要等固件回执（最长几百毫秒），放线程里跑，
            # 否则 time.sleep 会卡住整个事件循环（状态广播同时停摆）。
            try:
                needs_thread = json.loads(raw).get("type") == "motion"
            except Exception:
                needs_thread = False
            if needs_thread:
                resp = await loop.run_in_executor(
                    None, app.handle_message, raw, websocket)
            else:
                resp = app.handle_message(raw, websocket)
            if resp:
                await websocket.send(json.dumps({"type": "ack", **resp}))
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        app.clients.discard(websocket)
        if app.ai_client is websocket:
            app.ai_client = None
            app.ai_started = False
        log.info("client disconnected")


async def ai_startup_probe():
    """诊断：启动时立即测 ollama GET 是否在 web 进程内正常工作。"""
    t0 = asyncio.get_event_loop().time()
    try:
        timeout = aiohttp.ClientTimeout(total=15)
        async with aiohttp.ClientSession(timeout=timeout) as session:
            async with session.get("http://127.0.0.1:11434/api/tags") as r:
                data = await r.json()
        log.info("AI_PROBE: OK %.1fs models=%d",
                 asyncio.get_event_loop().time() - t0, len(data.get("models", [])))
    except Exception as e:
        log.error("AI_PROBE: FAILED after %.1fs: %s",
                  asyncio.get_event_loop().time() - t0, e)


async def main():
    cfg = load_config()
    bridge, thread = rb.start_bridge(cfg)
    asyncio.create_task(ai_startup_probe())

    app = WebApp(cfg, bridge)
    WebApp.inst = app

    loop = asyncio.get_event_loop()
    loop.create_task(app.loop())
    # 里程计转发（/sportmodestate -> /sportmodestate_odom）常驻
    threading.Thread(target=app.slam._ensure_odom_bridge, daemon=True).start()
    # 上次关机前开着定位 -> 自动恢复（后台等 DDS 稳定后再拉起）
    if cfg.get("slam", {}).get("resume"):
        threading.Thread(target=app.slam.auto_resume, daemon=True).start()

    host = cfg.get("host", "0.0.0.0")
    port = int(cfg.get("port", 8080))

    async def handler(websocket):
        await ws_handler(websocket)

    server = await websockets.serve(
        handler,
        host,
        port,
        max_size=64 * 1024 * 1024,
        process_request=http_handler,
        # 默认 open_timeout=10s 会掐断长请求：WiFi 扫描/连接、蓝牙扫描都要十几秒以上，
        # 超时后浏览器只看到 "Failed to fetch"（服务端连日志都不留）。放宽到 180s，
        # 够覆盖 nmcli 连接 70s + 失败回滚 40s。
        open_timeout=180,
    )
    log.info("go2 web console listening on http://%s:%d", host, port)

    # ---- HTTPS（Web Bluetooth 要求安全上下文）----
    # 浏览器只在 https:// 或 localhost 下暴露 navigator.bluetooth。
    # 证书用 make_cert.sh 生成的自签证书：手机点过"继续访问"后，
    # Chrome 仍把它算作安全上下文（判断依据是 scheme=https，不是证书链）。
    https_port = int(cfg.get("https_port", 8443))
    cert = os.path.join(BASE_DIR, "certs", "server.crt")
    key = os.path.join(BASE_DIR, "certs", "server.key")
    https_servers = []
    if os.path.exists(cert) and os.path.exists(key):
        try:
            ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            ctx.load_cert_chain(cert, key)
        except Exception as e:
            ctx = None
            log.warning("HTTPS 证书加载失败（HTTP 仍可用，但浏览器蓝牙不可用）: %s", e)
        # 8443 是主端口；443 只是让手机能省掉端口号（https://IP 默认走 443，
        # 少写端口就不会出现"打不开"的误会）。任一端口失败不影响其他端口。
        for p in [https_port] + [int(x) for x in cfg.get("https_extra_ports", [443])]:
            if ctx is None:
                break
            if any(s is not None and getattr(s, "sockets", None) and
                   p in [sock.getsockname()[1] for sock in s.sockets]
                   for s in https_servers):
                continue
            try:
                srv = await websockets.serve(
                    handler, host, p,
                    max_size=64 * 1024 * 1024,
                    process_request=http_handler,
                    open_timeout=180,
                    ssl=ctx,
                )
                https_servers.append(srv)
                log.info("go2 web console listening on https://%s:%d", host, p)
            except Exception as e:
                log.warning("HTTPS 端口 %d 启动失败（其他端口不受影响）: %s", p, e)
        if not https_servers:
            log.warning("HTTPS 全部端口启动失败，浏览器蓝牙不可用")
    else:
        log.warning("没有证书（%s），跳过 HTTPS；浏览器蓝牙需要它，跑 ./make_cert.sh 后重启", cert)

    if https_servers:
        await asyncio.gather(server.wait_closed(),
                             *(s.wait_closed() for s in https_servers))
    else:
        await server.wait_closed()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")
    try:
        asyncio.run(main())
        # 正常情况下 main() 永不返回（server.wait_closed() 一直挂着）。
        # 走到这里说明事件循环自己结束了 —— 记下来，这是 app 静默消失的一种可能。
        log.warning("main() returned normally -- event loop finished, exiting")
    except KeyboardInterrupt:
        log.warning("KeyboardInterrupt -- exiting")
    except BaseException:
        log.exception("main() raised -- exiting")
        raise
    finally:
        logging.shutdown()
