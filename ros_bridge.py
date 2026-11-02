#!/usr/bin/env python3
"""ROS2 bridge for the Go2 web console.

Subscribes to map/odometry topics, publishes navigation/control commands.
Runs rclpy in a background thread; exposes thread-safe state via a lock.

This file lives entirely inside hu/go2/web and talks to the already-running
SCAN-Planner stack over standard ROS2 topics.
"""

import json
import math
import os
import sys
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from unitree_api.msg import Request

# Sport API ids (mirror of ros2_sport_client.h)
SPORT_DAMP = 1001
SPORT_BALANCESTAND = 1002
SPORT_STOPMOVE = 1003
SPORT_STANDUP = 1004
SPORT_STANDDOWN = 1005
SPORT_RECOVERYSTAND = 1006
SPORT_MOVE = 1008
SPORT_SIT = 1009
SPORT_RISESIT = 1010
SPORT_SWITCHJOYSTICK = 1027
SPORT_HEART = 1036
SPORT_HELLO = 1016
SPORT_STRETCH = 1017
SPORT_CONTENT = 1020
SPORT_SCRAPE = 1029
SPORT_DANCE1 = 1022
SPORT_DANCE2 = 1023
SPORT_POSE = 1028
SPORT_FREEWALK = 2045
SPORT_FREEBOUND = 2046
SPORT_FREEJUMP = 2047
SPORT_FREEAVOID = 2048
SPORT_CLASSICWALK = 2049
SPORT_WALKUPRIGHT = 2050
SPORT_CROSSSTEP = 2051
SPORT_BODYHEIGHT = 2056
# 爬楼梯：官方 sports_services 文档有 WalkStair(bool)，但未公布 api_id，
# 且 1.23 固件上 2052/2057 等候选都回 3203（未知 api_id）。所以本页面
# 的爬楼按钮走「探测 + 明确报错」路径，而不是假装成功。

# Exhibit actions: name -> (api_id, default duration seconds, label)
# Per official Unitree sports_services docs:
#   Hello=打招呼, Dance1=舞蹈段落1, Dance2=舞蹈段落2, Scrape=拜年作揖,
#   Stretch=伸懒腰, Heart=比心, BalanceStand=平衡站
# Go2 has no "handshake" action in the official docs, so it is not included.
EXHIBIT_ACTIONS = {
    "": (None, 0, "无动作"),
    "hello": (SPORT_HELLO, 2.5, "👋 打招呼"),
    "heart": (SPORT_HEART, 2.0, "❤️ 比心"),
    "dance1": (SPORT_DANCE1, 4.0, "💃 舞蹈1"),
    "dance2": (SPORT_DANCE2, 4.0, "🕺 舞蹈2"),
    "stretch": (SPORT_STRETCH, 2.0, "🧘 伸懒腰"),
    "scrape": (SPORT_SCRAPE, 2.5, "🙏 拜年作揖"),
    "balance": (SPORT_BALANCESTAND, 3.0, "⚖️ 平衡站"),
}

# 运动模式页专用动作表：name -> (api_id, 是否带 bool 参数, 目标开关状态, 说明)
# bool 参数的 API 用 {"data": true/false} 下发（见 ros2_sport_client.cpp）。
#
# 灵动模式（FreeWalk, 2045）在 Go2 上是一次性「进入自由行走」命令，不带参数，
# 进入后由底层自主选步态，本身就能上下楼梯，不需要单独的爬楼接口。
# 退出灵动回普通站立用 StandUp，所以它不算 bool 型模式。
MOTION_ACTIONS = {
    "hello": (SPORT_HELLO, False, None, "打招呼"),
    "sit": (SPORT_SIT, False, None, "坐下"),
    "risesit": (SPORT_RISESIT, False, None, "站起"),
    "stretch": (SPORT_STRETCH, False, None, "伸懒腰"),
    "scrape": (SPORT_SCRAPE, False, None, "拜年作揖"),
    "dance1": (SPORT_DANCE1, False, None, "舞蹈段落1"),
    "dance2": (SPORT_DANCE2, False, None, "舞蹈段落2"),
    "heart": (SPORT_HEART, False, None, "比心"),
    "freewalk": (SPORT_FREEWALK, False, None, "灵动模式"),
    # 下列为「进入/退出」型模式：true 进入，false 退出回灵动模式
    "walkupright": (SPORT_WALKUPRIGHT, True, True, "后腿直立模式"),
    "crossstep": (SPORT_CROSSSTEP, True, True, "交叉步模式"),
    "freebound": (SPORT_FREEBOUND, True, True, "并腿跑模式"),
    "freejump": (SPORT_FREEJUMP, True, True, "跳跃模式"),
    "freeavoid": (SPORT_FREEAVOID, True, True, "闪避模式"),
    "classicwalk": (SPORT_CLASSICWALK, True, True, "经典步态"),
    "pose": (SPORT_POSE, True, True, "摆姿势"),
}

# 退出型模式 -> api_id，用于「一键退出所有模式」与互斥切换
BOOL_MODES = {name: (api, desc) for name, (api, is_bool, _on, desc)
              in MOTION_ACTIONS.items() if is_bool}
# 互斥模式组：同一时刻只允许一个「灵动子模式」处于开启状态
EXCLUSIVE_MODES = ("walkupright", "crossstep", "freebound", "freejump",
                   "freeavoid", "classicwalk", "pose")


RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)

# pointcloud_to_laserscan 以 BEST_EFFORT 发布 /scan，订阅必须匹配，否则收不到
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    durability=DurabilityPolicy.VOLATILE,
)


def _pointcloud2_to_points(msg, max_points=4000):
    """Extract (x, y, z) from a PointCloud2, downsampled to max_points.

    先整体拷贝出 DDS 借出的缓冲，再用 numpy 按 stride 一次取出 x/y/z。
    逐点 struct.unpack 在 18Hz/2.4MB/s 的地图点云上太慢，解析窗口越长，
    底层缓冲被回收后越容易踩到已知的段错误（见 web.log 历史记录）。
    """
    fields = {f.name: (f.offset, f.datatype) for f in msg.fields}
    if "x" not in fields or "y" not in fields:
        return []
    step = msg.point_step
    n = msg.width * msg.height
    if n == 0 or step <= 0:
        return []
    data = bytes(msg.data)
    if len(data) < n * step:
        n = len(data) // step
        if n == 0:
            return []

    try:
        import numpy as np
        ox = fields["x"][0]
        oy = fields["y"][0]
        oz = fields["z"][0] if "z" in fields else None
        dt = np.dtype({
            "names": ["x", "y", "z"],
            "formats": ["<f4", "<f4", "<f4"],
            "offsets": [ox, oy, oz if oz is not None else 0],
            "itemsize": step,
        })
        a = np.frombuffer(data, dtype=dt, count=n)
        stride = max(1, n // max_points)
        xs = a["x"][::stride]
        ys = a["y"][::stride]
        zs = a["z"][::stride] if oz is not None else np.zeros_like(xs)
        ok = np.isfinite(xs) & np.isfinite(ys)
        return list(zip(xs[ok].tolist(), ys[ok].tolist(), zs[ok].tolist()))
    except Exception:
        pass

    stride = max(1, n // max_points)
    points = []
    for i in range(0, n, stride):
        off = i * step
        x = _read_float(data, off + fields["x"][0])
        y = _read_float(data, off + fields["y"][0])
        z = _read_float(data, off + fields["z"][0]) if "z" in fields else 0.0
        if math.isfinite(x) and math.isfinite(y):
            points.append((x, y, z))
    return points


def _read_float(data, offset):
    """Read a float32 at offset from a bytes-like buffer."""
    if offset + 4 > len(data):
        return float("nan")
    import struct

    return struct.unpack_from("<f", data, offset)[0]


class TfFilter:
    """map->odom 平滑器：把 slam_toolbox 的逐帧重解噪声压成缓慢修正。

    背景：slam_toolbox 的 localization 模式每帧都会重新求解绝对位姿并直接
    写进 map->odom。展厅是开放展台结构、特征稀疏，配准时存在大量"看起来
    都合理"的解，实测 map->odom 每 3 秒跳 0.76m（累计跳变位移甚至超过机器人
    自身走过的路程），导致 map 系下的机器人位置随机乱跳、导览目标点漂移。

    这里不改 slam_toolbox 的输出，只在消费侧做三道限制：

    1. 限速：每帧最多采纳 MAX_STEP_M / MAX_STEP_YAW 的变化，多余的留到后续
       帧慢慢消化 —— 把跳变磨成平滑过渡。
    2. 拒绝孤立跳变：单帧变化超过 JUMP_M / JUMP_YAW 视为误匹配，直接丢弃
       （不更新状态），除非同一方向持续出现（见 3）。
    3. 持续偏离才采纳：连续 PERSIST_S 秒、且每次都在相近方向偏离，说明是
       真实漂移而非噪声，此时放开跳变拒绝，逐步跟上。

    参数与场地尺度无关：约束的是"单次修正上限"和"持续性判据"，40m 展厅
    和 4m 办公室同样适用。
    """
    MAX_STEP_M = 0.15         # 每帧（约 10Hz）最多移动 15cm -> 上限 1.5 m/s
    MAX_STEP_YAW = math.radians(8.0)
    JUMP_M = 0.50             # 单帧超过 50cm 视为可疑跳变
    JUMP_YAW = math.radians(20.0)
    # "持续同向偏离"才认定是真实漂移。判据要严：跳变本身每 3 秒一次、
    # 方向随匹配噪声乱变，若用 1.5s/0.25m 这种松阈值，几乎每次跳变都会被
    # 当成真实漂移放行，滤波等于没用（实测只有 3 倍改善）。改成：
    # 至少持续 4s、累计同向偏移 >= 0.8m、且期间每次偏移方向一致。
    PERSIST_S = 4.0
    PERSIST_M = 0.8
    PERSIST_DIR_COS = 0.7     # 相邻偏移方向夹角余弦 > 0.7 才算"同向"
    UNSTABLE_WINDOW = 10.0    # 不稳定判定的滑动窗口
    UNSTABLE_COUNT = 8        # 窗口内跳变次数超过这个 -> 判为不稳定

    def __init__(self):
        self.value = None             # 平滑后的 (x, y, yaw)
        self._raw = None              # 最近一次原始值（用于偏离判据）
        self._rstamp = 0.0
        self._rejects = 0             # 连续拒绝次数
        self._reject_dir = None       # 连续拒绝的累计方向
        self._last_off = None         # 上一次偏移向量（判同向）
        self._reject_since = 0.0
        self._jump_times = []         # 近期疑似跳变的时间戳
        self._last_apply = 0.0
        self.stats = {"accepted": 0, "rejected": 0, "clipped": 0,
                      "jumps": 0, "persisted": 0}

    # ---- 数学小工具 ----
    @staticmethod
    def _wrap(a):
        return math.atan2(math.sin(a), math.cos(a))

    @staticmethod
    def _dist(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _is_unstable(self, now):
        # 只能在这里裁剪：unstable_count() 是纯查询，不能有副作用
        self._jump_times = [t for t in self._jump_times
                            if now - t <= self.UNSTABLE_WINDOW]
        return len(self._jump_times) >= self.UNSTABLE_COUNT

    def unstable(self, now=None):
        return self._is_unstable(now if now is not None else time.time())

    def unstable_count(self, now=None):
        """近 UNSTABLE_WINDOW 秒内的跳变次数（只读，不改状态）。

        注意：这里必须自己过滤时间窗口，不能依赖 _jump_times 已被裁剪——
        _is_unstable() 才有裁剪副作用，而状态广播会以 10Hz 调本函数。
        """
        now = now if now is not None else time.time()
        return len([t for t in self._jump_times
                    if now - t <= self.UNSTABLE_WINDOW])

    def push(self, raw, now=None):
        """送入一帧原始 map->odom(dx, dy, yaw)，返回平滑后应使用的值。"""
        now = now if now is not None else time.time()
        raw = (float(raw[0]), float(raw[1]), float(raw[2]))
        self._raw = raw
        self._rstamp = now

        if self.value is None:
            # 首帧（或刚启动定位）直接采纳，作为基准
            self.value = raw
            self._last_apply = now
            self.stats["accepted"] += 1
            return self.value

        dpos = self._dist(raw, self.value)
        dyaw = abs(self._wrap(raw[2] - self.value[2]))

        # ---- 2. 疑似跳变 ----
        if dpos > self.JUMP_M or dyaw > self.JUMP_YAW:
            self.stats["jumps"] += 1
            self._jump_times.append(now)
            off_x, off_y = raw[0] - self.value[0], raw[1] - self.value[1]
            off_len = math.hypot(off_x, off_y)
            same_dir = True
            if self._reject_dir is not None and self._last_off is not None:
                prev_len = math.hypot(*self._last_off)
                if prev_len > 1e-6 and off_len > 1e-6:
                    cosang = ((self._last_off[0] * off_x
                               + self._last_off[1] * off_y)
                              / (prev_len * off_len))
                    same_dir = cosang >= self.PERSIST_DIR_COS
            if self._reject_dir is None or not same_dir:
                # 方向变了 -> 重新开始累计（说明是抖动，不是漂移）
                self._reject_dir = (off_x, off_y)
                self._reject_since = now
            else:
                self._reject_dir = (self._reject_dir[0] + off_x,
                                    self._reject_dir[1] + off_y)
            self._last_off = (off_x, off_y)
            acc = math.hypot(*self._reject_dir)
            # ---- 3. 持续同向偏离 -> 认定真实漂移，放开限制 ----
            if (now - self._reject_since >= self.PERSIST_S
                    and acc >= self.PERSIST_M):
                self.stats["persisted"] += 1
                # 真实漂移：直接跳到新基准（但保留限速，避免一次性瞬移）
                self.value = self._clip_toward(self.value, raw)
                self._rejects = 0
                self._reject_dir = None
                self._last_off = None
                self._last_apply = now
                self.stats["accepted"] += 1
                return self.value
            self.stats["rejected"] += 1
            self._rejects += 1
            return self.value

        # ---- 1. 正常范围内：限速逼近 ----
        self._rejects = 0
        self._reject_dir = None
        self._last_off = None
        new = self._clip_toward(self.value, raw)
        if self._dist(new, raw) > 1e-4 or abs(self._wrap(new[2] - raw[2])) > 1e-4:
            self.stats["clipped"] += 1
        else:
            self.stats["accepted"] += 1
        self.value = new
        self._last_apply = now
        return self.value

    def _clip_toward(self, cur, target):
        """朝 target 移动，但单帧不超过 MAX_STEP_M / MAX_STEP_YAW。"""
        dx, dy = target[0] - cur[0], target[1] - cur[1]
        d = math.hypot(dx, dy)
        if d > self.MAX_STEP_M:
            k = self.MAX_STEP_M / d
            dx, dy = dx * k, dy * k
        dyaw = self._wrap(target[2] - cur[2])
        if abs(dyaw) > self.MAX_STEP_YAW:
            dyaw = math.copysign(self.MAX_STEP_YAW, dyaw)
        return (cur[0] + dx, cur[1] + dy, self._wrap(cur[2] + dyaw))

    def reset(self, value=None):
        self.value = value
        self._raw = value
        self._rstamp = time.time() if value is not None else 0.0
        self._rejects = 0
        self._reject_dir = None
        self._last_off = None
        self._reject_since = 0.0
        self._jump_times = []


class RosBridge(Node):
    """ROS2 node bridging robot topics to the web layer."""

    def __init__(self, cfg):
        super().__init__("go2_web_bridge")
        self._lock = threading.Lock()

        self.cfg = cfg
        topics = cfg.get("topics", {})

        # ---- shared state ----
        self.map_points = []        # [(x, y, z), ...]  (downsampled)
        self.map_stamp = 0.0
        self.position = (0.0, 0.0, 0.0)
        self.yaw = 0.0
        self.odom_stamp = 0.0
        self.odom_receive_time = 0.0
        self.planner_alive = False
        self.planner_stamp = 0.0
        # 点云解析限频：地图点云 18Hz/2.4MB/s，没必要每帧都进 Python
        self._map_parse_ts = 0.0
        self._lidar_parse_ts = 0.0

        # ---- subscribers ----
        self._map_sub = self.create_subscription(
            PointCloud2, topics.get("map", "/grid_map/occupancy"),
            self._map_cb, RELIABLE_QOS)
        self._odom_sub = self.create_subscription(
            Odometry, topics.get("odom", "/sportmodestate_odom"),
            self._odom_cb, RELIABLE_QOS)

        # ---- publishers ----
        self._goal_pub = self.create_publisher(
            PoseStamped, topics.get("goal", "/move_base_simple/goal"), 10)
        self._cmd_vel_pub = self.create_publisher(
            Twist, topics.get("cmd_vel", "/cmd_vel"), 10)
        self._sport_pub = self.create_publisher(
            Request, topics.get("sport_request", "/api/sport/request"), 10)
        # Pause signal for the bridge (so it stops overriding exhibit actions)
        self._pause_pub = self.create_publisher(
            Bool, topics.get("motion_pause", "/go2/motion_pause"), 10)

        # ---- SLAM 定位层（slam_toolbox 在 /tf 上发 map->odom）----
        # 定位激活时，点位的持久化与导览都以 map 系为准；本节点只负责
        # 维护 map<->odom 的 2D 换算，app 层决定何时使用。
        self.slam_mode = "off"          # off | mapping | localization（app 层写）
        self.slam_map = ""              # 当前地图名（app 层写）
        self._map_odom = None           # 原始 map->odom（最后收到的）
        self._map_odom_stamp = 0.0
        self._tf_filter = TfFilter()    # 平滑后的 map->odom，供 app 层使用
        self.create_subscription(TFMessage, "/tf", self._tf_cb, 10)

        # 持久地图栅格（slam_toolbox 在定位/建图时发布 /map），
        # 用于网页把"稳定地图"画成背景
        self._static_map = None   # (width, height, resolution, origin, bytes, stamp)
        try:
            from nav_msgs.msg import OccupancyGrid
            self.create_subscription(OccupancyGrid, "/map", self._grid_cb, 10)
        except ImportError:
            OccupancyGrid = None

        # 最近一帧 /scan，供 app 层做定位置信度自检（扫描 vs 地图匹配残差）
        self._last_scan = None
        self.create_subscription(
            LaserScan, topics.get("scan", "/scan"), self._scan_cb, SENSOR_QOS)

        # 运动状态（mode/gait/error_code）：运动模式页用来显示当前模式与
        # 下发指令后的真实回执。低频话题即可，动作反馈另有 response 订阅。
        self.sport_state = {}
        try:
            from unitree_go.msg import SportModeState
            self.create_subscription(
                SportModeState, topics.get("sport_state", "/lf/sportmodestate"),
                self._sport_state_cb, SENSOR_QOS)
        except ImportError:
            self.get_logger().warn("unitree_go not importable; sport state off")

        # 运动指令回执：未知 api_id 回 3203、参数错回 3202，界面据此报错，
        # 不再把「发出去」当成「成功了」。
        self.last_sport_resp = None
        try:
            from unitree_api.msg import Response
            self.create_subscription(Response, "/api/sport/response",
                                     self._sport_resp_cb, 10)
        except ImportError:
            pass

        # ---- 机器人麦克风 ----
        # /audiosender: unitree_go/AudioData，50Hz、20ms Opus 单声道 16k。
        # 回调里只入队原始包，解码/识别交给 voice.py，避免拖慢 DDS 线程。
        self._audio_q = deque(maxlen=250)   # 约 5s
        self._audio_n = 0                   # 单调递增包序号（供消费端做游标）
        self._audio_last = 0.0
        try:
            from unitree_go.msg import AudioData
            self.create_subscription(
                AudioData, topics.get("audio_mic", "/audiosender"),
                self._audio_cb, SENSOR_QOS)
        except ImportError:
            self.get_logger().warn("unitree_go AudioData 不可用，麦克风关闭")

        # ---- 激光雷达净空（/utlidar/cloud_base 是 base_link 系，正前 x、左 y）----
        self.lidar = {}
        self.create_subscription(
            PointCloud2, topics.get("lidar", "/utlidar/cloud_base"),
            self._lidar_cb, SENSOR_QOS)

        # ---- 电池 ----
        self.battery = {}
        try:
            from unitree_go.msg import LowState
            self.create_subscription(
                LowState, topics.get("lowstate", "/lf/lowstate"),
                self._lowstate_cb, SENSOR_QOS)
        except ImportError:
            pass

        # ---- 摄像头：videohub RPC ----
        # VideoClient.GetImageSample() 等价于向 /api/videohub/request 发
        # api_id=1001，响应 Response.binary 就是 JPEG。走 rclpy 比在主机
        # 上跑 unitree_sdk2py 稳定（后者在 py3.8 上直接段错误）。
        self._video_pub = self.create_publisher(
            Request, "/api/videohub/request", 10)
        self._video_lock = threading.Lock()
        self._video_wait = None          # (id, Event, result dict)
        self._video_fail = 0
        try:
            from unitree_api.msg import Response
            self.create_subscription(Response, "/api/videohub/response",
                                     self._video_resp_cb, 10)
        except ImportError:
            pass

        self.get_logger().info("ROS bridge ready")

    # ---------- callbacks ----------
    def _map_cb(self, msg):
        # 3Hz 足够画布刷新；18Hz 全解析只会放大已知的 DDS 缓冲段错误风险
        now = time.time()
        if now - self._map_parse_ts < 0.33:
            return
        self._map_parse_ts = now
        points = _pointcloud2_to_points(msg, max_points=self.cfg.get("map_downsample", 4000))
        with self._lock:
            self.map_points = points
            self.map_stamp = self._now_sec(msg.header.stamp)

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        with self._lock:
            self.position = (p.x, p.y, p.z)
            self.yaw = yaw
            self.odom_stamp = self._now_sec(msg.header.stamp)
            self.odom_receive_time = time.time()

    @staticmethod
    def _now_sec(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def _grid_cb(self, msg):
        """缓存 slam_toolbox 的 /map 栅格（-1=未知 0=空 100=占用）。"""
        info = msg.info
        data = bytes(msg.data)
        with self._lock:
            self._static_map = (info.width, info.height, info.resolution,
                                info.origin.position.x, info.origin.position.y,
                                data, time.time())

    def static_map(self):
        with self._lock:
            return self._static_map

    def _scan_cb(self, msg):
        with self._lock:
            self._last_scan = (list(msg.ranges), msg.angle_min,
                               msg.angle_increment, msg.range_min, msg.range_max)

    def _sport_state_cb(self, msg):
        with self._lock:
            self.sport_state = {
                "mode": int(msg.mode),
                "gait_type": int(msg.gait_type),
                "error_code": int(msg.error_code),
                "body_height": round(float(msg.body_height), 3),
                "foot_raise_height": round(float(msg.foot_raise_height), 3),
                "progress": round(float(msg.progress), 2),
                "stamp": self._now_sec(msg.stamp),
            }

    def _sport_resp_cb(self, msg):
        with self._lock:
            self.last_sport_resp = {
                "api_id": int(msg.header.identity.api_id),
                "code": int(msg.header.status.code),
                "data": msg.data,
                "ts": time.time(),
            }

    # ---------- 麦克风 / 雷达 / 电池 ----------
    def _audio_cb(self, msg):
        data = bytes(msg.data)
        if not data:
            return
        with self._lock:
            self._audio_n += 1
            self._audio_q.append((self._audio_n, data))
            self._audio_last = time.time()

    def audio_packets(self, cursor):
        """取游标之后的新 Opus 包，返回 (packets, next_cursor)。"""
        with self._lock:
            pkts = [d for n, d in self._audio_q if n > cursor]
            return pkts, self._audio_n

    def audio_cursor(self):
        with self._lock:
            return self._audio_n

    def audio_alive(self, max_age=1.0):
        return (time.time() - self._audio_last) < max_age

    def _lidar_cb(self, msg):
        """按四个扇区取最近障碍距离。z 轴过滤掉地面与天花板点。"""
        now = time.time()
        if now - self._lidar_parse_ts < 0.1:
            return
        self._lidar_parse_ts = now
        pts = _pointcloud2_to_points(msg, max_points=4000)
        if not pts:
            return
        best = {"front": 99.0, "front_narrow": 99.0,
                "left": 99.0, "right": 99.0, "back": 99.0}
        for p in pts:
            x, y, z = p[0], p[1], p[2]
            # base_link 离地约 0.39m：z<-0.30 基本是地面点，不能当障碍
            if z > 1.2 or z < -0.30:
                continue
            d = math.hypot(x, y)
            if d < 0.15:
                continue
            ang = math.atan2(y, x)
            if abs(ang) <= math.radians(20):
                # 窄扇区：只看向正前方，跟随时避免把旁边桌子算成障碍
                if d < best["front_narrow"]:
                    best["front_narrow"] = d
            if abs(ang) <= math.pi / 4:
                k = "front"
            elif abs(ang) >= 3 * math.pi / 4:
                k = "back"
            elif ang > 0:
                k = "left"
            else:
                k = "right"
            if d < best[k]:
                best[k] = d
        best["min"] = round(min(best["front"], best["left"],
                                best["right"], best["back"]), 3)
        for k in ("front", "front_narrow", "left", "right", "back"):
            best[k] = round(best[k], 3)
        best["stamp"] = time.time()
        with self._lock:
            self.lidar = best

    def _lowstate_cb(self, msg):
        soc = None
        try:
            soc = int(msg.bms_state.soc)
        except Exception:
            pass
        with self._lock:
            self.battery = {
                "soc": soc,
                "voltage": round(float(msg.power_v), 2),
                "current": round(float(msg.power_a), 2),
                "stamp": time.time(),
            }

    def _video_resp_cb(self, msg):
        ident = msg.header.identity
        with self._video_lock:
            w = self._video_wait
            if w is None or w[0] != int(ident.id):
                return
            if int(ident.api_id) != 1001:
                return
            w[2]["code"] = int(msg.header.status.code)
            try:
                import numpy as np
                w[2]["data"] = np.asarray(msg.binary, dtype=np.int8).tobytes()
            except Exception:
                w[2]["data"] = bytes(int(b) & 0xFF for b in msg.binary)
            w[1].set()

    def request_frame(self, timeout=1.5):
        """请求一帧前视相机 JPEG；失败返回 None。只能在非 rclpy 线程调用。"""
        if self._video_pub is None:
            return None
        ev = threading.Event()
        result = {}
        ident = time.monotonic_ns()
        with self._video_lock:
            self._video_wait = (ident, ev, result)
        try:
            req = Request()
            req.header.identity.id = ident
            req.header.identity.api_id = 1001
            req.header.lease.id = 0
            req.header.policy.priority = 0
            req.header.policy.noreply = False
            self._video_pub.publish(req)
            if not ev.wait(timeout):
                self._video_fail += 1
                return None
            if result.get("code", -1) != 0:
                self._video_fail += 1
                return None
            data = result.get("data") or None
            if data:
                self._video_fail = 0
            return data
        finally:
            with self._video_lock:
                self._video_wait = None

    def _tf_cb(self, msg):
        for t in msg.transforms:
            if t.header.frame_id == "map" and t.child_frame_id == "odom":
                tr = t.transform.translation
                q = t.transform.rotation
                yaw = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                )
                now = time.time()
                with self._lock:
                    self._map_odom = (tr.x, tr.y, yaw)
                    self._map_odom_stamp = now
                    # 平滑后的值才是 app/导览真正使用的 map->odom
                    smooth = self._tf_filter.push((tr.x, tr.y, yaw), now)
                # 存盘用平滑值：重启后恢复的基准与运行中一致
                self._save_last_tf(smooth)

    def _save_last_tf(self, tf):
        """把 map->odom 存盘：重启后即使定位没开，也能继续用 map 系显示点位。"""
        now = time.time()
        if now - getattr(self, "_tf_save_ts", 0) < 10.0:
            return
        self._tf_save_ts = now
        try:
            p = self.cfg.get("slam", {}).get("last_tf_file") or \
                "/home/unitree/hu/go2/web/slam_last_tf.json"
            with open(p, "w") as f:
                json.dump({"map": self.cfg.get("slam", {}).get("last_map", ""),
                           "tf": [round(v, 5) for v in tf],
                           "ts": int(now)}, f)
        except OSError:
            pass

    def load_last_tf(self):
        """启动时载入上次保存的 map->odom。"""
        try:
            p = self.cfg.get("slam", {}).get("last_tf_file") or \
                "/home/unitree/hu/go2/web/slam_last_tf.json"
            with open(p) as f:
                d = json.load(f)
            tf = d.get("tf")
            if tf and len(tf) == 3:
                with self._lock:
                    self._map_odom = tuple(float(v) for v in tf)
                    self._map_odom_stamp = 0.0  # 标记为非实时
                    # 平滑器以存盘值为起点，避免重启后第一次匹配被当成跳变
                    self._tf_filter.reset(tuple(float(v) for v in tf))
                self.get_logger().info(f"loaded last map->odom {tf}")
        except (OSError, ValueError):
            pass

    # ---------- map <-> odom 换算（2D, yaw 用 atan2 包装） ----------
    def slam_tf(self, max_age=5.0):
        """返回平滑后的 map->odom (x, y, yaw)；过期或不存在返回 None。"""
        with self._lock:
            if (self._tf_filter.value is None
                    or time.time() - self._map_odom_stamp > max_age):
                return None
            return self._tf_filter.value

    def slam_tf_raw(self):
        """最后收到的原始 map->odom（诊断用，不要用于换算）。"""
        with self._lock:
            return self._map_odom

    def slam_tf_last(self):
        """返回最后一次已知的平滑 map->odom（忽略时效）。

        用户手动停止定位后，点位仍以 map 系保存与显示；此时没有新的
        TF，就用最后已知值换算，避免整批点位跳回旧坐标系。
        """
        with self._lock:
            return self._tf_filter.value

    def slam_stability(self):
        """定位稳定性：unstable=True 表示近期跳变频繁，定位结果不可信。"""
        f = self._tf_filter
        return {"unstable": f.unstable(),
                "jumps_recent": f.unstable_count(),
                "stats": dict(f.stats)}

    def odom_to_map(self, x, y, yaw=0.0):
        tf = self.slam_tf()
        if tf is None:
            return None
        c, s = math.cos(tf[2]), math.sin(tf[2])
        mx = tf[0] + c * x - s * y
        my = tf[1] + s * x + c * y
        myaw = math.atan2(math.sin(tf[2] + yaw), math.cos(tf[2] + yaw))
        return (mx, my, myaw)

    def map_to_odom(self, x, y, yaw=0.0):
        tf = self.slam_tf()
        if tf is None:
            return None
        dx, dy = x - tf[0], y - tf[1]
        c, s = math.cos(-tf[2]), math.sin(-tf[2])
        ox = c * dx - s * dy
        oy = s * dx + c * dy
        oyaw = math.atan2(math.sin(yaw - tf[2]), math.cos(yaw - tf[2]))
        return (ox, oy, oyaw)

    # ---------- snapshot for web ----------
    def topic_publisher_count(self, topic):
        """Return the number of currently discovered publishers for *topic*.

        A process can remain alive after its DDS participant has lost the
        network or was started with a different CycloneDDS interface.  Process
        existence is therefore not enough to decide whether a bridge works;
        use the ROS graph seen by this node instead.
        """
        try:
            return self.count_publishers(str(topic))
        except Exception:
            return 0

    def snapshot(self):
        with self._lock:
            tf_ok = (self._tf_filter.value is not None
                     and time.time() - self._map_odom_stamp <= 5.0)
            return {
                "map_points": list(self.map_points),
                "map_stamp": self.map_stamp,
                "position": self.position,
                "yaw": self.yaw,
                "odom_stamp": self.odom_stamp,
                "odom_receive_time": self.odom_receive_time,
                "slam_mode": self.slam_mode,
                "slam_map": self.slam_map,
                "slam_tf_ok": tf_ok,
                "slam_unstable": self._tf_filter.unstable(),
                "slam_jumps": self._tf_filter.unstable_count(),
                "scan": self._last_scan,
            }

    # ---------- commands ----------
    def send_goal(self, x, y, yaw=0.0):
        msg = PoseStamped()
        msg.header.frame_id = "odom"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self._goal_pub.publish(msg)
        self.get_logger().info(f"goal -> ({x:.2f}, {y:.2f}, yaw {yaw:.2f})")

    def send_cmd_vel(self, vx, vy, vyaw):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(vyaw)
        self._cmd_vel_pub.publish(msg)

    def send_sport(self, api_id, param=None):
        """Send a low-level sport request (StandUp/Damp/Sit/...)."""
        req = Request()
        req.header.identity.api_id = api_id
        if param is not None:
            req.parameter = param
        self._sport_pub.publish(req)
        self.get_logger().info(f"sport api_id={api_id}")

    def send_sport_bool(self, api_id, flag):
        """Send a bool-parameterised sport request (WalkUpright/CrossStep/...).

        The firmware expects {"data": <bool>} — the same envelope that
        ros2_sport_client.cpp builds for every flag-taking API.
        """
        self.send_sport(api_id, json.dumps({"data": bool(flag)}))
        self.get_logger().info(f"sport api_id={api_id} flag={bool(flag)}")

    def send_move(self, vx, vy, vyaw):
        """Send a velocity command through the sport channel (like the official
        remote controller -> SportClient.Move). Independent of /cmd_vel, which
        is used by SCAN-Planner's closed-loop controller during tours."""
        import json as _json

        req = Request()
        req.header.identity.api_id = SPORT_MOVE
        req.parameter = _json.dumps({"x": float(vx), "y": float(vy), "z": float(vyaw)})
        self._sport_pub.publish(req)

    def send_stopmove(self):
        """StopMove (api 1003): halt motion."""
        req = Request()
        req.header.identity.api_id = SPORT_STOPMOVE
        self._sport_pub.publish(req)

    def send_damp(self):
        """Damp (api 1001): drop into damping mode immediately."""
        req = Request()
        req.header.identity.api_id = SPORT_DAMP
        self._sport_pub.publish(req)
        self.get_logger().warn("DAMP issued (emergency)")

    def send_pause(self, pause: bool):
        """Tell the bridge to pause (True) or resume (False) sending Move
        commands, so exhibit actions/turns are not overridden."""
        msg = Bool()
        msg.data = bool(pause)
        self._pause_pub.publish(msg)

    def send_action(self, name):
        """Send an exhibit action (heart/dance/...). Returns (api_id, duration)."""
        api_id, duration, _ = EXHIBIT_ACTIONS.get(name, (None, 0, ""))
        if api_id is None:
            return None, 0.0
        req = Request()
        req.header.identity.api_id = api_id
        self._sport_pub.publish(req)
        self.get_logger().info(f"action: {name} (api {api_id})")
        return api_id, duration


def start_bridge(cfg):
    """Start the rclpy bridge in a background thread. Returns (bridge, thread)."""
    rclpy.init()
    bridge = RosBridge(cfg)

    def _spin():
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(bridge)
        try:
            executor.spin()
        except BaseException as e:  # pragma: no cover
            bridge.get_logger().error(f"spin error: {e}")
            # rclpy/CycloneDDS 崩坏时解释器内存已被写坏（表现为
            # "XXX lineno: -38, opcode: 0" / "SystemError: unknown opcode"），
            # 进程随后必死。与其等 systemd 看门狗 2 分钟后才发现，不如立刻
            # 原地 re-exec 重建 DDS participant —— 只断约 2 秒。
            # 用 os.write 而非 log.*，避免另一线程正持 logging 锁时 execv 死锁。
            try:
                os.write(2, ("go2web: ROS spin died (%r), re-exec to rebuild DDS\n"
                             % (e,)).encode())
            except Exception:
                pass
            os.execv(sys.executable,
                     [sys.executable, "/home/unitree/hu/go2/web/app.py"])
        finally:
            executor.shutdown()

    thread = threading.Thread(target=_spin, daemon=True, name="rclpy-spin")
    thread.start()
    return bridge, thread
