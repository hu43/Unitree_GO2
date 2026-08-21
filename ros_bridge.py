#!/usr/bin/env python3
"""ROS2 bridge for the Go2 web console.

Subscribes to map/odometry topics, publishes navigation/control commands.
Runs rclpy in a background thread; exposes thread-safe state via a lock.

This file lives entirely inside hu/go2/web and talks to the already-running
SCAN-Planner stack over standard ROS2 topics.
"""

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
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
SPORT_FREEWALK = 2045

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)


def _pointcloud2_to_points(msg, max_points=4000):
    """Extract (x, y, z) from a PointCloud2, downsampled to max_points."""
    fields = {f.name: (f.offset, f.datatype) for f in msg.fields}
    if "x" not in fields or "y" not in fields:
        return []
    step = msg.point_step
    data = msg.data
    n = msg.width * msg.height
    if n == 0:
        return []

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
        self.planner_alive = False
        self.planner_stamp = 0.0

        # ---- subscribers ----
        self._map_sub = self.create_subscription(
            PointCloud2, topics.get("map", "/grid_map/occupancy"),
            self._map_cb, RELIABLE_QOS)
        self._odom_sub = self.create_subscription(
            Odometry, topics.get("odom", "/utlidar/robot_odom"),
            self._odom_cb, RELIABLE_QOS)

        # ---- publishers ----
        self._goal_pub = self.create_publisher(
            PoseStamped, topics.get("goal", "/move_base_simple/goal"), 10)
        self._cmd_vel_pub = self.create_publisher(
            Twist, topics.get("cmd_vel", "/cmd_vel"), 10)
        self._sport_pub = self.create_publisher(
            Request, topics.get("sport_request", "/api/sport/request"), 10)

        self.get_logger().info("ROS bridge ready")

    # ---------- callbacks ----------
    def _map_cb(self, msg):
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

    @staticmethod
    def _now_sec(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    # ---------- snapshot for web ----------
    def snapshot(self):
        with self._lock:
            return {
                "map_points": list(self.map_points),
                "map_stamp": self.map_stamp,
                "position": self.position,
                "yaw": self.yaw,
                "odom_stamp": self.odom_stamp,
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


def start_bridge(cfg):
    """Start the rclpy bridge in a background thread. Returns (bridge, thread)."""
    rclpy.init()
    bridge = RosBridge(cfg)

    def _spin():
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(bridge)
        try:
            executor.spin()
        except Exception as e:  # pragma: no cover
            bridge.get_logger().error(f"spin error: {e}")
        finally:
            executor.shutdown()

    thread = threading.Thread(target=_spin, daemon=True, name="rclpy-spin")
    thread.start()
    return bridge, thread
