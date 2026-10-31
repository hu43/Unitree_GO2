#!/usr/bin/env python3
"""Republish Go2 /sportmodestate as nav_msgs/Odometry.

Why: the SLAM/导航栈原来用 /utlidar/robot_odom（LiDAR LIO 里程计）作为 odom。
实测长距离运行漂移明显，改用狗本体运动控制器发布的 SportModeState
（足端里程计 + IMU，位置字段 position/imu_state.quaternion）。

本节点不做 TF 广播——odom->base_link 统一由 odom_tf_broadcaster 从
/sportmodestate_odom 转发，避免两个 TF 源打架。
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from unitree_go.msg import SportModeState

IN_TOPIC = "/sportmodestate"
OUT_TOPIC = "/sportmodestate_odom"
# SportModeState 原频约 286Hz，下游（TF 50Hz / SLAM / 导航）用不着这么密；
# 限到 50Hz，省 CPU 与 DDS 带宽。
PUB_HZ = 50.0


class SportOdomBridge(Node):
    def __init__(self):
        super().__init__("sport_odom_bridge")
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub = self.create_publisher(Odometry, OUT_TOPIC, 20)
        self.create_subscription(SportModeState, IN_TOPIC, self._cb, qos)
        self.count = 0
        self.last_pub = 0.0
        self.min_period = 1.0 / PUB_HZ
        self.get_logger().info(f"relaying {IN_TOPIC} -> {OUT_TOPIC} @ {PUB_HZ:.0f}Hz")

    def _cb(self, msg):
        now = time.monotonic()
        if now - self.last_pub < self.min_period:
            return
        self.last_pub = now

        od = Odometry()
        od.header.stamp.sec = msg.stamp.sec
        od.header.stamp.nanosec = msg.stamp.nanosec
        od.header.frame_id = "odom"
        od.child_frame_id = "base_link"

        od.pose.pose.position.x = float(msg.position[0])
        od.pose.pose.position.y = float(msg.position[1])
        od.pose.pose.position.z = float(msg.position[2])

        # Unitree IMUState.quaternion 顺序是 [w, x, y, z]（官方 SDK 约定）
        qw, qx, qy, qz = (float(msg.imu_state.quaternion[i]) for i in range(4))
        # 归一化，避免固件偶尔给出非单位四元数导致 TF 报错
        n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz) or 1.0
        od.pose.pose.orientation.x = qx / n
        od.pose.pose.orientation.y = qy / n
        od.pose.pose.orientation.z = qz / n
        od.pose.pose.orientation.w = qw / n

        od.twist.twist.linear.x = float(msg.velocity[0])
        od.twist.twist.linear.y = float(msg.velocity[1])
        od.twist.twist.linear.z = float(msg.velocity[2])
        od.twist.twist.angular.z = float(msg.yaw_speed)

        self.pub.publish(od)
        self.count += 1
        if self.count % 500 == 1:
            self.get_logger().info(
                f"pos=({msg.position[0]:.2f},{msg.position[1]:.2f}) "
                f"quat(wxyz)=({qw:.2f},{qx:.2f},{qy:.2f},{qz:.2f})")


def main():
    rclpy.init()
    node = SportOdomBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
