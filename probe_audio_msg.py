#!/usr/bin/env python3
"""Probe Go2 audio channel: try different /audio_msg payloads and watch player state."""
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TOPIC_AUDIO_MSG = "/audio_msg"
TOPIC_PLAYER_STATE = "/audiohub/player/state"


class AudioProbe(Node):
    def __init__(self):
        super().__init__("audio_probe")
        self.pub = self.create_publisher(String, TOPIC_AUDIO_MSG, 10)
        self.state_sub = self.create_subscription(String, TOPIC_PLAYER_STATE, self.state_cb, 10)
        self.state = None

    def state_cb(self, msg):
        self.state = msg.data

    def send(self, payload):
        m = String()
        m.data = payload
        self.pub.publish(m)
        self.get_logger().info(f"sent /audio_msg = {payload!r}")

    def wait(self, seconds, label):
        self.state = None
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.3)
            if self.state and '"is_playing": true' in self.state:
                self.get_logger().info(f"[{label}] >> PLAYING! state={self.state}")
                return True
        self.get_logger().info(f"[{label}] no play (state={self.state})")
        return False


def main():
    rclpy.init()
    probe = AudioProbe()
    candidates = [
        ("plain text", "你好，机器狗语音测试。"),
        ("json tts", json.dumps({"type": "tts", "text": "你好，机器狗语音测试。"})),
        ("json play", json.dumps({"cmd": "play", "text": "你好，机器狗语音测试。"})),
        ("json speak", json.dumps({"api": "tts", "text": "你好，机器狗语音测试。"})),
    ]
    try:
        for label, payload in candidates:
            probe.send(payload)
            probe.wait(4, label)
    except KeyboardInterrupt:
        pass
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
