#!/usr/bin/env python3
"""Test Go2 audiohub TTS: send a text-to-speech request and watch player state.

Usage (in ROS2 env with unitree_ros2 sourced):
    python3 test_audiohub_tts.py "你好，这是机器人语音测试"
"""
import json
import sys
import time

import rclpy
from rclpy.node import Node

from unitree_api.msg import Request
from std_msgs.msg import String

# G1-style voice API ids (audiohub may reuse these)
API_TTS = 1001
API_START_PLAY = 1003
API_STOP_PLAY = 1004

TOPIC_REQ = "/api/audiohub/request"
TOPIC_RESP = "/api/audiohub/response"
TOPIC_PLAYER_STATE = "/audiohub/player/state"


class AudioHubTtsTest(Node):
    def __init__(self, text):
        super().__init__("audiohub_tts_test")
        self.pub = self.create_publisher(Request, TOPIC_REQ, 10)
        self.state_sub = self.create_subscription(String, TOPIC_PLAYER_STATE, self.state_cb, 10)
        self.resp_sub = self.create_subscription(Request, TOPIC_RESP, self.resp_cb, 10)
        self.text = text
        self.state = None

    def state_cb(self, msg):
        self.state = msg.data
        self.get_logger().info(f"player state: {msg.data}")

    def resp_cb(self, msg):
        self.get_logger().info(f"audiohub response api_id={msg.header.identity.api_id} data={msg.data}")

    def send(self, api_id, param):
        req = Request()
        req.header.identity.api_id = api_id
        req.parameter = json.dumps(param)
        self.pub.publish(req)
        self.get_logger().info(f"sent api_id={api_id} param={param}")

    def run(self):
        # 1) TTS request
        self.send(API_TTS, {"index": 0, "speaker_id": 0, "text": self.text})
        for _ in range(30):
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.state and '"is_playing": true' in self.state:
                self.get_logger().info(">> player is PLAYING (TTS accepted!)")
                break
        # 2) wait until finished
        for _ in range(60):
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.state and '"is_playing": false' in self.state and '"not_in_use"' in self.state:
                self.get_logger().info(">> player back to idle (playback finished)")
                break


def main():
    text = sys.argv[1] if len(sys.argv) > 1 else "你好，我是宇树机器狗，欢迎参观展厅。"
    rclpy.init()
    node = AudioHubTtsTest(text)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
