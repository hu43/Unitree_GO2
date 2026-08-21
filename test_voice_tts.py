#!/usr/bin/env python3
"""Test Go2 voice service TTS (G1-style AUDIO_TTS=1001)."""
import json
import time
import sys
import rclpy
from rclpy.node import Node
from unitree_api.msg import Request
from std_msgs.msg import String

TOPIC_VOICE_REQ = "/api/voice/request"
TOPIC_VOICE_RESP = "/api/voice/response"
TOPIC_PLAYER_STATE = "/audiohub/player/state"

class VoiceTtsTest(Node):
    def __init__(self, text):
        super().__init__("voice_tts_test")
        self.pub = self.create_publisher(Request, TOPIC_VOICE_REQ, 10)
        self.state_sub = self.create_subscription(String, TOPIC_PLAYER_STATE, self.state_cb, 10)
        self.resp_sub = self.create_subscription(Request, TOPIC_VOICE_RESP, self.resp_cb, 10)
        self.text = text
        self.state = None

    def state_cb(self, msg):
        self.state = msg.data

    def resp_cb(self, msg):
        self.get_logger().info(f"voice response api_id={msg.header.identity.api_id} data={msg.data}")

    def send(self, api_id, param):
        req = Request()
        req.header.identity.api_id = api_id
        req.parameter = json.dumps(param)
        self.pub.publish(req)
        self.get_logger().info(f"voice sent api_id={api_id} param={json.dumps(param, ensure_ascii=False)}")

    def run(self):
        # G1 TTS format: {index, speaker_id, text}
        self.send(1001, {"index": 0, "speaker_id": 0, "text": self.text})
        played = False
        for _ in range(40):
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.state and '"is_playing": true' in self.state:
                self.get_logger().info(">> PLAYING (voice TTS accepted!)")
                played = True
                break
        if not played:
            self.get_logger().info(f">> no play (state={self.state})")

def main():
    text = sys.argv[1] if len(sys.argv) > 1 else "你好，我是宇树机器狗，欢迎参观展厅。"
    rclpy.init()
    node = VoiceTtsTest(text)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
