#!/usr/bin/env python3
"""Probe Go2 voice/audiohub API ids: send api_id 1001..1012, see which returns a response."""
import json
import time
import rclpy
from rclpy.node import Node
from unitree_api.msg import Request

TOPICS = ["/api/voice/request", "/api/audiohub/request"]
TOPIC_RESP = ["/api/voice/response", "/api/audiohub/response"]

class Probe(Node):
    def __init__(self):
        super().__init__("voice_api_probe")
        self.pubs = [self.create_publisher(Request, t, 10) for t in TOPICS]
        self.resp = {}
        for t in TOPIC_RESP:
            self.create_subscription(Request, t, lambda m, topic=t: self._resp(m, topic), 10)

    def _resp(self, msg, topic):
        self.resp.setdefault(topic, set()).add(msg.header.identity.api_id)
        self.get_logger().info(f"RESPONSE on {topic} api_id={msg.header.identity.api_id} data={msg.data}")

    def run(self):
        for idx, topic in enumerate(TOPICS):
            for api_id in list(range(1001, 1013)) + [0, 1]:
                req = Request()
                req.header.identity.api_id = api_id
                req.parameter = "{}"
                self.pubs[idx].publish(req)
                time.sleep(0.2)
        for _ in range(30):
            rclpy.spin_once(self, timeout_sec=0.3)
        self.get_logger().info(f"collected responses: {self.resp}")

def main():
    rclpy.init()
    probe = Probe()
    try:
        probe.run()
    except KeyboardInterrupt:
        pass
    finally:
        probe.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
