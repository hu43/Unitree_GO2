#!/usr/bin/env python3
"""Listen to Go2 /api/voice/request to capture the real request format.
Trigger by saying the wake word to the robot (e.g. "宇树" / "Hello Unitree")."""
import rclpy
from rclpy.node import Node
from unitree_api.msg import Request


def main():
    rclpy.init()
    n = Node("voice_listen")
    seen = set()

    def cb(msg):
        key = (msg.header.identity.api_id, msg.parameter)
        if key in seen:
            return
        seen.add(key)
        print(">>> VOICE REQUEST api_id=", msg.header.identity.api_id,
              "parameter=", repr(msg.parameter), flush=True)

    n.create_subscription(Request, "/api/voice/request", cb, 10)
    print("listening /api/voice/request ... (say the wake word to the robot)", flush=True)
    try:
        exec = rclpy.executors.SingleThreadedExecutor()
        exec.add_node(n)
        while rclpy.ok():
            exec.spin_once(timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
