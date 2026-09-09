import sys, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
rclpy.init()
node = rclpy.node.Node("ai_cmd_pub")
pub = node.create_publisher(String, "asr", 10)
deadline = time.time() + 8
while time.time() < deadline and pub.get_subscription_count() == 0:
    time.sleep(0.2)
msg = String()
msg.data = sys.argv[1] if len(sys.argv) > 1 else "test"
pub.publish(msg)
time.sleep(0.5)
print("PUBLISHED-ONCE:", msg.data)
