import math, time, json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import struct

def pc2pts(msg, maxn=4000):
    fields = {f.name: f.offset for f in msg.fields}
    data = bytes(msg.data); n = msg.width*msg.height
    if n == 0: return []
    step = msg.point_step; stride = max(1, n//maxn); out=[]
    for i in range(0, n, stride):
        off = i*step
        if off+8 > len(data): break
        x, y = struct.unpack_from("<ff", data, off+fields["x"])
        if math.isfinite(x) and math.isfinite(y):
            out.append((round(x,3), round(y,3)))
    return out

class N(Node):
    def __init__(self):
        super().__init__("probe")
        self.cloud=None; self.tf=None; self.odom=None
        self.create_subscription(PointCloud2, "/grid_map/occupancy", self.cb_map, 10)
        self.create_subscription(TFMessage, "/tf", self.cb_tf, 10)
        self.create_subscription(Odometry, "/sportmodestate_odom", self.cb_o, 10)
    def cb_map(self, m):
        self.cloud={"frame":m.header.frame_id,"stamp":m.header.stamp.sec+m.header.stamp.nanosec*1e-9,"pts":pc2pts(m)}
    def cb_tf(self, m):
        for t in m.transforms:
            if t.header.frame_id=="map" and t.child_frame_id=="odom":
                q=t.transform.rotation; tr=t.transform.translation
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                self.tf=(round(tr.x,4),round(tr.y,4),round(yaw,4))
    def cb_o(self, m):
        p=m.pose.pose.position; q=m.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.odom=(round(p.x,3),round(p.y,3),round(yaw,3))

rclpy.init(); n=N()
end=time.time()+22; samples=[]
while time.time()<end:
    rclpy.spin_once(n, timeout_sec=0.2)
    samples.append({"t":round(time.time(),2),"tf":n.tf,"odom":n.odom,
                    "cloud_n":len(n.cloud["pts"]) if n.cloud else 0,
                    "cloud_frame":n.cloud["frame"] if n.cloud else None})
# summarize
print("=== TF (map->odom) over time ===")
seen=set()
for s in samples:
    k=s["tf"]
    if k and k not in seen:
        seen.add(k); print(" t=%.0f tf=%s odom=%s cloud_n=%d cloud_frame=%s"%(
            s["t"]-samples[0]["t"], k, s["odom"], s["cloud_n"], s["cloud_frame"]))
if not seen: print(" NO map->odom TF FOUND")
print("total distinct tf values:", len(seen))
print("cloud frames:", set(s["cloud_frame"] for s in samples if s["cloud_frame"]))
print("cloud_n range:", min(s["cloud_n"] for s in samples), max(s["cloud_n"] for s in samples))
