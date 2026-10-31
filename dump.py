import math, time, json, struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry

def pc2pts(msg, maxn=20000):
    fields={f.name:f.offset for f in msg.fields}
    data=bytes(msg.data); n=msg.width*msg.height
    if n==0: return []
    step=msg.point_step; stride=max(1,n//maxn); out=[]
    for i in range(0,n,stride):
        off=i*step
        if off+8>len(data): break
        x,y=struct.unpack_from("<ff",data,off+fields["x"])
        if math.isfinite(x) and math.isfinite(y): out.append([round(x,3),round(y,3)])
    return out

class N(Node):
    def __init__(s):
        super().__init__("dumpnode")
        s.cloud=None; s.tfr=None; s.odom=None
        s.create_subscription(PointCloud2,"/grid_map/occupancy",s.cbm,10)
        s.create_subscription(TFMessage,"/tf",s.cbt,10)
        s.create_subscription(Odometry,"/sportmodestate_odom",s.cbo,10)
    def cbm(s,m): s.cloud=(m.header.frame_id, pc2pts(m))
    def cbt(s,m):
        for t in m.transforms:
            if t.header.frame_id=="map" and t.child_frame_id=="odom":
                q=t.transform.rotation; tr=t.transform.translation
                s.tfr=[tr.x,tr.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
    def cbo(s,m):
        p=m.pose.pose.position; q=m.pose.pose.orientation
        s.odom=[p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]

rclpy.init(); n=N(); t0=time.time()
while time.time()-t0<12: rclpy.spin_once(n,timeout_sec=0.2)
json.dump({"frame":n.cloud[0],"pts":n.cloud[1],"tf":n.tfr,"odom":n.odom},
          open("/home/unitree/hu/go2/web/dump.json","w"))
print("OK frame=%s n=%d tf=%s odom=%s"%(n.cloud[0],len(n.cloud[1]),n.tfr,n.odom))
