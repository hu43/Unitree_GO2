import math, time, numpy as np, struct
from PIL import Image, ImageDraw
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry

ORIGIN=(-12.8,-7.19); RES=0.05
def pc2pts(msg, maxn=20000):
    fields={f.name:f.offset for f in msg.fields}
    data=bytes(msg.data); n=msg.width*msg.height
    if n==0: return []
    step=msg.point_step; stride=max(1,n//maxn); out=[]
    for i in range(0,n,stride):
        off=i*step
        if off+8>len(data): break
        x,y=struct.unpack_from("<ff",data,off+fields["x"])
        if math.isfinite(x) and math.isfinite(y): out.append((x,y))
    return out

class N(Node):
    def __init__(s):
        super().__init__("probe2")
        s.cloud=None; s.tf=None; s.odom=None
        s.create_subscription(PointCloud2,"/grid_map/occupancy",s.cbm,10)
        s.create_subscription(TFMessage,"/tf",s.cbt,10)
        s.create_subscription(Odometry,"/sportmodestate_odom",s.cbo,10)
    def cbm(s,m): s.cloud=(m.header.frame_id, pc2pts(m))
    def cbt(s,m):
        for t in m.transforms:
            if t.header.frame_id=="map" and t.child_frame_id=="odom":
                q=t.transform.rotation; tr=t.transform.translation
                s.tf=(tr.x,tr.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
    def cbo(s,m):
        p=m.pose.pose.position; q=m.pose.pose.orientation
        s.odom=(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))

rclpy.init(); n=N(); t0=time.time()
while time.time()-t0<12: rclpy.spin_once(n,timeout_sec=0.2)
print("cloud_frame:", n.cloud[0], "npts:", len(n.cloud[1]))
print("tf map->odom:", [round(v,4) for v in n.tf])
print("sport odom:", [round(v,4) for v in n.odom])

img=Image.open("/tmp/go2web/map/test.pgm").convert("L"); a=np.array(img); H,W=a.shape
rgb=Image.new("RGB",(W,H),(16,20,32)); d=ImageDraw.Draw(rgb)
occ=a<100
# saved map walls: dim red
ys,xs=np.nonzero(occ)
for x,y in zip(xs,ys): rgb.putpixel((int(x),int(y)),(150,40,40))

# live cloud -> map frame
tf=n.tf; c_,s_=math.cos(tf[2]),math.sin(tf[2])
for (x,y) in n.cloud[1]:
    mx=tf[0]+c_*x-s_*y; my=tf[1]+s_*x+c_*y
    col=int((mx-ORIGIN[0])/RES); row=int(H-(my-ORIGIN[1])/RES)
    if 0<=col<W and 0<=row<H: rgb.putpixel((col,row),(80,255,120))

for name,x,y in [("P1",-1.829,0.254),("P2",-1.617,-1.1),("P3",-2.889,1.105),("P4",-0.889,4.937)]:
    col=(x-ORIGIN[0])/RES; row=H-(y-ORIGIN[1])/RES
    d.ellipse([col-8,row-8,col+8,row+8],outline=(80,170,255),width=3); d.text((col+11,row-6),name,fill=(120,190,255))
rgb.save("/tmp/go2web/map/overlay_live.png")

# quantitative alignment: for each live point, nearest saved wall
oy,ox=np.nonzero(occ)
ox=ox*RES+ORIGIN[0]; oy=(H-oy)*RES+ORIGIN[1]
pts=np.array(n.cloud[1]); mx=tf[0]+c_*pts[:,0]-s_*pts[:,1]; my=tf[1]+s_*pts[:,0]+c_*pts[:,1]
from scipy.spatial import cKDTree
tree=cKDTree(np.column_stack([ox,oy]))
dist,_=tree.query(np.column_stack([mx,my]))
print("live->savedmap nearest-wall dist: median %.3f m, p10 %.3f, p90 %.3f"%(np.median(dist),np.percentile(dist,10),np.percentile(dist,90)))
print("share within 0.10m: %.1f%%  within 0.25m: %.1f%%"%(100*(dist<0.10).mean(),100*(dist<0.25).mean()))
