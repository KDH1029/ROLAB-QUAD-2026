# /velodyne_points 수신 시각과 /Odometry 수신 시각으로 스캔당 처리 지연을 잰다 (raw 구독, 점구름 역직렬화 안 함)
import sys, time, struct, bisect, statistics as st
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
SEC=float(sys.argv[1]); OUT=sys.argv[2]
rclpy.init(); n=Node('lat_probe')
cl=[]  # (stamp, recv)
lat=[]
def stamp(b): s,ns=struct.unpack_from('<iI',b,4); return s+ns*1e-9
def cb_c(b): cl.append((stamp(b),time.monotonic()))
def cb_o(b):
    t=time.monotonic(); s=stamp(b)
    ks=[c[0] for c in cl[-60:]]
    i=bisect.bisect_right(ks,s+1e-6)-1
    if i>=0: lat.append(1000*(t-cl[-60:][i][1]))
q=QoSProfile(depth=50,reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(PointCloud2,'/velodyne_points',cb_c,q,raw=True)
n.create_subscription(Odometry,'/Odometry',cb_o,q,raw=True)
t0=time.monotonic()
while time.monotonic()-t0<SEC: rclpy.spin_once(n,timeout_sec=0.05)
lat=[x for x in lat if 0<=x<1000]
with open(OUT,'w') as f:
    if lat:
        ls=sorted(lat); p=lambda q: ls[min(len(ls)-1,int(q*len(ls)))]
        f.write(f"  지연(스캔→오도) n={len(ls)} 중앙 {st.median(ls):.1f} ms  p90 {p(.9):.1f}  p99 {p(.99):.1f}  최대 {ls[-1]:.1f} ms\n")
    else: f.write("  지연 측정 실패\n")
rclpy.shutdown()
