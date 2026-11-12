#!/usr/bin/env bash
# FAST-LIO NUC 벤치마크 — PID 직접 추적 (pkill -f 금지: 호출자 자신을 죽인다)
CFG="$1"; TAG="$2"; SEC="${3:-70}"
BAG=~/ws/lab/bags/live_0910_1512
OUT=~/ws/archbench/out
source ~/ws/lab/env.sh >/dev/null 2>&1
source ~/ws/lab/install/setup.bash >/dev/null 2>&1
[ -n "${OVERLAY:-}" ] && source "$OVERLAY" >/dev/null 2>&1

PIDS=""
# ros2 run 은 래퍼라 $! 만 죽이면 실제 노드가 고아로 살아남는다. 실행파일 이름으로도 확실히 정리한다.
stop(){ [ -n "${PLAYPG:-}" ] && kill -TERM -- -"$PLAYPG" 2>/dev/null
        for p in $PIDS; do kill "$p" 2>/dev/null; done
        for n in fastlio_mapping cloud_bridge_no imu_bridge_node; do pkill -x "$n" 2>/dev/null; done
        pkill -x ros2 2>/dev/null
        timeout 8 bash -c 'while pgrep -x fastlio_mapping >/dev/null; do :; done' 2>/dev/null
        for n in fastlio_mapping cloud_bridge_no imu_bridge_node; do pkill -9 -x "$n" 2>/dev/null; done
        PIDS=""; }
stop
trap stop EXIT INT TERM

echo "=== $TAG : $(basename "$CFG") : $(ros2 pkg prefix fast_lio) ==="
ros2 run go2_bridge imu_bridge_node --ros-args -r __node:=imu_bridge \
  -p input_topic:=/lowstate -p output_topic:=/imu/body -p frame_id:=imu_link \
  -p publish_static_tf:=true -p publish_stab:=true > "$OUT/$TAG.imu.log" 2>&1 & PIDS="$PIDS $!"
ros2 run go2_bridge cloud_bridge_node --ros-args -r __node:=cloud_bridge \
  -p input_topic:=/lidar_points -p output_topic:=/velodyne_points -p frame_id:=hesai_lidar \
  -p stride:=1 > "$OUT/$TAG.cloud.log" 2>&1 & PIDS="$PIDS $!"
ros2 run fast_lio fastlio_mapping --ros-args -r __node:=fastlio_odometry \
  --params-file "$CFG" > "$OUT/$TAG.lio.log" 2>&1 & PIDS="$PIDS $!"
sleep 5
LIO=$(pgrep -x fastlio_mapping | head -1)
[ -n "$LIO" ] || { echo "  ★ fastlio 기동 실패"; tail -3 "$OUT/$TAG.lio.log"; exit 1; }
echo "  fastlio PID=$LIO"

setsid ros2 bag play "$BAG" --topics /lidar_points /lowstate > "$OUT/$TAG.play.log" 2>&1 & PLAYPG=$!
PIDS="$PIDS $PLAYPG"
sleep 12    # IMU 초기화 + kdtree 구축

# 발행자가 정확히 1개인지 확인 — 둘이면 측정이 무의미하다
NPUB=$(ros2 topic info /Odometry 2>/dev/null | grep -oE 'Publisher count: [0-9]+' | grep -oE '[0-9]+')
NPROC=$(pgrep -x fastlio_mapping | wc -l)
echo "  /Odometry 발행자=$NPUB  fastlio 프로세스=$NPROC"
if [ "${NPUB:-0}" != "1" ] || [ "$NPROC" != "1" ]; then
  echo "  ★ 발행자/프로세스가 1이 아니다 — 측정 중단"; exit 2
fi

# 스캔에 실제로 점이 들어오는지 — 빈 스캔이면 CPU 수치가 무의미하다
PTS=$(timeout 6 python3 -c "
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy
rclpy.init(); n=Node('ptcheck'); got=[]
def cb(m): got.append(m.width*m.height)
n.create_subscription(PointCloud2,'/velodyne_points',cb,QoSProfile(depth=5,reliability=ReliabilityPolicy.RELIABLE))
import time
t=time.time()
while time.time()-t<4 and len(got)<8: rclpy.spin_once(n,timeout_sec=0.3)
print(int(sum(got)/len(got)) if got else 0)
rclpy.shutdown()
" 2>/dev/null | tail -1)
echo "  /velodyne_points 평균 점 수: ${PTS:-측정실패}"

TICK=$(getconf CLK_TCK)
c0=$(awk '{print $14+$15}' /proc/$LIO/stat 2>/dev/null); t0=$(date +%s.%N)
python3 ~/ws/archbench/lat_probe.py $((SEC-5)) "$OUT/$TAG.lat.txt" &
timeout "$SEC" ros2 topic hz /Odometry --window 200 > "$OUT/$TAG.hz.txt" 2>&1
c1=$(awk '{print $14+$15}' /proc/$LIO/stat 2>/dev/null); t1=$(date +%s.%N)
rss=$(awk '/VmRSS/{print $2}' /proc/$LIO/status 2>/dev/null)
alive=$(kill -0 "$LIO" 2>/dev/null && echo yes || echo "★죽음")

if [ -n "$c0" ] && [ -n "$c1" ]; then
python3 -c "
cpu=($c1-$c0)/$TICK; wall=$t1-$t0
print(f'  CPU(1코어기준): {100*cpu/wall:6.1f}%   |  16코어 대비 {100*cpu/wall/16:4.2f}%')
print(f'  CPU시간/실시간: {cpu:.1f}초 / {wall:.1f}초')
print(f'  메모리        : ${rss:-0} KB')
print(f'  프로세스      : $alive')
"
else echo "  CPU 측정 실패 (프로세스 $alive)"; fi
grep -oE 'average rate: [0-9.]+' "$OUT/$TAG.hz.txt" | tail -1 | sed 's/^/  Odometry주기  : /'
grep -oE 'min: [0-9.]+s max: [0-9.]+s std dev: [0-9.]+s' "$OUT/$TAG.hz.txt" | tail -1 | sed 's/^/  주기분포      : /'
stop
sleep 1; cat "$OUT/$TAG.lat.txt" 2>/dev/null
