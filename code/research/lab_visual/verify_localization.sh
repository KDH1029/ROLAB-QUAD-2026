#!/usr/bin/env bash
# 3D 정합 위치추정을 녹화본으로 검증한다 — 로봇도 젯슨도 쓰지 않는다.
#
#   ./verify_localization.sh data4 [측정초]
#   MAP=go2_prior_map_clean.pcd  POSE="x=0 y=0 yaw=0"  ./verify_localization.sh data4 120
#
# 흐름
#   ① bag 재생 (/lidar_points, /lowstate 만)
#   ② imu_bridge + cloud_bridge + fastlio_mapping  ->  /imu/body, /velodyne_points, /Odometry
#   ③ odom_to_tf                                   ->  TF odom -> base_link
#   ④ icp_localization                             ->  TF map -> odom, localization/pose
#   ⑤ /initialpose 발행 후 localization/status 를 모아 요약
#
# 격리: ~/ws/lab/env.sh (도메인 77 + 루프백 전용). 라이브 스택과 절대 섞이지 않는다.
BAG="${1:-data4}"; SEC="${2:-120}"
MAP="${MAP:-go2_prior_map_clean.pcd}"
POSE="${POSE:-x=0 y=0 yaw=0}"
POSE_AT="${POSE_AT:-8}"        # bag 몇 초에 초기자세를 찍을지 (FAST-LIO 가 자리잡은 뒤)
HZ="${HZ:-5}"; PTS="${PTS:-800}"; WIDE="${WIDE:-1}"; TAG="${TAG:-$BAG}"
RECORD="${RECORD:-0}"   # 1 이면 /Odometry 와 /velodyne_points 를 파생 녹화본으로 남긴다 (후보 비교용)
L=~/ws/lab_visual/logs; O=~/ws/lab_visual/out; mkdir -p "$L" "$O"
BAGDIR=~/ws/lab/bags/$BAG
[ -d "$BAGDIR" ] || { echo "★ 녹화본이 없다: $BAGDIR"; exit 1; }
MAPP=~/ws/lab_visual/maps/$MAP
[ -f "$MAPP" ] || { echo "★ 지도가 없다: $MAPP"; exit 1; }

source ~/ws/lab/env.sh
X=$(echo "$POSE"|grep -oE "x=[-+0-9.]+"|cut -d= -f2)
Y=$(echo "$POSE"|grep -oE "y=[-+0-9.]+"|cut -d= -f2)
YAW=$(echo "$POSE"|grep -oE "yaw=[-+0-9.]+"|cut -d= -f2)
ODOM_CFG=~/ws/lab/go2_ws/config/odometry.yaml

down(){ ~/ws/lab/kill_all.sh --nodes-only >/dev/null 2>&1; pkill -f "icp_localization.p[y]" 2>/dev/null; sleep 1; }
trap 'echo; echo "  중단"; down; exit 130' INT TERM

echo "══ 3D 정합 검증  녹화본 $BAG  지도 $MAP  초기자세 $POSE  ${SEC}초 ══"
down

# ② 오도메트리 (팀원 브리지 + FAST-LIO)
ros2 run go2_bridge imu_bridge_node --ros-args -r __node:=imu_bridge \
    -p input_topic:=/lowstate -p output_topic:=/imu/body -p frame_id:=imu_link \
    -p publish_static_tf:=true -p publish_stab:=true > "$L/imu_$BAG.log" 2>&1 &
ros2 run go2_bridge cloud_bridge_node --ros-args -r __node:=cloud_bridge \
    -p input_topic:=/lidar_points -p output_topic:=/velodyne_points -p frame_id:=hesai_lidar \
    -p stride:=1 > "$L/cloud_$BAG.log" 2>&1 &
ros2 run fast_lio fastlio_mapping --ros-args -r __node:=fastlio_odometry \
    --params-file "$ODOM_CFG" > "$L/lio_$BAG.log" 2>&1 &
# ③ odom -> base_link
ros2 run go2_amcl_layer odom_to_tf --ros-args -r __node:=odom_to_tf \
    -p odom_topic:=/Odometry -p parent_frame:=odom -p child_frame:=base_link > "$L/odomtf_$BAG.log" 2>&1 &
# ④ 위치추정
python3 ~/ws/lab_visual/src/icp_localization.py --map "$MAPP" --hz "$HZ" --scan-pts "$PTS" \
    $([ "$WIDE" = 1 ] && echo --wide) > "$L/loc_$BAG.log" 2>&1 &
sleep 6
( while sleep 5; do top -bn1 2>/dev/null | grep -F 'icp_localizatio' | awk '{print $9}' | head -1; echo "--"; done > "$L/cpu_$TAG.txt" ) &
CPUPID=$!

# 파생 녹화 — 오도메트리가 붙은 녹화본을 만들어 두면 후보 자세를 오프라인으로 빨리 비교할 수 있다
if [ "$RECORD" = 1 ]; then
    DERIV=~/ws/lab/bags/${BAG}_odom
    rm -rf "$DERIV"
    ros2 bag record -o "$DERIV" --max-cache-size 268435456 /Odometry /velodyne_points > "$L/rec_$TAG.log" 2>&1 &
    RECPID=$!
fi

# ① 재생 시작 — 이 순간이 bag 0초
T_BAG=$(date +%s.%N)
ros2 bag play "$BAGDIR" --topics /lidar_points /lowstate > "$L/play_$BAG.log" 2>&1 &
ok=0
for i in $(seq 20); do sleep 1; timeout 2 ros2 topic echo /Odometry --once >/dev/null 2>&1 && { ok=1; break; }; done
[ "$ok" = 1 ] || { echo "  ★ /Odometry 가 안 온다 (20초). FAST-LIO 로그:"; grep -iE "error|fail" "$L/lio_$BAG.log" | tail -5 | cut -c1-140; down; exit 1; }
echo "  오도메트리 시작 (bag $(python3 -c "import time;print(round(time.time()-$T_BAG,1))")초)"

# ⑤ 초기자세
python3 - "$X" "$Y" "$YAW" "$T_BAG" "$POSE_AT" <<'PY'
import sys, math, time, rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
x, y, yd, tb, at = map(float, sys.argv[1:6])
while time.time() - tb < at:
    time.sleep(0.05)
rclpy.init(); n = Node('initial_pose_publisher')
p = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
m = PoseWithCovarianceStamped(); m.header.frame_id = 'map'
m.pose.pose.position.x = x; m.pose.pose.position.y = y
m.pose.pose.orientation.z = math.sin(math.radians(yd) / 2)
m.pose.pose.orientation.w = math.cos(math.radians(yd) / 2)
m.pose.covariance[0] = m.pose.covariance[7] = 0.05; m.pose.covariance[35] = 0.02
for _ in range(3):
    m.header.stamp = n.get_clock().now().to_msg(); p.publish(m)
    rclpy.spin_once(n, timeout_sec=0.05); time.sleep(0.2)
print('  초기자세 발행 (bag %.1f초)' % (time.time() - tb)); rclpy.shutdown()
PY

# 기록
timeout $((SEC + 30)) python3 ~/ws/lab_visual/tools/record_localization.py --sec "$SEC" --out "$O/$TAG.csv"
kill $CPUPID 2>/dev/null
[ "$RECORD" = 1 ] && { kill -INT $RECPID 2>/dev/null; sleep 6; echo "  파생 녹화본: ~/ws/lab/bags/${BAG}_odom ($(du -sh ~/ws/lab/bags/${BAG}_odom 2>/dev/null | cut -f1))"; }
down
python3 ~/ws/lab_visual/tools/record_localization.py --summary "$O/$TAG.csv"
