#!/usr/bin/env bash
# 대회 주행 스택(통째 SLAM: FAST-LIO + ego_planner, ~/ws/gosu_slam) 을 녹화본으로 무모터 재생 검증한다.
# 사용: slam_replay_check.sh <bag> <태그> "x1,y1 x2,y2 ..."   (목표는 camera_init = 출발 자세 기준, 60초마다 다음 목표)
# 도메인 79(기본), lo 전용, motor:=false. 로봇이 연결돼 있으면 거부한다.
BAG="${1:?bag}"; TAG="${2:?태그}"; GOALS="${3:-2,2}"
OUT=~/ws/v_gosu/eval/out; mkdir -p "$OUT"
if ip link show robot-br 2>/dev/null | grep -q 'state UP'; then echo "★ robot-br 가 UP — 로봇 연결 중에는 재생하지 않는다"; exit 1; fi
export V_DOMAIN=${V_DOMAIN:-79}; source ~/ws/v_gosu/env_replay.sh
case "$ROS_DOMAIN_ID" in 78|79|80|81) ;; *) echo "★ 도메인 $ROS_DOMAIN_ID — v_gosu 재생은 78~81 만"; exit 1;; esac

PGS=()
stop(){ for p in "${PGS[@]}"; do kill -INT -- -"$p" 2>/dev/null; done; sleep 5
        for p in "${PGS[@]}"; do kill -TERM -- -"$p" 2>/dev/null; done; }
trap stop EXIT
echo "=== $TAG 통째 SLAM 무모터 재생  도메인 $ROS_DOMAIN_ID  목표 [$GOALS]  ($(date +%T)) ==="
setsid ros2 launch ~/ws/gosu_slam/launch/full_nav_slam.launch.py lidar:=false motor:=false foxglove:=false \
  >"$OUT/$TAG.slam.launch.log" 2>&1 & PGS+=($!)
sleep 8
rm -rf "$OUT/$TAG.slam_rec"
setsid ros2 bag record -s mcap -o "$OUT/$TAG.slam_rec" /Odometry /Odometry_filtered /goal_pose /ego_planner/trajectory /cmd_vel \
  >"$OUT/$TAG.slam.rec.log" 2>&1 & RPG=$!; PGS+=($RPG)
# 목표 발행기: 첫 목표는 15초 뒤, 이후 60초마다 다음 목표 (camera_init)
setsid python3 - "$GOALS" >"$OUT/$TAG.slam.goal.log" 2>&1 <<'PY' & PGS+=($!)
import sys, time, rclpy
from geometry_msgs.msg import PoseStamped
G = [tuple(map(float, g.split(','))) for g in sys.argv[1].split()]
rclpy.init(); n = rclpy.create_node('v_gosu_goal_feeder'); pub = n.create_publisher(PoseStamped, '/goal_pose', 10)
time.sleep(15); t0 = time.time(); i = -1
while rclpy.ok():
    k = int((time.time() - t0) // 60) % len(G)
    if k != i:
        i = k; m = PoseStamped(); m.header.frame_id = 'camera_init'; m.header.stamp = n.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y = G[i]; m.pose.orientation.w = 1.0
        for _ in range(3): pub.publish(m); time.sleep(0.2)
        print(f'{time.time() - t0:6.1f}s 목표 {G[i]}', flush=True)
    time.sleep(0.5)
PY
ros2 bag play "$BAG" --topics /lidar_points /lowstate /utlidar/robot_odom /sportmodestate >"$OUT/$TAG.slam.play.log" 2>&1
sleep 3; kill -INT -- -"$RPG" 2>/dev/null; sleep 4
echo "재생 끝 ($(date +%T)). 기록: $OUT/$TAG.slam_rec"
