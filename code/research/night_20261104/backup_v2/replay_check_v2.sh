#!/usr/bin/env bash
# [night_20261104 사본 — v2 노드] 백업 시스템 무모터 재생 시험 — 재생 도중 FAST-LIO 를 일부러 꺼서 전환·주행 지속을 확인한다.
# 사용: backup_replay_check.sh <bag> <태그> <사전지도.npz|.pcd> [FAST-LIO 끌 시각 s=200] ["x1,y1 x2,y2 ..." 목표]
# 도메인 79, lo 전용, motor:=false. 로봇이 연결돼 있으면 거부한다(09-10 재생 섞임 사고 방지).
BAG="${1:?bag}"; TAG="${2:?태그}"; MAP="${3:?지도}"; KILL_AT="${4:-200}"; GOALS="${5:-4.4,-1.6 6.0,0.2 5.9,-2.9 1.4,-3.8 0,0}"
OUT=~/ws/night_20261104/backup_v2/out; mkdir -p "$OUT"; export ROS_LOG_DIR=~/ws/night_20261104/logs/ros
if ip link show robot-br 2>/dev/null | grep -q 'state UP'; then echo "★ robot-br 가 UP — 로봇 연결 중에는 재생하지 않는다"; exit 1; fi
export V_DOMAIN=${V_DOMAIN:-80}; source ~/ws/v_gosu/env_replay.sh
case "$ROS_DOMAIN_ID" in 78|79|80|81) ;; *) echo "★ 도메인 $ROS_DOMAIN_ID — 재생은 78~81 만"; exit 1;; esac
PGS=()
stop(){ for p in "${PGS[@]}"; do kill -INT -- -"$p" 2>/dev/null; done; sleep 5; for p in "${PGS[@]}"; do kill -TERM -- -"$p" 2>/dev/null; done; }
trap stop EXIT
echo "=== $TAG 백업 재생 시험  도메인 $ROS_DOMAIN_ID  지도 $MAP  FAST-LIO 끔 ${KILL_AT}s  ($(date +%T)) ==="
setsid ros2 launch ~/ws/night_20261104/backup_v2/launch/full_nav_slam_backup_v2.launch.py lidar:=false motor:=false foxglove:=false \
  backup_map:="$MAP" >"$OUT/$TAG.launch.log" 2>&1 & PGS+=($!)
sleep 15     # 지도 적재·노드 기동
rm -rf "$OUT/$TAG.rec"
setsid ros2 bag record -s mcap -o "$OUT/$TAG.rec" /Odometry /Odometry_mux /Odometry_filtered /backup/odometry /backup/status \
  /mux/status /goal_pose /ego_planner/trajectory /cmd_vel >"$OUT/$TAG.rec.log" 2>&1 & RPG=$!; PGS+=($RPG)
setsid python3 - "$GOALS" >"$OUT/$TAG.goal.log" 2>&1 <<'PY' & PGS+=($!)
import sys, time, rclpy
from geometry_msgs.msg import PoseStamped
G = [tuple(map(float, g.split(','))) for g in sys.argv[1].split()]
rclpy.init(); n = rclpy.create_node('night_goal_feeder'); pub = n.create_publisher(PoseStamped, '/goal_pose', 10)
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
# FAST-LIO 를 KILL_AT 초에 끈다 (프로세스 이름 정확히 일치 — pkill -f 자기 자신 잡기 함정 회피)
( sleep "$KILL_AT"; P=$(pgrep -x fastlio_mapping); echo "$(date +%T) FAST-LIO 끔 pid=$P"; [ -n "$P" ] && kill -INT $P ) >"$OUT/$TAG.kill.log" 2>&1 &
ros2 bag play "$BAG" --topics /lidar_points /lowstate /utlidar/robot_odom /sportmodestate >"$OUT/$TAG.play.log" 2>&1
sleep 3; kill -INT -- -"$RPG" 2>/dev/null; sleep 4
echo "재생 끝 ($(date +%T)). 분석: python3 ~/ws/night_20261104/backup_v2/analyze_replay_v2.py $TAG"
