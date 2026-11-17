#!/usr/bin/env bash
# 실시간 수동 목표지점 주행 테스트 — RViz에서 지도 클릭("2D Goal Pose") -> /goal_pose
# -> ego_planner(패치판) -> /cmd_vel -> cmd_vel_to_sport_node -> 실제 로봇 모터.
# track_sequencer는 안 띄움(자동 상태머신 없이, 순수 수동 목표 재전송만 테스트).
set +u
source ~/setup_go2_ros2.sh
source ~/ws/v_gosu/ws_legfuse/install/setup.bash
source ~/ws/junsu/ws_ego_planner_tunable/install/setup.bash
set +u

LOGDIR=~/ws/junsu/maps/segments/logs
mkdir -p "$LOGDIR"

pkill -9 -f '[c]loud_bridge_node' 2>/dev/null
pkill -9 -f '[i]mu_bridge_node' 2>/dev/null
pkill -9 -f '[f]astlio_mapping' 2>/dev/null
pkill -9 -f '[e]go_planner_node' 2>/dev/null
pkill -9 -f '[c]md_vel_to_sport_node' 2>/dev/null
sleep 1

echo "[1/4] 브릿지 기동 (라이다/IMU) — 로봇 유선 인터페이스 탐지 결과가 위에 떴을 것, 192.168.123.x 아니면 중단!"
setsid ros2 run go2_bridge cloud_bridge_node </dev/null >"$LOGDIR/live_cloud.log" 2>&1 &
setsid ros2 run go2_bridge imu_bridge_node </dev/null >"$LOGDIR/live_imu.log" 2>&1 &
sleep 2

echo "[2/4] FAST-LIO(다리융합판) 기동"
setsid ros2 run fast_lio fastlio_mapping --ros-args --params-file ~/ws/v_gosu/config/fastlio_leg_n10.yaml \
  </dev/null >"$LOGDIR/live_fastlio.log" 2>&1 &
sleep 3

echo "[3/4] ego_planner(패치판) 기동"
setsid ros2 run ego_planner ego_planner_node --ros-args -p odom_topic:=/Odometry \
  </dev/null >"$LOGDIR/live_ego.log" 2>&1 &
sleep 2

echo "[4/4] cmd_vel -> 실제 로봇 모터 브릿지 기동 — ★여기서부터 로봇이 실제로 움직입니다★"
setsid ros2 run go2_bridge cmd_vel_to_sport_node </dev/null >"$LOGDIR/live_cmdvel.log" 2>&1 &

sleep 1
echo
echo "=== 기동 완료. 로그: $LOGDIR/live_*.log ==="
echo "FAST-LIO가 odom을 내기 시작할 때까지 수초~수십초 걸립니다 (tail -f $LOGDIR/live_fastlio.log 로 확인)."
echo
echo "★ 비상정지: 다른 터미널에서 ->  pkill -INT -f cmd_vel_to_sport_node"
echo "  (SIGINT로 죽여야 소멸자가 StopMove(1003)를 보냄. kill -9는 정지명령 없이 바로 죽으니 피할 것)"
echo
echo "전체 종료: pkill -INT -f cmd_vel_to_sport_node; sleep 1; pkill -9 -f 'cloud_bridge_node|imu_bridge_node|fastlio_mapping|ego_planner_node'"
