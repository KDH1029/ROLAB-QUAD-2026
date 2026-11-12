#!/usr/bin/env bash
# ★ 좌표 찍기용 — 전체 스택(go2_full_stack.sh) 중 1~3단계(XT16·브리지·FAST-LIO)만. 모터 노드 없음(조종기와 안 섞임).
#   그다음 다른 터미널에서 waypoint_logger.py. 원본: go2_full_stack.sh (같은 환경·같은 설정)
# 원본 머리말:
# 경기장 도착 당일용 — 전체 스택 한 번에 기동 (2026-11-04 업데이트: 카메라+신호등 추가)
#   1) hesai_ros_driver (실제 XT16, /lidar_points) — 이게 없으면 cloud_bridge가 0Hz
#      이거나, 로봇 내장 L1(/utlidar/cloud)로 잘못 연결되면 몇십초~몇분 안에 반드시
#      발산한다(어제 5번 재현, 원인 확정). 절대 L1로 우회하지 말 것.
#   2) go2_bridge: cloud_bridge_node(기본값 그대로, input_topic 오버라이드 하지 말 것 — 이제
#      /lidar_points에 진짜 발행자가 있음) + imu_bridge_node
#   3) FAST-LIO(Rgosu 패치판: det_range 30, motion_gating, 리그퓨전/코어 위치점프 상한)
#   4) wall_filter_node(Rgosu 신규, 2026-11-03) — /velodyne_points(ring 정보)+/Odometry로
#      수직인 벽만 기하학적으로 골라서 /mission/wall_cloud로 냄. 경사로/계단 디딤면은
#      링이 올라갈수록 거리가 달라져서 안 걸림 — 높이값으로 억지로 나누던 예전 방식보다 정교함.
#   5) ego_planner(Rgosu 패치판) — 장애물 입력을 /mission/wall_cloud로 받고(min/max_obs_h는
#      넓게 열어서 wall_filter의 판단만 믿음), inflate_radius 0.25m(로봇 반폭 0.155+여유
#      10cm) / safe_dist 0.30m로 — 계단이 안 높아서 과하게 멀리 돌 필요 없다는 실측 기준.
#   6) cmd_vel_to_sport_node — 여기서부터 로봇이 실제로 움직임
#   7) realsense2_camera(color만, depth/infra/imu는 끔 — USB 대역폭/CPU 절약, 팀
#      record_all.sh의 실측 설정에서 우리가 쓰는 색상 스트림만 가져옴)
#   8) traffic_light_detector_node(YOLO, venv_yolo 파이썬) — camera_active가 True일
#      때만 추론(기본 False, real_course_runner.py가 정지선 도착 시 켜줌). ★ 2026-11-04:
#      이 두 단계(7,8)가 없으면 real_course_runner.py가 신호를 영원히 못 받아서
#      정지선에서 무한 대기한다 — 반드시 같이 띄울 것.
#
# 사용 전 꼭 확인: `source ~/setup_go2_ros2.sh` 출력에서 로봇 인터페이스가
# 192.168.123.x로 탐지됐는지(유선 연결). 안 되면 랜선부터 확인.
#
# 비상정지: pkill -INT -f cmd_vel_to_sport_node   (SIGINT여야 StopMove가 나감, -9 금지)
set +u
source ~/setup_go2_ros2.sh
source ~/ws/merge_tmp_20261102/review/install/setup.bash >/dev/null 2>&1
source ~/ws/v_gosu/ws_legfuse/install/setup.bash >/dev/null 2>&1
source ~/ws/Rgosu/ws_fastlio_tunable/install/setup.bash >/dev/null 2>&1
source ~/ws/Rgosu/ws_ego_planner_tunable/install/setup.bash >/dev/null 2>&1
source ~/ws/Rgosu/ws_wall_filter/install/setup.bash >/dev/null 2>&1
set +u

LOGDIR=~/ws/Rgosu/maps/segments/logs
mkdir -p "$LOGDIR"

pkill -9 -f "[h]esai_ros_driver_node|[c]loud_bridge_node|[i]mu_bridge_node|[f]astlio_mapping" 2>/dev/null
sleep 1

echo "[1/8] XT16 라이다 드라이버"
setsid ros2 run hesai_ros_driver hesai_ros_driver_node --ros-args \
  -p config_path:=/home/ilovehan9283/ws/merge_tmp_20261102/review/src/external/hesai_ros_driver/config/config.yaml \
  </dev/null >"$LOGDIR/live_hesai.log" 2>&1 &
sleep 3

echo "[2/8] go2_bridge (cloud/imu) — input_topic 오버라이드 하지 말 것!"
# ★ 11-05: review go2_bridge 는 lifecycle 노드 — 설정 파일 + configure/activate 필요(안 하면 FAST-LIO 입력 없음). go2_bridge_lc.sh 참고
source ~/ws/Rgosu/tools/go2_bridge_lc.sh
setsid ros2 run go2_bridge cloud_bridge_node --ros-args --params-file $BRIDGE_PARAMS </dev/null >"$LOGDIR/live_cloud.log" 2>&1 &
setsid ros2 run go2_bridge imu_bridge_node --ros-args --params-file $BRIDGE_PARAMS </dev/null >"$LOGDIR/live_imu.log" 2>&1 &
sleep 2
lc_up go2_cloud_bridge; lc_up go2_imu_bridge

echo "[3/8] FAST-LIO(Rgosu 패치판) — 로봇을 가만히 두고 몇 초 기다릴 것"
setsid ros2 run fast_lio fastlio_mapping --ros-args \
  --params-file ~/ws/Rgosu/config/fastlio_leg_nurigwan_derange30.yaml \
  </dev/null >"$LOGDIR/live_fastlio.log" 2>&1 &
sleep 3

echo
echo "=== 1~3단계 기동 완료 (모터 노드 없음). 로그: $LOGDIR/live_*.log"
echo "확인: tail -f $LOGDIR/live_fastlio.log   — 로봇을 가만히 두고 몇 초"
echo "다음(다른 터미널): source ~/setup_go2_ros2.sh && cd ~/ws/Rgosu/tools && python3 waypoint_logger.py <파일.json>"
echo "끄기: pkill -INT -f '[f]astlio_mapping|[c]loud_bridge_node|[i]mu_bridge_node|[h]esai_ros_driver_node'"
