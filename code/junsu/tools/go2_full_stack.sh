#!/usr/bin/env bash
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
#   5) cone_raw_filter_node(Rgosu, 2026-11-04 밤 고침·11-05 밤 연결 — '작전명 ego') —
#      wall_filter 출력에 L1(/utlidar/cloud_base) 근거리 밴드를 합쳐 /mission/obstacle_cloud로
#      냄. /mission/cone_zone_active=True일 때만 보강(real_course_runner.py가 rabain/rabaout
#      도착 시 켜고/끔) — 밖에서는 wall_cloud 그대로 통과(기존 동작 안 바뀜). XT16(위쪽 라이다,
#      "HEXAI")은 가까운 낮은 콘을 못 봐서(바닥 위 0.57m, 가장 아래 링도 1.28m 밖에서야 23cm
#      콘 꼭대기에 닿음) 기본 꺼둠 — 콘 회피는 L1이 맞다.
#   6) ego_planner(Rgosu 패치판) — 장애물 입력을 /mission/obstacle_cloud로 받고(min/max_obs_h는
#      넓게 열어서 필터 쪽 판단만 믿음), inflate_radius 0.30m / safe_dist 0.30m로.
#      ★ 5cm까지만 허용(로봇 몸에만 안 닿으면 된다)로 더 좁히고 싶을 수 있는데 하지 말 것 —
#      grid_map.cpp가 로봇 중심 25cm 안 점을 버려서 0.25m 밑으로 내리면 오히려 장애물이
#      안 보이는 사각지대가 생겨 더 위험해진다(시뮬레이션 실측: 0.25→접촉 247초·5/7 도착,
#      0.30→접촉 54초·7/7 완주). 0.30m(로봇 반폭 0.155+여유 14.5cm)가 지금까지 검증된 값.
#   7) cmd_vel_to_sport_node — 여기서부터 로봇이 실제로 움직임
#   8) realsense2_camera(color만, depth/infra/imu는 끔 — USB 대역폭/CPU 절약, 팀
#      record_all.sh의 실측 설정에서 우리가 쓰는 색상 스트림만 가져옴)
#   9) traffic_light_detector_node(YOLO, venv_yolo 파이썬) — camera_active가 True일
#      때만 추론(기본 False, real_course_runner.py가 정지선 도착 시 켜줌). ★ 2026-11-04:
#      이 두 단계(8,9)가 없으면 real_course_runner.py가 신호를 영원히 못 받아서
#      정지선에서 무한 대기한다(단, 11-05부터 10초 타임아웃으로 신호 무관 출발하긴 함
#      — real_course_runner.py 참고) — 그래도 반드시 같이 띄울 것.
#
# 사용 전 꼭 확인: `source ~/setup_go2_ros2.sh` 출력에서 로봇 인터페이스가
# 192.168.123.x로 탐지됐는지(유선 연결). 안 되면 랜선부터 확인.
#
# 비상정지: pkill -INT -f cmd_vel_to_sport_node   (SIGINT여야 StopMove가 나감, -9 금지)
set +u
source ~/setup_go2_ros2.sh
export CYCLONEDDS_URI=file://$HOME/ws/junsu/config/cyclonedds_junsu.xml   # ★ 11-05: 데이터 유니캐스트(멀티캐스트면 점구름이 robot-br 100 Mb/s 포트로 넘쳐 XT16 7.6 Hz·로봇 링크 포화)
source ~/ws/merge_tmp_20261102/review/install/setup.bash >/dev/null 2>&1
source ~/ws/v_gosu/ws_legfuse/install/setup.bash >/dev/null 2>&1
source ~/ws/junsu/ws_fastlio_tunable/install/setup.bash >/dev/null 2>&1
source ~/ws/junsu/ws_ego_planner_tunable/install/setup.bash >/dev/null 2>&1
source ~/ws/junsu/ws_wall_filter/install/setup.bash >/dev/null 2>&1
set +u

LOGDIR=~/ws/junsu/maps/segments/logs
mkdir -p "$LOGDIR"

pkill -9 -f "hesai_ros_driver_node|cloud_bridge_node|imu_bridge_node|fastlio_mapping|wall_filter_node|cone_raw_filter_node|ego_planner_node|cmd_vel_to_sport_node|realsense2_camera|traffic_light_detector_node|obstacle_fusion_node|cone_raw_filter_node" 2>/dev/null
sleep 1

echo "[1/9] XT16 라이다 드라이버"
setsid ros2 run hesai_ros_driver hesai_ros_driver_node --ros-args \
  -p config_path:=/home/ilovehan9283/ws/merge_tmp_20261102/review/src/external/hesai_ros_driver/config/config.yaml \
  </dev/null >"$LOGDIR/live_hesai.log" 2>&1 &
sleep 3

echo "[2/9] go2_bridge (cloud/imu) — input_topic 오버라이드 하지 말 것!"
# ★ 11-05: review go2_bridge 는 lifecycle 노드 — 설정 파일 + configure/activate 필요(안 하면 FAST-LIO 입력 없음). go2_bridge_lc.sh 참고
source ~/ws/junsu/tools/go2_bridge_lc.sh
setsid ros2 run go2_bridge cloud_bridge_node --ros-args --params-file $BRIDGE_PARAMS </dev/null >"$LOGDIR/live_cloud.log" 2>&1 &
setsid ros2 run go2_bridge imu_bridge_node --ros-args --params-file $BRIDGE_PARAMS </dev/null >"$LOGDIR/live_imu.log" 2>&1 &
sleep 2
lc_up go2_cloud_bridge; lc_up go2_imu_bridge

echo "[3/9] FAST-LIO(Rgosu 패치판) — 로봇을 가만히 두고 몇 초 기다릴 것"
setsid ros2 run fast_lio fastlio_mapping --ros-args \
  --params-file ~/ws/junsu/config/fastlio_leg_nurigwan_derange30.yaml \
  </dev/null >"$LOGDIR/live_fastlio.log" 2>&1 &
sleep 3

echo "[4/9] wall_filter_node(Rgosu 신규) — 벽만 기하학적으로 골라서 /mission/wall_cloud"
setsid ros2 run wall_filter wall_filter_node </dev/null >"$LOGDIR/live_wallfilter.log" 2>&1 &
sleep 2

# ★ 11-05 밤('작전명 ego'): cone_raw_filter_node 연결 — wall_cloud에 콘 구간(cone_zone_active)
# 에서만 L1 근거리 밴드를 보강해서 /mission/obstacle_cloud로 냄. 콘 구간 밖에선 wall_cloud
# 그대로 통과하므로 기존 동작은 안 바뀜(무해). TODO: 실제 콘으로 clearance 값(min/max_obs_h_band)
# 재보정 필요(지금은 17:56 콘 녹화 기반 추정치) — 내일 동료들과 현장에서 확인할 것.
# ★ 11-05 새벽: 콘 감지는 항상 켠다(사용자) — cone_raw_filter_node(콘 구간만·L1만) 대신 obstacle_fusion_node:
#   벽(wall_cloud) + L1 낮은 장애물(경사로 마스크 밖, 3장 이상, 벽 20 cm 밖) + 내부 PC 카메라 콘(/mission/cone_cam: YOLO+뎁스·주황+뎁스)
#   → 다리 기구학 odom(/utlidar/robot_odom) 기준으로 2초 기억 → /mission/obstacle_cloud(토픽 같음 — ego·러너 그대로).
echo "[5/9] obstacle_fusion_node(junsu) — 벽 + 콘(카메라 YOLO+뎁스 · L1 은 FUSE_L1=true 일 때만, 다리 odom 기억) -> /mission/obstacle_cloud"
setsid python3 ~/ws/junsu/src/obstacle_fusion_node.py --ros-args -p use_l1:=${FUSE_L1:-false} -p use_cam:=true </dev/null >"$LOGDIR/live_obstaclefusion.log" 2>&1 &
# setsid python3 ~/ws/junsu/src/cone_raw_filter_node.py </dev/null >"$LOGDIR/live_conefilter.log" 2>&1 &
sleep 2

# ★ 11-04 밤: 벽 여유 0.25→0.30. ego_planner 는 로봇 중심 25 cm 안 점을 버려서(grid_map.cpp) 여유 0.25 면 모퉁이에서 벽이 사라진다.
#   시뮬레이션(sim/run_sim.sh, 목표 보정 켬): 0.25 접촉 247초·5/7 지점 / 0.30 접촉 54초·7/7 완주 / 0.35 는 X자 통로에서 막힘. 되돌리기: .bak_20261104_inflate025
# ★ 11-05 밤: cloud_topic을 wall_cloud -> obstacle_cloud로(콘 보강 포함) — 되돌리기: .bak_20261105_conelink
echo "[6/9] ego_planner(Rgosu 패치판) — obstacle_cloud(벽+콘) 입력, 여유 0.30 m"
setsid ros2 run ego_planner ego_planner_node --ros-args -p odom_topic:=/Odometry \
  -p cloud_topic:=/mission/obstacle_cloud -p min_obs_h:=-5.0 -p max_obs_h:=5.0 \
  -p inflate_radius:=0.30 -p safe_dist:=0.30 -p min_range_xy:=0.12 -p grid_res:=0.05 \
  </dev/null >"$LOGDIR/live_ego.log" 2>&1 &
sleep 2

echo "[7/9] cmd_vel -> 실제 로봇 모터 — ★여기부터 로봇이 움직입니다★"
setsid ros2 run go2_bridge cmd_vel_to_sport_node --ros-args --params-file $BRIDGE_PARAMS $( [ "${YAW_TRIM:-on}" = on ] || echo "-p yaw_trim_per_vx:=0.0 -p yaw_trim_bias:=0.0" ) </dev/null >"$LOGDIR/live_cmdvel.log" 2>&1 &
sleep 2
lc_up go2_cmd_vel_to_sport   # ★ 활성화해야 모터 명령이 나간다. 직진 보정 0.110/-0.002 기본 켬(YAW_TRIM=off 면 0)

# ★ 2026-11-04 저녁: D435i 는 내부 PC(Jetson) USB 에 꽂혀 있다 → 7·8단계 기본을 내부 PC 로 옮김.
#   VISION=onboard(기본): 내부 PC 에서 카메라(컬러만) + TensorRT INT8 신호등 노드(onboard_vision/traffic_light_trt_node.py).
#     토픽·형식이 NUC 노드와 같아서 real_course_runner.py 는 그대로. 원본 영상은 랜선을 안 탄다.
#   VISION=nuc: 예전처럼 NUC 에서 카메라 + ultralytics 노드(카메라가 NUC 에 꽂혀 있을 때만).
#   ★ 둘을 같이 켜지 말 것 — /mission/traffic_light_state 발행자가 둘이 되어 서로 덮어쓴다.
VISION=${VISION:-onboard}
if [ "$VISION" = onboard ]; then
  echo "[8·9/9] 내부 PC: 카메라 + 신호등(TensorRT INT8) — ~/junsu_vision/jetson_vision.sh start"
  ssh -o ControlPath=~/.ssh/go2_cm_%r@%h:%p -o ControlMaster=auto -o ControlPersist=3h -o ConnectTimeout=6 \
    unitree@192.168.123.18 'bash ~/junsu_vision/jetson_vision.sh start' > "$LOGDIR/live_trafficlight.log" 2>&1; rc=$?
  cat "$LOGDIR/live_trafficlight.log"
  [ $rc = 0 ] || echo "★ 내부 PC 신호등 노드 실패(rc=$rc) — 정지선에서 10초 타임아웃 뒤 신호 무관 출발(real_course_runner.py). ~/ws/junsu/onboard_vision/deploy_onboard_vision.sh status 로 확인"
else
echo "[8/9] realsense2_camera (color만 — depth/infra/imu는 USB 대역폭 절약 위해 끔)"
setsid ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=false \
  enable_infra1:=false enable_infra2:=false enable_gyro:=false enable_accel:=false \
  rgb_camera.color_profile:=848x480x30 \
  </dev/null >"$LOGDIR/live_camera.log" 2>&1 &
sleep 4

echo "[9/9] traffic_light_detector_node(YOLO, venv_yolo) — camera_active=False로 대기 시작"
setsid ~/ws/Rgosu/venv_yolo/bin/python3 ~/ws/junsu/src/traffic_light_detector_node.py \
  </dev/null >"$LOGDIR/live_trafficlight.log" 2>&1 &
fi

sleep 1
echo
echo "=== 기동 완료(9단계 전부). 로그: $LOGDIR/live_*.log ==="
echo "확인: tail -f $LOGDIR/live_fastlio.log   (kdtree init 이후 몇 초간 가만히 둘 것)"
echo "확인: ros2 topic hz /lidar_points   (XT16 20Hz 나오는지)"
echo "확인: tail -f $LOGDIR/live_conefilter.log   (콘 보강 노드 떴는지)"
echo "확인: tail -f $LOGDIR/live_trafficlight.log   (onboard: 'TensorRT 엔진 로드 완료' / nuc: 'YOLO 모델 로드 완료')"
echo "확인: ros2 topic hz /camera/color/image_raw   (카메라 나오는지)"
echo
echo "다음 단계: waypoint_logger.py로 이번 경기장 실측 좌표 찍기 ->"
echo "  cd ~/ws/junsu/tools && python3 waypoint_logger.py ~/ws/junsu/maps/real_waypoints_venue.json"
echo "그 다음(★ 11-05 '작전명 ego' — route_map.json이 제일 최신/정확, line 경유점+ㄷ자 주차출구 다리까지 포함):"
echo "  python3 ~/ws/junsu/src/real_course_runner.py ~/ws/junsu/maps/route_map.json"
echo "  (route_map.json이 아직 이번 경기장 좌표로 새로 안 찍혔으면 -> tools/route_editor.py로 새로 찍거나,"
echo "   예전처럼 raw 라벨 대응만 쓰려면 real_waypoints_venue_run.json도 됨 — real_course_runner.py 안 docstring 참고)"
echo
echo "비상정지: pkill -INT -f cmd_vel_to_sport_node"
