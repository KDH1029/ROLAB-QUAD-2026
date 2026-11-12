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
#   5) obstacle_fusion_node(Rgosu, 2026-11-05 밤 '작전명 ego') — 벽(wall_cloud) + L1(/utlidar/
#      cloud_base, 경사로 마스크로 X자 계단/램프는 제외) + 카메라 콘(/mission/cone_cam, 내부 PC
#      YOLO+뎁스, 주황색+바닥높이로 판정)을 하나로 합쳐 /mission/obstacle_cloud로 냄. ★ 콘 감지는
#      항상 켜져 있음(cone_zone_active 같은 구간 게이팅 없음) — 라바콘 슬라럼뿐 아니라 parking→exit
#      ㄷ자 통로에 놓인 콘도 똑같이 본다(사용자 확인: 그 통로 벽 틈 1.1cm로 보였던 지점이 실은
#      거기 놓인 콘일 수 있음 — wall_filter가 콘의 수직면을 벽으로 오인했을 가능성). 다리 기구학
#      오도메트리로 콘이 카메라 시야 밖으로 나가도 2초간 기억해서 따라옴.
#      ★★ 11-05 새벽에 이전 버전(cone_raw_filter_node, L1만·구간게이팅)을 연결했다가 시뮬레이션
#      에서 stopline1 직후 길막힘을 보고 되돌렸었는데, 나중에 콘 노드 없이(wall_cloud만) 같은
#      구간을 다시 돌려보니 *거기서도 똑같이* 막혀서(실행마다 막히는 위치도 다름) — 그 길막힘은
#      콘 노드 때문이 아니라 ego_planner 자체의 비결정적 타이밍 문제였던 것으로 결론 정정함
#      (README 참고). 그래서 이번엔 안전하게 다시 연결함 — 콘 데이터가 안 와도(카메라 꺼짐 등)
#      wall_cloud를 그대로 통과시키는 구조라 최악의 경우에도 예전과 동일하게 동작함.
#   6) ego_planner(Rgosu 패치판) — 장애물 입력을 /mission/obstacle_cloud(벽+콘)로 받고
#      (min/max_obs_h는 넓게 열어서 앞단 판단만 믿음), inflate_radius 0.30m / safe_dist 0.30m로.
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
#      — real_course_runner.py 참고) — 그래도 반드시 같이 띄울 것. VISION=onboard(기본)일
#      땐 이 단계의 traffic_light_trt_node.py가 /mission/cone_cam도 같이 냄(같은 YOLO
#      모델 tl5cone — 신호등+콘 같이 검출) → obstacle_fusion_node가 그걸 받음.
#
# 사용 전 꼭 확인: `source ~/setup_go2_ros2.sh` 출력에서 로봇 인터페이스가
# 192.168.123.x로 탐지됐는지(유선 연결). 안 되면 랜선부터 확인.
#
# ★ 2026-11-05 밤 — 작전명 ego / noego 스위치: CONE=1(기본, '작전명 ego')이면 obstacle_fusion_node를
# 띄워서 ego_planner가 벽+콘을 같이 보게 함. CONE=0('작전명 noego' — 콘 인식이 현장에서 말썽이면
# 즉시 전환, 콘은 무시하고 밀고 지나감, 대신 콘 충돌 감점 감수)이면 그 노드를 아예 안 띄우고
# ego_planner가 예전처럼 /mission/wall_cloud만 봄. real_course_runner.py는 두 경우 다 동일(안 건드림).
#   작전명 ego:    ~/ws/Rgosu/tools/go2_full_stack.sh
#   작전명 noego:  CONE=0 ~/ws/Rgosu/tools/go2_full_stack.sh
CONE=${CONE:-1}
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

echo "================================================================"
if [ "$CONE" = 1 ]; then echo "  작전명 EGO (콘 인식 켬 — obstacle_fusion_node 연결)"
else echo "  작전명 NOEGO (콘 인식 끔 — 벽만 보고 밀고 지나감)"; fi
echo "================================================================"

pkill -9 -f "hesai_ros_driver_node|cloud_bridge_node|imu_bridge_node|fastlio_mapping|wall_filter_node|obstacle_fusion_node|ego_planner_node|cmd_vel_to_sport_node|realsense2_camera|traffic_light_detector_node" 2>/dev/null
sleep 1

echo "[1/9] XT16 라이다 드라이버"
setsid ros2 run hesai_ros_driver hesai_ros_driver_node --ros-args \
  -p config_path:=/home/ilovehan9283/ws/merge_tmp_20261102/review/src/external/hesai_ros_driver/config/config.yaml \
  </dev/null >"$LOGDIR/live_hesai.log" 2>&1 &
sleep 3

echo "[2/9] go2_bridge (cloud/imu) — input_topic 오버라이드 하지 말 것!"
setsid ros2 run go2_bridge cloud_bridge_node </dev/null >"$LOGDIR/live_cloud.log" 2>&1 &
setsid ros2 run go2_bridge imu_bridge_node </dev/null >"$LOGDIR/live_imu.log" 2>&1 &
sleep 2

echo "[3/9] FAST-LIO(Rgosu 패치판) — 로봇을 가만히 두고 몇 초 기다릴 것"
setsid ros2 run fast_lio fastlio_mapping --ros-args \
  --params-file ~/ws/Rgosu/config/fastlio_leg_nurigwan_derange30.yaml \
  </dev/null >"$LOGDIR/live_fastlio.log" 2>&1 &
sleep 3

echo "[4/9] wall_filter_node(Rgosu 신규) — 벽만 기하학적으로 골라서 /mission/wall_cloud"
setsid ros2 run wall_filter wall_filter_node </dev/null >"$LOGDIR/live_wallfilter.log" 2>&1 &
sleep 2

if [ "$CONE" = 1 ]; then
  echo "[5/9] obstacle_fusion_node(Rgosu) — 벽 + L1 콘 + 카메라 콘 -> /mission/obstacle_cloud (작전명 ego)"
  setsid python3 ~/ws/Rgosu/src/obstacle_fusion_node.py --ros-args -p use_leg_odom:=true \
    </dev/null >"$LOGDIR/live_obstaclefusion.log" 2>&1 &
  sleep 2
  CLOUD_TOPIC=/mission/obstacle_cloud
else
  echo "[5/9] (건너뜀 — CONE=0, 작전명 noego: 콘 인식 없이 벽만 보고 그대로 밀고 지나감)"
  CLOUD_TOPIC=/mission/wall_cloud
fi

# ★ 11-04 밤: 벽 여유 0.25→0.30. ego_planner 는 로봇 중심 25 cm 안 점을 버려서(grid_map.cpp) 여유 0.25 면 모퉁이에서 벽이 사라진다.
#   시뮬레이션(sim/run_sim.sh, 목표 보정 켬): 0.25 접촉 247초·5/7 지점 / 0.30 접촉 54초·7/7 완주 / 0.35 는 X자 통로에서 막힘. 되돌리기: .bak_20261104_inflate025
echo "[6/9] ego_planner(Rgosu 패치판) — $CLOUD_TOPIC 입력, 여유 0.30 m"
setsid ros2 run ego_planner ego_planner_node --ros-args -p odom_topic:=/Odometry \
  -p cloud_topic:=$CLOUD_TOPIC -p min_obs_h:=-5.0 -p max_obs_h:=5.0 \
  -p inflate_radius:=0.30 -p safe_dist:=0.30 \
  </dev/null >"$LOGDIR/live_ego.log" 2>&1 &
sleep 2

echo "[7/9] cmd_vel -> 실제 로봇 모터 — ★여기부터 로봇이 움직입니다★"
setsid ros2 run go2_bridge cmd_vel_to_sport_node </dev/null >"$LOGDIR/live_cmdvel.log" 2>&1 &
sleep 2

# ★ 2026-11-04 저녁: D435i 는 내부 PC(Jetson) USB 에 꽂혀 있다 → 7·8단계 기본을 내부 PC 로 옮김.
#   VISION=onboard(기본): 내부 PC 에서 카메라(컬러만) + TensorRT INT8 신호등 노드(onboard_vision/traffic_light_trt_node.py).
#     토픽·형식이 NUC 노드와 같아서 real_course_runner.py 는 그대로. 원본 영상은 랜선을 안 탄다.
#   VISION=nuc: 예전처럼 NUC 에서 카메라 + ultralytics 노드(카메라가 NUC 에 꽂혀 있을 때만).
#   ★ 둘을 같이 켜지 말 것 — /mission/traffic_light_state 발행자가 둘이 되어 서로 덮어쓴다.
VISION=${VISION:-onboard}
if [ "$VISION" = onboard ]; then
  echo "[8·9/9] 내부 PC: 카메라 + 신호등+콘(TensorRT INT8, tl5cone) — ~/onboard_vision/jetson_vision.sh start"
  ssh -o ControlPath=~/.ssh/go2_cm_%r@%h:%p -o ControlMaster=auto -o ControlPersist=3h -o ConnectTimeout=6 \
    unitree@192.168.123.18 'bash ~/onboard_vision/jetson_vision.sh start' > "$LOGDIR/live_trafficlight.log" 2>&1; rc=$?
  cat "$LOGDIR/live_trafficlight.log"
  [ $rc = 0 ] || echo "★ 내부 PC 신호등 노드 실패(rc=$rc) — 정지선에서 10초 타임아웃 뒤 신호 무관 출발, 콘 카메라 인식도 안 됨(obstacle_fusion_node는 벽만으로 계속 동작). ~/ws/Rgosu/onboard_vision/deploy_onboard_vision.sh status 로 확인"
else
echo "[8/9] realsense2_camera (color만 — depth/infra/imu는 USB 대역폭 절약 위해 끔)"
setsid ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=false \
  enable_infra1:=false enable_infra2:=false enable_gyro:=false enable_accel:=false \
  rgb_camera.color_profile:=848x480x30 \
  </dev/null >"$LOGDIR/live_camera.log" 2>&1 &
sleep 4

echo "[9/9] traffic_light_detector_node(YOLO, venv_yolo) — camera_active=False로 대기 시작 (★ 콘 카메라 검출 없음 — VISION=nuc일 땐 obstacle_fusion_node가 벽+L1만으로 동작)"
setsid ~/ws/Rgosu/venv_yolo/bin/python3 ~/ws/Rgosu/src/traffic_light_detector_node.py \
  </dev/null >"$LOGDIR/live_trafficlight.log" 2>&1 &
fi

sleep 1
echo
if [ "$CONE" = 1 ]; then echo "=== 작전명 EGO 기동 완료(9단계 전부, 콘 인식 켬). 로그: $LOGDIR/live_*.log ==="
else echo "=== 작전명 NOEGO 기동 완료(콘 인식 끔, 5단계 건너뜀). 로그: $LOGDIR/live_*.log ==="; fi
echo "확인: tail -f $LOGDIR/live_fastlio.log   (kdtree init 이후 몇 초간 가만히 둘 것)"
echo "확인: ros2 topic hz /lidar_points   (XT16 20Hz 나오는지)"
[ "$CONE" = 1 ] && echo "확인: tail -f $LOGDIR/live_obstaclefusion.log   (5초마다 벽/L1/카메라 콘 점 개수 찍힘 — 카메라 콘 0이면 콘 인식 안 되는 중)"
echo "확인: tail -f $LOGDIR/live_trafficlight.log   (onboard: 'TensorRT 엔진 로드 완료' / nuc: 'YOLO 모델 로드 완료')"
echo "확인: ros2 topic hz /camera/color/image_raw   (카메라 나오는지)"
echo
echo "★★★ 이제 로봇을 start 지점 근처에 놓고 아래를 실행하면 경기 시작(자동으로 자기위치부터 확인함):"
echo
echo "  python3 ~/ws/Rgosu/src/real_course_runner.py ~/ws/Rgosu/maps/real_waypoints_venue_run.json"
echo
echo "  (검증된 7구간 코스: stopline1→rabain→rabaout→stopline2→line→parking→exit."
echo "   route_map.json은 line1→stopline1 구간에서 길막힘이 확인돼 아직 검증 안 됨 — 쓰지 말 것."
echo "   real_waypoints_venue_bridged.json은 line→parking 사이 벽/콘 틈 우회용 — 콘 인식이 잘 안 될 때만 대안으로.)"
echo
echo "비상정지: pkill -INT -f cmd_vel_to_sport_node   (또는 전체 정리: ~/ws/Rgosu/tools/go2_stop_all.sh -- real_course_runner.py 종료 시 자동 호출됨)"
