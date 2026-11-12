#!/usr/bin/env bash
# 같은 bag · 같은 브리지 · 같은 평가로 오도메트리 하나를 재생 평가한다.
# 사용: run_compare.sh <fastlio|livo> <태그> [재생초(기본 전체)] [bag]
# 도메인 78, lo 전용. 로봇이 연결돼 있으면 거부한다(재생이 라이브에 섞인 09-10 사고 방지).
SYS="$1"; TAG="$2"; DUR="${3:-0}"; BAG="${4:-$HOME/bags/arena_0921}"
OUT=~/ws/v_gosu/eval/out; mkdir -p "$OUT"
if ip link show robot-br 2>/dev/null | grep -q 'state UP'; then echo "★ robot-br 가 UP — 로봇 연결 중에는 재생하지 않는다"; exit 1; fi
source ~/ws/v_gosu/env_replay.sh
case "$ROS_DOMAIN_ID" in 78|79|80|81) ;; *) echo "★ 도메인 $ROS_DOMAIN_ID — v_gosu 재생은 78~81 만"; exit 1;; esac

case "$SYS" in
  fastlio) LIO=(ros2 run fast_lio fastlio_mapping --ros-args -r __node:=fastlio_odometry --params-file "${V_CFG:-$HOME/ws/gosu_slam/config/go2_slam_odometry.yaml}")
           ODOM=/Odometry; EXE=fastlio_mapping ;;
  legfuse) source ~/ws/v_gosu/ws_legfuse/install/setup.bash     # 다리 속도 결합판 FAST-LIO (v_gosu)
           LIO=(ros2 run fast_lio fastlio_mapping --ros-args -r __node:=fastlio_odometry --params-file "${V_CFG:-$HOME/ws/v_gosu/config/fastlio_leg_n10.yaml}")
           ODOM=/Odometry; EXE=fastlio_mapping ;;
  livo)    LIO=(ros2 run fast_livo fastlivo_mapping --ros-args -r __node:=laserMapping --params-file "$HOME/ws/v_gosu/config/livo_go2.yaml")
           ODOM=/aft_mapped_to_init; EXE=fastlivo_mapping ;;
  *) echo "시스템: fastlio | legfuse | livo"; exit 1 ;;
esac

PGS=()
start(){ setsid "$@" >"$OUT/${TAG}.$1.$2.log" 2>&1 & PGS+=($!); }
stop(){ for p in "${PGS[@]}"; do kill -INT -- -"$p" 2>/dev/null; done; sleep 4
        for p in "${PGS[@]}"; do kill -TERM -- -"$p" 2>/dev/null; done; }
trap stop EXIT

echo "=== $TAG : $SYS  도메인 $ROS_DOMAIN_ID  설정 ${V_CFG:-기본}  브리지 ${V_BRIDGE:-team} deskew ${V_DESKEW:-false}  ($(date +%T)) ==="
if [ "${V_BRIDGE:-team}" = ours ]; then
  # 우리 go2_liosam_bridge (~/ws/liosam_ws, 읽기 전용): IMU=로봇 tick, 라이다=Hesai 점 시각 → ClockAligner 로 호스트 시계에 정렬
  OB="source $HOME/ws/liosam_ws/install/setup.bash"
  setsid bash -c "$OB && exec ros2 run go2_liosam_bridge imu_bridge --ros-args -r __node:=imu_bridge -p input_topic:=/lowstate -p output_topic:=/imu/body -p frame_id:=imu_link -p reliable_qos:=true" >"$OUT/$TAG.imu.log" 2>&1 & PGS+=($!)
  setsid bash -c "$OB && exec ros2 run go2_liosam_bridge cloud_bridge --ros-args -r __node:=cloud_bridge -p input_topic:=/lidar_points -p output_topic:=/velodyne_points -p frame_id:=hesai_lidar -p stride:=1 -p reliable_qos:=true -p deskew:=${V_DESKEW:-false} -p imu_topic:=/imu/body -p base_frame:=base_link -p deskew_bin_s:=0.001" >"$OUT/$TAG.cloud.log" 2>&1 & PGS+=($!)
  if [ "${V_DESKEW:-false}" = true ]; then   # 왜곡보정용 base_link→hesai_lidar (팀 브리지가 내던 것과 같은 yaw 90°)
    setsid ros2 run tf2_ros static_transform_publisher --x 0.25 --y 0 --z 0.1588 --yaw 1.5708 --pitch 0 --roll 0 --frame-id base_link --child-frame-id hesai_lidar >"$OUT/$TAG.tf.log" 2>&1 & PGS+=($!)
  fi
else
setsid ros2 run go2_bridge imu_bridge_node --ros-args -r __node:=imu_bridge -p input_topic:=/lowstate \
  -p output_topic:=/imu/body -p frame_id:=imu_link -p publish_static_tf:=true -p publish_stab:=true >"$OUT/$TAG.imu.log" 2>&1 & PGS+=($!)
setsid ros2 run go2_bridge cloud_bridge_node --ros-args -r __node:=cloud_bridge -p input_topic:=/lidar_points \
  -p output_topic:=/velodyne_points -p frame_id:=hesai_lidar -p stride:=1 >"$OUT/$TAG.cloud.log" 2>&1 & PGS+=($!)
fi
[ "$SYS" = livo ] && { setsid ros2 run demo_nodes_cpp parameter_blackboard --ros-args --params-file "$HOME/ws/v_gosu/config/livo_go2.yaml" >"$OUT/$TAG.cam.log" 2>&1 & PGS+=($!); sleep 2; }   # FAST-LIVO2 카메라 값은 blackboard 에서 읽는다
setsid "${LIO[@]}" >"$OUT/$TAG.lio.log" 2>&1 & LIOPG=$!; PGS+=($LIOPG)
setsid python3 ~/ws/v_gosu/eval/live_eval.py "$ODOM" /cloud_registered "$OUT/$TAG" >"$OUT/$TAG.eval.log" 2>&1 & EVALPG=$!
sleep 6
LP=$(pgrep -s "$LIOPG" -x "${EXE:0:15}" | head -1)   # 이 실행의 세션 안에서만 찾는다(동시 실행 대비)
[ -n "$LP" ] || { echo "★ $EXE 기동 실패"; tail -5 "$OUT/$TAG.lio.log"; exit 1; }
c0=$(awk '{print $14+$15}' /proc/$LP/stat); w0=$(date +%s.%N)

PLAYARGS=(--topics /lidar_points /lowstate /utlidar/robot_odom)
[ "$DUR" != 0 ] && PLAYARGS+=(--playback-duration "$DUR")
ros2 bag play "$BAG" "${PLAYARGS[@]}" >"$OUT/$TAG.play.log" 2>&1
sleep 3
c1=$(awk '{print $14+$15}' /proc/$LP/stat 2>/dev/null); w1=$(date +%s.%N)
alive=$(kill -0 "$LP" 2>/dev/null && echo 살아있음 || echo "★죽음")
kill -INT -- -"$EVALPG" 2>/dev/null; sleep 5
python3 -c "
t=$(getconf CLK_TCK); c0=${c0:-0}; c1=${c1:-0}; w=$w1-$w0
print(f'  {\"$EXE\"} CPU(1코어 기준) {100*(c1-c0)/t/w:.1f}%   프로세스 $alive')" 2>/dev/null
grep -m1 -E '저장' "$OUT/$TAG.eval.log" | sed 's/.*저장/  저장/'
