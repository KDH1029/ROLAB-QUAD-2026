#!/usr/bin/env bash
# Rgosu 전용 래퍼 — 팀의 record_all.sh(~/ws/v_gosu/tools/record_all.sh)는 안 건드리고,
# 그 앞에 "최대한 모든 센서"가 다 발행되고 있는 상태를 만들어준 다음 record_all.sh를
# 그대로 호출한다(그 파일의 --all-topics 녹화 로직은 전혀 안 바꿈). 추가로 뎁스카메라
# 0.5초 간격 스냅샷(PNG, bag과 별도)도 같이 남긴다.
#
# 이 스크립트가 띄우는 것(이미 떠있으면 중복 안 띄움):
#   1) hesai_ros_driver_node — 외장 XT16 라이다(/lidar_points). ★로봇 내장 L1
#      (/utlidar/cloud 등)은 로봇 켜지면 자동으로 나오는 거라 우리가 따로 안 띄워도
#      --all-topics가 그냥 잡아줌 — 그래서 "둘 다" 잡으려면 XT16 드라이버만 추가로
#      띄우면 됨.
#   2) realsense2_camera — D435i color+depth+infra. IMU(gyro/accel)는 기본 끔 —
#      켜면 USB 대역폭 문제로 2초 간격으로 끊김(depth_terrain/README.md에 이미
#      기록된 문제). 로봇 자체 IMU(/lowstate, 500Hz)는 이거와 무관하게 항상 같이
#      녹화되니 "IMU 데이터 자체"가 빠지는 건 아님.
#   3) depth_snapshot_saver.py — 뎁스카메라를 0.5초 간격으로 PNG 한 장씩 저장(16비트,
#      mm 단위 그대로). bag에는 전체 프레임(보통 15fps)이 다 들어가고, 이건 그와
#      별개로 "ROS 없이 이미지 뷰어로 바로 훑어보기 좋은" 폴더를 추가로 만드는 것.
#
# 사용법: record_all_with_camera.sh <이름> "<메모>"   (record_all.sh와 인자 동일)
#   예:   record_all_with_camera.sh cone_slalom_1104 "콘 슬라럼 구간, 전체 센서+카메라"
#
# 시작: 이 명령 실행하는 순간부터 녹화 시작(토픽 점검 뒤 몇 초 내로).
# 종료: Ctrl+C 한 번 — record_all.sh가 bag을 정상적으로 마무리(meta.txt 생성 등)
#       하고, 이 스크립트가 그 뒤에 카메라/라이다/스냅샷 전부 같이 내린다.

set -u
NAME="${1:?사용: record_all_with_camera.sh <이름> \"<메모>\"}"
NOTE="${2:-메모 없음}"

RECORD_ALL="$HOME/ws/v_gosu/tools/record_all.sh"
if [ ! -x "$RECORD_ALL" ]; then
  echo "★ $RECORD_ALL 을 못 찾음 — 팀 워크스페이스 경로가 다르면 이 줄만 고칠 것."
  exit 1
fi

source ~/setup_go2_ros2.sh > /dev/null 2>&1
source ~/ws/merge_tmp_20261102/review/install/setup.bash >/dev/null 2>&1

LOGDIR="$HOME/ws/Rgosu/maps/segments/logs"
mkdir -p "$LOGDIR"
PIDS=()

# --- XT16 라이다 (중복 방지) ---
if pgrep -f hesai_ros_driver_node > /dev/null; then
  echo "[라이다] hesai_ros_driver_node 이미 떠있음 — 중복 안 띄움"
else
  echo "[1/3] XT16 라이다 드라이버 기동"
  setsid ros2 run hesai_ros_driver hesai_ros_driver_node --ros-args \
    -p config_path:="$HOME/ws/merge_tmp_20261102/review/src/external/hesai_ros_driver/config/config.yaml" \
    </dev/null >"$LOGDIR/live_hesai_record.log" 2>&1 &
  PIDS+=("$!")
  sleep 2
fi

# --- D435i 카메라 (중복 방지) ---
if pgrep -f "realsense2_camera" > /dev/null; then
  echo "[카메라] realsense2_camera 이미 떠있음 — 중복 안 띄움"
else
  echo "[2/3] D435i 카메라 드라이버 기동 (color+depth+infra, IMU는 끔 -- USB 불안정 문제)"
  setsid ros2 launch realsense2_camera rs_launch.py \
    enable_color:=true enable_depth:=true enable_infra1:=true enable_infra2:=true \
    enable_gyro:=false enable_accel:=false depth_module.emitter_on_off:=true \
    rgb_camera.color_profile:=848x480x30 depth_module.depth_profile:=848x480x30 \
    depth_module.infra_profile:=848x480x30 \
    </dev/null >"$LOGDIR/live_camera_record.log" 2>&1 &
  PIDS+=("$!")
  sleep 4
fi

# --- 뎁스 스냅샷 저장기(0.5초 간격 PNG) ---
STAMP="$(date +%Y%m%d_%H%M%S)_${NAME}"
SNAP_DIR="$HOME/ws/Rgosu/maps/depth_snapshots/${STAMP}"
echo "[3/3] 뎁스 스냅샷 저장기 기동 (0.5초 간격) -> $SNAP_DIR"
setsid python3 "$HOME/ws/Rgosu/tools/depth_snapshot_saver.py" "$SNAP_DIR" \
  </dev/null >"$LOGDIR/live_depthsnap_record.log" 2>&1 &
PIDS+=("$!")

cleanup() {
  echo "녹화 종료 처리 -- 이 스크립트가 띄운 것들(라이다/카메라/스냅샷) 정리 중"
  for pid in "${PIDS[@]:-}"; do
    [ -n "$pid" ] && kill -INT "$pid" 2>/dev/null
  done
  sleep 1
  for pid in "${PIDS[@]:-}"; do
    [ -n "$pid" ] && wait "$pid" 2>/dev/null
  done
  echo "정리 완료. 뎁스 스냅샷: $SNAP_DIR"
}
trap cleanup EXIT

echo
echo "=== 센서 2초 안정화 대기 후 record_all.sh 시작 (--all-topics, 카메라/XT16/L1 전부 포함) ==="
sleep 2
"$RECORD_ALL" "$NAME" "$NOTE"
