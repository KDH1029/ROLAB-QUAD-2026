#!/usr/bin/env bash
# 녹화 재생 (~/go2_record) — NUC 녹화 + 카메라(NUC 시계) 를 시각 맞춰 함께 재생하고 rviz2 를 띄운다.
# ★ 격리 재생: 도메인 79 + 루프백(lo) 전용 — 연결된 실제 로봇과 섞이지 않는다(09-10 재생 섞임 사고).
# 사용:  go2_record_play.sh <녹화폴더> [속도=1.0] [rviz=1]
#   예:  go2_record_play.sh ~/bags/gosu_20261104_143520
#        go2_record_play.sh ~/bags/gosu_20261104_143520 0.5 0      # 절반 속도, rviz 안 띄움
# 끝내기: Ctrl+C
set -u
OUT="${1:?사용: go2_record_play.sh <녹화폴더> [속도] [rviz=1]}"; OUT="${OUT%/}"; RATE="${2:-1.0}"; RVIZ="${3:-1}"
[ -f "$OUT/metadata.yaml" ] || { echo "★ $OUT 에 metadata.yaml 이 없다 — 녹화 폴더(~/bags/gosu_<시각>)를 주세요"; exit 1; }
export V_DOMAIN=79; source ~/ws/v_gosu/env_replay.sh
echo "격리 재생: 도메인 $ROS_DOMAIN_ID, DDS $(echo "$CYCLONEDDS_URI" | grep -o 'NetworkInterface name="[a-z0-9]*"'), 탐색 $ROS_AUTOMATIC_DISCOVERY_RANGE"
case "$ROS_DOMAIN_ID" in 78|79|80|81) ;; *) echo "★ 도메인 $ROS_DOMAIN_ID — 격리 도메인이 아니다. 중단"; exit 1;; esac
ARGS=(-i "$OUT" mcap)
for c in "$OUT"/camera_nuctime_*; do [ -f "$c/metadata.yaml" ] && ARGS+=(-i "$c" mcap); done
echo "재생: $OUT  + 카메라 $(( (${#ARGS[@]} - 3) / 3 ))조각  · 속도 $RATE"
echo "rviz2 에서 Fixed Frame 을 hesai_lidar 또는 camera_depth_optical_frame 으로, Add → By topic → /lidar_points, /camera/color/image_raw, /camera/depth/image_rect_raw"
[ "$RVIZ" = 1 ] && { setsid rviz2 >/dev/null 2>&1 & RP=$!; trap 'kill $RP 2>/dev/null' EXIT; sleep 2; }
ros2 bag play "${ARGS[@]}" -r "$RATE" --read-ahead-queue-size 2000
