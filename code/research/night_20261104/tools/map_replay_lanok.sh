#!/usr/bin/env bash
# [night 사본 — 안전장치만 '로봇 응답 시 거부'로 바꿈, 원본 ~/ws/v_gosu/tools/map_replay.sh]
# 녹화 bag 을 FAST-LIO 로 재생해 지도 그림 재료(복셀·궤적)를 만든다.
# 사용: [V_VOX=0.02] map_replay.sh <bag> [태그]      결과: ~/ws/v_gosu/eval/out/<태그>_map.npz (복셀 기본 5 cm)
# 도메인 78, lo 전용. 로봇이 연결돼 있으면 거부한다(재생이 라이브에 섞인 09-10 사고 방지).
BAG="${1:?사용: map_replay.sh <bag> [태그]}"; TAG="${2:-$(basename "$BAG")}"
OUT=~/ws/v_gosu/eval/out; mkdir -p "$OUT"
# [사본 11-04] 랜 포트에 다른 컴퓨터가 꽂혀 robot-br 이 UP 이어도 돈다. 대신 로봇이 실제로 켜져 있으면(본체 .161·로봇 PC .18 응답) 거부.
for h in 161 18; do ping -c 1 -W 1 192.168.123.$h >/dev/null 2>&1 && { echo "★ 로봇(192.168.123.$h)이 응답 — 켜진 로봇과 섞일 수 있어 재생하지 않는다"; exit 1; }; done
ip link show robot-br 2>/dev/null | grep -q 'state UP' && echo "(경고) robot-br UP — 로봇은 꺼져 있음 확인. 재생은 lo 전용이라 랜으로 안 나감"
source ~/ws/v_gosu/env_replay.sh
case "$ROS_DOMAIN_ID" in 78|79|80|81) ;; *) echo "★ 도메인 $ROS_DOMAIN_ID — v_gosu 재생은 78~81 만"; exit 1;; esac

PGS=()
stop(){ for p in "${PGS[@]}"; do kill -INT -- -"$p" 2>/dev/null; done; sleep 4
        for p in "${PGS[@]}"; do kill -TERM -- -"$p" 2>/dev/null; done; }
trap stop EXIT

echo "=== $TAG 지도 재생  도메인 $ROS_DOMAIN_ID  ($(date +%T)) ==="; echo "DDS: $(echo "$CYCLONEDDS_URI" | grep -o 'NetworkInterface name="[a-z0-9]*"')  자동탐색 범위 $ROS_AUTOMATIC_DISCOVERY_RANGE"
# 팀 go2_bridge (실주행과 같은 입력) → FAST-LIO (gosu_slam 검증 설정 사본, 등록 스캔 전체 발행)
setsid ros2 run go2_bridge imu_bridge_node --ros-args -r __node:=imu_bridge -p input_topic:=/lowstate \
  -p output_topic:=/imu/body -p frame_id:=imu_link -p publish_static_tf:=true -p publish_stab:=true >"$OUT/$TAG.map.imu.log" 2>&1 & PGS+=($!)
setsid ros2 run go2_bridge cloud_bridge_node --ros-args -r __node:=cloud_bridge -p input_topic:=/lidar_points \
  -p output_topic:=/velodyne_points -p frame_id:=hesai_lidar -p stride:=1 >"$OUT/$TAG.map.cloud.log" 2>&1 & PGS+=($!)
setsid ros2 run fast_lio fastlio_mapping --ros-args -r __node:=fastlio_odometry \
  --params-file "$HOME/ws/v_gosu/config/map_replay.yaml" >"$OUT/$TAG.map.lio.log" 2>&1 & PGS+=($!)
setsid python3 ~/ws/v_gosu/eval/map_collect.py "$OUT/${TAG}_map.npz" "${V_VOX:-0.05}" >"$OUT/$TAG.map.collect.log" 2>&1 & CPG=$!
sleep 6
pgrep -x fastlio_mapping >/dev/null || { echo "★ fastlio_mapping 기동 실패"; tail -5 "$OUT/$TAG.map.lio.log"; exit 1; }

ros2 bag play "$BAG" --topics /lidar_points /lowstate >"$OUT/$TAG.map.play.log" 2>&1
sleep 3
kill -INT -- -"$CPG" 2>/dev/null; sleep 8
grep -m1 '저장' "$OUT/$TAG.map.collect.log" | sed 's/.*저장/  저장/'
