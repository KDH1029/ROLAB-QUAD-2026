#!/usr/bin/env bash
# 10-30 '베이스라인(통맵핑)' 방식 그대로 — FAST-LIO(gosu1030_baseline.yaml: filter_size_map 0.08, map_en, det_range 15)가 직접 쌓은 지도를 PCD 로.
# 원본 run_fastlio_segment.sh 와 다른 점(격리만): 도메인 78·lo 전용, 재생은 /lidar_points·/lowstate 만, 남의 프로세스를 죽이지 않음,
# 저장은 map_save 서비스 먼저(FAST-LIO 포크는 종료만으로 안 쓸 수 있음) → 그다음 SIGINT.
# 사용: map_baseline_isolated.sh <녹화폴더> <출력.pcd> [설정=config/gosu1030_baseline.yaml]
set -u
BAG=${1:?녹화}; OUTP=${2:?출력.pcd}; CFG=${3:-$HOME/ws/Rgosu/config/gosu1030_baseline.yaml}; L=$(dirname "$OUTP"); mkdir -p "$L"
for h in 161 18; do ping -c1 -W1 192.168.123.$h >/dev/null 2>&1 && { echo "★ 로봇(.$h) 응답 — 재생하지 않는다"; exit 1; }; done
export V_DOMAIN=${V_DOMAIN:-78}; source ~/ws/v_gosu/env_replay.sh
case "$ROS_DOMAIN_ID" in 78|79|80|81) ;; *) exit 1;; esac
echo "격리 도메인 $ROS_DOMAIN_ID · fast_lio = $(ros2 pkg prefix fast_lio 2>/dev/null) · 설정 $CFG → $OUTP"
PGS=(); run(){ local n=$1; shift; setsid "$@" > "$L/$n.log" 2>&1 < /dev/null & PGS+=($!); }
stop(){ for p in "${PGS[@]}"; do kill -INT -- -$p 2>/dev/null; done; sleep 5; for p in "${PGS[@]}"; do kill -TERM -- -$p 2>/dev/null; done; }
trap stop EXIT
run cloud ros2 run go2_bridge cloud_bridge_node
run imu ros2 run go2_bridge imu_bridge_node
sleep 2
run lio ros2 run fast_lio fastlio_mapping --ros-args --params-file "$CFG" -p map_file_path:="$OUTP"
sleep 4
ros2 bag play "$BAG" --topics /lidar_points /lowstate > "$L/play.log" 2>&1
sleep 3
echo "재생 끝 $(date +%T) — map_save 호출"; timeout 60 ros2 service call /map_save std_srvs/srv/Trigger 2>&1 | tail -2
sleep 2; stop; trap - EXIT; sleep 2
ls -la "$OUTP" 2>/dev/null || { echo "★ PCD 없음 — $L/lio.log 확인"; ls -la "$L"; }
