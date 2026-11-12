#!/usr/bin/env bash
# 고친 wall_filter(C++)를 실제 녹화로 재생 확인 — FAST-LIO(주행 스택과 같은 설정) → wall_filter → /mission/wall_cloud 녹화.
# ★ 격리: 도메인 79 + lo 전용. 로봇(.161/.18)이 응답하면 거부. 사용: replay_wallfilter_check.sh <녹화폴더> [재생 초=0(전부)]
set -u
BAG=${1:?녹화폴더}; DUR=${2:-0}; TS=$(date +%H%M%S); OUT=~/ws/junsu/maps/wallfilter_fixcheck_$TS; mkdir -p $OUT
for h in 161 18; do ping -c1 -W1 192.168.123.$h >/dev/null 2>&1 && { echo "★ 로봇(.$h) 응답 — 재생하지 않는다"; exit 1; }; done
export V_DOMAIN=${V_DOMAIN:-79}; source ~/ws/v_gosu/env_replay.sh
case "$ROS_DOMAIN_ID" in 78|79|80|81) ;; *) exit 1;; esac
set +u
source ~/ws/v_gosu/ws_legfuse/install/setup.bash >/dev/null 2>&1
source ~/ws/junsu/ws_fastlio_tunable/install/setup.bash >/dev/null 2>&1
source ~/ws/junsu/ws_wall_filter/install/setup.bash >/dev/null 2>&1
set -u
echo "격리 도메인 $ROS_DOMAIN_ID $(echo "$CYCLONEDDS_URI" | grep -o 'NetworkInterface name="[a-z0-9]*"') → $OUT"
PGS=(); run(){ local n=$1; shift; setsid "$@" > $OUT/$n.log 2>&1 < /dev/null & PGS+=($!); }
stop(){ for p in "${PGS[@]}"; do kill -INT -- -$p 2>/dev/null; done; sleep 4; for p in "${PGS[@]}"; do kill -TERM -- -$p 2>/dev/null; done; }
trap stop EXIT
run cloud ros2 run go2_bridge cloud_bridge_node
run imu ros2 run go2_bridge imu_bridge_node
sleep 2
run lio ros2 run fast_lio fastlio_mapping --ros-args --params-file ~/ws/junsu/config/fastlio_leg_nurigwan_derange30.yaml
run wall ros2 run wall_filter wall_filter_node
sleep 4
run rec ros2 bag record -s mcap -o $OUT/out /mission/wall_cloud /Odometry
sleep 3
grep -m2 "wall_filter" $OUT/wall.log
if [ "$DUR" -gt 0 ]; then timeout -s INT $DUR ros2 bag play $BAG --topics /lidar_points /lowstate /sportmodestate > $OUT/play.log 2>&1
else ros2 bag play $BAG --topics /lidar_points /lowstate /sportmodestate > $OUT/play.log 2>&1; fi
sleep 3; stop; trap - EXIT
echo "■ 끝 $(date +%T) → $OUT/out"; ls $OUT/out
