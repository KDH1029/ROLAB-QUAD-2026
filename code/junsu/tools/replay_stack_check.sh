#!/usr/bin/env bash
# 주행 스택 앞부분(모터 빼고)을 '실제 주행과 같은 작업공간 순서'로 녹화 재생 — review go2_bridge(lifecycle) 활성화가 되는지,
# FAST-LIO·wall_filter·obstacle_fusion 이 흐르는지 확인. ★ 격리: 도메인 81(V_DOMAIN) + lo 전용, 로봇(.161/.18) 응답하면 거부. cmd_vel_to_sport 안 띄움.
# 사용: replay_stack_check.sh <녹화폴더> [재생 초=60]
set -u
BAG=${1:?녹화폴더}; DUR=${2:-60}; TS=$(date +%H%M%S); OUT=~/ws/junsu/maps/stack_check_$TS; mkdir -p $OUT
for h in 161 18; do ping -c1 -W1 192.168.123.$h >/dev/null 2>&1 && { echo "★ 로봇(.$h) 응답 — 재생하지 않는다"; exit 1; }; done
export V_DOMAIN=${V_DOMAIN:-81}; source ~/ws/v_gosu/env_replay.sh
case "$ROS_DOMAIN_ID" in 78|79|80|81) ;; *) exit 1;; esac
set +u   # go2_full_stack.sh 와 같은 순서
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash >/dev/null 2>&1
source ~/ws/merge_tmp_20261102/review/install/setup.bash >/dev/null 2>&1
source ~/ws/v_gosu/ws_legfuse/install/setup.bash >/dev/null 2>&1
source ~/ws/junsu/ws_fastlio_tunable/install/setup.bash >/dev/null 2>&1
source ~/ws/junsu/ws_ego_planner_tunable/install/setup.bash >/dev/null 2>&1
source ~/ws/junsu/ws_wall_filter/install/setup.bash >/dev/null 2>&1
set -u
source ~/ws/junsu/tools/go2_bridge_lc.sh
echo "격리 도메인 $ROS_DOMAIN_ID · go2_bridge = $(ros2 pkg prefix go2_bridge) → $OUT"
PGS=(); run(){ local n=$1; shift; setsid "$@" > $OUT/$n.log 2>&1 < /dev/null & PGS+=($!); }
stop(){ for p in "${PGS[@]}"; do kill -INT -- -$p 2>/dev/null; done; sleep 4; for p in "${PGS[@]}"; do kill -TERM -- -$p 2>/dev/null; done; }
trap stop EXIT
run cloud ros2 run go2_bridge cloud_bridge_node --ros-args --params-file $BRIDGE_PARAMS
run imu ros2 run go2_bridge imu_bridge_node --ros-args --params-file $BRIDGE_PARAMS
sleep 2; lc_up go2_cloud_bridge | tee -a $OUT/lifecycle.txt; lc_up go2_imu_bridge | tee -a $OUT/lifecycle.txt
run lio ros2 run fast_lio fastlio_mapping --ros-args --params-file ~/ws/junsu/config/fastlio_leg_nurigwan_derange30.yaml
run wall ros2 run wall_filter wall_filter_node
run fuse python3 ~/ws/junsu/src/obstacle_fusion_node.py
sleep 4
run rec ros2 bag record -s mcap -o $OUT/out /Odometry /mission/wall_cloud /mission/obstacle_cloud /imu/body
sleep 3
timeout -s INT $DUR ros2 bag play $BAG --topics /lidar_points /lowstate /sportmodestate /utlidar/cloud_base /utlidar/robot_odom > $OUT/play.log 2>&1
sleep 3; stop; trap - EXIT
echo "■ 끝 $(date +%T) → $OUT/out"; grep -E "name:|message_count" $OUT/out/metadata.yaml | paste - - | head -6
