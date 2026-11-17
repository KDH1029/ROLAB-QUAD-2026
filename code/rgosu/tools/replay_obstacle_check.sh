#!/usr/bin/env bash
# 장애물 합치기 재생 확인 — FAST-LIO(주행 설정) → wall_filter → obstacle_fusion_node(L1 콘 + 다리 odom 기억) → /mission/obstacle_cloud 녹화.
# ★ 격리: 도메인 79(V_DOMAIN) + lo 전용. 로봇(.161/.18)이 응답하면 거부. 사용: [CAM=1] replay_obstacle_check.sh <녹화폴더> [재생 초=0(전부)]
# CAM=1: 카메라 녹화(<녹화폴더>/camera_nuctime_p1, NUC 시계)도 같이 틀고 내부 PC 카메라 노드(ONNX 시험 모드, 콘 감지)를 NUC 에서 띄운다
set -u
BAG=${1:?녹화폴더}; DUR=${2:-0}; TS=$(date +%H%M%S); OUT=~/ws/Rgosu/maps/obstacle_check_$TS; mkdir -p $OUT
for h in 161 18; do ping -c1 -W1 192.168.123.$h >/dev/null 2>&1 && { echo "★ 로봇(.$h) 응답 — 재생하지 않는다"; exit 1; }; done
export V_DOMAIN=${V_DOMAIN:-79}; source ~/ws/v_gosu/env_replay.sh
case "$ROS_DOMAIN_ID" in 78|79|80|81) ;; *) exit 1;; esac
set +u
source ~/ws/v_gosu/ws_legfuse/install/setup.bash >/dev/null 2>&1
source ~/ws/Rgosu/ws_fastlio_tunable/install/setup.bash >/dev/null 2>&1
source ~/ws/Rgosu/ws_wall_filter/install/setup.bash >/dev/null 2>&1
set -u
echo "격리 도메인 $ROS_DOMAIN_ID $(echo "$CYCLONEDDS_URI" | grep -o 'NetworkInterface name="[a-z0-9]*"') → $OUT"
PGS=(); run(){ local n=$1; shift; setsid "$@" > $OUT/$n.log 2>&1 < /dev/null & PGS+=($!); }
stop(){ for p in "${PGS[@]}"; do kill -INT -- -$p 2>/dev/null; done; sleep 4; for p in "${PGS[@]}"; do kill -TERM -- -$p 2>/dev/null; done; }
trap stop EXIT
run cloud ros2 run go2_bridge cloud_bridge_node
run imu ros2 run go2_bridge imu_bridge_node
sleep 2
run lio ros2 run fast_lio fastlio_mapping --ros-args --params-file ~/ws/Rgosu/config/fastlio_leg_nurigwan_derange30.yaml
run wall ros2 run wall_filter wall_filter_node
[ "${CAM:-0}" = 1 ] && TL_FAKE_ONNX=~/ws/Rgosu/onboard_vision/tl5cone_384x640.onnx run cam ~/venv/go2/bin/python3 ~/ws/Rgosu/onboard_vision/traffic_light_trt_node.py
run fuse python3 ~/ws/Rgosu/src/obstacle_fusion_node.py --ros-args ${FUSE_ARGS:--p use_leg_odom:=true}
sleep 4
run rec ros2 bag record -s mcap -o $OUT/out /mission/wall_cloud /mission/obstacle_cloud /mission/cone_cam /Odometry
sleep 3
grep -m2 "wall_filter" $OUT/wall.log
IN=(-i $BAG); [ "${CAM:-0}" = 1 ] && IN+=(-i $BAG/camera_nuctime_p1)
TOP=(/lidar_points /lowstate /sportmodestate /utlidar/cloud_base /utlidar/robot_odom); [ "${CAM:-0}" = 1 ] && TOP+=(/camera/color/image_raw /camera/depth/image_rect_raw /camera/color/camera_info /camera/depth/camera_info)
if [ "$DUR" -gt 0 ]; then timeout -s INT $DUR ros2 bag play "${IN[@]}" --topics "${TOP[@]}" > $OUT/play.log 2>&1
else ros2 bag play "${IN[@]}" --topics "${TOP[@]}" > $OUT/play.log 2>&1; fi
sleep 3; stop; trap - EXIT
echo "■ 끝 $(date +%T) → $OUT/out"; ls $OUT/out
