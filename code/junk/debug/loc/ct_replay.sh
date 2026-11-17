#!/usr/bin/env bash
# 라이다 콘 검출 재생 시험: 원본 bag → 대회 브리지 + junk FAST-LIO → 지도←몸 TF(/tf_amcl, 출발 T) → 콘 없는 지도(A) + 콘 추적기 2개
#   cone_def = 팀원 설정 그대로(작은 콘) · cone_big = 큰 콘 설정.  사용: ct_replay.sh <bag> <태그> <도메인>  → out/<태그>/ct (mcap)
set +u
BAG=$1; TAG=$2; DOM=$3; D="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"; O=$D/out/$TAG; rm -rf $O/ct; mkdir -p $O
if ip link show robot-br 2>/dev/null | grep -q 'state UP'; then echo "★ 로봇 연결됨 — 거부"; exit 1; fi
export V_DOMAIN=$DOM; source ~/ws/v_gosu/env_replay.sh; source ~/ws/merge_tmp_20261102/review/install/setup.bash; source ~/ws/junk/ws_fastlio_tunable/install/setup.bash
case "$ROS_DOMAIN_ID" in 78|79|80|81) ;; *) echo "★ 도메인"; exit 1;; esac
BP=~/ws/merge_tmp_20261102/review/install/go2_bridge/share/go2_bridge/config/bridge_params.yaml
PGS=(); stop(){ for p in "${PGS[@]}"; do kill -INT -- -"$p" 2>/dev/null; done; sleep 3; for p in "${PGS[@]}"; do kill -TERM -- -"$p" 2>/dev/null; done; }; trap stop EXIT
setsid ros2 run go2_bridge imu_bridge_node --ros-args --params-file $BP -p use_sim_time:=true >$O/ct_imu.log 2>&1 & PGS+=($!)
setsid ros2 run go2_bridge cloud_bridge_node --ros-args --params-file $BP -p use_sim_time:=true >$O/ct_cloud.log 2>&1 & PGS+=($!)
setsid ros2 run fast_lio fastlio_mapping --ros-args --params-file ~/ws/junk/config/fastlio_leg_nurigwan_derange30.yaml -p use_sim_time:=true >$O/ct_lio.log 2>&1 & PGS+=($!)
setsid python3 $D/amcl_relay.py --ros-args -p scen:=S0 -p csv:=$O/ct_relay.csv -p use_sim_time:=true >$O/ct_relay.log 2>&1 & PGS+=($!)
setsid ros2 run tf2_ros static_transform_publisher --x -0.025 --y -0.039 --yaw -0.0604 --frame-id map --child-frame-id lio_odom \
  --ros-args -r /tf_static:=/tf_amcl_static -p use_sim_time:=true >$O/ct_static.log 2>&1 & PGS+=($!)
setsid ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$D/out/maps/A/map.yaml -p use_sim_time:=true >$O/ct_map.log 2>&1 & PGS+=($!)
for n in def big; do
  setsid ros2 run mission_vision cone_tracker_node --ros-args -r __node:=cone_$n --params-file $D/ct_params.yaml -r /tf:=/tf_amcl -r /tf_static:=/tf_amcl_static \
    -r /mission/cones:=/mission/cones_$n -r /mission/cone_obstacles:=/mission/cone_obs_$n -r /mission/cone_markers:=/mission/cone_mk_$n >$O/ct_$n.log 2>&1 & PGS+=($!)
done
sleep 5
for n in go2_imu_bridge go2_cloud_bridge map_server cone_def cone_big; do for s in configure activate; do timeout 10 ros2 lifecycle set /$n $s >/dev/null 2>&1; done; echo -n "$n:$(timeout 5 ros2 lifecycle get /$n 2>/dev/null | cut -d' ' -f1) "; done; echo
setsid ros2 bag record -s mcap -o $O/ct --use-sim-time /mission/cones_def /mission/cones_big >$O/ct_rec.log 2>&1 & RPG=$!
sleep 3
ros2 bag play "$BAG" --clock 100 --topics /lidar_points /lowstate /utlidar/robot_odom /sportmodestate < /dev/null >$O/ct_play.log 2>&1
sleep 3; kill -TERM -- -"$RPG" 2>/dev/null; sleep 3
echo "콘 검출 재생 끝 $(date +%T)"
