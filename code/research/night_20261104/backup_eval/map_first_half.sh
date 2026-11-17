#!/usr/bin/env bash
# 녹화 앞부분만 FAST-LIO 로 재생해 공정 검증용 지도를 만든다. 결과·로그는 이 폴더에만.
# 사용: map_first_half.sh <초>
DUR=${1:-270}; N=~/ws/night_20261104/backup_eval; BAG=~/bags/gosu_20261030_001126
if ip link show robot-br 2>/dev/null | grep -q 'state UP'; then echo "★ 로봇 연결 중 — 재생 안 함"; exit 1; fi
export V_DOMAIN=80 ROS_LOG_DIR=~/ws/night_20261104/logs/ros; source ~/ws/v_gosu/env_replay.sh
PGS=(); stop(){ for p in "${PGS[@]}"; do kill -INT -- -"$p" 2>/dev/null; done; sleep 4; for p in "${PGS[@]}"; do kill -TERM -- -"$p" 2>/dev/null; done; }; trap stop EXIT
setsid ros2 run go2_bridge imu_bridge_node --ros-args -r __node:=imu_bridge -p input_topic:=/lowstate -p output_topic:=/imu/body -p frame_id:=imu_link -p publish_static_tf:=true -p publish_stab:=true >$N/half.imu.log 2>&1 & PGS+=($!)
setsid ros2 run go2_bridge cloud_bridge_node --ros-args -r __node:=cloud_bridge -p input_topic:=/lidar_points -p output_topic:=/velodyne_points -p frame_id:=hesai_lidar -p stride:=1 >$N/half.cloud.log 2>&1 & PGS+=($!)
setsid ros2 run fast_lio fastlio_mapping --ros-args -r __node:=fastlio_odometry --params-file $HOME/ws/v_gosu/config/map_replay.yaml >$N/half.lio.log 2>&1 & PGS+=($!)
setsid python3 ~/ws/v_gosu/eval/map_collect.py $N/map_first${DUR}s_fastlio.npz 0.02 >$N/half.collect.log 2>&1 & CPG=$!
sleep 6
ros2 bag play $BAG --topics /lidar_points /lowstate --playback-duration $DUR >$N/half.play.log 2>&1
sleep 3; kill -INT -- -"$CPG" 2>/dev/null; sleep 8; grep -m1 '저장' $N/half.collect.log
