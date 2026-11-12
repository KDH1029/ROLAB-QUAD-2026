#!/usr/bin/env bash
# terrain_veto(C++) 무모터 재생 시험 — 도메인 81, lo 전용. 결과·로그는 이 폴더에만. 팀원 빌드(go2_bridge, fast_lio)는 읽기 전용 실행.
#   FAST-LIO(재생) → /cloud_registered·/Odometry ─┐
#   /utlidar/cloud_base (L1) · /camera/depth (D435i) ─┴→ terrain_veto → /cloud_planner
# 사용: veto_replay.sh [bag] [태그] [재생초(0=끝까지)] [추가 --ros-args 파라미터...]
BAG=${1:-$HOME/bags/gosu_20261030_001126}; TAG=${2:-tv1}; DUR=${3:-0}; shift 3 2>/dev/null; EXTRA=("$@")
N=~/ws/night_20261104/ramp/replay; mkdir -p "$N"
if ip link show robot-br 2>/dev/null | grep -q 'state UP'; then echo "★ robot-br UP — 로봇 연결 중에는 재생 안 함"; exit 1; fi
export V_DOMAIN=81 ROS_LOG_DIR=~/ws/night_20261104/logs/ros; source ~/ws/v_gosu/env_replay.sh
case "$ROS_DOMAIN_ID" in 78|79|80|81) ;; *) echo "★ 도메인 $ROS_DOMAIN_ID — 재생은 78~81 만"; exit 1;; esac
source ~/ws/night_20261104/ws/install/setup.bash
PGS=(); stop(){ for p in "${PGS[@]}"; do kill -INT -- -"$p" 2>/dev/null; done; sleep 4; for p in "${PGS[@]}"; do kill -TERM -- -"$p" 2>/dev/null; done; }; trap stop EXIT
echo "=== terrain_veto 재생 $TAG  도메인 $ROS_DOMAIN_ID  ($(date +%T)) ==="
setsid ros2 run go2_bridge imu_bridge_node --ros-args -r __node:=imu_bridge -p input_topic:=/lowstate -p output_topic:=/imu/body -p frame_id:=imu_link -p publish_static_tf:=true -p publish_stab:=true >"$N/$TAG.imu.log" 2>&1 & PGS+=($!)
setsid ros2 run go2_bridge cloud_bridge_node --ros-args -r __node:=cloud_bridge -p input_topic:=/lidar_points -p output_topic:=/velodyne_points -p frame_id:=hesai_lidar -p stride:=1 >"$N/$TAG.cloud.log" 2>&1 & PGS+=($!)
setsid ros2 run fast_lio fastlio_mapping --ros-args -r __node:=fastlio_odometry --params-file $HOME/ws/v_gosu/config/map_replay.yaml >"$N/$TAG.lio.log" 2>&1 & PGS+=($!)
setsid ros2 run terrain_veto terrain_veto_node --ros-args -p cloud_topic:=/cloud_registered -p odom_topic:=/Odometry "${EXTRA[@]}" >"$N/$TAG.veto.log" 2>&1 & VPG=$!; PGS+=($VPG)
rm -rf "$N/$TAG.rec"
setsid ros2 bag record -s mcap -o "$N/$TAG.rec" /cloud_registered /cloud_planner /Odometry >"$N/$TAG.rec.log" 2>&1 & RPG=$!; PGS+=($RPG)
sleep 6
VP=$(pgrep -f -n "terrain_veto_node"); ( while kill -0 "$VP" 2>/dev/null; do echo "$(date +%s) $(ps -o %cpu=,rss= -p "$VP")"; sleep 1; done ) >"$N/$TAG.cpu" 2>/dev/null &
if [ "$DUR" != "0" ]; then PD=(--playback-duration "$DUR"); else PD=(); fi
ros2 bag play "$BAG" --topics /lidar_points /lowstate /utlidar/cloud_base /camera/depth/image_rect_raw "${PD[@]}" >"$N/$TAG.play.log" 2>&1
sleep 3; kill -INT -- -"$RPG" 2>/dev/null; sleep 4
echo "재생 끝 ($(date +%T)). CPU: awk '{s+=\$2;n++}END{print s/n}' $N/$TAG.cpu  로그: $N/$TAG.veto.log"
