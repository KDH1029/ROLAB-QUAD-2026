#!/usr/bin/env bash
# /scan 사각지대 시험: 팀원 cloud_bridge_node(그대로)에 min_range 만 바꿔 11:18 라이다를 재생하고 /scan 을 녹화.
# 사용: unshare -rn bash -c 'ip link set lo up; replay.sh <min_range> <도메인>'   (네트워크 공간 분리 — 로봇과 안 섞임)
MR=$1; DOM=${2:-79}; H="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"; O=$H/mr$MR; rm -rf $O
export V_DOMAIN=$DOM; source ~/ws/v_gosu/env_replay.sh >/dev/null 2>&1
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash >/dev/null 2>&1; source ~/ws/merge_tmp_20261102/review/install/setup.bash >/dev/null 2>&1
BP=~/ws/merge_tmp_20261102/review/install/go2_bridge/share/go2_bridge/config/bridge_params.yaml
setsid ros2 run go2_bridge imu_bridge_node --ros-args --params-file $BP > $H/imu_$MR.log 2>&1 & P1=$!
setsid ros2 run go2_bridge cloud_bridge_node --ros-args --params-file $BP -p min_range:=$MR > $H/cloud_$MR.log 2>&1 & P2=$!
sleep 4
for n in go2_imu_bridge go2_cloud_bridge; do for s in configure activate; do timeout 10 ros2 lifecycle set /$n $s >/dev/null 2>&1; done; echo "$n $(timeout 5 ros2 lifecycle get /$n 2>/dev/null)"; done
setsid ros2 bag record -s mcap -o $O /scan > $H/rec_$MR.log 2>&1 & P3=$!
sleep 2
ros2 bag play ~/bags/gosu_20261106_111853_cone1106 --topics /lidar_points /lowstate --start-offset 10 --playback-duration 70 < /dev/null > $H/play_$MR.log 2>&1
sleep 2; kill -TERM -- -$P3 2>/dev/null; sleep 3; kill -INT -- -$P1 -$P2 2>/dev/null; sleep 2; kill -KILL -- -$P1 -$P2 2>/dev/null
echo "끝 $MR $(ls $O 2>/dev/null | tr '\n' ' ')"
