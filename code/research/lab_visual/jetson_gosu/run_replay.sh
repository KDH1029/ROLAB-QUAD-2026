#!/usr/bin/env bash
# 젯슨에서 **녹화본으로** 내 스택만 돌려 본다. 로봇도 팀원 스택도 건드리지 않는다.
#
#   ./run_replay.sh <녹화본경로> [측정초]
#   ./run_replay.sh ~/bags/data3 120
#
# 흐름
#   ① 격리 점검 (실패하면 여기서 멈춘다)
#   ② bag 재생 (/lidar_points, /lowstate 만)
#   ③ 팀원 브리지(메시지 변환용) + FAST-LIO      -> /velodyne_points, /imu/body, /Odometry
#   ④ 내 3D 정합 위치추정                        -> TF map->odom, localization/status
#   ⑤ 내 오도메트리 감시                          -> odometry/health
#   전부 도메인 77, lo 전용. 로봇으로 나가는 경로가 없다.
#
# 【무엇을 확인하는가】
#   · 젯슨 CPU 로 정합이 실시간을 따라가는가 (노트북에서는 11.2% 였다)
#   · 잔차가 노트북과 같은 값이 나오는가 (정상 녹화본 0.043~0.046 m)
#   · 감시 노드가 정상 주행을 이상으로 오판하지 않는가
set -u
BAG="${1:-}"; SEC="${2:-120}"
LAB=~/gosu/lab_visual
[ -n "$BAG" ] || { sed -n 2,8p "$0"; exit 1; }
[ -d "$BAG" ] || { echo "★ 녹화본이 없다: $BAG"; exit 1; }

# ROS 의 setup.bash 는 안 정해진 변수를 건드린다. set -u 와 부딪히므로 잠시 푼다.
set +u
source "$LAB/env.sh"
set -u
bash "$LAB/check_isolation.sh" || { echo "중단한다."; exit 1; }

MAP="$LAB/maps/go2_prior_map_v05.pcd"
[ -f "$MAP" ] || { echo "★ 지도가 없다: $MAP"; exit 1; }
L="$LAB/logs"; mkdir -p "$L"

# 우리 도메인의 우리 프로세스만 정리한다. 팀원 것은 도메인이 달라 애초에 안 잡힌다.
down(){
  for p in $(pgrep -f "icp_localization.py|odometry_monitor.py|fastlio_mapping|imu_bridge_node|cloud_bridge_node|odom_to_tf|ros2 bag play" 2>/dev/null); do
    if tr '\0' '\n' < "/proc/$p/environ" 2>/dev/null | grep -q '^ROS_DOMAIN_ID=77$'; then kill "$p" 2>/dev/null; fi
  done
  sleep 1
}
trap 'echo; echo "  중단"; down; exit 130' INT TERM
down

echo "══ 격리 재생 시험  녹화본 $(basename "$BAG")  ${SEC}초 ══"
ros2 run go2_bridge imu_bridge_node --ros-args -r __node:=imu_bridge \
    -p use_sim_time:=true -p input_topic:=/lowstate -p output_topic:=/imu/body -p frame_id:=imu_link \
    -p publish_static_tf:=true -p publish_stab:=true > "$L/imu.log" 2>&1 &
ros2 run go2_bridge cloud_bridge_node --ros-args -r __node:=cloud_bridge \
    -p use_sim_time:=true -p input_topic:=/lidar_points -p output_topic:=/velodyne_points -p frame_id:=hesai_lidar \
    -p stride:=1 > "$L/cloud.log" 2>&1 &
ros2 run fast_lio fastlio_mapping --ros-args -r __node:=fastlio_odometry \
    -p use_sim_time:=true --params-file ~/go2_ws/config/odometry.yaml > "$L/lio.log" 2>&1 &
sleep 3
python3 "$LAB/src/icp_localization.py" --map "$MAP" --hz 5 --scan-pts 600 --wide --ros-args -p use_sim_time:=true \
    > "$L/loc.log" 2>&1 &
python3 "$LAB/src/odometry_monitor.py" --report-every 10 --ros-args -p use_sim_time:=true > "$L/monitor.log" 2>&1 &
sleep 2

echo "  재생 시작 (${SEC}초)"
timeout "$SEC" ros2 bag play "$BAG" --clock --rate "${RATE:-1}" > "$L/play.log" 2>&1

echo
echo "── 위치추정 요약 ──"; grep -E "rms=|잔차|healthy" "$L/loc.log" | tail -8
echo "── 감시 요약 ──";     tail -8 "$L/monitor.log"
echo "── CPU ──"
for n in icp_localization odometry_monitor fastlio_mapping; do
    p=$(pgrep -f "$n" | head -1)
    [ -n "$p" ] && printf '   %-20s %s%%\n' "$n" "$(ps -p "$p" -o %cpu= | tr -d ' ')"
done
down
echo "  로그: $L"
