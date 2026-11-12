#!/usr/bin/env bash
# 노트북에서 녹화본을 재생하면서 화면으로 본다. 로봇도 젯슨도 건드리지 않는다.
#
#   ./run_gui_replay.sh data3           기본 (끝까지 재생, 창 켜짐)
#   ./run_gui_replay.sh data3 120       120초만
#   RATE=1 ./run_gui_replay.sh data3    실시간 (창 없이 NOGUI=1 일 때만 권함)
#
# 기본 0.5배속인 이유: 8코어 노트북에서 fastlio 혼자 160%, rviz 19% 를 먹는다.
# 1배속으로 돌리면 스캔 입력이 20 -> 0~13 Hz 로 떨어지고 잔차가 0.045 -> 0.148 m 로 나빠지며,
# rviz 의 DDS 수신 스레드가 밀린 큐를 처리하다 스택을 넘겨 죽는다(실측).
#   NOGUI=1 ./run_gui_replay.sh data3   창 없이 숫자만
#   POSE=1.2,-4.8,8 ./run_gui_replay.sh data3   시작 자세를 직접 줄 때
#
# 시작 자세는 아는 녹화본이면 자동으로 넣는다(tools/publish_initialpose.py 의 표).
# 모르는 녹화본이면 rviz 의 "2D Pose Estimate" 로 화면에서 직접 찍으면 된다.
#
# 【무엇을 보면 되나】
#   흰 점 = 사전 지도, 색 점 = 지금 스캔.  **이 둘이 겹치면 정합이 맞는 것이다.**
#   벽이 두 겹으로 보이거나 스캔이 지도 밖으로 나가면 틀어진 것이다.
#   초록 화살표 = 위치추정 결과, 주황 화살표 = FAST-LIO 오도메트리.
#
# 【시각 — 중요】
#   모든 노드에 use_sim_time:=true 를 준다. bag 이 --clock 으로 /clock 을 내므로 now() 가 bag 시각이 된다.
#   이게 없으면 cloud_bridge 가 벽시계로 시각을 다시 찍는데(cloud_bridge_node.cpp:95),
#   배속이 1이 아니면 벽시계와 녹화 시각이 벌어져 2초마다 +2.1초씩 튄다(실측).
#   그러면 rviz 가 "Could not transform from [hesai_lidar] to [map]" 를 내고 정합도 무너진다.
#
# 【격리】
#   ~/ws/lab/env.sh 가 도메인 77 + 루프백 전용 + 멀티캐스트 끔을 건다.
#   로봇이 켜져 있어도 이 노트북 밖으로 한 바이트도 안 나간다.
set -u
BAG="${1:-data3}"; SEC="${2:-0}"
# 기본 0.5배속 — 노트북은 실시간(1배)을 못 버틴다. CPU 포화로 잔차가 3배 나빠지고
# rviz 의 DDS 수신 스레드가 밀린 큐를 처리하다 스택을 넘겨 죽는다(둘 다 실측).
RATE="${RATE:-0.5}"
NOGUI="${NOGUI:-0}"
LAB=~/ws/lab_visual
BAGDIR=~/ws/lab/bags/$BAG
[ -d "$BAGDIR" ] || { echo "★ 녹화본이 없다: $BAGDIR"; echo "  있는 것:"; ls ~/ws/lab/bags | sed 's/^/    /'; exit 1; }
MAP="$LAB/maps/go2_prior_map_v05.pcd"
[ -f "$MAP" ] || { echo "★ 지도가 없다: $MAP"; exit 1; }

# ROS 의 setup.bash 는 안 정해진 변수를 건드린다. set -u 와 부딪히므로 잠시 푼다.
set +u
source ~/ws/lab/env.sh
set -u
L="$LAB/logs"; mkdir -p "$L"

# 우리 도메인(77) 프로세스만 정리한다. 다른 도메인은 애초에 대상이 아니다.
down(){
  for p in $(pgrep -f "icp_localization.py|odometry_monitor.py|map_publisher.py|fastlio_mapping|imu_bridge_node|cloud_bridge_node|odom_to_tf|ros2 bag play|rviz2" 2>/dev/null); do
    tr '\0' '\n' < "/proc/$p/environ" 2>/dev/null | grep -q '^ROS_DOMAIN_ID=77$' && kill "$p" 2>/dev/null
  done
  sleep 1
}
trap 'echo; echo "  끝낸다"; down; exit 0' INT TERM
down

echo "══ 재생 + 화면  녹화본 $BAG  ${RATE}배속 ══"

# ① 지도를 먼저 올린다 (latched — rviz 가 나중에 떠도 받는다)
python3 "$LAB/src/map_publisher.py" --map "$MAP" --voxel 0.12 --band --ros-args -p use_sim_time:=true > "$L/gui_map.log" 2>&1 &
# ② 팀원 브리지 + FAST-LIO
ros2 run go2_bridge imu_bridge_node --ros-args -r __node:=imu_bridge \
    -p use_sim_time:=true -p input_topic:=/lowstate -p output_topic:=/imu/body -p frame_id:=imu_link \
    -p publish_static_tf:=true -p publish_stab:=true > "$L/gui_imu.log" 2>&1 &
ros2 run go2_bridge cloud_bridge_node --ros-args -r __node:=cloud_bridge \
    -p use_sim_time:=true -p input_topic:=/lidar_points -p output_topic:=/velodyne_points -p frame_id:=hesai_lidar \
    -p stride:=1 > "$L/gui_cloud.log" 2>&1 &
ros2 run fast_lio fastlio_mapping --ros-args -r __node:=fastlio_odometry \
    -p use_sim_time:=true --params-file ~/ws/lab/go2_ws/config/odometry.yaml > "$L/gui_lio.log" 2>&1 &
# ③ odom -> base_link
ros2 run go2_amcl_layer odom_to_tf --ros-args -r __node:=odom_to_tf \
    -p use_sim_time:=true -p odom_topic:=/Odometry -p parent_frame:=odom -p child_frame:=base_link > "$L/gui_odomtf.log" 2>&1 &
sleep 4
# ④ 내 위치추정 + 감시
python3 "$LAB/src/icp_localization.py" --map "$MAP" --hz 5 --scan-pts 600 --wide --ros-args -p use_sim_time:=true > "$L/gui_loc.log" 2>&1 &
python3 "$LAB/src/odometry_monitor.py" --report-every 10 --ros-args -p use_sim_time:=true > "$L/gui_monitor.log" 2>&1 &
sleep 2

if [ "$NOGUI" != "1" ]; then
    rviz2 -d "$LAB/config/lab_visual.rviz" --ros-args -p use_sim_time:=true > "$L/gui_rviz.log" 2>&1 &
    echo "  창을 띄웠다. 흰 지도 위에 색 스캔이 겹치는지 보면 된다."
    sleep 3
fi

# ⑤ 시작 자세를 넘긴다. 이게 없으면 정합 노드는 /initialpose 를 기다리며 아무것도 안 한다.
#    오도메트리가 붙은 뒤에 보내야 하므로 재생을 먼저 띄우고 뒤에서 보낸다.
(
  sleep 6
  if [ -n "${POSE:-}" ]; then
      python3 "$LAB/tools/publish_initialpose.py" --pose "$POSE"
  else
      python3 "$LAB/tools/publish_initialpose.py" --bag "$BAG" \
        || echo "  ★ 아는 시작 자세가 없다 — rviz 의 '2D Pose Estimate' 로 직접 찍을 것"
  fi
) > "$L/gui_init.log" 2>&1 &

echo "  재생 시작$([ "$SEC" != 0 ] && echo " (${SEC}초)")"
if [ "$SEC" != 0 ]; then
    timeout "$SEC" ros2 bag play "$BAGDIR" --clock --rate "$RATE" > "$L/gui_play.log" 2>&1
else
    ros2 bag play "$BAGDIR" --clock --rate "$RATE" > "$L/gui_play.log" 2>&1
fi

echo
echo "── 위치추정 ──"; grep -E "rms=|healthy" "$L/gui_loc.log" | tail -6
echo "── 감시 ──";     tail -6 "$L/gui_monitor.log"
echo
echo "  재생이 끝났다. 창은 켜 둔 채다 — 다 보고 Ctrl-C 로 끄면 전부 정리된다."
echo "  로그: $L/gui_*.log"
wait
