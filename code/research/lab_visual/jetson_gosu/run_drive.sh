#!/usr/bin/env bash
# 팀원 계획기로 주행하되 **위치추정만 내 것으로 바꾼다.**
#
#   ./run_drive.sh --no-motor          센서·위치추정만. 모터 경로를 안 띄운다  ← 먼저 이것부터
#   ./run_drive.sh --no-motor --view   화면(8766)까지 같이
#   ./run_drive.sh                     모터까지. 로봇이 움직인다
#   POSE="1.2,-4.8,8" ./run_drive.sh --no-motor    시작 자세를 직접 줄 때
#
# ── 격리에 대해 정직하게 ────────────────────────────────────────────────
#   로봇은 eno1(192.168.123.99 -> .18) 위 **도메인 0** 으로만 말한다.
#   그래서 주행하려면 도메인 0 에 있어야 한다. gosu 의 도메인 77 + lo 격리로는
#   로봇이 움직이지 않는다. 격리의 목적이 원래 그것이다.
#
#   여기서 얻는 격리는 두 가지다:
#     · 파일 격리 — 설정·노드를 gosu 안에 둔다. 팀원 파일은 읽기만 한다
#     · 배타 실행 — 띄우기 전에 기존 스택을 전부 내린다
#   중복은 조용히 망가뜨린다. /imu/body 발행자가 2개가 되면 FAST-LIO 의
#   "imu loop back" 방어가 매 프레임 IMU 버퍼를 비워, 노드는 살아있는데 출력이 안 나온다.
#
# ── 무엇이 바뀌나 ───────────────────────────────────────────────────────
#   팀원 원래 사슬:  … fastlio -> go2_3d_loc_node.py -> /Odometry_map -> 계획기 -> /cmd_vel
#   여기서는:        … fastlio -> icp_localization + odometry_map_adapter -> /Odometry_map -> 계획기 -> /cmd_vel
#   계획기·미션·모터 브리지는 **팀원 코드 그대로** 쓴다. 다만 gosu 복사본을 빌드해 쓴다
#   (~/gosu/go2_ws). 팀원이 리빌드해도 여기 동작은 안 바뀐다.
set -u -m
GOSU_LAB=~/gosu/lab_visual
MOTOR=1; VIEW=0; PLANNER="${PLANNER:-direct}"
for x in "$@"; do
    case "$x" in
        --no-motor) MOTOR=0 ;;
        --view)     VIEW=1 ;;
        -h|--help)  sed -n 2,9p "$0"; exit 0 ;;
        *) echo "★ 모르는 인자: $x"; exit 1 ;;
    esac
done

# 환경은 gosu 것을 쓴다. 팀원 ~/go2_ws/install 은 쓰지 않는다.
set +u; source ~/gosu/env_drive.sh; set -u
if [ ! -f ~/gosu/go2_ws/install/setup.bash ]; then
    echo "★ gosu 빌드가 없다. 먼저:  ~/gosu/build.sh"
    exit 1
fi
[ -n "${CYCLONEDDS_URI:-}" ] || { echo "★ eno1 링크가 없다 — 로봇에 못 붙는다"; exit 1; }
L="$GOSU_LAB/logs"; mkdir -p "$L"

MAP="$GOSU_LAB/maps/go2_prior_map_v05.pcd"
[ -f "$MAP" ] || { echo "★ 지도가 없다: $MAP"; exit 1; }

PATS='fastlio_mapping|imu_bridge_node|cloud_bridge_node|cmd_vel_to_sport|odom_filter_node'
PATS+='|go2_3d_loc_node|ego_planner_node|go2_direct_navigator|go2_mission_manager'
PATS+='|go2_auto_cw_node|udp_goal_bridge|hesai_ros_driver|icp_localization|odometry_map_adapter'
PATS+='|odometry_monitor|map_publisher|go2_localization|amcl|map_server|bt_navigator|controller_server|foxglove_bridge'

down(){
    pkill -INT -f "$PATS" 2>/dev/null || true
    sleep 2
    pkill -KILL -f "$PATS" 2>/dev/null || true
    sleep 1
}
trap 'echo; echo "── 내린다 ──"; down; exit 0' INT TERM HUP EXIT

echo "══ gosu 주행  위치추정=내 것  계획기=팀원 것($PLANNER)  모터=$([ $MOTOR = 1 ] && echo 켬 || echo 끔) ══"
echo "── ① 남의 스택이 도는지 먼저 본다 ──"
# 우리 것은 ~/gosu/ 아래에서 돈다. 그 밖에서 도는 go2 프로세스는 **남의 것**이다.
# 그냥 죽이면 주행 중인 남의 로봇을 끊는다. 2026-09-12 에 팀원의 go2_wall_patrol 이
# 돌고 있었고, 확인 없이 죽였으면 순찰이 중간에 끊겼을 것이다.
OTHERS=""
for pid in $(pgrep -f "$PATS" 2>/dev/null); do
    exe=$(readlink -f "/proc/$pid/exe" 2>/dev/null || echo "")
    cmd=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | cut -c1-90)
    case "$exe$cmd" in
        *"/gosu/"*) ;;                       # 우리 것
        "") ;;                               # 이미 사라짐
        *) OTHERS+="   [$pid] $cmd"$'\n' ;;
    esac
done
if [ -n "$OTHERS" ] && [ "${FORCE:-0}" != "1" ]; then
    echo "   ★ gosu 밖에서 도는 프로세스가 있다 — 팀원이 로봇을 쓰는 중일 수 있다:"
    printf '%s' "$OTHERS"
    echo
    echo "   확인하고, 정말 내려도 되면  FORCE=1 로 다시 실행할 것."
    echo "   (모터 경로가 섞여 있으면 주행 중인 로봇이 멈춘다)"
    trap - EXIT; exit 1
fi
[ -n "$OTHERS" ] && echo "   ⚠ FORCE=1 — 남의 프로세스도 내린다"

echo "── ② 정리 (중복은 조용히 망가뜨린다) ──"
down
LEFT=$(pgrep -cf "$PATS" 2>/dev/null || echo 0)
echo "   남은 프로세스 $LEFT개"
[ "$LEFT" -gt 0 ] && { echo "   ★ 안 죽은 게 있다. 확인 후 다시 실행할 것"; pgrep -af "$PATS" | head -5; exit 1; }

echo "── ③ 센서·오도메트리 (팀원 노드 그대로) ──"
ros2 run hesai_ros_driver hesai_ros_driver_node --ros-args -r __ns:=/hesai_ros_driver \
    > "$L/d_hesai.log" 2>&1 &
ros2 run go2_bridge imu_bridge_node --ros-args -r __node:=go2_imu_bridge \
    -p input_topic:=/lowstate -p output_topic:=/imu/body -p frame_id:=imu_link \
    -p publish_static_tf:=true -p publish_stab:=true > "$L/d_imu.log" 2>&1 &
ros2 run go2_bridge cloud_bridge_node --ros-args -r __node:=go2_cloud_bridge \
    -p input_topic:=/lidar_points -p output_topic:=/velodyne_points -p frame_id:=hesai_lidar \
    -p stride:=1 > "$L/d_cloud.log" 2>&1 &
ros2 run fast_lio fastlio_mapping --ros-args -r __node:=fastlio_odometry \
    --params-file ~/gosu/go2_ws/config/odometry.yaml > "$L/d_lio.log" 2>&1 &
ros2 run go2_bridge odom_filter_node --ros-args -r __node:=go2_odom_filter \
    -p input_topic:=/Odometry -p output_topic:=/Odometry_filtered \
    -p alpha_pos:=0.65 -p alpha_rot:=0.60 -p max_step_dist:=0.20 > "$L/d_filter.log" 2>&1 &
sleep 6

echo "── ④ 위치추정 = 내 것 (팀원 go2_3d_loc_node.py 는 안 띄운다) ──"
# FAST-LIO 의 고정 프레임은 odom 이 아니라 camera_init 이다. 사슬을 거기에 맞춘다.
# 이제는 빌드된 패키지다 (gosu/go2_ws/src/go2_localization). 팀원 노드와 같은 방식으로 뜬다.
ros2 run go2_localization icp_localization --ros-args -r __node:=icp_localization -- \
    --map "$MAP" --hz 5 --scan-pts 800 --wide --odom-frame camera_init > "$L/d_loc.log" 2>&1 &
ros2 run go2_localization odometry_map_adapter --ros-args -r __node:=odometry_map_adapter -- \
    --odom-frame camera_init > "$L/d_adapter.log" 2>&1 &
ros2 run go2_localization odometry_monitor --ros-args -r __node:=odometry_monitor -- \
    --report-every 10 > "$L/d_monitor.log" 2>&1 &
ros2 run go2_localization map_publisher --ros-args -r __node:=map_publisher -- \
    --map "$MAP" --voxel 0.12 --band > "$L/d_map.log" 2>&1 &
sleep 3

echo "── ⑤ 시작 자세 ──"
if [ -n "${POSE:-}" ]; then
    python3 ~/gosu/lab_visual/tools/publish_initialpose.py --pose "$POSE" 2>&1 | sed 's/^/   /'
else
    echo "   POSE 를 안 줬다. 화면에서 '2D Pose Estimate' 로 찍거나"
    echo "   POSE=\"x,y,yaw\" 로 다시 실행할 것. 정합은 그때까지 대기한다."
fi
sleep 2

echo "── ⑥ 계획기·미션 (팀원 것 그대로) ──"
if [ "$PLANNER" = "ego" ]; then
    ros2 run ego_planner ego_planner_node --ros-args -r __node:=ego_planner_node \
        --params-file ~/gosu/go2_ws/config/ego_planner.yaml > "$L/d_planner.log" 2>&1 &
else
    ros2 run go2_bringup go2_direct_navigator_node --ros-args -r __node:=go2_direct_navigator \
        > "$L/d_planner.log" 2>&1 &
fi
ros2 run go2_bringup go2_mission_manager_node --ros-args -r __node:=go2_mission_manager \
    > "$L/d_mission.log" 2>&1 &
# PC 쪽 GUI(gui_app/main.py)가 UDP 9870 으로 붙는다. 이게 없으면 화면에서 목표를 못 보낸다.
ros2 run go2_bridge udp_goal_bridge_node --ros-args -r __node:=udp_goal_bridge_node \
    -p odom_topic:=/Odometry_map -p map_frame:=map -p goal_topic:=/goal_pose \
    -p path_topic:=/ego_planner/trajectory -p cloud_topic:=/cloud_registered \
    -p udp_port:=9870 > "$L/d_udp.log" 2>&1 &

if [ "$MOTOR" = 1 ]; then
    echo "── ⑦ 모터 경로 ★ 로봇이 움직인다 ★ ──"
    ros2 run go2_bridge cmd_vel_to_sport_node --ros-args -r __node:=go2_cmd_vel_to_sport \
        -p cmd_vel_topic:=/cmd_vel -p sport_req_topic:=/api/sport/request \
        -p max_vx:=0.8 -p max_vy:=0.3 -p max_wz:=1.5 -p deadman_s:=0.5 > "$L/d_motor.log" 2>&1 &
else
    echo "── ⑦ 모터 경로 안 띄움 (--no-motor) — /cmd_vel 이 나가도 로봇은 안 움직인다 ──"
fi

if [ "$VIEW" = 1 ]; then
    sleep 2
    ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8766 -p address:=0.0.0.0 \
        -p topic_whitelist:='["/tf","/tf_static","map/points","/velodyne_points","/Odometry_map","localization/status","odometry/health","/cmd_vel"]' \
        -p max_qos_depth:=1 -p use_compression:=true > "$L/d_view.log" 2>&1 &
    echo "── 화면: ws://100.66.225.8:8766 ──"
fi

echo
echo "── 상태 (5초마다). Ctrl-C 로 전부 내린다 ──"
while sleep 5; do
    printf '  정합 %-46s | 감시 %-30s | 변환 %s\n' \
      "$(grep -a '정합' "$L/d_loc.log" 2>/dev/null | tail -1 | sed 's/.*icp_localization\]: //' | cut -c1-46)" \
      "$(grep -aE '요약|★★' "$L/d_monitor.log" 2>/dev/null | tail -1 | sed 's/.*odometry_monitor\]: //' | cut -c1-30)" \
      "$(grep -a 'Hz' "$L/d_adapter.log" 2>/dev/null | tail -1 | sed 's/.*adapter\]: //' | cut -c1-26)"
done
