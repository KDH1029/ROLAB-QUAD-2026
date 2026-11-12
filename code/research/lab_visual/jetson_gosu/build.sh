#!/usr/bin/env bash
# gosu 안의 팀원 코드 복사본을 빌드한다. 팀원 ~/go2_ws 는 건드리지 않는다.
#
#   ~/gosu/build.sh              전부 빌드 (처음 한 번, 10~25분)
#   ~/gosu/build.sh go2_bridge   그 패키지만
#   ~/gosu/build.sh --clean      build/install 지우고 처음부터
#
# 【왜 따로 빌드하나】
# 팀원이 ~/go2_ws 를 리빌드해도 여기 동작이 안 바뀌게 하려는 것이다.
# 벤더 드라이버(unitree_ros2, hesai_ws)는 복사하지 않고 빌려 쓴다 — 바뀔 일이 없다.
#
# 【주의】
# 빌드는 CPU 를 다 쓴다. 주행 중이거나 팀원이 젯슨을 쓰는 중이면 하지 마라.
set -u
WS=~/gosu/go2_ws
cd "$WS" || { echo "★ $WS 가 없다"; exit 1; }

CLEAN=0; PKGS=()
for x in "$@"; do
    case "$x" in
        --clean) CLEAN=1 ;;
        -h|--help) sed -n 2,6p "$0"; exit 0 ;;
        *) PKGS+=("$x") ;;
    esac
done

# 주행 중이면 빌드하지 않는다. 심볼릭 설치라 도는 노드의 파일을 갈아끼우게 된다.
RUNNING=$(pgrep -cf "fastlio_mapping|ego_planner_node|go2_mission_manager|cmd_vel_to_sport" 2>/dev/null || echo 0)
if [ "$RUNNING" -gt 0 ]; then
    echo "★ 주행 스택이 돌고 있다 ($RUNNING개). 내리고 빌드할 것."
    pgrep -af "fastlio_mapping|ego_planner_node|go2_mission_manager|cmd_vel_to_sport" | head -4
    exit 1
fi

set +u
source /opt/ros/humble/setup.bash
[ -f ~/unitree_ros2/cyclonedds_ws/install/setup.bash ] && source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
[ -f ~/ws/hesai_ws/install/setup.bash ] && source ~/ws/hesai_ws/install/setup.bash
set -u

if [ "$CLEAN" = 1 ]; then
    echo "── build/install/log 지운다 ──"
    rm -rf build install log
fi

# 젯슨 RAM 이 7.6 GB 다. 6개 병렬로 C++ 를 컴파일하면 메모리가 터진다.
JOBS="${JOBS:-3}"
echo "══ gosu 빌드  ($WS)  병렬 $JOBS ══"
echo "  (CPU 를 다 쓴다. 10~25분 걸린다)"
ARGS=(--symlink-install --parallel-workers "$JOBS"
      --cmake-args -DCMAKE_BUILD_TYPE=Release)
[ ${#PKGS[@]} -gt 0 ] && ARGS=(--packages-select "${PKGS[@]}" "${ARGS[@]}")

t0=$(date +%s)
colcon build "${ARGS[@]}" 2>&1 | tail -30
rc=${PIPESTATUS[0]}
t1=$(date +%s)
echo
echo "── $(( (t1-t0)/60 ))분 $(( (t1-t0)%60 ))초 ──"
if [ $rc -ne 0 ]; then
    echo "✗ 빌드 실패. 자세한 로그: $WS/log/latest_build"
    exit $rc
fi

echo "── 확인 ──"
set +u; source "$WS/install/setup.bash"; set -u
ok=1
for p in "go2_bridge imu_bridge_node" "go2_bridge cloud_bridge_node" "go2_bridge odom_filter_node" \
         "go2_bridge cmd_vel_to_sport_node" "go2_bridge udp_goal_bridge_node" \
         "fast_lio fastlio_mapping" "ego_planner ego_planner_node" \
         "go2_bringup go2_direct_navigator_node" "go2_bringup go2_mission_manager_node"; do
    set -- $p
    if ros2 pkg executables "$1" 2>/dev/null | grep -q " $2$"; then
        printf '   ✓ %-14s %s\n' "$1" "$2"
    else
        printf '   ★ 없음 %-11s %s\n' "$1" "$2"; ok=0
    fi
done
echo
[ "$ok" = 1 ] && echo "✔ 빌드 끝. 이제 ~/gosu/lab_visual/run_drive.sh 를 쓸 수 있다." \
              || echo "✗ 빠진 실행파일이 있다. 로그를 볼 것."
exit $(( ok == 1 ? 0 : 1 ))
