#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════════
#  긴급 교환      우리 스택을 비키고 팀원 스택에 자리를 내준다
# ═══════════════════════════════════════════════════════════════════════════
#
#   ./handover.sh           우리 것만 내리고, 자리가 비었는지 확인한다
#   ./handover.sh --check   내리지 않고 지금 누가 무엇을 쥐고 있는지만 본다
#
# 【왜 gosu.sh stop 으로 하면 안 되나】
# go2_stop.sh 는 이름 패턴으로 죽인다 — hesai_ros_driver, cloud_bridge,
# imu_bridge, cmd_vel_to_sport 는 **팀원 스택도 쓰는 이름**이다.
# 교환 상황에서 팀원이 먼저 켰다면 그걸 같이 죽인다. 정확히 반대로 동작한다.
#
# 여기서는 `GO2_STACK_OWNER=go2_run` 표식이 붙은 것만 내린다.
# 표식이 없는 것은 남의 것이거나 표식 이전에 뜬 잔재다 — 죽이지 않고 보고만 한다.
#
# 【자리가 비었다는 것의 기준】
# 라이다 UDP 2368 을 아무도 안 듣고 있어야 한다. 이게 가장 중요하다 —
# 2026-09-13 에 드라이버 두 개가 같은 포트를 물어 커널이 패킷을 반씩 나눠주고,
# 반쪽 스캔 두 줄기가 섞여 FAST-LIO 가 48m 발산했다. 포트가 안 비면 교환해도
# 팀원 스택이 똑같이 무너진다.
set -u
CG='\033[32m'; CR='\033[31m'; CY='\033[33m'; CB='\033[1m'; CD='\033[2m'; C0='\033[0m'

CHECK=0
[ "${1:-}" = "--check" ] && CHECK=1

PATS='hesai_ros_driver|cloud_bridge|imu_bridge|nav_bridge|fastlio_mapping'
PATS="$PATS|laser_mapping|pointcloud_to_laserscan|cmd_vel_to_sport|loc_watchdog"
PATS="$PATS|amcl|map_server|controller_server|planner_server|bt_navigator"
PATS="$PATS|behavior_server|lifecycle_manager|foxglove_bridge|zone_markers|odom_tf"
COMM_PATS='^(hesai_ros_driv|fastlio_mapping|laser_mapping|cloud_bridge|imu_bridge'
COMM_PATS="$COMM_PATS|nav_bridge|cmd_vel_to_spor|pointcloud_to_l|amcl|map_server"
COMM_PATS="$COMM_PATS|controller_serv|planner_server|bt_navigator|behavior_serve"
COMM_PATS="$COMM_PATS|lifecycle_mana|foxglove_bridg|odom_tf|python3)"

is_ours() { grep -qz 'GO2_STACK_OWNER=go2_run' "/proc/$1/environ" 2>/dev/null; }

is_node() {
    ps -o comm= -p "$1" 2>/dev/null | grep -qE "$COMM_PATS" || return 1
    # python3 로 뜬 노드는 argv[1] 로 가린다 (grep 한 줄을 노드로 오인하지 않게)
    if [ "$(ps -o comm= -p "$1" 2>/dev/null)" = "python3" ]; then
        local a1; a1=$(tr '\0' '\n' < "/proc/$1/cmdline" 2>/dev/null | sed -n 2p)
        [ -n "$a1" ] || return 1
        basename "$a1" 2>/dev/null | grep -qE 'cmd_vel_to_sport|loc_watchdog|zone_markers|robot_marker' || return 1
    fi
    return 0
}

scan_procs() {   # 출력: PID<TAB>ours|other<TAB>comm<TAB>cmdline
    local p
    for p in $(pgrep -f "$PATS" 2>/dev/null); do
        [ "$p" = "$$" ] && continue
        is_node "$p" || continue
        printf '%s\t%s\t%s\t%s\n' "$p" \
            "$(is_ours "$p" && echo ours || echo other)" \
            "$(ps -o comm= -p "$p" 2>/dev/null)" \
            "$(tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null | head -c 90)"
    done
}

port_owners() { ss -ulnp 2>/dev/null | grep -c ':2368' || true; }

report() {
    local rows; rows=$(scan_procs)
    local no_ours no_other
    no_ours=$(printf '%s\n' "$rows" | grep -c "$(printf '\tours\t')" || true)
    no_other=$(printf '%s\n' "$rows" | grep -c "$(printf '\tother\t')" || true)
    printf "  ${CB}지금 떠 있는 노드${C0}\n"
    if [ -z "$rows" ]; then
        printf "    없음\n"
    else
        printf '%s\n' "$rows" | while IFS=$'\t' read -r p own comm cmd; do
            [ -z "$p" ] && continue
            if [ "$own" = ours ]; then
                printf "    ${CG}우리${C0}  PID %-7s %-16s %s\n" "$p" "$comm" "$cmd"
            else
                printf "    ${CY}남의것/잔재${C0} PID %-7s %-16s %s\n" "$p" "$comm" "$cmd"
                printf "            ${CD}부모: %s${C0}\n" \
                    "$(tr '\0' ' ' < /proc/"$(ps -o ppid= -p "$p" 2>/dev/null | tr -d ' ')"/cmdline 2>/dev/null | head -c 90)"
            fi
        done
    fi
    printf "    ${CD}우리 %s개, 우리가 아닌 것 %s개${C0}\n" "${no_ours:-0}" "${no_other:-0}"
    printf "\n  ${CB}라이다 UDP 2368${C0}\n"
    local n; n=$(port_owners)
    if [ "${n:-0}" -eq 0 ]; then
        printf "    ${CG}비어 있음 — 팀원 스택이 물 수 있다${C0}\n"
    else
        printf "    ${CR}%s 개가 물고 있다${C0}\n" "$n"
        ss -ulnp 2>/dev/null | grep ':2368' | sed 's/^/      /'
    fi
}

printf "${CB}── 긴급 교환 ──${C0}\n\n"
report

if [ "$CHECK" = 1 ]; then
    printf "\n  ${CD}--check 라 아무것도 내리지 않았다${C0}\n"
    exit 0
fi

# ── 우리 것만 내린다 ──────────────────────────────────────────────────
OURS=$(scan_procs | grep "$(printf '\tours\t')" | cut -f1 || true)
# 우리 실행 스크립트를 **먼저** 세운다. 노드를 먼저 죽이면 지킴이가 되살린다.
RUN=$(pgrep -f 'go2_run\.sh' 2>/dev/null | while read -r p; do is_ours "$p" && echo "$p"; done)
printf "\n  ${CB}우리 것을 내린다${C0}\n"
if [ -n "$RUN" ]; then
    printf "    실행 스크립트 %s 에 INT\n" "$(echo $RUN | tr '\n' ' ')"
    kill -INT $RUN 2>/dev/null || true
    sleep 4
fi
OURS=$(scan_procs | grep "$(printf '\tours\t')" | cut -f1 || true)
if [ -n "$OURS" ]; then
    printf "    남은 노드 %s 에 INT\n" "$(echo $OURS | tr '\n' ' ')"
    kill -INT $OURS 2>/dev/null || true
    sleep 3
    OURS=$(scan_procs | grep "$(printf '\tours\t')" | cut -f1 || true)
    [ -n "$OURS" ] && { printf "    아직 남음 — KILL\n"; kill -9 $OURS 2>/dev/null || true; sleep 1; }
fi

printf "\n"
report

# ── 판정 ──────────────────────────────────────────────────────────────
printf "\n"
LEFT_OURS=$(scan_procs | grep -c "$(printf '\tours\t')" || true)
LEFT_OTHER=$(scan_procs | grep -c "$(printf '\tother\t')" || true)
PORT=$(port_owners)
if [ "${LEFT_OURS:-0}" -eq 0 ] && [ "${PORT:-0}" -eq 0 ]; then
    printf "  ${CG}${CB}자리가 비었다 — 팀원 스택을 켜도 된다${C0}\n"
    printf "    ${CD}2026-09-13 에 관측된 팀원 기동 명령:${C0}\n"
    printf "    ros2 launch go2_wall_patrol patrol.launch.py\n"
elif [ "${LEFT_OTHER:-0}" -gt 0 ] && [ "${LEFT_OURS:-0}" -eq 0 ]; then
    printf "  ${CY}우리 것은 다 내렸다. 남은 것은 우리 표식이 없다${C0}\n"
    printf "    남의 것이면 그대로 두면 된다(이미 팀원이 켠 것일 수 있다).\n"
    printf "    표식 이전 실행의 잔재라면 그 터미널에서 Ctrl-C 하거나:\n"
    printf "    ${CB}~/ws/go2_3d/go2_stop.sh${C0}  ${CD}(주의: 이름으로 죽인다 — 팀원 것도 죽는다)${C0}\n"
else
    printf "  ${CR}아직 자리가 안 비었다${C0}  우리 노드 %s개, 포트 점유 %s\n" \
        "${LEFT_OURS:-0}" "${PORT:-0}"
    printf "    다시 실행해 보고, 그래도 남으면 위 go2_stop.sh 를 쓴다.\n"
fi
