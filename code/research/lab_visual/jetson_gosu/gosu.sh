#!/usr/bin/env bash
# gosu — 문 하나. 환경은 이 스크립트가 알아서 고른다.
#
#   ./gosu.sh                    무엇을 할 수 있는지 보여준다
#   ./gosu.sh status             지금 상태 (로봇·젯슨·도는 노드·좌표)
#
#   ./gosu.sh run [--no-motor]   주행 스택 올리기      [로봇]
#      GO2_START_POSE="x,y,yaw"  초기위치를 알 때 — 탐색 400초를 건너뛴다
#      GO2_FAST=1                일치율 확인도 건너뛴다 (가장 빠름)
#   ./gosu.sh go <이름>          웨이포인트 하나로 주행  [로봇]
#   ./gosu.sh through <이름들>   여러 점을 **안 멈추고** 관통 [로봇]
#      뒤에 --go 를 붙여야 실제로 보낸다. 안 붙이면 계획만 보여준다
#
#   ── 신호등 (30초 주기, 시계로 판단한다) ──
#   ./gosu.sh signal start       출발 신호 순간에 실행 — 여기가 t=0
#   ./gosu.sh signal now         지금 어느 쪽이 초록이고 몇 초 남았나
#   ./gosu.sh signal route 1 3   1번에서 3번까지 가장 빨리 닿는 길과 시간표
#   ./gosu.sh cross 1-4 <이름들> 초록이 될 때까지 기다렸다가 건넌다  [로봇]
#
#   ── 미션 (자율주행: 한 번 실행하면 끝까지 혼자 돈다) ──
#   ./gosu.sh mission plan       계획과 예상 시간              [로봇 불필요]
#   ./gosu.sh mission dry        시계만 돌려 예행              [로봇 불필요]
#   ./gosu.sh mission go         **출발 신호에 이것 하나**      [로봇]
#   ./gosu.sh circuit            시계방향으로 계속 돈다 (권장)  [로봇]
#      계획 없이 2->1->4->3 을 반복. 어디서 끊겨도 다음 존으로 이어간다
#   ./gosu.sh zone <번호>        지금 로봇 자리를 그 존 목표로 기록 [로봇]
#      같은 번호로 다시 찍으면 덮어쓴다. 잘못 찍었으면 그냥 다시 찍으면 된다
#   ./gosu.sh marks              찍힌 좌표 보기            [로봇 불필요]
#   ./gosu.sh mark-start         지금 로봇 자리를 출발 위치로 기록   [로봇]
#      기록해 두면 ./gosu.sh run 이 그 자세로 바로 시작한다(400초 탐색 생략)
#      ./gosu.sh run --start-zone 2   등록된 존 2의 자세로 시작
#   ./gosu.sh overpass           지금 로봇 자리를 육교로 기록        [로봇]
#      2<->3 은 신호와 무관하게 언제나 이 지점을 거친다
#   ./gosu.sh pose               지금 좌표만 읽어본다 (등록이 안 될 때) [로봇]
#   ./gosu.sh goto <번호>        등록한 그 존 목표로 보낸다 (신호 무시)  [로봇]
#   ./gosu.sh reach [번호…]      안 움직이고 '경로가 나오나'만 물어본다  [로봇]
#
#   ── 진단 (좌표가 이상할 때 여기부터) ──
#   ./gosu.sh verify             지금 정합이 맞는지 한 번 본다          [로봇]
#      GO2_FAST=1 로 띄워 ⑤검증을 건너뛰었을 때 이걸로 대신 본다
#   ./gosu.sh register 2         [본선용] 존 좌표+기준 스캔을 한 번에 등록 [로봇]
#      ./gosu.sh register 3 --from 2 --dist 6.90    줄자 실측까지 같이
#      ./gosu.sh register check   등록 상태와 사각형 어긋남 점검
#      ./gosu.sh register fit     직사각형을 강제해 어긋남을 접는다(미리보기)
#      ./gosu.sh register fit --go   실제로 적용
#   ./gosu.sh handover           [긴급] 우리 것만 내리고 팀원 스택에 자리를 내준다
#      ./gosu.sh handover --check  내리지 않고 누가 무엇을 쥐고 있는지만 본다
#   ./gosu.sh penalty            [페널티] 모션 정지 + 신호 시계 멈춤    [로봇]
#   ./gosu.sh back 2             원위치에 놓았다 — 자세 재심기 + 시계 재개 [로봇]
#   ./gosu.sh signal pause|resume   시계만 멈추고/되돌리기
#   ./gosu.sh orders             주문서와 필요한 랩 수
#   ./gosu.sh teach 2            지금 자리의 스캔을 존 2의 기준으로 저장  [로봇]
#      그 뒤:  ./gosu.sh goto 2 --dock-mode scan
#   ./gosu.sh why                왜 멈췄나 — 멈출 수 있는 모든 경로를 한 번에
#      ./gosu.sh why 600          최근 10분   /  ./gosu.sh why follow  실시간
#   ./gosu.sh check              자세가 가만히 있는지 24초 지켜본다     [로봇]
#   ./gosu.sh costmap            로봇 자리와 존이 코스트맵에서 막혔나   [로봇]
#   ./gosu.sh tftree             TF 프레임 부모가 둘인지               [로봇]
#   ./gosu.sh stop               전부 내리기            [로봇]
#   ./gosu.sh mark <구역>        구역 네 모서리 찍기    [로봇]
#   ./gosu.sh crosswalks         존 좌표에서 횡단보도 4곳을 만들어 낸다 [로봇 불필요]
#      --width 2.0  폭(m)   --trim 1.0  존 쪽 끝을 자르는 길이(m)
#   ./gosu.sh keepout            횡단보도 밖을 못 가게 하는 Nav2 마스크 생성
#
#   ./gosu.sh venue              경기장 한 파일 보기 (지도·초기위치·좌표·규정)
#   ./gosu.sh map <이름>         매핑 -> 2D 변환 -> 등록까지 한 번에   [로봇]
#   ./gosu.sh map2d <pcd>        이미 있는 PCD 만 2D 로 바꾼다  [로봇 불필요]
#   ./gosu.sh wp                 웨이포인트 목록·검사
#   ./gosu.sh zones              저장된 구역 목록
#   ./gosu.sh replay <녹화본>    격리 재생 검증         [로봇 불필요]
#   ./gosu.sh view               화면 (브라우저)
#
# ── 왜 이 파일이 있나 ─────────────────────────────────────────────────────
# 실행이 여러 갈래로 나뉘고, 그때마다 어느 환경을 source 해야 하는지가 달랐다.
# 격리(도메인 77, lo 전용)와 주행(도메인 0, eno1)을 섞으면 조용히 망가진다 —
# 격리 환경으로는 로봇이 안 움직이고, 주행 환경으로 재생하면 로봇에 토픽이 나간다.
# **그 선택을 사람이 하지 않게** 하려고 만들었다. 명령마다 맞는 환경을 여기서 건다.
#
# ── 원본은 건드리지 않는다 ────────────────────────────────────────────────
# 주행은 검증된 원본(~/ws/go2_3d/go2_run.sh 등)을 그대로 부른다. 다시 만들지 않는다.
# ~/gosu/ws 는 **정리용 사본**이다. 아직 실행에 쓰지 않는다 (대회 뒤에 옮긴다).
set -u
GOSU=~/gosu
LAB="$GOSU/lab_visual"
C0=$'\033[0m'; CB=$'\033[1;36m'; CG=$'\033[1;32m'; CY=$'\033[1;33m'; CR=$'\033[1;31m'; CD=$'\033[2m'

usage() { sed -n '2,22p' "$0" | sed 's/^# \?//'; }

# 주행 환경: 도메인 0 + eno1. 로봇과 말한다.
env_drive() {
    set +u; source "$GOSU/env_drive.sh" >/dev/null 2>&1; set -u
    [ -n "${CYCLONEDDS_URI:-}" ] || { printf "${CR}★ eno1 링크가 없다 — 로봇에 못 붙는다${C0}\n"; return 1; }
}
# 격리 환경: 도메인 77 + lo 전용. 로봇 밖으로 한 바이트도 안 나간다.
env_lab() { set +u; source "$LAB/env.sh" >/dev/null 2>&1; set -u; }

cmd_status() {
    printf "${CB}══ 지금 상태 ══${C0}\n"
    printf "  젯슨 부하    %s\n" "$(uptime | sed 's/.*load average: //')"
    printf "  메모리       %s\n" "$(free -m | awk 'NR==2{printf "%d MB 사용 / %d MB", $3, $2}')"
    if ping -c1 -W1 192.168.123.18 >/dev/null 2>&1; then
        printf "  로봇         ${CG}응답${C0}\n"
    else
        printf "  로봇         ${CR}무응답${C0}  (랜선·전원 확인)\n"
    fi
    printf "  eno1 링크    %s\n" "$([ "$(cat /sys/class/net/eno1/carrier 2>/dev/null)" = 1 ] && echo 살아있음 || echo 없음)"
    echo
    printf "${CB}도는 노드${C0}\n"
    local mine others
    # pgrep -cf 는 없으면 "0" 을 찍고 **종료코드 1** 을 낸다. `|| echo 0` 을 붙이면
    # 0 이 두 줄이 된다 (go2_run.sh 의 H() 가 같은 이유로 망가져 있었다).
    mine=$(pgrep -cf "icp_localization|odometry_monitor|odometry_map_adapter|map_publisher" 2>/dev/null; true)
    mine=${mine:-0}
    others=$(pgrep -af "fastlio_mapping|amcl|bt_navigator|ego_planner|go2_mission|wall_runner|cmd_vel_to_sport|hesai_ros_driver" 2>/dev/null \
             | grep -v pgrep | head -6)
    printf "  gosu 것      %s개\n" "$mine"
    if [ -n "$others" ]; then
        printf "  그 밖에      ${CY}%s개${C0}\n" "$(echo "$others" | wc -l)"
        echo "$others" | sed 's/^/     /' | cut -c1-95
    else
        printf "  그 밖에      없음\n"
    fi
    echo
    printf "${CB}좌표${C0}\n"
    local w z
    w=$(python3 -c "import json,os;p=os.path.expanduser('$GOSU/config/waypoints.json');print(len([1 for v in json.load(open(p)).values() if isinstance(v,dict)]) if os.path.exists(p) else 0)" 2>/dev/null || echo '?')
    z=$(python3 -c "import json,os;p=os.path.expanduser('$GOSU/config/zones_box.json');print(len(json.load(open(p))) if os.path.exists(p) else 0)" 2>/dev/null || echo 0)
    printf "  웨이포인트   %s개\n" "$w"
    printf "  구역(박스)   %s개  %s\n" "$z" "$([ "$z" = 0 ] && printf "${CY}<- 횡단보도를 아직 안 찍었다${C0}")"
    echo
    printf "${CB}미션 준비${C0}\n"
    python3 - <<'PYEOF'
import json, os
V = os.path.expanduser('~/gosu/config/venue.json')
d = {}
if os.path.exists(V):
    try:
        d = json.load(open(V, encoding='utf-8'))
    except Exception:
        pass
CY = '\033[1;33m'; CG = '\033[1;32m'; C0 = '\033[0m'
sp = d.get('start_pose')
if sp:
    print('  출발 위치    %s기록됨%s  x %.2f  y %.2f  yaw %.0f도'
          % (CG, C0, sp['x'], sp['y'], sp.get('yaw_deg', 0)))
else:
    print('  출발 위치    %s없음%s  <- 로봇을 출발선에 세우고: ~/gosu/gosu.sh mark-start'
          % (CY, C0))
zp = d.get('zone_poses') or {}
have = sorted(zp)
miss = [z for z in '1234' if z not in zp]
if miss:
    print('  존 목표      %s%d/4%s  없는 곳 %s  <- 각 존에서: ~/gosu/gosu.sh zone <번호>'
          % (CY, len(have), C0, ', '.join(miss)))
else:
    print('  존 목표      %s4/4 다 찍혔다%s' % (CG, C0))
sg = (d.get('signals') or {})
t0 = sg.get('t0_epoch')
print('  신호 시계    %s' % ('맞춰짐' if t0 else
      '%s아직%s  <- 출발 신호에: ~/gosu/gosu.sh signal start' % (CY, C0)))
ready = bool(sp) and not miss
print()
print('  %s' % ('%s주행 준비 완료 — ~/gosu/gosu.sh mission go%s' % (CG, C0)
                if ready else '%s위 항목을 채워야 미션이 돈다%s' % (CY, C0)))
PYEOF
}

need_robot() {
    if ! ping -c1 -W1 192.168.123.18 >/dev/null 2>&1; then
        printf "${CR}★ 로봇이 응답하지 않는다. 전원과 랜선을 확인할 것${C0}\n"; exit 1
    fi
    local other
    other=$(pgrep -af "wall_runner|go2_mission_manager|ego_planner_node" 2>/dev/null | grep -v pgrep | head -3)
    if [ -n "$other" ]; then
        printf "${CY}★ 다른 스택이 로봇을 쓰고 있다:${C0}\n"
        echo "$other" | sed 's/^/   /' | cut -c1-90
        printf "   끝날 때까지 기다리거나, 확인 후 ./gosu.sh stop 으로 내릴 것\n"
        exit 1
    fi
}

case "${1:-}" in
    ''|-h|--help) usage ;;
    status) cmd_status ;;

    run)    shift; need_robot; env_drive || exit 1
            printf "${CD}환경: 도메인 0 + eno1 (로봇과 말한다)${C0}\n"
            # ── 어느 지도를 올릴지는 venue.json 이 정한다 ────────────────
            # 예전에는 MAP_YAML 환경변수가 정했는데 그걸 거는 스크립트가 없어서,
            # 새 셸에서 실행하면 launch 기본값(map_0910)이 조용히 올라갔다.
            # 2026-09-13 실제로 그 일이 났다 — 대회 전날 지도로 주행을 시도했다.
            # 사람이 기억하지 않게 파일에서 읽어 건다.
            if [ -z "${MAP_YAML:-}" ]; then
                _m=$(python3 - <<'PYEOF'
import json, os
V = os.path.expanduser('~/gosu/config/venue.json')
try:
    print((json.load(open(V, encoding='utf-8')).get('map_2d') or {}).get('yaml', ''))
except Exception:
    print('')
PYEOF
)
                if [ -n "$_m" ]; then
                    export MAP_YAML="$HOME/gosu/maps/$_m"
                    printf "${CD}지도: %s (venue.json)${C0}\n" "$_m"
                else
                    printf "${CY}★ venue.json 에 지도가 없다 — launch 기본값이 올라간다${C0}\n"
                fi
            else
                printf "${CD}지도: {'PY': 'PYEOF'} (MAP_YAML 직접 지정)${C0}\n" "$MAP_YAML"
            fi
            # ── 출발 자세도 venue.json 이 정한다 ──────────────────────────
            # 지도와 **같은 문제**가 있었다. mark-start 로 찍어 venue.json 에
            # start_pose 를 넣어 두는데, go2_run.sh 는 그 파일을 안 읽는다.
            # 그래서 찍어놓고도 숫자를 손으로 GO2_START_POSE 에 옮겨야 했고,
            # 안 옮기면 auto_initialpose 가 **최대 400초** 격자탐색을 한다.
            # 경기 당일에 400초는 없다. 사람이 기억하지 않게 파일에서 읽어 건다.
            #
            #   ./gosu.sh run                  venue.json 의 start_pose 를 쓴다
            #   ./gosu.sh run --start-zone 2   그 존의 등록 자세로 시작한다
            #   GO2_START_POSE="x,y,도" ./gosu.sh run   직접 지정(이게 이긴다)
            _sz=""
            case "${1:-}" in
                --start-zone) _sz="${2:-}"; shift 2 ;;
            esac
            if [ -z "${GO2_START_POSE:-}" ]; then
                _sp=$(SZ="$_sz" python3 - <<'PYEOF'
import json, os
V = os.path.expanduser('~/gosu/config/venue.json')
sz = os.environ.get('SZ', '')
try:
    d = json.load(open(V, encoding='utf-8'))
except Exception:
    d = {}
p, src = None, ''
if sz:
    p = (d.get('zone_poses') or {}).get(str(sz))
    src = '존 %s' % sz
if p is None:
    p = d.get('start_pose')
    src = 'start_pose'
if p:
    print('%.4f,%.4f,%.2f|%s' % (p['x'], p['y'], p.get('yaw_deg', 0.0), src))
else:
    print('')
PYEOF
)
                if [ -n "$_sp" ]; then
                    export GO2_START_POSE="${_sp%%|*}"
                    printf "${CD}출발 자세: %s (%s)${C0}\n" \
                        "$GO2_START_POSE" "${_sp##*|}"
                else
                    printf "${CY}★ 출발 자세가 없다 — auto_initialpose 가 최대 400초 탐색한다${C0}\n"
                    printf "${CD}   로봇을 출발선에 세우고: ./gosu.sh mark-start${C0}\n"
                    printf "${CD}   또는 등록된 존에서 시작: ./gosu.sh run --start-zone 2${C0}\n"
                fi
            else
                printf "${CD}출발 자세: %s (직접 지정)${C0}\n" "$GO2_START_POSE"
            fi
            exec ~/ws/go2_3d/go2_run.sh "$@" ;;

    go)     [ -n "${2:-}" ] || { printf "${CR}웨이포인트 이름이 필요하다. ./gosu.sh wp 로 목록${C0}\n"; exit 1; }
            env_drive || exit 1
            exec python3 ~/ws/go2_ctrl/waypoint.py go "$2" ;;

    # 경유점을 한 번에 보내 중간에서 멈추지 않는다. Nav2 공식 액션
    # navigate_through_poses 를 쓴다 — bt_navigator 가 이미 물고 있어 스택은 안 고친다.
    # 목표 하나씩 보내면 경유점마다 감속·정지·재가속 2~4초 + 사람 대기 5~15초를 버린다.
    through)
            shift; [ -n "${1:-}" ] || { printf "${CR}경유점 이름을 두 개 이상 줄 것. ./gosu.sh wp 로 목록${C0}\n"; exit 1; }
            env_drive || exit 1
            exec python3 "$LAB/tools/waypoints.py" through "$@" ;;

    # 긴급 교환 — 우리 것만 내리고 팀원 스택에 자리를 내준다
    handover) shift; exec bash "$(dirname "$0")/handover.sh" "$@" ;;

    stop)   env_drive || true
            printf "${CB}내린다${C0}\n"
            [ -x ~/ws/go2_3d/go2_stop.sh ] && exec ~/ws/go2_3d/go2_stop.sh
            pkill -INT -f 'fastlio_mapping|amcl|bt_navigator|controller_server|cmd_vel_to_sport|hesai_ros_driver|map_server|loc_watchdog' 2>/dev/null
            sleep 2; local left; left=$(pgrep -cf 'fastlio_mapping|amcl|bt_navigator' 2>/dev/null; true)
            printf "  남은 노드 %s개\n" "${left:-0}" ;;

    mark)   [ -n "${2:-}" ] || { printf "${CR}구역 이름이 필요하다${C0}\n"; exit 1; }
            env_drive || exit 1
            exec python3 "$LAB/tools/zones.py" mark "$2" ;;

    wp)     shift; env_drive >/dev/null 2>&1 || env_lab >/dev/null 2>&1
            exec python3 "$LAB/tools/waypoints.py" list "$@" ;;

    zones)  exec python3 "$LAB/tools/zones.py" list ;;

    # 존 네 곳에서 횡단보도 사각형을 유도한다. 16번 걸어가 찍는 대신 한 줄.
    crosswalks) shift; exec python3 "$LAB/tools/zones.py" from-zones "$@" ;;

    keepout)    shift; exec python3 "$LAB/tools/zones.py" mask "$@" ;;

    marks)  exec python3 "$LAB/tools/mission.py" zones ;;

    signal) shift; exec python3 "$LAB/tools/signals.py" "${1:-now}" "${@:2}" ;;

    # 자율주행 대회다 — 사람은 출발할 때와 긴급점검 때만 건드린다.
    # 그래서 구간마다 명령을 치지 않고, 이것 하나로 끝까지 돈다.
    mission)
            shift; sub="${1:-plan}"; shift || true
            case "$sub" in
                go)  need_robot; env_drive || exit 1
                     exec python3 "$LAB/tools/mission.py" run --go "$@" ;;
                run) need_robot; env_drive || exit 1
                     exec python3 "$LAB/tools/mission.py" run "$@" ;;
                *)   exec python3 "$LAB/tools/mission.py" "$sub" "$@" ;;
            esac ;;

    pose)   env_drive || exit 1
            exec python3 "$LAB/tools/mission.py" pose ;;

    # ── 진단 ──────────────────────────────────────────────────────────
    # 좌표가 튀면 원인이 셋 중 하나다: 감시자의 재정위 / 코스트맵이 막음 / TF 트리.
    # 순서대로 가린다.
    # 스캔이 지도와 맞는지 한 번 본다. go2_run.sh 의 ⑤검증과 같은 것을 부른다.
    # 볼 것은 '상대 판정' 줄이다 — 절대 일치율은 걷거나 앉으면 정상적으로 떨어진다.
    why)    shift
            if [ "${1:-}" = "follow" ]; then
                exec python3 "$LAB/tools/why_stopped.py" --follow
            fi
            exec python3 "$LAB/tools/why_stopped.py" --sec "${1:-120}" ;;

    verify) shift; env_drive || exit 1
            printf "${CD}스캔과 지도를 대조한다 (최대 45초)${C0}\n"
            timeout 45 python3 ~/ws/go2_ctrl/check_localization.py "$@" 2>&1 \
              | grep -viE 'userwarning|warnings.warn' \
              | grep -E '일치율|상대 판정|=>|허용오차|지도 수신'
            exit 0 ;;

    check)  shift; env_drive || exit 1
            printf "${CD}가만히 둔 채로 지켜본다. 이 동안 로봇을 건드리지 말 것${C0}\n"
            exec python3 "$LAB/tools/diag_tfwatch.py" "${1:-24}" ;;
    costmap) env_drive || exit 1
            exec python3 "$LAB/tools/diag_cm.py" ;;
    tftree) env_drive || exit 1
            exec python3 "$LAB/tools/diag_tftree.py" ;;

    # 등록한 존 목표로 보낸다. 신호는 안 본다 — 준비·시험용이다.
    goto)   [ -n "${2:-}" ] || { printf "${CR}존 번호가 필요하다 (1~4)${C0}\n"; exit 1; }
            need_robot; env_drive || exit 1
            exec python3 "$LAB/tools/mission.py" goto "$2" ;;

    # 보내기 전에 계획만 시켜 본다. 헛걸음 한 번(회복 약 17초)을 아낀다.
    reach)  shift; env_drive || exit 1
            exec python3 "$LAB/tools/mission.py" reach "$@" ;;

    orders) exec python3 "$LAB/tools/mission.py" orders ;;

    # 횡단보도 페널티 — 모션만 세우고 신호 시계를 멈춘다 (스택은 안 내린다)
    penalty) env_drive || exit 1
            exec python3 "$LAB/tools/mission.py" penalty ;;

    # 원위치에 놓은 뒤 — 자세를 다시 심고 시계를 되돌린다
    back)   [ -n "${2:-}" ] || { printf "${CR}어느 존에 놓았는지 줄 것 (1~4)${C0}\n"; exit 1; }
            env_drive || exit 1
            exec python3 "$LAB/tools/mission.py" back "$2" ;;

    # 본선 당일 등록 — 존 좌표 + 기준 스캔 + 줄자 실측을 한 번에
    register)
            shift
            case "${1:-check}" in
                check|fit) exec python3 "$LAB/tools/mission.py" register "$@" ;;
                *) env_drive || exit 1
                   exec python3 "$LAB/tools/mission.py" register "$@" ;;
            esac ;;

    teach)  [ -n "${2:-}" ] || { printf "${CR}존 번호가 필요하다 (1~4)${C0}\n"; exit 1; }
            env_drive || exit 1
            exec python3 "$LAB/tools/mission.py" teach "$2" ;;

    overpass)
            env_drive || exit 1
            exec python3 "$LAB/tools/mission.py" mark-overpass ;;

    mark-start)
            env_drive || exit 1
            exec python3 "$LAB/tools/venue.py" set-start --from-robot ;;

    circuit) shift; need_robot; env_drive || exit 1
            exec python3 "$LAB/tools/mission.py" circuit --go "$@" ;;

    zone)   [ -n "${2:-}" ] || { printf "${CR}존 번호가 필요하다 (1~4)${C0}\n"; exit 1; }
            env_drive || exit 1
            exec python3 "$LAB/tools/mission.py" mark-zone "$2" ;;

    # 초록이 될 때까지 기다렸다가 건넌다. 기다림은 ROS 밖에서 한다 —
    # 시계를 하나만 쓰기 위해서다(ROS 시간·sim time 을 섞으면 어느 쪽이 맞는지 모르게 된다).
    cross)  shift
            [ -n "${1:-}" ] || { printf "${CR}횡단보도와 경유점이 필요하다: ./gosu.sh cross 1-4 Z1_2 Z4_1${C0}\n"; exit 1; }
            LEG="$1"; shift
            [ -n "${1:-}" ] || { printf "${CR}건너갈 경유점을 줄 것${C0}\n"; exit 1; }
            need_robot
            python3 "$LAB/tools/signals.py" wait "$LEG" || {
                printf "${CY}건너지 않는다${C0}\n"; exit 1; }
            env_drive || exit 1
            exec python3 "$LAB/tools/waypoints.py" through "$@" --go ;;

    venue)  shift; exec python3 "$LAB/tools/venue.py" "${1:-show}" "${2:-}" "${@:3}" ;;

    # 매핑 -> 2D 변환 -> venue.json 등록까지 한 번에.
    # go2_map.sh 는 PCD 까지만 만들고 "잘라서 변환하세요" 하고 끝난다.
    # 그 사이에서 2026-09-13 에 반나절을 썼다(기울기 보정 누락, 경기장 밖 포함,
    # 성긴 원본). 그 뒷부분을 make_map.py 가 대신한다.
    map)    [ -n "${2:-}" ] || { printf "${CR}지도 이름이 필요하다: ./gosu.sh map 대회장${C0}\n"; exit 1; }
            need_robot; env_drive || exit 1
            printf "${CB}매핑${C0}\n"
            printf "${CY}  ★ 매핑 전 확인${C0}\n"
            printf "    · 로봇을 **세운 뒤** 시작한다 (앉아서 띄우면 중력정렬이 틀어진다)\n"
            printf "    · 옮겨질 물건(보행자 박스 등)은 치우고 찍는다\n"
            printf "    · **5분 이내**. 스캔이 전부 RAM 에 쌓인다 (예전 2.8GB OOM)\n"
            printf "    · 끝낼 때 Ctrl-C 한 번. 저장에 40초 걸린다. 두 번 누르지 말 것\n\n"
            ~/ws/go2_3d/go2_map.sh --name "$2" || true
            src=~/ws/go2_3d/maps/"$2".pcd
            if [ ! -f "$src" ]; then
                printf "${CR}★ %s 가 없다 — 지도가 저장되지 않았다${C0}\n" "$src"
                printf "   /tmp/go2_map/lio.log 를 볼 것\n"
                exit 1
            fi
            printf "\n${CB}2D 로 바꾸고 등록한다${C0}\n"
            exec python3 "$LAB/tools/make_map.py" "$src" --name "$2" ;;

    # 이미 있는 PCD 를 2D 로 바꿀 때 (매핑을 다시 하지 않고)
    map2d)  [ -n "${2:-}" ] || { printf "${CR}PCD 경로가 필요하다${C0}\n"; ls ~/ws/go2_3d/maps/*.pcd ~/gosu/maps/*.pcd 2>/dev/null | sed 's/^/   /'; exit 1; }
            shift; exec python3 "$LAB/tools/make_map.py" "$@" ;;

    replay) [ -n "${2:-}" ] || { printf "${CR}녹화본 경로가 필요하다${C0}\n"; ls -d ~/go2_ws/bags/*/ 2>/dev/null | sed 's/^/   /'; exit 1; }
            printf "${CD}환경: 도메인 77 + lo 전용 (로봇에 한 바이트도 안 나간다)${C0}\n"
            exec "$LAB/run_replay.sh" "$2" "${3:-120}" ;;

    view)   shift; exec "$LAB/run_live_view.sh" "$@" ;;

    *) printf "${CR}모르는 명령: %s${C0}\n\n" "$1"; usage; exit 1 ;;
esac
