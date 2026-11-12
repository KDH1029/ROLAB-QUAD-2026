#!/usr/bin/env bash
# 슬라럼 디버깅 실행 — junk 대회 코드(nail-1105)는 안 건드림. 부품(라이다·브리지·FAST-LIO·모터)·비상정지(x)는 junk 관리자 그대로.
#   순서: junk_ctl 에서 a → 로봇을 #6 에 #7 쪽을 보게 놓기 → 3(FAST-LIO 다시 켜기: 여기가 원점) → 기립 → m → 이 스크립트
#   사용: ~/ws/junk/debug/slalom/run.sh [시작번호=6] [끝번호=25]      조정값: SLALOM_V=0.25 SLALOM_LOOK=0.35 SLALOM_WP_R=0.25 SLALOM_TURN=0.6 SLALOM_HEADING=1(몸 방향=경로, 0=예전)
#   끝나면(끝 점 도착 or Ctrl+C) 자동으로 그림: runs/<시각>/slalom.png
set +u
D="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"; J="$(cd "$D/../.." && pwd)"
A=${1:-6}; C=${2:-25}; NAME=${SLALOM_ROUTE:-rec1650}
source ~/setup_go2_ros2.sh >/dev/null 2>&1
export CYCLONEDDS_URI=file://$J/config/cyclonedds_junsu.xml
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash >/dev/null 2>&1
source ~/ws/merge_tmp_20261102/review/install/setup.bash >/dev/null 2>&1
source $J/ws_fastlio_tunable/install/setup.bash >/dev/null 2>&1
RUN=$D/runs/$(date +%m%d_%H%M%S); mkdir -p $RUN
stopmove() { timeout 5 ros2 topic pub --times 3 -r 20 /api/sport/request unitree_api/msg/Request "{header: {identity: {id: 1, api_id: 1003}}, parameter: '{}'}" >/dev/null 2>&1; }
pgrep -f 'real_course_runne[r].py' >/dev/null && { echo "★ 다른 러너가 돌고 있음 — junk 관리자에서 x 로 끄고 다시"; exit 1; }
PRIOR=$(python3 $D/make_route.py $NAME $RUN/route.json $A $C) || exit 1
echo "출발 정렬(로봇 가만히, 약 5초) — 기준 $PRIOR"
python3 $D/start_align/go2_start_align.py $RUN/route.json $RUN/route_today.json --map $J/routes/$NAME/map.yaml --prior "$PRIOR" < /dev/null > $RUN/align.log 2>&1
if [ $? -ne 0 ]; then
  echo "★ 출발 정렬 실패 — 로봇이 #$A 자리·방향에 있는지, FAST-LIO 를 거기서 다시 켰는지(3) 확인"; tail -3 $RUN/align.log; exit 1
fi
python3 -c "import json;d=json.load(open('$RUN/route_today.json.align.json'));print('정렬 OK  T',[round(v,3) for v in d['T']],' 일치',round(d['match'],2))"
trap 'echo; echo "Ctrl+C — 정지"; stopmove' INT
echo "출발 — 슬라럼 #$A→#$C (Ctrl+C 또는 junk 관리자 x = 정지)"
RUNNER_PATH_ONLY=1 RUNNER_TL=0 SLALOM_TRACE=$RUN/trace.csv \
RUNNER_PIN_V=${SLALOM_V:-0.25} RUNNER_PIN_LOOKAHEAD=${SLALOM_LOOK:-0.35} RUNNER_PIN_WP_R=${SLALOM_WP_R:-0.25} RUNNER_FOLLOW_TURN_FIRST=${SLALOM_TURN:-0.6} RUNNER_HEADING_TANGENT=${SLALOM_HEADING:-1} \
  python3 $D/real_course_runner.py $RUN/route_today.json < /dev/null 2>&1 | sed -u 's/\x1b\[[0-9;]*m//g' | grep --line-buffered -v '^\[INFO\]' | tee $RUN/runner.log
stopmove; trap - INT
echo "V=${SLALOM_V:-0.25} LOOK=${SLALOM_LOOK:-0.35} WP_R=${SLALOM_WP_R:-0.25} TURN=${SLALOM_TURN:-0.6} HEADING=${SLALOM_HEADING:-1}" > $RUN/params.txt
~/venv/go2/bin/python3 $D/plot.py $RUN
