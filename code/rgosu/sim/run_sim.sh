#!/usr/bin/env bash
# 폐루프 코스 시뮬레이션 — 가상 로봇 + 실제 러너·ego_planner(Rgosu 패치판)·cmd_vel_to_sport·신호등 노드(onboard 판, ONNX).
# ★ 격리: 도메인 79 + lo 전용(~/ws/v_gosu/env_replay.sh). 실제 로봇(.161/.18)이 응답하면 실행하지 않는다(09-10 재생 섞임 사고).
# 사용: [SIM_DOMAIN=79] [EXIT_AT=start] ~/ws/Rgosu/sim/run_sim.sh [제한시간 초=420] [신호 주기 초=5] [벽 여유 inflate_radius=0.25]
#   EXIT_AT=start : 좌표 파일에 exit 이 없을 때 시뮬레이션의 exit 을 출발점으로(도달 가능한 끝점) — 비교 실험용
# 결과: ~/ws/Rgosu/sim/runs/sim_<시각>/ (events.jsonl · 노드 로그 · report.png · report.txt)
set -u
S=~/ws/Rgosu/sim; TMAX=${1:-420}; CYCLE=${2:-5}; INFL=${3:-0.25}; TS=$(date +%H%M%S); OUT=$S/runs/sim_${TS}_infl${INFL}; mkdir -p $OUT
for h in 161 18; do ping -c1 -W1 192.168.123.$h >/dev/null 2>&1 && { echo "★ 로봇(192.168.123.$h) 응답 — 가상 명령이 실제 로봇에 섞일 수 있어 실행하지 않는다. 로봇 전원이나 랜선을 빼고 다시."; exit 1; }; done
export V_DOMAIN=${SIM_DOMAIN:-79}; source ~/ws/v_gosu/env_replay.sh
case "$ROS_DOMAIN_ID" in 78|79|80|81) ;; *) echo "★ 도메인 $ROS_DOMAIN_ID — 격리 아님. 중단"; exit 1;; esac
set +u; source ~/ws/Rgosu/ws_ego_planner_tunable/install/setup.bash; set -u     # Rgosu 패치판 ego_planner 가 이기게 마지막에
echo "벽 여유 $INFL · exit=${EXIT_AT:-parkout} · 격리: 도메인 $ROS_DOMAIN_ID · $(echo "$CYCLONEDDS_URI" | grep -o 'NetworkInterface name="[a-z0-9]*"') · $ROS_AUTOMATIC_DISCOVERY_RANGE   결과 $OUT"
# 시뮬레이션 전용 좌표 사본 — 원본(maps/real_waypoints_venue.json)은 안 건드린다. 라벨만 러너 ORDER 에 맞추고, 없는 line·exit 은 parkin·parkout 으로 대신(시뮬레이션 가정)
# ★ 11-05: SIM_ROUTE=<route_v1 파일>(tools/route_editor.py 저장본)을 주면 그 경로를 그대로 쓴다(러너도 파일 순서대로 감)
if [ -n "${SIM_ROUTE:-}" ]; then cp "$SIM_ROUTE" "$OUT/waypoints_sim.json"; echo "경로 파일: $SIM_ROUTE"; else
EXIT_AT=${EXIT_AT:-parkout} python3 - "$OUT/waypoints_sim.json" <<'PY'
import json, sys
src = json.load(open('/home/ilovehan9283/ws/Rgosu/maps/real_waypoints_venue.json'))
ren = {'rabarin': 'rabain', 'rabarout': 'rabaout', 'park': 'parking', 'parkin': 'line', 'parkout': 'exit'}
out = [dict(w, label=ren.get(w['label'], w['label']), sim_from=w['label']) for w in src]
import os
if os.environ.get('EXIT_AT') == 'start':
    st = [w for w in out if w['label'] == 'start'][0]
    for w in out:
        if w['label'] == 'exit': w['x'], w['y'], w['sim_from'] = st['x'], st['y'], 'start(EXIT_AT=start)'
json.dump(out, open(sys.argv[1], 'w'), ensure_ascii=False, indent=1); print('시뮬레이션 좌표:', [(w['sim_from'], '→', w['label']) for w in out if w['sim_from'] != w['label']])
PY
fi
read SX SY < <(python3 -c "import json; d=json.load(open('$OUT/waypoints_sim.json')); d=d['route'] if isinstance(d, dict) else d; w=[x for x in d if x['label']=='start'][0]; print(w['x'], w['y'])")
PGS=()
run(){ local name=$1; shift; setsid "$@" > $OUT/$name.log 2>&1 < /dev/null & PGS+=($!); }
stop(){ for p in "${PGS[@]}"; do kill -INT -- -$p 2>/dev/null; done; sleep 3; for p in "${PGS[@]}"; do kill -TERM -- -$p 2>/dev/null; done; }
trap stop EXIT
# ★ 11-05('작전명 ego'): 기본값이 예전엔 sim_walls.npy(00:05, FAST-LIO max_range 고치기 전이라
# 벽이 번져있던 지도 기반)로 떨어졌음 -- SIM_WALLS를 안 주면 매번 그 낡은 지도로 돌아서 혼란
# 생길 뻔함(이번에 직접 겪음). sim_walls_fix1105.npy(02:09, 고친 뒤)가 있으면 그걸 우선 쓰게 바꿈.
WALLS=${SIM_WALLS:-$S/sim_walls_edited.npy}
[ -f "$WALLS" ] || WALLS=$S/sim_walls_fix1105.npy
[ -f "$WALLS" ] || WALLS=$S/sim_walls.npy   # map_editor.py 로 고친 지도가 있으면 그것
echo "$WALLS" > $OUT/walls_used.txt; echo "벽 지도: $WALLS"
run world ~/venv/go2/bin/python3 $S/sim_world_node.py --walls $WALLS --tl-dir $S/tl_images --log $OUT/events.jsonl --start $SX $SY 0.0 --cycle $CYCLE
sleep 3
# ★ 11-05 새벽('작전명 ego'): 구버전(cone_raw_filter_node, L1만·구간게이팅)을 여기 끼웠다가 stopline1
# 직후 길막힘을 보고 되돌렸었는데, 나중에 콘 노드 자체를 완전히 뺀(wall_cloud 직결) 상태로 같은
# 구간을 다시 여러 번 돌려보니 *그때도 똑같이, 실행마다 다른 위치에서* 막혀서 — 그 길막힘은 콘
# 연결과 무관한 ego_planner 자체의 비결정적 타이밍 문제였던 것으로 결론 정정함(README 참고).
# ★ 11-05 밤: obstacle_fusion_node(새 버전, 벽+L1+카메라콘, 콘 감지 항상 켜짐)로 교체해서 다시 연결.
# sim_world_node는 /utlidar/cloud_base·/mission/cone_cam을 안 내서(가짜 신호등 이미지만 냄, 콘
# 장면 없음) 콘 데이터 자체는 여전히 못 흉내내지만, 그런 입력이 전혀 없을 때는 wall_cloud를
# 그대로 통과시키는 구조라 최악의 경우에도 예전과 동일하게 동작 — 배선 자체의 안전성만 확인하는 용도.
run fuse python3 ~/ws/Rgosu/src/obstacle_fusion_node.py --ros-args -p use_leg_odom:=false
sleep 1
run ego ros2 run ego_planner ego_planner_node --ros-args -p odom_topic:=/Odometry -p cloud_topic:=/mission/obstacle_cloud \
  -p min_obs_h:=-5.0 -p max_obs_h:=5.0 -p inflate_radius:=$INFL -p safe_dist:=0.30
run cmdvel ros2 run go2_bridge cmd_vel_to_sport_node
run tl env TL_FAKE_ONNX=$HOME/ws/Rgosu/onboard_vision/tl5cone_384x640.onnx ~/venv/go2/bin/python3 ~/ws/Rgosu/onboard_vision/traffic_light_trt_node.py --ros-args -p engine_path:=onnx
sleep 4
run runner python3 ~/ws/Rgosu/src/real_course_runner.py $OUT/waypoints_sim.json
RUNNER_IDX=$((${#PGS[@]} - 1))   # 하드코딩(4) 대신 동적으로 구함 -- 나중에 노드를 더 끼워도 안 깨지게
echo "=== 시작 $(date +%T) — 최대 ${TMAX}초. 러너 로그: tail -f $OUT/runner.log"
for i in $(seq 1 $TMAX); do
  sleep 1
  grep -q "exit까지 전부 도착" $OUT/runner.log 2>/dev/null && { echo "■ 러너 완주 ($i 초)"; sleep 2; break; }
  kill -0 ${PGS[$RUNNER_IDX]} 2>/dev/null || { echo "★ 러너가 꺼졌다 — $OUT/runner.log"; break; }
  [ $((i % 30)) = 0 ] && echo "  $i 초 · $(grep -o '\[[a-z0-9]*\] [^ ]*' $OUT/runner.log | tail -1)"
done
stop; trap - EXIT
~/venv/go2/bin/python3 $S/sim_report.py $OUT 2>&1 | grep -vE "Warn|findfont"
