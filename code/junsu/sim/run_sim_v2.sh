#!/usr/bin/env bash
# junsu_v2 실제 스택 폐루프 시뮬(11-06): 가상 세계(sim_world_v2) + 합치기 + review ego + 몸 모양 안전 필터 + cmd_vel_to_sport(쏠림 보정) + v2 러너.
# 사용: run_sim_v2.sh <이름> <도메인 78~81> [경로 파일]   — 조건은 환경변수로:
#   CONE=det|walls|none  MISS=0.2  CNOISE=0.05  STEP=0.10  WZTHR=0.12  NOISE=0  SEED=0  START="x y yaw"(경로 출발점에서 더할 오차)
#   MAXV=0.5  SAFE=0.50(벽 밀어내기 시작 거리)  LOBS=250(벽 페널티)  CSAFE=(콘 밀어내기, 기본 yaml 0.38)  CINFL=(콘 단단한 여유, 기본 0.28)
#   PASS=0.8  SKIP=25  GUARD=on|off  TMAX=360  EXIT_X=0.70(새 코스 출구 선 가정)
# ★ 11-06 xentry: 가상 라이다 벽 기본값 = 외톨이 칸 뺀 벽(sim_walls_rec1650_noiso.npy). 옛 벽(1.98,0.10 외톨이 칸 포함)은 WALLS=$S/sim_walls_rec1650.npy
#   외톨이 칸 하나가 X 통로 입구(벽 끝과 0.396 m)를 ego 팽창 0.20 m 격자에서 막아 북쪽 막다른 통로로 빠지게 했다(debug/agents/xentry/REPORT.md).
# 결과: sim/runs_v2/<이름>/ (world.jsonl · 노드 로그 · report.txt). ★ 격리: 도메인 78~81 + lo 전용, 로봇이 응답하면 거부.
set +u
NAME=$1; DOM=$2; S="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"; J="$(cd "$S/.." && pwd)"; ROUTE=${3:-$J/maps/route_rec1650_v2.json}
# ★ STACK=main → 11-05 16:54 실주행 코드(~/ws/junsu_v2_bak = junsu a438045): 그 관리자와 같은 ego 인자·몸 모양 필터 없음·러너 환경
if [ "${STACK:-v2}" = main ]; then J=$HOME/ws/junsu_v2_bak; GUARD=off; fi
for h in 161 18; do ping -c1 -W1 192.168.123.$h >/dev/null 2>&1 && { echo "★ 로봇(192.168.123.$h) 응답 — 거부"; exit 1; }; done
if ip link show robot-br 2>/dev/null | grep -q 'state UP'; then echo "★ 로봇 연결됨 — 거부"; exit 1; fi
export V_DOMAIN=$DOM; source ~/ws/v_gosu/env_replay.sh
case "$ROS_DOMAIN_ID" in 78|79|80|81) ;; *) echo "★ 도메인"; exit 1;; esac
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash >/dev/null 2>&1; source ~/ws/merge_tmp_20261102/review/install/setup.bash >/dev/null 2>&1
O=$S/runs_v2/$NAME; rm -rf $O; mkdir -p $O; cp $ROUTE $O/route.json
read SX SY SA < <(python3 -c "
import json, math; d = json.load(open('$ROUTE'))['route']; s = d[0]; dx, dy, da = [float(v) for v in '${START:-0 0 0}'.split()]
print(s['x'] + dx, s['y'] + dy, math.radians(s.get('yaw', 0.0)) + math.radians(da))")
BP=~/ws/merge_tmp_20261102/review/install/go2_bridge/share/go2_bridge/config/bridge_params.yaml; EGO=$J/ws_ego_review
PGS=(); run(){ local n=$1; shift; setsid "$@" > $O/$n.log 2>&1 < /dev/null & PGS+=($!); }
stop(){ for p in "${PGS[@]}"; do kill -INT -- -$p 2>/dev/null; done; sleep 2; for p in "${PGS[@]}"; do kill -TERM -- -$p 2>/dev/null; done; }; trap stop EXIT
run world python3 $S/sim_world_v2.py --walls ${WALLS:-$S/sim_walls_rec1650_noiso.npy} --obc $S/sim_obc_rec1650.npy --cones ${CONES:-$S/sim_cones_rec1650.npy} --log $O/world.jsonl \
  --start $SX $SY $SA --cone-mode ${CONE:-det} --cone-miss ${MISS:-0.2} --cone-noise ${CNOISE:-0.05} --step-thr ${STEP:-0.10} --wz-thr ${WZTHR:-0.12} --noise ${NOISE:-0} --seed ${SEED:-0}
sleep 2
run fuse python3 $J/src/obstacle_fusion_node.py --ros-args -p use_l1:=false -p use_cam:=false -p cones_topic:="''" -p lidar_cones_topic:="''"
EGOARGS="-p obstacle_source:=scan -p cones.topic:=/mission/cones -p max_vel:=${MAXV:-0.5} -p optimizer.lambda_obs:=${LOBS:-250.0} -p safe_dist:=${SAFE:-0.50} -p extra_obstacle_topic:=/mission/wall_memory -p control.rotate_in_place_rad:=1.8"
[ "${STACK:-v2}" = main -o -n "${EGO_LIKE_MAIN:-}" ] && EGOARGS="-p obstacle_source:=scan -p cones.topic:=/mission/cones -p max_vel:=${MAXV:-0.5}"   # 16:54 관리자 ego 인자 그대로(팀원 yaml 기본)
if [ -n "${PRIOR:-}" ]; then printf "ego_planner_node:\n  ros__parameters:\n    use_prior_map: true\n    static_map_path: \"%s\"\n    prior_map:\n      frame: \"\"\n" "$PRIOR" > $O/ego_prior.yaml; EGOARGS="$EGOARGS --params-file $O/ego_prior.yaml"; fi; [ -n "$INFL" ] && EGOARGS="$EGOARGS -p inflate_radius:=$INFL"; [ -n "$CSAFE" ] && EGOARGS="$EGOARGS -p cones.safe_dist:=$CSAFE"; [ -n "$CINFL" ] && EGOARGS="$EGOARGS -p cones.inflate_radius:=$CINFL"
run ego bash -c "source $EGO/install/setup.bash && exec ros2 run ego_planner ego_planner_node --ros-args --params-file $EGO/src/ego_planner/config/ego_planner_params.yaml $EGOARGS"
if [ "${GUARD:-on}" = on ]; then run guard python3 $J/src/footprint_guard_node.py --ros-args ${GUARD_ARGS:-}; CMDT=/cmd_vel_safe; else CMDT=/cmd_vel; fi
run motor ros2 run go2_bridge cmd_vel_to_sport_node --ros-args --params-file $BP -p cmd_vel_topic:=$CMDT -p yaw_trim_per_vx:=0.0156 -p yaw_trim_bias:=0.0397
sleep 4
for n in /ego_planner_node /go2_cmd_vel_to_sport; do for s in configure activate; do timeout 10 ros2 lifecycle set $n $s >/dev/null 2>&1; done; done
echo "$NAME d$DOM: ego $(timeout 5 ros2 lifecycle get /ego_planner_node 2>/dev/null | cut -d' ' -f1) motor $(timeout 5 ros2 lifecycle get /go2_cmd_vel_to_sport 2>/dev/null | cut -d' ' -f1) · CONE=${CONE:-det} MISS=${MISS:-0.2} STEP=${STEP:-0.10} MAXV=${MAXV:-0.5} SAFE=${SAFE:-0.50} PASS=${PASS:-0.8} SKIP=${SKIP:-25} GUARD=${GUARD:-on} $(date +%T)"
RUNNER_GREEN_CONFIRM_SECONDS=1.0 RUNNER_STOPLINE_GREEN_TIMEOUT_S=10.0 RUNNER_INFLATE_NORMAL=0.30 RUNNER_INFLATE_CONE=0.20 RUNNER_GAIT_MODE=classic RUNNER_PARK_MODE=down \
RUNNER_PASS_SWITCH=${PASS:-0.8} RUNNER_EGO_REVIEW=1 RUNNER_ROTATE_SKIP_DEG=${SKIP:-25} RUNNER_SLALOM_MODE=${SMODE:-auto} RUNNER_EXIT_LINE_X=${EXIT_X:-0.70} RUNNER_SNAP_CLEARANCE=${SNAPC:-0.40} RUNNER_SNAP_MAX=${SNAPM:-0.35} \
  run runner python3 $J/src/real_course_runner.py $O/route.json
RIDX=$((${#PGS[@]} - 1)); T0=$(date +%s)
for i in $(seq 1 ${TMAX:-360}); do
  sleep 1
  tail -1 $O/world.jsonl 2>/dev/null | grep -q "\"rs\": \"DONE\"" && { sleep 2; break; }
  kill -0 ${PGS[$RIDX]} 2>/dev/null || { echo "★ $NAME 러너 꺼짐"; break; }
done
echo "$NAME 끝 $(( $(date +%s) - T0 ))초 $(date +%T)"
stop; trap - EXIT
~/venv/go2/bin/python3 $S/sim_report_v2.py $O > $O/report.txt 2>&1; cat $O/report.txt
