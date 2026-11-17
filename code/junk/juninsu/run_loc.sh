#!/usr/bin/env bash
# 멈춘 곳 위치 확인(②안) 켜기 — junk 관리자(junk_ctl)와 '따로' 다른 터미널에서. 대회 코드는 안 건드림. Ctrl+C 로 끔.
#   실제 로봇:  ~/ws/junk/juninsu/run_loc.sh
#   재생 검증:  LOC_REPLAY=79 ~/ws/junk/juninsu/run_loc.sh   (격리 도메인 78~81 · lo 전용 · bag 시계)
# 켜지는 것: 지도 서버 + AMCL(/loc, /tf_loc — 실제 /tf 는 안 건드림) + stop_loc_node(멈춘 곳에서만 ICP 확인 → /mission/align_T)
set +u
D="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"; J="$(cd "$D/.." && pwd)"
MAP=${LOC_MAP:-$J/routes/rec1650}; SIM=false; EXTRA=()
if [ -n "${LOC_REPLAY:-}" ]; then
  if ip link show robot-br 2>/dev/null | grep -q 'state UP'; then echo "★ 로봇 연결됨 — 재생 검증 거부"; exit 1; fi
  export V_DOMAIN=$LOC_REPLAY; source ~/ws/v_gosu/env_replay.sh
  case "$ROS_DOMAIN_ID" in 78|79|80|81) ;; *) echo "★ 격리 아님"; exit 1;; esac
  SIM=true
else
  source ~/setup_go2_ros2.sh
  export CYCLONEDDS_URI=file://$J/config/cyclonedds_junsu.xml
  source ~/unitree_ros2/cyclonedds_ws/install/setup.bash >/dev/null 2>&1
fi
source ~/ws/merge_tmp_20261102/review/install/setup.bash >/dev/null 2>&1
PGS=(); stop(){ for p in "${PGS[@]}"; do kill -INT -- -"$p" 2>/dev/null; done; sleep 2; for p in "${PGS[@]}"; do kill -TERM -- -"$p" 2>/dev/null; done; }
trap 'echo; echo "위치 확인 끄는 중"; stop' EXIT
L=${LOC_LOGDIR:-$J/logs}; mkdir -p $L
echo "=== 멈춘 곳 위치 확인 켬 $(date +%T) · 도메인 ${ROS_DOMAIN_ID:-0} · 지도 $MAP"
setsid ros2 run nav2_map_server map_server --ros-args -r __ns:=/loc --params-file $D/loc_params.yaml -p yaml_filename:=$MAP/map.yaml -p use_sim_time:=$SIM >$L/loc_map.log 2>&1 & PGS+=($!)
setsid ros2 run nav2_amcl amcl --ros-args -r __ns:=/loc --params-file $D/loc_params.yaml -r /tf:=/tf_loc -r /tf_static:=/tf_loc_static -p use_sim_time:=$SIM >$L/loc_amcl.log 2>&1 & PGS+=($!)
setsid python3 $D/stop_loc_node.py --ros-args -p use_sim_time:=$SIM -p map_dir:=$MAP -p log:=$L/stop_loc.log ${LOC_ARGS:-} >$L/stop_loc_node.log 2>&1 & PGS+=($!)
sleep 4
for n in /loc/map_server /loc/amcl; do for s in configure activate; do timeout 10 ros2 lifecycle set $n $s >/dev/null 2>&1; done; echo "  $n: $(timeout 5 ros2 lifecycle get $n 2>/dev/null | cut -d' ' -f1)"; done
echo "  기록: $L/stop_loc.log  (반영/안 고침 한 줄씩)"
tail -n +1 -f $L/stop_loc.log 2>/dev/null &
wait "${PGS[2]}"
