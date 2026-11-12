#!/usr/bin/env bash
# stop_loc_node 재생 검증 한 번: val_loc.sh <S0|S1|S2> <도메인>   (덤프 out/r1645/rec 재생 + run_loc.sh(LOC_REPLAY) + val_helper)
set +u
SC=$1; DOM=$2; D="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"; O=$D/out/val/$SC; rm -rf $O; mkdir -p $O
if ip link show robot-br 2>/dev/null | grep -q 'state UP'; then echo "★ 로봇 연결됨 — 거부"; exit 1; fi
echo '{"T": [-0.025, -0.039, -0.0604], "ok": true, "reason": "검증용(16:45 출발 T)"}' > $O/align.json
export V_DOMAIN=$DOM; source ~/ws/v_gosu/env_replay.sh >/dev/null 2>&1
PGS=(); stop(){ for p in "${PGS[@]}"; do kill -INT -- -"$p" 2>/dev/null; done; sleep 3; for p in "${PGS[@]}"; do kill -TERM -- -"$p" 2>/dev/null; done; }; trap stop EXIT
LOC_REPLAY=$DOM LOC_LOGDIR=$O LOC_ARGS="-p odom_topic:=/Odometry_c -p align_file:=$O/align.json" setsid ~/ws/junk/juninsu/run_loc.sh > $O/run_loc.out 2>&1 & PGS+=($!)
setsid python3 $D/val_helper.py --ros-args -p scen:=$SC -p out:=$O/val.csv -p use_sim_time:=true > $O/helper.log 2>&1 & PGS+=($!)
sleep 12
ros2 bag play $D/out/r1645/rec --clock 50 --topics /Odometry /scan /tf /cloud_registered_body /sportmodestate < /dev/null > $O/play.log 2>&1
sleep 4; echo "val $SC 끝 $(date +%T) · 반영 $(grep -c '반영 →' $O/stop_loc.log) · 안 고침 $(grep -c '안 고침' $O/stop_loc.log)"
