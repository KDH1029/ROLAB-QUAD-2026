#!/usr/bin/env bash
# 패치 매트릭스 끝난 뒤 추가 2판(도메인 80, 한 번에 하나): 몸 모양 안전 필터 켬 · 얇은 벽(사람 주행 보정 −4 cm)
H=$(cd "$(dirname "$0")" && pwd); ROUTE=$H/route_slalom_sim.json; SIM=$H/mirror/sim
until grep -q "=== 끝" $H/matrix_patched.log 2>/dev/null; do sleep 3; done
run1(){ local n=$1; shift; echo "=== $n $(date +%T)" >> $H/matrix_extra.log
  ( cd $SIM && env TMAX=200 EXIT_X=5.45 SEED=1 "$@" ./run_sim_v2.sh $n 80 $ROUTE ) >> $H/matrix_extra.log 2>&1
  ~/venv/go2/bin/python3 $H/slalom_report.py $SIM/runs_v2/$n >> $H/matrix_extra.log 2>&1; }
run1 slalomP_det_path_guard CONE=det SMODE=path GUARD=on
run1 slalomP_walls_path_thin CONE=walls SMODE=path GUARD=off WALLS=$HOME/ws/junsu_v2/sim/sim_walls_rec1650_thin.npy
echo "=== 끝 $(date +%T)" >> $H/matrix_extra.log
