#!/usr/bin/env bash
# slalom 에이전트: 슬라럼 구간만 떼어 조건 매트릭스(도메인 80, 한 번에 하나). 사용: run_matrix.sh "<조건들>"  조건 = CONE:SMODE:판
H=$(cd "$(dirname "$0")" && pwd); ROUTE=$H/route_slalom_sim.json; SIM=$H/mirror/sim
for spec in ${1:-det:path:1 walls:path:1 det:auto:1 walls:auto:1 det:path:2 walls:path:2 det:auto:2 walls:auto:2}; do
  IFS=: read c m r <<< "$spec"; n=slalomP_${c}_${m}_${r}${TAG:-}
  echo "=== $n $(date +%T)" >> $H/matrix_patched.log
  ( cd $SIM && TMAX=${TMAX:-200} EXIT_X=5.45 CONE=$c GUARD=${GUARD:-off} SMODE=$m SEED=$r ./run_sim_v2.sh $n 80 $ROUTE ) >> $H/matrix_patched.log 2>&1
  ~/venv/go2/bin/python3 $H/slalom_report.py $SIM/runs_v2/$n >> $H/matrix_patched.log 2>&1
done
echo "=== 끝 $(date +%T)" >> $H/matrix_patched.log
