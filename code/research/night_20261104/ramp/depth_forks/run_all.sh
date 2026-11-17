#!/usr/bin/env bash
# 깊이 갈래 평가 전체 실행 (bag 오프라인 읽기만, DDS 참여 없음). 4 프로세스 병렬 × 2 묶음, 약 10분.
#   묶음 a: 표본 t0+k·1.0 s (본 결과)   묶음 b: t0+0.5+k·1.0 s (겹치지 않는 둘째 표본 — 재현성·검정력)
set -e
cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 DT=1.0
[ -f prep_cache.npz ] || python3 eval_depth_forks.py prep
for set_ in "a 0.0" "b 0.5"; do
  set -- $set_; S=$1; export PHASE=$2
  rm -f part_${S}*.json
  for seg in "0 136 ${S}0" "136 272 ${S}1" "272 408 ${S}2" "408 545 ${S}3"; do
    set -- $seg; nice -n 10 python3 eval_depth_forks.py run $1 $2 $3 > run_$3.log 2>&1 &
  done
  wait
done
PARTS='part_a*.json' OUTJ=depth_forks_results.json python3 eval_depth_forks.py merge > merge_a.log 2>&1
PARTS='part_b*.json' OUTJ=depth_forks_results_phase05.json python3 eval_depth_forks.py merge > merge_b.log 2>&1
PARTS='part_[ab]*.json' OUTJ=depth_forks_results_dt05_all.json DT=0.5 python3 eval_depth_forks.py merge > merge_ab.log 2>&1
echo done; for f in run_[ab]*.log; do echo "$f: $(tail -n 1 $f)"; done
