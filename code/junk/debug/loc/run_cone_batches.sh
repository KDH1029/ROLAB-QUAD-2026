#!/usr/bin/env bash
# 콘 바꾼 지도 A → B 순서로 AMCL 재생(각 4개 동시: S0/S1 × 높이대 all/M). 한 묶음이 다 끝나야 다음 묶음(도메인 겹침 방지).
cd "$(dirname "$(readlink -f "$0")")"; T45="-0.025,-0.039,-0.0604"
for MV in A B; do
  d=78; pids=()
  for sc in S0 S1; do for b in all M; do
    if [ $b = M ]; then S=/scan_M; BB=$PWD/out/r1645/bands; else S=/scan; BB=; fi
    MV=$MV SCAN=$S BANDS=$BB ./amcl_run.sh r1645 $sc tuned $d "$T45" > runc_${MV}_${sc}_$b.txt 2>&1 & pids+=($!); d=$((d+1))
  done; done
  wait "${pids[@]}"; echo "묶음 $MV 끝 $(date +%T)"; sleep 3
done
