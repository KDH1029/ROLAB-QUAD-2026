#!/usr/bin/env bash
# 다른 녹화본으로 추적 검증. 자세는 전역 탐색 1등 후보(bag 14초 시점).
set -- "data4 x=0.90 y=-4.70 yaw=10" "data3 x=1.20 y=-4.70 yaw=10" "data_final_final_final2 x=0.60 y=0.10 yaw=0"
for v in "$@"; do
  b=$(echo $v | cut -d' ' -f1); p=$(echo $v | cut -d' ' -f2-)
  echo; echo "▶ $b  ($p)  $(date +%H:%M:%S)"
  BAG=$b POSE="$p" TAG=chk_$b SEC=130 ~/ws/lab_visual/run_one.sh 2>&1 | grep -vE "^\s*$|죽었음" | tail -10
done
