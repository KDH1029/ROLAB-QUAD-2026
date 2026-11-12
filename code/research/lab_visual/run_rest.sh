#!/usr/bin/env bash
for v in "data2 x=7.80 y=-4.10 yaw=120" "data_final_final_final x=3.30 y=0.10 yaw=340"; do
  b=$(echo $v|cut -d' ' -f1); p=$(echo $v|cut -d' ' -f2-)
  echo; echo "▶ $b ($p) $(date +%H:%M:%S)"
  BAG=$b POSE="$p" TAG=chk_$b SEC=130 ~/ws/lab_visual/run_one.sh 2>&1 | grep -vE "^\s*$|죽었음" | tail -10
done
