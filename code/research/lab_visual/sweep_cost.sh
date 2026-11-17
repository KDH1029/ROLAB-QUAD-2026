#!/usr/bin/env bash
# 정합 설정별 정확도 vs 비용 — 비전(YOLO)에 남길 CPU 여유를 정하기 위한 측정.
# data1 의 주행 구간(0~140초)만 본다. 정지 구간은 아무 설정이나 잘 나와서 판단에 도움이 안 된다.
export MAP=go2_prior_map_v05.pcd
export POSE="x=0.406 y=-0.211 yaw=90.1"
export POSE_AT=14
SEC=130
for v in "5 800 1 hz5_p800_wide" "5 800 0 hz5_p800_track" "3 600 0 hz3_p600_track" "2 400 0 hz2_p400_track" "5 1500 0 hz5_p1500_track"; do
  set -- $v
  echo; echo "▶ $4  ($(date +%H:%M:%S))"
  HZ=$1 PTS=$2 WIDE=$3 TAG=$4 ~/ws/lab_visual/verify_localization.sh data1 $SEC > ~/ws/lab_visual/logs/sweep_$4.log 2>&1
  grep -E "잔차 +중앙|한 번 보정" ~/ws/lab_visual/logs/sweep_$4.log | sed 's/^/    /'
  awk '{s+=$1;n++} END{if(n) printf "    정합 노드 CPU 평균 %.0f%% (표본 %d)\n", s/n, n}' <(grep -oE "^[0-9.]+" ~/ws/lab_visual/logs/cpu_$4.txt 2>/dev/null)
done
echo; echo "════ 요약 ════"
python3 ~/ws/lab_visual/tools/compare_runs.py
