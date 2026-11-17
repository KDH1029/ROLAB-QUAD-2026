#!/usr/bin/env bash
cd ~/ws/lab_visual
for H in 10 20; do
  ~/ws/lab/kill_all.sh --nodes-only >/dev/null 2>&1
  read -t 5 </dev/null
  MAP=go2_prior_map_v05.pcd POSE="x=0.379 y=3.178 yaw=150.5" POSE_AT=8 HZ=$H TAG=hz$H \
    ./verify_localization.sh live_0910_1512 100
  echo "════════════════════════════════════"
done
