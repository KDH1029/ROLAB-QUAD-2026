#!/usr/bin/env bash
CFG=~/ws/lab/go2_ws/config/odometry.yaml
cd ~/ws/archbench
for r in 1 2; do
  OVERLAY= ./bench_lat.sh "$CFG" "LAT_MP3_r$r" 50
  OVERLAY=~/ws/archbench/install/setup.bash ./bench_lat.sh "$CFG" "LAT_MP1_r$r" 50
done
echo ALL_DONE
