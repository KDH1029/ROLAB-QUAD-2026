#!/usr/bin/env bash
CFG=~/ws/lab/go2_ws/config/odometry.yaml
cd ~/ws/archbench
for r in 1 2; do
  OVERLAY= ./bench_arch.sh "$CFG" "MP3_r$r" 60
  OVERLAY=~/ws/archbench/install/setup.bash ./bench_arch.sh "$CFG" "MP1_r$r" 60
done
echo ALL_DONE
