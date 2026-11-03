#!/usr/bin/env bash
# Environment for running FAST-LIO and the bridges from this workspace by hand.
# FAST-LIO / livox msgs / Hesai driver are forked into src/external and built with colcon.
# 사용: source scripts/go2_slam_env.sh
_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
CYCLONEDDS_WS="${CYCLONEDDS_WS:-$HOME/unitree_ros2/cyclonedds_ws/install}"
[ -f "$CYCLONEDDS_WS/setup.bash" ] && source "$CYCLONEDDS_WS/setup.bash"
source "$_WS/install/setup.bash"
export GO2_FASTLIO_CONFIG="${GO2_FASTLIO_CONFIG:-$_WS/config/fastlio/fastlio_xt16_odometry.yaml}"
echo "[INFO] fast_lio : $(ros2 pkg prefix fast_lio)"
echo "[INFO] go2_bridge: $(ros2 pkg prefix go2_bridge)"
echo "[INFO] config   : $GO2_FASTLIO_CONFIG"
