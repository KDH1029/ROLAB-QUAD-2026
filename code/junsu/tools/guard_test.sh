#!/usr/bin/env bash
# 몸 모양 안전 필터 실기 시험(로봇이 움직임). front | back | spin
J="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
set +u
source ~/setup_go2_ros2.sh >/dev/null 2>&1
export CYCLONEDDS_URI=file://$J/config/cyclonedds_junsu.xml
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash >/dev/null 2>&1
exec python3 $J/tools/guard_test.py "$@"
