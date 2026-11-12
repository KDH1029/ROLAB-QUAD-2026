#!/usr/bin/env bash
# 직진 쏠림 보정 다시 재기(로봇이 움직임). 관리자(go2_ctl)에서 모터(m)가 켜져 있고 러너는 꺼진 상태로.
J="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
set +u
source ~/setup_go2_ros2.sh >/dev/null 2>&1
export CYCLONEDDS_URI=file://$J/config/cyclonedds_junsu.xml
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash >/dev/null 2>&1
exec python3 $J/tools/yaw_trim_measure.py "$@"
