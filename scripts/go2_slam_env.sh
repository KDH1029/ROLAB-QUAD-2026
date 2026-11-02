#!/usr/bin/env bash
# SLAM(FAST-LIO) 은 이 저장소에 없다. 팀이 원래 쓰던 빌드본을 underlay 로 참조한다(읽기 전용).
#   SLAM_UNDERLAY  기본 ~/ws/lab/install  (fast_lio = ~/ws/lab/go2_ws/src/fast_lio, motion_gating 포함 포크)
# 원본 소스/설정은 고치지 않는다. 설정을 바꿔야 하면 config/fastlio/ 에 포크해서 쓴다.
# 사용: source scripts/go2_slam_env.sh
_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SLAM_UNDERLAY="${SLAM_UNDERLAY:-$HOME/ws/lab/install}"
source /opt/ros/jazzy/setup.bash
if [ ! -f "$SLAM_UNDERLAY/setup.bash" ] || [ ! -d "$SLAM_UNDERLAY/fast_lio" ]; then
    echo "[ERROR] fast_lio underlay not found: $SLAM_UNDERLAY" >&2
    return 1 2>/dev/null || exit 1
fi
source "$SLAM_UNDERLAY/setup.bash"
# underlay 에도 go2_bridge 가 있다. 이 저장소의 go2_bridge 가 위에 덮이도록 overlay 를 나중에 source.
source "$_WS/install/setup.bash"
export GO2_FASTLIO_CONFIG="${GO2_FASTLIO_CONFIG:-$_WS/config/fastlio/fastlio_xt16_odometry.yaml}"
echo "[INFO] fast_lio : $(ros2 pkg prefix fast_lio)"
echo "[INFO] go2_bridge: $(ros2 pkg prefix go2_bridge)"
echo "[INFO] config   : $GO2_FASTLIO_CONFIG"
