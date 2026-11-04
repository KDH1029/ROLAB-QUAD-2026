#!/usr/bin/env bash
# 2026 RoboWorld Competition Full Mission Runner Script
# Usage:
#   ./scripts/go2_mission_run.sh          # Hardware competition mode (map frame)
#   ./scripts/go2_mission_run.sh --sim    # Simulation closed-loop mission test
set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_DIR"

SIM_MODE=false
while [[ $# -gt 0 ]]; do
    case "$1" in
        --sim)
            SIM_MODE=true
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [--sim]"
            exit 0
            ;;
        *)
            shift
            ;;
    esac
done

if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
fi

if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
fi

if [ "$SIM_MODE" = false ]; then
    export ROS_DOMAIN_ID=0
    for iface in eno1 eth0; do
        if ip link show "$iface" >/dev/null 2>&1 && [ "$(cat /sys/class/net/${iface}/carrier 2>/dev/null)" = "1" ]; then
            export CYCLONEDDS_URI="<CycloneDDS><Domain><General><NetworkInterfaceAddress>${iface}</NetworkInterfaceAddress></General></Domain></CycloneDDS>"
            break
        fi
    done
    REL_ROUTE="false"
    REF_FRAME="map"
else
    export ROS_LOCALHOST_ONLY=1
    REL_ROUTE="true"
    REF_FRAME="odom"
fi

echo "----------------------------------------------------------------------"
echo "2026 RoboWorld Quadruped Competition Mission Stack (450-pt Full Course)"
echo "  Vision node  : (teammate's YOLO, start it separately - README 비전 입력)"
echo "  Controller   : mission_fsm_node (7-Stage C++ State Machine)"
echo "  Frame        : $REF_FRAME (sim_mode=$SIM_MODE)"
echo "----------------------------------------------------------------------"

cleanup() {
    echo -e "\n[INFO] Stopping competition mission stack..."
    # Only our own children. A name-based pkill -f would also kill other workspaces'
    # cloud_bridge_node / imu_bridge_node running on the same machine.
    pkill -INT -P $$ 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [ "$SIM_MODE" = false ]; then
    echo "[INFO] Waypoints are in the 'map' frame: start localization first (go2_nav_run.sh)."
    echo "       Without map->odom TF the FSM holds still instead of driving."
fi

if [ "$SIM_MODE" = true ]; then
    python3 "$WS_DIR/scripts/sim_nav_test.py" >/dev/null 2>&1 &
    sleep 1
fi

ros2 launch go2_bridge bridge.launch.py >/dev/null 2>&1 &
sleep 1

ros2 launch mission_runner mission.launch.py \
    use_relative_route:="$REL_ROUTE" \
    reference_frame:="$REF_FRAME"
