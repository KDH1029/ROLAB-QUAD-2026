#!/usr/bin/env bash
# Go2 Odometry & Kinematics Bridge Launch Script
# Usage:
#   ./scripts/go2_odom_run.sh                  # Hardware odometry stack
#   ./scripts/go2_odom_run.sh --sim            # Simulated odometry stack
set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_DIR"

SIM_MODE=false
ODOM_TOPIC="/utlidar/robot_odom"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --sim)
            SIM_MODE=true
            shift
            ;;
        --odom)
            ODOM_TOPIC="${2:-$ODOM_TOPIC}"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [--sim] [--odom <odom_topic>]"
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
else
    export ROS_LOCALHOST_ONLY=1
fi

echo "----------------------------------------------------------------------"
echo "Unitree Go2 Odometry & Kinematics Bridge"
echo "  Odom source : $ODOM_TOPIC"
echo "  Attitude    : 12-DoF leg FK + gyro complementary filter (base_stab)"
echo "  Control     : /cmd_vel -> /api/sport/request"
echo "----------------------------------------------------------------------"
echo "Direct odometry navigation examples:"
echo "  python3 scripts/go2_odom_nav.py status"
echo "  python3 scripts/go2_odom_nav.py --x 1.5 --y -0.5 --yaw 90"
echo "  python3 scripts/go2_odom_nav.py save wp1 && python3 scripts/go2_odom_nav.py go wp1"
echo "----------------------------------------------------------------------"

cleanup() {
    echo -e "\n[INFO] Stopping odometry bridge stack..."
    pkill -P $$ 2>/dev/null || true
    pkill -f "odom_tf_node|imu_bridge_node|cloud_bridge_node|cmd_vel_to_sport_node|odom_nav_node|sim_nav_test" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [ "$SIM_MODE" = true ]; then
    python3 "$WS_DIR/scripts/sim_nav_test.py" &
    sleep 1
fi

ros2 launch go2_bridge bridge.launch.py odom_topic:="$ODOM_TOPIC"
