#!/usr/bin/env bash
# Go2 Navigation Stack Launch Script
# Usage:
#   ./scripts/go2_nav_run.sh                 # Hardware mode
#   ./scripts/go2_nav_run.sh --sim           # Simulation mode
#   ./scripts/go2_nav_run.sh --map <path>    # Custom map YAML file
#   ./scripts/go2_nav_run.sh --odom <topic>  # Custom odometry topic
set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_DIR"

SIM_MODE=false
MAP_FILE="$WS_DIR/maps/map_default.yaml"
ODOM_TOPIC="/utlidar/robot_odom"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --sim)
            SIM_MODE=true
            shift
            ;;
        --map)
            if [[ -z "${2:-}" ]]; then
                echo "[ERROR] --map requires a file path."
                exit 1
            fi
            MAP_FILE="$2"
            shift 2
            ;;
        --odom)
            if [[ -z "${2:-}" ]]; then
                echo "[ERROR] --odom requires a topic name."
                exit 1
            fi
            ODOM_TOPIC="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [--sim] [--map <path_to_yaml>] [--odom <topic_name>]"
            exit 0
            ;;
        *)
            echo "[WARN] Ignoring unknown argument: $1"
            shift
            ;;
    esac
done

if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "[ERROR] ROS 2 installation not found under /opt/ros."
    exit 1
fi

if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
else
    echo "[INFO] Workspace not built. Running colcon build --symlink-install..."
    colcon build --symlink-install
    source "$WS_DIR/install/setup.bash"
fi

if [ "$SIM_MODE" = false ]; then
    export ROS_DOMAIN_ID=0
    for iface in eno1 eth0; do
        if ip link show "$iface" >/dev/null 2>&1 && [ "$(cat /sys/class/net/${iface}/carrier 2>/dev/null)" = "1" ]; then
            export CYCLONEDDS_URI="<CycloneDDS><Domain><General><NetworkInterfaceAddress>${iface}</NetworkInterfaceAddress></General></Domain></CycloneDDS>"
            echo "[INFO] Bound CycloneDDS to interface: ${iface}"
            break
        fi
    done
else
    echo "[INFO] Running in simulation mode (ROS_LOCALHOST_ONLY=1)"
    export ROS_LOCALHOST_ONLY=1
fi

echo "----------------------------------------------------------------------"
echo "Unitree Go2 Navigation Stack"
echo "  Workspace  : $WS_DIR"
echo "  Map        : $MAP_FILE"
echo "  Odom topic : $ODOM_TOPIC"
echo "  TF tree    : map -> odom -> base_link -> base_stab"
echo "----------------------------------------------------------------------"
echo "Goal command example:"
echo "  python3 scripts/go2_goto.py --x 2.0 --y 1.0 --yaw 90"
echo "----------------------------------------------------------------------"

cleanup() {
    echo -e "\n[INFO] Shutting down navigation nodes..."
    pkill -P $$ 2>/dev/null || true
    pkill -f "odom_tf_node|imu_bridge_node|cloud_bridge_node|cmd_vel_to_sport_node|map_server|amcl|controller_server|planner_server" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

ros2 launch go2_navigation bringup.launch.py \
    map:="$MAP_FILE" \
    sim_mode:="$SIM_MODE" \
    odom_topic:="$ODOM_TOPIC"
