#!/usr/bin/env bash
# Go2 Mapping Script (Live 2D/2.5D Mapping & Offline PCD Conversion)
# Usage:
#   ./scripts/go2_map_run.sh --name my_map          # Live mapping on robot
#   ./scripts/go2_map_run.sh --sim --name sim_map   # Live mapping with simulator
#   ./scripts/go2_map_run.sh --pcd <file.pcd>       # Convert 3D PCD to 2D/2.5D map
set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_DIR"

SIM_MODE=false
MAP_NAME="map_$(date +%m%d_%H%M)"
PCD_FILE=""
SET_DEFAULT="--set-default"
DURATION="0"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --sim)
            SIM_MODE=true
            shift
            ;;
        --name)
            MAP_NAME="${2:-$MAP_NAME}"
            shift 2
            ;;
        --pcd)
            PCD_FILE="${2:-}"
            shift 2
            ;;
        --duration)
            DURATION="${2:-0}"
            shift 2
            ;;
        --no-default)
            SET_DEFAULT=""
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [--name <map_name>] [--sim] [--pcd <pcd_path>] [--duration <sec>]"
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

if [[ -n "$PCD_FILE" ]]; then
    echo "[INFO] Converting 3D PCD ($PCD_FILE) to 2D occupancy grid & 2.5D elevation map..."
    python3 "$WS_DIR/scripts/go2_mapper.py" pcd2map "$PCD_FILE" --name "$MAP_NAME" $SET_DEFAULT
    exit 0
fi

if [ "$SIM_MODE" = false ]; then
    export ROS_DOMAIN_ID=0
    # The robot-side interface is whichever one holds a 192.168.123.x address (eno1, enp86s0, robot-br, ...)
    iface=$(ip -o -4 addr show | awk '$4 ~ /^192\.168\.123\./ {print $2; exit}')
    if [ -z "$iface" ]; then
        echo "[ERROR] No interface with a 192.168.123.x address. Is the robot connected?"
        exit 1
    fi
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><NetworkInterfaceAddress>${iface}</NetworkInterfaceAddress></General></Domain></CycloneDDS>"
    echo "[INFO] Bound CycloneDDS to interface: ${iface}"
else
    export ROS_LOCALHOST_ONLY=1
fi

echo "----------------------------------------------------------------------"
echo "Unitree Go2 Live Mapping"
echo "  Target map : maps/${MAP_NAME} (.pcd, .pgm, .yaml, _elevation.npz)"
echo "  TF frame   : odom -> base_link -> base_stab"
echo "  Stop & save: Press Ctrl+C once when scanning is complete."
echo "----------------------------------------------------------------------"

cleanup() {
    echo -e "\n[INFO] Stopping mapping bridge nodes..."
    # Only our own children; a name-based pkill -f would also kill other workspaces' bridges.
    pkill -INT -P $$ 2>/dev/null || true
}
trap cleanup EXIT INT TERM

ros2 launch go2_bridge bridge.launch.py >/dev/null 2>&1 &
sleep 1

if [ "$SIM_MODE" = true ]; then
    python3 "$WS_DIR/scripts/sim_nav_test.py" >/dev/null 2>&1 &
    sleep 1
fi

python3 "$WS_DIR/scripts/go2_mapper.py" live \
    --name "$MAP_NAME" \
    --duration "$DURATION" \
    $SET_DEFAULT
