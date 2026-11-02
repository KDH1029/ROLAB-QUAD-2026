#!/usr/bin/env bash
# Go2 Mapping Script (Live 2D/2.5D Mapping & Offline PCD Conversion)
# Usage:
#   ./scripts/go2_map_run.sh --name my_map          # Live mapping on robot
#   ./scripts/go2_map_run.sh --sim --name sim_map   # Live mapping with simulator
#   ./scripts/go2_map_run.sh --pcd <file.pcd>       # Convert 3D PCD to 2D/2.5D map
#   ./scripts/go2_map_run.sh --lio --name my_map    # Live mapping on FAST-LIO poses (recommended)
#
# Mapping never starts the motor bridge (cmd_vel_to_sport): drive the robot with the remote.
# --lio needs the team's FAST-LIO build as underlay (SLAM_UNDERLAY, default ~/ws/lab/install).
set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_DIR"

SIM_MODE=false
LIO_MODE=false
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
        --lio)
            LIO_MODE=true
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

if [ "$LIO_MODE" = true ]; then
    SLAM_UNDERLAY="${SLAM_UNDERLAY:-$HOME/ws/lab/install}"
    if [ ! -d "$SLAM_UNDERLAY/fast_lio" ]; then
        echo "[ERROR] FAST-LIO underlay not found: $SLAM_UNDERLAY"
        exit 1
    fi
    source "$SLAM_UNDERLAY/setup.bash"
    source "$WS_DIR/install/setup.bash"
fi

if [ -n "${GO2_MAP_TEST_LO:-}" ]; then
    # Bag-replay test hook: loopback only, isolated domain, never reaches the robot
    export ROS_DOMAIN_ID="${GO2_MAP_TEST_LO}" RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    export CYCLONEDDS_URI='<CycloneDDS><Domain Id="any"><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex><Peers><Peer address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
    echo "[TEST] loopback replay mode, ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
elif [ "$SIM_MODE" = false ]; then
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
echo "  Pose source: $([ "$LIO_MODE" = true ] && echo 'FAST-LIO (camera_init -> body, /cloud_registered)' || echo 'leg odom (odom -> base_link)')"
echo "  Stop & save: Press Ctrl+C once when scanning is complete."
echo "----------------------------------------------------------------------"

cleanup() {
    echo -e "\n[INFO] Stopping mapping bridge nodes..."
    # Only our own children; a name-based pkill -f would also kill other workspaces' bridges.
    pkill -INT -P $$ 2>/dev/null || true
}
trap cleanup EXIT INT TERM

BR="$WS_DIR/install/go2_bridge/lib/go2_bridge"
LOG_DIR="$WS_DIR/maps/${MAP_NAME}_logs"; mkdir -p "$LOG_DIR"
if [ "$SIM_MODE" = false ] && [ -z "${GO2_MAP_TEST_LO:-}" ]; then
    if ! timeout 4 ros2 topic echo --once /lidar_points --field header >/dev/null 2>&1; then
        echo "[INFO] /lidar_points not found: starting Hesai driver"
        ros2 launch hesai_ros_driver start.py >"$LOG_DIR/hesai.log" 2>&1 &
        sleep 4
    fi
fi
if [ "$LIO_MODE" = true ]; then
    # FAST-LIO publishes camera_init->body, so imu_bridge must not publish base_link->body.
    "$BR/imu_bridge_node" --ros-args -p publish_static_tf:=false >"$LOG_DIR/imu.log" 2>&1 &
    "$BR/cloud_bridge_node" --ros-args -p publish_terrain_scan:=false >"$LOG_DIR/cloud.log" 2>&1 &
    sleep 2
    echo "[INFO] Starting FAST-LIO — keep the robot still for 2 s"
    "$SLAM_UNDERLAY/fast_lio/lib/fast_lio/fastlio_mapping" --ros-args -r __node:=fastlio_odometry \
        --params-file "$WS_DIR/config/fastlio/fastlio_xt16_odometry.yaml" >"$LOG_DIR/fastlio.log" 2>&1 &
    POSE_SOURCE=lio
else
    "$BR/odom_tf_node" >"$LOG_DIR/odom_tf.log" 2>&1 &
    "$BR/imu_bridge_node" >"$LOG_DIR/imu.log" 2>&1 &
    "$BR/cloud_bridge_node" >"$LOG_DIR/cloud.log" 2>&1 &
    POSE_SOURCE=leg
fi
sleep 1

if [ "$SIM_MODE" = true ]; then
    python3 "$WS_DIR/scripts/sim_nav_test.py" >/dev/null 2>&1 &
    sleep 1
fi

python3 "$WS_DIR/scripts/go2_mapper.py" live \
    --name "$MAP_NAME" \
    --pose-source "$POSE_SOURCE" \
    --duration "$DURATION" \
    $SET_DEFAULT
