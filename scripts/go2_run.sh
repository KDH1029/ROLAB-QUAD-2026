#!/usr/bin/env bash
# Go2 one-line runner. Run it in your own terminal: Ctrl+C stops everything (motor bridge sends StopMove).
#
#   ./scripts/go2_run.sh --map maps/arena.yaml                    # SLAM + AMCL + EGO-Planner
#   ./scripts/go2_run.sh --map maps/arena.yaml --no-motor         # same, cannot move the robot
#   ./scripts/go2_run.sh --map maps/arena.yaml --start "0 0 0"    # + AMCL start pose (x y yaw_deg)
#   ./scripts/go2_run.sh --map maps/arena.yaml --cones            # + cone tracker (auto: YOLO if it runs, else LiDAR)
#        --cone-mode auto|fusion|lidar_only   --cone-regions "x0 y0 x1 y1 ..."   (LiDAR-only rectangles, map frame)
#        lidar_only = no camera at all (stop lines then proceed 2.8 s after stopping: no light signal)
#   ./scripts/go2_run.sh --map maps/arena.yaml --route maps/route_arena.yaml   # + route runner
#   ./scripts/go2_run.sh --mode patrol                            # RANSAC wall-runner fallback
#
# Route: save points on site with  python3 scripts/go2_route.py save <name>  (AMCL pose), then
#        python3 scripts/go2_route.py start [step] | stop | resume | status
# Then send goals in the map frame:  python3 scripts/go2_goto.py --x 2.0 --y 1.0 --yaw 0
# Patrol starts idle: ros2 topic pub --once /patrol/cmd std_msgs/msg/String "{data: START}"
# FAST-LIO, livox msgs and the Hesai driver are built from src/external (colcon build).
set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE=nav
MAP=""
MOTOR=true
START=""
CONES=false
ROUTE=""
CONE_MODE=auto
CONE_REGIONS=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --mode) MODE="$2"; shift 2 ;;
        --map) MAP="$(realpath "$2")"; shift 2 ;;
        --no-motor) MOTOR=false; shift ;;
        --start) START="$2"; shift 2 ;;
        --cones) CONES=true; shift ;;
        --route) ROUTE="$(realpath "$2")"; shift 2 ;;
        --cone-mode) CONE_MODE="$2"; CONES=true; shift 2 ;;
        --cone-regions) CONE_REGIONS="$2"; shift 2 ;;
        -h|--help) sed -n 2,18p "$0"; exit 0 ;;
        *) echo "[ERROR] unknown argument: $1"; exit 1 ;;
    esac
done

if [ "$MODE" = nav ] && [ ! -f "$MAP" ]; then
    echo "[ERROR] --mode nav needs --map <map.yaml> (maps are made outside this repo; clean cones with scripts/go2_map_clean.py)"
    exit 1
fi

source /opt/ros/jazzy/setup.bash
# CycloneDDS RMW: from apt (ros-jazzy-rmw-cyclonedds-cpp) or a source build (CYCLONEDDS_WS)
CYCLONEDDS_WS="${CYCLONEDDS_WS:-$HOME/unitree_ros2/cyclonedds_ws/install}"
[ -f "$CYCLONEDDS_WS/setup.bash" ] && source "$CYCLONEDDS_WS/setup.bash"
if ! ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null 2>&1; then
    echo "[ERROR] rmw_cyclonedds_cpp not found (apt install ros-jazzy-rmw-cyclonedds-cpp, or set CYCLONEDDS_WS)"
    exit 1
fi
source "$WS_DIR/install/setup.bash"

if [ -n "${GO2_RUN_TEST_LO:-}" ]; then
    # Bag-replay test hook: loopback only, isolated domain, never reaches the robot
    export ROS_DOMAIN_ID="${GO2_RUN_TEST_LO}" RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    export CYCLONEDDS_URI='<CycloneDDS><Domain Id="any"><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex><Peers><Peer address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
    echo "[TEST] loopback replay mode, ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
else
    export ROS_DOMAIN_ID=0
    iface=$(ip -o -4 addr show | awk '$4 ~ /^192\.168\.123\./ {print $2; exit}')
    if [ -z "$iface" ]; then
        echo "[ERROR] No interface with a 192.168.123.x address. Is the robot connected?"
        exit 1
    fi
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><NetworkInterfaceAddress>${iface}</NetworkInterfaceAddress></General></Domain></CycloneDDS>"
    echo "[INFO] CycloneDDS on ${iface}"
fi

LOG_DIR="$WS_DIR/log/run_$(date +%m%d_%H%M%S)"; mkdir -p "$LOG_DIR"
cleanup() {
    echo -e "\n[INFO] Stopping (lifecycle shutdown, StopMove)..."
    pkill -INT -P $$ 2>/dev/null || true
    sleep 2
}
trap cleanup EXIT INT TERM

if [ -z "${GO2_RUN_TEST_LO:-}" ]; then
    if ! timeout 4 ros2 topic echo --once /lidar_points --field header >/dev/null 2>&1; then
        echo "[INFO] /lidar_points not found: starting Hesai driver"
        ros2 launch hesai_ros_driver start.py >"$LOG_DIR/hesai.log" 2>&1 &
        sleep 4
    fi
fi

echo "----------------------------------------------------------------------"
echo " Go2 stack: mode=$MODE motor=$MOTOR ${MAP:+map=$MAP}"
echo " Logs: $LOG_DIR"
echo " Stop: Ctrl+C   |   software stop: ros2 lifecycle set /go2_cmd_vel_to_sport deactivate"
echo "----------------------------------------------------------------------"

# Launch is a direct child (not inside a pipeline) so Ctrl+C / cleanup reaches it
ros2 launch go2_navigation go2_stack.launch.py mode:="$MODE" motor:="$MOTOR" map:="$MAP" cones:="$CONES" route:="$ROUTE" \
    cone_mode:="$CONE_MODE" cone_regions:="$CONE_REGIONS" \
    fastlio_config:="$WS_DIR/config/fastlio/fastlio_xt16_odometry.yaml" > >(tee "$LOG_DIR/stack.log") 2>&1 &

if [ "$MODE" = nav ] && [ -n "$START" ]; then
    # Wait until AMCL is active, then give it the start pose with uncertainty and let it settle
    for i in $(seq 60); do
        [ "$(timeout 3 ros2 lifecycle get /amcl 2>/dev/null | cut -d' ' -f1)" = active ] && break
        sleep 1
    done
    read -r SX SY SYAW <<<"$START"
    python3 "$WS_DIR/scripts/go2_amcl_init.py" "$SX" "$SY" "$SYAW" | tee -a "$LOG_DIR/stack.log"
fi
wait
