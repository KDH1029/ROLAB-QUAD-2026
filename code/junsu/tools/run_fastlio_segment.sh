#!/usr/bin/env bash
# run_fastlio_segment.sh <bag_path> <config_yaml> <log_tag>
# Starts go2_bridge (cloud+imu) + fast_lio against a bag, waits for bag playback
# to finish, then SIGINTs fast_lio's process group to trigger PCD save, and
# cleans everything up. Each invocation is self-contained (no leftover procs).
BAG="$1"
CFG="$2"
TAG="$3"
PCD_OUT="${4:-}"
LOGDIR=~/ws/junsu/maps/segments/logs
mkdir -p "$LOGDIR"

set +u
source ~/setup_go2_ros2.sh >/dev/null 2>&1
source ~/ws/lab/install/setup.bash >/dev/null 2>&1
set +u

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[$TAG] FATAL: ros2 not found after sourcing envs" >&2
  exit 1
fi
if [ ! -f "$BAG/metadata.yaml" ]; then
  echo "[$TAG] FATAL: bag not found at $BAG" >&2
  exit 1
fi
echo "[$TAG] env OK, ros2=$(command -v ros2), RMW=$RMW_IMPLEMENTATION"

# clean slate: kill any leftover bridge/fastlio/bag-play from a previous run
pkill -9 -f '[c]loud_bridge_node' 2>/dev/null
pkill -9 -f '[i]mu_bridge_node' 2>/dev/null
pkill -9 -f '[f]astlio_mapping' 2>/dev/null
pkill -9 -f '[r]os2 bag play' 2>/dev/null
sleep 1

setsid ros2 run go2_bridge cloud_bridge_node </dev/null >"$LOGDIR/${TAG}_cloudbridge.log" 2>&1 &
CLOUD_PID=$!
setsid ros2 run go2_bridge imu_bridge_node </dev/null >"$LOGDIR/${TAG}_imubridge.log" 2>&1 &
IMU_PID=$!
sleep 2

if [ -n "$PCD_OUT" ]; then
  mkdir -p "$(dirname "$PCD_OUT")"
  setsid ros2 run fast_lio fastlio_mapping --ros-args --params-file "$CFG" \
    -p map_file_path:="$PCD_OUT" \
    </dev/null >"$LOGDIR/${TAG}_fastlio.log" 2>&1 &
else
  setsid ros2 run fast_lio fastlio_mapping --ros-args --params-file "$CFG" \
    </dev/null >"$LOGDIR/${TAG}_fastlio.log" 2>&1 &
fi
FASTLIO_PID=$!
sleep 3

echo "[$TAG] bridges pid=$CLOUD_PID/$IMU_PID fastlio pid=$FASTLIO_PID, playing bag: $BAG"
setsid ros2 bag play "$BAG" --rate 1.0 \
  </dev/null >"$LOGDIR/${TAG}_bagplay.log" 2>&1 &
BAGPLAY_PID=$!
wait "$BAGPLAY_PID"
echo "[$TAG] bag playback finished, settling 3s before save signal"
sleep 3

kill -INT -- -"$FASTLIO_PID" 2>/dev/null
echo "[$TAG] sent SIGINT to fastlio process group, waiting up to 30s for save"
for i in $(seq 1 30); do
  if ! kill -0 "$FASTLIO_PID" 2>/dev/null; then
    echo "[$TAG] fastlio exited after ${i}s"
    break
  fi
  sleep 1
done

kill -9 -- -"$FASTLIO_PID" 2>/dev/null
kill -9 -- -"$CLOUD_PID" 2>/dev/null
kill -9 -- -"$IMU_PID" 2>/dev/null
pkill -9 -f '[c]loud_bridge_node' 2>/dev/null
pkill -9 -f '[i]mu_bridge_node' 2>/dev/null
pkill -9 -f '[f]astlio_mapping' 2>/dev/null
echo "[$TAG] cleanup done"
