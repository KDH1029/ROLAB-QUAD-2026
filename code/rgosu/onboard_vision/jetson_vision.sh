#!/usr/bin/env bash
# 내부 PC(Jetson) 신호등 비전 — 엔진 만들기·평가·노드 켜기/끄기. NUC 의 deploy_onboard_vision.sh 가 ssh 로 부른다.
# 사용: ~/onboard_vision/jetson_vision.sh build|eval|start|stop|stop-camera|status|bench
#   build        FP16·INT8 엔진 만들기 (nice 19, 코어 $CORES) — 로봇이 엎드려 있을 때만
#   eval         두 엔진으로 evalset/ 사진 돌려 results/*.json (NUC 가 채점)
#   start        카메라(없으면 컬러만 848x480x30) + 신호등 노드(camera_active 기다림)
#   bench        15초 동안 camera_active 없이 추론 — 실제 카메라로 속도·판정 확인(…_bench 토픽으로만 내서 러너에 안 섞임)
# ★ 로그인 메뉴(ros:foxy(1))가 하는 설정을 그대로 넣었다(ssh 로 메뉴에 '1' 을 먹이면 입력이 잘린다 — 11-04).
D="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
set +u; source /opt/ros/foxy/setup.bash; source ~/cyclonedds_ws/install/setup.bash; set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=~/cyclonedds_ws/cyclonedds.xml
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
ONNX=$D/tl5cone_384x640.onnx; E8=$D/tl5cone_384x640_int8.engine; E16=$D/tl5cone_384x640_fp16.engine
CORES=${CORES:-6,7}; mkdir -p $D/logs $D/results
RUN=(nice -n 19 taskset -c $CORES)
node_pid(){ pgrep -f "[t]raffic_light_trt_node.py"; }
cam_pid(){ pgrep -f "[r]ealsense2_camera_node"; }
case "${1:-status}" in
  build)
    python3 -c "import tensorrt as t, numpy, cv2; print('TensorRT', t.__version__, 'numpy', numpy.__version__, 'cv2', cv2.__version__)" || exit 1
    "${RUN[@]}" python3 $D/trt_build.py $ONNX $E16 --mode fp16 || exit 1
    "${RUN[@]}" python3 $D/trt_build.py $ONNX $E8 --mode int8 --calib $D/calib || exit 1;;
  eval)
    for e in $E16 $E8; do [ -f $e ] || { echo "★ $e 없음 — build 먼저"; exit 1; }
      "${RUN[@]}" python3 $D/trt_eval.py $e $D/results/eval_$(basename $e .engine | sed 's/.*_//').json $D/evalset/*/ || exit 1; done;;
  start)
    [ -f $E8 ] || { echo "★ $E8 없음 — build 먼저"; exit 1; }
    if [ -z "$(cam_pid)" ]; then
      P=848x480x30
      # ★ 11-05: 뎁스도 켠다 — 신호등 노드가 6 m 넘는 상자(전시장 조명)를 버리는 데 쓴다. 인자는 녹화(go2_record_auto.sh)와 같음
      setsid nohup ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=true enable_infra1:=false \
        enable_infra2:=false enable_gyro:=false enable_accel:=false rgb_camera.profile:=$P rgb_camera.color_profile:=$P \
        depth_module.profile:=$P depth_module.depth_profile:=$P \
        > $D/logs/camera_$(date +%H%M%S).log 2>&1 < /dev/null &
      echo "카메라 켬(컬러+뎁스 $P)"; sleep 6
    else echo "카메라 이미 켜져 있음(그대로 씀): pid $(cam_pid | tr '\n' ' ')"; fi
    if [ -n "$(node_pid)" ]; then echo "신호등 노드 이미 켜져 있음: pid $(node_pid)"; exit 0; fi
    L=$D/logs/tl_$(date +%H%M%S).log
    setsid nohup "${RUN[@]}" python3 $D/traffic_light_trt_node.py --ros-args -p engine_path:=$E8 > $L 2>&1 < /dev/null &
    sleep 5; tail -3 $L; [ -n "$(node_pid)" ] && echo "■ 신호등 노드 켬 (로그 $L)" || { echo "★ 노드가 안 떴다 — $L"; exit 1; };;
  stop)        pkill -INT -f "[t]raffic_light_trt_node.py" && echo "신호등 노드 끔" || echo "신호등 노드 없음";;
  stop-camera) pkill -INT -f "[r]s_launch.py"; pkill -INT -f "[r]ealsense2_camera_node" && echo "카메라 끔" || echo "카메라 없음";;
  bench)
    [ -f $E8 ] || { echo "★ $E8 없음"; exit 1; }
    [ -n "$(cam_pid)" ] || { echo "★ 카메라 노드 없음 — start 먼저"; exit 1; }
    timeout -s INT 16 "${RUN[@]}" python3 $D/traffic_light_trt_node.py --ros-args -p engine_path:=$E8 -p always_on:=true \
      -r __node:=traffic_light_bench -r /mission/traffic_light_state:=/mission/traffic_light_state_bench \
      -r /mission/traffic_light_debug:=/mission/traffic_light_debug_bench 2>&1 | grep -vE "^\s*$" | tail -8;;
  status)
    echo "엔진: $(ls -la $E16 $E8 2>/dev/null | awk '{print $NF, $5}' | tr '\n' ' ')"
    echo "카메라: $(cam_pid | tr '\n' ' ')  신호등 노드: $(node_pid | tr '\n' ' ')"
    L=$(ls -t $D/logs/tl_*.log 2>/dev/null | head -1); [ -n "$L" ] && { echo "--- $L"; tail -4 $L; }
    echo "온도: $(cat /sys/devices/virtual/thermal/thermal_zone*/temp 2>/dev/null | awk '{printf "%d ", $1/1000}')°C  부하: $(cut -d' ' -f1-3 /proc/loadavg)";;
  *) echo "사용: jetson_vision.sh build|eval|start|stop|stop-camera|status|bench"; exit 1;;
esac
