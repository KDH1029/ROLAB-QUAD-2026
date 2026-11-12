#!/usr/bin/env bash
# 내부 PC(Jetson) 신호등 비전 — 엔진 만들기·평가·노드 켜기/끄기. NUC 의 deploy_onboard_vision.sh 가 ssh 로 부른다.
# 사용: ~/junsu_vision/jetson_vision.sh build|eval|start|stop|stop-camera|status|bench
#   build        FP16·INT8 엔진 만들기 (nice 19, 코어 $CORES) — 로봇이 엎드려 있을 때만
#   eval         두 엔진으로 evalset/ 사진 돌려 results/*.json (NUC 가 채점)
#   start        카메라(없으면 컬러만 848x480x30) + 신호등 노드(camera_active 기다림)
#   bench        15초 동안 camera_active 없이 추론 — 실제 카메라로 속도·판정 확인(…_bench 토픽으로만 내서 러너에 안 섞임)
# ★ 로그인 메뉴(ros:foxy(1))가 하는 설정을 그대로 넣었다(ssh 로 메뉴에 '1' 을 먹이면 입력이 잘린다 — 11-04).
D="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
set +u; source /opt/ros/foxy/setup.bash; source ~/cyclonedds_ws/install/setup.bash; set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=~/cyclonedds_ws/cyclonedds.xml
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
# 모델: TL_MODEL(기본 tl_cone_1104 = 11-04 경기장 사진 학습 4클래스 · 예비 tl5cone 6클래스). NUC 의 deploy 가 넘겨 준다
TL_MODEL=${TL_MODEL:-tl_cone_1104}; export TL_MODEL
ONNX=$D/${TL_MODEL}_384x640.onnx; E8=$D/${TL_MODEL}_384x640_int8.engine; E16=$D/${TL_MODEL}_384x640_fp16.engine
# 노드가 쓸 엔진: TL_PREC=fp16(기본 — 11-05 채점: INT8 초록 재현 71% vs 원본 84%) | int8. 고른 게 없으면 다른 정밀도
TL_PREC=${TL_PREC:-fp16}; EN=$([ "$TL_PREC" = int8 ] && echo $E8 || echo $E16); [ -f $EN ] || EN=$([ $EN = $E16 ] && echo $E8 || echo $E16)
if [ "${1:-}" = start ] && [ ! -f $EN ]; then   # 고른 모델 엔진이 없으면 다른 모델·정밀도 엔진으로라도 켠다(신호등이 아예 없는 것보다 낫다)
  for f in $D/tl_cone_1104_384x640_fp16.engine $D/tl_cone_1104_384x640_int8.engine $D/tl5cone_384x640_fp16.engine $D/tl5cone_384x640_int8.engine; do
    [ -f $f ] && { echo "★ $(basename $EN) 없음 → $(basename $f) 로 켬"; EN=$f; TL_MODEL=$(basename $f | sed 's/_384x640.*//'); break; }; done
fi
CORES=${CORES:-6,7}; mkdir -p $D/logs $D/results
RUN=(nice -n 19 taskset -c $CORES)
node_pid(){ pgrep -f "[t]raffic_light_trt_node.py"; }
cam_pid(){ pgrep -f "[r]ealsense2_camera_node"; }
case "${1:-status}" in
  build)
    python3 -c "import tensorrt as t, numpy, cv2; print('TensorRT', t.__version__, 'numpy', numpy.__version__, 'cv2', cv2.__version__)" || exit 1
    "${RUN[@]}" python3 $D/trt_build.py $ONNX $E16 --mode fp16 || exit 1
    "${RUN[@]}" python3 $D/trt_build.py $ONNX $E8 --mode int8 --calib $D/calib || exit 1;;
  build16)  # ★ 11-05 FP16 만 — INT8 은 초록 재현이 84→71% 로 떨어져 FP16 을 기본으로 씀
    python3 -c "import tensorrt" || exit 1
    "${RUN[@]}" python3 $D/trt_build.py $ONNX $E16 --mode fp16 || exit 1;;
  eval16)
    [ -f $E16 ] || { echo "★ $E16 없음 — build16 먼저"; exit 1; }
    "${RUN[@]}" python3 $D/trt_eval.py $E16 $D/results/eval_${TL_MODEL}_fp16.json $D/evalset/*/ || exit 1;;
  build8)   # ★ 11-05 짧은 시험: INT8 만(약 7분)
    python3 -c "import tensorrt as t, numpy, cv2; print('TensorRT', t.__version__, 'numpy', numpy.__version__, 'cv2', cv2.__version__)" || exit 1
    "${RUN[@]}" python3 $D/trt_build.py $ONNX $E8 --mode int8 --calib $D/calib || exit 1;;
  eval8)
    [ -f $E8 ] || { echo "★ $E8 없음 — build8 먼저"; exit 1; }
    "${RUN[@]}" python3 $D/trt_eval.py $E8 $D/results/eval_${TL_MODEL}_int8.json $D/evalset/*/ || exit 1;;
  eval)
    for e in $E16 $E8; do [ -f $e ] || { echo "★ $e 없음 — build 먼저"; exit 1; }
      "${RUN[@]}" python3 $D/trt_eval.py $e $D/results/eval_${TL_MODEL}_$(basename $e .engine | sed 's/.*_//').json $D/evalset/*/ || exit 1; done;;
  start)
    [ -f $EN ] || { echo "★ 엔진 없음($EN) — build16 먼저"; exit 1; }
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
    setsid nohup "${RUN[@]}" python3 $D/traffic_light_trt_node.py --ros-args -p engine_path:=$EN > $L 2>&1 < /dev/null &
    sleep 5; tail -3 $L; [ -n "$(node_pid)" ] && echo "■ 신호등 노드 켬 (로그 $L)" || { echo "★ 노드가 안 떴다 — $L"; exit 1; };;
  stop)        pkill -INT -f "[t]raffic_light_trt_node.py" && echo "신호등 노드 끔" || echo "신호등 노드 없음";;
  stop-camera) pkill -INT -f "[r]s_launch.py"; pkill -INT -f "[r]ealsense2_camera_node" && echo "카메라 끔" || echo "카메라 없음";;
  bench)
    [ -f $EN ] || { echo "★ $EN 없음"; exit 1; }
    [ -n "$(cam_pid)" ] || { echo "★ 카메라 노드 없음 — start 먼저"; exit 1; }
    timeout -s INT 16 "${RUN[@]}" python3 $D/traffic_light_trt_node.py --ros-args -p engine_path:=$EN -p always_on:=true \
      -r __node:=traffic_light_bench -r /mission/traffic_light_state:=/mission/traffic_light_state_bench \
      -r /mission/traffic_light_debug:=/mission/traffic_light_debug_bench 2>&1 | grep -vE "^\s*$" | tail -8;;
  status)
    echo "모델: $TL_MODEL  쓸 엔진: $(basename $EN)  있는 엔진: $(ls -la $E16 $E8 2>/dev/null | awk '{print $NF, $5}' | tr '\n' ' ')"
    echo "카메라: $(cam_pid | tr '\n' ' ')  신호등 노드: $(node_pid | tr '\n' ' ')"
    L=$(ls -t $D/logs/tl_*.log 2>/dev/null | head -1); [ -n "$L" ] && { echo "--- $L"; tail -4 $L; }
    echo "온도: $(cat /sys/devices/virtual/thermal/thermal_zone*/temp 2>/dev/null | awk '{printf "%d ", $1/1000}')°C  부하: $(cut -d' ' -f1-3 /proc/loadavg)";;
  *) echo "사용: jetson_vision.sh build|eval|start|stop|stop-camera|status|bench"; exit 1;;
esac
