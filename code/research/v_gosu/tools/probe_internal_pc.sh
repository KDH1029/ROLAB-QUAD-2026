#!/usr/bin/env bash
# Go2 내부 PC(Orin NX) 환경 확인 — 읽기만 한다. 아무것도 설치·변경하지 않는다.
# 사용(NUC 에서, 비밀번호는 사용자가 입력):
#   ssh unitree@192.168.123.18 'bash -s' < ~/ws/v_gosu/tools/probe_internal_pc.sh | tee ~/ws/v_gosu/eval/out/internal_pc_probe.txt
echo "== 기기";      cat /proc/device-tree/model 2>/dev/null; echo; head -1 /etc/nv_tegra_release 2>/dev/null; uname -r
echo "== JetPack/TensorRT/CUDA"
dpkg -l 2>/dev/null | grep -E 'nvidia-jetpack |nvidia-l4t-core|tensorrt |libnvinfer8 |cuda-toolkit|libcudnn8 ' | awk '{print $2, $3}'
ls /usr/src/tensorrt/bin/trtexec /usr/local/cuda/bin/nvcc 2>&1
echo "== 파이썬";    python3 --version
python3 - <<'PY' 2>/dev/null
import importlib
for m in ['numpy','cv2','torch','torchvision','ultralytics','onnx','onnxruntime','tensorrt','pycuda','pyrealsense2','rclpy']:
    try:
        x = importlib.import_module(m); print(f'  {m:14s} {getattr(x, "__version__", "있음")}')
    except Exception as e:
        print(f'  {m:14s} 없음')
PY
echo "== 자원";      nproc; free -h | head -2; df -h ~ | tail -1
echo "== 전력 모드"; nvpmodel -q 2>/dev/null | head -3
echo "== 부하·온도 (tegrastats 2초)"; timeout 2.5 tegrastats --interval 1000 2>/dev/null | tail -1
echo "== CPU 상위 프로세스"; top -bn1 | sed -n '7,22p'
