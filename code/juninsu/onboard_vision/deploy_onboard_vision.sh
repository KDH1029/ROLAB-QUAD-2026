#!/usr/bin/env bash
# NUC → 내부 PC(Jetson): 신호등 비전 INT8 — 올리기·엔진 만들기(양자화)·채점·켜기 한 번에.
# 사용: ~/ws/junsu/onboard_vision/deploy_onboard_vision.sh [all|quick|push|build|eval|start|stop|status|bench]
#   all    push → 내부 PC 환경 확인 → build(FP16·INT8, 약 10분) → eval → NUC 에서 채점표
#   start  내부 PC 카메라(컬러만) + 신호등 노드 — go2_full_stack.sh 7·8단계가 이걸 부른다
#   bench  실제 카메라로 15초 추론(_bench 토픽에만 냄 — 러너에 안 섞임)
# 엔진 만드는 동안 NUC 가 /lowstate(로봇 제어 신호 500 Hz)를 보다가 300 Hz 아래로 두 번 떨어지면 멈춘다.
# ★ 엔진 만들기는 로봇이 엎드려 있을 때만. 내부 PC 는 로봇 펌웨어 PC 다.
set -u
D="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"; ROBOT=unitree@192.168.123.18
SSH=(ssh -o "ControlPath=$HOME/.ssh/go2_cm_%r@%h:%p" -o ControlMaster=auto -o ControlPersist=3h -o ConnectTimeout=6 -o ServerAliveInterval=5)
LOG=$D/logs; mkdir -p $LOG $D/results
# 모델: TL_MODEL=tl_cone_1104(기본, 11-04 경기장 학습) | tl5cone(예비). 예) TL_MODEL=tl5cone ./deploy_onboard_vision.sh start
export TL_MODEL=${TL_MODEL:-tl_cone_1104}
J(){ "${SSH[@]}" $ROBOT "TL_MODEL=$TL_MODEL bash ~/junsu_vision/jetson_vision.sh $*"; }
ping -c1 -W1 192.168.123.18 >/dev/null 2>&1 || { echo "★ 내부 PC(192.168.123.18) 무응답 — 로봇 전원·랜선(enp86s0) 확인"; exit 1; }
"${SSH[@]}" $ROBOT true || { echo "★ ssh 실패 — 비밀번호를 안 물으려면 한 번: ssh-copy-id $ROBOT"; exit 1; }

push(){
  rsync -a -e "${SSH[*]}" --exclude results --exclude logs --exclude '*.pt' --exclude __pycache__ \
    --exclude 'check_*' --exclude compare_eval.py --exclude 'deploy_*' --exclude models_1104 --exclude '*.bak_*' "$D/" "$ROBOT:junsu_vision/" || { echo "★ 올리기 실패"; exit 1; }
  echo "올림 → 내부 PC ~/junsu_vision ($(du -sh $D/calib | cut -f1) 보정 · $(du -sh $D/evalset | cut -f1) 평가 사진)"
  "${SSH[@]}" $ROBOT 'python3 -c "import tensorrt as t, numpy as n, cv2; print(\"  내부 PC: TensorRT\", t.__version__, \"· numpy\", n.__version__, \"· OpenCV\", cv2.__version__)"; df -h ~ | tail -1 | awk "{print \"  디스크 여유\", \$4}"' \
    || { echo "★ 내부 PC 에 tensorrt/numpy/cv2 파이썬 모듈이 없다 — JetPack 의 python3-libnvinfer 확인"; exit 1; }
}
guard(){   # $1 = 감시할 작업 pid
  ( set +u; source ~/gosu/env_drive.sh >/dev/null 2>&1; PYTHONUNBUFFERED=1 exec ros2 topic hz /lowstate -w 500 ) > $LOG/lowstate_hz.log 2>&1 &
  local HZ=$! low=0 r
  while kill -0 $1 2>/dev/null; do sleep 5
    r=$(grep -o 'average rate: [0-9.]*' $LOG/lowstate_hz.log | tail -1 | grep -o '[0-9.]*$')
    if [ -n "$r" ] && python3 -c "import sys; sys.exit(0 if float('$r') < 300 else 1)"; then low=$((low + 1)); echo "  ★ /lowstate ${r} Hz"; else low=0; fi
    if [ $low -ge 2 ]; then echo "★ 로봇 제어 신호가 흔들림 — 엔진 만들기 중단"; "${SSH[@]}" $ROBOT 'pkill -INT -f "[t]rt_build.py"'; break; fi
  done
  kill $HZ 2>/dev/null; echo "  (/lowstate 마지막 ${r:-?} Hz)"
}
build(){
  echo "=== 엔진 만들기 $(date +%T) [$TL_MODEL] — FP16 약 3분 + INT8(보정 400장) 약 7분. 로봇은 엎드린 상태로 둘 것"
  ( J build 2>&1 | tee $LOG/build_$(date +%H%M%S).log ) & local B=$!
  guard $B; wait $B
}
evaluate(){
  echo "=== 평가 $(date +%T)"; J eval || exit 1
  rsync -a -e "${SSH[*]}" "$ROBOT:junsu_vision/results/" "$D/results/" && ~/venv/go2/bin/python3 $D/compare_eval.py 2>&1 | grep -v Warn
}
case "${1:-all}" in
  all)    push; build; evaluate
          echo; echo "■ 끝 $(date +%T). 다음: 카메라로 확인 → $0 start ; $0 bench   (주행 스택은 go2_full_stack.sh 가 start 를 부른다)";;
  quick)  push   # ★ 11-05 짧은 시험: INT8 만 만들고(약 7분) 채점
          echo "=== INT8 엔진 만들기 $(date +%T) [$TL_MODEL] — 약 7분. 로봇은 엎드린 상태로"
          ( J build8 2>&1 | tee $LOG/build8_$(date +%H%M%S).log ) & B=$!; guard $B; wait $B || exit 1
          echo "=== 평가 $(date +%T)"; J eval8 || exit 1
          rsync -a -e "${SSH[*]}" "$ROBOT:junsu_vision/results/" "$D/results/" && ~/venv/go2/bin/python3 $D/compare_eval.py 2>&1 | grep -v Warn
          echo; echo "■ 끝 $(date +%T). 카메라로 확인: $0 start ; $0 bench";;
  eval8)  echo "=== 평가(INT8) $(date +%T)"; J eval8 || exit 1
          rsync -a -e "${SSH[*]}" "$ROBOT:junsu_vision/results/" "$D/results/" && ~/venv/go2/bin/python3 $D/compare_eval.py 2>&1 | grep -v Warn;;
  fp16)   push   # ★ 11-05 FP16 엔진(기본으로 씀) 만들고 채점
          echo "=== FP16 엔진 만들기 $(date +%T) [$TL_MODEL] — 로봇은 엎드린 상태로"
          ( J build16 2>&1 | tee $LOG/build16_$(date +%H%M%S).log ) & B=$!; guard $B; wait $B || exit 1
          echo "=== 평가 $(date +%T)"; J eval16 || exit 1
          rsync -a -e "${SSH[*]}" "$ROBOT:junsu_vision/results/" "$D/results/" && ~/venv/go2/bin/python3 $D/compare_eval.py 2>&1 | grep -v Warn;;
  push)   push;;
  build)  build;;
  eval)   evaluate;;
  start|stop|status|bench|stop-camera) J "$1";;
  *) echo "사용: $0 [all|quick|fp16|eval8|push|build|eval|start|stop|status|bench|stop-camera]"; exit 1;;
esac
