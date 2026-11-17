#!/usr/bin/env bash
# ★ 작전명 ego-plus (2026-11-05 아침) — merge_tmp_20261102/review(팀원 공유, 읽기 전용 참고)의
# AMCL 위치추정(3cm 정확도) + 라이다 전용 콘 추적(10/10 검증, 카메라/YOLO 불필요) + ego_planner로
# 기존 작전명 ego/noego의 위치추정·장애물 레이어를 갈아끼운 버전. real_course_runner.py는
# 전혀 안 바꿈(GOAL_FRAME 환경변수만 'map'으로 줘서 이 스택에 맞춤, 기본값 'camera_init'은
# 그대로라 기존 ego/noego 명령은 100% 그대로 동작함).
#
# ★★★ 아직 로봇으로 직접 검증 못 함 — 로봇을 다른 세션(junsu)이 쓰고 있어서 못 띄워봤음.
# 처음 쓸 때 반드시 --no-motor(이 스크립트의 NOMOTOR=1)로 먼저 켜서 go2_goto.py pose로
# 위치가 말이 되는지 확인부터 할 것. merge_tmp_20261102는 참고만 하고 안에 파일은 안 건드림.
#
# 사용:
#   ~/ws/Rgosu/tools/go2_full_stack_amcl.sh                  # 모터 켬(실제 주행)
#   NOMOTOR=1 ~/ws/Rgosu/tools/go2_full_stack_amcl.sh        # 모터 끔(위치 확인 전용, 절대 안 움직임)
#   START="0.045 0.12 0" ~/ws/Rgosu/tools/go2_full_stack_amcl.sh   # 시작 자세(x y yaw도, 기본값 = 우리 start 지점)
# 정지: 이 터미널에서 Ctrl+C(review의 go2_run.sh가 알아서 전부 정리 -- lifecycle shutdown + StopMove).
#   안 꺼지면 ~/ws/Rgosu/tools/go2_stop_all.sh
set -u
REVIEW=~/ws/merge_tmp_20261102/review
MAP=${MAP:-$REVIEW/maps/arena1459_amcl.yaml}   # 14:59 녹화 기반 -- 오늘 밤 waypoints랑 같은 녹화라 좌표가 그대로 맞을 가능성 높음
START=${START:-"0.045 0.12 0"}                  # 우리 real_waypoints_venue_run.json의 'start'(0.045,0.12), yaw는 추정치
NOMOTOR=${NOMOTOR:-0}

echo "================================================================"
echo "  작전명 EGO-PLUS (AMCL+라이다콘, merge_tmp_20261102/review 기반)"
echo "  map=$MAP"
echo "  start=\"$START\""
[ "$NOMOTOR" = 1 ] && echo "  ★ NOMOTOR=1 -- 모터 꺼짐, 로봇 절대 안 움직임(위치 확인 전용)"
echo "================================================================"

if [ ! -f "$MAP" ]; then
  echo "★ 지도 파일 없음: $MAP"; exit 1
fi

# review 쪽 노드(hesai/cloud_bridge/fastlio/ego_planner 등)가 이미 떠 있으면(다른 세션이 쓰는 중)
# 절대 같이 띄우지 않는다 -- 이름이 겹쳐서 충돌한다.
if pgrep -f "hesai_ros_driver_node|cloud_bridge_node|fastlio_mapping|ego_planner_node|amcl$|map_server$|cone_tracker_node" >/dev/null; then
  echo "★ 이미 관련 노드가 떠 있음(다른 세션이 로봇을 쓰고 있을 수 있음) — 확인 후 다시 실행할 것:"
  pgrep -af "hesai_ros_driver_node|cloud_bridge_node|fastlio_mapping|ego_planner_node|amcl$|map_server$|cone_tracker_node"
  exit 1
fi

MOTOR_ARGS=()
[ "$NOMOTOR" = 1 ] && MOTOR_ARGS+=(--no-motor)

cd "$REVIEW"
exec ./scripts/go2_run.sh --map "$MAP" --start "$START" --cones "${MOTOR_ARGS[@]}"
