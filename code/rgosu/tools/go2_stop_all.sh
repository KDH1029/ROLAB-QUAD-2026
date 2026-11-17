#!/usr/bin/env bash
# 작전명 ego/noego(또는 그 전 어떤 조합이든) 전체 정리 — 동료들이 자기 작전으로 바로 이어받을 수
# 있게 NUC·내부 PC(Jetson)에 떠 있는 우리 쪽 노드를 전부 끈다. 2026-11-05 밤.
#   go2_full_stack.sh가 띄운 것 + real_course_runner.py + (VISION=nuc였으면) 그쪽 카메라/신호등도 다 포함.
# 사용: ~/ws/Rgosu/tools/go2_stop_all.sh
set +u
echo "=== 작전 전체 정리 시작 $(date +%T) ==="
echo "★★★ 주의: 이 스크립트는 '누가 띄웠든' 이름이 같은 프로세스를 전부 끔(workspace 구분 안 함)."
echo "    지금 동료가 다른 workspace에서 같은 이름의 노드(hesai/FAST-LIO/ego_planner 등)로"
echo "    테스트 중이면 그것도 같이 꺼짐 — 우리 작전(ego/noego/ego-plus)에서 다른 작전으로"
echo "    넘어갈 때만 쓸 것, 동료가 테스트 중인지 먼저 확인할 것."

# ① cmd_vel_to_sport_node는 반드시 SIGINT부터 -- 그래야 마지막에 StopMove가 나가서 로봇이 확실히 멈춘다.
#    -9로 바로 죽이면 로봇이 마지막 속도 명령을 계속 수행 중인 채로 멈출 위험이 있음(go2_full_stack.sh
#    상단 "비상정지" 설명과 동일한 이유).
if pgrep -f "[c]md_vel_to_sport_node" >/dev/null; then
  pkill -INT -f "[c]md_vel_to_sport_node"
  echo "  cmd_vel_to_sport_node: SIGINT 보냄(StopMove 전송 대기)"
  sleep 1.5
else
  echo "  cmd_vel_to_sport_node: 안 떠있음"
fi

# ② 러너도 먼저 — 계속 목표를 보내고 있으면 안 되니까.
if pgrep -f "[r]eal_course_runner.py" >/dev/null; then
  pkill -INT -f "[r]eal_course_runner.py"; sleep 0.5
  echo "  real_course_runner.py: 끔"
else
  echo "  real_course_runner.py: 안 떠있음"
fi

# ③ NUC 쪽 나머지 전부 — go2_full_stack.sh(작전명 ego/noego) + go2_full_stack_amcl.sh(작전명
#    ego-plus, merge_tmp_20261102/review 기반 AMCL+라이다콘)가 띄우는 것 전부 포함.
NUC_PATTERN="hesai_ros_driver_node|cloud_bridge_node|imu_bridge_node|fastlio_mapping|wall_filter_node|obstacle_fusion_node|cone_raw_filter_node|ego_planner_node|realsense2_camera|traffic_light_detector_node|amcl$|map_server$|lifecycle_manager|cone_tracker_node|odom_tf_node|go2_run.sh"
if pgrep -f "($NUC_PATTERN)" >/dev/null; then
  pkill -INT -f "($NUC_PATTERN)"; sleep 1
  pkill -9 -f "($NUC_PATTERN)" 2>/dev/null   # 그래도 안 죽으면(드물게 SIGINT 안 받는 노드) 강제
  echo "  NUC 쪽 드라이버/노드: 끔(hesai·go2_bridge·FAST-LIO·wall_filter/obstacle_fusion·AMCL/map_server·cone_tracker·ego_planner·카메라·신호등)"
else
  echo "  NUC 쪽 드라이버/노드: 안 떠있음"
fi

# ④ 내부 PC(Jetson) — VISION=onboard였으면 거기서 카메라+신호등(+콘)이 돌고 있음. 응답 안 하면 그냥 넘어감(로봇이 꺼져있을 수도 있음).
if ping -c1 -W1 192.168.123.18 >/dev/null 2>&1; then
  ssh -o ControlPath=~/.ssh/go2_cm_%r@%h:%p -o ControlMaster=auto -o ControlPersist=3h -o ConnectTimeout=5 \
    unitree@192.168.123.18 'bash ~/onboard_vision/jetson_vision.sh stop; bash ~/onboard_vision/jetson_vision.sh stop-camera' 2>&1 \
    | sed 's/^/  내부 PC: /'
else
  echo "  내부 PC(192.168.123.18): 무응답 — 건너뜀(이미 꺼져있거나 랜선 안 꽂혀있을 수 있음)"
fi

sleep 1
LEFT=$(pgrep -f "($NUC_PATTERN|[c]md_vel_to_sport_node|[r]eal_course_runner.py)" 2>/dev/null)
if [ -n "$LEFT" ]; then
  echo "★ 아직 남아있는 프로세스 있음(확인 필요): $LEFT"
else
  echo "=== 정리 완료 $(date +%T) — NUC·내부 PC 모두 깨끗함, 동료 작전 바로 띄워도 됨 ==="
fi
