#!/usr/bin/env bash
# junsu 올인원 주행 관리자 실행기 — 환경을 go2_full_stack.sh 와 같은 순서로 맞추고 go2_ctl.py 를 띄운다.
# 실제 로봇:   ~/ws/junsu/tools/go2_ctl.sh
# 재생 시험:   GO2_CTL_REPLAY=80 ~/ws/junsu/tools/go2_ctl.sh   (격리 도메인 78~81 + lo 전용. 로봇이 응답하면 거부)
set +u
J="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"   # 이 저장소(브랜치 폴더면 그 폴더)
if [ -n "${GO2_CTL_REPLAY:-}" ]; then
  for h in 161 18; do ping -c1 -W1 192.168.123.$h >/dev/null 2>&1 && { echo "★ 로봇(.$h) 응답 — 재생 모드 거부"; exit 1; }; done
  export V_DOMAIN=$GO2_CTL_REPLAY; source ~/ws/v_gosu/env_replay.sh
  case "$ROS_DOMAIN_ID" in 78|79|80|81) ;; *) echo "★ 격리 아님"; exit 1;; esac
else
  source ~/setup_go2_ros2.sh
  export CYCLONEDDS_URI=file://$J/config/cyclonedds_junsu.xml   # ★ 11-05: 데이터 유니캐스트(멀티캐스트면 점구름이 robot-br 100 Mb/s 포트로 넘쳐 XT16 7.6 Hz·로봇 링크 포화)
fi
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash >/dev/null 2>&1
source ~/ws/merge_tmp_20261102/review/install/setup.bash >/dev/null 2>&1
source ~/ws/v_gosu/ws_legfuse/install/setup.bash >/dev/null 2>&1
source $J/ws_fastlio_tunable/install/setup.bash >/dev/null 2>&1
source $J/ws_ego_planner_tunable/install/setup.bash >/dev/null 2>&1
source $J/ws_wall_filter/install/setup.bash >/dev/null 2>&1
source $J/ws_mission_vision/install/setup.bash >/dev/null 2>&1   # merge 콘 추적기(라이다)
exec python3 $J/tools/go2_ctl.py "$@"
