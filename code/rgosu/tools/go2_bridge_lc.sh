# go2_bridge(review, ~/ws/merge_tmp_20261102/review) 는 11-03 커밋부터 lifecycle 노드 — 띄운 뒤 configure → activate 해야 토픽이 나온다.
# 그냥 띄우면 /velodyne_points·/imu/body 가 안 나와 FAST-LIO 입력이 없고, cmd_vel_to_sport 는 모터 명령을 안 낸다(11-05 병합 세션 확인).
# 설정 파일도 꼭 줄 것 — 안 주면 직진 쏠림 보정(yaw_trim 0.110/-0.002)·min_vx 등이 코드 기본값.
# 사용: source go2_bridge_lc.sh;  ros2 run go2_bridge cloud_bridge_node --ros-args --params-file $BRIDGE_PARAMS ... & ;  lc_up go2_cloud_bridge
BRIDGE_PARAMS=${BRIDGE_PARAMS:-$HOME/ws/merge_tmp_20261102/review/install/go2_bridge/share/go2_bridge/config/bridge_params.yaml}
lc_up(){   # $1 = 노드 이름(go2_cloud_bridge · go2_imu_bridge · go2_cmd_vel_to_sport)
  local n=$1 s=""
  for i in $(seq 1 10); do
    s=$(ros2 lifecycle get /$n 2>/dev/null | awk '{print $1}')
    case "$s" in
      active) echo "  /$n active"; return 0;;
      unconfigured) ros2 lifecycle set /$n configure >/dev/null 2>&1;;
      inactive) ros2 lifecycle set /$n activate >/dev/null 2>&1;;
    esac
    sleep 1
  done
  echo "★ /$n 활성화 실패(상태: ${s:-노드 없음}) — 로그 확인"; return 1
}
