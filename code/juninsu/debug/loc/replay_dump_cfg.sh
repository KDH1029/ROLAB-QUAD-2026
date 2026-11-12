#!/usr/bin/env bash
# 위치추정 검증 재료: 녹화 bag → (대회와 같은) go2_bridge + junk FAST-LIO 를 격리 재생 → 출력만 다시 녹화(mcap).
# 사용: replay_dump.sh <bag> <태그> [도메인 78~81]      결과: ~/ws/junk/debug/loc/out/<태그>/rec (mcap) + 로그
# 격리: 도메인 78~81 + lo 전용(v_gosu env_replay). 로봇이 연결돼 있으면(robot-br UP) 거부. 시각은 bag 시계(--clock, use_sim_time).
set +u
BAG="${1:?bag}"; TAG="${2:?태그}"; DOM="${3:-78}"
if ip link show robot-br 2>/dev/null | grep -q 'state UP'; then echo "★ 로봇 연결됨(robot-br UP) — 재생 거부"; exit 1; fi
export V_DOMAIN=$DOM; source ~/ws/v_gosu/env_replay.sh
source ~/ws/merge_tmp_20261102/review/install/setup.bash          # 대회와 같은 브리지(/scan 포함)
source ~/ws/junk/ws_fastlio_tunable/install/setup.bash            # 대회와 같은 FAST-LIO(다리 융합·게이트)
case "$ROS_DOMAIN_ID" in 78|79|80|81) ;; *) echo "★ 도메인 $ROS_DOMAIN_ID — 78~81 만"; exit 1;; esac
OUT=~/ws/junk/debug/loc/out/$TAG; rm -rf "$OUT/rec"; mkdir -p "$OUT"
BP=~/ws/merge_tmp_20261102/review/install/go2_bridge/share/go2_bridge/config/bridge_params.yaml
PGS=()
stop(){ for p in "${PGS[@]}"; do kill -INT -- -"$p" 2>/dev/null; done; sleep 3; for p in "${PGS[@]}"; do kill -TERM -- -"$p" 2>/dev/null; done; }
trap stop EXIT
echo "=== $TAG  도메인 $ROS_DOMAIN_ID  $(date +%T)"
setsid ros2 run go2_bridge imu_bridge_node --ros-args --params-file $BP -p use_sim_time:=true >"$OUT/imu.log" 2>&1 & PGS+=($!)
setsid ros2 run go2_bridge cloud_bridge_node --ros-args --params-file $BP -p use_sim_time:=true >"$OUT/cloud.log" 2>&1 & PGS+=($!)
setsid ros2 run fast_lio fastlio_mapping --ros-args --params-file ${LIO_CFG:-$HOME/ws/junk/config/fastlio_leg_nurigwan_derange30.yaml} -p use_sim_time:=true >"$OUT/lio.log" 2>&1 & PGS+=($!)
sleep 4
for n in go2_imu_bridge go2_cloud_bridge; do
  for s in configure activate; do timeout 10 ros2 lifecycle set /$n $s >/dev/null 2>&1; done
  echo "  $n: $(timeout 8 ros2 lifecycle get /$n 2>/dev/null | head -1)"
done
setsid ros2 bag record -s mcap -o "$OUT/rec" --use-sim-time /Odometry /cloud_registered_body /scan /tf /tf_static \
  /utlidar/robot_odom /sportmodestate /clock >"$OUT/record.log" 2>&1 & RPG=$!
sleep 3
ros2 bag play "$BAG" --clock 100 --topics /lidar_points /lowstate /utlidar/robot_odom /sportmodestate < /dev/null >"$OUT/play.log" 2>&1
sleep 3
kill -TERM -- -"$RPG" 2>/dev/null; sleep 4          # 백그라운드 bag record 는 SIGINT 를 무시함 → SIGTERM
echo "=== $TAG 끝 $(date +%T)  $(du -sh "$OUT/rec" | cut -f1)  FAST-LIO 경고: $(grep -ciE 'error|degenerat' "$OUT/lio.log")"
