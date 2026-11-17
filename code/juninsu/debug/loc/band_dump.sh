#!/usr/bin/env bash
# AMCL 링 높이 시험용 스캔: 원본 bag 을 브리지(대회와 같은 go2_bridge)로 재생하며 높이대별 AMCL 스캔을 동시에 만든다.
# 사용: band_dump.sh <원본 bag> <태그> <도메인>     결과: out/<태그>/bands (mcap: /scan_L /scan_D /scan_M /scan_U)
# 높이대 = base_stab(몸 중심, 중력 정렬) z. 몸 중심은 바닥 위 약 0.31 m → 바닥 기준 = z + 0.31.
#   L -0.26~0.00 (바닥 위 0.05~0.31) · D -0.02~0.26 (브리지 기본값, 0.29~0.57, 작은 콘 때 정한 값) · M 0.10~0.40 (0.41~0.71) · U 0.30~0.90 (0.61~1.21)
set +u
BAG=$1; TAG=$2; DOM=$3
if ip link show robot-br 2>/dev/null | grep -q 'state UP'; then echo "★ 로봇 연결됨 — 거부"; exit 1; fi
export V_DOMAIN=$DOM; source ~/ws/v_gosu/env_replay.sh; source ~/ws/merge_tmp_20261102/review/install/setup.bash
case "$ROS_DOMAIN_ID" in 78|79|80|81) ;; *) echo "★ 도메인"; exit 1;; esac
O=~/ws/junk/debug/loc/out/$TAG; rm -rf $O/bands
BP=~/ws/merge_tmp_20261102/review/install/go2_bridge/share/go2_bridge/config/bridge_params.yaml
PGS=(); stop(){ for p in "${PGS[@]}"; do kill -INT -- -"$p" 2>/dev/null; done; sleep 2; for p in "${PGS[@]}"; do kill -TERM -- -"$p" 2>/dev/null; done; }; trap stop EXIT
setsid ros2 run go2_bridge imu_bridge_node --ros-args --params-file $BP -p use_sim_time:=true >$O/band_imu.log 2>&1 & PGS+=($!)
declare -A ZL=([L]=-0.26 [D]=-0.02 [M]=0.10 [U]=0.30) ZH=([L]=0.00 [D]=0.26 [M]=0.40 [U]=0.90)
for b in L D M U; do
  setsid ros2 run go2_bridge cloud_bridge_node --ros-args --params-file ~/ws/junk/debug/loc/band_cloud_params.yaml -r __node:=cb_$b -p use_sim_time:=true \
    -p output_topic:=/vp_$b -p scan_topic:=/scan_all_$b -p amcl_scan_topic:=/scan_$b -p amcl_min_z:=${ZL[$b]} -p amcl_max_z:=${ZH[$b]} >$O/band_$b.log 2>&1 & PGS+=($!)
done
sleep 4
for n in go2_imu_bridge cb_L cb_D cb_M cb_U; do for s in configure activate; do timeout 10 ros2 lifecycle set /$n $s >/dev/null 2>&1; done; echo -n "$n:$(timeout 5 ros2 lifecycle get /$n 2>/dev/null | cut -d' ' -f1) "; done; echo
setsid ros2 bag record -s mcap -o $O/bands --use-sim-time /scan_L /scan_D /scan_M /scan_U >$O/band_rec.log 2>&1 & RPG=$!
sleep 3
ros2 bag play "$BAG" --clock 100 --topics /lidar_points /lowstate /utlidar/robot_odom < /dev/null >$O/band_play.log 2>&1
sleep 3; kill -TERM -- -"$RPG" 2>/dev/null; sleep 3
echo "밴드 덤프 끝 $(date +%T) $(du -sh $O/bands | cut -f1)"
