#!/usr/bin/env bash
# AMCL 재생 시험 한 번: amcl_run.sh <덤프 태그> <S0|S1|S2> <base|tuned> <도메인 78~81> <출발 T "x,y,yaw">
# 덤프(out/<태그>/rec: /Odometry·/scan·/tf)를 --clock 으로 재생, 오늘 2D 지도(rec1650) + nav2 AMCL + 중계(상황대로 자세 틀기). 로봇 연결 시 거부.
set +u
TAG=$1; SC=$2; PV=$3; DOM=$4; T=$5
D="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
if ip link show robot-br 2>/dev/null | grep -q 'state UP'; then echo "★ 로봇 연결됨 — 거부"; exit 1; fi
export V_DOMAIN=$DOM; source ~/ws/v_gosu/env_replay.sh
case "$ROS_DOMAIN_ID" in 78|79|80|81) ;; *) echo "★ 도메인"; exit 1;; esac
SCAN=${SCAN:-/scan}; BN=${SCAN#/scan}; BN=${BN#_}
O=$D/out/$TAG/amcl_${SC}_${PV}${BN:+_$BN}${MV:+_$MV}; mkdir -p $O; MAPY=${MV:+$D/out/maps/$MV/map.yaml}; MAPY=${MAPYAML:-${MAPY:-$HOME/ws/junk/routes/rec1650/map.yaml}}   # MAPYAML = 아무 지도(v2 옛 지도 시험)
PGS=(); stop(){ for p in "${PGS[@]}"; do kill -INT -- -"$p" 2>/dev/null; done; sleep 2; for p in "${PGS[@]}"; do kill -TERM -- -"$p" 2>/dev/null; done; }; trap stop EXIT
setsid ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$MAPY -p use_sim_time:=true >$O/map.log 2>&1 & PGS+=($!)
setsid ros2 run nav2_amcl amcl --ros-args --params-file $D/amcl_$PV.yaml -r /tf:=/tf_amcl -r /tf_static:=/tf_amcl_static -p scan_topic:=$SCAN >$O/amcl.log 2>&1 & PGS+=($!)
setsid python3 $D/amcl_relay.py --ros-args -p scen:=$SC -p "T:=[$T]" -p csv:=$O/poses.csv -p use_sim_time:=true >$O/relay.log 2>&1 & PGS+=($!)
sleep 4
for n in map_server amcl; do for s in configure activate; do timeout 10 ros2 lifecycle set /$n $s >/dev/null 2>&1; done; done
echo "$TAG $SC $PV d$DOM: map $(timeout 5 ros2 lifecycle get /map_server 2>/dev/null | cut -d' ' -f1) amcl $(timeout 5 ros2 lifecycle get /amcl 2>/dev/null | cut -d' ' -f1) $(date +%T)"
ros2 bag play -i $D/out/$TAG/rec ${BANDS:+-i $BANDS} --clock 50 --topics /Odometry $SCAN /tf < /dev/null >$O/play.log 2>&1
sleep 2
echo "$TAG $SC $PV 끝 $(date +%T) · amcl 자세 $(grep -c '^amcl' $O/poses.csv)개"
