#!/usr/bin/env bash
# 주차(엎드림) 구간 위치 보호 재생 시험: run.sh <이름> <posture_gate true|false> <도메인 78~81>
# 16:50 재생 덤프(~/ws/junk/debug/loc/out/r1650/rec: /Odometry·/scan·/tf·/sportmodestate)를 bag 시계로 1배속 재생,
# v2 위치 감시(map_server+AMCL /junsu, junsu_loc_node)와 러너 흉내(다시 선 뒤 재정렬 요청). 지도 = 16:45 녹화 2D 지도(다른 녹화 = 대회와 같은 조건).
# 격리: 도메인 78~81 + lo 전용(v_gosu env_replay). 로봇 연결(robot-br UP)이면 거부.
set +u
NAME=$1; GATE=$2; DOM=$3
H="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"; J="$(cd "$H/../.." && pwd)"
if ip link show robot-br 2>/dev/null | grep -q 'state UP'; then echo "★ 로봇 연결됨 — 거부"; exit 1; fi
export V_DOMAIN=$DOM; source ~/ws/v_gosu/env_replay.sh
case "$ROS_DOMAIN_ID" in 78|79|80|81) ;; *) echo "★ 도메인"; exit 1;; esac
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash >/dev/null 2>&1
MAPY=${MAPY:-$J/debug/map_compare/routes/c1645_15/map.yaml}; T0=${T0:-"-0.034,0.000,0.06091"}   # 16:50 FAST-LIO → 16:45 지도(3D ICP 출발 T, +3.49°)
O=$H/out/$NAME; rm -rf $O; mkdir -p $O
PGS=(); stop(){ for p in "${PGS[@]}"; do kill -INT -- -"$p" 2>/dev/null; done; sleep 2; for p in "${PGS[@]}"; do kill -TERM -- -"$p" 2>/dev/null; done; }; trap stop EXIT
TF="-r /tf:=/tf_junsu -r /tf_static:=/tf_junsu_static"
setsid ros2 run nav2_map_server map_server --ros-args -r __ns:=/junsu --params-file $J/config/map_server_junsu.yaml -p yaml_filename:=$MAPY -p use_sim_time:=true >$O/map.log 2>&1 & PGS+=($!)
setsid ros2 run nav2_amcl amcl --ros-args -r __ns:=/junsu $TF --params-file $J/config/amcl_junsu.yaml -p use_sim_time:=true >$O/amcl.log 2>&1 & PGS+=($!)
setsid python3 $J/src/junsu_loc_node.py --ros-args -p align_file:=$O/align.json -p map_yaml:=$MAPY -p posture_gate:=$GATE -p sport_topic:=/sportmodestate -p use_sim_time:=true >$O/loc.log 2>&1 & PGS+=($!)
setsid python3 $H/mimic_runner.py --ros-args -p use_sim_time:=true >$O/mimic.log 2>&1 & PGS+=($!)
setsid python3 $H/loc_log.py --ros-args -p csv:=$O/log.csv -p use_sim_time:=true >$O/logger.log 2>&1 & PGS+=($!)
sleep 4
for n in /junsu/map_server /junsu/amcl; do for s in configure activate; do timeout 10 ros2 lifecycle set $n $s >/dev/null 2>&1; done; done
echo "$NAME gate=$GATE d$DOM: map $(timeout 5 ros2 lifecycle get /junsu/map_server 2>/dev/null | cut -d' ' -f1) amcl $(timeout 5 ros2 lifecycle get /junsu/amcl 2>/dev/null | cut -d' ' -f1) $(date +%T)"
( sleep 6; python3 -c "import json; json.dump({'ok': True, 'T': [float(v) for v in '$T0'.split(',')], 'match': 0.9}, open('$O/align.json', 'w'))" ) &
ros2 bag play ~/ws/junk/debug/loc/out/r1650/rec --clock 50 --topics /Odometry /scan /tf /tf_static /sportmodestate < /dev/null >$O/play.log 2>&1
sleep 3
echo "$NAME 끝 $(date +%T) · 로그 $(wc -l < $O/log.csv)줄"
