#!/usr/bin/env bash
# xentry: 도메인 78 에서 run_sim_v2.sh 한 판 + ego 진단 기록기. 사용: run_rec.sh <이름(xentry_ 로 시작)> <경로파일> [그 밖 환경변수는 앞에]
set +u
NAME=$1; ROUTE=$2; D=78; X=$HOME/ws/junsu_v2/debug/agents/xentry; mkdir -p $X/rec
ip link show robot-br 2>/dev/null | grep -q 'state UP' && { echo "★ 로봇 연결됨 — 거부"; exit 1; }
for h in 161 18; do ping -c1 -W1 192.168.123.$h >/dev/null 2>&1 && { echo "★ 로봇 응답 — 거부"; exit 1; }; done
( export V_DOMAIN=$D; source ~/ws/v_gosu/env_replay.sh >/dev/null 2>&1; source ~/unitree_ros2/cyclonedds_ws/install/setup.bash >/dev/null 2>&1
  [ "$ROS_DOMAIN_ID" = 78 ] || exit 1
  exec setsid python3 $X/scripts/ego_rec.py $X/rec/$NAME.jsonl ) > $X/rec/$NAME.rec.log 2>&1 < /dev/null &
RP=$!; echo "rec pid $RP"
cd $HOME/ws/junsu_v2/sim && ./run_sim_v2.sh $NAME $D $ROUTE
kill -TERM $RP 2>/dev/null; sleep 1; kill -0 $RP 2>/dev/null && kill -KILL $RP
~/venv/go2/bin/python3 $X/scripts/entry_scan.py $NAME
