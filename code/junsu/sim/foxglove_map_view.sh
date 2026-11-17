#!/usr/bin/env bash
# Foxglove 로 지도 보기 — foxglove_bridge(포트 8765) + 지도 발행기. 격리 도메인 79·lo 전용(주행 스택과 안 섞임), /viz/* 만 낸다.
# 끄기: Ctrl+C.  ★ 경기 중에는 켜지 말 것(규정 3.3 센서 데이터 외부 전송 금지).
set -u
export V_DOMAIN=79; source ~/ws/v_gosu/env_replay.sh
case "$ROS_DOMAIN_ID" in 78|79|80|81) ;; *) echo "★ 격리 도메인 아님"; exit 1;; esac
PGS=(); trap 'for p in "${PGS[@]}"; do kill -INT -- -$p 2>/dev/null; done' EXIT
setsid ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765 -p address:=0.0.0.0 -p send_buffer_limit:=100000000 > /tmp/foxglove_bridge_viz.log 2>&1 & PGS+=($!)
sleep 2
echo "도메인 $ROS_DOMAIN_ID · Foxglove 접속: ws://<NUC 주소>:8765  (NUC 화면 브라우저면 ws://localhost:8765)"
for ip in $(ip -br -4 addr | awk '$1!="lo" && $1!~/docker/{split($3,a,"/"); print $1"="a[1]}'); do echo "  ${ip%%=*}: ws://${ip#*=}:8765"; done
~/venv/go2/bin/python3 ~/ws/junsu/sim/foxglove_map_pub.py
