#!/usr/bin/env bash
# 격리된 우리 스택을 브라우저에서 본다. 팀원 화면(포트 8765)과 따로 돈다.
#
#   ./run_gui.sh              포트 8766 으로 띄운다
#   PORT=8767 ./run_gui.sh    다른 포트
#   LIGHT=1 ./run_gui.sh      스캔 점군을 빼고 가볍게 (젯슨이 바쁠 때)
#
# 붙는 법
#   브라우저에서 https://app.foxglove.dev  ->  Open connection  ->  Foxglove WebSocket
#   ws://<젯슨주소>:8766
#   (같은 망이면 ws://go2-jetson.local:8766, Tailscale 이면 ws://100.66.225.8:8766)
#
# 【무엇을 보면 되나】
#   map/points        사전 지도 (흰색으로 두면 좋다)
#   /velodyne_points  지금 라이다 스캔
#   -> 이 둘이 겹쳐 보이면 정합이 맞는 것이다. 벽이 두 겹으로 보이면 틀어진 것이다.
#   localization/status  rms=… healthy=…   잔차가 0.06 m 를 넘으면 의심
#   odometry/health      ok=1 이어야 정상. ok=0 이면 발산이다
#   TF                map -> odom -> base_link
#
# 【부하 주의】
#   foxglove_bridge 는 실측 CPU 22.5% 를 먹는다. 젯슨이 이미 바쁘면 LIGHT=1 로 띄우거나
#   아예 띄우지 마라. 토픽 목록을 좁혀 놓은 이유도 그것이다.
set -u
LAB=~/gosu/lab_visual
PORT="${PORT:-8766}"
LIGHT="${LIGHT:-0}"

# ROS 의 setup.bash 는 안 정해진 변수를 건드린다. set -u 와 부딪히므로 잠시 푼다.
set +u
source "$LAB/env.sh"
set -u
bash "$LAB/check_isolation.sh" || { echo "중단한다."; exit 1; }

# 이 포트를 이미 누가 쓰고 있으면 멈춘다 — 팀원 화면을 빼앗으면 안 된다
if ss -lnt 2>/dev/null | grep -q ":$PORT "; then
    echo "★ 포트 $PORT 를 이미 누가 쓰고 있다. PORT=다른번호 로 다시 실행할 것"
    ss -lntp 2>/dev/null | grep ":$PORT "
    exit 1
fi

TOPICS='["/tf","/tf_static","map/points","localization/pose","localization/status","odometry/health","/Odometry","traffic_light/state","traffic_light/cross_allowed"]'
if [ "$LIGHT" != "1" ]; then
    TOPICS='["/tf","/tf_static","map/points","/velodyne_points","localization/pose","localization/status","odometry/health","/Odometry","traffic_light/state","traffic_light/cross_allowed","traffic_light/annotated/compressed"]'
fi

L="$LAB/logs"; mkdir -p "$L"
echo "══ 화면 서버 (포트 $PORT, 도메인 77) ══"
echo "  보낼 토픽: $TOPICS"
[ "$LIGHT" = "1" ] && echo "  가벼운 모드 — 스캔 점군은 안 보낸다"

# 지도를 먼저 올려 둔다 (latched 라 뷰어가 늦게 붙어도 받는다)
python3 "$LAB/src/map_publisher.py" --map "$LAB/maps/go2_prior_map_v05.pcd" \
        --voxel 0.12 --band > "$L/map_pub.log" 2>&1 &
MP=$!
sleep 2

ros2 run foxglove_bridge foxglove_bridge --ros-args \
    -p port:="$PORT" -p address:=0.0.0.0 \
    -p topic_whitelist:="$TOPICS" \
    -p max_qos_depth:=2 -p send_buffer_limit:=5000000 \
    -p use_compression:=true > "$L/foxglove.log" 2>&1 &
FG=$!
sleep 3

if ss -lnt 2>/dev/null | grep -q ":$PORT "; then
    IP=$(ip -4 addr show | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | grep -v 127.0.0.1 | head -3 | tr '\n' ' ')
    echo
    echo "✔ 떴다. 브라우저에서 https://app.foxglove.dev 열고 Foxglove WebSocket 으로:"
    for a in $IP; do echo "     ws://$a:$PORT"; done
    echo "     ws://go2-jetson.local:$PORT   (같은 망일 때)"
    echo
    echo "  지도(map/points)와 스캔(/velodyne_points)이 겹치면 정합이 맞는 것이다."
    echo "  Ctrl-C 로 끈다."
else
    echo "✗ 포트가 안 열렸다. 로그: $L/foxglove.log"
    tail -5 "$L/foxglove.log"
fi

trap 'echo; echo "  끈다"; kill $FG $MP 2>/dev/null; exit 0' INT TERM
wait $FG
kill $MP 2>/dev/null
