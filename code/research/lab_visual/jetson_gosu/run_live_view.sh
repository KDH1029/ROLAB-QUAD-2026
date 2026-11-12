#!/usr/bin/env bash
# 실제 주행을 브라우저에서 본다. **지도와 스캔만.** 읽기만 하고 아무것도 내지 않는다.
#
#   ./run_live_view.sh                 포트 8766
#   PORT=8767 ./run_live_view.sh       다른 포트
#   WITH_CLOUD=1 ./run_live_view.sh    3D 점군(/cloud_registered)도 같이 (무겁다)
#
# 붙는 법
#   https://app.foxglove.dev  ->  Open connection  ->  Foxglove WebSocket
#   ws://100.66.225.8:8766     (Tailscale)
#   ws://go2-jetson.local:8766 (같은 망)
#
# 【이건 라이브 도메인에 붙는다 — 격리가 아니다】
#   주행 중인 스택을 봐야 하므로 도메인 0 에 참여한다. 대신 세 겹으로 막았다:
#     ① 토픽 화이트리스트 — 지도와 스캔과 TF 만 받는다
#     ② capabilities 에서 clientPublish / services / parameters 를 뺐다
#        -> 브라우저에서 토픽을 낼 수도, 서비스를 부를 수도, 파라미터를 바꿀 수도 없다
#     ③ 이 스크립트는 구독만 하는 노드 하나만 띄운다. 발행하는 노드가 없다
#   그래도 DDS 참여자가 하나 늘고 CPU 를 조금 쓴다. 그건 격리로 못 없앤다.
#
# 【무엇을 보나】
#   /map   지도 (2D 점유격자)
#   /scan  지금 라이다 (2D)
#   이 둘이 겹치면 위치추정이 맞는 것이다. 벽이 어긋나면 틀어진 것이다.
set -u
PORT="${PORT:-8766}"
WITH_CLOUD="${WITH_CLOUD:-0}"

set +u
source /opt/ros/humble/setup.bash
[ -f ~/go2_ws/install/setup.bash ] && source ~/go2_ws/install/setup.bash
set -u
# 라이브 도메인. 격리 변수를 쓰지 않는다 — 주행 스택을 봐야 하기 때문이다.
unset ROS_LOCALHOST_ONLY CYCLONEDDS_URI 2>/dev/null || true
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_LIVE:-0}"

echo "══ 라이브 화면 (도메인 $ROS_DOMAIN_ID, 포트 $PORT) — 읽기 전용 ══"

if ss -lnt 2>/dev/null | grep -q ":$PORT "; then
    echo "★ 포트 $PORT 를 이미 누가 쓰고 있다. PORT=다른번호 로 실행할 것"
    ss -lntp 2>/dev/null | grep ":$PORT "
    exit 1
fi

# 주행 스택이 떠 있는지 먼저 본다. 없으면 볼 게 없다.
echo "  주행 스택 확인 중…"
NODES=$(timeout 10 ros2 node list 2>/dev/null | wc -l)
echo "  노드 $NODES개"
if [ "$NODES" -eq 0 ]; then
    echo "  ★ 주행 스택이 안 떠 있다. 먼저 스택을 띄우고 다시 실행할 것."
    exit 1
fi
echo "  있는 토픽 중 볼 것:"
TOPICS_FOUND=$(timeout 10 ros2 topic list 2>/dev/null | grep -xE "/map|/scan|/tf|/tf_static|/cloud_registered|/velodyne_points" | sed 's/^/    /')
echo "${TOPICS_FOUND:-    (없음 — 토픽 이름이 다를 수 있다)}"

WL='["/map","/scan","/tf","/tf_static"]'
[ "$WITH_CLOUD" = "1" ] && WL='["/map","/scan","/tf","/tf_static","/cloud_registered"]'

L=~/gosu/lab_visual/logs; mkdir -p "$L"
echo "  보낼 토픽: $WL"
ros2 run foxglove_bridge foxglove_bridge --ros-args \
    -p port:="$PORT" -p address:=0.0.0.0 \
    -p topic_whitelist:="$WL" \
    -p capabilities:='["connectionGraph"]' \
    -p max_qos_depth:=1 -p send_buffer_limit:=3000000 \
    -p use_compression:=true > "$L/live_view.log" 2>&1 &
FG=$!
sleep 4

if ss -lnt 2>/dev/null | grep -q ":$PORT "; then
    echo
    echo "✔ 떴다. 브라우저에서 https://app.foxglove.dev :"
    ip -4 addr show 2>/dev/null | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | grep -v 127.0.0.1 \
        | head -4 | sed 's|^|     ws://|; s|$|:'"$PORT"'|'
    echo "     ws://go2-jetson.local:$PORT"
    echo
    echo "  3D 패널에서 Map 과 LaserScan 을 켜고 고정 프레임을 map 으로 두면 된다."
    echo "  Ctrl-C 로 끈다."
else
    echo "✗ 포트가 안 열렸다. 로그: $L/live_view.log"
    tail -8 "$L/live_view.log"
    exit 1
fi

trap 'echo; echo "  끈다"; kill $FG 2>/dev/null; exit 0' INT TERM
wait $FG
