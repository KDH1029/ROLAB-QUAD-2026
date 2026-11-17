#!/usr/bin/env bash
# 주행 직전에 장애물 층을 지금 부팅 자세에 맞춰 굽는다.
#
#   ./prepare_planner_map.sh
#
# 【왜 매번 해야 하나】
# ego_planner 의 grid_map.cpp loadPriorMap 은 파일 좌표를 **그대로 월드 좌표로 쓴다**.
# 그런데 계획기의 월드는 `camera_init` — FAST-LIO 가 부팅한 자리다.
# 층 파일은 지도(map) 좌표라, 부팅 위치가 다르면 층 전체가 그만큼 밀린다 (실측 0.9~10.3 m).
# 그래서 부팅할 때마다 지금의 map->odom 으로 한 번 돌려 줘야 한다.
#
# 【순서가 중요하다】
#   ① FAST-LIO + 브리지를 띄운다            (경로계획은 아직 띄우지 않는다)
#   ② 정합(icp_localization)을 띄우고 초기자세를 찍어 자리 잡을 때까지 기다린다
#   ③ **이 스크립트**                        -> room_obstacles_ready.bin
#   ④ 그 다음에 ego_planner 를 띄운다
# FAST-LIO 를 다시 켜면 camera_init 이 바뀌므로 ③부터 다시 해야 한다.
#
# 파일을 못 만들면 계획기는 경고를 남기고 실시간 라이다만 쓴다 — 안전한 쪽으로 떨어진다.
set -u
LAB=~/ws/lab_visual
SRC="$LAB/maps/room_obstacles_map.bin"
OUT="$LAB/maps/room_obstacles_ready.bin"
WAIT="${WAIT:-30}"

[ -f "$SRC" ] || { echo "★ 원본 층이 없다: $SRC"
                   echo "  먼저 만들 것:"
                   echo "    python3 $LAB/tools/make_obstacle_layer.py \\"
                   echo "        --area=\"-1,10,-7,3.5\" --zmin 0.35 --zmax 1.60 --min-hits 30 \\"
                   echo "        --out $SRC --check"
                   exit 1; }

echo "══ 장애물 층을 지금 부팅 자세에 맞춘다 ══"
# 낡은 파일을 남겨 두면 틀린 층으로 주행하게 된다. 먼저 지운다.
if [ -f "$OUT" ]; then
    echo "  이전 파일을 지운다 ($(date -r "$OUT" '+%m-%d %H:%M') 에 만든 것)"
    rm -f "$OUT"
fi

python3 "$LAB/tools/align_obstacle_layer.py" --src "$SRC" --from-tf \
        --tf-wait "$WAIT" --out "$OUT" --check
rc=$?
echo
if [ $rc -eq 0 ] && [ -f "$OUT" ]; then
    echo "✔ 준비됨: $OUT"
    echo "  이제 ego_planner 를 띄우면 된다."
else
    echo "✗ 못 만들었다. 계획기는 실시간 라이다만 쓰게 된다 (그래도 주행은 된다)."
    echo "  TF map->odom 이 안 나온다면 정합이 자리를 잡았는지 확인할 것."
fi
exit $rc
