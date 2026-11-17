#!/usr/bin/env bash
# data4 의 시작 자세를 제대로 골라 재검증한다.
#   ① 전역 탐색으로 후보 5개
#   ② 오도메트리가 든 파생 녹화본 생성 (한 번만)
#   ③ 후보마다 25초 따라가 보고 보정이 가장 적은 것 선택
#   ④ 그 자세로 130초 추적 재검증
set -e
cd ~/ws/lab_visual
source ~/ws/lab/env.sh > /dev/null

echo "① 전역 탐색"
CAND=$(python3 tools/find_start_pose.py --bag ~/ws/lab/bags/data4 --map maps/go2_prior_map_v05.pcd \
       --at 14 --area="-6,14,-11,7" --step 0.3 2>/dev/null \
       | awk '/^   [1-5] /{gsub(/[(),°+]/,"",$0); print $3","$4","$5}' | paste -sd';')
echo "   후보: $CAND"

echo "② 파생 녹화본 생성 (오도메트리 포함)"
if [ ! -d ~/ws/lab/bags/data4_odom ]; then
    RECORD=1 TAG=rec_data4 BAG=data4 SEC=60 ~/ws/lab_visual/run_one.sh > logs/record_data4.log 2>&1
    grep -E "파생 녹화본" logs/record_data4.log || echo "   ★ 녹화 실패"
else
    echo "   이미 있음"
fi

echo "③ 후보 비교"
python3 tools/pick_start_pose.py --bag ~/ws/lab/bags/data4_odom --at 14 --sec 25 --cand "$CAND" 2>&1 | grep -v Opened | tail -10
