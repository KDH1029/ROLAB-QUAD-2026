#!/usr/bin/env bash
# ① rosbag → 지도. 녹화(새 콘이 놓인 코스를 리모컨으로 한 바퀴)를 FAST-LIO 로 다시 돌려 3D 지도(2 cm)와 2D 지도를 만든다.
# 사용: ~/ws/junk/tools/make_map.sh <bag 폴더> <이름>        예) make_map.sh ~/bags/gosu_20261105_180000 final1
# 결과: ~/ws/junk/routes/<이름>/map.npz(3D 복셀·궤적) · map.pgm/yaml(2D, 출발 정렬용)   → 다음: route_editor.py <이름>
# 걸리는 시간 ≈ bag 길이 + 1분. bag 에 /lidar_points 와 /lowstate 가 있어야 한다(~/go2_record/go2_record_auto.sh 녹화면 있음).
set -u
J="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
BAG="${1:?사용: make_map.sh <bag 폴더> <이름>}"; NAME="${2:?이름}"
bash $J/tools/map_replay.sh "$BAG" "$NAME" || exit 1
[ -s $J/routes/$NAME/map.npz ] || { echo "★ 지도가 안 만들어짐 — $J/routes/$NAME/*.log 확인"; exit 1; }
MPLBACKEND=Agg ~/venv/go2/bin/python3 $J/tools/route_editor.py "$NAME" --build || exit 1
echo "■ 지도 완성: $J/routes/$NAME/   다음 → ~/venv/go2/bin/python3 $J/tools/route_editor.py $NAME"
