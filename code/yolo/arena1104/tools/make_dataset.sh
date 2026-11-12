#!/usr/bin/env bash
# 11-04 경기장 녹화 → 신호등·라바콘 사진 1500장(라벨링 전 원본)
# 결과: ~/ws/datasets/arena1104/images/*.jpg · frames.csv(어느 녹화·몇 번째 프레임) · preview.jpg
D=~/ws/datasets/arena1104; T=$D/tools; C=$D/_candidates; N=${1:-1500}
set +u; source /opt/ros/jazzy/setup.bash
echo "=== 1/2 후보 뽑기 $(date +%T)  (D435i 3장마다 · 앞머리 720p 2장마다 · 같은 장면 건너뜀)"
python3 $T/extract_frames.py $C 3 2 0 ~/bags/gosu_20261104_113612 ~/bags/gosu_20261104_141152 ~/bags/gosu_20261104_142146 \
  ~/bags/gosu_20261104_143520 ~/bags/gosu_20261104_145915_arena3 ~/bags/gosu_20261104_150952_arena3 2>&1 | grep -vE '^\[|Warn' || exit 1
echo "후보 $(ls $C/*.jpg | wc -l)장"
echo "=== 2/2 신호등·콘 장면 고르기 $(date +%T)  (장당 약 60 ms)"
~/venv/go2/bin/python3 $T/select_frames.py $C $D $N 2>&1 | grep -v Warn
echo "■ 끝 $(date +%T) → $D/images ($(ls $D/images | wc -l)장)  · 후보 원본은 $C (지워도 됨)"
