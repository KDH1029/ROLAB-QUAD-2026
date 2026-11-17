#!/usr/bin/env bash
# 콘 무작위 배치 일괄 시뮬 — 판마다 따로 네트워크 네임스페이스(lo 만, 로봇 링크 안 보임) → 도메인 78 겹쳐도 안 섞임. P 판 동시.
# 사용: P=8 dr/dr_batch.sh <태그> L00_1 L02_1 ...   (조건은 환경변수 그대로 run_sim_v2.sh 로: SMODE CSAFE CINFL SAFE MAXV LOBS ...)
S="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"; TAG=$1; shift; P=${P:-8}
for nm in "$@"; do
  while [ $(jobs -rp | wc -l) -ge $P ]; do sleep 1; done
  unshare -rn bash -c "ip link set lo up; WALLS=$S/dr/${nm}_walls.npy CONES=$S/dr/${nm}_cones.npy SMODE=\${SMODE:-ego} CONE=\${CONE:-det} GUARD=\${GUARD:-off} TMAX=\${TMAX:-240} EXIT_X=\${EXIT_X:-5.2} $S/run_sim_v2.sh dr_${TAG}_${nm} 78 $S/dr/route_slalom_only.json > /dev/null 2>&1" &
  sleep 0.5
done
wait
~/venv/go2/bin/python3 $S/dr/dr_table.py $TAG "$@"
