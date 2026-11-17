#!/usr/bin/env bash
# terrain_veto_node.cpp 의 낭떠러지 함수 구간을 떼어 단독 컴파일 → 합성 격자(synth_cliff.py 가 저장)에서 파이썬 원형과 칸 단위 대조
set -e; D=$(cd "$(dirname "$0")" && pwd)
sed -n '/==CLIFF_BEGIN==/,/==CLIFF_END==/p' ~/ws/night_20261104/ws/src/terrain_veto/src/terrain_veto_node.cpp > "$D/cliff_block.hpp"
g++ -O2 -std=c++17 -Wall -Wextra -o "$D/test_cliff_cpp" "$D/test_cliff_main.cpp" -I"$D"
rc=0; for g in "$D"/synth_grid_*.txt "$D"/real_grid_*.txt; do [ -e "$g" ] || continue; "$D/test_cliff_cpp" "$g" 160 0.05 0.22 0.30 0.05 1 || rc=1; done
for g in "$D"/real_gridb_*.txt; do [ -e "$g" ] || continue; "$D/test_cliff_cpp" "$g" 160 0.05 0.22 0.30 0.05 1 $(cat "${g%.txt}.args") || rc=1; done   # band 0.20 + 발밑 평면
exit $rc
