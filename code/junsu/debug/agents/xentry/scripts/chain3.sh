#!/usr/bin/env bash
# chain2 뒤: 외톨이 칸 없는 벽에서 예전 결론 다시 보기 — (c) GUARD=on ×2  (d) CONE=walls(콘 검출 없음) ×1
X=$HOME/ws/junsu_v2/debug/agents/xentry; S=$HOME/ws/junsu_v2/sim
until grep -q CHAIN_DONE $X/chain2.out 2>/dev/null; do sleep 5; done
TMAX=120 EXIT_X=-0.14 CONE=det GUARD=on WALLS=$X/sim_walls_rec1650_noiso.npy $X/scripts/run_batch.sh xentry_noiso_guard $S/route_v2_1650_hybrid_sim.json 2
TMAX=100 EXIT_X=-0.14 CONE=walls GUARD=off WALLS=$X/sim_walls_rec1650_noiso.npy $X/scripts/run_batch.sh xentry_noiso_nodet $S/route_v2_1650_hybrid_sim.json 1
echo CHAIN3_DONE
