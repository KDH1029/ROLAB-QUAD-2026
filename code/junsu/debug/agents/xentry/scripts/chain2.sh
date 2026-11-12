#!/usr/bin/env bash
# 배치(noiso 3판)가 끝난 뒤 차례로: (a) 외톨이 칸 있음 + 되짚기 끔 ×2  (b) STACK=main + 외톨이 칸 없음 ×1
X=$HOME/ws/junsu_v2/debug/agents/xentry; S=$HOME/ws/junsu_v2/sim
TMAX=200 EXIT_X=-0.14 CONE=det GUARD=off RUNNER_OFF_ROUTE_M=99 WALLS=$S/sim_walls_rec1650.npy $X/scripts/run_batch.sh xentry_ghost_noretrace $S/route_v2_1650_hybrid_sim.json 2
STACK=main TMAX=110 CONE=det WALLS=$X/sim_walls_rec1650_noiso.npy $X/scripts/run_batch.sh xentry_main_noiso $S/route_main1654_in1650.json 1
echo CHAIN_DONE
