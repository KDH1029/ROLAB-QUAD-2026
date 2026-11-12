#!/usr/bin/env python3
"""얇은 벽(sim_walls_rec1650_thin, 기본에서 ~4 cm/면 깎음)을 한 칸(4 cm) 더 깎는다 — 단 그 방향 두께가 1칸뿐이면 남김(벽이 사라지지 않게). simcal"""
import numpy as np, sys
S = '/home/ilovehan9283/ws/junsu_v2/sim/'
W = np.load(S + 'sim_walls_rec1650_thin.npy'); V = 0.04
k = np.round(W[:, :2] / V).astype(int); occ = set(map(tuple, k.tolist()))
keep = []
for (i, j) in k.tolist():
    rm = False
    for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        if (i + di, j + dj) not in occ and (i - di, j - dj) in occ:   # 이 쪽은 비었고 반대쪽엔 벽이 더 있음 → 겉껍질
            rm = True; break
    keep.append(not rm)
keep = np.array(keep); out = W[keep]
np.save(sys.argv[1] if len(sys.argv) > 1 else 'sim_walls_rec1650_thin2.npy', out)
print(f'thin {len(W)} → thin2 {len(out)} 칸')
