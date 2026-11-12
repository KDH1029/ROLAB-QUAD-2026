#!/usr/bin/env python3
"""GroundGrid 가 494.6초 위치에서 +2.5 m 를 내는 이유: 앞부분 지도 vs 전체 지도의 칸 값과 그 칸의 점들."""
import sys, numpy as np
sys.path.insert(0, '/home/ilovehan9283/ws/v_gosu/backup')
from backup_core import GroundGrid, load_map, voxel
d = np.load('/home/ilovehan9283/ws/v_gosu/eval/out/gosu1030v2_map.npz')
FULL = voxel((d['vox'][d['cnt'] >= 2].astype(np.float64) + 0.5) * float(d['V']), 0.05)
HALF = voxel(load_map('map_first270s_fastlio.npz'), 0.05)
odom = d['odom']; o = np.argsort(odom[:, 1]); G = odom[o]
# 494.6 초 근처 FAST-LIO 위치: 녹화 시각 정렬 없이, 궤적 시간축 상대값으로 근사
t = G[:, 1] - G[0, 1]
for ts in (493.0, 494.0, 494.6, 495.0, 500.0, 505.5, 506.0):
    i = np.searchsorted(t, ts); x, y, z = G[i, 2:5]
    for nm, mp in (('half', HALF), ('full', FULL)):
        gg = GroundGrid(mp) if nm not in globals() else globals()[nm]
        globals()[nm] = gg
        k = np.floor(np.array([x, y]) / 0.1).astype(int)
        m = np.all(np.floor(mp[:, :2] / 0.1).astype(int) == k, axis=1)
        dist, ii = gg.tree.query([x / 0.1, y / 0.1])
        print(f'{ts:6.1f}s ({x:+.2f},{y:+.2f}) z{z:+.2f} {nm}: ground {gg.at(x, y):+.2f}  칸점 {m.sum():3d}  가장가까운칸 거리 {dist:.1f}칸  칸z {np.round(np.sort(mp[m,2])[:6],2) if m.any() else "-"}')
