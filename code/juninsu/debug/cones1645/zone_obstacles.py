#!/usr/bin/env python3
"""슬라럼 구역 장애물(편집기와 같은 규칙: 4 cm 칸에서 세로 0.25 m 넘게 선 면, 녹화 로봇 길 0.18 m 안 제외)을 관측 횟수 기준별로 — rec1650 좌표.
zone_<지도>_c<최소관측>.npy. 사람 흔적(관측 ~2회)과 콘·벽(8회 이상) 구분용."""
import json, math, os, sys
import numpy as np
from scipy.spatial import cKDTree
H = os.path.dirname(os.path.abspath(__file__)); J = os.path.dirname(os.path.dirname(H))
T = json.load(open(os.path.join(H, 'T_1650_1645.json')))['xy_yaw']
R = json.load(open(os.path.join(J, 'routes', 'rec1650', 'route.json')))['route']; SL = np.array([[e['x'], e['y']] for e in R[6:25]]); zl, zh = SL.min(0) - 0.4, SL.max(0) + 0.4
def apply(P):
    c, s = math.cos(T[2]), math.sin(T[2]); return np.c_[c * P[:, 0] - s * P[:, 1] + T[0], s * P[:, 0] + c * P[:, 1] + T[1]]
for name in ('rec1650', 'rec1645'):
    d = np.load(os.path.join(J, 'routes', name, 'map.npz')); od = d['odom']; tr = od[:, 2:4]
    for cmin in (2, 5, 8):
        P = (d['vox'][d['cnt'] >= cmin] + 0.5) * float(d['V'])
        if name == 'rec1645': P = np.c_[apply(P[:, :2]), P[:, 2]]; trr = apply(tr)
        else: trr = tr
        k = np.all((P[:, :2] > zl - 0.6) & (P[:, :2] < zh + 0.6), 1); P = P[k]
        kk = np.floor(P[:, :2] / 0.5).astype(np.int64); u, inv = np.unique(kk, axis=0, return_inverse=True); inv = inv.ravel()
        fl = np.full(len(u), np.inf); np.minimum.at(fl, inv, P[:, 2]); h = P[:, 2] - fl[inv]
        q = (h > -0.15) & (h < 1.5); Q, hq = P[q, :2], h[q]
        key = np.floor(Q / 0.04).astype(np.int64); u2, inv2 = np.unique(key, axis=0, return_inverse=True); inv2 = inv2.ravel()
        zmin = np.full(len(u2), 9.0); zmax = np.full(len(u2), -9.0); np.minimum.at(zmin, inv2, hq); np.maximum.at(zmax, inv2, hq)
        W = (u2[(zmax - zmin) >= 0.25] + 0.5) * 0.04
        W = W[cKDTree(trr).query(W)[0] >= 0.18]; W = W[np.all((W > zl) & (W < zh), 1)]
        np.save(os.path.join(H, f'zone_{name[3:]}_c{cmin}.npy'), W.astype(np.float32)); print(f'{name} 관측≥{cmin}: 슬라럼 구역 장애물 칸 {len(W)}')
