#!/usr/bin/env python3
"""장애물 집합 검증: 사람이 리모컨으로 몬 두 주행(16:45 클래식·16:50, 콘에 안 닿음)의 몸(0.70×0.31) 이 장애물 칸과 겹치는 시간.
진짜에 가까운 장애물일수록 0 에 가까워야 한다. 슬라럼 #7~#25 구역(±0.4 m) 안만."""
import json, math, os, sys
import numpy as np
from scipy.spatial import cKDTree
H = os.path.dirname(os.path.abspath(__file__)); J = os.path.dirname(os.path.dirname(H))
T = json.load(open(os.path.join(H, 'T_1650_1645.json')))['xy_yaw']
R = json.load(open(os.path.join(J, 'routes', 'rec1650', 'route.json')))['route']; SL = np.array([[e['x'], e['y']] for e in R[6:25]]); zl, zh = SL.min(0) - 0.4, SL.max(0) + 0.4
def drive(name):
    d = np.load(os.path.join(J, 'routes', name, 'map.npz')); od = d['odom']; od = od[np.argsort(od[:, 1])]; q = od[:, 5:9]
    yaw = np.arctan2(2 * (q[:, 3] * q[:, 2] + q[:, 0] * q[:, 1]), 1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2)); xy = od[:, 2:4]; t = od[:, 1]
    if name == 'rec1645':
        c, s = math.cos(T[2]), math.sin(T[2]); xy = np.c_[c * xy[:, 0] - s * xy[:, 1] + T[0], s * xy[:, 0] + c * xy[:, 1] + T[1]]; yaw = yaw + T[2]
    k = np.all((xy > zl) & (xy < zh), 1); return t[k], xy[k], yaw[k]
def hits(OB, t, xy, yaw, L2=0.35, W2=0.155):
    TO = cKDTree(OB); dt = np.r_[np.diff(t), 0.0]; dt = np.clip(dt, 0, 0.2); hit = 0.0; gaps = []
    for (x, y), a, d_ in zip(xy, yaw, dt):
        idx = TO.query_ball_point([x, y], 0.45)
        if not idx: gaps.append(9); continue
        q = OB[idx] - [x, y]; lx = q[:, 0] * math.cos(a) + q[:, 1] * math.sin(a); ly = -q[:, 0] * math.sin(a) + q[:, 1] * math.cos(a)
        ins = (np.abs(lx) < L2) & (np.abs(ly) < W2); hit += d_ * bool(ins.any())
        gaps.append(np.sqrt((np.maximum(np.abs(lx) - L2, 0) ** 2 + np.maximum(np.abs(ly) - W2, 0) ** 2).min()))
    return hit, float(np.sum(dt)), np.array(gaps)
D = {n: drive(n) for n in ('rec1645', 'rec1650')}
print('장애물 집합        | 16:45 주행 몸 겹침 / 구역 시간 | 16:50 주행 몸 겹침 / 구역 시간')
for f in sorted(x for x in os.listdir(H) if x.startswith('zone_') and x.endswith('.npy')):
    OB = np.load(os.path.join(H, f)); r = [hits(OB, *D[n]) for n in ('rec1645', 'rec1650')]
    print(f'{f[5:-4]:18s} | {r[0][0]:5.1f}s / {r[0][1]:4.0f}s ({r[0][0] / r[0][1] * 100:4.1f}%) | {r[1][0]:5.1f}s / {r[1][1]:4.0f}s ({r[1][0] / r[1][1] * 100:4.1f}%)')
