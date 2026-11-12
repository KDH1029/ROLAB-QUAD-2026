#!/usr/bin/env python3
"""rec1645 → rec1650 2D 변환: 편집기 벽(세로 면, real_walls)끼리 2D ICP(트림 70 %), 슬라럼 구역(콘) 제외. 출발값 여러 개 → 5 cm 안 비율 최대.
출력 T_1650_1645.json (x, y, yaw) + 벽 거리 중앙/p90/5cm 안."""
import importlib, json, math, os, sys
import numpy as np
from scipy.spatial import cKDTree
H = os.path.dirname(os.path.abspath(__file__)); J = os.path.dirname(os.path.dirname(H)); sys.path.insert(0, os.path.join(J, 'tools'))
R = json.load(open(os.path.join(J, 'routes', 'rec1650', 'route.json')))['route']; RP = np.array([[e['x'], e['y']] for e in R])
SL = RP[6:25]; ZLO, ZHI = SL.min(0) - 0.6, SL.max(0) + 0.6; LO, HI = RP.min(0) - 2.0, RP.max(0) + 2.0
def walls(name):
    sys.argv = ['x', name]; import route_editor as ed; ed = importlib.reload(ed); W = ed.real_walls(ed.load_cache()); return W
def crop(W, slalom_out=True):
    k = np.all((W > LO) & (W < HI), 1)
    if slalom_out: k &= ~np.all((W > ZLO) & (W < ZHI), 1)
    return W[k]
Wa_all = walls('rec1650'); Wb_all = walls('rec1645')
def apply(T, P):
    c, s = math.cos(T[2]), math.sin(T[2]); return np.c_[c * P[:, 0] - s * P[:, 1] + T[0], s * P[:, 0] + c * P[:, 1] + T[1]]
def icp(A, B, T, iters=60, trim=0.7):
    TA = cKDTree(A)
    for _ in range(iters):
        Q = apply(T, B); d, j = TA.query(Q); th = np.quantile(d, trim); s = d <= th
        X, Y = Q[s], A[j[s]]; cx, cy = X.mean(0), Y.mean(0); X0, Y0 = X - cx, Y - cy
        da = math.atan2((X0[:, 0] * Y0[:, 1] - X0[:, 1] * Y0[:, 0]).sum(), (X0 * Y0).sum())
        c, s_ = math.cos(da), math.sin(da); Rm = np.array([[c, -s_], [s_, c]]); t = cy - Rm @ cx
        # compose: new = Rm*(R*p + t0) + t
        T = np.array([*(Rm @ T[:2] + t), T[2] + da])
    d = TA.query(apply(T, B))[0]; return T, d
A = crop(Wa_all); B = crop(Wb_all)
best = None
for x0 in (-0.05, 0.0, 0.05):
    for y0 in (-0.05, 0.0, 0.05):
        for a0 in (-4.0, -3.0, -2.0, 0.0):
            T, d = icp(A, B, np.array([x0, y0, math.radians(a0)]))
            sc = np.mean(d < 0.05)
            if best is None or sc > best[1]: best = (T, sc, d)
T, sc, d = best
print(f'T_1650←1645 = ({T[0] * 100:.1f} cm, {T[1] * 100:.1f} cm, {math.degrees(T[2]):.2f}°) · 슬라럼 밖 벽: 중앙 {np.median(d) * 100:.1f} cm · p90 {np.percentile(d, 90) * 100:.1f} cm · 5 cm 안 {sc * 100:.0f}%')
# 슬라럼 안 벽(콘 제외 못 함) 참고
din = cKDTree(crop(Wa_all, False)).query(apply(T, Wb_all[np.all((Wb_all > ZLO) & (Wb_all < ZHI), 1)]))[0]
print(f'  (참고) 슬라럼 구역 안 점: 중앙 {np.median(din) * 100:.1f} cm · 5 cm 안 {np.mean(din < 0.05) * 100:.0f}%')
json.dump(dict(xy_yaw=T.tolist(), med_cm=float(np.median(d) * 100), in5=float(sc)), open(os.path.join(H, 'T_1650_1645.json'), 'w'), indent=1)
