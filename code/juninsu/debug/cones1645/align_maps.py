#!/usr/bin/env python3
"""rec1645 지도 → rec1650 좌표 변환 T(x, y, yaw) — 슬라럼 구역(콘)을 뺀 고정 벽끼리 3D ICP(icp_core.Matcher, 평면 3자유도).
출력 T_1650_1645.json. 검증: 슬라럼 밖 벽 점의 상대 지도까지 거리 중앙/p90, 출발점 차이."""
import json, math, os, sys
import numpy as np
H = os.path.dirname(os.path.abspath(__file__)); J = os.path.dirname(os.path.dirname(H))
sys.path.insert(0, os.path.join(J, 'debug', 'loc')); import icp_core as ic  # noqa: E402
R = json.load(open(os.path.join(J, 'routes', 'rec1650', 'route.json')))['route']; RP = np.array([[e['x'], e['y']] for e in R])
SL = RP[6:25]; ZLO, ZHI = SL.min(0) - 0.6, SL.max(0) + 0.6
def walls(name):
    d = np.load(os.path.join(J, 'routes', name, 'map.npz')); keep = d['cnt'] >= 5
    P = ((d['vox'][keep] + 0.5) * float(d['V']))
    P = P[(P[:, 0] > RP[:, 0].min() - 3) & (P[:, 0] < RP[:, 0].max() + 3) & (P[:, 1] > RP[:, 1].min() - 3) & (P[:, 1] < RP[:, 1].max() + 3)]
    key = np.floor(P[:, :2] / 0.5).astype(np.int64); u, inv = np.unique(key, axis=0, return_inverse=True); inv = inv.ravel()
    fl = np.full(len(u), np.inf); np.minimum.at(fl, inv, P[:, 2]); h = P[:, 2] - fl[inv]
    P = P[(h > 0.10) & (h < 1.8)]
    out = ~np.all((P[:, :2] > ZLO) & (P[:, :2] < ZHI), 1)
    return P[out]
A = walls('rec1650'); B = walls('rec1645')
M = ic.Matcher(A); scan = ic.voxel(B, 0.05)
best = None
for yaw0 in np.radians([-6, -3, 0, 3, 6]):
    T, rms, _ = M.align(scan, ic.T3_to4(np.array([0.0, 0.0, yaw0])))
    inl = M.inlier(scan, T, 0.05)
    if best is None or inl > best[1]:
        best = (T, inl, rms)
T, inl, rms = best; t3 = ic.T4_to3(T)
Q = (T[:3, :3] @ scan.T).T + T[:3, 3]; d = M.fine.query(Q)[0]
print(f'T_1650←1645 = ({t3[0] * 100:.1f} cm, {t3[1] * 100:.1f} cm, {math.degrees(t3[2]):.2f}°) · 5 cm 안 {inl * 100:.0f}% · 거리 중앙 {np.median(d) * 100:.1f} cm p90 {np.percentile(d, 90) * 100:.1f} cm (슬라럼 밖 벽)')
json.dump(dict(T4=T.tolist(), xy_yaw=t3.tolist(), inlier5=inl, med=float(np.median(d))), open(os.path.join(H, 'T_1650_1645.json'), 'w'), indent=1)
