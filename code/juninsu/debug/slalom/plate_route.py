#!/usr/bin/env python3
"""받침대 피하는 슬라럼 점 제안 — rec1650 경로 #7~#25 를 각각 반경 0.20 m 안(1 cm 간격)에서 옮겨, 그 점에 몸이 경로 접선 방향으로 섰을 때
네 발(±0.19, ±0.14)과 콘 중심 10개의 최소 거리를 최대로. 벽(콘이 아닌 지도 점)과 몸 중심은 원래보다 가까워지지 않게(또는 0.30 m 이상).
두 번 반복(접선이 바뀌므로). 출력: routes/rec1650_plate/ (rec1650 복사 + route.json 교체, 원래 경로는 그대로)."""
import json, math, os, shutil, sys
import numpy as np
from scipy.spatial import cKDTree
H = os.path.dirname(os.path.abspath(__file__)); J = os.path.dirname(os.path.dirname(H))
src = os.path.join(J, 'routes', 'rec1650'); dst = os.path.join(J, 'routes', 'rec1650_plate')
RJ = json.load(open(os.path.join(src, 'route.json'))); R = RJ['route']
C = np.load(os.path.join(H, 'cone_centres_rec1650.npy')); TC = cKDTree(C)
sys.argv = ['x', 'rec1650']; sys.path.insert(0, os.path.join(J, 'tools')); import route_editor as ed  # noqa: E402
XW = ed.real_walls(ed.load_cache()); WW = XW[TC.query(XW)[0] > 0.30]; TW = cKDTree(WW)
FEET = np.array([(0.19, 0.14), (0.19, -0.14), (-0.19, 0.14), (-0.19, -0.14)])
def feet(p, a):
    c, s = math.cos(a), math.sin(a); return p + FEET @ np.array([[c, s], [-s, c]])
P = np.array([[e['x'], e['y']] for e in R]); P0 = P.copy()
tang = lambda i: math.atan2(*(P[min(i + 1, len(P) - 1)] - P[max(i - 1, 0)])[::-1])
for it in range(2):
    for i in range(6, 25):
        a = tang(i); w0 = TW.query(P0[i])[0]; wmin = min(w0, 0.30)
        best = (TC.query(feet(P[i], a))[0].min(), P[i])
        for dx in np.arange(-0.20, 0.201, 0.01):
            for dy in np.arange(-0.20, 0.201, 0.01):
                if dx * dx + dy * dy > 0.04: continue
                q = P0[i] + (dx, dy)
                if TW.query(q)[0] < wmin: continue
                m = TC.query(feet(q, a))[0].min()
                if m > best[0] + 1e-4 or (abs(m - best[0]) < 1e-4 and np.hypot(*(q - P0[i])) < np.hypot(*(best[1] - P0[i]))):
                    best = (m, q)
        P[i] = best[1]
print(f'{"점":>3s} {"원래(x,y)":>14s} {"제안(x,y)":>14s} {"이동":>6s} {"발-콘 원래":>8s} {"발-콘 제안":>8s} {"벽 원래→제안":>12s}')
for i in range(6, 25):
    a0 = math.atan2(*(P0[min(i + 1, len(P0) - 1)] - P0[max(i - 1, 0)])[::-1]); a1 = tang(i)
    f0 = TC.query(feet(P0[i], a0))[0].min(); f1 = TC.query(feet(P[i], a1))[0].min()
    print(f'#{i + 1:2d} ({P0[i][0]:5.2f},{P0[i][1]:5.2f}) ({P[i][0]:5.2f},{P[i][1]:5.2f}) {np.hypot(*(P[i] - P0[i])) * 100:5.1f}cm {f0 * 100:7.1f}cm {f1 * 100:7.1f}cm {TW.query(P0[i])[0] * 100:5.1f}→{TW.query(P[i])[0] * 100:5.1f}cm')
if os.path.exists(dst): shutil.rmtree(dst)
shutil.copytree(src, dst, ignore=shutil.ignore_patterns('*.bak*', 'route.png'))
for i in range(6, 25):
    R[i]['x'], R[i]['y'] = round(float(P[i][0]), 3), round(float(P[i][1]), 3)
json.dump(RJ, open(os.path.join(dst, 'route.json'), 'w'), ensure_ascii=False, indent=1)
print('→', dst)
