#!/usr/bin/env python3
"""지금 경로(1459 좌표)를 새 지도 rec1114(11:14 리모컨, 새 콘 배치)에 겹쳐 점마다 장애물 거리를 잰다.
1459 지도(콘 없음) ↔ rec1114 지도 벽을 2D ICP 로 맞춤(거친 회전 탐색 → 점-점 ICP)."""
import json, sys, os, numpy as np, yaml, cv2
from scipy.spatial import cKDTree
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
plt.rcParams['font.family'] = 'NanumGothic'; plt.rcParams['axes.unicode_minus'] = False
J = os.path.expanduser('~/ws/junsu_v2'); H = os.path.dirname(os.path.abspath(__file__))
ROUTE = sys.argv[1] if len(sys.argv) > 1 else J + '/maps/route_map_1106.json'
def occ(yml):
    y = yaml.safe_load(open(yml)); im = cv2.imread(os.path.join(os.path.dirname(yml), y['image']), cv2.IMREAD_GRAYSCALE)
    r = y['resolution']; ox, oy = y['origin'][:2]; v, u = np.nonzero(im < 100)
    return np.c_[ox + (u + 0.5) * r, oy + (im.shape[0] - v - 0.5) * r]
A = occ(J + '/maps/align/arena1459_nocone_slalomfree.yaml'); B = occ(J + '/maps/rec1114/map.yaml')
# 같은 코스 범위만(전시장 잡음 줄이기)
A = A[(A[:, 0] > -1.5) & (A[:, 0] < 8) & (A[:, 1] > -1.5) & (A[:, 1] < 5.5)]; B = B[(B[:, 0] > -1.5) & (B[:, 0] < 8.5) & (B[:, 1] > -1.5) & (B[:, 1] < 5.5)]
TB = cKDTree(B)
def tf(P, x, y, th): c, s = np.cos(th), np.sin(th); return P @ np.array([[c, s], [-s, c]]) + [x, y]
best = None
for th in np.radians(np.arange(-20, 20.1, 1.0)):
    for dx in np.arange(-0.6, 0.61, 0.1):
        for dy in np.arange(-0.6, 0.61, 0.1):
            d, _ = TB.query(tf(A[::4], dx, dy, th)); sc = np.mean(d < 0.06)
            if best is None or sc > best[0]: best = (sc, dx, dy, th)
x, y, th = best[1:]
for it in range(30):
    P = tf(A, x, y, th); d, j = TB.query(P); g = d < max(0.08, 0.3 * 0.85 ** it); a, b = P[g], B[j[g]]
    ca, cb = a.mean(0), b.mean(0); U, S_, Vt = np.linalg.svd((a - ca).T @ (b - cb)); R = Vt.T @ U.T
    if np.linalg.det(R) < 0: Vt[-1] *= -1; R = Vt.T @ U.T
    dth = np.arctan2(R[1, 0], R[0, 0]); th += dth; t = cb - ca @ R.T; x, y = np.array([x, y]) @ R.T + t
inl = np.mean(TB.query(tf(A, x, y, th))[0] < 0.06)
print(f'1459 → rec1114: x {x:+.3f} y {y:+.3f} 회전 {np.degrees(th):+.2f}° · 벽 일치(6 cm 안) {inl * 100:.0f}%')
json.dump({'T_1114_from_1459': [float(x), float(y), float(th)], 'inlier': float(inl)}, open(H + '/T_1114_from_1459.json', 'w'))
R = json.load(open(ROUTE)); pts = [(p['label'], p['x'], p['y']) for p in R['route']]
P = tf(np.array([[p[1], p[2]] for p in pts]), x, y, th); d, _ = TB.query(P)
print(' 점          장애물까지(m)   ★=0.30 m 안')
bad = []
for (lab, _, _), q, dd in zip(pts, P, d):
    flag = '★' if dd < 0.30 else ''
    if flag: bad.append(lab)
    print(f'  {lab:10s} ({q[0]:5.2f},{q[1]:5.2f})  {dd:5.2f} {flag}')
SP = R.get('slalom_path')
fig, ax = plt.subplots(figsize=(12, 9)); ax.scatter(B[:, 0], B[:, 1], s=1, c='k', label='rec1114 장애물(벽·새 콘)')
ax.plot(P[:, 0], P[:, 1], 'b.-', lw=1, label='지금 경로(route_map_1106)')
for (lab, _, _), q, dd in zip(pts, P, d):
    ax.annotate(lab, q, fontsize=8, color='red' if dd < 0.30 else 'b')
    if dd < 0.30: ax.add_patch(plt.Circle(q, 0.30, fill=False, color='red'))
ax.set_aspect('equal'); ax.set_xlim(-1.5, 8.5); ax.set_ylim(-1.5, 5.5); ax.grid(alpha=.3); ax.legend(loc='upper left')
ax.set_title(f'지금 경로 on rec1114 — 빨강 = 장애물 0.30 m 안({len(bad)}개)'); plt.tight_layout(); plt.savefig(H + '/route_on_1114.png', dpi=85)
print('0.30 m 안:', bad)
