#!/usr/bin/env python3
"""사용자가 새로 찍은 슬라럼 핀(maps/route_v2_1118.json, 1118 좌표) 점검: 러너와 같은 회귀 곡선 → 몸(0.70×0.33, 곡선 방향) 이 지나갈 때
장애물(교집합 지도 rec1118x 의 벽·콘)까지 여유. 11:18 실제 궤적·인식 콘도 같이 그림."""
import json, math, os, numpy as np
from scipy.spatial import cKDTree
from scipy.interpolate import UnivariateSpline
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
plt.rcParams['font.family'] = 'NanumGothic'; plt.rcParams['axes.unicode_minus'] = False
J = os.path.expanduser('~/ws/junsu_v2'); H = os.path.dirname(os.path.abspath(__file__))
R = json.load(open(J + '/maps/route_v2_1118.json'))['route']; lab = [p['label'] for p in R]
i0 = lab.index('rabain'); i1 = lab.index('rabaout'); pins = [i for i in range(i0 + 1, i1) if lab[i].startswith('pin')]
pts = np.array([[R[i]['x'], R[i]['y']] for i in [i0] + pins])
d = np.r_[0, np.cumsum(np.hypot(*np.diff(pts, axis=0).T))]; L = d[-1]; u = np.linspace(0, L, int(L / 0.02) + 1); sm = len(pts) * 0.03 ** 2
C = np.c_[UnivariateSpline(d, pts[:, 0], k=3, s=sm)(u), UnivariateSpline(d, pts[:, 1], k=3, s=sm)(u)]
e = np.load(J + '/tools/route_editor_cache_rec1118x.npz'); W = e['walls'].astype(float); T = cKDTree(W)
cones = np.load(J + '/debug/run1118/live_cones.npy').reshape(-1, 2)
path = np.load(J + '/maps/rec1118/map.npz')['odom'][:, 2:4]
tan = np.gradient(C, axis=0); tan /= np.linalg.norm(tan, axis=1)[:, None]; nor = np.c_[-tan[:, 1], tan[:, 0]]
# 몸 사각형 둘레 점(앞뒤 ±0.35, 좌우 ±0.165)
bx = np.r_[np.linspace(-0.35, 0.35, 15), np.full(7, 0.35), np.linspace(0.35, -0.35, 15), np.full(7, -0.35)]
by = np.r_[np.full(15, 0.165), np.linspace(0.165, -0.165, 7), np.full(15, -0.165), np.linspace(-0.165, 0.165, 7)]
clr = np.array([T.query(c + bx[:, None] * t + by[:, None] * n)[0].min() for c, t, n in zip(C, tan, nor)])
cen = T.query(C)[0]
print(f'핀 {len(pins)}개 · 곡선 길이 {L:.2f} m · 곡선 중심↔장애물 최소 {cen.min():.2f} m · 몸 둘레↔장애물 최소 {clr.min():.2f} m')
for th in (0.0, 0.05, 0.10):
    print(f'  몸 둘레가 장애물 {th*100:.0f} cm 안인 곡선 길이: {np.sum(clr <= th) * 0.02:.2f} m')
ip = [int(np.argmin(np.hypot(*(C - pts[k]).T))) for k in range(len(pts))]
for k, i in enumerate(ip):
    nm = 'rabain' if k == 0 else lab[pins[k - 1]]; print(f'  {nm:7s} 중심↔장애물 {cen[i]:.2f} m · 몸 둘레 여유 {clr[i]:.2f} m')
bad = clr < 0.05
fig, ax = plt.subplots(figsize=(8, 11)); ax.scatter(W[:, 0], W[:, 1], s=4, c='k', label='벽·콘(교집합 지도)')
ax.scatter(cones[:, 0], cones[:, 1], s=40, marker='^', c='orange', edgecolors='k', label='11:18 인식 콘')
ax.plot(path[:, 0], path[:, 1], '-', c='limegreen', lw=1.2, label='11:18 실제 궤적(ego)')
sc = ax.scatter(C[:, 0], C[:, 1], c=np.clip(clr, 0, 0.3), cmap='RdYlGn', s=10, vmin=0, vmax=0.3); plt.colorbar(sc, ax=ax, fraction=0.04, label='몸 둘레 ↔ 장애물 여유 (m)')
for i in np.where(bad)[0][::6]:
    q = C[i] + bx[:, None] * tan[i] + by[:, None] * nor[i]; ax.plot(np.r_[q[:, 0], q[0, 0]], np.r_[q[:, 1], q[0, 1]], 'r-', lw=0.6)
ax.plot(pts[:, 0], pts[:, 1], 'bo', ms=6)
for k, p in enumerate(pts): ax.annotate('rabain' if k == 0 else lab[pins[k - 1]], p, fontsize=9, color='b', xytext=(5, 3), textcoords='offset points')
ra = R[i1]; ax.plot(ra['x'], ra['y'], 'ms', ms=8); ax.annotate('rabaout', (ra['x'], ra['y']), color='m')
ax.set_xlim(5.4, 7.6); ax.set_ylim(-0.7, 4.1); ax.set_aspect('equal'); ax.grid(alpha=.3); ax.legend(loc='lower left', fontsize=8)
ax.set_title('새 슬라럼 핀 — 러너 회귀 곡선(색 = 몸 둘레 여유), 빨간 사각형 = 5 cm 안'); plt.tight_layout(); plt.savefig(H + '/pins_check.png', dpi=90)
