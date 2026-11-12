#!/usr/bin/env python3
"""핀을 8 cm 안에서만 움직여 몸 둘레 여유(최소·5 cm 안 길이)를 넓히는 제안 → maps/route_v2_1118_pinfix.json(원본은 그대로)."""
import json, os, numpy as np, copy
from scipy.spatial import cKDTree
from scipy.interpolate import UnivariateSpline
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
plt.rcParams['font.family'] = 'NanumGothic'; plt.rcParams['axes.unicode_minus'] = False
J = os.path.expanduser('~/ws/junsu_v2'); H = os.path.dirname(os.path.abspath(__file__)); MAXD = 0.08
D = json.load(open(J + '/maps/route_v2_1118.json')); R = D['route']; lab = [p['label'] for p in R]
i0 = lab.index('rabain'); i1 = lab.index('rabaout'); idx = [i0] + [i for i in range(i0 + 1, i1) if lab[i].startswith('pin')]
P0 = np.array([[R[i]['x'], R[i]['y']] for i in idx])
W = np.load(J + '/tools/route_editor_cache_rec1118x.npz')['walls'].astype(float); T = cKDTree(W)
bx = np.r_[np.linspace(-0.35, 0.35, 15), np.full(7, 0.35), np.linspace(0.35, -0.35, 15), np.full(7, -0.35)]
by = np.r_[np.full(15, 0.165), np.linspace(0.165, -0.165, 7), np.full(15, -0.165), np.linspace(-0.165, 0.165, 7)]
def curve(P):
    d = np.r_[0, np.cumsum(np.hypot(*np.diff(P, axis=0).T))]; u = np.linspace(0, d[-1], int(d[-1] / 0.03) + 1); sm = len(P) * 0.03 ** 2
    return np.c_[UnivariateSpline(d, P[:, 0], k=3, s=sm)(u), UnivariateSpline(d, P[:, 1], k=3, s=sm)(u)]
def clear(P):
    C = curve(P); t = np.gradient(C, axis=0); t /= np.linalg.norm(t, axis=1)[:, None]; n = np.c_[-t[:, 1], t[:, 0]]
    Q = C[:, None, :] + bx[None, :, None] * t[:, None, :] + by[None, :, None] * n[:, None, :]
    return T.query(Q.reshape(-1, 2))[0].reshape(len(C), -1).min(1), C
def cost(P):
    cl, C = clear(P); return -np.min(cl) + 0.5 * np.mean(cl < 0.05) - 0.3 * np.mean(np.minimum(cl, 0.12))
P = P0.copy(); best = cost(P)
for it in range(4):
    for k in range(1, len(P)):              # rabain(0) 은 고정
        g = np.arange(-MAXD, MAXD + 1e-9, 0.02)
        for dx in g:
            for dy in g:
                if dx * dx + dy * dy > MAXD * MAXD: continue
                Q = P.copy(); Q[k] = P0[k] + [dx, dy]; c = cost(Q)
                if c < best - 1e-6: best, P = c, Q
cl0, C0 = clear(P0); cl1, C1 = clear(P)
st = lambda cl: f'최소 {cl.min()*100:.1f} cm · 5 cm 안 {np.sum(cl < 0.05) * 0.03:.2f} m · 10 cm 안 {np.sum(cl < 0.10) * 0.03:.2f} m'
print('사용자 핀 :', st(cl0)); print('8 cm 손봄:', st(cl1))
for k in range(1, len(P)): print(f'  {lab[idx[k]]:5s} ({P0[k,0]:.2f},{P0[k,1]:.2f}) → ({P[k,0]:.2f},{P[k,1]:.2f})  {np.hypot(*(P[k]-P0[k]))*100:.0f} cm')
out = copy.deepcopy(D)
for k, i in enumerate(idx):
    out['route'][i]['x'], out['route'][i]['y'] = round(float(P[k, 0]), 3), round(float(P[k, 1]), 3)
out['saved'] = D.get('saved', '') + ' · 슬라럼 핀 8 cm 안 손봄 제안'
json.dump(out, open(J + '/maps/route_v2_1118_pinfix.json', 'w'), ensure_ascii=False, indent=1); print('저장 maps/route_v2_1118_pinfix.json')
fig, ax = plt.subplots(figsize=(7.5, 11)); ax.scatter(W[:, 0], W[:, 1], s=4, c='k', label='벽·콘(교집합 지도)')
ax.plot(C0[:, 0], C0[:, 1], '-', c='0.5', lw=2, label='사용자 핀 곡선'); sc = ax.scatter(C1[:, 0], C1[:, 1], c=np.clip(cl1, 0, 0.2), cmap='RdYlGn', s=12, vmin=0, vmax=0.2, label='손본 곡선(색=몸 여유)')
plt.colorbar(sc, ax=ax, fraction=0.04, label='몸 둘레 여유 (m)')
for k in range(len(P)):
    ax.annotate('', P[k], P0[k], arrowprops=dict(arrowstyle='->', color='m')); ax.plot(*P0[k], 'bo', ms=5)
    ax.annotate(lab[idx[k]], P[k], fontsize=9, color='b', xytext=(5, 3), textcoords='offset points')
ax.set_xlim(5.9, 7.5); ax.set_ylim(-0.4, 3.9); ax.set_aspect('equal'); ax.grid(alpha=.3); ax.legend(loc='lower left', fontsize=8)
ax.set_title('슬라럼 핀: 사용자(회색) vs 8 cm 안 손봄(색)'); plt.tight_layout(); plt.savefig(H + '/pins_nudge.png', dpi=90)
