#!/usr/bin/env python3
"""rec1114(리모컨, 부딪힘 없음·높이 표류) ∩ rec1118(실주행, 유령 깎음·콘 밀림 자국) → 둘 다 장애물인 칸만.
① 1114 벽을 1118 좌표로(1459 경유 초기값 → 점-점 ICP, 다듬기는 오른쪽·가운데처럼 표류 적은 곳 위주로 걸러진 짝만)
② 1118 map.npz 의 '바닥 위' 복셀 중 근처(TOL)에 1114 벽이 없는 칸의 것은 뺌 → maps/rec1118x/map.npz(편집기용)."""
import json, math, os, numpy as np, shutil
from scipy.spatial import cKDTree
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
plt.rcParams['font.family'] = 'NanumGothic'; plt.rcParams['axes.unicode_minus'] = False
J = os.path.expanduser('~/ws/junsu_v2'); H = os.path.dirname(os.path.abspath(__file__)); TOL = 0.08
A = np.load(J + '/maps/rec1114/editor_cache.npz')['walls'].astype(float)        # 1114 좌표
Bw = np.load(J + '/tools/route_editor_cache_rec1118.npz')['walls'].astype(float)  # 1118 좌표(깎은 뒤)
x0, y0, th0 = json.load(open(J + '/debug/route1114/T_1114_from_1459.json'))['T_1114_from_1459']
TX, TY, TH = 0.0920693, 0.0848863, 0.1040741
def inv1114(P):   # 1114 → 1459
    Q = P - [x0, y0]; c, s = math.cos(th0), math.sin(th0); return Q @ np.array([[c, -s], [s, c]])
def to1118(P):
    c, s = math.cos(TH), math.sin(TH); Q = P - [TX, TY]; return np.c_[c * Q[:, 0] + s * Q[:, 1], -s * Q[:, 0] + c * Q[:, 1]]
A0 = to1118(inv1114(A)); TB = cKDTree(Bw)
R = np.eye(2); t = np.zeros(2); P = A0.copy()
for it in range(40):
    d, j = TB.query(P); thr = max(0.06, 0.4 * 0.85 ** it); g = d < thr
    a, b = P[g], Bw[j[g]]; ca, cb = a.mean(0), b.mean(0); U, S_, Vt = np.linalg.svd((a - ca).T @ (b - cb)); Rk = Vt.T @ U.T
    if np.linalg.det(Rk) < 0: Vt[-1] *= -1; Rk = Vt.T @ U.T
    P = (P - ca) @ Rk.T + cb
d, _ = TB.query(P); print(f'1114 → 1118 맞춤: 6 cm 안 {np.mean(d < 0.06) * 100:.0f}% · 3 cm 안 {np.mean(d < 0.03) * 100:.0f}% (1118 벽 쪽에서 본 1114 일치 {np.mean(cKDTree(P).query(Bw)[0] < 0.06) * 100:.0f}%)')
np.save(H + '/walls1114_in1118.npy', P)
TA = cKDTree(P); keepB = TA.query(Bw)[0] < TOL
print(f'1118 벽 점 {len(Bw)} 중 1114 에도 있음(≤{TOL*100:.0f} cm) {keepB.sum()} ({keepB.mean()*100:.0f}%) → 교집합에 남김')
# map.npz 거르기
D0 = J + '/maps/rec1118'; D1 = J + '/maps/rec1118x'; os.makedirs(D1, exist_ok=True)
dd = dict(np.load(D0 + '/map.npz')); V = float(dd['V']); od = dd['odom']; Pv = (dd['vox'] + 0.5) * V
_, jj = cKDTree(od[:, 2:4]).query(Pv[:, :2]); hf = Pv[:, 2] - (od[jj, 4] - 0.30)
above = hf > 0.05; sup = TA.query(Pv[:, :2])[0] < TOL
drop = above & ~sup
dd['vox'] = dd['vox'][~drop]; dd['cnt'] = dd['cnt'][~drop]; np.savez_compressed(D1 + '/map.npz', **dd)
print(f'복셀 {len(Pv)} → {len(dd["vox"])} (1114 에 없는 바닥 위 복셀 {drop.sum()} 뺌) → {D1}/map.npz')
fig, axs = plt.subplots(1, 3, figsize=(21, 6.5))
for ax, (nm, W, col) in zip(axs, [('1114 리모컨(1118 좌표로)', P, 'tab:blue'), ('1118 실주행(유령 깎음)', Bw, 'k'), ('교집합(둘 다)', Bw[keepB], 'k')]):
    ax.scatter(W[:, 0], W[:, 1], s=2, c=col); ax.set_aspect('equal'); ax.set_xlim(-1.2, 7.6); ax.set_ylim(-1.2, 4.6); ax.grid(alpha=.3); ax.set_title(f'{nm} — {len(W)}')
axs[2].scatter(Bw[~keepB, 0], Bw[~keepB, 1], s=4, c='r', label='1118 에만 있음(뺌)'); axs[2].legend(loc='lower right')
plt.tight_layout(); plt.savefig(H + '/intersect.png', dpi=70); print('그림')
