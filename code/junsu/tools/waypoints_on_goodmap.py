#!/usr/bin/env python3
"""저장한 좌표(waypoint_logger json)를 FAST-LIO 지도(map_collect npz) 위에 올린다 — 10-30 그림과 같은 높이 지도 바탕.
좌표 세션과 지도 녹화는 FAST-LIO 를 따로 켰으므로, ① 그대로(같은 출발점·방향 가정) ② 지도 녹화의 로봇 경로에 맞춘 2D 강체변환 두 가지를 그린다.
좌표마다: 경로까지 거리, 가장 가까운 벽(바닥 위 0.3~1.0 m 칸)까지 거리(차선 70 cm 면 가운데가 약 0.35 m).
사용: waypoints_on_goodmap.py <지도 npz> <좌표 json> <출력 png> [제목]"""
import sys, os, json, math, numpy as np
from scipy.spatial import cKDTree
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
plt.rcParams['font.family'] = ['NanumGothic', 'DejaVu Sans']; plt.rcParams['axes.unicode_minus'] = False
NPZ, WP, PNG = sys.argv[1:4]; TITLE = sys.argv[4] if len(sys.argv) > 4 else os.path.basename(NPZ)
HMAX, CELL = 1.8, 0.05

# 바탕: render_like_1030 과 같은 방법
src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'render_like_1030.py')).read()
exec(src[src.index('def revox'):src.index('def wall_band_thickness')])
d = np.load(os.path.expanduser(NPZ)); P = revox(d); od = d['odom']; od = od[np.argsort(od[:, 1])]; tr = od[:, 2:5]
REG = (tr[:, 0].min() - 1, tr[:, 0].max() + 1, tr[:, 1].min() - 1, tr[:, 1].max() + 1)
coef, tilt = fit_floor(P, REG); h = P[:, 2] - (coef[0] * P[:, 0] + coef[1] * P[:, 1] + coef[2])
keep = (h > -0.3) & (h < HMAX); Px, hx = P[keep], h[keep]
ext = (math.floor(tr[:, 0].min() - 2.0), math.ceil(tr[:, 0].max() + 2.0), math.floor(tr[:, 1].min() - 2.0), math.ceil(tr[:, 1].max() + 2.0))
img = grid_max(Px[:, :2], hx, CELL, ext)
# 벽 칸(경로 1.5 m 안, 바닥 위 0.3~1.0 m)
wm = (hx > 0.3) & (hx < 1.0); Wc = Px[wm, :2]
path = tr[:, :2]; seg = [path[0]]
for a, b in zip(path[:-1], path[1:]):
    n = max(1, int(np.hypot(*(b - a)) / 0.05)); seg += [a + (b - a) * k / n for k in range(1, n + 1)]
path = np.array(seg); Tp = cKDTree(path)
Wc = Wc[Tp.query(Wc)[0] < 1.5]; Tw = cKDTree(Wc)

W = json.load(open(os.path.expanduser(WP))); lab = [w['label'] for w in W]; wp = np.array([[w['x'], w['y']] for w in W])


def place(th, t):
    c, s = math.cos(th), math.sin(th); return wp @ np.array([[c, s], [-s, c]]) + t


def cost(q):
    dd, _ = Tp.query(q); return np.sum(np.minimum(dd, 1.0) ** 2), dd


best = None
for th0 in np.radians(np.arange(-180, 180, 2)):
    for t0 in (path[0] - place(th0, np.zeros(2))[0], np.zeros(2)):
        th, t = th0, t0.copy()
        for _ in range(30):
            q = place(th, t); _, idx = Tp.query(q)
            A = q - q.mean(0); Bm = path[idx] - path[idx].mean(0); U, S, Vt = np.linalg.svd(A.T @ Bm)
            th = th - math.atan2((U @ Vt)[0, 1], (U @ Vt)[0, 0]); t = path[idx].mean(0) - place(th, np.zeros(2)).mean(0)
        e, dd = cost(place(th, t))
        if best is None or e < best[0]: best = (e, th, t.copy())
e_id, _ = cost(wp)
if e_id <= best[0]: best = (e_id, 0.0, np.zeros(2))   # 그대로가 더 맞으면 그대로(같은 원점)
e, th, t = best
q_id = wp.copy(); q_al = place(th, t)
print(f'지도 {TITLE} · 바닥 기울기 보정 {tilt:.2f}° · 벽 칸 {len(Wc)}')
print(f'경로에 맞춘 변환: 회전 {math.degrees(th):+.1f}° · 이동 ({t[0]:+.2f}, {t[1]:+.2f}) m')
print(' 좌표 | 그대로: 경로까지 · 벽까지 | 맞춘 뒤: 경로까지 · 벽까지 (m)')
for i, l in enumerate(lab):
    print(f' {l:10s} | {Tp.query(q_id[i])[0]:.2f} · {Tw.query(q_id[i])[0]:.2f} | {Tp.query(q_al[i])[0]:.2f} · {Tw.query(q_al[i])[0]:.2f}')
fig, axs = plt.subplots(1, 2, figsize=(19, 8.8), dpi=90)
for ax, q, ttl in ((axs[0], q_id, '① 그대로 겹침 (같은 출발점·방향 가정)'), (axs[1], q_al, f'② 지도 경로에 맞춤 (회전 {math.degrees(th):+.1f}°, 이동 {t[0]:+.2f}, {t[1]:+.2f} m)')):
    im = ax.imshow(img, origin='lower', extent=ext, cmap='turbo', vmin=0, vmax=HMAX, interpolation='nearest')
    ax.plot(tr[:, 0], tr[:, 1], '-', color='w', lw=0.7, alpha=0.8)
    ax.plot(q[:, 0], q[:, 1], '--', color='magenta', lw=1.4)
    for i, ((x, y), l) in enumerate(zip(q, lab)):
        ax.scatter(x, y, s=90, c='magenta', edgecolors='k', zorder=5)
        ax.annotate(f'{i + 1} {l}\n벽 {Tw.query(q[i])[0]:.2f} m', (x, y), xytext=(6, 6), textcoords='offset points', fontsize=9.5, color='k',
                    bbox=dict(fc='white', alpha=0.8, lw=0, pad=1.5), zorder=6)
    ax.set_aspect('equal'); ax.grid(alpha=0.25, lw=0.5); ax.set_title(ttl, fontsize=11); ax.set_xlim(ext[0], ext[1]); ax.set_ylim(ext[2], ext[3])
fig.colorbar(im, ax=axs, fraction=0.02, pad=0.01).set_label('바닥 위 높이 (m)')
fig.suptitle(f'저장 좌표 {len(lab)}개 · {TITLE} · 흰 선 = 지도 녹화 로봇 경로', fontsize=12)
fig.savefig(PNG); print('그림', PNG)
