#!/usr/bin/env python3
"""ego 진단 기록(ego_rec.py) → 입구 부근 스냅샷: 실제 시뮬 벽 · ego 팽창 격자 · 스캔 · 계획 경로 · 목표.
사용: plot_rec.py <rec.jsonl> <out.png> [시각들(기록 시작 기준 초, 쉼표) | auto]"""
import json, sys, os
import numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
plt.rcParams['font.family'] = 'NanumGothic'; plt.rcParams['axes.unicode_minus'] = False
S = os.path.expanduser('~/ws/junsu_v2/sim')
R = [json.loads(l) for l in open(sys.argv[1]) if l.strip()]
T0 = R[0]['tw']; W = np.load(os.environ.get('WALLS', S + '/sim_walls_rec1650.npy'))
for r in R: r['t'] = r['tw'] - T0
od = [r for r in R if r['ev'] == 'odom']; gr = [r for r in R if r['ev'] == 'grid']; pa = [r for r in R if r['ev'] == 'path']
go = [r for r in R if r['ev'] == 'goal']; sc = [r for r in R if r['ev'] == 'scan']
def at(L, t):
    c = [r for r in L if r['t'] <= t]; return c[-1] if c else None
if len(sys.argv) > 3 and sys.argv[3] != 'auto':
    ts = [float(v) for v in sys.argv[3].split(',')]
else:   # auto: 첫 북쪽 계획(경로 점이 x<1.45, y>0.45) 직전·직후 + 첫 목표 바뀜 뒤
    ts = []
    g2 = [g for g in go if abs(g['x'] - go[0]['x']) > 0.3 or abs(g['y'] - go[0]['y']) > 0.3]
    if g2: ts.append(g2[0]['t'] + 0.3)
    north = [p for p in pa if any(x < 1.40 and y > 0.45 for x, y in p['p'])]
    if north: ts += [north[0]['t'] - 0.3, north[0]['t'] + 0.05]
    if not ts: ts = [od[len(od) // 3]['t']]
n = len(ts); fig, axs = plt.subplots(1, n, figsize=(7.5 * n, 7.5), squeeze=False)
x0, x1, y0, y1 = 0.3, 3.4, -0.7, 1.9
for ax, t in zip(axs[0], ts):
    m = (W[:, 0] > x0) & (W[:, 0] < x1) & (W[:, 1] > y0) & (W[:, 1] < y1)
    ax.scatter(W[m, 0], W[m, 1], s=6, c='k', marker='s', label='시뮬 벽 칸(참값)', zorder=3)
    g = at(gr, t)
    if g and g['p']:
        G = np.array(g['p']); ax.scatter(G[:, 0], G[:, 1], s=34, c='#f4a6a6', marker='s', alpha=.75, label=f'ego 팽창 격자(0.20 m, {g["t"]:.1f}s, 2칸마다)', zorder=1)
    s = at(sc, t)
    if s:
        P = np.array(s['p']); k = (P[:, 0] > x0) & (P[:, 0] < x1) & (P[:, 1] > y0) & (P[:, 1] < y1)
        ax.scatter(P[k, 0], P[k, 1], s=8, c='b', label='스캔 점', zorder=4)
    p = at(pa, t)
    if p:
        Q = np.array(p['p']); ax.plot(Q[:, 0], Q[:, 1], 'g-', lw=2.5, label=f'ego 계획 경로({p["t"]:.1f}s)', zorder=5); ax.plot(Q[-1, 0], Q[-1, 1], 'g*', ms=14, zorder=5)
    o = at(od, t)
    if o:
        ax.plot(o['x'], o['y'], 'mo', ms=10, zorder=6); ax.arrow(o['x'], o['y'], 0.3 * np.cos(o['yaw']), 0.3 * np.sin(o['yaw']), color='m', width=0.015, zorder=6)
        tr = np.array([[r['x'], r['y']] for r in od if r['t'] <= t]); ax.plot(tr[:, 0], tr[:, 1], 'm:', lw=1, zorder=2)
    gg = at(go, t)
    if gg: ax.plot(gg['x'], gg['y'], 'rX', ms=14, label=f'목표 ({gg["x"]:.2f},{gg["y"]:.2f})', zorder=6)
    ax.plot(1.98, 0.10, 'o', mfc='none', mec='orange', ms=22, mew=2.5, label='외톨이 벽 칸 (1.98,0.10)', zorder=7)
    ax.set_xlim(x0, x1); ax.set_ylim(y0, y1); ax.set_aspect('equal'); ax.grid(alpha=.3); ax.set_title(f'기록 {t:.1f}s'); ax.legend(fontsize=7, loc='upper left')
fig.tight_layout(); fig.savefig(sys.argv[2], dpi=95); print(sys.argv[2], ts)
