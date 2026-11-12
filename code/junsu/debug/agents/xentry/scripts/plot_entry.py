#!/usr/bin/env python3
"""입구 부근 벽 + 여러 판 궤적(X 통과 전까지) 겹쳐 그리기."""
import json, sys, os
import numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
plt.rcParams['font.family'] = 'NanumGothic'; plt.rcParams['axes.unicode_minus'] = False
S = os.path.expanduser('~/ws/junsu_v2/sim'); R = S + '/runs_v2'
out = sys.argv[1]; runs = sys.argv[2:]
W = np.load(S + '/sim_walls_rec1650.npy'); C = np.load(S + '/sim_cones_rec1650.npy')
x0, x1, y0, y1 = -0.6, 4.6, -1.2, 2.4
fig, ax = plt.subplots(figsize=(13, 9))
m = (W[:, 0] > x0) & (W[:, 0] < x1) & (W[:, 1] > y0) & (W[:, 1] < y1)
ax.scatter(W[m, 0], W[m, 1], s=30, c='k', marker='s', label='시뮬 벽 칸(0.12 m)')
ax.scatter(C[:, 0], C[:, 1], s=40, c='orange', marker='^', label='콘')
cols = plt.cm.tab10.colors
for i, n in enumerate(runs):
    d = n if os.path.isdir(n) else os.path.join(R, n)
    P = [json.loads(l) for l in open(d + '/world.jsonl')]; P = [p for p in P if p.get('ev') == 'pose']
    tx0 = next((p['t'] for p in P if p['x'] > 2.3 and p['y'] > 0.4), None); tx = tx0 if tx0 is not None else P[-1]['t']
    Q = np.array([[p['x'], p['y']] for p in P if p['t'] <= tx + 3])
    ax.plot(Q[:, 0], Q[:, 1], '-', color=cols[i % 10], lw=1.4, label=f"{os.path.basename(d)} (" + (f"X통과 {tx0:.0f}s" if tx0 is not None else f"X 못 지남, {tx:.0f}s까지") + ")" )
    rt = json.load(open(d + '/route.json'))['route']
    for p in rt[:5]: ax.plot(p['x'], p['y'], 'x', color=cols[i % 10], ms=10, mew=2); ax.annotate(p['label'], (p['x'], p['y']), fontsize=7, color=cols[i % 10])
ax.set_xlim(x0, x1); ax.set_ylim(y0, y1); ax.set_aspect('equal'); ax.grid(alpha=.3); ax.legend(fontsize=7, loc='upper left')
ax.set_title(os.environ.get('TITLE', '첫 X 통로 입구: 벽 칸과 판별 궤적(X 통과 전까지)'))
ax.plot(1.98, 0.10, 'o', mfc='none', mec='orange', ms=22, mew=2.5)
fig.savefig(out, dpi=110, bbox_inches='tight'); print(out)
