#!/usr/bin/env python3
"""slalom 에이전트: 여러 판 궤적을 한 그림에(러너 상태별 색, δ10 접촉 빨강). 사용: plot_slalom.py <출력.png> <결과 폴더>..."""
import json, os, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams['font.family'] = 'NanumGothic'; plt.rcParams['axes.unicode_minus'] = False
SIM = os.path.expanduser('~/ws/junsu_v2/sim')
W = np.load(os.path.join(SIM, 'sim_walls_rec1650.npy'))[:, :2]; C = np.load(os.path.join(SIM, 'sim_cones_rec1650.npy'))
X0, X1, Y0, Y1 = 3.8, 7.4, -1.3, 3.6
W = W[(W[:, 0] > X0) & (W[:, 0] < X1) & (W[:, 1] > Y0) & (W[:, 1] < Y1)]
COL = {'TO_GOAL': '#1f77b4', 'PIN_TRACK': '#2ca02c', 'BACKOFF': '#ff7f0e', 'ROUTE_FOLLOW': '#9467bd', 'ROTATE': '#000000'}
out, runs = sys.argv[1], sys.argv[2:]
n = len(runs); nc = min(4, n); nr = (n + nc - 1) // nc
fig, axs = plt.subplots(nr, nc, figsize=(4.2 * nc, 5.4 * nr), squeeze=False)
for ax, O in zip(axs.flat, runs):
    E = [json.loads(l) for l in open(os.path.join(O, 'world.jsonl')) if '"pose"' in l]
    R = json.load(open(os.path.join(O, 'route.json'))); sp = np.array(R['slalom_path'])
    ax.scatter(W[:, 0], W[:, 1], s=1, c='#bbbbbb'); ax.scatter(C[:, 0], C[:, 1], s=10, c='orange')
    ax.plot(sp[:, 0], sp[:, 1], '--', c='#2ca02c', lw=0.8)
    rp = np.array([[e['x'], e['y']] for e in R['route']]); ax.plot(rp[:, 0], rp[:, 1], 'o', c='b', ms=3)
    for st, c in COL.items():
        q = np.array([[e['x'], e['y']] for e in E if e['rs'] == st])
        if len(q):
            ax.scatter(q[:, 0], q[:, 1], s=2, c=c, label=st)
    q = np.array([[e['x'], e['y']] for e in E if e['gap'] < -0.10 and e['rs'] not in ('', 'START_CHECK', 'DONE')])
    if len(q):
        ax.scatter(q[:, 0], q[:, 1], s=6, c='red', label='접촉 δ10')
    r = None
    try:
        r = next(json.loads(l) for l in open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'results.jsonl')) if json.loads(l)['name'] == os.path.basename(O.rstrip('/')))
    except (StopIteration, OSError):
        pass
    ttl = os.path.basename(O.rstrip('/'))
    if r:
        ttl += f"\n{'완주' if r['done'] else '미완'} {r['total_s']}s · 덮음 {r['sl_cov']} · δ10 {r['d10_sl'][0]}s"
    ax.set_title(ttl, fontsize=9); ax.set_xlim(X0, X1); ax.set_ylim(Y0, Y1); ax.set_aspect('equal'); ax.grid(alpha=0.3)
axs.flat[0].legend(fontsize=6, loc='lower left', markerscale=3)
for ax in list(axs.flat)[n:]:
    ax.axis('off')
plt.tight_layout(); plt.savefig(out, dpi=90); print(out)
