#!/usr/bin/env python3
"""v2 시뮬 한 판 그림 — 벽(회색)·콘 중심(주황)·경로 점(파랑)·로봇 길(시간 색)·보정된 목표(빨강 x)·접촉(δ10, 빨강 점). 사용: plot_run_v2.py <결과 폴더> [x0 x1 y0 y1]"""
import json, os, re, sys
import numpy as np
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
plt.rcParams['font.family'] = 'NanumGothic'; plt.rcParams['axes.unicode_minus'] = False
O = sys.argv[1]; S = os.path.dirname(os.path.abspath(__file__))
W = np.load(os.path.join(S, 'sim_walls_rec1650.npy')); C = np.load(os.path.join(S, 'sim_cones_rec1650.npy'))
R = json.load(open(os.path.join(O, 'route.json')))['route']; RP = np.array([[e['x'], e['y']] for e in R])
P = [json.loads(l) for l in open(os.path.join(O, 'world.jsonl'))]; P = [e for e in P if e['ev'] == 'pose']
x = np.array([e['x'] for e in P]); y = np.array([e['y'] for e in P]); t = np.array([e['t'] for e in P]); g = np.array([e['gap'] for e in P])
rl = open(os.path.join(O, 'runner.log')).read()
SN = [(float(a), float(b)) for a, b in re.findall(r'목표 보정: .*?→ \(([-\d.]+),([-\d.]+)\)', rl)]
fig, ax = plt.subplots(figsize=(13, 9))
ax.scatter(W[:, 0], W[:, 1], s=0.5, c='0.6'); ax.scatter(C[:, 0], C[:, 1], s=14, c='orange', label='콘 중심(시뮬)')
ax.plot(RP[:, 0], RP[:, 1], 'b.-', lw=0.8, ms=5, label='경로 점')
for i, e in enumerate(R):
    ax.annotate(f'{i + 1}', (e['x'], e['y']), fontsize=6, color='b')
sc = ax.scatter(x, y, c=t, s=3, cmap='viridis', label='로봇 길(색 = 시간)'); plt.colorbar(sc, ax=ax, label='초')
k = g < -0.10; ax.scatter(x[k], y[k], s=8, c='r', label='접촉(δ 10 cm)')
if SN:
    sn = np.array(SN); ax.scatter(sn[:, 0], sn[:, 1], marker='x', s=60, c='r', label='보정된 목표')
if len(sys.argv) > 5:
    ax.set_xlim(float(sys.argv[2]), float(sys.argv[3])); ax.set_ylim(float(sys.argv[4]), float(sys.argv[5]))
ax.set_aspect('equal'); ax.grid(alpha=0.3); ax.legend(fontsize=8, loc='upper left'); ax.set_title(os.path.basename(O) + ' — ' + open(os.path.join(O, 'report.txt')).readline().strip() if os.path.exists(os.path.join(O, 'report.txt')) else '')
plt.tight_layout(); plt.savefig(os.path.join(O, 'run.png'), dpi=90); print(os.path.join(O, 'run.png'))
