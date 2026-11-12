#!/usr/bin/env python3
"""X 통로 입구(line1→stopline1) 확대: 두꺼운/얇은 벽, 콘, 경로, 여러 판의 길. simcal"""
import json, os, sys
import numpy as np
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
plt.rcParams['font.family'] = 'NanumGothic'; plt.rcParams['axes.unicode_minus'] = False
S = os.path.expanduser('~/ws/junsu_v2/sim')
W = np.load(f'{S}/sim_walls_rec1650.npy'); Wt = np.load(f'{S}/sim_walls_rec1650_thin.npy'); C = np.load(f'{S}/sim_cones_rec1650.npy')
R = json.load(open(f'{S}/route_main1654_in1650.json'))['route']; RP = np.array([[e['x'], e['y']] for e in R])
out, x0, x1, y0, y1 = sys.argv[1], *map(float, sys.argv[2:6]); runs = sys.argv[6:]
fig, ax = plt.subplots(figsize=(10, 8))
ax.scatter(W[:, 0], W[:, 1], s=6, c='0.75', marker='s', label='벽 칸(기본)'); ax.scatter(Wt[:, 0], Wt[:, 1], s=6, c='0.35', marker='s', label='벽 칸(얇게)')
ax.scatter(C[:, 0], C[:, 1], s=40, c='orange', label='콘 중심')
ax.plot(RP[:, 0], RP[:, 1], 'b.-', ms=8, label='16:54 경로(1650 좌표)')
for e in R: ax.annotate(e['label'], (e['x'], e['y']), fontsize=8, color='b')
cols = plt.cm.tab10.colors
for i, r in enumerate(runs):
    P = [json.loads(l) for l in open(os.path.join(r, 'world.jsonl')) if '"pose"' in l]
    x = np.array([e['x'] for e in P]); y = np.array([e['y'] for e in P]); g = np.array([e['gap'] for e in P])
    ax.plot(x, y, '-', c=cols[i % 10], lw=1, label=os.path.basename(r.rstrip('/')))
    k = g < -0.10; ax.scatter(x[k], y[k], s=5, c='r')
ax.set_xlim(x0, x1); ax.set_ylim(y0, y1); ax.set_aspect('equal'); ax.grid(alpha=0.3); ax.legend(fontsize=7, loc='lower right')
ax.set_title('X 통로 입구 확대 — 빨간 점 = 몸이 지도 장애물에 10 cm 넘게 파고듦')
plt.tight_layout(); plt.savefig(out, dpi=90); print(out)
