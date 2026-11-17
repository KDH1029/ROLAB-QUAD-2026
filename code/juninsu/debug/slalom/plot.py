#!/usr/bin/env python3
"""슬라럼 한 번 돈 결과 그림·숫자: 지도 벽(검정) + 찍은 점(번호) + 실제 지나간 길(초록, 몸이 벽에 닿은 곳 빨강) + 1초마다 몸 윤곽.
사용: ~/venv/go2/bin/python3 plot.py <runs/시각 폴더>   → <폴더>/slalom.png"""
import csv, json, math, os, sys
import numpy as np
import matplotlib; matplotlib.use('Agg'); matplotlib.rcParams['font.family'] = 'NanumGothic'; matplotlib.rcParams['axes.unicode_minus'] = False
import matplotlib.pyplot as plt

RUN = os.path.abspath(sys.argv[1]); J = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
R_ = json.load(open(os.path.join(RUN, 'route.json'))); name = os.path.basename(os.path.dirname(R_['source']))
sys.argv = ['x', name]; sys.path.insert(0, os.path.join(J, 'tools')); import route_editor as ed   # 벽 = 편집기와 같은 것(로봇·사람 점 뺀)
from scipy.spatial import cKDTree
W = ed.real_walls(ed.load_cache()); TW = cKDTree(W)
T = json.load(open(os.path.join(RUN, 'route_today.json.align.json')))['T']; c, s = math.cos(T[2]), math.sin(T[2])
rows = list(csv.DictReader(open(os.path.join(RUN, 'trace.csv')))) if os.path.exists(os.path.join(RUN, 'trace.csv')) else []
if not rows:
    sys.exit('trace.csv 없음/빔 — 러너가 위치를 못 받았음(FAST-LIO?)')
t = np.array([float(r['t']) for r in rows]); x0 = np.array([float(r['x']) for r in rows]); y0 = np.array([float(r['y']) for r in rows])
X = c * x0 - s * y0 + T[0]; Y = s * x0 + c * y0 + T[1]; A = np.array([float(r['yaw']) for r in rows]) + T[2]
L2, W2 = 0.35, 0.155                                                     # Go2 몸 0.70 × 0.31
def hits(x, y, a):
    idx = TW.query_ball_point([x, y], 0.40)
    if not idx: return 0
    d = W[idx] - [x, y]; lx = d[:, 0] * math.cos(a) + d[:, 1] * math.sin(a); ly = -d[:, 0] * math.sin(a) + d[:, 1] * math.cos(a)
    return int(np.sum((np.abs(lx) < L2) & (np.abs(ly) < W2)))
H = np.array([hits(*p) for p in zip(X, Y, A)]); clr = TW.query(np.c_[X, Y])[0]
P = R_['route']; PX = np.array([[e['x'], e['y']] for e in P])
fig, ax = plt.subplots(figsize=(8, 11))
ax.scatter(W[:, 0], W[:, 1], s=3, c='k')
ax.plot(PX[:, 0], PX[:, 1], '--', c='royalblue', lw=1, label='찍은 경로')
for e in P:
    ax.plot(e['x'], e['y'], 'o', c='royalblue', ms=5); ax.annotate(str(e['no']), (e['x'], e['y']), xytext=(4, 3), textcoords='offset points', fontsize=9, color='royalblue')
ax.plot(X, Y, c='limegreen', lw=2, label='실제 지나간 길')
bad = H > 0; ax.scatter(X[bad], Y[bad], c='red', s=12, zorder=5, label=f'몸이 벽·콘에 닿음({bad.sum()}회)')
for k in range(0, len(t), 10):                                           # 1초마다 몸 윤곽
    ca, sa = math.cos(A[k]), math.sin(A[k]); cr = np.array([[L2, W2], [L2, -W2], [-L2, -W2], [-L2, W2], [L2, W2]])
    ax.plot(X[k] + cr[:, 0] * ca - cr[:, 1] * sa, Y[k] + cr[:, 0] * sa + cr[:, 1] * ca, c='red' if H[k] else 'gray', lw=0.6, alpha=0.6)
m = 0.8; ax.set_xlim(min(PX[:, 0].min(), X.min()) - m, max(PX[:, 0].max(), X.max()) + m); ax.set_ylim(min(PX[:, 1].min(), Y.min()) - m, max(PX[:, 1].max(), Y.max()) + m)
ax.set_aspect('equal'); ax.grid(alpha=0.3); ax.legend(loc='best', fontsize=8)
prm = open(os.path.join(RUN, 'params.txt')).read().strip() if os.path.exists(os.path.join(RUN, 'params.txt')) else ''
dur = t[-1] - t[0]; dist = float(np.hypot(np.diff(X), np.diff(Y)).sum()); last = rows[-1]
ax.set_title(f'{os.path.basename(RUN)}  {dur:.0f}초 · {dist:.1f} m · 벽까지 최소 {clr.min():.2f} m\n{prm}  끝 상태 {last["state"]} 점 {last["wp_i"]}/{last["wp_n"]}', fontsize=9)
out = os.path.join(RUN, 'slalom.png'); fig.savefig(out, dpi=90, bbox_inches='tight')
print(f'{out}\n  {dur:.0f}초 · {dist:.1f} m · 끝 상태 {last["state"]} · 몸 중심~벽 최소 {clr.min():.2f} m · 몸이 닿은 기록 {bad.sum()}/{len(t)}')
if bad.any():
    near = [int(np.argmin(np.hypot(*(PX - [X[k], Y[k]]).T))) for k in np.where(bad)[0]]
    print('  닿은 곳(가까운 점 번호):', sorted({P[n]['no'] for n in near}))
