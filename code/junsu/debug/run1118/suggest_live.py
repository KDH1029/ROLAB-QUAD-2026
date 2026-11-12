#!/usr/bin/env python3
"""11:18 실주행 지도(벽+콘)로 경로 점 옮기기 제안 → maps/route_map_1106c.json(1459 좌표, 원본은 그대로)."""
import json, math, os, numpy as np
from scipy.spatial import cKDTree
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
plt.rcParams['font.family'] = 'NanumGothic'; plt.rcParams['axes.unicode_minus'] = False
J = os.path.expanduser('~/ws/junsu_v2'); H = os.path.dirname(os.path.abspath(__file__))
W = np.load(H + '/live_walls.npy'); Cc = np.load(H + '/live_cones.npy').reshape(-1, 2)
# 콘은 10 cm 칸 중심 — 콘 밑동 반지름 ~0.12 만큼 부풀린 원으로
ang = np.linspace(0, 2 * np.pi, 12, endpoint=False); Cr = (Cc[:, None, :] + 0.12 * np.stack([np.cos(ang), np.sin(ang)], 1)[None]).reshape(-1, 2)
Obs = np.vstack([W, Cr, Cc]); TO = cKDTree(Obs)
tx, ty, th = 0.1, 0.077, 0.087
c, s = math.cos(th), math.sin(th)
today = lambda X, Y: np.array([c * (X - tx) + s * (Y - ty), -s * (X - tx) + c * (Y - ty)])
to_map = lambda q: (float(c * q[0] - s * q[1] + tx), float(s * q[0] + c * q[1] + ty))
R = json.load(open(J + '/maps/route_map_1106.json')); rt = R['route']; P = np.array([today(p['x'], p['y']) for p in rt]); Q = P.copy()
FIX = {'start', 'parking', 'wall1', 'exit', 'stopline1', 'stopline2'}; RAD = 0.35
def segmin(a, b):
    n = max(2, int(np.hypot(*(b - a)) / 0.03)); t_ = np.linspace(0, 1, n)[:, None]; return float(TO.query(a + t_ * (b - a))[0].min())
def score(i, q):
    v = [TO.query(q)[0]]
    if i > 0: v.append(segmin(Q[i - 1], q) + 0.05)
    if i + 1 < len(Q): v.append(segmin(q, Q[i + 1]) + 0.05)
    return min(v)
for it in range(3):
    for i, p in enumerate(rt):
        if p['label'] in FIX: continue
        cur = score(i, Q[i])
        if TO.query(Q[i])[0] >= 0.30 and cur >= 0.22: continue
        g = np.arange(-RAD, RAD + 1e-9, 0.025); best = (cur, Q[i])
        for dx in g:
            for dy in g:
                if dx * dx + dy * dy > RAD * RAD: continue
                q = P[i] + [dx, dy]; sc = score(i, q) - 0.12 * math.hypot(dx, dy)
                if sc > best[0] + 1e-6: best = (sc, q)
        Q[i] = best[1]
print(' 점          여유(점) 지금 → 제안   구간(다음까지) 지금 → 제안   옮김 m')
moved = []
for i, p in enumerate(rt):
    a0, b0 = TO.query(P[i])[0], TO.query(Q[i])[0]
    sa = segmin(P[i], P[i + 1]) if i + 1 < len(P) else 9; sb = segmin(Q[i], Q[i + 1]) if i + 1 < len(Q) else 9
    mv = float(np.hypot(*(Q[i] - P[i])))
    if mv > 0.02: moved.append((p['label'], round(mv, 2)))
    print(f"  {p['label']:10s}   {a0:5.2f} → {b0:5.2f}        {sa:5.2f} → {sb:5.2f}          {mv:4.2f} {'★' if mv > 0.02 else ''}")
out = json.loads(json.dumps(R))
for p, q in zip(out['route'], Q): p['x'], p['y'] = [round(v, 3) for v in to_map(q)]
out['saved'] = '2026-11-06 12시 반 자동 제안 — 11:18 실주행 라이다 벽·콘 기준(원본 route_map_1106.json 은 그대로)'; out['problems'] = [f'{l} {m} m 옮김' for l, m in moved]
json.dump(out, open(J + '/maps/route_map_1106c.json', 'w'), ensure_ascii=False, indent=1); print('옮긴 점', moved); print('저장 maps/route_map_1106c.json')
fig, ax = plt.subplots(figsize=(14, 9.5)); ax.scatter(W[:, 0], W[:, 1], s=2, c='k', label='벽(실주행 라이다)')
ax.scatter(Cc[:, 0], Cc[:, 1], s=55, marker='^', c='orange', edgecolors='k', label='콘(실주행 인식)')
ax.plot(P[:, 0], P[:, 1], '-', c='0.6', lw=1.5, label='지금 경로'); ax.plot(Q[:, 0], Q[:, 1], 'r.-', lw=1.6, ms=8, label='제안 경로(route_map_1106c)')
for p, a, b in zip(rt, P, Q):
    if np.hypot(*(b - a)) > 0.02: ax.annotate('', b, a, arrowprops=dict(arrowstyle='->', color='m', lw=1.6))
    ax.annotate(p['label'], b, fontsize=8, color='red', xytext=(3, 3), textcoords='offset points')
ax.set_aspect('equal'); ax.grid(alpha=.3); ax.legend(loc='upper left', fontsize=9)
ax.set_xlim(P[:, 0].min() - 1, P[:, 0].max() + 1); ax.set_ylim(P[:, 1].min() - 1, P[:, 1].max() + 1)
ax.set_title('경로 제안(11:18 실주행 벽·콘 기준) — 보라 화살표 = 옮긴 점, 순서 그대로'); plt.tight_layout(); plt.savefig(H + '/route_suggest_live.png', dpi=85)
