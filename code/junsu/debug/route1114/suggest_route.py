#!/usr/bin/env python3
"""지금 경로(1459 좌표)를 rec1114(새 콘 배치) 장애물(편집기 벽·큰 콘 점)로 점검하고, 여유가 모자란 점을 반경 0.4 m 안에서
'그 점 + 앞뒤 구간' 여유가 가장 큰 자리로 옮기는 제안. 순서·라벨 그대로. 결과: maps/route_map_1106b.json(1459 좌표) + 그림."""
import json, os, math, numpy as np
from scipy.spatial import cKDTree
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
plt.rcParams['font.family'] = 'NanumGothic'; plt.rcParams['axes.unicode_minus'] = False
J = os.path.expanduser('~/ws/junsu_v2'); H = os.path.dirname(os.path.abspath(__file__))
SRC = J + '/maps/route_map_1106.json'; DST = J + '/maps/route_map_1106b.json'
W = np.load(J + '/maps/rec1114/editor_cache.npz')['walls'].astype(float); W = W[(W[:, 0] > -3) & (W[:, 0] < 9) & (W[:, 1] > -2.5) & (W[:, 1] < 6)]; T = cKDTree(W)
x0, y0, th = json.load(open(H + '/T_1114_from_1459.json'))['T_1114_from_1459']
fw = lambda P: np.asarray(P) @ np.array([[math.cos(th), math.sin(th)], [-math.sin(th), math.cos(th)]]) + [x0, y0]
bw = lambda Q: (np.asarray(Q) - [x0, y0]) @ np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
R = json.load(open(SRC)); rt = R['route']; P = fw([[p['x'], p['y']] for p in rt])
WANT, SEGW, RAD = 0.35, 0.30, 0.40        # 점 여유 목표(몸 반폭 0.165 + ego 벽 여유 0.20 근처) · 구간 여유 목표 · 옮길 수 있는 반경
FIX = {'start', 'parking', 'wall1', 'exit', 'stopline1', 'stopline2'}   # 규정 자리(출발·주차·출구·정지선)는 안 옮김
def segmin(a, b):
    n = max(2, int(np.hypot(*(b - a)) / 0.03)); s = np.linspace(0, 1, n)[:, None]; return float(T.query(a + s * (b - a))[0].min())
def score(i, q, Q):
    v = [T.query(q)[0]]
    if i > 0: v.append(segmin(Q[i - 1], q) + 0.05)
    if i + 1 < len(Q): v.append(segmin(q, Q[i + 1]) + 0.05)
    return min(v)
Q = P.copy(); rows = []
for it in range(2):          # 앞뒤 점이 같이 움직이므로 두 번
    for i, p in enumerate(rt):
        if p['label'] in FIX: continue
        cur = score(i, Q[i], Q)
        if T.query(P[i])[0] >= WANT and cur >= SEGW: continue
        g = np.arange(-RAD, RAD + 1e-9, 0.03); best = (cur, Q[i])
        for dx in g:
            for dy in g:
                if dx * dx + dy * dy > RAD * RAD: continue
                q = P[i] + [dx, dy]; sc = score(i, q, Q) - 0.15 * math.hypot(dx, dy)   # 덜 옮기는 쪽 조금 선호
                if sc > best[0] + 1e-6: best = (sc, q)
        Q[i] = best[1]
print(' 점          지금 여유(점/앞뒤구간 m)   →  제안(점/앞뒤구간)   옮김 m')
moved = []
for i, p in enumerate(rt):
    a = (T.query(P[i])[0], min([segmin(P[i - 1], P[i])] if i else [9] + []), min([segmin(P[i], P[i + 1])] if i + 1 < len(P) else [9]))
    b = (T.query(Q[i])[0], min([segmin(Q[i - 1], Q[i])] if i else [9]), min([segmin(Q[i], Q[i + 1])] if i + 1 < len(Q) else [9]))
    mv = float(np.hypot(*(Q[i] - P[i]))); flag = '★' if mv > 0.02 else ('!' if min(a) < 0.25 else '')
    if mv > 0.02: moved.append((p['label'], round(mv, 2)))
    print(f"  {p['label']:10s} {a[0]:.2f}/{min(a[1], a[2]):.2f}            →  {b[0]:.2f}/{min(b[1], b[2]):.2f}         {mv:.2f} {flag}")
print('옮긴 점:', moved)
out = json.loads(json.dumps(R)); Qb = bw(Q)
for p, q in zip(out['route'], Qb): p['x'], p['y'] = round(float(q[0]), 3), round(float(q[1]), 3)
out['saved'] = '2026-11-06 자동 제안(rec1114 장애물) — 원본 route_map_1106.json'; out['problems'] = [f'{l} {m} m 옮김' for l, m in moved]
json.dump(out, open(DST, 'w'), ensure_ascii=False, indent=1); print('저장', DST)
fig, ax = plt.subplots(figsize=(14, 9.5)); ax.scatter(W[:, 0], W[:, 1], s=1.5, c='k', label='rec1114 벽·큰 콘(편집기 추출)')
ax.plot(P[:, 0], P[:, 1], '-', c='0.6', lw=1.5, label='지금 경로'); ax.plot(Q[:, 0], Q[:, 1], 'r.-', lw=1.6, ms=8, label='제안 경로(route_map_1106b)')
for p, a, b in zip(rt, P, Q):
    if np.hypot(*(b - a)) > 0.02: ax.annotate('', b, a, arrowprops=dict(arrowstyle='->', color='m', lw=1.5))
    ax.annotate(p['label'], b, fontsize=8, color='red', xytext=(3, 3), textcoords='offset points')
    ax.add_patch(plt.Circle(b, 0.165, fill=False, color='r', lw=0.6))
ax.set_aspect('equal'); ax.set_xlim(-1.5, 8.0); ax.set_ylim(-1.2, 5.0); ax.grid(alpha=.3); ax.legend(loc='upper left')
ax.set_title('경로 제안 — 보라 화살표 = 옮긴 점(반경 0.4 m 안, 순서 그대로), 빨간 원 = 몸 반폭'); plt.tight_layout(); plt.savefig(H + '/route_suggest_1114.png', dpi=85)
