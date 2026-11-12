#!/usr/bin/env python3
"""저장한 좌표(waypoint_logger)를 FAST-LIO 지도 위에 맞춰 본다. 재생 없음(파일만 읽음).
사용: python3 waypoints_vs_map.py <지도 npz(map_collect)> <좌표 json> <출력 png>
좌표 세션과 지도 녹화는 FAST-LIO 를 따로 켰으므로 원점·방향이 다를 수 있다 →
좌표 8개가 지도 녹화의 '로봇이 실제로 걸은 길' 위에 오도록 2D 강체변환(x, y, 회전)을 찾는다.
(로봇은 코스를 따라 걸었으니 정지선·콘 구간·주차 좌표는 그 길 근처에 있어야 한다.)"""
import sys, json, math, numpy as np
NPZ, WP, PNG = sys.argv[1:4]
d = np.load(NPZ); V = float(d['V']); P = (d['vox'][d['cnt'] >= 2].astype(np.float64) + 0.5) * V
od = d['odom']; od = od[np.argsort(od[:, 1])]; path = od[:, 2:4]
W = json.load(open(WP)); lab = [w['label'] for w in W]; wp = np.array([[w['x'], w['y']] for w in W])
# 경로를 5 cm 간격으로 촘촘히
seg = [path[0]]
for a, b in zip(path[:-1], path[1:]):
    n = max(1, int(np.hypot(*(b - a)) / 0.05)); seg += [a + (b - a) * k / n for k in range(1, n + 1)]
path = np.array(seg)
from scipy.spatial import cKDTree
T = cKDTree(path)
def cost(th, t):
    c, s = math.cos(th), math.sin(th); q = wp @ np.array([[c, s], [-s, c]]) + t
    dd, _ = T.query(q); return np.sum(np.minimum(dd, 1.0) ** 2), q, dd
best = None
for th in np.radians(np.arange(-180, 180, 2)):
    c, s = math.cos(th), math.sin(th); q0 = wp @ np.array([[c, s], [-s, c]])
    for t0 in (path[0] - q0[0], np.zeros(2)):           # 출발 좌표 = 지도 출발점 가정 / 원점 그대로 가정
        t = t0.copy()
        for _ in range(30):                              # 가까운 경로점으로 당기기(ICP)
            _, q, _ = cost(th, t); _, idx = T.query(q); t = t + (path[idx] - q).mean(0)
            # 회전도 조금씩: 2D Procrustes
            A = q - q.mean(0); Bm = path[idx] - path[idx].mean(0); H = A.T @ Bm; U, S, Vt = np.linalg.svd(H)
            dth = math.atan2((U @ Vt)[0, 1], (U @ Vt)[0, 0]); th = th - dth
            c, s = math.cos(th), math.sin(th); q0 = wp @ np.array([[c, s], [-s, c]]); t = path[idx].mean(0) - q0.mean(0)
        e, q, dd = cost(th, t)
        if best is None or e < best[0]: best = (e, th, t.copy(), q, dd)
e0, q_id, dd_id = cost(0.0, np.zeros(2))
if e0 <= best[0]: best = (e0, 0.0, np.zeros(2), q_id, dd_id)   # 그대로가 더 맞으면 그대로(같은 원점)
e, th, t, q, dd = best
print(json.dumps({'맞춘 변환': {'회전_deg': round(math.degrees(th), 1), '이동_m': [round(float(t[0]), 2), round(float(t[1]), 2)]},
                  '좌표별 길까지 거리 m (맞춘 뒤)': {l: round(float(x), 2) for l, x in zip(lab, dd)},
                  '좌표별 길까지 거리 m (그대로 겹쳤을 때)': {l: round(float(x), 2) for l, x in zip(lab, dd_id)},
                  '중앙값 (맞춘 뒤 / 그대로)': [round(float(np.median(dd)), 2), round(float(np.median(dd_id)), 2)]}, ensure_ascii=False, indent=1))
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
plt.rcParams['font.family'] = ['NanumGothic', 'DejaVu Sans']
floor0 = np.percentile(P[np.hypot(P[:, 0], P[:, 1]) < 1.5, 2], 5)
m = (P[:, 2] > floor0 + 0.08) & (P[:, 2] < floor0 + 1.0)
lo, hi = path.min(0) - 1.5, path.max(0) + 1.5
m &= (P[:, 0] > lo[0]) & (P[:, 0] < hi[0]) & (P[:, 1] > lo[1]) & (P[:, 1] < hi[1])
fig, ax = plt.subplots(1, 2, figsize=(18, 8))
for a_, qq, ttl in ((ax[0], q_id, '그대로 겹침(같은 원점이라고 가정)'), (ax[1], q, '경로에 맞춘 뒤 (회전 %.1f°, 이동 %.2f, %.2f m)' % (math.degrees(th), t[0], t[1]))):
    a_.scatter(P[m, 0], P[m, 1], s=0.3, c='0.55'); a_.plot(path[:, 0], path[:, 1], 'b-', lw=0.8, alpha=0.6, label='14:59 녹화 로봇 경로')
    a_.plot(qq[:, 0], qq[:, 1], 'r--', lw=1)
    for (x, y), l in zip(qq, lab):
        a_.scatter(x, y, s=60, c='r', zorder=5); a_.annotate(l, (x, y), xytext=(4, 4), textcoords='offset points', fontsize=10, color='darkred')
    a_.set_title(ttl); a_.axis('equal'); a_.grid(alpha=0.3); a_.legend(loc='upper right')
plt.tight_layout(); plt.savefig(PNG, dpi=90); print('그림', PNG)
