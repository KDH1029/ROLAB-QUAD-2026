#!/usr/bin/env python3
"""v2 경로(maps/route_map.json, 11-04 1459 지도 좌표)에 junk 슬라럼 길(rec1650 #6~#25, 사람이 몰아 콘 사이로 지나간 길)을
'slalom_path' 로 붙인다 — 러너는 콘 구간에서 콘을 못 알아보거나(검출 없음) 못 뚫으면(막힘) 이 길을 따라간다.
좌표 변환: maps/T_1459_from_1650.json (rec1650 3D 지도를 1459 3D 지도에 정합, 11-06: (+0.246, +0.054, +9.24°), 일치 0.65).
결과: maps/route_map_slalompath.json + maps/route_map_slalompath.png(확인 그림).  사용: python3 tools/add_slalom_path.py"""
import json, math, os
import numpy as np
J = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
T = json.load(open(os.path.join(J, 'maps', 'T_1459_from_1650.json')))['T_1459_from_1650']
c, s = math.cos(T[2]), math.sin(T[2]); tf = lambda P: np.c_[c * P[:, 0] - s * P[:, 1] + T[0], s * P[:, 0] + c * P[:, 1] + T[1]]
v2 = json.load(open(os.path.join(J, 'maps', 'route_map.json')))
jk = json.load(open(os.path.expanduser('~/ws/junk/routes/rec1650/route.json')))['route']
SP = tf(np.array([[e['x'], e['y']] for e in jk[5:25]]))          # #6(슬라럼 입구) ~ #25(끝)
out = dict(v2); out['slalom_path'] = [[round(float(x), 3), round(float(y), 3)] for x, y in SP]
out['slalom_path_src'] = 'junk routes/rec1650 #6~#25 → 1459 좌표(T_1459_from_1650)'
lab = [e['label'] for e in v2['route']]; ri, ro = lab.index('rabain'), lab.index('rabaout')
A, B = np.array([v2['route'][ri]['x'], v2['route'][ri]['y']]), np.array([v2['route'][ro]['x'], v2['route'][ro]['y']])
print(f"v2 rabain ({A[0]:.2f},{A[1]:.2f}) ↔ 슬라럼 길 시작 ({SP[0, 0]:.2f},{SP[0, 1]:.2f}) {np.hypot(*(A - SP[0])):.2f} m · rabaout ({B[0]:.2f},{B[1]:.2f}) ↔ 끝 ({SP[-1, 0]:.2f},{SP[-1, 1]:.2f}) {np.hypot(*(B - SP[-1])):.2f} m")
json.dump(out, open(os.path.join(J, 'maps', 'route_map_slalompath.json'), 'w'), ensure_ascii=False, indent=1)
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
plt.rcParams['font.family'] = 'NanumGothic'
raw = open(os.path.join(J, 'maps', 'align', 'arena1459_amcl.pgm'), 'rb').read().split(b'\n', 3)
meta = dict(l.split(': ', 1) for l in open(os.path.join(J, 'maps', 'align', 'arena1459_amcl.yaml')).read().splitlines() if ': ' in l)
w, h = map(int, raw[1].split()); img = np.frombuffer(raw[3][:w * h], np.uint8).reshape(h, w); res = float(meta['resolution']); ox, oy = [float(v) for v in meta['origin'].strip('[]').split(',')[:2]]
yy, xx = np.where(img < 100); X = np.c_[ox + (xx + 0.5) * res, oy + (h - 1 - yy + 0.5) * res]
JW = tf(np.load(os.path.expanduser('~/ws/junsu_v2/sim/sim_walls_rec1650.npy'))[:, :2])
R = np.array([[e['x'], e['y']] for e in v2['route']])
fig, ax = plt.subplots(figsize=(12, 9)); ax.scatter(X[:, 0], X[:, 1], s=0.6, c='0.55', label='1459 지도 벽(v2)')
ax.scatter(JW[:, 0], JW[:, 1], s=0.4, c='tab:green', alpha=0.4, label='rec1650 벽(변환)')
ax.plot(R[:, 0], R[:, 1], 'b.-', label='v2 경로'); [ax.annotate(e['label'], (e['x'], e['y']), fontsize=7, color='b') for e in v2['route']]
ax.plot(SP[:, 0], SP[:, 1], 'r.-', label='junk 슬라럼 길(대체 길)')
ax.set_xlim(R[:, 0].min() - 1.5, R[:, 0].max() + 1.5); ax.set_ylim(R[:, 1].min() - 1.5, R[:, 1].max() + 1.5); ax.set_aspect('equal'); ax.grid(alpha=0.3); ax.legend(fontsize=8)
ax.set_title('v2 경로 + 슬라럼 대체 길(junk) — 1459 지도 좌표'); plt.tight_layout(); plt.savefig(os.path.join(J, 'maps', 'route_map_slalompath.png'), dpi=90)
print('저장 maps/route_map_slalompath.json · .png')
