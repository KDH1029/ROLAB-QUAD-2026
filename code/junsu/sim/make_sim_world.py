#!/usr/bin/env python3
"""시뮬레이션 재료 만들기 — 14:59 FAST-LIO 지도(arena1104b)에서 '수직 벽'만 골라 장애물로, 녹화 사진에서 빨강/초록 신호등 사진을 고른다.
사용: ~/venv/go2/bin/python3 make_sim_world.py
벽 고르기: 4 cm 기둥마다 바닥+5 cm ~ +1 m 사이에 점이 '세로로 20 cm 이상 이어지면' 벽(경기장 벽 60 cm). 경사로·계단 디딤면(한 높이에만 점)은 빠진다.
  → 실제 스택의 wall_filter(수직면만 통과)와 같은 취지. 라바콘(23 cm)은 대부분 빠진다 — 실제 wall_filter 도 대부분 걸러낸다(11-04 측정 35% 통과)."""
import os, json, glob, shutil, numpy as np
H = os.path.dirname(os.path.abspath(__file__))
d = np.load(os.path.expanduser('~/ws/v_gosu/eval/out/arena1104b_map.npz')); V = float(d['V'])
vox = d['vox']; cnt = d['cnt']; P = (vox + 0.5) * V
od = d['odom']; od = od[np.argsort(od[:, 1])]; path = od[:, 2:4]
lo, hi = path.min(0) - 1.5, path.max(0) + 1.5
m = (P[:, 0] > lo[0]) & (P[:, 0] < hi[0]) & (P[:, 1] > lo[1]) & (P[:, 1] < hi[1]); P, cnt = P[m], cnt[m]
# ★ 이 지도는 수직 흐트러짐(약 19 cm)이 있어 '세로로 이어짐'만으로는 경사로·계단도 벽이 된다 →
#   기둥(4 cm)마다 '주변 60 cm 안 가장 낮은 바닥'보다 45 cm 이상 솟은 점이 여러 번(≥6) 잡힌 기둥만 벽.
#   경기장 벽 60 cm 는 남고, 경사로·가운데 단·계단(그보다 낮음)·라바콘(23 cm)은 빠진다.
G = 0.04; key = np.floor(P[:, :2] / G).astype(np.int64); k0 = key.min(0); kk = key - k0; W_, H_ = kk.max(0) + 1
zmin = np.full((W_, H_), np.inf); np.minimum.at(zmin, (kk[:, 0], kk[:, 1]), P[:, 2])
from scipy.ndimage import minimum_filter
zmin[~np.isfinite(zmin)] = 1e9; ground = minimum_filter(zmin, size=15)          # 15칸 = 60 cm
hgt = P[:, 2] - ground[kk[:, 0], kk[:, 1]]
tall = (hgt >= 0.45) & (hgt <= 1.2) & (cnt >= 6)
cols = np.unique(kk[tall], axis=0)
walls = []
for c in cols:
    g = ground[c[0], c[1]]; x, y = (c + k0 + 0.5) * G
    walls += [(x, y, g + h) for h in (0.15, 0.35, 0.55)]   # 벽 한 기둥을 높이 세 점으로
walls = np.array(walls, np.float32)
# 로봇이 실제로 지나간 자리(반경 0.25 m)에는 고정 벽이 있을 수 없다 → 지운다(로봇을 따라 걷던 사람 흔적)
from scipy.spatial import cKDTree
dd, _ = cKDTree(path).query(walls[:, :2]); n0 = len(walls); walls = walls[dd > 0.25]
print('벽 기둥', len(cols), '· 지나간 자리 점 지움', n0 - len(walls), '· 벽 점', len(walls))
np.save(os.path.join(H, 'sim_walls.npy'), walls)
# 신호등 사진: 후보 검출 결과(detections.json)에서 D435i 사진 중 확실한 빨강만/초록만
C = os.path.expanduser('~/ws/datasets/arena1104/_candidates'); D = json.load(open(os.path.join(C, 'detections.json')))
pick = {'red': [], 'green': []}
for fn, bx in sorted(D.items()):
    if not fn.startswith('d435'): continue
    r = max([p for c, p, o in bx if c == 0] or [0]); g = max([p for c, p, o in bx if c == 1] or [0])
    if r >= 0.6 and g == 0: pick['red'].append((r, fn))
    if g >= 0.6 and r == 0: pick['green'].append((g, fn))
for k, v in pick.items():
    out = os.path.join(H, 'tl_images', k); shutil.rmtree(out, ignore_errors=True); os.makedirs(out)
    v.sort(reverse=True)
    for s, fn in v[:8]: shutil.copy(os.path.join(C, fn), out)
    print(k, '후보', len(v), '→', min(8, len(v)), '장', [f'{s:.2f}' for s, _ in v[:8]])
