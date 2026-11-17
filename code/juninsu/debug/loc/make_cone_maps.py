#!/usr/bin/env python3
"""콘이 바뀐 지도 만들기 — 슬라럼 구역(#6~#25) 안에서 바깥 사각 벽(좌·우·위·아래)을 뺀 나머지 = 콘으로 보고
  A(nocone): 콘을 지움            → 실제로는 지도에 없는 콘이 보이는 상황
  B(moved) : 콘 덩어리마다 0.3~0.6 m 무작위로 옮김(구역 안에서) → 지도·실제 둘 다 콘이 있는데 자리가 다름
결과: out/maps/<A|B>/map.npz(3D, 같은 형식) · map.pgm/map.yaml(2D, AMCL 용) · cones.png
사용: ~/venv/go2/bin/python3 make_cone_maps.py"""
import json
import os
import shutil

import numpy as np
from scipy import ndimage as ndi

R = os.path.expanduser('~/ws/junk/routes/rec1650'); OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'out', 'maps')
d = dict(np.load(os.path.join(R, 'map.npz'))); V = float(d['V'])
route = json.load(open(os.path.join(R, 'route.json')))['route']; SL = np.array([[e['x'], e['y']] for e in route[5:25]])
P = (d['vox'] + 0.5) * V
# 구역: 바깥 사각 벽 안쪽. y 0.25 m 칸마다 왼벽 = x∈[5.8,6.45] 의 하위 3%, 오른벽 = x∈[6.85,7.45] 의 상위 3%
zone = (P[:, 0] > 5.7) & (P[:, 0] < 7.5) & (P[:, 1] > -1.35) & (P[:, 1] < 3.6)
Z = P[zone & (d['cnt'] >= 2)]
ybins = np.arange(-1.35, 3.6, 0.25); left, right = {}, {}
for y0 in ybins:
    k = (Z[:, 1] >= y0) & (Z[:, 1] < y0 + 0.25)
    l = Z[k & (Z[:, 0] > 5.8) & (Z[:, 0] < 6.45), 0]; r = Z[k & (Z[:, 0] > 6.85) & (Z[:, 0] < 7.45), 0]
    left[y0] = np.percentile(l, 3) if len(l) > 20 else 6.0; right[y0] = np.percentile(r, 97) if len(r) > 20 else 7.2
yb = np.floor((P[:, 1] + 1.35) / 0.25) * 0.25 - 1.35
lx = np.array([left.get(round(v, 2), 6.0) for v in yb]); rx = np.array([right.get(round(v, 2), 7.2) for v in yb])
WALL_M = 0.12
_k = np.floor(P[:, :2] / 0.5).astype(int); _u, _inv = np.unique(_k, axis=0, return_inverse=True); _inv = _inv.ravel()
_fl = np.full(len(_u), np.inf); np.minimum.at(_fl, _inv, P[:, 2]); HP = P[:, 2] - _fl[_inv]      # 바닥 위 높이
import sys as _s; _s.argv = ['x', 'rec1650']; _s.path.insert(0, os.path.expanduser('~/ws/junk/tools')); import route_editor as _ed
VW = _ed.real_walls(_ed.load_cache())                                  # 세로 0.25 m 넘는 4 cm 칸(평평한 판·바닥 제외, 로봇 몸 점 제외)
_vk = set(map(tuple, np.floor(VW / 0.04).astype(int).tolist()))
_pk = np.floor(P[:, :2] / 0.04).astype(int)
vert = np.array([any((i + di, j + dj) in _vk for di in (-1, 0, 1) for dj in (-1, 0, 1)) for i, j in _pk]) if False else None
_ks = np.array(sorted(_vk)); _grid = {}
for di in (-1, 0, 1):
    for dj in (-1, 0, 1):
        for i, j in _ks: _grid[(i + di, j + dj)] = True
vert = np.fromiter((((i, j) in _grid) for i, j in _pk), bool, len(_pk))
interior = vert & (HP > 0.08) & zone & (P[:, 0] > lx + WALL_M) & (P[:, 0] < rx - WALL_M) & (P[:, 1] > -1.10) & (P[:, 1] < 3.30)
print(f'슬라럼 구역 콘(바깥 벽에서 {WALL_M} m 안쪽) 복셀 {interior.sum()} / 전체 {len(P)}')
# 콘 덩어리(바닥에서 본 xy, 5 cm 칸, 1칸 이음)
C = P[interior]; ij = np.floor((C[:, :2] - [5.7, -1.35]) / 0.04).astype(int); G = np.zeros(ij.max(0) + 2, bool); G[ij[:, 0], ij[:, 1]] = True
lab, n = ndi.label(G, structure=np.ones((3, 3))); cl = lab[ij[:, 0], ij[:, 1]]
sizes = np.bincount(cl)
print(f'  덩어리 {n}개 (복셀 50개 넘는 것 {np.sum(sizes[1:] > 50)}개)')
rng = np.random.default_rng(7)
shift = {}
for k in range(1, n + 1):
    a = rng.uniform(0, 2 * np.pi); m = rng.uniform(0.3, 0.6); shift[k] = np.array([m * np.cos(a), m * np.sin(a), 0.0])
Pm = P.copy(); idx = np.where(interior)[0]
for k in range(1, n + 1):
    sel = idx[cl == k]; s = shift[k].copy()
    c = P[sel, :2].mean(0) + s[:2]                                     # 구역 밖으로 나가면 반대로
    if not (6.1 < c[0] < 7.1 and -1.0 < c[1] < 3.2):
        s = -s
    Pm[sel] += s


def save(name, keep, pts):
    o = os.path.join(OUT, name); os.makedirs(o, exist_ok=True)
    vox = np.floor(pts[keep] / V).astype(np.int32)
    np.savez_compressed(os.path.join(o, 'map.npz'), vox=vox, cnt=d['cnt'][keep], V=d['V'], odom=d['odom'], lid=d['lid'])
    # 2D: 원본 pgm 에서 구역 콘 칸을 비우고(A) / 옮긴 콘 칸을 채움(B)
    meta = dict(l.split(': ', 1) for l in open(os.path.join(R, 'map.yaml')).read().splitlines() if ': ' in l)
    res = float(meta['resolution']); ox, oy = [float(v) for v in meta['origin'].strip('[]').split(',')[:2]]
    raw = open(os.path.join(R, 'map.pgm'), 'rb').read().split(b'\n', 3); w, h = map(int, raw[1].split())
    img = np.frombuffer(raw[3][:w * h], np.uint8).reshape(h, w).copy()
    def cells(Q):
        c_ = ((Q[:, 0] - ox) / res).astype(int); r_ = (h - 1 - (Q[:, 1] - oy) / res).astype(int); ok = (c_ >= 0) & (c_ < w) & (r_ >= 0) & (r_ < h); return r_[ok], c_[ok]
    # 원래 콘 자리(구역 안, 바깥 벽 제외) 칸을 비움
    yy, xx = np.mgrid[0:h, 0:w]; X = ox + (xx + 0.5) * res; Y = oy + (h - 1 - yy + 0.5) * res
    ybc = np.floor((Y + 1.35) / 0.25) * 0.25 - 1.35
    inz = (X > 5.7) & (X < 7.5) & (Y > -1.10) & (Y < 3.30)
    lxc = np.vectorize(lambda v: left.get(round(v, 2), 6.0))(np.where(inz, ybc, 0)); rxc = np.vectorize(lambda v: right.get(round(v, 2), 7.2))(np.where(inz, ybc, 0))
    clear = inz & (X > lxc + WALL_M) & (X < rxc - WALL_M) & (img < 100)
    img[clear] = 254
    if name == 'B':                                                     # 옮긴 콘: 바닥 위 0.25 m 넘는 점이 있는 칸을 채움
        key = np.floor(pts[:, :2] / 0.5).astype(int); u, inv = np.unique(key, axis=0, return_inverse=True); inv = inv.ravel()
        fl = np.full(len(u), np.inf); np.minimum.at(fl, inv, pts[:, 2]); hh = pts[:, 2] - fl[inv]
        mv = interior & (hh > 0.25) & (d['cnt'] >= 2); r_, c_ = cells(pts[mv]); img[r_, c_] = 0
    open(os.path.join(o, 'map.pgm'), 'wb').write(b'P5\n%d %d\n255\n' % (w, h) + img.tobytes())
    shutil.copy(os.path.join(R, 'map.yaml'), os.path.join(o, 'map.yaml'))
    shutil.copy(os.path.join(R, 'editor_cache.npz'), os.path.join(o, 'editor_cache.npz'))
    shutil.copy(os.path.join(R, 'route.json'), os.path.join(o, 'route.json'))
    print(f'  {name}: 3D 복셀 {keep.sum()} · 2D 비운 칸 {clear.sum()}' + (f' · 채운 칸 {len(r_)}' if name == 'B' else '') + f' → {o}')


save('A', ~interior, P)
save('B', np.ones(len(P), bool), Pm)
try:
    import matplotlib; matplotlib.use('Agg'); matplotlib.rcParams['font.family'] = 'NanumGothic'
    import matplotlib.pyplot as plt
    f, ax = plt.subplots(1, 3, figsize=(15, 9)); lo = (P[:, 0] > 5.4) & (P[:, 0] < 7.8) & (P[:, 1] > -1.5) & (P[:, 1] < 3.8) & (d['cnt'] >= 2)
    for a, (t, Q, k) in zip(ax, [('원래', P, lo & ~interior), ('A 콘 지움', P, lo & ~interior), ('B 콘 옮김', Pm, lo)]):
        a.scatter(Q[k, 0], Q[k, 1], s=0.2, c='k')
        if t == '원래':
            a.scatter(P[lo & interior, 0], P[lo & interior, 1], s=0.2, c='orange')
        if t.startswith('B'):
            a.scatter(Pm[lo & interior, 0], Pm[lo & interior, 1], s=0.2, c='orange')
        a.plot(SL[:, 0], SL[:, 1], 'r.-', lw=0.6); a.set_xlim(5.4, 7.8); a.set_ylim(-1.5, 3.8); a.set_aspect('equal'); a.set_title(t + ' (주황 = 콘으로 본 것)')
    f.savefig(os.path.join(OUT, 'cones.png'), dpi=60, bbox_inches='tight')
except Exception as e:  # noqa: BLE001
    print('그림 실패', e)
