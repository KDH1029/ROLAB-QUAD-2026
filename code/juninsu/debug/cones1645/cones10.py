#!/usr/bin/env python3
"""콘 10개(rec1650 콘 차이 지도로 찾은 것, ../slalom/cone_centres_rec1650.npy)를 씨앗으로, 두 지도 각각에서 중심을 다시 맞춘다(rec1650 좌표).
중심 = 씨앗 0.22 m 안, 바닥 위 0.05~0.45 m 점으로 원 맞추기(Kasa) 3회(움직임 > 0.12 m 면 멈춤). 반경·관측 방향 범위(°)도 기록.
검증 = 각 녹화의 리모컨 주행(콘에 안 닿음): 네 발 → 콘 중심 최소 거리. 콘 밑동 반지름보다 가까우면 콘 좌표가 틀린 것."""
import json, math, os, sys
import numpy as np
from scipy.spatial import cKDTree
H = os.path.dirname(os.path.abspath(__file__)); sys.argv = ['x']; exec(open(os.path.join(H, 'cones.py')).read().split('res = {}')[0])
S = np.load(os.path.join(J, 'debug', 'slalom', 'cone_centres_rec1650.npy'))
out = {}
for name, to in (('rec1650', False), ('rec1645', True)):
    Q, tr = load(name, to); TQ = cKDTree(Q); cs = []
    for s in S:
        c = s.copy()
        for _ in range(3):
            p = Q[TQ.query_ball_point(c, 0.22)]
            if len(p) < 20: break
            c2, r = circle(p)
            if np.hypot(*(c2 - s)) > 0.12 or not (0.06 < r < 0.30): break
            c = c2
        p = Q[TQ.query_ball_point(c, 0.22)]; ang = np.degrees(np.arctan2(*(p - c).T[::-1])) if len(p) else np.zeros(1)
        h = np.histogram(ang, bins=12, range=(-180, 180))[0]
        cs.append(dict(c=c, n=len(p), r=float(np.median(np.hypot(*(p - c).T))) if len(p) else 0, cover=int((h > 3).sum() * 30)))
    out[name] = dict(cones=cs, tr=tr)
print(' #   rec1650 (x, y)    반경  보인각  |  rec1645 (x, y)    반경  보인각  |  차이')
A = np.array([c['c'] for c in out['rec1650']['cones']]); B = np.array([c['c'] for c in out['rec1645']['cones']])
for i in range(len(S)):
    a, b = out['rec1650']['cones'][i], out['rec1645']['cones'][i]
    print(f'c{i}  ({A[i, 0]:5.2f},{A[i, 1]:5.2f}) {a["r"] * 100:4.1f}cm {a["cover"]:4d}°  |  ({B[i, 0]:5.2f},{B[i, 1]:5.2f}) {b["r"] * 100:4.1f}cm {b["cover"]:4d}°  |  {np.hypot(*(A[i] - B[i])) * 100:5.1f} cm')
dd = np.hypot(*(A - B).T); print(f'두 지도 콘 위치 차이: 중앙 {np.median(dd) * 100:.1f} cm · 최대 {dd.max() * 100:.1f} cm')
FE = np.array([(0.19, 0.14), (0.19, -0.14), (-0.19, 0.14), (-0.19, -0.14)])
def clr(tr, C):
    tr = tr[np.all((tr[:, :2] > ZLO) & (tr[:, :2] < ZHI), 1)]; TC = cKDTree(C)
    F = np.concatenate([tr[i, :2] + FE @ np.array([[math.cos(tr[i, 2]), math.sin(tr[i, 2])], [-math.sin(tr[i, 2]), math.cos(tr[i, 2])]]) for i in range(len(tr))])
    b = TC.query(tr[:, :2])[0]; f, j = TC.query(F); per = [f[j == k].min() if (j == k).any() else np.nan for k in range(len(C))]
    return b.min(), f.min(), float(np.mean(f < 0.15)), per
print('\n검증: 리모컨 주행 → 콘 중심 (몸 최소 / 발 최소 / 발 15 cm 안 비율) · 콘별 발 최소(cm)')
for drive in ('rec1645', 'rec1650'):
    for cn, C in (('1645 콘', B), ('1650 콘', A)):
        b, f, fr, per = clr(out[drive]['tr'], C)
        print(f'  {drive[3:]} 주행 × {cn}: 몸 {b * 100:5.1f} · 발 {f * 100:5.1f} · 15cm 안 {fr * 100:5.2f}% | ' + ' '.join('-' if np.isnan(x) else f'{x * 100:.0f}' for x in per))
np.save(os.path.join(H, 'cone_centres_1645_in1650.npy'), B); np.save(os.path.join(H, 'cone_centres_1650_refit.npy'), A)
json.dump({k: [dict(x=float(c['c'][0]), y=float(c['c'][1]), r=c['r'], cover=c['cover'], n=c['n']) for c in v['cones']] for k, v in out.items()}, open(os.path.join(H, 'cones10.json'), 'w'), indent=1)
