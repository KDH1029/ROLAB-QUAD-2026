#!/usr/bin/env python3
"""FAST-LIO 가 지도에서 얼마나 밀리나 — 출발 때 한 번 맞춘 T 로 본 위치 vs 그 시각 3D ICP(매번 출발 T 에서 새로 시작, 이어가기 없음).
이어가기(tracking) ICP 는 슬라럼에서 그럴듯하게 틀린 답으로 미끄러져(r1650: 일치 0.7 인데 31 cm·7°, 실제 FAST-LIO 차이는 5 cm) 기준이 못 됨.
판정: 일치(10 cm 안) ≥ 0.5 이고, 1초·2초 두 창의 답이 3 cm·1° 안으로 같을 때만 '믿을 수 있는 기준'. 슬라럼 구역은 따로 셈.
사용: ~/venv/go2/bin/python3 drift_check.py out/r1645 [--map rec1650]"""
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import icp_core as ic  # noqa: E402

D = os.path.abspath(sys.argv[1]); NAME = sys.argv[sys.argv.index('--map') + 1] if '--map' in sys.argv else 'rec1650'
R = os.path.expanduser(f'~/ws/junk/routes/{NAME}')
dump = ic.Dump(os.path.join(D, 'dump.npz')); path = np.load(os.path.join(R, 'editor_cache.npz'))['path']
m = ic.Matcher(ic.load_map(os.path.join(R, 'map.npz'), path_xy=path))
route = json.load(open(os.path.join(R if os.path.exists(os.path.join(R, 'route.json')) else os.path.expanduser('~/ws/junk/routes/rec1650'), 'route.json')))['route']   # 경로 없는 지도는 rec1650 슬라럼 자리로(좌표 거의 같음)
SLX = [e for i, e in enumerate(route) if 6 <= i + 1 <= 25]
sx, sy = [e['x'] for e in SLX], [e['y'] for e in SLX]
in_slalom = lambda p: min(sx) - 0.4 <= p[0] <= max(sx) + 0.4 and min(sy) - 0.4 <= p[1] <= max(sy) + 0.4


def solve(t, T3, win):
    S = dump.scan_at(t, win=win); P = dump.pose_at(t)
    if len(S) < 200:
        return None
    M, rms, _ = m.align(S, ic.T3_to4(T3) @ P)
    return ic.T4_to3(M @ np.linalg.inv(P)), m.inlier(S, M)


def ap(T, x, y):
    c, s = math.cos(T[2]), math.sin(T[2]); return np.array([c * x - s * y + T[0], s * x + c * y + T[1]])


t0 = dump.ot[0]
T_st = solve(t0 + 3.0, np.zeros(3), 2.0)[0]
rows = []
for t in np.arange(t0 + 3.0, dump.ot[-1], 1.0):
    a, b = solve(t, T_st, 1.0), solve(t, T_st, 2.0)
    if a is None or b is None:
        continue
    P = dump.pose_at(t); x, y = P[0, 3], P[1, 3]
    agree = np.hypot(*(ap(a[0], x, y) - ap(b[0], x, y))) < 0.03 and abs(math.degrees((a[0][2] - b[0][2] + math.pi) % (2 * math.pi) - math.pi)) < 1.0
    e = float(np.hypot(*(ap(T_st, x, y) - ap(b[0], x, y)))); ea = abs(math.degrees((b[0][2] - T_st[2] + math.pi) % (2 * math.pi) - math.pi))
    pm = ap(T_st, x, y)
    rows.append((t - t0, e, ea, b[1], agree, in_slalom(pm), dump.height_at(t), pm[0], pm[1]))
A = np.array(rows, dtype=float)
good = (A[:, 3] >= 0.5) & (A[:, 4] > 0)
print(f'{os.path.basename(D)} → {NAME}: 출발 T ({T_st[0]:+.3f}, {T_st[1]:+.3f}, {math.degrees(T_st[2]):+.2f}°) · 시점 {len(A)}개, 믿을 수 있는 기준 {good.sum()}개'
      f' (슬라럼 안 {int((good & (A[:, 5] > 0)).sum())}/{int((A[:, 5] > 0).sum())}, 밖 {int((good & (A[:, 5] == 0)).sum())}/{int((A[:, 5] == 0).sum())})')
for lab, k in (('슬라럼 밖', good & (A[:, 5] == 0)), ('슬라럼 안', good & (A[:, 5] > 0))):
    if k.any():
        print(f'  {lab}: 위치 오차 중앙 {np.median(A[k, 1]) * 100:.1f} · p95 {np.percentile(A[k, 1], 95) * 100:.1f} · 최대 {A[k, 1].max() * 100:.1f} cm ({A[k][A[k, 1].argmax(), 0]:.0f}초)'
              f' · 각도 최대 {A[k, 2].max():.2f}°')
for q0 in range(0, int(A[-1, 0]) + 1, 30):
    k = good & (A[:, 0] >= q0) & (A[:, 0] < q0 + 30)
    if k.any():
        print(f'    {q0:3d}~{q0 + 30:3d}초: 최대 {A[k, 1].max() * 100:5.1f} cm · {A[k, 2].max():.2f}°  (기준 {k.sum()}개)')
k = np.where(good)[0]
print(f'  마지막 믿을 기준({A[k[-1], 0]:.0f}초): {A[k[-1], 1] * 100:.1f} cm · {A[k[-1], 2]:.2f}°')
bad = (~good)
print(f'  믿을 수 없는 시점 {bad.sum()}개: 일치 < 0.5 {int((A[:, 3] < 0.5).sum())} · 1초/2초 답 불일치 {int((A[:, 4] == 0).sum())} (그중 슬라럼 안 {int((bad & (A[:, 5] > 0)).sum())})')
np.save(os.path.join(D, 'drift.npy'), A)
