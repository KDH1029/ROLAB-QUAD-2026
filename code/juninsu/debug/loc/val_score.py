#!/usr/bin/env python3
"""stop_loc_node 재생 검증 채점 — 노드가 낸 T(ok=true 만 반영)를 1초마다 적용해 참값(출발 T · 원래 FAST-LIO)과 비교.
사용: ~/venv/go2/bin/python3 val_score.py S0 S1 S2"""
import csv
import math
import os
import sys

import numpy as np

H = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, H)
from scen import Corrupt  # noqa: E402

T_ST = np.array([-0.025, -0.039, -0.0604])
O = np.load(os.path.join(H, 'out', 'r1645', 'dump.npz'))['odom']; t0 = O[0, 0]
yaw = lambda q: math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))
TRUE = np.array([(r[0], r[1], r[2], yaw(r[4:8])) for r in O])
true_at = lambda t: TRUE[int(np.clip(np.searchsorted(TRUE[:, 0], t), 0, len(TRUE) - 1)), 1:]


def comp(A, B):
    c, s = math.cos(A[2]), math.sin(A[2]); return np.array([c * B[0] - s * B[1] + A[0], s * B[0] + c * B[1] + A[1], A[2] + B[2]])


for sc in sys.argv[1:]:
    D = os.path.join(H, 'out', 'val', sc); rows = list(csv.reader(open(os.path.join(D, 'val.csv'))))[1:]
    Ts = [(float(r[1]), np.array([float(r[2]), float(r[3]), float(r[4])]), r[5] == '1') for r in rows if r[0] == 'T' and r[1] != 'None']
    cor = Corrupt(sc); ts = np.arange(t0 + 3, TRUE[-1, 0], 1.0); E = []
    for t in ts:
        okT = [T for (tt, T, ok) in Ts if ok and tt <= t]; T = okT[-1] if okT else T_ST
        x, y, a = true_at(t); Pc = np.array(cor.apply(t - t0, x, y, a)); est = comp(T, Pc); tr = comp(T_ST, true_at(t))
        E.append((t - t0, math.hypot(est[0] - tr[0], est[1] - tr[1]), abs(math.degrees((est[2] - tr[2] + math.pi) % (2 * math.pi) - math.pi))))
    E = np.array(E)
    print(f'\n[{sc}] 노드를 켠 경우 위치 오차: 중앙 {np.median(E[:, 1]) * 100:.1f} · p95 {np.percentile(E[:, 1], 95) * 100:.1f} · 최대 {E[:, 1].max() * 100:.1f} cm ({E[E[:, 1].argmax(), 0]:.0f}초) · 마지막 {E[-1, 1] * 100:.1f} cm · 각도 최대 {E[:, 2].max():.2f}°')
    for (tt, T, ok) in Ts:
        x, y, a = true_at(tt); Pc = np.array(cor.apply(tt - t0, x, y, a)); e = math.hypot(*(comp(T, Pc)[:2] - comp(T_ST, true_at(tt))[:2]))
        print(f'   {tt - t0:5.0f}초  {"반영" if ok else "안 고침(요청 답)"}  → 그 순간 오차 {e * 100:.1f} cm')
    for line in open(os.path.join(D, 'stop_loc.log')):
        if '안 고침' in line or '요청' in line:
            print('     ', line.strip()[:150])
