#!/usr/bin/env python3
"""입구가 ego 팽창 격자에서 열릴 확률 — 격자 원점(로봇 위치에 따라 0~5 cm 어긋남)·스캔 잡음(σ1 cm)을 무작위로 300번."""
import sys, os, numpy as np
sys.path.insert(0, os.path.dirname(__file__)); from astar_replica import build, entrance_open, GHOST
S = os.path.expanduser('~/ws/junsu_v2/sim'); rng = np.random.default_rng(0)
def load(f):
    W = np.load(f)[:, :2].astype(float); return W[(W[:, 0] > 0.8) & (W[:, 0] < 3.2) & (W[:, 1] > -0.8) & (W[:, 1] < 1.4)]
for name, f in (('기본 벽(시뮬)', S + '/sim_walls_rec1650.npy'), ('얇은 벽(-4 cm, 실제에 가까움)', S + '/sim_walls_rec1650_thin.npy')):
    W = load(f); g = np.hypot(*(W - GHOST).T) < 0.03
    for lab, P in (('외톨이 칸 있음', W), ('외톨이 칸 없음', W[~g])):
        for infl in (0.20, 0.15):
            ok = 0
            for _ in range(300):
                robot = np.array([1.4, -0.1]) + rng.uniform(0, 0.05, 2)
                org, occ, inf = build(robot, P + rng.normal(0, 0.01, P.shape), infl)
                ok += entrance_open(org, inf)
            print(f'{name:22s} {lab:8s} 팽창 {infl:.2f}: 입구 열림 {ok / 3:.0f}%')
