#!/usr/bin/env python3
"""v2 시뮬 세계 재료(rec1650 = 새 코스 16:50 지도) — 
  sim_walls_rec1650.npy : 벽 칸(편집기 real_walls, 4 cm, 콘 포함) Nx3 (z=0.3) — 가상 라이다·벽 구름 재료
  sim_obc_rec1650.npy   : 몸 접촉 판정용 장애물(외톨이 잡음 뺌) Nx2 — 사람 주행 보정(δ 10~12 cm)과 같은 판정
  sim_cones_rec1650.npy : 콘 중심(슬라럼 구역에서 콘으로 본 칸 = rec1650 에는 있고 콘 지운 지도 A 에는 없는 칸, 0.3 m 격자로 묶음) Mx2
사용: ~/venv/go2/bin/python3 make_sim_world_rec1650.py"""
import importlib, os, sys
import numpy as np
from scipy.spatial import cKDTree
H = os.path.dirname(os.path.abspath(__file__)); JK = os.path.expanduser('~/ws/junk')
sys.path.insert(0, os.path.join(JK, 'tools')); sys.argv = ['x', 'rec1650']; import route_editor as ed  # noqa: E402  (읽기만: rec1650 캐시)
W = ed.real_walls(ed.load_cache())
nb = np.array([len(x) - 1 for x in cKDTree(W).query_ball_point(W, 0.12)]); OBc = W[nb >= 2]
np.save(os.path.join(H, 'sim_walls_rec1650.npy'), np.c_[W, np.full(len(W), 0.3)].astype(np.float32)); np.save(os.path.join(H, 'sim_obc_rec1650.npy'), OBc.astype(np.float32))
# ★ 11-06 xentry: 가상 라이다도 외톨이 칸을 뺀 벽으로(몸 접촉 판정 OBc 와 같은 거르기). 지도 잡음 한 칸(1.98,0.10 — 바닥 위 0.46 m 복셀 1개)이
#   X 통로 입구를 ego 팽창 격자에서 막았다. 실제 라이다엔 안 보이는 칸이다. run_sim_v2.sh 기본 WALLS.
np.save(os.path.join(H, 'sim_walls_rec1650_noiso.npy'), np.c_[OBc, np.full(len(OBc), 0.3)].astype(np.float32))
# 콘 칸: rec1650 3D 지도와 콘 지운 지도 A 의 차이(바닥 위 0.10~0.65 m)
def cells(p):
    d = np.load(p); P = (d['vox'] + 0.5) * float(d['V']); k = np.floor(P[:, :2] / 0.5).astype(int)
    u, inv = np.unique(k, axis=0, return_inverse=True); inv = inv.ravel(); fl = np.full(len(u), np.inf); np.minimum.at(fl, inv, P[:, 2]); h = P[:, 2] - fl[inv]
    Q = P[(h > 0.10) & (h < 0.65) & (d['cnt'] >= 2), :2]; return set(map(tuple, np.floor(Q / 0.04).astype(int).tolist()))
c0 = cells(os.path.join(JK, 'routes/rec1650/map.npz')); cA = cells(os.path.join(JK, 'debug/loc/out/maps/A/map.npz'))
C = (np.array(sorted(c0 - cA)) + 0.5) * 0.04
k = np.floor(C / 0.3).astype(int); u, inv = np.unique(k, axis=0, return_inverse=True); inv = inv.ravel()
cen = np.array([C[inv == i].mean(0) for i in range(len(u)) if (inv == i).sum() >= 6])
np.save(os.path.join(H, 'sim_cones_rec1650.npy'), cen.astype(np.float32))
print(f'벽 칸 {len(W)} · 접촉 판정 칸 {len(OBc)} · 콘 칸 {len(C)} → 콘 중심 {len(cen)}개 (x {cen[:, 0].min():.2f}~{cen[:, 0].max():.2f}, y {cen[:, 1].min():.2f}~{cen[:, 1].max():.2f})')
