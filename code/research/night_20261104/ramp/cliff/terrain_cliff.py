#!/usr/bin/env python3
"""낭떠러지 가장자리 — 거부권(terrain_core.traversable)이 만든 통과면 T 의 가장자리에서 바깥으로 뚝 떨어지는 곳을 장애물로 되살린다.

왜: ego 규칙(rel_z ≥ −0.20 + 0.045·d)은 로봇보다 낮은 점을 장애물로 안 친다 → 단(0.34 m) 위에서는 옆면 낭떠러지를 못 본다.
    그동안은 경사로 헛장애물이 우연히 단 가장자리를 막아줬는데, 거부권이 그 헛장애물을 지우면서 가장자리가 열렸다(tv2 뚫림 최대 150칸).
규칙:
  가장자리 칸 = T 칸 중 8-이웃에 T 아닌 칸이 있는 것
  바깥 칸     = 가장자리 칸에서 중심거리 r_min(0.05)~reach(0.30) m 안, T 아닌 칸
  낭떠러지    = 바깥 칸 중 '데이터 있는' 칸의 최고 높이가 가장자리 칸 최고 높이보다 drop_h(0.22) 넘게 낮은 칸이 min_low 개 이상
                (계단 한 단 0.17~0.20 은 통과. 이어진 아래 계단은 T 라서 바깥 칸에서 빠진다)
  빈 가장자리 = 바깥 칸(T 아님)이 있는데 전부 데이터 없음 — 세기만 한다(성긴 XT16 이면 앞이 다 막힐 위험이라 장애물로 안 씀)
  장애물 점   = 낭떠러지 칸 중심, z = 로봇 z + dz(0.30) — ego 규칙이 장애물로 치는 높이
  (선택) band  = 가장자리 칸 높이가 발밑 지면 평면(몸체 z축 법선)에서 ±band 안일 때만 낭떠러지로 침.
                 경사로를 오를 때 L1 이 표면보다 0.2~0.3 m 높게 찍은 점이 T 에 붙어 '가짜 가장자리'가 되던 것(10-30 77~85 s) 차단.
                 바닥에서 본 단 윗면 가장자리(발보다 0.34 위)는 빠지지만, 바닥에서는 단 옆면이 ego 규칙상 이미 벽이라 손해 없음.
"""
import numpy as np
from scipy import ndimage


def ring_offsets(cell, r_min=0.05, reach=0.30):
    R = int(np.ceil(reach / cell + 1e-9))
    o = [(i, j) for i in range(-R, R + 1) for j in range(-R, R + 1)
         if r_min - 1e-9 <= cell * np.hypot(i, j) <= reach + 1e-9]
    return np.array(o, dtype=np.int64).reshape(-1, 2)


def cliff_edges(hi, T, cell, drop_h=0.22, reach=0.30, r_min=0.05, min_low=1, band=None, plane=None, origin=None):
    """hi: 칸 최고 높이(NaN=빈칸), T: 통과면 마스크 → (낭떠러지, 가장자리, 빈 가장자리) 마스크.
    band(m) 와 plane=((fx,fy,fz),(nx,ny,nz)), origin 을 주면 가장자리 높이가 발밑 평면 ±band 안인 것만 낭떠러지."""
    T = T.astype(bool)
    valid = np.isfinite(hi)
    interior = ndimage.binary_erosion(T, structure=np.ones((3, 3), bool), border_value=1)
    edge = T & ~interior
    offs = ring_offsets(cell, r_min, reach)
    n0, n1 = hi.shape
    if len(offs) == 0 or not edge.any():
        z = np.zeros_like(edge); return z, edge, z
    R = int(np.abs(offs).max())
    outer = valid & ~T
    H = np.pad(np.where(outer, hi, np.inf), R, constant_values=np.inf)
    V = np.pad(outer, R, constant_values=False)
    NT = np.pad(~T, R, constant_values=False)                   # 격자 안의 T 아닌 칸
    thr = np.where(edge, np.where(valid, hi, np.nan) - drop_h, -np.inf)
    nlow = np.zeros((n0, n1), np.int32); nval = np.zeros((n0, n1), np.int32); nnon = np.zeros((n0, n1), np.int32)
    for i, j in offs:
        sl = (slice(R + i, R + i + n0), slice(R + j, R + j + n1))
        nlow += H[sl] < thr; nval += V[sl]; nnon += NT[sl]
    cliff = edge & (nlow >= min_low)
    if band is not None and plane is not None:
        (fx, fy, fz), (nx, ny, nz) = plane; nz = max(nz, 0.5)
        ii, jj = np.meshgrid(np.arange(n0), np.arange(n1), indexing='ij')
        X = origin[0] + (ii + 0.5) * cell; Y = origin[1] + (jj + 0.5) * cell
        zp = fz - (nx * (X - fx) + ny * (Y - fy)) / nz
        with np.errstate(invalid='ignore'):
            cliff &= np.abs(hi - zp) <= band
    empty = edge & (nval == 0) & (nnon > 0)
    return cliff, edge, empty


def cliff_points(cliff, origin, cell, robot_z, dz=0.30):
    ij = np.argwhere(cliff)
    xy = np.asarray(origin) + (ij + 0.5) * cell
    return np.c_[xy, np.full(len(ij), robot_z + dz)]
