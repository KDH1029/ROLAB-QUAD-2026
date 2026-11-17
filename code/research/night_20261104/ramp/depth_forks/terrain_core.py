#!/usr/bin/env python3
"""지형 판정 핵심 — 경사로·계단은 통과, 벽·라바콘·단상 옆면은 장애물.

ego_planner(grid_map.cpp) 규칙: rel_z = 점 z − 로봇 z, 장애물 ⇔ −0.20 + 0.045·d ≤ rel_z ≤ 1.10 (d = 수평거리)
  → 땅이 기울면 1 m 앞 경사(> 약 8°)부터 벽. 로봇이 경사 위에 서 있어도 앞 경사면이 벽.
이 판정(지형 모양 기준, 로봇 높이와 무관):
  E = 칸(5 cm)별 표면 높이(칸 최고점, 로봇 z + max_h 위 점은 버림 — 천장)
  ① 단차   s = Ê − min(Ê, 반경 r_step), Ê = 3×3 중앙값(잡음 ±1 cm 가 min 에서 +2 cm 로 부풀던 것 제거)
           s > max_step(0.24 m) → 장애물   (벽 0.6, 단상 옆면 0.34)
  ② 솟음   p = E − q80(E, 고리 r_in~r_out)     p > peak_h(0.10 m)  → 장애물   (라바콘 0.23 — 주변 대부분이 바닥)
     계단 디딤판·경사로 칸은 뒤(오르막)쪽이 같거나 높아 q80 이 자기 높이 이상 → 솟음 아님
  계단 한 단(≤ 0.20)·경사로(칸 사이 1~2 cm)는 ①②에 안 걸린다. 라바콘(0.23)은 ① 경계라 ②가 잡는다.
"""
import numpy as np
from scipy import ndimage

EGO_MIN_H, EGO_MAX_H, EGO_SLOPE = -0.20, 1.10, 0.045


def ego_rule(W, robot_xyz, robot_yaw):
    """grid_map.cpp updateCloud 그대로 (본체 자기 필터 포함). W: 세계 좌표 N×3 → 장애물 점 마스크."""
    dx, dy = W[:, 0] - robot_xyz[0], W[:, 1] - robot_xyz[1]; d = np.hypot(dx, dy)
    rel = W[:, 2] - robot_xyz[2]; c, s = np.cos(robot_yaw), np.sin(robot_yaw)
    bx, by = c * dx + s * dy, -s * dx + c * dy
    selfm = (bx >= -0.35) & (bx <= 0.33) & (np.abs(by) <= 0.22) & (rel < 0.20)
    return (d >= 0.25) & (d <= 10.0) & ~selfm & (rel >= EGO_MIN_H + EGO_SLOPE * d) & (rel <= EGO_MAX_H)


def elevation(W, origin, shape, cell, zmax):
    """칸 최고점 높이 (zmax 위 점 버림). NaN = 빈 칸."""
    W = W[W[:, 2] <= zmax]
    k = np.floor((W[:, :2] - origin) / cell).astype(np.int64)
    ok = (k[:, 0] >= 0) & (k[:, 1] >= 0) & (k[:, 0] < shape[0]) & (k[:, 1] < shape[1]); k, z = k[ok], W[ok, 2]
    E = np.full(shape, -np.inf); np.maximum.at(E, (k[:, 0], k[:, 1]), z); E[np.isinf(E)] = np.nan
    return E


def _nanmed3(E):
    # 3×3 nan 중앙값 (빠른 판: 9장 쌓아 nanmedian)
    P = np.pad(E, 1, constant_values=np.nan)
    st = np.stack([P[i:i + E.shape[0], j:j + E.shape[1]] for i in range(3) for j in range(3)])
    with np.errstate(all='ignore'):
        import warnings; warnings.simplefilter('ignore')
        return np.nanmedian(st, axis=0)


def _disk(r_cells):
    y, x = np.mgrid[-r_cells:r_cells + 1, -r_cells:r_cells + 1]; return (x * x + y * y) <= r_cells * r_cells


def classify(E, cell, max_step=0.24, r_step=0.10, peak_h=0.10, r_in=0.15, r_out=0.30, min_ring=6):
    """E: 높이 격자(NaN 빈칸) → (장애물 마스크, 단차 s, 솟음 p)."""
    valid = np.isfinite(E)
    with np.errstate(all='ignore'):
        Es = _nanmed3(E)
    big = np.where(valid & np.isfinite(Es), Es, np.inf)
    lo = ndimage.minimum_filter(big, footprint=_disk(max(1, int(round(r_step / cell)))), mode='constant', cval=np.inf)
    s = np.where(valid & np.isfinite(lo) & np.isfinite(Es), Es - lo, 0.0)
    # 고리 q80 — 유효 칸만. generic_filter 는 느려서 고리 칸을 직접 모아 분위수
    ri, ro = int(round(r_in / cell)), int(round(r_out / cell))
    yy, xx = np.mgrid[-ro:ro + 1, -ro:ro + 1]; rr = np.hypot(xx, yy); offs = np.argwhere((rr >= ri) & (rr <= ro)) - ro
    p = np.zeros_like(s)
    idx = np.argwhere(valid)
    if len(idx):
        Ep = np.pad(np.where(valid, E, np.nan), ro, constant_values=np.nan)
        ring = Ep[idx[:, 0][:, None] + ro + offs[:, 0][None], idx[:, 1][:, None] + ro + offs[:, 1][None]]   # M × K
        cnt = np.isfinite(ring).sum(1)
        with np.errstate(all='ignore'):
            q = np.nanquantile(ring, 0.8, axis=1)
        pk = np.where(cnt >= min_ring, E[idx[:, 0], idx[:, 1]] - q, 0.0)
        p[idx[:, 0], idx[:, 1]] = np.nan_to_num(pk)
    obs = valid & ((s > max_step) | (p > peak_h))
    return obs, s, p


# ── 거부권(veto) 방식 — ego 규칙의 장애물은 그대로 두고, '로봇 발밑과 이어진 매끈한 면' 위의 점만 지운다 ──────────
# 11-04 실데이터: 지형 판정 단독은 헛장애물 0.6% 지만 벽을 절반 놓침(XT16 이 성겨 벽 옆 바닥이 비면 단차를 못 잰다).
# → 장애물을 '찾는' 대신 '통과 가능한 면을 찾아 그 위 점만 지운다'. 못 본 곳·애매한 곳은 ego 규칙 그대로(보수적).
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components


def grid_stats(W, origin, shape, cell, zmax):
    """칸별 최고·최저·점수 (zmax 위 점 버림)."""
    W = W[W[:, 2] <= zmax]
    k = np.floor((W[:, :2] - origin) / cell).astype(np.int64)
    ok = (k[:, 0] >= 0) & (k[:, 1] >= 0) & (k[:, 0] < shape[0]) & (k[:, 1] < shape[1]); k, z = k[ok], W[ok, 2]
    hi = np.full(shape, -np.inf); lo = np.full(shape, np.inf); n = np.zeros(shape, np.int32)
    np.maximum.at(hi, (k[:, 0], k[:, 1]), z); np.minimum.at(lo, (k[:, 0], k[:, 1]), z); np.add.at(n, (k[:, 0], k[:, 1]), 1)
    hi[n == 0] = np.nan; lo[n == 0] = np.nan
    return hi, lo, n


def traversable(hi, lo, cell, seed_xy, origin, robot_z, max_step=0.20, max_spread=0.20, hop=0.15, seed_r=0.8,
                below_body=0.15, peak_h=0.10, plane=None, seed_tol=0.08, seed_slope=0.05, dil=1, p=None, return_bad=False):
    """로봇 발밑에서 시작해 이어진 통과 가능 칸. 칸끼리는 hop 안, 높이차 ≤ max_step 이면 연결.
    후보 칸: 칸 안 높이 폭(hi−lo) ≤ max_spread(벽처럼 세로로 쌓인 칸 제외), 솟음(라바콘) 아님."""
    valid = np.isfinite(hi)
    if p is None:
        _, _, p = classify(hi, cell)
    bad0 = valid & (((hi - lo) > max_spread) | (p > peak_h))
    bad = ndimage.binary_dilation(bad0, structure=_disk(dil)) if dil > 0 else bad0   # 솟음·세로 칸 옆도 통과 후보에서 뺀다(라바콘 밑동)
    cand = valid & ~bad
    ij = np.argwhere(cand)
    T = np.zeros_like(cand)
    if len(ij) < 3: return (T, bad0) if return_bad else T
    xy = origin + (ij + 0.5) * cell; z = hi[ij[:, 0], ij[:, 1]]
    pr = cKDTree(xy).query_pairs(hop, output_type='ndarray')
    if len(pr) == 0: return (T, bad0) if return_bad else T
    keep = np.abs(z[pr[:, 0]] - z[pr[:, 1]]) <= max_step; pr = pr[keep]
    g = coo_matrix((np.ones(len(pr)), (pr[:, 0], pr[:, 1])), shape=(len(ij), len(ij)))
    _, lab = connected_components(g, directed=False)
    dd = np.hypot(*(xy - seed_xy).T)
    if plane is None:
        seeds = np.where((dd <= seed_r) & (z < robot_z - below_body))[0]
    else:   # 발밑 지면 평면(몸체 z축이 법선, 발 = 로봇 − h_stand·법선) 근처 칸을 씨앗으로 — XT16 은 1.75 m 안 바닥을 못 본다
        (fx, fy, fz), (nx, ny, nz) = plane
        zp = fz - (nx * (xy[:, 0] - fx) + ny * (xy[:, 1] - fy)) / nz
        seeds = np.where((dd <= seed_r) & (np.abs(z - zp) <= seed_tol + seed_slope * dd))[0]
    if len(seeds) == 0: return (T, bad0) if return_bad else T
    good = np.isin(lab, np.unique(lab[seeds]))
    T[ij[good, 0], ij[good, 1]] = True
    return (T, bad0) if return_bad else T


def veto(points, T, origin, cell):
    """points 중 통과 가능 칸 위에 있는 것 → True (지울 것)."""
    k = np.floor((points[:, :2] - origin) / cell).astype(np.int64)
    ok = (k[:, 0] >= 0) & (k[:, 1] >= 0) & (k[:, 0] < T.shape[0]) & (k[:, 1] < T.shape[1])
    out = np.zeros(len(points), bool); out[ok] = T[k[ok, 0], k[ok, 1]]
    return out
