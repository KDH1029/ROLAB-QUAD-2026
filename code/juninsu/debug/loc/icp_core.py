#!/usr/bin/env python3
"""3D ICP(x, y, yaw) 핵심 — ~/ws/lab/tools/icp3d.py 의 Matcher 를 복사(lab 은 불가침, 읽기만)해 오늘 지도용으로 묶음.
지도: junk routes/<이름>/map.npz (2 cm 복셀, 그 녹화의 camera_init). 바닥은 0.5 m 칸별 바닥 높이로 빼고 벽·콘만 쓴다.
스캔: FAST-LIO /cloud_registered_body 를 최근 W 초 모아 '지금 몸' 기준으로 합친 것."""
import math

import numpy as np
from scipy.spatial import cKDTree


def voxel(pts, size):
    if size <= 0:
        return pts
    key = np.floor(pts / size).astype(np.int64)
    _, idx = np.unique(key, axis=0, return_index=True)
    return pts[np.sort(idx)]


def quat_R(qx, qy, qz, qw):
    return np.array([[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                     [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                     [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])


def pose4(row):
    """odom 행 (t, x, y, z, qx, qy, qz, qw) → 4x4 (world ← body)."""
    T = np.eye(4); T[:3, :3] = quat_R(*row[4:8]); T[:3, 3] = row[1:4]; return T


def T3_to4(T):
    c, s = math.cos(T[2]), math.sin(T[2]); M = np.eye(4); M[:2, :2] = [[c, -s], [s, c]]; M[:2, 3] = T[:2]; return M


def T4_to3(M):
    return np.array([M[0, 3], M[1, 3], math.atan2(M[1, 0], M[0, 0])])


def load_map(npz, path_xy=None, margin=3.0, floor_cut=0.10, top=1.8, cell=0.5):
    d = np.load(npz); P = ((d['vox'][d['cnt'] >= 2] + 0.5) * float(d['V'])).astype(np.float64)
    if path_xy is not None:
        lo, hi = path_xy.min(0) - margin, path_xy.max(0) + margin
        P = P[(P[:, 0] > lo[0]) & (P[:, 0] < hi[0]) & (P[:, 1] > lo[1]) & (P[:, 1] < hi[1])]
    key = np.floor(P[:, :2] / cell).astype(np.int64); u, inv = np.unique(key, axis=0, return_inverse=True); inv = inv.ravel()
    floor = np.full(len(u), np.inf); np.minimum.at(floor, inv, P[:, 2])                 # 칸별 가장 낮은 점 ≈ 바닥
    h = P[:, 2] - floor[inv]
    return P[(h > floor_cut) & (h < top)]


class Matcher:
    """lab icp3d.Matcher 와 같은 계산(거칠게→곱게, 하위 trim 만 사용, 평면 3자유도)."""
    STAGES = ((0.80, 5.0, 30), (0.40, 2.5, 25), (0.20, 1.0, 20), (0.10, 0.4, 15))

    def __init__(self, map_pts, stages=None, trim=0.7):
        self.levels = [(cKDTree(voxel(map_pts, vx)), voxel(map_pts, vx), mc, it) for vx, mc, it in (stages or self.STAGES)]
        self.trim = trim
        self.fine = cKDTree(voxel(map_pts, 0.05))

    def align(self, scan, T0):
        T = T0.copy(); total = 0; rms = float('inf')
        for tree, mp, mc, it in self.levels:
            T, rms, n = self._one(scan, T, tree, mp, mc, it); total += n
            if not np.isfinite(rms):
                break
        return T, rms, total

    def inlier(self, scan, T, r=0.10):
        Q = (T[:3, :3] @ scan.T).T + T[:3, 3]
        return float(np.mean(self.fine.query(Q, distance_upper_bound=r)[0] < r))

    def _one(self, P, T0, tree, mp, max_corr, iters):
        T = T0.copy(); last = None
        for it in range(iters):
            Q = (T[:3, :3] @ P.T).T + T[:3, 3]
            d, j = tree.query(Q, k=1, distance_upper_bound=max_corr)
            ok = np.isfinite(d)
            if ok.sum() < 50:
                return T, float('inf'), 0
            dd = d[ok]; thr = np.quantile(dd, self.trim)
            sel = ok.copy(); sel[ok] = dd <= thr
            if sel.sum() < 50:
                sel = ok
            A, B = Q[sel], mp[j[sel]]
            ca, cb = A[:, :2].mean(0), B[:, :2].mean(0); X, Y = A[:, :2] - ca, B[:, :2] - cb
            dyaw = math.atan2(float((X[:, 0] * Y[:, 1] - X[:, 1] * Y[:, 0]).sum()), float((X[:, 0] * Y[:, 0] + X[:, 1] * Y[:, 1]).sum()))
            c, s = math.cos(dyaw), math.sin(dyaw); R2 = np.array([[c, -s], [s, c]])
            dT = np.eye(4); dT[:2, :2] = R2; dT[:2, 3] = cb - R2 @ ca; T = dT @ T
            rms = float(np.sqrt((dd[dd <= thr] ** 2).mean()))
            if last is not None and abs(last - rms) < 1e-4:
                return T, rms, it + 1
            last = rms
        return T, (last if last is not None else float('inf')), iters


class Dump:
    """load_dump.py 결과에서 시각 t 의 '지금 몸 기준' 누적 스캔."""

    def __init__(self, npz):
        d = np.load(npz, allow_pickle=True)
        self.odom = d['odom']; self.ct = d['cloud_t']; self.ci = d['cloud_idx']; self.cx = d['cloud_xyz']
        self.h = d['height']; self.leg = d['leg']
        self.ot = self.odom[:, 0]

    def pose_at(self, t):
        k = int(np.clip(np.searchsorted(self.ot, t), 0, len(self.ot) - 1)); return pose4(self.odom[k])

    def scan_at(self, t, win=1.0, zmin=-0.20, zmax=1.5, max_pts=6000, rng=None):
        Pt_inv = np.linalg.inv(self.pose_at(t)); out = []
        for k in np.where((self.ct > t - win) & (self.ct <= t))[0]:
            Pk = self.pose_at(self.ct[k]); M = Pt_inv @ Pk
            a = self.cx[self.ci[k]:self.ci[k + 1]].astype(np.float64)
            out.append((M[:3, :3] @ a.T).T + M[:3, 3])
        if not out:
            return np.zeros((0, 3))
        S = np.concatenate(out); S = S[(S[:, 2] > zmin) & (S[:, 2] < zmax) & (np.hypot(S[:, 0], S[:, 1]) > 0.4)]
        S = voxel(S, 0.05)
        if len(S) > max_pts:
            S = S[(rng or np.random.default_rng(0)).choice(len(S), max_pts, replace=False)]
        return S

    def height_at(self, t):
        if len(self.h) == 0:
            return float('nan')
        k = int(np.clip(np.searchsorted(self.h[:, 0], t), 0, len(self.h) - 1)); return float(self.h[k, 1])
