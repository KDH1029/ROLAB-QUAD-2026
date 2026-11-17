#!/usr/bin/env python3
"""small_gicp(1.0.1, MIT) 기반 정합기 — backup_core.Matcher 와 같은 인터페이스 align(P, T0) -> (T, rms).

- 지도: 한 번만 전처리(0.10 m 다운샘플, 이웃 10개로 공분산) + KdTree
- 스캔: 수평 세운 몸체 좌표 점(0.15 m 다운샘플) → GICP 6자유도, 초기값 T0
- rms: 결과로 옮긴 스캔 점과 지도 사이 최근접 거리의 하위 70% RMS (기존 게이트 0.08 m 를 그대로 쓰기 위해 같은 정의)
"""
import math
import numpy as np
import small_gicp
from scipy.spatial import cKDTree


class GicpMatcher:
    solve_z = False

    def __init__(self, mp, map_res=0.10, scan_res=0.15, max_corr=1.0, threads=4, iters=30, trim=0.7, need_converged=True):
        self.need_converged = need_converged    # False: 수렴 표시가 없어도 잔차(rms)로만 판정
        self.n_noconv = 0
        self.target, self.tree = small_gicp.preprocess_points(np.ascontiguousarray(mp, dtype=np.float64), map_res, 10, threads)
        self.kd = cKDTree(np.asarray(self.target.points())[:, :3])
        self.scan_res, self.max_corr, self.threads, self.iters, self.trim = scan_res, max_corr, threads, iters, trim

    def align(self, P, T0):
        src, _ = small_gicp.preprocess_points(np.ascontiguousarray(P, dtype=np.float64), self.scan_res, 10, self.threads)
        r = small_gicp.align(self.target, src, self.tree, init_T_target_source=T0, registration_type='GICP',
                             max_correspondence_distance=self.max_corr, num_threads=self.threads, max_iterations=self.iters)
        T = np.asarray(r.T_target_source)
        if not r.converged:
            self.n_noconv += 1
            if self.need_converged:
                return T, np.inf
        Q = (T[:3, :3] @ P.T).T + T[:3, 3]
        d, _ = self.kd.query(Q, k=1, distance_upper_bound=self.max_corr)
        d = d[np.isfinite(d)]
        if len(d) < 50:
            return T, np.inf
        thr = np.quantile(d, self.trim)
        return T, float(np.sqrt((d[d <= thr] ** 2).mean()))
