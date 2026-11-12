#!/usr/bin/env python3
"""small_gicp(1.0.1, MIT) 기반 정합기 — backup_core.Matcher 와 같은 인터페이스 align(P, T0) -> (T, rms).

- 지도: 한 번만 전처리(0.10 m 다운샘플, 이웃 10개로 공분산) + KdTree
- 스캔: 수평 세운 몸체 좌표 점(0.15 m 다운샘플) → GICP 6자유도, 초기값 T0
- rms: 결과로 옮긴 스캔 점과 지도 사이 최근접 거리의 하위 70% RMS (기존 게이트 0.08 m 를 그대로 쓰기 위해 같은 정의)

11-04 07:40 수정 (검토 ② — rms 게이트가 틀린 자세를 통과시킴): 정합 후 '지도 0.10 m 안 점 비율' >= min_inlier(0.70) 추가 판정
  미달이면 rms=inf 로 돌려 BackupEstimator 가 거부한다 (인터페이스 align(P, T0) -> (T, rms) 그대로)
  문턱 실험 tests_d/gicp_inlier_experiment.py (앞 270초 지도 · 뒤 절반 스캔 120장, 맞는 자세를 비틀어 잼):
                      정상 정합   x0.2   x0.3   x0.5   y0.3   yaw4   yaw6
    rms 게이트만 통과    100%     66%    19%   5.4%   7.9%   100%   95%
    + 비율>=0.70 통과    100%    4.6%   0.8%    0%     0%    84%    32%
  정상 정합 비율 최소 0.744 / p1 0.754 / 중앙 0.953.  '하위 70% 거리가 0.10 m 안' 과 같은 뜻이라 trim 0.7 과 맞물린다
  min_inlier=0 이면 예전과 똑같다. 진단: last_inlier(직전 비율), n_low_inlier(이 판정으로 거부한 수)
"""
import math
import numpy as np
import small_gicp
from scipy.spatial import cKDTree


class GicpMatcher:
    solve_z = False

    def __init__(self, mp, map_res=0.10, scan_res=0.15, max_corr=1.0, threads=4, iters=30, trim=0.7, need_converged=True,
                 min_inlier=0.70, inlier_d=0.10):
        self.need_converged = need_converged    # False: 수렴 표시가 없어도 잔차(rms)로만 판정
        self.n_noconv = 0
        self.min_inlier, self.inlier_d = min_inlier, inlier_d
        self.n_low_inlier = 0; self.last_inlier = float('nan')
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
        self.last_inlier = float((d <= self.inlier_d).mean()) if len(d) else 0.0   # 분모 = 전체 스캔 점 (지도 1 m 밖 점도 포함)
        d = d[np.isfinite(d)]
        if len(d) < 50:
            return T, np.inf
        if self.last_inlier < self.min_inlier:
            self.n_low_inlier += 1
            return T, np.inf
        thr = np.quantile(d, self.trim)
        return T, float(np.sqrt((d[d <= thr] ** 2).mean()))
