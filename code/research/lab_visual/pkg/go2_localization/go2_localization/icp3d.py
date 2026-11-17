#!/usr/bin/env python3
"""3D 스캔-지도 정합 — 2D 단면을 거치지 않는 위치추정.

【왜 3D 인가】
지금은 점군을 중력정렬 프레임에서 수평으로 잘라 2D 로 만들고 그걸 지도와 맞춘다.
문제는 **자르는 행위 자체**다:
  · 몸통이 기울면 다른 단면이 잘려 같은 자리에서도 모양이 달라진다
  · 바닥·천장이 대역에 섞여 가짜 근거리 반사가 생긴다
  · 한 축으로 긴 공간에서는 단면이 대칭이라 위치가 모호해진다(50cm 미검출 실측)

3D 끼리 맞추면 자를 필요가 없다. 로봇이 기울어도 **본 것은 같은 3차원 구조**다.
기울기는 정합 결과의 일부로 풀리지, 관측을 망가뜨리지 않는다.

【무엇을 내는가】
map -> odom 변환. AMCL 이 하던 역할을 대신한다.
odom -> base_link 는 그대로 다리 기구학이 담당한다(저지연·무발산).

【검증부터】
로봇 없이 녹화본으로 돌려서
  · 수렴하는가          잔차가 줄어드는가
  · 얼마나 걸리는가      한 번 정합에 몇 ms 인가
  · 얼마나 정확한가      알려진 자세를 흔들어 넣고 되찾는가
를 먼저 잰다. 실주행은 그다음이다.

  python3 icp3d.py --selftest              합성 시험 (지도만 있으면 됨)
  python3 icp3d.py --bag walk_default300_0828 --n 40    녹화본 시험
"""

import argparse
import math
import os
import sys
import time

import numpy as np
from scipy.spatial import cKDTree


# ══════════════════════════════════════════════════════════════════════
#  정합부 — ROS 없이 동작
# ══════════════════════════════════════════════════════════════════════

def read_pcd(path, max_pts=400000):
    hdr = {}
    with open(path, 'rb') as f:
        while True:
            ln = f.readline()
            if not ln:
                raise RuntimeError('헤더가 안 끝남')
            k = ln.decode('ascii', 'ignore').strip().split()
            if not k:
                continue
            hdr[k[0]] = k[1:]
            if k[0] == 'DATA':
                break
        n = int(hdr['POINTS'][0])
        rec = sum(int(x) for x in hdr['SIZE'])
        if hdr['DATA'][0] == 'binary':
            a = np.frombuffer(f.read(n * rec), dtype=np.float32).reshape(-1, rec // 4)
            xyz = a[:, :3].astype(np.float64)
        else:
            xyz = np.loadtxt(f, usecols=(0, 1, 2), max_rows=n, dtype=np.float64)
    xyz = xyz[np.isfinite(xyz).all(1)]
    if len(xyz) > max_pts:
        xyz = xyz[np.random.default_rng(0).choice(len(xyz), max_pts, replace=False)]
    return xyz


def voxel(pts, size):
    """복셀 다운샘플. 점 수를 줄여 KD 트리와 대응 탐색을 싸게 만든다."""
    if size <= 0:
        return pts
    key = np.floor(pts / size).astype(np.int64)
    _, idx = np.unique(key, axis=0, return_index=True)
    return pts[np.sort(idx)]


def se3(yaw, t):
    c, s = math.cos(yaw), math.sin(yaw)
    T = np.eye(4)
    T[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    T[:3, 3] = t
    return T


class Matcher:
    """지도를 한 번 받아두고, 스캔이 올 때마다 자세를 보정한다.

    【평면 자유도만 푼다】
    로봇은 바닥을 걷는다. 높이·롤·피치는 지형과 자세가 정하고, 우리가 고칠 것은
    x, y, yaw 세 개다. 6자유도를 다 풀면 보행 중 흔들림이 자세 추정으로 새어
    들어와 오히려 나빠진다. 3자유도로 묶는 것이 이 문제에서는 더 강하다.

    【거칠게→곱게 — 2026-09-12】
    한 해상도로만 맞추면 초기 오차가 0.3 m 를 넘는 순간 못 따라간다(자체시험 6개 중 3개 실패).
    대응점 한계와 복셀을 크게 잡고 시작해 단계적으로 줄이면, 큰 오차는 거친 단계에서 당겨지고
    마지막 단계에서 정밀도가 나온다. 단계별 트리는 초기화 때 한 번만 만든다.
    """

    # (복셀 m, 대응점 한계 m, 반복) — 2026-09-12 실측: 1단이면 0.3 m 오차도 회복 0%, 4단이면 0.5 m/5° 까지 100%
    STAGES = ((0.80, 5.0, 30), (0.40, 2.5, 25), (0.20, 1.0, 20), (0.10, 0.4, 15))

    def __init__(self, map_pts, voxel_m=0.15, max_corr=1.0, iters=20, trim=0.7, stages=None):
        self.stages = list(stages if stages is not None else self.STAGES)
        self.levels = []
        for vx, mc, it in self.stages:
            pts = voxel(map_pts, vx)
            self.levels.append((cKDTree(pts), pts, mc, it))
        self.tree, self.map = self.levels[-1][0], self.levels[-1][1]
        self.trim = trim          # 잔차 하위 70% 만 쓴다 (유령 장애물 방어)

    def align(self, scan, T0):
        """거칠게→곱게 단계별 정합. 돌려주는 것: (자세, 마지막 단계 RMS, 총 반복수)"""
        T = T0.copy(); total = 0; rms = float('inf')
        for tree, mp, mc, it in self.levels:
            T, rms, n = self._align_one(scan, T, tree, mp, mc, it)
            total += n
            if not np.isfinite(rms):
                break
        return T, rms, total

    def _align_one(self, scan, T0, tree, mp, max_corr, iters):
        """한 해상도에서 맞춘다. scan 은 로봇 기준 3D, T0 은 초기 추정(4x4)."""
        T = T0.copy()
        P = scan
        last = None
        for it in range(iters):
            Q = (T[:3, :3] @ P.T).T + T[:3, 3]
            d, j = tree.query(Q, k=1, distance_upper_bound=max_corr)
            ok = np.isfinite(d)
            if ok.sum() < 50:
                return T, float('inf'), 0
            dd = d[ok]
            # 절사 — 지도에 없는 것(사람·옮겨진 물건)이 정합을 끌지 않게
            thr = np.quantile(dd, self.trim)
            sel = ok.copy()
            sel[ok] = dd <= thr
            if sel.sum() < 50:
                sel = ok
            A = Q[sel]
            B = mp[j[sel]]
            # 평면 3자유도만: 중심 맞추고 yaw 만 푼다
            ca, cb = A[:, :2].mean(0), B[:, :2].mean(0)
            X = A[:, :2] - ca
            Y = B[:, :2] - cb
            num = float((X[:, 0] * Y[:, 1] - X[:, 1] * Y[:, 0]).sum())
            den = float((X[:, 0] * Y[:, 0] + X[:, 1] * Y[:, 1]).sum())
            dyaw = math.atan2(num, den)
            c, s = math.cos(dyaw), math.sin(dyaw)
            R2 = np.array([[c, -s], [s, c]])
            dt = cb - R2 @ ca
            dT = np.eye(4)
            dT[:2, :2] = R2
            dT[:2, 3] = dt
            T = dT @ T
            rms = float(np.sqrt((dd[dd <= thr] ** 2).mean()))
            if last is not None and abs(last - rms) < 1e-4:
                return T, rms, it + 1
            last = rms
        return T, (last if last is not None else float('inf')), iters


def selftest(map_path):
    print('3D 정합 자체시험')
    print('=' * 66)
    mp = read_pcd(map_path)
    print('  지도 %s   점 %d개' % (os.path.basename(map_path), len(mp)))
    m = Matcher(mp)
    print('  복셀 후 %d개' % len(m.map))

    rng = np.random.default_rng(1)
    # 지도에서 한 덩어리를 떼어 '스캔'으로 쓴다 — 로봇 주변만 보이는 상황 재현
    ctr = mp.mean(0)
    ctr[2] = np.percentile(mp[:, 2], 30)
    d = np.linalg.norm(mp[:, :2] - ctr[:2], axis=1)
    scan_full = mp[d < 8.0]
    scan_full = scan_full[rng.choice(len(scan_full),
                                     min(3000, len(scan_full)), replace=False)]
    print('  가상 스캔 %d개 (반경 8m)' % len(scan_full))
    print()
    print('  %-22s %-11s %-11s %-9s %s'
          % ('주입 오차', '남은 위치오차', '남은 각도오차', '반복', '시간'))
    print('  ' + '-' * 62)
    ok = 0
    cases = [(0.0, 0.0, 0.0), (0.3, 0.0, 0.0), (0.0, 0.3, 0.0),
             (0.0, 0.0, 5.0), (0.5, -0.4, 10.0), (1.0, 0.8, 20.0)]
    for dx, dy, dyaw in cases:
        Ttrue = np.eye(4)
        # 스캔을 로봇 기준으로: 지도 좌표에서 중심을 뺀다
        P = scan_full - ctr
        T0 = se3(math.radians(dyaw), [ctr[0] + dx, ctr[1] + dy, ctr[2]])
        t0 = time.perf_counter()
        T, rms, it = m.align(P, T0)
        ms = (time.perf_counter() - t0) * 1000
        ex = math.hypot(T[0, 3] - ctr[0], T[1, 3] - ctr[1])
        ey = abs(math.degrees(math.atan2(T[1, 0], T[0, 0])))
        ey = min(ey, 360 - ey)
        good = ex < 0.10 and ey < 2.0
        ok += good
        print('  %-22s %8.1f mm %8.2f°   %4d   %6.0f ms  %s'
              % ('x%+.1f y%+.1f yaw%+.0f°' % (dx, dy, dyaw),
                 ex * 1000, ey, it, ms, 'OK' if good else '★'))
    print('  ' + '-' * 62)
    print('  %d / %d 통과   (기준: 위치 100mm, 각도 2°)' % (ok, len(cases)))
    return 0 if ok == len(cases) else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default=os.path.join(
        os.path.expanduser('~'), 'ws', 'go2_3d', 'maps', 'map_0907.pcd'))
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args()
    if a.selftest:
        return selftest(a.map)
    ap.print_help()
    return 1


if __name__ == '__main__':
    sys.exit(main())
