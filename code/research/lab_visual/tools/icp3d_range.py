#!/usr/bin/env python3
"""3D 정합의 수렴 범위를 잰다 — 여러 지점에서 가상 스캔을 만들고 오차를 주입해 되찾는 비율.
  python3 icp3d_range.py --map 지도.pcd [--stages A|B|C] [--spots 6]
단계표 A = 기존 1단, B = 3단(현재 기본), C = 4단(더 거친 단계 추가)"""
import argparse, math, os, sys, time
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from icp3d import voxel, se3, Matcher
from clean_map import read_pcd   # 필드 순서를 헤더에서 찾는 판독기
ap = argparse.ArgumentParser()
ap.add_argument('--map', default=os.path.expanduser('~/ws/lab_visual/maps/go2_prior_map_clean.pcd'))
ap.add_argument('--stages', default='B'); ap.add_argument('--spots', type=int, default=6)
ap.add_argument('--area', default='', help='시험 지점을 제한할 구역 "xmin,xmax,ymin,ymax" (비우면 지도 전체)')
a = ap.parse_args()
ST = {'A': ((0.15, 1.0, 20),),
      'B': ((0.40, 2.5, 30), (0.20, 1.0, 20), (0.10, 0.4, 15)),
      'C': ((0.80, 5.0, 30), (0.40, 2.5, 25), (0.20, 1.0, 20), (0.10, 0.4, 15))}[a.stages]
mp = read_pcd(a.map)
m = Matcher(mp, stages=ST)
lo, hi = mp[:, :2].min(0), mp[:, :2].max(0)
if a.area:
    v = [float(x) for x in a.area.split(',')]
    lo, hi = np.array([v[0], v[2]]), np.array([v[1], v[3]])
    print('  시험 구역 제한: x %.1f~%.1f  y %.1f~%.1f' % (v[0], v[1], v[2], v[3]))
rng = np.random.default_rng(0)
# 지도 점이 주변에 충분한 지점만 가상 스캔 위치로 쓴다
spots = []
while len(spots) < a.spots:
    p = rng.uniform(lo + 1.0, hi - 1.0)
    near = np.hypot(mp[:, 0] - p[0], mp[:, 1] - p[1])
    if (near < 8.0).sum() > 3000: spots.append(p)
errs = [(0.3, 0), (0.5, 5), (0.8, 10), (1.2, 15), (1.5, 20), (2.0, 30)]
print('지도 %s  점 %d  단계표 %s %s' % (os.path.basename(a.map), len(mp), a.stages, ST))
print('  가상 스캔 %d지점 × 오차 %d종 = %d회' % (len(spots), len(errs), len(spots) * len(errs)))
print('  %-18s %8s %8s %8s %7s' % ('주입 오차', '성공률', '위치오차', '각도오차', '시간'))
print('  ' + '-' * 56)
for de, da in errs:
    ok = 0; pe = []; ae = []; tt = []
    for p in spots:
        near = mp[np.hypot(mp[:, 0] - p[0], mp[:, 1] - p[1]) < 8.0]
        idx = rng.choice(len(near), size=min(3000, len(near)), replace=False)
        world = near[idx]
        scan = world - np.r_[p, 0.0]                     # 로봇 기준 (yaw 0)
        th = rng.uniform(-math.pi, math.pi)
        T0 = se3(da * math.pi / 180, [p[0] + de * math.cos(th), p[1] + de * math.sin(th), 0.0])
        t0 = time.time(); T, rms, it = m.align(scan, T0); el = (time.time() - t0) * 1000
        dp = math.hypot(T[0, 3] - p[0], T[1, 3] - p[1]) * 1000
        dyaw = abs(math.degrees(math.atan2(T[1, 0], T[0, 0])))
        pe.append(dp); ae.append(dyaw); tt.append(el)
        if dp < 100 and dyaw < 2.0: ok += 1
    print('  %-18s %6d%%  %6.0f mm %7.2f° %6.0f ms' % ('%.1f m / %d°' % (de, da), 100 * ok // len(spots), np.median(pe), np.median(ae), np.median(tt)))
