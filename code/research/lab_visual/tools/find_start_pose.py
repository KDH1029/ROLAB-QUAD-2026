#!/usr/bin/env python3
"""녹화본의 시작 자세를 지도에서 찾는다 — 전역 탐색.

  python3 find_start_pose.py --bag ~/ws/lab/bags/data4 --map maps/go2_prior_map_clean.pcd --at 15

【왜】 위치추정은 초기 자세가 0.5 m / 5° 안이어야 수렴한다(실측). 녹화본이 지도의 어디에서
시작했는지 모르면 아무리 좋은 정합기도 엉뚱한 곳을 맴돈다.
여기서는 지도 전역을 격자로 훑어 스캔이 가장 잘 맞는 자세를 찾고, 2등과의 격차도 같이 본다.
격차가 작으면 그 지도는 그 자리에서 모호하다는 뜻이라 정합만으로는 못 푼다.
"""
import argparse, math, os, sys, time
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from read_pcd import read_pcd
from scipy.spatial import cKDTree
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

def storage_id(bag):
    """녹화본 형식을 파일 확장자로 판별한다. ROS 2 Jazzy 기본은 mcap, 예전 녹화본은 sqlite3."""
    import glob as _g, os as _o
    if _g.glob(_o.path.join(bag, '*.mcap')):
        return 'mcap'
    return 'sqlite3'


LIDAR_X, LIDAR_YAW = 0.25, math.radians(90.0)   # base_link -> hesai_lidar

ap = argparse.ArgumentParser()
ap.add_argument('--bag', required=True)
ap.add_argument('--map', default=os.path.expanduser('~/ws/lab_visual/maps/go2_prior_map_clean.pcd'))
ap.add_argument('--at', type=float, default=15.0)
ap.add_argument('--step', type=float, default=0.4)
ap.add_argument('--astep', type=float, default=10.0)
ap.add_argument('--area', default='', help='"xmin,xmax,ymin,ymax" 비우면 지도 전체')
ap.add_argument('--pts', type=int, default=1200)
a = ap.parse_args()

mp = read_pcd(os.path.expanduser(a.map))
fl = np.percentile(mp[:, 2], 1)
band = mp[(mp[:, 2] - fl > 0.2) & (mp[:, 2] - fl < 1.6)]      # 스캔이 보는 높이
kd = cKDTree(band[:, :2])
print('지도 %s  전체 %d점 -> 대역 %d점' % (os.path.basename(a.map), len(mp), len(band)))

r = SequentialReader(); r.open(StorageOptions(uri=os.path.expanduser(a.bag), storage_id=storage_id(os.path.expanduser(a.bag))), ConverterOptions('', ''))
r.set_filter(StorageFilter(topics=['/lidar_points']))
t0 = None; scan = None
while r.has_next():
    _, data, ts = r.read_next()
    if t0 is None: t0 = ts
    if (ts - t0) / 1e9 < a.at: continue
    c = deserialize_message(data, PointCloud2)
    p = np.array(pc2.read_points_list(c, field_names=('x', 'y', 'z'), skip_nans=True), dtype=np.float64)
    rr = np.hypot(p[:, 0], p[:, 1])
    p = p[(p[:, 2] > -0.2) & (p[:, 2] < 1.2) & (rr > 0.8) & (rr < 15)]
    if len(p) > a.pts:
        p = p[np.random.default_rng(0).choice(len(p), a.pts, replace=False)]
    # 점군은 라이다 프레임이다. base_link 로 옮겨야 '로봇 자세' 가 나온다.
    # base_link -> hesai_lidar : 앞 0.25 m, 위 0.1588 m, 요 90° (imu_bridge 가 내는 정적 TF)
    cl, sl = math.cos(LIDAR_YAW), math.sin(LIDAR_YAW)
    p = np.c_[cl * p[:, 0] - sl * p[:, 1] + LIDAR_X, sl * p[:, 0] + cl * p[:, 1], p[:, 2]]
    scan = p[:, :2]; break
if scan is None: sys.exit('★ 스캔을 못 읽었다')
print('스캔 %d점 (bag %.0f초)' % (len(scan), a.at))

lo, hi = band[:, :2].min(0), band[:, :2].max(0)
if a.area:
    v = [float(x) for x in a.area.split(',')]; lo, hi = np.array([v[0], v[2]]), np.array([v[1], v[3]])
print('탐색 범위 x %.1f~%.1f  y %.1f~%.1f  격자 %.2f m / %.0f°' % (lo[0], hi[0], lo[1], hi[1], a.step, a.astep))

xs = np.arange(lo[0], hi[0], a.step); ys = np.arange(lo[1], hi[1], a.step)
ths = np.radians(np.arange(0, 360, a.astep))
t_start = time.time(); best = []
sub = scan[::2]
for th in ths:
    c, s = math.cos(th), math.sin(th)
    R = np.c_[c * sub[:, 0] - s * sub[:, 1], s * sub[:, 0] + c * sub[:, 1]]
    for x in xs:
        for y in ys:
            d, _ = kd.query(R + (x, y), k=1, distance_upper_bound=1.0)
            # 이웃을 못 찾은 점(inf)도 '안 맞은 것'으로 세야 한다.
            # 분모에서 빼면 어느 자세든 100%가 나온다 (2026-09-12 실제로 당함).
            best.append((float((d < 0.15).sum()) / len(sub), x, y, math.degrees(th)))
best.sort(reverse=True)
print('격자 %d개, %.0f초' % (len(xs) * len(ys) * len(ths), time.time() - t_start))
picks = []
for b in best:
    if all(math.hypot(b[1] - p[1], b[2] - p[2]) > 1.5 for p in picks):
        picks.append(b)
    if len(picks) >= 5: break
print('  순위  일치율   자세 (x, y, yaw)')
for i, (r_, x, y, th) in enumerate(picks):
    print('   %d   %5.1f%%   (%+.2f, %+.2f, %+4.0f°)' % (i + 1, r_ * 100, x, y, th))
if len(picks) >= 2:
    gap = (picks[0][0] - picks[1][0]) * 100
    print('  1-2등 격차 %.1f%%p  거리 %.2f m  → %s'
          % (gap, math.hypot(picks[0][1] - picks[1][1], picks[0][2] - picks[1][2]),
             '판정 가능' if gap > 5 else '모호함 ★ 정합만으로는 못 푼다'))
