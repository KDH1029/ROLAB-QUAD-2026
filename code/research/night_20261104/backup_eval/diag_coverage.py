#!/usr/bin/env python3
"""오차가 몰린 구간(495~505초)이 '정합기 탓'인지 '앞부분 지도에 그곳이 없어서'인지 가른다.
FAST-LIO 자세로 놓은 스캔 점이 앞부분 지도 / 전체 지도 10 cm 안에 드는 비율을 시간대별로 비교."""
import sys, bisect, numpy as np, rosbag2_py
from scipy.spatial import cKDTree
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
sys.path.insert(0, '/home/ilovehan9283/ws/v_gosu/backup')
from backup_core import load_map, q2R, R_LIDAR, T_LIDAR, voxel
BAG = '/home/ilovehan9283/bags/gosu_20261030_001126'
NPZ = '/home/ilovehan9283/ws/v_gosu/eval/out/gosu1030v2_map.npz'
def reader(topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics)); return r
d = np.load(NPZ); odom, lid = d['odom'], d['lid']
FULL = (d['vox'][d['cnt'] >= 2].astype(np.float64) + 0.5) * float(d['V'])
hdr2bag = {}
r = reader(['/lidar_points'])
while r.has_next():
    tp, raw, t = r.read_next(); s, ns = np.frombuffer(raw[4:8], '<i4')[0], np.frombuffer(raw[8:12], '<u4')[0]
    hdr2bag[round(float(s) + float(ns) * 1e-9, 4)] = t * 1e-9
pairs = np.array([(w, hdr2bag[round(h, 4)]) for w, h in lid if round(h, 4) in hdr2bag])
Kp = float(np.median(pairs[:, 0] - pairs[:, 1])); wl = np.sort(lid[:, 0]); h_o = odom[:, 1]
dmed = float(np.median(h_o - wl[np.clip(np.searchsorted(wl, h_o) - 1, 0, len(wl) - 1)]))
tL = h_o - dmed - Kp; o = np.argsort(tL); tL = tL[o]; GT = odom[o]; t0 = tL[0]
def gt_at(ts): return GT[min(max(bisect.bisect_left(tL, ts), 0), len(tL) - 1)]
HALF = load_map(sys.argv[1]); kh = cKDTree(voxel(HALF, 0.05)); kf = cKDTree(voxel(FULL, 0.05))
first_xy = GT[(tL - t0) < 270, 2:4]; kxy = cKDTree(first_xy)
r = reader(['/lidar_points']); last = -1e9
print(' 초   앞지도10cm  전체지도10cm  앞부분궤적과의거리  z')
while r.has_next():
    tp, raw, t = r.read_next(); t *= 1e-9; rel = t - t0
    if rel < 430 or rel > 515 or rel - last < 2.5: continue
    last = rel; m = deserialize_message(raw, PointCloud2)
    a = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)
    B = a @ R_LIDAR.T + T_LIDAR; rr = np.hypot(B[:, 0], B[:, 1]); B = B[(rr > 0.7) & (rr < 12)]
    g = gt_at(t); W = voxel(B @ q2R(*g[5:9]).T + g[2:5], 0.15)
    dh, _ = kh.query(W); df, _ = kf.query(W); dd, _ = kxy.query(g[2:4])
    print(f'{rel:5.0f}  {np.mean(dh < 0.1):9.2f}  {np.mean(df < 0.1):11.2f}  {dd:14.2f} m  {g[4]:+.2f}')
