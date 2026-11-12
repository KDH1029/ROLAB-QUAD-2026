#!/usr/bin/env python3
"""495~505초 잔차 급등 원인: 다리 오도 roll·pitch 와 FAST-LIO roll·pitch 차이를 시간대별로."""
import sys, bisect, math, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
sys.path.insert(0, '/home/ilovehan9283/ws/v_gosu/backup')
from backup_core import q2R, rpy_of
BAG = '/home/ilovehan9283/bags/gosu_20261030_001126'
NPZ = '/home/ilovehan9283/ws/v_gosu/eval/out/gosu1030v2_map.npz'
def reader(topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics)); return r
d = np.load(NPZ); odom, lid = d['odom'], d['lid']
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
r = reader(['/utlidar/robot_odom']); rows = []
while r.has_next():
    tp, raw, t = r.read_next(); t *= 1e-9; rel = t - t0
    if rel < 270: continue
    m = deserialize_message(raw, Odometry); q = m.pose.pose.orientation
    lr, lp, _ = rpy_of(q2R(q.x, q.y, q.z, q.w)); g = gt_at(t); fr, fp, _ = rpy_of(q2R(*g[5:9]))
    rows.append((rel, math.degrees(lr), math.degrees(lp), math.degrees(fr), math.degrees(fp)))
R = np.array(rows); np.save('tilt_leg_vs_fastlio.npy', R)
dr, dp = R[:, 1] - R[:, 3], R[:, 2] - R[:, 4]
print('전 구간(270~) roll차 중앙', np.median(dr).round(2), 'p95|', np.percentile(abs(dr - np.median(dr)), 95).round(2),
      '/ pitch차 중앙', np.median(dp).round(2), 'p95|', np.percentile(abs(dp - np.median(dp)), 95).round(2))
for a in np.arange(484, 510, 1.0):
    m = (R[:, 0] >= a) & (R[:, 0] < a + 1)
    print(f'{a:.0f}s leg r/p {R[m,1].mean():+6.2f} {R[m,2].mean():+6.2f}   LIO r/p {R[m,3].mean():+6.2f} {R[m,4].mean():+6.2f}   차 {dr[m].mean():+5.2f} {dp[m].mean():+5.2f}')
