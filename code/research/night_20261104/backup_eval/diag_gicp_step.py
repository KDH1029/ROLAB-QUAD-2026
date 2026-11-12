#!/usr/bin/env python3
"""494.6초 잔차 급등: 씨앗(=FAST-LIO 자세, 다리 roll·pitch 수평)에서의 잔차 vs GICP 뒤 잔차. GICP 가 좋은 씨앗에서 미끄러지는지."""
import sys, bisect, math, numpy as np, rosbag2_py
from scipy.spatial import cKDTree
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
sys.path.insert(0, '/home/ilovehan9283/ws/v_gosu/backup'); sys.path.insert(0, '.')
from backup_core import load_map, q2R, rpy_of, R_LIDAR, T_LIDAR, voxel
from gicp_matcher import GicpMatcher
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
M = GicpMatcher(voxel(load_map(sys.argv[1]), 0.05), need_converged=False, iters=60); kd = M.kd
def trimmed(Q):
    dd, _ = kd.query(Q, k=1, distance_upper_bound=1.0); dd = dd[np.isfinite(dd)]; thr = np.quantile(dd, 0.7)
    return float(np.sqrt((dd[dd <= thr] ** 2).mean())), len(dd)
leg = None; r = reader(['/utlidar/robot_odom', '/lidar_points']); last = -1e9
print('  초   씨앗잔차  GICP잔차  GICP이동(xy,z,yaw°,pitch°)  점수')
while r.has_next():
    tp, raw, t = r.read_next(); t *= 1e-9; rel = t - t0
    if rel < 490: continue
    if rel > 500: break
    if tp == '/utlidar/robot_odom':
        m = deserialize_message(raw, Odometry); q = m.pose.pose.orientation; leg = rpy_of(q2R(q.x, q.y, q.z, q.w)); continue
    if leg is None or rel - last < 0.4: continue
    last = rel; m = deserialize_message(raw, PointCloud2)
    a = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)
    B = a @ R_LIDAR.T + T_LIDAR; rr = np.hypot(B[:, 0], B[:, 1]); B = B[(rr > 0.7) & (rr < 12)]
    ro, pi, _ = leg; cr, sr, cp, sp = math.cos(ro), math.sin(ro), math.cos(pi), math.sin(pi)
    P = B @ np.array([[cp, sp * sr, sp * cr], [0, cr, -sr], [-sp, cp * sr, cp * cr]]).T
    g = gt_at(t); yaw = rpy_of(q2R(*g[5:9]))[2]; T0 = np.eye(4); c, s = math.cos(yaw), math.sin(yaw)
    T0[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]; T0[:3, 3] = g[2:5]
    rs, n = trimmed(P @ T0[:3, :3].T + T0[:3, 3]); T, rg = M.align(P, T0)
    dT = np.linalg.inv(T0) @ T; dyaw = math.degrees(math.atan2(dT[1, 0], dT[0, 0])); dpit = math.degrees(math.asin(-dT[2, 0]))
    print(f'{rel:6.1f}  {rs:7.3f}  {rg:7.3f}   {math.hypot(*dT[:2,3]):.3f} {dT[2,3]:+.3f} {dyaw:+.2f} {dpit:+.2f}   {len(P)}')
