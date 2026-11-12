#!/usr/bin/env python3
"""11:18 실주행 지도 — 실시간 FAST-LIO(다리 결합) 자세 /Odometry + 원본 /lidar_points → 2 cm 복셀(편집기 map.npz 형식).
높이 표류 −0.23~+0.26 m(11:14 재생은 0.4 m 표류). 외부파라미터 = 실주행 설정 fastlio_leg_nurigwan_derange30.yaml."""
import rosbag2_py, numpy as np, bisect, sys
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation as Rot
B = '/home/ilovehan9283/bags/gosu_20261106_111853_cone1106'; OUT = sys.argv[1]
Rbl = np.array([[-0.0001707, -0.9999240, 0.0123287], [0.9999844, -0.0001019, 0.0055807], [-0.0055790, 0.0123295, 0.9999084]]); Tbl = np.array([0.25, 0.0, 0.1588])
Od, PC = get_message('nav_msgs/msg/Odometry'), get_message('sensor_msgs/msg/PointCloud2')
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=B, storage_id='mcap'), rosbag2_py.ConverterOptions('', '')); r.set_filter(rosbag2_py.StorageFilter(topics=['/Odometry']))
od = []
while r.has_next():
    tp, raw, t = r.read_next(); m = deserialize_message(raw, Od); p, q = m.pose.pose.position, m.pose.pose.orientation
    od.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, t * 1e-9, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
od = np.array(od); ts = od[:, 0]
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=B, storage_id='mcap'), rosbag2_py.ConverterOptions('', '')); r.set_filter(rosbag2_py.StorageFilter(topics=['/lidar_points']))
V = 0.02; acc = {}; n = 0; used = 0
while r.has_next():
    tp, raw, t = r.read_next(); n += 1
    if n % 2: continue
    m = deserialize_message(raw, PC); st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    k = int(np.argmin(np.abs(ts - st))) if len(ts) else -1
    if k < 0 or abs(ts[k] - st) > 0.06: continue
    f = {x.name: x.offset for x in m.fields}; a = np.frombuffer(m.data, np.uint8).reshape(-1, m.point_step)
    P = np.stack([a[:, f[c]:f[c] + 4].copy().view('<f4')[:, 0] for c in 'xyz'], 1)[::2]; P = P[np.isfinite(P).all(1)]
    rr = np.linalg.norm(P, axis=1); P = P[(rr > 0.3) & (rr < 10.0)]
    Rw = Rot.from_quat(od[k, 5:9]).as_matrix(); Pw = (P @ Rbl.T + Tbl) @ Rw.T + od[k, 2:5]
    for key, c in zip(*np.unique(np.floor(Pw / V).astype(np.int32), axis=0, return_counts=True)):
        kk = tuple(key); acc[kk] = acc.get(kk, 0) + 1
    used += 1
    if used % 300 == 0: print(used, '장', len(acc), '복셀', flush=True)
vox = np.array(list(acc.keys()), np.int32); cnt = np.array(list(acc.values()), np.int32)
np.savez_compressed(OUT, vox=vox, cnt=cnt, V=np.float64(V), odom=od, lid=np.c_[ts, ts])
print('저장', OUT, '복셀', len(vox), '스캔', used)
