#!/usr/bin/env python3
"""재생 덤프(out/<태그>/rec: /cloud_registered_body + /Odometry)로 2 cm 복셀 지도(map.npz, 지도 수집기와 같은 형식)를 쌓는다 — 지우기 없음.
clean_map.py 와 같은 자르기(길 둘레 ±6 m, 바닥 −0.3 ~ +2.0 m)라 '사람 지운 판'과 1:1 비교된다.  사용: python3 build_dump_map.py <덤프 폴더> <출력 폴더>"""
import os, sys, time
import numpy as np, rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
D, OUT = os.path.abspath(sys.argv[1]), os.path.abspath(sys.argv[2]); os.makedirs(OUT, exist_ok=True); V = 0.02; t0 = time.time()


def reader(topics):
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=os.path.join(D, 'rec'), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics)); return r


def quatR(qx, qy, qz, qw):
    return np.array([[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)], [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                     [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])


od = []; r = reader(['/Odometry'])
while r.has_next():
    _, data, _ = r.read_next(); m = deserialize_message(data, Odometry); p, q = m.pose.pose.position, m.pose.pose.orientation
    od.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
od = np.array(od); ot = od[:, 0]; floor_z = np.median(od[:, 3]) - 0.32; lo, hi = od[:, 1:3].min(0) - 6.0, od[:, 1:3].max(0) + 6.0
r = reader(['/cloud_registered_body']); keys = []; k = 0
while r.has_next():
    _, data, _ = r.read_next(); m = deserialize_message(data, PointCloud2); st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    j = int(np.clip(np.searchsorted(ot, st), 0, len(ot) - 1))
    if abs(ot[j] - st) > 0.06:
        continue
    a = np.frombuffer(bytes(m.data), np.uint8).reshape(-1, m.point_step)
    xyz = np.stack([a[:, o:o + 4].copy().view(np.float32)[:, 0] for o in (0, 4, 8)], 1); xyz = xyz[np.isfinite(xyz).all(1)].astype(np.float64)
    W = xyz @ quatR(*od[j, 4:8]).T + od[j, 1:4]
    kk = (W[:, 0] > lo[0]) & (W[:, 0] < hi[0]) & (W[:, 1] > lo[1]) & (W[:, 1] < hi[1]) & (W[:, 2] > floor_z - 0.3) & (W[:, 2] < floor_z + 2.0)
    keys.append(np.floor(W[kk] / V).astype(np.int32)); k += 1
U, cnt = np.unique(np.concatenate(keys), axis=0, return_counts=True)
np.savez_compressed(os.path.join(OUT, 'map.npz'), vox=U.astype(np.int32), cnt=cnt.astype(np.int32), V=np.float64(V), odom=np.c_[od[:, 0], od[:, 0], od[:, 1:8]], lid=np.c_[od[:, 0], od[:, 0]])
print(f'{os.path.basename(OUT)}: 프레임 {k} · 복셀 {len(U)} · {time.time() - t0:.0f}초')
