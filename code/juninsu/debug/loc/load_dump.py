#!/usr/bin/env python3
"""replay_dump.sh 결과(mcap) → npz 하나로: FAST-LIO 자세 · 몸 좌표 점구름 · 2D 스캔 · 몸 높이 · 다리 오도메트리.
사용: python3 load_dump.py <out/태그>      (ROS Jazzy 환경 필요 — rosbag2_py)  → <out/태그>/dump.npz"""
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2 as pc2

D = os.path.abspath(sys.argv[1])
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=os.path.join(D, 'rec'), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
want = ['/Odometry', '/cloud_registered_body', '/scan', '/sportmodestate', '/utlidar/robot_odom']
r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in want if t in types]))
cls = {t: get_message(types[t]) for t in want if t in types}
st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
odom, ct, cxyz, sc, sct, hgt, leg = [], [], [], [], [], [], []
scan_meta = None
while r.has_next():
    topic, data, t_rec = r.read_next()
    m = deserialize_message(data, cls[topic])
    if topic == '/Odometry':
        p, q = m.pose.pose.position, m.pose.pose.orientation
        odom.append((st(m.header), p.x, p.y, p.z, q.x, q.y, q.z, q.w))
    elif topic == '/cloud_registered_body':
        a = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float32)
        ct.append(st(m.header)); cxyz.append(a)
    elif topic == '/scan':
        sct.append(st(m.header)); sc.append(np.asarray(m.ranges, np.float32))
        scan_meta = (m.angle_min, m.angle_increment, m.range_min, m.range_max, m.header.frame_id)
    elif topic == '/sportmodestate':
        hgt.append((t_rec * 1e-9, m.body_height))
    elif topic == '/utlidar/robot_odom':
        p, q = m.pose.pose.position, m.pose.pose.orientation
        leg.append((st(m.header), p.x, p.y, p.z, q.x, q.y, q.z, q.w))
idx = np.r_[0, np.cumsum([len(a) for a in cxyz])]
out = os.path.join(D, 'dump.npz')
np.savez_compressed(out, odom=np.array(odom), cloud_t=np.array(ct), cloud_idx=idx,
                    cloud_xyz=np.concatenate(cxyz) if cxyz else np.zeros((0, 3), np.float32),
                    scan_t=np.array(sct), scan=np.array(sc) if sc else np.zeros((0, 0)), scan_meta=np.array(scan_meta, dtype=object),
                    height=np.array(hgt), leg=np.array(leg))
O = np.array(odom)
print(f'{out}: 자세 {len(odom)} · 점구름 {len(ct)} 장(평균 {idx[-1] / max(1, len(ct)):.0f}점) · 스캔 {len(sc)} · 높이 {len(hgt)} · 다리 {len(leg)}'
      f' · 길이 {O[-1, 0] - O[0, 0]:.0f}초 · 이동 {np.hypot(*np.diff(O[:, 1:3], axis=0).T).sum():.1f} m · 끝 위치 ({O[-1, 1]:.2f}, {O[-1, 2]:.2f})')
