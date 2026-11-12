#!/usr/bin/env python3
"""다리 오도 z 튐 전수조사 (/utlidar/robot_odom, 전 구간)."""
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
BAG = '/home/ilovehan9283/bags/gosu_20261030_001126'
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/utlidar/robot_odom'])); rows = []
while r.has_next():
    tp, raw, t = r.read_next(); m = deserialize_message(raw, Odometry); p = m.pose.pose.position
    rows.append((t * 1e-9, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y, p.z))
A = np.array(rows); np.save('leg_odom_all.npy', A); t = A[:, 0] - A[0, 0]
dz = np.diff(A[:, 4]); dxy = np.hypot(np.diff(A[:, 2]), np.diff(A[:, 3]))
print('메시지', len(A), '길이', round(t[-1], 1), 's  z 범위', A[:, 4].min().round(3), A[:, 4].max().round(3))
print('한 스텝 |dz| p99.9', np.percentile(abs(dz), 99.9).round(4), '최대', abs(dz).max().round(3), ' |dxy| p99.9', np.percentile(dxy, 99.9).round(4), '최대', dxy.max().round(3))
big = np.where(abs(dz) > 0.05)[0]
print('|dz|>5cm 스텝', len(big))
for i in big[:30]: print(f'  {t[i+1]:7.2f}s  z {A[i,4]:+.3f} → {A[i+1,4]:+.3f}  dxy {dxy[i]:.3f}  stamp dt {A[i+1,1]-A[i,1]:+.4f}')
