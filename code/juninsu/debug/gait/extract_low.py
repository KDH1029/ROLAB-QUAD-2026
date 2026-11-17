#!/usr/bin/env python3
"""/lowstate 발 힘(foot_force 4) + 무릎 관절 속도 → <bag>_low.npz (2개 중 1개, ~250 Hz)"""
import os, sys
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from unitree_go.msg import LowState
bag = sys.argv[1]; out = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.path.basename(bag.rstrip('/')) + '_low.npz')
sid = 'mcap' if 'mcap' in open(os.path.join(bag, 'metadata.yaml')).read() else 'sqlite3'
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=sid), rosbag2_py.ConverterOptions('', '')); r.set_filter(rosbag2_py.StorageFilter(topics=['/lowstate']))
L = []; i = 0
while r.has_next():
    tp, data, t = r.read_next(); i += 1
    if i % 2: continue
    m = deserialize_message(data, LowState); L.append([t * 1e-9, *m.foot_force, *[m.motor_state[j].dq for j in (2, 5, 8, 11)]])
L = np.array(L); np.savez(out, L=L); print(out, L.shape, 'force p50/p95', np.percentile(L[:, 1:5], [50, 95], axis=0).round(0).tolist())
