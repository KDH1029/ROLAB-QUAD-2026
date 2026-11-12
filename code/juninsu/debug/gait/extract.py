#!/usr/bin/env python3
"""녹화에서 걸음새 실측 자료 뽑기: /sportmodestate(걸음새·모드·속도·회전속도·발 힘·자세) + /wirelesscontroller(스틱) → <bag>.npz"""
import os, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from unitree_go.msg import SportModeState, WirelessController
bag = sys.argv[1]; out = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.path.basename(bag.rstrip('/')) + '.npz')
meta = open(os.path.join(bag, 'metadata.yaml')).read(); sid = 'mcap' if 'mcap' in meta else 'sqlite3'
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=sid), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/sportmodestate', '/wirelesscontroller']))
S, W = [], []
while r.has_next():
    tp, data, t = r.read_next(); t *= 1e-9
    if tp == '/sportmodestate':
        m = deserialize_message(data, SportModeState)
        S.append([t, m.mode, m.gait_type, m.progress, *m.velocity, m.yaw_speed, *m.imu_state.rpy, *m.foot_force, m.body_height, *m.position[:2], m.foot_raise_height])
    else:
        m = deserialize_message(data, WirelessController); W.append([t, m.lx, m.ly, m.rx, m.ry, m.keys])
S = np.array(S); W = np.array(W)
np.savez(out, S=S, W=W, cols_S='t mode gait progress vx vy vz yaw_speed roll pitch yaw f0 f1 f2 f3 body_h px py raise', cols_W='t lx ly rx ry keys')
print(out, S.shape, W.shape)
u, c = np.unique(S[:, 1:3], axis=0, return_counts=True)
print(' (mode, gait) 비율:', ', '.join(f'({int(a)},{int(b)}) {n / len(S) * 100:.0f}%' for (a, b), n in zip(u, c)))
