#!/usr/bin/env python3
"""시뮬레이션 벽 = 실제 wall_filter 노드(고친 판) 재생 출력(/mission/wall_cloud, FAST-LIO 좌표 = 저장 좌표와 같은 좌표계)에서
로봇 1~4 m 안 점만 쌓은 것(가까운 벽은 스캔 기준 오차 중앙 1.7 cm). 사용: make_sim_walls_from_replay.py <replay out 폴더> <출력 npy> [최대거리=4]"""
import sys, math, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2 as pc2
from scipy.spatial import cKDTree
B, OUT = sys.argv[1], sys.argv[2]; MAXR = float(sys.argv[3]) if len(sys.argv) > 3 else 4.0
PC, OD = get_message('sensor_msgs/msg/PointCloud2'), get_message('nav_msgs/msg/Odometry')
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=B, storage_id='mcap'), rosbag2_py.ConverterOptions('', '')); r.set_filter(rosbag2_py.StorageFilter(topics=['/mission/wall_cloud', '/Odometry']))
od, acc = [], []; last = None
while r.has_next():
    tp, raw, t = r.read_next()
    if tp == '/Odometry':
        p = deserialize_message(raw, OD).pose.pose.position; last = (p.x, p.y); od.append(last); continue
    if last is None: continue
    a = pc2.read_points(deserialize_message(raw, PC), field_names=['x', 'y'], skip_nans=True); P = np.stack([a['x'], a['y']], 1)
    d = np.hypot(P[:, 0] - last[0], P[:, 1] - last[1]); P = P[(d >= 1.0) & (d <= MAXR)]
    if len(P): acc.append(np.c_[P, np.full(len(P), t * 1e-9)])
W = np.concatenate(acc); G = 0.04; kk = np.floor(W[:, :2] / G).astype(np.int64); tb = ((W[:, 2] - W[:, 2].min()) // 10).astype(np.int64)
pairs = np.unique(np.c_[kk, tb], axis=0); cell, nwin = np.unique(pairs[:, :2], axis=0, return_counts=True)
cells = (cell[nwin >= 3] + 0.5) * G      # 서로 다른 10초 구간 3곳 이상에서 보인 칸(따라 걷던 사람 제외)
path = np.array(od); dd, _ = cKDTree(path).query(cells); onpath = dd < 0.20
np.save(OUT.replace('.npy', '_onpath.npy'), cells[onpath]); cells = cells[~onpath]
np.save(OUT, np.concatenate([np.c_[cells, np.full(len(cells), z)] for z in (0.15, 0.35)]).astype(np.float32))
print(f'벽 점 {len(W)} → 고정 칸 {len(cells) + onpath.sum()} (지나간 길 20 cm 안 {onpath.sum()} 칸은 따로) → {OUT} {len(cells)} 칸')
