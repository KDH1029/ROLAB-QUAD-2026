#!/usr/bin/env python3
"""라이다 콘 검출 채점 — 정답 = 지도에서 콘으로 본 점(rec1650 지도에는 있고 콘 지운 지도 A 에는 없는 점, 슬라럼 구역).
검출(추적기가 확정한 콘, 마지막 목록)이 정답 점 0.25 m 안이면 맞음, 아니면 헛검출. 콘 덩어리(4 cm 칸 이어짐) 중 검출이 0.3 m 안에 있는 비율 = 찾은 비율.
사용(ROS Jazzy 환경): python3 ct_score.py out/r1645"""
import os
import sys

import numpy as np
import rosbag2_py
from geometry_msgs.msg import PoseArray
from rclpy.serialization import deserialize_message
from scipy import ndimage as ndi
from scipy.spatial import cKDTree

D = os.path.abspath(sys.argv[1]); H = os.path.dirname(os.path.abspath(__file__))
a = np.load(os.path.expanduser('~/ws/junk/routes/rec1650/map.npz')); b = np.load(os.path.join(H, 'out', 'maps', 'A', 'map.npz'))
ka = set(map(tuple, a['vox'].tolist())); kb = set(map(tuple, b['vox'].tolist()))
cone = np.array([k for k in ka - kb], float) if len(ka - kb) else np.zeros((0, 3))
V = float(a['V']); G = (cone + 0.5) * V
print(f'정답 콘 점 {len(G)}개(복셀)')
ij = np.floor((G[:, :2] - G[:, :2].min(0)) / 0.04).astype(int); M = np.zeros(ij.max(0) + 2, bool); M[ij[:, 0], ij[:, 1]] = True
lab, n = ndi.label(ndi.binary_dilation(M, iterations=1), structure=np.ones((3, 3))); cl = lab[ij[:, 0], ij[:, 1]]
blobs = [G[cl == k, :2] for k in range(1, n + 1) if np.sum(cl == k) >= 30]
print(f'콘 덩어리(점 30개 넘는 것) {len(blobs)}개')
T = cKDTree(G[:, :2])
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=os.path.join(D, 'ct'), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
last, first_t, cnt = {}, {}, {}
while r.has_next():
    tp, data, t = r.read_next(); m = deserialize_message(data, PoseArray)
    P = np.array([[p.position.x, p.position.y] for p in m.poses]) if m.poses else np.zeros((0, 2))
    last[tp] = P; cnt[tp] = cnt.get(tp, 0) + 1
    if len(P) and tp not in first_t:
        first_t[tp] = t * 1e-9
for tp in ('/mission/cones_def', '/mission/cones_big'):
    P = last.get(tp, np.zeros((0, 2)))
    if len(P) == 0:
        print(f'{tp}: 확정 콘 0개 (메시지 {cnt.get(tp, 0)}개) — 검출 안 됨'); continue
    d = T.query(P)[0]; tp_ = d < 0.25
    found = sum(1 for B in blobs if np.min(np.hypot(*(B[:, None, :] - P[None, :, :]).transpose(2, 0, 1)).min(0)) < 0.3 or
                np.hypot(*(B[:, None, :] - P[None, :, :]).transpose(2, 0, 1)).min() < 0.3)
    print(f'{tp}: 확정 콘 {len(P)}개 · 맞음(콘 0.25 m 안) {tp_.sum()} · 헛검출 {(~tp_).sum()} · 콘 덩어리 {found}/{len(blobs)} 찾음 · 맞은 것 콘 점까지 중앙 {np.median(d[tp_]) * 100 if tp_.any() else float("nan"):.0f} cm')
    for p, dd in zip(P, d):
        print(f'     ({p[0]:.2f}, {p[1]:.2f}) 콘 점까지 {dd * 100:.0f} cm' + ('' if dd < 0.25 else '  ← 헛검출'))
