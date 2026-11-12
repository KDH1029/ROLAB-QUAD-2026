#!/usr/bin/env python3
"""2D 스캔에서 RANSAC 으로 '긴 직선(벽)'만 남기고 나머지(콘처럼 짧은 덩어리)를 지운다 → AMCL 이 콘 없는 지도에서도 쓰게.
한 스캔마다: 점들 → RANSAC 직선 반복 추출(대응 3 cm, 점 ≥ 12, 길이 ≥ 0.6 m — 콘 폭보다 김) → 직선 위 점만 남김.
사용(ROS Jazzy 환경): python3 ransac_scan.py out/r1645   → out/r1645/scan_rs (mcap: /scan_rs ← /scan, /scan_Drs ← /scan_D)"""
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import LaserScan

D = os.path.abspath(sys.argv[1]); TOL, MIN_PTS, MIN_LEN, ITERS = 0.03, 12, 0.6, 60
rng = np.random.default_rng(0)


def keep_lines(r, a0, da):
    a = a0 + da * np.arange(len(r)); ok = np.isfinite(r)
    P = np.c_[r * np.cos(a), r * np.sin(a)]; idx = np.where(ok)[0]; keep = np.zeros(len(r), bool)
    left = idx.copy()
    for _ in range(12):                                    # 직선 최대 12개
        if len(left) < MIN_PTS:
            break
        best = None
        for _ in range(ITERS):
            i, j = rng.choice(left, 2, replace=False); d = P[j] - P[i]; n = np.hypot(*d)
            if n < 0.05:
                continue
            nn = np.array([-d[1], d[0]]) / n; dist = np.abs((P[left] - P[i]) @ nn); inl = left[dist < TOL]
            if best is None or len(inl) > len(best[0]):
                best = (inl, d / n, P[i])
        if best is None or len(best[0]) < MIN_PTS:
            break
        inl, u, p0 = best; s = (P[inl] - p0) @ u
        # 이어진 덩어리로 나눠(15 cm 넘게 끊기면 다른 조각) 길이 ≥ MIN_LEN 인 조각만 벽
        o = np.argsort(s); s_, inl_ = s[o], inl[o]; cut = np.where(np.diff(s_) > 0.15)[0] + 1
        for seg_s, seg_i in zip(np.split(s_, cut), np.split(inl_, cut)):
            if len(seg_i) >= MIN_PTS and seg_s[-1] - seg_s[0] >= MIN_LEN:
                keep[seg_i] = True
        left = np.setdiff1d(left, inl)
    out = np.where(keep, r, np.inf).astype(np.float32); return out, ok.sum(), keep.sum()


w = rosbag2_py.SequentialWriter(); out_uri = os.path.join(D, 'scan_rs')
import shutil; shutil.rmtree(out_uri, ignore_errors=True)
w.open(rosbag2_py.StorageOptions(uri=out_uri, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
for t in ('/scan_rs', '/scan_Drs'):
    w.create_topic(rosbag2_py.TopicMetadata(id=0, name=t, type='sensor_msgs/msg/LaserScan', serialization_format='cdr'))
stat = {}
for uri, src, dst in ((os.path.join(D, 'rec'), '/scan', '/scan_rs'), (os.path.join(D, 'bands'), '/scan_D', '/scan_Drs')):
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=uri, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=[src])); n = tot = kept = 0
    while r.has_next():
        _, data, t = r.read_next(); m = deserialize_message(data, LaserScan)
        rr, a, b = keep_lines(np.asarray(m.ranges, float), m.angle_min, m.angle_increment); m.ranges = rr.tolist()
        w.write(dst, serialize_message(m), t); n += 1; tot += a; kept += b
    stat[dst] = (n, kept / max(tot, 1))
del w
for k, (n, f) in stat.items():
    print(f'{k}: 스캔 {n}개 · 남긴 점(벽 직선) {f * 100:.0f}%')
