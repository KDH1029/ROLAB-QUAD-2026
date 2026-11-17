#!/usr/bin/env python3
"""AMCL 링 높이대 비교 — out/r1645/amcl_<S>_tuned_<band>/poses.csv 를 참값(출발 T · 원래 FAST-LIO)과 비교.
'all' = 대회 /scan(높이 제한 없는 세로면 스캔, amcl_S?_tuned). 스캔에서 잡힌 빔 비율도 같이(ROS Jazzy 환경에서 실행)."""
import csv
import math
import os
import sys

import numpy as np

H = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, H)
from scen import Corrupt  # noqa: E402

TAG = 'r1645'; D = os.path.join(H, 'out', TAG); T_ST = (-0.025, -0.039, math.radians(-3.46))
O = np.load(os.path.join(D, 'dump.npz'))['odom']; t0 = O[0, 0]
yaw = lambda q: math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))
TRUE = np.array([(r[0], r[1], r[2], yaw(r[4:8])) for r in O])


def comp(A, B):
    c, s = math.cos(A[2]), math.sin(A[2]); return np.array([c * B[0] - s * B[1] + A[0], s * B[0] + c * B[1] + A[1], A[2] + B[2]])


def inv(A):
    c, s = math.cos(A[2]), math.sin(A[2]); return np.array([-(c * A[0] + s * A[1]), s * A[0] - c * A[1], -A[2]])


ts = np.arange(t0 + 3.0, TRUE[-1, 0], 1.0)
true_at = lambda t: TRUE[int(np.clip(np.searchsorted(TRUE[:, 0], t), 0, len(TRUE) - 1)), 1:]
BANDS = {'all': '높이 제한 없음(대회 /scan)', 'L': '바닥 위 0.05~0.31 m', 'D': '0.29~0.57 m (브리지 기본값, 작은 콘 때)', 'M': '0.41~0.71 m', 'U': '0.61~1.21 m'}
if os.environ.get('AMCL_SETS'):
    BANDS = dict(x.split('=', 1)[::-1] for x in os.environ['AMCL_SETS'].split('|'))   # {접미: 라벨}
    cover = None
cover = {} if not os.environ.get('AMCL_SETS') else None
try:
    if cover is None:
        raise RuntimeError('목록 모드 — 빔 비율 생략')
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    for b, uri, top in [('all', os.path.join(D, 'rec'), '/scan')] + [(b, os.path.join(D, 'bands'), f'/scan_{b}') for b in 'LDMU']:
        r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=uri, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
        r.set_filter(rosbag2_py.StorageFilter(topics=[top])); v = []; n = 0
        while r.has_next() and n < 2000:
            _, d, _ = r.read_next(); m = deserialize_message(d, LaserScan); a = np.asarray(m.ranges); v.append(np.isfinite(a).mean()); n += 1
        cover[b] = float(np.mean(v))
except Exception as e:  # noqa: BLE001
    print('빔 비율 생략:', e)
print(f'{"높이대":4s} {"설명":30s} {"빔%":>5s} | S0 그대로: 중앙/p95/최대 cm · 각도최대 | S1 36 cm 튐: 최대 cm · 복귀')
for b, desc in BANDS.items():
    out = []
    for sc in ('S0', 'S1'):
        p = os.path.join(D, f'amcl_{sc}_tuned' + ('' if b in ('all', '') else f'_{b}'), 'poses.csv')
        if not os.path.exists(p):
            out.append(None); continue
        rows = list(csv.DictReader(open(p)))
        od = np.array([(float(r['t']), float(r['x']), float(r['y']), float(r['yaw'])) for r in rows if r['kind'] == 'odom'])
        am = np.array([(float(r['t']), float(r['x']), float(r['y']), float(r['yaw'])) for r in rows if r['kind'] == 'amcl'])
        if len(am) == 0:
            out.append(None); continue
        C = [(a[0], *comp(a[1:], inv(od[int(np.clip(np.searchsorted(od[:, 0], a[0]), 0, len(od) - 1)), 1:]))) for a in am]; C = np.array(C)
        cor = Corrupt(sc); E = []
        for t in ts:
            k = np.searchsorted(C[:, 0], t) - 1
            if k < 0:
                continue
            x, y, a = true_at(t); Pc = np.array(cor.apply(t - t0, x, y, a)); est = comp(C[k, 1:], Pc); tr = comp(T_ST, true_at(t))
            E.append((t - t0, math.hypot(est[0] - tr[0], est[1] - tr[1]), abs(math.degrees((est[2] - tr[2] + math.pi) % (2 * math.pi) - math.pi))))
        E = np.array(E); rec = None
        if sc == 'S1':
            ok = (E[:, 0] >= 100) & (E[:, 1] < 0.10); rec = E[ok, 0][0] - 100 if ok.any() else None
        out.append((np.median(E[:, 1]) * 100, np.percentile(E[:, 1], 95) * 100, E[:, 1].max() * 100, E[:, 2].max(), rec))
    s0 = f'{out[0][0]:5.1f} /{out[0][1]:5.1f} /{out[0][2]:5.1f} · {out[0][3]:4.1f}°' if out[0] else '   (없음)              '
    s1 = (f'{out[1][2]:5.1f} · ' + (f'{out[1][4]:.0f}초' if out[1][4] is not None else '복귀 못 함')) if out[1] else '(없음)'
    print(f'{b:5s} {desc:30s} {(cover or {}).get(b, float("nan")) * 100:5.0f} | {s0}        | {s1}')
