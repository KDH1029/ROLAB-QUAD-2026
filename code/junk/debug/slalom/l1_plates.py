#!/usr/bin/env python3
"""L1 로 콘 받침대 찾기 + 지금 경로가 밟는지 검사.
L1(/utlidar/cloud_base, 몸 좌표, 몸 앞 ~0.7 m 바닥까지) → 16:50 녹화 재생 FAST-LIO 자세(debug/loc/out/r1650/dump.npz, rec1650 지도 좌표와 5 cm 안)로 지도에.
받침대 후보 = 바닥(0.25 m 칸별 하위 5%) 위 0.025~0.07 m 점이 4 cm 칸에 6개 이상(로봇 자기 다리 영역 뺌), 콘 몸통 = 0.10 m 위. XT16 지도(편집기 벽)와 비교.
발 = 몸 중심 기준 (±0.19, ±0.14) m (Go2 엉덩이 위치) — S자 시뮬레이션(V0 지금 설정) 궤적의 네 발 자리가 받침대 칸(4 cm, 1칸 넓힘)에 들어가면 '밟음'.
사용(ROS Jazzy): python3 l1_plates.py   → l1_plates.png, l1_plates.json"""
import json
import math
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from scipy import ndimage as ndi
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2

H = os.path.dirname(os.path.abspath(__file__)); J = os.path.expanduser('~/ws/junk')
sys.path.insert(0, os.path.join(J, 'debug', 'loc')); import icp_core as ic  # noqa: E402
dump = ic.Dump(os.path.join(J, 'debug', 'loc', 'out', 'r1650', 'dump.npz'))
R = json.load(open(os.path.join(J, 'routes', 'rec1650', 'route.json')))['route']; RP = np.array([[e['x'], e['y']] for e in R])
SL = RP[6:25]; ZLO, ZHI = SL.min(0) - 0.6, SL.max(0) + 0.6
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=os.path.expanduser('~/bags/gosu_20261105_165023_rec'), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/utlidar/cloud_base', '/lidar_points']))
# 시각 맞추기: /lidar_points 의 (녹화 시각 − 헤더) 로 NUC 지연, L1 은 녹화 시각 − 그 지연으로 FAST-LIO 자세를 찾음
pts = []; lat = []; n_l1 = 0; hdr_off = []
while r.has_next():
    tp, data, trec = r.read_next(); trec *= 1e-9
    m = deserialize_message(data, PointCloud2); st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    if tp == '/lidar_points':
        lat.append(trec - st); continue
    n_l1 += 1; hdr_off.append(trec - st)
    t = trec - (np.median(lat) if lat else 0.1)
    if t < dump.ot[0] or t > dump.ot[-1]:
        continue
    a = np.frombuffer(bytes(m.data), np.uint8).reshape(-1, m.point_step)
    xyz = np.stack([a[:, o:o + 4].copy().view(np.float32)[:, 0] for o in (0, 4, 8)], 1).astype(float); xyz = xyz[np.isfinite(xyz).all(1)]
    xyz = xyz[(xyz[:, 0] > 0.50) | (np.abs(xyz[:, 1]) > 0.28) | (xyz[:, 0] < -0.45)]     # 로봇 자기 다리·발(몸 앞 0.5 m·옆 0.28 m 안) 빼기
    P = dump.pose_at(t); w = (P[:3, :3] @ xyz.T).T + P[:3, 3]
    k = (w[:, 0] > ZLO[0]) & (w[:, 0] < ZHI[0]) & (w[:, 1] > ZLO[1]) & (w[:, 1] < ZHI[1])
    pts.append(w[k])
W = np.concatenate(pts)
print(f'L1 메시지 {n_l1} · XT16 지연 중앙 {np.median(lat) * 1000:.0f} ms · L1 (녹화−헤더) 중앙 {np.median(hdr_off):.2f} s · 슬라럼 구역 L1 점 {len(W)}')
key = np.floor(W[:, :2] / 0.25).astype(int); u, inv = np.unique(key, axis=0, return_inverse=True); inv = inv.ravel()
fl = np.zeros(len(u))
for i in range(len(u)):
    fl[i] = np.percentile(W[inv == i, 2], 5)
hgt = W[:, 2] - fl[inv]
plate = W[(hgt > 0.025) & (hgt < 0.07)]; body = W[(hgt >= 0.10) & (hgt < 0.8)]
# 받침대 칸: 4 cm 칸에 낮은 점 3개 이상, 근처(0.35 m)에 콘 몸통이 있는 것(바닥 요철·발 제외)
cell = np.floor(plate[:, :2] / 0.04).astype(int); cu, cc = np.unique(cell, axis=0, return_counts=True); cu = cu[cc >= 6]
cxy = cu * 0.04 + 0.02; TB = cKDTree(body[:, :2]) if len(body) else None
near_cone = np.array([len(TB.query_ball_point(p, 0.35)) >= 20 for p in cxy]) if TB is not None else np.zeros(len(cxy), bool)
PL = cxy[near_cone]
print(f'받침대 후보 칸(4 cm) {len(cxy)} · 콘 몸통 0.35 m 안 {len(PL)}')
# XT16 지도(편집기 벽)에 그 받침대가 있나
sys.argv = ['x', 'rec1650']; sys.path.insert(0, os.path.join(J, 'tools')); import route_editor as ed  # noqa: E402
XW = ed.real_walls(ed.load_cache()); TX = cKDTree(XW)
dx = TX.query(PL)[0] if len(PL) else np.array([])
print(f'받침대 칸 중 XT16 지도 벽에서 4 cm 넘게 떨어진(지도에 없는) 비율 {np.mean(dx > 0.04) * 100 if len(dx) else 0:.0f}%')
# 발 자국 검사: 지금 설정 시뮬레이션 궤적(V0)
sys.path.insert(0, H)
res = {}
try:
    import subprocess  # noqa: F401
    tr = np.load(os.path.join(H, 'sim_scurve_V0_trace.npy'))
except OSError:
    tr = None
TP = cKDTree(PL) if len(PL) else None
FEET = [(0.19, 0.14), (0.19, -0.14), (-0.19, 0.14), (-0.19, -0.14)]
steps = []
if tr is not None and TP is not None:
    for row in tr[::2]:
        x, y, a = row[1], row[2], row[3]; c, s = math.cos(a), math.sin(a)
        for fx, fy in FEET:
            q = (x + c * fx - s * fy, y + s * fx + c * fy)
            dd, j = TP.query(q)
            if dd < 0.05:
                steps.append((row[0], q[0], q[1], dd))
    print(f'지금 설정(V0) 궤적: 발이 받침대 칸 5 cm 안을 지난 기록 {len(steps)}개')
    if steps:
        S = np.array(steps); near = [int(np.argmin(np.hypot(*(RP - s_[1:3]).T))) + 1 for s_ in S]
        print('   가까운 경로 점:', sorted(set(near)))
        res['near_points'] = sorted(set(near))
# 경로 점 자체(몸 중심) 둘레 발 반경 0.24 m 안 받침대
if TP is not None:
    for i in range(5, 26):
        cnt = len(TP.query_ball_point(RP[i], 0.24))
        if cnt:
            res.setdefault('route_points_with_plate_within_0.24', []).append((i + 1, cnt))
    print('경로 점 0.24 m 안에 받침대 칸이 있는 점(번호, 칸 수):', res.get('route_points_with_plate_within_0.24', []))
json.dump(dict(n_plate_cells=int(len(PL)), plate_xy=PL.round(3).tolist(), **res), open(os.path.join(H, 'l1_plates.json'), 'w'), ensure_ascii=False)
try:
    import matplotlib; matplotlib.use('Agg'); matplotlib.rcParams['font.family'] = 'NanumGothic'; matplotlib.rcParams['axes.unicode_minus'] = False
    import matplotlib.pyplot as plt
    f, ax = plt.subplots(figsize=(8, 12))
    kb = (body[:, 0] > ZLO[0]) & (body[:, 1] > ZLO[1]); ax.scatter(body[kb, 0], body[kb, 1], s=0.3, c='0.75', label='L1 콘 몸통(0.10 m 위)')
    kx = np.all((XW > ZLO) & (XW < ZHI), 1); ax.scatter(XW[kx, 0], XW[kx, 1], s=4, c='k', marker='s', label='XT16 지도 벽(경로 편집기)')
    if len(PL):
        ax.scatter(PL[:, 0], PL[:, 1], s=10, c='orange', label='L1 받침대 후보(바닥 위 1.5~8 cm)')
    ax.plot(RP[5:26, 0], RP[5:26, 1], 'b.-', lw=1, ms=6, label='경로')
    for i in range(5, 26, 2):
        ax.annotate(f'#{i + 1}', RP[i], fontsize=8, color='b')
    if tr is not None:
        ax.plot(tr[:, 1], tr[:, 2], 'g-', lw=1, label='지금 설정 시뮬레이션 궤적')
    if steps:
        S = np.array(steps); ax.scatter(S[:, 1], S[:, 2], s=25, facecolors='none', edgecolors='r', label='발이 받침대 위')
    ax.set_xlim(ZLO[0], ZHI[0]); ax.set_ylim(ZLO[1], ZHI[1]); ax.set_aspect('equal'); ax.grid(alpha=.3); ax.legend(loc='lower left', fontsize=8)
    ax.set_title('슬라럼: L1 로 본 콘 받침대 vs 경로')
    f.savefig(os.path.join(H, 'l1_plates.png'), dpi=75, bbox_inches='tight')
except Exception as e:  # noqa: BLE001
    print('그림 실패', e)
