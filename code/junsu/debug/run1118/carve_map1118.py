#!/usr/bin/env python3
"""rec1118 지도에서 '유령'(사람·잠깐 있던 물체) 지우기 — 2D 빈 공간 깎기.
스캔마다 라이다 위치 → 점(바닥 위 5~60 cm) 광선을 3 cm 칸으로 따라가며 '지나감' 수, 끝 칸 '맞음' 수를 센다.
나중에 광선이 뚫고 지나간 칸(지나감 많고 맞음 비율 낮음) = 유령 → 그 칸 위 복셀을 map.npz 에서 뺀다(원본은 map_raw.npz 로 둠)."""
import rosbag2_py, numpy as np, os, shutil
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation as Rot
B = '/home/ilovehan9283/bags/gosu_20261106_111853_cone1106'; D = os.path.expanduser('~/ws/junsu_v2/maps/rec1118')
Rbl = np.array([[-0.0001707, -0.9999240, 0.0123287], [0.9999844, -0.0001019, 0.0055807], [-0.0055790, 0.0123295, 0.9999084]]); Tbl = np.array([0.25, 0.0, 0.1588])
od = np.load(D + '/map.npz')['odom']; ts = od[:, 0]
C = 0.03; X0, Y0, NX, NY = -4.0, -4.0, 480, 400          # 칸 격자(−4~10.4, −4~8)
hit = np.zeros((NX, NY), np.int32); pas = np.zeros((NX, NY), np.int32)
PC = get_message('sensor_msgs/msg/PointCloud2')
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=B, storage_id='mcap'), rosbag2_py.ConverterOptions('', '')); r.set_filter(rosbag2_py.StorageFilter(topics=['/lidar_points']))
n = 0; used = 0; steps = np.arange(0.06, 1.0, 1e-9)
while r.has_next():
    tp, raw, t = r.read_next(); n += 1
    if n % 3: continue
    m = deserialize_message(raw, PC); st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9; k = int(np.argmin(np.abs(ts - st)))
    if abs(ts[k] - st) > 0.06: continue
    f = {x.name: x.offset for x in m.fields}; a = np.frombuffer(m.data, np.uint8).reshape(-1, m.point_step)
    P = np.stack([a[:, f[c]:f[c] + 4].copy().view('<f4')[:, 0] for c in 'xyz'], 1)[::3]; P = P[np.isfinite(P).all(1)]
    Rw = Rot.from_quat(od[k, 5:9]).as_matrix(); Pw = (P @ Rbl.T + Tbl) @ Rw.T + od[k, 2:5]; O = Tbl @ Rw.T + od[k, 2:5]
    zf = od[k, 4] - 0.30                                   # 바닥 ≈ 몸통 IMU − 0.30 m
    h = Pw[:, 2] - zf; d2 = np.hypot(Pw[:, 0] - O[0], Pw[:, 1] - O[1]); s = (h > 0.05) & (h < 0.60) & (d2 > 0.3) & (d2 < 6.0); Pw = Pw[s]; d2 = d2[s]
    if not len(Pw): continue
    ci = np.floor((Pw[:, 0] - X0) / C).astype(int); cj = np.floor((Pw[:, 1] - Y0) / C).astype(int); ok = (ci >= 0) & (ci < NX) & (cj >= 0) & (cj < NY)
    np.add.at(hit, (ci[ok], cj[ok]), 1)
    # 광선: 라이다 위치 → 끝점 앞 6 cm 까지, 3 cm 간격
    ns = (d2 / C).astype(int); mx = ns.max(); fr = (np.arange(mx)[None, :] * C) / d2[:, None]
    valid = fr < (1.0 - 0.06 / d2[:, None]); fx = O[0] + fr * (Pw[:, 0] - O[0])[:, None]; fy = O[1] + fr * (Pw[:, 1] - O[1])[:, None]
    gi = np.floor((fx[valid] - X0) / C).astype(int); gj = np.floor((fy[valid] - Y0) / C).astype(int); ok = (gi >= 0) & (gi < NX) & (gj >= 0) & (gj < NY)
    cells = np.unique(gi[ok] * NY + gj[ok]); np.add.at(pas, (cells // NY, cells % NY), 1)
    used += 1
print('스캔', used)
ghost = (hit >= 1) & (pas >= 6) & (hit < 0.6 * pas)      # 맞은 것보다 뚫고 지나간 게 훨씬 많음
wall = (hit >= 3) & ~ghost
print(f'맞은 칸 {(hit >= 1).sum()} · 유령 칸 {ghost.sum()} · 벽 칸 {wall.sum()}')
d = dict(np.load(D + '/map.npz')); V = float(d['V'])
if not os.path.exists(D + '/map_raw.npz'): shutil.copy2(D + '/map.npz', D + '/map_raw.npz')
P = (d['vox'] + 0.5) * V; ci = np.floor((P[:, 0] - X0) / C).astype(int); cj = np.floor((P[:, 1] - Y0) / C).astype(int)
inb = (ci >= 0) & (ci < NX) & (cj >= 0) & (cj < NY); g = np.zeros(len(P), bool); g[inb] = ghost[ci[inb], cj[inb]]
# 바닥 복셀은 남김(편집기 바닥 계산용): 유령 칸 위 '바닥 위 5 cm 넘는' 복셀만 뺌 — 높이는 근처 궤적 z − 0.30 으로
from scipy.spatial import cKDTree
tz = cKDTree(od[:, 2:4]); _, j = tz.query(P[:, :2]); hf = P[:, 2] - (od[j, 4] - 0.30); drop = g & (hf > 0.05)
d['vox'] = d['vox'][~drop]; d['cnt'] = d['cnt'][~drop]; np.savez_compressed(D + '/map.npz', **d)
np.savez_compressed(D + '/carve_grid.npz', hit=hit, pas=pas, ghost=ghost, X0=X0, Y0=Y0, C=C)
print(f'복셀 {len(P)} → {len(d["vox"])} (뺀 것 {drop.sum()}) · 원본 map_raw.npz')
