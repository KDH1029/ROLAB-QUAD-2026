#!/usr/bin/env python3
"""KISS-ICP(1.3.0, MIT, pip — 야간 venv) 라이다 단독 오도메트리 평가 — IMU 를 안 쓰므로 계단 충격에 FAST-LIO 처럼 튀는지 본다.
입력: 10-30 녹화본 XT16 /lidar_points (점별 timestamp 로 deskew). 기준: FAST-LIO 궤적(gosu1030v2_map.npz).
출력: 시간대별 xy 오차, 끝점 오차, 이동거리 대비 %, 처리 시간. 쓰기: 이 폴더 kiss_eval.json, kiss_traj.npy
"""
import sys, time, bisect, json, math, numpy as np, rosbag2_py
from scipy.spatial.transform import Rotation as Rot
sys.path.insert(0, '/home/ilovehan9283/ws/v_gosu/backup')
from backup_core import q2R, R_LIDAR, T_LIDAR
from kiss_icp.kiss_icp import KissICP
from kiss_icp.config import KISSConfig
BAG = '/home/ilovehan9283/bags/gosu_20261030_001126'
NPZ = '/home/ilovehan9283/ws/v_gosu/eval/out/gosu1030v2_map.npz'
VOX = float(sys.argv[1]) if len(sys.argv) > 1 else 0.15
MAXR = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0


def reader(topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics)); return r


d = np.load(NPZ); odom, lid = d['odom'], d['lid']
hdr2bag = {}; r = reader(['/lidar_points'])
while r.has_next():
    tp, raw, t = r.read_next(); s, ns = np.frombuffer(raw[4:8], '<i4')[0], np.frombuffer(raw[8:12], '<u4')[0]
    hdr2bag[round(float(s) + float(ns) * 1e-9, 4)] = t * 1e-9
pairs = np.array([(w, hdr2bag[round(h, 4)]) for w, h in lid if round(h, 4) in hdr2bag])
Kp = float(np.median(pairs[:, 0] - pairs[:, 1])); wl = np.sort(lid[:, 0]); h_o = odom[:, 1]
dmed = float(np.median(h_o - wl[np.clip(np.searchsorted(wl, h_o) - 1, 0, len(wl) - 1)]))
tL = h_o - dmed - Kp; o = np.argsort(tL); tL = tL[o]; GT = odom[o]; t0 = tL[0]
def gt_at(ts): return GT[min(max(bisect.bisect_left(tL, ts), 0), len(tL) - 1)]

cfg = KISSConfig(); cfg.data.max_range = MAXR; cfg.data.min_range = 0.6; cfg.data.deskew = True; cfg.mapping.voxel_size = VOX
kiss = KissICP(cfg)
T_bl = np.eye(4); T_bl[:3, :3] = R_LIDAR; T_bl[:3, 3] = T_LIDAR; T_lb = np.linalg.inv(T_bl)
dt = np.dtype({'names': ['x', 'y', 'z', 'ts'], 'formats': ['<f4', '<f4', '<f4', '<f8'], 'offsets': [0, 4, 8, 18], 'itemsize': 26})
rows = []; ms = []; T0w = None
r = reader(['/lidar_points'])
while r.has_next():
    tp, raw, t = r.read_next(); t *= 1e-9
    if t < tL[0] or t > tL[-1]: continue
    # CDR: 헤더 뒤 PointCloud2 data 는 메시지 끝 쪽 — 역직렬화 대신 rclpy 로 (정확성 우선)
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    m = deserialize_message(raw, PointCloud2)
    a = np.frombuffer(bytes(m.data), dtype=dt, count=m.width * m.height)
    ok = np.isfinite(a['x']) & np.isfinite(a['y']) & np.isfinite(a['z'])
    P = np.c_[a['x'][ok], a['y'][ok], a['z'][ok]].astype(np.float64); ts = a['ts'][ok]
    if len(P) < 500: continue
    tsn = (ts - ts.min()) / max(ts.max() - ts.min(), 1e-6)
    _t = time.perf_counter(); kiss.register_frame(P, tsn); ms.append((time.perf_counter() - _t) * 1e3)
    Tk = kiss.last_pose                                   # 라이다 좌표, 첫 프레임 기준
    g = gt_at(t)
    if T0w is None:
        T0w = np.eye(4); T0w[:3, :3] = q2R(*g[5:9]); T0w[:3, 3] = g[2:5]                   # 첫 순간 FAST-LIO 몸체 자세에 맞춤
    Tw = T0w @ T_bl @ Tk @ T_lb                              # 몸체 자세 (세계)
    rows.append((t - t0, Tw[0, 3], Tw[1, 3], Tw[2, 3], g[2], g[3], g[4]))
T = np.array(rows); np.save('kiss_traj.npy', T)
e = np.hypot(T[:, 1] - T[:, 4], T[:, 2] - T[:, 5]); ez = np.abs(T[:, 3] - T[:, 6])
dist = float(np.sum(np.hypot(np.diff(T[:, 4]), np.diff(T[:, 5]))))
out = dict(복셀=VOX, 최대거리=MAXR, 프레임=len(T), 처리ms_중앙=round(float(np.median(ms)), 1), 처리ms_p95=round(float(np.percentile(ms, 95)), 1),
           xy오차_중앙=round(float(np.median(e)), 3), p95=round(float(np.percentile(e, 95)), 3), 최대=round(float(e.max()), 3), 끝=round(float(e[-1]), 3),
           이동거리=round(dist, 1), 끝오차_이동거리비=round(float(e[-1] / dist * 100), 2), z오차_p95=round(float(np.percentile(ez, 95)), 3))
for a_ in range(0, 545, 60):
    m_ = (T[:, 0] >= a_) & (T[:, 0] < a_ + 60)
    if m_.any(): out[f'{a_}~{a_ + 60}s_최대'] = round(float(e[m_].max()), 3)
print(json.dumps(out, ensure_ascii=False, indent=1)); json.dump(out, open(f'kiss_eval_v{VOX}.json', 'w'), ensure_ascii=False, indent=1)
