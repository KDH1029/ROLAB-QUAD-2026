#!/usr/bin/env python3
"""백업 위치추정 공정 검증 — 지도는 녹화 앞부분, 시험은 뒷부분 (같은 주행으로 지도·시험을 함께 쓰던 낙관 제거).

지도: 앞부분(0~SPLIT초) XT16 원본 스캔을 '이미 기록된' FAST-LIO 자세로 이어 붙임 (FAST-LIO 를 다시 돌리지 않음)
      → 전체 FAST-LIO 지도와 겹침 비율로 재구성 품질을 먼저 확인
시험: 뒷부분(SPLIT초~끝)을 그 지도로 BackupEstimator (v_gosu/backup/backup_core.py, 읽기만) 실행, FAST-LIO 대비 오차
읽기만: ~/bags/gosu_20261030_001126, ~/ws/v_gosu/eval/out/gosu1030v2_map.npz, ~/ws/v_gosu/backup/backup_core.py
쓰기:   이 폴더(out_*.json/npz)
"""
import sys, math, bisect, json, numpy as np, rosbag2_py
from scipy.spatial import cKDTree
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
sys.path.insert(0, '/home/ilovehan9283/ws/v_gosu/backup')
from backup_core import BackupEstimator, load_map, q2R, rpy_of, R_LIDAR, T_LIDAR, voxel, wrap
import time
sys.path.insert(0, '/home/ilovehan9283/ws/night_20261104/backup_eval')
from gicp_matcher import GicpMatcher
from robust_ground import RobustGroundGrid

SPLIT = float(sys.argv[1]) if len(sys.argv) > 1 else 270.0
HALF_NPZ = sys.argv[2] if len(sys.argv) > 2 else None   # FAST-LIO 로 만든 앞부분 지도(.npz) — 주면 손 재구성 대신 이걸 쓴다
BAG = '/home/ilovehan9283/bags/gosu_20261030_001126'
NPZ = '/home/ilovehan9283/ws/v_gosu/eval/out/gosu1030v2_map.npz'
OUT = '/home/ilovehan9283/ws/night_20261104/backup_eval'


def reader(topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics)); return r


d = np.load(NPZ); odom, lid = d['odom'], d['lid']
FULL = (d['vox'][d['cnt'] >= 2].astype(np.float64) + 0.5) * float(d['V'])
hdr2bag = {}; frames = []
r = reader(['/lidar_points'])
while r.has_next():
    tp, raw, t = r.read_next(); s, ns = np.frombuffer(raw[4:8], '<i4')[0], np.frombuffer(raw[8:12], '<u4')[0]
    hdr2bag[round(float(s) + float(ns) * 1e-9, 4)] = t * 1e-9
pairs = np.array([(w, hdr2bag[round(h, 4)]) for w, h in lid if round(h, 4) in hdr2bag])
Kp = float(np.median(pairs[:, 0] - pairs[:, 1])); wl = np.sort(lid[:, 0]); h_o = odom[:, 1]
dmed = float(np.median(h_o - wl[np.clip(np.searchsorted(wl, h_o) - 1, 0, len(wl) - 1)]))
tL = h_o - dmed - Kp; o = np.argsort(tL); tL = tL[o]; GT = odom[o]
t0 = tL[0]
def gt_at(ts): return GT[min(max(bisect.bisect_left(tL, ts), 0), len(tL) - 1)]

HALF = load_map(HALF_NPZ)
dist, _ = cKDTree(FULL).query(HALF[::5], k=1)
recon = dict(앞부분_초=SPLIT, 지도=HALF_NPZ.split('/')[-1], 지도점=int(len(HALF)), 전체지도와_5cm안=round(float((dist < 0.05).mean()), 3), _10cm안=round(float((dist < 0.10).mean()), 3))
print('① 앞부분 지도(FAST-LIO)', recon)
# ── ② 뒷부분 시험: 앞부분 지도 vs 전체 지도 (같은 구간, 같은 초기화)
ev = []
r = reader(['/utlidar/robot_odom', '/lidar_points']); last = -1e9
while r.has_next():
    tp, raw, t = r.read_next(); t *= 1e-9
    if t < t0 + SPLIT - 2: continue
    if tp == '/utlidar/robot_odom':
        m = deserialize_message(raw, Odometry); p, q = m.pose.pose.position, m.pose.pose.orientation
        rr_, pp, yy = rpy_of(q2R(q.x, q.y, q.z, q.w)); ev.append((t, 'leg', (p.x, p.y, p.z, yy, rr_, pp)))
    elif t - last >= 0.1 and t < tL[-1]:
        last = t; m = deserialize_message(raw, PointCloud2)
        ev.append((t, 'scan', pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)))
ev.sort(key=lambda e: e[0])
res = {}
import os
VARIANTS = [v.split(':') for v in os.environ.get('VARIANTS', 'g08:0.08:0.2:1:30:0').split(',')]   # 이름:rms_ok:정합간격s:수렴필요:반복:견고지면
for name, rmsok, dt_scan, needc, iters, rg in VARIANTS:
    rmsok, dt_scan = float(rmsok), float(dt_scan); mp = HALF
    est = BackupEstimator(mp, rms_ok=rmsok); leg = None; tr = []; ms = []; lastsc = -1e9
    est.matcher = GicpMatcher(voxel(mp, 0.05), need_converged=(needc == '1'), iters=int(iters))
    if rg == '1': est.ground = RobustGroundGrid(voxel(mp, 0.05))
    for t, kind, val in ev:
        if kind == 'leg':
            leg = val
            if est.inited: est.on_leg(val)
            continue
        if leg is None or t < t0 + SPLIT or t - lastsc < dt_scan - 1e-3: continue
        lastsc = t
        if not est.inited:
            g = gt_at(t); est.init(g[2], g[3], g[4], rpy_of(q2R(*g[5:9]))[2], leg, t)
        _t = time.perf_counter(); out = est.on_scan(val, t); ms.append((time.perf_counter() - _t) * 1000); x, y, *_ = est.pose(); g = gt_at(t)
        tr.append((t - t0, math.hypot(x - g[2], y - g[3]), out['rms'] if out else np.nan, g[4], est.z, math.degrees(wrap(est.yaw - rpy_of(q2R(*g[5:9]))[2]))))
    T = np.array(tr); e = T[:, 1]; hi = (T[:, 3] - np.median(GT[:50, 4])) > 0.10
    res[name] = dict(정합=len(T), 수락=est.n_ok, 거부=est.n_bad, 오차_중앙=round(float(np.median(e)), 3), p95=round(float(np.percentile(e, 95)), 3),
                     최대=round(float(e.max()), 3), 경사위_p95=round(float(np.percentile(e[hi], 95)), 3) if hi.any() else None,
                     잔차_중앙=round(float(np.nanmedian(T[:, 2])), 3), 정합ms_중앙=round(float(np.median(ms)), 1), 정합ms_p95=round(float(np.percentile(ms, 95)), 1), 미수렴=est.matcher.n_noconv, 지면유지=getattr(est.ground, 'n_hold', None), 단차수락=getattr(est.ground, 'n_jump_ok', None), 지면모름=getattr(est.ground, 'n_unknown', None), z오차_p95=round(float(np.percentile(abs(T[:, 4] - T[:, 3]), 95)), 3), z오차_최대=round(float(abs(T[:, 4] - T[:, 3]).max()), 3))
    np.save(f'{OUT}/traj_gicp_{name}.npy', T)
    print('②', name, res[name])
json.dump(dict(재구성=recon, 시험=res), open(f'{OUT}/split_result_gicp_variants_{os.environ.get("TAG", "x")}.json', 'w'), ensure_ascii=False, indent=1)
