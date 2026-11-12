#!/usr/bin/env python3
"""backup_core 녹화본 시험 — 노드와 똑같은 계산(BackupEstimator, MuxLogic)을 ROS 없이 돌린다.

① 추정기: 10-30 녹화본에서 FAST-LIO 대비 오차 (11-03 오프라인 검증 수치와 같아야 한다)
② 전환 장치: 정상 주행(전환 없어야 함) / FAST-LIO 끊김 / 3 m 점프 / 서서히 미끄러짐(0.05 m/s)
사용: test_offline.py
"""
import sys, math, bisect, json, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
sys.path.insert(0, '/home/ilovehan9283/ws/v_gosu/backup')
from backup_core import BackupEstimator, MuxLogic, load_map, q2R, rpy_of

BAG = '/home/ilovehan9283/bags/gosu_20261030_001126'
NPZ = '/home/ilovehan9283/ws/v_gosu/eval/out/gosu1030v2_map.npz'
HZ = 5.0


def reader(topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics)); return r


# FAST-LIO 기준 궤적 (bag 수신 시각) — render_v2 와 같은 대응
d = np.load(NPZ); odom, lid = d['odom'], d['lid']
hdr2bag = {}
r = reader(['/lidar_points'])
while r.has_next():
    tp, raw, t = r.read_next(); s, ns = np.frombuffer(raw[4:8], '<i4')[0], np.frombuffer(raw[8:12], '<u4')[0]
    hdr2bag[round(float(s) + float(ns) * 1e-9, 4)] = t * 1e-9
pairs = np.array([(w, hdr2bag[round(h, 4)]) for w, h in lid if round(h, 4) in hdr2bag])
Kp = float(np.median(pairs[:, 0] - pairs[:, 1])); wl = np.sort(lid[:, 0]); h_o = odom[:, 1]
dmed = float(np.median(h_o - wl[np.clip(np.searchsorted(wl, h_o) - 1, 0, len(wl) - 1)]))
tL = h_o - dmed - Kp; o = np.argsort(tL); tL = tL[o]; GT = odom[o]
def gt_at(ts): return GT[min(max(bisect.bisect_left(tL, ts), 0), len(tL) - 1)]

est = BackupEstimator(load_map(NPZ))
# 시간순 사건: 다리 오도(150 Hz) + 스캔(5 Hz)
ev = []
r = reader(['/utlidar/robot_odom', '/lidar_points']); last_scan = -1e9
while r.has_next():
    tp, raw, t = r.read_next(); t *= 1e-9
    if tp == '/utlidar/robot_odom':
        m = deserialize_message(raw, Odometry); p, q = m.pose.pose.position, m.pose.pose.orientation
        rr, pp, yy = rpy_of(q2R(q.x, q.y, q.z, q.w)); ev.append((t, 'leg', (p.x, p.y, p.z, yy, rr, pp)))
    elif t - last_scan >= 1.0 / HZ and tL[0] + 1 < t < tL[-1]:
        last_scan = t; m = deserialize_message(raw, PointCloud2)
        ev.append((t, 'scan', pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)))
ev.sort(key=lambda e: e[0])

traj = []; leg_now = None
for t, kind, val in ev:
    if kind == 'leg':
        leg_now = val
        if est.inited: est.on_leg(val)
        continue
    if leg_now is None: continue
    if not est.inited:
        g = gt_at(t); est.init(g[2], g[3], g[4], rpy_of(q2R(*g[5:9]))[2], leg_now, t)
    res = est.on_scan(val, t)
    x, y, z, _, pitch, yaw = est.pose(); g = gt_at(t)
    traj.append((t, x, y, z, yaw, g[2], g[3], g[4], math.degrees(pitch), res['rms'] if res else np.nan, est.health(t) == 'ok'))
np.save('/home/ilovehan9283/ws/v_gosu/backup/test_traj.npy', np.array(traj))
T = np.array(traj); e = np.hypot(T[:, 1] - T[:, 5], T[:, 2] - T[:, 6]); ez = np.abs(T[:, 3] - T[:, 7])
pit = np.abs(T[:, 8]) > 5; cen = np.hypot(T[:, 5] - 4.4, T[:, 6] + 1.6) < 1.2
S = {'추정기': dict(정합=len(T), 수락=est.n_ok, 거부=est.n_bad, 위치오차_중앙=round(float(np.median(e)), 3), p95=round(float(np.percentile(e, 95)), 3),
                 최대=round(float(e.max()), 3), 경사_p95=round(float(np.percentile(e[pit], 95)), 3), 가운데_p95=round(float(np.percentile(e[cen], 95)), 3),
                 높이오차_중앙=round(float(np.median(ez)), 3), 높이오차_p95=round(float(np.percentile(ez, 95)), 3), 건강ok비율=round(float(T[:, 10].mean()), 3))}
print('① 추정기', S['추정기'])

# ② 전환 장치 — FAST-LIO 20 Hz 흐름을 그대로 넣고, 시나리오별로 고장을 주입
bk_t = T[:, 0]
def bk_at(ts):
    i = min(max(bisect.bisect_right(bk_t, ts) - 1, 0), len(T) - 1); return (T[i, 1], T[i, 2]), ('ok' if T[i, 10] else 'degraded')
def run(name, fault):
    mux = MuxLogic(); sw = None
    for i, ts in enumerate(tL):
        xyz = fault(ts, GT[i, 2:5].copy())
        if xyz is not None: mux.on_fastlio(ts, *xyz)
        for k in range(4):     # 5 ms 단위 감시(20 Hz 사이)
            tt = ts + k * 0.0125; bxy, h = bk_at(tt)
            if mux.tick(tt, bxy, h) == 'backup' and sw is None: sw = (tt - tL[0], mux.reason)
        if sw: break
    return dict(시나리오=name, 전환=('없음' if sw is None else f'{sw[0]:.1f}초: {sw[1]}'))
t0 = tL[0]
scen = [run('정상 주행', lambda ts, p: p),
        run('FAST-LIO 끊김 (200초부터)', lambda ts, p: None if ts - t0 > 200 else p),
        run('FAST-LIO 3 m 점프 (300초)', lambda ts, p: p + np.array([3.0, 0, 0]) if ts - t0 > 300 else p),
        run('서서히 미끄러짐 0.05 m/s (150초부터)', lambda ts, p: p + np.array([0.05 * max(0, ts - t0 - 150), 0, 0]))]
for s in scen: print('②', s)
# 정상 주행에서 FAST-LIO 순간 속도·백업 차이의 최댓값 (오탐 여유)
v = np.hypot(np.diff(GT[:, 2]), np.diff(GT[:, 3])) / np.maximum(np.diff(tL), 1e-3)
dd = [math.hypot(GT[i, 2] - bk_at(ts)[0][0], GT[i, 3] - bk_at(ts)[0][1]) for i, ts in enumerate(tL[::5]) for i in [i * 5]]
S['오탐여유'] = dict(FAST_LIO_순간속도_최대=round(float(v.max()), 2), p999=round(float(np.percentile(v, 99.9)), 2), 백업차이_최대=round(float(max(dd)), 3))
print('  정상 주행 여유', S['오탐여유']); S['전환'] = scen
json.dump(S, open('/home/ilovehan9283/ws/v_gosu/backup/test_offline_result.json', 'w'), ensure_ascii=False, indent=1)
