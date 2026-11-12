#!/usr/bin/env python3
"""L1(/utlidar/cloud_base) 좌표계 점검 — 지도 표면(10 cm 칸 중앙 높이) 대비 L1 점 높이 오차를 '몸체 전체 회전' vs '요(yaw)만 회전' 두 가지로.
경사로를 오를 때 L1 점이 0.2 m 높게 찍히는 현상(eval_cliff 84 s) 원인 확인용."""
import sys, bisect, json, numpy as np, rosbag2_py, math
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
sys.path.insert(0, '/home/ilovehan9283/ws/v_gosu/backup')
from backup_core import q2R, rpy_of
BAG = '/home/ilovehan9283/bags/gosu_20261030_001126'; NPZ = '/home/ilovehan9283/ws/v_gosu/eval/out/gosu1030v2_map.npz'
def reader(topics):
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics)); return r
d = np.load(NPZ); odom, lid = d['odom'], d['lid']; FULL = (d['vox'][d['cnt'] >= 2].astype(np.float64) + 0.5) * float(d['V'])
hdr2bag = {}; r = reader(['/lidar_points'])
while r.has_next():
    tp, raw, t = r.read_next(); s, ns = np.frombuffer(raw[4:8], '<i4')[0], np.frombuffer(raw[8:12], '<u4')[0]; hdr2bag[round(float(s) + float(ns) * 1e-9, 4)] = t * 1e-9
pairs = np.array([(w, hdr2bag[round(h, 4)]) for w, h in lid if round(h, 4) in hdr2bag])
Kp = float(np.median(pairs[:, 0] - pairs[:, 1])); wl = np.sort(lid[:, 0]); h_o = odom[:, 1]
dmed = float(np.median(h_o - wl[np.clip(np.searchsorted(wl, h_o) - 1, 0, len(wl) - 1)]))
tL = h_o - dmed - Kp; o = np.argsort(tL); tL = tL[o]; GT = odom[o]; t0 = tL[0]
def gt_at(ts): return GT[min(max(bisect.bisect_left(tL, ts), 0), len(tL) - 1)]
G = 0.10; kk = np.floor(FULL[:, :2] / G).astype(np.int64)
od_ = np.lexsort((FULL[:, 2], kk[:, 1], kk[:, 0])); ks = kk[od_]; zs = FULL[od_, 2]
brk = np.r_[True, np.any(ks[1:] != ks[:-1], axis=1)]; st = np.where(brk)[0]; en = np.r_[st[1:], len(ks)]
MID = {tuple(ks[a]): float(np.median(zs[a:b])) for a, b in zip(st, en) if zs[b - 1] - zs[a] < 0.10 and b - a >= 3}   # 평평한 칸만(벽·물체 제외)
WINS = [(25, 35, '정지·평지'), (60, 70, '평지 걷기'), (80, 88, '경사로 오름'), (100, 115, '단 위 정지'), (130, 140, '단에서 내려감')]
r = reader(['/utlidar/cloud_base']); res = {w[2]: {'full': [], 'yaw': [], 'pitch': []} for w in WINS}; fid = None
while r.has_next():
    tp, raw, t = r.read_next(); t *= 1e-9; tr = t - t0
    w = [w for w in WINS if w[0] <= tr < w[1]]
    if not w: continue
    m = deserialize_message(raw, PointCloud2); fid = m.header.frame_id
    a = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64); a = a[np.linalg.norm(a, axis=1) > 0.2]
    g = gt_at(t); R = q2R(*g[5:9]); rr, pp, yy = rpy_of(R)
    Ry = np.array([[math.cos(yy), -math.sin(yy), 0], [math.sin(yy), math.cos(yy), 0], [0, 0, 1]])
    res[w[0][2]]['pitch'].append(math.degrees(pp))
    for nm, RR in (('full', R), ('yaw', Ry)):
        P = a @ RR.T + g[2:5]; dxy = np.hypot(P[:, 0] - g[2], P[:, 1] - g[3]); sel = (dxy > 0.4) & (dxy < 2.0)
        P = P[sel][::5]; k = np.floor(P[:, :2] / G).astype(np.int64)
        e = np.array([P[i, 2] - MID[tuple(k[i])] for i in range(len(P)) if tuple(k[i]) in MID])
        e = e[np.abs(e) < 0.3]
        if len(e) > 10: res[w[0][2]][nm].append(float(np.median(e)))
print('L1 frame_id:', fid)
for k, v in res.items():
    if v['full']: print(f"{k:10s} 피치 중앙 {np.median(v['pitch']):5.1f}°  지면 대비 L1 높이(부호) 중앙값: 전체회전 {np.median(v['full']):+.3f} m (분위 10/90 {np.percentile(v['full'],10):+.3f}/{np.percentile(v['full'],90):+.3f})  요만 {np.median(v['yaw']):+.3f} m  (프레임 {len(v['full'])})")
