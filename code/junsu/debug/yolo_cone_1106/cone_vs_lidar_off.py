#!/usr/bin/env python3
"""YOLO 콘 상자 → 몸체 xy (로봇 노드와 같은 계산) 를 옛 장착값(11-04, 아래 34°)·새 값(16:50 라이다 정합)으로 각각 구해
같은 순간 XT16 점(바닥 위 5~60 cm)과의 거리를 잰다. 콘 앞면 → 라이다 최근접 점 거리가 작을수록 맞다."""
import sys, os, json, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as Rot
sys.path.insert(0, os.path.expanduser('~/ws/junsu_v2/onboard_vision')); import cone_cam_core as K
HERE = os.path.dirname(os.path.abspath(__file__)); B = os.path.expanduser('~/bags/gosu_20261105_165023_rec')
d = json.load(open(B + '/yolo_1106/dets.json')); cc = d['cone_cls']; Kc = np.array(json.load(open(HERE + '/Kc.json')))
N = json.load(open(HERE + '/T_body_cam_1650.json')); Rn, tn = np.array(N['R']), np.array(N['t'])
Ro, to = K.T_BODY_CAM[:, :3], K.T_BODY_CAM[:, 3]
def body(u, v, z, R, t):
    pc = np.array([(u - Kc[0, 2]) / Kc[0, 0] * z, (v - Kc[1, 2]) / Kc[1, 1] * z, z]) - np.array(K.DEPTH_TO_COLOR_T); return R @ pc + t
rows = [r for r in d['rows'] if any(x[0] == cc and x[1] >= 0.45 and x[6] is not None and x[6] <= 3.5 for x in r['dets'])]
# 라이다: 받은 시각(NUC) 기준
PC = get_message('sensor_msgs/msg/PointCloud2'); r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=B, storage_id='mcap'), rosbag2_py.ConverterOptions('', '')); r.set_filter(rosbag2_py.StorageFilter(topics=['/lidar_points']))
Rl = Rot.from_euler('z', 90, degrees=True).as_matrix(); tl = np.array([0.25, 0, 0]); LT = []; LP = []
need = np.array([x['stamp'] + o for x in rows if 38 <= x['t'] < 72 for o in (-0.2,-0.1,-0.05,0,0.05,0.1,0.2)]); lo, hi = need.min() - 1, need.max() + 1
while r.has_next():
    tp, raw, tb = r.read_next(); tt = tb * 1e-9
    if tt < lo: continue
    if tt > hi: break
    if not len(need) or np.min(np.abs(need - tt)) > 0.06: continue
    m = deserialize_message(raw, PC); f = {x.name: x.offset for x in m.fields}; a = np.frombuffer(m.data, np.uint8).reshape(-1, m.point_step)
    xyz = np.stack([a[:, f[k]:f[k] + 4].copy().view('<f4')[:, 0] for k in 'xyz'], 1); xyz = xyz[np.isfinite(xyz).all(1)] @ Rl.T + tl
    LT.append(tt - 0.05); LP.append(xyz)      # XT16 한 바퀴 0.1 s → 가운데
LT = np.array(LT); print('라이다 프레임', len(LT), '카메라 콘 프레임', len(rows), flush=True)
SL = [x for x in rows if 38 <= x['t'] < 72]
for OFF in (-0.2,-0.1,-0.05,0.0,0.05,0.1,0.2):
    res = []
    for x in SL:
        j = int(np.argmin(np.abs(LT - (x['stamp']+OFF))))
        if abs(LT[j] - (x['stamp']+OFF)) > 0.06: continue
        P = LP[j]; rr = np.hypot(P[:, 0], P[:, 1])
        # 바닥 높이: 몸체 z 낮은 쪽 분포에서(로봇 앞 0.5~3 m)
        sel = (rr > 0.5) & (rr < 3.5); fl = np.percentile(P[sel, 2], 5) if sel.sum() > 50 else -0.31
        Q = P[sel & (P[:, 2] > fl + 0.05) & (P[:, 2] < fl + 0.75)]
        if len(Q) < 5: continue
        T = cKDTree(Q[:, :2])
        for c, s, x1, y1, x2, y2, z in x['dets']:
            if c != cc or s < 0.45 or z is None or z > 3.5: continue
            u, v = (x1 + x2) / 2, (y1 + y2) / 2; po = body(u, v, z, Ro, to); pn = body(u, v, z, Rn, tn)
            do, _ = T.query(po[:2]); dn, _ = T.query(pn[:2])
            res.append((x['t'], s, z, do, dn, po[:2].tolist(), pn[:2].tolist(), float(np.hypot(*(po[:2] - pn[:2])))))
    R_ = np.array([[a[0],a[1],a[2],a[3],a[4]] for a in res]); s = R_[:,2] < 2.0
    print(f'OFF {OFF:+.2f}s n={int(s.sum())} 옛 {np.median(R_[s,3])*100:.0f}cm 새 {np.median(R_[s,4])*100:.0f}cm', flush=True)
