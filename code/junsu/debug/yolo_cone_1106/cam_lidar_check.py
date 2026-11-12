#!/usr/bin/env python3
"""16:50 녹화 출발 정지 구간(로봇 안 움직임)에서 D435i 뎁스 점을 몸체로 옮겨 XT16 점과 겹쳐 본다.
T_BODY_CAM(11-04 보정, 아래 34°)이 이 녹화의 카메라 장착과 맞는지 → 안 맞으면 점-점 ICP 로 새 값 추정."""
import sys, os, json, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as Rot
sys.path.insert(0, os.path.expanduser('~/ws/junsu_v2/onboard_vision')); import cone_cam_core as K
B = os.path.expanduser('~/bags/gosu_20261105_165023_rec'); HERE = os.path.dirname(os.path.abspath(__file__))
T0, T1 = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0, float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
Img, Info, PC = get_message('sensor_msgs/msg/Image'), get_message('sensor_msgs/msg/CameraInfo'), get_message('sensor_msgs/msg/PointCloud2')
def rd(uri, topics):
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=uri, storage_id='mcap'), rosbag2_py.ConverterOptions('', '')); r.set_filter(rosbag2_py.StorageFilter(topics=topics)); return r
# 뎁스(정지 구간 평균 대신 몇 장 모음)
r = rd(B + '/camera_nuctime_p1', ['/camera/depth/image_rect_raw', '/camera/depth/camera_info']); Kd = None; t0 = None; D = []; tb0 = None
while r.has_next():
    tp, raw, tb = r.read_next()
    if tp.endswith('info'): Kd = np.array(deserialize_message(raw, Info).k).reshape(3, 3); continue
    m = deserialize_message(raw, Img); t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    if t0 is None: t0 = t
    if T0 <= t - t0 <= T1 and len(D) < 10 and int((t - t0) * 10) % 20 == 0 and Kd is not None:
        D.append(K.depth_points(np.frombuffer(m.data, np.uint16).reshape(m.height, m.step // 2)[:, :m.width], Kd, step=4, zmax=4.0))
    if t - t0 > T1: break
Pc = np.concatenate(D)[::4]; print('뎁스 점', len(Pc), '카메라 t0', t0, flush=True)
# XT16 (NUC 녹화) — 같은 NUC 시각 구간
r = rd(B, ['/lidar_points']); L = []
Rl = Rot.from_euler('z', 90, degrees=True).as_matrix(); tl = np.array([0.25, 0, 0])
while r.has_next():
    tp, raw, tb = r.read_next(); m = deserialize_message(raw, PC); t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    tt = tb * 1e-9
    if t0 + T0 <= tt <= t0 + T1 and len(L) < 20:
        f = {x.name: x.offset for x in m.fields}; a = np.frombuffer(m.data, np.uint8).reshape(-1, m.point_step)
        xyz = np.stack([a[:, f[k]:f[k] + 4].copy().view('<f4')[:, 0] for k in 'xyz'], 1); xyz = xyz[np.isfinite(xyz).all(1)]
        L.append(xyz @ Rl.T + tl)
    if tt > t0 + T1: break
Pl = np.concatenate(L); rr = np.hypot(Pl[:, 0], Pl[:, 1]); Pl = Pl[(rr > 0.3) & (rr < 5)]
Pl = Pl[::2]; print("라이다 점", len(Pl), "장", len(L), flush=True)
Tl = cKDTree(Pl)
def score(R, t, P=Pc):
    Pb = P @ R.T + t; rr = np.hypot(Pb[:, 0], Pb[:, 1]); Pb = Pb[(rr < 4)]
    d, _ = Tl.query(Pb); return np.median(d), np.mean(d < 0.05), Pb
R0, t0v = K.T_BODY_CAM[:, :3], K.T_BODY_CAM[:, 3]
md, f5, _ = score(R0, t0v); print(f'11-04 보정값(아래 34°): 뎁스→라이다 최근접 중앙 {md*100:.1f} cm, 5 cm 안 {f5*100:.0f}%')
# 아래로 보는 각을 바꿔 가며(몸체 y 축 회전) 가장 잘 맞는 값
best = None
for dp in np.arange(-40, 10.1, 2):
    Rd = Rot.from_euler('y', dp, degrees=True).as_matrix() @ R0
    for dz in np.arange(-0.1, 0.21, 0.05):
        m_, f_, _ = score(Rd, t0v + [0, 0, dz])
        if best is None or m_ < best[0]: best = (m_, f_, dp, dz)
print('거친 탐색 최적: 중앙 %.1f cm, 5cm 안 %.0f%%, 몸체 y축 %+.0f°, z %+.2f' % (best[0] * 100, best[1] * 100, best[2], best[3]))
# 점-점 ICP 다듬기(6자유도)
R = Rot.from_euler('y', best[2], degrees=True).as_matrix() @ R0; t = t0v + [0, 0, best[3]]
for it in range(40):
    Pb = Pc @ R.T + t; rr = np.hypot(Pb[:, 0], Pb[:, 1]); s = rr < 4; Pb = Pb[s]; d, j = Tl.query(Pb); g = d < max(0.10, 0.3 * 0.9 ** it)
    A, Bq = Pb[g], Pl[j[g]]; ca, cb = A.mean(0), Bq.mean(0); U, S_, Vt = np.linalg.svd((A - ca).T @ (Bq - cb)); dR = Vt.T @ U.T
    if np.linalg.det(dR) < 0: Vt[-1] *= -1; dR = Vt.T @ U.T
    R = dR @ R; t = dR @ t + cb - dR @ ca
md, f5, _ = score(R, t)
oz = R[:, 2]; pitch = np.degrees(np.arcsin(-oz[2])); yaw = np.degrees(np.arctan2(oz[1], oz[0]))
print(f'ICP 결과: 중앙 {md*100:.1f} cm, 5 cm 안 {f5*100:.0f}% — 광학축 아래 {pitch:.1f}° (11-04 값 {np.degrees(np.arcsin(-R0[2,2])):.1f}°), 좌우 {yaw:+.1f}°, 위치 {np.round(t,3).tolist()}')
json.dump({'R': R.tolist(), 't': t.tolist(), 'pitch_down_deg': pitch, 'yaw_deg': yaw, 'median_cm': md * 100, 'in5cm': f5, 'old_median_cm': float(score(R0, t0v)[0] * 100)}, open(HERE + '/T_body_cam_1650.json', 'w'), indent=1)
