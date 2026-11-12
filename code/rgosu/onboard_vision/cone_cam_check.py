#!/usr/bin/env python3
"""녹화로 카메라 콘 감지(cone_cam_core) 검증 — 학습 모델 콘 상자(cls 5) + 뎁스 위치를 기준으로 놓침·오탐을 센다.
사용(venv): source /opt/ros/jazzy/setup.bash; ~/venv/go2/bin/python3 cone_cam_check.py <카메라 mcap 폴더> <출력 앞이름> [Hz=5]"""
import sys, os, json, struct, numpy as np, cv2, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial import cKDTree
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tl_yolo_common as C
import cone_cam_core as K

SRC, OUT = os.path.expanduser(sys.argv[1]), os.path.expanduser(sys.argv[2]); HZ = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0
net = cv2.dnn.readNetFromONNX(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tl5cone_384x640.onnx'))
Img, Info = get_message('sensor_msgs/msg/Image'), get_message('sensor_msgs/msg/CameraInfo')
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=SRC, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/camera/color/image_raw', '/camera/depth/image_rect_raw', '/camera/color/camera_info', '/camera/depth/camera_info']))
Kc = Kd = None; dq = []; last = -1e9; t0 = None; stats = []; shots = []
while r.has_next():
    tp, raw, _ = r.read_next()
    if tp.endswith('camera_info'):
        m = deserialize_message(raw, Info); Kx = np.array(m.k).reshape(3, 3)
        if 'color' in tp: Kc = Kx
        else: Kd = Kx
        continue
    m = deserialize_message(raw, Img); t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    if tp.endswith('image_rect_raw'):
        dq.append((t, m)); dq = dq[-6:]; continue
    if t0 is None: t0 = t
    if t - last < 1 / HZ - 1e-3 or Kc is None or Kd is None or not dq: continue
    last = t; td, dm = min(dq, key=lambda x: abs(x[0] - t))
    if abs(td - t) > 0.05: continue
    a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1); bgr = cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if m.encoding == 'rgb8' else a.copy()
    D = np.frombuffer(dm.data, np.uint16).reshape(dm.height, dm.step // 2)[:, :dm.width]
    pts, info = K.cone_points(D, bgr, Kd, Kc)
    # 기준: 학습 모델 콘 상자 → 상자 가운데 뎁스 → 몸체 좌표
    blob, sc, l, tp_ = C.letterbox(bgr); net.setInput(blob); dets = [d for d in C.decode(net.forward(), sc, l, tp_, (m.height, m.width)) if d[0] == 5 and d[1] >= 0.5]
    proj = C.project_depth(D, Kd.astype(np.float32), Kc.astype(np.float32))
    ref = []
    for c, s, x1, y1, x2, y2 in dets:
        z = C.box_depth(proj, x1, y1, x2, y2)
        if not np.isfinite(z) or z > 3.0: continue
        uc_, vc_ = (x1 + x2) / 2, (y1 + y2) / 2; pc = np.array([(uc_ - Kc[0, 2]) / Kc[0, 0] * z, (vc_ - Kc[1, 2]) / Kc[1, 1] * z, z]) - np.array(K.DEPTH_TO_COLOR_T)
        ref.append(pc @ K.T_BODY_CAM[:, :3].T + K.T_BODY_CAM[:, 3])
    ref = np.array(ref).reshape(-1, 3)
    hit = miss = 0; far = 0
    if len(ref):
        if len(pts):
            Tq = cKDTree(pts[:, :2]); dd, _ = Tq.query(ref[:, :2]); hit = int((dd < 0.25).sum()); miss = int((dd >= 0.25).sum())
            far = int((cKDTree(ref[:, :2]).query(pts[:, :2])[0] > 0.30).sum())
        else:
            miss = len(ref)
    else:
        far = len(pts)
    stats.append((t - t0, len(ref), hit, miss, len(pts), far, info['floor'] is not None))
    if len(shots) < 8 and (len(ref) or len(pts)) and (not shots or t - t0 - shots[-1][0] > 6):
        vis = bgr.copy(); om = K.orange_mask(bgr); vis[om] = (vis[om] * 0.4 + np.array([255, 0, 255]) * 0.6).astype(np.uint8)
        for c, s, x1, y1, x2, y2 in dets: cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 255), 2)
        if len(pts):   # 콘 점을 다시 컬러 화면에 찍기
            Pc = (pts - K.T_BODY_CAM[:, 3]) @ K.T_BODY_CAM[:, :3] + np.array(K.DEPTH_TO_COLOR_T)
            for p in Pc:
                if p[2] > 0.1: cv2.circle(vis, (int(Kc[0, 0] * p[0] / p[2] + Kc[0, 2]), int(Kc[1, 1] * p[1] / p[2] + Kc[1, 2])), 3, (0, 255, 0), -1)
        cv2.putText(vis, f't={t - t0:.1f}s 모델콘 {len(ref)} 맞음 {hit} 콘점 {len(pts)} 동떨어진점 {far}', (8, 24), 0, 0.6, (255, 255, 255), 2)
        shots.append((t - t0, cv2.resize(vis, (424, 240))))
S = np.array(stats, float)
nref = S[:, 1].sum(); print(f'{SRC}: 프레임 {len(S)} · 바닥 맞춤 성공 {S[:, 6].mean() * 100:.0f}%')
print(f' 모델이 본 콘(3 m 안) {nref:.0f}개 중 콘 점이 0.25 m 안에 있음 {S[:, 2].sum():.0f} ({S[:, 2].sum() / max(nref, 1) * 100:.0f}%) · 놓침 {S[:, 3].sum():.0f}')
print(f' 콘 점 프레임당 중앙 {np.median(S[S[:, 4] > 0, 4]) if (S[:, 4] > 0).any() else 0:.0f}개 · 모델 콘에서 0.3 m 넘게 동떨어진 점 비율 {S[:, 5].sum() / max(S[:, 4].sum(), 1) * 100:.0f}% (모델이 놓친 콘도 여기 들어감)')
print(f' 모델 콘이 없는 프레임 {int((S[:, 1] == 0).sum())} 중 콘 점이 나온 프레임 {int(((S[:, 1] == 0) & (S[:, 4] > 0)).sum())}')
if shots:
    tiles = [s[1] for s in shots]; tiles += [np.zeros_like(tiles[0])] * ((-len(tiles)) % 4)
    cv2.imwrite(OUT + '.jpg', np.vstack([np.hstack(tiles[i:i + 4]) for i in range(0, len(tiles), 4)])); print('그림', OUT + '.jpg')
json.dump(S.tolist(), open(OUT + '.json', 'w'))
