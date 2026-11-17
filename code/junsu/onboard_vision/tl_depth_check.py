#!/usr/bin/env python3
"""녹화된 D435 컬러+뎁스로, 학습 모델이 잡은 신호등 상자마다 실제 거리를 잰다 — 진짜 신호등(정지선 앞) vs 뒤쪽 전시장 조명 가르기.
뎁스(/camera/depth/image_rect_raw)는 컬러와 정렬 안 된 원본 → 뎁스 내부파라미터로 3D 점 → depth_to_color 외부파라미터 → 컬러 내부파라미터로 투영.
상자 거리 = 상자 가운데 60% 안에 떨어진 뎁스 점의 중앙값(유효 점 없으면 nan — 너무 멀거나 반사가 없는 것).
사용(OpenCV 4.11 → venv): source /opt/ros/jazzy/setup.bash; ~/venv/go2/bin/python3 tl_depth_check.py <카메라 mcap 폴더> <출력 json> [Hz=5]"""
import sys, os, json, struct, numpy as np, cv2, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tl_yolo_common as C

SRC, OUT = os.path.expanduser(sys.argv[1]), os.path.expanduser(sys.argv[2]); HZ = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0
net = cv2.dnn.readNetFromONNX(C.onnx_path(os.path.dirname(os.path.abspath(__file__))))   # TL_MODEL(기본 tl_cone_1104)
Img, Info = get_message('sensor_msgs/msg/Image'), get_message('sensor_msgs/msg/CameraInfo')
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=SRC, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/camera/color/image_raw', '/camera/depth/image_rect_raw', '/camera/color/camera_info',
                                               '/camera/depth/camera_info', '/camera/extrinsics/depth_to_color']))
Kc = Kd = None; Rdc = np.eye(3); Tdc = np.zeros(3); depth_q = []; rows = []; last = -1e9; t0 = None; proj = None
while r.has_next():
    tp, raw, _ = r.read_next()
    if tp == '/camera/extrinsics/depth_to_color':
        rot = struct.unpack('<9d', raw[4:76]); Tdc = np.array(struct.unpack('<3d', raw[76:100]))
        Rdc = np.array(rot).reshape(3, 3).T            # realsense: 열 우선(column-major)
        continue
    if tp.endswith('camera_info'):
        m = deserialize_message(raw, Info); K = np.array(m.k).reshape(3, 3)
        if 'color' in tp: Kc = (K, m.width, m.height)
        else: Kd = (K, m.width, m.height)
        continue
    m = deserialize_message(raw, Img); t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    if tp == '/camera/depth/image_rect_raw':
        depth_q.append((t, m)); depth_q = depth_q[-6:]; continue
    if t0 is None: t0 = t
    if t - last < 1.0 / HZ - 1e-3 or Kc is None or Kd is None or not depth_q: continue
    last = t
    a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
    bgr = cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if m.encoding == 'rgb8' else a
    blob, sc, left, top = C.letterbox(bgr); net.setInput(blob)
    dets = C.decode(net.forward(), sc, left, top, (m.height, m.width), conf_thr=0.25)
    tl = [d for d in dets if d[0] in (C.RED, C.GREEN)]
    if not tl:
        rows.append([t - t0, []]); continue
    td, dm = min(depth_q, key=lambda x: abs(x[0] - t))
    if proj is None or proj[0] is not dm:
        D = np.frombuffer(dm.data, np.uint16).reshape(dm.height, dm.width).astype(np.float64) * 0.001
        v, u = np.nonzero(D > 0.1); z = D[v, u]
        K, _, _ = Kd; x = (u - K[0, 2]) / K[0, 0] * z; y = (v - K[1, 2]) / K[1, 1] * z
        P = np.c_[x, y, z] @ Rdc.T + Tdc; K2, W2, H2 = Kc
        uc = K2[0, 0] * P[:, 0] / P[:, 2] + K2[0, 2]; vc = K2[1, 1] * P[:, 1] / P[:, 2] + K2[1, 2]
        proj = (dm, uc, vc, P[:, 2])
    _, uc, vc, zc = proj; out = []
    for c, s, x1, y1, x2, y2 in tl:
        w, h = x2 - x1, y2 - y1; ix1, ix2, iy1, iy2 = x1 + 0.2 * w, x2 - 0.2 * w, y1 + 0.2 * h, y2 - 0.2 * h
        sel = (uc >= ix1) & (uc <= ix2) & (vc >= iy1) & (vc <= iy2)
        dz = float(np.median(zc[sel])) if sel.sum() >= 3 else float('nan')
        out.append([int(c), round(float(s), 3), round(float(x1), 1), round(float(y1), 1), round(float(w), 1), round(float(h), 1), round(dz, 3), int(sel.sum()), round(abs(td - t) * 1000, 1)])
    rows.append([t - t0, out])
json.dump({'src': SRC, 'Rdc': Rdc.tolist(), 'Tdc': Tdc.tolist(), 'rows': rows}, open(OUT, 'w'))
n = sum(len(x[1]) for x in rows); dz = np.array([d[6] for x in rows for d in x[1]], float)
print(f'{SRC}: 프레임 {len(rows)} · 신호등 상자 {n} · 거리 잰 것 {np.isfinite(dz).sum()} · 거리 분위 10/50/90 {np.nanpercentile(dz, [10, 50, 90]).round(2) if np.isfinite(dz).any() else "-"} m · depth→color T {Tdc.round(4)}')
