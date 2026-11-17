#!/usr/bin/env python3
"""11-06 바뀐 콘에 YOLO(tl_cone_1104)가 듣는지 — 내부 PC 카메라 녹화에 로봇과 같은 전처리·후처리로 돌린다.
사용: source /opt/ros/jazzy/setup.bash; ~/venv/go2/bin/python3 run_yolo_bag.py <카메라 mcap 폴더> <출력 폴더> [Hz=5]
출력: dets.json(프레임마다 상자·콘 거리·주황 점 수), frames/*.jpg(원본 848x480)"""
import sys, os, json, numpy as np, cv2, rosbag2_py, time
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
OV = os.path.expanduser('~/ws/junsu_v2/onboard_vision'); sys.path.insert(0, OV)
import tl_yolo_common as C
import cone_cam_core as K

SRC, OUT = os.path.expanduser(sys.argv[1]), os.path.expanduser(sys.argv[2]); HZ = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0
os.makedirs(OUT + '/frames', exist_ok=True)
net = cv2.dnn.readNetFromONNX(C.onnx_path(OV))
Img, Info = get_message('sensor_msgs/msg/Image'), get_message('sensor_msgs/msg/CameraInfo')
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=SRC, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/camera/color/image_raw', '/camera/depth/image_rect_raw', '/camera/color/camera_info', '/camera/depth/camera_info']))
Kc = Kd = None; dq = []; last = -1e9; t0 = None; rows = []; cone_c = None; tw = time.time()
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
    a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
    bgr = cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if m.encoding == 'rgb8' else a.copy()
    blob, sc, l, tp_ = C.letterbox(bgr); net.setInput(blob); out = net.forward()
    if cone_c is None: cone_c = C.cone_cls(out)
    dets = C.decode(out, sc, l, tp_, (m.height, m.width), conf_thr=0.25)
    D = None; proj = None; zs = []
    if abs(td - t) <= 0.05:
        D = np.frombuffer(dm.data, np.uint16).reshape(dm.height, dm.step // 2)[:, :dm.width]
        proj = C.project_depth(D, Kd.astype(np.float32), Kc.astype(np.float32))
    rec = []
    for c, s, x1, y1, x2, y2 in dets:
        z = C.cone_depth(proj, x1, y1, x2, y2) if (proj is not None and c == cone_c) else float('nan')
        rec.append([int(c), round(float(s), 3), round(x1, 1), round(y1, 1), round(x2, 1), round(y2, 1), None if not np.isfinite(z) else round(z, 2)])
    n_or = 0; n_orpix = int(K.orange_mask(bgr).sum())
    if D is not None:
        pts, _ = K.cone_points(D, np.ascontiguousarray(bgr), Kd.astype(np.float64), Kc.astype(np.float64)); n_or = int(len(pts))
    k = len(rows); cv2.imwrite(f'{OUT}/frames/{k:04d}.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 88])
    rows.append({'k': k, 't': round(t - t0, 3), 'stamp': t, 'dets': rec, 'orange_pts': n_or, 'orange_px': n_orpix, 'depth_ok': D is not None})
    if k % 100 == 0: print(f'{k} 장 t={t - t0:.1f}s ({time.time() - tw:.0f}s)', flush=True)
json.dump({'src': SRC, 'hz': HZ, 'cone_cls': cone_c, 'names': C.NAMES, 'model': C.MODEL, 'rows': rows}, open(OUT + '/dets.json', 'w'))
print('끝', len(rows), '장', f'{time.time() - tw:.0f}s', OUT)
