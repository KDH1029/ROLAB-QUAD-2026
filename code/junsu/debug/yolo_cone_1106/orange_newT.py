#!/usr/bin/env python3
"""주황+뎁스 콘 점(cone_cam_core.cone_points)이 옛 장착값/새 장착값에서 각각 몇 개·어디에 나오는지(몇 장)."""
import sys, os, json, numpy as np, cv2, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
sys.path.insert(0, os.path.expanduser('~/ws/junsu_v2/onboard_vision')); import cone_cam_core as K
HERE = os.path.dirname(os.path.abspath(__file__)); N = json.load(open(HERE + '/T_body_cam_1650.json')); Tn = np.c_[np.array(N['R']), np.array(N['t'])]
Img, Info = get_message('sensor_msgs/msg/Image'), get_message('sensor_msgs/msg/CameraInfo')
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=os.path.expanduser('~/bags/gosu_20261105_165023_rec/camera_nuctime_p1'), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/camera/color/image_raw', '/camera/depth/image_rect_raw', '/camera/color/camera_info', '/camera/depth/camera_info']))
WANT = [5, 15, 42, 44, 47, 49, 52, 58, 61, 64, 66, 92]; Kc = Kd = None; dq = []; t0 = None; wi = 0
while r.has_next() and wi < len(WANT):
    tp, raw, _ = r.read_next()
    if tp.endswith('camera_info'):
        m = deserialize_message(raw, Info); Kx = np.array(m.k).reshape(3, 3)
        if 'color' in tp: Kc = Kx
        else: Kd = Kx
        continue
    m = deserialize_message(raw, Img); t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    if t0 is None: t0 = t
    if tp.endswith('image_rect_raw'): dq.append((t, m)); dq = dq[-6:]; continue
    if t - t0 < WANT[wi] or Kc is None or Kd is None or not dq: continue
    wi += 1; td, dm = min(dq, key=lambda x: abs(x[0] - t))
    a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1); bgr = cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if m.encoding == 'rgb8' else a.copy()
    D = np.frombuffer(dm.data, np.uint16).reshape(dm.height, dm.step // 2)[:, :dm.width]
    po, io = K.cone_points(D, bgr, Kd, Kc); pn, inn = K.cone_points(D, bgr, Kd, Kc, T_bc=Tn)
    f = lambda p: (np.round(np.median(p[:, :2], 0), 2).tolist() if len(p) else '-')
    print(f't={t - t0:5.1f}s 주황화소 {int(K.orange_mask(bgr).sum()):6d} | 옛: 바닥 {io["floor"]} 콘점 {len(po):3d} 중앙xy {f(po)} | 새: 바닥 {inn["floor"]} 콘점 {len(pn):3d} 중앙xy {f(pn)}', flush=True)
