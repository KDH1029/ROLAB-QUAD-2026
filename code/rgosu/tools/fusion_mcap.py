#!/usr/bin/env python3
"""녹화 → Foxglove 용 '카메라 깊이 + 라이다' 퓨전 mcap. 재생 없음(파일만 읽고 새 파일을 쓴다).
사용: python3 fusion_mcap.py <녹화폴더(gosu_…, camera_nuctime_p1 포함)> [출력폴더=<녹화>/foxglove_fusion]
담는 것 (전부 NUC 시계, Foxglove 3D 패널에서 고정 프레임 odom)
  /tf            odom → base            /utlidar/robot_odom (로봇 자체 위치)
  /tf_static     base → hesai_lidar     FAST-LIO 외부파라미터
                 base → cam_old         10-30 카메라 보정(calib/d435i_to_body.json) — 지금 장착과 어긋남
                 base → cam_new         이 녹화로 다시 맞춘 값(깊이 점 ↔ XT16 점 정합)
  /lidar_points          XT16 (5 Hz, x y z intensity)
  /camera/depth/points_old · /camera/depth/points_new   같은 깊이 점(5 Hz), 프레임만 다름 → 겹쳐 보면 보정 차이가 보인다
  /camera/color/image/compressed   컬러 영상(JPEG, 10 Hz)
결과 보정값: <출력>/d435i_to_body_new.json"""
import sys, os, json, math, shutil
import numpy as np, cv2, rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2 as pc2
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as Rot

B = sys.argv[1].rstrip('/'); OUT = sys.argv[2] if len(sys.argv) > 2 else B + '/foxglove_fusion'
CAM = sorted(d for d in os.listdir(B) if d.startswith('camera_nuctime'))[0]
R_L = np.array([[-0.0001707, -0.9999240, 0.0123287], [0.9999844, -0.0001019, 0.0055807], [-0.0055790, 0.0123295, 0.9999084]]); T_L = np.array([0.25, 0.0, 0.1588])
CAL = os.path.expanduser('~/go2_record/calib/d435i_to_body.json'); T_old = np.array(json.load(open(CAL))['T_body_cam_optical'])
Img, Info, PC, Od = (get_message(t) for t in ('sensor_msgs/msg/Image', 'sensor_msgs/msg/CameraInfo', 'sensor_msgs/msg/PointCloud2', 'nav_msgs/msg/Odometry'))

def reader(uri, topics):
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=uri, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics)); return r

# ---- 읽기: 라이다(5 Hz), 로봇 위치, 깊이(5 Hz), 컬러(10 Hz)
lid, odo, k = [], [], 0
r = reader(B, ['/lidar_points', '/utlidar/robot_odom'])
while r.has_next():
    tp, raw, t = r.read_next()
    if tp == '/utlidar/robot_odom':
        m = deserialize_message(raw, Od); p, q = m.pose.pose.position, m.pose.pose.orientation; odo.append((t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
    else:
        k += 1
        if k % 4: continue
        m = deserialize_message(raw, PC); a = pc2.read_points(m, field_names=['x', 'y', 'z', 'intensity'], skip_nans=True)
        lid.append((t, np.stack([a['x'], a['y'], a['z'], a['intensity']], 1).astype(np.float32)))
odo = np.array(odo); ot = odo[:, 0]
info = None; dep, col, kd, kc = [], [], 0, 0
r = reader(os.path.join(B, CAM), ['/camera/depth/image_rect_raw', '/camera/depth/camera_info', '/camera/color/image_raw'])
while r.has_next():
    tp, raw, t = r.read_next()
    if tp == '/camera/depth/camera_info':
        if info is None: info = deserialize_message(raw, Info)
    elif tp == '/camera/depth/image_rect_raw':
        kd += 1
        if kd % 6: continue
        m = deserialize_message(raw, Img); dep.append((t, np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).copy()))
    else:
        kc += 1
        if kc % 3: continue
        m = deserialize_message(raw, Img); a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
        ok, enc = cv2.imencode('.jpg', a[:, :, ::-1] if m.encoding == 'rgb8' else a, [cv2.IMWRITE_JPEG_QUALITY, 80]); col.append((t, enc.tobytes()))
K = np.array(info.k).reshape(3, 3); fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
print(f'라이다 {len(lid)} · 깊이 {len(dep)} · 컬러 {len(col)} · 로봇위치 {len(odo)} · 깊이 카메라 {info.width}x{info.height} fx {fx:.1f}')

def depth_points(D, step=6):
    v, u = np.mgrid[0:D.shape[0]:step, 0:D.shape[1]:step]; z = D[v, u].astype(np.float32) * 0.001
    m = (z > 0.25) & (z < 5.0); z, u, v = z[m], u[m], v[m]
    return np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], 1)

def lidar_body(L): return L[:, :3].astype(np.float64) @ R_L.T + T_L

# ---- 카메라 보정 다시 맞추기: 깊이 점(몸체 좌표)을 같은 시각 XT16 점(몸체 좌표)에 정합
lt = np.array([t for t, _ in lid])
pairs = []
for t, D in dep[::3]:
    i = int(np.argmin(np.abs(lt - t)))
    if abs(lt[i] - t) > 0.06e9: continue
    Lb = lidar_body(lid[i][1]); Lb = Lb[(np.linalg.norm(Lb, axis=1) < 6)]
    Pc = depth_points(D, 8)
    if len(Pc) > 500 and len(Lb) > 500: pairs.append((Pc[::2], Lb))
print('정합에 쓰는 프레임', len(pairs))
def score(T, thr):
    n = tot = 0
    for Pc, Lb in pairs:
        Q = Pc @ T[:3, :3].T + T[:3, 3]; d, _ = cKDTree(Lb).query(Q); n += (d < thr).sum(); tot += len(d)
    return n / max(1, tot)
best = (score(T_old, 0.10), T_old)
# 굵게: 카메라 위치는 그대로 두고 방향(pitch·yaw·roll)을 넓게 훑는다
ypr0 = Rot.from_matrix(T_old[:3, :3]).as_euler('ZYX', degrees=True)
for dp in range(-30, 31, 3):
    for dy in (-6, 0, 6):
        T = T_old.copy(); T[:3, :3] = (Rot.from_euler('ZYX', [dy, dp, 0], degrees=True) * Rot.from_matrix(T_old[:3, :3])).as_matrix()
        s = score(T, 0.10)
        if s > best[0]: best = (s, T)
print(f'굵게 찾기: 10 cm 안 비율 {score(T_old, 0.10):.3f}(기존) → {best[0]:.3f}')
T = best[1]
for it, thr in enumerate([0.25, 0.15, 0.10, 0.06, 0.04, 0.04]):   # 점-점 ICP (정합 거리 줄여가며)
    A, Bm = [], []
    for Pc, Lb in pairs:
        Q = Pc @ T[:3, :3].T + T[:3, 3]; d, j = cKDTree(Lb).query(Q); m = d < thr; A.append(Q[m]); Bm.append(Lb[j[m]])
    A, Bm = np.concatenate(A), np.concatenate(Bm); ca, cb = A.mean(0), Bm.mean(0); U, S, Vt = np.linalg.svd((A - ca).T @ (Bm - cb))
    Rd = (U @ Vt).T
    if np.linalg.det(Rd) < 0: Vt[-1] *= -1; Rd = (U @ Vt).T
    dT = np.eye(4); dT[:3, :3] = Rd; dT[:3, 3] = cb - Rd @ ca; T = dT @ T
T_new = T
def desc(T):
    ypr = Rot.from_matrix(T[:3, :3] @ np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]]).T).as_euler('ZYX', degrees=True)   # 광학→카메라 몸체(x앞) 축
    return {'위치_m': [round(float(v), 4) for v in T[:3, 3]], '기울기_deg_yaw_pitch_roll': [round(float(v), 2) for v in ypr]}
res = {'기존(10-30)': dict(desc(T_old), 일치_10cm=round(score(T_old, 0.10), 3), 일치_5cm=round(score(T_old, 0.05), 3)),
       '새로 맞춤': dict(desc(T_new), 일치_10cm=round(score(T_new, 0.10), 3), 일치_5cm=round(score(T_new, 0.05), 3)), '프레임': len(pairs)}
print(json.dumps(res, ensure_ascii=False, indent=1))

# ---- 쓰기
shutil.rmtree(OUT, ignore_errors=True)
w = rosbag2_py.SequentialWriter(); w.open(rosbag2_py.StorageOptions(uri=OUT, storage_id='mcap', storage_preset_profile='zstd_fast'), rosbag2_py.ConverterOptions('', ''))
for i, (n, ty) in enumerate([('/tf', 'tf2_msgs/msg/TFMessage'), ('/tf_static', 'tf2_msgs/msg/TFMessage'), ('/lidar_points', 'sensor_msgs/msg/PointCloud2'),
                            ('/camera/depth/points_old', 'sensor_msgs/msg/PointCloud2'), ('/camera/depth/points_new', 'sensor_msgs/msg/PointCloud2'),
                            ('/camera/color/image/compressed', 'sensor_msgs/msg/CompressedImage'), ('/robot_odom', 'nav_msgs/msg/Odometry')]):
    w.create_topic(rosbag2_py.TopicMetadata(id=i, name=n, type=ty, serialization_format='cdr') if 'id' in rosbag2_py.TopicMetadata.__init__.__doc__ else rosbag2_py.TopicMetadata(name=n, type=ty, serialization_format='cdr'))
TF, TS, CI = get_message('tf2_msgs/msg/TFMessage'), get_message('geometry_msgs/msg/TransformStamped'), get_message('sensor_msgs/msg/CompressedImage')
def stampof(t): from builtin_interfaces.msg import Time; return Time(sec=int(t // 1e9), nanosec=int(t % 1e9))
def tfm(parent, child, T, t):
    s = TS(); s.header.frame_id = parent; s.child_frame_id = child; s.header.stamp = stampof(t)
    s.transform.translation.x, s.transform.translation.y, s.transform.translation.z = map(float, T[:3, 3])
    q = Rot.from_matrix(T[:3, :3]).as_quat(); s.transform.rotation.x, s.transform.rotation.y, s.transform.rotation.z, s.transform.rotation.w = map(float, q); return s
t0 = int(min(ot[0], lt[0], dep[0][0], col[0][0]))
TLm = np.eye(4); TLm[:3, :3] = R_L; TLm[:3, 3] = T_L
w.write('/tf_static', serialize_message(TF(transforms=[tfm('base', 'hesai_lidar', TLm, t0), tfm('base', 'cam_old', T_old, t0), tfm('base', 'cam_new', T_new, t0)])), t0)
ev = []
for row in odo[::3]:
    t = int(row[0]); T = np.eye(4); T[:3, :3] = Rot.from_quat(row[4:8]).as_matrix(); T[:3, 3] = row[1:4]
    ev.append((t, '/tf', serialize_message(TF(transforms=[tfm('odom', 'base', T, t)]))))
    o = Od(); o.header.frame_id = 'odom'; o.child_frame_id = 'base'; o.header.stamp = stampof(t); o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = map(float, row[1:4])
    o.pose.pose.orientation.x, o.pose.pose.orientation.y, o.pose.pose.orientation.z, o.pose.pose.orientation.w = map(float, row[4:8]); ev.append((t, '/robot_odom', serialize_message(o)))
from std_msgs.msg import Header
from sensor_msgs.msg import PointField
F4 = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate(('x', 'y', 'z', 'intensity'))]
for t, L in lid:
    h = Header(frame_id='hesai_lidar', stamp=stampof(t)); ev.append((t, '/lidar_points', serialize_message(pc2.create_cloud(h, F4, L))))
for t, D in dep:
    P = depth_points(D, 4); P4 = np.c_[P, P[:, 2]].astype(np.float32)
    for fr, tp in (('cam_old', '/camera/depth/points_old'), ('cam_new', '/camera/depth/points_new')):
        ev.append((t, tp, serialize_message(pc2.create_cloud(Header(frame_id=fr, stamp=stampof(t)), F4, P4))))
for t, jp in col:
    c = CI(); c.header.frame_id = 'cam_new'; c.header.stamp = stampof(t); c.format = 'jpeg'; c.data = jp; ev.append((t, '/camera/color/image/compressed', serialize_message(c)))
ev.sort(key=lambda e: e[0])
for t, tp, data in ev: w.write(tp, data, t)
del w
json.dump(dict(res, T_body_cam_optical_new=T_new.round(5).tolist()), open(os.path.join(OUT, 'd435i_to_body_new.json'), 'w'), ensure_ascii=False, indent=1)
print('■ 저장', OUT, f'{sum(os.path.getsize(os.path.join(OUT, f)) for f in os.listdir(OUT)) / 1e6:.0f} MB')
