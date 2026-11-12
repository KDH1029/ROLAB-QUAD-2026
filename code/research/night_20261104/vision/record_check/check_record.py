#!/usr/bin/env python3
"""분리 녹화 확인 그림 — 같은 순간의 컬러·깊이·적외선 + XT16 점을 깊이 영상에 겹치기(시각·외부파라미터 확인).
사용: check_record.py <NUC bag> <카메라 bag(NUC 시계)> <출력 png> [시각 비율 0~1=0.5]"""
import sys, json, numpy as np, rosbag2_py, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt, logging
logging.getLogger('matplotlib.font_manager').setLevel(logging.ERROR); plt.rcParams['font.family'] = ['NanumGothic', 'DejaVu Sans']
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
sys.path.insert(0, '/home/ilovehan9283/ws/v_gosu/backup')
from backup_core import R_LIDAR, T_LIDAR
NUC, CAM, PNG = sys.argv[1], sys.argv[2], sys.argv[3]; FR = float(sys.argv[4]) if len(sys.argv) > 4 else 0.5
CAL = json.load(open('/home/ilovehan9283/ws/night_20261104/vision/calib_depth_all.json')); TBC = np.array(CAL['T_body_cam_optical'])
FX = FY = 431.08; CX, CY = 422.28, 240.89
def rd(uri, topics, sid):
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=uri, storage_id=sid), rosbag2_py.ConverterOptions('', '')); r.set_filter(rosbag2_py.StorageFilter(topics=topics)); return r
def stamp(m): return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
# 카메라: 중간 시각의 컬러·깊이·적외선
r = rd(CAM, ['/camera/depth/image_rect_raw'], 'mcap'); ts = []
while r.has_next(): ts.append(r.read_next()[2])
tgt = ts[int(len(ts) * FR)]
imgs = {}
for tp in ('/camera/color/image_raw', '/camera/depth/image_rect_raw', '/camera/infra1/image_rect_raw'):
    r = rd(CAM, [tp], 'mcap'); best = None
    while r.has_next():
        _, raw, t = r.read_next()
        if best is None or abs(t - tgt) < abs(best[0] - tgt): best = (t, raw)
        if t > tgt + 5e8: break
    imgs[tp] = deserialize_message(best[1], Image)
dm = imgs['/camera/depth/image_rect_raw']; D = np.frombuffer(dm.data, np.uint16).reshape(dm.height, dm.width) * 1e-3
cm = imgs['/camera/color/image_raw']; Cimg = np.frombuffer(cm.data, np.uint8).reshape(cm.height, cm.width, 3)
im = imgs['/camera/infra1/image_rect_raw']; I = np.frombuffer(im.data, np.uint8).reshape(im.height, im.width)
t_cam = stamp(dm)
# XT16: 깊이 stamp 와 가장 가까운 스캔
r = rd(NUC, ['/lidar_points'], 'mcap'); best = None
while r.has_next():
    _, raw, t = r.read_next(); m = deserialize_message(raw, PointCloud2); s = stamp(m)
    if best is None or abs(s - t_cam) < abs(best[0] - t_cam): best = (s, m)
    if s > t_cam + 1: break
L = pc2.read_points_numpy(best[1], field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)
B = L @ R_LIDAR.T + T_LIDAR                                      # 라이다 → 몸체
Ti = np.linalg.inv(TBC); P = B @ Ti[:3, :3].T + Ti[:3, 3]          # 몸체 → 깊이 광학
P = P[(P[:, 2] > 0.3) & (P[:, 2] < 6)]
u = FX * P[:, 0] / P[:, 2] + CX; v = FY * P[:, 1] / P[:, 2] + CY; ok = (u >= 0) & (u < D.shape[1]) & (v >= 0) & (v < D.shape[0])
u, v, z = u[ok], v[ok], P[ok, 2]
dz = D[v.astype(int), u.astype(int)]; good = dz > 0.2
err = np.abs(dz[good] - z[good])
print(json.dumps(dict(카메라시각=round(t_cam, 3), 라이다시각차_ms=round((best[0] - t_cam) * 1e3, 1), 겹친점=int(ok.sum()),
                      깊이와_라이다_거리차_중앙_cm=round(float(np.median(err)) * 100, 1), _10cm안_비율=round(float((err < 0.10).mean()), 3)), ensure_ascii=False))
fig, ax = plt.subplots(2, 2, figsize=(16, 10))
ax[0, 0].imshow(Cimg); ax[0, 0].set_title(f'컬러 {cm.width}×{cm.height} ({cm.encoding})')
ax[0, 1].imshow(np.where(D > 0, D, np.nan), cmap='turbo', vmin=0.2, vmax=4); ax[0, 1].set_title('깊이 (m)')
ax[1, 0].imshow(I, cmap='gray'); ax[1, 0].set_title('적외선 1')
ax[1, 1].imshow(np.where(D > 0, D, np.nan), cmap='gray', vmin=0.2, vmax=4); sc = ax[1, 1].scatter(u, v, c=z, s=6, cmap='turbo', vmin=0.2, vmax=4)
ax[1, 1].set_title(f'깊이 위에 XT16 점 (시각차 {(best[0] - t_cam) * 1e3:+.0f} ms, 거리차 중앙 {np.median(err) * 100:.1f} cm)')
for a in ax.ravel(): a.axis('off')
fig.suptitle(f'분리 녹화 확인 — 카메라는 내부 PC, 라이다는 NUC, 시각은 NUC 기준으로 보정 ({FR * 100:.0f}% 지점)')
plt.tight_layout(); plt.savefig(PNG, dpi=75); print('저장', PNG)
