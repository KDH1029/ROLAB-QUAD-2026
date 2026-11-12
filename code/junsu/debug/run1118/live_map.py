#!/usr/bin/env python3
"""11:18 실주행에서 로봇이 쓴 벽(/mission/wall_cloud, 세계 camera_init)과 콘(/mission/cones)으로 2D 지도 — 여러 번 찍힌 칸만(사람 거름).
경로(route_map_1106, 1459 지도 좌표)는 그날 출발 정렬값(/mission/align_T: 오늘 = T⁻¹·지도)으로 같은 좌표에 놓고 점마다 여유를 잰다."""
import rosbag2_py, json, math, numpy as np, os
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial import cKDTree
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
plt.rcParams['font.family'] = 'NanumGothic'; plt.rcParams['axes.unicode_minus'] = False
B = '/home/ilovehan9283/bags/gosu_20261106_111853_cone1106'; J = os.path.expanduser('~/ws/junsu_v2'); H = os.path.dirname(os.path.abspath(__file__))
TY = {'/mission/wall_cloud': 'sensor_msgs/msg/PointCloud2', '/mission/cones': 'geometry_msgs/msg/PoseArray', '/mission/align_T': 'std_msgs/msg/String', '/Odometry': 'nav_msgs/msg/Odometry'}
M = {k: get_message(v) for k, v in TY.items()}
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=B, storage_id='mcap'), rosbag2_py.ConverterOptions('', '')); r.set_filter(rosbag2_py.StorageFilter(topics=list(TY)))
C = 0.03; hits = {}; cones = []; T = None; z = 0.0; n = 0; path = []
while r.has_next():
    tp, raw, t = r.read_next(); m = deserialize_message(raw, M[tp])
    if tp == '/Odometry': z = m.pose.pose.position.z; path.append((m.pose.pose.position.x, m.pose.pose.position.y))
    elif tp == '/mission/align_T': T = json.loads(m.data)['T']
    elif tp == '/mission/cones': cones += [(p.position.x, p.position.y) for p in m.poses]
    elif tp == '/mission/wall_cloud':
        n += 1
        if n % 2: continue
        f = {x.name: x.offset for x in m.fields}; a = np.frombuffer(m.data, np.uint8).reshape(-1, m.point_step)
        P = np.stack([a[:, f[k]:f[k] + 4].copy().view('<f4')[:, 0] for k in 'xyz'], 1); P = P[np.abs(P[:, 2] - z) < 0.6]
        for k in set(map(tuple, np.floor(P[:, :2] / C).astype(int))): hits[k] = hits.get(k, 0) + 1
K = np.array([k for k, v in hits.items() if v >= 4]); W = (K + 0.5) * C
cn = np.array(cones); Cc = []
if len(cn):
    kk, cnt = np.unique(np.round(cn / 0.10).astype(int), axis=0, return_counts=True); Cc = (kk[cnt >= 15] * 0.10)
print(f'벽 칸 {len(W)}(4번 이상 본 칸) · 콘(15번 이상 인식한 10 cm 칸) {len(Cc)} · 정렬 T {np.round(T, 3).tolist()}')
np.save(H + '/live_walls.npy', W); np.save(H + '/live_cones.npy', np.asarray(Cc))
tx, ty, th = T; c, s = math.cos(th), math.sin(th)
to_today = lambda X, Y: ((c * (X - tx) + s * (Y - ty)), (-s * (X - tx) + c * (Y - ty)))
R = json.load(open(J + '/maps/route_map_1106.json')); pts = [(p['label'],) + to_today(p['x'], p['y']) for p in R['route']]
Obs = np.vstack([W, np.asarray(Cc).reshape(-1, 2)]) if len(Cc) else W; TO = cKDTree(Obs); TW = cKDTree(W); TC = cKDTree(Cc) if len(Cc) else None
def segmin(a, b, tree):
    a, b = np.array(a), np.array(b); n_ = max(2, int(np.hypot(*(b - a)) / 0.03)); s_ = np.linspace(0, 1, n_)[:, None]; return float(tree.query(a + s_ * (b - a))[0].min())
print(' 점          벽까지  콘까지  다음 점까지 구간 최소 여유(벽·콘)')
for i, (lab, x, y) in enumerate(pts):
    dw = TW.query((x, y))[0]; dc = TC.query((x, y))[0] if TC is not None else 9
    sg = segmin((x, y), pts[i + 1][1:], TO) if i + 1 < len(pts) else 9
    flag = ('★' if min(dw, dc) < 0.25 else '') + ('◆' if sg < 0.20 else '')
    print(f'  {lab:10s} {dw:5.2f}  {dc:5.2f}   {sg:5.2f}  {flag}')
P = np.array([p[1:] for p in pts]); pa = np.array(path)
fig, ax = plt.subplots(figsize=(14, 9.5)); ax.scatter(W[:, 0], W[:, 1], s=2, c='k', label='벽(실주행 라이다, 4번 이상)')
if len(Cc): ax.scatter(np.asarray(Cc)[:, 0], np.asarray(Cc)[:, 1], s=55, marker='^', c='orange', edgecolors='k', label='콘(15번 이상 인식)')
ax.plot(pa[:, 0], pa[:, 1], '-', c='limegreen', lw=1, alpha=.6, label='11:18 실제 궤적'); ax.plot(P[:, 0], P[:, 1], 'r.-', lw=1.4, ms=8, label='지금 경로(그날 정렬값으로)')
for (lab, x, y) in pts:
    bad = min(TW.query((x, y))[0], TC.query((x, y))[0] if TC is not None else 9) < 0.25
    ax.annotate(lab, (x, y), fontsize=8, color='red' if bad else 'navy', xytext=(3, 3), textcoords='offset points', fontweight='bold' if bad else 'normal')
ax.set_aspect('equal'); ax.grid(alpha=.3); ax.legend(loc='upper left', fontsize=9); ax.set_xlim(pa[:, 0].min() - 1, pa[:, 0].max() + 1); ax.set_ylim(pa[:, 1].min() - 1, pa[:, 1].max() + 1)
ax.set_title('11:18 실주행 지도(로봇이 본 벽·콘) 위 지금 경로 — 빨간 이름 = 벽·콘 0.25 m 안'); plt.tight_layout(); plt.savefig(H + '/route_on_live1118.png', dpi=85)
