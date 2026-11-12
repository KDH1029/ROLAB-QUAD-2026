#!/usr/bin/env python3
"""11-06 11:18 junsu_v2 완주 주행 디버깅 — 벽 붙음·물러남·stopline2.
몸 사각형(IMU 기준 x −0.35~0.35, y ±0.165) ↔ 벽(/mission/wall_cloud, camera_init) 거리를 0.1초마다 잰다. /scan(range_min 0.4)이 그 벽을 봤는지도."""
import sys, os, json, math, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial import cKDTree
B = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else '~/bags/gosu_20261106_111853_cone1106'); OUT = os.path.dirname(os.path.abspath(__file__))
TY = {'/Odometry': 'nav_msgs/msg/Odometry', '/mission/wall_cloud': 'sensor_msgs/msg/PointCloud2', '/scan': 'sensor_msgs/msg/LaserScan', '/cmd_vel': 'geometry_msgs/msg/Twist',
      '/mission/runner_status': 'std_msgs/msg/String', '/goal_pose': 'geometry_msgs/msg/PoseStamped', '/mission/cones': 'geometry_msgs/msg/PoseArray',
      '/mission/traffic_light_state': 'std_msgs/msg/String', '/lf/sportmodestate': 'unitree_go/msg/SportModeState', '/ego_planner/trajectory': 'nav_msgs/msg/Path'}
M = {k: get_message(v) for k, v in TY.items()}
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=B, storage_id='mcap'), rosbag2_py.ConverterOptions('', '')); r.set_filter(rosbag2_py.StorageFilter(topics=list(TY)))
def xyz(m):
    f = {x.name: x.offset for x in m.fields}; a = np.frombuffer(m.data, np.uint8).reshape(-1, m.point_step)
    return np.stack([a[:, f[k]:f[k] + 4].copy().view('<f4')[:, 0] for k in 'xyz'], 1)
od, walls, scans, cmds, rs, goals, cones, tl, spd, traj = [], [], [], [], [], [], None, [], [], []
t0 = None
while r.has_next():
    tp, raw, t = r.read_next(); t *= 1e-9; t0 = t0 or t; t -= t0; m = deserialize_message(raw, M[tp])
    if tp == '/Odometry':
        p, q = m.pose.pose.position, m.pose.pose.orientation; od.append((t, p.x, p.y, p.z, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
    elif tp == '/mission/wall_cloud': walls.append((t, xyz(m)))
    elif tp == '/scan':
        rr = np.asarray(m.ranges); a = m.angle_min + np.arange(len(rr)) * m.angle_increment; ok = np.isfinite(rr) & (rr >= m.range_min) & (rr <= m.range_max)
        scans.append((t, np.c_[rr[ok] * np.cos(a[ok]), rr[ok] * np.sin(a[ok])]))
    elif tp == '/cmd_vel': cmds.append((t, m.linear.x, m.linear.y, m.angular.z))
    elif tp == '/mission/runner_status': d = json.loads(m.data); rs.append((t, d.get('idx'), d.get('label'), d.get('state')))
    elif tp == '/goal_pose': goals.append((t, m.pose.position.x, m.pose.position.y))
    elif tp == '/mission/cones': cones = np.array([[p.position.x, p.position.y] for p in m.poses]).reshape(-1, 2) if len(m.poses) else cones
    elif tp == '/mission/traffic_light_state': tl.append((t, m.data))
    elif tp == '/lf/sportmodestate': spd.append((t, float(np.hypot(m.velocity[0], m.velocity[1])), float(m.yaw_speed)))
od = np.array(od); cmds = np.array(cmds); spd = np.array(spd); wt = np.array([w[0] for w in walls]); st = np.array([s[0] for s in scans])
XF, XB, HW = 0.35, -0.35, 0.165
def rect_dist(P):   # 몸 좌표 점 → 몸 사각형까지 거리(안이면 0)
    dx = np.maximum(np.maximum(XB - P[:, 0], P[:, 0] - XF), 0); dy = np.maximum(np.abs(P[:, 1]) - HW, 0); return np.hypot(dx, dy)
rows = []
for t, x, y, z, yaw in od[::2]:
    i = int(np.clip(np.searchsorted(wt, t), 0, len(walls) - 1)); W = walls[i][1]
    c, s = math.cos(yaw), math.sin(yaw); d = W[:, :2] - [x, y]; Pb = np.c_[c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]]
    near = (np.abs(Pb[:, 0]) < 1.5) & (np.abs(Pb[:, 1]) < 1.5) & (W[:, 2] > z - 0.25) & (W[:, 2] < z + 0.5)
    Pn = Pb[near]; dist = rect_dist(Pn) if len(Pn) else np.array([9.9])
    k = int(np.argmin(dist)); side = 'L' if len(Pn) and Pn[k, 1] > HW else ('R' if len(Pn) and Pn[k, 1] < -HW else ('F' if len(Pn) and Pn[k, 0] > XF else 'B'))
    j = int(np.clip(np.searchsorted(st, t), 0, len(scans) - 1)); S = scans[j][1]
    # 스캔(base_stab, 몸 중심 근처 원점)에 그 벽 점 근처(10 cm) 점이 있었나
    seen = bool(len(S) and len(Pn) and cKDTree(S).query(Pn[k])[0] < 0.10)
    rows.append((t, x, y, yaw, float(dist.min()), side, seen))
R = np.array([(a[0], a[1], a[2], a[3], a[4]) for a in rows]); SIDE = [a[5] for a in rows]; SEEN = np.array([a[6] for a in rows])
def state_at(t):
    k = max([i for i, s in enumerate(rs) if s[0] <= t] or [0]); return rs[k]
print(f'주행 {rs[0][0]:.1f}~{rs[-1][0]:.1f}s, Odometry {len(od)} · 벽 구름 {len(walls)} · 스캔 {len(scans)}')
run = (R[:, 0] >= rs[0][0]) & (R[:, 0] <= rs[-1][0]); dt = np.median(np.diff(R[:, 0]))
for th in (0.03, 0.05, 0.10):
    s = run & (R[:, 4] < th); print(f' 몸↔벽 {th*100:.0f} cm 안: {s.sum() * dt:.1f}초 · 그중 /scan 이 그 벽을 못 본 비율 {np.mean(~SEEN[s]) * 100 if s.any() else 0:.0f}%')
# 구간별
print(' 구간(목표)        시간s  벽5cm안s  가장가까움cm  물러남')
segs = []
for i, (t, idx, lab, stt) in enumerate(rs):
    if not segs or segs[-1][1] != idx: segs.append([t, idx, lab, 0])
    if stt == 'BACKOFF' and (i == 0 or rs[i - 1][3] != 'BACKOFF'): segs[-1][3] += 1
for i, (t, idx, lab, nb) in enumerate(segs):
    te = segs[i + 1][0] if i + 1 < len(segs) else rs[-1][0]; s = (R[:, 0] >= t) & (R[:, 0] < te)
    if not s.any(): continue
    print(f'  {idx:2d} {lab:10s} {te - t:6.1f}  {np.sum(R[s, 4] < 0.05) * dt:6.1f}   {R[s, 4].min() * 100:6.1f}      {nb}')
# 물러남 장면
print(' 물러남(BACKOFF) 장면: 시각 · 목표 · 몸↔벽 cm(어느 쪽) · /scan 이 봤나 · 직전 3초 명령 평균(vx,vy,wz) · 실제 속도')
for i, (t, idx, lab, stt) in enumerate(rs):
    if stt == 'BACKOFF' and (i == 0 or rs[i - 1][3] != 'BACKOFF'):
        k = int(np.argmin(np.abs(R[:, 0] - t))); c = cmds[(cmds[:, 0] > t - 3) & (cmds[:, 0] < t)]; v = spd[(spd[:, 0] > t - 3) & (spd[:, 0] < t)]
        print(f'  {t:6.1f}s {lab:9s} 벽 {R[k, 4]*100:5.1f}cm({SIDE[k]}) scan봄={SEEN[k]}  명령 {np.round(c[:, 1:].mean(0), 2) if len(c) else "-"}  속도 {v[:, 1].mean() if len(v) else 0:.2f} m/s')
# stopline 들
for lab in ('stopline1', 'stopline2'):
    ts = [s[0] for s in rs if s[2] == lab]; 
    if not ts: continue
    s = (R[:, 0] >= ts[0]) & (R[:, 0] <= ts[-1] + 0.5); holds = [x for x in rs if x[2] == lab and x[3] in ('STOPLINE_HOLD', 'STOPLINE_WAIT_SIGNAL')]
    yaw = np.degrees(R[s, 3]); v = spd[(spd[:, 0] >= ts[0]) & (spd[:, 0] <= ts[-1])]
    print(f' {lab}: {ts[0]:.1f}~{ts[-1]:.1f}s ({ts[-1]-ts[0]:.1f}s) · 몸 방향 {yaw.min():.0f}~{yaw.max():.0f}° (끝 {yaw[-1]:.0f}°) · 속도 중앙 {np.median(v[:,1]):.2f} 최대 {v[:,1].max():.2f} · 상태 바뀜 {sum(1 for x in rs if x[2]==lab)}개')
    for x in tl:
        if ts[0] - 1 <= x[0] <= ts[-1] + 1: pass
    seq = [(round(x[0], 1), x[3]) for i, x in enumerate(rs) if x[2] == lab and (i == 0 or rs[i - 1][3] != x[3] or rs[i - 1][2] != lab)]
    print('   상태:', seq)
    tls = [x for x in tl if ts[0] - 1 <= x[0] <= ts[-1] + 1]; print('   신호:', sorted(set(x[1].split('|')[0] for x in tls)), len(tls), '개')
np.save(OUT + '/clear.npy', R); json.dump({'rs': rs, 'goals': goals, 'cones': cones.tolist() if cones is not None else [], 'side': SIDE, 'seen': SEEN.tolist()}, open(OUT + '/meta.json', 'w'))
# 벽 지도(마지막 10장 합침, 5 cm 칸)
Wall = np.concatenate([w[1] for w in walls[::40]]); np.save(OUT + '/walls.npy', np.unique(np.round(Wall[:, :2] / 0.05).astype(int), axis=0) * 0.05)
