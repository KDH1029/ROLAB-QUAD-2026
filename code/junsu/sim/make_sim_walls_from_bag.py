#!/usr/bin/env python3
"""시뮬레이션 장애물 = 실제 스택이 보는 것. 녹화 라이다 프레임마다 '고친 wall_filter'(11-04 18:26 C++ 과 같은 계산)를 돌려 쌓는다.
사용: python3 make_sim_walls_from_bag.py <녹화폴더> <FAST-LIO 지도 npz(같은 녹화)> <출력 npy>
자세: 프레임별 자세는 /utlidar/robot_odom(몸체) 를 쓰고, 그 좌표계를 FAST-LIO 경로(=저장 좌표와 같은 좌표계)에 2D 강체로 맞춘다."""
import sys, math, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2 as pc2
from scipy.spatial import cKDTree
B, NPZ, OUT = sys.argv[1].rstrip('/'), sys.argv[2], sys.argv[3]; MAXR = float(sys.argv[4]) if len(sys.argv) > 4 else 4.0
R_L = np.array([[-0.0001707, -0.9999240, 0.0123287], [0.9999844, -0.0001019, 0.0055807], [-0.0055790, 0.0123295, 0.9999084]]); T_L = np.array([0.25, 0.0, 0.1588])
TOL, MIN_RINGS, MIN_SPAN, N_AZ, RMIN, RMAX = 0.03, 3, 0.08, 360, 0.25, 10.0
def rot(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]), np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
def q2rpy(w, x, y, z):
    return math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)), math.asin(max(-1, min(1, 2 * (w * y - z * x)))), math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
def wall_filter(P, ring, n_ring=16):
    r = np.hypot(P[:, 0], P[:, 1]); m = (r >= RMIN) & (r < RMAX) & (ring < n_ring); P, r, ring = P[m], r[m], ring[m]
    a = np.clip(((np.arctan2(P[:, 1], P[:, 0]) + math.pi) / (2 * math.pi / N_AZ)).astype(int), 0, N_AZ - 1)
    rh = np.full((N_AZ, n_ring), np.inf); zz = np.zeros((N_AZ, n_ring)); o = np.argsort(-r); rh[a[o], ring[o]] = r[o]; zz[a[o], ring[o]] = P[o, 2]
    out = []
    for ai in range(N_AZ):
        rr, z = rh[ai], zz[ai]; i = 0
        while i < n_ring:
            if not np.isfinite(rr[i]): i += 1; continue
            lo = hi = rr[i]; zlo = zhi = z[i]; j = i + 1
            while j < n_ring and np.isfinite(rr[j]):
                nlo, nhi = min(lo, rr[j]), max(hi, rr[j])
                if nhi - nlo > TOL: break
                lo, hi, zlo, zhi = nlo, nhi, min(zlo, z[j]), max(zhi, z[j]); j += 1
            if j - i >= MIN_RINGS and zhi - zlo >= MIN_SPAN:
                az = -math.pi + (ai + 0.5) * 2 * math.pi / N_AZ; rm = 0.5 * (lo + hi); out.append((rm * math.cos(az), rm * math.sin(az), 0.5 * (zlo + zhi))); i = j
            else: i += 1
    return np.array(out).reshape(-1, 3)
rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=B, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
rd.set_filter(rosbag2_py.StorageFilter(topics=['/lidar_points', '/utlidar/robot_odom'])); OD, PC = get_message('nav_msgs/msg/Odometry'), get_message('sensor_msgs/msg/PointCloud2')
odom, outs, k = [], [], 0
pending = []
while rd.has_next():
    tp, raw, t = rd.read_next()
    if tp == '/utlidar/robot_odom':
        m = deserialize_message(raw, OD); p, q = m.pose.pose.position, m.pose.pose.orientation; odom.append((t * 1e-9, p.x, p.y, p.z) + q2rpy(q.w, q.x, q.y, q.z))
    else:
        k += 1
        if k % 2: continue   # 10 Hz (가까운 점만 쓰니 더 자주)
        m = deserialize_message(raw, PC); a = pc2.read_points(m, field_names=['x', 'y', 'z', 'ring'], skip_nans=True)
        pending.append((t * 1e-9, np.stack([a['x'], a['y'], a['z']], 1).astype(np.float64), np.asarray(a['ring']).astype(int)))
odom = np.array(odom); ot = odom[:, 0]
for t, L, ring in pending:
    i = min(np.searchsorted(ot, t), len(ot) - 1); tx, ty, tz, rr_, pp, yy = odom[i, 1:]; Rz, Rlev = rot(rr_, pp, yy)
    q = wall_filter(L @ R_L.T @ Rlev.T, ring)
    # ★ 11-05: 1~4 m 안 점만 쌓는다 — 스캔 하나 기준 벽 오차 중앙 1.7 cm·p90 2.6 cm(가까운 벽은 정확), 먼 점은 각도 오차로 번져 벽이 30 cm 넘게 두꺼워졌다
    if len(q): q = q[(np.hypot(q[:, 0], q[:, 1]) >= 1.0) & (np.hypot(q[:, 0], q[:, 1]) <= MAXR)]
    if len(q): w = (q + Rlev @ T_L) @ Rz.T + np.array([tx, ty, tz]); outs.append(np.c_[w, np.full(len(w), t)])
W = np.concatenate(outs); print('벽 점(쌓기 전)', len(W), '프레임', len(pending))
# robot_odom 좌표 → FAST-LIO(저장 좌표) 좌표: 두 경로를 2D 강체로 맞춘다
d = np.load(NPZ); fo = d['odom']; fo = fo[np.argsort(fo[:, 1])]; F = fo[:, 2:4]; Rp = odom[:, 1:3]
best = None
for th0 in np.radians(np.arange(-180, 180, 5)):
    th = th0; t = F[0] - Rp[0] @ np.array([[math.cos(th), math.sin(th)], [-math.sin(th), math.cos(th)]])
    for _ in range(40):
        c, s = math.cos(th), math.sin(th); Q = Rp[::10] @ np.array([[c, s], [-s, c]]) + t
        dd, idx = cKDTree(F).query(Q); A = Q - Q.mean(0); Bm = F[idx] - F[idx].mean(0)
        U, S, Vt = np.linalg.svd(A.T @ Bm); Rr = (U @ Vt).T; dth = math.atan2(Rr[1, 0], Rr[0, 0]); th += dth
        c, s = math.cos(th), math.sin(th); t = F[idx].mean(0) - (Rp[::10] @ np.array([[c, s], [-s, c]])).mean(0)
    c, s = math.cos(th), math.sin(th); dd, _ = cKDTree(F).query(Rp[::10] @ np.array([[c, s], [-s, c]]) + t); e = np.median(dd)
    if best is None or e < best[0]: best = (e, th, t)
e, th, t = best; c, s = math.cos(th), math.sin(th)
print(f'robot_odom→FAST-LIO 맞춤: 회전 {math.degrees(th):.1f}° 이동 ({t[0]:.2f},{t[1]:.2f}) 경로 차 중앙 {e*100:.1f} cm')
Wxy = W[:, :2] @ np.array([[c, s], [-s, c]]) + t
# 4 cm 칸마다 '서로 다른 10초 구간' 몇 개에서 보였나 — 벽·계단처럼 고정된 것은 바퀴 내내 여러 구간, 따라 걷는 사람은 잠깐
G = 0.04; kk = np.floor(Wxy / G).astype(np.int64); tb = ((W[:, 3] - W[:, 3].min()) // 10).astype(np.int64)
pairs = np.unique(np.c_[kk, tb], axis=0); cell, nwin = np.unique(pairs[:, :2], axis=0, return_counts=True)
keep = nwin >= 3; cells = (cell[keep] + 0.5) * G
dd, _ = cKDTree(F).query(cells); onpath = dd < 0.20
print(f'고정 벽 칸 {keep.sum()} (지나간 경로 20 cm 안에 있는 고정 칸 {onpath.sum()} = 계단 턱 등 길 위에 벽으로 판정되는 것)')
np.save(OUT.replace('.npy', '_onpath.npy'), cells[onpath])
cells = cells[~onpath]   # 시뮬레이션 장애물에서는 로봇이 실제로 지나간 자리는 뺀다(따로 보고)
walls = np.concatenate([np.c_[cells, np.full(len(cells), z)] for z in (0.15, 0.35)]).astype(np.float32)
np.save(OUT, walls); print('벽 칸', keep.sum(), '→', OUT)
