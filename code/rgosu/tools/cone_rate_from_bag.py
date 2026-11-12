#!/usr/bin/env python3
"""녹화(bag)만으로 라바콘 인식률 재기 — 재생 없음(파일만 읽음, 켜진 로봇과 안 섞임).
사용: python3 cone_rate_from_bag.py <녹화폴더> [출력 png]

하는 일
 1. /lidar_points(XT16, 라이다 좌표) + /utlidar/robot_odom(몸체 자세)로 매 프레임 세계 좌표 점을 만든다.
    라이다→몸체 외부파라미터는 FAST-LIO 설정값(R 은 z축 약 +90°, T=[0.25,0,0.1588]).
 2. 바닥 위 4~45 cm 점을 쌓아 덩어리로 나누고, 발자국 40 cm 이하 덩어리를 콘으로 본다(벽은 길어서 빠짐).
 3. 프레임마다 각 콘이 ① 원시 라이다에 보였나 ② wall_filter_node.cpp 알고리즘(그대로 옮김)을 통과했나를 센다.
    wall_filter 는 두 가지로 돌린다:
      (가) 지금 코드 그대로 — 라이다 좌표 점에 몸체 roll/pitch/yaw 를 바로 곱함(외부파라미터 없음)
      (나) 외부파라미터를 넣어 몸체 좌표로 바꾼 뒤 같은 알고리즘
    (가)가 실제 벽과 안 맞으면 → 지금 ego_planner 가 보는 벽·콘이 어긋나 있다는 뜻.
"""
import sys, math, json
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2 as pc2

B = sys.argv[1].rstrip('/')
PNG = sys.argv[2] if len(sys.argv) > 2 else B + '/cone_rate.png'
R_L = np.array([[-0.0001707, -0.9999240, 0.0123287], [0.9999844, -0.0001019, 0.0055807], [-0.0055790, 0.0123295, 0.9999084]])
T_L = np.array([0.25, 0.0, 0.1588])
TOL, MIN_RINGS, MIN_SPAN, N_AZ, RMIN, RMAX = 0.03, 3, 0.08, 360, 0.25, 10.0   # wall_filter 기본값
BAND = (0.04, 0.45)


def rot(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]); Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]); Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz, Ry @ Rx


def q2rpy(w, x, y, z):
    return (math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)), math.asin(max(-1, min(1, 2 * (w * y - z * x)))),
            math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def wall_filter(P_level, ring, n_ring=16):
    """wall_filter_node.cpp onCloud 의 판정부를 그대로 옮김. 입력은 '수평보정된' 점. 출력: 수평보정 좌표의 벽 점들."""
    r = np.hypot(P_level[:, 0], P_level[:, 1]); m = (r >= RMIN) & (r < RMAX) & (ring < n_ring)
    P, r, ring = P_level[m], r[m], ring[m]
    a = np.clip(((np.arctan2(P[:, 1], P[:, 0]) + math.pi) / (2 * math.pi / N_AZ)).astype(int), 0, N_AZ - 1)
    rh = np.full((N_AZ, n_ring), np.inf); zz = np.zeros((N_AZ, n_ring))
    order = np.argsort(-r)   # 가까운 것이 마지막에 써지게 → 칸마다 최소 거리
    rh[a[order], ring[order]] = r[order]; zz[a[order], ring[order]] = P[order, 2]
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
                az = -math.pi + (ai + 0.5) * 2 * math.pi / N_AZ; rm = 0.5 * (lo + hi)
                out.append((rm * math.cos(az), rm * math.sin(az), 0.5 * (zlo + zhi))); i = j
            else:
                i += 1
    return np.array(out).reshape(-1, 3)


rd = rosbag2_py.SequentialReader(); rd.open(rosbag2_py.StorageOptions(uri=B, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
rd.set_filter(rosbag2_py.StorageFilter(topics=['/lidar_points', '/utlidar/robot_odom']))
OD, PC = get_message('nav_msgs/msg/Odometry'), get_message('sensor_msgs/msg/PointCloud2')
odom, clouds, k = [], [], 0
while rd.has_next():
    tp, raw, t = rd.read_next()
    if tp == '/utlidar/robot_odom':
        m = deserialize_message(raw, OD); p, q = m.pose.pose.position, m.pose.pose.orientation
        odom.append((t * 1e-9, p.x, p.y, p.z) + q2rpy(q.w, q.x, q.y, q.z))
    else:
        k += 1
        if k % 2: continue   # 10 Hz 로 충분
        m = deserialize_message(raw, PC); a = pc2.read_points(m, field_names=['x', 'y', 'z', 'ring'], skip_nans=True)
        clouds.append((t * 1e-9, np.stack([a['x'], a['y'], a['z']], 1).astype(np.float64), np.asarray(a['ring']).astype(int)))
odom = np.array(odom); ot = odom[:, 0]
el_by_ring = None
frames = []
for t, L, ring in clouds:
    i = min(np.searchsorted(ot, t), len(ot) - 1); tx, ty, tz, roll, pitch, yaw = odom[i, 1:]
    Rz, Rlev = rot(roll, pitch, yaw); t_w = np.array([tx, ty, tz])
    if el_by_ring is None:
        el = np.degrees(np.arctan2(L[:, 2], np.hypot(L[:, 0], L[:, 1])))
        el_by_ring = [round(float(np.median(el[ring == g])), 1) for g in range(16)]
    Bd = L @ R_L.T + T_L; Lv = Bd @ Rlev.T            # 몸체 좌표 → 수평보정
    rr = np.hypot(Lv[:, 0], Lv[:, 1]); near = (rr > 0.5) & (rr < 3.0)
    zs = Lv[near, 2]; hist, edges = np.histogram(zs, bins=np.arange(-1.0, 0.3, 0.02)); floor = edges[hist.argmax()] + 0.01
    W = Lv @ Rz.T + t_w; hgt = Lv[:, 2] - floor
    band = (hgt > BAND[0]) & (hgt < BAND[1]) & (rr > 0.3) & (rr < 6.0)
    tall = (hgt >= BAND[1]) & (hgt < 1.2) & (rr > 0.3) & (rr < 6.0)
    wf_now = wall_filter((L @ Rlev.T), ring) @ Rz.T + t_w                  # (가) 지금 코드: 라이다 좌표에 몸체 자세
    # (나) 11-04 수정판 C++ 과 똑같이: 점 = Rlev·R_L·p(원점은 라이다), 출력 = Rz·(q + Rlev·T) + t
    wf_fix = (wall_filter(L @ R_L.T @ Rlev.T, ring) + Rlev @ T_L) @ Rz.T + t_w
    frames.append(dict(t=t, pos=t_w[:2], yaw=yaw, band=W[band, :2], tall=W[tall, :2], wf_now=wf_now[:, :2], wf_fix=wf_fix[:, :2], floor=floor))

# 콘 찾기: 쌓인 높이밴드 점 → 4 cm 격자 → 덩어리
G = 0.04
allb = np.concatenate([f['band'] for f in frames]); kk = np.floor(allb / G).astype(int)
cells, cnt = np.unique(kk, axis=0, return_counts=True); occ = {tuple(c): n for c, n in zip(cells, cnt) if n >= 8}
seen, comps = set(), []
for c in occ:
    if c in seen: continue
    st, comp = [c], []
    seen.add(c)
    while st:
        u = st.pop(); comp.append(u)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                v = (u[0] + dx, u[1] + dy)
                if v in occ and v not in seen: seen.add(v); st.append(v)
    comps.append(np.array(comp))
allt = np.concatenate([f['tall'] for f in frames]); tk, tc = np.unique(np.floor(allt / G).astype(int), axis=0, return_counts=True)
tallc = {tuple(c) for c, n in zip(tk, tc) if n >= 8}   # 45 cm 위에도 점이 꾸준히 있는 칸 = 벽·사람·구조물
def isolated(comp, gap=5):   # 덩어리 둘레 20 cm 안에 높은 칸이 없어야 콘(벽에 붙은 조각 제외)
    return not any((u[0] + dx, u[1] + dy) in tallc for u in comp for dx in range(-gap, gap + 1) for dy in range(-gap, gap + 1))
cones = []
for c in comps:
    ext = (c.max(0) - c.min(0) + 1) * G
    if 0.08 <= ext.max() <= 0.32 and len(c) >= 4 and isolated(c):
        w = np.array([occ[tuple(u)] for u in c]); cones.append(((c + 0.5) * G * w[:, None]).sum(0) / w.sum())
cones = np.array(cones).reshape(-1, 2)


def near(pts, c, d):
    return len(pts) and (np.hypot(*(pts - c).T) < d).sum()


stat = {'원시': 0, '지금wall_filter': 0, '고친wall_filter': 0}
byr = {}
for f in frames:
    for c in cones:
        d = float(np.hypot(*(c - f['pos'])))
        if d > 5 or d < 0.3: continue
        if near(f['band'], c, 0.2) >= 3:
            stat['원시'] += 1; b = '%.0f~%.0f m' % (math.floor(d), math.floor(d) + 1); byr.setdefault(b, [0, 0, 0]); byr[b][0] += 1
            if near(f['wf_now'], c, 0.25): stat['지금wall_filter'] += 1; byr[b][1] += 1
            if near(f['wf_fix'], c, 0.25): stat['고친wall_filter'] += 1; byr[b][2] += 1

# (가)가 실제 벽과 맞는가: wall_filter 점이 쌓인 원시 점유칸 근처(10 cm)에 있는 비율
def align(key):
    P = np.concatenate([f[key] for f in frames]); kk = np.floor(P / G).astype(int); ok = 0
    for u in kk:
        if any((u[0] + dx, u[1] + dy) in occ for dx in (-2, -1, 0, 1, 2) for dy in (-2, -1, 0, 1, 2)): ok += 1
    return ok / max(1, len(P)), len(P)

a_now, n_now = align('wf_now'); a_fix, n_fix = align('wf_fix')
res = {'프레임': len(frames), '시간_s': round(frames[-1]['t'] - frames[0]['t'], 1), '링별 고도(°)': el_by_ring,
       '콘 후보': [[round(x, 2), round(y, 2)] for x, y in cones],
       '콘이 원시 라이다에 보인 (프레임×콘)': stat['원시'],
       '지금 wall_filter 통과율': '%.1f%%' % (100 * stat['지금wall_filter'] / max(1, stat['원시'])),
       '외부파라미터 넣은 wall_filter 통과율': '%.1f%%' % (100 * stat['고친wall_filter'] / max(1, stat['원시'])),
       '거리별 [보임, 지금통과, 고친통과]': byr,
       '벽점이 실제 구조물 위에 있는 비율 (지금 / 고친)': '%.1f%% / %.1f%%' % (100 * a_now, 100 * a_fix)}
print(json.dumps(res, ensure_ascii=False, indent=1))
json.dump(res, open(PNG.replace('.png', '.json'), 'w'), ensure_ascii=False, indent=1)

import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
plt.rcParams['font.family'] = ['NanumGothic', 'DejaVu Sans']
fig, ax = plt.subplots(1, 2, figsize=(18, 8))
pos = np.array([f['pos'] for f in frames])
for a_, key, ttl in ((ax[0], 'wf_now', '지금 wall_filter 출력(빨강) — ego_planner 가 보는 벽'), (ax[1], 'wf_fix', '외부파라미터 넣은 wall_filter(빨강)')):
    cc = np.array(list(occ.keys())) * G
    a_.scatter(cc[:, 0], cc[:, 1], s=1, c='0.6', label='바닥 위 4~45 cm 라이다(누적)')
    P = np.concatenate([f[key] for f in frames[::5]]); a_.scatter(P[:, 0], P[:, 1], s=1, c='r', alpha=0.4)
    a_.plot(pos[:, 0], pos[:, 1], 'b-', lw=1, label='로봇 경로')
    if len(cones): a_.scatter(cones[:, 0], cones[:, 1], s=120, facecolors='none', edgecolors='orange', lw=2, label='찾은 콘')
    a_.set_title(ttl); a_.axis('equal'); a_.grid(alpha=0.3); a_.legend(loc='upper right', fontsize=8)
plt.tight_layout(); plt.savefig(PNG, dpi=90); print('그림', PNG)
