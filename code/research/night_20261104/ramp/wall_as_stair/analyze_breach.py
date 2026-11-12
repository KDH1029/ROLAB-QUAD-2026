#!/usr/bin/env python3
"""[벽 뚫림 판정] terrain_veto 재생 녹화본 — 원래 ego 장애물(팽창 0.40 m)로 막혀 있던 지도 벽 칸이 거부권 뒤 안 막히는(뚫린) 경우를 센다.
(원 설명) terrain_veto(C++) 재생 녹화본 분석 — 같은 순간의 /cloud_registered(원래 플래너 입력) vs /cloud_planner(거부권 후)에
ego_planner 규칙을 똑같이 적용해, 걸어간 길 위 헛장애물과 벽 유지를 비교. 기준 궤적 = 재생 FAST-LIO /Odometry.
사용: analyze_tv.py <태그>"""
import sys, math, bisect, json, numpy as np, rosbag2_py
from scipy.spatial import cKDTree
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
sys.path.insert(0, '/home/ilovehan9283/ws/v_gosu/backup'); sys.path.insert(0, '/home/ilovehan9283/ws/night_20261104/ramp')
from backup_core import q2R, rpy_of
from terrain_core import ego_rule
TAG = sys.argv[1]; D = f'/home/ilovehan9283/ws/night_20261104/ramp/replay/{TAG}.rec'
G10, CXX, CYY = 0.10, 4.3, -1.6
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=D, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
od, reg, pla = [], [], []
while r.has_next():
    tp, raw, t = r.read_next(); t *= 1e-9
    if tp == '/Odometry':
        m = deserialize_message(raw, Odometry); p, q = m.pose.pose.position, m.pose.pose.orientation
        od.append((t, p.x, p.y, p.z, rpy_of(q2R(q.x, q.y, q.z, q.w))[2]))
    elif tp in ('/cloud_registered', '/cloud_planner'):
        m = deserialize_message(raw, PointCloud2); st = round(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, 4)
        (reg if tp == '/cloud_registered' else pla).append((t, st, raw))
od = np.array(od); tO = od[:, 0]; t0 = tO[0]
def pose_at(ts): i = min(max(bisect.bisect_left(tO, ts), 0), len(tO) - 1); return od[i]
path = od[:, 1:3]; cum = np.r_[0, np.cumsum(np.hypot(*np.diff(path, axis=0).T))]
# 벽 정답(원 지도, 같은 출발 좌표계)
d = np.load('/home/ilovehan9283/ws/v_gosu/eval/out/gosu1030v2_map.npz'); FULL = (d['vox'][d['cnt'] >= 2].astype(np.float64) + 0.5) * float(d['V'])
kk = np.floor(FULL[:, :2] / G10).astype(np.int64); u, inv = np.unique(kk, axis=0, return_inverse=True); inv = inv.ravel()
cmin = np.full(len(u), np.inf); np.minimum.at(cmin, inv, FULL[:, 2])
nb = cKDTree(u * G10 + G10 / 2).query_ball_point(u * G10 + G10 / 2, 0.5)
floor = np.array([np.percentile(cmin[n], 5) for n in nb]); hgt = FULL[:, 2] - floor[inv]
WALL = set(map(tuple, np.unique(kk[(hgt > 0.30) & (hgt < 1.0)], axis=0)))
kw = np.array(sorted(WALL)); cmax = {}
for k_, z_ in zip(map(tuple, kk), FULL[:, 2]):
    pass
_ord = np.lexsort((FULL[:, 2], kk[:, 1], kk[:, 0])); _ks = kk[_ord]; _zs = FULL[_ord, 2]
_brk = np.r_[True, np.any(_ks[1:] != _ks[:-1], axis=1)]; _st = np.where(_brk)[0]; _en = np.r_[_st[1:], len(_ks)]
CMAX = {tuple(_ks[a_]): float(_zs[b_ - 1]) for a_, b_ in zip(_st, _en)}
_tr = cKDTree(od[:, 1:3]); _dd, _ii = _tr.query((kw + 0.5) * G10)
_zg = od[_ii, 3] - 0.31
_keep = np.array([CMAX.get(tuple(c), -9) - zg >= 0.25 for c, zg in zip(kw, _zg)]) & (_dd > 0.20)
WALL_ALL = WALL; WALL = set(map(tuple, kw[_keep]))
print('벽 정답 칸', len(WALL_ALL), '→ 걸어간 면(단 윗면·경사면) 뺀 진짜 벽', len(WALL))
def cells(xy): return set(map(tuple, np.floor(xy / G10).astype(np.int64)))
def dilate(cs): return {(a + i, b + j) for a, b in cs for i in (-1, 0, 1) for j in (-1, 0, 1)}
def corridor_ahead(t, L=2.0):
    i = min(max(bisect.bisect_left(tO, t), 0), len(tO) - 1); j = np.searchsorted(cum, cum[i] + L); seg = path[i:max(j, i + 1)]
    if len(seg) < 2: return set()
    g = np.floor(seg / G10).astype(np.int64); out = set()
    for a, b in np.unique(g, axis=0):
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                c = np.array([(a + di + 0.5) * G10, (b + dj + 0.5) * G10])
                if np.min(np.hypot(*(seg - c).T)) <= 0.12: out.add((a + di, b + dj))
    me = path[i]; return {c for c in out if math.hypot((c[0] + .5) * G10 - me[0], (c[1] + .5) * G10 - me[1]) > 0.35}
pla_by = {st: raw for _, st, raw in pla}
BIG = []
S = []; last = -1e9; lat = []
for t, st, raw in reg:
    if t - last < 0.5 or st not in pla_by: continue
    last = t; p = pose_at(t); rx, ry, rz, ryaw = p[1:5]
    A = pc2.read_points_numpy(deserialize_message(raw, PointCloud2), field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)
    B = pc2.read_points_numpy(deserialize_message(pla_by[st], PointCloud2), field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)
    ma = ego_rule(A, (rx, ry, rz), ryaw); mb = ego_rule(B, (rx, ry, rz), ryaw)
    PA, PB = A[ma, :2], B[mb, :2]
    near_w = [c for c in WALL if math.hypot((c[0] + .5) * G10 - rx, (c[1] + .5) * G10 - ry) <= 3.0]
    if not near_w or len(PA) == 0:
        continue
    WC = (np.array(near_w) + 0.5) * G10
    da, _ = cKDTree(PA).query(WC); covA = da <= 0.40
    db = cKDTree(PB).query(WC)[0] if len(PB) else np.full(len(WC), 9.0); covB = db <= 0.40
    lost = covA & ~covB
    # 뚫린 벽 칸이 이어진 최대 길이(10 cm 칸 연결)
    L = set(map(tuple, np.array(near_w)[lost])); best = 0; seen = set()
    for c in L:
        if c in seen: continue
        st = [c]; seen.add(c); n = 0
        while st:
            q = st.pop(); n += 1
            for i in (-1, 0, 1):
                for j in (-1, 0, 1):
                    r_ = (q[0] + i, q[1] + j)
                    if r_ in L and r_ not in seen: seen.add(r_); st.append(r_)
        best = max(best, n)
        if n == best and n >= 30: BIG.append((round(t - t0, 1), n, round(rx, 2), round(ry, 2), round(rz, 2)))
    S.append(dict(zone='X자' if math.hypot(rx - CXX, ry - CYY) < 2.5 else '평지', covA=int(covA.sum()), lost=int(lost.sum()), run=best))
out = {}
for z in ('X자', '평지', '전체'):
    Z = [s_ for s_ in S if z == '전체' or s_['zone'] == z]
    if not Z: continue
    lost = np.array([s_['lost'] for s_ in Z]); cov = np.array([s_['covA'] for s_ in Z]); run = np.array([s_['run'] for s_ in Z])
    out[z] = dict(표본=len(Z), 원래_막힌벽칸_평균=round(float(cov.mean()), 1), 뚫린벽칸_평균=round(float(lost.mean()), 2),
                  뚫린벽칸_비율=round(float(lost.sum() / max(cov.sum(), 1)), 4), 뚫림있는_표본비율=round(float((lost > 0).mean()), 3),
                  뚫림_연속최대_칸=int(run.max()), 뚫림_연속3칸이상_표본=int((run >= 3).sum()))
print('30칸 넘는 뚫림 덩어리(시각 s, 칸, 로봇 x,y,z):', BIG[:15], '… 총', len(BIG))
print(json.dumps(out, ensure_ascii=False, indent=1)); json.dump(out, open(f'/home/ilovehan9283/ws/night_20261104/ramp/replay/{TAG}.breach2.json', 'w'), ensure_ascii=False, indent=1)
