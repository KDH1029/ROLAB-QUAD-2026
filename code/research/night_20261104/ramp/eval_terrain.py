#!/usr/bin/env python3
"""실데이터 평가 (10-30 녹화본) — ego_planner 규칙 vs 지형 판정, 같은 입력.

정답 없이 쓸 수 있는 두 잣대
  ① 헛장애물: 로봇이 '실제로 걸어간' 통로(FAST-LIO 궤적 ±0.12 m) 위에 찍힌 장애물 칸. 걸어 지나간 곳이니 전부 오탐.
     로봇 앞 경로 2 m 안만 센다(지금 당장 길을 막는가).
  ② 벽 유지: 전체 지도에서 '바닥+0.30~1.0 m 에 점이 있는 칸'(벽·단상 옆면) 중, ego 규칙이 지금 잡은 것을 지형 판정도 잡는 비율(±1칸).
입력
  XT16: /lidar_points 원본을 FAST-LIO 자세로 세계 좌표(=/cloud_registered 근사, deskew 없음). 단일 스캔(ego 와 같음) / 0.5초 누적
  L1 높이지도: /utlidar/height_map_array (Unitree 공식, 로봇 내부 계산, odom=다리 오도 좌표, 6 cm 128×128) — 모양으로 판정 후 xy 만 세계로
쓰기: 이 폴더(eval_terrain.json, eval_terrain_samples.npy)
"""
import sys, math, bisect, json, numpy as np, rosbag2_py, warnings
warnings.simplefilter('ignore')
from scipy.spatial import cKDTree
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
from unitree_go.msg import HeightMap
sys.path.insert(0, '/home/ilovehan9283/ws/v_gosu/backup'); sys.path.insert(0, '/home/ilovehan9283/ws/night_20261104/ramp')
from backup_core import q2R, rpy_of, R_LIDAR, T_LIDAR
from terrain_core import ego_rule, elevation, classify, grid_stats, traversable, veto
import os
METHODS = os.environ.get('METHODS', 'all')

BAG = '/home/ilovehan9283/bags/gosu_20261030_001126'
NPZ = '/home/ilovehan9283/ws/v_gosu/eval/out/gosu1030v2_map.npz'
OUT = '/home/ilovehan9283/ws/night_20261104/ramp'
DT = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
CELL, HALF = 0.05, 4.0          # 지형 격자 5 cm, 로봇 중심 8×8 m
G10 = 0.10                      # 평가 칸 (ego 격자와 같은 10 cm)
CX, CY = 4.3, -1.6              # X자 가운데 단


def reader(topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics)); return r


d = np.load(NPZ); odom, lid = d['odom'], d['lid']
FULL = (d['vox'][d['cnt'] >= 2].astype(np.float64) + 0.5) * float(d['V'])
hdr2bag = {}
r = reader(['/lidar_points'])
while r.has_next():
    tp, raw, t = r.read_next(); s, ns = np.frombuffer(raw[4:8], '<i4')[0], np.frombuffer(raw[8:12], '<u4')[0]
    hdr2bag[round(float(s) + float(ns) * 1e-9, 4)] = t * 1e-9
pairs = np.array([(w, hdr2bag[round(h, 4)]) for w, h in lid if round(h, 4) in hdr2bag])
Kp = float(np.median(pairs[:, 0] - pairs[:, 1])); wl = np.sort(lid[:, 0]); h_o = odom[:, 1]
dmed = float(np.median(h_o - wl[np.clip(np.searchsorted(wl, h_o) - 1, 0, len(wl) - 1)]))
tL = h_o - dmed - Kp; o = np.argsort(tL); tL = tL[o]; GT = odom[o]; t0 = tL[0]
def gt_at(ts): return GT[min(max(bisect.bisect_left(tL, ts), 0), len(tL) - 1)]

# 통로(걸어간 곳): 10 cm 칸, 궤적 ±0.12 m
path = GT[:, 2:4]; ktree = cKDTree(path)
cum = np.r_[0, np.cumsum(np.hypot(*np.diff(path, axis=0).T))]
# 벽 정답 칸: 지도 점 중 지역 바닥(반경 0.5 m 최저 5%)+0.30 ~ +1.0
kk = np.floor(FULL[:, :2] / G10).astype(np.int64); u, inv = np.unique(kk, axis=0, return_inverse=True); inv = inv.ravel()
cmin = np.full(len(u), np.inf); np.minimum.at(cmin, inv, FULL[:, 2])
ut = cKDTree(u * G10 + G10 / 2); nb = ut.query_ball_point(u * G10 + G10 / 2, 0.5)
floor = np.array([np.percentile(cmin[n], 5) for n in nb])
hgt = FULL[:, 2] - floor[inv]; wallk = np.unique(kk[(hgt > 0.30) & (hgt < 1.0)], axis=0)
WALL = set(map(tuple, wallk)); print('벽 정답 칸', len(WALL))


def cells(xy):
    return set(map(tuple, np.floor(xy / G10).astype(np.int64)))


def dilate(cs):
    return {(a + i, b + j) for a, b in cs for i in (-1, 0, 1) for j in (-1, 0, 1)}


def corridor_ahead(t, L=2.0):
    i = min(max(bisect.bisect_left(tL, t), 0), len(tL) - 1); j = np.searchsorted(cum, cum[i] + L)
    seg = path[i:max(j, i + 1)]
    if len(seg) < 2: return set()
    # 궤적 ±0.12 m 칸
    g = np.floor(seg / G10).astype(np.int64); out = set()
    for a, b in np.unique(g, axis=0):
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                c = np.array([(a + di + 0.5) * G10, (b + dj + 0.5) * G10])
                if np.min(np.hypot(*(seg - c).T)) <= 0.12: out.add((a + di, b + dj))
    # 로봇 자기 위치 0.35 m 안은 뺀다(ego 자기 필터 구역과 겹침)
    me = path[i]; return {c for c in out if math.hypot((c[0] + .5) * G10 - me[0], (c[1] + .5) * G10 - me[1]) > 0.35}


def terrain_cells(W, center, zmax):
    org = center - HALF; n = int(2 * HALF / CELL)
    E = elevation(W, org, (n, n), CELL, zmax); obs, s, p = classify(E, CELL)
    ij = np.argwhere(obs); xy = org + (ij + 0.5) * CELL
    return xy


# 사건 모으기
ev = []
r = reader(['/lidar_points', '/utlidar/robot_odom', '/utlidar/height_map_array'])
buf = []; last_s = -1e9; leg = None; hm = None
S = []   # 표본
while r.has_next():
    tp, raw, t = r.read_next(); t *= 1e-9
    if tp == '/utlidar/robot_odom':
        m = deserialize_message(raw, Odometry); p, q = m.pose.pose.position, m.pose.pose.orientation
        leg = (p.x, p.y, p.z, rpy_of(q2R(q.x, q.y, q.z, q.w))[2]); continue
    if tp == '/utlidar/height_map_array':
        hm = (t, raw); continue
    if t < tL[0] + 2 or t > tL[-1]: continue
    # XT16: 매 프레임 누적 버퍼(0.5초)
    m = deserialize_message(raw, PointCloud2)
    a = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)
    B = a @ R_LIDAR.T + T_LIDAR; rr = np.hypot(B[:, 0], B[:, 1]); B = B[(rr > 0.3) & (rr < 12)]
    g = gt_at(t); Rw = q2R(*g[5:9]); Wt = B @ Rw.T + g[2:5]
    buf.append((t, Wt)); buf = [b for b in buf if t - b[0] <= 0.5]
    if t - last_s < DT or leg is None or hm is None: continue
    last_s = t
    rx, ry, rz = g[2:5]; ryaw = rpy_of(Rw)[2]; me = np.array([rx, ry])
    ahead = corridor_ahead(t)
    zmax = rz + 1.10
    res = {}
    # ego 규칙 (단일 스캔)
    em = ego_rule(Wt, (rx, ry, rz), ryaw); ego_c = cells(Wt[em, :2])
    res['ego'] = ego_c
    Wacc = np.concatenate([b[1] for b in buf])
    # 거부권: ego 장애물 점 중 '발밑과 이어진 매끈한 면' 위의 것을 지운다 (높이 격자 = XT16 0.5초 누적 / 단일)
    org = me - HALF; nn = int(2 * HALF / CELL)
    for nm, src in (('veto_acc', Wacc), ('veto_1', Wt)):
        hi, lo, _ = grid_stats(src, org, (nn, nn), CELL, zmax); Tm = traversable(hi, lo, CELL, me, org, rz)
        res[nm] = cells(Wt[em & ~veto(Wt, Tm, org, CELL), :2])
    if METHODS == 'veto':
        pass
    else:
        # 지형 판정 단독: XT16 단일 / 0.5초 누적
        res['xt_1'] = cells(terrain_cells(Wt, me, zmax))
        res['xt_acc'] = cells(terrain_cells(Wacc, me, zmax))
    # L1 높이지도 (Unitree): 자기 격자(6 cm)에서 판정 → 다리 오도 → 세계 (xy)
    if METHODS != 'veto':
        h = deserialize_message(hm[1], HeightMap)
        D = np.array(h.data, dtype=np.float32).reshape(h.height, h.width).T  # [ix, iy]
        D = np.where(np.abs(D) < 1e3, D, np.nan).astype(np.float64)
        lx, ly, lz, lyaw = leg; D = np.where(D <= lz + 1.10, D, np.nan)
        obs, _, _ = classify(D, float(h.resolution))
        ij = np.argwhere(obs); po = np.array(h.origin) + (ij + 0.5) * h.resolution          # odom xy
        c1, s1 = math.cos(lyaw), math.sin(lyaw); dxo, dyo = po[:, 0] - lx, po[:, 1] - ly
        bx, by = c1 * dxo + s1 * dyo, -s1 * dxo + c1 * dyo                                  # 몸체
        c2, s2 = math.cos(ryaw), math.sin(ryaw); wxy = np.c_[rx + c2 * bx - s2 * by, ry + s2 * bx + c2 * by]
        res['l1_hmap'] = cells(wxy[np.hypot(bx, by) > 0.35])
        res['xt_acc+l1'] = res['xt_acc'] | res['l1_hmap']
    zone = 'X자' if math.hypot(rx - CX, ry - CY) < 2.5 else '평지'
    near = lambda cs: {c for c in cs if math.hypot((c[0] + .5) * G10 - rx, (c[1] + .5) * G10 - ry) <= 3.0}
    wall_ego = near(ego_c) & WALL
    row = dict(t=t - t0, zone=zone, z=rz)
    for k, cs in res.items():
        row[k + '_헛'] = len(cs & ahead)
        row[k + '_벽유지'] = (len(dilate(near(cs)) & wall_ego) / len(wall_ego)) if wall_ego else np.nan
        row[k + '_칸'] = len(near(cs))
    S.append(row)
    if len(S) % 100 == 0: print(len(S), f'{t - t0:.0f}s', {k: row[k + '_헛'] for k in res}, flush=True)

keys = [k for k in S[0] if k.endswith('_헛')]
summ = {}
for zone in ('X자', '평지', '전체'):
    Z = [s for s in S if zone == '전체' or s['zone'] == zone]
    summ[zone] = {'표본': len(Z)}
    for k in keys:
        m = k[:-2]; v = np.array([s[k] for s in Z]); w = np.array([s[m + '_벽유지'] for s in Z], dtype=float)
        summ[zone][m] = dict(헛장애물_표본비율=round(float((v > 0).mean()), 3), 헛칸_평균=round(float(v.mean()), 2),
                             벽유지=round(float(np.nanmean(w)), 3), 칸수_3m=round(float(np.mean([s[m + '_칸'] for s in Z])), 1))
print(json.dumps(summ, ensure_ascii=False, indent=1))
json.dump(summ, open(f'{OUT}/eval_terrain_{METHODS}.json', 'w'), ensure_ascii=False, indent=1)
np.save(f'{OUT}/eval_terrain_samples_{METHODS}.npy', np.array([[s['t'], s['z']] + [s[k] for k in keys] for s in S]))
