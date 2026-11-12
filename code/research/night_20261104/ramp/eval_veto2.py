#!/usr/bin/env python3
"""거부권 2차 평가 — 높이 격자에 L1(/utlidar/cloud_base)·D435i 깊이를 더하고, 씨앗을 '발밑 지면 평면' 으로.
1차(eval_veto.log): XT16 만으론 바닥이 1.75 m 밖부터 찍혀 씨앗이 없어 거부권이 사실상 0.
ego 입력은 두 가지:  ① XT16 단일 스캔(지금 플래너)  ② XT16 + 깊이(융합 입력) — 깊이가 가까운 경사면을 새 헛장애물로 만드는지도 본다.
"""
import sys, math, bisect, json, numpy as np, rosbag2_py, warnings, os
warnings.simplefilter('ignore')
from scipy.spatial import cKDTree
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs_py.point_cloud2 as pc2
sys.path.insert(0, '/home/ilovehan9283/ws/v_gosu/backup'); sys.path.insert(0, '/home/ilovehan9283/ws/night_20261104/ramp')
from backup_core import q2R, rpy_of, R_LIDAR, T_LIDAR
from terrain_core import ego_rule, grid_stats, traversable, veto
BAG = '/home/ilovehan9283/bags/gosu_20261030_001126'
NPZ = '/home/ilovehan9283/ws/v_gosu/eval/out/gosu1030v2_map.npz'
OUT = '/home/ilovehan9283/ws/night_20261104/ramp'
CAL = json.load(open('/home/ilovehan9283/ws/night_20261104/vision/calib_depth_all.json'))
TBC = np.array(CAL['T_body_cam_optical']); S2N = CAL['stamp_to_nuc']
FX = FY = 431.08; CX, CY = 422.28, 240.89
DT = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
HOP = float(os.environ.get('HOP', 0.15)); H_STAND = 0.31
CELL, HALF, G10, CXX, CYY = 0.05, 4.0, 0.10, 4.3, -1.6


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
path = GT[:, 2:4]; cum = np.r_[0, np.cumsum(np.hypot(*np.diff(path, axis=0).T))]
kk = np.floor(FULL[:, :2] / G10).astype(np.int64); u, inv = np.unique(kk, axis=0, return_inverse=True); inv = inv.ravel()
cmin = np.full(len(u), np.inf); np.minimum.at(cmin, inv, FULL[:, 2])
nb = cKDTree(u * G10 + G10 / 2).query_ball_point(u * G10 + G10 / 2, 0.5)
floor = np.array([np.percentile(cmin[n], 5) for n in nb]); hgt = FULL[:, 2] - floor[inv]
WALL = set(map(tuple, np.unique(kk[(hgt > 0.30) & (hgt < 1.0)], axis=0)))
def cells(xy): return set(map(tuple, np.floor(xy / G10).astype(np.int64)))
def dilate(cs): return {(a + i, b + j) for a, b in cs for i in (-1, 0, 1) for j in (-1, 0, 1)}
def corridor_ahead(t, L=2.0):
    i = min(max(bisect.bisect_left(tL, t), 0), len(tL) - 1); j = np.searchsorted(cum, cum[i] + L); seg = path[i:max(j, i + 1)]
    if len(seg) < 2: return set()
    g = np.floor(seg / G10).astype(np.int64); out = set()
    for a, b in np.unique(g, axis=0):
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                c = np.array([(a + di + 0.5) * G10, (b + dj + 0.5) * G10])
                if np.min(np.hypot(*(seg - c).T)) <= 0.12: out.add((a + di, b + dj))
    me = path[i]; return {c for c in out if math.hypot((c[0] + .5) * G10 - me[0], (c[1] + .5) * G10 - me[1]) > 0.35}
uu, vv = np.meshgrid(np.arange(0, 848, 4), np.arange(0, 480, 4))
def depth_world(raw, tn):
    m = deserialize_message(raw, Image); D = np.frombuffer(m.data, np.uint16).reshape(480, 848)[::4, ::4] * 1e-3
    ok = (D > 0.3) & (D < 2.5); z = D[ok]; P = np.c_[(uu[ok] - CX) * z / FX, (vv[ok] - CY) * z / FY, z]
    g = gt_at(tn); return (P @ TBC[:3, :3].T + TBC[:3, 3]) @ q2R(*g[5:9]).T + g[2:5]

xbuf, lbuf, dbuf = [], [], []; last_s = -1e9; S = []; dn = 0
r = reader(['/lidar_points', '/utlidar/cloud_base', '/camera/depth/image_rect_raw'])
while r.has_next():
    tp, raw, t = r.read_next(); t *= 1e-9
    if t < tL[0] + 2 or t > tL[-1]: continue
    if tp == '/camera/depth/image_rect_raw':
        dn += 1
        if dn % 3: continue
        s_, ns_ = np.frombuffer(raw[4:8], '<i4')[0], np.frombuffer(raw[8:12], '<u4')[0]; st = float(s_) + ns_ * 1e-9
        tn = st + S2N['offset'] + S2N['slope'] * (st + S2N['offset'] - S2N['t_ref'])
        dbuf.append((t, depth_world(raw, tn))); continue
    if tp == '/utlidar/cloud_base':
        m = deserialize_message(raw, PointCloud2); a = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)
        a = a[np.linalg.norm(a, axis=1) > 0.2]; g = gt_at(t); lbuf.append((t, a @ q2R(*g[5:9]).T + g[2:5])); continue
    m = deserialize_message(raw, PointCloud2)
    a = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)
    B = a @ R_LIDAR.T + T_LIDAR; rr = np.hypot(B[:, 0], B[:, 1]); B = B[(rr > 0.3) & (rr < 12)]
    g = gt_at(t); Rw = q2R(*g[5:9]); Wt = B @ Rw.T + g[2:5]
    xbuf.append((t, Wt))
    xbuf = [b for b in xbuf if t - b[0] <= 0.5]; lbuf = [b for b in lbuf if t - b[0] <= 0.5]; dbuf = [b for b in dbuf if t - b[0] <= 0.5]
    if t - last_s < DT or not dbuf or not lbuf: continue
    last_s = t
    rx, ry, rz = g[2:5]; ryaw = rpy_of(Rw)[2]; me = np.array([rx, ry]); zmax = rz + 1.10
    nrm = Rw[:, 2]; foot = g[2:5] - H_STAND * nrm; plane = (tuple(foot), tuple(nrm))
    ahead = corridor_ahead(t); org = me - HALF; nn = int(2 * HALF / CELL)
    X = np.concatenate([b[1] for b in xbuf]); L1 = np.concatenate([b[1] for b in lbuf]); DP = np.concatenate([b[1] for b in dbuf])
    Dcur = dbuf[-1][1]
    res = {}
    em = ego_rule(Wt, (rx, ry, rz), ryaw); res['ego(XT16)'] = cells(Wt[em, :2])
    WD = np.concatenate([Wt, Dcur]); emd = ego_rule(WD, (rx, ry, rz), ryaw); res['ego(XT16+깊이)'] = cells(WD[emd, :2])
    for nm, src in (('X', X), ('X+L1', np.concatenate([X, L1])), ('X+깊이', np.concatenate([X, DP])), ('X+L1+깊이', np.concatenate([X, L1, DP]))):
        hi, lo, _ = grid_stats(src, org, (nn, nn), CELL, zmax)
        Tm = traversable(hi, lo, CELL, me, org, rz, hop=HOP, seed_r=1.5, plane=plane)
        res[f'거부권[{nm}]'] = cells(Wt[em & ~veto(Wt, Tm, org, CELL), :2])
        if nm == 'X+L1+깊이':
            res['거부권[X+L1+깊이] 융합입력'] = cells(WD[emd & ~veto(WD, Tm, org, CELL), :2])
            row_T = int(Tm.sum())
    zone = 'X자' if math.hypot(rx - CXX, ry - CYY) < 2.5 else '평지'
    near = lambda cs: {c for c in cs if math.hypot((c[0] + .5) * G10 - rx, (c[1] + .5) * G10 - ry) <= 3.0}
    wall_ego = near(res['ego(XT16)']) & WALL
    row = dict(t=t - t0, zone=zone, T=row_T)
    for k, cs in res.items():
        row[k + '|헛'] = len(cs & ahead); row[k + '|벽'] = (len(dilate(near(cs)) & wall_ego) / len(wall_ego)) if wall_ego else np.nan
    S.append(row)
    if len(S) % 100 == 0: print(len(S), f'{t - t0:.0f}s 통과면 {row_T}칸', {k.split('|')[0]: row[k] for k in row if k.endswith('|헛')}, flush=True)
keys = [k[:-2] for k in S[0] if k.endswith('|헛')]
summ = {}
for zone in ('X자', '평지', '전체'):
    Z = [s for s in S if zone == '전체' or s['zone'] == zone]; summ[zone] = {'표본': len(Z), '통과면칸_평균': round(float(np.mean([s['T'] for s in Z])), 1)}
    for k in keys:
        v = np.array([s[k + '|헛'] for s in Z]); w = np.array([s[k + '|벽'] for s in Z], dtype=float)
        summ[zone][k] = dict(헛장애물_표본비율=round(float((v > 0).mean()), 3), 헛칸_평균=round(float(v.mean()), 2), 벽유지=round(float(np.nanmean(w)), 3))
tag = os.environ.get('TAG', f'hop{HOP}')
print(json.dumps(summ, ensure_ascii=False, indent=1)); json.dump(summ, open(f'{OUT}/eval_veto2_{tag}.json', 'w'), ensure_ascii=False, indent=1)
