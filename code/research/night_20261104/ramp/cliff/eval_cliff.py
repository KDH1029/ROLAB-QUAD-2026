#!/usr/bin/env python3
"""낭떠러지 검출 실데이터 오프라인 평가 (10-30 녹화본). eval_veto3.py 의 bag 읽기·시각 대응·격자 코드를 복사해 씀(원본 안 건드림).
격자 갈래: a_X(XT16 만) b_XL1(+L1) c_XD(+깊이) d_XL1D(+L1+깊이). 각 갈래에서 [거부권만] vs [거부권+낭떠러지 reach 0.30 / 0.50].
지표: (1) 걸어간 길 헛장애물(궤적 ±0.12 m, 앞 2 m)  (2) 벽 뚫림(analyze_breach '걸어간 면 뺀 진짜 벽', 원래 ego 팽창 0.40 기준)
      (3) 단 위(|xy−(4.3,−1.6)|<1.0, z>0.2)에서 0.5 m 안 '진짜 낭떠러지 가장자리'(지도 기준)가 장애물(0.40 / 0.20 m)로 덮였나
사용: DT=1.0 TMIN=0 TMAX=1e9 TAG=p0 python3 eval_cliff.py → rows_<TAG>.json (summarize_cliff.py 로 합침)"""
import sys, math, bisect, json, numpy as np, rosbag2_py, warnings, os, time
warnings.simplefilter('ignore')
from scipy.spatial import cKDTree
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs_py.point_cloud2 as pc2
sys.path.insert(0, '/home/ilovehan9283/ws/v_gosu/backup'); sys.path.insert(0, '/home/ilovehan9283/ws/night_20261104/ramp')
sys.path.insert(0, '/home/ilovehan9283/ws/night_20261104/ramp/cliff')
from backup_core import q2R, rpy_of, R_LIDAR, T_LIDAR
from terrain_core import ego_rule, grid_stats, traversable, veto, classify
from terrain_cliff import cliff_edges, cliff_points
BAG = '/home/ilovehan9283/bags/gosu_20261030_001126'
NPZ = '/home/ilovehan9283/ws/v_gosu/eval/out/gosu1030v2_map.npz'
OUT = '/home/ilovehan9283/ws/night_20261104/ramp/cliff'
CAL = json.load(open('/home/ilovehan9283/ws/night_20261104/vision/calib_depth_all.json'))
TBC = np.array(CAL['T_body_cam_optical']); S2N = CAL['stamp_to_nuc']
FX = FY = 431.08; CX, CY = 422.28, 240.89
DT = float(os.environ.get('DT', 1.0)); TMIN = float(os.environ.get('TMIN', 0)); TMAX = float(os.environ.get('TMAX', 1e9))
TAG = os.environ.get('TAG', 'p0'); H_STAND = 0.31
CELL, HALF, G10, CXX, CYY = 0.05, 4.0, 0.10, 4.3, -1.6
DROP, MINLOW = float(os.environ.get('DROP', 0.22)), int(os.environ.get('MINLOW', 1))
REACHES = ((0.30, None), (0.50, None), (0.30, 0.20), (0.50, 0.20))   # (reach, band)
DUMP = [float(v) for v in os.environ.get('DUMP', '').split(',') if v]


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

# ── 벽 정답: analyze_breach.py 와 같은 정의(궤적만 GT 로) ──
kk = np.floor(FULL[:, :2] / G10).astype(np.int64); u, inv = np.unique(kk, axis=0, return_inverse=True); inv = inv.ravel()
cmin = np.full(len(u), np.inf); np.minimum.at(cmin, inv, FULL[:, 2])
nb = cKDTree(u * G10 + G10 / 2).query_ball_point(u * G10 + G10 / 2, 0.5)
floor = np.array([np.percentile(cmin[n], 5) for n in nb]); hgt = FULL[:, 2] - floor[inv]
WALL0 = set(map(tuple, np.unique(kk[(hgt > 0.30) & (hgt < 1.0)], axis=0)))
kw = np.array(sorted(WALL0))
_ord = np.lexsort((FULL[:, 2], kk[:, 1], kk[:, 0])); _ks = kk[_ord]; _zs = FULL[_ord, 2]
_brk = np.r_[True, np.any(_ks[1:] != _ks[:-1], axis=1)]; _st = np.where(_brk)[0]; _en = np.r_[_st[1:], len(_ks)]
CMAX = {tuple(_ks[a_]): float(_zs[b_ - 1]) for a_, b_ in zip(_st, _en)}
SMIN = {tuple(_ks[a_]): float(_zs[a_]) for a_ in _st}
_dd, _ii = cKDTree(path).query((kw + 0.5) * G10); _zg = GT[_ii, 4] - 0.31
_keep = np.array([CMAX.get(tuple(c), -9) - zg >= 0.25 for c, zg in zip(kw, _zg)]) & (_dd > 0.20)
WALL = np.array(sorted(map(tuple, kw[_keep]))); WALLC = (WALL + 0.5) * G10
print('벽 정답', len(WALL0), '→ 걸어간 면 뺀 진짜 벽', len(WALL), flush=True)
# ── 낭떠러지 정답(지도): 칸(10 cm) 중앙 높이로(최저는 판 틈·잡음 점에 끌려 단 가운데까지 '낭떠러지'가 됨)
#    단 중심 1.6 m 안, 중앙 높이 ≥ −0.17(단·경사로 윗부분), 0.21 m 안 이웃 중앙 높이가 0.25 넘게 낮음, 궤적에서 0.15 m 넘게 떨어짐(걸어간 곳은 낭떠러지 아님)
_mid = {tuple(_ks[a_]): float(np.median(_zs[a_:b_])) for a_, b_ in zip(_st, _en)}
NB2 = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1), (2, 0), (-2, 0), (0, 2), (0, -2)]
CLF = []
for c, s_ in _mid.items():
    x, y = (c[0] + .5) * G10, (c[1] + .5) * G10
    if math.hypot(x - CXX, y - CYY) > 1.6 or s_ < -0.17: continue
    if any(_mid.get((c[0] + a, c[1] + b), 9) < s_ - 0.25 for a, b in NB2): CLF.append(c)
CLF = np.array(CLF); CLF = CLF[cKDTree(path).query((CLF + 0.5) * G10)[0] > 0.15]; CLFC = (CLF + 0.5) * G10
print('낭떠러지 정답 칸(지도)', len(CLF), flush=True)
np.save(f'{OUT}/cliff_truth_cells.npy', CLF)


def cells(xy): return set(map(tuple, np.floor(xy / G10).astype(np.int64)))
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
def max_run(cs):
    L = set(cs); best = 0; seen = set()
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
    return best
uu, vv = np.meshgrid(np.arange(0, 848, 4), np.arange(0, 480, 4))
def depth_world(raw, tn):
    m = deserialize_message(raw, Image); D = np.frombuffer(m.data, np.uint16).reshape(480, 848)[::4, ::4] * 1e-3
    ok = (D > 0.3) & (D < 2.5); z = D[ok]; P = np.c_[(uu[ok] - CX) * z / FX, (vv[ok] - CY) * z / FY, z]
    g = gt_at(tn); return (P @ TBC[:3, :3].T + TBC[:3, 3]) @ q2R(*g[5:9]).T + g[2:5]

xbuf, lbuf, dbuf = [], [], []; last_s = -1e9; ROWS = []; dn = 0; T_START = time.time()
r = reader(['/lidar_points', '/utlidar/cloud_base', '/camera/depth/image_rect_raw'])
while r.has_next():
    tp, raw, t = r.read_next(); t *= 1e-9
    if t - t0 > TMAX: break
    if t < tL[0] + 2 or t > tL[-1] or t - t0 < TMIN - 1.0: continue
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
    if t - t0 < TMIN or t - last_s < DT or not dbuf or not lbuf: continue
    last_s = t
    rx, ry, rz = g[2:5]; ryaw = rpy_of(Rw)[2]; me = np.array([rx, ry]); zmax = rz + 1.10
    nrm = Rw[:, 2]; foot = g[2:5] - H_STAND * nrm; plane = (tuple(foot), tuple(nrm))
    ahead = corridor_ahead(t); org = me - HALF; nn = int(2 * HALF / CELL)
    X = np.concatenate([b[1] for b in xbuf]); L1 = np.concatenate([b[1] for b in lbuf]); DP = np.concatenate([b[1] for b in dbuf])
    em = ego_rule(Wt, (rx, ry, rz), ryaw); PA = Wt[em]
    onplat = bool(math.hypot(rx - CXX, ry - CYY) < 1.0 and rz > 0.2)
    zone = 'X자' if math.hypot(rx - CXX, ry - CYY) < 2.5 else '평지'
    wsel = np.hypot(WALLC[:, 0] - rx, WALLC[:, 1] - ry) <= 3.0; WC = WALLC[wsel]; Wcells = [tuple(c) for c in WALL[wsel]]
    dcl = np.hypot(CLFC[:, 0] - rx, CLFC[:, 1] - ry); CC = CLFC[dcl <= 0.5]; CC8 = CLFC[dcl <= 0.8]
    ahc = (np.array(sorted(ahead)) + 0.5) * G10 if ahead else np.zeros((0, 2))
    covA = (cKDTree(PA[:, :2]).query(WC)[0] <= 0.40) if (len(WC) and len(PA)) else np.zeros(len(WC), bool)
    row = dict(t=round(t - t0, 2), x=round(float(rx), 3), y=round(float(ry), 3), z=round(float(rz), 3), zone=zone, plat=onplat,
               nwall=int(len(WC)), covA=int(covA.sum()), ncl=int(len(CC)), ncl8=int(len(CC8)))

    def score(name, OB):
        kd = cKDTree(OB[:, :2]) if len(OB) else None
        row[name + '|헛'] = len(cells(OB[:, :2]) & ahead) if len(OB) else 0
        if len(WC):
            cv = (kd.query(WC)[0] <= 0.40) if kd is not None else np.zeros(len(WC), bool)
            lost = covA & ~cv
            row[name + '|뚫림'] = int(lost.sum()); row[name + '|뚫림연속'] = max_run([Wcells[i] for i in np.where(lost)[0]])
        if len(CC):
            dc = kd.query(CC)[0] if kd is not None else np.full(len(CC), 9.0)
            row[name + '|낭덮40'] = round(float((dc <= 0.40).mean()), 3); row[name + '|낭덮20'] = round(float((dc <= 0.20).mean()), 3)
        if len(CC8):
            dc = kd.query(CC8)[0] if kd is not None else np.full(len(CC8), 9.0)
            row[name + '|낭8덮40'] = round(float((dc <= 0.40).mean()), 3); row[name + '|낭8덮20'] = round(float((dc <= 0.20).mean()), 3)
    score('ego', PA)
    for vn, parts in (('a_X', [X]), ('b_XL1', [X, L1]), ('c_XD', [X, DP]), ('d_XL1D', [X, L1, DP])):
        hi, lo, _ = grid_stats(np.concatenate(parts), org, (nn, nn), CELL, zmax)
        _, _, pk = classify(hi, CELL)
        Tm = traversable(hi, lo, CELL, me, org, rz, hop=0.15, seed_r=1.5, plane=plane, p=pk)
        OV = Wt[em & ~veto(Wt, Tm, org, CELL)]
        score(vn + '|거부권', OV)
        for rc, bd in REACHES:
            cl, ed, emp = cliff_edges(hi, Tm, CELL, DROP, rc, 0.05, MINLOW, band=bd, plane=plane, origin=org)
            CP = cliff_points(cl, org, CELL, rz)
            if len(CP): CP = CP[ego_rule(CP, (rx, ry, rz), ryaw)]
            tag = f'{vn}|+낭{int(rc * 100)}' + (f'b{int(bd * 100)}' if bd else '')
            score(tag, np.vstack([OV, CP]) if len(CP) else OV)
            # 가장자리·낭떠러지·빈가장자리 칸 수 (로봇 1.5 m 안)
            ij = np.argwhere(ed); dxy = org + (ij + 0.5) * CELL - me; near = np.hypot(*dxy.T) <= 1.5
            row[tag + '|가장자리'] = int(near.sum()); row[tag + '|낭칸'] = int(cl[ij[near, 0], ij[near, 1]].sum())
            row[tag + '|빈칸'] = int(emp[ij[near, 0], ij[near, 1]].sum())
            row[tag + '|낭점_길0.25'] = int(len(CP) > 0 and len(ahc) > 0 and cKDTree(CP[:, :2]).query(ahc)[0].min() < 0.25)
            if DUMP and rc == 0.30 and bd is None and any(abs(t - t0 - td) < DT / 2 + 0.05 for td in DUMP):
                np.savez(f'{OUT}/dump_{vn}_{t - t0:.0f}.npz', hi=hi, lo=lo, T=Tm, cl=cl, ed=ed, emp=emp, org=org, rz=rz, me=me, CP=CP,
                         ahc=ahc, OV=OV[:, :3], Wt=Wt, L1=L1)
                clb, edb, emb = cliff_edges(hi, Tm, CELL, DROP, 0.30, 0.05, MINLOW, band=0.20, plane=plane, origin=org)
                np.savetxt(f'{OUT}/real_gridb_{vn}_{t - t0:.0f}.txt', np.c_[np.nan_to_num(hi.ravel(), nan=-999.0), Tm.ravel().astype(int), clb.ravel().astype(int), edb.ravel().astype(int), emb.ravel().astype(int)], fmt='%.6f %d %d %d %d')
                open(f'{OUT}/real_gridb_{vn}_{t - t0:.0f}.args', 'w').write(' '.join(f'{v:.9f}' for v in (0.20, *plane[0], *plane[1], org[0], org[1])))
                np.savetxt(f'{OUT}/real_grid_{vn}_{t - t0:.0f}.txt', np.c_[np.nan_to_num(hi.ravel(), nan=-999.0), Tm.ravel().astype(int), cl.ravel().astype(int), ed.ravel().astype(int), emp.ravel().astype(int)], fmt='%.6f %d %d %d %d')
    ROWS.append(row)
    if len(ROWS) % 20 == 0: print(TAG, len(ROWS), f'{t - t0:.0f}s', f'{time.time() - T_START:.0f}s 경과', flush=True)
json.dump(ROWS, open(f'{OUT}/rows_{TAG}.json', 'w'), ensure_ascii=False)
print(TAG, '끝', len(ROWS), f'{time.time() - T_START:.0f}s')
