#!/usr/bin/env python3
"""[H] 낭떠러지 갈래 스윕 — eval_cliff.py 복사·확장(원본 안 건드림). 격자 = XT16+L1 (재생 C++ 깊이 끔과 같음), 0.5 초 표본.
갈래: drop {0.22,0.26,0.30} × 높은곳만 {끔,켬} × 연속 {1,2} × 방향 {전방위, 앞 ±60° 제외}  (reach 0.30, rmin 0.05, min_low 1)
  높은곳만: 발밑(평면 발 z) − '로봇 3 m 안 유효 칸 hi 의 2% 분위'(최저 지면) ≥ HG(0.15) 일 때만 낭떠러지
  연속 2  : 같은 갈래의 직전 표본(0.5 s 전)에 0.075 m 안 낭떠러지 칸이 있었던 칸만
  방향    : 로봇 몸체 yaw 기준 ±60° 안 낭떠러지 칸 버림
지표: 헛장애물(궤적 ±0.12 m, 앞 2 m), 벽 뚫림(걸어간 면 뺀 진짜 벽 3 m 안, 원래 ego 팽창 0.40 기준), 단 가장자리(지도 정답 0.5 m 안) 덮임
진단: drop 0.22 기본 갈래에서 '낭떠러지 점 때문에 새로 생긴 헛장애물 칸' 하나하나의 위치·낙차·L1 기여·지도 낙차 → diag_<TAG>.json
사용: TMIN=0 TMAX=100 TAG=s0 python3 eval_sweep.py"""
import sys, math, bisect, json, numpy as np, rosbag2_py, warnings, os, time
warnings.simplefilter('ignore')
from scipy.spatial import cKDTree
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
sys.path.insert(0, '/home/ilovehan9283/ws/v_gosu/backup'); sys.path.insert(0, '/home/ilovehan9283/ws/night_20261104/ramp')
sys.path.insert(0, '/home/ilovehan9283/ws/night_20261104/ramp/cliff')
from backup_core import q2R, rpy_of, R_LIDAR, T_LIDAR
from terrain_core import ego_rule, grid_stats, traversable, veto, classify
from terrain_cliff import cliff_edges, cliff_points
BAG = '/home/ilovehan9283/bags/gosu_20261030_001126'
NPZ = '/home/ilovehan9283/ws/v_gosu/eval/out/gosu1030v2_map.npz'
OUT = '/home/ilovehan9283/ws/night_20261104/ramp/cliff/sweep_h'
DT = float(os.environ.get('DT', 0.5)); TMIN = float(os.environ.get('TMIN', 0)); TMAX = float(os.environ.get('TMAX', 300))
TAG = os.environ.get('TAG', 's0'); H_STAND = 0.31; HG = float(os.environ.get('HG', 0.15)); FRONT = math.radians(60)
CELL, HALF, G10, CXX, CYY = 0.05, 4.0, 0.10, 4.3, -1.6
DROPS = (0.22, 0.26, 0.30); WARM = TMIN - 0.6 if TMIN > 0 else 0.0
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

# ── 벽 정답 (eval_cliff 와 같음) ──
kk = np.floor(FULL[:, :2] / G10).astype(np.int64); u, inv = np.unique(kk, axis=0, return_inverse=True); inv = inv.ravel()
cmin = np.full(len(u), np.inf); np.minimum.at(cmin, inv, FULL[:, 2])
nb = cKDTree(u * G10 + G10 / 2).query_ball_point(u * G10 + G10 / 2, 0.5)
floor = np.array([np.percentile(cmin[n], 5) for n in nb]); hgt = FULL[:, 2] - floor[inv]
WALL0 = set(map(tuple, np.unique(kk[(hgt > 0.30) & (hgt < 1.0)], axis=0)))
kw = np.array(sorted(WALL0))
_ord = np.lexsort((FULL[:, 2], kk[:, 1], kk[:, 0])); _ks = kk[_ord]; _zs = FULL[_ord, 2]
_brk = np.r_[True, np.any(_ks[1:] != _ks[:-1], axis=1)]; _st = np.where(_brk)[0]; _en = np.r_[_st[1:], len(_ks)]
CMAX = {tuple(_ks[a_]): float(_zs[b_ - 1]) for a_, b_ in zip(_st, _en)}
_dd, _ii = cKDTree(path).query((kw + 0.5) * G10); _zg = GT[_ii, 4] - 0.31
_keep = np.array([CMAX.get(tuple(c), -9) - zg >= 0.25 for c, zg in zip(kw, _zg)]) & (_dd > 0.20)
WALL = np.array(sorted(map(tuple, kw[_keep]))); WALLC = (WALL + 0.5) * G10
_mid = {tuple(_ks[a_]): float(np.median(_zs[a_:b_])) for a_, b_ in zip(_st, _en)}
_p10 = {tuple(_ks[a_]): float(np.percentile(_zs[a_:b_], 10)) for a_, b_ in zip(_st, _en)}
CLF = np.load('/home/ilovehan9283/ws/night_20261104/ramp/cliff/cliff_truth_cells.npy'); CLFC = (CLF + 0.5) * G10
print('진짜 벽', len(WALL), '낭떠러지 정답', len(CLF), flush=True)


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
def map_drop(x, y, R=0.30):
    """지도(10 cm 칸 중앙 높이) 기준: (x,y) 칸 높이 − R 안 이웃 칸 최저 중앙 높이."""
    c0 = (int(np.floor(x / G10)), int(np.floor(y / G10))); z0 = _mid.get(c0, np.nan); m = np.inf; n = int(np.ceil(R / G10))
    for a in range(-n, n + 1):
        for b in range(-n, n + 1):
            if (a or b) and G10 * math.hypot(a, b) <= R + 1e-9:
                v = _mid.get((c0[0] + a, c0[1] + b))
                if v is not None: m = min(m, v)
    return z0, (z0 - m) if np.isfinite(m) else np.nan


VARS = [(dr, hg, dr_, cn) for dr in DROPS for hg in (0, 1) for dr_ in ('all', 'nofront') for cn in (1, 2)]
def vname(dr, hg, di, cn): return f'd{int(round(dr * 100))}|h{hg}|{di}|c{cn}'
prev = {}   # (dr,hg,di) → (t, xy)
xbuf, lbuf = [], []; last_s = -1e9; ROWS = []; DIAG = []; T_START = time.time()
r = reader(['/lidar_points', '/utlidar/cloud_base'])
while r.has_next():
    tp, raw, t = r.read_next(); t *= 1e-9
    if t - t0 > TMAX: break
    if t < tL[0] + 2 or t > tL[-1] or t - t0 < WARM - 1.0: continue
    if tp == '/utlidar/cloud_base':
        m = deserialize_message(raw, PointCloud2); a = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)
        a = a[np.linalg.norm(a, axis=1) > 0.2]; g = gt_at(t); lbuf.append((t, a @ q2R(*g[5:9]).T + g[2:5])); continue
    m = deserialize_message(raw, PointCloud2)
    a = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)
    B = a @ R_LIDAR.T + T_LIDAR; rr = np.hypot(B[:, 0], B[:, 1]); B = B[(rr > 0.3) & (rr < 12)]
    g = gt_at(t); Rw = q2R(*g[5:9]); Wt = B @ Rw.T + g[2:5]
    xbuf.append((t, Wt))
    xbuf = [b for b in xbuf if t - b[0] <= 0.5]; lbuf = [b for b in lbuf if t - b[0] <= 0.5]
    if t - t0 < WARM or t - last_s < DT - 0.02 or not lbuf: continue
    last_s = t; store = t - t0 >= TMIN
    rx, ry, rz = g[2:5]; ryaw = rpy_of(Rw)[2]; me = np.array([rx, ry]); zmax = rz + 1.10
    nrm = Rw[:, 2]; foot = g[2:5] - H_STAND * nrm; plane = (tuple(foot), tuple(nrm))
    ahead = corridor_ahead(t); org = me - HALF; nn = int(2 * HALF / CELL)
    X = np.concatenate([b[1] for b in xbuf]); L1 = np.concatenate([b[1] for b in lbuf])
    em = ego_rule(Wt, (rx, ry, rz), ryaw); PA = Wt[em]
    onplat = bool(math.hypot(rx - CXX, ry - CYY) < 1.0 and rz > 0.2)
    zone = 'X자' if math.hypot(rx - CXX, ry - CYY) < 2.5 else '평지'
    wsel = np.hypot(WALLC[:, 0] - rx, WALLC[:, 1] - ry) <= 3.0; WC = WALLC[wsel]; Wcells = [tuple(c) for c in WALL[wsel]]
    dcl = np.hypot(CLFC[:, 0] - rx, CLFC[:, 1] - ry); CC = CLFC[dcl <= 0.5]
    bea = np.arctan2(CC[:, 1] - ry, CC[:, 0] - rx) - ryaw; CCf = np.abs((bea + np.pi) % (2 * np.pi) - np.pi) <= FRONT
    covA = (cKDTree(PA[:, :2]).query(WC)[0] <= 0.40) if (len(WC) and len(PA)) else np.zeros(len(WC), bool)
    hi, lo, _ = grid_stats(np.concatenate([X, L1]), org, (nn, nn), CELL, zmax)
    _, _, pk = classify(hi, CELL)
    Tm = traversable(hi, lo, CELL, me, org, rz, hop=0.15, seed_r=1.5, plane=plane, p=pk)
    OV = Wt[em & ~veto(Wt, Tm, org, CELL)]
    # 최저 지면: 로봇 3 m 안 유효 칸 hi 의 2% 분위 / 통과면 칸 최저
    ii, jj = np.meshgrid(np.arange(nn), np.arange(nn), indexing='ij')
    CX_ = org[0] + (ii + 0.5) * CELL; CY_ = org[1] + (jj + 0.5) * CELL; R3 = np.hypot(CX_ - rx, CY_ - ry) <= 3.0
    hv = hi[R3 & np.isfinite(hi)]; gnd = float(np.percentile(hv, 2)) if len(hv) else foot[2]
    hT = hi[R3 & Tm]; gndT = float(hT.min()) if len(hT) else foot[2]
    hgh = float(foot[2] - gnd); high = hgh >= HG
    row = dict(t=round(t - t0, 2), x=round(float(rx), 3), y=round(float(ry), 3), z=round(float(rz), 3), yaw=round(float(ryaw), 3), zone=zone, plat=onplat,
               nwall=int(len(WC)), covA=int(covA.sum()), ncl=int(len(CC)), nclf=int(CCf.sum()), foot_gnd=round(hgh, 3), foot_gndT=round(float(foot[2] - gndT), 3))

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
            if CCf.any(): row[name + '|낭앞덮20'] = round(float((dc[CCf] <= 0.20).mean()), 3)
            if (~CCf).any(): row[name + '|낭옆덮20'] = round(float((dc[~CCf] <= 0.20).mean()), 3)
    score('ego', PA)
    score('거부권', OV)
    for dr in DROPS:
        cl, ed, emp = cliff_edges(hi, Tm, CELL, dr, 0.30, 0.05, 1)
        ij0 = np.argwhere(cl); xy0 = org + (ij0 + 0.5) * CELL
        b0 = np.arctan2(xy0[:, 1] - ry, xy0[:, 0] - rx) - ryaw; front0 = np.abs((b0 + np.pi) % (2 * np.pi) - np.pi) <= FRONT
        for hg in (0, 1):
            for di in ('all', 'nofront'):
                sel = np.ones(len(ij0), bool)
                if hg and not high: sel[:] = False
                if di == 'nofront': sel &= ~front0
                xy = xy0[sel]; key = (dr, hg, di)
                pv = prev.get(key)
                for cn in (1, 2):
                    if cn == 2:
                        if pv is not None and abs((t - pv[0]) - DT) < 0.2 and len(pv[1]) and len(xy):
                            xy_c = xy[cKDTree(pv[1]).query(xy)[0] <= 0.075]
                        else:
                            xy_c = xy[:0]
                    else:
                        xy_c = xy
                    CP = np.c_[xy_c, np.full(len(xy_c), rz + 0.30)]
                    if len(CP): CP = CP[ego_rule(CP, (rx, ry, rz), ryaw)]
                    nm = vname(dr, hg, di, cn)
                    score(nm, np.vstack([OV, CP]) if len(CP) else OV)
                    row[nm + '|낭칸'] = int(len(CP))
                    if dr == 0.22 and hg == 0 and di == 'all' and cn == 1 and store and len(CP):
                        newc = (cells(CP[:, :2]) & ahead) - (cells(OV[:, :2]) & ahead)
                        if newc:
                            # 새 헛장애물 칸 하나하나 진단 (그 칸 안 5 cm 낭떠러지 칸)
                            k10 = np.floor(xy0 / G10).astype(np.int64)
                            hiX, _, _ = grid_stats(X, org, (nn, nn), CELL, zmax)
                            for c in newc:
                                for q in np.where((k10[:, 0] == c[0]) & (k10[:, 1] == c[1]))[0]:
                                    i_, j_ = ij0[q]; ehi = float(hi[i_, j_])
                                    # 바깥 낮은 칸들
                                    lows = []
                                    for a_ in range(-6, 7):
                                        for b_ in range(-6, 7):
                                            dd_ = CELL * math.hypot(a_, b_)
                                            if 0.05 - 1e-9 <= dd_ <= 0.30 + 1e-9:
                                                p_, q_ = i_ + a_, j_ + b_
                                                if 0 <= p_ < nn and 0 <= q_ < nn and not Tm[p_, q_] and np.isfinite(hi[p_, q_]) and hi[p_, q_] < ehi - 0.22:
                                                    lows.append((float(hi[p_, q_]), dd_, p_, q_))
                                    lmin = min(lows) if lows else (np.nan, np.nan, -1, -1)
                                    zp = foot[2] - (nrm[0] * (xy0[q, 0] - foot[0]) + nrm[1] * (xy0[q, 1] - foot[1])) / max(nrm[2], 0.5)
                                    mz, mdrop = map_drop(*xy0[q])
                                    lx, ly = org + (np.array(lmin[2:]) + 0.5) * CELL if lows else (np.nan, np.nan)
                                    DIAG.append(dict(t=row['t'], rx=row['x'], ry=row['y'], rz=row['z'], yaw=row['yaw'], cx=round(float(xy0[q, 0]), 3), cy=round(float(xy0[q, 1]), 3),
                                                     edge_hi=round(ehi, 3), edge_minus_foot=round(ehi - zp, 3), drop=round(ehi - lmin[0], 3), low_dist=lmin[1],
                                                     low_x=round(float(lx), 3), low_y=round(float(ly), 3), nlow=len(lows),
                                                     hiX=None if not np.isfinite(hiX[i_, j_]) else round(float(hiX[i_, j_]), 3),
                                                     map_mid=None if not np.isfinite(mz) else round(mz, 3), map_drop30=None if not np.isfinite(mdrop) else round(mdrop, 3),
                                                     map_p10=_p10.get((int(np.floor(xy0[q, 0] / G10)), int(np.floor(xy0[q, 1] / G10)))),
                                                     dplat=round(math.hypot(xy0[q, 0] - CXX, xy0[q, 1] - CYY), 3), front=bool(front0[q]),
                                                     foot_gnd=row['foot_gnd'], zone=zone, plat=onplat))
                prev[key] = (t, xy)
    if DUMP and any(abs(t - t0 - td) < DT / 2 + 0.05 for td in DUMP):
        hiX, _, _ = grid_stats(X, org, (nn, nn), CELL, zmax)
        cl, ed, emp = cliff_edges(hi, Tm, CELL, 0.22, 0.30, 0.05, 1)
        np.savez(f'{OUT}/dump_{t - t0:.1f}.npz', hi=hi, hiX=hiX, T=Tm, cl=cl, ed=ed, org=org, rz=rz, me=me, yaw=ryaw, foot=foot, nrm=nrm,
                 L1=L1, X=X, ahc=(np.array(sorted(ahead)) + 0.5) * G10 if ahead else np.zeros((0, 2)), OV=OV[:, :3])
    if store: ROWS.append(row)
    if len(ROWS) % 40 == 0 and store: print(TAG, len(ROWS), f'{t - t0:.0f}s', f'{time.time() - T_START:.0f}s 경과', flush=True)
json.dump(ROWS, open(f'{OUT}/rows_{TAG}.json', 'w'), ensure_ascii=False)
json.dump(DIAG, open(f'{OUT}/diag_{TAG}.json', 'w'), ensure_ascii=False)
print(TAG, '끝', len(ROWS), '진단', len(DIAG), f'{time.time() - T_START:.0f}s')
