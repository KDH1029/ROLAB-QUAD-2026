#!/usr/bin/env python3
"""깊이 갈래 평가 — D435i 깊이를 지형 격자에 더하면 왜 경사로 헛장애물이 늘었나 (11-04, depth_forks).

기준: ramp/eval_veto3.py 와 같은 tL/GT·평가 잣대(걸어간 길 앞 2 m 위 ego 장애물 칸 = 헛장애물, 벽 유지 3 m).
플래너 입력은 모든 갈래에서 XT16 단일 스캔 Wt. 깊이는 격자(통과면 찾기)에만.
격자는 최고·최저만 쓰므로 XT16+L1 격자 하나 + 갈래별 깊이 격자를 fmax/fmin 으로 합친다(점을 합쳐 다시 만든 것과 같음).

갈래:
  G1  XT16+L1 (기준)
  G2  +깊이 전부 0.3~2.5 m (veto3 와 같은 자세 조회: 다음 odom)
  G3  +깊이 0.3~1.5 m          G4  +깊이 0.3~1.0 m
  G5  +깊이, 로봇에서 수평 1.75 m 안 점만
  G6m/G6p  깊이 외부파라미터 x −0.02 / +0.02 m (몸체 x)
  G2i      깊이 자세 보간(δ=0)  — ⑦의 기준
  G7m/G7p  깊이 시각 δ −0.04 / +0.04 s (보간 자세)
  G8  깊이 3x3 중앙값(원해상도에서 표본 화소 주변 9칸, 유효 ≥5)
  (2차, 원인 겨냥)
  G9   깊이는 라이다 빈칸만 채움 (라이다가 잰 칸은 손대지 않음)
  G10  깊이 z −0.04 m (걸어간 길에서 잰 깊이−라이다 칸 최고 차이만큼 내림)
  G11  칸 높이폭을 출처별로 (max(라이다 폭, 깊이 폭)) — 두 센서 사이 높이 어긋남을 '세로'로 안 봄
  G12  (진단용 반사실) G2 높이 + G1 의 세로/솟음 표시 — 손해가 세로/솟음 표시 경로인지 확인
  G13  G9 + G11 은 G9 와 같음(빈칸만 채우면 출처 섞인 칸이 없음) → 대신 G11 + 깊이 0.3~1.0 m
PHASE 환경변수: 표본 격자를 t0+PHASE+k·DT 로 민다(0.5 → 1초 간격 표본과 겹치지 않는 둘째 묶음).
사용: python3 eval_depth_forks.py prep            # tL/GT/벽 캐시
      python3 eval_depth_forks.py run A B TAG     # bag 시각(t−t0) [A,B) 구간 처리 → part_TAG.json
      python3 eval_depth_forks.py merge           # part_*.json 합쳐 RESULTS json
"""
import sys, math, bisect, json, os, glob, warnings
import numpy as np
warnings.simplefilter('ignore')
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, '/home/ilovehan9283/ws/v_gosu/backup'); sys.path.insert(0, HERE)
from terrain_core import ego_rule, grid_stats, traversable, veto, classify, _disk
from scipy import ndimage

BAG = '/home/ilovehan9283/bags/gosu_20261030_001126'
NPZ = '/home/ilovehan9283/ws/v_gosu/eval/out/gosu1030v2_map.npz'
CAL = json.load(open('/home/ilovehan9283/ws/night_20261104/vision/calib_depth_all.json'))
TBC = np.array(CAL['T_body_cam_optical']); S2N = CAL['stamp_to_nuc']
FX = FY = 431.08; CX, CY = 422.28, 240.89
DT = float(os.environ.get('DT', 1.0)); H_STAND = 0.31
CELL, HALF, G10, CXX, CYY = 0.05, 4.0, 0.10, 4.3, -1.6
CACHE = f'{HERE}/prep_cache.npz'
FORKS = ['G1', 'G2', 'G3', 'G4', 'G5', 'G6m', 'G6p', 'G2i', 'G7m', 'G7p', 'G8', 'G9', 'G10', 'G11', 'G12', 'G13']
PHASE = float(os.environ.get('PHASE', 0.0))
FAN_R0, FAN_R1, FAN_HALF = 0.3, 1.75, math.radians(30)


def traversable_bad(hi, cell, seed_xy, origin, bad0, plane, max_step=0.20, hop=0.15, seed_r=1.5, seed_tol=0.08, seed_slope=0.05, dil=1):
    """terrain_core.traversable 와 같은 연결·씨앗 규칙, 세로/솟음 표시(bad0)만 밖에서 받는다."""
    from scipy.spatial import cKDTree
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    valid = np.isfinite(hi)
    bad = ndimage.binary_dilation(bad0, structure=_disk(dil)) if dil > 0 else bad0
    cand = valid & ~bad; ij = np.argwhere(cand); T = np.zeros_like(cand)
    if len(ij) < 3: return T
    xy = origin + (ij + 0.5) * cell; z = hi[ij[:, 0], ij[:, 1]]
    pr = cKDTree(xy).query_pairs(hop, output_type='ndarray')
    if len(pr) == 0: return T
    pr = pr[np.abs(z[pr[:, 0]] - z[pr[:, 1]]) <= max_step]
    g = coo_matrix((np.ones(len(pr)), (pr[:, 0], pr[:, 1])), shape=(len(ij), len(ij)))
    _, lab = connected_components(g, directed=False)
    dd = np.hypot(*(xy - seed_xy).T); (fx, fy, fz), (nx, ny, nz) = plane
    zp = fz - (nx * (xy[:, 0] - fx) + ny * (xy[:, 1] - fy)) / nz
    seeds = np.where((dd <= seed_r) & (np.abs(z - zp) <= seed_tol + seed_slope * dd))[0]
    if len(seeds) == 0: return T
    good = np.isin(lab, np.unique(lab[seeds])); T[ij[good, 0], ij[good, 1]] = True
    return T


def reader(topics):
    import rosbag2_py
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics)); return r


def prep():
    """eval_veto3.py 의 tL/GT/WALL 계산 그대로 → 캐시."""
    from scipy.spatial import cKDTree
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
    tL = h_o - dmed - Kp; o = np.argsort(tL); tL = tL[o]; GT = odom[o]
    kk = np.floor(FULL[:, :2] / G10).astype(np.int64); u, inv = np.unique(kk, axis=0, return_inverse=True); inv = inv.ravel()
    cmin = np.full(len(u), np.inf); np.minimum.at(cmin, inv, FULL[:, 2])
    nb = cKDTree(u * G10 + G10 / 2).query_ball_point(u * G10 + G10 / 2, 0.5)
    floor = np.array([np.percentile(cmin[n], 5) for n in nb]); hgt = FULL[:, 2] - floor[inv]
    WALL = np.unique(kk[(hgt > 0.30) & (hgt < 1.0)], axis=0)
    np.savez(CACHE, tL=tL, GT=GT, WALL=WALL)
    print('prep ok', len(tL), len(WALL), f'{tL[-1] - tL[0]:.1f}s')


def run(A, B, tag):
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2, Image
    import sensor_msgs_py.point_cloud2 as pc2
    from backup_core import q2R, rpy_of, R_LIDAR, T_LIDAR
    C = np.load(CACHE); tL, GT = C['tL'], C['GT']; WALL = set(map(tuple, C['WALL'])); t0 = tL[0]
    path = GT[:, 2:4]; cum = np.r_[0, np.cumsum(np.hypot(*np.diff(path, axis=0).T))]

    def gt_at(ts): return GT[min(max(bisect.bisect_left(tL, ts), 0), len(tL) - 1)]

    def pose_interp(ts):
        i = min(max(bisect.bisect_left(tL, ts), 1), len(tL) - 1); a, b = GT[i - 1], GT[i]
        w = 0.0 if tL[i] == tL[i - 1] else min(max((ts - tL[i - 1]) / (tL[i] - tL[i - 1]), 0.0), 1.0)
        p = (1 - w) * a[2:5] + w * b[2:5]; qa, qb = a[5:9], b[5:9]
        if np.dot(qa, qb) < 0: qb = -qb
        q = (1 - w) * qa + w * qb; q /= np.linalg.norm(q); return p, q2R(*q)

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
    rays = np.c_[(uu.ravel() - CX) / FX, (vv.ravel() - CY) / FY, np.ones(uu.size)]

    def depth_frame(raw, tn):
        """한 프레임 → 갈래별 세계 점 + 점별 속성."""
        m = deserialize_message(raw, Image); Dfull = np.frombuffer(m.data, np.uint16).reshape(480, 848).astype(np.float32) * 1e-3
        D = Dfull[::4, ::4].ravel(); ok = (D > 0.3) & (D < 2.5)
        Pc = rays[ok] * D[ok, None]; Pb = Pc @ TBC[:3, :3].T + TBC[:3, 3]
        g = gt_at(tn); Rn = q2R(*g[5:9]); pn = g[2:5]
        out = {'z': D[ok], 'W': Pb @ Rn.T + pn}
        out['hd'] = np.hypot(out['W'][:, 0] - pn[0], out['W'][:, 1] - pn[1])
        out['Wxm'] = out['W'] + Rn @ np.array([-0.02, 0, 0]); out['Wxp'] = out['W'] + Rn @ np.array([0.02, 0, 0])
        for k, dl in (('Wi0', 0.0), ('Wim', -0.04), ('Wip', 0.04)):
            p, R = pose_interp(tn + dl); out[k] = Pb @ R.T + p
        # 3x3 중앙값 (원해상도, 표본 화소 주변)
        P = np.pad(Dfull, 1); st = np.stack([P[vv + 1 + di, uu + 1 + dj] for di in (-1, 0, 1) for dj in (-1, 0, 1)]).reshape(9, -1)
        st[st <= 0] = np.nan; nv = np.isfinite(st).sum(0); md = np.nanmedian(st, axis=0)
        okm = (nv >= 5) & (md > 0.3) & (md < 2.5)
        out['Wmed'] = (rays[okm] * md[okm, None] @ TBC[:3, :3].T + TBC[:3, 3]) @ Rn.T + pn
        return out

    xbuf, lbuf, dbuf = [], [], []; S = []; NEW = []; GROUND = []
    next_s = t0 + PHASE + math.ceil((max(A, 2.0) - PHASE) / DT) * DT; last_bin = None
    r = reader(['/lidar_points', '/utlidar/cloud_base', '/camera/depth/image_rect_raw'])
    r.seek(int((t0 + A - 0.8) * 1e9))
    while r.has_next():
        tp, raw, t = r.read_next(); t *= 1e-9
        if t - t0 >= B: break
        if t < tL[0] + 2 or t > tL[-1]: continue
        if tp == '/camera/depth/image_rect_raw':
            b_ = int(t * 10)            # 0.1 초 칸마다 첫 프레임 (30 Hz → 10 Hz, veto3 의 '3장에 1장'과 같은 비율, 구간 나눠도 결정적)
            if b_ == last_bin: continue
            last_bin = b_
            s_, ns_ = np.frombuffer(raw[4:8], '<i4')[0], np.frombuffer(raw[8:12], '<u4')[0]; st = float(s_) + ns_ * 1e-9
            tn = st + S2N['offset'] + S2N['slope'] * (st + S2N['offset'] - S2N['t_ref'])
            dbuf.append((t, depth_frame(raw, tn))); continue
        if tp == '/utlidar/cloud_base':
            m = deserialize_message(raw, PointCloud2); a = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)
            a = a[np.linalg.norm(a, axis=1) > 0.2]; g = gt_at(t); lbuf.append((t, a @ q2R(*g[5:9]).T + g[2:5])); continue
        m = deserialize_message(raw, PointCloud2)
        a = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)
        Bp = a @ R_LIDAR.T + T_LIDAR; rr = np.hypot(Bp[:, 0], Bp[:, 1]); Bp = Bp[(rr > 0.3) & (rr < 12)]
        g = gt_at(t); Rw = q2R(*g[5:9]); Wt = Bp @ Rw.T + g[2:5]
        xbuf.append((t, Wt))
        xbuf = [b for b in xbuf if t - b[0] <= 0.5]; lbuf = [b for b in lbuf if t - b[0] <= 0.5]; dbuf = [b for b in dbuf if t - b[0] <= 0.5]
        if t - t0 < A or t < next_s or not dbuf or not lbuf: continue
        next_s = t0 + PHASE + (math.floor((t - t0 - PHASE) / DT) + 1) * DT
        rx, ry, rz = g[2:5]; ryaw = rpy_of(Rw)[2]; me = np.array([rx, ry]); zmax = rz + 1.10
        nrm = Rw[:, 2]; foot = g[2:5] - H_STAND * nrm; plane = (tuple(foot), tuple(nrm))
        ahead = corridor_ahead(t); org = me - HALF; nn = int(2 * HALF / CELL); shp = (nn, nn)
        X = np.concatenate([b[1] for b in xbuf]); L1 = np.concatenate([b[1] for b in lbuf])
        cat = lambda k: np.concatenate([b[1][k] for b in dbuf])
        DW = cat('W'); Dz = cat('z'); Dhd = cat('hd')
        dsrc = {'G2': DW, 'G3': DW[Dz < 1.5], 'G4': DW[Dz < 1.0], 'G5': DW[Dhd <= 1.75], 'G6m': cat('Wxm'), 'G6p': cat('Wxp'),
                'G2i': cat('Wi0'), 'G7m': cat('Wim'), 'G7p': cat('Wip'), 'G8': cat('Wmed'), 'G9': DW, 'G10': DW - np.array([0, 0, 0.04]),
                'G11': DW, 'G12': DW, 'G13': DW[Dz < 1.0]}
        hiX, loX, nX = grid_stats(X, org, shp, CELL, zmax); hiL, loL, nL = grid_stats(L1, org, shp, CELL, zmax)
        hiXL, loXL = np.fmax(hiX, hiL), np.fmin(loX, loL)
        em = ego_rule(Wt, (rx, ry, rz), ryaw)
        res = {'ego(XT16)': cells(Wt[em, :2])}; info = {}
        for f in FORKS:
            if f == 'G1':
                hi, lo, nD = hiXL, loXL, None
            else:
                hiD, loD, nD = grid_stats(dsrc[f], org, shp, CELL, zmax); hi, lo = np.fmax(hiXL, hiD), np.fmin(loXL, loD)
                if f == 'G9':
                    hX_ok = np.isfinite(hiXL); hi, lo = np.where(hX_ok, hiXL, hiD), np.where(hX_ok, loXL, loD)
            _, _, pk = classify(hi, CELL)
            if f in ('G11', 'G13'):
                spr = np.fmax(hiXL - loXL, hiD - loD)
                b0 = np.isfinite(hi) & ((spr > 0.20) | (pk > 0.10)); Tm = traversable_bad(hi, CELL, me, org, b0, plane); bad0 = b0
            elif f == 'G12':
                b0 = np.nan_to_num(info['G1']['bad0'].astype(float)).astype(bool) & np.isfinite(hi); Tm = traversable_bad(hi, CELL, me, org, b0, plane); bad0 = b0
            else:
                Tm, bad0 = traversable(hi, lo, CELL, me, org, rz, hop=0.15, seed_r=1.5, plane=plane, p=pk, return_bad=True)
            res[f] = cells(Wt[em & ~veto(Wt, Tm, org, CELL), :2])
            if f in ('G1', 'G2'):
                info[f] = dict(hi=hi, lo=lo, pk=pk, T=Tm, bad0=bad0, hiD=None if f == 'G1' else hiD, loD=None if f == 'G1' else loD, nD=nD)
        if os.environ.get('DUMP_T') and any(abs(t - t0 - float(x)) < 0.2 for x in os.environ['DUMP_T'].split(',')):
            np.savez(f'{HERE}/dump_{t - t0:.2f}.npz', hiXL=hiXL, loXL=loXL, hiD=info['G2']['hiD'], loD=info['G2']['loD'], foot=foot, nrm=nrm, me=me, org=org, rz=rz,
                     Wt=Wt, em=em, ahead=np.array(sorted(ahead)), T1=info['G1']['T'], T2=info['G2']['T'])
        # ── 진단: G2 에서 새로 생긴 헛장애물 칸 ──
        I1, I2 = info['G1'], info['G2']; bad_dil = ndimage.binary_dilation(I2['bad0'], structure=_disk(1))
        # 깊이 점 칸별 거리(카메라 깊이·수평거리) 평균
        kD = np.floor((DW[:, :2] - org) / CELL).astype(np.int64); okD = (kD >= 0).all(1) & (kD < nn).all(1) & (DW[:, 2] <= zmax)
        sz = np.zeros(shp); sh = np.zeros(shp); cn = np.zeros(shp)
        np.add.at(sz, (kD[okD, 0], kD[okD, 1]), Dz[okD]); np.add.at(sh, (kD[okD, 0], kD[okD, 1]), Dhd[okD]); np.add.at(cn, (kD[okD, 0], kD[okD, 1]), 1)
        new = (res['G2'] & ahead) - (res['G1'] & ahead)
        kW = np.floor((Wt[:, :2] - org) / CELL).astype(np.int64); okW = (kW >= 0).all(1) & (kW < nn).all(1)
        keepW = em & ~veto(Wt, I2['T'], org, CELL) & okW
        c10 = np.floor(Wt[:, :2] / G10).astype(np.int64)
        for c in new:
            sel = keepW & (c10[:, 0] == c[0]) & (c10[:, 1] == c[1])
            ks = np.unique(kW[sel], axis=0); why = []
            for a_, b_ in ks:
                if not np.isfinite(I2['hi'][a_, b_]): why.append('빈칸')
                elif I2['hi'][a_, b_] - I2['lo'][a_, b_] > 0.20: why.append('세로(칸 안 높이폭>0.20)')
                elif I2['pk'][a_, b_] > 0.10: why.append('솟음(고리 q80 대비>0.10)')
                elif bad_dil[a_, b_]: why.append('세로/솟음 옆칸')
                else: why.append('안이어짐')
            order = ['세로(칸 안 높이폭>0.20)', '솟음(고리 q80 대비>0.10)', '세로/솟음 옆칸', '안이어짐', '빈칸']
            w = next((o for o in order if o in why), '?')
            # 근처(3x3, 5 cm) 깊이 점 통계 — 이 칸들의 깊이 높이폭, 깊이 최고 − 라이다 최고, 거리
            a0, a1 = max(ks[:, 0].min() - 1, 0), min(ks[:, 0].max() + 2, nn); b0, b1 = max(ks[:, 1].min() - 1, 0), min(ks[:, 1].max() + 2, nn)
            hD, lD = I2['hiD'][a0:a1, b0:b1], I2['loD'][a0:a1, b0:b1]; hX_ = hiXL[a0:a1, b0:b1]; lX_ = loXL[a0:a1, b0:b1]
            nd = cn[a0:a1, b0:b1].sum()
            rec = dict(t=round(t - t0, 2), zone='X자' if math.hypot(rx - CXX, ry - CYY) < 2.5 else '평지', why=w, n_depth=int(nd),
                       d_spread=float(np.nanmax(hD) - np.nanmin(lD)) if np.isfinite(hD).any() else None,
                       d_hi_minus_lidar_hi=float(np.nanmax(hD) - np.nanmax(hX_)) if (np.isfinite(hD).any() and np.isfinite(hX_).any()) else None,
                       d_lo_minus_lidar_lo=float(np.nanmin(lD) - np.nanmin(lX_)) if (np.isfinite(lD).any() and np.isfinite(lX_).any()) else None,
                       lidar_spread=float(np.nanmax(hX_) - np.nanmin(lX_)) if np.isfinite(hX_).any() else None,
                       g2_spread=float(np.nanmax(I2['hi'][a0:a1, b0:b1] - I2['lo'][a0:a1, b0:b1])),
                       cam_z=float(sz[a0:a1, b0:b1].sum() / nd) if nd else None, hd=float(sh[a0:a1, b0:b1].sum() / nd) if nd else None,
                       dist_robot=float(math.hypot((c[0] + .5) * G10 - rx, (c[1] + .5) * G10 - ry)))
            rec['cells5'] = [[round(float(hiXL[a_, b_] - loXL[a_, b_]), 3) if np.isfinite(hiXL[a_, b_]) else None,
                              round(float(I2['hiD'][a_, b_] - I2['loD'][a_, b_]), 3) if np.isfinite(I2['hiD'][a_, b_]) else None,
                              round(float(I2['hi'][a_, b_] - I2['lo'][a_, b_]), 3) if np.isfinite(I2['hi'][a_, b_]) else None,
                              bool(I1['T'][a_, b_]), bool(I2['bad0'][a_, b_]), bool(I1['bad0'][a_, b_])] for a_, b_ in ks]
            # G2 에서 새로 세로/솟음이 된 칸(G1 에선 아님) — 이 헛칸 0.5 m 안 개수
            nb_ = I2['bad0'] & ~I1['bad0']; ci_, cj_ = int(ks[:, 0].mean()), int(ks[:, 1].mean())
            rec['newbad_0p5m'] = int(nb_[max(ci_ - 10, 0):ci_ + 11, max(cj_ - 10, 0):cj_ + 11].sum())
            NEW.append(rec)
        def zmean(P):
            k = np.floor((P[:, :2] - org) / CELL).astype(np.int64); ok = (k >= 0).all(1) & (k < nn).all(1) & (P[:, 2] <= zmax)
            s_ = np.zeros(shp); c_ = np.zeros(shp); np.add.at(s_, (k[ok, 0], k[ok, 1]), P[ok, 2]); np.add.at(c_, (k[ok, 0], k[ok, 1]), 1)
            with np.errstate(all='ignore'): return np.where(c_ > 0, s_ / c_, np.nan)
        mX, mL, mD = zmean(X), zmean(L1), zmean(DW)
        # ── 걸어간 길(통과 가능 확정) 위 5 cm 칸: 깊이 단독 높이폭·라이다 대비 오프셋을 거리별로 ──
        if ahead:
            ca = np.array(list(ahead)); cc = []
            for di in (0.25, 0.75):
                for dj in (0.25, 0.75):
                    cc.append(np.floor(((ca + np.array([di, dj])) * G10 - org) / CELL).astype(np.int64))
            cc = np.unique(np.concatenate(cc), axis=0); cc = cc[(cc >= 0).all(1) & (cc < nn).all(1)]
            hD, lD = I2['hiD'][cc[:, 0], cc[:, 1]], I2['loD'][cc[:, 0], cc[:, 1]]; okc = np.isfinite(hD)
            for (a_, b_), h_, l_ in zip(cc[okc], hD[okc], lD[okc]):
                GROUND.append((round(t - t0, 2), float(sz[a_, b_] / cn[a_, b_]), float(sh[a_, b_] / cn[a_, b_]), float(h_ - l_),
                               float(h_ - hiXL[a_, b_]) if np.isfinite(hiXL[a_, b_]) else np.nan, int(cn[a_, b_]),
                               float(mD[a_, b_] - mX[a_, b_]), float(mD[a_, b_] - mL[a_, b_]), float(mL[a_, b_] - mX[a_, b_]),
                               float(hiXL[a_, b_] - loXL[a_, b_]), float(I2['hi'][a_, b_] - I2['lo'][a_, b_])))
        # ── 커버리지: 발 앞 0.3~1.75 m, ±30° 부채꼴 5 cm 칸 중 데이터 있는 비율 ──
        ci = (np.arange(nn) + 0.5) * CELL; gx, gy = np.meshgrid(org[0] + ci, org[1] + ci, indexing='ij')
        dx, dy = gx - rx, gy - ry; rr_ = np.hypot(dx, dy); ang = np.abs((np.arctan2(dy, dx) - ryaw + np.pi) % (2 * np.pi) - np.pi)
        fan = (rr_ >= FAN_R0) & (rr_ <= FAN_R1) & (ang <= FAN_HALF); fn = (rr_ >= FAN_R0) & (rr_ < 1.0) & (ang <= FAN_HALF); ff = (rr_ >= 1.0) & (rr_ <= FAN_R1) & (ang <= FAN_HALF)
        vX, vL, vD = nX > 0, nL > 0, I2['nD'] > 0
        cov = {}
        for nm, v in (('XT16', vX), ('L1', vL), ('깊이', vD), ('XT16+L1', vX | vL), ('XT16+L1+깊이', vX | vL | vD)):
            cov[nm] = [float(v[fan].mean()), float(v[fn].mean()), float(v[ff].mean())]
        zone = 'X자' if math.hypot(rx - CXX, ry - CYY) < 2.5 else '평지'
        near = lambda cs: {c for c in cs if math.hypot((c[0] + .5) * G10 - rx, (c[1] + .5) * G10 - ry) <= 3.0}
        wall_ego = near(res['ego(XT16)']) & WALL
        row = dict(t=round(t - t0, 2), zone=zone, cov=cov, n_new=len(new))
        for k, cs in res.items():
            row[k + '|헛'] = len(cs & ahead); row[k + '|벽'] = (len(dilate(near(cs)) & wall_ego) / len(wall_ego)) if wall_ego else None
        S.append(row)
        if len(S) % 25 == 0: print(tag, len(S), f'{t - t0:.0f}s', flush=True)
    json.dump(dict(S=S, NEW=NEW, GROUND=GROUND), open(f'{HERE}/part_{tag}.json', 'w'), ensure_ascii=False)
    print(tag, 'done', len(S), 'samples', len(NEW), 'new-false cells', flush=True)


def merge():
    S, NEW, GROUND = [], [], []
    for fn in sorted(glob.glob(f"{HERE}/{os.environ.get('PARTS', 'part_*.json')}")):
        if 'smoke' in fn or 'dbg' in fn: continue
        d = json.load(open(fn)); S += d['S']; NEW += d['NEW']; GROUND += d['GROUND']
    S.sort(key=lambda s: s['t']); ts = [s['t'] for s in S]; assert len(ts) == len(set(ts)), '표본 중복'
    keys = ['ego(XT16)'] + FORKS
    summ = {}
    for zone in ('X자', '평지', '전체'):
        Z = [s for s in S if zone == '전체' or s['zone'] == zone]; summ[zone] = {'표본': len(Z)}
        for k in keys:
            v = np.array([s[k + '|헛'] for s in Z]); w = np.array([np.nan if s[k + '|벽'] is None else s[k + '|벽'] for s in Z], float)
            summ[zone][k] = dict(헛장애물_표본비율=round(float((v > 0).mean()), 4), 헛칸_평균=round(float(v.mean()), 3), 벽유지=round(float(np.nanmean(w)), 4),
                                 헛표본수=int((v > 0).sum()))
        # G1 대비 갈래별: 새로 생긴 표본 / 없어진 표본
        g1 = np.array([s['G1|헛'] > 0 for s in Z])
        for k in FORKS[1:]:
            gk = np.array([s[k + '|헛'] > 0 for s in Z]); summ[zone][k]['G1대비_새표본'] = int((gk & ~g1).sum()); summ[zone][k]['G1대비_사라진표본'] = int((~gk & g1).sum())
        cv = {}
        for nm in ('XT16', 'L1', '깊이', 'XT16+L1', 'XT16+L1+깊이'):
            a = np.array([s['cov'][nm] for s in Z]); cv[nm] = dict(부채꼴_0p3_1p75=round(float(a[:, 0].mean()), 3), 근_0p3_1p0=round(float(a[:, 1].mean()), 3), 원_1p0_1p75=round(float(a[:, 2].mean()), 3))
        summ[zone]['커버리지'] = cv
    # 새 헛장애물 진단
    from collections import Counter
    dg = {}
    for zone in ('X자', '평지', '전체'):
        R = [n for n in NEW if zone == '전체' or n['zone'] == zone]
        def q(key):
            a = np.array([n[key] for n in R if n[key] is not None], float)
            return dict(n=len(a), p10=round(float(np.percentile(a, 10)), 3), 중앙=round(float(np.median(a)), 3), p90=round(float(np.percentile(a, 90)), 3)) if len(a) else None
        dg[zone] = dict(칸수=len(R), 이유=dict(Counter(n['why'] for n in R)),
                        깊이높이폭=q('d_spread'), 깊이최고_minus_라이다최고=q('d_hi_minus_lidar_hi'), 깊이최저_minus_라이다최저=q('d_lo_minus_lidar_lo'),
                        라이다높이폭=q('lidar_spread'), 합친격자높이폭=q('g2_spread'),
                        카메라깊이_m=q('cam_z'), 깊이점_수평거리_m=q('hd'), 칸_로봇거리_m=q('dist_robot'))
        for why in set(n['why'] for n in R):
            Rw = [n for n in R if n['why'] == why]
            a = np.array([n['cam_z'] for n in Rw if n['cam_z'] is not None]); b = np.array([n['d_hi_minus_lidar_hi'] for n in Rw if n['d_hi_minus_lidar_hi'] is not None])
            dg[zone]['이유별_' + why] = dict(n=len(Rw), 카메라깊이_중앙=round(float(np.median(a)), 3) if len(a) else None, 깊이최고_minus_라이다최고_중앙=round(float(np.median(b)), 3) if len(b) else None)
    # 걸어간 길 위 깊이 단독 높이폭 / 라이다 대비 오프셋 (카메라 깊이 구간별)
    G = np.array(GROUND, float) if GROUND else np.zeros((0, 11)); gd = {}
    for lo_, hi_ in ((0.3, 1.0), (1.0, 1.5), (1.5, 2.0), (2.0, 2.5)):
        m = (G[:, 1] >= lo_) & (G[:, 1] < hi_)
        if not m.any(): continue
        sp, off, cnt = G[m, 3], G[m, 4], G[m, 5]; off = off[np.isfinite(off)]
        gd[f'{lo_}~{hi_}m'] = dict(칸수=int(m.sum()), 깊이높이폭_중앙=round(float(np.median(sp)), 3), 깊이높이폭_p90=round(float(np.percentile(sp, 90)), 3),
                                    높이폭_gt0p20_비율=round(float((sp > 0.20).mean()), 4), 높이폭_gt0p10_비율=round(float((sp > 0.10).mean()), 4),
                                    깊이최고_minus_라이다최고_중앙=round(float(np.median(off)), 3) if len(off) else None,
                                    깊이최고_minus_라이다최고_p90=round(float(np.percentile(off, 90)), 3) if len(off) else None, 칸당점수_중앙=float(np.median(cnt)))
        if G.shape[1] >= 11:
            for j, nm in ((6, '평균z_깊이_minus_XT16'), (7, '평균z_깊이_minus_L1'), (8, '평균z_L1_minus_XT16')):
                a = G[m, j]; a = a[np.isfinite(a)]
                gd[f'{lo_}~{hi_}m'][nm] = dict(n=int(len(a)), 중앙=round(float(np.median(a)), 3), p10=round(float(np.percentile(a, 10)), 3), p90=round(float(np.percentile(a, 90)), 3)) if len(a) else None
            a, b = G[m, 9], G[m, 10]; ok_ = np.isfinite(a) & np.isfinite(b)
            if ok_.any():
                gd[f'{lo_}~{hi_}m']['칸높이폭_라이다만_중앙'] = round(float(np.median(a[ok_])), 3); gd[f'{lo_}~{hi_}m']['칸높이폭_합침_중앙'] = round(float(np.median(b[ok_])), 3)
                gd[f'{lo_}~{hi_}m']['칸높이폭_합침_minus_라이다만_중앙'] = round(float(np.median(b[ok_] - a[ok_])), 3)
                gd[f'{lo_}~{hi_}m']['칸높이폭_라이다만_gt0p20'] = round(float((a[ok_] > 0.2).mean()), 4); gd[f'{lo_}~{hi_}m']['칸높이폭_합침_gt0p20'] = round(float((b[ok_] > 0.2).mean()), 4)
    out = dict(설정=dict(표본간격_s=DT, 갈래=FORKS, 부채꼴=[FAN_R0, FAN_R1, 30]), 요약=summ, 새헛장애물_진단_G2대비G1=dg, 걸어간길_깊이품질=gd)
    out['G1대비_짝비교'] = {}
    from math import comb
    for zone in ('X자', '평지', '전체'):
        Z = [s for s in S if zone == '전체' or s['zone'] == zone]; g1 = np.array([s['G1|헛'] > 0 for s in Z]); out['G1대비_짝비교'][zone] = {}
        for k in FORKS[1:]:
            gk = np.array([s[k + '|헛'] > 0 for s in Z]); a, b = int((gk & ~g1).sum()), int((~gk & g1).sum()); n = a + b
            p = min(1.0, 2 * sum(comb(n, i) for i in range(0, min(a, b) + 1)) / 2 ** n) if n else 1.0   # 부호검정(McNemar 정확) 양측
            out['G1대비_짝비교'][zone][k] = dict(새로생김=a, 사라짐=b, 부호검정_p=round(p, 4))
    json.dump(out, open(f"{HERE}/{os.environ.get('OUTJ', 'depth_forks_results.json')}", 'w'), ensure_ascii=False, indent=1)
    print(json.dumps(out, ensure_ascii=False, indent=1))


if __name__ == '__main__':
    cmd = sys.argv[1]
    if cmd == 'prep': prep()
    elif cmd == 'run': run(float(sys.argv[2]), float(sys.argv[3]), sys.argv[4])
    elif cmd == 'merge': merge()
