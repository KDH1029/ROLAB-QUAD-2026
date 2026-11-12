#!/usr/bin/env python3
"""②③ yd_extract.py 결과(dets.npz, depth_frames.npy) → 검출별 3D(몸체·세계) + 검증 통계.

실행: ~/venv/go2/bin/python yd_3d.py [shrink=0.6] [tag]
쓰기: 이 폴더 det3d[_tag].json (검출별), summary[_tag].json (표 숫자)
검증: ① 같은 정지 물체 여러 프레임 → 세계 좌표 퍼짐(단일연결 0.3 m 군집)
      ② 3D 점 → FAST-LIO 지도(gosu1030v2_map.npz) 최근접 거리
      ③ 박스(가운데 60%) 안 깊이 점 개수 (stride 2 = 픽셀 4개 중 1개)
"""
import sys, os, json
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as Rot, Slerp
from scipy.cluster.hierarchy import fcluster, linkage

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import yd_core as C

NPZ = '/home/ilovehan9283/ws/v_gosu/eval/out/gosu1030v2_map.npz'
SHRINK = float(sys.argv[1]) if len(sys.argv) > 1 else 0.6
TAG = sys.argv[2] if len(sys.argv) > 2 else ''
FLIP = os.environ.get('YD_FLIP_TX') == '1'          # A/B: 컬러-깊이 x 오프셋 부호 반대로

E = np.load(f'{HERE}/dets.npz'); FR, DET = E['frames'], E['dets']
DEP = np.load(f'{HERE}/depth_frames.npy', mmap_mode='r')
T_bd, ST, DELTA = C.load_calib()
lc = C.T_LINK_COLOR
if FLIP: lc = ([lc[0][0], -lc[0][1], lc[0][2]], lc[1])
T_cd = C.T_copt_dopt(lc)

# FAST-LIO 궤적을 bag 시각으로 (calib_depth.py 와 같은 대응)
d = np.load(NPZ); odom, lid = d['odom'], d['lid']; V = float(d['V'])
FULL = (d['vox'][d['cnt'] >= 2].astype(np.float64) + 0.5) * V
h2b = {round(float(h), 4): b for h, b in E['hdr2bag']}
pairs = np.array([(w, h2b[round(h, 4)]) for w, h in lid if round(h, 4) in h2b])
Kp = float(np.median(pairs[:, 0] - pairs[:, 1])); wl = np.sort(lid[:, 0]); h_o = odom[:, 1]
dmed = float(np.median(h_o - wl[np.clip(np.searchsorted(wl, h_o) - 1, 0, len(wl) - 1)]))
tL = h_o - dmed - Kp; o = np.argsort(tL); tL = tL[o]; GT = odom[o]
print('FAST-LIO 자세', len(tL), '구간', round(tL[0], 1), '~', round(tL[-1], 1), 'lidar 대응', len(pairs))


def pose_at(ts):
    if ts < tL[0] or ts > tL[-1]: return None
    i = int(np.clip(np.searchsorted(tL, ts), 1, len(tL) - 1)); a = (ts - tL[i - 1]) / max(tL[i] - tL[i - 1], 1e-6); a = min(max(a, 0), 1)
    p = GT[i - 1, 2:5] * (1 - a) + GT[i, 2:5] * a
    R = Slerp([0, 1], Rot.from_quat(GT[i - 1:i + 1, 5:9]))([a])[0].as_matrix()
    return R, p


kd = cKDTree(FULL)
rows = []
for fi in np.unique(DET[:, 0]).astype(int):
    f = FR[fi]; di = int(f[4])
    if di < 0: continue
    t_nuc = C.stamp_to_nuc(f[5], ST) + DELTA            # 깊이 stamp → NUC, 남은 지연 δ
    P = pose_at(t_nuc)
    if P is None: continue
    dets = DET[DET[:, 0] == fi][:, 1:]
    for r in C.detect_to_world(dets, np.asarray(DEP[di]), T_bd, P[0], P[1], T_cd, shrink=SHRINK):
        r.update(frame=int(fi), t=float(t_nuc), dt_color_depth=float(f[1] - f[5]), robot_xy=P[1][:2].tolist(), robot_z=float(P[1][2]))
        if r['ok']:
            pw = np.array(r['p_world']); dd, _ = kd.query(pw); r['map_nn'] = float(dd)
            # 지도 바닥: 그 자리 수평 0.3 m 안 지도점 z 하위 5%
            idx = kd.query_ball_point(pw, 0.6)
            Q = FULL[idx] if idx else np.zeros((0, 3))
            Q = Q[np.hypot(Q[:, 0] - pw[0], Q[:, 1] - pw[1]) < 0.3] if len(Q) else Q
            r['map_floor_z'] = float(np.percentile(Q[:, 2], 5)) if len(Q) >= 5 else None
        rows.append(r)

json.dump(rows, open(f'{HERE}/det3d{("_" + TAG) if TAG else ""}.json', 'w'), ensure_ascii=False)
ok = [r for r in rows if r['ok']]
print('검출(깊이 프레임 있음)', len(rows), '3D 성공', len(ok), 'shrink', SHRINK, 'flip', FLIP)


def pct(a, q=(10, 50, 90)):
    a = np.asarray([x for x in a if x is not None and np.isfinite(x)], float)
    return [round(float(np.percentile(a, x)), 3) for x in q] if len(a) else [None] * len(q)


S = dict(shrink=SHRINK, flip=FLIP, n_frames=int(len(FR)), n_det=int(len(DET)), classes={})
for c in sorted(set(int(x) for x in DET[:, 1])):
    R_ = [r for r in rows if r['cls'] == c]; O = [r for r in R_ if r['ok']]
    s = dict(n_det=int((DET[:, 1] == c).sum()), n_with_depth=len(R_), n_3d=len(O),
             conf_p10_50_90=pct([r['conf'] for r in R_]),
             n_pts_p10_50_90=pct([r['n_pts'] for r in R_]),
             consist_p10_50_90=pct([r['consist'] for r in O]),
             range_m_p10_50_90=pct([r['range'] for r in O]),
             map_nn_m_p10_50_90=pct([r['map_nn'] for r in O]),
             map_nn_lt_5cm=round(float(np.mean([r['map_nn'] < 0.05 for r in O])), 3) if O else None,
             map_nn_lt_10cm=round(float(np.mean([r['map_nn'] < 0.10 for r in O])), 3) if O else None)
    # 군집 (세계 xy+z, 단일연결 0.3 m)
    if len(O) >= 2:
        X = np.array([r['p_world'] for r in O]); lab = fcluster(linkage(X, 'single'), 0.3, 'distance')
        cl = []
        for L in np.unique(lab):
            m = lab == L
            if m.sum() < 3: continue
            Xm = X[m]; med = np.median(Xm, 0); dxy = np.hypot(*(Xm[:, :2] - med[:2]).T); d3 = np.linalg.norm(Xm - med, axis=1)
            ts = np.array([O[i]['t'] for i in np.where(m)[0]]); rg = np.array([O[i]['range'] for i in np.where(m)[0]])
            gz = [O[i]['ground_z'] for i in np.where(m)[0] if O[i]['ground_z'] is not None]
            mf = [O[i]['map_floor_z'] for i in np.where(m)[0] if O[i]['map_floor_z'] is not None]
            cl.append(dict(n=int(m.sum()), center=med.round(3).tolist(), span_s=round(float(ts.max() - ts.min()), 1),
                           range_min_max=[round(float(rg.min()), 2), round(float(rg.max()), 2)],
                           xy_med_cm=round(float(np.median(dxy)) * 100, 1), xy_p90_cm=round(float(np.percentile(dxy, 90)) * 100, 1),
                           xy_max_cm=round(float(dxy.max()) * 100, 1), d3_med_cm=round(float(np.median(d3)) * 100, 1),
                           z_std_cm=round(float(Xm[:, 2].std()) * 100, 1),
                           ground_z_med=round(float(np.median(gz)), 3) if gz else None,
                           map_floor_z_med=round(float(np.median(mf)), 3) if mf else None,
                           map_nn_med_cm=round(float(np.median([O[i]['map_nn'] for i in np.where(m)[0]])) * 100, 1)))
        cl.sort(key=lambda x: -x['n'])
        s['n_clusters_ge3'] = len(cl); s['clusters'] = cl[:15]
        s['clustered_frac'] = round(sum(x['n'] for x in cl) / len(O), 3)
    S['classes'][C.NAMES[c]] = s

json.dump(S, open(f'{HERE}/summary{("_" + TAG) if TAG else ""}.json', 'w'), ensure_ascii=False, indent=1)
print(json.dumps(S, ensure_ascii=False, indent=1)[:6000])
