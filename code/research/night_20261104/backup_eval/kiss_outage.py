#!/usr/bin/env python3
"""정합 끊김(outage) 시험 — 백업 위치추정의 예측원: (a) 다리 오도 vs (b) KISS-ICP(x,y,yaw) + 다리(z,roll,pitch).

공정 시험 구성(split_validate_gicp_var.py 와 같음): 지도 = 앞 270초 FAST-LIO 지도(map_first270s_fastlio.npz), 시험 = 270~542초
정합 = GicpMatcher(need_converged, iters 30) + RobustGroundGrid, rms_ok 0.08, 5 Hz (RESULTS.md '채택')
끊김 창: 10초 × 4 (300/360/420/480 시작), 30초 × 1 (380 시작) — 창마다 따로 돌린다(서로 오염 안 되게) + 끊김 없는 기준 1회
창 동안 on_scan 을 부르지 않고 예측만. KISS 는 0초부터 전 프레임 돌려 워밍업 (kiss_eval.py 와 같은 설정, 복셀 0.15, 20 m)
읽기만: ~/bags/gosu_20261030_001126, ~/ws/v_gosu/eval/out/gosu1030v2_map.npz, ~/ws/v_gosu/backup/backup_core.py
쓰기: 이 폴더 kiss_outage.json
"""
import sys, math, bisect, json, time, os, numpy as np, rosbag2_py
from multiprocessing import Pool
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
sys.path.insert(0, '/home/ilovehan9283/ws/v_gosu/backup')
from backup_core import BackupEstimator, load_map, q2R, rpy_of, R_LIDAR, T_LIDAR, voxel, wrap
sys.path.insert(0, '/home/ilovehan9283/ws/night_20261104/backup_eval')
from gicp_matcher import GicpMatcher
from robust_ground import RobustGroundGrid
from kiss_icp.kiss_icp import KissICP
from kiss_icp.config import KISSConfig

BAG = '/home/ilovehan9283/bags/gosu_20261030_001126'
NPZ = '/home/ilovehan9283/ws/v_gosu/eval/out/gosu1030v2_map.npz'
OUT = '/home/ilovehan9283/ws/night_20261104/backup_eval'
HALF_NPZ = f'{OUT}/map_first270s_fastlio.npz'
SPLIT = 270.0
WINDOWS = [('10s@300', 300, 10), ('10s@360', 360, 10), ('10s@420', 420, 10), ('10s@480', 480, 10), ('30s@380', 380, 30)]
dt_lid = np.dtype({'names': ['x', 'y', 'z', 'ts'], 'formats': ['<f4', '<f4', '<f4', '<f8'], 'offsets': [0, 4, 8, 18], 'itemsize': 26})


def reader(topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics)); return r


_T = time.time()
d = np.load(NPZ); odom, lid = d['odom'], d['lid']
# ── 한 번 읽기: 라이다(헤더 시각 대응 + KISS + 시험 스캔 저장) + 다리 오도
cfg = KISSConfig(); cfg.data.max_range = 20.0; cfg.data.min_range = 0.6; cfg.data.deskew = True; cfg.mapping.voxel_size = 0.15
kiss = KissICP(cfg)
hdr2bag = {}; kt = []; kT = []; scans = []; legs = []; tb0 = None; last = -1e9; kms = []
r = reader(['/utlidar/robot_odom', '/lidar_points'])
while r.has_next():
    tp, raw, t = r.read_next(); t *= 1e-9
    if tb0 is None: tb0 = t
    if tp == '/utlidar/robot_odom':
        if t < tb0 + SPLIT - 60: continue
        m = deserialize_message(raw, Odometry); p, q = m.pose.pose.position, m.pose.pose.orientation
        rr_, pp, yy = rpy_of(q2R(q.x, q.y, q.z, q.w)); legs.append((t, p.x, p.y, p.z, yy, rr_, pp)); continue
    s, ns = np.frombuffer(raw[4:8], '<i4')[0], np.frombuffer(raw[8:12], '<u4')[0]
    hdr2bag[round(float(s) + float(ns) * 1e-9, 4)] = t
    m = deserialize_message(raw, PointCloud2)
    a = np.frombuffer(bytes(m.data), dtype=dt_lid, count=m.width * m.height)
    ok = np.isfinite(a['x']) & np.isfinite(a['y']) & np.isfinite(a['z'])
    P = np.c_[a['x'][ok], a['y'][ok], a['z'][ok]].astype(np.float64); ts = a['ts'][ok]
    if len(P) >= 500:
        tsn = (ts - ts.min()) / max(ts.max() - ts.min(), 1e-6)
        _t = time.perf_counter(); kiss.register_frame(P, tsn); kms.append((time.perf_counter() - _t) * 1e3)
        kt.append(t); kT.append(kiss.last_pose.copy())
    if t >= tb0 + SPLIT - 60 and t - last >= 0.1:
        last = t
        rg = np.linalg.norm(P, axis=1); scans.append((t, P[(rg > 0.3) & (rg < 13.5)]))   # prepare() 가 몸체 좌표로 다시 0.7~12 m 거른다(상위집합)
print(f'읽기 완료 {time.time() - _T:.0f}s, KISS {len(kt)} 프레임 {np.median(kms):.1f} ms, 스캔 {len(scans)}, 다리 {len(legs)}', flush=True)

pairs = np.array([(w, hdr2bag[round(h, 4)]) for w, h in lid if round(h, 4) in hdr2bag])
Kp = float(np.median(pairs[:, 0] - pairs[:, 1])); wl = np.sort(lid[:, 0]); h_o = odom[:, 1]
dmed = float(np.median(h_o - wl[np.clip(np.searchsorted(wl, h_o) - 1, 0, len(wl) - 1)]))
tL = h_o - dmed - Kp; o = np.argsort(tL); tL = tL[o]; GT = odom[o]; t0 = tL[0]
def gt_at(ts): return GT[min(max(bisect.bisect_left(tL, ts), 0), len(tL) - 1)]

# ── KISS 몸체 자세(세계): 첫 FAST-LIO 시각 프레임 기준으로 수평만 맞춤(증분만 쓰므로 원점·방향은 무관)
kt = np.array(kt); T_bl = np.eye(4); T_bl[:3, :3] = R_LIDAR; T_bl[:3, 3] = T_LIDAR; T_lb = np.linalg.inv(T_bl)
iref = int(np.searchsorted(kt, tL[0])); g = gt_at(kt[iref]); Tg = np.eye(4); Tg[:3, :3] = q2R(*g[5:9]); Tg[:3, 3] = g[2:5]
A = Tg @ T_bl @ np.linalg.inv(kT[iref])
KB = np.array([(lambda W: (W[0, 3], W[1, 3], rpy_of(W[:3, :3])[2]))(A @ Tk @ T_lb) for Tk in kT])   # (x, y, yaw)
KZ = np.array([(A @ Tk @ T_lb)[2, 3] for Tk in kT])

ev = [(t, 'leg', (x, y, z, yy, rr_, pp)) for t, x, y, z, yy, rr_, pp in legs if t >= t0 + SPLIT - 2] + \
     [(t, 'scan', P) for t, P in scans if t >= t0 + SPLIT - 2 and t < tL[-1]]
ev.sort(key=lambda e: e[0])
HALF = load_map(HALF_NPZ)


def run(job):
    branch, wname, ws, wl_ = job
    w0, w1 = (t0 + ws, t0 + ws + wl_) if ws is not None else (1e18, 1e18)
    mp = HALF; est = BackupEstimator(mp, rms_ok=0.08)
    est.matcher = GicpMatcher(voxel(mp, 0.05), need_converged=True, iters=30, threads=2)
    est.ground = RobustGroundGrid(voxel(mp, 0.05))
    leg = None; lastsc = -1e9; tr = []; reacq = None; end_err = None; win_max = 0.0; rej_after = 0
    for t, kind, val in ev:
        if kind == 'leg':
            if branch == 'kiss':
                i = int(np.searchsorted(kt, t, side='right')) - 1        # t 이전에 끝난 KISS 프레임
                kx, ky, kyaw = KB[i]; val = (kx, ky, val[2], kyaw, val[4], val[5])
            leg = val
            if est.inited: est.on_leg(val)
            continue
        if leg is None or t < t0 + SPLIT or t - lastsc < 0.2 - 1e-3: continue
        lastsc = t
        if not est.inited:
            g = gt_at(t); est.init(g[2], g[3], g[4], rpy_of(q2R(*g[5:9]))[2], leg, t)
        x, y, *_ = est.pose(); g = gt_at(t); e_pred = math.hypot(x - g[2], y - g[3])
        in_win = w0 <= t < w1
        if in_win:
            win_max = max(win_max, e_pred); tr.append((t - t0, e_pred, 1)); continue
        if t >= w1 and end_err is None: end_err = e_pred                     # 창 끝 = 창이 끝나고 첫 정합 직전의 예측 오차
        out = est.on_scan(val, t); x, y, *_ = est.pose(); e = math.hypot(x - g[2], y - g[3])
        tr.append((t - t0, e, 0))
        if t >= w1 and reacq is None:
            if out and out['accepted']: reacq = t - w1
            else: rej_after += 1
    T = np.array(tr); e = T[:, 1]; post = (T[:, 0] >= (w1 - t0)) & (T[:, 0] < (w1 - t0) + 10) if ws is not None else np.zeros(len(T), bool)
    res = dict(갈래=branch, 창=wname, 정합=int((T[:, 2] == 0).sum()), 수락=est.n_ok, 거부=est.n_bad,
               전체_중앙=round(float(np.median(e)), 3), 전체_p95=round(float(np.percentile(e, 95)), 3), 전체_최대=round(float(e.max()), 3))
    if ws is not None:
        res.update(창끝_오차=round(float(end_err), 3), 창중_최대=round(float(win_max), 3),
                   재수락_s=None if reacq is None else round(float(reacq), 2), 재수락전_거부=rej_after,
                   창후10s_최대=round(float(e[post].max()), 3) if post.any() else None)
    return res


if __name__ == '__main__':
    jobs = [(b, 'none', None, None) for b in ('leg', 'kiss')] + [(b, n, s, l) for b in ('leg', 'kiss') for n, s, l in WINDOWS]
    with Pool(len(jobs)) as pool:
        R = pool.map(run, jobs)
    for x in R: print(x, flush=True)
    # 예측원 단독 표류(정합 없이): 창 시작에서 FAST-LIO 에 맞춘 뒤 창 끝 xy 오차 — 다리 오도 vs KISS
    Lt = np.array([l[0] for l in legs]); LX = np.array([l[1:5] for l in legs])
    def drift(src, ws, wl_):
        a_, b_ = t0 + ws, t0 + ws + wl_
        if src == 'kiss':
            ia, ib = int(np.searchsorted(kt, a_)), int(np.searchsorted(kt, b_)) - 1; pa, pb = KB[ia], KB[ib]; ta, tb = kt[ia], kt[ib]
        else:
            ia, ib = int(np.searchsorted(Lt, a_)), int(np.searchsorted(Lt, b_)) - 1; pa, pb = LX[ia][[0, 1, 3]], LX[ib][[0, 1, 3]]; ta, tb = Lt[ia], Lt[ib]
        ga, gb = gt_at(ta), gt_at(tb); gya = rpy_of(q2R(*ga[5:9]))[2]; gyb = rpy_of(q2R(*gb[5:9]))[2]
        dx, dy = pb[0] - pa[0], pb[1] - pa[1]; c, s = math.cos(gya - pa[2]), math.sin(gya - pa[2])
        px, py = ga[2] + c * dx - s * dy, ga[3] + s * dx + c * dy
        return round(math.hypot(px - gb[2], py - gb[3]), 3), round(math.degrees(abs(wrap((pb[2] - pa[2]) - (gyb - gya)))), 2), round(math.hypot(gb[2] - ga[2], gb[3] - ga[3]), 2)
    D = {n: dict(다리=drift('leg', s, l), KISS=drift('kiss', s, l)) for n, s, l in WINDOWS}
    print('단독 표류 (xy m, yaw°, 이동 m):', D)
    json.dump(dict(설정=dict(지도='map_first270s_fastlio.npz', 시험='270~542s', 정합='GICP+견고지면 rms_ok0.08 5Hz', KISS='vox0.15 maxr20 deskew, 0초부터',
                             KISS_ms_중앙=round(float(np.median(kms)), 1)),
                   결과=R, 단독표류_xy_yaw_이동=D), open(f'{OUT}/kiss_outage.json', 'w'), ensure_ascii=False, indent=1)
    print(f'끝 {time.time() - _T:.0f}s')
