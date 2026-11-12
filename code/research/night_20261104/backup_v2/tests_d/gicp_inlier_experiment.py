#!/usr/bin/env python3
"""② 문턱 실험 — rms 게이트(하위 70% RMS <= 0.08)가 틀린 자세를 통과시키는가, '0.10 m 안 점 비율'은 가르는가.

지도: 앞 270초 FAST-LIO 지도(map_first270s_fastlio.npz)  스캔: 뒤 270~542초에서 고르게 N장 (공정 시험과 같은 조건)
각 스캔: 수평 세우기(다리 roll·pitch) + 1500점 표본 = BackupEstimator.prepare 와 같은 처리
  정상   : FAST-LIO 자세를 씨앗으로 GICP → T_ok (정상 정합 분포)
  비틀기 : T_ok 를 x / y 0.1·0.2·0.3·0.5 m (±), yaw 2·4·6° (±) 비튼 '틀린 자세'에서 그대로 잰 값 (정합기가 틀린 곳에 멈춘 경우)
  재정합 : 같은 비틀기를 씨앗으로 GICP 를 다시 돌린 결과 (실제로 틀린 곳에 멈추는지)
잰 값: rms(현행 정의), in10 = 0.10 m 안 점 비율(지도 0.10 m 다운샘플 기준 = 현행 kd), in10d = 같은 비율(지도 0.05 m 기준)
읽기만: bag, gosu1030v2_map.npz, map_first270s_fastlio.npz, v_gosu/backup/backup_core.py   쓰기: 이 폴더
사용: python gicp_inlier_experiment.py [N=120]
"""
import sys, os, math, bisect, json, time, numpy as np, rosbag2_py
from scipy.spatial import cKDTree
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
sys.path.insert(0, '/home/ilovehan9283/ws/night_20261104/backup_v2')
sys.path.insert(1, '/home/ilovehan9283/ws/v_gosu/backup')
from backup_core import load_map, q2R, rpy_of, R_LIDAR, T_LIDAR, voxel, wrap
import small_gicp

N = int(sys.argv[1]) if len(sys.argv) > 1 else 120
SPLIT = 270.0
BAG = '/home/ilovehan9283/bags/gosu_20261030_001126'
NPZ = '/home/ilovehan9283/ws/v_gosu/eval/out/gosu1030v2_map.npz'
HALF_NPZ = '/home/ilovehan9283/ws/night_20261104/backup_eval/map_first270s_fastlio.npz'
OUT = os.path.dirname(os.path.abspath(__file__))


def reader(topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics)); return r


t_start = time.time()
# ── 기준 궤적 대응 (split_validate_gicp_var.py 와 같은 계산)
d = np.load(NPZ); odom, lid = d['odom'], d['lid']
hdr2bag = {}
r = reader(['/lidar_points'])
while r.has_next():
    tp, raw, t = r.read_next(); s, ns = np.frombuffer(raw[4:8], '<i4')[0], np.frombuffer(raw[8:12], '<u4')[0]
    hdr2bag[round(float(s) + float(ns) * 1e-9, 4)] = t * 1e-9
pairs = np.array([(w, hdr2bag[round(h, 4)]) for w, h in lid if round(h, 4) in hdr2bag])
Kp = float(np.median(pairs[:, 0] - pairs[:, 1])); wl = np.sort(lid[:, 0]); h_o = odom[:, 1]
dmed = float(np.median(h_o - wl[np.clip(np.searchsorted(wl, h_o) - 1, 0, len(wl) - 1)]))
tL = h_o - dmed - Kp; o = np.argsort(tL); tL = tL[o]; GT = odom[o]
t0 = tL[0]
def gt_at(ts): return GT[min(max(bisect.bisect_left(tL, ts), 0), len(tL) - 1)]

# ── 고를 스캔 시각 (뒤 절반 고르게)
want = list(np.linspace(t0 + SPLIT + 1.0, tL[-1] - 1.0, N))
legs_t, legs = [], []; scans = []
r = reader(['/utlidar/robot_odom', '/lidar_points']); wi = 0
while r.has_next() and wi < len(want):
    tp, raw, t = r.read_next(); t *= 1e-9
    if t < t0 + SPLIT - 2: continue
    if tp == '/utlidar/robot_odom':
        m = deserialize_message(raw, Odometry); q = m.pose.pose.orientation
        rr_, pp, yy = rpy_of(q2R(q.x, q.y, q.z, q.w)); legs_t.append(t); legs.append((rr_, pp))
    elif t >= want[wi] and legs:
        wi += 1
        m = deserialize_message(raw, PointCloud2)
        scans.append((t, pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64), legs[-1]))
print(f'스캔 {len(scans)}장 읽음 ({time.time() - t_start:.0f}s)', flush=True)

mp = voxel(load_map(HALF_NPZ), 0.05)
target, tree = small_gicp.preprocess_points(np.ascontiguousarray(mp), 0.10, 10, 4)
kd10 = cKDTree(np.asarray(target.points())[:, :3]); kd05 = cKDTree(mp)
rng = np.random.default_rng(0)


def metrics(P, T):
    Q = (T[:3, :3] @ P.T).T + T[:3, 3]
    dd, _ = kd10.query(Q, k=1, distance_upper_bound=1.0); fin = dd[np.isfinite(dd)]
    rms = np.inf
    if len(fin) >= 50:
        thr = np.quantile(fin, 0.7); rms = float(np.sqrt((fin[fin <= thr] ** 2).mean()))
    d5, _ = kd05.query(Q, k=1, distance_upper_bound=1.0)
    return rms, float((dd <= 0.10).mean()), float((d5 <= 0.10).mean()), float((d5 <= 0.05).mean())


def align(P, T0):
    src, _ = small_gicp.preprocess_points(np.ascontiguousarray(P), 0.15, 10, 4)
    rr = small_gicp.align(target, src, tree, init_T_target_source=T0, registration_type='GICP',
                          max_correspondence_distance=1.0, num_threads=4, max_iterations=60)
    return np.asarray(rr.T_target_source)


def perturb(T, dx=0.0, dy=0.0, dyaw_deg=0.0):
    c, s = math.cos(math.radians(dyaw_deg)), math.sin(math.radians(dyaw_deg))
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    T2 = T.copy(); T2[:3, :3] = Rz @ T[:3, :3]; T2[0, 3] += dx; T2[1, 3] += dy   # yaw 는 로봇 위치 둘레
    return T2


PERT = [('x', v) for v in (0.1, 0.2, 0.3, 0.5)] + [('y', v) for v in (0.1, 0.2, 0.3, 0.5)] + [('yaw', v) for v in (2, 4, 6)]
rows_ok, rows_p, rows_re = [], [], []
for t, pts, (lr, lp) in scans:
    g = gt_at(t); gyaw = rpy_of(q2R(*g[5:9]))[2]
    B = pts @ R_LIDAR.T + T_LIDAR
    rr_ = np.hypot(B[:, 0], B[:, 1]); B = B[(rr_ > 0.7) & (rr_ < 12.0)]
    cr, sr, cp, sp = math.cos(lr), math.sin(lr), math.cos(lp), math.sin(lp)
    Rrp = np.array([[cp, sp * sr, sp * cr], [0, cr, -sr], [-sp, cp * sr, cp * cr]])
    Pall = B @ Rrp.T
    P = Pall if len(Pall) <= 1500 else Pall[rng.choice(len(Pall), 1500, replace=False)]
    T0 = np.eye(4); c, s = math.cos(gyaw), math.sin(gyaw)
    T0[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]; T0[:3, 3] = [g[2], g[3], g[4]]
    Tok = align(P, T0)
    exy = math.hypot(Tok[0, 3] - g[2], Tok[1, 3] - g[3]); eyaw = math.degrees(wrap(math.atan2(Tok[1, 0], Tok[0, 0]) - gyaw))
    rows_ok.append((t - t0, exy, eyaw) + metrics(P, Tok))
    for ax, v in PERT:
        for sg in (1, -1):
            kw = {'x': dict(dx=sg * v), 'y': dict(dy=sg * v), 'yaw': dict(dyaw_deg=sg * v)}[ax]
            Tp = perturb(Tok, **kw)
            rows_p.append((t - t0, ax, v, sg) + metrics(P, Tp))
            Tr = align(P, Tp)
            e2 = math.hypot(Tr[0, 3] - Tok[0, 3], Tr[1, 3] - Tok[1, 3]); y2 = math.degrees(wrap(math.atan2(Tr[1, 0], Tr[0, 0]) - math.atan2(Tok[1, 0], Tok[0, 0])))
            rows_re.append((t - t0, ax, v, sg, e2, y2) + metrics(P, Tr))
print(f'계산 끝 ({time.time() - t_start:.0f}s)', flush=True)

OK = np.array(rows_ok)
np.save(f'{OUT}/inlier_exp_ok.npy', OK)
import pickle
pickle.dump(dict(ok=rows_ok, pert=rows_p, re=rows_re), open(f'{OUT}/inlier_exp_rows.pkl', 'wb'))


def q(a, p): return round(float(np.percentile(a, p)), 3)


res = dict(스캔=len(OK), 정상=dict(xy오차_p95=q(OK[:, 1], 95), yaw오차_p95_deg=q(np.abs(OK[:, 2]), 95),
          rms_중앙=q(OK[:, 3], 50), rms_최대=q(OK[:, 3], 100), in10_최소=q(OK[:, 4], 0), in10_p1=q(OK[:, 4], 1), in10_p5=q(OK[:, 4], 5), in10_중앙=q(OK[:, 4], 50),
          in10d_최소=q(OK[:, 5], 0), in10d_p1=q(OK[:, 5], 1), in10d_p5=q(OK[:, 5], 5), in10d_중앙=q(OK[:, 5], 50)), 비틀기={}, 재정합={})
for ax, v in PERT:
    A = np.array([rw[4:] for rw in rows_p if rw[1] == ax and rw[2] == v], float)
    res['비틀기'][f'{ax}{v}'] = dict(rms통과율=round(float((A[:, 0] <= 0.08).mean()), 3), rms_중앙=q(A[:, 0], 50),
                                   in10_중앙=q(A[:, 1], 50), in10_p95=q(A[:, 1], 95), in10_최대=q(A[:, 1], 100),
                                   in10d_중앙=q(A[:, 2], 50), in10d_p95=q(A[:, 2], 95), in10d_최대=q(A[:, 2], 100))
    R = [rw for rw in rows_re if rw[1] == ax and rw[2] == v]
    E = np.array([rw[4:] for rw in R], float)
    bad = (E[:, 0] > 0.10) | (np.abs(E[:, 1]) > 2.0)          # 다시 돌려도 0.10 m / 2° 밖에 멈춘 것 = 실제 틀린 정합
    res['재정합'][f'{ax}{v}'] = dict(틀린곳멈춤=int(bad.sum()), 그중_rms통과=int((bad & (E[:, 2] <= 0.08)).sum()),
                                   그중_in10=[round(float(x), 3) for x in E[bad, 3][:12]])
json.dump(res, open(f'{OUT}/inlier_exp_result.json', 'w'), ensure_ascii=False, indent=1)
print(json.dumps(res, ensure_ascii=False, indent=1))
print(f'총 {time.time() - t_start:.0f}s')
