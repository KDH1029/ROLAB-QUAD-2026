#!/usr/bin/env python3
"""③ 전환 장치 가드 오프라인 시험 — v1(MuxLogic 그대로) vs v2(odom_mux_node_v2.GuardedMux: 백업 ok 일 때만 전환).

v_gosu/backup/test_offline.py 와 같은 틀: 10-30 녹화본 전체를 v2 백업 추정기(GICP+견고 지면표, 전체 지도, 5 Hz)로 돌려
백업 위치·건강을 만들고, FAST-LIO 20 Hz 흐름에 고장을 넣어 전환 여부를 본다.
  /backup/status 는 1 Hz(백업 노드와 같음), /backup/odometry 는 감시 주기마다 들어온다고 둔다(살아 있을 때).
시나리오: test_offline 의 넷(정상·끊김·점프·미끄러짐) + 백업 건강이 나쁠 때(lost / degraded / 상태 끊김 / 위치 끊김 / init) + 보류 뒤 회복
ROS 노드 없음 (rosbag2_py 로 파일만 읽음). 쓰기: 이 폴더 test_mux_guard_result.json, bk_traj_full.npy
"""
import sys, os, math, bisect, json, time, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
V2 = '/home/ilovehan9283/ws/night_20261104/backup_v2'
sys.path.insert(0, V2); sys.path.insert(1, '/home/ilovehan9283/ws/v_gosu/backup')
from backup_core import BackupEstimator, MuxLogic, load_map, q2R, rpy_of, voxel
from odom_mux_node_v2 import GuardedMux, HealthGuard

OUT = os.path.dirname(os.path.abspath(__file__))
BAG = '/home/ilovehan9283/bags/gosu_20261030_001126'
NPZ = '/home/ilovehan9283/ws/v_gosu/eval/out/gosu1030v2_map.npz'
HZ = 5.0
CACHE = f'{OUT}/bk_traj_full.npy'


def reader(topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics)); return r


t_start = time.time()
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
def gt_at(ts): return GT[min(max(bisect.bisect_left(tL, ts), 0), len(tL) - 1)]

if os.path.exists(CACHE) and '--recompute' not in sys.argv:
    T = np.load(CACHE); S_est = json.load(open(f'{OUT}/bk_traj_full_summary.json'))
else:
    from gicp_matcher import GicpMatcher
    from robust_ground import RobustGroundGrid
    mpts = load_map(NPZ)
    est = BackupEstimator(mpts, rms_ok=0.08); mp = voxel(mpts, 0.05)          # backup_core_v2.make_estimator 와 같은 구성
    est.matcher = GicpMatcher(mp, need_converged=False, iters=60); est.ground = RobustGroundGrid(mp)
    ev = []
    r = reader(['/utlidar/robot_odom', '/lidar_points']); last_scan = -1e9
    while r.has_next():
        tp, raw, t = r.read_next(); t *= 1e-9
        if tp == '/utlidar/robot_odom':
            m = deserialize_message(raw, Odometry); p, q = m.pose.pose.position, m.pose.pose.orientation
            rr, pp, yy = rpy_of(q2R(q.x, q.y, q.z, q.w)); ev.append((t, 'leg', (p.x, p.y, p.z, yy, rr, pp)))
        elif t - last_scan >= 1.0 / HZ and tL[0] + 1 < t < tL[-1]:
            last_scan = t; m = deserialize_message(raw, PointCloud2)
            ev.append((t, 'scan', pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)))
    ev.sort(key=lambda e: e[0])
    traj = []; leg_now = None
    for t, kind, val in ev:
        if kind == 'leg':
            leg_now = val
            if est.inited: est.on_leg(val)
            continue
        if leg_now is None: continue
        if not est.inited:
            g = gt_at(t); est.init(g[2], g[3], g[4], rpy_of(q2R(*g[5:9]))[2], leg_now, t)
        res = est.on_scan(val, t)
        x, y, z, _, pitch, yaw = est.pose(); g = gt_at(t)
        traj.append((t, x, y, z, yaw, g[2], g[3], g[4], math.degrees(pitch), res['rms'] if res else np.nan, est.health(t) == 'ok'))
    T = np.array(traj); np.save(CACHE, T)
    e = np.hypot(T[:, 1] - T[:, 5], T[:, 2] - T[:, 6])
    S_est = dict(정합=len(T), 수락=est.n_ok, 거부=est.n_bad, 비율거부=est.matcher.n_low_inlier, 위치오차_중앙=round(float(np.median(e)), 3),
                 p95=round(float(np.percentile(e, 95)), 3), 최대=round(float(e.max()), 3), 건강ok비율=round(float(T[:, 10].mean()), 3))
    json.dump(S_est, open(f'{OUT}/bk_traj_full_summary.json', 'w'), ensure_ascii=False, indent=1)
print('① v2 추정기(전체 지도·전체 녹화)', S_est, f'({time.time() - t_start:.0f}s)', flush=True)

bk_t = T[:, 0]; t0 = tL[0]
def bk_at(ts):
    i = min(max(bisect.bisect_right(bk_t, ts) - 1, 0), len(T) - 1); return (T[i, 1], T[i, 2]), ('ok' if T[i, 10] else 'degraded')


def run(name, fault, health=None, status_alive=None, odom_alive=None):
    """fault(ts, xyz)->xyz|None : FAST-LIO 고장.  health(rel)->str|None : 백업 health 덮어쓰기(None 이면 추정기 값).
    status_alive(rel)/odom_alive(rel)->bool : /backup/status · /backup/odometry 가 오는가 (기본 늘 옴)"""
    out = {}
    for ver in ('v1', 'v2'):
        mux = MuxLogic(); gm = GuardedMux(mux=MuxLogic(), guard=HealthGuard()) if ver == 'v2' else None
        sw = None; last_stat = -1e9; holds = []; mode = 'fastlio'
        for i, ts in enumerate(tL):
            xyz = fault(ts, GT[i, 2:5].copy())
            for k in range(4):                       # 12.5 ms 감시 (20 Hz 사이) — test_offline 과 같음
                tt = ts + k * 0.0125; rel = tt - t0
                bxy, h = bk_at(tt)
                if health is not None and health(rel) is not None: h = health(rel)
                if ver == 'v1':
                    if k == 0 and xyz is not None: mux.on_fastlio(ts, *xyz)
                    mode = mux.tick(tt, bxy, h)
                else:
                    if (status_alive is None or status_alive(rel)) and tt - last_stat >= 1.0:
                        gm.guard.on_status(tt, h); last_stat = tt
                    if odom_alive is None or odom_alive(rel):
                        gm.guard.on_odom(tt)
                    if k == 0 and xyz is not None: gm.on_fastlio(ts, *xyz)
                    mode = gm.tick(tt, bxy)
                    for kind, msg in gm.pop_events():
                        if kind == 'hold' and len(holds) < 3: holds.append(f'{rel:.1f}초 {msg}')
                if mode == 'backup' and sw is None:
                    sw = (rel, (mux if ver == 'v1' else gm.mux).reason)
            if sw: break
        r = dict(전환=('없음 (FAST-LIO 유지)' if sw is None else f'{sw[0]:.1f}초: {sw[1]}'))
        if ver == 'v2':
            r['보류경고'] = gm.n_hold_warn
            if holds: r['경고예'] = holds
        out[ver] = r
    print('②', name, json.dumps(out, ensure_ascii=False), flush=True)
    return dict(시나리오=name, **out)


cut200 = lambda ts, p: None if ts - t0 > 200 else p
jump300 = lambda ts, p: p + np.array([3.0, 0, 0]) if ts - t0 > 300 else p
scen = [
    run('정상 주행', lambda ts, p: p),
    run('FAST-LIO 끊김 (200초부터)', cut200),
    run('FAST-LIO 3 m 점프 (300초)', jump300),
    run('서서히 미끄러짐 0.05 m/s (150초부터)', lambda ts, p: p + np.array([0.05 * max(0, ts - t0 - 150), 0, 0])),
    # ── 새 시나리오: 백업이 나쁠 때
    run('끊김 200초 + 백업 health=lost (190초부터 끝까지)', cut200, health=lambda rel: 'lost' if rel > 190 else None),
    run('끊김 200초 + 백업 lost 190~230초 → 회복', cut200, health=lambda rel: 'lost' if 190 < rel < 230 else None),
    run('점프 300초 + 백업 health=degraded (295초부터)', jump300, health=lambda rel: 'degraded' if rel > 295 else None),
    run('끊김 200초 + /backup/status 소식 없음 (195초부터, 백업 노드 죽음)', cut200, status_alive=lambda rel: rel < 195, odom_alive=lambda rel: rel < 195),
    run('끊김 200초 + 백업 상태는 ok 인데 /backup/odometry 끊김 (198초부터)', cut200, odom_alive=lambda rel: rel < 198),
    run('끊김 6초 + 백업 아직 init (12초까지)', lambda ts, p: None if ts - t0 > 6 else p, health=lambda rel: 'init' if rel < 12 else None),
    run('미끄러짐 0.05 m/s (150초부터) + 백업 degraded (145초부터)', lambda ts, p: p + np.array([0.05 * max(0, ts - t0 - 150), 0, 0]),
        health=lambda rel: 'degraded' if rel > 145 else None),
]
v = np.hypot(np.diff(GT[:, 2]), np.diff(GT[:, 3])) / np.maximum(np.diff(tL), 1e-3)
dd = [math.hypot(GT[i, 2] - bk_at(ts)[0][0], GT[i, 3] - bk_at(ts)[0][1]) for i, ts in list(enumerate(tL))[::5]]
margin = dict(FAST_LIO_순간속도_최대=round(float(v.max()), 2), 백업차이_최대=round(float(max(dd)), 3))
print('  정상 주행 여유', margin)
json.dump(dict(추정기=S_est, 전환=scen, 오탐여유=margin), open(f'{OUT}/test_mux_guard_result.json', 'w'), ensure_ascii=False, indent=1)
print(f'총 {time.time() - t_start:.0f}s')
