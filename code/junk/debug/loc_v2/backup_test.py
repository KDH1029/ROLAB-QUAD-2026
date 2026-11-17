# 사본: v_gosu/backup/offline_backup_loc.py (원본은 안 건드림) — 녹화·지도·기준 좌표를 환경변수로
#!/usr/bin/env python3
"""백업 위치추정 오프라인 검증 — FAST-LIO 없이 '다리 오도메트리 + 3D 지도 정합'이 버티는가.

방식 (FAST-LIO 와 독립)
  odom   : /utlidar/robot_odom (다리 기구학, Unitree 공식). 증분(dx, dy, dyaw)만 쓴다
  scan   : XT16 /lidar_points → 몸체 좌표(FAST-LIO 와 같은 외부파라미터)
           ★ 수평 세우기: 다리 오도메트리 자세의 roll·pitch 로 스캔을 중력 기준으로 돌린다 (경사로 대책)
  map    : 사전 3D 지도 (10-30 FAST-LIO 지도, 2 cm 복셀 → 0.1 m)
  정합   : 직전 결과 + 다리 증분을 씨앗으로 3자유도(x, y, yaw) 다해상도 ICP (기존 icp3d 방식), z 는 오도메트리 증분
비교     : 다리 오도메트리 단독 / 정합(수평 세우기 없음) / 정합(수평 세우기) 를 FAST-LIO 궤적 대비 오차로
사용: offline_backup_loc.py [초당 정합 횟수=5]
"""
import sys, math, bisect, json, numpy as np, rosbag2_py
from scipy.spatial import cKDTree
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2

HZ = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
import os as _os
BAG = _os.environ['BAG']
NPZ = _os.environ['NPZ_GT']; MAPNPZ = _os.environ['MAPNPZ']; T_GT = [float(v) for v in _os.environ.get('T_GT', '0,0,0').split(',')]
RX = np.array([[-0.0001707, -0.9999240, 0.0123287], [0.9999844, -0.0001019, 0.0055807], [-0.0055790, 0.0123295, 0.9999084]])
TX = np.array([0.25, 0.0, 0.1588])


def q2R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rpy(R):
    return math.atan2(R[2, 1], R[2, 2]), math.asin(-max(-1, min(1, R[2, 0]))), math.atan2(R[1, 0], R[0, 0])


def voxel(P, v):
    k = np.floor(P / v).astype(np.int64); _, i = np.unique(k, axis=0, return_index=True); return P[i]


class Matcher:   # 기존 ~/ws/lab_visual/src/icp3d.py 와 같은 3자유도 다해상도 ICP (읽기 전용 참고, 사본)
    def __init__(self, mp, stages=((0.40, 2.5, 20), (0.20, 1.0, 15), (0.10, 0.4, 12)), trim=0.7):
        self.levels = [(cKDTree(voxel(mp, vx)), voxel(mp, vx), mc, it) for vx, mc, it in stages]; self.trim = trim

    def align(self, P, T0):
        T = T0.copy(); rms = np.inf
        for tree, mp, mc, it in self.levels:
            last = None
            for _ in range(it):
                Q = (T[:3, :3] @ P.T).T + T[:3, 3]
                d, j = tree.query(Q, k=1, distance_upper_bound=mc); ok = np.isfinite(d)
                if ok.sum() < 50: return T, np.inf
                thr = np.quantile(d[ok], self.trim); sel = ok.copy(); sel[ok] = d[ok] <= thr
                A, B = Q[sel], mp[j[sel]]
                ca, cb = A[:, :2].mean(0), B[:, :2].mean(0); X, Y = A[:, :2] - ca, B[:, :2] - cb
                dy = math.atan2(float((X[:, 0] * Y[:, 1] - X[:, 1] * Y[:, 0]).sum()), float((X * Y).sum()))
                c, s = math.cos(dy), math.sin(dy); R2 = np.array([[c, -s], [s, c]])
                dT = np.eye(4); dT[:2, :2] = R2; dT[:2, 3] = cb - R2 @ ca; T = dT @ T
                rms = float(np.sqrt((d[ok][d[ok] <= thr] ** 2).mean()))
                if last is not None and abs(last - rms) < 1e-4: break
                last = rms
        return T, rms


def reader(topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics)); return r


# ── 사전 지도 + FAST-LIO 기준 궤적(bag 수신 시각) — render_v2.py 와 같은 대응
d = np.load(NPZ); dm = np.load(MAPNPZ); V = float(dm['V']); MAP = (dm['vox'][dm['cnt'] >= 2].astype(np.float64) + 0.5) * V
odom, lid = d['odom'].copy(), d['lid']
# ★ 사본: 기준 궤적(이 녹화의 FAST-LIO)을 시험 지도 좌표로 옮김 (지도 = T_GT · 녹화 FAST-LIO)
_c, _s = math.cos(T_GT[2]), math.sin(T_GT[2]); _x, _y = odom[:, 2].copy(), odom[:, 3].copy()
odom[:, 2], odom[:, 3] = _c * _x - _s * _y + T_GT[0], _s * _x + _c * _y + T_GT[1]
_cz, _sz = math.cos(T_GT[2] / 2), math.sin(T_GT[2] / 2); qx, qy, qz, qw = odom[:, 5].copy(), odom[:, 6].copy(), odom[:, 7].copy(), odom[:, 8].copy()
odom[:, 8], odom[:, 5], odom[:, 6], odom[:, 7] = _cz * qw - _sz * qz, _cz * qx - _sz * qy, _cz * qy + _sz * qx, _cz * qz + _sz * qw
_P = MAP; _lo, _hi = odom[:, 2:4].min(0) - 4, odom[:, 2:4].max(0) + 4; MAP = _P[np.all((_P[:, :2] > _lo) & (_P[:, :2] < _hi), 1)]
hdr2bag = {}
r = reader(['/lidar_points'])
while r.has_next():
    tp, raw, t = r.read_next()
    s, ns = np.frombuffer(raw[4:8], '<i4')[0], np.frombuffer(raw[8:12], '<u4')[0]
    hdr2bag[round(float(s) + float(ns) * 1e-9, 4)] = t * 1e-9
pairs = np.array([(w, hdr2bag[round(h, 4)]) for w, h in lid if round(h, 4) in hdr2bag])
Kp = float(np.median(pairs[:, 0] - pairs[:, 1])); wl = np.sort(lid[:, 0]); h_o = odom[:, 1]
dmed = float(np.median(h_o - wl[np.clip(np.searchsorted(wl, h_o) - 1, 0, len(wl) - 1)]))
tL = h_o - dmed - Kp; o = np.argsort(tL); tL = tL[o]; GT = odom[o]          # GT: 2:5 위치, 5:9 사원수

# ── 다리 오도메트리 (bag 수신 시각)
LO = []
r = reader(['/utlidar/robot_odom'])
while r.has_next():
    tp, raw, t = r.read_next(); m = deserialize_message(raw, Odometry); p, q = m.pose.pose.position, m.pose.pose.orientation
    LO.append((t * 1e-9, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
LO = np.array(LO)
def leg_at(ts):
    i = min(max(bisect.bisect_left(LO[:, 0], ts), 1), len(LO) - 1); return LO[i]

M = Matcher(voxel(MAP, 0.05))
gt_at = lambda ts: GT[min(max(bisect.bisect_left(tL, ts), 0), len(tL) - 1)]

# ── 시작: FAST-LIO 기준 첫 자세에 맞춰 둔다 (대회에선 출발 자세 = 원점)
import os, time
METHODS = os.environ.get('METHODS', 'leg,icp_flat,icp_level').split(',')
GATE = os.environ.get('GATE', '0') == '1'; RMS_OK = float(os.environ.get('RMS_OK', '0.08'))
CAP_M = float(os.environ.get('CAP_M', '0.10')); CAP_DEG = float(os.environ.get('CAP_DEG', '2.0'))
results = {k: [] for k in METHODS}
icp_ms = []
state = {}
r = reader(['/lidar_points']); last = -1e9; nfr = 0
while r.has_next():
    tp, raw, t = r.read_next(); t *= 1e-9
    if t < tL[0] + 1 or t > tL[-1] or t - last < 1.0 / HZ: continue
    last = t; m = deserialize_message(raw, PointCloud2)
    a = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)
    B = a @ RX.T + TX; rr = np.hypot(B[:, 0], B[:, 1]); B = B[(rr > 0.7) & (rr < 12.0)]
    if len(B) > 1500: B = B[np.random.default_rng(nfr).choice(len(B), 1500, replace=False)]
    L = leg_at(t); RL = q2R(*L[4:8]); lr, lp, ly = rpy(RL)
    g = gt_at(t); Rg = q2R(*g[5:9]); gyaw = rpy(Rg)[2]
    if not state:   # 초기화: 지도 좌표의 자세 = FAST-LIO 자세, 다리 오도는 증분만
        for k in results: state[k] = dict(x=g[2], y=g[3], z=g[4], yaw=gyaw, L=L)
    for k in results:
        S = state[k]; Lp = S['L']; dyaw = rpy(RL)[2] - rpy(q2R(*Lp[4:8]))[2]
        # 다리 증분을 이전 다리 자세 기준(몸체)으로 바꿔 지도 좌표에 붙인다
        Rp = rpy(q2R(*Lp[4:8]))[2]; dxw, dyw = L[1] - Lp[1], L[2] - Lp[2]
        bx, by = math.cos(-Rp) * dxw - math.sin(-Rp) * dyw, math.sin(-Rp) * dxw + math.cos(-Rp) * dyw
        x = S['x'] + math.cos(S['yaw']) * bx - math.sin(S['yaw']) * by
        y = S['y'] + math.sin(S['yaw']) * bx + math.cos(S['yaw']) * by
        yaw = S['yaw'] + dyaw; z = S['z'] + (L[3] - Lp[3]); rms = np.nan
        if k != 'leg':
            if k == 'icp_level':     # roll·pitch 로 수평 세우기 (yaw 는 정합이 푼다)
                cr, sr, cp, sp = math.cos(lr), math.sin(lr), math.cos(lp), math.sin(lp)
                Rrp = np.array([[cp, sp * sr, sp * cr], [0, cr, -sr], [-sp, cp * sr, cp * cr]]); P = B @ Rrp.T
            else:
                P = B
            T0 = np.eye(4); c, s = math.cos(yaw), math.sin(yaw); T0[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]; T0[:3, 3] = [x, y, z]
            _t = time.perf_counter(); T, rms = M.align(P, T0); icp_ms.append((time.perf_counter() - _t) * 1000)
            nyaw = math.atan2(T[1, 0], T[0, 0]); dpos = math.hypot(T[0, 3] - x, T[1, 3] - y)
            dang = abs((nyaw - yaw + math.pi) % (2 * math.pi) - math.pi)
            if GATE:   # 한 번에 고치는 양 제한 + 잔차 나쁜 정합 버림 (정합 사이 다리 표류는 mm 수준)
                if np.isfinite(rms) and rms <= RMS_OK:
                    f = min(1.0, CAP_M / max(dpos, 1e-9)); g_ = min(1.0, math.radians(CAP_DEG) / max(dang, 1e-9))
                    x, y = x + (T[0, 3] - x) * f, y + (T[1, 3] - y) * f
                    yaw = yaw + ((nyaw - yaw + math.pi) % (2 * math.pi) - math.pi) * g_
            elif np.isfinite(rms) and dpos < 1.0:
                x, y, yaw = T[0, 3], T[1, 3], nyaw
        S.update(x=x, y=y, z=z, yaw=yaw, L=L)
        err = math.hypot(x - g[2], y - g[3]); eyaw = math.degrees((yaw - gyaw + math.pi) % (2 * math.pi) - math.pi)
        results[k].append((t, err, eyaw, rms, math.degrees(lp), g[2], g[3]))
    nfr += 1

out = {}
print(f'정합 {nfr}회 ({HZ} Hz), 기준 = FAST-LIO 궤적, 1회 정합 중앙 {np.median(icp_ms):.1f} ms p95 {np.percentile(icp_ms,95):.1f} ms')
for k, v in results.items():
    v = np.array(v); e = v[:, 1]; pit = np.abs(v[:, 4]) > 5
    out[k] = dict(위치오차_중앙_m=round(float(np.median(e)), 3), p95_m=round(float(np.percentile(e, 95)), 3), 최대_m=round(float(e.max()), 3),
                  끝_m=round(float(e[-1]), 3), 경사구간_p95_m=round(float(np.percentile(e[pit], 95)), 3) if pit.any() else None,
                  yaw오차_p95_deg=round(float(np.percentile(np.abs(v[:, 2]), 95)), 2))
    print(f'{k:10s}', out[k])
np.savez(_os.environ.get('OUT', '/tmp/bk') + '.npz', **{k: np.array(v) for k, v in results.items()})
json.dump(out, open(_os.environ.get('OUT', '/tmp/bk') + '.json', 'w'), ensure_ascii=False, indent=1)
