#!/usr/bin/env python3
"""백업 위치추정 핵심 계산 — ROS 와 분리 (녹화본·단위시험으로 그대로 검증한다).

FAST-LIO 와 독립:
  odom  : 다리 기구학 /utlidar/robot_odom (Unitree 공식). 증분만 쓰고, roll·pitch 는 스캔 수평 세우기에 쓴다
  scan  : XT16 /lidar_points (라이다 드라이버 출력 그대로, FAST-LIO 브리지를 거치지 않음)
  map   : 사전 3D 지도 (FAST-LIO 지도, 출발 자세 = 원점)
  정합  : 수평 세운 스캔 ↔ 지도, x·y·yaw 3자유도 다해상도 ICP (lab_visual/icp3d.py 와 같은 방식)
  z     : 지도 지면 높이(칸별) + 다리 오도메트리 몸높이
11-03 오프라인 검증(10-30 녹화본, 5 Hz): 위치오차 중앙 0.048 / p95 0.077 / 최대 0.13 m, X자 가운데 p95 0.069 m.
"""
import math
import numpy as np
from scipy.spatial import cKDTree

# XT16 → 몸체(IMU) 외부파라미터 — gosu_slam/config/go2_slam_odometry.yaml 과 같은 값
R_LIDAR = np.array([[-0.0001707, -0.9999240, 0.0123287],
                    [0.9999844, -0.0001019, 0.0055807],
                    [-0.0055790, 0.0123295, 0.9999084]])
T_LIDAR = np.array([0.25, 0.0, 0.1588])


def q2R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rpy_of(R):
    return math.atan2(R[2, 1], R[2, 2]), math.asin(-max(-1.0, min(1.0, R[2, 0]))), math.atan2(R[1, 0], R[0, 0])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def voxel(P, v):
    k = np.floor(P / v).astype(np.int64)
    _, i = np.unique(k, axis=0, return_index=True)
    return P[i]


def load_map(path):
    """사전 지도 읽기 — map_collect.py 의 .npz(vox,cnt,V) 또는 PCD(x y z 가 앞 3필드, ascii/binary)."""
    if path.endswith('.npz'):
        d = np.load(path)
        return (d['vox'][d['cnt'] >= 2].astype(np.float64) + 0.5) * float(d['V'])
    with open(path, 'rb') as f:
        h = {}
        while True:
            k, *v = f.readline().decode(errors='ignore').split()
            h[k] = v
            if k == 'DATA':
                break
        n, nf = int(h['POINTS'][0]), len(h['FIELDS'])
        if h['DATA'][0] == 'binary':
            a = np.frombuffer(f.read(n * 4 * nf), np.float32).reshape(n, nf)
        else:
            a = np.loadtxt(f, max_rows=n)
    fx = [h['FIELDS'].index(c) for c in ('x', 'y', 'z')]
    return a[:, fx].astype(np.float64)


class Matcher:
    """3자유도(x, y, yaw) 다해상도 ICP. 잔차 하위 70% 만 써서 지도에 없는 것(사람 등)에 끌리지 않는다."""
    def __init__(self, mp, stages=((0.40, 2.5, 20), (0.20, 1.0, 15), (0.10, 0.4, 12)), trim=0.7, solve_z=False):
        self.solve_z = solve_z    # 11-04 시험: z 를 같이 풀면 악화(대응점 대부분이 벽). 기본 끔 — 플래너는 점-로봇 높이차만 보므로 절대 z 오차는 무관
        self.levels = []
        for vx, mc, it in stages:
            p = voxel(mp, vx)
            self.levels.append((cKDTree(p), p, mc, it))
        self.trim = trim

    def align(self, P, T0):
        T = T0.copy(); rms = np.inf
        for tree, mp, mc, it in self.levels:
            last = None
            for _ in range(it):
                Q = (T[:3, :3] @ P.T).T + T[:3, 3]
                d, j = tree.query(Q, k=1, distance_upper_bound=mc)
                ok = np.isfinite(d)
                if ok.sum() < 50:
                    return T, np.inf
                thr = np.quantile(d[ok], self.trim)
                sel = ok.copy(); sel[ok] = d[ok] <= thr
                A, B = Q[sel], mp[j[sel]]
                ca, cb = A[:, :2].mean(0), B[:, :2].mean(0)
                X, Y = A[:, :2] - ca, B[:, :2] - cb
                dy = math.atan2(float((X[:, 0] * Y[:, 1] - X[:, 1] * Y[:, 0]).sum()), float((X * Y).sum()))
                c, s = math.cos(dy), math.sin(dy); R2 = np.array([[c, -s], [s, c]])
                dT = np.eye(4); dT[:2, :2] = R2; dT[:2, 3] = cb - R2 @ ca
                if self.solve_z:
                    dT[2, 3] = float(np.median(B[:, 2] - A[:, 2]))
                T = dT @ T
                rms = float(np.sqrt((d[ok][d[ok] <= thr] ** 2).mean()))
                if last is not None and abs(last - rms) < 1e-4:
                    break
                last = rms
        return T, rms


class GroundGrid:
    """지도에서 칸(10 cm)별 지면 높이 = 그 칸 점들의 하위 10% 높이. 빈 칸은 가장 가까운 칸 값."""
    def __init__(self, mp, cell=0.10):
        self.c = cell
        k = np.floor(mp[:, :2] / cell).astype(np.int64)
        o = np.lexsort((mp[:, 2], k[:, 1], k[:, 0])); ks, zs = k[o], mp[o, 2]
        brk = np.r_[True, np.any(ks[1:] != ks[:-1], axis=1)]
        starts = np.where(brk)[0]; ends = np.r_[starts[1:], len(ks)]
        self.keys = ks[starts]
        self.z = np.array([zs[a + int(0.1 * (b - a))] for a, b in zip(starts, ends)])
        self.tree = cKDTree(self.keys.astype(np.float64))

    def at(self, x, y):
        _, i = self.tree.query([x / self.c, y / self.c])
        return float(self.z[i])


class BackupEstimator:
    """상태: 지도 좌표의 x, y, z, yaw. 다리 오도 증분으로 예측하고 5 Hz 정합으로 고친다."""
    def __init__(self, map_pts, rms_ok=0.08, cap_m=0.10, cap_deg=2.0, max_pts=1500, r_min=0.7, r_max=12.0,
                 degraded_s=10.0, lost_s=30.0):
        mp = voxel(map_pts, 0.05)
        self.matcher = Matcher(mp)
        self.ground = GroundGrid(mp)
        self.rms_ok, self.cap_m, self.cap_rad = rms_ok, cap_m, math.radians(cap_deg)
        self.max_pts, self.r_min, self.r_max = max_pts, r_min, r_max
        self.degraded_s, self.lost_s = degraded_s, lost_s
        self.rng = np.random.default_rng(0)
        self.inited = False
        self.x = self.y = self.z = self.yaw = 0.0
        self.z_off = 0.0
        self.leg_last = None          # (x, y, z, yaw) 다리 오도 직전값
        self.roll = self.pitch = 0.0
        self.t_good = None
        self.n_ok = self.n_bad = 0
        self.last_rms = float('nan')

    # ── 초기화: 지도 좌표의 출발 자세 (보통 원점 = FAST-LIO camera_init)
    def init(self, x, y, z, yaw, leg_pose, t):
        self.x, self.y, self.yaw = x, y, yaw
        lx, ly, lz, lyaw, lr, lp = leg_pose
        self.leg_last = (lx, ly, lz, lyaw)
        self.roll, self.pitch = lr, lp
        self.z_off = z - (self.ground.at(x, y) + lz)
        self.z = z
        self.t_good = t
        self.inited = True

    # ── 다리 오도 증분 (150 Hz)
    def on_leg(self, leg_pose):
        lx, ly, lz, lyaw, lr, lp = leg_pose
        self.roll, self.pitch = lr, lp
        if not self.inited:
            self.leg_last = (lx, ly, lz, lyaw); return
        px, py, pz, pyaw = self.leg_last
        dxw, dyw = lx - px, ly - py
        bx = math.cos(-pyaw) * dxw - math.sin(-pyaw) * dyw      # 이전 다리 자세 기준 몸체 증분
        by = math.sin(-pyaw) * dxw + math.cos(-pyaw) * dyw
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += c * bx - s * by
        self.y += s * bx + c * by
        self.yaw = wrap(self.yaw + wrap(lyaw - pyaw))
        self.z = self.ground.at(self.x, self.y) + lz + self.z_off
        self.leg_last = (lx, ly, lz, lyaw)

    # ── 스캔 정합 (5 Hz). pts_lidar: XT16 좌표 N×3
    #    노드에선 prepare(잠금 안) → matcher.align(잠금 밖, 16~40 ms) → apply(잠금 안) 로 나눠,
    #    정합 도는 동안 들어온 다리 증분을 잃지 않는다(보정은 씨앗 대비 '차이'만 더한다).
    def prepare(self, pts_lidar):
        if not self.inited:
            return None
        B = pts_lidar @ R_LIDAR.T + T_LIDAR
        rr = np.hypot(B[:, 0], B[:, 1]); B = B[(rr > self.r_min) & (rr < self.r_max)]
        if len(B) < 200:
            return None
        cr, sr, cp, sp = math.cos(self.roll), math.sin(self.roll), math.cos(self.pitch), math.sin(self.pitch)
        Rrp = np.array([[cp, sp * sr, sp * cr], [0, cr, -sr], [-sp, cp * sr, cp * cr]])   # 수평 세우기
        Pall = B @ Rrp.T
        P = Pall if len(Pall) <= self.max_pts else Pall[self.rng.choice(len(Pall), self.max_pts, replace=False)]
        T0 = np.eye(4); c, s = math.cos(self.yaw), math.sin(self.yaw)
        T0[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]; T0[:3, 3] = [self.x, self.y, self.z]
        return dict(P=P, Pall=Pall, T0=T0, seed=(self.x, self.y, self.yaw))

    def apply(self, prep, T, rms, t):
        self.last_rms = rms
        sx, sy, syaw = prep['seed']
        accepted = False
        if np.isfinite(rms) and rms <= self.rms_ok:
            nyaw = math.atan2(T[1, 0], T[0, 0])
            ddx, ddy, ddyaw = T[0, 3] - sx, T[1, 3] - sy, wrap(nyaw - syaw)      # 씨앗 대비 보정량
            dpos = math.hypot(ddx, ddy)
            f = min(1.0, self.cap_m / max(dpos, 1e-9)); g = min(1.0, self.cap_rad / max(abs(ddyaw), 1e-9))
            self.x += ddx * f; self.y += ddy * f; self.yaw = wrap(self.yaw + ddyaw * g)
            if self.matcher.solve_z:
                dz = max(-self.cap_m, min(self.cap_m, T[2, 3] - prep['T0'][2, 3]))
                self.z += dz; self.z_off += dz
            self.t_good = t; self.n_ok += 1; accepted = True
        else:
            self.n_bad += 1
        cw, sw = math.cos(self.yaw), math.sin(self.yaw)
        Rw = np.array([[cw, -sw, 0], [sw, cw, 0], [0, 0, 1]])
        world = prep['Pall'] @ Rw.T + np.array([self.x, self.y, self.z])     # 플래너 장애물용 (전 점)
        return dict(rms=rms, accepted=accepted, world=world)

    def on_scan(self, pts_lidar, t):
        prep = self.prepare(pts_lidar)
        if prep is None:
            return None
        T, rms = self.matcher.align(prep['P'], prep['T0'])
        return self.apply(prep, T, rms, t)

    def health(self, t):
        if not self.inited:
            return 'init'
        dt = t - self.t_good
        return 'ok' if dt < self.degraded_s else ('degraded' if dt < self.lost_s else 'lost')

    def pose(self):
        return self.x, self.y, self.z, self.roll, self.pitch, self.yaw


class MuxLogic:
    """FAST-LIO 이상 감지 → 백업으로 전환. 한 번 전환하면 유지(경기 중 오락가락 금지). 수동 복귀만."""
    def __init__(self, timeout_s=0.5, max_speed=2.0, div_m=0.5, div_hold_s=1.0, warmup_s=5.0):
        self.timeout_s, self.max_speed, self.div_m, self.div_hold_s, self.warmup_s = timeout_s, max_speed, div_m, div_hold_s, warmup_s
        self.mode = 'fastlio'; self.reason = ''
        self.t_fl = None; self.fl_prev = None; self.div_since = None; self.t0 = None

    def on_fastlio(self, t, x, y, z, ts=None):
        # t: 받은 시각(무소식 판정용), ts: 메시지에 찍힌 시각(속도 계산용 — 받은 시각은 몰려 들어오면 dt 가 0 에 가까워 속도가 튄다)
        if self.t0 is None: self.t0 = t
        ts = t if ts is None else ts
        if self.fl_prev is not None and self.mode == 'fastlio':
            pt, px, py, pz = self.fl_prev; dt = ts - pt
            if dt > 1e-3 and math.hypot(x - px, y - py) / dt > self.max_speed and t - self.t0 > self.warmup_s:
                self._switch(f'FAST-LIO 속도 {math.hypot(x - px, y - py) / dt:.1f} m/s')
        self.fl_prev = (ts, x, y, z); self.t_fl = t

    def tick(self, t, backup_xy, backup_health):
        if self.mode != 'fastlio' or self.t0 is None or t - self.t0 < self.warmup_s:
            return self.mode
        if self.t_fl is not None and t - self.t_fl > self.timeout_s:
            self._switch(f'FAST-LIO {t - self.t_fl:.1f}초 무소식')
        elif backup_health == 'ok' and backup_xy is not None and self.fl_prev is not None:
            d = math.hypot(self.fl_prev[1] - backup_xy[0], self.fl_prev[2] - backup_xy[1])
            if d > self.div_m:
                self.div_since = self.div_since or t
                if t - self.div_since >= self.div_hold_s:
                    self._switch(f'FAST-LIO 와 백업 차이 {d:.2f} m 가 {self.div_hold_s:.0f}초 지속')
            else:
                self.div_since = None
        return self.mode

    def _switch(self, why):
        if self.mode == 'fastlio':
            self.mode, self.reason = 'backup', why

    def reset(self):
        self.mode, self.reason, self.div_since = 'fastlio', '', None
