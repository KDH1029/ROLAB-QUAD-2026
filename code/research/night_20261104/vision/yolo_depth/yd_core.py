"""YOLO 박스 + D435i 깊이 → 3D. 오프라인 평가(yd_3d.py)와 실행 노드(cone_fusion_node.py)가 같이 쓰는 순수 함수들.

ROS 의존 없음 (numpy 만). 좌표 약속:
  깊이 광학(z 앞, x 오른, y 아래) --T_BODY_DOPT--> 몸체(FAST-LIO body, x 앞 y 왼 z 위) --자세--> 세계(camera_init)
  깊이 광학 --T_COPT_DOPT--> 컬러 광학 --K_COLOR--> 컬러 픽셀
"""
import json, math, collections, bisect
import numpy as np

CALIB_JSON = '/home/ilovehan9283/ws/night_20261104/vision/calib_depth_all.json'
K_DEPTH = np.array([[431.08444214, 0, 422.2805481], [0, 431.08444214, 240.88748169], [0, 0, 1]])
K_COLOR = np.array([[914.03436279, 0, 649.3460083], [0, 915.20739746, 380.72357178], [0, 0, 1]])   # /camera/color/camera_info (10-30)
Q_OPT = [-0.5, 0.5, -0.5, 0.5]                      # camera_X_frame → camera_X_optical_frame (xyzw)
T_LINK_COLOR = ([-0.001, 0.015, 0.001], [0.002, 0.005, 0.001, 1.0])   # tf_static camera_link → camera_color_frame
NAMES = ['tl_red', 'tl_green', 'pedestrian', 'temi', 'unknown', 'cone']
CONE = 5


def q2R(q):
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def Tmat(R, t):
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t; return T


def T_copt_dopt(link_color=T_LINK_COLOR):
    """깊이 광학 → 컬러 광학. camera_depth_frame = camera_link (항등)."""
    T_l_dopt = Tmat(q2R(Q_OPT), [0, 0, 0])
    T_l_copt = Tmat(q2R(link_color[1]), link_color[0]) @ Tmat(q2R(Q_OPT), [0, 0, 0])
    return np.linalg.inv(T_l_copt) @ T_l_dopt


def load_calib(path=CALIB_JSON):
    c = json.load(open(path))
    return np.array(c['T_body_cam_optical'], float), c['stamp_to_nuc'], float(c.get('시각지연_s', 0.0))


def stamp_to_nuc(s, st):
    """녹화본 고정 보정(오프라인 전용): NUC = s + off + slope·(s + off − t_ref)."""
    return s + st['offset'] + st['slope'] * (s + st['offset'] - st['t_ref'])


# ── 깊이 → 점 ────────────────────────────────────────────────────────────
_grid_cache = {}


def depth_points(D, stride=2, zmin=0.2, zmax=6.0, scale=1e-3, K=K_DEPTH):
    """16UC1 깊이(mm) → 깊이 광학 좌표 점 (N,3)."""
    key = (D.shape, stride)
    if key not in _grid_cache:
        vv, uu = np.mgrid[0:D.shape[0]:stride, 0:D.shape[1]:stride]
        _grid_cache[key] = (uu.ravel().astype(np.float32), vv.ravel().astype(np.float32))
    uu, vv = _grid_cache[key]
    z = D[::stride, ::stride].ravel().astype(np.float32) * scale
    ok = (z > zmin) & (z < zmax)
    z = z[ok]
    return np.c_[(uu[ok] - K[0, 2]) * z / K[0, 0], (vv[ok] - K[1, 2]) * z / K[1, 1], z]


def project(P, K=K_COLOR):
    """광학 좌표 점 → 픽셀 (u, v), 앞(z>0)만 의미 있음."""
    z = np.maximum(P[:, 2], 1e-6)
    return K[0, 0] * P[:, 0] / z + K[0, 2], K[1, 1] * P[:, 1] / z + K[1, 2]


def shrink_box(b, f=0.6):
    x1, y1, x2, y2 = b; cx, cy = (x1 + x2) / 2, (y1 + y2) / 2; hw, hh = (x2 - x1) * f / 2, (y2 - y1) * f / 2
    return cx - hw, cy - hh, cx + hw, cy + hh


def box_3d(box, uc, vc, Pd, shrink=0.6, min_pts=8):
    """박스(컬러 픽셀, 가운데 shrink 로 줄임) 안에 투영된 깊이 점들의 성분별 중앙값 (깊이 광학 좌표).
    돌려줌: (p3[3] 또는 None, 점 개수, 중앙 깊이 ±10 cm 안 비율, 전체 박스 점 인덱스)"""
    x1, y1, x2, y2 = shrink_box(box, shrink)
    m = (uc >= x1) & (uc <= x2) & (vc >= y1) & (vc <= y2)
    n = int(m.sum())
    X1, Y1, X2, Y2 = box
    full = np.where((uc >= X1) & (uc <= X2) & (vc >= Y1) & (vc <= Y2))[0]
    if n < min_pts:
        return None, n, float('nan'), full
    Q = Pd[m]; p = np.median(Q, axis=0)
    cons = float((np.abs(Q[:, 2] - p[2]) < 0.10).mean())
    return p, n, cons, full


def ground_z_from_box(Pw_full, pct=5.0, min_pts=8):
    """박스 전체(줄이지 않은) 깊이 점의 세계 z 하위 pct% — 콘 밑면/바닥 높이 추정."""
    if len(Pw_full) < min_pts:
        return None
    return float(np.percentile(Pw_full[:, 2], pct))


def cone_shape(Qb, pb, near_r=0.4):
    """박스 전체 깊이점(몸체 좌표) 중 검출 3D 점 수평 near_r 안 → (높이 z95−z5, 가로폭 y p10~p90). 점 20개 미만이면 None."""
    if len(Qb) < 20:
        return None, None
    near = np.hypot(Qb[:, 0] - pb[0], Qb[:, 1] - pb[1]) < near_r
    Q = Qb[near] if near.sum() >= 20 else Qb
    z5, z95 = np.percentile(Q[:, 2], [5, 95]); y10, y90 = np.percentile(Q[:, 1], [10, 90])
    return float(z95 - z5), float(y90 - y10)


def shape_ok(h, w, hmin=0.12, hmax=0.35, wmax=0.30):
    """라바콘(23 cm, 밑면 14 cm) 모양 검사 — 실제 콘으로 재현율 검증 안 됨 (10-30 녹화본엔 콘 없음)."""
    return h is not None and hmin <= h <= hmax and w <= wmax


def cone_column(xy, z0, r=0.07, h=0.25, dxy=0.035, dz=0.05):
    """콘 자리 세로 점기둥: 반경 r 원판 격자 × 높이 0~h (지면 z0 기준). (N,3)"""
    g = np.arange(-r, r + 1e-9, dxy)
    gx, gy = np.meshgrid(g, g); k = gx ** 2 + gy ** 2 <= r * r + 1e-9
    disk = np.c_[gx[k], gy[k]]
    zs = np.arange(0.0, h + 1e-9, dz)
    pts = [np.c_[disk + np.asarray(xy[:2]), np.full(len(disk), z0 + z)] for z in zs]
    return np.vstack(pts)


# ── 시각 / 자세 ────────────────────────────────────────────────────────────
class ClockOffset:
    """카메라 stamp(로봇 PC 시계) → NUC 시계. offset = 최근 win 초 (수신 − stamp) 최솟값 (하한 포락선)."""

    def __init__(self, win=10.0):
        self.win = win; self.q = collections.deque()          # (수신, 수신−stamp), 단조 최소 덱

    def add(self, recv, stamp):
        lag = recv - stamp
        while self.q and self.q[-1][1] >= lag:
            self.q.pop()
        self.q.append((recv, lag))
        while self.q and self.q[0][0] < recv - self.win:
            self.q.popleft()

    def offset(self):
        return self.q[0][1] if self.q else None

    def to_nuc(self, stamp):
        o = self.offset()
        return None if o is None else stamp + o


def _slerp(q0, q1, a):
    q0 = np.asarray(q0, float); q1 = np.asarray(q1, float)
    d = float(np.dot(q0, q1))
    if d < 0: q1 = -q1; d = -d
    if d > 0.9995:
        q = q0 + a * (q1 - q0); return q / np.linalg.norm(q)
    th = math.acos(d); s = math.sin(th)
    return (math.sin((1 - a) * th) * q0 + math.sin(a * th) * q1) / s


class PoseBuffer:
    """(t, p, q[xyzw]) 링버퍼, 선형+slerp 보간. 범위 밖이면 max_extrap 초까지만 끝값 사용."""

    def __init__(self, maxlen=400):
        self.t = collections.deque(maxlen=maxlen); self.p = collections.deque(maxlen=maxlen); self.q = collections.deque(maxlen=maxlen)

    def add(self, t, p, q):
        if self.t and t <= self.t[-1]:
            return
        self.t.append(t); self.p.append(np.asarray(p, float)); self.q.append(np.asarray(q, float))

    def at(self, ts, max_extrap=0.1):
        if not self.t:
            return None
        tl = self.t
        if ts <= tl[0]:
            return (q2R(self.q[0]), self.p[0]) if tl[0] - ts <= max_extrap else None
        if ts >= tl[-1]:
            return (q2R(self.q[-1]), self.p[-1]) if ts - tl[-1] <= max_extrap else None
        i = bisect.bisect_left(list(tl), ts)
        a = (ts - tl[i - 1]) / max(tl[i] - tl[i - 1], 1e-9)
        return q2R(_slerp(self.q[i - 1], self.q[i], a)), self.p[i - 1] * (1 - a) + self.p[i] * a


# ── 콘 기억 ────────────────────────────────────────────────────────────────
class ConeMemory:
    """세계 좌표 콘 기억: merge_r 안이면 같은 콘(평균 갱신), hits ≥ min_hits 이고 keep_s 안에 본 것만 내보냄."""

    def __init__(self, merge_r=0.3, min_hits=2, keep_s=30.0):
        self.merge_r, self.min_hits, self.keep_s = merge_r, min_hits, keep_s
        self.c = []          # dict(id, xy, z0, hits, last, conf)
        self.nid = 0

    def add(self, xy, z0, t, conf):
        best, bd = None, self.merge_r
        for c in self.c:
            d = math.hypot(c['xy'][0] - xy[0], c['xy'][1] - xy[1])
            if d < bd: best, bd = c, d
        if best is None:
            self.c.append(dict(id=self.nid, xy=np.array(xy[:2], float), z0=z0, hits=1, last=t, conf=conf)); self.nid += 1; return
        w = 1.0 / (best['hits'] + 1)
        best['xy'] = best['xy'] * (1 - w) + np.asarray(xy[:2]) * w
        best['z0'] = best['z0'] * (1 - w) + z0 * w
        best['hits'] += 1; best['last'] = t; best['conf'] = max(best['conf'], conf)

    def active(self, t):
        self.c = [c for c in self.c if t - c['last'] <= self.keep_s]
        return [c for c in self.c if c['hits'] >= self.min_hits]


def detect_to_world(dets, D, T_body_dopt, Rwb, pwb, T_cd, shrink=0.6, stride=2, min_pts=8, Kc=K_COLOR):
    """한 프레임: dets[(cls, conf, x1,y1,x2,y2)] + 깊이 영상 + 몸체 자세 → 검출별 dict 목록."""
    Pd = depth_points(D, stride)
    Pc = Pd @ T_cd[:3, :3].T + T_cd[:3, 3]
    uc, vc = project(Pc, Kc)
    front = Pc[:, 2] > 0.05
    uc = np.where(front, uc, -1e9)
    Rbd, tbd = T_body_dopt[:3, :3], T_body_dopt[:3, 3]
    out = []
    for d in dets:
        cls, conf = int(d[0]), float(d[1]); box = d[2:6]
        p, n, cons, full = box_3d(box, uc, vc, Pd, shrink, min_pts)
        r = dict(cls=cls, name=NAMES[cls] if cls < len(NAMES) else str(cls), conf=conf, box=[float(x) for x in box], n_pts=n, consist=(None if cons != cons else cons))
        if p is None:
            r.update(ok=False); out.append(r); continue
        pb = Rbd @ p + tbd
        pw = Rwb @ pb + pwb
        Qb_full = Pd[full] @ Rbd.T + tbd
        Pw_full = Qb_full @ Rwb.T + pwb
        gz = ground_z_from_box(Pw_full)
        sh, sw = cone_shape(Qb_full, pb)
        r.update(ok=True, p_opt=p.tolist(), p_body=pb.tolist(), p_world=pw.tolist(), range=float(np.linalg.norm(pb[:2])),
                 ground_z=gz, n_full=int(len(full)), shape_h=sh, shape_w=sw, shape_ok=shape_ok(sh, sw))
        out.append(r)
    return out
