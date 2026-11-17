"""AMCL 감시 판정 (ROS 없음) — junsu 11-05.

출발 ICP(start_align_core)로 정한 T(지도 ← 오늘 FAST-LIO)로 FAST-LIO 위치를 지도에 올리고,
AMCL(같은 지도, FAST-LIO odom) 위치와 비교한다. AMCL 은 주행을 고치지 않는다 — 감시만.

상태
  WARMUP   AMCL 초기화 직후(WARMUP_S) 또는 아직 AMCL 위치 없음         → 판정 안 함
  NO_AMCL  AMCL 위치가 STALE_S 넘게 안 옴                               → 판정 안 함(멈추지 않음: fail-open)
  UNSURE   AMCL 이 스스로 불확실(σxy > SIGMA_MAX)                        → 판정 안 함
  OK       차이 < WARN
  WARN     WARN ≤ 차이 < STOP, 또는 STOP 이상이지만 아직 HOLD_S 안 됨
  STOP     STOP_XY 또는 STOP_YAW 넘는 차이가 HOLD_S 이상 계속            → 러너가 그 자리에 섬

문턱 근거(11-05 오프라인, review amcl3 o_arena4/o_113612/o_145915 = AMCL·FAST-LIO odom, 정상 주행 ~470 s):
  출발 ICP 정렬 뒤 AMCL 과의 차이 최대 0.149 m / 4.78°(p95 0.116 m / 2.78°), AMCL σxy 중앙 0.09 p95 0.19(초기화 직후만 0.47).
  → STOP 0.30 m / 8° 는 정상 최대의 2배·1.7배. 다리 odom AMCL 은 정상에서도 0.315 m / 7.3° 라 쓰지 않음.
"""
import math

WARN_XY = 0.15
WARN_YAW = math.radians(5.0)
STOP_XY = 0.30
STOP_YAW = math.radians(8.0)
HOLD_S = 1.0
WARMUP_S = 3.0
STALE_S = 3.0
SIGMA_MAX = 0.25


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def compose(T, pose):
    """T * pose (2D x, y, yaw)."""
    c, s = math.cos(T[2]), math.sin(T[2])
    return (c * pose[0] - s * pose[1] + T[0], s * pose[0] + c * pose[1] + T[1], wrap(pose[2] + T[2]))


class Watch:
    def __init__(self, T, t_init, warn_xy=WARN_XY, warn_yaw=WARN_YAW, stop_xy=STOP_XY, stop_yaw=STOP_YAW,
                 hold_s=HOLD_S, warmup_s=WARMUP_S, stale_s=STALE_S, sigma_max=SIGMA_MAX):
        self.T = tuple(T)
        self.t_init = t_init
        self.p = dict(warn_xy=warn_xy, warn_yaw=warn_yaw, stop_xy=stop_xy, stop_yaw=stop_yaw,
                      hold_s=hold_s, warmup_s=warmup_s, stale_s=stale_s, sigma_max=sigma_max)
        self.last_amcl_t = None
        self.bad_since = None
        self.latched = False          # STOP 은 한 번 나면 reset() 전까지 유지(오가는 판정으로 섰다 갔다 하지 않게)
        self.e_xy = self.e_yaw = self.sigma = None
        self.state = 'WARMUP'

    def reset(self, T=None, t_init=None):
        if T is not None:
            self.T = tuple(T)
        if t_init is not None:
            self.t_init = t_init
        self.bad_since = None
        self.latched = False
        self.state = 'WARMUP'

    def update(self, t, lio_pose, amcl_pose, sigma_xy):
        """t: 지금(초), lio_pose: 같은 시각 FAST-LIO (x, y, yaw), amcl_pose: AMCL (x, y, yaw), sigma_xy: AMCL √cov_xx."""
        self.last_amcl_t = t
        p = compose(self.T, lio_pose)
        self.e_xy = math.hypot(p[0] - amcl_pose[0], p[1] - amcl_pose[1])
        self.e_yaw = abs(wrap(p[2] - amcl_pose[2]))
        self.sigma = sigma_xy
        return self.judge(t)

    def judge(self, t):
        P = self.p
        if self.latched:
            self.state = 'STOP'
        elif self.last_amcl_t is None or t - self.t_init < P['warmup_s']:
            self.state = 'WARMUP'; self.bad_since = None
        elif t - self.last_amcl_t > P['stale_s']:
            self.state = 'NO_AMCL'; self.bad_since = None
        elif self.sigma is not None and self.sigma > P['sigma_max']:
            self.state = 'UNSURE'; self.bad_since = None
        elif self.e_xy >= P['stop_xy'] or self.e_yaw >= P['stop_yaw']:
            self.bad_since = t if self.bad_since is None else self.bad_since
            if t - self.bad_since >= P['hold_s']:
                self.latched = True
                self.state = 'STOP'
            else:
                self.state = 'WARN'
        else:
            self.bad_since = None
            self.state = 'WARN' if (self.e_xy >= P['warn_xy'] or self.e_yaw >= P['warn_yaw']) else 'OK'
        return self.state

    def status(self):
        r = lambda v, k=3: None if v is None else round(v, k)
        return dict(state=self.state, e_xy=r(self.e_xy), e_yaw_deg=r(None if self.e_yaw is None else math.degrees(self.e_yaw), 2),
                    sigma=r(self.sigma), T=[round(self.T[0], 4), round(self.T[1], 4), round(self.T[2], 5)])
