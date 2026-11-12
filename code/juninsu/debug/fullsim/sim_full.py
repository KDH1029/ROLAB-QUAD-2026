#!/usr/bin/env python3
"""코스 전체 시간 시뮬레이션 — 실제 러너(경로 전용)를 rec1650 경로 전 구간(출발 → 정지선1 → 슬라럼 → 정지선2 → 주차 → 출구)으로 운동 모델에서 돌린다.
모델(실측 근거):
  걸음새 클래식: 지연 회전 τ0.18·전진 τ0.35 s(16:45 녹화), 전진 문턱 0.06 m/s, 회전 문턱 0.10 rad/s, 실제 속도 상한 0.33 m/s(16:45 p99)
  속도 이득 GAIN_V: 18:06 실주행 첫 구간(출발→정지선1 4.45 m, 27.5 s)에 맞춤 — --calib 로 구함
  주차 뒤 ClassicWalk 보내기 전(기본 걸음새): 회전 43 %만(18:06 실주행 9 s 에 77°)
  앉기: StandDown 0.3 s 뒤 1.2 s 동안 0.32→0.06 m · 일어나기: StandUp 0.3 s 뒤 1.5 s 동안 0.32 m
  신호: 카메라 켜지면 초록 · 재정렬 응답 1.0 s · 장애물(앞 막힘 판정용 스캔) = rec1650 편집기 벽, 슬라럼만 rec1645(관측≥8)
사용: python3 sim_full.py [--calib] [변형이름 ...]   변형은 아래 VARIANTS"""
import json, math, os, sys, tempfile
import numpy as np
H = os.path.dirname(os.path.abspath(__file__)); J = os.path.dirname(os.path.dirname(H))
os.environ['RUNNER_PATH_ONLY'] = '1'; os.environ['RUNNER_TL'] = '1'
sys.path.insert(0, os.path.join(J, 'src')); sys.path.insert(0, os.path.join(J, 'tools'))
import rclpy  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from scipy.spatial import cKDTree  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from unitree_go.msg import SportModeState  # noqa: E402
import real_course_runner as rcr  # noqa: E402
sys.argv, _argv = ['x', 'rec1650'], sys.argv
import route_editor as ed  # noqa: E402
sys.argv = _argv
RJ = json.load(open(os.path.join(J, 'routes', 'rec1650', 'route.json'))); R = RJ['route']
OB = ed.real_walls(ed.load_cache()); SL = np.array([[e['x'], e['y']] for e in R[6:25]]); zl, zh = SL.min(0) - 0.4, SL.max(0) + 0.4
OB = np.r_[OB[~np.all((OB > zl) & (OB < zh), 1)], np.load(os.path.join(J, 'debug', 'cones1645', 'zone_1645_c8.npy'))]; TO = cKDTree(OB)
DEFAULTS = {k: getattr(rcr, k) for k in ('PIN_V', 'PATH_V_MIN', 'PATH_SLOW_K', 'ROTATE_RATE_MAX', 'ROTATE_RATE_MIN', 'ROTATE_SETTLE_S', 'POST_STAND_S',
                                         'PARK_PRE_S', 'FOLLOW_WZ', 'PIN_LOOKAHEAD', 'STOPLINE_HOLD_S', 'GREEN_CONFIRM_SECONDS', 'REGAIT_AT_S', 'RISE_MIN_S', 'PATH_STOP_TOL', 'FRONT_STOP_MAX_S', 'FRONT_STOP_X')}
class Clock:
    t = 1000.0
    def monotonic(self): return self.t
def run(var, gain_v, seed=None, stop_at=None, regait=True, verbose=False, fallen=None):
    clock = Clock(); rcr.time.monotonic = clock.monotonic
    for k, v in DEFAULTS.items(): setattr(rcr, k, v)
    for k, v in var.items(): setattr(rcr, k, v)
    rng = np.random.default_rng(seed) if seed is not None else None
    f = tempfile.NamedTemporaryFile('w', suffix='.json', delete=False); json.dump(RJ, f); f.close()
    rcr.WAYPOINTS_PATH = f.name; rcr.ALIGN_T = (0.0, 0.0, 0.0); s0, goals = rcr.load_waypoints(); os.unlink(f.name)
    n = rcr.RealCourseRunner(s0, goals); cmd = [0.0, 0.0, 0.0]; st = dict(h=0.32, h_from=0.32, h_to=0.32, h_t0=0.0, h_dur=1.0, classic=False, cam=False)
    def req(m):
        a = m.header.identity.api_id; t = clock.t
        if a == rcr.ROBOT_SPORT_API_ID_CLASSICWALK:
            if regait or n.state not in ('PARK_SETTLE', 'PARK_REALIGN', 'ROTATE'): st['classic'] = True   # 옛 코드: 주차 뒤엔 이동(TO_GOAL) 때 처음 보냄
        elif a == rcr.STANDDOWN_ID: st.update(h_from=st['h'], h_to=0.06, h_t0=t + 0.3, h_dur=1.2, classic=False)
        elif a == rcr.STANDUP_ID: st.update(h_from=st['h'], h_to=0.32, h_t0=t + 0.3, h_dur=1.5)
    n.pub_cmd_vel.publish = lambda m: cmd.__setitem__(slice(0, 3), [m.linear.x, m.linear.y, m.angular.z])
    n.pub_goal.publish = lambda m: None; n.pub_req.publish = req; n.pub_camera_active.publish = lambda m: st.__setitem__('cam', m.data)
    n.pub_realign.get_subscription_count = lambda: 1; realign = {'t': None}
    _pr = n.pub_realign.publish; n.pub_realign.publish = lambda m: realign.__setitem__('t', clock.t + 1.0)
    x, y = R[0]['x'], R[0]['y']; yaw = R[0].get('yaw') or math.atan2(R[1]['y'] - y, R[1]['x'] - x)
    if rng is not None: x += rng.uniform(-0.05, 0.05); y += rng.uniform(-0.05, 0.05); yaw += math.radians(rng.uniform(-3, 3))
    tau_v, tau_w = 0.35, 0.18; gv = gain_v * (rng.uniform(0.9, 1.1) if rng is not None else 1.0)
    vel = np.zeros(3); dt = 0.05; log = []; t_start = clock.t; prev = None; seg = {}
    def feed():
        m = Odometry(); m.pose.pose.position.x, m.pose.pose.position.y = x, y; m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        n.cb_odom(m); s = SportModeState(); s.velocity = [float(math.hypot(vel[0], vel[1])), 0.0, 0.0]; s.body_height = float(st['h']); n.cb_state(s)
        idx = TO.query_ball_point([x, y], 0.8)
        extra = FAL[np.hypot(*(FAL - [x, y]).T) < 0.8] if fallen is not None else np.zeros((0, 2))
        if idx or len(extra):
            q = np.r_[OB[idx], extra] - [x, y]; c, s_ = math.cos(yaw), math.sin(yaw); n.scan_xy = np.c_[q[:, 0] * c + q[:, 1] * s_, -q[:, 0] * s_ + q[:, 1] * c]
        else:
            n.scan_xy = np.zeros((0, 2))
        n.scan_t = clock.t
        if st['cam']: n.cb_light(String(data='GREEN|0.9'))
        if realign['t'] is not None and clock.t >= realign['t']: realign['t'] = None; n.align_ver += 1
    FAL = (np.array(fallen) + 0.18 * np.c_[np.cos(np.linspace(0, 6.28, 24)), np.sin(np.linspace(0, 6.28, 24))]) if fallen is not None else None   # 넘어진 콘: 지름 0.36 m 원(지도에 없음)
    feed(); n.tick(); n.tick(); steps = 0; cur_lab = None; lab_t = clock.t
    while steps < int(400 / dt):
        steps += 1; clock.t += dt
        a = min(1.0, max(0.0, (clock.t - st['h_t0']) / st['h_dur'])); st['h'] = st['h_from'] + (st['h_to'] - st['h_from']) * a
        if steps % 4 == 0:
            feed(); n.tick()
            lab = n.cur()[0] if n.idx < len(n.goals) else 'END'
            key = n.state; seg[key] = seg.get(key, 0.0) + 0.2
            if lab != cur_lab:
                if cur_lab is not None: log.append((cur_lab, clock.t - lab_t))
                cur_lab, lab_t = lab, clock.t
            if n.state == 'DONE' or (stop_at and lab == stop_at and n.state in ('PATH_ARRIVE', 'ROTATE', 'STOPLINE_HOLD')):
                break
            if n.state in ('EXIT_PUSH',) and n.cur()[0] == 'exit':
                break
        c = np.array(cmd, float)
        if st['h'] < 0.25: c[:] = 0.0                  # 앉아 있거나 일어나는 중엔 안 움직임
        for i, mn in enumerate((0.08, 0.08, 0.15)):
            if 1e-4 < abs(c[i]) < mn: c[i] = mn * np.sign(c[i])
        if not (abs(c[0]) >= 0.06 or abs(c[1]) >= 0.06 or abs(c[2]) >= 0.10): c[:] = 0.0
        if not st['classic']: c[2] *= 0.43               # 기본 걸음새(주차 뒤 ClassicWalk 전): 회전 43 %
        c[:2] *= gv; sp = math.hypot(c[0], c[1])
        if sp > 0.33: c[:2] *= 0.33 / sp
        vel += (c - vel) * (dt / np.array([tau_v, tau_v, tau_w]))
        cy, sy = math.cos(yaw), math.sin(yaw); nx_ = x + (cy * vel[0] - sy * vel[1]) * dt; ny_ = y + (sy * vel[0] + cy * vel[1]) * dt
        if fallen is not None and math.hypot(nx_ - fallen[0], ny_ - fallen[1]) < 0.18 + 0.30 and math.hypot(nx_ - fallen[0], ny_ - fallen[1]) < math.hypot(x - fallen[0], y - fallen[1]):
            nx_, ny_ = x, y; vel[:2] = 0.0                      # 콘에 막혀 못 나감(가까워지는 쪽만)
        x, y = nx_, ny_; yaw += vel[2] * dt
    if cur_lab is not None: log.append((cur_lab, clock.t - lab_t))
    try: n.destroy_node()
    except Exception: pass
    return clock.t - t_start, log, seg, n.state
VARIANTS = {
    'base': ({}, False),                                   # 지금(주차 뒤 걸음새 고치기 전)
    'gait': ({}, True),                                    # 주차 뒤 ClassicWalk 고침
    'v30': ({'PIN_V': 0.30}, True),
    'v33': ({'PIN_V': 0.33}, True),
    'v33_rot': ({'PIN_V': 0.33, 'ROTATE_RATE_MAX': 0.8, 'ROTATE_SETTLE_S': 0.2}, True),
    'v33_rot_park': ({'PIN_V': 0.33, 'ROTATE_RATE_MAX': 0.8, 'ROTATE_SETTLE_S': 0.2, 'PARK_PRE_S': 0.3}, True),
    'fs1': ({'FRONT_STOP_MAX_S': 1.0}, True),
    'v33_nofs': ({'PIN_V': 0.33, 'FRONT_STOP_MAX_S': 0.0}, True),
    'v33_fs1': ({'PIN_V': 0.33, 'FRONT_STOP_MAX_S': 1.0}, True),
    'v30_fs1': ({'PIN_V': 0.30, 'FRONT_STOP_MAX_S': 1.0}, True),
    'v30_fallen': ({'PIN_V': 0.30}, True, (6.70, 1.30)),
    'v33_all': ({'PIN_V': 0.33, 'ROTATE_RATE_MAX': 0.8, 'ROTATE_SETTLE_S': 0.2, 'PARK_PRE_S': 0.3, 'PATH_SLOW_K': 1.5, 'PATH_V_MIN': 0.18}, True),
}
if __name__ == '__main__':
    rclpy.init()
    if '--calib' in sys.argv:
        for g in (1.0, 0.8, 0.7, 0.65, 0.6, 0.55, 0.5):
            T, log, seg, s = run({'PIN_V': 0.25}, g, stop_at='stopline1', regait=False)
            print(f'GAIN_V {g:.2f}: 출발→정지선1 도착 {T:.1f} s (실측 27.5 s)')
        sys.exit()
    G = float(os.environ.get('GAIN_V', '0.65'))
    names = [a for a in sys.argv[1:] if a in VARIANTS] or list(VARIANTS)
    out = {}
    for nm in names:
        var, rg, *fl = VARIANTS[nm]; fl = fl[0] if fl else None; T, log, seg, s = run(var, G, regait=rg, fallen=fl)
        Ts = [run(var, G, seed=k, regait=rg, fallen=fl)[0] for k in range(int(os.environ.get('NSEED', '6')))]
        out[nm] = dict(T=T, Tmean=float(np.mean(Ts)), Tsd=float(np.std(Ts)), state=s, seg={k: round(v, 1) for k, v in sorted(seg.items(), key=lambda kv: -kv[1])}, log=[(a, round(b, 1)) for a, b in log])
        top = ', '.join(f'{k} {v:.1f}' for k, v in list(out[nm]['seg'].items())[:7])
        print(f'{nm:14s} 총 {T:6.1f} s (무작위 {len(Ts)}판 {np.mean(Ts):.1f}±{np.std(Ts):.1f}) 끝 {s} | {top}')
    json.dump(out, open(os.path.join(H, f'sim_full_{int(G * 100)}.json'), 'w'), ensure_ascii=False, indent=1)
