#!/usr/bin/env python3
"""지금 juninsu 전체 평가 — sim_full 과 같은 러너·지도·클래식 모델에 무작위 조건을 넣어 N판.
무작위: 출발 ±5 cm·±3° · 위치 잡음 σ1.5 cm·0.5° · 지연 τ회전 0.14~0.22, τ전진 0.30~0.40 · 속도 이득 0.85~1.10 · 남은 쏠림 ±0.01 rad/s
        · 정지선마다 신호 상황(바로 초록 / 빨강 3~25 s 뒤 초록 / 아무것도 안 보임 / 빨강 중 0.5 s 끊김 섞임 15 s 뒤 초록)
        · 재정렬 응답 0.5~3 s · 30 % 판: 슬라럼 아닌 곳에서 사람이 몸 앞 0.4 m 를 1.5 s 지나감(지도에 없음)
지표: 완주·총 시간 · 신호 규칙대로 출발했나(기대 시각과 차이) · 정지선·주차 멈춘 자리 오차 · 몸(0.70×0.31) 닿음(구간별) · 회전 타임아웃·점 건너뜀·앞 막힘 횟수
사용: python3 sim_eval.py [N=40]"""
import json, math, os, sys, tempfile
import numpy as np
H = os.path.dirname(os.path.abspath(__file__)); sys.argv, _a = ['x'], sys.argv
exec(open(os.path.join(H, 'sim_full_route.py')).read().split("DEFAULTS = {")[0])   # ★ 사본: SIM_ROUTE 경로     # 러너·지도·장애물(OB, TO)·경로(R, RJ)
sys.argv = _a
STOPS = {e['label']: (e['x'], e['y']) for e in R if e['label'].startswith(('stopline', 'parking'))}
SLZ = (zl, zh)
L2, W2 = 0.35, 0.155
def body_hit(x, y, a):
    idx = TO.query_ball_point([x, y], 0.45)
    if not idx: return False
    q = OB[idx] - [x, y]; lx = q[:, 0] * math.cos(a) + q[:, 1] * math.sin(a); ly = -q[:, 0] * math.sin(a) + q[:, 1] * math.cos(a)
    return bool(np.any((np.abs(lx) < L2) & (np.abs(ly) < W2)))
class Clock:
    t = 1000.0
    def monotonic(self): return self.t
class Log:
    def __init__(s, clk): s.m = []; s.clk = clk
    def warn(s, x, *a, **k): s.m.append((s.clk.t, 'W', x))
    warning = warn
    def info(s, x, *a, **k): s.m.append((s.clk.t, 'I', x))
    def error(s, x, *a, **k): s.m.append((s.clk.t, 'E', x))
    def debug(s, *a, **k): pass
def light_plan(rng):
    k = rng.choice(['green', 'red_then_green', 'none', 'red_flicker'], p=[0.4, 0.3, 0.15, 0.15])
    r = float(rng.uniform(3, 25))
    return k, r
def light_at(plan, el):     # el = 카메라 켠 뒤 경과(정지 2 s 끝부터)
    k, r = plan
    if k == 'green': return 'GREEN|0.9'
    if k == 'none': return 'UNKNOWN|0.0'
    if k == 'red_then_green': return 'RED|0.9' if el < r else 'GREEN|0.9'
    if el < 15.0: return 'UNKNOWN|0.0' if (el % 3.0) < 0.5 else 'RED|0.9'
    return 'GREEN|0.9'
def expected_go(plan):      # 규칙: 초록 1 s 연속 → 출발 / 10 s 지나면 빨강 2 s 안 보일 때
    k, r = plan
    if k == 'green': return 1.0
    if k == 'none': return 10.0
    if k == 'red_then_green': return min(r + 1.0, max(10.0, r + 2.0)) if r + 1.0 > 10.0 else r + 1.0
    return 16.0
def run_eval(seed):
    rng = np.random.default_rng(seed)
    clock = Clock(); rcr.time.monotonic = clock.monotonic
    f = tempfile.NamedTemporaryFile('w', suffix='.json', delete=False); json.dump(RJ, f); f.close()
    rcr.WAYPOINTS_PATH = f.name; rcr.ALIGN_T = (0.0, 0.0, 0.0); s0, goals = rcr.load_waypoints(); os.unlink(f.name)
    n = rcr.RealCourseRunner(s0, goals); lg = Log(clock); n.get_logger = lambda: lg
    cmd = [0.0, 0.0, 0.0]; st = dict(h=0.32, h_from=0.32, h_to=0.32, h_t0=0.0, h_dur=1.0, classic=False, cam=False, cam_t=None)
    def req(m):
        a = m.header.identity.api_id; t = clock.t
        if a == rcr.ROBOT_SPORT_API_ID_CLASSICWALK: st['classic'] = True
        elif a == rcr.STANDDOWN_ID: st.update(h_from=st['h'], h_to=0.06, h_t0=t + 0.3, h_dur=float(rng.uniform(1.0, 1.6)), classic=False)
        elif a == rcr.STANDUP_ID: st.update(h_from=st['h'], h_to=0.32, h_t0=t + 0.3, h_dur=float(rng.uniform(1.2, 2.0)))
    def cam(m):
        st['cam'] = m.data; st['cam_t'] = clock.t if m.data else None
    n.pub_cmd_vel.publish = lambda m: cmd.__setitem__(slice(0, 3), [m.linear.x, m.linear.y, m.angular.z])
    n.pub_goal.publish = lambda m: None; n.pub_req.publish = req; n.pub_camera_active.publish = cam
    n.pub_realign.get_subscription_count = lambda: 1; realign = {'t': None}; rl = float(rng.uniform(0.5, 3.0))
    n.pub_realign.publish = lambda m: realign.__setitem__('t', clock.t + rl)
    x, y = R[0]['x'] + rng.uniform(-0.05, 0.05), R[0]['y'] + rng.uniform(-0.05, 0.05)
    yaw = (R[0].get('yaw') and math.radians(R[0]['yaw'])) or math.atan2(R[1]['y'] - R[0]['y'], R[1]['x'] - R[0]['x']); yaw += math.radians(rng.uniform(-3, 3))
    tau_v, tau_w = rng.uniform(0.30, 0.40), rng.uniform(0.14, 0.22); gv = rng.uniform(0.85, 1.10); wb = rng.uniform(-0.01, 0.01)
    plans = {}; person = None
    if rng.random() < 0.3: person = {'at': float(rng.uniform(15, 60)), 'dur': 1.5, 'done': False}
    vel = np.zeros(3); dt = 0.05; steps = 0; hit = {'slalom': 0.0, 'other': 0.0}; stop_err = {}; go = {}; front_unknown = 0
    def feed():
        nx, ny, na = x + rng.normal(0, 0.015), y + rng.normal(0, 0.015), yaw + math.radians(rng.normal(0, 0.5))
        m = Odometry(); m.pose.pose.position.x, m.pose.pose.position.y = nx, ny; m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(na / 2), math.cos(na / 2)
        n.cb_odom(m); s = SportModeState(); s.velocity = [float(math.hypot(vel[0], vel[1])), 0.0, 0.0]; s.body_height = float(st['h']); n.cb_state(s)
        idx = TO.query_ball_point([x, y], 0.8); c, s_ = math.cos(yaw), math.sin(yaw)
        q = OB[idx] - [x, y] if idx else np.zeros((0, 2)); P = np.c_[q[:, 0] * c + q[:, 1] * s_, -q[:, 0] * s_ + q[:, 1] * c]
        if person and person['at'] <= clock.t - 1000 < person['at'] + person['dur'] and not np.all((np.array([x, y]) > SLZ[0]) & (np.array([x, y]) < SLZ[1])):
            P = np.r_[P, [[0.40, 0.0], [0.40, 0.05], [0.42, -0.05]]]
        n.scan_xy = P; n.scan_t = clock.t
        if st['cam'] and st['cam_t'] is not None:
            lab = n.cur()[0]; plans.setdefault(lab, light_plan(rng)); n.cb_light(String(data=light_at(plans[lab], clock.t - st['cam_t'])))
        if realign['t'] is not None and clock.t >= realign['t']: realign['t'] = None; n.align_ver += 1
    feed(); n.tick(); n.tick(); t0 = clock.t; prev_state = None; hold_from = {}
    while steps < int(500 / dt):
        steps += 1; clock.t += dt
        a = min(1.0, max(0.0, (clock.t - st['h_t0']) / st['h_dur'])); st['h'] = st['h_from'] + (st['h_to'] - st['h_from']) * a
        if steps % 4 == 0:
            feed(); n.tick(); s = n.state; lab = n.cur()[0] if n.idx < len(n.goals) else 'END'
            if s != prev_state:
                if s == 'STOPLINE_HOLD' or (s == 'PARK_PRE'):
                    sx, sy = STOPS.get(lab, (x, y)); stop_err[lab] = math.hypot(x - sx, y - sy)
                if s == 'STOPLINE_WAIT_SIGNAL': hold_from[lab] = clock.t
                if prev_state == 'STOPLINE_WAIT_SIGNAL': go[n.goals[n.idx - 1][0]] = clock.t - hold_from.get(n.goals[n.idx - 1][0], clock.t)
                prev_state = s
            if s == 'DONE' or (s == 'EXIT_PUSH'): break
        c = np.array(cmd, float)
        if st['h'] < 0.25: c[:] = 0.0
        for i, mn in enumerate((0.08, 0.08, 0.15)):
            if 1e-4 < abs(c[i]) < mn: c[i] = mn * np.sign(c[i])
        if not (abs(c[0]) >= 0.06 or abs(c[1]) >= 0.06 or abs(c[2]) >= 0.10): c[:] = 0.0
        if not st['classic']: c[2] *= 0.43
        c[:2] *= gv; sp = math.hypot(c[0], c[1])
        if sp > 0.33: c[:2] *= 0.33 / sp
        if abs(c[0]) > 1e-3: c[2] += wb
        vel += (c - vel) * (dt / np.array([tau_v, tau_v, tau_w]))
        cy, sy = math.cos(yaw), math.sin(yaw); x += (cy * vel[0] - sy * vel[1]) * dt; y += (sy * vel[0] + cy * vel[1]) * dt; yaw += vel[2] * dt
        if body_hit(x, y, yaw):
            hit['slalom' if np.all((np.array([x, y]) > SLZ[0]) & (np.array([x, y]) < SLZ[1])) else 'other'] += dt
    msgs = [m for _, _, m in lg.m]
    cnt = lambda k: sum(k in m for m in msgs)
    try: n.destroy_node()
    except Exception: pass
    sig = {lab: dict(kind=str(plans[lab][0]), red_s=round(plans[lab][1], 1), went=round(go.get(lab, -1), 1), expect=expected_go(plans[lab])) for lab in plans}
    return dict(seed=seed, done=n.state in ('DONE', 'EXIT_PUSH'), state=n.state, T=round(clock.t - t0, 1), hit_slalom=round(hit['slalom'], 2), hit_other=round(hit['other'], 2),
                stop_err={k: round(v * 100, 1) for k, v in stop_err.items()}, signal=sig, rot_timeout=cnt('회전 타임아웃'), skip=cnt('막혀서'), front=cnt('앞 15 cm'),
                person=bool(person), regait=cnt('CLASSICWALK'), fell=cnt('넘어'))
if __name__ == '__main__':
    rclpy.init(); N = int(sys.argv[1]) if len(sys.argv) > 1 else 40; S0 = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    out = [run_eval(s) for s in range(S0, S0 + N)]
    json.dump(out, open(os.path.join(H, f'eval_{ROUTE}_{S0}.json'), 'w'), ensure_ascii=False, indent=1)
    print('끝', sum(o['done'] for o in out), '/', len(out))
