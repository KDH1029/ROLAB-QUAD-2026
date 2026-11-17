#!/usr/bin/env python3
"""슬라럼 S자 랜덤화(몬테카를로) — sim_scurve.py 와 같은 러너·지도·몸 판정에 무작위 조건을 넣어 판별로 여러 번.
무작위: 출발 위치 ±0.10 m·방향 ±5° · 위치 잡음(매 틱 σ 1.5 cm·0.5°) · 반응 지연 τ 0.2~0.5 s · 속도 추종 80~110 % · 남은 쏠림 ±0.01 rad/s
같은 씨앗을 판끼리 공유(짝 비교). 사용: python3 sim_mc.py <판 V0|V2> <씨앗 시작> <개수> → mc_<판>_<시작>.json"""
import json
import math
import os
import sys

H = os.path.dirname(os.path.abspath(__file__))
VAR, S0, NRUN = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
MINS = {'G0': (0.08, 0.08, 0.15), 'G1': (0.12, 0.12, 0.30)}.get(VAR, (0.08, 0.08, 0.15)); STICK = VAR in ('G0', 'G1', 'G2', 'G3')
VMIN = {'G2': 0.15, 'G3': 0.18}.get(VAR, 0.12)   # 표시 지점 앞 감속 하한(PATH_V_MIN)
sys.argv = [sys.argv[0], '6', '26']
src = open(os.path.join(H, 'sim_scurve.py')).read()
exec(compile(src[:src.index('rclpy.init()')], 'sim_scurve_head', 'exec'))   # 같은 경로·장애물·몸 판정·러너
_nb = np.array([len(TO.query_ball_point(q, 0.12)) - 1 for q in OB]); OBc = OB[_nb >= 2]; TOc = cKDTree(OBc)   # 외톨이 잡음 뺀 장애물(사람 주행 보정과 같은 판정)


def sgap(x, y, a):
    """몸 0.70×0.31 과 장애물의 부호 있는 거리(파고들면 음수) — rec_check/calib 과 같은 함수."""
    idx = TOc.query_ball_point([x, y], 0.55)
    if not idx:
        return 0.5
    q = OBc[idx] - [x, y]; lx = np.abs(q[:, 0] * math.cos(a) + q[:, 1] * math.sin(a)); ly = np.abs(-q[:, 0] * math.sin(a) + q[:, 1] * math.cos(a))
    out = np.hypot(np.maximum(lx - L2, 0), np.maximum(ly - W2, 0)); ins = (lx < L2) & (ly < W2)
    return float(np.where(ins, -np.minimum(L2 - lx, W2 - ly), out).min())


def run_rand(variant, seed):
    rng = np.random.default_rng(seed)
    clock = Clock(); rcr.time.monotonic = clock.monotonic
    rcr.FOLLOW_TURN_FIRST = 1.2 if variant in ('V1', 'V3') else 0.6
    rcr.HEADING_TANGENT = variant in ('V2', 'V3', 'G0', 'G1', 'G2', 'G3'); rcr.PATH_V_MIN = VMIN
    f = tempfile.NamedTemporaryFile('w', suffix='.json', delete=False); json.dump({'format': 'route_v1', 'route': route}, f); f.close()
    rcr.WAYPOINTS_PATH = f.name; rcr.ALIGN_T = (0.0, 0.0, 0.0); s0, goals = rcr.load_waypoints(); os.unlink(f.name)
    n = rcr.RealCourseRunner(s0, goals); cmd = [0.0, 0.0, 0.0]
    n.pub_cmd_vel.publish = lambda m: cmd.__setitem__(slice(0, 3), [m.linear.x, m.linear.y, m.angular.z])
    n.pub_goal.publish = lambda m: None; n.pub_req.publish = lambda m: None; n.pub_camera_active.publish = lambda m: None
    # 몸 방향 = 경로 접선은 이제 러너 안에 있음(RUNNER_HEADING_TANGENT) — 판별로 켜고 끔
    a0 = math.atan2(route[1]['y'] - route[0]['y'], route[1]['x'] - route[0]['x'])
    x = route[0]['x'] + rng.uniform(-0.1, 0.1); y = route[0]['y'] + rng.uniform(-0.1, 0.1); yaw = a0 + math.radians(rng.uniform(-5, 5))
    tau = rng.uniform(0.2, 0.5); gain = rng.uniform(0.8, 1.1); wz_bias = rng.uniform(-0.01, 0.01); vel = np.zeros(3); dt = 0.05
    thr_v = rng.uniform(0.08, 0.14); thr_w = rng.uniform(0.20, 0.35); stuck = 0.0; stuck_log = []   # ★ 발 떼기 문턱(실측 없음 → 범위): 이보다 작으면 몸만 기울고 안 움직임

    def feed():
        nx, ny, na = x + rng.normal(0, 0.015), y + rng.normal(0, 0.015), yaw + math.radians(rng.normal(0, 0.5))
        m = Odometry(); m.header.stamp.sec = int(clock.t); m.header.stamp.nanosec = int((clock.t % 1) * 1e9)
        m.pose.pose.position.x, m.pose.pose.position.y = nx, ny; m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(na / 2), math.cos(na / 2)
        n.cb_odom(m); s = SportModeState(); s.velocity = [float(math.hypot(vel[0], vel[1])), 0.0, 0.0]; s.body_height = 0.30; n.cb_state(s)

    feed(); n.tick(); n.tick()
    trace = []; spin = 0; steps = 0
    while steps < int(90 / dt):
        steps += 1; clock.t += dt
        if steps % 4 == 0:
            feed(); n.tick()
            if n.state == 'DONE' or (n.state == 'PATH_ARRIVE' and n.cur()[0] == 'exit'):
                break
            if abs(cmd[0]) < 1e-3 and abs(cmd[1]) < 1e-3 and abs(cmd[2]) > 0.05 and n.state == 'PIN_TRACK':
                spin += 1
        c = np.array(cmd, float)
        for i, mn in enumerate(MINS):
            if 1e-4 < abs(c[i]) < mn:
                c[i] = mn * np.sign(c[i])
        c *= gain; c[2] += wz_bias if abs(c[0]) > 1e-3 else 0.0
        if STICK and not (abs(c[0]) >= thr_v or abs(c[1]) >= thr_v or abs(c[2]) >= thr_w):   # 어느 축도 문턱을 못 넘으면 발을 안 뗌
            if np.any(np.abs(c) > 1e-3):
                stuck += dt; stuck_log.append((round(clock.t - 1000.0, 1), round(math.hypot(x - route[-1]['x'], y - route[-1]['y']), 2), n.state, round(float(np.abs(cmd[0])), 3), round(float(np.abs(cmd[2])), 3)))
            c = np.zeros(3)
        vel += (c - vel) * (dt / tau)
        cy, sy = math.cos(yaw), math.sin(yaw)
        x += (cy * vel[0] - sy * vel[1]) * dt; y += (sy * vel[0] + cy * vel[1]) * dt; yaw += vel[2] * dt
        hit, gap = body_hits(x, y, yaw); trace.append((clock.t - 1000.0, x, y, yaw, hit, gap))
    Tr = np.array(trace); inz = (Tr[:, 1] > ZLO[0]) & (Tr[:, 1] < ZHI[0]) & (Tr[:, 2] > ZLO[1]) & (Tr[:, 2] < ZHI[1])
    curve = np.vstack([np.linspace([route[i]['x'], route[i]['y']], [route[i + 1]['x'], route[i + 1]['y']], 30) for i in range(len(route) - 1)])
    dev = cKDTree(curve).query(Tr[:, 1:3])[0]
    done = n.state in ('DONE', 'PATH_ARRIVE') or math.hypot(x - route[-1]['x'], y - route[-1]['y']) < 0.25
    try:
        n.destroy_node()
    except Exception:  # noqa: BLE001
        pass
    end_d = math.hypot(x - route[-1]['x'], y - route[-1]['y']); along = (x - route[-1]['x']) * math.cos(yaw) + (y - route[-1]['y']) * math.sin(yaw)
    tg = [[round(float(r_[0]), 2), round(float(r_[1]), 3), round(float(r_[2]), 3), round(sgap(r_[1], r_[2], r_[3]), 3)] for r_, z_ in zip(Tr, inz) if z_]   # 슬라럼 안 [t, x, y, 부호 있는 여유]
    return dict(seed=seed, tg=tg, end_d=end_d, overshoot=max(0.0, along), stuck=stuck, stuck_first=stuck_log[:1], stuck_last=stuck_log[-1:], stuck_n=len(stuck_log), thr_v=thr_v, thr_w=thr_w, done=bool(done), time=float(inz.sum() * dt), total=float(Tr[-1, 0]), spin=spin * 0.2, hit=float(np.sum(Tr[inz, 4] > 0) * dt),
                gap=float(Tr[inz, 5].min()), dev=float(dev[inz].max()), tau=tau, gain=gain)


rclpy.init()
out = [run_rand(VAR, s) for s in range(S0, S0 + NRUN)]
json.dump(out, open(os.path.join(H, f'mc_{VAR}{os.environ.get("SIM_TAG", "")}_{S0}.json'), 'w'))
print(VAR, S0, '끝', sum(o['done'] for o in out), '/', len(out))
