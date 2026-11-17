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
GAIT = {'C2': 'classic', 'S2': 'sport', 'S2h': 'sport', 'C0': 'classic', 'S0': 'sport'}.get(VAR); SPORT_WALK_TWIST = VAR == 'S2h'   # S2h = 사용자 관찰 가정: 걸으면서도 작은 회전(문턱 아래)은 30 %만 돎   # ★ 11-06 걸음새 모델(debug/gait 실측) — 러너 값(PATH_V_MIN 등)은 그대로
sys.argv = [sys.argv[0], '6', '26']
src = open(os.path.join(H, 'sim_scurve.py')).read()
exec(compile(src[:src.index('rclpy.init()')], 'sim_scurve_head', 'exec'))   # 같은 경로·장애물·몸 판정·러너


def run_rand(variant, seed):
    rng = np.random.default_rng(seed)
    clock = Clock(); rcr.time.monotonic = clock.monotonic
    rcr.FOLLOW_TURN_FIRST = 1.2 if variant in ('V1', 'V3') else 0.6
    rcr.HEADING_TANGENT = variant in ('V2', 'V3', 'G0', 'G1', 'G2', 'G3', 'C2', 'S2', 'S2h')
    if GAIT is None: rcr.PATH_V_MIN = VMIN
    f = tempfile.NamedTemporaryFile('w', suffix='.json', delete=False); json.dump({'format': 'route_v1', 'route': route}, f); f.close()
    rcr.WAYPOINTS_PATH = f.name; rcr.ALIGN_T = (0.0, 0.0, 0.0); s0, goals = rcr.load_waypoints(); os.unlink(f.name)
    n = rcr.RealCourseRunner(s0, goals); cmd = [0.0, 0.0, 0.0]
    n.pub_cmd_vel.publish = lambda m: cmd.__setitem__(slice(0, 3), [m.linear.x, m.linear.y, m.angular.z])
    n.pub_goal.publish = lambda m: None; n.pub_req.publish = lambda m: None; n.pub_camera_active.publish = lambda m: None
    # 몸 방향 = 경로 접선은 이제 러너 안에 있음(RUNNER_HEADING_TANGENT) — 판별로 켜고 끔
    a0 = math.atan2(route[1]['y'] - route[0]['y'], route[1]['x'] - route[0]['x'])
    x = route[0]['x'] + rng.uniform(-0.1, 0.1); y = route[0]['y'] + rng.uniform(-0.1, 0.1); yaw = a0 + math.radians(rng.uniform(-5, 5))
    tau = rng.uniform(0.2, 0.5); gain = rng.uniform(0.8, 1.1); wz_bias = rng.uniform(-0.01, 0.01); vel = np.zeros(3); dt = 0.05
    thr_v = rng.uniform(0.08, 0.14); thr_w = rng.uniform(0.20, 0.35); stuck = 0.0; stuck_log = []
    tau_v = tau_w = tau; part_v = part_w = 0.0
    if GAIT == 'classic':   # 16:45 실측: 지연 회전 0.16 · 전진 0.35 s, 전진 문턱 ≈0.06~0.08 m/s(아래는 0), 회전 문턱은 자료 없음 → 0.08~0.14 가정, 문턱 아래 몸만 돌기 0.8 s/241 s ≈ 없음
        tau_w = rng.uniform(0.14, 0.22); tau_v = rng.uniform(0.30, 0.40); thr_v = rng.uniform(0.05, 0.08); thr_w = rng.uniform(0.08, 0.14)
    elif GAIT == 'sport':   # 16:50·08-28 실측: 지연 회전 0.17 · 전진 0.32 s, 전진 문턱 ≈0.10~0.15(아래 15~28 %), 회전 문턱 ≈0.25(08-28), 발 안 떼고 몸만 도는 회전 ≈ 명령의 30 % (9.0 s/136 s)
        tau_w = rng.uniform(0.14, 0.22); tau_v = rng.uniform(0.28, 0.38); thr_v = rng.uniform(0.10, 0.15); thr_w = rng.uniform(0.20, 0.30); part_v = 0.2; part_w = 0.3
    TAU = np.array([tau_v, tau_v, tau_w])   # ★ 발 떼기 문턱(실측 없음 → 범위): 이보다 작으면 몸만 기울고 안 움직임

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
        if GAIT and not (abs(c[0]) >= thr_v or abs(c[1]) >= thr_v or abs(c[2]) >= thr_w):   # 발 안 뗌: 클래식은 그대로 섬, 스포츠는 몸만 일부 돌고/기움
            if np.any(np.abs(c) > 1e-3):
                stuck += dt; stuck_log.append((round(clock.t - 1000.0, 1), round(math.hypot(x - route[-1]['x'], y - route[-1]['y']), 2), n.state, round(float(np.abs(cmd[0])), 3), round(float(np.abs(cmd[2])), 3)))
            c = np.array([c[0] * part_v, 0.0, c[2] * part_w])
        elif SPORT_WALK_TWIST and 1e-3 < abs(c[2]) < thr_w:   # 걷는 중(전진은 문턱 넘음)에도 작은 회전은 일부만
            c[2] *= part_w
        elif STICK and not (abs(c[0]) >= thr_v or abs(c[1]) >= thr_v or abs(c[2]) >= thr_w):   # 어느 축도 문턱을 못 넘으면 발을 안 뗌
            if np.any(np.abs(c) > 1e-3):
                stuck += dt; stuck_log.append((round(clock.t - 1000.0, 1), round(math.hypot(x - route[-1]['x'], y - route[-1]['y']), 2), n.state, round(float(np.abs(cmd[0])), 3), round(float(np.abs(cmd[2])), 3)))
            c = np.zeros(3)
        vel += (c - vel) * (dt / TAU)
        cy, sy = math.cos(yaw), math.sin(yaw)
        x += (cy * vel[0] - sy * vel[1]) * dt; y += (sy * vel[0] + cy * vel[1]) * dt; yaw += vel[2] * dt
        hit, gap = body_hits(x, y, yaw); trace.append((clock.t - 1000.0, x, y, yaw, hit, gap))
    Tr = np.array(trace); inz = (Tr[:, 1] > ZLO[0]) & (Tr[:, 1] < ZHI[0]) & (Tr[:, 2] > ZLO[1]) & (Tr[:, 2] < ZHI[1])
    curve = np.vstack([np.linspace([route[i]['x'], route[i]['y']], [route[i + 1]['x'], route[i + 1]['y']], 30) for i in range(len(route) - 1)])
    dev = cKDTree(curve).query(Tr[:, 1:3])[0]
    # ★ 11-06 받침대: 슬라럼 콘 10개 중심(cone_centres_rec1650.npy) − 네 발(±0.19, ±0.14) 거리
    Fx = []
    for _, x_, y_, a_ in Tr[inz, :4]:
        c_, s_ = math.cos(a_), math.sin(a_); Fx += [(x_ + c_ * fx - s_ * fy, y_ + s_ * fx + c_ * fy) for fx, fy in ((0.19, 0.14), (0.19, -0.14), (-0.19, 0.14), (-0.19, -0.14))]
    fdist = TCONE.query(np.array(Fx))[0] if Fx else np.array([9.0])
    done = n.state in ('DONE', 'PATH_ARRIVE') or math.hypot(x - route[-1]['x'], y - route[-1]['y']) < 0.25
    try:
        n.destroy_node()
    except Exception:  # noqa: BLE001
        pass
    end_d = math.hypot(x - route[-1]['x'], y - route[-1]['y']); along = (x - route[-1]['x']) * math.cos(yaw) + (y - route[-1]['y']) * math.sin(yaw)
    return dict(seed=seed, end_d=end_d, overshoot=max(0.0, along), stuck=stuck, stuck_first=stuck_log[:1], stuck_last=stuck_log[-1:], stuck_n=len(stuck_log), thr_v=thr_v, thr_w=thr_w, done=bool(done), time=float(inz.sum() * dt), total=float(Tr[-1, 0]), spin=spin * 0.2, hit=float(np.sum(Tr[inz, 4] > 0) * dt),
                gap=float(Tr[inz, 5].min()), dev=float(dev[inz].max()), tau=tau, gain=gain, gait=GAIT, path_v_min=rcr.PATH_V_MIN,
                dev_p95=float(np.percentile(dev[inz], 95)), dev_mean=float(dev[inz].mean()),
                foot_cone_min=float(fdist.min()), plate38_best=float(np.mean(fdist < 0.19 + 0.041)), plate38_worst=float(np.mean(fdist < 0.269 + 0.041)))


TCONE = cKDTree(np.load(os.path.join(J, 'debug', 'cones1645', 'cone_centres_1645_in1650.npy')) if os.environ.get('SIM_CONES') == '1645' else np.load(os.path.join(H, 'cone_centres_rec1650.npy')))
rclpy.init()
out = [run_rand(VAR, s) for s in range(S0, S0 + NRUN)]
json.dump(out, open(os.path.join(H, f'mc_{VAR}{os.environ.get("SIM_TAG", "")}_{S0}.json'), 'w'))
print(VAR, S0, '끝', sum(o['done'] for o in out), '/', len(out))
