#!/usr/bin/env python3
"""슬라럼 S자 시뮬레이션 — 실제 러너(real_course_runner, 경로 전용)를 rec1650 슬라럼 구간(#A → #B)에서 운동 모델로 돌려 비교.
운동 모델: 러너가 5 Hz 로 내는 cmd_vel(vx, vy, wz, 몸 좌표) → cmd_vel_to_sport 최소값(0.08/0.08/0.15) → 1차 지연(τ 0.3 s) → 위치 적분.
장애물(콘·벽) = 3D 지도에서 바닥 위 0.10~0.65 m · 관측 5회 이상(지나간 사람·로봇 몸 점 제외)의 4 cm 칸.
비교: V0 지금 설정 / V1 먼저 돌기 기준 0.6 → 1.2 rad / V2 몸 방향 = 경로 방향(접선), 이동은 그대로 / V3 = V2 + V1
지표: 걸린 시간 · 제자리 돌기(앞으로 0, 회전만) 횟수 · 경로에서 벗어난 최대 · 몸이 콘·벽에 닿은 시간 · 몸-장애물 최소 여유
사용: ~/venv/go2/bin/python3 sim_scurve.py [A=6] [B=26]   (ROS Jazzy 환경 — 러너가 rclpy 를 씀)"""
import json
import math
import os
import sys
import tempfile

os.environ['RUNNER_PATH_ONLY'] = '1'; os.environ['RUNNER_TL'] = '0'
J = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))); sys.path.insert(0, os.path.join(J, 'src'))
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from scipy.spatial import cKDTree  # noqa: E402
from unitree_go.msg import SportModeState  # noqa: E402

import real_course_runner as rcr  # noqa: E402

_args = [a for a in sys.argv[1:] if a.isdigit()]; A = int(_args[0]) if _args else 6; B = int(_args[1]) if len(_args) > 1 else 26
R = json.load(open(os.environ.get('SIM_ROUTE') or os.path.join(J, 'routes', 'rec1650', 'route.json')))['route']   # SIM_ROUTE = 다른 경로 파일(같은 rec1650 지도)
seg = R[A - 1:B]
route = [dict(label='start', x=seg[0]['x'], y=seg[0]['y'])] + [dict(label=f'line{i + 1}', x=e['x'], y=e['y']) for i, e in enumerate(seg[1:-1])] + [dict(label='exit', x=seg[-1]['x'], y=seg[-1]['y'])]
# 장애물
d = np.load(os.path.join(J, 'routes', 'rec1650', 'map.npz')); P = (d['vox'] + 0.5) * float(d['V']); cnt = d['cnt']
lo, hi = np.array([[e['x'], e['y']] for e in route]).min(0) - 1.0, np.array([[e['x'], e['y']] for e in route]).max(0) + 1.0
k = np.all((P[:, :2] > lo) & (P[:, :2] < hi), 1); P, cnt = P[k], cnt[k]
key = np.floor(P[:, :2] / 0.5).astype(int); u, inv = np.unique(key, axis=0, return_inverse=True); inv = inv.ravel()
fl = np.full(len(u), np.inf); np.minimum.at(fl, inv, P[:, 2]); h = P[:, 2] - fl[inv]
if os.environ.get('OB_MODE', 'vertical') == 'vertical':      # 편집기와 같은 벽: 4 cm 칸에서 세로 0.25 m 넘게 선 면(평평한 판 제외), 녹화 길 0.18 m 안 몸 점 제외
    sys.argv = [sys.argv[0], 'rec1650'] + sys.argv[1:]; sys.path.insert(0, os.path.join(J, 'tools')); import route_editor as _ed
    OB = _ed.real_walls(_ed.load_cache()); OB = OB[np.all((OB > lo) & (OB < hi), 1)]
else:
    OB = P[(h > 0.10) & (h < 0.65) & (cnt >= 5), :2]; OB = np.unique(np.floor(OB / 0.04).astype(int), axis=0) * 0.04 + 0.02
if os.environ.get('SIM_CONES') == '1645':   # ★ 11-06: 슬라럼 구역 장애물은 rec1645(자세히 매핑한 첫 녹화) 것으로 — 2D 벽 ICP 로 rec1650 좌표에 옮김(debug/cones1645)
    _T = json.load(open(os.path.join(J, 'debug', 'cones1645', 'T_1650_1645.json')))['xy_yaw']
    sys.argv = [sys.argv[0], 'rec1645'] + sys.argv[2:]; import importlib; _ed = importlib.reload(_ed); _W = _ed.real_walls(_ed.load_cache())
    _c, _s = math.cos(_T[2]), math.sin(_T[2]); _W = np.c_[_c * _W[:, 0] - _s * _W[:, 1] + _T[0], _s * _W[:, 0] + _c * _W[:, 1] + _T[1]]
    _SL = np.array([[e['x'], e['y']] for e in R[6:25]]); _zl, _zh = _SL.min(0) - 0.4, _SL.max(0) + 0.4
    _inz = lambda Q: np.all((Q > _zl) & (Q < _zh), 1)
    OB = np.r_[OB[~_inz(OB)], _W[_inz(_W)]]
    sys.argv = [sys.argv[0], 'rec1650'] + sys.argv[2:]; _ed = importlib.reload(_ed)
if os.environ.get('SIM_ZONE_OB'):   # ★ 11-06: 슬라럼 구역 장애물을 파일로(debug/cones1645/zone_<지도>_c<관측>.npy, rec1650 좌표)
    _SL = np.array([[e['x'], e['y']] for e in R[6:25]]); _zl, _zh = _SL.min(0) - 0.4, _SL.max(0) + 0.4
    OB = np.r_[OB[~np.all((OB > _zl) & (OB < _zh), 1)], np.load(os.environ['SIM_ZONE_OB'])]
TO = cKDTree(OB)
L2, W2 = 0.35, 0.155


def body_hits(x, y, a):
    idx = TO.query_ball_point([x, y], 0.40)
    if not idx:
        return 0, 9.0
    q = OB[idx] - [x, y]; lx = q[:, 0] * math.cos(a) + q[:, 1] * math.sin(a); ly = -q[:, 0] * math.sin(a) + q[:, 1] * math.cos(a)
    inside = (np.abs(lx) < L2) & (np.abs(ly) < W2)
    gap = np.maximum(np.abs(lx) - L2, 0) ** 2 + np.maximum(np.abs(ly) - W2, 0) ** 2
    return int(inside.sum()), float(np.sqrt(gap.min()))


SL = np.array([[e['x'], e['y']] for e in R[6:25]]); ZLO, ZHI = SL.min(0) - 0.4, SL.max(0) + 0.4     # 슬라럼 #7~#25 구역


def recorded_baseline():
    """녹화(16:50, 사람이 리모컨으로 몬 주행 = 콘에 안 부딪힘)를 같은 몸 판정으로 — 판정이 얼마나 엄격한지의 기준."""
    od = d['odom']; od = od[np.argsort(od[:, 1])]; t = od[:, 1] - od[0, 1]; x, y = od[:, 2], od[:, 3]
    q = od[:, 5:9]; yaw = np.arctan2(2 * (q[:, 3] * q[:, 2] + q[:, 0] * q[:, 1]), 1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2))
    inz = (x > ZLO[0]) & (x < ZHI[0]) & (y > ZLO[1]) & (y < ZHI[1]); dtt = np.r_[np.diff(t), 0.05]
    hits = np.array([body_hits(a, b, c)[0] for a, b, c in zip(x, y, yaw)]); gaps = np.array([body_hits(a, b, c)[1] for a, b, c in zip(x, y, yaw)])
    curve = np.vstack([np.linspace([route[i]['x'], route[i]['y']], [route[i + 1]['x'], route[i + 1]['y']], 30) for i in range(len(route) - 1)])
    dev = cKDTree(curve).query(np.c_[x, y])[0]
    return dict(variant='REC', time=float(dtt[inz].sum()), done=True, spin_s=float('nan'), dev_max=float(dev[inz].max()), dev_p95=float(np.percentile(dev[inz], 95)),
                hit_s=float(dtt[inz & (hits > 0)].sum()), hit_n=int(hits[inz].sum() * 0), gap_min=float(gaps[inz].min()), side_v=float('nan'))


class Clock:
    t = 1000.0

    def monotonic(self):
        return self.t


def run(variant):
    clock = Clock(); rcr.time.monotonic = clock.monotonic
    rcr.FOLLOW_TURN_FIRST = 1.2 if variant in ('V1', 'V3') else 0.6
    rcr.HEADING_TANGENT = variant in ('V2', 'V3')
    f = tempfile.NamedTemporaryFile('w', suffix='.json', delete=False); json.dump({'format': 'route_v1', 'route': route}, f); f.close()
    rcr.WAYPOINTS_PATH = f.name; rcr.ALIGN_T = (0.0, 0.0, 0.0); s0, goals = rcr.load_waypoints(); os.unlink(f.name)
    n = rcr.RealCourseRunner(s0, goals); cmd = [0.0, 0.0, 0.0]
    n.pub_cmd_vel.publish = lambda m: cmd.__setitem__(slice(0, 3), [m.linear.x, m.linear.y, m.angular.z])
    n.pub_goal.publish = lambda m: None; n.pub_req.publish = lambda m: None; n.pub_camera_active.publish = lambda m: None
    # 몸 방향 = 경로 접선은 이제 러너 안에 있음(RUNNER_HEADING_TANGENT) — 판별로 켜고 끔
    a0 = math.atan2(route[1]['y'] - route[0]['y'], route[1]['x'] - route[0]['x'])     # #A 에 #A+1 쪽을 보게 놓음
    x, y, yaw = route[0]['x'], route[0]['y'], a0; vel = np.zeros(3); tau, dt = 0.3, 0.05

    def feed():
        m = Odometry(); m.header.stamp.sec = int(clock.t); m.header.stamp.nanosec = int((clock.t % 1) * 1e9)
        m.pose.pose.position.x, m.pose.pose.position.y = x, y; m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        n.cb_odom(m); s = SportModeState(); s.velocity = [float(math.hypot(vel[0], vel[1])), 0.0, 0.0]; s.body_height = 0.30; n.cb_state(s)

    feed(); n.tick(); n.tick()
    trace = []; spin_ticks = 0; steps = 0
    while steps < int(240 / dt):
        steps += 1; clock.t += dt
        if steps % 4 == 0:                               # 러너 5 Hz
            feed(); n.tick()
            if n.state == 'DONE' or (n.state in ('PATH_ARRIVE',) and n.cur()[0] == 'exit'):
                break
            if abs(cmd[0]) < 1e-3 and abs(cmd[1]) < 1e-3 and abs(cmd[2]) > 0.05 and n.state == 'PIN_TRACK':
                spin_ticks += 1
        c = np.array(cmd, float)
        for i, mn in enumerate((0.08, 0.08, 0.15)):       # cmd_vel_to_sport 최소 유효 명령
            if 1e-3 < abs(c[i]) < mn:
                c[i] = mn * np.sign(c[i])
        vel += (c - vel) * (dt / tau)
        cy, sy = math.cos(yaw), math.sin(yaw)
        x += (cy * vel[0] - sy * vel[1]) * dt; y += (sy * vel[0] + cy * vel[1]) * dt; yaw += vel[2] * dt
        hit, gap = body_hits(x, y, yaw); trace.append((clock.t - 1000.0, x, y, yaw, hit, gap, vel[0], vel[1], vel[2]))
    Tr = np.array(trace)
    inz = (Tr[:, 1] > ZLO[0]) & (Tr[:, 1] < ZHI[0]) & (Tr[:, 2] > ZLO[1]) & (Tr[:, 2] < ZHI[1])
    C = np.array(n.pin_path) if hasattr(n, 'pin_path') else None
    curve = np.vstack([np.linspace([route[i]['x'], route[i]['y']], [route[i + 1]['x'], route[i + 1]['y']], 30) for i in range(len(route) - 1)])
    dev = cKDTree(curve).query(Tr[:, 1:3])[0]
    done = n.state in ('DONE', 'PATH_ARRIVE') or math.hypot(x - route[-1]['x'], y - route[-1]['y']) < 0.2
    out = dict(variant=variant, time=float(inz.sum() * dt), done=bool(done), spin_s=spin_ticks * 0.2, dev_max=float(dev[inz].max()), dev_p95=float(np.percentile(dev[inz], 95)),
               hit_s=float(np.sum(Tr[inz, 4] > 0) * dt), hit_n=int(Tr[inz, 4].sum()), gap_min=float(Tr[inz, 5].min()), side_v=float(np.percentile(np.abs(Tr[inz, 7]), 95)))
    return out, Tr


rclpy.init()
NAMES = {'V0': '지금 설정', 'V1': '먼저 돌기 기준 완화(0.6→1.2 rad)', 'V2': '몸 방향 = 경로 방향', 'V3': 'V2 + 먼저 돌기 완화'}
res = {}
print(f'슬라럼 #{A} → #{B} ({len(route)}점, 경로 {sum(math.hypot(route[i + 1]["x"] - route[i]["x"], route[i + 1]["y"] - route[i]["y"]) for i in range(len(route) - 1)):.1f} m) · 장애물 칸 {len(OB)}')
print('(지표는 슬라럼 #7~#25 구역 안만)'); print(f'{"판":3s} {"설명":28s} {"완주":4s} {"시간":>6s} {"제자리돌기":>8s} {"경로이탈최대":>10s} {"몸닿음":>6s} {"최소여유":>7s} {"옆걸음p95":>8s}')
rb = recorded_baseline()
print(f'REC 녹화(사람 조종, 기준)          -   {rb["time"]:5.1f}s        -   {rb["dev_max"] * 100:8.1f}cm {rb["hit_s"]:5.1f}s {rb["gap_min"] * 100:6.1f}cm        -')
for v in ('V0', 'V1', 'V2', 'V3'):
    o, Tr = run(v); res[v] = (o, Tr)
    np.save(os.path.join(os.path.dirname(os.path.abspath(__file__)), f'sim_scurve_{v}{os.environ.get("SIM_TAG", "")}_trace.npy'), Tr)
    print(f'{v:3s} {NAMES[v]:28s} {"예" if o["done"] else "아니":4s} {o["time"]:5.1f}s {o["spin_s"]:7.1f}s {o["dev_max"] * 100:8.1f}cm {o["hit_s"]:5.1f}s {o["gap_min"] * 100:6.1f}cm {o["side_v"]:7.2f}m/s')
json.dump({v: o for v, (o, _) in res.items()}, open(os.path.join(os.path.dirname(os.path.abspath(__file__)), f'sim_scurve_{A}_{B}{os.environ.get("SIM_TAG", "")}.json'), 'w'), ensure_ascii=False, indent=1)
try:
    import matplotlib; matplotlib.use('Agg'); matplotlib.rcParams['font.family'] = 'NanumGothic'; matplotlib.rcParams['axes.unicode_minus'] = False
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 4, figsize=(20, 9), sharey=True)
    RP = np.array([[e['x'], e['y']] for e in route])
    for a_, v in zip(ax, ('V0', 'V1', 'V2', 'V3')):
        o, Tr = res[v]; a_.scatter(OB[:, 0], OB[:, 1], s=3, c='0.6'); a_.plot(RP[:, 0], RP[:, 1], 'b.--', lw=0.8, ms=5)
        a_.plot(Tr[:, 1], Tr[:, 2], c='g', lw=1.5)
        for kk in range(0, len(Tr), 20):
            xx, yy, aa, hh = Tr[kk, 1], Tr[kk, 2], Tr[kk, 3], Tr[kk, 4]; c_, s_ = math.cos(aa), math.sin(aa)
            cr = np.array([[L2, W2], [L2, -W2], [-L2, -W2], [-L2, W2], [L2, W2]])
            a_.plot(xx + cr[:, 0] * c_ - cr[:, 1] * s_, yy + cr[:, 0] * s_ + cr[:, 1] * c_, c='r' if hh else '0.3', lw=0.6, alpha=0.7)
        a_.set_aspect('equal'); a_.set_xlim(RP[:, 0].min() - 0.6, RP[:, 0].max() + 0.6); a_.grid(alpha=0.3)
        a_.set_title(f"{v} {NAMES[v]}\n{o['time']:.0f}초 · 제자리 {o['spin_s']:.0f}초 · 닿음 {o['hit_s']:.1f}초", fontsize=10)
    fig.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), f'sim_scurve_{A}_{B}.png'), dpi=70, bbox_inches='tight')
except Exception as e:  # noqa: BLE001
    print('그림 실패', e)
