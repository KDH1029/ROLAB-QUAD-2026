#!/usr/bin/env python3
"""review 재현 테스트(ROS 없이 함수 단위, 가짜 시계). 원본 src 를 그대로 import 해서 버그가 재현되는지 본다.
  원본:   ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=81 PYTHONDONTWRITEBYTECODE=1 python3 test_review_repro.py
  패치본: REVIEW_SRC=patched ... python3 test_review_repro.py      (patched/src 의 고친 사본)
각 항목: '재현됨(버그)' 또는 '고쳐짐'. 원본에서 전부 재현, 패치본에서 전부 고쳐짐이면 정상.
"""
import importlib.util
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
J = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
SRC = os.path.join(HERE, 'patched', 'src') if os.environ.get('REVIEW_SRC') == 'patched' else os.path.join(J, 'src')
sys.path.insert(0, SRC)
sys.argv = [sys.argv[0]]

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from unitree_go.msg import SportModeState  # noqa: E402
import real_course_runner as rcr  # noqa: E402

print(f'러너: {rcr.__file__}')
RESULTS = []


def report(name, bug, detail):
    RESULTS.append((name, bug))
    print(f"[{'재현됨(버그)' if bug else '고쳐짐/정상'}] {name}: {detail}")


class Clock:
    def __init__(self):
        self.t = 1000.0

    def monotonic(self):
        return self.t


def make(mod, clock, goals, start=(0.0, 0.0)):
    n = mod.RealCourseRunner(start, goals)
    n._cmd = []; n._goal = []
    n.pub_goal.publish = lambda m: n._goal.append((m.pose.position.x, m.pose.position.y))
    n.pub_req.publish = lambda m: None
    n.pub_camera_active.publish = lambda m: None
    n.pub_cone_zone_active.publish = lambda m: None
    n.pub_cmd_vel.publish = lambda m: n._cmd.append((m.linear.x, m.linear.y, m.angular.z))
    if hasattr(n, "pub_realign"):
        n.pub_realign.publish = lambda m: None
    n.pub_status.publish = lambda m: None
    mod.time.monotonic = clock.monotonic

    def odom(x, y, yaw=0.0):
        m = Odometry(); m.pose.pose.position.x = float(x); m.pose.pose.position.y = float(y)
        m.pose.pose.orientation.z = math.sin(yaw / 2); m.pose.pose.orientation.w = math.cos(yaw / 2); n.cb_odom(m)
    n._odom = odom

    def state(v=0.0, h=0.31):
        m = SportModeState(); m.velocity = [float(v), 0.0, 0.0]; m.body_height = float(h); n.cb_state(m)
    n._state = state
    return n


def wall_cells(points):
    return set(map(tuple, np.floor(np.asarray(points, float) / 0.05).astype(int).tolist()))


def hline(y, x0, x1):
    xs = np.arange(x0, x1, 0.02); return np.c_[xs, np.full(len(xs), y)]


def vline(x, y0, y1):
    ys = np.arange(y0, y1, 0.02); return np.c_[np.full(len(ys), x), ys]


rclpy.init()

# ---------------------------------------------------------------- B1 목표 보정이 벽 0.08 m 안 점에서 꺼짐(16:54 대비 퇴행)
clock = Clock()
goals = [('line1', (0.0, 0.03)), ('exit', (3.0, 1.0))]
W = wall_cells(hline(-0.01, -1.0, 1.0))           # 벽 칸 중심 y = -0.025
n = make(rcr, clock, goals); n.wall_cells = W; n._odom(-1.0, 0.5)
tx, ty = n.target()
spec = importlib.util.spec_from_file_location('old_runner_1654', os.path.join(HERE, 'old_runner_1654.py'))
old = importlib.util.module_from_spec(spec); spec.loader.exec_module(old)
o = make(old, clock, goals); o.wall_cells = W; o._odom(-1.0, 0.5)
ox, oy = o.target()
c0 = float(n._tree().query([0.0, 0.03])[0])
report('B1 목표 보정(벽 0.08 m 안)', abs(ty - 0.03) < 1e-6 and oy > 0.3,
       f'원래 점 벽까지 {c0:.3f} m → v2 목표 y {ty:.2f} (16:54 코드 y {oy:.2f})')
# 얇은 벽 너머로 옮기지 않는지(11-06 의도)는 유지돼야 함
W2 = wall_cells(np.r_[hline(-0.01, -1.0, 1.0), hline(0.30, -1.0, 1.0)])    # 폭 0.3 m 통로 + 위쪽 넓은 통로
n.wall_cells = W2; n.wall_tree = None; n.snapped = {}
tx2, ty2 = n.target()
report('B1b 얇은 벽 너머로 안 옮김(11-06 의도 유지)', ty2 > 0.30, f'통로 안 점 (0, 0.03) → ({tx2:.2f}, {ty2:.2f}) (y>0.30 이면 벽 너머)')
o.destroy_node(); n.destroy_node()

# ---------------------------------------------------------------- B2 target() CPU
clock = Clock()
n = make(rcr, clock, [('line1', (0.0, 0.15)), ('exit', (3.0, 1.0))])
pts = np.r_[hline(-0.01, -3, 3), hline(0.62, -3, 3), vline(-3, -0.01, 0.62), vline(3, -0.01, 0.62)]
pts = np.r_[pts] + 0.0
n.wall_cells = wall_cells(pts); n._odom(-1.0, 0.3)
n.target()
f = (lambda: n._target_calc('line1', 0.0, 0.15, n._tree())) if hasattr(n, '_target_calc') else n.target
t0 = time.perf_counter()
for _ in range(20):
    f()
ms = (time.perf_counter() - t0) / 20 * 1000
calls = 1 if hasattr(n, '_target_calc') else 4.5          # 패치본: 같은 목표·같은 벽이면 틱 안에서 다시 안 셈
report('B2 target() 계산 비용', ms * calls > 5.0, f'한 번 계산 {ms:.2f} ms × 틱당 {calls} 번 → 틱당 ~{ms * calls:.1f} ms (5 Hz, 목표가 벽 0.40 m 안이면 매 틱)')
n.destroy_node()

# ---------------------------------------------------------------- B3 되짚기 6초 포기 직후 다시 되짚기(경로 위 자리 없는 trail)
clock = Clock()
goals = [('line1', (0.0, 0.0)), ('line2', (5.0, 0.0)), ('exit', (6.0, 0.0))]
n = make(rcr, clock, goals); n.idx = 1; n.state = 'TO_GOAL'
n.trail = []; n.off_since = None                 # pin_step 의 6초 포기 직후와 같음
started = None
for k in range(20):                              # ego 가 막다른 통로 안에서 조금씩 움직임(경로에서 1.5~2.1 m)
    n._odom(2.0, 1.5 + 0.03 * k); clock.t += 0.2
    if n.offroute_check(clock.t):
        started = k; break
if started is not None:
    end = n.pin_path[-1]
    dseg = abs(end[1])
    report('B3 포기 직후 되짚기 재발', dseg > 0.4, f'{started * 0.2:.1f}초 뒤 다시 되짚기 시작, 끝점 ({end[0]:.2f},{end[1]:.2f}) = 경로에서 {dseg:.2f} m(경로로 안 돌아감)')
else:
    report('B3 포기 직후 되짚기 재발', False, '경로 위 자리가 없는 trail 로는 되짚기 안 함(ego 에 맡김)')
n.destroy_node()

# ---------------------------------------------------------------- B4 좁은 통로(0.60 m)에서 뒤로 걸어 나오기 막힘
clock = Clock()
n = make(rcr, clock, [('line1', (2.0, 0.0)), ('exit', (3.0, 0.0))])
n.wall_cells = wall_cells(np.r_[hline(0.30, -2, 2), hline(-0.30 - 0.05, -2, 2)])   # 벽 칸 중심 y ≈ ±0.325
n._odom(0.0, 0.0, 0.0); n._cmd.clear()
n.back_out_toward(-1.0, 0.0)
vx, vy, _ = n._cmd[-1]
report('B4 좁은 통로 뒤로 나오기', vx > -0.1, f'통로 폭 0.60 m 가운데, 뒤 1 m 로 → 명령 vx {vx:+.2f} vy {vy:+.2f} (기대 vx -0.20)')
n.destroy_node()

# ---------------------------------------------------------------- B5 주차 바로 옆 wall1(1~2 cm 구간) — ego 없이 앞으로 계속 걸음
clock = Clock()
goals = [('line12', (1.0, -1.0)), ('parking', (0.0, 0.0)), ('wall1', (0.0, 0.02)), ('line13', (1.2, 0.14)), ('exit', (3.0, 0.0))]
n = make(rcr, clock, goals)
n.wall_cells = wall_cells(np.r_[vline(-0.9, -1.5, 1.5), hline(0.50, -1.5, 3.0)])   # 주차 칸: 뒤-왼쪽 벽 x=-0.9, 로봇 왼쪽 벽 y=0.5
n.idx = 2; n.state = 'TO_GOAL'; x, y = 0.10, -0.25; n._odom(x, y, 0.0)              # 주차 뒤 회전으로 line13(+x) 쪽을 봄, wall1 에서 0.29 m
n.tick()
for k in range(25):                                                                  # 5초
    if n.state != 'WALL_TRACK':
        break
    bx = np.arange(-0.5, 1.2, 0.04); n.scan_xy = np.c_[bx, np.full(len(bx), 0.525 - y)]; n.scan_t = clock.t   # 몸체 왼쪽 벽(앞벽 없음)
    n._cmd.clear(); n.wall_step(clock.t)
    if n.state == 'WALL_TRACK' and n._cmd:
        vx_, vy_, _ = n._cmd[-1]; x += vx_ * 0.2; y += vy_ * 0.2; n._odom(x, y, 0.0)
    clock.t += 0.2
moved = math.hypot(x - 0.10, y + 0.25)
report('B5 wall1 퇴화 구간', moved > 0.5, f'구간 길이 0.02 m 인데 벽 추종으로 ego 없이 {moved:.2f} m 걸음(5초, 상태 {n.state}, 목표 {n.cur()[0]}) — 실제론 앞벽 0.5 m 가 나올 때까지')
n.destroy_node()

# ---------------------------------------------------------------- B6 위치 노드: FALL_REALIGN 중 재정렬 요청을 러너가 포기할 때까지 미룸
os.environ.setdefault('PYTHONDONTWRITEBYTECODE', '1')
sys.path.insert(0, os.path.join(J, 'tools')); sys.path.insert(0, os.path.join(J, 'tools', 'start_align'))
import junsu_loc_node as L  # noqa: E402
loc = L.Loc()
called = []
loc.start_realign = lambda d: called.append((loc.lio_t, d))
loc.init_amcl = lambda: None
loc.aligned = True; loc.lio = (0.0, 0.0, 0.0); loc.lio_t = 100.0; loc.body_h = 0.31
loc.cb_runner(String(data=json.dumps({'state': 'FALLEN'})))
loc.holding()
loc.cb_runner(String(data=json.dumps({'state': 'FALL_REALIGN'})))           # 러너: 다시 섬 → BalanceStand → 재정렬 요청 → 응답 기다림(최대 6초)
loc.cb_realign(String(data=json.dumps({'max_xy': 0.6, 'max_yaw': 45.0, 'why': 'fall'})))
t_req = loc.lio_t
for k in range(60):                                                           # 러너가 FALL_REALIGN 인 6초 동안
    loc.lio_t += 0.1; loc.holding()
during = list(called)
loc.cb_runner(String(data=json.dumps({'state': 'TO_GOAL'})))                # 러너가 '응답 없음'으로 포기하고 주행 시작
for k in range(25):
    loc.lio_t += 0.1; loc.holding()
if not during and called:
    report('B6 넘어진 뒤 재정렬', True, f'러너가 기다리는 6초 동안 재정렬 0번, 러너가 포기하고 주행 시작 {called[0][0] - t_req - 6.0:.1f}초 뒤(걷는 중) ±{called[0][1]["max_xy"]} m/±{called[0][1]["max_yaw"]:.0f}° 로 실행')
else:
    report('B6 넘어진 뒤 재정렬', False, f'러너가 기다리는 동안 실행 {len(during)}번' + (f'(요청 {during[0][0] - t_req:.1f}초 뒤)' if during else ''))
loc.destroy_node()

# ---------------------------------------------------------------- B7 관리자 상태 파일이 새 기본 경로를 덮어씀
sys.path.insert(0, os.path.join(HERE, 'patched', 'tools') if os.environ.get('REVIEW_SRC') == 'patched' else os.path.join(J, 'tools'))
import go2_ctl as G  # noqa: E402
G.J = J; G.STATE = os.path.join(J, 'maps', 'segments', 'logs', 'go2_ctl_state.json'); G.LOG = os.path.dirname(G.STATE)   # 패치본도 실제 상태 파일(읽기만)
G.OPS['route'] = os.path.join(J, 'maps', 'route_map_1106.json')
default_route = G.OPS['route']
G.threading.Thread = lambda *a, **k: type('T', (), {'start': lambda self: None})()   # lc_loop(ros2 lifecycle 반복) 안 띄움


class _Mon:
    hz = {}; runner = {}; robot = {}


real_state = G.STATE
c = G.Ctl(_Mon())
report('B7 상태 파일이 기본 경로를 덮음', os.path.basename(G.OPS['route']) != os.path.basename(default_route),
       f"코드 기본 {os.path.basename(default_route)} → 관리자가 실제로 쓸 경로 {os.path.basename(G.OPS['route'])} "
       f"(상태 파일 {real_state}, guard {G.OPS.get('guard')})")
names = [o for o in ('guard',)]
src = open(G.__file__).read()
report('B8 필터 끔인데 a(전체 켜기)가 guard 노드를 띄움', "o[0] != 'guard'" not in src, "start_all 순서에 guard 가 OPS['guard'] 와 무관하게 들어 있음")

rclpy.shutdown()
bugs = [r for r in RESULTS if r[1]]
print(f"\n요약: {len(bugs)}/{len(RESULTS)} 재현됨 — " + ', '.join(r[0].split()[0] for r in bugs))
