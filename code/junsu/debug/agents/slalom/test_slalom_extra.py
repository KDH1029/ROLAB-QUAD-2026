#!/usr/bin/env python3
"""slalom 에이전트 추가 단위 테스트 — 슬라럼 대체 길·회전·rabaout 뒤 이어가기의 빠진 경우.
원본 테스트(src/test_real_course_runner.py)와 같은 가짜 시계 틀. 실패해도 멈추지 않고 끝에 표로 보여 줌.
  원본 러너:   PYTHONPATH=~/ws/junsu_v2/src python3 test_slalom_extra.py
  패치 러너:   PYTHONPATH=~/ws/junsu_v2/debug/agents/slalom/mirror/src python3 test_slalom_extra.py
[B] 표시 = 원본의 버그를 잡는 시험(원본에선 FAIL 이 정상, 패치에선 PASS 여야 함)."""
import math
import traceback
import numpy as np
import rclpy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from unitree_go.msg import SportModeState
import real_course_runner as rcr
from real_course_runner import RealCourseRunner

print('러너 파일:', rcr.__file__)
PATCHED = hasattr(RealCourseRunner, 'slalom_line')


class FakeClock:
    def __init__(self):
        self.t = 1000.0

    def monotonic(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def make_node(clock, goals):
    rclpy.init()
    node = RealCourseRunner((0.0, 0.0), goals)
    node._goals = []; node._cmds = []; node._zone = []; node._sports = []
    node.pub_goal.publish = lambda m: node._goals.append((m.pose.position.x, m.pose.position.y))
    node.pub_req.publish = lambda m: node._sports.append(m.header.identity.api_id)
    node.pub_camera_active.publish = lambda m: None
    node.pub_cmd_vel.publish = lambda m: node._cmds.append((m.linear.x, m.linear.y, m.angular.z))
    node.pub_cone_zone_active.publish = lambda m: node._zone.append(m.data)
    node.pub_realign.publish = lambda m: None
    node.set_ego_params = lambda p: None
    rcr.time.monotonic = clock.monotonic

    def odom(x, y, yaw=0.0):
        m = Odometry(); m.pose.pose.position.x = x; m.pose.pose.position.y = y
        m.pose.pose.orientation.z = math.sin(yaw / 2.0); m.pose.pose.orientation.w = math.cos(yaw / 2.0)
        node.cb_odom(m)

    def state(vel, h=0.30):
        m = SportModeState(); m.velocity = [vel, 0.0, 0.0]; m.body_height = h
        node.cb_state(m)
    node._odom = odom; node._state = state
    return node


def teardown(node):
    node.destroy_node(); rclpy.shutdown()


def set_walls(node, pts):
    node.wall_cells = set(map(tuple, np.floor(np.asarray(pts, float) / 0.05).astype(int).tolist())); node.wall_tree = None


# 지그재그 슬라럼 길(지도 = 오늘 좌표, 정렬 T 없음). rabain (1,0) → rabaout (3,0)
SP = [(0.5, 0.0), (1.0, 0.0), (1.5, 0.4), (2.0, -0.4), (2.5, 0.4), (3.0, 0.0)]
GOALS = [('rabain', (1.0, 0.0)), ('rabaout', (3.0, 0.0)), ('line', (3.0, 2.0)), ('exit', (3.0, 3.0))]


def into_zone(node, clock, yaw=0.0):
    """rabain 을 방향 맞게(회전 없이) 지나 콘 구간(rabaout 목표)까지."""
    node._odom(0.1, 0.0); node.tick()
    node._state(0.3); node._odom(1.0, 0.0, yaw=yaw); node.tick()
    assert node.cur()[0] == 'rabaout' and node.state == 'TO_GOAL', (node.cur(), node.state)


def ride(node, clock, path, dt=0.2, every=5):
    for q in path[::every]:
        clock.advance(dt)
        if len(path) > 1:
            node._odom(float(q[0]), float(q[1]))
        node.tick()
        if node.state != 'PIN_TRACK':
            return


RES = []


def case(name, bug=False):
    def deco(f):
        clock = FakeClock(); rcr.SLALOM_PATH_MAP = list(SP); mode0 = rcr.SLALOM_MODE
        rcr.EGO_REVIEW = 1.0          # 관리자(go2_ctl runner_env)·시뮬과 같게 — 0 이면 check_giveup(막히면 대체 길) 자체가 안 불림
        try:
            f(clock); RES.append((name, bug, 'PASS', ''))
        except Exception as e:  # noqa
            RES.append((name, bug, 'FAIL', f'{type(e).__name__}: {e}'))
            traceback.print_exc(limit=1)
        finally:
            rcr.SLALOM_MODE = mode0; rcr.SLALOM_PATH_MAP = []
            try:
                teardown(case.node)
            except Exception:  # noqa
                pass
        return f
    return deco


@case('A1 path 모드: 콘 구간 들어서자마자 대체 길(첫 tick)')
def _(clock):
    rcr.SLALOM_MODE = 'path'
    node = case.node = make_node(clock, GOALS); into_zone(node, clock)
    node.tick()
    assert node.state == 'PIN_TRACK' and node.pin_mode == 'slalom', (node.state, node.pin_mode)
    assert node._zone and node._zone[-1] is True, f'rabain 에서 cone_zone_active ON 기대: {node._zone}'


@case('A2 ego 모드: 검출 없어도·막혀도 대체 길 안 씀(콘 무시 직진 ROUTE_FOLLOW)')
def _(clock):
    rcr.SLALOM_MODE = 'ego'
    node = case.node = make_node(clock, GOALS); into_zone(node, clock)
    for _ in range(20):
        clock.advance(0.2); node.tick()
    assert node.state == 'TO_GOAL', node.state
    node.set_cone_level(2, clock.t, 'test'); clock.advance(rcr.CONE_GIVEUP_S_CONE + 0.1); node.tick()
    assert node.state == 'ROUTE_FOLLOW', f'ego 모드 포기 → ROUTE_FOLLOW 기대: {node.state}'


@case('A3 auto: 콘을 보는데 막힘(콘 무시 5초 못 나아감) → 대체 길')
def _(clock):
    rcr.SLALOM_MODE = 'auto'
    node = case.node = make_node(clock, GOALS); into_zone(node, clock)
    for _ in range(15):
        node.cone_seen_t = clock.t; clock.advance(0.2); node.tick()
    assert node.state == 'TO_GOAL'
    node.set_cone_level(2, clock.t, 'test')
    for _ in range(30):
        node.cone_seen_t = clock.t; clock.advance(0.2); node.tick()
        if node.state != 'TO_GOAL':
            break
    assert node.state == 'PIN_TRACK' and node.pin_mode == 'slalom', (node.state, node.pin_mode)


@case('A4 대체 길 도중 막힘(6초 진행 없음) → 보통 주행, 콘 구간 유지, slalom_check 가 바로 다시 안 시작')
def _(clock):
    rcr.SLALOM_MODE = 'path'
    node = case.node = make_node(clock, GOALS); into_zone(node, clock); node.tick()
    assert node.state == 'PIN_TRACK'
    q = node.pin_path[20]; node._odom(float(q[0]), float(q[1])); clock.advance(0.2); node.tick()   # 조금 가다가
    for _ in range(int(rcr.TRACK_STALL_S / 0.2) + 3):                                             # 그 자리에 낌
        clock.advance(0.2); node.tick()
    assert node.state == 'TO_GOAL' and node.pin_mode is None, (node.state, node.pin_mode)
    assert node.cur()[0] == 'rabaout' and node.idx in node.cone_zone_idx, node.cur()
    assert node._zone[-1] is True, f'막혀도 콘 구간 표시는 켜진 채 기대: {node._zone}'
    assert node._cmds[-1] == (0.0, 0.0, 0.0) or any(c == (0.0, 0.0, 0.0) for c in node._cmds[-3:]), '포기할 때 정지 명령 기대'
    clock.advance(0.2); node.tick()
    assert node.state == 'TO_GOAL', f'slalom_done 이라 바로 다시 대체 길로 안 가야: {node.state}'


@case('A5 rabaout 이 대체 길 끝보다 0.9 m 멀 때 → 길 끝에 rabaout 붙여 끝까지 → 도착 처리·회전·cone_zone OFF·다음 점')
def _(clock):
    rcr.SLALOM_MODE = 'path'
    goals = [('rabain', (1.0, 0.0)), ('rabaout', (3.9, 0.0)), ('line', (3.9, 2.0)), ('exit', (3.9, 3.0))]
    node = case.node = make_node(clock, goals); into_zone(node, clock); node.tick()
    assert node.state == 'PIN_TRACK'
    end = node.pin_path[-1]; assert math.hypot(end[0] - 3.9, end[1]) < 0.03, f'길 끝 = rabaout 기대: {end}'
    ride(node, clock, node.pin_path)
    node._odom(3.9, 0.0, yaw=0.0); node._state(0.0); node.tick()
    for _ in range(5):
        if node.state == 'ROTATE':
            break
        clock.advance(0.2); node.tick()
    assert node.state == 'ROTATE', f'rabaout(다음 점 90° 왼쪽) 도착 → 회전 기대: {node.state} {node.cur()}'
    for deg in (30, 60, 80, 95):
        clock.advance(0.3); node._odom(3.9, 0.0, yaw=math.radians(deg)); node.tick()
    assert node.cur()[0] == 'line' and node.state == 'TO_GOAL', (node.cur(), node.state)
    assert node._zone[-1] is False, f'rabaout 끝 → cone_zone_active OFF 기대: {node._zone}'


@case('A6 대체 길 끝이 rabaout 0.4 m 밖에서 끝나도(PIN_REACH) rabaout 도착으로 이어짐(같은 자리 0.4 m 안)')
def _(clock):
    rcr.SLALOM_MODE = 'path'
    node = case.node = make_node(clock, GOALS); into_zone(node, clock); node.tick()
    ride(node, clock, node.pin_path)
    assert node.state != 'PIN_TRACK', '길 끝까지 갔으면 대체 길이 끝나야'
    node._state(0.0); node._odom(2.85, 0.05, yaw=0.0)
    for _ in range(5):
        clock.advance(0.2); node.tick()
    assert node.cur()[0] in ('line', 'exit') or node.state == 'ROTATE', (node.cur(), node.state)


@case('B1 [B] 슬라럼 길에서 멀면(2 m, 다른 통로) 대체 길 안 시작(벽 너머 직선 금지)', bug=True)
def _(clock):
    rcr.SLALOM_MODE = 'auto'
    node = case.node = make_node(clock, GOALS); into_zone(node, clock)
    node._odom(1.5, 2.0); ok = node.start_slalom_path('test')
    assert not ok and node.state != 'PIN_TRACK', f'2 m 밖에서 대체 길 시작함(원본 동작): {node.state}'


@case('B2 [B] 꼭짓점 바로 앞(선분 위)에서 시작해도 그 꼭짓점을 건너뛰지 않음(콘 사이 모서리 가로지르기 금지)', bug=True)
def _(clock):
    rcr.SLALOM_MODE = 'auto'
    node = case.node = make_node(clock, GOALS); into_zone(node, clock)
    node._odom(1.85, -0.25)                       # (1.5,0.4)→(2.0,-0.4) 선분 위, (2.0,-0.4) 꼭짓점 앞 0.18 m
    assert node.start_slalom_path('test')
    C = node.pin_path
    dmin = float(np.min(np.hypot(C[:, 0] - 2.0, C[:, 1] + 0.4)))
    assert dmin < 0.05, f'(2.0,-0.4) 꼭짓점을 지나야 하는데 {dmin:.2f} m 비껴감(모서리 가로지름)'


@case('B3 [B] 콘 구간에서 ego 가 슬라럼 길을 1 m 넘게 3초 벗어나면(코스 거꾸로) 되짚은 뒤 대체 길', bug=True)
def _(clock):
    rcr.SLALOM_MODE = 'auto'
    node = case.node = make_node(clock, GOALS); into_zone(node, clock)
    for k in range(12):                           # 콘을 보면서(대체 길 안 씀) rabain 에서 왼쪽 위로 빠짐
        node.cone_seen_t = clock.t; clock.advance(0.5); node._odom(1.0 - 0.05 * k, 0.15 * k, yaw=math.radians(120)); node.tick()
    for _ in range(int(rcr.OFF_ROUTE_S / 0.5) + 2):
        node.cone_seen_t = clock.t; clock.advance(0.5); node._odom(0.45, 1.65, yaw=math.radians(120)); node.tick()
        if node.state == 'PIN_TRACK':
            break
    assert node.state == 'PIN_TRACK' and node.pin_mode == 'rejoin', f'콘 구간 이탈 → 되짚기 기대: {node.state} {node.pin_mode}'
    ride(node, clock, node.pin_path, every=2)
    assert node.state == 'PIN_TRACK' and node.pin_mode == 'slalom', f'되짚은 뒤 슬라럼 대체 길 기대: {node.state} {node.pin_mode}'


@case('B4 [B] drive_toward: 옆(90°) 0.18 m 콘은 앞 막힘 아님 — 앞으로 감', bug=True)
def _(clock):
    node = case.node = make_node(clock, GOALS); node._odom(1.0, 0.0, yaw=0.0)
    set_walls(node, [(1.0 + dx, 0.18) for dx in (-0.05, 0.0, 0.03)])
    node.drive_toward(2.0, 0.0, 0.25)
    vx = node._cmds[-1][0]
    assert vx > 0.15, f'옆 콘 때문에 앞으로 안 감(vx {vx:+.2f})'



@case('B5 drive_toward: 바로 앞 0.15 m 벽이면 앞으로 안 감(그대로 지켜야 함)')
def _(clock):
    node = case.node = make_node(clock, GOALS); node._odom(1.0, 0.0, yaw=0.0)
    set_walls(node, [(1.15, dy) for dy in (-0.05, 0.0, 0.05)])
    node.drive_toward(2.0, 0.0, 0.25)
    vx = node._cmds[-1][0]
    assert vx <= 0.0, f'앞 벽인데 앞으로 감(vx {vx:+.2f})'


@case('B6 [B] drive_toward: 막혀서 밀어내기만 남아도 발 떼기 문턱(0.10 m/s) 넘는 명령', bug=True)
def _(clock):
    node = case.node = make_node(clock, GOALS); node._odom(1.0, 0.0, yaw=0.0)
    set_walls(node, [(1.17, 0.12)])               # 앞-왼쪽 비스듬히 0.21 m 점 하나: 막힘 + 약한 밀어내기
    node.drive_toward(2.0, 0.0, 0.25)
    vx, vy, wz = node._cmds[-1]; sp = math.hypot(vx, vy)
    assert sp == 0.0 or sp >= 0.10, f'문턱 아래 명령 {sp:.3f} m/s → 제자리(시뮬: 6초 진행 없음 포기)'


@case('B7 [B] 미끄러지는 목표(CARROT_M=1): 콘 구간 목표가 rabain 뒤(슬라럼 길 첫 점 쪽)로 가지 않음', bug=True)
def _(clock):
    rcr.SLALOM_MODE = 'ego'; c0 = rcr.CARROT_M; rcr.CARROT_M = 1.0
    try:
        rcr.SLALOM_PATH_MAP = [(0.3, 0.0)] + SP[1:]   # 실제처럼 슬라럼 길 첫 점이 rabain 0.7 m 뒤(1650: 1.1 m)
        node = case.node = make_node(clock, GOALS); into_zone(node, clock)
        node._odom(1.0, 0.0); c = node.carrot()
        assert c is not None and c[0] > 1.2, f'목표가 뒤로 감: {c}'
    finally:
        rcr.CARROT_M = c0


@case('C1 회전 점 규칙: 25° 안이면 서지 않고 통과, 넘으면 서서 회전·15° 안 1초면 완료')
def _(clock):
    node = case.node = make_node(clock, [('rabain', (1.0, 0.0)), ('rabaout', (2.0, 0.3)), ('line', (2.0, 2.0)), ('exit', (2.0, 3.0))])
    rcr.SLALOM_MODE = 'ego'
    node._odom(0.1, 0.0); node.tick(); node._state(0.3); node._odom(1.0, 0.0, yaw=0.0); node.tick()
    assert node.cur()[0] == 'rabaout' and node.state == 'TO_GOAL', f'17° 차이 → 회전 없이 통과 기대: {node.cur()} {node.state}'
    node._state(0.0); node._odom(2.0, 0.3, yaw=0.0); node.tick()
    assert node.state == 'ROTATE', f'90° 차이 → 회전 기대: {node.state}'
    for k in range(8):
        clock.advance(0.2); node._odom(2.0, 0.3, yaw=math.radians(78 - (k % 2))); node.tick()
    assert node.cur()[0] == 'line' and node._zone[-1] is False, (node.cur(), node._zone)


@case('C2 회전 점에서 아직 움직이는 중(속도 0.2)이면 회전 시작 안 함(이동하며 돌지 않기)')
def _(clock):
    node = case.node = make_node(clock, GOALS); rcr.SLALOM_MODE = 'ego'
    node._odom(0.1, 0.0); node.tick(); node._state(0.2); node._odom(1.0, 0.0, yaw=math.radians(90)); node.tick()
    assert node.state == 'TO_GOAL' and node.cur()[0] == 'rabain', (node.state, node.cur())
    node._state(0.0); node.tick(); assert node.state == 'ROTATE'


print('\n===== 결과 (' + ('패치 러너' if PATCHED else '원본 러너') + ') =====')
for name, bug, r, why in RES:
    print(f'{r:4s} {name}' + (f'\n      └ {why}' if why else ''))
nb = [x for x in RES if not x[1]]; bb = [x for x in RES if x[1]]
print(f"\n일반 {sum(x[2] == 'PASS' for x in nb)}/{len(nb)} PASS · 버그 잡는 시험 [B] {sum(x[2] == 'PASS' for x in bb)}/{len(bb)} PASS")
