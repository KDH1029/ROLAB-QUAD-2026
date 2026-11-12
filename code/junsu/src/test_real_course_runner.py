#!/usr/bin/env python3
"""real_course_runner.py 전체 상태기 오프라인 검증 (가짜 시계, 실제 sleep 없음).
이 파일은 지금까지 테스트가 하나도 없었음 -- 실제 대회에 쓸 가능성이 제일 높은
스크립트인데도 커버리지 0이었던 것도 이번에 같이 메움.

확인 항목:
1) START_CHECK: 시작 위치가 맞으면 TO_GOAL로, 많이 떨어지면 계속 멈춰있음(에러만 로그).
2) 일반 구간(line* 등) 도착 -> 바로 다음 구간.
3) stopline: 도착 -> camera_active=True, 2.5초 정지(STOPLINE_HOLD) -> 신호 확인
   (STOPLINE_WAIT_SIGNAL) -> 단발 GREEN은 무시, 0.5초 연속 GREEN이어야 출발
   -> camera_active=False.
4) parking: 도착 -> Sit 전송 -> 1.5초(SIT_SETTLE) -> 5.5초(PARK_HOLD) ->
   RiseSit 전송 -> 2.0초(RISE_SETTLE) -> 다음(exit).
5) send_sport()가 identity.id(0이 아닌 값)와 parameter='{}' 를 채워서 보내는지
   (2026-11-04 수정 -- 로봇이 조용히 무시하는 걸 막기 위함).
"""
import sys
import math
import numpy as np
sys.path.insert(0, '/tmp')
import rclpy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from unitree_go.msg import SportModeState
import real_course_runner as rcr
from real_course_runner import (RealCourseRunner, STOPLINE_HOLD_S, GREEN_CONFIRM_SECONDS,
                                 SIT_SETTLE_S, PARK_HOLD_S, RISE_SETTLE_S, PARK_DOWN_ID as SIT_ID, PARK_UP_ID as RISESIT_ID,
                                 ROBOT_SPORT_API_ID_STATICWALK, ROBOT_SPORT_API_ID_TROTRUN,
                                 STOPLINE_GREEN_TIMEOUT_S, ROTATE_SETTLE_S, ROTATE_DONE_THRESH,
                                 PARK_PRE_S, SIT_RETRY_MAX, RISE_RETRY_MAX, BALANCESTAND_ID, EXIT_PASS_MARGIN)


# ★ 11-06 13시: 실주행 기본 PASS_SWITCH 1.2·SHORTCUT_CLEAR 0.15 — 이 시험들의 점 간격(약 1 m)은 0.8·0.25 를 전제로 짜여 있어 시험 안에서는 예전 값
rcr.PASS_SWITCH = 0.8; rcr.SHORTCUT_CLEAR = 0.25
rcr.PIN_EGO_FIRST = False   # ★ 11-06 13시: 핀 'ego 먼저'는 4q2 에서만 시험(나머지 핀 시험은 예전처럼 바로 핀)


class FakeClock:
    def __init__(self):
        self.t = 1000.0

    def monotonic(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def make_node(clock, goals=None):
    rclpy.init()
    if goals is None:
        goals = [('line1', (1.0, 0.0)), ('stopline1', (2.0, 0.0)),
                  ('parking', (3.0, 0.0)), ('exit', (4.0, 0.0))]
    node = RealCourseRunner((0.0, 0.0), goals)
    sent_goals = []
    sent_sports = []
    camera_states = []
    sent_cmd_vel = []
    node.pub_goal.publish = lambda m: sent_goals.append((m.pose.position.x, m.pose.position.y))
    node.pub_req.publish = lambda m: sent_sports.append(
        (m.header.identity.api_id, m.header.identity.id, m.parameter))
    node.pub_camera_active.publish = lambda m: camera_states.append(m.data)
    node.pub_cmd_vel.publish = lambda m: sent_cmd_vel.append(m.angular.z)
    rcr.time.monotonic = clock.monotonic
    node._sent_goals = sent_goals
    node._sent_sports = sent_sports
    node._camera_states = camera_states
    node._sent_cmd_vel = sent_cmd_vel

    def odom(x, y, yaw=0.0):
        m = Odometry()
        m.pose.pose.position.x = x
        m.pose.pose.position.y = y
        m.pose.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.pose.orientation.w = math.cos(yaw / 2.0)
        node.cb_odom(m)
    node._odom = odom

    def state(vel, body_height=0.30):
        # body_height 기본값 0.30 -- SIT_HEIGHT_THRESH(0.15)보다 높아서 "서있는
        # 상태"를 뜻함. 앉은 상태를 흉내낼 땐 명시적으로 0.10 등을 넘겨야 함.
        m = SportModeState(); m.velocity = [vel, 0.0, 0.0]; m.body_height = body_height
        node.cb_state(m)
    node._state = state

    def light(state):
        node.cb_light(String(data=f'{state}|0.90'))
    node._light = light

    return node


def teardown(node):
    node.destroy_node()
    rclpy.shutdown()


# --- 1) START_CHECK: 시작 위치가 틀리면 멈춰있음, 맞으면 TO_GOAL ---
clock = FakeClock()
node = make_node(clock)
node._odom(5.0, 5.0)  # start(0,0)에서 한참 떨어짐
node.tick()
assert node.state == 'START_CHECK', f"자가진단 실패해야 하는데 넘어감: {node.state}"
assert len(node._sent_goals) == 0, "자가진단 실패했는데 목표가 나감(버그!)"
node._odom(0.1, 0.0)  # start 근처로 복귀
node.count_publishers = lambda topic: 1   # 실제 로봇처럼 /api/sport/response 가 있음
node.tick()
# ★ 11-06: classic 걸음새면 출발 전 START_GAIT(BalanceStand → 0.5초 → ClassicWalk → 성공 응답 확인)
assert node.state == 'START_GAIT', f"자가진단 통과 뒤 START_GAIT 여야 함: {node.state}"
assert any(a == rcr.BALANCESTAND_ID for a, *_ in node._sent_sports), f"BalanceStand 를 먼저 보내야 함: {node._sent_sports}"
clock.advance(0.6); node.tick()
assert any(a == rcr.ROBOT_SPORT_API_ID_CLASSICWALK for a, *_ in node._sent_sports), f"0.5초 뒤 ClassicWalk: {node._sent_sports}"
class _H: pass
def _resp(code):
    m = _H(); m.header = _H(); m.header.identity = _H(); m.header.status = _H()
    m.header.identity.api_id = rcr.ROBOT_SPORT_API_ID_CLASSICWALK; m.header.status.code = code; return m
node.cb_sport_resp(_resp(-1)); node.tick()            # 거절 → BalanceStand 부터 다시
assert node.state == 'START_GAIT' and node.gait_try == 2, f"거절 뒤 다시 시도해야 함: {node.state} {node.gait_try}"
clock.advance(0.6); node.tick(); node.cb_sport_resp(_resp(0)); node.tick()
assert node.state == 'TO_GOAL', f"ClassicWalk 성공 뒤 출발해야 함: {node.state}"
print('1b) 출발 걸음새 확인(START_GAIT: 거절 → 다시 → 성공 → 출발) OK')
print("1) START_CHECK 통과/실패 분기 OK")
teardown(node)

# --- 2) 일반 구간 도착 -> 바로 다음 구간(line1 -> stopline1 전까지는 그대로 TO_GOAL) ---
clock = FakeClock()
node = make_node(clock)
node._odom(0.1, 0.0)
node.tick()  # START_CHECK -> TO_GOAL (line1)
assert node.cur()[0] == 'line1'
node._odom(1.0, 0.0)  # line1 도착
node.tick()
assert node.cur()[0] == 'stopline1', f"line1 도착 후 다음 구간 전환 실패: {node.cur()[0]}"
print("2) 일반 구간(line1) 도착 -> 바로 다음 구간 전환 OK")
teardown(node)


def advance_to_stopline(node, clock):
    node._odom(0.1, 0.0)
    node.tick()  # -> TO_GOAL(line1)
    node._odom(1.0, 0.0)
    node.tick()  # line1 도착 -> stopline1
    node._state(0.0)  # 2026-11-04: 속도 게이트 추가 -- 도착 전에 "멈춤"을 먼저 알려줘야 함
    node._odom(2.0, 0.0)  # stopline1 도착
    node.tick()
    assert node.state == 'STOPLINE_HOLD', f"정지선 도착 후 STOPLINE_HOLD 기대: {node.state}"


# --- 3) stopline: camera_active ON -> 2.5초 정지 -> 신호확인 -> 단발 GREEN 무시 -> 연속 GREEN 출발 ---
clock = FakeClock()
node = make_node(clock)
advance_to_stopline(node, clock)
assert node._camera_states[-1] is True, "정지선 도착 시 camera_active=True 발행 기대"

clock.advance(STOPLINE_HOLD_S - 0.1)
node._odom(2.0, 0.0)
node.tick()
assert node.state == 'STOPLINE_HOLD', "2.5초 다 안 지났는데 신호확인 단계로 넘어감(버그!)"

clock.advance(0.2)  # 이제 2.5초 넘음
node._odom(2.0, 0.0)
node.tick()
assert node.state == 'STOPLINE_WAIT_SIGNAL', f"2.5초 경과 후 신호확인 단계 기대: {node.state}"

node._light('GREEN')
clock.advance(0.1)  # 확인 시간 안 채움 -- 단발성 오탐 시뮬레이션
node._odom(2.0, 0.0)
node.tick()
assert node.state == 'STOPLINE_WAIT_SIGNAL', "0.1초만 GREEN인데 출발함(디바운스 버그!)"

clock.advance(GREEN_CONFIRM_SECONDS - 0.2)  # 아직 확인 시간 직전(경기장 가짜 GREEN 최장 0.4초보다 길게) -- 출발하면 안 됨
node._light('GREEN')
node._odom(2.0, 0.0)
node.tick()
assert node.state == 'STOPLINE_WAIT_SIGNAL', f"{GREEN_CONFIRM_SECONDS - 0.1:.1f}초 GREEN인데 벌써 출발함"

clock.advance(0.2)  # 끊기지 않고 확인 시간 + 0.1초 누적
node._light('GREEN')
node._odom(2.0, 0.0)
node.tick()
assert node.state == 'TO_GOAL', f"{GREEN_CONFIRM_SECONDS + 0.1:.1f}초 연속 GREEN인데 출발 안 함: {node.state}"
assert node.cur()[0] == 'parking', f"정지선 통과 후 다음 구간 기대(parking): {node.cur()[0]}"
assert node._camera_states[-1] is False, "출발 시 camera_active=False 발행 기대"
print(f"3) stopline: camera ON -> 2.5초 정지 -> 단발 GREEN 무시 -> {GREEN_CONFIRM_SECONDS}초 연속 GREEN 출발 -> camera OFF  OK")
teardown(node)

# --- 4) parking: Sit -> 1.5s -> 5.5s -> RiseSit -> 2.0s -> 다음(exit) ---
clock = FakeClock()
node = make_node(clock)
advance_to_stopline(node, clock)
clock.advance(STOPLINE_HOLD_S + 0.1)
node._odom(2.0, 0.0)
node.tick()  # -> STOPLINE_WAIT_SIGNAL
node._light('GREEN')
clock.advance(GREEN_CONFIRM_SECONDS + 0.1)
node._light('GREEN')   # 노드는 10 Hz 로 계속 보낸다(끊김 판정 LIGHT_STALE_S)
node._odom(2.0, 0.0)
node.tick()  # -> TO_GOAL(parking)
assert node.cur()[0] == 'parking'

node._state(0.0)  # 속도 게이트 -- 엎드리기는 "멈춘 뒤에만"
node._odom(3.0, 0.0, yaw=0.0)
node.tick()
# ★ 11-05 오후: 주차는 도착 회전 없이 바로 엎드리기 준비(PARK_PRE: ego 를 제자리 목표로 세움)
assert node.state == 'PARK_PRE', f"주차 도착 후 PARK_PRE 기대: {node.state}"
assert node._sent_goals[-1] == (3.0, 0.0), f"엎드리기 전 ego 를 제자리에 세우는 목표 기대: {node._sent_goals[-1]}"
clock.advance(PARK_PRE_S + 0.05); node.tick()
assert node.state == 'PARK_SIT_WAIT' and node._sent_sports[-1][0] == SIT_ID, (node.state, node._sent_sports[-1])
clock.advance(SIT_SETTLE_S + 0.1); node._odom(3.0, 0.0); node.tick()
assert node.state == 'PARK_SIT_WAIT', f"아직 안 엎드렸는데 PARK_HOLD로 넘어감(버그!): {node.state}"
node._state(0.0, body_height=0.06); node._odom(3.0, 0.0); node.tick()
assert node.state == 'PARK_HOLD', f"엎드림 확인됐는데 PARK_HOLD로 안 넘어감: {node.state}"
clock.advance(PARK_HOLD_S + 0.1); node._odom(3.0, 0.0); node.tick()
assert node.state == 'PARK_RISE_WAIT' and node._sent_sports[-1][0] == RISESIT_ID, (node.state, node._sent_sports[-1])
clock.advance(RISE_SETTLE_S + 0.6); node._odom(3.0, 0.0); node.tick()
assert node.state == 'PARK_RISE_WAIT' and sum(x[0] == RISESIT_ID for x in node._sent_sports) == 2, "일어나기 재전송 기대"
node._state(0.0, body_height=0.30); clock.advance(1.1); node._odom(3.0, 0.0); node.tick()
ids = [x[0] for x in node._sent_sports]
if rcr.PARK_MODE == 'down':
    assert rcr.STANDDOWN_ID in ids and rcr.STANDUP_ID in ids and ids[-1] == rcr.BALANCESTAND_ID, ids
assert node.state == 'PARK_REALIGN', f"일어난 뒤 재정렬 기다림 기대: {node.state}"
node.cb_align_T(String(data='{"T": [0.0, 0.0, 0.0], "ok": true, "reason": "realign"}')); node.tick()
assert node.state == 'ROTATE' and node.cur()[0] == 'parking', f"재정렬 응답 뒤 회전 기대: {node.state}"
# ★ 11-06: 회전 전에 주행 걸음새(ClassicWalk)부터 — BalanceStand(기본 보행)로 돌면 기울었다 튐. 걸음새 뒤 GAIT_SETTLE_S 동안은 회전 명령 0
if rcr.GAIT_MODE == 'classic':
    assert [x[0] for x in node._sent_sports][-1] == rcr.ROBOT_SPORT_API_ID_CLASSICWALK, f"회전 전에 ClassicWalk 기대: {[x[0] for x in node._sent_sports][-3:]}"
    n0 = len(node._sent_cmd_vel); clock.advance(rcr.GAIT_SETTLE_S - 0.2); node._odom(3.0, 0.0, yaw=0.0); node.tick()
    assert node.state == 'ROTATE' and all(w == 0.0 for w in node._sent_cmd_vel[n0:]), f"걸음새 자리 잡는 중엔 회전 0 기대: {node._sent_cmd_vel[n0:]}"
    clock.advance(0.3)
node.tick(); clock.advance(ROTATE_SETTLE_S + 0.1); node._odom(3.0, 0.0, yaw=0.0); node.tick()
assert node.cur()[0] == 'exit', f"주차 뒤 회전 끝나면 exit 구간 기대: {node.cur()[0]} {node.state}"
print(f"4) parking: 도착 -> PARK_PRE(ego 정지) -> 엎드리기 -> 높이 확인 -> {PARK_HOLD_S}s -> 일어나기(재전송) -> BalanceStand -> 재정렬 -> ClassicWalk {rcr.GAIT_SETTLE_S}s -> 회전 -> exit OK")
teardown(node)

# --- 4b) 속도가 안 죽으면(빠르게 지나가는 중) 도착해도 Sit을 안 보냄(전도 방지 게이트) ---
clock = FakeClock()
node = make_node(clock)
advance_to_stopline(node, clock)
clock.advance(STOPLINE_HOLD_S + 0.1)
node._odom(2.0, 0.0)
node.tick()
node._light('GREEN')
clock.advance(GREEN_CONFIRM_SECONDS + 0.1)
node._light('GREEN')   # 노드는 10 Hz 로 계속 보낸다(끊김 판정 LIGHT_STALE_S)
node._odom(2.0, 0.0)
node.tick()
assert node.cur()[0] == 'parking'

node._state(0.5)  # 아직 빠르게 움직이는 중(STOP_VEL_THRESH=0.05보다 훨씬 큼)
node._odom(3.0, 0.0)  # 위치상으론 parking 도착
node.tick()
assert node.state == 'TO_GOAL', f"속도가 안 죽었는데 Sit 보내고 넘어감(전도 위험 버그!): {node.state}"
assert not any(s[0] == SIT_ID for s in node._sent_sports), "속도 안 죽었는데 Sit 명령이 나감(버그!)"
node._state(0.0)  # 이제 멈춤
node._odom(3.0, 0.0, yaw=0.0)
node.tick()
assert node.state == 'PARK_PRE', f"멈춘 뒤엔 바로 엎드리기 준비(회전 없음): {node.state}"
node.tick()
clock.advance(0.0)
node._odom(3.0, 0.0, yaw=0.0)
node.tick()
clock.advance(PARK_PRE_S + 0.05)
node.tick()
assert node.state == 'PARK_SIT_WAIT', f"회전 완료 후 정상적으로 Sit 보내고 넘어가야 함: {node.state}"
print("4b) 속도 안 죽은 상태로 도착 -> 엎드리기 보류, 멈춘 뒤 엎드리기 전송 OK")
teardown(node)

# --- 4c) 정지 유지 중(STOPLINE_HOLD) 다시 움직이면 처음부터(TO_GOAL) 다시 시작 ---
clock = FakeClock()
node = make_node(clock)
advance_to_stopline(node, clock)  # -> STOPLINE_HOLD, 속도 0으로 들어옴
assert node.state == 'STOPLINE_HOLD'

clock.advance(1.0)  # 2.5초 중 1초만 지남
node._state(0.2)  # 밀림/재출발 등으로 다시 움직임 감지(STOP_VEL_THRESH=0.05보다 큼)
node._odom(2.0, 0.0)
node.tick()
assert node.state == 'STOPLINE_HOLD', "★ 11-05 오후: 순간 움직임(속도 잡음·미끄럼)으로는 다시 접근 안 함"
clock.advance(rcr.HOLD_MOVE_S + 0.05); node._odom(2.0, 0.0); node.tick()
assert node.state == 'TO_GOAL', f"{rcr.HOLD_MOVE_S}초 넘게 움직였는데 계속 STOPLINE_HOLD에 머무름(버그!): {node.state}"

# 다시 멈추면 처음부터(0초부터) 2.5초를 다시 채워야 함 -- 이전에 지난 1초가
# 누적되면 안 됨
node._state(0.0)
node._odom(2.0, 0.0)
node.tick()
assert node.state == 'STOPLINE_HOLD', f"다시 멈췄는데 STOPLINE_HOLD로 안 돌아감: {node.state}"
clock.advance(1.0)  # 이전에 지났던 1초만큼만 다시 지남(누적이면 2.5초 넘어서 통과했을 것)
node._odom(2.0, 0.0)
node.tick()
assert node.state == 'STOPLINE_HOLD', \
    f"리셋된 타이머가 이전 구간이랑 누적됐음(버그!): {node.state}"
clock.advance(1.6)  # 이제 리셋 후로 총 2.6초 지남(2.5초 기준 통과)
node._odom(2.0, 0.0)
node.tick()
assert node.state == 'STOPLINE_WAIT_SIGNAL', \
    f"리셋 후 제대로 2.5초 다 채웠는데 신호확인 단계로 안 넘어감: {node.state}"
print("4c) STOPLINE_HOLD 유지 중 재이동 -> TO_GOAL 리셋, 타이머 누적 안 됨 OK")
teardown(node)

rcr.GAIT_MODE = 'legacy'   # 4d·4d2 는 예전 걸음새 전환(RUNNER_GAIT_MODE=legacy) 시험
# --- 4d) 'top' 구간(계단/경사로 정상) 접근 시 STATICWALK로 전환, 빠져나오면 TROTRUN ---
clock = FakeClock()
node = make_node(clock, goals=[('line1', (1.0, 0.0)), ('top1', (2.0, 0.0)),
                                ('line2', (3.0, 0.0))])
node._odom(0.1, 0.0)
node.tick()  # START_CHECK -> TO_GOAL(line1) (상태전환만, TO_GOAL 로직은 다음 틱부터)
node.tick()  # TO_GOAL(line1) 로직 실제 실행 -> gait 발행
assert node.cur()[0] == 'line1'
gaits = [s[0] for s in node._sent_sports if s[0] in (ROBOT_SPORT_API_ID_STATICWALK, ROBOT_SPORT_API_ID_TROTRUN)]
assert gaits[-1] == ROBOT_SPORT_API_ID_TROTRUN, f"line1(평지) 접근 중엔 TROTRUN이어야 함: {gaits}"

node._odom(1.0, 0.0)  # line1 도착 -> top1로 전환
node.tick()  # 이 틱 안에서는 아직 'line1' 기준으로 gait 평가됨(도착은 이 틱 끝에 처리) -> 한 틱 더 필요
node.tick()  # 이제 cur()이 'top1'이라 STATICWALK 전송
assert node.cur()[0] == 'top1'
gaits = [s[0] for s in node._sent_sports if s[0] in (ROBOT_SPORT_API_ID_STATICWALK, ROBOT_SPORT_API_ID_TROTRUN)]
assert gaits[-1] == ROBOT_SPORT_API_ID_STATICWALK, f"top1(계단/경사) 접근 중엔 STATICWALK여야 함: {gaits}"

node._odom(2.0, 0.0)  # top1 도착 -> line2로 전환(평지 복귀)
node.tick()
node.tick()  # 위와 동일한 이유로 한 틱 더
assert node.cur()[0] == 'line2'
gaits = [s[0] for s in node._sent_sports if s[0] in (ROBOT_SPORT_API_ID_STATICWALK, ROBOT_SPORT_API_ID_TROTRUN)]
assert gaits[-1] == ROBOT_SPORT_API_ID_TROTRUN, f"top1 통과 후 line2(평지)에선 TROTRUN으로 복귀해야 함: {gaits}"

# 중복 전송 방지(이미 같은 gait면 재전송 안 함) 확인
before = len(node._sent_sports)
node.tick()
after = len(node._sent_sports)
assert after == before, f"같은 구간에서 매 틱마다 gait를 재전송함(중복 전송 버그): {before} -> {after}"
print("4d) 'top' 구간 접근/통과에 따라 STATICWALK<->TROTRUN 자동 전환(중복 전송 없음) OK")
teardown(node)

# --- 4d2) 정지선 접근도 느린 걸음(정지선은 오르막 끝) — 출발 뒤 다음 평지 구간은 TROTRUN ---
clock = FakeClock()
node = make_node(clock, goals=[('line1', (1.0, 0.0)), ('stopline1', (2.0, 0.0)), ('line2', (3.0, 0.0))])
node._odom(0.1, 0.0); node.tick(); node.tick()
node._odom(1.0, 0.0); node.tick(); node.tick()
assert node.cur()[0] == 'stopline1'
gaits = [s[0] for s in node._sent_sports if s[0] in (ROBOT_SPORT_API_ID_STATICWALK, ROBOT_SPORT_API_ID_TROTRUN)]
assert gaits[-1] == ROBOT_SPORT_API_ID_STATICWALK, f"stopline1 접근 중엔 STATICWALK여야 함: {gaits}"
node._state(0.0); node._odom(2.0, 0.0); node.tick()    # 멈춘 채 도착 -> STOPLINE_HOLD
assert node.state == 'STOPLINE_HOLD', node.state
clock.advance(STOPLINE_HOLD_S + 0.1); node._odom(2.0, 0.0); node.tick()
assert node.state == 'STOPLINE_WAIT_SIGNAL', node.state
node._light('GREEN'); clock.advance(GREEN_CONFIRM_SECONDS + 0.1); node._light('GREEN'); node._odom(2.0, 0.0); node.tick()
assert node.cur()[0] == 'line2', node.cur()
node.tick()
gaits = [s[0] for s in node._sent_sports if s[0] in (ROBOT_SPORT_API_ID_STATICWALK, ROBOT_SPORT_API_ID_TROTRUN)]
assert gaits[-1] == ROBOT_SPORT_API_ID_TROTRUN, f"정지선 출발 뒤 line2 에선 TROTRUN: {gaits}"
print("4d2) stopline 접근 = STATICWALK, 녹색 출발 뒤 다음 구간 = TROTRUN OK")
teardown(node)

rcr.GAIT_MODE = 'classic'
# --- 4m) 11-05 기본: 전 구간 ClassicWalk(2049, {"data": true}) 한 번만 — top/stopline 에서도 안 바꿈, 주차에서 일어나면 다시 ---
clock = FakeClock(); node = make_node(clock, goals=[('top1', (1.0, 0.0)), ('line1', (2.0, 0.0)), ('exit', (3.0, 0.0))])
node._state(0.0); node._odom(0.0, 0.0); node.tick(); node.tick()
cw = [x for x in node._sent_sports if x[0] == rcr.ROBOT_SPORT_API_ID_CLASSICWALK]
assert len(cw) == 1 and cw[0][2] == '{"data": true}', node._sent_sports
node._odom(1.0, 0.0); node.tick(); node._odom(1.5, 0.0); node.tick(); node.tick()
assert not any(x[0] in (ROBOT_SPORT_API_ID_STATICWALK, ROBOT_SPORT_API_ID_TROTRUN) for x in node._sent_sports), "classic 인데 다른 걸음새를 보냄"
assert sum(x[0] == rcr.ROBOT_SPORT_API_ID_CLASSICWALK for x in node._sent_sports) == 1, "ClassicWalk 를 매번 다시 보냄"
print("4m) 전 구간 ClassicWalk 하나({\"data\": true}), 구간마다 안 바꿈 OK")
teardown(node)

# --- 4e) rabain/rabaout(라바콘 슬라럼 진입/탈출) -- ★ 11-05: 이제 정지+회전 후 통과
# (예전엔 정지 없이 바로 통과였는데, 사용자가 "콘에 안 부딪히려면 방향 맞추고 가야
# 한다"고 명시적으로 요구해서 바뀜) + cone_zone_active True/False 발행 ---
# ★ 11-06: 다음 점 방향이 ROTATE_SKIP_DEG(25°) 넘게 틀어졌을 때만 서서 돎 — 이 판은 90° 꺾이는 코스(회전 필요)
H = math.radians(90)
clock = FakeClock()
node = make_node(clock, goals=[('rabain', (1.0, 0.0)), ('rabaout', (1.0, 1.0)),
                                ('line', (2.0, 1.0))])
cone_states = []
node.pub_cone_zone_active.publish = lambda m: cone_states.append(m.data)

node._odom(0.1, 0.0)
node.tick()  # START_CHECK -> TO_GOAL(rabain)
assert node.cur()[0] == 'rabain'

node._state(0.3)  # 아직 움직이는 중 -- 회전 시작하면 안 됨(이동하며 돌지 말 것)
node._odom(1.0, 0.0)  # 위치상 rabain 도착(다음 점은 90° 왼쪽)
node.tick()
assert node.state == 'TO_GOAL', f"속도 안 죽었는데 회전 시작함(버그!): {node.state}"

node._state(0.0)  # 정지
node._odom(1.0, 0.0, yaw=0.0)
node.tick()
assert node.state == 'ROTATE', f"rabain 도착(90° 꺾임) 후 회전 상태 기대: {node.state}"
node.tick()
node._odom(1.0, 0.0, yaw=H); node.tick()  # 다 돎 → settle 시작
clock.advance(ROTATE_SETTLE_S + 0.1)
node._odom(1.0, 0.0, yaw=H)
node.tick()
assert node.cur()[0] == 'rabaout', f"rabain 회전 완료 후 다음 구간 전환 실패: {node.cur()[0]}"
assert cone_states and cone_states[-1] is True, f"rabain 통과 시 cone_zone_active=True 기대: {cone_states}"

node._state(0.0)
node._odom(1.0, 1.0, yaw=H)  # rabaout 도착(다음 점은 오른쪽 90°)
node.tick()
assert node.state == 'ROTATE', f"rabaout 도착 후 회전 상태 기대: {node.state}"
node.tick()
node._odom(1.0, 1.0, yaw=0.0); node.tick()
clock.advance(ROTATE_SETTLE_S + 0.1)
node._odom(1.0, 1.0, yaw=0.0)
node.tick()
assert node.cur()[0] == 'line', f"rabaout 회전 완료 후 다음 구간 전환 실패: {node.cur()[0]}"
assert cone_states[-1] is False, f"rabaout 통과 시 cone_zone_active=False 기대: {cone_states}"
print("4e) rabain/rabaout(90° 꺾임) 정지+회전 후 통과 + cone_zone_active True/False 전환 OK")
teardown(node)

# --- 4e2) ★ 11-06: 방향이 이미 맞으면(25° 안) 서지도 돌지도 않고 그대로 통과(지연 줄이기), 콘 구간 표시는 그대로 ---
clock = FakeClock()
node = make_node(clock, goals=[('rabain', (1.0, 0.0)), ('rabaout', (2.0, 0.1)), ('line', (3.0, 0.1))])
cone_states = []
node.pub_cone_zone_active.publish = lambda m: cone_states.append(m.data)
node._odom(0.1, 0.0); node.tick()
node._state(0.3); node._odom(1.0, 0.0, yaw=0.0); node.tick()     # 걷는 중 도착, 다음 점 방향 ~6°
assert node.state == 'TO_GOAL' and node.cur()[0] == 'rabaout', f"방향 맞으면 멈추지 않고 통과 기대: {node.state} {node.cur()[0]}"
assert cone_states and cone_states[-1] is True, cone_states
assert not any(abs(w) > 1e-6 for w in node._sent_cmd_vel), "회전 명령이 나감"
print(f"4e2) 회전 점이라도 다음 점 방향이 {rcr.ROTATE_SKIP_DEG:.0f}° 안이면 서지 않고 통과 OK")
teardown(node)

# --- 4f) ROTATE 실제 회전량 검증 -- 90도 틀어진 상태에서 cmd_vel 부호가 맞는 방향으로
# 나가고, odom이 그 방향으로 따라가면(흉내) 결국 수렴해서 다음 단계로 넘어가는지 ---
clock = FakeClock()
node = make_node(clock, goals=[('rabain', (0.0, 0.0)), ('rabaout', (1.0, 0.0))])
node._odom(0.0, 0.0, yaw=0.0)
node.tick()  # START_CHECK -> TO_GOAL(rabain)
node._state(0.0)
node._odom(0.0, 0.0, yaw=math.radians(90))  # 로봇이 90도 돌아간 채로 도착(목표는 0도 쪽)
node.tick()  # 도착 -> start_rotate()로 상태만 ROTATE로 전환(이 틱에선 아직 cmd_vel 안 나감)
assert node.state == 'ROTATE', node.state
node.tick()  # ROTATE 로직이 실제로 한 번 돌아야 cmd_vel이 나감
w0 = node._sent_cmd_vel[-1]
assert w0 < 0, f"90도(+방향) 틀어져 있으면 -방향(시계)으로 돌아야 하는데 부호가 틀림: {w0}"
# 폐루프 흉내: 매 틱 cmd_vel 방향으로 yaw를 조금씩 실제로 움직여서 수렴하는지 확인
yaw = math.radians(90)
for _ in range(200):
    clock.advance(0.2)
    w = node._sent_cmd_vel[-1]
    yaw += w * 0.2
    node._odom(0.0, 0.0, yaw=yaw)
    node.tick()
    if node.state != 'ROTATE':
        break
assert node.state != 'ROTATE', f"200틱 안에 회전이 안 끝남(수렴 안 함/버그): yaw={math.degrees(yaw):.1f}"
assert abs(rcr.ang_diff(0.0, yaw)) < ROTATE_DONE_THRESH + 0.05, \
    f"회전이 끝났는데 목표각도 근처가 아님: 최종 yaw={math.degrees(yaw):.1f}도"
print(f"4f) ROTATE: 90도 오차에서 올바른 방향으로 수렴해 완료 (최종 오차 {math.degrees(rcr.ang_diff(0.0, yaw)):.1f}도) OK")
teardown(node)

# --- 4g) STOPLINE_WAIT_SIGNAL 타임아웃 -- ★ 11-05: GREEN이 STOPLINE_GREEN_TIMEOUT_S
# 안에 확정 안 되면(내일 YOLO 모델이 아직 없거나 오탐/미탐 심한 경우 대비) 신호와
# 무관하게 그냥 출발(사용자가 직접 승인한 단순화된 방어전략) ---
clock = FakeClock()
node = make_node(clock)
advance_to_stopline(node, clock)
clock.advance(STOPLINE_HOLD_S + 0.1)
node._odom(2.0, 0.0)
node.tick()
assert node.state == 'STOPLINE_WAIT_SIGNAL', node.state

clock.advance(STOPLINE_GREEN_TIMEOUT_S - 0.1)  # 타임아웃 직전 -- 아직 출발하면 안 됨
node._odom(2.0, 0.0)
node.tick()
assert node.state == 'STOPLINE_WAIT_SIGNAL', "타임아웃 다 안 됐는데 벌써 출발함(버그!)"

clock.advance(0.2)  # 이제 타임아웃 넘음, 신호는 여전히 UNKNOWN(한번도 GREEN 안 들어옴)
node._odom(2.0, 0.0)
node.tick()
assert node.cur()[0] == 'parking', f"타임아웃 후 신호 무관하게 출발해야 하는데 안 됨: {node.state}, {node.cur()[0]}"
assert node._camera_states[-1] is False, "타임아웃 출발 시에도 camera_active=False로 꺼야 함"
print(f"4g) 정지선 신호 {STOPLINE_GREEN_TIMEOUT_S}초 타임아웃 -> 신호 무관 출발 OK")
teardown(node)

# --- 4h) junsu 11-05: 착석이 높이로 확인 안 돼도 재전송 상한 뒤 시간 기준으로 진행(주차 칸에 갇히지 않게) + 상대 높이 판정 ---
def to_park_sit_wait(node, clock, h0=0.30):
    node._state(0.0, body_height=h0); node._odom(0.0, 0.0); node.tick()          # START_CHECK -> TO_GOAL
    node._odom(1.0, 0.0, yaw=0.0); node.tick()                                    # parking 도착 -> PARK_PRE(회전 없음)
    assert node.state == 'PARK_PRE', node.state
    clock.advance(PARK_PRE_S + 0.05); node.tick()
    assert node.state == 'PARK_SIT_WAIT', node.state

clock = FakeClock()
node = make_node(clock, goals=[('parking', (1.0, 0.0)), ('exit', (2.0, 0.0))])
to_park_sit_wait(node, clock)
for k in range(SIT_RETRY_MAX - 1):
    clock.advance(SIT_SETTLE_S + 0.1); node.tick()
    assert node.state == 'PARK_SIT_WAIT', node.state
assert sum(x[0] == SIT_ID for x in node._sent_sports) == SIT_RETRY_MAX, node._sent_sports
clock.advance(SIT_SETTLE_S + 0.1); node.tick()
assert node.state == 'PARK_HOLD' and not node.sit_confirmed, f"재전송 상한 뒤 시간 기준 유지 기대: {node.state}"
clock.advance(PARK_HOLD_S + 0.1); node.tick()
assert node.state == 'PARK_RISE_WAIT', node.state
clock.advance(1.1); node.tick()              # 높이는 계속 선 높이(0.30) → 일어남으로 봄
assert node.state == 'PARK_REALIGN', node.state
clock.advance(rcr.PARK_REALIGN_WAIT_S + 0.1); node.tick()   # 재정렬 응답 없음 → 회전
clock.advance(rcr.GAIT_SETTLE_S); node.tick(); clock.advance(ROTATE_SETTLE_S + 0.1); node._odom(1.0, 0.0, yaw=0.0); node.tick()   # ★ 11-06: 주차 뒤엔 걸음새 자리 잡기(GAIT_SETTLE_S) 뒤 회전
assert node.cur()[0] == 'exit', (node.cur(), node.state)
teardown(node)
clock = FakeClock()
node = make_node(clock, goals=[('parking', (1.0, 0.0)), ('exit', (2.0, 0.0))])
to_park_sit_wait(node, clock, h0=0.32)
node._state(0.0, body_height=0.22)           # 문턱(0.15)보다 높지만 선 높이보다 0.10 낮음 → 앉음
node.tick()
assert node.state == 'PARK_HOLD' and node.sit_confirmed, f"상대 높이로 착석 판정 기대: {node.state}"
print(f"4h) Sit 미확인 {SIT_RETRY_MAX}번 재전송 뒤 시간 기준 진행 · 선 높이 대비 낮아짐으로 착석 판정 OK")
teardown(node)

# --- 4i) junsu 11-05: 출구 — 몸 전체가 Out Line 밖이면 완주. 도착했는데 모자라면 밀고 나감. 앞이 막히면 그 자리 정지 ---
def make_exit(clock):
    node = make_node(clock, goals=[('exit', (-0.83, 0.0))])
    lin = []
    node.pub_cmd_vel.publish = lambda m: lin.append((m.linear.x, m.linear.y))
    node._lin = lin
    rcr.EXIT_LINE_X = -0.54
    node._state(0.0); node._odom(0.0, 0.0); node.tick()
    assert node.state == 'TO_GOAL', node.state
    return node

clock = FakeClock(); node = make_exit(clock)
node._odom(-0.50, 0.0, yaw=0.0); node.tick()          # 0.33 m 앞 = 도착, 중심 x -0.50 → 뒤끝 선 안
assert node.state == 'EXIT_PUSH', f"선 통과 모자라면 EXIT_PUSH 기대: {node.state}"
assert node._sent_goals[-1] == (-0.50, 0.0), "밀기 전 ego 를 제자리에 세워야 함"
node.tick()
assert node._lin[-1][0] < -0.1, f"바깥(-x, 몸체 앞 0도라 뒤쪽)으로 밀어야 함: {node._lin[-1]}"
node._odom(-0.54 - EXIT_PASS_MARGIN - 0.01, 0.0, yaw=0.0); node.tick()
assert node.state == 'DONE' and node._lin[-1] == (0.0, 0.0), f"통과 후 DONE+정지 기대: {node.state} {node._lin[-1]}"
teardown(node)
clock = FakeClock(); node = make_exit(clock)
node._odom(-1.00, 0.0, yaw=3.14159); node.tick()       # 가는 도중 이미 몸 전체가 선 밖
assert node.state == 'DONE', f"TO_GOAL 중 선 통과면 바로 완주: {node.state}"
teardown(node)
import numpy as np
clock = FakeClock(); node = make_exit(clock)
node._odom(-0.50, 0.0, yaw=0.0); node.tick()
node.obs_xy = np.array([[-0.65, 0.0]]); node.obs_t = clock.t   # 바로 앞(0.15 m) 장애물
node.tick()
assert node.state == 'DONE', f"앞이 막히면 밀기 멈춤: {node.state}"
print("4i) exit: 선 통과 판정 · 도착 후 모자라면 밀고 나감 · 앞이 막히면 정지 OK")
teardown(node)

# --- 4j) junsu 11-05: 신호 판정이 끊기면(비전 노드 죽음) 마지막 GREEN 으로 출발하지 않음 ---
clock = FakeClock()
node = make_node(clock)
advance_to_stopline(node, clock)
clock.advance(STOPLINE_HOLD_S + 0.1); node._odom(2.0, 0.0); node.tick()
assert node.state == 'STOPLINE_WAIT_SIGNAL', node.state
node._light('GREEN')                       # 한 번 오고 끊김
clock.advance(GREEN_CONFIRM_SECONDS + 0.5); node._odom(2.0, 0.0); node.tick()
assert node.state == 'STOPLINE_WAIT_SIGNAL' and node.cur()[0] == 'stopline1', f"신호 끊겼는데 출발함(버그!): {node.state}"
print(f"4j) 신호 판정 {rcr.LIGHT_STALE_S}초 끊김 -> 초록 무효, 출발 안 함 OK")
teardown(node)

# --- 4k) 출발 정렬 T 가 있으면 출구 선 판정은 지도 좌표로(경로점은 오늘 좌표, 선은 지도 좌표) ---
import math as _m
rcr.ALIGN_T = (0.10, 0.0, _m.radians(5.0))      # 지도 = T·오늘
clock = FakeClock(); node = make_exit(clock)
# 지도 x = cos5°·x − sin5°·y + 0.10. 오늘 (-1.00, 0) → 지도 x ≈ -0.896 → 선(-0.54)에서 0.356 < 0.45 → 아직 통과 아님
node._odom(-1.00, 0.0, yaw=3.14159); node.tick()
assert node.state != 'DONE', f"T 반영하면 아직 선 안인데 완주로 봄: {node.state}"
node._odom(-1.12, 0.0, yaw=3.14159); node.tick()   # 지도 x ≈ -1.016 → 0.476 ≥ 0.45
assert node.state == 'DONE', f"T 반영해 통과인데 완주 아님: {node.state}"
rcr.ALIGN_T = (0.0, 0.0, 0.0)
print("4k) 출발 정렬 T 로 출구 선을 지도 좌표에서 판정 OK")
teardown(node)

# --- 4l) 위치 감시 STOP → 그 자리에 섬, 풀리면 출발, 안 풀려도 LOC_HOLD_MAX_S 뒤 계속(fail-open). 새 T → 남은 목표 다시 계산 ---
import json as _json
clock = FakeClock(); node = make_node(clock, goals=[('line1', (3.0, 0.0)), ('exit', (5.0, 0.0))])
node._state(0.0); node._odom(0.0, 0.0); node.tick(); assert node.state == 'TO_GOAL'
node.cb_loc(String(data=_json.dumps({'state': 'STOP'})))
assert node.state == 'LOC_HOLD', node.state
node.tick(); assert node.state == 'LOC_HOLD'
node.cb_loc(String(data=_json.dumps({'state': 'OK'}))); node.tick()
assert node.state == 'TO_GOAL', f"감시 풀렸는데 안 감: {node.state}"
clock.advance(11.0); node.cb_loc(String(data=_json.dumps({'state': 'STOP'}))); assert node.state == 'LOC_HOLD'
clock.advance(rcr.LOC_HOLD_MAX_S + 0.1); node.tick()
assert node.state == 'TO_GOAL', f"STOP 이 안 풀려도 {rcr.LOC_HOLD_MAX_S}초 뒤엔 계속 가야 함: {node.state}"
node.cb_loc(String(data=_json.dumps({'state': 'STOP'}))); assert node.state == 'TO_GOAL', "10초 안 같은 STOP 으로 또 섬"
rcr.ALIGN_T = (0.0, 0.0, 0.0)
node.cb_align_T(String(data=_json.dumps({'T': [0.10, 0.0, 0.0], 'ok': True, 'reason': 'reanchor'})))
assert abs(node.goals[0][1][0] - 2.90) < 1e-6 and abs(node.goals[1][1][0] - 4.90) < 1e-6, node.goals
rcr.ALIGN_T = (0.0, 0.0, 0.0)
print(f"4l) 위치 감시 STOP 정지·해제 출발·{rcr.LOC_HOLD_MAX_S:.0f}초 fail-open·재정렬 목표 다시 계산 OK")
teardown(node)

# --- 4n) 11-05 review ego: 벽에 붙으면 콘을 덜 피함(1), 막히면 콘 무시(2), 빠져나오면 되돌림 — 벽 설정은 안 바꿈 ---
rcr.EGO_REVIEW = 1.0
clock = FakeClock(); node = make_node(clock, goals=[('line1', (5.0, 0.0)), ('exit', (8.0, 0.0))])
node.wall_cells = {(int(x / 0.05), int(0.20 / 0.05)) for x in [i * 0.05 for i in range(-20, 200)]}   # y=0.20 의 벽(로봇 중심 y=0 → 0.2 m)
node._state(0.0); node._odom(0.0, 0.0); node.tick(); assert node.state == 'TO_GOAL'
node.tick(); assert node.cone_level == 0
clock.advance(0.6); node._odom(0.05, 0.0); node.tick()
assert node.cone_level == 1, f"벽 0.2 m 0.5초 넘었는데 콘 단계 {node.cone_level}"
for _ in range(25):                      # 4초 넘게 제자리 → 막힘 → 콘 무시
    clock.advance(0.2); node._odom(0.05, 0.0); node.tick()
    if node.cone_level == 2: break
assert node.cone_level == 2, f"막혔는데 콘 무시 안 함: {node.cone_level}"
assert rcr.CONE_LEVELS[2]['cones.inflate_radius'] == 0.0 and rcr.CONE_LEVELS[2]['cones.weight'] == 0.0
assert all(not k.startswith('inflate') for lv in rcr.CONE_LEVELS for k in lv), "벽 여유(inflate_radius)는 건드리면 안 됨"
node.state = 'TO_GOAL'; node.wall_cells = set(); node.wall_tree = None
clock.advance(0.2); node._odom(0.8, -0.3); node.tick()
assert node.cone_level == 1, f"0.6 m 빠져나왔는데 {node.cone_level}"
for _ in range(15):
    clock.advance(0.2); node._odom(0.9, -0.3); node.tick()
assert node.cone_level == 0, f"벽에서 멀어진 지 2초 넘었는데 {node.cone_level}"
rcr.EGO_REVIEW = 0.0
print("4n) 벽에 붙음 → 콘 덜 피함 · 막힘 → 콘 무시 · 빠져나옴 → 되돌림(벽 여유는 그대로) OK")
teardown(node)

# --- 4o) 11-05 콘 무시 뒤에도 오래 못 가면 경로 따라가기(ego 세움, 러너가 직접, 벽만 봄) → 목표 근처에서 보통 주행 ---
rcr.EGO_REVIEW = 1.0
clock = FakeClock(); node = make_node(clock, goals=[('rabaout', (3.0, 0.0)), ('exit', (6.0, 0.0))])
lin = []; node.pub_cmd_vel.publish = lambda m: lin.append((m.linear.x, m.linear.y, m.angular.z))
node._state(0.0); node._odom(0.0, 0.0, yaw=0.0); node.tick(); assert node.state == 'TO_GOAL'
node.set_cone_level(2, clock.t, '시험')
for _ in range(60):
    clock.advance(0.2); node._odom(0.0, 0.0, yaw=0.0); node.tick()
    if node.state == 'ROUTE_FOLLOW': break
assert node.state == 'ROUTE_FOLLOW', f"콘 무시 뒤 {rcr.CONE_GIVEUP_S}초 못 갔는데 경로 따라가기 안 함: {node.state}"
assert node._sent_goals[-1] == (0.0, 0.0), "경로 따라가기 전 ego 를 세워야 함"
clock.advance(0.2); node.tick()
assert lin[-1][0] > 0.2 and abs(lin[-1][2]) < 0.05, f"목표(+x) 쪽으로 직진해야 함: {lin[-1]}"
node.wall_cells = {(int(0.15 / 0.05), j) for j in range(-10, 11)}; node.wall_tree = None   # x=0.15 앞에 벽
clock.advance(0.2); node.tick()
assert lin[-1][0] <= 0.0, f"앞이 벽인데 앞으로 감: {lin[-1]}"
node.wall_cells = set(); node.wall_tree = None
node._odom(2.7, 0.0, yaw=0.0); clock.advance(0.2); node.tick()
assert node.state == 'TO_GOAL' and lin[-1] == (0.0, 0.0, 0.0), f"목표 근처에서 보통 주행으로 안 돌아감: {node.state} {lin[-1]}"
rcr.EGO_REVIEW = 0.0
print(f"4o) 콘 무시 {rcr.CONE_GIVEUP_S:.0f}초 뒤 경로 따라가기 · 목표로 직진 · 앞 벽이면 안 감 · 목표 근처 보통 주행 OK")
teardown(node)

# --- 4p) 11-05 오후: 빨강이 보이는 동안은 STOPLINE_GREEN_TIMEOUT_S 넘어도 안 떠남(최대 STOPLINE_RED_MAX_S) ---
clock = FakeClock(); node = make_node(clock)
advance_to_stopline(node, clock)
clock.advance(STOPLINE_HOLD_S + 0.1); node._odom(2.0, 0.0); node.tick(); assert node.state == 'STOPLINE_WAIT_SIGNAL'
t = 0.0
while t < rcr.STOPLINE_GREEN_TIMEOUT_S + 5:
    node._light('RED'); clock.advance(0.5); t += 0.5; node._odom(2.0, 0.0); node.tick()
assert node.state == 'STOPLINE_WAIT_SIGNAL', f"빨강 보는 중인데 {rcr.STOPLINE_GREEN_TIMEOUT_S}초에 떠남: {node.state}"
while t < rcr.STOPLINE_RED_MAX_S + 1:
    node._light('RED'); clock.advance(0.5); t += 0.5; node._odom(2.0, 0.0); node.tick()
assert node.cur()[0] == 'parking', f"빨강이 {rcr.STOPLINE_RED_MAX_S}초 넘게 계속이면 떠나야 함: {node.cur()}"
print(f"4p) 빨강 보이는 동안 대기(UNKNOWN {rcr.STOPLINE_GREEN_TIMEOUT_S:.0f}초 · 빨강 최대 {rcr.STOPLINE_RED_MAX_S:.0f}초) OK")
teardown(node)

# --- 4q) 11-05 오후: 핀 구간 = 앞 점 + 연속 핀을 회귀 곡선(스무딩 스플라인)으로, ego 세우고 내다보기 추종 → 끝나면 보통 주행 ---
C, S_ = RealCourseRunner.fit_pin_path([(1.0, 0.0), (2.0, 1.0), (3.0, 1.0), (4.0, 0.3)])
for px, py in [(1.0, 0.0), (2.0, 1.0), (3.0, 1.0), (4.0, 0.3)]:
    dmin = float(np.min(np.hypot(C[:, 0] - px, C[:, 1] - py)))
    assert dmin < 0.06, f"회귀 곡선이 핀 ({px},{py}) 에서 {dmin:.3f} m 벗어남"
ang = np.unwrap(np.arctan2(np.diff(C[:, 1]), np.diff(C[:, 0])))
assert np.max(np.abs(np.diff(ang))) < 0.2, "곡선이 꺾임(매끄럽지 않음)"
C2, _ = RealCourseRunner.fit_pin_path([(0.0, 0.0), (2.0, 0.0)]); assert np.allclose(C2[:, 1], 0.0), "점 2개는 직선"
clock = FakeClock(); node = make_node(clock, goals=[('line1', (1.0, 0.0)), ('pin1', (2.0, 1.0)), ('pin2', (3.0, 1.0)), ('exit', (5.0, 1.0))])
lin = []; node.pub_cmd_vel.publish = lambda m: lin.append((m.linear.x, m.linear.y, m.angular.z))
node._state(0.0); node._odom(0.0, 0.0); node.tick(); node._odom(1.0, 0.0); node.tick()
assert node.cur()[0] == 'pin1', node.cur()
node.tick(); assert node.state == 'PIN_TRACK' and node.pin_last == 2, (node.state, getattr(node, 'pin_last', None))
assert node._sent_goals[-1] == (1.0, 0.0), "핀 구간 시작 때 ego 를 세워야 함"
node._odom(1.0, 0.0, yaw=math.radians(45)); node.tick()
assert lin[-1][0] > 0.15, f"곡선 방향(≈45°)을 보고 있으면 앞으로 가야 함: {lin[-1]}"
node._odom(1.2, 0.6, yaw=math.radians(45)); node.tick()      # 곡선 왼쪽으로 벗어남 → 오른쪽으로 꺾음
assert lin[-1][2] < 0, f"곡선 왼쪽으로 벗어났으면 오른쪽으로 꺾어야 함: {lin[-1]}"
node._odom(2.0, 0.97, yaw=0.0); node.tick(); assert node.state == 'PIN_TRACK', "중간 핀에서는 멈추지 않고 곡선을 계속 따라감"
node._odom(2.95, 1.0, yaw=0.0); node.tick()
assert node.cur()[0] == 'exit' and node.state == 'TO_GOAL', (node.cur(), node.state)
print("4q) 핀 구간: 회귀 곡선(핀 6 cm 안·매끄러움) · ego 세움 · 내다보기 추종 · 중간 핀 안 멈춤 · 끝나면 보통 주행 OK")
teardown(node)

# --- 4q2) 11-06 핀 구간 ego 먼저 → 첫 실패(제자리 2초·ego 침묵)면 지금 자리부터 핀 경로, 성공하면 핀 건너 통과 ---
rcr.PIN_EGO_FIRST = True
G4 = [('line1', (1.0, 0.0)), ('pin1', (2.0, 1.0)), ('pin2', (3.0, 1.0)), ('exit', (5.0, 1.0))]
def to_pin_ego():
    clock = FakeClock(); node = make_node(clock, goals=G4)
    node._state(0.0); node._odom(0.0, 0.0); node.tick(); node._odom(1.0, 0.0); node.tick(); node.tick()
    assert node.state == 'PIN_EGO', node.state
    assert node._sent_goals[-1] == (3.0, 1.0), f"ego 목표 = 마지막 핀: {node._sent_goals[-1]}"
    return clock, node
clock, node = to_pin_ego(); node.cb_traj(None)                     # A) ego 로 끝까지 → 핀 건너 다음 점
clock.advance(0.5); node._odom(2.0, 1.0); node.cb_traj(None); node.tick(); assert node.state == 'PIN_EGO'
clock.advance(0.5); node._odom(2.8, 1.0); node.cb_traj(None); node.tick()
assert node.cur()[0] == 'exit' and node.state == 'TO_GOAL', f"ego 로 핀 구간 통과 → exit: {node.cur()} {node.state}"
teardown(node)
clock, node = to_pin_ego(); node.cb_traj(None)                     # B) ego 가 길은 내는데 2초 제자리 → 핀
clock.advance(0.5); node._odom(1.6, 0.6); node.cb_traj(None); node.tick()
for _ in range(5):
    clock.advance(0.5); node.cb_traj(None); node.tick()
assert node.state == 'PIN_TRACK' and node.pin_ego_failed.get(1), f"제자리 2초 → 핀 경로: {node.state}"
k = node.pin_ci; dd_ = np.hypot(*(node.pin_path - np.array([1.6, 0.6])).T)
assert k >= int(np.argmin(dd_)) and dd_[k] <= dd_.min() + 0.051 and k > 10, f"지금 자리(최근접 +5 cm 안 가장 앞)부터 이어서: ci {k} {node.pin_path[k]} 최근접 {dd_.min():.3f}"
teardown(node)
clock, node = to_pin_ego(); node.cb_traj(None)                     # C) 움직이긴 하는데 ego 궤적이 끊김(1.5초) → 핀
for i in range(4):
    clock.advance(0.5); node._odom(1.1 + 0.1 * i, 0.1 + 0.1 * i); node.tick()
assert node.state == 'PIN_TRACK', f"ego 침묵 → 핀 경로: {node.state}"
teardown(node)
rcr.PIN_EGO_FIRST = False
# 4q3) 핀 따라가다 2.5초 막힘 → 0.25 m/s 로 빠져나옴(장애물 반대쪽) · 회전은 진행으로 봄 · 3번 넘으면 핀 그만두고 다음 점
clock = FakeClock(); node = make_node(clock, goals=G4); cmds = []; node.pub_cmd_vel.publish = lambda m: cmds.append((m.linear.x, m.linear.y, m.angular.z))
node._state(0.0); node._odom(0.0, 0.0); node.tick(); node._odom(1.0, 0.0); node.tick(); node.tick(); assert node.state == 'PIN_TRACK', node.state
for i in range(4):                                      # 제자리 회전 2초(10°씩) → 막힘 아님
    clock.advance(0.5); node._odom(1.0, 0.0, yaw=math.radians(15 * (i + 1))); node.tick()
assert node.pin_stalls == 0, f"회전 중은 막힘 아님: {node.pin_stalls}"
for _ in range(6):
    clock.advance(0.5); node.tick()
assert node.pin_stalls == 1 and math.hypot(cmds[-1][0], cmds[-1][1]) > 0.2, f"2.5초 막힘 → 0.25 m/s 빠져나옴: {node.pin_stalls} {cmds[-1]}"
for _ in range(30):
    clock.advance(0.5); node.tick()
    if node.state != 'PIN_TRACK': break
assert node.cur()[0] == 'exit', f"3번 넘게 막히면 핀 그만두고 다음 점: {node.cur()} {node.state}"
teardown(node)
print("4q2) 핀 구간 ego 먼저: 성공이면 통과 · 제자리 2초/ego 침묵이면 지금 자리부터 핀 경로 OK")

# --- 4r) 11-05 오후 벽 추종 구간: go2_wall_patrol RANSAC(콘 이상치 버림) · 옆벽 거리·나란함 · 오른쪽 벽 · 구간 끝 ---
rng_ = np.random.default_rng(0)
def wall_pts(y0, yaw_deg=0.0, n=120, cones=True):
    x = np.linspace(-0.7, 1.1, n); P = np.c_[x, np.full(n, y0) + rng_.normal(0, 0.008, n)]
    if cones: P = np.r_[P, [[0.6, y0 * 0.55], [0.62, y0 * 0.55], [0.58, y0 * 0.5]]]   # 벽 앞 콘 점(이상치)
    t = math.radians(-yaw_deg); c_, s_ = math.cos(t), math.sin(t)
    return P @ np.array([[c_, s_], [-s_, c_]])
ln = rcr.ransac_wall(wall_pts(0.6), False, 0.5); assert ln and abs(ln['dist'] - 0.6) < 0.02 and abs(math.atan2(-ln['a'], ln['b'])) < 0.03, ln
ln = rcr.ransac_wall(wall_pts(0.6, 10), False, 0.5); assert ln and abs(math.degrees(math.atan2(-ln['a'], ln['b'])) + 10) < 1.5, ln
fr = rcr.ransac_wall(np.c_[np.full(40, 1.0), np.linspace(-0.25, 0.25, 40)], True); assert fr and abs(fr['dist'] - 1.0) < 0.01, fr
for side, wy in ((1, 0.5), (-1, -0.5)):
    clock = FakeClock(); node = make_node(clock, goals=[('line1', (0.3, 0.0)), ('wall1', (3.0, 0.0)), ('exit', (5.0, 0.0))])
    lin = []; node.pub_cmd_vel.publish = lambda m: lin.append((m.linear.x, m.linear.y, m.angular.z))
    node.wall_cells = {(int(x / 0.05), int(wy / 0.05)) for x in [i * 0.05 for i in range(-20, 120)]}
    node._state(0.0); node._odom(0.0, 0.0); node.tick(); node._odom(0.3, 0.0); node.tick(); node.tick()
    assert node.state == 'WALL_TRACK' and node.wseg['side'] == side and abs(node.wseg['target'] - 0.5) < 0.06, (node.state, node.wseg)
    node.scan_xy = wall_pts(0.6 * side); node.scan_t = rcr.time.monotonic(); node._odom(0.5, -0.1 * side, yaw=0.0); node.tick()
    assert lin[-1][0] > 0.2 and abs(lin[-1][1] - 0.08 * side) < 0.03 and abs(lin[-1][2]) < 0.06, (side, lin[-1])
    node.scan_xy = wall_pts(0.5 * side, 10); node.scan_t = rcr.time.monotonic(); node.tick()
    assert lin[-1][2] < -0.15, f"로봇이 왼쪽으로 10° 틀어졌으면 오른쪽으로 돌아 벽과 나란히: {lin[-1]}"
    node._odom(2.97, 0.0); node.tick()
    assert node.cur()[0] == 'exit' and node.state == 'TO_GOAL', (node.cur(), node.state)
    teardown(node)
print("4r) 벽 추종: RANSAC(콘 이상치 무시)·옆벽 거리 유지·벽과 나란히(왼쪽/오른쪽 벽)·구간 끝 OK")

# --- 4s) 같은 자리에 찍은 점(parking 바로 뒤 wall 등)이 있어도 주차 뒤 회전은 0.3 m 넘게 떨어진 점을 향함 ---
clock = FakeClock(); node = make_node(clock, goals=[('parking', (1.0, 0.0)), ('wall1', (1.0, 0.01)), ('line7', (1.0, 2.0)), ('exit', (3.0, 2.0))])
node._state(0.0); node._odom(0.0, 0.0); node.tick(); node._odom(1.0, 0.0, yaw=0.0); node.tick()
assert node.state == 'PARK_PRE', node.state
node.post_park = True; node.start_rotate()
assert node.state == 'ROTATE' and abs(math.degrees(node.rotate_target_yaw) - 90) < 1, (node.state, math.degrees(node.rotate_target_yaw))
print("4s) 주차 뒤 회전: 바로 옆 점 건너뛰고 0.3 m 넘는 점(line7) 쪽 OK")
teardown(node)

# --- 4t) 11-05 예선(넘어짐): 기울기 50° 넘으면 멈춤 → 다시 서서 2초 → BalanceStand + 넓은 재정렬 → 가던 목표 ---
clock = FakeClock(); node = make_node(clock, goals=[('line1', (3.0, 0.0)), ('exit', (6.0, 0.0))])
rq = []; node.pub_realign.publish = lambda m: rq.append(m.data)
def st_(h, roll=0.0, pitch=0.0):
    m = SportModeState(); m.velocity = [0.0, 0.0, 0.0]; m.body_height = h; m.imu_state.rpy = [roll, pitch, 0.0]; node.cb_state(m)
st_(0.30); node._odom(0.0, 0.0); node.tick(); node._odom(1.0, 0.0); node.tick(); assert node.state == 'TO_GOAL'
st_(0.10, pitch=1.2); node.tick()
assert node.state == 'FALLEN' and node._sent_goals[-1] == (1.0, 0.0), f"넘어졌는데 안 멈춤: {node.state}"
clock.advance(5.0); node.tick(); assert node.state == 'FALLEN', "누워 있는 동안은 그대로"
st_(0.30); node.tick(); clock.advance(rcr.FALL_STABLE_S + 0.1); node.tick()
assert node.state == 'FALL_REALIGN' and rq and '"fall"' in rq[-1] and node._sent_sports[-1][0] == rcr.BALANCESTAND_ID, (node.state, rq)
node.cb_align_T(String(data='{"T": [0.0, 0.0, 0.0], "ok": true, "reason": "realign"}')); node.tick()
assert node.state == 'TO_GOAL' and node.cur()[0] == 'line1', (node.state, node.cur())
teardown(node)
# 같은 목표에서 물러나기 6번 → 재정렬 요청, 12번 → STUCK_HOLD(멈춤)
clock = FakeClock(); node = make_node(clock, goals=[('top1', (5.0, 0.0)), ('exit', (9.0, 0.0))])
rq = []; node.pub_realign.publish = lambda m: rq.append(m.data)
node._state(0.0); node._odom(0.0, 0.0); node.tick(); assert node.state == 'TO_GOAL'
for i in range(20):
    node.obs_xy = np.array([[0.0, 0.22]]); node.obs_t = clock.t
    for _ in range(50):
        node.obs_t = clock.t
        clock.advance(0.2); node._odom(0.0, 0.0); node.tick()
        if node.state != 'TO_GOAL': break
    if node.state == 'STUCK_HOLD': break
    clock.advance(rcr.BACKOFF_S + 0.1); node.tick()
assert node.state == 'STUCK_HOLD' and node.backoffs_here == rcr.STUCK_HOLD_AFTER, (node.state, node.backoffs_here)
assert any('stuck_top1' in r for r in rq), rq
print(f"4t) 넘어짐: 멈춤 → 다시 서면 BalanceStand·넓은 재정렬 → 이어감 · 물러나기 {rcr.STUCK_REALIGN_AFTER}번 재정렬 요청, {rcr.STUCK_HOLD_AFTER}번 멈춤 OK")
teardown(node)

# --- 4u) 뒤쪽 벽 기억: 앞이 막혀 물러날 때, 지금 안 보이는(XT16 가림) 뒤 벽 칸도 보고 그쪽으로는 안 감 ---
clock = FakeClock(); node = make_node(clock, goals=[('top1', (5.0, 0.0)), ('exit', (9.0, 0.0))])
lin = []; node.pub_cmd_vel.publish = lambda m: lin.append((m.linear.x, m.linear.y))
node._state(0.0); node._odom(0.0, 0.0, yaw=0.0); node.tick()
node.wall_cells = {(int(-0.45 / 0.05), j) for j in range(-6, 7)}; node.wall_tree = None     # 뒤 0.45 m 벽(기억)
for _ in range(40):
    node.obs_xy = np.array([[0.30, 0.0]]); node.obs_t = clock.t                     # 앞 0.30 m 장애물(지금 스캔)
    clock.advance(0.2); node._odom(0.0, 0.0, yaw=0.0); node.tick()
    if node.state == 'BACKOFF': break
assert node.state == 'BACKOFF', node.state
vx, vy = lin[-1]
assert -0.05 < vx < 0.05 and abs(vy) > 0.1, f"뒤(기억 벽)도 앞(장애물)도 아닌 옆으로 물러나야 함: vx {vx:+.2f} vy {vy:+.2f}"
print(f"4u) 뒤쪽 벽 기억: 앞 막힘 물러나기가 뒤 벽 쪽으로 안 감(vx {vx:+.2f}, vy {vy:+.2f}) OK")
teardown(node)

# --- 4v) 지나가는 점(line*)은 PASS_SWITCH(0.8 m) 앞에서 미리 다음 점 — 모서리를 곡선으로(멈추지 않음) ---
clock = FakeClock(); node = make_node(clock, goals=[('line1', (2.0, 0.0)), ('line2', (2.0, 2.0)), ('exit', (4.0, 2.0))])
node._state(0.3); node._odom(0.0, 0.0); node.tick(); node._odom(1.0, 0.0); node.tick(); assert node.cur()[0] == 'line1'
node._odom(1.3, 0.0); node.tick()
assert node.cur()[0] == 'line2' and node.state == 'TO_GOAL', f"0.7 m 앞(< {rcr.PASS_SWITCH})인데 다음 점으로 안 넘어감: {node.cur()}"
print(f"4v) 지나가는 점은 {rcr.PASS_SWITCH} m 앞에서 다음 점(멈추지 않고 곡선) OK")
teardown(node)

# --- 4w) ★ 11-06: 콘 구간(rabain 다음 ~ rabaout)에선 목표 보정(SNAP) 안 함 — 촘촘한 슬라럼 점이 콘 건너편으로 옮겨지지 않게 ---
clock = FakeClock()
node = make_node(clock, goals=[('line1', (1.0, 0.0)), ('rabain', (2.0, 0.0)), ('line2', (2.5, 0.0)), ('rabaout', (3.0, 0.0)), ('exit', (4.0, 0.0))])
node.wall_cells = {(int(np.floor(x / 0.05)), int(np.floor(0.15 / 0.05))) for x in np.arange(0.0, 4.0, 0.05)}   # y=0.15 에 벽(점들이 벽 0.15 m 옆)
node.idx = 0; tx, ty = node.target(); assert abs(ty) > 0.05, f"콘 구간 밖은 보정 기대: {(tx, ty)}"
node.idx = 2; assert 2 in node.cone_zone_idx; tx, ty = node.target(); assert (tx, ty) == (2.5, 0.0), f"콘 구간 안은 보정 안 함 기대: {(tx, ty)}"
print("4w) 콘 구간에선 목표 보정 안 함(밖은 그대로 보정) OK")
teardown(node)

# --- 4x) ★ 11-06 사용자: 콘 구간에서 콘을 못 알아보면(2초 검출 없음) 출구로 직진 대신 슬라럼 대체 길(junk) — 실제 움직인 만큼만 앞으로, 끝나면 rabaout 도착 처리 ---
_sm0 = rcr.SLALOM_MODE; rcr.SLALOM_MODE = 'auto'   # ★ 기본은 path(11-06) — auto 동작 자체를 시험
clock = FakeClock()
rcr.SLALOM_PATH_MAP = [(1.0, 0.0), (1.5, 0.4), (2.0, -0.4), (2.5, 0.4), (3.0, 0.0)]
node = make_node(clock, goals=[('rabain', (1.0, 0.0)), ('rabaout', (3.0, 0.0)), ('line', (4.0, 0.0))])
node._odom(0.1, 0.0); node.tick()
node._state(0.3); node._odom(1.0, 0.0, yaw=0.0); node.tick()          # rabain 통과(방향 맞음) → 콘 구간
assert node.cur()[0] == 'rabaout' and node.state == 'TO_GOAL', (node.cur(), node.state)
node.tick(); clock.advance(1.0); node._odom(1.05, 0.0); node.tick()
assert node.state == 'TO_GOAL', f"2초 전엔 그대로 ego 기대: {node.state}"
clock.advance(1.2); node._odom(1.1, 0.0); node.tick()
assert node.state == 'PIN_TRACK' and node.pin_mode == 'slalom', f"콘 검출 없음 → 슬라럼 대체 길 기대: {node.state} {node.pin_mode}"
P = node.pin_path
for q in P[::5]:
    clock.advance(0.2); node._odom(float(q[0]), float(q[1])); node.tick()
    if node.state != 'PIN_TRACK':
        break
node._odom(3.0, 0.0); node.tick()
assert node.state in ('TO_GOAL', 'ROTATE') and node.cur()[0] in ('rabaout', 'line'), f"대체 길 끝 → rabaout 도착 처리(방향 맞으면 통과) 기대: {node.state} {node.cur()}"
print("4x) 콘 구간 2초 검출 없음 → 슬라럼 대체 길(꺾은선, 움직인 만큼만) → 끝나면 rabaout 도착 처리 OK")
teardown(node)

# --- 4x2) 콘을 보고 있으면(검출 있음) ego 그대로 — 막혀서 포기할 때만 대체 길 ---
clock = FakeClock()
node = make_node(clock, goals=[('rabain', (1.0, 0.0)), ('rabaout', (3.0, 0.0)), ('line', (4.0, 0.0))])
node._odom(0.1, 0.0); node.tick(); node._state(0.3); node._odom(1.0, 0.0, yaw=0.0); node.tick()
for k in range(30):
    node.cone_seen_t = clock.t; clock.advance(0.1); node._odom(1.0 + 0.01 * k, 0.0); node.tick()
assert node.state == 'TO_GOAL', f"콘을 보는 중엔 ego 기대: {node.state}"
print("4x2) 콘 검출 있으면 ego 그대로 OK")
teardown(node); rcr.SLALOM_PATH_MAP = []; rcr.SLALOM_MODE = _sm0

# --- 4y) ★ 11-06 사용자 '경로 지정된 순서로': 다른 통로로 0.8 m 넘게 2초 빠지면 지나온 길을 되짚어 경로로 ---
clock = FakeClock()
node = make_node(clock, goals=[('line1', (2.0, 0.0)), ('line2', (4.0, 0.0)), ('exit', (5.0, 0.0))])
node._odom(0.1, 0.0); node.tick()
node._state(0.3); node._odom(2.0, 0.0); node.tick()            # line1 → line2 구간
for y_ in (0.0, 0.2, 0.4, 0.6, 0.9, 1.1, 1.3):                  # 위쪽 다른 통로로 빠짐
    clock.advance(0.5); node._odom(2.5, y_, yaw=math.radians(90)); node.tick()
clock.advance(rcr.OFF_ROUTE_S + 0.1); node._odom(2.5, 1.3, yaw=math.radians(90)); node.tick()
assert node.state == 'PIN_TRACK' and node.pin_mode == 'rejoin', f"경로 이탈 → 되짚기 기대: {node.state}"
end = node.pin_path[-1]; assert abs(end[1]) < 0.45, f"되짚기 끝은 경로 가까이 기대: {end}"
for q in node.pin_path[::4]:
    clock.advance(0.2); node._odom(float(q[0]), float(q[1])); node.tick()
    if node.state != 'PIN_TRACK':
        break
assert node.state == 'TO_GOAL' and node.cur()[0] == 'line2', (node.state, node.cur())
print("4y) 다른 통로로 빠지면 지나온 길 되짚어 경로로 → 같은 목표로 보통 주행 OK")
teardown(node)

# --- 4z) ★ 11-06 실주행(11-05 16:54) 회전 4번 모두 목표 5~15° 근처에서 넘었다 돌아왔다 하다 10초 타임아웃 → 넘으면(부호 바뀜) 또는 15° 안 1초면 완료 ---
clock = FakeClock()
node = make_node(clock, goals=[('rabain', (0.0, 0.0)), ('rabaout', (1.0, 0.0))])
node._odom(0.0, 0.0, yaw=0.0); node.tick(); node._state(0.0)
node._odom(0.0, 0.0, yaw=math.radians(90)); node.tick()               # 90° 틀어져 도착 → 회전
assert node.state == 'ROTATE', node.state
for deg in (60, 30, 12, -8):                                            # 목표(0°)를 넘어 -8° 로
    clock.advance(0.3); node._odom(0.0, 0.0, yaw=math.radians(deg)); node.tick()
assert node.cur()[0] == 'rabaout' and node.state != 'ROTATE', f"목표를 넘으면 완료 기대: {node.state}"
teardown(node)
clock = FakeClock()
node = make_node(clock, goals=[('rabain', (0.0, 0.0)), ('rabaout', (1.0, 0.0))])
node._odom(0.0, 0.0, yaw=0.0); node.tick(); node._state(0.0)
node._odom(0.0, 0.0, yaw=math.radians(90)); node.tick()
for k in range(8):                                                      # 12° 근처에서 맴돎(넘지 않음)
    clock.advance(0.2); node._odom(0.0, 0.0, yaw=math.radians(12 - (k % 2))); node.tick()
assert node.cur()[0] == 'rabaout', f"15° 안 1초면 완료 기대: {node.state}"
print(f"4z) 회전: 목표를 넘거나 {rcr.ROTATE_NEAR_DEG:.0f}° 안 {rcr.ROTATE_NEAR_S:.0f}초면 완료(타임아웃까지 맴돌지 않음) OK")
teardown(node)

# --- 4za) ★ 11-06 '다이나믹한 판단': ego 목표 = 경로선 위 로봇보다 CARROT_M 앞(먼 경로 점 통째로 X) — 다음 일하는 점(정지선)은 안 넘음 ---
_carrot0 = rcr.CARROT_M; rcr.CARROT_M = 1.0   # 기본은 끔(실기 전) — 기능 자체를 시험
clock = FakeClock()
node = make_node(clock, goals=[('line1', (2.0, 0.0)), ('line2', (2.0, 2.0)), ('stopline1', (2.0, 2.5)), ('exit', (5.0, 2.5))])
node._odom(0.1, 0.0); node.tick()
node._state(0.3); node._odom(0.5, 0.1); clock.advance(1.1); node.tick()
gx, gy = node._sent_goals[-1]; assert abs(gx - (0.5 + rcr.CARROT_M)) < 0.05 and abs(gy) < 0.05, f"경로선 위 1 m 앞 기대: {(gx, gy)}"
node._odom(1.6, 0.1); clock.advance(1.1); node.tick()
gx, gy = node._sent_goals[-1]; assert abs(gx - 2.0) < 0.05 and 0.4 < gy < 0.8, f"모퉁이 지나 다음 선분 위 기대: {(gx, gy)}"
node._odom(2.0, 2.0); node.tick(); node._odom(2.0, 2.1); clock.advance(1.1); node.tick()
gx, gy = node._sent_goals[-1]; assert abs(gy - 2.5) < 0.05, f"정지선(일하는 점)은 안 넘음 기대: {(gx, gy)}"
print(f"4za) ego 목표 = 경로선 위 {rcr.CARROT_M} m 앞(모퉁이 따라감, 정지선 안 넘음) OK")
teardown(node)
rcr.CARROT_M = _carrot0

# --- 5) send_sport가 identity.id(0 아님)와 parameter='{}' 채우는지 ---
clock = FakeClock()
node = make_node(clock)
node.send_sport(SIT_ID, 'Sit')
api_id, ident_id, param = node._sent_sports[-1]
assert api_id == SIT_ID
assert ident_id != 0, "identity.id가 0 그대로 나감(로봇이 무시할 위험 -- 수정 전 상태로 되돌아감!)"
assert param == '{}', f"parameter가 빈 문자열 그대로 나감: {param!r}"
print(f"5) send_sport: identity.id={ident_id}(0 아님), parameter={param!r} OK")
teardown(node)

print("\n전체 PASS")
