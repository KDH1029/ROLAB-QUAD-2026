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
sys.path.insert(0, '/tmp')
import rclpy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from unitree_go.msg import SportModeState
import real_course_runner as rcr
from real_course_runner import (RealCourseRunner, STOPLINE_HOLD_S, GREEN_CONFIRM_SECONDS,
                                 SIT_SETTLE_S, PARK_HOLD_S, RISE_SETTLE_S, SIT_ID, RISESIT_ID,
                                 ROBOT_SPORT_API_ID_STATICWALK, ROBOT_SPORT_API_ID_TROTRUN,
                                 STOPLINE_GREEN_TIMEOUT_S, ROTATE_SETTLE_S, ROTATE_DONE_THRESH)


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
node.tick()
assert node.state == 'TO_GOAL', f"자가진단 통과해야 하는데 안 넘어감: {node.state}"
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
node._odom(2.0, 0.0)
node.tick()
assert node.state == 'STOPLINE_WAIT_SIGNAL', f"{GREEN_CONFIRM_SECONDS - 0.1:.1f}초 GREEN인데 벌써 출발함"

clock.advance(0.2)  # 끊기지 않고 확인 시간 + 0.1초 누적
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
node._odom(2.0, 0.0)
node.tick()  # -> TO_GOAL(parking)
assert node.cur()[0] == 'parking'

node._state(0.0)  # 2026-11-04: 속도 게이트 -- Sit은 "멈춘 뒤에만" 보내야 함
node._odom(3.0, 0.0, yaw=0.0)  # parking 도착, exit(4,0) 방향이라 목표각도 0도
node.tick()
# ★ 11-05: parking도 이제 회전 대상 -- 도착 직후 바로 Sit이 아니라 ROTATE부터
assert node.state == 'ROTATE', f"주차 도착 후 ROTATE(회전) 기대: {node.state}"
assert not any(s[0] == SIT_ID for s in node._sent_sports), "회전도 안 했는데 Sit부터 보냄(버그!)"
node.tick()  # 오차가 이미 0이라 이 틱에서 rotate_ok_since가 시작됨(정착시간은 여기부터 셈)
clock.advance(ROTATE_SETTLE_S + 0.1)  # 이미 목표각도(0도)라 오차 0 -- 정착시간만 지나면 완료
node._odom(3.0, 0.0, yaw=0.0)
node.tick()
assert node.state == 'PARK_SIT_WAIT', f"회전 완료 후 PARK_SIT_WAIT 기대: {node.state}"
assert node._sent_sports[-1][0] == SIT_ID, f"Sit 명령 전송 기대: {node._sent_sports[-1]}"

# 2026-11-04: body_height가 아직 안 내려왔으면(실제로 안 앉았으면) PARK_HOLD로
# 넘어가면 안 됨 -- SIT_SETTLE_S 넘으면 재전송만 하고 계속 PARK_SIT_WAIT에 머묾
clock.advance(SIT_SETTLE_S + 0.1)
node._odom(3.0, 0.0)
node.tick()
assert node.state == 'PARK_SIT_WAIT', f"아직 안 앉았는데 PARK_HOLD로 넘어감(버그!): {node.state}"

# 이제 실제로 앉음(body_height가 임계값 아래로)
node._state(0.0, body_height=0.10)
node._odom(3.0, 0.0)
node.tick()
assert node.state == 'PARK_HOLD', f"착석 확인됐는데 PARK_HOLD로 안 넘어감: {node.state}"

clock.advance(PARK_HOLD_S + 0.1)
node._odom(3.0, 0.0)
node.tick()
assert node.state == 'PARK_RISE_WAIT', f"RiseSit 전송 후 PARK_RISE_WAIT 기대: {node.state}"
assert node._sent_sports[-1][0] == RISESIT_ID, f"RiseSit 명령 전송 기대: {node._sent_sports[-1]}"

clock.advance(RISE_SETTLE_S + 0.1)
node._odom(3.0, 0.0)
node.tick()
assert node.cur()[0] == 'exit', f"주차 완료 후 exit 구간 기대: {node.cur()[0]}"
print("4) parking: Sit -> 1.5s -> 5.5s -> RiseSit -> 2.0s -> exit 전환 OK")
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
assert node.state == 'ROTATE', f"멈춘 뒤엔 회전부터 시작해야 함: {node.state}"
node.tick()
clock.advance(ROTATE_SETTLE_S + 0.1)
node._odom(3.0, 0.0, yaw=0.0)
node.tick()
assert node.state == 'PARK_SIT_WAIT', f"회전 완료 후 정상적으로 Sit 보내고 넘어가야 함: {node.state}"
print("4b) 속도 안 죽은 상태로 도착 -> Sit 보류, 멈춘 뒤 회전 -> Sit 전송 OK")
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
assert node.state == 'TO_GOAL', f"유지 중 움직였는데 계속 STOPLINE_HOLD에 머무름(버그!): {node.state}"

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
node._light('GREEN'); clock.advance(GREEN_CONFIRM_SECONDS + 0.1); node._odom(2.0, 0.0); node.tick()
assert node.cur()[0] == 'line2', node.cur()
node.tick()
gaits = [s[0] for s in node._sent_sports if s[0] in (ROBOT_SPORT_API_ID_STATICWALK, ROBOT_SPORT_API_ID_TROTRUN)]
assert gaits[-1] == ROBOT_SPORT_API_ID_TROTRUN, f"정지선 출발 뒤 line2 에선 TROTRUN: {gaits}"
print("4d2) stopline 접근 = STATICWALK, 녹색 출발 뒤 다음 구간 = TROTRUN OK")
teardown(node)

# --- 4e) rabain/rabaout(라바콘 슬라럼 진입/탈출) -- ★ 11-05: 이제 정지+회전 후 통과
# (예전엔 정지 없이 바로 통과였는데, 사용자가 "콘에 안 부딪히려면 방향 맞추고 가야
# 한다"고 명시적으로 요구해서 바뀜) + cone_zone_active True/False 발행 ---
clock = FakeClock()
node = make_node(clock, goals=[('rabain', (1.0, 0.0)), ('rabaout', (2.0, 0.0)),
                                ('line', (3.0, 0.0))])
cone_states = []
node.pub_cone_zone_active.publish = lambda m: cone_states.append(m.data)

node._odom(0.1, 0.0)
node.tick()  # START_CHECK -> TO_GOAL(rabain)
assert node.cur()[0] == 'rabain'

node._state(0.3)  # 아직 움직이는 중 -- 회전 시작하면 안 됨(이동하며 돌지 말 것)
node._odom(1.0, 0.0)  # 위치상 rabain 도착
node.tick()
assert node.state == 'TO_GOAL', f"속도 안 죽었는데 회전 시작함(버그!): {node.state}"

node._state(0.0)  # 정지
node._odom(1.0, 0.0, yaw=0.0)
node.tick()
assert node.state == 'ROTATE', f"rabain 도착 후 회전 상태 기대: {node.state}"
# 이 코스는 직선(rabain->rabaout 방위각이 이미 0도=현재 yaw)이라 오차가 0이라 cmd_vel
# 없이 바로 settle 단계로 감 -- 실제 회전량/cmd_vel 부호 검증은 4f에서 90도로 함
node.tick()  # settle 타이머 시작
clock.advance(ROTATE_SETTLE_S + 0.1)  # rabaout(2,0) 방향 = 0도 = 이미 목표각
node._odom(1.0, 0.0, yaw=0.0)
node.tick()
assert node.cur()[0] == 'rabaout', f"rabain 회전 완료 후 다음 구간 전환 실패: {node.cur()[0]}"
assert cone_states and cone_states[-1] is True, f"rabain 통과 시 cone_zone_active=True 기대: {cone_states}"

node._state(0.0)
node._odom(2.0, 0.0, yaw=0.0)  # rabaout 도착
node.tick()
assert node.state == 'ROTATE', f"rabaout 도착 후 회전 상태 기대: {node.state}"
node.tick()
clock.advance(ROTATE_SETTLE_S + 0.1)
node._odom(2.0, 0.0, yaw=0.0)
node.tick()
assert node.cur()[0] == 'line', f"rabaout 회전 완료 후 다음 구간 전환 실패: {node.cur()[0]}"
assert cone_states[-1] is False, f"rabaout 통과 시 cone_zone_active=False 기대: {cone_states}"
print("4e) rabain/rabaout 정지+회전 후 통과 + cone_zone_active True/False 전환 OK")
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
