#!/usr/bin/env python3
"""목표지점(/goal_pose) 발행이 올바른 시점(start 시작 시, parking 완료 후
exit 진입 시)에 올바른 좌표로 나가는지 검증. 중간 구간(콘슬라럼/정지선/계단)
에서는 목표가 재발행되지 않아야 한다(의도대로 '단순화'됐는지 확인)."""
import sys
sys.path.insert(0, '/tmp')
import rclpy
from nav_msgs.msg import Odometry
from unitree_go.msg import SportModeState
from std_msgs.msg import String
from track_sequencer_node import (TrackSequencer, STOP_HOLD_SECONDS,
                                   PARK_HOLD_SECONDS, GREEN_CONFIRM_SECONDS)

rclpy.init()
node = TrackSequencer('/tmp/segments_config.json')
goals = []
node.pub_goal.publish = lambda m: goals.append((m.pose.position.x, m.pose.position.y))
node.pub_req.publish = lambda m: None
node.pub_status.publish = lambda m: None
node.pub_camera_active.publish = lambda m: None

# __init__ 안에서 이미 start의 goal이 (몽키패치 전이라 못 잡고) 실제로 한 번
# 발행됐을 것이다 -- 같은 호출을 다시 트리거해서 "start면 정확히 이 좌표로
# 발행한다"는 동작 자체를 검증한다.
node._publish_goal_if_any(node._cur())
assert len(goals) == 1, f"start 목표 발행 기대, 실제 {len(goals)}개: {goals}"
assert goals[0] == (6.0, 0.0), f"start 목표 좌표 불일치: {goals[0]}"
print(f"1) start 목표 발행 내용 확인: {goals[0]} OK")
goals.clear()


def odom(x, y):
    m = Odometry(); m.pose.pose.position.x = x; m.pose.pose.position.y = y
    node.cb_odom(m)


def state(vel, h):
    m = SportModeState(); m.velocity = [vel, 0.0, 0.0]; m.body_height = h
    node.cb_state(m)


def light(s):
    m = String(); m.data = f'{s}|0.9'
    node.cb_light(m)


# cone_slalom, stopline_1, ramp_or_stairs, stopline_2를 전부 통과시키는 동안
# 목표가 재발행되면 안 된다(그 구간들엔 'goal' 필드가 없으므로). 각 구간은
# 반드시 그 zone "안쪽"을 한 번 찍고 지나가야 entered 플래그가 기록된다 —
# 한 틱에 구간을 통째로 건너뛰면(zone보다 큰 보폭) 진입 자체가 기록 안 됨.
odom(0.0, 0.0); node.tick()                      # start 통과 -> cone_slalom
assert node._cur()['name'] == 'cone_slalom', f"start 통과 실패: {node._cur()['name']}"

odom(1.5, 0.0); node.tick()                      # cone_slalom 안쪽
odom(3.4, 0.0); node.tick()                      # cone_slalom 이탈 -> stopline_1
assert node._cur()['name'] == 'stopline_1'

state(0.0, 0.31); node.tick()                    # 정지 감지 -> STOPPING
node.sub_state['stop_t'] -= (STOP_HOLD_SECONDS + 0.1)
state(0.0, 0.31); node.tick()                    # 2초+ 정지 완료 -> WAIT_SIGNAL
light('GREEN')
node.green_since_t -= (GREEN_CONFIRM_SECONDS + 0.1)  # ★ 2026-11-04: GREEN 디바운스 추가로
# 1프레임만으론 안 넘어감 -- 연속 확인 시간이 이미 지난 것처럼 되감아서 시뮬레이션
node.tick()                                          # 녹색 연속 확인 -> ramp_or_stairs
assert node._cur()['name'] == 'ramp_or_stairs'

odom(4.2, 0.0); node.tick()                      # ramp_or_stairs 안쪽(진입 기록용)
odom(5.4, 0.0); node.tick()                      # 이탈(마진 넘음) -> stopline_2
assert node._cur()['name'] == 'stopline_2'

state(0.0, 0.31); node.tick()
node.sub_state['stop_t'] -= (STOP_HOLD_SECONDS + 0.1)
state(0.0, 0.31); node.tick()
light('GREEN')
node.green_since_t -= (GREEN_CONFIRM_SECONDS + 0.1)
node.tick()                                          # -> parking
assert node._cur()['name'] == 'parking'

assert len(goals) == 0, f"중간 구간들 통과 중 목표가 발행됨(안 돼야 함): {goals}"
print(f"2) 콘슬라럼/정지선/계단 통과 중 목표 재발행 없음 확인: OK (여전히 {len(goals)}개)")

# parking 완료(Sit->Hold 5.5s->RiseSit) -> exit 진입 시 두 번째 목표 발행돼야 함
odom(6.0, 0.0); node.tick()                      # Sit 전송
state(0.0, 0.31); node.tick()
state(0.0, 0.10); node.tick()                    # 착석 확인 -> HOLDING
node.sub_state['hold_t'] -= (PARK_HOLD_SECONDS + 0.1)
state(0.0, 0.10); node.tick()                    # RiseSit -> exit 진입

assert len(goals) == 1, f"exit 진입 시 목표 발행 기대, 실제 {len(goals)}개: {goals}"
assert goals[0] == (8.0, 0.0), f"exit 목표 좌표 불일치: {goals[0]}"
print(f"3) 주차 완료 후 exit 목표 발행 확인: {goals[0]} OK")

print(f"\n전체 발행된 목표 시퀀스: {goals}")
print("전체 PASS: 목표지점 발행이 설계대로(시작+주차후 둘뿐) 동작함")
rclpy.shutdown()
