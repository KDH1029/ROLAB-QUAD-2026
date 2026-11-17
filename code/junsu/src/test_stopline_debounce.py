#!/usr/bin/env python3
"""stopline_mission_node의 GREEN 디바운스(GREEN_CONFIRM_SECONDS=0.5) 검증.
가짜 시계(fake monotonic)로 실제 sleep 없이 시간 흐름을 통제한다.
1) 단발성 오탐(한 틱만 GREEN, 바로 RED로 복귀) -> GO로 못 넘어가야 함.
2) 0.5초 넘게 끊기지 않고 GREEN -> GO로 넘어가야 함.
3) GREEN 중간에 한 번 끊김(RED) -> 그 뒤 다시 GREEN이어도 끊긴 시점부터
   새로 0.5초를 채워야 함(이전 구간 누적 안 됨)."""
import sys
sys.path.insert(0, '/tmp')
import rclpy
from std_msgs.msg import String
from unitree_go.msg import SportModeState
import stopline_mission_node as sm
from stopline_mission_node import StoplineMission, GREEN_CONFIRM_SECONDS

assert GREEN_CONFIRM_SECONDS == 0.5, f"이 테스트는 0.5초 기준으로 짜여있음, 실제 {GREEN_CONFIRM_SECONDS}"


class FakeClock:
    def __init__(self):
        self.t = 1000.0

    def monotonic(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def make_node(clock):
    rclpy.init()
    node = StoplineMission()
    node.pub_state.publish = lambda m: None
    sm.time.monotonic = clock.monotonic  # 모듈 전체의 time.monotonic을 가짜 시계로 교체

    def stopped(v=0.0):
        msg = SportModeState()
        msg.velocity = [v, 0.0, 0.0]
        node.cb_state(msg)
    node._stopped = stopped

    def light(state):
        node.cb_light(String(data=f'{state}|0.90'))
    node._light = light

    # APPROACH -> STOPPING -> WAIT_SIGNAL까지 빠르게 진행
    node._stopped(0.0)
    node.tick()  # APPROACH -> STOPPING, stop_start_t = clock.t
    clock.advance(STOP_HOLD := 2.6)
    node._stopped(0.0)
    node.tick()  # STOPPING -> WAIT_SIGNAL
    assert node.state == 'WAIT_SIGNAL', f"WAIT_SIGNAL 진입 실패: {node.state}"
    return node


def teardown(node):
    node.destroy_node()
    rclpy.shutdown()


# --- 1) 단발성 오탐: 한 틱만 GREEN, 바로 RED로 복귀 ---
clock = FakeClock()
node = make_node(clock)
node._light('GREEN')
clock.advance(0.1)
node._stopped(0.0)
node.tick()
assert node.state == 'WAIT_SIGNAL', f"0.1초만 GREEN인데 GO로 넘어감(버그!): {node.state}"
node._light('RED')  # 바로 꺼짐 (지나가는 사람/조명이 사라짐)
clock.advance(1.0)   # 충분히 시간이 지나도
node._stopped(0.0)
node.tick()
assert node.state == 'WAIT_SIGNAL', f"단발성 GREEN 오탐인데 결국 GO로 넘어감(버그!): {node.state}"
print("1) 단발성 오탐(0.1초 GREEN -> RED) -> WAIT_SIGNAL 유지 OK")
teardown(node)

# --- 2) 0.5초 넘게 끊기지 않고 GREEN -> GO ---
clock = FakeClock()
node = make_node(clock)
node._light('GREEN')
clock.advance(0.6)
node._stopped(0.0)
node.tick()
assert node.state == 'GO', f"0.6초 연속 GREEN인데 GO로 안 넘어감: {node.state}"
print("2) 0.6초 연속 GREEN -> GO 전환 OK")
teardown(node)

# --- 3) GREEN 중간에 한 번 끊김 -> 누적 안 되고 끊긴 시점부터 다시 0.5초 필요 ---
clock = FakeClock()
node = make_node(clock)
node._light('GREEN')
clock.advance(0.3)
node._stopped(0.0)
node.tick()
assert node.state == 'WAIT_SIGNAL'
node._light('RED')          # 0.3초 만에 끊김
clock.advance(0.1)
node._light('GREEN')        # 다시 GREEN 시작(여기서부터 새로 0.5초 카운트)
clock.advance(0.3)          # 끊긴 뒤로 0.3초밖에 안 지남(누적하면 0.6초지만 끊겼으니 무효)
node._stopped(0.0)
node.tick()
assert node.state == 'WAIT_SIGNAL', f"끊긴 구간이 누적돼서 GO로 넘어감(버그!): {node.state}"
clock.advance(0.3)          # 끊긴 시점부터 다시 0.6초 채움
node._stopped(0.0)
node.tick()
assert node.state == 'GO', f"끊긴 뒤 새로 0.5초 이상 채웠는데 GO로 안 넘어감: {node.state}"
print("3) 중간에 끊긴 GREEN은 누적 안 되고, 끊긴 뒤 새로 0.5초 채워야 GO OK")
teardown(node)

print("\n전체 PASS")
