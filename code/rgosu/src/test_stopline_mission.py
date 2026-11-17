#!/usr/bin/env python3
"""stopline_mission_node 상태기 단위 검증."""
import sys, time
sys.path.insert(0, '/tmp')
import rclpy
from std_msgs.msg import String
from unitree_go.msg import SportModeState
from stopline_mission_node import StoplineMission, STOP_HOLD_SECONDS

rclpy.init()
node = StoplineMission()

def light(s):
    m = String(); m.data = f'{s}|0.90'
    node.cb_light(m)

def vel(v):
    m = SportModeState(); m.velocity = [v, 0.0, 0.0]
    node.cb_state(m)

print(f"초기 상태: {node.state}")
assert node.state == 'APPROACH'

# 1) 아직 움직이는 중 -> APPROACH 유지
vel(0.5); light('UNKNOWN'); node.tick()
assert node.state == 'APPROACH'
print("1) 주행 중 -> APPROACH 유지: OK")

# 2) 정지 -> STOPPING 전환
vel(0.0); node.tick()
assert node.state == 'STOPPING', f"기대 STOPPING 실제 {node.state}"
print("2) 정지 -> STOPPING: OK")

# 3) 2초 못 채우고 다시 움직임 -> APPROACH로 복귀
vel(0.3); node.tick()
assert node.state == 'APPROACH', f"기대 APPROACH(복귀) 실제 {node.state}"
print("3) 2초 전 재출발 -> APPROACH 복귀: OK")

# 4) 다시 정지, 이번엔 2초+ 유지 -> WAIT_SIGNAL
vel(0.0); node.tick()
assert node.state == 'STOPPING'
node.stop_start_t -= (STOP_HOLD_SECONDS + 0.1)
node.tick()
assert node.state == 'WAIT_SIGNAL', f"기대 WAIT_SIGNAL 실제 {node.state}"
print("4) 2초+ 정지 완료 -> WAIT_SIGNAL: OK")

# 5) 빨간불 -> 계속 WAIT_SIGNAL (출발 안 함)
light('RED'); node.tick()
assert node.state == 'WAIT_SIGNAL'
assert node.go_latched is False
print("5) 빨간불 -> WAIT_SIGNAL 유지, go_latched=False: OK")

# 6) UNKNOWN(오인식/안 보임)도 안전하게 대기 유지
light('UNKNOWN'); node.tick()
assert node.state == 'WAIT_SIGNAL'
print("6) UNKNOWN 신호 -> 안전하게 WAIT_SIGNAL 유지: OK")

# 7) 녹색불 -> GO 전환, go_latched=True
light('GREEN'); node.tick()
assert node.state == 'GO', f"기대 GO 실제 {node.state}"
assert node.go_latched is True
print("7) 녹색불 -> GO, go_latched=True: OK")

# 8) GO 상태에서 빨간불로 바뀌어도 이미 출발한 회차는 유지(규정 반영)
light('RED'); vel(0.8); node.tick()
assert node.state == 'GO'
assert node.go_latched is True
print("8) GO 후 빨간불 바뀌어도 latched 유지: OK")

print("\n전체 PASS: 정지선 상태기 정상 동작")
rclpy.shutdown()
