#!/usr/bin/env python3
"""parking_mission_node 상태기 흐름 단위 검증 (ROS 그래프 없이 콜백 직접 호출)."""
import sys, time, json
sys.path.insert(0, '/tmp')
import rclpy
from std_msgs.msg import String
from unitree_go.msg import SportModeState
from parking_mission_node import ParkingMission, SIT_HEIGHT_THRESH, HOLD_SECONDS

rclpy.init()
node = ParkingMission()
sent_requests = []
node.pub_req.publish = lambda m: sent_requests.append(m.header.identity.api_id)

def qr(found, frac):
    m = String(); m.data = json.dumps({'found': found, 'payload': '', 'box_area_frac': frac, 'center_x_frac': 0.5})
    node.cb_qr(m)

def state(h):
    m = SportModeState(); m.body_height = h
    node.cb_state(m)

print(f"초기 상태: {node.state}")
assert node.state == 'APPROACH'

# 1) QR 멀리 있음 -> 아직 APPROACH 유지
qr(True, 0.02); state(0.31); node.tick()
assert node.state == 'APPROACH', f"기대 APPROACH, 실제 {node.state}"
assert len(sent_requests) == 0
print("1) 먼 QR -> APPROACH 유지: OK")

# 2) QR 충분히 가까움 -> Sit 전송, SITTING 전환
qr(True, 0.10); state(0.31); node.tick()
assert node.state == 'SITTING', f"기대 SITTING, 실제 {node.state}"
assert sent_requests == [1009], f"Sit(1009) 전송 기대, 실제 {sent_requests}"
print("2) 가까운 QR -> Sit 전송 + SITTING: OK")

# 3) 아직 몸통 안 낮아짐 -> SITTING 유지
state(0.30); node.tick()
assert node.state == 'SITTING'
print("3) 미착석 상태 -> SITTING 유지: OK")

# 4) 몸통 낮아짐(착석 확인) -> HOLDING 전환
state(0.10); node.tick()
assert node.state == 'HOLDING', f"기대 HOLDING, 실제 {node.state}"
print("4) 착석 확인(body_height<임계) -> HOLDING: OK")

# 5) HOLDING 중 시간 경과 시뮬레이션 (hold_start_t를 과거로 강제 이동)
node.hold_start_t -= (HOLD_SECONDS + 0.1)
state(0.10); node.tick()
assert node.state == 'DONE', f"기대 DONE, 실제 {node.state}"
assert node.scored is True
assert sent_requests == [1009, 1010], f"Sit+RiseSit 순서 기대, 실제 {sent_requests}"
print("5) 5초+ 경과 -> DONE, scored=True, RiseSit(1010) 전송: OK")

print("\n전체 PASS: 주차 미션 상태기 정상 동작")
rclpy.shutdown()
