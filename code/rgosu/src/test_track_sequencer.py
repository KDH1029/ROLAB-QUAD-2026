#!/usr/bin/env python3
"""track_sequencer_node 전체 7구간 순차 진행 단위 검증 (합성 오도메트리로 로봇이
전체 트랙을 쭉 걸어가는 상황을 흉내낸다)."""
import sys, time
sys.path.insert(0, '/tmp')
import rclpy
from nav_msgs.msg import Odometry
from unitree_go.msg import SportModeState
from std_msgs.msg import String
from track_sequencer_node import TrackSequencer, STOP_HOLD_SECONDS, PARK_HOLD_SECONDS

rclpy.init()
node = TrackSequencer('/tmp/segments_config.json')
sent = []
node.pub_req.publish = lambda m: sent.append(m.header.identity.api_id)
node.pub_status.publish = lambda m: None

def odom(x, y):
    m = Odometry(); m.pose.pose.position.x = x; m.pose.pose.position.y = y
    node.cb_odom(m)

def state(vel, h):
    m = SportModeState(); m.velocity = [vel, 0.0, 0.0]; m.body_height = h
    node.cb_state(m)

def light(s):
    m = String(); m.data = f'{s}|0.9'
    node.cb_light(m)

def seg_name():
    return node._cur()['name'] if node.idx < len(node.segments) else 'DONE'

# 0) start -> 진입만 하면 바로 통과
odom(0.0, 0.0); state(0.0, 0.31); light('UNKNOWN'); node.tick()
assert seg_name() == 'cone_slalom', f"start 통과 실패, 현재 {seg_name()}"
print(f"0) start 통과 -> {seg_name()}: OK")

# 1) cone_slalom -> 구간 통과 중(tune_planner 진입 로그만), 빠져나가면 다음
odom(1.5, 0.0); node.tick()
assert seg_name() == 'cone_slalom' and node.sub_state.get('entered')
odom(3.4, 0.0); node.tick()  # zone 밖으로 (cone_slalom x_max=3.0+마진0.3=3.3 넘음, stopline_1 x 3.0~3.5 안)
assert seg_name() == 'stopline_1', f"콘구간 이탈 실패, 현재 {seg_name()}"
print(f"1) cone_slalom 통과 -> {seg_name()}: OK")

# 2) stopline_1 -> 정지 2.5초 + 녹색 확인해야 통과
state(0.5, 0.31); node.tick()  # 아직 움직이는 중
assert seg_name() == 'stopline_1'
state(0.0, 0.31); node.tick()  # 정지
assert node.sub_state.get('phase') == 'STOPPING'
node.sub_state['stop_t'] -= (STOP_HOLD_SECONDS + 0.1)
state(0.0, 0.31); node.tick()
assert node.sub_state.get('phase') == 'WAIT_SIGNAL', f"기대 WAIT_SIGNAL 실제 {node.sub_state.get('phase')}"
light('RED'); node.tick()
assert seg_name() == 'stopline_1', "빨간불인데 통과하면 안 됨"
light('GREEN'); node.tick()
assert seg_name() == 'ramp_or_stairs', f"녹색불 통과 실패, 현재 {seg_name()}"
print(f"2) stopline_1(정지+신호) 통과 -> {seg_name()}: OK")

# 3) ramp_or_stairs -> 진입 시 STATICWALK(1061), 이탈 시 TROTRUN(1062)
odom(4.2, 0.0); node.tick()
assert 1061 in sent, "STATICWALK 전송 안 됨"
odom(5.4, 0.0); node.tick()  # ramp_or_stairs 이탈 마진(5.0+0.3=5.3) 넘어서 stopline_2 zone(5.0~5.5)으로 이동
assert seg_name() == 'stopline_2', f"경사/계단 이탈 실패, 현재 {seg_name()}"
assert sent[-1] == 1062, f"TROTRUN 복귀 안 됨, 마지막 전송 {sent[-1]}"
print(f"3) ramp_or_stairs(걸음걸이 전환) 통과 -> {seg_name()}: OK")

# 4) stopline_2 -> 동일 로직
state(0.0, 0.31); node.tick()
node.sub_state['stop_t'] -= (STOP_HOLD_SECONDS + 0.1)
state(0.0, 0.31); node.tick()
light('GREEN'); node.tick()
assert seg_name() == 'parking', f"stopline_2 통과 실패, 현재 {seg_name()}"
print(f"4) stopline_2 통과 -> {seg_name()}: OK")

# 5) parking -> Sit(1009) 전송, body_height 낮아지면 5.5초 유지 후 RiseSit(1010)
odom(6.0, 0.0); node.tick()
assert sent[-1] == 1009, f"Sit 전송 안 됨, 마지막 {sent[-1]}"
state(0.0, 0.31); node.tick()  # 아직 안 앉음
assert node.sub_state.get('phase') == 'SITTING'
state(0.0, 0.10); node.tick()  # 착석 확인
assert node.sub_state.get('phase') == 'HOLDING'
node.sub_state['hold_t'] -= (PARK_HOLD_SECONDS + 0.1)
state(0.0, 0.10); node.tick()
assert seg_name() == 'exit', f"주차 통과 실패, 현재 {seg_name()}"
assert sent[-1] == 1010, f"RiseSit 전송 안 됨, 마지막 {sent[-1]}"
print(f"5) parking(앉기+5초유지+RiseSit) 통과 -> {seg_name()}: OK")

# 6) exit -> 진입하면 전체 완료
odom(7.0, 0.0); node.tick()
assert seg_name() == 'DONE', f"exit 통과 실패, 현재 {seg_name()}"
print(f"6) exit 통과 -> 전체 구간 완료: OK")

print(f"\n전체 전송된 명령 시퀀스: {sent}")
print("전체 PASS: 7구간 전부 순서대로 정상 통과")
rclpy.shutdown()
