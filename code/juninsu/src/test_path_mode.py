#!/usr/bin/env python3
"""junk 경로 전용 모드: 기록 경로(pin + stopline/parking/exit)를 ego 없이 따라가 표시 지점에 서고, 지점 동작을 하는지."""
import math, sys, os
os.environ['RUNNER_PATH_ONLY'] = '1'
import numpy as np
import rclpy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from unitree_go.msg import SportModeState
import real_course_runner as rcr


class Clock:
    t = 1000.0
    def monotonic(self): return self.t


clock = Clock(); rcr.time.monotonic = clock.monotonic
assert rcr.TL_ON, "기본은 신호등 켬"   # 1~4: 신호등 켠 경우
rclpy.init()
goals = [('pin1', (0.3, 0.0)), ('line1', (0.6, 0.0)), ('stopline1', (0.9, 0.0)), ('pin3', (1.2, 0.0)), ('parking', (1.5, 0.0)),
         ('pin4', (1.5, 0.3)), ('exit', (1.5, 0.6))]
n = rcr.RealCourseRunner((0.0, 0.0), goals)
lin, sports, cam = [], [], []
n.pub_cmd_vel.publish = lambda m: lin.append((m.linear.x, m.linear.y, m.angular.z))
n.pub_goal.publish = lambda m: None
n.pub_req.publish = lambda m: sports.append(m.header.identity.api_id)
n.pub_camera_active.publish = lambda m: cam.append(m.data)
def odom(x, y, yaw=0.0):
    m = Odometry(); m.pose.pose.position.x = x; m.pose.pose.position.y = y
    m.pose.pose.orientation.z = math.sin(yaw / 2); m.pose.pose.orientation.w = math.cos(yaw / 2); n.cb_odom(m)
def state(v, h=0.30):
    m = SportModeState(); m.velocity = [v, 0.0, 0.0]; m.body_height = h; n.cb_state(m)
state(0.0); odom(0.0, 0.0); n.tick(); n.tick()
assert n.state == 'PIN_TRACK' and n.pin_event == 2, (n.state, n.pin_event)
assert rcr.ROBOT_SPORT_API_ID_CLASSICWALK in sports, "경로 전용도 ClassicWalk"
odom(0.1, 0.0); n.tick(); assert lin[-1][0] > 0.2, f"경로 따라 앞으로: {lin[-1]}"
odom(0.80, 0.0); n.tick(); assert 0.11 < lin[-1][0] < 0.2, f"정지선 앞 감속: {lin[-1]}"
n.scan_xy = np.array([[0.40, 0.0]]); n.scan_t = clock.t; odom(0.80, 0.0); n.tick()
assert lin[-1][0] <= 1e-6, f"앞 15 cm 물체면 앞으로 안 감: {lin[-1]}"
clock.t += rcr.FRONT_STOP_MAX_S + 0.1; n.scan_t = clock.t; n.tick(); assert lin[-1][0] > 0.05, f"{rcr.FRONT_STOP_MAX_S}초 넘으면 그냥 감: {lin[-1]}"
n.scan_xy = None
odom(0.88, 0.0); n.tick()
assert n.state == 'PATH_ARRIVE' and n.cur()[0] == 'stopline1', (n.state, n.cur())
state(0.0); n.tick(); assert n.state == 'ROTATE' and abs(n.rotate_target_yaw) < 1e-6, ("정지선: 가는 쪽으로 자동 정렬", n.state)
clock.t += rcr.ROTATE_SETTLE_S + 0.05; n.tick(); clock.t += rcr.ROTATE_SETTLE_S + 0.05; n.tick()
assert n.state == 'STOPLINE_HOLD' and (not cam or cam[-1] is False), (n.state, cam)
print("1) 경로(pin·line 지나감) 따라 정지선까지 · 감속 · 앞 물체 정지(3초 한도) · 6 cm 안 도착 → 정지(아직 YOLO 끔) OK")
clock.t += rcr.STOPLINE_HOLD_S + 0.1; n.tick(); assert n.state == 'STOPLINE_WAIT_SIGNAL' and cam[-1] is True, "정지 뒤에 YOLO 켬"
n.cb_light(String(data='GREEN|0.9')); clock.t += rcr.GREEN_CONFIRM_SECONDS + 0.1; n.cb_light(String(data='GREEN|0.9')); n.tick()
assert n.cur()[0] == 'pin3' and n.state == 'TO_GOAL', (n.cur(), n.state)
n.tick(); assert n.state == 'PIN_TRACK' and n.goals[n.pin_event][0] == 'parking'
odom(1.2, 0.0); n.tick(); odom(1.48, 0.0); n.tick(); assert n.state == 'PATH_ARRIVE' and n.cur()[0] == 'parking'   # pin3(1.2) 를 지나야 주차
n.tick(); assert n.state == 'PARK_PRE', n.state
print(f"2) {rcr.STOPLINE_HOLD_S}초 정지 뒤 YOLO 켬 → 녹색 1초 → 출발 → 다음 구간 경로로 주차 지점 → 엎드리기 준비 OK")
def run(sec, h, dt=0.05):
    for _ in range(int(round(sec / dt))):
        clock.t += dt; state(0.0, h); n.tick()
clock.t += rcr.PARK_PRE_S + 0.05; n.tick(); assert sports[-1] == rcr.STANDDOWN_ID
run(0.5, 0.20); assert n.state == 'PARK_SIT_WAIT', f"내려가는 중(0.20 m)엔 아직 안 셈: {n.state}"
run(0.3, 0.06); assert n.state == 'PARK_HOLD' and n.park_hold_s == rcr.PARK_HOLD_S
run(rcr.PARK_HOLD_S - 0.3, 0.06); assert n.state == 'PARK_HOLD', "바닥에서 5.5초 다 채움"
run(0.4, 0.06); assert n.state == 'PARK_RISE_WAIT' and sports[-1] == rcr.STANDUP_ID
run(1.2, 0.283); assert n.state == 'PARK_RISE_WAIT' and sports[-1] == rcr.STANDUP_ID, f"예선 순간(1.2초, 0.283 m)엔 BalanceStand 안 보냄: {n.state}"
run(1.0, 0.30); assert n.state == 'PARK_SETTLE' and rcr.BALANCESTAND_ID in sports[-3:], (n.state, sports[-3:])
t_b = n.state_t
while clock.t < t_b + rcr.POST_STAND_S - 0.06:
    run(0.05, 0.30); assert n.state == 'PARK_SETTLE' and sports[-1] in (rcr.BALANCESTAND_ID, rcr.ROBOT_SPORT_API_ID_CLASSICWALK), "BalanceStand 뒤 1초는 걸음새(ClassicWalk) 말고 명령 없음"
run(0.2, 0.30); assert n.state in ('PARK_REALIGN', 'ROTATE'), n.state
_ib = len(sports) - 1 - sports[::-1].index(rcr.BALANCESTAND_ID)
assert sports[_ib + 1:].count(rcr.ROBOT_SPORT_API_ID_CLASSICWALK) >= 2, ("★ 주차 뒤 회전 전에 ClassicWalk 두 번", sports[_ib:])
print("3b) 주차 뒤 BalanceStand → 선 채로 ClassicWalk 2번 → 그다음 회전 OK")
n.tick(); n.tick(); clock.t += rcr.ROTATE_SETTLE_S + 0.1; odom(1.5, 0.0, yaw=math.pi / 2); n.tick(); n.tick(); clock.t += rcr.ROTATE_SETTLE_S + 0.1; n.tick()
assert n.cur()[0] in ('pin4', 'exit'), (n.cur(), n.state)
for _ in range(3): n.tick()
odom(1.5, 0.3, yaw=math.pi / 2); n.tick(); odom(1.5, 0.58, yaw=math.pi / 2); n.tick(); n.tick(); clock.t += 1.6; n.tick()
assert n.state == 'DONE', (n.state, n.cur())
print(f"3) 주차(바닥 높이부터 {rcr.PARK_HOLD_S}초 · 다 일어나 안정된 뒤 BalanceStand · 1초 뒤 출발, 위치 노드 없으면 재정렬 건너뜀) → 회전 → 출구 → 완주 OK")
# 4) 각도 찍은 점: 지나가는 점(line)이라도 거기서 멈춰 찍은 방향으로 돌고 다음으로. 정렬 회전(ALIGN_T)만큼 빼서 오늘 좌표로.
import json, tempfile
f = tempfile.NamedTemporaryFile('w', suffix='.json', delete=False)
json.dump({'format': 'route_v1', 'route': [dict(label='start', x=0, y=0), dict(label='line1', x=0.3, y=0), dict(label='line2', x=0.6, y=0, yaw=100.0),
                                          dict(label='stopline1', x=0.9, y=0, yaw=-170.0), dict(label='exit', x=1.2, y=0)]}, f); f.close()
rcr.WAYPOINTS_PATH = f.name; rcr.ALIGN_T = (0.0, 0.0, 0.0); rcr.USE_YAW = True   # RUNNER_USE_YAW=1 일 때만
sxy, g2 = rcr.load_waypoints(); rcr.ALIGN_T = (0.0, 0.0, math.radians(10))
assert [None if v is None else round(math.degrees(v)) for v in rcr.GOAL_YAW] == [None, 100, -170, None], rcr.GOAL_YAW
n2 = rcr.RealCourseRunner(sxy, g2); lin2, cam2 = [], []
n2.pub_cmd_vel.publish = lambda m: lin2.append((m.linear.x, m.linear.y, m.angular.z)); n2.pub_goal.publish = lambda m: None
n2.pub_req.publish = lambda m: None; n2.pub_camera_active.publish = lambda m: cam2.append(m.data)
n = n2; state(0.0); odom(0.0, 0.0); n.tick(); n.tick()
assert n.state == 'PIN_TRACK' and n.pin_event == 1, ("각도 찍은 line2 에서 구간이 끝나야", n.state, n.pin_event)
odom(0.3, 0.0); n.tick(); odom(0.58, 0.0); n.tick(); assert n.state == 'PATH_ARRIVE' and n.cur()[0] == 'line2'
n.tick(); assert n.state == 'ROTATE' and abs(math.degrees(n.rotate_target_yaw) - 90) < 1e-6, ("100° − 정렬 10° = 90°", n.state, n.rotate_target_yaw)
n.tick(); assert lin2[-1][0] == 0 and lin2[-1][2] > 0, f"제자리(선속도 0)로 왼쪽 회전: {lin2[-1]}"
odom(0.58, 0.0, yaw=math.radians(90)); n.tick(); clock.t += rcr.ROTATE_SETTLE_S + 0.1; n.tick()
assert n.cur()[0] == 'stopline1' and n.state == 'TO_GOAL', ("돈 뒤 다음 점", n.cur(), n.state)
n.tick(); assert n.state == 'PIN_TRACK' and n.pin_event == 2
odom(0.88, 0.0, yaw=math.radians(90)); n.tick(); n.tick()
assert n.state == 'ROTATE' and abs(math.degrees(n.rotate_target_yaw) + 180) < 1e-6 and n.rotate_timeout >= rcr.ROTATE_TIMEOUT_S, (n.state, n.rotate_target_yaw, n.rotate_timeout)
odom(0.88, 0.0, yaw=math.pi); n.tick(); clock.t += rcr.ROTATE_SETTLE_S + 0.1; n.tick()
n.tick(); clock.t += rcr.ROTATE_SETTLE_S + 0.1; odom(0.88, 0.0, yaw=0.0); n.tick(); clock.t += rcr.ROTATE_SETTLE_S + 0.1; n.tick()
assert n.state == 'STOPLINE_HOLD' and (not cam2 or cam2[-1] is False), ("돈 다음 정지선 대기(YOLO 아직 끔)", n.state)
os.unlink(f.name)
n.yaw = math.pi; n.start_rotate_to(0.0); assert n.rotate_timeout > 10, f"반 바퀴는 6초 한도보다 길게: {n.rotate_timeout}"
print("4) 각도 점: 지나가는 점도 멈춰 찍은 방향(정렬 회전 반영)으로 제자리 회전 → 다음 · 정지선은 돈 뒤 대기 · 반 바퀴도 시간 충분 OK")
rcr.USE_YAW = False; assert n.goal_yaw(1) is None, "기본(RUNNER_USE_YAW=0)은 찍은 각도 무시"
print("5) 기본은 찍은 각도 무시(자동 방향) OK")
# 6) 신호등 끔(RUNNER_TL=0, s→Y): 정지선에서 정렬 없이 2.5초 서고 바로 다음 점, 카메라 안 켬
rcr.TL_ON = False
n3 = rcr.RealCourseRunner((0.0, 0.0), [('line1', (0.3, 0.0)), ('stopline1', (0.6, 0.0)), ('exit', (0.9, 0.0))]); cam3 = []
n3.pub_cmd_vel.publish = lambda m: None; n3.pub_goal.publish = lambda m: None; n3.pub_req.publish = lambda m: None
n3.pub_camera_active.publish = lambda m: cam3.append(m.data); n = n3
state(0.0); odom(0.0, 0.0); n.tick(); n.tick(); odom(0.3, 0.0); n.tick(); odom(0.58, 0.0); n.tick(); assert n.state == 'PATH_ARRIVE' and n.cur()[0] == 'stopline1'
n.tick(); assert n.state == 'STOPLINE_HOLD', ("정렬 없이 바로 정지", n.state)
clock.t += rcr.STOPLINE_HOLD_S - 0.2; n.tick(); assert n.state == 'STOPLINE_HOLD'
clock.t += 0.3; n.tick(); assert n.cur()[0] == 'exit' and True not in cam3, ("2.5초 뒤 바로 출발·카메라 안 켬", n.cur(), cam3)
print(f"6) 신호등 끔(선택): 정지선 {rcr.STOPLINE_HOLD_S}초 정지 → 바로 출발, YOLO 안 켬 OK")
# 7) 정지선1·2 가 같은 자리(교차로, 3 cm) 사이에 고리 길: 출발하자마자 '정지선2 도착'으로 건너뛰면 안 됨 · 고리를 다 돈 뒤에만 도착
loop = [('stopline1', (1.0, 0.0)), ('line1', (2.0, 0.0)), ('line2', (2.0, 1.0)), ('line3', (1.0, 1.0)), ('line4', (0.97, 0.5)), ('stopline2', (1.0, 0.0)), ('exit', (1.0, -1.0))]   # 두 정지선 좌표 똑같이
n4 = rcr.RealCourseRunner((0.0, 0.0), loop); lin4 = []
n4.pub_cmd_vel.publish = lambda m: lin4.append((m.linear.x, m.linear.y, m.angular.z)); n4.pub_goal.publish = lambda m: None
n4.pub_req.publish = lambda m: None; n4.pub_camera_active.publish = lambda m: None; n = n4
state(0.0); odom(0.0, 0.0); n.tick(); n.tick(); odom(0.98, 0.0); n.tick(); n.tick()
clock.t += rcr.STOPLINE_HOLD_S + 0.1; n.tick(); assert n.cur()[0] == 'line1', n.cur()
for _ in range(5): odom(1.0, 0.0); n.tick()
assert n.state == 'PIN_TRACK' and n.goals[n.pin_event][0] == 'stopline2', ("같은 자리 정지선2 를 바로 도착으로 보면 안 됨", n.state, n.cur())
x, y, yaw = 1.0, 0.0, 0.0
for _ in range(2000):          # cmd_vel 대로 움직여 고리를 돈다(0.05 초)
    vx, vy, wz = lin4[-1] if lin4 else (0, 0, 0); c_, s_ = math.cos(yaw), math.sin(yaw)
    x += (c_ * vx - s_ * vy) * 0.05; y += (s_ * vx + c_ * vy) * 0.05; yaw += wz * 0.05; clock.t += 0.05
    odom(x, y, yaw); state(math.hypot(vx, vy)); n.tick()
    if n.state != 'PIN_TRACK': break
assert n.cur()[0] == 'stopline2' and n.state in ('PATH_ARRIVE', 'STOPLINE_HOLD') and math.hypot(x - 1.0, y) < 0.08, (n.state, n.cur(), x, y)
assert n.pin_wi == len(n.pin_wp) - 1, "중간 점 전부 순서대로 지남"
rcr.HEADING_TANGENT = False   # 8) 은 예전 몸 방향(목표 쪽)으로 순서 규칙만 봄
# 8) 순서대로만: 지름길(다음다음 점이 더 가까움)이어도 안 지난 점부터 — 로봇을 3번 점 옆에 갖다 놔도 2번 점 쪽으로 감
n5 = rcr.RealCourseRunner((0.0, 0.0), [('line1', (1.0, 0.0)), ('line2', (1.0, 1.0)), ('line3', (0.0, 1.0)), ('exit', (0.0, 2.0))]); lin5 = []
n5.pub_cmd_vel.publish = lambda m: lin5.append((m.linear.x, m.linear.y, m.angular.z)); n5.pub_goal.publish = lambda m: None; n5.pub_req.publish = lambda m: None
n = n5; state(0.0); odom(0.0, 0.0); n.tick(); n.tick(); odom(0.2, 0.5); n.tick()   # line3(0,1) 이 0.54 m 로 더 가깝고 line1(1,0) 은 0.94 m
assert lin5[-1][2] < -0.1 and lin5[-1][0] >= 0 and n.pin_wi == 0, ("line1(1,0) 쪽(오른쪽)으로 돎, 지름길 line3(왼쪽) 아님", lin5[-1], n.pin_wi)
print("8) 순서대로만: 다음다음 점이 더 가까워도 안 지난 점부터 감 OK")
print(f"7) 같은 자리 두 정지선(교차로): 바로 도착 안 함 → 고리 {n.pin_s[-1]:.1f} m 다 돌고 정지선2 도착({x:.2f},{y:.2f}) OK")
# 9) 막힘: 5초 넘게 제자리면 경로 점 건너뜀, 마지막(표시 지점) 0.5 m 안이면 도착으로
n6 = rcr.RealCourseRunner((0.0, 0.0), [('line1', (0.5, 0.0)), ('line2', (0.8, 0.0)), ('stopline1', (1.1, 0.0)), ('exit', (2.0, 0.0))])
n6.pub_cmd_vel.publish = lambda m: None; n6.pub_goal.publish = lambda m: None; n6.pub_req.publish = lambda m: None; n6.pub_camera_active.publish = lambda m: None
n = n6; state(0.0); odom(0.0, 0.0); n.tick(); n.tick(); assert n.state == 'PIN_TRACK' and n.pin_wi == 0
for _ in range(int(rcr.BLOCK_SKIP_S / 0.2) - 2): clock.t += 0.2; odom(0.0, 0.0); n.tick()
assert n.pin_wi == 0, "5초 전엔 그대로"
for _ in range(4): clock.t += 0.2; odom(0.0, 0.0); n.tick()
assert n.pin_wi == 1, ("5초 제자리 → line1 건너뜀", n.pin_wi)
odom(0.75, 0.0); n.tick(); assert n.pin_wi == 2, "line2(0.8) 25 cm 안 → 정상 통과"
for _ in range(int(rcr.BLOCK_SKIP_S / 0.2) + 3): clock.t += 0.2; odom(0.75, 0.0); n.tick()
assert n.state in ('PATH_ARRIVE', 'ROTATE', 'STOPLINE_HOLD') and n.cur()[0] == 'stopline1', ("정지선 0.35 m 앞에서 5초 막힘 → 도착으로", n.state, n.cur())
print("9) 막힘 5초: 경로 점 건너뜀 · 표시 지점 0.5 m 안이면 도착으로 OK")
# 10) 정지선에 찍은 각도가 있으면(USE_YAW 꺼져 있어도) 그 방향으로 서서 신호등 봄 · 없으면 다음 점 쪽
import json as _j, tempfile as _tf
rcr.TL_ON = True; rcr.USE_YAW = False
f2 = _tf.NamedTemporaryFile('w', suffix='.json', delete=False)
_j.dump({'format': 'route_v1', 'route': [dict(label='start', x=0, y=0), dict(label='stopline1', x=0.6, y=0, yaw=30.0), dict(label='exit', x=1.2, y=0)]}, f2); f2.close()
rcr.WAYPOINTS_PATH = f2.name; rcr.ALIGN_T = (0.0, 0.0, 0.0); s7, g7 = rcr.load_waypoints(); rcr.ALIGN_T = (0.0, 0.0, math.radians(5))
n7 = rcr.RealCourseRunner(s7, g7); n7.pub_cmd_vel.publish = lambda m: None; n7.pub_goal.publish = lambda m: None; n7.pub_req.publish = lambda m: None; n7.pub_camera_active.publish = lambda m: None
n = n7; state(0.0); odom(0.0, 0.0); n.tick(); n.tick(); odom(0.58, 0.0); n.tick(); n.tick()
assert n.state == 'ROTATE' and abs(math.degrees(n.rotate_target_yaw) - 25.0) < 1e-6, ("정지선 찍은 각도 30° − 정렬 5° = 25°", n.state, n.rotate_target_yaw)
os.unlink(f2.name); rcr.ALIGN_T = (0.0, 0.0, 0.0)
print("10) 정지선: 찍은 각도로 서서 신호등 봄(정렬 회전 반영), 다른 점 각도는 계속 무시 OK")
# 11) 몸 방향 = 경로 접선(기본): 이동은 목표 쪽, 회전은 경로 방향으로
rcr.HEADING_TANGENT = True; rcr.TL_ON = False
n8 = rcr.RealCourseRunner((0.0, 0.0), [('line1', (1.0, 0.0)), ('line2', (2.0, 1.0)), ('exit', (3.0, 1.0))]); c8 = []
n8.pub_cmd_vel.publish = lambda m: c8.append((m.linear.x, m.linear.y, m.angular.z)); n8.pub_goal.publish = lambda m: None; n8.pub_req.publish = lambda m: None
n = n8; state(0.0); odom(0.0, 0.0, yaw=0.0); n.tick(); n.tick(); odom(0.1, 0.0, yaw=math.radians(20)); n.tick()
assert n.state == 'PIN_TRACK' and c8[-1][2] < -0.05, ("경로는 x 쪽(0°)인데 몸이 20° → 오른쪽으로 돌아 맞춤", c8[-1])
print("11) 몸 방향 = 경로 방향: 몸이 경로와 어긋나면 경로 쪽으로 돎 OK")
print("\n전체 PASS")

# 12) ★ 11-06 juninsu v3: 2초 정지 → 10초 안 초록 없음 → 빨강 보이는 동안은 계속 섬, 빨강이 2초 안 보이면(아무것도/초록) 출발
DBG = os.environ.get('DBG') == '1'; rcr.TL_ON = True   # 6) 에서 끈 신호등 다시 켬
def stop_case(seq, limit=60.0):
    """seq(경과초) → 판정 문자열. 출발한 경과초(정지선 도착 기준) 반환, 안 떠나면 None."""
    global n
    n = rcr.RealCourseRunner((0.0, 0.0), [('stopline1', (0.05, 0.0)), ('exit', (0.6, 0.0))]); cam.clear()
    n.pub_cmd_vel.publish = lambda m: None; n.pub_goal.publish = lambda m: None; n.pub_req.publish = lambda m: None
    n.pub_camera_active.publish = lambda m: cam.append(m.data)
    state(0.0); odom(0.05, 0.0); n.tick(); n.tick()
    for _ in range(80):
        if n.state == 'STOPLINE_HOLD': break
        clock.t += 0.2; state(0.0); odom(0.05, 0.0); n.tick()
    assert n.state == 'STOPLINE_HOLD', n.state
    t_arr = clock.t
    while clock.t < t_arr + limit:
        clock.t += 0.1; state(0.0); odom(0.05, 0.0); el = clock.t - t_arr
        if el > rcr.STOPLINE_HOLD_S: n.cb_light(String(data=seq(el)))
        n.tick()
        if DBG: print(round(el, 1), n.state, n.cur()[0])
        if n.cur()[0] != 'stopline1': return el
    return None
t = stop_case(lambda e: 'RED|0.9'); assert t is None, f"빨강 계속이면 60초 지나도 안 떠남: {t}"
print("12a) 빨강 계속 60초: 계속 섬 OK")
t = stop_case(lambda e: 'RED|0.9' if e < 20.0 else 'UNKNOWN|0.0'); assert t is not None and 21.9 <= t <= 22.3, t
print(f"12b) 빨강 20초까지 → 아무것도 안 보임: {t:.1f}초에 출발(빨강 안 보인 지 2초) OK")
t = stop_case(lambda e: 'UNKNOWN|0.0'); assert t is not None and 11.9 <= t <= 12.3, t
print(f"12c) 처음부터 아무것도 안 보임: {t:.1f}초(2+10)에 출발 OK")
t = stop_case(lambda e: 'RED|0.9' if e < 15.0 else 'GREEN|0.9'); assert t is not None and 15.9 <= t <= 16.3, t
print(f"12d) 빨강 15초까지 → 초록: {t:.1f}초에 출발(초록 1초 확인) OK")
t = stop_case(lambda e: 'RED|0.9' if e < 5.0 else 'GREEN|0.9'); assert t is not None and 5.9 <= t <= 6.3, t
print(f"12e) 10초 안 초록: {t:.1f}초에 바로 출발(초록 1초) OK")
t = stop_case(lambda e: 'RED|0.9' if (e < 13.0 or 13.5 < e) and e < 40 else 'UNKNOWN|0.0'); assert t is not None and 41.9 <= t <= 42.3, t
print(f"12f) 빨강 중 0.5초 끊김(판정 빈틈)은 무시, 40초에 빨강 끝 → {t:.1f}초 출발 OK")
print('\n12 포함 PASS')

# 13) ★ 11-06: 앞 막힘 판정 — 지도에 있는 벽 점은 무시, 지도에 없는 물체(사람·넘어진 콘)엔 섬
from scipy.spatial import cKDTree as _KD
n = rcr.RealCourseRunner((0.0, 0.0), [('line1', (2.0, 0.0)), ('exit', (3.0, 0.0))]); lin.clear()
n.pub_cmd_vel.publish = lambda m: lin.append((m.linear.x, m.linear.y, m.angular.z)); n.pub_goal.publish = lambda m: None; n.pub_req.publish = lambda m: None
state(0.0); odom(0.0, 0.0); n.tick(); n.tick(); odom(0.5, 0.0); n.tick()
rcr.KNOWN_WALLS = _KD(np.array([[0.90, 0.0]]))          # 지도 벽: 몸(0.5,0) 앞 0.40 m
n.scan_xy = np.array([[0.40, 0.0]]); n.scan_t = clock.t; odom(0.5, 0.0); n.tick()
assert lin[-1][0] > 0.05, f"아는 벽(지도 0.90,0)은 무시하고 감: {lin[-1]}"
n.scan_xy = np.array([[0.40, 0.15]]); n.scan_t = clock.t; n.front_block_t = None; odom(0.5, 0.0); n.tick()
assert lin[-1][0] <= 1e-6, f"지도 벽에서 15 cm 떨어진 물체(지도에 없음) → 섬: {lin[-1]}"
rcr.KNOWN_WALLS = None
print("13) 앞 막힘: 지도 벽 점은 무시하고 감 · 지도에 없는 물체(벽에서 15 cm)엔 섬 OK")
