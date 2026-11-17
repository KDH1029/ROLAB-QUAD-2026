#!/usr/bin/env python3
"""11-06 본선 재현: 오늘 경로(logs/route_today.json)의 정지선1(39°로 섬)에서 초록 출발 → 로봇이 '제자리 회전은 안 함'(전진·옆 0.06 m/s 미만이면 아무것도 안 함) 모델.
TURN_STALL_S=99 면 고치기 전과 같음. 20 s 동안 얼마나 갔나·다음 점(line7)까지 남은 거리."""
import os, sys, math
os.environ['RUNNER_PATH_ONLY'] = '1'
J = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))); sys.path.insert(0, J + '/src')
import numpy as np, rclpy
from nav_msgs.msg import Odometry; from unitree_go.msg import SportModeState
import real_course_runner as rcr
if len(sys.argv) > 1: rcr.TURN_STALL_S = float(sys.argv[1])
class C: t = 1000.0
clk = C(); rcr.time.monotonic = lambda: clk.t
rcr.WAYPOINTS_PATH = J + '/logs/route_today.json'; s0, goals = rcr.load_waypoints(); i = [g[0] for g in goals].index('stopline1')
rclpy.init(); n = rcr.RealCourseRunner(s0, goals); cmd = [0.0, 0.0, 0.0]; msgs = []
n.pub_cmd_vel.publish = lambda m: cmd.__setitem__(slice(0, 3), [m.linear.x, m.linear.y, m.angular.z])
n.pub_goal.publish = lambda m: None; n.pub_req.publish = lambda m: None; n.pub_camera_active.publish = lambda m: None
x, y = goals[i][1]; yaw = math.radians(39); vel = np.zeros(3)
def feed():
    m = Odometry(); m.pose.pose.position.x, m.pose.pose.position.y = x, y; m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2); n.cb_odom(m)
    s = SportModeState(); s.velocity = [float(math.hypot(vel[0], vel[1])), 0., 0.]; s.body_height = 0.31; n.cb_state(s)
    n.scan_xy = np.zeros((0, 2)); n.scan_t = clk.t
feed(); n.tick(); n.tick(); n.idx = i; n.advance()
x0, y0 = x, y; dt = 0.05; first_move = None
for k in range(int(30 / dt)):
    clk.t += dt
    if k % 4 == 0: feed(); n.tick()
    c = np.array(cmd)
    if math.hypot(c[0], c[1]) < 0.06: c[:] = 0.0          # 제자리 회전 안 함(본선 관찰)
    vel += (c - vel) * (dt / np.array([0.35, 0.35, 0.18]))
    cy, sy = math.cos(yaw), math.sin(yaw); x += (cy * vel[0] - sy * vel[1]) * dt; y += (sy * vel[0] + cy * vel[1]) * dt; yaw += vel[2] * dt
    if first_move is None and math.hypot(x - x0, y - y0) > 0.05: first_move = clk.t - 1000
lx, ly = goals[i + 1][1]
print(f'TURN_STALL_S={rcr.TURN_STALL_S}: 30 s 뒤 이동 {math.hypot(x - x0, y - y0):.2f} m · 처음 움직인 시각 {first_move} s · 몸 방향 {math.degrees(yaw):.0f}° · 상태 {n.state} 목표 {n.cur()[0]} 경유점 {n.pin_wi}/{len(n.pin_wp)}')
