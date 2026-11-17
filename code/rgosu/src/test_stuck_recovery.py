#!/usr/bin/env python3
"""real_course_runner 벽·콘 붙음 복구(BACKOFF) 오프라인 검증(가짜 시계).
1) 목표로 가다가 벽 0.10 m 옆에서 4초 제자리 → BACKOFF: /cmd_vel 이 벽 반대쪽(몸체 기준)으로 0.15 m/s → 1.3초 뒤 정지 + 목표 다시 보냄
2) 장애물이 멀면(0.45 m 밖) 제자리여도 물러나지 않음(ego 가 생각 중일 수 있음)
3) line* 경유점에서 SKIP_VIA_AFTER 번 물러나도 못 가면 그 경유점을 건너뜀 / 임무 점(parking)은 건너뛰지 않음
"""
import sys
import math
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
import real_course_runner as rcr
from real_course_runner import RealCourseRunner, STUCK_S, BACKOFF_S, BACKOFF_V, BACKOFF_GAP_S, SKIP_VIA_AFTER


class FakeClock:
    def __init__(self):
        self.t = 1000.0

    def monotonic(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def cloud(pts):
    pts = np.asarray(pts, np.float32)
    m = PointCloud2(height=1, width=len(pts), point_step=12, row_step=12 * len(pts), is_dense=True,
                    fields=[PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyz')])
    m.data = pts.tobytes(); return m


def make(clock, goals):
    rclpy.init()
    node = RealCourseRunner((0.0, 0.0), goals)
    node._cmd = []; node._goals = []
    node.pub_cmd_vel.publish = lambda m: node._cmd.append((m.linear.x, m.linear.y, m.angular.z))
    node.pub_goal.publish = lambda m: node._goals.append((m.pose.position.x, m.pose.position.y))
    node.pub_req.publish = lambda m: None
    node.pub_camera_active.publish = lambda m: None
    rcr.time.monotonic = clock.monotonic

    def odom(x, y, yaw=0.0):
        m = Odometry(); m.pose.pose.position.x = x; m.pose.pose.position.y = y
        m.pose.pose.orientation.z = math.sin(yaw / 2); m.pose.pose.orientation.w = math.cos(yaw / 2); node.cb_odom(m)
    node._odom = odom
    return node


def run(node, clock, secs, x, y, yaw=0.0, obs=None):
    for _ in range(int(round(secs / 0.2))):
        clock.advance(0.2); node._odom(x, y, yaw)
        if obs is not None:
            node.cb_obstacle(cloud(obs))
        node.tick()


# 벽: y = +0.10 에 x 방향으로 늘어선 벽(로봇 왼쪽 0.10 m). 로봇은 (1,0) 에서 앞(+x)을 보고 서 있음.
wall = [(1.0 + dx, 0.10, 0.3) for dx in np.arange(-0.5, 0.5, 0.05)]

# --- 1) 붙어서 제자리 → BACKOFF
clock = FakeClock(); node = make(clock, [('line1', (3.0, 0.0)), ('exit', (5.0, 0.0))])
run(node, clock, 0.4, 0.0, 0.0)                        # START_CHECK → TO_GOAL
assert node.state == 'TO_GOAL', node.state
run(node, clock, STUCK_S + 0.6, 1.0, 0.0, obs=wall)    # 4초 넘게 제자리
assert node.state == 'BACKOFF', f'벽 옆 4초 제자리인데 BACKOFF 안 함: {node.state}'
vx, vy, wz = node._cmd[-1]
assert vy < -0.1 and abs(vx) < 0.05 and wz == 0.0, f'벽(왼쪽 +y) 반대쪽(-y)으로 물러나야 함: {node._cmd[-1]}'
assert abs(math.hypot(vx, vy) - BACKOFF_V) < 1e-6
n_goal = len(node._goals)
run(node, clock, BACKOFF_S + 0.2, 1.0, -0.2, obs=wall)
assert node.state == 'TO_GOAL' and node._cmd[-1] == (0.0, 0.0, 0.0), f'물러난 뒤 정지+TO_GOAL: {node.state} {node._cmd[-1]}'
run(node, clock, 0.2, 1.0, -0.2, obs=wall)
assert len(node._goals) > n_goal, '물러난 뒤 목표를 바로 다시 보내야 함'
print(f'1) 벽 0.10 m 옆 {STUCK_S:.0f}초 제자리 → 벽 반대쪽(vy {vy:+.2f}) {BACKOFF_S}초 물러남 → 정지·목표 재발행 OK')
node.destroy_node(); rclpy.shutdown()

# --- 2) 장애물이 멀면 안 물러남
clock = FakeClock(); node = make(clock, [('line1', (3.0, 0.0)), ('exit', (5.0, 0.0))])
run(node, clock, 0.4, 0.0, 0.0)
far = [(1.0 + dx, 0.80, 0.3) for dx in np.arange(-0.5, 0.5, 0.05)]
run(node, clock, STUCK_S + 2.0, 1.0, 0.0, obs=far)
assert node.state == 'TO_GOAL', f'장애물 0.8 m 면 물러나지 않아야 함: {node.state}'
print('2) 장애물 0.8 m(0.45 m 밖)면 제자리여도 안 물러남 OK')
node.destroy_node(); rclpy.shutdown()

# --- 3) line* 경유점은 SKIP_VIA_AFTER 번 뒤 건너뜀, parking 은 안 건너뜀
for lab, expect_skip in (('line1', True), ('parking', False)):
    clock = FakeClock(); node = make(clock, [(lab, (3.0, 0.0)), ('exit', (5.0, 0.0))])
    run(node, clock, 0.4, 0.0, 0.0)
    for k in range(SKIP_VIA_AFTER + 2):
        run(node, clock, STUCK_S + 0.6, 1.0, 0.0, obs=wall)     # 붙음 → BACKOFF
        run(node, clock, BACKOFF_S + BACKOFF_GAP_S + 0.4, 1.0, 0.0, obs=wall)   # 물러났는데도 제자리(위치 그대로)
    skipped = node.cur()[0] != lab
    assert skipped == expect_skip, f'{lab}: 건너뜀 기대 {expect_skip}, 실제 {skipped} (지금 목표 {node.cur()[0]}, 물러남 {node.backoffs_here})'
    print(f"3) {lab}: 계속 막히면 {'건너뜀' if expect_skip else '건너뛰지 않고 계속 시도'} OK")
    node.destroy_node(); rclpy.shutdown()
print('\n전체 PASS')
