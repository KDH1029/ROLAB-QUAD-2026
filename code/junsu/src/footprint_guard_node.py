#!/usr/bin/env python3
"""몸 모양 안전 필터 노드 — 모든 이동 명령(/cmd_vel: ego·러너·측정 도구)을 모터 브리지로 보내기 전에 검사(junsu v2 11-05).

  /cmd_vel → [이 노드] → /cmd_vel_safe → go2_cmd_vel_to_sport(cmd_vel_topic:=/cmd_vel_safe)
벽 = 합치기 노드가 쌓아 둔 벽 기억(/mission/wall_local, 세계 좌표, 2 m 안 전 방향 — 뒤쪽 XT16 가림 포함).
콘은 안 넣는다(벽 접촉만 감점, 콘 무시·경로 따라가기와 안 싸우게). 판정은 footprint_guard_core.
벽 기억이 1초 넘게 안 오면 검사 없이 그대로 넘김(이 노드가 주행을 막지 않게) — 상태판·로그에 표시.
"""
import json
import math
import time

import numpy as np
import rclpy
import rclpy.executors
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

import footprint_guard_core as G


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Guard(Node):
    def __init__(self):
        super().__init__('junsu_footprint_guard')
        self.pose = None; self.W = np.zeros((0, 2)); self.W_t = -1e9
        self.n = {'in': 0, 'ok': 0, 'changed': 0, 'stop': 0, 'nowall': 0, 'level': 0}
        self.blocked_since = None; self.level = 0   # 고착 탈출: 움직이라는 명령을 1.5초 넘게 막으면 1, 3초 넘으면 2 단계
        self.pub = self.create_publisher(Twist, '/cmd_vel_safe', 10)
        self.pub_st = self.create_publisher(String, '/mission/guard', 2)
        self.create_subscription(Odometry, '/Odometry', self.cb_odom, 20)
        self.create_subscription(PointCloud2, '/mission/wall_local', self.cb_wall, 5)
        self.create_subscription(Twist, '/cmd_vel', self.cb_cmd, 20)
        self.create_timer(1.0, self.report)
        self.last_log = 0.0
        # ★ 11-06 완화(시뮬: 0.40+0.03 m 몸·1초 앞이면 새 코스 0.5~0.65 m 통로에서 매번 막힘). 예전 값: half_w 0.20 · margin 0.03 · horizon 1.0 · nudge false
        p = self.declare_parameter
        G.configure(half_w=p('half_w', 0.165).value, margin=p('margin', 0.02).value, horizon=p('horizon', 0.6).value,
                    dvy=(0.0, -0.08, 0.08, -0.15, 0.15))
        self.nudge = bool(p('nudge', True).value); self.nudge_d = float(p('nudge_dist', 0.08).value); self.nudge_k = float(p('nudge_gain', 0.6).value)
        self.unstick = (float(p('unstick1_s', 1.0).value), float(p('unstick2_s', 2.0).value))
        self.get_logger().info(f'몸 모양 안전 필터 — 몸 {G.FRONT + G.REAR:.2f}×{2 * G.HALF_W:.2f} m(+{G.MARGIN}), {G.HORIZON}초 앞, 가운데로 밀기 {"켬" if self.nudge else "끔"}')

    def cb_odom(self, m):
        p = m.pose.pose.position; self.pose = (p.x, p.y, yaw_of(m.pose.pose.orientation))

    def cb_wall(self, m):
        a = np.frombuffer(bytes(m.data), dtype=np.float32).reshape(-1, m.point_step // 4) if m.width * m.height else np.zeros((0, 3), np.float32)
        self.W = a[:, :2].astype(float); self.W_t = time.monotonic()

    def cb_cmd(self, m):
        self.n['in'] += 1
        cmd = (m.linear.x, m.linear.y, m.angular.z)
        if self.pose is None or time.monotonic() - self.W_t > 1.0:
            self.n['nowall'] += 1; self.pub.publish(m); return
        x, y, th = self.pose; c, s = math.cos(th), math.sin(th); d = self.W - [x, y]
        P = np.c_[c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]]          # 세계 → 몸체
        now = time.monotonic(); moving = abs(cmd[0]) + abs(cmd[1]) + abs(cmd[2]) > 1e-3
        out, why = G.guard(P, cmd, 0)
        if why != 'stop' or not moving:                    # 0 단계로 되면(틈을 지났거나 명령이 바뀜) 풀었던 단계 되돌림
            if self.level:
                self.get_logger().info('0 단계 검사 통과 — 검사 0 단계로')
            self.blocked_since = None; self.level = 0
        else:                                              # 0 단계로는 막힘 — 막힌 시간만큼 풀어 본다
            self.blocked_since = now if self.blocked_since is None else self.blocked_since
            lv = 2 if now - self.blocked_since > self.unstick[1] else (1 if now - self.blocked_since > self.unstick[0] else 0)
            if lv != self.level:
                self.level = lv; self.get_logger().warn(f'막힌 지 {now - self.blocked_since:.1f}초 — 검사 {lv} 단계로 풂(고착 탈출)')
            if self.level:
                out, why = G.guard(P, cmd, self.level)
        # ★ 11-06 다른 방식: 막는 대신 가운데로 — 앞으로 가는 중 한쪽 벽이 몸 옆 nudge_dist 안이면 넓은 쪽으로 옆걸음을 조금 더함(부딪히기 전에 비킴)
        if self.nudge and why != 'stop' and abs(out[0]) > 0.05:
            dl, dr = G.side_clear(P)
            if min(dl, dr) < self.nudge_d:
                vy_add = float(np.clip(self.nudge_k * (min(dl, 0.3) - min(dr, 0.3)), -0.10, 0.10))
                cand = (out[0], out[1] + vy_add, out[2])
                if not G.collides(P, cand, None, 0.0, 0.4):
                    out = cand; why = 'nudge' if why == 'ok' else why
        self.n[why] = self.n.get(why, 0) + 1; self.n['level'] = self.level
        if why != 'ok' and time.monotonic() - self.last_log > 1.0:
            self.last_log = time.monotonic()
            self.get_logger().warn(f'벽 접촉 예상 — 명령 {why}: ({cmd[0]:+.2f},{cmd[1]:+.2f},{cmd[2]:+.2f}) → ({out[0]:+.2f},{out[1]:+.2f},{out[2]:+.2f})')
        t = Twist(); t.linear.x, t.linear.y, t.angular.z = float(out[0]), float(out[1]), float(out[2])
        self.pub.publish(t)

    def report(self):
        self.pub_st.publish(String(data=json.dumps(self.n)))
        for k in self.n:
            self.n[k] = 0


def main():
    rclpy.init(); n = Guard()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
