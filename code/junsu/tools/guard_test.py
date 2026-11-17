#!/usr/bin/env python3
"""몸 모양 안전 필터 실기 시험 — ★ 로봇이 움직인다(필터가 세우는지 보는 시험). 리모컨을 손에.

  ~/ws/junsu_v2/tools/guard_test.sh front   # 벽을 보고 1.5 m 앞에서: 0.3 m/s 전진 최대 8초 → 서서히 줄다 몸 앞 8~12 cm 에서 서야 함
  ~/ws/junsu_v2/tools/guard_test.sh back    # 벽을 등지고(벽을 먼저 본 뒤 돌아서) 0.15 m/s 후진 최대 8초 → 뒤 벽 앞에서 서야 함(XT16 사각)
  ~/ws/junsu_v2/tools/guard_test.sh spin    # 벽 옆에서 제자리 0.6 rad/s 6초 → 꼬리가 벽에 닿을 회전이면 줄이거나 세움
조건: v2 관리자에서 a(스택+필터) · m(모터, /cmd_vel_safe) — 러너는 끔.
기록: /cmd_vel(보낸 것)·/cmd_vel_safe(필터가 낸 것)·/sportmodestate 속도·/mission/wall_local(벽 기억)으로 몸 앞/뒤 벽 간격.
"""
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

FRONT, REAR, HALF_W = 0.34, 0.36, 0.20
MODES = {'front': ((0.3, 0.0, 0.0), 8.0), 'back': ((-0.15, 0.0, 0.0), 8.0), 'spin': ((0.0, 0.0, 0.6), 6.0)}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else 'front'
    (vx, vy, wz), secs = MODES[mode]
    from rosidl_runtime_py.utilities import get_message
    rclpy.init(); n = rclpy.create_node('junsu_guard_test')
    pub = n.create_publisher(Twist, '/cmd_vel', 10)
    st = {'safe': None, 'v': None, 'pose': None, 'W': np.zeros((0, 2))}
    n.create_subscription(Twist, '/cmd_vel_safe', lambda m: st.__setitem__('safe', (m.linear.x, m.linear.y, m.angular.z)), 10)
    n.create_subscription(get_message('unitree_go/msg/SportModeState'), '/sportmodestate',
                          lambda m: st.__setitem__('v', (m.velocity[0], m.yaw_speed)), qos_profile_sensor_data)
    n.create_subscription(Odometry, '/Odometry', lambda m: st.__setitem__('pose', (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))), 20)
    def cb_w(m):
        a = np.frombuffer(bytes(m.data), np.float32).reshape(-1, m.point_step // 4) if m.width else np.zeros((0, 3), np.float32)
        st['W'] = a[:, :2].astype(float)
    n.create_subscription(PointCloud2, '/mission/wall_local', cb_w, 5)

    def gaps():
        if st['pose'] is None or not len(st['W']):
            return None, None, None
        x, y, th = st['pose']; c, s = math.cos(th), math.sin(th); d = st['W'] - [x, y]
        lx, ly = c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]
        band = np.abs(ly) <= HALF_W
        f = lx[band & (lx > 0)]; b = -lx[band & (lx < 0)]
        side = np.abs(ly[(lx > -REAR) & (lx < FRONT)])
        return (float(f.min() - FRONT) if len(f) else None, float(b.min() - REAR) if len(b) else None,
                float(side.min() - HALF_W) if len(side) else None)

    t = time.time()
    while time.time() - t < 2.0:
        rclpy.spin_once(n, timeout_sec=0.05)
    if pub.get_subscription_count() == 0:
        print('★ /cmd_vel 받는 노드 없음 — v2 관리자 a(필터) 확인'); return 1
    g0 = gaps(); print(f'시작: 몸 앞 벽까지 {g0[0]} m · 뒤 {g0[1]} m · 옆 {g0[2]} m (벽 기억 기준)')
    if g0[0] is None and mode == 'front':
        print('★ 앞에 벽 기억이 없음 — 벽을 보게 세웠나? 그래도 하려면 리모컨 준비')
    for i in (3, 2, 1):
        print(f'{mode}: ({vx:+.2f},{vy:+.2f},{wz:+.2f}) {secs:.0f}초 — {i} … (Ctrl+C 로 멈춤)', flush=True); time.sleep(1.0)
    log = []; t0 = time.time()
    try:
        while time.time() - t0 < secs:
            c = Twist(); c.linear.x, c.linear.y, c.angular.z = vx, vy, wz; pub.publish(c)
            rclpy.spin_once(n, timeout_sec=0.1)
            g = gaps(); log.append((round(time.time() - t0, 2), st['safe'], st['v'], g))
            if len(log) % 5 == 0 and st['safe'] is not None:
                print(f"  {log[-1][0]:4.1f}s 필터 출력 vx {st['safe'][0]:+.2f} wz {st['safe'][2]:+.2f} · 실제 vx {st['v'][0] if st['v'] else 0:+.2f} · 앞 {g[0] if g[0] is None else round(g[0], 3)} 뒤 {g[1] if g[1] is None else round(g[1], 3)} 옆 {g[2] if g[2] is None else round(g[2], 3)}", flush=True)
    finally:
        for _ in range(5):
            pub.publish(Twist()); time.sleep(0.05)
    g1 = gaps()
    print(f'\n■ 끝: 몸 앞 벽 간격 {g1[0]} m · 뒤 {g1[1]} m · 옆 {g1[2]} m  (몸 앞 0.34 / 뒤 0.36 / 반폭 0.20 기준, 벽 기억으로 잰 값)')
    out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'maps', 'segments', 'logs', time.strftime(f'guard_test_{mode}_%H%M%S.json'))
    json.dump(log, open(out, 'w')); print('기록', out)
    n.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
