#!/usr/bin/env python3
"""시작 자세를 정합 노드에 넘긴다. 언제 보내도 맞도록 오도메트리를 반영해서 계산한다.

  python3 publish_initialpose.py --pose 1.214,-4.758,7.9
  python3 publish_initialpose.py --bag data3            # 아는 녹화본이면 자세를 찾아 쓴다

【왜 그냥 보내면 안 되나】
`/initialpose` 는 **map -> base_link** 를 뜻한다. 그런데 우리가 아는 값은
로봇이 **부팅한 자리**(= map -> odom)다. 보내는 시점에 로봇이 이미 움직였다면
그 둘은 다르다. 그 차이만큼 위치추정이 틀린 데서 시작한다.

그래서 지금 오도메트리를 읽어 합쳐서 보낸다:

    map->base_link = (map->odom) ⊗ (odom->base_link)

이렇게 하면 재생 몇 초에 보내든 결과가 같다.
"""
import argparse
import json
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

# 3D 정합이 수렴한 뒤 읽어낸 부팅 자세 (map->odom). out/chk_*.csv 끝부분 중앙값.
KNOWN = {
    'data2': (7.499, -4.376, 122.6, 0.045),
    'data3': (1.214, -4.758, 7.9, 0.047),
    'data4': (1.591, -4.127, 41.2, 0.101),          # 잔차가 문턱을 넘는다 — 믿지 말 것
    'data_final_final_final': (0.099, 0.078, 6.8, 0.047),
}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))



def parse_args(ap):
    """ROS 인자(--ros-args …)와 우리 인자를 갈라서 읽는다.

    argparse 는 --ros-args 를 모르는 인자로 보고 거절한다. 그래서 rclpy 의 유틸로
    ROS 인자를 먼저 걷어낸 뒤 나머지만 argparse 에 준다.
    ROS 인자는 rclpy.init(args=sys.argv) 로 그대로 넘겨야 use_sim_time 같은 게 먹는다.
    """
    from rclpy.utilities import remove_ros_args
    return ap.parse_args(remove_ros_args(sys.argv)[1:])

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pose', default='', help='"x,y,yaw" 부팅 자세 (지도 좌표, 각도는 도)')
    ap.add_argument('--bag', default='', help='아는 녹화본 이름이면 자세를 찾아 쓴다')
    ap.add_argument('--odom-topic', default='/Odometry')
    ap.add_argument('--wait', type=float, default=25.0, help='오도메트리를 기다릴 시간')
    ap.add_argument('--times', type=int, default=3, help='몇 번 보낼지 (놓치지 않게)')
    ap.add_argument('--no-odom', action='store_true',
                    help='오도메트리를 안 기다리고 그대로 보낸다 (로봇이 아직 안 움직였을 때만)')
    a = parse_args(ap)

    if a.bag:
        key = os.path.basename(a.bag.rstrip('/'))
        if key not in KNOWN:
            raise SystemExit('★ 모르는 녹화본이다: %s\n  아는 것: %s\n  --pose 로 직접 줄 것'
                             % (key, ', '.join(sorted(KNOWN))))
        x, y, yd, rms = KNOWN[key]
        print('  %s 의 알려진 부팅 자세: (%.3f, %.3f, %.1f°)  당시 잔차 %.3f m' % (key, x, y, yd, rms))
        if rms > 0.06:
            print('  ★ 잔차 %.3f m 는 문턱(0.06)을 넘는다 — 이 자세는 확실하지 않다' % rms)
    elif a.pose:
        v = [float(t) for t in a.pose.split(',')]
        if len(v) != 3:
            raise SystemExit('★ --pose 는 "x,y,yaw" 세 값이어야 한다')
        x, y, yd = v
    else:
        raise SystemExit('★ --pose 나 --bag 중 하나가 필요하다')

    rclpy.init(args=sys.argv)
    n = Node('publish_initialpose')
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
    box = {}

    def on_odom(m):
        p = m.pose.pose.position
        box['v'] = (p.x, p.y, yaw_of(m.pose.pose.orientation))

    if not a.no_odom:
        n.create_subscription(Odometry, a.odom_topic, on_odom, qos)
        t0 = time.monotonic()
        while 'v' not in box and time.monotonic() - t0 < a.wait:
            rclpy.spin_once(n, timeout_sec=0.2)
        if 'v' not in box:
            print('  ★ %s 를 %.0f초 동안 못 받았다 — 오도메트리 없이 그대로 보낸다'
                  % (a.odom_topic, a.wait))

    th = math.radians(yd)
    if 'v' in box:
        ox, oy, oyaw = box['v']
        c, s = math.cos(th), math.sin(th)
        bx = x + c * ox - s * oy          # map->base_link = (map->odom) ⊗ (odom->base_link)
        by = y + s * ox + c * oy
        byaw = th + oyaw
        print('  오도메트리 (%.3f, %.3f, %.1f°) 를 반영 -> 보낼 자세 (%.3f, %.3f, %.1f°)'
              % (ox, oy, math.degrees(oyaw), bx, by, math.degrees(byaw)))
    else:
        bx, by, byaw = x, y, th

    pub = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    m = PoseWithCovarianceStamped()
    m.header.frame_id = 'map'
    m.pose.pose.position.x = bx
    m.pose.pose.position.y = by
    m.pose.pose.orientation.z = math.sin(byaw / 2)
    m.pose.pose.orientation.w = math.cos(byaw / 2)
    m.pose.covariance[0] = m.pose.covariance[7] = 0.05
    m.pose.covariance[35] = 0.02
    for _ in range(a.times):
        m.header.stamp = n.get_clock().now().to_msg()
        pub.publish(m)
        rclpy.spin_once(n, timeout_sec=0.3)
        time.sleep(0.3)
    print('  /initialpose 로 %d번 보냈다' % a.times)
    n.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
