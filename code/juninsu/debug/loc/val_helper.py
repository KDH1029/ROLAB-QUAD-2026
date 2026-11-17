#!/usr/bin/env python3
"""stop_loc_node 재생 검증 도우미: /Odometry → 상황대로 틀어 /Odometry_c, 정해진 시각(첫 자세 기준 초)에 /mission/realign,
/mission/align_T 와 /mission/loc_stop 을 CSV/로그로. (재생 도메인 안, use_sim_time)"""
import json
import math
import os
import sys

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scen import Corrupt  # noqa: E402


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class V(Node):
    def __init__(self):
        super().__init__('val_helper')
        self.declare_parameter('scen', 'S0'); self.declare_parameter('out', '/tmp/val.csv'); self.declare_parameter('realign_at', [39.0, 148.0, 205.0])
        self.cor = Corrupt(self.get_parameter('scen').value); self.at = list(self.get_parameter('realign_at').value)
        self.f = open(self.get_parameter('out').value, 'w'); self.f.write('kind,t,a,b,c,d\n'); self.t0 = None; self.now = None
        self.pub = self.create_publisher(Odometry, '/Odometry_c', 50)
        self.pub_r = self.create_publisher(String, '/mission/realign', 2)
        tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, '/Odometry', self.cb, 50)
        self.create_subscription(String, '/mission/align_T', self.cb_T, tl)
        self.create_subscription(String, '/mission/loc_stop', lambda m: self.f.write(f'log,{self.now},{json.loads(m.data)["msg"]!r},,,\n'), 10)

    def cb(self, m):
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = st
        self.now = st; p, q = m.pose.pose.position, m.pose.pose.orientation
        x, y, a = self.cor.apply(st - self.t0, p.x, p.y, yaw_of(q))
        d = a - yaw_of(q); cz, sz = math.cos(d / 2), math.sin(d / 2)
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        w, qx, qy, qz = q.w, q.x, q.y, q.z
        m.pose.pose.orientation.w, m.pose.pose.orientation.x = cz * w - sz * qz, cz * qx - sz * qy
        m.pose.pose.orientation.y, m.pose.pose.orientation.z = cz * qy + sz * qx, cz * qz + sz * w
        self.pub.publish(m)
        if self.at and st - self.t0 >= self.at[0]:
            self.pub_r.publish(String(data=json.dumps({'max_xy': 0.5, 'max_yaw': 20, 'why': 'park'})))
            self.f.write(f'req,{st},{st - self.t0:.1f},,,\n'); self.at.pop(0)

    def cb_T(self, m):
        d = json.loads(m.data); T = d['T']
        self.f.write(f"T,{self.now},{T[0]},{T[1]},{T[2]},{int(bool(d.get('ok')))}\n"); self.f.flush()


def main():
    rclpy.init(); n = V()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.f.close()


if __name__ == '__main__':
    main()
