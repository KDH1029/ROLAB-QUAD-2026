#!/usr/bin/env python3
"""AMCL 재생 시험용 중계 — 재생한 /Odometry 를 상황(S0/S1/S2)대로 틀어 /tf_amcl 에 lio_odom→base_link 로 내고,
/tf 의 base_link→base_stab(스캔 프레임)을 복사, 출발 3초 뒤 /initialpose(= 출발 T · 지금 자세) 한 번, /amcl_pose 를 CSV 로 남김.
사용(재생 도메인 안에서): python3 amcl_relay.py --ros-args -p scen:=S1 -p T:="[x,y,yaw]" -p csv:=out.csv -p use_sim_time:=true"""
import math
import os
import sys

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scen import Corrupt  # noqa: E402


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Relay(Node):
    def __init__(self):
        super().__init__('amcl_relay')
        self.declare_parameter('scen', 'S0'); self.declare_parameter('T', [0.0, 0.0, 0.0]); self.declare_parameter('csv', '/tmp/amcl.csv')
        self.cor = Corrupt(self.get_parameter('scen').value); self.T = list(self.get_parameter('T').value)
        self.f = open(self.get_parameter('csv').value, 'w'); self.f.write('kind,t,x,y,yaw\n')
        self.t0 = None; self.init_sent = False
        self.pub_tf = self.create_publisher(TFMessage, '/tf_amcl', 50)
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, '/Odometry', self.cb_odom, 50)
        self.create_subscription(TFMessage, '/tf', self.cb_tf, 100)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.cb_amcl, 10)

    def cb_tf(self, m):
        out = [t for t in m.transforms if t.header.frame_id == 'base_link' and t.child_frame_id == 'base_stab']
        if out:
            self.pub_tf.publish(TFMessage(transforms=out))

    def cb_odom(self, m):
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = st
        p, q = m.pose.pose.position, m.pose.pose.orientation
        x, y, yaw = self.cor.apply(st - self.t0, p.x, p.y, yaw_of(q))
        # 기울기(roll·pitch)는 그대로 두고 yaw 만 바꿈: q' = Rz(dyaw) · q
        dy_ = yaw - yaw_of(q); cz, sz = math.cos(dy_ / 2), math.sin(dy_ / 2)
        qw, qx, qy, qz = cz * q.w - sz * q.z, cz * q.x - sz * q.y, cz * q.y + sz * q.x, cz * q.z + sz * q.w
        t = TransformStamped(); t.header.stamp = m.header.stamp; t.header.frame_id = 'lio_odom'; t.child_frame_id = 'base_link'
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, p.z
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = qx, qy, qz, qw
        self.pub_tf.publish(TFMessage(transforms=[t]))
        self.f.write(f'odom,{st:.3f},{x:.4f},{y:.4f},{yaw:.5f}\n')
        if not self.init_sent and st - self.t0 >= 3.0:
            c, s = math.cos(self.T[2]), math.sin(self.T[2])
            i = PoseWithCovarianceStamped(); i.header.stamp = m.header.stamp; i.header.frame_id = 'map'
            i.pose.pose.position.x = c * x - s * y + self.T[0]; i.pose.pose.position.y = s * x + c * y + self.T[1]
            a = yaw + self.T[2]; i.pose.pose.orientation.z = math.sin(a / 2); i.pose.pose.orientation.w = math.cos(a / 2)
            cov = [0.0] * 36; cov[0] = cov[7] = 0.05 ** 2; cov[35] = math.radians(3) ** 2; i.pose.covariance = cov
            self.pub_init.publish(i); self.init_sent = True
            self.get_logger().info(f'initialpose ({i.pose.pose.position.x:.2f}, {i.pose.pose.position.y:.2f}, {math.degrees(a):.1f}°)')

    def cb_amcl(self, m):
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9; p = m.pose.pose.position
        self.f.write(f'amcl,{st:.3f},{p.x:.4f},{p.y:.4f},{yaw_of(m.pose.pose.orientation):.5f}\n'); self.f.flush()


def main():
    rclpy.init(); n = Relay()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.f.close()


if __name__ == '__main__':
    main()
