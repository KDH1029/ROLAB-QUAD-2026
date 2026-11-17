#!/usr/bin/env python3
"""정합 결과를 팀원 계획기가 받는 모양으로 바꿔 준다.

받는 것
  /Odometry            nav_msgs/Odometry   FAST-LIO 출력 (프레임 camera_init)
  TF map -> camera_init                    icp_localization 이 내는 보정

내는 것
  /Odometry_map        nav_msgs/Odometry   map 프레임 절대 오도메트리

【왜 필요한가】
팀원 스택은 `/Odometry_map` 을 받아 주행한다 (go2_direct_navigator, udp_goal_bridge).
그건 `go2_3d_loc_node.py` 가 내는 토픽이다.
내 `icp_localization.py` 는 TF 만 내고 토픽은 안 낸다. 그래서 그 자리를 대신하려면
TF 와 오도메트리를 합쳐 같은 토픽을 내 줘야 한다. 이 노드가 그 한 겹이다.

    map->base_link = (map->camera_init) ⊗ (camera_init->base_link)

【주의 — 프레임 이름】
FAST-LIO 의 고정 프레임은 `odom` 이 아니라 **`camera_init`** 이다.
그래서 icp_localization 을 띄울 때 `--odom-frame camera_init` 을 줘야 한다.
안 그러면 TF 사슬이 안 이어지고 이 노드는 아무것도 못 낸다.

【이 노드는 주행 명령을 내지 않는다】
좌표만 바꿔 전달한다. cmd_vel 도 goal 도 건드리지 않는다.
"""
import argparse
import math
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
import tf2_ros


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def rot_apply(q, v):
    w, x, y, z = q
    t = (2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0]))
    return (v[0] + w * t[0] + (y * t[2] - z * t[1]),
            v[1] + w * t[1] + (z * t[0] - x * t[2]),
            v[2] + w * t[2] + (x * t[1] - y * t[0]))


def parse_args(ap):
    from rclpy.utilities import remove_ros_args
    return ap.parse_args(remove_ros_args(sys.argv)[1:])


class OdometryMapAdapter(Node):

    def __init__(self, a):
        super().__init__('odometry_map_adapter')
        self.a = a
        self.n_in = self.n_out = 0
        self.warned = False
        self.buf = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=5))
        self.lis = tf2_ros.TransformListener(self.buf, self)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Odometry, a.in_topic, self.on_odom, qos)
        self.pub = self.create_publisher(Odometry, a.out_topic, 10)
        self.create_timer(5.0, self.report)
        self.get_logger().info('%s + TF(%s->%s) -> %s'
                               % (a.in_topic, a.map_frame, a.odom_frame, a.out_topic))

    def on_odom(self, m):
        self.n_in += 1
        try:
            tf = self.buf.lookup_transform(self.a.map_frame, self.a.odom_frame,
                                           rclpy.time.Time())
        except Exception as e:
            if not self.warned:
                self.warned = True
                self.get_logger().warn('TF %s -> %s 가 아직 없다 (%s). '
                                       'icp_localization 이 초기자세를 받았는가?'
                                       % (self.a.map_frame, self.a.odom_frame, str(e)[:70]))
            return

        q = tf.transform.rotation
        t = tf.transform.translation
        qm = (q.w, q.x, q.y, q.z)
        p = m.pose.pose.position
        o = m.pose.pose.orientation
        # 자세 합성: map->base = (map->odom) ⊗ (odom->base)
        rp = rot_apply(qm, (p.x, p.y, p.z))
        qo = quat_mul(qm, (o.w, o.x, o.y, o.z))

        out = Odometry()
        out.header.stamp = m.header.stamp
        out.header.frame_id = self.a.map_frame
        out.child_frame_id = m.child_frame_id or 'base_link'
        out.pose.pose.position.x = rp[0] + t.x
        out.pose.pose.position.y = rp[1] + t.y
        out.pose.pose.position.z = rp[2] + t.z
        out.pose.pose.orientation.w = qo[0]
        out.pose.pose.orientation.x = qo[1]
        out.pose.pose.orientation.y = qo[2]
        out.pose.pose.orientation.z = qo[3]
        # 속도는 로봇 몸통 기준이라 좌표계를 바꿔도 그대로다
        out.twist = m.twist
        out.pose.covariance = m.pose.covariance
        self.pub.publish(out)
        self.n_out += 1

    def report(self):
        if self.n_out:
            self.get_logger().info('받음 %.1f Hz  -> 냄 %.1f Hz' % (self.n_in / 5.0, self.n_out / 5.0))
        else:
            self.get_logger().warn('받음 %.1f Hz 인데 하나도 못 냈다 — TF %s->%s 를 확인할 것'
                                   % (self.n_in / 5.0, self.a.map_frame, self.a.odom_frame))
        self.n_in = self.n_out = 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--in-topic', default='/Odometry')
    ap.add_argument('--out-topic', default='/Odometry_map')
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--odom-frame', default='camera_init',
                    help='FAST-LIO 의 고정 프레임. odom 이 아니라 camera_init 이다')
    a = parse_args(ap)
    rclpy.init(args=sys.argv)
    n = OdometryMapAdapter(a)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    n.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
