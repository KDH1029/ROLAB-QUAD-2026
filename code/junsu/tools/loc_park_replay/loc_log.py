#!/usr/bin/env python3
"""재생 기록 — /mission/loc_watch(상태)·/mission/align_T(T 바뀜)·/junsu/amcl_pose·/Odometry·몸 높이를 CSV 로(시각 = bag 시계)."""
import json, math, sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from unitree_go.msg import SportModeState


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Log(Node):
    def __init__(self):
        super().__init__('loc_log')
        self.f = open(self.declare_parameter('csv', '/tmp/loc_log.csv').value, 'w'); self.h = None; self.lio = None
        lat = QoSProfile(depth=5, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(String, '/mission/loc_watch', lambda m: self.w('watch', json.loads(m.data).get('state'), json.dumps(json.loads(m.data).get('T'))), 10)
        self.create_subscription(String, '/mission/align_T', lambda m: self.w('T', json.loads(m.data).get('reason'), json.dumps(json.loads(m.data).get('T'))), lat)
        self.create_subscription(PoseWithCovarianceStamped, '/junsu/amcl_pose', lambda m: self.w('amcl', f'{m.pose.pose.position.x:.4f}', f'{m.pose.pose.position.y:.4f}', f'{yaw(m.pose.pose.orientation):.5f}'), 10)
        self.create_subscription(Odometry, '/Odometry', self.cb_odom, 20)
        self.create_subscription(SportModeState, '/sportmodestate', lambda m: setattr(self, 'h', m.body_height), qos_profile_sensor_data)

    def cb_odom(self, m):
        p = m.pose.pose.position; self.w('lio', f'{p.x:.4f}', f'{p.y:.4f}', f'{yaw(m.pose.pose.orientation):.5f}', f'{p.z:.4f}')

    def w(self, *a):
        t = self.get_clock().now().nanoseconds * 1e-9
        self.f.write(','.join([f'{t:.3f}', f'{self.h if self.h is not None else -1:.3f}'] + [str(x) for x in a]) + '\n'); self.f.flush()


def main():
    rclpy.init(); n = Log()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
