#!/usr/bin/env python3
"""arena_0921 재생 중 /sportmodestate를 CSV로 로깅 — 계단/경사 구간 위치 특정용.

msg.stamp(TimeSpec, 로봇 자체 시계)는 구간에 따라 역행/점프가 있어 못 믿는다.
대신 도착 시점의 벽시계를 쓰고, --rate 로 나눠서 원본 bag 시간으로 환산한다.
드롭 방지를 위해 QoS depth를 키우고, 파일 I/O는 메모리에 버퍼링했다가 끝에 한번에 쓴다.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from unitree_go.msg import SportModeState


class Logger(Node):
    def __init__(self, out_path, rate):
        super().__init__('sportstate_logger')
        self.out_path = out_path
        self.rate = rate
        self.rows = []
        self.t_wall0 = None
        self.n = 0
        self.n_expected_total = 262327  # arena_0921 전체 메시지 수 (ros2 bag info 기준)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5000,
        )
        self.sub = self.create_subscription(
            SportModeState, '/sportmodestate', self.cb, qos)

    def cb(self, msg):
        now = time.monotonic()
        if self.t_wall0 is None:
            self.t_wall0 = now
        t_bag = (now - self.t_wall0) * self.rate
        rpy = msg.imu_state.rpy
        self.rows.append((
            f'{t_bag:.3f}', msg.mode, msg.gait_type,
            f'{msg.body_height:.4f}',
            f'{msg.position[0]:.4f}', f'{msg.position[1]:.4f}', f'{msg.position[2]:.4f}',
            f'{rpy[0]:.4f}', f'{rpy[1]:.4f}', f'{rpy[2]:.4f}',
            msg.foot_force[0], msg.foot_force[1], msg.foot_force[2], msg.foot_force[3],
        ))
        self.n += 1
        if self.n % 20000 == 0:
            self.get_logger().info(f'{self.n} rows @ t_bag~{t_bag:.1f}s')

    def close(self):
        import csv
        with open(self.out_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t_bag_sec', 'mode', 'gait_type', 'body_height',
                        'x', 'y', 'z', 'roll', 'pitch', 'yaw',
                        'ff0', 'ff1', 'ff2', 'ff3'])
            w.writerows(self.rows)
        pct = 100.0 * self.n / self.n_expected_total
        self.get_logger().info(
            f'done: {self.n}/{self.n_expected_total} rows ({pct:.1f}%) -> {self.out_path}')


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/sportstate.csv'
    rate = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    rclpy.init()
    node = Logger(out_path, rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
