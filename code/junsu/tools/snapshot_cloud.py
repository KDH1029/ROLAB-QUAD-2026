#!/usr/bin/env python3
"""/cloud_registered를 일정 간격(벽시계)으로 스냅샷 떠서 npy로 저장.
계단 구간 기하 분석용 — 전체를 다 저장하면 너무 크니 샘플링."""
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class Snapshotter(Node):
    def __init__(self, out_dir, interval_s):
        super().__init__('cloud_snapshotter')
        self.out_dir = out_dir
        self.interval = interval_s
        os.makedirs(out_dir, exist_ok=True)
        self.t_wall0 = None
        self.last_save = -999.0
        self.idx = 0
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        self.sub = self.create_subscription(
            PointCloud2, '/cloud_registered', self.cb, qos)
        self.get_logger().info(f'watching /cloud_registered -> {out_dir}')

    def cb(self, msg):
        now = time.monotonic()
        if self.t_wall0 is None:
            self.t_wall0 = now
        t = now - self.t_wall0
        if t - self.last_save < self.interval:
            return
        self.last_save = t
        pts = point_cloud2.read_points_numpy(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        fname = os.path.join(self.out_dir, f'cloud_{self.idx:03d}_t{t:.1f}.npy')
        np.save(fname, pts.astype(np.float32))
        self.get_logger().info(f'saved {fname} ({pts.shape[0]} pts)')
        self.idx += 1


def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else '/tmp/cloud_snaps'
    interval = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0
    rclpy.init()
    node = Snapshotter(out_dir, interval)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
