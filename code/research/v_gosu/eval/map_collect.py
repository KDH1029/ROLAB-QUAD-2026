#!/usr/bin/env python3
"""재생 중 FAST-LIO 결과를 지도 그림용으로 모은다.

기록: /cloud_registered (월드 좌표 등록 스캔) → 5 cm 복셀 집합과 복셀별 점 수
      /Odometry → 자세 전체 (header 시각, 수신 시각)
      /lidar_points → (수신 시각, 원본 header 시각)  — 재생 시각을 녹화 시각으로 되돌리는 데 쓴다
사용: map_collect.py <out.npz>
"""
import sys, time, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

OUT = sys.argv[1]
V = float(sys.argv[2]) if len(sys.argv) > 2 else 0.05   # 복셀 크기 (기본 5 cm)
OFF = 1 << 20                            # 음수 칸을 양수 키로


class Collect(Node):
    def __init__(self):
        super().__init__('v_gosu_map_collect')
        q = QoSProfile(depth=500, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        be = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.odom, self.lid = [], []
        self.K = np.zeros(0, np.int64); self.C = np.zeros(0, np.int64); self.buf = []; self.ncloud = 0
        self.create_subscription(Odometry, '/Odometry', self.cb_odom, q)
        self.create_subscription(PointCloud2, '/cloud_registered', self.cb_cloud, q)
        self.create_subscription(PointCloud2, '/lidar_points', self.cb_lid, be)

    def cb_odom(self, m):
        p, o, s = m.pose.pose.position, m.pose.pose.orientation, m.header.stamp
        self.odom.append((time.time(), s.sec + s.nanosec * 1e-9, p.x, p.y, p.z, o.x, o.y, o.z, o.w))

    def cb_lid(self, m):
        s = m.header.stamp
        self.lid.append((time.time(), s.sec + s.nanosec * 1e-9))

    def cb_cloud(self, m):
        self.ncloud += 1
        a = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True)
        if a.size == 0:
            return
        i = np.floor(a / V).astype(np.int64) + OFF
        self.buf.append((i[:, 0] << 42) | (i[:, 1] << 21) | i[:, 2])
        if len(self.buf) >= 100:
            self.reduce()

    def reduce(self):
        if not self.buf:
            return
        k = np.concatenate(self.buf); self.buf = []
        u, c = np.unique(k, return_counts=True)
        allk = np.concatenate([self.K, u]); allc = np.concatenate([self.C, c])
        self.K, inv = np.unique(allk, return_inverse=True)
        self.C = np.bincount(inv, weights=allc).astype(np.int64)

    def save(self):
        self.reduce()
        ix = (self.K >> 42) - OFF; iy = ((self.K >> 21) & ((1 << 21) - 1)) - OFF; iz = (self.K & ((1 << 21) - 1)) - OFF
        np.savez_compressed(OUT, vox=np.stack([ix, iy, iz], 1).astype(np.int32), cnt=self.C.astype(np.int32), V=V,
                            odom=np.array(self.odom), lid=np.array(self.lid))
        self.get_logger().info(f'저장 {OUT}  복셀 {len(self.K)}  odom {len(self.odom)}  lidar {len(self.lid)}  cloud {self.ncloud}')


def main():
    rclpy.init(); n = Collect()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        n.save()
        try: rclpy.shutdown()
        except Exception: pass


if __name__ == '__main__':
    main()
