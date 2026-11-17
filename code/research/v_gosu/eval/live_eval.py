#!/usr/bin/env python3
"""재생 중 오도메트리를 같은 기준으로 기록한다 (FAST-LIO / FAST-LIVO2 공통).

기록: 오도메트리 궤적, IMU 가속도(충격 판정용), 다리 오도(/utlidar/robot_odom),
      바닥 점의 칸별 높이 통계(재방문 일관성).
사용: live_eval.py <odom_topic> <cloud_topic> <out_prefix>
"""
import sys, time, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

ODOM, CLOUD, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
C, LIM = 0.05, 20.0                     # 5 cm 칸, ±20 m
N = int(2 * LIM / C)


class Eval(Node):
    def __init__(self):
        super().__init__('v_gosu_eval')
        q = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        be = QoSProfile(depth=500, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.odom, self.leg, self.imu = [], [], []
        self.cnt = np.zeros(N * N, np.int32); self.s = np.zeros(N * N); self.ss = np.zeros(N * N)
        self.t0 = np.full(N * N, np.inf); self.t1 = np.full(N * N, -np.inf)
        self.floor = None; self.floor_buf = []; self.ncloud = 0
        self.create_subscription(Odometry, ODOM, self.cb_odom, q)
        self.create_subscription(Odometry, '/utlidar/robot_odom', self.cb_leg, be)
        self.create_subscription(Imu, '/imu/body', self.cb_imu, be)
        self.create_subscription(PointCloud2, CLOUD, self.cb_cloud, q)

    @staticmethod
    def now():
        return time.monotonic()

    def cb_odom(self, m):
        p, o = m.pose.pose.position, m.pose.pose.orientation
        self.odom.append((self.now(), p.x, p.y, p.z, o.x, o.y, o.z, o.w))

    def cb_leg(self, m):
        p = m.pose.pose.position
        self.leg.append((self.now(), p.x, p.y, p.z))

    def cb_imu(self, m):
        a = m.linear_acceleration
        self.imu.append((self.now(), a.x, a.y, a.z))

    def cb_cloud(self, m):
        t = self.now(); self.ncloud += 1
        a = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)
        if a.size == 0:
            return
        if self.floor is None:                       # 처음 5초: 로봇 3 m 안쪽 낮은 점으로 바닥 높이 추정
            near = a[np.hypot(a[:, 0], a[:, 1]) < 3.0]
            if len(near): self.floor_buf.append(np.percentile(near[:, 2], 5))
            if len(self.floor_buf) >= 100:
                self.floor = float(np.median(self.floor_buf))
            return
        b = a[np.abs(a[:, 2] - self.floor) < 0.25]   # 바닥 띠 ±25 cm
        ix = ((b[:, 0] + LIM) / C).astype(np.int64); iy = ((b[:, 1] + LIM) / C).astype(np.int64)
        ok = (ix >= 0) & (ix < N) & (iy >= 0) & (iy < N)
        k = ix[ok] * N + iy[ok]; z = b[ok, 2]
        np.add.at(self.cnt, k, 1); np.add.at(self.s, k, z); np.add.at(self.ss, k, z * z)
        np.minimum.at(self.t0, k, t); np.maximum.at(self.t1, k, t)

    def save(self):
        np.savez_compressed(OUT + '.npz', odom=np.array(self.odom), leg=np.array(self.leg), imu=np.array(self.imu),
                            cnt=self.cnt, s=self.s, ss=self.ss, t0=self.t0, t1=self.t1,
                            floor=np.array([self.floor if self.floor is not None else np.nan]), C=C, LIM=LIM)
        self.get_logger().info(f'저장 {OUT}.npz  odom {len(self.odom)}  leg {len(self.leg)}  imu {len(self.imu)}  cloud {self.ncloud}')


def main():
    rclpy.init(); n = Eval()
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
