#!/usr/bin/env python3
"""ego 진단 기록기(xentry, 시뮬 도메인 전용): /ego_planner/trajectory · /ego_planner/grid_map(입구 상자만) · /goal_pose · /Odometry(10Hz)
· /scan(입구 근처에 있을 때만) · /mission/wall_memory · /mission/runner_status 를 JSON 줄로 저장. SIGTERM/SIGINT 로 끝.
사용: ego_rec.py <out.jsonl> [x0 x1 y0 y1]"""
import json, sys, signal, time, math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, LaserScan
from std_msgs.msg import String
from sensor_msgs_py import point_cloud2 as pc2

OUT = sys.argv[1]; BOX = [float(v) for v in sys.argv[2:6]] if len(sys.argv) >= 6 else [0.0, 4.2, -0.8, 2.2]


class Rec(Node):
    def __init__(self):
        super().__init__('xentry_ego_rec')
        self.f = open(OUT, 'w'); self.t0 = time.monotonic(); self.pose = None; self.last_odom = 0.0; self.last_scan = 0.0; self.last_grid = 0.0
        self.create_subscription(Path, '/ego_planner/trajectory', self.cb_path, 10)
        self.create_subscription(PointCloud2, '/ego_planner/grid_map', self.cb_grid, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.cb_goal, 10)
        self.create_subscription(Odometry, '/Odometry', self.cb_odom, 20)
        self.create_subscription(LaserScan, '/scan', self.cb_scan, 10)
        self.create_subscription(PointCloud2, '/mission/wall_memory', self.cb_wm, 10)
        self.create_subscription(String, '/mission/runner_status', self.cb_rs, 5)

    def w(self, ev, **k):
        k['ev'] = ev; k['tw'] = round(time.time(), 3); self.f.write(json.dumps(k) + '\n')

    def near(self):
        return self.pose is not None and BOX[0] < self.pose[0] < BOX[1] and BOX[2] < self.pose[1] < BOX[3]

    def cb_odom(self, m):
        p = m.pose.pose; q = p.orientation; yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.position.x, p.position.y, yaw)
        if time.monotonic() - self.last_odom >= 0.1:
            self.last_odom = time.monotonic(); self.w('odom', x=round(p.position.x, 3), y=round(p.position.y, 3), yaw=round(yaw, 3))

    def cb_path(self, m):
        self.w('path', p=[[round(s.pose.position.x, 3), round(s.pose.position.y, 3)] for s in m.poses])

    def cb_grid(self, m):
        if not self.near() or time.monotonic() - self.last_grid < 0.25:
            return
        self.last_grid = time.monotonic()
        P = np.array([[p[0], p[1]] for p in pc2.read_points(m, field_names=('x', 'y'), skip_nans=True)], dtype=float).reshape(-1, 2)
        k = (P[:, 0] > BOX[0]) & (P[:, 0] < BOX[1]) & (P[:, 1] > BOX[2]) & (P[:, 1] < BOX[3])
        self.w('grid', p=np.round(P[k], 3).tolist())

    def cb_scan(self, m):
        if not self.near() or time.monotonic() - self.last_scan < 0.5:
            return
        self.last_scan = time.monotonic(); x, y, yaw = self.pose
        r = np.array(m.ranges); a = m.angle_min + np.arange(len(r)) * m.angle_increment; ok = np.isfinite(r)
        P = np.c_[x + r[ok] * np.cos(a[ok] + yaw), y + r[ok] * np.sin(a[ok] + yaw)]
        self.w('scan', robot=[round(x, 3), round(y, 3), round(yaw, 3)], p=np.round(P, 3).tolist())

    def cb_wm(self, m):
        if not self.near():
            return
        P = np.array([[p[0], p[1]] for p in pc2.read_points(m, field_names=('x', 'y'), skip_nans=True)], dtype=float).reshape(-1, 2)
        self.w('wm', p=np.round(P, 3).tolist())

    def cb_goal(self, m):
        self.w('goal', x=round(m.pose.position.x, 3), y=round(m.pose.position.y, 3))

    def cb_rs(self, m):
        self.w('rs', s=m.data[:200])


def main():
    rclpy.init(); n = Rec()
    def stop(*_):
        n.f.flush(); n.f.close(); rclpy.shutdown() if rclpy.ok() else None; sys.exit(0)
    signal.signal(signal.SIGTERM, stop); signal.signal(signal.SIGINT, stop)
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.2); n.f.flush()
    except SystemExit:
        pass


if __name__ == '__main__':
    main()
