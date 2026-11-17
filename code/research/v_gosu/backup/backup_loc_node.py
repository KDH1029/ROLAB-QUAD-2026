#!/usr/bin/env python3
"""백업 위치추정 노드 — FAST-LIO 없이 '다리 오도메트리 + 3D 지도 정합(수평 세우기)'.

받는 것   /utlidar/robot_odom (다리 기구학 150 Hz)   /lidar_points (XT16 원본, 원시 바이트로 받아 5 Hz 분만 해석)
          /Odometry (FAST-LIO) — 시작 자세를 한 번 받는 데만 씀. 없으면 원점
내는 것   /backup/odometry   nav_msgs/Odometry  (지도 좌표 = camera_init, 다리 갱신마다 50 Hz)
          /backup/cloud      sensor_msgs/PointCloud2 (지도 좌표 스캔, 정합마다 5 Hz) — 전환 시 플래너 장애물
          /backup/status     std_msgs/String (JSON: health ok/degraded/lost, rms, 수락/거부)
계산은 backup_core.BackupEstimator — test_offline.py 로 녹화본 검증한 것과 같은 코드.
사용: backup_loc_node.py --map <지도.npz|.pcd> [--hz 5] [--init fastlio|origin]
"""
import argparse, json, math, threading, time, sys, os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
import sensor_msgs_py.point_cloud2 as pc2
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from backup_core import BackupEstimator, load_map, q2R, rpy_of


class BackupLoc(Node):
    def __init__(self, a):
        super().__init__('backup_localization')
        t0 = time.time()
        self.est = BackupEstimator(load_map(os.path.expanduser(a.map)))
        self.get_logger().info(f'지도 {a.map} 적재 ({time.time() - t0:.1f}s), 정합 {a.hz} Hz, 시작 자세 {a.init}')
        self.hz, self.init_mode, self.frame = a.hz, a.init, a.map_frame
        self.lock = threading.Lock()
        self.leg = None; self.fl0 = None; self.t_first_leg = None
        self.last_scan = 0.0; self.busy = False; self.n_leg = 0
        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        g_leg, g_scan = MutuallyExclusiveCallbackGroup(), MutuallyExclusiveCallbackGroup()
        self.create_subscription(Odometry, '/utlidar/robot_odom', self.on_leg, qos_profile_sensor_data, callback_group=g_leg)
        self.create_subscription(PointCloud2, a.cloud, self.on_cloud, qos_profile_sensor_data, callback_group=g_scan, raw=True)
        self.create_subscription(Odometry, '/Odometry', self.on_fastlio, rel, callback_group=g_leg)
        self.pub_odom = self.create_publisher(Odometry, '/backup/odometry', rel)
        self.pub_cloud = self.create_publisher(PointCloud2, '/backup/cloud', rel)
        self.pub_stat = self.create_publisher(String, '/backup/status', rel)
        self.create_timer(1.0, self.report, callback_group=g_leg)

    def on_fastlio(self, m):
        if self.fl0 is None:
            p, q = m.pose.pose.position, m.pose.pose.orientation
            self.fl0 = (p.x, p.y, p.z, rpy_of(q2R(q.x, q.y, q.z, q.w))[2])

    def on_leg(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        r, pi, y = rpy_of(q2R(q.x, q.y, q.z, q.w)); leg = (p.x, p.y, p.z, y, r, pi)
        now = self.get_clock().now()
        with self.lock:
            self.leg = leg
            if not self.est.inited:
                # FAST-LIO 첫 자세를 첫 다리 메시지부터 5초 기다린다(재생·실기 모두 같은 기준). 없으면 원점
                self.t_first_leg = self.t_first_leg or time.monotonic()
                waited = time.monotonic() - self.t_first_leg
                if self.init_mode == 'fastlio' and self.fl0 is not None:
                    self.est.init(*self.fl0, leg, now.nanoseconds * 1e-9); self.get_logger().info(f'시작 자세 = FAST-LIO {self.fl0}')
                elif self.init_mode == 'origin' or waited > 5.0:
                    self.est.init(0.0, 0.0, 0.0, 0.0, leg, now.nanoseconds * 1e-9); self.get_logger().info('시작 자세 = 원점')
                return
            self.est.on_leg(leg)
            x, y, z, rr, pp, yy = self.est.pose()
        self.n_leg += 1
        if self.n_leg % 3: return                      # 150 Hz → 50 Hz 발행
        o = Odometry(); o.header.stamp = now.to_msg(); o.header.frame_id = self.frame; o.child_frame_id = 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = x, y, z
        cr, sr, cp, sp, cy, sy = math.cos(rr / 2), math.sin(rr / 2), math.cos(pp / 2), math.sin(pp / 2), math.cos(yy / 2), math.sin(yy / 2)
        o.pose.pose.orientation.w = cr * cp * cy + sr * sp * sy; o.pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
        o.pose.pose.orientation.y = cr * sp * cy + sr * cp * sy; o.pose.pose.orientation.z = cr * cp * sy - sr * sp * cy
        o.twist = m.twist                              # 다리 오도메트리 속도(몸체 좌표) 그대로
        self.pub_odom.publish(o)

    def on_cloud(self, raw):
        now = time.monotonic()
        if now - self.last_scan < 0.95 / self.hz or not self.est.inited:
            return                                       # 원시 바이트만 받고 버림 — 해석 비용 없음
        self.last_scan = now
        m = deserialize_message(raw, PointCloud2)
        pts = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)
        with self.lock:
            prep = self.est.prepare(pts)
        if prep is None: return
        T, rms = self.est.matcher.align(prep['P'], prep['T0'])      # 잠금 밖 — 다리 갱신은 계속 들어온다
        t = self.get_clock().now()
        with self.lock:
            res = self.est.apply(prep, T, rms, t.nanoseconds * 1e-9)
        w = res['world'].astype(np.float32)
        c = PointCloud2(); c.header.stamp = t.to_msg(); c.header.frame_id = self.frame
        c.height, c.width = 1, len(w); c.point_step, c.row_step = 12, 12 * len(w); c.is_dense = True
        c.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyz')]
        c.data = w.tobytes(); self.pub_cloud.publish(c)

    def report(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        with self.lock:
            e = self.est; x, y, z, _, _, yy = e.pose()
            s = dict(health=e.health(t), rms=None if not np.isfinite(e.last_rms) else round(e.last_rms, 3), ok=e.n_ok, bad=e.n_bad,
                     x=round(x, 3), y=round(y, 3), z=round(z, 3), yaw_deg=round(math.degrees(yy), 1),
                     since_good_s=None if e.t_good is None else round(t - e.t_good, 1))
        self.pub_stat.publish(String(data=json.dumps(s, ensure_ascii=False)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True); ap.add_argument('--hz', type=float, default=5.0)
    ap.add_argument('--init', choices=['fastlio', 'origin'], default='fastlio')
    ap.add_argument('--cloud', default='/lidar_points'); ap.add_argument('--map-frame', default='camera_init')
    a, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args); n = BackupLoc(a)
    ex = MultiThreadedExecutor(num_threads=3); ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        try: rclpy.shutdown()
        except Exception: pass


if __name__ == '__main__':
    main()
