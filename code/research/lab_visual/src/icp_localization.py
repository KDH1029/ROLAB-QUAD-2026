#!/usr/bin/env python3
"""3D scan matching localization — 점군 정합으로 map -> odom 변환을 발행한다.

AMCL 을 대신하는 노드다. AMCL 은 점군을 수평으로 잘라 2D 로 만든 뒤 맞추는데,
이 노드는 3D 점군을 그대로 3D 지도에 맞춘다. 자르지 않으므로 몸통이 기울어도
관측이 망가지지 않고, 천장·상부 구조까지 근거로 쓴다.

역할 분담 (표준 ROS 규약)
  map  -> odom        이 노드. 지도 대비 누적 오차를 보정한다
  odom -> base_link   오도메트리(FAST-LIO 또는 다리 기구학). 저지연, 여기서 건드리지 않는다

내는 것
  TF  map -> odom
  localization/pose    geometry_msgs/PoseWithCovarianceStamped — AMCL 의 amcl_pose 와 같은 형식.
                       공분산은 정합 잔차에서 추정한다(잔차가 크면 덜 믿는다)
  localization/status  std_msgs/String — 잔차·반복수·소요시간

받는 것
  /velodyne_points     sensor_msgs/PointCloud2 (--cloud 로 바꿀 수 있다)
  /initialpose         geometry_msgs/PoseWithCovarianceStamped — 시작 자세

AMCL 과 동시에 띄우면 안 된다. 둘 다 map->odom 을 내면 서로 덮어쓴다.

초기 자세
  정합은 초기 추정이 0.5 m / 5° 안이어야 수렴한다(대회 지도 실측: 그 범위 100%, 0.8 m/10° 는 25%).
  그래서 처음에는 /initialpose 를 기다리고, 그다음부터는 직전 결과에 오도메트리 증분을 붙여 쓴다.
  전역 재탐색은 이 노드가 하지 않는다 — 그건 초기자세 탐색기의 몫이다.

실행
  python3 icp_localization.py --map go2_prior_map_clean.pcd
  python3 icp_localization.py --map go2_prior_map_clean.pcd --hz 5 --scan-pts 800 --wide
"""

import argparse
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from std_msgs.msg import String
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from icp3d import Matcher            # noqa: E402
from read_pcd import read_pcd       # noqa: E402  필드 순서를 헤더에서 찾는 판독기


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_z(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class IcpLocalization(Node):

    def __init__(self, a):
        super().__init__('icp_localization')
        self.hz = a.hz
        self.map_frame = a.map_frame
        self.odom_frame = a.odom_frame
        self.base_frame = a.base_frame
        self.max_pts = a.scan_pts
        self.rng = np.random.default_rng(0)

        mp_path = a.map if os.path.isabs(a.map) else os.path.join(
            os.path.expanduser('~'), 'go2_ws', 'maps', a.map)
        t0 = time.time()
        mp = read_pcd(mp_path)
        # 2026-09-12: 한 해상도 정합은 0.3 m 오차도 못 잡았다(실측 성공률 0%).
        #   추적 중에는 오도메트리 증분 덕에 오차가 몇 cm 라 가벼운 2단으로 충분하다(노트북 62 ms).
        #   --wide 는 4단 전체(0.5 m/5° 까지 100%). 초기 수렴이나 재정합 직후에 쓴다.
        TRACK = ((0.20, 0.8, 12), (0.10, 0.35, 8))
        self.m = Matcher(mp, trim=a.trim, stages=(None if a.wide else TRACK))
        # 잔차가 나쁠 때만 쓰는 넓은 단계표 (느리지만 멀리서도 돌아온다)
        self.m_wide = self.m if a.wide else Matcher(mp, trim=a.trim)
        self.get_logger().info(
            '지도 %s  점 %d -> 복셀 %d개  (%.1f초)'
            % (os.path.basename(mp_path), len(mp), len(self.m.map),
               time.time() - t0))

        # map -> odom (우리가 내는 것)
        self.mx = 0.0
        self.my = 0.0
        self.myaw = 0.0
        self.have_init = False

        self.buf = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10))
        self.lis = tf2_ros.TransformListener(self.buf, self)
        self.bc = tf2_ros.TransformBroadcaster(self)

        # 센서 데이터는 BEST_EFFORT 가 규약이다 (rclpy 의 SensorDataQoS 와 같은 설정).
        #   · RELIABLE 은 재전송과 하트비트 왕복이 붙는다. 어차피 depth=1 로 밀리면 버릴
        #     것이면서 그 비용을 냈다.
        #   · 더 중요한 건 호환성이다. 발행자가 BEST_EFFORT 인데 구독자가 RELIABLE 이면
        #     ROS 2 에서는 **아예 연결되지 않는다.** 브리지 설정이 바뀌면 조용히 끊긴다.
        #     BEST_EFFORT 구독자는 RELIABLE 발행자에도 붙으므로 이쪽이 항상 안전하다.
        self.create_subscription(
            PointCloud2, a.cloud, self.on_cloud,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=1))
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.on_init,
            # 초기자세는 센서가 아니라 **명령**이다. 한 번 놓치면 영영 안 온다.
            # 그래서 여기는 RELIABLE 이 맞다. VOLATILE 인 이유는 rviz 의
            # "2D Pose Estimate" 가 VOLATILE 로 내기 때문 — TRANSIENT_LOCAL 로 두면 안 붙는다.
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST, depth=1,
                       durability=DurabilityPolicy.VOLATILE))
        self.pub_stat = self.create_publisher(String, 'localization/status', 10)
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, 'localization/pose', 10)

        # 위치 신뢰도 — 잔차가 판정 기준이다.
        #   녹화본 5개 실측(2026-09-12): 자리를 잡으면 0.043~0.046 m, 못 잡으면 0.088 m.
        #   그래서 0.06 m 를 경계로 본다. 연속으로 넘으면 넓은 단계표로 다시 맞춘다.
        self.rms_bad = a.rms_bad
        self.bad_streak = 0
        self.bad_need = a.bad_need
        self.healthy = False
        self.n_rewide = 0

        self.last = 0.0
        self.n_in = 0
        self.n_run = 0
        self.rms_acc = []
        self.ms_acc = []
        self.t_rep = time.time()
        self.create_timer(1.0 / max(1.0, self.hz), self.tick_tf)
        self.create_timer(5.0, self.report)
        self.get_logger().info(
            '입력 %s  %.0fHz  스캔 %d점  복셀 %.2f  — /initialpose 대기'
            % (a.cloud, self.hz, self.max_pts, a.voxel))

    def on_init(self, msg):
        p = msg.pose.pose
        self.mx, self.my = p.position.x, p.position.y
        self.myaw = yaw_of(p.orientation)
        # /initialpose 는 map->base_link 를 준다. 우리가 낼 것은 map->odom 이므로
        # 현재 odom->base_link 를 빼야 한다.
        try:
            tr = self.buf.lookup_transform(self.odom_frame, self.base_frame, rclpy.time.Time())
            ox, oy = tr.transform.translation.x, tr.transform.translation.y
            oyaw = yaw_of(tr.transform.rotation)
            c, s = math.cos(self.myaw - oyaw), math.sin(self.myaw - oyaw)
            self.myaw = self.myaw - oyaw
            self.mx = self.mx - (c * ox - s * oy)
            self.my = self.my - (s * ox + c * oy)
        except Exception:
            pass
        self.have_init = True
        self.get_logger().info('초기 자세 받음: map->odom x=%.2f y=%.2f yaw=%.1f도'
                               % (self.mx, self.my, math.degrees(self.myaw)))

    def tick_tf(self):
        """map->odom 을 꾸준히 낸다. 정합이 늦어도 TF 는 끊기면 안 된다."""
        if not self.have_init:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = self.mx
        t.transform.translation.y = self.my
        qx, qy, qz, qw = quat_z(self.myaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.bc.sendTransform(t)

    def on_cloud(self, msg):
        self.n_in += 1
        if not self.have_init:
            return
        now = time.time()
        if now - self.last < 0.95 / self.hz:
            return
        self.last = now

        # odom -> base_link 를 받아 현재 map->base_link 초기값을 만든다
        try:
            tr = self.buf.lookup_transform(self.odom_frame, self.base_frame, rclpy.time.Time())
        except Exception:
            return
        ox, oy = tr.transform.translation.x, tr.transform.translation.y
        oyaw = yaw_of(tr.transform.rotation)
        c, s = math.cos(self.myaw), math.sin(self.myaw)
        bx = self.mx + c * ox - s * oy
        by = self.my + s * ox + c * oy
        byaw = self.myaw + oyaw

        # 점군은 라이다 프레임(hesai_lidar)으로 온다. base_link 기준으로 옮겨야 한다.
        #   라이다는 base_link 에서 앞 0.25 m, 요 90° 돌아가 있다(imu_bridge 가 내는 정적 TF).
        #   이걸 안 하면 정합은 map->lidar 를 풀면서 결과를 map->base_link 로 해석해,
        #   로봇이 돌 때마다 map->odom 이 흘러간다 (2026-09-12 실측: 130초에 6~10 m).
        src_frame = msg.header.frame_id.lstrip('/')
        if src_frame and src_frame != self.base_frame:
            try:
                tb = self.buf.lookup_transform(self.base_frame, src_frame, rclpy.time.Time())
            except Exception:
                if self.n_in % 50 == 1:
                    self.get_logger().warn('TF %s -> %s 가 없다 — 스캔을 못 옮긴다'
                                           % (self.base_frame, src_frame))
                return
            q = tb.transform.rotation
            tvec = np.array([tb.transform.translation.x, tb.transform.translation.y,
                             tb.transform.translation.z])
            # 사원수 -> 회전행렬
            xq, yq, zq, wq = q.x, q.y, q.z, q.w
            Rb = np.array([
                [1 - 2 * (yq * yq + zq * zq), 2 * (xq * yq - zq * wq), 2 * (xq * zq + yq * wq)],
                [2 * (xq * yq + zq * wq), 1 - 2 * (xq * xq + zq * zq), 2 * (yq * zq - xq * wq)],
                [2 * (xq * zq - yq * wq), 2 * (yq * zq + xq * wq), 1 - 2 * (xq * xq + yq * yq)]])
        else:
            Rb, tvec = np.eye(3), np.zeros(3)

        raw = np.asarray(pc2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True))
        if raw.size < 500:
            return
        P = np.stack([raw['x'], raw['y'], raw['z']], -1).astype(np.float64)
        P = P.reshape(-1, 3)
        P = (Rb @ P.T).T + tvec           # 라이다 -> base_link
        r = np.hypot(P[:, 0], P[:, 1])
        P = P[(r > 0.7) & (r < 12.0)]
        if len(P) < 200:
            return
        if len(P) > self.max_pts:
            P = P[self.rng.choice(len(P), self.max_pts, replace=False)]

        T0 = np.eye(4)
        cc, ss = math.cos(byaw), math.sin(byaw)
        T0[:3, :3] = [[cc, -ss, 0], [ss, cc, 0], [0, 0, 1]]
        T0[:3, 3] = [bx, by, 0.0]

        t0 = time.perf_counter()
        T, rms, it = self.m.align(P, T0)
        # 잔차가 계속 나쁘면 좁은 단계표로는 못 돌아온다. 넓은 단계표로 한 번 다시 맞춘다.
        if math.isfinite(rms) and rms > self.rms_bad and self.bad_streak + 1 >= self.bad_need:
            T, rms, it2 = self.m_wide.align(P, T0)
            it += it2
            self.n_rewide += 1
        ms = (time.perf_counter() - t0) * 1000
        if not math.isfinite(rms):
            return

        good = rms <= self.rms_bad
        self.bad_streak = 0 if good else self.bad_streak + 1
        if good != self.healthy:
            self.healthy = good
            self.get_logger().info('위치추정 %s (잔차 %.3f m, 기준 %.3f)'
                                   % ('정상' if good else '의심 — 재정합 중', rms, self.rms_bad))

        nyaw = math.atan2(T[1, 0], T[0, 0])
        nx, ny = T[0, 3], T[1, 3]
        # 한 번에 크게 튀면 버린다 — 발산 방어
        if math.hypot(nx - bx, ny - by) > 1.0 or \
           abs(((nyaw - byaw + math.pi) % (2 * math.pi)) - math.pi) > 0.5:
            self.get_logger().warn('정합 결과가 너무 멀다 — 버림 (rms %.3f)' % rms)
            return

        # map->base_link 결과에서 odom->base_link 를 빼 map->odom 을 얻는다
        dy = nyaw - oyaw
        c2, s2 = math.cos(dy), math.sin(dy)
        self.myaw = dy
        self.mx = nx - (c2 * ox - s2 * oy)
        self.my = ny - (s2 * ox + c2 * oy)

        self.n_run += 1
        self.rms_acc.append(rms)
        self.ms_acc.append(ms)
        self.pub_stat.publish(String(
            data='rms=%.3f iters=%d ms=%.0f healthy=%d rewide=%d'
                 % (rms, it, ms, int(self.healthy), self.n_rewide)))

        # AMCL 과 같은 형식으로 자세를 낸다 — 다른 노드가 골라 쓸 수 있게
        pm = PoseWithCovarianceStamped()
        pm.header.stamp = self.get_clock().now().to_msg()
        pm.header.frame_id = self.map_frame
        pm.pose.pose.position.x = float(nx)
        pm.pose.pose.position.y = float(ny)
        qz, qw = math.sin(nyaw * 0.5), math.cos(nyaw * 0.5)
        pm.pose.pose.orientation.z = qz
        pm.pose.pose.orientation.w = qw
        # 공분산은 정합 잔차에서 추정한다 (잔차가 크면 덜 믿는다)
        var = max(rms, 0.02) ** 2
        pm.pose.covariance[0] = var
        pm.pose.covariance[7] = var
        pm.pose.covariance[35] = (max(rms, 0.02) / 2.0) ** 2
        self.pub_pose.publish(pm)

    def report(self):
        dt = time.time() - self.t_rep
        if self.n_run:
            self.get_logger().info(
                '입력 %.1fHz  정합 %.1fHz  잔차 %.3f m  %.0f ms  %s  재정합 %d회'
                % (self.n_in / dt, self.n_run / dt,
                   float(np.mean(self.rms_acc)), float(np.mean(self.ms_acc)),
                   '정상' if self.healthy else '의심', self.n_rewide))
        elif not self.have_init:
            self.get_logger().warn('/initialpose 대기 중 — auto_initialpose.py 를 돌릴 것')
        else:
            self.get_logger().warn('입력 %.1fHz 인데 정합이 한 번도 안 돌았다' % (self.n_in / dt))
        self.n_in = self.n_run = 0
        self.rms_acc = []
        self.ms_acc = []
        self.t_rep = time.time()



def parse_args(ap):
    """ROS 인자(--ros-args …)와 우리 인자를 갈라서 읽는다.

    argparse 는 --ros-args 를 모르는 인자로 보고 거절한다. 그래서 rclpy 의 유틸로
    ROS 인자를 먼저 걷어낸 뒤 나머지만 argparse 에 준다.
    ROS 인자는 rclpy.init(args=sys.argv) 로 그대로 넘겨야 use_sim_time 같은 게 먹는다.
    """
    from rclpy.utilities import remove_ros_args
    return ap.parse_args(remove_ros_args(sys.argv)[1:])

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default=os.environ.get('MAP_PCD', 'map_0907.pcd'))
    ap.add_argument('--cloud', default='/velodyne_points')
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--odom-frame', default='odom')
    ap.add_argument('--base-frame', default='base_link')
    ap.add_argument('--hz', type=float, default=5.0)
    ap.add_argument('--scan-pts', type=int, default=800)
    ap.add_argument('--voxel', type=float, default=0.25)
    ap.add_argument('--max-corr', type=float, default=1.0)
    ap.add_argument('--iters', type=int, default=20)
    ap.add_argument('--trim', type=float, default=0.7)
    ap.add_argument('--rms-bad', type=float, default=0.06,
                    help='잔차가 이보다 크면 위치를 의심한다 (m). 실측: 정상 0.045, 실패 0.088')
    ap.add_argument('--bad-need', type=int, default=3,
                    help='연속 이 횟수 의심되면 넓은 단계표로 다시 맞춘다')
    ap.add_argument('--wide', action='store_true',
                    help='수렴 범위 우선(4단, 느림). 기본은 추적 전용 2단')
    a = parse_args(ap)

    rclpy.init(args=sys.argv)
    n = IcpLocalization(a)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if e.__class__.__name__ != 'ExternalShutdownException':
            raise
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
