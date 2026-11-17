#!/usr/bin/env python3
"""전환 장치 — FAST-LIO 가 이상하면 플래너 입력(위치 + 장애물 점구름)을 백업으로 바꾼다.

평소      /Odometry        → /Odometry_mux      /cloud_registered → /cloud_active
전환 후   /backup/odometry → /Odometry_mux      /backup/cloud     → /cloud_active
전환 조건 (backup_core.MuxLogic, 녹화본 시험 통과: 정상 주행 오탐 0)
  · FAST-LIO 0.5초 무소식   · FAST-LIO 순간속도 2 m/s 초과   · 백업(건강 ok)과 0.5 m 넘게 1초 지속
  · 시작 5초는 판단 안 함.  한 번 전환하면 유지(경기 중 오락가락 금지) — 복귀는 /mux/reset 서비스로만
점구름은 원시 바이트로 받아 그대로 넘긴다(해석 비용 없음).
내는 것   /mux/status (std_msgs/String JSON, 유지형) — mode, reason
"""
import json, os, sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from backup_core import MuxLogic


class OdomMux(Node):
    def __init__(self, no_switch=False):
        super().__init__('odom_mux')
        self.no_switch = no_switch      # 백업을 안 띄울 때: 중계만 하고 전환은 안 한다(원본과 같은 구성)
        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mux = MuxLogic(); self.bk_xy = None; self.bk_health = 'init'; self.mode = 'fastlio'
        self.pub_odom = self.create_publisher(Odometry, '/Odometry_mux', rel)
        self.pub_cloud = self.create_publisher(PointCloud2, '/cloud_active', rel)
        self.pub_stat = self.create_publisher(String, '/mux/status', latched)
        self.create_subscription(Odometry, '/Odometry', self.on_fl, rel)
        self.create_subscription(Odometry, '/backup/odometry', self.on_bk, rel)
        self.create_subscription(String, '/backup/status', self.on_bk_stat, rel)
        self.create_subscription(PointCloud2, '/cloud_registered', lambda b: self.relay_cloud(b, 'fastlio'), rel, raw=True)
        self.create_subscription(PointCloud2, '/backup/cloud', lambda b: self.relay_cloud(b, 'backup'), rel, raw=True)
        self.create_service(Trigger, '/mux/reset', self.on_reset)
        self.create_timer(0.01, self.tick)
        self.publish_status()

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_fl(self, m):
        p = m.pose.pose.position
        if not self.no_switch:
            self.mux.on_fastlio(self.now(), p.x, p.y, p.z, ts=m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            self.check()
        if self.mode == 'fastlio':
            self.pub_odom.publish(m)

    def on_bk(self, m):
        self.bk_xy = (m.pose.pose.position.x, m.pose.pose.position.y)
        if self.mode == 'backup':
            m.header.stamp = self.get_clock().now().to_msg(); self.pub_odom.publish(m)

    def on_bk_stat(self, m):
        try: self.bk_health = json.loads(m.data).get('health', 'init')
        except Exception: pass

    def relay_cloud(self, raw, src):
        if src == self.mode:
            self.pub_cloud.publish(raw)

    def tick(self):
        if self.no_switch: return
        self.mux.tick(self.now(), self.bk_xy, self.bk_health); self.check()

    def check(self):
        if self.mux.mode != self.mode:
            self.mode = self.mux.mode
            self.get_logger().warn(f'★ 전환 → {self.mode}: {self.mux.reason} (백업 상태 {self.bk_health})')
            self.publish_status()

    def publish_status(self):
        self.pub_stat.publish(String(data=json.dumps(dict(mode=self.mode, reason=self.mux.reason), ensure_ascii=False)))

    def on_reset(self, req, res):
        self.mux.reset(); self.mode = 'fastlio'; self.publish_status()
        self.get_logger().warn('수동 복귀 → fastlio'); res.success = True; res.message = 'fastlio'; return res


def main():
    no_switch = '--no-switch' in sys.argv
    rclpy.init(args=[a for a in sys.argv if a != '--no-switch']); n = OdomMux(no_switch)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        try: rclpy.shutdown()
        except Exception: pass


if __name__ == '__main__':
    main()
