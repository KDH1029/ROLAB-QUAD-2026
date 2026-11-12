#!/usr/bin/env python3
"""재생용 러너 흉내 — 몸 높이가 낮았다(엎드림) 다시 서면, 러너처럼 '일어남 확인 + 1초'(여기선 1.2초) 뒤 /mission/realign 을 보낸다.
(러너 PARK_RISE_WAIT → BalanceStand → request_realign(±0.5 m, ±20°) 와 같은 요청)"""
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from unitree_go.msg import SportModeState


class Mimic(Node):
    def __init__(self):
        super().__init__('mimic_runner')
        self.topic = self.declare_parameter('sport_topic', '/sportmodestate').value
        self.pub = self.create_publisher(String, '/mission/realign', 2)
        self.create_subscription(SportModeState, self.topic, self.cb, qos_profile_sensor_data)
        self.low = False; self.up_t = None; self.sent = 0

    def cb(self, m):
        t = self.get_clock().now().nanoseconds * 1e-9; h = m.body_height
        if h < 0.22:
            self.low = True; self.up_t = None
        elif self.low and h > 0.25:
            if self.up_t is None:
                self.up_t = t
            elif t - self.up_t >= 1.2:
                self.low = False; self.up_t = None; self.sent += 1
                self.pub.publish(String(data=json.dumps({'max_xy': 0.5, 'max_yaw': 20.0, 'why': 'park'})))
                self.get_logger().warn(f'재정렬 요청 {self.sent}번째 (다시 선 뒤 1.2초)')


def main():
    rclpy.init(); n = Mimic()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
