#!/usr/bin/env python3
"""주차 미션 인식 노드 — 2026 로보월드 규정 5.2: 지정 주차구역(QR코드 + 문자 'P'
표식)에 정차 후 5초 이상 정지해야 득점(방향 무관). /camera/color/image_raw를 구독해
cv2.QRCodeDetector로 QR을 찾고, 화면 내 QR 유무/크기(=접근도)를 발행한다. 정차 5초
판정 자체(타이머)는 로봇 속도가 0에 가까운 구간을 보는 별도 상태기(mission_state_node
같은 곳)에서 이 토픽을 구독해 하게 된다 -- 이 노드는 "지금 QR이 보이는가/얼마나
가까운가"만 책임진다.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import cv2
import json


class ParkingDetector(Node):
    def __init__(self):
        super().__init__('parking_detector')

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(Image, '/camera/color/image_raw', self.cb_image, qos)
        self.pub = self.create_publisher(String, '/mission/parking_qr', 10)
        self.detector = cv2.QRCodeDetector()

        self.get_logger().info('parking_detector 시작 — /camera/color/image_raw 구독')

    def cb_image(self, msg: Image):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

        found = False
        payload = ''
        box_area_frac = 0.0
        center_x_frac = 0.5
        try:
            ok, decoded, points, _ = self.detector.detectAndDecodeMulti(gray)
        except cv2.error:
            ok, decoded, points = False, [], None

        if ok and points is not None and len(points) > 0:
            # 가장 큰 QR 하나만 본다 (경기장엔 주차구역 QR 1개만 있다고 가정)
            areas = [cv2.contourArea(p.astype(np.float32)) for p in points]
            i = int(np.argmax(areas))
            found = True
            payload = decoded[i] if i < len(decoded) else ''
            box_area_frac = areas[i] / (msg.width * msg.height)
            cx = float(points[i][:, 0].mean())
            center_x_frac = cx / msg.width

        out = String()
        out.data = json.dumps({
            'found': found,
            'payload': payload,
            'box_area_frac': round(box_area_frac, 4),   # 커질수록 로봇이 가까움
            'center_x_frac': round(center_x_frac, 3),   # 0=왼쪽 끝, 1=오른쪽 끝, 0.5=정중앙
        })
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ParkingDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
