#!/usr/bin/env python3
"""경기장 도착 당일 체크리스트 2단계용 — 카메라+YOLO+QR이 실제로 멀쩡히 보고
있는지 한 장 찍어서 바로 확인하는 진단 도구. 라이브 화면(cv2.imshow)은 nuc16이
보통 헤드리스라 안 쓰고, 그냥 바운딩박스를 그린 JPEG 한 장을 저장한다 —
scp로 노트북에 가져가거나 VNC/원격화면 되면 그걸로 바로 열어보면 됨.

실행: ~/ws/Rgosu/venv_yolo/bin/python3 ~/ws/Rgosu/tools/vision_snapshot_check.py
  [출력파일경로, 기본 ~/ws/Rgosu/maps/vision_snapshot_<시각>.jpg]

go2_full_stack.sh가 떠있어야 함(카메라+신호등 노드). camera_active가 꺼져있어도
이 스크립트는 /mission/camera_active를 True로 잠깐 켜서 강제로 한 프레임
추론시켜본다(끝나면 다시 꺼줌 -- 실제 운영 상태를 건드리지 않음).
"""
import sys
import os
import time
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
import numpy as np
import cv2


class SnapshotChecker(Node):
    def __init__(self):
        super().__init__('vision_snapshot_checker')
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)
        self.img = None
        self.tl_state = None
        self.qr_info = None
        self.pub_camera_active = self.create_publisher(Bool, '/mission/camera_active', 10)
        self.create_subscription(Image, '/camera/color/image_raw', self.cb_image, qos)
        self.create_subscription(String, '/mission/traffic_light_state', self.cb_tl, 10)
        self.create_subscription(String, '/mission/parking_qr', self.cb_qr, 10)

    def cb_image(self, msg: Image):
        self.img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3).copy()

    def cb_tl(self, msg: String):
        self.tl_state = msg.data

    def cb_qr(self, msg: String):
        try:
            self.qr_info = json.loads(msg.data)
        except Exception:
            pass


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
        f'~/ws/Rgosu/maps/vision_snapshot_{time.strftime("%Y%m%d_%H%M%S")}.jpg')

    rclpy.init()
    node = SnapshotChecker()

    print('카메라 프레임 수신 대기 중 (최대 5초)...')
    node.pub_camera_active.publish(Bool(data=True))  # YOLO 추론 강제로 켬
    deadline = time.monotonic() + 5.0
    while node.img is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    if node.img is None:
        print('✗ 5초 안에 /camera/color/image_raw가 안 들어옴 -- 카메라 드라이버'
              '(go2_full_stack.sh [7/8]) 떠있는지, ros2 topic hz /camera/color/image_raw로'
              ' 확인할 것.')
        node.pub_camera_active.publish(Bool(data=False))
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # YOLO/QR 결과가 이 프레임에 대해 나올 시간을 좀 더 준다(추론 지연 고려, 16ms면
    # 충분하지만 네트워크/큐 지연 감안해서 넉넉히)
    deadline = time.monotonic() + 1.5
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.pub_camera_active.publish(Bool(data=False))  # 원래 상태로 복귀

    img_bgr = node.img[:, :, ::-1].copy()  # RGB -> BGR (cv2로 그리기/저장용)

    tl_text = node.tl_state or 'UNKNOWN|0.00'
    state, _, conf = (tl_text.partition('|'))
    color = {'RED': (0, 0, 255), 'GREEN': (0, 255, 0)}.get(state, (128, 128, 128))
    cv2.putText(img_bgr, f'traffic_light: {tl_text}', (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

    qr_text = 'QR: (아직 메시지 없음)'
    if node.qr_info is not None:
        qr_text = f"QR: found={node.qr_info.get('found')} area={node.qr_info.get('box_area_frac')}"
    cv2.putText(img_bgr, qr_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 200), 2)
    cv2.putText(img_bgr, time.strftime('%Y-%m-%d %H:%M:%S'), (10, img_bgr.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    cv2.imwrite(out_path, img_bgr)
    print(f'✓ 저장 완료: {out_path}')
    print(f'  신호등 상태: {tl_text}')
    print(f'  QR 상태: {qr_text}')
    print('  (scp로 노트북에 가져가서 열어보거나, VNC/원격화면 되면 그걸로 바로 확인할 것)')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
