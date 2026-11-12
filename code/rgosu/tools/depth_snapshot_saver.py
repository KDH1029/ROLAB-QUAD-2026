#!/usr/bin/env python3
"""뎁스카메라를 0.5초 간격으로 한 장씩 PNG(16비트, mm 단위 그대로)로 저장.
bag 녹화(전체 스트림, 보통 15fps)와는 별개로 동시에 돌아감 -- bag엔 모든 프레임이
다 들어가고, 이건 그와 별도로 "빠르게 눈으로 훑어보기 좋은" 저빈도 스냅샷 폴더를
추가로 만드는 용도(ROS/rviz 없이 그냥 이미지 뷰어로 바로 열어볼 수 있음).
"""
import sys
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
import numpy as np
import cv2

SAVE_INTERVAL_S = 0.5


class DepthSnapshotSaver(Node):
    def __init__(self, out_dir, topic):
        super().__init__('depth_snapshot_saver')
        self.out_dir = out_dir
        os.makedirs(out_dir, exist_ok=True)
        self.last_save_t = 0.0
        self.n_saved = 0
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, topic, self.cb, qos)
        self.get_logger().info(
            f'뎁스 스냅샷 저장 시작 -> {out_dir} ({SAVE_INTERVAL_S}초 간격, 구독: {topic})')

    def cb(self, msg: Image):
        now = time.monotonic()
        if now - self.last_save_t < SAVE_INTERVAL_S:
            return
        self.last_save_t = now
        # realsense depth_rect_raw는 16UC1(밀리미터 단위) -- 그대로 16비트 PNG로 저장
        # (8비트로 변환하면 거리 정보가 손실됨, 16비트 PNG는 그대로 보존)
        depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        fname = os.path.join(
            self.out_dir, f'depth_{time.strftime("%Y%m%d_%H%M%S")}_{self.n_saved:05d}.png')
        cv2.imwrite(fname, depth)
        self.n_saved += 1
        if self.n_saved % 20 == 0:
            self.get_logger().info(f'{self.n_saved}장 저장됨 (최근: {fname})')


def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
        f'~/ws/Rgosu/maps/depth_snapshots/{time.strftime("%Y%m%d_%H%M%S")}')
    topic = sys.argv[2] if len(sys.argv) > 2 else '/camera/depth/image_rect_raw'

    rclpy.init()
    node = DepthSnapshotSaver(out_dir, topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'종료 -- 총 {node.n_saved}장 저장됨 ({out_dir})')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
