#!/usr/bin/env python3
"""RealSense depth 프레임 1장을 받아서 포인트클라우드로 변환하고,
arena_0921에서 썼던 것과 같은 방식(경사+거칠기)으로 위험도 그리드를 만들어본다.
목적: depth 데이터가 위험도 그리드 입력으로 실제로 쓸만한지 검증(PoC)."""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo


class OneShotGrab(Node):
    def __init__(self):
        super().__init__('depth_grab')
        self.depth = None
        self.info = None
        self.sub_d = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self.depth_cb, 1)
        self.sub_i = self.create_subscription(
            CameraInfo, '/camera/camera/depth/camera_info', self.info_cb, 1)

    def depth_cb(self, msg):
        if self.depth is None:
            arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            self.depth = arr.copy()
            self.get_logger().info(f'depth 프레임 받음: {msg.width}x{msg.height}')

    def info_cb(self, msg):
        if self.info is None:
            self.info = {'fx': msg.k[0], 'fy': msg.k[4], 'cx': msg.k[2], 'cy': msg.k[5]}
            self.get_logger().info(f'intrinsics: {self.info}')


def main():
    rclpy.init()
    node = OneShotGrab()
    while rclpy.ok() and (node.depth is None or node.info is None):
        rclpy.spin_once(node, timeout_sec=1.0)
    rclpy.shutdown()

    depth = node.depth.astype(np.float32) / 1000.0  # mm -> m
    fx, fy, cx, cy = node.info['fx'], node.info['fy'], node.info['cx'], node.info['cy']
    h, w = depth.shape

    uu, vv = np.meshgrid(np.arange(w), np.arange(h))
    valid = (depth > 0.1) & (depth < 4.0)  # D435 유효 근거리 범위
    z = depth[valid]
    u = uu[valid].astype(np.float32)
    v = vv[valid].astype(np.float32)
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    # 카메라 광학 프레임: x=오른쪽, y=아래, z=전방. 지금은 바닥 기준 미보정(핸드헬드라 거치 각도 모름)
    print(f"유효 포인트: {len(z)}개 / 전체 {h*w}개 ({100*len(z)/(h*w):.1f}%)")
    print(f"깊이 범위: {z.min():.2f}~{z.max():.2f}m")
    print(f"X(좌우) 범위: {x.min():.2f}~{x.max():.2f}m")
    print(f"Y(상하, 카메라 기준 아래가 +): {y.min():.2f}~{y.max():.2f}m")

    np.savez('/home/gamulchi/arena0921_map/depth_pointcloud_sample.npz', x=x, y=y, z=z)
    print("저장: depth_pointcloud_sample.npz")


if __name__ == '__main__':
    main()
