#!/usr/bin/env python3
"""parking_detector_node.py 기초 검증 -- 아직 실제 경기장 QR+P 표식 사진이 없어서
(현장 실측 전) 대신 cv2.QRCodeEncoder로 진짜 QR 구조를 가진 코드를 직접 생성해서
먹인다(가짜 색상 블롭이 아니라 실제로 디코딩 가능한 진짜 QR이라 traffic_light
합성테스트 때 겪은 문제와는 다름). 확인 항목:
1) QR이 멀리 있을 때(화면 작게) found=True지만 box_area_frac이 작음.
2) 가까이 있을 때(화면 크게) box_area_frac이 커짐 -- "접근도" 신호가 방향대로 움직이는지.
3) QR이 전혀 없는 빈 이미지는 found=False.
4) center_x_frac이 좌/우 위치에 맞게 나오는지.
"""
import sys
sys.path.insert(0, '/tmp')
import numpy as np
import cv2
import json
import rclpy
from sensor_msgs.msg import Image
from parking_detector_node import ParkingDetector

rclpy.init()
node = ParkingDetector()
captured = []
node.pub.publish = lambda m: captured.append(json.loads(m.data))

encoder = cv2.QRCodeEncoder.create()
qr_img = encoder.encode('PARKING_P_MARK')  # 실제 디코딩 가능한 QR 비트맵 (흑백, 작은 해상도)
qr_img_bgr = cv2.cvtColor(qr_img, cv2.COLOR_GRAY2BGR)


def make_frame(qr_px_size, cx_frac, width=848, height=480):
    frame = np.full((height, width, 3), 200, dtype=np.uint8)  # 밝은 배경(바닥 느낌)
    qr_resized = cv2.resize(qr_img_bgr, (qr_px_size, qr_px_size), interpolation=cv2.INTER_NEAREST)
    cx = int(cx_frac * width)
    x0 = max(0, min(width - qr_px_size, cx - qr_px_size // 2))
    y0 = (height - qr_px_size) // 2
    frame[y0:y0 + qr_px_size, x0:x0 + qr_px_size] = qr_resized
    msg = Image()
    msg.height, msg.width = height, width
    msg.data = frame.tobytes()
    return msg


def send(qr_px_size, cx_frac):
    captured.clear()
    node.cb_image(make_frame(qr_px_size, cx_frac))
    return captured[-1]


# --- 1) 멀리(작게) ---
far = send(qr_px_size=80, cx_frac=0.5)
assert far['found'] is True, f"작은 QR인데 못 찾음: {far}"
print(f"1) 멀리 있는 QR(80px): found={far['found']}, box_area_frac={far['box_area_frac']} OK")

# --- 2) 가까이(크게) -- box_area_frac이 멀리보다 커야 함 ---
near = send(qr_px_size=300, cx_frac=0.5)
assert near['found'] is True, f"큰 QR인데 못 찾음: {near}"
assert near['box_area_frac'] > far['box_area_frac'], \
    f"가까운데(300px) 접근도가 먼 것(80px)보다 안 큼: near={near['box_area_frac']} far={far['box_area_frac']}"
print(f"2) 가까이 있는 QR(300px): box_area_frac={near['box_area_frac']} > 멀리({far['box_area_frac']}) OK")

# --- 3) QR 없는 빈 이미지 ---
empty_msg = Image()
empty_msg.height, empty_msg.width = 480, 848
empty_msg.data = np.full((480, 848, 3), 200, dtype=np.uint8).tobytes()
captured.clear()
node.cb_image(empty_msg)
assert captured[-1]['found'] is False, f"QR 없는데 found=True(오탐!): {captured[-1]}"
print(f"3) QR 없는 빈 이미지: found={captured[-1]['found']} OK")

# --- 4) 좌/우 위치 반영 ---
left = send(qr_px_size=150, cx_frac=0.15)
right = send(qr_px_size=150, cx_frac=0.85)
assert left['center_x_frac'] < 0.5 < right['center_x_frac'], \
    f"좌/우 위치가 반영 안 됨: left={left['center_x_frac']} right={right['center_x_frac']}"
print(f"4) 좌({left['center_x_frac']})/우({right['center_x_frac']}) 위치 반영 OK")

print("\n전체 PASS (참고: 실제 경기장 QR+P 표식 사진으로는 아직 검증 못 함 -- 현장 실측 필요)")
node.destroy_node()
rclpy.shutdown()
