#!/usr/bin/env python3
"""센서 비중 동적 전환 검증:
  1) track_sequencer가 구간별로 올바른 camera_active를 발행하는가
     (정지선=True, 나머지=False)
  2) traffic_light_detector가 camera_active=False일 때 실제로 HSV 분석을
     건너뛰는가 (빨간 이미지를 줘도 RED로 오탐하지 않아야 함)
"""
import sys, json
sys.path.insert(0, '/tmp')
import numpy as np
import rclpy
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from track_sequencer_node import TrackSequencer
from traffic_light_detector_node import TrafficLightDetector

rclpy.init()

# --- 1) 구간별 camera_active 발행값 확인 ---
seq = TrackSequencer('/tmp/segments_config.json')
published = []
seq.pub_camera_active.publish = lambda m: published.append(m.data)
seq.pub_status.publish = lambda m: None
seq.pub_req.publish = lambda m: None

def odom(x, y):
    m = Odometry(); m.pose.pose.position.x = x; m.pose.pose.position.y = y
    seq.cb_odom(m)

# start(pass_through)는 진입 즉시 통과하므로, "이 틱에서 어느 구간 기준으로 발행됐나"는
# published[-1]로만 확인한다(틱 직후엔 이미 다음 구간으로 넘어가 있을 수 있음).
odom(0.0, 0.0); seq.tick()
assert published[-1] is False, f"start에서 camera_active 기대 False, 실제 {published[-1]}"
print(f"구간 start: camera_active={published[-1]} (기대 False) OK")

odom(3.0, 0.0); seq.tick()
assert seq._cur()['name'] == 'cone_slalom'
assert published[-1] is False, f"cone_slalom에서 camera_active 기대 False, 실제 {published[-1]}"
print(f"구간 cone_slalom: camera_active={published[-1]} (기대 False) OK")

# stopline_1까지 진입 (cone_slalom 이탈 -> stopline_1). 구간 전환 자체가 같은 틱
# 안에서 일어나므로, 전환 틱에서 발행되는 값은 "떠난" 구간 기준(False)이고,
# stopline_1 자체의 camera_active는 한 틱 더 지나야 반영된다 — 다음 틱으로 확인.
odom(6.0, 0.0); seq.tick()
assert seq._cur()['name'] == 'stopline_1'
seq.tick()  # 같은 위치로 한 번 더 — 이번엔 stopline_1 기준으로 발행됨
assert published[-1] is True, f"stopline_1에서 camera_active=True 기대, 실제 {published[-1]}"
print(f"구간 stopline_1: camera_active={published[-1]} (기대 True) OK")

# --- 2) traffic_light_detector가 실제로 게이팅되는가 ---
import cv2
tl = TrafficLightDetector()
captured = []
tl.pub_state.publish = lambda m: captured.append(m.data)

# ★ 2026-11-04: 전엔 합성(단색 원) 이미지를 썼는데, YOLO 모델을 더 잘 학습된
# tl5cone_640(9 epoch, 실제 사진 기반)으로 바꾸면서 이 테스트가 깨짐 -- 그냥
# 둥글고 빨간 합성 블롭에는 반응을 안 함(실제 신호등 하우징/배경 맥락을 학습해서
# 더 구체적으로 구분하게 된 것 -- 오탐 방지 관점에서는 바람직한 변화). 그래서
# 게이팅(camera_active) 자체를 검증하려면 실제 val셋의 '빨강만 있는' 사진을
# 써야 함 -- eval_tl5cone.py로 확인한, GT가 tl_red 단독이고 모델도 자신있게
# (conf=0.64) RED로 맞추는 실제 사진.
_RED_IMG_PATH = '/home/ilovehan9283/ws/datasets/tl5/ds_tl5_cone/images/val/tl_frame_0169.jpg'


def red_image_msg():
    img_bgr = cv2.imread(_RED_IMG_PATH)
    img_rgb = img_bgr[:, :, ::-1]  # 노드는 RGB8을 가정하므로 채널 순서 뒤집기
    msg = Image()
    msg.height, msg.width = img_rgb.shape[0], img_rgb.shape[1]
    msg.data = np.ascontiguousarray(img_rgb).tobytes()
    msg.header.stamp.sec = 1
    return msg

assert tl.camera_active is False, "기본값은 False여야 함(자원 절약 쪽 안전)"
tl.cb_image(red_image_msg())
assert len(captured) == 0, f"camera_active=False인데 처리됐음: {captured}"
print("camera_active=False -> 빨간 이미지 줘도 처리 안 함(발행 0건): OK")

tl.cb_active(Bool(data=True))  # ON 전환 자체는 발행 안 함(아직 이미지 처리 전) -- OFF 전환만 UNKNOWN을 강제 발행
tl.cb_image(red_image_msg())
assert captured[-1].startswith('RED'), f"ON 상태에서 빨간 이미지 -> RED 기대, 실제 {captured[-1]}"
print(f"camera_active=True -> 정상 인식(RED): OK ({captured[-1]})")

tl.cb_active(Bool(data=False))
assert captured[-1] == 'UNKNOWN|0.00', "OFF 전환 직후 UNKNOWN 발행 기대(하위 상태기가 낡은 GREEN 안 믿게)"
print("camera_active OFF 전환 -> UNKNOWN 즉시 발행(낡은 값 방지): OK")

print("\n전체 PASS: 센서 비중 동적 전환 정상 동작")
rclpy.shutdown()
