#!/usr/bin/env python3
"""[DEAD/삭제 대상, 2026-11-04] 더 이상 안 쓰는 테스트 — 2026-11-01에 노드가
HSV 색분할에서 YOLO로 바뀌면서 여기서 테스트하던 TrafficLightDetector._best_blob()
메서드 자체가 지워졌음. 지금 그냥 실행하면 AttributeError로 바로 죽음(ROS 노드엔
영향 없음, 이 테스트 파일만 깨짐). 지워도 되는 파일 — 자동승인 시스템이 rm을 막아서
일단 표시만 해둠. YOLO 버전 테스트는 test_yolo_on_val.py / test_yolo_node_replay.py 참고.
"""
import numpy as np
import cv2
import sys
sys.path.insert(0, '/tmp')
from traffic_light_detector_node import TrafficLightDetector

def make_test_image(color_bgr, radius=30, bg=(20, 20, 20)):
    img = np.full((480, 848, 3), bg, dtype=np.uint8)
    cv2.circle(img, (424, 240), radius, color_bgr, -1)
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # RGB로 반환 (노드가 RGB 가정)

def eval_frame(rgb_img, min_blob_area=80, min_circularity=0.55):
    hsv = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2HSV)
    red_mask = (cv2.inRange(hsv, (0, 120, 100), (10, 255, 255))
                | cv2.inRange(hsv, (170, 120, 100), (180, 255, 255)))
    green_mask = cv2.inRange(hsv, (45, 80, 100), (85, 255, 255))
    red_area, red_circ = TrafficLightDetector._best_blob(red_mask)
    green_area, green_circ = TrafficLightDetector._best_blob(green_mask)
    state = 'UNKNOWN'
    if red_area > min_blob_area and red_circ > min_circularity and red_area >= green_area:
        state = 'RED'
    elif green_area > min_blob_area and green_circ > min_circularity:
        state = 'GREEN'
    return state, red_area, green_area

tests = [
    ('순수 빨간 원',  (0, 0, 255), 'RED'),     # BGR
    ('순수 초록 원',  (0, 255, 0), 'GREEN'),
    ('빈 배경(신호등 없음)', None, 'UNKNOWN'),
    ('주황/노랑 원(빨강 아님)', (0, 165, 255), 'UNKNOWN'),
]

print(f"{'테스트':30s} {'기대':8s} {'실제':8s} {'red_area':10s} {'green_area':10s} 결과")
all_ok = True
for name, color, expected in tests:
    img = make_test_image(color) if color else np.full((480, 848, 3), (20,20,20), dtype=np.uint8)
    state, ra, ga = eval_frame(img)
    ok = state == expected
    all_ok &= ok
    print(f"{name:30s} {expected:8s} {state:8s} {ra:10.0f} {ga:10.0f} {'OK' if ok else 'FAIL'}")

print("\n전체:", "PASS" if all_ok else "FAIL")
