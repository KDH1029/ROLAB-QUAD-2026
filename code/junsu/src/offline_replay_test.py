#!/usr/bin/env python3
"""bag의 실제 이미지 몇 프레임을 꺼내 traffic_light/parking 노드의 콜백 로직에
직접 먹여서(ROS 그래프 없이) 크래시 여부와 출력값을 확인한다."""
import sys
sys.path.insert(0, '/tmp')
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
import rclpy
from traffic_light_detector_node import TrafficLightDetector
from parking_detector_node import ParkingDetector

bag_path = sys.argv[1]
n_frames = int(sys.argv[2]) if len(sys.argv) > 2 else 20

rclpy.init()
tl_node = TrafficLightDetector()
tl_node.camera_active = True  # 이 테스트는 비전 로직 자체를 검증하는 거라 게이트를 수동으로 켠다
pk_node = ParkingDetector()

captured = {'tl': [], 'pk': []}
tl_node.pub_state.publish = lambda m: captured['tl'].append(m.data)
pk_node.pub.publish = lambda m: captured['pk'].append(m.data)

storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap")
converter_options = rosbag2_py.ConverterOptions("", "")
reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)
reader.set_filter(rosbag2_py.StorageFilter(topics=["/camera/color/image_raw"]))

n = 0
errors = 0
while reader.has_next() and n < n_frames:
    topic, data, t_ns = reader.read_next()
    msg = deserialize_message(data, Image)
    try:
        tl_node.cb_image(msg)
    except Exception as e:
        print(f"[frame {n}] traffic_light_detector 예외: {e}")
        errors += 1
    try:
        pk_node.cb_image(msg)
    except Exception as e:
        print(f"[frame {n}] parking_detector 예외: {e}")
        errors += 1
    n += 1

print(f"\n처리한 프레임: {n}, 예외 발생: {errors}")
print(f"신호등 노드 출력 샘플(최근 5개): {captured['tl'][-5:]}")
print(f"주차 노드 출력 샘플(최근 5개): {captured['pk'][-5:]}")
print("PASS: 실데이터로 크래시 없음" if errors == 0 else "FAIL: 예외 발생")

rclpy.shutdown()
