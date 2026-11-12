#!/usr/bin/env python3
"""YOLO 기반 traffic_light_detector_node를 실제 bag의 카메라 프레임으로 재생 테스트."""
import sys
sys.path.insert(0, '/home/ilovehan9283/ws/Rgosu/src')
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import rclpy
from traffic_light_detector_node import TrafficLightDetector

bag_path = sys.argv[1]
n_frames = int(sys.argv[2]) if len(sys.argv) > 2 else 200

rclpy.init()
node = TrafficLightDetector()
captured = []
node.pub_state.publish = lambda m: captured.append(m.data)
node.cb_active(Bool(data=True))  # 카메라 강제 ON (정상적으론 track_sequencer가 켜줌)

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
        node.cb_image(msg)
    except Exception as e:
        print(f"[frame {n}] 예외: {e}")
        errors += 1
    n += 1

print(f"\n처리한 프레임: {n}, 예외: {errors}")
print(f"출력 샘플(최근 10개): {captured[-10:]}")
states = [c.split('|')[0] for c in captured]
from collections import Counter
print(f"상태 분포: {Counter(states)}")
print("PASS: 크래시 없음" if errors == 0 else "FAIL")
rclpy.shutdown()
