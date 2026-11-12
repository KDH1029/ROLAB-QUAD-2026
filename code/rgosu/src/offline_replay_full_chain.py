#!/usr/bin/env python3
"""bag의 실제 /camera/color/image_raw + /sportmodestate를 같이 재생하면서
비전노드 -> 미션상태기 전체 체인에 먹여 크래시 여부를 최종 확인한다."""
import sys
sys.path.insert(0, '/tmp')
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from unitree_go.msg import SportModeState
import rclpy
from traffic_light_detector_node import TrafficLightDetector
from parking_detector_node import ParkingDetector
from stopline_mission_node import StoplineMission
from parking_mission_node import ParkingMission

bag_path = sys.argv[1]
n_frames = int(sys.argv[2]) if len(sys.argv) > 2 else 300

rclpy.init()
tl_node = TrafficLightDetector()
tl_node.camera_active = True  # track_sequencer가 평소엔 꺼두지만, 이 테스트는 비전 체인 자체를 검증
pk_node = ParkingDetector()
sl_mission = StoplineMission()
pk_mission = ParkingMission()

# 비전노드 publish -> 미션노드 cb로 직결 (ROS 그래프 없이 수동 배선)
tl_node.pub_state.publish = lambda m: sl_mission.cb_light(m)
pk_node.pub.publish = lambda m: pk_mission.cb_qr(m)
sl_mission.pub_state.publish = lambda m: None
pk_mission.pub_state.publish = lambda m: None
pk_mission.pub_req.publish = lambda m: None

storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap")
converter_options = rosbag2_py.ConverterOptions("", "")
reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)
reader.set_filter(rosbag2_py.StorageFilter(topics=["/camera/color/image_raw", "/sportmodestate"]))

n = 0
errors = 0
n_img, n_state = 0, 0
while reader.has_next() and n < n_frames:
    topic, data, t_ns = reader.read_next()
    try:
        if topic == "/camera/color/image_raw":
            msg = deserialize_message(data, Image)
            tl_node.cb_image(msg)
            pk_node.cb_image(msg)
            n_img += 1
        elif topic == "/sportmodestate":
            msg = deserialize_message(data, SportModeState)
            sl_mission.cb_state(msg)
            pk_mission.cb_state(msg)
            n_state += 1
        sl_mission.tick()
        pk_mission.tick()
    except Exception as e:
        print(f"[frame {n}, topic {topic}] 예외: {e}")
        errors += 1
    n += 1

print(f"\n처리: {n}개 메시지 (이미지 {n_img}, 상태 {n_state}), 예외 {errors}건")
print(f"stopline_mission 최종 상태: {sl_mission.state}")
print(f"parking_mission 최종 상태: {pk_mission.state}")
print("PASS: 전체 체인 실데이터로 크래시 없음" if errors == 0 else "FAIL")

rclpy.shutdown()
