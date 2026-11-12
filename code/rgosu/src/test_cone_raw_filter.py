#!/usr/bin/env python3
"""cone_raw_filter_node.py 오프라인 검증 (합성 데이터만 -- 실제 콘 bag 없음).
확인 항목:
1) cone_zone_active=False(기본값)면 wall_cloud를 그대로 통과시키고 원시 높이밴드는
   안 섞는다(기존 동작 그대로 -- 안전한 기본값).
2) cone_zone_active=True면 높이밴드(0.05~0.45m) 안의 원시 포인트가 합쳐져서 나온다.
3) 높이밴드 밖(바닥 노이즈·로봇보다 높은 천장 등) 포인트는 True여도 안 섞인다.
4) 레벨링(roll/pitch 보정)이 적용되는지 -- 로봇이 기울어진 상태에서도 월드기준
   높이밴드가 올바르게 적용되는지.
"""
import sys
sys.path.insert(0, '/tmp')
import numpy as np
import rclpy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header, Bool
from nav_msgs.msg import Odometry
from cone_raw_filter_node import ConeRawFilterNode


def make_cloud(points_xyz, frame='velodyne'):
    fields = [pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
              pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
              pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1)]
    h = Header(); h.frame_id = frame
    return pc2.create_cloud(h, fields, points_xyz)


def read_cloud(msg):
    if msg.width == 0:
        return np.zeros((0, 3), np.float32)
    pts = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
    return np.asarray(pts, dtype=np.float32).reshape(-1, 3)


rclpy.init()
node = ConeRawFilterNode()
published = []
node.pub.publish = lambda m: published.append(m)

odom = Odometry()
odom.pose.pose.position.x = 0.0
odom.pose.pose.position.y = 0.0
odom.pose.pose.position.z = 0.0
odom.pose.pose.orientation.w = 1.0  # 수평, roll=pitch=yaw=0
node.cb_odom(odom)

wall_points = [(5.0, 0.0, 0.3), (5.0, 0.1, 0.5)]  # wall_filter가 이미 찾은 "진짜 벽" 2점
raw_points = [
    (1.5, 0.0, 0.20),   # 콘 추정 -- 높이밴드(0.05~0.45) 안
    (1.5, 0.0, 0.01),   # 바닥 노이즈 -- 밴드 아래, 제외돼야 함
    (1.5, 0.0, 0.60),   # 밴드 위(너무 높음) -- 제외돼야 함
    (2.0, 0.3, 0.30),   # 콘 추정 2 -- 밴드 안
]

# --- 1) cone_zone_active=False(기본값) -- wall만 통과, 원시 밴드는 안 섞임 ---
node.cb_cloud(make_cloud(raw_points))  # zone 꺼진 상태로 raw cloud 먼저 들어옴
node.cb_wall(make_cloud(wall_points))   # wall_cloud 수신 -> 여기서 publish 트리거
out = read_cloud(published[-1])
assert len(out) == len(wall_points), f"zone 꺼졌는데 wall 외 포인트가 섞임: {out}"
print(f"1) cone_zone_active=False -> wall만 통과({len(out)}점) OK")

# --- 2) cone_zone_active=True -- 높이밴드 안의 원시 포인트가 합쳐짐 ---
node.cb_zone(Bool(data=True))
node.cb_cloud(make_cloud(raw_points))
node.cb_wall(make_cloud(wall_points))
out = read_cloud(published[-1])
# wall 2점 + 밴드 안 원시점 2점(0.20m, 0.30m) = 4점 기대, 밴드 밖 2점(0.01,0.60)은 제외
assert len(out) == 4, f"기대 4점(wall2+band2), 실제 {len(out)}점: {out}"
zs = sorted(round(float(z), 2) for z in out[:, 2])
assert 0.01 not in zs and 0.60 not in zs, f"밴드 밖 포인트가 섞여 들어옴: {zs}"
print(f"2) cone_zone_active=True -> wall(2) + 밴드안 원시점(2) = {len(out)}점, 밴드밖 제외 확인 OK")

# --- 3) 로봇이 기울어진 상태(pitch)에서도 월드기준 높이밴드가 맞게 적용되는지 ---
node2 = ConeRawFilterNode()
published2 = []
node2.pub.publish = lambda m: published2.append(m)
node2.cb_zone(Bool(data=True))

import math
tilted = Odometry()
tilted.pose.pose.position.x = 0.0
tilted.pose.pose.position.y = 0.0
tilted.pose.pose.position.z = 0.0
pitch = math.radians(15)  # 15도 앞으로 기울어진 상태(경사로 위라고 가정)
tilted.pose.pose.orientation.w = math.cos(pitch / 2)
tilted.pose.pose.orientation.y = math.sin(pitch / 2)  # REP103: y축이 pitch
node2.cb_odom(tilted)

# 몸체좌표(기울어진 센서 기준) 점 하나를 주고, wall_filter_node와 동일한
# 수식(level_z = -sin(pitch)*x + cos(pitch)*z_body)으로 "레벨링 후 월드기준
# 높이가 정확히 얼마여야 하는지"를 독립적으로 계산(오라클)해서 그 값과 비교한다
# -- 단순히 "body z값이 그대로 보존될 것"이라 가정하면 틀림(기운 만큼 실제로
# 바뀌는 게 맞는 물리임), 그래서 직접 기대값을 계산해 검증한다.
x_body, z_body = 1.5, 0.661  # 레벨링하면 world z가 약 0.25m(밴드 안)가 되도록 역산
expected_level_z = -math.sin(pitch) * x_body + math.cos(pitch) * z_body
assert 0.05 <= expected_level_z <= 0.45, f"테스트 설계 오류: 기대값이 밴드 밖: {expected_level_z}"

node2.cb_cloud(make_cloud([(x_body, 0.0, z_body)]))
node2.cb_wall(make_cloud([]))
out2 = read_cloud(published2[-1])
assert len(out2) == 1, f"기울어진 상태에서 밴드 안 포인트가 안 살아남음: {out2}"
assert abs(float(out2[0][2]) - expected_level_z) < 0.01, \
    f"레벨링 결과가 기대값과 다름: 실제={out2[0][2]:.3f} 기대={expected_level_z:.3f}"
print(f"3) 로봇 기울어짐(pitch 15도) 레벨링 수식 검증 OK (z={out2[0][2]:.3f}, 기대={expected_level_z:.3f})")

print("\n전체 PASS (참고: 전부 합성 데이터 -- 실제 콘 bag으로는 아직 검증 못 함)")
node.destroy_node()
node2.destroy_node()
rclpy.shutdown()
