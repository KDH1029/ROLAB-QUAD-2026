#!/usr/bin/env python3
"""실제 leg_odom.csv 궤적(obstacle_04 구간, t=182.5~198.29s)으로 gait_switch
지오펜스가 채터링 없이 동작하는지 확인."""
import sys, time
sys.path.insert(0, '/tmp')
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from track_sequencer_node import TrackSequencer, in_zone, ZONE_HYSTERESIS_M

rows = np.loadtxt('/home/ilovehan9283/ws/junsu/maps/segments/leg_odom.csv', delimiter=',', skiprows=1)
# t,x,y,z,qx,qy,qz,qw
mask = (rows[:,0] >= 175.0) & (rows[:,0] <= 205.0)  # obstacle_04 구간 앞뒤로 여유
seg_rows = rows[mask]
print(f"구간 내 샘플 수: {len(seg_rows)}")

# 이 구간의 실제 x,y 범위로 지오펜스 박스를 만든다 (가운데 70%만 "계단구역"으로 지정해서
# 진입/이탈이 실제로 일어나게)
xs, ys = seg_rows[:,1], seg_rows[:,2]
x_lo, x_hi = np.percentile(xs, 15), np.percentile(xs, 85)
y_lo, y_hi = np.percentile(ys, 15), np.percentile(ys, 85)
print(f"지오펜스 박스: x[{x_lo:.2f},{x_hi:.2f}] y[{y_lo:.2f},{y_hi:.2f}]")

import json
cfg = {"segments": [
    {"name": "stairs_test", "type": "gait_switch", "gait": "STATICWALK",
     "zone": {"x_min": float(x_lo), "x_max": float(x_hi), "y_min": float(y_lo), "y_max": float(y_hi)}},
    {"name": "exit", "type": "pass_through", "zone": {"x_min": -1000, "x_max": 1000, "y_min": -1000, "y_max": 1000}},
]}
with open('/tmp/real_test_config.json', 'w') as f:
    json.dump(cfg, f)

rclpy.init()
node = TrackSequencer('/tmp/real_test_config.json')
sent = []
node.pub_req.publish = lambda m: sent.append((m.header.identity.api_id))
node.pub_status.publish = lambda m: None

for row in seg_rows:
    t, x, y = row[0], row[1], row[2]
    m = Odometry(); m.pose.pose.position.x = x; m.pose.pose.position.y = y
    node.cb_odom(m)
    node.tick()

print(f"\n전체 명령 전송 횟수: {len(sent)}")
print(f"명령 시퀀스: {sent}")
n_staticwalk = sent.count(1061)
n_trotrun = sent.count(1062)
print(f"STATICWALK 전송: {n_staticwalk}회, TROTRUN 전송: {n_trotrun}회")
if n_staticwalk <= 1 and n_trotrun <= 1:
    print("PASS: 채터링 없음 (각 1회 이하)")
else:
    print(f"FAIL: 채터링 의심 ({n_staticwalk + n_trotrun}회 전송됨, 2회 넘으면 비정상)")

rclpy.shutdown()
