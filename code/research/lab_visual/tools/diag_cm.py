#!/usr/bin/env python3
"""전역 코스트맵을 한 장 받아 로봇 자리와 존 목표의 칸 값을 본다."""
import json, math, os, sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
import tf2_ros

rclpy.init()
n = Node('cm_peek')
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
got = {}
for topic, key in (('/global_costmap/costmap', 'cm'), ('/map', 'map')):
    n.create_subscription(OccupancyGrid, topic,
                          lambda m, k=key: got.setdefault(k, m), qos)
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
import time
end = time.time() + 12
while time.time() < end and len(got) < 2:
    rclpy.spin_once(n, timeout_sec=0.1)

pose = None
end = time.time() + 6
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.05)
    try:
        tr = buf.lookup_transform('map', 'base_link', rclpy.time.Time())
        pose = (tr.transform.translation.x, tr.transform.translation.y); break
    except Exception:
        pass

def stat(m, name):
    import collections
    d = m.data
    c = collections.Counter(d)
    tot = len(d)
    free = sum(v for k, v in c.items() if 0 <= k <= 10)
    leth = sum(v for k, v in c.items() if k >= 99)
    inf  = sum(v for k, v in c.items() if 11 <= k <= 98)
    unk  = c.get(-1, 0)
    print('%s  %dx%d @ %.3fm  원점(%.2f, %.2f)'
          % (name, m.info.width, m.info.height, m.info.resolution,
             m.info.origin.position.x, m.info.origin.position.y))
    print('   자유 %5.1f%%   팽창 %5.1f%%   점유 %5.1f%%   모름 %5.1f%%'
          % (100*free/tot, 100*inf/tot, 100*leth/tot, 100*unk/tot))

def cell(m, x, y):
    i = int((x - m.info.origin.position.x) / m.info.resolution)
    j = int((y - m.info.origin.position.y) / m.info.resolution)
    if not (0 <= i < m.info.width and 0 <= j < m.info.height):
        return None, (i, j)
    return m.data[j * m.info.width + i], (i, j)

for key, label in (('cm', '전역 코스트맵'), ('map', '/map (지도)')):
    if key not in got:
        print('%s  ★ 못 받음' % label); continue
    stat(got[key], label)

pts = []
if pose: pts.append(('로봇', pose[0], pose[1]))
V = os.path.expanduser('~/gosu/config/venue.json')
if os.path.exists(V):
    zp = (json.load(open(V, encoding='utf-8')).get('zone_poses') or {})
    for z in sorted(zp):
        pts.append(('존 '+z, zp[z]['x'], zp[z]['y']))

print()
print('  지점        x       y    코스트맵   지도')
for name, x, y in pts:
    a, _ = cell(got['cm'], x, y) if 'cm' in got else (None, None)
    b, _ = cell(got['map'], x, y) if 'map' in got else (None, None)
    f = lambda v: ('지도밖' if v is None else
                   ('모름' if v == -1 else ('자유' if v <= 10 else
                    ('점유(%d)' % v if v >= 99 else '팽창(%d)' % v))))
    print('  %-8s %6.2f  %6.2f   %-10s %s' % (name, x, y, f(a), f(b)))
n.destroy_node(); rclpy.try_shutdown()
