#!/usr/bin/env python3
"""가만히 서 있는데 자세가 튀는 것이 어느 단계인지 가린다.

    map -> odom        AMCL 이 낸다
    odom -> base_link  오도메트리가 낸다 (다리 기구학 또는 FAST-LIO)
    map -> base_link   둘의 합. 주행이 실제로 믿는 값.

어느 쪽이 움직이는지 보면 범인이 하나로 좁혀진다.
"""
import math, sys, time
import rclpy
from rclpy.node import Node
import tf2_ros

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
rclpy.init()
n = Node('tf_watch')
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
end = time.time() + 3
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.05)

PAIRS = [('map', 'odom'), ('odom', 'base_link'), ('map', 'base_link'),
         ('camera_init', 'body')]
hist = {p: [] for p in PAIRS}
t0 = time.time()
print('  초   %-18s %-18s %-18s %s' % ('map->odom', 'odom->base_link',
                                       'map->base_link', 'camera_init->body'))
nxt = 0.0
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.05)
    el = time.time() - t0
    if el < nxt:
        continue
    nxt = el + 2.0
    row = []
    for p in PAIRS:
        try:
            tr = buf.lookup_transform(p[0], p[1], rclpy.time.Time())
            x = tr.transform.translation.x; y = tr.transform.translation.y
            q = tr.transform.rotation
            yaw = math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
            hist[p].append((el, x, y, yaw))
            row.append('%6.2f,%6.2f,%5.0f' % (x, y, yaw))
        except Exception:
            row.append('        -         ')
    print('%5.0f  %s' % (el, ' '.join(row)))

print()
print('  같은 자리에 서 있었다면 아래가 전부 0 에 가까워야 한다:')
for p in PAIRS:
    h = hist[p]
    if len(h) < 2:
        print('  %-22s 표본 부족' % ('%s->%s' % p)); continue
    dmax = max(math.hypot(b[1]-a[1], b[2]-a[2]) for a in h for b in h)
    ymax = max(abs(b[3]-a[3]) for a in h for b in h)
    flag = '  <<< 이게 튄다' if dmax > 0.3 else ''
    print('  %-22s 위치 최대변화 %6.2f m   방향 최대변화 %6.1f도%s'
          % ('%s->%s' % p, dmax, ymax, flag))
n.destroy_node(); rclpy.try_shutdown()
