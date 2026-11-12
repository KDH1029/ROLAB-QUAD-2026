#!/usr/bin/env python3
"""/tf 와 /tf_static 을 실제로 받아 '누가 어느 변을 발행하는지' 센다.

같은 자식 프레임에 부모가 둘이면 TF 트리가 깨진다 — 조회 결과가 시간에 따라 오락가락한다.
"""
import collections, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from tf2_msgs.msg import TFMessage

rclpy.init()
n = Node('tf_tree_peek')
cnt = collections.Counter()
static = set()
def on(msg, is_static=False):
    for t in msg.transforms:
        e = (t.header.frame_id.lstrip('/'), t.child_frame_id.lstrip('/'))
        cnt[e] += 1
        if is_static: static.add(e)
n.create_subscription(TFMessage, '/tf', lambda m: on(m), 50)
n.create_subscription(TFMessage, '/tf_static',
    lambda m: on(m, True),
    QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
               durability=DurabilityPolicy.TRANSIENT_LOCAL,
               history=HistoryPolicy.KEEP_LAST))
end = time.time() + 10
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.1)

parents = collections.defaultdict(list)
for (p, c), k in cnt.items():
    parents[c].append((p, k, (p, c) in static))
print('  자식 프레임        부모        10초간 발행수   종류')
for c in sorted(parents):
    for p, k, st in sorted(parents[c], key=lambda z: -z[1]):
        mark = '  <<< 부모가 둘 이상' if len(parents[c]) > 1 else ''
        print('  %-16s %-12s %8d      %s%s'
              % (c, p, k, 'static' if st else '%.0f Hz' % (k/10.0), mark))
bad = [c for c in parents if len(parents[c]) > 1]
print()
if bad:
    print('  ★ 부모가 둘인 프레임: %s' % ', '.join(bad))
    print('    TF 트리가 깨진 상태다. 조회할 때마다 다른 답이 나온다.')
else:
    print('  부모가 둘인 프레임 없음 — 트리는 정상')
n.destroy_node(); rclpy.try_shutdown()
