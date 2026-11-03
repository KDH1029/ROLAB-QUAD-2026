#!/usr/bin/env python3
"""Save route points on site (AMCL map pose) and drive the route runner.

Walk the robot with the remote to each point, stop, then save it. A point is saved only when
  - the robot stood still for --still s (TF map -> base_link spread < 3 cm, < 2 deg),
  - AMCL is sure enough (/amcl_pose std x, y <= --max-std m, yaw <= --max-yaw-std deg),
  - it lies on a free cell of the map (/map) with no wall within 0.10 m.
Otherwise nothing is written and the reasons are printed. Coordinates come from AMCL (map frame),
never from FAST-LIO (its pose jumped kilometres when it diverged).

    python3 scripts/go2_route.py save stop1 --route maps/route_arena.yaml
    python3 scripts/go2_route.py list --route maps/route_arena.yaml
    python3 scripts/go2_route.py start [step] | stop | resume        # -> /route/cmd
    python3 scripts/go2_route.py status

The route file is a ROS parameter file for route_runner_node: `points` (saved here) and
`sequence` (edited by hand: "<point> [stopline|parking|finish] [tol=m] [vmax=m/s]", in driving
order; a point may appear more than once).
"""
import argparse
import math
import os
import sys
import time

import yaml

TEMPLATE_SEQUENCE = [
    '# example - replace with your points in driving order',
    'start', 'ramp', 'stop1 stopline tol=0.10 vmax=0.30', 'top', 'slalom_in', 'slalom_out',
    'stop2 stopline tol=0.10 vmax=0.30', 'bottom', 'park parking tol=0.10 vmax=0.30', 'out finish',
]


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def load(path):
    if not os.path.exists(path):
        return {'route_runner_node': {'ros__parameters': {'points': {}, 'sequence': []}}}
    d = yaml.safe_load(open(path)) or {}
    p = d.setdefault('route_runner_node', {}).setdefault('ros__parameters', {})
    p.setdefault('points', {})
    p.setdefault('sequence', [])
    return d


def write(path, d):
    p = d['route_runner_node']['ros__parameters']
    head = ('# Route for route_runner_node (go2_run.sh --route <this file>).\n'
            '# points: [x, y, yaw_deg] in the AMCL map frame, saved by scripts/go2_route.py save <name>.\n'
            '# sequence: "<point> [stopline|parking|finish] [tol=m] [vmax=m/s]" in driving order.\n')
    seq = [s for s in p['sequence'] if not str(s).startswith('#')]
    out = {'route_runner_node': {'ros__parameters': dict(p, sequence=seq)}}
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        f.write(head)
        yaml.safe_dump(out, f, sort_keys=False, default_flow_style=None, allow_unicode=True)
    os.replace(tmp, path)


def cmd_save(a):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import OccupancyGrid
    import tf2_ros

    rclpy.init()
    n = Node('go2_route_save')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    got = {}
    n.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: got.__setitem__('amcl', m), latched)
    n.create_subscription(OccupancyGrid, '/map', lambda m: got.__setitem__('map', m), latched)
    samples = []
    t_end = time.time() + a.still + 1.0
    while time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.05)
        try:
            tf = buf.lookup_transform(a.map_frame, 'base_link', rclpy.time.Time())
            samples.append((tf.transform.translation.x, tf.transform.translation.y, yaw_of(tf.transform.rotation)))
        except Exception:
            pass
    deadline = time.time() + 2.0
    while ('amcl' not in got or 'map' not in got) and time.time() < deadline:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.destroy_node()
    rclpy.shutdown()

    problems = []
    if len(samples) < 10:
        print(f'no TF {a.map_frame} -> base_link - is AMCL running?')
        return 1
    xs, ys, yaws = zip(*samples[len(samples) // 3:])      # the last ~still seconds
    mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
    myaw = math.atan2(sum(math.sin(v) for v in yaws), sum(math.cos(v) for v in yaws))
    spread = max(math.hypot(x - mx, y - my) for x, y in zip(xs, ys))
    yspread = max(abs(math.degrees((v - myaw + math.pi) % (2 * math.pi) - math.pi)) for v in yaws)
    if spread > 0.03 or yspread > 2.0:
        problems.append(f'not still: moved {spread * 100:.1f} cm / {yspread:.1f} deg during {a.still:.1f} s')
    if 'amcl' in got:
        c = got['amcl'].pose.covariance
        sx, sy, syaw = math.sqrt(max(c[0], 0)), math.sqrt(max(c[7], 0)), math.degrees(math.sqrt(max(c[35], 0)))
        if sx > a.max_std or sy > a.max_std or syaw > a.max_yaw_std:
            problems.append(f'AMCL unsure: std x {sx:.2f} y {sy:.2f} m, yaw {syaw:.1f} deg '
                            f'(limits {a.max_std} m, {a.max_yaw_std} deg) - drive a little, or go2_amcl_init.py')
        ap = got['amcl'].pose.pose.position
        if math.hypot(ap.x - mx, ap.y - my) > 0.10:
            print(f'note: /amcl_pose ({ap.x:.2f}, {ap.y:.2f}) differs from TF ({mx:.2f}, {my:.2f}) by > 10 cm')
    else:
        problems.append('no /amcl_pose received')
    if 'map' in got:
        g = got['map']
        res, ox, oy, w, h = g.info.resolution, g.info.origin.position.x, g.info.origin.position.y, g.info.width, g.info.height
        ix, iy = int((mx - ox) / res), int((my - oy) / res)
        if not (0 <= ix < w and 0 <= iy < h):
            problems.append(f'outside the map ({mx:.2f}, {my:.2f})')
        else:
            v = g.data[iy * w + ix]
            if v < 0 or v > 25:
                problems.append(f'map cell here is {"unknown" if v < 0 else "occupied"} ({v})')
            r = max(1, int(0.10 / res))
            near = [g.data[j * w + i] for j in range(max(0, iy - r), min(h, iy + r + 1))
                    for i in range(max(0, ix - r), min(w, ix + r + 1))]
            if any(c > 65 for c in near):
                problems.append('a wall within 0.10 m of the point')
    else:
        problems.append('no /map received (map_server running?)')

    if problems:
        print(f'NOT saved "{a.name}":')
        for p in problems:
            print('  - ' + p)
        return 1
    d = load(a.route)
    pts = d['route_runner_node']['ros__parameters']['points']
    old = pts.get(a.name)
    pts[a.name] = [round(mx, 3), round(my, 3), round(math.degrees(myaw), 1)]
    if not d['route_runner_node']['ros__parameters']['sequence']:
        d['route_runner_node']['ros__parameters']['sequence'] = TEMPLATE_SEQUENCE
    write(a.route, d)
    print(f'saved "{a.name}" = ({mx:.3f}, {my:.3f}, {math.degrees(myaw):.1f} deg) -> {a.route}'
          + (f'  (was {old})' if old else ''))
    return 0


def cmd_list(a):
    d = load(a.route)
    p = d['route_runner_node']['ros__parameters']
    print(f'{a.route}: {len(p["points"])} points')
    for k, v in p['points'].items():
        print(f'  {k:14s} x {v[0]:7.3f}  y {v[1]:7.3f}  yaw {v[2]:6.1f}')
    print('sequence:')
    missing = []
    for i, s in enumerate(x for x in p['sequence'] if not str(x).startswith('#')):
        name = str(s).split()[0]
        flag = '' if name in p['points'] else '   <- not saved yet'
        if flag:
            missing.append(name)
        print(f'  {i:2d}  {s}{flag}')
    if missing:
        print(f'missing points: {", ".join(sorted(set(missing)))}')
    return 1 if missing else 0


def cmd_send(a):
    import rclpy
    from std_msgs.msg import String
    rclpy.init()
    n = rclpy.create_node('go2_route_cmd')
    pub = n.create_publisher(String, '/route/cmd', 10)
    text = {'start': f'START {a.step}', 'stop': 'STOP', 'resume': 'RESUME'}[a.cmd]
    t_end = time.time() + 2.0
    while pub.get_subscription_count() == 0 and time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        print('route_runner_node is not listening on /route/cmd (go2_run.sh --route ...?)')
        return 1
    pub.publish(String(data=text))
    rclpy.spin_once(n, timeout_sec=0.2)
    print(f'sent "{text}"')
    return 0


def cmd_status(a):
    import rclpy
    from std_msgs.msg import String
    rclpy.init()
    n = rclpy.create_node('go2_route_status')
    got = []
    n.create_subscription(String, '/route/status', lambda m: got.append(m.data), 10)
    t_end = time.time() + 2.0
    while not got and time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.1)
    print(got[-1] if got else 'no /route/status (route_runner_node not active?)')
    return 0 if got else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('save')
    s.add_argument('name')
    s.add_argument('--route', default='maps/route_arena.yaml')
    s.add_argument('--still', type=float, default=1.5, help='s the robot must stand still')
    s.add_argument('--max-std', type=float, default=0.15, help='m, AMCL std x/y limit')
    s.add_argument('--max-yaw-std', type=float, default=8.0, help='deg, AMCL yaw std limit')
    s.add_argument('--map-frame', default='map')
    l = sub.add_parser('list')
    l.add_argument('--route', default='maps/route_arena.yaml')
    st = sub.add_parser('start')
    st.add_argument('step', nargs='?', type=int, default=0)
    sub.add_parser('stop')
    sub.add_parser('resume')
    sub.add_parser('status')
    a = ap.parse_args()
    if a.cmd == 'save':
        return cmd_save(a)
    if a.cmd == 'list':
        return cmd_list(a)
    if a.cmd == 'status':
        return cmd_status(a)
    return cmd_send(a)


if __name__ == '__main__':
    sys.exit(main())
