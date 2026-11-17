#!/usr/bin/env python3
"""Start alignment, one shot: after the robot stands on the start mark, match its first scans to the map and
write the waypoints in TODAY's FAST-LIO frame. Run it after FAST-LIO + the bridges are up, before the runner:

    python3 scripts/go2_start_align.py ~/ws/Rgosu/maps/real_waypoints_venue_run.json /tmp/waypoints_today.json
    python3 ~/ws/Rgosu/src/real_course_runner.py /tmp/waypoints_today.json

Waits until the robot stands still (/sportmodestate body_height > --min-height, speed < 0.05 m/s for 1 s),
collects --win s of /scan with the /Odometry pose of each scan, runs start_align_core.align against --map
(the cone-free 2D map made from the same recording as the waypoints), and writes the converted file.
If the alignment is rejected (best fit outside +-max_xy / +-max_yaw: robot not on the mark, or too few points
on walls) the output is the unchanged waypoints (= what the stack did before) and the exit code is 2.
A side file <out>.align.json keeps T, the match and the reason. Logic: start_align_core.py (tested on bags).
"""
import argparse
import bisect
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import start_align_core as sa  # noqa: E402

import rclpy  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from unitree_go.msg import SportModeState  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def convert(data, T):
    """Waypoints (list of {label,x,y} or {'route': [...]}) from the map frame to today's FAST-LIO frame."""
    entries = data['route'] if isinstance(data, dict) else data
    for e in entries:
        if e.get('x') is not None and e.get('y') is not None:
            e['x'], e['y'] = (round(v, 4) for v in sa.inverse_point(T, e['x'], e['y']))
    return data


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('waypoints_in')
    ap.add_argument('waypoints_out')
    ap.add_argument('--map', default=os.path.join(HERE, '..', 'maps', 'arena1459_nocone.yaml'))
    ap.add_argument('--win', type=float, default=3.0, help='seconds of scans to match')
    ap.add_argument('--max-xy', type=float, default=0.30)
    ap.add_argument('--max-yaw', type=float, default=10.0, help='deg')
    ap.add_argument('--min-height', type=float, default=0.25, help='body_height that counts as standing (m)')
    ap.add_argument('--timeout', type=float, default=20.0, help='give up (unchanged waypoints) after this long')
    ap.add_argument('--prior', default='0,0,0', help='x,y,yaw(deg): 오늘 원점이 지도에서 대략 어디인가(슬라럼 디버깅: 로봇을 놓은 점)')
    a = ap.parse_args()

    data = json.load(open(a.waypoints_in))
    wm = sa.WallMap(a.map)
    rclpy.init()
    n = rclpy.create_node('go2_start_align')
    odom, scans, st = [], [], {'h': 0.0, 'v': 9.0, 'still_since': None}

    def on_state(m):
        st['h'] = m.body_height
        st['v'] = math.hypot(m.velocity[0], m.velocity[1])
        now = time.monotonic()
        if st['h'] > a.min_height and st['v'] < 0.05:
            st['still_since'] = st['still_since'] or now
        else:
            st['still_since'] = None

    n.create_subscription(SportModeState, '/sportmodestate', on_state, qos_profile_sensor_data)
    n.create_subscription(Odometry, '/Odometry', lambda m: odom.append(
        (stamp(m.header), m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))), 50)
    collecting = {'on': False, 't0': None}
    n.create_subscription(LaserScan, '/scan', lambda m: scans.append((stamp(m.header), m)) if collecting['on'] else None,
                          qos_profile_sensor_data)

    t_start = time.monotonic()
    result = None
    print(f'start align: map {os.path.abspath(a.map)}, waiting for the robot to stand still ...', flush=True)
    while rclpy.ok() and time.monotonic() - t_start < a.timeout:
        rclpy.spin_once(n, timeout_sec=0.05)
        ss = st['still_since']
        if not collecting['on'] and ss and time.monotonic() - ss > 1.0 and odom:
            collecting.update(on=True, t0=time.monotonic())
            print(f'  standing (body_height {st["h"]:.2f} m), collecting {a.win:.0f} s of scans', flush=True)
        if collecting['on'] and st['still_since'] is None:
            print('  robot moved while collecting - starting over', flush=True)
            collecting['on'] = False
            scans.clear()
        if collecting['on'] and time.monotonic() - collecting['t0'] >= a.win:
            ts = [o[0] for o in odom]
            pairs = []
            for t, m in scans:
                j = bisect.bisect_left(ts, t)
                cand = [k for k in (j - 1, j) if 0 <= k < len(ts)]
                if not cand:
                    continue
                k = min(cand, key=lambda k: abs(ts[k] - t))
                if abs(ts[k] - t) > 0.1:
                    continue
                pairs.append((sa.scan_points(m.ranges, m.angle_min, m.angle_increment), odom[k][1:]))
            px, py, pa = (float(v) for v in a.prior.split(','))
            result = sa.align(wm, pairs, max_xy=a.max_xy, max_yaw_deg=a.max_yaw, prior=(px, py, math.radians(pa)))
            break
    n.destroy_node()
    rclpy.shutdown()

    if result is None:
        result = dict(ok=False, T=(0.0, 0.0, 0.0), match=0.0,
                      reason=f'timeout {a.timeout:.0f} s (standing {st["h"]:.2f} m, speed {st["v"]:.2f}, '
                             f'odom {len(odom)}, scans {len(scans)}) - is FAST-LIO / cloud_bridge /scan running?')
    T = result['T']
    out = convert(data, T) if result['ok'] else data
    json.dump(out, open(a.waypoints_out, 'w'), indent=1, ensure_ascii=False)
    side = {k: (list(v) if isinstance(v, tuple) else v) for k, v in result.items()}
    side.update(waypoints_in=os.path.abspath(a.waypoints_in), map=os.path.abspath(a.map), time=time.time())
    json.dump(side, open(a.waypoints_out + '.align.json', 'w'), indent=1, ensure_ascii=False)
    if result['ok']:
        print(f'OK  today origin on the map: {T[0]*100:+.1f} cm {T[1]*100:+.1f} cm {math.degrees(T[2]):+.2f} deg '
              f'(scan on walls {result["match"]*100:.0f} %) -> {a.waypoints_out}', flush=True)
        return 0
    print(f'REJECTED ({result["reason"]}) -> waypoints written UNCHANGED to {a.waypoints_out}', flush=True)
    return 2


if __name__ == '__main__':
    sys.exit(main())
