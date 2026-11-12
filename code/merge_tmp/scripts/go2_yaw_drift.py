#!/usr/bin/env python3
"""Straight-walk yaw drift: measure it, get the cmd_vel_to_sport yaw trim.

    python3 scripts/go2_yaw_drift.py bag ~/bags/<bag> [--trim A,B]
    python3 scripts/go2_yaw_drift.py live --vx 0.3 --secs 6 [--trim A,B]     # MOVES THE ROBOT (--vx -0.3: backwards)

bag:  takes the stretches where the robot walked forward (>= 0.15 m/s) on a straight command -
      autonomous Move with yaw 0 for >= 1 s (or = the trim given with --trim, if the bag was made with
      it on), or the remote's left stick forward with the right stick centred - and reports the yaw
      rate the IMU measured (+ = left) per speed band and the trim to set.
live: run it in your own terminal, remote in hand, 2+ m clear in front. Needs the motor bridge
      (go2_cmd_vel_to_sport) active and nothing else publishing /cmd_vel. Sends /cmd_vel vx for
      --secs, measures /sportmodestate yaw_speed and forward speed, stops (zero + bridge StopMove),
      prints the residual turn against the COMMANDED vx (what the bridge trim multiplies) and whether
      it is within 1 deg/m. (The bag mode still fits against the actual speed - do not use its numbers
      as the trim: 11-04 that over-corrected by 2 deg/m.) Ctrl+C stops at once.
Trim: bridge_params.yaml go2_cmd_vel_to_sport yaw_trim_per_vx (rad/s per m/s) and yaw_trim_bias (rad/s):
      wz_out = wz - (per_vx * vx + bias) while walking. One speed -> per_vx only; two or more speeds
      -> both from a line fit. --trim A,B = the trim that was active (results become corrections to it).
"""
import argparse
import json
import math
import sys
import time

import numpy as np


def suggest(samples, trim):
    """samples: (vx, measured_wz) while told to go straight. Returns (per_vx, bias) to set."""
    a0, b0 = trim
    v = np.array([s[0] for s in samples])
    w = np.array([s[1] for s in samples])
    bands = [(lo, hi) for lo, hi in [(0.15, 0.3), (0.3, 0.5), (0.5, 1.5)] if ((v >= lo) & (v < hi)).sum() >= 20]
    print(f'  {len(w)} samples, forward {v.mean():.2f} m/s: yaw {math.degrees(w.mean()):+.2f} deg/s '
          f'(median {math.degrees(np.median(w)):+.2f}) = {math.degrees(w.mean() / v.mean()):+.1f} deg per metre  [+ = left]')
    for lo, hi in bands:
        m = (v >= lo) & (v < hi)
        print(f'    vx {lo:.2f}-{hi:.2f}: {m.sum():4d} samples, yaw {math.degrees(w[m].mean()):+.2f} deg/s')
    if len(bands) >= 2:
        k, c = np.polyfit(v, w, 1)
    else:
        k, c = w.mean() / v.mean(), 0.0
    # the robot turned by (k vx + c) on top of what the active trim removed: add it to the trim
    print(f'  -> yaw_trim_per_vx: {a0 + k:.3f}   yaw_trim_bias: {b0 + c:.3f}'
          + ('' if len(bands) >= 2 else '   (one speed only: per_vx alone; test a second speed for the bias)'))
    return a0 + k, b0 + c


def from_bag(path, trim):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    want = [t for t in ('/api/sport/request', '/wirelesscontroller', '/sportmodestate') if t in types]
    if '/sportmodestate' not in want:
        sys.exit('bag has no /sportmodestate')
    r.set_filter(rosbag2_py.StorageFilter(topics=want))
    M = {k: get_message(types[k]) for k in want}
    req, wc, sm = [], [], []
    k = 0
    while r.has_next():
        tp, data, t = r.read_next()
        t *= 1e-9
        if tp == '/sportmodestate':
            k += 1
            if k % 10:
                continue
        m = deserialize_message(data, M[tp])
        if tp == '/api/sport/request' and m.header.identity.api_id == 1008:
            p = json.loads(m.parameter or '{}')
            req.append((t, p.get('x', 0.0), p.get('z', 0.0)))
        elif tp == '/wirelesscontroller':
            wc.append((t, m.ly, m.rx))
        elif tp == '/sportmodestate':
            sm.append((t, m.velocity[0], m.yaw_speed))
    req, wc, sm = np.array(req or np.zeros((0, 3))), np.array(wc or np.zeros((0, 3))), np.array(sm)
    a0, b0 = trim
    auto, stick = [], []
    for t, vx, wz in sm:
        if vx < 0.15:
            continue
        if len(req):
            past = req[(req[:, 0] > t - 1.0) & (req[:, 0] <= t)]
            if len(past) >= 5 and past[-1, 1] > 0.1:
                expected = -(a0 * past[:, 1] + b0)          # what the trim alone would send
                if np.all(np.abs(past[:, 2] - expected) < 0.02):
                    auto.append((vx, wz))
                continue
        if len(wc):
            i = np.searchsorted(wc[:, 0], t) - 1
            if i >= 0 and t - wc[i, 0] < 0.5 and wc[i, 1] > 0.1 and abs(wc[i, 2]) < 0.03:
                stick.append((vx, wz))
    out = None
    if auto:
        print('autonomous straight commands (the trim applies to these):')
        out = suggest(auto, trim)
    else:
        print('no autonomous straight commands in this bag')
    if stick:
        print('remote, left stick forward / right stick centred (for comparison - the trim does not touch the remote):')
        suggest(stick, (0.0, 0.0))
    return out


def live(vx, secs, trim):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist, Vector3
    from rosidl_runtime_py.utilities import get_message
    rclpy.init()
    n = rclpy.create_node('go2_yaw_drift_test')
    pub = n.create_publisher(Twist, '/cmd_vel', 10)
    got = []
    n.create_subscription(get_message('unitree_go/msg/SportModeState'), '/sportmodestate',
                          lambda m: got.append((time.time(), m.velocity[0], m.yaw_speed)), qos_profile_sensor_data)
    t_end = time.time() + 2.0
    while pub.get_subscription_count() == 0 and time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        print('nothing subscribes /cmd_vel - is go2_cmd_vel_to_sport active?')
        return None
    for i in (3, 2, 1):
        print(f'walking straight at {vx} m/s for {secs} s in {i} ... (Ctrl+C stops)', flush=True)
        time.sleep(1.0)
    t0 = time.time()
    try:
        while time.time() - t0 < secs:
            pub.publish(Twist(linear=Vector3(x=vx)))
            rclpy.spin_once(n, timeout_sec=0.1)
    finally:
        for _ in range(5):
            pub.publish(Twist())
            time.sleep(0.05)
        print('stopped')
    sgn = 1.0 if vx > 0 else -1.0
    samples = [(v, w) for t, v, w in got if t > t0 + 1.5 and t <= t0 + secs and sgn * v > 0.15]   # skip the start-up
    n.destroy_node()
    rclpy.shutdown()
    if len(samples) < 20:
        print(f'only {len(samples)} samples while walking - did it walk?')
        return None
    return live_report(vx, samples, trim)


def live_report(vx_cmd, samples, trim):
    """Fit against the COMMANDED vx (the bridge multiplies the commanded vx, not the actual speed)."""
    a0, b0 = trim
    v = np.array([s[0] for s in samples])
    w = np.array([s[1] for s in samples])
    wm = w.mean()
    deg_m = math.degrees(wm / abs(v.mean()))
    print(f'  commanded vx {vx_cmd:+.2f} m/s, actual {v.mean():+.2f} m/s, {len(w)} samples')
    print(f'  residual yaw {math.degrees(wm):+.2f} deg/s (median {math.degrees(np.median(w)):+.2f}) = {deg_m:+.1f} deg per metre  '
          f'[+ = left; trim active {a0:.3f} / {b0:.3f}]')
    ok = abs(deg_m) <= 1.0
    print('  -> OK (|residual| <= 1 deg/m), keep the trim' if ok else
          f'  -> this speed alone asks for yaw_trim_per_vx {a0 + wm / vx_cmd:.3f} (bias {b0:.3f} kept); '
          'fit per_vx and bias from two commanded speeds before changing it')
    return wm


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    b = sub.add_parser('bag')
    b.add_argument('path')
    l = sub.add_parser('live')
    l.add_argument('--vx', type=float, default=0.3)
    l.add_argument('--secs', type=float, default=6.0)
    for p in (b, l):
        p.add_argument('--trim', default='0,0', help='per_vx,bias that was active (default 0,0)')
    a = ap.parse_args()
    trim = tuple(float(v) for v in a.trim.split(','))
    if a.cmd == 'bag':
        from_bag(a.path, trim)
    else:
        if not 0.1 <= abs(a.vx) <= 0.6 or not 2 <= a.secs <= 15:
            sys.exit('keep |--vx| 0.1..0.6 (negative = backwards) and --secs 2..15')
        live(a.vx, a.secs, trim)
    return 0


if __name__ == '__main__':
    sys.exit(main())
