#!/usr/bin/env python3
"""위치추정 결과를 기록하고 요약한다 — 장시간 주행에서 틀어지는지 보는 것이 목적이다.

  python3 record_localization.py --sec 120 --out out/data4.csv
  python3 record_localization.py --summary out/data4.csv

【무엇을 왜 재는가】
목표는 '오래 달려도 안 틀어지는 것'이다. 한 순간의 정확도보다 **시간에 따른 추세**가 중요하다.
  · 잔차(rms)        스캔이 지도에서 얼마나 떨어져 있나. 커지면 위치가 밀리는 중이다
  · 보정량(correction) map->odom 이 한 번에 얼마나 움직였나. 계속 크면 오도메트리가 밀린다
  · map->odom 누적    보정이 한 방향으로 계속 쌓이면 드리프트다. 진동하면 정상
  · 정합 실패/거부    몇 번이나 못 맞췄나
구간별(30초)로 나눠 보여 주므로 앞뒤를 비교해 틀어짐을 바로 볼 수 있다.
"""
import argparse
import csv
import math
import os
import sys
import time


def summarize(path):
    if not os.path.exists(path):
        print('★ 파일이 없다:', path)
        return 1
    rows = list(csv.DictReader(open(path)))
    if not rows:
        print('★ 비어 있다:', path)
        return 1
    import statistics as st
    f = lambda k, r: float(r[k]) if r.get(k) not in ('', None) else float('nan')
    rms = [f('rms', r) for r in rows if r.get('rms')]
    corr = [f('corr', r) for r in rows if r.get('corr')]
    print('표본 %d개  %.0f초' % (len(rows), f('t', rows[-1])))
    if rms:
        print('  잔차      중앙 %.3f m  95%% %.3f  최악 %.3f'
              % (st.median(rms), sorted(rms)[int(len(rms) * 0.95)], max(rms)))
    if corr:
        print('  한 번 보정 중앙 %.3f m  최대 %.3f' % (st.median(corr), max(corr)))
    print()
    print('  구간      잔차   보정합   map->odom 위치        이동   판정')
    prev = None
    for a in range(0, int(f('t', rows[-1])) + 1, 30):
        seg = [r for r in rows if a <= f('t', r) < a + 30]
        if not seg:
            continue
        sr = [f('rms', r) for r in seg if r.get('rms')]
        sc = sum(f('corr', r) for r in seg if r.get('corr'))
        mx, my = f('mx', seg[-1]), f('my', seg[-1])
        d = math.hypot(mx - prev[0], my - prev[1]) if prev else 0.0
        prev = (mx, my)
        mark = '' if d < 0.15 else ('  <- 밀림' if d < 0.5 else '  <- 크게 밀림 ★')
        print('  %3d~%3d초  %.3f  %6.2f m  (%+.2f, %+.2f)  %6.3f m%s'
              % (a, a + 30, st.median(sr) if sr else float('nan'), sc, mx, my, d, mark))
    print()
    print('  읽는 법: map->odom 위치가 한 방향으로 계속 움직이면 오도메트리가 밀리는 중이다.')
    print('           제자리에서 진동하면 정상. 잔차가 구간마다 커지면 위치를 잃어가는 중이다.')
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sec', type=float, default=120)
    ap.add_argument('--out', default='')
    ap.add_argument('--summary', default='', help='이미 기록한 CSV 를 요약만')
    a = ap.parse_args()
    if a.summary:
        return summarize(a.summary)
    if not a.out:
        print('★ --out 이 필요하다')
        return 1

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    n = Node('localization_recorder')
    state = {'rms': None, 'ms': None, 'mx': 0.0, 'my': 0.0, 'myaw': 0.0,
             'px': None, 'py': None, 'prev': None}
    rows = []

    def on_status(m):
        for tok in m.data.split():
            if '=' in tok:
                k, v = tok.split('=', 1)
                if k in ('rms', 'ms'):
                    try:
                        state[k] = float(v)
                    except ValueError:
                        pass

    def on_pose(m):
        state['px'] = m.pose.pose.position.x
        state['py'] = m.pose.pose.position.y

    def on_tf(m):
        for t in m.transforms:
            if t.header.frame_id.lstrip('/') == 'map' and t.child_frame_id.lstrip('/') == 'odom':
                state['mx'] = t.transform.translation.x
                state['my'] = t.transform.translation.y
                q = t.transform.rotation
                state['myaw'] = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                           1 - 2 * (q.y * q.y + q.z * q.z))

    qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_LAST, depth=10)
    n.create_subscription(String, 'localization/status', on_status, qos)
    n.create_subscription(PoseWithCovarianceStamped, 'localization/pose', on_pose, qos)
    n.create_subscription(TFMessage, '/tf', on_tf, qos)

    t0 = time.time()
    nxt = 0.0
    print('  기록 시작 (%.0f초)' % a.sec)
    while time.time() - t0 < a.sec:
        rclpy.spin_once(n, timeout_sec=0.05)
        t = time.time() - t0
        if t < nxt:
            continue
        nxt = t + 1.0
        corr = ''
        if state['prev'] is not None:
            corr = '%.4f' % math.hypot(state['mx'] - state['prev'][0],
                                       state['my'] - state['prev'][1])
        state['prev'] = (state['mx'], state['my'])
        rows.append(dict(t=round(t, 1),
                         rms='' if state['rms'] is None else '%.4f' % state['rms'],
                         ms='' if state['ms'] is None else '%.0f' % state['ms'],
                         mx='%.4f' % state['mx'], my='%.4f' % state['my'],
                         myaw='%.2f' % math.degrees(state['myaw']),
                         px='' if state['px'] is None else '%.3f' % state['px'],
                         py='' if state['py'] is None else '%.3f' % state['py'],
                         corr=corr))
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, 'w', newline='') as fp:
        w = csv.DictWriter(fp, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    got = sum(1 for r in rows if r['rms'])
    print('  저장 %s (%d행, 정합 결과가 들어온 행 %d)' % (a.out, len(rows), got))
    n.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
