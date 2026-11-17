#!/usr/bin/env python3
"""관절각에서 네 발의 위치를 푼다 — 규정이 몸통이 아니라 발을 기준으로 하기 때문이다.

  python3 foot_pose.py --bag ~/ws/lab/bags/live_0910_1512 --at 30

【왜 필요한가】
대회 규정은 "횡단보도 위에 발 최소 1개", "정지구역에 발 3개" 를 요구한다.
몸통 중심만 알면 이걸 판정할 수 없다. 발 간격이 앞뒤 0.39 m, 좌우 0.28 m 라
몸통이 구역 안에 있어도 발은 밖일 수 있다.

【무엇으로 푸는가】
/lowstate 의 관절각 12개(다리마다 고관절·대퇴·종아리)와 Go2 링크 치수.
접지 여부는 foot_force 로 안다 — 공중에 뜬 발은 규정 판정에서 빼야 한다.

【정확도의 한계】
링크 치수는 공개 사양값이라 실제와 몇 mm 차이가 날 수 있다. 발 간격(수십 cm)에 비하면 작다.
더 큰 오차는 위치추정 쪽(0.04~0.10 m)이다.
"""
import argparse
import math
import os
import sys

import numpy as np

# Go2 링크 치수 (공개 사양, m)
L_HIP, L_THIGH, L_CALF = 0.0955, 0.213, 0.213
BX, BY = 0.1934, 0.0465            # 몸통 중심 -> 고관절
LEGS = ('FR', 'FL', 'RR', 'RL')    # unitree 순서
SGN = ((+1, -1), (+1, +1), (-1, -1), (-1, +1))   # (앞뒤, 좌우) 부호


def feet_in_body(q):
    """관절각 12개 -> 네 발의 몸통 기준 (x, y, z). q 는 FR,FL,RR,RL 순서로 3개씩."""
    out = []
    for i, (sx, sy) in enumerate(SGN):
        q0, q1, q2 = q[i * 3], q[i * 3 + 1], q[i * 3 + 2]
        # 다리 평면 안에서 (전후, 상하)
        lx = -L_THIGH * math.sin(q1) - L_CALF * math.sin(q1 + q2)
        lz = -(L_THIGH * math.cos(q1) + L_CALF * math.cos(q1 + q2))
        # 고관절 롤(q0)이 다리 평면을 좌우로 기울인다
        y = sy * L_HIP * math.cos(q0) - lz * math.sin(q0) * sy
        z = sy * L_HIP * math.sin(q0) * sy + lz * math.cos(q0)
        out.append((BX * sx + lx, BY * sy + y, z))
    return out


def feet_in_map(q, x, y, yaw):
    """발을 지도 좌표로. yaw 는 rad."""
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * fx - s * fy, y + s * fx + c * fy, fz)
            for fx, fy, fz in feet_in_body(q)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', default=os.path.expanduser('~/ws/lab/bags/live_0910_1512'))
    ap.add_argument('--at', type=float, default=30.0, help='bag 몇 초 지점')
    a = ap.parse_args()

    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
    from rclpy.serialization import deserialize_message
    from unitree_go.msg import LowState, SportModeState

    r = SequentialReader()
    r.open(StorageOptions(uri=a.bag, storage_id='sqlite3'), ConverterOptions('', ''))
    r.set_filter(StorageFilter(topics=['/lowstate', '/sportmodestate']))
    t0 = None
    low = sp = None
    while r.has_next():
        tp, data, ts = r.read_next()
        if t0 is None:
            t0 = ts
        if (ts - t0) / 1e9 < a.at:
            continue
        if tp == '/lowstate' and low is None:
            low = deserialize_message(data, LowState)
        if tp == '/sportmodestate' and sp is None:
            sp = deserialize_message(data, SportModeState)
        if low and sp:
            break
    if low is None:
        print('★ /lowstate 를 못 읽었다')
        return 1

    q = [m.q for m in low.motor_state[:12]]
    ff = list(low.foot_force)
    feet = feet_in_body(q)
    print('bag %.1f초   몸통 높이 %.3f m   접지력 %s' % (a.at, sp.body_height if sp else float('nan'), ff))
    print('  발 (몸통 기준)        x       y       z    접지')
    for leg, (fx, fy, fz), f in zip(LEGS, feet, ff):
        print('  %-4s              %+.3f  %+.3f  %+.3f   %s' % (leg, fx, fy, fz, 'O' if f > 20 else '-'))
    xs = [f[0] for f in feet]
    ys = [f[1] for f in feet]
    print('  발 사각형: 앞뒤 %.2f m, 좌우 %.2f m' % (max(xs) - min(xs), max(ys) - min(ys)))
    print('  접지한 발 %d개 (힘 20 초과 기준)' % sum(1 for f in ff if f > 20))
    return 0


if __name__ == '__main__':
    sys.exit(main())
