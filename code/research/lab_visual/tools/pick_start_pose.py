#!/usr/bin/env python3
"""시작 자세 후보를 짧게 추적해 보고 옳은 것을 고른다 — 한 장의 스캔으로는 못 가리기 때문이다.

  python3 pick_start_pose.py --bag ~/ws/lab/bags/data4_odom \\
      --map ../maps/go2_prior_map_v05.pcd --at 14 --sec 25

【왜 필요한가】
경기장이 직사각형이라 스캔 한 장은 여러 자리에서 비슷하게 맞는다(실측: 1-2등 격차 0~2%p).
그러나 **움직이면 갈린다.** 자세가 맞으면 오도메트리를 붙여 나아가도 계속 맞지만,
틀리면 몇 초 안에 벽을 뚫거나 정합이 계속 끌려다닌다.
그래서 후보마다 수십 초를 따라가 보고 '보정이 가장 적은' 것을 고른다.

【입력】
오도메트리가 들어 있는 녹화본이 필요하다(/Odometry + /velodyne_points).
원본 녹화본에는 /lidar_points 와 /lowstate 뿐이라, verify_localization.sh 를 RECORD=1 로
한 번 돌려 파생 녹화본을 만들어 두면 이 비교를 ROS 없이 빠르게 반복할 수 있다.
"""
import argparse
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from icp3d import Matcher, se3
from read_pcd import read_pcd

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2

def storage_id(bag):
    """녹화본 형식을 파일 확장자로 판별한다. ROS 2 Jazzy 기본은 mcap, 예전 녹화본은 sqlite3."""
    import glob as _g, os as _o
    if _g.glob(_o.path.join(bag, '*.mcap')):
        return 'mcap'
    return 'sqlite3'


LIDAR_X, LIDAR_YAW = 0.25, math.radians(90.0)     # base_link -> hesai_lidar


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def load(bag, t_from, t_to, hz, pts, rng):
    """녹화본에서 (시각, base_link 기준 스캔, 오도메트리) 를 뽑는다."""
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id=storage_id(bag)), ConverterOptions('', ''))
    r.set_filter(StorageFilter(topics=['/velodyne_points', '/Odometry']))
    t0 = None
    last_odom = None
    nxt = t_from
    out = []
    cl, sl = math.cos(LIDAR_YAW), math.sin(LIDAR_YAW)
    while r.has_next():
        tp, data, ts = r.read_next()
        if t0 is None:
            t0 = ts
        t = (ts - t0) / 1e9
        if t > t_to:
            break
        if tp == '/Odometry':
            o = deserialize_message(data, Odometry)
            p = o.pose.pose.position
            last_odom = (p.x, p.y, yaw_of(o.pose.pose.orientation))
            continue
        if t < nxt or last_odom is None:
            continue
        nxt = t + 1.0 / hz
        c = deserialize_message(data, PointCloud2)
        P = np.array(pc2.read_points_list(c, field_names=('x', 'y', 'z'), skip_nans=True),
                     dtype=np.float64)
        rr = np.hypot(P[:, 0], P[:, 1])
        P = P[(rr > 0.7) & (rr < 12.0)]
        if len(P) < 200:
            continue
        if len(P) > pts:
            P = P[rng.choice(len(P), pts, replace=False)]
        if c.header.frame_id.lstrip('/') != 'base_link':
            P = np.c_[cl * P[:, 0] - sl * P[:, 1] + LIDAR_X, sl * P[:, 0] + cl * P[:, 1], P[:, 2]]
        out.append((t, P, last_odom))
    return out


def track(m, frames, x0, y0, yaw0):
    """후보 자세에서 출발해 오도메트리 증분을 붙여 따라간다. 돌려주는 것: (잔차중앙, 보정합, 최대보정)"""
    tfix = se3(math.radians(yaw0), [x0, y0, 0.0])
    o_at = frames[0][2]
    rms_list = []
    corr_sum = 0.0
    corr_max = 0.0
    for _, P, od in frames:
        ox, oy, oyaw = od
        px, py, pyaw = o_at
        c, s = math.cos(pyaw), math.sin(pyaw)
        lx = c * (ox - px) + s * (oy - py)
        ly = -s * (ox - px) + c * (oy - py)
        byaw = math.atan2(tfix[1, 0], tfix[0, 0])
        cb, sb = math.cos(byaw), math.sin(byaw)
        guess = se3(byaw + (oyaw - pyaw),
                    [tfix[0, 3] + cb * lx - sb * ly, tfix[1, 3] + sb * lx + cb * ly, 0.0])
        T, rms, _ = m.align(P, guess)
        if not math.isfinite(rms):
            return float('inf'), float('inf'), float('inf'), float('inf'), float('inf')
        d = math.hypot(T[0, 3] - guess[0, 3], T[1, 3] - guess[1, 3])
        corr_sum += d
        corr_max = max(corr_max, d)
        rms_list.append(rms)
        tfix = T
        o_at = od
    # 앞/뒤 절반 잔차를 따로 본다. 자세가 틀리면 움직일수록 안 맞아 뒤쪽이 커진다.
    h = len(rms_list) // 2
    return (float(np.median(rms_list)), corr_sum, corr_max,
            float(np.median(rms_list[:h])), float(np.median(rms_list[h:])))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', required=True, help='/Odometry 가 들어 있는 녹화본')
    ap.add_argument('--map', default=os.path.expanduser('~/ws/lab_visual/maps/go2_prior_map_v05.pcd'))
    ap.add_argument('--at', type=float, default=14.0, help='후보 자세의 기준 시각')
    ap.add_argument('--sec', type=float, default=25.0, help='후보마다 따라가 볼 시간')
    ap.add_argument('--hz', type=float, default=2.0)
    ap.add_argument('--pts', type=int, default=800)
    ap.add_argument('--cand', required=True,
                    help='"x,y,yaw;x,y,yaw;..." 전역 탐색 상위 후보들')
    a = ap.parse_args()

    mp = read_pcd(os.path.expanduser(a.map))
    m = Matcher(mp, stages=((0.20, 0.8, 12), (0.10, 0.35, 8)))
    rng = np.random.default_rng(0)
    frames = load(os.path.expanduser(a.bag), a.at, a.at + a.sec, a.hz, a.pts, rng)
    if len(frames) < 5:
        print('★ 프레임이 부족하다 (%d개) — 녹화본 구간을 확인할 것' % len(frames))
        return 1
    print('지도 %s  후보를 %.0f초(%d프레임) 따라가 본다'
          % (os.path.basename(a.map), a.sec, len(frames)))
    print('  후보 (x, y, yaw)          잔차중앙   보정합   최대보정   앞절반   뒤절반  추세')
    best = None
    for c in a.cand.split(';'):
        x, y, yaw = [float(v) for v in c.split(',')]
        t0 = time.time()
        rms, csum, cmax, r1, r2 = track(m, frames, x, y, yaw)
        trend = '나빠짐 ★' if r2 > r1 * 1.25 else ('좋아짐' if r2 < r1 * 0.8 else '유지')
        print('  (%+.2f, %+.2f, %+4.0f°)   %8.3f %8.2f %9.3f %8.3f %8.3f  %s'
              % (x, y, yaw, rms, csum, cmax, r1, r2, trend))
        # 점수: 뒤쪽 잔차가 낮고 보정이 적어야 좋다. 뒤쪽 잔차를 더 크게 본다.
        score = r2 * 2 + csum / max(len(frames), 1)
        if best is None or score < best[0]:
            best = (score, x, y, yaw, r2, csum)
    print()
    print('고른 자세: x=%.3f y=%.3f yaw=%.1f  (뒤절반 잔차 %.3f m, 보정합 %.2f m)'
          % (best[1], best[2], best[3], best[4], best[5]))
    print('  뒤절반 잔차가 뚜렷이 낮아야 믿을 수 있다. 비슷하면 --sec 을 늘려 더 오래 따라갈 것')
    return 0


if __name__ == '__main__':
    sys.exit(main())
