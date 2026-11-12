#!/usr/bin/env python3
"""장애물 층을 출발 자세에 맞춰 돌려 굽는다 — 계획기가 좌표 변환을 안 하기 때문이다.

  # 출발 자세를 알 때 (지도 좌표로 x, y, 방향)
  python3 align_obstacle_layer.py --start 1.21,-4.77,8.1 --out ../maps/room_obstacles_aligned.bin

  # 정합 기록에서 출발 자세를 자동으로 읽을 때
  python3 align_obstacle_layer.py --from-csv ../out/chk_data3.csv --out ../maps/room_obstacles_aligned.bin

  # 만들고 나서 확인 (얼마나 맞는지 격자로 센다)
  python3 align_obstacle_layer.py --start 1.21,-4.77,8.1 --check

【문제】
ego_planner 는 `room_obstacles.bin` 을 읽어 그 값을 **그대로 월드 좌표로 쓴다**
(grid_map.cpp loadPriorMap — 변환 코드가 없다). 그런데 계획기의 월드는 `camera_init`,
즉 **FAST-LIO 가 부팅한 자리**다. 파일 안의 좌표는 지도(map) 기준이다.
둘이 다르면 장애물 층 전체가 그 차이만큼 밀리고 돌아간다. 실측 0.92 ~ 10.34 m.
결과: 진짜 장애물 자리는 비어 보이고(26~41%), 빈 곳에 유령벽이 선다(14~40%).

【해법】
계획기 코드를 안 고치고, **파일 쪽을 미리 돌려 놓는다.**
로봇이 지도 좌표 (x0, y0) 에서 방향 yaw0 으로 부팅하면, camera_init 원점이 거기에 놓인다.
따라서 지도 좌표 p_m 의 진짜 camera_init 좌표는

    p_odom = R(-yaw0) · (p_m - [x0, y0])

이 변환을 파일에 미리 적용해서 새 파일로 저장한다. 원본은 건드리지 않는다.

【전제】
· 로봇이 **그 자세에서 부팅**해야 한다. 다른 자리에서 켜면 이 파일도 틀린다.
· FAST-LIO 가 발산하지 않아야 한다. 발산하면 camera_init 기준 자체가 무너진다.
· 계획기는 시작할 때 한 번만 읽으므로, 주행 중에 바꿔 봐야 반영되지 않는다.

【파일 형식】 (grid_map.cpp loadPriorMap 과 일치)
  uint32  점 개수 n
  float32 x, y 를 n 쌍
"""
import argparse
import csv
import math
import os
import struct
import sys

import numpy as np

DEFAULT_SRC = os.path.expanduser('~/go2_ws/maps/room_obstacles.bin')
RES = 0.10                          # ego_planner 의 격자 크기
INFLATE = 0.25                      # ego_planner.yaml 의 safe_dist 와 맞춰야 뜻이 있다


def read_layer(path):
    """장애물 층을 읽는다. .bin (헤더 있는 float32 쌍) 과 .npy 를 모두 받는다."""
    if path.endswith('.npy'):
        a = np.load(path).astype(np.float64)
        if a.ndim != 2 or a.shape[1] != 2:
            raise SystemExit('★ npy 모양이 (N,2) 가 아니다: %s' % (a.shape,))
        return a
    with open(path, 'rb') as f:
        raw = f.read()
    if len(raw) < 4:
        raise SystemExit('★ 파일이 너무 짧다: %s' % path)
    n = struct.unpack('<I', raw[:4])[0]
    need = 4 + 8 * n
    if len(raw) != need:
        raise SystemExit('★ 크기가 안 맞는다: 머리말은 %d점이라는데 파일은 %d바이트 (%d바이트여야 한다)'
                         % (n, len(raw), need))
    return np.frombuffer(raw, dtype='<f4', count=2 * n, offset=4).reshape(-1, 2).astype(np.float64)


def write_layer(path, pts):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'wb') as f:
        f.write(struct.pack('<I', len(pts)))
        f.write(pts.astype('<f4').tobytes())


def to_odom(pts, x0, y0, yaw0_deg):
    """지도 좌표 -> camera_init(odom) 좌표. 로봇이 (x0,y0,yaw0) 에서 부팅했다고 볼 때."""
    th = math.radians(yaw0_deg)
    c, s = math.cos(-th), math.sin(-th)
    q = pts - np.array([x0, y0])
    return np.c_[c * q[:, 0] - s * q[:, 1], s * q[:, 0] + c * q[:, 1]]


def cells(pts, keep=None):
    """ego_planner 와 같은 조건으로 점유 격자를 만든다 (0.1 m 격자 + 0.45 m 팽창).

    keep 은 **점 단위** 선택이다. 좌표계가 다른 두 점군을 비교할 때
    각자의 좌표로 구역을 자르면 서로 다른 물체를 비교하게 되므로,
    자를 거라면 같은 물체 집합을 골라 두고 그 선택을 양쪽에 똑같이 써야 한다.
    """
    if keep is not None:
        pts = pts[keep]
    ij = np.floor(pts / RES).astype(np.int64)
    base = set(map(tuple, ij))
    r = int(math.ceil(INFLATE / RES))
    out = set()
    for dx in range(-r, r + 1):
        for dy in range(-r, r + 1):
            if dx * dx + dy * dy <= r * r:
                out |= {(i + dx, j + dy) for i, j in base}
    return out


def start_from_tf(map_frame, odom_frame, wait):
    """지금 돌고 있는 정합에서 map->odom 을 직접 받아 온다.

    이게 가장 정확한 길이다. 로봇을 아무 데서나 켜도, 정합이 자리를 잡으면
    그 값이 곧 camera_init 이 지도 어디에 놓였는지를 말해 준다.
    """
    import rclpy
    import tf2_ros
    from rclpy.node import Node
    rclpy.init()
    n = Node('read_map_to_odom')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)
    t0 = __import__('time').monotonic()
    tf = None
    while __import__('time').monotonic() - t0 < wait:
        rclpy.spin_once(n, timeout_sec=0.2)
        try:
            tf = buf.lookup_transform(map_frame, odom_frame, rclpy.time.Time())
            break
        except Exception:
            continue
    n.destroy_node()
    rclpy.try_shutdown()
    if tf is None:
        raise SystemExit('★ %s -> %s 변환을 %.0f초 동안 못 받았다. 정합이 돌고 있는가?'
                         % (map_frame, odom_frame, wait))
    q = tf.transform.rotation
    yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                  1 - 2 * (q.y * q.y + q.z * q.z)))
    x = tf.transform.translation.x
    y = tf.transform.translation.y
    print('  TF %s -> %s 에서 읽었다: (%.3f, %.3f, %.1f°)' % (map_frame, odom_frame, x, y, yaw))
    return x, y, yaw


def start_from_csv(path):
    """정합 기록(record_localization.py 형식)에서 수렴한 map->odom 을 읽는다."""
    rows = [r for r in csv.DictReader(open(path)) if r.get('mx') and r.get('rms')]
    if not rows:
        raise SystemExit('★ 기록에 쓸 만한 행이 없다: %s' % path)
    tail = rows[max(0, len(rows) - 20):]          # 끝부분이 자리 잡은 값이다
    x = float(np.median([float(r['mx']) for r in tail]))
    y = float(np.median([float(r['my']) for r in tail]))
    yaws = np.radians([float(r['myaw']) for r in tail])
    yaw = math.degrees(math.atan2(np.mean(np.sin(yaws)), np.mean(np.cos(yaws))))
    rms = float(np.median([float(r['rms']) for r in tail]))
    print('  기록 %s 의 끝 %d행에서 출발 자세를 읽었다: (%.3f, %.3f, %.1f°)  잔차 %.3f m'
          % (os.path.basename(path), len(tail), x, y, yaw, rms))
    if rms > 0.06:
        print('  ★ 잔차 %.3f m 는 문턱(0.06)을 넘는다 — 이 자세를 믿으면 안 된다' % rms)
    return x, y, yaw


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--src', default=DEFAULT_SRC, help='원본 장애물 층 (.bin 또는 .npy)')
    ap.add_argument('--start', default='',
                    help='"x,y,yaw" 로봇이 부팅하는 자세 (지도 좌표, 각도는 도)')
    ap.add_argument('--from-csv', default='', help='정합 기록에서 출발 자세를 읽는다')
    ap.add_argument('--from-tf', action='store_true',
                    help='지금 돌고 있는 정합의 TF map->odom 에서 읽는다 (가장 정확하다)')
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--odom-frame', default='odom')
    ap.add_argument('--tf-wait', type=float, default=20.0, help='TF 를 기다릴 시간 (초)')
    ap.add_argument('--out', default='', help='저장할 경로. 없으면 계산만 한다')
    ap.add_argument('--check', action='store_true',
                    help='변환 전후로 진짜 장애물 자리와 얼마나 맞는지 격자로 센다')
    ap.add_argument('--area', default='-2,12,-9,5', help='"xmin,xmax,ymin,ymax" 확인할 구역')
    a = ap.parse_args()

    src = os.path.expanduser(a.src)
    if not os.path.exists(src):
        raise SystemExit('★ 원본이 없다: %s' % src)
    P = read_layer(src)
    print('원본 %s' % src)
    print('  %d점   x %.2f~%.2f   y %.2f~%.2f'
          % (len(P), P[:, 0].min(), P[:, 0].max(), P[:, 1].min(), P[:, 1].max()))

    if a.from_tf:
        x0, y0, yaw0 = start_from_tf(a.map_frame, a.odom_frame, a.tf_wait)
    elif a.from_csv:
        x0, y0, yaw0 = start_from_csv(os.path.expanduser(a.from_csv))
    elif a.start:
        v = [float(t) for t in a.start.split(',')]
        if len(v) != 3:
            raise SystemExit('★ --start 는 "x,y,yaw" 세 값이어야 한다')
        x0, y0, yaw0 = v
    else:
        raise SystemExit('★ --start, --from-csv, --from-tf 중 하나가 필요하다')

    print()
    print('부팅 자세 (지도 좌표): x %.3f  y %.3f  방향 %.1f°' % (x0, y0, yaw0))
    print('  -> 장애물 좌표에 R(%.1f°) 회전과 (%.3f, %.3f) 평행이동의 역변환을 적용한다'
          % (-yaw0, -x0, -y0))
    Q = to_odom(P, x0, y0, yaw0)
    d = np.hypot(P[:, 0] - Q[:, 0], P[:, 1] - Q[:, 1])
    print('  옮겨진 거리: 중앙 %.2f m, 최대 %.2f m' % (np.median(d), d.max()))

    if a.check:
        # 경기장 안의 장애물만 본다. 선택은 **지도 좌표에서 한 번** 하고 양쪽에 똑같이 쓴다.
        # (각자의 좌표로 자르면 서로 다른 물체를 비교하게 된다.)
        ar = [float(t) for t in a.area.split(',')]
        keep = ((P[:, 0] > ar[0]) & (P[:, 0] < ar[1]) &
                (P[:, 1] > ar[2]) & (P[:, 1] < ar[3]))
        truth = cells(Q, keep)      # 그 물체들이 실제로 있는 odom 자리
        before = cells(P, keep)     # 변환 안 한 파일을 계획기가 놓는 자리
        print()
        print('경기장(지도 좌표 x %.0f~%.0f, y %.0f~%.0f)의 장애물 %d점 -> 점유 격자 %d칸'
              % (ar[0], ar[1], ar[2], ar[3], int(keep.sum()), len(truth)))
        for tag, S in (('지금 (변환 없음)', before), ('변환 후', truth)):
            if not truth:
                break
            print('  %-16s  맞춘 자리 %6.1f%%   유령벽 %6.1f%%   놓친 벽 %6.1f%%'
                  % (tag, 100.0 * len(truth & S) / len(truth),
                     100.0 * len(S - truth) / len(truth),
                     100.0 * len(truth - S) / len(truth)))

    if a.out:
        out = os.path.expanduser(a.out)
        if os.path.abspath(out) == os.path.abspath(src):
            raise SystemExit('★ 원본을 덮어쓰지 마라. 다른 경로를 줘라')
        write_layer(out, Q)
        print()
        print('저장: %s  (%d점, %d바이트)' % (out, len(Q), os.path.getsize(out)))
        print('쓰는 법 — ego_planner 설정에서:')
        print('    static_map_path: "%s"' % out)
        print('  계획기는 시작할 때 한 번만 읽는다. 주행 중에 바꾸면 반영되지 않는다.')
        print('  로봇을 위 부팅 자세가 아닌 곳에서 켜면 이 파일도 틀린다.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
