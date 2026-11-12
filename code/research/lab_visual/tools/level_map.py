#!/usr/bin/env python3
"""지도를 수평으로 세운다 — 한 번 해 두면 그 뒤로는 z 가 곧 바닥 위 높이가 된다.

  python3 level_map.py --check
  python3 level_map.py --out ../maps/go2_prior_map_level.pcd --also ../maps/go2_prior_map_v05.pcd

【왜 필요한가】
FAST-LIO 지도의 z 축은 중력이 아니라 **로봇이 부팅한 순간의 자세**다.
경기장 바닥은 완전한 평지인데(대회 자료), 지도 좌표에서는 10 m 가는 동안 바닥 z 가 약 1 m 달라진다.
이건 경기장이 기운 게 아니라 지도가 기운 것이다. 두 가지로 확인했다:
  · 바닥면을 맞추면 법선이 z 축에서 4.55° 기울어 있다 (방위 +2.6°)
  · 실제로는 수직인 신호등 기둥 13개가 중앙값 6.53° 누워 있다 (방위 평균 +1.2°)
  두 방위가 일치한다 -> 지도 전체가 한 방향으로 기울어 있다.

기울어진 채로 두면 "바닥 위 몇 m" 를 z 값으로 못 자른다. 장애물 층이 망가진 원인이 이것이다.

【무엇을 하나】
바닥면의 법선을 z 축으로 돌리는 **회전 하나**를 구해 점군 전체에 적용하고,
바닥이 z=0 이 되게 내린다. 회전이므로 거리와 모양은 하나도 안 변한다.

【주의 — 좌표가 바뀐다】
이미 찍어 둔 웨이포인트와 장애물 층도 **같은 변환을 적용해야** 맞는다.
--matrix 로 나오는 4x4 행렬을 쓰면 된다. 지도만 바꾸고 웨이포인트를 그냥 두면 전부 어긋난다.
"""
import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from read_pcd import read_pcd


def write_pcd(path, P):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'wb') as f:
        f.write(('# .PCD v0.7 - Point Cloud Data file format\n'
                 'VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                 'WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n'
                 % (len(P), len(P))).encode())
        f.write(P.astype('<f4').tobytes())


def fit_floor(A, keep=0.04, iters=12):
    sel = A[:, 2] < np.percentile(A[:, 2], 30)
    for _ in range(iters):
        X = np.c_[np.ones(sel.sum()), A[sel, 0], A[sel, 1]]
        c, *_ = np.linalg.lstsq(X, A[sel, 2], rcond=None)
        r = A[:, 2] - (c[0] + c[1] * A[:, 0] + c[2] * A[:, 1])
        sel = np.abs(r) < keep
    return c, int(sel.sum()), float(np.std(r[sel]))


def rotation_to_z(n):
    """법선 n 을 +z 로 보내는 최소 회전. 축은 n×z, 각은 둘 사이 각."""
    n = n / np.linalg.norm(n)
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(n, z)
    s = np.linalg.norm(v)
    if s < 1e-12:
        return np.eye(3)
    c = float(np.dot(n, z))
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + K + K @ K * ((1 - c) / (s * s))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default=os.path.expanduser(
        '~/ws/lab_visual/maps/go2_prior_map_clean.pcd'))
    ap.add_argument('--fit-area', default='1.0,8.0,-5.0,1.5',
                    help='"xmin,xmax,ymin,ymax" 바닥을 맞출 구역. 경기장 안쪽 평평한 곳만')
    ap.add_argument('--out', default='', help='수평으로 세운 지도를 저장할 경로')
    ap.add_argument('--also', default='', help='같은 변환을 적용할 지도를 더 지정 (쉼표로 여러 개)')
    ap.add_argument('--check', action='store_true', help='세운 뒤 바닥이 정말 평평해졌는지 센다')
    ap.add_argument('--matrix', action='store_true', help='4x4 변환 행렬을 출력한다')
    a = ap.parse_args()

    P = read_pcd(os.path.expanduser(a.map))
    fa = [float(v) for v in a.fit_area.split(',')]
    m = ((P[:, 0] > fa[0]) & (P[:, 0] < fa[1]) & (P[:, 1] > fa[2]) & (P[:, 1] < fa[3]))
    c, nfl, resid = fit_floor(P[m])
    n = np.array([-c[1], -c[2], 1.0])
    n /= np.linalg.norm(n)
    tilt = math.degrees(math.acos(min(1.0, n[2])))
    az = math.degrees(math.atan2(n[1], n[0]))
    print('지도 %s  %d점' % (os.path.basename(a.map), len(P)))
    print('  바닥 맞춤 구역 안 %d점 -> 바닥 점 %d개, 평면 잔차 sd %.3f m' % (m.sum(), nfl, resid))
    print('  바닥 법선 %s' % np.round(n, 4))
    print('  **기울기 %.2f°  방위 %+.1f°**  (10 m 가면 바닥 z 가 %.2f m 달라진다)'
          % (tilt, az, 10 * math.tan(math.radians(tilt))))
    if resid > 0.05:
        print('  ★ 잔차가 크다 — --fit-area 에 단이나 계단이 섞였는지 확인할 것')

    R = rotation_to_z(n)
    Q = P @ R.T
    # 바닥을 z=0 으로
    mq = ((Q[:, 0] > fa[0] * 1.0) & (Q[:, 0] < fa[1] * 1.2) &
          (Q[:, 1] > fa[2] * 1.2) & (Q[:, 1] < fa[3] * 1.2))
    dz = np.percentile(Q[mq if mq.sum() > 1000 else slice(None), 2], 1.0)
    Q[:, 2] -= dz
    print('  회전 적용, 바닥을 z=0 으로 내림 (%.3f m)' % dz)

    if a.check:
        mm = ((Q[:, 0] > fa[0]) & (Q[:, 0] < fa[1]) & (Q[:, 1] > fa[2]) & (Q[:, 1] < fa[3]))
        A = Q[mm]
        k = np.floor(A[:, :2] / 0.5).astype(np.int64)
        u, inv = np.unique(k, axis=0, return_inverse=True)
        lo = np.full(len(u), 9.0)
        np.minimum.at(lo, inv, A[:, 2])
        cnt = np.zeros(len(u), int)
        np.add.at(cnt, inv, 1)
        v = lo[cnt > 100]
        print()
        print('  세운 뒤 국소 바닥 높이 (0.5 m 칸 %d개): 중앙 %+.3f  5~95%% %+.3f~%+.3f  폭 %.3f m'
              % (len(v), np.median(v), np.percentile(v, 5), np.percentile(v, 95),
                 np.percentile(v, 95) - np.percentile(v, 5)))
        print('    (경기장이 평지이므로 이 폭이 작을수록 잘 세워진 것이다)')
        for lo_, hi_, nm in ((-0.05, 0.05, '바닥'), (0.05, 0.30, '낮은 것 (나무판자 등)'),
                             (0.30, 0.60, '중간'), (0.60, 1.60, '벽·기둥·마네킹'),
                             (1.60, 9.0, '그 위')):
            kk = (Q[:, 2] >= lo_) & (Q[:, 2] < hi_)
            print('    %-22s z %+5.2f~%+5.2f : %8d점' % (nm, lo_, hi_, kk.sum()))

    if a.matrix:
        T = np.eye(4)
        T[:3, :3] = R
        T[2, 3] = -dz
        print()
        print('  4x4 변환 행렬 (웨이포인트·장애물 층에도 똑같이 적용할 것):')
        for row in T:
            print('    [%+.6f %+.6f %+.6f %+.6f]' % tuple(row))

    if a.out:
        write_pcd(os.path.expanduser(a.out), Q)
        print()
        print('저장: %s  (%d점)' % (a.out, len(Q)))
        for extra in [x for x in a.also.split(',') if x.strip()]:
            E = read_pcd(os.path.expanduser(extra.strip()))
            F = E @ R.T
            F[:, 2] -= dz
            o2 = os.path.expanduser(extra.strip()).replace('.pcd', '_level.pcd')
            write_pcd(o2, F)
            print('  같이 변환: %s -> %s (%d점)' % (os.path.basename(extra), o2, len(F)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
