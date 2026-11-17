#!/usr/bin/env python3
"""장애물 층을 지도에서 다시 굽는다 — 바닥면을 맞춰서 지도 기울기에 속지 않게.

  python3 make_obstacle_layer.py --check
  python3 make_obstacle_layer.py --out ../maps/room_obstacles_map.bin --png /tmp/layer.png

【무엇이 잘못됐었나】
지금 쓰는 `room_obstacles.bin` 은 코스 바닥 격자의 **93%** 를 장애물로 표시한다.
0.45 m 팽창 뒤 코스 안 자유공간이 4.3 m² 짜리 섬 하나뿐이라 계획기가 아무 데도 못 간다.

원인은 지도가 **5.51° 기울어 있다**는 것이다. FAST-LIO 지도는 출발 순간의 기울기를 물려받는다.
바닥은 실제로 평평한데(평면 잔차 표준편차 0.026 m), 지도 좌표에서는 10 m 가로지르는 동안
바닥 높이가 0.96 m 씩 달라진다. 그래서 "바닥 위 몇 m" 를 **z 값 하나로** 자르면
바닥이 통째로 장애물 대역에 들어온다.

【고치는 법】
바닥면을 최소제곱으로 맞춘 뒤, **그 평면으로부터의 높이**로 자른다.
기본 대역 0.6~1.6 m — 실측으로 이 대역이 깨끗하다 (경기장 벽 + 마네킹 + 기둥만 남는다).
0.3~0.6 m 는 아직 바닥 잔여물이 섞인다 (매핑 중 흔들림으로 바닥이 위아래로 번진 것).

【나오는 것】
`ego_planner` 의 loadPriorMap 형식 (uint32 개수 + float32 x,y 쌍), **지도(map) 좌표**로 저장한다.
로봇에 쓸 때는 align_obstacle_layer.py 로 부팅 자세에 맞춰 한 번 더 돌려야 한다.
계획기가 좌표 변환을 안 하기 때문이다.
"""
import argparse
import math
import os
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from read_pcd import read_pcd

RES = 0.10                          # ego_planner 의 격자 크기
INFLATE = 0.25                      # ego_planner.yaml 의 safe_dist 와 맞춰야 뜻이 있다


def fit_floor(A, keep=0.05, iters=10):
    """바닥면을 맞춘다. 낮은 쪽에서 시작해 반복적으로 다듬는다.

    돌려주는 것: (계수 [c0,cx,cy], 바닥으로 판정된 점 수, 잔차 표준편차)
    잔차 표준편차가 크면 바닥이 평평하지 않거나 맞추기가 실패한 것이다.
    """
    sel = A[:, 2] < np.percentile(A[:, 2], 30)
    c = None
    for _ in range(iters):
        if sel.sum() < 100:
            raise SystemExit('★ 바닥으로 쓸 점이 너무 적다 — 구역(--area)을 확인할 것')
        X = np.c_[np.ones(sel.sum()), A[sel, 0], A[sel, 1]]
        c, *_ = np.linalg.lstsq(X, A[sel, 2], rcond=None)
        r = A[:, 2] - (c[0] + c[1] * A[:, 0] + c[2] * A[:, 1])
        sel = np.abs(r) < keep
    return c, int(sel.sum()), float(np.std(r[sel]))


def occupied(pts):
    ij = np.floor(pts / RES).astype(np.int64)
    base = set(map(tuple, ij))
    r = int(math.ceil(INFLATE / RES))
    out = set()
    for dx in range(-r, r + 1):
        for dy in range(-r, r + 1):
            if dx * dx + dy * dy <= r * r:
                out |= {(i + dx, j + dy) for i, j in base}
    return out


def free_report(pts, area, tag):
    """그 층으로 계획기가 다닐 수 있는 자유공간이 얼마나 되는지 센다."""
    occ = occupied(pts)
    nx = int((area[1] - area[0]) / RES)
    ny = int((area[3] - area[2]) / RES)
    i0 = int(math.floor(area[0] / RES))
    j0 = int(math.floor(area[2] / RES))
    free = sum(1 for i in range(nx) for j in range(ny)
               if (i0 + i, j0 + j) not in occ)
    tot = nx * ny
    print('  %-22s 자유공간 %6.1f m² / %.0f m²  (%.0f%%)'
          % (tag, free * RES * RES, tot * RES * RES, 100.0 * free / tot))
    return free, tot


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default=os.path.expanduser(
        '~/ws/lab_visual/maps/go2_prior_map_clean.pcd'))
    ap.add_argument('--area', default='-2,14,-9,6',
                    help='"xmin,xmax,ymin,ymax" 층을 뽑아낼 구역 (지도 좌표)')
    ap.add_argument('--fit-area', default='1.0,8.0,-5.0,1.5',
                    help='바닥면을 맞출 구역. **경기장 안쪽의 확실히 평평한 곳**만 줘야 한다. '
                         '계단이나 단이 섞이면 평면이 틀어진다. 바닥이 평평하면 여기서 맞춘 '
                         '평면을 넓은 구역에 그대로 늘려 써도 된다')
    ap.add_argument('--zmin', type=float, default=0.60,
                    help='바닥면 위 이 높이부터 장애물로 본다. 0.6 아래는 바닥 잔여물이 섞인다')
    ap.add_argument('--zmax', type=float, default=1.60,
                    help='이 높이까지. 로봇보다 충분히 높으면 된다')
    ap.add_argument('--min-hits', type=int, default=20,
                    help='0.1 m 칸이 이 횟수 이상 찍혀야 남긴다. 이게 핵심이다 — '
                         '벽은 주행 내내 수백~수천 번 찍히지만, 지나가던 사람은 몇 번뿐이다')
    ap.add_argument('--out', default='')
    ap.add_argument('--check', action='store_true', help='지금 쓰는 층과 자유공간을 비교한다')
    ap.add_argument('--compare', default=os.path.expanduser('~/go2_ws/maps/room_obstacles.npy'))
    ap.add_argument('--inflate', type=float, default=INFLATE,
                    help='자유공간을 셀 때 쓸 팽창 반경. ego_planner.yaml 의 safe_dist 와 같게 둘 것')
    ap.add_argument('--png', default='')
    a = ap.parse_args()
    globals()['INFLATE'] = a.inflate

    P = read_pcd(os.path.expanduser(a.map))
    ar = [float(v) for v in a.area.split(',')]
    fa = [float(v) for v in a.fit_area.split(',')]
    m = ((P[:, 0] > ar[0]) & (P[:, 0] < ar[1]) & (P[:, 1] > ar[2]) & (P[:, 1] < ar[3]))
    A = P[m]
    mf = ((P[:, 0] > fa[0]) & (P[:, 0] < fa[1]) & (P[:, 1] > fa[2]) & (P[:, 1] < fa[3]))
    F = P[mf]
    print('지도 %s' % os.path.basename(a.map))
    print('  뽑아낼 구역 x %.1f~%.1f y %.1f~%.1f : %d점' % (*ar, len(A)))
    print('  바닥 맞출 구역 x %.1f~%.1f y %.1f~%.1f : %d점' % (*fa, len(F)))

    c, nfloor, resid = fit_floor(F)
    tilt = math.degrees(math.atan(math.hypot(c[1], c[2])))
    span = max(ar[1] - ar[0], ar[3] - ar[2])
    print('바닥면: z = %.4f %+.5f·x %+.5f·y' % tuple(c))
    print('  기울기 %.2f°  ->  %.0f m 가로지르면 바닥 z 가 %.2f m 달라진다'
          % (tilt, span, span * math.tan(math.radians(tilt))))
    print('  바닥 점 %d개, 평면으로부터 잔차 표준편차 %.3f m' % (nfloor, resid))
    if resid > 0.06:
        print('  ★ 잔차가 크다 — 바닥이 평평하지 않거나 구역에 단이 섞여 있다')
    if tilt > 1.0:
        print('  ★ 기울기 %.2f° 는 무시할 수 없다. z 값 하나로 자르면 바닥이 장애물이 된다.'
              % tilt)

    h = A[:, 2] - (c[0] + c[1] * A[:, 0] + c[2] * A[:, 1])
    band = A[(h > a.zmin) & (h < a.zmax)]
    print()
    print('바닥면 위 %.2f~%.2f m 대역: %d점' % (a.zmin, a.zmax, len(band)))
    if len(band) < 100:
        raise SystemExit('★ 대역에 점이 거의 없다 — --zmin/--zmax 를 확인할 것')

    # 찍힌 횟수로 거른다 — 붙박이와 지나가던 사람을 가르는 것은 밀도다.
    # 실측: 칸당 점수 중앙 3, 90퍼센타일 96, 99퍼센타일 542. 180배 차이가 난다.
    g = np.floor(band[:, :2] / RES).astype(np.int64)
    gu, cnt = np.unique(g, axis=0, return_counts=True)
    print('  0.1 m 칸 %d개.  칸당 찍힌 횟수: 중앙 %d, 75%% %d, 90%% %d, 99%% %d'
          % (len(gu), *[int(np.percentile(cnt, q)) for q in (50, 75, 90, 99)]))
    gu = gu[cnt >= a.min_hits]
    pts = (gu + 0.5) * RES
    print('  %d회 이상 찍힌 칸만 남기면 %d칸 (%.1f m²)' % (a.min_hits, len(pts), len(pts) * RES * RES))
    if len(pts) < 50:
        raise SystemExit('★ 남은 칸이 너무 적다 — --min-hits 를 낮출 것')

    if a.check:
        print()
        print('경기장 x %.1f~%.1f, y %.1f~%.1f 에서 계획기가 쓸 수 있는 자유공간:'
              % tuple(ar))
        old = os.path.expanduser(a.compare)
        if os.path.exists(old):
            O = np.load(old).astype(np.float64) if old.endswith('.npy') else None
            if O is not None:
                mo = ((O[:, 0] > ar[0]) & (O[:, 0] < ar[1]) &
                      (O[:, 1] > ar[2]) & (O[:, 1] < ar[3]))
                free_report(O[mo], ar, '지금 쓰는 층')
        free_report(pts, ar, '새로 구운 층')

    if a.out:
        out = os.path.expanduser(a.out)
        os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
        with open(out, 'wb') as f:
            f.write(struct.pack('<I', len(pts)))
            f.write(pts.astype('<f4').tobytes())
        print()
        print('저장: %s  (%d점, %d바이트)  ** 지도(map) 좌표다 **'
              % (out, len(pts), os.path.getsize(out)))
        print('로봇에 쓰기 전에 부팅 자세로 한 번 더 돌려야 한다:')
        print('    python3 align_obstacle_layer.py --src %s --from-tf --out <최종파일>' % out)

    if a.png:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(15, 6.6))
        ax[0].scatter(pts[:, 0], pts[:, 1], s=1.6, c='k')
        ax[0].set_title('new obstacle layer (%d pts)' % len(pts))
        occ = occupied(pts)
        nx = int((ar[1] - ar[0]) / RES)
        ny = int((ar[3] - ar[2]) / RES)
        i0 = int(math.floor(ar[0] / RES))
        j0 = int(math.floor(ar[2] / RES))
        img = np.array([[0 if (i0 + i, j0 + j) in occ else 1
                         for i in range(nx)] for j in range(ny)])
        ax[1].imshow(img, origin='lower', extent=ar, cmap='gray', vmin=0, vmax=1)
        ax[1].set_title('free space after %.2f m inflate (white = drivable)' % INFLATE)
        for x in ax:
            x.set_aspect('equal')
            x.set_xlim(ar[0], ar[1])
            x.set_ylim(ar[2], ar[3])
            x.grid(alpha=.2)
        plt.tight_layout()
        plt.savefig(a.png, dpi=105)
        print('  그림: %s' % a.png)
    return 0


if __name__ == '__main__':
    sys.exit(main())
