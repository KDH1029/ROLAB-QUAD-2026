#!/usr/bin/env python3
"""지도 원점이 경기장 어디인지 줄자로 찾을 수 있게 알려준다.

  python3 start_pose_guide.py
  python3 start_pose_guide.py --pose 0,0,0 --png /tmp/start.png

【왜 필요한가】
ego_planner 는 장애물 층을 `camera_init`(FAST-LIO 가 **부팅한 자리**) 기준으로 쓴다.
그러니 매핑 주행을 시작했던 그 자리에서 부팅하면 어긋남이 0 이 된다.
문제는 "그 자리" 가 바닥 어디인지 사람이 모른다는 것이다.

그래서 지도에서 **원점에서 사방으로 줄자를 쐈을 때의 거리**를 뽑아 준다.
경기장에서 같은 거리가 나오는 자리를 찾으면 그곳이 원점이다.

【쓰는 법】
1. 아래 표의 거리 중 벽처럼 또렷한 방향 두세 개를 고른다.
2. 경기장에서 줄자로 그 거리가 되는 지점을 찾아 바닥에 표시한다.
3. 로봇 몸통 중심을 그 표시에 놓고, 표의 '앞(0°)' 방향으로 맞춘 뒤 부팅한다.

【정밀도】
지도 격자와 라이다 분해능을 감안하면 ±5 cm, ±2° 면 충분하다.
그 정도 어긋남은 장애물 층을 5 cm 밀 뿐이고, 팽창 반경 0.45 m 안에 묻힌다.
"""
import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from read_pcd import read_pcd


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default=os.path.expanduser(
        '~/ws/lab_visual/maps/go2_prior_map_clean.pcd'))
    ap.add_argument('--pose', default='0,0,0', help='"x,y,yaw" 재고 싶은 자세 (지도 좌표)')
    ap.add_argument('--zmin', type=float, default=0.30, help='바닥 위 이 높이부터 본다')
    ap.add_argument('--zmax', type=float, default=1.50, help='이 높이까지 본다 (줄자 높이)')
    ap.add_argument('--every', type=float, default=22.5, help='몇 도 간격으로 잴지')
    ap.add_argument('--wedge', type=float, default=4.0, help='각 방향의 부채꼴 폭 (도)')
    ap.add_argument('--min-range', type=float, default=0.0,
                    help='이 거리 안쪽은 무시한다. 매핑 시작 때 옆에 서 있던 사람이나 '
                         '로봇 자신이 찍혀 있으므로 1.0 정도를 주는 게 낫다')
    ap.add_argument('--max-range', type=float, default=30.0)
    ap.add_argument('--png', default='')
    a = ap.parse_args()

    xyz = read_pcd(os.path.expanduser(a.map))
    floor = np.percentile(xyz[:, 2], 1.0)
    h = xyz[:, 2] - floor
    band = xyz[(h > a.zmin) & (h < a.zmax)]
    x0, y0, yaw0 = [float(v) for v in a.pose.split(',')]
    print('지도 %s   %d점 중 높이 %.2f~%.2f m 대역 %d점  (바닥 z=%.2f)'
          % (os.path.basename(a.map), len(xyz), a.zmin, a.zmax, len(band), floor))
    print('기준 자세: x %.3f  y %.3f  방향 %.1f°' % (x0, y0, yaw0))
    print()

    dx = band[:, 0] - x0
    dy = band[:, 1] - y0
    r = np.hypot(dx, dy)
    ang = np.degrees(np.arctan2(dy, dx)) - yaw0
    ang = (ang + 180.0) % 360.0 - 180.0

    print('  로봇 앞을 0°로 놓고, 왼쪽이 +. 각 방향에서 처음 만나는 구조까지의 거리:')
    print()
    print('   방향      거리      점수   비고')
    n = int(round(360.0 / a.every))
    rows = []
    for k in range(n):
        th = -180.0 + k * (360.0 / n)
        m = ((np.abs((ang - th + 180.0) % 360.0 - 180.0) < a.wedge / 2)
             & (r > a.min_range) & (r < a.max_range))
        if m.sum() < 8:
            print('   %+6.1f°      —            %4d   (점이 없다)' % (th, m.sum()))
            rows.append((th, float('nan'), int(m.sum())))
            continue
        rr = r[m]
        d = float(np.percentile(rr, 3.0))      # 가장 가까운 쪽. 잡음 한두 점에 안 흔들리게 3퍼센타일
        spread = float(np.percentile(rr, 20.0) - d)
        note = '또렷함 ★' if spread < 0.15 else ('쓸 만함' if spread < 0.5 else '흐릿함')
        print('   %+6.1f°   %6.2f m       %4d   %s' % (th, d, m.sum(), note))
        rows.append((th, d, int(m.sum())))

    good = [(t, d) for t, d, c in rows if np.isfinite(d)]
    if good:
        good.sort(key=lambda z: z[1])
        print()
        print('  가장 가까운 구조: %+.1f° 방향 %.2f m' % (good[0][0], good[0][1]))
        print('  줄자로 재기 좋은 방향(가깝고 또렷한 것부터):')
        for t, d in good[:4]:
            print('     %+6.1f°  %.2f m' % (t, d))

    if a.png:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(10, 9))
        ax.scatter(band[::4, 0], band[::4, 1], s=0.4, c='0.75')
        ax.plot([x0], [y0], 'o', ms=12, mfc='none', mec='r', mew=2.5)
        L = 1.6
        ax.arrow(x0, y0, L * math.cos(math.radians(yaw0)), L * math.sin(math.radians(yaw0)),
                 head_width=0.32, color='r', length_includes_head=True)
        for t, d, c in rows:
            if not np.isfinite(d):
                continue
            th = math.radians(yaw0 + t)
            ax.plot([x0, x0 + d * math.cos(th)], [y0, y0 + d * math.sin(th)],
                    '-', lw=0.8, color='tab:blue', alpha=0.65)
            ax.text(x0 + d * math.cos(th), y0 + d * math.sin(th), '%.1f' % d,
                    fontsize=7, color='tab:blue')
        ax.set_aspect('equal')
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_title('start pose (%.2f, %.2f, %.0f deg) — tape measure distances' % (x0, y0, yaw0))
        ax.grid(alpha=0.2)
        plt.tight_layout()
        plt.savefig(a.png, dpi=110)
        print('  그림: %s' % a.png)
    return 0


if __name__ == '__main__':
    sys.exit(main())
