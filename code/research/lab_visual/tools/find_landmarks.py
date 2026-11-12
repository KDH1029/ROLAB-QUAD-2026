#!/usr/bin/env python3
"""지도에서 기둥 모양 구조를 찾는다 — 시각 랜드마크의 지도 좌표를 얻기 위해서다.

  python3 find_landmarks.py --map ../maps/go2_prior_map_clean.pcd
  python3 find_landmarks.py --map ... --out ../config/landmarks.json --png /tmp/landmarks.png

【왜 필요한가】
YOLO 가 신호등을 찾아 방향각을 재면(정밀도 0.62° 실측) 위치가 구속된다.
그러려면 **그 신호등이 지도 어디에 있는지** 알아야 한다.
신호등은 가는 기둥에 달려 있고, 라이다 지도에도 그 기둥이 찍혀 있다. 그걸 찾는다.

【무엇을 기둥으로 보나】
  · 가로로 좁다          0.1 m 격자 몇 칸 안에만 점이 있다 (벽은 길게 이어진다)
  · 세로로 길다          바닥 위 0.3 m 부터 1.6 m 이상까지 점이 이어진다
  · 주변이 비어 있다     반경 0.6 m 안에 다른 구조가 없다 (벽에 붙은 돌출부 제외)

【한계】
지도만 보고는 '어느 기둥이 신호등인가' 를 모른다. 사람이 확인해서 이름을 붙여야 한다.
이 도구는 후보를 좁혀 줄 뿐이다.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from read_pcd import read_pcd


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default=os.path.expanduser('~/ws/lab_visual/maps/go2_prior_map_clean.pcd'))
    ap.add_argument('--cell', type=float, default=0.10, help='격자 크기 (m)')
    ap.add_argument('--zmin', type=float, default=0.3, help='바닥 위 하한')
    ap.add_argument('--zmax', type=float, default=2.2)
    ap.add_argument('--min-span', type=float, default=1.0, help='이 높이 이상 이어져야 기둥')
    ap.add_argument('--max-width', type=float, default=0.5, help='이보다 두꺼우면 벽으로 본다')
    ap.add_argument('--clear', type=float, default=0.6, help='주변이 이만큼 비어 있어야 한다')
    ap.add_argument('--area', default='-2,12,-9,5', help='"xmin,xmax,ymin,ymax"')
    ap.add_argument('--out', default='')
    ap.add_argument('--png', default='')
    a = ap.parse_args()

    xyz = read_pcd(os.path.expanduser(a.map))
    v = [float(x) for x in a.area.split(',')]
    m = ((xyz[:, 0] > v[0]) & (xyz[:, 0] < v[1]) & (xyz[:, 1] > v[2]) & (xyz[:, 1] < v[3]))
    xyz = xyz[m]
    floor = np.percentile(xyz[:, 2], 1.0)
    h = xyz[:, 2] - floor
    band = xyz[(h > a.zmin) & (h < a.zmax)]
    hb = h[(h > a.zmin) & (h < a.zmax)]
    print('지도 %s  구역 안 %d점 -> 대역 %d점  (바닥 z=%.2f)'
          % (os.path.basename(a.map), len(xyz), len(band), floor))

    # 격자마다 z 범위를 본다
    key = np.floor(band[:, :2] / a.cell).astype(np.int64)
    uniq, inv = np.unique(key, axis=0, return_inverse=True)
    zmin = np.full(len(uniq), 1e9)
    zmax = np.full(len(uniq), -1e9)
    cnt = np.zeros(len(uniq), int)
    np.minimum.at(zmin, inv, hb)
    np.maximum.at(zmax, inv, hb)
    np.add.at(cnt, inv, 1)
    span = zmax - zmin
    tall = (span > a.min_span) & (cnt > 20)
    print('  격자 %d칸 중 세로로 %.1f m 이상 이어진 칸 %d개' % (len(uniq), a.min_span, tall.sum()))
    if not tall.any():
        print('  ★ 기둥 후보가 없다 — --min-span 을 낮춰 볼 것')
        return 1

    # 인접한 칸을 하나의 기둥으로 묶는다
    cells = uniq[tall]
    cs = set(map(tuple, cells))
    seen = set()
    groups = []
    for c in map(tuple, cells):
        if c in seen:
            continue
        stack = [c]
        seen.add(c)
        g = []
        while stack:
            p = stack.pop()
            g.append(p)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    q = (p[0] + dx, p[1] + dy)
                    if q in cs and q not in seen:
                        seen.add(q)
                        stack.append(q)
        groups.append(g)

    # 굵기와 주변 여유로 거른다
    tree_pts = band[:, :2]
    from scipy.spatial import cKDTree
    kd = cKDTree(tree_pts)
    out = []
    for g in groups:
        arr = (np.array(g) + 0.5) * a.cell
        w = max(arr[:, 0].max() - arr[:, 0].min(), arr[:, 1].max() - arr[:, 1].min()) + a.cell
        if w > a.max_width:
            continue
        cx, cy = arr[:, 0].mean(), arr[:, 1].mean()
        near = kd.query_ball_point((cx, cy), a.clear, return_length=True)
        inside = kd.query_ball_point((cx, cy), w / 2 + a.cell, return_length=True)
        if near > inside * 2.5:          # 주변에 딴 구조가 많으면 벽의 일부다
            continue
        idx = [i for i, c in enumerate(map(tuple, uniq)) if c in set(g)]
        out.append(dict(x=round(float(cx), 3), y=round(float(cy), 3),
                        width=round(float(w), 2),
                        height=round(float(span[idx].max()), 2),
                        points=int(cnt[idx].sum())))
    out.sort(key=lambda d: -d['height'])
    print('  기둥 후보 %d개 (굵기 %.1f m 이하, 주변이 비어 있음)' % (len(out), a.max_width))
    print('   번호    x       y     굵기   높이   점수')
    for i, d in enumerate(out[:20]):
        print('   %2d  %+7.2f %+7.2f  %.2f  %.2f  %5d'
              % (i + 1, d['x'], d['y'], d['width'], d['height'], d['points']))

    if a.out:
        js = {'map': os.path.basename(a.map), 'frame': 'map',
              'note': '지도에서 자동으로 찾은 기둥 후보. 어느 것이 신호등인지는 사람이 확인해 name 을 채운다.',
              'landmarks': [dict(id=i + 1, name='', **d) for i, d in enumerate(out)]}
        os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
        json.dump(js, open(a.out, 'w'), ensure_ascii=False, indent=1)
        print('  저장:', a.out)

    if a.png:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(11, 9))
        ax.scatter(band[::3, 0], band[::3, 1], s=0.3, c='0.7')
        for i, d in enumerate(out[:20]):
            ax.plot(d['x'], d['y'], 'o', ms=9, mfc='none', mec='r', mew=2)
            ax.text(d['x'] + 0.15, d['y'] + 0.15, str(i + 1), color='r', fontsize=11)
        ax.set_aspect('equal')
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_title('pole candidates (%d)' % len(out))
        ax.grid(alpha=0.2)
        plt.tight_layout()
        plt.savefig(a.png, dpi=100)
        print('  그림:', a.png)
    return 0


if __name__ == '__main__':
    sys.exit(main())
