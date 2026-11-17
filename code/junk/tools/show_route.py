#!/usr/bin/env python3
"""기록한 경로를 그 지도 위에 그린다 — 경로가 벽·콘에 얼마나 붙는지 숫자와 그림으로.
사용: ~/venv/go2/bin/python3 ~/ws/junk/tools/show_route.py <이름>   → routes/<이름>/route.png
  빨간 점 = 벽·콘에서 0.25 m 안으로 지나가는 경로(몸 반폭 0.155 + 여유) — 기록을 다시 하거나 그 부분을 가운데로."""
import json
import os
import sys

import cv2
import numpy as np
from scipy import ndimage as ndi

J = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    d = os.path.join(J, 'routes', sys.argv[1])
    meta = dict(l.split(': ', 1) for l in open(os.path.join(d, 'map.yaml')).read().splitlines() if ': ' in l)
    res = float(meta['resolution']); ox, oy = [float(v) for v in meta['origin'].strip('[]').split(',')[:2]]
    img = cv2.imread(os.path.join(d, 'map.pgm'), cv2.IMREAD_GRAYSCALE); H, W = img.shape
    occ = img < 100
    dist = ndi.distance_transform_edt(~occ) * res
    route = json.load(open(os.path.join(d, 'route.json')))['route']
    P = np.array([[e['x'], e['y']] for e in route])
    # 점 사이를 2 cm 로 채워 경로 전체의 여유
    seg = np.vstack([np.linspace(P[i], P[i + 1], max(2, int(np.hypot(*(P[i + 1] - P[i])) / 0.02))) for i in range(len(P) - 1)])
    col = ((seg[:, 0] - ox) / res).astype(int); row = (H - 1 - (seg[:, 1] - oy) / res).astype(int)
    ok = (col >= 0) & (col < W) & (row >= 0) & (row < H)
    clr = np.full(len(seg), 9.0); clr[ok] = dist[row[ok], col[ok]]
    vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    for (c, r), m in zip(np.c_[col, row][ok], clr[ok]):
        cv2.circle(vis, (int(c), int(r)), 1, (0, 0, 255) if m < 0.25 else (0, 160, 0), -1)
    for e in route:
        if not e['label'].startswith('pin'):
            c = int((e['x'] - ox) / res); r = int(H - 1 - (e['y'] - oy) / res)
            cv2.circle(vis, (c, r), 5, (255, 0, 0), 2); cv2.putText(vis, e['label'] if not e['label'].startswith('line') else e['label'][4:], (c + 3, r - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 0, 0), 1)
    # 경로 둘레 1.2 m 만 잘라 4배(경기장만 크게)
    cs = ((P[:, 0] - ox) / res).astype(int); rs = (H - 1 - (P[:, 1] - oy) / res).astype(int); m_ = int(1.2 / res)
    vis = vis[max(0, rs.min() - m_):rs.max() + m_, max(0, cs.min() - m_):cs.max() + m_]
    vis = cv2.resize(vis, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST)
    out = os.path.join(d, 'route.png'); cv2.imwrite(out, vis)
    labs = [e['label'] for e in route if not e['label'].startswith('pin')]
    print(f'{out}\n  점 {len(route)}개 · 표시 {labs} · 벽·콘까지 최소 {clr.min():.2f} m · 0.25 m 안 {np.mean(clr < 0.25) * 100:.0f}%')


if __name__ == '__main__':
    main()
