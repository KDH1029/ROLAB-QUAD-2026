#!/usr/bin/env python3
"""만든 지도 미리보기 — 벽(검정) + 녹화 때 지나간 길(파랑, 출발 초록·끝 빨강), 1 m 격자.
사용: ~/venv/go2/bin/python3 ~/ws/junk/tools/show_map.py <이름>   → routes/<이름>/map_preview.png"""
import os
import sys

import cv2
import numpy as np

J = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    d = os.path.join(J, 'routes', sys.argv[1])
    c = dict(np.load(os.path.join(d, 'editor_cache.npz')))
    W, P = c['walls'], c['path']
    x0, y0 = np.minimum(W.min(0), P.min(0)) - 0.5
    x1, y1 = np.maximum(W.max(0), P.max(0)) + 0.5
    # 경로 둘레 4 m 안만(먼 전시장 잘라냄)
    x0, y0 = max(x0, P[:, 0].min() - 4), max(y0, P[:, 1].min() - 4)
    x1, y1 = min(x1, P[:, 0].max() + 4), min(y1, P[:, 1].max() + 4)
    s = 100  # px/m
    w, h = int((x1 - x0) * s) + 1, int((y1 - y0) * s) + 1
    img = np.full((h, w, 3), 255, np.uint8)
    px = lambda x, y: (int((x - x0) * s), int((y1 - y) * s))
    for gx in np.arange(np.ceil(x0), x1, 1.0):
        cv2.line(img, px(gx, y0), px(gx, y1), (225, 225, 225), 1)
    for gy in np.arange(np.ceil(y0), y1, 1.0):
        cv2.line(img, px(x0, gy), px(x1, gy), (225, 225, 225), 1)
    for x, y in W:
        if x0 <= x <= x1 and y0 <= y <= y1:
            cv2.rectangle(img, px(x - 0.02, y + 0.02), px(x + 0.02, y - 0.02), (0, 0, 0), -1)
    pts = np.array([px(x, y) for x, y in P], np.int32)
    cv2.polylines(img, [pts], False, (220, 120, 0), 2)
    cv2.circle(img, px(*P[0]), 8, (0, 170, 0), -1); cv2.circle(img, px(*P[-1]), 8, (0, 0, 220), -1)
    cv2.putText(img, f'{sys.argv[1]}  path {np.hypot(*np.diff(P, axis=0).T).sum():.1f} m  (green=start, red=end, grid 1 m)',
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
    out = os.path.join(d, 'map_preview.png'); cv2.imwrite(out, img)
    print(out, f'· 길 {np.hypot(*np.diff(P, axis=0).T).sum():.1f} m · 시작-끝 {np.hypot(*(P[-1] - P[0])):.2f} m · 벽 점 {len(W)}')


if __name__ == '__main__':
    main()
