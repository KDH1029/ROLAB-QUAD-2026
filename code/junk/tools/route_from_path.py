#!/usr/bin/env python3
"""경로 초안 = 녹화 때 실제로 지나간 길. 콘이 고정이라 그때 콘 사이로 지나간 길이 그대로 안전한 길이다.
사용: ~/venv/go2/bin/python3 ~/ws/junk/tools/route_from_path.py <이름> [간격 m=0.5]
  → routes/<이름>/route.json (start · line … · exit). 편집기에서 정지선·주차 점을 골라 4 / 7 로 바꾸면 끝.
  이미 route.json 이 있으면 route.json.bak_<시각> 으로 남기고 새로 만든다."""
import json
import os
import shutil
import sys
import time

import numpy as np

J = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    name = sys.argv[1]; step = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
    d = os.path.join(J, 'routes', name)
    P = dict(np.load(os.path.join(d, 'editor_cache.npz')))['path'].astype(float)
    keep = [0]                                           # 멈춰 있던·제자리 돈 부분은 하나로
    for k in range(1, len(P)):
        if np.hypot(*(P[k] - P[keep[-1]])) >= 0.03:
            keep.append(k)
    Q = P[keep]
    n = 5                                                # 걸음 흔들림 펴기(앞뒤 5점 ≈ 0.3 m)
    S = np.array([Q[max(0, i - n):i + n + 1].mean(0) for i in range(len(Q))]); S[0], S[-1] = Q[0], Q[-1]
    s = np.r_[0.0, np.cumsum(np.hypot(*np.diff(S, axis=0).T))]
    pts = [S[int(np.searchsorted(s, v))] for v in np.arange(step, s[-1] - step * 0.5, step)]
    route = [dict(label='start', x=round(float(S[0, 0]), 3), y=round(float(S[0, 1]), 3))]
    route += [dict(label=f'line{i + 1}', x=round(float(p[0]), 3), y=round(float(p[1]), 3)) for i, p in enumerate(pts)]
    route += [dict(label='exit', x=round(float(S[-1, 0]), 3), y=round(float(S[-1, 1]), 3))]
    out = os.path.join(d, 'route.json')
    if os.path.exists(out):
        shutil.copy(out, out + time.strftime('.bak_%H%M%S'))
    json.dump({'format': 'route_v1', 'source': 'route_from_path', 'route': route,
               'note': '녹화 때 지나간 길(0.03 m 미만 이동 버림, 0.3 m 이동평균). 정지선·주차는 편집기에서 4/7 로'},
              open(out, 'w'), ensure_ascii=False, indent=1)
    print(f'{out}: 점 {len(route)}개(간격 {step} m, 길이 {s[-1]:.1f} m) — 편집기에서 정지선(4)·주차(7) 지정')


if __name__ == '__main__':
    main()
