#!/usr/bin/env python3
"""러너 '목표 보정: 벽까지 c0' 짝 비교 — 같은 경로점에서 실제(16:54 live 벽 구름) vs 시뮬(지도 벽 칸)의 벽까지 거리. simcal 11-06"""
import re, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from timeline import read_lines, REAL_LOG, REAL_LINES
def snaps(ev):
    d = {}
    for t, m in ev:
        g = re.match(r'\[(\w+)\] 목표 보정: 벽까지 ([\d.]+) m', m)
        if g: d.setdefault(g.group(1), []).append(float(g.group(2)))
    return d
R = snaps(read_lines(REAL_LOG, REAL_LINES))
for f in sys.argv[1:]:
    S = snaps(read_lines(os.path.join(f, 'runner.log')))
    print('###', os.path.basename(f.rstrip('/')))
    diffs = []
    for k in R:
        if k in S:
            diffs.append(R[k][0] - S[k][0])
            print(f'  {k:9s} 실제 첫 {R[k][0]:.2f} (최소 {min(R[k]):.2f}, {len(R[k])}번) · 시뮬 첫 {S[k][0]:.2f} (최소 {min(S[k]):.2f}, {len(S[k])}번) · 실제−시뮬 {R[k][0]-S[k][0]:+.2f}')
        else:
            print(f'  {k:9s} 실제 첫 {R[k][0]:.2f} · 시뮬 보정 없음(≥0.40)')
    for k in S:
        if k not in R: print(f'  {k:9s} 실제 보정 없음(≥0.40) · 시뮬 첫 {S[k][0]:.2f}')
    import numpy as np
    print(f'  짝 {len(diffs)}개 · 실제−시뮬 평균 {np.mean(diffs):+.3f} m · 중앙 {np.median(diffs):+.3f} m')
