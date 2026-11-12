#!/usr/bin/env python3
"""구간별 이동 시간 막대: 실제 16:54 vs 시뮬 여러 판(+ 회전 시간). simcal. 사용: plot_timeline.py out.png 이름=폴더 ..."""
import sys, os
import numpy as np
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
plt.rcParams['font.family'] = 'NanumGothic'; plt.rcParams['axes.unicode_minus'] = False
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from timeline import load, LABELS
out = sys.argv[1]; runs = [('실제 16:54', 'real')] + [tuple(a.split('=', 1)) for a in sys.argv[2:]]
fig, axs = plt.subplots(2, 1, figsize=(13, 8), sharex=True)
w = 0.8 / len(runs); x = np.arange(len(LABELS))
for i, (n, s) in enumerate(runs):
    rows, tot = load(s); d = {r['label']: r for r in rows}
    tv = [d[k]['travel'] if d.get(k, {}).get('travel') is not None else np.nan for k in LABELS]
    ro = [d[k]['rot'][0] if d.get(k, {}).get('rot') else 0 for k in LABELS]
    c = 'k' if s == 'real' else plt.cm.tab10(i - 1)
    axs[0].bar(x + (i - len(runs) / 2 + 0.5) * w, tv, w, color=c, label=f'{n} (완주 {tot:.0f}초)' if tot else f'{n} (미완주)')
    axs[1].bar(x + (i - len(runs) / 2 + 0.5) * w, ro, w, color=c)
axs[0].set_ylabel('이동 시간(초, 앞 점 출발→도착)'); axs[0].set_yscale('symlog', linthresh=10); axs[0].legend(fontsize=8); axs[0].grid(alpha=0.3, axis='y')
axs[0].set_title('구간별 시간 — 트인 구간은 시뮬≈실제, 좁은 곳(정지선1 입구·라바콘·line6)에서만 시뮬이 갇힘')
axs[1].set_ylabel('도착 후 제자리 회전(초)'); axs[1].axhline(10, ls='--', c='r', lw=0.8); axs[1].text(-0.4, 10.3, '실제: 10초 시간초과 4번', color='r', fontsize=8)
axs[1].set_xticks(x); axs[1].set_xticklabels(LABELS, rotation=30); axs[1].grid(alpha=0.3, axis='y')
plt.tight_layout(); plt.savefig(out, dpi=90); print(out)
