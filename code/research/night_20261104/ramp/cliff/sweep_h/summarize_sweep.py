#!/usr/bin/env python3
"""rows_s*.json 합쳐 갈래별 요약 → sweep_summary.json + sweep_table.md"""
import json, sys, numpy as np
OUT = '/home/ilovehan9283/ws/night_20261104/ramp/cliff/sweep_h'
tags = sys.argv[1:] or ['s0', 's1', 's2']
R = []
for t in tags: R += json.load(open(f'{OUT}/rows_{t}.json'))
R.sort(key=lambda r: r['t'])
V = ['ego', '거부권'] + [f'd{d}|h{h}|{di}|c{c}' for d in (22, 26, 30) for h in (0, 1) for di in ('all', 'nofront') for c in (1, 2)]
ZONES = {'전체': lambda r: True, 'X자': lambda r: r['zone'] == 'X자', '단위': lambda r: r['plat'], '평지': lambda r: r['zone'] == '평지'}
S = {}
for zn, f in ZONES.items():
    Z = [r for r in R if f(r)]; S[zn] = {'표본': len(Z)}
    for k in V:
        h = np.array([r.get(k + '|헛', 0) for r in Z])
        W = [r for r in Z if r['nwall'] and (k + '|뚫림') in r]
        lost = np.array([r[k + '|뚫림'] for r in W]) if W else np.zeros(1); run = np.array([r[k + '|뚫림연속'] for r in W]) if W else np.zeros(1)
        cov = sum(r['covA'] for r in W)
        d = dict(헛=round(100 * float((h > 0).mean()), 2), 헛칸=round(float(h.mean()), 3), 뚫림=round(100 * float(lost.sum() / max(cov, 1)), 3),
                 연속최대=int(run.max()), 연속3=int((run >= 3).sum()), 연속30=int((run >= 30).sum()))
        for m in ('낭덮20', '낭덮40', '낭앞덮20', '낭옆덮20'):
            C = [r[k + '|' + m] for r in Z if (k + '|' + m) in r]
            if C: d[m] = round(float(np.mean(C)), 3); d[m + '_n'] = len(C)
        S[zn][k] = d
json.dump(S, open(f'{OUT}/sweep_summary.json', 'w'), ensure_ascii=False, indent=1)
L = [f"표본(0.5 s): 전체 {S['전체']['표본']} / X자 {S['X자']['표본']} / 단 위 {S['단위']['표본']} / 평지 {S['평지']['표본']}  (10-30 녹화 0~300 s, 격자 XT16+L1, 자세=지도 궤적)", '',
     '| 갈래 (drop·높은곳만·방향·연속) | 헛% 전체 | X자 | 단 위 | 뚫림% 전체 | X자 | 단 위 | 연속최대 전체 | 단 위 | 연속≥30 전체 | 단 위 | 가장자리 덮임(0.20) 단 0.5m | 앞 | 옆 |',
     '|---|---|---|---|---|---|---|---|---|---|---|---|---|---|']
for k in V:
    a, x, p = S['전체'][k], S['X자'][k], S['단위'][k]
    nm = k if k in ('ego', '거부권') else k.replace('|', ' · ')
    L.append(f"| {nm} | {a['헛']:.1f} | {x['헛']:.1f} | {p['헛']:.1f} | {a['뚫림']:.2f} | {x['뚫림']:.2f} | {p['뚫림']:.2f} | {a['연속최대']} | {p['연속최대']} | {a['연속30']} | {p['연속30']} | "
             f"{a.get('낭덮20', float('nan')):.3f} | {a.get('낭앞덮20', float('nan')):.3f} | {a.get('낭옆덮20', float('nan')):.3f} |")
open(f'{OUT}/sweep_table.md', 'w').write('\n'.join(L) + '\n'); print('\n'.join(L))
