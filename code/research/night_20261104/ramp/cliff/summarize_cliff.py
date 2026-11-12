#!/usr/bin/env python3
"""rows_p*.json 합쳐 요약 → summary.json + 표(마크다운) 출력."""
import json, glob, sys, numpy as np
OUT = '/home/ilovehan9283/ws/night_20261104/ramp/cliff'
tags = sys.argv[1:] or ['p0', 'p1', 'p2']
R = []
for t in tags: R += json.load(open(f'{OUT}/rows_{t}.json'))
R.sort(key=lambda r: r['t'])
import os
MODES = os.environ.get('MODES', '거부권,+낭30,+낭50,+낭30b20,+낭50b20').split(',')
V = ['a_X', 'b_XL1', 'c_XD', 'd_XL1D']; M = ['ego'] + [f'{v}|{m}' for v in V for m in MODES]
ZONES = {'전체': lambda r: True, 'X자': lambda r: r['zone'] == 'X자', '평지': lambda r: r['zone'] == '평지', '단위': lambda r: r['plat']}
S = {}
for zn, f in ZONES.items():
    Z = [r for r in R if f(r)]; S[zn] = {'표본': len(Z)}
    if not Z: continue
    for k in M:
        h = np.array([r.get(k + '|헛', 0) for r in Z])
        W = [r for r in Z if r['nwall'] and (k + '|뚫림') in r]
        lost = np.array([r[k + '|뚫림'] for r in W]) if W else np.zeros(1); run = np.array([r[k + '|뚫림연속'] for r in W]) if W else np.zeros(1)
        cov = sum(r['covA'] for r in W)
        C = [r for r in Z if r['ncl'] and (k + '|낭덮40') in r]
        d = dict(헛_표본비율=round(float((h > 0).mean()), 3), 헛칸_평균=round(float(h.mean()), 2),
                 뚫림비율=round(float(lost.sum() / max(cov, 1)), 4), 뚫림_표본비율=round(float((lost > 0).mean()), 3),
                 뚫림_연속최대=int(run.max()), 뚫림_연속3이상_표본=int((run >= 3).sum()), 뚫림_연속30이상_표본=int((run >= 30).sum()))
        if C:
            c4 = np.array([r[k + '|낭덮40'] for r in C]); c2 = np.array([r[k + '|낭덮20'] for r in C])
            d.update(낭표본=len(C), 낭덮40=round(float(c4.mean()), 3), 낭덮20=round(float(c2.mean()), 3), 낭완전덮임40_표본비율=round(float((c4 >= 0.999).mean()), 3))
        C8 = [r for r in Z if r.get('ncl8') and (k + '|낭8덮40') in r]
        if C8:
            c4 = np.array([r[k + '|낭8덮40'] for r in C8]); c2 = np.array([r[k + '|낭8덮20'] for r in C8])
            d.update(낭8표본=len(C8), 낭8덮40=round(float(c4.mean()), 3), 낭8덮20=round(float(c2.mean()), 3), 낭8완전덮임40_표본비율=round(float((c4 >= 0.999).mean()), 3))
        if '+낭' in k:
            d.update(가장자리_평균=round(float(np.mean([r[k + '|가장자리'] for r in Z])), 1), 낭칸_평균=round(float(np.mean([r[k + '|낭칸'] for r in Z])), 1),
                     빈칸_평균=round(float(np.mean([r[k + '|빈칸'] for r in Z])), 1), 낭점이길0_25안_표본비율=round(float(np.mean([r[k + '|낭점_길0.25'] for r in Z])), 3))
        S[zn][k] = d
json.dump(S, open(f'{OUT}/summary.json', 'w'), ensure_ascii=False, indent=1)
print('표본', {z: S[z]['표본'] for z in S}, '단위 낭떠러지 0.5m 안 표본', sum(1 for r in R if r['plat'] and r['ncl']), '0.8m 안', sum(1 for r in R if r['plat'] and r.get('ncl8')))
for zn in ('전체', 'X자', '단위', '평지'):
    if zn not in S or S[zn]['표본'] == 0: continue
    print(f'\n### {zn} (표본 {S[zn]["표본"]})')
    print('| 방법 | 헛장애물 표본% | 헛칸 평균 | 벽 뚫림 비율 | 뚫림 표본% | 뚫림 연속최대 | 연속≥3 표본 | 연속≥30 표본 | 낭떠러지≤0.5m 덮임(0.40) | 덮임(0.20) | 완전덮임 표본% | 낭떠러지≤0.8m 덮임(0.40) | 덮임(0.20) | 낭칸/빈칸(1.5m) | 낭점 길 0.25m 안 표본% |')
    print('|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|')
    for k in M:
        d = S[zn][k]
        nd = f"{d['낭덮40']:.3f}" if '낭덮40' in d else '-'; n2 = f"{d['낭덮20']:.3f}" if '낭덮20' in d else '-'
        nf = f"{100 * d['낭완전덮임40_표본비율']:.1f}" if '낭완전덮임40_표본비율' in d else '-'
        e4 = f"{d['낭8덮40']:.3f}" if '낭8덮40' in d else '-'; e2 = f"{d['낭8덮20']:.3f}" if '낭8덮20' in d else '-'
        nc = f"{d['낭칸_평균']}/{d['빈칸_평균']}" if '낭칸_평균' in d else '-'; npth = f"{100 * d['낭점이길0_25안_표본비율']:.1f}" if '낭점이길0_25안_표본비율' in d else '-'
        print(f"| {k.replace('|', ' · ')} | {100 * d['헛_표본비율']:.1f} | {d['헛칸_평균']} | {100 * d['뚫림비율']:.2f}% | {100 * d['뚫림_표본비율']:.1f} | {d['뚫림_연속최대']} | "
              f"{d['뚫림_연속3이상_표본']} | {d['뚫림_연속30이상_표본']} | {nd} | {n2} | {nf} | {e4} | {e2} | {nc} | {npth} |")
