#!/usr/bin/env python3
"""조건별 입구 통과·완주 표(markdown)."""
import json, os
R = os.path.expanduser('~/ws/junsu_v2/sim/runs_v2')
SETS = [
    ('v2 · 외톨이 칸 있음(예전 시뮬) · guard off', ['H1_hybrid_det', 'H2_hybrid_nodet', 'H3_hybrid_nocarrot_det', 'H4_hybrid_nocarrot_nodet', 'R1_fix_carrot_det', 'R2_fix_carrot_nodet', 'R3_fix_nocarrot_det', 'R4_fix_nocarrot_nodet', 'xentry_base1']),
    ('main(16:54) · 외톨이 칸 있음', ['M_det', 'M_nodet', 'B4_main_det_s1']),
    ('v2 · 외톨이 칸 있음 · 되짚기 끔(RUNNER_OFF_ROUTE_M=99)', ['xentry_ghost_noretrace_1', 'xentry_ghost_noretrace_2']),
    ('**v2 · 외톨이 칸 없음(고친 시뮬) · guard off**', ['xentry_noiso_1', 'xentry_noiso_2', 'xentry_noiso_3']),
    ('v2 · 외톨이 칸 없음 · guard **on**', ['xentry_noiso_guard_1', 'xentry_noiso_guard_2']),
    ('v2 · 외톨이 칸 없음 · 콘 검출 없음(CONE=walls)', ['xentry_noiso_nodet_1']),
    ('main · 외톨이 칸 없음', ['xentry_main_noiso_1']),
]
print('| 조건 | 판 | 북쪽 통로로 빠짐 | X 통과(첫 출발 뒤 초) | 완주(주행 시간) | TMAX |')
print('|---|---|---|---|---|---|')
for name, runs in SETS:
    runs = [r for r in runs if os.path.exists(os.path.join(R, r, 'world.jsonl'))]
    if not runs: continue
    north = 0; xs = []; done = []; tm = []
    for r in runs:
        P = [json.loads(l) for l in open(os.path.join(R, r, 'world.jsonl')) if l.strip()]; P = [p for p in P if p['ev'] == 'pose']
        go = next((p['t'] for p in P if abs(p['v']) > 0.02), 13.0)
        tx = next((p['t'] for p in P if p['x'] > 2.3 and p['y'] > 0.4), None)
        tn = next((p['t'] for p in P if 0.5 < p['x'] < 1.35 and p['y'] > 0.30 and p['t'] < (tx or 1e9)), None)
        north += tn is not None; xs.append('—' if tx is None else f'{tx - go:.0f}')
        rep = open(os.path.join(R, r, 'report.txt')).readline() if os.path.exists(os.path.join(R, r, 'report.txt')) else ''
        done.append(rep.split('주행 시간')[1].split('초')[0].strip() + '초' if rep.startswith('완주 예') else '✗(' + (rep.split('(')[1].split(')')[0] if '(' in rep else '?') + ')')
        tm.append(round(P[-1]['t']))
    nd = sum(d[0] != '✗' for d in done)
    print(f"| {name} | {len(runs)} | {north}/{len(runs)} | {', '.join(xs)} | {nd}/{len(runs)}: {', '.join(done)} | {max(tm)}s |")
