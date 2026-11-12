#!/usr/bin/env python3
"""각 시뮬 판에서 X 통로 입구 행동 분류: 북쪽 막다른 통로(x 0.6~1.35, y>0.25) 진입 여부·시각, 정지선1 도달 시각."""
import json, sys, os, glob
R = os.path.expanduser('~/ws/junsu_v2/sim/runs_v2')
def scan(d):
    P = []
    for l in open(os.path.join(d, 'world.jsonl')):
        try: e = json.loads(l)
        except Exception: continue
        if e.get('ev') == 'pose': P.append(e)
    if not P: return None
    t_go = next((p['t'] for p in P if abs(p['v']) > 0.02), None)
    t_north = next((p['t'] for p in P if 0.5 < p['x'] < 1.35 and p['y'] > 0.30), None)
    t_sl1 = next((p['t'] for p in P if (p['x']-3.93)**2 + (p['y']-1.62)**2 < 0.5**2), None)
    t_x2 = next((p['t'] for p in P if p['x'] > 2.3 and p['y'] > 0.4), None)
    maxy_north = max([p['y'] for p in P if 0.5 < p['x'] < 1.35] + [-9])
    last = P[-1]
    return dict(go=t_go, north=t_north, maxyN=round(maxy_north, 2), x2=t_x2, sl1=t_sl1, last=(round(last['x'], 2), round(last['y'], 2), last['rs']), T=last['t'])
names = sys.argv[1:] or sorted(os.listdir(R))
for n in names:
    d = os.path.join(R, n) if not os.path.isdir(n) else n
    if not os.path.exists(os.path.join(d, 'world.jsonl')): continue
    r = scan(d)
    if not r: continue
    rep = open(os.path.join(d, 'report.txt')).readline().strip()[:30] if os.path.exists(os.path.join(d, 'report.txt')) else ''
    route = json.load(open(os.path.join(d, 'route.json')))['route']
    l1 = route[1]
    print(f"{os.path.basename(d):26s} l1=({l1['x']:.2f},{l1['y']:.2f}) {route[2]['label']:9s} go={r['go']} 북진입={r['north']} maxyN={r['maxyN']} X통과={r['x2']} SL1={r['sl1']} 끝={r['last']} | {rep}")
