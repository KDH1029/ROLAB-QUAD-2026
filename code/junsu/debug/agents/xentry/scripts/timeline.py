#!/usr/bin/env python3
"""입구 구간 시각 순서(세계 t 기준): 첫 목표 바뀜 → ego 실패 → 북쪽 진입 → 되짚기·물러나기·경로 따라가기 → X 통과."""
import json, os, re, sys
R = os.path.expanduser('~/ws/junsu_v2/sim/runs_v2')
def ts(l):
    m = re.search(r'\[(\d+\.\d+)\]', l); return float(m.group(1)) if m else None
for n in sys.argv[1:]:
    d = os.path.join(R, n); E = [json.loads(l) for l in open(d + '/world.jsonl') if l.strip()]
    P = [e for e in E if e['ev'] == 'pose']; sp = [e for e in E if e['ev'] == 'sport' and e.get('api') == 2049]
    RL = open(d + '/runner.log').read().splitlines(); EL = open(d + '/ego.log').read().splitlines()
    cw = next((ts(l) for l in RL if 'CLASSICWALK' in l), None)
    if cw is None or not sp: print(n, '동기 실패'); continue
    off = cw - sp[0]['t']
    tx = next((p['t'] for p in P if p['x'] > 2.3 and p['y'] > 0.4), None); tend = (tx or P[-1]['t'])
    tn = next((p['t'] for p in P if 0.5 < p['x'] < 1.35 and p['y'] > 0.30 and p['t'] < tend), None)
    arr1 = next((ts(l) - off for l in RL if '[line1] 도착' in l), None)
    nofree = [ts(l) - off for l in EL if 'No collision-free' in l and ts(l) and arr1 and arr1 - 0.5 < ts(l) - off < tend]
    hold = [ts(l) - off for l in EL if 'holding' in l and ts(l) and arr1 and arr1 - 0.5 < ts(l) - off < tend]
    ev = lambda k: [round(ts(l) - off, 1) for l in RL if k in l and ts(l) and ts(l) - off < tend + 1]
    print(f"{n:24s} line1도착 {arr1 and round(arr1,1)} | ego 실패 {len(nofree)}번(첫 {nofree and round(nofree[0],1)}) holding {len(hold)} | 북쪽진입 {tn} | "
          f"되짚기 {len(ev('지나온 길 되짚어'))} 물러남 {len(ev('물러남('))} 경로따라가기 {ev('경로 따라가기(ego 세움')} | X통과 {tx}")
