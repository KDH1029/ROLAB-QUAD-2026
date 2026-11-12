#!/usr/bin/env python3
"""신호 규칙 v3(11-06 사용자): 2초 정지 → 초록 1초 연속이면 출발 / 10초 지나면 '빨강이 안 보일 때' 출발(빨강 보이는 동안은 끝없이 대기).
'빨강 안 보임' = 마지막 RED 판정 뒤 RED_RECENT 초 동안 RED 없음. 이 값을 녹화 판정 시간표로 정한다.
15:09 녹화: 빨강 고정 구간(진짜 빨강) 5~82 s — 이 안에서 타임아웃 쪽으로 출발하면 '빨강인데 출발'(오출발)."""
import json, numpy as np
D = '/home/ilovehan9283/ws/junsu/onboard_vision/results/tl_timeline/'
def run(tl, t0, hold=2.0, tmo=10.0, recent=2.0, confirm=1.0):
    tw = t0 + hold; gs = None; red_t = -1e9; prev = 'UNKNOWN'
    for t, s, *_ in tl:
        if t < tw: continue
        if s == 'GREEN':
            if prev != 'GREEN': gs = t
        else: gs = None
        if s == 'RED': red_t = t
        prev = s
        if gs is not None and t - gs >= confirm: return t - t0, 'green', t
        if t - tw >= tmo and t - red_t > recent: return t - t0, 'notred', t
    return None, 'end', None
tl9 = json.load(open(D + 'arena_150952_10hz.json')); tl4 = json.load(open(D + 'arena1459_10hz.json'))
# 빨강 판정 끊김(연속 RED 사이 빈 시간) 분포 — 빨강 고정 구간
t = np.array([r[0] for r in tl9]); s = np.array([r[1] for r in tl9]); rt = t[(s == 'RED') & (t > 5) & (t < 82)]; gaps = np.diff(rt)
print(f'15:09 빨강 고정 5~82 s: RED 판정 {len(rt)}개, 판정 사이 빈틈 최대 {gaps.max():.1f}s · 0.5s 넘는 빈틈 {np.sum(gaps > 0.5)}번 · 1s 넘는 {np.sum(gaps > 1)} · 2s 넘는 {np.sum(gaps > 2)} · 3s 넘는 {np.sum(gaps > 3)}')
print('\nRED_RECENT | 15:09 빨강 중 도착(5~70 s) → 빨강 끝(82 s) 전에 출발(오출발) | 평균 출발 | 14:59 정상 주기 출발 평균/최대')
for rec in (0.5, 1.0, 1.5, 2.0, 3.0):
    res = [run(tl9, t0, recent=rec) for t0 in np.arange(5.0, 70.0, 0.1)]
    early = [r for r in res if r[2] is not None and r[2] < 82 and r[1] == 'notred']; fakeg = [r for r in res if r[2] is not None and r[2] < 82 and r[1] == 'green']
    dts = np.array([r[0] for r in res if r[0] is not None])
    r4 = [run(tl4, t0, recent=rec) for a, b in ((50, 58), (135, 146)) for t0 in np.arange(a, b, 0.1)]; d4 = np.array([r[0] for r in r4 if r[0] is not None])
    print(f'   {rec:.1f} s   | 오출발(빨강 안 보임 판정) {len(early) / len(res) * 100:5.1f}% · 가짜 초록 출발 {len(fakeg) / len(res) * 100:4.1f}% (깊이 필터 전 판정) | {dts.mean():5.1f}s | {d4.mean():.1f}/{d4.max():.1f}s')
