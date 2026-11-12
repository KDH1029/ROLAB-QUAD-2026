#!/usr/bin/env python3
"""정지선 신호 대기 시뮬 — 경기장 녹화에 YOLO 를 돌린 10 Hz 판정 시간표(junsu tl_timeline)를 러너 판정식에 그대로 넣는다.
도착 시각을 0.5초 간격으로 훑어서, 출발까지 걸린 시간과 출발 이유(초록 확정/타임아웃)를 센다.
러너 식(real_course_runner.py STOPLINE_HOLD → STOPLINE_WAIT_SIGNAL)과 같다:
  초록 연속 >= CONFIRM → 출발 / (대기 >= TIMEOUT 이고 최근 RED_RECENT 안에 빨강 없음) 또는 대기 >= RED_MAX → 출발"""
import json, sys, numpy as np
D = '/home/ilovehan9283/ws/junsu/onboard_vision/results/tl_timeline/'
CONFIRM, RED_RECENT = 1.0, 2.0
SETS = {'이전(3/15/40)': (3.0, 15.0, 40.0), '새(2/10/10)': (2.0, 10.0, 10.0)}
def run(tl, t0, hold, tmo, redmax):
    t_wait = t0 + hold; green_since = None; red_t = -1e9; prev = 'UNKNOWN'
    for t, s, *_ in tl:
        if t < t_wait:            # 카메라 꺼져 있음(정지 중)
            continue
        if s == 'GREEN':
            if prev != 'GREEN': green_since = t
        else:
            green_since = None
        if s == 'RED': red_t = t
        prev = s
        if s == 'GREEN' and green_since is not None and t - green_since >= CONFIRM:
            return t - t0, 'green'
        w = t - t_wait
        if (w >= tmo and t - red_t > RED_RECENT) or w >= redmax:
            return t - t0, 'timeout_red' if t - red_t <= RED_RECENT else 'timeout'
    return None, 'end'
for f, note in (('arena1459_10hz.json', '14:59 정상 5초 주기'), ('arena_150952_10hz.json', '15:09 빨강 고정 63초 + 전시장 가짜 초록')):
    tl = json.load(open(D + f)); T = tl[-1][0]
    st = {k: 0 for k in 'RG'}
    for _, s, *_ in tl: st['R'] += s == 'RED'; st['G'] += s == 'GREEN'
    print(f'\n== {f} ({note}) 길이 {T:.0f}s, RED {st["R"]} GREEN {st["G"]} 프레임')
    for name, (h, tm, rm) in SETS.items():
        res = [run(tl, t0, h, tm, rm) for t0 in np.arange(0, T - 45, 0.5)]
        res = [r for r in res if r[0] is not None]
        dt = np.array([r[0] for r in res]); why = [r[1] for r in res]
        n = len(res)
        print(f'  {name:12s} 도착 {n}회 | 출발까지 평균 {dt.mean():5.1f}s 중앙 {np.median(dt):5.1f}s 최대 {dt.max():5.1f}s | '
              f'초록확정 {why.count("green")/n*100:5.1f}% 타임아웃 {why.count("timeout")/n*100:5.1f}% 빨강중타임아웃 {why.count("timeout_red")/n*100:5.1f}%')

# ── 신호등이 실제로 보인 구간에 도착한 경우만(로봇이 정지선에 서 있던 때) ──
WIN = {'arena1459_10hz.json': [(50, 70), (135, 158)], 'arena_150952_10hz.json': [(5, 122)]}
print('\n\n### 신호등이 보이는 구간 안에서 도착(도착 후 12초가 구간 안에 들어가는 경우만)')
for f, wins in WIN.items():
    tl = json.load(open(D + f))
    for name, (h, tm, rm) in SETS.items():
        res = []
        for a, b in wins:
            for t0 in np.arange(a, b - 12.0 + 1e-6, 0.1):
                sub = [r for r in tl if r[0] <= b + 40]
                res.append(run(sub, t0, h, tm, rm))
        res = [r for r in res if r[0] is not None]
        dt = np.array([r[0] for r in res]); why = [r[1] for r in res]; n = len(res)
        print(f'  {f[:14]:14s} {name:12s} 도착 {n:4d}회 | 출발까지 평균 {dt.mean():5.1f}s 최대 {dt.max():5.1f}s | '
              f'초록확정 {why.count("green")/n*100:5.1f}% 타임아웃(빨강X) {why.count("timeout")/n*100:5.1f}% 타임아웃(빨강 보는 중) {why.count("timeout_red")/n*100:5.1f}%')
