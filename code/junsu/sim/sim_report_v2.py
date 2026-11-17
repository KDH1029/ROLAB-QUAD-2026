#!/usr/bin/env python3
"""v2 시뮬 결과 요약 — world.jsonl(0.1초 자세·몸 여유·러너 상태) + 노드 로그.
접촉: 몸 0.70×0.31 이 지도 장애물(외톨이 뺌)에 파고든 깊이 > δ. δ 0 = 엄격, δ 10 cm = 16:50 사람 주행(콘 2개 건드림)으로 맞춘 기준.
머뭇: 러너가 TO_GOAL(가는 중)인데 속도 < 0.03 m/s 인 시간(일부러 서는 정지선·주차·회전 제외). 사용: sim_report_v2.py <결과 폴더>"""
import json, os, re, sys
import numpy as np
O = sys.argv[1]
E = [json.loads(l) for l in open(os.path.join(O, 'world.jsonl')) if l.strip()]
P = [e for e in E if e['ev'] == 'pose']
t = np.array([e['t'] for e in P]); g = np.array([e['gap'] for e in P]); v = np.array([e['v'] for e in P]); rs = [e.get('rs', '') for e in P]
x = np.array([e['x'] for e in P]); y = np.array([e['y'] for e in P])
RJ = json.load(open(os.path.join(O, 'route.json'))); R = RJ['route']
# ★ 11-06 slalom 점검: R[6:25] 는 옛 경로(rec1650 #7~#25) 번호 — hybrid 경로에선 line3~exit(코스 대부분)을 '슬라럼'으로 셌다. slalom_path → rabain~rabaout → 옛 번호 순
_lab = [e['label'] for e in R]
if RJ.get('slalom_path'):
    SL = np.array(RJ['slalom_path'], float)
elif 'rabain' in _lab and 'rabaout' in _lab:
    SL = np.array([[e['x'], e['y']] for e in R[_lab.index('rabain'):_lab.index('rabaout') + 1]])
else:
    SL = np.array([[e['x'], e['y']] for e in R[6:25]] or [[1e9, 1e9]])
lo, hi = SL.min(0) - 0.4, SL.max(0) + 0.4
inz = (x > lo[0]) & (x < hi[0]) & (y > lo[1]) & (y < hi[1])
st = [i for i, s in enumerate(rs) if s and s not in ('START_CHECK', '')]
t_start = t[st[0]] if st else t[0]
done = [i for i, s in enumerate(rs) if s == 'DONE']; t_end = t[done[0]] if done else t[-1]
run = (t >= t_start) & (t <= t_end)


def episodes(mask):
    n, prev, last_t = 0, False, -9
    for ti, m in zip(t, mask):
        if m and not prev and ti - last_t > 0.5:
            n += 1
        if m:
            last_t = ti
        prev = m
    return n


lines = []
lines.append(f"완주 {'예' if done else '아니오(' + (rs[-1] or '?') + ')'} · 주행 시간 {t_end - t_start:.1f}초 · 슬라럼 구간 {np.sum(inz & run) * 0.1:.1f}초")
for dl in (0.0, 0.10):
    m = run & (g < -dl)
    lines.append(f"  접촉(δ {dl * 100:.0f} cm): 전체 {np.sum(m) * 0.1:.1f}초 {episodes(m)}번 · 슬라럼 {np.sum(m & inz) * 0.1:.1f}초 {episodes(m & inz)}번 · 가장 깊이 {max(0, -g[run].min()) * 100:.0f} cm")
hes = run & (v < 0.03) & np.array([s == 'TO_GOAL' for s in rs])
lines.append(f"  머뭇(가는 중인데 멈춤): {np.sum(hes) * 0.1:.1f}초 · 회전(ROTATE) {sum(1 for s, r in zip(rs, run) if r and s == 'ROTATE') * 0.1:.1f}초 · 콘 무시 직진(ROUTE_FOLLOW) {sum(1 for s, r in zip(rs, run) if r and s == 'ROUTE_FOLLOW') * 0.1:.1f}초 · 물러남(BACKOFF) {sum(1 for s, r in zip(rs, run) if r and s == 'BACKOFF') * 0.1:.1f}초")
lines.append(f"  발 안 뗌(문턱 아래 명령) {P[-1]['lean']:.1f}초 · 기본 보행 회전 기울었다 튐 {P[-1]['hop']:.1f}초")
rl = open(os.path.join(O, 'runner.log')).read() if os.path.exists(os.path.join(O, 'runner.log')) else ''
el = open(os.path.join(O, 'ego.log')).read() if os.path.exists(os.path.join(O, 'ego.log')) else ''
gl = open(os.path.join(O, 'guard.log')).read() if os.path.exists(os.path.join(O, 'guard.log')) else ''
lines.append(f"  러너: 목표 보정 {rl.count('목표 보정')} · 회전 없이 통과 {rl.count('회전 없이 통과')} · 회전 타임아웃 {rl.count('회전 타임아웃')} · 콘 무시 {rl.count('콘 무시')} · 막힘 {rl.count('막힘')}"
             f" · ego: 경로 못 찾음 {el.count('Failed to find')} · 제자리 대기(holding) {el.count('holding')} · 안전필터 멈춤/바꿈 {gl.count('stop')}/{gl.count('changed')}")
seg = []
for m_ in re.finditer(r'\[(\w+)\] (도착|경로 따라 도착)', rl):
    seg.append(m_.group(1))
lines.append(f"  도착한 점 {len(set(seg))}개 / 경로 {len(R) - 1}개")
print('\n'.join(lines))
