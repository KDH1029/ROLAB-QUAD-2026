#!/usr/bin/env python3
"""판정 보정 — 16:50 녹화(사람이 리모컨, 사용자 확인: 콘 '2개 정도' 건드림)가 '닿음 2번'이 되는 문턱 δ 를 찾고, 같은 δ 로 로봇 시뮬(mc_G3<태그>_*.json 의 tg)을 다시 센다.
닿음 = 부호 있는 여유 g < −δ (지도 콘 표면 두께·몸 상자 과대를 δ 만큼 봐줌). 한 번 = 이어진 구간(0.5 s 안 끊김은 합침). 같은 콘 = 위치 0.35 m 안.
사용(ROS Jazzy 환경): python3 calib.py [태그...]"""
import json, math, os, sys
H = os.path.dirname(os.path.abspath(__file__)); TAGS = sys.argv[1:] or ['_cnow', '_cjun', '_cjbest', '_cnowbest']; sys.argv = [sys.argv[0], '6', '26']
src = open(os.path.join(H, 'best_line.py')).read(); exec(compile(src[:src.index('free = np.where')], 'h', 'exec'))


def episodes(t, x, y, g, dl):
    hit = g < -dl; ev = []
    for i in np.where(hit)[0]:
        if ev and t[i] - ev[-1]['t1'] < 0.5:
            ev[-1]['t1'] = t[i]; ev[-1]['pts'].append((x[i], y[i])); ev[-1]['gmin'] = min(ev[-1]['gmin'], g[i])
        else:
            ev.append(dict(t0=t[i], t1=t[i], pts=[(x[i], y[i])], gmin=g[i]))
    for e in ev:
        e['c'] = np.mean(e['pts'], 0)
    places = []
    for e in ev:
        if not any(np.hypot(*(e['c'] - p)) < 0.35 for p in places):
            places.append(e['c'])
    dt = np.r_[np.diff(t), 0.05]; dt = np.minimum(dt, 0.2)
    return len(ev), len(places), float(np.sum(dt[hit])), ev


od = d['odom']; od = od[np.argsort(od[:, 1])]; x, y = od[:, 2], od[:, 3]; q = od[:, 5:9]
yaw = np.arctan2(2 * (q[:, 3] * q[:, 2] + q[:, 0] * q[:, 1]), 1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2)); t = od[:, 1] - od[0, 1]
z = (x > ZLO[0]) & (x < ZHI[0]) & (y > ZLO[1]) & (y < ZHI[1]); t, x, y, yaw = t[z], x[z], y[z], yaw[z]
g = np.array([sgap(np.array([a, b]), c) for a, b, c in zip(x, y, yaw)])
print('16:50 사람 주행(슬라럼 안 %.0f초) — δ(봐주는 폭)별 닿음:' % (t[-1] - t[0]))
DL = np.arange(0, 0.161, 0.01); fit = []
for dl in DL:
    n, npl, sec, ev = episodes(t, x, y, g, dl)
    print(f'  δ {dl * 100:4.0f} cm: 닿은 번 {n:3d} · 다른 자리 {npl:2d} · 닿은 시간 {sec:5.1f}초' + ('  ← 사용자 "2개 정도"' if npl == 2 else ''))
    if npl == 2:
        fit.append(dl)
if not fit:
    fit = [DL[int(np.argmin([abs(episodes(t, x, y, g, dl)[1] - 2) for dl in DL]))]]
lo_, hi_ = min(fit), max(fit)
print(f'→ 보정 δ = {lo_ * 100:.0f}~{hi_ * 100:.0f} cm (사람 주행이 2자리에서 닿는 범위)')
for dl in sorted({lo_, hi_}):
    ev = episodes(t, x, y, g, dl)[3]
    print(f'  δ {dl * 100:.0f} cm 일 때 사람 주행이 닿은 자리: ' + ' · '.join(f'({e["c"][0]:.2f},{e["c"][1]:.2f}) {e["t0"]:.0f}초 {(e["t1"] - e["t0"]) + 0.05:.1f}s 최대 {-e["gmin"] * 100:.0f} cm' for e in ev))
json.dump(dict(delta_lo=lo_, delta_hi=hi_), open(os.path.join(H, 'calib_delta.json'), 'w'))
LAB = {'_cnow': 'junk 지금(nail-1105) · 원래 선', '_cjun': 'juninsu(11-06) · 원래 선', '_cjbest': 'juninsu · 넓힌 선', '_cnowbest': 'junk 지금 · 넓힌 선'}
print('\n로봇 시뮬(24판, 같은 판정·같은 δ):')
for tag in TAGS:
    fs = [os.path.join(H, f'mc_G3{tag}_{s}.json') for s in (0, 12)]
    if not all(os.path.exists(f) for f in fs):
        print(f'  {tag}: 아직 없음'); continue
    runs = json.load(open(fs[0])) + json.load(open(fs[1]))
    for dl in sorted({lo_, hi_}):
        R = []
        for r in runs:
            A = np.array(r['tg']); n, npl, sec, _ = episodes(A[:, 0], A[:, 1], A[:, 2], A[:, 3], dl); R.append((n, npl, sec))
        R = np.array(R)
        print(f'  {LAB.get(tag, tag)} · δ {dl * 100:.0f} cm: 닿는 자리 평균 {R[:, 1].mean():.1f} (중앙 {np.median(R[:, 1]):.0f}, 최대 {R[:, 1].max():.0f}) · 닿은 시간 평균 {R[:, 2].mean():.1f}초 (감점 약 {R[:, 2].mean() * 10:.0f}점) · 0번인 판 {int(np.sum(R[:, 1] == 0))}/{len(R)}')
