#!/usr/bin/env python3
"""걸음새 실측: 발 힘 → 접지(다리별 '높은 힘'의 40 % 넘으면 접지) → 걸음 여부(0.3 s 안 어느 발이든 뜸)·동시에 뜬 발 수·걸음 주기.
리모컨 스틱(ly 전진, rx 회전)과 맞춰 '발을 떼기 시작하는 스틱 크기'와 '발 안 떼고 몸만 도는' 비율, 반응 지연(스틱→회전속도 상호상관)을 잰다."""
import sys, numpy as np
def load(f):
    d = np.load(f); S, W = d['S'], d['W']; return S, W
def analyze(f, label=''):
    S, W = load(f); t = S[:, 0]; L = np.load(f.replace('.npz', '_low.npz'))['L']
    j = np.clip(np.searchsorted(L[:, 0], t), 0, len(L) - 1); F = L[j, 1:5]
    hi = np.array([np.percentile(L[:, 1 + i], 95) for i in range(4)]); con = F > 0.25 * hi[None, :]
    nair = (~con).sum(1)
    # 0.3 s 창 걸음 여부
    dt = np.median(np.diff(t)); w = max(1, int(0.3 / dt)); air_any = (nair > 0).astype(float)
    step = np.convolve(air_any, np.ones(w), 'same') > 0
    # 스틱을 sportmodestate 시각에 보간
    st = {k: np.interp(t, W[:, 0], W[:, i]) for i, k in ((1, 'lx'), (2, 'ly'), (3, 'rx'), (4, 'ry'))}
    yaw_rate = S[:, 7]; vx = S[:, 4]
    moving = step
    # 걸음 주기: 다리0 접지→뜸 전이 간격
    tr = t[1:][(con[:-1, 0]) & (~con[1:, 0])]; per = np.diff(tr); per = per[(per > 0.15) & (per < 1.5)]
    sw = nair[step]
    print(f'\n== {label or f}  ({t[-1] - t[0]:.0f}s, {1 / dt:.0f} Hz)')
    print(f'  걷는 시간 {step.mean() * 100:.0f}% · 걸을 때 동시에 뜬 발 수 분포 0/1/2/3: ' + '/'.join(f'{np.mean(sw == k) * 100:.0f}%' for k in range(4)) + f' · 걸음 주기 중앙 {np.median(per) if len(per) else float("nan"):.2f}s')
    # 제자리 회전: 전진·옆 스틱 거의 0
    pure_turn = (np.abs(st['ly']) < 0.05) & (np.abs(st['lx']) < 0.05)
    print('  제자리 회전 (|ly|,|lx|<0.05)  |rx| 구간 : 시간s · 걸음% · |회전속도| 중앙 rad/s')
    for a, b in ((0.0, 0.02), (0.02, 0.05), (0.05, 0.10), (0.10, 0.15), (0.15, 0.25), (0.25, 0.4), (0.4, 1.01)):
        k = pure_turn & (np.abs(st['rx']) >= a) & (np.abs(st['rx']) < b)
        if k.sum() * dt < 0.3: continue
        print(f'     {a:.2f}~{b:.2f}: {k.sum() * dt:6.1f}s · 걸음 {step[k].mean() * 100:5.1f}% · 회전 {np.median(np.abs(yaw_rate[k])):.3f} (걸음 없을 때 {np.median(np.abs(yaw_rate[k & ~step])) if (k & ~step).sum() > 10 else float("nan"):.3f})')
    pure_fwd = (np.abs(st['rx']) < 0.05) & (np.abs(st['lx']) < 0.05)
    print('  직진 (|rx|,|lx|<0.05)  |ly| 구간 : 시간s · 걸음% · vx 중앙 m/s')
    for a, b in ((0.0, 0.02), (0.02, 0.05), (0.05, 0.10), (0.10, 0.15), (0.15, 0.25), (0.25, 0.4), (0.4, 1.01)):
        k = pure_fwd & (np.abs(st['ly']) >= a) & (np.abs(st['ly']) < b)
        if k.sum() * dt < 0.3: continue
        print(f'     {a:.2f}~{b:.2f}: {k.sum() * dt:6.1f}s · 걸음 {step[k].mean() * 100:5.1f}% · vx {np.median(np.abs(vx[k])):.3f}')
    # 걷는 중 회전 속도와 '걸음 없이 회전' 
    nostep_turn = (~step) & (np.abs(yaw_rate) > 0.05)
    print(f'  발 안 떼고 도는 시간(|회전|>0.05) {nostep_turn.sum() * dt:.1f}s · 걷는 중 회전 중앙 {np.median(np.abs(yaw_rate[step & (np.abs(yaw_rate) > 0.05)])):.3f} rad/s')
    # 지연: 스틱 rx → yaw_rate 상호상관
    for name, x, y in (('회전', -st['rx'], yaw_rate), ('전진', st['ly'], vx)):
        m = np.abs(x) > 0.02
        if m.sum() < 100: continue
        best = max(range(0, int(1.0 / dt)), key=lambda L: np.corrcoef(x[:len(x) - L], y[L:])[0, 1] if L else np.corrcoef(x, y)[0, 1])
        c = np.corrcoef(x[:len(x) - best] if best else x, y[best:])[0, 1]
        g = np.polyfit(x[:len(x) - best][m[:len(x) - best]] if best else x[m], (y[best:])[m[:len(x) - best]] if best else y[m], 1)[0]
        print(f'  {name}: 스틱→실제 지연 {best * dt:.2f}s (상관 {c:.2f}) · 이득 {g:.2f}/스틱1')
for f in sys.argv[1:]: analyze(f)
