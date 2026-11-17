#!/usr/bin/env python3
"""슬라럼에서 '가장 넓게 비켜 가는 선'이 얼마나 가능한가 — 그린 선(러너 곡선)을 옆으로만 밀어(점마다 ±0.35 m) 몸 0.70×0.31 과 콘·벽 사이 여유를 최대로.
장애물 = sim_scurve 와 같은 편집기 벽 칸에서 외톨이(0.12 m 안 이웃 2개 미만 = #12 옆 잡음 같은 것) 뺌. 몸 방향 = 경로 접선(0.25 m 앞) 또는 고정.
결과: best_line_<모드>.npy(곡선), best_line.png, 후보 경로점 debug/slalom/route_slalom_best.json(원본 route.json 은 안 건드림)
사용(ROS Jazzy 환경): python3 best_line.py"""
import json, math, os, sys
H = os.path.dirname(os.path.abspath(__file__)); sys.argv = [sys.argv[0], '6', '26']
src = open(os.path.join(H, 'sim_scurve.py')).read(); exec(compile(src[:src.index('def body_hits')], 'h', 'exec'))
from scipy.optimize import minimize
TR0 = cKDTree(OB); nb = np.array([len(TR0.query_ball_point(q, 0.12)) - 1 for q in OB]); iso = nb < 2
OBc = OB[~iso]; TO2 = cKDTree(OBc)
SL = np.array([[e['x'], e['y']] for e in R[6:25]]); ZLO, ZHI = SL.min(0) - 0.4, SL.max(0) + 0.4
print(f'장애물 칸 {len(OB)} · 외톨이 뺌 {iso.sum()} ({", ".join(f"({a:.2f},{b:.2f})" for a, b in OB[iso & np.all((OB > ZLO) & (OB < ZHI), 1)])})')
C = np.load(os.path.join(H, 'decompose_curve.npy'))[:, :2]
S = np.r_[0, np.cumsum(np.hypot(*np.diff(C, axis=0).T))]
st = np.arange(0, S[-1], 0.05); B = np.c_[np.interp(st, S, C[:, 0]), np.interp(st, S, C[:, 1])]
tg = np.gradient(B, axis=0); tg /= np.linalg.norm(tg, axis=1)[:, None]; Nn = np.c_[-tg[:, 1], tg[:, 0]]
inz = (B[:, 0] > ZLO[0]) & (B[:, 0] < ZHI[0]) & (B[:, 1] > ZLO[1]) & (B[:, 1] < ZHI[1])
L2, W2 = 0.35, 0.155


def sgap(P, a):
    """몸 사각형과 장애물 최소 '부호 있는' 거리(안으로 파고들면 음수)."""
    idx = TO2.query_ball_point(P, 0.55)
    if not idx:
        return 0.5
    q = OBc[idx] - P; lx = np.abs(q[:, 0] * math.cos(a) + q[:, 1] * math.sin(a)); ly = np.abs(-q[:, 0] * math.sin(a) + q[:, 1] * math.cos(a))
    out = np.hypot(np.maximum(lx - L2, 0), np.maximum(ly - W2, 0)); inside = (lx < L2) & (ly < W2)
    g = np.where(inside, -np.minimum(L2 - lx, W2 - ly), out); return float(g.min())


def heads(P, mode):
    if mode == 'fixed':
        a = math.atan2(*(P[inz][-1] - P[inz][0])[::-1]); return np.full(len(P), a)
    j = np.minimum(np.arange(len(P)) + 5, len(P) - 1); d = P[j] - P; d[-5:] = P[-1] - P[-6]
    return np.arctan2(d[:, 1], d[:, 0])


def evalp(P, mode):
    h = heads(P, mode); return np.array([sgap(P[i], h[i]) for i in range(len(P))])


free = np.where(inz)[0]; free = free[(free > 3) & (free < len(B) - 4)]
MARGIN = 0.10


def cost(o_free, mode):
    o = np.zeros(len(B)); o[free] = o_free; P = B + o[:, None] * Nn; g = evalp(P, mode)
    pen = np.sum(np.logaddexp(0, (MARGIN - g[inz]) / 0.01) * 0.01) * 40
    kap = np.diff(o, 2) / 0.05 ** 2; smooth = np.sum(np.maximum(np.abs(kap) - 0.5, 0) ** 2) * 0.002 + np.sum(np.diff(o) ** 2) * 2
    return pen + smooth


res = {}
for mode in ('tangent', 'fixed'):
    g0 = evalp(B, mode)
    if os.environ.get('PLOT_ONLY'):
        A_ = np.load(os.path.join(H, f'best_line_{mode}.npy')); res[mode] = (A_[:, :2], A_[:, 2], A_[:, 3]); continue
    r = minimize(cost, np.zeros(len(free)), args=(mode,), method='Powell', bounds=[(-0.35, 0.35)] * len(free), options=dict(maxiter=6000, xtol=1e-3, ftol=1e-4))
    o = np.zeros(len(B)); o[free] = r.x; P = B + o[:, None] * Nn; g = evalp(P, mode)
    res[mode] = (P, g, o)
    ov0, ov = np.mean(g0[inz] < 0) * 100, np.mean(g[inz] < 0) * 100
    print(f'[{mode}] 그린 선: 몸 겹침 {ov0:.0f}% · 최소 여유 {g0[inz].min() * 100:+.1f} cm → 최적 선: 몸 겹침 {ov:.0f}% · 최소 여유 {g[inz].min() * 100:+.1f} cm · p10 {np.percentile(g[inz], 10) * 100:+.1f} cm · 옆으로 민 최대 {np.abs(o).max() * 100:.0f} cm')
    np.save(os.path.join(H, f'best_line_{mode}.npy'), np.c_[P, g, o])
# 그림
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
plt.rcParams['font.family'] = 'NanumGothic'; plt.rcParams['axes.unicode_minus'] = False
fig, ax = plt.subplots(1, 2, figsize=(13, 9))
for a_, mode in zip(ax, ('tangent', 'fixed')):
    P, g, o = res[mode]; zz = np.all((OB > ZLO - 0.3) & (OB < ZHI + 0.3), 1)
    a_.scatter(OB[zz & ~iso, 0], OB[zz & ~iso, 1], s=4, c='k', label='콘·벽(지도)'); a_.scatter(OB[zz & iso, 0], OB[zz & iso, 1], s=30, marker='x', c='m', label='외톨이(잡음, 뺌)')
    g0 = evalp(B, mode); a_.plot(B[:, 0], B[:, 1], 'b-', lw=1, label='지금 그린 선(러너 곡선)'); a_.scatter(B[inz & (g0 < 0), 0], B[inz & (g0 < 0), 1], s=10, c='r', label='그린 선: 몸이 겹치는 자리')
    a_.plot(P[:, 0], P[:, 1], 'g-', lw=2, label='가장 넓게 비켜 가는 선'); a_.scatter(P[inz & (g < 0), 0], P[inz & (g < 0), 1], s=10, c='orange', label='최적 선도 겹침')
    hh = heads(P, mode)
    for i in range(0, len(P), 8):
        if inz[i]:
            c_, s_ = math.cos(hh[i]), math.sin(hh[i]); Rm = np.array([[c_, -s_], [s_, c_]]); box = (Rm @ np.array([[L2, W2], [L2, -W2], [-L2, -W2], [-L2, W2], [L2, W2]]).T).T + P[i]
            a_.plot(box[:, 0], box[:, 1], '-', c='g' if g[i] >= 0 else 'orange', lw=0.5)
    a_.scatter(SL[:, 0], SL[:, 1], s=25, c='b', marker='o', zorder=5)
    for i, e in enumerate(SL):
        a_.annotate(f'#{i + 7}', e, fontsize=7, color='b')
    a_.set_xlim(ZLO[0] - 0.2, ZHI[0] + 0.2); a_.set_ylim(ZLO[1] - 0.2, ZHI[1] + 0.2); a_.set_aspect('equal'); a_.grid(alpha=0.3)
    a_.set_title(f'몸 방향 = {"경로 접선(지금 설정)" if mode == "tangent" else "고정(옆걸음)"} — 최소 여유 {g[inz].min() * 100:+.1f} cm')
ax[0].legend(fontsize=7, loc='lower left')
plt.tight_layout(); plt.savefig(os.path.join(H, 'best_line.png'), dpi=110)
print('그림', os.path.join(H, 'best_line.png'))
