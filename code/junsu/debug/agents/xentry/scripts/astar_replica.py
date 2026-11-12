#!/usr/bin/env python3
"""ego(review) 격자·A* 를 파이썬으로 그대로 옮겨, 기록된 스캔 한 장에서 '외톨이 칸 있음/없음' 계획을 비교.
grid_map.cpp: 0.05 m, 20 m 정사각(로봇 중심), 스캔 점 + 추가 장애물(뒤쪽 벽 기억) → 원판 팽창 ceil(0.20/0.05)=4칸.
a_star.cpp: 8방향, 옥타일 휴리스틱, 15000회, 시작/목표가 팽창 안이면 가까운 빈칸, 못 닿으면 휴리스틱 최소 노드(g>0.8 m)로 부분 경로.
사용: astar_replica.py <rec.jsonl> <t(기록 기준 초)> <out.png>"""
import json, sys, heapq, math, os
import numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
plt.rcParams['font.family'] = 'NanumGothic'; plt.rcParams['axes.unicode_minus'] = False
RES, SIZE, INFL, ITERS, PARTIAL, HOR = 0.05, 20.0, 0.20, 15000, 0.8, 7.5
GHOST = np.array([1.98, 0.10])


def build(robot, pts, infl=INFL):
    org = np.array(robot[:2]) - SIZE / 2; n = int(math.ceil(SIZE / RES))
    occ = np.zeros((n, n), bool); ij = np.floor((pts - org) / RES).astype(int)
    ok = (ij >= 0).all(1) & (ij < n).all(1); occ[ij[ok, 0], ij[ok, 1]] = True
    r = int(math.ceil(infl / RES)); inf = np.zeros_like(occ)
    for dx in range(-r, r + 1):
        for dy in range(-r, r + 1):
            if dx * dx + dy * dy <= r * r:
                inf |= np.roll(np.roll(occ, dx, 0), dy, 1)
    return org, occ, inf


def astar(org, inf, p0, goal):
    n = inf.shape[0]; g = np.array(goal, float)
    if np.hypot(*(g - p0)) > HOR: g = p0 + (g - p0) / np.hypot(*(g - p0)) * HOR
    s = tuple(np.floor((p0 - org) / RES).astype(int)); t = tuple(np.floor((g - org) / RES).astype(int))
    def near_free(c, R):
        if not inf[c]: return c
        for r in range(1, R + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    q = (c[0] + dx, c[1] + dy)
                    if 0 <= q[0] < n and 0 <= q[1] < n and not inf[q]: return q
        return None
    s = near_free(s, 15); t = near_free(t, 24)
    if t is None: return None, 'goal 막힘'
    h = lambda a: (max(abs(a[0] - t[0]), abs(a[1] - t[1])) + (math.sqrt(2) - 1) * min(abs(a[0] - t[0]), abs(a[1] - t[1]))) * RES
    op = [(h(s), 0.0, s)]; gs = {s: 0.0}; par = {s: None}; best = (1e9, None); it = 0; found = None
    while op and it < ITERS - 1:
        it += 1; f, gc, c = heapq.heappop(op)
        hc = h(c)
        if hc < best[0]: best = (hc, c)
        if c == t: found = c; break
        if gc > gs[c]: continue
        for dx, dy, w in ((1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (-1, -1, 1.414)):
            q = (c[0] + dx, c[1] + dy)
            if not (0 <= q[0] < n and 0 <= q[1] < n) or inf[q]: continue
            ng = gc + w * RES
            if ng < gs.get(q, 1e9): gs[q] = ng; par[q] = c; heapq.heappush(op, (ng + h(q), ng, q))
    why = f'목표 도달({it}회)'
    if found is None:
        b = best[1]
        if b is not None and gs[b] > PARTIAL: found = b; why = f'부분 경로(목표 못 닿음, {it}회, 남은 {best[0]:.2f} m)'
        else: return None, f'실패({it}회, 최근접 g={gs.get(b, 0):.2f} m ≤ 0.8)'
    path = []; c = found
    while c is not None: path.append((org + (np.array(c) + 0.5) * RES)); c = par[c]
    return np.array(path[::-1]), why


def entrance_open(org, inf):
    """입구 상자(x 1.55~2.25, y -0.15~0.45)만 써서 남서(입구 앞) ↔ 북동(X 통로 안)이 이어지나."""
    n = inf.shape[0]; ij = lambda p: tuple(np.floor((np.array(p) - org) / RES).astype(int))
    a, b = ij((1.55, -0.15)), ij((2.25, 0.45))
    sub = ~inf[a[0]:b[0] + 1, a[1]:b[1] + 1]
    from scipy.ndimage import label
    lab, _ = label(sub, structure=np.ones((3, 3)))
    sw = set(lab[:, :3].ravel()) - {0}   # 서쪽 가장자리(x≈1.55)
    ne = set(lab[-3:, sub.shape[1] // 2:].ravel()) - {0}   # 동쪽 가장자리 위쪽(x≈2.25, y>0.15) = X 통로
    return len(sw & ne) > 0


if __name__ == '__main__':
    R = [json.loads(l) for l in open(sys.argv[1]) if l.strip()]; T0 = R[0]['tw']; t = float(sys.argv[2])
    for r in R: r['t'] = r['tw'] - T0
    at = lambda ev: [r for r in R if r['ev'] == ev and r['t'] <= t][-1]
    sc, od, go = at('scan'), at('odom'), at('goal')
    wm = [r for r in R if r['ev'] == 'wm' and r['t'] <= t]; WM = np.array(wm[-1]['p']).reshape(-1, 2) if wm else np.zeros((0, 2))
    P = np.array(sc['p']).reshape(-1, 2); robot = np.array(sc['robot'][:2])
    rr = np.hypot(*(P - robot).T); P = P[(rr > 0.25) & (rr < 10.0)]
    allp = np.r_[P, WM]
    ghost = np.hypot(*(allp - GHOST).T) < 0.06
    print('스캔 점', len(P), '뒤쪽 기억', len(WM), '외톨이 칸 근처 점', ghost.sum())
    cases = [('기록 그대로(외톨이 칸 보임)', allp), ('외톨이 칸 점만 뺌', allp[~ghost])]
    fig, axs = plt.subplots(1, 2, figsize=(15, 7.5))
    for ax, (name, pts) in zip(axs, cases):
        org, occ, inf = build(robot, pts)
        path, why = astar(org, inf, robot, (go['x'], go['y']))
        op = entrance_open(org, inf)
        x0, x1, y0, y1 = 0.3, 4.2, -0.7, 2.6
        ext = [org[0], org[0] + inf.shape[0] * RES, org[1], org[1] + inf.shape[1] * RES]
        ax.imshow(inf.T, origin='lower', extent=ext, cmap='Reds', alpha=.35, vmin=0, vmax=1.5)
        k = (pts[:, 0] > x0) & (pts[:, 0] < x1) & (pts[:, 1] > y0) & (pts[:, 1] < y1)
        ax.scatter(pts[k, 0], pts[k, 1], s=5, c='b', label='장애물 점(스캔+뒤쪽 기억)')
        if path is not None: ax.plot(path[:, 0], path[:, 1], 'g-', lw=2.5, label='A* 경로(복제)')
        ax.plot(*robot, 'mo', ms=10); ax.plot(go['x'], go['y'], 'rX', ms=14, label=f'목표({go["x"]:.2f},{go["y"]:.2f})')
        ax.plot(*GHOST, 'o', mfc='none', mec='orange', ms=20, mew=2.5, label='외톨이 칸(1.98,0.10)')
        ax.set_xlim(x0, x1); ax.set_ylim(y0, y1); ax.set_aspect('equal'); ax.grid(alpha=.3); ax.legend(fontsize=8, loc='upper left')
        ax.set_title(f'{name}\n입구 {"열림" if op else "막힘"} · {why}', fontsize=10)
        print(name, '입구', '열림' if op else '막힘', '|', why, '| 경로 끝', None if path is None else path[-1].round(2), '| 북쪽 최대 y(x<1.45)', None if path is None else round(max([p[1] for p in path if p[0] < 1.45] + [-9]), 2))
    fig.suptitle(f'ego A* 복제 — 기록 {t:.1f}s, 로봇 ({robot[0]:.2f},{robot[1]:.2f})'); fig.tight_layout(); fig.savefig(sys.argv[3], dpi=95); print(sys.argv[3])
