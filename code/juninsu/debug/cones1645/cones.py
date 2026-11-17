#!/usr/bin/env python3
"""두 지도(rec1645·rec1650)에서 같은 방법으로 슬라럼 콘 중심 찾기 → rec1650 좌표로 비교·검증.
방법: 지도 점(관측 ≥3) 중 칸별 바닥 위 0.05~0.45 m · 슬라럼 구역 → 반경 0.15 m 밀도 봉우리(서로 0.30 m 이상) → 각 봉우리 0.25 m 안 점으로 원 맞추기(Kasa, 2회 다듬기).
길게 이어진 벽(긴 덩어리)은 제외 — 콘이 아닌 점: 0.15 m 밀도 봉우리 주변 점들의 주축 길이 > 0.5 m.
검증: 각 녹화 자기 주행(리모컨, 콘 안 닿음)의 몸 중심·네 발 → 콘 중심 최소 거리. 콘 밑동 반지름(~0.15 m)보다 작으면 '불가능'(콘 좌표가 틀림).
출력 cones_cmp.json, cones_cmp.txt 는 표준출력."""
import json, math, os, sys
import numpy as np
from scipy.spatial import cKDTree
H = os.path.dirname(os.path.abspath(__file__)); J = os.path.dirname(os.path.dirname(H))
T = np.array(json.load(open(os.path.join(H, 'T_1650_1645.json')))['xy_yaw'])
R = json.load(open(os.path.join(J, 'routes', 'rec1650', 'route.json')))['route']; RP = np.array([[e['x'], e['y']] for e in R])
SL = RP[6:25]; ZLO, ZHI = SL.min(0) - 0.4, SL.max(0) + 0.4
def apply(T, P):
    c, s = math.cos(T[2]), math.sin(T[2]); return np.c_[c * P[:, 0] - s * P[:, 1] + T[0], s * P[:, 0] + c * P[:, 1] + T[1]]
def load(name, to1650):
    d = np.load(os.path.join(J, 'routes', name, 'map.npz')); P = (d['vox'][d['cnt'] >= 3] + 0.5) * float(d['V'])
    if to1650: P = np.c_[apply(T, P[:, :2]), P[:, 2]]
    key = np.floor(P[:, :2] / 0.3).astype(np.int64); u, inv = np.unique(key, axis=0, return_inverse=True); inv = inv.ravel()
    fl = np.full(len(u), np.inf)
    for i in range(len(u)): pass
    # 바닥 = 칸별 하위 3 %
    order = np.argsort(inv); s = np.split(P[order, 2], np.cumsum(np.bincount(inv))[:-1]); fl = np.array([np.percentile(z, 3) for z in s])
    h = P[:, 2] - fl[inv]
    od = d['odom']; od = od[np.argsort(od[:, 1])]; q = od[:, 5:9]
    yaw = np.arctan2(2 * (q[:, 3] * q[:, 2] + q[:, 0] * q[:, 1]), 1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2))
    tr = np.c_[od[:, 2], od[:, 3], yaw]
    if to1650: tr = np.c_[apply(T, tr[:, :2]), tr[:, 2] + T[2]]
    k = (h > 0.05) & (h < 0.45) & np.all((P[:, :2] > ZLO) & (P[:, :2] < ZHI), 1)
    return P[k, :2], tr
def circle(p):
    A = np.c_[2 * p, np.ones(len(p))]; b = (p ** 2).sum(1); c, *_ = np.linalg.lstsq(A, b, rcond=None)
    r = math.sqrt(max(c[2] + c[0] ** 2 + c[1] ** 2, 1e-6)); return c[:2], r
def find(Q):
    TQ = cKDTree(Q); xs = np.arange(ZLO[0], ZHI[0], 0.02); ys = np.arange(ZLO[1], ZHI[1], 0.02)
    G = np.array(np.meshgrid(xs, ys)).reshape(2, -1).T; sc = np.array([len(v) for v in TQ.query_ball_point(G, 0.15)])
    o = np.argsort(-sc); pk = []
    for i in o:
        if sc[i] < 0.20 * sc[o[0]]: break
        if all(np.hypot(*(G[i] - p)) >= 0.30 for p in pk): pk.append(G[i])
    out = []
    for p in pk:
        c = p.copy()
        for _ in range(3):
            pts = Q[TQ.query_ball_point(c, 0.25)]
            c2, r = circle(pts)
            if np.hypot(*(c2 - c)) > 0.15 or not (0.06 < r < 0.30): break   # 원 맞추기 실패(한쪽만 보임) → 봉우리 유지
            c = c2
        pts = Q[TQ.query_ball_point(c, 0.25)]; ev = np.linalg.eigvalsh(np.cov(pts.T)) if len(pts) > 5 else [0, 0]
        out.append(dict(c=c, n=len(pts), r=float(np.median(np.hypot(*(pts - c).T))), elong=float(math.sqrt(ev[1] / max(ev[0], 1e-9)))))
    return out
FE = np.array([(0.19, 0.14), (0.19, -0.14), (-0.19, 0.14), (-0.19, -0.14)])
def clearance(tr, C):
    tr = tr[np.all((tr[:, :2] > ZLO) & (tr[:, :2] < ZHI), 1)]; TC = cKDTree(C)
    body = TC.query(tr[:, :2])[0]
    F = np.concatenate([tr[:, :2] + FE @ np.array([[math.cos(a), math.sin(a)], [-math.sin(a), math.cos(a)]]) for a in tr[:, 2]]) if False else \
        np.concatenate([tr[i, :2] + FE @ np.array([[math.cos(tr[i, 2]), math.sin(tr[i, 2])], [-math.sin(tr[i, 2]), math.cos(tr[i, 2])]]) for i in range(len(tr))])
    fd = TC.query(F)[0]
    return body.min(), fd.min(), float(np.mean(fd < 0.15)), np.percentile(fd, 1)
res = {}
for name, to in (('rec1650', False), ('rec1645', True)):
    Q, tr = load(name, to); cs = find(Q); res[name] = dict(Q=Q, tr=tr, cones=cs)
    print(f'\n[{name}] 슬라럼 구역 점 {len(Q)} · 콘 후보 {len(cs)}개 (rec1650 좌표)')
    for k, c in enumerate(sorted(cs, key=lambda c: (round(c["c"][0], 1), c["c"][1]))):
        print(f'   ({c["c"][0]:5.2f}, {c["c"][1]:5.2f})  점 {c["n"]:5d}  반경중앙 {c["r"] * 100:4.1f} cm  길쭉함 {c["elong"]:.1f}')
A = np.array([c['c'] for c in res['rec1650']['cones']]); B = np.array([c['c'] for c in res['rec1645']['cones']])
TB = cKDTree(B); d, j = TB.query(A)
print('\n콘 짝 (rec1650 ↔ rec1645, rec1650 좌표)')
for i in np.argsort(A[:, 1]):
    print(f'   1650 ({A[i, 0]:5.2f},{A[i, 1]:5.2f}) ↔ 1645 ({B[j[i], 0]:5.2f},{B[j[i], 1]:5.2f})  차이 {d[i] * 100:5.1f} cm')
extra = sorted(set(range(len(B))) - set(j.tolist())); print(f'rec1645 에만 있는 후보 {len(extra)}개:', [tuple(B[e].round(2)) for e in extra])
print('\n검증: 리모컨 주행(콘 안 닿음) → 콘 중심 최소 거리 (몸 중심 / 네 발 / 발이 15 cm 안인 비율 / 발 1 % 분위)')
for drive in ('rec1645', 'rec1650'):
    for cname in ('rec1645', 'rec1650'):
        C = np.array([c['c'] for c in res[cname]['cones']]); b, f, fr, p1 = clearance(res[drive]['tr'], C)
        print(f'   {drive[3:]} 주행 × {cname[3:]} 콘: 몸 {b * 100:5.1f} cm · 발 {f * 100:5.1f} cm · 발 15 cm 안 {fr * 100:5.2f}% · 발 p1 {p1 * 100:5.1f} cm')
json.dump({k: [dict(x=float(c['c'][0]), y=float(c['c'][1]), n=c['n'], r=c['r'], elong=c['elong']) for c in v['cones']] for k, v in res.items()}, open(os.path.join(H, 'cones_cmp.json'), 'w'), indent=1)
