#!/usr/bin/env python3
"""위치 보정 방식 비교 — 같은 녹화·같은 '틀어진 FAST-LIO'(scen.py) 위에서 지도 기준 위치 오차를 잰다.
참값 = 출발 T · 원래 FAST-LIO (실측 밀림 ≤ 4 cm). 오차 = 로봇 자리에서 추정 위치와 참값의 거리.
방식: FAST-LIO만 / AMCL만(재생한 nav2 AMCL CSV) / ICP 1초마다 / ICP 멈췄을 때만 / AMCL+ICP(둘이 맞을 때만 고침)
사용: ~/venv/go2/bin/python3 compare.py r1645 [base|tuned]   (out/<태그>/amcl_<S>_<변형>/poses.csv 를 읽음)"""
import csv
import json
import math
import os
import sys

import numpy as np

H = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, H)
import icp_core as ic  # noqa: E402
from scen import Corrupt  # noqa: E402

TAG = sys.argv[1]; PV = sys.argv[2] if len(sys.argv) > 2 else 'base'
D = os.path.join(H, 'out', TAG); R = os.environ.get('MAPDIR') or os.path.expanduser('~/ws/junk/routes/rec1650')   # MAPDIR = 콘 바꾼 지도(out/maps/A|B)
T_ST = {'r1645': (-0.025, -0.039, math.radians(-3.46)), 'r1650': (-0.021, -0.037, math.radians(0.38))}[TAG]
dump = ic.Dump(os.path.join(D, 'dump.npz')); _path = np.load(os.path.join(R, 'editor_cache.npz'))['path']
m = ic.Matcher(ic.load_map(os.path.join(R, 'map.npz'), path_xy=_path))
mh = ic.Matcher(ic.load_map(os.path.join(R, 'map.npz'), path_xy=_path, floor_cut=0.75))   # 벽 위주: 바닥 위 0.75 m 넘는 것만(콘 꼭대기 중앙 0.7 m)
scans_h = {}
O = dump.odom; t0 = O[0, 0]
yaw = lambda q: math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))
TRUE = np.array([(r[0], r[1], r[2], yaw(r[4:8])) for r in O])


def comp(A, B):                       # A ⊕ B (2D)
    c, s = math.cos(A[2]), math.sin(A[2]); return np.array([c * B[0] - s * B[1] + A[0], s * B[0] + c * B[1] + A[1], A[2] + B[2]])


def inv(A):
    c, s = math.cos(A[2]), math.sin(A[2]); return np.array([-(c * A[0] + s * A[1]), s * A[0] - c * A[1], -A[2]])


def true_at(t):
    k = int(np.clip(np.searchsorted(TRUE[:, 0], t), 0, len(TRUE) - 1)); return TRUE[k, 1:]


def err(e, g):
    return math.hypot(e[0] - g[0], e[1] - g[1]), abs(math.degrees((e[2] - g[2] + math.pi) % (2 * math.pi) - math.pi))


speed = np.r_[0, np.hypot(*np.diff(TRUE[:, 1:3], axis=0).T) / np.maximum(np.diff(TRUE[:, 0]), 1e-3)]
ts = np.arange(t0 + 3.0, TRUE[-1, 0], 1.0)
still = np.array([np.all(speed[(TRUE[:, 0] > t - 1.0) & (TRUE[:, 0] <= t)] < 0.03) for t in ts])
scans = {}
route = json.load(open(os.path.join(R, 'route.json')))['route']; RP = np.array([[e['x'], e['y']] for e in route])
SL = RP[5:25]; SLLO, SLHI = SL.min(0) - 0.4, SL.max(0) + 0.4
in_slalom = lambda q: bool(np.all(q[:2] >= SLLO) and np.all(q[:2] <= SLHI))
# 대회에서 로봇이 서는 곳: 출발 · 정지선(#5=#30) · 주차(#40) — 녹화가 그 점 0.4 m 안을 지날 때 가장 가까운 순간(대회에선 거기 서 있음)
EV = {}
for lab, n in (('정지선', 4), ('주차', 39)):
    d = np.array([np.hypot(*(comp(T_ST, true_at(t))[:2] - RP[n])) for t in ts]); near = np.where(d < 0.4)[0]
    for g in np.split(near, np.where(np.diff(near) > 2)[0] + 1):
        if len(g):
            EV[ts[g[np.argmin(d[g])]]] = lab
EV[ts[0]] = '출발'


def icp(t, Tguess, Pc, win=1.0):
    if (t, win) not in scans:
        scans[(t, win)] = dump.scan_at(t, win=win)
    S = scans[(t, win)]
    if len(S) < 200:
        return None
    Pt = dump.pose_at(t); Ptrue3 = true_at(t)
    # 스캔은 센서가 본 그대로(참 상대 운동), 초기값만 '틀어진 자세' 기준: 지도←몸 = Tguess ⊕ Pc
    G = comp(Tguess, Pc); M0 = ic.T3_to4(G); M0[2, 3] = Pt[2, 3]; M0[:3, :3] = ic.T3_to4((0, 0, G[2]))[:3, :3] @ ic.T3_to4((0, 0, -Ptrue3[2]))[:3, :3] @ Pt[:3, :3]
    M, rms, _ = m.align(S, M0)
    Mb = np.array([M[0, 3], M[1, 3], G[2] + math.atan2(M[1, 0], M[0, 0]) - math.atan2(M0[1, 0], M0[0, 0])])
    return comp(Mb, inv(Pc)), m.inlier(S, M)        # 새 지도←틀어진 odom


def icp_high(t, Tguess, Pc):
    if t not in scans_h:
        scans_h[t] = dump.scan_at(t, win=1.0, zmin=0.44)      # 몸 중심이 바닥 위 약 0.31 m → 0.75 m = z 0.44
    S = scans_h[t]
    if len(S) < 150:
        return None
    Pt = dump.pose_at(t); Ptrue3 = true_at(t); G = comp(Tguess, Pc)
    M0 = ic.T3_to4(G); M0[2, 3] = Pt[2, 3]; M0[:3, :3] = ic.T3_to4((0, 0, G[2]))[:3, :3] @ ic.T3_to4((0, 0, -Ptrue3[2]))[:3, :3] @ Pt[:3, :3]
    M, rms, _ = mh.align(S, M0)
    Mb = np.array([M[0, 3], M[1, 3], G[2] + math.atan2(M[1, 0], M[0, 0]) - math.atan2(M0[1, 0], M0[0, 0])])
    return comp(Mb, inv(Pc)), mh.inlier(S, M)


def icp_strict(t, Tguess, Pc):
    """개선판: 2초 스캔, 1초 결과와 3 cm·1° 안으로 같고, 일치 ≥ 0.6, 슬라럼 밖일 때만."""
    if in_slalom(comp(Tguess, Pc)):
        return None
    a, b = icp(t, Tguess, Pc, 2.0), icp(t, Tguess, Pc, 1.0)
    if a is None or b is None or a[1] < 0.6:
        return None
    e = err(comp(a[0], Pc), comp(b[0], Pc))
    return a if e[0] < 0.03 and e[1] < 1.0 else None


def amcl_track(sc):
    p = os.path.join(D, f'amcl_{sc}_{PV}', 'poses.csv')
    if not os.path.exists(p):
        return None
    rows = list(csv.DictReader(open(p)))
    od = np.array([(float(r['t']), float(r['x']), float(r['y']), float(r['yaw'])) for r in rows if r['kind'] == 'odom'])
    am = np.array([(float(r['t']), float(r['x']), float(r['y']), float(r['yaw'])) for r in rows if r['kind'] == 'amcl'])
    if len(am) == 0:
        return None
    C = []
    for a in am:                                       # 지도←odom 보정 = AMCL 자세 ⊕ (그 시각 틀어진 odom)^-1
        k = int(np.clip(np.searchsorted(od[:, 0], a[0]), 0, len(od) - 1)); C.append((a[0], *comp(a[1:], inv(od[k, 1:]))))
    return np.array(C)


res = {}
for sc in os.environ.get('SCENS', 'S0 S1 S2').split():
    cor = Corrupt(sc); Pc_of = {}
    for t in ts:
        x, y, a = true_at(t); Pc_of[t] = np.array(cor.apply(t - t0, x, y, a))
    truth = {t: comp(T_ST, true_at(t)) for t in ts}
    AM = amcl_track(sc)
    est = {k: {} for k in ('FAST-LIO만', 'AMCL만', 'ICP 1초마다', 'ICP 1초마다(개선)', 'ICP 1초마다(벽 위주)', 'ICP 멈췄을 때만', 'ICP 서는 곳만(개선)', 'AMCL+ICP', 'AMCL+ICP(개선)')}
    T_c, T_s, T_f = np.array(T_ST), np.array(T_ST), np.array(T_ST); last_still = False; fused_n = icp_n = still_n = 0
    T_e, T_g, T_k, T_h = np.array(T_ST), np.array(T_ST), np.array(T_ST), np.array(T_ST); ev_n = g_n = k_n = h_n = 0; dis = 0
    for i, t in enumerate(ts):
        Pc = Pc_of[t]
        est['FAST-LIO만'][t] = comp(T_ST, Pc)
        Camcl = None
        if AM is not None:
            k = np.searchsorted(AM[:, 0], t) - 1
            if k >= 0:
                Camcl = AM[k, 1:]; est['AMCL만'][t] = comp(Camcl, Pc)
        # ICP 1초마다: 지금 추정에서 시작, 일치 ≥ 0.5 이고 로봇 자리 보정 0.6 m·12° 안이면 받아들임
        r = icp(t, T_c, Pc)
        if r is not None and r[1] >= 0.5 and err(comp(r[0], Pc), comp(T_c, Pc))[0] < 0.6:
            T_c = r[0]; icp_n += 1
        est['ICP 1초마다'][t] = comp(T_c, Pc)
        r6 = icp_strict(t, T_k, Pc)                       # 개선: 슬라럼 밖·2초/1초 일치·일치 ≥ 0.6, 보정 0.6 m 안
        if r6 is not None and err(comp(r6[0], Pc), comp(T_k, Pc))[0] < 0.6:
            T_k = r6[0]; k_n += 1
        est['ICP 1초마다(개선)'][t] = comp(T_k, Pc)
        rh = icp_high(t, T_h, Pc)                          # 벽 위주(0.75 m 위): 받아들임은 'ICP 1초마다' 와 같음
        if rh is not None and rh[1] >= 0.5 and err(comp(rh[0], Pc), comp(T_h, Pc))[0] < 0.6:
            T_h = rh[0]; h_n += 1
        est['ICP 1초마다(벽 위주)'][t] = comp(T_h, Pc)
        # ICP 멈췄을 때만: 멈춘 구간마다 1번, 같은 받아들임 기준
        if still[i] and not last_still:
            r2 = icp(t, T_s, Pc)
            if r2 is not None and r2[1] >= 0.5 and err(comp(r2[0], Pc), comp(T_s, Pc))[0] < 0.6:
                T_s = r2[0]; still_n += 1
        last_still = still[i]
        est['ICP 멈췄을 때만'][t] = comp(T_s, Pc)
        # AMCL+ICP: AMCL 과 지금 추정이 15 cm·5° 넘게 다르면 AMCL 자세에서 ICP — ICP 답이 AMCL 과 15 cm·5° 안으로 맞을 때만 고침
        if Camcl is not None:
            ea = err(comp(Camcl, Pc), comp(T_f, Pc))
            if ea[0] > 0.15 or ea[1] > 5.0:
                r3 = icp(t, Camcl, Pc)
                if r3 is not None and r3[1] >= 0.5:
                    eb = err(comp(r3[0], Pc), comp(Camcl, Pc))
                    if eb[0] < 0.15 and eb[1] < 5.0:
                        T_f = r3[0]; fused_n += 1
        est['AMCL+ICP'][t] = comp(T_f, Pc)
        # 개선: 서는 곳(출발·정지선·주차)에서만 엄격 ICP, 보정 0.6 m 안
        if t in EV:
            r4 = icp_strict(t, T_e, Pc)
            if r4 is not None and err(comp(r4[0], Pc), comp(T_e, Pc))[0] < 0.6:
                T_e = r4[0]; ev_n += 1
        est['ICP 서는 곳만(개선)'][t] = comp(T_e, Pc)
        # 개선: AMCL 과 2초 연속 15 cm·5° 넘게 다를 때, AMCL 자세에서 엄격 ICP → AMCL 과 15 cm·5° 안이면 고침
        if Camcl is not None:
            ea = err(comp(Camcl, Pc), comp(T_g, Pc)); dis = dis + 1 if (ea[0] > 0.15 or ea[1] > 5.0) else 0
            if dis >= 2:
                r5 = icp_strict(t, Camcl, Pc)
                if r5 is not None:
                    eb = err(comp(r5[0], Pc), comp(Camcl, Pc))
                    if eb[0] < 0.15 and eb[1] < 5.0:
                        T_g = r5[0]; g_n += 1; dis = 0
        est['AMCL+ICP(개선)'][t] = comp(T_g, Pc)
    res[sc] = {}
    for k, E in est.items():
        if not E:
            continue
        e = np.array([err(E[t], truth[t]) for t in ts if t in E]); tt = np.array([t for t in ts if t in E]) - t0
        row = dict(med=float(np.median(e[:, 0])), p95=float(np.percentile(e[:, 0], 95)), max=float(e[:, 0].max()), last=float(e[-1, 0]), deg_max=float(e[:, 1].max()))
        if sc == 'S1':
            after = (tt >= 100) & (e[:, 0] < 0.10)
            row['recover_s'] = float(tt[after][0] - 100) if after.any() else None
        res[sc][k] = row
    res[sc]['_n'] = dict(icp_1s=icp_n, icp_still=still_n, fused=fused_n, ev=ev_n, fused2=g_n, icp_1s_strict=k_n, icp_high=h_n, still_moments=int(np.sum(np.diff(np.r_[0, still.astype(int)]) == 1)))

res['_n_by'] = None
names = {'S0': 'S0 그대로(보정이 망가뜨리나)', 'S1': 'S1 100초에 36 cm·8° 튐(넘어짐 흉내)', 'S2': 'S2 서서히 50 cm·6° 밀림'}
print(f'== {TAG} · 지도 {os.path.basename(R)} · AMCL {PV} · 시점 {len(ts)}개(1초) · 멈춘 구간 {res["S0"]["_n"]["still_moments"]}번 · 서는 곳(초) {sorted((round(t - t0), l) for t, l in EV.items())}')
for sc in [k for k in res if not k.startswith('_')]:
    print(f'\n[{names[sc]}]   위치 오차 cm: 중앙 / p95 / 최대 / 마지막 · 각도 최대' + (' · 튄 뒤 10 cm 안 복귀' if sc == 'S1' else ''))
    for k, r in res[sc].items():
        if k.startswith('_'):
            continue
        rec = (f" · {r['recover_s']:.0f}초" if r.get('recover_s') is not None else ' · 복귀 못 함') if sc == 'S1' else ''
        print(f"   {k:14s} {r['med'] * 100:6.1f} {r['p95'] * 100:6.1f} {r['max'] * 100:6.1f} {r['last'] * 100:6.1f}   {r['deg_max']:5.1f}°{rec}")
    n = res[sc]['_n']; print(f"   (받아들인 보정: ICP 1초마다 {n['icp_1s']}번 · 멈췄을 때 {n['icp_still']}번 · 서는 곳 {n['ev']}/{len(EV)}번 · AMCL+ICP {n['fused']}번 · 개선 {n['fused2']}번 · ICP 1초(개선) {n['icp_1s_strict']}번)")
json.dump({k: v for k, v in res.items() if not k.startswith('_')}, open(os.path.join(D, f'compare_{PV}_{os.path.basename(R)}.json'), 'w'), ensure_ascii=False, indent=1)
