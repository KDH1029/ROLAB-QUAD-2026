#!/usr/bin/env python3
"""'AMCL 로 찾고, 서는 곳에서 ICP 로 확인' 방식 평가(콘은 지도와 같다는 전제 → 원래 지도).
서는 곳(출발·정지선·주차)에서만: AMCL 자세를 씨앗으로 ICP(1초·2초 스캔) → 둘이 3 cm·1° 안으로 같고, 일치 ≥ 0.25, 슬라럼 밖,
ICP 답이 AMCL 과 15 cm·5° 안이면 그 ICP 답으로 좌표를 고침. 아니면 그대로(멈추지 않음). 주행 중에는 안 고침(대회 러너와 같은 조건).
사용: ~/venv/go2/bin/python3 stop_fuse.py r1645 tuned"""
import math
import os
import sys

import numpy as np

H = os.path.dirname(os.path.abspath(__file__)); os.chdir(H)
os.environ['SCENS'] = ''
src = open('compare.py').read(); exec(compile(src[:src.index('res = {}')], 'compare_head', 'exec'))   # 같은 지도·덤프·도구


def icp_check(t, seed, Pc, min_inl=0.25):
    q = comp(seed, Pc)
    if in_slalom(q):
        return None, '슬라럼'
    a, b = icp(t, seed, Pc, 2.0), icp(t, seed, Pc, 1.0)
    if a is None or b is None:
        return None, '스캔 부족'
    e = err(comp(a[0], Pc), comp(b[0], Pc))
    if not (e[0] < 0.03 and e[1] < 1.0):
        return None, f'1초/2초 불일치 {e[0] * 100:.0f} cm'
    if a[1] < min_inl:
        return None, f'일치 {a[1]:.2f}'
    return a, 'ok'


for sc in ('S0', 'S1', 'S2'):
    cor = Corrupt(sc); AM = amcl_track(sc)
    T_fl, T_icp, T_fu = np.array(T_ST), np.array(T_ST), np.array(T_ST)
    E = {k: [] for k in ('FAST-LIO만', 'AMCL만', '서는 곳 ICP만(일치 0.25)', '서는 곳 AMCL→ICP 확인')}
    log = []
    for t in ts:
        x, y, a = true_at(t); Pc = np.array(cor.apply(t - t0, x, y, a)); tr = comp(T_ST, true_at(t))
        k = np.searchsorted(AM[:, 0], t) - 1; Cam = AM[k, 1:] if k >= 0 else np.array(T_ST)
        if t in EV:
            r, why = icp_check(t, T_icp, Pc)
            if r is not None and err(comp(r[0], Pc), comp(T_icp, Pc))[0] < 0.6:
                T_icp = r[0]
            r2, why2 = icp_check(t, Cam, Pc)
            ok = r2 is not None and all(v < lim for v, lim in zip(err(comp(r2[0], Pc), comp(Cam, Pc)), (0.15, 5.0)))
            if ok:
                T_fu = r2[0]
            log.append(f"{t - t0:4.0f}초 {EV[t]}: AMCL→ICP {'반영' if ok else '안 함(' + why2 + ')'} · 반영 뒤 오차 {err(comp(T_fu, Pc), tr)[0] * 100:.1f} cm")
        E['FAST-LIO만'].append(err(comp(T_fl, Pc), tr)); E['AMCL만'].append(err(comp(Cam, Pc), tr))
        E['서는 곳 ICP만(일치 0.25)'].append(err(comp(T_icp, Pc), tr)); E['서는 곳 AMCL→ICP 확인'].append(err(comp(T_fu, Pc), tr))
    print(f'\n[{sc}] 위치 오차 cm: 중앙 / p95 / 최대 / 마지막 · 각도 최대')
    for k, v in E.items():
        v = np.array(v); print(f'   {k:22s} {np.median(v[:, 0]) * 100:6.1f} {np.percentile(v[:, 0], 95) * 100:6.1f} {v[:, 0].max() * 100:6.1f} {v[-1, 0] * 100:6.1f}   {v[:, 1].max():4.1f}°')
    for l in log:
        print('     ', l)
