#!/usr/bin/env python3
"""junsu_v2 위치 보정 소스 시험 — 16:45 녹화(재생 덤프), v2 옛 지도(11-04 1459) vs 새 지도(rec1650).
① 2D ICP (v2 위치 감시 노드의 재정렬 = start_align_core, ±0.5 m·±20°, 2초 스캔) — 지도: 옛 arena1459_nocone_slalomfree / 새 rec1650 map.pgm
② 3D ICP (icp_core 4단, 2초 점구름)                                         — 지도: 옛 fixed1459_2cm_map.npz / 새 rec1650 map.npz
기준: FAST-LIO(지도 대비 4 cm 안) — 출발 정렬 T 로 본 자리와 ICP 답의 차이 = ICP 오차. 받아들임: 2D = ok, 3D = 일치 ≥ 0.5.
시점: 3초부터 5초마다 + 서는 곳(정지선 39·148초, 주차 205초).  사용: ~/venv/go2/bin/python3 v2_icp_test.py"""
import math
import os
import sys

import numpy as np

H = os.path.expanduser('~/ws/junk/debug'); sys.path.insert(0, os.path.join(H, 'loc')); sys.path.insert(0, os.path.join(H, 'slalom', 'start_align'))
import icp_core as ic  # noqa: E402
import start_align_core as sa  # noqa: E402

TAG = os.environ.get('TAG', 'r1645')
d = np.load(os.path.join(H, 'loc', 'out', TAG, 'dump.npz'), allow_pickle=True)
dump = ic.Dump(os.path.join(H, 'loc', 'out', TAG, 'dump.npz'))
st, sc, meta = d['scan_t'], d['scan'], d['scan_meta']; a0, da = float(meta[0]), float(meta[1]); t0 = dump.ot[0]
MAPS = {'옛(v2, 11-04)': dict(pgm=os.path.expanduser('~/ws/junsu_v2/maps/align/arena1459_nocone_slalomfree.yaml'), npz=os.path.expanduser('~/ws/v_gosu/eval/out/fixed1459_2cm_map.npz'), T=(0.295, 0.022, math.radians(6.0))),
        '새(rec1650)': dict(pgm=os.path.expanduser('~/ws/junk/routes/rec1650/map.yaml'), npz=os.path.expanduser('~/ws/junk/routes/rec1650/map.npz'), T=(0.043, -0.010, math.radians(-3.0)))}
path = np.load(os.path.expanduser('~/ws/junk/routes/rec1650/editor_cache.npz'))['path']
moments = sorted(set(list(np.arange(t0 + 3, dump.ot[-1] - 1, 5.0)) + ([t0 + 39, t0 + 148, t0 + 205] if os.environ.get('TAG', 'r1645') == 'r1645' else [])))
yaw = lambda q: math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))


def ap(T, x, y):
    c, s = math.cos(T[2]), math.sin(T[2]); return np.array([c * x - s * y + T[0], s * x + c * y + T[1]])


def start_T(wm):
    """v2 가 r 에서 하는 출발 정렬(±0.3 m·±10°), 거부면 넓혀서(±1.0 m·±20°) — 이 녹화의 지도←FAST-LIO."""
    pairs = []
    for tt, rr in zip(st, sc):
        if t0 + 0.5 <= tt <= t0 + 3.5:
            kk = int(np.argmin(abs(dump.ot - tt))); o = dump.odom[kk]; pairs.append((sa.scan_points(rr, a0, da), (o[1], o[2], yaw(o[4:8]))))
    r = sa.align(wm, pairs, max_xy=0.30, max_yaw_deg=10.0)
    if r['ok']:
        return np.array(r['T']), 'v2 기본 범위 OK'
    r2 = sa.align(wm, pairs, max_xy=1.0, max_yaw_deg=20.0)
    return np.array(r2['T'] if r2['ok'] else r2.get('T_rejected', (0, 0, 0))), f"★ v2 기본 범위(±0.3 m·±10°)에선 거부 → 넓혀서 {'OK' if r2['ok'] else '거부'}"


for name, M in MAPS.items():
    wm = sa.WallMap(M['pgm']); T, how = start_T(wm)
    print(f'\n[{TAG}] 지도 {name} 출발 정렬 T = ({T[0]:+.3f}, {T[1]:+.3f}, {math.degrees(T[2]):+.2f}°) · {how}')
    lo_hi = path  # 3D 지도 자르기(경로 둘레) — 옛 지도도 같은 경기장 자리
    P3 = ic.load_map(M['npz'], path_xy=np.c_[ap(T, path[:, 0], path[:, 1])[0], ap(T, path[:, 0], path[:, 1])[1]] if False else None)
    k = np.all((P3[:, :2] > path.min(0) - 3) & (P3[:, :2] < path.max(0) + 3), 1) if name.startswith('새') else np.ones(len(P3), bool)
    m3 = ic.Matcher(P3[k])
    r2, r3 = [], []
    for t in moments:
        P = dump.pose_at(t); x, y = P[0, 3], P[1, 3]; q = ap(T, x, y)
        # ① 2D ICP
        pairs = []
        for tt, rr in zip(st, sc):
            if t - 2.0 <= tt <= t:
                kk = int(np.argmin(abs(dump.ot - tt))); o = dump.odom[kk]; pairs.append((sa.scan_points(rr, a0, da), (o[1], o[2], yaw(o[4:8]))))
        if pairs:
            r = sa.align(wm, pairs, max_xy=0.5, max_yaw_deg=20.0, prior=tuple(T))
            TT = r['T'] if r['ok'] else r.get('T_rejected', T)
            r2.append((t - t0, r['ok'], float(np.hypot(*(ap(TT, x, y) - q))), r['match']))
        # ② 3D ICP
        S = dump.scan_at(t, win=2.0)
        if len(S) > 200:
            M0 = ic.T3_to4(T) @ P; Mo, rms, _ = m3.align(S, M0); Tn = ic.T4_to3(Mo @ np.linalg.inv(P))
            r3.append((t - t0, m3.inlier(S, Mo), float(np.hypot(*(ap(Tn, x, y) - q))), rms))
    A2, A3 = np.array(r2, float), np.array(r3, float)
    ok2 = A2[:, 1] > 0; ok3 = A3[:, 1] >= 0.5
    sp2 = np.abs(A2[ok2, 2] - np.median(A2[ok2, 2])) if ok2.any() else np.array([np.nan]); sp3 = np.abs(A3[ok3, 2] - np.median(A3[ok3, 2])) if ok3.any() else np.array([np.nan])
    print(f'  (흔들림 = 받은 답 오차 − 그 중앙값: 2D p95 {np.percentile(sp2, 95) * 100:.1f} cm · 3D p95 {np.percentile(sp3, 95) * 100:.1f} cm)')
    print(f'  ① 2D ICP(±0.5 m·±20°): 받아들임 {ok2.sum()}/{len(A2)} · 받은 답 오차 중앙 {np.median(A2[ok2, 2]) * 100 if ok2.any() else float("nan"):.1f} · p95 {np.percentile(A2[ok2, 2], 95) * 100 if ok2.any() else float("nan"):.1f} · 최대 {A2[ok2, 2].max() * 100 if ok2.any() else float("nan"):.1f} cm · 15 cm 넘게 틀린 걸 받아들임 {int(np.sum(ok2 & (A2[:, 2] > 0.15)))}번 · 일치 중앙 {np.median(A2[:, 3]):.2f}')
    print(f'  ② 3D ICP(4단):         일치≥0.5 {ok3.sum()}/{len(A3)} · 받은 답 오차 중앙 {np.median(A3[ok3, 2]) * 100 if ok3.any() else float("nan"):.1f} · p95 {np.percentile(A3[ok3, 2], 95) * 100 if ok3.any() else float("nan"):.1f} · 최대 {A3[ok3, 2].max() * 100 if ok3.any() else float("nan"):.1f} cm · 15 cm 넘게 틀린 걸 받아들임 {int(np.sum(ok3 & (A3[:, 2] > 0.15)))}번 · 일치 중앙 {np.median(A3[:, 1]):.2f}')
    for lab, tt in ((('정지선1', 39), ('정지선2', 148), ('주차', 205)) if TAG == 'r1645' else ()):
        i2 = int(np.argmin(abs(A2[:, 0] - tt))); i3 = int(np.argmin(abs(A3[:, 0] - tt)))
        print(f'     {lab}({tt}초): 2D {"OK" if A2[i2, 1] else "거부"} {A2[i2, 2] * 100:.1f} cm · 3D 일치 {A3[i3, 1]:.2f} {A3[i3, 2] * 100:.1f} cm')
