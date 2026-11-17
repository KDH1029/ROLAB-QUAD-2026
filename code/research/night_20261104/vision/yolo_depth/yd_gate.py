#!/usr/bin/env python3
"""콘 오검출(10-30 녹화본엔 실제 콘이 없음) → 가짜 콘이 얼마나 확정되는지, 문턱·모양 검사로 얼마나 줄이는지.

모양 특징 (박스 전체 깊이점, 세계): 높이 = z95 − 바닥(z5), 가로폭 = 카메라 좌우 방향 p10~p90 폭.
라바콘(23 cm, 밑면 14 cm) 기대: 높이 0.12~0.35 m, 가로폭 ≤ 0.30 m — 실제 콘으로 검증 안 됨(재현율 모름).
또 det3d.json 을 시간순으로 ConeMemory 에 넣어 conf 문턱 × min_hits × 모양검사 별 가짜 콘 수를 센다.
실행: ~/venv/go2/bin/python yd_gate.py   쓰기: gate_sweep.json
"""
import os, sys, json, time
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import yd_core as C

E = np.load(f'{HERE}/dets.npz'); FR, DET = E['frames'], E['dets']
DEP = np.load(f'{HERE}/depth_frames.npy', mmap_mode='r')
T_bd, ST, DELTA = C.load_calib(); T_cd = C.T_copt_dopt()
R3 = json.load(open(f'{HERE}/det3d.json'))
Rb, tb = T_bd[:3, :3], T_bd[:3, 3]
feat = []; tms = []
for r in R3:
    if not r['ok'] or r['cls'] != C.CONE: continue
    D = np.asarray(DEP[int(FR[r['frame'], 4])])
    t0 = time.perf_counter()
    Pd = C.depth_points(D, 2); Pc = Pd @ T_cd[:3, :3].T + T_cd[:3, 3]; uc, vc = C.project(Pc)
    X1, Y1, X2, Y2 = r['box']; m = (uc >= X1) & (uc <= X2) & (vc >= Y1) & (vc <= Y2) & (Pc[:, 2] > 0.05)
    tms.append(time.perf_counter() - t0)
    Q = Pd[m] @ Rb.T + tb                         # 몸체 (박스 전체)
    h, w = C.cone_shape(Q, r['p_body'])
    if h is None: continue
    feat.append(dict(i=len(feat), frame=r['frame'], t=r['t'], conf=r['conf'], p_world=r['p_world'], ground_z=r['ground_z'],
                     h=h, w=w))
H = np.array([f['h'] for f in feat]); W = np.array([f['w'] for f in feat])
gate = lambda f: C.shape_ok(f['h'], f['w'])
print('콘 오검출 모양: 높이 m p10/50/90', np.percentile(H, [10, 50, 90]).round(3), '가로폭 m p10/50/90', np.percentile(W, [10, 50, 90]).round(3))
print('모양검사 통과 비율', round(float(np.mean([gate(f) for f in feat])), 3), 'n', len(feat))
print('투영+박스 선택 시간 ms p50/p90', (np.percentile(tms, [50, 90]) * 1e3).round(2))

rows = []
for th in [0.4, 0.5, 0.6, 0.7, 0.8]:
    for mh in [2, 3]:
        for g in [False, True]:
            cm = C.ConeMemory(0.3, mh, 30.0); ever = set(); last_t = None; n_on = 0; n_t = 0; mx = 0
            F = sorted([f for f in feat if f['conf'] >= th and (not g or gate(f))], key=lambda f: f['t'])
            # 시간 격자 5 Hz 로 활성 수 세기
            fi = 0
            ts =[C.stamp_to_nuc(s, ST) + DELTA for s in FR[:, 1]]
            for t in ts:
                while fi < len(F) and F[fi]['t'] <= t:
                    f = F[fi]; cm.add(f['p_world'][:2], f['ground_z'] or 0.0, f['t'], f['conf']); fi += 1
                a = cm.active(t); ever.update(c['id'] for c in a); n_on += bool(a); n_t += 1; mx = max(mx, len(a))
            rows.append(dict(conf=th, min_hits=mh, shape_gate=g, det=len(F), phantom_ever=len(ever), max_at_once=mx, frac_time=round(n_on / n_t, 3)))
print('| conf ≥ | min_hits | 모양검사 | 남은 콘 오검출 | 확정된 가짜 콘(전체) | 동시 최대 | 가짜 콘 떠 있는 시간 비율 |')
print('|---|---|---|---|---|---|---|')
for r in rows:
    print(f"| {r['conf']} | {r['min_hits']} | {'켬' if r['shape_gate'] else '끔'} | {r['det']} | {r['phantom_ever']} | {r['max_at_once']} | {r['frac_time']} |")
json.dump(dict(features=feat, sweep=rows), open(f'{HERE}/gate_sweep.json', 'w'), ensure_ascii=False, indent=1)
