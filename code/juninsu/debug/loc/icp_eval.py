#!/usr/bin/env python3
"""3D ICP 오프라인 검증 — 재생 덤프(out/<태그>/dump.npz)를 오늘 지도(rec1650)에 맞춘다. ROS 안 씀.
사용: ~/venv/go2/bin/python3 icp_eval.py out/r1645 [--map rec1650] [--quick]
  ① 출발 T: 출발 3초 뒤 스캔으로 지도에 맞춤(오늘 출발 정렬과 같은 역할)
  ② 기준 궤적 T_ref(t): 1초마다, 2초 누적 스캔·4단 ICP·앞 결과에서 이어감 → FAST-LIO 가 지도에서 얼마나 흐르나(주차 앞뒤 포함)
  ③ 강건성: 여러 시점에서 초기값을 일부러 틀어(0.1/0.3/0.5 m · 5/10/20°) 설정별로 되찾는 비율·시간·받아들임 기준
결과: out/<태그>/icp_eval.json, icp_eval.png"""
import json
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import icp_core as ic  # noqa: E402

D = os.path.abspath(sys.argv[1]); NAME = sys.argv[sys.argv.index('--map') + 1] if '--map' in sys.argv else 'rec1650'
QUICK = '--quick' in sys.argv
R = os.path.expanduser(f'~/ws/junk/routes/{NAME}')
dump = ic.Dump(os.path.join(D, 'dump.npz'))
path = np.load(os.path.join(R, 'editor_cache.npz'))['path']
MAP = ic.load_map(os.path.join(R, 'map.npz'), path_xy=path)
FULL = ic.Matcher(MAP)
print(f'지도 {NAME}: 벽·콘 점 {len(MAP)} (바닥 위 0.10~1.8 m, 경로 둘레 3 m) · 덤프 {os.path.basename(D)}: 자세 {len(dump.odom)} 점구름 {len(dump.ct)}', flush=True)
t0, t1 = dump.ot[0], dump.ot[-1]

def solve(m, t, T3, win, rng=None):
    S = dump.scan_at(t, win=win, rng=rng); P = dump.pose_at(t)
    if len(S) < 200:
        return None
    M0 = ic.T3_to4(T3) @ P
    t_ = time.perf_counter(); M, rms, it = m.align(S, M0); ms = (time.perf_counter() - t_) * 1000
    T = ic.T4_to3(M @ np.linalg.inv(P))
    return dict(T=T, rms=rms, inl=m.inlier(S, M), ms=ms, n=len(S))

def dT(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1]), abs(math.degrees((a[2] - b[2] + math.pi) % (2 * math.pi) - math.pi))

# ① 출발
ts = t0 + 3.0
st = solve(FULL, ts, np.zeros(3), 2.0)
print(f"① 출발 T = ({st['T'][0]:+.3f}, {st['T'][1]:+.3f} m, {math.degrees(st['T'][2]):+.2f}°)  잔차 {st['rms'] * 100:.1f} cm  일치(10 cm 안) {st['inl'] * 100:.0f}%  {st['ms']:.0f} ms", flush=True)

# ② 기준 궤적
ref = []; T = st['T']
for t in np.arange(ts, t1, 1.0):
    r = solve(FULL, t, T, 2.0)
    if r is None:
        continue
    if r['inl'] > 0.5:
        T = r['T']
    P = dump.pose_at(t); ref.append((t - t0, *r['T'], r['rms'], r['inl'], dump.height_at(t), P[0, 3], P[1, 3]))
ref = np.array(ref)
def apply(T, x, y):
    c, s_ = math.cos(T[2]), math.sin(T[2]); return np.array([c * x - s_ * y + T[0], s_ * x + c * y + T[1]])
# 로봇 자리에서의 위치 오차: 출발 때 한 번 맞춘 T 로 본 위치 vs 기준 T(t) 로 본 위치 (yaw 차이의 지렛대까지 포함)
dev = np.array([(np.hypot(*(apply(st['T'], row[7], row[8]) - apply(row[1:4], row[7], row[8]))), dT(row[1:4], st['T'])[1]) for row in ref])
low = ref[:, 6] < 0.15
print(f"② FAST-LIO 밀림(출발 1회 정렬만 했을 때 로봇 자리 위치 오차, 기준 궤적 {len(ref)}점): 중앙 {np.median(dev[:, 0]) * 100:.1f} cm · 최대 {dev[:, 0].max() * 100:.1f} cm"
      f"({ref[dev[:, 0].argmax(), 0]:.0f}초) · 마지막 {dev[-1, 0] * 100:.1f} cm · 각도 최대 {dev[:, 1].max():.2f}° 마지막 {dev[-1, 1]:.2f}° · 기준 궤적 일치 중앙 {np.median(ref[:, 5]) * 100:.0f}% 최저 {ref[:, 5].min() * 100:.0f}%", flush=True)
for q in (30, 60, 90, 120, 180, 240):
    k = np.where(ref[:, 0] <= q)[0]
    if len(k) and q <= ref[-1, 0] + 1:
        print(f"     ~{q:3d}초: 최대 {dev[k, 0].max() * 100:5.1f} cm · {dev[k, 1].max():.2f}°", flush=True)
parks = []
if low.any():
    k = np.where(low)[0]; groups = np.split(k, np.where(np.diff(k) > 2)[0] + 1)
    for g in groups:
        a, b = g[0], g[-1]
        bef = ref[max(0, a - 3), 1:4]; aft_i = min(len(ref) - 1, b + 4); aft = ref[aft_i, 1:4]
        x_, y_ = ref[aft_i, 7], ref[aft_i, 8]; e = (float(np.hypot(*(apply(bef, x_, y_) - apply(aft, x_, y_)))), dT(bef, aft)[1]); parks.append(dict(t_down=float(ref[a, 0]), t_up=float(ref[b, 0]), jump_m=e[0], jump_deg=e[1], t_after=float(ref[aft_i, 0] + t0)))
        print(f"   주차(몸 높이 < 0.15 m) {ref[a, 0]:.0f}~{ref[b, 0]:.0f}초: 앉기 전→일어난 뒤 T 변화 {e[0] * 100:.1f} cm · {e[1]:.2f}°  ← FAST-LIO 가 주차 충격으로 흐른 양", flush=True)
else:
    print('   이 녹화에는 주차(몸 높이 < 0.15 m)가 없음', flush=True)

# ③ 강건성
CFG = {} if '--drift-only' in sys.argv else {'lab4단·1초': (ic.Matcher.STAGES, 1.0), 'lab4단·2초': (ic.Matcher.STAGES, 2.0),
       '3단(0.8 뺌)·1초': (ic.Matcher.STAGES[1:], 1.0), '5단(1.6 추가)·2초': (((1.6, 8.0, 30),) + ic.Matcher.STAGES, 2.0)}
if QUICK:
    CFG = {k: CFG[k] for k in list(CFG)[:2]}
mats = {k: ic.Matcher(MAP, stages=v[0]) for k, v in CFG.items()}
moments = list(ref[::10 if not QUICK else 20, 0] + t0) + [p['t_after'] for p in parks]
PERT = [(0.1, 5), (0.3, 10), (0.5, 20), (0.8, 30)]
rng = np.random.default_rng(3); res = {k: [] for k in CFG}
Tref = lambda t: ref[np.argmin(abs(ref[:, 0] + t0 - t)), 1:4]
for t in moments:
    Tr = Tref(t)
    for mag, deg in PERT:
        for rep in range(4):
            a = rng.uniform(0, 2 * math.pi); sgn = 1 if rep % 2 == 0 else -1
            T0 = Tr + [mag * math.cos(a), mag * math.sin(a), sgn * math.radians(deg)]
            for k, (stg, win) in CFG.items():
                r = solve(mats[k], t, T0, win, rng=np.random.default_rng(rep))
                if r is None:
                    continue
                e = dT(r['T'], Tr); res[k].append((mag, deg, e[0], e[1], r['inl'], r['rms'], r['ms']))
print('③ 강건성 — 일부러 틀린 초기값에서 기준 T 로 되찾는 비율(5 cm·2° 안) · 시간', flush=True)
print('   설정              ' + '  '.join(f'{m:.1f}m/{d}°' for m, d in PERT) + '   평균 ms   틀린 답 중 일치≥60% (받아들이면 사고)')
summary = {}
for k, rows in res.items():
    A = np.array(rows); ok = (A[:, 2] < 0.05) & (A[:, 3] < 2.0)
    rate = [ok[(A[:, 0] == m)].mean() * 100 for m, _ in PERT]
    bad = ~ok; fa = float(np.mean(A[bad, 4] >= 0.6) * 100) if bad.any() else 0.0
    summary[k] = dict(rate=rate, ms=float(A[:, 6].mean()), false_accept=fa, inl_ok=float(np.median(A[ok, 4])) if ok.any() else None,
                      inl_bad_max=float(A[bad, 4].max()) if bad.any() else None)
    print(f"   {k:16s} " + '  '.join(f'{v:6.0f}%' for v in rate) + f"   {A[:, 6].mean():6.0f}    {fa:4.0f}% (틀린 답 {bad.sum()}개, 일치 최고 {summary[k]['inl_bad_max'] or 0:.2f} · 맞은 답 일치 중앙 {summary[k]['inl_ok'] or 0:.2f})")
json.dump(dict(start=dict(T=list(st['T']), rms=st['rms'], inl=st['inl']), ref=ref.tolist(), parks=parks, robust=summary,
               map=NAME, dump=os.path.basename(D)), open(os.path.join(D, 'icp_eval.json'), 'w'), ensure_ascii=False, indent=1)
try:
    import matplotlib; matplotlib.use('Agg'); matplotlib.rcParams['font.family'] = 'NanumGothic'; matplotlib.rcParams['axes.unicode_minus'] = False
    import matplotlib.pyplot as plt
    f, ax = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    ax[0].plot(ref[:, 0], (ref[:, 1] - st['T'][0]) * 100, label='x'); ax[0].plot(ref[:, 0], (ref[:, 2] - st['T'][1]) * 100, label='y'); ax[0].set_ylabel('T 변화 cm'); ax[0].legend()
    ax[1].plot(ref[:, 0], np.degrees(ref[:, 3] - st['T'][2])); ax[1].set_ylabel('yaw 변화 °')
    ax[2].plot(ref[:, 0], ref[:, 6], c='k', label='몸 높이 m'); ax[2].plot(ref[:, 0], ref[:, 5], c='g', label='일치율'); ax[2].legend(); ax[2].set_xlabel('초')
    ax[0].set_title(f'{os.path.basename(D)} → {NAME} 지도: FAST-LIO 가 지도에서 흐른 양(기준 궤적)')
    f.savefig(os.path.join(D, 'icp_eval.png'), dpi=80, bbox_inches='tight')
except Exception as e:  # noqa: BLE001
    print('그림 실패:', e)
print('저장', os.path.join(D, 'icp_eval.json'))
