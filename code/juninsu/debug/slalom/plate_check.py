#!/usr/bin/env python3
"""콘 받침대 밟음 검사(받침대 크기 모델) — 녹화 지도(rec1650, XT16 3D)에서 슬라럼 콘을 묶어 중심·지름을 재고,
그 둘레에 정사각 받침대(한 변 S)를 씌워, 시뮬레이션 궤적(V0·V2)과 사람 조종 녹화(16:50)의 네 발 자리가 받침대 위를 지나는지 센다.
받침대 방향은 모름 → 최선(내접원 S/2)·최악(외접원 S/√2) 두 경우. 발 반지름 2 cm + 발 배치 산포 2.1 cm(실측) 더함.
발 자리 = 몸 중심 기준 (±0.19, ±0.14) m. 사용: ~/venv/go2/bin/python3 plate_check.py [S=0.38]"""
import json, math, os, sys
import numpy as np
from scipy import ndimage as ndi
from scipy.spatial import cKDTree
H = os.path.dirname(os.path.abspath(__file__)); J = os.path.dirname(os.path.dirname(H))
SIDES = [float(a) for a in sys.argv[1:]] or [0.30, 0.34, 0.38, 0.42]
R = json.load(open(os.path.join(J, 'routes', 'rec1650', 'route.json')))['route']; RP = np.array([[e['x'], e['y']] for e in R])
SL = RP[6:25]; ZLO, ZHI = SL.min(0) - 0.4, SL.max(0) + 0.4
sys.argv = ['x', 'rec1650']; sys.path.insert(0, os.path.join(J, 'tools')); import route_editor as ed  # noqa: E402
XW = ed.real_walls(ed.load_cache()); XW = XW[np.all((XW > ZLO - 0.3) & (XW < ZHI + 0.3), 1)]
# 콘 중심: rec1650 지도 − 콘 지운 지도 A 의 차이(콘 점)에서 바닥 위 0.05~0.45 m, 반경 0.15 m 밀도 봉우리(서로 0.30 m 이상) — 10개
C = np.load(os.path.join(H, 'cone_centres_rec1650.npy')); DIA = np.full(len(C), np.nan)
print(f'슬라럼 콘 {len(C)}개 (cone_centres_rec1650.npy)')
FEET = [(0.19, 0.14), (0.19, -0.14), (-0.19, 0.14), (-0.19, -0.14)]
FOOT_R, SPREAD = 0.02, 0.021
def feet_xy(tr):
    out = []
    for x, y, a in tr:
        c, s = math.cos(a), math.sin(a)
        out.append([(x + c * fx - s * fy, y + s * fx + c * fy) for fx, fy in FEET])
    return np.array(out)  # (T,4,2)
def load_trace(v):
    t = np.load(os.path.join(H, f'sim_scurve_{v}_trace.npy')); return t[:, 1:4]
d = np.load(os.path.join(J, 'routes', 'rec1650', 'map.npz')); od = d['odom']; od = od[np.argsort(od[:, 1])]
q = od[:, 5:9]; yaw = np.arctan2(2 * (q[:, 3] * q[:, 2] + q[:, 0] * q[:, 1]), 1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2))
rec = np.c_[od[:, 2], od[:, 3], yaw]; rec = rec[np.all((rec[:, :2] > ZLO) & (rec[:, :2] < ZHI), 1)]
TRS = {'V0 지금 설정(시뮬)': load_trace('V0'), 'V2 몸=경로(시뮬, juninsu 기본)': load_trace('V2'), 'REC 사람 조종 녹화': rec}
TC = cKDTree(C); out = {'cones': C.round(3).tolist(), 'cone_width': DIA.round(3).tolist(), 'res': {}}
for name, tr in TRS.items():
    F = feet_xy(tr).reshape(-1, 2); dist, idx = TC.query(F)
    print(f'\n[{name}] 발 자리 {len(F)}개 · 콘 중심까지 발 최소 {dist.min() * 100:.1f} cm')
    for S in SIDES:
        for case, rad in (('최선', S / 2), ('최악', S / math.sqrt(2))):
            lim = rad + FOOT_R + SPREAD
            hit = dist < lim; hc = sorted(set(idx[hit].tolist()))
            print(f'  받침대 {S * 100:.0f} cm {case}(반경 {rad * 100:.1f}+발 4.1 cm): 밟는 발 자리 {hit.mean() * 100:5.2f}% · 밟히는 콘 {len(hc)}/{len(C)}')
            out['res'][f'{name}|{S}|{case}'] = dict(frac=float(hit.mean()), cones=hc)
# 경로 점별: 점(몸 중심)에서 가장 가까운 콘까지 · 발이 받침대(38 cm 최악)에 안 닿으려면 필요한 콘-몸중심 거리
need = 0.38 / math.sqrt(2) + FOOT_R + SPREAD   # 콘 중심 - 발 최소
print(f'\n경로 점별 콘 거리 (발이 38 cm 받침대 최악 반경에 안 닿으려면 콘 중심-발 ≥ {need * 100:.1f} cm)')
prop = []
for i in range(5, 26):
    p = RP[i]; dd, j = TC.query(p)
    # 그 점에 몸이 경로 방향으로 섰을 때 발-콘 최소
    a = math.atan2(*(RP[min(i + 1, len(RP) - 1)] - RP[max(i - 1, 0)])[::-1])
    fd = TC.query(feet_xy([(p[0], p[1], a)]).reshape(-1, 2))[0].min()
    flag = fd < need
    msg = ''
    if flag:   # 콘 반대쪽으로 밀기: 필요한 만큼
        v = p - C[j]; v /= np.linalg.norm(v); shift = need - fd + 0.01; np_ = p + v * shift
        fd2 = TC.query(feet_xy([(np_[0], np_[1], a)]).reshape(-1, 2))[0].min()
        msg = f' → 제안 ({np_[0]:.2f}, {np_[1]:.2f}) {shift * 100:.0f} cm 이동, 발-콘 {fd2 * 100:.1f} cm'
        prop.append(dict(n=i + 1, x=round(float(np_[0]), 3), y=round(float(np_[1]), 3), shift_cm=round(shift * 100, 1), foot_cone_after_cm=round(fd2 * 100, 1)))
    print(f'  #{i + 1:2d} ({p[0]:5.2f},{p[1]:5.2f}) 콘까지 {dd * 100:5.1f} cm · 발-콘 최소 {fd * 100:5.1f} cm {"★밟을 수 있음" if flag else ""}{msg}')
out['proposals'] = prop
json.dump(out, open(os.path.join(H, 'plate_check.json'), 'w'), ensure_ascii=False, indent=1)
try:
    import matplotlib; matplotlib.use('Agg'); matplotlib.rcParams['font.family'] = 'NanumGothic'; matplotlib.rcParams['axes.unicode_minus'] = False
    import matplotlib.pyplot as plt
    f, ax = plt.subplots(figsize=(8, 12)); ax.scatter(XW[:, 0], XW[:, 1], s=3, c='k', marker='s', label='지도 벽·콘(XT16)')
    for c in C:
        ax.add_patch(plt.Rectangle(c - 0.19, 0.38, 0.38, fill=False, ec='orange', lw=1)); ax.add_patch(plt.Circle(c, 0.38 / math.sqrt(2), fill=False, ec='orange', ls=':', lw=0.8))
    ax.plot(RP[5:26, 0], RP[5:26, 1], 'b.-', ms=6, label='경로'); [ax.annotate(f'#{i + 1}', RP[i], fontsize=8, color='b') for i in range(5, 26)]
    F = feet_xy(TRS['V2 몸=경로(시뮬, juninsu 기본)'][::3]).reshape(-1, 2); ax.scatter(F[:, 0], F[:, 1], s=1, c='g', label='V2 발 자리')
    for pp in prop: ax.plot(pp['x'], pp['y'], 'r*', ms=10)
    ax.set_xlim(ZLO[0], ZHI[0]); ax.set_ylim(ZLO[1], ZHI[1]); ax.set_aspect('equal'); ax.grid(alpha=.3); ax.legend(loc='lower left', fontsize=8)
    ax.set_title('슬라럼 받침대(38 cm 네모·점선=방향 최악) vs 발 자리'); f.savefig(os.path.join(H, 'plate_check.png'), dpi=80, bbox_inches='tight')
except Exception as e:  # noqa: BLE001
    print('그림 실패', e)
