#!/usr/bin/env python3
"""지도 판 비교 그림 — routes/<판>/ 의 지도(2 cm 복셀 → 편집기 벽 칸)를 16:50 좌표로 맞춰 나란히.
빨강 = 경로 0.6 m 안에 있는데 '다른 녹화 지도'엔 없는 벽 칸(사람 흔적·잡음 후보), 주황 = 사람 지운 판에서 지운 칸.
수치: map_quality_<판>.json, routes/<판>/drift_*.txt(다른 녹화로 위치 찾기).  사용: python3 compare_fig.py"""
import importlib, json, math, os, re, sys
import numpy as np
from scipy.spatial import cKDTree
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
plt.rcParams['font.family'] = 'NanumGothic'; plt.rcParams['axes.unicode_minus'] = False
H = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(H, 'tools'))
V = [('c1650_15', '16:50 · 15 m  (지금 쓰는 지도)'), ('c1645_15', '16:45 · 15 m  원본'), ('c1645_08', '16:45 · 8 m'), ('d1645_15', '16:45 · 15 m  재구성'),
     ('d1645_20', '16:45 · 20 m'), ('d1645_30', '16:45 · 30 m'), ('d1645_15clean', '16:45 · 15 m  사람 지움(1차)')]
T0 = (-0.025, -0.039, math.radians(-3.46))


def walls(n):
    sys.argv = ['x', n]; import route_editor as ed; ed = importlib.reload(ed); c = ed.load_cache(); return ed.real_walls(c), c['path']


def tf(P, T):
    c, s = math.cos(T[2]), math.sin(T[2]); return np.c_[c * P[:, 0] - s * P[:, 1] + T[0], s * P[:, 0] + c * P[:, 1] + T[1]]


def inv(T):
    c, s = math.cos(T[2]), math.sin(T[2]); return (-(c * T[0] + s * T[1]), -(-s * T[0] + c * T[1]), -T[2])


route = json.load(open(os.path.expanduser('~/ws/junk/routes/rec1650/route.json')))['route']; RP = np.array([[e['x'], e['y']] for e in route])
seg = np.vstack([np.linspace(RP[i], RP[i + 1], max(2, int(np.hypot(*(RP[i + 1] - RP[i])) / 0.02))) for i in range(len(RP) - 1)]); TS = cKDTree(seg)
D = {}
for n, lab in V:
    W, path = walls(n); q = json.load(open(os.path.join(H, f'map_quality_{n}.json')))
    T = (0, 0, 0) if n.startswith('c1650') else inv((q['두녹화지도_고정벽']['맞춘변환'][0], q['두녹화지도_고정벽']['맞춘변환'][1], math.radians(q['두녹화지도_고정벽']['맞춘변환'][2])))
    D[n] = dict(lab=lab, W=tf(W, T), path=tf(path, T), q=q, T=T)
    if n.endswith('clean'):
        rm = np.load(os.path.join(H, 'routes', n, 'removed.npy')); D[n]['rm'] = tf(rm[:, :2], T)
REF = {n: ('c1645_15' if n == 'c1650_15' else 'c1650_15') for n, _ in V}
for n, _ in V:
    W = D[n]['W']; near = TS.query(W)[0] < 0.6; other = cKDTree(D[REF[n]]['W']); ghost = near & (other.query(W)[0] > 0.08)
    D[n]['ghost'] = ghost; D[n]['n_near'] = int(near.sum()); D[n]['n_ghost'] = int(ghost.sum())
    D[n]['clear'] = float(np.mean(cKDTree(W).query(seg)[0] < 0.155) * 100)   # 경로 선이 몸 반폭 안으로 벽을 지나는 비율
    dr = [f for f in os.listdir(os.path.join(H, 'routes', n)) if f.startswith('drift_') and f.endswith('.txt')]
    D[n]['drift'] = None
    if dr:
        txt = open(os.path.join(H, 'routes', n, dr[0])).read(); m_ = re.search(r'슬라럼 밖: 위치 오차 중앙 ([\d.]+) · p95 ([\d.]+) · 최대 ([\d.]+)', txt); g_ = re.search(r'시점 (\d+)개, 믿을 수 있는 기준 (\d+)개', txt)
        mi = re.search(r'슬라럼 안: 위치 오차 중앙 ([\d.]+) · p95 ([\d.]+) · 최대 ([\d.]+)', txt)
        if m_ and g_:
            D[n]['drift'] = dict(med=float(m_[1]), p95=float(m_[2]), mx=float(m_[3]), good=int(g_[2]), tot=int(g_[1]), smx=float(mi[3]) if mi else float('nan'))
lo, hi = RP.min(0) - 1.2, RP.max(0) + 1.2
fig, axs = plt.subplots(2, 4, figsize=(24, 13))
for ax, (n, lab) in zip(axs.ravel(), V):
    d = D[n]; W = d['W']; k = np.all((W > lo) & (W < hi), 1)
    ax.scatter(W[k & ~d['ghost'], 0], W[k & ~d['ghost'], 1], s=0.6, c='0.25', linewidths=0)
    if 'rm' in d:
        rm = d['rm']; kr = np.all((rm > lo) & (rm < hi), 1); ax.scatter(rm[kr, 0], rm[kr, 1], s=1.5, c='orange', linewidths=0, label=f'지운 칸 {kr.sum()}')
    ax.scatter(W[k & d['ghost'], 0], W[k & d['ghost'], 1], s=5, c='red', linewidths=0, label=f'경로 0.6 m 안·다른 녹화엔 없음 {d["n_ghost"]}')
    ax.plot(seg[:, 0], seg[:, 1], '-', c='tab:blue', lw=0.8, alpha=0.7); ax.plot(d['path'][:, 0], d['path'][:, 1], '-', c='tab:green', lw=0.5, alpha=0.5)
    q = d['q']; dr = d['drift']
    t2 = f"벽 두께 {q['벽두께']['중앙_cm']} / p90 {q['벽두께']['p90_cm']} cm · 16:50 벽이 이 지도에도 {q['두녹화지도_고정벽']['안5cm비율']:.0f}%" if not n.startswith('c1650') else f"벽 두께 {q['벽두께']['중앙_cm']} / p90 {q['벽두께']['p90_cm']} cm · (기준 지도)"
    t3 = (f"다른 녹화로 위치 찾기: 슬라럼 밖 p95 {dr['p95']:.1f} · 최대 {dr['mx']:.1f} cm · 슬라럼 안 최대 {dr['smx']:.0f} cm" if dr else '다른 녹화로 위치 찾기: (계산 중)')
    ax.set_title(f'{lab}\n{t2}\n{t3}', fontsize=10, loc='left'); ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_aspect('equal'); ax.grid(alpha=0.2)
    ax.legend(fontsize=8, loc='lower right', markerscale=4)
ax = axs.ravel()[7]; ax.axis('off')
rows = [['판', '벽 두께\n중앙/p90', '16:50 벽\n덮음', '경로 옆\n남는 점', '외톨이\n잡음', '위치 찾기\n밖 p95/최대', '슬라럼 안\n최대']]
for n, lab in V:
    d = D[n]; q = d['q']; dr = d['drift']
    rows.append([lab.split('  ')[0] + (' ' + lab.split('  ')[1] if '  ' in lab else ''), f"{q['벽두께']['중앙_cm']}/{q['벽두께']['p90_cm']}", '기준' if n.startswith('c1650') else f"{q['두녹화지도_고정벽']['안5cm비율']:.0f}%",
                 str(d['n_ghost']), str(q['외톨이잡음']['외톨이']), f"{dr['p95']:.1f}/{dr['mx']:.1f}" if dr else '-', f"{dr['smx']:.0f}" if dr else '-'])
tb = ax.table(cellText=rows, loc='center', cellLoc='center', colWidths=[0.32, 0.12, 0.11, 0.1, 0.09, 0.14, 0.1]); tb.auto_set_font_size(False); tb.set_fontsize(9); tb.scale(1, 2.2)
ax.set_title('숫자 (cm) — 경로 옆 남는 점 = 경로 0.6 m 안 벽 칸 중 다른 녹화 지도엔 없는 것(사람 흔적 후보)\n위치 찾기 = 다른 녹화(16:50 지도는 16:45, 나머지는 16:50)를 이 지도에 ICP, FAST-LIO 대비', fontsize=9, loc='left')
fig.suptitle('지도 판 비교 (16:50 좌표로 맞춤) — 회색 벽 · 파랑 대회 경로 · 초록 그 녹화의 실제 길 · 빨강 경로 옆 남는 점 · 주황 지운 칸', fontsize=13)
plt.tight_layout(); out = os.path.join(H, 'map_compare.png'); plt.savefig(out, dpi=90); print(out)
json.dump({n: dict(n_ghost=D[n]['n_ghost'], n_near=D[n]['n_near'], clear=D[n]['clear'], drift=D[n]['drift']) for n, _ in V}, open(os.path.join(H, 'compare_summary.json'), 'w'), ensure_ascii=False, indent=1)
for n, lab in V:
    print(f"{lab:32s} 경로옆 남는점 {D[n]['n_ghost']:4d}/{D[n]['n_near']} · 경로선 몸반폭 안 {D[n]['clear']:.1f}% · 위치찾기 {D[n]['drift']}")
