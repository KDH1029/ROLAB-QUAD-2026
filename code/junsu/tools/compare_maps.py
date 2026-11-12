#!/usr/bin/env python3
"""여러 녹화로 만든 FAST-LIO 지도(map_collect npz) 품질 비교 — 벽이 얼마나 날카로운가(여러 겹이면 두꺼워진다).
사용: compare_maps.py <이름=npz> [<이름=npz> ...]  → 표 + <첫 npz 폴더>/compare_maps.png
벽 날카로움: 세로 벽 기둥(바닥 위 10~25 cm 와 40~55 cm 에 둘 다 점) 칸마다 반경 15 cm 이웃의 2D 퍼짐에서 '가장 얇은 방향' 표준편차.
  얇은 판벽 하나면 1~2 cm, 두세 겹이면 4~8 cm 이상. (모서리·구조물도 섞이므로 절대값보다 지도끼리 비교용)"""
import sys, os, json, numpy as np
from scipy.spatial import cKDTree
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
plt.rcParams['font.family'] = ['NanumGothic', 'DejaVu Sans']
args = [a.split('=', 1) for a in sys.argv[1:]]
rows, views = [], []
rng = np.random.default_rng(0)
for name, f in args:
    d = np.load(os.path.expanduser(f)); V = float(d['V']); P = (d['vox'][d['cnt'] >= 2] + 0.5) * V
    od = d['odom']; od = od[np.argsort(od[:, 1])]; xy = od[:, 2:4]; z = od[:, 4]
    floor = np.percentile(z, 50) - 0.31
    lo, hi = xy.min(0) - 1.5, xy.max(0) + 1.5
    A = P[(P[:, 0] > lo[0]) & (P[:, 0] < hi[0]) & (P[:, 1] > lo[1]) & (P[:, 1] < hi[1])]
    # 세로 벽 기둥만: 같은 2 cm 칸에 바닥 위 10~25 cm 점과 40~55 cm 점이 둘 다 있어야 벽(경사로·계단 턱·바닥은 빠짐)
    h = A[:, 2] - floor; k2 = np.round(A[:, :2] / 0.02).astype(np.int64); key = k2[:, 0] * 1000003 + k2[:, 1]
    low = set(key[(h > 0.10) & (h < 0.25)].tolist()); high = set(key[(h > 0.40) & (h < 0.55)].tolist()); both = np.array(sorted(low & high), dtype=np.int64)
    W = np.c_[np.floor_divide(both + 500000, 1000003), both - np.floor_divide(both + 500000, 1000003) * 1000003].astype(float) * 0.02
    T = cKDTree(W); S = W[rng.choice(len(W), min(15000, len(W)), replace=False)]
    sig = []
    for p, nb in zip(S, T.query_ball_point(S, 0.15)):
        if len(nb) < 8: continue
        Q = W[nb] - W[nb].mean(0); ev = np.linalg.eigvalsh(Q.T @ Q / len(nb)); sig.append(np.sqrt(max(ev[0], 0)))
    sig = np.array(sig)
    step = np.hypot(*np.diff(xy, axis=0).T)
    js = {}
    zf = os.path.expanduser(f).replace('_map.npz', '').replace('/v_gosu/eval/out/', '/night_20261104/maps/') + '_zoom.json'
    if os.path.exists(zf): js = json.load(open(zf))
    rows.append({'지도': name, '경로_m': round(float(step.sum()), 1), '시간_s': round(float(od[-1, 1] - od[0, 1])), '출발↔끝_m': round(float(np.hypot(*(xy[-1] - xy[0]))), 2),
                 '로봇높이폭_m': round(float(np.percentile(z, 95) - np.percentile(z, 5)), 2), '벽두께σ_중앙_cm': round(float(np.median(sig)) * 100, 2),
                 '벽칸': len(W), '얇은벽(σ<2cm)%': round(float((sig < 0.02).mean()) * 100, 1), '두꺼운벽(σ>4cm)%': round(float((sig > 0.04).mean()) * 100, 1),
                 '바닥기울기°': js.get('바닥기울기_deg'), '바닥두께_mm': js.get('바닥두께_평면뺀_중앙_mm')})
    c = 0.04; m = (A[:, 2] > floor - 0.1) & (A[:, 2] < floor + 1.2); Qp = A[m]; k = np.floor((Qp[:, :2] - lo) / c).astype(int); Wd, Hd = k.max(0) + 1
    img = np.full((Wd, Hd), np.nan); np.fmax.at(img, (k[:, 0], k[:, 1]), Qp[:, 2] - floor)
    views.append((name, img, (lo[0], lo[0] + Wd * c, lo[1], lo[1] + Hd * c), xy))
keys = list(rows[0].keys())
print(' | '.join(keys)); [print(' | '.join(str(r[k]) for k in keys)) for r in rows]
fig, axs = plt.subplots(1, len(views), figsize=(7 * len(views), 7))
for ax, (name, img, ext, xy), r in zip(np.atleast_1d(axs), views, rows):
    ax.imshow(img.T, origin='lower', extent=ext, cmap='turbo', vmin=-0.05, vmax=0.8, interpolation='nearest'); ax.plot(xy[:, 0], xy[:, 1], 'w-', lw=0.5, alpha=0.7)
    ax.set_aspect('equal'); ax.set_title(f"{name} — 벽두께σ {r['벽두께σ_중앙_cm']} cm · 얇은벽 {r['얇은벽(σ<2cm)%']}%", fontsize=11)
plt.tight_layout(); out = os.path.join(os.path.dirname(os.path.expanduser(args[0][1])), 'compare_maps.png'); plt.savefig(out, dpi=65); print('그림', out)
