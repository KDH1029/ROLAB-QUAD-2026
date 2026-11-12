import json, math, os, sys
import numpy as np
import matplotlib; matplotlib.use('Agg'); matplotlib.rcParams['font.family'] = 'NanumGothic'; matplotlib.rcParams['axes.unicode_minus'] = False
import matplotlib.pyplot as plt
H = os.path.dirname(os.path.abspath(__file__)); sys.argv = ['x']; exec(open(os.path.join(H, 'cones.py')).read().split('res = {}')[0])
f, ax = plt.subplots(1, 2, figsize=(14, 13), sharex=True, sharey=True)
for a, (name, to) in zip(ax, (('rec1645', True), ('rec1650', False))):
    d = np.load(os.path.join(J, 'routes', name, 'map.npz')); P = (d['vox'][d['cnt'] >= 3] + 0.5) * float(d['V']); cnt = d['cnt'][d['cnt'] >= 3]
    if to: P = np.c_[apply(T, P[:, :2]), P[:, 2]]
    key = np.floor(P[:, :2] / 0.3).astype(np.int64); u, inv = np.unique(key, axis=0, return_inverse=True); inv = inv.ravel()
    order = np.argsort(inv); s = np.split(P[order, 2], np.cumsum(np.bincount(inv))[:-1]); fl = np.array([np.percentile(z, 3) for z in s]); h = P[:, 2] - fl[inv]
    z = np.all((P[:, :2] > ZLO - 0.3) & (P[:, :2] < ZHI + 0.3), 1)
    lo = z & (h > 0.05) & (h < 0.45); hi = z & (h > 0.60) & (h < 0.85); top = z & (h > 0.95) & (h < 1.6)
    a.scatter(P[lo, 0], P[lo, 1], s=0.2, c='0.6', label='바닥 위 5~45 cm')
    a.scatter(P[hi, 0], P[hi, 1], s=0.5, c='tab:orange', label='60~85 cm (콘 꼭대기·벽)')
    a.scatter(P[top, 0], P[top, 1], s=0.5, c='tab:purple', label='95~160 cm (벽/사람만)')
    _, tr = load(name, to); k = np.all((tr[:, :2] > ZLO - 0.3) & (tr[:, :2] < ZHI + 0.3), 1); a.plot(tr[k, 0], tr[k, 1], 'g-', lw=1, label=f'{name[3:]} 리모컨 주행')
    a.plot(RP[5:26, 0], RP[5:26, 1], 'b.--', lw=0.8, ms=4, label='경로 rec1650')
    a.set_title(name + (' (rec1650 좌표로 옮김)' if to else '')); a.set_aspect('equal'); a.grid(alpha=.3); a.legend(fontsize=7, loc='lower left')
    a.set_xticks(np.arange(5.4, 7.8, 0.2)); a.set_yticks(np.arange(-1.6, 4.0, 0.2)); a.tick_params(labelsize=6)
ax[0].set_xlim(ZLO[0] - 0.3, ZHI[0] + 0.3); ax[0].set_ylim(ZLO[1] - 0.3, ZHI[1] + 0.3)
f.savefig(os.path.join(H, 'zone_view.png'), dpi=70, bbox_inches='tight')
