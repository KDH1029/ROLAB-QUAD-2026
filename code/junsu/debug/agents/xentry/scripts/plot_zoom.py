import json, sys, os
import numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
plt.rcParams['font.family'] = 'NanumGothic'; plt.rcParams['axes.unicode_minus'] = False
S = os.path.expanduser('~/ws/junsu_v2/sim')
W = np.load(S + '/sim_walls_rec1650.npy'); Wt = np.load(S + '/sim_walls_rec1650_thin.npy')
print('walls', W.shape, 'thin', Wt.shape)
xs = np.unique(np.round(W[:,0],3)); print('x spacing', np.diff(xs)[:10])
x0, x1, y0, y1 = 0.3, 2.8, -0.5, 1.8
fig, ax = plt.subplots(figsize=(12, 9))
for arr, c, lab in ((W, 'k', '시뮬 벽(기본)'), (Wt, 'r', '얇은 벽(thin)')):
    m = (arr[:, 0] > x0) & (arr[:, 0] < x1) & (arr[:, 1] > y0) & (arr[:, 1] < y1)
    ax.scatter(arr[m, 0], arr[m, 1], s=12, c=c, label=lab)
ax.set_xticks(np.arange(x0, x1, 0.1), minor=True); ax.set_yticks(np.arange(y0, y1, 0.1), minor=True)
ax.grid(which='both', alpha=.25); ax.set_aspect('equal'); ax.legend()
for lab, p in (('line1 main', (1.837, -0.134)), ('line1 v2', (1.449, -0.103)), ('line2 v2', (1.909, 0.22)), ('line3', (2.866, 0.767))):
    ax.plot(*p, 'bx', ms=12, mew=2); ax.annotate(lab, p)
fig.savefig(sys.argv[1], dpi=100, bbox_inches='tight')
