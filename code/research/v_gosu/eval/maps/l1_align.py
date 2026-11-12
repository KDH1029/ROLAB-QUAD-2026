# L1(base_link) → FAST-LIO 몸체 사이의 yaw 오차와 시각 오차를 XT16 벽에 맞춰 격자 탐색한다.
# 점수 = L1 벽 점(바닥 위 0.3~1.5 m)이 XT16 벽 칸(5 cm, 1칸 팽창)에 떨어지는 비율
import numpy as np, bisect, json
exec(open('render_1030.py').read().split('# ── 3. L1')[0])          # 1~2단계(XT16 점·바닥·자세 시각) 재사용
from scipy.ndimage import binary_dilation
W = Px[(hx > 0.3) & (hx < 1.5)]; C = 0.05
x0, y0 = W[:, 0].min() - 1, W[:, 1].min() - 1
nx, ny = int((W[:, 0].max() + 1 - x0) / C) + 1, int((W[:, 1].max() + 1 - y0) / C) + 1
occ = np.zeros((ny, nx), bool); occ[((W[:, 1] - y0) / C).astype(int), ((W[:, 0] - x0) / C).astype(int)] = True
occ = binary_dilation(occ, iterations=1)
frames = []; rng = np.random.default_rng(0)
r = reader(['/utlidar/cloud_base']); k = 0
while r.has_next():
    tp, raw, t = r.read_next(); k += 1
    if k % 4: continue
    m = deserialize_message(raw, PointCloud2)
    a = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)
    a = a[~((np.abs(a[:, 0]) < 0.45) & (np.abs(a[:, 1]) < 0.30))]
    a = a[rng.random(len(a)) < 0.3]
    frames.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 + robot_off, a))
fl = coef_x
def score(dyaw, dt):
    c, s = np.cos(dyaw), np.sin(dyaw); Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    hit = tot = 0
    for ts, a in frames:
        ts += dt; i = bisect.bisect_left(t_pose, ts)
        if i <= 0 or i >= len(t_pose): continue
        f = (ts - t_pose[i - 1]) / max(t_pose[i] - t_pose[i - 1], 1e-6); j = i - 1 if f < 0.5 else i
        w = (a @ Rz.T) @ q2R(*Q[j, 5:9]).T + Q[j, 2:5]
        h = w[:, 2] - (fl[0] * w[:, 0] + fl[1] * w[:, 1] + fl[2])
        w = w[(h > 0.2) & (h < 1.6)]
        ix = ((w[:, 0] - x0) / C).astype(int); iy = ((w[:, 1] - y0) / C).astype(int)
        ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
        hit += occ[iy[ok], ix[ok]].sum(); tot += len(w)
    return hit / max(tot, 1)
res = {}
for dt in np.arange(-0.40, 0.101, 0.05):
    for dy in np.radians(np.arange(-4, 4.01, 1.0)):
        res[(round(dt, 3), round(np.degrees(dy), 1))] = score(dy, dt)
best = max(res, key=res.get); print('거친 탐색 최고', best, round(res[best], 4), '  (0,0)', round(res[(0.0, 0.0)], 4))
dt0, dy0 = best
for dt in np.arange(dt0 - 0.05, dt0 + 0.051, 0.01):
    for dy in np.arange(dy0 - 0.8, dy0 + 0.81, 0.2):
        res[(round(dt, 3), round(dy, 1))] = score(np.radians(dy), dt)
best = max(res, key=res.get); print('세밀 탐색 최고 (dt s, yaw deg)', best, round(res[best], 4))
json.dump(dict(dt=best[0], yaw_deg=best[1], score=res[best], score0=res[(0.0, 0.0)], frames=len(frames)), open('l1_align_1030.json', 'w'), ensure_ascii=False)
