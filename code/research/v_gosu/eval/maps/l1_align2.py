# L1(base_link) → FAST-LIO 몸체 보정 4개: 시각 dt, yaw, pitch, roll
#   dt·yaw : L1 벽 점이 XT16 벽 칸에 떨어지는 비율(최대화)
#   pitch·roll : 로봇 2.5 m 안 바닥 점의 높이 퍼짐(XT16 바닥 평면 기준, MAD 최소화) — 자세가 틀리면 방향마다 바닥이 기울어 퍼진다
import numpy as np, bisect, json
exec(open('render_1030.py').read().split('# ── 3. L1')[0])
from scipy.ndimage import binary_dilation
W = Px[(hx > 0.3) & (hx < 1.5)]; C = 0.05
x0, y0 = W[:, 0].min() - 1, W[:, 1].min() - 1
nx, ny = int((W[:, 0].max() + 1 - x0) / C) + 1, int((W[:, 1].max() + 1 - y0) / C) + 1
occ = np.zeros((ny, nx), bool); occ[((W[:, 1] - y0) / C).astype(int), ((W[:, 0] - x0) / C).astype(int)] = True
occ = binary_dilation(occ, iterations=1)
frames = []; rng = np.random.default_rng(0); r = reader(['/utlidar/cloud_base']); k = 0
while r.has_next():
    tp, raw, t = r.read_next(); k += 1
    if k % 4: continue
    m = deserialize_message(raw, PointCloud2)
    a = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)
    a = a[~((np.abs(a[:, 0]) < 0.45) & (np.abs(a[:, 1]) < 0.30))]; a = a[rng.random(len(a)) < 0.3]
    frames.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 + robot_off, a))
fl = coef_x
def Rxyz(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
def world(dt, R):
    for ts, a in frames:
        ts += dt; i = bisect.bisect_left(t_pose, ts)
        if i <= 0 or i >= len(t_pose): continue
        j = i if (t_pose[i] - ts) < (ts - t_pose[i - 1]) else i - 1
        yield (a @ R.T) @ q2R(*Q[j, 5:9]).T + Q[j, 2:5], Q[j, 2:5]
def wall_score(dt, R):
    hit = tot = 0
    for w, _ in world(dt, R):
        h = w[:, 2] - (fl[0] * w[:, 0] + fl[1] * w[:, 1] + fl[2]); w = w[(h > 0.2) & (h < 1.6)]
        ix = ((w[:, 0] - x0) / C).astype(int); iy = ((w[:, 1] - y0) / C).astype(int)
        ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny); hit += occ[iy[ok], ix[ok]].sum(); tot += len(w)
    return hit / max(tot, 1)
def floor_mad(dt, R):
    hs = []
    for w, c in world(dt, R):
        h = w[:, 2] - (fl[0] * w[:, 0] + fl[1] * w[:, 1] + fl[2])
        near = np.hypot(w[:, 0] - c[0], w[:, 1] - c[1]) < 2.5
        hs.append(h[near & (np.abs(h) < 0.25)])
    h = np.concatenate(hs); med = np.median(h); h = h[np.abs(h - med) < 0.12]
    return float(np.median(np.abs(h - np.median(h)))), float(np.median(h)), len(h)
D = np.radians
dt, yaw = -0.18, -2.0
base = floor_mad(dt, Rxyz(0, 0, D(yaw)))
best = None
for p in np.arange(-3, 3.01, 0.5):
    for rl in np.arange(-3, 3.01, 0.5):
        v = floor_mad(dt, Rxyz(D(rl), D(p), D(yaw)))[0]
        if best is None or v < best[0]: best = (v, p, rl)
_, p0, r0 = best
for p in np.arange(p0 - 0.4, p0 + 0.41, 0.1):
    for rl in np.arange(r0 - 0.4, r0 + 0.41, 0.1):
        v = floor_mad(dt, Rxyz(D(rl), D(p), D(yaw)))[0]
        if v < best[0]: best = (v, round(p, 2), round(rl, 2))
_, pb, rb = best
# pitch·roll 고정 후 dt·yaw 다시 다듬기
res = {}
for d in np.arange(dt - 0.04, dt + 0.041, 0.01):
    for y in np.arange(yaw - 0.6, yaw + 0.61, 0.2):
        res[(round(d, 3), round(y, 2))] = wall_score(d, Rxyz(D(rb), D(pb), D(y)))
(db, yb) = max(res, key=res.get)
fin = floor_mad(db, Rxyz(D(rb), D(pb), D(yb)))
out = dict(dt=db, yaw_deg=yb, pitch_deg=pb, roll_deg=rb, wall_score=res[(db, yb)], wall_score0=wall_score(0, np.eye(3)),
           floor_mad_mm=round(fin[0] * 1000, 1), floor_mad0_mm=round(floor_mad(0, np.eye(3))[0] * 1000, 1),
           floor_mad_dtyaw_only_mm=round(base[0] * 1000, 1), floor_offset_m=round(fin[1], 3), frames=len(frames))
json.dump(out, open('l1_align_1030.json', 'w'), ensure_ascii=False); print(out)
