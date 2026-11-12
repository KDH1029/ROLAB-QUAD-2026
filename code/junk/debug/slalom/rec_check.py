import json, math, os, sys
H = os.path.dirname(os.path.abspath(__file__)); sys.argv = [sys.argv[0], '6', '26']
src = open(os.path.join(H, 'best_line.py')).read(); exec(compile(src[:src.index('free = np.where')], 'h', 'exec'))
od = d['odom']; od = od[np.argsort(od[:, 1])]; x, y = od[:, 2], od[:, 3]; q = od[:, 5:9]
yaw = np.arctan2(2 * (q[:, 3] * q[:, 2] + q[:, 0] * q[:, 1]), 1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2))
t = od[:, 1] - od[0, 1]; z = (x > ZLO[0]) & (x < ZHI[0]) & (y > ZLO[1]) & (y < ZHI[1]); dt = np.r_[np.diff(t), 0.05]
g = np.array([sgap(np.array([a, b]), c) for a, b, c in zip(x[z], y[z], yaw[z])])
mv = np.r_[np.hypot(np.diff(x), np.diff(y)), 0][z]
print(f'녹화 16:50(사람이 리모컨, 콘 안 쓰러뜨림): 슬라럼 안 {dt[z].sum():.0f}초 · 같은 판정으로 몸 겹침 {np.sum(dt[z][g < 0]):.1f}초 ({np.mean(g < 0) * 100:.0f}%) · 최소 여유 {g.min() * 100:+.1f} · p5 {np.percentile(g, 5) * 100:+.1f} · 중앙 {np.median(g) * 100:+.1f} cm')
Cc = np.load(os.path.join(H, 'decompose_curve.npy'))[:, :2]; dv = cKDTree(Cc).query(np.c_[x[z], y[z]])[0]
print(f'  녹화 길 ↔ 그린 선(러너 곡선) 거리: 중앙 {np.median(dv) * 100:.0f} · p95 {np.percentile(dv, 95) * 100:.0f} · 최대 {dv.max() * 100:.0f} cm')
# 녹화 길에서 몸 방향 vs 진행 방향
mvdir = np.arctan2(np.gradient(y[z]), np.gradient(x[z])); dd = np.degrees((yaw[z] - mvdir + np.pi) % (2 * np.pi) - np.pi); ok = np.r_[mv > 0.002]
print(f'  녹화 때 몸 방향 − 진행 방향: 중앙 |{np.median(np.abs(dd[ok])):.0f}°| · p90 {np.percentile(np.abs(dd[ok]), 90):.0f}° (사람이 옆걸음 섞었는지)')
np.save(os.path.join(H, 'rec_gap.npy'), np.c_[x[z], y[z], yaw[z], g])
