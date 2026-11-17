import json, math, os, sys
import numpy as np
from scipy.spatial import cKDTree
H = os.path.dirname(os.path.abspath(__file__)); sys.argv, _a = ['x'], sys.argv
exec(open(os.path.join(H, 'sim_full.py')).read().split("DEFAULTS = {")[0]); sys.argv = _a
# 전체 코스 벽(관측≥8, 편집기 규칙) — rec1650, 슬라럼 구역은 rec1645 c8
d = np.load(os.path.join(J, 'routes', 'rec1650', 'map.npz')); P = (d['vox'][d['cnt'] >= 8] + 0.5) * float(d['V']); tr = d['odom'][:, 2:4]
kk = np.floor(P[:, :2] / 0.5).astype(np.int64); u, inv = np.unique(kk, axis=0, return_inverse=True); inv = inv.ravel()
fl = np.full(len(u), np.inf); np.minimum.at(fl, inv, P[:, 2]); h = P[:, 2] - fl[inv]; q = (h > -0.15) & (h < 1.5); Q, hq = P[q, :2], h[q]
key = np.floor(Q / 0.04).astype(np.int64); u2, i2 = np.unique(key, axis=0, return_inverse=True); i2 = i2.ravel()
zmin = np.full(len(u2), 9.0); zmax = np.full(len(u2), -9.0); np.minimum.at(zmin, i2, hq); np.maximum.at(zmax, i2, hq)
W8 = (u2[(zmax - zmin) >= 0.25] + 0.5) * 0.04; W8 = W8[cKDTree(tr).query(W8)[0] >= 0.18]
W8 = np.r_[W8[~np.all((W8 > zl) & (W8 < zh), 1)], np.load(os.path.join(J, 'debug', 'cones1645', 'zone_1645_c8.npy'))]
RP = np.array([[e['x'], e['y']] for e in R]); TR = cKDTree(RP)
def hits(OBx, xs):
    T_ = cKDTree(OBx); out = []
    for x, y, a in xs:
        idx = T_.query_ball_point([x, y], 0.45)
        if not idx: continue
        qq = OBx[idx] - [x, y]; lx = qq[:, 0] * math.cos(a) + qq[:, 1] * math.sin(a); ly = -qq[:, 0] * math.sin(a) + qq[:, 1] * math.cos(a)
        if np.any((np.abs(lx) < 0.35) & (np.abs(ly) < 0.155)): out.append((x, y))
    return np.array(out)
# 사람 조종 16:50 주행(전 코스)
od = d['odom']; od = od[np.argsort(od[:, 1])]; qq = od[:, 5:9]; yaw = np.arctan2(2 * (qq[:, 3] * qq[:, 2] + qq[:, 0] * qq[:, 1]), 1 - 2 * (qq[:, 1] ** 2 + qq[:, 2] ** 2))
hum = np.c_[od[:, 2], od[:, 3], yaw]; dth = np.median(np.diff(od[:, 1]))
# 시뮬 궤적: sim_full 결정판(속도 0.30) — run() 이 궤적을 안 돌려주니 러너 tick 감싸서 기록
import real_course_runner as r2
trace = []; orig = r2.RealCourseRunner.tick
def tk(self):
    orig(self)
    if self.pos is not None: trace.append((self.pos[0], self.pos[1], self.yaw or 0.0))
r2.RealCourseRunner.tick = tk
rclpy.init(); exec(open(os.path.join(H, 'sim_full.py')).read().split("if __name__ == '__main__':")[0].split("DEFAULTS = {")[1].join(['DEFAULTS = {', ''])) if False else None
src = open(os.path.join(H, 'sim_full.py')).read(); g = dict(globals()); exec(src.split("if __name__ == '__main__':")[0], g)
g['run']({}, 1.0); sim = np.array(trace)
for nm, OBx in (('편집기 벽(관측≥2, 지금 시뮬)', OB), ('관측≥8 벽', W8)):
    for who, xs, dtt in (('시뮬 0.30', sim, 0.2), ('사람 16:50', hum, dth)):
        hh = hits(OBx, xs)
        if len(hh) == 0: print(f'{nm} · {who}: 닿음 0'); continue
        near = TR.query(hh)[1] + 1; u_, c_ = np.unique(near, return_counts=True); top = sorted(zip(c_, u_), reverse=True)[:6]
        print(f'{nm} · {who}: 닿음 {len(hh) * dtt:.1f} s · 가까운 점(번호: 초) ' + ', '.join(f'#{p}:{c * dtt:.1f}' for c, p in top))
