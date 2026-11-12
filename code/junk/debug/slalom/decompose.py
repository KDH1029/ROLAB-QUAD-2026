#!/usr/bin/env python3
"""슬라럼 접촉 원인 나누기 — ① 그린 선을 '완벽하게' 따라가도(몸 중심 = 러너 곡선, 몸 방향 = 접선) 닿는가 ② 시뮬 궤적의 접촉 중 ①이 아닌 것(따라가기 오차 몫)
사용(ROS Jazzy 환경): python3 decompose.py"""
import json, math, os, sys
H = os.path.dirname(os.path.abspath(__file__))
sys.argv = [sys.argv[0], '6', '26']
src = open(os.path.join(H, 'sim_scurve.py')).read()
exec(compile(src[:src.index('rclpy.init()')], 'sim_scurve_head', 'exec'))
rclpy.init()
clock = Clock(); rcr.time.monotonic = clock.monotonic; rcr.HEADING_TANGENT = True
f = tempfile.NamedTemporaryFile('w', suffix='.json', delete=False); json.dump({'format': 'route_v1', 'route': route}, f); f.close()
rcr.WAYPOINTS_PATH = f.name; rcr.ALIGN_T = (0.0, 0.0, 0.0); s0, goals = rcr.load_waypoints()
n = rcr.RealCourseRunner(s0, goals); n.pub_cmd_vel.publish = lambda m: None; n.pub_goal.publish = lambda m: None; n.pub_req.publish = lambda m: None; n.pub_camera_active.publish = lambda m: None
m = Odometry(); m.pose.pose.position.x, m.pose.pose.position.y = route[0]['x'], route[0]['y']; a0 = math.atan2(route[1]['y'] - route[0]['y'], route[1]['x'] - route[0]['x'])
m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(a0 / 2), math.cos(a0 / 2); n.cb_odom(m)
s = SportModeState(); s.body_height = 0.30; n.cb_state(s); n.tick(); n.tick()
C = np.asarray(n.pin_path); S = np.r_[0, np.cumsum(np.hypot(*np.diff(C, axis=0).T))]
inz = lambda P: (P[:, 0] > ZLO[0]) & (P[:, 0] < ZHI[0]) & (P[:, 1] > ZLO[1]) & (P[:, 1] < ZHI[1])
kz = inz(C)
# ① 완벽 추종
hd = np.array([math.atan2(*(C[min(k + 12, len(C) - 1)] - C[k])[::-1]) if k < len(C) - 1 else 0 for k in range(len(C))])
ideal = np.array([body_hits(C[k, 0], C[k, 1], hd[k]) for k in range(len(C))])
cl = TO.query(C)[0]
ds = np.r_[np.diff(S), 0]
print(f'러너 곡선 슬라럼 구간 {S[kz][-1] - S[kz][0]:.2f} m · 몸 0.70×0.31')
print(f'① 선 위 완벽 추종(방향=접선): 몸이 콘·벽과 겹치는 길이 {np.sum(ds[kz & (ideal[:, 0] > 0)]):.2f} m ({np.sum(ds[kz & (ideal[:, 0] > 0)]) / np.sum(ds[kz]) * 100:.0f}%) · 몸-장애물 최소 여유 {ideal[kz, 1].min() * 100:.1f} cm')
print(f'   중심선-장애물 거리: 최소 {cl[kz].min() * 100:.1f} · p5 {np.percentile(cl[kz], 5) * 100:.1f} · 중앙 {np.median(cl[kz]) * 100:.1f} cm · 몸 반폭 15.5 cm 안 {np.mean(cl[kz] < 0.155) * 100:.0f}% · 20 cm 안 {np.mean(cl[kz] < 0.20) * 100:.0f}%')
# 찍은 점별 중심 여유
for i, e in enumerate(route):
    d0 = TO.query([e['x'], e['y']])[0]
    if d0 < 0.22:
        print(f'   찍은 점 #{i + 6} ({e["x"]:.2f},{e["y"]:.2f}) 장애물까지 {d0 * 100:.0f} cm')
# ② 시뮬 궤적(V2 = 지금 설정과 같은 접선 방향)
for v in (() if os.environ.get('SIM_TAG') else ('V0', 'V2')):
    Tr = np.load(os.path.join(H, f'sim_scurve_{v}_trace.npy')); z = inz(Tr[:, 1:3]); dt = np.r_[np.diff(Tr[:, 0]), 0.05]
    kk = cKDTree(C).query(Tr[:, 1:3])[1]; dev = cKDTree(C).query(Tr[:, 1:3])[0]
    hit = z & (Tr[:, 4] > 0); geo = hit & (ideal[kk, 0] > 0)
    print(f'② 시뮬 {v}: 접촉 {np.sum(dt[hit]):.1f}초 중 선 자체가 닿는 자리 {np.sum(dt[geo]):.1f}초 · 선은 안 닿는데 벗어나서 닿음 {np.sum(dt[hit & ~geo]):.1f}초 · 선에서 벗어남 p95 {np.percentile(dev[z], 95) * 100:.0f} cm · 최대 {dev[z].max() * 100:.0f} cm')
np.save(os.path.join(H, f'decompose_curve{os.environ.get("SIM_TAG", "")}.npy'), np.c_[C, hd, ideal, cl])
os.unlink(f.name)
