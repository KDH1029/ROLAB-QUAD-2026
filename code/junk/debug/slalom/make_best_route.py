"""best_line_tangent.npy(가장 넓게 비켜 가는 선)로 #7~#25 자리만 옮긴 후보 경로 — 점 개수·라벨·순서 그대로(슬라럼 구역 정의가 안 바뀜). 원본 route.json 은 안 건드림."""
import json, os, numpy as np
H = os.path.dirname(os.path.abspath(__file__)); J = os.path.expanduser('~/ws/junk')
r = json.load(open(os.path.join(J, 'routes/rec1650/route.json'))); C = np.load(os.path.join(H, 'decompose_curve.npy'))[:, :2]
S = np.r_[0, np.cumsum(np.hypot(*np.diff(C, axis=0).T))]; st = np.arange(0, S[-1], 0.05); B = np.c_[np.interp(st, S, C[:, 0]), np.interp(st, S, C[:, 1])]
P = np.load(os.path.join(H, 'best_line_tangent.npy'))[:, :2]; last = 0
for i in range(6, 25):
    e = r['route'][i]; k = last + int(np.argmin(np.hypot(B[last:last + 40, 0] - e['x'], B[last:last + 40, 1] - e['y']))); last = k
    print(f"#{i + 1} ({e['x']:.2f},{e['y']:.2f}) → ({P[k, 0]:.2f},{P[k, 1]:.2f})  {np.hypot(P[k, 0] - e['x'], P[k, 1] - e['y']) * 100:.0f} cm")
    e['x'], e['y'] = round(float(P[k, 0]), 3), round(float(P[k, 1]), 3)
json.dump(r, open(os.path.join(J, 'debug/slalom/route_slalom_best.json'), 'w'), ensure_ascii=False, indent=1)
