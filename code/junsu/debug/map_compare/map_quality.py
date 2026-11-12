#!/usr/bin/env python3
"""지도 품질 점검 — routes/<이름>/map.npz(2 cm 복셀, FAST-LIO) · map.pgm. 경로 둘레(2 m)만.
① 벽 두께: 0.5 m 칸마다 바닥 위 0.2~0.6 m 벽 점을 직선(주성분)으로 맞춰 수직 방향 폭(p10~p90) — 진짜 평면이면 2 cm 대가 정상(센서+정합 예산 21 mm)
② 두 겹 벽: 같은 칸에서 수직 방향 분포가 5 cm 넘게 떨어진 두 봉우리
③ 바닥: 경로 둘레 바닥 점 평면 맞춤 → 기울기 · 잔차(p95)
④ 두 녹화 지도끼리: 다른 녹화 지도(rec1645)를 3D ICP 로 맞춘 뒤 벽 점 거리(중앙·p95)
⑤ 외톨이 잡음: 편집기 벽 칸 중 0.12 m 안에 다른 벽 칸이 2개 미만이고 관측이 적은 것(경로 0.5 m 안)
⑥ 2D 지도: 3D 벽 칸 중 pgm 에서 점유(검정)로 나온 비율, 경로 둘레 점유 칸 중 3D 벽 근거가 없는 비율
사용: ~/venv/go2/bin/python3 map_quality.py [rec1650] [비교=rec1645]"""
import json
import math
import os
import sys

import numpy as np
from scipy.spatial import cKDTree

NAME = sys.argv[1] if len(sys.argv) > 1 else 'rec1650'; OTHER = sys.argv[2] if len(sys.argv) > 2 else 'rec1645'
H = os.path.dirname(os.path.abspath(__file__)); J = H; R = os.path.join(J, 'routes', NAME)   # junsu_v2/debug/map_compare 사본: 지도는 여기 routes/ 에
sys.path.insert(0, os.path.expanduser('~/ws/junk/debug/loc')); import icp_core as ic  # noqa: E402
sys.argv = ['x', NAME]; sys.path.insert(0, os.path.join(J, 'tools')); import route_editor as ed  # noqa: E402
d = np.load(os.path.join(R, 'map.npz')); V = float(d['V']); P = (d['vox'] + 0.5) * V; cnt = d['cnt']
cache = ed.load_cache(); path = cache['path']
RP = np.array([[e['x'], e['y']] for e in json.load(open(os.path.join(R, 'route.json')))['route']]) if os.path.exists(os.path.join(R, 'route.json')) else path[::20]
lo, hi = np.minimum(path.min(0), RP.min(0)) - 2.0, np.maximum(path.max(0), RP.max(0)) + 2.0
k = np.all((P[:, :2] > lo) & (P[:, :2] < hi), 1); P, cnt = P[k], cnt[k]
key = np.floor(P[:, :2] / 0.5).astype(int); u, inv = np.unique(key, axis=0, return_inverse=True); inv = inv.ravel()
fl = np.full(len(u), np.inf); np.minimum.at(fl, inv, P[:, 2]); h = P[:, 2] - fl[inv]
out = {}
# ① ② 벽 두께
Wp = P[(h > 0.2) & (h < 0.6) & (cnt >= 2)]
cell = np.floor(Wp[:, :2] / 0.5).astype(int); cu, ci = np.unique(cell, axis=0, return_inverse=True); ci = ci.ravel()
thick, double = [], 0
for i in range(len(cu)):
    Q = Wp[ci == i, :2]
    if len(Q) < 60:
        continue
    c = Q.mean(0); X = Q - c; ev, evec = np.linalg.eigh(X.T @ X / len(X))
    if ev[1] < 25 * ev[0] or math.sqrt(ev[1]) < 0.10:        # 길쭉한(직선) 칸만 = 평면 벽
        continue
    n = evec[:, 0]; s = X @ n; w = np.percentile(s, 90) - np.percentile(s, 10); thick.append(w)
    hist, edges = np.histogram(s, bins=np.arange(s.min(), s.max() + 0.02, 0.02))
    pk = [j for j in range(1, len(hist) - 1) if hist[j] >= hist[j - 1] and hist[j] >= hist[j + 1] and hist[j] > 0.15 * hist.max()]
    if len(pk) >= 2 and (edges[pk[-1]] - edges[pk[0]]) > 0.05:
        double += 1
thick = np.array(thick)
out['벽두께'] = dict(평면칸=int(len(thick)), 중앙_cm=round(float(np.median(thick)) * 100, 1), p90_cm=round(float(np.percentile(thick, 90)) * 100, 1), 두겹칸=int(double))
# ③ 바닥
F = P[(h < 0.03) & (cnt >= 2)]; F = F[cKDTree(path).query(F[:, :2])[0] < 0.6]
rng = np.random.default_rng(0); best = None
for _ in range(300):
    i3 = rng.choice(len(F), 3, replace=False); a_, b_, c_ = F[i3]; nrm = np.cross(b_ - a_, c_ - a_)
    if np.linalg.norm(nrm) < 1e-6 or abs(nrm[2]) / np.linalg.norm(nrm) < 0.95:
        continue
    nrm = nrm / np.linalg.norm(nrm); dist = np.abs((F - a_) @ nrm); inl = dist < 0.02
    if best is None or inl.sum() > best[0].sum():
        best = (inl, nrm)
inl, nrm = best; Fi = F[inl]; A = np.c_[Fi[:, 0], Fi[:, 1], np.ones(len(Fi))]; coef, *_ = np.linalg.lstsq(A, Fi[:, 2], rcond=None); res = Fi[:, 2] - A @ coef
out['바닥'] = dict(가장큰평면_비율=round(float(inl.mean()) * 100, 1), 기울기_도=round(math.degrees(math.atan(math.hypot(coef[0], coef[1]))), 2),
                 평면두께p5_p95_cm=round(float(np.percentile(res, 95) - np.percentile(res, 5)) * 100, 1), 나머지는_경사로단_비율=round(float(1 - inl.mean()) * 100, 1))
# ④ 다른 녹화 지도와
B = ic.load_map(os.path.join(J, 'routes', OTHER, 'map.npz'), path_xy=path); Aw = ic.load_map(os.path.join(R, 'map.npz'), path_xy=path)
m = ic.Matcher(Aw); S = ic.voxel(B, 0.05); S = S[np.random.default_rng(0).choice(len(S), min(20000, len(S)), replace=False)]
_T0 = (-0.025, -0.039, math.radians(-3.46)); _c, _s = math.cos(_T0[2]), math.sin(_T0[2])
_Ti = (-(_c * _T0[0] + _s * _T0[1]), -(-_s * _T0[0] + _c * _T0[1]), -_T0[2])   # 1650→1645
M, rms, _ = m.align(S, ic.T3_to4(_T0 if ('1650' in NAME and '1645' in OTHER) else (_Ti if ('1645' in NAME and '1650' in OTHER) else (0.0, 0.0, 0.0)))); Q = (M[:3, :3] @ S.T).T + M[:3, 3]
T = ic.T4_to3(M)
Wa = ed.real_walls(cache)
sys.argv = ['x', OTHER]; import importlib; ed2 = importlib.reload(ed); Wb = ed2.real_walls(ed2.load_cache()); sys.argv = ['x', NAME]; ed = importlib.reload(ed)
cb, sb = math.cos(T[2]), math.sin(T[2]); Wb2 = np.c_[cb * Wb[:, 0] - sb * Wb[:, 1] + T[0], sb * Wb[:, 0] + cb * Wb[:, 1] + T[1]]
kk = np.all((Wb2 > lo) & (Wb2 < hi), 1); dd = cKDTree(Wa).query(Wb2[kk])[0]
out['두녹화지도_고정벽'] = dict(맞춘변환=[round(T[0], 3), round(T[1], 3), round(math.degrees(T[2]), 2)], 벽점거리_중앙_cm=round(float(np.median(dd)) * 100, 1),
                    p95_cm=round(float(np.percentile(dd, 95)) * 100, 1), 안5cm비율=round(float(np.mean(dd < 0.05)) * 100, 1))
# ⑤ 외톨이 잡음(편집기 벽, 경로 0.5 m 안)
Wr = ed.real_walls(cache); TR = cKDTree(Wr); near_route = cKDTree(RP).query(Wr)[0] < 0.5
nb = np.array([len(TR.query_ball_point(q, 0.12)) - 1 for q in Wr])
iso = near_route & (nb < 2)
out['외톨이잡음'] = dict(경로0_5m안벽칸=int(near_route.sum()), 외톨이=int(iso.sum()), 위치=[[round(float(a), 2), round(float(b), 2)] for a, b in Wr[iso][:12]])
# ⑥ 2D 지도
meta = dict(l.split(': ', 1) for l in open(os.path.join(R, 'map.yaml')).read().splitlines() if ': ' in l)
res_ = float(meta['resolution']); ox, oy = [float(v) for v in meta['origin'].strip('[]').split(',')[:2]]
raw = open(os.path.join(R, 'map.pgm'), 'rb').read().split(b'\n', 3); w, hh = map(int, raw[1].split()); img = np.frombuffer(raw[3][:w * hh], np.uint8).reshape(hh, w)
cc = ((Wr[:, 0] - ox) / res_).astype(int); rr = (hh - 1 - (Wr[:, 1] - oy) / res_).astype(int); okc = (cc >= 0) & (cc < w) & (rr >= 0) & (rr < hh)
occ_hit = np.mean([img[max(0, r - 1):r + 2, max(0, c - 1):c + 2].min() < 100 for r, c in zip(rr[okc], cc[okc])])
yy, xx = np.where(img < 100); X2 = np.c_[ox + (xx + 0.5) * res_, oy + (hh - 1 - yy + 0.5) * res_]; kx = np.all((X2 > lo) & (X2 < hi), 1); X2 = X2[kx]
unsup = np.mean(TR.query(X2)[0] > 0.06) if len(X2) else 0.0
out['2D지도'] = dict(벽칸이_pgm에_점유로_나온비율=round(float(occ_hit) * 100, 1), pgm점유칸중_3D벽근거없음=round(float(unsup) * 100, 1))
print(json.dumps(out, ensure_ascii=False, indent=1))
json.dump(out, open(os.path.join(H, f'map_quality_{NAME}.json'), 'w'), ensure_ascii=False, indent=1)
