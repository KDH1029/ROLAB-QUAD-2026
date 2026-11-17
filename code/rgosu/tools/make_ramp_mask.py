#!/usr/bin/env python3
"""X자 경사로·계단 마스크 — 콘(낮은 장애물) 감지기가 이 안에서는 콘을 찾지 않는다(경사면·계단 턱을 콘으로 잡아 오르막 앞에서 멈추지 않게).
자동 판단(높이·기울기)은 검은 매트의 성긴 점과 좁은 차선 벽 밑동 때문에 차선 전체를 덮어서 접었다(11-05).
대신: X자 가운데에서 네 팔 방향 중심선을 따라 바닥 높이를 재고, 바깥 차선 높이(+3 cm)로 돌아오는 거리 + 여유까지를 팔 방향 띠로 덮는다.
좌표 = 지도(14:59 녹화 FAST-LIO) = 주행 FAST-LIO(같은 출발점·방향일 때).
사용: python3 make_ramp_mask.py [지도 npz] [출력 npz=maps/ramp_mask.npz]"""
import os, sys, math, numpy as np
H = os.path.dirname(os.path.abspath(__file__))
src = open(os.path.join(H, 'render_like_1030.py')).read(); g = {'np': np, 'math': math}
exec(src[src.index('def revox'):src.index('def wall_band_thickness')], g)
MAP = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else '~/ws/v_gosu/eval/out/fixed1459_2cm_map.npz')
OUT = os.path.expanduser(sys.argv[2] if len(sys.argv) > 2 else '~/ws/Rgosu/maps/ramp_mask.npz')
PNG = OUT.replace('.npz', '.png')
CENTER = np.array([3.78, 2.08])                                   # X자 교차점(벽 꼭짓점 네 개의 가운데)
ARMS = {'SW': (2.0, 0.2), 'NE': (5.6, 4.2), 'SE': (5.9, 0.35), 'NW': (1.75, 3.95)}   # 팔 끝(바깥 차선) — 경로 점 근처
C, PAD, HALFW, CORE = 0.05, 0.30, 0.70, 1.0                        # 칸 · 끝 여유 · 띠 반폭(차선 1 m + 여유) · 가운데 단 반지름

d = np.load(MAP); P = (d['vox'][d['cnt'] >= 2] + 0.5) * float(d['V']); od = d['odom']; tr = od[:, 2:5]
REG = (tr[:, 0].min() - 1, tr[:, 0].max() + 1, tr[:, 1].min() - 1, tr[:, 1].max() + 1)
coef, _ = g['fit_floor'](P, REG); h = P[:, 2] - (coef[0] * P[:, 0] + coef[1] * P[:, 1] + coef[2])
q = (h > -0.15) & (h < 0.7); Q, hq = P[q, :2], h[q]
key = np.floor(Q / 0.04).astype(np.int64); u, inv = np.unique(key, axis=0, return_inverse=True); inv = inv.ravel()
zmin = np.full(len(u), 9.); zmax = np.full(len(u), -9.); np.minimum.at(zmin, inv, hq); np.maximum.at(zmax, inv, hq)
ok = (zmax - zmin) < 0.04; F, fz = (u[ok] + 0.5) * 0.04, zmin[ok]       # 바닥 같은 칸(칸 안 높이 폭 4 cm 미만)
ends = {}
for nm, e in ARMS.items():
    v = np.array(e) - CENTER; L = np.linalg.norm(v); ua = v / L; na = np.array([-ua[1], ua[0]])
    rel = F - CENTER; al = rel @ ua; s = np.abs(rel @ na) < 0.20
    prof = [(a, np.median(fz[s & (np.abs(al - a) < 0.1)])) for a in np.arange(0, L + 0.01, 0.1) if (s & (np.abs(al - a) < 0.1)).sum() >= 3]
    lane = np.median([z for a, z in prof if a >= L - 0.6]); top = np.median([z for a, z in prof if a <= 0.3])
    end = next((a for a, z in prof if a > 0.3 and z <= lane + 0.03), L)
    ends[nm] = (ua, min(end + PAD, L)); print(f' {nm}: 가운데 {top:.2f} m · 바깥 차선 {lane:.2f} m · 차선 높이로 돌아오는 곳 {end:.1f} m → 마스크 끝 {min(end + PAD, L):.1f} m')
lo = np.array([REG[0] - 1, REG[2] - 1]); W, Hh = (np.ceil((np.array([REG[1] + 1, REG[3] + 1]) - lo) / C)).astype(int)
gx, gy = np.meshgrid(lo[0] + (np.arange(W) + 0.5) * C, lo[1] + (np.arange(Hh) + 0.5) * C, indexing='ij')
rel = np.stack([gx - CENTER[0], gy - CENTER[1]], -1); mask = np.hypot(rel[..., 0], rel[..., 1]) <= CORE
for nm, (ua, ln) in ends.items():
    al = rel @ ua; pe = rel @ np.array([-ua[1], ua[0]]); mask |= (al >= 0) & (al <= ln) & (np.abs(pe) <= HALFW)
np.savez_compressed(OUT, mask=mask, origin=lo, cell=C, center=CENTER, map=MAP)
print(f'마스크 {mask.sum() * C * C:.2f} m² → {OUT}')
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
plt.rcParams['font.family'] = ['NanumGothic', 'DejaVu Sans']
k = (h > -0.3) & (h < 1.8); ext = (lo[0], lo[0] + W * C, lo[1], lo[1] + Hh * C); img = g['grid_max'](P[k, :2], h[k], 0.03, ext)
fig, ax = plt.subplots(figsize=(11, 7), dpi=80); ax.imshow(img, origin='lower', extent=ext, cmap='turbo', vmin=0, vmax=1.2)
ax.contour(mask.T.astype(float), levels=[0.5], origin='lower', extent=ext, colors='w', linewidths=2)
ax.set_xlim(-2.5, 8.5); ax.set_ylim(-1.5, 6); ax.set_aspect('equal'); ax.grid(alpha=0.3)
ax.set_title('X자 경사로·계단 마스크(흰 선 안 = 콘 감지 끔) — 바탕 = 지도 높이'); fig.savefig(PNG, bbox_inches='tight'); print('그림', PNG)
