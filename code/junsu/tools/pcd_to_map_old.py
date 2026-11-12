#!/usr/bin/env python3
"""9월(09-10) pcd_to_map.py 방식 재현 — 메모(go2-map-tilt-two-converters)에 남은 설정 그대로. 원본 스크립트는 NUC 에 없다.
  --floor plane : 0.5 m 칸마다 바닥 평면(z = a·x + b·y + c)을 맞춰 그 위 높이로 자른다(지도가 기울어도 반대편 벽이 안 빠짐)
  벽 대역       : 바닥 위 0.29~0.99 m 점이 있는 칸 = 점유
  격자          : 0.03 m · 미지 = 회색(205) · --min-blob 20(작은 덩어리 제거) · --fill 15(틈 메우기, 칸 단위 닫기) · --clean-free
  좋은 지도 기준 : 점유 4,000칸 이상(0.03 m)
사용: pcd_to_map_old.py <npz 또는 pcd> <출력 앞이름> [--min-blob 20] [--fill 15] [--band 0.29 0.99] [--res 0.03]
출력: <앞이름>.pgm/.yaml(ROS map_server 형식) · <앞이름>.png · <앞이름>_walls.npy(시뮬레이션 벽 형식)"""
import sys, os, argparse, numpy as np
from scipy import ndimage
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pcd_wall_sharpness import load

ap = argparse.ArgumentParser(); ap.add_argument('src'); ap.add_argument('out')
ap.add_argument('--res', type=float, default=0.03); ap.add_argument('--band', type=float, nargs=2, default=[0.29, 0.99])
ap.add_argument('--min-blob', type=int, default=20); ap.add_argument('--fill', type=int, default=15); ap.add_argument('--cell', type=float, default=0.5)
ap.add_argument('--crop', type=float, nargs=4, default=None, help='x0 x1 y0 y1')
a = ap.parse_args()
P = load(a.src); P = P[np.isfinite(P).all(1)]
if a.crop:
    x0, x1, y0, y1 = a.crop; P = P[(P[:, 0] > x0) & (P[:, 0] < x1) & (P[:, 1] > y0) & (P[:, 1] < y1)]
# 1) 0.5 m 칸마다 바닥: 칸 안 낮은 점(하위 10%)들로 평면 → 주변 칸으로 메움
C = a.cell; ck = np.floor(P[:, :2] / C).astype(int); c0 = ck.min(0); ck -= c0; CW, CH = ck.max(0) + 1
fl = np.full((CW, CH), np.nan)
order = np.lexsort((P[:, 2], ck[:, 1], ck[:, 0])); Ps, cks = P[order], ck[order]
brk = np.r_[0, np.where(np.any(np.diff(cks, axis=0) != 0, axis=1))[0] + 1, len(Ps)]
for s, e in zip(brk[:-1], brk[1:]):
    z = Ps[s:e, 2]
    if len(z) >= 20: fl[cks[s, 0], cks[s, 1]] = np.percentile(z, 10)
# 칸 바닥을 전역 평면으로 다듬고(튀는 칸 제거), 빈 칸은 가장 가까운 값
g = np.isfinite(fl); ix, iy = np.nonzero(g)
A = np.c_[ix, iy, np.ones(len(ix))]; coef, *_ = np.linalg.lstsq(A, fl[g], rcond=None)
plane = coef[0] * np.arange(CW)[:, None] + coef[1] * np.arange(CH)[None, :] + coef[2]
res = fl - plane; bad = ~g | (np.abs(res) > 0.15); fl[bad] = plane[bad]
tilt = np.degrees(np.arctan(np.hypot(coef[0], coef[1]) / C))
h = P[:, 2] - fl[ck[:, 0], ck[:, 1]]
# 2) 벽 대역 → 0.03 m 격자 점유
B = P[(h >= a.band[0]) & (h <= a.band[1])]
r = a.res; lo = P[:, :2].min(0); k = np.floor((B[:, :2] - lo) / r).astype(int); W_, H_ = (np.floor((P[:, :2].max(0) - lo) / r).astype(int) + 1)
occ = np.zeros((W_, H_), bool); occ[k[:, 0], k[:, 1]] = True
seen = np.zeros((W_, H_), bool); kk = np.floor((P[:, :2] - lo) / r).astype(int); seen[kk[:, 0], kk[:, 1]] = True
n_raw = occ.sum()
# 3) --fill: 칸 단위 닫기(작은 틈 메우기) — 원본은 --fill 15(출입구 ~80 cm 라 --fill 6 은 물이 샜다는 기록)
if a.fill > 0:
    occ = ndimage.binary_closing(occ, structure=np.ones((3, 3)), iterations=max(1, a.fill // 5))
# 4) --min-blob: 작은 덩어리 제거
lab, n = ndimage.label(occ, structure=np.ones((3, 3))); sizes = ndimage.sum(occ, lab, range(1, n + 1))
occ[np.isin(lab, np.where(sizes < a.min_blob)[0] + 1)] = False
# 5) pgm: 점유 0(검정) · 본 곳 254(흰) · 미지 205(회색), --clean-free: 점유 둘레 1칸 밖 본 곳은 흰색
img = np.full((W_, H_), 205, np.uint8); img[ndimage.binary_dilation(seen, iterations=2)] = 254; img[occ] = 0
os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
pgm = img.T[::-1]   # map_server: 위가 +y
with open(a.out + '.pgm', 'wb') as f:
    f.write(f'P5\n{W_} {H_}\n255\n'.encode()); f.write(pgm.tobytes())
open(a.out + '.yaml', 'w').write(f'image: {os.path.basename(a.out)}.pgm\nresolution: {r}\norigin: [{lo[0]:.3f}, {lo[1]:.3f}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
ox, oy = np.nonzero(occ); c = np.c_[lo[0] + (ox + 0.5) * r, lo[1] + (oy + 0.5) * r]
np.save(a.out + '_walls.npy', np.concatenate([np.c_[c, np.full(len(c), z)] for z in (0.15, 0.35)]).astype(np.float32))
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
plt.figure(figsize=(10, 8)); plt.imshow(img.T, origin='lower', cmap='gray', extent=(lo[0], lo[0] + W_ * r, lo[1], lo[1] + H_ * r), vmin=0, vmax=255)
plt.title(f'pcd_to_map(9월 방식) {r} m · 점유 {occ.sum()} 칸 · 바닥 기울기 {tilt:.2f}°'); plt.savefig(a.out + '.png', dpi=80)
print(f'점 {len(P)} · 바닥 평면 기울기 {tilt:.2f}° · 벽 대역 점 {len(B)} · 점유 {n_raw} → 정리 후 {occ.sum()} 칸 (기준 ≥4000) · {a.out}.pgm/.yaml/.png/_walls.npy')
