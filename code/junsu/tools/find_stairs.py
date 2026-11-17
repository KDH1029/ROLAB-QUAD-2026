#!/usr/bin/env python3
import numpy as np

path = '/home/gamulchi/arena0921_map/arena0921_map_ascii.pcd'
with open(path) as f:
    lines = f.readlines()
header_end = next(i for i, l in enumerate(lines) if l.startswith('DATA'))
data = np.loadtxt(lines[header_end + 1:], dtype=np.float32)
x, y, z = data[:, 0], data[:, 1], data[:, 2]
n0 = len(x)

# 바닥 추정 (정제 전 전체 기준)
hist, edges = np.histogram(z, bins=np.arange(z.min(), z.max(), 0.02))
floor_z = edges[np.argmax(hist)] + 0.01

# 정제: 관찰된 "진짜 방" 밀집 구역만 (X: -8~9, Y: -7~9), 그리고 바닥 기준 3m 이하
clean = (x > -8) & (x < 9) & (y > -7) & (y < 9) & ((z - floor_z) < 3.0) & ((z - floor_z) > -0.5)
x, y, z_rel = x[clean], y[clean], (z - floor_z)[clean]
n = len(x)
print(f"정제: {n0} -> {n} ({100*n/n0:.1f}% 유지)")
print(f"정제된 X: {x.min():.2f}~{x.max():.2f}  Y: {y.min():.2f}~{y.max():.2f}  Z상대: {z_rel.min():.2f}~{z_rel.max():.2f}")

# 높은 구조물(계단 후보): 바닥+0.12m 이상
high = z_rel > 0.12
xh, yh, zh = x[high], y[high], z_rel[high]
print(f"\n높은 구조물 포인트: {len(xh)}개")

# 50cm 그리드 밀도 + 평균높이
gx = np.arange(x.min(), x.max() + 0.5, 0.5)
gy = np.arange(y.min(), y.max() + 0.5, 0.5)
H, xe, ye = np.histogram2d(xh, yh, bins=[gx, gy])
Hz = np.zeros_like(H)
cnt = np.zeros_like(H)
ix = np.clip(np.digitize(xh, gx) - 1, 0, len(gx) - 2)
iy = np.clip(np.digitize(yh, gy) - 1, 0, len(gy) - 2)
for k in range(len(xh)):
    Hz[ix[k], iy[k]] += zh[k]
    cnt[ix[k], iy[k]] += 1
with np.errstate(invalid='ignore', divide='ignore'):
    avgH = np.where(cnt > 0, Hz / cnt, 0)

print("\n밀도 상위 셀 (중심좌표, 점개수, 평균높이):")
flat_idx = np.argsort(H.flatten())[::-1][:20]
cells = []
for idx in flat_idx:
    i, j = np.unravel_index(idx, H.shape)
    if H[i, j] > 10:
        cx, cy = (xe[i] + xe[i+1]) / 2, (ye[j] + ye[j+1]) / 2
        cells.append((cx, cy, int(H[i, j]), avgH[i, j]))
        print(f"  X~{cx:5.1f} Y~{cy:5.1f}: {int(H[i,j]):4d}점  평균높이 {avgH[i,j]:.2f}m")

if cells:
    cxs = [c[0] for c in cells]
    cys = [c[1] for c in cells]
    print(f"\n★ 높은-밀도 셀들의 bounding box (계단/높은구조물 후보):")
    print(f"  X: {min(cxs)-0.25:.2f} ~ {max(cxs)+0.25:.2f}")
    print(f"  Y: {min(cys)-0.25:.2f} ~ {max(cys)+0.25:.2f}")
