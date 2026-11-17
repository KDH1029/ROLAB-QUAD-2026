#!/usr/bin/env python3
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

path = '/home/gamulchi/arena0921_map/arena0921_map_ascii.pcd'
with open(path) as f:
    lines = f.readlines()
he = next(i for i, l in enumerate(lines) if l.startswith('DATA'))
data = np.loadtxt(lines[he + 1:], dtype=np.float32)
x, y, z = data[:, 0], data[:, 1], data[:, 2]

# 물리적으로 역산한 바닥 기준:
# t=0 시점 body_height(IMU 높이) = 0.329m, 라이다 레버암 +0.1588m
# => 라이다(=camera_init 원점 근처)는 바닥에서 약 0.488m 위
# => 절대 Z=0(원점)이 바닥 기준 +0.488m 이므로, 바닥은 절대 Z = -0.488
floor_z_physical = -0.488
z_rel = z - floor_z_physical
print(f"물리 기반 바닥 기준 사용: floor_z={floor_z_physical}")
print(f"전체 z_rel 범위: {z_rel.min():.2f}~{z_rel.max():.2f}")

# 기존 히스토그램 모드 방식과 비교
hist, edges = np.histogram(z, bins=np.arange(z.min(), z.max(), 0.02))
floor_z_hist = edges[np.argmax(hist)] + 0.01
print(f"(참고) 히스토그램 최빈값 방식: floor_z={floor_z_hist:.3f} -- 차이 {floor_z_hist - floor_z_physical:.3f}m")

# 방 핵심부만
clean = (x > -3) & (x < 7) & (y > -4) & (y < 9) & (z_rel < 3.5) & (z_rel > -0.3)
xc, yc, zc = x[clean], y[clean], z_rel[clean]
print(f"정제: {len(xc)}개, 높이범위 {zc.min():.2f}~{zc.max():.2f}")

fig, axes = plt.subplots(1, 3, figsize=(28, 9))

sc = axes[0].scatter(xc, yc, c=zc, cmap='turbo', s=2, vmin=0, vmax=0.6)
axes[0].set_title('CORRECTED floor ref: 0~60cm (real stair-step height range)')
axes[0].set_aspect('equal'); axes[0].grid(True, alpha=0.3)
plt.colorbar(sc, ax=axes[0], label='height above real floor (m)')

mid = (zc > 0.5)
sc2 = axes[1].scatter(xc[mid], yc[mid], c=zc[mid], cmap='turbo', s=2, vmin=0.5, vmax=2.5)
axes[1].set_title('CORRECTED: >50cm (walls + tall structures)')
axes[1].set_aspect('equal'); axes[1].grid(True, alpha=0.3)
plt.colorbar(sc2, ax=axes[1], label='height (m)')

h2d, xe, ye = np.histogram2d(xc, yc, bins=150)
im = axes[2].imshow(h2d.T, origin='lower', extent=[xe[0], xe[-1], ye[0], ye[-1]],
                     cmap='hot', aspect='equal', vmax=np.percentile(h2d[h2d > 0], 95))
axes[2].set_title('Point density')
plt.colorbar(im, ax=axes[2], label='point count')

plt.tight_layout()
out = '/home/gamulchi/arena0921_map/topdown_corrected.png'
plt.savefig(out, dpi=150)
print(f"저장: {out}")

# 전체 높이 히스토그램도 (바닥 기준 교정됨)
fig2, ax = plt.subplots(figsize=(12,5))
hh, he2 = np.histogram(zc, bins=np.arange(-0.3, 3.0, 0.05))
ax.bar((he2[:-1]+he2[1:])/2, hh, width=0.04)
ax.set_xlabel('height above real floor (m)')
ax.set_ylabel('point count')
ax.set_title('Height histogram (corrected floor reference)')
plt.tight_layout()
plt.savefig('/home/gamulchi/arena0921_map/height_histogram_corrected.png', dpi=150)
print("저장: height_histogram_corrected.png")
