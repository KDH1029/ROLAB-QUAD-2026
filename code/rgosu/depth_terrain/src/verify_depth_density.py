#!/usr/bin/env python3
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

d = np.load('/home/gamulchi/arena0921_map/depth_pointcloud_sample.npz')
x, y, z = d['x'], d['y'], d['z']

fig, axes = plt.subplots(1, 2, figsize=(18, 8))

# 카메라 시점에서 본 점구름 (전방 거리로 색칠)
sc = axes[0].scatter(x, -y, c=z, cmap='turbo', s=0.5, vmin=z.min(), vmax=z.max())
axes[0].set_title(f'RealSense depth point cloud ({len(x)} pts)')
axes[0].set_xlabel('left-right (m)')
axes[0].set_ylabel('up-down (m)')
axes[0].set_aspect('equal')
plt.colorbar(sc, ax=axes[0], label='depth (m)')

# 1m x 1m 구간에서의 포인트 밀도 비교 (라이다 vs 뎁스카메라)
# 라이다: arena0921 지도 164561점이 대략 12x12m 방에 퍼짐 -> 밀도 추정
lidar_density = 164561 / (12 * 12)  # points per m^2, 매우 거친 추정(3D라 단순비교지만 참고용)
# 뎁스카메라: 이번 프레임이 커버하는 대략적 면적
depth_area = (x.max() - x.min()) * (z.max() - z.min())
depth_density = len(x) / max(depth_area, 0.01)
print(f"라이다 지도 전체 평균 밀도(대략): {lidar_density:.0f} pts/m^2")
print(f"뎁스카메라 한 프레임 밀도(대략): {depth_density:.0f} pts/m^2")
print(f"밀도 비율: 뎁스카메라가 라이다의 약 {depth_density/lidar_density:.0f}배")

axes[1].bar(['LiDAR (XT16, whole-room avg)', 'RealSense (one frame)'],
            [lidar_density, depth_density], color=['steelblue', 'orange'])
axes[1].set_ylabel('points / m^2 (rough estimate)')
axes[1].set_title('Point density comparison')
axes[1].set_yscale('log')
for i, v in enumerate([lidar_density, depth_density]):
    axes[1].text(i, v, f'{v:.0f}', ha='center', va='bottom')

plt.tight_layout()
plt.savefig('/home/gamulchi/arena0921_map/depth_vs_lidar_density.png', dpi=150)
print("저장: depth_vs_lidar_density.png")
