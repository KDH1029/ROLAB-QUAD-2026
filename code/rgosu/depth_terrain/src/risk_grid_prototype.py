#!/usr/bin/env python3
"""위험도 그리드 + A* 경로탐색 프로토타입.
KAIST '위험도 기반 경로계획' 아이디어를 기존 arena_0921 지도로 검증.
결과 웨이포인트는 나중에 ego_planner에 순서대로 넘기는 용도로 쓸 것."""
import heapq
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

FLOOR_Z = -0.488  # 물리 기반 보정값 (body_height 0.329 + lever arm 0.1588)
CELL = 0.15       # 15cm 그리드 — 발 디딜 자리 스케일

path = '/home/gamulchi/arena0921_map/arena0921_map_ascii.pcd'
with open(path) as f:
    lines = f.readlines()
he = next(i for i, l in enumerate(lines) if l.startswith('DATA'))
data = np.loadtxt(lines[he + 1:], dtype=np.float32)
x, y, z = data[:, 0], data[:, 1], data[:, 2]
z_rel = z - FLOOR_Z

# 방 핵심부만 (어제 확인된 '진짜 방' 범위)
m = (x > -3) & (x < 7) & (y > -4) & (y < 9) & (z_rel > -0.3) & (z_rel < 3.5)
x, y, z_rel = x[m], y[m], z_rel[m]

# --- 1) 셀별 "지면 높이"(elevation) 맵 만들기 ---
gx = np.arange(x.min(), x.max() + CELL, CELL)
gy = np.arange(y.min(), y.max() + CELL, CELL)
nx, ny = len(gx) - 1, len(gy) - 1
ix = np.clip(np.digitize(x, gx) - 1, 0, nx - 1)
iy = np.clip(np.digitize(y, gy) - 1, 0, ny - 1)

elevation = np.full((nx, ny), np.nan)
roughness = np.full((nx, ny), np.nan)
low_point_frac = np.zeros((nx, ny))  # 바닥 근처(<30cm) 점 비율 — 낮을수록 "못 보는/막힌" 영역
counts = np.zeros((nx, ny), dtype=int)

for i in range(nx):
    for j in range(ny):
        sel = (ix == i) & (iy == j)
        n = sel.sum()
        counts[i, j] = n
        if n < 3:
            continue
        zc = z_rel[sel]
        low = zc[zc < 0.5]
        if len(low) >= 2:
            elevation[i, j] = np.percentile(low, 10)  # 로버스트 바닥 추정
            roughness[i, j] = np.std(low)
        low_point_frac[i, j] = len(zc[zc < 0.3]) / n

# --- 2) 경사(slope) 맵: 이웃 셀과의 elevation 차이 ---
slope = np.full((nx, ny), np.nan)
for i in range(1, nx - 1):
    for j in range(1, ny - 1):
        if np.isnan(elevation[i, j]):
            continue
        neigh = [elevation[i+1,j], elevation[i-1,j], elevation[i,j+1], elevation[i,j-1]]
        neigh = [v for v in neigh if not np.isnan(v)]
        if neigh:
            slope[i, j] = max(abs(elevation[i, j] - v) for v in neigh) / CELL

# --- 3) 위험도 합성 (0~100, KAIST 스타일: 낮을수록 안전) ---
risk = np.full((nx, ny), 100.0)  # 기본값: 미관측=위험(모름)
valid = ~np.isnan(elevation)
rough_n = np.clip(roughness / 0.15, 0, 1)      # 15cm 거칠기면 최대
slope_n = np.clip(slope / 2.0, 0, 1)           # 기울기 2(63도)면 최대
risk[valid] = 20 + 40 * np.nan_to_num(rough_n[valid]) + 40 * np.nan_to_num(slope_n[valid])
# 벽/높은 구조물(로봇이 못 지나감): low_point_frac 너무 낮으면 사실상 막힘 취급은 아니고,
# 그냥 '관측 적음'으로 두고 risk 기본값(100) 유지하게 valid만 갱신했음

# 완전히 못 지나가는 곳(바닥 안 보임, 관측 자체가 없음) 마스크
blocked = counts < 3

print(f"그리드 크기: {nx}x{ny}, 유효 셀: {valid.sum()}, 평균 위험도(유효): {np.nanmean(risk[valid]):.1f}")

# --- 4) A* 경로탐색 (위험도를 비용에 더해서) ---
def astar(risk_grid, blocked_grid, start, goal, risk_weight=2.0):
    nx, ny = risk_grid.shape
    def h(a, b):
        return np.hypot(a[0]-b[0], a[1]-b[1])
    openset = [(h(start, goal), 0, start)]
    came_from = {}
    gscore = {start: 0}
    visited = set()
    while openset:
        _, g, cur = heapq.heappop(openset)
        if cur in visited:
            continue
        visited.add(cur)
        if cur == goal:
            path = [cur]
            while cur in came_from:
                cur = came_from[cur]
                path.append(cur)
            return path[::-1]
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nxt = (cur[0]+dx, cur[1]+dy)
                if not (0 <= nxt[0] < nx and 0 <= nxt[1] < ny):
                    continue
                if blocked_grid[nxt]:
                    continue
                step = np.hypot(dx, dy) * CELL
                cell_risk = risk_grid[nxt] if not np.isnan(risk_grid[nxt]) else 100.0
                cost = step * (1.0 + risk_weight * cell_risk / 100.0)
                ng = g + cost
                if nxt not in gscore or ng < gscore[nxt]:
                    gscore[nxt] = ng
                    came_from[nxt] = cur
                    heapq.heappush(openset, (ng + h(nxt, goal), ng, nxt))
    return None

# 시작점(출발자세=원점 근처)과, 어제 발견한 계단 후보 반대편(예: 방 반대쪽 모서리)
def to_cell(px, py):
    return (int(np.clip(np.digitize(px, gx)-1, 0, nx-1)), int(np.clip(np.digitize(py, gy)-1, 0, ny-1)))

start_cell = to_cell(0.0, 0.0)
goal_cell = to_cell(1.5, 5.5)
print(f"시작 셀 {start_cell} (막힘={blocked[start_cell]}), 목표 셀 {goal_cell} (막힘={blocked[goal_cell]})")

route = astar(risk, blocked, start_cell, goal_cell, risk_weight=3.0)
route_direct = astar(np.zeros_like(risk), blocked, start_cell, goal_cell, risk_weight=0.0)  # 거리만 고려(비교용)

# --- 시각화 ---
fig, axes = plt.subplots(1, 2, figsize=(20, 9))
im = axes[0].imshow(risk.T, origin='lower', cmap='RdYlGn_r', vmin=0, vmax=100,
                     extent=[gx[0], gx[-1], gy[0], gy[-1]], aspect='equal')
axes[0].set_title('Risk grid (green=safe, red=risky)')
plt.colorbar(im, ax=axes[0], label='risk (0-100)')

im2 = axes[1].imshow(risk.T, origin='lower', cmap='RdYlGn_r', vmin=0, vmax=100,
                      extent=[gx[0], gx[-1], gy[0], gy[-1]], aspect='equal', alpha=0.7)
if route:
    rx = [gx[c[0]] + CELL/2 for c in route]
    ry = [gy[c[1]] + CELL/2 for c in route]
    axes[1].plot(rx, ry, 'b-', linewidth=3, label='risk-aware A*')
if route_direct:
    rx2 = [gx[c[0]] + CELL/2 for c in route_direct]
    ry2 = [gy[c[1]] + CELL/2 for c in route_direct]
    axes[1].plot(rx2, ry2, 'k--', linewidth=2, label='shortest (risk ignored)')
axes[1].plot(0, 0, 'go', markersize=12, label='start')
axes[1].plot(1.5, 5.5, 'r*', markersize=15, label='goal')
axes[1].legend()
axes[1].set_title('A* path comparison')
plt.colorbar(im2, ax=axes[1], label='risk (0-100)')

plt.tight_layout()
out = '/home/gamulchi/arena0921_map/risk_grid_prototype.png'
plt.savefig(out, dpi=150)
print(f"저장: {out}")
if route:
    print(f"위험도 고려 경로: {len(route)}칸, 직선거리 대비 ratio 계산 생략")
