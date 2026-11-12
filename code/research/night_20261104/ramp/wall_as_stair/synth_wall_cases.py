#!/usr/bin/env python3
"""'벽을 계단으로 보는' 경우 합성 시험 — 거부권(traversable+veto)이 벽 점을 지우는지.
로봇(0,0) 몸 z=0.31, 바닥 0. 벽은 x=1.0 에 y −1~1, 두께 2 cm. 경우별 ego 장애물 점 중 지워진 비율, 팽창 0.40 m 후 벽 선이 끊기는지(틈 최대 길이)."""
import sys, numpy as np, warnings
warnings.simplefilter('ignore')
sys.path.insert(0, '/home/ilovehan9283/ws/night_20261104/ramp')
from terrain_core import ego_rule, grid_stats, traversable, veto
rng = np.random.default_rng(0)
C = 0.05; ORG = np.array([-4.0, -4.0]); N = 160; ROB = np.array([0.0, 0.0, 0.31])
plane = ((0.0, 0.0, 0.0), (0.0, 0.0, 1.0))


def floor(x0=-1.0, x1=3.0, y0=-1.2, y1=1.2, z=0.0, step=0.03, keep=1.0):
    X, Y = np.meshgrid(np.arange(x0, x1, step), np.arange(y0, y1, step)); P = np.c_[X.ravel(), Y.ravel(), np.full(X.size, z)]
    return P[rng.random(len(P)) < keep]


def wall(x=1.0, h=0.6, zb=0.0, y0=-1.0, y1=1.0, step=0.03, keep=1.0, rows=None):
    zs = np.arange(zb, zb + h + 1e-9, step) if rows is None else np.array(rows)
    Y, Z = np.meshgrid(np.arange(y0, y1, step), zs); P = np.c_[np.full(Y.size, x), Y.ravel(), Z.ravel()]
    return P[rng.random(len(P)) < keep]


def run(name, W, wall_mask):
    W = W + rng.normal(0, 0.008, W.shape)
    em = ego_rule(W, ROB, 0.0)
    hi, lo, _ = grid_stats(W, ORG, (N, N), C, ROB[2] + 1.1)
    T = traversable(hi, lo, C, ROB[:2], ORG, ROB[2], hop=0.15 + 1e-6, seed_r=1.5, plane=plane)
    v = veto(W, T, ORG, C)
    we = em & wall_mask; kept = we & ~v
    # 팽창 0.40 m 후 벽 선(y −1~1)이 끊긴 최대 길이
    ys = np.arange(-1.0, 1.0, 0.05); cov = []
    K = W[em & ~v]
    for y in ys:
        d = np.hypot(K[:, 0] - 1.0, K[:, 1] - y) if len(K) else np.array([9.0])
        cov.append(d.min() <= 0.40)
    gap = 0; g = 0
    for c in cov:
        g = 0 if c else g + 0.05; gap = max(gap, g)
    print(f'{name:34s} 벽 ego점 {we.sum():5d}  지워짐 {100 * (1 - kept.sum() / max(we.sum(), 1)):5.1f}%  팽창 후 벽 틈 최대 {gap:.2f} m')


F = floor()
for h in (0.10, 0.15, 0.20, 0.25, 0.30, 0.60):
    Wl = wall(h=h); run(f'낮은 벽/턱 높이 {h:.2f} m', np.r_[F, Wl], np.r_[np.zeros(len(F), bool), np.ones(len(Wl), bool)])
# 벽 아래쪽만 보임(위는 시야 밖): 0~0.18 m 만 관측
Wl = wall(h=0.18); run('0.6 m 벽인데 아래 0.18 m 만 보임', np.r_[F, Wl], np.r_[np.zeros(len(F), bool), np.ones(len(Wl), bool)])
# 벽 윗부분만 링 하나로 보임(바닥 쪽 안 보임)
Wl = wall(rows=[0.55]); run('0.6 m 벽인데 윗줄 하나만 보임', np.r_[F, Wl], np.r_[np.zeros(len(F), bool), np.ones(len(Wl), bool)])
# 성긴 벽(점 20%)
Wl = wall(keep=0.2); run('0.6 m 벽 점 20%만', np.r_[F, Wl], np.r_[np.zeros(len(F), bool), np.ones(len(Wl), bool)])
# 벽 바로 앞 바닥이 빔(벽 밑동 그림자)
F2 = F[~((F[:, 0] > 0.8) & (F[:, 0] < 1.0))]; Wl = wall()
run('0.6 m 벽, 벽 앞 20 cm 바닥 빔', np.r_[F2, Wl], np.r_[np.zeros(len(F2), bool), np.ones(len(Wl), bool)])
# 계단 옆 벽: 계단 0.17×3 (x 0.6~) 위로 옆벽(y=+0.6, 높이 계단+0.6)
st = []
for i in range(3):
    st.append(floor(x0=0.6 + 0.3 * i, x1=0.9 + 0.3 * i if i < 2 else 3.0, y0=-0.55, y1=0.55, z=0.17 * (i + 1)))
S = np.concatenate(st)
sw = []
for xx in np.arange(-1.0, 3.0, 0.03):
    gz = 0.17 * min(3, max(0, int((xx - 0.6) // 0.3) + 1)) if xx >= 0.6 else 0.0
    for zz in np.arange(gz, gz + 0.6, 0.03): sw.append((xx, 0.6, zz))
SW = np.array(sw); F3 = floor(x1=0.6, y0=-0.55, y1=0.55)
Wall = np.r_[F3, S, SW]; m = np.r_[np.zeros(len(F3) + len(S), bool), np.ones(len(SW), bool)]
W_ = Wall + rng.normal(0, 0.008, Wall.shape); em = ego_rule(W_, ROB, 0.0)
hi, lo, _ = grid_stats(W_, ORG, (N, N), C, ROB[2] + 1.1)
T = traversable(hi, lo, C, ROB[:2], ORG, ROB[2], hop=0.15 + 1e-6, seed_r=1.5, plane=plane); v = veto(W_, T, ORG, C)
print(f'{"계단 0.17×3 옆벽 0.6 m":34s} 옆벽 ego점 {(em & m).sum():5d}  지워짐 {100 * (1 - (em & m & ~v).sum() / max((em & m).sum(), 1)):5.1f}%  계단면 ego점 {(em & ~m).sum()} → 남음 {(em & ~m & ~v).sum()}')
