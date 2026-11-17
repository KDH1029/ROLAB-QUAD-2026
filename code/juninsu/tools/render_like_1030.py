#!/usr/bin/env python3
"""map_replay 지도(npz)를 10-30 그림(render_1030.py XT16 판)과 똑같은 방법으로 그린다 — 지도끼리 눈으로 공정 비교용.
같은 것: 2회 이상 찍힌 복셀 · 바닥 평면 맞춤(칸별 최저점 최빈 띠, 두 번 다듬음) · 바닥 위 높이 · 5 cm 칸 최대 높이 · turbo 0~1.8 m
다른 복셀 크기(2 cm)로 만든 지도는 5 cm 로 다시 묶은 뒤(횟수 합) 같은 기준을 쓴다.
사용: render_like_1030.py <출력 png> <이름=npz> [<이름=npz> ...]"""
import sys, os, numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams['font.family'] = ['NanumGothic', 'DejaVu Sans']; plt.rcParams['axes.unicode_minus'] = False
HMAX, CELL = 1.8, 0.05


def revox(d, V2=0.05):
    V = float(d['V']); P = (d['vox'].astype(np.float64) + 0.5) * V; c = d['cnt']
    if abs(V - V2) < 1e-9:
        return P[c >= 2]
    k = np.floor(P / V2).astype(np.int64); u, inv = np.unique(k, axis=0, return_inverse=True)
    s = np.bincount(inv.ravel(), weights=c, minlength=len(u))
    return (u[s >= 2] + 0.5) * V2


def fit_floor(P, region):
    x0, x1, y0, y1 = region
    m = (P[:, 0] > x0) & (P[:, 0] < x1) & (P[:, 1] > y0) & (P[:, 1] < y1); Q = P[m]
    key = np.floor(Q[:, 0] / 0.1).astype(np.int64) * 100000 + np.floor(Q[:, 1] / 0.1).astype(np.int64)
    o = np.lexsort((Q[:, 2], key)); k, Qs = key[o], Q[o]
    L = Qs[np.r_[True, k[1:] != k[:-1]]]
    h, e = np.histogram(L[:, 2], bins=np.arange(L[:, 2].min(), L[:, 2].max() + 0.02, 0.02))
    z0 = e[np.argmax(h)] + 0.01; sel = np.abs(L[:, 2] - z0) < 0.06
    for tol in (0.04, 0.025):
        A = np.c_[L[sel, 0], L[sel, 1], np.ones(sel.sum())]
        coef = np.linalg.lstsq(A, L[sel, 2], rcond=None)[0]
        sel = np.abs(L[:, 2] - (np.c_[L[:, 0], L[:, 1], np.ones(len(L))] @ coef)) < tol
    return coef, np.degrees(np.arctan(np.hypot(coef[0], coef[1])))


def grid_max(xy, z, cell, ext):
    x0, x1, y0, y1 = ext
    nx = int((x1 - x0) / cell) + 1; ny = int((y1 - y0) / cell) + 1
    ix = ((xy[:, 0] - x0) / cell).astype(int); iy = ((xy[:, 1] - y0) / cell).astype(int)
    m = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    img = np.full((ny, nx), np.nan, np.float32); flat = img.reshape(-1); idx = iy[m] * nx + ix[m]
    o = np.argsort(z[m]); flat[idx[o]] = z[m][o]
    return img


def wall_band_thickness(xy, h, ext):
    """벽 띠(바닥 위 0.35~0.9 m) 5 cm 칸이 벽을 가로질러 몇 칸 두께인가 — 칸마다 가장 얇은 방향 폭(4방향 중 최소 연속 칸 수)의 중앙값 × 5 cm"""
    m = (h > 0.35) & (h < 0.9); x0, x1, y0, y1 = ext
    nx = int((x1 - x0) / CELL) + 1; ny = int((y1 - y0) / CELL) + 1
    ix = ((xy[m, 0] - x0) / CELL).astype(int); iy = ((xy[m, 1] - y0) / CELL).astype(int); ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    B = np.zeros((ny, nx), bool); B[iy[ok], ix[ok]] = True
    runs = []; ys_, xs_ = np.nonzero(B); S = set(zip(ys_.tolist(), xs_.tolist()))
    for dy, dx in ((0, 1), (1, 0), (1, 1), (1, -1)):
        Lr = {}   # 방향마다 칸이 속한 연속 구간 길이
        for (y, x) in S:
            if (y - dy, x - dx) in S: continue
            run = []; yy, xx = y, x
            while (yy, xx) in S: run.append((yy, xx)); yy += dy; xx += dx
            for c in run: Lr[c] = len(run) * (np.hypot(dy, dx))
        runs.append(Lr)
    th = [min(r[c] for r in runs) for c in runs[0]]
    return float(np.median(th)) * CELL, int(B.sum())


out = sys.argv[1]; args = [a.split('=', 1) for a in sys.argv[2:]]
fig, axs = plt.subplots(1, len(args), figsize=(8.2 * len(args), 8.6), dpi=90)
for ax, (name, f) in zip(np.atleast_1d(axs), args):
    d = np.load(os.path.expanduser(f)); P = revox(d); od = d['odom']; tr = od[:, 2:5]
    REG = (tr[:, 0].min() - 1, tr[:, 0].max() + 1, tr[:, 1].min() - 1, tr[:, 1].max() + 1)
    coef, tilt = fit_floor(P, REG); h = P[:, 2] - (coef[0] * P[:, 0] + coef[1] * P[:, 1] + coef[2])
    keep = (h > -0.3) & (h < HMAX); Px, hx = P[keep], h[keep]
    px0, px1 = np.percentile(Px[:, 0], [0.5, 99.5]); py0, py1 = np.percentile(Px[:, 1], [0.5, 99.5])
    ext = (np.floor(max(tr[:, 0].min() - 2.5, px0 - 0.3)), np.ceil(min(tr[:, 0].max() + 2.5, px1 + 0.3)),
           np.floor(max(tr[:, 1].min() - 2.5, py0 - 0.3)), np.ceil(min(tr[:, 1].max() + 4.5, py1 + 0.3)))
    img = grid_max(Px[:, :2], hx, CELL, ext)
    thk, nb = wall_band_thickness(Px[:, :2], hx, ext)
    dur = float(od[:, 1].max() - od[:, 1].min()); plen = float(np.sum(np.linalg.norm(np.diff(tr, axis=0), axis=1)))
    im = ax.imshow(img, origin='lower', extent=ext, cmap='turbo', vmin=0, vmax=HMAX, interpolation='nearest')
    ax.plot(0, 0, marker='o', ms=7, mfc='white', mec='k'); ax.set_aspect('equal'); ax.grid(alpha=0.25, lw=0.5)
    ax.set_title(f'{name} · {dur:.0f}초 · 경로 {plen:.0f} m · 원래 칸 {float(d["V"]) * 100:.0f} cm\n5 cm 칸 · 바닥 평면 보정 {tilt:.2f}° · 벽띠 두께 중앙 {thk * 100:.0f} cm', fontsize=11)
    print(f'{name}: {dur:.0f} s · 경로 {plen:.0f} m · 원래 칸 {float(d["V"]) * 100:.0f} cm · 바닥 기울기 {tilt:.2f}° · 벽띠(0.35~0.9 m) 칸 {nb} · 벽 두께(가장 얇은 방향) 중앙 {thk * 100:.1f} cm')
fig.colorbar(im, ax=axs, fraction=0.02, pad=0.01).set_label('바닥 위 높이 (m)')
fig.savefig(out); print(out)
