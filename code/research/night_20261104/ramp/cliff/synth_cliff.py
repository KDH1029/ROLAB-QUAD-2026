#!/usr/bin/env python3
"""합성 시험 — 높이 0.34 m 단(1.2×1.2 m) 위 로봇. +x 쪽 계단 0.17×2(디딤 0.30 m), −x 쪽 10° 경사로, ±y 옆면 = 낭떠러지.
기대: ±y 옆면 가장자리(및 경사로 옆면 중 높이 > 0.22 인 윗부분)에만 장애물. 계단 내려가는 쪽·계단 1단 앞·경사로 쪽엔 0.
밀도 100% / 30% (모든 면 점을 무작위로 남김) + '가림'(단 옆 0.35 m 안 바닥 점 없음 — 단 위에서 내려다볼 때 모서리에 가림).
출력: synth_cliff.json (C++ 대조용 격자도 synth_grid_*.txt 로 저장)"""
import sys, json, numpy as np
sys.path.insert(0, '/home/ilovehan9283/ws/night_20261104/ramp'); sys.path.insert(0, '/home/ilovehan9283/ws/night_20261104/ramp/cliff')
from terrain_core import grid_stats, classify, traversable, ego_rule
from terrain_cliff import cliff_edges, cliff_points

OUT = '/home/ilovehan9283/ws/night_20261104/ramp/cliff'
CELL, HALF, TOP, STEP, TREAD, PW = 0.05, 4.0, 0.34, 0.17, 0.30, 0.60
RAMP_L = TOP / np.tan(np.radians(10.0))


def surface_z(x, y):
    """지형 높이 (바닥 0)."""
    z = np.zeros_like(x)
    iny = np.abs(y) <= PW
    z = np.where(iny & (np.abs(x) <= PW), TOP, z)
    z = np.where(iny & (x > PW) & (x <= PW + TREAD), STEP, z)
    rx = (x < -PW) & (x >= -PW - RAMP_L)
    z = np.where(iny & rx, TOP - (-PW - x) * np.tan(np.radians(10.0)), z)
    return z


def make_cloud(rng, density, occl=False, sp=0.02):
    g = np.arange(-4.0, 4.0, sp); X, Y = np.meshgrid(g, g, indexing='ij'); X = X.ravel(); Y = Y.ravel()
    Z = surface_z(X, Y)
    P = [np.c_[X, Y, Z + rng.normal(0, 0.005, len(X))]]
    # 세로 면: 단·계단·경사로 옆면(±y), 계단 챌판(x=PW, x=PW+TREAD)
    zs = np.arange(0, TOP, sp)
    for sy in (-PW, PW):
        xs = np.arange(-PW - RAMP_L, PW + TREAD, sp)
        XX, ZZ = np.meshgrid(xs, zs, indexing='ij'); XX = XX.ravel(); ZZ = ZZ.ravel()
        k = ZZ <= surface_z(XX, np.full_like(XX, 0.0))
        P.append(np.c_[XX[k], np.full(k.sum(), sy * (1 + 1e-3)), ZZ[k]])
    ys = np.arange(-PW, PW, sp)
    for x0, z0, z1 in ((PW, STEP, TOP), (PW + TREAD, 0.0, STEP)):
        YY, ZZ = np.meshgrid(ys, np.arange(z0, z1, sp), indexing='ij')
        P.append(np.c_[np.full(YY.size, x0 + 1e-3), YY.ravel(), ZZ.ravel()])
    W = np.concatenate(P)
    if occl:   # 단 위에서는 단 옆(±y) 0.35 m 안 바닥이 모서리에 가려 안 보인다
        hide = (np.abs(W[:, 1]) >= PW) & (np.abs(W[:, 1]) < PW + 0.35) & (W[:, 0] > -PW - RAMP_L) & (W[:, 0] < PW + TREAD) & (W[:, 2] < TOP - 0.03)   # 옆 바닥 + 옆면
        W = W[~hide]
    keep = rng.random(len(W)) < density
    return W[keep]


def zone_of(x, y):
    ay = np.abs(y)
    if ay > PW - 0.10 and ay <= PW and -PW <= x <= PW: return '단 옆면(±y)'
    if ay < PW - 0.10 and PW - 0.10 < x <= PW: return '계단 내려가는 쪽(단 앞)'
    if ay < PW - 0.10 and PW + TREAD - 0.10 < x <= PW + TREAD: return '계단 1단 앞(0.17)'
    if ay < PW - 0.10 and -PW - 0.10 <= x < -PW + 0.10: return '경사로 시작(단 뒤)'
    if ay > PW - 0.10 and x < -PW:
        h = TOP - (-PW - x) * np.tan(np.radians(10.0))
        return '경사로 옆면 h>0.24' if h > 0.24 else ('경사로 옆면 h<0.20' if h < 0.20 else '경사로 옆면 0.20~0.24')
    if ay > PW - 0.10 and PW < x <= PW + TREAD: return '계단 1단 옆면(0.17)'
    return '기타'


def run(case, density, occl, min_low, seed=0, save=False, band=None):
    rng = np.random.default_rng(seed)
    W = make_cloud(rng, density, occl)
    me = np.array([0.0, 0.0]); rz = TOP + 0.31; org = me - HALF; nn = int(2 * HALF / CELL)
    hi, lo, _ = grid_stats(W, org, (nn, nn), CELL, rz + 1.10)
    _, _, pk = classify(hi, CELL)
    plane = ((0.0, 0.0, TOP), (0.0, 0.0, 1.0))
    T = traversable(hi, lo, CELL, me, org, rz, hop=0.15, seed_r=1.5, plane=plane, p=pk)
    cl, ed, em = cliff_edges(hi, T, CELL, 0.22, 0.30, 0.05, min_low, band=band, plane=plane, origin=org)
    CP = cliff_points(cl, org, CELL, rz)
    ego_ok = int(ego_rule(CP, (0, 0, rz), 0.0).sum()) if len(CP) else 0
    # 구역별: 가장자리 칸 수, 낭떠러지 칸 수
    Z = {}
    for name, M in (('edge', ed), ('cliff', cl), ('empty', em)):
        for i, j in np.argwhere(M):
            x, y = org + (np.array([i, j]) + 0.5) * CELL; z_ = zone_of(x, y)
            Z.setdefault(z_, {'edge': 0, 'cliff': 0, 'empty': 0})[name] += 1
    side = Z.get('단 옆면(±y)', {'edge': 0, 'cliff': 0})
    # 옆면 덮임: 단 옆 가장자리 선(x 따라 5 cm 간격) 중 0.10 m 안에 낭떠러지 점이 있는 비율
    xs = np.arange(-PW + 0.025, PW, 0.05); L = np.r_[np.c_[xs, np.full_like(xs, PW - 0.025)], np.c_[xs, np.full_like(xs, -PW + 0.025)]]
    if len(CP):
        from scipy.spatial import cKDTree
        cov = float((cKDTree(CP[:, :2]).query(L)[0] <= 0.10).mean())
    else:
        cov = 0.0
    # 정답 대조: 칸 안 최고 지형 − 반경 0.30 m 안 최저 지형 > 0.22 이면 진짜 낭떠러지 근처
    th = np.linspace(0, 2 * np.pi, 24, endpoint=False); rr = np.array([0.05, 0.10, 0.15, 0.20, 0.25, 0.30])
    DX = (rr[:, None] * np.cos(th)[None]).ravel(); DY = (rr[:, None] * np.sin(th)[None]).ravel()
    cxy = org + (np.argwhere(cl) + 0.5) * CELL; nfalse = 0; npath = 0
    for x, y in cxy:
        gx, gy = np.meshgrid(x + np.linspace(-0.024, 0.024, 5), y + np.linspace(-0.024, 0.024, 5))
        top = surface_z(gx.ravel(), gy.ravel()).max(); low = surface_z(x + DX, y + DY).min()
        nfalse += (top - low) <= 0.22
        npath += abs(y) < 0.30 and -PW - RAMP_L - 0.3 < x < PW + TREAD + 0.3
    if save:
        np.savetxt(f'{OUT}/synth_grid_{case}.txt', np.c_[np.nan_to_num(hi.ravel(), nan=-999.0), T.ravel().astype(int), cl.ravel().astype(int), ed.ravel().astype(int), em.ravel().astype(int)], fmt='%.6f %d %d %d %d')
    return dict(case=case, density=density, occl=occl, min_low=min_low, band=band, T칸=int(T.sum()), 가장자리=int(ed.sum()), 낭떠러지=int(cl.sum()),
                빈가장자리=int(em.sum()), 낭떠러지점_ego통과=ego_ok, 거짓낭떠러지=int(nfalse), 계단길_경사로길_낭떠러지=int(npath), 단옆면_덮임=round(cov, 3), 구역=Z)


if __name__ == '__main__':
    R = []
    for case, dens, occl in (('d100', 1.0, False), ('d30', 0.3, False), ('d10', 0.1, False), ('d100_가림', 1.0, True), ('d30_가림', 0.3, True)):
        for ml, bd in ((1, None), (2, None), (1, 0.20)):
            r = run(case, dens, occl, ml, save=(ml == 1 and bd is None), band=bd)
            R.append(r)
            print(f"{case:10s} min_low={ml} band={bd}  T {r['T칸']:5d}  가장자리 {r['가장자리']:4d}  낭떠러지 {r['낭떠러지']:4d}  빈가장자리 {r['빈가장자리']:4d}  "
                  f"단옆면 덮임 {r['단옆면_덮임']:.3f}  거짓 {r['거짓낭떠러지']}  계단·경사로길(|y|<0.3) {r['계단길_경사로길_낭떠러지']}  ego통과 {r['낭떠러지점_ego통과']}/{r['낭떠러지']}")
            for k, v in sorted(r['구역'].items()):
                print(f"      {k:22s} 가장자리 {v['edge']:4d}  낭떠러지 {v['cliff']:4d}  빈 {v['empty']:4d}")
    json.dump(R, open(f'{OUT}/synth_cliff.json', 'w'), ensure_ascii=False, indent=1)
