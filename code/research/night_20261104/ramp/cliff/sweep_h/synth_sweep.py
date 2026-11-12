#!/usr/bin/env python3
"""합성 시험(synth_cliff.py 의 단·계단·경사로 지형)을 [H] 갈래로: drop {0.22,0.26,0.30} × 높은곳만 {끔,켬} × 방향 {전방위, 앞 ±60° 제외},
로봇 yaw 0°(계단 쪽을 봄) / 90°(+y 옆면 낭떠러지를 봄). 연속 2 프레임은 정지 합성이라 1 과 같다(같은 격자 두 번) — 따로 안 셈.
옆면 덮임 = 단 ±y 가장자리 선(5 cm 간격) 중 0.10 m 안에 낭떠러지 점이 있는 비율 (+y, −y 따로).
'로봇 경사로 위' 경우: 경사로 중간(x = −1.6, z 0.17 지형)에 서서 단 쪽을 봄 — 높은곳만 이 무엇을 끄는지 확인."""
import sys, json, math, numpy as np
sys.path.insert(0, '/home/ilovehan9283/ws/night_20261104/ramp'); sys.path.insert(0, '/home/ilovehan9283/ws/night_20261104/ramp/cliff')
from terrain_core import grid_stats, classify, traversable, ego_rule
from terrain_cliff import cliff_edges
from scipy.spatial import cKDTree
import synth_cliff as SC
OUT = '/home/ilovehan9283/ws/night_20261104/ramp/cliff/sweep_h'
CELL, HALF = 0.05, 4.0
PW, TOP, TREAD = SC.PW, SC.TOP, SC.TREAD


def gates(cl, org, me, yaw, foot_z, hi, hg, di, HG=0.15):
    nn = hi.shape[0]
    ij = np.argwhere(cl); xy = org + (ij + 0.5) * CELL
    ii, jj = np.meshgrid(np.arange(nn), np.arange(nn), indexing='ij')
    R3 = np.hypot(org[0] + (ii + 0.5) * CELL - me[0], org[1] + (jj + 0.5) * CELL - me[1]) <= 3.0
    hv = hi[R3 & np.isfinite(hi)]; gnd = float(np.percentile(hv, 2)) if len(hv) else foot_z
    sel = np.ones(len(ij), bool)
    if hg and foot_z - gnd < HG: sel[:] = False
    if di == 'nofront':
        b = np.arctan2(xy[:, 1] - me[1], xy[:, 0] - me[0]) - yaw
        sel &= np.abs((b + np.pi) % (2 * np.pi) - np.pi) > math.radians(60)
    return xy[sel], foot_z - gnd


def run(case, dens, occl, yaw_deg, where='top', seed=0):
    rng = np.random.default_rng(seed); W = SC.make_cloud(rng, dens, occl)
    if where == 'top':
        me = np.array([0.0, 0.0]); fz = TOP
    else:   # 경사로 중간
        me = np.array([-PW - SC.RAMP_L / 2, 0.0]); fz = float(SC.surface_z(np.array([me[0]]), np.array([0.0]))[0])
    rz = fz + 0.31; org = me - HALF; nn = int(2 * HALF / CELL); yaw = math.radians(yaw_deg)
    hi, lo, _ = grid_stats(W, org, (nn, nn), CELL, rz + 1.10)
    _, _, pk = classify(hi, CELL)
    plane = ((me[0], me[1], fz), (0.0, 0.0, 1.0))
    T = traversable(hi, lo, CELL, me, org, rz, hop=0.15, seed_r=1.5, plane=plane, p=pk)
    xs = np.arange(-PW + 0.025, PW, 0.05)
    Lp = np.c_[xs, np.full_like(xs, PW - 0.025)]; Lm = np.c_[xs, np.full_like(xs, -PW + 0.025)]
    out = []
    for dr in (0.22, 0.26, 0.30):
        cl, ed, em = cliff_edges(hi, T, CELL, dr, 0.30, 0.05, 1)
        for hg in (0, 1):
            for di in ('all', 'nofront'):
                xy, fg = gates(cl, org, me, yaw, fz, hi, hg, di)
                if len(xy):
                    kd = cKDTree(xy); cp = float((kd.query(Lp)[0] <= 0.10).mean()); cm = float((kd.query(Lm)[0] <= 0.10).mean())
                else:
                    cp = cm = 0.0
                npath = int(sum(abs(y) < 0.30 and -PW - SC.RAMP_L - 0.3 < x < PW + TREAD + 0.3 for x, y in xy))
                out.append(dict(case=case, where=where, yaw=yaw_deg, drop=dr, high=hg, dir=di, n=int(len(xy)), cov_py=round(cp, 3), cov_my=round(cm, 3),
                                path_cells=npath, foot_gnd=round(fg, 3)))
    return out


if __name__ == '__main__':
    R = []
    for case, dens, occl in (('d100', 1.0, False), ('d30', 0.3, False), ('d10', 0.1, False), ('d100_가림', 1.0, True)):
        for yd in (0, 90):
            R += run(case, dens, occl, yd)
    R += run('d100', 1.0, False, 0, where='ramp')
    json.dump(R, open(f'{OUT}/synth_sweep.json', 'w'), ensure_ascii=False, indent=1)
    print('| 경우 | 로봇 | yaw | drop | 높은곳만 | 방향 | 낭떠러지 칸 | +y 옆면 덮임 | −y 옆면 덮임 | 계단·경사로 길 칸 | 발−최저지면 |')
    print('|---|---|---|---|---|---|---|---|---|---|---|')
    for r in R:
        print(f"| {r['case']} | {'단 위' if r['where'] == 'top' else '경사로 중간'} | {r['yaw']}° | {r['drop']:.2f} | {'켬' if r['high'] else '끔'} | {'앞±60° 제외' if r['dir'] == 'nofront' else '전방위'} | "
              f"{r['n']} | {r['cov_py']:.3f} | {r['cov_my']:.3f} | {r['path_cells']} | {r['foot_gnd']:.2f} |")
