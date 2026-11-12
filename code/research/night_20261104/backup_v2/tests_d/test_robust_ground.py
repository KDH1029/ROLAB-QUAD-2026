#!/usr/bin/env python3
"""① 견고 지면표 z_ref 시험 — 고치기 전(robust_ground.py.orig_0740, 최빈값) vs 고친 것(robust_ground.py, 출발점 바닥).

A 실제 지도 둘(앞 270초, 전체)       : z_ref 값과 경고 여부
B 바닥을 덜 찍은 지도 (출발점 1.5 m 밖 칸의 f 비율에서 z<0 점을 지움) / B2 같은 칸에서 z<1.9 점을 다 지움(천장만 남은 칸) : 최빈값이 천장으로 뒤집히는가,
                                         FAST-LIO 궤적 위 지면값이 천장(>1.0 m)으로 나오는 비율
C 출발점 반경 1.5 m 가 비어 있는 지도  : '가까운 유효 칸' 대체가 바닥을 잡는가
D 출발점 주변만 바닥이 빠진 지도       : 출발점 값이 틀어질 때 최빈값과 비교 경고가 뜨는가
읽기만: 지도 두 개(.npz), v_gosu/backup/backup_core.py   쓰기: 이 폴더 test_robust_ground_result.json
"""
import sys, os, json, importlib.util, importlib.machinery, numpy as np
V2 = '/home/ilovehan9283/ws/night_20261104/backup_v2'
sys.path.insert(0, '/home/ilovehan9283/ws/v_gosu/backup')
from backup_core import load_map, voxel


def _load(name, path):
    ld = importlib.machinery.SourceFileLoader(name, path)
    sp = importlib.util.spec_from_file_location(name, path, loader=ld); m = importlib.util.module_from_spec(sp); ld.exec_module(m); return m


OLD = _load('rg_orig', f'{V2}/robust_ground.py.orig_0740').RobustGroundGrid
NEW = _load('rg_new', f'{V2}/robust_ground.py').RobustGroundGrid
FULL = '/home/ilovehan9283/ws/v_gosu/eval/out/gosu1030v2_map.npz'
HALF = '/home/ilovehan9283/ws/night_20261104/backup_eval/map_first270s_fastlio.npz'
traj = np.load(FULL)['odom'][::4, 2:4]          # FAST-LIO 궤적 xy (5 Hz 표본)


def ceil_frac(g):
    v = [g.raw_at(x, y) for x, y in traj]
    v = np.array([np.nan if a is None else a for a in v], float)
    return round(float(np.nanmean(v > 1.0)), 4), round(float(np.isnan(v).mean()), 4)


def run(tag, mp, ref=None):
    o, n = OLD(mp), NEW(mp, verbose=False)
    co, cn = ceil_frac(o), ceil_frac(n)
    r = dict(점=len(mp), 예전_z_ref=round(o.z_ref, 3), 새_z_ref=round(n.z_ref, 3), 새_출처=n.z_ref_src, 그냥중앙값=round(n.z_ref_start_median, 3),
             경고=n.z_ref_warn is not None, 예전_궤적위_천장지면비율=co[0], 새_궤적위_천장지면비율=cn[0], 예전_모름=co[1], 새_모름=cn[1])
    print(tag, r, flush=True); return r


res = {}
mfull, mhalf = voxel(load_map(FULL), 0.05), voxel(load_map(HALF), 0.05)
res['A_전체지도'] = run('A 전체', mfull)
res['A_앞270초지도'] = run('A 앞270', mhalf)

rng = np.random.default_rng(0)
for base_name, base in (('전체', mfull), ('앞270', mhalf)):
    k = np.floor(base[:, :2] / 0.10).astype(np.int64)
    _, inv = np.unique(k, axis=0, return_inverse=True); inv = inv.ravel()
    ncell = inv.max() + 1
    cen = np.zeros((ncell, 2)); np.add.at(cen, inv, base[:, :2]); cen /= np.bincount(inv)[:, None]
    far = np.hypot(cen[:, 0], cen[:, 1]) > 1.5
    u = rng.random(ncell)
    for f in (0.5, 0.7, 0.8, 0.9):
        drop = far[inv] & (u[inv] < f) & (base[:, 2] < 0.0)
        res[f'B_{base_name}_바닥{int(f * 100)}%지움'] = run(f'B {base_name} f={f}', base[~drop])
    for f in (0.3, 0.5, 0.6, 0.7):                            # B2: 칸의 f 비율이 '천장만 찍힌 칸' (z<1.9 점 전부 지움) — 검토가 말한 뒤집힘
        drop = far[inv] & (u[inv] < f) & (base[:, 2] < 1.9)
        res[f'B2_{base_name}_천장만{int(f * 100)}%'] = run(f'B2 {base_name} 천장만 f={f}', base[~drop])
    near = np.hypot(base[:, 0], base[:, 1]) <= 1.6
    res[f'C_{base_name}_출발점비움'] = run(f'C {base_name} 출발점 비움', base[~near])
    res[f'D_{base_name}_출발점바닥만지움'] = run(f'D {base_name} 출발점 바닥만 지움', base[~(near & (base[:, 2] < 0.0))])
json.dump(res, open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'test_robust_ground_result.json'), 'w'), ensure_ascii=False, indent=1)
