#!/usr/bin/env python3
"""몸 높이 변화 표본(329.55·425.05 s) 진단 — G1(XT16+L1) vs G2(+깊이) 씨앗·연결 비교. 입력: eval_depth_forks.py DUMP_T 로 만든 dump_*.npz"""
import numpy as np, sys, json, glob, os
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
from terrain_core import classify, _disk
from scipy import ndimage
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
CELL = 0.05


def comps(hi, lo, me, org, plane):
    _, _, p = classify(hi, CELL); valid = np.isfinite(hi)
    bad = ndimage.binary_dilation(valid & (((hi - lo) > 0.20) | (p > 0.10)), structure=_disk(1))
    ij = np.argwhere(valid & ~bad); xy = org + (ij + 0.5) * CELL; z = hi[ij[:, 0], ij[:, 1]]
    pr = cKDTree(xy).query_pairs(0.15, output_type='ndarray'); pr = pr[np.abs(z[pr[:, 0]] - z[pr[:, 1]]) <= 0.20]
    _, lab = connected_components(coo_matrix((np.ones(len(pr)), (pr[:, 0], pr[:, 1])), shape=(len(ij), len(ij))), directed=False)
    (fx, fy, fz), (nx, ny, nz) = plane; zp = fz - (nx * (xy[:, 0] - fx) + ny * (xy[:, 1] - fy)) / nz; dd = np.hypot(*(xy - me).T)
    near = dd <= 1.5; resid = z - zp; tol = 0.08 + 0.05 * dd; seeds = np.where(near & (np.abs(resid) <= tol))[0]
    return dict(ij=ij, lab=lab, seeds=seeds, resid=resid, tol=tol, T=int(np.isin(lab, np.unique(lab[seeds])).sum()) if len(seeds) else 0)


out = {}
for f in sorted(glob.glob(f'{HERE}/dump_*.npz')):
    d = np.load(f); me, org = d['me'], d['org']; plane = (tuple(d['foot']), tuple(d['nrm']))
    hi2, lo2 = np.fmax(d['hiXL'], d['hiD']), np.fmin(d['loXL'], d['loD'])
    c1 = comps(d['hiXL'], d['loXL'], me, org, plane); c2 = comps(hi2, lo2, me, org, plane)
    pos2 = {tuple(x): i for i, x in enumerate(c2['ij'])}; s1 = c1['seeds']
    r2 = [float(c2['resid'][pos2[tuple(c1['ij'][s])]]) for s in s1 if tuple(c1['ij'][s]) in pos2]
    dh = [float(hi2[tuple(c1['ij'][s])] - d['hiXL'][tuple(c1['ij'][s])]) for s in s1]
    out[os.path.basename(f)] = dict(
        G1_씨앗=len(s1), G2_씨앗=len(c2['seeds']), G1_통과면칸=c1['T'], G2_통과면칸=c2['T'],
        G1씨앗_잔차_중앙=round(float(np.median(c1['resid'][s1])), 3) if len(s1) else None, 씨앗허용_중앙=round(float(np.median(c1['tol'][s1])), 3) if len(s1) else None,
        같은칸_G2잔차_중앙=round(float(np.median(r2)), 3) if r2 else None, 같은칸_깊이로_올라간높이_중앙=round(float(np.median(dh)), 3) if dh else None,
        몸높이_로봇z_minus_G1씨앗바닥=round(float(d['rz'] - np.median(d['hiXL'][tuple(c1['ij'][s1].T)])), 3) if len(s1) else None)
print(json.dumps(out, ensure_ascii=False, indent=1)); json.dump(out, open(f'{HERE}/debug_seed.json', 'w'), ensure_ascii=False, indent=1)
