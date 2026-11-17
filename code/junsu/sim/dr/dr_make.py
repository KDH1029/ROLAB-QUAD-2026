#!/usr/bin/env python3
"""콘 배치 무작위화(11-06): 간격은 그대로, 콘 전체를 입구 쪽(+y, rabain 쪽)으로 d m 당김 + 콘마다 놓는 오차(σ jit).
콘은 시뮬 벽 칸에 박혀 있다 → 콘 점 0.10 m 안 칸을 그 콘과 같이 옮긴다. 출력 dr/L_<d>_<seed>_{walls,cones}.npy + 슬라럼만 도는 짧은 경로."""
import numpy as np, json, math, sys, os
from scipy.spatial import cKDTree
S = os.path.dirname(os.path.dirname(os.path.abspath(__file__))); D = S + '/dr'
W = np.load(S + '/sim_walls_rec1650_noiso.npy'); C = np.load(S + '/sim_cones_rec1650.npy')
d, j = cKDTree(C[:, :2]).query(W[:, :2]); cone = d < 0.10
SH = [float(x) for x in (sys.argv[1] if len(sys.argv) > 1 else '0,0.2,0.4,0.6,0.8').split(',')]
SEEDS = [int(x) for x in (sys.argv[2] if len(sys.argv) > 2 else '1,2').split(',')]; JIT = 0.05
out = []
for sh in SH:
    for sd in SEEDS:
        rng = np.random.default_rng(sd * 100 + int(sh * 10)); off = rng.normal(0, JIT, (len(C), 2)); off[:, 1] += sh
        Cn = C + off; Wc = W[cone].copy(); Wc[:, :2] += off[j[cone]]
        nm = f'L{int(round(sh * 10)):02d}_{sd}'; np.save(f'{D}/{nm}_walls.npy', np.vstack([W[~cone], Wc])); np.save(f'{D}/{nm}_cones.npy', Cn); out.append(nm)
r = json.load(open(S + '/route_v2_1650_hybrid_sim.json')); R = r['route']; lab = [p['label'] for p in R]
i0 = lab.index('stopline1'); i1 = lab.index('line4')
sub = [dict(R[i0], label='start')] + [dict(p) for p in R[i0 + 1:i1 + 1]]; sub[-1]['label'] = 'exit'   # 러너 규칙: 마지막은 exit (EXIT_X=5.2 로 그 앞 선)
sub[0]['yaw'] = math.degrees(math.atan2(sub[1]['y'] - sub[0]['y'], sub[1]['x'] - sub[0]['x']))
json.dump(dict({k: v for k, v in r.items() if k != 'route'}, route=sub), open(D + '/route_slalom_only.json', 'w'), ensure_ascii=False, indent=1)
print(' '.join(out)); print('경로', [p['label'] for p in sub])
