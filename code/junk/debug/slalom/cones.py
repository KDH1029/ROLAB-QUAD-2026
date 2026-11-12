import json, math, os, sys
H = os.path.dirname(os.path.abspath(__file__)); sys.argv = [sys.argv[0], '6', '26']
src = open(os.path.join(H, 'sim_scurve.py')).read(); exec(compile(src[:src.index('def body_hits')], 'h', 'exec'))
from scipy.sparse.csgraph import connected_components
from scipy.sparse import coo_matrix
SL = np.array([[e['x'], e['y']] for e in R[6:25]]); ZLO, ZHI = SL.min(0) - 0.4, SL.max(0) + 0.4
z = (OB[:, 0] > ZLO[0] - 0.3) & (OB[:, 0] < ZHI[0] + 0.3) & (OB[:, 1] > ZLO[1] - 0.3) & (OB[:, 1] < ZHI[1] + 0.3); Q = OB[z]
pr = cKDTree(Q).query_pairs(0.09, output_type='ndarray'); g = coo_matrix((np.ones(len(pr)), (pr[:, 0], pr[:, 1])), shape=(len(Q), len(Q)))
nc, lab = connected_components(g, directed=False)
print('구역', ZLO.round(2), ZHI.round(2), '벽칸', len(Q))
for c in range(nc):
    P = Q[lab == c]
    if len(P) < 3: 
        print(f'  작은 덩어리 {len(P)}칸 ({P[:,0].mean():.2f},{P[:,1].mean():.2f})'); continue
    ext = P.max(0) - P.min(0)
    print(f'  덩어리 {c}: {len(P)}칸 중심 ({P[:,0].mean():.2f},{P[:,1].mean():.2f}) 크기 {ext[0]:.2f}×{ext[1]:.2f}')
print('찍은 점(#7~#25):', ' '.join(f'#{i+7}({e["x"]:.2f},{e["y"]:.2f})' for i, e in enumerate(R[6:25])))
