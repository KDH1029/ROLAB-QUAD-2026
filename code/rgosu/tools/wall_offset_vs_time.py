#!/usr/bin/env python3
"""벽이 여러 겹이 되는 원인 가르기 — '시간이 지나 위치가 흘렀다'(드리프트) 인지 '같은 벽을 반대편에서 봤다'(벽 두께) 인지.
입력: replay_wallfilter_check.sh 출력 폴더(/mission/wall_cloud 는 camera_init 좌표, /Odometry).
방법: 녹화를 CH 초 조각으로 자르고, 조각 i 의 벽(2 cm 칸, 국소 평면 법선을 '본 쪽'으로 향하게) 에 조각 j 의 벽 칸이 얼마나 떨어져 있는지 잰다.
  같은 쪽에서 본 쌍(시선 각도 차 < 60°)  → 어긋남이 시간 간격 Δt 에 따라 커지면 드리프트
  반대쪽에서 본 쌍(시선 각도 차 > 120°) → 부호 있는 어긋남이 일정하게 음수(뒤쪽)면 벽 두께(진짜 두 면)
사용: wall_offset_vs_time.py <폴더> [조각 초=10] [최소 m=1] [최대 m=4]"""
import sys, os, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2 as pc2
from scipy.spatial import cKDTree

d = os.path.expanduser(sys.argv[1]); CH = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
RMIN = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0; RMAX = float(sys.argv[4]) if len(sys.argv) > 4 else 4.0
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=d, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
Od, Pc = get_message('nav_msgs/msg/Odometry'), get_message('sensor_msgs/msg/PointCloud2')
odo, clouds = [], []
while r.has_next():
    tp, raw, _ = r.read_next()
    if tp == '/Odometry':
        m = deserialize_message(raw, Od); p = m.pose.pose.position
        odo.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y, p.z))
    elif tp == '/mission/wall_cloud':
        m = deserialize_message(raw, Pc); a = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True)
        clouds.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, a[:, :3].astype(np.float64)))
odo = np.array(odo); odo = odo[np.argsort(odo[:, 0])]; t0 = odo[0, 0]
G = 0.02
chunks = {}   # (k, 'all'|0|1) -> {cell: [sum_xy(2), sum_bearing(2), n]}
RAW = {}      # k -> 벽 점(3D) 목록 — 조각 빼고 다시 쌓기용
for t, a in clouds:
    if len(a) == 0: continue
    i = min(np.searchsorted(odo[:, 0], t), len(odo) - 1); rob = odo[i, 1:3]
    v = a[:, :2] - rob; rr = np.hypot(v[:, 0], v[:, 1]); s = (rr > RMIN) & (rr < RMAX)
    if not s.any(): continue
    a3 = a[s]; a, v, rr = a[s][:, :2], v[s], rr[s]; b = v / rr[:, None]
    k_ = int((t - t0) // CH); RAW.setdefault(k_, []).append(a3)
    k = int((t - t0) // CH); fi = len(chunks.get(('n', k), [])) ; chunks.setdefault(('n', k), []).append(0)
    for D in (chunks.setdefault((k, 'all'), {}), chunks.setdefault((k, fi % 2), {})):
      for c, p, bb in zip(map(tuple, np.floor(a / G).astype(int).tolist()), a, b):
        e = D.get(c)
        if e is None: D[c] = [p.copy(), bb.copy(), 1]
        else: e[0] += p; e[1] += bb; e[2] += 1

def build(D):
    P = np.array([e[0] / e[2] for e in D.values()]); B = np.array([e[1] / max(np.hypot(*e[1]), 1e-9) for e in D.values()])
    T = cKDTree(P); n = np.full((len(P), 2), np.nan); mu = np.full((len(P), 2), np.nan)
    for i, nb in enumerate(T.query_ball_point(P, 0.10)):
        if len(nb) < 5: continue
        Q = P[nb]; m0 = Q.mean(0); ev, evec = np.linalg.eigh((Q - m0).T @ (Q - m0) / len(nb))
        if ev[0] > 0.1 * ev[1]: continue          # 직선(벽)이 아니면 버림 — 모서리·덩어리
        nn = evec[:, 0]
        if nn @ (-B[i]) < 0: nn = -nn               # 법선 = 벽에서 관찰자 쪽
        n[i] = nn; mu[i] = m0
    g = np.isfinite(n[:, 0])
    return dict(P=P[g], B=B[g], n=n[g], mu=mu[g], T=cKDTree(P[g]))


def pair(A, Bc, side):
    """Bc 를 A 에 강체 정합(점-평면 ICP, 2D). 반환: 정합 전/후 |어긋남| 중앙, 이동 m, 회전 deg, 짝 수"""
    Pj = Bc['P'].copy(); R = np.eye(2); t = np.zeros(2); before = None
    for it in range(12):
        Q = Pj @ R.T + t; dist, idx = A['T'].query(Q, distance_upper_bound=0.15); ok = np.isfinite(dist)
        if ok.sum() < 40: return None
        ia = idx[ok]; na = A['n'][ia]; nb_ = Bc['n'][ok] @ R.T
        c_n = np.abs(np.einsum('ij,ij->i', na, nb_)); c_b = np.einsum('ij,ij->i', A['B'][ia], Bc['B'][ok] @ R.T)
        sel = (c_n > np.cos(np.radians(25))) & ((c_b > 0.5) if side == 'same' else (c_b < -0.5))
        if sel.sum() < 40: return None
        q = Q[ok][sel]; m = A['mu'][ia][sel]; nn = na[sel]; e = np.einsum('ij,ij->i', q - m, nn)
        if before is None: before = float(np.median(np.abs(e)))
        if side == 'opp': return before, float(np.median(e)), int(sel.sum())
        w = np.abs(e) < max(3 * np.median(np.abs(e)), 0.01)
        J = np.c_[nn, nn[:, 0] * (-q[:, 1]) + nn[:, 1] * q[:, 0]]
        x, *_ = np.linalg.lstsq(J[w], -e[w], rcond=None)
        dR = np.array([[np.cos(x[2]), -np.sin(x[2])], [np.sin(x[2]), np.cos(x[2])]]); R = dR @ R; t = dR @ t + x[:2]
    return before, float(np.median(np.abs(e))), float(np.hypot(*t)), float(np.degrees(np.arctan2(R[1, 0], R[0, 0]))), int(sel.sum())


ks = sorted(k for (k, tag) in chunks if tag == 'all' and len(chunks[(k, 'all')]) > 200)
M = {k: build(chunks[(k, 'all')]) for k in ks}
print(f'조각 {CH:.0f} 초 × {len(ks)} 개 · 사거리 {RMIN}~{RMAX} m · 벽 칸 2 cm · 같은 방향 벽(법선 25° 안)끼리만 짝')
fl = []
for k in ks:
    if (k, 0) in chunks and (k, 1) in chunks and len(chunks[(k, 0)]) > 200 and len(chunks[(k, 1)]) > 200:
        r_ = pair(build(chunks[(k, 0)]), build(chunks[(k, 1)]), 'same')
        if r_: fl.append(r_)
print(f'잡음 바닥(같은 10 초 안 짝/홀 프레임): 어긋남 {np.median([x[0] for x in fl]) * 100:.2f} cm · 정합 이동 {np.median([x[2] for x in fl]) * 100:.2f} cm · 회전 {np.median(np.abs([x[3] for x in fl])):.2f}°  ({len(fl)} 조각)')
rows = []
for ii, ki in enumerate(ks):
    for kj in ks[ii + 1:]:
        r_ = pair(M[ki], M[kj], 'same')
        if r_: rows.append(((kj - ki) * CH,) + r_)
print('\n[같은 쪽에서 본 같은 벽] 시간 간격별 — 드리프트면 정합 이동이 Δt 따라 커진다')
print(' Δt | 쌍 | 정합 전 어긋남 cm | 정합 이동 cm | 회전 ° | 정합 후 어긋남 cm')
for lo, hi in [(10, 10), (20, 30), (40, 60), (70, 120), (130, 200), (210, 999)]:
    q = [x for x in rows if lo <= x[0] <= hi]
    if q: print(f' {lo:>3.0f}~{hi:>3.0f} s | {len(q):>3} | {np.median([x[1] for x in q]) * 100:5.2f} | {np.median([x[3] for x in q]) * 100:5.2f} | {np.median(np.abs([x[4] for x in q])):4.2f} | {np.median([x[2] for x in q]) * 100:5.2f}')
op = []
for ii, ki in enumerate(ks):
    for kj in ks[ii + 1:]:
        r_ = pair(M[ki], M[kj], 'opp')
        if r_: op.append(((kj - ki) * CH,) + r_)
print('\n[반대쪽에서 본 같은 벽] — 부호가 늘 음수(관찰자 반대편)면 벽 두께')
if op: print(f' 쌍 {len(op)} · |어긋남| 중앙 {np.median([x[1] for x in op]) * 100:.2f} cm · 부호 중앙 {np.median([x[2] for x in op]) * 100:+.2f} cm · 음수 쌍 {np.mean([x[2] < 0 for x in op]) * 100:.0f}%')
else: print(' 없음')

# ── 어느 조각이 튀었나: 조각마다 다른 조각들과의 어긋남(같은 쪽·같은 방향 벽) 중앙값
per = {k: [] for k in ks}
for x in rows:
    pass
for ii, ki in enumerate(ks):
    for kj in ks[ii + 1:]:
        r_ = pair(M[ki], M[kj], 'same')
        if r_: per[ki].append(r_[0]); per[kj].append(r_[0])
print('\n[조각별] 다른 조각과의 어긋남 중앙 cm · 그 10 초 동안 로봇: 위치, 높이 변화, 이동 거리, 회전')
bad = []
for k in ks:
    sel = (odo[:, 0] >= t0 + k * CH) & (odo[:, 0] < t0 + (k + 1) * CH); o = odo[sel]
    if not per[k] or len(o) < 2: continue
    med = np.median(per[k]); dz = o[:, 3].max() - o[:, 3].min(); path = np.hypot(*np.diff(o[:, 1:3], axis=0).T).sum()
    tag = ' ← 튐' if med > 0.04 else ''
    if med > 0.04: bad.append(k)
    print(f' {k * CH:5.0f}~{(k + 1) * CH:3.0f} s | {med * 100:5.2f} ({len(per[k])}쌍) | ({o[:, 1].mean():5.2f},{o[:, 2].mean():5.2f}) | 높이폭 {dz * 100:4.1f} cm | {path:4.2f} m{tag}')


def sharp(P):
    """compare_maps 와 같은 기준의 2D 벽 두께: 2 cm 칸, 반경 15 cm 이웃의 가장 얇은 방향 σ"""
    k2 = np.unique(np.round(P[:, :2] / 0.02).astype(np.int64), axis=0); W = k2.astype(float) * 0.02
    T = cKDTree(W); S = W[np.random.default_rng(0).choice(len(W), min(15000, len(W)), replace=False)]; sig = []
    for nb in T.query_ball_point(S, 0.15):
        if len(nb) < 8: continue
        Q = W[nb] - W[nb].mean(0); sig.append(np.sqrt(max(np.linalg.eigvalsh(Q.T @ Q / len(nb))[0], 0)))
    sig = np.array(sig); return np.median(sig) * 100, (sig < 0.02).mean() * 100, len(W)


allP = np.concatenate([np.concatenate(RAW[k]) for k in ks]); goodP = np.concatenate([np.concatenate(RAW[k]) for k in ks if k not in bad])
for name, P in (('전부', allP), (f'튄 조각 {len(bad)}개 뺌', goodP)):
    m_, th, n_ = sharp(P); print(f' 지도({name}): 벽두께σ {m_:.2f} cm · 얇은벽 {th:.1f}% · 칸 {n_}')
