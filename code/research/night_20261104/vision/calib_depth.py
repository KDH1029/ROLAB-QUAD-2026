#!/usr/bin/env python3
"""D435i 깊이 → 몸체(FAST-LIO body) 외부파라미터 추정 — 여러 프레임 동시 점-평면 최소제곱 (FAST-LIO 지도 기준).

한 프레임은 대부분 바닥(평면) → x·y·yaw 가 안 잡힘. 벽이 보이는 여러 프레임을 한 번에 풀어야 6자유도가 선다.
시각: 카메라 stamp 는 로봇 PC 시계 → (bag 수신시각 − stamp) 의 하한 포락선을 직선 맞춤해 NUC 시각으로. 남는 지연 δ 는 격자 탐색.
초기값: 높이 0.417 m·아래 32.1°(11-03 바닥 평면 실측) → 몸체 원점(IMU, 바닥 위 약 0.31 m) 기준 z≈+0.11, x≈+0.30 가정
쓰기: 이 폴더 calib_depth.json
"""
import sys, math, bisect, json, numpy as np, rosbag2_py, warnings
warnings.simplefilter('ignore')
import small_gicp
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as Rot
sys.path.insert(0, '/home/ilovehan9283/ws/v_gosu/backup')
from backup_core import q2R, voxel

BAG = '/home/ilovehan9283/bags/gosu_20261030_001126'
NPZ = '/home/ilovehan9283/ws/v_gosu/eval/out/gosu1030v2_map.npz'
OUT = '/home/ilovehan9283/ws/night_20261104/vision'
FX = FY = 431.08; CX, CY = 422.28, 240.89; Wd, Hd = 848, 480
NF = int(sys.argv[1]) if len(sys.argv) > 1 else 80
SUBSET = sys.argv[2] if len(sys.argv) > 2 else 'all'     # all | odd | even (교차검증)
import os
CACHE = f'{OUT}/calib_frames_{NF}.npz'


def reader(topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics)); return r


# FAST-LIO 궤적을 bag 시각으로 (다른 스크립트와 같은 대응)
d = np.load(NPZ); odom, lid = d['odom'], d['lid']
FULL = (d['vox'][d['cnt'] >= 2].astype(np.float64) + 0.5) * float(d['V'])
hdr2bag = {}
r = reader(['/lidar_points'])
while r.has_next():
    tp, raw, t = r.read_next(); s, ns = np.frombuffer(raw[4:8], '<i4')[0], np.frombuffer(raw[8:12], '<u4')[0]
    hdr2bag[round(float(s) + float(ns) * 1e-9, 4)] = t * 1e-9
pairs = np.array([(w, hdr2bag[round(h, 4)]) for w, h in lid if round(h, 4) in hdr2bag])
Kp = float(np.median(pairs[:, 0] - pairs[:, 1])); wl = np.sort(lid[:, 0]); h_o = odom[:, 1]
dmed = float(np.median(h_o - wl[np.clip(np.searchsorted(wl, h_o) - 1, 0, len(wl) - 1)]))
tL = h_o - dmed - Kp; o = np.argsort(tL); tL = tL[o]; GT = odom[o]; t0 = tL[0]
Rs = Rot.from_quat(GT[:, 5:9])   # x y z w
yaw = np.unwrap(Rs.as_euler('zyx')[:, 0]); wz = np.gradient(yaw, tL)
def pose_at(ts):
    i = int(np.clip(np.searchsorted(tL, ts), 1, len(tL) - 1)); a = (ts - tL[i - 1]) / max(tL[i] - tL[i - 1], 1e-6); a = min(max(a, 0), 1)
    p = GT[i - 1, 2:5] * (1 - a) + GT[i, 2:5] * a
    from scipy.spatial.transform import Slerp
    R = Slerp([0, 1], Rot.from_quat(GT[i - 1:i + 1, 5:9]))([a])[0].as_matrix()
    return R, p

# ① 깊이 stamp 와 bag 시각 (원시 바이트 머리만)
st = []
r = reader(['/camera/depth/image_rect_raw'])
while r.has_next():
    tp, raw, t = r.read_next(); s, ns = np.frombuffer(raw[4:8], '<i4')[0], np.frombuffer(raw[8:12], '<u4')[0]
    st.append((float(s) + ns * 1e-9, t * 1e-9))
st = np.array(st); lag = st[:, 1] - st[:, 0]
bins = np.floor((st[:, 1] - st[0, 1]) / 10).astype(int); env = np.array([(st[bins == b, 1].mean(), lag[bins == b].min()) for b in np.unique(bins)])
A = np.polyfit(env[:, 0] - st[0, 1], env[:, 1], 1); print('stamp→NUC: 오프셋', round(A[1], 4), '기울기', f'{A[0] * 1e6:.1f} µs/s', '지연(포락선 위) 중앙', round(float(np.median(lag - np.polyval(A, st[:, 1] - st[0, 1]))) * 1e3, 1), 'ms')
def cam2nuc(s, b0=st[0, 1]): return s + np.polyval(A, (s + A[1]) - b0)

# ② 프레임 고르기: 회전 느림(|wz|<0.15 rad/s), 시간 고르게
tn = np.array([cam2nuc(s) for s in st[:, 0]])
okm = (tn > tL[0] + 3) & (tn < tL[-1] - 1)
wz_at = np.interp(tn, tL, np.abs(wz)); cand = np.where(okm & (wz_at < 0.15))[0]
pick = cand[np.linspace(0, len(cand) - 1, NF).astype(int)]; pick_set = set(pick.tolist()); print('후보', len(cand), '선택', len(pick))

# ③ 깊이 → 광학좌표 점 (4픽셀 간격, 0.3~3.0 m) — 캐시 있으면 재사용
if os.path.exists(CACHE):
    z_ = np.load(CACHE, allow_pickle=True); frames = list(z_['frames']); pick_set = set()
else:
    frames = None
uu, vv = np.meshgrid(np.arange(0, Wd, 4), np.arange(0, Hd, 4))
load = frames is None
if load: frames = []
i = -1
r = reader(['/camera/depth/image_rect_raw'])
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
while load and r.has_next():
    tp, raw, t = r.read_next(); i += 1
    if i not in pick_set: continue
    m = deserialize_message(raw, Image); D = np.frombuffer(m.data, np.uint16).reshape(Hd, Wd)[::4, ::4] * 1e-3
    ok = (D > 0.3) & (D < 3.0); z = D[ok]
    P = np.c_[(uu[ok] - CX) * z / FX, (vv[ok] - CY) * z / FY, z]
    frames.append((tn[i], voxel(P, 0.03)))
if load: np.savez(CACHE, frames=np.array(frames, dtype=object))
if SUBSET != 'all': frames = frames[(1 if SUBSET == 'odd' else 0)::2]
print('프레임', len(frames), SUBSET, '점/프레임 중앙', int(np.median([len(f[1]) for f in frames])))

# ④ 지도: 3 cm, 법선
M = voxel(FULL, 0.03); pc = small_gicp.PointCloud(M); tree = small_gicp.KdTree(pc, num_threads=8)
small_gicp.estimate_normals(pc, tree, num_neighbors=15, num_threads=8)
N = np.asarray(pc.normals())[:, :3]; kd = cKDTree(M); print('지도점', len(M))

# 초기 외부파라미터: 광학(z앞 x오른 y아래) → 몸체(x앞 y왼 z위), 아래로 32.1°
R_ob = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)
R0 = Rot.from_euler('y', 32.1, degrees=True).as_matrix() @ R_ob
T0 = np.eye(4); T0[:3, :3] = R0; T0[:3, 3] = [0.30, 0.0, 0.11]


def cost_and_solve(T, delta, maxd, iters=1, solve=True):
    H = np.zeros((6, 6)); g = np.zeros(6); res_all = []
    for tt, P in frames:
        Rwb, pwb = pose_at(tt + delta)
        qb = P @ T[:3, :3].T + T[:3, 3]; qw = qb @ Rwb.T + pwb
        dd, j = kd.query(qw, distance_upper_bound=maxd); ok = np.isfinite(dd)
        if ok.sum() < 30: continue
        n = N[j[ok]]; rres = np.einsum('ij,ij->i', n, qw[ok] - M[j[ok]])
        res_all.append(np.abs(rres))
        if solve:
            a = n @ Rwb; Jw = np.cross(qb[ok], a); J = np.c_[Jw, a]
            w = 1.0 / np.maximum(1.0, np.abs(rres) / 0.02)       # Huber 2 cm
            H += (J * w[:, None]).T @ J; g += (J * w[:, None]).T @ rres
    r = np.concatenate(res_all) if res_all else np.array([np.inf])
    if not solve: return r, None
    dx = -np.linalg.solve(H + 1e-6 * np.eye(6), g)
    return r, dx


def update(T, dx):
    D = np.eye(4); D[:3, :3] = Rot.from_rotvec(dx[:3]).as_matrix(); D[:3, 3] = dx[3:]
    return D @ T                         # 몸체 좌표에서의 좌측 섭동 (qb' = D qb)


def frac_in(T, delta, tol=0.03):
    fr = []
    for tt, P in frames:
        Rwb, pwb = pose_at(tt + delta); qw = (P @ T[:3, :3].T + T[:3, 3]) @ Rwb.T + pwb
        dd, _ = kd.query(qw); fr.append((dd < tol).mean())
    return float(np.median(fr)), float(np.percentile(fr, 10))


T = T0.copy(); delta = 0.0
print('초기   3cm안 비율(중앙, 하위10%)', frac_in(T, delta))
for it, maxd in enumerate([0.5, 0.3, 0.2, 0.1, 0.1] + [0.06] * 40):
    r, dx = cost_and_solve(T, delta, maxd); T = update(T, dx)
    if it > 6 and np.linalg.norm(dx[3:]) < 2e-4 and np.linalg.norm(dx[:3]) < 2e-4: print('수렴', it); break
    if it % 5 == 0: print(f'반복{it} maxd {maxd} 잔차중앙 {np.median(r) * 1e3:.1f} mm 보정 {np.linalg.norm(dx[:3]) * 57.3:.2f}° {np.linalg.norm(dx[3:]) * 1e3:.1f} mm')
# 시각 지연 δ 격자
best = None
for dl in np.arange(-0.12, 0.121, 0.02):
    r, _ = cost_and_solve(T, dl, 0.05, solve=False); c = float(np.median(r))
    if best is None or c < best[1]: best = (dl, c)
    print(f'  δ {dl * 1e3:+4.0f} ms 잔차중앙 {c * 1e3:.2f} mm')
delta = best[0]
for it in range(30):
    r, dx = cost_and_solve(T, delta, 0.06); T = update(T, dx)
    if np.linalg.norm(dx) < 3e-4: break
fi = frac_in(T, delta)
eul = Rot.from_matrix(T[:3, :3] @ R_ob.T).as_euler('zyx', degrees=True)
out = dict(T_body_cam_optical=T.round(5).tolist(), 위치_m=T[:3, 3].round(4).tolist(), 기울기_deg_yaw_pitch_roll=eul.round(2).tolist(),
           시각지연_s=round(float(delta), 3), stamp_to_nuc=dict(offset=float(A[1]), slope=float(A[0]), t_ref=float(st[0, 1])),
           잔차중앙_mm=round(float(np.median(r)) * 1e3, 2), 지도3cm안_중앙=round(fi[0], 3), 지도3cm안_하위10=round(fi[1], 3), 프레임=len(frames))
print(json.dumps(out, ensure_ascii=False, indent=1)); json.dump(out, open(f'{OUT}/calib_depth_{SUBSET}.json', 'w'), ensure_ascii=False, indent=1)
