#!/usr/bin/env python3
"""XT16 대 L1 지도 — 같은 조건으로 다시 그린다 (v2).

v1 의 잘못: ① XT16 5 cm 칸 대 L1 2 cm 칸 (L1 이 더 촘촘해 보임)
           ② 칸마다 '가장 높은 점' → 잡음이 큰 L1(바닥 MAD 34.5 mm)은 높이가 부풀고 더 또렷해 보임
           ③ L1 시각·yaw 어긋남 (l1_align2.py 로 보정)
v2 규칙: 두 센서 모두 2 cm 복셀, 복셀마다 2번 이상 찍혀야 인정(KMIN), 칸 높이 = 인정된 복셀 중 가장 높은 것,
         높이는 같은 바닥 평면(XT16 기준, L1 은 바닥 오프셋 보정) 위로 잰다.
추가: 하단 5~30 cm 띠 지도, 높이별 점유 복셀 분포, 한 프레임에 발 앞 바닥을 얼마나 보는가.
"""
import numpy as np, rosbag2_py, bisect, json, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2

for f in ['NanumGothic', 'NanumBarunGothic', 'Noto Sans CJK KR']:
    if any(f in x.name for x in fm.fontManager.ttflist): plt.rcParams['font.family'] = f; break
plt.rcParams['axes.unicode_minus'] = False
BAG = '/home/ilovehan9283/bags/gosu_20261030_001126'
NPZ = '/home/ilovehan9283/ws/v_gosu/eval/out/gosu1030v2_map.npz'
HMAX, KMIN, LO, HI = 1.8, 2, 0.05, 0.30
AL = json.load(open('l1_align_1030.json'))
d = np.load(NPZ); V = float(d['V']); vox, cnt, odom, lid = d['vox'], d['cnt'], d['odom'], d['lid']
S = {'복셀': f'{V * 100:.0f} cm', '인정 기준': f'복셀당 {KMIN}회 이상', 'L1 보정': AL}


def reader(topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics)); return r


def q2R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def Rxyz(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def fit_floor(P, region):
    x0, x1, y0, y1 = region
    m = (P[:, 0] > x0) & (P[:, 0] < x1) & (P[:, 1] > y0) & (P[:, 1] < y1); Q = P[m]
    key = np.floor(Q[:, 0] / 0.1).astype(np.int64) * 100000 + np.floor(Q[:, 1] / 0.1).astype(np.int64)
    o = np.lexsort((Q[:, 2], key)); k, Qs = key[o], Q[o]; L = Qs[np.r_[True, k[1:] != k[:-1]]]
    h, e = np.histogram(L[:, 2], bins=np.arange(L[:, 2].min(), L[:, 2].max() + 0.02, 0.02))
    z0 = e[np.argmax(h)] + 0.01; sel = np.abs(L[:, 2] - z0) < 0.06
    for tol in (0.04, 0.025):
        coef = np.linalg.lstsq(np.c_[L[sel, 0], L[sel, 1], np.ones(sel.sum())], L[sel, 2], rcond=None)[0]
        sel = np.abs(L[:, 2] - (np.c_[L[:, 0], L[:, 1], np.ones(len(L))] @ coef)) < tol
    return coef, float(np.degrees(np.arctan(np.hypot(coef[0], coef[1]))))


def voxelize(P, w=None):
    i = np.floor(P / V).astype(np.int64)
    u, inv, c = np.unique(i, axis=0, return_inverse=True, return_counts=True)
    return u, c


# ── XT16: FAST-LIO 등록 스캔 2 cm 복셀
tr = odom[:, 2:5]
REG = (tr[:, 0].min() - 1, tr[:, 0].max() + 1, tr[:, 1].min() - 1, tr[:, 1].max() + 1)
Pc = (vox.astype(np.float64) + 0.5) * V
coef, tilt = fit_floor(Pc[cnt >= KMIN], REG)
def hgt(P): return P[:, 2] - (coef[0] * P[:, 0] + coef[1] * P[:, 1] + coef[2])
X = dict(P=Pc, c=cnt.astype(np.int64))

# ── 재생 시각 → 녹화 시각, 로봇 시계 오프셋 (render_1030.py 와 같음)
hdr2bag = {}
r = reader(['/lidar_points'])
while r.has_next():
    tp, raw, t = r.read_next()
    s, ns = np.frombuffer(raw[4:8], '<i4')[0], np.frombuffer(raw[8:12], '<u4')[0]
    hdr2bag[round(float(s) + float(ns) * 1e-9, 4)] = t * 1e-9
pairs = np.array([(w, hdr2bag[round(h, 4)]) for w, h in lid if round(h, 4) in hdr2bag])
Kp = float(np.median(pairs[:, 0] - pairs[:, 1]))
wl = np.sort(lid[:, 0]); h_o = odom[:, 1]
dmed = float(np.median(h_o - wl[np.clip(np.searchsorted(wl, h_o) - 1, 0, len(wl) - 1)]))
t_pose = h_o - dmed - Kp; o = np.argsort(t_pose); t_pose = t_pose[o]; Q = odom[o]
offs = []
r = reader(['/utlidar/robot_odom'])
while r.has_next() and len(offs) < 3000:
    tp, raw, t = r.read_next(); m = deserialize_message(raw, Odometry)
    offs.append(t * 1e-9 - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))
robot_off = float(np.median(offs))

# ── L1: 보정 4개 적용, 2 cm 복셀로 누적. 같은 김에 '발 앞 바닥' 한 프레임 커버리지(몸체 좌표)도 잰다
RL = Rxyz(*np.radians([AL['roll_deg'], AL['pitch_deg'], AL['yaw_deg']]))
FX = np.arange(0.2, 2.01, 0.05); FY = np.arange(-0.5, 0.501, 0.05)     # 발 앞 0.2~2.0 m, 좌우 ±0.5 m, 5 cm 칸
def front_cov(B, floor_z):
    m = np.abs(B[:, 2] - floor_z) < 0.06   # (안 씀)
    ix = np.floor((B[m, 0] - FX[0]) / 0.05).astype(int); iy = np.floor((B[m, 1] - FY[0]) / 0.05).astype(int)
    ok = (ix >= 0) & (ix < len(FX) - 1) & (iy >= 0) & (iy < len(FY) - 1)
    g = np.zeros((len(FX) - 1, len(FY) - 1), bool); g[ix[ok], iy[ok]] = True
    return g
keys, cl = [], []; covL = []; nL = 0; npts = 0; k = 0; buf = []
r = reader(['/utlidar/cloud_base'])
while r.has_next():
    tp, raw, t = r.read_next(); m = deserialize_message(raw, PointCloud2); k += 1
    a = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64); npts += len(a)
    a = a[~((np.abs(a[:, 0]) < 0.45) & (np.abs(a[:, 1]) < 0.30))]
    b = a @ RL.T
    ts = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 + robot_off + AL['dt']
    if k % 5 == 0: covL.append((ts, b))
    i = bisect.bisect_left(t_pose, ts)
    if i <= 0 or i >= len(t_pose) or t_pose[i] - t_pose[i - 1] > 0.2: continue
    j = i if (t_pose[i] - ts) < (ts - t_pose[i - 1]) else i - 1
    buf.append(np.floor((b @ q2R(*Q[j, 5:9]).T + Q[j, 2:5]) / V).astype(np.int64)); nL += 1
    if len(buf) >= 400:
        u, c = np.unique(np.concatenate(buf), axis=0, return_counts=True); keys.append(u); cl.append(c); buf = []
if buf:
    u, c = np.unique(np.concatenate(buf), axis=0, return_counts=True); keys.append(u); cl.append(c)
u, inv = np.unique(np.concatenate(keys), axis=0, return_inverse=True)
L = dict(P=(u.astype(np.float64) + 0.5) * V, c=np.bincount(inv.ravel(), weights=np.concatenate(cl)).astype(np.int64))
off_l = float(np.median(hgt(L['P'][(L['c'] >= KMIN)])[np.abs(hgt(L['P'][(L['c'] >= KMIN)])) < 0.08]))
dur = float(t_pose[-1] - t_pose[0])
S['L1'] = {'프레임': nL, '점/초': round(npts / dur), '바닥오프셋_m': round(off_l, 3)}

# XT16 한 프레임 발 앞 커버리지: /lidar_points(센서 좌표) → 몸체 (FAST-LIO 외부파라미터, 같은 설정 값)
RX = np.array([[-0.0001707, -0.9999240, 0.0123287], [0.9999844, -0.0001019, 0.0055807], [-0.0055790, 0.0123295, 0.9999084]])
TX = np.array([0.25, 0.0, 0.1588]); covX = []; nxp = 0; k = 0
r = reader(['/lidar_points'])
while r.has_next():
    tp, raw, t = r.read_next(); k += 1
    if k % 13: continue
    m = deserialize_message(raw, PointCloud2)
    a = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64); nxp += len(a)
    covX.append((t * 1e-9, a @ RX.T + TX))
S['XT16'] = {'점/초(원본)': round(nxp / len(covX) * 20)}
# 바닥 판정은 두 센서 같은 방식: 프레임 점을 FAST-LIO 자세로 월드에 놓고 공통 바닥 평면 ±6 cm 안이면 바닥 점
def front_cov_w(lst):
    gs = []
    for ts, B in lst:
        i = bisect.bisect_left(t_pose, ts)
        if i <= 0 or i >= len(t_pose) or t_pose[i] - t_pose[i - 1] > 0.2: continue
        j = i if (t_pose[i] - ts) < (ts - t_pose[i - 1]) else i - 1
        w = B @ q2R(*Q[j, 5:9]).T + Q[j, 2:5]
        fl = np.abs(hgt(w)) < 0.06
        Bf = B[fl]
        ix = np.floor((Bf[:, 0] - FX[0]) / 0.05).astype(int); iy = np.floor((Bf[:, 1] - FY[0]) / 0.05).astype(int)
        ok = (ix >= 0) & (ix < len(FX) - 1) & (iy >= 0) & (iy < len(FY) - 1)
        g = np.zeros((len(FX) - 1, len(FY) - 1), bool); g[ix[ok], iy[ok]] = True; gs.append(g)
    return np.mean(gs, axis=0), len(gs)
gX, nfx = front_cov_w(covX); gL, nfl = front_cov_w(covL)
S['발앞_표본프레임'] = dict(XT16=nfx, L1=nfl)

# ── 칸 높이: 인정된 복셀(KMIN) 중 가장 높은 것
def column_top(D, h, lo=-0.3, hi=HMAX):
    m = (D['c'] >= KMIN) & (h > lo) & (h < hi)
    return D['P'][m], h[m]
hX = hgt(X['P']); hL = hgt(L['P']) - off_l
PX, HX = column_top(X, hX); PL, HL = column_top(L, hL)
BX, BHX = column_top(X, hX, LO, HI); BL, BHL = column_top(L, hL, LO, HI)
px0, px1 = np.percentile(PX[:, 0], [0.5, 99.5]); py0, py1 = np.percentile(PX[:, 1], [0.5, 99.5])
ext = (np.floor(max(tr[:, 0].min() - 2.5, px0 - 0.3)), np.ceil(min(tr[:, 0].max() + 2.5, px1 + 0.3)),
       np.floor(max(tr[:, 1].min() - 2.5, py0 - 0.3)), np.ceil(min(tr[:, 1].max() + 4.5, py1 + 0.3)))
def grid_max(P, z, ext):
    x0, x1, y0, y1 = ext; nx = int(round((x1 - x0) / V)); ny = int(round((y1 - y0) / V))
    ix = ((P[:, 0] - x0) / V).astype(int); iy = ((P[:, 1] - y0) / V).astype(int)
    m = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    img = np.full((ny, nx), np.nan, np.float32); flat = img.reshape(-1); idx = iy[m] * nx + ix[m]
    o = np.argsort(z[m]); flat[idx[o]] = z[m][o]; return img
iX, iL = grid_max(PX, HX, ext), grid_max(PL, HL, ext)
bX, bL = grid_max(BX, BHX, ext), grid_max(BL, BHL, ext)
# 경기장 안쪽(궤적 상자 ±0.3 m) 하단 띠 칸 채움
ax0, ax1, ay0, ay1 = tr[:, 0].min() - 0.3, tr[:, 0].max() + 0.3, tr[:, 1].min() - 0.3, tr[:, 1].max() + 0.3
def inside(img):
    x0, x1, y0, y1 = ext; ny, nx = img.shape
    c0, c1 = int((ax0 - x0) / V), int((ax1 - x0) / V); r0, r1 = int((ay0 - y0) / V), int((ay1 - y0) / V)
    s = img[r0:r1, c0:c1]; return float(np.isfinite(s).mean())
S['하단띠_칸채움'] = dict(XT16=round(inside(bX) * 100, 1), L1=round(inside(bL) * 100, 1))
S['발앞_한프레임_바닥칸'] = dict(XT16=round(float(gX.mean()) * 100, 1), L1=round(float(gL.mean()) * 100, 1))
S['XT16_바닥기울기_deg'] = round(tilt, 2)
plen = float(np.sum(np.linalg.norm(np.diff(tr, axis=0), axis=1)))
sub = f'gosu_20261030_001126 (10-30 녹화, {dur:.0f}초, 경로 {plen:.0f} m) · 두 센서 같은 조건: {V * 100:.0f} cm 복셀, 복셀당 {KMIN}회 이상 찍힌 것만'


def draw(ax, img, title, traj=None, vmin=0, vmax=HMAX):
    im = ax.imshow(img, origin='lower', extent=ext, cmap='turbo', vmin=vmin, vmax=vmax, interpolation='nearest')
    if traj is not None: ax.plot(traj[:, 0], traj[:, 1], '-', color='k', lw=0.6, alpha=0.5)
    ax.plot(0, 0, marker='o', ms=7, mfc='white', mec='k')
    ax.set_title(title, fontsize=11); ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_aspect('equal'); ax.grid(alpha=0.25, lw=0.5)
    return im


tX = f'XT16 (Hesai 16채널, 원본 {S["XT16"]["점/초(원본)"] / 1000:.0f}만 점/초) · FAST-LIO 매핑'
tL = f'L1 (Unitree 내장, {S["L1"]["점/초"] / 1000:.1f}만 점/초) · FAST-LIO 자세로 누적 (시각·자세 보정)'
# 그림 1: 전체 지도 나란히
fig, axs = plt.subplots(1, 2, figsize=(17, 9), dpi=130)
im = draw(axs[0], iX, tX + f'\n바닥~{HMAX} m · 칸 높이 = 인정 복셀 최고'); draw(axs[1], iL, tL + f'\n바닥~{HMAX} m · 칸 높이 = 인정 복셀 최고', tr[::20])
fig.colorbar(im, ax=axs, fraction=0.025, pad=0.01).set_label('바닥 위 높이 (m)'); fig.suptitle(sub, fontsize=11)
fig.savefig('v2_xt16_vs_l1_full.png'); plt.close(fig)
for img, t, fn, trj in [(iX, tX, 'v2_xt16_map.png', None), (iL, tL, 'v2_l1_map.png', tr[::20])]:
    fig, ax = plt.subplots(figsize=(8.8, 9), dpi=150); im = draw(ax, img, t + f'\n{sub.split(" · ")[0]}', trj)
    fig.colorbar(im, ax=ax, fraction=0.04, pad=0.02).set_label('바닥 위 높이 (m)'); fig.tight_layout(); fig.savefig(fn); plt.close(fig)
# 그림 2: 하단 띠 + 높이 분포 + 발 앞 한 프레임
fig = plt.figure(figsize=(16, 15), dpi=120); gs = fig.add_gridspec(2, 2, height_ratios=[1.25, 0.8])
a0 = fig.add_subplot(gs[0, 0]); a1 = fig.add_subplot(gs[0, 1])
im = draw(a0, bX, f'XT16 — 바닥 위 {LO * 100:.0f}~{HI * 100:.0f} cm 에 인정 복셀이 있는 칸\n경기장 안 칸 채움 {S["하단띠_칸채움"]["XT16"]}%', vmin=LO, vmax=HI)
draw(a1, bL, f'L1 — 바닥 위 {LO * 100:.0f}~{HI * 100:.0f} cm 에 인정 복셀이 있는 칸\n경기장 안 칸 채움 {S["하단띠_칸채움"]["L1"]}%', vmin=LO, vmax=HI)
for a in (a0, a1): a.add_patch(plt.Rectangle((ax0, ay0), ax1 - ax0, ay1 - ay0, fill=False, ec='k', ls='--', lw=0.8))
fig.colorbar(im, ax=[a0, a1], fraction=0.02, pad=0.01).set_label('바닥 위 높이 (m)')
a2 = fig.add_subplot(gs[1, 0]); bins = np.arange(-0.1, HMAX + 0.001, 0.02)
def inarena(P): return (P[:, 0] > ax0) & (P[:, 0] < ax1) & (P[:, 1] > ay0) & (P[:, 1] < ay1)
hx_, _ = np.histogram(HX[inarena(PX)], bins); hl_, _ = np.histogram(HL[inarena(PL)], bins)
a2.plot(bins[:-1] + 0.01, hx_, color='#1f77b4', label='XT16'); a2.plot(bins[:-1] + 0.01, hl_, color='#d62728', label='L1')
a2.axvspan(LO, HI, color='gold', alpha=0.25, label='하단 띠'); a2.set_yscale('log'); a2.grid(alpha=0.3, which='both'); a2.legend(fontsize=9)
a2.set_xlabel('바닥 위 높이 (m)'); a2.set_ylabel('인정 복셀 수 (경기장 안)'); a2.set_title('높이별 점유 복셀 수 (2 cm 복셀)', fontsize=11)
a3 = fig.add_subplot(gs[1, 1]); dx = FX[:-1] + 0.025
a3.plot(dx, gX.mean(axis=1) * 100, color='#1f77b4', label=f'XT16 (평균 {S["발앞_한프레임_바닥칸"]["XT16"]}%)')
a3.plot(dx, gL.mean(axis=1) * 100, color='#d62728', label=f'L1 (평균 {S["발앞_한프레임_바닥칸"]["L1"]}%)')
a3.set_xlabel('로봇 앞 거리 (m)'); a3.set_ylabel('바닥 5 cm 칸 중 점이 있는 비율 (%)'); a3.set_ylim(0, 100); a3.grid(alpha=0.3); a3.legend(fontsize=9)
a3.set_title('한 프레임에 발 앞 바닥(좌우 ±0.5 m)을 얼마나 보나', fontsize=11)
fig.suptitle('하단을 누가 더 잘 보나 — ' + sub, fontsize=11); fig.savefig('v2_lowband.png'); plt.close(fig)
json.dump(S, open('render_v2_stats.json', 'w'), ensure_ascii=False, indent=1); print(json.dumps(S, ensure_ascii=False, indent=1))
