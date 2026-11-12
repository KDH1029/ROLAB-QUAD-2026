#!/usr/bin/env python3
"""gosu_20261030_001126 지도 그림 — XT16 FAST-LIO 지도, L1 을 FAST-LIO 자세로 누적, 둘 나란히.

입력  ~/ws/v_gosu/eval/out/gosu1030_map.npz  (map_replay.sh 결과: 5 cm 복셀, FAST-LIO 자세, 재생 시각표)
      bag 의 /lidar_points (재생 시각 → 녹화 시각 대응), /utlidar/cloud_base, /utlidar/robot_odom (로봇 시계 오프셋)
높이  각 지도에서 바닥 평면을 따로 맞춰 '바닥 위 높이'로 그린다(FAST-LIO 월드는 처음 자세 기준이라 기울어 있다).
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
NPZ = '/home/ilovehan9283/ws/v_gosu/eval/out/gosu1030_map.npz'
HMAX = 1.8
d = np.load(NPZ)
V = float(d['V']); vox, cnt, odom, lid = d['vox'], d['cnt'], d['odom'], d['lid']
stats = {}


def reader(topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics)); return r


def q2R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def fit_floor(P, region):
    """칸별 최저 z 의 최빈 띠로 바닥 평면 z = a x + b y + c 를 맞춘다 (두 번 다듬음)."""
    x0, x1, y0, y1 = region
    m = (P[:, 0] > x0) & (P[:, 0] < x1) & (P[:, 1] > y0) & (P[:, 1] < y1); Q = P[m]
    key = np.floor(Q[:, 0] / 0.1).astype(np.int64) * 100000 + np.floor(Q[:, 1] / 0.1).astype(np.int64)
    o = np.lexsort((Q[:, 2], key)); k, Qs = key[o], Q[o]
    first = np.r_[True, k[1:] != k[:-1]]; L = Qs[first]                 # 칸마다 가장 낮은 점
    h, e = np.histogram(L[:, 2], bins=np.arange(L[:, 2].min(), L[:, 2].max() + 0.02, 0.02))
    z0 = e[np.argmax(h)] + 0.01; sel = np.abs(L[:, 2] - z0) < 0.06
    for tol in (0.04, 0.025):
        A = np.c_[L[sel, 0], L[sel, 1], np.ones(sel.sum())]
        coef = np.linalg.lstsq(A, L[sel, 2], rcond=None)[0]
        sel = np.abs(L[:, 2] - (np.c_[L[:, 0], L[:, 1], np.ones(len(L))] @ coef)) < tol
    tilt = np.degrees(np.arctan(np.hypot(coef[0], coef[1])))
    return coef, tilt, int(sel.sum())


def height(P, coef):
    return P[:, 2] - (coef[0] * P[:, 0] + coef[1] * P[:, 1] + coef[2])


def grid_max(xy, z, cell, ext):
    x0, x1, y0, y1 = ext
    nx = int((x1 - x0) / cell) + 1; ny = int((y1 - y0) / cell) + 1
    ix = ((xy[:, 0] - x0) / cell).astype(int); iy = ((xy[:, 1] - y0) / cell).astype(int)
    m = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    img = np.full((ny, nx), np.nan, np.float32); flat = img.reshape(-1); idx = iy[m] * nx + ix[m]
    o = np.argsort(z[m]); flat[idx[o]] = z[m][o]
    return img


def low_cov(xy, h, ext, cell=0.05):
    """경기장 안 바닥 위 5~30 cm 띠에 점이 있는 칸 비율 (하단 장애물이 보이는 정도)."""
    x0, x1, y0, y1 = ext
    m = (h > 0.05) & (h < 0.30) & (xy[:, 0] > x0) & (xy[:, 0] < x1) & (xy[:, 1] > y0) & (xy[:, 1] < y1)
    k = np.unique(np.floor(xy[m, 0] / cell).astype(np.int64) * 100000 + np.floor(xy[m, 1] / cell).astype(np.int64))
    return len(k), int(round((x1 - x0) / cell)) * int(round((y1 - y0) / cell))


# ── 1. XT16: FAST-LIO 등록 스캔 복셀 (2회 이상 찍힌 칸만 — 지나가는 사람 등 제거)
P = (vox[cnt >= 2].astype(np.float64) + 0.5) * V
tr = odom[:, 2:5]
pad = 1.0
REG = (tr[:, 0].min() - pad, tr[:, 0].max() + pad, tr[:, 1].min() - pad, tr[:, 1].max() + pad)
coef_x, tilt_x, nfl_x = fit_floor(P, REG)
hx = height(P, coef_x)
keep = (hx > -0.3) & (hx < HMAX); Px, hx = P[keep], hx[keep]
stats['xt16'] = dict(복셀=int(len(vox)), 두번이상=int((cnt >= 2).sum()), 바닥기울기_deg=round(tilt_x, 2), 바닥칸=nfl_x)

# ── 2. 재생 시각 → 녹화 시각: /lidar_points 원본 header 로 bag 수신 시각을 찾아 붙인다
hdr2bag = {}
r = reader(['/lidar_points'])
while r.has_next():
    tp, raw, t = r.read_next()
    s, ns = np.frombuffer(raw[4:12], '<i4')[0], np.frombuffer(raw[8:12], '<u4')[0]
    hdr2bag[round(float(s) + float(ns) * 1e-9, 4)] = t * 1e-9
pairs = np.array([(w, hdr2bag[round(h, 4)]) for w, h in lid if round(h, 4) in hdr2bag])
Kp = float(np.median(pairs[:, 0] - pairs[:, 1]))                       # 재생 벽시각 - bag 수신시각
wl = np.sort(lid[:, 0]); h_o = odom[:, 1]
dd = h_o - wl[np.clip(np.searchsorted(wl, h_o) - 1, 0, len(wl) - 1)]     # odom stamp - 직전 라이다 수신
dmed = float(np.median(dd))
t_pose = h_o - dmed - Kp                                               # 자세 시각(bag 수신 시각 기준 = 스캔 끝 도착)
stats['time'] = dict(라이다대응=int(len(pairs)), 재생오프셋_s=round(Kp, 3), odom지연_ms=round(dmed * 1000, 1),
                     odom지연_p95_ms=round(float(np.percentile(dd, 95)) * 1000, 1))
order = np.argsort(t_pose); t_pose = t_pose[order]; Q = odom[order]

# 로봇 시계 → NUC: /utlidar/robot_odom 은 즉시 발행이라 (수신 - header) 가 시계 차다
offs = []
r = reader(['/utlidar/robot_odom'])
while r.has_next() and len(offs) < 3000:
    tp, raw, t = r.read_next(); m = deserialize_message(raw, Odometry)
    offs.append(t * 1e-9 - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))
robot_off = float(np.median(offs)); stats['time']['로봇시계오프셋_s'] = round(robot_off, 4)

# ── 3. L1: /utlidar/cloud_base(base_link) 를 FAST-LIO 자세로 월드에 놓는다
#    l1_align.py 가 XT16 벽에 맞춰 찾은 보정(시각 dt, yaw)을 쓴다. 없으면 0
try:
    AL = json.load(open('l1_align_1030.json'))
except FileNotFoundError:
    AL = dict(dt=0.0, yaw_deg=0.0)
cz, sz = np.cos(np.radians(AL['yaw_deg'])), np.sin(np.radians(AL['yaw_deg']))
RZ = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
L1 = []; nused = 0
r = reader(['/utlidar/cloud_base'])
while r.has_next():
    tp, raw, t = r.read_next(); m = deserialize_message(raw, PointCloud2)
    ts = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 + robot_off + AL['dt']
    i = bisect.bisect_left(t_pose, ts)
    if i <= 0 or i >= len(t_pose) or t_pose[i] - t_pose[i - 1] > 0.2: continue
    j = i if (t_pose[i] - ts) < (ts - t_pose[i - 1]) else i - 1
    a = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)
    a = a[~((np.abs(a[:, 0]) < 0.45) & (np.abs(a[:, 1]) < 0.30))]     # 몸통·다리 자기 점 제거
    R = q2R(*Q[j, 5:9]); w = (a @ RZ.T) @ R.T + Q[j, 2:5]
    L1.append(w.astype(np.float32)); nused += 1
P1 = np.concatenate(L1).astype(np.float64)
coef_l, tilt_l, nfl_l = fit_floor(P1, REG)
hl = height(P1, coef_l)
keep = (hl > -0.3) & (hl < HMAX); P1, hl = P1[keep], hl[keep]
stats['l1'] = dict(보정_dt_s=AL['dt'], 보정_yaw_deg=AL['yaw_deg'], 프레임=nused, 점=int(len(P1)), 바닥기울기_deg=round(tilt_l, 2), 바닥칸=nfl_l,
                   바닥높이차_m=round(float(coef_l[2] - coef_x[2]), 3))

# ── 4. 그림
# 틀: 궤적 ±2.5 m 와 XT16 점 분포(0.5~99.5%) 중 좁은 쪽 — 경기장 벽까지 다 보이되 문 너머 방은 자른다
px0, px1 = np.percentile(Px[:, 0], [0.5, 99.5]); py0, py1 = np.percentile(Px[:, 1], [0.5, 99.5])
ext = (np.floor(max(tr[:, 0].min() - 2.5, px0 - 0.3)), np.ceil(min(tr[:, 0].max() + 2.5, px1 + 0.3)),
       np.floor(max(tr[:, 1].min() - 2.5, py0 - 0.3)), np.ceil(min(tr[:, 1].max() + 4.5, py1 + 0.3)))
ix_img = grid_max(Px[:, :2], hx, 0.05, ext); il_img = grid_max(P1[:, :2], hl, 0.02, ext)
arena = (tr[:, 0].min() - 0.3, tr[:, 0].max() + 0.3, tr[:, 1].min() - 0.3, tr[:, 1].max() + 0.3)
nx, tot = low_cov(Px[:, :2], hx, arena); nl, _ = low_cov(P1[:, :2], hl, arena)
stats['하단띠_5_30cm_칸채움'] = dict(XT16=f'{nx}/{tot} = {nx / tot * 100:.1f}%', L1=f'{nl}/{tot} = {nl / tot * 100:.1f}%')
dur = float(t_pose[-1] - t_pose[0]); plen = float(np.sum(np.linalg.norm(np.diff(tr, axis=0), axis=1)))


def draw(ax, img, title, traj=None):
    im = ax.imshow(img, origin='lower', extent=(ext[0], ext[1], ext[2], ext[3]), cmap='turbo', vmin=0, vmax=HMAX,
                   interpolation='nearest')
    if traj is not None:
        ax.plot(traj[:, 0], traj[:, 1], '-', color='k', lw=0.7, alpha=0.55)
    ax.plot(0, 0, marker='o', ms=7, mfc='white', mec='k')
    ax.set_title(title, fontsize=11); ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    ax.set_aspect('equal'); ax.grid(alpha=0.25, lw=0.5)
    return im


sub = f'gosu_20261030_001126 bag (10-30 녹화, {dur:.0f}초, 경로 {plen:.0f} m)'
t1 = f'XT16 (Hesai 16채널) · FAST-LIO 매핑 (10-30 재생)\n{sub} · 바닥~{HMAX} m · 5 cm 칸 · 바닥 평면 보정 {tilt_x:.2f}°'
t2 = f'L1 (Unitree 내장) · FAST-LIO 자세로 누적 (10-30, 시각 {AL["dt"]:+.2f} s·yaw {AL["yaw_deg"]:+.1f}° 보정)\n{sub} · 바닥~{HMAX} m · 2 cm 칸 · 바닥 평면 보정 {tilt_l:.2f}°'
for img, title, fn, traj in [(ix_img, t1, 'xt16_map_1030.png', None), (il_img, t2, 'l1_map_1030.png', tr[::20])]:
    fig, ax = plt.subplots(figsize=(8.6, 8.8), dpi=150)
    im = draw(ax, img, title, traj)
    if traj is not None: ax.legend([ax.lines[0]], ['FAST-LIO 궤적'], loc='lower right', fontsize=9)
    cb = fig.colorbar(im, ax=ax, fraction=0.04, pad=0.02); cb.set_label('바닥 위 높이 (m)')
    fig.tight_layout(); fig.savefig(fn); plt.close(fig)
fig, axs = plt.subplots(1, 2, figsize=(17, 8.8), dpi=130)
im = draw(axs[0], ix_img, t1.replace(f'\n{sub} · ', '\n')); draw(axs[1], il_img, t2.replace(f'\n{sub} · ', '\n'), tr[::20])
axs[0].text(0.01, 0.01, f'하단 5~30 cm 띠 칸 채움 {nx / tot * 100:.1f}%', transform=axs[0].transAxes, fontsize=10,
            bbox=dict(fc='white', alpha=0.85, lw=0))
axs[1].text(0.01, 0.01, f'하단 5~30 cm 띠 칸 채움 {nl / tot * 100:.1f}%', transform=axs[1].transAxes, fontsize=10,
            bbox=dict(fc='white', alpha=0.85, lw=0))
fig.colorbar(im, ax=axs, fraction=0.025, pad=0.01).set_label('바닥 위 높이 (m)')
fig.suptitle(sub, fontsize=12); fig.savefig('xt16_vs_l1_1030.png'); plt.close(fig)
stats['경로_m'] = round(plen, 1); stats['궤적범위'] = dict(x=[round(tr[:, 0].min(), 2), round(tr[:, 0].max(), 2)],
                                                     y=[round(tr[:, 1].min(), 2), round(tr[:, 1].max(), 2)])
json.dump(stats, open('render_1030_stats.json', 'w'), ensure_ascii=False, indent=1)
print(json.dumps(stats, ensure_ascii=False, indent=1))
