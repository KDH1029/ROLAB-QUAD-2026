#!/usr/bin/env python3
"""두 녹화의 XT16 원본 스캔·IMU 를 나란히 잰다 — FAST-LIO 가 한쪽에서만 출렁이는 이유 가르기용. 녹화 파일만 읽는다(노드·DDS 없음).
잰 것(스캔은 STEP 개마다 하나):
  점 수 · 점별 시각 필드 폭(디스큐 가능 여부) · 거리 분포(blind 0.5, det_range 15 기준) · 바닥 점 비율 · 라이다보다 높은 점 비율 · 반사 강도
  퇴화: 0.1 m 로 묶은 점의 국소 평면 법선으로 점-평면 정보 행렬(수평 이동 2×2, yaw) → 가장 약한 방향/가장 강한 방향 비. 복도면 0 에 가까움
  IMU(/lowstate): 각속도·가속도 크기 분포(걸음 충격)
사용: raw_scan_compare.py <이름=bag 폴더> [...]  (STEP=5)"""
import sys, os, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2 as pc2
from scipy.spatial import cKDTree

STEP = int(os.environ.get('STEP', '5'))
Pc = get_message('sensor_msgs/msg/PointCloud2')
try:
    LS = get_message('unitree_go/msg/LowState')
except Exception:
    LS = None


def reader(uri, topics):
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=uri, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics)); return r


def degeneracy(p):
    """p: 라이다 좌표 점(N,3). 0.1 m 칸 대표점 → 국소 평면 법선 → 수평 이동·yaw 정보량"""
    k = np.unique(np.floor(p / 0.1).astype(np.int64), axis=0, return_index=True)[1]; q = p[k]
    if len(q) < 50: return np.nan, np.nan, 0
    T = cKDTree(q); H = np.zeros((2, 2)); yaw = 0.0; n_ok = 0
    for i, nb in enumerate(T.query_ball_point(q, 0.3)):
        if len(nb) < 6: continue
        Q = q[nb] - q[nb].mean(0); ev, evec = np.linalg.eigh(Q.T @ Q / len(nb))
        if ev[0] > 0.05 * ev[1]: continue                    # 평면 아님
        n = evec[:, 0]; nxy = n[:2]
        H += np.outer(nxy, nxy); yaw += (q[i, 0] * n[1] - q[i, 1] * n[0]) ** 2; n_ok += 1
    if n_ok < 30: return np.nan, np.nan, n_ok
    e = np.linalg.eigvalsh(H)
    return e[0] / max(e[1], 1e-9), yaw / n_ok, n_ok


R_EXT = np.array([[-0.0001707, -0.9999240, 0.0123287], [0.9999844, -0.0001019, 0.0055807], [-0.0055790, 0.0123295, 0.9999084]])   # map_replay.yaml extrinsic_R


def floor_normal_body(p, rr):
    """라이다 아래 0.25~0.75 m, 0.8~4.5 m 거리 점으로 바닥 평면(세 번 다듬음) → 몸통 좌표 법선(위쪽). 실패 nan"""
    q = p[(p[:, 2] < -0.25) & (p[:, 2] > -0.75) & (rr > 0.8) & (rr < 4.5)]
    if len(q) < 80: return (np.nan,) * 4
    sel = np.ones(len(q), bool)
    for tol in (0.08, 0.04, 0.02):
        A = np.c_[q[sel, 0], q[sel, 1], np.ones(sel.sum())]; c = np.linalg.lstsq(A, q[sel, 2], rcond=None)[0]
        sel = np.abs(q[:, 2] - (q[:, 0] * c[0] + q[:, 1] * c[1] + c[2])) < tol
        if sel.sum() < 50: return (np.nan,) * 4
    n = np.array([-c[0], -c[1], 1.0]); n /= np.linalg.norm(n); nb = R_EXT @ n
    ext = np.ptp(q[sel, :2] @ np.linalg.eigh(np.cov(q[sel, :2].T))[1], 0).min()   # 바닥 점의 좁은 쪽 폭(작으면 기울기 추정 약함)
    return (*nb, ext)


for arg in sys.argv[1:]:
    name, uri = arg.split('=', 1); uri = os.path.expanduser(uri)
    r = reader(uri, ['/lidar_points']); i = 0; S = []; FN = []; fields = None
    while r.has_next():
        _, raw, tt = r.read_next(); i += 1
        if i % STEP: continue
        m = deserialize_message(raw, Pc)
        if fields is None:
            fields = [f.name for f in m.fields]; tf = next((f for f in ('timestamp', 'time', 't', 'offset_time') if f in fields), None)
        st = pc2.read_points(m, field_names=['x', 'y', 'z', 'intensity'] + ([tf] if tf else []), skip_nans=True)
        a = np.stack([st[f].astype(np.float64) for f in st.dtype.names], 1)
        a = a[np.isfinite(a[:, :3]).all(1)]
        rr = np.hypot(a[:, 0], a[:, 1]); a = a[rr > 0.5]; rr = rr[rr > 0.5]
        if len(a) < 100: continue
        span = (a[:, 4].max() - a[:, 4].min()) * 1000 if tf else np.nan
        z = a[:, 2]
        floor = ((z > -0.62) & (z < -0.34) & (rr < 6)).mean()
        hi = (z > 0.25).mean()
        h = np.histogram(rr, bins=[0.5, 1, 2, 4, 8, 15, 1e9])[0] / len(rr)
        dg, yw, nok = degeneracy(a[(rr < 15)][:, :3])
        nb = floor_normal_body(a[:, :3], rr)
        S.append([len(a), span, floor, hi, np.median(a[:, 3]), dg, yw, nok, *h]); FN.append((tt * 1e-9, *nb))
    S = np.array(S)
    print(f'\n=== {name}  ({uri})  스캔 {len(S)} 개 표본(STEP {STEP}) · 필드 {fields} · 시각필드 {tf}')
    print(f' 점/스캔 중앙 {np.median(S[:, 0]):.0f} · 스캔 안 시각 폭 중앙 {np.nanmedian(S[:, 1]):.1f} ms (5%~95% {np.nanpercentile(S[:, 1], 5):.1f}~{np.nanpercentile(S[:, 1], 95):.1f})')
    print(f' 거리 분포 0.5~1 / 1~2 / 2~4 / 4~8 / 8~15 / 15+ m : ' + ' / '.join(f'{x * 100:.0f}%' for x in np.median(S[:, 8:], 0)))
    print(f' 바닥 점(라이다 아래 0.34~0.62 m, 6 m 안) {np.median(S[:, 2]) * 100:.1f}% · 라이다보다 25 cm 이상 높은 점 {np.median(S[:, 3]) * 100:.1f}% · 반사강도 중앙 {np.median(S[:, 4]):.0f}')
    print(f' 수평 이동 구속비(약한/강한, 복도≈0) 중앙 {np.nanmedian(S[:, 5]):.3f} · 하위 10% {np.nanpercentile(S[:, 5], 10):.3f} · 0.05 미만 스캔 {np.nanmean(S[:, 5] < 0.05) * 100:.0f}% · 평면 칸 {np.median(S[:, 7]):.0f}')
    print(f' yaw 구속(평균 (p×n)_z², m²) 중앙 {np.nanmedian(S[:, 6]):.2f}')
    if LS is not None:
        r = reader(uri, ['/lowstate']); g = []; ac = []; t = []; j = 0
        while r.has_next():
            _, raw, tt = r.read_next(); j += 1
            if j % 5: continue
            m = deserialize_message(raw, LS); g.append(m.imu_state.gyroscope); ac.append(m.imu_state.accelerometer); t.append(tt * 1e-9)
        g, ac, t = np.array(g), np.array(ac), np.array(t)
        FN = np.array(FN); ok = np.isfinite(FN[:, 1]) & (FN[:, 4] > 1.0)
        res = []
        for row in FN[ok]:
            w = (t > row[0] - 0.15) & (t < row[0] + 0.05)
            if w.sum() < 5: continue
            gmax = np.linalg.norm(g[w], axis=1).max(); a_ = ac[w].mean(0); a_ /= np.linalg.norm(a_)
            ang = np.degrees(np.arccos(np.clip(a_ @ row[1:4], -1, 1)))
            # 기울기 차이를 몸통 앞뒤(pitch)·좌우(roll) 성분으로
            d = row[1:4] - a_
            res.append((gmax, ang, np.degrees(d[0]), np.degrees(d[1])))
        res = np.array(res); still = res[:, 0] < 0.08
        print(f' [틀어짐] 라이다 바닥 법선 vs IMU 중력(몸통 좌표) — 바닥 폭 1 m 넘는 스캔 {len(res)}')
        for lab, m_ in (('정지', still), ('걷는 중', ~still)):
            if m_.sum() >= 5:
                print(f'   {lab}: {m_.sum()} 스캔 · 어긋남 중앙 {np.median(res[m_, 1]):.2f}° (25~75% {np.percentile(res[m_, 1], 25):.2f}~{np.percentile(res[m_, 1], 75):.2f}) · 앞뒤 성분 {np.median(res[m_, 2]):+.2f}° · 좌우 성분 {np.median(res[m_, 3]):+.2f}°')
        gm = np.linalg.norm(g, axis=1); am = np.linalg.norm(ac, axis=1)
        mv = gm > 0.2   # 걷는 중(대략)
        print(f' IMU /lowstate {len(t) * 5 / (t[-1] - t[0]):.0f} Hz · 걷는 중 비율 {mv.mean() * 100:.0f}% · 각속도 크기 중앙/99% {np.median(gm[mv]):.2f}/{np.percentile(gm[mv], 99):.2f} rad/s · 가속도 |a|-g 99% {np.percentile(np.abs(am[mv] - np.median(am)), 99):.2f} m/s²')
