#!/usr/bin/env python3
"""replay_check_v2.sh 녹화(out/<태그>.rec)에서 좌표 정렬(frame_align)만 본다 — 오프라인, bag 파일 읽기만.
  · /backup/status 의 align 상태 변화(identity → estimating → frozen), 고정 시각, T, 잔차, health 'aligning' 구간
  · 지도 파일(.npz)에 A_yaw_deg/A_t 가 있으면(tests/map_first270s_rot3deg.npz) 정답 T = A⁻¹ 과 비교
  · /backup/odometry(camera_init) − /Odometry(FAST-LIO) 받은 시각 최근접 대응 오차 — 고정 전/후
  · /mux/status 전환 시각·사유, 전환 뒤 백업 궤적 ↔ 10-30 원 FAST-LIO 경로 거리
사용: python3 analyze_align_replay.py <태그> [지도.npz]
"""
import sys, json, math, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from scipy.spatial import cKDTree
TAG = sys.argv[1]; MAP = sys.argv[2] if len(sys.argv) > 2 else None
D = f'/home/ilovehan9283/ws/night_20261104/backup_v2/out/{TAG}.rec'
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=D, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
T = {'/Odometry': Odometry, '/backup/odometry': Odometry, '/backup/status': String, '/mux/status': String}
ev = {k: [] for k in T}
while r.has_next():
    tp, d, t = r.read_next()
    if tp in T: ev[tp].append((t * 1e-9, deserialize_message(d, T[tp])))
t0 = min(v[0][0] for v in ev.values() if v)
st = [(t - t0, json.loads(m.data)) for t, m in ev['/backup/status']]
prev = None; t_frz = None
for t, s in st:
    a = s.get('align', {}); k = (a.get('state'), s.get('health'))
    if k != prev:
        print(f"{t:7.1f}s  align={a.get('state')}  health={s.get('health')} (raw {s.get('health_raw')})  T={a.get('T')}  쌍={a.get('pairs')}  {a.get('why', '')}")
        prev = k
    if t_frz is None and a.get('state') in ('frozen', 'identity_frozen'): t_frz = t
if st:
    a = st[-1][1].get('align', {}); print('마지막 align', json.dumps(a, ensure_ascii=False))
    if st[-1][1].get('exceptions'): print('★ 콜백 예외', st[-1][1]['exceptions'])
    if MAP and MAP.endswith('.npz'):
        z = np.load(MAP)
        if 'A_yaw_deg' in z:
            ya = math.radians(float(z['A_yaw_deg'])); ax, ay = map(float, z['A_t']); c, s_ = math.cos(ya), math.sin(ya)
            Tt = (-(c * ax + s_ * ay), -(-s_ * ax + c * ay), -math.degrees(ya)); Te = a.get('T', {})
            if Te: print(f"정답 T {np.round(Tt, 3)} | 추정 ({Te['x']}, {Te['y']}, {Te['yaw_deg']}°) → xy 오차 {math.hypot(Te['x'] - Tt[0], Te['y'] - Tt[1]):.3f} m, yaw 오차 {Te['yaw_deg'] - Tt[2]:+.2f}°")
# 백업(camera_init) − FAST-LIO, 받은 시각 최근접
F = np.array([(t - t0, m.pose.pose.position.x, m.pose.pose.position.y) for t, m in ev['/Odometry']])
B = np.array([(t - t0, m.pose.pose.position.x, m.pose.pose.position.y) for t, m in ev['/backup/odometry']])
if len(F) and len(B):
    j = np.clip(np.searchsorted(B[:, 0], F[:, 0]), 1, len(B) - 1); j -= (F[:, 0] - B[j - 1, 0]) < (B[j, 0] - F[:, 0])
    ok = np.abs(B[j, 0] - F[:, 0]) < 0.05; e = np.hypot(F[:, 1] - B[j, 1], F[:, 2] - B[j, 2])
    for nm, sel in (('고정 전', ok & (F[:, 0] < (t_frz if t_frz else 1e9))), ('고정 후', ok & (F[:, 0] >= (t_frz if t_frz else 1e9)))):
        if sel.any():
            print(f"{nm} |백업 − FAST-LIO| {sel.sum()}점  중앙 {np.median(e[sel]):.3f} / p95 {np.percentile(e[sel], 95):.3f} / 최대 {e[sel].max():.3f} m  (0.5 m 초과 {np.mean(e[sel] > 0.5):.1%})")
sw = [(t - t0, json.loads(m.data)) for t, m in ev['/mux/status'] if json.loads(m.data)['mode'] == 'backup']
print('전환:', f"{sw[0][0]:.1f}s — {sw[0][1]['reason']}" if sw else '없음')
if sw and len(B):
    ref = np.load('/home/ilovehan9283/ws/v_gosu/eval/out/gosu1030v2_map.npz')['odom'][:, 2:4]
    Bs = B[B[:, 0] >= sw[0][0]]
    if len(Bs):
        dd, _ = cKDTree(ref).query(Bs[:, 1:3])
        print(f"전환 뒤 백업 궤적 {len(Bs)}점 → 10-30 원 FAST-LIO 경로까지 중앙 {np.median(dd):.3f} / p95 {np.percentile(dd, 95):.3f} / 최대 {dd.max():.3f} m")
