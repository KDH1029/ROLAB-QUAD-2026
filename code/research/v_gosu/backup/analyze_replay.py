#!/usr/bin/env python3
"""backup_replay_check.sh 결과 분석 — 전환 시점, 전환 전후 위치 출력·경로·속도명령이 끊기지 않았는가.
사용: analyze_replay.py <태그>
"""
import sys, json, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
from std_msgs.msg import String
TAG = sys.argv[1]; D = f'/home/ilovehan9283/ws/v_gosu/backup/out/{TAG}.rec'
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=D, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
T = {'/Odometry': Odometry, '/Odometry_mux': Odometry, '/backup/odometry': Odometry, '/ego_planner/trajectory': Path,
     '/cmd_vel': Twist, '/mux/status': String, '/backup/status': String}
ev = {k: [] for k in T}
while r.has_next():
    tp, d, t = r.read_next()
    if tp in T: ev[tp].append((t * 1e-9, deserialize_message(d, T[tp])))
t0 = min(v[0][0] for v in ev.values() if v)
sw = [(t - t0, json.loads(m.data)) for t, m in ev['/mux/status'] if json.loads(m.data)['mode'] == 'backup']
fl_last = ev['/Odometry'][-1][0] - t0 if ev['/Odometry'] else None
print(f"FAST-LIO 마지막 출력 {fl_last:.1f}s" if fl_last else 'FAST-LIO 출력 없음')
print('전환:', f"{sw[0][0]:.1f}s — {sw[0][1]['reason']}" if sw else '없음')
cut = sw[0][0] + t0 if sw else (fl_last + t0 if fl_last else None)
def rate(k, a, b):
    ts = [t for t, _ in ev[k] if a <= t < b]; return len(ts) / max(b - a, 1e-6), (max(np.diff(ts)) if len(ts) > 1 else None)
if cut:
    for k in ('/Odometry_mux', '/ego_planner/trajectory', '/cmd_vel'):
        r1, g1 = rate(k, cut - 30, cut); r2, g2 = rate(k, cut, cut + 30)
        print(f"{k:24s} 전환 전 30s {r1:5.1f} Hz (최대 공백 {g1 if g1 is None else round(g1, 2)} s) | 전환 후 30s {r2:5.1f} Hz (최대 공백 {g2 if g2 is None else round(g2, 2)} s)")
    # 전환 순간 위치 점프
    mux = [(t, m.pose.pose.position.x, m.pose.pose.position.y) for t, m in ev['/Odometry_mux']]
    a = [p for p in mux if p[0] < cut][-1:]; b = [p for p in mux if p[0] >= cut][:1]
    if a and b: print(f"전환 순간 위치 변화 {np.hypot(b[0][1] - a[0][1], b[0][2] - a[0][2]):.3f} m")
st = [json.loads(m.data) for _, m in ev['/backup/status']]
if st: print('백업 상태 마지막', st[-1], '| ok 비율', round(np.mean([s['health'] == 'ok' for s in st]), 3))
