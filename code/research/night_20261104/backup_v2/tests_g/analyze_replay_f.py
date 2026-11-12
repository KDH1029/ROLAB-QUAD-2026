#!/usr/bin/env python3
"""[F] replay_check_v2.sh 녹화(out/<태그>.rec) 오프라인 분석 — 정렬·가드 상태 변화만 본다(파일 읽기만, DDS 참여 없음).
  · /backup/status: health(가림 포함)·health_raw 변화 시각, align.state 변화(고정 시각·T·사유·경고), gicp 진단 마지막 값
  · /mux/status   : mode·pending·hold_reason 변화 시각 (FAST-LIO 끄기 전 pending/backup 이면 오전환)
  · 전환 순간 /Odometry_mux 위치 점프
사용: source /opt/ros/jazzy/setup.bash && python3 analyze_replay_f.py <태그> [FAST-LIO 끈 시각 s]
"""
import sys, json, math, rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from std_msgs.msg import String
TAG = sys.argv[1]; KILL = float(sys.argv[2]) if len(sys.argv) > 2 else None
D = f'/home/ilovehan9283/ws/night_20261104/backup_v2/out/{TAG}.rec'
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=D, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
T = {'/Odometry_mux': Odometry, '/mux/status': String, '/backup/status': String, '/Odometry': Odometry}
ev = {k: [] for k in T}
while r.has_next():
    tp, d, t = r.read_next()
    if tp in T: ev[tp].append((t * 1e-9, deserialize_message(d, T[tp])))
t0 = min(v[0][0] for v in ev.values() if v)
print(f'== {TAG}  (시각 = 녹화 시작 기준 초)')
last = None
for t, m in ev['/backup/status']:
    s = json.loads(m.data); a = s.get('align', {})
    key = (s.get('health'), s.get('health_raw'), a.get('state'))
    if key != last:
        print(f"  {t - t0:7.1f}s backup health={key[0]} raw={key[1]} align={key[2]} T={a.get('T')} {('경고: ' + a['warn']) if a.get('warn') else ''} {a.get('why', '')[:90]}")
        last = key
if ev['/backup/status']:
    s = json.loads(ev['/backup/status'][-1][1].data)
    print('  마지막 gicp 진단', s.get('gicp'), '| align', {k: s['align'].get(k) for k in ('state', 'T', 'pairs', 'fl_jumps', 'frozen_after_s')}, '| 예외', s.get('exceptions'))
print('  /mux/status 변화:')
sw = None
for t, m in ev['/mux/status']:
    s = json.loads(m.data)
    print(f"  {t - t0:7.1f}s mode={s.get('mode')} pending={s.get('pending')} hold={s.get('hold_reason', '')} reason={s.get('reason', '')} backup={s.get('backup_health')}")
    if s.get('mode') == 'backup' and sw is None: sw = t
if sw is not None:
    O = ev['/Odometry_mux']; i = min(range(len(O)), key=lambda k: abs(O[k][0] - sw)) if O else None
    if O and 0 < i < len(O) - 3:
        jumps = [math.hypot(O[k + 1][1].pose.pose.position.x - O[k][1].pose.pose.position.x, O[k + 1][1].pose.pose.position.y - O[k][1].pose.pose.position.y) for k in range(max(0, i - 5), min(len(O) - 1, i + 10))]
        print(f'  전환 {sw - t0:.1f}s, 전환 전후 /Odometry_mux 한 칸 이동 최대 {max(jumps):.3f} m')
    if KILL is not None:
        print('  ★ 오전환 (FAST-LIO 끄기 전 전환)' if sw - t0 < KILL - 1.0 else '  전환은 FAST-LIO 끈 뒤 — 오전환 없음')
else:
    print('  전환 없음')
