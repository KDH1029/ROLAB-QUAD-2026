#!/usr/bin/env python3
"""점이 바뀔 때(러너 idx 증가) 앞 1초 ~ 뒤 2.5초 속도. '멈칫' = 그 사이 실제 속도가 0.08 m/s 아래로 떨어진 시간."""
import rosbag2_py, json, numpy as np, collections
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
B = '/home/ilovehan9283/bags/gosu_20261106_111853_cone1106'
TY = {'/lf/sportmodestate': 'unitree_go/msg/SportModeState', '/mission/runner_status': 'std_msgs/msg/String', '/ego_planner/trajectory': 'nav_msgs/msg/Path', '/goal_pose': 'geometry_msgs/msg/PoseStamped', '/cmd_vel': 'geometry_msgs/msg/Twist'}
M = {k: get_message(v) for k, v in TY.items()}
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=B, storage_id='mcap'), rosbag2_py.ConverterOptions('', '')); r.set_filter(rosbag2_py.StorageFilter(topics=list(TY)))
sp = []; rs = []; tr = []; gp = []; cv = []
while r.has_next():
    tp, raw, t = r.read_next(); t *= 1e-9; m = deserialize_message(raw, M[tp])
    if tp == '/lf/sportmodestate': sp.append((t, float(np.hypot(m.velocity[0], m.velocity[1])), m.yaw_speed))
    elif tp == '/mission/runner_status': d = json.loads(m.data); rs.append((t, d.get('idx'), d.get('label'), d.get('state')))
    elif tp == '/ego_planner/trajectory': tr.append(t)
    elif tp == '/goal_pose': gp.append(t)
    elif tp == '/cmd_vel': cv.append((t, float(np.hypot(m.linear.x, m.linear.y)), m.angular.z))
sp = np.array(sp); tr = np.array(tr); gp = np.array(gp); cv = np.array(cv)
sw = [(rs[i][0], rs[i - 1][2], rs[i][2], rs[i - 1][3]) for i in range(1, len(rs)) if rs[i][1] != rs[i - 1][1]]
print(f'점 바뀜 {len(sw)}번')
tot = 0; rows = []
for t, a, b, sa in sw:
    w = (sp[:, 0] > t - 1.0) & (sp[:, 0] < t + 2.5); v = sp[w]
    if not len(v): continue
    slow = (v[:, 1] < 0.08) & (np.abs(v[:, 2]) < 0.3); dt = np.median(np.diff(sp[:, 0])); lost = slow.sum() * dt
    # 새 목표 뒤 첫 ego 궤적까지 · 첫 명령까지
    tt = tr[tr > t]; first_tr = (tt[0] - t) if len(tt) else 9
    cc = cv[(cv[:, 0] > t) & (cv[:, 1] > 0.05)]; first_cmd = (cc[0, 0] - t) if len(cc) else 9
    tot += lost; rows.append((lost, a, b, sa, v[:, 1].min(), first_tr, first_cmd))
for x in sorted(rows, key=lambda r: -r[0]):
    print(f'  {x[1]:>9s} → {x[2]:9s} ({x[3]:12s}) 느림 {x[0]:.1f}s · 최저 {x[4]:.2f} m/s · 새 궤적까지 {x[5]:.2f}s · 첫 이동명령까지 {x[6]:.2f}s')
print(f'점 바뀜 근처 멈칫 합 {tot:.1f}s')
# 주행 전체에서 속도 분포(TO_GOAL 중)
