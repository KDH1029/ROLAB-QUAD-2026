#!/usr/bin/env python3
"""slam_replay_check.sh 결과 분석 — 오도메트리가 경기장 안인가, 목표마다 경로가 나왔나, 경로가 목표에 닿나, 벽을 뚫나.
사용: slam_check_analyze.py <태그>
"""
import sys, json, re, numpy as np, rosbag2_py, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
for f in ['NanumGothic', 'NanumBarunGothic', 'Noto Sans CJK KR']:
    if any(f in x.name for x in fm.fontManager.ttflist): plt.rcParams['font.family'] = f; break
plt.rcParams['axes.unicode_minus'] = False
TAG = sys.argv[1]; O = '/home/ilovehan9283/ws/v_gosu/eval/out/'
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=O + TAG + '.slam_rec', storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
T = {'/Odometry': Odometry, '/goal_pose': PoseStamped, '/ego_planner/trajectory': Path, '/cmd_vel': Twist}
od, goals, paths, cmd = [], [], [], []
while r.has_next():
    tp, d, t = r.read_next()
    if tp not in T: continue
    m = deserialize_message(d, T[tp]); t *= 1e-9
    if tp == '/Odometry': p = m.pose.pose.position; od.append((t, p.x, p.y, p.z))
    elif tp == '/goal_pose': goals.append((t, m.pose.position.x, m.pose.position.y))
    elif tp == '/ego_planner/trajectory':
        paths.append((t, np.array([[q.pose.position.x, q.pose.position.y, q.pose.position.z] for q in m.poses])))
    else: cmd.append((t, m.linear.x, m.linear.y, m.angular.z))
od = np.array(od); cmd = np.array(cmd) if cmd else np.zeros((0, 4)); t0 = od[0, 0]
# 목표 구간: 같은 목표가 3번 연달아 발행되므로 바뀔 때만 구간 시작
G = []
for t, x, y in goals:
    if not G or (abs(G[-1][1] - x) + abs(G[-1][2] - y)) > 1e-3: G.append((t, x, y))
# XT16 지도(2 cm 복셀, 바닥 위 0.3~1.5 m = 벽) 로 경로가 벽을 뚫는지 본다
mp = np.load(O + 'gosu1030v2_map.npz'); V = float(mp['V']); P = (mp['vox'][mp['cnt'] >= 2] + 0.5) * V
sel = (P[:, 2] > np.percentile(P[:, 2], 5) + 0.45) & (P[:, 2] < np.percentile(P[:, 2], 5) + 1.5)   # 경사로 윗면(약 0.3 m)은 벽으로 안 친다
W = P[sel][:, :2]; C = 0.05; x0, y0 = W.min(0) - 1
occ = np.zeros((int((W[:, 1].max() + 1 - y0) / C) + 1, int((W[:, 0].max() + 1 - x0) / C) + 1), bool)
occ[((W[:, 1] - y0) / C).astype(int), ((W[:, 0] - x0) / C).astype(int)] = True
def hits_wall(pp):
    ix = ((pp[:, 0] - x0) / C).astype(int); iy = ((pp[:, 1] - y0) / C).astype(int)
    ok = (ix >= 0) & (ix < occ.shape[1]) & (iy >= 0) & (iy < occ.shape[0]); return int(occ[iy[ok], ix[ok]].sum())
rows = []
for k, (tg, gx, gy) in enumerate(G):
    te = G[k + 1][0] if k + 1 < len(G) else od[-1, 0]
    ps = [(t, p) for t, p in paths if tg <= t < te and len(p)]
    ends = [np.hypot(p[-1, 0] - gx, p[-1, 1] - gy) for t, p in ps]
    wall = sum(hits_wall(p) > 0 for t, p in ps)
    i = np.searchsorted(od[:, 0], tg); robot = od[min(i, len(od) - 1), 1:3]
    rows.append(dict(시작s=round(tg - t0, 1), 목표=(gx, gy), 그때_로봇=[round(v, 2) for v in robot], 경로수=len(ps),
                     경로끝_목표거리_중앙m=round(float(np.median(ends)), 3) if ends else None,
                     벽뚫은경로=wall, 첫경로까지s=round(ps[0][0] - tg, 2) if ps else None))
lg = open(O + TAG + '.slam.launch.log', errors='ignore').read()
S = dict(오도=dict(x=[round(od[:, 1].min(), 2), round(od[:, 1].max(), 2)], y=[round(od[:, 2].min(), 2), round(od[:, 2].max(), 2)],
                   z=[round(od[:, 3].min(), 2), round(od[:, 3].max(), 2)],
                   경로m=round(float(np.sum(np.linalg.norm(np.diff(od[:, 1:4], axis=0), axis=1))), 1),
                   끝_출발거리m=round(float(np.hypot(od[-1, 1], od[-1, 2])), 2), 표본=len(od)),
         목표구간=rows, 경로메시지=len(paths), cmd_vel=dict(개수=len(cmd), 영아닌비율=round(float((np.abs(cmd[:, 1:]).sum(1) > 1e-3).mean()), 3) if len(cmd) else 0,
                                                     vx최대=round(float(np.abs(cmd[:, 1]).max()), 2) if len(cmd) else 0, wz최대=round(float(np.abs(cmd[:, 3]).max()), 2) if len(cmd) else 0),
         플래너경고=dict(재계획실패=len(re.findall('replanning attempt failed', lg)), 충돌=len(re.findall('(?i)collision', lg)),
                    비상=len(re.findall('(?i)emergency', lg)), 목표수신=len(re.findall('Received new target goal', lg))))
json.dump(S, open(O + TAG + '.slam_check.json', 'w'), ensure_ascii=False, indent=1); print(json.dumps(S, ensure_ascii=False, indent=1))
# 그림: 벽 지도 위에 오도 궤적, 목표, 목표마다 마지막 경로
fig, ax = plt.subplots(figsize=(9, 8.5), dpi=130)
ax.imshow(occ, origin='lower', extent=(x0, x0 + occ.shape[1] * C, y0, y0 + occ.shape[0] * C), cmap='Greys', alpha=0.6)
ax.plot(od[:, 1], od[:, 2], color='#1f77b4', lw=0.7, label='FAST-LIO 오도메트리 (bag 의 실제 움직임)')
cols = plt.cm.tab10(np.arange(len(G)) % 10)
for k, (tg, gx, gy) in enumerate(G):
    te = G[k + 1][0] if k + 1 < len(G) else od[-1, 0]
    ps = [p for t, p in paths if tg <= t < te and len(p)]
    if ps: ax.plot(ps[-1][:, 0], ps[-1][:, 1], '-', color=cols[k], lw=1.6)
    ax.plot(gx, gy, '*', ms=14, color=cols[k], mec='k'); ax.annotate(f'목표{k + 1}', (gx, gy), xytext=(5, 5), textcoords='offset points', fontsize=9)
ax.plot(0, 0, 'o', mfc='w', mec='k'); ax.set_aspect('equal'); ax.grid(alpha=0.3); ax.legend(fontsize=9, loc='lower left')
ax.set_xlim(-2, 9); ax.set_ylim(-6.5, 4.5); ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
ax.set_title(f'통째 SLAM 무모터 재생 — {TAG}\n별 = 목표(camera_init), 색 선 = 그 목표에 대한 ego_planner 마지막 경로, 회색 = XT16 벽(바닥 위 0.45~1.5 m)', fontsize=11)
fig.tight_layout(); fig.savefig(O + TAG + '.slam_check.png'); print('그림', O + TAG + '.slam_check.png')
