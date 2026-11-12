#!/usr/bin/env python3
"""시뮬레이션 결과 정리: 시간표 + 규정 점검 + 그림. 사용: sim_report.py <runs/sim_시각>"""
import sys, os, json, re, numpy as np
O = sys.argv[1].rstrip('/'); E = [json.loads(l) for l in open(os.path.join(O, 'events.jsonl'))]
wp = json.load(open(os.path.join(O, 'waypoints_sim.json'))); wp = wp['route'] if isinstance(wp, dict) else wp   # route_v1(지도에서 정한 경로)도 읽음
R = open(os.path.join(O, 'runner.log')).read() if os.path.exists(os.path.join(O, 'runner.log')) else ''
lines = []
def say(s): lines.append(s); print(s)
say(f'== 시뮬레이션 {os.path.basename(O)} — {E[-1]["t"]:.0f}초')
# 시간표(위치 기록 제외)
say('\n[시간표]')
for e in E:
    if e['ev'] in ('pose',): continue
    extra = {k: v for k, v in e.items() if k not in ('t', 'ev', 'x', 'y', 'yaw', 'v', 'light', 'cam')}
    say(f"  {e['t']:7.1f}s  {e['ev']:13s} 위치({e['x']:5.2f},{e['y']:5.2f}) 속도 {e['v']:.2f}  실제신호 {e['light']:5s}  {extra if extra else ''}")
# 정지선 점검: camera_active ON~OFF 구간
say('\n[정지선 규정 점검] 앞다리 표식 안 2초 이상 정지 + 녹색일 때만 출발')
on = None
for e in E:
    if e['ev'] == 'camera_active' and e['on']: on = e
    if e['ev'] == 'camera_active' and not e['on'] and on:
        poses = [p for p in E if p['ev'] == 'pose' and on['t'] <= p['t'] <= e['t']]
        still = max([p['v'] for p in poses] or [0])
        seen = [x['state'] for x in E if x['ev'] == 'tl_state' and on['t'] <= x['t'] <= e['t']]
        ok = e['light'] == 'GREEN' and e['t'] - on['t'] >= 2.0 and still < 0.05
        say(f"  {on['t']:.1f}s~{e['t']:.1f}s ({e['t']-on['t']:.1f}초) 정지 중 최대속도 {still:.3f} m/s · 판정 흐름 {seen} · 출발 순간 실제 신호 {e['light']} → {'✔ 규정 지킴' if ok else '★ 위반'}")
        on = None
if on: say(f"  {on['t']:.1f}s 부터 카메라 켜진 채 끝남 — 정지선에서 못 떠남")
# 주차
sits = [e for e in E if e['ev'] == 'sport' and e['api'] == 1009]; rises = [e for e in E if e['ev'] == 'sport' and e['api'] == 1010]
if sits:
    say(f"\n[주차] Sit {sits[0]['t']:.1f}s ({sits[0]['x']:.2f},{sits[0]['y']:.2f}) → RiseSit {rises[0]['t'] if rises else '없음'}s · 앉아 있던 시간 {(rises[0]['t'] - sits[0]['t']) if rises else 0:.1f}초 (규정 5초)")
cs = [e for e in E if e['ev'] == 'end']; c = cs[-1]['contact_s'] if cs else max([e.get('contact_s', 0) for e in E if e['ev'] == 'contact'] or [0])
say(f"\n[접촉] 벽까지 0.15 m 미만이었던 시간 {c:.1f}초 (규정: 접촉 1초당 −10점)")
gl = re.findall(r'\[([a-z0-9]+)\] 도착', R); say(f"[도착한 지점] {gl}  ·  완주 {'예' if '전부 도착' in R else '아니오'}")
fails = len(re.findall(r'A\*|failed|실패', open(os.path.join(O, 'ego.log')).read())) if os.path.exists(os.path.join(O, 'ego.log')) else -1
say(f"[ego_planner 경로 실패 로그] {fails}줄")
open(os.path.join(O, 'report.txt'), 'w').write('\n'.join(lines))
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
plt.rcParams['font.family'] = ['NanumGothic', 'DejaVu Sans']
wu = os.path.join(O, 'walls_used.txt'); W = np.load(open(wu).read().strip() if os.path.exists(wu) else os.path.expanduser('~/ws/junsu/sim/sim_walls.npy')); P = np.array([[e['x'], e['y'], e['v']] for e in E if e['ev'] == 'pose'])
fig, ax = plt.subplots(figsize=(12, 9)); ax.scatter(W[:, 0], W[:, 1], s=0.4, c='k')
if len(P): sc = ax.scatter(P[:, 0], P[:, 1], c=P[:, 2], s=6, cmap='viridis', vmin=0, vmax=0.8); plt.colorbar(sc, label='속도 m/s')
for w in wp: ax.scatter(w['x'], w['y'], c='r', s=50, zorder=5); ax.annotate(w['label'], (w['x'], w['y']), xytext=(4, 4), textcoords='offset points', color='darkred')
for e in E:
    if e['ev'] == 'contact': ax.scatter(e['x'], e['y'], marker='x', c='orange', s=40)
    if e['ev'] == 'camera_active': ax.scatter(e['x'], e['y'], marker='s' if e['on'] else 'D', facecolors='none', edgecolors='m', s=120)
ax.set_xlim(-3, 9); ax.set_ylim(-2.5, 7); ax.set_aspect('equal'); ax.grid(alpha=.3)
ax.set_title('시뮬레이션 경로(색=속도) · 벽(검정) · 목표(빨강) · 접촉(주황 x) · 카메라 켬□/끔◇'); plt.tight_layout(); plt.savefig(os.path.join(O, 'report.png'), dpi=80)
print('그림', os.path.join(O, 'report.png'))
