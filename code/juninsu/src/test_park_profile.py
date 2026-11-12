#!/usr/bin/env python3
"""주차 엎드리기·일어나기를 11-04 녹화의 실제 몸 높이 곡선(sportmodestate, 6 Hz 로 뽑은 값)으로 돌려 본다 — 로봇 없이.
  ① 바닥(0.061 m)에 붙은 시간이 PARK_HOLD_S(5.5초) 이상인가 ② BalanceStand 를 다 일어나 안정된 뒤 보내나 ③ 그 뒤 1초 안에 걸음새·이동 명령이 없나
  ④ 같은 곡선을 예전 판정(선 높이 −0.06 이면 착석, −0.03 이면 일어남, 1초)으로 돌리면 예선처럼 짧게 엎드리고 일어나는 중에 보내는가(대조)."""
import os
os.environ['RUNNER_PATH_ONLY'] = '1'
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from unitree_go.msg import SportModeState
import real_course_runner as rcr

STEP = 1 / 6.0   # 녹화에서 뽑은 간격(0.17 초)
DOWN = [0.317, 0.317, 0.313, 0.313, 0.319, 0.327, 0.333, 0.326, 0.306, 0.283, 0.263, 0.239, 0.215, 0.195, 0.172, 0.149, 0.125, 0.100, 0.077, 0.061]   # 145915_arena3 43.8 s~
UP = [0.061, 0.061, 0.062, 0.063, 0.068, 0.120, 0.204, 0.270, 0.324, 0.329, 0.329, 0.329, 0.326, 0.320, 0.320, 0.320, 0.320, 0.320, 0.318, 0.317]     # 145915_arena3 212.97 s~


class Clock:
    t = 1000.0
    def monotonic(self): return self.t


clock = Clock(); rcr.time.monotonic = clock.monotonic
rclpy.init()
n = rcr.RealCourseRunner((0.0, 0.0), [('pin1', (0.3, 0.0)), ('parking', (0.6, 0.0)), ('pin2', (1.2, 0.0)), ('exit', (1.8, 0.0))])
sports = []; moves = []
n.pub_req.publish = lambda m: sports.append((clock.t, m.header.identity.api_id))
n.pub_cmd_vel.publish = lambda m: moves.append((clock.t, abs(m.linear.x) + abs(m.linear.y) + abs(m.angular.z)))
n.pub_goal.publish = lambda m: None
n.pub_camera_active.publish = lambda m: None
o = Odometry(); o.pose.pose.position.x = 0.6; o.pose.pose.orientation.w = 1.0; n.cb_odom(o)
H = {'h': 0.317, 'curve': None, 't0': None}
def feed():
    if H['curve'] is not None:
        k = int((clock.t - H['t0'] - 0.3) / STEP)          # 명령 → 움직임 시작 0.3 초(가정)
        H['h'] = H['curve'][min(max(k, 0), len(H['curve']) - 1)]
    m = SportModeState(); m.velocity = [0.0, 0.0, 0.0]; m.body_height = H['h']; n.cb_state(m)
feed()
n.idx = 1; n.dispatch_arrival(); assert n.state == 'PARK_PRE'
t_lie = t_up = t_bal = None; hist = []
while clock.t < 1030 and n.state not in ('PARK_REALIGN', 'ROTATE', 'TO_GOAL', 'PIN_TRACK'):
    for _ in range(5):                                       # 높이 50 Hz, tick 10 Hz
        clock.t += 0.02; feed()
    n.tick(); hist.append((clock.t, H['h'], n.state))
    last = sports[-1][1] if sports else None
    if last == rcr.STANDDOWN_ID and H['curve'] is not DOWN:
        H['curve'] = DOWN; H['t0'] = sports[-1][0]
    if last == rcr.STANDUP_ID and H['curve'] is not UP:
        H['curve'] = UP; H['t0'] = sports[-1][0]; t_up = sports[-1][0]
    if t_lie is None and H['h'] < 0.065:
        t_lie = clock.t
    if last == rcr.BALANCESTAND_ID and t_bal is None:
        t_bal = sports[-1][0]
assert t_lie and t_up and t_bal, (t_lie, t_up, t_bal, n.state)
t_end = clock.t
while clock.t < t_end + 3.0:                                 # 그다음(회전·출발) 명령까지 더 돌려 본다
    for _ in range(5):
        clock.t += 0.02; feed()
    n.tick()
lie = t_up + 0.3 - t_lie                                     # 일어나기 시작(명령+0.3)까지 바닥에 붙은 시간
print(f'① 바닥(0.061 m)에 붙은 시간 {lie:.2f} 초 (규정 5초, 목표 {rcr.PARK_HOLD_S})'); assert lie >= 5.0, lie
h_bal = [h for t, h, _ in hist if abs(t - t_bal) < 0.11][0]
win = [h for t, h, _ in hist if t_bal - 0.5 <= t <= t_bal]
t_full = next(t for t, h, _ in hist if t > t_up and h >= 0.315)
print(f'② BalanceStand: StandUp 뒤 {t_bal - t_up:.2f} 초(다 올라온 지 {t_bal - t_full:.2f} 초), 그때 높이 {h_bal:.3f} m, 직전 0.5 초 높이 폭 {1000 * (max(win) - min(win)):.0f} mm')
assert h_bal >= 0.315 and t_bal - t_up >= rcr.RISE_MIN_S and t_bal - t_full >= 0.45 and max(win) - min(win) < 0.01
after = [t for t, a in sports if t > t_bal] + [t for t, v in moves if t > t_bal and v > 1e-3]
first = min(after) - t_bal if after else float('inf')
print(f'③ BalanceStand 뒤 첫 걸음새·이동 명령까지 {first:.2f} 초 (≥ {rcr.POST_STAND_S})'); assert first >= rcr.POST_STAND_S - 1e-6

# ④ 대조: 예전 판정(선 높이 h0 기준, 1초)으로 같은 곡선
h0 = 0.317
def at(curve, dt):
    return curve[min(max(int((dt - 0.3) / STEP), 0), len(curve) - 1)]
t_sat = next(dt for dt in np.arange(0, 5, 0.1) if at(DOWN, dt) < h0 - rcr.SIT_DROP_M)
t_flat = next(dt for dt in np.arange(0, 5, 0.02) if at(DOWN, dt) < 0.065)
t_old_up = next(dt for dt in np.arange(1.0, 5, 0.2) if at(UP, dt) > h0 - 0.03)
print(f'④ 대조(예전 판정): 착석 확인 {t_sat:.1f} 초 → 바닥까지 {t_flat - t_sat:.1f} 초 더 → 바닥 시간 {rcr.PARK_HOLD_S - (t_flat - t_sat):.1f} 초 · '
      f'일어남 확인 {t_old_up:.1f} 초(높이 {at(UP, t_old_up):.3f}, 곡선 안정 {STEP * 13 + 0.3:.1f} 초 전)')
print('\n전체 PASS')
