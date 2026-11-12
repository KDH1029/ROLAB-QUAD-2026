#!/usr/bin/env python3
"""cmd_vel_to_sport 의 위치추정 감시를 ROS 없이 시험한다.

확인할 것 — 전부 '영구 차단이 사라졌는가' 에 관한 것이다:
  1) 점프 한 번은 보류가 아니다 (2026-09-12 에 런을 끝낸 그 경우)
  2) 점프가 연속 3회면 보류된다
  3) 보류된 뒤 정상 값이 오면 **스스로 풀린다**
  4) NaN 한 장 뒤 정상 값이 오면 스스로 풀린다
  5) 폭주(±100m 초과)는 돌아오지 않는 한 계속 보류된다
  6) 정상 주행에서 lio 버퍼가 실제로 쌓인다 (표류 감시가 살아났는가)
"""
import sys, types, math, os

# ── ROS 를 흉내 낸다. 이 시험은 판단 로직만 본다 ──────────────────────
def stub(name, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(m, k, v)
    sys.modules[name] = m
    return m

class _Any:
    def __init__(self, *a, **k): pass
    def __getattr__(self, n): return _Any()
    def __call__(self, *a, **k): return _Any()

stub('rclpy', init=_Any(), spin=_Any(), shutdown=_Any(), ok=lambda: True)
stub('rclpy.node', Node=object)
stub('rclpy.qos', QoSProfile=_Any, ReliabilityPolicy=_Any())
stub('geometry_msgs'); stub('geometry_msgs.msg', Twist=_Any, PoseWithCovarianceStamped=_Any)
stub('sensor_msgs'); stub('sensor_msgs.msg', LaserScan=_Any)
stub('nav_msgs'); stub('nav_msgs.msg', Odometry=_Any)
stub('unitree_api'); stub('unitree_api.msg', Request=_Any)
stub('std_msgs'); stub('std_msgs.msg', String=_Any)

SRC = os.path.expanduser('~/ws/go2_ctrl/cmd_vel_to_sport.py')
ns = {'__name__': 'mod_under_test', '__file__': SRC}
exec(compile(open(SRC, encoding='utf-8').read(), SRC, 'exec'), ns)
C = ns['CmdVelToSport']

LOG = []
class Logger:
    def error(self, m): LOG.append(('E', m))
    def warn(self, m):  LOG.append(('W', m))
    def info(self, m):  LOG.append(('I', m))

class Msg:
    def __init__(self, x, y, z):
        self.pose = types.SimpleNamespace(
            pose=types.SimpleNamespace(position=types.SimpleNamespace(x=x, y=y, z=z)))

def fresh():
    n = C.__new__(C)
    n._t = 0.0
    n.get_logger = lambda: Logger()
    n._now_s = lambda: n._t
    n.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=n._t*1e9))
    n._send_stop = lambda: None
    n._send_move = lambda *a: None
    n.prev_pos = None; n.last_odom_t = None; n.blocked = False
    n._holds = {}; n._step_streak = 0; n._drift_streak = 0; n._hold_nag = 0.0
    n._odom_hold = False; n.odom_seen = False; n.was_active = False
    n.guard_enabled = True; n.drift_guard = True; n.drift_tol = ns['DRIFT_TOL']
    n.lio = ns['OdomTrack'](ns['WINDOW_S']); n.rob = ns['OdomTrack'](ns['WINDOW_S'])
    return n

def feed(n, pts, dt=0.05):
    for p in pts:
        n._t += dt
        n.on_lio(Msg(*p))

fails = []
def check(name, cond, detail=''):
    print(('  %s  %s' % ('✔' if cond else '✘', name)) + (('   ' + detail) if detail else ''))
    if not cond: fails.append(name)

print('\n【1】 점프 한 번은 보류가 아니다  (2026-09-12 에 런을 끝낸 경우)')
n = fresh(); LOG.clear()
feed(n, [(0,0,0), (0.05,0,0), (0.10,0,0)])
feed(n, [(1.41,0,0)])                 # 1.31m 점프 — 어젯밤 실제 값
feed(n, [(1.46,0,0), (1.51,0,0), (1.56,0,0)])
check('점프 1회 뒤에도 주행한다', not n._holds, '보류=%s' % (list(n._holds) or '없음'))
check('연속 카운터가 복귀 후 0 으로 돌아간다', n._step_streak == 0, 'streak=%d' % n._step_streak)

print('\n【2】 점프가 연속 3회면 보류된다')
n = fresh(); LOG.clear()
feed(n, [(0,0,0), (0.05,0,0)])
feed(n, [(2.0,0,0), (4.0,0,0), (6.0,0,0)])
check('연속 3회에 보류된다', '점프' in n._holds, '보류=%s' % list(n._holds))

print('\n【3】 보류된 뒤 정상 값이 오면 스스로 풀린다  ★ 핵심')
before = dict(n._holds)
feed(n, [(6.05,0,0), (6.10,0,0), (6.15,0,0)])
check('사람 개입 없이 자동 재개', not n._holds,
      '보류 %s -> %s' % (list(before), list(n._holds) or '없음'))

print('\n【4】 NaN 한 장 뒤 정상 값이 오면 스스로 풀린다')
n = fresh(); LOG.clear()
feed(n, [(0,0,0), (0.05,0,0)])
feed(n, [(float('nan'), 0, 0)])
held_nan = 'NaN' in n._holds
feed(n, [(0.10,0,0), (0.15,0,0)])
check('NaN 이면 즉시 보류', held_nan)
check('정상 복귀 시 자동 재개', not n._holds, '보류=%s' % (list(n._holds) or '없음'))

print('\n【5】 진짜 폭주는 돌아오지 않는 한 계속 보류된다  (08-24 전복 사고 방어)')
n = fresh(); LOG.clear()
feed(n, [(0,0,0), (0.05,0,0)])
feed(n, [(269505.0, 0, 0)] * 5)
check('폭주는 보류 상태를 유지', '폭주' in n._holds, '보류=%s' % list(n._holds))

print('\n【6】 정상 주행에서 표류 감시 버퍼가 실제로 쌓인다')
n = fresh(); LOG.clear()
feed(n, [(i*0.045, 0, 0.0) for i in range(60)])   # 3초, 평지
check('lio 버퍼에 표본이 쌓인다', len(n.lio.buf) > 10, '표본 %d개' % len(n.lio.buf))
check('보류 없이 주행한다', not n._holds)

print('\n【7】 감시를 끄면 정말로 꺼진다')
n = fresh(); n.guard_enabled = False; LOG.clear()
feed(n, [(0,0,0), (999999.0, 0, 0), (0,0,0)])
check('odom_guard=False 면 안 막는다', not n._holds, '보류=%s' % (list(n._holds) or '없음'))

print()
if fails:
    print('실패 %d개: %s' % (len(fails), ', '.join(fails))); sys.exit(1)
print('전부 통과 — 영구 차단이 사라졌고 안전장치는 남아 있다')
