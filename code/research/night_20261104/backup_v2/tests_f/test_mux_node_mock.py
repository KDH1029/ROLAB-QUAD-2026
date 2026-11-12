#!/usr/bin/env python3
"""③ odom_mux_node_v2 노드 껍데기 시험 — DDS 없이. rclpy.init / spin / Node 를 가짜로 바꿔 콜백을 직접 부른다.

확인: (가) 백업 lost 일 때 FAST-LIO 끊겨도 /Odometry_mux 는 FAST-LIO 쪽 유지 + 경고 로그 + /mux/status pending
      (나) 백업이 ok 로 돌아오면 전환, 이후 /Odometry_mux 는 백업 메시지, /cloud_active 는 백업 점구름
      (다) /mux/reset 이 보류·전환을 지움
      (라) spin 이 ExternalShutdownException 을 던져도(= launch Ctrl-C) Traceback 없이 끝남, KeyboardInterrupt 도 같음
      (마) --no-switch 는 중계만
실제 rclpy 는 메시지 타입·QoS 만 쓴다(참여자 안 만듦). 사용: source /opt/ros/jazzy/setup.bash; python3 test_mux_node_mock.py
"""
import sys, json, importlib.util, importlib.machinery
import rclpy, rclpy.node
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from std_msgs.msg import String

V2 = '/home/ilovehan9283/ws/night_20261104/backup_v2'
CLOCK = [0.0]


class _Now:
    def __init__(self, t): self.nanoseconds = int(t * 1e9)
    def to_msg(self):
        from builtin_interfaces.msg import Time
        return Time(sec=int(CLOCK[0]), nanosec=int((CLOCK[0] % 1) * 1e9))


class _Clock:
    def now(self): return _Now(CLOCK[0])


class _Log:
    def __init__(self, sink): self.sink = sink
    def warn(self, m): self.sink.append(('WARN', m))
    def info(self, m): self.sink.append(('INFO', m))


class _Pub:
    def __init__(self, topic, sink): self.topic, self.sink = topic, sink
    def publish(self, m): self.sink.append((self.topic, m))


class FakeNode:
    def __init__(self, name):
        self.subs, self.pubs, self.logs, self.out, self.srv, self.timers = {}, {}, [], [], {}, []
    def create_publisher(self, typ, topic, qos): return _Pub(topic, self.out)
    def create_subscription(self, typ, topic, cb, qos, raw=False): self.subs[topic] = cb
    def create_service(self, typ, name, cb): self.srv[name] = cb
    def create_timer(self, dt, cb): self.timers.append(cb)
    def get_clock(self): return _Clock()
    def get_logger(self): return _Log(self.logs)
    def destroy_node(self): pass


STATE = {}


def odom(x, y, t):
    m = Odometry(); m.pose.pose.position.x, m.pose.pose.position.y = x, y
    m.header.stamp.sec = int(t); m.header.stamp.nanosec = int((t % 1) * 1e9); return m


def scenario(n):
    """FAST-LIO 20 Hz 0~8초, 8초에 끊김. 백업: 상태 1 Hz (0~20초 lost, 20초부터 ok), 위치 50 Hz."""
    tick = n.timers[0]
    for k in range(0, 3000):                     # 10 ms 단위 30초
        t = k * 0.01; CLOCK[0] = t
        if k % 5 == 0 and t < 8.0: n.subs['/Odometry'](odom(0.01 * t, 0, t))
        if k % 2 == 0: n.subs['/backup/odometry'](odom(0.01 * t + 0.02, 0, t))
        if k % 100 == 0: n.subs['/backup/status'](String(data=json.dumps(dict(health='lost' if t < 20 else 'ok'))))
        if k % 10 == 0:
            n.subs['/cloud_registered'](b'FL'); n.subs['/backup/cloud'](b'BK')
        tick()
        if abs(t - 15.0) < 1e-9: STATE['at15'] = (n.mode, n.gm.pending, len(n.out))
    STATE['end_mode'] = n.mode


def load():
    ld = importlib.machinery.SourceFileLoader('mux_v2', f'{V2}/odom_mux_node_v2.py')
    sp = importlib.util.spec_from_file_location('mux_v2', f'{V2}/odom_mux_node_v2.py', loader=ld)
    m = importlib.util.module_from_spec(sp); ld.exec_module(m); return m


def run(argv, spin_exc, body):
    rclpy.node.Node = FakeNode
    rclpy.init = lambda args=None: None
    rclpy.try_shutdown = lambda: STATE.__setitem__('shutdown', True)
    def spin(n):
        STATE['node'] = n; body(n); raise spin_exc
    rclpy.spin = spin
    sys.argv = argv
    mod = load(); STATE.clear(); STATE['shutdown'] = False
    mod._ros_main()                                 # 예외가 새면 여기서 Traceback → 시험 실패
    return STATE['node']


ok_all = True
def check(name, cond):
    global ok_all
    ok_all &= bool(cond); print(('통과 ' if cond else '실패 ') + name)


n = run(['odom_mux_node_v2.py'], ExternalShutdownException(), scenario)
m15, pend15, _ = STATE['at15']
odo = [(i, m) for i, (tp, m) in enumerate(n.out) if tp == '/Odometry_mux']
stats = [json.loads(m.data) for tp, m in n.out if tp == '/mux/status']
warns = [m for lv, m in n.logs if lv == 'WARN']
check('(가) 15초(FAST-LIO 끊김 7초째, 백업 lost): 모드 fastlio 유지', m15 == 'fastlio' and pend15)
check('(가) 보류 경고 로그', any('전환 안 함' in w for w in warns))
check('(가) /mux/status 에 pending=true', any(s['pending'] and s['mode'] == 'fastlio' for s in stats))
check('(나) 백업 ok(20초) 뒤 전환', STATE['end_mode'] == 'backup' and any('보류했었음' in w for w in warns))
check('(나) 전환 뒤 /Odometry_mux 는 백업 위치(x 가 +0.02 앞섬)', abs(odo[-1][1].pose.pose.position.x - (0.01 * 29.98 + 0.02)) < 1e-6)
check('(나) 전환 뒤 /cloud_active 는 백업 점구름', [m for tp, m in n.out if tp == '/cloud_active'][-1] == b'BK')
check('(나) 전환 전 /cloud_active 는 FAST-LIO 점구름', [m for tp, m in n.out if tp == '/cloud_active'][0] == b'FL')
from std_srvs.srv import Trigger
r = n.srv['/mux/reset'](Trigger.Request(), Trigger.Response())
check('(다) /mux/reset → fastlio, 보류 지움', r.success and n.mode == 'fastlio' and n.gm.pending is None and n.gm.mux.mode == 'fastlio')
check('(라) ExternalShutdownException → Traceback 없이 종료 + try_shutdown 호출', STATE['shutdown'])
n2 = run(['odom_mux_node_v2.py'], KeyboardInterrupt(), lambda n: None)
check('(라) KeyboardInterrupt → Traceback 없이 종료', STATE['shutdown'])
n3 = run(['odom_mux_node_v2.py', '--no-switch'], ExternalShutdownException(), scenario)
check('(마) --no-switch: 끝까지 fastlio, 전환 경고 없음', STATE['end_mode'] == 'fastlio' and not any('전환' in m and lv == 'WARN' for lv, m in n3.logs))
n4 = run(['odom_mux_node_v2.py', '--status-timeout', '5', '--odom-timeout', '1.0'], ExternalShutdownException(), lambda n: None)
check('(마) --status-timeout/--odom-timeout 인자', n4.gm.guard.status_timeout_s == 5.0 and n4.gm.guard.odom_timeout_s == 1.0)
print('전부 통과' if ok_all else '실패 있음')
sys.exit(0 if ok_all else 1)
