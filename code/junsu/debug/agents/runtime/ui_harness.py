#!/usr/bin/env python3
"""go2_ctl.py 의 실제 ui() 를 가짜 Mon/Ctl 로 돌린다(ROS·로봇·부품 없음). pty 안에서 drive_ui.py 가 띄운다.
- stdscr 를 프록시로 감싸 addstr/getstr 의 화면 폭 초과·curses.error 를 기록
- 1초마다 화면 갱신 횟수(=ui 루프 횟수)를 기록(ask 뒤 바쁜 루프 확인)
- ui 가 예외로 끝나면 traceback 을 기록하고 종료 코드 3
사용: python3 ui_harness.py <이벤트로그> [variant]
  variant: normal | nounitree(LowState None) | bigdata(긴 라벨·None 값) | patched(패치 적용본 go2_ctl.py 경로를 HARNESS_CTL 로)"""
import curses, json, os, sys, threading, time, traceback, unicodedata
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ctl_loader
if os.environ.get('HARNESS_CTL'):
    ctl_loader.CTL = os.environ['HARNESS_CTL']
LOGF = sys.argv[1]; VAR = sys.argv[2] if len(sys.argv) > 2 else 'normal'
g = ctl_loader.load(with_unitree=(VAR != 'nounitree'))
ev = open(LOGF, 'a', buffering=1)
def log(**k):
    k['t'] = round(time.monotonic(), 3); ev.write(json.dumps(k, ensure_ascii=False) + '\n')

def dw(t):
    return sum(2 if unicodedata.east_asian_width(ch) in ('W', 'F') else 1 for ch in t)

class Spy:
    """stdscr 프록시 — 실제 curses 창에 그대로 넘기되 넘침·오류를 기록."""
    def __init__(self, scr): self._s = scr
    def __getattr__(self, k): return getattr(self._s, k)
    def addstr(self, *a):
        y, x, text = (a[0], a[1], a[2]) if len(a) >= 3 and isinstance(a[0], int) else (None, None, a[0])
        h, w = self._s.getmaxyx()
        if y is not None and x + dw(text) > w:
            log(ev='addstr_overflow', y=y, x=x, dw=dw(text), w=w, text=text[:50])
        try:
            return self._s.addstr(*a)
        except curses.error as e:
            log(ev='addstr_error', y=y, x=x, dw=dw(text), w=w, h=h, err=str(e), text=text[:50]); raise
    def getstr(self, *a):
        h, w = self._s.getmaxyx()
        log(ev='getstr', args=list(a), w=w, h=h)
        try:
            return self._s.getstr(*a)
        except curses.error as e:
            log(ev='getstr_error', args=list(a), w=w, h=h, err=str(e)); raise
    def move(self, *a):
        try:
            return self._s.move(*a)
        except curses.error as e:
            log(ev='move_error', args=list(a), err=str(e)); raise
    def getkey(self):
        k = self._s.getkey(); log(ev='key', k=k); return k

class FakeMon:
    def __init__(self):
        self.hz = {t: 0.0 for t in ('/lidar_points', '/imu/body', '/Odometry', '/mission/wall_cloud', '/mission/obstacle_cloud',
                                    '/mission/cone_cam', '/mission/runner_status', '/mission/traffic_light_state', '/mission/loc_watch', '/mission/guard')}
        self.hz.update({'/lidar_points': 19.8, '/imu/body': 498.0, '/Odometry': 20.1})
        self.pose = (1.234, -0.567, 12.3); self.light = 'GREEN'; self.robot = {'soc': 87, 'temp': 79, 'h': 0.31, 'v': 0.0, 'mode': 1}
        self.loc = {'state': 'OK_WARN', 'e_xy': 0.123, 'e_yaw_deg': 2.1, 'sigma': 0.05, 'T': [0.01, -0.02, 0.03], 'aligned': True,
                    'icp': {'ok': False, 'reason': '스캔 3장뿐 — 아주 긴 이유 문자열 ' * 3}}
        self.runner = {}; self.guard = {}
        if VAR == 'bigdata':
            self.pose = None; self.robot = {}; self.light = '빨강빨강빨강빨강빨강빨강빨강'
            self.loc = {'state': None, 'e_xy': None, 'e_yaw_deg': None, 'sigma': None, 'T': None, 'aligned': False, 'icp': None}
            self.runner = {'state': 'TO_GOAL', 'idx': 3, 'n': 23, 'label': '긴라벨' * 10, 'dist': None, 'cone_zone': True,
                           'backoffs': 2, 'pos': [1.0, 2.0], '_t': 1e18}
        self.calls = []
    def stopmove(self): self.calls.append('stopmove'); return True
    def send_gait_now(self, mode): self.calls.append(('gait', mode)); return f'{mode} 보냄(가짜)'
    def set_ego(self, **kw): self.calls.append(('set_ego', kw)); time.sleep(1.0); return False   # 실제처럼 1초 기다림(서비스 없음)

class FakeCtl:
    def __init__(self, mon):
        self.mon = mon; self.procs = {}; self.msg = '준비'; self.lc = {'go2_cloud_bridge': 'active', 'go2_imu_bridge': 'unconfigured', 'go2_cmd_vel_to_sport': '없음'}
        self.last_runner_idx = 1; self.specs = {s[0]: s for s in g.comp_specs()}; self.loops = 0
    def alive(self, name):
        if name == 'xt16': self.loops += 1
        return name in ('xt16', 'bridge', 'fastlio', 'motor')
    def start(self, name, *a, **k):
        self.specs = {s[0]: s for s in g.comp_specs()}; log(ev='ctl', f='start', name=name); self.msg = f'{name} 켬'
    def stop(self, name, quiet=False): log(ev='ctl', f='stop', name=name); self.msg = f'{name} 끔'
    def start_all(self):
        log(ev='ctl', f='start_all'); self.msg = '★ fastlio: /Odometry 가 안 나옴 — 로그(l) 확인. 나머지는 계속 켬 ' * 3
    def motor_on(self): log(ev='ctl', f='motor_on'); self.msg = '★ 모터 켬 — 이제 로봇이 움직일 수 있음'
    def motor_off(self): log(ev='ctl', f='motor_off'); self.msg = '모터 끔(소프트 정지)'
    def runner_start(self, start_at=None):
        log(ev='ctl', f='runner_start', start_at=start_at)
        self.msg = '출발 정렬 거부(원래 경로): ' + json.dumps({'T': [0.0, 0.0, 0.0], 'match': 0.0, 'reason': 'timeout 20 s (standing 0.00 m, speed 9.00, odom 0, scans 0) - is FAST-LIO / cloud_bridge /scan running?'}, ensure_ascii=False)[:120]
    def pause(self): log(ev='ctl', f='pause'); self.msg = f'정비 멈춤 — 마지막 목표 {self.last_runner_idx}번 [?]. 고친 뒤 R 로 이어가기(FAST-LIO 는 켜 둘 것)'
    def estop(self): log(ev='ctl', f='estop'); self.msg = f'■ 비상 정지 — 모터 끔·StopMove·러너 끔. 마지막 목표 {self.last_runner_idx}번 — m 후 R 엔터로 이어가기'
    def save(self): log(ev='ctl', f='save(무시)')

mon = FakeMon(); ctl = FakeCtl(mon)
log(ev='start', var=VAR, route=g.OPS['route'], lang=os.environ.get('LANG'))
def rate():
    last = 0
    while True:
        time.sleep(1.0); n = ctl.loops; log(ev='rate', loops_per_s=n - last); last = n
threading.Thread(target=rate, daemon=True).start()
code = 0
try:
    curses.wrapper(lambda scr: g.ui(Spy(scr), ctl, mon))
    log(ev='exit_ok', ops={k: g.OPS.get(k) for k in ('route', 'guard', 'gait', 'planner', 'cones', 'align', 'loc', 'yaw_trim', 'park', 'pass_switch', 'turn_in_place', 'inflate_review', 'max_vel')})
except BaseException as e:
    log(ev='CRASH', err=repr(e), tb=traceback.format_exc()); code = 3
ev.close(); sys.exit(code)
