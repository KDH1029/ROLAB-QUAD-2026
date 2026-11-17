#!/usr/bin/env python3
"""runtime 패치의 Ctl 쪽 변경을 가짜 subprocess·임시 LOG 폴더로 시험(ROS·로봇·실제 부품 없음, 실제 LOG 안 건드림).
사용: python3 test_ctl_patch.py [HEAD|patched]"""
import json, os, subprocess as real_sp, sys, tempfile, types
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
which = sys.argv[1] if len(sys.argv) > 1 else 'patched'
import ctl_loader
if which == 'patched':
    ctl_loader.CTL = os.path.join(HERE, 'go2_ctl.patched.py'); os.environ['HARNESS_CTL_AS'] = os.path.expanduser('~/ws/junsu_v2/tools/go2_ctl.py')
g = ctl_loader.load()
TMP = tempfile.mkdtemp(prefix='ctltest_', dir=os.environ.get('SCRATCH', '/tmp'))
g.LOG = TMP; g.STATE = os.path.join(TMP, 'state.json')

class FakeSP:
    """subprocess 흉내: pgrep 결과는 alive 집합으로, ros2 lifecycle set 은 hang=True 면 TimeoutExpired."""
    DEVNULL = real_sp.DEVNULL; STDOUT = real_sp.STDOUT; TimeoutExpired = real_sp.TimeoutExpired; PIPE = real_sp.PIPE
    def __init__(self): self.alive = set(); self.hang = False; self.calls = []; self.align_writes = None; self.align_rc = 0
    def run(self, args, **k):
        self.calls.append(args if isinstance(args, str) else ' '.join(args))
        r = types.SimpleNamespace(returncode=0, stdout='', stderr='')
        if isinstance(args, list) and args[0] == 'pgrep':
            r.returncode = 0 if any(n in args[-1] for n in self.alive) else 1
        elif isinstance(args, list) and args[0] == 'pkill':
            for n in list(self.alive):
                if n in args[-1]: self.alive.discard(n)
        elif isinstance(args, list) and args[:3] == ['ros2', 'lifecycle', 'set'] and self.hang:
            raise real_sp.TimeoutExpired(args, k.get('timeout'))
        elif isinstance(args, list) and args[:3] == ['ros2', 'lifecycle', 'get']:
            r.stdout = 'active [3]'
        elif isinstance(args, list) and args[0] == 'python3' and args[1].endswith('go2_start_align.py'):
            r.returncode = self.align_rc
            if self.align_writes:   # 정렬 도구가 쓰는 파일 흉내
                out = args[3]; json.dump({'route': []}, open(out, 'w')); json.dump(self.align_writes, open(out + '.align.json', 'w'))
        return r
    def Popen(self, args, **k):
        self.calls.append('Popen ' + args[-1])
        if 'real_course_runner.py' in args[-1]: self.alive.add('real_course_runner.py')
        return types.SimpleNamespace(pid=999999)
sp = FakeSP(); g.subprocess = sp
g.time.sleep = lambda s: None   # 기다림 생략(모듈의 time 은 진짜 time 모듈이라 여기서만 바꿈)
g.os.killpg = lambda *a: None

class FakeMon:
    def __init__(self): self.hz = {}; self.runner = {'idx': 4, 'label': 'rabaout'}; self.n_stop = 0
    def stopmove(self): self.n_stop += 1; return True

def mk(alive=()):
    sp.alive = set(alive); sp.calls.clear(); sp.hang = False
    c = object.__new__(g.Ctl); c.mon = FakeMon(); c.procs = {}; c.msg = ''; c.lc = {'go2_cmd_vel_to_sport': 'active'}; c.last_runner_idx = 0
    c.specs = {s[0]: s for s in g.comp_specs()}
    return c

res = {}
# 1) x(비상 정지) 때 ros2 lifecycle set 이 멈추면?
c = mk({'real_course_runner.py', 'cmd_vel_to_sport_node'}); sp.hang = True
try:
    c.estop(); res['estop_lifecycle_hang'] = f"예외 없음, 러너 남음={'real_course_runner.py' in sp.alive}, StopMove {c.mon.n_stop}번"
except Exception as e:
    res['estop_lifecycle_hang'] = f"★ 예외 {type(e).__name__} (관리자 튕김), 러너 남음={'real_course_runner.py' in sp.alive}, StopMove {c.mon.n_stop}번"
# 2) a(전체 켜기) 때 남은 러너
c = mk({'real_course_runner.py'}); g.OPS['guard'] = 'off'; g.OPS['loc'] = 'on'
try:
    c.start_all(); res['start_all_stale_runner'] = f"러너 남음={'real_course_runner.py' in sp.alive}"
except Exception as e:
    res['start_all_stale_runner'] = f'예외 {e!r}'
# 3) 정렬 도구가 파일을 못 쓰고 죽음 + 지난번 파일(다른 경로) 남아 있음
c = mk(); old = os.path.join(TMP, 'route_today.json')
json.dump({'route': 'OLD'}, open(old, 'w')); json.dump({'ok': True, 'T': [0.2, 0.1, 0.05], 'waypoints_in': '/x/route_map.json'}, open(old + '.align.json', 'w'))
g.OPS['route'] = os.path.expanduser('~/ws/junsu_v2/maps/route_map_1106_hybrid.json'); sp.align_writes = None; sp.align_rc = 1
r = c.align(); res['align_tool_died'] = f"러너에 넘길 파일={os.path.basename(r)} (지난번 파일 그대로면 위험) msg={c.msg[:60]}"
# 4) 정렬 정상
sp.align_writes = {'ok': True, 'T': [0.1, 0.0, 0.02], 'match': 0.97, 'reason': '', 'waypoints_in': g.OPS['route']}; sp.align_rc = 0
r = c.align(); res['align_ok'] = f"파일={os.path.basename(r)} msg={c.msg[:50]}"
# 5) 관리자를 다시 켠 뒤 R(이어가기): route_run 없음, route_today.json 은 지금 경로로 만든 것
c = mk(); c.runner_start(5)
pop = [x for x in sp.calls if x.startswith('Popen')]
res['R_after_restart'] = f"러너 경로={'route_today.json' if any('route_today.json' in x for x in pop) else os.path.basename(g.OPS['route'])+'(정렬 안 된 지도 좌표)'}"
print(json.dumps({'which': which, **res}, ensure_ascii=False, indent=1))
