#!/usr/bin/env python3
"""junsu 올인원 주행 관리자 — NUC 터미널 하나에서 켜기·끄기·상태판·부품 재기동·정비(멈춤/이어가기)·작전 바꾸기·비상 정지.
실행: ~/ws/junsu/tools/go2_ctl.sh   (환경을 go2_full_stack.sh 와 같은 순서로 맞춘 뒤 이 파일을 띄운다)
★ 규정: 모두 로봇 위 NUC 에서 돈다. 화면은 글자 상태뿐(센서 데이터를 밖으로 보내지 않음). 경기 중 조작은 시작 신호·비상·정비 시간만.
★ 이 관리자를 꺼도(q) 부품은 계속 돈다. 전부 끄려면 Q.

키
  a 전체 켜기(라이다→브리지→FAST-LIO→벽→합치기→ego→카메라, 모터·러너 빼고)   Q 전부 끄기
  1~9 그 부품 다시 켜기        m 모터 켜기(★로봇 움직임)  M 모터 끄기(소프트 정지)
  r 러너 처음부터            R 이어가기(번호 입력)       p 정비 멈춤(러너 끔+StopMove+모터 끔)
  x 비상 정지(모터 끔 + StopMove + 러너 끔)              s 작전 바꾸기       l 로그 보기   q 관리자만 끝내기
"""
import curses
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue, ParameterType
try:
    from unitree_api.msg import Request
    from unitree_go.msg import LowState, SportModeState
except Exception:            # 환경이 덜 잡혔으면 StopMove·로봇 상태만 빠진다
    Request = LowState = SportModeState = None

J = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # 이 저장소(브랜치 폴더면 그 폴더)
LOG = os.path.join(J, 'logs')
STATE = os.path.join(LOG, 'go2_ctl_state.json')
BRIDGE_PARAMS = os.path.expanduser('~/ws/merge_tmp_20261102/review/install/go2_bridge/share/go2_bridge/config/bridge_params.yaml')
HESAI_CFG = os.path.expanduser('~/ws/merge_tmp_20261102/review/src/external/hesai_ros_driver/config/config.yaml')
ROBOT_PC = 'unitree@192.168.123.18'
SSH = "ssh -o ControlPath=~/.ssh/go2_cm_%r@%h:%p -o ControlMaster=auto -o ControlPersist=3h -o ConnectTimeout=6 -o BatchMode=yes"

# ── 작전(바꿀 수 있는 값). s 로 바꾸고, 해당 부품을 다시 띄우거나 실행 중 파라미터로 보낸다
OPS = {
    'route': os.path.join(J, 'routes', os.environ.get('JUNK_ROUTE', 'rec1118x'), 'route.json'),   # ★ 11-06 오후: 기본 rec1118x(1114∩1118 실주행 지도 — 1114 단독은 높이 표류로 유령 벽). 전 rec1114·rec1650(전: 가장 최근 수정 경로 — 시험 경로가 기본이 돼 버림). 바꾸기: JUNK_ROUTE=이름 또는 s → 1
    'path_v': 0.30,           # ★ junk: 경로 따라가기 속도(m/s)
    'heading': 'tangent',     # ★ 11-06 경로 따라갈 때 몸 방향: tangent = 경로 방향(기본, 랜덤 시뮬로 몸 닿음 −19 %) | target = 예전(목표 쪽)
    'tl': 'on',               # 신호등: on = YOLO 초록 대기(기본, 18:20 다시 켬) | off = 정지선 2.5초 서고 출발·비전 안 켬
    'inflate': 0.30,          # ego 벽 여유(m) — 실행 중 바꿈(junsu ego)
    'inflate_cone': 0.20,     # 콘 구간 벽 여유(러너가 rabain~rabaout 에서)
    'max_vel': 0.5,           # ego 최고 속도(m/s) — 실행 중 바꿈
    'green_confirm': 1.0,     # 녹색 몇 초 연속이면 출발
    'green_timeout': 10.0,    # 녹색을 이 안에 못 보면 신호 무관 출발
    'cones': 'cam',           # 콘 감지: cam(기본, 카메라 YOLO+뎁스만) | l1+cam(L1 도) | off(벽만). ★ 11-05 L1 헛장애물 위험으로 cam 기본
    'ego_cloud': '/mission/obstacle_cloud',   # ego 장애물 입력(벽만 쓰려면 /mission/wall_cloud)
    'min_range_xy': 0.12,     # ego 가 로봇 가까이 버리는 거리(원래 0.25 — 벽에 붙으면 벽이 사라짐). 시뮬: 실패 로그 24→19, 콘 경로 45→23
    'grid_res': 0.05,
    'planner': 'review',      # ★ 11-05 사용자: 경로 생성은 merge_tmp review 의 ego(팀원 조정판: /scan 벽 + /mission/cones 콘 여유 0.28, 벽 0.20, 격자 0.05, lifecycle) | junsu(이전)
    'inflate_review': None,
    'wall_penalty': 250.0,    # ★ 11-05 오후 "벽 페널티 강화": review ego optimizer.lambda_obs(팀원 120). 벽 여유(hard 0.20)는 그대로 — 막힘 안 늘게
    'wall_soft': 0.50,        # 벽 밀어내기 시작 거리(soft, 팀원 0.45)
    'yaw_trim': 'on',         # ★ 11-05 직진 쏠림 보정(merge bridge_params 0.110/-0.002). 사용자 요청으로 다시 켬(기본). off = 0 으로 덮어씀(s → T)
    'gait': 'classic',        # ★ 11-05 걸음새: classic(전 구간 ClassicWalk) | legacy(TROTRUN + 정지선·꼭대기 STATICWALK)
    'park': 'down',           # ★ 11-05 주차: down(엎드리기 StandDown→StandUp→BalanceStand) | sit(Sit→RiseSit)
    'loc': 'on',              # ★ 11-05 위치 감시(AMCL+ICP 재정렬)·라이다 콘 추적기 — a 로 같이 켬. off 면 안 켬(러너는 감시 없이 그대로)
    'align': 'on',            # ★ 11-05 출발 정렬(review go2_start_align): r 누르면 서 있는 상태 1초+스캔 3초 → 지도에 맞춰 경로를 오늘 좌표로. ±0.3 m·±10° 밖이면 원래 경로   # review ego 벽 여유 덮어쓰기(None = 팀원 yaml 0.20). 실행 중 못 바꿈 → ego 다시 켬         # ego 격자(m). 시뮬(콘 4개) 0.1: 접촉 11.3초, 0.05: 1.8초(둘 다 완주). ★ 실기에서 ego CPU 확인
}


EGO_REVIEW = os.path.join(J, 'ws_ego_review')   # merge_tmp review/src/core/ego_planner 를 복사해 junsu 에서 빌드(SOURCE.txt 에 커밋)


def ego_spec():
    if OPS.get('planner', 'review') == 'review':
        infl = f" -p inflate_radius:={OPS['inflate_review']}" if OPS.get('inflate_review') else ''
        return ('ego', [f"source {EGO_REVIEW}/install/setup.bash && exec ros2 run ego_planner ego_planner_node --ros-args "
                        f"--params-file {EGO_REVIEW}/src/ego_planner/config/ego_planner_params.yaml "
                        f"-p obstacle_source:=scan -p cones.topic:=/mission/cones -p max_vel:={OPS['max_vel']}{infl} "
                        f"-p optimizer.lambda_obs:={OPS.get('wall_penalty', 250.0)} -p safe_dist:={OPS.get('wall_soft', 0.50)}"],
                'ego_planner_node', None, 0, ['ego_planner_node'])
    return ('ego', [f"exec ros2 run ego_planner ego_planner_node --ros-args -p odom_topic:=/Odometry -p cloud_topic:={OPS['ego_cloud']} "
                    f"-p min_obs_h:=-5.0 -p max_obs_h:=5.0 -p inflate_radius:={OPS['inflate']} -p safe_dist:=0.30 -p max_vel:={OPS['max_vel']} "
                    f"-p min_range_xy:={OPS['min_range_xy']} -p grid_res:={OPS['grid_res']}"],
            'ego_planner_node', None, 0, None)


def loc_spec():
    """★ 11-06 위치 확인(juninsu ②안): 지도 서버 + AMCL(/loc, /tf_loc) + stop_loc_node — 멈춘 곳(출발·정지선·주차)에서만
    AMCL 로 찾고 3D ICP 로 확인해 맞으면 /mission/align_T 로 좌표를 고침. STOP 은 절대 안 냄. 기록 logs/stop_loc.log"""
    return ('loc', [f"exec {J}/juninsu/run_loc.sh"], 'juninsu/run_loc.sh|juninsu/stop_loc_node.py|__ns:=/loc', None, 0, None)


def yaw_trim_vals():
    """직진 쏠림 보정값: tools/yaw_trim_measure.sh 가 저장한 값(maps/segments/logs/yaw_trim.json), 없으면 merge 값 0.110/-0.002."""
    try:
        d = json.load(open(os.path.join(J, 'config', 'yaw_trim.json'))); return float(d['per_vx']), float(d['bias'])   # junk: 11-05 ClassicWalk 실측(junsu f9f989f)
    except (OSError, ValueError, KeyError):
        return 0.110, -0.002


def comp_specs():
    """부품: 이름 → (명령 목록, pkill 패턴, 상태 토픽, 최소 Hz, 켠 뒤 할 일)."""
    rp = f"--ros-args --params-file {BRIDGE_PARAMS}"
    use_l1, use_cam = OPS['cones'] == 'l1+cam', OPS['cones'] in ('cam', 'l1+cam')
    return [
        ('xt16', [f"exec ros2 run hesai_ros_driver hesai_ros_driver_node --ros-args -p config_path:={HESAI_CFG}"],
         'hesai_ros_driver_node', '/lidar_points', 15, None),
        ('bridge', [f"exec ros2 run go2_bridge cloud_bridge_node {rp}", f"exec ros2 run go2_bridge imu_bridge_node {rp}"],
         'cloud_bridge_node|imu_bridge_node', '/imu/body', 200, ['go2_cloud_bridge', 'go2_imu_bridge']),
        ('fastlio', [f"exec ros2 run fast_lio fastlio_mapping --ros-args --params-file {J}/config/fastlio_leg_nurigwan_derange30.yaml"],
         'fastlio_mapping', '/Odometry', 15, None),
        ('vision', [f"{SSH} {ROBOT_PC} 'bash ~/junsu_vision/jetson_vision.sh start' && while :; do sleep 60; done"],   # 루프여야 bash 가 남아 '켜짐'으로 보임
         'junsu_vision/jetson_vision.sh start', '/mission/cone_cam', 2, None),
        ('motor', [f"exec ros2 run go2_bridge cmd_vel_to_sport_node {rp}"
                   + (" -p yaw_trim_per_vx:={:.4f} -p yaw_trim_bias:={:.4f}".format(*yaw_trim_vals()) if OPS.get('yaw_trim', 'on') == 'on'
                      else " -p yaw_trim_per_vx:=0.0 -p yaw_trim_bias:=0.0")],
         'cmd_vel_to_sport_node', None, 0, ['go2_cmd_vel_to_sport']),
        ('runner', None, 'real_course_runner.py', '/mission/runner_status', 1, None),
        loc_spec(),
    ]


class Mon(Node):
    """토픽 빈도(역직렬화 없이 raw 로 셈) + 러너 상태 + 로봇 배터리·모터 온도."""
    TOPICS = {'/lidar_points': PointCloud2, '/imu/body': Imu, '/Odometry': Odometry, '/mission/wall_cloud': PointCloud2,
              '/mission/obstacle_cloud': PointCloud2, '/mission/cone_cam': PointCloud2, '/mission/runner_status': String,
              '/mission/traffic_light_state': String, '/mission/loc_watch': String}

    def __init__(self):
        super().__init__('junsu_go2_ctl')
        be = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.cnt = {t: 0 for t in self.TOPICS}; self.hz = {t: 0.0 for t in self.TOPICS}; self.t0 = time.monotonic()
        for t, typ in self.TOPICS.items():
            if typ is String or t == '/Odometry':
                continue
            self.create_subscription(typ, t, lambda m, t=t: self.bump(t), be, raw=True)
        self.pose = None   # FAST-LIO 위치(x, y, 방향°) — 비상 정지 뒤 이어가기 전에 눈으로 확인
        self.create_subscription(Odometry, '/Odometry', self.cb_odom, be)
        self.runner = {}; self.light = ''; self.loc = {}
        self.create_subscription(String, '/mission/loc_watch', self.cb_loc, 5)
        self.create_subscription(String, '/mission/runner_status', self.cb_runner, 5)
        self.create_subscription(String, '/mission/traffic_light_state', self.cb_light, 5)
        self.robot = {}
        if LowState is not None:
            self.create_subscription(LowState, '/lf/lowstate', self.cb_low, be)
            self.create_subscription(SportModeState, '/lf/sportmodestate', self.cb_sport, be)
            self.pub_req = self.create_publisher(Request, '/api/sport/request', 10)
        self.create_timer(2.0, self.roll)

    def cb_odom(self, m):
        self.bump('/Odometry'); p = m.pose.pose.position; q = m.pose.pose.orientation
        self.pose = (p.x, p.y, math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))

    def bump(self, t):
        self.cnt[t] += 1

    def cb_runner(self, m):
        self.bump('/mission/runner_status')
        try:
            self.runner = json.loads(m.data); self.runner['_t'] = time.monotonic()
        except Exception:
            pass

    def cb_loc(self, m):
        self.bump('/mission/loc_watch')
        try:
            self.loc = json.loads(m.data)
        except ValueError:
            pass

    def cb_light(self, m):
        self.bump('/mission/traffic_light_state'); self.light = m.data

    def cb_low(self, m):
        try:
            self.robot['soc'] = int(m.bms_state.soc)
            self.robot['temp'] = max(int(x.temperature) for x in m.motor_state[:12])
        except Exception:
            pass

    def cb_sport(self, m):
        try:
            self.robot['h'] = float(m.body_height); self.robot['v'] = float((m.velocity[0] ** 2 + m.velocity[1] ** 2) ** 0.5)
            self.robot['mode'] = int(m.mode)
        except Exception:
            pass

    def roll(self):
        dt = time.monotonic() - self.t0; self.t0 = time.monotonic()
        for t in self.cnt:
            self.hz[t] = self.cnt[t] / max(dt, 1e-3); self.cnt[t] = 0

    def stopmove(self):
        if LowState is None:
            return False
        for _ in range(3):
            r = Request(); r.header.identity.api_id = 1003; r.header.identity.id = int(time.time() * 1e6); r.parameter = '{}'
            self.pub_req.publish(r); time.sleep(0.05)
        return True

    def set_ego(self, **kw):
        cli = self.create_client(SetParameters, '/ego_planner_node/set_parameters')
        if not cli.wait_for_service(timeout_sec=1.0):
            return False
        req = SetParameters.Request()
        for k, v in kw.items():
            p = ParameterMsg(); p.name = k; p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(v)); req.parameters.append(p)
        cli.call_async(req); return True


class Ctl:
    def __init__(self, mon):
        self.mon = mon; self.procs = {}; self.msg = '준비'; self.lc = {}; self.last_runner_idx = 0
        self.specs = {s[0]: s for s in comp_specs()}
        os.makedirs(LOG, exist_ok=True)
        try:
            st = json.load(open(STATE)); OPS.update({k: v for k, v in st.get('ops', {}).items() if k != 'route'}); self.last_runner_idx = st.get('last_idx', 0)   # ★ 11-06: 경로는 저장값으로 안 덮음(기본·JUNK_ROUTE 우선 — 저장된 rec1650 이 기본 rec1118x 를 덮던 문제). s→1 선택은 그 실행 동안만
            if OPS.get('cones') == 'on':
                OPS['cones'] = 'cam'      # 예전 저장값(on/off) → 지금 이름
        except Exception:
            pass
        threading.Thread(target=self.lc_loop, daemon=True).start()

    def save(self):
        try:
            json.dump({'ops': OPS, 'last_idx': self.last_runner_idx}, open(STATE, 'w'), ensure_ascii=False, indent=1)
        except Exception:
            pass

    # ── 부품
    def alive(self, name):
        pat = self.specs[name][2]
        if name == 'vision' and self.mon.hz.get('/mission/cone_cam', 0) > 0.5:
            return True   # 내부 PC 노드는 원격 — 콘 토픽이 나오면 켜진 것(이 관리자가 띄우지 않았어도)
        return subprocess.run(['pgrep', '-f', pat], stdout=subprocess.DEVNULL).returncode == 0

    def start(self, name, extra_env=None, runner_args='', route=None):
        self.specs = {s[0]: s for s in comp_specs()}
        spec = self.specs[name]; cmds = spec[1]
        if name == 'runner':
            env = ' '.join(f"RUNNER_{k}={v}" for k, v in (extra_env or {}).items())
            cmds = [f"exec env {env} python3 {J}/src/real_course_runner.py {route or OPS['route']} {runner_args}"]
        self.stop(name, quiet=True)
        logf = open(os.path.join(LOG, f'ctl_{name}.log'), 'ab')
        ps = []
        for c in cmds:
            ps.append(subprocess.Popen(['bash', '-c', c], stdin=subprocess.DEVNULL, stdout=logf, stderr=subprocess.STDOUT, start_new_session=True))
        self.procs[name] = ps
        if spec[5] and name != 'motor':
            time.sleep(2.0)
            for n in spec[5]:
                self.lc_up(n)
        self.msg = f'{name} 켬'

    def stop(self, name, quiet=False):
        for p in self.procs.pop(name, []):
            try:
                os.killpg(p.pid, signal.SIGINT)
            except Exception:
                pass
        time.sleep(0.5)
        if name == 'vision':
            subprocess.run(f"{SSH} {ROBOT_PC} 'bash ~/junsu_vision/jetson_vision.sh stop'", shell=True, timeout=15,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.run(['pkill', '-INT', '-f', self.specs[name][2]], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        for _ in range(6):
            if not self.alive(name):
                break
            time.sleep(0.5)
        if self.alive(name):
            subprocess.run(['pkill', '-TERM', '-f', self.specs[name][2]], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if not quiet:
            self.msg = f'{name} 끔'

    def lc_up(self, node):
        for _ in range(10):
            s = self.lc_get(node)
            if s == 'active':
                return True
            subprocess.run(['ros2', 'lifecycle', 'set', '/' + node, 'configure' if s == 'unconfigured' else 'activate'],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)
            time.sleep(0.5)
        self.msg = f'★ /{node} 활성화 실패'; return False

    def lc_down(self, node):
        subprocess.run(['ros2', 'lifecycle', 'set', '/' + node, 'deactivate'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)

    def lc_get(self, node):
        try:
            out = subprocess.run(['ros2', 'lifecycle', 'get', '/' + node], capture_output=True, text=True, timeout=8).stdout.split()
            return out[0] if out else '없음'
        except Exception:
            return '?'

    def lc_loop(self):
        while True:
            for n in ('go2_cloud_bridge', 'go2_imu_bridge', 'go2_cmd_vel_to_sport'):
                self.lc[n] = self.lc_get(n)
            time.sleep(5)

    # ── 묶음 동작
    def start_all(self):
        order = [('xt16', '/lidar_points', 8), ('bridge', '/imu/body', 8), ('fastlio', '/Odometry', 12)] + ([('vision', None, 0)] if OPS.get('tl', 'off') == 'on' else []) \
            + [('loc', None, 0)]                                                        # ★ 11-06 juninsu 위치 확인(s → J 로 끔)
        if OPS.get('loc', 'on') != 'on':
            order = [o for o in order if o[0] != 'loc']
        for name, topic, wait in order:
            self.msg = f'{name} 켜는 중…'; self.start(name)
            t0 = time.monotonic()
            while topic and time.monotonic() - t0 < wait and self.mon.hz.get(topic, 0) < 1:
                time.sleep(0.5)
            if topic and self.mon.hz.get(topic, 0) < 1:
                self.msg = f'★ {name}: {topic} 가 안 나옴 — 로그(l) 확인. 나머지는 계속 켬'
        self.msg = '전체 켬 — 모터(m)·러너(r)는 따로. ★ 기립 후 30초 안에 출발(뒷다리 과열)'

    def motor_on(self):
        want = (OPS.get('yaw_trim', 'on'), yaw_trim_vals())   # 측정으로 값이 바뀌어도 m 에서 새 값으로 다시 뜸
        if self.alive('motor') and getattr(self, 'motor_trim', None) != want:
            self.stop('motor', quiet=True)        # 다른 쏠림 보정 설정(또는 예전 관리자)으로 뜬 노드 → 지금 설정으로 다시(모터 꺼진 때라 안전)
        if not self.alive('motor'):
            self.start('motor'); self.motor_trim = want
            time.sleep(2.0)
        ok = self.lc_up('go2_cmd_vel_to_sport'); self.msg = '★ 모터 켬 — 이제 로봇이 움직일 수 있음' if ok else self.msg

    def motor_off(self):
        self.mon.stopmove(); self.lc_down('go2_cmd_vel_to_sport'); self.msg = '모터 끔(소프트 정지)'

    def runner_env(self):
        e = {'GREEN_CONFIRM_SECONDS': OPS['green_confirm'], 'STOPLINE_GREEN_TIMEOUT_S': OPS['green_timeout'],
             'STOPLINE_RED_MAX_S': 0.0, 'STOPLINE_HOLD_S': OPS.get('stop_hold', 2.0),   # ★ 11-06 v3: 빨강 보이는 동안은 상한 없이 대기
             'INFLATE_NORMAL': OPS['inflate'], 'INFLATE_CONE': OPS['inflate_cone']}
        e['GAIT_MODE'] = OPS.get('gait', 'classic'); e['PARK_MODE'] = OPS.get('park', 'down')
        e['PATH_ONLY'] = 1; e['PIN_V'] = OPS.get('path_v', 0.30)   # junk: 기록 경로만 따라감(ego 없음)
        e['TL'] = 1 if OPS.get('tl', 'off') == 'on' else 0
        e['HEADING_TANGENT'] = 1 if OPS.get('heading', 'tangent') == 'tangent' else 0
        return e

    def align(self):
        """출발 정렬 — 오늘 좌표 경로 파일을 만든다. 실패·거부면 원래 경로(= 정렬 전과 같음)."""
        out = os.path.join(LOG, 'route_today.json')
        self.msg = '출발 정렬 중 — 로봇을 세운 채 가만히(약 5초)'
        try:
            r = subprocess.run(['python3', os.path.join(J, 'tools', 'start_align', 'go2_start_align.py'), OPS['route'], out,
                                '--map', os.path.join(os.path.dirname(OPS['route']), 'map.yaml')],   # junk: 기록 주행 때 만든 지도(새 콘 포함)
                               capture_output=True, text=True, timeout=40)
            side = json.load(open(out + '.align.json'))
            self.align_info = side
            ok = r.returncode == 0
            self.msg = (f"출발 정렬 {'OK' if ok else '거부(원래 경로)'}: " + json.dumps({k: side[k] for k in side if k in ('T', 'match', 'reason')}, ensure_ascii=False)[:120])
            return out
        except Exception as e:
            self.msg = f'★ 출발 정렬 실패({e}) — 원래 경로로'
            return OPS['route']

    def runner_start(self, start_at=None):
        if self.lc.get('go2_cmd_vel_to_sport') != 'active':
            self.msg = '★ 모터가 꺼져 있음 — m 으로 먼저 켤 것(러너는 켬)'
        if start_at is None:
            self.route_run = self.align() if OPS.get('align', 'on') == 'on' else OPS['route']
        route = getattr(self, 'route_run', None) or OPS['route']   # 이어가기는 처음 출발 때 정렬한 파일 그대로(같은 FAST-LIO 이므로)
        self.start('runner', self.runner_env(), ('' if start_at is None else f'--start-at {start_at}'), route=route)
        if start_at is not None:
            self.msg = f'러너 이어가기: {start_at}번 목표부터 ({os.path.basename(route)})'

    def pause(self):
        st = self.mon.runner
        if st.get('idx') is not None:
            self.last_runner_idx = int(st['idx'])
        self.stop('runner', quiet=True); self.motor_off(); self.save()
        self.msg = f'정비 멈춤 — 마지막 목표 {self.last_runner_idx}번 [{st.get("label", "?")}]. 고친 뒤 R 로 이어가기(FAST-LIO 는 켜 둘 것)'

    def estop(self):
        self.mon.stopmove(); self.lc_down('go2_cmd_vel_to_sport')
        st = self.mon.runner
        if st.get('idx') is not None:
            self.last_runner_idx = int(st['idx'])     # 이어가기(R) 기본값 = 가던 목표
        self.stop('runner', quiet=True); self.mon.stopmove(); self.save()
        self.msg = f'■ 비상 정지 — 모터 끔·StopMove·러너 끔. 마지막 목표 {self.last_runner_idx}번 — m 후 R 엔터로 이어가기'


def route_labels(path):
    try:
        d = json.load(open(path))
        return [e['label'] for e in d['route'][1:]] if isinstance(d, dict) else []
    except Exception:
        return []


def ui(stdscr, ctl, mon):
    curses.curs_set(0); stdscr.nodelay(True); stdscr.timeout(500)
    curses.start_color(); curses.use_default_colors()
    for i, c in enumerate((curses.COLOR_GREEN, curses.COLOR_RED, curses.COLOR_YELLOW, curses.COLOR_CYAN), 1):
        curses.init_pair(i, c, -1)
    names = ['xt16', 'bridge', 'fastlio', 'vision', 'motor', 'runner', 'loc']   # junk: 경로 전용 + 7 = juninsu 위치 확인

    def ask(prompt):
        stdscr.nodelay(False); curses.echo(); curses.curs_set(1)
        h, w = stdscr.getmaxyx(); stdscr.addstr(h - 1, 0, ' ' * (w - 1)); stdscr.addstr(h - 1, 0, prompt[:w - 2])
        s = stdscr.getstr(h - 1, min(len(prompt.encode()), w - 10), 60).decode(errors='ignore').strip()
        curses.noecho(); curses.curs_set(0); stdscr.nodelay(True); return s

    def run_bg(f, *a):
        threading.Thread(target=f, args=a, daemon=True).start()

    # ★ 11-06 사용자 — 작전 끝나면(DONE) 사람이 Q 안 눌러도 잔재(라이다·브리지·FAST-LIO·
    # 카메라·모터·위치확인) 하나도 안 남게 자동으로 다 끔. 예전엔 러너가 DONE 돼도 아무것도
    # 안 끄고, q(소문자, 그냥 나가기)로 종료해도 백그라운드 프로세스는 그대로 살아있었음 —
    # 우리 작전이 실패해서 동료 작전으로 급히 바꿀 때 충돌 위험(겹친 네트워크·DDS 토픽).
    auto_stopped = {'v': False}

    def auto_stop_all():
        auto_stopped['v'] = True
        ctl.estop()
        for n in reversed(names):
            ctl.stop(n, quiet=True)
        ctl.msg = '■ 작전 완료 — 아무도 안 눌러도 자동으로 전부 끔(잔재 없음)'

    while True:
        stdscr.erase(); h, w = stdscr.getmaxyx(); y = 0

        def put(text, attr=0):
            nonlocal y
            if y < h - 1:
                stdscr.addstr(y, 0, text[:w - 1], attr); y += 1
        put(f"junk 경로 전용  {time.strftime('%H:%M:%S')}  도메인 {os.environ.get('ROS_DOMAIN_ID', '0')}  경로 {os.path.basename(OPS['route'])}", curses.A_BOLD)
        r = mon.robot
        put(f"로봇: 배터리 {r.get('soc', '?')}%  모터 최고 {r.get('temp', '?')}°C  속도 {r.get('v', 0):.2f} m/s  몸 높이 {r.get('h', 0):.2f} m  "
            f"신호 {mon.light or '-'}", curses.color_pair(2) if isinstance(r.get('temp'), int) and r['temp'] >= 75 else 0)
        pz = mon.pose
        put(f"위치(FAST-LIO, 지도 좌표): " + (f"x {pz[0]:+.2f}  y {pz[1]:+.2f}  방향 {pz[2]:+.0f}°" if pz else '없음(FAST-LIO 꺼짐)')
            + "   ★ 비상 정지 뒤 이 값이 실제 자리와 맞으면 m → R 엔터로 이어가기. FAST-LIO(3)를 다시 켜면 위치를 잃음")
        lw = mon.loc
        if lw:
            T = lw.get('T') or [0, 0, 0]; ic = lw.get('icp') or {}
            put(f"위치 감시: {lw.get('state')}  AMCL 차이 {lw.get('e_xy')} m / {lw.get('e_yaw_deg')}°  σ {lw.get('sigma')}  "
                f"T {T[0]:+.2f} {T[1]:+.2f} {math.degrees(T[2]):+.1f}°  정렬 {'OK' if lw.get('aligned') else '안 됨'}"
                + (f"  ICP {'OK' if ic.get('ok') else '실패'} {ic.get('reason', '')}" if ic else ''),
                curses.color_pair(2 if lw.get('state') == 'STOP' else (3 if str(lw.get('state')).endswith('WARN') else 0)))
        put('')
        for i, n in enumerate(names, 1):
            spec = ctl.specs[n]; alive = ctl.alive(n); topic, mn = spec[3], spec[4]
            hz = mon.hz.get(topic, 0.0) if topic else None
            ok = alive and (topic is None or hz >= mn)
            extra = ''
            if spec[5]:
                extra = '  ' + ' '.join(f"{x.replace('go2_', '')}={ctl.lc.get(x, '?')}" for x in spec[5])
            hzs = f"{topic} {hz:5.1f} Hz" if topic else ''
            put(f" [{i % 10}] {n:8s} {'켜짐' if alive else '꺼짐':4s}  {hzs:38s}{extra}", curses.color_pair(1 if ok else (3 if alive else 2)))
        put('')
        st = mon.runner; fresh = st and time.monotonic() - st.get('_t', 0) < 3
        if fresh and st.get('state') == 'DONE' and not auto_stopped['v']:
            run_bg(auto_stop_all)
        labs = route_labels(OPS['route'])
        if fresh:
            put(f"러너: {st.get('state')}  목표 {st.get('idx')}/{st.get('n')} [{st.get('label')}]  남은 {st.get('dist')} m  "
                f"{'콘 구간 ' if st.get('cone_zone') else ''}물러남 {st.get('backoffs')}  위치 {st.get('pos')}", curses.color_pair(4))
        else:
            put(f"러너: 안 돎 (마지막 목표 {ctl.last_runner_idx}번{(' [' + labs[ctl.last_runner_idx] + ']') if ctl.last_runner_idx < len(labs) else ''})")
        put(f"작전: 경로 {os.path.basename(os.path.dirname(OPS['route']))}(점 {len(labs)}) · 속도 {OPS.get('path_v', 0.30)} · 신호등 {OPS.get('tl', 'off')} · 위치확인 {OPS.get('loc', 'on')} · 몸방향 {OPS.get('heading', 'tangent')} · 녹색 {OPS['green_confirm']}s · 신호대기 최대 {OPS['green_timeout']}s · "
            f"출발정렬 {OPS.get('align', 'on')} · 쏠림보정 {OPS.get('yaw_trim', 'on')} · 걸음새 {OPS.get('gait', 'classic')} · 주차 {OPS.get('park', 'down')}")
        put('')
        put("a 전체켜기  1~6 다시켜기  m 모터켜기  M 모터끄기  r 러너처음  R 이어가기  p 정비멈춤  x 비상정지  s 작전  l 로그  q 나가기  Q 전부끄기")
        put(f"> {ctl.msg}", curses.color_pair(3))
        stdscr.refresh()
        try:
            k = stdscr.getkey()
        except Exception:
            continue
        if k == 'q':
            ctl.save(); return
        if k == 'Q':
            if ask('전부 끌까요? (y/N) ').lower() == 'y':
                ctl.estop()
                for n in reversed(names):
                    ctl.stop(n, quiet=True)
                ctl.msg = '전부 끔'
        elif k == 'a':
            run_bg(ctl.start_all)
        elif k in '1234567':
            n = names[int(k) - 1]
            if n == 'motor':
                ctl.msg = '모터는 m/M 으로'
            elif n == 'runner':
                ctl.msg = '러너는 r/R 로'
            else:
                run_bg(ctl.start, n)
        elif k == 'm':
            if ask('★ 모터를 켜면 로봇이 움직일 수 있습니다. 켤까요? (y/N) ').lower() == 'y':
                run_bg(ctl.motor_on)
        elif k == 'M':
            run_bg(ctl.motor_off)
        elif k == 'r':
            auto_stopped['v'] = False
            run_bg(ctl.runner_start, None)
        elif k == 'R':
            s = ask(f"이어갈 목표 번호(0~{max(len(labs) - 1, 0)}, 엔터={ctl.last_runner_idx}) {' '.join(f'{i}:{l}' for i, l in enumerate(labs))[:w - 60]} : ")
            n = ctl.last_runner_idx if s == '' else int(s) if s.isdigit() else None
            if n is not None:
                auto_stopped['v'] = False
                run_bg(ctl.runner_start, n)
        elif k == 'p':
            run_bg(ctl.pause)
        elif k in ('x', ' '):
            ctl.estop()
        elif k == 's':
            s = ask('작전 [1]경로(routes) [4]경로 속도 [5]녹색확인s [6]신호대기s [0]출발정렬 on/off [T]쏠림보정 [G]걸음새 [K]주차 down/sit [Y]신호등 on/off [J]위치확인(juninsu) on/off [H]몸방향 경로/목표 : ')
            try:
                if s in ('H', 'h'):
                    OPS['heading'] = 'target' if OPS.get('heading', 'tangent') == 'tangent' else 'tangent'
                    ctl.msg = f"몸 방향 {OPS['heading']} — 러너를 r/R 로 다시 켜야 적용"
                elif s in ('J', 'j'):
                    OPS['loc'] = 'off' if OPS.get('loc', 'on') == 'on' else 'on'
                    if OPS['loc'] == 'on':
                        run_bg(ctl.start, 'loc')
                    else:
                        run_bg(ctl.stop, 'loc')
                    ctl.msg = f"위치 확인(juninsu) {OPS['loc']} — 바로 적용(꺼도 러너는 그대로 감)"
                elif s in ('Y', 'y'):
                    OPS['tl'] = 'on' if OPS.get('tl', 'off') == 'off' else 'off'
                    ctl.msg = f"신호등 {OPS['tl']} — " + ('a 로 비전 켜고 ' if OPS['tl'] == 'on' else '') + '러너를 r 로 다시 켜야 적용'
                elif s == '1':
                    import glob
                    files = sorted(glob.glob(os.path.join(J, 'routes', '*', 'route.json')), key=os.path.getmtime)
                    c = ask(' '.join(f'{i}:{os.path.basename(os.path.dirname(f))}' for i, f in enumerate(files)) + ' → 번호: ')
                    OPS['route'] = files[int(c)]; ctl.msg = f"경로 {os.path.basename(os.path.dirname(OPS['route']))} — 러너를 r 로 다시 켜야 적용"
                elif s == '4':
                    OPS['path_v'] = float(ask('경로 따라가기 속도(m/s, 기본 0.30 · 시뮬 0.33 까지 안전): ')); ctl.msg = f"경로 속도 {OPS['path_v']} — 러너 다시(r/R)"
                elif s == '5':
                    OPS['green_confirm'] = float(ask('녹색 연속 확인(초, 기본 1.0): ')); ctl.msg = '러너를 다시 켜야 적용'
                elif s == '6':
                    OPS['green_timeout'] = float(ask('초록 기다리는 시간(초, 기본 10 — 넘으면 빨강이 안 보일 때 출발): ')); ctl.msg = '러너를 다시 켜야 적용'
                elif s == '0':
                    OPS['align'] = 'off' if OPS.get('align', 'on') == 'on' else 'on'; ctl.msg = f"출발 정렬 {OPS['align']} (r 누를 때 적용)"
                elif s.lower() == 't':
                    OPS['yaw_trim'] = 'off' if OPS.get('yaw_trim', 'on') == 'on' else 'on'
                    if ctl.alive('motor'):
                        ctl.motor_off(); run_bg(ctl.stop, 'motor')
                    ctl.msg = f"직진 쏠림 보정 {OPS['yaw_trim']} — 모터 노드 내림, m 으로 다시 켜야 적용"
                elif s.lower() == 'g':
                    OPS['gait'] = 'legacy' if OPS.get('gait', 'classic') == 'classic' else 'classic'; ctl.msg = f"걸음새 {OPS['gait']} — 러너 다시(r/R)"
                elif s.lower() == 'k':
                    OPS['park'] = 'sit' if OPS.get('park', 'down') == 'down' else 'down'; ctl.msg = f"주차 {OPS['park']} — 러너 다시(r/R)"
                ctl.save()
            except Exception as e:
                ctl.msg = f'작전 바꾸기 실패: {e}'
        elif k == 'l':
            n = ask('로그 볼 부품 이름(xt16/bridge/fastlio/vision/motor/runner): ')
            f = os.path.join(LOG, f'ctl_{n}.log')
            if os.path.exists(f):
                curses.endwin(); subprocess.run(['bash', '-c', f'tail -n 40 {f}; read -p "엔터로 돌아가기" _'])
                stdscr.refresh()
            else:
                ctl.msg = f'로그 없음: {f}'


def main():
    rclpy.init()
    mon = Mon()
    spin_t = threading.Thread(target=rclpy.spin, args=(mon,), daemon=True); spin_t.start()
    ctl = Ctl(mon)

    def bye():   # 수신 스레드가 끝난 뒤 나가야 종료 때 'terminate called' 로 죽지 않는다
        rclpy.try_shutdown(); spin_t.join(2.0)
        try:
            mon.destroy_node()
        except Exception:
            pass
    if '--selftest' in sys.argv:      # 화면 없이: 상태 몇 초 읽고 끝(부품은 안 건드림)
        time.sleep(4.5)
        print('부품:', {n: ctl.alive(n) for n in ctl.specs}); print('Hz:', {k: round(v, 1) for k, v in mon.hz.items()})
        print('lifecycle:', ctl.lc); print('작전:', OPS); print('경로 라벨:', route_labels(OPS['route']))
        bye(); return
    try:
        curses.wrapper(ui, ctl, mon)
    finally:
        bye()


if __name__ == '__main__':
    main()
