#!/usr/bin/env python3
"""실측 좌표(기본 real_waypoints_GOOD_1103.json, 또는 실행 인자로 다른 파일
지정 가능 -- `python3 real_course_runner.py <파일경로>`)를 저장된 순서 그대로
따라가는 러너.

시작 시 자가진단: 첫 /Odometry가 들어오면 저장된 'start' 좌표와 비교한다.
START_CHECK_THRESH(0.5m)보다 많이 떨어져 있으면 — 로봇을 손으로 옮겼거나
FAST-LIO가 잘못 초기화된 경우라 — 목표를 하나도 안 보내고 그대로 멈춘다
(출력만 하고 종료하지 않는 건, 재확인하려고 다시 띄우기 귀찮지 않게 하려는 것뿐 —
Ctrl+C로 끄고 FAST-LIO를 그 자리에서 재시작한 뒤 다시 실행할 것).

start는 목표가 아니라 출발 위치 확인용이라 실제로는 ORDER(현재
stopline1->rabain->rabaout->stopline2->line->parking->exit) 순서대로 /goal_pose를
보낸다. 도착(ARRIVE_THRESH 이내) 확인되면 다음 목표로 넘어가는데, 라벨이
'stopline'으로 시작하면 2.5초 정지 -> 신호 확인(아래) 후 출발, 'rabain'/'rabaout'
(라바콘 슬라럼 진입/탈출)은 정지 없이 통과하면서 /mission/cone_zone_active만
켜고/끔(2026-11-04 추가, 받는 쪽 노드는 아직 미연결이라 지금은 그냥 발행만 함),
parking에서는 Sit(1009) -> 5.5초 앉아있다가 -> RiseSit(1010) 후 계속 진행한다.

★ 2026-11-04: 이전엔 이 러너가 /mission/traffic_light_state를 전혀 구독하지
않고 /mission/camera_active도 안 켜줘서, traffic_light_detector_node(YOLO)가
go2_full_stack.sh로 떠있어도 실제로는 추론 자체를 안 하고(camera_active 기본
False) 아무도 그 결과를 보지도 않는 상태였음 -- 즉 비전 신호인식 전체가 실제
주행과 연결이 안 돼 있었음(STOPLINE_HOLD가 신호 무관하게 2.5초 뒤 그냥 출발).
수정: 정지선 도착 시 camera_active=True로 켜고, 2.5초 정지 뒤
STOPLINE_WAIT_SIGNAL 상태에서 GREEN이 GREEN_CONFIRM_SECONDS(0.5초) 동안
끊기지 않고 들어와야 출발(stopline_mission_node.py와 동일한 디바운스 -- 대회장
외부요인의 단발성 오탐으로 섣불리 출발하는 것 방지). RED/UNKNOWN이면 계속 대기
(안전 쪽). 출발하면 camera_active=False로 꺼서 다음 구간에서 불필요한 추론을
안 함.

장애물 회피는 ego_planner에 맡긴다 — 2026-11-03부터 장애물 입력이 wall_filter_node의
/mission/wall_cloud(벽만 기하학적으로 골라낸 것, 경사로/계단은 이미 안 걸림)라서,
구간별로 높이필터를 따로 완화할 필요가 없어짐(go2_full_stack.sh에서 min/max_obs_h를
넓게 고정). 아래 EGO_STRICT/EGO_RELAXED는 같은 값으로 둬서 과거 구조만 남겨둔 것뿐.

★★ 2026-11-05 밤('작전명 ego'): 사용자가 실제 대회 전략을 구체적으로 설명하며 두 가지를
새로 요구함 — 추가함:
① **신호 대기 타임아웃**: 기존엔 STOPLINE_WAIT_SIGNAL이 GREEN이 뜰 때까지 무한정 기다렸음
   (안전 쪽으로는 맞지만, 내일 YOLO 모델이 아직 없거나 오탐/미탐이 심하면 경기 자체가
   멈춰버림). 사용자가 제안한 두 방어전략(①GREEN 미인식 10초+RED 인식 후 5초 더 대기
   ②그냥 GREEN 미인식 10초면 바로 출발) 중, 사용자 본인이 "구현이 어려우면 ②로"라고
   먼저 허락해서 ②(단순 타임아웃)를 채택함 — ①은 RED를 "진짜 RED"와 "아직 미인식"을
   구분해야 해서 오히려 더 깨지기 쉬움. STOPLINE_GREEN_TIMEOUT_S(10초) 동안 GREEN
   디바운스가 안 끝나면 신호와 무관하게 그냥 출발.
② **도착 후 제자리 회전(ROTATE)**: rabain/rabaout/parking 도착 시(및 parking 바로 전
   점) "저장된 각도"로 제자리 회전 후에 다음 구간으로 간다 — 사용자 설명: 원래 현장에서
   각 지점마다 조종기로 로봇을 돌려서 다음 진행 방향을 맞춘 뒤 지점을 저장했으므로,
   "그 지점에서 다음 목표를 향하는 방향"이 곧 저장된 각도임(새 좌표 저장 불필요,
   goals[idx]→goals[idx+1] 방위각으로 계산). ★ 사용자가 명시적으로 경고한 것: "이동하면서
   돌면 안 된다"(장애물/벽에 부딪힐 확률 증가) — 그래서 반드시 ①위치 도착 ②속도 0 근처로
   정지 확인(기존 STOP_VEL_THRESH 패턴 재사용) 뒤에만 회전을 시작하고, 회전 중에는
   선속도 없이 순수 각속도만 명령한다. 회전은 ego_planner를 거치지 않고 /cmd_vel을
   직접 발행(go2_bridge/cmd_vel_to_sport_node가 그대로 받아 Move(1008) {x:0,y:0,z:wz}로
   변환 — sim/run_sim.sh도 이 노드를 그대로 띄우므로 시뮬레이션에서도 동일하게 동작).
   목표 도착 시 이미 ego_planner의 목표 추적이 끝난 상태(속도 정지 확인됨)라 cmd_vel
   발행자 충돌은 없음 — STOPLINE_HOLD/PARK_SIT_WAIT이 이미 같은 가정에 의존하고 있었음.
   rabain/rabaout은 예전엔 "정지 없이 통과"였는데 이번에 "정지 후 회전 후 통과"로
   동작이 바뀜(사용자가 명시적으로 요구 — 콘 진입/탈출 때 비스듬하면 부딪힐 위험).
   parking 바로 앞 점(라벨이 뭐든 goals에서 parking 바로 앞)도 회전 대상에 자동으로
   포함시킴 — route_map.json처럼 중간 통과점이 많은 경로 파일도 그대로 지원하려고
   라벨 하드코딩 대신 "parking 인덱스 - 1"로 찾음.
"""
import sys
import json
import math
import os
import subprocess
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from unitree_api.msg import Request
from unitree_go.msg import SportModeState
from std_msgs.msg import String, Bool
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue, ParameterType
from sensor_msgs.msg import PointCloud2
import numpy as np

WAYPOINTS_PATH = (sys.argv[1] if len(sys.argv) > 1
                  else os.path.expanduser('~/ws/Rgosu/maps/real_waypoints_GOOD_1103.json'))
# ★ 2026-11-05 아침('작전명 ego-plus'): merge_tmp_20261102/review의 AMCL+ego_planner 스택과
# 묶어 쓸 때만 GOAL_FRAME=map으로 바꿔서 실행 -- 그 스택은 /goal_pose를 'map' 프레임으로
# 받게 설계돼 있고(go2_goto.py 참고), FAST-LIO의 camera_init TF는 일부러 /tf_lio로
# 분리해놔서 메인 TF 트리엔 camera_init이 없음(그 스택 go2_stack.launch.py 설명 참고) --
# 'camera_init' 그대로 보내면 목표 프레임 변환이 안 될 수 있음. 기본값은 기존 그대로
# 'camera_init'(지금까지 검증된 작전명 ego/noego는 안 건드림). arena1459_amcl.yaml이
# 오늘 밤 저장한 waypoints와 같은 14:59 녹화에서 만들어졌으니 좌표값 자체는 그대로 써도
# 될 가능성이 높지만(같은 물리적 원점이라는 가정 -- 우리 waypoints 전체가 이미 의존하고
# 있는 가정과 동일), 로봇으로 직접 검증은 못 함(다른 세션이 로봇을 쓰고 있어서) --
# 처음 쓸 때 반드시 --no-motor로 먼저 확인할 것(tools/go2_full_stack_amcl.sh 참고).
GOAL_FRAME = os.environ.get('GOAL_FRAME', 'camera_init')
# 2026-11-04: 체크리스트(README "경기장 도착 당일")엔 이미 "인자로 웨이포인트 파일
# 지정 가능"이라고 적혀있었는데, 실제 코드엔 그 오버라이드가 빠져있었음(항상 누리관
# 고정 파일만 읽음) -- 이번에 맞춰서 실제로 인자를 받게 고침.
# 2026-11-04: 누리관 전용 순서(아래, 참고용으로 보존 -- 지우지 않음)에서
# 실제 대회 코스 기준 순서로 교체. 'rabain'/'rabaout'은 라바콘 슬라럼 진입/탈출
# 지점 -- cone_raw_filter_node.py 연결용 /mission/cone_zone_active 게이팅에 씀
# (아직 go2_full_stack.sh엔 그 노드가 연결 안 돼 있음, "라바콘이 wall_filter를
# 통과 못 할 수 있음" 섹션 참고 -- 이 라벨들은 토픽 발행 자체는 하지만 그 뒤
# 파이프라인이 아직 안 받고 있어도 무해함).
# 옛 순서(누리관 전용, 참고용): ['line1', 'stopline1', 'top1', 'line2', 'stopline2',
#   'top2', 'line3', 'stopline3', 'top3', 'line4', 'parking', 'exit']
ORDER = ['stopline1', 'rabain', 'rabaout', 'stopline2', 'line', 'parking', 'exit']
ARRIVE_THRESH = 0.4   # m
# ★ 2026-11-04 밤 목표 보정(goal snap): 저장 좌표가 벽에 너무 붙어 있으면(사람이 조종하며 벽 옆에서 찍음 —
# rabarin 10 cm, park 22 cm) ego_planner 가 벽 여유(inflate_radius) 안의 목표로는 경로를 못 만든다
# (시뮬레이션: 여유 0.35 에서 rabain 에 영영 못 감). 그래서 목표를 보낼 때 /mission/wall_cloud 로 쌓은 벽에서
# SNAP_CLEARANCE 이상 떨어진 가장 가까운 점으로 옮긴다. 정지선은 앞다리가 표식 안이어야 하므로 10 cm 까지만.
# 도착 판정도 옮긴 점 기준. False 로 두면 예전과 똑같이 동작한다.
GOAL_SNAP = True
SNAP_CLEARANCE = 0.40          # m — ego_planner inflate_radius(0.35) + 5 cm
SNAP_MAX = {'stopline': 0.10}  # 라벨 앞부분 → 최대 이동 거리(m). 나머지는 SNAP_MAX_DEFAULT
SNAP_MAX_DEFAULT = 0.35
STOP_VEL_THRESH = 0.05   # m/s -- 이 아래면 "정지"로 판정 (stopline_mission_node.py/
# track_sequencer_node.py와 동일 기준). 2026-11-04: 정지선/주차 둘 다 위치(ARRIVE_THRESH)
# 도착만으로 바로 홀드/Sit을 시작했었는데, 속도가 아직 안 죽은 상태에서 시작하면
# (1) 정지선: "2초 이상 정지" 규정을 사실 못 채운 채로 타이머만 돌아가는 꼴
# (2) 주차: 빠른 속도로 Sit -> 전도(-30점) 위험. 그래서 "위치 도착 + 속도 충분히
# 낮음"을 둘 다 만족해야 홀드/Sit을 시작하도록 맞춤.
GOAL_RESEND_INTERVAL = 1.0
START_CHECK_THRESH = 0.5   # m — 이보다 멀면 로봇이 옮겨졌다고 보고 멈춤
SIT_ID = 1009
RISESIT_ID = 1010
ROBOT_SPORT_API_ID_STATICWALK = 1061  # track_sequencer_node.py와 동일, 공식
ROBOT_SPORT_API_ID_TROTRUN = 1062     # unitree_ros2 ros2_sport_client.h로 확인됨
# 2026-11-04: track_sequencer_node.py는 'top'(계단/경사로 정상) 구간으로 향할 때
# STATICWALK로 전환하고 빠져나오면 TROTRUN으로 되돌리는 gait_switch 기능이 있는데,
# real_course_runner.py엔 이게 아예 없었음 -- 계단/경사를 기본 걸음걸이(TROTRUN)로
# 그대로 올라갈 뻔했음. 이 프로젝트 초기 계획표에도 "STATICWALK 자동전환"이
# 2순위로 올라있던 핵심 항목이라 빠뜨리면 안 됨(전도 위험).
SIT_HEIGHT_THRESH = 0.15  # stopline_mission_node.py/parking_mission_node.py와 동일
# 추정치(미보정) -- body_height가 이 아래로 내려가면 "실제로 앉았다"로 판정.
# 2026-11-04: 전엔 Sit 보내고 그냥 SIT_SETTLE_S(1.5초)만 기다리면 앉았다고
# "가정"했는데, 실제로 앉았는지 확인을 안 했음 -- Sit이 씹히거나 늦게 반영돼도
# 모르고 그대로 5.5초를 센 뒤 RiseSit을 보내버리는 구조였음(규정은 "실제로
# 앉아서 5초+" 라서 사실과 다른 걸 기록할 위험). track_sequencer_node.py/
# parking_mission_node.py와 동일하게 body_height로 실제 착석을 확인하도록 맞춤.
SIT_SETTLE_S = 1.5
STOPLINE_HOLD_S = 2.5
GREEN_CONFIRM_SECONDS = 1.0  # ★ 11-05: 0.5→1.0. 경기장 녹화(15:09 빨강 고정 63초, 10 Hz)에서 부스 조명 가짜 GREEN 최장 0.4초 연속 — 0.5 는 여유 0.1초뿐. 진짜 녹색 뒤 출발 0.8~0.9초. GREEN이
# 끊기지 않고 이만큼 연속으로 들어와야 출발. 대회장 외부요인(지나가는 사람/조명)의
# 단발성 오탐으로 섣불리 출발하는 걸 막는다.
PARK_HOLD_S = 5.5
RISE_SETTLE_S = 2.0

# ★ 11-05 밤('작전명 ego') — 신호 타임아웃: GREEN 디바운스가 이 안에 안 끝나면
# 신호 상태(RED/UNKNOWN 무관) 그냥 출발. 사용자가 직접 승인한 단순화된 방어전략.
STOPLINE_GREEN_TIMEOUT_S = 10.0

# ★ 11-05 밤 — 도착 후 제자리 회전(ROTATE). 장애물 가까운 구간(콘/주차)이라 보수적으로 느리게.
ROTATE_LABELS = {'rabain', 'rabaout', 'parking'}  # + parking 바로 앞 점(아래서 동적으로 추가)
ROTATE_RATE_MAX = 0.5      # rad/s — cmd_vel_to_sport max_wz(1.5)보다 한참 보수적
ROTATE_RATE_MIN = 0.15     # rad/s — 이보다 느리면 정지마찰로 거의 안 돎(추정치, 실기 보정 필요)
ROTATE_KP = 1.2
ROTATE_DONE_THRESH = 0.07  # rad (~4도) 이내로 들어오면 "다 돈 것"
ROTATE_SETTLE_S = 0.3      # 그 상태로 이만큼 유지돼야 확정(흔들림 방지, STOP_VEL_THRESH와 같은 패턴)
ROTATE_TIMEOUT_S = 10.0    # 이 안에 못 끝내면 포기하고 그냥 계속 진행(회전 센서 이상 등 — 정지보다 진행이 안전 쪽)

EGO_STRICT = {'min_obs_h': -5.0, 'max_obs_h': 5.0}
EGO_RELAXED = {'min_obs_h': -5.0, 'max_obs_h': 5.0}

# ★ 11-05 — 벽·콘 붙음 복구(BACKOFF). 시뮬레이션에서 확인한 실패: 로봇이 벽 0.10 m 까지 붙으면 ego 의 벽 여유(0.30) 안에서
# 출발점부터 충돌로 판정 → 경로를 영영 못 찾고 그 자리에 멈춤(접촉 감점이 계속 쌓임). ego 는 실패하면 /cmd_vel 을 안 내므로
# (IDLE) 그동안 러너가 /cmd_vel 로 장애물 반대쪽으로 잠깐 물러난 뒤 목표를 다시 보낸다(ROTATE 와 같은 경로).
STUCK_S = 4.0              # 이 시간 동안
STUCK_MOVE = 0.05          # 이만큼(m)도 못 움직이면 붙은 것으로 본다
STUCK_NEAR = 0.45          # 이 안(m)에 장애물이 있을 때만 물러남(없으면 ego 가 생각 중일 수 있음)
BACKOFF_V = 0.15           # m/s — cmd_vel_to_sport min_vx/min_vy(0.08) 보다 크게
BACKOFF_S = 1.3            # 초 → 약 0.2 m
BACKOFF_GAP_S = 3.0        # 물러난 뒤 이만큼은 다시 안 물러남(다시 계획할 시간)
SKIP_VIA_AFTER = 4         # line* 경유점에서 이만큼 물러나도 못 가면 그 경유점은 건너뜀(임무 점은 계속 시도)
OBSTACLE_TOPIC = '/mission/obstacle_cloud'   # 벽+콘(obstacle_fusion_node). 안 오면 벽(wall_cloud)만 본다


def quat_to_yaw(x, y, z, w):
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def ang_diff(target, cur):
    """target - cur을 [-pi, pi]로 감아서 반환."""
    return float((target - cur + np.pi) % (2 * np.pi) - np.pi)


ROUTE_KINDS = ('start', 'stopline', 'top', 'line', 'rabain', 'rabaout', 'parking', 'exit')


def load_waypoints():
    with open(WAYPOINTS_PATH) as f:
        data = json.load(f)
    if isinstance(data, dict) and 'route' in data:
        # ★ 11-05: 지도 위에서 정한 경로(tools/route_editor.py → maps/route_map.json) — 파일에 적힌 순서 그대로 간다.
        #   라벨 앞부분으로 동작이 정해진다(tick 과 같음): stopline* 정지+신호 · top* 느린 걸음 · rabain/rabaout 콘 구간 ·
        #   parking 앉기 · exit 끝 · 그 밖(line*) 통과.
        route = data['route']
        if not route or route[0]['label'] != 'start':
            raise RuntimeError("경로 파일 첫 점은 'start' 여야 합니다")
        if route[-1]['label'] != 'exit':
            raise RuntimeError("경로 파일 마지막 점은 'exit' 여야 합니다")
        bad = [e['label'] for e in route if not e['label'].startswith(ROUTE_KINDS)]
        if bad:
            raise RuntimeError(f'경로 파일에 모르는 라벨: {bad} (쓸 수 있는 앞부분: {ROUTE_KINDS})')
        return (route[0]['x'], route[0]['y']), [(e['label'], (e['x'], e['y'])) for e in route[1:]]
    last = {}
    for e in data:
        last[e['label']] = (e['x'], e['y'])
    if 'start' not in last:
        raise RuntimeError("real_waypoints 파일에 'start' 라벨이 없습니다")
    missing = [label for label in ORDER if label not in last]
    if missing:
        raise RuntimeError(f'real_waypoints 파일에 없는 라벨: {missing}')
    goals = [(label, last[label]) for label in ORDER]
    return last['start'], goals


class RealCourseRunner(Node):
    def __init__(self, start_xy, goals):
        super().__init__('real_course_runner')
        self.start_xy = start_xy
        self.goals = goals
        self.idx = 0
        self.pos = None
        self.yaw = None
        self.state = 'START_CHECK'
        self.state_t = 0.0
        self.wait_signal_t = 0.0   # STOPLINE_WAIT_SIGNAL 진입 시각(타임아웃 기준점)
        self.goal_last_sent = 0.0
        self.last_ego_params_sent = None
        self.last_gait_sent = None
        self.light_state = 'UNKNOWN'
        self.green_since_t = None  # GREEN이 끊기지 않고 시작된 시각. 끊기면 None.
        self.velocity = None
        self.body_height = None
        # ★ 11-05 — rabain/rabaout/parking + parking 바로 앞 점은 도착 후 제자리 회전.
        # 라벨 하드코딩 대신 인덱스로 찾아서 route_map.json처럼 중간점이 많은 파일도 지원.
        self.rotate_idx = {i for i, (label, _) in enumerate(self.goals) if label in ROTATE_LABELS}
        for i, (label, _) in enumerate(self.goals):
            # parking 바로 앞 점도 회전 대상 -- 단, 그 점이 stopline이면 제외(정지선
            # 자체 흐름(정지+신호확인)과 충돌 -- 테스트에서 실제로 걸려서 발견함, 둘 다
            # "도착 직후 특수 동작"이라 동시에 적용할 수 없음. 실제 route_map.json에선
            # parking 바로 앞이 stopline이 아니라(line6) 해당 없음).
            if label == 'parking' and i > 0 and not self.goals[i - 1][0].startswith('stopline'):
                self.rotate_idx.add(i - 1)
        self.rotate_target_yaw = None
        self.rotate_start_t = 0.0
        self.rotate_ok_since = None
        self.wall_cells = set()      # 목표 보정용 — 지금까지 본 벽(5 cm 칸)
        self.wall_tree = None; self.wall_tree_t = 0.0; self.wall_tree_n = 0
        self.snapped = {}            # idx → (x, y) 보정된 목표
        self.create_subscription(PointCloud2, '/mission/wall_cloud', self.cb_wall, 5)
        self.create_subscription(PointCloud2, OBSTACLE_TOPIC, self.cb_obstacle, 2)
        self.obs_xy = None; self.obs_t = 0.0            # 가까운 장애물(벽+콘) 최신 — BACKOFF 방향 계산용
        self.prog = []                                  # (시각, x, y) — 붙음 판정용
        self.backoff_until = 0.0; self.backoff_cmd = None; self.last_backoff_t = -1e9; self.backoffs_here = 0
        self.create_subscription(Odometry, '/Odometry', self.cb_odom, 10)
        self.create_subscription(SportModeState, '/sportmodestate', self.cb_state, 10)
        self.create_subscription(String, '/mission/traffic_light_state', self.cb_light, 10)
        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.pub_req = self.create_publisher(Request, '/api/sport/request', 10)
        self.pub_camera_active = self.create_publisher(Bool, '/mission/camera_active', 10)
        self.pub_cone_zone_active = self.create_publisher(Bool, '/mission/cone_zone_active', 10)
        # ★ 11-05 — ROTATE 상태 전용. ego_planner를 거치지 않고 go2_bridge/cmd_vel_to_sport_node로
        # 직접 보냄(그 노드가 /cmd_vel -> Move(1008) 변환, sim/run_sim.sh도 그 노드를 그대로 띄움).
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        # 2026-11-04: 'rabain'(라바콘 슬라럼 진입)/'rabaout'(탈출) 도착 시 이 토픽을
        # True/False로 켜줌 -- cone_raw_filter_node.py(아직 go2_full_stack.sh에 미연결,
        # "라바콘이 wall_filter를 통과 못 할 수 있음" 섹션 참고)가 구독해서 그 구간에서만
        # 원시 라이다 높이밴드를 보강하게 하는 용도. 지금은 받는 쪽이 없어도 그냥
        # 발행만 하는 거라 무해함 -- 나중에 그 노드를 연결하면 바로 쓸 수 있게 미리 켬.
        self.ego_param_client = self.create_client(SetParameters, '/ego_planner_node/set_parameters')
        self.create_timer(0.2, self.tick)
        self.get_logger().info(f'실측 코스 러너 시작 — {len(goals)}개 목표: {[g[0] for g in goals]}')

    def cb_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.pos = (p.x, p.y)
        q = msg.pose.pose.orientation
        self.yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

    def cb_state(self, msg: SportModeState):
        vx, vy, vz = msg.velocity
        self.velocity = (vx * vx + vy * vy) ** 0.5
        self.body_height = msg.body_height

    def cb_light(self, msg: String):
        new_state = msg.data.split('|')[0] if msg.data else 'UNKNOWN'
        if new_state == 'GREEN':
            if self.light_state != 'GREEN':
                self.green_since_t = time.monotonic()  # GREEN 연속 구간 새로 시작
        else:
            self.green_since_t = None  # 끊겼으니 디바운스 카운터 리셋
        self.light_state = new_state

    def set_camera_active(self, active: bool):
        self.pub_camera_active.publish(Bool(data=active))

    def set_cone_zone_active(self, active: bool):
        self.pub_cone_zone_active.publish(Bool(data=active))

    def cur(self):
        return self.goals[self.idx]

    # ---- 목표 보정(GOAL_SNAP)
    def cb_wall(self, msg: PointCloud2):
        if not GOAL_SNAP or msg.width * msg.height == 0:
            return
        a = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(-1, msg.point_step)
        off = {f.name: f.offset for f in msg.fields}
        xy = np.stack([a[:, off['x']:off['x'] + 4].copy().view(np.float32)[:, 0], a[:, off['y']:off['y'] + 4].copy().view(np.float32)[:, 0]], 1)
        xy = xy[np.isfinite(xy).all(1)]
        self.wall_cells.update(map(tuple, np.floor(xy / 0.05).astype(int).tolist()))

    # ---- 벽·콘 붙음 복구(BACKOFF)
    def cb_obstacle(self, msg: PointCloud2):
        if self.pos is None or msg.width * msg.height == 0:
            return
        a = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(-1, msg.point_step)
        off = {f.name: f.offset for f in msg.fields}
        xy = np.stack([a[:, off['x']:off['x'] + 4].copy().view(np.float32)[:, 0], a[:, off['y']:off['y'] + 4].copy().view(np.float32)[:, 0]], 1)
        xy = xy[np.isfinite(xy).all(1)]
        self.obs_xy = xy[np.hypot(xy[:, 0] - self.pos[0], xy[:, 1] - self.pos[1]) < 1.0]; self.obs_t = time.monotonic()

    def near_obstacles(self):
        """로봇 STUCK_NEAR 안 장애물 점(벽+콘). obstacle_cloud 가 1초 넘게 안 왔으면 벽 칸(wall_cells)으로."""
        if self.pos is None:
            return np.zeros((0, 2))
        if self.obs_xy is not None and time.monotonic() - self.obs_t < 1.0:
            P = self.obs_xy
        else:
            T = self._tree()
            if T is None:
                return np.zeros((0, 2))
            P = T.data[T.query_ball_point(self.pos, STUCK_NEAR)]
        return P[np.hypot(P[:, 0] - self.pos[0], P[:, 1] - self.pos[1]) < STUCK_NEAR] if len(P) else P

    def check_stuck(self, now):
        """TO_GOAL 중 STUCK_S 동안 STUCK_MOVE 도 못 움직였고 장애물이 가까우면 BACKOFF 시작. 시작했으면 True."""
        if self.pos is None:
            return False
        self.prog.append((now, self.pos[0], self.pos[1]))
        self.prog = [p for p in self.prog if now - p[0] <= STUCK_S + 0.5]
        if now - self.prog[0][0] < STUCK_S or now - self.last_backoff_t < BACKOFF_GAP_S or self.dist_to_cur() < ARRIVE_THRESH:
            return False
        if max(math.hypot(x - self.pos[0], y - self.pos[1]) for _, x, y in self.prog) >= STUCK_MOVE:
            return False
        label = self.cur()[0]
        P = self.near_obstacles()
        if not len(P):
            return False
        if label.startswith('line') and self.backoffs_here >= SKIP_VIA_AFTER:
            self.get_logger().error(f'[{label}] 물러나기 {self.backoffs_here}번에도 못 감 — 경유점 건너뜀')
            self.prog = []; self.advance(); return True
        d = np.array(self.pos) - P; w = 1.0 / np.maximum(np.einsum('ij,ij->i', d, d), 1e-4)
        v = (d * w[:, None]).sum(0); n = float(np.hypot(*v))
        if n < 1e-6:
            return False
        v /= n; c, s = math.cos(self.yaw or 0.0), math.sin(self.yaw or 0.0)
        bx, by = c * v[0] + s * v[1], -s * v[0] + c * v[1]     # 세계 → 몸체
        cmd = Twist(); cmd.linear.x = BACKOFF_V * bx; cmd.linear.y = BACKOFF_V * by
        self.backoff_cmd = cmd; self.backoff_until = now + BACKOFF_S; self.last_backoff_t = now; self.backoffs_here += 1
        self.state = 'BACKOFF'; self.pub_cmd_vel.publish(cmd)   # 첫 명령은 바로
        self.get_logger().warn(f'[{label}] {STUCK_S:.0f}초간 제자리 + 장애물 {float(np.min(np.hypot(*d.T))):.2f} m — '
                               f'반대쪽으로 {BACKOFF_V * BACKOFF_S:.2f} m 물러남(몸체 vx {cmd.linear.x:+.2f} vy {cmd.linear.y:+.2f}, {self.backoffs_here}번째)')
        return True

    def _tree(self):
        now = time.monotonic()
        if self.wall_cells and (self.wall_tree is None or (now - self.wall_tree_t > 1.0 and len(self.wall_cells) != self.wall_tree_n)):
            from scipy.spatial import cKDTree
            self.wall_tree = cKDTree((np.array(list(self.wall_cells)) + 0.5) * 0.05); self.wall_tree_t = now; self.wall_tree_n = len(self.wall_cells)
        return self.wall_tree

    def target(self):
        """지금 목표의 실제로 보낼 좌표(보정 반영). 벽 정보가 없거나 GOAL_SNAP=False 면 저장 좌표 그대로."""
        label, (x, y) = self.cur()
        T = self._tree() if GOAL_SNAP else None
        if T is None:
            return x, y
        c0 = T.query([x, y])[0]
        if c0 >= SNAP_CLEARANCE:
            self.snapped.pop(self.idx, None); return x, y
        rmax = next((v for k, v in SNAP_MAX.items() if label.startswith(k)), SNAP_MAX_DEFAULT)
        g = np.arange(-rmax, rmax + 1e-9, 0.05); gx, gy = np.meshgrid(g, g); m = np.hypot(gx, gy) <= rmax
        cand = np.c_[x + gx[m], y + gy[m]]; c = T.query(cand)[0]; off = np.hypot(cand[:, 0] - x, cand[:, 1] - y)
        ok = c >= SNAP_CLEARANCE
        i = int(np.argmin(np.where(ok, off, np.inf))) if ok.any() else int(np.argmax(c))
        if not ok.any() and c[i] <= c0:
            return x, y
        nx, ny = float(cand[i, 0]), float(cand[i, 1])
        if self.snapped.get(self.idx) is None or np.hypot(nx - self.snapped[self.idx][0], ny - self.snapped[self.idx][1]) > 0.02:
            self.get_logger().warn(f'[{label}] 목표 보정: 벽까지 {c0:.2f} m → ({nx:.2f},{ny:.2f}) 로 {off[i]:.2f} m 옮김 (벽까지 {c[i]:.2f} m)')
        self.snapped[self.idx] = (nx, ny)
        return nx, ny

    def dist(self, x, y):
        if self.pos is None:
            return 999.0
        return ((self.pos[0] - x) ** 2 + (self.pos[1] - y) ** 2) ** 0.5

    def dist_to_cur(self):
        x, y = self.target()   # 보정된 목표 기준(보정이 없으면 저장 좌표)
        return self.dist(x, y)

    def send_goal(self):
        label, _ = self.cur()
        x, y = self.target()
        msg = PoseStamped()
        msg.header.frame_id = GOAL_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.w = 1.0
        self.pub_goal.publish(msg)
        self.goal_last_sent = time.monotonic()
        self.get_logger().warn(f'[{label}] 목표 발행 -> ({x:.2f}, {y:.2f})')

    def send_sport(self, api_id, name):
        req = Request()
        # ★ 2026-11-04: go2_bridge/cmd_vel_to_sport_node.cpp(실제 로봇 구동 검증됨)와
        # 팀 review 브랜치 전체(mission_fsm_node.cpp, route_runner_node.cpp,
        # odom_nav_node.cpp)가 공통으로 identity.id + parameter를 채워서 보낸다 --
        # api_id만 보내면 로봇 쪽에서 조용히 무시될 위험이 있어 같은 패턴으로 맞춤.
        req.header.identity.id = self.get_clock().now().nanoseconds
        req.header.identity.api_id = api_id
        req.parameter = '{}'
        self.pub_req.publish(req)
        self.get_logger().warn(f'명령 전송: {name}({api_id})')

    def set_gait(self, want_staticwalk: bool):
        api_id = ROBOT_SPORT_API_ID_STATICWALK if want_staticwalk else ROBOT_SPORT_API_ID_TROTRUN
        if api_id == self.last_gait_sent:
            return
        self.send_sport(api_id, 'STATICWALK' if want_staticwalk else 'TROTRUN')
        self.last_gait_sent = api_id

    def set_ego_params(self, params):
        if params == self.last_ego_params_sent:
            return
        if not self.ego_param_client.service_is_ready():
            self.get_logger().warn('ego_planner set_parameters 서비스 아직 준비 안 됨 — 건너뜀')
            return
        req = SetParameters.Request()
        for k, v in params.items():
            p = ParameterMsg()
            p.name = k
            p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(v))
            req.parameters.append(p)
        self.ego_param_client.call_async(req)
        self.last_ego_params_sent = params
        self.get_logger().warn(f'ego_planner 장애물 필터 전환 -> {params}')

    def advance(self):
        self.idx += 1
        self.backoffs_here = 0; self.prog = []
        if self.idx >= len(self.goals):
            self.get_logger().warn('exit까지 전부 도착 — 종료')
            self.state = 'DONE'
            return
        self.state = 'TO_GOAL'
        self.goal_last_sent = 0.0

    def dispatch_arrival(self):
        """도착(+필요시 회전까지) 끝난 뒤 라벨별 동작. TO_GOAL에서 회전이 필요 없으면
        바로, ROTATE에서 회전이 끝나면 여기로 들어온다."""
        label, _ = self.cur()
        if label == 'parking':
            self.send_sport(SIT_ID, 'Sit')
            self.state = 'PARK_SIT_WAIT'
            self.state_t = time.monotonic()
        elif label.startswith('stopline'):
            self.light_state = 'UNKNOWN'  # 이전 정지선의 묵은 값이 섞이지 않게
            self.green_since_t = None
            self.set_camera_active(True)
            self.get_logger().warn(f'[{label}] 정지선 진입 -> 카메라 ON, 2.5초 정지 시작')
            self.state = 'STOPLINE_HOLD'
            self.state_t = time.monotonic()
        elif label == 'rabain':
            # 라바콘 슬라럼 진입 -- cone_zone_active 켬(회전은 이미 TO_GOAL에서 끝냄)
            self.set_cone_zone_active(True)
            self.get_logger().warn(f'[{label}] 라바콘 슬라럼 진입 -> cone_zone_active ON')
            self.advance()
        elif label == 'rabaout':
            # 라바콘 슬라럼 탈출 -- cone_zone_active 끔
            self.set_cone_zone_active(False)
            self.get_logger().warn(f'[{label}] 라바콘 슬라럼 탈출 -> cone_zone_active OFF')
            self.advance()
        else:
            self.advance()

    def start_rotate(self):
        """현재 점 -> 다음 점 방위각으로 제자리 회전 시작. 다음 점이 없으면(마지막 점) 회전 생략."""
        if self.idx + 1 >= len(self.goals) or self.pos is None:
            self.dispatch_arrival()
            return
        nx, ny = self.goals[self.idx + 1][1]
        self.rotate_target_yaw = float(np.arctan2(ny - self.pos[1], nx - self.pos[0]))
        self.rotate_start_t = time.monotonic()
        self.rotate_ok_since = None
        self.state = 'ROTATE'
        self.get_logger().warn(
            f'[{self.cur()[0]}] 도착 -> 제자리 회전 시작 (목표 {np.degrees(self.rotate_target_yaw):.0f}도)')

    def tick(self):
        now = time.monotonic()
        if self.state == 'START_CHECK':
            if self.pos is None:
                return
            d = self.dist(*self.start_xy)
            if d <= START_CHECK_THRESH:
                self.get_logger().warn(
                    f'자가진단 OK — 현재 ({self.pos[0]:.2f},{self.pos[1]:.2f}), '
                    f'start와 {d:.2f}m 차이. 코스 시작.')
                self.state = 'TO_GOAL'
            else:
                self.get_logger().error(
                    f'자가진단 실패 — 현재 ({self.pos[0]:.2f},{self.pos[1]:.2f})가 '
                    f'start({self.start_xy[0]:.2f},{self.start_xy[1]:.2f})에서 {d:.2f}m 떨어짐 '
                    f'(한계 {START_CHECK_THRESH}m). 로봇이 옮겨졌거나 FAST-LIO가 잘못 '
                    f'초기화된 것으로 보임 — 목표를 보내지 않음. Ctrl+C로 끄고 FAST-LIO를 '
                    f'이 자리에서 재시작한 뒤 다시 실행하세요.')
            return
        if self.state == 'DONE':
            return
        if self.state == 'TO_GOAL':
            label, _ = self.cur()
            # ★ 11-05: 정지선은 규정상 '언덕 및 계단을 올라간 위치' — 정지선으로 가는 동안도 느린 걸음(top 과 같게)
            is_top = label.startswith('top') or label.startswith('stopline')
            self.set_ego_params(EGO_RELAXED if is_top else EGO_STRICT)
            self.set_gait(is_top)
            if now - self.goal_last_sent >= GOAL_RESEND_INTERVAL:
                self.send_goal()
            if self.check_stuck(now):   # ★ 11-05: 벽·콘에 붙어 4초 제자리면 물러남(BACKOFF)
                return
            if self.dist_to_cur() < ARRIVE_THRESH:
                # ★ 11-05: 회전 대상 점도 정지 확인 후에만 다음 단계로 — "이동하면서
                # 돌면 안 된다"(사용자 지시)를 지키려면 회전 시작 전 반드시 완전히 서야 함.
                needs_full_stop = label == 'parking' or label.startswith('stopline') or self.idx in self.rotate_idx
                if needs_full_stop:
                    is_stopped = self.velocity is not None and self.velocity < STOP_VEL_THRESH
                    if not is_stopped:
                        return  # 위치는 도착했지만 속도가 아직 안 죽음 -- 계속 TO_GOAL에서 대기
                self.get_logger().warn(f'[{label}] 도착')
                if self.idx in self.rotate_idx:
                    self.start_rotate()
                else:
                    self.dispatch_arrival()
        elif self.state == 'BACKOFF':
            if now < self.backoff_until:
                self.pub_cmd_vel.publish(self.backoff_cmd)
                return
            self.pub_cmd_vel.publish(Twist())                 # 정지 → 목표 바로 다시 보냄
            self.state = 'TO_GOAL'; self.goal_last_sent = 0.0; self.prog = []
        elif self.state == 'STOPLINE_HOLD':
            # 2026-11-04: 처음엔 STOPLINE_HOLD 진입 시에만 속도를 확인하고, 2.5초
            # 동안은 그냥 시계만 돌렸음 -- 그 사이에 로봇이 밀리거나 다시 움직이면
            # "2초 이상 연속 정지" 규정을 사실 못 채운 채로 통과하는 구멍이 있었음
            # (track_sequencer_node.py는 이미 매 틱 재확인하고 있었음, 거기 맞춤).
            # 유지 중 다시 움직이면 처음부터(TO_GOAL) 다시 시작한다.
            is_stopped = self.velocity is not None and self.velocity < STOP_VEL_THRESH
            if not is_stopped:
                self.get_logger().warn('정지 유지 중 이동 감지 -> 다시 접근 단계로')
                self.state = 'TO_GOAL'
                self.goal_last_sent = 0.0
                return
            if now - self.state_t >= STOPLINE_HOLD_S:
                self.get_logger().info('2.5초 정지 완료 -> 신호 확인 단계')
                self.state = 'STOPLINE_WAIT_SIGNAL'
                self.wait_signal_t = now
        elif self.state == 'STOPLINE_WAIT_SIGNAL':
            # 규정: 녹색이면 주행 가능, 빨간불/UNKNOWN이면 녹색 될 때까지 대기(안전 쪽).
            # GREEN_CONFIRM_SECONDS 동안 끊기지 않고 GREEN이어야 출발 -- 단발성
            # 오탐(지나가는 사람/조명)이면 cb_light에서 green_since_t가 리셋되므로
            # 여기로 못 넘어온다. 2026-11-04: 신호 대기 중에도 로봇이 밀리는 등
            # 다시 움직이면(STOPLINE_HOLD와 동일 기준) 처음부터 다시 접근하게 함.
            # ★ 11-05: STOPLINE_GREEN_TIMEOUT_S 안에 GREEN이 확정 안 되면(내일 YOLO
            # 모델 자체가 없거나 오탐/미탐이 심한 경우 대비) 신호와 무관하게 그냥 출발 —
            # 사용자가 직접 승인한 단순화된 방어전략(①RED 확인 후 5초 추가 대기안보다
            # 더 안 깨짐). 타이밍은 wait_signal_t(이 상태 진입 시각) 기준.
            is_stopped = self.velocity is not None and self.velocity < STOP_VEL_THRESH
            if not is_stopped:
                self.get_logger().warn('신호 대기 중 이동 감지 -> 다시 접근 단계로')
                self.state = 'TO_GOAL'
                self.goal_last_sent = 0.0
                return
            if (self.light_state == 'GREEN' and self.green_since_t is not None
                    and now - self.green_since_t >= GREEN_CONFIRM_SECONDS):
                self.get_logger().warn(
                    f'녹색 신호 {GREEN_CONFIRM_SECONDS}초 연속 확인 -> 출발, 카메라 OFF')
                self.set_camera_active(False)
                self.advance()
            elif now - self.wait_signal_t >= STOPLINE_GREEN_TIMEOUT_S:
                self.get_logger().error(
                    f'신호 확인 {STOPLINE_GREEN_TIMEOUT_S}초 안에 GREEN 미확정'
                    f'(현재 {self.light_state}) -> 타임아웃, 신호 무관하게 출발')
                self.set_camera_active(False)
                self.advance()
        elif self.state == 'ROTATE':
            # ★ 11-05: 선속도 0, 순수 각속도만 — "이동하면서 돌면 안 된다" 지키는 핵심.
            if self.yaw is None:
                return
            err = ang_diff(self.rotate_target_yaw, self.yaw)
            if abs(err) < ROTATE_DONE_THRESH:
                if self.rotate_ok_since is None:
                    self.rotate_ok_since = now
                elif now - self.rotate_ok_since >= ROTATE_SETTLE_S:
                    self.pub_cmd_vel.publish(Twist())  # 정지
                    self.get_logger().warn(f'[{self.cur()[0]}] 회전 완료 -> 다음 동작')
                    self.dispatch_arrival()
                return
            self.rotate_ok_since = None
            if now - self.rotate_start_t > ROTATE_TIMEOUT_S:
                self.get_logger().error(
                    f'[{self.cur()[0]}] 회전 타임아웃({ROTATE_TIMEOUT_S}초, 남은 오차 '
                    f'{np.degrees(err):.0f}도) -> 포기하고 계속 진행')
                self.pub_cmd_vel.publish(Twist())
                self.dispatch_arrival()
                return
            w = float(np.clip(ROTATE_KP * err, -ROTATE_RATE_MAX, ROTATE_RATE_MAX))
            if abs(w) < ROTATE_RATE_MIN:
                w = ROTATE_RATE_MIN * (1.0 if w >= 0 else -1.0)
            cmd = Twist()
            cmd.angular.z = w
            self.pub_cmd_vel.publish(cmd)
        elif self.state == 'PARK_SIT_WAIT':
            # 2026-11-04: SIT_SETTLE_S만큼 그냥 기다리던 것 -> body_height로 실제
            # 착석을 확인(다른 두 구현체와 동일 패턴). SIT_SETTLE_S 넘도록 확인
            # 안 되면 Sit을 재전송(명령이 씹혔을 가능성).
            sat = self.body_height is not None and self.body_height < SIT_HEIGHT_THRESH
            if sat:
                self.get_logger().info(f'착석 확인됨(body_height={self.body_height:.3f}) -> 5.5초 유지 시작')
                self.state = 'PARK_HOLD'
                self.state_t = now
            elif now - self.state_t > SIT_SETTLE_S:
                self.get_logger().warn('Sit 명령 후 착석 미확인 -> 재전송')
                self.send_sport(SIT_ID, 'Sit')
                self.state_t = now
        elif self.state == 'PARK_HOLD':
            # 유지 중 몸통이 다시 올라오면(흔들림/밀림 등) 착석이 풀린 것으로 보고
            # 처음부터(PARK_SIT_WAIT) 다시 확인
            if self.body_height is not None and self.body_height >= SIT_HEIGHT_THRESH:
                self.get_logger().warn('유지 중 이탈 감지(body_height 복귀) -> 다시 착석 확인')
                self.state = 'PARK_SIT_WAIT'
                self.state_t = now
                return
            if now - self.state_t >= PARK_HOLD_S:
                self.send_sport(RISESIT_ID, 'RiseSit')
                self.state = 'PARK_RISE_WAIT'
                self.state_t = now
        elif self.state == 'PARK_RISE_WAIT':
            if now - self.state_t >= RISE_SETTLE_S:
                self.advance()


AUTO_STOP = os.environ.get('AUTO_STOP', '1') == '1'
# ★ 2026-11-05 아침: 사용자 요청 -- "작전(ego/noego/ego-plus) 끝나면 드라이버 전부 자동으로
# 꺼지게" (완주든 Ctrl+C든) -- 동료가 테스트하다 우리가 안 되면 바로 자기 작전으로 넘어가야
# 하는데 우리 쪽 노드가 안 꺼져 있으면 이름이 겹쳐서 충돌함. tools/go2_stop_all.sh를 끝에서
# 자동 호출. AUTO_STOP=0으로 끌 수 있음(끝나고 로그·상태 들여다보고 싶을 때). 단위시험은
# main()을 안 거치고 RealCourseRunner를 직접 생성해서 쓰므로(test_real_course_runner.py 등)
# 영향 없음.
def stop_everything():
    stop_script = os.path.expanduser('~/ws/Rgosu/tools/go2_stop_all.sh')
    if not os.path.exists(stop_script):
        return
    print('[real_course_runner] 작전 종료 -> tools/go2_stop_all.sh로 전체 정리 중...', flush=True)
    try:
        subprocess.run([stop_script], timeout=30)
    except Exception as e:
        print(f'[real_course_runner] 정리 스크립트 실행 실패({e}) -- 수동으로 '
              f'~/ws/Rgosu/tools/go2_stop_all.sh 실행할 것', flush=True)


def main():
    start_xy, goals = load_waypoints()
    rclpy.init()
    node = RealCourseRunner(start_xy, goals)
    try:
        while rclpy.ok() and node.state != 'DONE':
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    if AUTO_STOP:
        stop_everything()


if __name__ == '__main__':
    main()
