#!/usr/bin/env python3
"""상위 주행 로직 — TrackSequencer. 2026 로보월드 트랙(IN→콘슬라럼→정지선1→
경사/계단→정지선2→주차→OUT)을 순서가 정해진 "구간" 목록으로 표현하고, 로봇
오도메트리 위치가 각 구간 영역(zone)에 들어오면 그 구간에 연결된 동작을 실행한다.

왜 이렇게 만들었나 (설계 배경, 2026-11-01):
  - 정지선/주차는 비전(QR, 신호등 색)보다 '위치'가 1차 기준이어야 한다 — 주차 라인이
    아주 타이트하게 그려져 있을 거라 비전 마커 인식만으론 경계 판단이 부정확함.
    (이전엔 QR 접근도를 1차 트리거로 썼는데, 이번에 오돔 우선으로 재설계함)
  - 트랙의 "큰 틀"(전체 구간 순서와 대략적 위치)은 규정집 치수로 이미 알려져 있다.
    처음부터 범용 전역경로 플래너를 돌리기보다, 구간별 웨이포인트 순서를 미리
    정해두고 "그 구간 안에서의" 장애물 회피만 ego_planner(라이다)에게 맡기는 게
    더 안정적이다 (구간 경계는 장애물 위치가 바뀌어도 안 바뀌니까).
  - 좌표(zone)는 전부 PLACEHOLDER. 누리관 연습(계단/경사) + 복도(콘 슬라럼 별도
    연습) + 최종 실제 경기장 bag이 갖춰지면, segments_config.json만 실측값으로
    교체하면 된다 — 이 노드의 상태기 로직 자체는 안 건드려도 됨.

구간 타입(type)별 동작:
  pass_through  : 그냥 통과만 확인하고 바로 다음 구간으로 (시작/끝 등)
  gait_switch   : 진입 시 지정 걸음걸이로 전환(예: STATICWALK), 이탈 시 기본값 복귀
  tune_planner  : 진입 시 ego_planner 파라미터 변경(예: 콘 슬라럼 구간 저속화),
                  이탈 시 기본값 복귀
  stopline      : 진입 후 정지(속도 0 근접) 2.5초 유지 + 신호등 녹색 확인해야 완료
  parking       : 진입 후 Sit 명령 + body_height로 착석 확인 + 5.5초 유지 + RiseSit

구간마다 추가로 두 가지를 매 틱 발행/적용한다 (타입과 무관하게 전 구간 공통):
  camera_active   : true인 구간(정지선)에서만 비전 처리 활성화, 나머진 OFF —
                    /mission/camera_active(Bool) 발행, traffic_light_detector_node가
                    구독해서 OFF면 이미지 콜백을 건너뛴다 (오탐지 방지 + 라이다에
                    연산 자원 몰아주기, 2026-11-01).
  obstacle_filter : "strict"(기본, 라바콘 등 전부 회피 대상으로 엄격 적용) /
                    "relaxed"(계단·경사 구간 — 통행가능 지형을 장애물로 오인하지
                    않게 완화). 콘과 벽은 "둘 다 피해야 할 것"이라 구분 안 함 —
                    구분이 진짜 필요한 건 "회피 대상 vs 통행 가능 지형"이고, 이건
                    이미 아는 트랙 구간 위치로 그냥 토글하면 되니 별도 지형분류기가
                    필요 없다는 결론.

★ 목표지점(/goal_pose) 발행 — 단순화된 설계 (2026-11-02):
  구간마다 세세한 웨이포인트를 주지 않는다. segments_config.json의 'goal' 필드가
  있는 구간(지금은 start, exit 둘뿐)에 진입하는 순간만 /goal_pose를 한 번
  발행한다 — start에서 주차구역으로 목표 1개, 주차(Sit→RiseSit) 끝나고 exit로
  넘어가면서 출구 너머로 목표 1개. 그 사이(콘 슬라럼/정지선/계단) 구간은 목표
  변경 없이 ego_planner가 "이미 받은 목표"를 향해 라이다로 장애물을 알아서
  피하며 계속 주행한다 — 계단/경사 구간은 obstacle_filter=relaxed로 이미
  통행 가능하게 열어놨으니 같은 목표를 향해 가는 도중 자연히 넘어가는 구조.

★ ego_planner 런타임 파라미터 변경 — 실제로 적용됨 (2026-11-01):
  /ego_planner_node/set_parameters 서비스로 min_obs_h/max_obs_h/max_vel을 매 틱
  보낸다(값이 바뀔 때만 실제 전송). 원래 ego_planner_node.cpp는 파라미터를
  생성자에서 한 번만 읽어서 ros2 param set이 전혀 안 먹혔는데,
  ~/ws/Rgosu/ws_ego_planner_tunable 에 add_on_set_parameters_callback 패치를
  추가해서 실제로 반영되게 만들었다(라이브 테스트로 확인함 — 서비스 호출 후
  ego_planner 자체 로그에 "[Rgosu] min_obs_h 런타임 변경 -> ..."가 찍힘).
  ego_planner가 안 떠있으면(서비스 미연결) 조용히 건너뛰고 경고만 남긴다.
"""
import json
import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from unitree_go.msg import SportModeState
from unitree_api.msg import Request
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue, ParameterType

ROBOT_SPORT_API_ID_STATICWALK = 1061
ROBOT_SPORT_API_ID_TROTRUN = 1062
ROBOT_SPORT_API_ID_SIT = 1009
ROBOT_SPORT_API_ID_RISESIT = 1010
# 전부 공식 unitree_ros2 ros2_sport_client.h 로 확인함 (2026-10-31)

STOP_VEL_THRESH = 0.05
STOP_HOLD_SECONDS = 2.5
GREEN_CONFIRM_SECONDS = 0.5  # stopline_mission_node.py/real_course_runner.py와 동일한
# 안전장치 -- GREEN이 끊기지 않고 이만큼 연속으로 들어와야 출발. 수정 전엔 1프레임만
# GREEN이어도 바로 _advance() 했음(디바운스 없는 안전 구멍).
SIT_HEIGHT_THRESH = 0.15     # TODO: 현장에서 Sit 실측 후 보정
PARK_HOLD_SECONDS = 5.5

# ego_planner 기본(엄격) 파라미터 — gosu_slam/config/ego_planner_slam.yaml 실측값 그대로.
EGO_PLANNER_DEFAULT = {'min_obs_h': -0.20, 'max_obs_h': 1.10, 'max_vel': 0.8}
# 계단/경사 구간용 완화값 — TODO: 현장 실측 후 보정. 지금은 "로봇 키(약 0.3~0.4m)보다
# 훨씬 넓게 잡아서 정상 주행 높이 범위에서는 아무것도 장애물로 안 걸리게" 하는 추정치.
EGO_PLANNER_RELAXED_OBS = {'min_obs_h': -1.0, 'max_obs_h': 2.5}


ZONE_HYSTERESIS_M = 0.3  # stair_geofence_node.py에서 가져온 값 — 경계선에서 떨리는 것 방지


def in_zone(x, y, zone, margin=0.0):
    return (zone['x_min'] - margin <= x <= zone['x_max'] + margin
            and zone['y_min'] - margin <= y <= zone['y_max'] + margin)


class TrackSequencer(Node):
    def __init__(self, config_path):
        super().__init__('track_sequencer')

        with open(config_path) as f:
            cfg = json.load(f)
        self.segments = cfg['segments']
        self.idx = 0
        self.sub_state = {}  # 구간별 진행 상태(시작시각 등)를 담는 범용 딕셔너리

        self.pos = None
        self.velocity = None
        self.body_height = None
        self.light_state = 'UNKNOWN'
        self.green_since_t = None  # GREEN이 끊기지 않고 시작된 시각. 끊기면 None.

        self.create_subscription(Odometry, '/Odometry', self.cb_odom, 10)
        self.create_subscription(SportModeState, '/sportmodestate', self.cb_state, 10)
        self.create_subscription(String, '/mission/traffic_light_state', self.cb_light, 10)
        self.pub_req = self.create_publisher(Request, '/api/sport/request', 10)
        self.pub_status = self.create_publisher(String, '/mission/track_sequencer_status', 10)
        self.pub_camera_active = self.create_publisher(Bool, '/mission/camera_active', 10)
        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # ego_planner 런타임 파라미터 변경용 서비스 클라이언트 (2026-11-01).
        # ego_planner_node.cpp에 add_on_set_parameters_callback 패치를 추가해서
        # (~/ws/Rgosu/ws_ego_planner_tunable) 이게 실제로 동작하게 만들었다 — 원래
        # 코드는 파라미터를 생성자에서 한 번만 읽어서 런타임 변경이 안 먹혔음.
        self.ego_param_client = self.create_client(SetParameters, '/ego_planner_node/set_parameters')
        self._last_ego_params_sent = None

        self._last_camera_active = None
        self._last_obstacle_filter = None

        # 목표지점 재전송용 상태 (2026-11-02, 실측으로 발견한 버그의 수정):
        # ego_planner의 onGoal()은 아직 /Odometry를 한 번도 못 받았으면
        # ("has_odom_" false) 목표를 그냥 버리고 경고만 남긴다 — 재시도가 없다.
        # TrackSequencer가 노드 시작 즉시 목표를 보내면, FAST-LIO가 아직 첫
        # 오도메트리를 못 낸 시점일 수 있어서(초기화에 몇 초 걸림) 그 목표가
        # 조용히 유실될 수 있다. 라이브 테스트로 실제로 재현해서 확인함.
        # 해법: 목표를 한 번만 보내지 않고, 구간 진입 후 일정 시간 동안 1초
        # 간격으로 같은 목표를 계속 재전송한다 — ego_planner가 odom을 받는
        # 순간 그중 하나는 반드시 통과한다. 같은 좌표를 반복 수신하는 건
        # 무해함(멱등).
        #
        # 추가 버그(2026-11-02, 무모터 재생 검증으로 발견): odom 수신 자체는
        # 성공해도("Received new target goal" 로그 찍힘) A*가 매번
        # "search failed to reach target"으로 실패했다. FAST-LIO 로그의
        # "Initialize the map kdtree" 시각과 첫 A* 시도 시각이 0.78초밖에
        # 안 차이나는 걸 확인 — 맵(포인트클라우드 누적)이 거의 비어있는
        # 상태에서 6m짜리 경로를 요청한 게 원인으로 보인다. 재전송 창을
        # 10초 -> 90초로 늘려서, 그 사이 FAST-LIO가 더 많은 맵을 쌓은
        # 뒤의 재전송 중 하나가 A*를 통과시킬 기회를 준다.
        self._pending_goal = None
        self._pending_goal_deadline = 0.0
        self._pending_goal_last_sent = 0.0
        self._pending_goal_resend_interval = 1.0
        self._pending_goal_window = 90.0

        self.create_timer(0.1, self.tick)
        self.get_logger().info(
            f'TrackSequencer 시작 — {len(self.segments)}개 구간, 현재: {self._cur()["name"]}')
        self._publish_goal_if_any(self._cur())  # 0번 구간(start)은 _advance()를 안 거치니 시작할 때 한 번 발행

    def _cur(self):
        return self.segments[self.idx]

    def cb_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.pos = (p.x, p.y)

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

    def _send_gait(self, api_id):
        req = Request()
        # ★ 2026-11-04: go2_bridge/cmd_vel_to_sport_node.cpp + 팀 review 브랜치
        # 공통 패턴(identity.id + parameter 채우기)에 맞춤 -- api_id만 보내면
        # 로봇 쪽에서 조용히 무시될 위험.
        req.header.identity.id = self.get_clock().now().nanoseconds
        req.header.identity.api_id = api_id
        req.parameter = '{}'
        self.pub_req.publish(req)

    def _publish_goal_if_any(self, seg):
        """세그먼트에 'goal'이 있으면 /goal_pose로 발행하고, 재전송 타이머를
        건다 — ego_planner가 이걸 받으면 그 지점까지 라이다로 장애물(콘/벽)
        피해가며 알아서 주행한다 (2026-11-02, 사용자 요청: 구간마다 웨이포인트를
        세세히 주지 말고 start에서 주차구역으로 목표 1개, 주차 끝나면 출구
        너머로 목표 1개 — 나머지는 ego_planner의 라이다 회피에 맡긴다. 계단/경사
        구간은 이미 obstacle_filter=relaxed로 통행 가능하게 열어뒀으니 그
        목표를 향해 가는 도중 자연히 넘어간다).

        한 번만 보내지 않는 이유: ego_planner는 아직 /Odometry를 한 번도 못
        받았으면 목표를 버리고 경고만 남기며 재시도가 없다(라이브 테스트로
        확인한 실제 버그) — 그래서 여기서 보낸 뒤 10초간 1초 간격으로 같은
        목표를 _resend_pending_goal()이 계속 재전송한다."""
        goal = seg.get('goal')
        if goal is None:
            return
        self._pending_goal = goal
        now = time.monotonic()
        self._pending_goal_deadline = now + self._pending_goal_window
        self._pending_goal_last_sent = now
        self._send_goal_msg(seg['name'], goal)

    def _send_goal_msg(self, seg_name, goal):
        msg = PoseStamped()
        msg.header.frame_id = 'camera_init'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(goal['x'])
        msg.pose.position.y = float(goal['y'])
        msg.pose.orientation.w = 1.0  # ego_planner는 목표 방향(yaw)은 안 쓰고 위치만 봄
        self.pub_goal.publish(msg)
        self.get_logger().warn(f'[{seg_name}] 목표지점 발행 -> ({goal["x"]}, {goal["y"]})')

    def _resend_pending_goal(self):
        if self._pending_goal is None:
            return
        now = time.monotonic()
        if now > self._pending_goal_deadline:
            self._pending_goal = None  # 재전송 창 종료 — 이쯤이면 odom이 안 나오는 게 더 큰 문제
            return
        if now - self._pending_goal_last_sent >= self._pending_goal_resend_interval:
            self._pending_goal_last_sent = now
            self._send_goal_msg(f'{self._cur()["name"]}(재전송)', self._pending_goal)

    def _set_ego_planner_params(self, params_dict):
        """ego_planner_node에 ros2 param set과 동일한 효과를 서비스 호출로 보낸다.
        같은 값을 중복 전송하지 않고(변경된 것만), 서비스가 아직 안 떠있으면
        조용히 건너뛴다(로봇 없이 테스트할 때 ego_planner가 안 떠있어도 안 죽게)."""
        if params_dict == self._last_ego_params_sent:
            return
        if not self.ego_param_client.service_is_ready():
            self.get_logger().warn(
                f'ego_planner set_parameters 서비스 미연결 — 파라미터 변경 건너뜀: {params_dict}')
            return

        req = SetParameters.Request()
        for name, value in params_dict.items():
            p = ParameterMsg()
            p.name = name
            p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
            req.parameters.append(p)

        future = self.ego_param_client.call_async(req)
        future.add_done_callback(
            lambda f, pd=params_dict: self.get_logger().info(f'ego_planner 파라미터 변경 적용됨: {pd}'))
        self._last_ego_params_sent = params_dict

    def _advance(self):
        self.sub_state = {}
        self.idx += 1
        if self.idx < len(self.segments):
            self.get_logger().warn(f'구간 완료 -> 다음: {self._cur()["name"]}')
            self._publish_goal_if_any(self._cur())
        else:
            self.get_logger().warn('모든 구간 완료 (OUT)')

    def tick(self):
        self._resend_pending_goal()  # self.pos 유무와 무관 — ego_planner가 odom 받을 때까지 독립적으로 재시도
        if self.idx >= len(self.segments) or self.pos is None:
            return
        seg = self._cur()
        x, y = self.pos
        now = time.monotonic()

        # 센서 비중 동적 전환 — 정지선 구간에서만 카메라 켜고 나머진 라이다 전담.
        # 대회장이 열린 공간이라 주변 사람/사물 때문에 신호등 오인식(빨강 오탐) 위험이
        # 있고, 콘 슬라럼처럼 정밀 회피가 필요한 구간엔 연산을 라이다 쪽에 몰아줘야
        # 한다는 사용자 지적 반영 (2026-11-01, README '센서 비중 동적 전환' 참고).
        cam_active = bool(seg.get('camera_active', False))
        if cam_active != self._last_camera_active:
            self.get_logger().info(f'[{seg["name"]}] 카메라 비중 {"ON" if cam_active else "OFF"}')
            self._last_camera_active = cam_active
        self.pub_camera_active.publish(Bool(data=cam_active))

        # 장애물 높이필터 — 계단/경사 구간은 완화(통행가능 지형을 장애물로 오인하지
        # 않게), 콘 슬라럼 등 나머지는 엄격 유지(둘 다 "피해야 할 것"으로 동일 취급
        # — 콘과 벽을 굳이 구분할 필요는 없다는 결론, README 참고).
        # tune_planner 구간의 params(예: 콘 슬라럼 저속화)도 여기서 같이 합쳐서
        # 한 번에 보낸다 — 높이필터랑 속도를 따로 호출하면 서로 덮어쓸 수 있어서.
        obs_filter = seg.get('obstacle_filter', 'strict')
        if obs_filter != self._last_obstacle_filter:
            self.get_logger().info(f'[{seg["name"]}] 장애물 높이필터 -> {obs_filter}')
            self._last_obstacle_filter = obs_filter

        desired_ego_params = dict(EGO_PLANNER_DEFAULT)
        if obs_filter == 'relaxed':
            desired_ego_params.update(EGO_PLANNER_RELAXED_OBS)
        if seg.get('type') == 'tune_planner':
            seg_params = seg.get('params', {})
            desired_ego_params.update({k: v for k, v in seg_params.items() if k in desired_ego_params})
        self._set_ego_planner_params(desired_ego_params)
        zone = seg.get('zone')
        # 진입(entered)은 엄격 경계, 이탈(left)은 마진만큼 더 나가야 인정 — 경계선
        # 바로 위에서 위치가 ±수cm 흔들려도 걸음걸이 전환이 반복(채터링)되지 않게.
        entered_now = (zone is None) or in_zone(x, y, zone, margin=0.0)
        left_now = (zone is not None) and not in_zone(x, y, zone, margin=ZONE_HYSTERESIS_M)
        inside = entered_now  # stopline/parking 등 엄격 판정이 필요한 타입용

        stype = seg['type']

        if stype == 'pass_through':
            if entered_now:
                self._advance()

        elif stype == 'gait_switch':
            if entered_now and not self.sub_state.get('entered'):
                self.sub_state['entered'] = True
                gait = {'STATICWALK': ROBOT_SPORT_API_ID_STATICWALK,
                        'TROTRUN': ROBOT_SPORT_API_ID_TROTRUN}[seg.get('gait', 'STATICWALK')]
                self.get_logger().warn(f'[{seg["name"]}] 진입 -> 걸음걸이 전환 {seg.get("gait")}')
                self._send_gait(gait)
            if left_now and self.sub_state.get('entered'):
                self.get_logger().info(f'[{seg["name"]}] 이탈(마진 {ZONE_HYSTERESIS_M}m 확인) -> TROTRUN 복귀, 다음 구간')
                self._send_gait(ROBOT_SPORT_API_ID_TROTRUN)
                self._advance()

        elif stype == 'tune_planner':
            # 실제 파라미터 변경은 위에서 desired_ego_params로 매 틱 이미 보내고
            # 있다 — 여기선 구간 진입/이탈 로그와 다음 구간으로의 전환만 담당.
            if entered_now and not self.sub_state.get('entered'):
                self.sub_state['entered'] = True
                self.get_logger().warn(f'[{seg["name"]}] 진입 -> 플래너 파라미터 {seg.get("params")} 적용')
            if left_now and self.sub_state.get('entered'):
                self.get_logger().info(f'[{seg["name"]}] 이탈(마진 확인) -> 파라미터 복원, 다음 구간')
                self._advance()

        elif stype == 'stopline':
            is_stopped = self.velocity is not None and self.velocity < STOP_VEL_THRESH
            if not inside:
                return
            sub = self.sub_state.setdefault('phase', 'APPROACH')
            if sub == 'APPROACH':
                if is_stopped:
                    self.sub_state['stop_t'] = now
                    self.sub_state['phase'] = 'STOPPING'
            elif sub == 'STOPPING':
                if not is_stopped:
                    self.sub_state['phase'] = 'APPROACH'
                elif now - self.sub_state['stop_t'] >= STOP_HOLD_SECONDS:
                    self.sub_state['phase'] = 'WAIT_SIGNAL'
                    self.get_logger().info(f'[{seg["name"]}] 2초+ 정지 완료 -> 신호 확인')
            elif sub == 'WAIT_SIGNAL':
                if not is_stopped:
                    self.sub_state['phase'] = 'APPROACH'
                elif (self.light_state == 'GREEN' and self.green_since_t is not None
                      and now - self.green_since_t >= GREEN_CONFIRM_SECONDS):
                    self.get_logger().warn(
                        f'[{seg["name"]}] 녹색 신호 {GREEN_CONFIRM_SECONDS}초 연속 확인 -> 출발, 다음 구간')
                    self._advance()
                # GREEN이어도 연속 확인 시간을 못 채웠으면 계속 WAIT_SIGNAL 대기
                # (단발성 오탐이면 cb_light에서 green_since_t가 리셋돼서 여기로 못 넘어옴)

        elif stype == 'parking':
            if not inside:
                return
            sub = self.sub_state.setdefault('phase', 'ENTER')
            if sub == 'ENTER':
                # 2026-11-04: 정지선(stopline) 로직은 속도가 충분히 낮아진 걸
                # 확인하고서야 다음 단계로 넘어가는데, 주차는 구역 진입 즉시
                # (속도 무관) Sit을 보내고 있었음 -- 빠르게 들어오면서 앉으면
                # 전도(-30점) 위험. stopline과 같은 기준(STOP_VEL_THRESH)으로
                # 맞춤: 멈출 때까지 ENTER에서 기다렸다가 Sit을 보냄.
                is_stopped = self.velocity is not None and self.velocity < STOP_VEL_THRESH
                if not is_stopped:
                    return
                self.get_logger().warn(f'[{seg["name"]}] 구역 진입+정지 확인 -> Sit 전송')
                self._send_gait(ROBOT_SPORT_API_ID_SIT)
                self.sub_state['sit_t'] = now
                self.sub_state['phase'] = 'SITTING'
            elif sub == 'SITTING':
                sat = self.body_height is not None and self.body_height < SIT_HEIGHT_THRESH
                if sat:
                    self.sub_state['hold_t'] = now
                    self.sub_state['phase'] = 'HOLDING'
                elif now - self.sub_state['sit_t'] > 3.0:
                    self._send_gait(ROBOT_SPORT_API_ID_SIT)
                    self.sub_state['sit_t'] = now
            elif sub == 'HOLDING':
                if self.body_height is not None and self.body_height >= SIT_HEIGHT_THRESH:
                    self.sub_state['phase'] = 'SITTING'
                    self.sub_state['sit_t'] = now
                    return
                if now - self.sub_state['hold_t'] >= PARK_HOLD_SECONDS:
                    self.get_logger().warn(f'[{seg["name"]}] 5초+ 유지 완료 -> RiseSit, 다음 구간')
                    self._send_gait(ROBOT_SPORT_API_ID_RISESIT)
                    self._advance()

        out = String()
        out.data = f'seg={seg["name"]}({self.idx+1}/{len(self.segments)})|phase={self.sub_state.get("phase")}'
        self.pub_status.publish(out)


def main():
    rclpy.init()
    config_path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/segments_config.json'
    node = TrackSequencer(config_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
