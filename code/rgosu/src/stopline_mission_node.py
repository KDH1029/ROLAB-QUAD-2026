#!/usr/bin/env python3
"""정지선 상태기 — 2026 로보월드 규정 5.2:
  "정지선 표식 내에 앞다리 2개가 들어온 상태로 2초 이상 정지하여 신호등을 인식,
   신호가 녹색이면 주행 가능, 빨간불이면 녹색 될 때까지 대기(신호는 5초 주기 변경).
   정지(2초)+신호 모두 정확히 수행하면 건당 50점."

입력:
  /mission/traffic_light_state  (traffic_light_detector_node, "RED|0.xx"/"GREEN|0.xx"/"UNKNOWN|0.00")
  /sportmodestate                 (로봇) — velocity로 "정지했는가" 판정

출력:
  /mission/stopline_state (std_msgs/String) — 상태 로그. 실제 출발/정지 자체는
  상위 주행 로직(ego_planner 등)이 이 상태를 보고 결정하게 둔다 — 이 노드는 로봇을
  직접 멈추거나 움직이지 않는다 (규정상 "정차 판단"이지 로봇 제어 자체가 다른
  서브시스템 책임이라, 여기서 섣불리 Move/StopMove를 보내면 주행 로직과 충돌 위험).

설계 메모:
  - "표식 안에 들어왔는가"(위치 판단)는 현장 좌표 실측 전이라 못 넣음 -- 지금은
    "속도가 충분히 낮다"만 proxy로 쓴다. 현장 가서 정지선 좌표 알면
    stair_geofence_node.py 패턴처럼 bounding-box 체크 추가할 것 (TODO).
  - 신호등 인식이 UNKNOWN인 동안은(예: 각도상 안 보임) 안전 쪽으로 "아직 정지 유지"로
    처리한다(=RED와 동일 취급) -- 오인식으로 섣불리 출발하는 것보다 안전.
"""
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from unitree_go.msg import SportModeState

STOP_VEL_THRESH = 0.05     # m/s, 이 아래면 "정지"로 판정
STOP_HOLD_SECONDS = 2.5    # 규정 2초 + 여유 0.5초
GREEN_CONFIRM_SECONDS = 0.5  # ★ 2026-11-04: GREEN을 이 시간만큼 "끊기지 않고 연속으로"
# 봐야 GO로 래치한다. 대회장엔 지나가는 사람 옷 색/외부 조명 같은 외부요인이 있어서
# 단 한 프레임의 오탐(YOLO conf>=0.25로 우연히 넘은 것)만으로 바로 출발 판정이
# 나면 안 됨(수정 전엔 cb_light에서 1프레임만 GREEN이어도 다음 tick에 즉시 GO
# 래치됐음 -- 디바운스가 전혀 없었던 안전 구멍). 신호는 5초 주기로 바뀌므로
# 0.5초 연속 확인은 실제 녹색일 때 충분히 빨리 통과하면서, 단발성 오탐은 걸러낸다.


class StoplineMission(Node):
    def __init__(self):
        super().__init__('stopline_mission')

        self.state = 'APPROACH'  # APPROACH -> STOPPING -> WAIT_SIGNAL -> GO
        self.light_state = 'UNKNOWN'
        self.velocity = None
        self.stop_start_t = None
        self.go_latched = False
        self.green_since_t = None  # GREEN이 끊기지 않고 시작된 시각(monotonic). 끊기면 None.

        self.create_subscription(String, '/mission/traffic_light_state', self.cb_light, 10)
        self.create_subscription(SportModeState, '/sportmodestate', self.cb_state, 10)
        self.pub_state = self.create_publisher(String, '/mission/stopline_state', 10)

        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info('stopline_mission 시작 (APPROACH)')

    def cb_light(self, msg: String):
        new_state = msg.data.split('|')[0] if msg.data else 'UNKNOWN'
        if new_state == 'GREEN':
            if self.light_state != 'GREEN':
                self.green_since_t = time.monotonic()  # GREEN 연속 구간 새로 시작
        else:
            self.green_since_t = None  # 끊겼으니 디바운스 카운터 리셋
        self.light_state = new_state

    def cb_state(self, msg: SportModeState):
        vx, vy, vz = msg.velocity
        self.velocity = (vx * vx + vy * vy) ** 0.5

    def tick(self):
        now = time.monotonic()
        is_stopped = (self.velocity is not None and self.velocity < STOP_VEL_THRESH)

        if self.state == 'APPROACH':
            if is_stopped:
                self.get_logger().info('정지 감지(추정) -> 2초 유지 시작')
                self.stop_start_t = now
                self.state = 'STOPPING'

        elif self.state == 'STOPPING':
            if not is_stopped:
                self.get_logger().warn('유지 중 재출발 감지 -> APPROACH로 복귀 (2초 못 채움)')
                self.state = 'APPROACH'
                return
            if now - self.stop_start_t >= STOP_HOLD_SECONDS:
                self.get_logger().info('2초+ 정지 완료 -> 신호 확인 단계')
                self.state = 'WAIT_SIGNAL'

        elif self.state == 'WAIT_SIGNAL':
            if not is_stopped:
                # 정지 유지 중에 밀렸거나 미끄러진 경우 -- 처음부터 다시
                self.get_logger().warn('신호 대기 중 이동 감지 -> APPROACH로 복귀')
                self.state = 'APPROACH'
                return
            if (self.light_state == 'GREEN' and self.green_since_t is not None
                    and now - self.green_since_t >= GREEN_CONFIRM_SECONDS):
                self.get_logger().warn(
                    f'녹색 신호 {GREEN_CONFIRM_SECONDS}초 연속 확인 -> GO (출발 가능)')
                self.go_latched = True
                self.state = 'GO'
            # RED/UNKNOWN이면 계속 WAIT_SIGNAL 유지 (안전 쪽: UNKNOWN도 대기).
            # GREEN이어도 아직 연속 확인 시간을 못 채웠으면 그냥 대기 -- 단발성
            # 오탐(지나가는 사람/조명)이었다면 다음 프레임에서 RED/UNKNOWN으로
            # 돌아오면서 green_since_t가 None으로 리셋되고 GO로 안 넘어간다.

        elif self.state == 'GO':
            # 규정: "주행 시작 후 빨간불로 바뀌어도 해당 회차는 주행 가능" -- 한 번
            # GO 래치되면 다시 안 묶는다. (다음 정지선은 새 인스턴스/리셋으로 처리하거나
            # 이 노드를 정지선마다 새로 띄우는 운용을 상정 -- 상위 로직에서 토픽
            # 네임스페이스로 구분하면 됨, TODO: 두 정지선 구분 리셋 로직)
            pass

        out = String()
        out.data = f'{self.state}|light={self.light_state}|vel={self.velocity}|go={self.go_latched}'
        self.pub_state.publish(out)


def main():
    rclpy.init()
    node = StoplineMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
