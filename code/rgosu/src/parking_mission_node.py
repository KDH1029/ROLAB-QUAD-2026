#!/usr/bin/env python3
"""주차 미션 상태기 — 2026 로보월드 규정 5.2:
  "지정 주차공간에 정차 후 앉아서 5초 이상 정지해야 득점(방향 무관).
   바닥 지정범위를 벗어나거나, 5초 미만 정지, 4다리 중 하나라도 제대로 안 접히면 무득점."

입력:
  /mission/parking_qr   (parking_detector_node) — QR 인식 여부/화면 내 비율(접근도)
  /sportmodestate        (로봇) — body_height로 "실제로 앉았는가" 확인
  /Odometry (또는 leg-fusion 쪽 동일 토픽) — 정차(속도 0 근접) 판정용, 현재는 속도만 봄

출력:
  /api/sport/request (unitree_api/msg/Request) — Sit(1009)/RiseSit(1010) 명령
  /mission/parking_state (std_msgs/String) — 상태 로그/디버그용

설계 메모:
  - "바닥 지정범위를 벗어나면 안 됨" 조건은 실제 주차구역의 월드 좌표를 아직 몰라서
    (현장 실측 전) 정확히 구현 못 함 -- 지금은 QR 접근도(box_area_frac)가 임계값
    이상으로 "충분히 가까움"을 proxy로 쓴다. 현장 가서 좌표 알게 되면
    stair_geofence_node.py와 같은 bounding-box 체크를 추가할 것 (TODO 표시해둠).
  - body_height 임계값(SIT_HEIGHT_THRESH)은 일반적인 Go2 sit 자세 높이 추정치 —
    실측해서 보정 필요 (로봇 연결되면 Sit 커맨드 보내보고 실제 /sportmodestate.body_height
    값 확인할 것).
"""
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from unitree_api.msg import Request
from unitree_go.msg import SportModeState

ROBOT_SPORT_API_ID_SIT = 1009       # 공식 unitree_ros2 ros2_sport_client.h 로 확인함 (2026-10-31)
ROBOT_SPORT_API_ID_RISESIT = 1010

# TODO: 로봇 연결되면 실측해서 교체
QR_CLOSE_ENOUGH_FRAC = 0.08   # 화면의 8% 이상 차지하면 "충분히 가까움"으로 proxy 판정
SIT_HEIGHT_THRESH = 0.15      # body_height 가 이 아래로 내려가면 "앉았다"로 판정 (추정치, 미보정)
HOLD_SECONDS = 5.5            # 규정 5초 + 여유 0.5초
STOP_VEL_THRESH = 0.05        # m/s -- 2026-11-04: docstring엔 "정차(속도 0 근접) 판정"이
# 있다고 적혀있었는데 실제 코드엔 없었음(QR 근접도만으로 바로 Sit) -- 빠른 속도로
# 들어오면서 Sit하면 전도(-30점) 위험이라 실제로 채워넣음(real_course_runner.py/
# track_sequencer_node.py와 동일 기준).


class ParkingMission(Node):
    def __init__(self):
        super().__init__('parking_mission')

        self.state = 'APPROACH'  # APPROACH -> SITTING -> HOLDING -> DONE
        self.qr_close = False
        self.body_height = None
        self.velocity = None
        self.sit_sent_t = None
        self.hold_start_t = None
        self.scored = False

        self.create_subscription(String, '/mission/parking_qr', self.cb_qr, 10)
        self.create_subscription(SportModeState, '/sportmodestate', self.cb_state, 10)
        self.pub_req = self.create_publisher(Request, '/api/sport/request', 10)
        self.pub_state = self.create_publisher(String, '/mission/parking_state', 10)

        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info('parking_mission 시작 (APPROACH)')

    def cb_qr(self, msg: String):
        import json
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        self.qr_close = bool(d.get('found')) and d.get('box_area_frac', 0) >= QR_CLOSE_ENOUGH_FRAC

    def cb_state(self, msg: SportModeState):
        self.body_height = msg.body_height
        vx, vy, vz = msg.velocity
        self.velocity = (vx * vx + vy * vy) ** 0.5

    def tick(self):
        now = time.monotonic()

        if self.state == 'APPROACH':
            is_stopped = self.velocity is not None and self.velocity < STOP_VEL_THRESH
            if self.qr_close and is_stopped:
                self.get_logger().warn('QR 충분히 접근 + 정지 확인 -> Sit 명령 전송')
                self._send(ROBOT_SPORT_API_ID_SIT)
                self.sit_sent_t = now
                self.state = 'SITTING'

        elif self.state == 'SITTING':
            # Sit 명령 보낸 뒤 실제로 몸통이 낮아졌는지 확인 (TODO: 임계값 현장 보정)
            sat_down = (self.body_height is not None and self.body_height < SIT_HEIGHT_THRESH)
            if sat_down:
                self.get_logger().info(f'착석 확인됨(body_height={self.body_height:.3f}) -> 5초 유지 시작')
                self.hold_start_t = now
                self.state = 'HOLDING'
            elif now - self.sit_sent_t > 3.0:
                # 3초 지나도 안 앉았으면 재전송 (한 번 유실됐을 수 있음)
                self.get_logger().warn('Sit 명령 후 3초 경과했는데 착석 미확인 -> 재전송')
                self._send(ROBOT_SPORT_API_ID_SIT)
                self.sit_sent_t = now

        elif self.state == 'HOLDING':
            # 유지 중 몸통이 다시 올라오면(흔들림/밀림 등) 실패로 보고 처음부터
            if self.body_height is not None and self.body_height >= SIT_HEIGHT_THRESH:
                self.get_logger().warn('유지 중 이탈 감지(body_height 복귀) -> 다시 SITTING')
                self.state = 'SITTING'
                self.sit_sent_t = now
                return
            elapsed = now - self.hold_start_t
            if elapsed >= HOLD_SECONDS:
                self.scored = True
                self.get_logger().warn(f'주차 미션 완료 ({elapsed:.1f}초 유지) -> RiseSit')
                self._send(ROBOT_SPORT_API_ID_RISESIT)
                self.state = 'DONE'

        out = String()
        out.data = f'{self.state}|qr_close={self.qr_close}|body_h={self.body_height}|scored={self.scored}'
        self.pub_state.publish(out)

    def _send(self, api_id):
        req = Request()
        # 2026-11-04: real_course_runner.py/track_sequencer_node.py/stair_geofence_node.py와
        # 같은 이유로 identity.id + parameter 채움 -- api_id만 보내면 로봇이 조용히 무시할
        # 위험(검증된 패턴: go2_bridge/cmd_vel_to_sport_node.cpp, 팀 review 브랜치).
        req.header.identity.id = self.get_clock().now().nanoseconds
        req.header.identity.api_id = api_id
        req.parameter = '{}'
        self.pub_req.publish(req)


def main():
    rclpy.init()
    node = ParkingMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
