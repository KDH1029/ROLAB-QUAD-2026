#!/usr/bin/env python3
"""계단 지오펜스 전환 노드 (초안, 미완성 — 좌표 확정 후 완성할 것)

FAST-LIO 실시간 오도메트리(/Odometry_filtered)가 미리 알아둔 계단 영역
(camera_init = 출발자세 기준 bounding box)에 들어오면 STATICWALK(API 1061)로
전환하고, 벗어나면 평소 게이트로 복귀시킨다.

TODO (다음 세션에서 할 것):
- [ ] STAIR_BOX 좌표를 CloudCompare에서 확인한 실제 값으로 교체
      (1차 후보: X 0.0~3.5, Y 1.5~5.0 — ~/ws/junsu/README.md 참고,
       사람이 직접 확인 전까지 미확정)
  - [ ] 진입/이탈에 히스테리시스(마진) 추가 — 경계선에서 떨렸다 반복하는 거 방지
  - [ ] "진입 시 1회만 STATICWALK 호출" 로직 (현재는 매 틱마다 호출 — 비효율)
  - [ ] ego_planner의 max_obs_h 필터를 이 구간에서 완화하는 방법 조사
        (런타임 파라미터 변경 서비스가 있는지, 아니면 재시작 필요한지 확인 안 됨)
  - [ ] 실제 로봇으로 지오펜스 진입/이탈 반응속도 테스트
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from unitree_api.msg import Request

ROBOT_SPORT_API_ID_STATICWALK = 1061  # 공식 unitree_ros2 ros2_sport_client.h 로 확인함 (2026-10-31)
ROBOT_SPORT_API_ID_TROTRUN = 1062      # 공식 확인됨 (2026-10-31)

# TODO: 실측 좌표로 교체. camera_init(출발자세) 기준.
STAIR_BOX = {'x_min': 0.0, 'x_max': 3.5, 'y_min': 1.5, 'y_max': 5.0}
MARGIN = 0.3  # 히스테리시스 — 박스보다 이 값만큼 안쪽에서 나가야 "이탈"로 인정


class StairGeofence(Node):
    def __init__(self):
        super().__init__('stair_geofence')
        self.in_stair_zone = False
        self.sub = self.create_subscription(
            Odometry, '/Odometry_filtered', self.odom_cb, 10)
        self.pub = self.create_publisher(Request, '/api/sport/request', 10)
        self.get_logger().info(
            f'계단 지오펜스 감시 시작: X[{STAIR_BOX["x_min"]}~{STAIR_BOX["x_max"]}] '
            f'Y[{STAIR_BOX["y_min"]}~{STAIR_BOX["y_max"]}] (TODO: 실측값 아님)')

    def odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        inside = (STAIR_BOX['x_min'] <= x <= STAIR_BOX['x_max']
                  and STAIR_BOX['y_min'] <= y <= STAIR_BOX['y_max'])
        # 이탈 판정은 마진을 둬서 경계에서 떨지 않게
        outside = (x < STAIR_BOX['x_min'] - MARGIN or x > STAIR_BOX['x_max'] + MARGIN
                   or y < STAIR_BOX['y_min'] - MARGIN or y > STAIR_BOX['y_max'] + MARGIN)

        if inside and not self.in_stair_zone:
            self.in_stair_zone = True
            self.get_logger().warn(f'계단 구역 진입 (x={x:.2f}, y={y:.2f}) -> STATICWALK')
            self._send_gait(ROBOT_SPORT_API_ID_STATICWALK)
        elif outside and self.in_stair_zone:
            self.in_stair_zone = False
            self.get_logger().info(f'계단 구역 이탈 (x={x:.2f}, y={y:.2f}) -> TROTRUN 복귀')
            self._send_gait(ROBOT_SPORT_API_ID_TROTRUN)

    def _send_gait(self, api_id):
        req = Request()
        # ★ 2026-11-04: go2_bridge/cmd_vel_to_sport_node.cpp + 팀 review 브랜치
        # 공통 패턴(identity.id + parameter 채우기)에 맞춤 -- api_id만 보내면
        # 로봇 쪽에서 조용히 무시될 위험.
        req.header.identity.id = self.get_clock().now().nanoseconds
        req.header.identity.api_id = api_id
        req.parameter = '{}'
        self.pub.publish(req)


def main():
    rclpy.init()
    node = StairGeofence()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
