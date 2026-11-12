#!/usr/bin/env python3
"""콘 보강 필터 노드 — ★ 2026-11-04 밤 고침: L1 근거리 추가 + 좌표·높이 기준 버그 수정 (계산은 cone_band.py).
★ 2026-11-05 밤: cb_wall이 매 메시지 .tolist()로 포인트클라우드 전체를 파이썬 리스트로
바꿔서 pc2.create_cloud_xyz32에 넘기던 걸 fast_cloud_xyz32(numpy .tobytes() 직접 포장)로
바꿈 — 11-05 새벽 시뮬레이션에서 이 노드를 ego_planner에 연결하자 길찾기가 막혔던 것의
유력한 원인으로 의심됐던 부분(포인트 수가 많아지면 점당 struct.pack 루프가 뒤처지면서
오래된 지도를 넘겼을 가능성). 벤치마크·재연결 재검증은 아래 README 기록 참고.

고친 것 (11-04 밤, 17:56 콘 녹화로 확인):
  ① XT16 은 바닥 위 약 0.57 m → 가장 아래 링(-15°)도 23 cm 콘 꼭대기에 1.28 m 밖에서야 닿는다(가까운 콘은 XT16 에 안 보임).
     같은 녹화 0.3~0.9 m 낮은 물체: XT16 프레임당 0 개, L1(/utlidar/cloud_base) 1.1~1.4 개 → L1 을 기본 입력으로.
  ② 예전 코드는 /velodyne_points(라이다 좌표)에 몸체 자세를 바로 곱했다(외부파라미터 없음, wall_filter 와 같은 버그).
  ③ 예전 높이밴드(0.05~0.45)는 '라이다 원점 기준'이라 실제로는 바닥 위 0.62~1.02 m 를 골랐다 → 콘이 하나도 안 들어감.
     이제 바닥 기준: 프레임마다 L1 바닥 점으로 평면을 맞춰 그 위 0.08~0.30 m, 2 m 안(콘 23 cm).
  XT16 보강은 기본 꺼 둠(use_xt16) — 가까운 콘은 못 보고 파이썬으로 32k 점 20 Hz 를 받으면 CPU 를 많이 먹는다.
  콘 구간 밖(cone_zone_active=False)에서는 wall_cloud 를 그대로 넘긴다 — 지금 주행과 같은 동작.

(예전 설명)

배경: wall_filter_node(2026-11-03)는 "여러 인접 링이 거의 같은 수평거리"를 벽의
기준으로 쓴다 — 경사로/계단을 벽으로 오인하지 않으려고 일부러 그렇게 만듦.
그런데 2026-11-04 밤 시뮬레이션(sim_cone_vs_wallfilter.py, XT16 실제 16채널 각도
그대로 사용)으로 확인한 결과, **라바콘(원뿔)도 상당수 케이스에서 이 기준을 통과
못 해서 /mission/wall_cloud에서 빠진다**(거리·크기에 따라 들쭉날쭉, 18케이스 중
8개만 통과) — 즉 ego_planner가 콘을 장애물로 못 볼 수 있다는 뜻. 이건 전부터
README에 "포인트 사전분류 레이어(콘=회피대상) 추가, 미구현"으로 적혀있던 그
문제를 수치로 확정한 것.

이 노드는 그 해법의 1단계: wall_filter의 "진짜 벽" 판정은 그대로 믿고(건드리지
않음), 거기에 "콘 구간에서만" 높이밴드 기반 원시 포인트를 보강해서 합친다.
계단/경사 구간은 콘 슬라럼과 겹치지 않는 별개 구간이라(경기장 레이아웃:
라바콘 슬라럼 -> 신호등/정지선 -> X자교차(경사/계단) -> 주차), 높이밴드
필터를 "콘 구간에서만" 켜면 옛날 계단 오인 문제가 재발하지 않는다.

입력:
  /velodyne_points        (원시 ring 포인트클라우드)
  /Odometry               (수평보정용 roll/pitch, wall_filter와 동일한 수학)
  /mission/wall_cloud      (wall_filter_node 출력 -- 그대로 합쳐서 내보냄)
  /mission/cone_zone_active (Bool) -- ★ 아직 아무도 발행 안 함. 콘 슬라럼 구간
    실측 좌표 생기면 real_course_runner.py/track_sequencer_node.py에서 그 구간
    진입/이탈 시 True/False로 발행해줘야 함(camera_active와 같은 패턴).
    지금은 기본 False라서 이 노드를 띄워도 wall_cloud를 그대로 통과시키기만
    함(안전한 기본값 -- 아직 연결 안 돼도 기존 동작을 안 바꿈).
출력:
  /mission/obstacle_cloud  -- ego_planner의 cloud_topic을 나중에 이걸로
    바꿔야 완성됨(아직 안 바꿈, go2_full_stack.sh는 그대로 /mission/wall_cloud 사용).

TODO(실측 좌표 생기면):
  - min_obs_h_band/max_obs_h_band 를 실제 콘 높이로 재보정(지금은 0.05~0.45m 추정치)
  - real_course_runner.py 쪽에 cone_zone_active 발행 추가
  - go2_full_stack.sh의 ego_planner 호출을 -p cloud_topic:=/mission/obstacle_cloud 로 변경
  - 실제 콘 bag으로 재검증(지금은 합성 데이터로만 검증함)
"""
import sys, os, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Header
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cone_band import lidar_to_body, estimate_floor_below, band_world  # noqa: E402


def quat_to_rpy(w, x, y, z):
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = 2.0 * (w * y - z * x)
    pitch = np.copysign(np.pi / 2, sinp) if abs(sinp) >= 1 else np.arcsin(sinp)
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return float(roll), float(pitch), float(yaw)


def xyz(msg):
    if msg.width * msg.height == 0:
        return np.zeros((0, 3))
    a = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
    return np.stack([a['x'], a['y'], a['z']], 1).astype(np.float64)


_XYZ32_FIELDS = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                 for i, n in enumerate('xyz')]


def fast_cloud_xyz32(header, pts_f32):
    """pc2.create_cloud_xyz32(header, pts.tolist())와 같은 결과를 내지만, 점마다 파이썬
    struct.pack을 도는 create_cloud_xyz32 내부 루프 대신 .tobytes()로 한 번에 직렬화한다.
    ★ 2026-11-05: 이게 실주행 시뮬레이션에서 cone_raw_filter_node가 ego_planner 길찾기를
    막는 원인으로 의심됐던 부분(포인트 수·주행 부하가 늘수록 .tolist() 변환이 느려져서
    점점 오래된 지도를 넘겼을 가능성) — numpy 버퍼를 직접 포장해서 그 병목을 없앤다."""
    msg = PointCloud2()
    msg.header = header
    n = len(pts_f32)
    msg.height = 1
    msg.width = n
    msg.fields = _XYZ32_FIELDS
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * n
    msg.is_dense = True
    msg.data = np.ascontiguousarray(pts_f32, dtype=np.float32).tobytes()
    return msg


class ConeRawFilterNode(Node):
    def __init__(self):
        super().__init__('cone_raw_filter_node')
        P = self.declare_parameter
        P('odom_topic', '/Odometry'); P('wall_cloud_topic', '/mission/wall_cloud'); P('output_topic', '/mission/obstacle_cloud')
        P('l1_topic', '/utlidar/cloud_base'); P('use_l1', True)
        P('xt16_topic', '/velodyne_points'); P('use_xt16', False)
        P('min_obs_h_band', 0.08); P('max_obs_h_band', 0.30)          # 바닥 평면 기준(콘 23 cm)
        P('min_range', 0.30); P('max_range', 2.0)
        P('floor_below_default', 0.415)                                 # 몸체 아래 바닥(L1 로 갱신)
        P('band_max_age_s', 0.3)
        P('always_on', False)                                           # 시험용: 콘 구간 아니어도 보강
        g = lambda k: self.get_parameter(k).value
        self.min_h, self.max_h, self.rmin, self.rmax = g('min_obs_h_band'), g('max_obs_h_band'), g('min_range'), g('max_range')
        self.floor_below = g('floor_below_default'); self.max_age = g('band_max_age_s'); self.always_on = g('always_on')
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, g('odom_topic'), self.cb_odom, 10)
        self.create_subscription(PointCloud2, g('wall_cloud_topic'), self.cb_wall, 10)
        self.create_subscription(Bool, '/mission/cone_zone_active', self.cb_zone, 10)
        if g('use_l1'):
            self.create_subscription(PointCloud2, g('l1_topic'), lambda m: self.cb_band(m, 'l1'), qos)
        if g('use_xt16'):
            self.create_subscription(PointCloud2, g('xt16_topic'), lambda m: self.cb_band(m, 'xt16'), qos)
        self.pub = self.create_publisher(PointCloud2, g('output_topic'), 10)
        self.have_odom = False; self.t = (0.0, 0.0, 0.0); self.rpy = (0.0, 0.0, 0.0)
        self.zone = False; self.band = {}; self.n_band = 0
        self.create_timer(5.0, self.report)
        self.get_logger().info(f'cone_raw_filter_node(고친 판) — L1 {g("use_l1")} · XT16 {g("use_xt16")} · 바닥 위 [{self.min_h},{self.max_h}] m · '
                               f'{self.rmin}~{self.rmax} m · 콘 구간에서만 보강(always_on={self.always_on})')

    def cb_zone(self, msg):
        if bool(msg.data) != self.zone:
            self.get_logger().warn(f'cone_zone_active -> {msg.data}')
        self.zone = bool(msg.data)

    def cb_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.have_odom = True; self.t = (p.x, p.y, p.z); self.rpy = quat_to_rpy(q.w, q.x, q.y, q.z)

    def cb_band(self, msg, src):
        if not self.have_odom or not (self.zone or self.always_on):
            self.band.pop(src, None); return
        P = xyz(msg)
        if src == 'xt16':
            P = lidar_to_body(P)
        else:   # L1 은 바닥을 잘 보니 바닥 높이도 여기서 갱신(천천히)
            fb = estimate_floor_below(P, self.rpy[0], self.rpy[1], default=self.floor_below)
            self.floor_below = 0.8 * self.floor_below + 0.2 * fb
        W = band_world(P, self.rpy[0], self.rpy[1], self.rpy[2], self.t, self.floor_below, self.min_h, self.max_h, self.rmin, self.rmax)
        self.band[src] = (time.monotonic(), W); self.n_band += len(W)

    def cb_wall(self, msg):
        walls = xyz(msg).astype(np.float32)
        now = time.monotonic()
        extra = [W for (t0, W) in self.band.values() if now - t0 <= self.max_age and len(W)] if (self.zone or self.always_on) else []
        merged = np.vstack([walls] + extra) if extra else walls
        h = Header(); h.stamp = msg.header.stamp; h.frame_id = msg.header.frame_id or 'camera_init'
        self.pub.publish(fast_cloud_xyz32(h, merged))

    def report(self):
        if self.zone or self.always_on:
            self.get_logger().info(f'콘 보강 중 — 5초간 밴드 점 {self.n_band} · 바닥 몸체 아래 {self.floor_below:.3f} m')
        self.n_band = 0


def main():
    rclpy.init(args=sys.argv)
    node = ConeRawFilterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        # ★ 11-05: SIGTERM(run_sim.sh stop(), 혹은 그냥 프로세스 종료)이 오면 rclpy가
        # ExternalShutdownException을 던지는데 KeyboardInterrupt만 잡고 있어서 지저분한
        # traceback이 남았었음 -- 실행 중 크래시가 아니라 정상 종료 경로였던 것, 조용히 처리.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
