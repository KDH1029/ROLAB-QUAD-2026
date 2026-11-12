#!/usr/bin/env python3
"""오도메트리가 무너지는 것을 감시한다. 구독만 하고 주행에는 절대 개입하지 않는다.

받는 것
  /Odometry   nav_msgs/Odometry          FAST-LIO 출력
  /lowstate   unitree_go/msg/LowState    로봇 자체 IMU (중력 기준이라 롤·피치를 믿을 수 있다)

내는 것
  odometry/health   std_msgs/String   ok=1|0 과 판단 근거
  (그게 전부다. cmd_vel 도 goal 도 TF 도 건드리지 않는다)

【무엇을 잡아내나 — 조사로 확인된 실제 고장 모습】
FAST-LIO 발산은 서서히 흐르는 게 아니라 **자세가 한 번에 깨지고 영영 못 돌아온다.**
매핑본에서 한 스캔 만에 6.5 m -> 985 m 로 튀었고, 배증시간 14스캔으로 약 4초 만에 24 km 까지 갔다.
재생에서도 50 ms 사이에 1.36 m(=27 m/s) 도약이 나왔다.

그래서 세 가지를 본다.

① **자세 대조** — 로봇 자체 IMU 의 롤·피치는 중력 기준이라 발산 중에도 옳았다(±5~10° 안).
   같은 시각 FAST-LIO 는 ±40° 로 튀었다. 둘의 차이가 벌어지면 FAST-LIO 가 틀린 것이다.
   **이게 가장 확실한 신호다.** 실측: 정상 구간 중앙값 차이 1.4°, 발산 후 9.7~17.1°.

② **물리적 불가능** — 한 주기 이동량이 로봇 최고속도(3.7 m/s)를 넘으면 추정 오차다.

③ **출력 끊김** — 오도메트리가 멈췄다가 다시 붙는 순간이 위험하다. 공백이 길수록 위험하다
   (실측: 0.15~0.40초 공백은 전부 회복, 1.55초 공백은 회복 못 함).

【공분산은 쓰지 않는다 — 중요한 부정 결과】
`/Odometry` 의 pose.covariance 는 고착 순간에도 23%밖에 안 움직였다. 정상 주행의 변동폭이
이미 3.5배라 신호가 묻힌다. **필터는 17° 틀린 채로 완전히 확신하고 있었다.** 그래서 안 본다.

【왜 자동으로 멈추지 않나】
사용자 요구: "주행을 절대로 방해하면 안 된다."
이 노드는 알리기만 한다. 멈출지는 사람이 정한다.
"""
import argparse
import math
import sys
import time
from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def rpy_of(q):
    sr = 2 * (q.w * q.x + q.y * q.z)
    cr = 1 - 2 * (q.x * q.x + q.y * q.y)
    sp = 2 * (q.w * q.y - q.z * q.x)
    sp = max(-1.0, min(1.0, sp))
    sy = 2 * (q.w * q.z + q.x * q.y)
    cy = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(sr, cr), math.asin(sp), math.atan2(sy, cy)


class OdometryMonitor(Node):

    def __init__(self, a):
        super().__init__('odometry_monitor')
        self.a = a
        self.imu_rp = None            # 로봇 자체 IMU 의 (roll, pitch), 라디안
        self.imu_at = 0.0
        # 자세차의 최근 기준선 — (시각, 롤차, 피치차)를 창 길이만큼 들고 있는다
        self.att_hist = deque()
        self.last = None              # (시각, x, y, z)
        self.att_bad = 0
        self.jump_bad = 0
        self.worst_gap = 0.0
        self.worst_att = 0.0
        self.worst_speed = 0.0
        self.alarmed = False       # 한 번이라도 이상이 있었나 (기록용, 안 지워진다)
        self.bad_now = False       # 지금 이상인가 (회복하면 꺼진다)
        self.clean = 0             # 연속으로 깨끗한 프레임 수
        self.events = 0            # 이상 사건 횟수
        self.t_first = None        # 첫 오도메트리를 받은 시각 — 예열 기준
        self.n = 0

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Odometry, a.odom_topic, self.on_odom, qos)
        self.pub = self.create_publisher(String, 'odometry/health', 10)
        self.create_timer(a.report_every, self.report)

        try:
            from unitree_go.msg import LowState
            self.create_subscription(LowState, a.lowstate_topic, self.on_lowstate, qos)
            self.have_imu = True
            self.get_logger().info('로봇 IMU(%s)로 자세를 대조한다 — 가장 확실한 신호다'
                                   % a.lowstate_topic)
        except ImportError:
            self.have_imu = False
            self.get_logger().warn('unitree_go 메시지가 없다 — 자세 대조를 못 한다. '
                                   '도약과 끊김만 본다 (감지력이 많이 떨어진다)')

    def on_lowstate(self, m):
        r = m.imu_state.rpy
        self.imu_rp = (float(r[0]), float(r[1]))
        self.imu_at = time.time()

    def on_odom(self, m):
        self.n += 1
        now = time.time()
        if self.t_first is None:
            self.t_first = now
            self.get_logger().info('오도메트리가 들어오기 시작했다 — %.0f초 예열 뒤 판정을 시작한다'
                                   % self.a.warmup)
        # 【예열】 FAST-LIO 는 IMU 초기화에 몇 초가 걸리고, 그동안 자세도 출력 간격도 쓰레기다.
        # 재생에서는 bag 시계까지 튄다. 이 구간을 판정하면 시작하자마자 오경보가 난다(실측).
        warming = (now - self.t_first) < self.a.warmup
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p = m.pose.pose.position
        roll, pitch, _ = rpy_of(m.pose.pose.orientation)
        reasons = []

        # ③ 끊김
        if self.last is not None:
            dt = t - self.last[0]
            if dt > self.a.gap_warn:
                self.worst_gap = max(self.worst_gap, dt)
                reasons.append('출력이 %.2f초 끊겼다' % dt)
            # ② 물리적 불가능
            if dt > 1e-4:
                d = math.dist((p.x, p.y, p.z), self.last[1:])
                v = d / dt
                self.worst_speed = max(self.worst_speed, v)
                if v > self.a.max_speed:
                    self.jump_bad += 1
                    reasons.append('한 주기에 %.2f m 이동 (=%.1f m/s, 로봇 최고속도 %.1f 초과)'
                                   % (d, v, self.a.max_speed))
                else:
                    self.jump_bad = max(0, self.jump_bad - 1)
        self.last = (t, p.x, p.y, p.z)

        # ① 자세 대조 — 이게 핵심이다
        #
        # 【절대값에서 기준선 대비로 — 2026-09-21】
        # 예전에는 |FAST-LIO 자세 - IMU 자세| 를 그대로 문턱과 비교했다.
        # bag 3종을 재보니 그 차이에 **세션마다 다른 치우침**이 들어 있었다:
        #     live_0910_1512  롤 +5.93도  피치 -3.51도
        #     live_0910_1521  롤 -0.52도  피치 +3.60도
        #     live_0910_1524  롤 -0.09도  피치 +4.95도
        # 게다가 같은 세션 안에서도 흘러간다(1512: 15~38초 롤 -1.64 -> 120초~ +5.94).
        # 그래서 절대 문턱 8도가 **건강한 구간에서도 32회 연속**으로 터졌다. 오탐이다.
        #
        # 상수 편향을 빼는 안은 데이터가 **반증했다** — bag 전체 중앙값을 빼면
        # 알람 연속이 76 -> 239 로 오히려 나빠졌다(흘러간 만큼 반대로 틀어져서).
        #
        # 그래서 최근 att_win 초의 중앙값을 기준선으로 두고 **거기서 얼마나
        # 벗어났나**를 본다. 느린 흐름은 기준선이 따라가므로 안 걸리고,
        # 발산은 기준선이 못 따라갈 만큼 빨라서 그대로 걸린다.
        # 실측(창 10초 / 문턱 8도): 건강 구간 최대연속 1회(조용), 발산 49회(잡힘).
        #
        # ★ 2026-09-13 에 젯슨의 z 가드가 똑같은 문제를 겪었고 같은 방법으로 고쳤다.
        if self.imu_rp is not None and time.time() - self.imu_at < 1.0:
            sr = math.degrees(roll - self.imu_rp[0])
            sp = math.degrees(pitch - self.imu_rp[1])
            self.att_hist.append((now, sr, sp))
            while self.att_hist and now - self.att_hist[0][0] > self.a.att_win:
                self.att_hist.popleft()
            if len(self.att_hist) >= self.a.att_min_n:
                _r = sorted(h[1] for h in self.att_hist)
                _p = sorted(h[2] for h in self.att_hist)
                br = _r[len(_r) // 2]
                bp = _p[len(_p) // 2]
            else:
                br = bp = 0.0          # 기준선을 못 잡으면 예전처럼 절대값으로 본다
            dr = abs(sr - br)
            dp = abs(sp - bp)
            worst = max(dr, dp)
            self.worst_att = max(self.worst_att, worst)
            if worst > self.a.att_tol:
                self.att_bad += 1
                if self.att_bad >= self.a.att_need:
                    reasons.append('자세가 최근 기준선에서 %.1f° 벗어났다 '
                                   '(롤 %.1f°, 피치 %.1f°, 기준선 롤 %+.1f° 피치 %+.1f°) '
                                   '— FAST-LIO 가 틀린 것이다' % (worst, dr, dp, br, bp))
            else:
                self.att_bad = max(0, self.att_bad - 1)

        if warming:
            self.last = (t, p.x, p.y, p.z)
            self.att_bad = 0
            self.att_hist.clear()     # 예열 중 값으로 기준선을 잡으면 안 된다
            self.pub.publish(String(data='ok=1 예열 중 (%.0f초 남음)'
                                    % (self.a.warmup - (now - self.t_first))))
            return

        if reasons:
            self.clean = 0
            self.pub.publish(String(data='ok=0 ' + ' | '.join(reasons)))
            if not self.bad_now:
                self.bad_now = True
                self.alarmed = True
                self.events += 1
                self.get_logger().error('★★ 오도메트리 이상 #%d ★★  %s'
                                        % (self.events, ' | '.join(reasons)))
                self.get_logger().error('   조사 결과 발산은 스스로 회복한 적이 없다. '
                                        '자세차가 계속 벌어지면 주행을 멈추고 다시 세울 것.')
        else:
            self.clean += 1
            # 회복 판정: 발산은 회복하지 않으므로, 깨끗해지면 그건 일시적 잡음이었다는 뜻이다.
            if self.bad_now and self.clean >= self.a.clear_need:
                self.bad_now = False
                self.get_logger().warn('이상 #%d 이 가라앉았다 (%d프레임 연속 정상) '
                                       '— 일시적이었던 것으로 본다'
                                       % (self.events, self.clean))
            self.pub.publish(String(data='ok=1' if not self.bad_now else 'ok=0 회복 확인 중'))

    def report(self):
        s = self.a.report_every
        att = ('자세차 최대 %.1f°' % self.worst_att) if self.have_imu else '자세차 (IMU 없음)'
        # rclpy 는 **같은 호출 위치**에서 심각도를 바꾸면 예외를 낸다.
        # 그래서 한 변수에 warn/info 를 담아 부르면 안 되고, 줄을 나눠야 한다.
        if self.t_first is None:
            state = '오도메트리 없음'
        elif time.time() - self.t_first < self.a.warmup:
            state = '예열 중'
        elif self.bad_now:
            state = '★ 지금 이상 (#%d)' % self.events
        elif self.events:
            state = '정상 (지난 이상 %d건, 가라앉음)' % self.events
        else:
            state = '정상'
        line = ('%.0f초 요약: 오도메트리 %.1f Hz   %s   최대 속도 %.2f m/s   최대 끊김 %.2f초   %s'
                % (s, self.n / s, att, self.worst_speed, self.worst_gap, state))
        if self.bad_now:
            self.get_logger().warn(line)
        else:
            self.get_logger().info(line)
        self.n = 0
        self.worst_att = self.worst_speed = self.worst_gap = 0.0



def parse_args(ap):
    """ROS 인자(--ros-args …)와 우리 인자를 갈라서 읽는다.

    argparse 는 --ros-args 를 모르는 인자로 보고 거절한다. 그래서 rclpy 의 유틸로
    ROS 인자를 먼저 걷어낸 뒤 나머지만 argparse 에 준다.
    ROS 인자는 rclpy.init(args=sys.argv) 로 그대로 넘겨야 use_sim_time 같은 게 먹는다.
    """
    from rclpy.utilities import remove_ros_args
    return ap.parse_args(remove_ros_args(sys.argv)[1:])

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--odom-topic', default='/Odometry')
    ap.add_argument('--lowstate-topic', default='/lowstate')
    ap.add_argument('--att-tol', type=float, default=8.0,
                    help='로봇 IMU 와 이 각도 이상 벌어지면 의심 (실측: 정상 1.4°, 발산 후 9.7~17.1°)')
    ap.add_argument('--att-need', type=int, default=5, help='이 횟수 연속으로 벌어져야 알린다')
    ap.add_argument('--att-win', type=float, default=10.0,
                    help='자세차 기준선을 잡는 창(초). 느린 드리프트는 기준선이 따라가 안 걸리고 '
                         '발산은 못 따라가 걸린다. 3~60초 전부 동작 확인 (실측 2026-09-21)')
    ap.add_argument('--att-min-n', type=int, default=20,
                    help='기준선을 쓰려면 창 안에 이만큼 표본이 있어야 한다. '
                         '모자라면 예전처럼 절대값으로 본다')
    ap.add_argument('--max-speed', type=float, default=4.0,
                    help='한 주기 이동량으로 계산한 속도의 상한 (Go2 최고 3.7 m/s)')
    ap.add_argument('--gap-warn', type=float, default=0.30,
                    help='오도메트리가 이만큼 끊기면 알린다 (실측: 1.55초 공백 뒤 영구 고착)')
    ap.add_argument('--warmup', type=float, default=15.0,
                    help='첫 오도메트리 이후 이만큼은 판정하지 않는다. FAST-LIO 초기화 구간이라 '
                         '자세도 출력 간격도 못 믿는다 (없으면 시작하자마자 오경보가 난다)')
    ap.add_argument('--clear-need', type=int, default=40,
                    help='이 횟수 연속 정상이면 경보를 내린다. 진짜 발산은 회복하지 않으므로 '
                         '회복했다면 일시적 잡음이었다는 뜻이다')
    ap.add_argument('--report-every', type=float, default=10.0)
    a = parse_args(ap)
    rclpy.init(args=sys.argv)
    n = OdometryMonitor(a)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass          # Ctrl-C 나 SIGTERM. 정상 종료로 본다
    n.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
