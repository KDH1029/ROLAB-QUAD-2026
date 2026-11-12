#!/usr/bin/env python3
"""전환 장치 v2 — FAST-LIO 가 이상하면 플래너 입력(위치 + 장애물 점구름)을 백업으로 바꾼다.
원본 ~/ws/v_gosu/backup/odom_mux_node.py 의 사본 (원본은 그대로). 다른 점은 아래 【v2】 둘뿐.

평소      /Odometry        → /Odometry_mux      /cloud_registered → /cloud_active
전환 후   /backup/odometry → /Odometry_mux      /backup/cloud     → /cloud_active
전환 조건 (backup_core.MuxLogic 그대로 — 녹화본 시험 통과: 정상 주행 오탐 0)
  · FAST-LIO 0.5초 무소식   · FAST-LIO 순간속도 2 m/s 초과   · 백업(건강 ok)과 0.5 m 넘게 1초 지속
  · 시작 5초는 판단 안 함.  한 번 전환하면 유지(경기 중 오락가락 금지) — 복귀는 /mux/reset 서비스로만
점구름은 원시 바이트로 받아 그대로 넘긴다(해석 비용 없음).
내는 것   /mux/status (std_msgs/String JSON, 유지형) — mode, reason, (v2) pending, hold_reason, backup_health

【v2 ① 백업 건강 확인 (11-04 검토 ③)】
  (F: 아래 ① 로 바뀜) 전환 조건이 걸려도 백업이 ok 가 아니면 전환하지 않고 FAST-LIO 를 유지 + 경고(2초마다).
    ok 아님 = /backup/status 의 health 가 init / degraded / lost, 또는 /backup/status 소식 없음(기본 3초, 백업 노드는 1 Hz),
              또는 /backup/odometry 소식 없음(기본 0.5초, 백업 노드는 50 Hz)
  전환 조건은 MuxLogic 그대로 '걸린 채' 남는다(보류). 보류 중 백업이 ok 가 되면 그때 전환한다 —
  원본은 걸리는 즉시 전환·유지였으므로, 바뀐 것은 '백업이 나쁠 때 나쁜 백업으로 넘어가지 않는다' 하나.
  보류 중 /mux/reset 을 부르면 보류도 지워진다.
【F 11-04 08:25 고침 (검증 회귀 (1)(3)(4), 메인 결정 a·c)】
  ① 막는 것은 백업 health ∈ {init, lost} 또는 /backup/status·/backup/odometry 끊김(상태없음/상태끊김/위치없음/위치끊김) 일 때만.
     'degraded' 는 허용 — FAST-LIO 가 죽었는데 막으면 /Odometry_mux 가 끊긴다(위치 정전). degraded 백업은 다리 오도로 위치를 계속 낸다.
     'aligning'(정렬 고정 전) 도 허용 — MuxLogic 이 'ok' 일 때만 '백업과 차이' 를 보므로 차이 조건만 빠지고 무소식·속도 조건은 바로 전환.
     (예전 v2: ok 아니면 전부 보류 → 고정 전 표류 61초, 점프 23.6초, FAST-LIO 사망 + degraded = 정전)
  ② 보류 해제: 보류 중 FAST-LIO 가 다시 정상(보류 뒤 새 메시지가 무소식 문턱 안, 순간속도 ≤ max_speed, 백업 ok 이고 차이 ≤ div_m)이면
     보류(무소식·속도 사유)를 지운다 — 회복된 지 29초 뒤 멀쩡한 FAST-LIO 를 버리던 문제. 판정 순서: 해제 → 전환.
     한 번 전환하면 유지(영구 전환)는 그대로. 해제 뒤 MuxLogic 은 다시 감시(시작 5초 유예는 이미 지남).
【v2 ② 종료】 Ctrl-C(launch 종료) 때 나던 ExternalShutdownException Traceback 을 잡는다.

판정 로직(HealthGuard, GuardedMux)은 ROS 와 떼어 두었다 → tests_d/test_mux_guard.py 로 오프라인 시험.
"""
import json, math, os, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(1, os.path.expanduser('~/ws/v_gosu/backup'))      # backup_core.MuxLogic (읽기만)
from backup_core import MuxLogic


class HealthGuard:
    """백업이 지금 넘겨받을 수 있는 상태인가. t 는 받은 시각(초)."""
    def __init__(self, status_timeout_s=3.0, odom_timeout_s=0.5):
        self.status_timeout_s, self.odom_timeout_s = status_timeout_s, odom_timeout_s
        self.health = None; self.t_stat = None; self.t_odom = None

    def on_status(self, t, health):
        self.health, self.t_stat = health, t

    def on_odom(self, t):
        self.t_odom = t

    def state(self, t):
        """백업이 보낸 health(ok/aligning/degraded/init/lost) 또는 상태없음, 상태끊김 x.x초, 위치없음, 위치끊김 x.x초.
        [F] degraded·aligning 도 위치 끊김을 먼저 본다(허용 상태라 위치가 실제로 와야 함). init/lost 는 그대로 그 값."""
        if self.t_stat is None:
            return '상태없음'
        if t - self.t_stat > self.status_timeout_s:
            return f'상태끊김 {t - self.t_stat:.1f}초'
        if self.health is None or self.health in ('init', 'lost'):
            return str(self.health)
        if self.t_odom is None:
            return '위치없음'
        if t - self.t_odom > self.odom_timeout_s:
            return f'위치끊김 {t - self.t_odom:.1f}초'
        return str(self.health)

    @staticmethod
    def blocks(h):
        """[F ①] 이 상태면 전환을 보류. init/lost 와 소식 없음·끊김만 막는다 (ok·degraded·aligning 은 허용)."""
        return h is None or h in ('init', 'lost') or h.startswith(('상태없음', '상태끊김', '위치없음', '위치끊김'))


class GuardedMux:
    """MuxLogic(전환 조건 그대로) + HealthGuard(백업이 ok 일 때만 실제 전환). 사건은 events 에 쌓인다 — 노드가 로그로 낸다."""
    def __init__(self, mux=None, guard=None, warn_period_s=2.0):
        self.mux = mux or MuxLogic(); self.guard = guard or HealthGuard()
        self.warn_period_s = warn_period_s
        self.mode = 'fastlio'; self.pending = None; self.t_pending = None; self.t_warn = None
        self.n_hold_warn = 0; self.events = []
        self.bk_xy = None; self.fl_speed = 0.0; self.fl_last = None; self.n_clear = 0

    def on_fastlio(self, t, x, y, z, ts=None):
        ts_ = t if ts is None else ts
        if self.fl_last is not None and ts_ - self.fl_last[0] > 1e-3:                  # [F ②] 보류 해제 판정용 순간속도 (MuxLogic 과 같은 정의)
            self.fl_speed = math.hypot(x - self.fl_last[1], y - self.fl_last[2]) / (ts_ - self.fl_last[0])
        self.fl_last = (ts_, x, y)
        self.mux.on_fastlio(t, x, y, z, ts=ts)
        return self.update(t)

    def tick(self, t, backup_xy):
        h = self.guard.state(t); self.bk_xy = backup_xy
        self.mux.tick(t, backup_xy, h)    # 차이 조건은 MuxLogic 안에서 'ok' 일 때만 본다(원본 그대로) — aligning·degraded 면 차이만 빠진다
        return self.update(t)

    def fl_recovered(self, t, h):
        """[F ②] 보류 뒤 FAST-LIO 가 다시 정상인가: 보류 뒤 새 메시지·무소식 문턱 안·순간속도 정상·백업 ok 이고 차이 ≤ div_m."""
        m = self.mux
        if h != 'ok' or self.bk_xy is None or m.t_fl is None or m.fl_prev is None or self.t_pending is None:
            return False
        if m.t_fl <= self.t_pending or t - m.t_fl > m.timeout_s or self.fl_speed > m.max_speed:
            return False
        return math.hypot(m.fl_prev[1] - self.bk_xy[0], m.fl_prev[2] - self.bk_xy[1]) <= m.div_m

    def update(self, t):
        if self.mode == 'backup' or self.mux.mode != 'backup':
            return self.mode
        h = self.guard.state(t)
        if self.pending is not None and self.fl_recovered(t, h):
            m = self.mux; d = math.hypot(m.fl_prev[1] - self.bk_xy[0], m.fl_prev[2] - self.bk_xy[1])
            self.events.append(('clear', f'보류 해제: FAST-LIO 회복(보류 {t - self.t_pending:.1f}초 뒤, 백업 ok, 차이 {d:.2f} m) — 전환 안 함, 사유였던 것: {self.pending}'))
            self.mux.reset(); self.pending = None; self.t_pending = None; self.n_clear += 1
            return self.mode
        if not self.guard.blocks(h):
            waited = 0.0 if self.t_pending is None else t - self.t_pending
            self.mode = 'backup'
            self.events.append(('switch', f'★ 전환 → backup: {self.mux.reason}' + (f' (백업 상태가 풀릴 때까지 {waited:.1f}초 보류했었음)' if self.pending else '') + f' (백업 상태 {h})'))
            self.pending = None
        elif self.pending is None:
            self.pending, self.t_pending, self.t_warn = self.mux.reason, t, t; self.n_hold_warn += 1
            self.events.append(('hold', f'전환 조건 걸림({self.mux.reason}) — 그러나 백업 상태 {h} → 전환 안 함, FAST-LIO 유지 (백업이 풀리면 전환, FAST-LIO 가 회복하면 해제)'))
        elif t - self.t_warn >= self.warn_period_s:
            self.t_warn = t; self.n_hold_warn += 1
            self.events.append(('hold', f'전환 보류 {t - self.t_pending:.0f}초째: {self.pending} / 백업 상태 {h} → FAST-LIO 유지'))
        return self.mode

    def reset(self):
        self.mux.reset(); self.mode = 'fastlio'; self.pending = None; self.t_pending = None

    def pop_events(self):
        ev, self.events = self.events, []
        return ev


def _ros_main():
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    class OdomMux(Node):
        def __init__(self, no_switch=False, status_timeout_s=3.0, odom_timeout_s=0.5):
            super().__init__('odom_mux')
            self.no_switch = no_switch      # 백업을 안 띄울 때: 중계만 하고 전환은 안 한다(원본과 같은 구성)
            rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.gm = GuardedMux(guard=HealthGuard(status_timeout_s, odom_timeout_s))
            self.bk_xy = None; self.mode = 'fastlio'; self.last_pending = None
            self.pub_odom = self.create_publisher(Odometry, '/Odometry_mux', rel)
            self.pub_cloud = self.create_publisher(PointCloud2, '/cloud_active', rel)
            self.pub_stat = self.create_publisher(String, '/mux/status', latched)
            self.create_subscription(Odometry, '/Odometry', self.on_fl, rel)
            self.create_subscription(Odometry, '/backup/odometry', self.on_bk, rel)
            self.create_subscription(String, '/backup/status', self.on_bk_stat, rel)
            self.create_subscription(PointCloud2, '/cloud_registered', lambda b: self.relay_cloud(b, 'fastlio'), rel, raw=True)
            self.create_subscription(PointCloud2, '/backup/cloud', lambda b: self.relay_cloud(b, 'backup'), rel, raw=True)
            self.create_service(Trigger, '/mux/reset', self.on_reset)
            self.create_timer(0.01, self.tick)
            self.publish_status()
            self.get_logger().info(f'odom_mux v2 — 백업 ok 일 때만 전환 (상태 {status_timeout_s:.1f}초 / 위치 {odom_timeout_s:.1f}초 무소식이면 ok 아님)'
                                   + (' [--no-switch: 중계만]' if no_switch else ''))

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def on_fl(self, m):
            p = m.pose.pose.position
            if not self.no_switch:
                self.gm.on_fastlio(self.now(), p.x, p.y, p.z, ts=m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
                self.check()
            if self.mode == 'fastlio':
                self.pub_odom.publish(m)

        def on_bk(self, m):
            self.bk_xy = (m.pose.pose.position.x, m.pose.pose.position.y)
            self.gm.guard.on_odom(self.now())
            if self.mode == 'backup':
                m.header.stamp = self.get_clock().now().to_msg(); self.pub_odom.publish(m)

        def on_bk_stat(self, m):
            try: h = json.loads(m.data).get('health', 'init')
            except Exception: return
            self.gm.guard.on_status(self.now(), h)

        def relay_cloud(self, raw, src):
            if src == self.mode:
                self.pub_cloud.publish(raw)

        def tick(self):
            if self.no_switch: return
            self.gm.tick(self.now(), self.bk_xy); self.check()

        def check(self):
            for kind, msg in self.gm.pop_events():
                self.get_logger().warn(msg)
            changed = self.gm.mode != self.mode or self.gm.pending != self.last_pending
            self.mode = self.gm.mode; self.last_pending = self.gm.pending
            if changed:
                self.publish_status()

        def publish_status(self):
            g = self.gm
            self.pub_stat.publish(String(data=json.dumps(dict(mode=self.mode, reason=g.mux.reason if self.mode == 'backup' else '',
                                                              pending=g.pending is not None, hold_reason=g.pending or '',
                                                              backup_health=g.guard.state(self.now())), ensure_ascii=False)))

        def on_reset(self, req, res):
            self.gm.reset(); self.mode = 'fastlio'; self.last_pending = None; self.publish_status()
            self.get_logger().warn('수동 복귀 → fastlio (보류도 지움)'); res.success = True; res.message = 'fastlio'; return res

    argv = list(sys.argv)
    no_switch = '--no-switch' in argv
    def _opt(name, default):
        if name in argv:
            i = argv.index(name); v = float(argv[i + 1]); del argv[i:i + 2]; return v
        return default
    st = _opt('--status-timeout', 3.0); ot = _opt('--odom-timeout', 0.5)
    rclpy.init(args=[a for a in argv if a != '--no-switch']); n = OdomMux(no_switch, st, ot)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass                                      # Ctrl-C / launch 종료 — 정상 종료 (v1 의 끝 Traceback 원인)
    finally:
        try: n.destroy_node()
        except Exception: pass
        try: rclpy.try_shutdown()
        except Exception: pass


if __name__ == '__main__':
    _ros_main()
