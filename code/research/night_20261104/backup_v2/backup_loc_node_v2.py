#!/usr/bin/env python3
"""백업 위치추정 노드 v2 (GICP + 견고 지면표, night_20261104/backup_v2) — 원본 v_gosu/backup/backup_loc_node.py 사본. FAST-LIO 없이 '다리 오도메트리 + 3D 지도 정합(수평 세우기)'.

받는 것   /utlidar/robot_odom (다리 기구학 150 Hz)   /lidar_points (XT16 원본, 원시 바이트로 받아 5 Hz 분만 해석)
          /Odometry (FAST-LIO) — 시작 자세(씨앗) + 좌표 정렬 쌍(frame_align). 없으면 원점·항등
          /mux/status (전환 장치, 유지형) — mode=backup 이 되면 정렬을 그 자리에서 고정
내는 것   /backup/odometry   nav_msgs/Odometry  (오늘 camera_init 좌표 = T_cam_map 적용, 다리 갱신마다 50 Hz)
          /backup/cloud      sensor_msgs/PointCloud2 (오늘 camera_init 좌표 스캔, 정합마다 5 Hz) — 전환 시 플래너 장애물
          /backup/status     std_msgs/String (JSON: health ok/degraded/lost/aligning, rms, 수락/거부, align{state,T,res,pairs})
11-04 07:40 고침 (차단 버그: 사전 지도 좌표 ≠ 오늘 camera_init)
  · 추정기 내부는 '지도 만든 날 camera_init'(사전 지도 좌표) 그대로. 내보내는 값만 frame_align.FrameAligner 의 T_cam_map 으로 바꾼다
  · 정렬 고정 전엔 항등 + health 를 'aligning' 으로 낸다(--no-align-mask 로 끔) — 전환 장치(MuxLogic)는 health=='ok' 일 때만
    '백업과 0.5 m·1초 차이' 를 보므로, 아직 안 맞춘 백업 때문에 멀쩡한 FAST-LIO 를 버리지 않는다(무소식·속도 판정은 그대로 산다)
    가림은 init 뒤 20초까지만(FrameAligner.mask_max_s, [F] 60→20, --align-mask-max 로 바꿈), FAST-LIO 가 0.5초 넘게 끊기면 바로 풂(전환 장치 v2 가 넘겨받게)
  · FAST-LIO 가 처음부터 없으면 항등 고정 + 경고. 콜백 예외는 잡아 로그만 남긴다(노드가 죽지 않게)
11-04 08:25 고침 [F] (검증 회귀 (2)(5), 메인 결정 b·d)
  · /mux/status 의 pending=true(전환 조건 걸림·보류) 도 mode=backup 처럼 '이상 감지' → 정렬 즉시 고정 (FrameAligner.on_anomaly)
    — 점프 뒤 쌍으로 틀린 T 를 고정하던 문제. init 전에 온 유지형 메시지도 기억(정렬기가 init 순간 항등 고정)
  · FrameAligner 가 FAST-LIO 점프를 스스로 보고 고정하면 그 자리에서 상태 발행. 정렬 실패('align_failed')면 가림 풀림 + 경고 로그
  · /backup/status 에 gicp={n_low_inlier, last_inlier} (gicp_matcher 의 비율 판정 거부 수·직전 비율 — 속성이 있을 때만)
계산은 backup_core_v2.make_estimator — split_validate_gicp_var.py 로 녹화본 검증한 것과 같은 구성. 정렬 검증 tests/test_frame_align.py
실행: 야간 venv 파이썬(small_gicp)  ~/ws/night_20261104/venv/bin/python backup_loc_node_v2.py --map ...
사용: backup_loc_node.py --map <지도.npz|.pcd> [--hz 5] [--init fastlio|origin] [--no-align] [--no-align-mask] [--align-mask-max 20]
"""
import argparse, json, math, threading, time, sys, os, traceback, functools
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
import sensor_msgs_py.point_cloud2 as pc2
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from backup_core_v2 import make_estimator, load_map, q2R, rpy_of, match_correction
from frame_align import FrameAligner


def safe_cb(fn):
    """콜백 예외를 잡아 로그만 남긴다 — 노드가 죽으면 전환 장치가 백업을 못 쓴다. 같은 콜백 로그는 2초에 한 번(첫 번은 전체 Traceback)."""
    @functools.wraps(fn)
    def w(self, *a, **k):
        try:
            return fn(self, *a, **k)
        except Exception as e:      # noqa: BLE001
            self.n_exc[fn.__name__] = self.n_exc.get(fn.__name__, 0) + 1
            now = time.monotonic(); last = self.t_exc_log.get(fn.__name__)
            if last is None or now - last > 2.0:
                self.t_exc_log[fn.__name__] = now
                tb = traceback.format_exc() if last is None else f'{type(e).__name__}: {e}'
                try: self.get_logger().error(f'콜백 {fn.__name__} 예외 (누적 {self.n_exc[fn.__name__]}회, 노드는 계속): {tb}')
                except Exception: pass
            return None
    return w


class BackupLoc(Node):
    def __init__(self, a):
        super().__init__('backup_localization')
        self.n_exc, self.t_exc_log = {}, {}
        t0 = time.time()
        self.est = make_estimator(load_map(os.path.expanduser(a.map)))
        self.get_logger().info(f'지도 {a.map} 적재 ({time.time() - t0:.1f}s), 정합 {a.hz} Hz, 시작 자세 {a.init}, 좌표 정렬 {"끔" if a.no_align else "켬"}')
        self.hz, self.init_mode, self.frame = a.hz, a.init, a.map_frame
        self.align_on, self.align_mask = not a.no_align, not a.no_align_mask
        self.al = FrameAligner(mask_max_s=a.align_mask_max)
        if not self.align_on:
            self.al.state, self.al.why = 'identity_frozen', '--no-align (정렬 끔: 지도 좌표 = camera_init 가정)'
        self.lock = threading.Lock()
        self.leg = None; self.fl0 = None; self.t_first_leg = None
        self.last_scan = 0.0; self.busy = False; self.n_leg = 0
        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        g_leg, g_scan = MutuallyExclusiveCallbackGroup(), MutuallyExclusiveCallbackGroup()
        self.create_subscription(Odometry, '/utlidar/robot_odom', self.on_leg, qos_profile_sensor_data, callback_group=g_leg)
        self.create_subscription(PointCloud2, a.cloud, self.on_cloud, qos_profile_sensor_data, callback_group=g_scan, raw=True)
        self.create_subscription(Odometry, '/Odometry', self.on_fastlio, rel, callback_group=g_leg)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)          # 전환 장치 /mux/status 는 유지형
        self.create_subscription(String, '/mux/status', self.on_mux_status, latched, callback_group=g_leg)
        self.pub_odom = self.create_publisher(Odometry, '/backup/odometry', rel)
        self.pub_cloud = self.create_publisher(PointCloud2, '/backup/cloud', rel)
        self.pub_stat = self.create_publisher(String, '/backup/status', rel)
        self.create_timer(1.0, self.report, callback_group=g_leg)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @safe_cb
    def on_fastlio(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = rpy_of(q2R(q.x, q.y, q.z, q.w))[2]
        if self.fl0 is None:
            self.fl0 = (p.x, p.y, p.z, yaw)
        if self.align_on:
            st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            with self.lock:
                amsg = self.al.on_fastlio(self.now_s(), p.x, p.y, yaw, stamp=st)    # [F] 점프를 보고 고정하면 설명이 돌아온다
            if amsg:
                self.publish_status(self.now_s(), amsg)

    @safe_cb
    def on_mux_status(self, m):
        """[F] mode=backup 또는 pending=true(전환 조건 걸림) → 정렬 즉시 고정. init 전이면 정렬기가 기억했다가 init 때 항등 고정."""
        if not self.align_on:
            return
        d = json.loads(m.data)
        if d.get('mode') == 'backup':
            why = f"전환 장치가 백업 사용 시작({d.get('reason', '')})"
        elif d.get('pending'):
            why = f"전환 장치 이상 감지·보류({d.get('hold_reason', '')})"
        else:
            return
        with self.lock:
            msg = self.al.on_anomaly(self.now_s(), why)
        if msg:
            self.publish_status(self.now_s(), msg)

    @safe_cb
    def on_leg(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        r, pi, y = rpy_of(q2R(q.x, q.y, q.z, q.w)); leg = (p.x, p.y, p.z, y, r, pi)
        now = self.get_clock().now(); ts = now.nanoseconds * 1e-9
        with self.lock:
            self.leg = leg
            if not self.est.inited:
                # FAST-LIO 첫 자세를 첫 다리 메시지부터 5초 기다린다(재생·실기 모두 같은 기준). 없으면 원점
                self.t_first_leg = self.t_first_leg or time.monotonic()
                waited = time.monotonic() - self.t_first_leg
                if self.init_mode == 'fastlio' and self.fl0 is not None:
                    self.est.init(*self.fl0, leg, ts); amsg = self.al.on_init(ts); self.get_logger().info(f'시작 자세 = FAST-LIO {self.fl0} (지도 좌표 씨앗)')
                elif self.init_mode == 'origin' or waited > 5.0:
                    self.est.init(0.0, 0.0, 0.0, 0.0, leg, ts); amsg = self.al.on_init(ts); self.get_logger().info('시작 자세 = 원점')
                else:
                    amsg = None
                if amsg:
                    self.get_logger().warn(f'좌표 정렬 {self.al.state}: {amsg}')
                return
            self.est.on_leg(leg)
            x, y, z, rr, pp, yy = self.est.pose()
            self.al.on_backup_pose(ts, x, y, yy)
            x, y, yy = self.al.map_to_cam(x, y, yy)    # 지도 → 오늘 camera_init (고정 전 항등)
        self.n_leg += 1
        if self.n_leg % 3: return                      # 150 Hz → 50 Hz 발행
        o = Odometry(); o.header.stamp = now.to_msg(); o.header.frame_id = self.frame; o.child_frame_id = 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = x, y, z
        cr, sr, cp, sp, cy, sy = math.cos(rr / 2), math.sin(rr / 2), math.cos(pp / 2), math.sin(pp / 2), math.cos(yy / 2), math.sin(yy / 2)
        o.pose.pose.orientation.w = cr * cp * cy + sr * sp * sy; o.pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
        o.pose.pose.orientation.y = cr * sp * cy + sr * cp * sy; o.pose.pose.orientation.z = cr * cp * sy - sr * sp * cy
        o.twist = m.twist                              # 다리 오도메트리 속도(몸체 좌표) 그대로 — z축 회전이라 몸체 속도는 안 바뀐다
        self.pub_odom.publish(o)

    @safe_cb
    def on_cloud(self, raw):
        now = time.monotonic()
        if now - self.last_scan < 0.95 / self.hz or not self.est.inited:
            return                                       # 원시 바이트만 받고 버림 — 해석 비용 없음
        self.last_scan = now
        m = deserialize_message(raw, PointCloud2)
        pts = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)
        with self.lock:
            prep = self.est.prepare(pts)
        if prep is None: return
        T, rms = self.est.matcher.align(prep['P'], prep['T0'])      # 잠금 밖 — 다리 갱신은 계속 들어온다
        cm, cr = match_correction(prep, T)
        t = self.get_clock().now(); ts = t.nanoseconds * 1e-9
        with self.lock:
            res = self.est.apply(prep, T, rms, ts)
            self.al.on_match(ts, res['accepted'], cm, cr)
            self.al.on_backup_pose(ts, self.est.x, self.est.y, self.est.yaw)
            amsg = self.al.tick(ts) if self.align_on else None     # 5 Hz 로도 판단 — FAST-LIO 가 끊기면 0.5~0.7 s 안에 고정
            w = self.al.points_to_cam(res['world']).astype(np.float32)
        if amsg:
            self.publish_status(ts, amsg)                          # 고정 순간 바로 알림 (전환 장치 v2 는 health ok 를 기다린다)
        c = PointCloud2(); c.header.stamp = t.to_msg(); c.header.frame_id = self.frame
        c.height, c.width = 1, len(w); c.point_step, c.row_step = 12, 12 * len(w); c.is_dense = True
        c.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyz')]
        c.data = w.tobytes(); self.pub_cloud.publish(c)

    @safe_cb
    def report(self):
        t = self.now_s()
        with self.lock:
            msg = self.al.tick(t) if (self.align_on and self.est.inited) else None
        self.publish_status(t, msg)

    def publish_status(self, t, msg=None):
        with self.lock:
            e = self.est; mx, my, z, _, _, myaw = e.pose()
            x, y, yy = self.al.map_to_cam(mx, my, myaw)
            h = e.health(t)
            hp = 'aligning' if (self.align_mask and self.align_on and h == 'ok' and self.al.mask_health(t)) else h
            s = dict(health=hp, health_raw=h, rms=None if not np.isfinite(e.last_rms) else round(e.last_rms, 3), ok=e.n_ok, bad=e.n_bad,
                     x=round(x, 3), y=round(y, 3), z=round(z, 3), yaw_deg=round(math.degrees(yy), 1),
                     map=dict(x=round(mx, 3), y=round(my, 3), yaw_deg=round(math.degrees(myaw), 1)),
                     since_good_s=None if e.t_good is None else round(t - e.t_good, 1), align=self.al.status(t))
            warn = self.al.warn
            mt = getattr(e, 'matcher', None)                # [F] gicp_matcher 진단 (속성 있을 때만)
            if mt is not None and hasattr(mt, 'n_low_inlier'):
                li = getattr(mt, 'last_inlier', float('nan'))
                s['gicp'] = dict(n_low_inlier=int(mt.n_low_inlier), last_inlier=None if not np.isfinite(li) else round(float(li), 3))
        if self.n_exc:
            s['exceptions'] = dict(self.n_exc)
        if msg:
            (self.get_logger().warn if warn == msg else self.get_logger().info)(f'좌표 정렬 {self.al.state}: {msg} → T_cam_map {s["align"]["T"]} 잔차 {s["align"].get("res")}')
        self.pub_stat.publish(String(data=json.dumps(s, ensure_ascii=False)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True); ap.add_argument('--hz', type=float, default=5.0)
    ap.add_argument('--init', choices=['fastlio', 'origin'], default='fastlio')
    ap.add_argument('--cloud', default='/lidar_points'); ap.add_argument('--map-frame', default='camera_init')
    ap.add_argument('--no-align', action='store_true', help='좌표 정렬 끔 (지도 좌표를 그대로 camera_init 으로 냄 — 예전 동작)')
    ap.add_argument('--no-align-mask', action='store_true', help="정렬 고정 전에도 health 를 'ok' 로 냄 (전환 장치 차이 판정이 바로 살아남)")
    ap.add_argument('--align-mask-max', type=float, default=20.0, help="[F] 'aligning' 가림 최대 초 (init 기준, 예전 60)")
    a, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args); n = BackupLoc(a)
    ex = MultiThreadedExecutor(num_threads=3); ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        try: rclpy.shutdown()
        except Exception: pass


if __name__ == '__main__':
    main()
