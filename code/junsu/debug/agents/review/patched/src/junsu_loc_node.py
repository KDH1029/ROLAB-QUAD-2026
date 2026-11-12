#!/usr/bin/env python3
"""junsu 위치추정: 출발 ICP + AMCL 감시 + 어긋나면 ICP 재정렬 (11-05, merge-review 브랜치).

주행 좌표는 FAST-LIO(camera_init) 그대로다. 이 노드는 그 위에 '지도와 맞는가'를 지킨다.
  1) /tf_junsu 트리(실제 /tf 는 안 건드림 — body 부모가 둘이라 merge 노드들이 못 씀):
       map →(T: 지도 = T·FAST-LIO)→ lio_odom →(/Odometry)→ base_link →(/tf 의 base_link→base_stab)→ base_stab
       정적: base_link→hesai_lidar·imu_link·body (/tf_static 에서 복사)
     merge AMCL(/junsu/amcl, TF 방송 안 함)·콘 추적기(cone_tracker_node)가 이 트리를 본다.
  2) T: 출발 정렬 결과 파일(<경로>.align.json, go2_ctl 의 r 이 씀)이 새로 생기면 읽고 /mission/align_T 로 낸다.
     AMCL 초기자세 = T·지금 FAST-LIO (공분산 0.05 m / 3°). 서 있어 AMCL 이 안 내면 request_nomotion_update.
  3) 감시: tools/amcl_watch_core.Watch (review amcl3 재생으로 정한 문턱) → /mission/loc_watch (JSON).
     정렬이 거부됐으면(T=0) 감시는 기록만(STOP 안 냄 — T=0 이면 정상에서도 오탐, 인계 문서).
  4) STOP 이면 최근 2초 스캔으로 지금 T 둘레 ICP(start_align_core.align ±0.30 m / ±10°):
       맞음 + 보정량 큼 → FAST-LIO 가 흘렀다 → T 갱신(/mission/align_T, 정적 TF 다시) — 러너가 남은 목표를 다시 계산
       맞음 + 보정량 작음 → AMCL 이 틀렸다 → AMCL 을 T·FAST-LIO 로 다시 초기화
       ICP 실패 → STOP 유지(러너가 그 자리에 섬). 어느 경우든 감시 초기화는 맞았을 때만.
  5) ★ 11-06 자세 보호: 엎드림·일어남·넘어짐(몸 높이 < posture_low_h, 또는 러너가 PARK_/FALL 상태) 동안은
       스캔을 버리고(재정렬 재료에서 뺌) 감시·ICP 를 멈춘다(상태 POSTURE_HOLD — 러너는 서지 않음).
       다시 선 뒤 posture_recover_s 동안 선 스캔을 모은 다음 AMCL 을 T·FAST-LIO 로 다시 초기화하고 감시 재개.
       그 사이 온 재정렬 요청(러너: 주차 뒤)은 미뤘다가 그때 처리 — 기울어진 스캔으로 T 를 틀리게 바꾸지 않게.
"""
import json
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty
from tf2_msgs.msg import TFMessage
try:
    from unitree_go.msg import SportModeState
except ImportError:      # 재생 환경에 unitree 메시지가 없으면 몸 높이 없이(러너 상태만으로) 판단
    SportModeState = None

HERE = os.path.dirname(os.path.abspath(__file__))
J = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(J, 'tools')); sys.path.insert(0, os.path.join(J, 'tools', 'start_align'))
import amcl_watch_core as wc  # noqa: E402
import start_align_core as sa  # noqa: E402


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def tf_msg(parent, child, stamp, x, y, z, qx, qy, qz, qw):
    t = TransformStamped(); t.header.frame_id = parent; t.child_frame_id = child; t.header.stamp = stamp
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = float(x), float(y), float(z)
    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = qx, qy, qz, qw
    return t


def small(dT, xy=0.05, yaw=math.radians(1.0)):
    return math.hypot(dT[0], dT[1]) < xy and abs(dT[2]) < yaw


class Loc(Node):
    def __init__(self):
        super().__init__('junsu_loc')
        p = self.declare_parameter
        self.align_file = os.path.expanduser(p('align_file', os.path.join(J, 'maps', 'segments', 'logs', 'route_today.json.align.json')).value)
        self.map_yaml = os.path.expanduser(p('map_yaml', os.path.join(J, 'maps', 'align', 'arena1459_nocone_slalomfree.yaml')).value)   # 11-05: 슬라럼 복도 안 비움
        self.reanchor = bool(p('reanchor', True).value)
        self.low_h = float(p('posture_low_h', 0.22).value)            # ★ 11-06: 몸 높이 이 아래 = 엎드림·넘어짐(선 높이 ~0.31)
        self.recover_s = float(p('posture_recover_s', 1.5).value)     # 다시 선 뒤 이만큼 선 스캔을 모으고 감시 재개
        self.posture_on = bool(p('posture_gate', True).value)
        self.body_h = None; self.runner_pose_state = False; self.posture_bad = False; self.posture_ok_t = None; self.pending_realign = None
        self.T = (0.0, 0.0, 0.0); self.aligned = False; self.align_mtime = None
        self.watch = None; self.state = 'NO_ALIGN'; self.last = {}
        self.lio = None; self.lio_t = None; self.amcl = None; self.amcl_t = None; self.sigma = None
        self.statics = {}          # child → TransformStamped (base_link→*)
        self.scans = []            # [(t, pts Nx2, lio pose)] 최근 2.5초
        self.icp_busy = False; self.icp_last = -1e9; self.icp_info = {}
        self.wm = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)
        self.pub_tf = self.create_publisher(TFMessage, '/tf_junsu', 50)
        self.pub_tfs = self.create_publisher(TFMessage, '/tf_junsu_static', latched)
        self.pub_T = self.create_publisher(String, '/mission/align_T', latched)
        self.pub_watch = self.create_publisher(String, '/mission/loc_watch', 5)
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped, '/junsu/initialpose', 5)
        self.cli_nomo = self.create_client(Empty, '/junsu/request_nomotion_update')
        self.create_subscription(Odometry, '/Odometry', self.cb_odom, 20)
        self.create_subscription(TFMessage, '/tf_static', self.cb_tf_static, latched)
        self.create_subscription(TFMessage, '/tf', self.cb_tf, 50)
        self.create_subscription(LaserScan, '/scan', self.cb_scan, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/junsu/amcl_pose', self.cb_amcl, 5)
        self.create_subscription(String, '/mission/realign', self.cb_realign, 2)   # ★ 러너: 주차에서 일어난 뒤 넓은 범위 재정렬
        if SportModeState is not None:
            self.create_subscription(SportModeState, p('sport_topic', '/lf/sportmodestate').value, self.cb_sport, qos_profile_sensor_data)
        self.create_subscription(String, '/mission/runner_status', self.cb_runner, 5)
        self.create_timer(1.0, self.poll_align)
        self.create_timer(0.1, self.tick)
        self.nomo_t = 0.0
        self.t_start = time.time()   # 이보다 오래된 정렬 파일은 안 읽음(지난 FAST-LIO 의 T)
        self.get_logger().info(f'위치 감시 시작 — 정렬 파일 {self.align_file}, ICP 지도 {self.map_yaml}, 재정렬 {"켬" if self.reanchor else "끔"}')

    # ── TF 트리
    def cb_tf_static(self, m):
        for t in m.transforms:
            if t.header.frame_id == 'base_link':
                self.statics[t.child_frame_id] = t
        self.publish_statics()

    def publish_statics(self):
        x, y, th = self.T
        now = self.get_clock().now().to_msg()
        ts = [tf_msg('map', 'lio_odom', now, x, y, 0.0, 0.0, 0.0, math.sin(th / 2), math.cos(th / 2))]
        ts += list(self.statics.values())
        self.pub_tfs.publish(TFMessage(transforms=ts))

    def cb_tf(self, m):
        out = [t for t in m.transforms if t.header.frame_id == 'base_link' and t.child_frame_id == 'base_stab']
        if out:
            self.pub_tf.publish(TFMessage(transforms=out))

    def cb_odom(self, m):
        q = m.pose.pose.orientation; pp = m.pose.pose.position
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.lio_t is not None and (t - self.lio_t > 2.0 or t < self.lio_t - 0.5) and self.aligned:
            # FAST-LIO 가 다시 켜짐(좌표 원점이 바뀜) → 지금 T 는 무효. r 로 다시 정렬해야 함
            self.aligned = False; self.watch = None
            self.set_T((0.0, 0.0, 0.0), 'fastlio_restarted')
            self.get_logger().error('FAST-LIO 다시 켜짐 감지 — 정렬 무효(r 로 다시 출발해야 함)')
        self.lio = (pp.x, pp.y, yaw_of(q)); self.lio_t = t
        # base_link→body 는 항등(11-05 /tf_static 확인) — FAST-LIO body 자세 = base_link 자세
        self.pub_tf.publish(TFMessage(transforms=[tf_msg('lio_odom', 'base_link', m.header.stamp, pp.x, pp.y, pp.z, q.x, q.y, q.z, q.w)]))

    def cb_scan(self, m):
        if self.lio is None:
            return
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.posture_bad and self.posture_ok_t is None:
            return                     # ★ 11-06: 엎드림·넘어짐 중 스캔은 버림(다시 선 뒤 스캔만 재정렬 재료로)
        pts = sa.scan_points(m.ranges, m.angle_min, m.angle_increment)
        self.scans.append((t, pts, self.lio))
        while self.scans and t - self.scans[0][0] > 2.5:
            self.scans.pop(0)

    def cb_amcl(self, m):
        q = m.pose.pose.orientation; pp = m.pose.pose.position; c = m.pose.covariance
        self.amcl = (pp.x, pp.y, yaw_of(q)); self.amcl_t = self.lio_t if self.lio_t is not None else time.time()
        self.sigma = math.sqrt(max(c[0], 0.0) + max(c[7], 0.0))
        if self.watch is not None and self.lio is not None and not self.posture_bad:
            self.watch.update(self.amcl_t, self.lio, self.amcl, self.sigma)   # 새 AMCL 자세가 올 때만(끊김 판정이 살게)

    # ── ★ 11-06 자세 보호
    def cb_sport(self, m):
        if m.body_height > 0.0:
            self.body_h = float(m.body_height)

    def cb_runner(self, m):
        try:
            st = str(json.loads(m.data).get('state', ''))
        except ValueError:
            return
        # ★ review: 'FALL' 앞부분이면 FALL_REALIGN(다시 선 뒤 러너가 재정렬 응답을 기다리는 상태)까지 '자세 낮음'으로 봐서
        #   재정렬을 러너가 포기(6초)할 때까지 미뤘다가, 걷는 중에 ±0.6 m/±45° 로 돌렸다. 넘어져 있는 동안(FALLEN)만.
        self.runner_pose_state = st.startswith('PARK_SIT') or st in ('PARK_HOLD', 'PARK_RISE_WAIT', 'FALLEN')

    def holding(self):
        """자세가 낮거나 막 다시 선 중(선 스캔 모으는 중)이면 True. 바뀌는 순간 처리(스캔 비움 · AMCL 다시 초기화 · 미룬 재정렬)."""
        if not self.posture_on or self.lio_t is None:
            return False
        bad = (self.body_h is not None and self.body_h < self.low_h) or self.runner_pose_state
        now = self.lio_t
        if bad:
            if not self.posture_bad:
                self.posture_bad = True; self.scans = []
                self.get_logger().warn(f'자세 낮음(몸 높이 {self.body_h}) — 위치 감시·재정렬 멈춤, 스캔 버림')
            self.posture_ok_t = None
            return True
        if self.posture_bad:
            if self.posture_ok_t is None:
                self.posture_ok_t = now
            if now - self.posture_ok_t < self.recover_s:
                return True
            self.posture_bad = False
            self.get_logger().warn(f'다시 섬({self.recover_s:.1f}초 선 스캔 {len(self.scans)}개) — AMCL 을 T·FAST-LIO 로 다시 초기화, 감시 재개')
            if self.aligned:
                self.init_amcl()
            if self.pending_realign is not None:
                d = self.pending_realign; self.pending_realign = None
                self.get_logger().warn('미뤄 둔 재정렬 요청 처리(선 스캔으로)')
                self.start_realign(d)
        return False

    # ── 정렬(T)
    def poll_align(self):
        try:
            mt = os.path.getmtime(self.align_file)
        except OSError:
            return
        if mt == self.align_mtime or mt < self.t_start:
            return
        self.align_mtime = mt
        try:
            side = json.load(open(self.align_file))
        except (OSError, ValueError):
            return
        self.aligned = bool(side.get('ok'))
        self.set_T(tuple(float(v) for v in side.get('T', (0, 0, 0))) if self.aligned else (0.0, 0.0, 0.0),
                   'start' if self.aligned else 'start_rejected', side.get('match'))
        self.init_amcl()

    def set_T(self, T, reason, match=None):
        self.T = T
        self.publish_statics()
        self.pub_T.publish(String(data=json.dumps({'T': list(T), 'ok': self.aligned, 'reason': reason, 'match': match})))
        if self.lio_t is not None:
            self.watch = wc.Watch(T, t_init=self.lio_t)
        self.get_logger().warn(f'T = ({T[0]:+.3f}, {T[1]:+.3f}, {math.degrees(T[2]):+.2f}°) [{reason}]')

    def init_amcl(self):
        if self.lio is None:
            return
        x, y, th = wc.compose(self.T, self.lio)
        m = PoseWithCovarianceStamped(); m.header.frame_id = 'map'; m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = float(x), float(y)
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
        cov = [0.0] * 36; cov[0] = cov[7] = 0.05 ** 2; cov[35] = math.radians(3.0) ** 2
        m.pose.covariance = cov
        for _ in range(3):
            self.pub_init.publish(m)
        self.amcl = None; self.amcl_t = None
        if self.lio_t is not None:
            self.watch = wc.Watch(self.T, t_init=self.lio_t)
        self.get_logger().info(f'AMCL 초기자세 = T·FAST-LIO = ({x:.2f}, {y:.2f}, {math.degrees(th):.1f}°)')

    # ── 감시·재정렬
    def tick(self):
        if self.lio_t is None:
            return
        now = self.lio_t
        if self.holding():             # ★ 11-06: 엎드림·일어남 중 — 감시·ICP·AMCL 갱신 요청 모두 멈춤(러너는 이 상태로 서지 않음)
            out = dict(state='POSTURE_HOLD', body_h=self.body_h, T=[round(v, 4) for v in self.T], aligned=self.aligned, icp=self.icp_info)
            self.state = 'POSTURE_HOLD'; self.pub_watch.publish(String(data=json.dumps(out, ensure_ascii=False)))
            return
        stale = self.amcl_t is None or now - self.amcl_t > 0.5      # 서 있으면 AMCL 이 안 냄(초기화 직후 포함) → 갱신 요청
        if self.watch is not None and stale and time.monotonic() - self.nomo_t > 0.5 and self.cli_nomo.service_is_ready():
            self.cli_nomo.call_async(Empty.Request()); self.nomo_t = time.monotonic()
        if self.watch is None:
            st = 'NO_ALIGN'
        else:
            st = self.watch.judge(now)
            if not self.aligned and st in ('WARN', 'STOP'):
                st = 'UNALIGNED_' + st          # 정렬 거부(T=0) — 기록만, 러너는 안 섬
        if st == 'STOP' and self.reanchor and not self.icp_busy and time.monotonic() - self.icp_last > 3.0:
            self.icp_busy = True; self.icp_last = time.monotonic()
            threading.Thread(target=self.reanchor_icp, args=(list(self.scans), self.T), daemon=True).start()
        self.state = st
        info = self.watch.status() if self.watch is not None else {}
        out = {k: (round(v, 3) if isinstance(v, float) else v) for k, v in info.items()}
        out.update(state=st, T=[round(v, 4) for v in self.T], aligned=self.aligned, icp=self.icp_info)
        self.pub_watch.publish(String(data=json.dumps(out, ensure_ascii=False)))

    def cb_realign(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            d = {}
        if self.icp_busy or not self.aligned:
            self.get_logger().warn(f'재정렬 요청 무시({"ICP 중" if self.icp_busy else "출발 정렬 안 됨"})'); return
        if self.holding():             # ★ 11-06: 아직 엎드렸거나 막 일어나는 중 — 선 스캔이 모이면 그때
            self.pending_realign = d; self.get_logger().warn('재정렬 요청 — 자세 회복 뒤로 미룸'); return
        self.start_realign(d)

    def start_realign(self, d):
        if self.icp_busy:
            return
        self.icp_busy = True; self.icp_last = time.monotonic()
        threading.Thread(target=self.reanchor_icp, args=(list(self.scans), self.T, float(d.get('max_xy', 0.5)),
                                                         float(d.get('max_yaw', 20.0)), 'realign'), daemon=True).start()

    def reanchor_icp(self, scans, T0, max_xy=0.45, max_yaw=15.0, why='reanchor'):
        try:
            if self.wm is None:
                self.wm = sa.WallMap(self.map_yaml)
            pairs = [(pts, wc.compose(T0, pose)) for _, pts, pose in scans[-40:] if len(pts) > 20]
            if len(pairs) < 5:
                self.icp_info = {'ok': False, 'reason': f'스캔 {len(pairs)}장뿐'}; return
            r = sa.align(self.wm, pairs, max_xy=max_xy, max_yaw_deg=max_yaw)   # 11-05 실주행: ±0.3/±10° 로는 주차 뒤 틀어짐을 못 잡음
            dT = tuple(r['T'])
            self.icp_info = {'ok': bool(r['ok']), 'dT': [round(v, 4) for v in dT], 'match': round(float(r['match']), 3), 'reason': r.get('reason', '')}
            if not r['ok']:
                self.get_logger().error(f'재정렬 ICP 실패({r.get("reason")}) — STOP 유지(러너 정지)')
                return
            if why == 'realign':
                Tn = wc.compose(dT, T0)
                self.get_logger().warn(f'주차 뒤 재정렬: 보정 {dT[0]*100:+.1f}/{dT[1]*100:+.1f} cm {math.degrees(dT[2]):+.1f}° (지도 일치 {r["match"]*100:.0f}%)')
                self.set_T(Tn, 'realign', r['match']); self.init_amcl()
            elif small(dT):
                self.get_logger().warn(f'재정렬 ICP: 지금 T 가 맞음(보정 {dT[0]*100:+.1f}/{dT[1]*100:+.1f} cm {math.degrees(dT[2]):+.1f}°) → AMCL 이 틀림, AMCL 다시 초기화')
                self.init_amcl()
            else:
                Tn = wc.compose(dT, T0)
                self.get_logger().warn(f'재정렬 ICP: FAST-LIO 흐름 보정 {dT[0]*100:+.1f}/{dT[1]*100:+.1f} cm {math.degrees(dT[2]):+.1f}° (지도 일치 {r["match"]*100:.0f}%) → T 갱신')
                self.set_T(Tn, 'reanchor', r['match'])
                self.init_amcl()
        except Exception as e:      # 감시가 주행을 죽이지 않게
            self.icp_info = {'ok': False, 'reason': f'예외 {e}'}
            self.get_logger().error(f'재정렬 ICP 예외: {e}')
        finally:
            self.icp_busy = False


def main():
    rclpy.init()
    n = Loc()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
