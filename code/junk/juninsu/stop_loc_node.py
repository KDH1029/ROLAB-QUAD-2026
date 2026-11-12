#!/usr/bin/env python3
"""멈춘 곳 위치 확인 노드 (11-05 ②안) — AMCL 로 찾고, 로봇이 멈췄을 때만 3D ICP 로 확인해 좌표(T)를 고친다.

대회 러너(junk nail-1105)는 그대로. 러너가 이미 듣는 /mission/align_T 로 새 T(지도 = T · FAST-LIO)를 보낼 뿐이다.
  · 주행 중에는 아무것도 안 고침(러너도 주행 중 T 를 반영 못 함). /mission/loc_watch(STOP)는 아예 안 냄 → 로봇을 세우지 않음.
  · 멈춤(1.5초 넘게 0.03 m/s 아래, 서 있음)마다 한 번, 그리고 러너의 주차 후 /mission/realign 요청 때:
      AMCL 자세를 씨앗으로 최근 2초·1초 점구름(/cloud_registered_body, 그때만 받음)을 3D 지도에 ICP
      → 통과 조건: 슬라럼 밖 · 1초/2초 답이 3 cm·1° 안 · 일치 ≥ 0.25 · ICP 답이 AMCL 과 15 cm·5° 안
      → 통과면 /mission/align_T {"ok": true}. 실패면 그대로(주차 요청이었으면 {"ok": false, "reason": "realign …"} 로 바로 답해 러너가 안 기다리게).
  · AMCL(nav2, /loc 네임스페이스, /tf_loc 트리)은 감시·씨앗용: 이 노드가 /tf_loc 에 lio_odom→base_link(/Odometry)와
    base_link→base_stab(/tf 에서 복사)을 내고, 출발 정렬 파일(T)이 새로 생기면 AMCL 초기자세 = T · 지금 자세 를 준다.
재생 검증(debug/loc 덤프 + 일부러 틀어진 자세): 서는 곳 4곳 모두 반영, 반영 뒤 0.1~1.1 cm(주차 지나가며 8.7 cm). 36 cm·8° 튐도 다음 정지선에서 0.7 cm.
"""
import json
import math
import os
import sys
import threading
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import icp_core as ic  # noqa: E402

try:
    from unitree_go.msg import SportModeState
except ImportError:          # 재생 환경에 없을 수 있음 → 높이 판정 생략
    SportModeState = None

J = os.path.expanduser('~/ws/junk')


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def comp(A, B):
    c, s = math.cos(A[2]), math.sin(A[2]); return np.array([c * B[0] - s * B[1] + A[0], s * B[0] + c * B[1] + A[1], A[2] + B[2]])


def inv(A):
    c, s = math.cos(A[2]), math.sin(A[2]); return np.array([-(c * A[0] + s * A[1]), s * A[0] - c * A[1], -A[2]])


def err(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1]), abs(math.degrees((a[2] - b[2] + math.pi) % (2 * math.pi) - math.pi))


class StopLoc(Node):
    def __init__(self):
        super().__init__('stop_loc')
        P = lambda n, v: self.declare_parameter(n, v).value
        self.align_file = P('align_file', os.path.join(J, 'logs', 'route_today.json.align.json'))
        route = P('route', os.path.join(J, 'routes', 'rec1650', 'route.json'))
        mapdir = P('map_dir', os.path.join(J, 'routes', 'rec1650'))
        self.odom_topic = P('odom_topic', '/Odometry')
        self.cloud_topic = P('cloud_topic', '/cloud_registered_body')
        self.still_v, self.still_s, self.min_h = P('still_speed', 0.03), P('still_sec', 1.5), P('min_height', 0.25)
        self.agree_m, self.agree_deg, self.min_inl = P('agree_m', 0.15), P('agree_deg', 5.0), P('min_inlier', 0.25)
        self.log_path = P('log', os.path.join(J, 'logs', 'stop_loc.log'))
        r = json.load(open(route))['route']; SL = np.array([[e['x'], e['y']] for e in r[5:25]])   # 슬라럼 #6~#25
        self.sl_lo, self.sl_hi = SL.min(0) - 0.4, SL.max(0) + 0.4
        t0 = time.time()
        self.m = ic.Matcher(ic.load_map(os.path.join(mapdir, 'map.npz'), path_xy=np.load(os.path.join(mapdir, 'editor_cache.npz'))['path']))
        self.T = np.zeros(3); self.align_mtime = None; self.amcl_init_t = None; self.init_pending = False
        self.odom = deque(maxlen=400); self.amcl = None; self.height = None
        self.busy = False; self.req_realign = False; self.last_try_xy = None; self.clouds = []; self.cloud_sub = None; self.collect_until = None
        self.lock = threading.Lock()
        tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_tf = self.create_publisher(TFMessage, '/tf_loc', 50)
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped, '/loc/initialpose', tl)
        self.pub_T = self.create_publisher(String, '/mission/align_T', tl)
        self.pub_st = self.create_publisher(String, '/mission/loc_stop', 5)
        self.create_subscription(Odometry, self.odom_topic, self.cb_odom, 50)
        self.create_subscription(TFMessage, '/tf', self.cb_tf, 100)
        self.create_subscription(PoseWithCovarianceStamped, '/loc/amcl_pose', self.cb_amcl, 10)
        self.create_subscription(String, '/mission/realign', self.cb_realign, 5)
        if SportModeState is not None:
            self.create_subscription(SportModeState, '/sportmodestate', self.cb_state, 10)
        self.create_timer(0.2, self.tick)
        self.say(f'시작 — 지도 {mapdir} (준비 {time.time() - t0:.1f}초) · 정렬 파일 {self.align_file} · STOP 은 절대 안 냄')

    # ── 기록
    def say(self, msg, err_=False):
        (self.get_logger().error if err_ else self.get_logger().warn)(msg)
        try:
            open(self.log_path, 'a').write(time.strftime('%H:%M:%S ') + msg + '\n')
        except OSError:
            pass
        self.pub_st.publish(String(data=json.dumps({'t': time.time(), 'msg': msg}, ensure_ascii=False)))

    # ── 입력
    def cb_odom(self, m):
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9; p, q = m.pose.pose.position, m.pose.pose.orientation
        M = np.eye(4); M[:3, :3] = ic.quat_R(q.x, q.y, q.z, q.w); M[:3, 3] = (p.x, p.y, p.z)
        with self.lock:
            self.odom.append((st, M, np.array([p.x, p.y, yaw_of(q)])))
        t = TransformStamped(); t.header.stamp = m.header.stamp; t.header.frame_id = 'lio_odom'; t.child_frame_id = 'base_link'
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.x, p.y, p.z
        t.transform.rotation = q
        self.pub_tf.publish(TFMessage(transforms=[t]))

    def cb_tf(self, m):
        out = [t for t in m.transforms if t.header.frame_id == 'base_link' and t.child_frame_id == 'base_stab']
        if out:
            self.pub_tf.publish(TFMessage(transforms=out))

    def cb_amcl(self, m):
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9; p = m.pose.pose.position
        self.amcl = (st, np.array([p.x, p.y, yaw_of(m.pose.pose.orientation)]), m.pose.covariance[0], m.pose.covariance[35])

    def cb_state(self, m):
        self.height = float(m.body_height)

    def cb_realign(self, m):
        self.say(f'러너 재정렬 요청 {m.data}')
        self.req_realign = True
        if not self.busy:
            self.start_check('realign')

    def cb_cloud(self, m):
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        a = np.frombuffer(bytes(m.data), np.uint8).reshape(-1, m.point_step)
        xyz = np.stack([a[:, o:o + 4].copy().view(np.float32)[:, 0] for o in (0, 4, 8)], 1)
        self.clouds.append((st, xyz[np.isfinite(xyz).all(1)]))

    # ── 도우미
    def odom_at(self, t):
        with self.lock:
            if not self.odom:
                return None
            ts = np.array([o[0] for o in self.odom]); k = int(np.argmin(abs(ts - t)))
            return self.odom[k] if abs(ts[k] - t) < 0.2 else None

    def speed(self, sec):
        with self.lock:
            o = [x for x in self.odom if x[0] >= self.odom[-1][0] - sec] if self.odom else []
        if len(o) < 3 or o[-1][0] - o[0][0] < sec * 0.8:
            return None
        return max(math.hypot(*(o[i + 1][2][:2] - o[i][2][:2])) / max(o[i + 1][0] - o[i][0], 1e-3) for i in range(len(o) - 1))

    def in_slalom(self, q):
        return bool(np.all(q[:2] >= self.sl_lo) and np.all(q[:2] <= self.sl_hi))

    def amcl_C(self):
        """AMCL 이 말하는 지도←FAST-LIO(T). 없으면 None."""
        if self.amcl is None:
            return None
        o = self.odom_at(self.amcl[0])
        return comp(self.amcl[1], inv(o[2])) if o is not None else None

    # ── 주기
    def tick(self):
        try:
            mt = os.path.getmtime(self.align_file)
        except OSError:
            mt = None
        if mt is not None and mt != self.align_mtime:
            self.align_mtime = mt
            try:
                d = json.load(open(self.align_file)); T = np.array(d['T'], float) if d.get('ok') else np.zeros(3)
                self.T = T; self.init_pending = True; self.last_try_xy = None
                # ★ align_T 는 '마지막 값 보관'(TRANSIENT_LOCAL) — 새 주행(r)에서 러너가 지난 주행의 보정 T 를 받지 않게, 이번 출발 T 를 바로 다시 냄
                self.pub_T.publish(String(data=json.dumps({'T': [float(v) for v in T], 'ok': True, 'reason': 'start stop_loc: 출발 정렬 파일'}, ensure_ascii=False)))
                self.say(f"출발 정렬 파일 → T = ({T[0]:+.3f}, {T[1]:+.3f}, {math.degrees(T[2]):+.2f}°) {'' if d.get('ok') else '(정렬 거부 → 0)'} · 보관값도 이걸로 바꿈")
            except (OSError, ValueError, KeyError) as e:
                self.say(f'정렬 파일 읽기 실패: {e}', True)
        if self.init_pending and self.odom and (self.height is None or self.height > self.min_h):
            st, _, p = self.odom[-1]; q = comp(self.T, p)
            i = PoseWithCovarianceStamped(); i.header.frame_id = 'map'
            i.header.stamp.sec = int(st); i.header.stamp.nanosec = int((st - int(st)) * 1e9)
            i.pose.pose.position.x, i.pose.pose.position.y = float(q[0]), float(q[1])
            i.pose.pose.orientation.z, i.pose.pose.orientation.w = math.sin(q[2] / 2), math.cos(q[2] / 2)
            cov = [0.0] * 36; cov[0] = cov[7] = 0.05 ** 2; cov[35] = math.radians(3) ** 2; i.pose.covariance = cov
            self.pub_init.publish(i); self.init_pending = False; self.amcl_init_t = st
            self.say(f'AMCL 초기자세 = T · 지금 자세 = ({q[0]:.2f}, {q[1]:.2f}, {math.degrees(q[2]):.1f}°)')
        if self.collect_until is not None and ((self.clouds and self.clouds[-1][0] >= self.collect_until) or time.time() - self.collect_wall > 4.0):
            self.finish_collect()                                   # 2.1초치 모이면(점구름이 안 오면 4초 뒤) 확인
            return
        if self.busy or not self.odom or self.amcl_init_t is None:
            return
        v = self.speed(self.still_s)
        if v is None or v >= self.still_v or (self.height is not None and self.height < self.min_h):
            return
        xy = self.odom[-1][2][:2]
        if self.last_try_xy is not None and math.hypot(*(xy - self.last_try_xy)) < 0.5:
            return                                                  # 같은 멈춤에서는 한 번만
        if self.odom[-1][0] - self.amcl_init_t < 3.0:
            return                                                  # AMCL 이 자리잡을 시간
        self.start_check('stop')

    def start_check(self, why):
        self.busy = True; self.why = why; self.clouds = []
        self.last_try_xy = self.odom[-1][2][:2].copy() if self.odom else None
        self.collect_until = (self.odom[-1][0] if self.odom else 0.0) + 2.1; self.collect_wall = time.time()
        self.cloud_sub = self.create_subscription(PointCloud2, self.cloud_topic, self.cb_cloud, qos_profile_sensor_data)

    def finish_collect(self):
        self.collect_until = None
        if self.cloud_sub is not None:
            self.destroy_subscription(self.cloud_sub); self.cloud_sub = None
        clouds = self.clouds; self.clouds = []
        threading.Thread(target=self.check, args=(clouds,), daemon=True).start()

    def scan(self, clouds, win):
        t_end = clouds[-1][0]; o_end = self.odom_at(t_end)
        if o_end is None:
            return None, None
        Pi = np.linalg.inv(o_end[1]); out = []
        for st, xyz in clouds:
            if st < t_end - win:
                continue
            o = self.odom_at(st)
            if o is None:
                continue
            M = Pi @ o[1]; out.append((M[:3, :3] @ xyz.T.astype(np.float64)).T + M[:3, 3])
        if not out:
            return None, None
        S = np.concatenate(out); S = S[(S[:, 2] > -0.20) & (S[:, 2] < 1.5) & (np.hypot(S[:, 0], S[:, 1]) > 0.4)]
        S = ic.voxel(S, 0.05)
        if len(S) > 6000:
            S = S[np.random.default_rng(0).choice(len(S), 6000, replace=False)]
        return S, o_end

    def check(self, clouds):
        why = 'realign' if (self.why == 'realign' or self.req_realign) else 'stop'
        t0 = time.time(); res = None; reason = ''
        try:
            C = self.amcl_C()
            if C is None:
                reason = 'AMCL 자세 없음'
            elif len(clouds) < 5:
                reason = f'점구름 부족({len(clouds)}장)'
            else:
                out = {}
                for win in (2.0, 1.0):
                    S, o = self.scan(clouds, win)
                    if S is None or len(S) < 200:
                        break
                    p = o[2]; G = comp(C, p)
                    M0 = o[1].copy(); Rz = ic.T3_to4((0, 0, G[2] - p[2]))[:3, :3]; M0[:3, :3] = Rz @ o[1][:3, :3]; M0[:2, 3] = G[:2]
                    M, rms, _ = self.m.align(S, M0)
                    Mb = np.array([M[0, 3], M[1, 3], G[2] + math.atan2(M[1, 0], M[0, 0]) - math.atan2(M0[1, 0], M0[0, 0])])
                    out[win] = (comp(Mb, inv(p)), self.m.inlier(S, M), p, G)
                if len(out) < 2:
                    reason = '스캔 부족'
                else:
                    (Ta, inl, p, G), (Tb, _, _, _) = out[2.0], out[1.0]
                    e12 = err(comp(Ta, p), comp(Tb, p)); eA = err(comp(Ta, p), G); eT = err(comp(Ta, p), comp(self.T, p))
                    if self.in_slalom(G):
                        reason = '슬라럼 안'
                    elif not (e12[0] < 0.03 and e12[1] < 1.0):
                        reason = f'1초/2초 불일치 {e12[0] * 100:.1f} cm {e12[1]:.1f}°'
                    elif inl < self.min_inl:
                        reason = f'일치 {inl:.2f} < {self.min_inl}'
                    elif not (eA[0] < self.agree_m and eA[1] < self.agree_deg):
                        reason = f'AMCL 과 안 맞음 {eA[0] * 100:.0f} cm {eA[1]:.1f}°'
                    else:
                        res = Ta; reason = f'ICP 확인(일치 {inl:.2f}, AMCL 차 {eA[0] * 100:.1f} cm) · 지금 T 대비 {eT[0] * 100:.1f} cm {eT[1]:.2f}°'
        except Exception as e:  # noqa: BLE001  — 어떤 실패도 '그대로 둠'
            reason = f'오류 {e!r}'
        ms = (time.time() - t0) * 1000
        if res is not None:
            self.T = res
            self.pub_T.publish(String(data=json.dumps({'T': [float(v) for v in res], 'ok': True, 'reason': f'{why} stop_loc: {reason}'}, ensure_ascii=False)))
            self.say(f'[{why}] 반영 → T = ({res[0]:+.3f}, {res[1]:+.3f}, {math.degrees(res[2]):+.2f}°) · {reason} · {ms:.0f} ms')
        else:
            if why == 'realign':      # 러너가 5초 기다리지 않게 바로 '안 고침' 답
                self.pub_T.publish(String(data=json.dumps({'T': [float(v) for v in self.T], 'ok': False, 'reason': f'realign stop_loc 거부: {reason}'}, ensure_ascii=False)))
            self.say(f'[{why}] 안 고침(그대로 감) · {reason} · {ms:.0f} ms')
        self.req_realign = False; self.busy = False


def main():
    rclpy.init(); n = StopLoc()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    try:
        rclpy.shutdown()
    except Exception:  # noqa: BLE001
        pass


if __name__ == '__main__':
    main()
