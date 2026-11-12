#!/usr/bin/env python3
"""v2 시뮬 세계 — ★ simcal 보정판(11-06): --lat 명령→움직임 지연, --rot-thr 제자리 회전 정지마찰(16:54 회전 타임아웃 4번 재현). 원본: sim/sim_world_v2.py
junsu_v2 실제 스택(러너 · review ego · 합치기 · 몸 모양 안전 필터 · cmd_vel_to_sport)을 로봇 없이 돌린다(11-06).
★ 격리 도메인(run_sim_v2.sh: 78~81, lo 전용)에서만. 로봇이 응답하면 run_sim_v2.sh 가 거부한다.

받음  /api/sport/request  Move(1008 {"x","y","z"} 몸체 기준) · StopMove(1003) · StandDown(1005) · StandUp(1004) · BalanceStand(1002)
                          · ClassicWalk(2049) · Sit(1009) · RiseSit(1010)
      /mission/camera_active
냄    /Odometry · /utlidar/robot_odom (camera_init, 50 Hz) · /sportmodestate · /lf/sportmodestate (속도·몸 높이)
      /scan (가상 라이다: 벽 칸 8 m 안, 0.5° 칸, 뒤 ±35° 가림, 10 Hz) · /mission/wall_cloud (10 m 안 벽 칸, 10 Hz)
      /mission/cones (PoseArray, 콘 검출 흉내: 카메라 앞 ±43°·3 m 안, 놓침 확률·위치 잡음) · /mission/traffic_light_state (카메라 켜졌을 때 진짜 신호)
움직임 모델(가정 — 실측 아님, 판마다 바꿔 봄)
  · 가속 제한 1.0 m/s² · 0.8 · 3.0 rad/s², 명령 0.5초 끊기면 멈춤
  · 발 떼기 문턱(--step-thr, --wz-thr): 모든 축이 문턱 아래면 몸만 기울고 제자리
  · 걸음새: BalanceStand·StandUp 뒤 = 기본 보행. 기본 보행에서 회전만 하는 명령이 --default-rot-thr 아래면 '기울었다 튐'(회전 안 됨, 기록)
  · 걸음새 바꾼 뒤 0.5초는 명령 무시(바뀌는 중) · 엎드리기/일어나기 1.5초
  · 직진 쏠림: 걸을 때 왼쪽으로 (0.0156·vx + 0.0397) rad/s — 모터 노드의 쏠림 보정이 이걸 지움(11-05 실측과 같은 크기)
기록  --log JSON 줄: 0.1초마다 자세·속도·걸음새·몸-장애물 부호 있는 여유(0.70×0.31, 파고들면 음수) + 명령·사건
"""
import argparse, json, math, random, time
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import LaserScan
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Bool, String, Header
from unitree_api.msg import Request
from unitree_go.msg import SportModeState
from scipy.spatial import cKDTree

MOVE, STOPMOVE, STANDDOWN, STANDUP, BALANCE, CLASSIC, SIT, RISESIT = 1008, 1003, 1005, 1004, 1002, 2049, 1009, 1010
STAND_H, DOWN_H = 0.31, 0.07
L2, W2 = 0.35, 0.155


class SimWorld(Node):
    def __init__(self, a):
        super().__init__('sim_world_v2')
        self.a = a; self.rng = np.random.default_rng(a.seed); random.seed(a.seed)
        self.walls = np.load(a.walls).astype(np.float32); self.wtree = cKDTree(self.walls[:, :2])
        self.obc = np.load(a.obc).astype(np.float64); self.otree = cKDTree(self.obc)
        self.cones = np.load(a.cones).astype(np.float64) if a.cone_mode != 'none' else np.zeros((0, 2))
        self.x, self.y, self.yaw = a.start
        self.vx = self.vy = self.wz = 0.0; self.cx = self.cy = self.cz = 0.0; self.cmd_t = -1e9
        self.h = STAND_H; self.down = False; self.pose_busy_until = -1e9
        self.gait = 'default'; self.gait_busy_until = -1e9
        self.cam_on = False; self.t0 = time.monotonic(); self.lean_t = 0.0; self.hop_t = 0.0
        self.log = open(a.log, 'w')
        self.pub_odom = self.create_publisher(Odometry, '/Odometry', 10)
        self.pub_leg = self.create_publisher(Odometry, '/utlidar/robot_odom', 10)
        self.pub_state = self.create_publisher(SportModeState, '/sportmodestate', 10)
        self.pub_lf = self.create_publisher(SportModeState, '/lf/sportmodestate', 10)
        self.pub_scan = self.create_publisher(LaserScan, '/scan', 10)
        self.pub_wall = self.create_publisher(pc2.PointCloud2, '/mission/wall_cloud', 10)
        self.pub_cones = self.create_publisher(PoseArray, '/mission/cones', 5)
        self.pub_light = self.create_publisher(String, '/mission/traffic_light_state', 10)
        self.create_subscription(Request, '/api/sport/request', self.cb_req, 50)
        self.create_subscription(Bool, '/mission/camera_active', self.cb_cam, 10)
        self.create_subscription(String, '/mission/runner_status', self.cb_status, 5)
        self.runner_state = ''
        self.pend = []; self.rot_stall_t = 0.0   # ★ simcal: (적용 시각, x, y, z) 지연 버퍼 · 제자리 회전 정지마찰 누적 시간
        self.dt = 0.02
        self.create_timer(self.dt, self.step)
        self.create_timer(0.1, self.sensors)
        self.create_timer(0.1, self.record)
        self.ev('start', walls=len(self.walls), cones=len(self.cones), cone_mode=a.cone_mode, step_thr=a.step_thr, wz_thr=a.wz_thr, lat=a.lat, rot_thr=a.rot_thr)

    def now(self):
        return time.monotonic() - self.t0

    def ev(self, kind, **kw):
        d = {'t': round(self.now(), 2), 'ev': kind}; d.update(kw)
        self.log.write(json.dumps(d, ensure_ascii=False) + '\n'); self.log.flush()

    def true_light(self):
        return 'RED' if int(self.now() // self.a.cycle) % 2 == 0 else 'GREEN'

    def cb_status(self, m):
        try:
            self.runner_state = json.loads(m.data).get('state', '')
        except ValueError:
            pass

    def cb_cam(self, m):
        if bool(m.data) != self.cam_on:
            self.cam_on = bool(m.data); self.ev('camera_active', on=self.cam_on)

    def cb_req(self, m):
        api = m.header.identity.api_id; t = self.now()
        if api == MOVE:
            try:
                p = json.loads(m.parameter or '{}')
            except ValueError:
                p = {}
            self.pend.append((t + self.a.lat, float(p.get('x', 0)), float(p.get('y', 0)), float(p.get('z', 0))))   # ★ simcal: 지연
            return
        if api == STOPMOVE:
            self.pend.append((t + self.a.lat, 0.0, 0.0, 0.0))
        elif api in (STANDDOWN, SIT):
            self.down = True; self.pose_busy_until = t + 1.5
        elif api in (STANDUP, RISESIT):
            self.down = False; self.pose_busy_until = t + 1.5; self.gait = 'default'
        elif api == BALANCE:
            self.gait = 'default'; self.gait_busy_until = t + 0.5
        elif api == CLASSIC:
            if self.gait != 'classic':
                self.gait_busy_until = t + 0.5
            self.gait = 'classic'
        self.ev('sport', api=int(api), gait=self.gait, param=(m.parameter or '')[:40])

    def step(self):
        dt = self.dt; t = self.now()
        while self.pend and self.pend[0][0] <= t:                          # ★ simcal: 지연된 명령 적용(cmd_t 도 적용 시각 기준)
            _, self.cx, self.cy, self.cz = self.pend.pop(0); self.cmd_t = t
        moving_ok = not self.down and t >= self.pose_busy_until and t >= self.gait_busy_until and t - self.cmd_t <= 0.5
        tx, ty, tz = (self.cx, self.cy, self.cz) if moving_ok else (0.0, 0.0, 0.0)
        if moving_ok and (abs(tx) > 1e-3 or abs(ty) > 1e-3 or abs(tz) > 1e-3):
            if self.a.step_thr > 0 and abs(tx) < self.a.step_thr and abs(ty) < self.a.step_thr and abs(tz) < self.a.wz_thr:
                tx = ty = tz = 0.0; self.lean_t += dt                     # 발 안 뗌(몸만 기움)
            elif self.a.rot_thr > 0 and abs(tx) < 0.05 and abs(ty) < 0.05 and 0 < abs(tz) < self.a.rot_thr:
                tz = 0.0; self.rot_stall_t += dt                           # ★ simcal: 제자리 회전 정지마찰(16:54: 남은 5~15° 에서 10초 안 돎)
            elif self.gait != 'classic' and abs(tx) < 0.05 and abs(ty) < 0.05 and 0 < abs(tz) < self.a.default_rot_thr:
                tz = 0.0; self.hop_t += dt                                 # 기본 보행 제자리 회전이 느려 기울었다 튐
        lim = lambda cur, tgt, acc: cur + max(-acc * dt, min(acc * dt, tgt - cur))
        self.vx, self.vy, self.wz = lim(self.vx, tx, 1.0), lim(self.vy, ty, 0.8), lim(self.wz, tz, 3.0)
        walking = math.hypot(self.vx, self.vy) > 0.03
        drift = (0.0156 * self.vx + 0.0397) if walking else 0.0          # 실제 로봇의 왼쪽 쏠림(모터 노드 보정이 상쇄)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (c * self.vx - s * self.vy) * dt; self.y += (s * self.vx + c * self.vy) * dt
        self.yaw = (self.yaw + (self.wz + drift) * dt + math.pi) % (2 * math.pi) - math.pi
        tgt_h = DOWN_H if self.down else STAND_H
        self.h += max(-0.2 * dt, min(0.2 * dt, tgt_h - self.h))
        st = self.get_clock().now().to_msg()
        nx, ny, na = self.x, self.y, self.yaw
        if self.a.noise > 0:
            nx += self.rng.normal(0, self.a.noise); ny += self.rng.normal(0, self.a.noise); na += math.radians(self.rng.normal(0, self.a.noise * 30))
        for pub, frame in ((self.pub_odom, 'camera_init'), (self.pub_leg, 'odom')):
            o = Odometry(); o.header.stamp = st; o.header.frame_id = frame; o.child_frame_id = 'body'
            o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = nx, ny, self.h - STAND_H
            o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(na / 2), math.cos(na / 2)
            o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.angular.z = self.vx, self.vy, self.wz
            pub.publish(o)
        sm = SportModeState(); sm.velocity = [float(c * self.vx - s * self.vy), float(s * self.vx + c * self.vy), 0.0]
        sm.body_height = float(self.h); sm.position = [float(self.x), float(self.y), 0.0]; sm.mode = 1 if self.down else 3
        self.pub_state.publish(sm)
        if int(t / dt) % 5 == 0:
            self.pub_lf.publish(sm)

    def sgap(self):
        idx = self.otree.query_ball_point([self.x, self.y], 0.55)
        if not idx:
            return 0.5
        q = self.obc[idx] - [self.x, self.y]; a = self.yaw
        lx = np.abs(q[:, 0] * math.cos(a) + q[:, 1] * math.sin(a)); ly = np.abs(-q[:, 0] * math.sin(a) + q[:, 1] * math.cos(a))
        out = np.hypot(np.maximum(lx - L2, 0), np.maximum(ly - W2, 0)); ins = (lx < L2) & (ly < W2)
        return float(np.where(ins, -np.minimum(L2 - lx, W2 - ly), out).min())

    def record(self):
        self.ev('pose', x=round(self.x, 3), y=round(self.y, 3), yaw=round(math.degrees(self.yaw), 1), v=round(math.hypot(self.vx, self.vy), 3),
                wz=round(self.wz, 3), cmd=[round(self.cx, 3), round(self.cy, 3), round(self.cz, 3)], h=round(self.h, 3), gait=self.gait,
                gap=round(self.sgap(), 3), lean=round(self.lean_t, 2), hop=round(self.hop_t, 2), rstall=round(self.rot_stall_t, 2), rs=self.runner_state)

    def sensors(self):
        h = Header(); h.stamp = self.get_clock().now().to_msg(); h.frame_id = 'camera_init'
        idx = self.wtree.query_ball_point([self.x, self.y], 10.0)
        self.pub_wall.publish(pc2.create_cloud_xyz32(h, self.walls[idx]))
        # 가상 라이다: 몸체 좌표 0.5° 칸 최소 거리, 뒤 ±35° 가림
        P = self.walls[self.wtree.query_ball_point([self.x, self.y], 8.0), :2].astype(np.float64)
        n = 720; rng = np.full(n, np.inf)
        if len(P):
            d = P - [self.x, self.y]; r = np.hypot(d[:, 0], d[:, 1]); ang = (np.arctan2(d[:, 1], d[:, 0]) - self.yaw + math.pi) % (2 * math.pi) - math.pi
            b = ((ang + math.pi) / (2 * math.pi) * n).astype(int) % n; ok = (r > 0.25) & (np.abs(ang) < math.radians(145))
            np.minimum.at(rng, b[ok], r[ok] + self.rng.normal(0, 0.01, ok.sum()))
        sc = LaserScan(); sc.header = Header(); sc.header.stamp = h.stamp; sc.header.frame_id = 'base_link'
        sc.angle_min = -math.pi; sc.angle_increment = 2 * math.pi / n; sc.angle_max = sc.angle_min + sc.angle_increment * (n - 1)
        sc.range_min = 0.25; sc.range_max = 15.0; sc.ranges = [float(v) for v in rng]
        self.pub_scan.publish(sc)
        # 콘 검출 흉내(카메라 앞 ±43°, 3 m)
        if self.a.cone_mode == 'det' and len(self.cones):
            d = self.cones - [self.x, self.y]; r = np.hypot(d[:, 0], d[:, 1]); ang = (np.arctan2(d[:, 1], d[:, 0]) - self.yaw + math.pi) % (2 * math.pi) - math.pi
            vis = (r < 3.0) & (np.abs(ang) < math.radians(43)) & (self.rng.random(len(r)) > self.a.cone_miss)
            pa = PoseArray(); pa.header = h
            for cx, cy in self.cones[vis]:
                p = Pose(); p.position.x = float(cx + self.rng.normal(0, self.a.cone_noise)); p.position.y = float(cy + self.rng.normal(0, self.a.cone_noise)); p.orientation.w = 1.0
                pa.poses.append(p)
            self.pub_cones.publish(pa)
        if self.cam_on:
            self.pub_light.publish(String(data=f'{self.true_light()}|0.90'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--walls', required=True); ap.add_argument('--obc', required=True); ap.add_argument('--cones', required=True)
    ap.add_argument('--log', required=True); ap.add_argument('--start', type=float, nargs=3, required=True)
    ap.add_argument('--cycle', type=float, default=5.0); ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--cone-mode', default='det', choices=['det', 'walls', 'none'], help='det: 벽(스캔)+콘 검출 / walls: 벽(스캔)만 / none: 콘 없음(콘 칸도 벽에서 안 뺌 — 벽 파일 그대로)')
    ap.add_argument('--cone-miss', type=float, default=0.2); ap.add_argument('--cone-noise', type=float, default=0.05)
    ap.add_argument('--step-thr', type=float, default=0.10); ap.add_argument('--wz-thr', type=float, default=0.12)
    ap.add_argument('--default-rot-thr', type=float, default=0.6); ap.add_argument('--noise', type=float, default=0.0)
    ap.add_argument('--lat', type=float, default=0.0, help='★ simcal: 명령→움직임 지연(초)')
    ap.add_argument('--rot-thr', type=float, default=0.0, help='★ simcal: 제자리 회전(|vx|,|vy|<0.05) 이 이 각속도 아래면 안 돎(rad/s). 16:54 맞춤 0.30')
    a, _ = ap.parse_known_args()
    rclpy.init(); n = SimWorld(a)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
