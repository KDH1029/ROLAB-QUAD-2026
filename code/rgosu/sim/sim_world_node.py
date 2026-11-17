#!/usr/bin/env python3
"""가상 로봇 + 가상 세계 — 실제 로봇 없이 러너·ego_planner·명령 노드·신호등 노드를 통째로 돌린다.
★ 격리 도메인(run_sim.sh: 도메인 79, lo 전용)에서만. 실제 로봇이 응답하면 run_sim.sh 가 거부한다.

흉내 내는 것 (실제 스택이 주고받는 토픽 그대로)
  받음  /api/sport/request   Move(1008, {"x":vx,"y":vy,"z":wz} 몸체 기준) · StopMove(1003) · Sit(1009) · RiseSit(1010) · 걸음새 등은 기록만
  냄    /Odometry            camera_init → body, 50 Hz (FAST-LIO 대신)
        /sportmodestate      velocity · body_height (러너의 정지·앉기 판정용), 50 Hz
        /mission/wall_cloud  sim_walls.npy(14:59 녹화에 고친 wall_filter 를 돌린 결과) 중 10 m 안, 10 Hz (wall_filter 대신)
        /camera/color/image_raw  카메라가 켜져 있으면(camera_active) 실제 녹화 사진 — 신호가 RED 면 빨강 사진, GREEN 이면 초록 사진, 10 Hz
  신호  RED/GREEN 을 --cycle 초씩 번갈아(규정: 5초 주기). 시뮬레이션 시작 때 RED.
  기록  --log 에 JSON 줄: 명령·목표·카메라·신호 판정·접촉(벽까지 0.15 m 미만)·1초마다 자세
"""
import argparse, glob, json, math, os, random, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Bool, String, Header
from unitree_api.msg import Request
from unitree_go.msg import SportModeState
from scipy.spatial import cKDTree

MOVE, STOPMOVE, SIT, RISESIT = 1008, 1003, 1009, 1010
STAND_H, SIT_H = 0.31, 0.09


class SimWorld(Node):
    def __init__(self, a):
        super().__init__('sim_world')
        self.a = a
        self.walls = np.load(a.walls).astype(np.float32)
        self.wtree = cKDTree(self.walls[:, :2])
        self.x, self.y, self.yaw = a.start
        self.vx = self.vy = self.wz = 0.0           # 몸체 기준 실제 속도
        self.cx = self.cy = self.cz = 0.0           # 명령
        self.cmd_t = -1e9
        self.h = STAND_H; self.sitting = False
        self.cam_on = False
        self.t0 = time.monotonic()
        self.imgs = {k: [cv2.cvtColor(cv2.imread(f), cv2.COLOR_BGR2RGB) for f in sorted(glob.glob(os.path.join(a.tl_dir, k, '*.jpg')))]
                     for k in ('red', 'green')}
        self.log = open(a.log, 'a')
        self.contact_s = 0.0
        self.last_light = None
        self.pub_odom = self.create_publisher(Odometry, '/Odometry', 10)
        self.pub_state = self.create_publisher(SportModeState, '/sportmodestate', 10)
        self.pub_wall = self.create_publisher(PointCloud2, '/mission/wall_cloud', 10)
        self.pub_img = self.create_publisher(Image, '/camera/color/image_raw',
                                             QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST))
        self.create_subscription(Request, '/api/sport/request', self.cb_req, 50)
        self.create_subscription(Bool, '/mission/camera_active', self.cb_cam, 10)
        self.create_subscription(Bool, '/mission/cone_zone_active', lambda m: self.ev('cone_zone', on=bool(m.data)), 10)
        self.create_subscription(PoseStamped, '/goal_pose', lambda m: self.ev('goal', gx=round(m.pose.position.x, 2), gy=round(m.pose.position.y, 2)), 10)
        self.create_subscription(String, '/mission/traffic_light_state', self.cb_light, 10)
        self.create_subscription(Twist, '/cmd_vel', lambda m: None, 10)
        self.dt = 0.02
        self.create_timer(self.dt, self.step)
        self.create_timer(0.1, self.pub_walls_and_image)
        self.create_timer(0.2, lambda: self.ev('pose'))
        self.ev('start', walls=len(self.walls), red=len(self.imgs['red']), green=len(self.imgs['green']), cycle=a.cycle)

    # ---- 기록
    def now(self):
        return time.monotonic() - self.t0

    def true_light(self):
        return 'RED' if int(self.now() // self.a.cycle) % 2 == 0 else 'GREEN'

    def ev(self, kind, **kw):
        d = {'t': round(self.now(), 2), 'ev': kind, 'x': round(self.x, 3), 'y': round(self.y, 3), 'yaw': round(math.degrees(self.yaw), 1),
             'v': round(math.hypot(self.vx, self.vy), 3), 'light': self.true_light(), 'cam': self.cam_on}
        d.update(kw)
        self.log.write(json.dumps(d, ensure_ascii=False) + '\n'); self.log.flush()

    # ---- 받는 것
    def cb_req(self, m):
        api = m.header.identity.api_id
        if api == MOVE:
            try:
                p = json.loads(m.parameter or '{}')
            except ValueError:
                p = {}
            self.cx, self.cy, self.cz = float(p.get('x', 0)), float(p.get('y', 0)), float(p.get('z', 0)); self.cmd_t = self.now()
            return
        if api == STOPMOVE:
            self.cx = self.cy = self.cz = 0.0
        elif api == SIT:
            self.sitting = True
        elif api == RISESIT:
            self.sitting = False
        self.ev('sport', api=int(api), param=m.parameter[:60])

    def cb_cam(self, m):
        if bool(m.data) != self.cam_on:
            self.cam_on = bool(m.data); self.ev('camera_active', on=self.cam_on)

    def cb_light(self, m):
        s = m.data.split('|')[0]
        if s != self.last_light:
            self.last_light = s; self.ev('tl_state', state=m.data)

    # ---- 움직임
    def step(self):
        dt = self.dt
        if self.sitting or self.now() - self.cmd_t > 0.5:   # 앉았거나 Move 가 0.5초 끊기면 멈춤(실제 로봇과 같은 안전동작)
            tx = ty = tz = 0.0
        else:
            tx, ty, tz = self.cx, self.cy, self.cz
        lim = lambda cur, tgt, a: cur + max(-a * dt, min(a * dt, tgt - cur))
        self.vx, self.vy, self.wz = lim(self.vx, tx, 1.0), lim(self.vy, ty, 0.8), lim(self.wz, tz, 3.0)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (c * self.vx - s * self.vy) * dt; self.y += (s * self.vx + c * self.vy) * dt
        self.yaw = (self.yaw + self.wz * dt + math.pi) % (2 * math.pi) - math.pi
        tgt_h = SIT_H if self.sitting else STAND_H
        self.h += max(-0.25 * dt, min(0.25 * dt, tgt_h - self.h))
        d, _ = self.wtree.query([self.x, self.y])
        if d < 0.15:
            self.contact_s += dt
            if int(self.contact_s / dt) % 25 == 1:
                self.ev('contact', wall_dist=round(float(d), 3), contact_s=round(self.contact_s, 2))
        st = self.get_clock().now().to_msg()
        o = Odometry(); o.header.stamp = st; o.header.frame_id = 'camera_init'; o.child_frame_id = 'body'
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = self.x, self.y, 0.0
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.angular.z = self.vx, self.vy, self.wz
        self.pub_odom.publish(o)
        sm = SportModeState(); sm.velocity = [float(c * self.vx - s * self.vy), float(s * self.vx + c * self.vy), 0.0]
        sm.body_height = float(self.h); sm.position = [float(self.x), float(self.y), 0.0]; sm.mode = 1 if self.sitting else 3
        self.pub_state.publish(sm)

    def pub_walls_and_image(self):
        idx = self.wtree.query_ball_point([self.x, self.y], 10.0)
        h = Header(); h.stamp = self.get_clock().now().to_msg(); h.frame_id = 'camera_init'
        self.pub_wall.publish(pc2.create_cloud_xyz32(h, self.walls[idx]))
        if self.cam_on:
            k = 'red' if self.true_light() == 'RED' else 'green'
            im = random.choice(self.imgs[k]); m = Image(); m.header = h; m.header.frame_id = 'camera_color_optical_frame'
            m.height, m.width = im.shape[:2]; m.encoding = 'rgb8'; m.step = m.width * 3; m.data = im.tobytes()
            self.pub_img.publish(m)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--walls', required=True); ap.add_argument('--tl-dir', required=True); ap.add_argument('--log', required=True)
    ap.add_argument('--start', type=float, nargs=3, required=True, help='x y yaw(rad)')
    ap.add_argument('--cycle', type=float, default=5.0)
    from rclpy.utilities import remove_ros_args
    import sys
    a = ap.parse_args(remove_ros_args(sys.argv)[1:])
    rclpy.init(args=sys.argv)
    n = SimWorld(a)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.ev('end', contact_s=round(n.contact_s, 2)); n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
