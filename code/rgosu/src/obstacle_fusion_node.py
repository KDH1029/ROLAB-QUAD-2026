#!/usr/bin/env python3
"""장애물 합치기 노드(NUC) — 벽 + 콘을 한 점구름으로 묶어 ego_planner 에 준다. 콘 감지는 항상 켜져 있다(11-05).
  받음  /mission/wall_cloud   벽(wall_filter, 세계 camera_init) — 이게 올 때마다 내보낸다(ego 는 구름마다 격자를 새로 만든다)
        /utlidar/cloud_base   L1(base_link) — 바닥 위 8~35 cm, 0.3~2.0 m(cone_band.band_select). X자 경사로·계단 마스크 안은 버림
        /mission/cone_cam     내부 PC 카메라 콘(몸체 좌표) — YOLO 콘 상자+뎁스(먼 콘) · 주황+뎁스(가까운 콘)
        /utlidar/robot_odom   다리 기구학 odom — 콘을 '본 순간의 odom 좌표'로 기억해서, 시야 밖으로 나가도 로봇이 움직인 만큼만 따라 옮긴다
        /Odometry             FAST-LIO — 내보낼 때만 현재 자세로 세계 좌표에 놓는다(ego 와 같은 좌표계)
  냄    /mission/obstacle_cloud  벽 점 + 기억한 콘 점(높이 = 로봇 + 0.25 m: ego 의 몸 상자 필터에 안 걸리게)
시계: 로봇·내부 PC·NUC 시계가 달라 stamp 대신 '받은 순간의 최신 odom' 을 쓴다(지연 0.1 s × 0.3 m/s ≈ 3 cm).
"""
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cone_band as CB  # noqa: E402


def quat_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def read_xyz(msg):
    """PointCloud2 → (N,3) float64. x,y,z 가 float32 라고 가정(L1·wall_filter·cone_cam 모두 그렇다)."""
    off = {f.name: f.offset for f in msg.fields}
    n = msg.width * msg.height
    if n == 0 or not all(k in off for k in 'xyz'):
        return np.zeros((0, 3))
    a = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, msg.point_step)
    return np.stack([a[:, off[k]:off[k] + 4].copy().view(np.float32).ravel() for k in 'xyz'], 1).astype(np.float64)


def make_cloud(pts, frame, stamp):
    pts = np.ascontiguousarray(pts, dtype=np.float32)
    msg = PointCloud2(header=Header(frame_id=frame, stamp=stamp), height=1, width=len(pts), is_bigendian=False, is_dense=True,
                      point_step=12, row_step=12 * len(pts),
                      fields=[PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyz')])
    msg.data = pts.tobytes()
    return msg


class ObstacleFusion(Node):
    def __init__(self):
        super().__init__('obstacle_fusion')
        p = self.declare_parameter
        self.out_topic = p('out_topic', '/mission/obstacle_cloud').value
        self.mem_s = float(p('memory_s', 2.0).value)
        self.voxel = float(p('voxel', 0.05).value)
        self.cone_dz = float(p('cone_z_above_robot', 0.25).value)
        self.l1_min_hits = int(p('l1_min_hits', 3).value)        # L1 칸(5 cm)은 이만큼 여러 장에서 찍혀야 인정(잡음 거름)
        self.l1_wall_gap = float(p('l1_wall_gap', 0.20).value)   # 벽(wall_filter)에서 이 안의 L1 점은 버림 — 17:56 재생: L1 벽 밑동이 XT16 벽보다 10~25 cm 안쪽에 줄지어 찍혀 차선을 좁힘. 벽 옆 콘은 카메라가 맡는다
        self.use_l1 = bool(p('use_l1', True).value)
        self.use_cam = bool(p('use_cam', True).value)
        self.use_leg = bool(p('use_leg_odom', True).value)
        mask_path = os.path.expanduser(p('ramp_mask', '~/ws/Rgosu/maps/ramp_mask.npz').value)
        self.mask = None
        if mask_path and os.path.exists(mask_path):
            M = np.load(mask_path, allow_pickle=True)
            self.mask, self.mask_o, self.mask_c = M['mask'], M['origin'], float(M['cell'])
        self.lio = None          # (R, t) FAST-LIO 몸체 → 세계
        self.leg = None          # (R, t) base_link → odom(다리 기구학)
        self.mem = []            # [(받은 시각, 점 (N,3) — odom 좌표(다리 odom 있으면) 또는 세계 좌표, 출처 'l1'|'cam')]
        self.mem_frame = None    # 'odom' | 'world' — 기억 좌표계(다리 odom 이 끊기면 세계로 바뀌고 기억을 비움)
        self.n = {'wall': 0, 'l1': 0, 'l1_pts': 0, 'l1_mask': 0, 'cam': 0, 'cam_pts': 0, 'out_pts': 0}
        be = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, '/Odometry', self.cb_lio, 20)
        self.create_subscription(Odometry, '/utlidar/robot_odom', self.cb_leg, be)
        self.create_subscription(PointCloud2, '/mission/wall_cloud', self.cb_wall, be)
        if self.use_l1:
            self.create_subscription(PointCloud2, '/utlidar/cloud_base', self.cb_l1, be)
        if self.use_cam:
            self.create_subscription(PointCloud2, '/mission/cone_cam', self.cb_cam, be)
        self.pub = self.create_publisher(PointCloud2, self.out_topic, 2)
        self.last_leg_t = 0.0
        self.create_timer(5.0, self.report)
        self.get_logger().info(f'장애물 합치기: 벽 + L1({self.use_l1}) + 카메라 콘({self.use_cam}) · 기억 {self.mem_s} s(다리 odom {self.use_leg})'
                               f' · 경사로 마스크 {"있음" if self.mask is not None else "없음"} → {self.out_topic}')

    # ── 자세
    def cb_lio(self, m):
        self.lio = (quat_R(m.pose.pose.orientation), np.array([m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z]))

    def cb_leg(self, m):
        self.leg = (quat_R(m.pose.pose.orientation), np.array([m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z]))
        self.last_leg_t = time.monotonic()

    def leg_ok(self):
        return self.use_leg and self.leg is not None and time.monotonic() - self.last_leg_t < 0.5

    # ── 기억
    def remember(self, pts_body, src='l1'):
        """몸체 좌표 점을 기억한다 — 다리 odom 좌표(가능하면) 또는 세계 좌표."""
        if not len(pts_body):
            return
        frame = 'odom' if self.leg_ok() else ('world' if self.lio is not None else None)
        if frame is None:
            return
        if frame != self.mem_frame:
            self.mem = []; self.mem_frame = frame
            self.get_logger().warn(f'콘 기억 좌표계 → {frame}')
        R, t = self.leg if frame == 'odom' else self.lio
        self.mem.append((time.monotonic(), pts_body @ R.T + t, src))

    def memory_world(self, walls_xy=None):
        now = time.monotonic()
        self.mem = [m for m in self.mem if now - m[0] <= self.mem_s]
        if not self.mem or self.lio is None:
            return np.zeros((0, 3))
        out = []
        for src in ('l1', 'cam'):
            ps = [m[1] for m in self.mem if m[2] == src]
            if not ps:
                continue
            P = np.concatenate(ps)
            if self.mem_frame == 'odom':
                if not self.leg_ok():
                    return np.zeros((0, 3))
                Rl, tl = self.leg; Pb = (P - tl) @ Rl          # odom → 지금 몸체
                Rw, tw = self.lio; P = Pb @ Rw.T + tw            # 지금 몸체 → 세계(FAST-LIO)
            if src == 'l1':
                # 칸마다 몇 장에서 찍혔나 — 장마다 한 번만 세려고 장별로 칸을 먼저 유일화
                keys = []; i0 = 0
                for p in ps:
                    kk = np.unique(np.floor(P[i0:i0 + len(p), :2] / self.voxel).astype(np.int64), axis=0); keys.append(kk); i0 += len(p)
                K = np.concatenate(keys) if keys else np.zeros((0, 2), np.int64)
                if not len(K):
                    continue
                u, c = np.unique(K, axis=0, return_counts=True); k = u[c >= self.l1_min_hits]
            else:
                k = np.unique(np.floor(P[:, :2] / self.voxel).astype(np.int64), axis=0)
            xy = (k + 0.5) * self.voxel
            if src == 'l1' and walls_xy is not None and len(walls_xy) and len(xy) and self.l1_wall_gap > 0:
                from scipy.spatial import cKDTree
                xy = xy[cKDTree(walls_xy).query(xy, distance_upper_bound=self.l1_wall_gap)[0] > self.l1_wall_gap]
            out.append(xy)
        if not out:
            return np.zeros((0, 3))
        xy = np.unique(np.round(np.concatenate(out) / self.voxel).astype(np.int64), axis=0) * self.voxel
        return np.c_[xy, np.full(len(xy), self.lio[1][2] + self.cone_dz)]

    def in_mask(self, P_world):
        if self.mask is None or not len(P_world):
            return np.zeros(len(P_world), bool)
        ij = np.floor((P_world[:, :2] - self.mask_o) / self.mask_c).astype(int)
        ok = (ij[:, 0] >= 0) & (ij[:, 0] < self.mask.shape[0]) & (ij[:, 1] >= 0) & (ij[:, 1] < self.mask.shape[1])
        out = np.zeros(len(P_world), bool); out[ok] = self.mask[ij[ok, 0], ij[ok, 1]]
        return out

    # ── 입력
    def cb_l1(self, m):
        if self.lio is None:
            return
        P = read_xyz(m)
        if len(P) < 50:
            return
        R = self.leg[0] if self.leg_ok() else self.lio[0]
        roll = math.atan2(R[2, 1], R[2, 2]); pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
        fb = CB.estimate_floor_below(P, roll, pitch)
        S = CB.band_select(P, roll, pitch, fb)
        if len(S):
            Rw, tw = self.lio; m_ = self.in_mask(S @ Rw.T + tw)
            self.n['l1_mask'] += int(m_.sum()); S = S[~m_]
        self.n['l1'] += 1; self.n['l1_pts'] += len(S)
        self.remember(S)

    def cb_cam(self, m):
        P = read_xyz(m)
        self.n['cam'] += 1; self.n['cam_pts'] += len(P)
        self.remember(P, 'cam')

    def cb_wall(self, m):
        W = read_xyz(m)
        C = self.memory_world(W[:, :2] if len(W) else None)
        out = np.concatenate([W, C]) if len(C) else W
        self.pub.publish(make_cloud(out, m.header.frame_id or 'camera_init', m.header.stamp))
        self.n['wall'] += 1; self.n['out_pts'] += len(C)

    def report(self):
        n = self.n
        self.get_logger().info(f"5초: 벽 {n['wall']}장 · L1 {n['l1']}장 점 {n['l1_pts']}(경사로 마스크로 뺀 {n['l1_mask']}) · 카메라 콘 {n['cam']}장 점 {n['cam_pts']}"
                               f" · 내보낸 콘 점 평균 {n['out_pts'] / max(n['wall'], 1):.0f} · 기억 {self.mem_frame} · 다리 odom {'OK' if self.leg_ok() else '없음'}")
        if n['wall'] == 0:
            self.get_logger().warn('/mission/wall_cloud 가 5초간 안 옴 — ego 에 장애물이 안 간다(wall_filter 확인)')
        for k in n:
            n[k] = 0


def main():
    rclpy.init()
    node = ObstacleFusion()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        # ★ 11-05: SIGTERM(run_sim.sh/replay_obstacle_check.sh의 stop(), 또는 그냥 종료)이 오면
        # rclpy가 ExternalShutdownException을 던지는데 KeyboardInterrupt만 잡고 있어서 지저분한
        # traceback이 남았음(obstacle_check_035713/fuse.log에서 실제로 봄) — 정상 종료 경로라 조용히 처리.
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
