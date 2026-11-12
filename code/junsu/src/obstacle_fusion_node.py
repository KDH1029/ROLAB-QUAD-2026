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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray, Pose

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
        self.use_l1 = bool(p('use_l1', False).value)   # ★ 11-05 사용자: L1 헛장애물로 빈 길을 못 가는 위험 → 기본은 카메라만(L1 은 작전에서 켤 수 있음)
        self.use_cam = bool(p('use_cam', True).value)
        self.use_leg = bool(p('use_leg_odom', True).value)
        # ★ 11-05 review ego(merge_tmp) 용: 콘 중심을 PoseArray 로도 낸다 — review ego 는 콘을 벽과 따로(여유 0.28) 다룬다
        self.cones_topic = p('cones_topic', '/mission/cones').value
        self.cone_cluster_r = float(p('cone_cluster_r', 0.15).value)
        self.cone_min_vox = int(p('cone_min_vox', 3).value)
        self.pub_cones = self.create_publisher(PoseArray, self.cones_topic, 5) if self.cones_topic else None
        # ★ 11-05 merge-review: merge 콘 추적기(라이다, map 좌표)와 묶기 — 카메라가 고르고 라이다가 위치를 정함, 라이다만 본 콘(콘 구역)도 씀
        # ★ 11-05 뒤쪽 벽 기억: XT16 은 뒤 ±35° 를 못 보고(가림), L1·range_obstacle 도 뒤를 못 봄(녹화 확인) →
        #   본 벽을 5 cm 격자로 쌓아 두고, 로봇 뒤쪽 rear_half_deg 안 rear_range m 이내 벽 칸을 ego 추가 장애물로 낸다.
        self.wm_topic = p('wall_memory_topic', '/mission/wall_memory').value
        self.wm_range = float(p('rear_range', 1.5).value); self.wm_half = math.radians(float(p('rear_half_deg', 50.0).value))
        self.wm_cell = 0.05; self.wm_org = np.array([-10.0, -10.0]); self.wm = np.full((600, 600), -1e9)   # 30 m × 30 m, 칸마다 마지막으로 본 시각
        self.wm_ttl = float(p('memory_ttl', 20.0).value)   # ★ 이만큼 다시 못 본 칸은 잊음 — 위치가 흐르거나 넘어진 뒤 남는 '유령 벽'으로 고착되지 않게
        self.pub_wm = self.create_publisher(PointCloud2, self.wm_topic, 2) if self.wm_topic else None
        self.wl_range = float(p('local_range', 2.0).value)     # 몸 모양 안전 필터용: 2 m 안 전 방향 벽 기억
        self.pub_wl = self.create_publisher(PointCloud2, p('wall_local_topic', '/mission/wall_local').value, 2)
        self.lidar_cones_topic = p('lidar_cones_topic', '/mission/cones_lidar').value
        self.pick_r = float(p('lidar_pick_r', 0.30).value)
        self.lidar_cones = np.zeros((0, 2)); self.lidar_cones_t = -1e9; self.T = (0.0, 0.0, 0.0)
        if self.lidar_cones_topic:
            from std_msgs.msg import String
            lat = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(PoseArray, self.lidar_cones_topic, self.cb_lidar_cones, 5)
            self.create_subscription(String, '/mission/align_T', self.cb_T, lat)
        mask_path = os.path.expanduser(p('ramp_mask', '~/ws/junsu/maps/ramp_mask.npz').value)
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

    def cb_T(self, m):
        import json as _j
        try:
            d = _j.loads(m.data); self.T = tuple(float(v) for v in d['T'])
        except (ValueError, KeyError, TypeError):
            pass

    def cb_lidar_cones(self, m):
        """라이다 콘(map 좌표) → 주행 좌표(camera_init = T⁻¹·map)."""
        tx, ty, th = self.T; c, s_ = math.cos(th), math.sin(th)
        P = np.array([[q.position.x, q.position.y] for q in m.poses]).reshape(-1, 2)
        if m.header.frame_id not in ('map', ''):
            self.lidar_cones = P                         # 이미 주행 좌표로 온 경우
        else:
            d = P - np.array([tx, ty]); self.lidar_cones = np.c_[c * d[:, 0] + s_ * d[:, 1], -s_ * d[:, 0] + c * d[:, 1]]
        self.lidar_cones_t = time.monotonic()

    def fuse_lidar(self, cam):
        """카메라 콘 중심 + 라이다 콘 → 최종 콘. 카메라 콘 근처(pick_r) 라이다 콘이 있으면 라이다 위치, 남은 라이다 콘은 그대로 추가."""
        L = self.lidar_cones if time.monotonic() - self.lidar_cones_t < 1.0 else np.zeros((0, 2))
        if not len(L):
            return cam
        used = np.zeros(len(L), bool); out = []
        for p_ in cam:
            d = np.hypot(L[:, 0] - p_[0], L[:, 1] - p_[1]); d[used] = np.inf; j = int(np.argmin(d))
            if d[j] < self.pick_r:
                out.append(L[j]); used[j] = True; self.n['picked'] = self.n.get('picked', 0) + 1
            else:
                out.append(p_)
        out += [q for q, u in zip(L, used) if not u]
        self.n['lidar_only'] = int((~used).sum())
        return np.array(out).reshape(-1, 2)

    def cone_centers(self, C):
        """기억한 콘 점(세계 xy, 5 cm 칸) → 콘 중심들. 반지름 cone_cluster_r 안을 한 덩어리로, 칸 cone_min_vox 개 미만은 버림."""
        if not len(C):
            return np.zeros((0, 2))
        P = C[:, :2].copy(); left = np.ones(len(P), bool); out = []
        for i in range(len(P)):
            if not left[i]:
                continue
            d = np.hypot(P[:, 0] - P[i, 0], P[:, 1] - P[i, 1]); g = left & (d < 2 * self.cone_cluster_r)
            c = P[g].mean(0); g2 = left & (np.hypot(P[:, 0] - c[0], P[:, 1] - c[1]) < self.cone_cluster_r * 1.5)
            left &= ~(g | g2)
            if (g | g2).sum() >= self.cone_min_vox:
                out.append(P[g | g2].mean(0))
        return np.array(out).reshape(-1, 2)

    def wall_memory(self, W, frame, stamp):
        """벽 칸 쌓기 + 로봇 뒤쪽 벽 칸 내보내기(세계 좌표, 높이는 로봇 높이)."""
        if len(W):
            ij = np.floor((W[:, :2] - self.wm_org) / self.wm_cell).astype(int)
            ok = (ij >= 0).all(1) & (ij[:, 0] < self.wm.shape[0]) & (ij[:, 1] < self.wm.shape[1])
            self.wm[ij[ok, 0], ij[ok, 1]] = time.monotonic()
        if self.pub_wm is None or self.lio is None:
            return
        R_, t = self.lio; c0 = np.floor((t[:2] - self.wm_org) / self.wm_cell).astype(int); r = int(max(self.wm_range, self.wl_range) / self.wm_cell) + 1
        i0, j0 = max(c0[0] - r, 0), max(c0[1] - r, 0)
        sub = self.wm[i0:c0[0] + r + 1, j0:c0[1] + r + 1]
        ii, jj = np.nonzero(sub > time.monotonic() - self.wm_ttl)
        P = (np.c_[ii + i0, jj + j0] + 0.5) * self.wm_cell + self.wm_org
        L = P[np.hypot(P[:, 0] - t[0], P[:, 1] - t[1]) <= self.wl_range] if len(P) else P
        self.pub_wl.publish(make_cloud(np.c_[L, np.full(len(L), t[2])] if len(L) else np.zeros((0, 3)), frame, stamp))   # 빈 것도(벽 없음 = 검사 통과)
        if len(P):
            d = P - t[:2]; dist = np.hypot(d[:, 0], d[:, 1])
            yaw = math.atan2(R_[1, 0], R_[0, 0]); rel = np.arctan2(d[:, 1], d[:, 0]) - yaw
            rel = (rel + np.pi) % (2 * np.pi) - np.pi
            P = P[(dist <= self.wm_range) & (np.abs(rel) >= np.pi - self.wm_half)]
        self.pub_wm.publish(make_cloud(np.c_[P, np.full(len(P), t[2])] if len(P) else np.zeros((0, 3)), frame, stamp))
        self.n['rear'] = len(P)

    def cb_wall(self, m):
        W = read_xyz(m)
        self.wall_memory(W, m.header.frame_id or 'camera_init', m.header.stamp)
        C = self.memory_world(W[:, :2] if len(W) else None)
        out = np.concatenate([W, C]) if len(C) else W
        frame = m.header.frame_id or 'camera_init'
        self.pub.publish(make_cloud(out, frame, m.header.stamp))
        self.n['wall'] += 1; self.n['out_pts'] += len(C)
        if self.pub_cones is not None:   # 빈 배열도 낸다(콘이 사라지면 ego 가 지우게)
            pa = PoseArray(); pa.header = Header(frame_id=frame, stamp=m.header.stamp)
            for x, y in self.fuse_lidar(self.cone_centers(C)):
                q = Pose(); q.position.x = float(x); q.position.y = float(y); q.orientation.w = 1.0; pa.poses.append(q)
            self.pub_cones.publish(pa); self.n['cones'] = len(pa.poses)

    def report(self):
        n = self.n
        self.get_logger().info(f"5초: 벽 {n['wall']}장 · L1 {n['l1']}장 점 {n['l1_pts']}(경사로 마스크로 뺀 {n['l1_mask']}) · 카메라 콘 {n['cam']}장 점 {n['cam_pts']}"
                               f" · 내보낸 콘 점 평균 {n['out_pts'] / max(n['wall'], 1):.0f} · 콘 중심 {n.get('cones', 0)}개(라이다 위치로 고친 {n.get('picked', 0)} · 라이다만 {n.get('lidar_only', 0)}) · 뒤쪽 벽 기억 {n.get('rear', 0)}칸 · 기억 {self.mem_frame} · 다리 odom {'OK' if self.leg_ok() else '없음'}")
        if n['wall'] == 0:
            self.get_logger().warn('/mission/wall_cloud 가 5초간 안 옴 — ego 에 장애물이 안 간다(wall_filter 확인)')
        for k in n:
            n[k] = 0


def main():
    rclpy.init()
    node = ObstacleFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
