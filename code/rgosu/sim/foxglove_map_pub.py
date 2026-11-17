#!/usr/bin/env python3
"""Foxglove 로 지도 보기용 발행기 — 격리 도메인(foxglove_map_view.sh 가 79·lo 로 띄움). 주행 토픽은 하나도 안 낸다(전부 /viz/...).
  /viz/map_1459        14:59 FAST-LIO 3D 지도(4 cm, 필드 height = 바닥 위 m) — 10초마다
  /viz/map_baseline_1459  10-30 '베이스라인(통맵핑)' 방식 — FAST-LIO 가 직접 쌓은 지도(gosu1030_baseline.yaml, 8 cm)
  /viz/map_1459_15cm   같은 지도를 FAST-LIO 내장 filter_size_map(0.15 m)처럼 15 cm 칸 평균으로 — 내부 지도 모양
  /viz/sim_walls       시뮬레이션 벽(편집본이 있으면 편집본)
  /viz/walls_on_path   실제 wall_filter 가 로봇이 지나간 길 위(20 cm 안)에서 벽으로 판정한 칸 — 계단 턱·경사로 의심(시뮬레이션 벽에선 뺌)
  /viz/path_human_1459 14:59 사람이 몬 경로(FAST-LIO)      /viz/path_sim  마지막 시뮬레이션 경로
  /viz/waypoints       저장 좌표(구 + 이름)
고정 프레임: map (Foxglove 3D 패널 Display frame = map)."""
import os, glob, json, sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header

S = os.path.dirname(os.path.abspath(__file__))
d = np.load(os.path.expanduser('~/ws/v_gosu/eval/out/arena1104b_map.npz')); V = float(d['V'])
P = (d['vox'][d['cnt'] >= 2] + 0.5) * V; od = d['odom']; od = od[np.argsort(od[:, 1])]
lo, hi = od[:, 2:4].min(0) - 2.5, od[:, 2:4].max(0) + 2.5; FLOOR = np.percentile(od[:, 4], 50) - 0.31
P = P[(P[:, 0] > lo[0]) & (P[:, 0] < hi[0]) & (P[:, 1] > lo[1]) & (P[:, 1] < hi[1]) & (P[:, 2] > FLOOR - 0.3) & (P[:, 2] < FLOOR + 2.5)]
u = np.unique(np.floor(P / 0.04).astype(np.int32), axis=0); MAP = ((u + 0.5) * 0.04).astype(np.float32)
MAP = np.c_[MAP, MAP[:, 2] - FLOOR].astype(np.float32)
# FAST-LIO 내장 설정(filter_size_map 0.15 m)과 같은 방식: 15 cm 칸마다 점들의 평균 하나 — 내부 지도가 실제로 이렇게 생겼다
k15 = np.floor(P / 0.15).astype(np.int64); u15, inv = np.unique(k15, axis=0, return_inverse=True); inv = inv.ravel()
M15 = np.zeros((len(u15), 3)); np.add.at(M15, inv, P); M15 /= np.bincount(inv)[:, None]
MAP15 = np.c_[M15, M15[:, 2] - FLOOR].astype(np.float32)
sys.path.insert(0, os.path.expanduser('~/ws/Rgosu/tools'))
from pcd_wall_sharpness import read_pcd
BASEP = os.path.expanduser('~/ws/Rgosu/maps/fastlio_baseline_1459/baseline_map.pcd')
BASE = np.zeros((0, 4), np.float32)
if os.path.exists(BASEP):
    _b = read_pcd(BASEP); _b = _b[np.isfinite(_b).all(1)]; BASE = np.c_[_b, _b[:, 2] - FLOOR].astype(np.float32)
WALLS = np.load(os.path.join(S, 'sim_walls_edited.npy') if os.path.exists(os.path.join(S, 'sim_walls_edited.npy')) else os.path.join(S, 'sim_walls.npy'))
ONPATH = np.load(os.path.join(S, 'sim_walls_onpath.npy')) if os.path.exists(os.path.join(S, 'sim_walls_onpath.npy')) else np.zeros((0, 2))
HUMAN = od[::5, 2:5]
runs = [r for r in sorted(glob.glob(os.path.join(S, 'runs', '*', 'events.jsonl')), key=os.path.getmtime) if 'infl0.30' in r] or sorted(glob.glob(os.path.join(S, 'runs', '*', 'events.jsonl')), key=os.path.getmtime)   # 완주한 0.30 실행 우선
SIM = np.array([[e['x'], e['y']] for e in map(json.loads, open(runs[-1])) if e['ev'] == 'pose']) if runs else np.zeros((0, 2))
WP = json.load(open(os.path.expanduser('~/ws/Rgosu/maps/real_waypoints_venue.json')))
F4 = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate(('x', 'y', 'z', 'height'))]


class Pub(Node):
    def __init__(self):
        super().__init__('viz_map_pub')
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.p_map = self.create_publisher(PointCloud2, '/viz/map_1459', q)
        self.p_map15 = self.create_publisher(PointCloud2, '/viz/map_1459_15cm', q)
        self.p_base = self.create_publisher(PointCloud2, '/viz/map_baseline_1459', q)
        self.p_wall = self.create_publisher(PointCloud2, '/viz/sim_walls', q)
        self.p_on = self.create_publisher(PointCloud2, '/viz/walls_on_path', q)
        self.p_h = self.create_publisher(Path, '/viz/path_human_1459', q)
        self.p_s = self.create_publisher(Path, '/viz/path_sim', q)
        self.p_wp = self.create_publisher(MarkerArray, '/viz/waypoints', q)
        self.create_timer(10.0, self.send); self.send()
        self.get_logger().info(f'지도 {len(MAP)}점(15 cm 판 {len(MAP15)} · 10-30 방식 베이스라인 {len(BASE)}) · 벽 {len(WALLS)}점 · 사람 경로 {len(HUMAN)} · 시뮬레이션 경로 {len(SIM)} ({os.path.basename(os.path.dirname(runs[-1])) if runs else "-"}) · 좌표 {len(WP)}')

    def hdr(self):
        h = Header(); h.frame_id = 'map'; h.stamp = self.get_clock().now().to_msg(); return h

    def path(self, xyz):
        m = Path(); m.header = self.hdr()
        for p in xyz:
            ps = PoseStamped(); ps.header = m.header; ps.pose.position.x, ps.pose.position.y = float(p[0]), float(p[1])
            ps.pose.position.z = float(p[2]) if len(p) > 2 else 0.0; ps.pose.orientation.w = 1.0; m.poses.append(ps)
        return m

    def send(self):
        self.p_map.publish(pc2.create_cloud(self.hdr(), F4, MAP))
        self.p_map15.publish(pc2.create_cloud(self.hdr(), F4, MAP15))
        if len(BASE): self.p_base.publish(pc2.create_cloud(self.hdr(), F4, BASE))
        W = np.c_[WALLS[:, :3], WALLS[:, 2]].astype(np.float32)
        self.p_wall.publish(pc2.create_cloud(self.hdr(), F4, W))
        On = np.c_[ONPATH[:, :2], np.full(len(ONPATH), FLOOR + 0.25), np.full(len(ONPATH), 0.25)].astype(np.float32) if len(ONPATH) else np.zeros((0, 4), np.float32)
        self.p_on.publish(pc2.create_cloud(self.hdr(), F4, On))
        self.p_h.publish(self.path(HUMAN)); self.p_s.publish(self.path(np.c_[SIM, np.zeros(len(SIM))] if len(SIM) else SIM))
        ma = MarkerArray()
        for i, w in enumerate(WP):
            for k, (typ, sz) in enumerate(((Marker.SPHERE, 0.18), (Marker.TEXT_VIEW_FACING, 0.25))):
                m = Marker(); m.header = self.hdr(); m.ns = 'wp'; m.id = 2 * i + k; m.type = typ; m.action = Marker.ADD
                m.pose.position.x, m.pose.position.y, m.pose.position.z = float(w['x']), float(w['y']), float(FLOOR + (0.15 if k == 0 else 0.6))
                m.pose.orientation.w = 1.0; m.scale.x = m.scale.y = m.scale.z = sz; m.color.r, m.color.a = 1.0, 1.0
                if k: m.text = w['label']; m.color.g = m.color.b = 1.0
                ma.markers.append(m)
        self.p_wp.publish(ma)


def main():
    rclpy.init(args=sys.argv); n = Pub()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
