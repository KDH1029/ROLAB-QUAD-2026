#!/usr/bin/env python3
"""cone_fusion_node (초안, 미실행) — YOLO 박스 + D435i 깊이 → 콘 세계 좌표 → 세로 점기둥.

구독 (raw=True, 바이트를 numpy 로 직접 해석 — cdr_parse.py):
  /camera/color/image_raw  (sensor_msgs/Image rgb8 1280x720)
  /camera/depth/image_rect_raw (sensor_msgs/Image 16UC1 848x480, mm)
  /Odometry_mux (nav_msgs/Odometry, 세계 = camera_init, 몸체 = FAST-LIO body)
발행:
  /cone_fusion/points      sensor_msgs/PointCloud2 (camera_init) — 확인된 콘마다 반경 0.07 m · 지면 위 0~0.25 m 점기둥
  /cone_fusion/detections  std_msgs/String (JSON) — 이번 프레임 검출 전부(클래스, conf, 박스, 몸체·세계 좌표, 깊이점 수)
시각: 카메라 stamp 는 로봇 PC 시계 → (수신 − stamp) 최근 10 s 최솟값을 오프셋으로 온라인 추정 (+ delta −0.04 s, calib_depth_all.json).

실행 (사용자 터미널에서, 로봇 연결 상태 확인 후):
  source /opt/ros/jazzy/setup.bash
  ~/venv/go2/bin/python ~/ws/night_20261104/vision/yolo_depth/cone_fusion_node.py --ros-args -p torch_threads:=2
"""
import os, sys, json, time, collections
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import yd_core as C
from cdr_parse import parse_image, parse_odom, hdr_stamp

DEFAULTS = dict(
    model='/home/ilovehan9283/ws/datasets/tl5/runs/tl5cone_640/weights/best.pt',
    calib_json=C.CALIB_JSON, conf=0.4, imgsz=640, rate_hz=5.0, torch_threads=2,
    color_topic='/camera/color/image_raw', depth_topic='/camera/depth/image_rect_raw', odom_topic='/Odometry_mux',
    world_frame='camera_init', odom_time='stamp',          # 'stamp' = 헤더 시각, 'recv' = 수신 시각
    clock_win_s=10.0, delta_s=None,                        # None → calib_json 의 시각지연_s
    shrink=0.6, stride=2, min_pts=8, max_range=6.0, max_depth_dt=0.05,
    cone_cls=5, col_r=0.07, col_h=0.25,
    ground_mode='depth', body_height=0.31,                  # 'depth' = 박스 깊이점 z 하위 5%, 'body' = 몸체 z − body_height
    merge_r=0.3, min_hits=2, keep_s=30.0,
    shape_gate=False,                                      # True = 높이 0.12~0.35 m·폭 ≤0.30 m 만 콘으로 (실제 콘 재현율 미검증)
)


class FusionCore:
    """ROS 없는 처리부. 노드와 오프라인 시험(test_node_offline.py)이 같이 쓴다."""

    def __init__(self, P, model=None):
        self.P = P
        self.T_bd, _, dl = C.load_calib(P['calib_json'])
        self.delta = dl if P['delta_s'] is None else float(P['delta_s'])
        self.T_cd = C.T_copt_dopt()
        self.clock = C.ClockOffset(P['clock_win_s'])
        self.poses = C.PoseBuffer(maxlen=600)
        self.mem = C.ConeMemory(P['merge_r'], P['min_hits'], P['keep_s'])
        self.color = None                                   # (raw, recv)
        self.depth = collections.deque(maxlen=15)           # (stamp, recv, raw)
        self.last_color_stamp = None
        self.model = model
        self.t_inf = collections.deque(maxlen=200)

    # 콜백 — 해석은 최소로 (헤더 stamp 만)
    def on_color(self, raw, recv):
        self.color = (raw, recv)

    def on_depth(self, raw, recv):
        s = hdr_stamp(raw); self.clock.add(recv, s); self.depth.append((s, recv, raw))

    def on_odom(self, raw, recv):
        s, fid, ch, p, q = parse_odom(raw)
        self.poses.add(recv if self.P['odom_time'] == 'recv' else s, p, q)

    def infer(self, rgb):
        t0 = time.perf_counter()
        res = self.model.predict(np.ascontiguousarray(rgb[:, :, ::-1]), imgsz=self.P['imgsz'], conf=self.P['conf'], verbose=False)[0]
        self.t_inf.append(time.perf_counter() - t0)
        b = res.boxes
        return np.c_[b.cls.numpy(), b.conf.numpy(), b.xyxy.numpy()] if len(b) else np.zeros((0, 6))

    def step(self, now, dets=None):
        """타이머 한 번. 돌려줌: None(할 일 없음) 또는 dict(stamp, dets[...], cones[...], cloud (N,3), status)."""
        if self.color is None:
            return None
        raw, recv = self.color
        cs = hdr_stamp(raw)
        if cs == self.last_color_stamp:
            return None
        self.last_color_stamp = cs
        if dets is None:
            rgb = parse_image(raw); dets = self.infer(rgb)
        out = dict(stamp=None, dets=[], cones=[], cloud=np.zeros((0, 3), np.float32), status='ok')
        t_c = self.clock.to_nuc(cs)
        if t_c is None:
            out['status'] = 'no_clock'; return out
        out['stamp'] = t_c + self.delta
        if len(dets):
            if not self.depth:
                out['status'] = 'no_depth'; return out
            st = np.array([d[0] for d in self.depth]); j = int(np.argmin(np.abs(st - cs)))
            if abs(st[j] - cs) > self.P['max_depth_dt']:
                out['status'] = 'depth_dt'; return out
            t_nuc = self.clock.to_nuc(st[j]) + self.delta
            pose = self.poses.at(t_nuc)
            if pose is None:
                out['status'] = 'no_pose'; return out
            Rwb, pwb = pose
            D = parse_image(self.depth[j][2])
            rows = C.detect_to_world(dets, D, self.T_bd, Rwb, pwb, self.T_cd, self.P['shrink'], self.P['stride'], self.P['min_pts'])
            for r in rows:
                r.pop('p_opt', None)
                if (r['ok'] and r['cls'] == self.P['cone_cls'] and r['range'] <= self.P['max_range']
                        and (not self.P['shape_gate'] or r['shape_ok'])):
                    gz = r['ground_z'] if (self.P['ground_mode'] == 'depth' and r['ground_z'] is not None) else float(pwb[2] - self.P['body_height'])
                    r['ground_z_used'] = gz
                    self.mem.add(r['p_world'][:2], gz, t_nuc, r['conf'])
            out['dets'] = rows
            out['stamp'] = t_nuc
        act = self.mem.active(out['stamp'])
        out['cones'] = [dict(xy=c['xy'].round(3).tolist(), z0=round(float(c['z0']), 3), hits=c['hits'], conf=round(c['conf'], 2)) for c in act]
        if act:
            out['cloud'] = np.vstack([C.cone_column(c['xy'], c['z0'], self.P['col_r'], self.P['col_h']) for c in act]).astype(np.float32)
        return out


def make_cloud(pts, frame, stamp_msg):
    from sensor_msgs.msg import PointCloud2, PointField
    m = PointCloud2(); m.header.frame_id = frame; m.header.stamp = stamp_msg
    m.height = 1; m.width = int(len(pts))
    m.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyz')]
    m.is_bigendian = False; m.point_step = 12; m.row_step = 12 * m.width; m.is_dense = True
    m.data = np.ascontiguousarray(pts, np.float32).tobytes()
    return m


def to_time_msg(t):
    from builtin_interfaces.msg import Time
    s = int(np.floor(t)); return Time(sec=s, nanosec=int(round((t - s) * 1e9)) % 1000000000)


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image, PointCloud2
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String

    class ConeFusionNode(Node):
        def __init__(self):
            super().__init__('cone_fusion')
            P = {k: self.declare_parameter(k, v if v is not None else -999.0).value for k, v in DEFAULTS.items()}
            if P['delta_s'] == -999.0: P['delta_s'] = None
            try: os.nice(5)
            except OSError: pass
            import torch; torch.set_num_threads(int(P['torch_threads']))
            from ultralytics import YOLO
            model = YOLO(P['model']); model.predict(np.zeros((720, 1280, 3), np.uint8), imgsz=P['imgsz'], verbose=False)
            self.core = FusionCore(P, model); self.P = P
            now = lambda: self.get_clock().now().nanoseconds * 1e-9
            q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=2)
            self.create_subscription(Image, P['color_topic'], lambda m: self.core.on_color(m, now()), q, raw=True)
            self.create_subscription(Image, P['depth_topic'], lambda m: self.core.on_depth(m, now()), qos_profile_sensor_data, raw=True)
            self.create_subscription(Odometry, P['odom_topic'], lambda m: self.core.on_odom(m, now()), 50, raw=True)
            self.pub_pts = self.create_publisher(PointCloud2, '/cone_fusion/points', 2)
            self.pub_det = self.create_publisher(String, '/cone_fusion/detections', 10)
            self.create_timer(1.0 / P['rate_hz'], self.tick)
            self.n = 0; self.last_log = 0.0
            self.get_logger().info(f'cone_fusion 시작 conf {P["conf"]} {P["rate_hz"]} Hz delta {self.core.delta}')

        def tick(self):
            t = self.get_clock().now().nanoseconds * 1e-9
            try:
                out = self.core.step(t)
            except Exception as e:                       # 한 프레임 실패로 노드가 죽지 않게
                self.get_logger().warn(f'step 실패: {e}'); return
            if out is None: return
            st = to_time_msg(out['stamp'] if out['stamp'] else t)
            self.pub_pts.publish(make_cloud(out['cloud'], self.P['world_frame'], st))
            self.pub_det.publish(String(data=json.dumps(dict(stamp=out['stamp'], status=out['status'], dets=out['dets'], cones=out['cones']), ensure_ascii=False)))
            self.n += 1
            if t - self.last_log > 5.0:
                ti = self.core.t_inf; o = self.core.clock.offset()
                self.get_logger().info(f'프레임 {self.n} 콘 {len(out["cones"])} 상태 {out["status"]} 추론 중앙 {np.median(ti) * 1e3 if ti else 0:.0f} ms 시계오프셋 {o if o is None else round(o, 3)}')
                self.last_log = t

    rclpy.init()
    n = ConeFusionNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()
