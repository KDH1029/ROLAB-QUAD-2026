#!/usr/bin/env python3
"""신호등 인식 노드 — 내부 PC(Jetson, ROS 2 Foxy, Python 3.8) + TensorRT INT8 엔진판.

NUC 의 src/traffic_light_detector_node.py(ultralytics) 와 **주고받는 것이 똑같다** — 그래서
real_course_runner.py(정지선 → 2.5초 정지 → GREEN 1.0초 연속이면 출발 명령)는 손대지 않는다.
  받음  /mission/camera_active  std_msgs/Bool   러너가 정지선 도착 시 True, 출발 시 False
        /camera/color/image_raw  sensor_msgs/Image (같은 PC 의 D435i, BEST_EFFORT)
  냄    /mission/traffic_light_state  std_msgs/String  "RED|0.87" · "GREEN|0.91" · "UNKNOWN|0.00"
        /mission/traffic_light_debug  std_msgs/String  JSON(상자·지연) — 확인용, 작다
판정 규칙도 같다(tl_yolo_common.judge): 빨강·초록 중 가장 확실한 것, 동점이면 빨강, 없으면 UNKNOWN.
★ 11-05 뎁스 거리 필터: 판정 전에 신호등 상자를 뎁스로 재서 depth_max_m(6 m) 넘거나 못 잰 것은 버린다
  (경기장 뒤 전시장 조명 16~21 m 를 신호등으로 잡던 오판). 받음 /camera/depth/image_rect_raw + 두 camera_info.
  뎁스가 안 들어오면 필터 없이 예전처럼 판정하고 5초마다 경고(정지선에서 영원히 서 있는 것보다 낫다).

다른 점(내부 PC 라서):
- camera_active 가 False 인 동안은 **영상 구독 자체를 끊는다.** 30 Hz × 1.2 MB 를 파이썬이 받기만 해도
  CPU 를 먹는다(로봇 펌웨어가 도는 PC). True 가 되면 다시 구독한다.
- 결과는 std_msgs/String 만 쓴다(Foxy↔Jazzy 맞춤 메시지 깨짐 방지).
- jetson_vision.sh 가 nice 19 + taskset(코어 2개)으로 띄운다.
★ Python 3.8 호환으로 쓸 것.
"""
import json
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header
from std_msgs.msg import String, Bool

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import tl_yolo_common as C  # noqa: E402
import cone_cam_core as K  # noqa: E402

if os.environ.get('TL_FAKE_ONNX'):
    # NUC 시험용(TensorRT 없음): 같은 자리에 OpenCV DNN 으로 ONNX 를 돌린다. 노드의 ROS 배선만 시험하는 용도.
    import cv2

    class TrtEngine:
        def __init__(self, path):
            self.net = cv2.dnn.readNetFromONNX(os.environ['TL_FAKE_ONNX'])
            self.input_shape = (1, 3) + C.IMGSZ

        def infer(self, blob):
            self.net.setInput(blob)
            return self.net.forward()

        def close(self):
            pass
else:
    from trt_engine import TrtEngine  # noqa: E402


class TrafficLightTrt(Node):
    def __init__(self):
        super().__init__('traffic_light_detector')   # NUC 노드와 같은 이름 — 둘이 동시에 뜨면 로그에서 바로 보인다
        self.declare_parameter('engine_path', os.path.join(HERE, C.MODEL + '_384x640_int8.engine'))   # TL_MODEL 로 고름(기본 tl_cone_1104)
        self.declare_parameter('conf_threshold', 0.25)
        self.declare_parameter('publish_rate_limit_hz', 10.0)
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('always_on', False)   # 시험용: camera_active 없이도 계속 추론
        self.declare_parameter('depth_max_m', C.DEPTH_MAX_M)   # 0 이하면 뎁스 필터 끔
        self.declare_parameter('depth_topic', '/camera/depth/image_rect_raw')
        self.declare_parameter('depth_info_topic', '/camera/depth/camera_info')
        self.declare_parameter('color_info_topic', '/camera/color/camera_info')
        self.depth_max = float(self.get_parameter('depth_max_m').value)
        self.depth_msg = None
        self.Kd = self.Kc = None
        self.sub_depth = []
        self.n_gate = self.n_drop = self.n_nodepth = 0
        # ★ 11-05 콘 감지(항상 켬): 같은 추론으로 콘(6클래스 cls 5 · 4클래스 cls 3)도 — YOLO 콘 상자+뎁스(먼 콘) + 주황+뎁스(가까운 콘) → /mission/cone_cam(몸체 좌표)
        self.declare_parameter('cone_enable', True)
        self.declare_parameter('cone_hz', 5.0)          # 정지선 밖(신호등 안 볼 때) 추론 빈도 — 내부 PC 부하 조건(5 Hz)
        self.declare_parameter('cone_conf', 0.45)
        self.declare_parameter('cone_max_m', 3.5)
        self.declare_parameter('cone_topic', '/mission/cone_cam')
        self.cone_on = bool(self.get_parameter('cone_enable').value)
        self.cone_c = C.CONE   # 첫 추론에서 출력 모양으로 다시 정함
        self.cone_dt = 1.0 / max(float(self.get_parameter('cone_hz').value), 0.1)
        self.cone_conf = float(self.get_parameter('cone_conf').value)
        self.cone_max = float(self.get_parameter('cone_max_m').value)
        self.pub_cone = self.create_publisher(PointCloud2, self.get_parameter('cone_topic').value, 2)
        self.active = False
        self.n_cone_frames = self.n_cone_yolo = self.n_cone_orange = 0
        self.conf = float(self.get_parameter('conf_threshold').value)
        hz = float(self.get_parameter('publish_rate_limit_hz').value)
        self.min_dt = 1.0 / hz if hz > 0 else 0.0
        self.image_topic = self.get_parameter('image_topic').value
        path = self.get_parameter('engine_path').value
        self.eng = TrtEngine(path)
        self.H, self.W = self.eng.input_shape[2], self.eng.input_shape[3]
        self.eng.infer(np.zeros(self.eng.input_shape, np.float32))   # 예열
        self.get_logger().info('TensorRT 엔진 로드 완료: %s 입력 %dx%d (TL_MODEL=%s)' % (os.path.basename(path), self.W, self.H, C.MODEL))
        self.get_logger().info('카메라 장착값: %s · 갈라진 콘 합치기 %s' % ('16:50(수평, GO2_CAM_FIX=1650)' if K.CAM_FIX == '1650' else '11-04(아래 34°)', 'on' if K.CAM_FIX == '1650' else 'off'))

        self.qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.sub_img = None
        self.create_subscription(Bool, '/mission/camera_active', self.cb_active, 10)
        self.pub_state = self.create_publisher(String, '/mission/traffic_light_state', 10)
        self.pub_dbg = self.create_publisher(String, '/mission/traffic_light_debug', 10)
        self.last_t = 0.0
        self.n_in = self.n_run = 0
        self.ms = []
        self.state = 'UNKNOWN'
        self.create_timer(5.0, self.report)
        if self.cone_on:
            self.subscribe()       # 콘 감지 때문에 영상·뎁스는 늘 받는다(신호등 판정은 camera_active 일 때만)
        if self.get_parameter('always_on').value:
            self.set_active(True)
        self.get_logger().info('대기 중 — 콘 감지 %s(%.0f Hz) · /mission/camera_active=True 면 신호등 판정(%s)'
                               % ('켬' if self.cone_on else '끔', 1.0 / self.cone_dt, self.image_topic))

    def subscribe(self):
        if self.sub_img is not None:
            return
        self.sub_img = self.create_subscription(Image, self.image_topic, self.cb_image, self.qos)
        if self.depth_max > 0 or self.cone_on:
            g = self.get_parameter
            self.sub_depth = [self.create_subscription(Image, g('depth_topic').value, self.cb_depth, self.qos),
                              self.create_subscription(CameraInfo, g('depth_info_topic').value, self.cb_dinfo, self.qos),
                              self.create_subscription(CameraInfo, g('color_info_topic').value, self.cb_cinfo, self.qos)]

    def set_active(self, on):
        self.active = bool(on)
        if self.cone_on:           # 구독은 그대로(콘) — 신호등 판정만 켜고 끈다
            if not on:
                self.state = 'UNKNOWN'
                self.pub_state.publish(String(data='UNKNOWN|0.00'))
            self.get_logger().info('camera_active -> %s' % on)
            return
        if on and self.sub_img is None:
            self.subscribe()
        elif not on and self.sub_img is not None:
            self.destroy_subscription(self.sub_img)
            self.sub_img = None
            for sub in self.sub_depth:
                self.destroy_subscription(sub)
            self.sub_depth = []
            self.depth_msg = None
            self.state = 'UNKNOWN'
            self.pub_state.publish(String(data='UNKNOWN|0.00'))   # 묵은 GREEN 이 남지 않게(NUC 노드와 같은 동작)
        self.get_logger().info('camera_active -> %s' % on)

    def cb_active(self, msg):
        if bool(msg.data) != self.active:
            self.set_active(bool(msg.data))

    def cb_depth(self, msg):
        self.depth_msg = msg

    def cb_dinfo(self, msg):
        self.Kd = np.array(msg.k, np.float32).reshape(3, 3)

    def cb_cinfo(self, msg):
        self.Kc = np.array(msg.k, np.float32).reshape(3, 3)

    def fresh_depth(self, msg):
        """영상과 0.15 초 안의 뎁스(배열) — 없으면 None."""
        dm = self.depth_msg
        tc = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if dm is None or self.Kd is None or self.Kc is None or \
                abs(dm.header.stamp.sec + dm.header.stamp.nanosec * 1e-9 - tc) > 0.15:
            return None
        return np.frombuffer(dm.data, dtype=np.uint16).reshape(dm.height, dm.step // 2)[:, :dm.width]

    def depth_gate(self, dets, msg, D=None, proj=None):
        """신호등 상자를 뎁스로 재서 먼 것을 뺀다. 뎁스가 없거나 묵었으면(0.15 초) 그대로 + 셈."""
        if self.depth_max <= 0 or not any(d[0] in (C.RED, C.GREEN) for d in dets):
            return dets, []
        if D is None:
            D = self.fresh_depth(msg)
        if D is None:
            self.n_nodepth += 1
            return dets, []
        keep, info = C.gate_by_depth(dets, proj if proj is not None else C.project_depth(D, self.Kd, self.Kc), self.depth_max)
        self.n_gate += 1
        self.n_drop += sum(1 for x in info if not x[3])
        return keep, info

    def cones(self, dets, bgr, D, proj, msg):
        """콘 점(몸체 좌표) 발행 — YOLO 콘 상자+뎁스(가까운 면, 반지름 0.08 m 원) + 주황+뎁스 점."""
        pts = []
        if K.CAM_FIX == '1650':   # ★ 11-06(junsu_v2): 띠로 갈라진 큰 콘 상자 합치기 — 조각(0.30↑)끼리 합친 뒤 cone_conf 로 거름
            cb = [d for d in dets if d[0] == self.cone_c and d[1] >= 0.30]
            dets = [d for d in dets if d[0] != self.cone_c] + K.merge_stacked(cb, lambda a, b, c, d: C.cone_depth(proj, a, b, c, d))
        for c, s, x1, y1, x2, y2 in dets:
            if c != self.cone_c or s < self.cone_conf:
                continue
            z = C.cone_depth(proj, x1, y1, x2, y2)
            if not np.isfinite(z) or z > self.cone_max:
                continue
            u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            pc = np.array([(u - self.Kc[0, 2]) / self.Kc[0, 0] * z, (v - self.Kc[1, 2]) / self.Kc[1, 1] * z, z]) - np.array(K.DEPTH_TO_COLOR_T)
            pb = K.T_BODY_CAM[:, :3] @ pc + K.T_BODY_CAM[:, 3]
            ang = np.linspace(0, 2 * np.pi, 8, endpoint=False)
            pts.append(np.c_[pb[0] + np.r_[0, 0.08 * np.cos(ang)], pb[1] + np.r_[0, 0.08 * np.sin(ang)], np.full(9, pb[2])])   # 콘 몸통 앞면 — ego 여유가 따로 붙는다
            self.n_cone_yolo += 1
        orange, _ = K.cone_points(D, np.ascontiguousarray(bgr), self.Kd.astype(np.float64), self.Kc.astype(np.float64))
        if len(orange):
            pts.append(orange); self.n_cone_orange += len(orange)
        P = np.concatenate(pts).astype(np.float32) if pts else np.zeros((0, 3), np.float32)
        out = PointCloud2(header=Header(frame_id='body', stamp=msg.header.stamp), height=1, width=len(P), is_bigendian=False,
                          is_dense=True, point_step=12, row_step=12 * len(P),
                          fields=[PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyz')])
        out.data = P.tobytes()
        self.pub_cone.publish(out); self.n_cone_frames += 1

    def cb_image(self, msg):
        self.n_in += 1
        now = time.monotonic()
        if now - self.last_t < (self.min_dt if self.active else self.cone_dt):
            return
        if not self.active and not self.cone_on:
            return
        self.last_t = now
        t0 = time.perf_counter()
        a = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
        a = a.reshape(msg.height, msg.width, 3)
        bgr = a[:, :, ::-1] if msg.encoding == 'rgb8' else a
        blob, r, left, top = C.letterbox(bgr, (self.H, self.W))
        out = self.eng.infer(blob)
        dets = C.decode(out, r, left, top, (msg.height, msg.width), conf_thr=self.conf)
        self.cone_c = C.cone_cls(out)   # 6클래스 모델 5 · 4클래스 모델 3
        D = self.fresh_depth(msg)
        proj = C.project_depth(D, self.Kd, self.Kc) if D is not None and any(d[0] in (C.RED, C.GREEN, self.cone_c) for d in dets) else None
        if self.cone_on and D is not None:
            self.cones(dets, bgr, D, proj, msg)
        if not self.active:
            self.ms.append((time.perf_counter() - t0) * 1000); self.n_run += 1
            return
        dets, dinfo = self.depth_gate(dets, msg, D, proj)
        state, conf = C.judge(dets)
        ms = (time.perf_counter() - t0) * 1000
        self.ms.append(ms)
        self.n_run += 1
        if state != self.state:
            self.get_logger().info('신호 %s -> %s (%.2f)' % (self.state, state, conf))
            self.state = state
        self.pub_state.publish(String(data='%s|%.2f' % (state, conf)))
        self.pub_dbg.publish(String(data=json.dumps({
            'state': state, 'conf': round(conf, 3), 'ms': round(ms, 1), 'stamp': [msg.header.stamp.sec, msg.header.stamp.nanosec],
            'boxes': [[c, round(s, 3), int(x1), int(y1), int(x2), int(y2)] for c, s, x1, y1, x2, y2 in dets
                      if c in (C.RED, C.GREEN)][:8],
            'depth': [[c, round(s, 3), (round(z, 2) if np.isfinite(z) else None), ok] for c, s, z, ok in dinfo][:8]})))

    def report(self):
        if self.sub_img is None:
            return
        if self.n_run:
            self.get_logger().info('영상 %.1fHz  추론 %.1fHz  %.1f ms(전·추론·후 합)  신호 %s(%s)  뎁스필터 %d장(먼 상자 %d개 버림)  콘 %d장(YOLO %d · 주황 점 %d)'
                                   % (self.n_in / 5.0, self.n_run / 5.0, float(np.mean(self.ms)), self.state, '판정 중' if self.active else '안 봄',
                                      self.n_gate, self.n_drop, self.n_cone_frames, self.n_cone_yolo, self.n_cone_orange))
            if self.cone_on and self.n_cone_frames == 0:
                self.get_logger().warn('콘 감지 켰는데 5초간 콘 발행 0 — 뎁스(%s)·camera_info 확인' % self.get_parameter('depth_topic').value)
            if self.n_nodepth:
                self.get_logger().warn('신호등을 봤는데 뎁스가 없어서 필터 없이 판정한 장 %d — %s 와 camera_info 가 나오는지 확인'
                                       % (self.n_nodepth, self.get_parameter('depth_topic').value))
        else:
            self.get_logger().warn('camera_active 인데 5초간 영상 %d장 — %s 가 나오는지 확인' % (self.n_in, self.image_topic))
        self.n_in = self.n_run = 0
        self.n_gate = self.n_drop = self.n_nodepth = 0
        self.n_cone_frames = self.n_cone_yolo = self.n_cone_orange = 0
        self.ms = []


def main():
    rclpy.init(args=sys.argv)
    node = TrafficLightTrt()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.eng.close()
        node.destroy_node()
        if rclpy.ok():          # Ctrl-C 로 이미 내려갔으면 다시 부르지 않음(Foxy 에는 try_shutdown 없음)
            rclpy.shutdown()


if __name__ == '__main__':
    main()
