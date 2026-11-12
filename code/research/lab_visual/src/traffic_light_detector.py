#!/usr/bin/env python3
"""신호등을 보고 건너도 되는지 판정한다. 라이다가 못 하는 일은 이것 하나뿐이다.

내는 것
  traffic_light/state          std_msgs/String   RED | GREEN | UNKNOWN  (시간 필터를 거친 값)
  traffic_light/green_elapsed  std_msgs/Float32  초록불이 켜진 뒤 흐른 초. 모르면 -1
  traffic_light/cross_allowed  std_msgs/Bool     지금 출발하면 다 건널 수 있는가
  pedestrian/detections        geometry_msgs/PoseArray  카메라 프레임에서 본 보행자 (안전 확인용)
  traffic_light/annotated/compressed  sensor_msgs/CompressedImage  (--annotate)

받는 것
  이미지 토픽 + 카메라 정보

【왜 초록불만 보면 안 되는가】
규정은 "30초간 켜져 있는 초록불일 때만 건넌다" 다. 그런데 도착했을 때 이미 초록이면
**그게 켜진 지 1초인지 29초인지 알 수 없다.** 29초째에 출발하면 건너는 중에 빨간불이 된다.
그래서 이 노드는 빨강->초록 **전환을 직접 본 경우에만** 남은 시간을 안다고 판단한다.
전환을 못 봤으면 다음 주기를 기다린다. 한 주기 손해 보는 대신 확실하다.

【왜 한 프레임으로 판정하지 않는가】
사람이 앞을 가리거나 햇빛에 날아가면 한 프레임이 통째로 틀린다.
최근 몇 프레임 중 몇 개 이상이 같은 답을 낼 때만 상태를 바꾼다.

【보행자는 왜 그냥 내보내기만 하는가】
마네킹은 라이다에도 찍힌다. 팀원의 경로계획이 `/cloud_registered` 로 이미 피하고 있다.
여기서 내는 것은 그 위에 얹는 **확인용**이다 — 라이다가 놓친 경우를 사람이 알아채기 위해서다.
이 노드는 주행 명령을 내지 않는다.
"""
import argparse
import math
import os
import sys
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String, Float32, Bool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from traffic_light_logic import TrafficLightLogic, RED, GREEN, UNKNOWN


class TrafficLightDetector(Node):

    def __init__(self, a):
        super().__init__('traffic_light_detector')
        self.a = a
        self.fx = self.fy = self.cx = self.cy = None
        # 판정은 ROS 를 모르는 모듈이 한다. 그래야 사진만으로 시험할 수 있다.
        self.logic = TrafficLightLogic(a.vote_of, a.vote_need,
                                       a.green_seconds, a.cross_seconds, a.margin)
        self.n_in = self.n_run = 0
        self.ms = []
        self.last = 0.0

        import cv2
        self.cv2 = cv2
        from ultralytics import YOLO
        w = os.path.expanduser(a.weights)
        if not os.path.exists(w):
            raise SystemExit('★ 가중치가 없다: %s' % w)
        self.model = YOLO(w)
        self.names = self.model.names
        self.get_logger().info('모델 %s  클래스 %s  장치 %s'
                               % (os.path.basename(w), list(self.names.values()),
                                  a.device or 'auto'))
        self.get_logger().info(self.logic.explain())

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.compressed = a.topic.endswith('/compressed')
        self.create_subscription(CompressedImage if self.compressed else Image,
                                 a.topic, self.on_image, qos)
        info_topic = a.camera_info or (a.topic.rsplit('/', 2)[0] + '/camera_info')
        self.create_subscription(CameraInfo, info_topic, self.on_info, qos)

        self.pub_state = self.create_publisher(String, 'traffic_light/state', 10)
        self.pub_elapsed = self.create_publisher(Float32, 'traffic_light/green_elapsed', 10)
        self.pub_allow = self.create_publisher(Bool, 'traffic_light/cross_allowed', 10)
        self.pub_ped = self.create_publisher(PoseArray, 'pedestrian/detections', 10)
        self.pub_img = (self.create_publisher(CompressedImage,
                                              'traffic_light/annotated/compressed', 1)
                        if a.annotate else None)
        self.create_timer(5.0, self.report)

    def on_info(self, m):
        if self.fx is None and m.k[0] > 0:
            self.fx, self.fy = float(m.k[0]), float(m.k[4])
            self.cx, self.cy = float(m.k[2]), float(m.k[5])
            self.get_logger().info('카메라 정보: fx %.1f fy %.1f cx %.1f cy %.1f'
                                   % (self.fx, self.fy, self.cx, self.cy))

    def _fallback(self, w, h):
        if self.fx is None:
            self.fx = w / (2 * math.tan(math.radians(self.a.hfov) / 2))
            self.fy = self.fx
            self.cx, self.cy = w / 2.0, h / 2.0
            self.get_logger().warn('카메라 정보가 없다 — 화각 %.0f°로 초점거리 %.0f px 가정'
                                   % (self.a.hfov, self.fx))

    # ------------------------------------------------------------------ 판정 위임
    @property
    def state(self):
        return self.logic.state

    def judge(self):
        """지금 출발하면 다 건널 수 있는가. (건너도됨, 초록경과)"""
        if self.logic.state != GREEN or self.logic.green_since is None:
            return False, -1.0
        el = time.time() - self.logic.green_since
        return el <= self.logic.deadline, el

    # ------------------------------------------------------------------ 입력
    def on_image(self, msg):
        self.n_in += 1
        now = time.time()
        if now - self.last < 0.95 / self.a.hz:
            return
        self.last = now
        if self.compressed:
            bgr = self.cv2.imdecode(np.frombuffer(msg.data, np.uint8),
                                    self.cv2.IMREAD_COLOR)
        else:
            bgr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
            if msg.encoding == 'rgb8':
                bgr = bgr[:, :, ::-1]
        if bgr is None or bgr.size == 0:
            return
        h, w = bgr.shape[:2]
        self._fallback(w, h)

        t0 = time.perf_counter()
        r = self.model.predict(bgr, conf=self.a.conf, imgsz=self.a.imgsz,
                               device=self.a.device or None, verbose=False)[0]
        self.ms.append((time.perf_counter() - t0) * 1000)
        self.n_run += 1

        lights, peds = [], []
        for b in r.boxes:
            nm = self.names.get(int(b.cls), '')
            x1, y1, x2, y2 = [float(v) for v in b.xyxy[0].tolist()]
            c = float(b.conf)
            if nm in ('tl_red', 'tl_green'):
                if (y2 - y1) >= self.a.min_light_box:
                    lights.append((c, nm, x1, y1, x2, y2))
            elif nm == 'pedestrian':
                peds.append((c, x1, y1, x2, y2))
        lights.sort(reverse=True)

        raw = UNKNOWN if not lights else (RED if lights[0][1] == 'tl_red' else GREEN)
        before = self.logic.state
        _, el, ok, note = self.logic.update(raw, time.time())
        if self.logic.state != before:
            self.get_logger().info('신호 %s -> %s  %s' % (before, self.logic.state, note))

        self.pub_state.publish(String(data=self.state))
        self.pub_elapsed.publish(Float32(data=float(el)))
        self.pub_allow.publish(Bool(data=bool(ok)))

        pa = PoseArray()
        pa.header = msg.header
        for c, x1, y1, x2, y2 in peds:
            bh = y2 - y1
            if bh < self.a.min_ped_box:
                continue
            z = self.fy * self.a.pedestrian_height / bh
            if not (self.a.min_range < z < self.a.max_range):
                continue
            p = Pose()
            p.position.x = ((x1 + x2) / 2.0 - self.cx) * z / self.fx
            p.position.y = ((y1 + y2) / 2.0 - self.cy) * z / self.fy
            p.position.z = z
            p.orientation.w = c
            pa.poses.append(p)
        self.pub_ped.publish(pa)

        if self.pub_img is not None:
            self.annotate(bgr, lights, peds, ok, el, msg.header)

    def annotate(self, bgr, lights, peds, ok, el, header):
        cv2 = self.cv2
        im = bgr.copy()
        for c, nm, x1, y1, x2, y2 in lights:
            col = (60, 60, 235) if nm == 'tl_red' else (90, 210, 110)
            cv2.rectangle(im, (int(x1), int(y1)), (int(x2), int(y2)), col, 2)
            cv2.putText(im, '%s %.2f' % (nm, c), (int(x1), max(16, int(y1) - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
        for c, x1, y1, x2, y2 in peds:
            cv2.rectangle(im, (int(x1), int(y1)), (int(x2), int(y2)), (40, 170, 240), 2)
            z = self.fy * self.a.pedestrian_height / max(y2 - y1, 1)
            cv2.putText(im, 'ped %.1fm' % z, (int(x1), max(16, int(y1) - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (40, 170, 240), 2)
        col = (90, 210, 110) if ok else (60, 60, 235)
        txt = '%s  %s' % (self.state, 'CROSS OK' if ok else 'WAIT')
        if el >= 0:
            txt += '  green %.0fs' % el
        cv2.putText(im, txt, (10, 34), cv2.FONT_HERSHEY_SIMPLEX, 1.0, col, 2)
        okj, enc = cv2.imencode('.jpg', im, [cv2.IMWRITE_JPEG_QUALITY, self.a.jpeg_quality])
        if okj:
            m = CompressedImage()
            m.header = header
            m.format = 'jpeg'
            m.data = enc.tobytes()
            self.pub_img.publish(m)

    def report(self):
        if self.n_run:
            ok, el = self.judge()
            self.get_logger().info('영상 %.1fHz  추론 %.1fHz  %.0f ms   신호 %s  %s%s'
                                   % (self.n_in / 5.0, self.n_run / 5.0, float(np.mean(self.ms)),
                                      self.state, '건너도 됨' if ok else '대기',
                                      ('  (초록 %.0f초째)' % el) if el >= 0 else ''))
        else:
            self.get_logger().warn('영상 %.1fHz 인데 추론이 한 번도 안 돌았다' % (self.n_in / 5.0))
        self.n_in = self.n_run = 0
        self.ms = []



def parse_args(ap):
    """ROS 인자(--ros-args …)와 우리 인자를 갈라서 읽는다.

    argparse 는 --ros-args 를 모르는 인자로 보고 거절한다. 그래서 rclpy 의 유틸로
    ROS 인자를 먼저 걷어낸 뒤 나머지만 argparse 에 준다.
    ROS 인자는 rclpy.init(args=sys.argv) 로 그대로 넘겨야 use_sim_time 같은 게 먹는다.
    """
    from rclpy.utilities import remove_ros_args
    return ap.parse_args(remove_ros_args(sys.argv)[1:])

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--weights', default='~/ws/go2_vision/tl5_aug.pt',
                    help='젯슨에서는 ~/tl5_int8/tl5_aug.engine (INT8, 1.66배 빠름)')
    ap.add_argument('--topic', default='/camera_sync/color/image_raw/compressed')
    ap.add_argument('--camera-info', default='')
    ap.add_argument('--hz', type=float, default=5.0)
    ap.add_argument('--imgsz', type=int, default=1280)
    ap.add_argument('--conf', type=float, default=0.35)
    ap.add_argument('--device', default='', help='cuda:0 또는 cpu. 비우면 자동')
    ap.add_argument('--hfov', type=float, default=69.0)
    # 시간 필터
    ap.add_argument('--vote-of', type=int, default=5, help='최근 이만큼의 프레임을 본다')
    ap.add_argument('--vote-need', type=int, default=3, help='그중 이만큼이 같아야 상태를 바꾼다')
    # 규정
    ap.add_argument('--green-seconds', type=float, default=30.0, help='초록불 지속 시간 (규정)')
    ap.add_argument('--cross-seconds', type=float, default=12.0,
                    help='다 건너는 데 걸리는 시간. 실측해서 넣어야 한다')
    ap.add_argument('--margin', type=float, default=5.0, help='안전 여유')
    # 크기
    ap.add_argument('--min-light-box', type=float, default=14.0)
    ap.add_argument('--min-ped-box', type=float, default=40.0)
    ap.add_argument('--pedestrian-height', type=float, default=1.60,
                    help='마네킹 실제 키 (m). 거리 계산의 근거다')
    ap.add_argument('--min-range', type=float, default=0.5)
    ap.add_argument('--max-range', type=float, default=15.0)
    ap.add_argument('--annotate', action='store_true')
    ap.add_argument('--jpeg-quality', type=int, default=70)
    a = parse_args(ap)
    if a.vote_need > a.vote_of:
        raise SystemExit('★ --vote-need 가 --vote-of 보다 클 수 없다')
    if a.cross_seconds + a.margin >= a.green_seconds:
        raise SystemExit('★ 건너는 시간 %.0f + 여유 %.0f 가 초록 %.0f 초 이상이다 — 절대 못 건넌다'
                         % (a.cross_seconds, a.margin, a.green_seconds))
    rclpy.init(args=sys.argv)
    n = TrafficLightDetector(a)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass          # Ctrl-C 나 SIGTERM. 정상 종료로 본다
    n.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
