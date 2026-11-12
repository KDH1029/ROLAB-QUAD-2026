#!/usr/bin/env python3
"""신호등 색 인식 노드 — 2026 로보월드 4족보행 규정 5.2: 정지선에서 신호등이
녹색일 때만 출발 가능(신호는 5초 주기로 바뀜). /camera/color/image_raw(RGB8)를
구독해 YOLO로 tl_red/tl_green 객체를 찾고 std_msgs/String("RED"/"GREEN"/"UNKNOWN")
+ 신뢰도를 ~10Hz로 발행한다.

★ 2026-11-01: HSV 색분할 방식을 버리고 YOLO로 교체함. 팀원(Jetson Nano)이 만든
실제 신호등 데이터셋(~/ws/datasets/tl5, 1,345장, 5클래스)으로 둘을 직접
비교했더니:
  HSV(완화 임계값)  빨강 재현율 6.1%   초록 재현율 68.8%
  YOLO(1/60 epoch만 학습된 모델인데도)  빨강 93.9%   초록 81.2%
원인 조사: 실제 신호등은 전체 장면에서 아주 작게(14~33px) 찍히고, 라벨 박스
안에 LED 자체보다 하우징 반사/배경이 더 많이 섞여서 전역 HSV 임계값으로는
원천적으로 어려움 — 단순 색상 분할이 아니라 객체 인식이 필요한 문제였음.
자세한 분석은 ~/ws/junsu/README.md 'YOLO 실측' 섹션 참고.

★ 2026-11-04: 모델을 tl5cone_640(6클래스, cone 추가, 9/80 epoch에서 중단된
학습)로 교체. val셋(221장) 기준 재현율 비교:
  tl5_640_aug(1/60 epoch)   빨강 93.9%   초록 81.2%
  tl5cone_640(9/80 epoch)   빨강 96.3%   초록 89.2%   (오탐: 빨강 2장, 초록 5장 / 221장)
80 epoch까지 다 끝난 게 아니라 9 epoch짜리 중간 체크포인트인데도 이미 더 낫다 —
학습이 nuc16 공유 자원 문제로 재개 보류 중이라, 지금 있는 것 중 제일 좋은 걸 씀.
나중에 학습이 이어져서 더 좋은 체크포인트가 나오면 또 이 방식(eval_tl5cone.py로
클래스별 재현율 비교)으로 교체 여부 판단할 것.

모델: ~/ws/datasets/tl5/runs/tl5cone_640/weights/best.pt (YOLOv8n, 클래스
0=tl_red, 1=tl_green, 2=pedestrian, 3=temi, 4=unknown, 5=cone — 이 노드는 0/1만 본다).
실행 환경: 시스템 python3엔 ultralytics가 없다 — 반드시
~/ws/Rgosu/venv_yolo/bin/python3로 실행해야 함.

센서 비중 동적 전환 (기존과 동일): track_sequencer_node가 /mission/camera_active
(Bool)로 "지금 정지선 근처인가"를 알려준다. False면 이미지가 들어와도 추론
자체를 건너뛴다 — 오탐지 방지 + 라이다에 연산 자원 몰아주기 + YOLO 추론 자체가
HSV보다 훨씬 무거우니 더더욱 중요함. OFF로 전환되는 순간엔 UNKNOWN을 한 번
발행해서, 하위 상태기가 이전 정지선의 오래된 GREEN 값을 믿어버리는 일이 없게 한다.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
import numpy as np

DEFAULT_MODEL_PATH = '/home/ilovehan9283/ws/datasets/tl5/runs/tl5cone_640/weights/best.pt'
CLASS_RED = 0
CLASS_GREEN = 1


class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detector')

        self.declare_parameter('model_path', DEFAULT_MODEL_PATH)
        self.declare_parameter('conf_threshold', 0.25)
        self.declare_parameter('publish_rate_limit_hz', 10.0)
        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('conf_threshold').value

        from ultralytics import YOLO  # 여기서 import -- venv_yolo 파이썬으로만 실행된다는 전제
        self.model = YOLO(model_path)
        self.get_logger().info(f'YOLO 모델 로드 완료: {model_path} (클래스: {self.model.names})')

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(Image, '/camera/color/image_raw', self.cb_image, qos)
        self.sub_active = self.create_subscription(Bool, '/mission/camera_active', self.cb_active, 10)
        self.pub_state = self.create_publisher(String, '/mission/traffic_light_state', 10)

        self._last_pub_t = 0.0
        self._min_dt = 1.0 / self.publish_rate_limit_hz if self.publish_rate_limit_hz > 0 else 0.0
        self.camera_active = False  # track_sequencer가 켜주기 전까진 기본 OFF (자원 절약 쪽으로 안전)

        self.get_logger().info('traffic_light_detector(YOLO) 시작 — /camera/color/image_raw 구독 (camera_active=False 대기)')

    def cb_active(self, msg: Bool):
        if msg.data != self.camera_active:
            self.camera_active = msg.data
            self.get_logger().info(f'camera_active -> {self.camera_active}')
            if not self.camera_active:
                out = String()
                out.data = 'UNKNOWN|0.00'
                self.pub_state.publish(out)

    @property
    def publish_rate_limit_hz(self):
        return self.get_parameter('publish_rate_limit_hz').value

    def cb_image(self, msg: Image):
        if not self.camera_active:
            return  # 정지선 구간이 아니면 YOLO 추론 자체를 안 한다 (CPU 절약 + 오탐지 방지)

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t - self._last_pub_t < self._min_dt:
            return
        self._last_pub_t = t

        img_rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        # ultralytics는 BGR(cv2 관례)을 기대하므로 채널 순서를 뒤집어 준다
        img_bgr = img_rgb[:, :, ::-1]

        results = self.model.predict(img_bgr, verbose=False, conf=self.conf_threshold)

        best_red_conf, best_green_conf = 0.0, 0.0
        for r in results:
            for cls_id, conf in zip(r.boxes.cls.tolist(), r.boxes.conf.tolist()):
                cls_id = int(cls_id)
                if cls_id == CLASS_RED:
                    best_red_conf = max(best_red_conf, conf)
                elif cls_id == CLASS_GREEN:
                    best_green_conf = max(best_green_conf, conf)

        if best_red_conf > 0 and best_red_conf >= best_green_conf:
            state, confidence = 'RED', best_red_conf
        elif best_green_conf > 0:
            state, confidence = 'GREEN', best_green_conf
        else:
            state, confidence = 'UNKNOWN', 0.0

        out = String()
        out.data = f'{state}|{confidence:.2f}'
        self.pub_state.publish(out)


def main():
    rclpy.init()
    node = TrafficLightDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
