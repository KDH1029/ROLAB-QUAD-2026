#!/usr/bin/env python3
"""
Version-Agnostic Modular YOLO Vision Node for 2026 RoboWorld Competition.

Decouples the YOLO inference engine (ONNX / OpenCV DNN / Ultralytics / Mock)
from the competition mission post-processing handlers:
  1. TrafficLightHandler : Extracts Red / Green state inside detected traffic light BBox
  2. ConeHandler         : Back-projects 23cm cone BBox center with Depth to 3D PoseArray
  3. ParkingHandler      : Verifies 'P' sign BBox and publishes parking_ready flag
"""

import copy
import json
import math
import os
import sys
from pathlib import Path

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Bool, String
try:
    from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
    HAS_VISION_MSGS = True
except ImportError:
    HAS_VISION_MSGS = False

# ==============================================================================
# 1. Engine Abstraction (YOLO Version Agnostic)
# ==============================================================================

class BaseYoloEngine:
    """Base class for interchangeable YOLO inference backends."""
    def predict(self, bgr_image: np.ndarray, conf_thresh: float):
        """Returns list of detections: [{'bbox': [x1, y1, x2, y2], 'conf': float, 'cls': int}]"""
        raise NotImplementedError


class MockYoloEngine(BaseYoloEngine):
    """Dummy engine for pipeline tests only (engine_type:=mock)."""
    def __init__(self, tl_cls=0):
        self.frame_idx = 0
        self.tl_cls = tl_cls

    def predict(self, bgr_image: np.ndarray, conf_thresh: float):
        self.frame_idx += 1
        h, w = bgr_image.shape[:2]
        dets = []
        # Simulate traffic light in upper center ROI
        dets.append({
            'bbox': [int(w * 0.40), int(h * 0.15), int(w * 0.60), int(h * 0.45)],
            'conf': 0.88,
            'cls': self.tl_cls
        })
        return dets


def onnx_class_names(onnx_path: str):
    """Class names from an Ultralytics ONNX export's metadata ("names": "{0: 'traffic_light', ...}"),
    read from the raw file so no onnx package is needed. {} when absent."""
    import re
    try:
        raw = open(onnx_path, 'rb').read()
    except OSError:
        return {}
    m = re.search(rb"\{(\s*\d+\s*:\s*'[^']*'\s*,?)+\s*\}", raw)
    if not m:
        return {}
    return {int(k): v.decode() for k, v in re.findall(rb"(\d+)\s*:\s*'([^']*)'", m.group(0))}


def decode_yolo(preds: np.ndarray, num_classes: int, layout: str, conf_thresh: float, iou: float,
                sx: float, sy: float, orig_w: int, orig_h: int, px: float = 0.0, py: float = 0.0):
    """Raw YOLO output -> [{'bbox': [x1, y1, x2, y2], 'conf', 'cls'}] in original image pixels.

    Layouts (layout:=auto picks by shape; the exporter decides, not us):
      v8   (1, 4+nc, N) or (1, N, 4+nc): cx cy w h + class scores      (YOLOv8 / v11 / v12)
      v5   (1, N, 5+nc): cx cy w h objectness + class scores           (YOLOv5 / v7)
      e2e  (1, N, 6): x1 y1 x2 y2 score class, NMS already done         (YOLOv10, nms=True exports)
    Network-input pixel (u, v) -> original ((u - px) * sx, (v - py) * sy): px, py = letterbox padding.
    NMS per class, like Ultralytics."""
    p = np.asarray(preds, dtype=np.float32)
    while p.ndim > 2:
        p = p[0]
    if p.ndim != 2:
        raise ValueError(f'unexpected YOLO output shape {np.shape(preds)}')
    a, b = p.shape
    if layout == 'auto':
        if min(a, b) in (6, 7) and max(a, b) <= 1000 and (num_classes <= 0 or num_classes + 4 != min(a, b)):
            layout = 'e2e'
        else:
            c = min(a, b)
            if num_classes > 0 and c == 5 + num_classes:
                layout = 'v5'
            else:
                layout = 'v8'
    if layout in ('v8', 'v5') and a < b:
        p = p.T                                   # channels-first export
    results = []
    if layout == 'e2e':
        if p.shape[1] not in (6, 7) and p.shape[0] in (6, 7):
            p = p.T
        for r in p:
            if r[4] < conf_thresh:
                continue
            x1, y1, x2, y2 = (r[0] - px) * sx, (r[1] - py) * sy, (r[2] - px) * sx, (r[3] - py) * sy
            results.append({'bbox': [max(0, int(x1)), max(0, int(y1)), min(orig_w, int(x2)), min(orig_h, int(y2))],
                            'conf': float(r[4]), 'cls': int(r[5])})
        return results
    if layout == 'v5':
        scores = p[:, 5:] * p[:, 4:5]
    else:
        scores = p[:, 4:]
    cls = np.argmax(scores, axis=1)
    conf = scores[np.arange(len(scores)), cls]
    keep = conf >= conf_thresh
    if not keep.any():
        return results
    q, cls, conf = p[keep], cls[keep], conf[keep]
    boxes = [[int((r[0] - 0.5 * r[2] - px) * sx), int((r[1] - 0.5 * r[3] - py) * sy), int(r[2] * sx), int(r[3] * sy)] for r in q]
    idx = []
    for c in np.unique(cls):                      # per class: a cone box must not suppress a light
        sel = np.nonzero(cls == c)[0]
        k = cv2.dnn.NMSBoxes([boxes[i] for i in sel], conf[sel].tolist(), conf_thresh, iou)
        idx += [int(sel[j]) for j in np.array(k).flatten()]
    for i in sorted(idx, key=lambda i: -conf[i]):
        bx, by, bw, bh = boxes[i]
        results.append({'bbox': [max(0, bx), max(0, by), min(orig_w, bx + bw), min(orig_h, by + bh)],
                        'conf': float(conf[i]), 'cls': int(cls[i])})
    return results


class OnnxYoloEngine(BaseYoloEngine):
    """.onnx through onnxruntime (CUDA/TensorRT providers when present, e.g. on the Jetson), else
    OpenVINO, else OpenCV DNN. OpenCV < 4.7 cannot load YOLOv8 heads (its Add broadcast fails:
    the 11-04 laptop with system OpenCV 4.6 could not load yolo_mission.onnx), so it is the last try."""
    def __init__(self, onnx_path: str, in_w=640, in_h=640, device="cpu", num_classes=0, layout="auto", iou=0.45):
        if not os.path.exists(onnx_path):
            raise FileNotFoundError(f"ONNX model file not found: {onnx_path}")
        self.in_w, self.in_h, self.layout, self.iou = in_w, in_h, layout, iou
        self.names = onnx_class_names(onnx_path)
        self.num_classes = num_classes or len(self.names)
        errors = []
        self.backend = None
        try:
            import onnxruntime as ort
            prov = [p for p in ('TensorrtExecutionProvider', 'CUDAExecutionProvider') if device == 'cuda'
                    and p in ort.get_available_providers()] + ['CPUExecutionProvider']
            self.sess = ort.InferenceSession(onnx_path, providers=prov)
            self.inp = self.sess.get_inputs()[0].name
            self.backend = f"onnxruntime({self.sess.get_providers()[0]})"
        except Exception as e:
            errors.append(f"onnxruntime: {e}")
        if self.backend is None:
            try:
                import openvino as ov
                core = ov.Core()
                self.ov = core.compile_model(onnx_path, 'GPU' if device == 'gpu' else 'CPU')
                self.backend = "openvino"
            except Exception as e:
                errors.append(f"openvino: {e}")
        if self.backend is None:
            try:
                self.net = cv2.dnn.readNetFromONNX(onnx_path)
                if device == "cuda" and cv2.cuda.getCudaEnabledDeviceCount() > 0:
                    self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
                    self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
                self.backend = f"opencv_dnn({cv2.__version__})"
            except Exception as e:
                errors.append(f"opencv {cv2.__version__}: {str(e).splitlines()[0]}")
        if self.backend is None:
            raise RuntimeError("no backend could load the model -> " + " | ".join(errors) +
                               " (install onnxruntime, or use the .pt / TensorRT .engine with ultralytics)")

    def letterbox(self, bgr_image: np.ndarray):
        """Resize keeping the aspect ratio, pad with 114 gray - the way Ultralytics trains and runs
        its models. A plain stretch to 640x640 shifted boxes and confidences (11-04 check against
        ultralytics: 43 % of its boxes matched with the stretch)."""
        h, w = bgr_image.shape[:2]
        r = min(self.in_w / w, self.in_h / h)
        nw, nh = int(round(w * r)), int(round(h * r))
        px, py = (self.in_w - nw) / 2.0, (self.in_h - nh) / 2.0
        img = cv2.resize(bgr_image, (nw, nh), interpolation=cv2.INTER_LINEAR) if (nw, nh) != (w, h) else bgr_image
        top, left = int(round(py - 0.1)), int(round(px - 0.1))
        img = cv2.copyMakeBorder(img, top, self.in_h - nh - top, left, self.in_w - nw - left,
                                 cv2.BORDER_CONSTANT, value=(114, 114, 114))
        return img, r, left, top

    def raw(self, bgr_image: np.ndarray):
        img, self._r, self._px, self._py = self.letterbox(bgr_image)
        blob = cv2.dnn.blobFromImage(img, 1.0 / 255.0, (self.in_w, self.in_h), swapRB=True, crop=False)
        if self.backend.startswith('onnxruntime'):
            return self.sess.run(None, {self.inp: blob})[0]
        if self.backend == 'openvino':
            return list(self.ov([blob]).values())[0]
        self.net.setInput(blob)
        return self.net.forward()

    def predict(self, bgr_image: np.ndarray, conf_thresh: float):
        h, w = bgr_image.shape[:2]
        out = self.raw(bgr_image)
        return decode_yolo(out, self.num_classes, self.layout, conf_thresh, self.iou,
                           1.0 / self._r, 1.0 / self._r, w, h, self._px, self._py)


OpenCvDnnEngine = OnnxYoloEngine   # old name


def create_engine(engine_type: str, model_path: str, in_w=640, in_h=640, device="cpu", tl_cls=0,
                  num_classes=0, layout="auto"):
    """Factory creating the backend engine. Fails loudly unless mock is requested."""
    if engine_type == "mock":
        return MockYoloEngine(tl_cls)

    if os.path.isfile(model_path):
        if model_path.endswith(".onnx"):
            return OnnxYoloEngine(model_path, in_w, in_h, device, num_classes, layout)
        elif model_path.endswith(".pt") or model_path.endswith(".engine"):   # .engine = TensorRT (ultralytics)
            try:
                from ultralytics import YOLO
                class UltralyticsEngine(BaseYoloEngine):
                    def __init__(self, pt_path):
                        self.model = YOLO(pt_path)
                    def predict(self, bgr_image, conf_thresh):
                        res = self.model.predict(bgr_image, conf=conf_thresh, verbose=False)[0]
                        dets = []
                        for box in res.boxes:
                            xyxy = [int(v) for v in box.xyxy[0].cpu().numpy()]
                            dets.append({'bbox': xyxy, 'conf': float(box.conf[0]), 'cls': int(box.cls[0])})
                        return dets
                return UltralyticsEngine(model_path)
            except Exception as e:
                raise RuntimeError(f"Ultralytics load failed: {e}") from e

    # Mock 은 engine_type="mock" 으로 명시했을 때만. 실기에서 조용히 Mock 으로 바뀌면
    # 화면 중앙의 아무 초록색이나 GREEN 으로 판정해 신호를 무시하고 출발한다.
    raise RuntimeError(f"YOLO model could not be loaded: '{model_path}' (set engine_type:=mock for pipeline tests)")


# ==============================================================================
# 2. Mission Post-Processing Handlers
# ==============================================================================

class TrafficLightHandler:
    """Interprets Red / Green states within detected traffic light bounding boxes."""
    def __init__(self):
        self.state_history = []

    def process(self, bgr_crop: np.ndarray) -> str:
        if bgr_crop.size == 0:
            return "UNKNOWN"
        hsv = cv2.cvtColor(bgr_crop, cv2.COLOR_BGR2HSV)
        h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
        sat_val_mask = (s > 100) & (v > 100)

        red_mask = sat_val_mask & ((h <= 15) | (h >= 165))
        green_mask = sat_val_mask & (h >= 45) & (h <= 95)

        r_cnt = int(np.count_nonzero(red_mask))
        g_cnt = int(np.count_nonzero(green_mask))

        raw = "UNKNOWN"
        if g_cnt > 30 and g_cnt > r_cnt * 1.3:
            raw = "GREEN"
        elif r_cnt > 30 and r_cnt > g_cnt * 1.3:
            raw = "RED"

        self.state_history.append(raw)
        if len(self.state_history) > 5:
            self.state_history.pop(0)

        g_votes = self.state_history.count("GREEN")
        r_votes = self.state_history.count("RED")
        if g_votes >= 3:
            return "GREEN"
        elif r_votes >= 3:
            return "RED"
        return "UNKNOWN"


class ConeHandler:
    """Back-projects 23cm traffic cone bounding boxes to 3D coordinates using aligned Depth."""
    def __init__(self, fx=605.0, fy=605.0, cx=320.0, cy=240.0):
        self.fx, self.fy = fx, fy
        self.cx, self.cy = cx, cy

    def process(self, bbox, depth_img: np.ndarray):
        x1, y1, x2, y2 = bbox
        u = int((x1 + x2) / 2)
        v = int((y1 + y2) / 2)

        dh, dw = depth_img.shape[:2]
        if not (0 <= u < dw and 0 <= v < dh):
            return None

        # Sample 5x5 center patch to avoid depth noise / holes
        patch = depth_img[max(0, v-2):min(dh, v+3), max(0, u-2):min(dw, u+3)]
        valid = patch[patch > 0.20]
        if len(valid) == 0:
            return None
        z_m = float(np.median(valid))
        if depth_img.dtype == np.uint16:
            z_m *= 0.001  # convert mm to meters

        if not (0.30 <= z_m <= 4.0):
            return None

        # Camera frame (Z forward, X right) -> Robot body frame base_stab (X forward, Y left)
        robot_x = z_m + 0.25
        robot_y = -((u - self.cx) * z_m / self.fx)
        robot_z = 0.115  # Half of 23cm cone height
        return robot_x, robot_y, robot_z


class ParkingHandler:
    """Evaluates parking sign confidence and validates bay clearance."""
    def __init__(self):
        self.front_wall_dist = 1.0

    def set_scan(self, msg: LaserScan):
        if not msg.ranges:
            return
        fov = math.radians(20.0)
        min_d = 99.9
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            ang = msg.angle_min + i * msg.angle_increment
            if abs(ang) <= fov and r < min_d:
                min_d = r
        self.front_wall_dist = min_d

    def process(self, conf: float) -> bool:
        dist_ok = (0.35 <= self.front_wall_dist <= 1.35)
        return bool(conf >= 0.40 and dist_ok)


# ==============================================================================
# 3. Main ROS 2 Node
# ==============================================================================

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')

        self.declare_parameter('engine_type', 'auto')
        self.declare_parameter('model_path', 'models/yolo_mission.onnx')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('input_width', 640)
        self.declare_parameter('input_height', 640)
        self.declare_parameter('conf_threshold', 0.35)
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_rect_raw')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('traffic_light_topic', '/mission/traffic_light')
        self.declare_parameter('cone_poses_topic', '/mission/cone_poses')
        self.declare_parameter('parking_ready_topic', '/mission/parking_ready')
        self.declare_parameter('debug_image_topic', '/mission/yolo_debug_image')
        # Raw cone boxes for cone_tracker_node (lidar/depth fusion, world-frame tracking)
        self.declare_parameter('cone_detections_topic', '/mission/cone_detections')
        # yolo_mission.onnx 메타데이터 names = {0: traffic_light, 1: traffic_cone, 2: parking_sign}
        self.declare_parameter('class_traffic_light', 0)
        self.declare_parameter('class_cone', 1)
        self.declare_parameter('class_parking', 2)
        # 추론 주기 상한 (Hz). 카메라 30Hz 를 전부 추론하면 CPU 를 다 먹는다. 0 이면 제한 없음.
        self.declare_parameter('max_infer_hz', 5.0)
        self.declare_parameter('publish_debug_image', False)
        # Output tensor layout of the model: auto | v8 | v5 | e2e (see decode_yolo); 0 classes = from metadata
        self.declare_parameter('output_layout', 'auto')
        self.declare_parameter('num_classes', 0)
        # Cone boxes for cone_tracker_node: "string" (std_msgs/String JSON - works between Foxy on the
        # robot PC and Jazzy on the NUC; vision_msgs differs between them), "vision_msgs", or "both"
        self.declare_parameter('detections_format', 'string')
        self.declare_parameter('cone_detections_json_topic', '/mission/cone_detections_json')

        engine_type = self.get_parameter('engine_type').value
        model_path = self.get_parameter('model_path').value
        device = self.get_parameter('device').value
        in_w = self.get_parameter('input_width').value
        in_h = self.get_parameter('input_height').value
        self.conf_thresh = self.get_parameter('conf_threshold').value
        self.cls_tl = int(self.get_parameter('class_traffic_light').value)
        self.cls_cone = int(self.get_parameter('class_cone').value)
        self.cls_park = int(self.get_parameter('class_parking').value)
        hz = float(self.get_parameter('max_infer_hz').value)
        self.min_period_ns = int(1e9 / hz) if hz > 0 else 0
        self.last_infer_ns = 0
        self.publish_debug = bool(self.get_parameter('publish_debug_image').value)

        # 상대경로는 설치된 share/mission_vision 기준 (symlink-install 여부와 무관)
        if os.path.isabs(model_path):
            full_model_path = model_path
        else:
            from ament_index_python.packages import get_package_share_directory
            full_model_path = os.path.join(get_package_share_directory('mission_vision'), model_path)

        self.engine = create_engine(engine_type, full_model_path, in_w, in_h, device, self.cls_tl,
                                    int(self.get_parameter('num_classes').value),
                                    self.get_parameter('output_layout').value)
        names = getattr(self.engine, 'names', {})
        if names:
            self.get_logger().info(f"model classes {names} | traffic_light={self.cls_tl} cone={self.cls_cone} parking={self.cls_park}")
        self.tl_handler = TrafficLightHandler()
        self.cone_handler = ConeHandler()
        self.parking_handler = ParkingHandler()

        self.last_depth = None

        # Publishers
        self.tl_pub = self.create_publisher(String, self.get_parameter('traffic_light_topic').value, 10)
        self.cone_pub = self.create_publisher(PoseArray, self.get_parameter('cone_poses_topic').value, 10)
        self.park_pub = self.create_publisher(Bool, self.get_parameter('parking_ready_topic').value, 10)
        self.debug_pub = self.create_publisher(Image, self.get_parameter('debug_image_topic').value, 5)
        fmt = self.get_parameter('detections_format').value
        self.det_pub = None
        if fmt in ('vision_msgs', 'both'):
            if HAS_VISION_MSGS:
                self.det_pub = self.create_publisher(
                    Detection2DArray, self.get_parameter('cone_detections_topic').value, 10)
            else:
                self.get_logger().warn("vision_msgs not installed: cone boxes only as JSON")
        self.det_json_pub = None
        if fmt in ('string', 'both') or self.det_pub is None:
            self.det_json_pub = self.create_publisher(
                String, self.get_parameter('cone_detections_json_topic').value, 10)

        # Subscribers
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, self.get_parameter('image_topic').value, self.on_image, qos)
        self.create_subscription(Image, self.get_parameter('depth_topic').value, self.on_depth, qos)
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value, self.on_scan, qos)

        self.get_logger().info(
            f"YOLO Vision Node initialized [Engine: {self.engine.__class__.__name__}, Model: {Path(full_model_path).name}]"
        )

    def on_depth(self, msg: Image):
        # Convert depth data to numpy array
        if msg.encoding in ("16UC1", "mono16"):
            self.last_depth = np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width))
        elif msg.encoding in ("32FC1",):
            self.last_depth = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))

    def on_scan(self, msg: LaserScan):
        self.parking_handler.set_scan(msg)

    def on_image(self, msg: Image):
        now_ns = self.get_clock().now().nanoseconds
        if self.min_period_ns and now_ns - self.last_infer_ns < self.min_period_ns:
            return
        self.last_infer_ns = now_ns
        channels = 4 if 'a' in msg.encoding else 3
        if msg.step < msg.width * channels:
            return
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, channels))
        bgr = raw[:, :, :3] if msg.encoding.startswith('bgr') else cv2.cvtColor(raw[:, :, :3], cv2.COLOR_RGB2BGR)

        detections = self.engine.predict(bgr, self.conf_thresh)

        dets = Detection2DArray() if self.det_pub else None
        if dets is not None:
            dets.header = msg.header
        json_boxes = []
        cone_poses = PoseArray()
        cone_poses.header = copy.deepcopy(msg.header)   # a plain assignment shares the image header:
        cone_poses.header.frame_id = "base_stab"        # its frame_id then became base_stab for every user

        parking_ready = False
        tl_state = "UNKNOWN"

        for det in detections:
            bbox = det['bbox']
            cls_id = det['cls']
            conf = det['conf']
            x1, y1, x2, y2 = bbox

            if cls_id == self.cls_tl:
                crop = bgr[y1:y2, x1:x2]
                tl_state = self.tl_handler.process(crop)
                cv2.rectangle(bgr, (x1, y1), (x2, y2), (0, 255, 255), 2)
                cv2.putText(bgr, f"TL:{tl_state}", (x1, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

            elif cls_id == self.cls_cone:
                json_boxes.append([x1, y1, x2, y2, round(float(conf), 3)])
                if dets is not None:
                    d = Detection2D()
                    d.header = msg.header
                    d.bbox.center.position.x = 0.5 * (x1 + x2)
                    d.bbox.center.position.y = 0.5 * (y1 + y2)
                    d.bbox.size_x = float(x2 - x1)
                    d.bbox.size_y = float(y2 - y1)
                    h = ObjectHypothesisWithPose()
                    h.hypothesis.class_id = 'cone'
                    h.hypothesis.score = float(conf)
                    d.results.append(h)
                    dets.detections.append(d)
                cv2.rectangle(bgr, (x1, y1), (x2, y2), (0, 140, 255), 2)
                if self.last_depth is not None:
                    res3d = self.cone_handler.process(bbox, self.last_depth)
                    if res3d is not None:
                        rx, ry, rz = res3d
                        p = Pose()
                        p.position.x, p.position.y, p.position.z = rx, ry, rz
                        p.orientation.w = 1.0
                        cone_poses.poses.append(p)
                        cv2.putText(bgr, f"Cone:({rx:.1f},{ry:.1f})m", (x1, y1 - 6),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 140, 255), 2)

            elif cls_id == self.cls_park:
                parking_ready = self.parking_handler.process(conf)
                color = (0, 255, 0) if parking_ready else (0, 0, 255)
                cv2.rectangle(bgr, (x1, y1), (x2, y2), color, 2)
                cv2.putText(bgr, f"Park:ready={parking_ready}", (x1, y1 - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Publish mission topics
        self.tl_pub.publish(String(data=tl_state))
        self.cone_pub.publish(cone_poses)
        if dets is not None:
            self.det_pub.publish(dets)
        if self.det_json_pub is not None:
            # image stamp kept: cone_tracker maps it to its own clock and picks depth / LiDAR at it
            self.det_json_pub.publish(String(data=json.dumps({
                'stamp': [msg.header.stamp.sec, msg.header.stamp.nanosec], 'frame': msg.header.frame_id,
                'width': msg.width, 'height': msg.height, 'cones': json_boxes}, separators=(',', ':'))))
        self.park_pub.publish(Bool(data=parking_ready))

        if not self.publish_debug:
            return
        debug_msg = Image()
        debug_msg.header = msg.header
        debug_msg.height, debug_msg.width = bgr.shape[:2]
        debug_msg.encoding = "bgr8"
        debug_msg.step = msg.width * 3
        debug_msg.data = bgr.tobytes()
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
