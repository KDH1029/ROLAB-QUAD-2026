#!/usr/bin/env python3
"""
Version-Agnostic Modular YOLO Vision Node for 2026 RoboWorld Competition.

Decouples the YOLO inference engine (ONNX / OpenCV DNN / Ultralytics / Mock)
from the competition mission post-processing handlers:
  1. TrafficLightHandler : Extracts Red / Green state inside detected traffic light BBox
  2. ConeHandler         : Back-projects 23cm cone BBox center with Depth to 3D PoseArray
  3. ParkingHandler      : Verifies 'P' sign BBox and publishes parking_ready flag
"""

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

# ==============================================================================
# 1. Engine Abstraction (YOLO Version Agnostic)
# ==============================================================================

class BaseYoloEngine:
    """Base class for interchangeable YOLO inference backends."""
    def predict(self, bgr_image: np.ndarray, conf_thresh: float):
        """Returns list of detections: [{'bbox': [x1, y1, x2, y2], 'conf': float, 'cls': int}]"""
        raise NotImplementedError


class MockYoloEngine(BaseYoloEngine):
    """Fallback engine used when weights are not yet loaded; enables full-pipeline testing."""
    def __init__(self):
        self.frame_idx = 0

    def predict(self, bgr_image: np.ndarray, conf_thresh: float):
        self.frame_idx += 1
        h, w = bgr_image.shape[:2]
        dets = []
        # Simulate traffic light in upper center ROI
        dets.append({
            'bbox': [int(w * 0.40), int(h * 0.15), int(w * 0.60), int(h * 0.45)],
            'conf': 0.88,
            'cls': 9  # traffic light
        })
        return dets


class OpenCvDnnEngine(BaseYoloEngine):
    """Native OpenCV DNN engine that runs any exported YOLO .onnx model without PyTorch."""
    def __init__(self, onnx_path: str, in_w=640, in_h=640, device="cpu"):
        if not os.path.exists(onnx_path):
            raise FileNotFoundError(f"ONNX model file not found: {onnx_path}")
        self.net = cv2.dnn.readNetFromONNX(onnx_path)
        if device == "cuda" and cv2.cuda.getCudaEnabledDeviceCount() > 0:
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
        else:
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        self.in_w = in_w
        self.in_h = in_h

    def predict(self, bgr_image: np.ndarray, conf_thresh: float):
        orig_h, orig_w = bgr_image.shape[:2]
        blob = cv2.dnn.blobFromImage(
            bgr_image, 1.0 / 255.0, (self.in_w, self.in_h), swapRB=True, crop=False
        )
        self.net.setInput(blob)
        preds = self.net.forward()

        # Handle standard YOLOv5 / v8 / v11 output shapes: (1, 84, 8400) or (1, 8400, 84)
        if preds.ndim == 3:
            if preds.shape[1] < preds.shape[2]:
                preds = np.transpose(preds, (0, 2, 1))
            preds = preds[0]

        boxes = []
        confidences = []
        class_ids = []

        x_factor = orig_w / float(self.in_w)
        y_factor = orig_h / float(self.in_h)

        for row in preds:
            classes_scores = row[4:]
            _, max_score, _, max_class_loc = cv2.minMaxLoc(classes_scores)
            if max_score >= conf_thresh:
                cx, cy, w, h = row[0], row[1], row[2], row[3]
                x1 = int((cx - 0.5 * w) * x_factor)
                y1 = int((cy - 0.5 * h) * y_factor)
                x2 = int((cx + 0.5 * w) * x_factor)
                y2 = int((cy + 0.5 * h) * y_factor)
                boxes.append([x1, y1, x2 - x1, y2 - y1])
                confidences.append(float(max_score))
                class_ids.append(int(max_class_loc[0]))

        indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_thresh, 0.45)
        results = []
        if len(indices) > 0:
            for i in indices.flatten():
                bx, by, bw, bh = boxes[i]
                results.append({
                    'bbox': [max(0, bx), max(0, by), min(orig_w, bx + bw), min(orig_h, by + bh)],
                    'conf': confidences[i],
                    'cls': class_ids[i]
                })
        return results


def create_engine(engine_type: str, model_path: str, in_w=640, in_h=640, device="cpu"):
    """Factory creating the appropriate backend engine with graceful fallback."""
    if engine_type == "mock":
        return MockYoloEngine()

    if os.path.isfile(model_path):
        if model_path.endswith(".onnx"):
            try:
                return OpenCvDnnEngine(model_path, in_w, in_h, device)
            except Exception as e:
                print(f"[WARN] Failed to load ONNX model via OpenCV DNN: {e}. Falling back to Mock.")
        elif model_path.endswith(".pt"):
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
                print(f"[WARN] Ultralytics import failed: {e}. Falling back to Mock.")

    print(f"[INFO] No valid YOLO model found at '{model_path}'. Running in Mock mode.")
    return MockYoloEngine()


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

        engine_type = self.get_parameter('engine_type').value
        model_path = self.get_parameter('model_path').value
        device = self.get_parameter('device').value
        in_w = self.get_parameter('input_width').value
        in_h = self.get_parameter('input_height').value
        self.conf_thresh = self.get_parameter('conf_threshold').value

        # Resolve model path relative to package root
        pkg_root = Path(__file__).resolve().parent.parent
        full_model_path = str(pkg_root / model_path) if not os.path.isabs(model_path) else model_path

        self.engine = create_engine(engine_type, full_model_path, in_w, in_h, device)
        self.tl_handler = TrafficLightHandler()
        self.cone_handler = ConeHandler()
        self.parking_handler = ParkingHandler()

        self.last_depth = None

        # Publishers
        self.tl_pub = self.create_publisher(String, self.get_parameter('traffic_light_topic').value, 10)
        self.cone_pub = self.create_publisher(PoseArray, self.get_parameter('cone_poses_topic').value, 10)
        self.park_pub = self.create_publisher(Bool, self.get_parameter('parking_ready_topic').value, 10)
        self.debug_pub = self.create_publisher(Image, self.get_parameter('debug_image_topic').value, 5)

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
        channels = 4 if 'a' in msg.encoding else 3
        if msg.step < msg.width * channels:
            return
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, channels))
        bgr = raw[:, :, :3] if msg.encoding.startswith('bgr') else cv2.cvtColor(raw[:, :, :3], cv2.COLOR_RGB2BGR)

        detections = self.engine.predict(bgr, self.conf_thresh)

        cone_poses = PoseArray()
        cone_poses.header = msg.header
        cone_poses.header.frame_id = "base_stab"

        parking_ready = False
        tl_state = "UNKNOWN"

        for det in detections:
            bbox = det['bbox']
            cls_id = det['cls']
            conf = det['conf']
            x1, y1, x2, y2 = bbox

            # Class 9 (COCO) or custom: Traffic Light
            if cls_id == 9:
                crop = bgr[y1:y2, x1:x2]
                tl_state = self.tl_handler.process(crop)
                cv2.rectangle(bgr, (x1, y1), (x2, y2), (0, 255, 255), 2)
                cv2.putText(bgr, f"TL:{tl_state}", (x1, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

            # Class 0: Traffic Cone (23cm)
            elif cls_id == 0:
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

            # Class 1: Parking Sign ('P' / QR)
            elif cls_id == 1:
                parking_ready = self.parking_handler.process(conf)
                color = (0, 255, 0) if parking_ready else (0, 0, 255)
                cv2.rectangle(bgr, (x1, y1), (x2, y2), color, 2)
                cv2.putText(bgr, f"Park:ready={parking_ready}", (x1, y1 - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Publish mission topics
        self.tl_pub.publish(String(data=tl_state))
        self.cone_pub.publish(cone_poses)
        self.park_pub.publish(Bool(data=parking_ready))

        # Publish visual debug image
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
