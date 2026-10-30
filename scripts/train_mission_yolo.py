#!/usr/bin/env python3
"""
scripts/train_mission_yolo.py
RoboWorld Quadruped 2026 - Synthetic Dataset Generator & YOLOv8n Fine-Tuning Pipeline

Generates a realistic annotated dataset for 3 competition classes:
  0: traffic_light (1.3m tall stand, Red/Green lamps)
  1: traffic_cone  (23cm small rubber cone with reflective stripes)
  2: parking_sign  (10cm blue 'P' square badge / QR pattern)

Trains YOLOv8n via transfer learning and exports OpenCV DNN-compatible ONNX.
"""

import os
import shutil
import random
import cv2
import numpy as np

WORKSPACE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DATASET_DIR = os.path.join(WORKSPACE_DIR, "dataset")
MODELS_DIR = os.path.join(WORKSPACE_DIR, "src/missions/mission_vision/models")


# ==============================================================================
# 1. Realistic Procedural Object Renderers
# ==============================================================================

def render_traffic_cone(canvas, cx, cy, height, aspect=0.55):
    """
    Renders a standard 23cm competition traffic cone with perspective.
    Returns: (x1, y1, x2, y2) bounding box.
    """
    w = int(height * aspect)
    h = int(height)
    x1 = cx - w // 2
    x2 = cx + w // 2
    y1 = cy - h
    y2 = cy

    # Base trapezoid (black / dark gray plastic base)
    base_w = int(w * 1.35)
    base_h = max(3, int(h * 0.12))
    pts_base = np.array([
        [cx - base_w // 2, y2],
        [cx + base_w // 2, y2],
        [cx + int(base_w * 0.4), y2 - base_h],
        [cx - int(base_w * 0.4), y2 - base_h]
    ], dtype=np.int32)
    cv2.fillPoly(canvas, [pts_base], (35, 35, 35))

    # Conical body (orange trapezoid/triangle)
    cone_color = (15, random.randint(90, 130), random.randint(220, 255))
    top_w = max(2, int(w * 0.18))
    pts_cone = np.array([
        [cx - w // 2, y2 - base_h],
        [cx + w // 2, y2 - base_h],
        [cx + top_w // 2, y1],
        [cx - top_w // 2, y1]
    ], dtype=np.int32)
    cv2.fillPoly(canvas, [pts_cone], cone_color)

    # Reflective white collars (1 or 2 bands)
    if h > 20:
        band_y_top = y1 + int(h * 0.35)
        band_y_bot = y1 + int(h * 0.55)
        w_top = int(w * 0.45)
        w_bot = int(w * 0.65)
        pts_band = np.array([
            [cx - w_top // 2, band_y_top],
            [cx + w_top // 2, band_y_top],
            [cx + w_bot // 2, band_y_bot],
            [cx - w_bot // 2, band_y_bot]
        ], dtype=np.int32)
        cv2.fillPoly(canvas, [pts_band], (235, 238, 240))

    # Shading on one side for 3D realism
    overlay = canvas.copy()
    shadow_pts = np.array([
        [cx, y1],
        [cx + top_w // 2, y1],
        [cx + w // 2, y2 - base_h],
        [cx, y2 - base_h]
    ], dtype=np.int32)
    cv2.fillPoly(overlay, [shadow_pts], (0, 0, 0))
    cv2.addWeighted(overlay, 0.2, canvas, 0.8, 0, canvas)

    bbox_x1 = max(0, cx - base_w // 2 - 1)
    bbox_y1 = max(0, y1 - 1)
    bbox_x2 = min(canvas.shape[1] - 1, cx + base_w // 2 + 1)
    bbox_y2 = min(canvas.shape[0] - 1, y2 + 1)
    return [bbox_x1, bbox_y1, bbox_x2, bbox_y2]


def render_traffic_light(canvas, cx, cy, height, state="GREEN"):
    """
    Renders a 1.3m competition traffic light stand.
    Returns: (x1, y1, x2, y2) bounding box.
    """
    w = int(height * 0.38)
    h = int(height)
    x1 = cx - w // 2
    x2 = cx + w // 2
    y1 = cy - h
    y2 = cy

    # Mounting pole underneath
    pole_w = max(3, int(w * 0.15))
    pole_h = int(h * 1.5)
    cv2.rectangle(canvas, (cx - pole_w // 2, y2), (cx + pole_w // 2, min(canvas.shape[0]-1, y2 + pole_h)), (70, 70, 75), -1)

    # Dark housing box
    cv2.rectangle(canvas, (x1, y1), (x2, y2), (25, 25, 25), -1)
    cv2.rectangle(canvas, (x1, y1), (x2, y2), (60, 60, 60), 1)

    # 3 circular lamps: Red (top), Yellow (mid), Green (bot)
    radius = max(2, int(w * 0.28))
    centers = [
        (cx, y1 + int(h * 0.22)),
        (cx, y1 + int(h * 0.50)),
        (cx, y1 + int(h * 0.78))
    ]

    # Red
    red_color = (30, 30, 255) if state == "RED" else (15, 15, 60)
    cv2.circle(canvas, centers[0], radius, red_color, -1)
    if state == "RED":
        cv2.circle(canvas, centers[0], radius + 2, (100, 100, 255), 1)

    # Yellow
    cv2.circle(canvas, centers[1], radius, (15, 60, 70), -1)

    # Green
    green_color = (40, 240, 60) if state == "GREEN" else (15, 60, 20)
    cv2.circle(canvas, centers[2], radius, green_color, -1)
    if state == "GREEN":
        cv2.circle(canvas, centers[2], radius + 2, (120, 255, 140), 1)

    bbox_x1 = max(0, x1 - 1)
    bbox_y1 = max(0, y1 - 1)
    bbox_x2 = min(canvas.shape[1] - 1, x2 + 1)
    bbox_y2 = min(canvas.shape[0] - 1, y2 + 1)
    return [bbox_x1, bbox_y1, bbox_x2, bbox_y2]


def render_parking_sign(canvas, cx, cy, size, mode="P"):
    """
    Renders a 10cm competition parking marker (Blue square 'P' or QR badge).
    Returns: (x1, y1, x2, y2) bounding box.
    """
    half = size // 2
    x1 = cx - half
    x2 = cx + half
    y1 = cy - half
    y2 = cy + half

    # Blue square background
    cv2.rectangle(canvas, (x1, y1), (x2, y2), (200, 75, 10), -1)
    cv2.rectangle(canvas, (x1, y1), (x2, y2), (245, 245, 245), max(1, size // 16))

    if mode == "P":
        # White bold 'P'
        scale = size / 38.0
        thickness = max(2, int(size * 0.12))
        cv2.putText(canvas, "P", (cx - int(half * 0.55), cy + int(half * 0.55)),
                    cv2.FONT_HERSHEY_DUPLEX, scale, (255, 255, 255), thickness, cv2.LINE_AA)
    else:
        # QR pattern badge
        inner = max(4, size // 4)
        cv2.rectangle(canvas, (x1 + 3, y1 + 3), (x1 + 3 + inner, y1 + 3 + inner), (0, 0, 0), -1)
        cv2.rectangle(canvas, (x2 - 3 - inner, y1 + 3), (x2 - 3, y1 + 3 + inner), (0, 0, 0), -1)
        cv2.rectangle(canvas, (x1 + 3, y2 - 3 - inner), (x1 + 3 + inner, y2 - 3), (0, 0, 0), -1)

    bbox_x1 = max(0, x1 - 1)
    bbox_y1 = max(0, y1 - 1)
    bbox_x2 = min(canvas.shape[1] - 1, x2 + 1)
    bbox_y2 = min(canvas.shape[0] - 1, y2 + 1)
    return [bbox_x1, bbox_y1, bbox_x2, bbox_y2]


# ==============================================================================
# 2. Synthetic Scene Generator with Varied Backgrounds & Lighting
# ==============================================================================

def create_synthetic_frame(scene_type: str, img_w=640, img_h=640):
    """
    Creates a full 640x640 frame with background floor, track markings, and objects.
    Returns: (image_bgr, list_of_yolo_labels)
    """
    canvas = np.full((img_h, img_w, 3), random.randint(140, 180), dtype=np.uint8)

    # Draw floor horizon and perspective grid / track carpet
    horizon_y = random.randint(180, 260)
    wall_color = (random.randint(190, 220), random.randint(190, 220), random.randint(195, 225))
    floor_color = (random.randint(80, 110), random.randint(85, 115), random.randint(85, 115))
    canvas[:horizon_y, :] = wall_color
    canvas[horizon_y:, :] = floor_color

    # Add track lane lines (white / yellow tape)
    if random.random() > 0.3:
        lane_pts_l = np.array([[0, img_h], [int(img_w * 0.25), horizon_y], [int(img_w * 0.30), horizon_y], [int(img_w * 0.1), img_h]])
        lane_pts_r = np.array([[img_w, img_h], [int(img_w * 0.75), horizon_y], [int(img_w * 0.70), horizon_y], [int(img_w * 0.9), img_h]])
        cv2.fillPoly(canvas, [lane_pts_l], (240, 240, 240))
        cv2.fillPoly(canvas, [lane_pts_r], (240, 240, 240))

    labels = []  # entries: [class_id, x_center, y_center, w, h] (normalized)

    if scene_type == "traffic_light":
        # Stop line horizontal bar
        stop_line_y = random.randint(img_h - 140, img_h - 60)
        cv2.rectangle(canvas, (0, stop_line_y), (img_w, stop_line_y + 20), (245, 245, 245), -1)

        # Traffic light stand (centered or slightly off to the side)
        tl_x = random.randint(int(img_w * 0.35), int(img_w * 0.65))
        tl_y = random.randint(int(horizon_y * 0.9), int(horizon_y * 1.3))
        tl_h = random.randint(70, 130)
        state = random.choice(["RED", "GREEN"])
        bbox = render_traffic_light(canvas, tl_x, tl_y, tl_h, state=state)
        labels.append((0, bbox))

        # Maybe 1 or 2 distant cones near edges
        if random.random() > 0.5:
            cx = random.randint(50, 150)
            cy = random.randint(horizon_y + 40, horizon_y + 120)
            ch = random.randint(30, 55)
            labels.append((1, render_traffic_cone(canvas, cx, cy, ch)))

    elif scene_type == "slalom_cones":
        # 3 to 6 staggered cones along corridor
        num_cones = random.randint(3, 6)
        y_positions = np.linspace(horizon_y + 30, img_h - 40, num_cones)
        for i, cy in enumerate(y_positions):
            # Stagger left and right of center
            offset = -random.randint(60, 180) if (i % 2 == 0) else random.randint(60, 180)
            cx = int(img_w * 0.5 + offset)
            cx = max(40, min(img_w - 40, cx))
            # Perspective height: closer cones are larger
            rel_depth = (cy - horizon_y) / float(img_h - horizon_y)
            ch = int(25 + rel_depth * 85)
            bbox = render_traffic_cone(canvas, cx, int(cy), ch)
            labels.append((1, bbox))

    elif scene_type == "parking":
        # Parking bay box
        p_cx = random.randint(int(img_w * 0.4), int(img_w * 0.6))
        p_cy = random.randint(int(img_h * 0.45), int(img_h * 0.70))
        p_size = random.randint(40, 75)
        bbox = render_parking_sign(canvas, p_cx, p_cy, p_size, mode=random.choice(["P", "P", "QR"]))
        labels.append((2, bbox))

        # 2 bounding cones on sides of parking spot
        cone_l_x = p_cx - random.randint(int(p_size * 1.5), int(p_size * 2.2))
        cone_r_x = p_cx + random.randint(int(p_size * 1.5), int(p_size * 2.2))
        cone_y = p_cy + random.randint(10, 40)
        cone_h = int(p_size * 0.9)
        labels.append((1, render_traffic_cone(canvas, cone_l_x, cone_y, cone_h)))
        labels.append((1, render_traffic_cone(canvas, cone_r_x, cone_y, cone_h)))

    else:  # Mixed scene
        if random.random() > 0.4:
            tl_x = random.randint(100, 220)
            tl_y = random.randint(horizon_y - 20, horizon_y + 40)
            labels.append((0, render_traffic_light(canvas, tl_x, tl_y, random.randint(60, 100), state=random.choice(["RED", "GREEN"]))))
        for _ in range(random.randint(1, 3)):
            cx = random.randint(80, img_w - 80)
            cy = random.randint(horizon_y + 40, img_h - 40)
            labels.append((1, render_traffic_cone(canvas, cx, cy, random.randint(35, 75))))

    # Add realistic sensor noise & lighting jitter
    noise = np.random.normal(0, random.uniform(2, 6), canvas.shape).astype(np.int16)
    canvas = np.clip(canvas.astype(np.int16) + noise, 0, 255).astype(np.uint8)

    # Random slight motion blur
    if random.random() > 0.6:
        ksize = random.choice([3, 5])
        canvas = cv2.GaussianBlur(canvas, (ksize, ksize), 0)

    # Convert bounding boxes to YOLO normalized coordinates (cls, x_c, y_c, w, h)
    yolo_entries = []
    for cls_id, (bx1, by1, bx2, by2) in labels:
        bw = bx2 - bx1
        bh = by2 - by1
        if bw <= 4 or bh <= 4:
            continue
        xc = (bx1 + bx2) / 2.0 / img_w
        yc = (by1 + by2) / 2.0 / img_h
        nw = bw / float(img_w)
        nh = bh / float(img_h)
        yolo_entries.append(f"{cls_id} {xc:.6f} {yc:.6f} {nw:.6f} {nh:.6f}")

    return canvas, yolo_entries


# ==============================================================================
# 3. Dataset Generation Orchestrator
# ==============================================================================

def generate_full_dataset(train_count=320, val_count=64):
    print(f"[DATASET] Initializing dataset directory at: {DATASET_DIR}")
    for split in ["train", "val"]:
        os.makedirs(os.path.join(DATASET_DIR, "images", split), exist_ok=True)
        os.makedirs(os.path.join(DATASET_DIR, "labels", split), exist_ok=True)

    scenes = ["traffic_light", "slalom_cones", "parking", "mixed"]

    for split, count in [("train", train_count), ("val", val_count)]:
        print(f"[DATASET] Generating {count} synthetic samples for '{split}'...")
        for idx in range(count):
            st = scenes[idx % len(scenes)]
            img, annots = create_synthetic_frame(st)
            base_name = f"sample_{idx:04d}"
            img_path = os.path.join(DATASET_DIR, "images", split, f"{base_name}.jpg")
            lbl_path = os.path.join(DATASET_DIR, "labels", split, f"{base_name}.txt")

            cv2.imwrite(img_path, img)
            with open(lbl_path, "w") as f:
                f.write("\n".join(annots) + "\n")

    # Generate data.yaml
    yaml_content = f"""# RoboWorld 2026 Mission Vision Dataset
path: {DATASET_DIR}
train: images/train
val: images/val

names:
  0: traffic_light
  1: traffic_cone
  2: parking_sign
"""
    yaml_path = os.path.join(DATASET_DIR, "data.yaml")
    with open(yaml_path, "w") as f:
        f.write(yaml_content)
    print(f"[DATASET] Successfully generated {yaml_path}")
    return yaml_path


# ==============================================================================
# 4. YOLOv8n Training & ONNX Export
# ==============================================================================

def run_training_and_export(yaml_path, epochs=15):
    from ultralytics import YOLO

    base_pt = os.path.join(MODELS_DIR, "yolov8n.pt")
    if not os.path.isfile(base_pt):
        print(f"[TRAIN] Pretrained {base_pt} not found. Downloading via Ultralytics...")
        base_pt = "yolov8n.pt"

    print(f"\n[TRAIN] Loading pretrained base model: {base_pt}")
    model = YOLO(base_pt)

    runs_dir = os.path.join(WORKSPACE_DIR, "runs")
    print(f"[TRAIN] Starting fine-tuning ({epochs} epochs, imgsz=640, device=cpu)...")
    results = model.train(
        data=yaml_path,
        epochs=epochs,
        imgsz=640,
        batch=16,
        device="cpu",
        project=runs_dir,
        name="mission_v8n",
        exist_ok=True,
        verbose=True,
        workers=4
    )

    best_pt = os.path.join(runs_dir, "mission_v8n", "weights", "best.pt")
    if not os.path.isfile(best_pt):
        best_pt = os.path.join(runs_dir, "mission_v8n", "weights", "last.pt")

    print(f"\n[EXPORT] Best weights saved at: {best_pt}")
    export_model = YOLO(best_pt)

    print("[EXPORT] Exporting fine-tuned model to ONNX format (opset=12)...")
    exported_onnx = export_model.export(format="onnx", opset=12)

    # Destination in mission_vision package
    target_onnx = os.path.join(MODELS_DIR, "yolo_mission.onnx")
    shutil.copy(exported_onnx, target_onnx)
    print(f"[EXPORT] Installed target model: {target_onnx} ({os.path.getsize(target_onnx) / (1024*1024):.1f} MB)")

    # Verify loading via OpenCV DNN
    net = cv2.dnn.readNetFromONNX(target_onnx)
    print(f"[VERIFY] OpenCV DNN loaded fine-tuned model successfully! Layers: {len(net.getLayerNames())}")


if __name__ == "__main__":
    yaml_file = generate_full_dataset(train_count=240, val_count=48)
    run_training_and_export(yaml_file, epochs=10)
