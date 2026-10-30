#!/usr/bin/env python3
import os
import sys
import cv2

WORKSPACE = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.append(os.path.join(WORKSPACE, "src/missions/mission_vision/scripts"))

from yolo_detector_node import OpenCvDnnEngine

def main():
    model_path = os.path.join(WORKSPACE, "src/missions/mission_vision/models/yolo_mission.onnx")
    engine = OpenCvDnnEngine(model_path, 640, 640, "cpu")

    classes = ["traffic_light", "traffic_cone", "parking_sign"]
    val_dir = os.path.join(WORKSPACE, "dataset/images/val")
    sample_files = sorted(os.listdir(val_dir))[:3]

    print("==================================================")
    print("YOLOv8-Nano Mission ONNX Inference Verification")
    print("==================================================")
    for fname in sample_files:
        img_path = os.path.join(val_dir, fname)
        img = cv2.imread(img_path)
        dets = engine.predict(img, conf_thresh=0.40)
        print(f"\n[IMAGE] {fname} ({img.shape[1]}x{img.shape[0]}):")
        for d in dets:
            c = d["cls"]
            name = classes[c] if c < len(classes) else str(c)
            conf = d["conf"] * 100.0
            print(f"  -> Detected: {name:15s} | Confidence: {conf:5.1f}% | BBox: {d['bbox']}")

if __name__ == "__main__":
    main()
