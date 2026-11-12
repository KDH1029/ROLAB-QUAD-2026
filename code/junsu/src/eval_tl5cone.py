#!/usr/bin/env python3
"""tl5cone_640 체크포인트(9/80 epoch, 중단된 학습)를 val셋에서 클래스별로 평가.
기존 test_yolo_on_val.py와 같은 방식이지만 6클래스(cone 포함) 데이터셋 기준."""
import glob
import os
from ultralytics import YOLO

MODEL_PATH = '/home/ilovehan9283/ws/datasets/tl5/runs/tl5cone_640/weights/best.pt'
img_dir = '/home/ilovehan9283/ws/datasets/tl5/ds_tl5_cone/images/val'
lbl_dir = '/home/ilovehan9283/ws/datasets/tl5/ds_tl5_cone/labels/val'

model = YOLO(MODEL_PATH)
print(f"모델 클래스: {model.names}")

images = sorted(glob.glob(os.path.join(img_dir, '*.jpg')) + glob.glob(os.path.join(img_dir, '*.png')))
print(f"val 이미지 수: {len(images)}")

# 클래스별 GT/예측 집계 (이미지 단위, multi-label 가능)
class_names = model.names
n_classes = len(class_names)
gt_counts = {i: 0 for i in range(n_classes)}
tp_counts = {i: 0 for i in range(n_classes)}  # 이미지에 해당 클래스가 GT로 있고 예측에도 있으면 TP
fp_images = {i: 0 for i in range(n_classes)}  # GT엔 없는데 예측에만 있으면 FP(이미지 단위)

for img_path in images:
    base = os.path.splitext(os.path.basename(img_path))[0]
    lbl_path = os.path.join(lbl_dir, base + '.txt')
    gt_classes = set()
    if os.path.exists(lbl_path):
        with open(lbl_path) as f:
            for line in f:
                parts = line.strip().split()
                if parts:
                    gt_classes.add(int(parts[0]))

    results = model.predict(img_path, verbose=False, conf=0.25)
    pred_classes = set()
    for r in results:
        for c in r.boxes.cls.tolist():
            pred_classes.add(int(c))

    for c in range(n_classes):
        if c in gt_classes:
            gt_counts[c] += 1
            if c in pred_classes:
                tp_counts[c] += 1
        elif c in pred_classes:
            fp_images[c] += 1

print("\n=== 클래스별 이미지 단위 재현율/오탐 ===")
for c in range(n_classes):
    recall = 100 * tp_counts[c] / max(gt_counts[c], 1)
    print(f"  {c}={class_names[c]:12s} GT={gt_counts[c]:4d}  재현율={recall:5.1f}%  "
          f"(GT없는데 예측된 이미지 수={fp_images[c]})")
