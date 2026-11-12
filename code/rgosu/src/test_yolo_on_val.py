#!/usr/bin/env python3
"""팀원의 1-epoch 학습 YOLO 모델(best.pt)을 val셋(103장)에 돌려서 HSV 방식과
같은 기준(빨강/초록 재현율, 오탐)으로 비교한다."""
import glob
import os
from ultralytics import YOLO

MODEL_PATH = '/home/ilovehan9283/ws/datasets/tl5/runs/tl5_640_aug/weights/best.pt'
img_dir = '/home/ilovehan9283/ws/datasets/tl5/ds_aug/images/val'
lbl_dir = '/home/ilovehan9283/ws/datasets/tl5/ds_aug/labels/val'

model = YOLO(MODEL_PATH)
print(f"모델 클래스: {model.names}")


def ground_truth(label_path):
    if not os.path.exists(label_path):
        return 'OTHER'
    classes = set()
    with open(label_path) as f:
        for line in f:
            parts = line.strip().split()
            if parts:
                classes.add(int(parts[0]))
    has_red, has_green = 0 in classes, 1 in classes
    if has_red and not has_green:
        return 'RED'
    if has_green and not has_red:
        return 'GREEN'
    if has_red and has_green:
        return 'BOTH'
    return 'OTHER'


images = sorted(glob.glob(os.path.join(img_dir, '*.jpg')) + glob.glob(os.path.join(img_dir, '*.png')))
print(f"val 이미지 수: {len(images)}")

confusion = {}
rows = []
for img_path in images:
    base = os.path.splitext(os.path.basename(img_path))[0]
    lbl_path = os.path.join(lbl_dir, base + '.txt')
    gt = ground_truth(lbl_path)

    results = model.predict(img_path, verbose=False, conf=0.25)
    detected_classes = set()
    for r in results:
        for c in r.boxes.cls.tolist():
            detected_classes.add(int(c))

    has_red_pred = 0 in detected_classes
    has_green_pred = 1 in detected_classes
    if has_red_pred and not has_green_pred:
        pred = 'RED'
    elif has_green_pred and not has_red_pred:
        pred = 'GREEN'
    elif has_red_pred and has_green_pred:
        pred = 'BOTH'
    else:
        pred = 'UNKNOWN'

    key = (gt, pred)
    confusion[key] = confusion.get(key, 0) + 1
    rows.append((base, gt, pred))

print("\n=== 혼동행렬 (정답, 예측) -> 개수 ===")
for k in sorted(confusion.keys()):
    print(f"  GT={k[0]:7s} 예측={k[1]:7s} : {confusion[k]}")

n_red = sum(v for k, v in confusion.items() if k[0] == 'RED')
n_red_correct = confusion.get(('RED', 'RED'), 0) + confusion.get(('RED', 'BOTH'), 0)
n_green = sum(v for k, v in confusion.items() if k[0] == 'GREEN')
n_green_correct = confusion.get(('GREEN', 'GREEN'), 0) + confusion.get(('GREEN', 'BOTH'), 0)
n_other = sum(v for k, v in confusion.items() if k[0] == 'OTHER')
n_other_correct = confusion.get(('OTHER', 'UNKNOWN'), 0)

print(f"\n빨강 재현율: {n_red_correct}/{n_red} = {100*n_red_correct/max(n_red,1):.1f}%")
print(f"초록 재현율: {n_green_correct}/{n_green} = {100*n_green_correct/max(n_green,1):.1f}%")
print(f"기타(신호등 없음) 중 UNKNOWN 정답률: {n_other_correct}/{n_other} = {100*n_other_correct/max(n_other,1):.1f}%")
