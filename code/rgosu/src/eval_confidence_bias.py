#!/usr/bin/env python3
"""tl5cone_640이 '안 좋은 이미지'에서 그냥 포기(저신뢰도)하는 편향이 있는지 확인.
GT에 tl_red/tl_green이 있는 이미지에 대해 임계값(0.25) 통과 여부가 아니라
'모델이 그 클래스에 부여한 최고 confidence'를 그대로 뽑아서 분포를 본다.
- 임계값 바로 아래(0.15~0.25): 애매한 경계 case (threshold 튜닝 문제)
- 0에 가까움(<0.05): 모델이 그 클래스를 전혀 못 찾음 (진짜 '포기'에 가까움)
밝기(명암)와의 상관도 같이 본다 -- 어두운/저품질 이미지일수록 완전히 놓치는지."""
import glob
import os
import cv2
import numpy as np
from ultralytics import YOLO

MODEL_PATH = '/home/ilovehan9283/ws/datasets/tl5/runs/tl5cone_640/weights/best.pt'
img_dir = '/home/ilovehan9283/ws/datasets/tl5/ds_tl5_cone/images/val'
lbl_dir = '/home/ilovehan9283/ws/datasets/tl5/ds_tl5_cone/labels/val'

model = YOLO(MODEL_PATH)
images = sorted(glob.glob(os.path.join(img_dir, '*.jpg')) + glob.glob(os.path.join(img_dir, '*.png')))

rows = []  # (base, gt_class_name, max_conf_for_that_class, mean_brightness)
for img_path in images:
    base = os.path.splitext(os.path.basename(img_path))[0]
    lbl_path = os.path.join(lbl_dir, base + '.txt')
    if not os.path.exists(lbl_path):
        continue
    gt_classes = set()
    with open(lbl_path) as f:
        for line in f:
            parts = line.strip().split()
            if parts:
                gt_classes.add(int(parts[0]))
    if 0 not in gt_classes and 1 not in gt_classes:
        continue

    img = cv2.imread(img_path)
    brightness = float(np.mean(img)) if img is not None else -1.0

    # conf=0.0 으로 다 뽑아서 임계값 걸기 전 raw confidence를 본다
    results = model.predict(img_path, verbose=False, conf=0.001)
    max_conf = {0: 0.0, 1: 0.0}
    for r in results:
        for cls_id, conf in zip(r.boxes.cls.tolist(), r.boxes.conf.tolist()):
            cls_id = int(cls_id)
            if cls_id in max_conf:
                max_conf[cls_id] = max(max_conf[cls_id], conf)

    for c in (0, 1):
        if c in gt_classes:
            rows.append((base, c, max_conf[c], brightness))

names = {0: 'tl_red', 1: 'tl_green'}
for c in (0, 1):
    confs = [r[2] for r in rows if r[1] == c]
    brights = [r[3] for r in rows if r[1] == c]
    confs = np.array(confs)
    brights = np.array(brights)
    print(f"\n=== GT={names[c]} (n={len(confs)}) ===")
    print(f"  confidence 분포: min={confs.min():.3f} p25={np.percentile(confs,25):.3f} "
          f"median={np.median(confs):.3f} p75={np.percentile(confs,75):.3f} max={confs.max():.3f}")
    near_zero = (confs < 0.05).sum()
    borderline = ((confs >= 0.05) & (confs < 0.25)).sum()
    confident = (confs >= 0.25).sum()
    print(f"  완전히 못 찾음(<0.05): {near_zero}/{len(confs)}  |  "
          f"애매(0.05~0.25): {borderline}/{len(confs)}  |  "
          f"자신있게 맞춤(>=0.25): {confident}/{len(confs)}")
    if near_zero > 0:
        miss_bright = brights[confs < 0.05]
        hit_bright = brights[confs >= 0.25]
        print(f"  완전히 못 찾은 이미지 평균 밝기: {miss_bright.mean():.1f}  "
              f"(참고: 잘 맞춘 이미지 평균 밝기: {hit_bright.mean() if len(hit_bright)>0 else float('nan'):.1f})")
        print(f"  완전히 못 찾은 파일들: {[r[0] for r in rows if r[1]==c and r[2] < 0.05][:10]}")
