#!/usr/bin/env python3
"""NUC 검증: tl_yolo_common(레터박스·해석·NMS) + ONNX(OpenCV DNN) 결과가 ultralytics(.pt)와 같은가.
사용: ~/venv/go2/bin/python3 check_common_vs_ultralytics.py [사진 폴더 ...]
같다면 → 로봇 위 TensorRT 엔진 결과가 다를 때 원인은 엔진(양자화) 쪽이다."""
import glob, os, sys, json
import cv2, numpy as np
os.environ.setdefault('YOLO_AUTOINSTALL', 'false')
from ultralytics import YOLO
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import tl_yolo_common as C

dirs = sys.argv[1:] or [os.path.expanduser('~/ws/datasets/tl5/ds_tl5_cone/images/val')]
fs = sorted(f for d in dirs for f in glob.glob(os.path.join(d, '*.jpg')) + glob.glob(os.path.join(d, '*.png')))
net = cv2.dnn.readNetFromONNX(os.path.join(HERE, 'tl5cone_384x640.onnx'))
pt = YOLO(os.path.join(HERE, 'tl5cone_640.pt'))

def iou(a, b):
    ix = max(0, min(a[2], b[2]) - max(a[0], b[0])); iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - ix * iy
    return ix * iy / u if u > 0 else 0

n_ref = n_ours = n_match = 0; state_same = 0; dconf = []; worst = []
for k in range(0, len(fs), 8):
    chunk = fs[k:k + 8]
    # 기본(rect) 예측 = 848×480·1280×720 사진이면 640×384 입력 — 엔진(640×384 고정)과 같은 조건
    for f, r in zip(chunk, pt.predict(chunk, imgsz=640, conf=0.25, iou=0.7, verbose=False, device='cpu')):
        bgr = cv2.imread(f)
        blob, rr, left, top = C.letterbox(bgr)
        net.setInput(blob); out = net.forward()
        ours = C.decode(out, rr, left, top, bgr.shape[:2])
        ref = [(int(c), float(s), *map(float, b)) for c, s, b in zip(r.boxes.cls.numpy(), r.boxes.conf.numpy(), r.boxes.xyxy.numpy())]
        n_ref += len(ref); n_ours += len(ours)
        used = set()
        for d in ref:
            best, bi = 0, -1
            for j, o in enumerate(ours):
                if j in used or o[0] != d[0]: continue
                v = iou(d[2:], o[2:])
                if v > best: best, bi = v, j
            if best >= 0.9:
                n_match += 1; used.add(bi); dconf.append(abs(ours[bi][1] - d[1]))
        s1, s2 = C.judge(ref)[0], C.judge(ours)[0]
        state_same += s1 == s2
        if s1 != s2: worst.append((os.path.basename(f), s1, s2))
print(json.dumps({'사진': len(fs), 'ultralytics 박스': n_ref, '우리 박스': n_ours, '일치(IoU≥0.9 같은 클래스)': n_match,
                  '일치율': round(n_match / max(1, n_ref), 4), '신뢰도 차 최대': round(max(dconf or [0]), 4),
                  '신호 판정 같음': f'{state_same}/{len(fs)}', '판정 다른 사진': worst[:10]}, ensure_ascii=False, indent=1))
