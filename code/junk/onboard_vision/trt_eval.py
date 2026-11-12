#!/usr/bin/env python3
"""엔진으로 사진 폴더를 돌려 검출 결과 + 속도를 JSON 으로 남긴다(내부 PC). NUC 의 compare_eval.py 가 채점한다.
사용: python3 trt_eval.py <엔진> <출력.json> <사진폴더> [<사진폴더> ...]
★ Python 3.8 호환."""
import glob
import json
import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import tl_yolo_common as C  # noqa: E402
from trt_engine import TrtEngine  # noqa: E402

eng_path, out_path, dirs = sys.argv[1], sys.argv[2], sys.argv[3:]
eng = TrtEngine(eng_path)
H, W = eng.input_shape[2], eng.input_shape[3]
fs = sorted(f for d in dirs for f in glob.glob(os.path.join(d, '*.jpg')) + glob.glob(os.path.join(d, '*.png')))
res, tp, ti, td = {}, [], [], []
dummy = np.zeros(eng.input_shape, np.float32)
for _ in range(10):
    eng.infer(dummy)   # 예열 (첫 몇 번은 느리다)
for f in fs:
    bgr = cv2.imread(f)
    t0 = time.perf_counter(); blob, r, left, top = C.letterbox(bgr, (H, W))
    t1 = time.perf_counter(); out = eng.infer(blob)
    t2 = time.perf_counter(); dets = C.decode(out, r, left, top, bgr.shape[:2])
    t3 = time.perf_counter()
    tp.append(t1 - t0); ti.append(t2 - t1); td.append(t3 - t2)
    res[os.path.basename(f)] = [[c, round(s, 4), round(x1, 1), round(y1, 1), round(x2, 1), round(y2, 1)] for c, s, x1, y1, x2, y2 in dets]
ms = lambda v: {'중앙': round(1000 * float(np.median(v)), 2), 'p95': round(1000 * float(np.percentile(v, 95)), 2)}
meta = {'engine': os.path.basename(eng_path), '사진': len(fs), '전처리_ms': ms(tp), '추론_ms': ms(ti), '후처리_ms': ms(td),
        '합계_중앙_ms': round(1000 * float(np.median(np.array(tp) + np.array(ti) + np.array(td))), 2)}
with open(out_path, 'w') as f:
    json.dump({'meta': meta, 'dets': res}, f)
print(json.dumps(meta, ensure_ascii=False))
eng.close()
