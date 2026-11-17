#!/usr/bin/env python3
"""NUC 채점: 내부 PC 엔진(FP16·INT8) 결과를 원본 모델(.pt)·정답 라벨과 비교한다.
사용: ~/venv/go2/bin/python3 compare_eval.py          (results/eval_*.json 을 읽음)
보는 것
  ① 원본과 같은가 — 원본 상자 중 엔진도 찾은 비율(같은 클래스, IoU≥0.5), 신호 판정(RED/GREEN/UNKNOWN) 일치
  ② 정답 대비 — evalset/val169(라벨 있음)에서 빨강·초록·콘 재현율/정밀도 (확신 ≥0.25, IoU≥0.5)
  ③ 속도 — 내부 PC 에서 잰 전처리·추론·후처리 ms
★ 가장 중요한 숫자: 초록불 정밀도(초록 오판 = 빨간불에 출발)."""
import glob, json, os, sys
import numpy as np
os.environ.setdefault('YOLO_AUTOINSTALL', 'false')
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import tl_yolo_common as C
import cv2

EV = os.path.join(HERE, 'evalset'); RES = os.path.join(HERE, 'results')
LBL = os.path.expanduser('~/ws/datasets/tl5/ds_tl5_cone/labels/val')
files = sorted(glob.glob(os.path.join(EV, '*', '*.jpg')) + glob.glob(os.path.join(EV, '*', '*.png')))
ref_path = os.path.join(RES, 'ref_pt.json' if C.MODEL == 'tl5cone' else 'ref_%s_onnx.json' % C.MODEL); os.makedirs(RES, exist_ok=True)
if os.path.exists(ref_path):
    REF = json.load(open(ref_path))
elif C.MODEL != 'tl5cone':
    # ★ 11-05 새 모델(tl_cone_1104)은 .pt 가 없다 → 원본(OpenVINO FP16)과 상자 100% 같은 ONNX 를 NUC 에서 FP32 로 돌린 것이 기준
    net = cv2.dnn.readNetFromONNX(C.onnx_path(HERE)); REF = {}
    for f in files:
        bgr = cv2.imread(f); blob, r, l, t = C.letterbox(bgr); net.setInput(blob)
        REF[os.path.basename(f)] = [list(d) for d in C.decode(net.forward(), r, l, t, bgr.shape[:2])]
    json.dump(REF, open(ref_path, 'w'))
else:
    from ultralytics import YOLO
    m = YOLO(os.path.join(HERE, 'tl5cone_640.pt')); REF = {}
    for k in range(0, len(files), 8):
        for f, r in zip(files[k:k + 8], m.predict(files[k:k + 8], imgsz=640, conf=0.25, iou=0.7, verbose=False, device='cpu')):
            REF[os.path.basename(f)] = [[int(c), float(s), *map(float, b)] for c, s, b in zip(r.boxes.cls.numpy(), r.boxes.conf.numpy(), r.boxes.xyxy.numpy())]
    json.dump(REF, open(ref_path, 'w'))

def iou(a, b):
    ix = max(0, min(a[2], b[2]) - max(a[0], b[0])); iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - ix * iy
    return ix * iy / u if u > 0 else 0

def match(gt, pr, thr=0.5):
    """같은 클래스끼리 IoU 큰 순으로 1:1 짝. 짝 지은 수."""
    used, n = set(), 0
    for g in gt:
        best, bi = thr, -1
        for j, p in enumerate(pr):
            if j in used or p[0] != g[0]: continue
            v = iou(g[-4:], p[-4:])
            if v >= best: best, bi = v, j
        if bi >= 0: used.add(bi); n += 1
    return n

GT = {}
for f in files:
    if os.sep + 'val169' + os.sep not in f: continue
    st = os.path.splitext(os.path.basename(f))[0]; lp = os.path.join(LBL, st + '.txt')
    if not os.path.exists(lp): continue
    h, w = cv2.imread(f).shape[:2]; g = []
    for ln in open(lp):
        v = ln.split()
        if len(v) < 5: continue
        c, cx, cy, bw, bh = int(v[0]), *map(float, v[1:5])
        if C.MODEL != 'tl5cone':   # 라벨은 6클래스(0 빨강 1 초록 2 보행자 3 temi 4 unknown 5 콘) → 4클래스로
            c = {0: 0, 1: 1, 4: 2, 5: 3}.get(c, -1)
            if c < 0: continue
        g.append([c, (cx - bw / 2) * w, (cy - bh / 2) * h, (cx + bw / 2) * w, (cy + bh / 2) * h])
    GT[os.path.basename(f)] = g

def score(D, name, meta=None):
    nref = nm = 0; same = 0
    for fn, ref in REF.items():
        pr = D.get(fn, [])
        nref += len(ref); nm += match([[r[0]] + r[2:] for r in ref], [[p[0]] + p[2:] for p in pr])
        same += C.judge([tuple(r) for r in ref])[0] == C.judge([tuple(p) for p in pr])[0]
    row = {'모델': name, '원본 상자 재현': '%.1f%%' % (100 * nm / max(1, nref)), '신호 판정 일치': '%d/%d' % (same, len(REF))}
    for c, nmc in ((0, '빨강'), (1, '초록'), (C.CONE, '콘')):
        tp = ng = npd = 0
        for fn, g in GT.items():
            gc = [x for x in g if x[0] == c]; pc = [[p[0]] + p[2:] for p in D.get(fn, []) if p[0] == c]
            tp += match(gc, pc); ng += len(gc); npd += len(pc)
        row[nmc + ' 재현/정밀'] = '%.1f / %.1f' % (100 * tp / max(1, ng), 100 * tp / max(1, npd))
    if meta: row['속도(내부PC) 추론·합계 ms'] = '%s · %s' % (meta['추론_ms']['중앙'], meta['합계_중앙_ms'])
    return row

rows = [score(REF, '원본 (NUC, 기준 — %s)' % C.MODEL)]
for p in sorted(glob.glob(os.path.join(RES, 'eval_%s_*.json' % C.MODEL))):
    d = json.load(open(p)); rows.append(score(d['dets'], d['meta']['engine'], d['meta']))
keys = list(rows[0].keys()) + [k for k in rows[-1] if k not in rows[0]]
print('평가 사진 %d장 (라벨 있는 것 %d장)\n' % (len(REF), len(GT)))
for r in rows:
    print('  ' + '  |  '.join('%s %s' % (k, r.get(k, '-')) for k in keys))
