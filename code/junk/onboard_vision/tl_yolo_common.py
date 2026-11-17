#!/usr/bin/env python3
"""YOLOv8 전처리·후처리 + 신호 판정 — torch/ultralytics 없이 numpy+OpenCV 만 쓴다.

왜 따로 있나: 내부 PC(Jetson, Foxy, Python 3.8)에는 ultralytics·torch 가 없다(설치도 무겁다).
TensorRT 엔진은 텐서만 주고받으므로, ultralytics 가 해 주던 일(레터박스 → 출력 해석 → NMS)을
여기서 똑같이 한다. NUC 에서 OpenCV DNN(ONNX)으로 돌려 ultralytics 결과와 일치하는지 먼저
검증한다(check_common_vs_ultralytics.py) — 그래야 로봇 위에서 틀리면 엔진 문제인지 바로 안다.

★ Python 3.8 호환으로 쓸 것(내부 PC): 타입 힌트 `list[int]`·`X | None`·match 금지.
"""
import cv2
import numpy as np

import os

# ── 모델 고르기 (2026-11-05) ── 환경변수 TL_MODEL. 파일은 <이름>_384x640.onnx / _fp16.engine / _int8.engine
#   tl_cone_1104 (기본) 4클래스 — 팀원이 11-04 경기장 사진 1500장 더해 학습(OpenVINO 로 받음 → models_1104/ 에서 가중치를 꺼내 ONNX 로 되살림, 원본과 상자 100% 일치)
#   tl5cone            6클래스 — 11-02 학습(경기장 사진 없음). 예비
MODEL = os.environ.get('TL_MODEL', 'tl_cone_1104')
NAMES_BY_NC = {6: {0: 'tl_red', 1: 'tl_green', 2: 'pedestrian', 3: 'temi', 4: 'unknown', 5: 'cone'},
               4: {0: 'tl_red', 1: 'tl_green', 2: 'unknown', 3: 'cone'}}
NAMES = NAMES_BY_NC[4 if MODEL == 'tl_cone_1104' else 6]
RED, GREEN = 0, 1                       # 두 모델 같음
CONE = max(k for k, v in NAMES.items() if v == 'cone')   # 기본 모델 기준. 실제 출력으로는 cone_cls(out)
IMGSZ = (384, 640)   # (높이, 너비) — 848×480·1280×720 둘 다 640×384 에 딱 맞는다. 기존 성능(rect 입력)과 같은 조건, 계산 40%↓


def onnx_path(here):
    return os.path.join(here, MODEL + '_384x640.onnx')


def cone_cls(out):
    """모델 출력 (1, 4+클래스수, N) 에서 콘 번호 — 6클래스 5, 4클래스 3. 엔진을 바꿔 끼워도 틀리지 않게 출력 모양으로 정한다."""
    nc = int(np.shape(out)[-2]) - 4
    return max(k for k, v in NAMES_BY_NC[nc].items() if v == 'cone') if nc in NAMES_BY_NC else -1


def letterbox(bgr, size=IMGSZ):
    """ultralytics LetterBox(center=True, 회색 114) 와 같은 계산. size=(높이, 너비).
    돌려주는 것: (1,3,높이,너비) float32 RGB 0~1, 비율 r, 왼쪽·위 여백(px)."""
    h, w = bgr.shape[:2]
    H, W = size
    r = min(H / h, W / w)
    nw, nh = int(round(w * r)), int(round(h * r))
    dw, dh = (W - nw) / 2.0, (H - nh) / 2.0
    im = bgr if (w, h) == (nw, nh) else cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114))
    blob = np.ascontiguousarray(im[:, :, ::-1].transpose(2, 0, 1)[None], dtype=np.float32) / 255.0
    return blob, r, left, top


def _nms(boxes, scores, iou_thr):
    """한 클래스 안의 NMS (x1,y1,x2,y2). 남길 인덱스를 점수 순으로 돌려준다."""
    x1, y1, x2, y2 = boxes.T
    area = np.maximum(x2 - x1, 0) * np.maximum(y2 - y1, 0)
    order = scores.argsort()[::-1]
    keep = []
    while order.size:
        i = order[0]
        keep.append(i)
        if order.size == 1:
            break
        xx1 = np.maximum(x1[i], x1[order[1:]]); yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]]); yy2 = np.minimum(y2[i], y2[order[1:]])
        inter = np.maximum(xx2 - xx1, 0) * np.maximum(yy2 - yy1, 0)
        iou = inter / (area[i] + area[order[1:]] - inter + 1e-9)
        order = order[1:][iou <= iou_thr]
    return keep


def decode(out, r, left, top, orig_hw, conf_thr=0.25, iou_thr=0.7, max_det=300):
    """(1, 4+클래스수, 8400) → [(cls, conf, x1, y1, x2, y2)] 원본 이미지 좌표, 점수 내림차순.
    ultralytics 기본(conf 0.25, iou 0.7, 클래스별 NMS, max_det 300)과 같게 맞췄다."""
    p = np.asarray(out, dtype=np.float32).reshape(out.shape[-2], out.shape[-1]).T   # (8400, 10)
    sc = p[:, 4:]
    cls = sc.argmax(1); conf = sc[np.arange(len(sc)), cls]
    m = conf > conf_thr
    if not m.any():
        return []
    p, cls, conf = p[m], cls[m], conf[m]
    cx, cy, w, h = p[:, 0], p[:, 1], p[:, 2], p[:, 3]
    b = np.stack([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2], 1)
    dets = []
    for c in np.unique(cls):
        idx = np.where(cls == c)[0]
        for k in _nms(b[idx], conf[idx], iou_thr):
            dets.append((int(c), float(conf[idx][k]), b[idx][k]))
    dets.sort(key=lambda d: -d[1])
    dets = dets[:max_det]
    H, W = orig_hw
    res = []
    for c, s, bb in dets:
        x1 = float(np.clip((bb[0] - left) / r, 0, W)); y1 = float(np.clip((bb[1] - top) / r, 0, H))
        x2 = float(np.clip((bb[2] - left) / r, 0, W)); y2 = float(np.clip((bb[3] - top) / r, 0, H))
        res.append((c, s, x1, y1, x2, y2))
    return res


def judge(dets):
    """기존 traffic_light_detector_node.py 와 같은 규칙: 빨강·초록 중 가장 확실한 것.
    동점이면 빨강(안전 쪽). 아무것도 없으면 UNKNOWN. → ('RED'|'GREEN'|'UNKNOWN', 신뢰도)"""
    red = max([d[1] for d in dets if d[0] == RED] or [0.0])
    green = max([d[1] for d in dets if d[0] == GREEN] or [0.0])
    if red > 0 and red >= green:
        return 'RED', red
    if green > 0:
        return 'GREEN', green
    return 'UNKNOWN', 0.0


# ── 뎁스 거리 필터 (2026-11-05) ─────────────────────────────────────────────────────────────
# 왜: 경기장 뒤 전시장 조명(빨강·초록)을 신호등으로 잡는다. 11-04 녹화 뎁스로 잰 거리 —
#     진짜 신호등(정지선에서 5초 주기로 바뀐 것 372개) 1.5~4.9 m · 전시장 가짜 녹색 16.5~21 m.
#     → 6 m 넘는(또는 거리를 못 잰) 신호등 상자는 버린다. 2 m 로 자르면 진짜 신호등의 약 90% 가 잘린다.
# 뎁스(/camera/depth/image_rect_raw)는 컬러와 정렬 안 된 원본이라, 뎁스 내부파라미터 → 3D → depth_to_color → 컬러 투영.
DEPTH_MAX_M = 6.0
DEPTH_TO_COLOR_T = (-0.0147, -0.0009, -0.0008)   # 이 D435i 의 /camera/extrinsics/depth_to_color 이동(11-04 녹화에서 읽음, 회전은 거의 단위)


def project_depth(depth_u16, Kd, Kc, R=None, T=DEPTH_TO_COLOR_T, step=2, scale=0.001):
    """정렬 안 된 뎁스(16UC1, mm) → 컬러 화면 좌표 (u, v, z[m]) 배열. Kd·Kc: 3x3 내부파라미터. step 칸마다 하나(속도)."""
    D = depth_u16[::step, ::step].astype(np.float32) * scale
    v, u = np.nonzero(D > 0.1)
    z = D[v, u]
    u = u * step
    v = v * step
    x = (u - Kd[0, 2]) / Kd[0, 0] * z
    y = (v - Kd[1, 2]) / Kd[1, 1] * z
    P = np.stack([x, y, z], 1)
    if R is not None:
        P = P @ np.asarray(R, np.float32).T
    P = P + np.asarray(T, np.float32)
    zc = P[:, 2]
    ok = zc > 0.05
    P, zc = P[ok], zc[ok]
    return Kc[0, 0] * P[:, 0] / zc + Kc[0, 2], Kc[1, 1] * P[:, 1] / zc + Kc[1, 2], zc


def box_depth(proj, x1, y1, x2, y2, inner=0.6, min_pts=3):
    """상자 가운데 inner 비율 안에 떨어진 뎁스 점의 중앙값(m). 점이 min_pts 보다 적으면 nan(너무 멀거나 반사 없음)."""
    uc, vc, zc = proj
    mx, my = (1 - inner) / 2 * (x2 - x1), (1 - inner) / 2 * (y2 - y1)
    sel = (uc >= x1 + mx) & (uc <= x2 - mx) & (vc >= y1 + my) & (vc <= y2 - my)
    return float(np.median(zc[sel])) if int(sel.sum()) >= min_pts else float('nan')


def gate_by_depth(dets, proj, max_m=DEPTH_MAX_M):
    """신호등(RED·GREEN) 상자 중 거리 max_m 넘거나 못 잰 것을 뺀다(다른 클래스는 그대로).
    돌려주는 것: (남은 dets, [(cls, conf, 거리 m, 남겼나)] — 신호등 상자만)."""
    keep, info = [], []
    for d in dets:
        c, s, x1, y1, x2, y2 = d
        if c not in (RED, GREEN):
            keep.append(d)
            continue
        z = box_depth(proj, x1, y1, x2, y2)
        ok = bool(np.isfinite(z) and z <= max_m)
        info.append((c, s, z, ok))
        if ok:
            keep.append(d)
    return keep, info


def cone_depth(proj, x1, y1, x2, y2, pct=25, min_pts=4):
    """콘 상자 거리(m) — 상자 아래 60%·가운데 50%(콘 몸통) 안 뎁스 점의 가까운 쪽 pct 백분위.
    중앙값은 콘 뒤 벽까지 섞여 콘이 벽 쪽으로 밀렸다(11-05 17:56 재생: 벽 밖 0.3 m). 점이 모자라면 nan."""
    uc, vc, zc = proj
    w, h = x2 - x1, y2 - y1
    sel = (uc >= x1 + 0.25 * w) & (uc <= x2 - 0.25 * w) & (vc >= y1 + 0.4 * h) & (vc <= y2)
    return float(np.percentile(zc[sel], pct)) if int(sel.sum()) >= min_pts else float('nan')
