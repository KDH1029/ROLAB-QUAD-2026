#!/usr/bin/env python3
"""카메라 콘 감지 핵심 계산 — 주황색 화소 + 뎁스 + 바닥 위 높이. ROS 를 모르는 순수 함수(노드·녹화 시험이 같이 쓴다).
경기장 장애물은 주황색 콘뿐(11-05 사용자). 색으로 거르므로 벽 밑동·경사면·사람 다리를 콘으로 안 잡는다.
뎁스(정렬 안 된 원본) 점 → ① 몸체 좌표(T_BODY_CAM, 11-04 보정) ② 프레임마다 바닥 평면 맞춤 → 바닥 위 높이
③ 같은 점을 컬러 화면에 투영(depth_to_color) → 그 화소가 주황색이고 바닥 위 2~40 cm 면 콘 점.
★ Python 3.8 호환(내부 PC).
"""
import numpy as np
import cv2

import os
# 뎁스 광학 → FAST-LIO 몸체(IMU) — terrain_veto 와 같은 값(11-04 calib_depth.py, 아래 34°)
T_BODY_CAM_1104 = np.array([[0.00198, -0.55953, 0.82881, 0.46644],
                            [-0.9993, -0.03209, -0.01927, 0.00958],
                            [0.03738, -0.82819, -0.5592, 0.11276]])
# ★ 11-06: 카메라가 거의 수평으로 다시 달려 있다(11-05 16:50 녹화·11-06 10:28 실측 바닥각 2.3°). 11-04 값이면 뎁스↔XT16 54 cm 어긋남.
#   16:50 출발 정지 18초 뎁스→XT16 ICP(중앙 2.1 cm) — junsu_v2/debug/yolo_cone_1106/REPORT.md
T_BODY_CAM_1650 = np.array([[0.00781, 0.07030, 0.99750, 0.45215],
                            [-0.99997, -0.00027, 0.00785, 0.01144],
                            [0.00082, -0.99753, 0.07030, 0.03636]])
# GO2_CAM_FIX=1650 일 때만 새 값(junsu_v2 관리자가 넘김). 없으면 예전 그대로 — juninsu·bak 은 안 바뀜
CAM_FIX = os.environ.get('GO2_CAM_FIX', '')
T_BODY_CAM = T_BODY_CAM_1650 if CAM_FIX == '1650' else T_BODY_CAM_1104
DEPTH_TO_COLOR_T = (-0.0147, -0.0009, -0.0008)
# 주황(OpenCV HSV, H 0~180) — 콘 몸통. 흰 띠는 색으로 안 잡지만 같은 콘의 주황 부분이 잡힌다
H_LO, H_HI, S_MIN, V_MIN = 3, 22, 110, 70
MIN_H, MAX_H, RMIN, RMAX = 0.02, 0.40, 0.25, 3.0


def depth_points(depth_u16, Kd, step=4, scale=0.001, zmin=0.2, zmax=4.0):
    """뎁스 → 뎁스 광학 좌표 점 (N,3) m."""
    D = depth_u16[::step, ::step].astype(np.float32) * scale
    v, u = np.nonzero((D > zmin) & (D < zmax))
    z = D[v, u]; u = u * step; v = v * step
    return np.stack([(u - Kd[0, 2]) / Kd[0, 0] * z, (v - Kd[1, 2]) / Kd[1, 1] * z, z], 1)


def floor_plane(Pb, h_guess=-0.31):
    """몸체 좌표 점에서 바닥 평면 z = a x + b y + c (몸 앞 0.3~2.5 m, 바닥 예상 높이 ±12 cm 에서 시작, 두 번 다듬음). 실패면 None."""
    r = np.hypot(Pb[:, 0], Pb[:, 1])
    g = (r > 0.3) & (r < 2.5) & (np.abs(Pb[:, 2] - h_guess) < 0.12)
    for tol in (0.05, 0.025):
        if g.sum() < 80:
            return None
        A = np.c_[Pb[g, 0], Pb[g, 1], np.ones(g.sum())]
        coef = np.linalg.lstsq(A, Pb[g, 2], rcond=None)[0]
        res = Pb[:, 2] - (Pb[:, 0] * coef[0] + Pb[:, 1] * coef[1] + coef[2])
        g = (r > 0.3) & (r < 2.5) & (np.abs(res) < tol)
    return coef


def orange_mask(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    return (hsv[:, :, 0] >= H_LO) & (hsv[:, :, 0] <= H_HI) & (hsv[:, :, 1] >= S_MIN) & (hsv[:, :, 2] >= V_MIN)


def cone_points(depth_u16, bgr, Kd, Kc, T_bc=T_BODY_CAM, T_dc=DEPTH_TO_COLOR_T, step=4, voxel=0.04):
    """콘 점(몸체 좌표, voxel 로 줄임)과 진단값. 바닥을 못 맞추면 빈 배열."""
    Pd = depth_points(depth_u16, Kd, step)
    info = {'n_depth': int(len(Pd)), 'floor': None, 'n_orange': 0, 'n_cone': 0}
    if len(Pd) < 100:
        return np.zeros((0, 3), np.float32), info
    Pb = Pd @ T_bc[:, :3].T + T_bc[:, 3]
    coef = floor_plane(Pb)
    if coef is None:
        return np.zeros((0, 3), np.float32), info
    info['floor'] = [round(float(c), 4) for c in coef]
    h = Pb[:, 2] - (Pb[:, 0] * coef[0] + Pb[:, 1] * coef[1] + coef[2])
    r = np.hypot(Pb[:, 0], Pb[:, 1])
    cand = (h >= MIN_H) & (h <= MAX_H) & (r >= RMIN) & (r <= RMAX)
    if not cand.any():
        return np.zeros((0, 3), np.float32), info
    Pc = Pd[cand] + np.asarray(T_dc, np.float32)           # 뎁스 → 컬러 광학(회전은 거의 단위)
    uc = (Kc[0, 0] * Pc[:, 0] / Pc[:, 2] + Kc[0, 2]).astype(int); vc = (Kc[1, 1] * Pc[:, 1] / Pc[:, 2] + Kc[1, 2]).astype(int)
    Hh, Ww = bgr.shape[:2]; inimg = (uc >= 0) & (uc < Ww) & (vc >= 0) & (vc < Hh)
    om = orange_mask(bgr); org = np.zeros(len(Pc), bool); org[inimg] = om[vc[inimg], uc[inimg]]
    info['n_orange'] = int(om.sum())
    C = Pb[cand][org]
    if len(C):
        k = np.unique(np.floor(C / voxel).astype(np.int64), axis=0)
        C = (k + 0.5) * voxel
    info['n_cone'] = int(len(C))
    return C.astype(np.float32), info


def merge_stacked(boxes, depth_fn=None, max_dz=0.25):
    """큰 콘(가운데 회색 띠)이 위·아래 두 상자로 갈라진 것을 하나로 — [(cls, conf, x1, y1, x2, y2)] → 같은 모양 목록.
    같은 콘: 가로로 절반 넘게 겹침 · 위 상자 아래끝~아래 상자 위끝 틈 ≤ 위 상자 높이 · 위가 좁음(≤1.6배) · (뎁스 있으면) 거리 차 ≤ max_dz.
    합친 상자의 conf = 조각 중 최대. 16:50 녹화: 갈라진 쌍 174개(둘 다 0.45↑ 140) → 바닥 위 두 점이 30 cm 떨어져 콘이 둘로 보였다."""
    n = len(boxes)
    if n < 2:
        return list(boxes)
    par = list(range(n))

    def f(i):
        while par[i] != i:
            par[i] = par[par[i]]; i = par[i]
        return i
    z = [depth_fn(b[2], b[3], b[4], b[5]) if depth_fn is not None else float('nan') for b in boxes]
    for i in range(n):
        for j in range(n):
            if i == j or f(i) == f(j):
                continue
            a, b = boxes[i], boxes[j]
            if a[3] >= b[3]:
                continue                                   # a 가 위
            wa, wb, ha = a[4] - a[2], b[4] - b[2], a[5] - a[3]
            if min(a[4], b[4]) - max(a[2], b[2]) < 0.5 * min(wa, wb):
                continue
            gap = b[3] - a[5]
            if gap > 1.0 * ha + 6 or gap < -0.6 * ha or wa > 1.6 * wb:
                continue
            if np.isfinite(z[i]) and np.isfinite(z[j]) and abs(z[i] - z[j]) > max_dz:
                continue
            par[f(i)] = f(j)
    g = {}
    for i in range(n):
        g.setdefault(f(i), []).append(boxes[i])
    out = []
    for grp in g.values():
        B = np.array([b[2:6] for b in grp], float)
        out.append((grp[0][0], max(b[1] for b in grp), float(B[:, 0].min()), float(B[:, 1].min()), float(B[:, 2].max()), float(B[:, 3].max())))
    return out
