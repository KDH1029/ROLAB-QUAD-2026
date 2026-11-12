#!/usr/bin/env python3
"""카메라 콘 감지 핵심 계산 — 주황색 화소 + 뎁스 + 바닥 위 높이. ROS 를 모르는 순수 함수(노드·녹화 시험이 같이 쓴다).
경기장 장애물은 주황색 콘뿐(11-05 사용자). 색으로 거르므로 벽 밑동·경사면·사람 다리를 콘으로 안 잡는다.
뎁스(정렬 안 된 원본) 점 → ① 몸체 좌표(T_BODY_CAM, 11-04 보정) ② 프레임마다 바닥 평면 맞춤 → 바닥 위 높이
③ 같은 점을 컬러 화면에 투영(depth_to_color) → 그 화소가 주황색이고 바닥 위 2~40 cm 면 콘 점.
★ Python 3.8 호환(내부 PC).
"""
import numpy as np
import cv2

# 뎁스 광학 → FAST-LIO 몸체(IMU) — terrain_veto 와 같은 값(11-04 calib_depth.py)
T_BODY_CAM = np.array([[0.00198, -0.55953, 0.82881, 0.46644],
                       [-0.9993, -0.03209, -0.01927, 0.00958],
                       [0.03738, -0.82819, -0.5592, 0.11276]])
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
