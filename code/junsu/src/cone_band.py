#!/usr/bin/env python3
"""콘(낮은 장애물) 높이밴드 계산 — ROS 를 모르는 순수 함수. cone_raw_filter_node.py 가 쓰고, 녹화로 바로 시험할 수 있다.

좌표: 입력 점은 '몸체(body ≈ base_link)' 기준. /Odometry(FAST-LIO)의 자세는 몸체(IMU) 자세다.
  XT16(/velodyne_points)은 라이다 좌표라 먼저 lidar_to_body() 로 바꿔서 넣을 것(외부파라미터 = FAST-LIO 설정값).
  L1(/utlidar/cloud_base)은 이미 base_link 기준이다.
높이: 바닥 기준. 바닥은 '몸체 아래 floor_below 미터'(L1 으로 잼 — 11-04 17:56 녹화 0.415 m).
  ★ 예전 cone_raw_filter 는 라이다 원점 기준 높이를 바닥 기준인 줄 알고 써서, 실제로는 바닥 위 0.6~1.0 m 를 골랐다.
"""
import math

import numpy as np

R_LIDAR = np.array([[-0.0001707, -0.9999240, 0.0123287], [0.9999844, -0.0001019, 0.0055807], [-0.0055790, 0.0123295, 0.9999084]])
T_LIDAR = np.array([0.25, 0.0, 0.1588])


def lidar_to_body(pts, R=R_LIDAR, T=T_LIDAR):
    return np.asarray(pts, dtype=np.float64) @ np.asarray(R).T + np.asarray(T)


def rot_level(roll, pitch):
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    return np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])


def estimate_floor_below(pts_body, roll, pitch, default=0.415, rmin=0.3, rmax=1.0):
    """수평 맞춘 몸체 좌표에서 0.3~1.0 m 안 점의 높이 최빈값 = 바닥 → '몸체 아래 몇 m'. 점이 모자라면 default."""
    if len(pts_body) < 50:
        return default
    lv = np.asarray(pts_body, dtype=np.float64) @ rot_level(roll, pitch).T
    d = np.hypot(lv[:, 0], lv[:, 1]); z = lv[(d > rmin) & (d < rmax), 2]
    if len(z) < 50:
        return default
    h, e = np.histogram(z, bins=np.arange(-0.8, 0.0, 0.01))
    if h.max() < 20:
        return default
    return float(-(e[h.argmax()] + 0.005))


def band_world(pts_body, roll, pitch, yaw, t, floor_below, min_h=0.08, max_h=0.30, rmin=0.30, rmax=2.0,
               body_box=(-0.40, 0.40, 0.25)):
    """몸체 좌표 점 → 바닥 위 [min_h, max_h] 이고 몸에서 rmin~rmax 인 점만 세계 좌표로.
    body_box=(x최소, x최대, |y|최대): 로봇 몸·다리(L1 에 찍힘)를 뺀다."""
    P = np.asarray(pts_body, dtype=np.float64).reshape(-1, 3)
    if not len(P):
        return np.zeros((0, 3), np.float32)
    x0, x1, ym = body_box
    P = P[~((P[:, 0] > x0) & (P[:, 0] < x1) & (np.abs(P[:, 1]) < ym))]
    lv = P @ rot_level(roll, pitch).T
    r = np.hypot(lv[:, 0], lv[:, 1])
    h = lv[:, 2] + floor_below
    # ★ 바닥 평면 맞춤: 보행 중 몸 기울기 추정 오차(1~2°)가 1.5 m 에서 5 cm → 바닥이 밴드로 샌다(11-04 콘 통로 재생에서 확인).
    #   이 프레임의 바닥 근처 점(±6 cm)으로 z = a·x + b·y + c 를 두 번 맞춰 그 평면 기준 높이를 쓴다.
    g = (np.abs(h) < 0.06) & (r > 0.3) & (r < 2.0)
    for _ in range(2):
        if g.sum() < 30:
            break
        A = np.c_[lv[g, 0], lv[g, 1], np.ones(g.sum())]
        coef, *_ = np.linalg.lstsq(A, lv[g, 2], rcond=None)
        h = lv[:, 2] - (coef[0] * lv[:, 0] + coef[1] * lv[:, 1] + coef[2])
        g = (np.abs(h) < 0.04) & (r > 0.3) & (r < 2.0)
    lv = lv[(h >= min_h) & (h <= max_h) & (r >= rmin) & (r <= rmax)]
    cy, sy = math.cos(yaw), math.sin(yaw)
    W = lv @ np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]).T + np.asarray(t, dtype=np.float64)
    return W.astype(np.float32)


def band_select(pts_body, roll, pitch, floor_below, min_h=0.08, max_h=0.35, rmin=0.30, rmax=2.0, body_box=(-0.40, 0.40, 0.25)):
    """band_world 와 같은 판단(몸 상자 빼기 → 수평 맞춤 → 프레임 바닥 평면 → 바닥 위 [min_h, max_h], rmin~rmax)을 하고
    고른 점의 '원래 몸체 좌표'를 돌려준다(11-05: obstacle_fusion_node 가 다리 기구학 odom 좌표로 기억하려고)."""
    P = np.asarray(pts_body, dtype=np.float64).reshape(-1, 3)
    if not len(P):
        return P
    x0, x1, ym = body_box
    P = P[~((P[:, 0] > x0) & (P[:, 0] < x1) & (np.abs(P[:, 1]) < ym))]
    lv = P @ rot_level(roll, pitch).T
    r = np.hypot(lv[:, 0], lv[:, 1])
    h = lv[:, 2] + floor_below
    g = (np.abs(h) < 0.06) & (r > 0.3) & (r < 2.0)
    for _ in range(2):
        if g.sum() < 30:
            break
        A = np.c_[lv[g, 0], lv[g, 1], np.ones(g.sum())]
        coef, *_ = np.linalg.lstsq(A, lv[g, 2], rcond=None)
        h = lv[:, 2] - (coef[0] * lv[:, 0] + coef[1] * lv[:, 1] + coef[2])
        g = (np.abs(h) < 0.04) & (r > 0.3) & (r < 2.0)
    return P[(h >= min_h) & (h <= max_h) & (r >= rmin) & (r <= rmax)]
