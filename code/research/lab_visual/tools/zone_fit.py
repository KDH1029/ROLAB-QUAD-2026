#!/usr/bin/env python3
"""네 존에 **직사각형을 강제**해서 등록 오차를 접는다.

【왜 필요한가 — 2026-09-13】
존은 하나씩 따로 찍는다. 찍을 때마다 AMCL 오차가 그대로 저장되므로, 네 점이
서로 어긋난다. 실제로 지금 등록된 값을 재보면:

    마주보는 변  1-2 vs 3-4 :  4.937 vs 5.213   276 mm 차이
                 2-3 vs 4-1 :  6.715 vs 7.174   459 mm 차이
    대각선       1-3 vs 2-4 :  8.760 vs 8.430   330 mm 차이
    내각         존 2 에서 96.3도  (+6.3도)

경기장이 직사각형이면 저건 전부 **등록 오차**다. 46 cm 다.

【무엇을 하는가】
네 점을 따로 믿는 대신, "네 점은 직사각형을 이룬다" 는 **아는 사실**을 넣는다.
측정 4점(8개 수치)에 미지수 5개(가로, 세로, 위치 x·y, 방향)를 최소제곱으로 맞춘다.
남는 잔차가 곧 측정이 얼마나 나빴는지다. 가로·세로를 도면에서 알면 넣는다 —
그러면 미지수가 3개로 줄어 더 세게 접힌다.

【경기장을 못 쓸 때 왜 이게 답인가】
본선 당일에 네 번 다시 찍을 시간이 없다. 그런데 모양은 이미 안다.
두 점만 제대로 찍어도 나머지는 모양에서 나온다.

ROS 가 없다. 로봇 없이 시험할 수 있다.
"""
import math

import numpy as np

try:
    from scipy.optimize import least_squares
    HAVE_OPT = True
except Exception:
    HAVE_OPT = False

# 반시계로 돈 순서. 존 2(휘문) 에서 시작해 3(강남경찰서) 4(공사현장) 1(코엑스).
CCW = ('2', '3', '4', '1')


def _corners(W, H, tx, ty, th):
    """가로 W, 세로 H 인 직사각형의 네 꼭짓점 — CCW 순서."""
    loc = np.array([[0.0, 0.0], [W, 0.0], [W, H], [0.0, H]])
    c, s = math.cos(th), math.sin(th)
    R = np.array([[c, -s], [s, c]])
    return (R @ loc.T).T + np.array([tx, ty])


def _seed(P):
    """초기 추정 — 변 길이의 평균과 첫 변의 방향."""
    e = [P[(i + 1) % 4] - P[i] for i in range(4)]
    W = (np.linalg.norm(e[0]) + np.linalg.norm(e[2])) / 2
    H = (np.linalg.norm(e[1]) + np.linalg.norm(e[3])) / 2
    th = math.atan2(e[0][1], e[0][0])
    return W, H, P[0][0], P[0][1], th


def fit(meas, order=CCW, W=None, H=None):
    """측정한 존 좌표에 직사각형을 맞춘다.

    meas  : {존번호: (x, y)}  — **네 개를 다 줄 필요는 없다**
    W, H  : 도면에서 아는 값이 있으면 넣는다 (m). 없으면 같이 추정한다.
    돌려줌: dict(fitted={존: (x,y)}, W, H, theta_deg, resid={존: mm}, rms_mm)

    【두 존만 찍어도 되는 이유 — 본선용】
    가로·세로를 알면 남는 미지수는 위치(x, y)와 방향 셋뿐이다.
    존 두 개를 찍으면 수치가 넷이라 셋을 정할 수 있다. 즉
    **두 번만 내려놓으면 네 존이 전부 정해진다.**
    경기장을 미리 못 쓰는데 본선 준비 시간이 짧을 때 이게 유일한 길이다.
    단 두 존은 **서로 이웃**이어야 안정적이다(마주보는 둘은 방향이 덜 잡힌다).
    """
    have = [z for z in order if z in meas]
    if len(have) < 2:
        raise ValueError('존이 최소 두 개는 필요하다 (지금 %d개)' % len(have))
    if len(have) < 4 and (W is None or H is None):
        raise ValueError('존이 %d개뿐이면 가로·세로를 알려줘야 한다 '
                         '(--width, --height)' % len(have))
    idx = [order.index(z) for z in have]
    P = np.array([meas[z] for z in have], dtype=np.float64)
    if len(have) == 4:
        w0, h0, tx0, ty0, th0 = _seed(P)
    else:
        # 두세 점만 있을 때: 첫 두 점이 이루는 변의 방향을 쓴다.
        w0 = W if W is not None else 1.0
        h0 = H if H is not None else 1.0
        v = P[1] - P[0]
        # 그 변이 직사각형의 어느 변인지에 따라 방향 기준이 다르다
        step = (idx[1] - idx[0]) % 4
        th0 = math.atan2(v[1], v[0]) - (math.pi / 2) * (idx[0] if step == 1 else 0)
        c0 = _corners(w0, h0, 0.0, 0.0, th0)
        tx0, ty0 = (P[0] - c0[idx[0]]).tolist()

    def unpack(p):
        if W is not None and H is not None:
            return W, H, p[0], p[1], p[2]
        if W is not None:
            return W, p[0], p[1], p[2], p[3]
        if H is not None:
            return p[0], H, p[1], p[2], p[3]
        return p[0], p[1], p[2], p[3], p[4]

    if W is not None and H is not None:
        p0 = [tx0, ty0, th0]
    elif W is not None:
        p0 = [h0, tx0, ty0, th0]
    elif H is not None:
        p0 = [w0, tx0, ty0, th0]
    else:
        p0 = [w0, h0, tx0, ty0, th0]

    def resid(p):
        C = _corners(*unpack(p))
        return (C[idx] - P).ravel()

    if HAVE_OPT:
        sol = least_squares(resid, p0, method='lm', max_nfev=20000)
        p = sol.x
    else:                                   # scipy 가 없으면 거친 하강법
        p = np.array(p0, dtype=np.float64)
        step = np.array([0.02] * len(p))
        cur = float(np.sum(resid(p) ** 2))
        for _ in range(6000):
            improved = False
            for i in range(len(p)):
                for sgn in (1, -1):
                    q = p.copy(); q[i] += sgn * step[i]
                    v = float(np.sum(resid(q) ** 2))
                    if v < cur:
                        p, cur, improved = q, v, True
                        break
            if not improved:
                step *= 0.5
                if step.max() < 1e-7:
                    break

    Wf, Hf, txf, tyf, thf = unpack(p)
    if Wf < 0:
        Wf = -Wf
    if Hf < 0:
        Hf = -Hf
    C = _corners(Wf, Hf, txf, tyf, thf)
    out = {z: (float(C[i][0]), float(C[i][1])) for i, z in enumerate(order)}
    # 잔차는 **찍은 존에 대해서만** 뜻이 있다. 안 찍은 존은 도면에서 나온 값이다.
    res = {z: float(np.linalg.norm(C[order.index(z)] - np.array(meas[z])) * 1000)
           for z in have}
    rms = float(np.sqrt(np.mean([v ** 2 for v in res.values()])))
    return dict(fitted=out, W=float(Wf), H=float(Hf),
                theta_deg=math.degrees(thf), resid=res, rms_mm=rms)


def report(meas, order=CCW):
    """직사각형이 얼마나 안 맞는지 사람이 읽을 수 있게 낸다."""
    def d(a, b):
        return math.hypot(meas[a][0] - meas[b][0], meas[a][1] - meas[b][1])

    e = [(order[i], order[(i + 1) % 4]) for i in range(4)]
    lines = []
    for a, b in e:
        lines.append('  %s-%s  %.3f m' % (a, b, d(a, b)))
    o1 = abs(d(*e[0]) - d(*e[2])) * 1000
    o2 = abs(d(*e[1]) - d(*e[3])) * 1000
    dg = abs(d(order[0], order[2]) - d(order[1], order[3])) * 1000
    lines.append('  마주보는 변 차이  %.0f mm / %.0f mm' % (o1, o2))
    lines.append('  대각선 차이       %.0f mm' % dg)
    worst = max(o1, o2, dg)
    return lines, worst
