"""몸 모양 안전 필터 판정(ROS 없음) — junsu v2 11-05.

이동 명령(vx, vy, wz, 몸체 좌표)대로 HORIZON 초 움직였을 때 로봇 직사각형(다리 포함)이 벽 점과 겹치는지 DT 간격으로 미리 본다.
겹치면 비슷한 명령(속도 줄이기·회전/옆걸음 조금 바꾸기) 중 안 겹치는 것, 없으면 멈춤(0).
ego 는 로봇을 반지름 0.20 원으로 계획하지만 실제 몸은 0.70 × 0.36(다리 포함) — 돌 때 꼬리(중심에서 ~0.38 m)가 벽을 친다.
이미 겹쳐 있으면(붙어 있거나 위치 잡음) 그 점들은 충돌 판정에서 빼고, 그 점들에서 멀어지는 명령을 고른다.
"""
import math

import numpy as np

FRONT, REAR, HALF_W = 0.34, 0.36, 0.20      # m — ego body 상자(x -0.35..0.33, y ±0.22)와 같은 크기의 다리 포함 몸
MARGIN = 0.03
HORIZON, DT = 1.0, 0.1
SCALES = (1.0, 0.7, 0.5, 0.35, 0.25)   # 벽 앞에서 서서히 줄임(1초 앞을 보므로 0.25×0.5 m/s → 몸 앞 12 cm 남기고 섬)
DWZ = (0.0, -0.2, 0.2, -0.4, 0.4)
DVY = (0.0, -0.1, 0.1)
V_FLOOR = 0.15   # ★ 11-06 m/s: 줄여 줄 때 이 아래(0~0.15)로는 안 냄 — 발 떼기 문턱 근처라 몸만 기울고 제자리(사용자: '0으로 자르면 고착' → 0 아니면 하한 이상)


def poses(vx, vy, wz, horizon=HORIZON, dt=DT):
    """몸체 좌표에서 시작해 일정 명령으로 움직인 자세들 (k, 3) [x, y, yaw]."""
    out = []; x = y = th = 0.0
    for _ in range(int(round(horizon / dt))):
        c, s = math.cos(th), math.sin(th)
        x += (vx * c - vy * s) * dt; y += (vx * s + vy * c) * dt; th += wz * dt
        out.append((x, y, th))
    return np.array(out)


def inside(P, pose, margin=MARGIN):
    """점들(N,2, 지금 몸체 좌표)이 pose 의 몸 직사각형 안인가 (N,) bool."""
    x, y, th = pose; c, s = math.cos(th), math.sin(th); d = P - [x, y]
    lx, ly = c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]
    return (lx <= FRONT + margin) & (lx >= -REAR - margin) & (np.abs(ly) <= HALF_W + margin)


def depth(P, pose):
    """점이 몸 안으로 들어온 깊이(밖이면 0) — 이미 겹친 점에서 멀어지는지 볼 때."""
    x, y, th = pose; c, s = math.cos(th), math.sin(th); d = P - [x, y]
    lx, ly = c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]
    dx = np.minimum(FRONT + MARGIN - lx, lx + REAR + MARGIN); dy = HALF_W + MARGIN - np.abs(ly)
    return np.clip(np.minimum(dx, dy), 0.0, None)


def depth_m(P, pose, margin):
    x, y, th = pose; c, s = math.cos(th), math.sin(th); d = P - [x, y]
    lx, ly = c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]
    dx = np.minimum(FRONT + margin - lx, lx + REAR + margin); dy = HALF_W + margin - np.abs(ly)
    return np.clip(np.minimum(dx, dy), 0.0, None)


def collides(P, cmd, ignore=None, margin=MARGIN, horizon=HORIZON):
    if not len(P):
        return False
    Q = P if ignore is None else P[~ignore]
    if not len(Q):
        return False
    return any(inside(Q, ps, margin).any() for ps in poses(*cmd, horizon=horizon))


# 고착 탈출 단계: 움직이라는 명령을 계속 막고 있으면 풀어 준다(guard_node 가 막힌 시간으로 정함)
LEVELS = [dict(margin=MARGIN, horizon=HORIZON), dict(margin=0.0, horizon=0.6), dict(margin=None, horizon=0.4)]   # 2 = 파고드는 것만 막음


def configure(half_w=None, margin=None, horizon=None, dvy=None):
    """★ 11-06: 몸 폭·여유·내다보는 시간·옆걸음 후보를 바꾼다(노드 파라미터). 새 코스 통로 0.5~0.65 m 에서 0.40+0.06 m 몸·1초는 매번 막혔다(시뮬)."""
    global HALF_W, MARGIN, HORIZON, DVY, LEVELS
    if half_w is not None: HALF_W = float(half_w)
    if margin is not None: MARGIN = float(margin)
    if horizon is not None: HORIZON = float(horizon)
    if dvy is not None: DVY = tuple(dvy)
    LEVELS = [dict(margin=MARGIN, horizon=HORIZON), dict(margin=0.0, horizon=min(0.6, HORIZON)), dict(margin=None, horizon=0.4)]


def side_clear(P):
    """몸 옆(앞뒤 길이 안) 벽까지 왼쪽·오른쪽 여유(m, 몸 반폭 밖) — 가운데로 미는 데 씀."""
    if not len(P):
        return 9.0, 9.0
    band = (P[:, 0] <= FRONT) & (P[:, 0] >= -REAR)
    L = P[band & (P[:, 1] > 0), 1]; R = -P[band & (P[:, 1] < 0), 1]
    return (float(L.min()) - HALF_W if len(L) else 9.0), (float(R.min()) - HALF_W if len(R) else 9.0)


def guard(P, cmd, level=0):
    """P: 벽 점 (N,2) 지금 몸체 좌표. cmd: (vx, vy, wz). → (안전 명령, 이유) 이유: 'ok' | 'changed' | 'stop'.
    level 2: 몸 안으로 더 파고드는(가장 깊은 점이 깊어지는) 움직임만 막는다."""
    vx, vy, wz = cmd
    if abs(vx) < 1e-3 and abs(vy) < 1e-3 and abs(wz) < 1e-3:
        return cmd, 'ok'
    P = np.asarray(P, float).reshape(-1, 2)
    if len(P):
        P = P[np.hypot(P[:, 0], P[:, 1]) < 1.6]          # 1초·최고 0.8 m/s + 몸 길이
    L = LEVELS[min(level, len(LEVELS) - 1)]
    if L['margin'] is None:                                # 2 단계: 0.4초 뒤 몸 안 깊이(여유 0)가 지금보다 깊어지지만 않으면 통과
        if not len(P):
            return cmd, 'ok'
        d0 = depth_m(P, (0.0, 0.0, 0.0), 0.0).max(); d1 = depth_m(P, poses(*cmd, horizon=L['horizon'])[-1], 0.0).max()
        return (cmd, 'ok') if d1 <= max(d0, 0.0) + 1e-3 else ((0.0, 0.0, 0.0), 'stop')
    margin, horizon = L['margin'], L['horizon']
    pre = inside(P, (0.0, 0.0, 0.0), margin) if len(P) else None
    if pre is not None and not pre.any():
        pre = None
    def ok(c):
        if collides(P, c, pre, margin, horizon):
            return False
        if pre is not None:                                 # 이미 겹친 점: 0.5초 뒤 깊이가 늘면 안 됨
            ps = poses(*c, horizon=0.5)[-1]
            return depth(P[pre], ps).max() <= depth(P[pre], (0.0, 0.0, 0.0)).max() + 1e-3
        return True
    if ok(cmd):
        return cmd, 'ok'
    best = None
    lin0 = math.hypot(vx, vy)
    for s in SCALES:
        for dw in DWZ:
            for dv in DVY:
                c = (vx * s, vy * s + dv, wz * s + dw)
                lin = math.hypot(c[0], c[1])
                if lin0 >= V_FLOOR and 1e-3 < lin < V_FLOOR:   # ★ 11-06: 원래 명령은 하한 위인데 줄인 결과가 어중간하면 하한까지만 줄임
                    k = V_FLOOR / lin; c = (c[0] * k, c[1] * k, c[2])
                if (s, dw, dv) == (1.0, 0.0, 0.0) or not ok(c):
                    continue
                cost = abs(c[0] - vx) + 1.5 * abs(c[1] - vy) + 0.5 * abs(c[2] - wz)
                if best is None or cost < best[0]:
                    best = (cost, c)
    if best is None:
        return (0.0, 0.0, 0.0), 'stop'
    return best[1], 'changed'
