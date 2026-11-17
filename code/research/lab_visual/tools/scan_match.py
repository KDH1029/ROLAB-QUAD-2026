#!/usr/bin/env python3
"""스캔 대 스캔 정합 — 도킹의 '어디로 가야 하나' 를 라이다만으로 푼다.

【왜 이것이 필요한가 — 2026-09-13】
지금 도킹은 AMCL 자세를 보고 목표와의 차이를 구한다. 그래서 AMCL 이 표류하면
도킹도 같이 표류한다. 존이 밀리고, 재정위가 튀고, 앉았다 일어나면 또 틀어진다.
원인이 전부 측위에 있다.

벽은 표류하지 않는다. 존에 제대로 선 자세에서 스캔을 한 장 떠 두고(가르치기),
다음부터는 **지금 스캔이 그 기준 스캔과 같아질 때까지** 움직이면 된다.
odom 도 AMCL 도 지도도 필요 없다.

【이 파일에 ROS 가 없는 이유】
정합은 순수한 수치 계산이다. 로봇 없이 시험할 수 있어야 한다.
경기장 점군으로 스캔을 합성해 시험한 결과(2026-09-13):
    ±12 cm / ±8 도 안에서 12전 12승, 남은 오차 전부 1 mm 이하
    존 앞에 기둥(마네킹 흉내)을 세워도 0.1~1.3 mm
    ±20 cm 를 넘으면 국소최소에 빠질 수 있다 -> 포획 범위는 ±12 cm 로 본다

【좌표계 — 확인된 사실】
/scan 의 target_frame 은 base_stab 이다. stabilized_frame.py 는 평행이동을
넣지 않고(원점이 base_link 와 같다) roll/pitch 만 지운다(yaw 는 보존).
따라서 스캔 점은 **그대로 몸 좌표**다. x 앞, y 왼쪽, 중력 정렬됨.
라이다가 몸보다 25cm 앞에 있는 것은 p2l 이 이미 처리했다 — 여기서 또 빼면 안 된다.
"""
import math

import numpy as np

try:
    from scipy.spatial import cKDTree
    HAVE_KD = True
except Exception:                                   # scipy 가 없어도 돌게 둔다
    HAVE_KD = False


# ── 적합도 문턱 ───────────────────────────────────────────────────────
#
# 【왜 필요한가 — 2026-09-13 경기장 점군 시험】
# 정합은 대부분 0.1~0.9 mm 로 맞지만, 가끔 국소최소에 빠져 48 mm 를 틀린다.
# 다행히 그 둘은 **적합도(대응점 평균거리)로 깨끗하게 갈린다**:
#     잘 맞은 것        3.2 ~  6.2 mm  ->  남은 오차 0.1 ~ 0.9 mm
#     마네킹이 있는 것  12.2 mm        ->  남은 오차 5.5 mm   (여전히 정확)
#     국소최소에 빠진 것 24.7 mm       ->  남은 오차 48.4 mm  ★ 걸러야 한다
# 그래서 15 mm 를 문턱으로 둔다. 이보다 나쁘면 **움직이지 않는다** —
# 못 믿을 정합으로 움직이는 것이 가만히 있는 것보다 나쁘다.
FIT_OK = 0.015


def is_good(fit):
    """이 정합을 믿고 움직여도 되는가."""
    return fit is not None and fit <= FIT_OK


def scan_points(ranges, angle_min, angle_increment,
                range_min=0.0, range_max=float('inf')):
    """LaserScan 의 거리 배열을 몸 좌표 (N,2) 점으로 바꾼다."""
    r = np.asarray(ranges, dtype=np.float64)
    a = angle_min + np.arange(len(r)) * angle_increment
    ok = np.isfinite(r) & (r > max(1e-6, range_min)) & (r < range_max)
    r = r[ok]; a = a[ok]
    return np.stack([r * np.cos(a), r * np.sin(a)], axis=1)


def _nearest(tree, dst, pts):
    if tree is not None:
        return tree.query(pts)
    d = np.linalg.norm(pts[:, None, :] - dst[None, :, :], axis=2)
    j = np.argmin(d, axis=1)
    return d[np.arange(len(pts)), j], j


def match(cur, ref, trim=0.8, iters=60, init=None, tol=1e-6):
    """cur 를 ref 에 포개는 강체변환 (tx, ty, tth) 를 찾는다.

    trim 은 대응점 중 **오차가 작은 쪽 이 비율만** 쓴다는 뜻이다.
    기준 스캔에 없던 것(사람, 마네킹, 다른 로봇)이 섞여도 끌려가지 않는다.
    이것이 없으면 관중 한 명에 도킹이 무너진다.

    돌려주는 값의 뜻:  q_ref = R(tth) q_cur + (tx, ty)
    """
    if len(cur) < 20 or len(ref) < 20:
        return None
    tree = cKDTree(ref) if HAVE_KD else None
    T = np.eye(3)
    if init is not None:
        ix, iy, ith = init
        T[:2, :2] = [[math.cos(ith), -math.sin(ith)],
                     [math.sin(ith), math.cos(ith)]]
        T[:2, 2] = [ix, iy]
    pts = (T[:2, :2] @ cur.T).T + T[:2, 2]
    last = None
    for _ in range(iters):
        dist, j = _nearest(tree, ref, pts)
        k = max(20, int(len(pts) * trim))
        sel = np.argsort(dist)[:k]
        A = pts[sel]; B = ref[j[sel]]
        ca = A.mean(axis=0); cb = B.mean(axis=0)
        H = (A - ca).T @ (B - cb)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[1, :] *= -1
            R = Vt.T @ U.T
        t = cb - R @ ca
        M = np.eye(3); M[:2, :2] = R; M[:2, 2] = t
        T = M @ T
        pts = (R @ pts.T).T + t
        last = float(np.mean(np.sort(dist)[:k]))
        if np.linalg.norm(t) < tol:
            break
    return T[0, 2], T[1, 2], math.atan2(T[1, 0], T[0, 0]), last


def body_correction(tx, ty, tth):
    """정합 결과를 **지금 몸 좌표에서 이만큼 움직여라** 로 바꾼다.

    유도
      정합의 뜻          q_ref = R(tth) q_cur + t
      곧                 (지금 방향 - 기준 방향) = tth
                         (지금 위치 - 기준 위치) = R(기준방향) t      (지도 좌표)
      가야 할 지도 변위   -R(기준방향) t
      그것을 지금 몸 좌표로 :  R(-지금방향) * (-R(기준방향) t) = -R(-tth) t

    돌려주는 것: (dx, dy, dyaw) — 지금 몸 좌표 기준. x 앞, y 왼쪽.
    """
    c, s = math.cos(-tth), math.sin(-tth)
    return -(c * tx - s * ty), -(s * tx + c * ty), -tth


def offset(cur_scan, ref_scan, trim=0.8, init=None):
    """두 스캔(딕셔너리)을 받아 몸 좌표 보정량을 돌려준다.

    스캔 딕셔너리: {'ranges', 'angle_min', 'angle_increment',
                    'range_min', 'range_max'}
    돌려줌: (dx, dy, dyaw, 적합도) 또는 None
      적합도는 대응점 평균 거리(m). 작을수록 잘 맞은 것이다.
    """
    a = scan_points(cur_scan['ranges'], cur_scan['angle_min'],
                    cur_scan['angle_increment'],
                    cur_scan.get('range_min', 0.0),
                    cur_scan.get('range_max', float('inf')))
    b = scan_points(ref_scan['ranges'], ref_scan['angle_min'],
                    ref_scan['angle_increment'],
                    ref_scan.get('range_min', 0.0),
                    ref_scan.get('range_max', float('inf')))
    r = match(a, b, trim=trim, init=init)
    if r is None:
        return None
    tx, ty, tth, fit = r
    dx, dy, dyaw = body_correction(tx, ty, tth)
    return dx, dy, dyaw, fit


def merge_frames(frames):
    """여러 장의 스캔을 각도칸마다 중앙값으로 합친다.

    한 장만 쓰면 잡음 한 점이 정합을 끌 수 있다. 정지 상태에서 몇 장 모아
    중앙값을 쓰면 튀는 값이 사라진다. 칸 구조가 다르면 첫 장을 따른다.
    """
    if not frames:
        return None
    base = frames[0]
    n = len(base['ranges'])
    ok = [f for f in frames if len(f['ranges']) == n]
    arr = np.asarray([f['ranges'] for f in ok], dtype=np.float64)
    arr[~np.isfinite(arr)] = np.nan
    with np.errstate(invalid='ignore'):
        med = np.nanmedian(arr, axis=0)
    out = dict(base)
    out['ranges'] = [None if not np.isfinite(v) else float(v) for v in med]
    out['ranges'] = [float('inf') if v is None else v for v in out['ranges']]
    out['frames'] = len(ok)
    return out
