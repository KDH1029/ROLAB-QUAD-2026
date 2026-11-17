#!/usr/bin/env python3
"""wall_filter_node.cpp의 실제 알고리즘(연속 링 수평거리 일치 검사)을 그대로
Python으로 재현해서, 라바콘(원뿔)이 "벽"으로 판정되어 /mission/wall_cloud에
남는지, 아니면 걸러져서(=ego_planner한테 안 보임) 사라지는지를 수치 시뮬레이션
으로 검증한다. XT16 실제 채널 각도(PandarXT-16_Angle Correction File.csv,
+15~-15도 2도씩 16채널)를 그대로 쓴다.
"""
import numpy as np

# wall_filter_node.cpp 파라미터 (기본값, go2_full_stack.sh에서 오버라이드 안 함)
VERTICAL_TOL_M = 0.03
VERTICAL_MIN_RINGS = 3
VERTICAL_MIN_SPAN_M = 0.08
N_AZ = 360
D_AZ = 2 * np.pi / N_AZ

RING_ELEV_DEG = list(range(15, -16, -2))  # XT16 실측: +15,+13,...,-15 (16채널)
RING_ELEV = np.radians(RING_ELEV_DEG)


def ray_cone_hit(elev, az, cx, cy, h, b, sensor_z_above_ground):
    """센서원점에서 (elev,az) 방향 레이가 원뿔(바닥 반지름 b, 높이 h, 축이
    수직, 바닥원 중심 (cx,cy,-sensor_z_above_ground))의 측면과 만나는 가장 가까운
    교점을 수치적으로 찾는다. 못 만나면 None."""
    dz = np.sin(elev)
    horiz = np.cos(elev)
    dx = horiz * np.cos(az)
    dy = horiz * np.sin(az)

    ts = np.linspace(0.05, 6.0, 12000)
    xs = ts * dx
    ys = ts * dy
    zs = ts * dz

    z_ground = zs + sensor_z_above_ground  # 바닥 기준 높이(0~h 사이여야 원뿔 측면)
    radial = np.hypot(xs - cx, ys - cy)
    cone_r_at_z = np.where((z_ground >= 0) & (z_ground <= h), b * (1 - z_ground / h), -1.0)

    diff = radial - cone_r_at_z
    valid = cone_r_at_z >= 0
    if not valid.any():
        return None
    # 부호가 바뀌는(레이가 원뿔 표면을 통과하는) 첫 지점을 찾는다
    d = np.where(valid, diff, np.nan)
    sign = np.sign(d)
    idx = None
    for i in range(1, len(d)):
        if np.isnan(sign[i - 1]) or np.isnan(sign[i]):
            continue
        if sign[i - 1] > 0 and sign[i] <= 0:
            idx = i
            break
    if idx is None:
        return None
    return float(np.hypot(xs[idx], ys[idx])), float(zs[idx])  # (수평거리 r, 센서기준 z)


def simulate(cone_dist, cone_b, cone_h, sensor_h, label):
    print(f"\n=== {label}: 거리{cone_dist}m, 바닥반지름{cone_b}m, 높이{cone_h}m, "
          f"센서높이{sensor_h}m ===")
    cx, cy = cone_dist, 0.0
    az_center = np.arctan2(cy, cx)
    # 원뿔이 걸리는 방위각 범위를 넉넉히 스캔 (±10도)
    az_range = np.radians(np.arange(-10, 10.01, 0.5)) + az_center

    # az 1도 bin별로 (ring -> 최솟값 r, z) 기록 (실제 코드와 동일한 binning)
    bins = {}
    for az in az_range:
        a_idx = int((az + np.pi) / D_AZ) % N_AZ
        for ring_i, elev in enumerate(RING_ELEV):
            hit = ray_cone_hit(elev, az, cx, cy, cone_h, cone_b, sensor_h)
            if hit is None:
                continue
            r, z = hit
            key = (a_idx, ring_i)
            if key not in bins or r < bins[key][0]:
                bins[key] = (r, z)

    # azimuth bin별로 그룹화 후, 실제 알고리즘(연속 링 r-일치 검사) 그대로 적용
    by_az = {}
    for (a_idx, ring_i), (r, z) in bins.items():
        by_az.setdefault(a_idx, {})[ring_i] = (r, z)

    wall_detected = False
    best_run = (0, 0.0)  # (연속 링 수, z-span)
    for a_idx, ring_map in by_az.items():
        rings_sorted = sorted(ring_map.keys())
        i = 0
        while i < len(rings_sorted):
            j = i
            lo = hi = ring_map[rings_sorted[i]][0]
            zlo = zhi = ring_map[rings_sorted[i]][1]
            # 실제 코드처럼 "ring 인덱스가 연속"일 필요는 코드상 없음(인접 ring 인덱스만 보되
            # 존재하는 ring만 순서대로 봄) -- 여기선 ring_i가 0..15 인덱스라 연속성 그대로 반영
            k = i + 1
            while k < len(rings_sorted) and rings_sorted[k] == rings_sorted[k - 1] + 1:
                r2, z2 = ring_map[rings_sorted[k]]
                nlo, nhi = min(lo, r2), max(hi, r2)
                if nhi - nlo > VERTICAL_TOL_M:
                    break
                lo, hi = nlo, nhi
                zlo, zhi = min(zlo, z2), max(zhi, z2)
                k += 1
            run_len = k - i
            span = zhi - zlo
            if run_len > best_run[0] or (run_len == best_run[0] and span > best_run[1]):
                best_run = (run_len, span)
            if run_len >= VERTICAL_MIN_RINGS and span >= VERTICAL_MIN_SPAN_M:
                wall_detected = True
            i = k if k > i else i + 1

    print(f"  최대 연속 '수평거리 일치' 링 수={best_run[0]} (기준 {VERTICAL_MIN_RINGS}개 이상 필요), "
          f"그 구간 z-span={best_run[1]:.3f}m (기준 {VERTICAL_MIN_SPAN_M}m 이상 필요)")
    print(f"  => wall_filter 판정: {'벽으로 인식됨(통과)' if wall_detected else '필터링됨(ego_planner에 안 보임)'}")
    return wall_detected


print("라바콘(원뿔)이 wall_filter_node를 통과해서 ego_planner 장애물로 보이는지 시뮬레이션")
print(f"(XT16 실제 16채널 각도, wall_filter 기본 파라미터: tol={VERTICAL_TOL_M}m, "
      f"min_rings={VERTICAL_MIN_RINGS}, min_span={VERTICAL_MIN_SPAN_M}m)")

results = []
# 작은 콘(표준 소형, 28~30cm) + 큰 콘(45~50cm) x 여러 거리 x 여러 센서높이 가정
for sensor_h in (0.35, 0.45, 0.55):
    for dist in (0.8, 1.5, 2.5):
        results.append(simulate(dist, 0.09, 0.30, sensor_h, f"소형콘(28~30cm급)"))
for sensor_h in (0.35, 0.45, 0.55):
    for dist in (0.8, 1.5, 2.5):
        results.append(simulate(dist, 0.15, 0.50, sensor_h, f"대형콘(45~50cm급)"))

n_pass = sum(results)
print(f"\n=== 총괄: {len(results)}개 케이스 중 {n_pass}개가 '벽'으로 통과 판정 ===")

# 대조군: 진짜 평평한 벽(참고용, 같은 알고리즘이 벽은 제대로 잡는지 확인)
print("\n--- 대조군: 진짜 평평한 수직 벽(원뿔 대신 b=h인 원기둥으로 근사, 반지름 매우 큼) ---")
simulate(1.5, 50.0, 0.30, 0.45, "거의 평평한 벽(반지름 50m 근사)")
