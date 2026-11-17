#!/usr/bin/env python3
"""직진 쏠림(yaw trim) 다시 재기 — 인수인계서(yaw_trim_handover.md) 절차를 junsu 에 맞춘 것. ★ 로봇이 움직인다.

  ~/ws/junsu/tools/yaw_trim_measure.sh            # 측정 → 새 값 계산 → 저장(maps/segments/logs/yaw_trim.json)
  ~/ws/junsu/tools/yaw_trim_measure.sh --check    # 지금 값으로 다시 재서 ±1°/m 안인지만 확인
  ~/ws/junsu/tools/yaw_trim_measure.sh --one 0.25 # 한 속도만 재고 결과를 쌓음(yaw_trim_runs.json) — 클로드가 사용자 말에 맞춰 한 번씩 돌릴 때
  ~/ws/junsu/tools/yaw_trim_measure.sh --fit      # 쌓인 결과로 새 값 계산·저장(쌓인 결과는 비움)

- 측정은 merge scripts/go2_yaw_drift.py 의 live() 를 그대로 쓴다(명령 vx 기준 — bag 모드 값은 쓰지 말 것).
- 걸음새를 먼저 ClassicWalk 로 맞춘다(junsu 러너가 전 구간 ClassicWalk — 예전 값 0.110/-0.002 는 다른 걸음새·0.5 에서만 검증).
- 속도는 junsu 가 실제로 쓰는 범위: 0.25(러너 직접 이동) / 0.4 / 0.5(ego 최고속).
- 새 값 = 지금 값 + 잔여 직선(w = k·v + c): per_vx += k, bias += c (잔여 + = 왼쪽 → 보정 키움).
- 저장하면 go2_ctl 이 모터를 켤 때(M → m) 그 값을 넘긴다. merge 의 bridge_params.yaml 은 안 고친다.
"""
import json
import math
import os
import sys
import time

import numpy as np

J = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TRIM_FILE = os.path.join(J, 'maps', 'segments', 'logs', 'yaw_trim.json')
DEFAULT = (0.110, -0.002)
RUNS_FILE = os.path.join(J, 'maps', 'segments', 'logs', 'yaw_trim_runs.json')
SPEEDS = [(0.25, 8.0), (0.40, 6.0), (0.50, 5.0)]     # (명령 vx, 초) — 대략 2~2.5 m 씩
sys.path.insert(0, os.path.expanduser('~/ws/merge_tmp_20261102/review/scripts'))
import go2_yaw_drift as yd  # noqa: E402  (팀원 측정 도구, 읽기만)


def load_trim():
    try:
        d = json.load(open(TRIM_FILE)); return float(d['per_vx']), float(d['bias'])
    except (OSError, ValueError, KeyError):
        return DEFAULT


def classic_walk():
    """걸음새를 ClassicWalk 로(공식 SportClient::ClassicWalk: api 2049, parameter {"data": true})."""
    import rclpy
    from unitree_api.msg import Request
    rclpy.init(); n = rclpy.create_node('yaw_trim_gait')
    pub = n.create_publisher(Request, '/api/sport/request', 10)
    cmd_pubs = n.count_publishers('/cmd_vel')
    for _ in range(3):
        r = Request(); r.header.identity.id = n.get_clock().now().nanoseconds; r.header.identity.api_id = 2049
        r.parameter = '{"data": true}'; pub.publish(r); time.sleep(0.2)
    n.destroy_node(); rclpy.shutdown()
    return cmd_pubs


def save_trim(trim, res):
    """res: [(vx, wm)] 를 지금 trim 에서 잰 잔여 → 새 값 저장. 반환 (new, worst °/m)."""
    worst = max(abs(math.degrees(w / v)) for v, w in res)
    v = np.array([r[0] for r in res]); w = np.array([r[1] for r in res])
    k, c = (np.polyfit(v, w, 1) if len(set(np.round(v, 3))) >= 2 else (float(np.mean(w / v)), 0.0))
    new = (round(trim[0] + k, 4), round(trim[1] + c, 4))
    hist = []
    try:
        hist = json.load(open(TRIM_FILE)).get('history', [])
    except (OSError, ValueError):
        pass
    hist.append({'time': time.strftime('%m-%d %H:%M'), 'old': list(trim), 'new': list(new),
                 'residual_deg_m': {str(vv): round(math.degrees(ww / vv), 3) for vv, ww in res}, 'gait': 'ClassicWalk'})
    os.makedirs(os.path.dirname(TRIM_FILE), exist_ok=True)
    json.dump({'per_vx': new[0], 'bias': new[1], 'history': hist}, open(TRIM_FILE, 'w'), ensure_ascii=False, indent=1)
    print(f'■ 새 보정값 per_vx {new[0]:.3f}  bias {new[1]:+.3f}  (k {k:+.3f}, c {c:+.3f}) → {TRIM_FILE}')
    return new, worst


def one(vx):
    trim = load_trim(); secs = dict(SPEEDS).get(vx, 6.0)
    print(f'지금 보정값 per_vx {trim[0]:.3f} bias {trim[1]:+.3f} · {vx} m/s {secs:.0f}초 직진 — 멈추려면 Ctrl+C')
    others = classic_walk()
    if others:
        print(f'★ /cmd_vel 을 내는 노드 {others}개 — 러너가 돌고 있으면 측정이 섞인다(관리자 p 후 m)')
    wm = yd.live(vx, secs, trim)
    if wm is None:
        return 1
    try:
        runs = json.load(open(RUNS_FILE))
    except (OSError, ValueError):
        runs = []
    runs.append({'vx': vx, 'wm': wm, 'trim': list(trim), 'time': time.strftime('%H:%M:%S')})
    json.dump(runs, open(RUNS_FILE, 'w'), indent=1)
    print(f'■ {vx} m/s 잔여 {math.degrees(wm / vx):+.2f} °/m [+ = 왼쪽] — 쌓인 측정 {len(runs)}개')
    return 0


def fit():
    trim = load_trim()
    try:
        runs = [r for r in json.load(open(RUNS_FILE)) if tuple(r['trim']) == tuple(trim)]
    except (OSError, ValueError):
        runs = []
    if not runs:
        print('지금 보정값으로 쌓인 측정이 없음'); return 1
    res = [(r['vx'], r['wm']) for r in runs]
    print('속도별 잔여(°/m):', '  '.join(f"{v}: {math.degrees(w / v):+.2f}" for v, w in res))
    worst = max(abs(math.degrees(w / v)) for v, w in res)
    if worst <= 1.0:
        print(f'■ 모두 ±1°/m 안 — 지금 값({trim[0]:.3f}/{trim[1]:+.3f}) 그대로 OK')
    else:
        save_trim(trim, res)
        print('  적용: 관리자에서 M → m')
    os.replace(RUNS_FILE, RUNS_FILE + '.' + time.strftime('%H%M%S'))
    return 0


def main():
    if '--one' in sys.argv:
        return one(float(sys.argv[sys.argv.index('--one') + 1]))
    if '--fit' in sys.argv:
        return fit()
    check = '--check' in sys.argv
    trim = load_trim()
    print(f'지금 보정값: per_vx {trim[0]:.3f}  bias {trim[1]:+.3f}  ({"저장 파일" if os.path.exists(TRIM_FILE) else "기본값"})')
    print('★ 로봇이 앞으로 2~2.5 m 걷는다. 앞 3 m 비우고 리모컨을 손에. 멈추려면 Ctrl+C.')
    others = classic_walk()
    print('걸음새 ClassicWalk 보냄.' + (f'  ★ /cmd_vel 을 내는 노드가 {others}개 있다 — 러너가 돌면 관리자에서 p 후 m' if others else ''))
    res = []
    for vx, secs in SPEEDS:
        input(f'\n[{vx} m/s, {secs:.0f}초] 로봇을 곧은 길 앞에 세우고 엔터 (건너뛰려면 Ctrl+C 아님 — 그냥 엔터 후 리모컨으로 멈춰도 됨) ')
        wm = yd.live(vx, secs, trim)
        if wm is None:
            print('  측정 실패(안 걸었거나 모터 꺼짐) — 이 속도는 빼고 계속'); continue
        dm = math.degrees(wm / vx)
        res.append((vx, wm)); print(f'  → {vx} m/s 잔여 {dm:+.2f} °/m  [+ = 왼쪽]')
        input('  로봇을 리모컨으로 처음 자리로 돌려놓고 엔터 ')
    if not res:
        print('측정값 없음'); return 1
    worst = max(abs(math.degrees(w / v)) for v, w in res)
    print('\n속도별 잔여(°/m):', '  '.join(f'{v}: {math.degrees(w / v):+.2f}' for v, w in res))
    if worst <= 1.0:
        print(f'■ 모두 ±1°/m 안 — 지금 값({trim[0]:.3f}/{trim[1]:+.3f}) 그대로 OK'); return 0
    if check:
        print(f'★ ±1°/m 넘음(최대 {worst:.2f}) — --check 없이 다시 돌려 값을 고칠 것'); return 2
    v = np.array([r[0] for r in res]); w = np.array([r[1] for r in res])
    if len(res) >= 2:
        k, c = np.polyfit(v, w, 1)
    else:
        k, c = float(w[0] / v[0]), 0.0
    new = (round(trim[0] + k, 4), round(trim[1] + c, 4))
    hist = []
    try:
        hist = json.load(open(TRIM_FILE)).get('history', [])
    except (OSError, ValueError):
        pass
    hist.append({'time': time.strftime('%m-%d %H:%M'), 'old': list(trim), 'new': list(new),
                 'residual_deg_m': {str(vv): round(math.degrees(ww / vv), 3) for vv, ww in res}, 'gait': 'ClassicWalk'})
    os.makedirs(os.path.dirname(TRIM_FILE), exist_ok=True)
    json.dump({'per_vx': new[0], 'bias': new[1], 'history': hist}, open(TRIM_FILE, 'w'), ensure_ascii=False, indent=1)
    print(f'■ 새 보정값 per_vx {new[0]:.3f}  bias {new[1]:+.3f}  (k {k:+.3f}, c {c:+.3f}) → {TRIM_FILE}')
    print('  적용: 관리자에서 M → m (모터 노드가 새 값으로 다시 뜸) → 확인: ~/ws/junsu/tools/yaw_trim_measure.sh --check')
    return 0


if __name__ == '__main__':
    sys.exit(main())
