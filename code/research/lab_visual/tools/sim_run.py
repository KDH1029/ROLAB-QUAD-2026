#!/usr/bin/env python3
"""경기를 통째로 계산해 본다 — 로봇도 스택도 없이.

무엇을 보는가
  · 신호 시계가 실제로 어떻게 걸리는가 (어느 구간에서 몇 초 기다리는가)
  · 주문 8건을 반시계 순환으로 돌면 몇 분이 걸리는가
  · 예선 8분 / 본선 12분 안에 들어오는가
  · 속도를 올리면 얼마나 줄어드는가 (감이 아니라 표로)

무엇을 못 보는가 (반드시 기억할 것)
  이건 **계산**이다. 실제 주행의 실패·재시도·회복동작·도킹 편차가 없다.
  나오는 시간은 **가장 좋은 경우의 하한**이다. 실제는 반드시 이보다 길다.

실행:
  python3 tools/sim_run.py                    기본 (속도 0.5 m/s)
  python3 tools/sim_run.py --speed 0.8
  python3 tools/sim_run.py --sweep            속도별 표
  python3 tools/sim_run.py --skip-empty       배달 없는 존은 안 선다
  python3 tools/sim_run.py --timeline         구간별 시간표를 다 찍는다
"""
import argparse
import json
import math
import os
import sys

GOSU = os.path.expanduser('~/gosu')
HERE = os.path.dirname(os.path.abspath(__file__))

C0 = '\033[0m'; CB = '\033[1m'; CD = '\033[2m'
CG = '\033[32m'; CY = '\033[33m'; CR = '\033[31m'; CC = '\033[36m'

# ── 경기장 값 ─────────────────────────────────────────────────────────
# venue.json 이 있으면 그쪽을 쓰고, 없으면(노트북에서 계산만 할 때)
# 2026-09-13 에 실제로 읽은 값을 쓴다.
FALLBACK_ZONES = {
    '1': (0.254, 4.919), '2': (0.208, -0.018),
    '3': (6.875, -0.817), '4': (7.407, 4.369),
}
FALLBACK_WP = {
    'cw_1-2_00': (0.470, 3.638), 'cw_1-2_01': (0.305, 0.825),
    'cw_2-3_00': (1.001, 0.108), 'cw_2-3_01': (5.690, -0.234),
    'cw_4-1_00': (6.330, 4.324), 'cw_4-1_01': (1.678, 4.450),
    'overpass': (7.722, 2.003),
}
FALLBACK_VIA = {
    '1-2': ['cw_1-2_00', 'cw_1-2_01'],
    '2-3': ['cw_2-3_00', 'cw_2-3_01'],
    '4-1': ['cw_4-1_00', 'cw_4-1_01'],
    '3-4': ['overpass'],
}
LABELS = {'1': '코엑스', '2': '휘문고등학교', '3': '강남경찰서', '4': 'GTX 공사현장'}

GROUPS = [['1-2', '3-4'], ['2-3', '4-1']]   # A 가 먼저 초록
PERIOD = 30.0
CROSS_S = 12.0
MARGIN_S = 2.0
FREE_LEGS = {'3-4': 20.0}                   # 육교 — 신호 무관, 대신 오래 걸린다
CCW = ['1', '2', '3', '4']                  # 반시계 고리
# ★ 2026-09-13 정정: 적재지는 존 2(휘문고등학교)다. 코엑스가 아니다.
#   "2번구역에서 음식 나오기까지 기다렸다가 출발" — 사용자 확인.
LOAD_ZONE = '2'
# 규정: 주문서 한 건에 대해 음식이 **20초에 하나씩** 나온다.
#   5개짜리 주문이면 적재에만 100초다. 이게 시간 예산을 지배한다.
LOAD_PER_ITEM = 20.0

# 존 하나에 서면 드는 시간 (mission.py 실측 구성)
SIT = 0.5 + 1.5          # StopMove + StandDown 대기
STAND = 2.5 + 1.2        # RecoveryStand 대기 + 자세 안정
DOCK = 6.0               # 정밀 도킹에 드는 대략 시간 (실측하면 고칠 것)


def norm(leg):
    return '-'.join(sorted(leg.split('-')))


def load_cfg():
    v = os.path.join(GOSU, 'config', 'venue.json')
    if os.path.exists(v):
        try:
            d = json.load(open(v, encoding='utf-8'))
            zp = {k: (p['x'], p['y']) for k, p in (d.get('zone_poses') or {}).items()}
            wp = {k: (p['x'], p['y']) for k, p in (d.get('waypoints') or {}).items()}
            via = d.get('leg_via') or {}
            if zp and via:
                return zp, wp, via, (d.get('zone_labels') or LABELS), '실제 venue.json'
        except Exception:
            pass
    return FALLBACK_ZONES, FALLBACK_WP, FALLBACK_VIA, LABELS, \
        '내장값 (2026-09-13 실측). venue.json 을 못 읽었다'


def load_orders():
    for p in (os.path.join(GOSU, 'config', 'orders.json'),
              os.path.join(HERE, '..', 'orders.json')):
        if os.path.exists(p):
            try:
                return json.load(open(p, encoding='utf-8'))
            except Exception:
                pass
    return None


def leg_path(zp, wp, via, src, dst):
    """이 구간에서 실제로 지나는 점들. 방향이 반대면 순서를 뒤집는다."""
    k = '%s-%s' % (src, dst)
    rk = '%s-%s' % (dst, src)
    names = list(via.get(k, [])) if k in via else list(reversed(via.get(rk, [])))
    pts = [zp[src]] + [wp[n] for n in names if n in wp] + [zp[dst]]
    return pts


def path_len(pts):
    return sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))


def group_of(leg):
    n = norm(leg)
    for i, g in enumerate(GROUPS):
        if n in [norm(x) for x in g]:
            return i
    return None


def wait_for(t, leg):
    """t 시각에 이 구간을 건너려면 몇 초 기다려야 하나."""
    if norm(leg) in {norm(k) for k in FREE_LEGS}:
        return 0.0
    gi = group_of(leg)
    if gi is None:
        return 0.0
    need = CROSS_S + MARGIN_S
    n = len(GROUPS)
    cyc = t % (PERIOD * n)
    start = gi * PERIOD
    left = (start + PERIOD) - cyc
    if start <= cyc < start + PERIOD and left >= need:
        return 0.0                       # 지금 초록이고 시간도 넉넉하다
    w = start - cyc
    while w < 0:
        w += PERIOD * n
    return w


def simulate(speed, skip_empty, orders, zp, wp, via, timeline=False,
             by_grade=False, load_sec=LOAD_PER_ITEM):
    by_zone = {}
    if orders:
        for o in orders.get('orders', []):
            by_zone.setdefault(str(o['zone']), []).append(o)
    # 랩마다 주문 하나씩 — 용량 5 인데 가장 작은 두 건이 3+3=6 이라 합칠 수 없다
    todo = list(orders.get('orders', [])) if orders else []
    if by_grade:
        # 【왜 순서가 중요한가 — 2026-09-13】
        # 랩 시간이 신호 주기에 양자화돼 **2분**으로 고정된다(아래 결과 참조).
        # 그래서 8건을 다 못 돈다 — 예선 8분이면 4랩, 본선 12분이면 6랩이다.
        # 그러면 **무엇을 먼저 싣느냐**가 점수를 정한다. 배점이 높은 것부터 싣는다.
        rank = {'상': 0, '중': 1, '하': 2}
        todo.sort(key=lambda o: (rank.get(o.get('grade'), 9), -int(o.get('num', 0))))
    laps = max(1, len(todo))

    t = 0.0
    wait_tot = drive_tot = stop_tot = load_tot = 0.0
    rows = []
    lap_end = []            # 각 랩이 끝난 시각 — 제한시간 안에 몇 건이 들어오나
    for lap in range(laps):
        order = todo[lap] if lap < len(todo) else None
        target = str(order['zone']) if order else None
        # 적재지에 서서 음식이 다 나올 때까지 기다린다
        n_items = sum((order.get('items') or {}).values()) if order else 0
        food = n_items * load_sec
        t += SIT + STAND + DOCK + food
        stop_tot += SIT + STAND + DOCK
        load_tot += food
        # 앞 랩의 주문이 **적재지행**이었다면 여기서 내린다.
        # (예전에는 dst==LOAD_ZONE 에서 continue 해버려 기록에서 통째로 빠졌다)
        if lap > 0:
            prev = todo[lap - 1]
            if str(prev['zone']) == LOAD_ZONE:
                lap_end.append((t, prev))
                rows.append((t, lap + 1, LOAD_ZONE, '배달',
                             '%s번 배달(적재지와 같은 곳)' % prev['num'], 0.0))
        rows.append((t, lap + 1, LOAD_ZONE, '적재',
                     '%s번 %d개 (%.0f초 대기)' % (order['num'], n_items, food)
                     if order else '—', 0.0))
        j0 = CCW.index(LOAD_ZONE)
        loop = CCW[j0:] + CCW[:j0]       # 적재지에서 출발하는 반시계 고리
        for i in range(len(loop)):
            src = loop[i]
            dst = loop[(i + 1) % len(loop)]
            leg = '%s-%s' % (src, dst)
            w = wait_for(t, leg)
            t += w
            wait_tot += w
            pts = leg_path(zp, wp, via, src, dst)
            L = path_len(pts)
            free = norm(leg) in {norm(k) for k in FREE_LEGS}
            dt = FREE_LEGS[norm(leg)] if free else L / speed
            t += dt
            drive_tot += dt
            rows.append((t, lap + 1, leg, '육교' if free else '주행',
                         '%.2fm' % L, w))
            if dst == LOAD_ZONE:
                continue                  # 적재지는 랩 머리에서 처리한다
            stop_here = (dst == target) or (not skip_empty)
            if stop_here:
                s = SIT + STAND + DOCK
                t += s
                stop_tot += s
                rows.append((t, lap + 1, dst, '정지',
                             ('%s번 배달' % order['num']) if dst == target
                             else '배달 없음', 0.0))
                if dst == target:
                    lap_end.append((t, order))
    return dict(total=t, wait=wait_tot, drive=drive_tot, stop=stop_tot,
                load=load_tot, laps=laps, rows=rows, lap_end=lap_end, todo=todo)


def fmt(s):
    return '%d분 %02d초' % (int(s) // 60, int(s) % 60)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--speed', type=float, default=0.5, help='평균 주행 속도 m/s')
    ap.add_argument('--load-sec', type=float, default=LOAD_PER_ITEM,
                    help='음식 1개가 나오는 데 걸리는 초 (규정 20초)')
    ap.add_argument('--skip-empty', action='store_true',
                    help='그 랩에 배달할 게 없는 존은 서지 않고 지나간다')
    ap.add_argument('--sweep', action='store_true', help='속도별 표')
    ap.add_argument('--by-grade', action='store_true',
                    help='배점 높은 주문부터 싣는다 (상 -> 중 -> 하)')
    ap.add_argument('--timeline', action='store_true', help='구간별 시간표를 다 찍는다')
    a = ap.parse_args()

    zp, wp, via, lab, src_note = load_cfg()
    orders = load_orders()
    print('  %s설정 출처%s  %s' % (CD, C0, src_note))
    if orders is None:
        print('  %s주문서를 못 읽었다 — 1랩만 계산한다%s' % (CY, C0))
    print()

    print('  %s구간 거리%s (통과점을 지나는 실제 경로)' % (CB, C0))
    for i in range(len(CCW)):
        s_, d_ = CCW[i], CCW[(i + 1) % len(CCW)]
        leg = '%s-%s' % (s_, d_)
        L = path_len(leg_path(zp, wp, via, s_, d_))
        gi = group_of(leg)
        free = norm(leg) in {norm(k) for k in FREE_LEGS}
        tag = '육교(신호 무관, %.0f초)' % FREE_LEGS[norm(leg)] if free else \
            '묶음 %s' % ('AB'[gi] if gi is not None else '?')
        print('    %s  %s -> %s   %5.2f m   %s'
              % (leg, lab.get(s_, ''), lab.get(d_, ''), L, tag))
    print()
    print('  %s존 하나에 서면%s  앉기 %.1f + 일어서기 %.1f + 도킹 %.1f = %.1f초'
          % (CB, C0, SIT, STAND, DOCK, SIT + STAND + DOCK))
    print('  %s신호%s  주기 %.0f초, 건너는 데 %.0f+%.0f초 필요. 시작은 %s 초록'
          % (CB, C0, PERIOD, CROSS_S, MARGIN_S, ', '.join(GROUPS[0])))
    print()

    if a.sweep:
        print('  %s속도별 총 시간%s' % (CB, C0))
        print('    %-7s %-12s %-12s %-12s %-12s  %s'
              % ('속도', '전부 정지', '(대기)', '배달만 정지', '(대기)', '예선8분'))
        for sp in (0.3, 0.4, 0.5, 0.6, 0.8, 1.0):
            r1 = simulate(sp, False, orders, zp, wp, via)
            r2 = simulate(sp, True, orders, zp, wp, via)
            ok = CG + '들어감' + C0 if r2['total'] <= 480 else CR + '초과' + C0
            print('    %-7s %-12s %-12s %-12s %-12s  %s'
                  % ('%.1f m/s' % sp, fmt(r1['total']), fmt(r1['wait']),
                     fmt(r2['total']), fmt(r2['wait']), ok))
        print()
        print('  %s예선 8분 = 480초,  본선 12분 = 720초%s' % (CD, C0))
        return 0

    r = simulate(a.speed, a.skip_empty, orders, zp, wp, via, a.timeline,
                 by_grade=a.by_grade, load_sec=a.load_sec)
    if a.timeline:
        print('  %s시간표%s  (속도 %.1f m/s, %s)'
              % (CB, C0, a.speed,
                 '배달 있는 존에만 정지' if a.skip_empty else '모든 존에 정지'))
        for t, lap, what, kind, note, w in r['rows']:
            wtag = '  %s신호대기 %.0f초%s' % (CY, w, C0) if w > 0.5 else ''
            print('    %8s  랩%-2d %-6s %-5s %-14s%s'
                  % (fmt(t), lap, what, kind, note, wtag))
        print()
    print('  %s합계%s  랩 %d회' % (CB, C0, r['laps']))
    print('    주행    %s' % fmt(r['drive']))
    print('    %s적재대기 %s%s  (음식 %.0f초/개)'
          % (CY, fmt(r['load']), C0, a.load_sec))
    print('    신호대기 %s' % fmt(r['wait']))
    print('    정지    %s' % fmt(r['stop']))
    print('    %s전체    %s%s' % (CB, fmt(r['total']), C0))
    for lim, name in ((480, '예선 8분'), (720, '본선 12분')):
        col = CG if r['total'] <= lim else CR
        print('    %s%s: %s%s' % (col, name,
                                  '들어감 (%s 남음)' % fmt(lim - r['total'])
                                  if r['total'] <= lim
                                  else '초과 (%s 모자람)' % fmt(r['total'] - lim), C0))
    if r.get('lap_end'):
        print()
        print('  %s제한시간 안에 끝나는 배달%s%s' % (CB, C0,
              '  (배점 높은 것부터 실음)' if a.by_grade else '  (주문서 순서대로)'))
        for lim, name in ((480, '예선 8분'), (720, '본선 12분')):
            done = [(t, o) for t, o in r['lap_end'] if t <= lim]
            g = {}
            for _, o in done:
                g[o.get('grade', '?')] = g.get(o.get('grade', '?'), 0) + 1
            print('    %-8s %d건 완료  %s' % (name, len(done),
                  ' '.join('%s %d' % (k, v) for k, v in sorted(g.items())) or '—'))
            for t, o in done:
                print('        %8s  %s번 (%s) -> 존 %s'
                      % (fmt(t), o['num'], o.get('grade'), o['zone']))
    print()
    print('  %s★ 이건 계산이다. 실패·재시도·회복동작·도킹 편차가 없다.%s' % (CY, C0))
    print('  %s  실제 주행은 반드시 이보다 길다. 하한으로만 볼 것.%s' % (CY, C0))
    return 0


if __name__ == '__main__':
    sys.exit(main())
