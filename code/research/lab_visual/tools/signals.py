#!/usr/bin/env python3
"""경기장 신호등 시간표. 30초마다 바뀌는 걸 시계로 안다.

  python3 tools/signals.py start              출발 신호 순간에 실행 — 여기가 t=0
  python3 tools/signals.py sync 4-1           "지금 4-1 이 막 초록이 됐다" 로 맞춤
  python3 tools/signals.py now                지금 어느 쪽이 초록이고 몇 초 남았나
  python3 tools/signals.py plan 1-4 4-3 3-2   이 순서로 건널 때 언제 건너고 얼마나 기다리나
  python3 tools/signals.py wait 1-4           건널 수 있을 때까지 기다렸다가 끝난다 (종료코드 0)
  python3 tools/signals.py route 1 3          1번에서 3번까지 **가장 빨리 닿는 길**과 시간표
  python3 tools/signals.py tour 1 2 3 4       이 존들을 다 들르는 **가장 빠른 순서**

【규칙】
존은 교차로 네 모서리에 있고 **대각선으로는 못 간다** — 1에서 3으로 가려면 2나 4를 거친다.
횡단보도는 넷뿐이다: 1-2, 2-3, 3-4, 4-1.
4방향 교차로의 횡단보도 넷은 평행한 두 쌍으로 묶여 번갈아 초록이 된다.
  묶음 A : 4-1, 2-3     <- 경기 시작 순간 **이쪽이 초록**
  묶음 B : 1-2, 3-4
주기 30초. 즉 t=0~30 A 초록, t=30~60 B 초록, t=60~90 A 초록 … 경기 끝까지.

【2-3 은 육교·지하차도라 신호를 안 본다】
신호가 빨강이든 초록이든 건널 수 있다. 대신 오르내리느라 평지보다 오래 걸린다(기본 20초).
그래서 묶음 A 가 실제로 가두는 것은 4-1 하나뿐이다. 경로 계산이 이걸 반영한다.

【시계와 신호등 인식은 서로 발목을 잡지 않는다】
둘은 **의존 관계가 아니다.** 어느 한쪽이 없어도 나머지가 혼자 돌아간다.
  · 판단은 **시계가 한다.** 건널지 말지는 오직 t0 와 30초 주기로 정한다.
  · 신호등 인식은 **감시만 한다.** 시계와 어긋나면 알리고, 계속 어긋나면 사람에게
    `sync` 를 권한다. **혼자서는 절대 주행을 막지 않는다.**
  · 인식이 죽어도 시계는 그대로 돈다. 시계가 없으면(t0 미설정) 인식이 sync 를 한 번
    도와줄 수는 있지만, 그때도 인식이 통과·정지를 결정하지는 않는다.
이렇게 두는 이유: 사진 120장 시험에서 17%가 빨강·초록을 동시에 잡았다(4방향이라 대각선이
같이 보인다). 그런 신호를 통과 판정에 걸면 동전던지기가 되고, 시계와 엮으면 둘 다 멈춘다.

【왜 시계로 판단하나】
신호는 결정론적이다. 30초 주기이고 시작 위상을 안다. 그러면 시계가 가장 정확한 판정기다.
YOLO 검출은 **시계를 검증하는 쪽**으로 쓴다 — 사진 120장 시험에서 17%가 빨강·초록을 동시에
잡았다(4방향이라 대각선 신호가 같이 보인다). 거기에 통과·정지를 걸면 동전던지기가 된다.
시계가 틀어졌는지만 YOLO 로 보고, 틀어졌으면 사람이 sync 로 다시 맞춘다.

【시계는 하나만 쓴다】
t0 도 판정도 전부 이 기계의 벽시계(time.time())로 한다. ROS 시간·sim time 을 섞지 않는다.
섞으면 어느 쪽이 맞는지 아무도 모르게 된다.

【대각선은 반드시 신호를 한 번 기다린다】
1-4 는 묶음 A, 4-3 은 묶음 B 다. 이웃한 두 구간은 **항상 다른 묶음**이라, 연달아 건널 수 없다.
그래서 1->3 은 어느 길로 가든 중간에 한 번 신호를 기다린다. 어느 쪽으로 도느냐가 그 대기를 정한다.
실제로 t=0(A 초록)에 1에서 출발하면:
    1->4->3 : 바로 건너고(0~12) B 를 18초 기다려 30~42.  **42초에 도착**
    1->2->3 : B 를 30초 기다려 30~42, 다시 A 를 18초 기다려 60~72.  72초에 도착
같은 목적지인데 30초가 갈린다. `route` 가 이 계산을 대신 한다.

【건너는 조건】
기본은 **다 건널 때까지 초록이 남아 있을 것**. 즉 남은 초록 >= 건너는시간 + 여유.
모자라면 다음 초록이 켜지는 순간까지 기다렸다가 **켜지자마자** 출발한다.
(규정이 "초록에 출발만 하면 된다" 면 --only-start 로 바꾼다.)
"""
import argparse
import json
import os
import sys
import time

GOSU = os.path.expanduser('~/gosu')
VENUE = os.path.join(GOSU, 'config', 'venue.json')

# ── 신호 묶음 — 경기 시작 순간 A 가 초록 ──────────────────────────────
# ★ 2026-09-13 정정: 예전 값은 [['4-1','2-3'], ['1-2','3-4']] 로 A/B 가
#   뒤바뀌어 있었다. 존 번호를 +1 로 재정렬하기 전 값이다.
#   venue.json 에 값이 있으면 그쪽이 이기므로(setdefault) 지금까지 드러나지
#   않았지만, venue.json 이 한 번이라도 재생성되면 신호가 통째로 뒤집힌다.
#   맞는 값: 짧은 횡단보도(1-2, 3-4)가 먼저 초록이다.
GROUPS = [['1-2', '3-4'], ['2-3', '4-1']]   # A(시작 초록), B
PERIOD = 30.0
CROSS_S = 12.0      # 횡단보도 하나 건너는 데 걸리는 시간. 실측하면 고친다.
MARGIN_S = 2.0      # 안전 여유

# ── 신호와 무관하게 건널 수 있는 구간 ──────────────────────────────────
# 3-4 사이에는 육교(또는 지하차도)가 있어 신호등과 상관없이 건넌다.
# 대신 오르내리느라 평지 횡단보도보다 오래 걸린다. 그 가중치를 따로 둔다.
# ★ 2026-09-13 정정: 육교는 2-3 이 아니라 **3-4**(강남경찰서<->GTX공사현장)다.
#   위 GROUPS 와 같은 이유로 낡아 있었다.
FREE_LEGS = {'3-4': 20.0}   # 구간 -> 건너는 데 걸리는 초


def guard_path(p):
    a = os.path.abspath(os.path.expanduser(p))
    if not a.startswith(os.path.abspath(GOSU)):
        raise SystemExit('★ gosu 밖에는 쓰지 않는다: %s\n'
                         '   남의 작업환경을 오염시키지 않는 것이 대원칙이다.' % a)
    return a


def load():
    if os.path.exists(VENUE):
        try:
            d = json.load(open(VENUE, encoding='utf-8'))
        except Exception:
            d = {}
    else:
        d = {}
    s = d.get('signals') or {}
    s.setdefault('period_s', PERIOD)
    s.setdefault('groups', GROUPS)
    s.setdefault('cross_seconds', CROSS_S)
    s.setdefault('margin_s', MARGIN_S)
    s.setdefault('t0_epoch', None)
    s.setdefault('pause_total', 0.0)     # 멈춰 있던 시간의 합(초)
    s.setdefault('paused_at', None)      # 지금 멈춰 있으면 그 시작 시각
    s.setdefault('free_legs', FREE_LEGS)
    return d, s


def save(d, s):
    d['signals'] = s
    guard_path(VENUE)
    os.makedirs(os.path.dirname(VENUE), exist_ok=True)
    json.dump(d, open(VENUE, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)


def norm(leg):
    """'1-4' 와 '4-1' 을 같은 것으로 본다."""
    p = [x.strip() for x in str(leg).replace('to', '-').split('-') if x.strip()]
    if len(p) != 2:
        raise SystemExit('★ 횡단보도는 "1-4" 처럼 준다: %s' % leg)
    return tuple(sorted(p))


def free_cost(leg, s):
    """신호와 무관하게 건널 수 있으면 그 소요초, 아니면 None."""
    k = norm(leg)
    for f, sec in (s.get('free_legs') or {}).items():
        if norm(f) == k:
            return float(sec)
    return None


def group_of(leg, groups):
    k = norm(leg)
    for gi, g in enumerate(groups):
        if any(norm(x) == k for x in g):
            return gi
    raise SystemExit('★ 모르는 횡단보도: %s\n   있는 것: %s'
                     % (leg, ', '.join(x for g in groups for x in g)))


def elapsed_of(s, at=None):
    """시계가 실제로 흐른 시간. **멈춰 있던 시간은 빼고** 센다.

    【왜 멈출 수 있어야 하나 — 2026-09-13】
    횡단보도 패널티를 받으면 심판이 로봇을 원래 자리로 되돌린다.
    그 동안 로봇은 주행하지 않는다. 그런데 우리 시계는 계속 흘러서,
    돌아왔을 때 우리가 믿는 신호 위상과 실제 신호가 어긋난다.
    비전(YOLO)을 못 쓰므로 그 어긋남을 알아챌 방법이 없다 — 그대로 빨간불에 건넌다.

    그래서 세 값을 들고 있는다:
      t0_epoch    위상 A 가 초록이던 기준 시각
      pause_total 지금까지 멈춰 있던 시간의 합
      paused_at   지금 멈춰 있다면 그 시작 시각 (안 멈췄으면 None)

    ★ 주의: 실제 경기장 신호등은 우리가 멈춰도 계속 돈다. 이 일시정지는
      **대회 진행 자체가 멈추는 경우**에만 맞다(심판이 시계를 세우는 경우).
      진행이 안 멈춘다면 멈추지 말고 돌아온 뒤 `signal sync` 로 다시 맞춰라.
      둘을 헷갈리면 빨간불에 건넌다.
    """
    t0 = s.get('t0_epoch')
    if t0 is None:
        return None
    now = time.time() if at is None else at
    el = now - float(t0) - float(s.get('pause_total') or 0.0)
    pa = s.get('paused_at')
    if pa:
        el -= (now - float(pa))          # 멈춘 뒤로 흐른 만큼 더 뺀다
    return max(0.0, el)


def is_paused(s):
    return bool(s.get('paused_at'))


def state(s, at=None):
    """지금(또는 at 시각)의 위상과 각 묶음의 남은 초."""
    t0 = s.get('t0_epoch')
    if t0 is None:
        return None
    per = float(s['period_s'])
    el = elapsed_of(s, at)
    cyc = el % (per * len(s['groups']))
    gi = int(cyc // per)                 # 지금 초록인 묶음
    left = per - (cyc % per)             # 그 초록이 남은 초
    return dict(elapsed=el, green=gi, left=left, period=per,
                paused=is_paused(s))


def next_window(s, gi, need, at=None):
    """묶음 gi 를 need 초 동안 쓸 수 있는 **가장 이른 시각**까지의 대기 초."""
    st = state(s, at)
    per = float(s['period_s'])
    n = len(s['groups'])
    if st['green'] == gi and st['left'] >= need:
        return 0.0, st['left']
    # 다음에 gi 가 켜지기까지
    # ★ 여기도 반드시 elapsed_of 를 쓴다. state() 와 다른 시간축을 쓰면
    #   "지금 초록" 과 "다음 초록까지" 가 서로 어긋난다.
    cyc = elapsed_of(s, at) % (per * n)
    start_of_gi = gi * per
    wait = start_of_gi - cyc
    if wait < 0:
        wait += per * n
    return wait, per


def fmt(sec):
    return '%d분 %02d초' % (int(sec) // 60, int(sec) % 60) if sec >= 60 else '%.1f초' % sec


def _clear_pause(s):
    """시계를 새로 잡으면 누적 정지시간도 같이 지운다.

    안 지우면 elapsed_of 가 예전 정지시간을 계속 빼서, 새로 맞춘 시계가
    그만큼 과거에 머문다. 2026-09-13 에 일시정지를 넣으면서 생긴 함정이다.
    """
    s['pause_total'] = 0.0
    s['paused_at'] = None


def cmd_start(a, d, s):
    _clear_pause(s)
    s['t0_epoch'] = time.time()
    s['t0_note'] = '출발 신호 순간. 이때 %s 가 초록이었다.' % ', '.join(s['groups'][0])
    save(d, s)
    print('  t0 를 지금으로 잡았다. %s 초록으로 시작.' % ', '.join(s['groups'][0]))
    cmd_now(a, d, s)


def cmd_sync(a, d, s):
    """'지금 이 횡단보도가 막 초록이 됐다' 로 시계를 다시 맞춘다."""
    if not a.leg:
        raise SystemExit('★ 어느 횡단보도가 방금 초록이 됐는지 줄 것 (예: 4-1)')
    gi = group_of(a.leg[0], s['groups'])
    per = float(s['period_s'])
    # 지금이 묶음 gi 의 초록 시작이 되도록 t0 를 역산한다.
    _clear_pause(s)
    s['t0_epoch'] = time.time() - gi * per
    s['t0_note'] = '%s 가 초록이 되는 순간에 맞춤' % a.leg[0]
    save(d, s)
    print('  %s 초록 시작에 맞췄다.' % a.leg[0])
    cmd_now(a, d, s)


def cmd_pause(a, d, s):
    """시계를 세운다. 패널티로 로봇을 원위치시키는 동안 쓴다.

    ★ 대회 진행 자체가 멈출 때만 맞다. 실제 신호등이 계속 돌고 있다면
      멈추지 말고, 돌아온 뒤 `sync <구간>` 으로 다시 맞춰라.
    """
    if s.get('t0_epoch') is None:
        print('  ★ 아직 시계가 없다. start 나 sync 로 먼저 맞출 것')
        return 1
    if is_paused(s):
        print('  이미 멈춰 있다 (%s 전부터)' % fmt(time.time() - float(s['paused_at'])))
        return 0
    s['paused_at'] = time.time()
    save(d, s)
    st = state(s)
    print('  %s시계를 세웠다%s  경과 %s 에서 멈춤. 지금 초록: %s'
          % ('\033[1;33m', '\033[0m', fmt(st['elapsed']),
             ', '.join(s['groups'][st['green']])))
    print('  다시 돌리려면:  signals.py resume')
    print('  실제 신호가 계속 돌았다면 resume 대신:  signals.py sync <막 초록이 된 구간>')
    return 0


def cmd_resume(a, d, s):
    """세워 둔 시계를 다시 돌린다. 멈춰 있던 만큼은 안 흐른 것으로 친다."""
    if not is_paused(s):
        print('  멈춰 있지 않다. 그냥 돌고 있다.')
        return cmd_now(a, d, s)
    was = time.time() - float(s['paused_at'])
    s['pause_total'] = float(s.get('pause_total') or 0.0) + was
    s['paused_at'] = None
    save(d, s)
    print('  %s시계를 다시 돌린다%s  %s 동안 멈춰 있었다 (누적 %s)'
          % ('\033[32m', '\033[0m', fmt(was), fmt(float(s['pause_total']))))
    return cmd_now(a, d, s)


def cmd_now(a, d, s):
    st = state(s)
    if st is None:
        print('  ★ t0 가 없다. 출발 신호에 맞춰 `signals.py start` 를 실행할 것')
        print('     이미 경기가 시작됐으면 `signals.py sync 4-1` (막 초록이 된 쪽)')
        return 1
    print()
    if st.get('paused'):
        print('  %s★ 시계가 멈춰 있다%s  (resume 하기 전까지 위상이 안 바뀐다)'
              % ('\033[1;33m', '\033[0m'))
    print('  경과        %s%s' % (fmt(st['elapsed']),
                                 '  (멈춤 누적 %s)' % fmt(float(s.get('pause_total') or 0))
                                 if float(s.get('pause_total') or 0) > 0 else ''))
    for gi, g in enumerate(s['groups']):
        mark = '초록' if gi == st['green'] else '빨강'
        extra = '  (%.1f초 남음)' % st['left'] if gi == st['green'] else ''
        print('  %-9s %s%s' % (', '.join(g), mark, extra))
    for f, sec in (s.get('free_legs') or {}).items():
        print('  %-9s 육교·지하차도 — 언제나 건널 수 있다 (%.0f초)' % (f, sec))
    need = float(s['cross_seconds']) + float(s['margin_s'])
    print()
    print('  건너는 데 필요 %.0f초 (건너기 %.0f + 여유 %.0f)'
          % (need, s['cross_seconds'], s['margin_s']))
    for gi, g in enumerate(s['groups']):
        w, _ = next_window(s, gi, need)
        print('  %-9s %s' % (', '.join(g),
                             '지금 건널 수 있다' if w <= 0 else '%s 뒤' % fmt(w)))
    return 0


def cmd_plan(a, d, s):
    st = state(s)
    if st is None:
        print('  ★ t0 가 없다 — 시작 시각을 0으로 가정하고 시간표만 보여준다')
        base = 0.0
    else:
        base = st['elapsed']
    need = float(s['cross_seconds']) + float(s['margin_s'])
    per = float(s['period_s'])
    n = len(s['groups'])
    t = base
    total_wait = 0.0
    print()
    print('  구간    묶음   출발(t=)   대기      건너고 나면')
    for leg in a.leg:
        gi = group_of(leg, s['groups'])
        cyc = t % (per * n)
        left = per - (cyc % per) if int(cyc // per) == gi else -1
        if left >= need:
            wait = 0.0
        else:
            start_of_gi = gi * per
            wait = start_of_gi - cyc
            if wait < 0:
                wait += per * n
        t += wait
        print('  %-6s  %s     %6.1f초   %-8s  %6.1f초'
              % (leg, 'AB'[gi], t, fmt(wait) if wait > 0 else '없음',
                 t + s['cross_seconds']))
        t += float(s['cross_seconds'])
        total_wait += wait
    print()
    print('  총 소요 %s 중 **신호 대기 %s**' % (fmt(t - base), fmt(total_wait)))
    if total_wait > 0:
        print('  순서를 바꾸면 대기를 줄일 수 있다. 다른 순서로 다시 돌려볼 것.')
    return 0


def cmd_wait(a, d, s):
    if not a.leg:
        raise SystemExit('★ 어느 횡단보도인지 줄 것 (예: 1-4)')
    st = state(s)
    if st is None:
        print('  ★ t0 가 없다 — 신호를 모른다. 기다리지 않고 통과시킨다.')
        print('     출발 신호에 맞춰 `signals.py start` 를 먼저 실행할 것')
        return 0 if a.allow_unknown else 2
    fc = free_cost(a.leg[0], s)
    if fc is not None:
        print('  %s 는 육교·지하차도다 — 신호를 보지 않는다. 바로 건넌다.' % a.leg[0])
        return 0
    gi = group_of(a.leg[0], s['groups'])
    need = float(a.need) if a.need else float(s['cross_seconds']) + float(s['margin_s'])
    if a.only_start:
        need = 0.1
    w, green_len = next_window(s, gi, need)
    if w <= 0:
        st = state(s)
        print('  %s 초록. %.1f초 남음 — 지금 건넌다.' % (a.leg[0], st['left']))
        return 0
    print('  %s 빨강. %s 뒤에 초록이 된다. 기다린다…' % (a.leg[0], fmt(w)))
    end = time.time() + w
    try:
        while True:
            r = end - time.time()
            if r <= 0:
                break
            print('\r    남은 대기 %5.1f초    ' % r, end='', flush=True)
            time.sleep(0.2 if r < 2 else 0.5)
    except KeyboardInterrupt:
        print('\n  기다리기를 중단했다. 건너지 않는다.')
        return 130
    print('\r  %s 초록. 지금 건넌다.            ' % a.leg[0])
    return 0



# ── 존 그래프 ────────────────────────────────────────────────────────────
# 대각선 횡단보도는 없다. 이웃끼리만 건넌다.
ZONES = ['1', '2', '3', '4']
EDGES = [('1', '2'), ('2', '3'), ('3', '4'), ('4', '1')]


def neighbors(z):
    out = []
    for a, b in EDGES:
        if a == z:
            out.append(b)
        elif b == z:
            out.append(a)
    return out


def earliest_cross(s, leg, t):
    """시각 t(경과초)에 그 횡단보도 앞에 서 있을 때, **건너기 시작하는 시각**."""
    if free_cost(leg, s) is not None:
        return t          # 육교·지하차도 — 신호를 안 본다
    gi = group_of(leg, s['groups'])
    per = float(s['period_s'])
    n = len(s['groups'])
    need = float(s['cross_seconds']) + float(s['margin_s'])
    cyc = t % (per * n)
    if int(cyc // per) == gi and (per - (cyc % per)) >= need:
        return t
    w = gi * per - cyc
    if w < 0:
        w += per * n
    return t + w


def cmd_route(a, d, s):
    if len(a.leg) < 2:
        raise SystemExit('★ 출발 존과 도착 존을 줄 것 (예: route 1 3)')
    src, dst = str(a.leg[0]), str(a.leg[1])
    for z in (src, dst):
        if z not in ZONES:
            raise SystemExit('★ 존은 1~4 다: %s' % z)
    st = state(s)
    t_now = st['elapsed'] if st else 0.0
    if st is None:
        print('  ★ t0 가 없다 — 경기 시작(t=0, %s 초록)을 가정한다'
              % ', '.join(s['groups'][0]))
    cs = float(s['cross_seconds'])

    # 시각에 따라 비용이 달라지는 최단 **도착시각** 탐색. 존이 넷뿐이라 전부 펼친다.
    best = {src: (t_now, [])}
    frontier = [(t_now, src, [])]
    seen_paths = []
    while frontier:
        frontier.sort()
        t, z, path = frontier.pop(0)
        if z == dst:
            seen_paths.append((t, path))
            continue
        if len(path) > len(ZONES):
            continue
        for nb in neighbors(z):
            leg = '%s-%s' % (z, nb)
            go = earliest_cross(s, leg, t)
            fc = free_cost(leg, s)
            arr = go + (fc if fc is not None else cs)
            if nb not in best or arr < best[nb][0] - 1e-9:
                best[nb] = (arr, path + [(leg, go, arr)])
                frontier.append((arr, nb, path + [(leg, go, arr)]))
    if not seen_paths:
        raise SystemExit('★ %s 에서 %s 로 가는 길이 없다' % (src, dst))
    seen_paths.sort()
    t_end, path = seen_paths[0]

    print()
    print('  %s번 -> %s번   현재 t=%.1f초' % (src, dst, t_now))
    print()
    print('  구간    묶음   기다림    건너기 시작   도착')
    prev = t_now
    total_wait = 0.0
    for leg, go, arr in path:
        fc = free_cost(leg, s)
        tag = '육교' if fc is not None else 'AB'[group_of(leg, s['groups'])]
        w = go - prev
        total_wait += w
        print('  %-6s  %-4s  %-8s  %8.1f초    %6.1f초'
              % (leg, tag, fmt(w) if w > 0.05 else '없음', go, arr))
        prev = arr
    print()
    print('  도착 t=%.1f초 — 지금부터 %s (신호 대기 %s)'
          % (t_end, fmt(t_end - t_now), fmt(total_wait)))

    # 더 늦게 떠나도 같은 시각에 닿는가? 그 여유는 다른 일에 쓸 수 있다.
    slack = 0.0
    for extra in range(1, int(float(s['period_s']) * len(s['groups']))):
        t2 = t_now + extra
        tt = t2
        for leg, _, _ in path:
            go = earliest_cross(s, leg, tt)
            fc = free_cost(leg, s)
            tt = go + (fc if fc is not None else cs)
        if tt <= t_end + 1e-6:
            slack = extra
        else:
            break
    if slack > 0:
        print('  ★ %.0f초 늦게 출발해도 도착시각은 같다. 그 시간은 다른 일에 써도 된다.'
              % slack)

    # 다른 길과 비교해 보여준다 — 왜 이 길인지 납득되게.
    others = [p for p in seen_paths[1:] if len(p[1]) == len(path)]
    if others:
        t2, p2 = others[0]
        print('  (다른 길 %s 는 t=%.1f초 — %s 느리다)'
              % (' '.join(l for l, _, _ in p2), t2, fmt(t2 - t_end)))
    return 0



def _leg_time(s, leg, t):
    go = earliest_cross(s, leg, t)
    fc = free_cost(leg, s)
    return go, go + (fc if fc is not None else float(s['cross_seconds']))


def _path_between(s, src, dst, t):
    """src 에서 dst 까지 시각 t 에 출발해 가장 빨리 닿는 (도착시각, 구간목록)."""
    best = {src: (t, [])}
    frontier = [(t, src, [])]
    out = []
    while frontier:
        frontier.sort()
        tt, z, path = frontier.pop(0)
        if z == dst:
            out.append((tt, path))
            continue
        if len(path) > len(ZONES):
            continue
        for nb in neighbors(z):
            leg = '%s-%s' % (z, nb)
            go, arr = _leg_time(s, leg, tt)
            if nb not in best or arr < best[nb][0] - 1e-9:
                best[nb] = (arr, path + [(leg, go, arr)])
                frontier.append((arr, nb, path + [(leg, go, arr)]))
    if not out:
        return None, None
    out.sort()
    return out[0]


def cmd_tour(a, d, s):
    """주어진 존들을 다 들르는 가장 빠른 순서. 첫 번째로 준 존이 출발점이다."""
    if len(a.leg) < 2:
        raise SystemExit('★ 출발 존과 들를 존들을 줄 것 (예: tour 1 2 3 4)')
    src = str(a.leg[0])
    rest = [str(x) for x in a.leg[1:]]
    for z in [src] + rest:
        if z not in ZONES:
            raise SystemExit('★ 존은 1~4 다: %s' % z)
    st = state(s)
    t0 = st['elapsed'] if st else 0.0
    if st is None:
        print('  ★ t0 가 없다 — 경기 시작(t=0, %s 초록)을 가정한다'
              % ', '.join(s['groups'][0]))

    import itertools
    best = None
    for order in itertools.permutations(rest):
        t = t0
        legs = []
        cur = src
        ok = True
        for nxt in order:
            arr, path = _path_between(s, cur, nxt, t)
            if arr is None:
                ok = False
                break
            legs += path
            t = arr
            cur = nxt
        if ok and (best is None or t < best[0] - 1e-9):
            best = (t, list(order), legs)
    if best is None:
        raise SystemExit('★ 다 들르는 길이 없다')
    t_end, order, legs = best

    print()
    print('  순서   %s' % ' -> '.join([src] + order))
    print()
    print('  구간    묶음  기다림    건너기 시작   도착')
    prev = t0
    wait = 0.0
    for leg, go, arr in legs:
        fc = free_cost(leg, s)
        tag = '육교' if fc is not None else 'AB'[group_of(leg, s['groups'])]
        w = go - prev
        wait += w
        print('  %-6s  %-4s %-8s  %8.1f초    %6.1f초'
              % (leg, tag, fmt(w) if w > 0.05 else '없음', go, arr))
        prev = arr
    print()
    print('  전부 도는 데 %s (신호 대기 %s)' % (fmt(t_end - t0), fmt(wait)))

    # 다른 순서들과 비교 — 왜 이 순서인지 보이게
    alts = []
    for o in itertools.permutations(rest):
        if list(o) == order:
            continue
        t = t0
        cur = src
        ok = True
        for nxt in o:
            arr, _ = _path_between(s, cur, nxt, t)
            if arr is None:
                ok = False
                break
            t = arr
            cur = nxt
        if ok:
            alts.append((t, list(o)))
    alts.sort()
    for t2, o2 in alts[:3]:
        print('    %s -> %s : %s  (%s 느림)'
              % (src, ' -> '.join(o2), fmt(t2 - t0), fmt(t2 - t_end)))
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('cmd', choices=['start', 'sync', 'now', 'plan', 'wait',
                                    'route', 'tour', 'pause', 'resume'])
    ap.add_argument('leg', nargs='*', default=[])
    ap.add_argument('--need', type=float, default=0.0,
                    help='이 구간을 건너는 데 필요한 초 (기본: cross_seconds + margin)')
    ap.add_argument('--only-start', action='store_true',
                    help='규정이 "초록에 출발만 하면 된다" 일 때')
    ap.add_argument('--allow-unknown', action='store_true',
                    help='t0 를 모를 때 막지 않고 통과시킨다')
    a = ap.parse_args()
    d, s = load()
    return {'start': cmd_start, 'sync': cmd_sync, 'now': cmd_now,
            'plan': cmd_plan, 'wait': cmd_wait, 'route': cmd_route,
            'tour': cmd_tour, 'pause': cmd_pause,
            'resume': cmd_resume}[a.cmd](a, d, s) or 0


if __name__ == '__main__':
    sys.exit(main())
