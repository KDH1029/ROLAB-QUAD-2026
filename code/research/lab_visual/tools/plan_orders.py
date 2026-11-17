#!/usr/bin/env python3
"""대회 당일 주문서로 팀 알고리즘을 돌려 배달 계획을 낸다.

팀 원본(gangnam_dispatch/algorithm.py)은 **읽기 전용으로 import 만** 한다.
바꾸는 것은 ORDER_SHEETS 하나뿐이고, 그것도 메모리에서만 갈아끼운다.
파일은 건드리지 않는다.

  python3 plan_orders.py                      계획 표
  python3 plan_orders.py --json 계획.json      실행기가 읽을 형식으로 저장

【표 읽는 법 — 2026-09-13 확인】
표의 장소는 **목적지**다. 픽업 장소가 아니다.
품목에서 어느 가게(R1~R3)를 들러야 하는지가 정해진다.
  R1 도미노피자  0 슈퍼슈프림피자 / 1 치즈피자
  R2 한솥도시락  0 스팸김치도시락 / 1 치킨마요도시락
  R3 이디야커피  0 아메리카노(아이스 포함) / 1 카페라떼
  D1 코엑스  D2 휘문고등학교  D3 강남경찰서  D4 GTX 공사현장
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.expanduser('~/ws/src/gangnam_dispatch'))
from gangnam_dispatch import algorithm            # noqa: E402  원본 그대로
from gangnam_dispatch.drivers import zone_hop_path  # noqa: E402

MENU = {
    '슈퍼슈프림피자': ('R1', 0), '치즈피자': ('R1', 1),
    '스팸김치도시락': ('R2', 0), '치킨마요도시락': ('R2', 1),
    '아메리카노': ('R3', 0), '아이스아메리카노': ('R3', 0), '카페라떼': ('R3', 1),
}
DEST = {'코엑스': 'D1', '휘문고등학교': 'D2', '강남경찰서': 'D3', 'GTX 공사현장': 'D4'}

# 2026-09-13 대회 당일 주문서. 표의 장소 = **목적지**.
TABLE = [
    (1, '하', '강남경찰서',   [('치즈피자', 2), ('치킨마요도시락', 1)]),
    (2, '하', 'GTX 공사현장', [('아메리카노', 1), ('스팸김치도시락', 1), ('치킨마요도시락', 1)]),
    (3, '하', '휘문고등학교', [('슈퍼슈프림피자', 1), ('아메리카노', 1), ('카페라떼', 1)]),
    (4, '중', 'GTX 공사현장', [('치즈피자', 1), ('아메리카노', 1), ('스팸김치도시락', 2)]),
    (5, '중', '휘문고등학교', [('슈퍼슈프림피자', 1), ('치즈피자', 1), ('아메리카노', 2)]),
    (6, '중', 'GTX 공사현장', [('슈퍼슈프림피자', 2), ('스팸김치도시락', 2)]),
    (7, '상', 'GTX 공사현장', [('슈퍼슈프림피자', 2), ('아메리카노', 2), ('치킨마요도시락', 1)]),
    (8, '상', 'GTX 공사현장', [('치즈피자', 1), ('아메리카노', 2), ('스팸김치도시락', 2)]),
]


def build_sheets():
    out = []
    for oid, tier, dest, items in TABLE:
        if dest not in DEST:
            raise SystemExit('★ 모르는 목적지: %s' % dest)
        row = {'id': oid, 'tier': tier, 'dest': DEST[dest], 'items': []}
        for name, qty in items:
            if name not in MENU:
                raise SystemExit('★ 모르는 품목: %s' % name)
            r, mi = MENU[name]
            row['items'].append({'rest': r, 'menuIdx': mi, 'qty': qty})
        out.append(row)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json', default='')
    ap.add_argument('--start', default='Z2', help='출발 존 (알고리즘 번호). 기본 Z2=휘문고')
    ap.add_argument('--dwell', type=float, default=None,
                    help='한 지점에서 앉아 있는 시간(초). 기본은 알고리즘 값 5. 실제는 7')
    ap.add_argument('--interval', type=float, default=None,
                    help='음식이 하나 나오는 주기(초). 규정 20')
    ap.add_argument('--overhead', type=float, default=20.0,
                    help='정차 leg 하나당 실제로 더 드는 시간(초). '
                         '도킹 10~15 + 보행전환 3 + 기립 2.5. 모델에는 이게 없다')
    ap.add_argument('--budget', type=float, default=0.0,
                    help='경기 시간(초). 규정: 예선 480, 본선 720. '
                         '넘어가는 leg 은 계획에서 잘라낸다')
    ap.add_argument('--signals', action='store_true',
                    help='신호 있는 횡단보도를 모델에 넣는다 (기본은 팀 원본대로 전부 free)')
    ap.add_argument('--no-diag', action='store_true',
                    help='대각선 엣지(Z2-Z4 등)를 뺀다')
    ap.add_argument('--edge-scale', type=float, default=1.0,
                    help='구간 이동시간 배율. 도킹까지 포함한 실제가 모델보다 길면 올린다')
    a = ap.parse_args()

    # ── 시간 모델을 실제에 맞춘다 (메모리에서만) ───────────────────────
    # 규정: 음식은 20초마다 하나씩 나온다 -> INTERVAL.
    # 그 값이 "도착했을 때 몇 개가 나와 있나"(ready_count)를 정하므로,
    # 이동·정차 시간이 모델보다 길면 실제로는 **더 많이** 나와 있게 된다.
    # 모델과 실제가 어긋나면 알고리즘이 덜 싣고 헛걸음을 한다.
    if a.dwell is not None:
        algorithm.DWELL_R = algorithm.DWELL_D = a.dwell
    if a.interval is not None:
        algorithm.INTERVAL = a.interval
    if a.no_diag:
        # 사각 광장에서 1-3, 2-4 는 대각선이다. 팀 원본에는 Z2-Z4 가
        # 'free, 8초, 육교' 로 들어 있는데, 현장에서 대각선 이동이 불가하면 빼야 한다.
        before = len(algorithm.EDGES)
        DIAG = {('Z1', 'Z3'), ('Z3', 'Z1'), ('Z2', 'Z4'), ('Z4', 'Z2')}
        algorithm.EDGES = [e for e in algorithm.EDGES
                           if (e['u'], e['v']) not in DIAG]
        print('대각선 엣지 %d개 제거' % (before - len(algorithm.EDGES)))
    if a.signals:
        # 현장: 육교는 3-4 하나뿐. 1-2, 2-3, 4-1 은 신호 있는 횡단보도다.
        #   묶음 A(경기 시작 초록) = 1-2, 3-4   -> pair 1
        #   묶음 B                 = 2-3, 4-1   -> pair 2
        # algorithm 의 crosswalk_finish 가 30초 주기와 3초 유예를 계산한다.
        PAIR = {('Z1', 'Z2'): 1, ('Z2', 'Z3'): 2, ('Z4', 'Z1'): 2}
        n_sig = 0
        for e in algorithm.EDGES:
            key = (e['u'], e['v'])
            rkey = (e['v'], e['u'])
            pr = PAIR.get(key) or PAIR.get(rkey)
            if pr:
                e['type'] = 'crosswalk'
                e['pair'] = pr
                n_sig += 1
        print('신호 구간 %d개 설정 (묶음1=1-2, 묶음2=2-3·4-1, 3-4 는 육교라 free)' % n_sig)
    if a.edge_scale != 1.0:
        for e in algorithm.EDGES:
            e['time'] = e['time'] * a.edge_scale
    print('시간 모델: 정차 %.0f초, 생산주기 %.0f초, 이동 x%.2f'
          % (algorithm.DWELL_R, algorithm.INTERVAL, a.edge_scale))

    algorithm.ORDER_SHEETS = build_sheets()      # ★ 메모리에서만 갈아끼운다
    print('주문서 %d건 (대회 당일 표)' % len(algorithm.ORDER_SHEETS))
    for s in algorithm.ORDER_SHEETS:
        items = ' '.join('%s%s x%d' % (i['rest'], i['menuIdx'], i['qty']) for i in s['items'])
        print('  %d  %s  ->%s   %s' % (s['id'], s['tier'], s['dest'], items))

    # gen_mission_commands.py 의 PlanDriver 를 그대로 재사용한다 — 베끼지 않는다.
    sys.path.insert(0, os.path.expanduser('~/go2_ws/gangnam_shared'))
    import gen_mission_commands as G
    G.algorithm.ORDER_SHEETS = algorithm.ORDER_SHEETS
    start_zone = int(a.start[1:])
    # ── 시간 도장을 찍는 PlanDriver ──────────────────────────────────
    # 원본 PlanDriver 는 fake_time 을 들고 있지만 leg 마다 기록하지 않는다.
    # 그 값을 leg 에 찍어두면 "몇 분까지 몇 leg 을 할 수 있나"를 잘라낼 수 있다.
    class TimedDriver(G.PlanDriver):
        def call_py(self, kind, args):
            r = super().call_py(kind, args)
            if self.steps:
                self.steps[-1].setdefault('t_arrive', self.fake_time)
            return r

        def sleep(self, sec):
            super().sleep(sec)
            if self.steps:
                self.steps[-1]['t_end'] = self.fake_time

        def advance(self, sec):
            super().advance(sec)
            if self.steps:
                self.steps[-1]['t_end'] = self.fake_time

    driver = TimedDriver(start_zone=start_zone)
    rc, state, total = G.algorithm.run_dispatch(driver, start_node=a.start)
    legs = driver.steps
    for l in legs:
        l.setdefault('t_arrive', 0.0)
        l.setdefault('t_end', l['t_arrive'])
    # ── 모델에 없는 실제 오버헤드를 얹는다 ──────────────────────────
    # algorithm 의 시간 모델은 이동 + 정차(dwell)만 센다. 실제로는 존마다
    #   정밀 도킹 10~15초 + 보행 전환 3초 + 기립 2.5초 가 더 든다.
    # 그걸 안 넣으면 "22 leg 이 12분에 들어간다"는 낙관이 나온다.
    if a.overhead > 0:
        acc = 0.0
        for l in legs:
            if l.get('sit_down'):
                acc += a.overhead
            l['t_arrive'] += acc - (a.overhead if l.get('sit_down') else 0)
            l['t_end'] += acc
    if a.budget > 0:
        keep = [l for l in legs if l['t_end'] <= a.budget]
        cut = len(legs) - len(keep)
        if cut:
            print()
            print('★ 경기 시간 %.0f초 안에 들어가는 leg 은 %d개다. 뒤의 %d개는 잘라낸다.'
                  % (a.budget, len(keep), cut))
            print('   (모델 시간이다. 실제는 도킹·앉기까지 더 걸리므로 더 적게 된다)')
            legs = keep

    print()
    print('출발 %s,  총 %d leg,  rc=%s' % (a.start, len(legs), rc))
    print('배달현황: ' + ', '.join(
        '#%s %s/%s' % (sh['id'], state['deliveredPerSheet'][sh['id']], total[sh['id']])
        for sh in algorithm.ORDER_SHEETS))
    print('  leg  노드  존  동작       모델시각   hops')
    for l in legs:
        print('  [%2d] %-4s   %s  %-8s %5.0f~%5.0f초  %s'
              % (l['leg'], l['raw_node'], l['algo_zone_num'], l['action'],
                 l['t_arrive'], l['t_end'], l['hops']))
    if legs:
        print('  모델상 전체 %.0f초 = %.1f분' % (legs[-1]['t_end'], legs[-1]['t_end'] / 60))
    if a.json:
        p = os.path.expanduser(a.json)
        os.makedirs(os.path.dirname(os.path.abspath(p)), exist_ok=True)
        json.dump(legs, open(p, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
        print('저장 %s' % p)
    return 0


if __name__ == '__main__':
    sys.exit(main())
