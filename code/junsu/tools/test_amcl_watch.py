#!/usr/bin/env python3
"""amcl_watch_core 시험(ROS 없음). python3 tools/test_amcl_watch.py
1) 규칙 시험(가짜 값)  2) 실제 재생 기록(review amcl3 o_*.npz) — 정상 주행에서 STOP 0번,
   FAST-LIO 에 틀어짐을 넣으면 STOP. (AMCL 은 기록 그대로 = 'AMCL 이 맞다' 가정 — 실기 반응은 ROS 재생으로 따로 확인)"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, 'start_align'))   # 출발 정렬 코어는 tools/start_align/ (review 4b0625b 복사본)
import amcl_watch_core as wc  # noqa: E402
import start_align_core as sa  # noqa: E402

AMCL3 = os.path.expanduser('~/ws/merge_tmp_20261102/amcl3')
MAP = os.path.join(HERE, '..', 'maps', 'align', 'arena1459_nocone.yaml')
fails = []


def check(name, cond, info=''):
    print(('PASS ' if cond else 'FAIL ') + name + (f'  ({info})' if info else ''))
    if not cond:
        fails.append(name)


def rules():
    T = (0.1, -0.05, math.radians(3))
    w = wc.Watch(T, t_init=0.0)
    truth = lambda lio: wc.compose(T, lio)
    lio = (1.0, 2.0, 0.5)
    check('준비 중엔 판정 안 함', w.update(1.0, lio, truth(lio), 0.05) == 'WARMUP')
    check('일치 → OK', w.update(4.0, lio, truth(lio), 0.05) == 'OK')
    a = truth(lio); off = (a[0] + 0.2, a[1], a[2])
    check('0.2 m → WARN', w.update(4.1, lio, off, 0.05) == 'WARN')
    big = (a[0] + 0.4, a[1], a[2])
    check('0.4 m 0.5초 → 아직 WARN', w.update(5.0, lio, big, 0.05) == 'WARN' and w.update(5.5, lio, big, 0.05) == 'WARN')
    check('0.4 m 1초 → STOP', w.update(6.0, lio, big, 0.05) == 'STOP')
    check('STOP 유지(다시 맞아도)', w.update(6.5, lio, truth(lio), 0.05) == 'STOP')
    w.reset(t_init=7.0)
    check('reset → WARMUP', w.update(7.5, lio, truth(lio), 0.05) == 'WARMUP')
    check('방향 9° 1초 → STOP', [w.update(t, lio, (a[0], a[1], a[2] + math.radians(9)), 0.05) for t in (11.0, 12.0)][-1] == 'STOP')
    w.reset(t_init=20.0)
    check('AMCL 불확실 → UNSURE(안 멈춤)', [w.update(t, lio, big, 0.4) for t in (24.0, 26.0)][-1] == 'UNSURE')
    check('AMCL 끊김 → NO_AMCL(안 멈춤)', w.judge(30.0) == 'NO_AMCL')
    w2 = wc.Watch(T, t_init=0.0)
    check('AMCL 한 번도 안 옴 → WARMUP', w2.judge(10.0) == 'WARMUP')


def align_T(d, wm):
    lio, st, sr = d['lio'], d['scan_t'], d['scan_r']
    am, ai = float(d['scan_meta'][0]), float(d['scan_meta'][1])
    t0 = lio[0, 0]; pairs = []
    for t, r in zip(st, sr):
        if t0 + 1.0 <= t <= t0 + 4.0:
            k = int(np.argmin(abs(lio[:, 0] - t)))
            if abs(lio[k, 0] - t) <= 0.1:
                pairs.append((sa.scan_points(r, am, ai), tuple(lio[k, 1:4])))
    return sa.align(wm, pairs)


def run(d, T, inject=None):
    """inject(t_rel, lio_pose) -> lio_pose. 반환: (상태 개수, 첫 STOP 시각)"""
    lio, a = d['lio'], d['a']; t0 = lio[0, 0]
    w = wc.Watch(T, t_init=a[0, 0])
    counts, first_stop, emax = {}, None, 0.0
    for row in a:
        t = row[0]; k = int(np.argmin(abs(lio[:, 0] - t)))
        if abs(lio[k, 0] - t) > 0.1:
            continue
        lp = tuple(lio[k, 1:4])
        if inject:
            lp = inject(t - t0, lp)
        s = w.update(t, lp, (row[1], row[2], row[3]), math.sqrt(max(row[4], 0.0)))
        counts[s] = counts.get(s, 0) + 1
        if s == 'STOP' and first_stop is None:
            first_stop = t - t0
        if s in ('OK', 'WARN') and w.e_xy is not None:
            emax = max(emax, w.e_xy)
    return counts, first_stop, emax


def replays():
    if not os.path.isdir(AMCL3):
        print('SKIP 재생 기록 없음', AMCL3); return
    wm = sa.WallMap(MAP)
    for tag in ('o_arena4', 'o_113612', 'o_145915'):
        f = os.path.join(AMCL3, tag + '.npz')
        if not os.path.exists(f):
            print('SKIP', f); continue
        d = np.load(f, allow_pickle=True)
        res = align_T(d, wm)
        check(f'{tag} 출발 정렬 OK', res['ok'], f"T {res['T'][0]*100:+.1f} {res['T'][1]*100:+.1f} cm {math.degrees(res['T'][2]):+.2f}°, match {res['match']*100:.0f}%")
        T = res['T']
        c, fs, emax = run(d, T)
        check(f'{tag} 정상 주행 STOP 0번', fs is None, f'{c}, 판정 중 최대 {emax*100:.1f} cm')
        # 주입 시각 = 움직이는 구간(AMCL 갱신이 촘촘한 곳). 기록 재생엔 nomotion 요청이 없어 서 있으면 AMCL 이 안 나온다
        #   (113612: 4~58 s 서 있음 → 53 s 공백). 라이브 amcl_watch.py 는 0.5 s 넘게 없으면 nomotion 갱신을 요청한다.
        ta = d['a'][:, 0] - d['lio'][0, 0]; gap = np.r_[np.diff(ta), 9.0]
        dense = [t for t, g in zip(ta, gap) if t > 20.0 and g < 0.6]
        t_f = float(dense[len(dense) // 3])
        # 서서히 틀어짐: t_f 부터 0.10 m/s 로 x 가 흐름 → 0.30 m 는 3 s 뒤, +1 s 지속 → 4~5 s 안에 STOP
        c, fs, _ = run(d, T, lambda tr, p: (p[0] + 0.10 * max(0.0, tr - t_f), p[1], p[2]))
        check(f'{tag} 흐름 0.1 m/s 주입 → STOP', fs is not None and fs - t_f <= 5.5, f'주입 {t_f:.0f}s, STOP {fs if fs is None else round(fs, 1)}s')
        # 한 번에 튐: 0.5 m
        c, fs, _ = run(d, T, lambda tr, p: (p[0], p[1] + (0.5 if tr >= t_f else 0.0), p[2]))
        check(f'{tag} 0.5 m 튐 주입 → STOP', fs is not None and fs - t_f <= 2.0, f'주입 {t_f:.0f}s, STOP {fs if fs is None else round(fs, 1)}s')
        # 방향 10° 튐
        c, fs, _ = run(d, T, lambda tr, p: (p[0], p[1], p[2] + (math.radians(10) if tr >= t_f else 0.0)))
        check(f'{tag} 방향 10° 튐 주입 → STOP', fs is not None and fs - t_f <= 2.0, f'STOP {fs if fs is None else round(fs, 1)}s')
        # 정렬 안 하고(T=0) 쓰면? — 정렬이 왜 필요한지 보여주는 대조군
        c, fs, emax = run(d, (0.0, 0.0, 0.0))
        print(f'     (대조) 정렬 없이 T=0: STOP {fs if fs is None else round(fs, 1)}s, {c}')


if __name__ == '__main__':
    rules()
    replays()
    print('ALL PASS' if not fails else f'FAIL {len(fails)}: {fails}')
    sys.exit(1 if fails else 0)
