#!/usr/bin/env python3
"""러너 로그 → 구간별 시간표(실제 16:54 · 시뮬 공통). simcal 11-06.
사용: timeline.py real                       → 실제 16:54 표
      timeline.py <시뮬 폴더> [...]           → 시뮬 표(+ world.jsonl 있으면 구간별 머뭇·물러남·콘무시·회전)
      timeline.py --cmp <폴더> [...]         → 실제와 나란히(구간별 시간·차이) + 합계 오차 %
      timeline.py --json <폴더> [...]        → 기계용
구간 정의: travel(L) = 도착(L) − 출발(앞 점). 출발 = 정지선이면 녹색 확인, 회전점이면 회전 끝(완료/타임아웃), 주차면 일어남 확인, 아니면 도착 즉시.
"""
import json, os, re, sys
import numpy as np

REAL_LOG = os.path.expanduser('~/ws/junsu/maps/segments/logs/ctl_runner.log')
REAL_LINES = (972, 1103)   # 16:54:34 러너 시작 ~ 그 판 끝(1104 = 18:04 판 시작)
LABELS = ['line1', 'stopline1', 'line2', 'rabain', 'rabaout', 'line3', 'stopline2', 'line4', 'line5', 'line6', 'parking', 'line7', 'line8', 'exit']
TS = re.compile(r'^\[[A-Z]+\] \[(\d+\.\d+)\] \[real_course_runner\]: (.*)$')


def read_lines(path, rng=None):
    L = open(path, errors='replace').read().splitlines()
    if rng:
        L = L[rng[0] - 1:rng[1]]
    out = []
    for l in L:
        m = TS.match(l)
        if m:
            out.append((float(m.group(1)), m.group(2)))
    return out


def timeline(ev):
    t0 = next(t for t, m in ev if '코스 시작' in m)
    arr, dep, rot, wait, extra = {}, {}, {}, {}, {}
    cur_rot = None; stop_t = None; cur = None
    cnt = {k: {'backoff': 0, 'follow': 0, 'conestep': 0} for k in LABELS}
    for t, m in ev:
        t = t - t0
        lab = re.match(r'\[(\w+)\]', m); lab = lab.group(1) if lab else None
        if lab in LABELS:
            cur = lab
        if lab in LABELS and re.match(r'\[\w+\] (도착$|도착 ->|경로 따라 도착)', m) and lab not in arr:
            arr[lab] = t; dep[lab] = t
        if '제자리 회전 시작' in m:
            cur_rot = (lab, t)
        if ('회전 완료' in m or '회전 타임아웃' in m) and cur_rot:
            e = re.search(r'남은 오차 (-?\d+)', m)
            rot[cur_rot[0]] = (t - cur_rot[1], 'timeout' if '타임아웃' in m else 'ok', int(e.group(1)) if e else 0)
            dep[cur_rot[0]] = t; cur_rot = None
        if '정지선 진입' in m:
            stop_t = (lab, t)
        if ('녹색 신호' in m or '타임아웃, 신호 무관' in m) and stop_t:
            wait[stop_t[0]] = t - stop_t[1]; dep[stop_t[0]] = t; stop_t = None
        if '일어남 확인' in m:
            dep['parking'] = t
        if '착석 확인' in m or 'Sit' in m and '정지' in m:
            extra.setdefault('park_t0', t)
        if 'Out Line 통과' in m or '완주' in m:
            arr['exit'] = extra.get('exit_arr', arr.get('exit', t)); extra['finish'] = t; dep['exit'] = t
        if lab == 'exit' and '도착했지만' in m:
            extra['exit_arr'] = t; arr['exit'] = t
        if cur in cnt:
            if '물러남' in m: cnt[cur]['backoff'] += 1
            if '경로 따라가기(' in m: cnt[cur]['follow'] += 1
            if re.search(r'콘 피하기 \d → [12] 단계', m): cnt[cur]['conestep'] += 1
    rows = []; prev_dep = 0.0
    for k in LABELS:
        if k not in arr:
            rows.append(dict(label=k, travel=None)); continue
        r = dict(label=k, arr=arr[k], travel=arr[k] - prev_dep, rot=rot.get(k), wait=wait.get(k), dep=dep.get(k, arr[k]), **cnt[k])
        if k == 'parking' and 'parking' in dep and k in rot:
            r['park'] = dep['parking'] - (arr[k] + rot[k][0])
        if k == 'exit':
            r['push'] = extra.get('finish', arr[k]) - arr[k]
        rows.append(r); prev_dep = r['dep']
    total = extra.get('finish', None)
    return rows, total


def world_seg(folder, rows, t0_runner_wall):
    """world.jsonl 의 러너 상태로 구간별 머뭇(TO_GOAL 인데 v<0.03)·물러남·콘무시 시간."""
    p = os.path.join(folder, 'world.jsonl')
    if not os.path.exists(p):
        return None
    E = [json.loads(l) for l in open(p) if '"pose"' in l]
    t = np.array([e['t'] for e in E]); rs = [e.get('rs', '') for e in E]; v = np.array([e['v'] for e in E])
    st = [i for i, s in enumerate(rs) if s and s != 'START_CHECK']
    if not st:
        return None
    ts = t[st[0]]
    out = {}; prev = 0.0
    for r in rows:
        if r.get('travel') is None:
            continue
        a, b = ts + prev, ts + r['arr']
        m = (t >= a) & (t < b)
        out[r['label']] = dict(hes=float(np.sum(m & (v < 0.03) & np.array([s == 'TO_GOAL' for s in rs])) * 0.1),
                               back=float(np.sum(m & np.array([s == 'BACKOFF' for s in rs])) * 0.1),
                               follow=float(np.sum(m & np.array([s == 'ROUTE_FOLLOW' for s in rs])) * 0.1))
        prev = r['dep']
    return out


def load(src):
    if src == 'real':
        return timeline(read_lines(REAL_LOG, REAL_LINES))
    return timeline(read_lines(os.path.join(src, 'runner.log')))


def fmt(v, w=6):
    return f'{v:{w}.1f}' if isinstance(v, (int, float)) else ' ' * (w - 1) + '-'


def sums(rows, total):
    tr = sum(r['travel'] for r in rows if r.get('travel') is not None)
    rot = sum(r['rot'][0] for r in rows if r.get('rot'))
    wt = sum(r['wait'] for r in rows if r.get('wait'))
    return dict(total=total, travel=tr, rot=rot, wait=wt, nto=sum(1 for r in rows if r.get('rot') and r['rot'][1] == 'timeout'),
                done=sum(1 for r in rows if r.get('travel') is not None))


def main():
    a = sys.argv[1:]
    mode = 'tab'
    if a and a[0] in ('--cmp', '--json'):
        mode = a.pop(0)
    if not a:
        a = ['real']
    real, rtot = load('real')
    if mode == '--json':
        out = {}
        for s in a:
            rows, tot = load(s); out[s] = dict(rows=rows, sum=sums(rows, tot))
        print(json.dumps(out, ensure_ascii=False, indent=1)); return
    for s in a:
        rows, tot = load(s)
        ws = world_seg(s, rows, 0) if s != 'real' else None
        print(f'### {s}  완주 {fmt(tot)} 초')
        if mode == 'tab':
            print(f"{'구간':10s} {'이동':>6s} {'회전':>6s} {'결과':>9s} {'대기':>6s} 물러남 콘무시")
            for r in rows:
                if r.get('travel') is None:
                    print(f"{r['label']:10s}  (도착 못 함)"); continue
                ro = r.get('rot'); rr = f"{ro[1]}{ro[2]:+d}" if ro else ''
                print(f"{r['label']:10s} {fmt(r['travel'])} {fmt(ro[0] if ro else None)} {rr:>9s} {fmt(r.get('wait'))} {r['backoff']:5d} {r['follow']:5d}")
        else:
            rd = {r['label']: r for r in real}
            print(f"{'구간':10s} {'실제':>6s} {'시뮬':>6s} {'차':>6s} | {'회전실':>6s} {'회전심':>6s} {'남은°':>5s} | 대기실 대기심 | 물러남 콘무시 {'머뭇':>5s}")
            for r in rows:
                R = rd[r['label']]
                if r.get('travel') is None:
                    print(f"{r['label']:10s} {fmt(R['travel'])}  (도착 못 함)"); continue
                ro, Ro = r.get('rot'), R.get('rot'); w = (ws or {}).get(r['label'], {})
                print(f"{r['label']:10s} {fmt(R['travel'])} {fmt(r['travel'])} {fmt(r['travel'] - R['travel'])} | {fmt(Ro[0] if Ro else None)} {fmt(ro[0] if ro else None)} "
                      f"{(str(ro[2]) if ro and ro[1] == 'timeout' else ('ok' if ro else '')):>5s} | {fmt(R.get('wait'))} {fmt(r.get('wait'))} | {r['backoff']:5d} {r['follow']:5d} {fmt(w.get('hes'), 5)}")
            S, RS = sums(rows, tot), sums(real, rtot)
            pct = lambda x, y: f'{100 * (x - y) / y:+.0f}%' if (x is not None and y) else '-'
            print(f"합계  완주 {fmt(RS['total'])} → {fmt(S['total'])} ({pct(S['total'], RS['total'])}) · 이동 {RS['travel']:.1f} → {S['travel']:.1f} ({pct(S['travel'], RS['travel'])})"
                  f" · 회전 {RS['rot']:.1f} → {S['rot']:.1f} (타임아웃 {RS['nto']} → {S['nto']}) · 신호대기 {RS['wait']:.1f} → {S['wait']:.1f}")
            # 구간별 이동 시간 오차(정지선 대기·회전 뺀 순수 이동): 평균 절대 오차
            errs = [abs(r['travel'] - rd[r['label']]['travel']) for r in rows if r.get('travel') is not None]
            print(f"구간 이동 평균 절대 오차 {np.mean(errs):.1f} 초 · 중앙 {np.median(errs):.1f} 초 · 최대 {max(errs):.1f} 초({rows[int(np.argmax([abs(r['travel'] - rd[r['label']]['travel']) if r.get('travel') is not None else -1 for r in rows]))]['label']})")
        print()


if __name__ == '__main__':
    main()
