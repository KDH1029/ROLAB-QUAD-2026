#!/usr/bin/env python3
"""pty(pexpect) 로 ui_harness.py 를 화면 크기별로 띄우고 모든 키·s 하위 메뉴를 눌러 본다. 튕김·넘침·바쁜 루프를 집계.
사용: python3 drive_ui.py [결과폴더]   (HARNESS_CTL=<고친 go2_ctl.py> 로 패치본 시험)"""
import json, os, sys, time
import pexpect
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, 'ui_runs')
os.makedirs(OUT, exist_ok=True)
ENTER = '\r'

def S(choice, *more):           # s → 하위 메뉴 → (추가 입력들)
    return [('s', 0.3), (choice + ENTER, 0.4)] + [(m + ENTER, 0.4) for m in more]

SEQ = [('', 2.5)]               # 처음 2.5초: ask 전 루프 속도
SEQ += [('a', 0.4), ('r', 0.4), ('R', 0.3), (ENTER, 0.4)]   # ← 여기서 첫 ask
SEQ += [('', 2.5)]              # ask 뒤 루프 속도
SEQ += [('R', 0.3), ('5' + ENTER, 0.4), ('R', 0.3), ('abc' + ENTER, 0.4), ('R', 0.3), ('99' + ENTER, 0.4),
        ('p', 0.3), ('x', 0.3), (' ', 0.3), ('m', 0.3), ('n' + ENTER, 0.3), ('Q', 0.3), ('n' + ENTER, 0.3),
        ('M', 0.3), ('1', 0.2), ('3', 0.2), ('8', 0.2), ('9', 0.2), ('0', 0.2),
        ('l', 0.3), ('runner' + ENTER, 1.0), (ENTER, 0.6), ('l', 0.3), ('없는부품' + ENTER, 0.4), ('l', 0.3), (ENTER, 0.4)]
SEQ += S('1', '0') + S('1', '') + S('1', '99') + S('2', '0.25') + S('2', '') + S('2', 'abc') + S('3', '0.2') + S('4', '0.5')
SEQ += S('5', '1.0') + S('6', '10') + S('7') + S('7') + S('7') + S('8') + S('9') + S('2', '0.3') + S('9') + S('0') + S('0')
SEQ += S('L') + S('L') + S('G', 'n') + S('G', 'n') + S('G', 'n') + S('K') + S('K') + S('T') + S('T') + S('V') + S('V')
SEQ += S('F') + S('F') + S('z') + S('') + S('l') + S('l') + S('g', 'n') + S('g', 'n') + S('g', 'n') + S('k') + S('k') + S('t') + S('t') + S('v') + S('v') + S('f') + S('f')
SEQ += S('긴입력' * 15) + [('ㄴ', 0.3), ('ㅁ', 0.3), ('\x1b[A', 0.3)]
SEQ += [('RESIZE', 0.8), ('s', 0.3), ('1' + ENTER, 0.3), (ENTER, 0.4), ('RESTORE', 0.8), ('', 1.0), ('q', 1.0)]

def run(rows, cols, var='normal', lang='en_US.UTF-8'):
    name = f'{var}_{cols}x{rows}'
    logf = os.path.join(OUT, name + '.jsonl')
    if os.path.exists(logf): os.remove(logf)
    env = dict(os.environ, TERM='xterm-256color', LANG=lang, LC_ALL='')
    env.pop('LC_ALL')
    ch = pexpect.spawn(sys.executable, [os.path.join(HERE, 'ui_harness.py'), logf, var], dimensions=(rows, cols), env=env,
                       encoding='utf-8', codec_errors='ignore', timeout=5)
    marks = []
    try:
        for key, wait in SEQ:
            if not ch.isalive(): break
            if key == 'RESIZE':
                ch.setwinsize(max(8, rows // 2), max(20, cols // 3)); marks.append(('resize', time.monotonic()))
            elif key == 'RESTORE':
                ch.setwinsize(rows, cols)
            elif key:
                ch.send(key)
                if key == 'R' and not marks: marks.append(('first_ask', time.monotonic()))
            t0 = time.monotonic()
            while time.monotonic() - t0 < wait:
                try: ch.read_nonblocking(65536, timeout=0.05)
                except pexpect.TIMEOUT: pass
                except pexpect.EOF: break
        ch.expect(pexpect.EOF, timeout=8)
    except Exception as e:
        marks.append(('driver_exc', repr(e)))
    ch.close()
    evs = [json.loads(l) for l in open(logf)] if os.path.exists(logf) else []
    crash = [e for e in evs if e['ev'] == 'CRASH']
    err = [e for e in evs if e['ev'].endswith('_error')]
    ovf = [e for e in evs if e['ev'] == 'addstr_overflow']
    rates = [(e['t'], e['loops_per_s']) for e in evs if e['ev'] == 'rate']
    fa = next((t for m, t in marks if m == 'first_ask'), None)
    before = [r for t, r in rates if fa and t < fa]
    after = [r for t, r in rates if fa and fa + 2 < t < fa + 5]
    ok = any(e['ev'] == 'exit_ok' for e in evs)
    res = dict(name=name, exit=ch.exitstatus, signal=ch.signalstatus, exit_ok=ok, crash=[c['tb'][-600:] for c in crash],
               addstr_error=len(err), err_samples=err[:3], overflow=len(ovf), ovf_samples=ovf[:2],
               loops_before_ask=before, loops_after_ask=after, keys=sum(1 for e in evs if e['ev'] == 'key'),
               getstr=[e['args'] for e in evs if e['ev'] == 'getstr'][:3], marks=[m for m in marks if m[0] == 'driver_exc'])
    return res

if __name__ == '__main__':
    sizes = [(24, 60), (24, 80), (30, 120), (45, 184), (12, 40)]
    results = []
    for rows, cols in sizes:
        results.append(run(rows, cols))
    results.append(run(24, 80, 'bigdata')); results.append(run(24, 60, 'nounitree')); results.append(run(24, 80, 'normal', 'C.UTF-8'))
    json.dump(results, open(os.path.join(OUT, 'summary.json'), 'w'), ensure_ascii=False, indent=1)
    for r in results:
        print(f"{r['name']:24s} exit={r['exit']} sig={r['signal']} 정상종료={r['exit_ok']} 튕김={len(r['crash'])} curses오류={r['addstr_error']} "
              f"넘침={r['overflow']} 키={r['keys']} 루프/초 ask전={r['loops_before_ask']} ask뒤={r['loops_after_ask']}")
        for c in r['crash']: print('   CRASH:', c)
        for e in r['err_samples']: print('   err:', e)
        if r['marks']: print('   driver:', r['marks'])
