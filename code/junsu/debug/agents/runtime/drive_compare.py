#!/usr/bin/env python3
"""지금 v2(HEAD) 관리자와 runtime 패치본을 같은 키 순서로 — 튕김 0·ask 뒤 루프 속도 비교."""
import os, sys, json
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import drive_ui as D
res = []
for tag, ctl in [('HEAD', None), ('patched', os.path.join(HERE, 'go2_ctl.patched.py'))]:
    if ctl:
        os.environ['HARNESS_CTL'] = ctl; os.environ['HARNESS_CTL_AS'] = os.path.expanduser('~/ws/junsu_v2/tools/go2_ctl.py')
    else:
        os.environ.pop('HARNESS_CTL', None); os.environ.pop('HARNESS_CTL_AS', None)
    D.OUT = os.path.join(HERE, 'ui_runs_' + tag); os.makedirs(D.OUT, exist_ok=True)
    for rows, cols in [(24, 60), (24, 80), (30, 120), (45, 184), (12, 40)]:
        r = D.run(rows, cols); r['tag'] = tag; res.append(r)
        print(f"{tag:8s} {cols}x{rows}: 정상종료={r['exit_ok']} 튕김={len(r['crash'])} curses오류={r['addstr_error']} 키={r['keys']} 루프/초 ask전={r['loops_before_ask']} ask뒤={r['loops_after_ask']}", flush=True)
        for c in r['crash']: print('   CRASH', c[-300:])
    r = D.run(24, 80, 'bigdata'); print(f"{tag:8s} bigdata 80x24: 정상종료={r['exit_ok']} 튕김={len(r['crash'])}", flush=True); res.append(r)
json.dump(res, open(os.path.join(HERE, 'ui_compare.json'), 'w'), ensure_ascii=False, indent=1)
