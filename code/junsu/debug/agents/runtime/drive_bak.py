#!/usr/bin/env python3
"""junsu_v2_bak(16:54) 관리자 ui 를 같은 하네스로 — s 를 눌렀을 때 화면 폭별로 튕기는지만 본다(되돌리기 준비 점검)."""
import os, sys, json
os.environ['HARNESS_CTL'] = os.path.expanduser('~/ws/junsu_v2_bak/tools/go2_ctl.py')
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import drive_ui as D
D.SEQ = [('', 1.0), ('s', 0.4), ('z\r', 0.4), ('', 0.5), ('m', 0.3), ('n\r', 0.3), ('R', 0.3), ('\r', 0.4), ('Q', 0.3), ('n\r', 0.3), ('l', 0.3), ('\r', 0.3), ('q', 1.0)]
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ui_runs_bak'); D.OUT = out; os.makedirs(out, exist_ok=True)
for rows, cols in [(24, 80), (30, 120), (45, 184), (45, 220), (50, 260)]:
    r = D.run(rows, cols, 'normal')
    print(f"bak {cols}x{rows}: 정상종료={r['exit_ok']} 튕김={len(r['crash'])}")
    for c in r['crash']: print('   ', c.strip().splitlines()[-1], '|', [l for l in c.splitlines() if 'line' in l][-2:])
