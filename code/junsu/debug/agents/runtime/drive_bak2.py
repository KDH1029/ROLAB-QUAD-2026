import os, sys
os.environ['HARNESS_CTL'] = os.path.expanduser('~/ws/junsu_v2_bak/tools/go2_ctl.py')
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import drive_ui as D
D.OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ui_runs_bak'); os.makedirs(D.OUT, exist_ok=True)
for name, seq in [('m', [('m', 0.3), ('n\r', 0.3)]), ('R', [('R', 0.3), ('\r', 0.4)]), ('Q', [('Q', 0.3), ('n\r', 0.3)]), ('l', [('l', 0.3), ('없음\r', 0.4)]),
                  ('keys', [('a', 0.3), ('r', 0.3), ('p', 0.3), ('x', 0.3), ('M', 0.3), ('1', 0.2), ('0', 0.2)])]:
    D.SEQ = [('', 0.8)] + seq + [('q', 1.0)]
    for rows, cols in [(24, 80), (45, 184)]:
        r = D.run(rows, cols, 'normal')
        print(f"bak [{name}] {cols}x{rows}: 정상종료={r['exit_ok']} 튕김={len(r['crash'])}" + (' ' + r['crash'][0].strip().splitlines()[-1] if r['crash'] else ''))
