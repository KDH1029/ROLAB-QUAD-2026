#!/usr/bin/env python3
"""slalom 에이전트: 모든 판(원본 sim/runs_v2/slalom_* · 패치 mirror/sim/runs_v2/slalomP_*)을 다시 요약 → results.jsonl 새로 쓰고 마크다운 표 출력."""
import glob, json, os, subprocess
H = os.path.dirname(os.path.abspath(__file__))
runs = sorted(glob.glob(os.path.expanduser('~/ws/junsu_v2/sim/runs_v2/slalom_*'))) + sorted(glob.glob(os.path.join(H, 'mirror/sim/runs_v2/slalomP_*')))
rows = []
for O in runs:
    if not os.path.exists(os.path.join(O, 'world.jsonl')):
        continue
    r = subprocess.run([os.path.expanduser('~/venv/go2/bin/python3'), os.path.join(H, 'slalom_report.py'), O, '--json'], capture_output=True, text=True)
    try:
        rows.append(json.loads(r.stdout))
    except ValueError:
        print('실패', O, r.stderr[-300:])
with open(os.path.join(H, 'results.jsonl'), 'w') as f:
    for x in rows:
        f.write(json.dumps(x, ensure_ascii=False) + '\n')
print('| 판 | 완주 | 전체 s | 슬라럼 s | 길 덮음 | 콘구간 이탈(우회) | 접촉 δ0 슬라럼 s/번 | 접촉 δ10 슬라럼 s/번 | 최대 깊이 cm(슬라럼) | 회전 rabain | 회전 rabaout | 대체 길 시작(진입 뒤·이유) | 길 끝 도착 | 6초 포기 | 되짚기(포기) | 물러남 | ego 실패 |')
print('|' + '---|' * 17)
for x in rows:
    ps = '; '.join(p.replace('콘 구간 ', '').replace('(못 알아봄)', '') for p in x['path_starts']) or '-'
    print(f"| {x['name']} | {'예' if x['done'] else '아니오'} | {x['total_s']} | {x['slalom_s']} | {x['sl_cov']} | {'예' if x['left_zone'] else '아니오'} | "
          f"{x['d0_sl'][0]}/{x['d0_sl'][1]} | {x['d10_sl'][0]}/{x['d10_sl'][1]} | {x['depth_sl_cm']} | {x['rot_rabain']} | {x['rot_rabaout']} | {ps} | {x['path_end']} | {x['path_stall']} | "
          f"{x['rejoin']}({x['rejoin_stall']}) | {x['backoff']} | {x['ego_fail']} |")
