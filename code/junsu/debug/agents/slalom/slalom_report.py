#!/usr/bin/env python3
"""slalom 에이전트: 슬라럼 구간 시험 한 판 요약(runner.log + world.jsonl). 사용: slalom_report.py <결과 폴더> [--json]
시각 맞춤: runner.log '자가진단 OK' 의 ROS 시각 = world.jsonl 에서 rs 가 처음 TO_GOAL 이 된 t (상태판 0.5초 주기라 ±0.5초).
접촉: world gap(몸 0.70×0.31 이 지도 장애물에 파고든 깊이) < −δ. 슬라럼 = '슬라럼 진입'(rabain 끝) ~ '슬라럼 탈출'(rabaout 끝) 로그 시각."""
import json, os, re, sys
import numpy as np

O = sys.argv[1]
name = os.path.basename(O.rstrip('/'))
E = [json.loads(l) for l in open(os.path.join(O, 'world.jsonl')) if l.strip()]
P = [e for e in E if e['ev'] == 'pose']
t = np.array([e['t'] for e in P]); g = np.array([e['gap'] for e in P]); rs = [e.get('rs', '') for e in P]
x = np.array([e['x'] for e in P]); y = np.array([e['y'] for e in P])
rl = open(os.path.join(O, 'runner.log')).read() if os.path.exists(os.path.join(O, 'runner.log')) else ''
el = open(os.path.join(O, 'ego.log')).read() if os.path.exists(os.path.join(O, 'ego.log')) else ''
L = []
for line in rl.splitlines():
    m = re.match(r'\[(\w+)\] \[(\d+\.\d+)\] \[real_course_runner\]: (.*)', line)
    if m:
        L.append((float(m.group(2)), m.group(1), m.group(3)))


def first(pat, after=-1e18):
    for ts, lv, msg in L:
        if ts >= after and re.search(pat, msg):
            return ts
    return None


def all_(pat):
    return [(ts, msg) for ts, lv, msg in L if re.search(pat, msg)]


ok_t = first('자가진단 OK')
w0 = next((t[i] for i, s in enumerate(rs) if s == 'TO_GOAL'), t[0])
def w(ts):          # ROS 시각 → world t
    return None if ts is None or ok_t is None else ts - ok_t + w0


done_i = next((i for i, s in enumerate(rs) if s == 'DONE'), None)
t_end = t[done_i] if done_i is not None else t[-1]
run = (t >= w0) & (t <= t_end)
zin = first(r'라바콘 슬라럼 진입'); zout = first(r'라바콘 슬라럼 탈출')
zs = w(zin) if zin else None; ze = w(zout) if zout else t_end
inz = run & (t >= zs) & (t <= ze) if zs is not None else np.zeros_like(run)
box = run & (x > 6.0) & (y > -1.1) & (y < 3.45)   # 위치로 본 콘 구간(참고)


def episodes(mask):           # 0.5초 안에 다시 닿으면 같은 번(sim_report_v2 와 같게)
    n, prev, last_t = 0, False, -9
    for ti, m in zip(t, mask):
        if m and not prev and ti - last_t > 0.5:
            n += 1
        if m:
            last_t = ti
        prev = m
    return n


def contact(mask, dl):
    m = mask & (g < -dl)
    return round(float(np.sum(m) * 0.1), 1), episodes(m)


def rot(label):
    s = first(rf'\[{label}\] 도착 -> 제자리 회전 시작')
    if s is None:
        return 'skip' if first(rf'\[{label}\] 도착 — 다음 점 방향과') else '-'
    e = first(rf'\[{label}\] 회전 (완료|타임아웃)', s)
    how = ''
    for ts, lv, msg in L:
        if ts == e:
            how = '넘음' if '목표를 넘음' in msg else ('15°안' if '안 1초' in msg else ('타임아웃' if '타임아웃' in msg else '4°안'))
    return f'{e - s:.1f}s({how})' if e else '미완'


sp = [(ts, msg) for ts, msg in all_(r'슬라럼 대체 길\(junk\) 따라감')]
sp_info = []
for ts, msg in sp:
    why = msg.split('] ', 1)[-1].split(' — ')[0]
    sp_info.append(f'+{ts - zin:.1f}s:{why}' if zin else why)
# 슬라럼 길 덮음: 경로 파일 slalom_path 점 중 로봇 궤적이 0.30 m 안으로 지나간 비율(콘 구간을 실제로 지났나 — 우회하면 낮음)
try:
    SP = np.array(json.load(open(os.path.join(O, 'route.json')))['slalom_path'], float)
    XY = np.c_[x[run], y[run]]
    cov = float(np.mean([np.min(np.hypot(*(XY - q).T)) < 0.30 for q in SP])) if len(XY) else 0.0
    far = float(np.max(np.c_[x, y][run & (t >= (zs or 0)) & (t <= ze)][:, 0] < 6.0)) if zs is not None else 0.0
except (KeyError, ValueError, OSError):
    cov, far = None, None
res = dict(
    sl_cov=round(cov, 2) if cov is not None else None, left_zone=bool(far),
    name=name,
    done=done_i is not None,
    end_state=rs[-1] if rs else '?',
    total_s=round(float(t_end - w0), 1),
    slalom_s=round(float(ze - zs), 1) if zs is not None else None,
    d0_all=contact(run, 0.0), d10_all=contact(run, 0.10),
    d0_sl=contact(inz, 0.0), d10_sl=contact(inz, 0.10),
    d0_box=contact(box, 0.0), d10_box=contact(box, 0.10),
    depth_cm=round(max(0.0, -float(g[run].min())) * 100, 0) if run.any() else None,
    depth_sl_cm=round(max(0.0, -float(g[inz].min())) * 100, 0) if np.any(inz) else None,
    rot_rabain=rot('rabain'), rot_rabaout=rot('rabaout'),
    path_starts=sp_info,
    path_end=len(all_(r'슬라럼 대체 길 끝')), path_stall=len(all_(r'슬라럼 대체 길 6초 진행 없음')),
    rejoin=len(all_(r'지나온 길 되짚어')), rejoin_stall=len(all_(r'되짚기 6초 진행 없음')),
    backoff=len(all_(r'물러남\(몸체')), giveup=len(all_(r'콘 무시 뒤에도')), follow=len(all_(r'경로 따라가기\(ego 세움')),
    cone_lv2=len(all_(r'→ 2 단계')),
    ego_fail=el.count('Failed to find'),
    rabaout_arrive=bool(first(r'\[rabaout\] 도착') or first(r'슬라럼 대체 길 끝')),
    exit_ok=bool(first('Out Line 통과')),
)
if '--json' in sys.argv:
    print(json.dumps(res, ensure_ascii=False))
else:
    print(f"[{name}] 완주 {'예' if res['done'] else '아니오(' + res['end_state'] + ')'} · 전체 {res['total_s']}s · 슬라럼(진입~탈출) {res['slalom_s']}s · "
          f"접촉 δ0 슬라럼 {res['d0_sl'][0]}s/{res['d0_sl'][1]}번 δ10 {res['d10_sl'][0]}s/{res['d10_sl'][1]}번 (전체 δ0 {res['d0_all'][0]}s δ10 {res['d10_all'][0]}s, 최대 {res['depth_cm']} cm) · "
          f"길 덮음 {res['sl_cov']} 콘구간 이탈 {'예' if res['left_zone'] else '아니오'} · 회전 rabain {res['rot_rabain']} rabaout {res['rot_rabaout']} · 대체 길 시작 {res['path_starts'] or '없음'} 끝 {res['path_end']} 6초포기 {res['path_stall']} · "
          f"되짚기 {res['rejoin']}(포기 {res['rejoin_stall']}) 물러남 {res['backoff']} 콘무시직진 {res['follow']} ego실패 {res['ego_fail']}")
    with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'results.jsonl'), 'a') as f:
        f.write(json.dumps(res, ensure_ascii=False) + '\n')
