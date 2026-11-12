#!/usr/bin/env python3
"""설정별 결과를 한 표로 — 정확도와 비용을 나란히 본다."""
import csv, glob, math, os, statistics as st
O = os.path.expanduser('~/ws/lab_visual/out')
L = os.path.expanduser('~/ws/lab_visual/logs')
print('%-22s %8s %8s %9s %9s %8s %9s' % ('설정', '잔차중앙', '잔차95%', '보정중앙', '총이동', '1회ms', '코어점유'))
for f in sorted(glob.glob(O + '/*.csv')):
    tag = os.path.basename(f)[:-4]
    r = list(csv.DictReader(open(f)))
    rms = [float(x['rms']) for x in r if x['rms']]
    cor = [float(x['corr']) for x in r if x['corr']]
    if not rms:
        print('%-20s (정합 결과 없음)' % tag); continue
    # 끝 자세가 초기에서 얼마나 밀렸나 (map->odom 기준. 로봇이 제자리로 돌아오지 않으므로 참고값)
    mx0, my0 = float(r[0]['mx']), float(r[0]['my'])
    mx1, my1 = float(r[-1]['mx']), float(r[-1]['my'])
    ms = [float(x['ms']) for x in r if x.get('ms')]
    # 점유율 = 한 번 정합에 걸린 시간 x 초당 횟수. top 이 이름을 자르는 문제를 피한다.
    n_match = len(ms)
    dur = float(r[-1]['t']) - float(r[0]['t'])
    hz = n_match / dur if dur > 0 else 0
    duty = (st.median(ms) * hz / 1000.0) if ms else float('nan')
    print('%-22s %8.3f %8.3f %9.3f %9.3f %8.0f %8.0f%%'
          % (tag, st.median(rms), sorted(rms)[int(len(rms) * 0.95)],
             st.median(cor) if cor else float('nan'),
             math.hypot(mx1 - mx0, my1 - my0),
             st.median(ms) if ms else float('nan'), duty * 100))
print()
print('  잔차: 스캔이 지도에서 떨어진 거리. 낮을수록 좋다')
print('  보정중앙: 매초 map->odom 이 움직인 양. 작을수록 안정')
print('  1회ms: 정합 한 번에 걸린 시간.  코어점유: 그것 x 초당 횟수 = 한 코어의 몇 %%')
print('  젯슨은 노트북의 2~3배로 잡는다. 비전(GPU)까지 올리려면 여기서 60%% 이하가 안전하다')
