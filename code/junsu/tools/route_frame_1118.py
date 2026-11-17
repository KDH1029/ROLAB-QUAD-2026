#!/usr/bin/env python3
"""경로 좌표 바꾸기: 1459 지도(러너·출발 정렬) ↔ 11:18 실주행 지도(maps/rec1118, 편집용).
변환 = 그 주행 출발 정렬값 /mission/align_T(start) T=(0.0921, 0.0849, 0.1041 rad): 오늘(1118) = T⁻¹·지도(1459).
사용: python3 tools/route_frame_1118.py to1118 [입력=maps/route_map_1106.json] [출력=maps/route_v2_1118.json]
      python3 tools/route_frame_1118.py to1459 [입력=maps/route_v2_1118.json] [출력=maps/route_map_1106_user.json]"""
import json, math, os, shutil, sys, time
J = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TX, TY, TH = 0.0920693, 0.0848863, 0.1040741; c, s = math.cos(TH), math.sin(TH)
to1118 = lambda X, Y: (round(c * (X - TX) + s * (Y - TY), 3), round(-s * (X - TX) + c * (Y - TY), 3))
to1459 = lambda x, y: (round(c * x - s * y + TX, 3), round(s * x + c * y + TY, 3))
mode = sys.argv[1]
src = sys.argv[2] if len(sys.argv) > 2 else os.path.join(J, 'maps', 'route_map_1106.json' if mode == 'to1118' else 'route_v2_1118.json')
dst = sys.argv[3] if len(sys.argv) > 3 else os.path.join(J, 'maps', 'route_v2_1118.json' if mode == 'to1118' else 'route_map_1106_user.json')
f = to1118 if mode == 'to1118' else to1459
d = json.load(open(src))
for e in d['route']:
    e['x'], e['y'] = f(e['x'], e['y'])
d.pop('slalom_path', None)   # ★ 11-06 13시: 예전 junk 슬라럼 길은 콘 배치가 바뀌어 무효 — 버림(편집기 빨간 점선)
d['frame'] = ('11:18 실주행 camera_init(maps/rec1118, 편집용)' if mode == 'to1118' else '1459 지도 좌표(v2 출발 정렬·AMCL 과 같음)') + f' — {os.path.basename(src)} 에서 {time.strftime("%m-%d %H:%M")}'
if mode == 'to1118':
    d['map'] = os.path.join(J, 'maps', 'rec1118', 'map.npz')
if os.path.exists(dst):
    shutil.copy2(dst, dst.replace('.json', time.strftime('.bak_%H%M%S.json')))
json.dump(d, open(dst, 'w'), ensure_ascii=False, indent=1)
print(f'{os.path.basename(src)} → {os.path.basename(dst)} ({mode}): {len(d["route"])}점')
