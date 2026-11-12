#!/usr/bin/env python3
"""1650 지도에서 고친 경로(maps/route_v2_1650.json) → v2 가 쓰는 1459 지도 좌표(maps/route_map_1106.json).
v2 의 출발 정렬·AMCL 지도는 1459(콘 없는 지도)라 러너 경로도 그 좌표여야 한다. 변환 = maps/T_1459_from_1650.json(3D 지도 정합, 일치 0.65).
경로 점과 slalom_path(슬라럼 대체 길)를 같이 옮긴다. 사용: python3 tools/route_1650_to_1459.py [입력=maps/route_v2_1650.json] [출력=maps/route_map_1106.json]"""
import json, math, os, shutil, sys, time
J = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
src = sys.argv[1] if len(sys.argv) > 1 else os.path.join(J, 'maps', 'route_v2_1650.json')
dst = sys.argv[2] if len(sys.argv) > 2 else os.path.join(J, 'maps', 'route_map_1106.json')
T = json.load(open(os.path.join(J, 'maps', 'T_1459_from_1650.json')))['T_1459_from_1650']; c, s = math.cos(T[2]), math.sin(T[2])
fw = lambda x, y: (round(c * x - s * y + T[0], 3), round(s * x + c * y + T[1], 3))
d = json.load(open(src))
for e in d['route']:
    e['x'], e['y'] = fw(e['x'], e['y']); e.pop('yaw', None)
if d.get('slalom_path'):
    d['slalom_path'] = [list(fw(x, y)) for x, y in d['slalom_path']]
d['frame'] = f'1459 지도 좌표(v2 출발 정렬·AMCL 과 같음). 원본 {os.path.basename(src)}(1650 좌표) → T_1459_from_1650, {time.strftime("%m-%d %H:%M")}'
d.pop('out_line_x_note', None)
if os.path.exists(dst):
    shutil.copy2(dst, dst.replace('.json', time.strftime('.bak_%H%M%S.json')))
json.dump(d, open(dst, 'w'), ensure_ascii=False, indent=1)
labs = [e['label'] for e in d['route']]
print(f'{os.path.basename(src)} → {os.path.basename(dst)}: {len(labs)}점 {labs[0]}…{labs[-1]} · 정지선 {sum(l.startswith("stopline") for l in labs)} · 슬라럼 대체 길 {len(d.get("slalom_path", []))}점')
