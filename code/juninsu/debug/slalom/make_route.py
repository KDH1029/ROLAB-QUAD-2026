#!/usr/bin/env python3
"""슬라럼 디버깅 경로: routes/<이름>/route.json 에서 #A(출발, 로봇 놓는 자리) → #B+1 … #C-1 → #C(끝).
사용: python3 make_route.py <이름> <출력 json> [A=6] [C=25]   → 마지막 줄에 출발 정렬 prior 'x,y,yaw도' 를 찍는다.
점 번호는 편집기에 보이는 번호(1부터). 경로를 편집기로 고치면 다음 실행 때 그대로 반영된다."""
import json, math, sys

name, out = sys.argv[1], sys.argv[2]
A = int(sys.argv[3]) if len(sys.argv) > 3 else 6
C = int(sys.argv[4]) if len(sys.argv) > 4 else 25
src = f'{__import__("os").path.dirname(__import__("os").path.dirname(__import__("os").path.dirname(__import__("os").path.abspath(__file__))))}/routes/{name}/route.json'
d = json.load(open(src)); r = d['route']
seg = r[A - 1:C]
a, b = seg[0], seg[1]
yaw = math.degrees(math.atan2(b['y'] - a['y'], b['x'] - a['x']))     # 놓을 때 바라볼 쪽 = 다음 점
route = [dict(label='start', x=a['x'], y=a['y'])] + [dict(label=f'line{i + 1}', x=e['x'], y=e['y']) for i, e in enumerate(seg[1:-1])] \
        + [dict(label='exit', x=seg[-1]['x'], y=seg[-1]['y'])]
for i, e in enumerate(route):
    e['no'] = A + i                                                   # 원래 경로 번호(그림·로그용)
json.dump({'format': 'route_v1', 'source': src, 'slalom': [A, C], 'route': route}, open(out, 'w'), ensure_ascii=False, indent=1)
print(f'슬라럼 경로 #{A}(출발)→#{C}(끝) {len(route)}점 · 로봇을 #{A} ({a["x"]:.2f}, {a["y"]:.2f}) 에 #{A + 1} 쪽({yaw:.0f}°)을 보게 놓을 것', file=sys.stderr)
print(f'{a["x"]:.3f},{a["y"]:.3f},{yaw:.1f}')
