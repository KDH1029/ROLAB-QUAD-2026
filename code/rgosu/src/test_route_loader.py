#!/usr/bin/env python3
"""real_course_runner.load_waypoints — 지도에서 정한 경로(route_v1, 파일 순서 그대로) + 예전 형식(고정 ORDER) 둘 다 읽는지."""
import json, os, sys, tempfile
sys.argv = [sys.argv[0], '/dev/null']
import real_course_runner as R

d = tempfile.mkdtemp()


def load(obj):
    p = os.path.join(d, 'w.json'); json.dump(obj, open(p, 'w')); R.WAYPOINTS_PATH = p; return R.load_waypoints()


route = {'format': 'route_v1', 'route': [
    {'label': 'start', 'x': 0.0, 'y': 0.1}, {'label': 'top1', 'x': 2.0, 'y': 1.2}, {'label': 'stopline1', 'x': 3.5, 'y': 1.9},
    {'label': 'line1', 'x': 5.0, 'y': 3.0}, {'label': 'rabain', 'x': 6.3, 'y': 4.4}, {'label': 'rabaout', 'x': 6.6, 'y': 0.3},
    {'label': 'stopline2', 'x': 3.5, 'y': 1.9}, {'label': 'line2', 'x': 1.0, 'y': 4.0}, {'label': 'parking', 'x': -0.3, 'y': 1.9},
    {'label': 'line3', 'x': -0.1, 'y': 3.9}, {'label': 'exit', 'x': -1.2, 'y': 4.2}]}
s, g = load(route)
assert s == (0.0, 0.1) and [l for l, _ in g] == ['top1', 'stopline1', 'line1', 'rabain', 'rabaout', 'stopline2', 'line2', 'parking', 'line3', 'exit'], g
print('1) route_v1: 파일 순서 그대로(top·line 경유점 포함) OK')
for bad, why in ((dict(route=route['route'][1:]), "첫 점"), (dict(route=route['route'][:-1]), "마지막 점"),
                 (dict(route=route['route'][:3] + [{'label': 'parkout', 'x': 0, 'y': 0}] + route['route'][3:]), '모르는 라벨')):
    try:
        load(bad); raise AssertionError(f'{why} 오류를 안 냄')
    except RuntimeError as e:
        assert why.split()[0] in str(e) or '모르는' in str(e), e
print("2) start 없음 · exit 없음 · 모르는 라벨(parkout) → 거부 OK")
old = json.load(open(os.path.expanduser('~/ws/Rgosu/maps/real_waypoints_venue_run.json')))
s, g = load(old); assert [l for l, _ in g] == R.ORDER, g
print('3) 예전 형식(로봇으로 저장한 목록) → 고정 ORDER 그대로 OK')
if os.path.exists(os.path.expanduser('~/ws/Rgosu/maps/route_map.json')):
    R.WAYPOINTS_PATH = os.path.expanduser('~/ws/Rgosu/maps/route_map.json'); s, g = R.load_waypoints(); print('4) maps/route_map.json 읽기 OK:', [l for l, _ in g])
print('\n전체 PASS')
