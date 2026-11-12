#!/usr/bin/env python3
"""러너(real_course_runner.py)의 load_waypoints 를 ROS 없이 떼어내(AST) 경로 파일을 그대로 검사한다.
사용: python3 check_routes.py <경로파일>...   (러너는 import 하지 않음 — rclpy 불필요)"""
import ast, json, os, sys, math
RUNNER = os.path.expanduser(sys.argv[1]) if sys.argv[1].endswith('.py') else os.path.expanduser('~/ws/junsu_v2/src/real_course_runner.py')
files = [a for a in sys.argv[1:] if not a.endswith('.py')]
src = open(RUNNER).read(); tree = ast.parse(src)
want_assign = {'ROUTE_KINDS', 'ORDER', 'SLALOM_PATH_MAP', 'EXIT_LINE_X', 'ALIGN_T', 'ROTATE_LABELS'}
nodes = []
for n in tree.body:
    if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id in want_assign for t in n.targets):
        nodes.append(n)
    if isinstance(n, ast.FunctionDef) and n.name == 'load_waypoints':
        nodes.append(n)
mod = ast.Module(body=nodes, type_ignores=[])
for f in files:
    ns = {'os': os, 'json': json, 'math': math, '__name__': 'x'}
    exec(compile(mod, RUNNER, 'exec'), ns)
    ns['WAYPOINTS_PATH'] = f
    print(f'== {f}')
    try:
        start, goals = ns['load_waypoints']()
    except Exception as e:
        print('  ★ load_waypoints 실패:', type(e).__name__, e); continue
    labels = [g[0] for g in goals]
    print(f'  OK start={start} 목표 {len(goals)}개: {labels}')
    print(f'  SLALOM_PATH_MAP {len(ns["SLALOM_PATH_MAP"])}점  EXIT_LINE_X {ns["EXIT_LINE_X"]}  ALIGN_T {ns["ALIGN_T"]}')
    ri = labels.index('rabain') if 'rabain' in labels else None; ro = labels.index('rabaout') if 'rabaout' in labels else None
    print(f'  콘 구간 rabain {ri} < rabaout {ro}: {ri is not None and ro is not None and ri < ro}')
    sl = [l for l in labels if l.startswith('stopline')]; print(f'  정지선 {sl}  parking {"parking" in labels}  wall {[l for l in labels if l.startswith("wall")]}')
    # 인접 중복·0 거리 점
    pts = [start] + [g[1] for g in goals]; lab = ['start'] + labels
    for i in range(1, len(pts)):
        d = math.dist(pts[i - 1], pts[i])
        if d < 0.05:
            print(f'  ! {lab[i-1]}→{lab[i]} 거리 {d:.3f} m (거의 같은 점)')
    dup = {l for l in labels if labels.count(l) > 1}
    if dup: print('  ! 중복 라벨', dup)
